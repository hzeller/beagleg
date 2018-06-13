/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2016 Henner Zeller <h.zeller@acm.org>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdlib.h>

#include <cmath>  // We use these functions as they work type-agnostic

#include "common/logging.h"
#include "common/container.h"

#include "planner.h"
#include "hardware-mapping.h"
#include "gcode-machine-control.h"
#include "motor-operations.h"

namespace {
// The target position vector is essentially a position in the
// GCODE_NUM_AXES-dimensional space.
//
// An AxisTarget has a position vector, in absolute machine coordinates, and a
// speed when arriving at that position.
//
// The speed is initially the aimed goal; if it cannot be reached, the
// AxisTarget will be modified to contain the actually reachable value. That is
// used in planning along the path.
struct AxisTarget {
  int position_steps[GCODE_NUM_AXES];  // Absolute position at end of segment. In steps.

  // Derived values
  int delta_steps[GCODE_NUM_AXES];     // Difference to previous position.
  enum GCodeParserAxis defining_axis;  // index into defining axis.
  double speed;                         // (desired) speed in steps/s on defining axis.
  unsigned short aux_bits;             // Auxillary bits in this segment; set with M42
  double dx, dy, dz;                    // 3D delta_steps in real units
  double len;                           // 3D length
};
}  // end anonymous namespace

class Planner::Impl {
public:
  Impl(const MachineControlConfig *config,
       HardwareMapping *hardware_mapping,
       MotorOperations *motor_backend);
  ~Impl();

  void move_machine_steps(const struct AxisTarget *last_pos,
                          struct AxisTarget *target_pos,
                          const struct AxisTarget *upcoming);

  void assign_steps_to_motors(struct LinearSegmentSteps *command,
                              enum GCodeParserAxis axis,
                              int steps);

  void issue_motor_move_if_possible();
  void machine_move(const AxesRegister &axis, float feedrate);
  void bring_path_to_halt();

  float acceleration_for_move(const int *axis_steps,
                              enum GCodeParserAxis defining_axis) {
    return max_axis_accel_[defining_axis];
    // TODO: we need to scale the acceleration if one of the other axes could't
    // deal with it. Look at axis steps for that.
  }

  // Avoid division by zero if there is no config defined for axis.
  double axis_delta_to_mm(const AxisTarget *pos, enum GCodeParserAxis axis) {
    if (cfg_->steps_per_mm[axis] != 0.0)
      return (double)pos->delta_steps[axis] / cfg_->steps_per_mm[axis];
    return 0.0;
  }

  double euclidian_speed(const struct AxisTarget *t);

  void GetCurrentPosition(AxesRegister *pos);
  int DirectDrive(GCodeParserAxis axis, float distance, float v0, float v1);
  void SetExternalPosition(GCodeParserAxis axis, float pos);

  // Given the desired target speed of the defining axis and the steps to be
  // performed on all axes, determine if we need to scale down as to not exceed
  // the individual maximum speed constraints on any axis. Return the new speed
  // of the defining axis.
  double clamp_to_limits(enum GCodeParserAxis defining_axis,
                         const double target_value,
                         const int *axis_steps);

private:
  const struct MachineControlConfig *const cfg_;
  HardwareMapping *const hardware_mapping_;
  MotorOperations *const motor_ops_;

  // Next buffered positions. Written by incoming gcode, read by outgoing
  // motor movements.
  RingDeque<AxisTarget, 4> planning_buffer_;

  // Pre-calculated per axis limits in steps, steps/s, steps/s^2
  // All arrays are indexed by axis.
  AxesRegister max_axis_speed_;   // max travel speed hz
  AxesRegister max_axis_accel_;   // acceleration hz/s
  float highest_accel_;           // hightest accel of all axes.

  HardwareMapping::AuxBitmap last_aux_bits_;  // last enqueued aux bits.

  bool path_halted_;
  bool position_known_;
};

// Speed relative to defining axis
static double get_speed_factor_for_axis(const struct AxisTarget *t,
                                        enum GCodeParserAxis request_axis) {
  if (t->delta_steps[t->defining_axis] == 0) return 0.0;
  return 1.0 * t->delta_steps[request_axis] / t->delta_steps[t->defining_axis];
}

// Get the speed for a particular axis. Depending on the direction, this can
// be positive or negative.
static double get_speed_for_axis(const struct AxisTarget *target,
                                 enum GCodeParserAxis request_axis) {
  return target->speed * get_speed_factor_for_axis(target, request_axis);
}

// Given that we want to travel "s" steps, start with speed "v0",
// accelerate peak speed v1 and slow down to "v2" with acceleration "a",
// what is v1 ?
static double get_peak_speed(int s, double v0, double v2, double a) {
  return std::sqrt(v2*v2 + v0*v0 + 2 * a * s) / std::sqrt(2.0);
}

static double euclid_distance(double x, double y, double z) {
  return std::sqrt(x*x + y*y + z*z);
}

// Number of steps to accelerate or decelerate (negative "a") from speed
// v0 to speed v1. Modifies v1 if we can't reach the speed with the allocated
// number of steps.
static double steps_for_speed_change(double a, double v0, double *v1,
                                     int max_steps) {
  // s = v0 * t + a/2 * t^2
  // v1 = v0 + a*t
  const double t = (*v1 - v0) / a;
  // TODO:
  if (t < 0) Log_error("Error condition: t=%.1f INSUFFICIENT LOOKAHEAD\n", t);
  double steps = a/2 * t*t + v0 * t;
  if (steps <= max_steps) return steps;
  // Ok, we would need more steps than we have available. We correct the speed to what
  // we actually can reach.
  *v1 = std::sqrt(v0*v0 + 2 * a * max_steps);
  return max_steps;
}

// Returns true, if all results in zero movement
static bool subtract_steps(struct LinearSegmentSteps *value,
                           const struct LinearSegmentSteps &subtract) {
  bool has_nonzero = false;
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    value->steps[i] -= subtract.steps[i];
    has_nonzero |= (value->steps[i] != 0);
  }
  return has_nonzero;
}

static bool within_acceptable_range(double new_val, double old_val,
                                    double fraction) {
  const double max_diff = fraction * old_val;
  if (new_val < old_val - max_diff) return false;
  if (new_val > old_val + max_diff) return false;
  return true;
}

// Determine the fraction of the speed that "from" should decelerate
// to at the end of its travel.
// The way trapezoidal moves work, be still have to decelerate to zero in
// most times, which is inconvenient. TODO(hzeller): speed matching is not
// cutting it :)
static double determine_joining_speed(const struct AxisTarget *from,
                                      const struct AxisTarget *to,
                                      const double threshold,
                                      const double speed_tune_angle) {
  // the dot product of the vectors
  const double dot = from->dx*to->dx + from->dy*to->dy + from->dz*to->dz;
  const double mag = from->len * to->len;
  if (dot == 0) return 0.0;         // orthogonal 90 degree, full stop
  if (dot < 0) return 0.0;          // turning around, full stop
  const double dotmag = dot / mag;
  if (within_acceptable_range(1.0, dotmag, 1e-5))
    return to->speed;               // codirectional 0 degree, keep accelerating

  // the angle between the vectors
  const double rad2deg = 180.0 / M_PI;
  const double angle = std::fabs(std::acos(dotmag) * rad2deg);

  if (angle >= 45.0)
    return 0.0;                     // angle to large, come to full stop
  if (angle <= threshold) {         // in tolerance, keep accelerating
    if (dot < 1) {                  // speed tune segments less than 1mm (i.e. arcs)
      const double deg2rad = M_PI / 180.0;
      const double angle_speed_adj = std::cos((angle + speed_tune_angle) * deg2rad);
      return to->speed * angle_speed_adj;
    }
    return to->speed;
  }

  // The angle between the from and to segments is < 45 degrees but greater
  // than the threshold. Use the "old" logic to determine the joining speed
  //
  // Our goal is to figure out what our from defining speed should
  // be at the end of the move.
  bool is_first = true;
  double from_defining_speed = from->speed;
  const int from_defining_steps = from->delta_steps[from->defining_axis];
  for (const GCodeParserAxis axis : AllAxes()) {
    const int from_delta = from->delta_steps[axis];
    const int to_delta = to->delta_steps[axis];

    // Quick integer decisions
    if (from_delta == 0 && to_delta == 0) continue;   // uninteresting: no move.
    if (from_delta == 0 || to_delta == 0) return 0.0; // accel from/to zero
    if ((from_delta < 0 && to_delta > 0) || (from_delta > 0 && to_delta < 0))
      return 0.0;  // turing around

    double to_speed = get_speed_for_axis(to, axis);
    // What would this speed translated to our defining axis be ?
    double speed_conversion = 1.0 * from_defining_steps / from_delta;
    double goal = to_speed * speed_conversion;
    if (goal < 0.0) return 0.0;
    if (is_first || within_acceptable_range(goal, from_defining_speed, 1e-5)) {
      if (goal < from_defining_speed) from_defining_speed = goal;
      is_first = false;
    } else {
      return 0.0;  // Too far off.
    }
  }

  return from_defining_speed;
}

Planner::Impl::Impl(const MachineControlConfig *config,
                    HardwareMapping *hardware_mapping,
                    MotorOperations *motor_backend)
  : cfg_(config), hardware_mapping_(hardware_mapping),
    motor_ops_(motor_backend),
    highest_accel_(-1), path_halted_(true), position_known_(true) {
  // Initial machine position. We assume the homed position here, which is
  // wherever the endswitch is for each axis.
  struct AxisTarget *init_axis = planning_buffer_.append();
  bzero(init_axis, sizeof(*init_axis));
  for (const GCodeParserAxis axis : AllAxes()) {
    HardwareMapping::AxisTrigger trigger = cfg_->homing_trigger[axis];
    const float home_pos = trigger == HardwareMapping::TRIGGER_MAX
        ? cfg_->move_range_mm[axis] : 0;
    SetExternalPosition(axis, home_pos);
  }
  position_known_ = true;

  float lowest_accel = cfg_->max_feedrate[AXIS_X] * cfg_->steps_per_mm[AXIS_X];
  for (const GCodeParserAxis i : AllAxes()) {
    max_axis_speed_[i] = cfg_->max_feedrate[i] * cfg_->steps_per_mm[i];
    const float accel = cfg_->acceleration[i] * cfg_->steps_per_mm[i];
    max_axis_accel_[i] = accel;
    if (accel > highest_accel_)
      highest_accel_ = accel;
    if (accel < lowest_accel)
      lowest_accel = accel;
  }
}

Planner::Impl::~Impl() {
  bring_path_to_halt();
}

// Assign steps to all the motors responsible for given axis.
void Planner::Impl::assign_steps_to_motors(struct LinearSegmentSteps *command,
                                           enum GCodeParserAxis axis,
                                           int steps) {
  hardware_mapping_->AssignMotorSteps(axis, steps, command);
}

// Example with speed: given i the axis with the highest (relative) out of bounds
// speed, what's the speed of the defining_axis so that every speed respects i's
// bounds? The defining axis should be rescaled with this maximum offset.
// offset = speed_limit[i] / speed[i]
double Planner::Impl::clamp_to_limits(enum GCodeParserAxis defining_axis,
                                      const double target_speed,
                                      const int *axis_steps) {
  double ratio, max_offset = 1, offset;
  const FloatAxisConfig &max_axis_speed = cfg_->max_feedrate;
  const FloatAxisConfig &steps_per_mm = cfg_->steps_per_mm;
  for (const GCodeParserAxis i : AllAxes()) {
    ratio = std::fabs((1.0 * axis_steps[i] * steps_per_mm[defining_axis])
                      / (axis_steps[defining_axis] * steps_per_mm[i]));
    offset = ratio > 0 ? max_axis_speed[i] / (target_speed * ratio) : 1;
    if (offset < max_offset) max_offset = offset;
  }
  return target_speed * max_offset;
}

double Planner::Impl::euclidian_speed(const struct AxisTarget *t) {
  double speed_factor = 1.0;
  if (t->len > 0) {
    const double axis_len_mm = axis_delta_to_mm(t, t->defining_axis);
    speed_factor = std::fabs(axis_len_mm) / t->len;
  }
  return t->speed * speed_factor;
}

// Move the given number of machine steps for each axis.
//
// This will be up to three segments: accelerating from last_pos speed to
// target speed, regular travel, and decelerating to the speed that the
// next segment is never forced to decelerate, but stays at speed or accelerate.
//
// The segments are sent to the motor operations backend.
//
// Since we calculate the deceleration, this modifies the speed of target_pos
// to reflect what the last speed was at the end of the move.
void Planner::Impl::move_machine_steps(const struct AxisTarget *last_pos,
                                       struct AxisTarget *target_pos,
                                       const struct AxisTarget *upcoming) {
  if (target_pos->delta_steps[target_pos->defining_axis] == 0) {
    if (last_aux_bits_ != target_pos->aux_bits) {
      // Special treatment: bits changed since last time, let's push them through.
      struct LinearSegmentSteps bit_set_command = {};
      bit_set_command.aux_bits = target_pos->aux_bits;
      motor_ops_->Enqueue(bit_set_command);
      last_aux_bits_ = target_pos->aux_bits;
    }
    return;
  }
  struct LinearSegmentSteps accel_command = {};
  struct LinearSegmentSteps move_command = {};
  struct LinearSegmentSteps decel_command = {};

  assert(target_pos->speed > 0);  // Speed is always a positive scalar.

  // Aux bits are set synchronously with what we need.
  move_command.aux_bits = target_pos->aux_bits;
  const enum GCodeParserAxis defining_axis = target_pos->defining_axis;

  // Common settings.
  memcpy(&accel_command, &move_command, sizeof(accel_command));
  memcpy(&decel_command, &move_command, sizeof(decel_command));

  // Always start from the last steps/sec speed to avoid motion glitches.
  // The planner will use that speed to determine what the peak speed for
  // this move is and if we need to accel to reach the desired target speed.
  const double last_speed = last_pos->speed;

  // We need to arrive at a speed that the upcoming move does not have
  // to decelerate further (after all, it has a fixed feed-rate it should not
  // go over).
  double next_speed = determine_joining_speed(target_pos, upcoming,
                                             cfg_->threshold_angle,
                                             cfg_->speed_tune_angle);
  // Clamp the next speed to insure that this segment does not go over.
  if (next_speed > target_pos->speed)
    next_speed = target_pos->speed;

  const int *axis_steps = target_pos->delta_steps;  // shortcut.
  const int abs_defining_axis_steps = abs(axis_steps[defining_axis]);
  const double a = acceleration_for_move(axis_steps, defining_axis);
  const double peak_speed = get_peak_speed(abs_defining_axis_steps,
                                          last_speed, next_speed, a);
  assert(peak_speed > 0);

  // TODO: if we only have < 5 steps or so, we should not even consider
  // accelerating or decelerating, but just do one speed.

  if (peak_speed < target_pos->speed) {
    target_pos->speed = peak_speed;  // Don't manage to accelerate to desired v
  }

  // Make sure the target feedrate for the move is clamped to what all the
  // moving axes can reach.
  double target_feedrate = target_pos->speed / cfg_->steps_per_mm[defining_axis];
  target_feedrate = clamp_to_limits(defining_axis, target_feedrate, axis_steps);
  target_pos->speed = target_feedrate * cfg_->steps_per_mm[defining_axis];

  const double accel_fraction =
    (last_speed < target_pos->speed)
    ? steps_for_speed_change(a, last_speed, &target_pos->speed,
                             abs_defining_axis_steps) / abs_defining_axis_steps
    : 0;

  // We only decelerate if the upcoming speed is _slower_
  double dummy_next_speed = next_speed;  // Don't care to modify; we don't have
  const double decel_fraction =
    (next_speed < target_pos->speed)
    ? steps_for_speed_change(-a, target_pos->speed, &dummy_next_speed,
                             abs_defining_axis_steps) / abs_defining_axis_steps
    : 0;

#if 0
  // Useful debugging info.
  // TODO: we get a decel glitch when the last_pos has accelerated to a speed
  // that is faster than the target can decelerate from.
  if (dummy_next_speed != next_speed) {
    fprintf(stderr, "  \033[1m\033[31mGLITCH\033[0m "
                    "- move too short for full decel (reached v1: %10.2f)\n",
            dummy_next_speed);
  }
#endif

  assert(accel_fraction + decel_fraction <= 1.0 + 1e-3);

#if 0
  // fudging: if we have tiny acceleration segments, don't do these at all
  // but only do speed; otherwise we have a lot of rattling due to many little
  // segments of acceleration/deceleration (e.g. for G2/G3).
  // This is not optimal. Ideally, we would actually calculate in terms of
  // jerk and optimize to stay within that constraint.
  const int accel_decel_steps
    = (accel_fraction + decel_fraction) * abs_defining_axis_steps;
  const double accel_decel_mm
    = (accel_decel_steps / cfg_->steps_per_mm[defining_axis]);
  const char do_accel = (accel_decel_mm > 2 || accel_decel_steps > 16);
#else
  const char do_accel = 1;
#endif

  bool has_accel = false;
  bool has_move = false;
  bool has_decel = false;

  if (do_accel && accel_fraction * abs_defining_axis_steps > 0) {
    has_accel = true;
    accel_command.v0 = (float)last_speed;        // Last speed of defining axis
    accel_command.v1 = (float)target_pos->speed; // New speed of defining axis

    // Now map axis steps to actual motor driver
    for (const GCodeParserAxis a : AllAxes()) {
      const int accel_steps = std::lround(accel_fraction * axis_steps[a]);
      assign_steps_to_motors(&accel_command, a, accel_steps);
    }
  } else {
    if (last_speed) target_pos->speed = last_speed; // No accel so use the last speed
  }

  move_command.v0 = (float)target_pos->speed;
  move_command.v1 = (float)target_pos->speed;

  if (do_accel && decel_fraction * abs_defining_axis_steps > 0) {
    has_decel = true;
    decel_command.v0 = (float)target_pos->speed;
    decel_command.v1 = (float)next_speed;
    target_pos->speed = next_speed;

    // Now map axis steps to actual motor driver
    for (const GCodeParserAxis a : AllAxes()) {
      const int decel_steps = std::lround(decel_fraction * axis_steps[a]);
      assign_steps_to_motors(&decel_command, a, decel_steps);
    }
  }

  // Move is everything that hasn't been covered in speed changes.
  // So we start with all steps and subtract steps done in acceleration and
  // deceleration.
  for (const GCodeParserAxis a : AllAxes()) {
    assign_steps_to_motors(&move_command, a, axis_steps[a]);
  }
  subtract_steps(&move_command, accel_command);
  has_move = subtract_steps(&move_command, decel_command);

  if (cfg_->synchronous) motor_ops_->WaitQueueEmpty();

  if (has_accel) motor_ops_->Enqueue(accel_command);
  if (has_move) motor_ops_->Enqueue(move_command);
  if (has_decel) motor_ops_->Enqueue(decel_command);

  last_aux_bits_ = target_pos->aux_bits;
}

// If we have enough data in the queue, issue motor move.
void Planner::Impl::issue_motor_move_if_possible() {
  if (planning_buffer_.size() >= 3) {
    move_machine_steps(planning_buffer_[0],  // Current established position.
                       planning_buffer_[1],  // Position we want to move to.
                       planning_buffer_[2]); // Next upcoming.
    planning_buffer_.pop_front();
  }
}

void Planner::Impl::machine_move(const AxesRegister &axis, float feedrate) {
  assert(position_known_);   // call SetExternalPosition() after DirectDrive()
  // We always have a previous position.
  struct AxisTarget *previous = planning_buffer_.back();
  struct AxisTarget *new_pos = planning_buffer_.append();
  int max_steps = -1;
  enum GCodeParserAxis defining_axis = AXIS_X;

  // Real world -> machine coordinates. Here, we are rounding to the next full
  // step, but we never accumulate the error, as we always use the absolute
  // position as reference.
  for (const GCodeParserAxis a : AllAxes()) {
    new_pos->position_steps[a] = std::lround(axis[a] * cfg_->steps_per_mm[a]);
    new_pos->delta_steps[a] = new_pos->position_steps[a] - previous->position_steps[a];

    // The defining axis is the one that has to travel the most steps. It defines
    // the frequency to go.
    // All the other axes are doing a fraction of the defining axis.
    if (abs(new_pos->delta_steps[a]) > max_steps) {
      max_steps = abs(new_pos->delta_steps[a]);
      defining_axis = a;
    }
  }

  if (max_steps == 0) {
    // Nothing to do, ignore this move.
    planning_buffer_.pop_back();
    return;
  }

  assert(max_steps > 0);

  new_pos->aux_bits = hardware_mapping_->GetAuxBits();
  new_pos->defining_axis = defining_axis;

  // Work out the real units values for the euclidian axes now to avoid
  // having to replicate the calcs later.
  new_pos->dx = axis_delta_to_mm(new_pos, AXIS_X);
  new_pos->dy = axis_delta_to_mm(new_pos, AXIS_Y);
  new_pos->dz = axis_delta_to_mm(new_pos, AXIS_Z);
  new_pos->len = euclid_distance(new_pos->dx, new_pos->dy, new_pos->dz);

  // Work out the desired euclidian travel speed in steps/s on the defining axis.
  new_pos->speed = feedrate * cfg_->steps_per_mm[defining_axis];
  new_pos->speed = euclidian_speed(new_pos);
  if (new_pos->speed > max_axis_speed_[defining_axis])
    new_pos->speed = max_axis_speed_[defining_axis];

  issue_motor_move_if_possible();
  path_halted_ = false;
}

void Planner::Impl::bring_path_to_halt() {
  if (path_halted_) return;
  // Enqueue a new position that is the same position as the last
  // one seen, but zero speed. That will allow the previous segment to
  // slow down. Enqueue.
  struct AxisTarget *previous = planning_buffer_.back();
  struct AxisTarget *new_pos = planning_buffer_.append();
  for (const GCodeParserAxis a : AllAxes()) {
    new_pos->position_steps[a] = previous->position_steps[a];
    new_pos->delta_steps[a] = 0;
  }
  new_pos->defining_axis = AXIS_X;
  new_pos->speed = 0;
  new_pos->aux_bits = hardware_mapping_->GetAuxBits();
  new_pos->dx = new_pos->dy = new_pos->dz = new_pos->len = 0.0;
  issue_motor_move_if_possible();
  path_halted_ = true;
}

void Planner::Impl::GetCurrentPosition(AxesRegister *pos) {
  pos->zero();
  PhysicalStatus physical_status;
  if (!motor_ops_->GetPhysicalStatus(&physical_status))
    return;   // Should we return boolean to indicate that not supported ?
  for (const GCodeParserAxis a : AllAxes()) {
#if M114_DEBUG
    Log_debug("Machine steps Axis %c : %8d\n", gcodep_axis2letter(a),
              hardware_mapping_->GetAxisSteps(a, physical_status));
#endif
    if (cfg_->steps_per_mm[a] != 0) {
      (*pos)[a] = hardware_mapping_->GetAxisSteps(a, physical_status) / cfg_->steps_per_mm[a];
    }
  }
}

int Planner::Impl::DirectDrive(GCodeParserAxis axis, float distance,
                               float v0, float v1) {
  bring_path_to_halt();     // Precondition. Let's just do it for good measure.
  position_known_ = false;

  const float steps_per_mm = cfg_->steps_per_mm[axis];

  struct LinearSegmentSteps move_command = {};

  move_command.v0 = v0 * steps_per_mm;
  if (move_command.v0 > max_axis_speed_[axis])
    move_command.v0 = max_axis_speed_[axis];
  move_command.v1 = v1 * steps_per_mm;
  if (move_command.v1 > max_axis_speed_[axis])
    move_command.v1 = max_axis_speed_[axis];

  move_command.aux_bits = hardware_mapping_->GetAuxBits();

  const int segment_move_steps = std::lround(distance * steps_per_mm);
  assign_steps_to_motors(&move_command, axis, segment_move_steps);

  motor_ops_->Enqueue(move_command);
  motor_ops_->WaitQueueEmpty();

  return segment_move_steps;
}

void Planner::Impl::SetExternalPosition(GCodeParserAxis axis, float pos) {
  assert(path_halted_);   // Precondition.
  position_known_ = true;

  const int motor_position = std::lround(pos * cfg_->steps_per_mm[axis]);
  planning_buffer_.back()->position_steps[axis] = motor_position;
  planning_buffer_[0]->position_steps[axis] = motor_position;

  const uint8_t motormap_for_axis = hardware_mapping_->GetMotorMap(axis);
  for (int motor = 0; motor < BEAGLEG_NUM_MOTORS; ++motor) {
    if (motormap_for_axis & (1 << motor))
      motor_ops_->SetExternalPosition(motor, motor_position);
  }
}

// -- public interface

Planner::Planner(const MachineControlConfig *config,
                 HardwareMapping *hardware_mapping,
                 MotorOperations *motor_backend)
  : impl_(new Impl(config, hardware_mapping, motor_backend)) {
}

Planner::~Planner() { delete impl_; }

void Planner::Enqueue(const AxesRegister &target_pos, float speed) {
  impl_->machine_move(target_pos, speed);
}

void Planner::BringPathToHalt() {
  impl_->bring_path_to_halt();
}

void Planner::GetCurrentPosition(AxesRegister *pos) {
  impl_->GetCurrentPosition(pos);
}

int Planner::DirectDrive(GCodeParserAxis axis, float distance,
                          float v0, float v1) {
  return impl_->DirectDrive(axis, distance, v0, v1);
}

void Planner::SetExternalPosition(GCodeParserAxis axis, float pos) {
  impl_->SetExternalPosition(axis, pos);
}

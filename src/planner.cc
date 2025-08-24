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
#include "planner.h"

#include <math.h>

#include <cmath>  // We use these functions as they work type-agnostic
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include "common/container.h"
#include "common/logging.h"
#include "gcode-machine-control.h"
#include "gcode-parser/gcode-parser.h"
#include "hardware-mapping.h"
#include "segment-queue.h"

#define PLANNING_BUFFER_CAPACITY 1024

using StepsAxesRegister = FixedArray<int, GCODE_NUM_AXES, GCodeParserAxis>;

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
// The starting_speed is the required speed at the start of the segment.
// It is used to combine segments with different trajectory angles.
// In the AxisTarget we define "motion invariants" (or targets). Values and
// parameters that assuming the requested final trajectory stays the same, they
// are constants, no matter the previous or next segments.
struct AxisTarget {
  StepsAxesRegister position_steps;  // Absolute position at end of segment.
                                     // In steps. (steps)

  // Derived values
  StepsAxesRegister delta_steps;  // Difference to previous position. (steps)
  enum GCodeParserAxis defining_axis;  // index into defining axis.
  double start_speed;  // Starting speed of the segment (steps/s).
  double speed;        // Maximum speed of the defining axis (steps/s).
  double accel;        // Maximum acceleration of the defining axis (steps/s^2).
  uint16_t aux_bits;   // Auxillary bits in this segment; set with M42
  double dx, dy, dz;   // 3D delta_steps in real units (mm)
  double len;          // 3D length (mm)

  std::string ToJsonString() const {
    std::ostringstream ss;
    ss << "{";
    ss << "\"start_speed\":" << start_speed << ",";
    ss << "\"speed\":" << speed << ",";
    ss << "\"accel\":" << accel << ",";
    ss << "\"defining_axis\":" << defining_axis << ",";
    ss << "\"delta\":" << "[";
    bool first = true;
    for (const GCodeParserAxis a : AllAxes()) {
      if (first) {
        first = false;
      } else {
        ss << ",";
      }
      ss << delta_steps[a];
    }
    ss << "],";
    ss << "\"pos\":" << "[";
    first = true;
    for (const GCodeParserAxis a : AllAxes()) {
      if (first) {
        first = false;
      } else {
        ss << ",";
      }
      ss << position_steps[a];
    }
    ss << "]}";
    return ss.str();
  }
};

/*
** Define values that might change depending on new segments added to
** the planning buffer.
** Currently planned profile for an AxisTarget.
**    |  v1
**    |   /¯¯¯¯¯¯¯¯¯¯\
**    |  /            \ v2
**    | /
** v0 |/________________
**      acc  travel   dec
*/
struct PlannedProfile {
  uint32_t accel, travel, decel;  // Steps of the defining axis for each ramp
  double v0, v1, v2;              // Speed(steps/s) of the defining axis,
                                  // v0-v1 accel, v1-v2 travel, v2-v3 decel.

  // Return the total number of steps in the profile.
  uint32_t TotalSteps() const { return accel + travel + decel; }

  std::string ToJsonString() const {
    std::ostringstream ss;
    ss << "{";
    ss << "\"accel\":" << accel << ",";
    ss << "\"travel\":" << travel << ",";
    ss << "\"decel\":" << decel << ",";
    ss << "\"v0\":" << v0 << ",";
    ss << "\"v1\":" << v1 << ",";
    ss << "\"v2\":" << v2;
    ss << "}";
    return ss.str();
  }
};
}  // end anonymous namespace

// Given the desired target speed(or acceleration) of the defining axis and the
// steps to be performed on all axes, determine if we need to scale down as to
// not exceed the individual maximum speed or acceleration constraints on any
// axis. Return the new defining axis limit based on the euclidean motion
// requested.
//
// Example with speed: given i the axis with the highest (relative) out of
// bounds speed, what's the speed of the defining_axis so that every speed
// respects i's bounds? We just need to project each speed to the defining
// axis speed, and pick the smallest.
//
// We need to work in mm, since steps is not a uniform unit of space across
// the different axes.
static float clamp_defining_axis_limit(const StepsAxesRegister &axes_steps,
                                       const FloatAxisConfig &axes_limits_mm,
                                       enum GCodeParserAxis defining_axis,
                                       const FloatAxisConfig &steps_per_mm) {
  float new_defining_axis_limit = axes_limits_mm[defining_axis];
  for (const GCodeParserAxis i : AllAxes()) {
    if (axes_steps[i] == 0) continue;
    const float ratio =
      std::fabs((1.0 * axes_steps[i] * steps_per_mm[defining_axis]) /
                (axes_steps[defining_axis] * steps_per_mm[i]));
    new_defining_axis_limit =
      std::min(new_defining_axis_limit, axes_limits_mm[i] / ratio);
  }
  return new_defining_axis_limit * steps_per_mm[defining_axis];
}

// Uniformly accelerated ramp distance estimation.
// The ramp is parametrized by v0 as the starting speed, v1 the final speed
// and the constant acceleration.
static double uniformly_accelerated_ramp_distance(double v0, double v1,
                                                  double acceleration) {
  return (v1 * v1 - v0 * v0) / (2 * acceleration);
}

// Uniformly accelerated ramp final speed estimation where
// s is the travelled distance, v0 the starting speed and
// acceleration is the constant acceleration.
static double uniformly_accelerated_ramp_speed(double s, double v0,
                                               double acceleration) {
  return std::sqrt(2 * acceleration * s + v0 * v0);
}

// Compute the distance for an acceleration ramp to cross a deceleration
// ramp with the same module of the acceleration.
static double find_uniformly_accelerated_ramps_intersection(double v0,
                                                            double v1,
                                                            double acceleration,
                                                            double distance) {
  const double min_v = v0 <= v1 ? v0 : v1;
  const double max_v = v0 <= v1 ? v1 : v0;
  // Distance we would need to reach the maximum between the two speeds using
  // the acceleration provided.
  const double reaching_distance =
    uniformly_accelerated_ramp_distance(min_v, max_v, acceleration);
  return reaching_distance * (v0 < v1) + (distance - reaching_distance) / 2;
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

class Planner::Impl {
 public:
  Impl(const MachineControlConfig *config, HardwareMapping *hardware_mapping,
       SegmentQueue *motor_backend);
  ~Impl();

  void assign_steps_to_motors(struct LinearSegmentSteps *command,
                              enum GCodeParserAxis axis, int steps);

  bool issue_motor_move_if_possible(bool flush_planning_queue = false);
  bool machine_move(const AxesRegister &axis, float feedrate);
  void bring_path_to_halt();

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

  bool SetLookahead(int size) {
    if (size <= 0 || size > GetMaxLookahead()) return false;
    lookahead_size_ = size;
    return true;
  }
  int Lookahead() const { return lookahead_size_; }
  static constexpr int GetMaxLookahead() { return PLANNING_BUFFER_CAPACITY; }

 private:
  const struct MachineControlConfig *const cfg_;
  HardwareMapping *const hardware_mapping_;
  SegmentQueue *const motor_ops_;

  struct PlanningSegment {
    AxisTarget target;
    PlannedProfile planned;

    std::string ToJsonString() const {
      std::ostringstream ss;
      ss << "{";
      ss << "\"target\":" << target.ToJsonString() << ",";
      ss << "\"planned\":" << planned.ToJsonString();
      ss << "}";
      return ss.str();
    }
  };

  bool DecelerationPlanSegment(PlanningSegment *segment_to_plan,
                               const PlanningSegment *next_segment);
  bool AccelerationPlanSegment(const PlanningSegment *previous_segment,
                               PlanningSegment *segment_to_plan);

  // Run backward and forward passes to
  // compute the new profile.
  void UpdateMotionProfile();

  // Next buffered positions. Written by incoming gcode, read by outgoing
  // motor movements.
  RingDeque<PlanningSegment, PLANNING_BUFFER_CAPACITY> planning_buffer_;

  // The planned motion is stored in the planning_buffer_ "planned" member.
  // The planned motion always reaches speed zero at the end of the trajectory.
  // This means there will always be a final deceleration ramp, maybe with some
  // travels but no accels, that might change. Everything before this final
  // deceleration ramp, is bound to stay the same, for this reason, we call it
  // planned-invariant. In this variable we store the position of the segment
  // from which this deceleration ramp starts. This allows us to avoid running
  // the motion profile update as long as we have enough planned-invariants
  // segments and update only the last deceleration ramp.
  int num_segments_ready_ = 0;

  // Number of maximum planning steps allow to enqueue.
  size_t lookahead_size_ = PLANNING_BUFFER_CAPACITY;

  // Pre-calculated per axis limits in steps, steps/s, steps/s^2
  // All arrays are indexed by axis.
  AxesRegister max_axis_speed_;  // max travel speed hz
  AxesRegister max_axis_accel_;  // acceleration hz/s

  bool path_halted_;
  bool position_known_;
};

// Every AxisTarget, the defining axis might change. Speeds though, should stay
// continuous between each target for every axis. We need a function to compute
// what was the speed in the previous target for the new defining axis.
static double get_speed_factor_for_axis(const struct AxisTarget *t,
                                        enum GCodeParserAxis request_axis) {
  if (t->delta_steps[t->defining_axis] == 0) return 0.0;
  return (request_axis == t->defining_axis)
           ? 1.0
           : fabs(1.0 * t->delta_steps[request_axis] /
                  t->delta_steps[t->defining_axis]);
}

// Get the speed for a particular axis. Depending on the direction, this can
// be positive or negative.
static double get_speed_for_axis(const struct AxisTarget *target,
                                 enum GCodeParserAxis request_axis) {
  return target->speed * get_speed_factor_for_axis(target, request_axis);
}

static double euclid_distance(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
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
  const double dot = from->dx * to->dx + from->dy * to->dy + from->dz * to->dz;
  const double mag = from->len * to->len;
  if (dot == 0) return 0.0;  // orthogonal 90 degree, full stop
  if (dot < 0) return 0.0;   // turning around, full stop
  const double dotmag = dot / mag;
  if (within_acceptable_range(1.0, dotmag, 1e-5))
    return to->speed;  // codirectional 0 degree, keep accelerating

  // the angle between the vectors
  const double rad2deg = 180.0 / M_PI;
  const double angle = std::fabs(std::acos(dotmag) * rad2deg);

  if (angle >= 45.0) return 0.0;  // angle to large, come to full stop
  if (angle <= threshold) {       // in tolerance, keep accelerating
    if (dot < 1) {  // speed tune segments less than 1mm (i.e. arcs)
      const double deg2rad = M_PI / 180.0;
      const double angle_speed_adj =
        std::cos((angle + speed_tune_angle) * deg2rad);
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
    if (from_delta == 0 && to_delta == 0) continue;  // uninteresting: no move.
    if (from_delta == 0 || to_delta == 0) return 0.0;  // accel from/to zero
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
                    SegmentQueue *motor_backend)
    : cfg_(config),
      hardware_mapping_(hardware_mapping),
      motor_ops_(motor_backend),
      path_halted_(true),
      position_known_(true) {
  // Initial machine position. We assume the homed position here, which is
  // wherever the endswitch is for each axis.
  struct PlanningSegment *init_axis = planning_buffer_.append();
  *init_axis = {};

  for (const GCodeParserAxis axis : AllAxes()) {
    HardwareMapping::AxisTrigger trigger = cfg_->homing_trigger[axis];
    const float home_pos =
      trigger == HardwareMapping::TRIGGER_MAX ? cfg_->move_range_mm[axis] : 0;
    SetExternalPosition(axis, home_pos);
  }
  position_known_ = true;

  for (const GCodeParserAxis i : AllAxes()) {
    max_axis_speed_[i] = cfg_->max_feedrate[i] * cfg_->steps_per_mm[i];
    const float accel = cfg_->acceleration[i] * cfg_->steps_per_mm[i];
    max_axis_accel_[i] = accel;
  }
#if 0
  fprintf(stderr, "\nmax_accelerations:\n[");
  for (unsigned i = 0; i < max_axis_accel_.size(); ++i) {
    fprintf(stderr, "%s%f", !i ? "" : ",", max_axis_accel_[(GCodeParserAxis)i]);
  }
  fprintf(stderr, "]\n");
#endif
}

Planner::Impl::~Impl() { bring_path_to_halt(); }

// Assign steps to all the motors responsible for given axis.
void Planner::Impl::assign_steps_to_motors(struct LinearSegmentSteps *command,
                                           enum GCodeParserAxis axis,
                                           int steps) {
  hardware_mapping_->AssignMotorSteps(axis, steps, command);
}

double Planner::Impl::euclidian_speed(const struct AxisTarget *t) {
  double speed_factor = 1.0;
  if (t->len > 0) {
    const double axis_len_mm = axis_delta_to_mm(t, t->defining_axis);
    speed_factor = std::fabs(axis_len_mm) / t->len;
  }
  return t->speed * speed_factor;
}

// Select a starting chunk of the planned trajectory and send to the backend.
// if flush_planning_queue is true, we flush the planned trajectory and commit
// to the planned motion until reaching zero speed.
bool Planner::Impl::issue_motor_move_if_possible(
  const bool flush_planning_queue) {
  bool ret = true;

  // We need to change the other axes steps for accel travel decel. We have to
  // be careful of having the right rounding but at the same time accel + travel
  // + decel has to stay the same. 1) find the deceleration ramp and the number
  // of segments you can push in the backend.
  uint32_t num_segments = planning_buffer_.size();
  if (!flush_planning_queue) {
    num_segments = 0;
  }

#if 0
  fprintf(stderr, "\nplanning buffer dump start: %d/%zu.\n[", num_segments, planning_buffer_.size());
  for (unsigned i = 0; i < planning_buffer_.size(); ++i) {
    fprintf(stderr, "%s%s", !i ? "" : ",", planning_buffer_[i]->ToJsonString().c_str());
  }
  fprintf(stderr, "]\n");
#endif

  // Count the amount of segments that are extra the lookahead size.
  const int segments_extra = planning_buffer_.size() - lookahead_size_;
  if (num_segments == 0 && (segments_extra >= 0))
    num_segments =
      segments_extra + 1;  // Always pop one free slot for the next.

  // No segments to enqueue, skip.
  if (!num_segments) return ret;

  // 2) submit the segments
  // Flush the full queue.
  struct LinearSegmentSteps accel_command = {};
  struct LinearSegmentSteps move_command = {};
  struct LinearSegmentSteps decel_command = {};
  PlanningSegment *segment = NULL;

  for (uint32_t i = 0; i < num_segments; ++i) {
    segment = planning_buffer_[0];

    memset(&move_command, 0, sizeof(move_command));
    move_command.aux_bits = segment->target.aux_bits;
    memcpy(&accel_command, &move_command, sizeof(accel_command));
    memcpy(&decel_command, &move_command, sizeof(decel_command));

    const unsigned defining_axis_steps = segment->planned.TotalSteps();
    const double accel_fraction =
      (double)segment->planned.accel / defining_axis_steps;
    const double decel_fraction =
      (double)segment->planned.decel / defining_axis_steps;

    // Accel
    if (segment->planned.accel) {
      for (const GCodeParserAxis a : AllAxes()) {
        const unsigned accel_steps =
          std::lround(accel_fraction * segment->target.delta_steps[a]);
        assign_steps_to_motors(&accel_command, a, accel_steps);
      }
      accel_command.v0 = segment->planned.v0;
      accel_command.v1 = segment->planned.v1;
    }

    // Decel
    if (segment->planned.decel) {
      for (const GCodeParserAxis a : AllAxes()) {
        const unsigned decel_steps =
          std::lround(decel_fraction * segment->target.delta_steps[a]);
        assign_steps_to_motors(&decel_command, a, decel_steps);
      }
      decel_command.v0 = segment->planned.v1;
      decel_command.v1 = segment->planned.v2;
    }

    // Travel
    for (const GCodeParserAxis a : AllAxes()) {
      assign_steps_to_motors(&move_command, a, segment->target.delta_steps[a]);
      move_command.v0 = segment->planned.v1;
      move_command.v1 = segment->planned.v1;
    }

    // Now we substract from travel both accel and decel.
    subtract_steps(&move_command, accel_command);
    const bool has_move = subtract_steps(&move_command, decel_command);

    if (cfg_->synchronous) motor_ops_->WaitQueueEmpty();

    if (segment->planned.accel) ret = motor_ops_->Enqueue(accel_command);
    if (has_move && ret) ret = motor_ops_->Enqueue(move_command);
    if (segment->planned.decel && ret) ret = motor_ops_->Enqueue(decel_command);

    // We always keep one segment to keep track of the last
    // position and aux values.
    if (planning_buffer_.size() > 1) {
      planning_buffer_.pop_front();
      if (num_segments_ready_ > 0) --num_segments_ready_;
    } else {
      // Let's create a zero-steps segment to hold last position.
      // We have only one segment, last speed is always 0.
      const StepsAxesRegister last_pos = segment->target.position_steps;
      *segment = {};
      segment->target.position_steps = last_pos;
      path_halted_ = true;
      ret = false;
      num_segments_ready_ = 0;
    }
  }

  return ret;
}

// At this stage of the planning we receive a new position we want to reach and
// a target feedrate. We always try to reach the highest possible speed whenever
// we move. For this reason, the feedrate is just interpreted as a speed limit.
// This function performs three fundamental steps:
// 1: Create a new empty planning segment in the planning buffer to be filled
// with a new AxisTarget. 2: Determine the defining axis of the newly pushed
// AxisTarget and its limits considering
//    the angle w.r.t the previous segment and maximum accelerations and speeds
//    of the other axes.
// 3: Update the planning buffer (UpdateMotionProfile) to enforce constrained
// motion
//    that will always end up at 0 speed.
// 4: Enqueue the front of the planning buffer to the motors if certain
//    conditions occur.
bool Planner::Impl::machine_move(const AxesRegister &axis, float feedrate) {
  assert(position_known_);  // call SetExternalPosition() after DirectDrive()

  // We always assume there's enough space to store a new segment and that we
  // have at least one segment.
  assert(planning_buffer_.size() < planning_buffer_.capacity() &&
         !planning_buffer_.empty());

  // We always have a previous position.
  struct AxisTarget *prev_pos = &planning_buffer_.back()->target;
  const StepsAxesRegister &previous_position_steps = prev_pos->position_steps;

  // Create a new empty segment.
  PlanningSegment *planning_segment = planning_buffer_.append();
  *planning_segment = {};
  struct AxisTarget *new_pos = &planning_segment->target;

  int max_steps = -1;
  enum GCodeParserAxis defining_axis = AXIS_X;

  // Real world -> machine coordinates. Here, we are rounding to the next full
  // step, but we never accumulate the error, as we always use the absolute
  // position as reference.
  for (const GCodeParserAxis a : AllAxes()) {
    new_pos->position_steps[a] = std::lround(axis[a] * cfg_->steps_per_mm[a]);
    new_pos->delta_steps[a] =
      new_pos->position_steps[a] - previous_position_steps[a];

    // The defining axis is the one that has to travel the most steps. It
    // defines the frequency to go. All the other axes are doing a fraction of
    // the defining axis.
    if (abs(new_pos->delta_steps[a]) > max_steps) {
      max_steps = abs(new_pos->delta_steps[a]);
      defining_axis = a;
    }
  }

  if (max_steps == 0) {
    // Nothing to do, ignore this move.
    planning_buffer_.pop_back();
    return true;
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

  // Convert the arc-length feedrate (hypotenuse) from mm/s
  // to the defininx axis step/s.
  new_pos->speed = feedrate * cfg_->steps_per_mm[defining_axis];

  // Project the euclidean speed to the defining axis leg.
  new_pos->speed = euclidian_speed(new_pos);
  new_pos->start_speed = new_pos->speed;

  // The previous segment needs to arrive at a speed that the upcoming
  // move does not have to decelerate further
  // (after all, it has a fixed feed-rate it should not go over).
  const double new_previous_speed = determine_joining_speed(
    prev_pos, new_pos, cfg_->threshold_angle, cfg_->speed_tune_angle);

  // Clamp the next speed to insure that this segment does not go over.
  // The new target speed must be equal or lower than the previously planned.
  if (new_previous_speed < new_pos->start_speed)
    new_pos->start_speed = new_previous_speed;

  // Make sure the target feedrate for the move is clamped to what all the
  // moving axes can reach.
  const double max_speed =
    clamp_defining_axis_limit(new_pos->delta_steps, cfg_->max_feedrate,
                              defining_axis, cfg_->steps_per_mm);
  if (max_speed < new_pos->speed) new_pos->speed = max_speed;

  // Define the maximum acceleration given the following motion angles and
  // defining axis.
  new_pos->accel =
    clamp_defining_axis_limit(new_pos->delta_steps, cfg_->acceleration,
                              defining_axis, cfg_->steps_per_mm);

  // Run the planning algorithm.
  // Update the planning_buffer_.planned struct, as well as
  // redefine starting and final speeds.
  UpdateMotionProfile();

  bool ret = issue_motor_move_if_possible();
  if (ret) path_halted_ = false;
  return ret;
}

void Planner::Impl::bring_path_to_halt() {
  if (path_halted_) return;

  // Flush the queue.
  issue_motor_move_if_possible(true);
}

// Compute the backard pass of <segment_to_plan> given the
// information (such as starting speed and defining axis) of the <next_segment>.
// If <next_segment> is nullptr, the next segment is assumed to not exist (end
// of the buffer). The function returns false if the final speed already
// connects with the starting speed of the next segment and this is not the last
// segment.
bool Planner::Impl::DecelerationPlanSegment(
  PlanningSegment *segment_to_plan, const PlanningSegment *next_segment) {
  // Handle special case of zero feedrate. The segment is dummy as any no amount
  // of steps and be executed at zero speed.
  if (segment_to_plan->target.speed == 0) {
    return true;
  }

  const bool is_last_segment = next_segment == nullptr;
  const enum GCodeParserAxis defining_axis =
    segment_to_plan->target.defining_axis;
  const AxisTarget &target = segment_to_plan->target;
  PlannedProfile &planned = segment_to_plan->planned;

  // Amount of absolute number of steps to be travelled in this segment
  // by the defining axis.
  const int delta_steps = abs(target.delta_steps[defining_axis]);
  const GCodeParserAxis next_defining_axis =
    is_last_segment ? defining_axis : next_segment->target.defining_axis;

  // This speed is the starting speed of the defining axis of the next segment
  // that we have to match our v2 with. The final speed we have to match is
  // either zero if this is the last segment or v0 otherwise.
  const double next_v0 =
    is_last_segment
      ? 0
      : std::min(next_segment->planned.v1, next_segment->target.start_speed);

  // We want to have continuous speeds. We calculate what would be that speed
  // for the current defining axis.
  const auto speed_factor =
    get_speed_factor_for_axis(&target, next_defining_axis);

  // Final speed constraint.
  const double end_speed = (speed_factor != 0) ? next_v0 / speed_factor : 0;

  // Check if the current segment v2 speed connects with the next
  // (except if this is the very last segment).
  // If we do, then we stop here. No need to compute the accel. It will happen
  // in the forward pass. The new profile deceleration and travel will always
  // be above or equal to the current profile. This will only then happen if
  // the new backward profile v2 equals the previous planned profile v2.
  // Since the compared floats are results of the same algorithm, we can safely
  // directly compare the floating values without checking some relative
  // distance.
  if (planned.v2 == end_speed && !is_last_segment) {
    // The current segment doesn't need to be touched in the forward pass.
    // We increment the buffer index and keep _speed_ the same as it should
    // already be the v0 of the next segment defining axis.
    // We return false as no deceleration occurs.
    return false;
  }

  // We can decelarate.
  if (target.speed > end_speed) {
    // Let's see if we can just fill the segment with a full decel ramp.
    // This is the total space required to reach max speed.
    const int steps_to_max_speed = uniformly_accelerated_ramp_distance(
      end_speed, target.speed, target.accel);

    planned.decel =
      (steps_to_max_speed > delta_steps) ? delta_steps : steps_to_max_speed;
    planned.v1 = (steps_to_max_speed > 0)
                   ? uniformly_accelerated_ramp_speed(planned.decel, end_speed,
                                                      target.accel)
                   : target.speed;
    planned.travel = delta_steps - planned.decel;
    planned.v2 = end_speed;
  } else {
    planned.decel = 0;
    planned.travel = delta_steps;
    planned.v1 = planned.v2 = target.speed;
  }

  // Let's reset the final speed always as v1. We want to keep a consistent
  // profile state which means if no accel -> v0 == v1 and no decel -> v1 ==
  // v2.
  // planned.v0 = planned.v1;
  planned.accel = 0;
  return true;
}

bool Planner::Impl::AccelerationPlanSegment(
  const PlanningSegment *previous_segment, PlanningSegment *segment_to_plan) {
  const bool is_first_segment = previous_segment == nullptr;
  const enum GCodeParserAxis defining_axis =
    segment_to_plan->target.defining_axis;
  const AxisTarget &target = segment_to_plan->target;
  PlannedProfile &planned = segment_to_plan->planned;

  const double previous_v2 = is_first_segment ? segment_to_plan->planned.v0
                                              : previous_segment->planned.v2;

  const auto speed_factor = get_speed_factor_for_axis(&target, defining_axis);
  const double start_speed =
    (speed_factor != 0) ? previous_v2 / speed_factor : 0;
  planned.v0 = start_speed;

  // Recompute the accel ramp if necessary.
  if (planned.v1 == 0) {
    return false;
  }

  // Nothing to accelerate. Starting speed is above the travel speed.
  if (start_speed >= planned.v1) {
    planned.accel = 0;
    planned.v0 = planned.v1;
    return false;
  }

  const uint32_t steps = planned.TotalSteps();
  const uint32_t steps_to_vmax =
    uniformly_accelerated_ramp_distance(start_speed, planned.v1, target.accel);

  // We intersect with the travel part.
  if (steps_to_vmax <= planned.travel) {
    planned.v0 = start_speed;
    planned.accel = steps_to_vmax;
    planned.travel = planned.travel - steps_to_vmax;
    return true;
  }

  // We don't intersect with travel.
  planned.travel = 0;
  const double end_speed =
    uniformly_accelerated_ramp_speed(steps, start_speed, target.accel);

  // We do only accel and decel, no travel.
  if (end_speed > planned.v2) {
    // NOTE(lromor): Here we are searching for the amount of steps at which
    // two accel-decel profiles would converge. If this value ends up being
    // very close to the end length of the segment, it means that we might
    // have a single step dedicated to deceleration. Due to step
    // discretization we have a rounding error that mighe undershoot or
    // overshoot the actual joining speed for instance going below the final
    // speed v2 or above v1. We fix this by casting to int and assuming that
    // by doing so, the worst case scenario is the new v1 to go below v2. At
    // that point we force v2 to be equal v1 and remove any other deceleration
    // step.
    planned.accel = find_uniformly_accelerated_ramps_intersection(
      start_speed, planned.v2, target.accel, steps);
    planned.v1 = uniformly_accelerated_ramp_speed(planned.accel, start_speed,
                                                  target.accel);
    if (planned.v1 <= planned.v2) {
      planned.v2 = planned.v1;
      planned.accel = steps;
    }

    planned.decel = steps - planned.accel;
    planned.v0 = start_speed;
    return true;
  }

  // At the end of the segment we fall below v2, means
  // we remove any travel and decel.
  planned = {steps, 0, 0, start_speed, end_speed, end_speed};
  return true;
}

// Perform a backward and forward pass across
// the whole planning buffer to adapt the start
// and final speed of all the segments.
void Planner::Impl::UpdateMotionProfile() {
  assert(!planning_buffer_.empty());

  // Starting speed for each Planning segment.
  int buffer_index = planning_buffer_.size() - 1;

  // Next speed is the initial speed of the following(next) segment on the
  // original defining axis. We start from the backward pass so it's always 0
  // since we plan to always decelerate to zero.
  PlanningSegment *next_segment = nullptr;
  PlanningSegment *previous_segment = nullptr;

  // Perform a backward pass.
  for (; buffer_index >= num_segments_ready_; --buffer_index) {
    // We connect to the previously existing profile, there's no need to updated
    // it further!
    previous_segment = planning_buffer_[buffer_index];
    if (!DecelerationPlanSegment(previous_segment, next_segment)) {
      break;
    }
    next_segment = previous_segment;
  }
  bool first_accel = false;
  previous_segment = nullptr;

  // Perform a forward pass.
  for (++buffer_index; buffer_index < (int)planning_buffer_.size();
       ++buffer_index) {
    next_segment = planning_buffer_[buffer_index];
    if (AccelerationPlanSegment(previous_segment, next_segment) &&
        !first_accel) {
      // We have acceleration.
      // Let's update the last invariant segment.
      num_segments_ready_ = buffer_index;
      first_accel = true;
    }
    previous_segment = next_segment;
  }
}

void Planner::Impl::GetCurrentPosition(AxesRegister *pos) {
  pos->zero();
  PhysicalStatus physical_status;
  if (!motor_ops_->GetPhysicalStatus(&physical_status))
    return;  // Should we return boolean to indicate that not supported ?
  for (const GCodeParserAxis a : AllAxes()) {
#if M114_DEBUG
    Log_debug("Machine steps Axis %c : %8d\n", gcodep_axis2letter(a),
              hardware_mapping_->GetAxisSteps(a, physical_status));
#endif
    if (cfg_->steps_per_mm[a] != 0) {
      (*pos)[a] = hardware_mapping_->GetAxisSteps(a, physical_status) /
                  cfg_->steps_per_mm[a];
    }
  }
}

int Planner::Impl::DirectDrive(GCodeParserAxis axis, float distance, float v0,
                               float v1) {
  bring_path_to_halt();  // Precondition. Let's just do it for good measure.
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
  assert(path_halted_ && planning_buffer_.size() == 1);  // Precondition.
  position_known_ = true;

  const int motor_position = std::lround(pos * cfg_->steps_per_mm[axis]);
  planning_buffer_.back()->target.position_steps[axis] = motor_position;
  planning_buffer_[0]->target.position_steps[axis] = motor_position;

  const uint8_t motormap_for_axis = hardware_mapping_->GetMotorMap(axis);
  for (int motor = 0; motor < BEAGLEG_NUM_MOTORS; ++motor) {
    if (motormap_for_axis & (1 << motor))
      motor_ops_->SetExternalPosition(motor, motor_position);
  }
}

// -- public interface

Planner::Planner(const MachineControlConfig *config,
                 HardwareMapping *hardware_mapping, SegmentQueue *motor_backend)
    : impl_(new Impl(config, hardware_mapping, motor_backend)) {}

Planner::~Planner() { delete impl_; }

bool Planner::Enqueue(const AxesRegister &target_pos, float speed) {
  return impl_->machine_move(target_pos, speed);
}

void Planner::BringPathToHalt() { impl_->bring_path_to_halt(); }

void Planner::GetCurrentPosition(AxesRegister *pos) {
  impl_->GetCurrentPosition(pos);
}

int Planner::DirectDrive(GCodeParserAxis axis, float distance, float v0,
                         float v1) {
  return impl_->DirectDrive(axis, distance, v0, v1);
}

void Planner::SetExternalPosition(GCodeParserAxis axis, float pos) {
  impl_->SetExternalPosition(axis, pos);
}

bool Planner::SetLookahead(int size) { return impl_->SetLookahead(size); }

int Planner::Lookahead() const { return impl_->Lookahead(); }

// Get the maximum allowed lookahead size.
int Planner::GetMaxLookahead() const {
  return Planner::Impl::GetMaxLookahead();
}

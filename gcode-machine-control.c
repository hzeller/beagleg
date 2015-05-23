/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
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

// TODO: this is motion planner and 'other stuff printers do' in one. Separate.
// TODO: this is somewhat work-in-progress.

#include "gcode-machine-control.h"

#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/types.h>
#include <unistd.h>

#include "motor-operations.h"
#include "gcode-parser.h"
#include "generic-gpio.h"

// In case we get a zero feedrate, send this frequency to motors instead.
#define ZERO_FEEDRATE_OVERRIDE_HZ 5

#define VERSION_STRING "PROTOCOL_VERSION:0.1 FIRMWARE_NAME:BeagleG "    \
  "FIRMWARE_URL:http%3A//github.com/hzeller/beagleg"

// aux_bits
#define AUX_BIT_MIST        (1 << 0)
#define AUX_BIT_FLOOD       (1 << 1)
#define AUX_BIT_VACUUM      (1 << 2)
#define AUX_BIT_SPINDLE_ON  (1 << 3)
#define AUX_BIT_SPINDLE_DIR (1 << 4)
#define MAX_AUX_PIN         4
#define NUM_ENDSTOPS        6

// Some default settings. These are most likely overrridden via flags by user.

// All these settings are in sequence of enum GCodeParserAxes: XYZEABCUVW. Axes not
// used by default are initialized to 0
static const float kMaxFeedrate[GCODE_NUM_AXES] = {  200,  200,   90,    10, 1 };
static const float kDefaultAccel[GCODE_NUM_AXES]= { 4000, 4000, 1000, 10000, 1 };
static const float kStepsPerMM[GCODE_NUM_AXES]  = {  160,  160,  160,    40, 1 };
static const float kMoveRange[GCODE_NUM_AXES] = { 100, 100, 100, -1, -1 };

// Output mapping of Axis to motor connectors from left to right.
static const char kAxisMapping[] = "XYZEA";

// Mapping the motor connectors to actual output channels.
// This is the channel layout on the Bumps-board ( github.com/hzeller/bumps ),
// currently the only cape existing for BeagleG, so we can as well hardcode it.
// (for BURPS, this would be "01234567")
static const char kChannelLayout[] = "23140";

static const char kHomeOrder[] = "ZXY";

// The vector is essentially a position in the GCODE_NUM_AXES dimensional
// space. An AxisTarget has a position vector, in absolute machine coordinates, and a
// speed when arriving at that position.
// The speed is generally an aimed goal; if it cannot be reached, the AxisTarget will
// be modified
struct AxisTarget {
  int position_steps[GCODE_NUM_AXES];  // Absolute position at end of segment. In steps.

  // Derived values
  int delta_steps[GCODE_NUM_AXES];     // Difference to previous position.
  enum GCodeParserAxis defining_axis;  // index into defining axis.
  float speed;                         // (desired) speed in steps/s on defining axis.
  unsigned int aux_bits;               // Auxiliarry bits in this segment; set with M42
};

// TargetBuffer implementation
struct TargetBuffer {
  unsigned write_pos;
  unsigned read_pos;
  struct AxisTarget ring_buffer[4];
};

// Initialize a new buffer.
static void target_buffer_init(struct TargetBuffer *b) {
  bzero(b, sizeof(*b));
}

// Returns number of available read items.
static int buffer_available(struct TargetBuffer *b) {
  return (b->write_pos + 4 - b->read_pos) % 4;
}

// Add a new target and return pointer. Makes sure that we never override
static struct AxisTarget *buffer_add_next_target(struct TargetBuffer *b) {
  assert(buffer_available(b) < 3);
  struct AxisTarget *result = &b->ring_buffer[b->write_pos];
  b->write_pos = (b->write_pos + 1) % 4;
  return result;
}

// Get the last item written.
static struct AxisTarget *buffer_get_last_written(struct TargetBuffer *b) {
  assert(buffer_available(b) > 0);
  return &b->ring_buffer[(b->write_pos + 3) % 4];
}

// Get the given target, delta 0 up to buffer_peek_available();
static struct AxisTarget *buffer_at(struct TargetBuffer *b, int delta) {
  assert(buffer_available(b) > delta);
  return &b->ring_buffer[(b->read_pos + delta) % 4];
}

// Move on to the next read position.
static void buffer_next(struct TargetBuffer *b) {
  assert(buffer_available(b) > 0);
  b->read_pos = (b->read_pos + 1) % 4;
}

typedef uint8_t DriverBitmap;

// Compact representation of an enstop configuration.
struct EndstopConfig {
  unsigned char trigger_value : 1;   // 0: trigged low 1: triggered high.
  unsigned char homing_use : 1;      // 0: no 1: yes.
  unsigned char endstop_number : 6;  // 0: no mapping; or (1..NUM_ENDSTOPS+1)
};

struct GCodeMachineControl {
  struct GCodeParserCb event_input;
  struct MotorOperations *motor_ops;
  const struct MachineControlConfig cfg;

  // Derived configuration
  float g0_feedrate_mm_per_sec;          // Highest of all axes; used for G0
                                         // (will be trimmed if needed)
  // Pre-calcualted per axis limits in steps/s, steps/s^2
  // All arrays are indexed by axis.
  float max_axis_speed[GCODE_NUM_AXES];  // max travel speed hz
  float max_axis_accel[GCODE_NUM_AXES];  // acceleration hz/s
  float highest_accel;                   // hightest accel of all axes.

  // "axis_to_driver": Which axis is mapped to which physical output drivers.
  // This allows to have a logical axis (e.g. X, Y, Z) output to any physical
  // or a set of multiple drivers (mirroring).
  // Bitmap of drivers output should go.
  DriverBitmap axis_to_driver[GCODE_NUM_AXES];

  int axis_flip[GCODE_NUM_AXES];        // 1 or -1 for direction flip of axis
  int driver_flip[BEAGLEG_NUM_MOTORS];  // 1 or -1 for for individual driver

  // Mapping of Axis to which endstop it affects.
  struct EndstopConfig min_endstop[GCODE_NUM_AXES];
  struct EndstopConfig max_endstop[GCODE_NUM_AXES];

  // Current machine configuration
  float current_feedrate_mm_per_sec;     // Set via Fxxx and remembered
  float prog_speed_factor;               // Speed factor set by program (M220)
  unsigned int aux_bits;                 // Set via M42.
  unsigned int spindle_rpm;              // Set via Sxxx of M3/M4 and remembered

  // Next buffered positions. Written by incoming gcode, read by outgoing
  // motor movements.
  struct TargetBuffer buffer;

  FILE *msg_stream;
};

static uint32_t get_endstop_gpio_descriptor(struct EndstopConfig config);

static inline int round2int(float x) { return (int) roundf(x); }

static void bring_path_to_halt(GCodeMachineControl_t *state);

// Dummy implementations of callbacks not yet handled.
static void dummy_set_temperature(void *userdata, float f) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "// BeagleG: set_temperature(%.1f) not implemented.\n", f);
  }
}
static void dummy_set_fanspeed(void *userdata, float speed) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "// BeagleG: set_fanspeed(%.0f) not implemented.\n", speed);
  }
}
static void dummy_wait_temperature(void *userdata) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "// BeagleG: wait_temperature() not implemented.\n");
  }
}
static void motors_enable(void *userdata, char b) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  bring_path_to_halt(state);
  state->motor_ops->motor_enable(state->motor_ops->user_data, b);
}
static void gcode_send_ok(void *userdata, char l, float v) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream, "ok\n");
  }
}

static const char *special_commands(void *userdata, char letter, float value,
				    const char *remaining) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  const int code = (int)value;

  if (letter == 'M') {
    int pin = -1;
    int aux_bit = -1;

    switch (code) {
    case 0: set_gpio(ESTOP_SW_GPIO); return remaining;
    case 3:
    case 4:
      for (;;) {
        const char* after_pair = gcodep_parse_pair(remaining, &letter, &value,
                                                   state->msg_stream);
        if (after_pair == NULL) break;
        else if (letter == 'S') state->spindle_rpm = round2int(value);
        else break;
        remaining = after_pair;
      }
      if (state->spindle_rpm) {
        state->aux_bits |= AUX_BIT_SPINDLE_ON;
	if (code == 3) state->aux_bits &= ~AUX_BIT_SPINDLE_DIR;
        else state->aux_bits |= AUX_BIT_SPINDLE_DIR;
      }
      return remaining;
    case 5: state->aux_bits &= ~(AUX_BIT_SPINDLE_ON | AUX_BIT_SPINDLE_DIR); return remaining;
    case 7: state->aux_bits |= AUX_BIT_MIST; return remaining;
    case 8: state->aux_bits |= AUX_BIT_FLOOD; return remaining;
    case 9: state->aux_bits &= ~(AUX_BIT_MIST | AUX_BIT_FLOOD); return remaining;
    case 10: state->aux_bits |= AUX_BIT_VACUUM; return remaining;
    case 11: state->aux_bits &= ~AUX_BIT_VACUUM; return remaining;
    case 42:
    case 62:
    case 63:
    case 64:
    case 65:
      for (;;) {
        const char* after_pair = gcodep_parse_pair(remaining, &letter, &value,
                                                   state->msg_stream);
        if (after_pair == NULL) break;
        if (letter == 'P') pin = round2int(value);
        else if (letter == 'S' && code == 42) aux_bit = round2int(value);
        else break;
        remaining = after_pair;
      }
      if (code == 62 || code == 64)
        aux_bit = 1;
      else if (code == 63 || code == 65)
        aux_bit = 0;
      if (pin >= 0 && pin <= MAX_AUX_PIN) {
        if (aux_bit >= 0 && aux_bit <= 1) {
          if (aux_bit) state->aux_bits |= 1 << pin;
          else state->aux_bits &= ~(1 << pin);
        } else if (code == 42 && state->msg_stream) {  // Just read operation.
          fprintf(state->msg_stream, "%d\n", (state->aux_bits >> pin) & 1);
	}
      }
      return remaining;
    case 80: set_gpio(MACHINE_PWR_GPIO); return remaining;
    case 81: clr_gpio(MACHINE_PWR_GPIO); return remaining;
    case 999: clr_gpio(ESTOP_SW_GPIO); return remaining;
    }

    // The remaining codes are only useful when we have an output stream.
    if (!state->msg_stream)
      return remaining;
    switch (code) {
    case 105: fprintf(state->msg_stream, "T-300\n"); break;  // no temp yet.
    case 114:
      if (buffer_available(&state->buffer)) {
        struct AxisTarget *current = buffer_at(&state->buffer, 0);
        fprintf(state->msg_stream, "X:%.3f Y:%.3f Z:%.3f E:%.3f\n",
                (1.0f * current->position_steps[AXIS_X] / state->cfg.steps_per_mm[AXIS_X]),
                (1.0f * current->position_steps[AXIS_Y] / state->cfg.steps_per_mm[AXIS_Y]),
                (1.0f * current->position_steps[AXIS_Z] / state->cfg.steps_per_mm[AXIS_Z]),
                (1.0f * current->position_steps[AXIS_E] / state->cfg.steps_per_mm[AXIS_E]));
      } else {
        fprintf(state->msg_stream, "// no current pos\n");
      }
      break;
    case 115: fprintf(state->msg_stream, "%s\n", VERSION_STRING); break;
    case 117: fprintf(state->msg_stream, "Msg: %s\n", remaining); break;
    case 119:
      for (int axis = 0; axis < GCODE_NUM_AXES; ++axis) {
        struct EndstopConfig config = state->min_endstop[axis];
        if (config.endstop_number) {
          int value = get_gpio(get_endstop_gpio_descriptor(config));
          fprintf(state->msg_stream, "%c_min:%s ",
                  tolower(gcodep_axis2letter(axis)),
                  value == config.trigger_value ? "TRIGGERED" : "open");
        }
        config = state->max_endstop[axis];
        if (config.endstop_number) {
          int value = get_gpio(get_endstop_gpio_descriptor(config));
          fprintf(state->msg_stream, "%c_max:%s ",
                  tolower(gcodep_axis2letter(axis)),
                  value == config.trigger_value ? "TRIGGERED" : "open");
        }
      }
      fprintf(state->msg_stream, "\n");
      break;
    default:  fprintf(state->msg_stream,
                      "// BeagleG: didn't understand ('%c', %d, '%s')\n",
                      letter, code, remaining);
      break;
    }
  }
  return remaining;
}

static void finish_machine_control(void *userdata) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  bring_path_to_halt(state);
}

static float euclid_distance(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

// Number of steps to accelerate or decelerate (negative "a") from speed
// v0 to speed v1. Modifies v1 if we can't reach the speed with the allocated
// number of steps.
static float steps_for_speed_change(float a, float v0, float *v1, int max_steps) {
  // s = v0 * t + a/2 * t^2
  // v1 = v0 + a*t
  const float t = (*v1 - v0) / a;
  // TODO:
  if (t < 0) fprintf(stderr, "Error condition: t=%.1f INSUFFICIENT LOOKAHEAD\n", t);
  float steps = a/2 * t*t + v0 * t;
  if (steps <= max_steps) return steps;
  // Ok, we would need more steps than we have available. We correct the speed to what
  // we actually can reach.
  *v1 = sqrtf(v0*v0 + 2 * a * max_steps);
  return max_steps;
}

static float acceleration_for_move(const GCodeMachineControl_t *state,
                                   const int *axis_steps,
                                   enum GCodeParserAxis defining_axis) {
  return state->max_axis_accel[defining_axis];
  // TODO: we need to scale the acceleration if one of the other axes could't
  //deal with it. Look at axis steps for that.
}

// Given that we want to travel "s" steps, start with speed "v0",
// accelerate peak speed v1 and slow down to "v2" with acceleration "a",
// what is v1 ?
static float get_peak_speed(float s, float v0, float v2, float a) {
  return sqrtf(v2*v2 + v0*v0 + 2 * a * s) / sqrtf(2);
}

// Speed relative to defining axis
static float get_speed_factor_for_axis(const struct AxisTarget *t,
                                       enum GCodeParserAxis request_axis) {
  if (t->delta_steps[t->defining_axis] == 0) return 0.0f;
  return 1.0f * t->delta_steps[request_axis] / t->delta_steps[t->defining_axis];
}

// Get the speed for a particular axis. Depending on the direction, this can
// be positive or negative.
static float get_speed_for_axis(const struct AxisTarget *target,
                                enum GCodeParserAxis request_axis) {
  return target->speed * get_speed_factor_for_axis(target, request_axis);
}

static char within_acceptable_range(float new, float old, float fraction) {
  const float max_diff = fraction * old;
  if (new < old - max_diff) return 0;
  if (new > old + max_diff) return 0;
  return 1;
}

// Determine the fraction of the speed that "from" should decelerate
// to at the end of its travel.
// The way trapezoidal moves work, be still have to decelerate to zero in
// most times, which is inconvenient. TODO(hzeller): speed matching is not
// cutting it :)
static float determine_joining_speed(const struct AxisTarget *from,
                                     const struct AxisTarget *to) {
  // Our goal is to figure out what our from defining speed should
  // be at the end of the move.
  char is_first = 1;
  float from_defining_speed = from->speed;
  for (int axis = 0; axis < GCODE_NUM_AXES; ++axis) {
    const int from_delta = from->delta_steps[axis];
    const int to_delta = to->delta_steps[axis];

    // Quick integer decisions
    if (from_delta == 0 && to_delta == 0) continue;   // uninteresting: no move.
    if (from_delta == 0 || to_delta == 0) return 0.0f; // accel from/to zero
    if ((from_delta < 0 && to_delta > 0) || (from_delta > 0 && to_delta < 0))
      return 0.0f;  // turing around

    float to_speed = get_speed_for_axis(to, axis);
    // What would this speed translated to our defining axis be ?
    float speed_conversion = 1.0f * from->delta_steps[from->defining_axis] / from->delta_steps[axis];
    float goal = to_speed * speed_conversion;
    if (goal < 0.0f) return 0.0f;
    if (is_first || within_acceptable_range(goal, from_defining_speed, 1e-5)) {
      if (goal < from_defining_speed) from_defining_speed = goal;
      is_first = 0;
    } else {
      return 0.0f;  // Too far off.
    }
  }
  return from_defining_speed;
}

// Assign steps to all the motors responsible for given axis.
static void assign_steps_to_motors(GCodeMachineControl_t *state,
                                   struct MotorMovement *command,
                                   enum GCodeParserAxis axis,
                                   int steps) {
  const DriverBitmap motormap_for_axis = state->axis_to_driver[axis];
  for (int motor = 0; motor < BEAGLEG_NUM_MOTORS; ++motor) {
    if (motormap_for_axis & (1 << motor)) {
      command->steps[motor] =
        state->axis_flip[axis] * state->driver_flip[motor] * steps;
    }
  }
}

// Returns true, if all results in zero movement
static uint8_t substract_steps(struct MotorMovement *value,
                               const struct MotorMovement *substract) {
  uint8_t has_nonzero = 0;
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    value->steps[i] -= substract->steps[i];
    has_nonzero |= (value->steps[i] != 0);
  }
  return has_nonzero;
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
static void move_machine_steps(GCodeMachineControl_t *state,
                               const struct AxisTarget *last_pos,
                               struct AxisTarget *target_pos,
                               const struct AxisTarget *upcoming) {
  if (target_pos->delta_steps[target_pos->defining_axis] == 0) {
    return;
  }
  struct MotorMovement accel_command = {0};
  struct MotorMovement move_command = {0};
  struct MotorMovement decel_command = {0};

  assert(target_pos->speed > 0);  // Speed is always a positive scalar.

  // Aux bits are set synchronously with what we need.
  move_command.aux_bits = target_pos->aux_bits;
  const enum GCodeParserAxis defining_axis = target_pos->defining_axis;

  // Common settings.
  memcpy(&accel_command, &move_command, sizeof(accel_command));
  memcpy(&decel_command, &move_command, sizeof(decel_command));

  move_command.v0 = target_pos->speed;
  move_command.v1 = target_pos->speed;

  // Let's see what our defining axis had as speed in the previous segment. The
  // last segment might have had a different defining axis, so we calculate
  // what the the fraction of the speed that our _current_ defining axis had.
  const float last_speed = fabsf(get_speed_for_axis(last_pos, defining_axis));

  // We need to arrive at a speed that the upcoming move does not have
  // to decelerate further (after all, it has a fixed feed-rate it should not
  // go over).
  float next_speed = determine_joining_speed(target_pos, upcoming);

  const int *axis_steps = target_pos->delta_steps;  // shortcut.
  const int abs_defining_axis_steps = abs(axis_steps[defining_axis]);
  const float a = acceleration_for_move(state, axis_steps, defining_axis);
  const float peak_speed = get_peak_speed(abs_defining_axis_steps,
                                          last_speed, next_speed, a);
  assert(peak_speed > 0);

  // TODO: if we only have < 5 steps or so, we should not even consider
  // accelerating or decelerating, but just do one speed.

  if (peak_speed < target_pos->speed) {
    target_pos->speed = peak_speed;  // Don't manage to accelerate to desired v
  }

  const float accel_fraction =
    (last_speed < target_pos->speed)
    ? steps_for_speed_change(a, last_speed, &target_pos->speed,
                             abs_defining_axis_steps) / abs_defining_axis_steps
    : 0;

  // We only decelerate if the upcoming speed is _slower_
  float dummy_next_speed = next_speed;  // Don't care to modify; we don't have
  const float decel_fraction =
    (next_speed < target_pos->speed)
    ? steps_for_speed_change(-a, target_pos->speed, &dummy_next_speed,
                             abs_defining_axis_steps) / abs_defining_axis_steps
    : 0;

  assert(accel_fraction + decel_fraction <= 1.0 + 1e-4);

#if 0
  // fudging: if we have tiny acceleration segments, don't do these at all
  // but only do speed; otherwise we have a lot of rattling due to many little
  // segments of acceleration/deceleration (think of drawing a circle).
  const float accel_decel_mm
    = (((accel_fraction + decel_fraction) * abs_defining_axis_steps)
       / state->cfg.steps_per_mm[defining_axis]);
  const char do_accel = (accel_decel_mm > 5);
#else
  const char do_accel = 1;
#endif

  char has_accel = 0;
  char has_move = 0;
  char has_decel = 0;

  if (do_accel && accel_fraction * abs_defining_axis_steps > 0) {
    has_accel = 1;
    accel_command.v0 = last_speed;           // Last speed of defining axis
    accel_command.v1 = target_pos->speed;    // New speed of defining axis

    // Now map axis steps to actual motor driver
    for (int i = 0; i < GCODE_NUM_AXES; ++i) {
      const int accel_steps = round2int(accel_fraction * axis_steps[i]);
      assign_steps_to_motors(state, &accel_command, i, accel_steps);
    }
  }

  if (do_accel && decel_fraction * abs_defining_axis_steps > 0) {
    has_decel = 1;
    decel_command.v0 = target_pos->speed;
    decel_command.v1 = next_speed;
    target_pos->speed = next_speed;

    // Now map axis steps to actual motor driver
    for (int i = 0; i < GCODE_NUM_AXES; ++i) {
      const int decel_steps = round2int(decel_fraction * axis_steps[i]);
      assign_steps_to_motors(state, &decel_command, i, decel_steps);
    }
  }

  // Move is everything that hasn't been covered in speed changes.
  // So we start with all steps and substract steps done in acceleration and
  // deceleration.
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    assign_steps_to_motors(state, &move_command, i, axis_steps[i]);
  }
  substract_steps(&move_command, &accel_command);
  has_move = substract_steps(&move_command, &decel_command);

  if (state->cfg.synchronous) {
    state->motor_ops->wait_queue_empty(state->motor_ops->user_data);
  }
  if (has_accel) state->motor_ops->enqueue(state->motor_ops->user_data,
                                           &accel_command, state->msg_stream);
  if (has_move) state->motor_ops->enqueue(state->motor_ops->user_data,
                                          &move_command, state->msg_stream);
  if (has_decel) state->motor_ops->enqueue(state->motor_ops->user_data,
                                           &decel_command, state->msg_stream);
}

// If we have enough data in the queue, issue motor move.
static void issue_motor_move_if_possible(GCodeMachineControl_t *state) {
  struct TargetBuffer *buffer = &state->buffer;
  if (buffer_available(buffer) >= 3) {
    move_machine_steps(state,
                       buffer_at(buffer, 0),  // Current established position.
                       buffer_at(buffer, 1),  // Position we want to move to.
                       buffer_at(buffer, 2)); // Next upcoming.
    buffer_next(buffer);
  }
}

static void machine_move(void *userdata, float feedrate, const float axis[]) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  struct TargetBuffer *buffer = &state->buffer;

  // We always have a previous position.
  struct AxisTarget *previous = buffer_get_last_written(buffer);
  struct AxisTarget *new_pos = buffer_add_next_target(buffer);
  int max_steps = -1;
  enum GCodeParserAxis defining_axis = AXIS_X;

  // Real world -> machine coordinates. Here, we are rounding to the next full
  // step, but we never accumulate the error, as we always use the absolute
  // position as reference.
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    new_pos->position_steps[i] = round2int(axis[i] * state->cfg.steps_per_mm[i]);
    new_pos->delta_steps[i] = new_pos->position_steps[i] - previous->position_steps[i];

    // The defining axis is the one that has to travel the most steps. It defines
    // the frequency to go.
    // All the other axes are doing a fraction of the defining axis.
    if (abs(new_pos->delta_steps[i]) > max_steps) {
      max_steps = abs(new_pos->delta_steps[i]);
      defining_axis = (enum GCodeParserAxis) i;
    }
  }
  new_pos->aux_bits = state->aux_bits;
  new_pos->defining_axis = defining_axis;

  // Now let's calculate the travel speed in steps/s on the defining axis.
  if (max_steps > 0) {
    float travel_speed = feedrate * state->cfg.steps_per_mm[defining_axis];

    // If we're in the euclidian space, choose the step-frequency according to
    // the relative feedrate of the defining axis.
    // (A straight 200mm/s should be the same as a diagnoal 200mm/s)
    if (defining_axis == AXIS_X || defining_axis == AXIS_Y || defining_axis == AXIS_Z) {
      // We need to calculate the feedrate in real-world coordinates as each
      // axis can have a different amount of steps/mm
      const float total_xyz_len_mm =
        euclid_distance(new_pos->delta_steps[AXIS_X] / state->cfg.steps_per_mm[AXIS_X],
                        new_pos->delta_steps[AXIS_Y] / state->cfg.steps_per_mm[AXIS_Y],
                        new_pos->delta_steps[AXIS_Z] / state->cfg.steps_per_mm[AXIS_Z]);
      const float steps_per_mm = state->cfg.steps_per_mm[defining_axis];
      const float defining_axis_len_mm = new_pos->delta_steps[defining_axis] / steps_per_mm;
      const float euclid_fraction = fabsf(defining_axis_len_mm) / total_xyz_len_mm;
      travel_speed *= euclid_fraction;
    }
    if (travel_speed > state->max_axis_speed[defining_axis]) {
      travel_speed = state->max_axis_speed[defining_axis];
    }
    new_pos->speed = travel_speed;
  } else {
    new_pos->speed = 0;
  }

  issue_motor_move_if_possible(state);
}

static void bring_path_to_halt(GCodeMachineControl_t *state) {
  // Enqueue a new position that is the same position as the last
  // one seen, but zero speed. That will allow the previous segment to
  // slow down. Enqueue.
  struct AxisTarget *previous = buffer_get_last_written(&state->buffer);
  struct AxisTarget *new_pos = buffer_add_next_target(&state->buffer);
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    new_pos->position_steps[i] = previous->position_steps[i];
    new_pos->delta_steps[i] = 0;
  }
  new_pos->defining_axis = AXIS_X;
  new_pos->speed = 0;
  new_pos->aux_bits = state->aux_bits;
  issue_motor_move_if_possible(state);
}

static void machine_G1(void *userdata, float feed, const float *axis) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  if (feed > 0) {
    state->current_feedrate_mm_per_sec = state->cfg.speed_factor * feed;
  }
  float feedrate = state->prog_speed_factor * state->current_feedrate_mm_per_sec;
  machine_move(userdata, feedrate, axis);
}

static void machine_G0(void *userdata, float feed, const float *axis) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  float rapid_feed = state->g0_feedrate_mm_per_sec;
  const float given = state->cfg.speed_factor * state->prog_speed_factor * feed;
  machine_move(userdata, given > 0 ? given : rapid_feed, axis);
}

static void machine_dwell(void *userdata, float value) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  bring_path_to_halt(state);
  state->motor_ops->wait_queue_empty(state->motor_ops->user_data);
  usleep((int) (value * 1000));
}

static void gcode_input_idle(void *userdata) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  bring_path_to_halt(state);
}

static void machine_set_speed_factor(void *userdata, float value) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  if (value < 0) {
    value = 1.0f + value;   // M220 S-10 interpreted as: 90%
  }
  if (value < 0.005) {
    if (state->msg_stream) fprintf(state->msg_stream,
				   "// M220: Not accepting speed "
				   "factors < 0.5%% (got %.1f%%)\n",
				   100.0f * value);
    return;
  }
  state->prog_speed_factor = value;
}

 static uint32_t get_endstop_gpio_descriptor(struct EndstopConfig config) {
   switch (config.endstop_number) {
   case 1: return END_1_GPIO;
   case 2: return END_2_GPIO;
   case 3: return END_3_GPIO;
   case 4: return END_4_GPIO;
   case 5: return END_5_GPIO;
   case 6: return END_6_GPIO;
   }
   return 0;
 }

// Get the endstop for the axis. If there are two axis configured for some
// reason, we prefer the one in the origin.
static uint32_t get_home_endstop(GCodeMachineControl_t *state,
				 enum GCodeParserAxis axis,
				 int *dir, int *trigger_value) {
  // Just extract one endstop. We prefer the lower one if we find it.
  *dir = 1;
  struct EndstopConfig config = state->max_endstop[axis];
  if (state->min_endstop[axis].endstop_number
      && state->min_endstop[axis].homing_use) {
    *dir = -1;
    config = state->min_endstop[axis];
  }
  if (!config.homing_use)
    return 0;
  *trigger_value = config.trigger_value;
  return get_endstop_gpio_descriptor(config);
}

#if 0
static uint32_t get_travel_endstop(GCodeMachineControl_t *state,
				   enum GCodeParserAxis axis,
				   int *dir, int *polarity) {
  struct EndstopConfig config = state->travel_endstop[axis];
  uint32_t gpio_def = 0;
  *dir = (config.direction) ? 1 : -1;
  *polarity = config.polarity;
  switch (config.endstop_number) {
  case 1: gpio_def = END_1_GPIO; break;
  case 2: gpio_def = END_2_GPIO; break;
  case 3: gpio_def = END_3_GPIO; break;
  case 4: gpio_def = END_4_GPIO; break;
  case 5: gpio_def = END_5_GPIO; break;
  case 6: gpio_def = END_6_GPIO; break;
  default: break;
  }
  return gpio_def;
}
#endif

static void move_to_endstop(GCodeMachineControl_t *state,
			    enum GCodeParserAxis axis,
			    float feedrate, int backoff,
			    int dir, int trigger_value, uint32_t gpio_def) {
  struct MotorMovement move_command = {0};
  const float steps_per_mm = state->cfg.steps_per_mm[axis];
  float target_speed = feedrate * steps_per_mm;
  if (target_speed > state->max_axis_speed[axis]) {
    target_speed = state->max_axis_speed[axis];
  }

  move_command.v0 = 0;
  move_command.v1 = target_speed;

  // move axis until endstop is hit
  assign_steps_to_motors(state, &move_command, axis, 0.5 * steps_per_mm * dir);
  while (get_gpio(gpio_def) != trigger_value) {
    state->motor_ops->enqueue(state->motor_ops->user_data,
                              &move_command, state->msg_stream);
    state->motor_ops->wait_queue_empty(state->motor_ops->user_data);
    // TODO: possibly acceleration over multiple segments.
    move_command.v0 = move_command.v1;
  }

  if (backoff) {
    // move axis off endstop
    assign_steps_to_motors(state, &move_command, axis, 0.1 * steps_per_mm * -dir);
    while (get_gpio(gpio_def) == trigger_value) {
      state->motor_ops->enqueue(state->motor_ops->user_data,
                                &move_command, state->msg_stream);
      state->motor_ops->wait_queue_empty(state->motor_ops->user_data);
    }
  }
}

static void home_axis(GCodeMachineControl_t *state, enum GCodeParserAxis axis) {
  const struct MachineControlConfig *cfg = &state->cfg;
  struct AxisTarget *last = buffer_get_last_written(&state->buffer);
  float home_pos = 0; // assume HOME_POS_ORIGIN
  int dir;
  int trigger_value;
  uint32_t gpio_def = get_home_endstop(state, axis, &dir, &trigger_value);
  if (!gpio_def)
    return;
  move_to_endstop(state, axis, 15, 1, dir, trigger_value, gpio_def);
  home_pos = (dir < 0) ? 0 : cfg->move_range_mm[axis];
  last->position_steps[axis] = round2int(home_pos * cfg->steps_per_mm[axis]);
}

static void machine_home(void *userdata, AxisBitmap_t axes_bitmap) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  const struct MachineControlConfig *cfg = &state->cfg;
  bring_path_to_halt(state);
  for (const char *order = cfg->home_order; *order; order++) {
    const enum GCodeParserAxis axis = gcodep_letter2axis(*order);
    if (axis == GCODE_NUM_AXES || !(axes_bitmap & (1 << axis)))
      continue;
    home_axis(state, axis);
  }
  motors_enable(state, 0);
}

// todo: currently disabled.
static int machine_probe(void *userdata, float feedrate,
			 enum GCodeParserAxis axis) {
  GCodeMachineControl_t *state = (GCodeMachineControl_t*)userdata;
  int dir;
  int polarity;
  uint32_t gpio_def = 0; // get_travel_endstop(state, axis, &dir, &polarity);
  if (gpio_def) {
    if (feedrate <= 0)
      feedrate = 20;
    move_to_endstop(state, axis, feedrate, 0, dir, polarity, gpio_def);
    // FIXME: should the last position be updated?
    return 1;
  }
  fprintf(state->msg_stream,
	  "// BeagleG: No probe - axis %c does not have a travel endstop\n",
	  gcodep_axis2letter(axis));
  return 0;
}

// Cleanup whatever is allocated. Return NULL for convenience.
static GCodeMachineControl_t *cleanup_state(GCodeMachineControl_t *object) {
  free(object);
  return NULL;
}

GCodeMachineControl_t *gcode_machine_control_new(const struct MachineControlConfig *config_in,
                                                 struct MotorOperations *motor_ops,
                                                 FILE *msg_stream) {
  GCodeMachineControl_t *result;
  // Initialize basic state and derived configuration.
  result = (GCodeMachineControl_t*) malloc(sizeof(GCodeMachineControl_t));
  bzero(result, sizeof(*result));
  result->motor_ops = motor_ops;
  result->msg_stream = msg_stream;

  // Always keep the steps_per_mm positive, but extract direction for
  // final assignment to motor.
  struct MachineControlConfig cfg = *config_in;
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    result->axis_flip[i] = cfg.steps_per_mm[i] < 0 ? -1 : 1;
    cfg.steps_per_mm[i] = fabsf(cfg.steps_per_mm[i]);
    if (cfg.max_feedrate[i] < 0) {
      fprintf(stderr, "Invalid negative feedrate %.1f for axis %c\n",
              cfg.max_feedrate[i], gcodep_axis2letter(i));
      return cleanup_state(result);
    }
    if (cfg.acceleration[i] < 0) {
      fprintf(stderr, "Invalid negative acceleration %.1f for axis %c\n",
              cfg.acceleration[i], gcodep_axis2letter(i));
      return cleanup_state(result);
    }
  }

  // Here we assign it to the 'const' cfg, all other accesses will check for
  // the readonly ness. So some nasty override here: we know what we're doing.
  *((struct MachineControlConfig*) &result->cfg) = cfg;
  result->current_feedrate_mm_per_sec = cfg.max_feedrate[AXIS_X] / 10;
  float lowest_accel = cfg.max_feedrate[AXIS_X] * cfg.steps_per_mm[AXIS_X];
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (cfg.max_feedrate[i] > result->g0_feedrate_mm_per_sec) {
      result->g0_feedrate_mm_per_sec = cfg.max_feedrate[i];
    }
    result->max_axis_speed[i] = cfg.max_feedrate[i] * cfg.steps_per_mm[i];
    const float accel = cfg.acceleration[i] * cfg.steps_per_mm[i];
    result->max_axis_accel[i] = accel;
    if (accel > result->highest_accel)
      result->highest_accel = accel;
    if (accel < lowest_accel)
      lowest_accel = accel;
  }
  result->prog_speed_factor = 1.0f;

  // Mapping axes to physical motors. We might have a larger set of logical
  // axes of which we map a subset to actual motors.
  // We do this in two steps: One identifies which io-pin actually goes to which
  // physical location (a property of the actual cape), the second maps
  // logical axes (e.g. 'X') to the location on the board.
  // This double mapping is done, so that it is intuitive for users to map
  // (as the first is a hardware property that doesn't really change and the
  // second the mapping the user wants).

  // Mapping of connector position on cape to driver ID (the axis in the
  // motor interface). This might differ due to physical board layout reasons.
  int pos_to_driver[BEAGLEG_NUM_MOTORS];
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    pos_to_driver[i] = -1;
  }
  const char *physical_mapping = cfg.channel_layout;
  if (physical_mapping == NULL) physical_mapping = kChannelLayout;
  if (strlen(physical_mapping) > BEAGLEG_NUM_MOTORS) {
    fprintf(stderr, "Physical mapping string longer than available motors. "
            "('%s', max axes=%d)\n", physical_mapping, BEAGLEG_NUM_MOTORS);
    return cleanup_state(result);
  }
  for (int pos = 0; *physical_mapping; pos++, physical_mapping++) {
    const int mapped_driver = *physical_mapping - '0';
    if (mapped_driver >= 0 && mapped_driver < BEAGLEG_NUM_MOTORS) {
      pos_to_driver[pos] = mapped_driver;
    }
    else {
      fprintf(stderr, "Invalid character '%c' in channel-layout mapping. "
              "Can be characters '0'..'%d'\n",
              *physical_mapping, BEAGLEG_NUM_MOTORS - 1);
      return cleanup_state(result);
    }
  }

  const char *axis_map = cfg.axis_mapping;
  if (axis_map == NULL) axis_map = kAxisMapping;
  for (int pos = 0; *axis_map; pos++, axis_map++) {
    if (pos >= BEAGLEG_NUM_MOTORS || pos_to_driver[pos] < 0) {
      fprintf(stderr, "Axis mapping string has more elements than available %d "
              "connectors (remaining=\"..%s\").\n", pos, axis_map);
      return cleanup_state(result);
    }
    if (*axis_map == '_')
      continue;
    const enum GCodeParserAxis axis = gcodep_letter2axis(*axis_map);
    if (axis == GCODE_NUM_AXES) {
      fprintf(stderr,
              "Illegal axis->connector mapping character '%c' in '%s' "
              "(Only valid axis letter or '_' to skip a connector).\n",
              toupper(*axis_map), cfg.axis_mapping);
      return cleanup_state(result);
    }
    const int driver = pos_to_driver[pos];
    result->driver_flip[driver] = (tolower(*axis_map) == *axis_map) ? -1 : 1;
    result->axis_to_driver[axis] |= (1 << driver);
  }

  // Extract enstop polarity
  char endstop_trigger[NUM_ENDSTOPS] = {0};
  if (cfg.endswitch_polarity) {
    const char *map = cfg.endswitch_polarity;
    for (int switch_connector = 0; *map; switch_connector++, map++) {
      if (*map == '_' || *map == '0' || *map == '-') {
        endstop_trigger[switch_connector] = 0;
      } else if (*map == '1' || *map == '+') {
        endstop_trigger[switch_connector] = 1;
      } else {
        fprintf(stderr, "Illegal endswitch polarity character '%c' in '%s'.\n",
                *map, cfg.endswitch_polarity);
        return cleanup_state(result);
      }
    }
  }

  // Now map the endstops. String position is position on the switch connector
  if (cfg.min_endswitch) {
    const char *map = cfg.min_endswitch;
    for (int switch_connector = 0; *map; switch_connector++, map++) {
      if (*map == '_')
        continue;
      const enum GCodeParserAxis axis = gcodep_letter2axis(*map);
      if (axis == GCODE_NUM_AXES) {
        fprintf(stderr,
                "Illegal axis->min_endswitch mapping character '%c' in '%s' "
                "(Only valid axis letter or '_' to skip a connector).\n",
                toupper(*map), cfg.min_endswitch);
        return cleanup_state(result);
      }
      result->min_endstop[axis].endstop_number = switch_connector + 1;
      result->min_endstop[axis].homing_use = (toupper(*map) == *map) ? 1 : 0;
      result->min_endstop[axis].trigger_value = endstop_trigger[switch_connector];
    }
  }

  if (cfg.max_endswitch) {
    const char *map = cfg.max_endswitch;
    for (int switch_connector = 0; *map; switch_connector++, map++) {
      if (*map == '_')
        continue;
      const enum GCodeParserAxis axis = gcodep_letter2axis(*map);
      if (axis == GCODE_NUM_AXES) {
        fprintf(stderr,
                "Illegal axis->min_endswitch mapping character '%c' in '%s' "
                "(Only valid axis letter or '_' to skip a connector).\n",
                toupper(*map), cfg.min_endswitch);
        return cleanup_state(result);
      }
      result->max_endstop[axis].endstop_number = switch_connector + 1;
      result->max_endstop[axis].homing_use = (toupper(*map) == *map) ? 1 : 0;
      result->max_endstop[axis].trigger_value = endstop_trigger[switch_connector];
    }
  }

  // Now let's see what motors are mapped to any useful output.
  if (result->cfg.debug_print) fprintf(stderr, "-- Config --\n");
  int error_count = 0;
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (result->axis_to_driver[i] == 0)
      continue;
    char is_error = (result->cfg.steps_per_mm[i] <= 0
                     || result->cfg.max_feedrate[i] <= 0);
    if (result->cfg.debug_print || is_error) {
      fprintf(stderr, "%c axis: %5.1fmm/s, %7.1fmm/s^2, %8.4f steps/mm%s ",
              gcodep_axis2letter(i), result->cfg.max_feedrate[i],
              result->cfg.acceleration[i],
              result->cfg.steps_per_mm[i],
              result->axis_flip[i] < 0 ? " (reversed)" : "");
      int endstop = result->min_endstop[i].endstop_number;
      const char *trg = result->min_endstop[i].trigger_value ? "hi" : "lo";
      if (endstop) {
        fprintf(stderr, "min-switch %d (%s-trigger)%s; ",
		endstop, trg,
                result->min_endstop[i].homing_use ? " [HOME]" : "       ");
      }
      endstop = result->max_endstop[i].endstop_number;
      trg = result->max_endstop[i].trigger_value ? "hi" : "lo";
      if (endstop) {
        fprintf(stderr, "max-switch %d (%s-trigger)%s;",
		endstop, trg,
                result->max_endstop[i].homing_use ? " [HOME]" : "");
      }
      fprintf(stderr, "\n");
    }
    if (is_error) {
      fprintf(stderr, "\tERROR: that is an invalid feedrate or steps/mm.\n");
      ++error_count;
    }
  }
  if (error_count)
    return cleanup_state(result);

  target_buffer_init(&result->buffer);

  // Initial position.
  struct AxisTarget *init_axis = buffer_add_next_target(&result->buffer);
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    init_axis->position_steps[i] = 0;
  }
  init_axis->speed = 0;

  result->event_input.user_data = result;
  result->event_input.coordinated_move = &machine_G1;
  result->event_input.rapid_move = &machine_G0;
  result->event_input.go_home = &machine_home;
  result->event_input.probe_axis = &machine_probe;
  result->event_input.dwell = &machine_dwell;
  result->event_input.set_speed_factor = &machine_set_speed_factor;
  result->event_input.motors_enable = &motors_enable;
  result->event_input.unprocessed = &special_commands;
  result->event_input.gcode_command_done = &gcode_send_ok;
  result->event_input.input_idle = &gcode_input_idle;

  // Lifecycle
  result->event_input.gcode_finished = &finish_machine_control;

  // Not yet implemented
  result->event_input.set_fanspeed = &dummy_set_fanspeed;
  result->event_input.set_temperature = &dummy_set_temperature;
  result->event_input.wait_temperature = &dummy_wait_temperature;

  return result;
}

struct GCodeParserCb *gcode_machine_control_event_receiver(GCodeMachineControl_t *object) {
  return &object->event_input;
}

void gcode_machine_control_delete(GCodeMachineControl_t *object) {
  cleanup_state(object);
}

void gcode_machine_control_default_config(struct MachineControlConfig *config) {
  bzero(config, sizeof(*config));
  memcpy(config->steps_per_mm, kStepsPerMM, sizeof(config->steps_per_mm));
  memcpy(config->move_range_mm, kMoveRange, sizeof(config->move_range_mm));
  memcpy(config->max_feedrate, kMaxFeedrate, sizeof(config->max_feedrate));
  memcpy(config->acceleration, kDefaultAccel, sizeof(config->acceleration));
  config->speed_factor = 1;
  config->debug_print = 0;
  config->synchronous = 0;
  config->channel_layout = kChannelLayout;
  config->axis_mapping = kAxisMapping;
  config->home_order = kHomeOrder;
}

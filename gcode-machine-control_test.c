/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for gcode machine control.
 *
 * We simulate inputs coming from the GCode parser and expect that
 * we get a certain sequence of MotorMovment commands.
 *
 * Very simple right now, I should probably build some testing
 * framework.
 */
#include "gcode-machine-control.h"

#include "motor-operations.h"
#include "gcode-parser.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define END_SENTINEL 0x42

// Set up config that they are the same for all the tests.
static void init_test_config(struct MachineControlConfig *c) {
  gcode_machine_control_default_config(c);
  for (int i = 0; i < GCODE_NUM_AXES; ++i)
    c->steps_per_mm[i] = 100;  // step/mm
  for (int i = 0; i < GCODE_NUM_AXES; ++i)
    c->acceleration[i] = 1000;  // mm/s^2
  for (int i = 0; i < GCODE_NUM_AXES; ++i)
    c->max_feedrate[i] = 10000;
  c->channel_layout = "01234567";
  c->axis_mapping   = "XYZEABCU";
}

// Helpers to compare and print MotorMovements.
static void print_movement(const char *msg, const struct MotorMovement *m) {
  printf("%s: v0=%.1f -> v1=%.1f {%d, %d, %d}", msg,
         m->v0, m->v1,
         m->steps[0], m->steps[1], m->steps[2]);
}

static void expect_movement_eq(const struct MotorMovement *expected,
			       const struct MotorMovement *reality,
                               int seg_number) {
  if (memcmp(expected, reality, sizeof(*expected)) != 0) {
    printf("segment #%d: ", seg_number);
    print_movement("expected", expected);
    print_movement("; but was", reality);
    printf("\n");
    assert(0);
  }
}


// Internal state of a test.
struct ExpectState {
  const struct MotorMovement *expect;
  const struct MotorMovement *current;
};

// Implementation of MotorOperations. This uses the "struct ExpectState" as
// user data keeping track of the MotorMovement sequence we expect.
static void test_motor_enable(void *user, char on) { /* ign */}

// Enqueue tests if we are called in the same sequence our we get in ExpectState
static int test_enqueue(void *user, const struct MotorMovement *param, FILE *f) {
  struct ExpectState *state = (struct ExpectState*) user;
  assert(state->current->aux_bits != END_SENTINEL);
  const int number = (int)(state->current - state->expect);
  expect_movement_eq(state->current, param, number);
  ++state->current;
  return 0;
}

static void init_expect_motor(struct MotorOperations *ops,
			      struct ExpectState *state) {
  state->current = state->expect;
  ops->user_data = state;
  ops->motor_enable = &test_motor_enable;
  ops->enqueue = &test_enqueue;
}

/*
 * TODO(hzeller): there is a lot of boilderplate in each of these test-cases.
 * make that a bit more common.
 */

// If we get two line segments that are straigt and don't change speed, we
// expect no slow-down.
void test_straight_segments_same_speed() {
  printf("%s\n", __func__);
  struct MachineControlConfig config;
  init_test_config(&config);
  static const struct MotorMovement expected[] = {
    { .v0 =     0.0, .v1 = 10000.0, .steps = { 500}},  // accel
    { .v0 = 10000.0, .v1 = 10000.0, .steps = {9500}},  // 1st move @100mm/s
    { .v0 = 10000.0, .v1 = 10000.0, .steps = {9500}},  // 2nd move @100mm/s
    { .v0 = 10000.0, .v1 =     0.0, .steps = { 500}},  // decel back to 0
    { .aux_bits = END_SENTINEL},
  };
  struct ExpectState state = { .expect = expected };
  struct MotorOperations expect_motor;
  init_expect_motor(&expect_motor, &state);
  GCodeMachineControl_t *obj = gcode_machine_control_new(&config,
                                                         &expect_motor,
                                                         NULL);
  struct GCodeParserCb *call = gcode_machine_control_event_receiver(obj);
  
  // Move to pos 100, then 200, first with speed 100, then speed 50
  float coordinates[GCODE_NUM_AXES] = {0};
  coordinates[0] = 100;
  call->coordinated_move(call->user_data, 100, coordinates);

  // second half, same speed
  coordinates[0] = 200;
  call->coordinated_move(call->user_data, 100, coordinates);

  // stay at that place, so we expect to decelerate to 0.
  call->coordinated_move(call->user_data, 50, coordinates);

  gcode_machine_control_delete(obj);
  assert(state.current->aux_bits == END_SENTINEL);
  printf("DONE %s\n", __func__);
}

// If we have two line segments in a straight line and one is slower than the
// other, slow down the first segment at the end to the travel speed of the
// next segment.
void test_straight_segments_speed_change() {
  printf("%s\n", __func__);
  struct MachineControlConfig config;
  init_test_config(&config);
  static const struct MotorMovement expected[] = {
    { .v0 =     0.0, .v1 = 10000.0, .steps = { 500}},  // accel
    { .v0 = 10000.0, .v1 = 10000.0, .steps = {9125}},  // move @100mm/s
    { .v0 = 10000.0, .v1 =  5000.0, .steps = { 375}},  // slow: match next speed
    { .v0 =  5000.0, .v1 =  5000.0, .steps = {9875}},  // move @50mm/s
    { .v0 =  5000.0, .v1 =     0.0, .steps = { 125}},  // final slow to zero
    { .aux_bits = END_SENTINEL},
  };
  struct ExpectState state = { .expect = expected };
  struct MotorOperations expect_motor;
  init_expect_motor(&expect_motor, &state);
  GCodeMachineControl_t *obj = gcode_machine_control_new(&config,
                                                         &expect_motor,
                                                         NULL);
  struct GCodeParserCb *call = gcode_machine_control_event_receiver(obj);
  
  // Move to pos 100, then 200, first with speed 100, then speed 50
  float coordinates[GCODE_NUM_AXES] = {0};
  coordinates[0] = 100;
  call->coordinated_move(call->user_data, 100, coordinates);

  // second half, less speed.
  coordinates[0] = 200;
  call->coordinated_move(call->user_data, 50, coordinates);

  // stay at that place, so we expect to decelerate to 0.
  call->coordinated_move(call->user_data, 50, coordinates);

  gcode_machine_control_delete(obj);
  assert(state.current->aux_bits == END_SENTINEL);
  printf("DONE %s\n", __func__);
}

int main() {
  printf("RUN (%s)\n", __FILE__);

  test_straight_segments_same_speed();
  test_straight_segments_speed_change();

  printf("PASS (%s)\n", __FILE__);
}

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
  c->axis_mapping   = "XYZEABCU";
}

// Helpers to compare and print MotorMovements.
static void print_movement(const char *msg, const struct MotorMovement *m) {
  printf("%s: v0=%.1f -> v1=%.1f {%d, %d, %d}", msg, m->v0, m->v1,
         m->steps[0], m->steps[1], m->steps[2]);
}

static void expect_movement_eq(const struct MotorMovement *expected,
			       const struct MotorMovement *reality,
                               int info_segment_number) {
  if (memcmp(expected, reality, sizeof(*expected)) != 0) {
    printf("segment #%d: ", info_segment_number);
    print_movement("expected", expected);
    print_movement("; but was", reality);
    printf("\n");
    assert(0);
  }
}

// Internal state of a test sequence.
struct ExpectState {
  const struct MotorMovement *expect;
  const struct MotorMovement *current;
};

// Implementation of Mock MotorOperations. This uses the "struct ExpectState" as
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

struct Harness {
  struct MachineControlConfig config;
  struct ExpectState expect_state;
  struct MotorOperations expect_motor_ops;
  GCodeMachineControl_t *object;
};

// Initialize harness with the expected sequence of motor movements
// and return the callback struct to receive simulated gcode calls.
void init_harness(struct Harness *harness,
                  const struct MotorMovement *expected,
                  struct GCodeParserCb *callbacks) {
  init_test_config(&harness->config);
  harness->expect_state.expect = expected;
  init_expect_motor(&harness->expect_motor_ops,
                    &harness->expect_state);
  harness->object = gcode_machine_control_new(&harness->config,
                                              &harness->expect_motor_ops, NULL);
  gcode_machine_control_init_callbacks(harness->object, callbacks);
}

void finish_harness(struct Harness *harness) {
  gcode_machine_control_delete(harness->object);
  // Did we walk through all states ?
  assert(harness->expect_state.current->aux_bits == END_SENTINEL);
}

// If we get two line segments that are straight and don't change speed, we
// expect no slow-down.
void TEST_straight_segments_same_speed() {
  printf(" %s\n", __func__);
  struct Harness harness;
  static const struct MotorMovement expected[] = {
    { .v0 =     0.0, .v1 = 10000.0, .steps = { 500}},  // accel
    { .v0 = 10000.0, .v1 = 10000.0, .steps = {9500}},  // 1st move @100mm/s
    { .v0 = 10000.0, .v1 = 10000.0, .steps = {9500}},  // 2nd move @100mm/s
    { .v0 = 10000.0, .v1 =     0.0, .steps = { 500}},  // decel back to 0
    { .aux_bits = END_SENTINEL},
  };
  struct GCodeParserCb call;
  init_harness(&harness, expected, &call);

  // Move to pos 100, then 200, first with speed 100, then speed 50
  float coordinates[GCODE_NUM_AXES] = {0};
  coordinates[0] = 100;
  call.coordinated_move(call.user_data, 100, coordinates);  // 100mm/s

  // second half, same speed
  coordinates[0] = 200;
  call.coordinated_move(call.user_data, 100, coordinates);  // also 100mm/s

  call.motors_enable(call.user_data, 0);  // finish movement.

  finish_harness(&harness);
  printf(" DONE %s\n", __func__);
}

// If we have two line segments in a straight line and one is slower than the
// other, slow down the first segment at the end to the travel speed of the
// next segment.
void TEST_straight_segments_speed_change() {
  printf(" %s\n", __func__);
  struct Harness harness;
  static const struct MotorMovement expected[] = {
    { .v0 =     0.0, .v1 = 10000.0, .steps = { 500}},  // accel
    { .v0 = 10000.0, .v1 = 10000.0, .steps = {9125}},  // move @100mm/s
    { .v0 = 10000.0, .v1 =  5000.0, .steps = { 375}},  // slow: match next speed
    { .v0 =  5000.0, .v1 =  5000.0, .steps = {9875}},  // move @50mm/s
    { .v0 =  5000.0, .v1 =     0.0, .steps = { 125}},  // final slow to zero
    { .aux_bits = END_SENTINEL},
  };
  struct GCodeParserCb call;
  init_harness(&harness, expected, &call);

  // Move to pos 100, then 200, first with speed 100, then speed 50
  float coordinates[GCODE_NUM_AXES] = {0};
  coordinates[0] = 100;
  call.coordinated_move(call.user_data, 100, coordinates); // 100mm/s

  // second half, less speed.
  coordinates[0] = 200;
  call.coordinated_move(call.user_data, 50, coordinates);  // 50mm/s

  call.motors_enable(call.user_data, 0);  // finish movement.

  finish_harness(&harness);
  printf(" DONE %s\n", __func__);
}

int main() {
  printf("RUN (%s)\n", __FILE__);

  TEST_straight_segments_same_speed();
  TEST_straight_segments_speed_change();

  printf("PASS (%s)\n", __FILE__);
}

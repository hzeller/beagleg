/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for gcode parser.
 */
#include "gcode-parser.h"

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <strings.h>

// 'home' position of our simulated machine. Arbitrary values.
#define HOME_X 123
#define HOME_Y 456
#define HOME_Z 789
#define PROBE_POSITION 42

// In our mock implementation, we keep counters for each call.
enum {
  CALL_gcode_start,
  CALL_gcode_finished,
  CALL_go_home,
  CALL_probe_axis,
  CALL_motors_enable,
  CALL_coordinated_move,
  CALL_rapid_move,
  CALL_unprocessed,
  NUM_COUNTED_CALLS,
};
struct CallCounter {
  GCodeParser_t *parser;
  int call_count[NUM_COUNTED_CALLS];
  AxesRegister abs_pos;         // last coordinates we got from a move.
  AxesRegister parser_offset;   // current offset in the parser.
};

// Parses gcode block and updates counters. Possibly creates a parser first.
static void GCodeTestParseLine(const char *block, struct CallCounter *counter);

// Shutdown counter (mostly deleting a parser that had been created).
static void ShutdownCounter(struct CallCounter *counter);

static char ExpectEqual(int a, int b) {
  if (a != b) {
    fprintf(stderr, "Expected %d == %d\n", a, b);
    return 0;
  }
  return 1;
}

static void TEST_axes_letter_conversion() {
  printf(" %s\n", __func__);
  // one way ..
  assert('X' == gcodep_axis2letter(AXIS_X));
  assert('A' == gcodep_axis2letter(AXIS_A));
  // .. and back
  assert(AXIS_X == gcodep_letter2axis('X'));
  assert(AXIS_X == gcodep_letter2axis('x'));
  assert(AXIS_Z == gcodep_letter2axis('z'));
  assert(AXIS_A == gcodep_letter2axis('A'));
  assert(AXIS_A == gcodep_letter2axis('a'));
  printf(" DONE %s\n", __func__);
}

static void TEST_simple_move() {
  printf(" %s\n", __func__);
  struct CallCounter counter;
  bzero(&counter, sizeof(counter));
  GCodeTestParseLine("G1 X100", &counter);
  assert(counter.call_count[CALL_coordinated_move] == 1);

  // If we move one axis, the others are still at the origin.
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 100));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y));
  assert(ExpectEqual(counter.abs_pos[AXIS_Z], HOME_Z));

  GCodeTestParseLine("G1 X10 Y10 Z-20", &counter);
  assert(counter.call_count[CALL_coordinated_move] == 2);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 10));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 10));
  assert(ExpectEqual(counter.abs_pos[AXIS_Z], HOME_Z - 20));

  ShutdownCounter(&counter);
  printf(" DONE %s\n", __func__);
}

static void TEST_absolute_relative() {
  printf(" %s\n", __func__);
  struct CallCounter counter;
  bzero(&counter, sizeof(counter));

  GCodeTestParseLine("G90", &counter);   // absolute mode.

  GCodeTestParseLine("G1 X10 Y11 Z12", &counter);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 10));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 11));
  assert(ExpectEqual(counter.abs_pos[AXIS_Z], HOME_Z + 12));

  // Another absolute position.
  GCodeTestParseLine("G1 X20 Y21 Z22", &counter);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 20));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 21));
  assert(ExpectEqual(counter.abs_pos[AXIS_Z], HOME_Z + 22));

  GCodeTestParseLine("G91", &counter);   // Now, go in relative mode.

  GCodeTestParseLine("G1 X5 Y6 Z7", &counter);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 20 + 5));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 21 + 6));
  assert(ExpectEqual(counter.abs_pos[AXIS_Z], HOME_Z + 22 + 7));

  ShutdownCounter(&counter);
  printf(" DONE %s\n", __func__);
}

static void TEST_set_origin_G92() {
  printf(" %s\n", __func__);
  struct CallCounter counter;
  bzero(&counter, sizeof(counter));
  GCodeTestParseLine("G1 X100 Y100", &counter);  // Some position.

  assert(ExpectEqual(counter.parser_offset[AXIS_X], HOME_X));
  assert(ExpectEqual(counter.parser_offset[AXIS_Y], HOME_Y));
  assert(ExpectEqual(counter.parser_offset[AXIS_Z], HOME_Z));

  GCodeTestParseLine("G92 X5 Y7", &counter);     // Tool left bottom of it.

  // New offset within machine cube.
  assert(ExpectEqual(counter.parser_offset[AXIS_X], HOME_X + 100 - 5));
  assert(ExpectEqual(counter.parser_offset[AXIS_Y], HOME_Y + 100 - 7));

  GCodeTestParseLine("G1 X12 Y17", &counter);    // Move relative to that

  // Final position.
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 100 - 5 + 12));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17));

  GCodeTestParseLine("G92 X3 Y4", &counter);  // Set new origin relative to here
  GCodeTestParseLine("G1 X1 Y1", &counter);

  // Now, we are relative to _that_
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 100 - 5 + 12 - 3 + 1));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17 - 4 + 1));

  GCodeTestParseLine("G92.2", &counter);      // Suspend.
  GCodeTestParseLine("G1 X1 Y1", &counter);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 1));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 1));

  // Only set one axis now, that way we see that the new axis is modified
  // and the original axis is pulled out of the G92 store again.
  GCodeTestParseLine("G92 X0", &counter);     // Set again. Modify Current G92.
  GCodeTestParseLine("G1 X1 Y1", &counter);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 1 + 1));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17 - 4 + 1));

  GCodeTestParseLine("G92.2", &counter);      // Suspend.
  GCodeTestParseLine("G1 X1 Y1", &counter);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 1));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 1));

  GCodeTestParseLine("G92.3", &counter);      // Restore.
  GCodeTestParseLine("G1 X7 Y8", &counter);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 1 + 7));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17 - 4 + 8));

  GCodeTestParseLine("G92.1", &counter);      // Reset. Back relative to machine.
  GCodeTestParseLine("G1 X1 Y1", &counter);
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 1));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 1));

  // Later, once that is implemented: test with a G54. It is not clear yet
  // if G92 works relative to the active coordinate system (G54, G55..), or
  // if it is an absolute offset at the time it was established.
  ShutdownCounter(&counter);
  printf(" DONE %s\n", __func__);
}

static void TEST_probe_axis() {
  printf(" %s\n", __func__);
  struct CallCounter counter;
  bzero(&counter, sizeof(counter));
  GCodeTestParseLine("G30 Z3", &counter);  // 3: thickness of our probe

  assert(ExpectEqual(counter.parser_offset[AXIS_Z], PROBE_POSITION - 3));

  GCodeTestParseLine("G1 Z1", &counter);  // Move Z axis hovering 1 over it
  assert(ExpectEqual(counter.abs_pos[AXIS_Z], PROBE_POSITION - 3 + 1));

  ShutdownCounter(&counter);
  printf(" DONE %s\n", __func__);
}

int main() {
  printf("RUN (%s)\n", __FILE__);

  TEST_axes_letter_conversion();
  TEST_simple_move();
  TEST_absolute_relative();
  TEST_set_origin_G92();
  TEST_probe_axis();

  printf("PASS (%s)\n", __FILE__);
}

// supporting mock counter functions.
static void do_count(void *p, int what) {
  ((struct CallCounter*)p)->call_count[what]++;
}
static void do_copy_register(void *p, const float axes[]) {
  memcpy(((struct CallCounter*)p)->abs_pos, axes, sizeof(AxesRegister));
}

static void swallow_scalar(void *p, float dont_care) {}

static void gcode_start(void *p) { do_count(p, CALL_gcode_start); }
static void gcode_finished(void *p) { do_count(p, CALL_gcode_finished); }

static void inform_origin_offset(void *p, const float *offset) {
  memcpy(((struct CallCounter*)p)->parser_offset, offset, sizeof(AxesRegister));
}

static void go_home(void *p, AxisBitmap_t b) { do_count(p, CALL_go_home); }
static char probe_axis(void *p, float feed_mm_p_sec, enum GCodeParserAxis axis,
                       float *probed_position) {
  do_count(p, CALL_probe_axis);
  *probed_position = PROBE_POSITION;
  return 1;
}

static void motors_enable(void *p, char b) { do_count(p, CALL_motors_enable); }
static char coordinated_move(void *p, float speed, const float *axes) {
  do_count(p, CALL_coordinated_move);
  do_copy_register(p, axes);
  return 1;
}
static char rapid_move(void *p, float speed, const float *axes) {
  do_count(p, CALL_rapid_move);
  do_copy_register(p, axes);
  return 1;
}

static void GCodeTestParseLine(const char *block, struct CallCounter *counter) {
  if (counter->parser == NULL) {
    struct GCodeParserConfig config;
    bzero(&config, sizeof(config));
    config.callbacks.gcode_start = &gcode_start;
    config.callbacks.gcode_finished = &gcode_finished;
    config.callbacks.inform_origin_offset = &inform_origin_offset;
    config.callbacks.go_home = &go_home;
    config.callbacks.probe_axis = &probe_axis;
    config.callbacks.motors_enable = &motors_enable;
    config.callbacks.coordinated_move = &coordinated_move;
    config.callbacks.rapid_move = &rapid_move;

    config.callbacks.set_temperature = &swallow_scalar;
    config.callbacks.set_fanspeed = &swallow_scalar;
    config.callbacks.set_speed_factor = &swallow_scalar;

    config.callbacks.user_data = counter;
    config.machine_origin[AXIS_X] = HOME_X;
    config.machine_origin[AXIS_Y] = HOME_Y;
    config.machine_origin[AXIS_Z] = HOME_Z;
    counter->parser = gcodep_new(&config);
  }
  gcodep_parse_line(counter->parser, block, NULL);
}

static void ShutdownCounter(struct CallCounter *counter) {
  gcodep_delete(counter->parser);
}

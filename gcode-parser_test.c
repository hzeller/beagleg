/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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
  AxesRegister abs_pos;  // last coordinates we got from a call.
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

void TEST_axes_letter_conversion() {
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

void TEST_simple_move() {
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

void TEST_set_origin_G92() {
  printf(" %s\n", __func__);
  struct CallCounter counter;
  bzero(&counter, sizeof(counter));
  GCodeTestParseLine("G1 X100 Y100", &counter);  // Some position.
  GCodeTestParseLine("G92 X5 Y7", &counter);     // Tool left bottom of it.
  GCodeTestParseLine("G1 X12 Y17", &counter);    // Move relative to that

  // Final position.
  assert(ExpectEqual(counter.abs_pos[AXIS_X], HOME_X + 100 - 5 + 12));
  assert(ExpectEqual(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17));
  
  ShutdownCounter(&counter);
  printf(" DONE %s\n", __func__);
}

int main() {
  printf("RUN (%s)\n", __FILE__);

  TEST_axes_letter_conversion();
  TEST_simple_move();
  TEST_set_origin_G92();

  printf("PASS (%s)\n", __FILE__);
}

// supporting mock counter functions.
static void do_count(void *p, int what) {
  ((struct CallCounter*)p)->call_count[what]++;
}
static void do_copy_register(void *p, const float axes[]) {
  memcpy(((struct CallCounter*)p)->abs_pos, axes, sizeof(AxesRegister));
}

static void gcode_start(void *p) { do_count(p, CALL_gcode_start); }
static void gcode_finished(void *p) { do_count(p, CALL_gcode_finished); }

static void go_home(void *p, AxisBitmap_t b) { do_count(p, CALL_go_home); }
static char probe_axis(void *p, float speed, enum GCodeParserAxis a,
                       float *probed) {
  do_count(p, CALL_probe_axis);
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
static struct GCodeParserCb mock_calls = {
  .gcode_start = &gcode_start,
  .gcode_finished = &gcode_finished,
  .go_home = &go_home,
  .probe_axis = &probe_axis,
  .motors_enable = &motors_enable,
  .coordinated_move = &coordinated_move,
  .rapid_move = &rapid_move
};

static void GCodeTestParseLine(const char *block, struct CallCounter *counter) {
  if (counter->parser == NULL) {
    struct GCodeParserConfig config;
    config.callbacks = mock_calls;
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

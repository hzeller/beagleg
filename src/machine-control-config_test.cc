/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2015 Henner Zeller <h.zeller@acm.org>
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

#include <gtest/gtest.h>

#include <iostream>

#include "config-parser.h"
#include "gcode-machine-control.h"  // contains struct MachineControlConfig

// TODO(hzeller): add tests for error conditions.

TEST(MachineControlConfig, GeneralMapping) {
  ConfigParser p;
  p.SetContent(
    "[ general ]\n"
    "home-order = ABC\n"
    "require-homing = 0\n"   // three ways ..
    "range-check = no\n"     // .. to say ...
    "synchronous = false\n"  // 'NO'.
    "auto-motor-disable-seconds = 188 \n");
  MachineControlConfig config;
  EXPECT_TRUE(config.ConfigureFromFile(&p));

  EXPECT_EQ("ABC", config.home_order);
  EXPECT_FALSE(config.require_homing);
  EXPECT_FALSE(config.range_check);
  EXPECT_FALSE(config.synchronous);
  EXPECT_EQ(188, config.auto_motor_disable_seconds);
}

TEST(MachineControlConfig, AxisMapping) {
  ConfigParser p;
  p.SetContent(
    "[ X-Axis ]\n"
    "steps-per-mm = 200 / (2 * 60)\n"  // simple expression support.
    "max-feedrate = 42\n"
    "max-acceleration = 4242\n"
    "range = 987\n"
    "home-pos = max\n"

    "[ Y-Axis ]\n"
    "home-pos = min\n"  // Different home pos.
  );

  MachineControlConfig config;
  EXPECT_TRUE(config.ConfigureFromFile(&p));

  EXPECT_FLOAT_EQ(200.0f / (2 * 60.0f), config.steps_per_mm[AXIS_X]);
  EXPECT_FLOAT_EQ(42.0f, config.max_feedrate[AXIS_X]);
  EXPECT_FLOAT_EQ(4242.0f, config.acceleration[AXIS_X]);
  EXPECT_FLOAT_EQ(987.0f, config.move_range_mm[AXIS_X]);
  EXPECT_EQ(HardwareMapping::TRIGGER_MAX, config.homing_trigger[AXIS_X]);
  EXPECT_EQ(HardwareMapping::TRIGGER_MIN, config.homing_trigger[AXIS_Y]);
}

#if 0
// TODO: needs to move to hardware mapping test
TEST(MachineControlConfig, MotorMapping) {
  ConfigParser p;
  p.SetContent("[ motor-mapping ]\n"
               "motor_1 = axis:x\n"
               "motor_2 = axis:-y\n"   // Invert axis with minus characters.
               // No mapping for motor 3
               "motor_4 = axis:z");
  MachineControlConfig config;
  EXPECT_TRUE(config.ConfigureFromFile(&p));

  EXPECT_EQ("Xy_Z", config.axis_mapping);
}
#endif

#if 0
// TODO: needs to move to hardware mapping test
TEST(MachineControlConfig, SwitchMapping) {
  ConfigParser p;
  p.SetContent("[ switch-mapping ]\n"
               "switch_3 = active:high \t min_x\n"    // multiple space
               "switch_4 = active:low min_y max_y\n"  // switch connected in series.
               );
  MachineControlConfig config;
  EXPECT_TRUE(config.ConfigureFromFile(&p));

  EXPECT_EQ(3, config.min_endstop_[AXIS_X].endstop_switch);
  EXPECT_TRUE(config.trigger_level_[3 - 1]);

  EXPECT_EQ(4, config.min_endstop_[AXIS_Y].endstop_switch);
  EXPECT_EQ(4, config.max_endstop_[AXIS_Y].endstop_switch);
  EXPECT_FALSE(config.trigger_level_[4 - 1]);
}
#endif

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

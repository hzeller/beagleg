// -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
//
// This file is part of BeagleG. http://github.com/hzeller/beagleg
//
// BeagleG is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// BeagleG is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
//
// see motor-interface-constants.h for available PINs

// CRAMPS cape Generic GPIO mapping
// http://reprap.org/wiki/CRAMPS

#define MOTOR_1_STEP_GPIO  PIN_P8_13  // X_STEP
#define MOTOR_2_STEP_GPIO  PIN_P8_15  // Y_STEP
#define MOTOR_3_STEP_GPIO  PIN_P8_19  // Z_STEP
#define MOTOR_4_STEP_GPIO  PIN_P9_16  // E0_STEP
#define MOTOR_5_STEP_GPIO  PIN_P9_17  // E1_STEP
#define MOTOR_6_STEP_GPIO  PIN_P9_24  // E2_STEP
#define MOTOR_7_STEP_GPIO  PIN_P9_42  // STEP_U (external)
#define MOTOR_8_STEP_GPIO  PIN_P9_31  // STEP_V (external)
#define MOTOR_9_STEP_GPIO  PIN_P9_29  // STEP_W (external)

#define MOTOR_1_DIR_GPIO   PIN_P8_12  // X_DIR
#define MOTOR_2_DIR_GPIO   PIN_P8_14  // Y_DIR
#define MOTOR_3_DIR_GPIO   PIN_P8_18  // Z_DIR
#define MOTOR_4_DIR_GPIO   PIN_P9_12  // E0_DIR
#define MOTOR_5_DIR_GPIO   PIN_P9_18  // E1_DIR
#define MOTOR_6_DIR_GPIO   PIN_P9_26  // E2_DIR
#define MOTOR_7_DIR_GPIO   PIN_P8_16  // DIR_U (external)
#define MOTOR_8_DIR_GPIO   PIN_P9_28  // DIR_V (external)
#define MOTOR_9_DIR_GPIO   PIN_P9_30  // DIR_W (external)

#define MOTOR_ENABLE_GPIO  PIN_P9_14  // AXIS_ENAn
#define MOTOR_ENABLE_IS_ACTIVE_HIGH 0  // 1 if EN, 0 if ~EN

#define MACHINE_PWR_GPIO   PIN_P9_23  // MACHINE_PWR
#define ESTOP_SW_GPIO      PIN_P8_26  // ESTOP_SW (output)

#define AUX_1_GPIO         PIN_P9_41  // FET5
#define AUX_2_GPIO         PIN_P9_22  // FET6
#define AUX_16_GPIO        PIN_P9_25  // LED

#define PWM_1_GPIO         PIN_P8_11  // FET1
#define PWM_2_GPIO         PIN_P9_15  // FET2
#define PWM_3_GPIO         PIN_P9_27  // FET3
#define PWM_4_GPIO         PIN_P9_21  // FET4

#define IN_1_GPIO          PIN_P8_8   // X-MIN
#define IN_2_GPIO          PIN_P8_10  // Y-MIN
#define IN_3_GPIO          PIN_P9_13  // Z-MIN
#define IN_4_GPIO          PIN_P8_7   // X-MAX
#define IN_5_GPIO          PIN_P8_9   // Y-MAX
#define IN_6_GPIO          PIN_P9_11  // Z-MAX
#define IN_7_GPIO          PIN_P8_17  // ESTOP

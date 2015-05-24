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

// This contains the defines the GPIO mappings for BUMPS
// https://github.com/hzeller/bumps

#define MOTOR_1_STEP_GPIO  PIN_P9_22  // STEP_X
#define MOTOR_2_STEP_GPIO  PIN_P9_21  // STEP_Y
#define MOTOR_3_STEP_GPIO  PIN_P9_18  // STEP_Z
#define MOTOR_4_STEP_GPIO  PIN_P9_17  // STEP_E
#define MOTOR_5_STEP_GPIO  PIN_P9_42  // STEP_A
#define MOTOR_6_STEP_GPIO  PIN_P9_26  // STEP_B
#define MOTOR_7_STEP_GPIO  PIN_P9_24  // STEP_C
#define MOTOR_8_STEP_GPIO  PIN_P9_41  // STEP_U
#define MOTOR_9_STEP_GPIO  GPIO_NOT_MAPPED

#define MOTOR_1_DIR_GPIO   PIN_P8_12  // DIR_X
#define MOTOR_2_DIR_GPIO   PIN_P8_11  // DIR_Y
#define MOTOR_3_DIR_GPIO   PIN_P8_16  // DIR_Z
#define MOTOR_4_DIR_GPIO   PIN_P8_15  // DIR_E
#define MOTOR_5_DIR_GPIO   PIN_P9_15  // DIR_A
#define MOTOR_6_DIR_GPIO   PIN_P9_23  // DIR_B
#define MOTOR_7_DIR_GPIO   PIN_P9_14  // DIR_C
#define MOTOR_8_DIR_GPIO   PIN_P9_16  // DIR_U
#define MOTOR_9_DIR_GPIO   GPIO_NOT_MAPPED

#define MOTOR_ENABLE_GPIO  PIN_P9_12  // ENn
#define MACHINE_PWR_GPIO   GPIO_NOT_MAPPED
#define ESTOP_HW_GPIO      GPIO_NOT_MAPPED
#define ESTOP_SW_GPIO      GPIO_NOT_MAPPED
#define LED_GPIO           GPIO_NOT_MAPPED

#define AUX_1_GPIO         PIN_P9_11  // AUX_1 "Aux, Open Collector"
#define AUX_2_GPIO         PIN_P9_13  // AUX_2 "Aux, Open Collector"

#define PWM_1_GPIO         PIN_P8_9   // PWM_1 "Power PWM"
#define PWM_2_GPIO         PIN_P8_10  // PWM_2 "Power PWM"
#define PWM_3_GPIO         PIN_P8_7   // PWM_3 "PWM, Open Collector"
#define PWM_4_GPIO         PIN_P8_8   // PWM_4 "PWM, Open Collector"

#define END_1_GPIO         PIN_P8_13  // END_X
#define END_2_GPIO         PIN_P8_14  // END_Y
#define END_3_GPIO         PIN_P8_17  // END_Z
#define END_4_GPIO         GPIO_NOT_MAPPED
#define END_5_GPIO         GPIO_NOT_MAPPED
#define END_6_GPIO         GPIO_NOT_MAPPED

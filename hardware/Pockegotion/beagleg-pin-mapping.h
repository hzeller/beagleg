// -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// (c) 2018 Henner Zeller <h.zeller@acm.org>
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

// Definition file for Pockegotion

#define MOTOR_1_STEP_GPIO  PIN_P1_29  // motor 1
#define MOTOR_2_STEP_GPIO  PIN_P1_31  // motor 2
#define MOTOR_3_STEP_GPIO  PIN_P1_33  // motor 3
#define MOTOR_4_STEP_GPIO  PIN_P1_34  // motor 4

#define MOTOR_1_DIR_GPIO   PIN_P2_24  // motor 1
#define MOTOR_2_DIR_GPIO   PIN_P2_33  // motor 2
#define MOTOR_3_DIR_GPIO   PIN_P2_22  // motor 3
#define MOTOR_4_DIR_GPIO   PIN_P2_18  // motor 4

#define MOTOR_ENABLE_GPIO  PIN_P2_10  // ENn
#define MOTOR_ENABLE_IS_ACTIVE_HIGH 0  // 1 if EN, 0 if ~EN

#define IN_1_GPIO          PIN_P2_05
#define IN_2_GPIO          PIN_P2_07
#define IN_3_GPIO          PIN_P2_09
#define IN_4_GPIO          PIN_P2_11

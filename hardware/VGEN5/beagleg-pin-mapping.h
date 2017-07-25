// -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// (c) 2015 H Hartley Sweeten <hsweeten@visionengravers.com>
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

// VISION GEN5 cape Generic GPIO mapping

#define MOTOR_1_STEP_GPIO  PIN_P9_41  // X_STEP
#define MOTOR_2_STEP_GPIO  PIN_P9_27  // Y_STEP
#define MOTOR_3_STEP_GPIO  PIN_P9_11  // Z_STEP
#define MOTOR_4_STEP_GPIO  PIN_P9_23  // Xb/Yb_STEP
#define MOTOR_5_STEP_GPIO  PIN_P9_13  // A_STEP

#define MOTOR_1_DIR_GPIO   PIN_P9_42  // X_DIR
#define MOTOR_2_DIR_GPIO   PIN_P9_30  // Y_DIR
#define MOTOR_3_DIR_GPIO   PIN_P9_12  // Z_DIR
#define MOTOR_4_DIR_GPIO   PIN_P9_25  // Xb/Yb_DIR
#define MOTOR_5_DIR_GPIO   PIN_P9_15  // A_DIR

#define MOTOR_ENABLE_GPIO  PIN_P9_16  // ENA (output)
#define MOTOR_ENABLE_IS_ACTIVE_HIGH 1  // 1 if EN, 0 if ~EN

#define AUX_1_GPIO         PIN_P8_7   // OUT0 - GPIO
#define AUX_2_GPIO         PIN_P8_8   // OUT1 - GPIO
#define AUX_3_GPIO         PIN_P8_9   // OUT2 - GPIO
#define AUX_4_GPIO         PIN_P8_10  // OUT3 - GPIO
#define AUX_5_GPIO         PIN_P9_14  // OUT4 - GPIO
#define AUX_6_GPIO         PIN_P9_17  // not routed (SPI0_CS0)
#define AUX_7_GPIO         PIN_P9_18  // not routed (SPI0_D1)
#define AUX_8_GPIO         PIN_P9_21  // not routed (SPI0_D0)
#define AUX_9_GPIO         PIN_P9_22  // not routed (SPI0_SCLK)
#define AUX_10_GPIO        PIN_P9_24  // not routed (UART1_TXD)
#define AUX_11_GPIO        PIN_P9_26  // not routed (UART1_RXD)
#define AUX_12_GPIO        PIN_P9_28  // not routed (SPI1_CS0)
#define AUX_13_GPIO        PIN_P9_29  // not routed (SPI1_DO)
#define AUX_14_GPIO        PIN_P9_31  // SPINDLE_ON
#define AUX_16_GPIO        PIN_P8_26  // LED

#define PWM_1_GPIO         AUX_1_GPIO // OUT0 - TIMER PWM
#define PWM_2_GPIO         AUX_2_GPIO // OUT1 - TIMER PWM
#define PWM_3_GPIO         AUX_3_GPIO // OUT2 - TIMER PWM
#define PWM_4_GPIO         AUX_4_GPIO // OUT3 - TIMER PWM

#define IN_1_GPIO          PIN_P8_15  // IN0 - X home (min)
#define IN_2_GPIO          PIN_P8_16  // IN1 - Y home (max)
#define IN_3_GPIO          PIN_P8_11  // IN2 - Z home (max)
#define IN_4_GPIO          PIN_P8_12  // IN3 - Z suface (min)
#define IN_5_GPIO          PIN_P8_13  // IN4 - LIM4
#define IN_6_GPIO          PIN_P8_14  // IN5 - LIM5
#define IN_7_GPIO          PIN_P8_17  // ESTOP
#define IN_8_GPIO          PIN_P8_18  // PAUSE
#define IN_9_GPIO          PIN_P8_19  // START

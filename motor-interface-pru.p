;; -*- asm -*-
;;
;; (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
;;
;; This file is part of BeagleG. http:;;github.com/hzeller/beagleg
;;
;; BeagleG is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; BeagleG is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with BeagleG.  If not, see <http:;;www.gnu.org/licenses/>.


#include "motor-interface-constants.h"

.origin 0
.entrypoint INIT

#define GPIO0 0x44e07000	; memory space mapped to GPIO	
#define GPIO1 0x4804c000	; memory space mapped to GPI1

#define GPIO_DATAOUT 0x13c      ; Set all the bits
#define GPIO_OE 0x134           ; setting direction.

#define PRU0_ARM_INTERRUPT 19
#define CONST_PRUDRAM	   C24

#define QUEUE_ELEMENT_SIZE (SIZE(QueueHeader) + SIZE(TravelParameters))

#define DIRECTION_GPIO1_SHIFT 12 ; contiguous direction bits in GPIO1

;; GPIO-0 - output steps.
#define MOTOR_1_STEP_BIT 2
#define MOTOR_2_STEP_BIT 3
#define MOTOR_3_STEP_BIT 4
#define MOTOR_4_STEP_BIT 5	
#define MOTOR_5_STEP_BIT 7
#define MOTOR_6_STEP_BIT 14
#define MOTOR_7_STEP_BIT 15
#define MOTOR_8_STEP_BIT 20

// Setting direction. Output bits are 0, and the assembler doesn't understand
// the ~ operator. Hence with xor.
#define MOTOR_OUT_BITS (0xFFFFFFFF ^ ((1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<7) | (1<<14) | (1<<15) | (1<<20)))
#define DIRECTION_OUT_BITS 0xFFFFFFFF ^ (0xFF << DIRECTION_GPIO1_SHIFT)

#define PARAM_START r8
#define PARAM_END  r17
.struct TravelParameters
	.u16 steps         // Total number of steps.
	.u16 dummy	   // for later.
	.u32 travel_delay  // delay-loop count.
	.u32 fraction_1    // Fixed point increments.
	.u32 fraction_2    // (Array would be good :) )
	.u32 fraction_3
	.u32 fraction_4
	.u32 fraction_5
	.u32 fraction_6
	.u32 fraction_7
	.u32 fraction_8
.ends

.struct QueueHeader
	.u8 state
	.u8 direction_bits
.ends

	;; counter states of the motors
#define STATE_START r18   	; after PARAM_END
#define STATE_END r25
.struct MotorState
	.u32 m1
	.u32 m2
	.u32 m3
	.u32 m4
	.u32 m5
	.u32 m6
	.u32 m7
	.u32 m8
.ends

.macro UpdateMotor
.mparam state_reg, fraction, bit
	ADD state_reg, state_reg, fraction
	QBBC M_END, state_reg, 31
	SET r1, r1, bit
M_END:
.endm
	
INIT:
	;; Clear STANDBY_INIT in SYSCFG register.
	LBCO r0, C4, 4, 4
	CLR r0, r0, 4
	SBCO r0, C4, 4, 4
	
	;; Make sure that we can write. Clearing the motor bits
	MOV r2, MOTOR_OUT_BITS
	MOV r3, GPIO0 | GPIO_OE
	SBBO r2, r3, 0, 4

	MOV r2, DIRECTION_OUT_BITS
	MOV r3, GPIO1 | GPIO_OE
	SBBO r2, r3, 0, 4

	MOV r2, 0		; Queue address in PRU memory

QUEUE_READ:
	.assign QueueHeader, r1.w0, r1.w0, queue_header
	LBCO queue_header, CONST_PRUDRAM, r2, SIZE(queue_header)
	QBNE QUEUE_READ, queue_header.state, STATE_FILLED ; wait until got data.

	;; Output direction bits
	MOV r3, queue_header.direction_bits
	LSL r3, r3, DIRECTION_GPIO1_SHIFT
	MOV r4, GPIO1 | GPIO_DATAOUT
	SBBO r3, r4, 0, 4

	;; queue_header processed, r1 is free to use
	ADD r1, r2, SIZE(QueueHeader) ; r2 stays at queue pos
	.assign TravelParameters, PARAM_START, PARAM_END, travel_params
	LBCO travel_params, CONST_PRUDRAM, r1, SIZE(travel_params)

	MOV r0, travel_params.steps
	LSL r0, r0, 1		; we need two loops to get one cycle
	QBEQ FINISH, r0, 0 ; zero steps signals finish.
	
	.assign MotorState, STATE_START, STATE_END, motor_state
	ZERO &motor_state, SIZE(motor_state)
	
	MOV r4, GPIO0 | GPIO_DATAOUT

	;; in use
	;; r2 = queue pos
	;; r4 = motor out GPIO
	;; status: r8..r16
STEP_GEN:
	;; update states and extract overflow bit into r1
	ZERO &r1, 4
	UpdateMotor motor_state.m1, travel_params.fraction_1, MOTOR_1_STEP_BIT
	UpdateMotor motor_state.m2, travel_params.fraction_2, MOTOR_2_STEP_BIT
	UpdateMotor motor_state.m3, travel_params.fraction_3, MOTOR_3_STEP_BIT
	UpdateMotor motor_state.m4, travel_params.fraction_4, MOTOR_4_STEP_BIT
	UpdateMotor motor_state.m5, travel_params.fraction_5, MOTOR_5_STEP_BIT
	UpdateMotor motor_state.m6, travel_params.fraction_6, MOTOR_6_STEP_BIT
	UpdateMotor motor_state.m7, travel_params.fraction_7, MOTOR_7_STEP_BIT
	UpdateMotor motor_state.m8, travel_params.fraction_8, MOTOR_8_STEP_BIT

	SBBO r1, r4, 0, 4	; motor bits to GPIO0

	;; TODO: figure out the number of steps according to acceleration.
	;; now, we just do the travel speed loop time.
	MOV r1, travel_params.travel_delay
STEP_DELAY:			; TODO: LOOP instruction ? Does it >16bit ?
	SUB r1, r1, 1
	QBNE STEP_DELAY, r1, 0


	SUB r0, r0, 1		; number of steps.
	QBNE STEP_GEN, r0, 0

DONE_STEP_GEN:
	;; We're done with instruction. Mark slot as empty...
	MOV queue_header.state, STATE_EMPTY
	SBCO queue_header.state, CONST_PRUDRAM, r2, 1
	MOV R31.b0, PRU0_ARM_INTERRUPT+16 ; signal host program free slot.
		
	;; Next position in ring buffer
	ADD r2, r2, QUEUE_ELEMENT_SIZE
	MOV r1, QUEUE_LEN * QUEUE_ELEMENT_SIZE ; end-of-queue
	QBLT QUEUE_READ, r1, r2
	ZERO &r2, 4
	JMP QUEUE_READ

FINISH:
	MOV queue_header.state, STATE_EMPTY
	SBCO queue_header.state, CONST_PRUDRAM, r2, 1
	MOV R31.b0, PRU0_ARM_INTERRUPT+16

	HALT
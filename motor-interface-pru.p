;; -*- asm -*-
;;
;; (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
;;
;; This file is part of BeagleG. http://github.com/hzeller/beagleg
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
;; along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.


#include "motor-interface-constants.h"
#include "idiv.hp"

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

// Setting direction. Output bits are 0, and the assembler does not understand
// the ~ operator in immediates. Thus we do it with xor.
// Also the pre-processor does not allow for backslash-line-continuation, hence
// this looks a bit ugly in one line.
#define MOTOR_OUT_BITS (0xFFFFFFFF ^ ( (1<<MOTOR_1_STEP_BIT) | (1<<MOTOR_2_STEP_BIT) | (1<<MOTOR_3_STEP_BIT) | (1<<MOTOR_4_STEP_BIT) | (1<<MOTOR_5_STEP_BIT) | (1<<MOTOR_6_STEP_BIT) | (1<<MOTOR_7_STEP_BIT) | (1<<MOTOR_8_STEP_BIT) ))
	
// Direction bits are a contiguous chunk, just a bit shifted.
#define DIRECTION_OUT_BITS 0xFFFFFFFF ^ (0xFF << DIRECTION_GPIO1_SHIFT)

#define PARAM_START r7
#define PARAM_END  r19
.struct TravelParameters
	// We do at most 2^16 loops to avoid accumulating too much rounding
	// error in the fraction addition. Longer moves are split into separate
	// requests by the host.
	.u16 loops_accel	 // Phase 1: steps spent in acceleration.
	.u16 loops_travel	 // Phase 2: steps spent in travel.
	.u16 loops_decel         // Phase 3: steps spent in deceleration.

	.u16 padding		 // not used right now.
	
	.u32 accel_series_index  // index into the taylor series.
	.u32 hires_accel_cycles  // initial delay cycles, for acceleration
	                         // shifted by DELAY_CYCLE_SHIFT
	                         // Changes in the different phases.
	.u32 travel_delay_cycles // Exact cycle value for travel (do not rely
	                         // on approximation to exactly reach that)
	
	// 1.31 Fixed point increments for each motor
	.u32 fraction_1
	.u32 fraction_2          
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
#define STATE_START r20   	; after PARAM_END
#define STATE_END r27
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

;;; Calculate the current delay depending on the phase (acceleration, travel,
;;; deceleration). Modifies the values in params, which is of type
;;; TravelParameters.
;;; Needs one state_register to keep its own state and two scratch registers.
;;; Outputs the resulting delay in output_reg.
;;; Returns special value 0 when done.
;;; Only used once, so just macro.
;;; 
;;; We are approximating the needed sqrt() operation with a few terms
;;; from an Taylor series which brings sufficient accuracy and boils down
;;; to a single (somewhat expensive) division. I tried as well using an
;;; integer sqrt directly, but the rounding errors were worse.
;;; Thanks to this paper for inspiration:
;;;   http://embedded.com/design/mcus-processors-and-socs/4006438
.macro CalculateDelay
.mparam output_reg, params, state_register, divident_tmp, divisor_tmp
;;; We use the 'state_register' to store the remainder of the division
;;; to carry it to the next division for higher accuracy.
;;; Note, we are inlining the division macro twice here instead of wrapping it
;;; in a function. There is no need, we have enough code-space.
PHASE_1_ACCELERATION:	; ==================================================
	QBEQ PHASE_2_TRAVEL, params.loops_accel, 0
	QBEQ accel_calc_done, params.accel_series_index, 0 // first ? no calc.
	
	;; divident = (hires_accel_cycles << 1) + remainder
	LSL divident_tmp, params.hires_accel_cycles, 1
	;; Add previous remainder for higher resolution.
	ADD divident_tmp, divident_tmp, state_register
	
	;; divisor = (accel_series_index << 2) + 1
	LSL divisor_tmp, params.accel_series_index, 2
	ADD divisor_tmp, divisor_tmp, 1
	
	idiv_macro divident_tmp, divisor_tmp, state_register
	
	;; params.hires_accel_cycles -= quotient (divident_tmp became quotient)
	SUB params.hires_accel_cycles, params.hires_accel_cycles, divident_tmp
accel_calc_done:
	ADD params.accel_series_index, params.accel_series_index, 1 ; series++
	SUB params.loops_accel, params.loops_accel, 1		; loops_accel--

	;; The calculation is done in higher resolution with DELAY_CYCLE_SHIFT
	;; more bits. Shift back: output_reg = hires_cycles << DELAY_CYCLE_SHIFT
	LSR output_reg, params.hires_accel_cycles, DELAY_CYCLE_SHIFT

	;; Correct Timing: Substract the number of cycles we have spent in this
	;; routine. We take half, because the delay-loop needs 2 cycles.
	SUB output_reg, output_reg, (IDIV_MACRO_CYCLE_COUNT + 9) / 2
	JMP DONE_CALCULATE_DELAY

PHASE_2_TRAVEL:		; ==================================================
	QBEQ PHASE_3_DECELERATION, params.loops_travel, 0
	SUB params.loops_travel, params.loops_travel, 1	        ; loops_travel--
	MOV output_reg, params.travel_delay_cycles
	SUB output_reg, output_reg, (4 / 2)
	JMP DONE_CALCULATE_DELAY

PHASE_3_DECELERATION:	; ==================================================
	QBNE calc_decel, params.loops_decel, 0
	ZERO &output_reg, 4                // we are done. Special stop value 0
	JMP DONE_CALCULATE_DELAY
calc_decel:
	;; divident = (hires_accel_cycles << 1) + remainder
	LSL divident_tmp, params.hires_accel_cycles, 1
	ADD divident_tmp, divident_tmp, state_register
	
	;; divisor = (accel_series_index << 2) - 1
	LSL divisor_tmp, params.accel_series_index, 2
	SUB divisor_tmp, divisor_tmp, 1
	
	idiv_macro divident_tmp, divisor_tmp, state_register
	
	;; params.hires_accel_cycles += quotient (divident_tmp became quotient)
	ADD params.hires_accel_cycles, params.hires_accel_cycles, divident_tmp

	SUB params.accel_series_index, params.accel_series_index, 1 ; series--
	SUB params.loops_decel, params.loops_decel, 1	        ; loops_decel--

	;; The calculation is done in higher resolution with DELAY_CYCLE_SHIFT
	;; more bits. Shift back: output_reg = hires_cycles << DELAY_CYCLE_SHIFT
	LSR output_reg, params.hires_accel_cycles, DELAY_CYCLE_SHIFT
	
	;; Correct timing: Substract the number of cycles we have spent here.
	SUB output_reg, output_reg, (IDIV_MACRO_CYCLE_COUNT + 11) / 2

DONE_CALCULATE_DELAY:
.endm

;;; Update the state register of a motor with its 1.31 resolution fraction.
;;; The 31st bit contains the overflow that we're interested in.
;;; Uses a fixed number of 4 cycles
.macro UpdateMotor
.mparam output_register, scratch, state_reg, fraction, bit
	ADD state_reg, state_reg, fraction
	LSR scratch, state_reg, 31
	LSL scratch, scratch, bit
	OR  output_register, output_register, scratch
.endm

INIT:
	;; Clear STANDBY_INIT in SYSCFG register.
	LBCO r0, C4, 4, 4
	CLR r0, r0, 4
	SBCO r0, C4, 4, 4
	
	;; Make sure that we can write. Clearing the bits means: output.
	;; Stepper bits on GPIO-0
	MOV r2, MOTOR_OUT_BITS
	MOV r3, GPIO0 | GPIO_OE
	SBBO r2, r3, 0, 4

	;; Direction bits on GPIO-1
	MOV r2, DIRECTION_OUT_BITS
	MOV r3, GPIO1 | GPIO_OE
	SBBO r2, r3, 0, 4

	MOV r2, 0		; Queue address in PRU memory

QUEUE_READ:
	;; 
	;; Read next element from ring-buffer
	;;
	
	;; Check queue header at our read-position until it contains something.
	.assign QueueHeader, r1.w0, r1.w0, queue_header
	LBCO queue_header, CONST_PRUDRAM, r2, SIZE(queue_header)
	QBEQ QUEUE_READ, queue_header.state, STATE_EMPTY ; wait until got data.

	QBEQ FINISH, queue_header.state, STATE_EXIT
	
	;; Output direction bits to GPIO-1
	MOV r3, queue_header.direction_bits
	LSL r3, r3, DIRECTION_GPIO1_SHIFT
	MOV r4, GPIO1 | GPIO_DATAOUT
	SBBO r3, r4, 0, 4

	;; queue_header processed, r1 is free to use
	ADD r1, r2, SIZE(QueueHeader) ; r2 stays at queue pos
	.assign TravelParameters, PARAM_START, PARAM_END, travel_params
	LBCO travel_params, CONST_PRUDRAM, r1, SIZE(travel_params)

	.assign MotorState, STATE_START, STATE_END, mstate
	ZERO &mstate, SIZE(mstate)
	
	MOV r4, GPIO0 | GPIO_DATAOUT
	ZERO &r3, 4		; initialize delay calculation state register.
	
	;; Registers
	;; r0, r1 free for calculation
	;; r2 = queue pos
	;; r3 = state for CalculateDelay
	;; r4 = motor out GPIO
	;; r5, r6 scratch
	;; parameter:    r7..r19
	;; motor-state: r20..r17
STEP_GEN:
	;; 
	;; Generate motion profile configured by TravelParameters
	;;
	
	;; update states and extract overflow bits into r1
	;; 8 times 4 = 32 cpu cycles = 160ns. So whatever step output we do
	;; is some time after we set the direction bit. Good, because the Allegro
	;; chip requires this time delay.
	ZERO &r1, 4
	UpdateMotor r1, r5, mstate.m1, travel_params.fraction_1, MOTOR_1_STEP_BIT
	UpdateMotor r1, r5, mstate.m2, travel_params.fraction_2, MOTOR_2_STEP_BIT
	UpdateMotor r1, r5, mstate.m3, travel_params.fraction_3, MOTOR_3_STEP_BIT
	UpdateMotor r1, r5, mstate.m4, travel_params.fraction_4, MOTOR_4_STEP_BIT
	UpdateMotor r1, r5, mstate.m5, travel_params.fraction_5, MOTOR_5_STEP_BIT
	UpdateMotor r1, r5, mstate.m6, travel_params.fraction_6, MOTOR_6_STEP_BIT
	UpdateMotor r1, r5, mstate.m7, travel_params.fraction_7, MOTOR_7_STEP_BIT
	UpdateMotor r1, r5, mstate.m8, travel_params.fraction_8, MOTOR_8_STEP_BIT

	SBBO r1, r4, 0, 4	; motor bits to GPIO-0

	CalculateDelay r1, travel_params, r3, r5, r6
	QBEQ DONE_STEP_GEN, r1, 0       ; special value 0: all steps consumed.
STEP_DELAY:				; Create time delay between steps.
	SUB r1, r1, 1
	QBNE STEP_DELAY, r1, 0

	JMP STEP_GEN

DONE_STEP_GEN:
	;; We are done with instruction. Mark slot as empty...
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

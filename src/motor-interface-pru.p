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

#define PRU0_ARM_INTERRUPT 19
#define CONST_PRUDRAM	   C24

#define QUEUE_ELEMENT_SIZE (SIZE(QueueHeader) + SIZE(TravelParameters))
#define QUEUE_OFFSET 4

#define PARAM_START r7
#define PARAM_END  r19
.struct TravelParameters
	// We do at most 2^16 loops to avoid accumulating too much rounding
	// error in the fraction addition. Longer moves are split into separate
	// requests by the host.
	.u16 loops_accel	 // Phase 1: steps spent in acceleration.
	.u16 loops_travel	 // Phase 2: steps spent in travel.
	.u16 loops_decel         // Phase 3: steps spent in deceleration.

	.u16 aux		 // all 16 bits can be used

	.u32 accel_series_index  // index into the taylor series.
	.u32 hires_accel_cycles  // initial delay cycles, for acceleration
	                         // shifted by DELAY_CYCLE_SHIFT
	                         // Changes in the different phases.
	.u32 travel_delay_cycles // Exact cycle value for travel (do not rely
	                         // on accel approx to exactly reach that)

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
.assign MotorState, STATE_START, STATE_END, mstate

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
;;;   http://embedded.com/design/mcus-processors-and-socs/4006438/stepper
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
	;; more bits. Shift back: output_reg = hires_cycles >> DELAY_CYCLE_SHIFT
	LSR output_reg, params.hires_accel_cycles, DELAY_CYCLE_SHIFT

	;; Correct Timing: Substract the number of cycles we have spent in this
	;; routine. We take half, because the delay-loop needs 2 cycles.
	SUB output_reg, output_reg, (IDIV_MACRO_CYCLE_COUNT + 9) / 2
	JMP DONE_CALCULATE_DELAY

PHASE_2_TRAVEL:		; ==================================================
	QBEQ PHASE_3_DECELERATION, params.loops_travel, 0
	SUB params.loops_travel, params.loops_travel, 1	        ; loops_travel--
	MOV output_reg, params.travel_delay_cycles
	SUB output_reg, output_reg, (4 / 2) ; substract cycles spent here
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
	;; more bits. Shift back: output_reg = hires_cycles >> DELAY_CYCLE_SHIFT
	LSR output_reg, params.hires_accel_cycles, DELAY_CYCLE_SHIFT

	;; Correct timing: Substract the number of cycles we have spent here.
	SUB output_reg, output_reg, (IDIV_MACRO_CYCLE_COUNT + 11) / 2

DONE_CALCULATE_DELAY:
.endm

;;; This macro decrease the counter that holds the overall number of loops left
;;; to be performed and then it push it in the PRU DRAM status register.
.macro UpdateQueueStatus
	;; Decrease the step counter
	SUB r28, r28, 1 ; status_loops--
	;; Push in DRAM
	MOV r0, 0
	SBCO r28, CONST_PRUDRAM, r0, 4
	SUB r1, r1, (4 / 2) ; Subtract the loops consumed for this macro.
.endm

INIT:
	;; Clear STANDBY_INIT in SYSCFG register.
	LBCO r0, C4, 4, 4
	CLR r0, r0, 4
	SBCO r0, C4, 4, 4

	MOV r2, QUEUE_OFFSET ; Queue address in PRU memory
	MOV r28, 0           ; Status register in PRU memory,
	                     ; r28.b3 for current queue position,
	                     ; bottom three for the remaining steps of the current slot.
QUEUE_READ:
	;;
	;; Read next element from ring-buffer
	;;

	;; Check queue header at our read-position until it contains something.
	.assign QueueHeader, r1.w0, r1.w0, queue_header
	LBCO queue_header, CONST_PRUDRAM, r2, SIZE(queue_header)
	QBEQ QUEUE_READ, queue_header.state, STATE_EMPTY ; wait until got data.
	QBEQ QUEUE_READ, queue_header.state, STATE_ABORT

	QBEQ FINISH, queue_header.state, STATE_EXIT

	;; Set direction bits
	MOV r3, queue_header.direction_bits
	CALL SetDirections

	;; queue_header processed, r1 is free to use
	ADD r1, r2, SIZE(QueueHeader) ; r2 stays at queue pos
	.assign TravelParameters, PARAM_START, PARAM_END, travel_params
	LBCO travel_params, CONST_PRUDRAM, r1, SIZE(travel_params)

	;; Set the Aux bits
	MOV r3, travel_params.aux
	CALL SetAuxBits

	ZERO &mstate, SIZE(mstate)	; clear the motor states
	ZERO &r3, 4			; initialize delay calculation state register.

	;; STATUS REGISTER
	;; ! We are assuming that writing the 4 bytes status register is atomic
	;; and we guarantee that the bottom three bytes are all zero so we just need
	;; to sum up the 3 loop counters. The upper bound of this sum will always be
	;; less than 2^18, thus fit in the lower 24 bits allocated for it.
	;; At each loop executed this counter is decreased of one unit.
	ADD r28, r28, travel_params.loops_accel
	ADD r28, r28, travel_params.loops_travel
	ADD r28, r28, travel_params.loops_decel

	MOV r0, 0 ; Status register address in PRU memory.
	SBCO r28, CONST_PRUDRAM, r0, 4

	;; Registers
	;; r0, r1 free for calculation
	;; r2 = queue pos
	;; r3 = state for CalculateDelay
	;; scratch:           r4..r6
	;; parameter:         r7..r19
	;; motor-state:       r20..r27
	;; status-variable:   r28
	;; call/ret:          r30
STEP_GEN:
	MOV r0, 0
	CALL CheckForEStop
	QBNE DO_STEP_GEN, r0, 1
	// Estop detected, update the queue status and abort
	ZERO &r28, 3			// Estop detected, zero the loop counter
	ADD r28, r28, 1			// status_loops++ (removed by UpdateQueueStatus)
	UpdateQueueStatus
	MOV queue_header.state, STATE_ABORT
	JMP STEP_GEN_ABORTED

DO_STEP_GEN:
	;;
	;; Generate motion profile configured by TravelParameters
	;;

	;;; Update the state registers with the 1.31 resolution fraction.
	;;; The 31st bit contains the overflow that causes a step.
	ADD mstate.m1, mstate.m1, travel_params.fraction_1
	ADD mstate.m2, mstate.m2, travel_params.fraction_2
	ADD mstate.m3, mstate.m3, travel_params.fraction_3
	ADD mstate.m4, mstate.m4, travel_params.fraction_4
	ADD mstate.m5, mstate.m5, travel_params.fraction_5
	ADD mstate.m6, mstate.m6, travel_params.fraction_6
	ADD mstate.m7, mstate.m7, travel_params.fraction_7
	ADD mstate.m8, mstate.m8, travel_params.fraction_8

	;; Set the step bits (Need to check timing)
	CALL SetSteps

	CalculateDelay r1, travel_params, r3, r5, r6
	QBEQ DONE_STEP_GEN, r1, 0       ; special value 0: all steps consumed.
	UpdateQueueStatus

STEP_DELAY:				; Create time delay between steps.
	SUB r1, r1, 1                   ; two cycles per loop.
	QBNE STEP_DELAY, r1, 0

	JMP STEP_GEN

DONE_STEP_GEN:
	;; We are done with instruction. Mark slot as empty...
	MOV queue_header.state, STATE_EMPTY
STEP_GEN_ABORTED:
	SBCO queue_header.state, CONST_PRUDRAM, r2, 1
	MOV R31.b0, PRU0_ARM_INTERRUPT+16 ; signal host program free slot.

	;; Next position in ring buffer
	ADD r2, r2, QUEUE_ELEMENT_SIZE
	ADD r28.b3, r28.b3, 1                  ; add + 1 to the MSB byte
	MOV r1, QUEUE_LEN * QUEUE_ELEMENT_SIZE ; end-of-queue
	QBLT QUEUE_READ, r1, r2
	MOV r2, QUEUE_OFFSET
	ZERO &r28, 4
	JMP QUEUE_READ

FINISH:
	MOV queue_header.state, STATE_EMPTY
	SBCO queue_header.state, CONST_PRUDRAM, r2, 1
	MOV R31.b0, PRU0_ARM_INTERRUPT+16

	HALT

;;; This include file needs to provide the subroutines
;;;    SetAuxBits
;;;    SetDirections
;;;    SetSteps
;;; There is one of these in each hardware directory. They can choose to just
;;; include pru-generic-io-routines.hp that just uses the generic bits
;;; or provide their own optimized version.
#include <pru-io-routines.hp>

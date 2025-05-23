/****************************************************************************
 * arch/arm/src/armv6-m/arm_exception.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2013 Gregory Nutt. All rights reserved.
 * SPDX-FileCopyrightText: 2012 Michael Smith. All rights reserved.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#ifdef CONFIG_SMP
#  include "chip.h"
#endif

#include "exc_return.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl		exception_common
	.file		"arm_exception.S"

/****************************************************************************
 * .text
 ****************************************************************************/

/* Common exception handling logic.  On entry here, the return stack is on either
 * the PSP or the MSP and looks like the following:
 *
 *      REG_XPSR
 *      REG_R15
 *      REG_R14
 *      REG_R12
 *      REG_R3
 *      REG_R2
 *      REG_R1
 * MSP->REG_R0
 *
 * And
 *      IPSR contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 */

	.text
	.align	2
	.code	16
	.thumb_func
	.syntax	unified
	.type	exception_common, function
exception_common:
	.cfi_sections	.debug_frame
	.cfi_startproc
	/* Complete the context save */

	/* Get the current stack pointer.  The EXC_RETURN value tells us whether
	 * the context is on the MSP or PSP.
	 */

	mov		r0, r14					/* Copy high register to low register */
	lsls		r0, #(31 - EXC_RETURN_PROCESS_BITNO)	/* Move to bit 31 */
	bmi		1f					/* Test bit 31 */
	mrs		r1, msp					/* R1=The main stack pointer */
	subs		r1, #SW_XCPT_SIZE			/* R1=Beginning of context array on the stack */
	msr		msp, r1					/* Reserved stack space */
	isb		sy
	b		2f

1:
	mrs		r1, psp					/* R1=The process stack pointer */
	subs		r1, #SW_XCPT_SIZE			/* R1=Beginning of context array on the stack */

	/* R1 is the current stack pointer.  HW_XCPT_REGS were pushed onto the stack
	 * when the interrupt was taken so (R1)+HW_XCPT_SIZE is the value of the
	 * stack pointer before the interrupt.  The total size of the context save
	 * area is XCPTCONTEXT_SIZE = SW_XCPT_SIZE + HW_XCPT_SIZE so (R1)-SW_XCPT_SIZE
	 * is the address of the beginning of the context save area.
	 */

2:
	/* Save SP, PRIMASK, and R4-R7 in the context array */

	movs		r2, #XCPTCONTEXT_SIZE			/* R2=Size of the context array */
	add		r2, r1					/* R2=MSP/PSP before the interrupt was taken */
								/* (ignoring the xPSR[9] alignment bit) */
	mrs		r3, primask				/* R3=Current PRIMASK setting */
	mov		r0, r1					/* Copy the context array pointer */
	stmia		r0!, {r2-r7}				/* Save the SP, PRIMASK, and R4-R7 in the context array */

	/* Save R8-R11 control and the EXEC_RETURN value in the context array */

	mov		r2, r8					/* Copy high registers to low */
	mov		r3, r9
	mov		r4, r10
	mov		r5, r11
	mrs		r6, control				/* R6=control */
	mov		r7, r14
	stmia		r0!, {r2-r7}				/* Save the high registers r8-r11 control and r14 */

	/* Get the exception number in R0=IRQ, R1=register save area on stack */

	mrs		r0, ipsr				/* R0=exception number */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will use the stack that was current when the interrupt was taken.
	 */

#if CONFIG_ARCH_INTERRUPTSTACK < 3
	/* If the interrupt stack is disabled, reserve xcpcontext to ensure
	 * that signal processing can have a separate xcpcontext to handle
	 * signal context (reference: arm_schedulesigaction.c):
	 *      ----------------------
	 *     |    IRQ XCP context   |
	 *      ----------------------
	 *     |  Signal XCP context  |
	 *      ----------------------   <- SP
	 * also the sp should be restore after arm_doirq()
	 */

	mov		r2, r14
	lsls		r2, #(31 - EXC_RETURN_THREAD_BITNO)	/* Move to bit 31 */
	bpl		3f					/* Test bit 31 */
	mov		r2, r1					/* Reserve signal context */
	subs		r2, r2, #XCPTCONTEXT_SIZE
	msr		msp, r2					/* We are using the main stack pointer */
	isb		sy
3:
#endif
	mov		fp,	r1
	.cfi_def_cfa	fp,	0				/* Register in fp, so we just set fp as frame */
	.cfi_offset	pc,	REG_PC * 4
	.cfi_offset	sp,	REG_SP * 4
	.cfi_offset	lr,	REG_LR * 4
	bl		arm_doirq				/* R0=IRQ, R1=register save area on stack */

	/* On return from arm_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.
	 */

	/* Recover R8-R11 and EXEC_RETURN (5 registers) */

	movs		r2, #(4*REG_R8)				/* R2=Offset to R8 storage */
	adds		r1, r0, r2				/* R1=Address of R8 storage */
	ldmia		r1!, {r2-r7}				/* Recover R8-R11 control and R14 (6 registers)*/
	mov		r8, r2					/* Move to position in high registers */
	mov		r9, r3
	mov		r10, r4
	mov		r11, r5
	msr		control, r6
	mov		r14, r7					/* EXEC_RETURN */

	/* Recover SP (R2), PRIMASK (R3), and R4-R7. Determine the value of
	 * the stack pointer as it was on entry to the exception handler.
	 */

	ldmia		r0!, {r2-r7}				/* Recover R4-R7 + 2 temp values */
	movs		r1, #HW_XCPT_SIZE			/* R1=Size of hardware-saved portion of the context array */
	subs		r1, r2, r1				/* R1=Value of MSP/PSP on exception entry */

	/* Restore the stack pointer.  The EXC_RETURN value tells us whether the
	 * context is on the MSP or PSP.
	 */

	mov		r0, r14					/* Copy high register to low register */
	lsls		r0, #(31 - EXC_RETURN_PROCESS_BITNO)	/* Move to bit 31 */
	bmi		4f					/* Test bit 31 */
	msr		msp, r1					/* R1=The main stack pointer */
	isb		sy
	b		5f

4:
	msr		psp, r1					/* R1=The process stack pointer */
	isb		sy

5:

	/* Restore the interrupt state */

	msr		primask, r3				/* Restore interrupts priority masking*/

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) select the correct stack.
	 */

	bx		r14					/* And return */
	.cfi_endproc
	.size	exception_common, .-exception_common

/****************************************************************************
 *  Name: g_intstackalloc/g_intstacktop
 *
 * Description:
 *   Shouldn't happen
 *
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.global	g_intstackalloc
	.global	g_intstacktop
	.balign	4
g_intstackalloc:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstacktop:
	.size	g_intstackalloc, .-g_intstackalloc
#endif

	.end

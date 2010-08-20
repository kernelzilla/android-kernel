/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * JTAG support functions for ARMv7-based Qualcomm SoCs.
 */
#include <linux/linkage.h>
#include <asm/assembler.h>

ENTRY(msm_save_jtag_debug)
	/* lock debug and preserve registers through power collapse */
	ldr     r3, =dbg_state        /* store state at dbg_state */

	ldr     r1, =0xc5ACCE55       /* set DBGOSLAR lock */
	mcr     p14,0,r1,c1,c0,4
	isb

	mrc     p14,0,r1,c1,c2,4      /* DBGOSSRR state register count */

	cmp     r1, #(0x20-1)         /* check for state overflow */
	movge   r1, #0                /* if not enough space, don't save */

	str     r1,[r3],#4            /* save count for restore */

1:	cmp     r1,#0
	mrcne   p14,0,r2,c1,c2,4      /* DBGOSSRR state value */
	strne   r2,[r3],#4            /* push value */
	subne   r1,r1,#1
	bne     1b

	/* unlock JTAG.  Works better than leaving locked. */
	stmfd   sp!, {lr}
	bl      msm_unlock_jtag_debug
	ldmfd   sp!, {lr}
	bx      lr

ENTRY(msm_unlock_jtag_debug)
	mov     r0, #0                /* unlock value */
	mcr     p14,0,r0,c1,c0,4      /* unlock DBGOSLAR */
	isb
	bx      lr

ENTRY(msm_restore_jtag_debug)
	/* restore debug registers after power collapse  */
	ldr     r3, =dbg_state        /* load state from dbg_state */

	ldr     r1, =0xc5ACCE55       /* set DBGOSLAR lock */
	mcr     p14,0,r1,c1,c0,4
	isb

	mrc     p14,0,r1,c1,c2,4      /* DBGOSSRR dummy read (required)*/
	ldr     r1,[r3],#4            /* load saved count */
	cmp     r1,#0                 /* skip if none stored
	beq     msm_pm_dbg_restore_end

	/* restores debug state except DBGDSCR */
1:	ldr     r2,[r3],#4
	cmp     r1,#0x10              /* DBGDSCR special case */
	biceq   r2,r2,#0xc000         /* DBGDSCR = DBGDSCR & ~0xc000 */
	mcr     p14,0,r2,c1,c2,4      /* DBGOSSRR write state value */
	subs    r1,r1,#1
	bne     1b
	isb

	/* second loop to restore DBGDSCR after other state restored */
	ldr     r3, =dbg_state        /* load state from dbg_state */

	ldr     r1, =0xc5ACCE55       /* set DBGOSLAR lock */
	mcr     p14,0,r1,c1,c0,4
	isb

	mrc     p14,0,r1,c1,c5,4      /* clear sticky power down bit */
	isb

	mrc     p14,0,r1,c1,c2,4      /* DBGOSSRR dummy read (required)*/
	ldr     r1,[r3],#4            /* load saved count */

1:	ldr     r2,[r3],#4
	mcr     p14,0,r2,c1,c2,4      /* DBGOSSRR write state value */
	subs    r1,r1,#1
	bne     1b
msm_pm_dbg_restore_end:
	mcr     p14,0,r1,c1,c0,4      /* unlock DBGOSLAR */
	isb
	bx      lr


	.data

dbg_state:
	.space  4 * 0x20


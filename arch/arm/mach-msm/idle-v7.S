/*
 * Idle processing for ARMv7-based Qualcomm SoCs.
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/vfpmacros.h>

ENTRY(msm_pm_collapse)
	ldr     r0, =saved_state
	stmia   r0!, {r4-r14}
	mrc     p15, 0, r1, c1, c0, 0 /* MMU control */
	mrc     p15, 0, r2, c2, c0, 0 /* ttb */
	mrc     p15, 0, r3, c3, c0, 0 /* dacr */
	mrc     p15, 0, ip, c13, c0, 1 /* context ID */
	stmia   r0!, {r1-r3, ip}
#ifdef CONFIG_VFP
	VFPFSTMIA r0, r1              /* Save VFP working registers */
	fmrx    r1, fpexc
	fmrx    r2, fpscr
	stmia   r0!, {r1, r2}         /* Save VFP state registers */
#endif
	stmfd   sp!, {r4-r5, r7, r9-r11, lr}
	bl      v7_flush_dcache_all
	ldmfd   sp!, {r4-r5, r7, r9-r11, lr}

	mrc     p15, 0, r1, c1, c0, 0    /* read current CR    */
	bic     r0, r1, #(1 << 2)        /* clear dcache bit   */
	bic     r0, r0, #(1 << 12)       /* clear icache bit   */
	mcr     p15, 0, r0, c1, c0, 0    /* disable d/i cache  */
	/* fall though */
ENTRY(msm_arch_idle)
#if defined(CONFIG_MSM_FIQ_SUPPORT)
	cpsid   f
#endif
	mov     r0, #0                   /* prepare wfi value
					  * also used as return value from
					  * msm_pm_collapse */
	wfi

#if defined(CONFIG_MSM_FIQ_SUPPORT)
	cpsie   f
#endif
	mov     pc, lr

ENTRY(msm_pm_collapse_exit)
#if 0 /* serial debug */
	mov     r0, #0x80000016
	mcr     p15, 0, r0, c15, c2, 4
	mov     r0, #0xA9000000
	add     r0, r0, #0x00A00000 /* UART1 */
	/*add     r0, r0, #0x00C00000*/ /* UART3 */
	mov     r1, #'A'
	str     r1, [r0, #0x00C]
#endif
	ldr     r1, =saved_state_end
	ldr     r2, =msm_pm_collapse_exit
	adr     r3, msm_pm_collapse_exit
	add     r1, r1, r3
	sub     r1, r1, r2
#ifdef CONFIG_VFP
	mrc     p15, 0, r2, c1, c0, 2 /* Read CP Access Control Register */
	orr     r2, r2, #0x00F00000   /* Enable full access for p10,11 */
	mcr     p15, 0, r2, c1, c0, 2 /* Write CPACR */
	isb
	mov     r2, #0x40000000       /* Enable VFP */
	fmxr    fpexc, r2
	isb
	ldmdb   r1!, {r2, r3}         /* Read saved VFP state registers */
	sub     r1, r1, #32*8         /* Jump to start of vfp regs area */
	VFPFLDMIA r1, r4              /* Restore VFP working registers,
				       * r1 incremented to end of vfp
				       * regs area */
	sub     r1, r1, #32*8         /* Jump back to start of vfp regs area */
	fmxr    fpscr, r3             /* Restore FPSCR */
	fmxr    fpexc, r2             /* Restore FPEXC last */
#endif
	ldmdb   r1!, {r2-r5}
	mcr     p15, 0, r4, c3, c0, 0 /* dacr */
	mcr     p15, 0, r3, c2, c0, 0 /* ttb */
	mcr     p15, 0, r5, c13, c0, 1 /* context ID */
	isb
	ldmdb   r1!, {r4-r14}
	/* Add 1:1 map in the PMD to allow smooth switch when turning on MMU */
	and     r3, r3, #~0x7F  /* mask off lower 7 bits of TTB */
	adr     r0, msm_pm_mapped_pa /* get address of the mapped instr */
	lsr     r1, r0, #20     /* get the addr range of addr in MB */
	lsl     r1, r1, #2      /* multiply by 4 to get to the pg index */
	add     r3, r3, r1      /* pgd + pgd_index(addr) */
	ldr     r1, [r3]        /* save current entry to r1 */
	lsr     r0, #20         /* align current addr to 1MB boundary */
	lsl     r0, #20
	/* Create new entry for this 1MB page */
	orr     r0, r0, #0x400   /* PMD_SECT_AP_WRITE */
	orr     r0, r0, #0x2     /* PMD_TYPE_SECT|PMD_DOMAIN(DOMAIN_KERNEL) */
	str     r0, [r3]         /* put new entry into the MMU table */
	mcr     p15, 0, r3, c7, c10, 1  /* flush_pmd */
	dsb
	isb
	mcr     p15, 0, r2, c1, c0, 0   /* MMU control */
	isb
msm_pm_mapped_pa:
	/* Switch to virtual */
	adr     r2, msm_pm_pa_to_va
	ldr     r0, =msm_pm_pa_to_va
	mov     pc, r0
msm_pm_pa_to_va:
	sub     r0, r0, r2
	/* Restore r1 in MMU table */
	add     r3, r3, r0
	str     r1, [r3]
	mcr     p15, 0, r3, c7, c10, 1  /* flush_pmd */
	dsb
	isb
	mov     r0, #1
	mov     pc, lr
	nop
	nop
	nop
	nop
	nop
1:	b       1b


	.data

saved_state:
	.space  4 * 11 /* r4-14 */
	.space  4 * 4  /* cp15 */
#ifdef CONFIG_VFP
	.space  8 * 32 /* VFP working registers */
	.space  4 * 2  /* VFP state registers */
#endif
saved_state_end:


#ifndef __ARCH_ARM_MACH_OMAP2_PM_H
#define __ARCH_ARM_MACH_OMAP2_PM_H
/*
 * linux/arch/arm/mach-omap2/pm.h
 *
 * OMAP Power Management Routines
 *
 * Copyright (C) 2008 Nokia Corporation
 * Jouni Hogander
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <mach/powerdomain.h>

extern int omap2_pm_init(void);
extern int omap3_pm_init(void);

extern unsigned short enable_dyn_sleep;
extern unsigned short clocks_off_while_idle;
extern atomic_t sleep_block;

extern void omap2_block_sleep(void);
extern void omap2_allow_sleep(void);


#ifdef CONFIG_PM_DEBUG
extern void omap2_pm_dump(int mode, int resume, unsigned int us);
extern int omap2_pm_debug;
extern void pm_dbg_update_time(struct powerdomain *pwrdm, int prev);
extern int pm_dbg_regset_save(int reg_set);
extern int pm_dbg_regset_init(int reg_set);
#else
#define omap2_pm_dump(mode, resume, us)		do {} while (0);
#define omap2_pm_debug				0
#define pm_dbg_update_time(pwrdm, prev) do {} while (0);
#define pm_dbg_regset_save(reg_set) do {} while (0);
#define pm_dbg_regset_init(reg_set) do {} while (0);
#endif /* CONFIG_PM_DEBUG */
#endif

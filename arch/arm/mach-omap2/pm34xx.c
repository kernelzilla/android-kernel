/*
 * linux/arch/arm/mach-omap2/pm34xx.c
 *
 * OMAP3 Power Management Routines
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * Copyright (C) 2006-2008 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 * Jouni Hogander
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_PM_VERBOSE
#define DEBUG
#endif

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/reboot.h>

#include <mach/gpio.h>
#include <mach/sram.h>
#include <mach/prcm.h>
#include <mach/clockdomain.h>
#include <mach/powerdomain.h>
#include <mach/resource.h>
#include <mach/serial.h>
#include <mach/control.h>
#include <mach/serial.h>
#include <mach/gpio.h>
#include <mach/sdrc.h>
#include <mach/dma.h>
#include <mach/gpmc.h>
#include <mach/dma.h>
#include <mach/dmtimer.h>

#include <asm/tlbflush.h>

#include <linux/delay.h>

#include "cm.h"
#include "cm-regbits-34xx.h"
#include "prm-regbits-34xx.h"

#include "prm.h"
#include "pm.h"
#include "smartreflex.h"
#include "sdrc.h"
#include <mach/omap-pm.h>

static int regset_save_on_suspend;

#ifdef CONFIG_SUSPEND
static suspend_state_t suspend_state_on;
#endif

/* Scratchpad offsets */
#define OMAP343X_TABLE_ADDRESS_OFFSET	   0x31
#define OMAP343X_TABLE_VALUE_OFFSET	   0x30
#define OMAP343X_CONTROL_REG_VALUE_OFFSET  0x32

struct power_state {
	struct powerdomain *pwrdm;
	u32 next_state;
#ifdef CONFIG_SUSPEND
	u32 saved_state;
#endif
	struct list_head node;
};

static LIST_HEAD(pwrst_list);

static void (*_omap_sram_idle)(u32 *addr, int save_state);

static int (*_omap_save_secure_sram)(u32 *addr);

static struct powerdomain *mpu_pwrdm, *neon_pwrdm;
static struct powerdomain *core_pwrdm, *per_pwrdm;
static struct powerdomain *wkup_pwrdm;
static struct powerdomain *cam_pwrdm;

static struct prm_setup_vc prm_setup = {
	.clksetup = 0xff,
	.voltsetup_time1 = 0xfff,
	.voltsetup_time2 = 0xfff,
	.voltoffset = 0xff,
	.voltsetup2 = 0xff,
	.vdd0_on = 0x30,
	.vdd0_onlp = 0x1e,
	.vdd0_ret = 0x1e,
	.vdd0_off = 0x30,
	.vdd1_on = 0x2c,
	.vdd1_onlp = 0x1e,
	.vdd1_ret = 0x1e,
	.vdd1_off = 0x2c,
	.i2c_slave_ra = (R_SRI2C_SLAVE_ADDR << OMAP3430_SMPS_SA1_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << OMAP3430_SMPS_SA0_SHIFT),
	.vdd_vol_ra = (R_VDD2_SR_CONTROL << OMAP3430_VOLRA1_SHIFT) |
			(R_VDD1_SR_CONTROL << OMAP3430_VOLRA0_SHIFT),
	/* vdd_vol_ra controls both cmd and vol, set the address equal */
	.vdd_cmd_ra = (R_VDD2_SR_CONTROL << OMAP3430_VOLRA1_SHIFT) |
			(R_VDD1_SR_CONTROL << OMAP3430_VOLRA0_SHIFT),
	.vdd_ch_conf = OMAP3430_CMD1 | OMAP3430_RAV1,
	.vdd_i2c_cfg = OMAP3430_MCODE_SHIFT | OMAP3430_HSEN | OMAP3430_SREN,
};

static inline void omap3_per_save_context(void)
{
	omap3_gpio_save_context();
}

static inline void omap3_per_restore_context(void)
{
	omap3_gpio_restore_context();
}

static void omap3_enable_io_chain(void)
{
	int timeout = 0;

	if (omap_rev() >= OMAP3430_REV_ES3_1) {
		prm_set_mod_reg_bits(OMAP3430_EN_IO_CHAIN, WKUP_MOD, PM_WKEN);
		/* Do a readback to assure write has been done */
		prm_read_mod_reg(WKUP_MOD, PM_WKEN);

		while (!(prm_read_mod_reg(WKUP_MOD, PM_WKST) &
			 OMAP3430_ST_IO_CHAIN)) {
			timeout++;
			if (timeout > 1000) {
				printk(KERN_ERR "Wake up daisy chain "
				       "activation failed.\n");
				return;
			}
		}
		prm_set_mod_reg_bits(OMAP3430_ST_IO_CHAIN,
				     WKUP_MOD, PM_WKST);
	}
}

static void omap3_disable_io_chain(void)
{
	if (omap_rev() >= OMAP3430_REV_ES3_1)
		prm_clear_mod_reg_bits(OMAP3430_EN_IO_CHAIN, WKUP_MOD, PM_WKEN);
}

static void omap3_core_save_context(void)
{
	u32 control_padconf_off;
	/* Save the padconf registers */
	control_padconf_off =
	omap_ctrl_readl(OMAP343X_CONTROL_PADCONF_OFF);
	control_padconf_off |= START_PADCONF_SAVE;
	omap_ctrl_writel(control_padconf_off, OMAP343X_CONTROL_PADCONF_OFF);
	/* wait for the save to complete */
	while (!omap_ctrl_readl(OMAP343X_CONTROL_GENERAL_PURPOSE_STATUS)
			& PADCONF_SAVE_DONE)
		;
	/* Save the Interrupt controller context */
	omap3_intc_save_context();
	/* Save the GPMC context */
	omap3_gpmc_save_context();
	/* Save the system control module context, padconf already save above*/
	omap3_control_save_context();
	omap_dma_global_context_save();
}

static void omap3_core_restore_context(void)
{
	/* Restore the control module context, padconf restored by h/w */
	omap3_control_restore_context();
	/* Restore the GPMC context */
	omap3_gpmc_restore_context();
	/* Restore the interrupt controller context */
	omap3_intc_restore_context();
	omap_dma_global_context_restore();
}

/*
 * FIXME: This function should be called before entering off-mode after
 * OMAP3 secure services have been accessed. Currently it is only called
 * once during boot sequence, but this works as we are not using secure
 * services.
 */
static void omap3_save_secure_ram_context(u32 target_mpu_state)
{
	u32 ret;

	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		/*
		 * MPU next state must be set to POWER_ON temporarily,
		 * otherwise the WFI executed inside the ROM code
		 * will hang the system.
		 */
		pwrdm_set_next_pwrst(mpu_pwrdm, PWRDM_POWER_ON);
		ret = _omap_save_secure_sram((u32 *)
				__pa(omap3_secure_ram_storage));
		pwrdm_set_next_pwrst(mpu_pwrdm, target_mpu_state);
		/* Following is for error tracking, it should not happen */
		if (ret) {
			printk(KERN_ERR "save_secure_sram() returns %08x\n",
				ret);
			while (1)
				;
		}
	}
}

#ifdef PM_DEBUG_INFO

#define LAST_IDLE_ST_ARR_SIZE 10
#define POWER_DOM_ARR_SIZE    16

static int flag_sleep_while_idle;
static int flag_enable_off_mode;
static int flag_uart_can_sleep;
static int flag_omap_dma_running;
static int pwrst_idlest[2] = {0, 0};
static int fclkst_array[8] = { 0 };
static int iclkst_array[8][2] = { {0, 0} };
static int pwrst_counter[POWER_DOM_ARR_SIZE][4] = { {0, 0, 0, 0} };
static int mpu_core_last_state[LAST_IDLE_ST_ARR_SIZE][2] = { {0, 0} };
static int modem_sad2d_idle_counter[2][2] = { {0, 0} };
static char *state_to_str[] = {"OFF", "RET", "INA", " ON"};

static void pwrdm_pre_transition_log(void)
{
	/* Read state of global setting before idle */
	flag_sleep_while_idle = enable_dyn_sleep;
	flag_enable_off_mode  = enable_off_mode;
	flag_uart_can_sleep   = omap_uart_can_sleep();
	flag_omap_dma_running = omap_dma_running();

	/* Read state of modem and sad2d before idle */
	pwrst_idlest[0] = cm_read_mod_reg(CORE_MOD, CM_IDLEST1);

	/* omap function fclk status */
	fclkst_array[0] = cm_read_mod_reg(CORE_MOD, CM_FCLKEN1);
	fclkst_array[1] = cm_read_mod_reg(CORE_MOD, OMAP3430ES2_CM_FCLKEN3);
	fclkst_array[2] = cm_read_mod_reg(OMAP3430ES2_SGX_MOD, CM_FCLKEN);
	fclkst_array[3] = cm_read_mod_reg(OMAP3430_CAM_MOD, CM_FCLKEN);
	fclkst_array[4] = cm_read_mod_reg(OMAP3430_PER_MOD, CM_FCLKEN);
	fclkst_array[5] = cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, CM_FCLKEN);
	fclkst_array[6] = cm_read_mod_reg(OMAP3430_DSS_MOD, CM_FCLKEN);

	/* omap iclk status */
	iclkst_array[0][0] = cm_read_mod_reg(CORE_MOD, CM_ICLKEN1);
	iclkst_array[0][1] = cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE1);
	iclkst_array[1][0] = cm_read_mod_reg(CORE_MOD, CM_ICLKEN2);
	iclkst_array[1][1] = cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE2);
	iclkst_array[2][0] = cm_read_mod_reg(CORE_MOD, CM_ICLKEN3);
	iclkst_array[2][1] = cm_read_mod_reg(CORE_MOD, CM_AUTOIDLE3);
	iclkst_array[3][0] = cm_read_mod_reg(OMAP3430ES2_SGX_MOD, CM_ICLKEN);
	iclkst_array[3][1] = 0;
	iclkst_array[4][0] = cm_read_mod_reg(OMAP3430_CAM_MOD, CM_ICLKEN);
	iclkst_array[4][1] = cm_read_mod_reg(OMAP3430_CAM_MOD, CM_AUTOIDLE);
	iclkst_array[5][0] = cm_read_mod_reg(OMAP3430_PER_MOD, CM_ICLKEN);
	iclkst_array[5][1] = cm_read_mod_reg(OMAP3430_PER_MOD, CM_AUTOIDLE);
	iclkst_array[6][0] = cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD,
			CM_ICLKEN);
	iclkst_array[6][1] = cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD,
			CM_AUTOIDLE);
	iclkst_array[7][0] = cm_read_mod_reg(OMAP3430_DSS_MOD, CM_ICLKEN);
	iclkst_array[7][1] = cm_read_mod_reg(OMAP3430_DSS_MOD, CM_AUTOIDLE);
}

static void pwrdm_post_transition_log(void)
{
	int i = 0;
	int idx = 0;
	int state;
	struct power_state *pwrst;

	/* Read the last 10 times MPU+CORE state by sequence */
	for (i = 0; i < LAST_IDLE_ST_ARR_SIZE-1; i++) {
		mpu_core_last_state[i][0] = mpu_core_last_state[i+1][0];
		mpu_core_last_state[i][1] = mpu_core_last_state[i+1][1];
	}
	mpu_core_last_state[i][0] = pwrdm_read_prev_pwrst(mpu_pwrdm);
	mpu_core_last_state[i][1] = pwrdm_read_prev_pwrst(core_pwrdm);

	/* counter of each power domain*/
	list_for_each_entry(pwrst, &pwrst_list, node) {
		state = pwrdm_read_prev_pwrst(pwrst->pwrdm);
		pwrst_counter[idx][state]++;
		idx++;
	}

	pwrst_idlest[1] = cm_read_mod_reg(CORE_MOD, CM_IDLEST1);

	/* modem and sad2d states counter */
	i = !!(pwrst_idlest[0] & 0x80000000);
	modem_sad2d_idle_counter[0][i]++;

	i = !!(pwrst_idlest[0] & 0x8);
	modem_sad2d_idle_counter[1][i]++;
}

#endif /* PM_DEBUG_INFO */

#ifdef CONFIG_SUSPEND
static void dump_wkst_regs(s16 module, u16 wkst_off, u32 wkst)
{
	/* only dump info that wake up from suspend */
	if (suspend_state_on != PM_SUSPEND_MEM &&
		suspend_state_on != PM_SUSPEND_STANDBY)
		return;

	if ((WKUP_MOD == module) && (PM_WKST == wkst_off))
		pr_debug("Waked up by WKUP. WKST 0x%x\n", wkst);
	else if ((CORE_MOD == module) && (PM_WKST1 == wkst_off))
		pr_debug("Waked up by CORE. WKST1 0x%x\n", wkst);
	else if ((CORE_MOD == module) && (OMAP3430ES2_PM_WKST3 == wkst_off))
		pr_debug("Waked up by CORE. WKST3 0x%x)\n", wkst);
	else if ((OMAP3430_PER_MOD == module) && (PM_WKST == wkst_off))
		pr_debug("Waked up by PER. WKST 0x%x)\n", wkst);
}
#endif /* CONFIG_SUSPEND */

/*
 * PRCM Interrupt Handler Helper Function
 *
 * The purpose of this function is to clear any wake-up events latched
 * in the PRCM PM_WKST_x registers. It is possible that a wake-up event
 * may occur whilst attempting to clear a PM_WKST_x register and thus
 * set another bit in this register. A while loop is used to ensure
 * that any peripheral wake-up events occurring while attempting to
 * clear the PM_WKST_x are detected and cleared.
 */
static void prcm_clear_mod_irqs(s16 module, u8 regs)
{
	u32 wkst, fclk, iclk, clken;
	u16 wkst_off = (regs == 3) ? OMAP3430ES2_PM_WKST3 : PM_WKST1;
	u16 fclk_off = (regs == 3) ? OMAP3430ES2_CM_FCLKEN3 : CM_FCLKEN1;
	u16 iclk_off = (regs == 3) ? CM_ICLKEN3 : CM_ICLKEN1;
	u16 grpsel_off = (regs == 3) ?
		OMAP3430ES2_PM_MPUGRPSEL3 : OMAP3430_PM_MPUGRPSEL;

	wkst = prm_read_mod_reg(module, wkst_off);
	wkst &= prm_read_mod_reg(module, grpsel_off);
	if (wkst) {
#ifdef CONFIG_SUSPEND
		dump_wkst_regs(module, wkst_off, wkst);
#endif
		iclk = cm_read_mod_reg(module, iclk_off);
		fclk = cm_read_mod_reg(module, fclk_off);
		while (wkst) {
			clken = wkst;
			cm_set_mod_reg_bits(clken, module, iclk_off);
			/*
			 * For USBHOST, we don't know whether HOST1 or
			 * HOST2 woke us up, so enable both f-clocks
			 */
			if (module == OMAP3430ES2_USBHOST_MOD)
				clken |= 1 << OMAP3430ES2_EN_USBHOST2_SHIFT;
			cm_set_mod_reg_bits(clken, module, fclk_off);
			prm_write_mod_reg(wkst, module, wkst_off);
			wkst = prm_read_mod_reg(module, wkst_off);
		}
		cm_write_mod_reg(iclk, module, iclk_off);
		cm_write_mod_reg(fclk, module, fclk_off);
	}
}

/*
 * PRCM Interrupt Handler
 *
 * The PRM_IRQSTATUS_MPU register indicates if there are any pending
 * interrupts from the PRCM for the MPU. These bits must be cleared in
 * order to clear the PRCM interrupt. The PRCM interrupt handler is
 * implemented to simply clear the PRM_IRQSTATUS_MPU in order to clear
 * the PRCM interrupt. Please note that bit 0 of the PRM_IRQSTATUS_MPU
 * register indicates that a wake-up event is pending for the MPU and
 * this bit can only be cleared if the all the wake-up events latched
 * in the various PM_WKST_x registers have been cleared. The interrupt
 * handler is implemented using a do-while loop so that if a wake-up
 * event occurred during the processing of the prcm interrupt handler
 * (setting a bit in the corresponding PM_WKST_x register and thus
 * preventing us from clearing bit 0 of the PRM_IRQSTATUS_MPU register)
 * this would be handled.
 */
static irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 irqstatus_mpu;

	do {
		prcm_clear_mod_irqs(WKUP_MOD, 1);
		prcm_clear_mod_irqs(CORE_MOD, 1);
		prcm_clear_mod_irqs(OMAP3430_PER_MOD, 1);
		if (omap_rev() > OMAP3430_REV_ES1_0) {
			prcm_clear_mod_irqs(CORE_MOD, 3);
			prcm_clear_mod_irqs(OMAP3430ES2_USBHOST_MOD, 1);
		}

		irqstatus_mpu = prm_read_mod_reg(OCP_MOD,
					OMAP2_PRM_IRQSTATUS_MPU_OFFSET);
		prm_write_mod_reg(irqstatus_mpu, OCP_MOD,
					OMAP2_PRM_IRQSTATUS_MPU_OFFSET);

	} while (prm_read_mod_reg(OCP_MOD, OMAP2_PRM_IRQSTATUS_MPU_OFFSET));

	return IRQ_HANDLED;
}

static void restore_control_register(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" : : "r" (val));
}

/* Function to restore the table entry that was modified for enabling MMU */
static void restore_table_entry(void)
{
	u32 *scratchpad_address;
	u32 previous_value, control_reg_value;
	u32 *address;
	scratchpad_address = OMAP2_IO_ADDRESS(OMAP343X_SCRATCHPAD);
	/* Get address of entry that was modified */
	address = (u32 *)__raw_readl(scratchpad_address
					+ OMAP343X_TABLE_ADDRESS_OFFSET);
	/* Get the previous value which needs to be restored */
	previous_value = __raw_readl(scratchpad_address
					+ OMAP343X_TABLE_VALUE_OFFSET);
	address = __va(address);
	*address = previous_value;
	flush_tlb_all();
	control_reg_value = __raw_readl(scratchpad_address
					+ OMAP343X_CONTROL_REG_VALUE_OFFSET);
	/* This will enable caches and prediction */
	restore_control_register(control_reg_value);
}

void omap_sram_idle(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 0;
	int mpu_next_state = PWRDM_POWER_ON;
	int per_next_state = PWRDM_POWER_ON;
	int core_next_state = PWRDM_POWER_ON;
	int core_prev_state, per_prev_state;
	u32 sdrc_pwr = 0;
	int per_state_modified = 0;

	if (!_omap_sram_idle)
		return;

	pwrdm_clear_all_prev_pwrst(mpu_pwrdm);
	pwrdm_clear_all_prev_pwrst(neon_pwrdm);
	pwrdm_clear_all_prev_pwrst(core_pwrdm);
	pwrdm_clear_all_prev_pwrst(per_pwrdm);

	mpu_next_state = pwrdm_read_next_pwrst(mpu_pwrdm);
	switch (mpu_next_state) {
	case PWRDM_POWER_ON:
	case PWRDM_POWER_RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PWRDM_POWER_OFF:
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}

#ifdef PM_DEBUG_INFO
	pwrdm_pre_transition_log();
#endif
	pwrdm_pre_transition();

	/* NEON control */
	if (pwrdm_read_pwrst(neon_pwrdm) == PWRDM_POWER_ON)
		pwrdm_set_next_pwrst(neon_pwrdm, mpu_next_state);

	/* PER */
	per_next_state = pwrdm_read_next_pwrst(per_pwrdm);
	core_next_state = pwrdm_read_next_pwrst(core_pwrdm);
	if (per_next_state < PWRDM_POWER_ON) {
		omap_uart_prepare_idle(2);
		omap2_gpio_prepare_for_idle(per_next_state);
		if (per_next_state == PWRDM_POWER_OFF) {
			if (core_next_state == PWRDM_POWER_ON) {
				per_next_state = PWRDM_POWER_RET;
				pwrdm_set_next_pwrst(per_pwrdm, per_next_state);
				per_state_modified = 1;
			} else
				omap3_per_save_context();
		}
	}

	if (pwrdm_read_pwrst(cam_pwrdm) == PWRDM_POWER_ON)
		omap2_clkdm_deny_idle(mpu_pwrdm->pwrdm_clkdms[0]);

	/*
	 * Disable smartreflex before entering WFI.
	 * Only needed if we are going to enter retention or off.
	 */
	if (core_next_state <= PWRDM_POWER_RET) {
		disable_smartreflex(SR1);
		disable_smartreflex(SR2);
	}

	/* CORE */
	if (core_next_state < PWRDM_POWER_ON ||
		mpu_next_state < PWRDM_POWER_ON) {
		omap_uart_prepare_idle(0);
		omap_uart_prepare_idle(1);
		if (core_next_state == PWRDM_POWER_OFF) {
			prm_set_mod_reg_bits(OMAP3430_AUTO_OFF,
					     OMAP3430_GR_MOD,
					     OMAP3_PRM_VOLTCTRL_OFFSET);
			omap3_core_save_context();
			omap3_prcm_save_context();
		}
		/* Enable IO-PAD and IO-CHAIN wakeups */
		prm_set_mod_reg_bits(OMAP3430_EN_IO, WKUP_MOD, PM_WKEN);
		omap3_enable_io_chain();
	}

	/*
	* On EMU/HS devices ROM code restores a SRDC value
	* from scratchpad which has automatic self refresh on timeout
	* of AUTO_CNT = 1 enabled. This takes care of errata 1.142.
	* Hence store/restore the SDRC_POWER register here.
	*/
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_pwr = sdrc_read_reg(SDRC_POWER);

	if (regset_save_on_suspend)
		pm_dbg_regset_save(1);

	/*
	 * omap3_arm_context is the location where ARM registers
	 * get saved. The restore path then reads from this
	 * location and restores them back.
	 */
	_omap_sram_idle(omap3_arm_context, save_state);
	cpu_init();

	/* Restore normal SDRC POWER settings */
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_write_reg(sdrc_pwr, SDRC_POWER);

	/* Restore table entry modified during MMU restoration */
	if (pwrdm_read_prev_pwrst(mpu_pwrdm) == PWRDM_POWER_OFF)
		restore_table_entry();


	/* CORE */
	if (core_next_state < PWRDM_POWER_ON ||
		mpu_next_state < PWRDM_POWER_ON) {
		core_prev_state = pwrdm_read_prev_pwrst(core_pwrdm);
		if (core_prev_state == PWRDM_POWER_OFF) {
			omap3_core_restore_context();
			omap3_prcm_restore_context();
			omap3_sram_restore_context();
			omap2_sms_restore_context();
		}
		omap_uart_resume_idle(0);
		omap_uart_resume_idle(1);
		if (core_next_state == PWRDM_POWER_OFF)
			prm_clear_mod_reg_bits(OMAP3430_AUTO_OFF,
					       OMAP3430_GR_MOD,
					       OMAP3_PRM_VOLTCTRL_OFFSET);
	}

	/*
	 * Enable smartreflex after WFI. Only needed if we entered
	 * retention or off
	 */
	if (core_next_state <= PWRDM_POWER_RET) {
		enable_smartreflex(SR1);
		enable_smartreflex(SR2);
	}

	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		per_prev_state = pwrdm_read_prev_pwrst(per_pwrdm);
		if (per_prev_state == PWRDM_POWER_OFF) {
			omap3_per_restore_context();
			omap3_gpio_restore_pad_context(0);
		} else if (per_next_state == PWRDM_POWER_OFF)
			omap3_gpio_restore_pad_context(1);
		omap2_gpio_resume_after_idle();
		omap_uart_resume_idle(2);
		if (per_state_modified)
			pwrdm_set_next_pwrst(per_pwrdm, PWRDM_POWER_OFF);
	}

	/* Disable IO-PAD and IO-CHAIN wakeup */
	if (core_next_state < PWRDM_POWER_ON ||
		mpu_next_state < PWRDM_POWER_ON) {
		prm_clear_mod_reg_bits(OMAP3430_EN_IO, WKUP_MOD, PM_WKEN);
		omap3_disable_io_chain();
	}


	pwrdm_post_transition();
#ifdef PM_DEBUG_INFO
	pwrdm_post_transition_log();
#endif

	omap2_clkdm_allow_idle(mpu_pwrdm->pwrdm_clkdms[0]);
}

int omap3_can_sleep(void)
{
	if (!enable_dyn_sleep)
		return 0;
	if (!omap_uart_can_sleep())
		return 0;
	if (atomic_read(&sleep_block) > 0)
		return 0;

	return 1;
}

/* This sets pwrdm state (other than mpu & core. Currently only ON &
 * RET are supported. Function is assuming that clkdm doesn't have
 * hw_sup mode enabled. */
int set_pwrdm_state(struct powerdomain *pwrdm, u32 state)
{
	u32 cur_state;
	int sleep_switch = 0;
	int ret = 0;

	if (pwrdm == NULL || IS_ERR(pwrdm))
		return -EINVAL;

	while (!(pwrdm->pwrsts & (1 << state))) {
		if (state == PWRDM_POWER_OFF)
			return ret;
		state--;
	}

	cur_state = pwrdm_read_next_pwrst(pwrdm);
	if (cur_state == state)
		return ret;

	if (pwrdm_read_pwrst(pwrdm) < PWRDM_POWER_ON) {
		omap2_clkdm_wakeup(pwrdm->pwrdm_clkdms[0]);
		sleep_switch = 1;
		pwrdm_wait_transition(pwrdm);
	}

	ret = pwrdm_set_next_pwrst(pwrdm, state);
	if (ret) {
		printk(KERN_ERR "Unable to set state of powerdomain: %s\n",
		       pwrdm->name);
		goto err;
	}

	if (sleep_switch) {
		omap2_clkdm_allow_idle(pwrdm->pwrdm_clkdms[0]);
		pwrdm_wait_transition(pwrdm);
		pwrdm_state_switch(pwrdm);
	}

err:
	return ret;
}

static void omap3_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();

	if (!omap3_can_sleep())
		goto out;

	if (omap_irq_pending() || need_resched())
		goto out;

	omap_sram_idle();

out:
	local_fiq_enable();
	local_irq_enable();
}

#ifdef CONFIG_SUSPEND
static void (*saved_idle)(void);
static suspend_state_t suspend_state;

static void omap2_pm_wakeup_on_timer(u32 seconds, u32 nseconds)
{
	u32 tick_rate, cycles;
	void __iomem *base;
	u32 cnt = 10;

	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(gptimer_wakeup));
	cycles = tick_rate * seconds;
	cycles += ((nseconds / NSEC_PER_MSEC) * tick_rate) / MSEC_PER_SEC;
	omap_dm_timer_stop(gptimer_wakeup);

	/* Cleared pending gpt1 irq if there is. */
	if (omap_dm_timer_read_status(gptimer_wakeup) & OMAP_TIMER_INT_OVERFLOW)
		omap_dm_timer_write_status(gptimer_wakeup,
				OMAP_TIMER_INT_OVERFLOW);

	/* Make sure the intc pending irq37 is automatically cleared. */
	base = OMAP2_IO_ADDRESS(OMAP34XX_IC_BASE);
	while ((__raw_readl(base + 0x00B8) & 0x20) == 0x20 && cnt) {
		udelay(1000000/tick_rate + 1);
		cnt--;
	}

	/* If the INTC gpt1 signal to MPU is still there, then finally
	 * refresh the IRQ INTC ouput to make sure it will not break wfi.
	 */
	if ((__raw_readl(base + 0x40) & 0x7F) == INT_24XX_GPTIMER1)
		__raw_writel(0x1, base + 0x0048);

	/* Not to start the timer again in airplane mode or no SIM insert */
	if (!seconds && !nseconds)
		return;

	omap_dm_timer_set_load_start(gptimer_wakeup, 0, 0xffffffff - cycles);

	pr_debug("PM: Resume timer in %d secs (%d ticks at %d ticks/sec.)\n",
		seconds, cycles, tick_rate);
}

static int omap3_pm_prepare(void)
{
	saved_idle = pm_idle;
	pm_idle = NULL;
	return 0;
}

static int omap3_pm_suspend(void)
{
	struct power_state *pwrst;
	int state, ret = 0;

	omap2_pm_wakeup_on_timer(wakeup_timer_seconds, wakeup_timer_nseconds);

	/* Read current next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node)
		pwrst->saved_state = pwrdm_read_next_pwrst(pwrst->pwrdm);
	/* Set ones wanted by suspend */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (set_pwrdm_state(pwrst->pwrdm, pwrst->next_state))
			goto restore;
		if (pwrdm_clear_all_prev_pwrst(pwrst->pwrdm))
			goto restore;
	}

	omap_uart_prepare_suspend();

	regset_save_on_suspend = 1;
	omap_sram_idle();
	regset_save_on_suspend = 0;

restore:
	/* Restore next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		state = pwrdm_read_prev_pwrst(pwrst->pwrdm);
		if (state > pwrst->next_state) {
			printk(KERN_INFO "Powerdomain (%s) didn't enter "
			       "target state %d\n",
			       pwrst->pwrdm->name, pwrst->next_state);
			ret = -1;
		}
		set_pwrdm_state(pwrst->pwrdm, pwrst->saved_state);
	}
	if (ret)
		printk(KERN_ERR "Could not enter target state in pm_suspend\n");
	else
		pr_debug(KERN_INFO "Successfully put all powerdomains "
		       "to target state\n");

	return ret;
}

static int omap3_pm_enter(suspend_state_t unused)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap3_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap3_pm_finish(void)
{
	pm_idle = saved_idle;
}

/* Hooks to enable / disable UART interrupts during suspend */
static int omap3_pm_begin(suspend_state_t state)
{
	suspend_state = state;
	omap_uart_enable_irqs(0);
	suspend_state_on = suspend_state;
	return 0;
}

static void omap3_pm_end(void)
{
	suspend_state = PM_SUSPEND_ON;
	omap_uart_enable_irqs(1);
	suspend_state_on = suspend_state;
	return;
}

static struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap3_pm_begin,
	.end		= omap3_pm_end,
	.prepare	= omap3_pm_prepare,
	.enter		= omap3_pm_enter,
	.finish		= omap3_pm_finish,
	.valid		= suspend_valid_only_mem,
};
#endif /* CONFIG_SUSPEND */


/**
 * omap3_iva_idle(): ensure IVA is in idle so it can be put into
 *                   retention
 *
 * In cases where IVA2 is activated by bootcode, it may prevent
 * full-chip retention or off-mode because it is not idle.  This
 * function forces the IVA2 into idle state so it can go
 * into retention/off and thus allow full-chip retention/off.
 *
 **/
static void __init omap3_iva_idle(void)
{
	/* ensure IVA2 clock is disabled */
	cm_write_mod_reg(0, OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* if no clock activity, nothing else to do */
	if (!(cm_read_mod_reg(OMAP3430_IVA2_MOD, OMAP3430_CM_CLKSTST) &
	      OMAP3430_CLKACTIVITY_IVA2_MASK))
		return;

	/* Reset IVA2 */
	prm_write_mod_reg(OMAP3430_RST1_IVA2 |
			  OMAP3430_RST2_IVA2 |
			  OMAP3430_RST3_IVA2,
			  OMAP3430_IVA2_MOD, RM_RSTCTRL);

	/* Enable IVA2 clock */
	cm_write_mod_reg(OMAP3430_CM_FCLKEN_IVA2_EN_IVA2,
			 OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* Set IVA2 boot mode to 'idle' */
	omap_ctrl_writel(OMAP3_IVA2_BOOTMOD_IDLE,
			 OMAP343X_CONTROL_IVA2_BOOTMOD);

	/* Un-reset IVA2 */
	prm_write_mod_reg(0, OMAP3430_IVA2_MOD, RM_RSTCTRL);

	/* Disable IVA2 clock */
	cm_write_mod_reg(0, OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* Reset IVA2 */
	prm_write_mod_reg(OMAP3430_RST1_IVA2 |
			  OMAP3430_RST2_IVA2 |
			  OMAP3430_RST3_IVA2,
			  OMAP3430_IVA2_MOD, RM_RSTCTRL);
}

static void __init prcm_setup_regs(void)
{
	/* XXX Reset all wkdeps. This should be done when initializing
	 * powerdomains */
	prm_write_mod_reg(0, OMAP3430_IVA2_MOD, PM_WKDEP);
	prm_write_mod_reg(0, MPU_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_DSS_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_NEON_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_CAM_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_PER_MOD, PM_WKDEP);
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		prm_write_mod_reg(0, OMAP3430ES2_SGX_MOD, PM_WKDEP);
		prm_write_mod_reg(0, OMAP3430ES2_USBHOST_MOD, PM_WKDEP);
		prm_write_mod_reg(0, OMAP3430ES2_USBHOST_MOD, OMAP3430_PM_IVAGRPSEL);
	} else
		prm_write_mod_reg(0, GFX_MOD, PM_WKDEP);

	/*
	 * Enable interface clock autoidle for all modules.
	 * Note that in the long run this should be done by clockfw
	 */
	cm_write_mod_reg(
		OMAP3430_AUTO_MODEM |
		OMAP3430ES2_AUTO_MMC3 |
		OMAP3430ES2_AUTO_ICR |
		OMAP3430_AUTO_AES2 |
		OMAP3430_AUTO_SHA12 |
		OMAP3430_AUTO_DES2 |
		OMAP3430_AUTO_MMC2 |
		OMAP3430_AUTO_MMC1 |
		OMAP3430_AUTO_MSPRO |
		OMAP3430_AUTO_HDQ |
		OMAP3430_AUTO_MCSPI4 |
		OMAP3430_AUTO_MCSPI3 |
		OMAP3430_AUTO_MCSPI2 |
		OMAP3430_AUTO_MCSPI1 |
		OMAP3430_AUTO_I2C3 |
		OMAP3430_AUTO_I2C2 |
		OMAP3430_AUTO_I2C1 |
		OMAP3430_AUTO_UART2 |
		OMAP3430_AUTO_UART1 |
		OMAP3430_AUTO_GPT11 |
		OMAP3430_AUTO_GPT10 |
		OMAP3430_AUTO_MCBSP5 |
		OMAP3430_AUTO_MCBSP1 |
		OMAP3430ES1_AUTO_FAC | /* This is es1 only */
		OMAP3430_AUTO_MAILBOXES |
		OMAP3430_AUTO_OMAPCTRL |
		OMAP3430ES1_AUTO_FSHOSTUSB |
		OMAP3430_AUTO_HSOTGUSB |
		OMAP3430_AUTO_SAD2D |
		OMAP3430_AUTO_SSI,
		CORE_MOD, CM_AUTOIDLE1);

	prm_rmw_mod_reg_bits(0x80000008,
		0,
		CORE_MOD,
		OMAP3430_PM_MPUGRPSEL1);

	cm_write_mod_reg(
		OMAP3430_AUTO_PKA |
		OMAP3430_AUTO_AES1 |
		OMAP3430_AUTO_RNG |
		OMAP3430_AUTO_SHA11 |
		OMAP3430_AUTO_DES1,
		CORE_MOD, CM_AUTOIDLE2);

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		cm_write_mod_reg(
			OMAP3430_AUTO_MAD2D |
			OMAP3430ES2_AUTO_USBTLL,
			CORE_MOD, CM_AUTOIDLE3);
	}

	cm_write_mod_reg(
		OMAP3430_AUTO_WDT2 |
		OMAP3430_AUTO_WDT1 |
		OMAP3430_AUTO_GPIO1 |
		OMAP3430_AUTO_32KSYNC |
		OMAP3430_AUTO_GPT12 |
		OMAP3430_AUTO_GPT1 ,
		WKUP_MOD, CM_AUTOIDLE);

	cm_write_mod_reg(
		OMAP3430_AUTO_DSS,
		OMAP3430_DSS_MOD,
		CM_AUTOIDLE);

	cm_write_mod_reg(
		OMAP3430_AUTO_CAM,
		OMAP3430_CAM_MOD,
		CM_AUTOIDLE);

	cm_write_mod_reg(
		OMAP3430_AUTO_GPIO6 |
		OMAP3430_AUTO_GPIO5 |
		OMAP3430_AUTO_GPIO4 |
		OMAP3430_AUTO_GPIO3 |
		OMAP3430_AUTO_GPIO2 |
		OMAP3430_AUTO_WDT3 |
		OMAP3430_AUTO_UART3 |
		OMAP3430_AUTO_GPT9 |
		OMAP3430_AUTO_GPT8 |
		OMAP3430_AUTO_GPT7 |
		OMAP3430_AUTO_GPT6 |
		OMAP3430_AUTO_GPT5 |
		OMAP3430_AUTO_GPT4 |
		OMAP3430_AUTO_GPT3 |
		OMAP3430_AUTO_GPT2 |
		OMAP3430_AUTO_MCBSP4 |
		OMAP3430_AUTO_MCBSP3 |
		OMAP3430_AUTO_MCBSP2,
		OMAP3430_PER_MOD,
		CM_AUTOIDLE);

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		cm_write_mod_reg(
			OMAP3430ES2_AUTO_USBHOST,
			OMAP3430ES2_USBHOST_MOD,
			CM_AUTOIDLE);
	}

	/*
	 * Set all plls to autoidle. This is needed until autoidle is
	 * enabled by clockfw
	 */
	cm_write_mod_reg(1 << OMAP3430_AUTO_IVA2_DPLL_SHIFT,
			 OMAP3430_IVA2_MOD, CM_AUTOIDLE2);
	cm_write_mod_reg(1 << OMAP3430_AUTO_MPU_DPLL_SHIFT,
			 MPU_MOD,
			 CM_AUTOIDLE2);
	cm_write_mod_reg((1 << OMAP3430_AUTO_PERIPH_DPLL_SHIFT) |
			 (1 << OMAP3430_AUTO_CORE_DPLL_SHIFT),
			 PLL_MOD,
			 CM_AUTOIDLE);
	cm_write_mod_reg(1 << OMAP3430ES2_AUTO_PERIPH2_DPLL_SHIFT,
			 PLL_MOD,
			 CM_AUTOIDLE2);

	/*
	 * Enable control of expternal oscillator through
	 * sys_clkreq. In the long run clock framework should
	 * take care of this.
	 */
	prm_rmw_mod_reg_bits(OMAP_AUTOEXTCLKMODE_MASK,
			     1 << OMAP_AUTOEXTCLKMODE_SHIFT,
			     OMAP3430_GR_MOD,
			     OMAP3_PRM_CLKSRC_CTRL_OFFSET);

	/* setup wakup source */
	prm_write_mod_reg(OMAP3430_EN_IO | OMAP3430_EN_GPIO1 |
			  OMAP3430_EN_GPT1 | OMAP3430_EN_GPT12,
			  WKUP_MOD, PM_WKEN);

	/* No need to write EN_IO, that is always enabled */
	prm_write_mod_reg(OMAP3430_EN_GPIO1 | OMAP3430_EN_GPT1 |
			  OMAP3430_EN_GPT12,
			  WKUP_MOD, OMAP3430_PM_MPUGRPSEL);
	/* For some reason IO doesn't generate wakeup event even if
	 * it is selected to mpu wakeup goup */
	prm_write_mod_reg(OMAP3430_IO_EN | OMAP3430_WKUP_EN,
			OCP_MOD, OMAP2_PRM_IRQENABLE_MPU_OFFSET);

	/* Don't attach IVA interrupts */
	prm_write_mod_reg(0, WKUP_MOD, OMAP3430_PM_IVAGRPSEL);
	prm_write_mod_reg(0, CORE_MOD, OMAP3430_PM_IVAGRPSEL1);
	prm_write_mod_reg(0, CORE_MOD, OMAP3430ES2_PM_IVAGRPSEL3);
	prm_write_mod_reg(0, OMAP3430_PER_MOD, OMAP3430_PM_IVAGRPSEL);
		
	/* Clear any pending 'reset' flags */
	prm_write_mod_reg(0xffffffff, MPU_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, CORE_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_PER_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_EMU_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_NEON_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_DSS_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430ES2_USBHOST_MOD, RM_RSTST);

	/* Clear any pending PRCM interrupts */
	prm_write_mod_reg(0, OCP_MOD, OMAP2_PRM_IRQSTATUS_MPU_OFFSET);

	/* Initialize Domains PREPWSTST to ON */
	pwrdm_clear_all_prev_pwrst(mpu_pwrdm);
	pwrdm_clear_all_prev_pwrst(neon_pwrdm);
	pwrdm_clear_all_prev_pwrst(core_pwrdm);
	pwrdm_clear_all_prev_pwrst(per_pwrdm);

	omap3_iva_idle();
}

void omap3_pm_off_mode_enable(int enable)
{
	struct power_state *pwrst;
	u32 state;

	if (enable)
		state = PWRDM_POWER_OFF;
	else
		state = PWRDM_POWER_RET;

#ifdef CONFIG_OMAP_PM_SRF
	resource_lock_opp(VDD1_OPP);
	resource_lock_opp(VDD2_OPP);
	if (resource_refresh())
		printk(KERN_ERR "Error: could not refresh resources\n");
	resource_unlock_opp(VDD1_OPP);
	resource_unlock_opp(VDD2_OPP);
#endif
	list_for_each_entry(pwrst, &pwrst_list, node) {
		pwrst->next_state = state;
		/* Temporarily disable PER OFF mode*/
		if (!strcmp(pwrst->pwrdm->name, "per_pwrdm"))
			pwrst->next_state = PWRDM_POWER_RET;
		/* Temporarily disable DSS OFF mode*/
		if (!strcmp(pwrst->pwrdm->name, "dss_pwrdm"))
			pwrst->next_state = PWRDM_POWER_RET;
		/* Temporarily disable CORE OFF mode*/
		if (!strcmp(pwrst->pwrdm->name, "core_pwrdm"))
			pwrst->next_state = PWRDM_POWER_RET;

		set_pwrdm_state(pwrst->pwrdm, state);
	}
}

int omap3_pm_get_suspend_state(struct powerdomain *pwrdm)
{
	struct power_state *pwrst;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (pwrst->pwrdm == pwrdm)
			return pwrst->next_state;
	}
	return -EINVAL;
}

int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state)
{
	struct power_state *pwrst;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (pwrst->pwrdm == pwrdm) {
			pwrst->next_state = state;
			return 0;
		}
	}
	return -EINVAL;
}

void omap3_set_prm_setup_vc(struct prm_setup_vc *setup_vc)
{
	prm_setup.clksetup = setup_vc->clksetup;
	prm_setup.voltsetup_time1 = setup_vc->voltsetup_time1;
	prm_setup.voltsetup_time2 = setup_vc->voltsetup_time2;
	prm_setup.voltoffset = setup_vc->voltoffset;
	prm_setup.voltsetup2 = setup_vc->voltsetup2;
	prm_setup.vdd0_on = setup_vc->vdd0_on;
	prm_setup.vdd0_onlp = setup_vc->vdd0_onlp;
	prm_setup.vdd0_ret = setup_vc->vdd0_ret;
	prm_setup.vdd0_off = setup_vc->vdd0_off;
	prm_setup.vdd1_on = setup_vc->vdd1_on;
	prm_setup.vdd1_onlp = setup_vc->vdd1_onlp;
	prm_setup.vdd1_ret = setup_vc->vdd1_ret;
	prm_setup.vdd1_off = setup_vc->vdd1_off;
	prm_setup.i2c_slave_ra = setup_vc->i2c_slave_ra;
	prm_setup.vdd_vol_ra = setup_vc->vdd_vol_ra;
	prm_setup.vdd_cmd_ra = setup_vc->vdd_cmd_ra;
	prm_setup.vdd_ch_conf = setup_vc->vdd_ch_conf;
	prm_setup.vdd_i2c_cfg = setup_vc->vdd_i2c_cfg;
}

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *unused)
{
	struct power_state *pwrst;

	if (!pwrdm->pwrsts)
		return 0;

	pwrst = kmalloc(sizeof(struct power_state), GFP_KERNEL);
	if (!pwrst)
		return -ENOMEM;
	pwrst->pwrdm = pwrdm;
	pwrst->next_state = PWRDM_POWER_RET;
	list_add(&pwrst->node, &pwrst_list);

	if (pwrdm_has_hdwr_sar(pwrdm))
		pwrdm_enable_hdwr_sar(pwrdm);

	return set_pwrdm_state(pwrst->pwrdm, pwrst->next_state);
}

/*
 * Enable hw supervised mode for all clockdomains if it's
 * supported. Initiate sleep transition for other clockdomains, if
 * they are not used
 */
static int __init clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	if (clkdm->flags & CLKDM_CAN_ENABLE_AUTO)
		omap2_clkdm_allow_idle(clkdm);
	else if (clkdm->flags & CLKDM_CAN_FORCE_SLEEP &&
		 atomic_read(&clkdm->usecount) == 0)
		omap2_clkdm_sleep(clkdm);
	return 0;
}

void omap_push_sram_idle(void)
{
	_omap_sram_idle = omap_sram_push(omap34xx_cpu_suspend,
					omap34xx_cpu_suspend_sz);
	if (omap_type() != OMAP2_DEVICE_TYPE_GP)
		_omap_save_secure_sram = omap_sram_push(save_secure_ram_context,
				save_secure_ram_context_sz);
}

#ifdef PM_DEBUG_INFO
static ssize_t pm_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int i = 0;
	int len = 0;
	int idx = 0;
	struct power_state *pwrst;

	len += sprintf(buf + len, "*************** PM info ****************\n");
	len += sprintf(buf + len, "sleep_while_idle:    %s\n",
			flag_sleep_while_idle ? "Enabled" : "Disabled");
	len += sprintf(buf + len, "enable_off_mode:     %s\n",
			flag_enable_off_mode ? "Enabled" : "Disabled");
	len += sprintf(buf + len, "omap_uart_can_sleep: %s\n",
			flag_uart_can_sleep ? "YES" : "NO");
	len += sprintf(buf + len, "dma status check:    %s\n",
			flag_omap_dma_running ? "Running" : "Not Running");

	/* last 10 timers power domain status */
	len += sprintf(buf + len, "\nLast 10 times state (MPU+CORE):\n");
	for (i = 0; i < LAST_IDLE_ST_ARR_SIZE; i++) {
		len += sprintf(buf + len, "#%d. MPU: %s CORE: %s\n",
			i, state_to_str[mpu_core_last_state[i][0]],
			state_to_str[mpu_core_last_state[i][1]]);
	}

	/* counter of each power domain*/
	len += sprintf(buf + len, "\nPowerdomains state statistic:\n");
	list_for_each_entry(pwrst, &pwrst_list, node) {
		len += sprintf(buf + len, "%s: OFF: %d RET: %d "
				"INA: %d ON: %d\n",
				pwrst->pwrdm->name,
				pwrst_counter[idx][PWRDM_POWER_OFF],
				pwrst_counter[idx][PWRDM_POWER_RET],
				pwrst_counter[idx][PWRDM_POWER_INACTIVE],
				pwrst_counter[idx][PWRDM_POWER_ON]);
		idx++;
	}

	/* modem and sad2d latest idle states */
	len += sprintf(buf + len, "\nModem & sad2d Last idle state:\n");
	len += sprintf(buf + len, "Before LPM: modem %s, sad2d %s\n",
			(pwrst_idlest[0] & 0x80000000) ? "idle" : "active",
			(pwrst_idlest[0] & 0x00000008) ? "idle" : "active");

	len += sprintf(buf + len, "After  LPM: modem %s, sad2d %s\n",
			(pwrst_idlest[1] & 0x80000000) ? "idle" : "active",
			(pwrst_idlest[1] & 0x00000008) ? "idle" : "active");

	/* modem and sad2d idle statistic */
	len += sprintf(buf + len, "\nModem & sad2d states statistic:\n");
	len += sprintf(buf + len, "modem: idle: %d, active: %d\n",
			modem_sad2d_idle_counter[0][1],
			modem_sad2d_idle_counter[0][0]);
	len += sprintf(buf + len, "sad2d: idle: %d, active: %d\n",
			modem_sad2d_idle_counter[1][1],
			modem_sad2d_idle_counter[1][0]);

	/* omap function fclk status */
	len += sprintf(buf + len, "\nOmap fclk checking: Before LPM\n");
	len += sprintf(buf + len, "CM_FCLKEN1_CORE   0x%x\n", fclkst_array[0]);
	len += sprintf(buf + len, "CM_FCLKEN3_CORE   0x%x\n", fclkst_array[1]);
	len += sprintf(buf + len, "CM_FCLKEN_SGX     0x%x\n", fclkst_array[2]);
	len += sprintf(buf + len, "CM_FCLKEN_CAM     0x%x\n", fclkst_array[3]);
	len += sprintf(buf + len, "CM_FCLKEN_PER     0x%x\n", fclkst_array[4]);
	len += sprintf(buf + len, "CM_FCLKEN_USBHOST 0x%x\n", fclkst_array[5]);
	len += sprintf(buf + len, "CM_FCLKEN_DSS     0x%x\n", fclkst_array[6]);

	len += sprintf(buf + len, "\nOmap iclk checking:  Before LPM\n");

	/* omap function iclk status */
	len += sprintf(buf + len, "CM_ICLKEN1_CORE   0x%x, CM_AUTOIDLE1 0x%x\n",
			iclkst_array[0][0], iclkst_array[0][1]);
	len += sprintf(buf + len, "CM_ICLKEN2_CORE   0x%x, CM_AUTOIDLE2 0x%x\n",
			iclkst_array[1][0], iclkst_array[1][1]);
	len += sprintf(buf + len, "CM_ICLKEN3_CORE   0x%x, CM_AUTOIDLE3 0x%x\n",
			iclkst_array[2][0], iclkst_array[2][1]);
	len += sprintf(buf + len, "CM_ICLKEN_SGX     0x%x\n",
			iclkst_array[3][0]);
	len += sprintf(buf + len, "CM_ICLKEN_CAM     0x%x, CM_AUTOIDLE 0x%x\n",
			iclkst_array[4][0], iclkst_array[4][1]);
	len += sprintf(buf + len, "CM_ICLKEN_PER     0x%x, CM_AUTOIDLE 0x%x\n",
			iclkst_array[5][0], iclkst_array[5][1]);
	len += sprintf(buf + len, "CM_ICLKEN_USBHOST 0x%x, CM_AUTOIDLE 0x%x\n",
			iclkst_array[6][0], iclkst_array[6][1]);
	len += sprintf(buf + len, "CM_ICLKEN_DSS     0x%x, CM_AUTOIDLE 0x%x\n",
			iclkst_array[7][0], iclkst_array[7][1]);

	len += sprintf(buf + len, "\n************ PM info End ************\n");

	WARN_ON(len > PAGE_SIZE);
	return len;
}

static struct kobj_attribute pm_info_attr =
__ATTR(pm_info, 0444, pm_info_show, NULL);

#endif /* PM_DEBUG_INFO */

#ifdef CONFIG_OMAP_PM_SRF
static void set_opps_max(void)
{
	resource_set_opp_level(VDD2_OPP, MAX_VDD2_OPP, OPP_IGNORE_LOCK);
	resource_set_opp_level(VDD1_OPP, MAX_VDD1_OPP, OPP_IGNORE_LOCK);
	return;
}
#else
static void set_opps_max(void)
{
	return;
}
#endif

static int prcm_prepare_reboot(struct notifier_block *this, unsigned long code,
					void *x)
{
	if ((code == SYS_DOWN) || (code == SYS_HALT) ||
		(code == SYS_POWER_OFF)) {
		set_opps_max();
	}
	return NOTIFY_DONE;
}

static struct notifier_block prcm_notifier = {
	.notifier_call	= prcm_prepare_reboot,
	.next		= NULL,
	.priority	= INT_MAX,
};

static int panic_prepare_reboot(struct notifier_block *this,
					unsigned long code, void *x)
{
	set_opps_max();
	return NOTIFY_DONE;
}

static struct notifier_block prcm_panic_notifier = {
	.notifier_call	= panic_prepare_reboot,
	.next		= NULL,
	.priority	= INT_MAX,
};

int __init omap3_pm_init(void)
{
	struct power_state *pwrst, *tmp;
	int ret;

	printk(KERN_ERR "Power Management for TI OMAP3.\n");

	/* XXX prcm_setup_regs needs to be before enabling hw
	 * supervised mode for powerdomains */
	prcm_setup_regs();

	ret = request_irq(INT_34XX_PRCM_MPU_IRQ,
			  (irq_handler_t)prcm_interrupt_handler,
			  IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
		       INT_34XX_PRCM_MPU_IRQ);
		goto err1;
	}

	ret = pwrdm_for_each(pwrdms_setup, NULL);
	if (ret) {
		printk(KERN_ERR "Failed to setup powerdomains\n");
		goto err2;
	}

	(void) clkdm_for_each(clkdms_setup, NULL);

	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");
	if (mpu_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get mpu_pwrdm\n");
		goto err2;
	}

	neon_pwrdm = pwrdm_lookup("neon_pwrdm");
	per_pwrdm = pwrdm_lookup("per_pwrdm");
	core_pwrdm = pwrdm_lookup("core_pwrdm");
	cam_pwrdm = pwrdm_lookup("cam_pwrdm");
	wkup_pwrdm = pwrdm_lookup("wkup_pwrdm");

	omap_push_sram_idle();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&omap_pm_ops);
#endif /* CONFIG_SUSPEND */

	pm_idle = omap3_pm_idle;

	omap3_idle_init();

	pwrdm_add_wkdep(neon_pwrdm, mpu_pwrdm);
	/*
	 * REVISIT: This wkdep is only necessary when GPIO2-6 are enabled for
	 * IO-pad wakeup.  Otherwise it will unnecessarily waste power
	 * waking up PER with every CORE wakeup - see
	 * http://marc.info/?l=linux-omap&m=121852150710062&w=2
	*/
	/*pwrdm_add_wkdep(per_pwrdm, core_pwrdm);*/
	/*
	 * A part of the fix for errata 1.158.
	 * GPIO pad spurious transition (glitch/spike) upon wakeup
	 * from SYSTEM OFF mode. The remaining fix is in:
	 * omap3_gpio_save_context, omap3_gpio_restore_context.
	 */
	if (omap_rev() <= OMAP3430_REV_ES3_1)
		pwrdm_add_wkdep(per_pwrdm, wkup_pwrdm);

	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		omap3_secure_ram_storage =
			kmalloc(0x803F, GFP_KERNEL);
		if (!omap3_secure_ram_storage)
			printk(KERN_ERR "Memory allocation failed when"
					"allocating for secure sram context\n");

		local_irq_disable();
		local_fiq_disable();

		omap_dma_global_context_save();
		omap3_save_secure_ram_context(PWRDM_POWER_ON);
		omap_dma_global_context_restore();

		local_irq_enable();
		local_fiq_enable();
	}

#ifdef PM_DEBUG_INFO
	if (sysfs_create_file(power_kobj, &pm_info_attr.attr))
		printk(KERN_ERR "sysfs_create_file failed: pm_info\n");
#endif

	omap3_save_scratchpad_contents();
	register_reboot_notifier(&prcm_notifier);
	atomic_notifier_chain_register(&panic_notifier_list,
					&prcm_panic_notifier);
err1:
	return ret;
err2:
	free_irq(INT_34XX_PRCM_MPU_IRQ, NULL);
	list_for_each_entry_safe(pwrst, tmp, &pwrst_list, node) {
		list_del(&pwrst->node);
		kfree(pwrst);
	}
	return ret;
}

/* Program Power IC via bypass interface */
int omap3_bypass_cmd(u8 slave_addr, u8 reg_addr, u8 cmd) {
	u32 vc_bypass_value;
	u32 loop_cnt = 0, retries_cnt = 0;

	vc_bypass_value = (cmd << OMAP3430_DATA_SHIFT) |
			(reg_addr << OMAP3430_REGADDR_SHIFT) |
			(slave_addr << OMAP3430_SLAVEADDR_SHIFT);

	prm_write_mod_reg(vc_bypass_value, OMAP3430_GR_MOD,
			OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	vc_bypass_value = prm_set_mod_reg_bits(OMAP3430_VALID, OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	while ((vc_bypass_value & OMAP3430_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			printk(KERN_ERR"Loop count exceeded in check SR I2C"
								"write\n");
			return 1;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);
	}

	return 0;
}

static void __init configure_vc(void)
{
	prm_write_mod_reg(prm_setup.i2c_slave_ra, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_SMPS_SA_OFFSET);
	prm_write_mod_reg(prm_setup.vdd_vol_ra, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_SMPS_VOL_RA_OFFSET);

	/* Only set if power_ic has different voltage and cmd addrs */
	if (prm_setup.vdd_vol_ra != prm_setup.vdd_cmd_ra)
		prm_write_mod_reg(prm_setup.vdd_cmd_ra, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_SMPS_CMD_RA_OFFSET);

	prm_write_mod_reg((prm_setup.vdd0_on << OMAP3430_VC_CMD_ON_SHIFT) |
		(prm_setup.vdd0_onlp << OMAP3430_VC_CMD_ONLP_SHIFT) |
		(prm_setup.vdd0_ret << OMAP3430_VC_CMD_RET_SHIFT) |
		(prm_setup.vdd0_off << OMAP3430_VC_CMD_OFF_SHIFT),
		OMAP3430_GR_MOD, OMAP3_PRM_VC_CMD_VAL_0_OFFSET);

	prm_write_mod_reg((prm_setup.vdd1_on << OMAP3430_VC_CMD_ON_SHIFT) |
		(prm_setup.vdd1_onlp << OMAP3430_VC_CMD_ONLP_SHIFT) |
		(prm_setup.vdd1_ret << OMAP3430_VC_CMD_RET_SHIFT) |
		(prm_setup.vdd1_off << OMAP3430_VC_CMD_OFF_SHIFT),
		OMAP3430_GR_MOD, OMAP3_PRM_VC_CMD_VAL_1_OFFSET);

	prm_write_mod_reg(prm_setup.vdd_ch_conf, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CH_CONF_OFFSET);

	prm_write_mod_reg(prm_setup.vdd_i2c_cfg, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_I2C_CFG_OFFSET);

	/* Setup value for voltctrl */
	prm_write_mod_reg(OMAP3430_AUTO_RET,
			  OMAP3430_GR_MOD, OMAP3_PRM_VOLTCTRL_OFFSET);

	/* Write setup times */
	prm_write_mod_reg(prm_setup.clksetup, OMAP3430_GR_MOD,
			OMAP3_PRM_CLKSETUP_OFFSET);
	prm_write_mod_reg((prm_setup.voltsetup_time2 <<
			OMAP3430_SETUP_TIME2_SHIFT) |
			(prm_setup.voltsetup_time1 <<
			OMAP3430_SETUP_TIME1_SHIFT),
			OMAP3430_GR_MOD, OMAP3_PRM_VOLTSETUP1_OFFSET);

	prm_write_mod_reg(prm_setup.voltoffset, OMAP3430_GR_MOD,
			OMAP3_PRM_VOLTOFFSET_OFFSET);
	prm_write_mod_reg(prm_setup.voltsetup2, OMAP3430_GR_MOD,
			OMAP3_PRM_VOLTSETUP2_OFFSET);

	pm_dbg_regset_init(1);
}

static int __init omap3_pm_early_init(void)
{
	prm_clear_mod_reg_bits(OMAP3430_OFFMODE_POL, OMAP3430_GR_MOD,
				OMAP3_PRM_POLCTRL_OFFSET);

	configure_vc();

	return 0;
}

arch_initcall(omap3_pm_early_init);

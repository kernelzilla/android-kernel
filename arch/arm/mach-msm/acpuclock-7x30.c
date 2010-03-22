/*
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/sort.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>

#include "smd_private.h"
#include "clock.h"
#include "acpuclock.h"
#include "socinfo.h"

#define SCSS_CLK_CTL_ADDR	(MSM_ACC_BASE + 0x04)
#define SCSS_CLK_SEL_ADDR	(MSM_ACC_BASE + 0x08)
#define SAW_VCTL		(MSM_SAW_BASE + 0x08)
#define SAW_STS			(MSM_SAW_BASE + 0x0C)
#define SAW_SPM_CTL		(MSM_SAW_BASE + 0x14)

#define dprintk(msg...) \
	cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, "cpufreq-msm", msg)

#define VREF_SEL_SHIFT	5
#define VREF_SEL_VAL	7

/* mv = (750mV + (raw*25mV))*(2-VREF_SEL_VAL)) */
#define VDD_RAW(_mv) ((((_mv) - 750) / 25) | (VREF_SEL_VAL << VREF_SEL_SHIFT))

#define MAX_AXI_KHZ 192000

struct clock_state {
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			vdd_switch_time_us;
};

struct clkctl_acpu_speed {
	unsigned int	acpu_clk_khz;
	int		src;
	unsigned int	acpu_src_sel;
	unsigned int	acpu_src_div;
	unsigned int	axi_clk_khz;
	unsigned int	vdd_mv;
	unsigned int	vdd_raw;
	unsigned long	lpj; /* loops_per_jiffy */
};

static struct clock_state drv_state = { 0 };

static struct cpufreq_frequency_table freq_table[] = {
	{ 0, 122880 },
	{ 1, 245760 },
	{ 2, 368640 },
	{ 3, 768000 },
	/* 806.4MHz is updated to 1024MHz at runtime for QSD8x55. */
	{ 4, 806400 },
	{ 5, CPUFREQ_TABLE_END },
};

/* Use negative numbers for sources that can't be enabled/disabled */
#define SRC_LPXO (-2)
#define SRC_AXI  (-1)
static struct clkctl_acpu_speed acpu_freq_tbl[] = {
	{ 24576,  SRC_LPXO, 0, 0,  30720,  1200, VDD_RAW(1200) },
	{ 61440,  PLL_3,    5, 11, 61440,  1200, VDD_RAW(1200) },
	{ 122880, PLL_3,    5, 5,  61440,  1200, VDD_RAW(1200) },
	{ 184320, PLL_3,    5, 4,  61440,  1200, VDD_RAW(1200) },
	{ MAX_AXI_KHZ, SRC_AXI, 1, 0, 61440, 1200, VDD_RAW(1200) },
	{ 245760, PLL_3,    5, 2,  61440,  1200, VDD_RAW(1200) },
	{ 368640, PLL_3,    5, 1,  122800, 1200, VDD_RAW(1200) },
	{ 768000, PLL_1,    2, 0,  153600, 1200, VDD_RAW(1200) },
	/* ACPU >= 806.4MHz requires MSMC1 @ 1.2V. Voting for
	 * AXI @ 192MHz accomplishes this implicitly. 806.4MHz
	 * is updated to 1024MHz at runtime for QSD8x55. */
	{ 806400, PLL_2,    3, 0,  192000, 1200, VDD_RAW(1200) },
	{ 0 }
};

#define POWER_COLLAPSE_KHZ MAX_AXI_KHZ
unsigned long acpuclk_power_collapse(void)
{
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(POWER_COLLAPSE_KHZ, SETRATE_PC);
	return ret;
}

#define WAIT_FOR_IRQ_KHZ MAX_AXI_KHZ
unsigned long acpuclk_wait_for_irq(void)
{
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(WAIT_FOR_IRQ_KHZ, SETRATE_SWFI);
	return ret;
}

#define STS_PMIC_DATA_SHIFT	10
#define STS_PMIC_DATA_VDD_MASK	(0x3F << STS_PMIC_DATA_SHIFT)
#define VCTL_PMIC_DATA_VDD_MASK	0x3F
#define PMIC_STATE_MASK		(0x3 << 20)
#define PMIC_STATE_IDLE		0
static int acpuclk_set_acpu_vdd(struct clkctl_acpu_speed *s)
{
	uint32_t reg_val, cur_raw_vdd;
	uint32_t timeout_count = 5;

	/* Set VDD. */
	reg_val = readl(SAW_VCTL);
	reg_val &= ~(VCTL_PMIC_DATA_VDD_MASK);
	reg_val |= s->vdd_raw;
	writel(reg_val, SAW_VCTL);

	/* Wait for PMIC to set VDD. Use timeout to detect unconfigured SAW. */
	while ((readl(SAW_STS) & PMIC_STATE_MASK) != PMIC_STATE_IDLE
			&& timeout_count > 0) {
		udelay(10);
		timeout_count--;
	}

	/* Read set voltage to check for success. */
	cur_raw_vdd = (readl(SAW_STS) & STS_PMIC_DATA_VDD_MASK)
				>> STS_PMIC_DATA_SHIFT;
	if (timeout_count == 0 || cur_raw_vdd != s->vdd_raw)
		return -EIO;

	/* Wait for voltage to stabilize. */
	udelay(drv_state.vdd_switch_time_us);

	return 0;
}

/* Set clock source and divider given a clock speed */
static void acpuclk_set_src(const struct clkctl_acpu_speed *s)
{
	uint32_t reg_clksel, reg_clkctl, src_sel;

	reg_clksel = readl(SCSS_CLK_SEL_ADDR);

	/* CLK_SEL_SRC1NO */
	src_sel = reg_clksel & 1;

	/* Program clock source and divider. */
	reg_clkctl = readl(SCSS_CLK_CTL_ADDR);
	reg_clkctl &= ~(0xFF << (8 * src_sel));
	reg_clkctl |= s->acpu_src_sel << (4 + 8 * src_sel);
	reg_clkctl |= s->acpu_src_div << (0 + 8 * src_sel);
	writel(reg_clkctl, SCSS_CLK_CTL_ADDR);

	/* Toggle clock source. */
	reg_clksel ^= 1;

	/* Program clock source selection. */
	writel(reg_clksel, SCSS_CLK_SEL_ADDR);
}

int acpuclk_set_rate(unsigned long rate, enum setrate_reason reason)
{
	struct clkctl_acpu_speed *tgt_s, *strt_s;
	int res, rc = 0;

	if (reason == SETRATE_CPUFREQ)
		mutex_lock(&drv_state.lock);

	strt_s = drv_state.current_speed;

	if (rate == strt_s->acpu_clk_khz)
		goto out;

	for (tgt_s = acpu_freq_tbl; tgt_s->acpu_clk_khz != 0; tgt_s++) {
		if (tgt_s->acpu_clk_khz == rate)
			break;
	}
	if (tgt_s->acpu_clk_khz == 0) {
		rc = -EINVAL;
		goto out;
	}

	if (reason == SETRATE_CPUFREQ) {
		/* Increase VDD if needed. */
		if (tgt_s->vdd_mv > strt_s->vdd_mv) {
			rc = acpuclk_set_acpu_vdd(tgt_s);
			if (rc < 0) {
				pr_err("ACPU VDD increase to %d mV failed "
					"(%d)\n", tgt_s->vdd_mv, rc);
				goto out;
			}
		}
	}

	dprintk("Switching from ACPU rate %u KHz -> %u KHz\n",
	       strt_s->acpu_clk_khz, tgt_s->acpu_clk_khz);

	/* Increase the AXI bus frequency if needed. This must be done before
	 * increasing the ACPU frequency, since voting for high AXI rates
	 * implicitly takes care of increasing the MSMC1 voltage, as needed. */
	if (tgt_s->axi_clk_khz > strt_s->axi_clk_khz) {
		rc = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK,
						tgt_s->axi_clk_khz * 1000);
		if (rc < 0) {
			pr_err("Setting AXI min rate failed (%d)\n", rc);
			goto out;
		}
	}

	/* Make sure target PLL is on. */
	if (strt_s->src != tgt_s->src && tgt_s->src >= 0) {
		dprintk("Enabling PLL %d\n", tgt_s->src);
		pll_enable(tgt_s->src);
	}

	/* Perform the frequency switch */
	acpuclk_set_src(tgt_s);
	drv_state.current_speed = tgt_s;
	loops_per_jiffy = tgt_s->lpj;

	/* Nothing else to do for SWFI. */
	if (reason == SETRATE_SWFI)
		goto out;

	/* Turn off previous PLL if not used. */
	if (strt_s->src != tgt_s->src && strt_s->src >= 0) {
		dprintk("Disabling PLL %d\n", strt_s->src);
		pll_disable(strt_s->src);
	}

	/* Decrease the AXI bus frequency if we can. */
	if (tgt_s->axi_clk_khz < strt_s->axi_clk_khz) {
		res = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK,
						tgt_s->axi_clk_khz * 1000);
		if (res < 0)
			pr_warning("Setting AXI min rate failed (%d)\n", res);
	}

	/* Nothing else to do for power collapse. */
	if (reason == SETRATE_PC)
		goto out;

	/* Drop VDD level if we can. */
	if (tgt_s->vdd_mv < strt_s->vdd_mv) {
		res = acpuclk_set_acpu_vdd(tgt_s);
		if (res < 0) {
			pr_warning("ACPU VDD decrease to %d mV failed (%d)\n",
					tgt_s->vdd_mv, res);
		}
	}

	dprintk("ACPU speed change complete\n");
out:
	if (reason == SETRATE_CPUFREQ)
		mutex_unlock(&drv_state.lock);

	return rc;
}

unsigned long acpuclk_get_rate(void)
{
	WARN_ONCE(drv_state.current_speed == NULL,
		  "acpuclk_get_rate: not initialized\n");
	if (drv_state.current_speed)
		return drv_state.current_speed->acpu_clk_khz;
	else
		return 0;
}

uint32_t acpuclk_get_switch_time(void)
{
	return drv_state.acpu_switch_time_us;
}

unsigned long clk_get_max_axi_khz(void)
{
	return MAX_AXI_KHZ;
}
EXPORT_SYMBOL(clk_get_max_axi_khz);


/*----------------------------------------------------------------------------
 * Clock driver initialization
 *---------------------------------------------------------------------------*/

static void __init acpuclk_init(void)
{
	struct clkctl_acpu_speed *s;
	uint32_t div, sel, src_num;
	uint32_t reg_clksel, reg_clkctl;
	int res;

	reg_clksel = readl(SCSS_CLK_SEL_ADDR);

	/* Determine the ACPU clock rate. */
	switch ((reg_clksel >> 1) & 0x3) {
	case 0:	/* Running off the output of the raw clock source mux. */
		reg_clkctl = readl(SCSS_CLK_CTL_ADDR);
		src_num = reg_clksel & 0x1;
		sel = (reg_clkctl >> (12 - (8 * src_num))) & 0x7;
		div = (reg_clkctl >> (8 -  (8 * src_num))) & 0xF;

		/* Check frequency table for matching sel/div pair. */
		for (s = acpu_freq_tbl; s->acpu_clk_khz != 0; s++) {
			if (s->acpu_src_sel == sel && s->acpu_src_div == div)
				break;
		}
		if (s->acpu_clk_khz == 0) {
			pr_err("Error - ACPU clock reports invalid speed\n");
			return;
		}
		break;
	case 2:	/* Running off of the SCPLL selected through the core mux. */
		/* Switch to run off of the SCPLL selected through the raw
		 * clock source mux. */
		for (s = acpu_freq_tbl; s->acpu_clk_khz != 0
			&& s->src != PLL_2 && s->acpu_src_div == 0; s++)
			;
		if (s->acpu_clk_khz != 0) {
			/* Program raw clock source mux. */
			acpuclk_set_src(s);

			/* Switch to raw clock source input of the core mux. */
			reg_clksel = readl(SCSS_CLK_SEL_ADDR);
			reg_clksel &= ~(0x3 << 1);
			writel(reg_clksel, SCSS_CLK_SEL_ADDR);
			break;
		}
		/* else fall through */
	default:
		pr_err("Error - ACPU clock reports invalid source\n");
		return;
	}

	/* Set initial ACPU VDD. */
	acpuclk_set_acpu_vdd(s);

	drv_state.current_speed = s;

	/* Initialize current PLL's reference count. */
	if (s->src >= 0)
		pll_enable(s->src);

	res = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK, s->axi_clk_khz * 1000);
	if (res < 0)
		pr_warning("Setting AXI min rate failed!\n");

	pr_info("ACPU running at %d KHz\n", s->acpu_clk_khz);

	return;
}

/* Initalize the lpj field in the acpu_freq_tbl. */
static void __init lpj_init(void)
{
	int i;
	const struct clkctl_acpu_speed *base_clk = drv_state.current_speed;

	for (i = 0; acpu_freq_tbl[i].acpu_clk_khz; i++) {
		acpu_freq_tbl[i].lpj = cpufreq_scale(loops_per_jiffy,
						base_clk->acpu_clk_khz,
						acpu_freq_tbl[i].acpu_clk_khz);
	}
}

/* Update frequency tables for a 1024MHz PLL2. */
void __init pll2_1024mhz_fixup(void)
{
	if (acpu_freq_tbl[ARRAY_SIZE(acpu_freq_tbl)-2].acpu_clk_khz != 806400
		  || freq_table[ARRAY_SIZE(freq_table)-2].frequency != 806400) {
		pr_err("Frequency table fixups for PLL2 rate failed.\n");
		BUG();
	}
	acpu_freq_tbl[ARRAY_SIZE(acpu_freq_tbl)-2].acpu_clk_khz = 1024000;
	freq_table[ARRAY_SIZE(freq_table)-2].frequency = 1024000;
}

#define RPM_BYPASS_MASK	(1 << 3)
#define PMIC_MODE_MASK	(1 << 4)
static void __init saw_init(void)
{
	uint32_t reg_spmctl;

	reg_spmctl = readl(SAW_SPM_CTL);
	reg_spmctl |= RPM_BYPASS_MASK;	/* No RPM interface on 7x30, so
					   bypass the RPM handshake. */
	reg_spmctl |= PMIC_MODE_MASK;	/* PMIC_MODE is MSM7XXX.  */
	writel(reg_spmctl, SAW_SPM_CTL);
}

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	pr_info("acpu_clock_init()\n");

	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = clkdata->acpu_switch_time_us;
	drv_state.vdd_switch_time_us = clkdata->vdd_switch_time_us;
	/* PLL2 runs at 1024MHz for QSD8x55. */
	if (cpu_is_qsd8x55())
		pll2_1024mhz_fixup();
	saw_init();
	acpuclk_init();
	lpj_init();
#ifdef CONFIG_CPU_FREQ_MSM
	cpufreq_frequency_table_get_attr(freq_table, smp_processor_id());
#endif
}

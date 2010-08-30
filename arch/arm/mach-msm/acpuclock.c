/* arch/arm/mach-msm/acpuclock.c
 *
 * MSM architecture clock driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, Code Aurora Forum. All rights reserved.
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
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
#include <linux/remote_spinlock.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>

#include "proc_comm.h"
#include "smd_private.h"
#include "clock.h"
#include "acpuclock.h"
#include "socinfo.h"

#define PERF_SWITCH_DEBUG 0
#define PERF_SWITCH_STEP_DEBUG 0

struct clock_state
{
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			max_speed_delta_khz;
	uint32_t			vdd_switch_time_us;
	unsigned long			power_collapse_khz;
	unsigned long			wait_for_irq_khz;
	unsigned int			max_axi_khz;
};

#define PLL_BASE	7

struct shared_pll_control {
	uint32_t	version;
	struct {
		/* Denotes if the PLL is ON. Technically, this can be read
		 * directly from the PLL registers, but this feild is here,
		 * so let's use it.
		 */
		uint32_t	on;
		/* One bit for each processor core. The application processor
		 * is allocated bit position 1. All other bits should be
		 * considered as votes from other processors.
		 */
		uint32_t	votes;
	} pll[PLL_BASE + ACPU_PLL_END];
};

static remote_spinlock_t pll_lock;
static struct shared_pll_control *pll_control;

static struct clock_state drv_state = { 0 };

static void __init acpuclk_init(void);

struct clkctl_acpu_speed {
	unsigned int	use_for_scaling;
	unsigned int	a11clk_khz;
	int		pll;
	unsigned int	a11clk_src_sel;
	unsigned int	a11clk_src_div;
	unsigned int	ahbclk_khz;
	unsigned int	ahbclk_div;
	int		vdd;
	unsigned int 	axiclk_khz;
	unsigned long	lpj; /* loops_per_jiffy */
/* Index in acpu_freq_tbl[] for steppings. */
	short		down;
	short		up;
};

/*
 * ACPU speed table. Complete table is shown but certain speeds are commented
 * out to optimized speed switching. Initalize loops_per_jiffy to 0.
 *
 * Table stepping up/down is calculated during boot to choose the largest
 * frequency jump that's less than max_speed_delta_khz and preferrably on the
 * same PLL. If no frequencies using the same PLL are within
 * max_speed_delta_khz, then the farthest frequency that is within
 * max_speed_delta_khz is chosen.
 *
 * The table is initialized with the most common clock plan. Table fix up is
 * done during boot up to match the actual clock plan supported by the
 * hardware.
 */
#if (1)
static struct clkctl_acpu_speed  acpu_freq_tbl[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 30720, 0, 0, 8 },
	{ 0, 61440, ACPU_PLL_0,  4, 3, 61440,  0, 0, 30720,  0, 0, 8 },
	{ 0, 81920, ACPU_PLL_0,  4, 2, 40960,  1, 0, 61440,  0, 0, 8 },
	{ 0, 96000, ACPU_PLL_1,  1, 7, 48000,  1, 0, 61440,  0, 0, 9 },
	{ 1, 122880, ACPU_PLL_0, 4, 1, 61440,  1, 3, 61440,  0, 0, 8 },
	{ 0, 128000, ACPU_PLL_1, 1, 5, 64000,  1, 3, 61440,  0, 0, 12 },
	{ 0, 176000, ACPU_PLL_2, 2, 5, 88000,  1, 3, 61440,  0, 0, 11 },
	{ 0, 192000, ACPU_PLL_1, 1, 3, 64000,  2, 3, 61440,  0, 0, 12 },
	{ 1, 245760, ACPU_PLL_0, 4, 0, 81920,  2, 4, 61440,  0, 0, 12 },
	{ 1, 256000, ACPU_PLL_1, 1, 2, 128000, 1, 5, 128000, 0, 0, 12 },
	{ 0, 264000, ACPU_PLL_2, 2, 3, 88000,  2, 5, 128000, 0, 6, 13 },
	{ 0, 352000, ACPU_PLL_2, 2, 2, 88000,  3, 5, 128000, 0, 6, 13 },
	{ 1, 384000, ACPU_PLL_1, 1, 1, 128000, 2, 6, 128000, 0, 5, -1 },
	{ 1, 528000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, 0, 11, -1 },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};
#else /* Table of freq we currently use. */
static struct clkctl_acpu_speed  acpu_freq_tbl[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 30720, 0, 0, 4 },
	{ 1, 122880, ACPU_PLL_0, 4, 1, 61440, 1, 3, 61440, 0, 0, 4 },
	{ 1, 128000, ACPU_PLL_1, 1, 5, 64000, 1, 3, 61440, 0, 0, 6 },
	{ 0, 176000, ACPU_PLL_2, 2, 5, 88000, 1, 3, 61440, 0, 0, 5 },
	{ 1, 245760, ACPU_PLL_0, 4, 0, 81920, 2, 4, 61440, 0, 0, 5 },
	{ 0, 352000, ACPU_PLL_2, 2, 2, 88000, 3, 5, 128000, 0, 3, 7 },
	{ 1, 384000, ACPU_PLL_1, 1, 1, 128000, 2, 6, 128000, 0, 2, -1 },
	{ 1, 528000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, 0, 5, -1 },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};
#endif

#ifdef CONFIG_CPU_FREQ_MSM
static struct cpufreq_frequency_table freq_table[20];

static void __init cpufreq_table_init(void)
{
	unsigned int i;
	unsigned int freq_cnt = 0;

	/* Construct the freq_table table from acpu_freq_tbl since the
	 * freq_table values need to match frequencies specified in
	 * acpu_freq_tbl and acpu_freq_tbl needs to be fixed up during init.
	 */
	for (i = 0; acpu_freq_tbl[i].a11clk_khz != 0
			&& freq_cnt < ARRAY_SIZE(freq_table)-1; i++) {
		if (acpu_freq_tbl[i].use_for_scaling) {
			freq_table[freq_cnt].index = freq_cnt;
			freq_table[freq_cnt].frequency
				= acpu_freq_tbl[i].a11clk_khz;
			freq_cnt++;
		}
	}

	/* freq_table not big enough to store all usable freqs. */
	BUG_ON(acpu_freq_tbl[i].a11clk_khz != 0);

	freq_table[freq_cnt].index = freq_cnt;
	freq_table[freq_cnt].frequency = CPUFREQ_TABLE_END;

	pr_info("%d scaling frequencies supported.\n", freq_cnt);
}
#endif

static int pc_pll_request(unsigned id, unsigned on)
{
	int res = 0;
	on = !!on;

#if PERF_SWITCH_DEBUG
	if (on)
		printk(KERN_DEBUG "Enabling PLL %d\n", id);
	else
		printk(KERN_DEBUG "Disabling PLL %d\n", id);
#endif

	if (id >= ACPU_PLL_END)
		return -EINVAL;

	if (pll_control) {
		remote_spin_lock(&pll_lock);
		if (on) {
			pll_control->pll[PLL_BASE + id].votes |= 2;
			if (!pll_control->pll[PLL_BASE + id].on) {
				writel(6, PLLn_MODE(id));
				udelay(50);
				writel(7, PLLn_MODE(id));
				pll_control->pll[PLL_BASE + id].on = 1;
			}
		} else {
			pll_control->pll[PLL_BASE + id].votes &= ~2;
			if (pll_control->pll[PLL_BASE + id].on
			    && !pll_control->pll[PLL_BASE + id].votes) {
				writel(0, PLLn_MODE(id));
				pll_control->pll[PLL_BASE + id].on = 0;
			}
		}
		remote_spin_unlock(&pll_lock);
	} else {
		res = msm_proc_comm(PCOM_CLKCTL_RPC_PLL_REQUEST, &id, &on);
		if (res < 0)
			return res;
		else if ((int) id < 0)
			return -EINVAL;
	}

#if PERF_SWITCH_DEBUG
	if (on)
		printk(KERN_DEBUG "PLL %d enabled\n", id);
	else
		printk(KERN_DEBUG "PLL %d disabled\n", id);
#endif
	return res;
}


/*----------------------------------------------------------------------------
 * ARM11 'owned' clock control
 *---------------------------------------------------------------------------*/

unsigned long acpuclk_power_collapse(void) {
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(drv_state.power_collapse_khz, SETRATE_PC);
	return ret * 1000;
}

unsigned long acpuclk_wait_for_irq(void) {
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(drv_state.wait_for_irq_khz, SETRATE_SWFI);
	return ret * 1000;
}

static int acpuclk_set_vdd_level(int vdd)
{
	uint32_t current_vdd;

	current_vdd = readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x07;

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "acpuclock: Switching VDD from %u -> %d\n",
	       current_vdd, vdd);
#endif
	writel((1 << 7) | (vdd << 3), A11S_VDD_SVS_PLEVEL_ADDR);
	udelay(drv_state.vdd_switch_time_us);
	if ((readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x7) != vdd) {
#if PERF_SWITCH_DEBUG
		printk(KERN_ERR "acpuclock: VDD set failed\n");
#endif
		return -EIO;
	}

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "acpuclock: VDD switched\n");
#endif
	return 0;
}

/* Set proper dividers for the given clock speed. */
static void acpuclk_set_div(const struct clkctl_acpu_speed *hunt_s) {
	uint32_t reg_clkctl, reg_clksel, clk_div, src_sel;

	reg_clksel = readl(A11S_CLK_SEL_ADDR);

	/* AHB_CLK_DIV */
	clk_div = (reg_clksel >> 1) & 0x03;
	/* CLK_SEL_SRC1NO */
	src_sel = reg_clksel & 1;

	/*
	 * If the new clock divider is higher than the previous, then
	 * program the divider before switching the clock
	 */
	if (hunt_s->ahbclk_div > clk_div) {
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}

	/* Program clock source and divider */
	reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl &= ~(0xFF << (8 * src_sel));
	reg_clkctl |= hunt_s->a11clk_src_sel << (4 + 8 * src_sel);
	reg_clkctl |= hunt_s->a11clk_src_div << (0 + 8 * src_sel);
	writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

	/* Program clock source selection */
	reg_clksel ^= 1;
	writel(reg_clksel, A11S_CLK_SEL_ADDR);

	/*
	 * If the new clock divider is lower than the previous, then
	 * program the divider after switching the clock
	 */
	if (hunt_s->ahbclk_div < clk_div) {
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}
}

int acpuclk_set_rate(unsigned long rate, enum setrate_reason reason)
{
	uint32_t reg_clkctl;
	struct clkctl_acpu_speed *cur_s, *tgt_s, *strt_s;
	int rc = 0;
	unsigned int plls_enabled = 0, pll;

	strt_s = cur_s = drv_state.current_speed;

	WARN_ONCE(cur_s == NULL, "acpuclk_set_rate: not initialized\n");
	if (cur_s == NULL)
		return -ENOENT;

	if (rate == (cur_s->a11clk_khz * 1000))
		return 0;

	for (tgt_s = acpu_freq_tbl; tgt_s->a11clk_khz != 0; tgt_s++) {
		if (tgt_s->a11clk_khz == (rate / 1000))
			break;
	}

	if (tgt_s->a11clk_khz == 0)
		return -EINVAL;

	/* Choose the highest speed at or below 'rate' with same PLL. */
	if (reason != SETRATE_CPUFREQ
	    && tgt_s->a11clk_khz < cur_s->a11clk_khz) {
		while (tgt_s->pll != ACPU_PLL_TCXO && tgt_s->pll != cur_s->pll)
			tgt_s--;
	}

	if (strt_s->pll != ACPU_PLL_TCXO)
		plls_enabled |= 1 << strt_s->pll;

	if (reason == SETRATE_CPUFREQ) {
		mutex_lock(&drv_state.lock);
		if (strt_s->pll != tgt_s->pll && tgt_s->pll != ACPU_PLL_TCXO) {
			rc = pc_pll_request(tgt_s->pll, 1);
			if (rc < 0) {
				pr_err("PLL%d enable failed (%d)\n",
					tgt_s->pll, rc);
				goto out;
			}
			plls_enabled |= 1 << tgt_s->pll;
		}
		/* Increase VDD if needed. */
		if (tgt_s->vdd > cur_s->vdd) {
			if ((rc = acpuclk_set_vdd_level(tgt_s->vdd)) < 0) {
				printk(KERN_ERR "Unable to switch ACPU vdd\n");
				goto out;
			}
		}
	}

	/* Set wait states for CPU inbetween frequency changes */
	reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl |= (100 << 16); /* set WT_ST_CNT */
	writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

#if PERF_SWITCH_DEBUG
	printk(KERN_INFO "acpuclock: Switching from ACPU rate %u -> %u\n",
	       strt_s->a11clk_khz * 1000, tgt_s->a11clk_khz * 1000);
#endif

	while (cur_s != tgt_s) {
		/*
		 * Always jump to target freq if within 256mhz, regulardless of
		 * PLL. If differnece is greater, use the predefinied
		 * steppings in the table.
		 */
		int d = abs((int)(cur_s->a11clk_khz - tgt_s->a11clk_khz));
		if (d > drv_state.max_speed_delta_khz) {
			/* Step up or down depending on target vs current. */
			int clk_index = tgt_s->a11clk_khz > cur_s->a11clk_khz ?
				cur_s->up : cur_s->down;
			if (clk_index < 0) { /* This should not happen. */
				printk(KERN_ERR "cur:%u target: %u\n",
					cur_s->a11clk_khz, tgt_s->a11clk_khz);
				rc = -EINVAL;
				goto out;
			}
			cur_s = &acpu_freq_tbl[clk_index];
		} else {
			cur_s = tgt_s;
		}
#if PERF_SWITCH_STEP_DEBUG
		printk(KERN_DEBUG "%s: STEP khz = %u, pll = %d\n",
			__FUNCTION__, cur_s->a11clk_khz, cur_s->pll);
#endif
		if (cur_s->pll != ACPU_PLL_TCXO
		    && !(plls_enabled & (1 << cur_s->pll))) {
			rc = pc_pll_request(cur_s->pll, 1);
			if (rc < 0) {
				pr_err("PLL%d enable failed (%d)\n",
					cur_s->pll, rc);
				goto out;
			}
			plls_enabled |= 1 << cur_s->pll;
		}

		acpuclk_set_div(cur_s);
		drv_state.current_speed = cur_s;
		/* Re-adjust lpj for the new clock speed. */
		loops_per_jiffy = cur_s->lpj;
		udelay(drv_state.acpu_switch_time_us);
	}

	/* Nothing else to do for SWFI. */
	if (reason == SETRATE_SWFI)
		return 0;

	/* Change the AXI bus frequency if we can. */
	if (strt_s->axiclk_khz != tgt_s->axiclk_khz) {
		rc = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK,
						tgt_s->axiclk_khz * 1000);
		if (rc < 0)
			pr_err("Setting AXI min rate failed!\n");
	}

	/* Nothing else to do for power collapse. */
	if (reason == SETRATE_PC)
		return 0;

	/* Disable PLLs we are not using anymore. */
	plls_enabled &= ~(1 << tgt_s->pll);
	for (pll = ACPU_PLL_0; pll <= ACPU_PLL_2; pll++)
		if (plls_enabled & (1 << pll)) {
			rc = pc_pll_request(pll, 0);
			if (rc < 0) {
				pr_err("PLL%d disable failed (%d)\n", pll, rc);
				goto out;
			}
		}

	/* Drop VDD level if we can. */
	if (tgt_s->vdd < strt_s->vdd) {
		if (acpuclk_set_vdd_level(tgt_s->vdd) < 0)
			printk(KERN_ERR "acpuclock: Unable to drop ACPU vdd\n");
	}

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "%s: ACPU speed change complete\n", __FUNCTION__);
#endif
out:
	if (reason == SETRATE_CPUFREQ)
		mutex_unlock(&drv_state.lock);
	return rc;
}

static void __init acpuclk_init(void)
{
	struct clkctl_acpu_speed *speed;
	uint32_t div, sel;
	int rc;

	/*
	 * Determine the rate of ACPU clock
	 */

	if (!(readl(A11S_CLK_SEL_ADDR) & 0x01)) { /* CLK_SEL_SRC1N0 */
		/* CLK_SRC0_SEL */
		sel = (readl(A11S_CLK_CNTL_ADDR) >> 12) & 0x7;
		/* CLK_SRC0_DIV */
		div = (readl(A11S_CLK_CNTL_ADDR) >> 8) & 0x0f;
	} else {
		/* CLK_SRC1_SEL */
		sel = (readl(A11S_CLK_CNTL_ADDR) >> 4) & 0x07;
		/* CLK_SRC1_DIV */
		div = readl(A11S_CLK_CNTL_ADDR) & 0x0f;
	}

	for (speed = acpu_freq_tbl; speed->a11clk_khz != 0; speed++) {
		if (speed->a11clk_src_sel == sel
		 && (speed->a11clk_src_div == div))
			break;
	}
	if (speed->a11clk_khz == 0) {
		printk(KERN_WARNING "Warning - ACPU clock reports invalid speed\n");
		return;
	}

	drv_state.current_speed = speed;

	rc = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK, speed->axiclk_khz * 1000);
	if (rc < 0)
		pr_err("Setting AXI min rate failed!\n");

	printk(KERN_INFO "ACPU running at %d KHz\n", speed->a11clk_khz);
}

unsigned long acpuclk_get_rate(void)
{
	WARN_ONCE(drv_state.current_speed == NULL,
		  "acpuclk_get_rate: not initialized\n");
	if (drv_state.current_speed)
		return drv_state.current_speed->a11clk_khz;
	else
		return 0;
}

uint32_t acpuclk_get_switch_time(void)
{
	return drv_state.acpu_switch_time_us;
}

/*----------------------------------------------------------------------------
 * Clock driver initialization
 *---------------------------------------------------------------------------*/

static int __init cmp_acpu_speed(const void *a, const void *b)
{
	return ((struct clkctl_acpu_speed *)a)->a11clk_khz
		- ((struct clkctl_acpu_speed *)b)->a11clk_khz;
}

#define DIV2REG(n)		((n)-1)
#define REG2DIV(n)		((n)+1)
#define SLOWER_BY(div, factor)	div = DIV2REG(REG2DIV(div) * factor)

static void __init pll0_a11_ahb_fixup(unsigned long pll0_l,
					struct clkctl_acpu_speed *t)
{
#define PLL0_196608_KHZ		10
#define PLL0_245760_KHZ		12
#define PLL0_491520_KHZ		25
#define PLL0_983000_KHZ		51

	switch (pll0_l) {
	case PLL0_983000_KHZ:
		SLOWER_BY(t->a11clk_src_div, 4);
		break;
	case PLL0_491520_KHZ:
		SLOWER_BY(t->a11clk_src_div, 2);
		break;
	case PLL0_245760_KHZ:
		/* Initialization correct except for 7x27. */
		if (cpu_is_msm7x27() && t->a11clk_khz == 245760) {
			t->ahbclk_div--;
			t->ahbclk_khz = 122880;
		}
		break;
	case PLL0_196608_KHZ:
		t->a11clk_khz = 196608 / REG2DIV(t->a11clk_src_div);
		switch (REG2DIV(t->a11clk_src_div)) {
		case 4: /* 49.152 MHz */
			t->ahbclk_khz = t->a11clk_khz;
			break;
		case 3: /* 65.536 MHz */
			t->ahbclk_khz = t->a11clk_khz;
			t->ahbclk_div = DIV2REG(1);
			break;
		case 2: /* 98.304 MHz */
			/* TODO: Increase AHB freq for sRoc
			 * and Roc 2.0 */
			t->ahbclk_khz = t->a11clk_khz >> 1;
			break;
		case 1: /* 196.608 MHz */
			/* TODO: Increase AHB freq for sRoc
			 * and Roc 2.0 */
			t->ahbclk_khz = t->a11clk_khz / 3;
			break;
		}
		break;
	default:
		printk(KERN_CRIT "Unknown PLL0 speed.\n");
		BUG();
		break;
	}
}

static void __init pll1_a11_ahb_fixup(unsigned long pll1_l,
					struct clkctl_acpu_speed *t)
{
#define PLL1_768000_KHZ		40
#define PLL1_960000_KHZ		50

	switch (pll1_l) {
	case PLL1_960000_KHZ:
		t->a11clk_khz = 960000 / REG2DIV(t->a11clk_src_div);
		/* XXX: Necessary for AHB freq of 128 MHz, but not
		 * necessary for lower AHB freqs. All ACPU freqs used
		 * for scaling have AHB of 128 MHz, so unconditional
		 * increment might be okay.
		 *
		 * 7x27 is an exception because running AHB at 160 MHz
		 * is meaningful (because it can run AXI at 160 MHz at
		 * lower CPU frequencies without impacting power
		 * consumption).
		 */
		if (!cpu_is_msm7x27())
			t->ahbclk_div++;
		t->ahbclk_khz = t->a11clk_khz / REG2DIV(t->ahbclk_div);
		break;
	case PLL1_768000_KHZ:
		/* Initialization correct. */
		break;
	default:
		printk(KERN_CRIT "Unknown PLL1 speed.\n");
		BUG();
		break;
	}
}

static void __init pll2_a11_ahb_fixup(unsigned long pll2_l,
					struct clkctl_acpu_speed *t)
{
#define PLL2_960000_KHZ		50
#define PLL2_1056000_KHZ	55
#define PLL2_1200000_KHZ	62

	switch (pll2_l) {
	case PLL2_1200000_KHZ:
		t->a11clk_khz = 1200000 / REG2DIV(t->a11clk_src_div);
		if (t->ahbclk_div > 2)
			t->ahbclk_div--;
		t->ahbclk_khz = t->a11clk_khz / REG2DIV(t->ahbclk_div);
		break;
	case PLL2_1056000_KHZ:
		/* Initialization correct. */
		break;
	case PLL2_960000_KHZ:
		t->a11clk_khz = 960000 / REG2DIV(t->a11clk_src_div);
		if (t->ahbclk_div > 1)
			t->ahbclk_div--;
		t->ahbclk_khz = t->a11clk_khz / REG2DIV(t->ahbclk_div);
		break;
	default:
		printk(KERN_CRIT "Unknown PLL2 speed.\n");
		BUG();
		break;
	}
}

static void __init acpu_freq_tbl_fixup(void)
{
	unsigned long pll0_l, pll1_l, pll2_l;
	int axi_160mhz = 0, axi_200mhz = 0;
	struct clkctl_acpu_speed *t;
	unsigned int n = 0;

	/* Wait for the PLLs to be initialized and then read their frequency.
	 */
	do {
		pll0_l = readl(PLLn_L_VAL(0)) & 0x3f;
		cpu_relax();
		udelay(50);
	} while (pll0_l == 0);
	do {
		pll1_l = readl(PLLn_L_VAL(1)) & 0x3f;
		cpu_relax();
		udelay(50);
	} while (pll1_l == 0);
	do {
		pll2_l = readl(PLLn_L_VAL(2)) & 0x3f;
		cpu_relax();
		udelay(50);
	} while (pll2_l == 0);

	printk(KERN_INFO "L val: PLL0: %d, PLL1: %d, PLL2: %d\n",
				(int)pll0_l, (int)pll1_l, (int)pll2_l);

	/* No fix up needed. */
	if (pll0_l == PLL0_245760_KHZ && pll1_l == PLL1_768000_KHZ
	    && pll2_l == PLL2_1056000_KHZ)
		goto print_info;

	/* The AXI bus frequency is also based off the PLL frequencies. For
	 * the AXI bus to be able to run at 160 MHz or 200 MHz, it needs a
	 * PLL that runs at an integer multiple of that frequency.
	 */
	axi_160mhz = (pll1_l == PLL1_960000_KHZ || pll2_l == PLL2_960000_KHZ);
	axi_200mhz = (pll2_l == PLL2_1200000_KHZ);

	/* Fix the dividers and the allowed clock rates based on the PLL
	 * frequencies.
	 *
	 * The CPU freq is based off the PLL frequencies. So if the actual
	 * PLL freq is different from the PLL freq assumed in the default
	 * table, then the dividers are adjusted to run the CPU at
	 * frequencies closer to the ones in the default table.
	 *
	 * The AHB freq is based off the CPU freq. Attempt is made to
	 * generally keep the AHB freq as close as possible to the AXI bus
	 * freq. But for each VDD level there is an upper limit for the AHB
	 * bus freq. Apart from the per VDD upper limit the AHB also has an
	 * absolute upper limit of 160 MHz. In most cases, these limitations
	 * would explain why the AHB divider is changed when a PLL freq
	 * changes.
	 */
	for (t = &acpu_freq_tbl[0]; t->a11clk_khz != 0; t++) {
		n++;

		switch (t->pll) {
		case ACPU_PLL_0:
			pll0_a11_ahb_fixup(pll0_l, t);
			break;
		case ACPU_PLL_1:
			pll1_a11_ahb_fixup(pll1_l, t);
			break;
		case ACPU_PLL_2:
			pll2_a11_ahb_fixup(pll2_l, t);
			break;
		}

		if (pll0_l == PLL0_196608_KHZ && t->a11clk_khz <= 196608)
			t->axiclk_khz = 24576;

		if (cpu_is_msm7x27() && t->a11clk_khz == 245760)
			t->axiclk_khz = 122880;

		if (axi_160mhz && drv_state.max_axi_khz >= 160000
		    && t->ahbclk_khz > 128000)
			t->axiclk_khz = 160000;
		if (axi_200mhz && drv_state.max_axi_khz >= 200000
		    && t->ahbclk_khz > 160000)
			t->axiclk_khz = 200000;

		/* 128 MHz is derived from PLL1 @ 768 MHz. If PLL1 @ 960 MHz
		 * then only 120 MHz can be derived from it. */
		if (pll1_l == PLL1_960000_KHZ && t->axiclk_khz == 128000)
			t->axiclk_khz = 120000;
	}

	/* The ACPU table is expected to be in ascending order by the rest
	 * of the code. This is the case after most fix ups. One example
	 * exception is when PLL1 runs at 960 MHz. In this case the entries
	 * corresponding to (256 and 264) need to be swapped.
	 */
	sort(acpu_freq_tbl, n, sizeof(struct clkctl_acpu_speed),
					cmp_acpu_speed, NULL);

print_info:
	/* The default 7x27 ACPU clock plan supports running the AXI bus at
	 * 200 MHz. So we don't classify it as Turbo mode.
	 */
	if (cpu_is_msm7x27())
		return;

	t = acpu_freq_tbl + n - 1;
	if (!axi_160mhz)
		pr_info("Turbo mode not supported.\n");
	else if (t->axiclk_khz == 160000)
		pr_info("Turbo mode supported and enabled.\n");
	else
		pr_info("Turbo mode supported but not enabled.\n");
}

/* Initalize the lpj field in the acpu_freq_tbl. */
static void __init lpj_init(void)
{
	int i;
	const struct clkctl_acpu_speed *base_clk = drv_state.current_speed;
	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {
		acpu_freq_tbl[i].lpj = cpufreq_scale(loops_per_jiffy,
						base_clk->a11clk_khz,
						acpu_freq_tbl[i].a11clk_khz);
	}
}

static void __init precompute_stepping(void)
{
	int i, step_idx, step_same_pll_idx;

#define cur_freq acpu_freq_tbl[i].a11clk_khz
#define step_freq acpu_freq_tbl[step_idx].a11clk_khz
#define cur_pll acpu_freq_tbl[i].pll
#define step_pll acpu_freq_tbl[step_idx].pll

	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {

		/* Calculate "Up" step. */
		step_idx = i + 1;
		step_same_pll_idx = -1;
		while (step_freq && (step_freq - cur_freq)
					<= drv_state.max_speed_delta_khz) {
			if (step_pll == cur_pll)
				step_same_pll_idx = step_idx;
			step_idx++;
		}

		/* Highest freq within max_speed_delta_khz. No step needed. */
		if (step_freq == 0)
			acpu_freq_tbl[i].up = -1;
		else if (step_idx == (i + 1)) {
			pr_crit("Delta between freqs %u KHz and %u KHz is"
				" too high!\n", cur_freq, step_freq);
			BUG();
		} else {
			/* There is only one TCXO freq. So don't complain. */
			if (cur_pll == ACPU_PLL_TCXO)
				step_same_pll_idx = step_idx - 1;
			if (step_same_pll_idx == -1) {
				pr_warning("Suboptimal up stepping for CPU "
					   "freq %u KHz.\n", cur_freq);
				acpu_freq_tbl[i].up = step_idx - 1;
			} else
				acpu_freq_tbl[i].up = step_same_pll_idx;
		}

		/* Calculate "Down" step. */
		step_idx = i - 1;
		step_same_pll_idx = -1;
		while (step_idx >= 0 && (cur_freq - step_freq)
					<= drv_state.max_speed_delta_khz) {
			if (step_pll == cur_pll)
				step_same_pll_idx = step_idx;
			step_idx--;
		}

		/* Lowest freq within max_speed_delta_khz. No step needed. */
		if (step_idx == -1)
			acpu_freq_tbl[i].down = -1;
		else if (step_idx == (i - 1)) {
			pr_crit("Delta between freqs %u KHz and %u KHz is"
				" too high!\n", cur_freq, step_freq);
			BUG();
		} else {
			if (step_same_pll_idx == -1) {
				pr_warning("Suboptimal down stepping for CPU "
					   "freq %u KHz.\n", cur_freq);
				acpu_freq_tbl[i].down = step_idx + 1;
			} else
				acpu_freq_tbl[i].down = step_same_pll_idx;
		}
	}
}

static void __init print_acpu_freq_tbl(void)
{
	struct clkctl_acpu_speed *t;
	pr_info("CPU-Freq  PLL  DIV  AHB-Freq  ADIV  AXI-Freq Dn Up\n");
	for (t = &acpu_freq_tbl[0]; t->a11clk_khz != 0; t++)
		pr_info("%8d  %3d  %3d  %8d  %4d  %8d %2d %2d\n",
			t->a11clk_khz, t->pll, t->a11clk_src_div + 1,
			t->ahbclk_khz, t->ahbclk_div + 1, t->axiclk_khz,
			t->down, t->up);
}

static void msm7x25_acpu_pll_hw_bug_fix(void)
{
	unsigned int n;

	/* The 7625 has a hardware bug and in order to select PLL2 we
	 * must program PLL3.  Use the same table, and just fix up the
	 * numbers on this target. */
	for (n = 0; acpu_freq_tbl[n].a11clk_khz != 0; n++)
		if (acpu_freq_tbl[n].pll == ACPU_PLL_2)
			acpu_freq_tbl[n].a11clk_src_sel = 3;
}

static void shared_pll_control_init(void)
{
#define PLL_REMOTE_SPINLOCK_ID	7
	unsigned smem_size;
	remote_spin_lock_init(&pll_lock, PLL_REMOTE_SPINLOCK_ID);
	pll_control = smem_get_entry(SMEM_CLKREGIM_SOURCES, &smem_size);

	if (!pll_control)
		pr_err("Unable to find shared PLL control data structure!\n");
	/* There might be more PLLs than what the application processor knows
	 * about. But the index used for each PLL is guaranteed to remain the
	 * same. */
	else if (smem_size < sizeof(struct shared_pll_control))
		pr_err("Shared PLL control data structure too small!\n");
	else if (pll_control->version != 0xCCEE0001)
		pr_err("Shared PLL control version mismatch!\n");
	else {
		pr_info("Shared PLL control available.\n");
		return;
	}

	pll_control = NULL;
	pr_err("Falling back to proc_comm PLL control.\n");
}

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	pr_info("acpu_clock_init()\n");

	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = clkdata->acpu_switch_time_us;
	drv_state.max_speed_delta_khz = clkdata->max_speed_delta_khz;
	drv_state.vdd_switch_time_us = clkdata->vdd_switch_time_us;
	drv_state.power_collapse_khz = clkdata->power_collapse_khz;
	drv_state.wait_for_irq_khz = clkdata->wait_for_irq_khz;
	drv_state.max_axi_khz = clkdata->max_axi_khz;
	acpu_freq_tbl_fixup();
	precompute_stepping();
	acpuclk_init();
	lpj_init();
	print_acpu_freq_tbl();
	if (cpu_is_msm7x25())
		msm7x25_acpu_pll_hw_bug_fix();
	if (cpu_is_msm7x27())
		shared_pll_control_init();
#ifdef CONFIG_CPU_FREQ_MSM
	cpufreq_table_init();
	cpufreq_frequency_table_get_attr(freq_table, smp_processor_id());
#endif
}

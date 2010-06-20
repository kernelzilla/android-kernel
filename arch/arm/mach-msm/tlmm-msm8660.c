/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <mach/tlmm.h>
#include <mach/msm_iomap.h>
#include "tlmm-msm8660.h"

enum msm_tlmm_register {
	SDC4_HDRV_PULL_CTL       = 0x20a0,
	SDC3_HDRV_PULL_CTL       = 0x20a4,
};

struct tlmm_field_cfg {
	enum msm_tlmm_register reg;
	u8                     off;
};

static struct tlmm_field_cfg tlmm_hdrv_cfgs[] = {
	{SDC4_HDRV_PULL_CTL, 6},       /* TLMM_HDRV_SDC4_CLK          */
	{SDC4_HDRV_PULL_CTL, 3},       /* TLMM_HDRV_SDC4_CMD          */
	{SDC4_HDRV_PULL_CTL, 0},       /* TLMM_HDRV_SDC4_DATA         */
	{SDC3_HDRV_PULL_CTL, 6},       /* TLMM_HDRV_SDC3_CLK          */
	{SDC3_HDRV_PULL_CTL, 3},       /* TLMM_HDRV_SDC3_CMD          */
	{SDC3_HDRV_PULL_CTL, 0},       /* TLMM_HDRV_SDC3_DATA         */
};

static struct tlmm_field_cfg tlmm_pull_cfgs[] = {
	{SDC4_HDRV_PULL_CTL, 11},       /* TLMM_PULL_SDC4_CMD       */
	{SDC4_HDRV_PULL_CTL, 9},        /* TLMM_PULL_SDC4_DATA      */
	{SDC3_HDRV_PULL_CTL, 11},       /* TLMM_PULL_SDC3_CMD       */
	{SDC3_HDRV_PULL_CTL, 9},        /* TLMM_PULL_SDC3_DATA      */
};

static DEFINE_SPINLOCK(tlmm_lock);

static void msm_tlmm_set_field(struct tlmm_field_cfg *configs,
			unsigned               id,
			unsigned               width,
			unsigned               val)
{
	unsigned long irqflags;
	u32 mask = (1 << width) - 1;
	u32 __iomem *reg = MSM_TLMM_BASE + configs[id].reg;
	u32 reg_val;

	spin_lock_irqsave(&tlmm_lock, irqflags);
	reg_val = readl(reg);
	reg_val &= ~(mask << configs[id].off);
	reg_val |= (val & mask) << configs[id].off;
	writel(reg_val, reg);
	spin_unlock_irqrestore(&tlmm_lock, irqflags);
}

void msm_tlmm_set_hdrive(enum msm_tlmm_hdrive_tgt tgt, int drv_str)
{
	msm_tlmm_set_field(tlmm_hdrv_cfgs, tgt, 3, drv_str);
}
EXPORT_SYMBOL(msm_tlmm_set_hdrive);

void msm_tlmm_set_pull(enum msm_tlmm_pull_tgt tgt, int pull)
{
	msm_tlmm_set_field(tlmm_pull_cfgs, tgt, 2, pull);
}
EXPORT_SYMBOL(msm_tlmm_set_pull);

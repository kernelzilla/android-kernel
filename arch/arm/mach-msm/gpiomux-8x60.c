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
#include <mach/msm_iomap.h>
#include "tlmm-msm8660.h"
#include "gpiomux.h"

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	#define SDCC1_DAT_0_3_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_1 | GPIOMUX_DRV_10MA)
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	#define SDCC1_DAT_4_7_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_1 | GPIOMUX_DRV_10MA)
#else
	#define SDCC1_DAT_4_7_CMD_ACTV_CFG 0
#endif
	#define SDCC1_CLK_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_NONE\
					| GPIOMUX_FUNC_1 | GPIOMUX_DRV_16MA)
#else
	#define SDCC1_DAT_0_3_CMD_ACTV_CFG 0
	#define SDCC1_DAT_4_7_CMD_ACTV_CFG 0
	#define SDCC1_CLK_ACTV_CFG 0
#endif

#define SDCC1_SUSPEND_CONFIG (GPIOMUX_VALID | GPIOMUX_PULL_UP)

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	#define SDCC2_DAT_0_3_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_10MA)
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	#define SDCC2_DAT_4_7_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_10MA)
#else
	#define SDCC2_DAT_4_7_CMD_ACTV_CFG 0
#endif
	#define SDCC2_CLK_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_NONE\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_16MA)
#else
	#define SDCC2_DAT_0_3_CMD_ACTV_CFG 0
	#define SDCC2_DAT_4_7_CMD_ACTV_CFG 0
	#define SDCC2_CLK_ACTV_CFG 0
#endif

#define SDCC2_SUSPEND_CONFIG (GPIOMUX_VALID | GPIOMUX_PULL_DOWN)

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	#define SDCC5_DAT_0_3_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_10MA)
#ifdef CONFIG_MMC_MSM_SDC5_8_BIT_SUPPORT
	#define SDCC5_DAT_4_7_CMD_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_UP\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_10MA)
#else
	#define SDCC5_DAT_4_7_CMD_ACTV_CFG 0
#endif
	#define SDCC5_CLK_ACTV_CFG (GPIOMUX_VALID | GPIOMUX_PULL_NONE\
					| GPIOMUX_FUNC_2 | GPIOMUX_DRV_16MA)
#else
	#define SDCC5_DAT_0_3_CMD_ACTV_CFG 0
	#define SDCC5_DAT_4_7_CMD_ACTV_CFG 0
	#define SDCC5_CLK_ACTV_CFG 0
#endif

#define SDCC5_SUSPEND_CONFIG (GPIOMUX_VALID | GPIOMUX_PULL_DOWN)

struct msm_gpiomux_config msm_gpiomux_configs[GPIOMUX_NGPIOS] = {
	/* SDCC1 data[0] */
	[159] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[1] */
	[160] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[2] */
	[161] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[3] */
	[162] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[4] */
	[163] = {
		.active = SDCC1_DAT_4_7_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[5] */
	[164] = {
		.active = SDCC1_DAT_4_7_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[6] */
	[165] = {
		.active = SDCC1_DAT_4_7_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 data[7] */
	[166] = {
		.active = SDCC1_DAT_4_7_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 CLK */
	[167] = {
		.active = SDCC1_CLK_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
	/* SDCC1 CMD */
	[168] = {
		.active = SDCC1_DAT_0_3_CMD_ACTV_CFG,
		.suspended = SDCC1_SUSPEND_CONFIG
	},
};

void __msm_gpiomux_write(unsigned gpio, gpiomux_config_t val)
{
	writel(val & ~GPIOMUX_CTL_MASK, GPIO_CONFIG(gpio));
}

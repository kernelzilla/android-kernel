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
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <linux/i2c.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/irqs.h>
#include <mach/msm_iomap.h>

#include "devices.h"
#include "timer.h"

void __iomem *gic_cpu_base_addr;

/*
 * The smc91x configuration varies depending on platform.
 * The resources data structure is filled in at runtime.
 */
static struct resource smc91x_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name          = "smc91x",
	.id            = 0,
	.num_resources = ARRAY_SIZE(smc91x_resources),
	.resource      = smc91x_resources,
};

#ifdef CONFIG_I2C_QUP
static void gsbi3_qup_i2c_gpio_config(int adap_id, int config_type)
{
}

static void gsbi4_qup_i2c_gpio_config(int adap_id, int config_type)
{
}

static void gsbi9_qup_i2c_gpio_config(int adap_id, int config_type)
{
}

static struct msm_i2c_platform_data msm_gsbi3_qup_i2c_pdata = {
	.clk_freq = 100000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi3_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi4_qup_i2c_pdata = {
	.clk_freq = 100000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi4_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi9_qup_i2c_pdata = {
	.clk_freq = 100000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi9_qup_i2c_gpio_config,
};
#endif

static struct platform_device *devices[] __initdata = {
	&smc91x_device,
#ifdef CONFIG_I2C_QUP
	&msm_gsbi3_qup_i2c_device,
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi9_qup_i2c_device,
#endif
};

unsigned long clk_get_max_axi_khz(void)
{
	return 0;
}

static void __init msm8x60_init_buses(void)
{
#ifdef CONFIG_I2C_QUP
	msm_gsbi3_qup_i2c_device.dev.platform_data = &msm_gsbi3_qup_i2c_pdata;
	msm_gsbi4_qup_i2c_device.dev.platform_data = &msm_gsbi4_qup_i2c_pdata;
	msm_gsbi9_qup_i2c_device.dev.platform_data = &msm_gsbi9_qup_i2c_pdata;
#endif
}

static void __init msm8x60_map_io(void)
{
	msm_map_msm8x60_io();
	msm_clock_init(msm_clocks_8x60, msm_num_clocks_8x60);
}

static void __init msm8x60_init_irq(void)
{
	unsigned int i;

	gic_dist_init(0, MSM_QGIC_DIST_BASE, GIC_PPI_START);
	gic_cpu_base_addr = (void *)MSM_QGIC_CPU_BASE;
	gic_cpu_init(0, MSM_QGIC_CPU_BASE);

	/* Edge trigger PPIs except AVS_SVICINT and AVS_SVICINTSWDONE */
	writel(0xFFFFD7FF, MSM_QGIC_DIST_BASE + GIC_DIST_CONFIG + 4);

	/* RUMI does not adhere to GIC spec by enabling STIs by default.
	 * Enable/clear is supposed to be RO for STIs, but is RW on RUMI.
	 */
	if (machine_is_msm8x60_surf() || machine_is_msm8x60_rumi3())
		writel(0x0000FFFF, MSM_QGIC_DIST_BASE + GIC_DIST_ENABLE_SET);

	/* FIXME: Not installing AVS_SVICINT and AVS_SVICINTSWDONE yet
	 * as they are configured as level, which does not play nice with
	 * handle_percpu_irq.
	 */
	for (i = GIC_PPI_START; i < GIC_SPI_START; i++) {
		if (i != AVS_SVICINT && i != AVS_SVICINTSWDONE)
			set_irq_handler(i, handle_percpu_irq);
	}
}

/*
 * Most segments of the EBI2 bus are disabled by default.
 */
static void __init msm8x60_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(0x1a100000, sizeof(uint32_t));
	if (ebi2_cfg_ptr != 0) {
		ebi2_cfg = readl(ebi2_cfg_ptr);

		if (machine_is_msm8x60_sim())
			ebi2_cfg |= (1 << 4); /* CS2_CFG */
		else if (machine_is_msm8x60_rumi3())
			ebi2_cfg |= (1 << 5); /* CS3_CFG */

		writel(ebi2_cfg, ebi2_cfg_ptr);
		iounmap(ebi2_cfg_ptr);
	}
}

static void __init msm8x60_configure_smc91x(void)
{
	if (machine_is_msm8x60_sim()) {

		smc91x_resources[0].start = 0x1b800300;
		smc91x_resources[0].end   = 0x1b8003ff;

		smc91x_resources[1].start = (NR_MSM_IRQS + 40);
		smc91x_resources[1].end   = (NR_MSM_IRQS + 40);

	} else if (machine_is_msm8x60_rumi3()) {

		smc91x_resources[0].start = 0x1d000300;
		smc91x_resources[0].end   = 0x1d0003ff;

		smc91x_resources[1].start = TLMM_SCSS_DIR_CONN_IRQ_0;
		smc91x_resources[1].end   = TLMM_SCSS_DIR_CONN_IRQ_0;
	}
}

static void __init msm8x60_init_tlmm(void)
{
	if (machine_is_msm8x60_rumi3())
		msm_gpio_install_direct_irq(NR_MSM_IRQS + 40, 0);
}

static void __init msm8x60_init(void)
{
	msm8x60_init_ebi2();
	msm8x60_init_tlmm();
	msm8x60_configure_smc91x();
	msm8x60_init_buses();
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

MACHINE_START(MSM8X60_RUMI3, "QCT MSM8X60 RUMI3")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_SIM, "QCT MSM8X60 SIMULATOR")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X60_SURF, "QCT MSM8X60 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.map_io = msm8x60_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = msm8x60_init,
	.timer = &msm_timer,
MACHINE_END

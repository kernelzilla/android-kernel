/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 *
 * OMAP Bus Glue
 *
 * Modified for OMAP by Tony Lindgren <tony@atomide.com>
 * Based on the 2.4 OMAP OHCI driver originally done by MontaVista Software Inc.
 * and on ohci-sa1111.c by Christopher Hoover <ch@hpl.hp.com>
 *
 * This file is licenced under the GPL.
 */

#include <linux/signal.h>	/* IRQF_DISABLED */
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include "ehci-omap.h"

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/mux.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/fpga.h>
#include <mach/usb.h>

/* OMAP-1510 OHCI has its own MMU for DMA */
#define OMAP1510_LB_MEMSIZE	32	/* Should be same as SDRAM size */
#define OMAP1510_LB_CLOCK_DIV	0xfffec10c
#define OMAP1510_LB_MMU_CTL	0xfffec208
#define OMAP1510_LB_MMU_LCK	0xfffec224
#define OMAP1510_LB_MMU_LD_TLB	0xfffec228
#define OMAP1510_LB_MMU_CAM_H	0xfffec22c
#define OMAP1510_LB_MMU_CAM_L	0xfffec230
#define OMAP1510_LB_MMU_RAM_H	0xfffec234
#define OMAP1510_LB_MMU_RAM_L	0xfffec238


#ifndef CONFIG_ARCH_OMAP
#error "This file is OMAP bus glue.  CONFIG_OMAP must be defined."
#endif

#ifdef CONFIG_TPS65010
#include <linux/i2c/tps65010.h>
#else

#define LOW	0
#define HIGH	1

#define GPIO1	1

static inline int tps65010_set_gpio_out_value(unsigned gpio, unsigned value)
{
	return 0;
}

#endif

extern int usb_disabled(void);
extern int ocpi_enable(void);

#ifndef CONFIG_ARCH_OMAP34XX
static struct clk *usb_host_ck;
static struct clk *usb_dc_ck;
#endif

static int host_enabled;
static int host_initialized;
static int ohci_irq_num;

static void omap_ohci_clock_power(int on)
{
#ifndef CONFIG_ARCH_OMAP34XX
	if (on) {
		clk_enable(usb_dc_ck);
		clk_enable(usb_host_ck);
		/* guesstimate for T5 == 1x 32K clock + APLL lock time */
		udelay(100);
	} else {
		clk_disable(usb_host_ck);
		clk_disable(usb_dc_ck);
	}
#endif
}

/*
 * Board specific gang-switched transceiver power on/off.
 * NOTE:  OSK supplies power from DC, not battery.
 */
static int omap_ohci_transceiver_power(int on)
{
	if (on) {
		if (machine_is_omap_innovator() && cpu_is_omap1510())
			fpga_write(fpga_read(INNOVATOR_FPGA_CAM_USB_CONTROL)
				| ((1 << 5/*usb1*/) | (1 << 3/*usb2*/)),
			       INNOVATOR_FPGA_CAM_USB_CONTROL);
		else if (machine_is_omap_osk())
			tps65010_set_gpio_out_value(GPIO1, LOW);
	} else {
		if (machine_is_omap_innovator() && cpu_is_omap1510())
			fpga_write(fpga_read(INNOVATOR_FPGA_CAM_USB_CONTROL)
				& ~((1 << 5/*usb1*/) | (1 << 3/*usb2*/)),
			       INNOVATOR_FPGA_CAM_USB_CONTROL);
		else if (machine_is_omap_osk())
			tps65010_set_gpio_out_value(GPIO1, HIGH);
	}

	return 0;
}

#ifdef CONFIG_ARCH_OMAP15XX
/*
 * OMAP-1510 specific Local Bus clock on/off
 */
static int omap_1510_local_bus_power(int on)
{
	if (on) {
		omap_writel((1 << 1) | (1 << 0), OMAP1510_LB_MMU_CTL);
		udelay(200);
	} else {
		omap_writel(0, OMAP1510_LB_MMU_CTL);
	}

	return 0;
}

/*
 * OMAP-1510 specific Local Bus initialization
 * NOTE: This assumes 32MB memory size in OMAP1510LB_MEMSIZE.
 *       See also arch/mach-omap/memory.h for __virt_to_dma() and
 *       __dma_to_virt() which need to match with the physical
 *       Local Bus address below.
 */
static int omap_1510_local_bus_init(void)
{
	unsigned int tlb;
	unsigned long lbaddr, physaddr;

	omap_writel((omap_readl(OMAP1510_LB_CLOCK_DIV) & 0xfffffff8) | 0x4,
	       OMAP1510_LB_CLOCK_DIV);

	/* Configure the Local Bus MMU table */
	for (tlb = 0; tlb < OMAP1510_LB_MEMSIZE; tlb++) {
		lbaddr = tlb * 0x00100000 + OMAP1510_LB_OFFSET;
		physaddr = tlb * 0x00100000 + PHYS_OFFSET;
		omap_writel((lbaddr & 0x0fffffff) >> 22, OMAP1510_LB_MMU_CAM_H);
		omap_writel(((lbaddr & 0x003ffc00) >> 6) | 0xc,
		       OMAP1510_LB_MMU_CAM_L);
		omap_writel(physaddr >> 16, OMAP1510_LB_MMU_RAM_H);
		omap_writel((physaddr & 0x0000fc00) | 0x300, OMAP1510_LB_MMU_RAM_L);
		omap_writel(tlb << 4, OMAP1510_LB_MMU_LCK);
		omap_writel(0x1, OMAP1510_LB_MMU_LD_TLB);
	}

	/* Enable the walking table */
	omap_writel(omap_readl(OMAP1510_LB_MMU_CTL) | (1 << 3), OMAP1510_LB_MMU_CTL);
	udelay(200);

	return 0;
}
#else
#define omap_1510_local_bus_power(x)	{}
#define omap_1510_local_bus_init()	{}
#endif

#ifdef	CONFIG_USB_OTG

static void start_hnp(struct ohci_hcd *ohci)
{
	const unsigned	port = ohci_to_hcd(ohci)->self.otg_port - 1;
	unsigned long	flags;
	u32 l;

	otg_start_hnp(ohci->transceiver);

	local_irq_save(flags);
	ohci->transceiver->state = OTG_STATE_A_SUSPEND;
	writel (RH_PS_PSS, &ohci->regs->roothub.portstatus [port]);
	l = omap_readl(OTG_CTRL);
	l &= ~OTG_A_BUSREQ;
	omap_writel(l, OTG_CTRL);
	local_irq_restore(flags);
}

#endif

/*-------------------------------------------------------------------------*/

static int omap_usb_port_ulpi_bypass(enum omap_usb_port_mode mode)
{
	return mode != OMAP_USB_PORT_MODE_ULPI_PHY;
}

static int omap_usb_port_ttl_chanel_config(enum omap_usb_port_mode mode)
{
	return (mode == OMAP_USB_PORT_MODE_UTMI_PHY_6PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_PHY_6PIN_ALT) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_PHY_3PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_PHY_4PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_6PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_6PIN_ALT) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_3PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_4PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_2PIN) ||
		(mode == OMAP_USB_PORT_MODE_UTMI_TLL_2PIN_ALT);
}

static unsigned omap_usb_port_fslsmode(enum omap_usb_port_mode mode)
{
	switch (mode) {
	/* non serial modes */
	case OMAP_USB_PORT_MODE_ULPI_PHY:
	case OMAP_USB_PORT_MODE_ULPI_TLL_SDR:
	case OMAP_USB_PORT_MODE_ULPI_TLL_DDR:
		return 0x0;

	/* serial modes */
	case OMAP_USB_PORT_MODE_UTMI_PHY_6PIN:
		return 0x0;

	case OMAP_USB_PORT_MODE_UTMI_PHY_6PIN_ALT:
		return 0x1;

	case OMAP_USB_PORT_MODE_UTMI_PHY_3PIN:
		return 0x2;

	case OMAP_USB_PORT_MODE_UTMI_PHY_4PIN:
		return 0x3;

	case OMAP_USB_PORT_MODE_UTMI_TLL_6PIN:
		return 0x4;

	case OMAP_USB_PORT_MODE_UTMI_TLL_6PIN_ALT:
		return 0x5;

	case OMAP_USB_PORT_MODE_UTMI_TLL_3PIN:
		return 0x6;

	case OMAP_USB_PORT_MODE_UTMI_TLL_4PIN:
		return 0x7;

	case OMAP_USB_PORT_MODE_UTMI_TLL_2PIN:
		return 0xa;

	case OMAP_USB_PORT_MODE_UTMI_TLL_2PIN_ALT:
		return 0xB;
	}

	return 0x0;
}

static void omap_usb_setup_ports(struct usb_hcd *hcd,
			struct omap_usb_platform_data *data)
{
	int i;
	u32 val;

	val = (1<<OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN_SHIFT)|
		(1<<OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN_SHIFT)|
		(1<<OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN_SHIFT)|
		(0<<OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN_SHIFT);


	if ((data->num_ports > 0) &&
	    (omap_usb_port_ulpi_bypass(data->port_data[0].mode)))
		val |= (1 << OMAP_UHH_HOSTCONFIG_P1_ULPI_BYPASS_SHIFT);
	if ((data->num_ports > 1) &&
	    (omap_usb_port_ulpi_bypass(data->port_data[1].mode)))
		val |= (1 << OMAP_UHH_HOSTCONFIG_P2_ULPI_BYPASS_SHIFT);
	if ((data->num_ports > 2) &&
	    (omap_usb_port_ulpi_bypass(data->port_data[2].mode)))
		val |= (1 << OMAP_UHH_HOSTCONFIG_P3_ULPI_BYPASS_SHIFT);

	omap_writel(val, OMAP_UHH_HOSTCONFIG);

	/* wait for change to sync */
	while ((omap_readl(OMAP_UHH_HOSTCONFIG) &
		OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_MASK) !=
	       (val & OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_MASK))
		cpu_relax();

	dev_dbg(hcd->self.controller, "ULPI Byppass mode congfigured: %08x\n",
		omap_readl(OMAP_UHH_HOSTCONFIG));

	/* Program the 3 channels upfront */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		/* AutoIdle */
		if ((data->num_ports > i) &&
		    (data->port_data[i].flags & OMAP_USB_PORT_FLAG_AUTOIDLE))
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
				    1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT,
				    OMAP_TLL_CHANNEL_CONF(i));
		else
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
				~(1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT),
				    OMAP_TLL_CHANNEL_CONF(i));

		/* BitStuffing */
		if ((data->num_ports > i) &&
		    (data->port_data[i].flags & OMAP_USB_PORT_FLAG_NOBITSTUFF))
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
				1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT,
				    OMAP_TLL_CHANNEL_CONF(i));
		else
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT),
				    OMAP_TLL_CHANNEL_CONF(i));

		/* SDR/DDR */
		if ((data->num_ports > i) &&
		(data->port_data[i].mode == OMAP_USB_PORT_MODE_ULPI_TLL_DDR))
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
				(1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
				    OMAP_TLL_CHANNEL_CONF(i));
		else
			omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
				~(1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
				    OMAP_TLL_CHANNEL_CONF(i));
	}

	/* Program Common TLL register */
	omap_writel((1 << OMAP_TLL_SHARED_CONF_FCLK_IS_ON_SHIFT) |
			(1 << OMAP_TLL_SHARED_CONF_USB_DIVRATION_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN_SHFT),
				OMAP_TLL_SHARED_CONF);

	/* Enable channels */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		u32 channel_conf;

		if ((data->num_ports <= i) ||
		   !(data->port_data[i].flags & OMAP_USB_PORT_FLAG_ENABLED)) {
			dev_dbg(hcd->self.controller,
				"port %d disabled. skipping.\n", i);
			continue;
		}

		channel_conf = omap_readl(OMAP_TLL_CHANNEL_CONF(i));
		channel_conf |= 1<<OMAP_TLL_CHANNEL_CONF_CHANEN_SHIFT;

		if (omap_usb_port_ttl_chanel_config(data->port_data[i].mode))
			channel_conf |= 1<<OMAP_TLL_CHANNEL_CONF_CHANMODE_SHIFT;
		else
			channel_conf &=
				~(3<<OMAP_TLL_CHANNEL_CONF_CHANMODE_SHIFT);

		channel_conf &= ~(OMAP_TLL_CHANNEL_CONF_FSLSMODE(0xf));
		channel_conf |= OMAP_TLL_CHANNEL_CONF_FSLSMODE(
			omap_usb_port_fslsmode(data->port_data[i].mode));


		omap_writel(channel_conf, OMAP_TLL_CHANNEL_CONF(i));
		dev_dbg(hcd->self.controller,
			"port %d enabled: OMAP_TTL_CHANNEL_CONF_%d=%08x\n",
			i, i+1, omap_readl(OMAP_TLL_CHANNEL_CONF(i)));

		omap_writeb(0xBE, OMAP_TLL_ULPI_SCRATCH_REGISTER(i));
		dev_dbg(hcd->self.controller, "ULPI_SCRATCH_REG[ch=%d]"
			"= 0x%02x\n",
			i+1, omap_readb(OMAP_TLL_ULPI_SCRATCH_REGISTER(i)));
	}
}

static int ohci_omap_init(struct usb_hcd *hcd)
{
	struct ohci_hcd		*ohci = hcd_to_ohci(hcd);
	struct omap_usb_config	*config = hcd->self.controller->platform_data;
	int			need_transceiver = (config->otg != 0);
	int			ret;

	dev_dbg(hcd->self.controller, "starting USB Controller\n");

	if (config->otg) {
		ohci_to_hcd(ohci)->self.otg_port = config->otg;
		/* default/minimum OTG power budget:  8 mA */
		ohci_to_hcd(ohci)->power_budget = 8;
	}

	/* boards can use OTG transceivers in non-OTG modes */
	need_transceiver = need_transceiver
			|| machine_is_omap_h2() || machine_is_omap_h3();

	if (cpu_is_omap16xx())
		ocpi_enable();

#ifdef	CONFIG_USB_OTG
	if (need_transceiver) {
		ohci->transceiver = otg_get_transceiver();
		if (ohci->transceiver) {
			int	status = otg_set_host(ohci->transceiver,
						&ohci_to_hcd(ohci)->self);
			dev_dbg(hcd->self.controller, "init %s transceiver, status %d\n",
					ohci->transceiver->label, status);
			if (status) {
				if (ohci->transceiver)
					put_device(ohci->transceiver->dev);
				return status;
			}
		} else {
			dev_err(hcd->self.controller, "can't find transceiver\n");
			return -ENODEV;
		}
		ohci->start_hnp = start_hnp;
	}
#endif

	omap_ohci_clock_power(1);

	if (cpu_is_omap15xx()) {
		omap_1510_local_bus_power(1);
		omap_1510_local_bus_init();
	}

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	/* board-specific power switching and overcurrent support */
	if (machine_is_omap_osk() || machine_is_omap_innovator()) {
		u32	rh = roothub_a (ohci);

		/* power switching (ganged by default) */
		rh &= ~RH_A_NPS;

		/* TPS2045 switch for internal transceiver (port 1) */
		if (machine_is_omap_osk()) {
			ohci_to_hcd(ohci)->power_budget = 250;

			rh &= ~RH_A_NOCP;

#ifndef CONFIG_ARCH_OMAP34XX
			/* gpio9 for overcurrent detction */
			omap_cfg_reg(W8_1610_GPIO9);
			gpio_request(9, "OHCI overcurrent");
			gpio_direction_input(9);

			/* for paranoia's sake:  disable USB.PUEN */
			omap_cfg_reg(W4_USB_HIGHZ);
#endif
		}
		ohci_writel(ohci, rh, &ohci->regs->roothub.a);
		ohci->flags &= ~OHCI_QUIRK_HUB_POWER;
	} else if (machine_is_nokia770()) {
		/* We require a self-powered hub, which should have
		 * plenty of power. */
		ohci_to_hcd(ohci)->power_budget = 0;
	}

	/* FIXME khubd hub requests should manage power switching */
	omap_ohci_transceiver_power(1);

	/* board init will have already handled HMC and mux setup.
	 * any external transceiver should already be initialized
	 * too, so all configured ports use the right signaling now.
	 */

	return 0;
}

/* omap_start_ohci
 *  - Start the TI USBHOST controller
 *  - Under the assumpation that only OHCI is used
 */
static int omap_start_ohci(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct omap_usb_config *data = dev->dev.platform_data;
	int i, r, reset_delay;

	dev_dbg(hcd->self.controller,
		 "Configure TI OHCI USB Controller Register\n");

	/* Start DPLL5 Programming:
	 * Clock Framework is not doing this now:
	 * This will be done in clock framework later
	 */
	/* Enable DPLL 5 : Based on Input of 13Mhz*/
	cm_write_mod_reg((12 << OMAP3430ES2_PERIPH2_DPLL_DIV_SHIFT)|
			(120 << OMAP3430ES2_PERIPH2_DPLL_MULT_SHIFT),
			PLL_MOD, OMAP3430ES2_CM_CLKSEL4);

	cm_write_mod_reg(1 << OMAP3430ES2_DIV_120M_SHIFT,
			PLL_MOD, OMAP3430ES2_CM_CLKSEL5);

	cm_write_mod_reg((7 << OMAP3430ES2_PERIPH2_DPLL_FREQSEL_SHIFT) |
			(7 << OMAP3430ES2_EN_PERIPH2_DPLL_SHIFT),
			PLL_MOD, OMAP3430ES2_CM_CLKEN2);

	while (!(cm_read_mod_reg(PLL_MOD, CM_IDLEST2) &
		OMAP3430ES2_ST_PERIPH2_CLK_MASK))
		dev_dbg(hcd->self.controller,
			"idlest2 = 0x%x\n",
			cm_read_mod_reg(PLL_MOD, CM_IDLEST2));
	/* End DPLL5 programming */


	/* PRCM settings for USBHOST:
	 * Interface clk un-related to domain transition
	 */
	cm_write_mod_reg(0 << OMAP3430ES2_AUTO_USBHOST_SHIFT,
		OMAP3430ES2_USBHOST_MOD, CM_AUTOIDLE);

	/* Disable sleep dependency with MPU and IVA */
	cm_write_mod_reg((0 << OMAP3430ES2_EN_MPU_SHIFT) |
			(0 << OMAP3430ES2_EN_IVA2_SHIFT),
			OMAP3430ES2_USBHOST_MOD, OMAP3430_CM_SLEEPDEP);

	/* Disable Automatic transition of clock */
	cm_write_mod_reg(0 << OMAP3430ES2_CLKTRCTRL_USBHOST_SHIFT,
			OMAP3430ES2_USBHOST_MOD, CM_CLKSTCTRL);

	clk_enable(clk_get(NULL, "usbhost_ick"));
	clk_enable(clk_get(NULL, "usbhost_120m_fck"));
	clk_enable(clk_get(NULL, "usbhost_48m_fck"));

	reset_delay = 0;
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		if ((data->plat_data->num_ports > i) &&
			 (data->plat_data->port_data[i].startup)) {
			reset_delay =
			reset_delay > data->plat_data->port_data[i].reset_delay ?
			reset_delay : data->plat_data->port_data[i].reset_delay;
			r = data->plat_data->port_data[i].startup(dev, i);
			if (r < 0)
				return r;
		}
	}
	if (reset_delay)
		udelay(reset_delay);

	clk_enable(clk_get(NULL, "usbtll_ick"));
	clk_enable(clk_get(NULL, "usbtll_fck"));

	/* Enable Auto Idle of USBTLL */
	cm_write_mod_reg((1 << OMAP3430ES2_AUTO_USBTLL_SHIFT),
				CORE_MOD, CM_AUTOIDLE3);

	/* Wait for TLL to be Active */
	while ((cm_read_mod_reg(CORE_MOD, OMAP2430_CM_IDLEST3)
		& (1 << OMAP3430ES2_ST_USBTLL_SHIFT)))
		cpu_relax();

	/* perform TLL soft reset, and wait until reset is complete */
	omap_writel(1 << OMAP_USBTLL_SYSCONFIG_SOFTRESET_SHIFT,
		OMAP_USBTLL_SYSCONFIG);
	/* Wait for TLL reset to complete */
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS)
		& (1 << OMAP_USBTLL_SYSSTATUS_RESETDONE_SHIFT)))
		cpu_relax();

	dev_dbg(hcd->self.controller, "TLL RESET DONE\n");

	/* Smart Idle mode */
	omap_writel((1 << OMAP_USBTLL_SYSCONFIG_ENAWAKEUP_SHIFT) |
		(2 << OMAP_USBTLL_SYSCONFIG_SIDLEMODE_SHIFT) |
		(0 << OMAP_USBTLL_SYSCONFIG_CACTIVITY_SHIFT) |
		(1 << OMAP_USBTLL_SYSCONFIG_AUTOIDLE_SHIFT),
		OMAP_USBTLL_SYSCONFIG);

	/* Put UHH in SmartIdle/SmartStandby mode */
	omap_writel((1 << OMAP_UHH_SYSCONFIG_AUTOIDLE_SHIFT) |
		(1 << OMAP_UHH_SYSCONFIG_ENAWAKEUP_SHIFT) |
		(2 << OMAP_UHH_SYSCONFIG_SIDLEMODE_SHIFT) |
		(0 << OMAP_UHH_SYSCONFIG_CACTIVITY_SHIFT) |
		(2 << OMAP_UHH_SYSCONFIG_MIDLEMODE_SHIFT),
		OMAP_UHH_SYSCONFIG);

	omap_usb_setup_ports(hcd, data->plat_data);

	reset_delay = 0;
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		if (data->plat_data->num_ports > i)
			reset_delay =
			reset_delay > data->plat_data->port_data[i].reset_delay ?
			reset_delay : data->plat_data->port_data[i].reset_delay;
	}

	if (reset_delay)
		udelay(reset_delay);

	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		if ((data->plat_data->num_ports > i)
			&& (data->plat_data->port_data[i].reset))
			data->plat_data->port_data[i].reset(dev, i, 1);
	}

	if (data->plat_data->flags & OMAP_USB_FLAG_VBUS_INTERNAL_CHARGEPUMP) {
		/* Refer ISSUE2: LINK assumes external charge pump */
		/* use Port1 VBUS to charge externally Port2: */
		/* So for PHY mode operation use Port2 only */
		omap_writel((0xA << EHCI_INSNREG05_ULPI_REGADD_SHIFT) |
					/* OTG ctrl reg*/
			(2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) |/*   Write */
			(1 << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT) |/* Port1 */
			(1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) |/* Start */
			(0x26),
			EHCI_INSNREG05_ULPI);
		while (!(omap_readl(EHCI_INSNREG05_ULPI)
			& (1<<EHCI_INSNREG05_ULPI_CONTROL_SHIFT)))
			cpu_relax();
	}
	return 0;
}

static void ohci_omap_stop(struct usb_hcd *hcd)
{
	dev_dbg(hcd->self.controller, "stopping USB Controller\n");
	ohci_stop(hcd);
	omap_ohci_clock_power(0);
}


/*-------------------------------------------------------------------------*/

/**
 * usb_hcd_omap_probe - initialize OMAP-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
static int usb_hcd_omap_probe (const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval, irq;
	struct usb_hcd *hcd = 0;
	struct ohci_hcd *ohci;

	if (pdev->num_resources != 2) {
		printk(KERN_ERR "hcd probe: invalid num_resources: %i\n",
		       pdev->num_resources);
		return -ENODEV;
	}

	if (pdev->resource[0].flags != IORESOURCE_MEM
			|| pdev->resource[1].flags != IORESOURCE_IRQ) {
		printk(KERN_ERR "hcd probe: invalid resource type\n");
		return -ENODEV;
	}

	printk(KERN_INFO "%s Enter. \n", __func__);

#ifndef CONFIG_ARCH_OMAP34XX
	usb_host_ck = clk_get(0, "usb_hhc_ck");
	if (IS_ERR(usb_host_ck))
		return PTR_ERR(usb_host_ck);

	if (!cpu_is_omap15xx())
		usb_dc_ck = clk_get(&pdev->dev, "usb_dc_ck");
	else
		usb_dc_ck = clk_get(&pdev->dev, "lb_ck");

	if (IS_ERR(usb_dc_ck)) {
		clk_put(usb_host_ck);
		return PTR_ERR(usb_dc_ck);
	}
#endif


	hcd = usb_create_hcd (driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err0;
	}
#if defined(CONFIG_ARCH_OMAP34XX)
	omap_start_ohci(pdev, hcd);
#endif
	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_dbg(&pdev->dev, "request_mem_region failed\n");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "can't ioremap OHCI HCD\n");
		retval = -ENOMEM;
		goto err2;
	}

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	host_initialized = 0;
	host_enabled = 1;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		retval = -ENXIO;
		goto err3;
	}
	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED);
	if (retval)
		goto err3;
	else
		ohci_irq_num = irq;


	host_initialized = 1;

	if (!host_enabled)
		omap_ohci_clock_power(0);

	return 0;
err3:
	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
err0:

#ifndef CONFIG_ARCH_OMAP34XX
	clk_put(usb_dc_ck);
	clk_put(usb_host_ck);
#endif
	return retval;
}


/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_omap_remove - shutdown processing for OMAP-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 */
static inline void
usb_hcd_omap_remove (struct usb_hcd *hcd, struct platform_device *pdev)
{
	struct ohci_hcd		*ohci = hcd_to_ohci (hcd);

	usb_remove_hcd(hcd);
	if (ohci->transceiver) {
		(void) otg_set_host(ohci->transceiver, 0);
		put_device(ohci->transceiver->dev);
	}
#ifndef CONFIG_ARCH_OMAP34XX
	if (machine_is_omap_osk())
		gpio_free(9);
#endif
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

#ifndef CONFIG_ARCH_OMAP34XX
	clk_put(usb_dc_ck);
	clk_put(usb_host_ck);
#endif
}

/*-------------------------------------------------------------------------*/

static int
ohci_omap_start (struct usb_hcd *hcd)
{
	struct omap_usb_config *config;
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int		ret;

	if (!host_enabled)
		return 0;
	config = hcd->self.controller->platform_data;
	if (config->otg || config->rwc) {
		ohci->hc_control = OHCI_CTRL_RWC;
		writel(OHCI_CTRL_RWC, &ohci->regs->control);
	}

	if ((ret = ohci_run (ohci)) < 0) {
		dev_err(hcd->self.controller, "can't start\n");
		ohci_stop (hcd);
		return ret;
	}
	return 0;
}

#ifdef CONFIG_PM
static int omap_ohci_bus_suspend(struct usb_hcd *hcd)
{
	int res = 0;
	int ret = 0;
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
	int chix;
	struct usb_device *childdev;
	int count = 0;
#if defined(CONFIG_ARCH_OMAP34XX)
	struct omap_usb_config *config = hcd->self.controller->platform_data;
#endif
	printk(KERN_INFO "%s Enter. \n", __func__);

	for (chix = 0; chix < hcd->self.root_hub->maxchild; chix++) {
		childdev = hcd->self.root_hub->children[chix];
		if (childdev)
			count++;
	}

	if (!count) {
		printk(KERN_ERR " %s no device, suspend failed. \n", __func__);
		return -EBUSY ;
	}

	ret = ohci_bus_suspend(hcd);
	if (ret)
		return ret;
	mdelay(8); /* MSTANDBY assertion is delayed by ~8ms */

#if defined(CONFIG_ARCH_OMAP34XX)
	if (config->usbhost_standby_status)
		res = config->usbhost_standby_status();
#endif
	if (res == 0) {
		printk(KERN_ERR "ohci: suspend failed!\n");
		ohci_bus_resume(hcd);
		return -EBUSY ;
	}

	if (ohci_irq_num)
		disable_irq(ohci_irq_num);

	/* go ahead turn off clock */
	clk_disable(clk_get(NULL, "usbtll_fck"));
	clk_disable(clk_get(NULL, "usbhost_120m_fck"));
	clk_disable(clk_get(NULL, "usbhost_48m_fck"));

	/* the omap usb host auto-idle is not fully functional,
	 * manually enable/disable usbtll_ick during
	 * the suspend/resume time.
	 */
	clk_disable(clk_get(NULL, "usbtll_ick"));

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	if (ohci_irq_num)
		enable_irq(ohci_irq_num);

	ohci_to_hcd(ohci)->state = HC_STATE_SUSPENDED;

	return ret;
}

static int omap_ohci_bus_resume(struct usb_hcd *hcd)
{
	int ret = 0;


	printk(KERN_INFO "%s Enter. \n", __func__);
	if (ohci_irq_num)
		disable_irq(ohci_irq_num);
	/* the omap usb host auto-idle is not fully functional,
	 * manually enable/disable usbtll_ick during
	 * the suspend/resume time.
	 */
	clk_enable(clk_get(NULL, "usbtll_ick"));
	clk_enable(clk_get(NULL, "usbtll_fck"));
	clk_enable(clk_get(NULL, "usbhost_120m_fck"));
	clk_enable(clk_get(NULL, "usbhost_48m_fck"));
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	if (ohci_irq_num)
		enable_irq(ohci_irq_num);

	ohci_finish_controller_resume(hcd);
	ret = ohci_bus_resume(hcd);
	return ret;
}
#endif /* CONFIG_PM */

static void omap_ohci_shutdown(struct usb_hcd *hcd)
{
	dev_dbg(hcd->self.controller, "%s %lu\n", __func__, jiffies);
	clk_enable(clk_get(NULL, "usbtll_fck"));
	clk_enable(clk_get(NULL, "usbhost_120m_fck"));
	clk_enable(clk_get(NULL, "usbhost_48m_fck"));
	ohci_shutdown(hcd);
	ohci_irq_num = 0;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_omap_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"OMAP OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ohci_omap_init,
	.start =		ohci_omap_start,
	.stop =			ohci_omap_stop,
	.shutdown =		omap_ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		omap_ohci_bus_suspend,
	.bus_resume =		omap_ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_omap_drv_probe(struct platform_device *dev)
{
	return usb_hcd_omap_probe(&ohci_omap_hc_driver, dev);
}

static int ohci_hcd_omap_drv_remove(struct platform_device *dev)
{
	struct usb_hcd		*hcd = platform_get_drvdata(dev);

	usb_hcd_omap_remove(hcd, dev);
	platform_set_drvdata(dev, NULL);

	return 0;
}

/*-------------------------------------------------------------------------*/

#ifdef	CONFIG_PM

static int ohci_omap_suspend(struct platform_device *dev, pm_message_t message)
{
	return 0;
}

static int ohci_omap_resume(struct platform_device *dev)
{
	return 0;
}

#endif

/*-------------------------------------------------------------------------*/

/*
 * Driver definition to register with the OMAP bus
 */
static struct platform_driver ohci_hcd_omap_driver = {
	.probe		= ohci_hcd_omap_drv_probe,
	.remove		= ohci_hcd_omap_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend        = ohci_omap_suspend,
	.resume         = ohci_omap_resume,
#endif
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ohci",
	},
};

MODULE_ALIAS("platform:ohci");

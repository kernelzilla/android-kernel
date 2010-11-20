/*
 *  arch/arm/include/mach/serial_omap.h
 *
 *  Copyright (C) 2009 San Mehat
 *  Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _ASM_ARM_MACH_SERIAL_OMAP_H
#define _ASM_ARM_MACH_SERIAL_OMAP_H

#include <linux/serial_core.h>
#include <linux/platform_device.h>

#ifdef CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL
#define SERIALOMAP_AUTO_RTS 0x01
#define SERIALOMAP_AUTO_CTS 0x02
#endif

/*
 * This is the platform device platform_data structure
 */

struct plat_serialomap_port {
	upf_t		flags;
	int		disabled;
	void		*private_data;
	void __iomem	*membase;
	unsigned char	regshift;
	int		irq;
#ifdef CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL
	unsigned char   rtscts;     /* bit0: rts, bit1: cts */
#endif
	int		wake_gpio_strobe;
};

#ifdef CONFIG_PM
int are_driveromap_uarts_active(int num);
#endif

#endif

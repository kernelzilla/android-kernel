/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef _LINUX_OMAP_MDM_CTRL_H__
#define _LINUX_OMAP_MDM_CTRL_H__

#define OMAP_MDM_CTRL_MODULE_NAME "omap_mdm_ctrl"

/* The number of bytes needed to return interrupt fire status */
#define GPIO_BYTE_COUNT        1
/* The number of GPIOs that can experience interrupts */
#define GPIO_INTERRUPT_COUNT   3

struct omap_mdm_ctrl_platform_data {
	unsigned int bp_ready_ap_gpio;
	unsigned int bp_ready2_ap_gpio;
	unsigned int bp_resout_gpio;
	unsigned int bp_pwron_gpio;
	unsigned int ap_to_bp_pshold_gpio;
	unsigned int ap_to_bp_flash_en_gpio;
};

#endif  /* _LINUX_OMAP_MDM_CTRL_H__ */


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

#ifndef _LINUX_SFH7743_H_
#define _LINUX_SFH7743_H_

#include <linux/regulator/consumer.h>

#define SFH7743_PROXIMITY_NEAR	0
#define SFH7743_PROXIMITY_FAR	0xFFFFFFFF
#define SFH7743_MODULE_NAME	"sfh7743"
#define SFH7743_NO_REGULATOR	"no_reg"

#define SFH7743_DISABLED 0
#define SFH7743_ENABLED  1

#ifdef __KERNEL__

struct sfh7743_platform_data {
	u8 gpio_prox_int;
	const char *regulator;
} __attribute__ ((packed));

#endif /* __KERNEL__ */

#endif /* _LINUX_SFH7743_H__ */

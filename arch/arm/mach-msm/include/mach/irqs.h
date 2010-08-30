/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_IRQS_H
#define __ASM_ARCH_MSM_IRQS_H

#define MSM_IRQ_BIT(irq)     (1 << ((irq) & 31))

#define NR_MSM_IRQS 64
#define NR_BOARD_IRQS 64

#if defined(CONFIG_ARCH_QSD)
#include "irqs-8xxx.h"
#include "sirc.h"
#else
#include "irqs-7xxx.h"
#endif

#define NR_IRQS (NR_MSM_IRQS + NR_GPIO_IRQS + NR_BOARD_IRQS)

#define MSM_GPIO_TO_INT(n) (NR_MSM_IRQS + (n))

#endif

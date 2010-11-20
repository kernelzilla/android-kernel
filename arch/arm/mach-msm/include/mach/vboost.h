/* linux/arch/arm/mach-msm/include/mach/vboost.h
 *
 * Copyright (C) 2008-2009 Motorola, Inc.
 * Author: Alina Yakovleva <qvdh43@motorola.com>
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

#ifndef __VBOOST_H__
#define __VBOOST_H__
#include <linux/kernel.h>

enum __vboost_users {
    VBOOST_FAN5646    = 1 << 0, 
    VBOOST_PM7540_KPD = 1 << 1, 
    VBOOST_CAMERA     = 1 << 2,
    VBOOST_PM7540_BTN = 1 << 3 
};

extern int vboost_enable (uint32_t app_mask);
extern int vboost_disable (uint32_t app_mask);

#endif

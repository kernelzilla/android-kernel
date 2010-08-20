/* linux/include/asm-arm/arch-msm/powerup_info.h
 *
 * Copyright (C) 2009 Motorola, Inc.
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

#ifndef _MOT_POWERUP_INFO_H_
#define _MOT_POWERUP_INFO_H_

#include <mot/mot_handover.h>

void set_powerdown_panic(void);

void set_powerdown_reset(void);

unsigned powerup_reason_charger(void);

#endif /* _MOT_POWERUP_INFO_H_ */

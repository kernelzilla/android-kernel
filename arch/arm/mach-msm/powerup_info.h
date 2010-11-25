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

typedef enum
{
    PU_MAIN_NO_INDICATION_YET_PU = 0,
    PU_MAIN_POWERCUT_RESET_PU,
    PU_MAIN_FACTORY_CABLE_PU,
    PU_MAIN_CHARGER_PU,
    PU_MAIN_USB_DATA_CABLE_PU,
    PU_MAIN_PWR_KEY_PRESS_PU,
    PU_MAIN_WDOG_TIMEOUT_PU,
    PU_MAIN_SOFTWARE_PANIC_PU,
    PU_MAIN_SOFTWARE_RESET_PU,
    PU_MAIN_AP_FLASH_PU,
    PU_MAIN_SYSTEM_REBOOT_PU,
    PU_MAIN_SOFTWARE_BP_PANIC_PU,
  
    /*  Add any new PU reason above this line */
    PU_MAIN_MAX_PU_REASON
} PU_MAIN_POWERUP_REASON_TYPE;

void set_powerdown_panic(void);

void set_powerdown_reset(void);

unsigned powerup_reason_charger(void);

#endif /* _MOT_POWERUP_INFO_H_ */

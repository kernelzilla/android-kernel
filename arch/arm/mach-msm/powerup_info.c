/* linux/arch/arm/mach-msm/powerup_info.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/setup.h>
#include "proc_comm.h"
#include "smd_private.h"
#include "powerup_info.h"

#define INFO_SIZE      20

/*
 * supports charge only mode
 */
unsigned powerup_reason_charger(void)
{
    if(PU_MAIN_CHARGER_PU == mot_handover_get_powerup_reason())
        return 1;
    else
        return 0;
}
EXPORT_SYMBOL(powerup_reason_charger);

/*
 * get_powerup_info fills in the /proc/powerup_info information.
 */

static int get_powerup_info(char *buf, char **start,
			    off_t offset, int count,
			    int *eof, void *data)
{
    int len = 0;
    char powerup_reason[INFO_SIZE];
    PU_MAIN_POWERUP_REASON_TYPE pu_reason = mot_handover_get_powerup_reason();

    switch (pu_reason)
    {
	case PU_MAIN_WDOG_TIMEOUT_PU:
		sprintf(powerup_reason, "WDOG_TIMEOUT");
		break;

	case PU_MAIN_PANIC_RESET_PU:
		sprintf(powerup_reason, "PANIC_RESET");
		break;

	case PU_MAIN_POWERCUT_RESET_PU:
		sprintf(powerup_reason, "POWERCUT_RESET");
		break;

	case PU_MAIN_SOFTWARE_RESET_PU:
		sprintf(powerup_reason, "SOFTWARE_RESET");
		break;

	case PU_MAIN_SHORT_POWER_KEY_PRESS_PU:
		sprintf(powerup_reason, "SHORT_POWER_KEY_PRESS");
		break;

	case PU_MAIN_USB_DATA_CABLE_PU:
		sprintf(powerup_reason, "USB_DATA_CABLE");
		break;

	case PU_MAIN_ACC_PWR_KEY_PRESS_PU:
		sprintf(powerup_reason, "ACC_PWR_KEY_PRESS");
		break;

	case PU_MAIN_CHARGER_PU:
		sprintf(powerup_reason, "CHARGER");
		break;

	case PU_MAIN_AIRPLANE_MODE_PU:
		sprintf(powerup_reason, "AIRPLANE_MODE");
		break;

	case PU_MAIN_PWR_KEY_PRESS_PU:
		sprintf(powerup_reason, "PWR_KEY_PRESS");
		break;

	case PU_MAIN_EXTERNAL_PWR_PU:
		sprintf(powerup_reason, "EXTERNAL_PWR");
		break;

	case PU_MAIN_FASTBOOT_MODE_PU:
		sprintf(powerup_reason, "FASTBOOT_MODE");
		break;

	case PU_MAIN_RECOVERY_FORCE_MODE_PU:
		sprintf(powerup_reason, "RECOVERY_FORCE_MODE");
		break;

	case PU_MAIN_RECOVERY_OTA_MODE_PU:
		sprintf(powerup_reason, "RECOVERY_OTA_MODE");
		break;

	case PU_MAIN_FOTA_OTA_MODE_PU:
		sprintf(powerup_reason, "FOTA_OTA_MODE");
		break;

	case PU_MAIN_FLASH_MODE_PU:
		sprintf(powerup_reason, "FLASH_MODE");
		break;

	default:
		sprintf(powerup_reason, "UNKNOWN_REASON");
		break;
    }

    len += sprintf(buf, "%s", powerup_reason);
    return len;
}

static struct proc_dir_entry *proc_powerup_info = NULL;
extern struct proc_dir_entry proc_root;

int __init powerup_info_init_module(void)
{

    proc_powerup_info = &proc_root;
    proc_powerup_info->owner = THIS_MODULE;
    create_proc_read_entry("powerup_info", 0, NULL,
			   get_powerup_info, NULL);
    return 0;
}

void __exit powerup_info_cleanup_module(void)
{
    if (proc_powerup_info) {
	remove_proc_entry("powerup_info", proc_powerup_info);
	proc_powerup_info = NULL;
    }
}

module_init(powerup_info_init_module);
module_exit(powerup_info_cleanup_module);

MODULE_AUTHOR("MOTOROLA");

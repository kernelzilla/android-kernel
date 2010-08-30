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

typedef struct
{
    /* Stores boot mode reason for apps */
    uint32_t apps_boot_reason;
} appsbl_interface_type;

void set_powerdown_panic(void)
{
	unsigned dummy = 0 ; /* Dummy variable for the second argument in the meta_proc api */

	meta_proc(PROCCOMM_MODEM_SET_PANIC_REASON, &dummy);
}
EXPORT_SYMBOL(set_powerdown_panic);

void set_powerdown_reset(void)
{
	unsigned dummy = 0 ; /* Dummy variable for the second argument in the meta_proc api */

	meta_proc(PROCCOMM_MODEM_SET_HARD_RESET_REASON, &dummy);
}
EXPORT_SYMBOL(set_powerdown_reset);

static PU_MAIN_POWERUP_REASON_TYPE get_modem_powerup_reason(void)
{
    appsbl_interface_type *smem_appsbl_interface = NULL ;

    smem_appsbl_interface = smem_alloc(SMEM_APPS_BOOT_MODE, sizeof(appsbl_interface_type));

    if (!smem_appsbl_interface)
    {
	return 0;
    }

    if (smem_appsbl_interface->apps_boot_reason >= PU_MAIN_MAX_PU_REASON)
    {
	return 0;
    }

    return smem_appsbl_interface->apps_boot_reason;
}

/*
 * supports charge only mode
 */
unsigned powerup_reason_charger(void)
{
    if(PU_MAIN_CHARGER_PU == get_modem_powerup_reason())
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
    char powerup_reason[20];
    PU_MAIN_POWERUP_REASON_TYPE pu_reason = get_modem_powerup_reason();

    switch (pu_reason)
    {
	case PU_MAIN_POWERCUT_RESET_PU:
		sprintf(powerup_reason, "POWER_CUT");
		break;

	case PU_MAIN_FACTORY_CABLE_PU:
		sprintf(powerup_reason, "FACTORY_CABLE");
		break;

	case PU_MAIN_CHARGER_PU:
		sprintf(powerup_reason, "CHARGER");
		break;

	case PU_MAIN_USB_DATA_CABLE_PU:
		sprintf(powerup_reason, "USB_CABLE");
		break;

	case PU_MAIN_PWR_KEY_PRESS_PU:
		sprintf(powerup_reason, "POWER_KEY_PRESS");
		break;

	case PU_MAIN_WDOG_TIMEOUT_PU:
		sprintf(powerup_reason, "WATCHDOG_TIMEOUT");
		break;

	case PU_MAIN_SOFTWARE_PANIC_PU:
		sprintf(powerup_reason, "PANIC");
		break;

	case PU_MAIN_SOFTWARE_BP_PANIC_PU:
		sprintf(powerup_reason, "BP_PANIC");
		break;

	case PU_MAIN_SOFTWARE_RESET_PU:
		sprintf(powerup_reason, "HARD_RESET");
		break;

	case PU_MAIN_AP_FLASH_PU:
		sprintf(powerup_reason, "AP_FLASH");
		break;

	case PU_MAIN_SYSTEM_REBOOT_PU:
		sprintf(powerup_reason, "SYS_REBOOT");
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

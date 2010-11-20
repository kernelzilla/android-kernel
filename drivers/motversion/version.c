/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/**********************************************************************
 * panic.c
 *
 * A driver for forcing a panic on either the modem or Linux side of
 * an Android phone.
 *
 **********************************************************************/
/*===========================================================================

                        EDIT HISTORY FOR MODULE

  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.


when          who         what, where, why
--------     ------       ----------------------------------------------------------
2009-5-1    R Stoddard    Initial authoring of mot_version


===========================================================================*/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>

#include "../../init/mot_version.h"

MODULE_LICENSE("GPL");

#ifndef CONFIG_MACH_MOT
static dev_t mot_version_dev;
#endif

int motver_readproc(char* page, char** start, off_t offset, int count, int *eof, void* data)
{
	int len = 0;

	for(; len < count; len++)
	{
		if((page[len] = PRODUCT_VERSION_STR[offset+len])  == 0)
		{
			break;
		}
	}

	return len;
}


static int motver_init(void)
{
	printk(KERN_EMERG "Motorola software version: %s", PRODUCT_VERSION_STR);
	create_proc_read_entry("mot_version", 0, NULL, motver_readproc, NULL);
	return 0;
}

static void motver_exit(void)
{
	remove_proc_entry("mot_version", NULL);
}


module_init(motver_init);
module_exit(motver_exit);

/*
  * Copyright (C) 2007-2009 Motorola, Inc.
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

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/io.h>


#define DIE_ID_REG_BASE                 (L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET            	0x218
#define DIE_ID_REG_NUM                  4

static int phoneid_proc_show(struct seq_file *m, void *v)
{
	unsigned int val[DIE_ID_REG_NUM] = {0}; /* Num of DIE_ID reg = 4 */
	unsigned int reg;
	int i;

	reg = (unsigned long)(DIE_ID_REG_BASE + DIE_ID_REG_OFFSET);

	for (i = 0; i < DIE_ID_REG_NUM; i++)
		val[i] = omap_readl(reg + 4 * i);

	seq_printf(m, "%08x%08x%08x%08x\n", val[3], val[2], val[1], val[0]);

	return 0;
}

static int phoneid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, phoneid_proc_show, NULL);
}

static const struct file_operations phoneid_proc_fops = {
	.open		= phoneid_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_phoneid_init(void)
{
	proc_create("phoneid", 0, NULL, &phoneid_proc_fops);
	return 0;
}

module_init(proc_phoneid_init);

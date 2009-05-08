/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
 * ioctl
 * currently plink-management only.
 */

#include <linux/uaccess.h>
#include "aufs.h"

long aufs_ioctl_dir(struct file *file, unsigned int cmd,
		    unsigned long arg __maybe_unused)
{
	long err;
	struct super_block *sb;
	struct au_sbinfo *sbinfo;

	err = -EACCES;
	if (!capable(CAP_SYS_ADMIN))
		goto out;

	err = 0;
	sb = file->f_dentry->d_sb;
	sbinfo = au_sbi(sb);
	switch (cmd) {
	case AUFS_CTL_PLINK_MAINT:
		/*
		 * pseudo-link maintenance mode,
		 * cleared by aufs_release_dir()
		 */
		si_write_lock(sb);
		if (!au_ftest_si(sbinfo, MAINTAIN_PLINK)) {
			au_fset_si(sbinfo, MAINTAIN_PLINK);
			au_fi(file)->fi_maintain_plink = 1;
		} else
			err = -EBUSY;
		si_write_unlock(sb);
		break;
	case AUFS_CTL_PLINK_CLEAN:
		if (au_opt_test(sbinfo->si_mntflags, PLINK)) {
			aufs_write_lock(sb->s_root);
			au_plink_put(sb);
			aufs_write_unlock(sb->s_root);
		}
		break;
	default:
		err = -EINVAL;
	}

 out:
	return err;
}

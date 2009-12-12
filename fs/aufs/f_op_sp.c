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
 * file operations for special files
 */

#include <linux/fs_stack.h>
#include <linux/poll.h>
#include "aufs.h"

/* currently, support only FIFO */
enum {AuSp_FIFO, AuSp_FIFO_R, AuSp_FIFO_W, AuSp_FIFO_RW,
      /* AuSp_SOCK, AuSp_CHR, AuSp_BLK, */
      AuSp_Last};
static const struct file_operations aufs_sp_fop[];

int au_special_file(umode_t mode)
{
	int ret;

	ret = 0;
	switch (mode & S_IFMT) {
	case S_IFIFO:
#if 0
	case S_IFCHR:
	case S_IFBLK:
	case S_IFSOCK:
#endif
		ret = 1;
	}

	return ret;
}

void au_init_special_fop(struct inode *inode, umode_t mode, dev_t rdev)
{
	init_special_inode(inode, mode, rdev);

	switch (mode & S_IFMT) {
	case S_IFIFO:
		inode->i_fop = aufs_sp_fop + AuSp_FIFO;
		/*FALLTHROUGH*/
	case S_IFCHR:
	case S_IFBLK:
	case S_IFSOCK:
		break;
	default:
		AuDebugOn(1);
	}
}

static void au_dbg_sp_fop(struct file *file)
{
#ifdef CONFIG_AUFS_DEBUG
	struct file *h_file = au_h_fptr(file, au_fbstart(file));
	const struct file_operations *fop1 = h_file->f_op,
		*fop2 = file->f_op;

#define Compare(name)	AuDebugOn(!!fop1->name != !!fop2->name)
	Compare(llseek);
	Compare(read);
	Compare(write);
	Compare(aio_read);
	Compare(aio_write);
	Compare(readdir);
	Compare(poll);
	Compare(ioctl);
	Compare(unlocked_ioctl);
	Compare(compat_ioctl);
	Compare(mmap);
	Compare(open);
	Compare(flush);
	Compare(release);
	Compare(fsync);
	Compare(aio_fsync);
	Compare(fasync);
	Compare(lock);
	Compare(sendpage);
	Compare(get_unmapped_area);
	Compare(check_flags);
	Compare(flock);
	Compare(splice_write);
	Compare(splice_read);
	Compare(setlease);
#undef Compare
#endif
}

/* ---------------------------------------------------------------------- */

static int au_cpup_sp(struct dentry *dentry, aufs_bindex_t bcpup)
{
	int err;
	struct au_pin pin;
	struct dentry *parent;

	AuDbg("%.*s\n", AuDLNPair(dentry));

	err = 0;
	parent = dget_parent(dentry);
	di_write_lock_parent(parent);
	if (!au_h_dptr(parent, bcpup)) {
		err = au_cpup_dirs(dentry, bcpup);
		if (unlikely(err))
			goto out;
	}

	err = au_pin(&pin, dentry, bcpup, AuOpt_UDBA_NONE,
		     AuPin_DI_LOCKED | AuPin_MNT_WRITE);
	if (!err) {
		err = au_sio_cpup_simple(dentry, bcpup, -1, AuCpup_DTIME);
		au_unpin(&pin);
	}

 out:
	di_write_unlock(parent);
	dput(parent);
	return err;
}

static int au_do_open_sp(struct file *file, int flags)
{
	int err;
	aufs_bindex_t bcpup, bindex, bend;
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	AuDbg("%.*s\n", AuDLNPair(dentry));

	err = 0;
	sb = dentry->d_sb;
	bend = au_dbstart(dentry);
	if (au_br_rdonly(au_sbr(sb, bend))) {
		/* copyup first */
		bcpup = -1;
		for (bindex = 0; bindex < bend; bindex++)
			if (!au_br_rdonly(au_sbr(sb, bindex))) {
				bcpup = bindex;
				break;
			}
		if (bcpup >= 0) {
			/* need to copyup */
			di_read_unlock(dentry, AuLock_IR);
			di_write_lock_child(dentry);
			if (bcpup < au_dbstart(dentry))
				err = au_cpup_sp(dentry, bcpup);
			di_downgrade_lock(dentry, AuLock_IR);
		} else
			err = -EIO;
	}

	if (!err)
		err = au_do_open_nondir(file, file->f_flags);
	if (!err) {
		switch (file->f_mode & (FMODE_READ | FMODE_WRITE)) {
		case FMODE_READ:
			file->f_op = aufs_sp_fop + AuSp_FIFO_R;
			break;
		case FMODE_WRITE:
			file->f_op = aufs_sp_fop + AuSp_FIFO_W;
			break;
		case FMODE_READ | FMODE_WRITE:
			file->f_op = aufs_sp_fop + AuSp_FIFO_RW;
			break;
		default:
			BUG();
		}
		au_dbg_sp_fop(file);
	}

	return err;
}

static int aufs_open_sp(struct inode *inode, struct file *file)
{
	return au_do_open(file, au_do_open_sp);
}

/* ---------------------------------------------------------------------- */

static ssize_t aufs_read_sp(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	ssize_t err;
	struct inode *inode;
	struct super_block *sb;
	struct file *h_file;

	inode = file->f_dentry->d_inode;
	sb = inode->i_sb;
	si_read_lock(sb, AuLock_FLUSH);
	fi_read_lock(file);
	h_file = au_h_fptr(file, au_fbstart(file));
	fi_read_unlock(file);
	si_read_unlock(sb);

	err = vfsub_read_u(h_file, buf, count, ppos);

	si_noflush_read_lock(sb);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	ii_read_lock_child(inode);
	fsstack_copy_attr_atime(inode, h_file->f_dentry->d_inode);
	ii_read_unlock(inode);
	si_read_unlock(sb);

	return err;
}

static ssize_t aufs_write_sp(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	ssize_t err;
	struct inode *inode;
	struct super_block *sb;
	struct file *h_file;
	char __user *buf = (char __user *)ubuf;

	inode = file->f_dentry->d_inode;
	sb = inode->i_sb;
	si_read_lock(sb, AuLock_FLUSH);
	fi_read_lock(file);
	h_file = au_h_fptr(file, au_fbstart(file));
	fi_read_unlock(file);
	si_read_unlock(sb);

	err = vfsub_write_u(h_file, buf, count, ppos);

	si_noflush_read_lock(sb);
	ii_write_lock_child(inode);
	au_cpup_attr_timesizes(inode);
	inode->i_mode = h_file->f_dentry->d_inode->i_mode;
	ii_write_unlock(inode);
	si_read_unlock(sb);

	return err;
}

static ssize_t aufs_aio_read_sp(struct kiocb *kio, const struct iovec *iov,
				unsigned long nv, loff_t pos)
{
	ssize_t err;
	struct file *file, *h_file;
	struct inode *inode;
	struct super_block *sb;

	file = kio->ki_filp;
	inode = file->f_dentry->d_inode;
	sb = inode->i_sb;
	si_read_lock(sb, AuLock_FLUSH);
	fi_read_lock(file);
	h_file = au_h_fptr(file, au_fbstart(file));
	fi_read_unlock(file);
	si_read_unlock(sb);

	AuDebugOn(!h_file->f_op || !h_file->f_op->aio_read);
	err = au_do_aio(h_file, MAY_READ, kio, iov, nv, pos);

	si_noflush_read_lock(sb);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	ii_read_lock_child(inode);
	fsstack_copy_attr_atime(inode, h_file->f_dentry->d_inode);
	ii_read_unlock(inode);
	si_read_unlock(sb);

	return err;
}

static ssize_t aufs_aio_write_sp(struct kiocb *kio, const struct iovec *iov,
				 unsigned long nv, loff_t pos)
{
	ssize_t err;
	struct inode *inode;
	struct super_block *sb;
	struct file *file, *h_file;

	file = kio->ki_filp;
	inode = file->f_dentry->d_inode;
	sb = inode->i_sb;
	/* mutex_lock(&inode->i_mutex); */
	si_read_lock(sb, AuLock_FLUSH);
	fi_read_lock(file);
	h_file = au_h_fptr(file, au_fbstart(file));
	fi_read_unlock(file);
	si_read_unlock(sb);

	AuDebugOn(!h_file->f_op || !h_file->f_op->aio_write);
	err = au_do_aio(h_file, MAY_WRITE, kio, iov, nv, pos);

	si_noflush_read_lock(sb);
	ii_write_lock_child(inode);
	au_cpup_attr_timesizes(inode);
	inode->i_mode = h_file->f_dentry->d_inode->i_mode;
	ii_write_unlock(inode);
	si_read_unlock(sb);
	/* mutex_unlock(&inode->i_mutex); */

	return err;
}

/* ---------------------------------------------------------------------- */

static unsigned int aufs_poll_sp(struct file *file, poll_table *wait)
{
	struct super_block *sb;
	struct file *h_file;

	sb = file->f_dentry->d_inode->i_sb;
	si_read_lock(sb, AuLock_FLUSH);
	fi_read_lock(file);
	h_file = au_h_fptr(file, au_fbstart(file));
	fi_read_unlock(file);
	si_read_unlock(sb);

	AuDebugOn(!h_file->f_op || !h_file->f_op->poll);
	return h_file->f_op->poll(h_file, wait);
}

static long aufs_unlocked_ioctl_sp(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	struct super_block *sb;
	struct file *h_file;

	sb = file->f_dentry->d_inode->i_sb;
	si_read_lock(sb, AuLock_FLUSH);
	fi_read_lock(file);
	h_file = au_h_fptr(file, au_fbstart(file));
	fi_read_unlock(file);
	si_read_unlock(sb);

	AuDebugOn(!h_file->f_op || !h_file->f_op->unlocked_ioctl);
	return h_file->f_op->unlocked_ioctl(h_file, cmd, arg);
}

static int aufs_fasync_sp(int fd, struct file *file, int on)
{
	struct super_block *sb;
	struct file *h_file;

	sb = file->f_dentry->d_inode->i_sb;
	si_read_lock(sb, AuLock_FLUSH);
	fi_read_lock(file);
	h_file = au_h_fptr(file, au_fbstart(file));
	fi_read_unlock(file);
	si_read_unlock(sb);

	AuDebugOn(!h_file->f_op || !h_file->f_op->fasync);
	return h_file->f_op->fasync(fd, h_file, on);
}

/* ---------------------------------------------------------------------- */

/* cf. linux/fs/pipe.c */
static const struct file_operations aufs_sp_fop[] = {
	[AuSp_FIFO] = {
		.open		= aufs_open_sp
	},
	[AuSp_FIFO_R] = {
		.llseek		= no_llseek,
		.read		= aufs_read_sp,
		.write		= aufs_write_sp,
		.aio_read	= aufs_aio_read_sp,
		.poll		= aufs_poll_sp,
		.unlocked_ioctl	= aufs_unlocked_ioctl_sp,
		.open		= aufs_open_sp,
		.release	= aufs_release_nondir,
		.fasync		= aufs_fasync_sp
	},
	[AuSp_FIFO_W] = {
		.llseek		= no_llseek,
		.read		= aufs_read_sp,
		.write		= aufs_write_sp,
		.aio_write	= aufs_aio_write_sp,
		.poll		= aufs_poll_sp,
		.unlocked_ioctl	= aufs_unlocked_ioctl_sp,
		.open		= aufs_open_sp,
		.release	= aufs_release_nondir,
		.fasync		= aufs_fasync_sp
	},
	[AuSp_FIFO_RW] = {
		.llseek		= no_llseek,
		.read		= aufs_read_sp,
		.write		= aufs_write_sp,
		.aio_read	= aufs_aio_read_sp,
		.aio_write	= aufs_aio_write_sp,
		.poll		= aufs_poll_sp,
		.unlocked_ioctl	= aufs_unlocked_ioctl_sp,
		.open		= aufs_open_sp,
		.release	= aufs_release_nondir,
		.fasync		= aufs_fasync_sp
	}
};

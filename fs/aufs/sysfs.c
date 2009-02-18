/*
 * Copyright (C) 2005-2009 Junjiro Okajima
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
 * sysfs interface
 */

#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/sysfs.h>
#include "aufs.h"

#ifdef CONFIG_AUFS_DEBUG
static ssize_t debug_show(struct kobject *kobj __maybe_unused,
			  struct kobj_attribute *attr __maybe_unused,
			  char *buf)
{
	return sprintf(buf, "%d\n", au_debug_test());
}

static ssize_t debug_store(struct kobject *kobj __maybe_unused,
			   struct kobj_attribute *attr __maybe_unused,
			   const char *buf, size_t sz)
{
	if (unlikely(!sz || (*buf != '0' && *buf != '1')))
		return -EOPNOTSUPP;

	if (*buf == '0')
		au_debug(0);
	else if (*buf == '1')
		au_debug(1);
	return sz;
}

static struct kobj_attribute au_debug_attr = __ATTR(debug, S_IRUGO | S_IWUSR,
						    debug_show, debug_store);
#endif

static struct attribute *au_attr[] = {
#ifdef CONFIG_AUFS_DEBUG
	&au_debug_attr.attr,
#endif
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sysaufs_attr_group_body = {
	.attrs = au_attr
};

struct attribute_group *sysaufs_attr_group = &sysaufs_attr_group_body;

/* ---------------------------------------------------------------------- */

/*
 * they are copied from linux/lib/kobject.c,
 * and will be exported in the future.
 */
static ssize_t au_attr_show(struct kobject *kobj, struct attribute *attr,
			    char *buf)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->show)
		ret = kattr->show(kobj, kattr, buf);
	return ret;
}

#ifdef CONFIG_AUFS_DEBUG
static ssize_t au_attr_store(struct kobject *kobj, struct attribute *attr,
			     const char *buf, size_t count)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->store)
		ret = kattr->store(kobj, kattr, buf, count);
	return ret;
}
#endif

static struct sysfs_ops sysaufs_ops = {
	.show   = au_attr_show,
#ifdef CONFIG_AUFS_DEBUG
	.store  = au_attr_store
#endif
};

static struct kobj_type sysaufs_ktype_body = {
	.sysfs_ops = &sysaufs_ops
};
struct kobj_type *sysaufs_ktype = &sysaufs_ktype_body;

/* ---------------------------------------------------------------------- */

static int sysaufs_sbi_xi(struct seq_file *seq, struct file *xf,
			  struct kstat *st)
{
	int err;

	err = vfs_getattr(xf->f_vfsmnt, xf->f_dentry, st);
	if (!err) {
		seq_printf(seq, "%llux%lu %lld",
			   st->blocks, st->blksize, (long long)st->size);
		seq_putc(seq, '\n');
	} else
		seq_printf(seq, "err %d\n", err);

	return err;
}

int sysaufs_si_xino(struct seq_file *seq, struct super_block *sb)
{
	int err;
	aufs_bindex_t bend, bindex;
	struct kstat st;
	struct au_sbinfo *sbinfo;
	struct file *xf;

	err = 0;
	sbinfo = au_sbi(sb);
	if (!au_opt_test(au_mntflags(sb), XINO))
		goto out; /* success */

	xf = sbinfo->si_xib;
	err = au_xino_path(seq, xf);
	seq_putc(seq, '\n');
	if (!err)
		err = sysaufs_sbi_xi(seq, xf, &st);

	bend = au_sbend(sb);
	for (bindex = 0; !err && bindex <= bend; bindex++) {
		xf = au_sbr(sb, bindex)->br_xino.xi_file;
		if (!xf)
			continue;

		seq_printf(seq, "%d: ", bindex);
		err = vfs_getattr(xf->f_vfsmnt, xf->f_dentry, &st);
		if (!err) {
			seq_printf(seq, "%ld, %llux%lu %lld",
				   (long)file_count(xf), st.blocks, st.blksize,
				   (long long)st.size);
			seq_putc(seq, '\n');
		} else
			seq_printf(seq, "err %d\n", err);
	}

 out:
	return err;
}

/*
 * the lifetime of branch is independent from the entry under sysfs.
 * sysfs handles the lifetime of the entry, and never call ->show() after it is
 * unlinked.
 */
#define SysaufsBr_PREFIX "br"
static int sysaufs_sbi_br(struct seq_file *seq, struct super_block *sb,
			  aufs_bindex_t bindex)
{
	int err;
	struct path path;
	struct dentry *root;
	struct au_branch *br;

	err = -ENOENT;
	if (unlikely(au_sbend(sb) < bindex))
		goto out;

	err = 0;
	root = sb->s_root;
	di_read_lock_parent(root, !AuLock_IR);
	br = au_sbr(sb, bindex);
	path.mnt = br->br_mnt;
	path.dentry = au_h_dptr(root, bindex);
	au_seq_path(seq, &path);
	di_read_unlock(root, !AuLock_IR);
	seq_printf(seq, "=%s\n", au_optstr_br_perm(br->br_perm));

 out:
	return err;
}

/* ---------------------------------------------------------------------- */

static struct seq_file *au_seq(char *p, ssize_t len)
{
	struct seq_file *seq;

	seq = kzalloc(sizeof(*seq), GFP_NOFS);
	if (seq) {
		/* mutex_init(&seq.lock); */
		seq->buf = p;
		seq->size = len;
		return seq; /* success */
	}

	seq = ERR_PTR(-ENOMEM);
	return seq;
}

/* todo: file size may exceed PAGE_SIZE */
ssize_t sysaufs_si_show(struct kobject *kobj, struct attribute *attr,
			 char *buf)
{
	ssize_t err;
	long l;
	struct au_sbinfo *sbinfo;
	struct super_block *sb;
	struct seq_file *seq;
	char *name;
	struct attribute **cattr;

	sbinfo = container_of(kobj, struct au_sbinfo, si_kobj);
	sb = sbinfo->si_sb;
	si_noflush_read_lock(sb);

	seq = au_seq(buf, PAGE_SIZE);
	err = PTR_ERR(seq);
	if (IS_ERR(seq))
		goto out;

	name = (void *)attr->name;
	cattr = sysaufs_si_attrs;
	while (*cattr) {
		if (!strcmp(name, (*cattr)->name)) {
			err = container_of(*cattr, struct sysaufs_si_attr, attr)
				->show(seq, sb);
			goto out_seq;
		}
		cattr++;
	}

	if (!strncmp(name, SysaufsBr_PREFIX, sizeof(SysaufsBr_PREFIX) - 1)) {
		name += sizeof(SysaufsBr_PREFIX) - 1;
		err = strict_strtol(name, 10, &l);
		if (!err)
			err = sysaufs_sbi_br(seq, sb, (aufs_bindex_t)l);
		goto out_seq;
	}
	BUG();

 out_seq:
	if (!err) {
		err = seq->count;
		/* sysfs limit */
		if (unlikely(err == PAGE_SIZE))
			err = -EFBIG;
	}
	kfree(seq);
 out:
	si_read_unlock(sb);
	return err;
}

/* ---------------------------------------------------------------------- */

void sysaufs_br_init(struct au_branch *br)
{
	br->br_attr.name = br->br_name;
	br->br_attr.mode = S_IRUGO;
	br->br_attr.owner = THIS_MODULE;
}

void sysaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex)
{
	struct au_sbinfo *sbinfo;
	aufs_bindex_t bend;

	if (!sysaufs_brs)
		return;

	sbinfo = au_sbi(sb);
	bend = au_sbend(sb);
	for (; bindex <= bend; bindex++)
		sysfs_remove_file(&sbinfo->si_kobj,
				  &au_sbr(sb, bindex)->br_attr);
}

void sysaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex)
{
	int err;
	aufs_bindex_t bend;
	struct kobject *kobj;
	struct au_branch *br;

	if (!sysaufs_brs)
		return;

	kobj = &au_sbi(sb)->si_kobj;
	bend = au_sbend(sb);
	for (; bindex <= bend; bindex++) {
		br = au_sbr(sb, bindex);
		snprintf(br->br_name, sizeof(br->br_name),
			 SysaufsBr_PREFIX "%d", bindex);
		err = sysfs_create_file(kobj, &br->br_attr);
		if (unlikely(err))
			AuWarn("failed %s under sysfs(%d)\n", br->br_name, err);
	}
}

/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * sysfs interface
 */

#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/sysfs.h>
#include "aufs.h"

static struct attribute *au_attr[] = {
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sysaufs_attr_group_body = {
	.attrs = au_attr
};

struct attribute_group *sysaufs_attr_group = &sysaufs_attr_group_body;

/* ---------------------------------------------------------------------- */

int sysaufs_si_xi_path(struct seq_file *seq, struct super_block *sb)
{
	int err;

	err = 0;
	if (au_opt_test(au_mntflags(sb), XINO)) {
		err = au_xino_path(seq, au_sbi(sb)->si_xib);
		seq_putc(seq, '\n');
	}
	return err;
}

static int sysaufs_xi_attr(struct seq_file *seq, struct file *xf,
			   struct kstat *st)
{
	int err;

	err = vfs_getattr(xf->f_vfsmnt, xf->f_dentry, st);
	if (!err)
		seq_printf(seq, "%llux%lu %lld\n",
			   st->blocks, st->blksize, (long long)st->size);
	else
		seq_printf(seq, "err %d\n", err);

	return err;
}

int sysaufs_si_xib(struct seq_file *seq, struct super_block *sb)
{
	int err;
	struct kstat st;

	err = 0;
	if (au_opt_test(au_mntflags(sb), XINO))
		err = sysaufs_xi_attr(seq, au_sbi(sb)->si_xib, &st);
	return err;
}

static int sysaufs_si_xino(struct seq_file *seq, struct super_block *sb,
			   aufs_bindex_t bindex)
{
	int err;
	struct kstat st;
	struct file *xf;

	err = 0;
	if (!au_opt_test(au_mntflags(sb), XINO))
		goto out; /* success */

	AuDbg("b%d\n", bindex);

	xf = au_sbr(sb, bindex)->br_xino.xi_file;
	if (xf) {
		err = vfs_getattr(xf->f_vfsmnt, xf->f_dentry, &st);
		if (!err)
			seq_printf(seq, "%ld, %llux%lu %lld\n",
				   (long)file_count(xf), st.blocks, st.blksize,
				   (long long)st.size);
		else
			seq_printf(seq, "err %d\n", err);
	}

 out:
	return err;
}

#ifdef CONFIG_AUFS_EXPORT
int sysaufs_si_xigen(struct seq_file *seq, struct super_block *sb)
{
	int err;
	struct kstat st;

	err = 0;
	if (au_opt_test(au_mntflags(sb), XINO))
		err = sysaufs_xi_attr(seq, au_sbi(sb)->si_xigen, &st);
	return err;
}
#endif

/*
 * the lifetime of branch is independent from the entry under sysfs.
 * sysfs handles the lifetime of the entry, and never call ->show() after it is
 * unlinked.
 */
static int sysaufs_si_br(struct seq_file *seq, struct super_block *sb,
			 aufs_bindex_t bindex)
{
	struct path path;
	struct dentry *root;
	struct au_branch *br;

	AuDbg("b%d\n", bindex);

	root = sb->s_root;
	di_read_lock_parent(root, !AuLock_IR);
	br = au_sbr(sb, bindex);
	path.mnt = br->br_mnt;
	path.dentry = au_h_dptr(root, bindex);
	au_seq_path(seq, &path);
	di_read_unlock(root, !AuLock_IR);
	seq_printf(seq, "=%s\n", au_optstr_br_perm(br->br_perm));
	return 0;
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

#define SysaufsBr_PREFIX "br"
#define SysaufsXi_PREFIX "xi"

/* todo: file size may exceed PAGE_SIZE */
ssize_t sysaufs_si_show(struct kobject *kobj, struct attribute *attr,
			 char *buf)
{
	ssize_t err;
	long l;
	aufs_bindex_t bend;
	struct au_sbinfo *sbinfo;
	struct super_block *sb;
	struct seq_file *seq;
	char *name;
	struct attribute **cattr;
	static struct {
		const int prefix_len;
		char *prefix;
		int (*func)(struct seq_file *seq, struct super_block *sb,
			    aufs_bindex_t bindex);
	} a[] = {
		{
			.prefix_len	= sizeof(SysaufsBr_PREFIX) - 1,
			.prefix		= SysaufsBr_PREFIX,
			.func		= sysaufs_si_br
		},
		{
			.prefix_len	= sizeof(SysaufsXi_PREFIX) - 1,
			.prefix		= SysaufsXi_PREFIX,
			.func		= sysaufs_si_xino
		},
		{
			.prefix_len	= 0
		}
	}, *p;

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

	p = a;
	bend = au_sbend(sb);
	while (p->prefix_len) {
		if (!strncmp(name, p->prefix, p->prefix_len)) {
			name += p->prefix_len;
			err = strict_strtol(name, 10, &l);
			if (!err) {
				if (l <= bend)
					err = p->func(seq, sb,
						      (aufs_bindex_t)l);
				else
					err = -ENOENT;
			}
			goto out_seq;
		}
		p++;
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
	struct au_xino_file *xi;

	br->br_attr.name = br->br_name;
	br->br_attr.mode = S_IRUGO;
	br->br_attr.owner = THIS_MODULE;

	xi = &br->br_xino;
	xi->xi_attr.name = xi->xi_name;
	xi->xi_attr.mode = S_IRUGO;
	xi->xi_attr.owner = THIS_MODULE;
}

void sysaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex)
{
	struct au_branch *br;
	struct au_xino_file *xi;
	struct kobject *kobj;
	aufs_bindex_t bend, bi;

	kobj = &au_sbi(sb)->si_kobj;
	bend = au_sbend(sb);
	for (bi = bindex; bi <= bend; bi++) {
		br = au_sbr(sb, bi);
		xi = &br->br_xino;
		sysfs_remove_file(kobj, &xi->xi_attr);
	}

	if (sysaufs_brs)
		for (; bindex <= bend; bindex++) {
			br = au_sbr(sb, bindex);
			sysfs_remove_file(kobj, &br->br_attr);
		}
}

static void sysaufs_brs_do_add(struct kobject *kobj, struct attribute *attr,
			       char name[], int nlen, char prefix[],
			       aufs_bindex_t bindex)
{
	int err;

	snprintf(name, nlen, "%s%d", prefix, bindex);
	err = sysfs_create_file(kobj, attr);
	if (unlikely(err))
		AuWarn("failed %s under sysfs(%d)\n", name, err);
}

void sysaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex)
{
	aufs_bindex_t bend, bi;
	struct kobject *kobj;
	struct au_branch *br;
	struct au_xino_file *xi;

	kobj = &au_sbi(sb)->si_kobj;
	bend = au_sbend(sb);
	for (bi = bindex; bi <= bend; bi++) {
		br = au_sbr(sb, bi);
		xi = &br->br_xino;
		/* todo: create link for shared xino */
		sysaufs_brs_do_add(kobj, &xi->xi_attr, xi->xi_name,
				   sizeof(xi->xi_name), SysaufsXi_PREFIX, bi);
	}

	if (sysaufs_brs)
		for (; bindex <= bend; bindex++) {
			br = au_sbr(sb, bindex);
			sysaufs_brs_do_add(kobj, &br->br_attr, br->br_name,
					   sizeof(br->br_name),
					   SysaufsBr_PREFIX, bindex);
		}
}

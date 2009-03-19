/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * sysfs interface and mount lifetime management
 */

#ifndef __SYSAUFS_H__
#define __SYSAUFS_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/aufs_type.h>

struct sysaufs_si_attr {
	struct attribute attr;
	int (*show)(struct seq_file *seq, struct super_block *sb);
};

/* ---------------------------------------------------------------------- */

/* sysaufs.c */
extern unsigned long sysaufs_si_mask;
extern struct kset *sysaufs_ket;
extern struct attribute *sysaufs_si_attrs[];
int sysaufs_si_init(struct au_sbinfo *sbinfo);
int __init sysaufs_init(void);
void sysaufs_fin(void);

/* ---------------------------------------------------------------------- */

/* some people doesn't like to show a pointer in kernel */
static inline unsigned long sysaufs_si_id(struct au_sbinfo *sbinfo)
{
	return sysaufs_si_mask ^ (unsigned long)sbinfo;
}

struct au_branch;
#ifdef CONFIG_SYSFS
/* sysfs.c */
extern struct attribute_group *sysaufs_attr_group;

int sysaufs_si_xino(struct seq_file *seq, struct super_block *sb);
#ifdef CONFIG_AUFS_EXPORT
int sysaufs_si_xigen(struct seq_file *seq, struct super_block *sb);
#endif
ssize_t sysaufs_si_show(struct kobject *kobj, struct attribute *attr,
			 char *buf);

void sysaufs_br_init(struct au_branch *br);
void sysaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex);
void sysaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex);

#define sysaufs_brs_init()	do {} while (0)

#else
#define sysaufs_attr_group	NULL

static inline
int sysaufs_si_xino(struct seq_file *seq, struct super_block *sb)
{
	return 0;
}

#ifdef CONFIG_AUFS_EXPORT
static inline
int sysaufs_si_xigen(struct seq_file *seq, struct super_block *sb)
{
	return 0;
}
#endif

static inline
ssize_t sysaufs_si_show(struct kobject *kobj, struct attribute *attr,
			 char *buf)
{
	return 0;
}

static inline void sysaufs_br_init(struct au_branch *br)
{
	/* empty */
}

static inline void sysaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex)
{
	/* nothing */
}

static inline void sysaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex)
{
	/* nothing */
}

static inline void sysaufs_brs_init(void)
{
	sysaufs_brs = 0;
}

#endif /* CONFIG_SYSFS */

#endif /* __KERNEL__ */
#endif /* __SYSAUFS_H__ */

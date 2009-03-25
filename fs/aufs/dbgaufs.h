/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * debugfs interface
 */

#ifndef __DBGAUFS_H__
#define __DBGAUFS_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/aufs_type.h>

struct au_sbinfo;
#ifdef CONFIG_DEBUG_FS
/* dbgaufs.c */
void dbgaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex);
void dbgaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex);
void dbgaufs_si_fin(struct au_sbinfo *sbinfo);
int dbgaufs_si_init(struct au_sbinfo *sbinfo);
void dbgaufs_fin(void);
int __init dbgaufs_init(void);

#else

static inline
void dbgaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex)
{
	/* empty */
}

static inline
void dbgaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex)
{
	/* empty */
}

static inline
void dbgaufs_si_fin(struct au_sbinfo *sbinfo)
{
	/* empty */
}

static inline
int dbgaufs_si_init(struct au_sbinfo *sbinfo)
{
	return 0;
}

#define dbgaufs_fin()	do {} while (0)

static inline
int __init dbgaufs_init(void)
{
	return 0;
}
#endif /* CONFIG_DEBUG_FS */

#endif /* __KERNEL__ */
#endif /* __DBGAUFS_H__ */

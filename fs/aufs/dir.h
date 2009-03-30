/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * directory operations
 */

#ifndef __AUFS_DIR_H__
#define __AUFS_DIR_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/aufs_type.h>

/* ---------------------------------------------------------------------- */

/* need to be faster and smaller */

#define AuSize_DEBLK	512
#define AuSize_NHASH	32

typedef char au_vdir_deblk_t[AuSize_DEBLK];

struct au_nhash {
	struct hlist_head heads[AuSize_NHASH];
};

struct au_vdir_destr {
	unsigned char	len;
	char		name[0];
} __packed;

struct au_vdir_dehstr {
	struct hlist_node	hash;
	struct au_vdir_destr	*str;
};

struct au_vdir_de {
	ino_t			de_ino;
	unsigned char		de_type;
	/* caution: packed */
	struct au_vdir_destr	de_str;
} __packed;

struct au_vdir_wh {
	struct hlist_node	wh_hash;
	aufs_bindex_t		wh_bindex;
	struct au_vdir_destr	wh_str;
} __packed;

union au_vdir_deblk_p {
	unsigned char		*p;
	au_vdir_deblk_t		*deblk;
	struct au_vdir_de	*de;
};

struct au_vdir {
	au_vdir_deblk_t	**vd_deblk;
	int		vd_nblk;
	struct {
		int			i;
		union au_vdir_deblk_p	p;
	} vd_last;

	unsigned long	vd_version;
	unsigned long	vd_jiffy;
};

/* ---------------------------------------------------------------------- */

/* dir.c */
extern const struct file_operations aufs_dir_fop;
void au_add_nlink(struct inode *dir, struct inode *h_dir);
void au_sub_nlink(struct inode *dir, struct inode *h_dir);
int au_test_empty_lower(struct dentry *dentry);
int au_test_empty(struct dentry *dentry, struct au_nhash *whlist);

/* vdir.c */
struct au_nhash *au_nhash_new(gfp_t gfp);
void au_nhash_del(struct au_nhash *nhash);
void au_nhash_init(struct au_nhash *nhash);
void au_nhash_move(struct au_nhash *dst, struct au_nhash *src);
void au_nhash_fin(struct au_nhash *nhash);
int au_nhash_test_longer_wh(struct au_nhash *whlist, aufs_bindex_t btgt,
			    int limit);
int au_nhash_test_known_wh(struct au_nhash *whlist, char *name, int namelen);
int au_nhash_append_wh(struct au_nhash *whlist, char *name, int namelen,
		       aufs_bindex_t bindex);
void au_vdir_free(struct au_vdir *vdir);
int au_vdir_init(struct file *file);
int au_vdir_fill_de(struct file *file, void *dirent, filldir_t filldir);

/* ioctl.c */
long aufs_ioctl_dir(struct file *file, unsigned int cmd, unsigned long arg);

#endif /* __KERNEL__ */
#endif /* __AUFS_DIR_H__ */

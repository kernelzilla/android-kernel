/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * file operations
 */

#ifndef __AUFS_FILE_H__
#define __AUFS_FILE_H__

#ifdef __KERNEL__

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/aufs_type.h>
#include "rwsem.h"

struct au_branch;
struct au_hfile {
	struct file		*hf_file;
	struct au_branch	*hf_br;
};

struct au_vdir;
struct au_finfo {
	atomic_t		fi_generation;

	struct rw_semaphore	fi_rwsem;
	struct au_hfile		*fi_hfile;
	aufs_bindex_t		fi_bstart, fi_bend;

	union {
		/* non-dir only */
		struct vm_operations_struct	*fi_h_vm_ops;

		/* dir only */
		struct {
			struct au_vdir		*fi_vdir_cache;
			int			fi_maintain_plink;
		};
	};
};

/* ---------------------------------------------------------------------- */

/* file.c */
extern struct address_space_operations aufs_aop;
void au_store_oflag(struct nameidata *nd);
unsigned int au_file_roflags(unsigned int flags);
struct file *au_h_open(struct dentry *dentry, aufs_bindex_t bindex, int flags,
		       struct file *file);
int au_do_open(struct file *file, int (*open)(struct file *file, int flags));
int au_reopen_nondir(struct file *file);
struct au_pin;
int au_ready_to_write(struct file *file, loff_t len, struct au_pin *pin);
int au_reval_and_lock_fdi(struct file *file, int (*reopen)(struct file *file),
			  int wlock);

/* f_op.c */
extern struct file_operations aufs_file_fop;
int aufs_flush(struct file *file, fl_owner_t id);

/* finfo.c */
void au_hfput(struct au_hfile *hf, struct file *file);
void au_set_h_fptr(struct file *file, aufs_bindex_t bindex,
		   struct file *h_file);

void au_update_figen(struct file *file);

void au_finfo_fin(struct file *file);
int au_finfo_init(struct file *file);
int au_fi_realloc(struct au_finfo *finfo, int nbr);

/* ---------------------------------------------------------------------- */

static inline struct au_finfo *au_fi(struct file *file)
{
	return file->private_data;
}

/* ---------------------------------------------------------------------- */

/*
 * fi_read_lock, fi_write_lock,
 * fi_read_unlock, fi_write_unlock, fi_downgrade_lock
 */
AuSimpleRwsemFuncs(fi, struct file *f, &au_fi(f)->fi_rwsem);

#define FiMustNoWaiters(f)	AuRwMustNoWaiters(&au_fi(f)->fi_rwsem)

/* ---------------------------------------------------------------------- */

/* todo: hard/soft set? */
static inline aufs_bindex_t au_fbstart(struct file *file)
{
	return au_fi(file)->fi_bstart;
}

static inline aufs_bindex_t au_fbend(struct file *file)
{
	return au_fi(file)->fi_bend;
}

static inline struct au_vdir *au_fvdir_cache(struct file *file)
{
	return au_fi(file)->fi_vdir_cache;
}

static inline void au_set_fbstart(struct file *file, aufs_bindex_t bindex)
{
	au_fi(file)->fi_bstart = bindex;
}

static inline void au_set_fbend(struct file *file, aufs_bindex_t bindex)
{
	au_fi(file)->fi_bend = bindex;
}

static inline void au_set_fvdir_cache(struct file *file,
				      struct au_vdir *vdir_cache)
{
	au_fi(file)->fi_vdir_cache = vdir_cache;
}

static inline struct file *au_h_fptr(struct file *file, aufs_bindex_t bindex)
{
	return au_fi(file)->fi_hfile[0 + bindex].hf_file;
}

/* todo: memory barrier? */
static inline unsigned int au_figen(struct file *f)
{
	return atomic_read(&au_fi(f)->fi_generation);
}

static inline int au_test_mmapped(struct file *f)
{
	return !!(au_fi(f)->fi_h_vm_ops);
}

#endif /* __KERNEL__ */
#endif /* __AUFS_FILE_H__ */

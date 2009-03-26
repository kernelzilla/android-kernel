/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * file private data
 */

#include "aufs.h"

void au_hfput(struct au_hfile *hf, struct file *file)
{
	if (file->f_mode & FMODE_EXEC)
		allow_write_access(hf->hf_file);
	fput(hf->hf_file);
	hf->hf_file = NULL;
	atomic_dec(&hf->hf_br->br_count);
	hf->hf_br = NULL;
}

void au_set_h_fptr(struct file *file, aufs_bindex_t bindex, struct file *val)
{
	struct au_finfo *finfo = au_fi(file);
	struct au_hfile *hf;

	hf = finfo->fi_hfile + bindex;
	if (hf->hf_file)
		au_hfput(hf, file);
	if (val) {
		hf->hf_file = val;
		hf->hf_br = au_sbr(file->f_dentry->d_sb, bindex);
	}
}

void au_update_figen(struct file *file)
{
	atomic_set(&au_fi(file)->fi_generation, au_digen(file->f_dentry));
	/* smp_mb(); */ /* atomic_set */
}

/* ---------------------------------------------------------------------- */

void au_finfo_fin(struct file *file)
{
	struct au_finfo *finfo;
	aufs_bindex_t bindex, bend;

	fi_write_lock(file);
	bend = au_fbend(file);
	bindex = au_fbstart(file);
	if (bindex >= 0)
		/*
		 * calls fput() instead of filp_close(),
		 * since no dnotify or lock for the lower file.
		 */
		for (; bindex <= bend; bindex++)
			au_set_h_fptr(file, bindex, NULL);

	finfo = au_fi(file);
	au_dbg_verify_hf(finfo);
	kfree(finfo->fi_hfile);
	fi_write_unlock(file);
	au_rwsem_destroy(&finfo->fi_rwsem);
	au_cache_free_finfo(finfo);
}

int au_finfo_init(struct file *file)
{
	struct au_finfo *finfo;
	struct dentry *dentry;
	union {
		unsigned int u;
		fmode_t m;
	} u;

	BUILD_BUG_ON(sizeof(u.m) != sizeof(u.u));

	dentry = file->f_dentry;
	finfo = au_cache_alloc_finfo();
	if (unlikely(!finfo))
		goto out;

	finfo->fi_hfile = kcalloc(au_sbend(dentry->d_sb) + 1,
				  sizeof(*finfo->fi_hfile), GFP_NOFS);
	if (unlikely(!finfo->fi_hfile))
		goto out_finfo;

	init_rwsem(&finfo->fi_rwsem);
	down_write(&finfo->fi_rwsem);
	finfo->fi_bstart = -1;
	finfo->fi_bend = -1;
	atomic_set(&finfo->fi_generation, au_digen(dentry));
	/* smp_mb(); */ /* atomic_set */

	/* cf. au_store_oflag() */
	u.u = (unsigned int)file->private_data;
	file->f_mode |= (u.m & FMODE_EXEC);
	file->private_data = finfo;
	return 0; /* success */

 out_finfo:
	au_cache_free_finfo(finfo);
 out:
	return -ENOMEM;
}

int au_fi_realloc(struct au_finfo *finfo, int nbr)
{
	int err, sz;
	struct au_hfile *hfp;

	err = -ENOMEM;
	sz = sizeof(*hfp) * (finfo->fi_bend + 1);
	if (!sz)
		sz = sizeof(*hfp);
	hfp = au_kzrealloc(finfo->fi_hfile, sz, sizeof(*hfp) * nbr, GFP_NOFS);
	if (hfp) {
		finfo->fi_hfile = hfp;
		err = 0;
	}

	return err;
}

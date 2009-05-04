/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * virtual or vertical directory
 */

#include "aufs.h"

static unsigned int calc_size(int nlen)
{
	BUILD_BUG_ON(sizeof(ino_t) != sizeof(long));
	return ALIGN(sizeof(struct au_vdir_de) + nlen, sizeof(ino_t));
}

static int set_deblk_end(union au_vdir_deblk_p *p,
			 union au_vdir_deblk_p *deblk_end)
{
	if (calc_size(0) <= deblk_end->deblk - p->deblk) {
		p->de->de_str.len = 0;
		/* smp_mb(); */
		return 0;
	}
	return -1; /* error */
}

/* returns true or false */
static int is_deblk_end(union au_vdir_deblk_p *p,
			union au_vdir_deblk_p *deblk_end)
{
	if (calc_size(0) <= deblk_end->deblk - p->deblk)
		return !p->de->de_str.len;
	return 1;
}

static unsigned char *last_deblk(struct au_vdir *vdir)
{
	return vdir->vd_deblk[vdir->vd_nblk - 1];
}

/* ---------------------------------------------------------------------- */

/*
 * the allocated memory has to be freed by
 * au_nhash_wh_free() or void au_nhash_de_free().
 */
struct au_nhash *au_nhash_alloc(struct super_block *sb, aufs_bindex_t bend,
				gfp_t gfp)
{
	struct au_nhash *nhash;
	struct hlist_head *head;
	unsigned int u, n;
	aufs_bindex_t bindex;

	nhash = kmalloc(sizeof(*nhash) * (bend + 1), gfp);
	if (unlikely(!nhash))
		goto out;

	n = au_sbi(sb)->si_rdhash;
	for (bindex = 0; bindex <= bend; bindex++) {
		head = kmalloc(sizeof(*nhash->nh_head) * n, gfp);
		if (unlikely(!head))
			goto out_free;
		nhash[bindex].nh_num = n;
		nhash[bindex].nh_head = head;
		for (u = 0; u < n; u++)
			INIT_HLIST_HEAD(head++);
	}
	return nhash; /* success */

 out_free:
	for (bindex--; bindex >= 0; bindex--)
		kfree(nhash[bindex].nh_head);
	kfree(nhash);
 out:
	return ERR_PTR(-ENOMEM);
}

static void au_nhash_wh_do_free(struct au_nhash *whlist)
{
	unsigned int u, n;
	struct hlist_head *head;
	struct au_vdir_wh *tpos;
	struct hlist_node *pos, *node;

	n = whlist->nh_num;
	head = whlist->nh_head;
	for (u = 0; u < n; u++) {
		hlist_for_each_entry_safe(tpos, pos, node, head, wh_hash) {
			/* hlist_del(pos); */
			kfree(tpos);
		}
		head++;
	}
}

void au_nhash_wh_free(struct au_nhash *whlist, aufs_bindex_t bend)
{
	aufs_bindex_t bindex;

	for (bindex = 0; bindex <= bend; bindex++) {
		au_nhash_wh_do_free(whlist + bindex);
		kfree(whlist[bindex].nh_head);
	}

	kfree(whlist);
}

static void au_nhash_de_do_free(struct au_nhash *delist)
{
	unsigned int u, n;
	struct hlist_head *head;
	struct au_vdir_dehstr *tpos;
	struct hlist_node *pos, *node;

	n = delist->nh_num;
	head = delist->nh_head;
	for (u = 0; u < n; u++) {
		hlist_for_each_entry_safe(tpos, pos, node, head, hash) {
			/* hlist_del(pos); */
			au_cache_free_dehstr(tpos);
		}
		head++;
	}
}

static void au_nhash_de_free(struct au_nhash *delist, aufs_bindex_t bend)
{
	aufs_bindex_t bindex;

	for (bindex = 0; bindex <= bend; bindex++) {
		au_nhash_de_do_free(delist + bindex);
		kfree(delist[bindex].nh_head);
	}

	kfree(delist);
}

/* ---------------------------------------------------------------------- */

int au_nhash_test_longer_wh(struct au_nhash *whlist, aufs_bindex_t btgt,
			    int limit)
{
	int num;
	unsigned int u, n;
	struct hlist_head *head;
	struct au_vdir_wh *tpos;
	struct hlist_node *pos;

	num = 0;
	n = whlist->nh_num;
	head = whlist->nh_head;
	for (u = 0; u < n; u++) {
		hlist_for_each_entry(tpos, pos, head, wh_hash)
			if (tpos->wh_bindex == btgt && ++num > limit)
				return 1;
		head++;
	}
	return 0;
}

static struct hlist_head *au_name_hash(struct au_nhash *nhash,
				       const unsigned char *name,
				       unsigned int len)
{
	return nhash->nh_head + full_name_hash(name, len) % nhash->nh_num;
}

/* returns found or not */
int au_nhash_test_known_wh(struct au_nhash *whlist, char *name, int nlen)
{
	struct hlist_head *head;
	struct au_vdir_wh *tpos;
	struct hlist_node *pos;
	struct au_vdir_destr *str;

	head = au_name_hash(whlist, name, nlen);
	hlist_for_each_entry(tpos, pos, head, wh_hash) {
		str = &tpos->wh_str;
		AuDbg("%.*s\n", str->len, str->name);
		if (str->len == nlen && !memcmp(str->name, name, nlen))
			return 1;
	}
	return 0;
}

int au_nhash_append_wh(struct au_nhash *whlist, char *name, int nlen,
		       aufs_bindex_t bindex)
{
	int err;
	struct au_vdir_destr *str;
	struct au_vdir_wh *wh;

	err = -ENOMEM;
	wh = kmalloc(sizeof(*wh) + nlen, GFP_NOFS);
	if (unlikely(!wh))
		goto out;

	err = 0;
	wh->wh_bindex = bindex;
	str = &wh->wh_str;
	str->len = nlen;
	memcpy(str->name, name, nlen);
	hlist_add_head(&wh->wh_hash, au_name_hash(whlist, name, nlen));
	/* smp_mb(); */

 out:
	return err;
}

/* ---------------------------------------------------------------------- */

void au_vdir_free(struct au_vdir *vdir)
{
	unsigned char **deblk;

	deblk = vdir->vd_deblk;
	while (vdir->vd_nblk--)
		kfree(*deblk++);
	kfree(vdir->vd_deblk);
	au_cache_free_vdir(vdir);
}

static int append_deblk(struct au_vdir *vdir)
{
	int err;
	unsigned long sz, ul;
	const unsigned int deblk_sz = vdir->vd_deblk_sz;
	union au_vdir_deblk_p p, deblk_end;
	unsigned char **o;

	err = -ENOMEM;
	sz = sizeof(*o) * vdir->vd_nblk;
	o = au_kzrealloc(vdir->vd_deblk, sz, sz + sizeof(*o), GFP_NOFS);
	if (unlikely(!o))
		goto out;

	vdir->vd_deblk = o;
	p.deblk = kmalloc(deblk_sz, GFP_NOFS);
	if (p.deblk) {
		ul = vdir->vd_nblk++;
		vdir->vd_deblk[ul] = p.deblk;
		vdir->vd_last.ul = ul;
		vdir->vd_last.p.deblk = p.deblk;
		deblk_end.deblk = p.deblk + deblk_sz;
		err = set_deblk_end(&p, &deblk_end);
	}

 out:
	return err;
}

static struct au_vdir *alloc_vdir(struct super_block *sb)
{
	struct au_vdir *vdir;
	int err;

	err = -ENOMEM;
	vdir = au_cache_alloc_vdir();
	if (unlikely(!vdir))
		goto out;

	vdir->vd_deblk = kzalloc(sizeof(*vdir->vd_deblk), GFP_NOFS);
	if (unlikely(!vdir->vd_deblk))
		goto out_free;

	vdir->vd_deblk_sz = au_sbi(sb)->si_rdblk;
	vdir->vd_nblk = 0;
	vdir->vd_version = 0;
	vdir->vd_jiffy = 0;
	err = append_deblk(vdir);
	if (!err)
		return vdir; /* success */

	kfree(vdir->vd_deblk);

 out_free:
	au_cache_free_vdir(vdir);
 out:
	vdir = ERR_PTR(err);
	return vdir;
}

static int reinit_vdir(struct au_vdir *vdir)
{
	int err;
	union au_vdir_deblk_p p, deblk_end;

	while (vdir->vd_nblk > 1) {
		kfree(vdir->vd_deblk[vdir->vd_nblk - 1]);
		/* vdir->vd_deblk[vdir->vd_nblk - 1] = NULL; */
		vdir->vd_nblk--;
	}
	p.deblk = vdir->vd_deblk[0];
	deblk_end.deblk = p.deblk + vdir->vd_deblk_sz;
	err = set_deblk_end(&p, &deblk_end);
	/* keep vd_dblk_sz */
	vdir->vd_last.ul = 0;
	vdir->vd_last.p.deblk = vdir->vd_deblk[0];
	vdir->vd_version = 0;
	vdir->vd_jiffy = 0;
	/* smp_mb(); */
	return err;
}

/* ---------------------------------------------------------------------- */

/* returns found(true) or not */
static int test_known(struct au_nhash *delist, char *name, int nlen)
{
	struct hlist_head *head;
	struct au_vdir_dehstr *tpos;
	struct hlist_node *pos;
	struct au_vdir_destr *str;

	head = au_name_hash(delist, name, nlen);
	hlist_for_each_entry(tpos, pos, head, hash) {
		str = tpos->str;
		AuDbg("%.*s\n", str->len, str->name);
		if (str->len == nlen && !memcmp(str->name, name, nlen))
			return 1;
	}
	return 0;

}

static int append_de(struct au_vdir *vdir, char *name, int nlen, ino_t ino,
		     unsigned int d_type, struct au_nhash *delist)
{
	int err;
	unsigned int sz;
	const unsigned int deblk_sz = vdir->vd_deblk_sz;
	union au_vdir_deblk_p p, *room, deblk_end;
	struct au_vdir_dehstr *dehstr;

	p.deblk = last_deblk(vdir);
	deblk_end.deblk = p.deblk + deblk_sz;
	room = &vdir->vd_last.p;
	AuDebugOn(room->deblk < p.deblk || deblk_end.deblk <= room->deblk
		  || !is_deblk_end(room, &deblk_end));

	sz = calc_size(nlen);
	if (unlikely(sz > deblk_end.deblk - room->deblk)) {
		err = append_deblk(vdir);
		if (unlikely(err))
			goto out;

		p.deblk = last_deblk(vdir);
		deblk_end.deblk = p.deblk + deblk_sz;
		/* smp_mb(); */
		AuDebugOn(room->deblk != p.deblk);
	}

	err = -ENOMEM;
	dehstr = au_cache_alloc_dehstr();
	if (unlikely(!dehstr))
		goto out;

	dehstr->str = &room->de->de_str;
	hlist_add_head(&dehstr->hash, au_name_hash(delist, name, nlen));
	room->de->de_ino = ino;
	room->de->de_type = d_type;
	room->de->de_str.len = nlen;
	memcpy(room->de->de_str.name, name, nlen);

	err = 0;
	room->deblk += sz;
	if (unlikely(set_deblk_end(room, &deblk_end)))
		err = append_deblk(vdir);
	/* smp_mb(); */

 out:
	return err;
}

/* ---------------------------------------------------------------------- */

static int au_ino(struct super_block *sb, aufs_bindex_t bindex, ino_t h_ino,
		  unsigned int d_type, ino_t *ino)
{
	int err;
	struct mutex *mtx;
	const int isdir = (d_type == DT_DIR);

	/* prevent hardlinks from race condition */
	mtx = NULL;
	if (!isdir) {
		mtx = &au_sbr(sb, bindex)->br_xino.xi_nondir_mtx;
		mutex_lock(mtx);
	}
	err = au_xino_read(sb, bindex, h_ino, ino);
	if (unlikely(err))
		goto out;

	if (!*ino) {
		err = -EIO;
		*ino = au_xino_new_ino(sb);
		if (unlikely(!*ino))
			goto out;
		err = au_xino_write(sb, bindex, h_ino, *ino);
		if (unlikely(err))
			goto out;
	}

 out:
	if (!isdir)
		mutex_unlock(mtx);
	return err;
}

#define AuFillVdir_CALLED	1
#define AuFillVdir_WHABLE	(1 << 1)
#define au_ftest_fillvdir(flags, name)	((flags) & AuFillVdir_##name)
#define au_fset_fillvdir(flags, name)	{ (flags) |= AuFillVdir_##name; }
#define au_fclr_fillvdir(flags, name)	{ (flags) &= ~AuFillVdir_##name; }

struct fillvdir_arg {
	struct file		*file;
	struct au_vdir		*vdir;
	struct au_nhash		*delist;
	struct au_nhash		*whlist;
	aufs_bindex_t		bindex;
	unsigned int		flags;
	int			err;
};

static int fillvdir(void *__arg, const char *__name, int nlen,
		    loff_t offset __maybe_unused, u64 h_ino,
		    unsigned int d_type)
{
	struct fillvdir_arg *arg = __arg;
	char *name = (void *)__name;
	struct super_block *sb;
	struct au_nhash *delist, *whlist;
	ino_t ino;
	aufs_bindex_t bindex, bend;

	bend = arg->bindex;
	arg->err = 0;
	au_fset_fillvdir(arg->flags, CALLED);
	/* smp_mb(); */
	if (nlen <= AUFS_WH_PFX_LEN
	    || memcmp(name, AUFS_WH_PFX, AUFS_WH_PFX_LEN)) {
		delist = arg->delist;
		whlist = arg->whlist;
		for (bindex = 0; bindex < bend; bindex++)
			if (test_known(delist++, name, nlen)
			    || au_nhash_test_known_wh(whlist + bindex, name,
						      nlen))
				goto out; /* already exists or whiteouted */

		sb = arg->file->f_dentry->d_sb;
		arg->err = au_ino(sb, bend, h_ino, d_type, &ino);
		if (!arg->err)
			arg->err = append_de(arg->vdir, name, nlen, ino,
					     d_type, arg->delist + bend);
	} else if (au_ftest_fillvdir(arg->flags, WHABLE)) {
		name += AUFS_WH_PFX_LEN;
		nlen -= AUFS_WH_PFX_LEN;
		whlist = arg->whlist;
		for (bindex = 0; bindex < bend; bindex++)
			if (au_nhash_test_known_wh(whlist++, name, nlen))
				goto out; /* already whiteouted */

		if (!arg->err)
			arg->err = au_nhash_append_wh
				(arg->whlist + bend, name, nlen, bend);
	}

 out:
	if (!arg->err)
		arg->vdir->vd_jiffy = jiffies;
	/* smp_mb(); */
	AuTraceErr(arg->err);
	return arg->err;
}

static int au_do_read_vdir(struct fillvdir_arg *arg)
{
	int err;
	loff_t offset;
	aufs_bindex_t bend, bindex;
	struct file *hf, *file;
	struct super_block *sb;

	err = -ENOMEM;
	file = arg->file;
	sb = file->f_dentry->d_sb;
	bend = au_fbend(file);
	arg->delist = au_nhash_alloc(sb, bend, GFP_NOFS);
	if (unlikely(!arg->delist))
		goto out;
	arg->whlist = au_nhash_alloc(sb, bend, GFP_NOFS);
	if (unlikely(!arg->whlist))
		goto out_delist;

	err = 0;
	arg->flags = 0;
	for (bindex = au_fbstart(file); !err && bindex <= bend; bindex++) {
		hf = au_h_fptr(file, bindex);
		if (!hf)
			continue;

		offset = vfsub_llseek(hf, 0, SEEK_SET);
		err = offset;
		if (unlikely(offset))
			break;

		arg->bindex = bindex;
		au_fclr_fillvdir(arg->flags, WHABLE);
		if (bindex != bend
		    && au_br_whable(au_sbr_perm(sb, bindex)))
			au_fset_fillvdir(arg->flags, WHABLE);
		do {
			arg->err = 0;
			au_fclr_fillvdir(arg->flags, CALLED);
			/* smp_mb(); */
			err = vfsub_readdir(hf, fillvdir, arg);
			if (err >= 0)
				err = arg->err;
		} while (!err && au_ftest_fillvdir(arg->flags, CALLED));
	}
	au_nhash_wh_free(arg->whlist, bend);

 out_delist:
	au_nhash_de_free(arg->delist, bend);
 out:
	return err;
}

static int read_vdir(struct file *file, int may_read)
{
	int err;
	unsigned long expire;
	unsigned char do_read;
	struct fillvdir_arg arg;
	struct inode *inode;
	struct au_vdir *vdir, *allocated;

	err = 0;
	inode = file->f_dentry->d_inode;
	IMustLock(inode);
	allocated = NULL;
	do_read = 0;
	expire = au_sbi(inode->i_sb)->si_rdcache;
	vdir = au_ivdir(inode);
	if (!vdir) {
		do_read = 1;
		vdir = alloc_vdir(inode->i_sb);
		err = PTR_ERR(vdir);
		if (IS_ERR(vdir))
			goto out;
		err = 0;
		allocated = vdir;
	} else if (may_read
		   && (inode->i_version != vdir->vd_version
		       || time_after(jiffies, vdir->vd_jiffy + expire))) {
		do_read = 1;
		err = reinit_vdir(vdir);
		if (unlikely(err))
			goto out;
	}

	if (!do_read)
		return 0; /* success */

	arg.file = file;
	arg.vdir = vdir;
	err = au_do_read_vdir(&arg);
	if (!err) {
		/* file->f_pos = 0; */
		vdir->vd_version = inode->i_version;
		vdir->vd_last.ul = 0;
		vdir->vd_last.p.deblk = vdir->vd_deblk[0];
		if (allocated)
			au_set_ivdir(inode, allocated);
	} else if (allocated)
		au_vdir_free(allocated);

 out:
	return err;
}

static int copy_vdir(struct au_vdir *tgt, struct au_vdir *src)
{
	int err, rerr;
	unsigned long ul, n;
	const unsigned int deblk_sz = src->vd_deblk_sz;

	AuDebugOn(tgt->vd_nblk != 1);

	err = -ENOMEM;
	if (tgt->vd_nblk < src->vd_nblk) {
		unsigned char **p;

		p = au_kzrealloc(tgt->vd_deblk, sizeof(*p) * tgt->vd_nblk,
				 sizeof(*p) * src->vd_nblk, GFP_NOFS);
		if (unlikely(!p))
			goto out;
		tgt->vd_deblk = p;
	}

	tgt->vd_nblk = src->vd_nblk;
	tgt->vd_deblk_sz = deblk_sz;
	memcpy(tgt->vd_deblk[0], src->vd_deblk[0], deblk_sz);
	/* tgt->vd_last.i = 0; */
	/* tgt->vd_last.p.deblk = tgt->vd_deblk[0]; */
	tgt->vd_version = src->vd_version;
	tgt->vd_jiffy = src->vd_jiffy;

	n = src->vd_nblk;
	for (ul = 1; ul < n; ul++) {
		tgt->vd_deblk[ul] = kmalloc(deblk_sz, GFP_NOFS);
		if (tgt->vd_deblk[ul])
			memcpy(tgt->vd_deblk[ul], src->vd_deblk[ul], deblk_sz);
		else
			goto out;
	}
	/* smp_mb(); */
	return 0; /* success */

 out:
	rerr = reinit_vdir(tgt);
	BUG_ON(rerr);
	return err;
}

int au_vdir_init(struct file *file)
{
	int err;
	struct inode *inode;
	struct au_vdir *vdir_cache, *allocated;

	err = read_vdir(file, !file->f_pos);
	if (unlikely(err))
		goto out;

	allocated = NULL;
	vdir_cache = au_fvdir_cache(file);
	if (!vdir_cache) {
		vdir_cache = alloc_vdir(file->f_dentry->d_sb);
		err = PTR_ERR(vdir_cache);
		if (IS_ERR(vdir_cache))
			goto out;
		allocated = vdir_cache;
	} else if (!file->f_pos && vdir_cache->vd_version != file->f_version) {
		err = reinit_vdir(vdir_cache);
		if (unlikely(err))
			goto out;
	} else
		return 0; /* success */

	inode = file->f_dentry->d_inode;
	err = copy_vdir(vdir_cache, au_ivdir(inode));
	if (!err) {
		file->f_version = inode->i_version;
		if (allocated)
			au_set_fvdir_cache(file, allocated);
	} else if (allocated)
		au_vdir_free(allocated);

 out:
	return err;
}

static loff_t calc_offset(struct au_vdir *vdir)
{
	loff_t offset;
	union au_vdir_deblk_p p;

	p.deblk = vdir->vd_deblk[vdir->vd_last.ul];
	offset = vdir->vd_last.p.deblk - p.deblk;
	offset += vdir->vd_deblk_sz * vdir->vd_last.ul;
	return offset;
}

/* returns true or false */
static int seek_vdir(struct file *file)
{
	int valid;
	unsigned int deblk_sz;
	unsigned long ul, n;
	loff_t offset;
	union au_vdir_deblk_p p, deblk_end;
	struct au_vdir *vdir_cache;

	valid = 1;
	vdir_cache = au_fvdir_cache(file);
	offset = calc_offset(vdir_cache);
	AuDbg("offset %lld\n", offset);
	if (file->f_pos == offset)
		goto out;

	vdir_cache->vd_last.ul = 0;
	vdir_cache->vd_last.p.deblk = vdir_cache->vd_deblk[0];
	if (!file->f_pos)
		goto out;

	valid = 0;
	deblk_sz = vdir_cache->vd_deblk_sz;
	ul = div64_u64(file->f_pos, deblk_sz);
	AuDbg("ul %lu\n", ul);
	if (ul >= vdir_cache->vd_nblk)
		goto out;

	n = vdir_cache->vd_nblk;
	for (; ul < n; ul++) {
		p.deblk = vdir_cache->vd_deblk[ul];
		deblk_end.deblk = p.deblk + deblk_sz;
		offset = ul;
		offset *= deblk_sz;
		while (!is_deblk_end(&p, &deblk_end) && offset < file->f_pos) {
			unsigned int l;

			l = calc_size(p.de->de_str.len);
			offset += l;
			p.deblk += l;
		}
		if (!is_deblk_end(&p, &deblk_end)) {
			valid = 1;
			vdir_cache->vd_last.ul = ul;
			vdir_cache->vd_last.p = p;
			break;
		}
	}

 out:
	/* smp_mb(); */
	AuTraceErr(!valid);
	return valid;
}

int au_vdir_fill_de(struct file *file, void *dirent, filldir_t filldir)
{
	int err;
	unsigned int l, deblk_sz;
	union au_vdir_deblk_p deblk_end;
	struct au_vdir *vdir_cache;
	struct au_vdir_de *de;

	vdir_cache = au_fvdir_cache(file);
	if (!seek_vdir(file))
		return 0;

	deblk_sz = vdir_cache->vd_deblk_sz;
	while (1) {
		deblk_end.deblk = vdir_cache->vd_deblk[vdir_cache->vd_last.ul];
		deblk_end.deblk += deblk_sz;
		while (!is_deblk_end(&vdir_cache->vd_last.p, &deblk_end)) {
			de = vdir_cache->vd_last.p.de;
			AuDbg("%.*s, off%lld, i%lu, dt%d\n",
			      de->de_str.len, de->de_str.name, file->f_pos,
			      (unsigned long)de->de_ino, de->de_type);
			err = filldir(dirent, de->de_str.name, de->de_str.len,
				      file->f_pos, de->de_ino, de->de_type);
			if (unlikely(err)) {
				AuTraceErr(err);
				/* todo: ignore the error caused by udba? */
				/* return err; */
				return 0;
			}

			l = calc_size(de->de_str.len);
			vdir_cache->vd_last.p.deblk += l;
			file->f_pos += l;
		}
		if (vdir_cache->vd_last.ul < vdir_cache->vd_nblk - 1) {
			vdir_cache->vd_last.ul++;
			vdir_cache->vd_last.p.deblk
				= vdir_cache->vd_deblk[vdir_cache->vd_last.ul];
			file->f_pos = deblk_sz * vdir_cache->vd_last.ul;
			continue;
		}
		break;
	}

	/* smp_mb(); */
	return 0;
}

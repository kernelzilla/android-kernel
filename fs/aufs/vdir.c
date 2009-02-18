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
 * virtual or vertical directory
 */

#include "aufs.h"

static int calc_size(int namelen)
{
	int sz;
	const int mask = sizeof(ino_t) - 1;

	BUILD_BUG_ON(sizeof(ino_t) != sizeof(long));

	sz = sizeof(struct au_vdir_de) + namelen;
	if (sz & mask) {
		sz += sizeof(ino_t);
		sz &= ~mask;
	}

	AuDebugOn(sz % sizeof(ino_t));
	return sz;
}

static int set_deblk_end(union au_vdir_deblk_p *p,
			 union au_vdir_deblk_p *deblk_end)
{
	if (calc_size(0) <= deblk_end->p - p->p) {
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
	if (calc_size(0) <= deblk_end->p - p->p)
		return !p->de->de_str.len;
	return 1;
}

static au_vdir_deblk_t *last_deblk(struct au_vdir *vdir)
{
	return vdir->vd_deblk[vdir->vd_nblk - 1];
}

void au_nhash_init(struct au_nhash *nhash)
{
	int i;
	struct hlist_head *heads;

	heads = nhash->heads;
	for (i = 0; i < AuSize_NHASH; i++)
		INIT_HLIST_HEAD(heads++);
}

struct au_nhash *au_nhash_new(gfp_t gfp)
{
	struct au_nhash *nhash;

	nhash = kmalloc(sizeof(*nhash), gfp);
	if (nhash) {
		au_nhash_init(nhash);
		return nhash;
	}
	return ERR_PTR(-ENOMEM);
}

void au_nhash_del(struct au_nhash *nhash)
{
	au_nhash_fin(nhash);
	kfree(nhash);
}

void au_nhash_move(struct au_nhash *dst, struct au_nhash *src)
{
	int i;
	struct hlist_head *dsth, *srch;

	*dst = *src;
	srch = src->heads;
	dsth = dst->heads;
	for (i = 0; i < AuSize_NHASH; i++) {
		if (dsth->first)
			dsth->first->pprev = &dsth->first;
		dsth++;
		INIT_HLIST_HEAD(srch++);
	}
	/* smp_mb(); */
}

/* ---------------------------------------------------------------------- */

void au_nhash_fin(struct au_nhash *whlist)
{
	int i;
	struct hlist_head *head;
	struct au_vdir_wh *tpos;
	struct hlist_node *pos, *n;

	head = whlist->heads;
	for (i = 0; i < AuSize_NHASH; i++) {
		hlist_for_each_entry_safe(tpos, pos, n, head, wh_hash) {
			/* hlist_del(pos); */
			kfree(tpos);
		}
		head++;
	}
}

int au_nhash_test_longer_wh(struct au_nhash *whlist, aufs_bindex_t btgt,
			    int limit)
{
	int n, i;
	struct hlist_head *head;
	struct au_vdir_wh *tpos;
	struct hlist_node *pos;

	n = 0;
	head = whlist->heads;
	for (i = 0; i < AuSize_NHASH; i++) {
		hlist_for_each_entry(tpos, pos, head, wh_hash)
			if (tpos->wh_bindex == btgt && ++n > limit)
				return 1;
		head++;
	}
	return 0;
}

static unsigned int au_name_hash(const unsigned char *name, unsigned int len)
{
	return full_name_hash(name, len) % AuSize_NHASH;
}

/* returns found or not */
int au_nhash_test_known_wh(struct au_nhash *whlist, char *name, int namelen)
{
	struct hlist_head *head;
	struct au_vdir_wh *tpos;
	struct hlist_node *pos;
	struct au_vdir_destr *str;

	head = whlist->heads + au_name_hash(name, namelen);
	hlist_for_each_entry(tpos, pos, head, wh_hash) {
		str = &tpos->wh_str;
		AuDbg("%.*s\n", str->len, str->name);
		if (str->len == namelen && !memcmp(str->name, name, namelen))
			return 1;
	}
	return 0;
}

int au_nhash_append_wh(struct au_nhash *whlist, char *name, int namelen,
		       aufs_bindex_t bindex)
{
	int err;
	struct au_vdir_destr *str;
	struct au_vdir_wh *wh;

	err = -ENOMEM;
	wh = kmalloc(sizeof(*wh) + namelen, GFP_NOFS);
	if (unlikely(!wh))
		goto out;

	err = 0;
	wh->wh_bindex = bindex;
	str = &wh->wh_str;
	str->len = namelen;
	memcpy(str->name, name, namelen);
	hlist_add_head(&wh->wh_hash,
		       whlist->heads + au_name_hash(name, namelen));
	/* smp_mb(); */

 out:
	return err;
}

/* ---------------------------------------------------------------------- */

void au_vdir_free(struct au_vdir *vdir)
{
	au_vdir_deblk_t **deblk;

	deblk = vdir->vd_deblk;
	while (vdir->vd_nblk--)
		kfree(*deblk++);
	kfree(vdir->vd_deblk);
	au_cache_free_vdir(vdir);
}

static int append_deblk(struct au_vdir *vdir)
{
	int err, sz, i;
	au_vdir_deblk_t **o;
	union au_vdir_deblk_p p, deblk_end;

	err = -ENOMEM;
	sz = sizeof(*o) * vdir->vd_nblk;
	o = au_kzrealloc(vdir->vd_deblk, sz, sz + sizeof(*o), GFP_NOFS);
	if (unlikely(!o))
		goto out;

	vdir->vd_deblk = o;
	p.deblk = kmalloc(sizeof(*p.deblk), GFP_NOFS);
	if (p.deblk) {
		i = vdir->vd_nblk++;
		vdir->vd_deblk[i] = p.deblk;
		vdir->vd_last.i = i;
		vdir->vd_last.p.p = p.p;
		deblk_end.deblk = p.deblk + 1;
		err = set_deblk_end(&p, &deblk_end);
	}

 out:
	return err;
}

static struct au_vdir *alloc_vdir(void)
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
		vdir->vd_deblk[vdir->vd_nblk - 1] = NULL;
		vdir->vd_nblk--;
	}
	p.deblk = vdir->vd_deblk[0];
	deblk_end.deblk = p.deblk + 1;
	err = set_deblk_end(&p, &deblk_end);
	vdir->vd_version = 0;
	vdir->vd_jiffy = 0;
	vdir->vd_last.i = 0;
	vdir->vd_last.p.deblk = vdir->vd_deblk[0];
	/* smp_mb(); */
	return err;
}

/* ---------------------------------------------------------------------- */

static void free_dehlist(struct au_nhash *dehlist)
{
	int i;
	struct hlist_head *head;
	struct au_vdir_dehstr *tpos;
	struct hlist_node *pos, *n;

	head = dehlist->heads;
	for (i = 0; i < AuSize_NHASH; i++) {
		hlist_for_each_entry_safe(tpos, pos, n, head, hash) {
			/* hlist_del(pos); */
			au_cache_free_dehstr(tpos);
		}
		head++;
	}
}

/* returns found(true) or not */
static int test_known(struct au_nhash *delist, char *name, int namelen)
{
	struct hlist_head *head;
	struct au_vdir_dehstr *tpos;
	struct hlist_node *pos;
	struct au_vdir_destr *str;

	head = delist->heads + au_name_hash(name, namelen);
	hlist_for_each_entry(tpos, pos, head, hash) {
		str = tpos->str;
		AuDbg("%.*s\n", str->len, str->name);
		if (str->len == namelen && !memcmp(str->name, name, namelen))
			return 1;
	}
	return 0;

}

static int append_de(struct au_vdir *vdir, char *name, int namelen, ino_t ino,
		     unsigned int d_type, struct au_nhash *delist)
{
	int err, sz;
	union au_vdir_deblk_p p, *room, deblk_end;
	struct au_vdir_dehstr *dehstr;

	p.deblk = last_deblk(vdir);
	deblk_end.deblk = p.deblk + 1;
	room = &vdir->vd_last.p;
	AuDebugOn(room->p < p.p || deblk_end.p <= room->p
		  || !is_deblk_end(room, &deblk_end));

	sz = calc_size(namelen);
	if (unlikely(sz > deblk_end.p - room->p)) {
		err = append_deblk(vdir);
		if (unlikely(err))
			goto out;

		p.deblk = last_deblk(vdir);
		deblk_end.deblk = p.deblk + 1;
		/* smp_mb(); */
		AuDebugOn(room->p != p.p);
	}

	err = -ENOMEM;
	dehstr = au_cache_alloc_dehstr();
	if (unlikely(!dehstr))
		goto out;

	dehstr->str = &room->de->de_str;
	hlist_add_head(&dehstr->hash,
		       delist->heads + au_name_hash(name, namelen));
	room->de->de_ino = ino;
	room->de->de_type = d_type;
	room->de->de_str.len = namelen;
	memcpy(room->de->de_str.name, name, namelen);

	err = 0;
	room->p += sz;
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

static int fillvdir(void *__arg, const char *__name, int namelen,
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
	if (namelen <= AUFS_WH_PFX_LEN
	    || memcmp(name, AUFS_WH_PFX, AUFS_WH_PFX_LEN)) {
		delist = arg->delist;
		for (bindex = 0; bindex < bend; bindex++)
			if (test_known(delist++, name, namelen)
			    || au_nhash_test_known_wh(arg->whlist + bindex,
						      name, namelen))
				goto out; /* already exists or whiteouted */

		ino = 1; /* why does gcc warn? */
		sb = arg->file->f_dentry->d_sb;
		arg->err = au_ino(sb, bend, h_ino, d_type, &ino);
		if (!arg->err)
			arg->err = append_de(arg->vdir, name, namelen, ino,
					     d_type, arg->delist + bend);
	} else {
		name += AUFS_WH_PFX_LEN;
		namelen -= AUFS_WH_PFX_LEN;
		whlist = arg->whlist;
		for (bindex = 0; bindex < bend; bindex++)
			if (au_nhash_test_known_wh(whlist++, name, namelen))
				goto out; /* already whiteouted */

		ino = 1; /* dummy */
		if (!arg->err)
			arg->err = au_nhash_append_wh
				(arg->whlist + bend, name, namelen, bend);
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
	aufs_bindex_t bend, bindex, bstart;
	struct file *hf, *file;
	struct au_nhash *delist, *whlist;

	err = -ENOMEM;
	bend = au_fbend(arg->file);
	arg->delist = kmalloc(sizeof(*arg->delist) * (bend + 1), GFP_NOFS);
	if (unlikely(!arg->delist))
		goto out;
	arg->whlist = kmalloc(sizeof(*arg->whlist) * (bend + 1), GFP_NOFS);
	if (unlikely(!arg->whlist))
		goto out_delist;

	err = 0;
	delist = arg->delist;
	whlist = arg->whlist;
	for (bindex = 0; bindex <= bend; bindex++) {
		au_nhash_init(delist++);
		au_nhash_init(whlist++);
	}

	arg->flags = 0;
	file = arg->file;
	bstart = au_fbstart(file);
	for (bindex = bstart; !err && bindex <= bend; bindex++) {
		hf = au_h_fptr(file, bindex);
		if (!hf)
			continue;

		offset = vfsub_llseek(hf, 0, SEEK_SET);
		err = offset;
		if (unlikely(offset))
			break;

		arg->bindex = bindex;
		do {
			arg->err = 0;
			au_fclr_fillvdir(arg->flags, CALLED);
			/* smp_mb(); */
			err = vfsub_readdir(hf, fillvdir, arg);
			if (err >= 0)
				err = arg->err;
		} while (!err && au_ftest_fillvdir(arg->flags, CALLED));
	}

	delist = arg->delist + bstart;
	whlist = arg->whlist + bstart;
	for (bindex = bstart; bindex <= bend; bindex++) {
		free_dehlist(delist++);
		au_nhash_fin(whlist++);
	}
	kfree(arg->whlist);

 out_delist:
	kfree(arg->delist);
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
		vdir = alloc_vdir();
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
		vdir->vd_last.i = 0;
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
	int err, i, rerr, n;

	AuDebugOn(tgt->vd_nblk != 1);

	err = -ENOMEM;
	if (tgt->vd_nblk < src->vd_nblk) {
		au_vdir_deblk_t **p;

		p = au_kzrealloc(tgt->vd_deblk, sizeof(*p) * tgt->vd_nblk,
				 sizeof(*p) * src->vd_nblk, GFP_NOFS);
		if (unlikely(!p))
			goto out;
		tgt->vd_deblk = p;
	}

	tgt->vd_nblk = src->vd_nblk;
	n = src->vd_nblk;
	memcpy(tgt->vd_deblk[0], src->vd_deblk[0], AuSize_DEBLK);
	/* tgt->vd_last.i = 0; */
	/* tgt->vd_last.p.deblk = tgt->vd_deblk[0]; */
	tgt->vd_version = src->vd_version;
	tgt->vd_jiffy = src->vd_jiffy;

	for (i = 1; i < n; i++) {
		tgt->vd_deblk[i] = kmalloc(AuSize_DEBLK, GFP_NOFS);
		if (tgt->vd_deblk[i])
			memcpy(tgt->vd_deblk[i], src->vd_deblk[i],
			       AuSize_DEBLK);
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
		vdir_cache = alloc_vdir();
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

	p.deblk = vdir->vd_deblk[vdir->vd_last.i];
	offset = vdir->vd_last.p.p - p.p;
	offset += sizeof(*p.deblk) * vdir->vd_last.i;
	return offset;
}

/* returns true or false */
static int seek_vdir(struct file *file)
{
	int valid, i, n;
	loff_t offset;
	union au_vdir_deblk_p p, deblk_end;
	struct au_vdir *vdir_cache;

	valid = 1;
	vdir_cache = au_fvdir_cache(file);
	offset = calc_offset(vdir_cache);
	AuDbg("offset %lld\n", offset);
	if (file->f_pos == offset)
		goto out;

	vdir_cache->vd_last.i = 0;
	vdir_cache->vd_last.p.deblk = vdir_cache->vd_deblk[0];
	if (!file->f_pos)
		goto out;

	valid = 0;
	i = file->f_pos / AuSize_DEBLK;
	AuDbg("i %d\n", i);
	if (i >= vdir_cache->vd_nblk)
		goto out;

	n = vdir_cache->vd_nblk;
	for (; i < n; i++) {
		p.deblk = vdir_cache->vd_deblk[i];
		deblk_end.deblk = p.deblk + 1;
		offset = i;
		offset *= AuSize_DEBLK;
		while (!is_deblk_end(&p, &deblk_end) && offset < file->f_pos) {
			int l;

			l = calc_size(p.de->de_str.len);
			offset += l;
			p.p += l;
		}
		if (!is_deblk_end(&p, &deblk_end)) {
			valid = 1;
			vdir_cache->vd_last.i = i;
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
	int err, l;
	union au_vdir_deblk_p deblk_end;
	struct au_vdir *vdir_cache;
	struct au_vdir_de *de;

	BUILD_BUG_ON(AuSize_DEBLK < NAME_MAX || PAGE_SIZE < AuSize_DEBLK);

	vdir_cache = au_fvdir_cache(file);
	if (!seek_vdir(file))
		return 0;

	while (1) {
		deblk_end.deblk
			= vdir_cache->vd_deblk[vdir_cache->vd_last.i] + 1;
		while (!is_deblk_end(&vdir_cache->vd_last.p, &deblk_end)) {
			de = vdir_cache->vd_last.p.de;
			AuDbg("%.*s, off%lld, i%lu, dt%d\n",
				  de->de_str.len, de->de_str.name,
				  file->f_pos, (unsigned long)de->de_ino,
				  de->de_type);
			err = filldir(dirent, de->de_str.name, de->de_str.len,
				      file->f_pos, de->de_ino, de->de_type);
			if (unlikely(err)) {
				AuTraceErr(err);
				/* todo: ignore the error caused by udba? */
				/* return err; */
				return 0;
			}

			l = calc_size(de->de_str.len);
			vdir_cache->vd_last.p.p += l;
			file->f_pos += l;
		}
		if (vdir_cache->vd_last.i < vdir_cache->vd_nblk - 1) {
			vdir_cache->vd_last.i++;
			vdir_cache->vd_last.p.deblk
				= vdir_cache->vd_deblk[vdir_cache->vd_last.i];
			file->f_pos = sizeof(*vdir_cache->vd_last.p.deblk)
				* vdir_cache->vd_last.i;
			continue;
		}
		break;
	}

	/* smp_mb(); */
	return 0;
}

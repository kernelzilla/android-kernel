/*
 * Copyright (C) 2005-2010 Junjiro R. Okajima
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
 * inotify for the lower directories
 */

#include "aufs.h"

struct inotify_handle *au_hin_handle;

static char *in_name(u32 mask)
{
#ifdef CONFIG_AUFS_DEBUG
#define test_ret(flag)	if (mask & flag) \
				return #flag;
	test_ret(IN_ACCESS);
	test_ret(IN_MODIFY);
	test_ret(IN_ATTRIB);
	test_ret(IN_CLOSE_WRITE);
	test_ret(IN_CLOSE_NOWRITE);
	test_ret(IN_OPEN);
	test_ret(IN_MOVED_FROM);
	test_ret(IN_MOVED_TO);
	test_ret(IN_CREATE);
	test_ret(IN_DELETE);
	test_ret(IN_DELETE_SELF);
	test_ret(IN_MOVE_SELF);
	test_ret(IN_UNMOUNT);
	test_ret(IN_Q_OVERFLOW);
	test_ret(IN_IGNORED);
	return "";
#undef test_ret
#else
	return "??";
#endif
}

static void aufs_inotify(struct inotify_watch *watch, u32 wd __maybe_unused,
			 u32 mask, u32 cookie __maybe_unused,
			 const char *h_child_name, struct inode *h_child_inode)
{
	struct au_hinotify *hinotify;
	struct au_postproc_args *args;
	int len, wkq_err;
	unsigned char isdir, isroot, wh;
	char *p;
	struct inode *dir;
	unsigned int flags[2];

	/* if IN_UNMOUNT happens, there must be another bug */
	AuDebugOn(mask & IN_UNMOUNT);
	if (mask & (IN_IGNORED | IN_UNMOUNT)) {
		put_inotify_watch(watch);
		return;
	}
#ifdef AuDbgHinotify
	au_debug(1);
	if (1 || !h_child_name || strcmp(h_child_name, AUFS_XINO_FNAME)) {
		AuDbg("i%lu, wd %d, mask 0x%x %s, cookie 0x%x, hcname %s,"
		      " hi%lu\n",
		      watch->inode->i_ino, wd, mask, in_name(mask), cookie,
		      h_child_name ? h_child_name : "",
		      h_child_inode ? h_child_inode->i_ino : 0);
		WARN_ON(1);
	}
	au_debug(0);
#endif

	hinotify = container_of(watch, struct au_hinotify, hin_watch);
	AuDebugOn(!hinotify || !hinotify->hin_aufs_inode);
	dir = igrab(hinotify->hin_aufs_inode);
	if (!dir)
		return;

	isroot = (dir->i_ino == AUFS_ROOT_INO);
	len = 0;
	wh = 0;
	if (h_child_name) {
		len = strlen(h_child_name);
		if (!memcmp(h_child_name, AUFS_WH_PFX, AUFS_WH_PFX_LEN)) {
			h_child_name += AUFS_WH_PFX_LEN;
			len -= AUFS_WH_PFX_LEN;
			wh = 1;
		}
	}

	isdir = 0;
	if (h_child_inode)
		isdir = !!S_ISDIR(h_child_inode->i_mode);
	flags[PARENT] = AuHinJob_ISDIR;
	flags[CHILD] = 0;
	if (isdir)
		flags[CHILD] = AuHinJob_ISDIR;
	au_fset_hinjob(flags[PARENT], DIRENT);
	au_fset_hinjob(flags[CHILD], GEN);
	switch (mask & IN_ALL_EVENTS) {
	case IN_MOVED_FROM:
	case IN_MOVED_TO:
		au_fset_hinjob(flags[CHILD], XINO0);
		au_fset_hinjob(flags[CHILD], MNTPNT);
		/*FALLTHROUGH*/
	case IN_CREATE:
		AuDebugOn(!h_child_name || !h_child_inode);
		break;

	case IN_DELETE:
		/*
		 * aufs never be able to get this child inode.
		 * revalidation should be in d_revalidate()
		 * by checking i_nlink, i_generation or d_unhashed().
		 */
		AuDebugOn(!h_child_name);
		au_fset_hinjob(flags[CHILD], TRYXINO0);
		au_fset_hinjob(flags[CHILD], MNTPNT);
		break;

	default:
		AuDebugOn(1);
	}

	if (wh)
		h_child_inode = NULL;

	/* iput() and kfree() will be called in postproc() */
	/*
	 * inotify_mutex is already acquired and kmalloc/prune_icache may lock
	 * iprune_mutex. strange.
	 */
	lockdep_off();
	args = kmalloc(sizeof(*args) + len + 1, GFP_NOFS);
	lockdep_on();
	if (unlikely(!args)) {
		AuErr1("no memory\n");
		iput(dir);
		return;
	}
	args->flags[PARENT] = flags[PARENT];
	args->flags[CHILD] = flags[CHILD];
	args->mask = mask;
	args->dir = dir;
	args->h_dir = igrab(watch->inode);
	if (h_child_inode)
		h_child_inode = igrab(h_child_inode); /* can be NULL */
	args->h_child_inode = h_child_inode;
	args->h_child_nlen = len;
	if (len) {
		p = (void *)args;
		p += sizeof(*args);
		memcpy(p, h_child_name, len + 1);
	}

	lockdep_off();
	wkq_err = au_wkq_nowait(au_postproc, args, dir->i_sb);
	lockdep_on();
	if (unlikely(wkq_err))
		AuErr("wkq %d\n", wkq_err);
}

static void aufs_inotify_destroy(struct inotify_watch *watch __maybe_unused)
{
	return;
}

const struct inotify_operations aufs_inotify_ops = {
	.handle_event	= aufs_inotify,
	.destroy_watch	= aufs_inotify_destroy
};

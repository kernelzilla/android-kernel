/*
 * Copyright (C) 2010 Junjiro R. Okajima
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
 * dynamically customizable operations for regular files
 */

#include "aufs.h"

#define DyPrSym(key)	AuDbgSym(key->dk_op.dy_hop)

/*
 * How large will these lists be?
 * Usually just a few elements, 20-30 at most for each, I guess.
 */
static struct au_splhead dynop[AuDyLast];

static struct au_dykey *dy_gfind_get(struct au_splhead *spl, const void *h_op)
{
	struct au_dykey *key, *tmp;
	struct list_head *head;

	key = NULL;
	head = &spl->head;
	rcu_read_lock();
	list_for_each_entry_rcu(tmp, head, dk_list)
		if (tmp->dk_op.dy_hop == h_op) {
			key = tmp;
			kref_get(&key->dk_kref);
			break;
		}
	rcu_read_unlock();

	return key;
}

static struct au_dykey *dy_bradd(struct au_branch *br, struct au_dykey *key)
{
	struct au_dykey **k, *found;
	const void *h_op = key->dk_op.dy_hop;
	int i;

	found = NULL;
	k = br->br_dykey;
	for (i = 0; i < AuBrDynOp; i++)
		if (k[i]) {
			if (k[i]->dk_op.dy_hop == h_op) {
				found = k[i];
				break;
			}
		} else
			break;
	if (!found) {
		spin_lock(&br->br_dykey_lock);
		for (; i < AuBrDynOp; i++)
			if (k[i]) {
				if (k[i]->dk_op.dy_hop == h_op) {
					found = k[i];
					break;
				}
			} else {
				k[i] = key;
				break;
			}
		spin_unlock(&br->br_dykey_lock);
		BUG_ON(i == AuBrDynOp); /* expand the array */
	}

	return found;
}

/* kref_get() if @key is already added */
static struct au_dykey *dy_gadd(struct au_splhead *spl, struct au_dykey *key)
{
	struct au_dykey *tmp, *found;
	struct list_head *head;
	const void *h_op = key->dk_op.dy_hop;

	found = NULL;
	head = &spl->head;
	spin_lock(&spl->spin);
	list_for_each_entry(tmp, head, dk_list)
		if (tmp->dk_op.dy_hop == h_op) {
			kref_get(&tmp->dk_kref);
			found = tmp;
			break;
		}
	if (!found)
		list_add_rcu(&key->dk_list, head);
	spin_unlock(&spl->spin);

	if (!found)
		DyPrSym(key);
	return found;
}

static void dy_free_rcu(struct rcu_head *rcu)
{
	struct au_dykey *key;

	key = container_of(rcu, struct au_dykey, dk_rcu);
	DyPrSym(key);
	kfree(key);
}

static void dy_free(struct kref *kref)
{
	struct au_dykey *key;
	struct au_splhead *spl;

	key = container_of(kref, struct au_dykey, dk_kref);
	spl = dynop + key->dk_op.dy_type;
	spin_lock(&spl->spin);
	list_del_rcu(&key->dk_list);
	spin_unlock(&spl->spin);
	call_rcu(&key->dk_rcu, dy_free_rcu);
}

void au_dy_put(struct au_dykey *key)
{
	kref_put(&key->dk_kref, dy_free);
}

/* ---------------------------------------------------------------------- */

#define DyDbgSize(cnt, op)	AuDebugOn(cnt != sizeof(op)/sizeof(void *))

#ifdef CONFIG_AUFS_DEBUG
#define DyDbgDeclare(cnt)	unsigned int cnt = 0
#define DyDbgInc(cnt)		cnt++
#else
#define DyDbgDeclare(cnt)	do {} while (0)
#define DyDbgInc(cnt)		do {} while (0)
#endif

#define DySet(func, dst, src, h_op, h_sb) do {				\
	DyDbgInc(cnt);							\
	if (h_op->func) {						\
		if (src.func)						\
			dst.func = src.func;				\
		else							\
			AuDbg("%s %s\n", au_sbtype(h_sb), #func);	\
	}								\
} while (0)

#define DySetForce(func, dst, src) do {		\
	AuDebugOn(!src.func);			\
	DyDbgInc(cnt);				\
	dst.func = src.func;			\
} while (0)

#define DySetFop(func) \
	DySet(func, dyfop->df_op, aufs_file_fop, h_fop, h_sb)
#define DySetFopForce(func) \
	DySetForce(func, dyfop->df_op, aufs_file_fop)

static void dy_fop(struct au_dykey *key, const void *h_op,
		   struct super_block *h_sb __maybe_unused)
{
	struct au_dyfop *dyfop = (void *)key;
	const struct file_operations *h_fop = h_op;
	DyDbgDeclare(cnt);

	AuDbg("%s\n", au_sbtype(h_sb));

	DySetFopForce(owner);	/* force */
	DySetFop(llseek);
	DySetFop(read);
	DySetFop(write);
	DySetFop(aio_read);
	DySetFop(aio_write);
	DySetFop(readdir);
	DySetFop(poll);
	DySetFop(ioctl);
	DySetFop(unlocked_ioctl);
	DySetFop(compat_ioctl);
	DySetFop(mmap);
	DySetFopForce(open);	/* force */
	DySetFop(flush);
	DySetFopForce(release);	/* force */
	DySetFop(fsync);
	DySetFop(aio_fsync);
	DySetFop(fasync);
	DySetFop(lock);
	DySetFop(sendpage);
	DySetFop(get_unmapped_area);
	DySetFop(check_flags);
	DySetFop(flock);
	DySetFop(splice_write);
	DySetFop(splice_read);
	DySetFop(setlease);

	DyDbgSize(cnt, *h_fop);
}

#define DySetVmop(func) \
	DySet(func, dyvmop->dv_op, aufs_vm_ops, h_vmop, h_sb)
#define DySetVmopForce(func) \
	DySetForce(func, dyvmop->dv_op, aufs_vm_ops)

static void dy_vmop(struct au_dykey *key, const void *h_op,
		    struct super_block *h_sb __maybe_unused)
{
	struct au_dyvmop *dyvmop = (void *)key;
	const struct vm_operations_struct *h_vmop = h_op;
	DyDbgDeclare(cnt);

	AuDbg("%s\n", au_sbtype(h_sb));

	DySetVmop(open);
	DySetVmop(close);
	DySetVmop(fault);
	DySetVmop(page_mkwrite);
	DySetVmop(access);
#ifdef CONFIG_NUMA
	DySetVmop(set_policy);
	DySetVmop(get_policy);
	DySetVmop(migrate);
#endif

	DyDbgSize(cnt, *h_vmop);
}

/* ---------------------------------------------------------------------- */

static void dy_bug(struct kref *kref)
{
	BUG();
}

static struct au_dykey *dy_get(struct au_dynop *op, struct au_branch *br)
{
	struct au_dykey *key, *old;
	struct au_splhead *spl;
	static const struct {
		unsigned int sz;
		void (*set_op)(struct au_dykey *key, const void *h_op,
			       struct super_block *h_sb __maybe_unused);
	} a[] = {
		[AuDy_FOP] = {
			.sz	= sizeof(struct au_dyfop),
			.set_op	= dy_fop
		},
		[AuDy_VMOP] = {
			.sz	= sizeof(struct au_dyvmop),
			.set_op	= dy_vmop
		}
	}, *p;

	spl = dynop + op->dy_type;
	key = dy_gfind_get(spl, op->dy_hop);
	if (key)
		goto out_add; /* success */

	p = a + op->dy_type;
	key = kzalloc(p->sz, GFP_NOFS);
	if (unlikely(!key)) {
		key = ERR_PTR(-ENOMEM);
		goto out;
	}

	key->dk_op.dy_hop = op->dy_hop;
	kref_init(&key->dk_kref);
	INIT_RCU_HEAD(&key->dk_rcu);
	p->set_op(key, op->dy_hop, br->br_mnt->mnt_sb);
	old = dy_gadd(spl, key);
	if (old) {
		kfree(key);
		key = old;
	}

out_add:
	old = dy_bradd(br, key);
	if (old)
		/* its ref-count should never be zero here */
		kref_put(&key->dk_kref, dy_bug);
out:
	return key;
}

/* ---------------------------------------------------------------------- */

static struct au_dyfop *dy_fget(struct au_branch *br,
				const struct file_operations *h_fop)
{
	struct au_dynop op = {
		.dy_type	= AuDy_FOP,
		/* .dy_hfop	= h_inode->i_fop */
	};

	op.dy_hfop = h_fop;
	return (void *)dy_get(&op, br);
}

int au_dy_ifop(struct inode *inode, aufs_bindex_t bindex, struct inode *h_inode)
{
	int err;
	struct au_branch *br;
	struct au_dyfop *dyfop;

	AuDebugOn(!S_ISREG(h_inode->i_mode));
	IiMustWriteLock(inode);

	br = au_sbr(inode->i_sb, bindex);
	dyfop = dy_fget(br, h_inode->i_fop);
	err = PTR_ERR(dyfop);
	if (IS_ERR(dyfop))
		goto out;

	err = 0;
	inode->i_fop = &dyfop->df_op;

out:
	return err;
}

const struct vm_operations_struct *
au_dy_vmop(struct file *file, struct au_branch *br,
	   const struct vm_operations_struct *h_vmop)
{
	struct au_dyvmop *dyvmop;
	struct au_dynop op;

	op.dy_type = AuDy_VMOP;
	op.dy_hvmop = h_vmop;
	dyvmop = (void *)dy_get(&op, br);
	if (IS_ERR(dyvmop))
		return (void *)dyvmop;
	return &dyvmop->dv_op;
}

/* ---------------------------------------------------------------------- */

void __init au_dy_init(void)
{
	int i;

	/* make sure that 'struct au_dykey *' can be any type */
	BUILD_BUG_ON(offsetof(struct au_dyfop, df_key));
	BUILD_BUG_ON(offsetof(struct au_dyvmop, dv_key));

	for (i = 0; i < AuDyLast; i++)
		au_spl_init(dynop + i);
}

void au_dy_fin(void)
{
	int i;

	for (i = 0; i < AuDyLast; i++)
		WARN_ON(!list_empty(&dynop[i].head));
}

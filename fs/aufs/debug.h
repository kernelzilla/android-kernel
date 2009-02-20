/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*
 * debug print functions
 */

#ifndef __AUFS_DEBUG_H__
#define __AUFS_DEBUG_H__

#ifdef __KERNEL__

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kd.h>
#include <linux/vt_kern.h>
#include <linux/sysrq.h>
#include <linux/aufs_type.h>

#ifdef CONFIG_AUFS_DEBUG
#define AuDebugOn(a)		BUG_ON(a)
extern atomic_t au_cond;
static inline void au_debug(int n)
{
	atomic_set(&au_cond, n);
	smp_mb();
}

static inline int au_debug_test(void)
{
	int ret;

	ret = atomic_read(&au_cond);
	smp_mb();
	return ret;
}
#else
#define AuDebugOn(a)		do {} while (0)
#define au_debug()		do {} while (0)
static inline int au_debug_test(void)
{
	return 0;
}
#endif /* CONFIG_AUFS_DEBUG */

/* ---------------------------------------------------------------------- */

/* debug print */

#define AuDpri(lvl, fmt, arg...) \
	printk(lvl AUFS_NAME " %s:%d:%s[%d]: " fmt, \
	       __func__, __LINE__, current->comm, current->pid, ##arg)
#define AuDbg(fmt, arg...) do { \
	if (au_debug_test()) \
		AuDpri(KERN_DEBUG, fmt, ##arg); \
} while (0)
#define AuLabel(l) 		AuDbg(#l "\n")
#define AuInfo(fmt, arg...)	AuDpri(KERN_INFO, fmt, ##arg)
#define AuWarn(fmt, arg...)	AuDpri(KERN_WARNING, fmt, ##arg)
#define AuErr(fmt, arg...)	AuDpri(KERN_ERR, fmt, ##arg)
#define AuIOErr(fmt, arg...)	AuErr("I/O Error, " fmt, ##arg)
#define AuWarn1(fmt, arg...) do { \
	static unsigned char _c; \
	if (!_c++) \
		AuWarn(fmt, ##arg); \
} while (0)

#define AuErr1(fmt, arg...) do { \
	static unsigned char _c; \
	if (!_c++) \
		AuErr(fmt, ##arg); \
} while (0)

#define AuIOErr1(fmt, arg...) do { \
	static unsigned char _c; \
	if (!_c++) \
		AuIOErr(fmt, ##arg); \
} while (0)

#define AuUnsupportMsg	"This operation is not supported." \
			" Please report this application to aufs-users ML."
#define AuUnsupport(fmt, args...) do { \
	AuErr(AuUnsupportMsg "\n" fmt, ##args); \
	dump_stack(); \
} while (0)

#define AuTraceErr(e) do { \
	if (unlikely((e) < 0)) \
		AuDbg("err %d\n", (int)(e)); \
} while (0)

#define AuTraceErrPtr(p) do { \
	if (IS_ERR(p)) \
		AuDbg("err %ld\n", PTR_ERR(p)); \
} while (0)

/* dirty macros for debug print, use with "%.*s" and caution */
#define AuLNPair(qstr)		(qstr)->len, (qstr)->name
#define AuDLNPair(d)		AuLNPair(&(d)->d_name)

/* ---------------------------------------------------------------------- */

struct au_sbinfo;
struct au_finfo;
#ifdef CONFIG_AUFS_DEBUG
extern char *au_plevel;
struct au_nhash;
void au_dpri_whlist(struct au_nhash *whlist);
struct au_vdir;
void au_dpri_vdir(struct au_vdir *vdir);
void au_dpri_inode(struct inode *inode);
void au_dpri_dentry(struct dentry *dentry);
void au_dpri_file(struct file *filp);
void au_dpri_sb(struct super_block *sb);

void au_dbg_sleep_jiffy(int jiffy);
void au_dbg_iattr(struct iattr *ia);

void au_dbg_verify_dir_parent(struct dentry *dentry, unsigned int sigen);
void au_dbg_verify_nondir_parent(struct dentry *dentry, unsigned int sigen);
void au_dbg_verify_gen(struct dentry *parent, unsigned int sigen);
void au_dbg_verify_hf(struct au_finfo *finfo);
void au_dbg_verify_kthread(void);

int __init au_debug_init(void);
void au_debug_sbinfo_init(struct au_sbinfo *sbinfo);
#define AuDbgWhlist(w) do { \
	AuDbg(#w "\n"); \
	au_dpri_whlist(w); \
} while (0)

#define AuDbgVdir(v) do { \
	AuDbg(#v "\n"); \
	au_dpri_vdir(v); \
} while (0)

#define AuDbgInode(i) do { \
	AuDbg(#i "\n"); \
	au_dpri_inode(i); \
} while (0)

#define AuDbgDentry(d) do { \
	AuDbg(#d "\n"); \
	au_dpri_dentry(d); \
} while (0)

#define AuDbgFile(f) do { \
	AuDbg(#f "\n"); \
	au_dpri_file(f); \
} while (0)

#define AuDbgSb(sb) do { \
	AuDbg(#sb "\n"); \
	au_dpri_sb(sb); \
} while (0)

#define AuDbgSleep(sec) do { \
	AuDbg("sleep %d sec\n", sec); \
	ssleep(sec); \
} while (0)

#define AuDbgSleepJiffy(jiffy) do { \
	AuDbg("sleep %d jiffies\n", jiffy); \
	au_dbg_sleep_jiffy(jiffy); \
} while (0)

#define AuDbgIAttr(ia) do { \
	AuDbg("ia_valid 0x%x\n", (ia)->ia_valid); \
	au_dbg_iattr(ia); \
} while (0)
#else
static inline void au_dbg_verify_dir_parent(struct dentry *dentry,
					    unsigned int sigen)
{
	/* empty */
}
static inline void au_dbg_verify_nondir_parent(struct dentry *dentry,
					   unsigned int sigen)
{
	/* empty */
}
static inline void au_dbg_verify_gen(struct dentry *parent, unsigned int sigen)
{
	/* empty */
}
static inline void au_dbg_verify_hf(struct au_finfo *finfo)
{
	/* empty */
}
static inline void au_dbg_verify_kthread(void)
{
	/* empty */
}

static inline int au_debug_init(void)
{
	return 0;
}
static inline void au_debug_sbinfo_init(struct au_sbinfo *sbinfo)
{
	/* empty */
}
#define AuDbgWhlist(w)		do {} while (0)
#define AuDbgVdir(v)		do {} while (0)
#define AuDbgInode(i)		do {} while (0)
#define AuDbgDentry(d)		do {} while (0)
#define AuDbgFile(f)		do {} while (0)
#define AuDbgSb(sb)		do {} while (0)
#define AuDbgSleep(sec)		do {} while (0)
#define AuDbgSleepJiffy(jiffy)	do {} while (0)
#define AuDbgIAttr(ia)		do {} while (0)
#endif /* CONFIG_AUFS_DEBUG */

/* ---------------------------------------------------------------------- */

#ifdef CONFIG_AUFS_MAGIC_SYSRQ
int __init au_sysrq_init(void);
void au_sysrq_fin(void);

#ifdef CONFIG_HW_CONSOLE
#define au_dbg_blocked() do { \
	WARN_ON(1); \
	handle_sysrq('w', vc_cons[fg_console].d->vc_tty); \
} while (0)
#else
#define au_dbg_blocked()	do {} while (0)
#endif

#else
static inline int au_sysrq_init(void)
{
	return 0;
}
#define au_sysrq_fin()		do {} while (0)
#define au_dbg_blocked()	do {} while (0)
#endif /* CONFIG_AUFS_MAGIC_SYSRQ */

#endif /* __KERNEL__ */
#endif /* __AUFS_DEBUG_H__ */

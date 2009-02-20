/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __AUFS_TYPE_H__
#define __AUFS_TYPE_H__

#include <linux/ioctl.h>

#define AUFS_VERSION	"2-27"

/* todo? move this to linux-2.6.19/include/magic.h */
#define AUFS_SUPER_MAGIC	('a' << 24 | 'u' << 16 | 'f' << 8 | 's')

/* ---------------------------------------------------------------------- */

#ifdef CONFIG_AUFS_BRANCH_MAX_127
/* some environments treat 'char' as 'unsigned char' by default */
typedef signed char aufs_bindex_t;
#define AUFS_BRANCH_MAX 127
#else
typedef short aufs_bindex_t;
#ifdef CONFIG_AUFS_BRANCH_MAX_511
#define AUFS_BRANCH_MAX 511
#elif defined(CONFIG_AUFS_BRANCH_MAX_1023)
#define AUFS_BRANCH_MAX 1023
#elif defined(CONFIG_AUFS_BRANCH_MAX_32767)
#define AUFS_BRANCH_MAX 32767
#endif
#endif

#ifdef __KERNEL__
#ifndef AUFS_BRANCH_MAX
#error unknown CONFIG_AUFS_BRANCH_MAX value
#endif
#endif /* __KERNEL__ */

/* ---------------------------------------------------------------------- */

#define AUFS_NAME		"aufs"
#define AUFS_FSTYPE		AUFS_NAME

#define AUFS_ROOT_INO		2
#define AUFS_FIRST_INO		11

#define AUFS_WH_PFX		".wh."
#define AUFS_WH_PFX_LEN		((int)sizeof(AUFS_WH_PFX) - 1)
#define AUFS_XINO_FNAME		"." AUFS_NAME ".xino"
#define AUFS_XINO_DEFPATH	"/tmp/" AUFS_XINO_FNAME
#define AUFS_XINO_TRUNC_INIT	64 /* blocks */
#define AUFS_XINO_TRUNC_STEP	4  /* blocks */
#define AUFS_DIRWH_DEF		3
#define AUFS_RDCACHE_DEF	10 /* seconds */
#define AUFS_WKQ_NAME		AUFS_NAME "d"
#define AUFS_NWKQ_DEF		4
#define AUFS_MFS_SECOND_DEF	30 /* seconds */
#define AUFS_PLINK_WARN		100 /* number of plinks */

#define AUFS_DIROPQ_NAME	AUFS_WH_PFX ".opq" /* whiteouted doubly */
#define AUFS_WH_DIROPQ		AUFS_WH_PFX AUFS_DIROPQ_NAME

#define AUFS_BASE_NAME		AUFS_WH_PFX AUFS_NAME
#define AUFS_PLINKDIR_NAME	AUFS_WH_PFX "plnk"
#define AUFS_ORPHDIR_NAME	AUFS_WH_PFX "orph"

/* doubly whiteouted */
#define AUFS_WH_BASE		AUFS_WH_PFX AUFS_BASE_NAME
#define AUFS_WH_PLINKDIR	AUFS_WH_PFX AUFS_PLINKDIR_NAME
#define AUFS_WH_ORPHDIR		AUFS_WH_PFX AUFS_ORPHDIR_NAME

/* branch permission */
#define AUFS_BRPERM_RW		"rw"
#define AUFS_BRPERM_RO		"ro"
#define AUFS_BRPERM_RR		"rr"
#define AUFS_BRPERM_WH		"wh"
#define AUFS_BRPERM_NLWH	"nolwh"
#define AUFS_BRPERM_ROWH	AUFS_BRPERM_RO "+" AUFS_BRPERM_WH
#define AUFS_BRPERM_RRWH	AUFS_BRPERM_RR "+" AUFS_BRPERM_WH
#define AUFS_BRPERM_RWNLWH	AUFS_BRPERM_RW "+" AUFS_BRPERM_NLWH

/* ---------------------------------------------------------------------- */

/* ioctl */
enum {
	AuCtl_PLINK_MAINT,
	AuCtl_PLINK_CLEAN
};

#define AuCtlType		'A'
#define AUFS_CTL_PLINK_MAINT	_IO(AuCtlType, AuCtl_PLINK_MAINT)
#define AUFS_CTL_PLINK_CLEAN	_IO(AuCtlType, AuCtl_PLINK_CLEAN)

#endif /* __AUFS_TYPE_H__ */

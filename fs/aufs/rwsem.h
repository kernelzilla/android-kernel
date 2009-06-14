/*
 * Copyright (C) 2005-2009 Junjiro R. Okajima
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
 * simple read-write semaphore wrappers
 */

#ifndef __AUFS_RWSEM_H__
#define __AUFS_RWSEM_H__

#ifdef __KERNEL__

#include <linux/rwsem.h>

#define au_rwsem_destroy(rw)	AuDebugOn(rwsem_is_locked(rw))
#define AuRwMustNoWaiters(rw)	AuDebugOn(!list_empty(&(rw)->wait_list))

#define AuSimpleLockRwsemFuncs(prefix, param, rwsem) \
static inline void prefix##_read_lock(param) \
{ down_read(rwsem); } \
static inline void prefix##_write_lock(param) \
{ down_write(rwsem); } \
static inline int prefix##_read_trylock(param) \
{ return down_read_trylock(rwsem); } \
static inline int prefix##_write_trylock(param) \
{ return down_write_trylock(rwsem); }
/* why is not _nested version defined */
/* static inline void prefix##_read_trylock_nested(param, lsc)
{ down_write_trylock_nested(rwsem, lsc)); }
static inline void prefix##_write_trylock_nestd(param, lsc)
{ down_write_trylock_nested(rwsem, lsc); } */

#define AuSimpleUnlockRwsemFuncs(prefix, param, rwsem) \
static inline void prefix##_read_unlock(param) \
{ up_read(rwsem); } \
static inline void prefix##_write_unlock(param) \
{ up_write(rwsem); } \
static inline void prefix##_downgrade_lock(param) \
{ downgrade_write(rwsem); }

#define AuSimpleRwsemFuncs(prefix, param, rwsem) \
	AuSimpleLockRwsemFuncs(prefix, param, rwsem) \
	AuSimpleUnlockRwsemFuncs(prefix, param, rwsem)

#endif /* __KERNEL__ */
#endif /* __AUFS_RWSEM_H__ */

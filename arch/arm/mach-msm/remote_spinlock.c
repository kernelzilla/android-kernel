/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/err.h>
#include <linux/kernel.h>

#include <asm/system.h>

/* This is ugly but it has to be done. If linux/spinlock_types is included,
 * then for a UP kernel, the spinlock will get stubbed out. Since this is a
 * remote spin lock, stubbing out is not the right thing to do.
 */
#define __LINUX_SPINLOCK_TYPES_H
#include <asm/spinlock_types.h>
#undef __LINUX_SPINLOCK_TYPES_H

#include <asm/spinlock.h>

#include <mach/remote_spinlock.h>
#include "smd_private.h"

#define SMEM_SPINLOCK_COUNT 8
#define SMEM_SPINLOCK_ARRAY_SIZE (SMEM_SPINLOCK_COUNT * sizeof(uint32_t))

int _remote_spin_lock_init(remote_spin_lock_id_t id, _remote_spinlock_t *lock)
{
	_remote_spinlock_t spinlock_start;

	/* The raw_spinlock_t structure should be the same as
	 * raw_remote_spinlock_t to be able to reuse the __raw_spin_lock()
	 * and __raw_spin_unlock() functions. If this condition is not met,
	 * then please write new code to replace calls to __raw_spin_lock()
	 * and __raw_spin_unlock(). */
	BUILD_BUG_ON(sizeof(raw_remote_spinlock_t) != sizeof(raw_spinlock_t));

	if (id >= SMEM_SPINLOCK_COUNT)
		return -EINVAL;

	spinlock_start = smem_alloc(SMEM_SPINLOCK_ARRAY,
				    SMEM_SPINLOCK_ARRAY_SIZE);
	if (spinlock_start == NULL)
		return -ENXIO;

	*lock = spinlock_start + id;

	return 0;
}

void _remote_spin_lock(_remote_spinlock_t *lock)
{
	__raw_spin_lock((raw_spinlock_t *) (*lock));
}

void _remote_spin_unlock(_remote_spinlock_t *lock)
{
	__raw_spin_unlock((raw_spinlock_t *) (*lock));
}

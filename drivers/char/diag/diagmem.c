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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <asm/atomic.h>
#include "diagchar.h"

void *diagmem_alloc(struct diagchar_dev *driver, int size, int pool_type)
{
	void *buf = NULL;

	if (pool_type == POOL_TYPE_COPY) {
		mutex_lock(&driver->diagmem_mutex);
		if (driver->count < driver->poolsize) {
			atomic_add(1, (atomic_t *)&driver->count);
			buf = mempool_alloc(driver->diagpool, GFP_ATOMIC);
		}
		mutex_unlock(&driver->diagmem_mutex);
	} else if (pool_type == POOL_TYPE_HDLC) {
		if (driver->count_hdlc_pool < driver->poolsize_hdlc) {
			atomic_add(1, (atomic_t *)&driver->count_hdlc_pool);
			buf = mempool_alloc(driver->diag_hdlc_pool, GFP_ATOMIC);
		}
	}
	return buf;
}

void diagmem_exit(struct diagchar_dev *driver)
{
	if (driver->diagpool) {
		if (driver->count == 0 && driver->ref_count == 0) {
			mempool_destroy(driver->diagpool);
			driver->diagpool = NULL;
		}
	} else
		printk(KERN_ALERT "\n Attempt to free up"
					"non existing COPY pool");

	if (driver->diag_hdlc_pool) {
		if (driver->count_hdlc_pool == 0 && driver->ref_count == 0) {
			mempool_destroy(driver->diag_hdlc_pool);
			driver->diag_hdlc_pool = NULL;
		}
	} else
		printk(KERN_ALERT "\n Attempt to free up"
					 "non existing HDLC pool");
}

void diagmem_free(struct diagchar_dev *driver, void *buf, int pool_type)
{
	if (pool_type == POOL_TYPE_COPY) {
		if (driver->diagpool != NULL && driver->count > 0) {
			mempool_free(buf, driver->diagpool);
			atomic_add(-1, (atomic_t *)&driver->count);
		} else
			printk(KERN_ALERT "\n Attempt to free up DIAG driver"
	       "mempool memory which is already free %d", driver->count);
	} else if (pool_type == POOL_TYPE_HDLC) {
		if (driver->diag_hdlc_pool != NULL &&
			 driver->count_hdlc_pool > 0) {
			mempool_free(buf, driver->diag_hdlc_pool);
			atomic_add(-1, (atomic_t *)&driver->count_hdlc_pool);
		} else
			printk(KERN_ALERT "\n Attempt to free up DIAG driver "
	"HDLC mempool which is already free %d ", driver->count_hdlc_pool);
	}

	diagmem_exit(driver);
}

void diagmem_init(struct diagchar_dev *driver)
{
	mutex_init(&driver->diagmem_mutex);
	driver->diagpool = mempool_create_kmalloc_pool(driver->poolsize,
						       driver->itemsize);
	driver->diag_hdlc_pool = mempool_create_kmalloc_pool(
	driver->poolsize_hdlc, driver->itemsize_hdlc);

	if (!driver->diagpool)
		printk(KERN_INFO "Cannot allocate diag mempool\n");

	if (!driver->diag_hdlc_pool)
		printk(KERN_INFO "Cannot allocate diag HDLC mempool\n");
}


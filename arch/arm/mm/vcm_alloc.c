/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/vcm.h>
#include <linux/vcm_alloc.h>
#include <linux/string.h>
#include <asm/sizes.h>

int basicalloc_init;

int chunk_sizes[NUM_CHUNK_SIZES] = {SZ_16M, SZ_1M, SZ_64K, SZ_4K};

#define LAST_SZ() (ARRAY_SIZE(chunk_sizes) - 1)
#define vcm_alloc_err(a, ...)						\
	pr_err("ERROR %s %i " a, __func__, __LINE__, ##__VA_ARGS__)

struct phys_chunk_head {
	struct list_head head;
	int num;
};

struct phys_pool {
	int size;
	struct phys_chunk_head *heads[ARRAY_SIZE(chunk_sizes)];
};

static struct phys_pool *vcm_phys_pool;
static struct vcm_memtype_map *memtype_map;

static struct phys_chunk_head *get_chunk_list(enum memtype_t memtype,
					      int chunk_size_idx)
{
	unsigned int pool_id;

	if (chunk_size_idx >= ARRAY_SIZE(chunk_sizes)) {
		vcm_alloc_err("bad chunk size\n");
		return NULL;
	}

	if (!vcm_phys_pool) {
		vcm_alloc_err("phys_pool is null\n");
		return NULL;
	}

	if (memtype >= NUM_MEMTYPES) {
		vcm_alloc_err("Bad memtype: %d\n", memtype);
		return NULL;
	}

	/* We don't have a "pool count" anywhere but this is coming
	 * strictly from data in a board file
	 */
	pool_id = memtype_map[memtype].pool_id[chunk_size_idx];

	return vcm_phys_pool[pool_id].heads[chunk_size_idx];
}

static int is_allocated(struct list_head *allocated)
{
	/* This should not happen under normal conditions */
	if (!allocated) {
		vcm_alloc_err("no allocated\n");
		return 0;
	}

	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		return 0;
	}
	return !list_empty(allocated);
}

static int count_allocated_size(enum memtype_t memtype, enum chunk_size_idx idx)
{
	int cnt = 0;
	struct phys_chunk *chunk, *tmp;
	struct phys_chunk_head *pch;

	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		return 0;
	}

	pch = get_chunk_list(memtype, idx);
	if (!pch) {
		vcm_alloc_err("null pch\n");
		return -1;
	}

	list_for_each_entry_safe(chunk, tmp, &pch->head, list) {
		if (is_allocated(&chunk->allocated))
			cnt++;
	}

	return cnt;
}


int vcm_alloc_get_mem_size(void)
{
	if (!vcm_phys_pool) {
		vcm_alloc_err("No physical pool set up!\n");
		return -1;
	}
	return vcm_phys_pool[0].size;
}
EXPORT_SYMBOL(vcm_alloc_get_mem_size);

void vcm_alloc_print_list(enum memtype_t memtype, int just_allocated)
{
	int i;
	struct phys_chunk *chunk, *tmp;
	struct phys_chunk_head *pch;

	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(chunk_sizes); ++i) {
		pch = get_chunk_list(memtype, i);

		if (!pch) {
			vcm_alloc_err("pch is null\n");
			return;
		}

		if (list_empty(&pch->head))
			continue;

		list_for_each_entry_safe(chunk, tmp, &pch->head, list) {
			if (just_allocated && !is_allocated(&chunk->allocated))
				continue;

			printk(KERN_INFO "pa = %#x, size = %#x\n",
			chunk->pa, chunk_sizes[chunk->size_idx]);
		}
	}
}
EXPORT_SYMBOL(vcm_alloc_print_list);

int vcm_alloc_blocks_avail(enum memtype_t memtype, enum chunk_size_idx idx)
{
	struct phys_chunk_head *pch;
	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		return 0;
	}
	pch = get_chunk_list(memtype, idx);

	if (!pch) {
		vcm_alloc_err("pch is null\n");
		return 0;
	}
	return pch->num;
}
EXPORT_SYMBOL(vcm_alloc_blocks_avail);


int vcm_alloc_get_num_chunks(void)
{
	return ARRAY_SIZE(chunk_sizes);
}
EXPORT_SYMBOL(vcm_alloc_get_num_chunks);


int vcm_alloc_all_blocks_avail(enum memtarget_t memtype)
{
	int i;
	int cnt = 0;

	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(chunk_sizes); ++i)
		cnt += vcm_alloc_blocks_avail(memtype, i);
	return cnt;
}
EXPORT_SYMBOL(vcm_alloc_all_blocks_avail);


int vcm_alloc_count_allocated(enum memtype_t memtype)
{
	int i;
	int cnt = 0;

	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(chunk_sizes); ++i)
		cnt += count_allocated_size(memtype, i);
	return cnt;
}
EXPORT_SYMBOL(vcm_alloc_count_allocated);


int vcm_alloc_idx_to_size(int idx)
{
	return chunk_sizes[idx];
}
EXPORT_SYMBOL(vcm_alloc_idx_to_size);


int vcm_alloc_destroy(void)
{
	int i, mt;
	struct phys_chunk *chunk, *tmp;

	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		return -1;
	}

	/* can't destroy a space that has allocations */
	for (mt = 0; mt < NUM_MEMTYPES; mt++)
		if (vcm_alloc_count_allocated(mt)) {
			vcm_alloc_err("allocations still present\n");
			return -1;
		}

	for (mt = 0; mt < NUM_MEMTYPES; mt++) {
		for (i = 0; i < ARRAY_SIZE(chunk_sizes); ++i) {
			struct phys_chunk_head *pch;
			pch = vcm_phys_pool[mt].heads[i];

			if (!pch) {
				vcm_alloc_err("pch is null\n");
				return -1;
			}

			if (list_empty(&pch->head))
				continue;
			list_for_each_entry_safe(chunk, tmp, &pch->head, list) {
				list_del(&chunk->list);
				memset(chunk, 0, sizeof(*chunk));
				kfree(chunk);
			}
			kfree(pch);
			vcm_phys_pool[mt].heads[i] = NULL;

		}
	}
	kfree(vcm_phys_pool);
	kfree(memtype_map);

	vcm_phys_pool = NULL;
	memtype_map = NULL;
	basicalloc_init = 0;

	return 0;
}
EXPORT_SYMBOL(vcm_alloc_destroy);


int vcm_alloc_init(struct physmem_region *mem, int n_regions,
		   struct vcm_memtype_map *mt_map, int n_mt)
{
	int i = 0, j = 0, r = 0, num_chunks;
	struct phys_chunk *chunk;
	struct phys_chunk_head *pch = NULL;
	unsigned long pa;

	/* no double inits */
	if (basicalloc_init) {
		vcm_alloc_err("double basicalloc_init\n");
		BUG();
		goto fail;
	}
	memtype_map = kzalloc(sizeof(*mt_map) * n_mt, GFP_KERNEL);
	if (!memtype_map) {
		vcm_alloc_err("Could not copy memtype map\n");
		goto fail;
	}
	memcpy(memtype_map, mt_map, sizeof(*mt_map) * n_mt);

	vcm_phys_pool = kzalloc(sizeof(*vcm_phys_pool) * n_regions, GFP_KERNEL);

	if (!vcm_phys_pool) {
		vcm_alloc_err("Could not allocate physical pool structure\n");
		goto fail;
	}

	/* separate out to ensure good cleanup */
	for (j = 0; j < n_regions; j++) {
		for (i = 0; i < ARRAY_SIZE(chunk_sizes); ++i) {

			pch = kzalloc(sizeof(struct phys_chunk_head),
								    GFP_KERNEL);
			if (!pch) {
				vcm_alloc_err("could not malloc pch\n");
				goto fail;
			}
			vcm_phys_pool[j].heads[i] = pch;

			INIT_LIST_HEAD(&pch->head);
			pch->num = 0;
		}
	}

	for (r = 0; r < n_regions; r++) {
		pa = mem[r].addr;
		vcm_phys_pool[r].size = mem[r].size;
		for (i = 0; i < ARRAY_SIZE(chunk_sizes); ++i) {
			pch = vcm_phys_pool[r].heads[i];

			/* A fraction of 0 means don't use that chunk size */
			if (mem[r].chunk_fraction[i] > 0)
				num_chunks = mem[r].size /
				      mem[r].chunk_fraction[i] / chunk_sizes[i];
			else
				num_chunks = 0;

			printk(KERN_INFO "VCM Init: region %d, chunk size=%d, "
			       "num=%d, pa=%p\n", r, chunk_sizes[i], num_chunks,
			       (void *)pa);

			for (j = 0; j < num_chunks; ++j) {
				chunk = kzalloc(sizeof(*chunk), GFP_KERNEL);
				if (!chunk) {
					vcm_alloc_err("null chunk\n");
					goto fail;
				}
				chunk->pa = pa; pa += chunk_sizes[i];
				chunk->size_idx = i;
				INIT_LIST_HEAD(&chunk->allocated);
				list_add_tail(&chunk->list, &pch->head);
				pch->num++;
			}
		}
	}

	basicalloc_init = 1;
	return 0;
fail:
	vcm_alloc_destroy();
	return -1;
}
EXPORT_SYMBOL(vcm_alloc_init);


int vcm_alloc_free_blocks(enum memtype_t memtype, struct phys_chunk *alloc_head)
{
	struct phys_chunk *chunk, *tmp;
	struct phys_chunk_head *pch = NULL;

	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		goto fail;
	}

	if (!alloc_head) {
		vcm_alloc_err("no alloc_head\n");
		goto fail;
	}

	list_for_each_entry_safe(chunk, tmp, &alloc_head->allocated,
				 allocated) {
		list_del_init(&chunk->allocated);
		pch = get_chunk_list(memtype, chunk->size_idx);
		if (!pch) {
			vcm_alloc_err("null pch\n");
			goto fail;
		}
		pch->num++;
	}

	return 0;
fail:
	return -1;
}
EXPORT_SYMBOL(vcm_alloc_free_blocks);


int vcm_alloc_num_blocks(int num, enum memtype_t memtype,
			 enum chunk_size_idx idx, /* chunk size */
			 struct phys_chunk *alloc_head)
{
	struct phys_chunk *chunk;
	struct phys_chunk_head *pch = NULL;
	int num_allocated = 0;

	if (!basicalloc_init) {
		vcm_alloc_err("no basicalloc_init\n");
		goto fail;
	}

	if (!alloc_head) {
		vcm_alloc_err("no alloc_head\n");
		goto fail;
	}

	pch = get_chunk_list(memtype, idx);

	if (!pch) {
		vcm_alloc_err("null pch\n");
		goto fail;
	}
	if (list_empty(&pch->head)) {
		vcm_alloc_err("list is empty\n");
		goto fail;
	}

	if (vcm_alloc_blocks_avail(memtype, idx) < num) {
		vcm_alloc_err("not enough blocks? num=%d\n", num);
		goto fail;
	}

	list_for_each_entry(chunk, &pch->head, list) {
		if (num_allocated == num)
			break;
		if (is_allocated(&chunk->allocated))
			continue;

		list_add_tail(&chunk->allocated, &alloc_head->allocated);
		pch->num--;
		num_allocated++;
	}
	return num_allocated;
fail:
	return 0;
}
EXPORT_SYMBOL(vcm_alloc_num_blocks);


int vcm_alloc_max_munch(int len, enum memtype_t memtype,
			struct phys_chunk *alloc_head)
{
	int i;

	int blocks_req = 0;
	int block_residual = 0;
	int blocks_allocated = 0;

	int ba = 0;

	if (!basicalloc_init) {
		vcm_alloc_err("basicalloc_init is 0\n");
		goto fail;
	}

	if (!alloc_head) {
		vcm_alloc_err("alloc_head is NULL\n");
		goto fail;
	}

	for (i = 0; i < ARRAY_SIZE(chunk_sizes); ++i) {
		blocks_req = len / chunk_sizes[i];
		block_residual = len % chunk_sizes[i];

		len = block_residual; /* len left */
		if (blocks_req) {
			int blocks_available = 0;
			int blocks_diff = 0;
			int bytes_diff = 0;

			blocks_available = vcm_alloc_blocks_avail(memtype, i);
			if (blocks_available < blocks_req) {
				blocks_diff =
					(blocks_req - blocks_available);
				bytes_diff =
					blocks_diff * chunk_sizes[i];

				/* add back in the rest */
				len += bytes_diff;
			} else {
				/* got all the blocks I need */
				blocks_available =
					(blocks_available > blocks_req)
					? blocks_req : blocks_available;
			}

			ba = vcm_alloc_num_blocks(blocks_available, memtype, i,
						  alloc_head);

			if (ba != blocks_available) {
				vcm_alloc_err("blocks allocated (%i) !="
					      " blocks_available (%i):"
					      " chunk size = %#x,"
					      " alloc_head = %p\n",
					      ba, blocks_available,
					      i, (void *) alloc_head);
				goto fail;
			}
			blocks_allocated += blocks_available;
		}
	}

	if (len) {
		int blocks_available = 0;

		blocks_available = vcm_alloc_blocks_avail(memtype, LAST_SZ());

		if (blocks_available > 1) {
			ba = vcm_alloc_num_blocks(1, memtype, LAST_SZ(),
						  alloc_head);
			if (ba != 1) {
				vcm_alloc_err("blocks allocated (%i) !="
					      " blocks_available (%i):"
					      " chunk size = %#x,"
					      " alloc_head = %p\n",
					      ba, 1,
					      LAST_SZ(),
					      (void *) alloc_head);
				goto fail;
			}
			blocks_allocated += 1;
		} else {
			vcm_alloc_err("blocks_available (%#x) <= 1\n",
				      blocks_available);
			goto fail;
		}
	}

	return blocks_allocated;
fail:
	vcm_alloc_free_blocks(memtype, alloc_head);
	return 0;
}
EXPORT_SYMBOL(vcm_alloc_max_munch);

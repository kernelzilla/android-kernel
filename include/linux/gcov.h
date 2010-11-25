/*
 * include/linux/gcov.h
 *
 * Type declarations and macros used by GCOV kernel profiling.
 *
 * This program is free software; you can redistribute it and/or modify
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
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (c) International Business Machines Corp., 2002-2003
 *
 * Author: Hubertus Franke <frankeh@us.ibm.com>
 *         Peter Oberparleiter <peter.oberparleiter@de.ibm.com>
 */

#ifndef GCOV_H
#define GCOV_H GCOV_H

#include <linux/module.h>
#include <linux/semaphore.h>
#include <asm/types.h>

#define GCC_VERSION_LOWER(major, minor) ((__GNUC__ < major) || \
					 (__GNUC__ == major) && \
					 (__GNUC_MINOR__ < minor))
#define GCC_VERSION_EQUAL(major, minor)	 (__GNUC__ == major) && \
					 (__GNUC_MINOR__ == minor)

#if GCC_VERSION_LOWER(3, 1)

/*
 * Profiling types for GCC prior to version 3.1
 */

typedef long gcov_type;

/* Same as gcc/libgcc2.c */
struct bb
{
	long zero_word;
	const char *filename;
	long *counts;
	long ncounts;
	struct bb *next;
	const unsigned long *addresses;
	long nwords;
	const char **functions;
	const long *line_nums;
	const char **filenames;
	char *flags;
};

#elif GCC_VERSION_LOWER(3, 3)

/*
 * Profiling types for GCC 3.1 to 3.2
 */

#if BITS_PER_LONG >= 64
typedef long gcov_type;
#else
typedef long long gcov_type;
#endif

/* Same as gcc/libgcc2.c */
struct bb
{
	long zero_word;
	const char *filename;
	gcov_type *counts;
	long ncounts;
	struct bb *next;
	const unsigned long *addresses;
	long nwords;
	const char **functions;
	const long *line_nums;
	const char **filenames;
	char *flags;
};

#elif GCC_VERSION_EQUAL(3, 3) && CONFIG_GCOV_HAMMER

/*
 * Profiling types for modified GCC 3.3 (hammer patch)
 */

#define GCOV_DATA_MAGIC		0x67636f76
#define GCOV_TAG_FUNCTION	0x01000000


#if BITS_PER_LONG >= 64
typedef long gcov_type;
#else
typedef long long gcov_type;
#endif

struct counter_section
{
	unsigned int tag;
	unsigned int n_counters;
};

struct counter_section_data
{
	unsigned int tag;
	unsigned int n_counters;
	gcov_type *counters;
};

struct function_info
{
	const char *name;
	unsigned int checksum;
	unsigned int n_counter_sections;
	const struct counter_section *counter_sections;
};

struct bb /* should be 'struct gcov_info' but we're sticking with the old name
	   * so we can reuse some of our pre-3.4 functions */
{
	unsigned long version;
	struct bb *next;
	const char *filename;
	long wkspc;
	unsigned int n_functions;
	const struct function_info *functions;
	unsigned int n_counter_sections;
	const struct counter_section_data *counter_sections;
};

extern unsigned long gcov_version;
#elif GCC_VERSION_LOWER(3, 4)

/*
 * Profiling types for vanilla GCC 3.3
 */

typedef long long gcov_type;

/* Same as gcc/libgcc2.c */
struct bb_function_info
{
	long checksum;
	int arc_count;
	const char *name;
};

struct bb
{
	long zero_word;
	const char *filename;
	gcov_type *counts;
	long ncounts;
	struct bb *next;
	long sizeof_bb;
	struct bb_function_info *function_infos;
};

#else /* GCC_VERSION */

/*
 * Profiling types for GCC 3.4 and above (see gcc-3.4/gcc/gcov-io.h)
 */

#define GCOV_COUNTERS		5
#define GCOV_DATA_MAGIC		((gcov_unsigned_t) 0x67636461)
#define GCOV_TAG_FUNCTION	((gcov_unsigned_t) 0x01000000)
#define GCOV_TAG_COUNTER_BASE	((gcov_unsigned_t) 0x01a10000)
#define GCOV_TAG_FOR_COUNTER(COUNT)					\
	(GCOV_TAG_COUNTER_BASE + ((gcov_unsigned_t) (COUNT) << 17))

#if BITS_PER_LONG >= 64
typedef long gcov_type;
#else
typedef long long gcov_type;
#endif

typedef unsigned int gcov_unsigned_t;
typedef unsigned int gcov_position_t;

typedef void (*gcov_merge_fn) (gcov_type *, gcov_unsigned_t);

struct gcov_fn_info
{
	gcov_unsigned_t ident;
	gcov_unsigned_t checksum;
	unsigned int n_ctrs[0];			/* Note: the number of bits
						 * set in bb->ctr_mask decides
						 * how big this array is. */
};

struct gcov_ctr_info
{
	gcov_unsigned_t num;
	gcov_type *values;
	gcov_merge_fn merge;
};

struct bb /* should be 'struct gcov_info' but we're sticking with the old name
	   * so we can reuse some of our pre-3.4 functions */
{
	gcov_unsigned_t version;
	struct bb *next;
	gcov_unsigned_t stamp;
	const char *filename;
	unsigned int n_functions;
	const struct gcov_fn_info *functions;
	unsigned int ctr_mask;
	struct gcov_ctr_info counts[0];		/* Note: the number of bits
						 * set in ctr_mask decides
						 * how big this array is. */
};

extern gcov_unsigned_t gcov_version;
#endif /* GCC_VERSION */

enum gcov_cmd {
	gcov_add,
	gcov_remove
};

typedef void (*ctor_t)(void);

extern struct bb *bb_head;
extern const char *gcov_sourcepath;
extern const char *gcov_objectpath;
extern struct semaphore gcov_core_lock;

extern void do_global_ctors(ctor_t[], unsigned long, struct module *);
extern void (*gcov_callback)(enum gcov_cmd, struct bb *);
extern void remove_bb_link(struct module *);

#endif /* GCOV_H */

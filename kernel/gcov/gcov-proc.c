/*
 * kernel/gcov/gcov-proc.c
 *
 * Provides proc filesystem entry for GCOV kernel profiling.
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
 *         Rajan Ravindran <rajancr@us.ibm.com>
 *         Peter Oberparleiter <Peter.Oberparleiter@de.ibm.com>
 *         Paul Larson
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/gcov.h>


MODULE_LICENSE("GPL");

#define GCOV_PROC_HEADER	"gcov-proc: "
#define GCOV_PROC_ROOT		"gcov"
#define GCOV_PROC_MODULE	"module"
#define GCOV_PROC_VMLINUX	"vmlinux"
#define PAD8(x)			(((x) + 7) & ~7)
#define PAD4(x)			(((x) + 3) & ~3)

typedef enum {
	status_normal,	/* Normal status */
	status_ghost	/* Module associated with this node has been unloaded
			 * but data was saved. */
} node_status;

/* Data structure used to manage proc filesystem entries. */
struct gcov_ftree_node
{
	char *fname;			 /* Hierarchy-relative name */
	struct gcov_ftree_node *sibling; /* First sibling of this node */
	struct gcov_ftree_node *files;	 /* First child of this node */
	struct gcov_ftree_node *parent;	 /* Parent of this node */
	struct proc_dir_entry *proc[4];	 /* Entries for .da, .bb, .bbg, .c */
	struct bb *bb;			 /* Associated struct bb */
	loff_t offset;			 /* Offset in vmlinux file */
	size_t da_size;			 /* Size of associated .da file */
	size_t header_size;		 /* Size of associated file header */
	struct gcov_ftree_node *next;	 /* Next leaf node */
	node_status status;		 /* Status of this node */
};


/* If set to non-zero, keep gcov data for modules after unload. */
static int gcov_persist = 0;

/* If set to non-zero, create links to additional files in proc filesystem
 * entries. */
static int gcov_link = 1;

/* Protect global variables from concurrent access. */
static struct semaphore gcov_lock;

/* Length of kernel source path string. */
static int sourcepath_len;

/* Length of kernel object path string. */
static int objectpath_len;

/* Filesystem entry for /proc/gcov/vmlinux */
static struct proc_dir_entry *proc_vmlinux = NULL;

/* First leaf node. */
static struct gcov_ftree_node *leaf_nodes = NULL;

/* Cached node used to speed up sequential reads in /proc/vmlinux. */
static struct gcov_ftree_node *cached_node = NULL;

/* Root node for internal data tree. */
static struct gcov_ftree_node tree_root;

#if GCC_VERSION_LOWER(3,4) && !CONFIG_GCOV_HAMMER
/* Filename extension for data files. */
static const char *da_ending = "da";

/* Array of filename endings to use when creating links. */
static const char *endings[] = { "bb", "bbg", "c" };
#elif GCC_VERSION_EQUAL(3,3) && CONFIG_GCOV_HAMMER
/* Filename extension for data files. */
static const char *da_ending = "da";

/* Array of filename endings to use when creating links. */
static const char *endings[] = { "bbg", "c" };
#else /* GCC_VERSION */
/* Filename extension for data files. */
static const char *da_ending = "gcda";

/* Array of filename endings to use when creating links. */
static const char *endings[] = { "gcno", "c" };
#endif /* GCC_VERSION_LOWER */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)) &&	\
    (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,4)) ||	\
    (LINUX_VERSION_CODE < KERNEL_VERSION(2,4,23))
/* Retrieve proc_dir_entry associated with INODE. */
static inline struct proc_dir_entry *
PDE(const struct inode *inode)
{
	return ((struct proc_dir_entry *) inode->u.generic_ip);
}
#endif


#ifdef MODULE
/* Parameter handling. */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
/* module_param is a 2.6 thing (though there are compat macros since 2.4.25) */
MODULE_PARM(gcov_persist, "i");
MODULE_PARM(gcov_link, "i");
#else
module_param(gcov_persist, int, 0400);
module_param(gcov_link, int, 0400);
#endif /* LINUX_VERSION_CODE */

MODULE_PARM_DESC(gcov_persist, "If set to non-zero, keep gcov data for modules "
			       "after unload");
MODULE_PARM_DESC(gcov_link, "If set to non-zero, create links to additional "
			    "files in proc filesystem entries");

#else /* MODULE */

/* Called when 'gcov_persist' is specified on the kernel command line. */
static int __init
gcov_persist_setup(char *str)
{
	gcov_persist = (int) simple_strtol(str, NULL, 10);
	return 1;
}

__setup("gcov_persist=", gcov_persist_setup);


/* Called when 'gcov_link' is specified on the kernel command line. */
static int __init
gcov_link_setup(char *str)
{
	gcov_link = (int) simple_strtol(str, NULL, 10);
	return 1;
}

__setup("gcov_link=", gcov_link_setup);
#endif /* MODULE */


/* Store a portable representation of VALUE in DEST using BYTES*8-1 bits.
 * Return a non-zero value if VALUE requires more than BYTES*8-1 bits
 * to store (adapted from gcc/gcov-io.h). */
static int
store_gcov_type(gcov_type value, char *dest, size_t bytes)
{
	int upper_bit = (value < 0 ? 128 : 0);
	size_t i;

	if (value < 0) {
		gcov_type oldvalue = value;
		value = -value;
		if (oldvalue != -value)
			return 1;
	}

	for (i = 0 ; i < (sizeof(value) < bytes ? sizeof(value) : bytes) ;
	     i++) {
		dest[i] = value & (i == (bytes - 1) ? 127 : 255);
		value = value / 256;
	}

	if (value && value != -1)
		return 1;

	for(; i < bytes ; i++)
		dest[i] = 0;
	dest[bytes - 1] |= upper_bit;
	return 0;
}


/* Return size of header which precedes .da file entry associated with BB
 * in the vmlinux file. */
static inline size_t
sizeof_vmlinux_header(struct bb *bb)
{
	return 8 + PAD8(strlen(bb->filename) + 1);
}


/* Store data of header which precedes .da file entry associated with NODE
 * in the vmlinux file to userspace memory at BUF. OFFSET specifies the offset
 * inside the header file. COUNT is the maximum number of bytes to store.
 * Return the number of bytes stored, zero for EOF or a negative number in
 * case of error. */
static ssize_t
store_vmlinux_header(struct gcov_ftree_node *node, char *buf, size_t count,
		     loff_t offset)
{
	char data[8];
	char *from;
	size_t namelen;
	ssize_t stored;
	size_t len;

	namelen = strlen(node->bb->filename);
	stored = 0;
	while (count > 0) {
		if (offset < 8) {
			/* Filename length */
			if (store_gcov_type(PAD8(namelen + 1), data, 8))
				return -EINVAL;
			from = data + offset;
			len = 8 - offset;
		} else if (offset < 8 + namelen) {
			/* Filename */
			from = (char *) node->bb->filename + offset - 8;
			len = namelen - (offset - 8);
		} else if (offset < node->header_size) {
			/* Nil byte padding */
			memset(data, 0, 8);
			from = data;
			len = PAD8(namelen + 1) - (offset - 8);
		} else
			break;
		if (len > count)
			len = count;
		if (copy_to_user(buf, from, len))
			return -EFAULT;
		stored += len;
		count -= len;
		offset += len;
		buf += len;
	}

	return stored;
}


#if GCC_VERSION_LOWER(3, 3)
/*
 *  pre-gcc 3.3 functions
 */
static const char* gcov_proc_format = "pre-gcc 3.3";


/* Return size of .da file associated with BB. */
static inline size_t
sizeof_da_file(struct bb *bb)
{
	return (bb->ncounts + 1) * 8;
}


/* Store data of .da file associated with NODE to userspace memory at BUF.
 * OFFSET specifies the offset inside the .da file. COUNT is the maximum
 * number of bytes to store. Return the number of bytes stored, zero for
 * EOF or a negative number in case of error. */
static ssize_t
store_da_file(struct gcov_ftree_node *node, char *buf, size_t count,
	      loff_t offset)
{
	char data[8];
	char *from;
	ssize_t stored;
	size_t len;

	stored = 0;
	while (count > 0) {
		if (offset < 8) {
			/* Number of counts */
			if (store_gcov_type(node->bb->ncounts, data, 8))
				return -EINVAL;
			from = data + offset;
			len = 8 - offset;
		} else if (offset < node->da_size) {
			/* Count data */
			if (store_gcov_type(node->bb->counts[(offset - 8) / 8],
					    data, 8))
				return -EINVAL;
			from = data + offset % 8;
			len = 8 - offset % 8;
		} else
			break;
		if (len > count)
			len = count;
		if (copy_to_user(buf, from, len))
			return -EFAULT;
		stored += len;
		count -= len;
		offset += len;
		buf += len;
	}
	return stored;
}

#elif GCC_VERSION_EQUAL(3, 3) && CONFIG_GCOV_HAMMER

/*
 *   gcc 3.3 specific functions (hammer patch)
 */
static const char* gcov_proc_format = "gcc 3.3 (hammer)";


/* Return length of string encoded in GCOV format. */
static size_t
sizeof_str(const char *str)
{
	size_t len;

	len = strlen(str);
	if (len == 0)
		return 4;

	return 4 + len + 4 - (len & 3);
}


/* Return length of counter section encoded in GCOV format. */
static size_t
sizeof_counter_data(const struct counter_section *section)
{
	return /* tag */ 4 + /* length */ 4 +
	       /* counters */ section->n_counters * 8;
}


/* Return length of function section encoded in GCOV format. */
static size_t
sizeof_func_data(const struct function_info *fn)
{
	size_t sec_len;
	unsigned int i;

	sec_len = 0;
	for (i=0; i < fn->n_counter_sections; i++)
		sec_len += sizeof_counter_data(&fn->counter_sections[i]);

	return /* tag */ 4 + /* length */ 4 + sizeof_str(fn->name) +
	       /* checksum */ 4 + sec_len;
}


/* Return length of node data encoded in GCOV format. */
static size_t
sizeof_da_file(struct bb *bb)
{
	size_t fn_len;
	unsigned int i;

	fn_len = 0;
	for (i=0; i < bb->n_functions; i++)
		fn_len += sizeof_func_data(&bb->functions[i]);
	return /* magic */ 4 + /* version */ 4 + fn_len;
}


/* Store a 32 bit unsigned integer value in GCOV format to memory at address
 * BUF. */
static void
store_unsigned(unsigned int v, unsigned char *buf)
{
	int i;

	for (i=0; i < 4; i++) {
		buf[3 - i] = v & 0xff;
		v >>= 8;
	}
}


/* Store a 64 bit unsigned integer value in GCOV format to memory at address
 * BUF. */
static void
store_counter(gcov_type v, unsigned char *buf)
{
	int i;

	for (i=0; i < 8; i++) {
		buf[7 - i] = v & 0xff;
		v >>= 8;
	}
}


/* Find counter section data with given TAG. Return number if found, -EINVAL
 * otherwise. */
static long
find_count_section(struct bb *bb, unsigned int tag)
{
	unsigned int i;

	for (i=0; i < bb->n_counter_sections; i++)
		if (bb->counter_sections[i].tag == tag)
			return i;

	return -EINVAL;
}


/* Store a string in GCOV format in userspace memory. The string is passed in
 * STR. The destination address is BUF. Store at most COUNT bytes beginning at
 * OFFSET. Return the number of bytes stored or a negative value on error. */
static ssize_t
store_string_data(const char *str, char *buf, size_t count, loff_t offset)
{
	const char *from;
	char data[8];
	size_t len;
	ssize_t result;
	size_t str_len;

	str_len = strlen(str);
	result = 0;
	while (count > 0) {
		if (offset < 4) {
			/* Tag ID */
			store_unsigned(str_len, data);
			len = 4 - offset;
			from = data + offset;
		} else if (offset < 4 + str_len) {
			/* String */
			len = str_len - (offset - 4);
			from = str + (offset - 4);
		} else if (offset < sizeof_str(str)) {
			/* Nil byte padding */
			store_unsigned(0, data);
			len = 4 - (str_len & 3) - (offset - 4 - str_len);
			from = data;
		} else
			/* EOF */
			break;
		if (len > count)
			len = count;
		if (copy_to_user(buf, from, len))
			return -EFAULT;
		count -= len;
		buf += len;
		offset += len;
		result += len;
	}
	return result;
}


/* Store a counter section in userspace memory. The counter section is
 * identified by BB, FUNC and SEC. The destination address is BUF. Store at
 * most COUNT bytes beginning at OFFSET. Return the number of bytes stored or a
 * negative value on error. */
static ssize_t
store_counter_data(struct bb *bb, unsigned int func, unsigned int sec,
		   gcov_type ***counters, char *buf, size_t count,
		   loff_t offset)
{
	const struct function_info *func_ptr;
	char data[8];
	char *from;
	size_t len;
	ssize_t result;
	unsigned int i;

	func_ptr = &bb->functions[func];
	result = 0;
	while (count > 0) {
		if (offset < 4) {
			/* Tag ID */
			store_unsigned(func_ptr->counter_sections[sec].tag,
				       data);
			len = 4 - offset;
			from = data + offset;
		} else if (offset < 8) {
			/* Tag length in bytes */
			store_unsigned(
				func_ptr->counter_sections[sec].n_counters * 8,
				data);
			len = 4 - (offset - 4);
			from = data + (offset - 4);
		} else {
			/* Actual counter data */
			i = (offset - 8) / 8;
			/* Check for EOF */
			if (i >= func_ptr->counter_sections[sec].n_counters)
				break;
			store_counter(*(counters[func][sec] + i), data);
			len = 8 - (offset - 8) % 8;
			from = data + (offset - 8) % 8;
		}
		if (len > count)
			len = count;
		if (copy_to_user(buf, from, len))
			return -EFAULT;
		count -= len;
		buf += len;
		offset += len;
		result += len;
	}
	return result;
}


/* Store a function section and associated counter sections in userspace memory.
 * The function section is identified by BB and FUNC. The destination address is
 * BUF. Store at most COUNT bytes beginning at OFFSET. Return the number of
 * bytes stored or a negative value on error. */
static ssize_t
store_func_data(struct bb *bb, unsigned int func, gcov_type ***counters,
		char *buf, size_t count, loff_t offset)
{
	const struct function_info *func_ptr;
	char data[4];
	char *from;
	size_t len;
	ssize_t result;
	ssize_t rc;
	size_t name_len;
	unsigned int i;
	loff_t off;
	size_t size;

	func_ptr = &bb->functions[func];
	name_len = sizeof_str(func_ptr->name);
	result = 0;
	from = NULL;
	len = 0;
	while (count > 0) {
		if (offset < 8) {
			if (offset < 4) {
				/* Tag ID */
				store_unsigned(GCOV_TAG_FUNCTION, data);
				len = 4 - offset;
				from = data + offset;
			} else if (offset < 8) {
				/* Tag length */
				store_unsigned(name_len + 4, data);
				len = 4 - (offset - 4);
				from = data + (offset - 4);
			}
			/* Do the actual store */
			if (len > count)
				len = count;
			if (copy_to_user(buf, from, len))
				return -EFAULT;
		} else if (offset < 8 + name_len) {
			/* Function name */
			rc = store_string_data(func_ptr->name, buf, count,
					       offset - 8);
			if (rc < 0)
				return rc;
			len = rc;
		} else if (offset < 8 + name_len + 4) {
			/* Function checksum */
			store_unsigned(func_ptr->checksum, data);
			len = 4 - (offset - (8 + name_len));
			from = data + (offset - (8 + name_len));
			/* Do the actual store */
			if (len > count)
				len = count;
			if (copy_to_user(buf, from, len))
				return -EFAULT;
		} else {
			off = 8 + name_len + 4;
			len = 0;
			for (i=0; i < func_ptr->n_counter_sections; i++) {
				size = sizeof_counter_data(
					&func_ptr->counter_sections[i]);
				if (offset < off + size) {
					rc = store_counter_data(bb, func, i,
								counters,
								buf, count,
								offset - off);
					if (rc < 0)
						return rc;
					len = rc;
					break;
				}
				off += size;
			}
			/* Check for EOF */
			if (len == 0)
				break;
		}
		count -= len;
		buf += len;
		offset += len;
		result += len;
	}
	return result;
}


/* Allocate and initialize counter pointer array for BB. Return zero on
 * success, non-zero otherwise. The array will have the following content:
 *
 *   counters[function_index][counter_section_index] = pointer to first count */
static int
create_counters_array(struct bb *bb, gcov_type ****counters)
{
	unsigned int f;
	long bb_c;
	unsigned int fn_c;
	unsigned int max_c;
	gcov_type ***fn_counters;
	gcov_type **bb_counters;
	const struct function_info *fn;
	const struct counter_section *sec;

	/* Calculate maximum number of counter_sections */
	max_c = 0;
	for (f=0; f < bb->n_functions; f++)
		if (bb->functions[f].n_counter_sections > max_c)
			max_c = bb->functions[f].n_counter_sections;
	fn_counters = kmalloc(bb->n_functions * sizeof(gcov_type **) +
			      bb->n_functions * max_c * sizeof(gcov_type *),
			      GFP_KERNEL);
	if (!fn_counters)
		return -ENOMEM;
	for (f=0; f < bb->n_functions; f++)
		fn_counters[f] =
			((gcov_type **) &fn_counters[bb->n_functions]) +
			f * max_c;
	bb_counters = kmalloc(bb->n_counter_sections * sizeof(gcov_type *),
			      GFP_KERNEL);
	if (!bb_counters) {
		kfree(fn_counters);
		return -ENOMEM;
	}
	/* Initialize arrays */
	for (bb_c=0; bb_c < bb->n_counter_sections; bb_c++)
		bb_counters[bb_c] = bb->counter_sections[bb_c].counters;
	for (f=0; f < bb->n_functions; f++) {
		fn = &bb->functions[f];
		for (fn_c=0; fn_c < fn->n_counter_sections; fn_c++) {
			sec = &fn->counter_sections[fn_c];
			bb_c = find_count_section(bb, sec->tag);
			if (bb_c < 0) {
				/* Tag not found */
				kfree(fn_counters);
				kfree(bb_counters);
				return -EINVAL;
			}
			fn_counters[f][fn_c] = bb_counters[bb_c];
			bb_counters[bb_c] += sec->n_counters;
		}
	}
	kfree(bb_counters);
	*counters = fn_counters;
	return 0;
}

/* Store data of .da file associated with NODE to userspace memory at BUF.
 * OFFSET specifies the offset inside the .da file. COUNT is the maximum
 * number of bytes to store. Return the number of bytes stored, zero for
 * EOF or a negative number in case of error. */
static ssize_t
store_da_file(struct gcov_ftree_node *node, char *buf, size_t count,
	      loff_t offset)
{
	struct bb *bb;
	char data[4];
	char *from;
	size_t len;
	unsigned int i;
	loff_t off;
	size_t size;
	ssize_t result;
	ssize_t rc;
	gcov_type ***counters;

	bb = node->bb;
	rc = create_counters_array(bb, &counters);
	if (rc)
		return rc;
	result = 0;
	while (count > 0) {
		if (offset < 8) {
			if (offset < 4) {
				/* File magic */
				store_unsigned(GCOV_DATA_MAGIC, data);
				len = 4 - offset;
				from = data + offset;
			} else {
				/* File format/GCC version */
				store_unsigned(gcov_version, data);
				len = 4 - (offset - 4);
				from = data + (offset - 4);
			}
			/* Do the actual store */
			if (len > count)
				len = count;
			if (copy_to_user(buf, from, len)) {
				kfree(counters);
				return -EFAULT;
			}
		} else {
			off = 8;
			len = 0;
			for (i=0; i < bb->n_functions; i++) {
				size = sizeof_func_data(&bb->functions[i]);
				if (offset < off + size) {
					rc = store_func_data(bb, i, counters,
							     buf, count,
							     offset - off);
					if (rc < 0) {
						kfree(counters);
						return rc;
					}
					len = rc;
					break;
				}
				off += size;
			}
			/* Check for EOF */
			if (len == 0)
				break;
		}
		count -= len;
		buf += len;
		offset += len;
		result += len;
	}
	kfree(counters);
	return result;
}

#elif GCC_VERSION_LOWER(3, 4)

/*
 *   gcc 3.3 specific functions (unmodified)
 */
static const char* gcov_proc_format = "gcc 3.3";


/* Return size of .da file section associated with function data FUNC. */
static inline size_t
sizeof_func_info(struct bb_function_info *func)
{
	return (size_t)
	       (/* delim */ 4 + /* strlen */ 4 + PAD4(strlen(func->name) + 1) +
	        /* delim */ 4 + /* checksum */ 4 + /* arc_count */ 4 +
	        /* count values */ func->arc_count * 8);
}


/* Return size of .da file associated with BB. */
static inline size_t
sizeof_da_file(struct bb *bb)
{
	struct bb_function_info *func;
	size_t size;

	size = ( /* magic */ 4 + /* num_func */ 4 + /* num_extra */ 4);
	for (func = bb->function_infos; func->arc_count != -1; func++)
		size += sizeof_func_info(func);
	return size;
}


/* Return the number of functions associated with BB. */
static inline gcov_type
count_functions(struct bb *bb)
{
	gcov_type result;

	for (result = 0; bb->function_infos[result].arc_count != -1;
	     result++);
	return result;
}


/* Return non-zero if OFFSET is within the range START <= OFFSET < START + SIZE,
 * zero otherwise. Update REL_OFF to contain the relative offset inside the
 * range, SIZE_VAR to contain the range size and START to point to the next
 * range after this one. */
static inline int in_range(loff_t offset, size_t size, loff_t *rel_off,
			   loff_t *start, size_t *size_var)
{
	int result;

	result = (offset >= *start) && (offset < *start + size);
	*rel_off = offset - *start;
	*start += size;
	*size_var = size;
	return result;
}


/* Store data of .da file associated with NODE to userspace memory at BUF.
 * OFFSET specifies the offset inside the .da file. COUNT is the maximum
 * number of bytes to store. Return the number of bytes stored, zero for
 * EOF or a negative number in case of error. */
static ssize_t
store_da_file(struct gcov_ftree_node *node, char *buf, size_t count,
	      loff_t offset)
{
	struct bb_function_info *func;
	gcov_type *count_ptr;
	char data[8];
	char *from;
	ssize_t stored;
	size_t len;
	size_t size;
	size_t func_off;
	size_t next_off;
	loff_t rel_off;
	loff_t start;

	func_off = 0;
	func = NULL;
	count_ptr = NULL;
	stored = 0;
	while (count > 0) {
		start = 0;
		if (in_range(offset, 4, &rel_off, &start, &size)) {
			/* Magic */
			if (store_gcov_type(-123, data, 4))
				return -EINVAL;
			from = data + rel_off;
			len = size - rel_off;
		} else if (in_range(offset, 4, &rel_off, &start, &size)) {
			/* Number of functions */
			if (store_gcov_type(count_functions(node->bb),
					    data, 4))
				return -EINVAL;
			from = data + rel_off;
			len = size - rel_off;
		} else if (in_range(offset, 4, &rel_off, &start, &size)) {
			/* Size of extra data */
			store_gcov_type(0, data, 4);
			from = data + rel_off;
			len = size - rel_off;
		} else if (offset < node->da_size) {
			/* Function data */
			rel_off = offset - 12;
			/* Try to minimize search effort */
			if (!(func && (func_off <= rel_off))) {
				func = node->bb->function_infos;
				func_off = 0;
				count_ptr = node->bb->counts;
			}
			/* Find function which is hit by offset */
			for (; func->arc_count != -1; func++) {
				next_off = func_off + sizeof_func_info(func);
				if (next_off > rel_off)
					break;
				func_off = next_off;
				count_ptr += func->arc_count;
			}
			start = 0;
			if (in_range(offset - func_off - 12, 4, &rel_off,
				     &start, &size)) {
				/* String delimiter */
				store_gcov_type(-1, data, 4);
				from = data + rel_off;
				len = size - rel_off;
			} else if (in_range(offset - func_off - 12, 4, &rel_off,
				   &start, &size)) {
				/* String length */
				if (store_gcov_type(strlen(func->name),
						    data, 4))
					return -EINVAL;
				from = data + rel_off;
				len = size - rel_off;
			} else if (in_range(offset - func_off - 12,
				   strlen(func->name), &rel_off, &start,
				   &size)) {
				/* Function name */
				from = (char *) func->name + rel_off;
				len = size - rel_off;
			} else if (in_range(offset - func_off - 12,
				   PAD4(strlen(func->name) + 1) -
				   strlen(func->name), &rel_off, &start,
				   &size)) {
				/* Nil byte padding */
				memset(data, 0, size);
				from = data;
				len = size - rel_off;
			} else if (in_range(offset - func_off - 12, 4, &rel_off,
				   &start, &size)) {
				/* String delimiter */
				store_gcov_type(-1, data, size);
				from = data + rel_off;
				len = size - rel_off;
			} else if (in_range(offset - func_off - 12, 4, &rel_off,
				   &start, &size)) {
				/* Checksum */
				store_gcov_type(func->checksum, data, 4);
				from = data + rel_off;
				len = size - rel_off;
			} else if (in_range(offset - func_off - 12, 4, &rel_off,
				   &start, &size)) {
				/* Number of arcs */
				if (store_gcov_type(func->arc_count, data, 4))
					return -EINVAL;
				from = data + rel_off;
				len = size - rel_off;
			} else if (in_range(offset - func_off - 12,
				   func->arc_count * 8, &rel_off, &start,
				   &size)) {
				/* Counts */
				if (store_gcov_type(count_ptr[rel_off / 8],
						    data, 8))
					return -EINVAL;
				from = data + rel_off % 8;
				len = 8 - rel_off % 8;
			} else {
				break;
			}
		} else
			break;
		if (len > count)
			len = count;
		if (copy_to_user(buf, from, len))
			return -EFAULT;
		stored += len;
		count -= len;
		offset += len;
		buf += len;
	}
	return stored;
}

#else /* GCC_VERSION */

/*
 *  gcc 3.4 functions
 */
static const char* gcov_proc_format = "gcc 3.4";

/* Determine whether counter TYPE is active in BB. */
static inline int
counter_active(struct bb *bb, unsigned int type)
{
	return (1 << type) & bb->ctr_mask;
}


/* Return the number of active counter types for BB. */
static inline unsigned int
num_counter_active(struct bb *bb)
{
	unsigned int i;
	unsigned int result;

	result = 0;
	for (i=0; i < GCOV_COUNTERS; i++)
		if (counter_active(bb, i))
			result++;
	return result;
}


/* Get number of bytes used for one entry in the gcov_fn_info array pointed to
 * by BB->functions. */
static inline unsigned int
get_fn_stride(struct bb *bb)
{
	unsigned int stride;

	stride = sizeof(struct gcov_fn_info) + num_counter_active(bb) *
		 sizeof(unsigned int);
	if (__alignof__(struct gcov_fn_info) > sizeof(unsigned int)) {
		stride += __alignof__(struct gcov_fn_info) - 1;
		stride &= ~(__alignof__(struct gcov_fn_info) - 1);
	}
	return stride;
}


/* Get the address of gcov_fn_info for function FUNC of BB. */
static inline struct gcov_fn_info *
get_fn_info(struct bb *bb, unsigned int func)
{
	return (struct gcov_fn_info *)
		((char *) bb->functions + func * get_fn_stride(bb));
}


/* Return size of .gcda counter section. */
static inline size_t
sizeof_counter_data(struct bb *bb, struct gcov_fn_info *func, unsigned int type)
{
	if (counter_active(bb, type)) {
		return /* tag */ 4 + /* length */ 4 +
		       /* counters */ func->n_ctrs[type] * 8;
	} else
		return 0;
}


/* Return size of .gcda data section associated with FUNC.  */
static inline size_t
sizeof_func_data(struct bb *bb, struct gcov_fn_info *func)
{
	size_t result;
	unsigned int type;

	result = /* tag */ 4 + /* length */ 4 + /* ident */ 4+
		 /* checksum */ 4;
	for (type=0; type < GCOV_COUNTERS; type++)
		result += sizeof_counter_data(bb, func, type);
	return result;
}


/* Get size of .gcda file associated with BB. */
static inline size_t
sizeof_da_file(struct bb *bb)
{
	size_t result;
	unsigned int i;

	result = /* magic */ 4 + /* version */ 4 + /* stamp */ 4;
	for (i=0; i < bb->n_functions; i++)
		result += sizeof_func_data(bb, get_fn_info(bb, i));
	return result;
}


/* Store a 32 bit unsigned integer value in GCOV format to memory at address
 * BUF. */
static inline void
store_int32(uint32_t i, char *buf)
{
	uint32_t *p;

	p = (int *) buf;
	*p = i;
}


/* Store a 64 bit unsigned integer value in GCOV format to memory at address
 * BUF. */
static inline void
store_int64(uint64_t i, char *buf)
{
	store_int32((uint32_t) (i & 0xffff), buf);
	store_int32((uint32_t) (i >> 32), buf + 4);
}


/* Store a gcov counter in GCOV format to memory at address BUF. The counter is
 * identified by BB, FUNC, TYPE and COUNTER. */
static inline void
store_counter(struct bb *bb, unsigned int func, unsigned int type,
	      unsigned int counter, char *buf)
{
	unsigned int counter_off;
	unsigned int type_off;
	unsigned int i;

	/* Get offset into counts array */
	type_off = 0;
	for (i=0; i < type; i++)
		if (counter_active(bb, i))
			type_off++;
	/* Get offset into values array. */
	counter_off = counter;
	for (i=0; i < func; i++)
		counter_off += get_fn_info(bb, i)->n_ctrs[type];
	/* Create in temporary storage */
	store_int64(bb->counts[type_off].values[counter_off], buf);
}


/* Store a counter section in userspace memory. The counter section is
 * identified by BB, FUNC and TYPE. The destination address is BUF. Store at
 * most COUNT bytes beginning at OFFSET. Return the number of bytes stored or a
 * negative value on error. */
static inline ssize_t
store_counter_data(struct bb *bb, unsigned int func, unsigned int type,
		  char *buf, size_t count, loff_t offset)
{
	struct gcov_fn_info *func_ptr;
	char data[8];
	char *from;
	size_t len;
	ssize_t result;
	unsigned int i;

	func_ptr = get_fn_info(bb, func);
	result = 0;
	while (count > 0) {
		if (offset < 4) {
			/* Tag ID */
			store_int32((uint32_t) GCOV_TAG_FOR_COUNTER(type),
				    data);
			len = 4 - offset;
			from = data + offset;
		} else if (offset < 8) {
			/* Tag length in groups of 4 bytes */
			store_int32((uint32_t)
				    func_ptr->n_ctrs[type] * 2, data);
			len = 4 - (offset - 4);
			from = data + (offset - 4);
		} else {
			/* Actual counter data */
			i = (offset - 8) / 8;
			/* Check for EOF */
			if (i >= func_ptr->n_ctrs[type])
				break;
			store_counter(bb, func, type, i, data);
			len = 8 - (offset - 8) % 8;
			from = data + (offset - 8) % 8;
		}
		if (len > count)
			len = count;
		if (copy_to_user(buf, from, len))
			return -EFAULT;
		count -= len;
		buf += len;
		offset += len;
		result += len;
	}
	return result;
}


/* Store a function section and associated counter sections in userspace memory.
 * The function section is identified by BB and FUNC. The destination address is
 * BUF. Store at most COUNT bytes beginning at OFFSET. Return the number of
 * bytes stored or a negative value on error. */
static inline ssize_t
store_func_data(struct bb *bb, unsigned int func, char *buf,
		size_t count, loff_t offset)
{
	struct gcov_fn_info *func_ptr;
	char data[4];
	char *from;
	size_t len;
	unsigned int i;
	loff_t off;
	size_t size;
	ssize_t result;
	ssize_t rc;

	func_ptr = get_fn_info(bb, func);
	result = 0;
	while (count > 0) {
		if (offset < 16) {
			if (offset < 4) {
				/* Tag ID */
				store_int32((uint32_t) GCOV_TAG_FUNCTION, data);
				len = 4 - offset;
				from = data + offset;
			} else if (offset < 8) {
				/* Tag length */
				store_int32(2, data);
				len = 4 - (offset - 4);
				from = data + (offset - 4);
			} else if (offset < 12) {
				/* Function ident */
				store_int32((uint32_t) func_ptr->ident, data);
				len = 4 - (offset - 8);
				from = data + (offset - 8);
			} else {
				/* Function checksum */
				store_int32((uint32_t) func_ptr->checksum,
					    data);
				len = 4 - (offset - 12);
				from = data + (offset - 12);
			}
			/* Do the actual store */
			if (len > count)
				len = count;
			if (copy_to_user(buf, from, len))
				return -EFAULT;
		} else {
			off = 16;
			len = 0;
			for (i=0; i < GCOV_COUNTERS; i++) {
				size = sizeof_counter_data(bb, func_ptr, i);
				if (offset < off + size) {
					rc = store_counter_data(bb, func, i,
								buf, count,
								offset - off);
					if (rc < 0)
						return rc;
					len = rc;
					break;
				}
				off += size;
			}
			/* Check for EOF */
			if (i == GCOV_COUNTERS)
				break;
		}
		count -= len;
		buf += len;
		offset += len;
		result += len;
	}
	return result;
}


/* Store data of .gcda file associated with NODE to userspace memory at BUF.
 * OFFSET specifies the offset inside the .da file. COUNT is the maximum
 * number of bytes to store. Return the number of bytes stored, zero for
 * EOF or a negative number in case of error. */
static ssize_t
store_da_file(struct gcov_ftree_node *node, char *buf, size_t count,
	      loff_t offset)
{
	struct bb *bb;
	char data[4];
	char *from;
	size_t len;
	unsigned int i;
	loff_t off;
	size_t size;
	ssize_t result;
	ssize_t rc;

	bb = node->bb;
	result = 0;
	while (count > 0) {
		if (offset < 12) {
			if (offset < 4) {
				/* File magic */
				store_int32((uint32_t) GCOV_DATA_MAGIC, data);
				len = 4 - offset;
				from = data + offset;
			} else if (offset < 8) {
				/* File format/GCC version */
				store_int32(gcov_version, data);
				len = 4 - (offset - 4);
				from = data + (offset - 4);
			} else {
				/* Time stamp */
				store_int32((uint32_t) bb->stamp, data);
				len = 4 - (offset - 8);
				from = data + (offset - 8);
			}
			/* Do the actual store */
			if (len > count)
				len = count;
			if (copy_to_user(buf, from, len))
				return -EFAULT;
		} else {
			off = 12;
			len = 0;
			for (i=0; i < bb->n_functions; i++) {
				size = sizeof_func_data(bb, get_fn_info(bb, i));
				if (offset < off + size) {
					rc = store_func_data(bb, i, buf, count,
							     offset - off);
					if (rc < 0)
						return rc;
					len = rc;
					break;
				}
				off += size;
			}
			/* Check for EOF */
			if (i == bb->n_functions)
				break;
		}
		count -= len;
		buf += len;
		offset += len;
		result += len;
	}
	return result;
}
#endif /* GCC_VERSION */


/* Update data related to vmlinux file. */
static void
update_vmlinux_data(void)
{
	struct gcov_ftree_node *node;
	loff_t offset;

	offset = 0;
	for (node = leaf_nodes; node; node = node->next) {
		node->offset = offset;
		node->da_size = sizeof_da_file(node->bb);
		node->header_size = sizeof_vmlinux_header(node->bb);
		offset += node->header_size + node->da_size;
	}
	proc_vmlinux->size = offset;
	cached_node = NULL;
}


/* Read .da or vmlinux file. */
static ssize_t
read_gcov(struct file *file, char *buf, size_t count, loff_t *pos)
{
	struct gcov_ftree_node *node;
	struct proc_dir_entry *dir_entry;
	ssize_t rc;

	down(&gcov_lock);
	dir_entry = PDE(file->f_dentry->d_inode);
	rc = 0;
	if (dir_entry == proc_vmlinux) {
		/* Are we in a sequential read? */
		if (cached_node && (*pos >= cached_node->offset))
			node = cached_node;
		else
			node = leaf_nodes;
		/* Find node corresponding to offset */
		while (node && node->next && (*pos >= node->next->offset))
			node = node->next;
		cached_node = node;
		if (node) {
			if (*pos - node->offset < node->header_size)
				rc = store_vmlinux_header(node, buf, count,
							  *pos - node->offset);
			else
				rc = store_da_file(node, buf, count,
						   *pos - node->offset -
							node->header_size);
		}
	} else {
		node = (struct gcov_ftree_node *) dir_entry->data;
		if (node)
			rc = store_da_file(node, buf, count, *pos);
	}
	if (rc > 0)
		*pos += rc;
	up(&gcov_lock);
	return rc;
}


static inline void
reset_bb(struct bb* bb)
{
#if GCC_VERSION_EQUAL(3,3) && defined(CONFIG_GCOV_HAMMER)
	unsigned int i;

	for (i=0; i < bb->n_counter_sections; i++)
		memset(bb->counter_sections[i].counters, 0,
		       bb->counter_sections[i].n_counters * sizeof(gcov_type));
#elif GCC_VERSION_LOWER(3, 4)
	memset(bb->counts, 0, bb->ncounts * sizeof(gcov_type));
#else /* GCC_VERSION */
	const struct gcov_ctr_info *ctr;
	unsigned int i;

	ctr = bb->counts;
	for (i=0; i < GCOV_COUNTERS; i++)
		if (counter_active(bb, i)) {
			memset(ctr->values, 0, ctr->num * sizeof(gcov_type));
			ctr++;
		}
#endif /* GCC_VERSION */
}


static void cleanup_node_and_path(struct gcov_ftree_node *node);

/* Reset counters on write request. */
static ssize_t
write_gcov(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct gcov_ftree_node *node;
	struct gcov_ftree_node *next;
	struct proc_dir_entry *dir_entry;

	down(&gcov_lock);
	dir_entry = PDE(file->f_dentry->d_inode);
	if (dir_entry == proc_vmlinux) {
		/* Reset all nodes */
		node = leaf_nodes;
		while (node) {
			next = node->next;
			/* Delete ghosted nodes */
			if (node->status == status_ghost)
				cleanup_node_and_path(node);
			else
				reset_bb(node->bb);
			node = next;
		}
		/* Nodes may have been deleted - update data */
		if (gcov_persist)
			update_vmlinux_data();
	} else {
		node = (struct gcov_ftree_node *) dir_entry->data;
		reset_bb(node->bb);
	}
	up(&gcov_lock);
	return count;
}


/* Return a newly allocated copy of STRING. */
static inline char *
strdup(const char *string)
{
	char *result;

	result = (char *) kmalloc(strlen(string) + 1, GFP_KERNEL);
	if (result)
		strcpy(result, string);
	return result;
}


/* Allocate a new node and fill in NAME and BB. */
static struct gcov_ftree_node *
alloc_node(const char *name, struct bb *bb)
{
	struct gcov_ftree_node *node;

	node = (struct gcov_ftree_node *)
		kmalloc(sizeof(struct gcov_ftree_node), GFP_KERNEL);
	if (!node)
		return NULL;
	memset(node, 0, sizeof(struct gcov_ftree_node));
	node->fname = strdup(name);
	if (!node->fname) {
		kfree(node);
		return NULL;
	}
	node->bb = bb;
	node->status = status_normal;
	return node;
}


/* Free memory allocated for BB. */
static void
free_bb(struct bb *bb)
{
#if GCC_VERSION_LOWER(3,4)
	kfree(bb);
#else /* GCC_VERSION */
	kfree(bb->functions);
	kfree(bb);
#endif /* GCC_VERSION */
}


/* Free memory allocated for NODE. */
static void
free_node(struct gcov_ftree_node *node)
{
	if (node == &tree_root)
		return;
	if (node->fname)
		kfree(node->fname);
	if (node->status == status_ghost)
		free_bb(node->bb);
	kfree(node);
}


/* Remove proc filesystem entries associated with NODE. */
static void
delete_from_proc(struct gcov_ftree_node *node)
{
	struct proc_dir_entry *parent;
	int i;

	if (node->parent)
		parent = node->parent->proc[0];
	else
		parent = NULL;
	for (i = 0; i < sizeof(node->proc) / sizeof(node->proc[0]); i++)
		if (node->proc[i])
			remove_proc_entry(node->proc[i]->name, parent);
}


/* Release all resources associated with NODE. If NODE is a directory node,
 * also clean up all children. */
static void
cleanup_node(struct gcov_ftree_node *node)
{
	struct gcov_ftree_node *curr;
	struct gcov_ftree_node *next;
	struct gcov_ftree_node *prev;

	next = node;
	do {
		/* Depth first traversal of all children */
		curr = next;
		while (curr->files)
			curr = curr->files;
		if (curr->sibling)
			next = curr->sibling;
		else
			next = curr->parent;
		/* Remove from tree */
		if (curr->parent) {
			if (curr->parent->files == curr)
				curr->parent->files = curr->sibling;
			else {
				for (prev = curr->parent->files;
				     prev->sibling != curr;
				     prev = prev->sibling);
				prev->sibling = curr->sibling;
			}
		}
		/* Remove from leaf node list if necessary */
		if (curr->bb) {
			if (leaf_nodes == curr)
				leaf_nodes = curr->next;
			else {
				for (prev = leaf_nodes;
				     prev && (prev->next != curr);
				     prev = prev->next);
				if (prev)
					prev->next = curr->next;
			}
		}
		/* Delete node */
		delete_from_proc(curr);
		free_node(curr);
	} while (node != curr);
}


/* Clean up NODE and containing path in case it would be left empty. */
static void
cleanup_node_and_path(struct gcov_ftree_node *node)
{
	while (node->parent &&
	       node->parent != &tree_root &&
	       !node->parent->files->sibling)
		node = node->parent;
	cleanup_node(node);
}


/* Create a new directory node named NAME under PARENT. Upon success return
 * zero and update RESULT to point to the newly created node. Return non-zero
 * otherwise. */
static int
create_dir_node(struct gcov_ftree_node *parent, char *name,
		struct gcov_ftree_node **result)
{
	struct gcov_ftree_node *node;

	/* Initialize new node */
	node = alloc_node(name, NULL);
	if (!node)
		return -ENOMEM;
	/* Create proc filesystem entry */
	node->proc[0] = proc_mkdir(name, parent->proc[0]);
	if (!node->proc[0]) {
		free_node(node);
		return -EIO;
	}
	/* Insert node into tree */
	node->parent = parent;
	node->sibling = parent->files;
	parent->files = node;
	*result = node;
	return 0;
}


static struct file_operations proc_gcov_operations = {
	read: read_gcov,
	write: write_gcov,
	owner: THIS_MODULE
};

/* Create a new file node named NAME under PARENT. Associate node with BB.
 * Return zero upon success, non-zero otherwise. */
static int
create_file_node(struct gcov_ftree_node *parent, char *name, struct bb *bb)
{
	struct gcov_ftree_node *node;
	char *link_target;
	char *link_name;
	int i;

	/* Initialize new node */
	node = alloc_node(name, bb);
	if (!node)
		return -ENOMEM;
	/* Create proc filesystem entry */
	node->proc[0] = create_proc_entry(name, S_IWUSR | S_IRUGO,
					  parent->proc[0]);
	if (!node->proc[0]) {
		free_node(node);
		return -EIO;
	}
	node->proc[0]->data = node;
	node->proc[0]->proc_fops = &proc_gcov_operations;
	node->proc[0]->size = sizeof_da_file(bb);
	/* Create symbolic links */
	if (gcov_link) {
		/* Note: temp string length is calculated as upper limit */
		link_target = (char *) kmalloc(strlen(bb->filename) +
					       strlen(da_ending) +
					       strlen(gcov_sourcepath),
					       GFP_KERNEL);
		if (!link_target) {
			delete_from_proc(node);
			free_node(node);
			return -ENOMEM;
		}
		for (i = 0; i < sizeof(endings) / sizeof(endings[0]); i++) {
			if ((strcmp(endings[i], "c") == 0) &&
			    (strcmp(gcov_sourcepath, gcov_objectpath) != 0) &&
			    (strncmp(bb->filename, gcov_objectpath,
				     objectpath_len) == 0)) {
				strcpy(link_target, gcov_sourcepath);
				strcat(link_target,
				       bb->filename + objectpath_len);
			} else {
				strcpy(link_target, bb->filename);
			}
			link_target[strlen(link_target) -
				    strlen(da_ending)] = 0;
			strcat(link_target, endings[i]);
			link_name = strrchr(link_target, '/') + 1;
			node->proc[i + 1] = proc_symlink(link_name,
							 parent->proc[0],
							 link_target);
			if (!node->proc[i + 1]) {
				kfree(link_target);
				delete_from_proc(node);
				free_node(node);
				return -EIO;
			}
		}
		kfree(link_target);
	}
	/* Insert node into tree */
	node->parent = parent;
	node->sibling = parent->files;
	parent->files = node;
	node->next = leaf_nodes;
	leaf_nodes = node;
	return 0;
}


/* Return proc filesystem entry name for FILENAME. */
static char *
get_proc_filename(const char *filename)
{
	char *result;

	/* Is this file located in the kernel source directory? */
	if (strncmp(filename, gcov_objectpath, objectpath_len) == 0) {
		/* Use relative path */
		result = strdup(filename + objectpath_len + 1);
	} else {
		/* Use full path in module subdirectory */
		result = (char *) kmalloc(strlen(GCOV_PROC_MODULE) +
					  strlen(filename) + 1, GFP_KERNEL);
		if (result) {
			strcpy(result, GCOV_PROC_MODULE);
			strcat(result, filename);
		}
	}
	return result;
}


/* Create tree node and proc filesystem entry for BB. Create subdirectories as
 * necessary. Return zero upon success, non-zero otherwise. */
static int
create_node(struct bb *bb)
{
	struct gcov_ftree_node *parent;
	struct gcov_ftree_node *node;
	char *filename;
	char *curr;
	char *next;
	int rc;

	filename = get_proc_filename(bb->filename);
	if (!filename)
		return -ENOMEM;
	/* Recreate directory path in proc filesystem */
	parent = &tree_root;
	for (curr = filename; (next = strchr(curr, '/')); curr = next + 1) {
		/* Skip empty path components */
		if (curr == next)
			continue;
		*next = 0;
		/* Check whether directory already exists */
		for (node = parent->files;
		     node && (strcmp(node->fname, curr) != 0);
		     node = node->sibling);
		if (!node) {
			/* Create directory node */
			rc = create_dir_node(parent, curr, &node);
			if (rc) {
				if (parent != &tree_root)
					cleanup_node_and_path(parent);
				kfree(filename);
				return rc;
			}
		}
		parent = node;
	}
	rc = create_file_node(parent, curr, bb);
	kfree(filename);
	return rc;
}


/* Return a copy of BB which contains only data relevant to this module. */
static struct bb *
clone_bb(struct bb *bb)
{
	struct bb *result;
	size_t len;

#if GCC_VERSION_LOWER(3, 3)

	/* Allocate memory */
	len = sizeof(struct bb) + bb->ncounts * sizeof(gcov_type) +
	      strlen(bb->filename) + 1;
	result = (struct bb *) kmalloc(len, GFP_KERNEL);
	if (!result)
		return NULL;
	memset(result, 0, len);
	/* Copy count data */
	result->counts = (gcov_type *) (result + 1);
	result->ncounts = bb->ncounts;
	memcpy(result->counts, bb->counts, result->ncounts * sizeof(gcov_type));
	/* Copy filename */
	result->filename = (const char *) &result->counts[result->ncounts];
	strcpy((char *) result->filename, bb->filename);

#elif GCC_VERSION_EQUAL(3, 3) && CONFIG_GCOV_HAMMER

	unsigned int i;
	unsigned char *curr;
	struct function_info *func;
	struct counter_section_data *sec;

	/* Allocate memory */
	len = sizeof(struct bb) +
	      bb->n_functions * sizeof(struct function_info) +
	      bb->n_counter_sections * sizeof(struct counter_section_data) +
	      strlen(bb->filename) + 1;
	for (i=0; i < bb->n_functions; i++) {
		len += strlen(bb->functions[i].name) + 1;
		len += bb->functions[i].n_counter_sections *
		       sizeof(struct counter_section);
	}
	for (i=0 ; i < bb->n_counter_sections; i++)
		len += bb->counter_sections[i].n_counters * sizeof(gcov_type);
	result = (struct bb *) kmalloc(len, GFP_KERNEL);
	if (!result)
		return NULL;
	memset(result, 0, len);
	curr = (unsigned char *) result + sizeof(struct bb);
	/* Copy relevant function data */
	result->n_functions = bb->n_functions;
	result->functions = (struct function_info *) curr;
	curr += sizeof(struct function_info) * bb->n_functions;
	for (i=0; i < bb->n_functions; i++) {
		func = (struct function_info *) &result->functions[i];
		func->checksum = bb->functions[i].checksum;
		func->n_counter_sections = bb->functions[i].n_counter_sections;
		func->counter_sections = (struct counter_section *) curr;
		curr += sizeof(struct counter_section) *
			func->n_counter_sections;
		memcpy((struct counter_section *) func->counter_sections,
		       bb->functions[i].counter_sections,
		       sizeof(struct counter_section) *
		       func->n_counter_sections);
	}
	/* Copy relevant counter section data */
	result->n_counter_sections = bb->n_counter_sections;
	result->counter_sections = (struct counter_section_data *) curr;
	curr += sizeof(struct counter_section_data) *
		result->n_counter_sections;
	for (i=0; i < bb->n_counter_sections; i++) {
		sec = (struct counter_section_data *)
			&result->counter_sections[i];
		sec->tag = bb->counter_sections[i].tag;
		sec->n_counters = bb->counter_sections[i].n_counters;
		sec->counters = (gcov_type *) curr;
		curr += sizeof(gcov_type) * sec->n_counters;
		memcpy(sec->counters, bb->counter_sections[i].counters,
		       sizeof(gcov_type) * sec->n_counters);
	}
	/* Copy filename */
	result->filename = (char *) curr;
	curr += strlen(bb->filename) + 1;
	strcpy((char *) result->filename, bb->filename);
	/* Copy function names */
	for (i=0; i < bb->n_functions; i++) {
		func = (struct function_info *) &result->functions[i];
		func->name = (char *) curr;
		curr += strlen(bb->functions[i].name) + 1;
		strcpy((char *) func->name, bb->functions[i].name);
	}

#elif GCC_VERSION_LOWER(3, 4)

	unsigned int i;
	char *name;

	/* Allocate memory */
	len = sizeof(struct bb) + bb->ncounts * sizeof(gcov_type) +
	      strlen(bb->filename) + 1 + sizeof(struct bb_function_info);
	for (i = 0; bb->function_infos[i].arc_count != -1; i++)
		len += sizeof(struct bb_function_info) +
		       strlen(bb->function_infos[i].name) + 1;
	result = (struct bb *) kmalloc(len, GFP_KERNEL);
	if (!result)
		return NULL;
	memset(result, 0, len);
	/* Copy count data */
	result->counts = (gcov_type *) (result + 1);
	result->ncounts = bb->ncounts;
	memcpy(result->counts, bb->counts, result->ncounts * sizeof(gcov_type));
	/* Prepare copy of function infos */
	result->function_infos = (struct bb_function_info *)
					&result->counts[result->ncounts];
	/* Copy filename */
	result->filename = (const char *) &result->function_infos[i + 1];
	strcpy((char *) result->filename, bb->filename);
	/* Copy function infos */
	name = (char *) result->filename + strlen(result->filename) + 1;
	for (i = 0; bb->function_infos[i].arc_count != -1; i++) {
		result->function_infos[i].checksum =
			bb->function_infos[i].checksum;
		result->function_infos[i].arc_count =
			bb->function_infos[i].arc_count;
		strcpy(name, bb->function_infos[i].name);
		result->function_infos[i].name = name;
		name += strlen(name) + 1;
	}
	result->function_infos[i].arc_count = -1;
	result->sizeof_bb = bb->sizeof_bb;

#else /* GCOV_VERSION */

	unsigned int active;
	unsigned int i;
	char *name;
	struct gcov_fn_info *func;

	/* Allocate memory for struct bb */
	active = num_counter_active(bb);
	len = sizeof(struct bb) +
	      sizeof(struct gcov_ctr_info) * active +
	      strlen(bb->filename) + 1;
	for (i=0; i < active; i++)
		len += sizeof(gcov_type) * bb->counts[i].num;
	result = (struct bb *) kmalloc(len, GFP_KERNEL);
	if (!result)
		return NULL;
	memset(result, 0, len);
	/* Allocate memory for array of struct gcov_fn_info */
	len = bb->n_functions * get_fn_stride(bb);
	func = (struct gcov_fn_info *) kmalloc(len, GFP_KERNEL);
	if (!func) {
		kfree(result);
		return NULL;
	}
	/* Copy function data */
	memcpy(func, bb->functions, len);
	result->functions = func;
	/* Copy counts */
	for (i=0; i < active; i++) {
		result->counts[i].num = bb->counts[i].num;
		result->counts[i].merge = bb->counts[i].merge;
		if (i == 0) {
			result->counts[i].values =
				(gcov_type *) &result->counts[active];
		} else {
			result->counts[i].values =
				result->counts[i - 1].values +
				result->counts[i - 1].num;
		}
		memcpy(result->counts[i].values, bb->counts[i].values,
		       sizeof(gcov_type) * result->counts[i].num);
	}
	/* Copy rest */
	result->stamp = bb->stamp;
	name = (char *) (result->counts[active - 1].values +
			 result->counts[active - 1].num);
	strcpy(name, bb->filename);
	result->filename = name;
	result->n_functions = bb->n_functions;
	result->ctr_mask = bb->ctr_mask;

#endif /* GCC_VERSION */

	return result;
}


/* Return non-zero if BB1 and BB2 are compatible, zero otherwise. */
static int
is_compatible(struct bb *bb1, struct bb *bb2)
{
#if GCC_VERSION_LOWER(3, 3)
	return (bb1->ncounts == bb2->ncounts);
#elif GCC_VERSION_EQUAL(3, 3) && CONFIG_GCOV_HAMMER
	unsigned int i;

	if (bb1->n_functions != bb2->n_functions ||
	    bb1->n_counter_sections != bb2->n_counter_sections)
		return 0;
	for (i=0; i < bb1->n_counter_sections; i++)
		if (bb1->counter_sections[i].tag !=
			bb2->counter_sections[i].tag ||
		    bb1->counter_sections[i].n_counters !=
			bb2->counter_sections[i].n_counters)
			return 0;
	for (i=0; i < bb1->n_functions; i++) {
		if (bb1->functions[i].checksum != bb2->functions[i].checksum ||
		    bb1->functions[i].n_counter_sections !=
			bb2->functions[i].n_counter_sections)
			return 0;
	}
	return 1;
#elif GCC_VERSION_LOWER(3, 4)
	int i;

	if ((bb1->ncounts != bb2->ncounts) ||
	    (bb1->sizeof_bb != bb2->sizeof_bb))
		return 0;
	for (i = 0; (bb1->function_infos[i].arc_count != -1) &&
		    (bb2->function_infos[i].arc_count != -1); i++)
		if (bb1->function_infos[i].checksum !=
		    bb2->function_infos[i].checksum)
			return 0;
	return (bb1->function_infos[i].arc_count == -1) &&
	       (bb2->function_infos[i].arc_count == -1);
#else /* GCC_VERSION */
	return (bb1->stamp == bb2->stamp);
#endif /* GCC_VERSION */
}


/* Add count data from SOURCE to DEST. */
static void
merge_bb(struct bb *dest, struct bb *source)
{
#if GCC_VERSION_EQUAL(3, 3) && CONFIG_GCOV_HAMMER
	unsigned int i;
	unsigned int j;

	for (i=0; i < dest->n_counter_sections; i++)
		for (j=0; j < dest->counter_sections[i].n_counters; j++)
			dest->counter_sections[i].counters[j] +=
				source->counter_sections[i].counters[j];
#elif GCC_VERSION_LOWER(3, 4)
	long i;

	for (i = 0; i < dest->ncounts; i++)
		dest->counts[i] += source->counts[i];
#else /* GCC_VERSION */
	unsigned int i;
	unsigned int j;

	for (i=0; i < num_counter_active(dest); i++)
		for (j=0; j < dest->counts[i].num; j++)
			dest->counts[i].values[j] +=
				source->counts[i].values[j];
#endif /* GCC_VERSION_LOWER */
}


/* If there is a ghosted node for BB, merge old and current data, set status
 * to normal and return zero. Return non-zero otherwise. */
static int
revive_node(struct bb *bb)
{
	struct gcov_ftree_node *node;

	/* Check for a ghosted node */
	for (node = leaf_nodes; node &&
	     (strcmp(node->bb->filename, bb->filename) != 0);
	     node=node->next);
	if (!node)
		return -ENOENT;
	/* Check for compatible data */
	if (!is_compatible(bb, node->bb)) {
		printk(KERN_WARNING GCOV_PROC_HEADER "discarding saved data "
		       "for %s due to incompatibilities\n", bb->filename);
		cleanup_node_and_path(node);
		update_vmlinux_data();
		return -EINVAL;
	}
	/* Revive */
	merge_bb(bb, node->bb);
	kfree(node->bb);
	node->bb = bb;
	node->status = status_normal;
	return 0;
}


/* Make a copy of the struct bb associated with node and set node status to
 * ghost. Return zero on success, non-zero otherwise. */
static int
ghost_node(struct gcov_ftree_node *node)
{
	struct bb *bb;

	/* Ghost node instead of removing it */
	bb = clone_bb(node->bb);
	if (!bb) {
		printk(KERN_ERR GCOV_PROC_HEADER "not enough memory to save "
		       "data for %s", node->bb->filename);
		return -ENOMEM;
	}
	node->bb = bb;
	node->status = status_ghost;
	return 0;
}


/* Callback used to keep track of changes in the bb list. */
static void
gcov_proc_callback(enum gcov_cmd cmd, struct bb *bb)
{
	struct gcov_ftree_node *node;
	int rc;

	down(&gcov_lock);
	switch (cmd) {
	case gcov_add:
		if (gcov_persist && (revive_node(bb) == 0))
			break;
		/* Insert node */
		rc = create_node(bb);
		if (rc) {
			printk(KERN_ERR GCOV_PROC_HEADER "add failed: could "
			       "not create node for %s (err=%d)\n",
			       bb->filename, rc);
		}
		update_vmlinux_data();
		break;
	case gcov_remove:
		/* Find node to remove */
		for (node = leaf_nodes; node && (node->bb != bb);
		     node=node->next);
		if (!node)
			break;
		if (gcov_persist && (ghost_node(node) == 0))
			break;
		/* Remove node and empty path */
		cleanup_node_and_path(node);
		update_vmlinux_data();
		break;
	}
	up(&gcov_lock);
}


/* Initialize module. */
static int __init
gcov_init_module(void)
{
	struct bb *bb;
	int rc;

	printk(KERN_INFO GCOV_PROC_HEADER "initializing proc module: "
	       "persist=%d link=%d format=%s\n",
	       gcov_persist, gcov_link, gcov_proc_format);
	sema_init(&gcov_lock, 0);
	sourcepath_len = strlen(gcov_sourcepath);
	objectpath_len = strlen(gcov_objectpath);
	/* Initialize root node and /proc/gcov entry */
	tree_root.fname = GCOV_PROC_ROOT;
	tree_root.proc[0] = proc_mkdir(tree_root.fname, NULL);
	if (!tree_root.proc[0]) {
		printk(KERN_ERR GCOV_PROC_HEADER "init failed: could not "
		       "create root proc filesystem entry\n");
		return -EIO;
	}
	/* Create /proc/gcov/vmlinux entry */
	proc_vmlinux = create_proc_entry(GCOV_PROC_VMLINUX, S_IWUSR | S_IRUGO,
					 tree_root.proc[0]);
	if (!proc_vmlinux) {
		printk(KERN_ERR GCOV_PROC_HEADER "init failed: could not "
		       "create proc filesystem entry %s\n", GCOV_PROC_VMLINUX);
		cleanup_node(&tree_root);
		return -EIO;
	}
	proc_vmlinux->proc_fops = &proc_gcov_operations;
	/* Initialize /proc/gcov tree */
	down(&gcov_core_lock);
	for (bb = bb_head; bb ; bb = bb->next) {
		rc = create_node(bb);
		if (rc) {
			printk(KERN_ERR GCOV_PROC_HEADER "init failed: could "
			       "not create node for %s (err=%d)\n",
			       bb->filename, rc);
			remove_proc_entry(proc_vmlinux->name,
					  tree_root.proc[0]);
			cleanup_node(&tree_root);
			up(&gcov_core_lock);
			return rc;
		}
	}
	gcov_callback = gcov_proc_callback;
	update_vmlinux_data();
	up(&gcov_core_lock);
	up(&gcov_lock);
	printk(KERN_INFO GCOV_PROC_HEADER "init done\n");
	return 0;
}


/* Clean up module data. */
static void __exit
gcov_cleanup_module(void)
{
	down(&gcov_lock);
	gcov_callback = NULL;
	remove_proc_entry(proc_vmlinux->name, tree_root.proc[0]);
	cleanup_node(&tree_root);
	printk(KERN_INFO GCOV_PROC_HEADER "proc module is now unloaded\n");
}


module_init(gcov_init_module);
module_exit(gcov_cleanup_module);

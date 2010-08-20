/*
 * kernel/gcov/gcov-core.c
 *
 * Core functionality for GCOV kernel profiling.
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
 * Copyright (c) International Business Machines Corp., 2002-2006
 *
 * Author: Hubertus Franke <frankeh@us.ibm.com>
 *         Rajan Ravindran <rajancr@us.ibm.com>
 *         Peter Oberparleiter <Peter.Oberparleiter@de.ibm.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/gcov.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>
#include <linux/semaphore.h>

/* Newer kernel versions no longer support EXPORT_SYMBOL_NOVERS */
#ifndef EXPORT_SYMBOL_NOVERS
#define EXPORT_SYMBOL_NOVERS(x)	EXPORT_SYMBOL(x)
#endif

#define GCOV_CORE_HEADER	"gcov-core: "

/* This structure is used to keep track of all struct bbs associated with a
 * module. */
struct gcov_context
{
	struct list_head list;
	struct module *module;
	unsigned long count;
	struct bb **bb;
};

struct ctor_list
{
	unsigned long num;
	ctor_t ctor[];
};

/* Start of global constructor list. Declared in vmlinux-ld script.  */
extern struct ctor_list __CTOR_LIST__;

/* Linked list for registered struct bbs. */
struct bb *bb_head;

/* Callback informed of struct bb addition and removal. */
void (*gcov_callback)(enum gcov_cmd, struct bb *bbptr) = NULL;

/* Path to kernel files. */
const char *gcov_sourcepath = GCOV_SRC_PATH;
const char *gcov_objectpath = GCOV_OBJ_PATH;

/* List of contexts for registered bb entries. */
static LIST_HEAD(context_list);

/* Context into which blocks are inserted during initialization. */
static struct gcov_context *current_context;

/* Protect global variables from concurrent access. */
struct semaphore gcov_core_lock;

#if GCC_VERSION_LOWER(3, 4) && !CONFIG_GCOV_HAMMER
static const char *gcov_format = "pre-gcc_3.4";

/* Register supplied struct BB. Called by each object code constructor. */
void
__bb_init_func(struct bb *bb)
{
	if (bb->zero_word)
		return;
	/* Set up linked list */
	bb->zero_word = 1;
	bb->next = bb_head;
	bb_head = bb;
	/* Associate with module context */
	if (current_context)
		current_context->bb[current_context->count++] = bb;
	/* Notify callback */
	if (gcov_callback != NULL)
		(*gcov_callback)(gcov_add, bb);
}


/* Unused functions needed to prevent linker errors. */
void __bb_fork_func(void) {}

EXPORT_SYMBOL_NOVERS(__bb_init_func);
EXPORT_SYMBOL_NOVERS(__bb_fork_func);
#elif GCC_VERSION_EQUAL(3, 3) && CONFIG_GCOV_HAMMER
static const char *gcov_format = "gcc_3.3_(hammer)";
unsigned long gcov_version;

/* Register supplied struct BB. Called by each object code constructor. */
void
__gcov_init(struct bb *bb)
{
	if (!bb->version)
		return;
	/* Check for compatible gcc version */
	if (gcov_version == 0)
		gcov_version = bb->version;
	else if (bb->version != gcov_version) {
		printk(KERN_WARNING GCOV_CORE_HEADER "gcc version mismatch in "
		       "file '%s'!\n", bb->filename);
		return;
	}
	/* Set up linked list */
	bb->version = 0;
	bb->next = bb_head;
	bb_head = bb;
	/* Associate with module context */
	if (current_context)
		current_context->bb[current_context->count++] = bb;
	/* Notify callback */
	if (gcov_callback != NULL)
		(*gcov_callback)(gcov_add, bb);
}

void __gcov_flush(void) {}

EXPORT_SYMBOL_NOVERS(gcov_version);
EXPORT_SYMBOL_NOVERS(__gcov_init);
EXPORT_SYMBOL_NOVERS(__gcov_flush);
#else
static const char *gcov_format = "gcc_3.4";
gcov_unsigned_t gcov_version = 0;

/* Register supplied struct BB. Called by each object code constructor. */
void
__gcov_init(struct bb *bb)
{
	if (!bb->version)
		return;
	/* Check for compatible gcc version */
	if (gcov_version == 0)
		gcov_version = bb->version;
	else if (bb->version != gcov_version) {
		printk(KERN_WARNING GCOV_CORE_HEADER "gcc version mismatch in "
		       "file '%s'!\n", bb->filename);
		return;
	}
	/* Set up linked list */
	bb->version = 0;
	bb->next = bb_head;
	bb_head = bb;
	/* Associate with module context */
	if (current_context)
		current_context->bb[current_context->count++] = bb;
	/* Notify callback */
	if (gcov_callback != NULL)
		(*gcov_callback)(gcov_add, bb);
}


/* Unused functions needed to prevent linker errors. */
void __gcov_flush(void) {}
void __gcov_merge_add(gcov_type *counters, unsigned int n_counters) {}
void __gcov_merge_single(gcov_type *counters, unsigned int n_counters) {}
void __gcov_merge_delta(gcov_type *counters, unsigned int n_counters) {}
int __gcov_execve(const char *path, char *const argv[], char *const envp[])
{
	return kernel_execve(path, argv, envp);
}

EXPORT_SYMBOL_NOVERS(gcov_version);
EXPORT_SYMBOL_NOVERS(__gcov_init);
EXPORT_SYMBOL_NOVERS(__gcov_flush);
EXPORT_SYMBOL_NOVERS(__gcov_merge_add);
EXPORT_SYMBOL_NOVERS(__gcov_merge_single);
EXPORT_SYMBOL_NOVERS(__gcov_merge_delta);
EXPORT_SYMBOL_NOVERS(__gcov_execve);
#endif /* GCC_VERSION */


/* Call NUM constructors in CTOR. If defined, MODULE specifies that resulting
 * bb data be associated with module. */
void
do_global_ctors(ctor_t ctor[], unsigned long num, struct module *module)
{
	unsigned long i;

	down(&gcov_core_lock);
	if (module) {
		/* Create a context to associate struct bbs with this MODULE */
		current_context = (struct gcov_context *) kmalloc(
					sizeof(struct gcov_context) +
					num * sizeof(struct bb *),
					GFP_KERNEL);
		if (!current_context) {
			printk(KERN_WARNING GCOV_CORE_HEADER "not enough memory"
			       " for coverage data!\n");
			up(&gcov_core_lock);
			return;
		}
		current_context->module = module;
		current_context->count = 0;
		current_context->bb = (struct bb **) (current_context + 1);
		list_add(&current_context->list, &context_list);
	}
	/* Call constructors */
	for (i = 0; i < num && ctor[i]; i++)
		ctor[i]();
	current_context = NULL;
	up(&gcov_core_lock);
}


/* Remove data associated with MODULE. */
void
remove_bb_link(struct module *module)
{
	struct gcov_context* context;
	struct gcov_context* tmp;
	struct bb *bb;
	struct bb *prev;
	unsigned long i;

	down(&gcov_core_lock);
	/* Get associated context */
	context = NULL;
	list_for_each_entry(tmp, &context_list, list) {
		if (tmp->module == module) {
			context = tmp;
			break;
		}
	}
	if (!context) {
		up(&gcov_core_lock);
		return;
	}
	/* Remove all bb entries belonging to this module */
	prev = NULL;
	for (bb = bb_head; bb ; bb = bb->next) {
		for (i = 0; i < context->count; i++) {
			if (context->bb[i] == bb) {
				/* Detach bb from list. */
				if (prev)
					prev->next = bb->next;
				else
					bb_head = bb->next;
				/* Notify callback */
				if (gcov_callback)
					(*gcov_callback)(gcov_remove, bb);
				break;
			}
		}
		if (i == context->count)
			prev = bb;
	}
	list_del(&context->list);
	kfree(context);
	up(&gcov_core_lock);
}


static int __init
gcov_core_init(void)
{
	printk(KERN_INFO GCOV_CORE_HEADER "initializing core module: "
	       "format=%s\n", gcov_format);
	sema_init(&gcov_core_lock, 1);
	do_global_ctors(__CTOR_LIST__.ctor, __CTOR_LIST__.num, NULL);
	printk(KERN_INFO GCOV_CORE_HEADER "init done\n");
	return 0;
}

module_init(gcov_core_init);


EXPORT_SYMBOL_NOVERS(bb_head);
EXPORT_SYMBOL_NOVERS(gcov_sourcepath);
EXPORT_SYMBOL_NOVERS(gcov_objectpath);
EXPORT_SYMBOL_NOVERS(gcov_callback);
EXPORT_SYMBOL_NOVERS(gcov_core_lock);
EXPORT_SYMBOL_NOVERS(do_global_ctors);

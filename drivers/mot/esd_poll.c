/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
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

/*=============================================================================
                                INCLUDE FILES
=============================================================================*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <mot/esd_poll.h>

/*=============================================================================
                               LOCAL MACROS
=============================================================================*/
#define ESD_POLL_PRINTK(fmt, ...) \
   do { \
      if (esd_poll_debug) \
         printk(fmt, ##__VA_ARGS__); \
   } while(0)

/*=============================================================================
                         LOCAL FUNCTION PROTOTYPES
=============================================================================*/
static void esd_poll_work(struct work_struct *work);
static void esd_poll_copy_list(struct list_head* source,
   struct list_head* target);

/*=============================================================================
                  LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
=============================================================================*/
struct esd_poll
{
   struct list_head list;
   void (*handler)(void *arg);
   void *arg;
   const char* name;
};

/*=============================================================================
                              LOCAL VARIABLES
=============================================================================*/
static LIST_HEAD(esd_poll_list);
static LIST_HEAD(esd_poll_list_copy);
static DECLARE_MUTEX(esd_poll_list_lock);
static DECLARE_DELAYED_WORK(esd_poll_workqueue, esd_poll_work);
static ulong poll_freq = 6;
static bool copy_dirty = 0;
static bool esd_poll_debug = 0;

MODULE_PARM_DESC(poll_freq, "ESD recovery polling frequency. Default=6 sec");
module_param(poll_freq, ulong, 0);

/*=============================================================================
                             GLOBAL FUNCTIONS
=============================================================================*/
/*=============================================================================

FUNCTION: esd_poll_start

DESCRIPTION: start polling of provided ESD recovery function

ARGUMENTS PASSED:
   void (*esd_handler)(void *arg): pointer ESD recovery function
   void* arg: ESD recovery function arg
   const char* name: function string name

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

=============================================================================*/
void
esd_poll_start(void (*esd_handler)(void *arg), void* arg, const char* name)
{
   bool found_handler = 0;
   struct esd_poll *esd_poll;

   ESD_POLL_PRINTK("%s: enter\n", __FUNCTION__);
   if (esd_handler && name)
   {
      down(&esd_poll_list_lock);
      list_for_each_entry(esd_poll, &esd_poll_list, list)
      {
         if (esd_poll->handler == esd_handler)
         {
            ESD_POLL_PRINTK("%s: %s already in list\n", __FUNCTION__,
               esd_poll->name);
            found_handler = 1;
            break;
         }
      }
      if (!found_handler)
      {
         esd_poll = kzalloc(sizeof(*esd_poll), GFP_KERNEL);
         if (esd_poll)
         {
            esd_poll->handler = esd_handler;
            esd_poll->arg = arg;
            esd_poll->name = name;
            if (list_empty(&esd_poll_list))
            {
               schedule_delayed_work(&esd_poll_workqueue, poll_freq * HZ);
            }
            list_add_tail(&esd_poll->list, &esd_poll_list);
            copy_dirty = 1;
            ESD_POLL_PRINTK("%s: %s added\n", __FUNCTION__, esd_poll->name);
         }
         else
         {
            printk(KERN_ERR "%s: kzalloc(struct esd_poll) failed\n",
               __FUNCTION__);
         }
      }
      else
      {
         printk(KERN_ERR "%s: provided esd_handler arg already exists\n",
            __FUNCTION__);
      }
      up(&esd_poll_list_lock);
   }
   else if (!esd_handler)
   {
      printk(KERN_ERR "%s: provided esd_handler arg is NULL\n", __FUNCTION__);
   }
   else if (!name)
   {
      printk(KERN_ERR "%s: provided name arg is NULL\n", __FUNCTION__);
   }
   ESD_POLL_PRINTK("%s: exit\n", __FUNCTION__);
}

/*=============================================================================

FUNCTION: esd_poll_stop

DESCRIPTION: stop polling of provided ESD recovery function

ARGUMENTS PASSED:
   void (*esd_handler)(void *arg): pointer ESD recovery function

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

=============================================================================*/
void
esd_poll_stop(void (*esd_handler)(void *arg))
{
   struct esd_poll *esd_poll = NULL;
   struct esd_poll *esd_poll_tmp = NULL;

   ESD_POLL_PRINTK("%s: enter\n", __FUNCTION__);
   if (esd_handler)
   {
      down(&esd_poll_list_lock);
      list_for_each_entry_safe(esd_poll, esd_poll_tmp, &esd_poll_list, list)
      {
         if (esd_poll->handler == esd_handler)
         {
            ESD_POLL_PRINTK("%s: %s found and deleted\n", __FUNCTION__,
               esd_poll->name);
            list_del(&esd_poll->list);
            kfree(esd_poll);
            copy_dirty = 1;
            break;
         }
      }
      up(&esd_poll_list_lock);
   }
   else
   {
      printk(KERN_ERR "%s: provided esd_handler arg is NULL\n", __FUNCTION__);
   }
   ESD_POLL_PRINTK("%s: exit\n", __FUNCTION__);
}
 
/*=============================================================================
                             LOCAL FUNCTIONS
=============================================================================*/

/*=============================================================================

FUNCTION: esd_poll_work

DESCRIPTION: ESD poll work queue function

ARGUMENTS PASSED:
   struct work_struct *work: pointer to ESD poll work queue

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

=============================================================================*/
static void
esd_poll_work(struct work_struct *work)
{
   struct esd_poll *esd_poll;

   ESD_POLL_PRINTK("%s: enter\n", __FUNCTION__);
   /*
    * Copy the poll handlers list, so that we can iterate it and call the
    * poll handlers without holding the list lock. Holding the list lock
    * while calling the poll handlers can lead to a dead lock if the
    * handler uses it's own lock.
    */
   down(&esd_poll_list_lock);
   if (copy_dirty)
   {
      esd_poll_copy_list(&esd_poll_list, &esd_poll_list_copy);
      copy_dirty = 0;
   }
   up(&esd_poll_list_lock);

   if (!list_empty(&esd_poll_list_copy))
   {
      list_for_each_entry(esd_poll, &esd_poll_list_copy, list)
      {
         ESD_POLL_PRINTK("%s: calling ESD function - %s\n", __FUNCTION__,
            esd_poll->name);
         esd_poll->handler(esd_poll->arg);
      }
      schedule_delayed_work(&esd_poll_workqueue, poll_freq * HZ);
   }
   ESD_POLL_PRINTK("%s: exit\n", __FUNCTION__);
}

/*=============================================================================

FUNCTION: esd_poll_copy_list

DESCRIPTION: Copy source list to target list.

ARGUMENTS PASSED:
   struct list_head *source: source list head pointer
   struct list_head *target: target list head pointer

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

=============================================================================*/
static void
esd_poll_copy_list(struct list_head* source, struct list_head* target)
{
   struct esd_poll *esd_poll = NULL;
   struct esd_poll *esd_poll_tmp = NULL;

   /* Empty out the target list first */
   list_for_each_entry_safe(esd_poll, esd_poll_tmp, target, list)
   {
      list_del(&esd_poll->list);
      kfree(esd_poll);
   }

   /* Copy source list to target list */
   list_for_each_entry(esd_poll, source, list)
   {
      esd_poll_tmp = kzalloc(sizeof(*esd_poll_tmp), GFP_KERNEL);
      if (esd_poll_tmp)
      {
         esd_poll_tmp->handler = esd_poll->handler;
         esd_poll_tmp->arg = esd_poll->arg;
         esd_poll_tmp->name = esd_poll->name;
         list_add_tail(&esd_poll_tmp->list, target);
      }
   }
}

static int __init esd_poll_init(void)
{
   return 0;
}

static void __exit esd_poll_exit(void)
{
}

module_init(esd_poll_init);
module_exit(esd_poll_exit);

MODULE_AUTHOR("Ryan Johnson <ryan.johnson@motorola.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
MODULE_DESCRIPTION("ESD recovery poll driver");


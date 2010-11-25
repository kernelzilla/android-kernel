/*=============================================================================

   Module Name: esd_poll.c

   DESCRIPTION: Functions to handle ESD recovery polling.

===============================================================================

                       Motorola Confidential Proprietary
                   Advanced Technology and Software Operations
                (c) Copyright Motorola 2009, All Rights Reserved
  
                       Modification   Tracking
Author                    Date         Number    Description of Changes
---------------------  ------------  ----------  -----------------------------
Ryan Johnson (dgc483)  06/05/2009    LIBss56936  Created
=============================================================================*/

/*=============================================================================
                                INCLUDE FILES
=============================================================================*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <mot/esd_poll.h>

/*=============================================================================
                         LOCAL FUNCTION PROTOTYPES
=============================================================================*/
static void esd_poll_work(struct work_struct *work);

/*=============================================================================
                  LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
=============================================================================*/
struct esd_poll
{
   struct list_head list;
   void (*handler)(void *arg);
   void *arg;
};

/*=============================================================================
                              LOCAL VARIABLES
=============================================================================*/
static LIST_HEAD(esd_poll_list);
static DECLARE_MUTEX(esd_poll_list_lock);
static DECLARE_DELAYED_WORK(esd_poll_workqueue, esd_poll_work);
static ulong poll_freq = 6;

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

RETURN VALUE:
   None

DEPENDENCIES:
   None

SIDE EFFECTS:
   None

=============================================================================*/
void
esd_poll_start(void (*esd_handler)(void *arg), void* arg)
{
   bool found_handler = 0;
   struct esd_poll *esd_poll;

   if (esd_handler)
   {
      down(&esd_poll_list_lock);
      list_for_each_entry(esd_poll, &esd_poll_list, list)
      {
         if (esd_poll->handler == esd_handler)
         {
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
            if (list_empty(&esd_poll_list))
            {
               schedule_delayed_work(&esd_poll_workqueue, poll_freq * HZ);
            }
            list_add_tail(&esd_poll->list, &esd_poll_list);
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
   else
   {
      printk(KERN_ERR "%s: provided esd_handler arg is NULL\n", __FUNCTION__);
   }
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

   if (esd_handler)
   {
      down(&esd_poll_list_lock);
      list_for_each_entry(esd_poll, &esd_poll_list, list)
      {
         if (esd_poll->handler == esd_handler)
         {
            list_del(&esd_poll->list);
            kfree(esd_poll);
            break;
         }
      }
      up(&esd_poll_list_lock);
   }
   else
   {
      printk(KERN_ERR "%s: provided esd_handler arg is NULL\n", __FUNCTION__);
   }
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

   down(&esd_poll_list_lock);
   if (!list_empty(&esd_poll_list))
   {
      list_for_each_entry(esd_poll, &esd_poll_list, list)
      {
         esd_poll->handler(esd_poll->arg);
      }
      schedule_delayed_work(&esd_poll_workqueue, poll_freq * HZ);
   }
   up(&esd_poll_list_lock);
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


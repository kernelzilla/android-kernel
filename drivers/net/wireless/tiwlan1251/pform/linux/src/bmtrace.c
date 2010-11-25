/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/


#include <linux/config.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/proc_fs.h>

#include "osApi.h"
#include "esta_drv.h"
#include "bmtrace.h"

#define OS_READ_REG(drv,reg,p_val)    \
   os_hwReadMemRegisterUINT32(drv, (UINT32 *)((unsigned long)drv->acx_reg.va + reg), p_val)

typedef struct {
      unsigned long loc;/* trace entry identification */
      unsigned long ts;/* Timestamp */
      unsigned long p1; /* Parameter 1 */
      unsigned long p2; /* Parameter 2 */
} bm_entry_t;

typedef struct {
      int pos;
      int count;
      int print_pos;
      int nusers;
      unsigned long self_delay;
      tiwlan_net_dev_t *drv;
      bm_entry_t entry[1]; /* Array of entries */
} bm_control_t;

static bm_control_t *bm_control;

static inline int bm_control_size(void)
{
   return offsetof(bm_control_t, entry) + sizeof(bm_entry_t)*BM_NUM_ENTRIES;
}

static int bm_res_read_proc(char *page, char **start, off_t off,
			    int count, int *eof, void *data)
{
   int i;
   int len=0;
   int limit=count-80;
   int entry_count;
   unsigned long prev=0;
   int print_pos;

   print_pos = bm_control->print_pos++; /* It will disable tracing as well */

   entry_count = (bm_control->count > BM_NUM_ENTRIES) ? BM_NUM_ENTRIES : bm_control->count;

   /* Skip off entries */
   if ( print_pos >= entry_count) /* paranoid */
   {
      bm_control->pos = bm_control->count = bm_control->print_pos = 0;
      *eof = 1;
      return 0;
   }

   if (!off)
   {
      len = sprintf(page, "Events stored: %u   discarded: %u\n",
		    entry_count, bm_control->count-entry_count);
      len += sprintf(page+len, "loc delta      ts         p1         p2\n");
   }

   /* Initial index */
   if (bm_control->count > BM_NUM_ENTRIES)
      i = (bm_control->pos+print_pos-1)%BM_NUM_ENTRIES;
   else
      i = bm_control->print_pos-1;

   for(; (print_pos<entry_count) && (len<=limit); print_pos++)
   {
      bm_entry_t *bme= &bm_control->entry[i];
      len += sprintf(page+len,
		     "%-3lu %-10lu %-10lu %-10lu %-10lu\n",
		     bme->loc,
		     ((bme->ts-prev)>bm_control->self_delay)?bme->ts-prev-bm_control->self_delay:0,
		     bme->ts,
		     bme->p1, bme->p2);
      prev = bme->ts;
      ++i;
      i %= BM_NUM_ENTRIES;
   }
   if (print_pos >= entry_count)
   {
      *eof = 1;
      bm_control->pos = bm_control->count = bm_control->print_pos = 0;
   }
   else
      bm_control->print_pos = print_pos;
   return len;
}


/* Initialization */
int bm_init(struct tiwlan_net_dev *drv)
{
   if (bm_control)
   {
      ++bm_control->nusers;
      return 0;
   }
   bm_control = (bm_control_t *)kmalloc(bm_control_size(), GFP_KERNEL);
   if (!bm_control)
      return -ENOMEM;
   memset(bm_control, 0, offsetof(bm_control_t, entry) + sizeof(bm_entry_t)*BM_NUM_ENTRIES);
   bm_control->nusers = 1;
   bm_control->drv = drv;

   create_proc_read_entry("bmtrace", 0, NULL, bm_res_read_proc, NULL);
   /* Measure self-delay */
   bm_trace(0, 0, 0);
   bm_trace(0, 0, 0);
   bm_control->self_delay = bm_control->entry[1].ts - bm_control->entry[0].ts;
   bm_control->pos = bm_control->count = 0;
   print_info("%s: self_delay=%lu\n", __FUNCTION__, bm_control->self_delay);
   return 0;
}

/* De-initialization */
void bm_destroy(void)
{
   if (--bm_control->nusers)
      return;
   remove_proc_entry("bmtrace", NULL);
   kfree( bm_control );
}


/* Add trace entry. not safe, but will do */
void bm_trace(int loc, unsigned long p1, unsigned long p2)
{
   int pos;
   if (!bm_control || bm_control->print_pos)
      return;
   pos = bm_control->pos;
   bm_control->pos = (pos+1) % BM_NUM_ENTRIES;
   ++bm_control->count;

   bm_control->entry[pos].ts = os_timeStampUs(NULL);
   bm_control->entry[pos].loc= loc;
   bm_control->entry[pos].p1 = p1;
   bm_control->entry[pos].p2 = p2;
}


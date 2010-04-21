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


#ifndef ESTA_DRV_INCLUDE_FILE
#define ESTA_DRV_INCLUDE_FILE

#include <linux/version.h>
#include <linux/completion.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#include <asm/arch/gpio.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#else
#include <asm/gpio.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#endif

#ifdef CONFIG_TROUT_PWRSINK
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#include <asm/arch/trout_pwrsink.h>
#else
#include <mach/trout_pwrsink.h>
#endif
#endif

#ifdef CONFIG_HTC_PWRSINK
#include <mach/htc_pwrsink.h>
#endif

#include "osTIType.h"
#include "osAdapter.h"
#include "paramOut.h"
#ifndef GWSI_DRIVER /* for GWSI Stand Alone */
#include "configMgr.h"
#endif
#include "linux_ioctl_common.h"

#define TIWLAN_DRV_NAME    "tiwlan"
#define TIWLAN_DRV_IF_NAME TIWLAN_DRV_NAME"%d"
#define TIWLAN_DRV_NAME_WIRELESS_PROTO "IEEE 802.11-DS"
#define TIWLAN_DBG_PROC    "wifidbg"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
#define NETDEV_SET_PRIVATE(dev, drv)	dev->priv = drv
#define NETDEV_GET_PRIVATE(dev)		dev->priv
#else
#define NETDEV_SET_PRIVATE(dev, drv)	dev->ml_priv = drv
#define NETDEV_GET_PRIVATE(dev)		dev->ml_priv
#endif   

void *wifi_kernel_prealloc(int section, unsigned long size);

#ifdef TIWLAN_MSM7000
#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/platform_device.h>
#include <linux/wifi_tiwlan.h>
int msm_wifi_power(int on);
int msm_wifi_reset(int on);
#else
extern int trout_wifi_power(int on);
extern int trout_wifi_reset(int on);
extern int trout_wifi_set_carddetect(int val);
#define msm_wifi_power(a)       trout_wifi_power(a)
#define msm_wifi_reset(a)       trout_wifi_reset(a)
#endif
#endif

#ifdef CONFIG_HAS_WAKELOCK
#define CONFIG_ANDROID_POWER
typedef struct wake_lock android_suspend_lock_t;
#define android_unlock_suspend(a)             wake_unlock(a)
#define android_lock_suspend(a)               wake_lock(a)
#define android_lock_suspend_auto_expire(a,t) wake_lock_timeout(a,t)
#define android_init_suspend_wakelock(a,n)    wake_lock_init(a,WAKE_LOCK_SUSPEND,n)
#define android_uninit_suspend_lock(a)        wake_lock_destroy(a)
#endif

#ifndef TIWLAN_OMAP1610_REGBASE

#if defined(TIWLAN_OMAP1610_INNOVATOR)
#define TIWLAN_OMAP1610_REGBASE 0xEC100000    /* VA*/
#elif defined(TIWLAN_OMAP1610_WIPP) || defined(TIWLAN_OMAP1610_CRTWIPP)
#ifndef OMAP_WLAN_BASE
#define OMAP_WLAN_BASE  0
#endif
#define TIWLAN_OMAP1610_REGBASE (OMAP_WLAN_BASE+0x100000)
#else
/* Dm: #error TIWLAN_OMAP1610_REGBASE not defined for this platform */
#endif

#endif /* #ifndef TIWLAN_OMAP1610_REGBASE */

#ifdef TIWLAN_OMAP1610_REGBASE

#define TIWLAN_OMAP1610_REGSIZE 0x10000
#define TIWLAN_OMAP1610_MEMBASE (TIWLAN_OMAP1610_REGBASE | 0x20000)
#define TIWLAN_OMAP1610_MEMSIZE 0x10000

#endif /* #ifdef TIWLAN_OMAP1610_REGBASE */

#ifndef TIWLAN_OMAP1610_IRQ

#if defined(TIWLAN_OMAP1610_INNOVATOR)
#define TIWLAN_OMAP1610_IRQ     (OMAP_GPIO_IRQ(8))
#elif defined(TIWLAN_OMAP1610_CRTWIPP)
#define TIWLAN_OMAP1610_IRQ     (OMAP_GPIO_IRQ(2))
#elif defined(TIWLAN_OMAP1610_WIPP)
#define TIWLAN_OMAP1610_IRQ     (OMAP_GPIO_IRQ(2))
#endif

#endif /* TIWLAN_OMAP1610_IRQ */
#define TIWLAN_IRQ_POLL_INTERVAL  HZ/100  /* Used when no Intr are handled from the FW */


#ifdef TI_DBG
#define ti_dprintf(log, fmt, args...) do { \
   if (log != TIWLAN_LOG_OTHER) {   \
      printk(KERN_INFO fmt, ## args); \
   } \
} while (0)
#else
#define ti_dprintf(log, fmt, args...)
#endif


#define ti_nodprintf(log, fmt, args...)

typedef enum {
   TIWLAN_LOG_ERROR,
   TIWLAN_LOG_INFO,
   TIWLAN_LOG_OTHER,
   TIWLAN_LOG_DUMMY
} tiwlan_log_t;

typedef struct tiwlan_region {
      unsigned long pa;
      void *va;
      unsigned long size;
} tiwlan_region_t;

/* Driver structure */
typedef struct tiwlan_net_dev tiwlan_net_dev_t;

#if defined (__KERNEL__)
/* Request/response to/from control tasklet */
typedef struct tiwlan_req {
      struct list_head list;
      tiwlan_net_dev_t *drv;
      struct
      {
            struct {
                  int (*f)(struct tiwlan_req *req);
                  unsigned long p1;
                  unsigned long p2;
                  unsigned long p3;
                  unsigned long p4;
                  int reply_expected;     /* 1=reply expected */
                  struct completion comp;
            } req;
            int reply;
      } u;
} tiwlan_req_t;

/* Timer structure */
typedef struct timer_obj {
      tiwlan_req_t req;
         /* Timer handler function ->p1 */
         /* Timer handler function parameter ->p2 */
         /* Periodic ->p3 */
         /* jiffies ->p4 */
      struct timer_list  timer;
      int use_count;
} timer_obj_t;


#ifdef DRIVER_PROFILING

/* Profiler banchmark function type */
typedef void (*tiwlan_pfofile_t) (void*, unsigned);

/* Maximum number of profile banchmarks */
#define MAX_PROFILE_BM 10

#endif

#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
#define PWRSINK_WIFI_PERCENT_BASE 4
#endif

/* #define DM_USE_WORKQUEUE */

/* Driver structure */
struct tiwlan_net_dev {
      struct list_head list;

      tiwlan_region_t acx_mem;
      tiwlan_region_t acx_reg;
      tiwlan_region_t eeprom_image;
      tiwlan_region_t firmware_image;
      int irq;
#ifdef DM_USE_WORKQUEUE
      struct workqueue_struct *tiwlan_wq; /* Work Queue */
      struct work_struct tirq;         /* Work Task for interrupt */
      struct work_struct tw;           /* Work Task for other stuff */
      mem_MSDU_T *txmit_msdu_next;
      mem_MSDU_T *txmit_msdu_last;
      struct work_struct txmit;        /* Work Task for transmit */
#else
      struct tasklet_struct tl;        /* Control tasklet */
#endif
#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
      struct delayed_work trxw;        /* Work Task for rx watchdog */
#endif
#ifdef CONFIG_ANDROID_POWER
      android_suspend_lock_t irq_wake_lock; /* Suspend Lock to keep system awake */
      android_suspend_lock_t xmit_wake_lock;
      android_suspend_lock_t timer_wake_lock;
      android_suspend_lock_t rx_wake_lock;
      android_suspend_lock_t exec_wake_lock;
      int receive_packet;              /* 1=packet was pushed to network stack */
#endif
      int interrupt_pending;           /* 1=tasklet has to handle interrupts when awakes */
      int dma_done;
      struct list_head request_q;      /* Requests queue: -> tasklet */
      spinlock_t lock;
      unsigned long flags;
      struct timer_list poll_timer;    /* Polling timer. Used only when working without interrupts */
      int started;                     /* 1=config manager started. 0=config manager stopped */
      int initialized;                 /* 1=succeeded to pass init stage, 0=otherwise */
      int unload_driver;               /* Driver unload indicator */
      struct net_device_stats stats;
      int alloc_msdu_failures;         /* Extra statistics */
      initTable_t  init_table;
      TIWLN_ADAPTER_T adapter;
      struct sock *wl_sock;
      struct completion comp;
      struct net_device *netdev;
      void *priv;                      /* Pointer to struct net_device */

#ifdef DRIVER_PROFILING
      tiwlan_pfofile_t fpro [MAX_PROFILE_BM];   /* Profiler functions */

      unsigned max_number_of_timers;            /* Maximum number of allocated timers */
      unsigned cur_number_of_timers;            /* Current number of allocated timers */

      unsigned max_heap_bytes_allocated;        /* Maximum number of allocated bytes on the heap */
      unsigned cur_heap_bytes_allocated;        /* Current number of allocated bytes on the heap */
      unsigned max_buf_bytes_allocated;         /* Maximum number of allocated bytes on the heap for TX/RX buffers */

      unsigned cpu_usage_estimator_start_time;  /* Time measured when CPU estimation was started */
      unsigned cpu_usage_estimator_stop_time;   /* Time measured when CPU estimation was stopped */
      unsigned cpu_usage_estimator_resolution;  /* Resolution of the CPU estimation in us */
      unsigned total_us_of_cpu_time;            /* Total number of CPU time used by the driver since CPU estimator started */
      unsigned total_us_of_bus_access_cpu_time; /* Total number of CPU time used by the bus driver since CPU estimator started */
      unsigned driver_entry_start_time;         /* Time measured at the start of last driver entry point */
      unsigned bus_driver_entry_start_time;     /* Time measured at the start of last bus driver entry point */
#endif

#ifdef GWSI_DRIVER
      void *gwsi;                      /* GWSI manager handler */
      void *gwsi_ev;                   /* GWSI event handler */
      char  gwsi_tester_buf [4096];    /* GWSI tester buffer */
#endif
};


#define VENDOR_ID_TI            0x104c
#define DEVICE_ID_TI_WLAN       0x9066

/* tiwlan_send_wait_reply
   This internal interface function creates request and sends
   it to the control tasklet for processing.
   The calling process is blocked until the request is replied.
   Function f is being called in the context of the control tasklet.
   The request block that is passed to the function as a parameter
   contains p1, p2, p3, p4.
   The function return code is propagated back to the caller.
   tiwlan_send_req_and_wait returns (*f) return code or
   -ENOMEM if failed to allocate a request.
*/
int tiwlan_send_wait_reply(tiwlan_net_dev_t *drv,
                           int (*f)(tiwlan_req_t *req),
                           unsigned long p1,
                           unsigned long p2,
                           unsigned long p3,
                           unsigned long p4);

#endif /* #if defined (__KERNEL__)*/

int tiwlan_init_drv(tiwlan_net_dev_t *drv, tiwlan_dev_init_t *init_info);
int tiwlan_start_drv(tiwlan_net_dev_t *drv);
int tiwlan_stop_drv(tiwlan_net_dev_t *drv);
int tiwlan_stop_and_destroy_drv(tiwlan_net_dev_t *drv);
int tiwlan_stop_and_destroy_drv_request(tiwlan_req_t *req);

#endif /* ESTA_DRV_INCLUDE_FILE*/

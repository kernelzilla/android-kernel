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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <net/sock.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/netdevice.h>
#include <linux/ioctl.h>
#include <linux/wireless.h>
#include <linux/etherdevice.h>
#include <linux/netlink.h>
#include <linux/completion.h>

#ifdef TIWLAN_CARDBUS
#include <linux/pci.h>
#else
#ifdef TIWLAN_OMAP1610
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#include <asm/arch-omap/tc.h>
#else
#include <mach/tc.h>
#endif
#endif
#ifdef TIWLAN_MSM7000
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#include <asm/arch/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#else
#include <mach/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#endif
#endif   /* !TIWLAN_CARDBUS */

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/if_arp.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>

#include "esta_drv.h"
#include "srcApi.h"
#include "osApi.h"
#include "whalHwRegs.h"

#if defined(DEBUG_UNKNOWN_INTERRUPT)
#define _STRING_H
#include "configMgr.h"
#include "whalCtrl.h"
#endif

#include "bmtrace.h"
#include "osrgstry_parser.h"
#include "osClsfr.h"
#include "TI_IPC_Api.h"
#include "802_11Defs.h"
#include "Ethernet.h"
#include "tiwlan_profile.h"
#include "ioctl_utils.h"

#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
#define RX_RATE_INTERVAL_SEC 10
unsigned long num_rx_pkt_new = 0;
static unsigned long num_rx_pkt_last = 0;
#endif

#ifdef TIWLAN_MSM7000
extern unsigned char *get_wifi_nvs_ram(void);
extern void SDIO_SetFunc( struct sdio_func * );
static struct proc_dir_entry *tiwlan_calibration;
static struct completion sdio_wait;
#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct wifi_platform_data *wifi_control_data = NULL;
#endif
#endif

/* WiFi chip information functions */
int export_wifi_fw_version( tiwlan_net_dev_t *drv );
int export_wifi_chip_id( void );

/* Drivers list */
static LIST_HEAD(tiwlan_drv_list);

/* debug memory access */
static struct proc_dir_entry *tiwlan_deb_entry;
static __u32 memdebug_addr;
static __u32 memdebug_size=1;
static __u32 memdebug_trans_size;

#define DRV_SHUTDOWN_TEST_DELAY_INTERVAL 100       /* Time in msec to "delay"(/sleep) while waiting for SME to shutdown */
#define DRV_SHUTDOWN_TEST_MAX_COUNTER  20          /* How many delay/sleep iterations to perform while waiting for SME to shutdown) */

MODULE_DESCRIPTION("TI WLAN Embedded Station Driver");
MODULE_LICENSE("GPL");

extern int packed_struct_tst(void);
extern int proc_stat_init(TI_HANDLE);
extern int proc_stat_destroy(void);

typedef void (* tiwlan_drv_isr_t)(int, void *, struct pt_regs *);

/* network device driver interface */
static int tiwlan_drv_net_open(struct net_device * dev);
static int tiwlan_drv_net_stop(struct net_device * dev);
static int tiwlan_drv_net_xmit(struct sk_buff * skb, struct net_device * dev);
static struct net_device_stats * tiwlan_drv_net_get_stats(struct net_device * dev);

#define OS_WRITE_REG(drv,reg,val)   \
    os_hwWriteMemRegisterUINT32(drv, (UINT32 *)((unsigned long)drv->acx_reg.va + reg), (__u32)(val))

#define OS_READ_REG(drv,reg,val)    \
    os_hwReadMemRegisterUINT32(drv, (UINT32 *)((unsigned long)drv->acx_reg.va + reg), &val)

#ifdef TIWLAN_OMAP1610
static void omap_memif_init(void)
{
    printk ("First function offset is: %p\n", omap_memif_init);
#if defined(TIWLAN_OMAP1610_INNOVATOR)
    print_info("Setting CS1 Ref Clock = TC/4. \n");
    omap_writel(0x00000004, 0xFFFECC40 ); /* wlan change for cs2 to dynamic wait state */
    omap_writel(0x0000113a, 0xFFFECC18 ); /* EMIFS (nCS2) configuration */
#elif defined(TIWLAN_OMAP1610_WIPP) || defined(TIWLAN_OMAP1610_CRTWIPP)

#if defined(TIWLAN_OMAP1610_CRTWIPP)
    /*
    Init the GPIO to output*/

    /* Set OMAP pin H19 to GPIO57*/

    omap_writel(omap_readl(0xFFFE1014) | 0x00E00000, 0xFFFE1014 );

    /*ELP_REQ (GPIO_57) by GPIO_DIRECTION - set it as output*/
    omap_writel(omap_readl(0xFFFBBC34) & (~0x00000200), 0xFFFBBC34 );
#endif  /* TIWLAN_OMAP1610_CRTWIPP */

/* The below configuration enables GPIO25 and GPIO_27 as output GPIOs - for debug purposes */
#if defined(TIWLAN_OMAP1610_CRTWIPP_GPIO_DEBUG)

    omap_writel(omap_readl(0xFFFE1030) | 0x00000E00, 0xFFFE1030 );/* enable GPIO25 */
    omap_writel(omap_readl(0xFFFE1030) | 0x00000038, 0xFFFE1030 );/* enable GPIO27 */

    omap_writel(omap_readl(0xFFFBEC34) & (~0x00000200), 0xFFFBEC34 );/* Setting direction (as output) for GPIO25 */
    omap_writel(omap_readl(0xFFFBEC34) & (~0x00000800), 0xFFFBEC34 );/* Setting direction (as output) for GPIO27 */
#endif   /* TIWLAN_OMAP1610_CRTWIPP_GPIO_DEBUG */


    /* RECOVERY*/
    print_info("Hard reset,perform PMEN toggle\n");
    os_hardResetTnetw();

    print_info("Setting CS2 Ref Clock = TC/2. \n");
    __raw_writel(0x1, TIWLAN_OMAP1610_REGBASE+0x4cc); /* CLK=80MHz */
    omap_writel(0x20, EMIF_CFG_DYNAMIC_WS); /* Full handshake on CS2 */
    omap_writel(0x2441, EMIFS_CS2_CONFIG); /* 0x2021 on reworked board */
    omap_writel(0, EMIFS_ACS2);

    print_info("%x=0x%lx\n", 0xFFFECC40, omap_readl(0xFFFECC40) );
    print_info("%x=0x%lx\n", 0xFFFECC18, omap_readl(0xFFFECC18) );
    print_info("%x=0x%lx\n", 0xFFFECC58, omap_readl(0xFFFECC58) );
#endif /* WIPP, CRTWIPP */
}
#endif

static int tiwlan_register_events(tiwlan_net_dev_t *drv)
{
    IPC_EVENT_PARAMS evParams;
    int i = 0;

    evParams.uDeliveryType      = DELIVERY_PUSH;
    evParams.uProcessID         = 0;
    evParams.uEventID           = 0;
    evParams.hUserParam        = drv;
    evParams.pfEventCallback    = os_IndicateEvent;


    for (;i < IPC_EVENT_MAX_OS_EVENT;i++)
    {
        evParams.uEventType = i;

        configMgr_RegisterEvent(drv->adapter.CoreHalCtx,(PUCHAR) &evParams,sizeof(IPC_EVENT_PARAMS));
    }

    return OK;
}

static int tiwlan_deb_read_proc(char *page, char **start, off_t off,
                                int count, int *eof, void *data)
{
    __u32 addr=memdebug_addr;
    __u32 size=memdebug_size;
    __u32 trans_size=memdebug_trans_size;
    __u32 end;
    int in_line=0, max_in_line;
    int limit=count-80;
    int i=0;
    static int toggle;

    *eof = 1;
    if (!addr || !trans_size)
        return 0;

    /* fixme: add address validation */

    if (!size)
        size=1;

    end = addr + size*trans_size;
    if (trans_size==4)
        max_in_line = 4;
    else if (trans_size==2)
        max_in_line = 8;
    else
        max_in_line = 16;

    while(i<limit && addr<end)
    {
        if (!in_line)
            i += sprintf(page+i, "0x%08x: ", addr);
        if (trans_size==4)
        {
            i += sprintf(page+i, "0x%08x", *(__u32 *)addr);
            addr += 4;
        }
        else if (trans_size==2)
        {
            i += sprintf(page+i, "0x%04x", *(__u16 *)addr);
            addr += 2;
        }
        else
        {
            i += sprintf(page+i, "0x%02x", *(__u8 *)addr);
            addr += 1;
        }
        if (++in_line < max_in_line)
            *(page+i++)=' ';
        else
        {
            *(page+i++)='\n';
            in_line = 0;
        }
    }
    *(page+i++)='\n';
    /* For some reason read proc is get called twice for
       each "cat" operation
    */
    if (toggle)
        memdebug_addr = addr;
    toggle = !toggle;

    return i;
}

static char *rm_get_token(const char **p_buffer, unsigned long *p_buffer_len,
                          char *token, unsigned long token_len,
                          char del)
{
    const char *buffer=*p_buffer;
    __u32 buffer_len = *p_buffer_len;

    while(buffer_len && token_len && *buffer!=del && *buffer)
    {
        *token++ = *buffer++;
        --buffer_len;
        --token_len;
    }
    while (buffer_len && *buffer==del)
    {
        ++buffer;
        --buffer_len;
    }
    *token = 0;
    *p_buffer = buffer;
    *p_buffer_len = buffer_len;
    return token;
}

static int tiwlan_deb_write_proc(struct file *file, const char *buffer,
                                 unsigned long count, void *data)
{
    __u32 addr, size;
    char token[15];
    __u32 value;
    char *end;
    int buflen=count;

    /* buffer format is:
       d{w,h,b} addr[/size]
       s{w,h,b} addr=value
    */
    /* Parse string */
    rm_get_token(&buffer, &count, token, sizeof(token)-1, ' ');
    if (token[0]=='d')
    {
        /* Display */
        if (!strcmp(token, "dw"))
            memdebug_trans_size = 4;
        else if (!strcmp(token, "dh"))
            memdebug_trans_size = 2;
        else if (!strcmp(token, "db"))
            memdebug_trans_size = 1;
        else
        {
            printk(KERN_INFO "rm: mem file write op is dw|dh|db|sw|sh|sb\n");
            return buflen;
        }
        /* Get address */
        rm_get_token(&buffer, &count, token, sizeof(token)-1, '/');
        addr = simple_strtoul(token, &end, 0);
        if ((end && *end) /* || !iopa(addr)*/)
        {
            printk(KERN_INFO "rm: address <%s> is invalid\n", token);
            return buflen;
        }
        if ((addr & (memdebug_trans_size-1)))
        {
            printk(KERN_INFO "rm: warning: address 0x%x is not aligned to size %u\n",
                   addr, memdebug_trans_size);
        }
        memdebug_addr = addr;
        if (count)
        {
            /* Get size */
            rm_get_token(&buffer, &count, token, sizeof(token)-1, ' ');
            size = simple_strtoul(token, &end, 0);
            if (end && *end)
            {
                printk(KERN_INFO "rm: size <%s> is invalid. end=<%s>\n",
                       token, end);
                return buflen;
            }
            memdebug_size = size;
        }
        return buflen;
    }
    if (token[0]=='s')
    {
        /* Display */
        if (!strcmp(token, "sw"))
            size = 4;
        else if (!strcmp(token, "sh"))
            size = 2;
        else if (!strcmp(token, "sb"))
            size = 1;
        else
        {
            printk(KERN_INFO "rm: mem file write op is dw|dh|db|sw|sh|sb\n");
            return buflen;
        }
        /* Get address */
        rm_get_token(&buffer, &count, token, sizeof(token)-1, ' ');
        addr = simple_strtoul(token, &end, 0);
        if ((end && *end) /*|| !iopa(addr)*/)
        {
            printk(KERN_INFO "rm: address <%s> is invalid\n", token);
            return buflen;
        }
        if ((addr & (size-1)))
        {
            printk(KERN_INFO "rm: warning: address 0x%x is not aligned to size %u\n",
                   addr, size);
        }

        /* Get value */
        rm_get_token(&buffer, &count, token, sizeof(token)-1, ' ');
        value = simple_strtoul(token, &end, 0);
        if (end && *end)
        {
            printk(KERN_INFO "rm: value <%s> is invalid. end <%s>\n",
                   token, end);
            return buflen;
        }
        if (size==4)
            *(__u32 *)addr = value;
        else if (size==2)
        {
            if (value > 0xffff)
            {
                printk(KERN_INFO "rm: value <%s> is out of range\n", token);
                return buflen;
            }
            *(__u16 *)addr = value;
        }
        else
        {
            if (value > 0xff)
            {
                printk(KERN_INFO "rm: value <%s> is out of range\n", token);
                return buflen;
            }
            *(__u8 *)addr = value;
        }
        memdebug_addr = addr;
        memdebug_size = 1;
        memdebug_trans_size = size;
    }
    else
        printk(KERN_INFO "rm: operation <%s> is not supported\n", token);
    return buflen;
}

#ifdef TIWLAN_MSM7000
#define WIFI_NVS_LEN_OFFSET     0x0C
#define WIFI_NVS_DATA_OFFSET    0x40
#define WIFI_NVS_MAX_SIZE       0x800UL

static unsigned long tiwlan_get_nvs_size( void )
{
    unsigned char *ptr;
    unsigned long len;

    ptr = get_wifi_nvs_ram();
    if( ptr == NULL ) {
        return 0;
    }
    /* Size in format LE assumed */
    memcpy( (void *)&len, (void *)(ptr + WIFI_NVS_LEN_OFFSET), sizeof(len) );
    len = min( len, (WIFI_NVS_MAX_SIZE-WIFI_NVS_DATA_OFFSET) );
    return len;
}

static int tiwlan_calibration_read_proc(char *page, char **start, off_t off,
                                int count, int *eof, void *data)
{
    unsigned char *ptr;
    unsigned long len;

    ptr = get_wifi_nvs_ram();
    if( ptr == NULL ) {
        return 0;
    }
    len = tiwlan_get_nvs_size();
    /* i += sprintf(page+i, "WiFi Calibration Size = %lu %x bytes\n", len); */
    memcpy( (void *)page, (void *)(ptr + WIFI_NVS_DATA_OFFSET), len );
    return len;
}

static int tiwlan_calibration_write_proc(struct file *file, const char *buffer,
                                 unsigned long count, void *data)
{
    return 0;
}
#endif

/*********************************************************************************************/
/*                                      Impelementation                                      */
/*********************************************************************************************/

static int tiwlan_drv_net_open(struct net_device * dev)
{
   tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev);

   ti_nodprintf(TIWLAN_LOG_INFO, "tiwlan_drv_net_open()\n");

   if (!drv->adapter.CoreHalCtx)
      return -ENODEV;

   netif_start_queue(dev);

   return 0;
}


static int tiwlan_drv_net_stop(struct net_device * dev)
{
   ti_nodprintf(TIWLAN_LOG_ERROR, "tiwlan_drv_net_stop()\n");

   netif_stop_queue(dev);

   return 0;
}


/* dummy send packet from Linux TCP/IP stack to WLAN
   Used when driver is not initialized
 */
static int tiwlan_drv_dummy_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
   /* Network stack takes care of deallocation */
   return -ENODEV;
}

void sendFreeFunc(TI_HANDLE pSkb, TI_HANDLE dummy1, TI_STATUS status)
{
    struct sk_buff *skb = (struct sk_buff *) pSkb;

    /* print_deb("^^^ free %p %d  bytes (%s)\n", skb->data, skb->len, (status==OK) ? "OK" : "ERROR" ); */
    dev_kfree_skb(skb);
}

#ifdef DM_USE_WORKQUEUE
void tiwlan_add_msdu(tiwlan_net_dev_t *drv, mem_MSDU_T *pMsdu)
{
    if( pMsdu == NULL )
        return;
    pMsdu->msdu_next = NULL;
    if( drv->txmit_msdu_next != NULL ) {
        drv->txmit_msdu_last->msdu_next = pMsdu;
    }
    else {
        drv->txmit_msdu_next = pMsdu;
    }
    drv->txmit_msdu_last = pMsdu;
}

mem_MSDU_T *tiwlan_del_msdu(tiwlan_net_dev_t *drv)
{
    mem_MSDU_T *pMsdu = NULL;

    if( drv->txmit_msdu_next != NULL ) {
        pMsdu = drv->txmit_msdu_next;
        drv->txmit_msdu_next = pMsdu->msdu_next;
        if( drv->txmit_msdu_next == NULL ) { /* Last MSDU */
            drv->txmit_msdu_last = NULL;
        }
    }
    return( pMsdu );
}

static void tiwlan_xmit_handler( struct work_struct *work )
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)container_of( work, struct tiwlan_net_dev, txmit );
    mem_MSDU_T *pMsdu;
    unsigned long flags;

#ifdef CONFIG_ANDROID_POWER
    android_lock_suspend( &drv->exec_wake_lock );
    android_unlock_suspend( &drv->xmit_wake_lock );
#endif
    /* printk("TI: %s:\t%lu\n", __FUNCTION__, jiffies); */
    do {
        spin_lock_irqsave(&drv->lock, flags);
        pMsdu = tiwlan_del_msdu(drv);
        spin_unlock_irqrestore(&drv->lock, flags);
        if( pMsdu ) {
            configMgr_sendMsdu(drv->adapter.CoreHalCtx, pMsdu, 0);
        }
    } while( pMsdu != NULL );
#ifdef CONFIG_ANDROID_POWER
    android_unlock_suspend( &drv->exec_wake_lock );
#endif
}
#endif

/* send packet from Linux TCP/IP stack to WLAN
 */
static int tiwlan_drv_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev);
    int status;
    mem_MSDU_T *pMsdu;
    UINT32      packetHeaderLength;

#ifndef NO_COPY_SKB
    char *pMsduData;
#else
    mem_BD_T *pCurBd=0;
#endif

#ifdef DRIVER_PROFILE
    os_profile (drv, 0, 0);
#endif
    bm_trace(20, skb->len, 0);

#ifdef NO_COPY_SKB

    status = configMgr_allocMSDUBufferOnly(drv->adapter.CoreHalCtx, &pMsdu, OS_ABS_TX_MODULE);
    if(status != OK)
    {
        ti_dprintf(TIWLAN_LOG_ERROR, " configMgr_allocMSDUBufferOnly failed !!!\n");
        ++drv->alloc_msdu_failures;
        return -ENOMEM;
    }
    /* print_deb("$$$ configMgr_allocMSDUBufferOnly()=OK pMsdu=%p\n", pMsdu ); */

    status = configMgr_allocBDs(drv->adapter.CoreHalCtx, 1, &pCurBd);

    if(status != OK) {
        ++drv->alloc_msdu_failures;
        ti_dprintf(TIWLAN_LOG_ERROR, "  configMgr_allocBDs failed !!!\n");
        configMgr_memMngrFreeMSDU(drv->adapter.CoreHalCtx, pMsdu->handle);
        return -ENOMEM;
    }
    /* print_deb("$$$ configMgr_allocBDs()=OK pCurBd=%p first=%p\n", pCurBd, pMsdu->firstBDPtr ); */

    pMsdu->freeFunc = sendFreeFunc;
    pMsdu->freeArgs[0] = (UINT32) skb;
    pMsdu->dataLen = skb->len;
    pMsdu->firstBDPtr = pCurBd;
    pCurBd->dataOffset = skb->data-skb->head;
    pCurBd->length = skb->len;
    pCurBd->data = skb->head;

    drv->stats.tx_packets++;
    drv->stats.tx_bytes += skb->len;

#else /* NO_COPY_SKB */

    /* 
     * Retrieve the Packet Header length 
     * from QoS Manager (through configMgr) 
     * (Header type is determined upon association)  
     */
    packetHeaderLength = configMgr_getPacketHeaderLength(drv->adapter.CoreHalCtx,skb->data,TX_DATA_DATA_MSDU);

   /*
    * need to reserve enough space for header translation 
    * in the same first Bd.
    * Allocate enough place also for 802.11 header (24 bytes or 26 for QoS) and LLC (8 bytes)
    * to replace the Ethernet header (14 bytes)
    */
    status = configMgr_allocMSDU(drv->adapter.CoreHalCtx, &pMsdu,
                                     skb->len + packetHeaderLength, OS_ABS_TX_MODULE);

    if(status != OK)
    {
        /*ti_dprintf(TIWLAN_LOG_ERROR, " configMgr_allocMSDU failed !!!\n");*/
        ++drv->alloc_msdu_failures;
        return -ENOMEM;
    }

    /* 
     * case 1: only legacy wlan header 
     *
     * case 2: only QoS wlan header 
     *
     * case 3: only legacy wlan header with new snap
     *
     * case 4: only QoS wlan header with new snap
     */
    pMsdu->firstBDPtr->dataOffset = packetHeaderLength - ETHERNET_HDR_LEN;
    pMsduData = pMsdu->firstBDPtr->data + pMsdu->firstBDPtr->dataOffset;
    memcpy(pMsduData, skb->data, skb->len);
    pMsdu->dataLen = skb->len;
    pMsdu->firstBDPtr->length = pMsdu->dataLen + pMsdu->firstBDPtr->dataOffset;

    drv->stats.tx_packets++;
    drv->stats.tx_bytes += skb->len;
    dev_kfree_skb(skb);
#endif /* NO_COPY_SKB */

    pMsdu->txFlags |= TX_DATA_FROM_OS;
    pMsdu->qosTag = 0;
    status = OK;

#ifdef TI_DBG
    /* Set packet-os-in time stamp */
    /* TODO: the skb time stamp is not good */
    /* printk ("\n### sec=%u, usec=%u", skb->stamp.tv_sec, skb->stamp.tv_usec);*/
    /* pMsdu->timeStamp[0] = skb->stamp.tv_sec * 1000000 + skb->stamp.tv_usec; */
    /* pMsdu->timeStampNum = 1; */
#endif

    bm_trace(21, 0, 0);
   /*
    * Propagate Msdu through Config Manager.
    * Set DTag to zero
    * (note that classification is further handled in the Core)
    */
    if (status == OK) {
#ifdef DM_USE_WORKQUEUE
        unsigned long flags;

        spin_lock_irqsave(&drv->lock, flags);
        tiwlan_add_msdu(drv, pMsdu);
        spin_unlock_irqrestore(&drv->lock, flags);
        /* printk("TI: %s:\t%lu\n", __FUNCTION__, jiffies); */
#ifdef CONFIG_ANDROID_POWER
        android_lock_suspend( &drv->xmit_wake_lock );
#endif
        queue_work( drv->tiwlan_wq, &drv->txmit );
#else
        status = configMgr_sendMsdu(drv->adapter.CoreHalCtx, pMsdu, 0);
#endif
    }
    else
        configMgr_memMngrFreeMSDU (drv->adapter.CoreHalCtx, (UINT32) pMsdu); /* If status != OK , we won't send the MSDU, so we need to free it */

    if(unlikely(status != OK))
    {
        drv->stats.tx_errors++;
#ifdef NO_COPY_SKB
        dev_kfree_skb(skb);
#endif
    }

    bm_trace(22, 0, 0);
#ifdef DRIVER_PROFILE
    os_profile (drv, 1, 0);
#endif

    return 0;
}

struct net_device_stats * tiwlan_drv_net_get_stats(struct net_device * dev)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)NETDEV_GET_PRIVATE(dev);
    ti_dprintf(TIWLAN_LOG_OTHER, "tiwlan_drv_net_get_stats()\n");

    return &drv->stats;
}

static const struct net_device_ops tiwlan_drv_net_dev_ops =
{
    .ndo_open = tiwlan_drv_net_open,
    .ndo_stop = tiwlan_drv_net_stop,
    .ndo_start_xmit = tiwlan_drv_net_xmit,
    .ndo_get_stats = tiwlan_drv_net_get_stats,
    .ndo_do_ioctl = ti1610_do_ioctl
};

static int setup_netif(tiwlan_net_dev_t *drv)
{
    struct net_device *dev;
    int res;

    dev = alloc_etherdev(0);
    if (dev == NULL)
    {
        ti_dprintf(TIWLAN_LOG_ERROR, "alloc_etherdev() failed\n");
        return -ENOMEM;
    }
    ether_setup(dev);
    NETDEV_SET_PRIVATE(dev, drv);
    drv->netdev = dev;
    strcpy(dev->name, TIWLAN_DRV_IF_NAME);
    netif_carrier_off(dev);
    dev->netdev_ops = &tiwlan_drv_net_dev_ops;
    dev->tx_queue_len = 100;

    res = register_netdev(dev);
    if (res != 0)
    {
        ti_dprintf(TIWLAN_LOG_ERROR, "register_netdev() failed : %d\n", res);
        kfree(dev);
        return res;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
    SET_MODULE_OWNER(dev);
#endif
    return 0;
}


/* tiwlan_interrupt
   TIWLAN interrupt handler. Disables interrupts and awakes tasklet.
*/
#if !(defined(HW_ACCESS_SDIO)||defined(HW_ACCESS_WSPI))
static irqreturn_t tiwlan_interrupt (int irq, void *netdrv, struct pt_regs *cpu_regs)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)netdrv;

    /*
     * Workaround for the Linux 2.6 pending IRQ bug:
     * If a pending IRQ is handled on a WLAN ISR, the ISR is called again
     * even though it disabled itself in the first call. To protect against
     * re-entrance, this flag is checked, and if it is already set (meaning
     * that the ISR is called twice before the tasklet was called) nothing is done.
     */
    if (drv->interrupt_pending == 0)
    {
        UINT32 interruptVector;

        interruptVector = configMgr_checkInterrupts(drv->adapter.CoreHalCtx);
        if (interruptVector != 0)
        {
            configMgr_disableInterrupts(drv->adapter.CoreHalCtx);
            drv->interrupt_pending = 1;
            tasklet_schedule (&drv->tl);
        }
        else
        {
#if DEBUG_UNKNOWN_INTERRUPT
            ti_dprintf (TIWLAN_LOG_ERROR,
                        "%s - ERROR - interrupt isn't TNET interrupt! interrupt vector = 0x%08X\n",
                        __FUNCTION__, interruptVector);
#endif
        }
    }
    return IRQ_HANDLED;
}

#else

static irqreturn_t tiwlan_interrupt (int irq, void *netdrv, struct pt_regs *cpu_regs)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)netdrv;

    /* printk("TI: %s:\t%lu\n", __FUNCTION__, jiffies); */
    drv->interrupt_pending = 1;
#ifdef DM_USE_WORKQUEUE
#ifdef CONFIG_ANDROID_POWER
    android_lock_suspend( &drv->irq_wake_lock );
#endif
    queue_work( drv->tiwlan_wq, &drv->tirq );
    /* disable_irq( drv->irq ); Dm: No need, we can loose IRQ */
#else
    tasklet_schedule( &drv->tl );
#endif
    return IRQ_HANDLED;
}
#endif


static void tiwlan_poll_irq_handler(unsigned long parm)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)parm;
    bm_trace(2, 0, 0);

    tiwlan_interrupt(0, drv, NULL);
    mod_timer(&drv->poll_timer, jiffies + TIWLAN_IRQ_POLL_INTERVAL);
}

static void tiwlan_handle_control_requests( tiwlan_net_dev_t *drv )
{
    bm_trace(4, 0, 0);

    /* Handle control requests (timers, ioctls) */
    while(!list_empty(&drv->request_q))
    {
       struct list_head *entry = drv->request_q.next;
       tiwlan_req_t *req = list_entry(entry, tiwlan_req_t, list);
       tiwlan_req_t tmp_req;
       unsigned long flags;

       spin_lock_irqsave(&drv->lock, flags);
       list_del_init(entry);
       spin_unlock_irqrestore(&drv->lock, flags);

       ti_nodprintf(TIWLAN_LOG_INFO, "%s: f=0x%x req=0x%x reply_expected=%d\n",
                  __FUNCTION__, req->u.req.f, req, req->u.req.reply_expected);

       tmp_req.u.req.p1 = 0x1234;
       tmp_req.u.req.p2 = 0x4321;
       tmp_req.u.req.p3 = 0x1221;
       tmp_req.u.req.p4 = 0x4334;
       tmp_req.u.req.reply_expected = 0x50;

       req->u.reply = req->u.req.f(req);

       if ((tmp_req.u.req.p1 != 0x1234) || (tmp_req.u.req.p2 != 0x4321) || (tmp_req.u.req.p3 != 0x1221) || (tmp_req.u.req.p4 != 0x4334) || (tmp_req.u.req.reply_expected != 0x50))
       {
               printk("\n\n !!! ERROR: STACK CORRUPTION !!! : \nf=%p\n", tmp_req.u.req.f);
               if (!req->u.req.reply_expected)
                       printk("timer handler: %p\n", (void *)tmp_req.u.req.p1);
       }

       ti_nodprintf(TIWLAN_LOG_INFO, "%s: f=0x%x req=0x%x reply_expected=%d reply=%d\n",
                  __FUNCTION__, req->u.req.f, req, req->u.req.reply_expected, req->u.reply);
       if (req->u.req.reply_expected)
       {
          ti_nodprintf(TIWLAN_LOG_INFO, "%s: about to awake task\n", __FUNCTION__);
          complete(&req->u.req.comp);
       }
    }

    bm_trace(5, 0, 0);

    /* DbgCB_Insert(0, DBG_MODULE_OS, DBG_TYPE_TASKLET, 1)*/
}

#ifdef DM_USE_WORKQUEUE
static void tiwlan_irq_handler( struct work_struct *work )
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)container_of( work, struct tiwlan_net_dev, tirq );

    /* printk("TI: %s:\t%lu\n", __FUNCTION__, jiffies); */
#ifdef CONFIG_ANDROID_POWER
    android_lock_suspend( &drv->exec_wake_lock );
    android_unlock_suspend( &drv->irq_wake_lock );
#endif
    /* if the driver was unloaded by that time we need to ignore all the timers */
    if (drv->unload_driver) {
#ifdef CONFIG_ANDROID_POWER
        android_unlock_suspend( &drv->exec_wake_lock );
#endif
        /* enable_irq( drv->irq ); */
        return;
    }
    configMgr_handleInterrupts( drv->adapter.CoreHalCtx );
    tiwlan_handle_control_requests( drv );
#ifdef CONFIG_ANDROID_POWER
    if( drv->receive_packet ) {
        drv->receive_packet = 0;
        /* Keep awake for 500 ms to give a chance to network stack */
        android_lock_suspend_auto_expire( &drv->rx_wake_lock, (HZ >> 1) );
    }
    android_unlock_suspend( &drv->exec_wake_lock );
#endif
    /* enable_irq( drv->irq ); */
}
#endif

/* tiwlan_tasklet_handler
   WLAN protocol tasklet. Most of work happens in the
   context of this tasklet.
*/
#ifdef DM_USE_WORKQUEUE
static void tiwlan_work_handler( struct work_struct *work )
#else
static void tiwlan_tasklet_handler( unsigned long netdrv )
#endif
{
#ifdef DM_USE_WORKQUEUE
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)container_of( work, struct tiwlan_net_dev, tw );
#else
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)netdrv;
#endif
#ifdef STACK_PROFILE
    unsigned int curr1, base1;
    unsigned int curr2, base2;
    static unsigned int maximum_stack = 0;
#endif

    /* printk("TI: %s:\t%lu\n", __FUNCTION__, jiffies); */
#ifdef CONFIG_ANDROID_POWER
    android_lock_suspend( &drv->exec_wake_lock );
    android_unlock_suspend( &drv->timer_wake_lock );
#endif

    /* if the driver was unloaded by that time we need to ignore all the timers */
    if (drv->unload_driver) {
#ifdef CONFIG_ANDROID_POWER
        android_unlock_suspend( &drv->exec_wake_lock );
#endif
        return;
    }
#if 0
    ti_dprintf(TIWLAN_LOG_INFO, "%s in\n" , __FUNCTION__);
#endif

#ifdef DRIVER_PROFILE
    os_profile (drv, 0, 0);
#endif
    bm_trace(3, 0, 0);

#ifdef STACK_PROFILE
    curr1 = check_stack_start(&base1);
#endif

    /* Handle bus transaction interrupts */
    if (drv->dma_done)
    {
        drv->dma_done = 0;
        configMgr_HandleBusTxn_Complete(drv->adapter.CoreHalCtx);
    }

    /* don't call for "Handle interrupts, timers, ioctls" while recovery process */
    if (configMgr_areInputsFromOsDisabled(drv->adapter.CoreHalCtx) == TRUE) {
#ifdef CONFIG_ANDROID_POWER
        android_unlock_suspend( &drv->exec_wake_lock );
#endif
        return;
    }

    /* Handle firmware interrupts */
#ifndef DM_USE_WORKQUEUE
    if (drv->interrupt_pending)
    {
        drv->interrupt_pending = 0;
        configMgr_handleInterrupts(drv->adapter.CoreHalCtx);
    }
#endif

    tiwlan_handle_control_requests( drv );

#ifdef STACK_PROFILE
    curr2 = check_stack_stop(&base2);

    if (base2 == base1)
    {
       /* if the current measurement is bigger then the maximum store it and print*/
        if ((curr1 - curr2) > maximum_stack)
        {
            printk("STACK PROFILER GOT THE LOCAL MAXIMMUM!!!! \n");
            printk("current operation stack use =%d \n",(curr1 - curr2));
            printk("total stack use=%d \n",8192 - curr2 + base2);
            printk("total stack usage= %d percent \n",100 * (8192 - curr2 + base2) / 8192);
                maximum_stack = curr1 - curr2;
       }
    }
#endif

#ifdef DRIVER_PROFILE
    os_profile (drv, 1, 0);
#endif

#if 0
    ti_dprintf(TIWLAN_LOG_INFO, "%s out\n" , __FUNCTION__);
#endif
#ifdef CONFIG_ANDROID_POWER
    android_unlock_suspend( &drv->exec_wake_lock );
#endif
}

#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
static void tiwlan_rx_watchdog(struct work_struct *work)
{
    struct delayed_work *dwork = (struct delayed_work *) container_of(work, struct delayed_work, work);
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)container_of( dwork, struct tiwlan_net_dev, trxw );

    unsigned long num_rx_pkts = num_rx_pkt_new - num_rx_pkt_last;
    /* Contribute 10mA (200mA x 5%) for 1 pkt/sec, and plus 8mA base. */
    unsigned percent = (5 * num_rx_pkts  / RX_RATE_INTERVAL_SEC) + PWRSINK_WIFI_PERCENT_BASE;

    if (drv->unload_driver)
        return;

    percent = (percent > 100) ? 100 : percent;
    /* printk(KERN_INFO "num_rx_pkts=%ld, percent=%d\n", num_rx_pkts, percent); */
#ifdef CONFIG_HTC_PWRSINK
    htc_pwrsink_set(PWRSINK_WIFI, percent);
#else
    trout_pwrsink_set(PWRSINK_WIFI, percent);
#endif

    num_rx_pkt_last = num_rx_pkt_new;

    if (drv && drv->tiwlan_wq)
        queue_delayed_work(drv->tiwlan_wq, &drv->trxw, msecs_to_jiffies(MSEC_PER_SEC * RX_RATE_INTERVAL_SEC));
}
#endif

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
                           unsigned long p4)
{
    tiwlan_req_t req;
    unsigned long flags;

    /* Send request to tiwlan_tasklet and wait for reply */
    if (!drv->adapter.CoreHalCtx) {
        return STATION_IS_NOT_RUNNING;
    }

    req.drv = drv;
    req.u.req.f = f;
    req.u.req.p1 = p1;
    req.u.req.p2 = p2;
    req.u.req.p3 = p3;
    req.u.req.p4 = p4;
    req.u.req.reply_expected = 1;
    init_completion(&req.u.req.comp);

    spin_lock_irqsave(&drv->lock, flags);
    list_add_tail(&req.list, &drv->request_q);
    spin_unlock_irqrestore(&drv->lock, flags);

#ifdef DM_USE_WORKQUEUE
    /* printk("TI: %s:\t%lu\n", __FUNCTION__, jiffies); */
#ifdef CONFIG_ANDROID_POWER
    android_lock_suspend( &drv->timer_wake_lock );
#endif
    queue_work( drv->tiwlan_wq, &drv->tw );
#else
    tasklet_schedule( &drv->tl );
#endif
    wait_for_completion(&req.u.req.comp);

    return req.u.reply;
}


#define WLAN_PCMCIA_CFG_REG       0x0524
/* tiwlan_set_hw_access */
static int tiwlan_set_hw_access(tiwlan_net_dev_t *drv)
{
#ifdef TIWLAN_OMAP1610
    OS_WRITE_REG(drv, HI_CFG, 0x00000a00);

#if ! ((defined(HW_ACCESS_SDIO)||defined(HW_ACCESS_WSPI)) && defined(TNETW1150))
    OS_WRITE_REG(drv, WLAN_PCMCIA_CFG_REG, 0xC6880000);
    OS_WRITE_REG(drv, PCI_ARB_CFG, 0x2);
#endif

#endif
    return 0;
}


/* tiwlan_free_drv
   Unmap h/w regions and free driver's structure
*/
static void tiwlan_free_drv(tiwlan_net_dev_t *drv)
{
#ifdef TIWLAN_OMAP1610
    if (drv->acx_mem.pa && drv->acx_mem.va)
        iounmap(drv->acx_mem.va);
    if (drv->acx_reg.pa && drv->acx_reg.va && drv->acx_reg.va != drv->acx_reg.va)
        iounmap(drv->acx_reg.va);
#endif
    kfree(drv);
}


/* tiwlan_alloc_drv
   Allocate driver's structure and map h/w regions
*/
static tiwlan_net_dev_t *
tiwlan_alloc_drv(unsigned long reg_start, unsigned long reg_size,
                 unsigned long mem_start, unsigned long mem_size,
                 int map_io, int irq)
{
    static tiwlan_net_dev_t *drv;
    drv = kmalloc(sizeof(tiwlan_net_dev_t), GFP_KERNEL);
#ifdef TI_MEM_ALLOC_TRACE
    os_printf("MTT:%s:%d ::kmalloc(%lu, %x) : %lu\n", __FUNCTION__, __LINE__, sizeof(tiwlan_net_dev_t), GFP_KERNEL, sizeof(tiwlan_net_dev_t));
#endif/*I_MEM_ALLOC_TRACE*/

    if (!drv)
        return NULL;
    memset(drv, 0, sizeof(tiwlan_net_dev_t));
    drv->acx_mem.size = mem_size;
    drv->acx_reg.size = reg_size;
#ifdef TIWLAN_OMAP1610
    if (map_io)
    {
        drv->acx_mem.pa = mem_start;
        drv->acx_reg.pa = reg_start;
        drv->acx_mem.va = ioremap(drv->acx_mem.pa, drv->acx_mem.size);
        if (drv->acx_mem.pa!=drv->acx_reg.pa || drv->acx_mem.size!=drv->acx_reg.size)
            drv->acx_reg.va = ioremap(drv->acx_reg.pa, drv->acx_reg.size);
        else
            drv->acx_reg.va = drv->acx_mem.va;
    }
    else
    {
        /* Memory is already mapped */
        drv->acx_mem.va = (void *)mem_start;
        drv->acx_reg.va = (void *)reg_start;
    }
#endif /* Dm: */
    drv->irq = irq;
    return drv;
}


/* tiwlan_init_drv
   Called in process context
 */
int tiwlan_init_drv (tiwlan_net_dev_t *drv, tiwlan_dev_init_t *init_info)
{
    initTable_t *init_table;
    int rc;
    void *pWLAN_Images[4];

    /* printk("%s\n", __FUNCTION__); */
    /* It is OK if already initialized */
    if (drv->adapter.CoreHalCtx)
        return 0;

    init_table = os_memoryAlloc (drv, sizeof(initTable_t));

#ifdef TI_MEM_ALLOC_TRACE
    osPrintf ("MTT:%s:%d ::kmalloc(%lu, %x) : %lu\n", __FUNCTION__, __LINE__, sizeof(initTable_t), GFP_KERNEL, sizeof(initTable_t));
#endif/*I_MEM_ALLOC_TRACE*/
    if (!init_table)
    {
        ti_dprintf(TIWLAN_LOG_ERROR, "Cannot allocate init_table\n");
        return -ENOMEM;
    }

    if (init_info)
    {
        drv->eeprom_image.size = init_info->eeprom_image_length;
        if (drv->eeprom_image.size)
        {
            drv->eeprom_image.va = os_memoryAlloc (drv, drv->eeprom_image.size);

#ifdef TI_MEM_ALLOC_TRACE
            osPrintf ("MTT:%s:%d ::kmalloc(%lu, %x) : %lu\n", __FUNCTION__, __LINE__, drv->eeprom_image.size, GFP_KERNEL, drv->eeprom_image.size);
#endif
            if (!drv->eeprom_image.va)
            {
                ti_dprintf (TIWLAN_LOG_ERROR, "Cannot allocate buffer for eeprom image\n");
                drv->eeprom_image.size = 0;
                return -ENOMEM;
            }
            memcpy (drv->eeprom_image.va, &init_info->data[0], drv->eeprom_image.size );
        }

#ifdef FIRMWARE_DYNAMIC_LOAD
        drv->firmware_image.size = init_info->firmware_image_length;
        if (!drv->firmware_image.size)
        {
            ti_dprintf (TIWLAN_LOG_ERROR, "No firmware image\n");
            return -EINVAL;
        }
        drv->firmware_image.va = os_memoryAlloc (drv,drv->firmware_image.size);
#ifdef TI_MEM_ALLOC_TRACE
        osPrintf ("MTT:%s:%d ::kmalloc(%lu, %x) : %lu\n", __FUNCTION__, __LINE__, drv->firmware_image.size, GFP_KERNEL, drv->firmware_image.size);
#endif
        if (!drv->firmware_image.va)
        {
            ti_dprintf(TIWLAN_LOG_ERROR, "Cannot allocate buffer for firmware image\n");
            drv->firmware_image.size = 0;
            if (drv->eeprom_image.va)
                os_memoryFree (drv, drv->eeprom_image.va, drv->eeprom_image.size);
            return -ENOMEM;
        }
        memcpy (drv->firmware_image.va,
                &init_info->data[init_info->eeprom_image_length],
                drv->firmware_image.size);
#else
        extern unsigned char tiwlan_fwimage[];
        extern unsigned int sizeof_tiwlan_fwimage;

        drv->firmware_image.size = sizeof_tiwlan_fwimage;
        drv->firmware_image.va = tiwlan_fwimage;
#endif
    }

    print_deb ("--------- Eeeprom=%p(%lu), Firmware=%p(%lu)\n",
                drv->eeprom_image.va,
                drv->eeprom_image.size,
                drv->firmware_image.va,
                drv->firmware_image.size);

    /* Init defaults */
    if ((rc = osInitTable_IniFile (drv,
                                   init_table,
                                   (init_info && init_info->init_file_length) ?
                                   &init_info->data[init_info->eeprom_image_length+init_info->firmware_image_length] : NULL,
                                   init_info ? init_info->init_file_length : 0)))
    {
        ti_dprintf (TIWLAN_LOG_ERROR, "osInitTable_IniFile failed :cannot initialize defaults\n");
        os_memoryFree (drv, init_table, sizeof(initTable_t));

#ifdef TI_MEM_ALLOC_TRACE
        os_printf("MTT:%s:%d ::kfree(0x%p) : %d\n", __FUNCTION__, __LINE__, sizeof(initTable_t), -sizeof(initTable_t));
#endif
        return rc;
    }

    pWLAN_Images[0] = (void *)drv->firmware_image.va;
    pWLAN_Images[1] = (void *)drv->firmware_image.size;
    pWLAN_Images[2] = (void *)drv->eeprom_image.va;
    pWLAN_Images[3] = (void *)drv->eeprom_image.size;

    drv->adapter.CoreHalCtx = configMgr_create (drv,
                                                pWLAN_Images,
                                                init_table,
                                                (macAddress_t *) &drv->adapter.CurrentAddr);
    if (!(drv->adapter.CoreHalCtx))
    {
#ifdef FIRMWARE_DYNAMIC_LOAD
        os_memoryFree(drv,drv->firmware_image.va, drv->firmware_image.size);
        os_memoryFree (drv, drv->eeprom_image.va, drv->eeprom_image.size);
#endif
        os_memoryFree (drv, init_table, sizeof(initTable_t));
        ti_dprintf(TIWLAN_LOG_ERROR, "Cannot allocate CoreHalCtx\n");
        return -ENOMEM;
    }

    drv->interrupt_pending = 0;
    drv->dma_done = 0;

    if (drv->irq)
    {
#ifndef PRIODIC_INTERRUPT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
        unsigned long flags;
        /*
         * Disable all interrupts for not to catch the tiwlan irq
         * between request_irq and disable_irq
         */
        spin_lock_irqsave (&(drv->lock), flags);
        if ((rc = request_irq (drv->irq, tiwlan_interrupt, SA_SHIRQ, drv->netdev->name, drv)))
#else
        if ((rc = request_irq (drv->irq, (irq_handler_t)tiwlan_interrupt, IRQF_SHARED | IRQF_TRIGGER_FALLING /*Dm:*/, drv->netdev->name, drv)))
#endif
        {
            print_err ("TIWLAN: Failed to register interrupt handler\n");
            configMgr_stop (drv->adapter.CoreHalCtx);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
            spin_unlock_irqrestore (&drv->lock, flags);
#endif
            return rc;
        }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
        set_irq_type (drv->irq, IRQT_FALLING);
#else
        set_irq_type (drv->irq, IRQ_TYPE_EDGE_FALLING);
#endif
        disable_irq (drv->irq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
        spin_unlock_irqrestore (&drv->lock, flags);
#endif
#else
        printk (" tiwlan_init_drv :PRIODIC_INTERRUPT drv->irq %x\n",drv->irq);
#endif
    }
    else
    {
        /* Debug mode: polling */
        mod_timer (&drv->poll_timer, jiffies + TIWLAN_IRQ_POLL_INTERVAL);
    }

    /*
     * Now that all parts of the driver have been created and handles linked
     * proceed to download the FW code
     */
    configMgr_init (drv,
                    drv->adapter.CoreHalCtx,
                    pWLAN_Images,
                    init_table,
                    (macAddress_t *) &drv->adapter.CurrentAddr);

    /* Wait for the download to complete */
    os_WaitComplete ((void *)drv);

    os_memoryFree (drv, init_table, sizeof(initTable_t));

    if (rc == OK)
    {
        proc_stat_init (drv->adapter.CoreHalCtx);
#ifdef TI_MEM_ALLOC_TRACE
        osPrintf ("MTT:%s:%d ::kfree(0x%p) : %d\n", __FUNCTION__, __LINE__, sizeof(initTable_t), -sizeof(initTable_t));
#endif/*I_MEM_ALLOC_TRACE*/

       if (drv->adapter.CoreHalCtx == NULL)
       {
           ti_dprintf (TIWLAN_LOG_ERROR, "configMgr_create failed\n");
           return -ENODEV;
       }

       /* eeprom buffer is going to be deallocated by the caller. It is no longer needed anyway */
#if 0
        drv->eeprom_image.va = NULL;
        drv->eeprom_image.size = 0;
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
        drv->wl_sock = netlink_kernel_create(NETLINK_USERSOCK, 0, NULL, THIS_MODULE); /* Dm: */
#else
        drv->wl_sock = netlink_kernel_create(NETLINK_USERSOCK, 0, NULL, NULL, THIS_MODULE); /* Dm: */
#endif
#else
        drv->wl_sock = netlink_kernel_create(&init_net, NETLINK_USERSOCK, 0, NULL, NULL, THIS_MODULE); /* Dm: */
#endif
        if (drv->wl_sock == NULL)
        {
            ti_dprintf(TIWLAN_LOG_ERROR, "netlink_kernel_create() failed !\n");
            /* TODO: free in destroy */
            return -EINVAL;
        }

        /* Finalize network interface setup */
        memcpy (drv->netdev->dev_addr, drv->adapter.CurrentAddr, MAC_ADDR_LEN);
        drv->netdev->addr_len = MAC_ADDR_LEN;

        /* Register the relevant events with the event handler */
        tiwlan_register_events (drv);

        /* Mark that init stage has succeded */
        drv->initialized = 1;

        return 0;
    }

    return -ENODEV;
}

#ifdef CONFIG_ANDROID_POWER
#ifndef CONFIG_HAS_WAKELOCK
/* Wrapper for Init wake lock */
static void android_init_suspend_wakelock(android_suspend_lock_t *lp,char *nm)
{
    lp->name = nm;
    android_init_suspend_lock( lp );
}
#endif
#endif

/* tiwlan_start_drv
*/
int tiwlan_start_drv(tiwlan_net_dev_t *drv)
{
    /* printk("%s\n", __FUNCTION__); */
    if (!drv->initialized)
    {
        ti_dprintf(TIWLAN_LOG_ERROR, "Attempt to start driver before initilization has succeeded\n");
        return -ENODEV;
    }
    if (!drv->adapter.CoreHalCtx)
    {
        ti_dprintf(TIWLAN_LOG_ERROR, "Attempt to start driver before creating config_manager\n");
        return -ENODEV;
    }
    if (drv->started)
    {
        /*ti_dprintf(TIWLAN_LOG_ERROR, "Attempt to start driver that has already started\n");*/
        return -EALREADY;
    }
    if (configMgr_start(drv->adapter.CoreHalCtx) != OK)
    {
        print_err("TIWLAN: Failed to start config manager\n");
        return -EINVAL;
    }
    drv->started = 1;

#ifdef SDIO_INTERRUPT_HANDLING_ON
    configMgr_SlaveAckMaskNotification(drv->adapter.CoreHalCtx);
#endif
    if (drv->netdev)
        netif_start_queue(drv->netdev);
#ifdef CONFIG_TROUT_PWRSINK
    trout_pwrsink_set(PWRSINK_WIFI, PWRSINK_WIFI_PERCENT_BASE);
#endif
#ifdef CONFIG_HTC_PWRSINK
    htc_pwrsink_set(PWRSINK_WIFI, PWRSINK_WIFI_PERCENT_BASE);
#endif
    export_wifi_fw_version(drv);
    return 0;
}


/* tiwlan_destroy_drc
*/
static void tiwlan_destroy_drv(tiwlan_net_dev_t *drv)
{
    int waitShutdownCounter;

    /* close the ipc_kernel socket*/
    if (drv && drv->wl_sock) {
        sock_release(drv->wl_sock->sk_socket);
    }

    bm_destroy();

    if (drv->started)
        tiwlan_send_wait_reply(drv, tiwlan_stop_and_destroy_drv_request, 0, 0, 0, 0);
    else
        tiwlan_stop_and_destroy_drv(drv);

#ifdef DM_USE_WORKQUEUE
    while( tiwlan_del_msdu(drv) != NULL );
#endif
    if (drv->adapter.CoreHalCtx)
    {
        /* Delay return to OS until all driver components (HAL/SME) are shutdown */
        for (waitShutdownCounter=1; waitShutdownCounter<=DRV_SHUTDOWN_TEST_MAX_COUNTER; waitShutdownCounter++)
        {
            /* Check if HAL/SME are stopped - If so - exit loop and return to OS */
            if (configMgr_DriverShutdownStatus(drv->adapter.CoreHalCtx) == DRIVER_SHUTDOWN_COMPLETE)
            {
                break;
            }
            /* Delay of 100ms between shutdown test */
            mdelay ( DRV_SHUTDOWN_TEST_DELAY_INTERVAL );
        }

        /* If driver was not shutdown properly - destroy all timers "manually" and exit*/
        if ( waitShutdownCounter == DRV_SHUTDOWN_TEST_MAX_COUNTER+1 )
        {
            os_printf("Timeout while waiting for driver to shutdown...Shutdown status flag=0x%x\n",configMgr_DriverShutdownStatus(drv->adapter.CoreHalCtx));
        }
   
        /* drv->unload_driver = 1; Dm: moved to tiwlan_stop_and_destroy_drv */

        proc_stat_destroy();
        if (drv->irq) {
#ifdef CONFIG_ANDROID_POWER
            set_irq_wake(drv->irq, 0);
#endif
            free_irq(drv->irq, drv);
        }
        else
            del_timer_sync(&drv->poll_timer);

        /* Unload all modules (free memory) & destroy timers */
        configMgr_UnloadModules (drv->adapter.CoreHalCtx);

#ifdef FIRMWARE_DYNAMIC_LOAD
        if( drv->firmware_image.va ) {
            os_memoryFree(drv,drv->firmware_image.va, drv->firmware_image.size);
#ifdef TI_MEM_ALLOC_TRACE        
            os_printf("MTT:%s:%d ::kfree(0x%p) : %d\n", __FUNCTION__, __LINE__, drv->firmware_image.size, -drv->firmware_image.size);
#endif /*I_MEM_ALLOC_TRACE*/
        }
        if( drv->eeprom_image.va )
        { 
            os_memoryFree (drv, drv->eeprom_image.va, drv->eeprom_image.size);
#ifdef TI_MEM_ALLOC_TRACE        
            os_printf("MTT:%s:%d ::kfree(0x%p) : %d\n", __FUNCTION__, __LINE__, drv->eeprom_image.size, -drv->eeprom_image.size);
#endif /*I_MEM_ALLOC_TRACE*/
        }
#endif /*FIRMWARE_DYNAMIC_LOAD*/
    }
#ifdef DM_USE_WORKQUEUE
#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
    cancel_delayed_work_sync(&drv->trxw);
#endif    
    destroy_workqueue(drv->tiwlan_wq);
#endif
#ifdef CONFIG_TROUT_PWRSINK
    trout_pwrsink_set(PWRSINK_WIFI, 0);
#endif
#ifdef CONFIG_HTC_PWRSINK
    htc_pwrsink_set(PWRSINK_WIFI, 0);
#endif
#ifdef CONFIG_ANDROID_POWER
    android_uninit_suspend_lock(&drv->irq_wake_lock);
    android_uninit_suspend_lock(&drv->xmit_wake_lock);
    android_uninit_suspend_lock(&drv->timer_wake_lock);
    android_uninit_suspend_lock(&drv->rx_wake_lock);
    android_uninit_suspend_lock(&drv->exec_wake_lock);
#endif
    unregister_netdev(drv->netdev);
    tiwlan_free_drv(drv);
}


/* tiwlan_create_dev
   Create tiwlan device instance.
   Returns 0 if OK
*/
static int
tiwlan_create_drv(unsigned long reg_start, unsigned long reg_size,
                  unsigned long mem_start, unsigned long mem_size,
                  int map_io, int irq,
                  void *priv, tiwlan_net_dev_t **p_drv)
{
    tiwlan_net_dev_t *drv;
    int rc;

    /* printk("%s\n", __FUNCTION__); */
    /* Allocate device and map h/w regions */
    drv = tiwlan_alloc_drv(reg_start, reg_size, mem_start, mem_size, map_io, irq);
    if (!drv)
        return -ENOMEM;

    /* Check h/w access */
    if (tiwlan_set_hw_access(drv))
    {
        tiwlan_free_drv(drv);
        return -ENODEV;
    }

#ifdef DM_USE_WORKQUEUE
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    drv->tiwlan_wq = create_singlethread_workqueue("tiwlan_wifi_wq");
#else
    drv->tiwlan_wq = create_freezeable_workqueue("tiwlan_wifi_wq");
#endif
    if( !(drv->tiwlan_wq) ) {
        tiwlan_free_drv(drv);
        printk(KERN_ERR "Failed to create workqueue\n");
        return -EINVAL;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
    INIT_WORK( &drv->tw, tiwlan_work_handler, &drv->tw );
    INIT_WORK( &drv->txmit, tiwlan_xmit_handler, &drv->txmit );
    INIT_WORK( &drv->tirq, tiwlan_irq_handler, &drv->tirq );
#else
    INIT_WORK( &drv->tw, tiwlan_work_handler );
    INIT_WORK( &drv->txmit, tiwlan_xmit_handler );
    INIT_WORK( &drv->tirq, tiwlan_irq_handler );
#endif
    drv->txmit_msdu_next = drv->txmit_msdu_last = NULL;
#else
    tasklet_init( &drv->tl, tiwlan_tasklet_handler, (unsigned long)drv );
#endif

#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
    INIT_DELAYED_WORK( &drv->trxw, tiwlan_rx_watchdog, &drv->trxw );
#else
    INIT_DELAYED_WORK( &drv->trxw, tiwlan_rx_watchdog );
#endif
#endif

#ifdef CONFIG_ANDROID_POWER
    drv->receive_packet = 0;
    android_init_suspend_wakelock(&drv->irq_wake_lock,"tiwlan_irq_wake");
    android_init_suspend_wakelock(&drv->xmit_wake_lock,"tiwlan_xmit_wake");
    android_init_suspend_wakelock(&drv->timer_wake_lock,"tiwlan_timer_wake");
    android_init_suspend_wakelock(&drv->rx_wake_lock,"tiwlan_rx_wake");
    android_init_suspend_wakelock(&drv->exec_wake_lock,"tiwlan_exec_wake");
#endif
    spin_lock_init(&drv->lock);
    INIT_LIST_HEAD(&drv->request_q);
    init_timer(&drv->poll_timer);
    drv->poll_timer.function = tiwlan_poll_irq_handler;
    drv->poll_timer.data   = (unsigned long)drv;

    /* Init the completion obhect needed for init async purpose */
    init_completion(&drv->comp);

    /* Register network device */
    rc = setup_netif(drv);
    if (rc)
    {
        tiwlan_free_drv(drv);
        return rc;
    }
    drv->priv = priv;

    list_add(&drv->list, &tiwlan_drv_list);
    if (p_drv)
        *p_drv = drv;

    drv->initialized = 0;

    /* Profiler */
#ifdef DRIVER_PROFILING
    tiwlan_profile_create (drv);
#endif

    bm_init(drv);

#ifdef NO_USERMODE_WORKAROUND
    rc = tiwlan_init_drv(drv, NULL);
    rc = rc ? rc : tiwlan_start_drv(drv);
#endif

    return 0;
}

/* tiwlan_stop_driver
*/
int tiwlan_stop_drv(tiwlan_net_dev_t *drv)
{
    /* printk("%s\n", __FUNCTION__); */
    if (!drv->adapter.CoreHalCtx)
        return 0;

    if (drv->netdev)
        netif_stop_queue(drv->netdev);

    drv->started = 0;
    configMgr_stop(drv->adapter.CoreHalCtx);

#ifdef CONFIG_TROUT_PWRSINK
    trout_pwrsink_set(PWRSINK_WIFI, 0);
#endif
#ifdef CONFIG_HTC_PWRSINK
    htc_pwrsink_set(PWRSINK_WIFI, 0);
#endif
    return 0;
}

/* tiwlan_stop__and_destroy_driver
*/
int tiwlan_stop_and_destroy_drv(tiwlan_net_dev_t *drv)
{
    if (!drv->adapter.CoreHalCtx)
        return 0;

    if (drv->netdev)
        netif_stop_queue(drv->netdev);

    /* Start unload process by calling smeSm_stop, and halting the HAL */
    /* SmeSm_stop finish notification will be one by setting flags */
    configMgr_InitiateUnload(drv->adapter.CoreHalCtx);
    drv->started = 0;
    drv->unload_driver = 1;
    return 0;
}

/* tiwlan_stop__and_destroy_driver from workqueue
*/
int tiwlan_stop_and_destroy_drv_request(tiwlan_req_t *req)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)(req->drv);
    printk("%s: Called\n",__FUNCTION__);
    return tiwlan_stop_and_destroy_drv(drv);
}

void *wifi_kernel_prealloc(int section, unsigned long size)
{
#ifdef CONFIG_WIFI_CONTROL_FUNC
    if( wifi_control_data && wifi_control_data->mem_prealloc )
        return wifi_control_data->mem_prealloc( section, size );
    else
#endif
    return NULL;
}

#ifdef TIWLAN_CARDBUS

static struct pci_device_id tnetw1130_pci_tbl[] __devinitdata =
{
    { VENDOR_ID_TI, DEVICE_ID_TI_WLAN, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { 0, }
};

static int __devinit
tnetw1130_pci_init_one(struct pci_dev *pcidev, const struct pci_device_id *id)
{
    tiwlan_net_dev_t *drv;
    int rc;

    print_info("tnetw1130_pci_init_one:\n");
    /* IT: for some reason interrupt doesn't work.
       use poling mode for now (comments around
       pcidev->irq below)
    */
    rc = tiwlan_create_drv(pcidev->resource[0].start,
                           pcidev->resource[0].end - pcidev->resource[0].start,
                           pcidev->resource[1].start,
                           pcidev->resource[1].end - pcidev->resource[1].start,
                           1,
                           0/*pcidev->irq*/, pcidev, &drv);
    if (!rc)
        pcidev->driver_data = drv;
    return rc;
}

void tnetw1130_pci_remove(struct pci_dev *dev)
{
    tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)dev->driver_data;
    if (drv)
    {
        dev->driver_data = NULL;
        tiwlan_destroy_drv(drv);
    }
}

static struct pci_driver tnetw1130_pci_driver = {
    .name =      "tnetw1130",
    .id_table =  tnetw1130_pci_tbl,
    .probe = tnetw1130_pci_init_one,
    .remove =    tnetw1130_pci_remove
};

#endif /* #ifdef TIWLAN_CARDBUS */

#ifdef TIWLAN_OMAP1610
int omap1610_drv_create(void)
{
    omap_memif_init();
    return tiwlan_create_drv(TIWLAN_OMAP1610_REGBASE, TIWLAN_OMAP1610_REGSIZE,
                             TIWLAN_OMAP1610_MEMBASE, TIWLAN_OMAP1610_MEMSIZE,
                             0, TIWLAN_OMAP1610_IRQ, NULL, NULL);
}
#endif /* #ifdef TIWLAN_OMAP1610 */

#ifdef TIWLAN_MSM7000

#define TROUT_IRQ MSM_GPIO_TO_INT(29)

static void tiwlan_sdio_irq(struct sdio_func *func)
{
    printk("%s:\n", __FUNCTION__);
}

static const struct sdio_device_id tiwlan_sdio_ids[] = {
    { SDIO_DEVICE_CLASS(SDIO_CLASS_WLAN)    },
    {                                       },
};

MODULE_DEVICE_TABLE(sdio, tiwlan_sdio_ids);

int tiwlan_sdio_init(struct sdio_func *func)
{
    int rc;

    rc = sdio_enable_func(func);
    if (rc)
        return rc;

    rc = sdio_set_block_size(func, 512);
    if( rc ) {
        sdio_disable_func(func);
    }
    return rc;
}

static int tiwlan_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
    int rc;

    SDIO_SetFunc( NULL );
    if (func->vendor != VENDOR_ID_TI || func->device != DEVICE_ID_TI_WLAN)
        return -ENODEV;

    printk(KERN_INFO
           "TIWLAN: Found SDIO controller (vendor 0x%x, device 0x%x)\n",
           func->vendor, func->device);

#ifdef CONFIG_TROUT_PWRSINK
    trout_pwrsink_set(PWRSINK_WIFI, PWRSINK_WIFI_PERCENT_BASE);
#endif
#ifdef CONFIG_HTC_PWRSINK
    htc_pwrsink_set(PWRSINK_WIFI, PWRSINK_WIFI_PERCENT_BASE);
#endif

    sdio_claim_host(func);

    rc = tiwlan_sdio_init(func);
    if (rc)
        goto err2;

    rc = sdio_claim_irq(func, tiwlan_sdio_irq);
    if (rc)
        goto err1;

    SDIO_SetFunc( func );

    rc = tiwlan_create_drv(0, 0, 0, 0, 0, TROUT_IRQ, NULL, NULL);

    printk(KERN_INFO "TIWLAN: Driver initialized (rc %d)\n", rc);
    complete(&sdio_wait);
    return rc;
err1:
    sdio_disable_func(func);
err2:
    sdio_release_host(func);
    complete(&sdio_wait);
    printk(KERN_ERR "TIWLAN: SDIO failure (err %d)\n", rc);
    return rc;
}

static void tiwlan_sdio_remove(struct sdio_func *func)
{
    printk(KERN_DEBUG "TIWLAN: Releasing SDIO resources\n");
    sdio_release_irq(func);
    sdio_disable_func(func);
    sdio_release_host(func);
    printk(KERN_DEBUG "TIWLAN: SDIO resources released\n");
}

static struct sdio_driver tiwlan_sdio_drv = {
    .probe          = tiwlan_sdio_probe,
    .remove         = tiwlan_sdio_remove,
    .name           = "sdio_tiwlan",
    .id_table       = tiwlan_sdio_ids,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static int wifi_probe( struct platform_device *pdev )
{
    struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

    printk("%s\n", __FUNCTION__);
    if( wifi_ctrl ) {
        wifi_control_data = wifi_ctrl;
        if( wifi_ctrl->set_power )
            wifi_ctrl->set_power(1);		/* Power On */
        if( wifi_ctrl->set_reset )
            wifi_ctrl->set_reset(0);		/* Reset clear */
        if( wifi_ctrl->set_carddetect )
            wifi_ctrl->set_carddetect(1);	/* CardDetect (0->1) */
    }
    return 0;
}

static int wifi_remove( struct platform_device *pdev )
{
    struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

    printk("%s\n", __FUNCTION__);
    if( wifi_ctrl ) {
        if( wifi_ctrl->set_carddetect )
            wifi_ctrl->set_carddetect(0);	/* CardDetect (1->0) */
        if( wifi_ctrl->set_reset )
            wifi_ctrl->set_reset(1);		/* Reset active */
        if( wifi_ctrl->set_power )
            wifi_ctrl->set_power(0);		/* Power Off */
    }
    return 0;
}

static struct platform_driver wifi_device = {
    .probe          = wifi_probe,
    .remove         = wifi_remove,
    .suspend        = NULL,
    .resume         = NULL,
    .driver         = {
        .name   = "msm_wifi",
    },
};

static int wifi_add_dev( void )
{
    return platform_driver_register( &wifi_device );
}

static void wifi_del_dev( void )
{
    platform_driver_unregister( &wifi_device );
}

int msm_wifi_power( int on )
{
    printk("%s\n", __FUNCTION__);
    if( wifi_control_data && wifi_control_data->set_power ) {
        wifi_control_data->set_power(on);
    }
    return 0;
}

int msm_wifi_reset( int on )
{
    printk("%s\n", __FUNCTION__);
    if( wifi_control_data && wifi_control_data->set_reset ) {
        wifi_control_data->set_reset(on);
    }
    return 0;
}
#endif
#endif /* TIWLAN_MSM7000 */

static int __init tiwlan_module_init(void)
{
    int rc = 0;

    printk(KERN_INFO "TIWLAN: Driver loading\n");
   /* Check sizes of basic structures to ensure that compilation
      options are OK
   */
    if (packed_struct_tst())
        ;/*IT: return -EINVAL; */

    tiwlan_deb_entry = create_proc_entry(TIWLAN_DBG_PROC, 0644, NULL);
    if (tiwlan_deb_entry == NULL)
        return -EINVAL;
    tiwlan_deb_entry->read_proc = tiwlan_deb_read_proc;
    tiwlan_deb_entry->write_proc = tiwlan_deb_write_proc;
#ifdef TIWLAN_MSM7000
    init_completion(&sdio_wait);
#endif
#ifdef TIWLAN_CARDBUS
    if ((rc=pci_register_driver(&tnetw1130_pci_driver)) <  0)
        print_err("TIWLAN: PCMCIA driver failed to register\n");
        remove_proc_entry(TIWLAN_DBG_PROC, NULL);
        return rc;
    }
    printk(KERN_INFO "TIWLAN: Driver loaded\n");
    return 0;

#elif defined(TIWLAN_OMAP1610)
    rc = omap1610_drv_create();
    export_wifi_chip_id();
    printk(KERN_INFO "TIWLAN: Driver loaded\n");
    return rc;

#elif defined(TIWLAN_MSM7000)
#ifdef CONFIG_WIFI_CONTROL_FUNC
    wifi_add_dev();
#else
    trout_wifi_power(1);          /* Power On */
    trout_wifi_reset(0);          /* Reset clear */
    trout_wifi_set_carddetect(1); /* CardDetect (0->1) */
#endif

    /* Register ourselves as an SDIO driver */
    rc = sdio_register_driver(&tiwlan_sdio_drv);
    if (rc < 0) {
        printk(KERN_ERR "sdio register failed (%d)\n", rc);
        remove_proc_entry(TIWLAN_DBG_PROC, NULL);
        return rc;
    }
    /* rc = tiwlan_create_drv(0, 0, 0, 0, 0, TROUT_IRQ, NULL, NULL); -- Called in probe */

    tiwlan_calibration = create_proc_entry("calibration", 0644, NULL);
    if (tiwlan_calibration == NULL) {
        remove_proc_entry(TIWLAN_DBG_PROC, NULL);
        return -EINVAL;
    }
    tiwlan_calibration->size = tiwlan_get_nvs_size();
    tiwlan_calibration->read_proc = tiwlan_calibration_read_proc;
    tiwlan_calibration->write_proc = tiwlan_calibration_write_proc;

    if (!wait_for_completion_timeout(&sdio_wait, msecs_to_jiffies(10000))) {
        printk(KERN_ERR "%s: Timed out waiting for device detect\n", __func__);
        remove_proc_entry(TIWLAN_DBG_PROC, NULL);
        remove_proc_entry("calibration", NULL);
        sdio_unregister_driver(&tiwlan_sdio_drv);
#ifdef CONFIG_WIFI_CONTROL_FUNC
        wifi_del_dev();
#else
        trout_wifi_set_carddetect(0); /* CardDetect (1->0) */
        trout_wifi_reset(1);          /* Reset active */
        trout_wifi_power(0);          /* Power Off */
#endif
        return -ENODEV;
    }
    export_wifi_chip_id();
    printk(KERN_INFO "TIWLAN: Driver loaded\n");
    return 0;

#else

#error Either TIWLAN_CARDBUS, TIWLAN_OMAP1610 or TIWLAN_MSM7000 must be defined

#endif
}

static void __exit tiwlan_module_cleanup(void)
{
    struct list_head *l;
    struct list_head *tmp;

    printk(KERN_INFO "TIWLAN: Driver unloading\n");
#ifdef TIWLAN_CARDBUS
    pci_unregister_driver(&tnetw1130_pci_driver);
#endif
    list_for_each_safe(l, tmp, &tiwlan_drv_list)
    {
        tiwlan_net_dev_t *drv = (tiwlan_net_dev_t *)list_entry(l, tiwlan_net_dev_t, list);
        list_del(l);
        tiwlan_destroy_drv(drv);
    }
    remove_proc_entry(TIWLAN_DBG_PROC, NULL);
#ifdef TIWLAN_MSM7000
    remove_proc_entry("calibration", NULL);
    sdio_unregister_driver(&tiwlan_sdio_drv);
#ifdef CONFIG_WIFI_CONTROL_FUNC
    wifi_del_dev();
#else
    trout_wifi_set_carddetect(0); /* CardDetect (1->0) */
    trout_wifi_reset(1);          /* Reset active */
    trout_wifi_power(0);          /* Power Off */
#endif
#endif
    printk(KERN_INFO "TIWLAN: Driver unloaded\n");
}

module_init(tiwlan_module_init);
module_exit(tiwlan_module_cleanup);

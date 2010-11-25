/*
 * Copyright (c) 2007 - 2008 Motorola, Inc, All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 * Changelog:
 * Date               Author           Comment
 * -----------------------------------------------------------------------------
 * 12/07/2007      Motorola        USB-IPC initial
 * 03/22/2008      Motorola        USB-IPC header support
 * 10/09/2008      Motorola        USB-IPC suspend/resume support
 * 
 */
 
/*!
 * @file drivers/usb/ipchost/ipc_data.c
 * @brief USB-IPC Descriptor Set
 *
 * This is the generic portion of the USB-IPC driver.
 *
 * @ingroup IPCFunction
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/usb_ipc.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#ifdef USE_OMAP_SDMA
#include <mach/dma.h>
#endif

/* For debug only */
#include <linux/io.h>
#include <mach/io.h>

/* Module */
MODULE_DESCRIPTION("OMAP SAM IPC Test Module");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

#define DEBUG(args...) //printk(args)

//#define USB_DATA_LOG

static struct usb_device_id usb_ipc_id_table [] = {
	{ USB_DEVICE(MOTO_USBIPC_VID, MOTO_USBIPC_PID) },
	{ }						/* Terminating entry */
};

// USB endpoint detection
#define IS_EP_BULK(ep)     (((ep)->bmAttributes) == USB_ENDPOINT_XFER_BULK ? 1 : 0)
#define IS_EP_BULK_IN(ep)  (IS_EP_BULK(ep) && (((ep)->bEndpointAddress) & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
#define IS_EP_BULK_OUT(ep) (IS_EP_BULK(ep) && (((ep)->bEndpointAddress) & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)

/* Wakes up kipcd */
static struct task_struct *kipcd_task;
DEFINE_SPINLOCK(ipc_event_lock);
DECLARE_WAIT_QUEUE_HEAD(kipcd_wait);

/*  */
USB_IPC_IFS_STRUCT usb_ipc_data_param;

#ifdef CONFIG_PM
extern USB_LOG_IFS_STRUCT ipc_log_param;
#endif // CONFIG_PM

/******************************************************/

//#define USB_STACK_SEND_ZERO_PACKET

#ifndef USB_STACK_SEND_ZERO_PACKET
static int ipc_data_urb_actual_len = 0;
#endif

int ipc_dbg_index;
char ipc_dbg_array[IPC_DBG_ARRAY_SIZE];
unsigned int dma_vsrc;
unsigned int dma_psrc;
unsigned int dma_vdest;
unsigned int dma_pdest;
unsigned int dma_size;
unsigned int dma_ch;

#ifdef CONFIG_PM
static void ipc_suspend_work(struct work_struct *work)
{
  unsigned long flags;
  USB_IPC_IFS_STRUCT *usb_ifs = container_of(work, USB_IPC_IFS_STRUCT, suspend_work.work);
  if(usb_ifs->sleeping == 0) {
    spin_lock_irqsave(&ipc_event_lock, flags);
    usb_ipc_data_param.ipc_events |= IPC_PM_SUSPEND;
    spin_unlock_irqrestore(&ipc_event_lock, flags);
    wake_up(&kipcd_wait);
  }
}

static void ipc_resume_work(struct work_struct *work)
{
  unsigned long flags;
  USB_IPC_IFS_STRUCT *usb_ifs = container_of(work, USB_IPC_IFS_STRUCT, wakeup_work);
  if(usb_ifs->sleeping == 1) {
    spin_lock_irqsave(&ipc_event_lock, flags);
    usb_ipc_data_param.ipc_events |= IPC_PM_RESUME;
    spin_unlock_irqrestore(&ipc_event_lock, flags);
    wake_up(&kipcd_wait);
  }
}
#endif

/*
 * write buffer
 */
static int ipc_data_write_buffer(unsigned char *buff, int size)
{
  int ret = 0;
  unsigned long flags;

#ifndef USB_STACK_SEND_ZERO_PACKET
  ipc_data_urb_actual_len = 0;
#endif

#ifdef USB_DATA_LOG
  printk("Enter %s:   size = %d\n", __FUNCTION__, size);
  for(ret = 0; ret < size; ret ++) {
    printk("0x%x, ", buff[ret]);
    if( ((ret + 1) % 10) == 0) {
      printk ("\n");
    }
  }
  ret = 0;
#endif

#ifdef USB_STACK_SEND_ZERO_PACKET
  usb_ipc_data_param.write_urb.transfer_flags |= URB_ZERO_PACKET;
#endif
  usb_ipc_data_param.write_urb.transfer_buffer = buff;
  usb_ipc_data_param.write_urb.transfer_buffer_length = size;
  usb_ipc_data_param.write_urb.dev = usb_ipc_data_param.udev;

  spin_lock_irqsave(&ipc_event_lock, flags);
  usb_ipc_data_param.ipc_events |= IPC_DATA_WR;
  spin_unlock_irqrestore(&ipc_event_lock, flags);
  wake_up(&kipcd_wait);

  return ret; 
}

/*
 * read buffer
 */
static int ipc_data_read_buffer(unsigned char *buff, int size)
{
  int ret = 0;
  unsigned long flags;

  usb_ipc_data_param.read_urb.transfer_buffer = buff;
  usb_ipc_data_param.read_urb.transfer_buffer_length = size;
  usb_ipc_data_param.read_urb.dev = usb_ipc_data_param.udev;

  spin_lock_irqsave(&ipc_event_lock, flags);
  usb_ipc_data_param.ipc_events |= IPC_DATA_RD;
  spin_unlock_irqrestore(&ipc_event_lock, flags);
  wake_up(&kipcd_wait);
	ipc_dbg_array[ipc_dbg_index++] = 0x1;
	if (ipc_dbg_index >= IPC_DBG_ARRAY_SIZE)
		ipc_dbg_index = 0;

  return ret;
}

/*
 *  BULK IN callback
 */
static void ipc_data_read_callback(struct urb *urb)
{
  unsigned long flags;
  DEBUG("\n%s: received %d bytes @ jiffies = %lu\n", __FUNCTION__, urb->actual_length, jiffies);

#ifdef USB_DATA_LOG
  int ret;
  char * buff;
  buff = (char *)urb->transfer_buffer;
  for(ret = 0; ret < urb->actual_length; ret ++) {
    printk("0x%x, ", buff[ret]); 
    if(((ret + 1) % 10) == 0) {
      printk ("\n");
    }
  }
#endif

  spin_lock_irqsave(&ipc_event_lock, flags);
  usb_ipc_data_param.ipc_events |= IPC_DATA_RD_CB;
  spin_unlock_irqrestore(&ipc_event_lock, flags);
  wake_up(&kipcd_wait);
	ipc_dbg_array[ipc_dbg_index++] = 0x11;
	if (ipc_dbg_index >= IPC_DBG_ARRAY_SIZE)
		ipc_dbg_index = 0;
}

/*
 *  BULK OUT callback
 */
static void ipc_data_write_callback(struct urb *urb)
{
  unsigned long flags;
  DEBUG("\n%s: transmitted %d bytes @ jiffies = %lu\n", __FUNCTION__, urb->actual_length, jiffies);

  /* if the last transmit is not zero, but it is multiple of MaxPacketSize, send zero package */
#ifndef USB_STACK_SEND_ZERO_PACKET
  ipc_data_urb_actual_len += urb->actual_length;

  if((usb_ipc_data_param.write_urb.transfer_buffer_length != 0) &&
     ((usb_ipc_data_param.write_urb.transfer_buffer_length % usb_ipc_data_param.write_wMaxPacketSize) ==0)) {
    usb_ipc_data_param.write_urb.transfer_buffer_length = 0;
    usb_ipc_data_param.write_urb.dev = usb_ipc_data_param.udev;
    usb_submit_urb(&usb_ipc_data_param.write_urb, GFP_ATOMIC|GFP_DMA);
  }
  else {
    spin_lock_irqsave(&ipc_event_lock, flags);
    usb_ipc_data_param.ipc_events |= IPC_DATA_WR_CB;
    spin_unlock_irqrestore(&ipc_event_lock, flags);
    wake_up(&kipcd_wait);
  }
#else /* USB_STACK_SEND_ZERO_PACKET */
  spin_lock_irqsave(&ipc_event_lock, flags);
  usb_ipc_data_param.ipc_events |= IPC_DATA_WR_CB;
  spin_unlock_irqrestore(&ipc_event_lock, flags);
  wake_up(&kipcd_wait);
#endif /* USB_STACK_SEND_ZERO_PACKET */
}

static void ipc_events(void)
{
  int ret = 0;
  int pending_events;
  unsigned long flags;

  spin_lock_irqsave(&ipc_event_lock, flags);
  pending_events = usb_ipc_data_param.ipc_events;
  usb_ipc_data_param.ipc_events = 0;
  spin_unlock_irqrestore(&ipc_event_lock, flags);

  while((pending_events != 0) && (ret == 0)) {
    /* process ipc_data_write_buffer */
    if(pending_events & IPC_DATA_WR) {
      pending_events &= ~IPC_DATA_WR;
#ifdef CONFIG_PM
      cancel_delayed_work_sync(&usb_ipc_data_param.suspend_work);
      spin_lock_bh(&usb_ipc_data_param.pm_lock);
      if(usb_ipc_data_param.sleeping == 0) {
        usb_ipc_data_param.working = 1;
        ret = usb_submit_urb(&usb_ipc_data_param.write_urb, GFP_ATOMIC|GFP_DMA);
        spin_unlock_bh(&usb_ipc_data_param.pm_lock);
        pending_events &= ~IPC_PM_SUSPEND;
        spin_lock_irqsave(&ipc_event_lock, flags);
        usb_ipc_data_param.ipc_events &= ~IPC_PM_SUSPEND;
        spin_unlock_irqrestore(&ipc_event_lock, flags);
      }
      else {
        usb_ipc_data_param.write_urb_used = 1;
        spin_unlock_bh(&usb_ipc_data_param.pm_lock);
        usb_autopm_get_interface(usb_ifnum_to_if(usb_ipc_data_param.udev, IPC_DATA_CH_NUM));
      }
#else
      ret = usb_submit_urb(&usb_ipc_data_param.write_urb, GFP_ATOMIC|GFP_DMA);
#endif
    }
    /* process ipc_data_read_buffer */
    if(pending_events & IPC_DATA_RD) {
      pending_events &= ~IPC_DATA_RD;
#ifdef CONFIG_PM
      spin_lock_bh(&usb_ipc_data_param.pm_lock);
      if(usb_ipc_data_param.sleeping == 0) {
        ret = usb_submit_urb(&usb_ipc_data_param.read_urb, GFP_ATOMIC|GFP_DMA);
        spin_unlock_bh(&usb_ipc_data_param.pm_lock);
	ipc_dbg_array[ipc_dbg_index++] = 0x2;
	if (ipc_dbg_index >= IPC_DBG_ARRAY_SIZE)
		ipc_dbg_index = 0;
      }
      else {
        usb_ipc_data_param.read_urb_used = 1;
        spin_unlock_bh(&usb_ipc_data_param.pm_lock);
	ipc_dbg_array[ipc_dbg_index++] = 0x3;
	if (ipc_dbg_index >= IPC_DBG_ARRAY_SIZE)
		ipc_dbg_index = 0;
      }
#else
      ret = usb_submit_urb(&usb_ipc_data_param.read_urb, GFP_ATOMIC|GFP_DMA);
#endif
    }
    /* process ipc_data_write_callback */
    if(pending_events & IPC_DATA_WR_CB) {
      pending_events &= ~IPC_DATA_WR_CB;
#ifdef CONFIG_PM
      spin_lock_bh(&usb_ipc_data_param.pm_lock);
      usb_ipc_data_param.working = 0;
      if(usb_ipc_data_param.sleeping == 0) {
        spin_unlock_bh(&usb_ipc_data_param.pm_lock);
        pending_events &= ~IPC_PM_SUSPEND;
        spin_lock_irqsave(&ipc_event_lock, flags);
        usb_ipc_data_param.ipc_events &= ~IPC_PM_SUSPEND;
        spin_unlock_irqrestore(&ipc_event_lock, flags);
        cancel_delayed_work_sync(&usb_ipc_data_param.suspend_work);
        queue_delayed_work(usb_ipc_data_param.ksuspend_usb_wq, &usb_ipc_data_param.suspend_work,
                           msecs_to_jiffies(USB_IPC_SUSPEND_DELAY));
      }
      else spin_unlock_bh(&usb_ipc_data_param.pm_lock);
#endif /* CONFIG_PM */
#ifndef USB_STACK_SEND_ZERO_PACKET
      if(usb_ipc_data_param.ipc_write_cb != NULL) {
        usb_ipc_data_param.ipc_write_cb(IPC_DATA_CH_NUM, 0, ipc_data_urb_actual_len);
      }
#else /* USB_STACK_SEND_ZERO_PACKET */
      if(usb_ipc_data_param.ipc_write_cb != NULL) {
        usb_ipc_data_param.ipc_write_cb(IPC_DATA_CH_NUM, 0, usb_ipc_data_param.write_urb.actual_length);
      }
#endif /* USB_STACK_SEND_ZERO_PACKET */
    }
    /* process ipc_data_read_callback */
    if(pending_events & IPC_DATA_RD_CB) {
      pending_events &= ~IPC_DATA_RD_CB;
#ifdef CONFIG_PM
      spin_lock_bh(&usb_ipc_data_param.pm_lock);
      if(usb_ipc_data_param.sleeping == 0) {
        spin_unlock_bh(&usb_ipc_data_param.pm_lock);
        pending_events &= ~IPC_PM_SUSPEND;
        spin_lock_irqsave(&ipc_event_lock, flags);
        usb_ipc_data_param.ipc_events &= ~IPC_PM_SUSPEND;
        spin_unlock_irqrestore(&ipc_event_lock, flags);
        cancel_delayed_work_sync(&usb_ipc_data_param.suspend_work);
        queue_delayed_work(usb_ipc_data_param.ksuspend_usb_wq, &usb_ipc_data_param.suspend_work,
                           msecs_to_jiffies(USB_IPC_SUSPEND_DELAY));
      }
      else spin_unlock_bh(&usb_ipc_data_param.pm_lock);
#endif
	if ((usb_ipc_data_param.read_urb.status < 0) &&
	    (usb_ipc_data_param.read_urb.actual_length != 0)) {
		printk(KERN_INFO "incomplete IN transfer status%d length%d\n",
			usb_ipc_data_param.read_urb.status,
			usb_ipc_data_param.read_urb.actual_length);
		if (usb_ipc_data_param.truncated_buf == NULL)
			usb_ipc_data_param.truncated_buf = (char *)
				usb_ipc_data_param.read_urb.transfer_buffer;
		usb_ipc_data_param.truncated_size +=
			usb_ipc_data_param.read_urb.actual_length;
		usb_ipc_data_param.read_urb.transfer_buffer =
			usb_ipc_data_param.truncated_buf
				+ usb_ipc_data_param.truncated_size;
		usb_ipc_data_param.read_urb.transfer_buffer_length -=
			usb_ipc_data_param.read_urb.actual_length;
		spin_lock_bh(&usb_ipc_data_param.pm_lock);
		if (usb_ipc_data_param.sleeping == 1) {
			spin_unlock_bh(&usb_ipc_data_param.pm_lock);
			usb_ipc_data_param.read_urb_used = 1;
		} else {
			spin_unlock_bh(&usb_ipc_data_param.pm_lock);
			ret = usb_submit_urb(&usb_ipc_data_param.read_urb,
					     GFP_ATOMIC|GFP_DMA);
		}
	} else {
		if ((usb_ipc_data_param.truncated_buf != NULL) &&
		    (usb_ipc_data_param.truncated_size != 0)) {
			printk(KERN_INFO "trying to re-assemble \
				incomplete IN transfer\n");
			usb_ipc_data_param.read_urb.transfer_buffer =
				usb_ipc_data_param.truncated_buf;
			usb_ipc_data_param.read_urb.actual_length +=
				usb_ipc_data_param.truncated_size;
			usb_ipc_data_param.truncated_buf = NULL;
			usb_ipc_data_param.truncated_size = 0;
		}
		if (usb_ipc_data_param.ipc_read_cb != NULL)
			usb_ipc_data_param.ipc_read_cb(IPC_DATA_CH_NUM,
				0, usb_ipc_data_param.read_urb.actual_length);
	}
    }
    /* process ipc_log_read_callback */
    if(pending_events & IPC_LOG_RD_CB) {
      pending_events &= ~IPC_LOG_RD_CB;
#ifdef CONFIG_PM
      spin_lock_bh(&usb_ipc_data_param.pm_lock);
      if(usb_ipc_data_param.sleeping == 0) {
        spin_unlock_bh(&usb_ipc_data_param.pm_lock);
        pending_events &= ~IPC_PM_SUSPEND;
        spin_lock_irqsave(&ipc_event_lock, flags);
        usb_ipc_data_param.ipc_events &= ~IPC_PM_SUSPEND;
        spin_unlock_irqrestore(&ipc_event_lock, flags);
        cancel_delayed_work_sync(&usb_ipc_data_param.suspend_work);
        queue_delayed_work(usb_ipc_data_param.ksuspend_usb_wq, &usb_ipc_data_param.suspend_work,
                           msecs_to_jiffies(USB_IPC_SUSPEND_DELAY));
      }
      else spin_unlock_bh(&usb_ipc_data_param.pm_lock);
#endif
    }
#ifdef CONFIG_PM
    if(pending_events & IPC_PM_SUSPEND) {
      pending_events &= ~IPC_PM_SUSPEND;
      DEBUG("%s @ jiffies=%lu\n", __FUNCTION__, jiffies);
	usb_ipc_data_param.allow_suspend = 1;
      usb_autopm_put_interface(usb_ifnum_to_if(usb_ipc_data_param.udev, IPC_DATA_CH_NUM));
    }
    if(pending_events & IPC_PM_RESUME) {
      pending_events &= ~IPC_PM_RESUME;
      DEBUG("%s @ jiffies=%lu\n", __FUNCTION__, jiffies);
      usb_autopm_get_interface(usb_ifnum_to_if(usb_ipc_data_param.udev, IPC_DATA_CH_NUM));
    }
#endif

    spin_lock_irqsave(&ipc_event_lock, flags);
    pending_events = usb_ipc_data_param.ipc_events;
    usb_ipc_data_param.ipc_events = 0;
    spin_unlock_irqrestore(&ipc_event_lock, flags);
  }
}

static int ipc_thread(void *__unused)
{
  set_freezable();
  do {
    ipc_events();
    wait_event_freezable(kipcd_wait,
                         (usb_ipc_data_param.ipc_events != 0) ||
                         kthread_should_stop());
  } while (!kthread_should_stop() || (usb_ipc_data_param.ipc_events != 0));

  return 0;
}

/*
 * usb ipc data driver probe function 
 */
int usb_ipc_data_probe(struct usb_interface *intf, const struct usb_device_id *id)
{	
  struct usb_endpoint_descriptor  *ipc_endpoint;		
  struct usb_device *dev = interface_to_usbdev (intf);

  usb_ipc_data_param.udev = dev;

#ifdef USE_OMAP_SDMA
	if (0 != omap_request_dma(IPC_DMA_NODE2BUF_ID, NULL,
				  ipc_dma_node2buf_callback, NULL,
				  &ipc_memcpy_node2buf.dma_ch)) {
		printk(KERN_ERR \
			"%s: Failed to allocate DMA channel for IPC write\n",
			__func__);
		ipc_memcpy_node2buf.dma_ch = -1;
	}
	if (0 != omap_request_dma(IPC_DMA_BUF2NODE_ID, NULL,
				  ipc_dma_buf2node_callback, NULL,
				  &ipc_memcpy_buf2node.dma_ch)) {
		printk(KERN_ERR \
			"%s: Failed to allocate DMA channel for IPC read\n",
			__func__);
		ipc_memcpy_buf2node.dma_ch = -1;
	}
	printk(KERN_INFO "IPC DMA: ch%d for read, ch%d for write\n",
		ipc_memcpy_buf2node.dma_ch, ipc_memcpy_node2buf.dma_ch);
#endif

  /* endpoint bulk in*/
  ipc_endpoint = &(intf->cur_altsetting->endpoint[0].desc);

  //DEBUG("%s: ep num = %d, ep bulk in Attr=0x%x, Addr = 0x%x\n", __FUNCTION__, intf->cur_altsetting->desc.bNumEndpoints, ipc_endpoint->bmAttributes, ipc_endpoint->bEndpointAddress);

  if ( (!IS_EP_BULK_IN(ipc_endpoint)) ) {
    printk("%s: Bulk endpoint bulk in type error\n", __FUNCTION__);
    return -ENOMEM;
  }

  usb_set_intfdata (intf, &usb_ipc_data_param);	
  /* generate read URB */
  /* urb size is the max value of ep MaxPacksize or FrameSize */
  usb_ipc_data_param.read_wMaxPacketSize = ipc_endpoint->wMaxPacketSize;
  usb_fill_bulk_urb(&usb_ipc_data_param.read_urb, dev, usb_rcvbulkpipe(dev,ipc_endpoint->bEndpointAddress), 0, 0, ipc_data_read_callback, 0);

  /* endpoint bulk out*/
  ipc_endpoint = &(intf->cur_altsetting->endpoint[1].desc);

  //DEBUG("%s: ep bulk out Attr=0x%x, Addr = 0x%x\n", __FUNCTION__, ipc_endpoint->bmAttributes, ipc_endpoint->bEndpointAddress);

  if ( (!IS_EP_BULK_OUT(ipc_endpoint)) ) {
    printk("%s: Bulk endpoint bulk out type error\n", __FUNCTION__);
    return -ENOMEM;
  }

  /* generate write URB */
  usb_ipc_data_param.write_wMaxPacketSize = ipc_endpoint->wMaxPacketSize;
  usb_fill_bulk_urb(&usb_ipc_data_param.write_urb, dev, usb_sndbulkpipe(dev,ipc_endpoint->bEndpointAddress), 0, 0, ipc_data_write_callback, 0);

	/* initialize parameters in IPC APIs, register this driver to IPC APIs */
  ipc_api_usb_probe(IPC_DATA_CH_NUM, &usb_ipc_data_param);

#ifdef CONFIG_PM
  spin_lock_init(&usb_ipc_data_param.pm_lock);
  INIT_WORK(&usb_ipc_data_param.wakeup_work, ipc_resume_work);
  INIT_DELAYED_WORK(&usb_ipc_data_param.suspend_work, ipc_suspend_work);

  usb_ipc_data_param.kwakeup_usb_wq = create_singlethread_workqueue("kwakeup_usb_ipcd");
  usb_ipc_data_param.ksuspend_usb_wq = create_singlethread_workqueue("ksuspend_usb_ipcd");

  usb_ipc_data_param.sleeping = 0;
  usb_ipc_data_param.working = 0;
  usb_ipc_data_param.write_urb_used = usb_ipc_data_param.read_urb_used = 0;
  //DEBUG("%s:ipc_probe try to suspend\n", __FUNCTION__);
  usb_autopm_set_interface(usb_ifnum_to_if(usb_ipc_data_param.udev, IPC_DATA_CH_NUM));
#endif
  usb_ipc_data_param.ipc_events = 0;
  kipcd_task = kthread_run(ipc_thread, NULL, "kipcd");

  return 0;
}

/*
 * usb ipc data disconnect
 */
void usb_ipc_data_disconnect(struct usb_interface *intf)
{
	int i;
	unsigned int dma_rev, dma_sys_st, dma_ocp_sys_conf, dma_gcr;
	unsigned int dma_irq_st_0, dma_irq_st_1, dma_irq_st_2, dma_irq_st_3;
	unsigned int dma_irq_en_0, dma_irq_en_1, dma_irq_en_2, dma_irq_en_3;
	unsigned int dma_caps_0, dma_caps_1, dma_caps_2, dma_caps_3;
	unsigned int dma_ccr, dma_clnk_ctrl, dma_cicr, dma_csr;
	unsigned int dma_csdp, dma_cen, dma_cfn, dma_cssa, dma_cdsa;
	unsigned int dma_csei, dma_csfi, dma_cdei, dma_cdfi, dma_csac;
	unsigned int dma_cdac, dma_ccen, dma_ccfn, dma_color;
	unsigned int intc_itr_0, intc_itr_1, intc_itr_2;
	unsigned int intc_mir_0, intc_mir_1, intc_mir_2;

  //DEBUG("Enter %s\n", __FUNCTION__);
  /* unlink URBs */
  kthread_stop(kipcd_task);
#ifdef CONFIG_PM
  cancel_work_sync(&usb_ipc_data_param.wakeup_work);
  cancel_delayed_work_sync(&usb_ipc_data_param.suspend_work);
  destroy_workqueue(usb_ipc_data_param.kwakeup_usb_wq);
  destroy_workqueue(usb_ipc_data_param.ksuspend_usb_wq);
#endif

  usb_unlink_urb (&usb_ipc_data_param.read_urb);
  usb_unlink_urb (&usb_ipc_data_param.write_urb);

	dma_rev = omap_readl(0x48056000);
	dma_irq_st_0 = omap_readl(0x48056008);
	dma_irq_st_1 = omap_readl(0x4805600c);
	dma_irq_st_2 = omap_readl(0x48056010);
	dma_irq_st_3 = omap_readl(0x48056014);
	dma_irq_en_0 = omap_readl(0x48056018);
	dma_irq_en_1 = omap_readl(0x4805601c);
	dma_irq_en_2 = omap_readl(0x48056020);
	dma_irq_en_3 = omap_readl(0x48056024);
	dma_sys_st = omap_readl(0x48056028);
	dma_ocp_sys_conf = omap_readl(0x4805602c);
	dma_caps_0 = omap_readl(0x48056064);
	dma_caps_1 = omap_readl(0x4805606c);
	dma_caps_2 = omap_readl(0x48056070);
	dma_caps_3 = omap_readl(0x48056074);
	dma_gcr = omap_readl(0x48056078);
	dma_ccr = omap_readl(0x48056080 + 0x60 * dma_ch);
	dma_clnk_ctrl = omap_readl(0x48056084 + 0x60 * dma_ch);
	dma_cicr = omap_readl(0x48056088 + 0x60 * dma_ch);
	dma_csr = omap_readl(0x4805608c + 0x60 * dma_ch);
	dma_csdp = omap_readl(0x48056090 + 0x60 * dma_ch);
	dma_cen = omap_readl(0x48056094 + 0x60 * dma_ch);
	dma_cfn = omap_readl(0x48056098 + 0x60 * dma_ch);
	dma_cssa = omap_readl(0x4805609c + 0x60 * dma_ch);
	dma_cdsa = omap_readl(0x480560a0 + 0x60 * dma_ch);
	dma_csei = omap_readl(0x480560a4 + 0x60 * dma_ch);
	dma_csfi = omap_readl(0x480560a8 + 0x60 * dma_ch);
	dma_cdei = omap_readl(0x480560ac + 0x60 * dma_ch);
	dma_cdfi = omap_readl(0x480560b0 + 0x60 * dma_ch);
	dma_csac = omap_readl(0x480560b4 + 0x60 * dma_ch);
	dma_cdac = omap_readl(0x480560b8 + 0x60 * dma_ch);
	dma_ccen = omap_readl(0x480560bc + 0x60 * dma_ch);
	dma_ccfn = omap_readl(0x480560c0 + 0x60 * dma_ch);
	dma_color = omap_readl(0x480560c4 + 0x60 * dma_ch);
	intc_itr_0 = omap_readl(0x48200080);
	intc_itr_1 = omap_readl(0x48200080 + 0x20);
	intc_itr_2 = omap_readl(0x48200080 + 0x40);
	intc_mir_0 = omap_readl(0x48200084);
	intc_mir_1 = omap_readl(0x48200084 + 0x20);
	intc_mir_2 = omap_readl(0x48200084 + 0x40);
	for (i = 0; i < IPC_DBG_ARRAY_SIZE; i++) {
		printk(KERN_INFO "\t0x%02x\n",
			(unsigned int)ipc_dbg_array[ipc_dbg_index++]);
		if (ipc_dbg_index >= IPC_DBG_ARRAY_SIZE)
			ipc_dbg_index = 0;
	}
	printk(KERN_INFO "Current index: %d\n", ipc_dbg_index);
	printk(KERN_INFO \
		"dma_psrc = 0x%08x\n"
		"dma_vdest = 0x%08x\n"
		"dma_pdest = 0x%08x\n"
		"dma_size = %u\n"
		"dma_ch = %u\n",
		dma_psrc, dma_vdest, dma_pdest, dma_size, dma_ch);
	printk(KERN_INFO "DMA Register Dump:\n"
		"dma_rev = 0x%08x\t\tdma_sys_st = 0x%08x\n"
		"dma_irq_st_0 = 0x%08x\tdma_irq_st_1 = 0x%08x\n"
		"dma_irq_st_2 = 0x%08x\tdma_irq_st_3 = 0x%08x\n"
		"dma_irq_en_0 = 0x%08x\tdma_irq_en_1 = 0x%08x\n"
		"dma_irq_en_2 = 0x%08x\tdma_irq_en_3 = 0x%08x\n"
		"dma_caps_0 = 0x%08x\t\tdma_caps_1 = 0x%08x\n"
		"dma_caps_2 = 0x%08x\t\tdma_caps_3 = 0x%08x\n"
		"dma_ocp_sys_conf = 0x%08x\n"
		"dma_gcr = 0x%08x\n",
		dma_rev, dma_sys_st, dma_irq_st_0, dma_irq_st_1,
		dma_irq_st_2, dma_irq_st_3, dma_irq_en_0, dma_irq_en_1,
		dma_irq_en_2, dma_irq_en_3, dma_caps_0, dma_caps_1,
		dma_caps_2, dma_caps_3, dma_ocp_sys_conf, dma_gcr);
	printk(KERN_INFO "DMA Channel Register Dump:\n"
		"dma_ccr = 0x%08x\tdma_clnk_ctrl = 0x%08x\n"
		"dma_cicr = 0x%08x\tdma_csr = 0x%08x\n"
		"dma_csdp = 0x%08x\tdma_cen = 0x%08x\n"
		"dma_cfn = 0x%08x\tdma_cssa = 0x%08x\n"
		"dma_cdsa = 0x%08x\tdma_csei = 0x%08x\n"
		"dma_csfi = 0x%08x\tdma_cdei = 0x%08x\n"
		"dma_cdfi = 0x%08x\tdma_csac = 0x%08x\n"
		"dma_cdac = 0x%08x\tdma_ccen = 0x%08x\n"
		"dma_ccfn = 0x%08x\tdma_color = 0x%08x\n",
		dma_ccr, dma_clnk_ctrl, dma_cicr, dma_csr, dma_csdp,
		dma_cen, dma_cfn, dma_cssa, dma_cdsa, dma_csei, dma_csfi,
		dma_cdei, dma_cdfi, dma_csac, dma_cdac, dma_ccen, dma_ccfn,
		dma_color);
	printk(KERN_INFO "INTC Status and Mask register Dump:\n"
		"intc_itr_0 = 0x%08x\tintc_itr_1 = 0x%08x\n"
		"intc_itr_2 = 0x%08x\tintc_mir_0 = 0x%08x\n"
		"intc_mir_1 = 0x%08x\tintc_mir_2 = 0x%08x\n",
		intc_itr_0, intc_itr_1, intc_itr_2,
		intc_mir_0, intc_mir_1, intc_mir_2);

  usb_set_intfdata (intf, NULL);

  ipc_api_usb_disconnect(IPC_DATA_CH_NUM);

#ifdef USE_OMAP_SDMA
	omap_stop_dma(ipc_memcpy_node2buf.dma_ch);
	omap_free_dma(ipc_memcpy_node2buf.dma_ch);
	omap_stop_dma(ipc_memcpy_buf2node.dma_ch);
	omap_free_dma(ipc_memcpy_buf2node.dma_ch);
	ipc_memcpy_node2buf.dma_ch = -1;
	ipc_memcpy_buf2node.dma_ch = -1;
#endif

  /* re-init "usb_ipc_data_param" */
  usb_ipc_data_init();
}

int usb_ipc_data_init(void)
{
  memset((void *)&usb_ipc_data_param, 0, sizeof(usb_ipc_data_param));
  usb_ipc_data_param.usb_read  = ipc_data_read_buffer;
  usb_ipc_data_param.usb_write = ipc_data_write_buffer;
  usb_init_urb (&usb_ipc_data_param.read_urb);
  usb_init_urb (&usb_ipc_data_param.write_urb);

  return 0;
}

/*
 * driver exit function
 */
void usb_ipc_data_exit(void)
{
}

/************************************************************************
 * IPC USB DRIVER REGISTER 
 ************************************************************************/

/*
 * usb ipc disconnect
 */
static void usb_ipc_disconnect(struct usb_interface *intf)
{
  if(intf->cur_altsetting->desc.bInterfaceNumber == USB_IPC_DATA_IF_NUM) {
    usb_ipc_data_disconnect(intf);
  }
  if(intf->cur_altsetting->desc.bInterfaceNumber == USB_IPC_LOG_IF_NUM) {
    usb_ipc_log_disconnect(intf);
  }
}

/*
 * usb ipc probe
 */
static int usb_ipc_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
  /* according to interface number to detect whether this is IPC DATA/LOG MSG interface */
  if(intf->cur_altsetting->desc.bInterfaceNumber == USB_IPC_DATA_IF_NUM) {
    return usb_ipc_data_probe(intf, id);
  }

  if(intf->cur_altsetting->desc.bInterfaceNumber == USB_IPC_LOG_IF_NUM) {
    return usb_ipc_log_probe(intf, id);
  }

  return -ENOMEM;
}

MODULE_DEVICE_TABLE (usb, usb_ipc_id_table);

#ifdef CONFIG_PM
static int usb_ipc_suspend(struct usb_interface *iface, pm_message_t message)
{
	DEBUG("%s:sleeping=%d working=%d\n", __func__,
		usb_ipc_data_param.sleeping, usb_ipc_data_param.working);
	spin_lock_bh(&usb_ipc_data_param.pm_lock);
	if (!usb_ipc_data_param.allow_suspend) {
		spin_unlock_bh(&usb_ipc_data_param.pm_lock);
		return -EBUSY;
	}
	if (usb_ipc_data_param.working == 1) {
		spin_unlock_bh(&usb_ipc_data_param.pm_lock);
		DEBUG("%s:working, can not suspend\n", __func__);
		return -1;
	}
	if (iface->cur_altsetting->desc.bInterfaceNumber ==
		USB_IPC_DATA_IF_NUM) {
		if (usb_ipc_data_param.sleeping == 1) {
			DEBUG("%s:data interface has already suspended\n",
				__func__);
		} else {
			usb_ipc_data_param.sleeping = 1;
			DEBUG("%s:suspend ipc data interface @ jiffies=%lu\n",
				__func__, jiffies);
		}
	} else if (iface->cur_altsetting->desc.bInterfaceNumber ==
		USB_IPC_LOG_IF_NUM) {
		DEBUG("%s:suspend ipc log interface @ jiffies=%lu\n",
			__func__, jiffies);
	}
	spin_unlock_bh(&usb_ipc_data_param.pm_lock);

	return 0;
}

static int usb_ipc_resume(struct usb_interface *iface)
{
	int ret;
	DEBUG("%s:sleeping=%d working=%d\n", __func__,
		usb_ipc_data_param.sleeping, usb_ipc_data_param.working);
	spin_lock_bh(&usb_ipc_data_param.pm_lock);
	usb_ipc_data_param.allow_suspend = 0;
	if (iface->cur_altsetting->desc.bInterfaceNumber
		== USB_IPC_DATA_IF_NUM) {
		if (usb_ipc_data_param.sleeping == 0) {
			DEBUG("%s:data interface has already resumed\n",
				__func__);
			spin_unlock_bh(&usb_ipc_data_param.pm_lock);
			return -1;
		} else {
			usb_ipc_data_param.sleeping = 0;
			DEBUG("%s:resume ipc data interface @ jiffies=%lu\n",
				__func__, jiffies);
		}
		if (usb_ipc_data_param.read_urb_used) {
			ret = usb_submit_urb(&usb_ipc_data_param.read_urb,
				GFP_ATOMIC|GFP_DMA);
			usb_ipc_data_param.read_urb_used = 0;
			ipc_dbg_array[ipc_dbg_index++] = 0x4;
			if (ipc_dbg_index >= IPC_DBG_ARRAY_SIZE)
				ipc_dbg_index = 0;
			DEBUG("data read urb restarted, ret=%d.\n", ret);
		}
		if (usb_ipc_data_param.write_urb_used) {
			usb_ipc_data_param.working = 1;
			ret = usb_submit_urb(&usb_ipc_data_param.write_urb,
				GFP_ATOMIC|GFP_DMA);
			usb_ipc_data_param.write_urb_used = 0;
			DEBUG("data write urb restarted, ret=%d.\n", ret);
		}
	} else if (iface->cur_altsetting->desc.bInterfaceNumber ==
		USB_IPC_LOG_IF_NUM) {
		DEBUG("%s:resume ipc log interface @ jiffies=%lu\n",
			__func__, jiffies);
		if ((ipc_log_param.write_buf != NULL)
			&& (ipc_log_param.urb_flag == 0)) {
			ipc_log_param.read_urb.transfer_buffer =
				ipc_log_param.write_buf->ptr;
			ipc_log_param.read_urb.transfer_buffer_length =
				ipc_log_param.read_bufsize;
			ret = usb_submit_urb(&ipc_log_param.read_urb,
				GFP_KERNEL);
			if (!ret)
				ipc_log_param.urb_flag = 1;
			DEBUG("log read urb restarted, ret=%d.\n", ret);
		}
	}
	spin_unlock_bh(&usb_ipc_data_param.pm_lock);

	return 0;
}
#endif

/* USB host stack entry fucntion for this driver */
static struct usb_driver usb_ipc_driver = {
	name:		"usb_ipc_data",
	probe:		usb_ipc_probe,
	disconnect:	usb_ipc_disconnect,
	id_table:	usb_ipc_id_table,
#ifdef CONFIG_PM
	supports_autosuspend:	1,
	suspend:	usb_ipc_suspend,
	resume:		usb_ipc_resume,
#endif
};

/*
 * driver module init/exit functions
 */
static int __init usb_ipc_init(void)
{
  int result;

	ipc_dbg_index = 0;
	memset((void *)ipc_dbg_array, 0xff, IPC_DBG_ARRAY_SIZE);

  /* IPC API relevant initialization */
  ipc_api_init();

  /* ipc DATA interface relevant initialization */
  result = usb_ipc_data_init();
  if(result != 0) {
    return result;
  }

  /* ipc DATA interface log initialization */
  result = usb_ipc_log_init();
  if(result != 0) {
    usb_ipc_data_exit();
    return result;
  }

  result = usb_register(&usb_ipc_driver);
  if (result < 0) {
    usb_ipc_data_exit();
    usb_ipc_log_exit();
    printk("%s: Register USB IPC driver failed", __FUNCTION__);
    return -1;
  }

  return 0;
}

/*
 * driver exit function
 */
static void __exit usb_ipc_exit(void)
{
  /* IPC API relevant exit */
  ipc_api_exit();

  /* USB IPC DATA driver exit */
  usb_ipc_data_exit();

  /* USB IPC LOG driver exit */
  usb_ipc_log_exit();

  /* unregister USB IPC driver */
  usb_deregister(&usb_ipc_driver);
}

/* the module entry declaration of this driver */
module_init(usb_ipc_init);
module_exit(usb_ipc_exit);


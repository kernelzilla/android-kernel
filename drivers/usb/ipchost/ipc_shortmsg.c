/*
 * Copyright (C) 2007-2008 Motorola, Inc
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
 * 
 * Changelog:
 * Date               Author           Comment
 * -----------------------------------------------------------------------------
 * 12/07/2007      Motorola        USB-IPC initial
 * 05/09/2008      Motorola        Change Copyright and Changelog
 * 
 */
 
/*!
 * @file drivers/usb/ipchost/ipc_shortmsg.c
 * @brief USB-IPC short message driver
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
#include <asm/hardware.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/major.h>
#include <linux/usb_ipc.h>

#define DEBUG(args...) printk(args)

#define IPC_SHORT_FRAME_SIZE   50

/* USB device ID information */
static struct usb_device_id usb_ipc_id_table [] = {
	{ USB_DEVICE(MOTO_USBIPC_VID, MOTO_USBIPC_PID) },
	{ }						/* Terminating entry */
};

// USB endpoint detection
#define IS_EP_INT(ep)     (((ep)->bmAttributes) == USB_ENDPOINT_XFER_INT ? 1 : 0)
#define IS_EP_INT_IN(ep)  (IS_EP_INT(ep) && (((ep)->bEndpointAddress) & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
#define IS_EP_INT_OUT(ep) (IS_EP_INT(ep) && (((ep)->bEndpointAddress) & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)



/* setup packet */
struct usb_ctrlrequest  usb_ipc_short_setup_pack = {
//	USB_IPC_SHORT_CONTROL_REQUEST_TYPE,
//	USB_IPC_SHORT_CONTROL_REQUEST,
//	USB_IPC_SHORT_CONTROL_VALUE,
//	USB_IPC_SHORT_CONTROL_INDEX,
//	USB_IPC_SHORT_CONTROL_LENGTH
};

/* Short message class driver interfaces with IPC APIs  */
USB_IPC_IFS_STRUCT usb_ipc_short_param;
static int ipc_short_probe_flag = 0;

/*
 *  This function is used as the short message INT IN endpoint callback
 */
static void ipc_short_read_callback(struct urb *urb)
{
	DEBUG("Enter %s\n", __FUNCTION__);
	if(usb_ipc_short_param.ipc_read_cb != NULL) {
//		usb_ipc_short_param.ipc_read_cb(IPC_LOG_CH_NUM, urb);
	}
}

/*
 *  This function is used as the short message EP0 endpoint callback
 */
static void ipc_short_write_callback(struct urb *urb)
{
	DEBUG("Enter %s\n", __FUNCTION__);
	if(usb_ipc_short_param.ipc_write_cb != NULL) {
//		usb_ipc_short_param.ipc_write_cb(IPC_LOG_CH_NUM, urb);
	}
}

/*
 * usb ipc data driver probe function 
 */
int usb_ipc_short_probe(struct usb_interface *intf, const struct usb_device_id *id)
{	
	struct usb_endpoint_descriptor  *ipc_endpoint;		
	struct usb_device *dev = interface_to_usbdev (intf);

	ipc_endpoint = &(intf->cur_altsetting->endpoint[0].desc);

        DEBUG("%s: ep num = %d, ep Attr=0x%x, Addr = 0x%x\n", __FUNCTION__, intf->cur_altsetting->desc.bNumEndpoints, ipc_endpoint->bmAttributes, ipc_endpoint->bEndpointAddress);

        if ( (!IS_EP_INT_IN(ipc_endpoint)) ) {
                printk("%s: Int endpoint in type error\n", __FUNCTION__);
                return -ENOMEM;
        }

	usb_ipc_short_param.udev = dev;
	/* generate URB */
	/* urb size is the max value of ep MaxPacksize or FrameSize */
	usb_ipc_short_param.read_bufsize = (IPC_SHORT_FRAME_SIZE > ipc_endpoint->wMaxPacketSize)? IPC_SHORT_FRAME_SIZE : ipc_endpoint->wMaxPacketSize;
	usb_fill_int_urb(&usb_ipc_short_param.read_urb, dev, usb_rcvintpipe(dev,ipc_endpoint->bEndpointAddress), 
	                  0, 0, ipc_short_read_callback, 0, ipc_endpoint->bInterval);

	/* control endpoint ep0 is used to send command */
	usb_ipc_short_param.write_bufsize = (IPC_SHORT_FRAME_SIZE > dev->descriptor.bMaxPacketSize0)? IPC_SHORT_FRAME_SIZE : dev->descriptor.bMaxPacketSize0;
	usb_fill_control_urb(&usb_ipc_short_param.write_urb, dev, usb_sndctrlpipe (dev, 0), (void *)&usb_ipc_short_setup_pack,
	                  0, 0, ipc_short_write_callback, 0);

	/* initialize parameters in IPC APIs, register this driver to IPC APIs */
//	usb_ipc_api_initialization(IPC_SHORT_CH_NUM, &usb_ipc_short_param);

	ipc_short_probe_flag = 1;
	return 0;
}

/*
 * usb ipc data disconnect
 */
void usb_ipc_short_disconnect(struct usb_interface *intf)
{
	if(ipc_short_probe_flag != 0)  	{
		DEBUG("Enter %s\n", __FUNCTION__);
		/* unlink URBs */
		usb_unlink_urb (&usb_ipc_short_param.read_urb);
		usb_unlink_urb (&usb_ipc_short_param.write_urb);
		ipc_short_probe_flag = 0;
	}
}

/*
 * driver init function
 */
int usb_ipc_short_init(void)
{
	memset((void *)&usb_ipc_short_param, 0, sizeof(usb_ipc_short_param));
	usb_init_urb(&usb_ipc_short_param.read_urb);	
	usb_init_urb(&usb_ipc_short_param.write_urb);	
	return 0;
}

/*
 * driver exit function
 */
void usb_ipc_short_exit(void)
{
}


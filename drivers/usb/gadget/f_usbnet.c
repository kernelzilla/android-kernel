/*
 * Gadget Driver for Motorola USBNet
 *
 * Copyright (C) 2009 Motorola Corporation
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
 */

#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/spinlock.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/if.h>
#include <linux/inetdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <asm/cacheflush.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <asm/cacheflush.h>
#include "f_usbnet.h"

#include "f_mot_android.h"

/*
 * Macro Defines
 */

#define EP0_BUFSIZE		256

/* Vendor Request to config IP */
#define USBNET_SET_IP_ADDRESS   0x05
#define USBNET_SET_SUBNET_MASK  0x06
#define USBNET_SET_HOST_IP      0x07

/* Linux Network Interface */
#define USB_MTU                 1536
#define MAX_BULK_TX_REQ_NUM	8
#define MAX_BULK_RX_REQ_NUM	8
#define MAX_INTR_RX_REQ_NUM	8

struct usbnet_if_configuration {
	u32 ip_addr;
	u32 subnet_mask;
	u32 router_ip;
	u32 iff_flag;
	struct work_struct usbnet_config_wq;
	struct net_device *usbnet_config_dev;
};

struct usbnet_context {
	spinlock_t lock;  /* For RX/TX list */
	struct net_device *dev;

	struct usb_gadget *gadget;

	struct usb_ep *bulk_in;
	struct usb_ep *bulk_out;
	struct usb_ep *intr_out;
	u16 config;		/* current USB config w_value */

	struct list_head rx_reqs;
	struct list_head tx_reqs;

	struct net_device_stats stats;
};

struct usbnet_device {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	struct usbnet_context *net_ctxt;
};

static struct usbnet_device             g_usbnet_device;
static struct usbnet_context 		*g_usbnet_context;
static struct net_device     		*g_net_dev;
static struct usbnet_if_configuration 	g_usbnet_ifc;

#ifdef CONFIG_USB_MOT_ANDROID
/* used when eth function is disabled */
static struct usb_descriptor_header *null_function[] = {
	NULL,
};
#endif

/*
 * USB descriptors
 */
#define STRING_INTERFACE        0

/* static strings, in UTF-8 */
static struct usb_string usbnet_string_defs[] = {
	[STRING_INTERFACE].s = "Motorola Networking Interface",
	{  /* ZEROES END LIST */ },
};

static struct usb_gadget_strings usbnet_string_table = {
	.language =             0x0409, /* en-us */
	.strings =              usbnet_string_defs,
};

static struct usb_gadget_strings *usbnet_strings[] = {
	&usbnet_string_table,
	NULL,
};

/* There is only one interface. */
static struct usb_interface_descriptor intf_desc = {
	.bLength = sizeof intf_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bNumEndpoints = 3,
	.bInterfaceClass = 0x02,
	.bInterfaceSubClass = 0x0a,
	.bInterfaceProtocol = 0x01,
};


static struct usb_endpoint_descriptor fs_bulk_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_intr_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.bInterval = 1,
};

static struct usb_descriptor_header *fs_function[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_intr_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
	.bInterval = 0,
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
	.bInterval = 0,
};

static struct usb_endpoint_descriptor hs_intr_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize = __constant_cpu_to_le16(64),
	.bInterval = 1,
};

static struct usb_descriptor_header *hs_function[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_intr_out_desc,
	NULL,
};

#define DO_NOT_STOP_QUEUE 0
#define STOP_QUEUE 1

#define USBNETDBG(context, fmt, args...)				\
	if (context && context->gadget)					\
		dev_dbg(&(context->gadget->dev) , fmt , ## args)


static inline struct usbnet_device *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct usbnet_device, function);
}

static int ether_queue_out(struct usb_request *req)
{
	unsigned long flags;
	struct sk_buff *skb;
	int ret;

	if (cpu_is_omap3630())
		skb = alloc_skb(USB_MTU + NET_IP_ALIGN + 2, GFP_ATOMIC);
	else
		skb = alloc_skb(USB_MTU + NET_IP_ALIGN, GFP_ATOMIC);
	if (!skb) {
		printk(KERN_INFO "%s: failed to alloc skb\n", __func__);
		ret = -ENOMEM;
		goto fail;
	}

	if (cpu_is_omap3630())
		skb_reserve(skb, NET_IP_ALIGN + 2);
	else
		skb_reserve(skb, NET_IP_ALIGN);

	req->buf = skb->data;
	req->length = USB_MTU;
	req->context = skb;

	ret = usb_ep_queue(g_usbnet_context->bulk_out, req, GFP_KERNEL);
	if (ret == 0)
		return 0;
	dev_kfree_skb_any(skb);
fail:
	spin_lock_irqsave(&g_usbnet_context->lock, flags);
	list_add_tail(&req->list, &g_usbnet_context->rx_reqs);
	spin_unlock_irqrestore(&g_usbnet_context->lock, flags);

	return ret;
}

static int usb_ether_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct usb_request *req;
	unsigned long flags;
	unsigned len;
	int rc;

	spin_lock_irqsave(&g_usbnet_context->lock, flags);
	if (list_empty(&g_usbnet_context->tx_reqs)) {
		req = 0;
	} else {
		req = list_first_entry(&g_usbnet_context->tx_reqs,
				       struct usb_request, list);
		list_del(&req->list);
		if (list_empty(&g_usbnet_context->tx_reqs))
			netif_stop_queue(dev);
	}
	spin_unlock_irqrestore(&g_usbnet_context->lock, flags);

	if (!req) {
		printk(KERN_INFO "%s: could not obtain tx request\n", __func__);
		return 1;
	}

	/* Add 4 bytes CRC */
	skb->len += 4;

	/* ensure that we end with a short packet */
	len = skb->len;
	if (!(len & 63) || !(len & 511))
		len++;

	req->context = skb;
	req->buf = skb->data;
	req->length = len;

	rc = usb_ep_queue(g_usbnet_context->bulk_in, req, GFP_KERNEL);
	if (rc != 0) {
		spin_lock_irqsave(&g_usbnet_context->lock, flags);
		list_add_tail(&req->list, &g_usbnet_context->tx_reqs);
		spin_unlock_irqrestore(&g_usbnet_context->lock, flags);

		dev_kfree_skb_any(skb);
		g_usbnet_context->stats.tx_dropped++;

		printk(KERN_INFO "%s: could not queue tx request\n", __func__);
	}

	return 0;
}

static int usb_ether_open(struct net_device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

static int usb_ether_stop(struct net_device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

static struct net_device_stats *usb_ether_get_stats(struct net_device *dev)
{
	printk(KERN_INFO "%s\n", __func__);
	return &g_usbnet_context->stats;
}

static void usbnet_if_config(struct work_struct *work)
{
	struct ifreq ifr;
	mm_segment_t saved_fs;
	unsigned err;
	struct sockaddr_in *sin;

	memset(&ifr, 0, sizeof(ifr));
	sin = (void *) &(ifr.ifr_ifru.ifru_addr);
	strncpy(ifr.ifr_ifrn.ifrn_name, "usb0", strlen("usb0") + 1);
	sin->sin_family = AF_INET;

	sin->sin_addr.s_addr = g_usbnet_ifc.ip_addr;
	saved_fs = get_fs();
	set_fs(get_ds());
	err = devinet_ioctl(dev_net(g_usbnet_ifc.usbnet_config_dev),
			  SIOCSIFADDR, &ifr);

	sin->sin_addr.s_addr = g_usbnet_ifc.subnet_mask;
	err = devinet_ioctl(dev_net(g_usbnet_ifc.usbnet_config_dev),
			  SIOCSIFNETMASK, &ifr);

	sin->sin_addr.s_addr =
	    g_usbnet_ifc.ip_addr | ~(g_usbnet_ifc.subnet_mask);
	err = devinet_ioctl(dev_net(g_usbnet_ifc.usbnet_config_dev),
			  SIOCSIFBRDADDR, &ifr);

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, "usb0", strlen("usb0") + 1);
	if (g_usbnet_ifc.iff_flag & IFF_UP)
		ifr.ifr_flags = ((g_usbnet_ifc.usbnet_config_dev->flags) |
			g_usbnet_ifc.iff_flag);
	else
		ifr.ifr_flags = (g_usbnet_ifc.usbnet_config_dev->flags)&
			~IFF_UP;

	err = devinet_ioctl(dev_net(g_usbnet_ifc.usbnet_config_dev),
			  SIOCSIFFLAGS, &ifr);

	set_fs(saved_fs);

	if (g_usbnet_ifc.iff_flag == IFF_UP)
		usb_interface_enum_cb(ETH_TYPE_FLAG);
	else {
		g_usbnet_ifc.subnet_mask = 0;
		g_usbnet_ifc.router_ip   = 0;
	}
}

static void __init usb_ether_setup(struct net_device *dev)
{
	g_usbnet_context = netdev_priv(dev);
	INIT_LIST_HEAD(&g_usbnet_context->rx_reqs);
	INIT_LIST_HEAD(&g_usbnet_context->tx_reqs);

	spin_lock_init(&g_usbnet_context->lock);
	g_usbnet_context->dev = dev;

	dev->open = usb_ether_open;
	dev->stop = usb_ether_stop;
	dev->hard_start_xmit = usb_ether_xmit;
	dev->get_stats = usb_ether_get_stats;
	dev->watchdog_timeo = 20;

	ether_setup(dev);

	random_ether_addr(dev->dev_addr);
}

/*-------------------------------------------------------------------------*/
static void usbnet_cleanup(void)
{
	if (g_net_dev) {
		unregister_netdev(g_net_dev);
		free_netdev(g_net_dev);
		g_net_dev = NULL;
		g_usbnet_context = NULL;
	}
}

static void usbnet_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct usbnet_device *dev = func_to_dev(f);
	struct usb_composite_dev *cdev = c->cdev;
	struct usb_request *req;
	unsigned long flags;

	dev->cdev = cdev;

	/* Free EP0 Request */
	usb_ep_disable(g_usbnet_context->bulk_in);
	usb_ep_disable(g_usbnet_context->bulk_out);
	usb_ep_disable(g_usbnet_context->intr_out);

	/* Free BULK OUT Requests */
	for (;;) {
		spin_lock_irqsave(&g_usbnet_context->lock, flags);
		if (list_empty(&g_usbnet_context->rx_reqs)) {
			break;
		} else {
			req = list_first_entry(&g_usbnet_context->rx_reqs,
					       struct usb_request, list);
			list_del(&req->list);
		}
		spin_unlock_irqrestore(&g_usbnet_context->lock, flags);
		if (!req)
			usb_ep_free_request(g_usbnet_context->bulk_out, req);
	}

	/* Free BULK IN Requests */
	for (;;) {
		spin_lock_irqsave(&g_usbnet_context->lock, flags);
		if (list_empty(&g_usbnet_context->tx_reqs)) {
			break;
		} else {
			req = list_first_entry(&g_usbnet_context->tx_reqs,
					       struct usb_request, list);
			list_del(&req->list);
		}
		spin_unlock_irqrestore(&g_usbnet_context->lock, flags);
		if (!req)
			usb_ep_free_request(g_usbnet_context->bulk_in, req);
	}

	g_usbnet_context->config = 0;

	usbnet_cleanup();
}

static void ether_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct sk_buff *skb = req->context;

	if (req->status == 0) {
		dmac_inv_range((void *)req->buf, (void *)(req->buf +
					req->actual));
		skb_put(skb, req->actual);
		skb->protocol = eth_type_trans(skb, g_usbnet_context->dev);
		g_usbnet_context->stats.rx_packets++;
		g_usbnet_context->stats.rx_bytes += req->actual;
		netif_rx(skb);
	} else {
		dev_kfree_skb_any(skb);
		g_usbnet_context->stats.rx_errors++;
	}

	/* don't bother requeuing if we just went offline */
	if ((req->status == -ENODEV) || (req->status == -ESHUTDOWN)) {
		unsigned long flags;
		spin_lock_irqsave(&g_usbnet_context->lock, flags);
		list_add_tail(&req->list, &g_usbnet_context->rx_reqs);
		spin_unlock_irqrestore(&g_usbnet_context->lock, flags);
	} else {
		if (ether_queue_out(req))
			printk(KERN_INFO "ether_out: cannot requeue\n");
	}
}

static void ether_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned long flags;
	struct sk_buff *skb = req->context;

	if (req->status == 0) {
		g_usbnet_context->stats.tx_packets++;
		g_usbnet_context->stats.tx_bytes += req->actual;
	} else {
		g_usbnet_context->stats.tx_errors++;
	}

	dev_kfree_skb_any(skb);

	spin_lock_irqsave(&g_usbnet_context->lock, flags);
	if (list_empty(&g_usbnet_context->tx_reqs))
		netif_start_queue(g_usbnet_context->dev);

	list_add_tail(&req->list, &g_usbnet_context->tx_reqs);
	spin_unlock_irqrestore(&g_usbnet_context->lock, flags);
}

static int __init usbnet_bind(struct usb_configuration *c,
			struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct usbnet_device  *dev = func_to_dev(f);
	int n, rc, id;
	struct usb_ep *ep;
	struct usb_request *req;
	unsigned long flags;

	dev->cdev = cdev;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	intf_desc.bInterfaceNumber = id;
	g_usbnet_context->gadget = cdev->gadget;

	/* config EPs */
	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_in_desc);
	if (!ep) {
		printk(KERN_INFO "%s auto-configure hs_bulk_in_desc error\n",
			__func__);
		goto autoconf_fail;
	}
	ep->driver_data = g_usbnet_context;
	g_usbnet_context->bulk_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_out_desc);
	if (!ep) {
		printk(KERN_INFO "%s auto-configure hs_bulk_out_desc error\n",
		      __func__);
		goto autoconf_fail;
	}
	ep->driver_data = g_usbnet_context;
	g_usbnet_context->bulk_out = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_intr_out_desc);
	if (!ep) {
		printk(KERN_INFO "%s auto-configure hs_intr_out_desc error\n",
		      __func__);
		goto autoconf_fail;
	}
	ep->driver_data = g_usbnet_context;
	g_usbnet_context->intr_out = ep;


	if (gadget_is_dualspeed(cdev->gadget)) {

		/* Assume endpoint addresses are the same for both speeds */
		hs_bulk_in_desc.bEndpointAddress =
		    fs_bulk_in_desc.bEndpointAddress;
		hs_bulk_out_desc.bEndpointAddress =
		    fs_bulk_out_desc.bEndpointAddress;
		hs_intr_out_desc.bEndpointAddress =
		    fs_intr_out_desc.bEndpointAddress;
	}


	rc = -ENOMEM;

	/* Allocate the request and buffer for endpoint 0 */
	for (n = 0; n < MAX_BULK_RX_REQ_NUM; n++) {
		req = usb_ep_alloc_request(g_usbnet_context->bulk_out,
					 GFP_KERNEL);
		if (!req) {
			printk(KERN_INFO "%s: alloc request bulk_out fail\n",
				__func__);
			break;
		}
		req->complete = ether_out_complete;
		spin_lock_irqsave(&g_usbnet_context->lock, flags);
		list_add_tail(&req->list, &g_usbnet_context->rx_reqs);
		spin_unlock_irqrestore(&g_usbnet_context->lock, flags);
	}
	for (n = 0; n < MAX_BULK_TX_REQ_NUM; n++) {
		req = usb_ep_alloc_request(g_usbnet_context->bulk_in,
					 GFP_KERNEL);
		if (!req) {
			printk(KERN_INFO "%s: alloc request bulk_in fail\n",
				__func__);
			break;
		}
		req->complete = ether_in_complete;
		spin_lock_irqsave(&g_usbnet_context->lock, flags);
		list_add_tail(&req->list, &g_usbnet_context->tx_reqs);
		spin_unlock_irqrestore(&g_usbnet_context->lock, flags);
	}

	usb_gadget_set_selfpowered(cdev->gadget);
	return 0;

autoconf_fail:
	rc = -ENOTSUPP;

	usbnet_unbind(c, f);
	return rc;
}

static void do_set_config(u16 new_config)
{
	int result = 0;
	unsigned long flags;
	struct usb_request *req;
	int high_speed_flag = 0;

	if (g_usbnet_context->config == new_config) /* Config did not change */
		return;

	g_usbnet_context->config = new_config;

	if (new_config == 1) { /* Enable End points */
		if (gadget_is_dualspeed(g_usbnet_context->gadget)
		    && g_usbnet_context->gadget->speed == USB_SPEED_HIGH)
			high_speed_flag = 1;

		if (high_speed_flag)
			result = usb_ep_enable(g_usbnet_context->bulk_in,
					  &hs_bulk_in_desc);
		else
			result = usb_ep_enable(g_usbnet_context->bulk_in,
					  &fs_bulk_in_desc);

		if (result != 0) {
			printk(KERN_INFO "%s:  failed to enable BULK_IN EP ret=%d\n",
			      __func__, result);
		}

		if (high_speed_flag)
			result = usb_ep_enable(g_usbnet_context->bulk_out,
					  &hs_bulk_out_desc);
		else
			result = usb_ep_enable(g_usbnet_context->bulk_out,
				  &fs_bulk_out_desc);

		if (result != 0) {
			printk(KERN_INFO "%s:  failed to enable BULK_OUT EP ret = %d\n",
			      __func__, result);
		}

		if (high_speed_flag)
			result = usb_ep_enable(g_usbnet_context->intr_out,
					&hs_intr_out_desc);
		else
			result = usb_ep_enable(g_usbnet_context->intr_out,
					&fs_intr_out_desc);

		if (result != 0) {
			printk(KERN_INFO "%s: failed to enable INTR_OUT EP ret = %d\n",
				__func__, result);
		}




		/* we're online -- get all rx requests queued */
		for (;;) {
			spin_lock_irqsave(&g_usbnet_context->lock, flags);
			if (list_empty(&g_usbnet_context->rx_reqs)) {
				req = 0;
			} else {
				req = list_first_entry(
						&g_usbnet_context->rx_reqs,
						struct usb_request, list);
				list_del(&req->list);
			}
			spin_unlock_irqrestore(&g_usbnet_context->lock, flags);
			if (!req)
				break;
			if (ether_queue_out(req)) {
				printk(KERN_INFO "%s: ether_queue_out failed\n",
					__func__);
				break;
			}
		}
		netif_start_queue(g_net_dev);

	} else {
		netif_stop_queue(g_net_dev);
		g_usbnet_ifc.ip_addr = 0;
		g_usbnet_ifc.iff_flag = 0;
		g_usbnet_ifc.usbnet_config_dev = g_usbnet_context->dev;
		schedule_work(&g_usbnet_ifc.usbnet_config_wq);
		/* Disable Endpoints */
		if (g_usbnet_context->bulk_in)
			usb_ep_disable(g_usbnet_context->bulk_in);
		if (g_usbnet_context->bulk_out)
			usb_ep_disable(g_usbnet_context->bulk_out);
		if (g_usbnet_context->intr_out)
			usb_ep_disable(g_usbnet_context->intr_out);
	}
}


static int usbnet_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	printk(KERN_INFO "usbnet_set_alt intf: %d alt: %d\n", intf, alt);
	do_set_config(1);
	return 0;
}

static int usbnet_setup(struct usb_function *f,
			const struct usb_ctrlrequest *ctrl)
{

	int rc = -EOPNOTSUPP;
	int wIndex = le16_to_cpu(ctrl->wIndex);
	int wValue = le16_to_cpu(ctrl->wValue);
	u16 wLength = le16_to_cpu(ctrl->wLength);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		switch (ctrl->bRequest) {
		case USBNET_SET_IP_ADDRESS:
			g_usbnet_ifc.ip_addr = (wValue << 16) | wIndex;
			rc = 0;
			break;
		case USBNET_SET_SUBNET_MASK:
			g_usbnet_ifc.subnet_mask = (wValue << 16) | wIndex;
			rc = 0;
			break;
		case USBNET_SET_HOST_IP:
			g_usbnet_ifc.router_ip = (wValue << 16) | wIndex;
			rc = 0;
			break;
		default:
			break;
		}

		if (g_usbnet_ifc.ip_addr && g_usbnet_ifc.subnet_mask
		    && g_usbnet_ifc.router_ip) {
			/* schedule a work queue to do this because we
				 need to be able to sleep */
			g_usbnet_ifc.usbnet_config_dev = g_usbnet_context->dev;
			g_usbnet_ifc.iff_flag = IFF_UP;
			schedule_work(&g_usbnet_ifc.usbnet_config_wq);
		}
	}

	/* respond with data transfer or status phase? */
	if (rc >= 0) {
		req->zero = rc < wLength;
		req->length = rc;
		rc = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (rc < 0)
			printk(KERN_INFO "usbnet setup response error\n");
	}

	return rc;
}

static void usbnet_disable(struct usb_function *f)
{
	printk(KERN_INFO "%s\n", __func__);
	do_set_config(0);
}

static void usbnet_suspend(struct usb_function *f)
{
	printk(KERN_INFO "%s\n", __func__);
}

static void usbnet_resume(struct usb_function *f)
{
	printk(KERN_INFO "%s\n", __func__);
}


int __init usbnet_function_add(struct usb_composite_dev *cdev,
		struct usb_configuration *c)
{
	struct usbnet_device *dev;
	int ret, status;

	printk(KERN_INFO "usbnet_function_add\n");

	status = usb_string_id(c->cdev);
	if (status >= 0) {
		usbnet_string_defs[STRING_INTERFACE].id = status;
		intf_desc.iInterface = status;
	}

	g_net_dev = alloc_netdev(sizeof(struct usbnet_context),
			   "usb%d", usb_ether_setup);
	if (!g_net_dev) {
		printk(KERN_INFO "%s: alloc_netdev error\n", __func__);
		return -EINVAL;
	}

	ret = register_netdev(g_net_dev);
	if (ret) {
		printk(KERN_INFO "%s: register_netdev error\n", __func__);
		free_netdev(g_net_dev);
		return -EINVAL;
	} else {
		INIT_WORK(&g_usbnet_ifc.usbnet_config_wq, usbnet_if_config);
		g_usbnet_context = netdev_priv(g_net_dev);
	}

	g_usbnet_context->config = 0;

	dev = &g_usbnet_device;
	dev->net_ctxt = g_usbnet_context;
	dev->cdev = cdev;
	dev->function.name = "usbnet";

#ifdef CONFIG_USB_MOT_ANDROID
	dev->function.descriptors = null_function;
	dev->function.hs_descriptors = null_function;
#else
	dev->function.descriptors = fs_function;
	dev->function.hs_descriptors = hs_function;
#endif
	dev->function.strings = usbnet_strings;
	dev->function.bind = usbnet_bind;
	dev->function.unbind = usbnet_unbind;
	dev->function.set_alt = usbnet_set_alt;
	dev->function.disable = usbnet_disable;
	dev->function.setup = usbnet_setup;
	dev->function.suspend = usbnet_suspend;
	dev->function.resume = usbnet_resume;

	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "usbnet gadget driver failed to initialize\n");
	usbnet_cleanup();
	return ret;
}

#ifdef CONFIG_USB_MOT_ANDROID
struct usb_function *usbnet_function_enable(int enable, int id)
{
	printk(KERN_DEBUG "%s enable=%d id = %d\n", __func__, enable, id);
	if (g_usbnet_context) {
		if (enable) {
			g_usbnet_device.function.descriptors = fs_function;
			g_usbnet_device.function.hs_descriptors = hs_function;
			intf_desc.bInterfaceNumber = id;
		} else {
			g_usbnet_device.function.descriptors = null_function;
			g_usbnet_device.function.hs_descriptors = null_function;
		}
		return &g_usbnet_device.function;
	}
	return NULL;
}
#endif


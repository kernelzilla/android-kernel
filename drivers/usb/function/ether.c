/* drivers/usb/function/ether.c
 *
 * Simple Ethernet Function Device
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Implements the "cdc_subset" bulk-only protocol supported by Linux.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "usb_function.h"

#if defined(CONFIG_KERNEL_MOTOROLA) || defined(CONFIG_MACH_MOT)
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <asm/atomic.h>

#include <linux/if.h>
#include <linux/in.h>
#include <linux/inetdevice.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>

#define ETHER_INTERFACE_NAME "Motorola Networking Interface"
#define ETHER_FUNCTION_NAME  "ethernet"
#endif

/* Ethernet frame is 1514 + FCS, but round up to 512 * 3 so we
 * always queue a multiple of the USB max packet size (64 or 512)
 */
#define USB_MTU 1536

#define MAX_TX 8
#define MAX_RX 8

struct ether_context {
	spinlock_t lock;
	struct net_device *dev;
	struct usb_endpoint *out;
	struct usb_endpoint *in;
#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
	struct usb_endpoint *int_out;

	atomic_t enable_excl;
#endif

	struct list_head rx_reqs;
	struct list_head tx_reqs;

	struct net_device_stats stats;
};

#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
static struct usb_interface_descriptor intf_desc = {
	.bLength = sizeof intf_desc,
	.bDescriptorType = USB_DT_INTERFACE,
	.bNumEndpoints = 3,
	.bInterfaceClass = 0x02,
	.bInterfaceSubClass = 0x0a,
	.bInterfaceProtocol = 0x01,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
	.bInterval = 0,
};
static struct usb_endpoint_descriptor fs_bulk_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(64),
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
static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(64),
	.bInterval = 0, 
};

static struct usb_endpoint_descriptor hs_int_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize = __constant_cpu_to_le16(64),
	.bInterval = 1, 
};
static struct usb_endpoint_descriptor fs_int_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize = __constant_cpu_to_le16(64),
	.bInterval = 1, 
};

static struct usb_function usb_func_ether;

struct ether_if_configuration {
	u32 ip_addr;
	u32 subnet_mask;
	u32 router_ip;
	struct work_struct ether_config_wq;
	struct net_device *ether_config_dev;
};

static struct ether_if_configuration eifc;

#define MOT_USB_SET_TIME         0x04
#define MOT_USB_SET_IP_ADDRESS   0x05
#define MOT_USB_SET_SUBNET_MASK  0x06
#define MOT_USB_SET_HOST_IP      0x07
#endif

#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
static int ether_bound;
#endif

static int ether_bound;

static int ether_queue_out(struct ether_context *ctxt,
			   struct usb_request *req);
static void ether_in_complete(struct usb_endpoint *ept,
			      struct usb_request *req);
static void ether_out_complete(struct usb_endpoint *ept,
			       struct usb_request *req);
#if defined(CONFIG_KERNEL_MOTOROLA) || defined(CONFIG_MACH_MOT)
static void ether_int_dummy(struct usb_endpoint *ept,
				struct usb_request *req);
static int ether_req_setup(struct usb_ctrlrequest *req, void *buf,
				int len, void *context);
static void ether_if_config(struct work_struct *work);

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static void ether_int_dummy(struct usb_endpoint *ept,
                             struct usb_request *req)
{
	/* we don't need to support anything here */
	return;
}

static int ether_req_setup (struct usb_ctrlrequest *req, void *buf,
                       int len, void *_ctxt)
{
	/* grab arguments, init return value */
	unsigned ret = -EOPNOTSUPP;
	struct ether_context *ctxt = _ctxt;
	u16 wIndex;
	u16 wValue;

	switch(req->bRequest) {
	case MOT_USB_SET_IP_ADDRESS:
		pr_info("%s: set ip address request\n", __func__);
		wIndex = le16_to_cpu(req->wIndex);
		wValue = le16_to_cpu(req->wValue);
		eifc.ip_addr = (wValue << 16) | wIndex;
		ret = 0;
		break;
	case MOT_USB_SET_SUBNET_MASK:
		pr_info("%s: set subnet mask request\n", __func__);
		wIndex = le16_to_cpu(req->wIndex);
		wValue = le16_to_cpu(req->wValue);
		eifc.subnet_mask = (wValue << 16) | wIndex;
		ret = 0;
		break;
	case MOT_USB_SET_HOST_IP:
		pr_info("%s: set host ip request\n", __func__);
		wIndex = le16_to_cpu(req->wIndex);
		wValue = le16_to_cpu(req->wValue);
		eifc.router_ip = (wValue << 16) | wIndex;
		ret = 0;
		break;
	case MOT_USB_SET_TIME:
		pr_info("%s: set time not supported\n", __func__);
	default:
		/* will return stall */
		return ret;
	}

	if(eifc.ip_addr && eifc.subnet_mask && eifc.router_ip) {
		/* schedule a work queue to do this because we need to be able to sleep */
		eifc.ether_config_dev = ctxt->dev;
		schedule_work(&eifc.ether_config_wq);
	}

       return ret;
}

static void ether_if_config(struct work_struct *work)
{
	struct ifreq ifr;
	mm_segment_t saved_fs;
	unsigned err;
	struct sockaddr_in *sin;

//	memzero(&ifr, sizeof(ifr));
	memset(&ifr, 0, sizeof(ifr));
	sin = (void *)&(ifr.ifr_ifru.ifru_addr);
	strncpy(ifr.ifr_ifrn.ifrn_name, "usb0", strlen("usb0") + 1);
	sin->sin_family = AF_INET;

	sin->sin_addr.s_addr = eifc.ip_addr;
	saved_fs = get_fs();
	set_fs(get_ds());
	err = devinet_ioctl(dev_net(eifc.ether_config_dev), SIOCSIFADDR, &ifr);
	if(err)
		pr_info("%s: Error in SIOCSIFADDR\n", __FUNCTION__);

	sin->sin_addr.s_addr = eifc.subnet_mask;
	err = devinet_ioctl(dev_net(eifc.ether_config_dev), SIOCSIFNETMASK, &ifr);
	if(err)
		pr_info("%s: Error in SIOCSIFNETMASK\n", __FUNCTION__);

	sin->sin_addr.s_addr = eifc.ip_addr | ~(eifc.subnet_mask);
	err = devinet_ioctl(dev_net(eifc.ether_config_dev), SIOCSIFBRDADDR, &ifr);	
	if(err)
		pr_info("%s: Error in SIOCSIFBRDADDR\n", __FUNCTION__);

//	memzero(&ifr, sizeof(ifr));
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_ifrn.ifrn_name, "usb0", strlen("usb0") + 1);	
	ifr.ifr_flags = ((eifc.ether_config_dev->flags) | IFF_UP);
	err = devinet_ioctl(dev_net(eifc.ether_config_dev), SIOCSIFFLAGS, &ifr);
	if(err)
		pr_info("%s: Error in SIOCSIFFLAGS\n", __FUNCTION__);
	set_fs(saved_fs);

	pr_info("%s: usb0 interface config complete\n", __FUNCTION__);
}

static void ether_unbind(void *_ctxt)
{
	struct ether_context *ctxt = _ctxt;
	unsigned long flags;

	if (!ctxt)
		return;

	if(!ether_bound)
		return;

	if(ctxt->in) {
		usb_ept_fifo_flush(ctxt->in);
		usb_ept_enable(ctxt->in, 0);
		usb_free_endpoint(ctxt->in);
	}
	if(ctxt->out) {
		usb_ept_fifo_flush(ctxt->out);
		usb_ept_enable(ctxt->out, 0);
		usb_free_endpoint(ctxt->out);
	}
	if(ctxt->int_out) {
		usb_ept_fifo_flush(ctxt->int_out);
		usb_ept_enable(ctxt->int_out, 0);
		usb_free_endpoint(ctxt->int_out);
	}

	ether_bound = 0;
}

static void ether_bind(void *_ctxt)
#else
static void ether_bind(struct usb_endpoint **ept, void *_ctxt)
#endif
{
	struct ether_context *ctxt = _ctxt;
	struct usb_request *req;
	unsigned long flags;
	int n;

#if defined(CONFIG_KERNEL_MOTOROLA) || defined(CONFIG_MACH_MOT)
	intf_desc.bInterfaceNumber = usb_msm_get_next_ifc_number(&usb_func_ether);

	intf_desc.iInterface = usb_msm_get_next_strdesc_id(ETHER_INTERFACE_NAME);

	ctxt->in = usb_alloc_endpoint(USB_DIR_IN);
	if(ctxt->in) {
		hs_bulk_in_desc.bEndpointAddress = USB_DIR_IN | ctxt->in->num;
		fs_bulk_in_desc.bEndpointAddress = USB_DIR_IN | ctxt->in->num;
	}
	ctxt->out = usb_alloc_endpoint(USB_DIR_OUT);
	if(ctxt->out) {
		hs_bulk_out_desc.bEndpointAddress = USB_DIR_OUT | ctxt->out->num;
		fs_bulk_out_desc.bEndpointAddress = USB_DIR_OUT | ctxt->out->num;
	}
	ctxt->int_out = usb_alloc_endpoint(USB_DIR_OUT);
	if(ctxt->int_out) {
		hs_int_out_desc.bEndpointAddress = USB_DIR_OUT | ctxt->int_out->num;
		fs_int_out_desc.bEndpointAddress = USB_DIR_OUT | ctxt->int_out->num;
	}
#else
	ctxt->out = ept[0];
	ctxt->in = ept[1];
#endif

	for (n = 0; n < MAX_RX; n++) {
		req = usb_ept_alloc_req(ctxt->out, 0);
		if (!req)
			break;
		req->complete = ether_out_complete;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->rx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}
	for (n = 0; n < MAX_TX; n++) {
		req = usb_ept_alloc_req(ctxt->in, 0);
		if (!req)
			break;
		req->complete = ether_in_complete;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->tx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}
#if defined(CONFIG_KERNEL_MOTOROLA) || defined(CONFIG_MACH_MOT)
	for (n = 0; n < MAX_RX; n++) {
		req = usb_ept_alloc_req(ctxt->int_out, 0);
		if (!req)
			break;
		req->complete = ether_int_dummy;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->rx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}
#endif
    ether_bound = 1;
}

static void ether_in_complete(struct usb_endpoint *ept,
			      struct usb_request *req)
{
	unsigned long flags;
	struct sk_buff *skb = req->context;
	struct ether_context *ctxt = *((void **) skb->cb);

	if (req->status == 0) {
		ctxt->stats.tx_packets++;
		ctxt->stats.tx_bytes += req->actual;
	} else {
		ctxt->stats.tx_errors++;
	}

	dev_kfree_skb_any(skb);

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(&ctxt->tx_reqs))
		netif_start_queue(ctxt->dev);
	list_add_tail(&req->list, &ctxt->tx_reqs);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

static void ether_out_complete(struct usb_endpoint *ept,
			       struct usb_request *req)
{
	struct sk_buff *skb = req->context;
	struct ether_context *ctxt = *((void **) skb->cb);

	if (req->status == 0) {
		skb_put(skb, req->actual);
		skb->protocol = eth_type_trans(skb, ctxt->dev);
		ctxt->stats.rx_packets++;
		ctxt->stats.rx_bytes += req->actual;
		netif_rx(skb);
	} else {
		dev_kfree_skb_any(skb);
		ctxt->stats.rx_errors++;
	}

	/* don't bother requeuing if we just went offline */
	if (req->status == -ENODEV) {
		unsigned long flags;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->rx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	} else {
		if (ether_queue_out(ctxt, req))
			pr_err("ether_out: cannot requeue\n");
	}
}

static int ether_queue_out(struct ether_context *ctxt,
			   struct usb_request *req)
{
	unsigned long flags;
	struct sk_buff *skb;
	int ret;

	skb = alloc_skb(USB_MTU + NET_IP_ALIGN, GFP_ATOMIC);
	if (!skb) {
		pr_err("ether_queue_out: failed to alloc skb\n");
		ret = -ENOMEM;
		goto fail;
	}

	skb_reserve(skb, NET_IP_ALIGN);

	*((void **) skb->cb) = ctxt;
	req->buf = skb->data;
	req->length = USB_MTU;
	req->context = skb;

	ret = usb_ept_queue_xfer(ctxt->out, req);
	if (ret) {
fail:
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->rx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}

	return ret;
}

static void ether_configure(int configured, void *_ctxt)
{
	unsigned long flags;
	struct ether_context *ctxt = _ctxt;
	struct usb_request *req;

#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
	if(!ctxt)
		return;
#endif

	pr_info("ether_configure() %d\n", configured);

	if (configured) {
#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
		if(usb_msm_get_speed() == USB_SPEED_HIGH) {
			usb_configure_endpoint(ctxt->in, &hs_bulk_in_desc);
			usb_configure_endpoint(ctxt->out, &hs_bulk_out_desc);
			usb_configure_endpoint(ctxt->int_out, &hs_int_out_desc);
		} else {
			usb_configure_endpoint(ctxt->in, &fs_bulk_in_desc);
			usb_configure_endpoint(ctxt->out, &fs_bulk_out_desc);
			usb_configure_endpoint(ctxt->int_out, &fs_int_out_desc);
		}
		usb_ept_enable(ctxt->in, 1);
		usb_ept_enable(ctxt->out, 1);
		usb_ept_enable(ctxt->int_out, 1);
#endif
		/* we're online -- get all rx requests queued */
		for (;;) {
			spin_lock_irqsave(&ctxt->lock, flags);
			if (list_empty(&ctxt->rx_reqs)) {
				req = 0;
			} else {
				req = list_first_entry(&ctxt->rx_reqs,
						       struct usb_request,
						       list);
				list_del(&req->list);
			}
			spin_unlock_irqrestore(&ctxt->lock, flags);
			if (!req)
				break;
			if (ether_queue_out(ctxt, req))
				break;
		}
	} else {
		/* all pending requests will be canceled */
#if defined(CONFIG_MACH_CALGARY) || defined(CONFIG_MACH_MOT)
		eifc.ip_addr = 0x0;
		eifc.subnet_mask = 0x0;
		eifc.router_ip = 0x0;
#endif
	}
}

static struct usb_function usb_func_ether = {
	.bind = ether_bind,
	.unbind = ether_unbind,
	.configure = ether_configure,
#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
	.setup = ether_req_setup,

	.name = ETHER_FUNCTION_NAME,

	.disabled = 1,
#else
	.name = "ether",

	.ifc_class = 0x02,
	.ifc_subclass = 0x0a,
	.ifc_protocol = 0x00,

	.ifc_name = "ether",

	.ifc_ept_count = 2,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN },
#endif
};

#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
struct usb_descriptor_header *ether_hs_descriptors[5];
struct usb_descriptor_header *ether_fs_descriptors[5];
#endif

static int usb_ether_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ether_context *ctxt = netdev_priv(dev);
	struct usb_request *req;
	unsigned long flags;
	unsigned len;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(&ctxt->tx_reqs)) {
		req = 0;
	} else {
		req = list_first_entry(&ctxt->tx_reqs,
				       struct usb_request, list);
		list_del(&req->list);
		if (list_empty(&ctxt->tx_reqs))
			netif_stop_queue(dev);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);

	if (!req) {
		pr_err("usb_ether_xmit: could not obtain tx request\n");
		return 1;
	}

	/* ensure that we end with a short packet */
	len = skb->len;
	if (!(len & 63) || !(len & 511))
		len++;

	*((void **) skb->cb) = ctxt;
	req->context = skb;
	req->buf = skb->data;
	req->length = len;

	if (usb_ept_queue_xfer(ctxt->in, req)) {
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->tx_reqs);
		netif_start_queue(dev);
		spin_unlock_irqrestore(&ctxt->lock, flags);

		dev_kfree_skb_any(skb);
		ctxt->stats.tx_dropped++;

		pr_err("usb_ether_xmit: could not queue tx request\n");
	}

	return 0;
}

static int usb_ether_open(struct net_device *dev)
{
	return 0;
}

static int usb_ether_stop(struct net_device *dev)
{
	return 0;
}

static struct net_device_stats *usb_ether_get_stats(struct net_device *dev)
{
	struct ether_context *ctxt = netdev_priv(dev);
	return &ctxt->stats;
}

static void __init usb_ether_setup(struct net_device *dev)
{
	struct ether_context *ctxt = netdev_priv(dev);

	pr_info("usb_ether_setup()\n");

	INIT_LIST_HEAD(&ctxt->rx_reqs);
	INIT_LIST_HEAD(&ctxt->tx_reqs);
	spin_lock_init(&ctxt->lock);
	ctxt->dev = dev;

	dev->open = usb_ether_open;
	dev->stop = usb_ether_stop;
	dev->hard_start_xmit = usb_ether_xmit;
	dev->get_stats = usb_ether_get_stats;
	dev->watchdog_timeo = 20;

	ether_setup(dev);

	random_ether_addr(dev->dev_addr);
}

#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
static int blan_enable_open(struct inode *ip, struct file *fp)
{
	struct ether_context *ctxt = usb_func_ether.context;

	if(_lock(&ctxt->enable_excl))
		return -EBUSY;

	printk(KERN_INFO "enabling blan function\n");
	usb_function_enable(ETHER_FUNCTION_NAME, 1);

	return 0;
}

static int blan_enable_release(struct inode *ip, struct file *fp)
{
	struct ether_context *ctxt = usb_func_ether.context;

	printk(KERN_INFO "disabling blan function\n");
	usb_function_enable(ETHER_FUNCTION_NAME, 0);
	_unlock(&ctxt->enable_excl);
	return 0;
}

static struct file_operations blan_enable_fops = {
	.owner = THIS_MODULE,
	.open = blan_enable_open,
	.release = blan_enable_release,
};

static struct miscdevice blan_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "blan_enable",
	.fops = &blan_enable_fops,
};
#endif

static int __init ether_init(void)
{
	struct net_device *dev;
	int ret;

	dev = alloc_netdev(sizeof(struct ether_context),
			   "usb%d", usb_ether_setup);
	if (!dev)
		return -ENOMEM;

	ret = register_netdev(dev);
	if (ret) {
		free_netdev(dev);
	} else {
		struct ether_context *ctxt = netdev_priv(dev);
		usb_func_ether.context = ctxt;
#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
		ether_hs_descriptors[0] = (struct usb_descriptor_header *)&intf_desc;
		ether_hs_descriptors[1] = (struct usb_descriptor_header *)&hs_bulk_in_desc;
		ether_hs_descriptors[2] = (struct usb_descriptor_header *)&hs_bulk_out_desc;
		ether_hs_descriptors[3] = (struct usb_descriptor_header *)&hs_int_out_desc;
		ether_hs_descriptors[4] = NULL;
		ether_fs_descriptors[0] = (struct usb_descriptor_header *)&intf_desc;
		ether_fs_descriptors[1] = (struct usb_descriptor_header *)&fs_bulk_in_desc;
		ether_fs_descriptors[2] = (struct usb_descriptor_header *)&fs_bulk_out_desc;
		ether_fs_descriptors[3] = (struct usb_descriptor_header *)&fs_int_out_desc;
		ether_fs_descriptors[4] = NULL;
		usb_func_ether.hs_descriptors = ether_hs_descriptors;
		usb_func_ether.fs_descriptors = ether_fs_descriptors;
#endif
		usb_function_register(&usb_func_ether);
#if defined(CONFIG_MACH_PITTSBURGH) || defined(CONFIG_MACH_MOT)
		INIT_WORK(&eifc.ether_config_wq, ether_if_config);
	}
	
	ret = misc_register(&blan_enable_device);
	if (ret) {
		printk(KERN_ERR "blan Can't register misc enable device %d \n",
				MISC_DYNAMIC_MINOR);
		free_netdev(dev);
	}
#endif
	return ret;
}

module_init(ether_init);

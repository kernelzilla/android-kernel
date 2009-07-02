/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define WDR_TIMEOUT       (HZ * 5)
#define MODEM_INTERFACE_NUM 0

#define BP_MODEM_STATUS 0x20a1
#define BP_RSP_AVAIL 0x01a1
#define BP_SPEED_CHANGE 0x2aa1

#define BP_CAR 0x01
#define BP_DSR 0x02
#define BP_BREAK 0x04
#define BP_RNG 0x08

#define BULKOUT_SIZE 1280

#define AP_NW  16
#define AP_NR  16

#define USB_IPC_SUSPEND_DELAY	5000	/*  delay in msec */


struct ap_rb {
	struct list_head list;
	int size;
	unsigned char *base;
	dma_addr_t dma;
};

struct ap_ru {
	struct list_head list;
	struct ap_rb *buffer;
	struct urb *urb;
	struct modem_port *instance;
};

struct modem_port {
	__u16 modem_status;	/* only used for data modem port */
	struct ap_ru ru[AP_NR];
	struct ap_rb rb[AP_NR];
	int rx_buflimit;
	int rx_endpoint;
	unsigned int susp_count;
	struct tasklet_struct urb_task;
	struct usb_serial_port *port;
	spinlock_t read_lock;
	spinlock_t write_lock;
	unsigned int readsize;
	struct list_head spare_read_urbs;
	struct list_head spare_read_bufs;
	struct list_head filled_read_bufs;
	int processing;
	int sending;
	struct delayed_work pm_put_work;
	struct work_struct wake_and_write;
	atomic_t write_count;
	atomic_t read_count;
	atomic_t intf_count;
};

static struct usb_device_id id_table[] = {
	{USB_DEVICE(0x22b8, 0x2a6e)}, 	/* Sholes CDMA BP modem */
	{USB_DEVICE(0x22b8, 0x2a6f)}, 	/* Sholes CDMA BP modem */
	{},
};

MODULE_DEVICE_TABLE(usb, id_table);

static uint32_t cdma_modem_debug;
module_param_named(cdma_mdm_debug, cdma_modem_debug, uint, 0664);

static void modem_read_buffers_free(
		struct modem_port *modem_ptr,
		struct usb_serial *serial)
{
	struct usb_device *usb_dev = serial->dev;
	int i;
	int n = modem_ptr->rx_buflimit;

	for (i = 0; i < n; i++)
		usb_buffer_free(usb_dev, modem_ptr->readsize,
				modem_ptr->rb[i].base,
				modem_ptr->rb[i].dma);
}

static void modem_read_bulk_callback(struct urb *urb)
{
	struct ap_rb *buf;
	struct ap_ru *rcv = urb->context;
	struct modem_port *modem_port_ptr;
	int status = urb->status;
	unsigned long flags;

	modem_port_ptr = rcv->instance;
	if (modem_port_ptr == NULL)
		return;

	buf = rcv->buffer;

	if (unlikely(status != 0)) {
		if (cdma_modem_debug)
			dev_info(&modem_port_ptr->port->dev,
			"%s: handle urb status none zero case  \n",
			__func__);

		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		list_add_tail(&rcv->list, &modem_port_ptr->spare_read_urbs);
		list_add(&buf->list, &modem_port_ptr->spare_read_bufs);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

		return;
	}

	atomic_set(&modem_port_ptr->read_count, 1);

	buf->size = urb->actual_length;

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	modem_port_ptr->processing++;
	list_add_tail(&rcv->list, &modem_port_ptr->spare_read_urbs);
	list_add_tail(&buf->list, &modem_port_ptr->filled_read_bufs);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

	if (likely(!modem_port_ptr->susp_count))
		tasklet_schedule(&modem_port_ptr->urb_task);
	return;
}

static int modem_open(struct tty_struct *tty,
		struct usb_serial_port *port,
		struct file *filp)
{
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(port->serial);
	int retval = 0;
	int i;
	unsigned long flags;

	if (cdma_modem_debug)
		dev_info(&port->dev, "%s: Enter. Open Port %d\n",
				 __func__, port->number);

	/*
	 * Force low_latency on
	 */
	if (tty)
		tty->low_latency = 1;

	/* clear the throttle flags */
	spin_lock_irqsave(&port->lock, flags);
	port->throttled = 0;
	port->throttle_req = 0;
	spin_unlock_irqrestore(&port->lock, flags);

	if (modem_port_ptr == NULL) {
		dev_err(&port->dev,
			 "%s: null modem port pointer. \n",
			 __func__);
		retval = -1;
		goto exit;
	}

	modem_port_ptr->port = port;
	atomic_set(&modem_port_ptr->write_count, 0);
	atomic_set(&modem_port_ptr->read_count, 0);


	INIT_LIST_HEAD(&modem_port_ptr->spare_read_urbs);
	INIT_LIST_HEAD(&modem_port_ptr->spare_read_bufs);
	INIT_LIST_HEAD(&modem_port_ptr->filled_read_bufs);

	for (i = 0; i < modem_port_ptr->rx_buflimit; i++) {
		list_add(&(modem_port_ptr->ru[i].list),
			 &modem_port_ptr->spare_read_urbs);
	}

	for (i = 0; i < modem_port_ptr->rx_buflimit; i++) {
		list_add(&(modem_port_ptr->rb[i].list),
			 &modem_port_ptr->spare_read_bufs);
	}

	if (modem_port_ptr->susp_count == 0) {
		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		tasklet_schedule(&modem_port_ptr->urb_task);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
	} else {
		if (cdma_modem_debug)
			dev_info(&port->dev,
			 "%s: susp_count not 0 at open time? \n",
				__func__);
	}

	atomic_set(&modem_port_ptr->intf_count, 1);

exit:
	schedule_delayed_work(&modem_port_ptr->pm_put_work,
	      msecs_to_jiffies(USB_IPC_SUSPEND_DELAY));


	if (cdma_modem_debug)
		dev_info(&port->dev, "%s: Exit. retval = %d\n",
			 __func__, retval);

	return retval;
}

static void modem_rx_tasklet(unsigned long _modem_port)
{
	struct modem_port *modem_port_ptr = (void *)_modem_port;
	struct ap_rb *buf;
	struct tty_struct *tty;
	struct usb_serial_port *port;
	struct ap_ru *rcv;
	unsigned long flags;
	unsigned char throttled;

	if (modem_port_ptr) {
		port = modem_port_ptr->port;
		if (port == NULL)
			return;
		tty = port->port.tty;
	} else {
		return;
	}

	spin_lock_irqsave(&modem_port_ptr->port->lock, flags);
	throttled = modem_port_ptr->port->throttle_req;
	spin_unlock_irqrestore(&modem_port_ptr->port->lock, flags);
	if (throttled) {
		dev_err(&port->dev,
			"%s: modem_rx_tasklet: throttled. \n",
			 __func__);
		return;
	}

next_buffer:
	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	if (list_empty(&modem_port_ptr->filled_read_bufs)) {
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
		goto urbs;
	}
	buf = list_entry(modem_port_ptr->filled_read_bufs.next,
			 struct ap_rb, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

	tty_buffer_request_room(tty, buf->size);
	spin_lock_irqsave(&modem_port_ptr->port->lock, flags);
	throttled = modem_port_ptr->port->throttle_req;
	spin_unlock_irqrestore(&modem_port_ptr->port->lock, flags);
	if (!throttled)
		tty_insert_flip_string(tty, buf->base, buf->size);
	tty_flip_buffer_push(tty);

	if (throttled) {
		dev_err(&port->dev, "%s: Throttling noticed.\n", __func__);
		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		list_add(&buf->list, &modem_port_ptr->filled_read_bufs);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
		return;
	}

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	list_add(&buf->list, &modem_port_ptr->spare_read_bufs);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
	goto next_buffer;

urbs:
	while (!list_empty(&modem_port_ptr->spare_read_bufs)) {
		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		if (list_empty(&modem_port_ptr->spare_read_urbs)) {
			modem_port_ptr->processing = 0;
			spin_unlock_irqrestore(&modem_port_ptr->read_lock,
					       flags);
			if (cdma_modem_debug)
				dev_info(&port->dev,
					 "%s: no urb to create. \n", __func__);
			return;
		}
		rcv = list_entry(modem_port_ptr->spare_read_urbs.next,
				 struct ap_ru, list);
		list_del(&rcv->list);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

		buf = list_entry(modem_port_ptr->spare_read_bufs.next,
				 struct ap_rb, list);
		list_del(&buf->list);

		rcv->buffer = buf;

		usb_fill_bulk_urb(rcv->urb, modem_port_ptr->port->serial->dev,
				  modem_port_ptr->rx_endpoint,
				  buf->base,
				  modem_port_ptr->readsize,
				  modem_read_bulk_callback, rcv);
		rcv->urb->transfer_dma = buf->dma;
		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		if (usb_submit_urb(rcv->urb, GFP_ATOMIC) < 0) {
			list_add(&buf->list, &modem_port_ptr->spare_read_bufs);
			list_add(&rcv->list, &modem_port_ptr->spare_read_urbs);
			modem_port_ptr->processing = 0;
			dev_err(&port->dev, "%s: submit bulk in  urb failed.\n",
				 __func__);
			spin_unlock_irqrestore(modem_port_ptr->read_lock,
					       flags);
			return;
		} else {
			spin_unlock_irqrestore(&modem_port_ptr->read_lock,
					       flags);
		}
	}
	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	modem_port_ptr->processing = 0;
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

}

static void modem_close(struct tty_struct *tty,
				  struct usb_serial_port *port,
				  struct file *filp)
{
	struct modem_port *modem_port_ptr;
	int i, nr;

	if (cdma_modem_debug)
		dev_info(&port->dev, "%s: Enter. Close Port %d  \n",
			 __func__, port->number);

	modem_port_ptr = usb_get_serial_data(port->serial);
	if (!modem_port_ptr) {
		dev_err(&port->dev,
			 "%s: null modem port pointer. \n",
			 __func__);
		return;
	}

	if (modem_port_ptr->susp_count)
		usb_autopm_get_interface(port->serial->interface);

	modem_port_ptr->modem_status = 0;

	nr = modem_port_ptr->rx_buflimit;
	for (i = 0; i < nr; i++)
		usb_kill_urb(modem_port_ptr->ru[i].urb);

	usb_kill_urb(port->write_urb);

	cancel_delayed_work_sync(&modem_port_ptr->pm_put_work);

	modem_port_ptr->port = 0;

	if (cdma_modem_debug)
		dev_info(&port->dev, "%s: Exit. \n", __func__);
}

static int modem_submit_write_urb(struct usb_serial_port *port)
{
	struct modem_port *modem_port_ptr =
	usb_get_serial_data(port->serial);
	int result;
	unsigned long flags;

	spin_lock_irqsave(&modem_port_ptr->write_lock, flags);
	modem_port_ptr->sending++;
	spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags);
	atomic_set(&modem_port_ptr->write_count, 1);
	result = usb_submit_urb(port->write_urb, GFP_ATOMIC);
	if (result) {
		dev_err(&port->dev,
			"%s: Submit bulk out URB failed. ret = %d \n",
			__func__, result);
		spin_lock_irqsave(&port->lock, flags);
		port->write_urb_busy = 0;
		spin_unlock_irqrestore(&port->lock, flags);
		spin_lock_irqsave(&modem_port_ptr->write_lock, flags);
		modem_port_ptr->sending--;
		spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags);
	}

	return result;
}


static void modem_wake_and_write(struct work_struct *work)
{
	struct modem_port *modem_port_ptr =
				 container_of(work,
					   struct
					   modem_port,
					   wake_and_write);
	int result;
	struct usb_serial *serial = modem_port_ptr->port->serial;
	struct usb_serial_port *port =  modem_port_ptr->port;
	unsigned long flags;

	result = usb_autopm_get_interface(serial->interface);
	if (result < 0) {
		dev_err(&port->dev, "%s: autopm failed. result = %d \n",
			__func__, result);
		spin_lock_irqsave(&port->lock, flags);
		port->write_urb_busy = 0;
		spin_unlock_irqrestore(&port->lock, flags);
	} else {
		result = modem_submit_write_urb(port);
	}
}


static void modem_pm_put_worker(struct work_struct *work)
{

	struct modem_port *modem_port_ptr =
				 container_of(work,
					   struct
					   modem_port,
					   pm_put_work.
					   work);
	int put_interface = 1;

	if (modem_port_ptr->port != NULL) {
		struct usb_serial *serial = modem_port_ptr->port->serial;
		if (atomic_read(&modem_port_ptr->write_count) == 1) {
			atomic_set(&modem_port_ptr->write_count, 0);
			put_interface = 0;
		}

		if (atomic_read(&modem_port_ptr->read_count) == 1) {
			atomic_set(&modem_port_ptr->read_count, 0);
			put_interface = 0;
		}

		if ((put_interface == 1) &&
			(atomic_read(&modem_port_ptr->intf_count) == 1)) {
			usb_autopm_put_interface(serial->interface);
			atomic_set(&modem_port_ptr->intf_count, 0);
			if (cdma_modem_debug)
				dev_info(&serial->dev->dev,
				"%s: put interface count \n",
				__func__);
		}
	}

	schedule_delayed_work(&modem_port_ptr->pm_put_work,
		msecs_to_jiffies(USB_IPC_SUSPEND_DELAY));

}

static void modem_write_bulk_callback(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	int status = urb->status;
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(port->serial);
	unsigned long flags;

	port->write_urb_busy = 0;

	spin_lock_irqsave(&modem_port_ptr->write_lock, flags);
	modem_port_ptr->sending--;
	spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags);
	if (status) {
		dev_err(&port->dev, "%s: status non-zero. status = %d \n",
			 __func__, status);
		return;
	}
	usb_serial_port_softint(port);

	return;
}

static int modem_write(struct tty_struct *tty,
				 struct usb_serial_port *port,
				 const unsigned char *buf, int count)
{
	struct usb_serial *serial = port->serial;
	int result;
	unsigned char *data;
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(port->serial);


	if (count == 0) {
		if (cdma_modem_debug)
			dev_info(&port->dev, "%s: Exit1: %s count = 0  \n",
				 __func__, dev_name(&port->dev));
		return 0;
	}

	if (serial->num_bulk_out) {
		unsigned long flags;
		spin_lock_irqsave(&port->lock, flags);
		if (port->write_urb_busy) {
			spin_unlock_irqrestore(&port->lock, flags);
			if (cdma_modem_debug)
				dev_info(&port->dev,
					 "%s: already writing. \n",
					 __func__);
			return 0;
		}
		port->write_urb_busy = 1;
		spin_unlock_irqrestore(&port->lock, flags);

		count = (count > port->bulk_out_size) ?
		    port->bulk_out_size : count;

		memcpy(port->write_urb->transfer_buffer, buf, count);
		data = port->write_urb->transfer_buffer;

		/* set up our urb */
		usb_fill_bulk_urb(port->write_urb, serial->dev,
				  usb_sndbulkpipe(serial->dev,
						  port->
						  bulk_out_endpointAddress),
				  port->write_urb->transfer_buffer, count,
				  modem_write_bulk_callback, port);

		/* start sending */
		if (modem_port_ptr->susp_count) {
			schedule_work(&modem_port_ptr->wake_and_write);
			result = 0;
		} else {
			result = modem_submit_write_urb(port);
		}
		if (result >= 0)
			result = count;
		return result;
	}

	/* no bulk out, so return 0 bytes written */
	return 0;
}

#ifdef CONFIG_PM
static int modem_suspend(struct usb_interface *intf,
				   pm_message_t message)
{
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(serial);
	struct usb_serial_port *port;
	unsigned long flags1, flags2;
	int ret = 0, tmp, nr, i;

	if (modem_port_ptr == NULL) {
		dev_err(&intf->dev, " NULL modem_port ptr \n");
		goto out;
	}

	port = modem_port_ptr->port;

	if (port == NULL) {
		if (cdma_modem_debug)
			dev_info(&intf->dev,
				"%s: this port is not open yet.\n",
				 __func__);
		modem_port_ptr->susp_count = 1;
		goto out;
	} else {
		if (cdma_modem_debug)
			dev_info(&intf->dev, "%s: Suspend Port  num %d.\n",
				 __func__, port->number);
	}

	if (modem_port_ptr->susp_count == 1) {
		if (cdma_modem_debug)
			dev_info(&intf->dev,
				 "%s: port %d is already suspend.\n",
				 __func__, port->number);
		goto out;
	}

	spin_lock_irqsave(&modem_port_ptr->write_lock, flags1);
	spin_lock_irqsave(&modem_port_ptr->read_lock, flags2);
	tmp = modem_port_ptr->processing + modem_port_ptr->sending;
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags2);
	spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags1);
	if (tmp) {
		if (cdma_modem_debug)
			dev_info(&intf->dev,
				 "%s:  sending = %d, receiving = %d.\n",
				 __func__, modem_port_ptr->sending,
				 modem_port_ptr->sending);
		ret = -EBUSY;
		goto out;
	}

	spin_lock_irqsave(&modem_port_ptr->write_lock, flags1);
	spin_lock_irqsave(&modem_port_ptr->read_lock, flags2);
	tmp = modem_port_ptr->susp_count = 1;
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags2);
	spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags1);

	if (cdma_modem_debug)
		dev_info(&intf->dev, "%s:  port %d is suspended.\n",
			 __func__, port->number);

	nr = modem_port_ptr->rx_buflimit;
	for (i = 0; i < nr; i++)
		usb_kill_urb(modem_port_ptr->ru[i].urb);

	usb_kill_urb(port->write_urb);

	/* clean up the read buf list */
	INIT_LIST_HEAD(&modem_port_ptr->spare_read_urbs);
	INIT_LIST_HEAD(&modem_port_ptr->spare_read_bufs);
	INIT_LIST_HEAD(&modem_port_ptr->filled_read_bufs);

	for (i = 0; i < modem_port_ptr->rx_buflimit; i++) {
		list_add(&(modem_port_ptr->ru[i].list),
			 &modem_port_ptr->spare_read_urbs);
	}

	for (i = 0; i < modem_port_ptr->rx_buflimit; i++) {
		list_add(&(modem_port_ptr->rb[i].list),
			 &modem_port_ptr->spare_read_bufs);
	}

out:
	if (cdma_modem_debug)
		dev_info(&intf->dev,
			 "%s: Exit. ret = %d\n", __func__, ret);
	return ret;
}

static int modem_resume(struct usb_interface *intf)
{
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(serial);
	struct usb_serial_port *port;
	unsigned long flags1, flags2, flags;
	int ret = 0;

	if (modem_port_ptr == NULL) {
		dev_err(&intf->dev, "%s: null modem port pointer. \n",
			 __func__);
		goto out;
	}

	port = modem_port_ptr->port;

	if (port == NULL) {
		if (cdma_modem_debug)
			dev_info(&intf->dev,
				 "%s: port not open yet \n",
				 __func__);

		spin_lock_irqsave(&modem_port_ptr->write_lock, flags1);
		spin_lock_irqsave(&modem_port_ptr->read_lock, flags2);
		modem_port_ptr->susp_count = 0;
		spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags1);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags2);
		goto out;
	}

	if (modem_port_ptr->susp_count == 0) {
		if (cdma_modem_debug)
			dev_info(&intf->dev,
				 "%s: port # %d is already resumed. \n",
				 __func__, port->number);
		goto out;
	} else {
		spin_lock_irqsave(&modem_port_ptr->write_lock, flags1);
		spin_lock_irqsave(&modem_port_ptr->read_lock, flags2);
		modem_port_ptr->susp_count = 0;
		spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags1);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags2);
		if (cdma_modem_debug)
			dev_info(&intf->dev, "%s: port %d is resumed here \n",
				 __func__, port->number);
	}

	atomic_set(&modem_port_ptr->intf_count, 1);

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	tasklet_schedule(&modem_port_ptr->urb_task);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
out:
	return ret;
}
#endif /* CONFIG_PM */

static int modem_startup(struct usb_serial *serial)
{
	struct usb_serial_port *port = serial->port[0];
	struct modem_port *modem_port_ptr = NULL;
	struct usb_interface *interface;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_host_interface *iface_desc;
	int readsize;
	int num_rx_buf;
	int i;

	interface = serial->interface;
	iface_desc = interface->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		epread = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_in(epread))
			break;
		else
			epread = NULL;
	}

	if (epread == NULL) {
		dev_err(&serial->dev->dev,
			 "%s: No Bulk In Endpoint for this Interface \n",
			 __func__);
		return -EPERM;
	}

	num_rx_buf = AP_NR;
	readsize = le16_to_cpu(epread->wMaxPacketSize) * 2;

	/* setup a buffer to store interface data */
	modem_port_ptr =
	    kzalloc(sizeof(struct modem_port), GFP_KERNEL);
	if (modem_port_ptr == NULL) {
		dev_err(&serial->dev->dev,
			 "%s: error -- no memory on start up. \n",
			 __func__);
		return -ENOMEM;
	}

	/* init tasklet for rx processing */
	tasklet_init(&modem_port_ptr->urb_task, modem_rx_tasklet,
		     (unsigned long)modem_port_ptr);
	modem_port_ptr->rx_buflimit = num_rx_buf;
	modem_port_ptr->rx_endpoint =
	    usb_rcvbulkpipe(serial->dev, port->bulk_in_endpointAddress);
	spin_lock_init(&modem_port_ptr->read_lock);
	spin_lock_init(&modem_port_ptr->write_lock);
	modem_port_ptr->susp_count = 0;
	modem_port_ptr->port = 0;
	modem_port_ptr->readsize = readsize;

	INIT_DELAYED_WORK(&modem_port_ptr->pm_put_work,
			  modem_pm_put_worker);
	INIT_WORK(&modem_port_ptr->wake_and_write, modem_wake_and_write);

	/* allocate multiple receive urb pool */
	for (i = 0; i < num_rx_buf; i++) {
		struct ap_ru *rcv = &(modem_port_ptr->ru[i]);

		rcv->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (rcv->urb == NULL) {
			dev_err(&serial->dev->dev,
				"%s: out of memory \n", __func__);
			goto alloc_mot_acm_fail;
		}

		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		rcv->instance = modem_port_ptr;
	}

	/* allocate multiple receive buffer */
	for (i = 0; i < num_rx_buf; i++) {
		struct ap_rb *rb = &(modem_port_ptr->rb[i]);

		rb->base = usb_buffer_alloc(serial->dev, readsize,
					    GFP_KERNEL, &rb->dma);
		if (!rb->base) {
			dev_err(&serial->dev->dev,
				 "%s : out of memory \n",
				__func__);
			goto alloc_mot_acm_fail;
		}
	}

	modem_port_ptr->modem_status = 0;

	/* install serial private data */
	usb_set_serial_data(serial, modem_port_ptr);

	return 0;
alloc_mot_acm_fail:
	modem_read_buffers_free(modem_port_ptr, serial);
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(modem_port_ptr->ru[i].urb);
	if (modem_port_ptr != NULL) {
		kfree(modem_port_ptr);
		usb_set_serial_data(serial, NULL);
	}

	return -ENOMEM;
}

static void modem_shutdown(struct usb_serial *serial)
{
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(serial);

	uint8_t interface_num =
		serial->interface->cur_altsetting->desc.bInterfaceNumber;

	if (cdma_modem_debug)
		dev_info(&serial->dev->dev,
			 "%s: Shutdown Interface %d  \n", __func__,
			interface_num);

	modem_read_buffers_free(modem_port_ptr, serial);

	if (modem_port_ptr) {
		/* free private structure allocated for serial device */
		kfree(modem_port_ptr);
		usb_set_serial_data(serial, NULL);
	}
}

static struct usb_driver modem_driver = {
	.name = "cdma-modem",
	.probe = usb_serial_probe,
	.disconnect = usb_serial_disconnect,
	.id_table = id_table,
	.no_dynamic_id = 1,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
	.suspend = modem_suspend,
	.resume = modem_resume,
#endif
};

static struct usb_serial_driver modem_device = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "qsc6085-modem",
		   },
	.description = "QSC 6085 Modem Driver",
	.usb_driver = &modem_driver,
	.id_table = id_table,
	.num_ports = 1,
	.write = modem_write,
	.write_bulk_callback = modem_write_bulk_callback,
	.open = modem_open,
	.close = modem_close,
	.attach = modem_startup,
	.shutdown = modem_shutdown,
};

static void __exit modem_exit(void)
{
	usb_deregister(&modem_driver);
	usb_serial_deregister(&modem_device);
}

static int __init modem_init(void)
{
	int retval;

	retval = usb_serial_register(&modem_device);
	if (retval)
		return retval;
	retval = usb_register(&modem_driver);
	if (retval)
		usb_serial_deregister(&modem_device);
	return retval;
}

module_init(modem_init);
module_exit(modem_exit);

MODULE_DESCRIPTION("USB IPC Driver for QSC 6085");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

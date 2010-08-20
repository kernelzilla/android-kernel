/****************************************************************

Siano Mobile Silicon, Inc.
MDTV receiver kernel modules.
Copyright (C) 2006-2008, Uri Shkolnik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/
/*!
	\file	spibusdrv.c

	\brief	spi bus driver module

	This file contains implementation of the spi bus driver.
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>
#include <linux/time.h>

#include "smscoreapi.h"
#include "smsdbg_prn.h"
#include "smsspicommon.h"
#include "smsspiphy.h"

#define ANDROID_2_6_25
#ifdef ANDROID_2_6_25
#include <linux/workqueue.h>
#endif


#define SMS_INTR_PIN			16  /* 0 for nova sip, 26 for vega */
#define TX_BUFFER_SIZE			0x200
#define RX_BUFFER_SIZE			(0x1000 + SPI_PACKET_SIZE + 0x100)
#define NUM_RX_BUFFERS			72

struct _spi_device_st {
	struct _spi_dev dev;
	void *phy_dev;

	struct completion write_operation;
	struct list_head tx_queue;
	int allocatedPackets;
	int padding_allowed;
	char *rxbuf;
	dma_addr_t rxbuf_phy_addr;

	struct smscore_device_t *coredev;
	struct list_head txqueue;
	char *txbuf;
	dma_addr_t txbuf_phy_addr;

#if defined(MOT_FEAT_OMAP_DMA_USE)
  int dma_tx_channel;
  int dma_rx_channel;
#endif
};

struct _smsspi_txmsg {
	struct list_head node;	/*! internal management */
	void *buffer;
	size_t size;
	int alignment;
	int add_preamble;
	struct completion completion;
	void (*prewrite) (void *);
	void (*postwrite) (void *);
};

struct _spi_device_st *spi_dev;

static void spi_worker_thread(void *arg);
static DECLARE_WORK(spi_work_queue, (void *)spi_worker_thread);
static u8 smsspi_preamble[] = { 0xa5, 0x5a, 0xe7, 0x7e };
static u8 smsspi_startup[] = { 0, 0, 0xde, 0xc1, 0xa5, 0x51, 0xf1, 0xed };
static u32 default_type = SMS_NOVA_B0; /*SMS_NOVA_A0;*/
static u32 intr_pin = SMS_INTR_PIN;

module_param(default_type, int, 0644);
MODULE_PARM_DESC(default_type, "default board type.");

module_param(intr_pin, int, 0644);
MODULE_PARM_DESC(intr_pin, "interrupt pin number.");

/******************************************/
static void spi_worker_thread(void *arg)
{
	struct _spi_device_st *spi_device = spi_dev;
	struct _smsspi_txmsg *msg = NULL;
	struct _spi_msg txmsg;

	/*PDEBUG("worker start\n");*/
	do {
		/* do we have a msg to write ? */
		if (!msg && !list_empty(&spi_device->txqueue))
			msg = (struct _smsspi_txmsg *)
					list_entry(spi_device->txqueue.
					next, struct _smsspi_txmsg, node);

		if (msg) {
			if (msg->add_preamble) {
				txmsg.len =
				    min(msg->size + sizeof(smsspi_preamble),
					(size_t) TX_BUFFER_SIZE);
				txmsg.buf = spi_device->txbuf;
				txmsg.buf_phy_addr = spi_device->txbuf_phy_addr;
				memcpy(txmsg.buf, smsspi_preamble,
				       sizeof(smsspi_preamble));
				memcpy(&txmsg.buf[sizeof(smsspi_preamble)],
				       msg->buffer,
				       txmsg.len - sizeof(smsspi_preamble));
				msg->add_preamble = 0;
				msg->buffer +=
				    txmsg.len - sizeof(smsspi_preamble);
				msg->size -=
				    txmsg.len - sizeof(smsspi_preamble);
				/* zero out the rest of aligned buffer */
				memset(&txmsg.buf[txmsg.len], 0,
				       TX_BUFFER_SIZE - txmsg.len);
				smsspi_common_transfer_msg(&spi_device->dev,
							   &txmsg, 1);
			} else {
				txmsg.len =
				    min(msg->size, (size_t) TX_BUFFER_SIZE);
				txmsg.buf = spi_device->txbuf;
				txmsg.buf_phy_addr = spi_device->txbuf_phy_addr;
				memcpy(txmsg.buf, msg->buffer, txmsg.len);

				msg->buffer += txmsg.len;
				msg->size -= txmsg.len;
				/* zero out the rest of aligned buffer */
				memset(&txmsg.buf[txmsg.len], 0,
				       TX_BUFFER_SIZE - txmsg.len);
				smsspi_common_transfer_msg(&spi_device->dev,
							   &txmsg, 0);
			}

		} else {
			smsspi_common_transfer_msg(&spi_device->dev, NULL, 1);
		}

		/* if there was write, have we finished ? */
		if (msg && !msg->size) {
			/* call postwrite call back */
			if (msg->postwrite)
				msg->postwrite(spi_device);

			list_del(&msg->node);
			complete(&msg->completion);
			msg = NULL;
		}
		/* if there was read, did we read anything ? */

	} while (!list_empty(&spi_device->txqueue) || msg);

	/*PDEBUG("worker end\n");*/

}

static void msg_found(void *context, void *buf, int offset, int len)
{
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	struct smscore_buffer_t *cb =
	    (struct smscore_buffer_t
	     *)(container_of(buf, struct smscore_buffer_t, p));

	/*PDEBUG("entering\n");*/
	cb->offset = offset;
	cb->size = len;
	/* PERROR ("buffer %p is sent back to core databuf=%p,
		offset=%d.\n", cb, cb->p, cb->offset); */
	smscore_onresponse(spi_device->coredev, cb);

	/*PDEBUG("exiting\n");*/

}

static void smsspi_int_handler(void *context)
{
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;

	PREPARE_WORK(&spi_work_queue, (void *)spi_worker_thread);
	spi_device->padding_allowed = 1;
	schedule_work(&spi_work_queue);
}

static int smsspi_queue_message_and_wait(struct _spi_device_st *spi_device,
					 struct _smsspi_txmsg *msg)
{
	init_completion(&msg->completion);
	list_add_tail(&msg->node, &spi_device->txqueue);
	schedule_work(&spi_work_queue);
	wait_for_completion(&msg->completion);

	return 0;
}

/*##w21558*/
#ifdef __SMS_DUMMY_INT__
static int smsspi_SetIntLine(void *context)
{
	struct _Msg {
		struct SmsMsgHdr_ST hdr;
		u32 data[3];
	} Msg = {
		{
		MSG_SMS_SPI_INT_LINE_SET_REQ, 0, HIF_TASK,
			    sizeof(struct _Msg), 0}, {
		0, intr_pin, 100}
	};
	struct _smsspi_txmsg msg;

	PDEBUG("Sending SPI Set Interrupt command sequence\n");
/*	fwDnlComplete(spi_device->phy_dev, 0);*/
	msg.buffer = &Msg;
	msg.size = sizeof(Msg);
	msg.alignment = SPI_PACKET_SIZE;
	msg.add_preamble = 1;
	msg.prewrite = NULL;
	msg.postwrite = NULL;	/* smsspiphy_restore_clock; */

	return smsspi_queue_message_and_wait(context, &msg);
/*	return 0;*/
}
#endif

static int smsspi_preload(void *context)
{
	struct _smsspi_txmsg msg;
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;

	prepareForFWDnl(spi_device->phy_dev);
	PDEBUG("Sending SPI init sequence\n");
	msg.buffer = smsspi_startup;
	msg.size = sizeof(smsspi_startup);
	msg.alignment = 4;
	msg.add_preamble = 0;
	msg.prewrite = NULL;	/* smsspiphy_reduce_clock; */
	msg.postwrite = NULL;

	smsspi_queue_message_and_wait(context, &msg);
	/*smsspi_SetIntLine(context);*/

	return 0;
}

/*#if 0
//ROCK 0331
//Move INT_LINE_SET_REQ to the preload, enable the INT before FW download
//Add MSG_SMS_SET_PERIODIC_STATS_REQ, enable the Period Statistics*/
static int smsspi_postload(void *context)
{
	struct _Msg {
		struct SmsMsgHdr_ST hdr;
		u32 data[3];
	} Msg = {
		{
		MSG_SMS_SPI_INT_LINE_SET_REQ, 0, HIF_TASK,
			    sizeof(struct _Msg), 0}, {
		0, intr_pin, 0}
	};
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	struct _smsspi_txmsg msg;

	PDEBUG("Sending SPI Set Interrupt command sequence\n");
	fwDnlComplete(spi_device->phy_dev, 0);
	msg.buffer = &Msg;
	msg.size = sizeof(Msg);
	msg.alignment = SPI_PACKET_SIZE;
	msg.add_preamble = 1;
	msg.prewrite = NULL;
	msg.postwrite = NULL;	/* smsspiphy_restore_clock; */

	return smsspi_queue_message_and_wait(context, &msg);
}
/*#endif*/


static int smsspi_write(void *context, void *txbuf, size_t len)
{
	struct _smsspi_txmsg msg;

	msg.buffer = txbuf;
	msg.size = len;
	msg.prewrite = NULL;
	msg.postwrite = NULL;

	if (len > 0x1000) {
		/* The FW is the only long message. Do not add preamble,
		and do not padd it */
		msg.alignment = 4;
		msg.add_preamble = 0;
		msg.prewrite = smschipreset;
	} else {
		msg.alignment = SPI_PACKET_SIZE;
		msg.add_preamble = 1;
	}

/*
	PDEBUG("Writing message to  SPI.\n");
	PDEBUG("msg hdr: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x.\n",
	       ((u8 *) txbuf)[0], ((u8 *) txbuf)[1], ((u8 *) txbuf)[2],
	       ((u8 *) txbuf)[3], ((u8 *) txbuf)[4], ((u8 *) txbuf)[5],
	       ((u8 *) txbuf)[6], ((u8 *) txbuf)[7]);
	PDEBUG("0x%x, 0x%x,\n",    ((u8 *) txbuf)[0], ((u8 *) txbuf)[1]);
*/

	return smsspi_queue_message_and_wait(context, &msg);
}

struct _rx_buffer_st *allocate_rx_buf(void *context, int size)
{
	struct smscore_buffer_t *buf;
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	if (size > RX_BUFFER_SIZE) {
		PERROR("Requested size is bigger than max buffer size.\n");
		return NULL;
	}
	buf = smscore_getbuffer(spi_device->coredev);
/*	PDEBUG("Recieved Rx buf %p physical 0x%x (contained in %p)\n", buf->p,
		buf->phys, buf);*/

	/* note: this is not mistake! the rx_buffer_st is identical to part of
	   smscore_buffer_t and we return the address of the start of the
	   identical part */
	return (struct _rx_buffer_st *) &buf->p;
}

static void free_rx_buf(void *context, struct _rx_buffer_st *buf)
{
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	struct smscore_buffer_t *cb =
	    (struct smscore_buffer_t
	     *)(container_of(((void *)buf), struct smscore_buffer_t, p));
/*	PDEBUG("buffer %p is released.\n", cb);*/
	smscore_putbuffer(spi_device->coredev, cb);
}

/*! Release device STUB

\param[in]	dev:		device control block
\return		void
*/
static void smsspi_release(struct device *dev)
{
	PDEBUG("nothing to do\n");
	/* Nothing to release */
}

static struct platform_device smsspi_device = {
	.name = "smsspi",
	.id = 1,
	.dev = {
		.release = smsspi_release,
		},
};

int smsspi_register(void)
{
	struct smsdevice_params_t params;
	int ret;
	struct _spi_device_st *spi_device;
	struct _spi_dev_cb_st common_cb;

	PDEBUG("entering \n");

	spi_device =
	    kmalloc(sizeof(struct _spi_device_st), GFP_KERNEL);
	spi_dev = spi_device;

	INIT_LIST_HEAD(&spi_device->txqueue);

	ret = platform_device_register(&smsspi_device);
	if (ret < 0) {
		PERROR("platform_device_register failed\n");
		return ret;
	}

/* w21558 */
#if defined(MOT_FEAT_OMAP_DMA_USE)
	spi_device->txbuf =
	    dma_alloc_coherent(NULL, TX_BUFFER_SIZE,
			       &spi_device->txbuf_phy_addr,
			       GFP_KERNEL | GFP_DMA);
	if (!spi_device->txbuf) {
		printk(KERN_INFO "%s dma_alloc_coherent(...) failed\n",
		       __func__);
		ret = -ENOMEM;
		goto txbuf_error;
	}
#endif

	spi_device->phy_dev =
	    smsspiphy_init(NULL, smsspi_int_handler, &spi_device);
	if (spi_device->phy_dev == 0) {
		printk(KERN_INFO "%s smsspiphy_init(...) failed\n", __func__);
		goto phy_error;
	}

	common_cb.allocate_rx_buf = allocate_rx_buf;
	common_cb.free_rx_buf = free_rx_buf;
	common_cb.msg_found_cb = msg_found;
	common_cb.transfer_data_cb = smsspibus_xfer;

	ret =
	    smsspicommon_init(&spi_device->dev, spi_device, spi_device->phy_dev,
			      &common_cb);
	if (ret) {
		printk(KERN_INFO "%s smsspiphy_init(...) failed\n", __func__);
		goto common_error;
	}

	/* register in smscore */
	memset(&params, 0, sizeof(params));
	params.context = spi_device;
	params.device = &smsspi_device.dev;
	params.buffer_size = RX_BUFFER_SIZE;
	params.num_buffers = NUM_RX_BUFFERS;
	params.flags = SMS_DEVICE_NOT_READY;
	params.sendrequest_handler = smsspi_write;
	strcpy(params.devpath, "spi");
	params.device_type = default_type;

	if (0) {
		/* device family */
		/* params.setmode_handler = smsspi_setmode; */
	} else {
		params.flags =
		    SMS_DEVICE_FAMILY2 | SMS_DEVICE_NOT_READY |
		    SMS_ROM_NO_RESPONSE;
		params.preload_handler = smsspi_preload;
		params.postload_handler = smsspi_postload;
	}

	ret = smscore_register_device(&params, &spi_device->coredev);
	if (ret < 0) {
		printk(KERN_INFO "%s smscore_register_device(...) failed\n",
		       __func__);
		goto reg_device_error;
	}

	ret = smscore_start_device(spi_device->coredev);
	if (ret < 0) {
		printk(KERN_INFO "%s smscore_start_device(...) failed\n",
		       __func__);
		goto start_device_error;
	}

	PDEBUG("exiting\n");
	return 0;

start_device_error:
	smscore_unregister_device(spi_device->coredev);

reg_device_error:

common_error:
	smsspiphy_deinit(spi_device->phy_dev);

phy_error:
	dma_free_coherent(NULL, TX_BUFFER_SIZE, spi_device->txbuf,
			  spi_device->txbuf_phy_addr);

txbuf_error:
	platform_device_unregister(&smsspi_device);

	PDEBUG("exiting error %d\n", ret);

	return ret;
}

void smsspi_unregister(void)
{
	struct _spi_device_st *spi_device = spi_dev;
	PDEBUG("entering\n");

	/* stop interrupts */
	smsspiphy_deinit(spi_device->phy_dev);
	smscore_unregister_device(spi_device->coredev);

	dma_free_coherent(NULL, TX_BUFFER_SIZE, spi_device->txbuf,
			spi_device->txbuf_phy_addr);

	platform_device_unregister(&smsspi_device);
	PDEBUG("exiting\n");
}

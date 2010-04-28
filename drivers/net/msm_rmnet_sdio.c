/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*
 * RMNET SDIO module.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>

#include <mach/sdio_al.h>

#define SDIO_CH_LOCAL_OPEN       0x1
#define SDIO_CH_REMOTE_OPEN      0x2

#define SDIO_MUX_HDR_MAGIC_NO    0x33fc

#define SDIO_MUX_HDR_CMD_DATA    0
#define SDIO_MUX_HDR_CMD_OPEN    1
#define SDIO_MUX_HDR_CMD_CLOSE   2

struct sdio_ch_info {
	uint32_t status;
	void (*receive_cb)(void *, struct sk_buff *);
	void (*write_done)(void *, struct sk_buff *);
	void *priv;
	struct mutex lock;
	struct sk_buff *skb;
};

static struct sdio_channel *sdio_mux_ch;
static struct sdio_ch_info sdio_ch[8];
struct wake_lock sdio_mux_ch_wakelock;

struct sdio_mux_hdr {
	uint16_t magic_num;
	uint8_t reserved;
	uint8_t cmd;
	uint8_t pad_len;
	uint8_t ch_id;
	uint16_t pkt_len;
};

struct sdio_partial_pkt_info {
	uint32_t valid;
	struct sk_buff *skb;
	struct sdio_mux_hdr *hdr;
};

static void sdio_mux_read_data(struct work_struct *work);
static void sdio_mux_write_data(struct work_struct *work);

static DEFINE_MUTEX(sdio_mux_lock);
static DECLARE_WORK(work_sdio_mux_read, sdio_mux_read_data);
static DECLARE_WORK(work_sdio_mux_write, sdio_mux_write_data);

static struct workqueue_struct *sdio_mux_workqueue;
static struct sdio_partial_pkt_info sdio_partial_pkt;

#define sdio_ch_is_open(x)						\
	(sdio_ch[(x)].status == (SDIO_CH_LOCAL_OPEN | SDIO_CH_REMOTE_OPEN))

#define sdio_ch_is_local_open(x)			\
	(sdio_ch[(x)].status & SDIO_CH_LOCAL_OPEN)

static inline void skb_set_data(struct sk_buff *skb,
				unsigned char *data,
				unsigned int len)
{
	/* panic if tail > end */
	skb->data = data;
	skb->tail = skb->data + len;
	skb->len  = len;
}

static void sdio_mux_save_partial_pkt(struct sdio_mux_hdr *hdr,
				      struct sk_buff *skb_mux)
{
	struct sk_buff *skb;

	/* i think we can avoid cloning here */
	skb =  skb_clone(skb_mux, GFP_KERNEL);
	if (!skb) {
		pr_err("rmnet_recv() cannot clone skb\n");
		return;
	}

	/* protect? */
	skb_set_data(skb, (unsigned char *)hdr,
		     skb->tail - (unsigned char *)hdr);
	sdio_partial_pkt.skb = skb;
	sdio_partial_pkt.valid = 1;
	return;
}

static void *handle_sdio_mux_data(struct sdio_mux_hdr *hdr,
				  struct sk_buff *skb_mux)
{
	struct sk_buff *skb;
	void *rp = (void *)hdr;

	/* protect? */
	rp += sizeof(*hdr);
	pr_info("%s: hdr %p rp %p tail %p\n", __func__, hdr, rp, skb_mux->tail);
	if (rp < (void *)skb_mux->tail)
		rp += (hdr->pkt_len + hdr->pad_len);

	pr_info("%s: hdr %p rp %p tail %p pkt_size %d\n",
		__func__, hdr, rp, skb_mux->tail, hdr->pkt_len + hdr->pad_len);

	if (rp > (void *)skb_mux->tail) {
		/* partial packet */
		sdio_mux_save_partial_pkt(hdr, skb_mux);
		goto packet_done;
	}

	skb =  skb_clone(skb_mux, GFP_KERNEL);
	if (!skb) {
		pr_err("rmnet_recv() cannot clone skb\n");
		goto packet_done;
	}


	pr_info("%s: head %p data %p tail %p end %p len %d\n",
		__func__, skb->head, skb->data, skb->tail, skb->end, skb->len);
	skb_set_data(skb, (unsigned char *)(hdr + 1), hdr->pkt_len);
	pr_info("%s: head %p data %p tail %p end %p len %d\n",
		__func__, skb->head, skb->data, skb->tail, skb->end, skb->len);

	/* probably we should check channel status */
	/* discard packet early if local side not open */
	mutex_lock(&sdio_ch[hdr->ch_id].lock);
	if (sdio_ch[hdr->ch_id].receive_cb)
		sdio_ch[hdr->ch_id].receive_cb(sdio_ch[hdr->ch_id].priv, skb);
	else
		dev_kfree_skb_any(skb);
	mutex_unlock(&sdio_ch[hdr->ch_id].lock);

packet_done:
	return rp;
}

static void *handle_sdio_mux_command(struct sdio_mux_hdr *hdr,
				     struct sk_buff *skb_mux)
{
	void *rp;

	pr_info("%s: cmd %d\n", __func__, hdr->cmd);
	switch (hdr->cmd) {
	case SDIO_MUX_HDR_CMD_DATA:
		rp = handle_sdio_mux_data(hdr, skb_mux);
		break;
	case SDIO_MUX_HDR_CMD_OPEN:
		mutex_lock(&sdio_ch[hdr->ch_id].lock);
		sdio_ch[hdr->ch_id].status |= SDIO_CH_REMOTE_OPEN;
		mutex_unlock(&sdio_ch[hdr->ch_id].lock);
		rp = hdr + 1;
		break;
	case SDIO_MUX_HDR_CMD_CLOSE:
		/* probably should drop pending write */
		mutex_lock(&sdio_ch[hdr->ch_id].lock);
		sdio_ch[hdr->ch_id].status &= ~SDIO_CH_REMOTE_OPEN;
		mutex_unlock(&sdio_ch[hdr->ch_id].lock);
		rp = hdr + 1;
		break;
	default:
		rp = hdr + 1;
	}

	return rp;
}

static void *handle_sdio_partial_pkt(struct sk_buff *skb_mux)
{
	struct sk_buff *p_skb;
	struct sdio_mux_hdr *p_hdr;
	void *ptr, *rp = skb_mux->data;

	/* protoect? */
	if (sdio_partial_pkt.valid) {
		p_skb = sdio_partial_pkt.skb;

		ptr = skb_push(skb_mux, p_skb->len);
		memcpy(ptr, p_skb->data, p_skb->len);
		sdio_partial_pkt.skb = NULL;
		sdio_partial_pkt.valid = 0;
		dev_kfree_skb_any(p_skb);

		p_hdr = (struct sdio_mux_hdr *)skb_mux->data;
		rp = handle_sdio_mux_command(p_hdr, skb_mux);
	}
	return rp;
}

static void sdio_mux_read_data(struct work_struct *work)
{
	struct sk_buff *skb_mux;
	void *ptr = 0;
	int sz, rc, len = 0;
	struct sdio_mux_hdr *hdr;

	pr_info("%s: reading\n", __func__);
	/* should probably have a separate read lock */
	mutex_lock(&sdio_mux_lock);
	sz = sdio_read_avail(sdio_mux_ch);
	pr_info("%s: read avail %d\n", __func__, sz);
	if (sz <= 0) {
		pr_info("rmnet_recv() no or invalid data\n");
		mutex_unlock(&sdio_mux_lock);
		return;
	}

	/* net_ip_aling is probably not required */
	if (sdio_partial_pkt.valid)
		len = sdio_partial_pkt.skb->len;
	skb_mux = dev_alloc_skb(sz + NET_IP_ALIGN + len);
	if (skb_mux == NULL) {
		pr_err("rmnet_recv() cannot allocate skb\n");
		mutex_unlock(&sdio_mux_lock);
		return;
	}

	skb_reserve(skb_mux, NET_IP_ALIGN);
	ptr = skb_put(skb_mux, sz);

	/* half second wakelock is fine? */
	wake_lock_timeout(&sdio_mux_ch_wakelock, HZ / 2);
	rc = sdio_read(sdio_mux_ch, ptr, sz);
	pr_info("%s: read %d\n", __func__, rc);
	if (rc) {
		pr_err("%s sdio read failed %d\n", __func__, rc);
		dev_kfree_skb_any(skb_mux);
		mutex_unlock(&sdio_mux_lock);
		queue_work(sdio_mux_workqueue, &work_sdio_mux_read);
		return;
	}
	mutex_unlock(&sdio_mux_lock);

	/* move to a separate function */
	/* probably do skb_pull instead of pointer adjustment */
	hdr = handle_sdio_partial_pkt(skb_mux);
	while ((void *)hdr < (void *)skb_mux->tail) {

		if (((void *)hdr + sizeof(*hdr)) > (void *)skb_mux->tail) {
			/* handle partial header */
			sdio_mux_save_partial_pkt(hdr, skb_mux);
			break;
		}

		if (hdr->magic_num != SDIO_MUX_HDR_MAGIC_NO) {
			pr_err("rmnet_recv() packet error\n");
			break;
		}

		hdr = handle_sdio_mux_command(hdr, skb_mux);
	}
	dev_kfree_skb_any(skb_mux);

	pr_info("%s: read done\n", __func__);
	queue_work(sdio_mux_workqueue, &work_sdio_mux_read);
}

static int sdio_mux_write(struct sk_buff *skb)
{
	int rc, sz;

	mutex_lock(&sdio_mux_lock);
	sz = sdio_write_avail(sdio_mux_ch);
	pr_info("%s: avail %d len %d\n", __func__, sz, skb->len);
	if (skb->len <= sz) {
		pr_info("%s: before sdio_write \n", __func__);
		rc = sdio_write(sdio_mux_ch, skb->data, skb->len);
		pr_info("%s: sdio_write returned %d\n", __func__, rc);
		if (rc)
			rc = -EAGAIN;
	} else
		rc = -ENOMEM;

	mutex_unlock(&sdio_mux_lock);
	return rc;
}

static int sdio_mux_write_cmd(void *data, uint32_t len)
{
	int avail, rc;
	for (;;) {
		mutex_lock(&sdio_mux_lock);
		avail = sdio_write_avail(sdio_mux_ch);
		pr_info("%s: before sdio_mux_write_cmd(), avail =  %d\n",
			__func__, avail);
		if (avail >= len) {
			rc = sdio_write(sdio_mux_ch, data, len);
			pr_info("%s: sdio_write returned = %d\n", __func__, rc);
			if (!rc)
				break;
		}
		mutex_unlock(&sdio_mux_lock);
		msleep(250);
	}
	mutex_unlock(&sdio_mux_lock);
	return 0;
}

static void sdio_mux_write_data(struct work_struct *work)
{
	int i, rc, reschedule = 0;

	for (i = 0; i < 8; i++) {
		mutex_lock(&sdio_ch[i].lock);
		if (sdio_ch_is_local_open(i) && sdio_ch[i].skb) {
			rc = sdio_mux_write(sdio_ch[i].skb);
			pr_info("%s: write returned %d\n", __func__, rc);
			if (rc == -EAGAIN) {
				reschedule = 1;
			} else if (!rc) {
				sdio_ch[i].write_done(sdio_ch[i].priv,
						      sdio_ch[i].skb);
				sdio_ch[i].skb = NULL;
			}
		}
		mutex_unlock(&sdio_ch[i].lock);
	}

	/* probably should use delayed work */
	if (reschedule)
		queue_work(sdio_mux_workqueue, &work_sdio_mux_write);
}

int msm_rmnet_sdio_write(uint32_t id, struct sk_buff *skb)
{
	int rc = 0;
	struct sdio_mux_hdr *hdr;

	if (!skb)
		return -EINVAL;

	pr_info("%s: writing ch %d\n", __func__, id);
	/* mutex_lock(&sdio_ch[id].lock); */
	if (!sdio_ch_is_local_open(id)) {
		pr_err("%s: port not open: %d\n", __func__, sdio_ch[id].status);
		rc = -ENODEV;
		goto write_done;
	}

	if (sdio_ch[id].skb) {
		pr_err("%s: packet pending ch: %d\n", __func__, id);
		rc = -EPERM;
		goto write_done;
	}

	pr_info("%s: skb len without hdr %d\n", __func__, skb->len);

	hdr = (struct sdio_mux_hdr *)skb_push(skb, sizeof(struct sdio_mux_hdr));

	/* caller should allocate for hdr and padding
	   hdr is fine, padding is tricky */
	hdr->magic_num = SDIO_MUX_HDR_MAGIC_NO;
	hdr->cmd = SDIO_MUX_HDR_CMD_DATA;
	hdr->reserved = 0;
	hdr->ch_id = id;
	hdr->pkt_len = skb->len - sizeof(struct sdio_mux_hdr);
	if (skb->len & 0x3)
		skb_put(skb, 4 - (skb->len & 0x3));

	hdr->pad_len = skb->len - (sizeof(struct sdio_mux_hdr) + hdr->pkt_len);

	pr_info("%s: data %p, tail %p skb len %d pkt len %d pad len %d\n",
		__func__, skb->data, skb->tail, skb->len,
		hdr->pkt_len, hdr->pad_len);
	sdio_ch[id].skb = skb;
	queue_work(sdio_mux_workqueue, &work_sdio_mux_write);

write_done:
	/* mutex_unlock(&sdio_ch[id].lock); */
	return rc;
}

int msm_rmnet_sdio_open(uint32_t id, void *priv,
			void (*receive_cb)(void *, struct sk_buff *),
			void (*write_done)(void *, struct sk_buff *))
{
	struct sdio_mux_hdr hdr;

	pr_info("%s: opening ch %d\n", __func__, id);

	if (id >= 8)
		return -EINVAL;

	/* mutex_lock(&sdio_ch[id].lock); */
	if (sdio_ch_is_local_open(id)) {
		pr_info("%s: Already opened %d\n", __func__, id);
		goto open_done;
	}

	sdio_ch[id].receive_cb = receive_cb;
	sdio_ch[id].write_done = write_done;
	sdio_ch[id].priv = priv;

	hdr.magic_num = SDIO_MUX_HDR_MAGIC_NO;
	hdr.cmd = SDIO_MUX_HDR_CMD_OPEN;
	hdr.reserved = 0;
	hdr.ch_id = id;
	hdr.pkt_len = 0;
	hdr.pad_len = 0;

	pr_info("%s: before sdio_mux_write_cmd() %d\n", __func__, id);

	sdio_ch[id].status |= SDIO_CH_LOCAL_OPEN;
	sdio_mux_write_cmd((void *)&hdr, sizeof(hdr));

open_done:
	/* mutex_unlock(&sdio_ch[id].lock); */
	pr_info("%s: opened ch %d\n", __func__, id);
	return 0;
}

int msm_rmnet_sdio_close(uint32_t id)
{
	struct sdio_mux_hdr hdr;

	mutex_lock(&sdio_ch[id].lock);
	sdio_ch[id].receive_cb = NULL;
	sdio_ch[id].priv = NULL;

	if (sdio_ch[id].skb)
		dev_kfree_skb_any(sdio_ch[id].skb);

	sdio_ch[id].skb = NULL;

	hdr.magic_num = SDIO_MUX_HDR_MAGIC_NO;
	hdr.cmd = SDIO_MUX_HDR_CMD_CLOSE;
	hdr.reserved = 0;
	hdr.ch_id = id;
	hdr.pkt_len = 0;
	hdr.pad_len = 0;

	sdio_mux_write_cmd((void *)&hdr, sizeof(hdr));

	mutex_unlock(&sdio_ch[id].lock);

	return 0;
}

static void sdio_mux_notify(void *_dev, unsigned event)
{
	pr_info("%s notify called\n", __func__);

	/* write avail may not be enouogh for a packet, but should be fine */
	if ((event == SDIO_EVENT_DATA_WRITE_AVAIL) &&
	    sdio_write_avail(sdio_mux_ch))
		queue_work(sdio_mux_workqueue, &work_sdio_mux_write);

	if ((event == SDIO_EVENT_DATA_READ_AVAIL) &&
	    sdio_read_avail(sdio_mux_ch))
		queue_work(sdio_mux_workqueue, &work_sdio_mux_read);

}

static int msm_rmnet_sdio_probe(struct platform_device *pdev)
{
	int rc;
	static int sdio_mux_initialized;

	pr_info("%s probe called\n", __func__);

	if (sdio_mux_initialized)
		return 0;

	/* is one thread gud enough for read and write? */
	sdio_mux_workqueue = create_singlethread_workqueue("msm_rmnet_sdio");
	if (!sdio_mux_workqueue)
		return -ENOMEM;

	rc = sdio_open("SDIO_RMNET_DATA", &sdio_mux_ch, NULL, sdio_mux_notify);
	pr_info("%s: sido open result %d\n", __func__, rc);
	if (rc < 0) {
		destroy_workqueue(sdio_mux_workqueue);
		return rc;
	}

	wake_lock_init(&sdio_mux_ch_wakelock, WAKE_LOCK_SUSPEND,
		       "rmnet_sdio_mux");
	for (rc = 0; rc < 8; rc++)
		mutex_init(&sdio_ch[rc].lock);

	sdio_mux_initialized = 1;
	return 0;
}

static struct platform_driver msm_rmnet_sdio_driver = {
	.probe		= msm_rmnet_sdio_probe,
	.driver		= {
		.name	= "SDIO_RMNET_DATA",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_rmnet_sdio_init(void)
{
	return platform_driver_register(&msm_rmnet_sdio_driver);
}

module_init(msm_rmnet_sdio_init);
MODULE_DESCRIPTION("MSM RMNET SDIO MUX");
MODULE_LICENSE("GPL v2");

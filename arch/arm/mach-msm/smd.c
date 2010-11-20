/* arch/arm/mach-msm/smd.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/termios.h>
#include <linux/ctype.h>
#include <mach/msm_smd.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <linux/io.h>

#include "smd_private.h"
#include "proc_comm.h"
#include "modem_notifier.h"

#define MODULE_NAME "msm_smd"
#define SMEM_VERSION 0x000B
#define SMD_VERSION 0x00020000

enum {
	MSM_SMD_DEBUG = 1U << 0,
	MSM_SMSM_DEBUG = 1U << 1,
	MSM_SMD_INFO = 1U << 2,
	MSM_SMSM_INFO = 1U << 3,
};

enum {
	SMEM_APPS_Q6_SMSM = 3,
	SMEM_Q6_APPS_SMSM = 5,
	SMSM_NUM_INTR_MUX = 8,
};

/* Internal definitions which are not exported in some targets */
enum {
	SMSM_Q6_I = 2,
};

enum {
	SMSM_APPS_DEM_I = 3,
};

enum {
	SMD_APPS_QDSP_I = 1,
	SMD_MODEM_QDSP_I = 2
};

static int msm_smd_debug_mask;
module_param_named(debug_mask, msm_smd_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

void *smem_find(unsigned id, unsigned size);
void smd_diag(void);

static unsigned last_heap_free = 0xffffffff;

#if defined(CONFIG_MSM_SMD_DEBUG)
#define SMD_DBG(x...) do {				\
		if (msm_smd_debug_mask & MSM_SMD_DEBUG) \
			printk(KERN_DEBUG x);		\
	} while (0)

#define SMSM_DBG(x...) do {					\
		if (msm_smd_debug_mask & MSM_SMSM_DEBUG)	\
			printk(KERN_DEBUG x);			\
	} while (0)

#define SMD_INFO(x...) do {			 	\
		if (msm_smd_debug_mask & MSM_SMD_INFO)	\
			printk(KERN_INFO x);		\
	} while (0)

#define SMSM_INFO(x...) do {				\
		if (msm_smd_debug_mask & MSM_SMSM_INFO) \
			printk(KERN_INFO x);		\
	} while (0)
#else
#define SMD_DBG(x...) do { } while (0)
#define SMSM_DBG(x...) do { } while (0)
#define SMD_INFO(x...) do { } while (0)
#define SMSM_INFO(x...) do { } while (0)
#endif

void ram_console_scribble(const char* s, unsigned int count);

#if defined(CONFIG_ARCH_MSM7X30)
#define MSM_TRIG_A2M_INT(n) (writel(1 << n, MSM_GCC_BASE + 0x8))
#else
#define MSM_TRIG_A2M_INT(n) (writel(1, MSM_CSR_BASE + 0x400 + (n) * 4))
#endif

static void notify_other_smsm(uint32_t smsm_entry,
			      uint32_t old_val, uint32_t new_val)
{
	uint32_t *smsm_intr_mask;
	uint32_t *smsm_intr_mux;

	smsm_intr_mask = smem_alloc(SMEM_SMSM_CPU_INTR_MASK,
				    SMSM_NUM_ENTRIES * SMSM_NUM_HOSTS *
				    sizeof(uint32_t));

	/* older protocol don't use smsm_intr_mask,
	   but still communicates with modem */
	if (!smsm_intr_mask ||
	    (smsm_intr_mask[smsm_entry * SMSM_NUM_HOSTS + SMSM_MODEM] &
	    (old_val ^ new_val)))
		MSM_TRIG_A2M_INT(5);

	if (smsm_intr_mask &&
	    (smsm_intr_mask[smsm_entry * SMSM_NUM_HOSTS + SMSM_Q6_I] &
	    (old_val ^ new_val))) {
		smsm_intr_mux = smem_alloc(SMEM_SMD_SMSM_INTR_MUX,
					   SMSM_NUM_INTR_MUX *
					   sizeof(uint32_t));
		if (smsm_intr_mux)
			smsm_intr_mux[SMEM_APPS_Q6_SMSM]++;

		MSM_TRIG_A2M_INT(8);
	}
}

static inline void notify_other_smd(uint32_t ch_type)
{
	if (ch_type == SMD_APPS_MODEM)
		MSM_TRIG_A2M_INT(0);
	else if (ch_type == SMD_APPS_QDSP_I)
		MSM_TRIG_A2M_INT(8);
}

void smd_diag(void)
{
	char *x;
	int size;

	x = smem_find(ID_DIAG_ERR_MSG, SZ_DIAG_ERR_MSG);
	if (x != 0) {
		x[SZ_DIAG_ERR_MSG - 1] = 0;
		SMD_INFO("smem: DIAG '%s'\n", x);
#if defined(CONFIG_MACH_MOT) && !defined(CONFIG_MOT_PANIC)
	printk(KERN_EMERG "smem: DIAG '%s'\n", x);
#else
	}
	x = smem_get_entry(SMEM_ERR_CRASH_LOG, &size);
	if (x != 0) {
		x[size - 1] = 0;
		printk(KERN_ERR "smem: CRASH LOG\n'%s'\n", x);
	}
#endif /* CONFIG_MACH_MOT */
}

extern int (*msm_check_for_modem_crash)(void);

static int check_for_modem_crash(void)
{
	uint32_t *smsm;

	smsm = smem_find(ID_SHARED_STATE, SMSM_NUM_ENTRIES * sizeof(uint32_t));

	/* if the modem's not ready yet, we have to hope for the best */
	if (!smsm)
		return 0;

	if (smsm[SMSM_MODEM_STATE] & SMSM_RESET) {
		pr_err("proc_comm: ARM9 has crashed\n");
		smd_diag();
	} else {
		return 0;
	}

	/* hard reboot if possible FIXME
	if (msm_reset_hook)
		msm_reset_hook(0);
	*/

/* MOT - mtnf78 */
        printk(KERN_ERR "BUG: possible modem crash [Modem State=%x]!\n", smsm[SMSM_MODEM_STATE]);
	BUG();
/* MOT - mtnf78 */


	for (;;)
		;
}

#define SMD_SS_CLOSED            0x00000000
#define SMD_SS_OPENING           0x00000001
#define SMD_SS_OPENED            0x00000002
#define SMD_SS_FLUSHING          0x00000003
#define SMD_SS_CLOSING           0x00000004
#define SMD_SS_RESET             0x00000005
#define SMD_SS_RESET_OPENING     0x00000006

#define SMD_BUF_SIZE 8192
#define SMD_CHANNELS 64

#define SMD_HEADER_SIZE 20


/* the spinlock is used to synchronize between the
** irq handler and code that mutates the channel
** list or fiddles with channel state
*/
static DEFINE_SPINLOCK(smd_lock);
static DEFINE_SPINLOCK(smem_lock);

/* the mutex is used during open() and close()
** operations to avoid races while creating or
** destroying smd_channel structures
*/
static DEFINE_MUTEX(smd_creation_mutex);

static int smd_initialized;

/* 'type' field of smd_alloc_elm structure
 * has the following breakup
 * bits 0-7   -> channel type
 * bits 8-11  -> xfer type
 * bits 12-31 -> reserved
 */
struct smd_alloc_elm {
	char name[20];
	uint32_t cid;
	uint32_t type;
	uint32_t ref_count;
};

#define SMD_CHANNEL_TYPE(x) ((x) & 0x000000FF)
#define SMD_XFER_TYPE(x)    (((x) & 0x00000F00) >> 8)

struct smd_half_channel {
	unsigned state;
	unsigned char fDSR;
	unsigned char fCTS;
	unsigned char fCD;
	unsigned char fRI;
	unsigned char fHEAD;
	unsigned char fTAIL;
	unsigned char fSTATE;
	unsigned char fUNUSED;
	unsigned tail;
	unsigned head;
};

struct smd_channel {
	volatile struct smd_half_channel *send;
	volatile struct smd_half_channel *recv;
	unsigned char *send_buf;
	unsigned char *recv_buf;
	unsigned buf_size;
	struct list_head ch_list;

	unsigned current_packet;
	unsigned n;
	void *priv;
	void (*notify)(void *priv, unsigned flags);

	int (*read)(smd_channel_t *ch, void *data, int len);
	int (*write)(smd_channel_t *ch, const void *data, int len);
	int (*read_avail)(smd_channel_t *ch);
	int (*write_avail)(smd_channel_t *ch);
	int (*read_from_cb)(smd_channel_t *ch, void *data, int len);

	void (*update_state)(smd_channel_t *ch);
	unsigned last_state;

	char name[20];
	struct platform_device pdev;
	unsigned type;
};

static LIST_HEAD(smd_ch_closed_list);
static LIST_HEAD(smd_ch_list);

static unsigned char smd_ch_allocated[64];
static struct work_struct probe_work;

static void smd_alloc_channel(struct smd_alloc_elm *alloc_elm);
static void *_smem_find(unsigned id, unsigned *size);

static void smd_channel_probe_worker(struct work_struct *work)
{
	struct smd_alloc_elm *shared;
	unsigned n;

	shared = smem_find(ID_CH_ALLOC_TBL, sizeof(*shared) * 64);

	BUG_ON(!shared);

	for (n = 0; n < 64; n++) {
		if (smd_ch_allocated[n])
			continue;

		/* channel should be allocated only if APPS
		   processor is involved */
		if (SMD_CHANNEL_TYPE(shared[n].type) == SMD_MODEM_QDSP_I)
			continue;
		if (!shared[n].ref_count)
			continue;
		if (!shared[n].name[0])
			continue;

		smd_alloc_channel(&shared[n]);
		smd_ch_allocated[n] = 1;
	}
}

static char *chstate(unsigned n)
{
	switch (n) {
	case SMD_SS_CLOSED:        return "CLOSED";
	case SMD_SS_OPENING:       return "OPENING";
	case SMD_SS_OPENED:        return "OPENED";
	case SMD_SS_FLUSHING:      return "FLUSHING";
	case SMD_SS_CLOSING:       return "CLOSING";
	case SMD_SS_RESET:         return "RESET";
	case SMD_SS_RESET_OPENING: return "ROPENING";
	default:                   return "UNKNOWN";
	}
}

/* how many bytes are available for reading */
static int smd_stream_read_avail(struct smd_channel *ch)
{
	return (ch->recv->head - ch->recv->tail) & (ch->buf_size - 1);
}

/* how many bytes we are free to write */
static int smd_stream_write_avail(struct smd_channel *ch)
{
	return (ch->buf_size - 1) -
		((ch->send->head - ch->send->tail) & (ch->buf_size - 1));
}

static int smd_packet_read_avail(struct smd_channel *ch)
{
	if (ch->current_packet) {
		int n = smd_stream_read_avail(ch);
		if (n > ch->current_packet)
			n = ch->current_packet;
		return n;
	} else {
		return 0;
	}
}

static int smd_packet_write_avail(struct smd_channel *ch)
{
	int n = smd_stream_write_avail(ch);
	return n > SMD_HEADER_SIZE ? n - SMD_HEADER_SIZE : 0;
}

static int ch_is_open(struct smd_channel *ch)
{
	return (ch->recv->state == SMD_SS_OPENED ||
		ch->recv->state == SMD_SS_FLUSHING)
		&& (ch->send->state == SMD_SS_OPENED);
}

/* provide a pointer and length to readable data in the fifo */
static unsigned ch_read_buffer(struct smd_channel *ch, void **ptr)
{
	unsigned head = ch->recv->head;
	unsigned tail = ch->recv->tail;
	*ptr = (void *) (ch->recv_buf + tail);

	if (tail <= head)
		return head - tail;
	else
		return ch->buf_size - tail;
}

/* advance the fifo read pointer after data from ch_read_buffer is consumed */
static void ch_read_done(struct smd_channel *ch, unsigned count)
{
	BUG_ON(count > smd_stream_read_avail(ch));
	ch->recv->tail = (ch->recv->tail + count) & (ch->buf_size - 1);
	ch->send->fTAIL = 1;
}

/* basic read interface to ch_read_{buffer,done} used
** by smd_*_read() and update_packet_state()
** will read-and-discard if the _data pointer is null
*/
static int ch_read(struct smd_channel *ch, void *_data, int len)
{
	void *ptr;
	unsigned n;
	unsigned char *data = _data;
	int orig_len = len;

	while (len > 0) {
		n = ch_read_buffer(ch, &ptr);
		if (n == 0)
			break;

		if (n > len)
			n = len;
		if (_data)
			memcpy(data, ptr, n);

		data += n;
		len -= n;
		ch_read_done(ch, n);
	}

	return orig_len - len;
}

static void update_stream_state(struct smd_channel *ch)
{
	/* streams have no special state requiring updating */
}

static void update_packet_state(struct smd_channel *ch)
{
	unsigned hdr[5];
	int r;

	/* can't do anything if we're in the middle of a packet */
	while (ch->current_packet == 0) {
		/* discard 0 length packets if any */

		/* don't bother unless we can get the full header */
		if (smd_stream_read_avail(ch) < SMD_HEADER_SIZE)
			return;

		r = ch_read(ch, hdr, SMD_HEADER_SIZE);
		BUG_ON(r != SMD_HEADER_SIZE);

		ch->current_packet = hdr[0];
	}
}

/* provide a pointer and length to next free space in the fifo */
static unsigned ch_write_buffer(struct smd_channel *ch, void **ptr)
{
	unsigned head = ch->send->head;
	unsigned tail = ch->send->tail;
	*ptr = (void *) (ch->send_buf + head);

	if (head < tail) {
		return tail - head - 1;
	} else {
		if (tail == 0)
			return ch->buf_size - head - 1;
		else
			return ch->buf_size - head;
	}
}

/* advace the fifo write pointer after freespace from ch_write_buffer is filled */
static void ch_write_done(struct smd_channel *ch, unsigned count)
{
	BUG_ON(count > smd_stream_write_avail(ch));
	ch->send->head = (ch->send->head + count) & (ch->buf_size - 1);
	ch->send->fHEAD = 1;
}

static void ch_set_state(struct smd_channel *ch, unsigned n)
{
	if (n == SMD_SS_OPENED) {
		ch->send->fDSR = 1;
		ch->send->fCTS = 1;
		ch->send->fCD = 1;
	} else {
		ch->send->fDSR = 0;
		ch->send->fCTS = 0;
		ch->send->fCD = 0;
	}
	ch->send->state = n;
	ch->send->fSTATE = 1;
	notify_other_smd(ch->type);
}

static void do_smd_probe(void)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	if (shared->heap_info.free_offset != last_heap_free) {
		last_heap_free = shared->heap_info.free_offset;
		schedule_work(&probe_work);
	}
}

static void smd_state_change(struct smd_channel *ch,
			     unsigned last, unsigned next)
{
	ch->last_state = next;

	SMD_INFO("SMD: ch %d %s -> %s\n", ch->n,
		 chstate(last), chstate(next));

	switch (next) {
	case SMD_SS_OPENING:
		if (ch->send->state == SMD_SS_CLOSING ||
		    ch->send->state == SMD_SS_CLOSED) {
			ch->recv->tail = 0;
			ch->send->head = 0;
			ch_set_state(ch, SMD_SS_OPENING);
		}
		break;
	case SMD_SS_OPENED:
		if (ch->send->state == SMD_SS_OPENING) {
			ch_set_state(ch, SMD_SS_OPENED);
			ch->notify(ch->priv, SMD_EVENT_OPEN);
		}
		break;
	case SMD_SS_FLUSHING:
	case SMD_SS_RESET:
		/* we should force them to close? */
		break;
	case SMD_SS_CLOSED:
		if (ch->send->state == SMD_SS_OPENED) {
			ch_set_state(ch, SMD_SS_CLOSING);
			ch->notify(ch->priv, SMD_EVENT_CLOSE);
		}
		break;
	}
}

static irqreturn_t smd_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct smd_channel *ch;
	int do_notify_modem = 0;
	int do_notify_qdsp = 0;
	unsigned ch_flags;
	unsigned tmp;

	spin_lock_irqsave(&smd_lock, flags);
	list_for_each_entry(ch, &smd_ch_list, ch_list) {
		ch_flags = 0;
		if (ch_is_open(ch)) {
			if (ch->recv->fHEAD) {
				ch->recv->fHEAD = 0;
				ch_flags |= 1;
				if (ch->type == SMD_APPS_MODEM)
					do_notify_modem |= 1;
				else if (ch->type == SMD_APPS_QDSP_I)
					do_notify_qdsp |= 1;
			}
			if (ch->recv->fTAIL) {
				ch->recv->fTAIL = 0;
				ch_flags |= 2;
				if (ch->type == SMD_APPS_MODEM)
					do_notify_modem |= 1;
				else if (ch->type == SMD_APPS_QDSP_I)
					do_notify_qdsp |= 1;
			}
			if (ch->recv->fSTATE) {
				ch->recv->fSTATE = 0;
				ch_flags |= 4;
				if (ch->type == SMD_APPS_MODEM)
					do_notify_modem |= 1;
				else if (ch->type == SMD_APPS_QDSP_I)
					do_notify_qdsp |= 1;
			}
		}
		tmp = ch->recv->state;
		if (tmp != ch->last_state)
			smd_state_change(ch, ch->last_state, tmp);
		if (ch_flags) {
			ch->update_state(ch);
			ch->notify(ch->priv, SMD_EVENT_DATA);
		}
	}
	if (do_notify_modem)
		notify_other_smd(SMD_APPS_MODEM);

	if (do_notify_qdsp)
		notify_other_smd(SMD_APPS_QDSP_I);

	spin_unlock_irqrestore(&smd_lock, flags);
	do_smd_probe();
	return IRQ_HANDLED;
}

static void smd_fake_irq_handler(unsigned long arg)
{
	smd_irq_handler(0, NULL);
}

static DECLARE_TASKLET(smd_fake_irq_tasklet, smd_fake_irq_handler, 0);

void smd_sleep_exit(void)
{
	unsigned long flags;
	struct smd_channel *ch;
	unsigned tmp;
	int need_int = 0;

	spin_lock_irqsave(&smd_lock, flags);
	list_for_each_entry(ch, &smd_ch_list, ch_list) {
		if (ch_is_open(ch)) {
			if (ch->recv->fHEAD) {
				SMD_DBG("smd_sleep_exit ch %d fHEAD "
					"%x %x %x\n",
					ch->n,
					ch->recv->fHEAD,
					ch->recv->head, ch->recv->tail);
				need_int = 1;
				break;
			}
			if (ch->recv->fTAIL) {
				SMD_DBG("smd_sleep_exit ch %d fTAIL "
					"%x %x %x\n",
					ch->n,
					ch->recv->fTAIL,
					ch->send->head, ch->send->tail);
				need_int = 1;
				break;
			}
			if (ch->recv->fSTATE) {
				SMD_DBG("smd_sleep_exit ch %d fSTATE %x"
					"\n", ch->n,
					ch->recv->fSTATE);
				need_int = 1;
				break;
			}
			tmp = ch->recv->state;
			if (tmp != ch->last_state) {
				SMD_DBG("smd_sleep_exit ch %d "
					"state %x != %x\n",
					ch->n, tmp,
					ch->last_state);
				need_int = 1;
				break;
			}
		}
	}
	spin_unlock_irqrestore(&smd_lock, flags);
	do_smd_probe();
	if (need_int) {
		SMD_DBG("smd_sleep_exit need interrupt\n");
		tasklet_schedule(&smd_fake_irq_tasklet);
	}
}

static int smd_is_packet(struct smd_alloc_elm *alloc_elm)
{
	if (SMD_XFER_TYPE(alloc_elm->type) == 1)
		return 0;
	else if (SMD_XFER_TYPE(alloc_elm->type) == 2)
		return 1;

	/* for cases where xfer type is 0 */
	if (!strncmp(alloc_elm->name, "DAL", 3))
		return 0;

	if (alloc_elm->cid > 4 || alloc_elm->cid == 1)
		return 1;
	else
		return 0;
}

static int smd_stream_write(smd_channel_t *ch, const void *_data, int len)
{
	void *ptr;
	const unsigned char *buf = _data;
	unsigned xfer;
	int orig_len = len;

	SMD_DBG("smd_stream_write() %d -> ch%d\n", len, ch->n);
	if (len < 0)
		return -EINVAL;
	else if (len == 0)
		return 0;

	while ((xfer = ch_write_buffer(ch, &ptr)) != 0) {
		if (!ch_is_open(ch))
			break;
		if (xfer > len)
			xfer = len;
		memcpy(ptr, buf, xfer);
		ch_write_done(ch, xfer);
		len -= xfer;
		buf += xfer;
		if (len == 0)
			break;
	}

	if (orig_len - len)
		notify_other_smd(ch->type);

	return orig_len - len;
}

static int smd_packet_write(smd_channel_t *ch, const void *_data, int len)
{
	int ret;
	unsigned hdr[5];

	SMD_DBG("smd_packet_write() %d -> ch%d\n", len, ch->n);
	if (len < 0)
		return -EINVAL;
	else if (len == 0)
		return 0;

	if (smd_stream_write_avail(ch) < (len + SMD_HEADER_SIZE))
		return -ENOMEM;

	hdr[0] = len;
	hdr[1] = hdr[2] = hdr[3] = hdr[4] = 0;


	ret = smd_stream_write(ch, hdr, sizeof(hdr));
	if (ret < 0 || ret != sizeof(hdr)) {
		SMD_DBG("%s failed to write pkt header: "
			"%d returned\n", __func__, ret);
		return -1;
	}


	ret = smd_stream_write(ch, _data, len);
	if (ret < 0 || ret != len) {
		SMD_DBG("%s failed to write pkt data: "
			"%d returned\n", __func__, ret);
		return ret;
	}

	return len;
}

static int smd_stream_read(smd_channel_t *ch, void *data, int len)
{
	int r;

	if (len < 0)
		return -EINVAL;

	r = ch_read(ch, data, len);
	if (r > 0)
		notify_other_smd(ch->type);

	return r;
}

static int smd_packet_read(smd_channel_t *ch, void *data, int len)
{
	unsigned long flags;
	int r;

	if (len < 0)
		return -EINVAL;

	if (len > ch->current_packet)
		len = ch->current_packet;

	r = ch_read(ch, data, len);
	if (r > 0)
		notify_other_smd(ch->type);

	spin_lock_irqsave(&smd_lock, flags);
	ch->current_packet -= r;
	update_packet_state(ch);
	spin_unlock_irqrestore(&smd_lock, flags);

	return r;
}

static int smd_packet_read_from_cb(smd_channel_t *ch, void *data, int len)
{
	int r;

	if (len < 0)
		return -EINVAL;

	if (len > ch->current_packet)
		len = ch->current_packet;

	r = ch_read(ch, data, len);
	if (r > 0)
		notify_other_smd(ch->type);

	ch->current_packet -= r;
	update_packet_state(ch);

	return r;
}

static struct smd_channel *_smd_alloc_channel_v1(uint32_t cid)
{
	struct smd_channel *ch;
	void *shared;

	shared = smem_alloc(ID_SMD_CHANNELS + cid,
			    2 * (sizeof(struct smd_half_channel) +
				 SMD_BUF_SIZE));
	if (!shared) {
		pr_err("smd_alloc_channel: cid %d does not exist\n", cid);
		return NULL;
	}

	ch = kzalloc(sizeof(struct smd_channel), GFP_KERNEL);
	if (ch) {
		ch->send = shared;
		ch->send_buf = shared + sizeof(struct smd_half_channel);
		ch->recv = (struct smd_half_channel *)
			(ch->send_buf + SMD_BUF_SIZE);
		ch->recv_buf = (unsigned char *)ch->recv +
			sizeof(struct smd_half_channel);
		ch->buf_size = SMD_BUF_SIZE;
		ch->n = cid;
	} else
		pr_err("smd_alloc_channel: out of memory\n");

	return ch;
}

static struct smd_channel *_smd_alloc_channel_v2(uint32_t cid)
{
	struct smd_channel *ch;
	void *shared, *shared_fifo;
	unsigned size;

	shared = smem_alloc(ID_SMD_CHANNELS + cid,
			    2 * sizeof(struct smd_half_channel));
	if (!shared) {
		pr_err("smd_alloc_channel: cid %d does not exist\n", cid);
		return NULL;
	}

	shared_fifo = _smem_find(SMEM_SMD_FIFO_BASE_ID + cid, &size);
	if (!shared_fifo) {
		pr_err("smd_alloc_channel: cid %d fifo do not exist\n", cid);
		return NULL;
	}
	SMD_INFO("smd_alloc_channel: cid %d fifo found; size = %d\n",
		 cid, (size / 2));

	ch = kzalloc(sizeof(struct smd_channel), GFP_KERNEL);
	if (ch) {
		ch->send = shared;
		ch->recv = shared + sizeof(struct smd_half_channel);
		ch->send_buf = shared_fifo;
		ch->recv_buf = shared_fifo + (size / 2);
		ch->buf_size = size / 2;
		ch->n = cid;
	} else
		pr_err("smd_alloc_channel() out of memory\n");

	return ch;
}

static void smd_alloc_channel(struct smd_alloc_elm *alloc_elm)
{
	struct smd_channel *ch;
	uint32_t *smd_ver;

	smd_ver = smem_alloc(SMEM_VERSION_SMD, 32 * sizeof(uint32_t));

	if (smd_ver && ((smd_ver[VERSION_MODEM] >> 16) >= 1))
		ch = _smd_alloc_channel_v2(alloc_elm->cid);
	else
		ch = _smd_alloc_channel_v1(alloc_elm->cid);

	if (ch == 0)
		return;

	ch->type = SMD_CHANNEL_TYPE(alloc_elm->type);
	memcpy(ch->name, alloc_elm->name, 20);
	ch->name[19] = 0;

	if (smd_is_packet(alloc_elm)) {
		ch->read = smd_packet_read;
		ch->write = smd_packet_write;
		ch->read_avail = smd_packet_read_avail;
		ch->write_avail = smd_packet_write_avail;
		ch->update_state = update_packet_state;
		ch->read_from_cb = smd_packet_read_from_cb;
	} else {
		ch->read = smd_stream_read;
		ch->write = smd_stream_write;
		ch->read_avail = smd_stream_read_avail;
		ch->write_avail = smd_stream_write_avail;
		ch->update_state = update_stream_state;
		ch->read_from_cb = smd_stream_read;
	}

	ch->pdev.name = ch->name;
	ch->pdev.id = ch->type;

	SMD_INFO("smd_alloc_channel() '%s' cid=%d\n",
		 ch->name, ch->n);

	mutex_lock(&smd_creation_mutex);
	list_add(&ch->ch_list, &smd_ch_closed_list);
	mutex_unlock(&smd_creation_mutex);

	platform_device_register(&ch->pdev);
}

static void do_nothing_notify(void *priv, unsigned flags)
{
}

struct smd_channel *smd_get_channel(const char *name, uint32_t type)
{
	struct smd_channel *ch;

	mutex_lock(&smd_creation_mutex);
	list_for_each_entry(ch, &smd_ch_closed_list, ch_list) {
		if (!strcmp(name, ch->name) &&
			(type == ch->type)) {
			list_del(&ch->ch_list);
			mutex_unlock(&smd_creation_mutex);
			return ch;
		}
	}
	mutex_unlock(&smd_creation_mutex);

	return NULL;
}

int smd_named_open_on_edge(const char *name, uint32_t edge,
			   smd_channel_t **_ch,
			   void *priv, void (*notify)(void *, unsigned))
{
	struct smd_channel *ch;
	unsigned long flags;

	if (smd_initialized == 0) {
		SMD_INFO("smd_open() before smd_init()\n");
		return -ENODEV;
	}

	SMD_DBG("smd_open('%s', %p, %p)\n", name, priv, notify);

	ch = smd_get_channel(name, edge);
	if (!ch)
		return -ENODEV;

	if (notify == 0)
		notify = do_nothing_notify;

	ch->notify = notify;
	ch->current_packet = 0;
	ch->last_state = SMD_SS_CLOSED;
	ch->priv = priv;

	*_ch = ch;

	SMD_DBG("smd_open: opening '%s'\n", ch->name);

	spin_lock_irqsave(&smd_lock, flags);
	list_add(&ch->ch_list, &smd_ch_list);
	SMD_DBG("%s: opening ch %d\n", __func__, ch->n);

	smd_state_change(ch, ch->last_state, SMD_SS_OPENING);

	spin_unlock_irqrestore(&smd_lock, flags);

	return 0;
}
EXPORT_SYMBOL(smd_named_open_on_edge);


int smd_open(const char *name, smd_channel_t **_ch,
	     void *priv, void (*notify)(void *, unsigned))
{
	return smd_named_open_on_edge(name, SMD_APPS_MODEM, _ch, priv,
				      notify);
}
EXPORT_SYMBOL(smd_open);

int smd_close(smd_channel_t *ch)
{
	unsigned long flags;

	SMD_INFO("smd_close(%p)\n", ch);

	if (ch == 0)
		return -1;

	spin_lock_irqsave(&smd_lock, flags);
	ch->notify = do_nothing_notify;
	list_del(&ch->ch_list);
	ch_set_state(ch, SMD_SS_CLOSED);
	spin_unlock_irqrestore(&smd_lock, flags);

	mutex_lock(&smd_creation_mutex);
	list_add(&ch->ch_list, &smd_ch_closed_list);
	mutex_unlock(&smd_creation_mutex);

	return 0;
}
EXPORT_SYMBOL(smd_close);

int smd_read(smd_channel_t *ch, void *data, int len)
{
	return ch->read(ch, data, len);
}
EXPORT_SYMBOL(smd_read);

int smd_read_from_cb(smd_channel_t *ch, void *data, int len)
{
	return ch->read_from_cb(ch, data, len);
}
EXPORT_SYMBOL(smd_read_from_cb);

int smd_write(smd_channel_t *ch, const void *data, int len)
{
	return ch->write(ch, data, len);
}
EXPORT_SYMBOL(smd_write);

int smd_read_avail(smd_channel_t *ch)
{
	return ch->read_avail(ch);
}
EXPORT_SYMBOL(smd_read_avail);

int smd_write_avail(smd_channel_t *ch)
{
	return ch->write_avail(ch);
}
EXPORT_SYMBOL(smd_write_avail);

int smd_wait_until_readable(smd_channel_t *ch, int bytes)
{
	return -1;
}

int smd_wait_until_writable(smd_channel_t *ch, int bytes)
{
	return -1;
}

int smd_cur_packet_size(smd_channel_t *ch)
{
	return ch->current_packet;
}

int smd_tiocmget(smd_channel_t *ch)
{
	return  (ch->recv->fDSR ? TIOCM_DSR : 0) |
		(ch->recv->fCTS ? TIOCM_CTS : 0) |
		(ch->recv->fCD ? TIOCM_CD : 0) |
		(ch->recv->fRI ? TIOCM_RI : 0) |
		(ch->send->fCTS ? TIOCM_RTS : 0) |
		(ch->send->fDSR ? TIOCM_DTR : 0);
}

int smd_tiocmset(smd_channel_t *ch, unsigned int set, unsigned int clear)
{
	unsigned long flags;

	spin_lock_irqsave(&smd_lock, flags);
	if (set & TIOCM_DTR)
		ch->send->fDSR = 1;

	if (set & TIOCM_RTS)
		ch->send->fCTS = 1;

	if (clear & TIOCM_DTR)
		ch->send->fDSR = 0;

	if (clear & TIOCM_RTS)
		ch->send->fCTS = 0;

	ch->send->fSTATE = 1;
	barrier();
	notify_other_smd(ch->type);
	spin_unlock_irqrestore(&smd_lock, flags);

	return 0;
}


/* -------------------------------------------------------------------------- */

void *smem_alloc(unsigned id, unsigned size)
{
	return smem_find(id, size);
}

void *smem_get_entry(unsigned id, unsigned *size)
{
	return _smem_find(id, size);
}

static void *_smem_find(unsigned id, unsigned *size)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	struct smem_heap_entry *toc = shared->heap_toc;

	if (id >= SMEM_NUM_ITEMS)
		return 0;

	if (toc[id].allocated) {
		*size = toc[id].size;
		return (void *) (MSM_SHARED_RAM_BASE + toc[id].offset);
	}

	return 0;
}

void *smem_find(unsigned id, unsigned size_in)
{
	unsigned size;
	void *ptr;

	ptr = _smem_find(id, &size);
	if (!ptr)
		return 0;

	size_in = ALIGN(size_in, 8);
	if (size_in != size) {
		pr_err("smem_find(%d, %d): wrong size %d\n",
		       id, size_in, size);
		return 0;
	}

	return ptr;
}

static int smem_init(void)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	uint32_t *smsm, i;

	smsm = smem_alloc(ID_SHARED_STATE,
			  SMSM_NUM_ENTRIES * sizeof(uint32_t));

	if (smsm) {
		smsm[SMSM_APPS_STATE] = 0;
		if ((shared->version[VERSION_MODEM] >> 16) >= 0xB)
			smsm[SMSM_APPS_DEM_I] = 0;
	}

	smsm = smem_alloc(SMEM_SMSM_CPU_INTR_MASK,
			  SMSM_NUM_ENTRIES * SMSM_NUM_HOSTS * sizeof(uint32_t));

	if (smsm)
		for (i = 0; i < SMSM_NUM_ENTRIES; i++)
			smsm[i * SMSM_NUM_HOSTS + SMSM_APPS] = 0xffffffff;

	return 0;
}

void smsm_reset_modem(unsigned mode)
{
	if (mode == SMSM_SYSTEM_DOWNLOAD) {
		mode = SMSM_RESET | SMSM_SYSTEM_DOWNLOAD;
	} else if (mode == SMSM_MODEM_WAIT) {
		mode = SMSM_RESET | SMSM_MODEM_WAIT;
	} else { /* reset_mode is SMSM_RESET or default */
		mode = SMSM_RESET;
	}

	smsm_change_state(SMSM_APPS_STATE, mode, mode);
}
EXPORT_SYMBOL(smsm_reset_modem);

void smsm_reset_modem_cont(void)
{
	unsigned long flags;
	uint32_t *smsm;

	spin_lock_irqsave(&smem_lock, flags);
	smsm = smem_alloc(ID_SHARED_STATE,
			  SMSM_NUM_ENTRIES * sizeof(uint32_t));
	smsm[SMSM_APPS_STATE] &= ~SMSM_MODEM_WAIT;
	spin_unlock_irqrestore(&smem_lock, flags);
}
EXPORT_SYMBOL(smsm_reset_modem_cont);

static irqreturn_t smsm_irq_handler(int irq, void *data)
{
	unsigned long flags;
	uint32_t *smsm;
	static uint32_t prev_smem_q6_apps_smsm;

	if (irq == INT_ADSP_A11) {
		smsm = smem_alloc(SMEM_SMD_SMSM_INTR_MUX,
				  SMSM_NUM_INTR_MUX * sizeof(uint32_t));
		if (!smsm ||
		    (smsm[SMEM_Q6_APPS_SMSM] == prev_smem_q6_apps_smsm))
			return IRQ_HANDLED;

		prev_smem_q6_apps_smsm = smsm[SMEM_Q6_APPS_SMSM];
	}

	spin_lock_irqsave(&smem_lock, flags);
	smsm = smem_alloc(ID_SHARED_STATE,
			  SMSM_NUM_ENTRIES * sizeof(uint32_t));

	if (smsm == 0) {
		SMSM_INFO("<SM NO STATE>\n");
	} else {
		unsigned old_apps, apps;
		unsigned modm = smsm[SMSM_MODEM_STATE];

		old_apps = apps = smsm[SMSM_APPS_STATE];

		SMSM_DBG("<SM %08x %08x>\n", apps, modm);
		if (apps & SMSM_RESET) {
			/* If we get an interrupt and the apps SMSM_RESET
			   bit is already set, the modem is acking the
			   app's reset ack. */
			apps &= ~SMSM_RESET;

			/* Issue a fake irq to handle any
			 * smd state changes during reset
			 */
			smd_fake_irq_handler(0);

			/* queue modem restart notify chain */
			modem_queue_start_reset_notify();

		} else if (modm & SMSM_RESET) {
			apps |= SMSM_RESET;
		} else {
			apps |= SMSM_INIT;
			if (modm & SMSM_SMDINIT)
				apps |= SMSM_SMDINIT;
			if (modm & SMSM_RPCINIT)
				apps |= SMSM_RPCINIT;
			if ((apps & (SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT)) ==
				(SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT))
				apps |= SMSM_RUN;
		}

		if (smsm[SMSM_APPS_STATE] != apps) {
			SMSM_DBG("<SM %08x NOTIFY>\n", apps);
			smsm[SMSM_APPS_STATE] = apps;
			do_smd_probe();
			notify_other_smsm(SMSM_APPS_STATE, old_apps, apps);
		}
	}
	spin_unlock_irqrestore(&smem_lock, flags);
	return IRQ_HANDLED;
}

int smsm_change_state(uint32_t smsm_entry,
		      uint32_t clear_mask, uint32_t set_mask)
{
	unsigned long flags;
	uint32_t  *smsm;
	uint32_t  old_state;

	if (smsm_entry >= SMSM_NUM_ENTRIES) {
		printk(KERN_ERR "smsm_change_state: Invalid entry %d",
		       smsm_entry);
		return -EINVAL;
	}

	spin_lock_irqsave(&smem_lock, flags);

	smsm = smem_alloc(ID_SHARED_STATE,
			  SMSM_NUM_ENTRIES * sizeof(uint32_t));

	if (smsm) {
		old_state = smsm[smsm_entry];
		smsm[smsm_entry] = (smsm[smsm_entry] & ~clear_mask) | set_mask;
		SMSM_DBG("smsm_change_state %x\n", smsm[smsm_entry]);
		notify_other_smsm(SMSM_APPS_STATE, old_state, smsm[smsm_entry]);
	}

	spin_unlock_irqrestore(&smem_lock, flags);

	if (smsm == NULL) {
		printk(KERN_ERR "smsm_change_state <SM NO STATE>\n");
		return -EIO;
	}
	return 0;
}

int smsm_change_intr_mask(uint32_t smsm_entry,
			  uint32_t clear_mask, uint32_t set_mask)
{
	uint32_t  *smsm;

	if (smsm_entry >= SMSM_NUM_ENTRIES) {
		printk(KERN_ERR "smsm_change_state: Invalid entry %d\n",
		       smsm_entry);
		return -EINVAL;
	}

	smsm = smem_alloc(SMEM_SMSM_CPU_INTR_MASK,
			  SMSM_NUM_ENTRIES * SMSM_NUM_HOSTS * sizeof(uint32_t));

	if (smsm) {
		smsm[smsm_entry * SMSM_NUM_HOSTS + SMSM_APPS] =
			(smsm[smsm_entry * SMSM_NUM_HOSTS + SMSM_APPS] &
			 ~clear_mask) | set_mask;
		SMSM_INFO("smsm_entry %d, new intr_mask %x\n", smsm_entry,
			  smsm[smsm_entry * SMSM_NUM_HOSTS + SMSM_APPS]);
	} else {
		printk(KERN_ERR "smsm_change_intr_mask <SM NO INTR_MASK>\n");
		return -EIO;
	}

	return 0;
}

int smsm_get_intr_mask(uint32_t smsm_entry, uint32_t *intr_mask)
{
	uint32_t  *smsm;

	if ((smsm_entry >= SMSM_NUM_ENTRIES) || (!intr_mask)) {
		printk(KERN_ERR "smsm_change_state: Invalid input "
		       "entry %d, mask 0x%x\n",
		       smsm_entry, (unsigned int)intr_mask);
		return -EINVAL;
	}

	smsm = smem_alloc(SMEM_SMSM_CPU_INTR_MASK,
			  SMSM_NUM_ENTRIES * SMSM_NUM_HOSTS * sizeof(uint32_t));

	if (smsm) {
		*intr_mask = smsm[smsm_entry * SMSM_NUM_HOSTS + SMSM_APPS];
	} else {
		printk(KERN_ERR "smsm_change_intr_mask <SM NO INTR_MASK>\n");
		return -EIO;
	}

	return 0;
}

uint32_t smsm_get_state(uint32_t smsm_entry)
{
	unsigned long flags;
	uint32_t *smsm;
	uint32_t rv;

	if (smsm_entry >= SMSM_NUM_ENTRIES) {
		printk(KERN_ERR "smsm_change_state: Invalid entry %d",
		       smsm_entry);
		return -EINVAL;
	}

	spin_lock_irqsave(&smem_lock, flags);

	smsm = smem_alloc(ID_SHARED_STATE,
			  SMSM_NUM_ENTRIES * sizeof(uint32_t));

	if (smsm)
		rv = smsm[smsm_entry];
	else
		rv = 0;

	spin_unlock_irqrestore(&smem_lock, flags);

	if (smsm == NULL)
		printk(KERN_ERR "smsm_get_state <SM NO STATE>\n");
	return rv;

}

#define MAX_NUM_SLEEP_CLIENTS       64
#define MAX_SLEEP_NAME_LEN          8

#define NUM_GPIO_INT_REGISTERS 6
#define GPIO_SMEM_NUM_GROUPS 2
#define GPIO_SMEM_MAX_PC_INTERRUPTS 8
struct tramp_gpio_save {
  unsigned int enable;
  unsigned int detect;
  unsigned int polarity;
};

struct tramp_gpio_smem {
	uint16_t num_fired[GPIO_SMEM_NUM_GROUPS];
	uint16_t fired[GPIO_SMEM_NUM_GROUPS][GPIO_SMEM_MAX_PC_INTERRUPTS];
	uint32_t enabled[NUM_GPIO_INT_REGISTERS];
	uint32_t detection[NUM_GPIO_INT_REGISTERS];
	uint32_t polarity[NUM_GPIO_INT_REGISTERS];
};

/*
 * Print debug information on shared memory sleep variables
 */
void smsm_print_sleep_info(uint32_t sleep_delay, uint32_t sleep_limit,
	uint32_t irq_mask, uint32_t wakeup_reason, uint32_t pending_irqs)
{
	unsigned long flags;
	uint32_t *ptr;
	struct tramp_gpio_smem *gpio;

	spin_lock_irqsave(&smem_lock, flags);

	printk(KERN_ERR "SMEM_SMSM_SLEEP_DELAY: %x\n", sleep_delay);
	printk(KERN_ERR "SMEM_SMSM_LIMIT_SLEEP: %x\n", sleep_limit);

#if defined(CONFIG_KERNEL_MOTOROLA)
	ptr = smem_alloc(SMEM_SLEEP_POWER_COLLAPSE_DISABLED, sizeof(uint32_t));
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
	ptr = smem_alloc(SMEM_SLEEP_POWER_COLLAPSE_DISABLED, sizeof(*ptr));
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
	if (ptr)
		printk(KERN_ERR "SMEM_SLEEP_POWER_COLLAPSE_DISABLED: %x\n", *ptr);
	else
		printk(KERN_ERR "SMEM_SLEEP_POWER_COLLAPSE_DISABLED: missing\n");

	printk(KERN_ERR "SMEM_SMSM_INT_INFO %x %x %x\n",
		irq_mask, pending_irqs, wakeup_reason);

#if defined(CONFIG_KERNEL_MOTOROLA)
	gpio = smem_alloc( SMEM_GPIO_INT, sizeof(struct tramp_gpio_smem)); 
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
	gpio = smem_alloc(SMEM_GPIO_INT, sizeof(*gpio));
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
	if (gpio) {
		int i;
		for (i = 0; i < NUM_GPIO_INT_REGISTERS; i++) {
			printk(KERN_ERR "SMEM_GPIO_INT: %d: e %x d %x p %x\n",
			       i, gpio->enabled[i], gpio->detection[i],
			       gpio->polarity[i]);
		}
		for (i = 0; i < GPIO_SMEM_NUM_GROUPS; i++) {
			printk(KERN_ERR "SMEM_GPIO_INT: %d: f %d: %d %d...\n",
			       i, gpio->num_fired[i], gpio->fired[i][0],
			       gpio->fired[i][1]);
		}
	} else
		printk(KERN_ERR "SMEM_GPIO_INT: missing\n");

#if 0
	ptr = smem_alloc(SMEM_SLEEP_STATIC,
			2 * MAX_NUM_SLEEP_CLIENTS * (MAX_SLEEP_NAME_LEN + 1));
	if (ptr)
		printk(KERN_ERR "SMEM_SLEEP_STATIC: %x %x %x %x\n",
			ptr[0], ptr[1], ptr[2], ptr[3]);
	else
		printk(KERN_ERR "SMEM_SLEEP_STATIC: missing\n");
#endif

	spin_unlock_irqrestore(&smem_lock, flags);
}

int smd_core_init(void)
{
	int r;
	SMD_INFO("smd_core_init()\n");

	r = request_irq(INT_A9_M2A_0, smd_irq_handler,
			IRQF_TRIGGER_RISING, "smd_dev", 0);
	if (r < 0)
		return r;
	r = enable_irq_wake(INT_A9_M2A_0);
	if (r < 0)
		printk(KERN_ERR "smd_core_init: "
			"enable_irq_wake failed for INT_A9_M2A_0\n");

	r = request_irq(INT_A9_M2A_5, smsm_irq_handler,
			IRQF_TRIGGER_RISING, "smsm_dev", 0);
	if (r < 0) {
		free_irq(INT_A9_M2A_0, 0);
		return r;
	}

	r = enable_irq_wake(INT_A9_M2A_5);
	if (r < 0)
		printk(KERN_ERR "smd_core_init: "
		       "enable_irq_wake failed for INT_A9_M2A_5\n");

	r = request_irq(INT_ADSP_A11, smd_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_SHARED, "smd_dev",
			smd_irq_handler);
	if (r < 0)
		printk(KERN_ERR "smd_core_init: "
		       "request_irq failed for INT_ADSP_A11\n");

	r = request_irq(INT_ADSP_A11, smsm_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_SHARED, "smsm_dev",
			smsm_irq_handler);
	if (r < 0)
		printk(KERN_ERR "smd_core_init: "
		       "request_irq failed for INT_ADSP_A11\n");

	r = enable_irq_wake(INT_ADSP_A11);
	if (r < 0)
		printk(KERN_ERR "smd_core_init: "
		       "enable_irq_wake failed for INT_ADSP_A11\n");

	/* we may have missed a signal while booting -- fake
	 * an interrupt to make sure we process any existing
	 * state
	 */
	smsm_irq_handler(0, 0);

	SMD_INFO("smd_core_init() done\n");

	return 0;
}

#if defined(CONFIG_DEBUG_FS)

static int debug_f3(char *buf, int max)
{
	char *x;
	int size;
	int i = 0, j = 0;
	unsigned cols = 0;
	char str[4*sizeof(unsigned)+1] = {0};

	i += scnprintf(buf + i, max - i,
		       "Printing to log\n");

	x = smem_get_entry(SMEM_ERR_F3_TRACE_LOG, &size);
	if (x != 0) {
		printk(KERN_ERR "smem: F3 TRACE LOG\n");
		while (size > 0) {
			if (size >= sizeof(unsigned)) {
				printk(KERN_ERR "%08x", *((unsigned *) x));
				for (j = 0; j < sizeof(unsigned); ++j)
					if (isprint(*(x+j)))
						str[cols*sizeof(unsigned) + j]
							= *(x+j);
					else
						str[cols*sizeof(unsigned) + j]
							= '-';
				x += sizeof(unsigned);
				size -= sizeof(unsigned);
			} else {
				while (size-- > 0)
					printk(KERN_ERR "%02x",
					       (unsigned) *x++);
				break;
			}
			if (cols == 3) {
				cols = 0;
				str[4*sizeof(unsigned)] = 0;
				printk(KERN_ERR " %s\n", str);
				str[0] = 0;
			} else {
				cols++;
				printk(KERN_ERR " ");
			}
		}
		printk(KERN_ERR "\n");
	}

	return max;
}

static int debug_diag(char *buf, int max)
{
	int i = 0;

	i += scnprintf(buf + i, max - i,
		       "Printing to log\n");
	smd_diag();

	return i;
}

static int debug_modem_err_f3(char *buf, int max)
{
	char *x;
	int size;
	int i = 0, j = 0;
	unsigned cols = 0;
	char str[4*sizeof(unsigned)+1] = {0};

	x = smem_get_entry(SMEM_ERR_F3_TRACE_LOG, &size);
	if (x != 0) {
		printk(KERN_ERR "smem: F3 TRACE LOG\n");
		while (size > 0 && max - i) {
			if (size >= sizeof(unsigned)) {
				i += scnprintf(buf + i, max - i, "%08x",
					       *((unsigned *) x));
				for (j = 0; j < sizeof(unsigned); ++j)
					if (isprint(*(x+j)))
						str[cols*sizeof(unsigned) + j]
							= *(x+j);
					else
						str[cols*sizeof(unsigned) + j]
							= '-';
				x += sizeof(unsigned);
				size -= sizeof(unsigned);
			} else {
				while (size-- > 0 && max - i)
					i += scnprintf(buf + i, max - i,
						       "%02x",
						       (unsigned) *x++);
				break;
			}
			if (cols == 3) {
				cols = 0;
				str[4*sizeof(unsigned)] = 0;
				i += scnprintf(buf + i, max - i, " %s\n",
					       str);
				str[0] = 0;
			} else {
				cols++;
				i += scnprintf(buf + i, max - i, " ");
			}
		}
		i += scnprintf(buf + i, max - i, "\n");
	}

	return i;
}

static int debug_modem_err(char *buf, int max)
{
	char *x;
	int size;
	int i = 0;

	x = smem_find(ID_DIAG_ERR_MSG, SZ_DIAG_ERR_MSG);
	if (x != 0) {
		x[SZ_DIAG_ERR_MSG - 1] = 0;
		i += scnprintf(buf + i, max - i,
			       "smem: DIAG '%s'\n", x);
	}

	x = smem_get_entry(SMEM_ERR_CRASH_LOG, &size);
	if (x != 0) {
		x[size - 1] = 0;
		i += scnprintf(buf + i, max - i,
			       "smem: CRASH LOG\n'%s'\n", x);
	}
	i += scnprintf(buf + i, max - i, "\n");

	return i;
}

static int dump_ch(char *buf, int max, int n,
		  struct smd_half_channel *s,
		  struct smd_half_channel *r)
{
	return scnprintf(
		buf, max,
		"ch%02d:"
		" %8s(%04d/%04d) %c%c%c%c%c%c%c <->"
		" %8s(%04d/%04d) %c%c%c%c%c%c%c\n", n,
		chstate(s->state), s->tail, s->head,
		s->fDSR ? 'D' : 'd',
		s->fCTS ? 'C' : 'c',
		s->fCD ? 'C' : 'c',
		s->fRI ? 'I' : 'i',
		s->fHEAD ? 'W' : 'w',
		s->fTAIL ? 'R' : 'r',
		s->fSTATE ? 'S' : 's',
		chstate(r->state), r->tail, r->head,
		r->fDSR ? 'D' : 'd',
		r->fCTS ? 'R' : 'r',
		r->fCD ? 'C' : 'c',
		r->fRI ? 'I' : 'i',
		r->fHEAD ? 'W' : 'w',
		r->fTAIL ? 'R' : 'r',
		r->fSTATE ? 'S' : 's'
		);
}

static int debug_read_diag_msg(char *buf, int max)
{
	char *msg;
	int i = 0;

	msg = smem_find(ID_DIAG_ERR_MSG, SZ_DIAG_ERR_MSG);

	if (msg) {
		msg[SZ_DIAG_ERR_MSG - 1] = 0;
		i += scnprintf(buf + i, max - i, "diag: '%s'\n", msg);
	}
	return i;
}

static int debug_read_mem(char *buf, int max)
{
	unsigned n;
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	struct smem_heap_entry *toc = shared->heap_toc;
	int i = 0;

	i += scnprintf(buf + i, max - i,
		       "heap: init=%d free=%d remain=%d\n",
		       shared->heap_info.initialized,
		       shared->heap_info.free_offset,
		       shared->heap_info.heap_remaining);

	for (n = 0; n < SMD_HEAP_SIZE; n++) {
		if (toc[n].allocated == 0)
			continue;
		i += scnprintf(buf + i, max - i,
			       "%04d: offset %08x size %08x\n",
			       n, toc[n].offset, toc[n].size);
	}
	return i;
}

static int debug_read_ch_v1(char *buf, int max)
{
	void *shared;
	int n, i = 0;

	for (n = 0; n < SMD_CHANNELS; n++) {
		shared = smem_find(ID_SMD_CHANNELS + n,
				   2 * (sizeof(struct smd_half_channel) +
					SMD_BUF_SIZE));

		if (shared == 0)
			continue;
		i += dump_ch(buf + i, max - i, n, shared,
			     (shared + sizeof(struct smd_half_channel) +
			      SMD_BUF_SIZE));
	}

	return i;
}

static int debug_read_ch_v2(char *buf, int max)
{
	void *shared;
	int n, i = 0;

	for (n = 0; n < SMD_CHANNELS; n++) {
		shared = smem_find(ID_SMD_CHANNELS + n,
				   2 * sizeof(struct smd_half_channel));

		if (shared == 0)
			continue;
		i += dump_ch(buf + i, max - i, n, shared,
			     (shared + sizeof(struct smd_half_channel)));
	}

	return i;
}

static int debug_read_smem_version(char *buf, int max)
{
	struct smem_shared *shared = (void *) MSM_SHARED_RAM_BASE;
	uint32_t n, version, i = 0;

	for (n = 0; n < 32; n++) {
		version = shared->version[n];
		i += scnprintf(buf + i, max - i,
			       "entry %d: smem = %d  proc_comm = %d\n", n,
			       version >> 16,
			       version & 0xffff);
	}

	return i;
}

static int debug_read_smd_version(char *buf, int max)
{
	uint32_t *smd_ver;
	uint32_t n, version, i = 0;

	smd_ver = smem_alloc(SMEM_VERSION_SMD, 32 * sizeof(uint32_t));

	if (smd_ver)
		for (n = 0; n < 32; n++) {
			version = smd_ver[n];
			i += scnprintf(buf + i, max - i,
				       "entry %d: %d.%d\n", n,
				       version >> 16,
				       version & 0xffff);
		}

	return i;
}

static int debug_read_alloc_tbl(char *buf, int max)
{
	struct smd_alloc_elm *shared;
	int n, i = 0;

	shared = smem_find(ID_CH_ALLOC_TBL, sizeof(struct smd_alloc_elm[64]));

	BUG_ON(!shared);

	for (n = 0; n < 64; n++) {
		i += scnprintf(buf + i, max - i,
				"name=%s cid=%d ch type=%d "
				"xfer type=%d ref_count=%d\n",
				shared[n].name,
				shared[n].cid,
				SMD_CHANNEL_TYPE(shared[n].type),
				SMD_XFER_TYPE(shared[n].type),
				shared[n].ref_count);
	}

	return i;
}

static int debug_read_smsm_state(char *buf, int max)
{
	uint32_t *smsm;
	int n, i = 0;

	smsm = smem_find(ID_SHARED_STATE,
			 SMSM_NUM_ENTRIES * sizeof(uint32_t));

	if (smsm)
		for (n = 0; n < SMSM_NUM_ENTRIES; n++)
			i += scnprintf(buf + i, max - i, "entry %d: 0x%08x\n",
				       n, smsm[n]);

	return i;

}

static int debug_read_intr_mask(char *buf, int max)
{
	uint32_t *smsm;
	int m, n, i = 0;

	smsm = smem_alloc(SMEM_SMSM_CPU_INTR_MASK,
			  SMSM_NUM_ENTRIES * SMSM_NUM_HOSTS * sizeof(uint32_t));

	if (smsm)
		for (m = 0; m < SMSM_NUM_ENTRIES; m++) {
			i += scnprintf(buf + i, max - i, "entry %d:", m);
			for (n = 0; n < SMSM_NUM_HOSTS; n++)
				i += scnprintf(buf + i, max - i,
					       "   host %d: 0x%08x",
					       n, smsm[m * SMSM_NUM_HOSTS + n]);
			i += scnprintf(buf + i, max - i, "\n");
		}

	return i;
}

static int debug_read_intr_mux(char *buf, int max)
{
	uint32_t *smsm;
	int n, i = 0;

	smsm = smem_alloc(SMEM_SMD_SMSM_INTR_MUX,
			  SMSM_NUM_INTR_MUX * sizeof(uint32_t));

	if (smsm)
		for (n = 0; n < SMSM_NUM_INTR_MUX; n++)
			i += scnprintf(buf + i, max - i, "entry %d: %d\n",
				       n, smsm[n]);

	return i;
}

#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

static void debug_create(const char *name, mode_t mode,
			 struct dentry *dent,
			 int (*fill)(char *buf, int max))
{
	debugfs_create_file(name, mode, dent, fill, &debug_ops);
}

static void smd_debugfs_init(void)
{
	struct dentry *dent;
	uint32_t *smd_ver;

	dent = debugfs_create_dir("smd", 0);
	if (IS_ERR(dent))
		return;

	smd_ver = smem_alloc(SMEM_VERSION_SMD, 32 * sizeof(uint32_t));

	if (smd_ver && ((smd_ver[VERSION_MODEM] >> 16) >= 1))
		debug_create("ch", 0444, dent, debug_read_ch_v2);
	else
		debug_create("ch", 0444, dent, debug_read_ch_v1);

	debug_create("diag", 0444, dent, debug_read_diag_msg);
	debug_create("mem", 0444, dent, debug_read_mem);
	debug_create("version", 0444, dent, debug_read_smd_version);
	debug_create("tbl", 0444, dent, debug_read_alloc_tbl);
	debug_create("modem_err", 0444, dent, debug_modem_err);
	debug_create("modem_err_f3", 0444, dent, debug_modem_err_f3);
	debug_create("print_diag", 0444, dent, debug_diag);
	debug_create("print_f3", 0444, dent, debug_f3);
}

static void smsm_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("smsm", 0);
	if (IS_ERR(dent))
		return;

	debug_create("state", 0444, dent, debug_read_smsm_state);
	debug_create("intr_mask", 0444, dent, debug_read_intr_mask);
	debug_create("intr_mux", 0444, dent, debug_read_intr_mux);
	debug_create("version", 0444, dent, debug_read_smem_version);
}
#else
static void smd_debugfs_init(void) {}
static void smsm_debugfs_init(void) {}
#endif

static int __init msm_smd_probe(struct platform_device *pdev)
{
	/* enable smd and smsm info messages */
	msm_smd_debug_mask = 0xc;

	SMD_INFO("smd probe\n");

	INIT_WORK(&probe_work, smd_channel_probe_worker);

	if (smem_init()) {
		printk(KERN_ERR "smem_init() failed\n");
		return -1;
	}

	if (smd_core_init()) {
		printk(KERN_ERR "smd_core_init() failed\n");
		return -1;
	}

	do_smd_probe();

	msm_check_for_modem_crash = check_for_modem_crash;

	smd_initialized = 1;

	smd_debugfs_init();
	smsm_debugfs_init();

	return 0;
}

static struct platform_driver msm_smd_driver = {
	.probe = msm_smd_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_smd_init(void)
{
	return platform_driver_register(&msm_smd_driver);
}

module_init(msm_smd_init);

MODULE_DESCRIPTION("MSM Shared Memory Core");
MODULE_AUTHOR("Brian Swetland <swetland@google.com>");
MODULE_LICENSE("GPL");

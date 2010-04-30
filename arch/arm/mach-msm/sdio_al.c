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
 */

/*
 * SDIO-Abstraction-Layer Module.
 *
 * To be used with Qualcomm's SDIO-Client connected to this host.
 */

#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include <mach/sdio_al.h>

#define MODULE_MAME "sdio_al"

/**
 *  Func#0 has SDIO standard registers
 *  Func#1 is for Mailbox.
 *  Functions 2..7 are for channels.
 *  Currently only functions 2..5 are active due to SDIO-Client
 *  number of pipes.
 *
 */
#define SDIO_AL_MAX_CHANNELS 4

/** Func 1..5 */
#define SDIO_AL_MAX_FUNCS    (SDIO_AL_MAX_CHANNELS+1)

/** Number of SDIO-Client pipes */
#define SDIO_AL_MAX_PIPES    16

/** CMD53/CMD54 Block size */
#define SDIO_AL_BLOCK_SIZE   128

/** Func#1 Mailbox base address    */
#define MAILBOX_ADDR			0x1000

/** Some Mailbox registers address, written by host for
 control */
#define PIPES_THRESHOLD_ADDR		0x01000

#define PIPES_0_7_IRQ_MASK_ADDR 	0x01048

#define PIPES_8_15_IRQ_MASK_ADDR	0x0104C

#define EOT_PIPES_ENABLE		0x00

/** Maximum read/write data available is SDIO-Client limitation */
#define MAX_DATA_AVILABLE   		(16*1024)
#define INVALID_DATA_AVILABLE  		(0x8000)

#define DEFAULT_WRITE_THRESHOLD 	(MAX_DATA_AVILABLE/2)
#define DEFAULT_READ_THRESHOLD  	(MAX_DATA_AVILABLE/2)
#define DEFAULT_MIN_WRITE_THRESHOLD 	1024
#define THRESHOLD_DISABLE_VAL  		(0xFFFFFFFF)

#define DEFAULT_POLL_DELAY_MSEC			10

#define PEER_BLOCK_SIZE (1536)

#define ROUND_UP(x, n) (((x + n - 1) / n) * n)

/** Func#2..7 FIFOs are r/w via
 sdio_readsb() & sdio_writesb(),when inc_addr=0 */
#define PIPE_RX_FIFO_ADDR   0x00
#define PIPE_TX_FIFO_ADDR   0x00

/** Context validity check */
#define SDIO_AL_SIGNATURE 0xAABBCCDD

/* Vendor Specific Command */
#define SD_IO_RW_EXTENDED_QCOM 54

/** Channel priority */
enum sdio_priority {
	SDIO_PRIORITY_HIGH = 1,
	SDIO_PRIORITY_MED  = 5,
	SDIO_PRIORITY_LOW  = 9,
};

enum sdio_irq_state {
	SDIO_IRQ_STATE_UNUSED = 0,
	SDIO_IRQ_STATE_CLAIMED = 1,
	SDIO_IRQ_STATE_DETECTED = 2,
	SDIO_IRQ_STATE_CLEARED = 3,
};

/**
 *  Mailbox structure.
 *  The Mailbox is located on the SDIO-Client Function#1.
 *  The mailbox size is 128 bytes, which is one block.
 *  The mailbox allows the host ton:
 *  1. Get the number of available bytes on the pipes.
 *  2. Enable/Disable SDIO-Client interrupt, related to pipes.
 *  3. Set the Threshold for generating interrupt.
 *
 */
struct sdio_mailbox {
	u32 pipe_bytes_threshold[SDIO_AL_MAX_PIPES]; /* Addr 0x1000 */

	/* Mask USER interrupts generated towards host - Addr 0x1040 */
	u32 mask_irq_func_1:8; /* LSB */
	u32 mask_irq_func_2:8;
	u32 mask_irq_func_3:8;
	u32 mask_irq_func_4:8;

	u32 mask_irq_func_5:8;
	u32 mask_irq_func_6:8;
	u32 mask_irq_func_7:8;
	u32 mask_mutex_irq:8;

	/* Mask PIPE interrupts generated towards host - Addr 0x1048 */
	u32 mask_eot_pipe_0_7:8;
	u32 mask_thresh_above_limit_pipe_0_7:8;
	u32 mask_overflow_pipe_0_7:8;
	u32 mask_underflow_pipe_0_7:8;

	u32 mask_eot_pipe_8_15:8;
	u32 mask_thresh_above_limit_pipe_8_15:8;
	u32 mask_overflow_pipe_8_15:8;
	u32 mask_underflow_pipe_8_15:8;

	/* Status of User interrupts generated towards host - Addr 0x1050 */
	u32 user_irq_func_1:8;
	u32 user_irq_func_2:8;
	u32 user_irq_func_3:8;
	u32 user_irq_func_4:8;

	u32 user_irq_func_5:8;
	u32 user_irq_func_6:8;
	u32 user_irq_func_7:8;
	u32 user_mutex_irq:8;

	/* Status of PIPE interrupts generated towards host */
	/* Note: All sources are cleared once they read. - Addr 0x1058 */
	u32 eot_pipe_0_7:8;
	u32 thresh_above_limit_pipe_0_7:8;
	u32 overflow_pipe_0_7:8;
	u32 underflow_pipe_0_7:8;

	u32 eot_pipe_8_15:8;
	u32 thresh_above_limit_pipe_8_15:8;
	u32 overflow_pipe_8_15:8;
	u32 underflow_pipe_8_15:8;

	u16 pipe_bytes_avail[SDIO_AL_MAX_PIPES];
};

/** Track pending Rx Packet size */
struct rx_packet_size {
	u32 size; /* in bytes */
	struct list_head	list;
};

/**
 * 	pending request info.
 */
struct sdio_request {
	struct sdio_channel *ch;
	int is_write;
	void *data;
	int len;
	struct list_head	list;
};

/**
 *  SDIO Channel context.
 *
 *  @name - channel name. Used by the caller to open the
 *  	  channel.
 *
 *  @priority - channel priority.
 *
 *  @read_threshold - Threshold on SDIO-Client mailbox for Rx
 *  				Data available bytes. When the limit exceed
 *  				the SDIO-Client generates an interrupt to the
 *  				host.
 *
 *  @write_threshold - Threshold on SDIO-Client mailbox for Tx
 *  				Data available bytes. When the limit exceed
 *  				the SDIO-Client generates an interrupt to the
 *  				host.
 *
 *  @min_write_avail - Threshold of minimal available bytes
 *  					 to write. Below that threshold the host
 *  					 will initiate reading the mailbox.
 *
 *  @poll_delay_msec - Delay between polling the mailbox. When
 *  				 the SDIO-Client doesn't generates EOT
 *  				 interrupt for Rx Available bytes, the host
 *  				 should poll the SDIO-Client mailbox.
 *
 *  @is_packet_mode - The host get interrupt when a packet is
 *  				available at the SDIO-client (pipe EOT
 *  				indication).
 *
 *  @num - channel number.
 *
 *  @notify - Client's callback. Should not call sdio read/write.
 *
 *  @priv - Client's private context, provided to callback.
 *
 *  @is_open - Channel is open.
 *
 *  @func - SDIO Function handle.
 *
 *  @rx_pipe_index - SDIO-Client Pipe Index for Rx Data.
 *
 *  @tx_pipe_index - SDIO-Client Pipe Index for Tx Data.
 *
 *  @rx_pending_bytes - Total number of Rx pending bytes, at Rx
 *  				  packet list. Maximum of 16KB-1 limited by
 *  				  SDIO-Client specification.
 *
 *  @read_avail - Avilable bytes to read.
 *
 *  @write_avail - Avilable bytes to write.
 *
 *  @rx_size_list_head - The head of Rx Pending Packets List.
 *
 *  @request - Pending client Read/Write request. There is a
 *  		 pending request when there is a higher priority
 *  		 channel using the bus.Only one request can be
 *  		 pending per channel since the SDIO-AL read/write
 *  		 API is blocking.
 *
 *  @completion - sync request completion.
 *
 *  @dev - user space device node.
 *
 *  @signature - Context Validity check.
 *
 */
struct sdio_channel {
	/* Channel Configuration Parameters*/
	const char *name;
	int priority;
	int read_threshold;
	int write_threshold;
	int min_write_avail;
	int poll_delay_msec;
	int is_packet_mode;

	/* Channel Info */
	int num;

	void (*notify)(void *priv, unsigned channel_event);
	void *priv;

	int is_open;

	struct sdio_func *func;

	int rx_pipe_index;
	int tx_pipe_index;

	struct mutex ch_lock;

	u32 read_avail;
	u32 write_avail;

	u32 peer_block_size;

	u16 rx_pending_bytes;

	struct list_head rx_size_list_head;

	struct sdio_request *request;
	struct completion *completion;

	struct device *dev;
	wait_queue_head_t   wait_cdev;

	u32 signature;
};

/**
 *  SDIO Abstraction Layer driver context.
 *
 *  @card - card claimed.
 *
 *  @mailbox - A shadow of the SDIO-Client mailbox.
 *
 *  @channel - Channels context.
 *
 *  @bus_lock - Lock when the bus is in use.
 *
 *  @workqueue - workqueue to read the mailbox and handle
 *  		   pending requests according to priority.
 *  		   Reading the mailbox should not happen in
 *  		   interrupt context.
 *
 *  @work - work to submitt to workqueue.
 *
 *  @is_ready - driver is ready.
 *
 *  @ask_mbox - Flag to request reading the mailbox,
 *  					  for different reasonsns.
 *
 *  @irq_state - interrupt detected. Need to release and
 *  			  re-calim the sdio-irq.
 *
 *  @timer - timer to use for polling the mailbox.
 *
 *  @poll_delay_msec - timer delay for polling the mailbox.
 *
 *  @use_irq - allow to work in polling mode or interrupt mode.
 *
 *  @pdev - platform device - clients to prob for the sdio-al.
 *
 *  @is_err - error detected.
 *
 *  @signature - Context Validity Check.
 *
 */
struct sdio_al {
	struct mmc_card *card;
	struct sdio_mailbox *mailbox;
	struct sdio_channel channel[SDIO_AL_MAX_CHANNELS];

	struct mutex bus_lock;
	struct workqueue_struct *workqueue;
	struct work_struct work;

	int is_ready;

	wait_queue_head_t   wait_mbox;
	int irq_state;
	int ask_mbox;

	struct timer_list timer;
	int poll_delay_msec;

	int use_irq;

	struct platform_device pdev;

	dev_t dev_num;
	struct class *dev_class;
	struct cdev *cdev;

	int is_err;

	u32 signature;
};

/** The driver context */
static struct sdio_al *sdio_al;

/* Static funcions declaration */
static int enable_eot_interrupt(int pipe_index, int enable);
static int enable_threshold_interrupt(int pipe_index, int enable);
static void sdio_func_irq(struct sdio_func *func);
static void timer_handler(unsigned long data);
static int get_min_poll_time_msec(void);
static u32 check_pending_rx_packet(struct sdio_channel *ch, u32 eot);
static u32 remove_handled_rx_packet(struct sdio_channel *ch);
static int set_pipe_threshold(int pipe_index, int threshold);

/**
 *  Read SDIO-Client Mailbox from Function#1.thresh_pipe
 *
 *  The mailbox contain the bytes available per pipe,
 *  and the End-Of-Transfer indication per pipe (if avilable).
 *
 * WARNING: Each time the Mailbox is read from the client, the
 * read_bytes_avail is incremented with another pending
 * transfer. Therefore, a pending rx-packet should be added to a
 * list before the next read of the mailbox.
 *
 * This function should run from a workqueue context since it
 * notifies the clients.
 *
 */
static int read_mailbox(void)
{
	int ret;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];
	struct sdio_mailbox *mailbox = sdio_al->mailbox;
	u32 write_avail = 0;
	u32 old_write_avail = 0;
	int i;
	u32 rx_notify_bitmask = 0;
	u32 tx_notify_bitmask = 0;
	u32 eot_pipe = 0;
	u32 thresh_pipe = 0;
	u32 overflow_pipe = 0;
	u32 underflow_pipe = 0;
	u32 thres_intr_mask = 0;

	if (sdio_al->is_err) {
		pr_info(MODULE_MAME ":In Error state, ignore request\n");
		return 0;
	}

	pr_debug(MODULE_MAME ":Wait for read mailbox request..\n");
	wait_event(sdio_al->wait_mbox,
	   sdio_al->ask_mbox);
	sdio_al->ask_mbox = false;


	if (mutex_is_locked(&sdio_al->bus_lock))
		pr_debug(MODULE_MAME ":bus is locked\n");

	pr_debug(MODULE_MAME ":Reading Mailbox...\n");

	mutex_lock(&sdio_al->bus_lock);
	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = sdio_memcpy_fromio(func1, mailbox,
			MAILBOX_ADDR, sizeof(*mailbox));
	sdio_release_host(sdio_al->card->sdio_func[0]);
	mutex_unlock(&sdio_al->bus_lock);

	eot_pipe =	(mailbox->eot_pipe_0_7) |
			(mailbox->eot_pipe_8_15<<8);
	thresh_pipe = 	(mailbox->thresh_above_limit_pipe_0_7) |
			(mailbox->thresh_above_limit_pipe_8_15<<8);

	overflow_pipe = (mailbox->overflow_pipe_0_7) |
			(mailbox->overflow_pipe_8_15<<8);
	underflow_pipe = mailbox->underflow_pipe_0_7 |
			(mailbox->underflow_pipe_8_15<<8);
	thres_intr_mask =
		(mailbox->mask_thresh_above_limit_pipe_0_7) |
		(mailbox->mask_thresh_above_limit_pipe_8_15<<8);


	if (ret) {
		pr_info(MODULE_MAME ":Fail to read Mailbox,"
				    " goto error state\n");
		sdio_al->is_err = true;
		/* Stop the timer to stop reading the mailbox */
		sdio_al->poll_delay_msec = 0;
		return ret;
	}

	if (overflow_pipe || underflow_pipe)
		pr_info(MODULE_MAME ":Mailbox ERROR "
				"overflow=0x%x, underflow=0x%x\n",
				overflow_pipe, underflow_pipe);


	pr_debug(MODULE_MAME ":eot=0x%x, thresh=0x%x\n",
			 eot_pipe, thresh_pipe);

	/* Scan for Rx Packets available and update read vailable bytes */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];
		u32 read_avail;
		u32 new_packet_size = 0;

		if (!ch->is_open)
			continue;

		read_avail = mailbox->pipe_bytes_avail[ch->rx_pipe_index];

		if (read_avail > INVALID_DATA_AVILABLE) {
			pr_debug(MODULE_MAME
				 ":Invalid read_avail 0x%x for pipe %d\n",
				 read_avail, ch->rx_pipe_index);
			continue;
		}

		if (ch->is_packet_mode)
			new_packet_size = check_pending_rx_packet(ch, eot_pipe);
		else
			ch->read_avail = read_avail;

		if (((ch->is_packet_mode) && (new_packet_size > 0)) ||
		    ((!ch->is_packet_mode) && (ch->read_avail > 0)))
			rx_notify_bitmask |= (1<<ch->num);
	}

	/* Update Write available */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];

		if (!ch->is_open)
			continue;

		write_avail = mailbox->pipe_bytes_avail[ch->tx_pipe_index];

		if (write_avail > INVALID_DATA_AVILABLE) {
			pr_debug(MODULE_MAME
				 ":Invalid write_avail 0x%x for pipe %d\n",
				 write_avail, ch->tx_pipe_index);
			continue;
		}

		old_write_avail = ch->write_avail;
		ch->write_avail = write_avail;

		if ((old_write_avail < ch->min_write_avail) &&
			(write_avail >= ch->write_threshold))
			tx_notify_bitmask |= (1<<ch->num);
	}

	if ((rx_notify_bitmask == 0) && (tx_notify_bitmask == 0))
		pr_debug(MODULE_MAME ":Nothing to Notify\n");

	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];

		if ((!ch->is_open) || (ch->notify == NULL))
			continue;

		if (rx_notify_bitmask & (1<<ch->num))
			ch->notify(ch->priv,
					   SDIO_EVENT_DATA_READ_AVAIL);

		if (tx_notify_bitmask & (1<<ch->num))
			ch->notify(ch->priv,
					   SDIO_EVENT_DATA_WRITE_AVAIL);
	}

	/* Enable/Disable of Interrupts */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];
		u32 pipe_thresh_intr_disabled = 0;

		if (!ch->is_open)
			continue;


		pipe_thresh_intr_disabled = thres_intr_mask &
			(1<<ch->tx_pipe_index);
	}

	pr_debug(MODULE_MAME ":Reading Mailbox Completed...\n");

	if (sdio_al->irq_state == SDIO_IRQ_STATE_DETECTED) {
		sdio_al->irq_state = SDIO_IRQ_STATE_CLEARED;
		wake_up(&sdio_al->wait_mbox);
	}

	return ret;
}

/**
 *  Check pending rx packet when reading the mailbox.
 */
static u32 check_pending_rx_packet(struct sdio_channel *ch, u32 eot)
{
	u32 rx_pending;
	u32 rx_avail;
	u32 new_packet_size = 0;

	mutex_lock(&ch->ch_lock);

	rx_pending = ch->rx_pending_bytes;
	rx_avail = sdio_al->mailbox->pipe_bytes_avail[ch->rx_pipe_index];

	pr_debug(MODULE_MAME ":pipe %d rx_avail=0x%x , rx_pending=0x%x\n",
	   ch->rx_pipe_index, rx_avail, rx_pending);


	/* new packet detected */
	if ((rx_avail > rx_pending) && (eot & (1<<ch->rx_pipe_index))) {
		struct rx_packet_size *p = NULL;
		new_packet_size = rx_avail - rx_pending;

		p = kzalloc(sizeof(*p), GFP_KERNEL);
		if (p == NULL)
			goto exit_err;
		p->size = new_packet_size;
		/* Add new packet as last */
		list_add_tail(&p->list, &ch->rx_size_list_head);
		ch->rx_pending_bytes += new_packet_size;

		if (ch->read_avail == 0)
			ch->read_avail = new_packet_size;
	}

exit_err:
	mutex_unlock(&ch->ch_lock);

	return new_packet_size;
}



/**
 *  Remove first pending packet from the list.
 */
static u32 remove_handled_rx_packet(struct sdio_channel *ch)
{
	struct rx_packet_size *p = NULL;

	mutex_lock(&ch->ch_lock);

	ch->rx_pending_bytes -= ch->read_avail;

	if (!list_empty(&ch->rx_size_list_head)) {
		p = list_first_entry(&ch->rx_size_list_head,
			struct rx_packet_size, list);
		list_del(&p->list);
		kfree(p);
	}

	if (list_empty(&ch->rx_size_list_head))	{
		ch->read_avail = 0;
	} else {
		p = list_first_entry(&ch->rx_size_list_head,
			struct rx_packet_size, list);
		ch->read_avail = p->size;
	}

	mutex_unlock(&ch->ch_lock);

	return ch->read_avail;
}

/**
 *  Worker function.
 *
 *  @note: clear the ask_mbox flag only after
 *  	 reading the mailbox, to ignore more requests while
 *  	 reading the mailbox.
 */
static void worker(struct work_struct *work)
{
	int ret = 0;

	pr_debug(MODULE_MAME ":Worker Started..\n");
	while ((sdio_al->is_ready) && (ret == 0))
		ret = read_mailbox();
	pr_debug(MODULE_MAME ":Worker Exit!\n");
}

/**
 *  Write command using CMD54 rather than CMD53.
 *  Writting with CMD54 generate EOT interrup at the
 *  SDIO-Client.
 *  Based on mmc_io_rw_extended()
 */
static int sdio_write_cmd54(struct mmc_card *card, unsigned fn,
	unsigned addr, const u8 *buf,
	unsigned blocks, unsigned blksz)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;
	int incr_addr = 1; /* MUST */
	int write = 1;

	BUG_ON(!card);
	BUG_ON(fn > 7);
	BUG_ON(blocks == 1 && blksz > 512);
	WARN_ON(blocks == 0);
	WARN_ON(blksz == 0);

	write = true;
	pr_debug(MODULE_MAME ":sdio_write_cmd54()"
		"fn=%d,buf=0x%x,blocks=%d,blksz=%d\n",
		fn, (u32) buf, blocks, blksz);

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_IO_RW_EXTENDED_QCOM;

	cmd.arg = write ? 0x80000000 : 0x00000000;
	cmd.arg |= fn << 28;
	cmd.arg |= incr_addr ? 0x04000000 : 0x00000000;
	cmd.arg |= addr << 9;
	if (blocks == 1 && blksz <= 512)
		cmd.arg |= (blksz == 512) ? 0 : blksz;  /* byte mode */
	else
		cmd.arg |= 0x08000000 | blocks; 	/* block mode */
	cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_ADTC;

	data.blksz = blksz;
	data.blocks = blocks;
	data.flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, buf, blksz * blocks);

	mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	if (mmc_host_is_spi(card->host)) {
		/* host driver already reported errors */
	} else {
		if (cmd.resp[0] & R5_ERROR)
			return -EIO;
		if (cmd.resp[0] & R5_FUNCTION_NUMBER)
			return -EINVAL;
		if (cmd.resp[0] & R5_OUT_OF_RANGE)
			return -ERANGE;
	}

	return 0;
}


/**
 *  Write data to channel.
 *  Handle different data size types.
 *
 */
static int sdio_ch_write(struct sdio_channel *ch,
						 const u8 *buf,	u32 len)
{
	int ret = 0;
	unsigned blksz = ch->func->cur_blksize;
	int blocks = len / blksz;
	int remain_bytes = len % blksz;
	struct mmc_card *card = NULL;
	u32 fn = ch->func->num;

	card = ch->func->card;

	if (remain_bytes) {
		/* CMD53 */
		if (blocks)
			ret = sdio_memcpy_toio(ch->func, PIPE_TX_FIFO_ADDR,
					       (void *) buf, blocks*blksz);

		if (ret != 0)
			return ret;

		buf += (blocks*blksz);

		ret = sdio_write_cmd54(card, fn, PIPE_TX_FIFO_ADDR,
				buf, 1, remain_bytes);
	} else {
		ret = sdio_write_cmd54(card, fn, PIPE_TX_FIFO_ADDR,
				buf, blocks, blksz);
	}

	return ret;
}

/**
 * Set Channels Configuration.
 *
 *  @todo read channel configuration from platform data rather
 *  	  than hard-coded.
 */
static void set_default_channels_config(void)
{
   int i;

	/* Set Default Channels configuration */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		struct sdio_channel *ch = &sdio_al->channel[i];

		ch->num = i;
		ch->read_threshold  = DEFAULT_READ_THRESHOLD;
		ch->write_threshold = DEFAULT_WRITE_THRESHOLD;
		ch->min_write_avail = DEFAULT_MIN_WRITE_THRESHOLD;
		if (sdio_al->use_irq)
			ch->poll_delay_msec = 0;
		else
			ch->poll_delay_msec = DEFAULT_POLL_DELAY_MSEC;
		ch->is_packet_mode = true;
	}


	/* Note: When agregation is used,
	   there is No EOT interrupt for Rx packet ready,
	   therefore need polling */
	sdio_al->channel[0].name = "SDIO_RPC";
	sdio_al->channel[0].priority = SDIO_PRIORITY_HIGH;

	sdio_al->channel[1].name = "SDIO_RMNET_DATA";
	sdio_al->channel[1].priority = SDIO_PRIORITY_MED;
	sdio_al->channel[1].is_packet_mode = false;  /* No EOT for Rx Data */
	sdio_al->channel[1].poll_delay_msec = 30; /* TBD */

	sdio_al->channel[1].read_threshold  = 128;
	sdio_al->channel[1].write_threshold = 128;
	sdio_al->channel[1].min_write_avail = 128;

	sdio_al->channel[2].name = "SDIO_QMI";
	sdio_al->channel[2].priority = SDIO_PRIORITY_LOW;

	sdio_al->channel[3].name = "SDIO_CS_DATA";
	sdio_al->channel[3].priority = SDIO_PRIORITY_LOW;
}

/**
 *  Enable/Disable EOT interrupt of a pipe.
 *
 */
static int enable_eot_interrupt(int pipe_index, int enable)
{
	int ret = 0;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];
	u32 mask;
	u32 pipe_mask;
	u32 addr;

	if (pipe_index < 8) {
		addr = PIPES_0_7_IRQ_MASK_ADDR;
		pipe_mask = (1<<pipe_index);
	} else {
		addr = PIPES_8_15_IRQ_MASK_ADDR;
		pipe_mask = (1<<(pipe_index-8));
	}

	mask = sdio_readl(func1, addr, &ret);
	if (ret) {
		pr_debug(MODULE_MAME ":enable_eot_interrupt fail\n");
		goto exit_err;
	}

	if (enable)
		mask &= (~pipe_mask); /* 0 = enable */
	else
		mask |= (pipe_mask);  /* 1 = disable */

	sdio_writel(func1, mask, addr, &ret);

exit_err:
	return ret;
}

/**
 *  Enable/Disable Threshold interrupt of a pipe.
 *
 */
static int enable_threshold_interrupt(int pipe_index, int enable)
{
	int ret = 0;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];
	u32 mask;
	u32 pipe_mask;
	u32 addr;

	if (pipe_index < 8) {
		addr = PIPES_0_7_IRQ_MASK_ADDR;
		pipe_mask = (1<<pipe_index);
	} else {
		addr = PIPES_8_15_IRQ_MASK_ADDR;
		pipe_mask = (1<<(pipe_index-8));
	}

	mask = sdio_readl(func1, addr, &ret);
	if (ret) {
		pr_debug(MODULE_MAME ":enable_threshold_interrupt fail\n");
		goto exit_err;
	}

	pipe_mask = pipe_mask<<8; /* Threshold bits 8..15 */
	if (enable)
		mask &= (~pipe_mask); /* 0 = enable */
	else
		mask |= (pipe_mask);  /* 1 = disable */

	sdio_writel(func1, mask, addr, &ret);

exit_err:
	return ret;
}

/**
 *  Set the threshold to trigger interrupt from SDIO-Card on
 *  pipe availiable bytes.
 *
 */
static int set_pipe_threshold(int pipe_index, int threshold)
{
	int ret = 0;
	struct sdio_func *func1 = sdio_al->card->sdio_func[0];

	sdio_writel(func1, threshold,
			PIPES_THRESHOLD_ADDR+pipe_index*4, &ret);
	if (ret)
		pr_info(MODULE_MAME ":set_pipe_threshold err=%d\n", -ret);

	return ret;
}


/**
 *  Open Channel
 *
 *  1. Init Channel Context.
 *  2. Init the Channel SDIO-Function.
 *  3. Init the Channel Pipes on Mailbox.
 */
static int open_channel(struct sdio_channel *ch)
{
	int ret = 0;

	/* Init channel Context */
	/** Func#1 is reserved for mailbox */
	ch->func = sdio_al->card->sdio_func[ch->num+1];
	ch->rx_pipe_index = ch->num*2;
	ch->tx_pipe_index = ch->num*2+1;
	ch->signature = SDIO_AL_SIGNATURE;

	mutex_init(&ch->ch_lock);

	pr_debug(MODULE_MAME ":open_channel %s func#%d\n",
			 ch->name, ch->func->num);

	INIT_LIST_HEAD(&(ch->rx_size_list_head));

	/* Init SDIO Function */
	ret = sdio_enable_func(ch->func);
	if (ret) {
		pr_info(MODULE_MAME ":sdio_enable_func() err=%d\n", -ret);
		goto exit_err;
	}

	/* Note: Patch Func CIS tuple issue */
	ret = sdio_set_block_size(ch->func, SDIO_AL_BLOCK_SIZE);
	if (ret) {
		pr_info(MODULE_MAME ":sdio_set_block_size() err=%d\n", -ret);
		goto exit_err;
	}

	ch->func->max_blksize = SDIO_AL_BLOCK_SIZE;

	sdio_set_drvdata(ch->func, ch);

	/* Set Pipes Threshold on Mailbox */
	ret = set_pipe_threshold(ch->rx_pipe_index, ch->read_threshold);
	if (ret)
		goto exit_err;
	ret = set_pipe_threshold(ch->tx_pipe_index, ch->write_threshold);
	if (ret)
		goto exit_err;

	/* Check if need to start the timer */
	if  ((ch->poll_delay_msec) && (sdio_al->poll_delay_msec == 0)) {
			sdio_al->poll_delay_msec = ch->poll_delay_msec;

			init_timer(&sdio_al->timer);
			sdio_al->timer.data = (unsigned long) sdio_al;
			sdio_al->timer.function = timer_handler;
			sdio_al->timer.expires = jiffies +
				msecs_to_jiffies(sdio_al->poll_delay_msec);
			add_timer(&sdio_al->timer);
	}

	init_waitqueue_head(&ch->wait_cdev);

	/* Set flag before interrupts are enabled to allow notify */
	ch->is_open = true;

	/* The new delay will be updated at the next time
	   that the timer expires */
	sdio_al->poll_delay_msec = get_min_poll_time_msec();

	/* Enable Pipes Interrupts */
	enable_eot_interrupt(ch->rx_pipe_index, true);
	enable_eot_interrupt(ch->tx_pipe_index, true);

	enable_threshold_interrupt(ch->rx_pipe_index, true);
	enable_threshold_interrupt(ch->tx_pipe_index, true);

exit_err:

	return ret;
}

/**
 *  Channel Close
 *
 *  Disable the relevant pipes interrupt.
 *  Disable the relevant SDIO-Client function.
 *  Update/stop the timer.
 *  Remove all pending Rx Packet list.
 *
 *  @note: The timer will not restart after expired if
 *  poll time is zero
 *
 */
static int close_channel(struct sdio_channel *ch)
{
	int ret;

	enable_eot_interrupt(ch->rx_pipe_index, false);
	enable_eot_interrupt(ch->tx_pipe_index, false);

	enable_threshold_interrupt(ch->rx_pipe_index, false);
	enable_threshold_interrupt(ch->tx_pipe_index, false);

	ret = sdio_disable_func(ch->func);

	return ret;
}


/**
 *  Ask the worker to read the mailbox.
 */
static void ask_reading_mailbox(void)
{
	if (!sdio_al->ask_mbox) {
		pr_debug(MODULE_MAME ":ask_reading_mailbox\n");
		sdio_al->ask_mbox = true;
		wake_up(&sdio_al->wait_mbox);
	}
}

/**
 *  SDIO Function Interrupt handler.
 *
 *  Interrupt shall be triggerd by SDIO-Client when:
 *  1. End-Of-Transfer (EOT) detected in packet mode.
 *  2. Bytes-avilable reached the threshold.
 *
 *  Reading the mailbox clears the EOT/Threshold interrupt
 *  source.
 *  The interrupt source should be cleared before this ISR
 *  returns. This ISR is called from IRQ Thread and not
 *  interrupt, so it may sleep.
 *
 */
static void sdio_func_irq(struct sdio_func *func)
{
	struct sdio_al *al = sdio_get_drvdata(func);

	pr_debug(MODULE_MAME ":-- IRQ Detected --\n");

	al->irq_state = SDIO_IRQ_STATE_DETECTED;

	/* patch - Allow the worker to claim the host
	 * for reading the mailbox */
	sdio_release_host(func);

	ask_reading_mailbox();

	wait_event(sdio_al->wait_mbox,
		   sdio_al->irq_state == SDIO_IRQ_STATE_CLEARED);

	/* patch - IRQ-Thread claim the host again */
	sdio_claim_host(func);

	pr_debug(MODULE_MAME ":-- IRQ Completed --\n");
}

/**
 *  Timer Expire Handler
 *
 */
static void timer_handler(unsigned long data)
{
	struct sdio_al *sdio_al = (struct sdio_al *) data;

	pr_debug(MODULE_MAME " Timer Expired\n");

	ask_reading_mailbox();

	/* Restart the timer */
	if (sdio_al->poll_delay_msec) {
		sdio_al->timer.expires = jiffies +
			msecs_to_jiffies(sdio_al->poll_delay_msec);
		add_timer(&sdio_al->timer);
	}
}

/**
 *  Driver Setup.
 *
 */
static int sdio_al_setup(void)
{
	int ret = 0;
	struct mmc_card *card = sdio_al->card;
	struct sdio_func *func1;
	int i = 0;
	int fn = 0;

	pr_info(MODULE_MAME ":sdio_al_setup\n");

	if (card == NULL) {
		pr_info(MODULE_MAME ":No Card detected\n");
		return -ENODEV;
	}

	func1 = card->sdio_func[0];

	sdio_claim_host(sdio_al->card->sdio_func[0]);

	sdio_al->mailbox = kzalloc(sizeof(struct sdio_mailbox), GFP_KERNEL);
	if (sdio_al->mailbox == NULL)
		return -ENOMEM;

	/* Init Func#1 */
	ret = sdio_enable_func(func1);
	if (ret) {
		pr_info(MODULE_MAME ":Fail to enable Func#%d\n", func1->num);
		goto exit_err;
	}

	/* Patch Func CIS tuple issue */
	ret = sdio_set_block_size(func1, SDIO_AL_BLOCK_SIZE);
	func1->max_blksize = SDIO_AL_BLOCK_SIZE;

	mutex_init(&sdio_al->bus_lock);

	sdio_al->workqueue = create_singlethread_workqueue("sdio_al_wq");
	INIT_WORK(&sdio_al->work, worker);

	init_waitqueue_head(&sdio_al->wait_mbox);

	/* disable all pipes interrupts before claim irq.
	   since all are enabled by default. */
	for (i = 0 ; i < SDIO_AL_MAX_PIPES; i++) {
		enable_eot_interrupt(i, false);
		enable_threshold_interrupt(i, false);
	}

	/* Disable all SDIO Functions before claim irq. */
	for (fn = 1 ; fn <= card->sdio_funcs; fn++)
		sdio_disable_func(card->sdio_func[fn-1]);

	if (sdio_al->use_irq) {
		sdio_set_drvdata(func1, sdio_al);

		ret = sdio_claim_irq(func1, sdio_func_irq);
		if (ret) {
			pr_info(MODULE_MAME ":Fail to claim IRQ\n");
			goto exit_err;
		}
		sdio_al->irq_state = SDIO_IRQ_STATE_CLAIMED;
	} else {
		pr_debug(MODULE_MAME ":Not using IRQ\n");
	}

	sdio_release_host(sdio_al->card->sdio_func[0]);
	sdio_al->is_ready = true;

	/* Start worker before interrupt might happen */
	queue_work(sdio_al->workqueue, &sdio_al->work);

	pr_debug(MODULE_MAME ":Ready.\n");

	return 0;

exit_err:
	sdio_release_host(sdio_al->card->sdio_func[0]);
	pr_err(MODULE_MAME ":Setup Failure.\n");

	return ret;
}

/**
 *  Driver Tear-Down.
 *
 */
static void sdio_al_tear_down(void)
{
	if (sdio_al->is_ready) {
		struct sdio_func *func1;

		func1 = sdio_al->card->sdio_func[0];

		sdio_al->is_ready = false; /* Flag worker to exit */
		ask_reading_mailbox(); /* Wakeup worker */
		mdelay(100); /* allow gracefull exit of the worker thread */

		flush_workqueue(sdio_al->workqueue);
		destroy_workqueue(sdio_al->workqueue);

		sdio_claim_host(func1);
		sdio_release_irq(func1);
		sdio_disable_func(func1);
		sdio_release_host(func1);
	}
}

/**
 *  Find channel by name.
 *
 */
static struct sdio_channel *find_channel_by_name(const char *name)
{
	struct sdio_channel *ch = NULL;
	int i;

	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++)
		if (strcmp(sdio_al->channel[i].name, name) == 0) {
			ch = &sdio_al->channel[i];
			break;
		}

	WARN_ON(ch == NULL);

	return ch;
}

/**
 *  Find the minimal poll time.
 *
 */
static int get_min_poll_time_msec(void)
{
	int i;
	int poll_delay_msec = 0x0FFFFFFF;

	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++)
		if ((sdio_al->channel[i].is_open) &&
		    (sdio_al->channel[i].poll_delay_msec > 0) &&
		    (sdio_al->channel[i].poll_delay_msec < poll_delay_msec))
			poll_delay_msec = sdio_al->channel[i].poll_delay_msec;

	if (poll_delay_msec == 0x0FFFFFFF)
		poll_delay_msec = 0;

	pr_debug(MODULE_MAME ":poll delay time is %d msec\n", poll_delay_msec);

	return poll_delay_msec;
}

/**
 *  Open SDIO Channel.
 *
 *  Enable the channel.
 *  Set the channel context.
 *  Trigger reading the mailbox to check avilable bytes.
 *
 */
int sdio_open(const char *name, struct sdio_channel **ret_ch, void *priv,
		 void (*notify)(void *priv, unsigned ch_event))
{
	int ret = 0;
	struct sdio_channel *ch = NULL;

	*ret_ch = NULL; /* default */

	if (sdio_al == NULL) {
		pr_info(MODULE_MAME
			":Try to open ch %s before Module Init\n", name);
		return -ENODEV;
	}

	if (!sdio_al->is_ready) {
		ret = sdio_al_setup();
		if (ret)
			return -ENODEV;
	}

	ch = find_channel_by_name(name);
	if (ch == NULL) {
		pr_info(MODULE_MAME ":Can't find channel name %s\n", name);
		return -EINVAL;
	}

	if (ch->is_open) {
		pr_info(MODULE_MAME ":Channel already opened %s\n", name);
		return -EPERM;
	}

	ch->name = name;
	ch->notify = notify;
	ch->priv = priv;

	/* Note: Set caller returned context before interrupts are enabled */
	*ret_ch = ch;

	mutex_lock(&sdio_al->bus_lock);
	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = open_channel(ch);
	sdio_release_host(sdio_al->card->sdio_func[0]);
	mutex_unlock(&sdio_al->bus_lock);

	if (ret)
		pr_info(MODULE_MAME ":sdio_open %s err=%d\n", name, -ret);
	else
		pr_info(MODULE_MAME ":sdio_open %s completed OK\n", name);

	/* Read the mailbox after the channel is open to detect
	   pending rx packets */
	if ((!ret) && (!sdio_al->use_irq))
		ask_reading_mailbox();

	return ret;
}
EXPORT_SYMBOL(sdio_open);

/**
 *  Close SDIO Channel.
 *
 */
int sdio_close(struct sdio_channel *ch)
{
	int ret;

	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);
	WARN_ON(!ch->is_open);

	pr_debug(MODULE_MAME ":sdio_close %s\n", ch->name);

	/* Stop channel notifications, and read/write operations. */
	ch->is_open = false;
	ch->notify = NULL;

	mutex_lock(&sdio_al->bus_lock);
	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = close_channel(ch);
	sdio_release_host(sdio_al->card->sdio_func[0]);
	mutex_unlock(&sdio_al->bus_lock);

	do
		ret = remove_handled_rx_packet(ch);
	while (ret > 0);

	if  (ch->poll_delay_msec > 0)
		sdio_al->poll_delay_msec = get_min_poll_time_msec();

	return ret;
}
EXPORT_SYMBOL(sdio_close);

/**
 *  Get the number of availiable bytes to write.
 *
 */
int sdio_write_avail(struct sdio_channel *ch)
{
	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	pr_debug(MODULE_MAME ":sdio_write_avail %s 0x%x\n",
			 ch->name, ch->write_avail);

	return ch->write_avail;
}
EXPORT_SYMBOL(sdio_write_avail);

/**
 *  Get the number of availiable bytes to read.
 *
 */
int sdio_read_avail(struct sdio_channel *ch)
{
	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	pr_debug(MODULE_MAME ":sdio_read_avail %s 0x%x\n",
			 ch->name, ch->read_avail);

	return ch->read_avail;

}
EXPORT_SYMBOL(sdio_read_avail);

/**
 *  Read from SDIO Channel.
 *
 *  Reading from the pipe will trigger interrupt if there are
 *  other pending packets on the SDIO-Client.
 *
 */
int sdio_read(struct sdio_channel *ch, void *data, int len)
{
	int ret = 0;


	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	if (sdio_al->is_err) {
		pr_info(MODULE_MAME ":In Error state, ignore sdio_read\n");
		return -ENODEV;
	}

	if (!ch->is_open) {
		pr_info(MODULE_MAME ":reading from closed channel %s\n",
				 ch->name);
		return -EINVAL;
	}

	pr_debug(MODULE_MAME ":sdio_read %s buf=0x%x len=0x%x\n",
			 ch->name, (u32) data, len);

	if ((ch->is_packet_mode) && (len != ch->read_avail)) {
		pr_info(MODULE_MAME ":sdio_read ch %s len != packet size\n",
				 ch->name);
		return -EINVAL;
	}

	if (mutex_is_locked(&sdio_al->bus_lock))
		pr_debug(MODULE_MAME ":bus is locked\n");

	mutex_lock(&sdio_al->bus_lock);
	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = sdio_memcpy_fromio(ch->func, data, PIPE_RX_FIFO_ADDR, len);

	if (ret)
		pr_info(MODULE_MAME ":sdio_read err=%d\n", -ret);

	/* Remove handled packet from the list regardless if ret is ok */
	if (ch->is_packet_mode)
		remove_handled_rx_packet(ch);
	else
		ch->read_avail -= len;

	sdio_release_host(sdio_al->card->sdio_func[0]);
	mutex_unlock(&sdio_al->bus_lock);

	if ((ch->read_avail == 0) &&
	    !((ch->is_packet_mode) && (sdio_al->use_irq)))
		ask_reading_mailbox();

	return ret;
}
EXPORT_SYMBOL(sdio_read);

/**
 *  Write to SDIO Channel.
 *
 */
int sdio_write(struct sdio_channel *ch, const void *data, int len)
{
	int ret = 0;


	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);
	WARN_ON(len > ch->write_avail);

	if (sdio_al->is_err) {
		pr_info(MODULE_MAME ":In Error state, ignore sdio_write\n");
		return -ENODEV;
	}

	if (!ch->is_open) {
		pr_info(MODULE_MAME ":writting to closed channel %s\n",
				 ch->name);
		return -EINVAL;
	}

	pr_debug(MODULE_MAME ":sdio_write %s buf=0x%x len=0x%x\n",
			 ch->name, (u32) data, len);

	if (mutex_is_locked(&sdio_al->bus_lock))
		pr_debug(MODULE_MAME ":bus is locked\n");

	mutex_lock(&sdio_al->bus_lock);
	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = sdio_ch_write(ch, data, len);
	sdio_release_host(sdio_al->card->sdio_func[0]);
	mutex_unlock(&sdio_al->bus_lock);

	if (ret)
		pr_info(MODULE_MAME ":sdio_write err=%d\n", -ret);
	else
		ch->write_avail -= ROUND_UP(len, PEER_BLOCK_SIZE);

	if (ch->write_avail < ch->min_write_avail)
		ask_reading_mailbox();

	return ret;
}
EXPORT_SYMBOL(sdio_write);

/**
 *  Set the threshold to trigger interrupt from SDIO-Card on
 *  availiable bytes to write.
 *
 */
int sdio_set_write_threshold(struct sdio_channel *ch, int threshold)
{
	int ret;

	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	ch->write_threshold = threshold;

	pr_debug(MODULE_MAME ":sdio_set_write_threshold %s 0x%x\n",
			 ch->name, ch->write_threshold);

	mutex_lock(&sdio_al->bus_lock);
	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = set_pipe_threshold(ch->tx_pipe_index, ch->write_threshold);
	sdio_release_host(sdio_al->card->sdio_func[0]);
	mutex_unlock(&sdio_al->bus_lock);

	return ret;
}
EXPORT_SYMBOL(sdio_set_write_threshold);

/**
 *  Set the threshold to trigger interrupt from SDIO-Card on
 *  availiable bytes to read.
 *
 */
int sdio_set_read_threshold(struct sdio_channel *ch, int threshold)
{
	int ret;

	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	ch->read_threshold = threshold;

	pr_debug(MODULE_MAME ":sdio_set_write_threshold %s 0x%x\n",
			 ch->name, ch->read_threshold);

	mutex_lock(&sdio_al->bus_lock);
	sdio_claim_host(sdio_al->card->sdio_func[0]);
	ret = set_pipe_threshold(ch->rx_pipe_index, ch->read_threshold);
	sdio_release_host(sdio_al->card->sdio_func[0]);
	mutex_unlock(&sdio_al->bus_lock);

	return ret;
}
EXPORT_SYMBOL(sdio_set_read_threshold);


/**
 *  Set the polling delay.
 *
 */
int sdio_poll_time(struct sdio_channel *ch, int poll_delay_msec)
{
	BUG_ON(ch->signature != SDIO_AL_SIGNATURE);

	ch->poll_delay_msec = poll_delay_msec;

	/* The new delay will be updated at the next time
	   that the timer expires */
	sdio_al->poll_delay_msec = get_min_poll_time_msec();

	return sdio_al->poll_delay_msec;
}
EXPORT_SYMBOL(sdio_poll_time);

/**
 * notify callback for user space.
 */
static void cdev_notify(void *priv, unsigned ch_event)
{
	struct sdio_channel *ch = (struct sdio_channel *) priv;

	pr_debug(MODULE_MAME ":cdev_notify ch %s event %d\n",
		 ch->name, ch_event);

	wake_up(&ch->wait_cdev);
}

/**
 * open channel	for user space.
 */
static int cdev_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct sdio_channel *ch;
	struct sdio_channel *dummy;
	const char *name = "unknown";
	u32 minor = iminor(inode);

	ch = &sdio_al->channel[minor];
	file->private_data = ch;
	name = ch->name;

	pr_debug(MODULE_MAME ":cdev_open ch %s\n", name);

	ret = sdio_open(name, &dummy, ch, cdev_notify);

	return ret;
}

/**
 * release channel.
 */
static int cdev_release(struct inode *inode, struct file *file)
{
	int ret;
	struct sdio_channel *ch = (struct sdio_channel *) file->private_data;

	pr_debug(MODULE_MAME ":cdev_release\n");

	ret = sdio_close(ch);

	return ret;
}

/**
 * read from channel to user space.
 */

static ssize_t cdev_read(struct file *file, char __user *buf,
			  size_t size, loff_t *pos)
{
	int ret = 0;
	struct sdio_channel *ch = (struct sdio_channel *) file->private_data;
	int read_avail = 0;
	u8 *kbuf = NULL;

	read_avail = sdio_read_avail(ch);

	if (read_avail == 0) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			pr_debug(MODULE_MAME ":Wait for read avail\n");
			wait_event(ch->wait_cdev, ch->read_avail > 0);
			read_avail = sdio_read_avail(ch);
		}
	}

	size = min(size, (size_t) read_avail);

	kbuf = kmalloc(size, GFP_KERNEL);
	if (kbuf == NULL)
		goto exit_err;
	ret = sdio_read(ch, kbuf, size);
	WARN_ON(ret);
	if (ret)
		goto exit_err;
	ret = copy_to_user(buf, kbuf, size);
	WARN_ON(ret);
	if (ret)
		goto exit_err;

	kfree(kbuf);
	*pos += size;
	return size;

exit_err:
	kfree(kbuf);
	return ret;
}

/**
 * write to channel from user space.
 */
static ssize_t cdev_write(struct file *file, const char __user *buf,
			   size_t size, loff_t *pos)
{
	int ret = 0;
	struct sdio_channel *ch = (struct sdio_channel *) file->private_data;
	int write_avail = 0;
	u8 *kbuf = NULL;

	write_avail = sdio_write_avail(ch);
	if (write_avail == 0) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			pr_debug(MODULE_MAME ":Wait for write avail\n");
			wait_event(ch->wait_cdev, ch->write_avail > 0);
			write_avail = sdio_write_avail(ch);
		}
	}

	size = min(size, (size_t) write_avail);

	kbuf = kmalloc(size, GFP_KERNEL);
	if (kbuf == NULL)
		goto exit_err;
	ret = copy_from_user(kbuf, buf, size);
	WARN_ON(ret);
	if (ret)
		goto exit_err;
	ret = sdio_write(ch, kbuf, size);
	WARN_ON(ret);
	if (ret)
		goto exit_err;

	pr_debug(MODULE_MAME ":cdev_write ok\n");

	kfree(kbuf);
	*pos += size;
	return size;

exit_err:
	kfree(kbuf);
	return ret;
}

/**
 * File Operations for user space char device.
 */
static const struct file_operations cdev_fops = {
		.owner = THIS_MODULE,
		.open = cdev_open,
		.release = cdev_release,
		.read = cdev_read,
		.write = cdev_write,
};

/**
 *  Prob to claim the SDIO card.
 *
 */
static int mmc_probe(struct mmc_card *card)
{
    int ret;
    int i;

	dev_info(&card->dev, "Probing..\n");

	if (!mmc_card_sdio(card))
		return -ENODEV;

	if (card->sdio_funcs < SDIO_AL_MAX_FUNCS) {
		dev_info(&card->dev,
			 "SDIO-functions# %d less than expected.\n",
			 card->sdio_funcs);
		return -ENODEV;
	}

	dev_info(&card->dev, "vendor_id = 0x%x, device_id = 0x%x\n",
			 card->cis.vendor, card->cis.device);

	dev_info(&card->dev, "SDIO Card claimed.\n");

	sdio_al->card = card;

	/* Allow client to prob for this driver */
	sdio_al->pdev.name = "SDIO_AL";
	platform_device_register(&sdio_al->pdev);

	ret = alloc_chrdev_region(&sdio_al->dev_num,
				  0, SDIO_AL_MAX_CHANNELS, MODULE_MAME);
	if (ret) {
		pr_err(MODULE_MAME "alloc_chrdev_region err\n");
		return -ENODEV;
	}

	sdio_al->dev_class = class_create(THIS_MODULE, "sdio_al_class");

	/* Create file nodes for user space */
	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++) {
		sdio_al->channel[i].dev =
			device_create(sdio_al->dev_class,
				NULL, /* parent */
				MKDEV(MAJOR(sdio_al->dev_num), i),
				&sdio_al->channel[i], /* drvdata */
				"sdio_al_ch%d", i);

		if (IS_ERR(sdio_al->channel[i].dev)) {
			pr_err(MODULE_MAME ":device_create ch=%d err\n", i);
			return -ENODEV;
		}
	}


	sdio_al->cdev = cdev_alloc();
	if (sdio_al->cdev == NULL) {
		pr_err(MODULE_MAME ":cdev_alloc err\n");
		return -ENODEV;
	}

	cdev_init(sdio_al->cdev, &cdev_fops);
	sdio_al->cdev->owner = THIS_MODULE;

	ret = cdev_add(sdio_al->cdev, sdio_al->dev_num, SDIO_AL_MAX_CHANNELS);

	if (ret)
		pr_info(MODULE_MAME ":mmc_probe err=%d\n", -ret);
	else
		pr_debug(MODULE_MAME ":mmc_probe ok\n");

	return ret;
}

/**
 *  Release the SDIO card.
 *
 */
static void mmc_remove(struct mmc_card *card)
{
}

static struct mmc_driver mmc_driver = {
	.drv		= {
		.name   = "sdio_al",
	},
	.probe  	= mmc_probe,
	.remove 	= mmc_remove,
};

/**
 *  Module Init.
 *
 *  @warn: allocate sdio_al context before registering driver.
 *
 */
static int __init sdio_al_init(void)
{
	int ret = 0;

	pr_debug(MODULE_MAME ":sdio_al_init\n");

	pr_debug(MODULE_MAME ":SDIO Mailbox size=%d\n",
		 (u32) sizeof(struct sdio_mailbox));

	sdio_al = kzalloc(sizeof(struct sdio_al), GFP_KERNEL);
	if (sdio_al == NULL)
		return -ENOMEM;

	sdio_al->is_ready = false;

	sdio_al->use_irq = true;

	sdio_al->signature = SDIO_AL_SIGNATURE;

	set_default_channels_config();

	ret = mmc_register_driver(&mmc_driver);

	return ret;
}

/**
 *  Module Exit.
 *
 *  Free allocated memory.
 *  Disable SDIO-Card.
 *  Unregister driver.
 *
 */
static void __exit sdio_al_exit(void)
{
	int i;

	if (sdio_al == NULL)
		return;

	pr_debug(MODULE_MAME ":sdio_al_exit\n");

	sdio_al_tear_down();

	platform_device_unregister(&sdio_al->pdev);

	if (sdio_al->cdev)
		cdev_del(sdio_al->cdev);

	for (i = 0; i < SDIO_AL_MAX_CHANNELS; i++)
		if (sdio_al->channel[i].dev)
			device_destroy(sdio_al->dev_class,
				   MKDEV(MAJOR(sdio_al->dev_num), i));

	unregister_chrdev_region(sdio_al->dev_num, SDIO_AL_MAX_CHANNELS);

	kfree(sdio_al->mailbox);
	kfree(sdio_al);

	mmc_unregister_driver(&mmc_driver);

	pr_debug(MODULE_MAME ":sdio_al_exit complete\n");
}

module_init(sdio_al_init);
module_exit(sdio_al_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SDIO Abstraction Layer");
MODULE_AUTHOR("Amir Samuelov <amirs@qualcomm.com>");

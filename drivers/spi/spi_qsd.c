/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * SPI driver for Qualcomm QSD platforms
 *
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <mach/msm_spi.h>

#define SPI_CONFIG                    0x0000
#define SPI_IO_CONTROL                0x0004
#define SPI_IO_MODES                  0x0008
#define SPI_SW_RESET                  0x000C
#define SPI_TIME_OUT                  0x0010
#define SPI_TIME_OUT_CURRENT          0x0014
#define SPI_MX_OUTPUT_COUNT           0x0018
#define SPI_MX_OUTPUT_CNT_CURRENT     0x001C
#define SPI_MX_INPUT_COUNT            0x0020
#define SPI_MX_INPUT_CNT_CURRENT      0x0024
#define SPI_MX_READ_COUNT             0x0028
#define SPI_MX_READ_CNT_CURRENT       0x002C
#define SPI_OPERATIONAL               0x0030
#define SPI_ERROR_FLAGS               0x0034
#define SPI_ERROR_FLAGS_EN            0x0038
#define SPI_DEASSERT_WAIT             0x003C
#define SPI_OUTPUT_DEBUG              0x0040
#define SPI_INPUT_DEBUG               0x0044
#define SPI_FIFO_WORD_CNT             0x0048
#define SPI_TEST_CTRL                 0x004C
#define SPI_OUTPUT_FIFO               0x0100
#define SPI_INPUT_FIFO                0x0200

/* SPI_CONFIG fields */
#define SPI_CFG_INPUT_FIRST           0x00000200
#define SPI_CFG_LOOPBACK              0x00000100
#define SPI_CFG_N                     0x0000001F

/* SPI_IO_CONTROL fields */
#define SPI_IO_C_CLK_IDLE_HIGH        0x00000400
#define SPI_IO_C_MX_CS_MODE           0x00000100
#define SPI_IO_C_CS_N_POLARITY        0x000000F0
#define SPI_IO_C_CS_N_POLARITY_0      0x00000010
#define SPI_IO_C_CS_SELECT            0x0000000C
#define SPI_IO_C_TRISTATE_CS          0x00000002
#define SPI_IO_C_NO_TRI_STATE         0x00000001

/* SPI_IO_MODES fields */
#define SPI_IO_M_OUTPUT_BIT_SHIFT_EN  0x00004000
#define SPI_IO_M_PACK_EN              0x00002000
#define SPI_IO_M_UNPACK_EN            0x00001000
#define SPI_IO_M_INPUT_MODE           0x00000C00
#define SPI_IO_M_OUTPUT_MODE          0x00000300
#define SPI_IO_M_INPUT_FIFO_SIZE      0x000000C0
#define SPI_IO_M_INPUT_BLOCK_SIZE     0x00000030
#define SPI_IO_M_OUTPUT_FIFO_SIZE     0x0000000C
#define SPI_IO_M_OUTPUT_BLOCK_SIZE    0x00000003

/* SPI_OPERATIONAL fields */
#define SPI_OP_MAX_INPUT_DONE_FLAG    0x00000800
#define SPI_OP_MAX_OUTPUT_DONE_FLAG   0x00000400
#define SPI_OP_INPUT_SERVICE_FLAG     0x00000200
#define SPI_OP_OUTPUT_SERVICE_FLAG    0x00000100
#define SPI_OP_INPUT_FIFO_FULL        0x00000080
#define SPI_OP_OUTPUT_FIFO_FULL       0x00000040
#define SPI_OP_IP_FIFO_NOT_EMPTY      0x00000020
#define SPI_OP_OP_FIFO_NOT_EMPTY      0x00000010
#define SPI_OP_STATE_VALID            0x00000004
#define SPI_OP_STATE                  0x00000003
#define SPI_OP_STATE_RESET            0x00000000
#define SPI_OP_STATE_RUN              0x00000001
#define SPI_OP_STATE_PAUSE            0x00000003

/* SPI_ERROR_FLAGS fields */
#define SPI_ERR_TIME_OUT_ERR          0x00000040
#define SPI_ERR_OUTPUT_OVER_RUN_ERR   0x00000020
#define SPI_ERR_INPUT_UNDER_RUN_ERR   0x00000010
#define SPI_ERR_OUTPUT_UNDER_RUN_ERR  0x00000008
#define SPI_ERR_INPUT_OVER_RUN_ERR    0x00000004
#define SPI_ERR_CLK_OVER_RUN_ERR      0x00000002
#define SPI_ERR_CLK_UNDER_RUN_ERR     0x00000001
#define SPI_ERR_MASK                  0x0000007F

#define SPI_NUM_CHIPSELECTS           4
#define SPI_QSD_NAME                  "spi_qsd"

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2");
MODULE_ALIAS("platform:spi_qsd");

#define SPI_CLOCK_MAX            19200000

#ifdef CONFIG_DEBUG_FS
/* Used to create debugfs entries */
static const struct {
	const char *name;
	mode_t mode;
	int offset;
} debugfs_spi_regs[] = {
	{"config",                S_IRUGO | S_IWUSR, SPI_CONFIG},
	{"io_control",            S_IRUGO | S_IWUSR, SPI_IO_CONTROL},
	{"io_modes",              S_IRUGO | S_IWUSR, SPI_IO_MODES},
	{"sw_reset",                        S_IWUSR, SPI_SW_RESET},
	{"time_out",              S_IRUGO | S_IWUSR, SPI_TIME_OUT},
	{"time_out_current",      S_IRUGO,           SPI_TIME_OUT_CURRENT},
	{"mx_output_count",       S_IRUGO | S_IWUSR, SPI_MX_OUTPUT_COUNT},
	{"mx_output_cnt_current", S_IRUGO,           SPI_MX_OUTPUT_CNT_CURRENT},
	{"mx_input_count",        S_IRUGO | S_IWUSR, SPI_MX_INPUT_COUNT},
	{"mx_input_cnt_current",  S_IRUGO,           SPI_MX_INPUT_CNT_CURRENT},
	{"mx_read_count",         S_IRUGO | S_IWUSR, SPI_MX_READ_COUNT},
	{"mx_read_cnt_current",   S_IRUGO,           SPI_MX_READ_CNT_CURRENT},
	{"operational",           S_IRUGO | S_IWUSR, SPI_OPERATIONAL},
	{"error_flags",           S_IRUGO | S_IWUSR, SPI_ERROR_FLAGS},
	{"error_flags_en",        S_IRUGO | S_IWUSR, SPI_ERROR_FLAGS_EN},
	{"deassert_wait",         S_IRUGO | S_IWUSR, SPI_DEASSERT_WAIT},
	{"output_debug",          S_IRUGO,           SPI_OUTPUT_DEBUG},
	{"input_debug",           S_IRUGO,           SPI_INPUT_DEBUG},
	{"fifo_word_cnt",         S_IRUGO,           SPI_FIFO_WORD_CNT},
	{"test_ctrl",             S_IRUGO | S_IWUSR, SPI_TEST_CTRL},
	{"output_fifo",                     S_IWUSR, SPI_OUTPUT_FIFO},
	{"input_fifo" ,           S_IRUSR,           SPI_INPUT_FIFO},
};
#endif

struct msm_spi {
	u8                      *read_buf;
	const u8                *write_buf;
	void __iomem		*base;
	struct device           *dev;
	spinlock_t               queue_lock;
	struct list_head         queue;
	struct workqueue_struct	*workqueue;
	struct work_struct       work_data;
	struct spi_message      *cur_msg;
	struct spi_transfer     *cur_transfer;
	struct completion        transfer_complete;
	struct clk              *clk;
	struct clk              *pclk;
	unsigned long            mem_phys_addr;
	size_t                   mem_size;
	u32                      rx_bytes_remaining;
	u32                      tx_bytes_remaining;
	u32                      clock_speed;
	u32                      irq_in;
	u32                      irq_out;
	u32                      irq_err;
	int                      bytes_per_word;
	bool                     suspended;
	bool                     transfer_in_progress;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent_spi;
	struct dentry *debugfs_spi_regs[ARRAY_SIZE(debugfs_spi_regs)];
#endif
};

static int input_fifo_size;

static void msm_spi_clock_set(struct msm_spi *dd, int speed)
{
	int rc;

	rc = clk_set_rate(dd->clk, speed);
	if (!rc)
		dd->clock_speed = speed;
}

static void __init msm_spi_calculate_fifo_size(struct msm_spi *dd)
{
	u32 spi_iom;
	int block;
	int mult;
	int words;

	spi_iom = readl(dd->base + SPI_IO_MODES);
	block = (spi_iom & SPI_IO_M_INPUT_BLOCK_SIZE) >> 4;
	mult = (spi_iom & SPI_IO_M_INPUT_FIFO_SIZE) >> 6;
	switch (block) {
	case 0:
		words = 1;
		break;
	case 1:
		words = 4;
		break;
	case 2:
		words = 8;
		break;
	default:
		goto fifo_size_err;
	}
	switch (mult) {
	case 0:
		input_fifo_size = words * 2;
		break;
	case 1:
		input_fifo_size = words * 4;
		break;
	case 2:
		input_fifo_size = words * 8;
		break;
	default:
		goto fifo_size_err;
	}

	return;

fifo_size_err:
	printk(KERN_WARNING "%s: invalid FIFO size, SPI_IO_MODES=0x%x\n",
	       __func__, spi_iom);
	return;
}

static void msm_spi_read_word_from_fifo(struct msm_spi *dd)
{
	u32   data_in;
	int   i;
	int   shift;

	data_in = readl(dd->base + SPI_INPUT_FIFO);
	if (dd->read_buf) {
		for (i = 0; (i < dd->bytes_per_word) &&
			     dd->rx_bytes_remaining; i++) {
			/* The data format depends on bytes_per_word:
			   4 bytes: 0x12345678
			   3 bytes: 0x00123456
			   2 bytes: 0x00001234
			   1 byte : 0x00000012
			*/
			shift = 8 * (dd->bytes_per_word - i - 1);
			*dd->read_buf++ = (data_in & (0xFF << shift)) >> shift;
			dd->rx_bytes_remaining--;
		}
	} else {
		if (dd->rx_bytes_remaining >= dd->bytes_per_word)
			dd->rx_bytes_remaining -= dd->bytes_per_word;
		else
			dd->rx_bytes_remaining = 0;
	}
}

static irqreturn_t msm_spi_input_irq(int irq, void *dev_id)
{
	struct msm_spi	       *dd = dev_id;

	while ((readl(dd->base + SPI_OPERATIONAL) & SPI_OP_IP_FIFO_NOT_EMPTY) &&
	       (dd->rx_bytes_remaining > 0)) {
		msm_spi_read_word_from_fifo(dd);
	}
	if (dd->rx_bytes_remaining == 0)
		complete(&dd->transfer_complete);

	return IRQ_HANDLED;
}

static void msm_spi_write_word_to_fifo(struct msm_spi *dd)
{
	u32    word;
	u8     byte;
	int    i;

	word = 0;
	if (dd->write_buf) {
		for (i = 0; (i < dd->bytes_per_word) &&
			     dd->tx_bytes_remaining; i++) {
			dd->tx_bytes_remaining--;
			byte = *dd->write_buf++;
			word |= (byte << (BITS_PER_BYTE * (3 - i)));
		}
	} else
		if (dd->tx_bytes_remaining > dd->bytes_per_word)
			dd->tx_bytes_remaining -= dd->bytes_per_word;
		else
			dd->tx_bytes_remaining = 0;
	writel(word, dd->base + SPI_OUTPUT_FIFO);
}

static irqreturn_t msm_spi_output_irq(int irq, void *dev_id)
{
	struct msm_spi	       *dd = dev_id;
	int                     count = 0;

	/* Output FIFO is empty. Transmit any outstanding write data. */
	/* There could be one word in input FIFO, so don't send more  */
	/* than input_fifo_size - 1 more words.                       */
	while ((dd->tx_bytes_remaining > 0) &&
	       (count < input_fifo_size - 1) &&
	       !(readl(dd->base + SPI_OPERATIONAL) & SPI_OP_OUTPUT_FIFO_FULL)) {
		msm_spi_write_word_to_fifo(dd);
		count++;
	}

	return IRQ_HANDLED;
}

static irqreturn_t msm_spi_error_irq(int irq, void *dev_id)
{
	struct spi_master	*master = dev_id;
	struct msm_spi          *dd = spi_master_get_devdata(master);
	u32                      spi_err;

	spi_err = readl(dd->base + SPI_ERROR_FLAGS);
	if (spi_err & SPI_ERR_TIME_OUT_ERR)
		dev_warn(master->dev.parent, "SPI timeout error\n");
	if (spi_err & SPI_ERR_OUTPUT_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI output overrun error\n");
	if (spi_err & SPI_ERR_INPUT_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI input underrun error\n");
	if (spi_err & SPI_ERR_OUTPUT_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI output underrun error\n");
	if (spi_err & SPI_ERR_INPUT_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI input overrun error\n");
	if (spi_err & SPI_ERR_CLK_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI clock overrun error\n");
	if (spi_err & SPI_ERR_CLK_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI clock underrun error\n");
	writel(SPI_ERR_MASK, dd->base + SPI_ERROR_FLAGS);
	return IRQ_HANDLED;
}

static void msm_spi_process_transfer(struct msm_spi *dd)
{
	u8  bpw;
	u32 spi_config;
	u32 spi_ioc;
	u32 spi_ioc_orig;
	u32 max_speed;
	u32 chip_select;
	u32 read_count;

	if (!dd->cur_transfer->len)
		return;
	dd->tx_bytes_remaining = dd->cur_transfer->len;
	dd->rx_bytes_remaining = dd->cur_transfer->len;
	dd->read_buf           = dd->cur_transfer->rx_buf;
	dd->write_buf          = dd->cur_transfer->tx_buf;
	init_completion(&dd->transfer_complete);
	if (dd->cur_transfer->bits_per_word)
		bpw = dd->cur_transfer->bits_per_word;
	else
		if (dd->cur_msg->spi->bits_per_word)
			bpw = dd->cur_msg->spi->bits_per_word;
		else
			bpw = 8;
	dd->bytes_per_word = (bpw + 7) / 8;

	spi_config = readl(dd->base + SPI_CONFIG);
	if ((bpw - 1) != (spi_config & SPI_CFG_N))
		spi_config = (spi_config & ~SPI_CFG_N) | (bpw - 1);
	if (dd->cur_msg->spi->mode & SPI_CPHA)
		spi_config &= ~SPI_CFG_INPUT_FIRST;
	else
		spi_config |= SPI_CFG_INPUT_FIRST;
	if (dd->cur_msg->spi->mode & SPI_LOOP)
		spi_config |= SPI_CFG_LOOPBACK;
	else
		spi_config &= ~SPI_CFG_LOOPBACK;
	writel(spi_config, dd->base + SPI_CONFIG);
	if (dd->cur_transfer->speed_hz)
		max_speed = dd->cur_transfer->speed_hz;
	else
		max_speed = dd->cur_msg->spi->max_speed_hz;
	if (!dd->clock_speed || max_speed < dd->clock_speed)
		msm_spi_clock_set(dd, max_speed);

	/* read_count cannot exceed fifo_size, and only one READ COUNT
	   interrupt is generated per transaction, so for transactions larger
	   than fifo size READ COUNT must be disabled.
	*/
	read_count = (dd->cur_transfer->len + dd->bytes_per_word - 1) /
		dd->bytes_per_word;
	if (read_count <= input_fifo_size)
		writel(read_count, dd->base + SPI_MX_READ_COUNT);
	else
		writel(0, dd->base + SPI_MX_READ_COUNT);

	spi_ioc = readl(dd->base + SPI_IO_CONTROL);
	spi_ioc_orig = spi_ioc;
	if (dd->cur_msg->spi->mode & SPI_CPOL)
		spi_ioc |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		spi_ioc &= ~SPI_IO_C_CLK_IDLE_HIGH;
	chip_select = dd->cur_msg->spi->chip_select << 2;
	if ((spi_ioc & SPI_IO_C_CS_SELECT) != chip_select)
		spi_ioc = (spi_ioc & ~SPI_IO_C_CS_SELECT) | chip_select;
	if (!dd->cur_transfer->cs_change)
		spi_ioc |= SPI_IO_C_MX_CS_MODE;
	if (spi_ioc != spi_ioc_orig)
		writel(spi_ioc, dd->base + SPI_IO_CONTROL);

	writel((readl(dd->base + SPI_OPERATIONAL)
		& ~SPI_OP_STATE) | SPI_OP_STATE_RUN,
	       dd->base + SPI_OPERATIONAL);
	/* The output fifo interrupt handler will handle all writes after
	   the first. Restricting this to one write avoids contention
	   issues and race conditions between this thread and the int handler
	*/
	msm_spi_write_word_to_fifo(dd);
	wait_for_completion(&dd->transfer_complete);
	writel(spi_ioc & ~SPI_IO_C_MX_CS_MODE, dd->base + SPI_IO_CONTROL);
	writel((readl(dd->base + SPI_OPERATIONAL)
		& ~SPI_OP_STATE) | SPI_OP_STATE_RESET,
	       dd->base + SPI_OPERATIONAL);
}

/* workqueue - pull messages from queue & process */
static void msm_spi_workq(struct work_struct *work)
{
	struct msm_spi      *dd =
		container_of(work, struct msm_spi, work_data);
	unsigned long        flags;
	u32                  spi_op;
	bool                 status_error = 0;

	spi_op = readl(dd->base + SPI_OPERATIONAL);
	if (spi_op & SPI_OP_STATE_VALID) {
		spi_op &= ~SPI_OP_STATE;
		spi_op |= SPI_OP_STATE_RUN;
	} else {
		dev_err(dd->dev, "%s: SPI operational state not valid\n",
			__func__);
		status_error = 1;
	}

	dd->transfer_in_progress = 1;
	spin_lock_irqsave(&dd->queue_lock, flags);
	while (!list_empty(&dd->queue)) {
		dd->cur_msg = list_entry(dd->queue.next,
					 struct spi_message, queue);
		list_del_init(&dd->cur_msg->queue);
		spin_unlock_irqrestore(&dd->queue_lock, flags);
		if (status_error)
			dd->cur_msg->status = -EIO;
		else {
			list_for_each_entry(dd->cur_transfer,
					    &dd->cur_msg->transfers,
					    transfer_list) {
				msm_spi_process_transfer(dd);
			}
		}
		if (dd->cur_msg->complete)
			dd->cur_msg->complete(dd->cur_msg->context);
		spin_lock_irqsave(&dd->queue_lock, flags);
	}
	spin_unlock_irqrestore(&dd->queue_lock, flags);
	dd->transfer_in_progress = 0;
}

static int msm_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct msm_spi	*dd;
	unsigned long    flags;

	dd = spi_master_get_devdata(spi->master);
	if (dd->suspended)
		return -EBUSY;
	spin_lock_irqsave(&dd->queue_lock, flags);
	list_add_tail(&msg->queue, &dd->queue);
	spin_unlock_irqrestore(&dd->queue_lock, flags);
	queue_work(dd->workqueue, &dd->work_data);
	return 0;
}

static int msm_spi_setup(struct spi_device *spi)
{
	struct msm_spi	*dd;
	int              rc = 0;
	u32              spi_ioc;
	u32              spi_config;
	u32              mask;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;
	if (spi->bits_per_word < 4 || spi->bits_per_word > 32) {
		dev_err(&spi->dev, "%s: invalid bits_per_word %d\n",
			__func__, spi->bits_per_word);
		rc = -EINVAL;
	}
	if (spi->mode & ~(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LOOP)) {
		dev_err(&spi->dev, "%s, unsupported mode bits %x\n",
			__func__,
			spi->mode & ~(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH
							  | SPI_LOOP));
		rc = -EINVAL;
	}
	if (spi->chip_select > SPI_NUM_CHIPSELECTS-1) {
		dev_err(&spi->dev, "%s, chip select %d exceeds max value %d\n",
			__func__, spi->chip_select, SPI_NUM_CHIPSELECTS - 1);
		rc = -EINVAL;
	}

	if (rc)
		goto err_setup_exit;

	dd = spi_master_get_devdata(spi->master);
	spi_ioc = readl(dd->base + SPI_IO_CONTROL);
	mask = SPI_IO_C_CS_N_POLARITY_0 << spi->chip_select;
	if (spi->mode & SPI_CS_HIGH)
		spi_ioc |= mask;
	else
		spi_ioc &= ~mask;
	if (spi->mode & SPI_CPOL)
		spi_ioc |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		spi_ioc &= ~SPI_IO_C_CLK_IDLE_HIGH;
	writel(spi_ioc, dd->base + SPI_IO_CONTROL);

	spi_config = readl(dd->base + SPI_CONFIG);
	if (spi->mode & SPI_LOOP)
		spi_config |= SPI_CFG_LOOPBACK;
	else
		spi_config &= ~SPI_CFG_LOOPBACK;
	if (spi->mode & SPI_CPHA)
		spi_config &= ~SPI_CFG_INPUT_FIRST;
	else
		spi_config |= SPI_CFG_INPUT_FIRST;
	writel(spi_config, dd->base + SPI_CONFIG);

err_setup_exit:
	return rc;
}

#ifdef CONFIG_DEBUG_FS
static int debugfs_iomem_x32_set(void *data, u64 val)
{
	iowrite32(val, data);
	wmb();
	return 0;
}

static int debugfs_iomem_x32_get(void *data, u64 *val)
{
	*val = ioread32(data);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_iomem_x32, debugfs_iomem_x32_get,
			debugfs_iomem_x32_set, "0x%08llx\n");

static void spi_debugfs_init(struct msm_spi *dd)
{
	dd->dent_spi = debugfs_create_dir(dev_name(dd->dev), NULL);
	if (dd->dent_spi) {
		int i;
		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++) {
			dd->debugfs_spi_regs[i] =
			   debugfs_create_file(
			       debugfs_spi_regs[i].name,
			       debugfs_spi_regs[i].mode,
			       dd->dent_spi,
			       dd->base + debugfs_spi_regs[i].offset,
			       &fops_iomem_x32);
		}
	}
}

static void spi_debugfs_exit(struct msm_spi *dd)
{
	if (dd->dent_spi) {
		int i;
		debugfs_remove_recursive(dd->dent_spi);
		dd->dent_spi = NULL;
		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++)
			dd->debugfs_spi_regs[i] = NULL;
	}
}
#else
#define spi_debugfs_init NULL
#define spi_debugfs_exit NULL
#endif

static int __init msm_spi_probe(struct platform_device *pdev)
{
	struct spi_master      *master;
	struct msm_spi	       *dd;
	struct resource	       *resmem;
	int			rc = 0;
	struct clk	       *pclk;
	struct msm_spi_platform_data *pdata = pdev->dev.platform_data;

	master = spi_alloc_master(&pdev->dev, sizeof(struct msm_spi));
	if (!master) {
		rc = -ENOMEM;
		dev_dbg(&pdev->dev, "master allocation failed\n");
		goto err_probe_exit;
	}

	master->bus_num        = pdev->id;
	master->num_chipselect = SPI_NUM_CHIPSELECTS;
	master->setup          = msm_spi_setup;
	master->transfer       = msm_spi_transfer;
	platform_set_drvdata(pdev, master);
	dd = spi_master_get_devdata(master);

	dd->irq_in  = platform_get_irq_byname(pdev, "spi_irq_in");
	dd->irq_out = platform_get_irq_byname(pdev, "spi_irq_out");
	dd->irq_err = platform_get_irq_byname(pdev, "spi_irq_err");
	if ((dd->irq_in < 0) || (dd->irq_out < 0) || (dd->irq_err < 0))
		goto err_probe_res;

	resmem  = platform_get_resource_byname(pdev,
					       IORESOURCE_MEM, "spi_base");
	if (!resmem) {
		rc = -ENXIO;
		goto err_probe_res;
	}

	if (pdata && pdata->gpio_config) {
		rc = pdata->gpio_config();
		if (rc) {
			dev_err(&pdev->dev, "%s: error configuring GPIOs\n",
			       __func__);
			goto err_probe_gpio;
		}
	}

	spin_lock_init(&dd->queue_lock);
	INIT_LIST_HEAD(&dd->queue);
	INIT_WORK(&dd->work_data, msm_spi_workq);
	dd->workqueue = create_singlethread_workqueue(
		master->dev.parent->bus_id);
	if (!dd->workqueue)
		goto err_probe_workq;

	dd->mem_phys_addr = resmem->start;
	dd->mem_size = (resmem->end - resmem->start) + 1;
	if (!request_mem_region(dd->mem_phys_addr, dd->mem_size,
				SPI_QSD_NAME)) {
		rc = -ENXIO;
		goto err_probe_reqmem;
	}

	dd->base = ioremap(dd->mem_phys_addr, dd->mem_size);
	if (!dd->base)
		goto err_probe_ioremap;

	dd->dev = &pdev->dev;
	dd->clk = clk_get(&pdev->dev, "spi_clk");
	if (IS_ERR(dd->clk)) {
		dev_err(&pdev->dev, "%s: unable to get spi_clk\n", __func__);
		rc = PTR_ERR(dd->clk);
		goto err_probe_clk_get;
	}
	rc = clk_enable(dd->clk);
	if (rc) {
		dev_err(&pdev->dev, "%s: unable to enable spi_clk\n",
			__func__);
		goto err_probe_clk_enable;
	}
	pclk = clk_get(&pdev->dev, "spi_pclk");
	if (!IS_ERR(pclk)) {
		dd->pclk = pclk;
		rc = clk_enable(dd->pclk);
		if (rc) {
			dev_err(&pdev->dev, "%s: unable to enable spi_pclk\n",
				__func__);
			goto err_probe_pclk_enable;
		}
	}
	if (pdata && pdata->max_clock_speed)
		msm_spi_clock_set(dd, pdata->max_clock_speed);
	msm_spi_calculate_fifo_size(dd);
	writel(0x00000000, dd->base + SPI_OPERATIONAL);
	writel(0x00000000, dd->base + SPI_CONFIG);
	writel(0x00000000, dd->base + SPI_IO_MODES);
	writel(SPI_IO_C_NO_TRI_STATE, dd->base + SPI_IO_CONTROL);
	if (!(readl(dd->base + SPI_OPERATIONAL) & SPI_OP_STATE_VALID)) {
		dev_err(&pdev->dev, "%s: SPI operational state not valid\n",
			__func__);
		rc = -1;
		goto err_probe_state;
	}
	writel(SPI_OP_STATE_RUN, dd->base + SPI_OPERATIONAL);

	dd->suspended = 0;
	dd->transfer_in_progress = 0;

	rc = request_irq(dd->irq_in, msm_spi_input_irq, IRQF_TRIGGER_RISING,
			  pdev->name, dd);
	if (rc)
		goto err_probe_irq1;
	rc = request_irq(dd->irq_out, msm_spi_output_irq, IRQF_TRIGGER_RISING,
			  pdev->name, dd);
	if (rc)
		goto err_probe_irq2;
	rc = request_irq(dd->irq_err, msm_spi_error_irq, IRQF_TRIGGER_RISING,
			  pdev->name, master);
	if (rc)
		goto err_probe_irq3;

	rc = spi_register_master(master);
	if (rc)
		goto err_probe_reg_master;

	spi_debugfs_init(dd);

	return 0;

err_probe_reg_master:
	free_irq(dd->irq_err, master);
err_probe_irq3:
	free_irq(dd->irq_out, dd);
err_probe_irq2:
	free_irq(dd->irq_in, dd);
err_probe_irq1:
err_probe_state:
	if (dd->pclk)
		clk_disable(dd->pclk);
err_probe_pclk_enable:
	if (dd->pclk)
		clk_put(dd->pclk);
	clk_disable(dd->clk);
err_probe_clk_enable:
	clk_put(dd->clk);
err_probe_clk_get:
	iounmap(dd->base);
err_probe_ioremap:
	release_mem_region(dd->mem_phys_addr, dd->mem_size);
err_probe_reqmem:
	destroy_workqueue(dd->workqueue);
err_probe_workq:
err_probe_gpio:
	if (pdata && pdata->gpio_release)
		pdata->gpio_release();
err_probe_res:
	spi_master_put(master);
err_probe_exit:
	return rc;
}

#ifdef CONFIG_PM
static int msm_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd;
	int                limit = 0;

	if (!master)
		goto suspend_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto suspend_exit;
	dd->suspended = 1;
	while ((!list_empty(&dd->queue) || dd->transfer_in_progress) &&
	       limit < 50) {
		limit++;
		msleep(1);
	}

	disable_irq(dd->irq_in);
	disable_irq(dd->irq_out);
	disable_irq(dd->irq_err);
	clk_disable(dd->clk);

suspend_exit:
	return 0;
}

static int msm_spi_resume(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd;
	int rc;

	if (!master)
		goto resume_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto resume_exit;

	rc = clk_enable(dd->clk);
	if (rc) {
		dev_err(dd->dev, "%s: unable to enable spi_clk\n",
			__func__);
		goto resume_exit;
	}

	enable_irq(dd->irq_in);
	enable_irq(dd->irq_out);
	enable_irq(dd->irq_err);
	dd->suspended = 0;
resume_exit:
	return 0;
}
#else
#define msm_spi_suspend NULL
#define msm_spi_resume NULL
#endif /* CONFIG_PM */

static int __devexit msm_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd = spi_master_get_devdata(master);
	struct msm_spi_platform_data *pdata = pdev->dev.platform_data;

	spi_debugfs_exit(dd);

	free_irq(dd->irq_in, dd);
	free_irq(dd->irq_out, dd);
	free_irq(dd->irq_err, master);

	if (pdata && pdata->gpio_release)
		pdata->gpio_release();

	iounmap(dd->base);
	release_mem_region(dd->mem_phys_addr, dd->mem_size);
	clk_disable(dd->clk);
	clk_put(dd->clk);
	if (dd->pclk) {
		clk_disable(dd->pclk);
		clk_put(dd->pclk);
	}
	destroy_workqueue(dd->workqueue);
	platform_set_drvdata(pdev, 0);
	spi_unregister_master(master);
	spi_master_put(master);

	return 0;
}

static struct platform_driver msm_spi_driver = {
	.probe          = msm_spi_probe,
	.driver		= {
		.name	= "spi_qsd",
		.owner	= THIS_MODULE,
	},
	.suspend        = msm_spi_suspend,
	.resume         = msm_spi_resume,
	.remove		= __exit_p(msm_spi_remove),
};

static int __init msm_spi_init(void)
{
	return platform_driver_register(&msm_spi_driver);
}
module_init(msm_spi_init);

static void __exit msm_spi_exit(void)
{
	platform_driver_unregister(&msm_spi_driver);
}
module_exit(msm_spi_exit);

/*
 * drivers/ide/s1r72v05.c
 *
 * Platform driver for Epson S1R72V05 IDE interface.
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/ide.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <mach/s1r72v05.h>

#define DRIVER_NAME "s1r72v05"

#define S1R72V05_HW_REVISION 0x50

/* parameters for changing power management states */
#define S1R72V05_PM_WAIT_USEC          50
#define S1R72V05_MAX_PM_WAIT_USEC      5000

/* chip config:
   bit 7: XINT active high
   bit 6: XINT 1/0 mode
   bit 5: XDREQ active low
   bit 4: XDACK active low
   bit 3: DMA0/1 is enabled by assertion of chip select and XDACK
   bit 2: endianness (even address is lower byte)
   bit 1-0: 16-bit BE mode
*/
#define S1R72V05_CHIPCONFIG_VALUE      0x0e

/* S1R72V05 IDE taskfile register identifiers */
#define S1R72V05_DATA_TF_REG           0x0
#define S1R72V05_ERROR_TF_REG          0x1
#define S1R72V05_FEATURES_TF_REG       0x1
#define S1R72V05_SECTOR_CNT_TF_REG     0x2
#define S1R72V05_LBA_LOW_TF_REG        0x3
#define S1R72V05_LBA_MID_TF_REG        0x4
#define S1R72V05_LBA_HIGH_TF_REG       0x5
#define S1R72V05_DEVICE_TF_REG         0x6
#define S1R72V05_STATUS_TF_REG         0x7
#define S1R72V05_COMMAND_TF_REG        0x7
#define S1R72V05_DEVICE_CONTROL_TF_REG 0xE
#define S1R72V05_ALT_STATUS_TF_REG     0xE
#define S1R72V05_UNSUPPORTED           -1

#define S1R72V05_TF_WRITE              0x80
#define S1R72V05_TF_READ               0x40

/* S1R72V05 register map (byte offsets from start of chip select) */
#define S1R72V05_MAININTSTAT_OFFSET             	0x00
#define   S1R72V05_MAININTSTAT_FINISHEDPM       	0x01
#define   S1R72V05_MAININTSTAT_MEDIAFIFO_INTSTAT      	0x08
#define   S1R72V05_MAININTSTAT_IDE_INTSTAT      	0x10
#define   S1R72V05_MAININTSTAT_ALLINTS          	0xF9
#define S1R72V05_DEVICEINTSTAT_OFFSET           	0x01
#define S1R72V05_HOSTINTSTAT_OFFSET             	0x02
#define S1R72V05_CPUINTSTAT_OFFSET              	0x03
#define S1R72V05_IDE_INTSTAT_OFFSET             	0x04
#define   S1R72V05_IDE_INTSTAT_DETECTINTRQ      	0x02
#define   S1R72V05_IDE_INTSTAT_IDE_CMP			0x04
#define   S1R72V05_IDE_INTSTAT_ALLINTS          	0xF7
#define S1R72V05_MEDIAFIFO_INTSTAT_OFFSET		0x05
#define   S1R72V04_MEDIAFIFO_INTSTAT_EMPTY		0x01
#define   S1R72V04_MEDIAFIFO_INTSTAT_FULL		0x02
#define   S1R72V04_MEDIAFIFO_INTSTAT_NOTEMPTY		0x04
#define   S1R72V04_MEDIAFIFO_INTSTAT_CMP		0x40
#define   S1R72V04_MEDIAFIFO_INTSTAT_ALLINTS		0x47
#define S1R72V05_MAININTENB_OFFSET              	0x10
#define   S1R72V05_MAININTENB_ENMEDIAFIFO_INTSTAT	0x08
#define   S1R72V05_MAININTENB_ENIDE_INTSTAT     	0x10
#define S1R72V05_IDE_INTENB_OFFSET              	0x14
#define   S1R72V05_IDE_INTENB_ENDETECTINTRQ     	0x02
#define   S1R72V05_IDE_INTENB_ENIDE_CMP		     	0x04
#define S1R72V05_MEDIAFIFOINTENB_OFFSET			0x15
#define   S1R72V05_MEDIAFIFOINTENB_EMPTY		0x01
#define   S1R72V05_MEDIAFIFOINTENB_NOTEMPTY		0x04
#define   S1R72V05_MEDIAFIFOINTENB_CMP			0x40
#define S1R72V05_REVISIONNUM_OFFSET             	0x20
#define S1R72V05_CHIPRESET_SWAPPED_OFFSET       	0x20
#define S1R72V05_CHIPRESET_OFFSET               	0x21
#define S1R72V05_REVISIONNUM_SWAPPED_OFFSET     	0x21
#define S1R72V05_PM_CONTROL_0_OFFSET            	0x22
#define   S1R72V05_PM_CONTROL_0_GOACTIVE60      	0x20
#define   S1R72V05_PM_CONTROL_0_GOSLEEP		      	0x80
#define S1R72V05_PM_CONTROL_1_OFFSET            	0x23
#define S1R72V05_FIFO_RD_0_OFFSET			0x30
#define S1R72V05_FIFO_RD_1_OFFSET			0x31
#define S1R72V05_FIFO_WR_0_OFFSET			0x32
#define S1R72V05_FIFO_WR_1_OFFSET			0x33
#define S1R72V05_FIFO_RDREMAIN_L_OFFSET			0x34
#define S1R72V05_FIFO_RDREMAIN_H_OFFSET			0x35
#define   S1R72V05_FIFO_RDREMAIN_VALID                  0x8000
#define S1R72V05_FIFO_WRREMAIN_L_OFFSET			0x36
#define S1R72V05_FIFO_WRREMAIN_H_OFFSET			0x37
#define S1R72V05_MEDIAFIFO_CONTROL_OFFSET		0x48
#define   S1R72V05_MEDIAFIFO_CONTROL_FIFOCLR    	0x01
#define S1R72V05_CLRALLMEDIAFIFO_JOIN_OFFSET		0x49
#define   S1R72V05_CLRALLMEDIAFIFO_JOIN_ALLBITS         0x8f
#define S1R72V05_MEDIAFIFO_JOIN_OFFSET			0x4A
#define   S1R72V05_MEDIAFIFO_JOINCPU_WR			0x01
#define   S1R72V05_MEDIAFIFO_JOINCPU_RD			0x02
#define   S1R72V05_MEDIAFIFO_JOIN_IDE			0x80
#define S1R72V05_IDE_STATUS_OFFSET              	0x90
#define S1R72V05_IDE_CONTROL_OFFSET             	0x91
#define   S1R72V05_IDE_CONTROL_IDE_GO           	0x01
#define   S1R72V05_IDE_CONTROL_DIR_FIFO_TO_IDE		0x08
#define   S1R72V05_IDE_CONTROL_IDE_CLR          	0x40
#define S1R72V05_IDE_CONFIG_0_OFFSET            	0x92
#define   S1R72V05_IDE_CONFIG_0_DMA			0x01
#define   S1R72V05_IDE_CONFIG_0_ULTRA			0x02
#define   S1R72V05_IDE_CONFIG_0_IDE_BUSRESET    	0x80
#define S1R72V05_IDE_CONFIG_1_OFFSET            	0x93
#define   S1R72V05_IDE_CONFIG_1_SWAP            	0x04
#define   S1R72V05_IDE_CONFIG_1_ACTIVEIDE       	0x80
#define S1R72V05_IDE_RMOD_OFFSET                	0x94
#define   S1R72V05_IDE_RMOD_MAX_TIMING          	0xFF
#define S1R72V05_IDE_TMOD_OFFSET                	0x95
#define S1R72V05_IDE_COUNT_H_OFFSET             	0x9C
#define S1R72V05_IDE_COUNT_M_OFFSET             	0x9F
#define S1R72V05_IDE_COUNT_L_OFFSET             	0x9E
#define S1R72V05_IDE_REGADRS_OFFSET             	0xA0
#define S1R72V05_IDE_RDREGVALUE_0_OFFSET        	0xA2
#define S1R72V05_IDE_RDREGVALUE_1_OFFSET        	0xA3
#define S1R72V05_IDE_WRREGVALUE_0_OFFSET        	0xA4
#define S1R72V05_IDE_WRREGVALUE_1_OFFSET        	0xA5
#define S1R72V05_CHIPCONFIG_SWAPPED_OFFSET      	0xB6
#define S1R72V05_CHIPCONFIG_OFFSET              	0xB7
#define S1R72V05_CPU_CHGENDIAN_SWAPPED_OFFSET   	0xB8
#define S1R72V05_CPU_CHGENDIAN_OFFSET           	0xB9

#define S1R72V05_REG_ADDR(reg) \
  (s1r72v05_data.virt_io_addr + S1R72V05_##reg##_OFFSET)

/* Uncomment to turn on output of taskfile register accesses. */
/* #define S1R72V05_TF_DEBUG */

#ifdef S1R72V05_TF_DEBUG
#define S1R72V05_TF_DBG dev_dbg
#else
#define S1R72V05_TF_DBG(...)
#endif

#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
static void s1r72v05_fifo_work_f(struct work_struct *work);
DECLARE_WORK(fifo_work, s1r72v05_fifo_work_f);

#define DMA_TIMER_EXPIRATION (2*WAIT_CMD)

#define S1R72V05_RDREMAIN_VALIDITY_MAX_CNT 20
#endif

static struct {
	void __iomem *virt_io_addr;
	ide_hwif_t *hwif;
	unsigned int hwif_idx;
	struct resource *io_res;
	struct resource *irq_res;
	struct platform_device *plat_dev;
	struct ide_host *host;
#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
	u8 dma;
#endif
} s1r72v05_data;

static inline void s1r72v05_read_tfreg(unsigned long port)
{
	unsigned long timeout;

	iowrite8(S1R72V05_TF_READ | port,
		 S1R72V05_REG_ADDR(IDE_REGADRS));
	timeout = jiffies + 2;
	while ((ioread8(S1R72V05_REG_ADDR(IDE_REGADRS)) &
		S1R72V05_TF_READ) && time_before(jiffies, timeout))
		cpu_relax();
}

static inline void s1r72v05_write_tfreg(unsigned long port)
{
	unsigned long timeout;

	iowrite8(S1R72V05_TF_WRITE | port, S1R72V05_REG_ADDR(IDE_REGADRS));
	timeout = jiffies + 2;
	while ((ioread8(S1R72V05_REG_ADDR(IDE_REGADRS)) &
		S1R72V05_TF_WRITE) && time_before(jiffies, timeout))
		cpu_relax();
}

static u8 s1r72v05_inb(unsigned long port)
{
	u8 retval;

	s1r72v05_read_tfreg(port);
	retval = ioread8(S1R72V05_REG_ADDR(IDE_RDREGVALUE_0));

	S1R72V05_TF_DBG(&s1r72v05_data.plat_dev->dev,
			"s1r72v05_inb(%#lx) = %#x \n", port, retval);

	return retval;
}

static u16 s1r72v05_inw(unsigned long port)
{
	S1R72V05_TF_DBG(&s1r72v05_data.plat_dev->dev,
			"s1r72v05_inw(%#lx) was called\n", port);

	s1r72v05_read_tfreg(port);

	return ioread16(S1R72V05_REG_ADDR(IDE_RDREGVALUE_0));
}

static void s1r72v05_insw(unsigned long port, void *addr, u32 count)
{
	u16 *to = addr;

	S1R72V05_TF_DBG(&s1r72v05_data.plat_dev->dev,
			"s1r72v05_insw(%#lx, %p, %#x) was called\n",
			port, addr, count);

	for (; count; count--) {
		s1r72v05_read_tfreg(port);
		*(to++) = ioread16(S1R72V05_REG_ADDR(IDE_RDREGVALUE_0));
	}
}

static void s1r72v05_outb(u8 val, unsigned long port)
{
	S1R72V05_TF_DBG(&s1r72v05_data.plat_dev->dev,
			"s1r72v05_outb(%#x, %#lx) was called\n", val, port);

	iowrite16(val, S1R72V05_REG_ADDR(IDE_WRREGVALUE_0));
	s1r72v05_write_tfreg(port);
}

static void s1r72v05_outw(u16 val, unsigned long port)
{
	S1R72V05_TF_DBG(&s1r72v05_data.plat_dev->dev,
			"s1r72v05_outw(%#x, %#lx) was called\n", val, port);

	iowrite16(val, S1R72V05_REG_ADDR(IDE_WRREGVALUE_0));
	s1r72v05_write_tfreg(port);
}

static void s1r72v05_outsw(unsigned long port, void *addr, u32 count)
{
	u16 *from = addr;

	S1R72V05_TF_DBG(&s1r72v05_data.plat_dev->dev,
			"s1r72v05_outsw(%#lx, %p, %#x) was called\n",
			port, addr, count);

	for (; count; count--) {
		iowrite16(*from++, S1R72V05_REG_ADDR(IDE_WRREGVALUE_0));
		s1r72v05_write_tfreg(port);
	}
}

static u8 s1r72v05_read_status(ide_hwif_t *hwif)
{
	return s1r72v05_inb(hwif->io_ports.status_addr);
}

static void s1r72v05_exec_command(ide_hwif_t *hwif, u8 cmd)
{
	s1r72v05_outb(cmd, hwif->io_ports.command_addr);
}

static u8 s1r72v05_read_altstatus(ide_hwif_t *hwif)
{
	return s1r72v05_inb(hwif->io_ports.ctl_addr);
}

static void s1r72v05_set_irq(ide_hwif_t *hwif, int on)
{
	u8 ctl = ATA_DEVCTL_OBS;

	if (on == 4) { /* hack for SRST */
		ctl |= 4;
		on &= ~4;
	}

	ctl |= on ? 0 : 2;
	s1r72v05_outb(ctl, hwif->io_ports.ctl_addr);
}

static void s1r72v05_tf_load(ide_drive_t *drive, ide_task_t *task)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	struct ide_taskfile *tf = &task->tf;
	u8 HIHI = (task->tf_flags & IDE_TFLAG_LBA48) ? 0xE0 : 0xEF;

	if (task->tf_flags & IDE_TFLAG_FLAGGED)
		HIHI = 0xFF;

	if (task->tf_flags & IDE_TFLAG_OUT_DATA) {
		u16 data = (tf->hob_data << 8) | tf->data;
		s1r72v05_outw(data, io_ports->data_addr);
	}

	if (task->tf_flags & IDE_TFLAG_OUT_HOB_FEATURE)
		s1r72v05_outb(tf->hob_feature, io_ports->feature_addr);
	if (task->tf_flags & IDE_TFLAG_OUT_HOB_NSECT)
		s1r72v05_outb(tf->hob_nsect, io_ports->nsect_addr);
	if (task->tf_flags & IDE_TFLAG_OUT_HOB_LBAL)
		s1r72v05_outb(tf->hob_lbal, io_ports->lbal_addr);
	if (task->tf_flags & IDE_TFLAG_OUT_HOB_LBAM)
		s1r72v05_outb(tf->hob_lbam, io_ports->lbam_addr);
	if (task->tf_flags & IDE_TFLAG_OUT_HOB_LBAH)
		s1r72v05_outb(tf->hob_lbah, io_ports->lbah_addr);

	if (task->tf_flags & IDE_TFLAG_OUT_FEATURE)
		s1r72v05_outb(tf->feature, io_ports->feature_addr);
	if (task->tf_flags & IDE_TFLAG_OUT_NSECT)
		s1r72v05_outb(tf->nsect, io_ports->nsect_addr);
	if (task->tf_flags & IDE_TFLAG_OUT_LBAL)
		s1r72v05_outb(tf->lbal, io_ports->lbal_addr);
	if (task->tf_flags & IDE_TFLAG_OUT_LBAM)
		s1r72v05_outb(tf->lbam, io_ports->lbam_addr);
	if (task->tf_flags & IDE_TFLAG_OUT_LBAH)
		s1r72v05_outb(tf->lbah, io_ports->lbah_addr);

	if (task->tf_flags & IDE_TFLAG_OUT_DEVICE)
		s1r72v05_outb((tf->device & HIHI) | drive->select,
			 io_ports->device_addr);
}

static void s1r72v05_tf_read(ide_drive_t *drive, ide_task_t *task)
{
	ide_hwif_t *hwif = drive->hwif;
	struct ide_io_ports *io_ports = &hwif->io_ports;
	struct ide_taskfile *tf = &task->tf;

	if (task->tf_flags & IDE_TFLAG_IN_DATA) {
		u16 data = s1r72v05_inw(io_ports->data_addr);

		tf->data = data & 0xff;
		tf->hob_data = (data >> 8) & 0xff;
	}

	/* be sure we're looking at the low order bits */
	s1r72v05_outb(ATA_DEVCTL_OBS & ~0x80, io_ports->ctl_addr);

	if (task->tf_flags & IDE_TFLAG_IN_FEATURE)
		tf->feature = s1r72v05_inb(io_ports->feature_addr);
	if (task->tf_flags & IDE_TFLAG_IN_NSECT)
		tf->nsect  = s1r72v05_inb(io_ports->nsect_addr);
	if (task->tf_flags & IDE_TFLAG_IN_LBAL)
		tf->lbal   = s1r72v05_inb(io_ports->lbal_addr);
	if (task->tf_flags & IDE_TFLAG_IN_LBAM)
		tf->lbam   = s1r72v05_inb(io_ports->lbam_addr);
	if (task->tf_flags & IDE_TFLAG_IN_LBAH)
		tf->lbah   = s1r72v05_inb(io_ports->lbah_addr);
	if (task->tf_flags & IDE_TFLAG_IN_DEVICE)
		tf->device = s1r72v05_inb(io_ports->device_addr);

	if (task->tf_flags & IDE_TFLAG_LBA48) {
		s1r72v05_outb(ATA_DEVCTL_OBS | 0x80, io_ports->ctl_addr);

		if (task->tf_flags & IDE_TFLAG_IN_HOB_FEATURE)
			tf->hob_feature = s1r72v05_inb(io_ports->feature_addr);
		if (task->tf_flags & IDE_TFLAG_IN_HOB_NSECT)
			tf->hob_nsect   = s1r72v05_inb(io_ports->nsect_addr);
		if (task->tf_flags & IDE_TFLAG_IN_HOB_LBAL)
			tf->hob_lbal    = s1r72v05_inb(io_ports->lbal_addr);
		if (task->tf_flags & IDE_TFLAG_IN_HOB_LBAM)
			tf->hob_lbam    = s1r72v05_inb(io_ports->lbam_addr);
		if (task->tf_flags & IDE_TFLAG_IN_HOB_LBAH)
			tf->hob_lbah    = s1r72v05_inb(io_ports->lbah_addr);
	}
}

static void s1r72v05_input_data(ide_drive_t *drive, struct request *rq,
		void *buf, unsigned int len)
{
	unsigned long data_addr = drive->hwif->io_ports.data_addr;

	s1r72v05_insw(data_addr, buf, (len+1) / 2);
}

static void s1r72v05_output_data(ide_drive_t *drive, struct request *rq,
		void *buf, unsigned int len)
{
	unsigned long data_addr = drive->hwif->io_ports.data_addr;

	s1r72v05_outsw(data_addr, buf, len / 2);
}

static void s1r72v05_set_pio_mode(ide_drive_t *drive, const u8 pio)
{
	u8 reg;

	/* Register values taken from s1r72v05 technical manual */
	switch (pio+XFER_PIO_0) {
	case XFER_PIO_0:
		reg = 0xFF;
		break;
	case XFER_PIO_1:
		reg = 0x88;
		break;
	case XFER_PIO_2:
		reg = 0x44;
		break;
	case XFER_PIO_3:
		reg = 0x22;
		break;
	case XFER_PIO_4:
		reg = 0x10;
		break;
	default:
		printk(KERN_WARNING "%s: Unexpected PIO mode: %d\n",
		       DRIVER_NAME, pio);
		return;
	}
	iowrite8(reg, S1R72V05_REG_ADDR(IDE_TMOD));
}

#ifdef DEBUG
static void s1r72v05_register_dump(void)
{
	int i;

	dev_dbg(&s1r72v05_data.plat_dev->dev, "begin register dump\n");
	dev_dbg(&s1r72v05_data.plat_dev->dev, "offset value|offset value\n");
	for (i = 0; i < resource_size(s1r72v05_data.io_res); i += 2) {
		dev_dbg(&s1r72v05_data.plat_dev->dev,
			"%#.2x: %#.2x | %#.2x: %#.2x\n",
			i, ioread8(s1r72v05_data.virt_io_addr + i),
			i + 1, ioread8(s1r72v05_data.virt_io_addr + i + 1));
	}
	dev_dbg(&s1r72v05_data.plat_dev->dev, "end register dump\n");
}
#endif

static int s1r72v05_ack_intr(struct hwif_s *hwif)
{
	unsigned char int_status;
	unsigned char sub_status;

	if (hwif != s1r72v05_data.hwif)
		return 0;

	/* Is this the correct type of interrupt? */
	int_status = ioread8(S1R72V05_REG_ADDR(MAININTSTAT));
#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
	if (((int_status & S1R72V05_MAININTSTAT_IDE_INTSTAT) == 0) &&
	    ((int_status & S1R72V05_MAININTSTAT_MEDIAFIFO_INTSTAT) == 0)) {
#else
	if (((int_status & S1R72V05_MAININTSTAT_IDE_INTSTAT) == 0)) {
#endif
		printk(KERN_ALERT "%s: unexpected interrupt!\n", DRIVER_NAME);
		printk(KERN_ALERT "%s: 0x0:%#.2x 0x1:%#.2x 0x2:%#.2x "
		       "0x3:%#.2x 0x4:%#.2x 0x5:%#.2x\n",
		       DRIVER_NAME, int_status,
		       ioread8(S1R72V05_REG_ADDR(DEVICEINTSTAT)),
		       ioread8(S1R72V05_REG_ADDR(HOSTINTSTAT)),
		       ioread8(S1R72V05_REG_ADDR(CPUINTSTAT)),
		       ioread8(S1R72V05_REG_ADDR(IDE_INTSTAT)),
		       ioread8(S1R72V05_REG_ADDR(MEDIAFIFO_INTSTAT)));
		return 0;
	}

	if ((int_status & S1R72V05_MAININTSTAT_IDE_INTSTAT) != 0) {
		/* Is this the right kind of IDE interrupt? */
		sub_status = ioread8(S1R72V05_REG_ADDR(IDE_INTSTAT));
#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
		if (!(sub_status & S1R72V05_IDE_INTSTAT_DETECTINTRQ) &&
		    !(sub_status & S1R72V05_IDE_INTSTAT_IDE_CMP)) {
#else
		if (!(sub_status & S1R72V05_IDE_INTSTAT_DETECTINTRQ)) {
#endif
			printk(KERN_ALERT
			       "%s: IDE interrupt for wrong reason: %#x!\n",
				       DRIVER_NAME, sub_status);
			return 0;
		}
	}

#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
	/* Clear IDE interrupt status. */
	iowrite8(S1R72V05_IDE_INTSTAT_DETECTINTRQ |
		    S1R72V05_IDE_INTSTAT_IDE_CMP,
		 S1R72V05_REG_ADDR(IDE_INTSTAT));
#else
	/* Clear IDE interrupt status. */
	iowrite8(S1R72V05_IDE_INTSTAT_DETECTINTRQ,
		 S1R72V05_REG_ADDR(IDE_INTSTAT));
#endif

	/* Make sure interrupt status is clear. */
	if (ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) &
				S1R72V05_MAININTSTAT_IDE_INTSTAT)
		printk(KERN_ALERT
		       "%s: interrupt status did not clear!\n", DRIVER_NAME);

	/* Note:  MediaFIFO status is cleared in the DMA processing */

	return 1;
}

#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
static void s1r72v05_dma_host_set(ide_drive_t *drive, int on)
{
	u8 reg = ioread8(S1R72V05_REG_ADDR(IDE_CONFIG_0));

	if (on) {
			printk(KERN_INFO "%s DMA enabled\n", DRIVER_NAME);
			iowrite8(reg | S1R72V05_IDE_CONFIG_0_DMA,
				 S1R72V05_REG_ADDR(IDE_CONFIG_0));
	} else {
			printk(KERN_INFO "%s DMA disabled\n", DRIVER_NAME);
			iowrite8(reg & ~S1R72V05_IDE_CONFIG_0_DMA,
				 S1R72V05_REG_ADDR(IDE_CONFIG_0));
	}
}

static void s1r72v05_set_dma_mode(ide_drive_t *drive, const u8 speed)
{
	u8 reg;

	/* values taken from s1r72v05 technical manual */
	switch (speed) {
	case XFER_MW_DMA_0:
		reg = 0xBB;
		break;
	case XFER_MW_DMA_1:
		reg = 0x20;
		break;
	case XFER_MW_DMA_2:
		reg = 0x10;
		break;
	default:
		printk(KERN_WARNING "%s: Unexpected DMA mode: %d\n",
		       DRIVER_NAME, speed);
		return;
	}
	iowrite8(reg, S1R72V05_REG_ADDR(IDE_TMOD));

	reg = ioread8(S1R72V05_REG_ADDR(IDE_CONFIG_0));
	iowrite8(reg & ~S1R72V05_IDE_CONFIG_0_ULTRA,
		 S1R72V05_REG_ADDR(IDE_CONFIG_0));
}

static int s1r72v05_init_dma(ide_hwif_t *hwif, const struct ide_port_info *d)
{
	return 1;
}

static int s1r72v05_dma_setup(ide_drive_t *drive)
{
	struct request *rq = drive->hwif->rq;
	u8 read = (rq_data_dir(rq) == READ);
	u8 cpu_bit;
	u8 fifo_int_bit;
	u32 ide_count;

	ide_init_sg_cmd(drive, rq);
	ide_map_sg(drive, rq);

	iowrite8(S1R72V05_MEDIAFIFO_CONTROL_FIFOCLR,
		 S1R72V05_REG_ADDR(MEDIAFIFO_CONTROL));

	if (read) {
		cpu_bit = S1R72V05_MEDIAFIFO_JOINCPU_RD;
		fifo_int_bit = S1R72V05_MEDIAFIFOINTENB_NOTEMPTY;
	} else {
		cpu_bit = S1R72V05_MEDIAFIFO_JOINCPU_WR;
		fifo_int_bit = S1R72V05_MEDIAFIFOINTENB_EMPTY;
	}
	iowrite8(S1R72V05_MEDIAFIFO_JOIN_IDE | cpu_bit,
		 S1R72V05_REG_ADDR(MEDIAFIFO_JOIN));

	ide_count = rq->nr_sectors * SECTOR_SIZE;
	iowrite8((u8)((ide_count & 0x00ff0000) >> 16),
		 S1R72V05_REG_ADDR(IDE_COUNT_H));
	iowrite8((u8)((ide_count & 0x0000ff00) >> 8),
		 S1R72V05_REG_ADDR(IDE_COUNT_M));
	iowrite8((u8)((ide_count & 0x000000ff) >> 0),
		 S1R72V05_REG_ADDR(IDE_COUNT_L));

	drive->waiting_for_dma = 1;

	return 0;
}

static void s1r72v05_write_fifo(u16 *buf, int bytes)
{
	u16 count;
	u16 i;

	count = bytes/2;
	while (count > 0) {
		/* chip spec requires 1 clock delay between writing to FIFO
		 * and reading WRREMAIN registers, so we read revision register
		 */
		if (ioread8(S1R72V05_REG_ADDR(REVISIONNUM)) !=
						S1R72V05_HW_REVISION)
			printk(KERN_INFO "%s: revision mismatch!\n",
			       DRIVER_NAME);
		i = ioread16(S1R72V05_REG_ADDR(FIFO_WRREMAIN_L));
		if ((count * 2) < i)
			i = count;
		else
			i = i / 2;
		count -= i;
		for (; i > 0; i--)
			iowrite16(*buf++, S1R72V05_REG_ADDR(FIFO_WR_0));
	}
}

static void s1r72v05_read_fifo(u16 *buf, int bytes)
{
	u16 count;
	u16 i;

	count = bytes/2;
	while (count > 0) {
		i = ioread16(S1R72V05_REG_ADDR(FIFO_RDREMAIN_L));
		if (i & S1R72V05_FIFO_RDREMAIN_VALID) {
			i = i & ~S1R72V05_FIFO_RDREMAIN_VALID;
			if ((count * 2) < i)
				i = count;
			else
				i = i / 2;
			count -= i;
			for (; i > 0; i--)
				*buf++ = ioread16(S1R72V05_REG_ADDR(FIFO_RD_0));
		}
	}

}

static void s1r72v05_fifo_work_f(struct work_struct *work)
{
	ide_hwif_t *hwif = s1r72v05_data.hwif;
	u8 read = (rq_data_dir(s1r72v05_data.hwif->rq) == READ);
	struct scatterlist *curr_sg;
	struct page *page;
	u16 offset;
	u16 *buf;
	unsigned int sectors = hwif->nleft;


	if (read)
		iowrite8(S1R72V05_IDE_CONTROL_IDE_GO &
			   ~S1R72V05_IDE_CONTROL_DIR_FIFO_TO_IDE,
			 S1R72V05_REG_ADDR(IDE_CONTROL));
	else
		iowrite8(S1R72V05_IDE_CONTROL_IDE_GO |
			   S1R72V05_IDE_CONTROL_DIR_FIFO_TO_IDE,
			 S1R72V05_REG_ADDR(IDE_CONTROL));

	while ((sectors > 0) && (s1r72v05_data.dma)) {
		if (hwif->cursg)
			curr_sg = hwif->cursg;
		else
			curr_sg = hwif->cursg = hwif->sg_table;
		page = sg_page(curr_sg);
		offset = curr_sg->offset + hwif->cursg_ofs * SECTOR_SIZE;
		page = nth_page(page, (offset >> PAGE_SHIFT));
		offset %= PAGE_SIZE;

		buf = kmap_atomic(page, KM_BIO_SRC_IRQ) + offset;
		if (!buf) {
			printk(KERN_ALERT "%s: could not map page\n",
			       DRIVER_NAME);
			break;
		}
		hwif->cursg_ofs++;
		hwif->nleft--;

		if ((hwif->cursg_ofs * SECTOR_SIZE) == curr_sg->length) {
			hwif->cursg = sg_next(hwif->cursg);
			hwif->cursg_ofs = 0;
		}

		if (read)
			s1r72v05_read_fifo(buf, SECTOR_SIZE);
		else
			s1r72v05_write_fifo(buf, SECTOR_SIZE);
		sectors--;

		kunmap_atomic(buf, KM_BIO_SRC_IRQ);
	}
}

static void s1r72v05_dma_start(ide_drive_t *drive)
{
	s1r72v05_data.dma = 1;
	wmb();
	schedule_work(&fifo_work);
}

static ide_startstop_t s1r72v05_ide_dma_intr(ide_drive_t *drive)
{
	ide_hwif_t *hwif = drive->hwif;
	u8 read = (rq_data_dir(hwif->rq) == READ);
	u8 inten;
	u16 fifobytes;
	int count = 0;

	if (read) {
		do {
			fifobytes =
				ioread16(S1R72V05_REG_ADDR(FIFO_RDREMAIN_L));
		} while (((fifobytes & S1R72V05_FIFO_RDREMAIN_VALID) == 0) &&
			 (count++ < S1R72V05_RDREMAIN_VALIDITY_MAX_CNT));

		if (count >= S1R72V05_RDREMAIN_VALIDITY_MAX_CNT)
			printk(KERN_ALERT "%s: FIFO RDREMAIN invalid!\n",
			       DRIVER_NAME);

		if ((fifobytes & ~S1R72V05_FIFO_RDREMAIN_VALID) != 0) {
			/* IDE transaction complete, but FIFO not empty,
			 * so turn on interrupt for when FIFO is emtpy */
			ide_set_handler(drive,
					s1r72v05_ide_dma_intr,
					WAIT_CMD,
					NULL);
			iowrite8(S1R72V04_MEDIAFIFO_INTSTAT_EMPTY,
				 S1R72V05_REG_ADDR(MEDIAFIFO_INTSTAT));
			iowrite8(S1R72V05_MEDIAFIFOINTENB_EMPTY,
				 S1R72V05_REG_ADDR(MEDIAFIFOINTENB));
			iowrite8(ioread8(S1R72V05_REG_ADDR(MAININTENB)) |
				   S1R72V05_MAININTENB_ENMEDIAFIFO_INTSTAT,
				  S1R72V05_REG_ADDR(MAININTENB));
			return ide_started;
		} else {
			iowrite8(S1R72V04_MEDIAFIFO_INTSTAT_ALLINTS,
				 S1R72V05_REG_ADDR(MEDIAFIFO_INTSTAT));
			iowrite8(0, S1R72V05_REG_ADDR(MEDIAFIFOINTENB));
			iowrite8(ioread8(S1R72V05_REG_ADDR(MAININTENB)) &
				   ~S1R72V05_MAININTENB_ENMEDIAFIFO_INTSTAT,
				  S1R72V05_REG_ADDR(MAININTENB));
		}
	}

	drive->waiting_for_dma = 0;
	s1r72v05_data.dma = 0;
	wmb();

	iowrite8(S1R72V05_CLRALLMEDIAFIFO_JOIN_ALLBITS,
		 S1R72V05_REG_ADDR(CLRALLMEDIAFIFO_JOIN));
	inten = ioread8(S1R72V05_REG_ADDR(IDE_INTENB));
	iowrite8(inten & ~S1R72V05_IDE_INTENB_ENIDE_CMP,
		 S1R72V05_REG_ADDR(IDE_INTENB));

	return ide_dma_intr(drive);
}

static void s1r72v05_dma_exec_cmd(ide_drive_t *drive, u8 command)
{
	ide_execute_command(drive, command, s1r72v05_ide_dma_intr,
			    DMA_TIMER_EXPIRATION, NULL);
}

static int s1r72v05_dma_end(ide_drive_t *drive)
{

	return 0;
}

static int s1r72v05_dma_test_irq(ide_drive_t *drive)
{
	return 1;
}

void s1r72v05_dma_lost_irq(ide_drive_t *drive)
{
}
#endif /* CONFIG_IDE_S1R72V05_USE_FIFO */

static const struct ide_tp_ops s1r72v05_tp_ops = {
	.exec_command		= s1r72v05_exec_command,
	.read_status		= s1r72v05_read_status,
	.read_altstatus		= s1r72v05_read_altstatus,

	.set_irq		= s1r72v05_set_irq,

	.tf_load		= s1r72v05_tf_load,
	.tf_read		= s1r72v05_tf_read,

	.input_data		= s1r72v05_input_data,
	.output_data		= s1r72v05_output_data,
};

#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
static const struct ide_dma_ops s1r72v05_ide_dma_ops __devinitdata = {
	.dma_host_set		= s1r72v05_dma_host_set,
	.dma_setup		= s1r72v05_dma_setup,
	.dma_exec_cmd		= s1r72v05_dma_exec_cmd,
	.dma_start		= s1r72v05_dma_start,
	.dma_end		= s1r72v05_dma_end,
	.dma_test_irq		= s1r72v05_dma_test_irq,
	.dma_lost_irq		= s1r72v05_dma_lost_irq,
};
#endif

static const struct ide_port_ops s1r72v05_ide_port_ops __devinitdata = {
	.set_pio_mode		= s1r72v05_set_pio_mode,
#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
	.set_dma_mode		= s1r72v05_set_dma_mode,
#endif
};

static const struct ide_port_info s1r72v05_port_info __devinitdata = {
	.name			= DRIVER_NAME,
	.chipset		= ide_unknown,
	.tp_ops			= &s1r72v05_tp_ops,
	.pio_mask		= ATA_PIO4,
#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
	.port_ops		= &s1r72v05_ide_port_ops,
	.init_dma		= s1r72v05_init_dma,
	.dma_ops		= &s1r72v05_ide_dma_ops,
	.swdma_mask		= 0,
	.mwdma_mask		= ATA_MWDMA2,
	.udma_mask		= 0,
#endif
};

static int __devinit s1r72v05_probe(struct platform_device *plat_dev)
{
	struct s1r72v05_platform_data *pdata;
	unsigned int num_sleeps;
	hw_regs_t hw, *hws[] = { &hw, NULL, NULL, NULL };
	struct ide_host *host;
	int ret = 0;

	dev_dbg(&plat_dev->dev, "s1r72v05_probe() was called\n");

	if (plat_dev->dev.platform_data != NULL) {
		pdata = plat_dev->dev.platform_data;
		if (pdata->gpio_setup != NULL) {
			ret = pdata->gpio_setup();
			if (ret)
				return ret;
		}
	}

#ifdef CONFIG_IDE_S1R72V05_USE_FIFO
	s1r72v05_data.dma = 0;
#endif
	s1r72v05_data.plat_dev = plat_dev;

	s1r72v05_data.io_res = platform_get_resource(plat_dev, IORESOURCE_MEM,
						     0);
	if (!s1r72v05_data.io_res) {
		printk(KERN_ALERT
		       "%s: could not get IORESOURCE_MEM\n", DRIVER_NAME);
		return -ENODEV;
	}

	s1r72v05_data.irq_res = platform_get_resource(plat_dev,
						      IORESOURCE_IRQ, 0);
	if (!s1r72v05_data.irq_res) {
		printk(KERN_ALERT "%s: could not get IORESOURCE_IRQ\n",
		       DRIVER_NAME);
		return -ENODEV;
	}

	if (!request_mem_region(s1r72v05_data.io_res->start,
			resource_size(s1r72v05_data.io_res), DRIVER_NAME)) {
		printk(KERN_ALERT
		       "%s: could not allocate io region\n", DRIVER_NAME);
		return -EBUSY;
	}

	s1r72v05_data.virt_io_addr = ioremap(s1r72v05_data.io_res->start,
					resource_size(s1r72v05_data.io_res));
	if (!s1r72v05_data.virt_io_addr) {
		printk(KERN_ALERT
		       "%s: could not ioremap io region\n", DRIVER_NAME);
		ret = -ENOMEM;
		goto release_mem_region;
	}

	/* Reset chip. We don't know if the endianness is wrong, so
	   write to both bytes. The other one is the revision register
	   which is read-only (so this won't break anything).
	 */
	iowrite8(0x1, S1R72V05_REG_ADDR(CHIPRESET));
	mdelay(1);
	iowrite8(0x1, S1R72V05_REG_ADDR(CHIPRESET_SWAPPED));
	mdelay(1);

	/* Before we fix the endianness, there's a strange issue where reading
	   the revision register can return 0 unless you read the reset
	   register first.
	 */
	if (ioread8(S1R72V05_REG_ADDR(REVISIONNUM)) != S1R72V05_HW_REVISION) {
		dev_dbg(&plat_dev->dev,
			"revision register not detected, assuming "
			"endianness is reversed\n");
		/* Make sure revision is showing up in its byte-swapped
		   position.
		 */
		if (ioread8(S1R72V05_REG_ADDR(REVISIONNUM_SWAPPED)) !=
		    S1R72V05_HW_REVISION) {
			printk(KERN_ALERT
			       "%s: could not read revision register before "
			       "setting endianness\n", DRIVER_NAME);
			goto release_iomap;
		}

		/* Initialize chip config register and fix endianness. */
		iowrite8(S1R72V05_CHIPCONFIG_VALUE,
			 S1R72V05_REG_ADDR(CHIPCONFIG_SWAPPED));
		ioread8(S1R72V05_REG_ADDR(CPU_CHGENDIAN_SWAPPED));
	} else {
		dev_dbg(&plat_dev->dev,
			"revision register detected, assuming "
			"endianness is correct\n");
		/* Initialize chip config register. */
		iowrite8(S1R72V05_CHIPCONFIG_VALUE,
			 S1R72V05_REG_ADDR(CHIPCONFIG));
	}

	/* Check revision, it should show up correctly now. */
	if (ioread8(S1R72V05_REG_ADDR(REVISIONNUM)) != S1R72V05_HW_REVISION) {
		printk(KERN_ALERT "%s: could not read revision register "
		       "after setting endianness\n", DRIVER_NAME);
		goto release_iomap;
	}

	/* The chip turns on in sleep mode. Go into Active60. */
	iowrite8(S1R72V05_MAININTSTAT_FINISHEDPM,
		 S1R72V05_REG_ADDR(MAININTSTAT));
	iowrite8(S1R72V05_PM_CONTROL_0_GOACTIVE60,
		 S1R72V05_REG_ADDR(PM_CONTROL_0));

	/* Wait for transition to Active60 state. */
	num_sleeps = 0;
	do {
		udelay(S1R72V05_PM_WAIT_USEC);
		num_sleeps += S1R72V05_PM_WAIT_USEC;
	} while (!(ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) &
		   S1R72V05_MAININTSTAT_FINISHEDPM) &&
		 num_sleeps < S1R72V05_MAX_PM_WAIT_USEC);
	if (!(ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) &
	      S1R72V05_MAININTSTAT_FINISHEDPM)) {
		printk(KERN_ALERT "%s: could not switch to Active60 state\n",
		       DRIVER_NAME);
		goto release_iomap;
	}

	/* Turn on IDE interface in the chip. */
	iowrite8(S1R72V05_IDE_CONFIG_1_SWAP | S1R72V05_IDE_CONFIG_1_ACTIVEIDE,
		 S1R72V05_REG_ADDR(IDE_CONFIG_1));

	/* "Initialize the IDE circuit." */
	iowrite8(S1R72V05_IDE_CONTROL_IDE_CLR, S1R72V05_REG_ADDR(IDE_CONTROL));

	/* Set the assert pulse and negate pulse width to the max for now. */
	iowrite8(S1R72V05_IDE_RMOD_MAX_TIMING, S1R72V05_REG_ADDR(IDE_RMOD));

	/* Reset the IDE bus and set to PIO mode. */
	iowrite8(S1R72V05_IDE_CONFIG_0_IDE_BUSRESET,
		 S1R72V05_REG_ADDR(IDE_CONFIG_0));

	/* Clear any pending interrupts. */
	iowrite8(S1R72V05_MAININTSTAT_ALLINTS, S1R72V05_REG_ADDR(MAININTSTAT));
	iowrite8(S1R72V05_IDE_INTSTAT_ALLINTS, S1R72V05_REG_ADDR(IDE_INTSTAT));

	/* Enable IDE interrupts. */
	iowrite8(S1R72V05_MAININTENB_ENIDE_INTSTAT,
		 S1R72V05_REG_ADDR(MAININTENB));
	iowrite8(S1R72V05_IDE_INTENB_ENDETECTINTRQ,
		 S1R72V05_REG_ADDR(IDE_INTENB));

	dev_dbg(&plat_dev->dev, "chip initialization complete\n");

	/*
	 * Initialize linux IDE data structures and register interface.
	 */
	hw.io_ports.data_addr = S1R72V05_DATA_TF_REG;
	hw.io_ports.error_addr = S1R72V05_ERROR_TF_REG;
	hw.io_ports.nsect_addr = S1R72V05_SECTOR_CNT_TF_REG;
	hw.io_ports.lbal_addr = S1R72V05_LBA_LOW_TF_REG;
	hw.io_ports.lbam_addr = S1R72V05_LBA_MID_TF_REG;
	hw.io_ports.lbah_addr = S1R72V05_LBA_HIGH_TF_REG;
	hw.io_ports.device_addr = S1R72V05_DEVICE_TF_REG;
	hw.io_ports.status_addr = S1R72V05_STATUS_TF_REG;
	hw.io_ports.ctl_addr = S1R72V05_DEVICE_CONTROL_TF_REG;
	hw.io_ports.irq_addr = S1R72V05_UNSUPPORTED;
	hw.irq = s1r72v05_data.irq_res->start;
	hw.ack_intr = s1r72v05_ack_intr;
	hw.dev = &plat_dev->dev;

	host = ide_host_alloc(&s1r72v05_port_info, hws);
	if (host == NULL)
		goto release_iomap;

	s1r72v05_data.hwif = host->ports[0];
	s1r72v05_data.host = host;

	if (ide_host_register(host, &s1r72v05_port_info, hws))
		goto free_host;

	dev_dbg(&plat_dev->dev,
		"probe complete, interface registered\n");

	return 0;

free_host:
	ide_host_free(host);
	s1r72v05_data.hwif = NULL;

release_iomap:
	iounmap(s1r72v05_data.virt_io_addr);

release_mem_region:
	release_mem_region(s1r72v05_data.io_res->start,
			resource_size(s1r72v05_data.io_res));
	return ret;
}

static int __devexit s1r72v05_remove(struct platform_device *plat_dev)
{
	ide_host_remove(s1r72v05_data.host);
	iounmap(s1r72v05_data.virt_io_addr);
	release_mem_region(s1r72v05_data.io_res->start,
			resource_size(s1r72v05_data.io_res));

	return 0;
}

#ifdef CONFIG_PM
static int s1r72v05_suspend(struct platform_device *dev, pm_message_t state)
{
	u16 num_sleeps;

	/* disable interrupts */
	disable_irq(s1r72v05_data.irq_res->start);

	/* Set controller to sleep */
	/* NOTE:  Assumes the USB controller is inactive */
	iowrite8(ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) |
		    S1R72V05_MAININTSTAT_FINISHEDPM,
		 S1R72V05_REG_ADDR(MAININTSTAT));
	iowrite8(S1R72V05_PM_CONTROL_0_GOSLEEP,
		 S1R72V05_REG_ADDR(PM_CONTROL_0));

	/* Wait for transition to Active60 state. */
	num_sleeps = 0;
	do {
		udelay(S1R72V05_PM_WAIT_USEC);
		num_sleeps += S1R72V05_PM_WAIT_USEC;
	} while (!(ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) &
		   S1R72V05_MAININTSTAT_FINISHEDPM) &&
		 num_sleeps < S1R72V05_MAX_PM_WAIT_USEC);
	if (!(ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) &
	      S1R72V05_MAININTSTAT_FINISHEDPM)) {
		printk(KERN_ALERT "%s: could not switch to sleep state\n",
		       DRIVER_NAME);
	}

	/* clear the pending PM interrupt */
	iowrite8(S1R72V05_MAININTSTAT_FINISHEDPM,
		 S1R72V05_REG_ADDR(MAININTSTAT));

	return 0;
}

static int s1r72v05_resume(struct platform_device *dev)
{
	u16 num_sleeps;

	/* Set controller to Active60 */
	/* NOTE:  Assumes the USB controller is inactive */
	iowrite8(ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) |
		    S1R72V05_MAININTSTAT_FINISHEDPM,
		 S1R72V05_REG_ADDR(MAININTSTAT));
	iowrite8(S1R72V05_PM_CONTROL_0_GOACTIVE60,
		 S1R72V05_REG_ADDR(PM_CONTROL_0));

	/* Wait for transition to Active60 state. */
	num_sleeps = 0;
	do {
		udelay(S1R72V05_PM_WAIT_USEC);
		num_sleeps += S1R72V05_PM_WAIT_USEC;
	} while (!(ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) &
		   S1R72V05_MAININTSTAT_FINISHEDPM) &&
		 num_sleeps < S1R72V05_MAX_PM_WAIT_USEC);
	if (!(ioread8(S1R72V05_REG_ADDR(MAININTSTAT)) &
	      S1R72V05_MAININTSTAT_FINISHEDPM)) {
		printk(KERN_ALERT "%s: could not switch to sleep state\n",
		       DRIVER_NAME);
	}

	/* clear the pending PM interrupt */
	iowrite8(S1R72V05_MAININTSTAT_FINISHEDPM,
		 S1R72V05_REG_ADDR(MAININTSTAT));

	/* enable interrupts */
	enable_irq(s1r72v05_data.irq_res->start);

	return 0;
}
#else
#define s1r72v05_suspend NULL
#define s1r72v05_resume  NULL
#endif

static struct platform_driver s1r72v05_driver = {
	.probe = s1r72v05_probe,
	.remove = __devexit_p(s1r72v05_remove),
	.suspend = s1r72v05_suspend,
	.resume = s1r72v05_resume,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
	},
};

static int __init s1r72v05_init(void)
{
	return platform_driver_register(&s1r72v05_driver);
}

static void __exit s1r72v05_exit(void)
{
	platform_driver_unregister(&s1r72v05_driver);
}

module_init(s1r72v05_init);
module_exit(s1r72v05_exit);

MODULE_DESCRIPTION("Epson S1R72V05 PATA interface driver");
MODULE_VERSION("1.1");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("ide:" DRIVER_NAME);

/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 * Qualcomm Tavarua FM core driver
 */

/* driver definitions */
#define DRIVER_AUTHOR "Qualcomm"
#define DRIVER_NAME "radio-tavarua"
#define DRIVER_CARD "Qualcomm FM Radio Transceiver"
#define DRIVER_DESC "I2C radio driver for Qualcomm FM Radio Transceiver "
#define DRIVER_VERSION "1.0.0"

#include <linux/version.h>
#include <linux/init.h>         /* Initdata                     */
#include <linux/delay.h>        /* udelay                       */
#include <linux/uaccess.h>      /* copy to/from user            */
#include <linux/kfifo.h>        /* lock free circular buffer    */
#include <linux/param.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

/* kernel includes */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/mutex.h>
#include <media/v4l2-common.h>
#include <media/rds.h>
#include <asm/unaligned.h>
#include <media/v4l2-ioctl.h>
#include <linux/unistd.h>

#include <media/tavarua.h>
#include <linux/mfd/marimba.h>
#include <linux/platform_device.h>

struct tavarua_device {
	struct video_device *videodev;
	/* driver management */
	unsigned int users;
	/* top level driver data */
	struct marimba *marimba;
	struct device *dev;
	/* gpio managment */
	int irqpin;
	int (*irqpin_setup)(struct marimba_fm_platform_data *pdata);
	int (*irqpin_teardown)(struct marimba_fm_platform_data *pdata);
	struct marimba_fm_platform_data *pdata;
	/*RDS buffers + Radio event buffer*/
	struct kfifo *data_buf[TAVARUA_BUF_MAX];
	/* internal register status */
	unsigned char registers[RADIO_REGISTERS];
	/* radio standard */
	enum tavarua_zone zone;
	/* global lock */
	struct mutex lock;
	/* work queue */
	struct work_struct work;
	/* wait queue for blocking event read */
	wait_queue_head_t event_queue;
};

/**************************************************************************
 * Module Parameters
 **************************************************************************/

/* Radio Nr */
static int radio_nr = -1;
module_param(radio_nr, int, 0);
MODULE_PARM_DESC(radio_nr, "Radio Nr");

/* RDS buffer blocks */
static unsigned int rds_buf = 100;
module_param(rds_buf, uint, 0);
MODULE_PARM_DESC(rds_buf, "RDS buffer entries: *100*");

/* forward declerations */
static int tavarua_start(struct tavarua_device *radio,
			enum radio_state_t state);
static int tavarua_request_irq(struct tavarua_device *radio);
/* work function */
static void read_int_stat(struct work_struct *work);

static irqreturn_t tavarua_isr(int irq, void *dev_id)
{
	struct tavarua_device *radio = dev_id;
	/* schedule a tasklet to handle host intr */
	schedule_work(&radio->work);
	return IRQ_HANDLED;
}

/**************************************************************************
 * Interface to radio internal registers over top level marimba driver
 *************************************************************************/
static int tavarua_read_registers(struct tavarua_device *radio,
				unsigned char offset, int len)
{
	radio->marimba->mod_id = MARIMBA_SLAVE_ID_FM;
	return marimba_read(radio->marimba, offset,
				&radio->registers[offset], len);

}
/*
 * tavarua_write_register - Writes the value a register
 */
static int tavarua_write_register(struct tavarua_device *radio,
			unsigned char offset, unsigned char value)
{
	int retval;
	radio->marimba->mod_id = MARIMBA_SLAVE_ID_FM;
	retval = marimba_write(radio->marimba, offset, &value, 1);
	if (retval > 0) {
		if (offset <= RADIO_REGISTERS)
			radio->registers[offset] = value;
	}
	return retval;
}

static int tavarua_write_registers(struct tavarua_device *radio,
			unsigned char offset, unsigned char *buf, int len)
{

	int i;
	int retval;
	radio->marimba->mod_id = MARIMBA_SLAVE_ID_FM;
	retval = marimba_write(radio->marimba, offset, buf, len);
	if (retval > 0) { /* if write successful, update internal state too */
		for (i = 0; i < len; i++) {
			if ((offset+i) <= RADIO_REGISTERS)
				radio->registers[offset+i] = buf[i];
		}
	}
	return retval;
}

/*
 * queue event for user
 */
static void tavarua_q_event(struct tavarua_device *radio,
				enum tavarua_evt_t event)
{

	struct kfifo *data_b = radio->data_buf[TAVARUA_BUF_EVENTS];
	unsigned char evt = event;
	FMDBG("updating event_q with event %x\n", event);
	if (__kfifo_put(data_b, &evt, 1))
		wake_up_interruptible(&radio->event_queue);
}


/**************************************************************************
 * Scheduled event handler
 *************************************************************************/
static void read_int_stat(struct work_struct *work)
{
	struct tavarua_device *radio = container_of(work,
					struct tavarua_device, work);
	mutex_lock(&radio->lock);
	tavarua_read_registers(radio, STATUS_REG1, STATUS_REG_NUM);

	FMDBG("INTSTAT1 <%x> \n", radio->registers[STATUS_REG1]);
	FMDBG("INTSTAT2 <%x> \n", radio->registers[STATUS_REG2]);
	FMDBG("INTSTAT3 <%x> \n", radio->registers[STATUS_REG3]);

	if (radio->registers[STATUS_REG1] & READY)
		tavarua_q_event(radio, TAVARUA_EVT_RADIO_READY);

	/* Tune completed */
	if (radio->registers[STATUS_REG1] & TUNE)
		tavarua_q_event(radio, TAVARUA_EVT_TUNE_SUCC);

	/* Search completed (read FREQ) */
	if (radio->registers[STATUS_REG1] & SEARCH)
		tavarua_q_event(radio, TAVARUA_EVT_SEEK_COMPLETE);

	/* Scanning for next station */
	if (radio->registers[STATUS_REG1] & SCANNEXT)
		tavarua_q_event(radio, TAVARUA_EVT_SCAN_NEXT);

	/* Signal indicator change (read SIGSTATE) */
	if (radio->registers[STATUS_REG1] & SIGNAL)
		tavarua_q_event(radio, TAVARUA_EVT_SIGNAL);

	/* RDS synchronization state change (read RDSSYNC) */
	if (radio->registers[STATUS_REG1] & SYNC)
		tavarua_q_event(radio, TAVARUA_EVT_SYNC);

	/* Audio Control indicator (read AUDIOIND) */
	if (radio->registers[STATUS_REG1] & AUDIO)
		tavarua_q_event(radio, TAVARUA_EVT_AUDIO);

	mutex_unlock(&radio->lock);
	FMDBG("handler is done\n");

}

/*************************************************************************
 * irq helper functions
 ************************************************************************/

/*
 * tavarua_request_irq
 */
static int tavarua_request_irq(struct tavarua_device *radio)
{
	int retval;
	int irq = radio->irqpin;
	if (radio == NULL)
		return -EINVAL;

	retval = request_irq(irq, tavarua_isr, IRQ_TYPE_EDGE_FALLING,
				"fm interrupt", radio);
	if (retval < 0) {
		FMDBG("Couldn't acquire FM gpio %d\n", irq);
		return retval;
	} else {
		FMDBG("FM GPIO %d registered \n", irq);
	}
	retval = enable_irq_wake(irq);
	if (retval < 0) {
		FMDBG("Could not enable FM interrupt\n ");
		free_irq(irq , radio);
	}
	return retval;
}

/*
 * tavarua_disable_irq
 */
static int tavarua_disable_irq(struct tavarua_device *radio)
{
	int irq;
	if (!radio)
		return -EINVAL;
	irq = radio->irqpin;
	disable_irq_wake(irq);
	flush_scheduled_work();
	free_irq(irq, radio);
	return 0;
}

/*************************************************************************
 * fops/IOCTL helper functions
 ************************************************************************/

/*
 * tavarua_get_freq - get the frequency
 */
static unsigned int tavarua_get_freq(struct tavarua_device *radio,
				struct v4l2_frequency *freq)
{
	int retval;
	unsigned short chan;
	unsigned int band_bottom = 87.5 * FREQ_MUL;
	unsigned int spacing = 0.100 * FREQ_MUL;

	switch (radio->zone) {
	/* 0: 200 kHz (USA, Australia) */
	default:
		spacing = 0.200 * FREQ_MUL;
		band_bottom = 87.5 * FREQ_MUL;
		break;
	/* 1: 100 kHz (Europe, Japan) */
	case TAVARUA_ZONE_1:
		spacing = 0.100 * FREQ_MUL;
		band_bottom = 76 * FREQ_MUL;
		break;
	/* 2:  50 kHz */
	case TAVARUA_ZONE_2:
		spacing = 0.050 * FREQ_MUL;
		band_bottom = 76 * FREQ_MUL;
		break;
	}

	/* read channel */
	retval = tavarua_read_registers(radio, FREQ, 2);
	chan = radio->registers[FREQ];

	/* Frequency (MHz) = Spacing (kHz) x Channel + Bottom of Band (MHz) */
	freq->frequency = chan * spacing + band_bottom;

	return retval;
}


static int tavarua_set_freq(struct tavarua_device *radio, unsigned int freq)
{
	unsigned int spacing = 0.100 * FREQ_MUL;
	unsigned int band_bottom = 87.5 * FREQ_MUL;
	unsigned char chan;
	unsigned char cmd[2];

	switch (radio->zone) {
	/* 0: 200 kHz (USA, Australia) */
	default:
		spacing = 0.200 * FREQ_MUL;
		band_bottom = 87.5 * FREQ_MUL;
		break;
	/* 1: 100 kHz (Europe, Japan) */
	case TAVARUA_ZONE_1:
		spacing = 0.100 * FREQ_MUL;
		band_bottom = 76 * FREQ_MUL;
		break;
	/* 2:  50 kHz */
	case TAVARUA_ZONE_2:
		spacing = 0.050 * FREQ_MUL;
		band_bottom = 76 * FREQ_MUL;
		break;
	}

	/* Chan = [ Freq (Mhz) - Bottom of Band (MHz) ] / Spacing (kHz) */
	chan = (freq - band_bottom) / spacing;

	cmd[0] = chan;
	cmd[1] = TUNE_STATION;
	return tavarua_write_registers(radio, FREQ, cmd, 2);
}

/**************************************************************************
 * File Operations Interface
 *************************************************************************/

/*
** tavarua_fops_open - file open
*/
static int tavarua_fops_open(struct file *file)
{
	struct tavarua_device *radio = video_get_drvdata(video_devdata(file));
	int retval;
	unsigned char value;
	/* FM core bring up */
	char buffer[] = {0x00, 0x48, 0x8A, 0x8E, 0x97, 0xB7};

	if (radio->users > 0)
		return -ENODEV;

	/* initial gpio pin config & Power up */
	retval = radio->irqpin_setup(radio->pdata);
	if (retval < 0) {
		printk(KERN_ERR "%s: failed config gpio & pmic\n", __func__);
		return retval;
	}
	/* enable irq */
	retval = tavarua_request_irq(radio);
	if (retval < 0) {
		printk(KERN_ERR "%s: failed to request irq\n", __func__);
		return retval;
	}
	/* call top level marimba interface here to enable FM core */
	radio->marimba->mod_id = MARIMBA_SLAVE_ID_MARIMBA;
	value = FM_ENABLE;
	retval = marimba_write_bit_mask(radio->marimba, MARIMBA_XO_BUFF_CNTRL,
							&value, 1, value);
	if (retval < 0) {
		printk(KERN_ERR "%s:XO_BUFF_CNTRL write failed \n", __func__);
		return retval;
	}

	/* Bring up FM core */
	radio->marimba->mod_id = MARIMBA_SLAVE_ID_FM;
	retval = tavarua_write_registers(radio, LEAKAGE_CNTRL, buffer, 6);
	if (retval < 0) {
		printk(KERN_ERR "%s: failed to bring up FM Core \n", __func__);
		return retval;
	}
	return 0;
}


/*
 * tavarua_fops_release - file release
 */
static int tavarua_fops_release(struct file *file)
{
	int retval;
	struct tavarua_device *radio = video_get_drvdata(video_devdata(file));
	unsigned char value;
	if (!radio)
		return -ENODEV;
	/* disable radio ctrl */
	retval = tavarua_write_register(radio, RDCTRL, 0x00);

	/* disable irq */
	retval = tavarua_disable_irq(radio);
	if (retval < 0) {
		printk(KERN_ERR "%s: failed to disable irq\n", __func__);
		return retval;
	}

	/* disable fm core */
	radio->marimba->mod_id = MARIMBA_SLAVE_ID_MARIMBA;
	value = 0x00;
	retval = marimba_write_bit_mask(radio->marimba, MARIMBA_XO_BUFF_CNTRL,
							&value, 1, FM_ENABLE);
	if (retval < 0) {
		printk(KERN_ERR "%s:XO_BUFF_CNTRL write failed \n", __func__);
		return retval;
	}
	/* teardown gpio and pmic */
	retval = radio->irqpin_teardown(radio->pdata);
	if (retval < 0) {
		printk(KERN_ERR "%s: failed to shutdown gpio&pmic\n", __func__);
		return retval;
	}

	return 0;
}

/*
 * tavarua_fops - file operations interface
 */
static const struct v4l2_file_operations tavarua_fops = {
	.owner = THIS_MODULE,
	.ioctl = video_ioctl2,
	.open  = tavarua_fops_open,
	.release = tavarua_fops_release,
};

/*************************************************************************
 * Video4Linux Interface
 *************************************************************************/

/*
 * tavarua_v4l2_queryctrl - query control
 */
static struct v4l2_queryctrl tavarua_v4l2_queryctrl[] = {
	{
		.id	       = V4L2_CID_AUDIO_VOLUME,
		.type	       = V4L2_CTRL_TYPE_INTEGER,
		.name	       = "Volume",
		.minimum       = 0,
		.maximum       = 15,
		.step	       = 1,
		.default_value = 15,
	},
	{
		.id	       = V4L2_CID_AUDIO_BALANCE,
		.flags	       = V4L2_CTRL_FLAG_DISABLED,
	},
	{
		.id	       = V4L2_CID_AUDIO_BASS,
		.flags	       = V4L2_CTRL_FLAG_DISABLED,
	},
	{
		.id	       = V4L2_CID_AUDIO_TREBLE,
		.flags	       = V4L2_CTRL_FLAG_DISABLED,
	},
	{
		.id	       = V4L2_CID_AUDIO_MUTE,
		.type	       = V4L2_CTRL_TYPE_BOOLEAN,
		.name	       = "Mute",
		.minimum       = 0,
		.maximum       = 1,
		.step	       = 1,
		.default_value = 1,
	},
	{
		.id	       = V4L2_CID_AUDIO_LOUDNESS,
		.flags	       = V4L2_CTRL_FLAG_DISABLED,
	},
	{
		.id	       = V4L2_CID_PRIVATE_TAVARUA_SRHCMODE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name	       = "freq search mode",
		.minimum       = 0,
		.maximum       = 7,
		.step	       = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_TAVARUA_SCANDWELL,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "freq search mode",
		.minimum       = 0,
		.maximum       = 7,
		.step          = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_PRIVATE_TAVARUA_SRCHON,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "Search on/off",
		.minimum       = 0,
		.maximum       = 1,
		.step          = 1,
		.default_value = 1,

	},
	{
		.id            = V4L2_CID_PRIVATE_TAVARUA_STATE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "radio rx/tx on/off",
		.minimum       = 0,
		.maximum       = 3,
		.step          = 1,
		.default_value = 1,

	},
	{
		.id            = V4L2_CID_PRIVATE_TAVARUA_ZONE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "radio standard",
		.minimum       = 0,
		.maximum       = 2,
		.step          = 1,
		.default_value = 0,
	},
};


/*
 * tavarua_vidioc_querycap - query device capabilities
 */
static int tavarua_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *capability)
{
	strlcpy(capability->driver, DRIVER_NAME, sizeof(capability->driver));
	strlcpy(capability->card, DRIVER_CARD, sizeof(capability->card));
	sprintf(capability->bus_info, "I2C");
	capability->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO;

	return 0;
}


/*
 * tavarua_vidioc_queryctrl - enumerate control items
 */
static int tavarua_vidioc_queryctrl(struct file *file, void *priv,
		struct v4l2_queryctrl *qc)
{
	unsigned char i;
	int retval = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(tavarua_v4l2_queryctrl); i++) {
		if (qc->id && qc->id == tavarua_v4l2_queryctrl[i].id) {
			memcpy(qc, &(tavarua_v4l2_queryctrl[i]), sizeof(*qc));
			retval = 0;
			break;
		}
	}
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": query conv4ltrol failed with %d\n", retval);

	return retval;
}


/*
 * tavarua_vidioc_g_ctrl - get the value of a control
 */
static int tavarua_vidioc_g_ctrl(struct file *file, void *priv,
		struct v4l2_control *ctrl)
{
	struct tavarua_device *radio = video_get_drvdata(video_devdata(file));
	int retval = 0;

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_VOLUME:
		break;
	case V4L2_CID_AUDIO_MUTE:
		break;
	case V4L2_CID_PRIVATE_TAVARUA_SRHCMODE:
		ctrl->value = radio->registers[SRCHCTRL] & SRCH_MODE;
		break;
	case V4L2_CID_PRIVATE_TAVARUA_SCANDWELL:
		ctrl->value = (radio->registers[SRCHCTRL] & SCAN_DWELL) >> 4;
		break;
	case V4L2_CID_PRIVATE_TAVARUA_SRCHON:
		ctrl->value = (radio->registers[SRCHCTRL] & SRCH_ON) >> 7 ;
		break;
	case V4L2_CID_PRIVATE_TAVARUA_STATE:
		ctrl->value = (radio->registers[RDCTRL] & 0x03);
		break;
	default:
		retval = -EINVAL;
	}
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
		": get control failed with %d, id: %d\n", retval, ctrl->id);

	return retval;
}


/*
 * tavarua_vidioc_s_ctrl - set the value of a control
 */
static int tavarua_vidioc_s_ctrl(struct file *file, void *priv,
		struct v4l2_control *ctrl)
{
	struct tavarua_device *radio = video_get_drvdata(video_devdata(file));
	int retval = 0;
	unsigned char value;
	char buffer[] = {0x00, 0x00, 0x00};

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_VOLUME:
		break;
	case V4L2_CID_AUDIO_MUTE:
		value = (radio->registers[IOCTRL] & ~IOC_HRD_MUTE) |
							ctrl->value;
		retval = tavarua_write_register(radio, IOCTRL, value);
		break;
	case V4L2_CID_PRIVATE_TAVARUA_SRHCMODE:
		value = (radio->registers[SRCHCTRL] & ~SRCH_MODE) |
							ctrl->value;
		radio->registers[SRCHCTRL] = value;
		break;
	case V4L2_CID_PRIVATE_TAVARUA_SCANDWELL:
		value = (radio->registers[SRCHCTRL] & ~SCAN_DWELL) |
						(ctrl->value << 4);
		radio->registers[SRCHCTRL] = value;
		break;
	/* start/stop search */
	case V4L2_CID_PRIVATE_TAVARUA_SRCHON:
		value = ((radio->registers[SRCHCTRL] & ~SRCH_ON)) |
						(ctrl->value << 7);
		FMDBG("starting search with: %x\n", value);
		buffer[2] = value;
		retval = tavarua_write_registers(radio, SRCHRDS1, buffer, 3);
		break;
	case V4L2_CID_PRIVATE_TAVARUA_STATE:
		/* check if already on */
		if ((ctrl->value == 1) && !(radio->registers[RDCTRL] &
							FM_RECV)) {
			FMDBG("turning on...\n");
			retval = tavarua_start(radio, FM_RECV);
		}
		/* check if off */
		else if ((ctrl->value == 0) && radio->registers[RDCTRL]) {
			FMDBG("turning off...\n");
			retval = tavarua_write_register(radio, RDCTRL,
							ctrl->value);
		} else if ((ctrl->value == 2) && ((radio->registers[RDCTRL] &
							0x03) != 0x02)) {
			FMDBG("transmit mode\n");
			retval = tavarua_start(radio, 2);
		}
		break;
	default:
	  retval = -EINVAL;
	}
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
		": set control failed with %d, id : %d\n", retval, ctrl->id);

	return retval;
}


/*
 * tavarua_vidioc_g_tuner - get tuner attributes
 */
static int tavarua_vidioc_g_tuner(struct file *file, void *priv,
		struct v4l2_tuner *tuner)
{
	struct tavarua_device *radio = video_get_drvdata(video_devdata(file));
	int retval;

	if (tuner->index > 0)
		return -EINVAL;

	/* read status rssi */
	retval = tavarua_read_registers(radio, IOCTRL, 1);
	if (retval < 0)
		return retval;

	strcpy(tuner->name, "FM");
	tuner->type = V4L2_TUNER_RADIO;
	switch (radio->zone) {
	/* 0: 87.5 - 108 MHz (USA, Europe, default) */
	case 0:
		tuner->rangelow  =  87.5 * FREQ_MUL;
		tuner->rangehigh = 108   * FREQ_MUL;
		break;
	/* 1: 76   - 108 MHz (Japan wide band) */
	case 1:
		tuner->rangelow  =  76   * FREQ_MUL;
		tuner->rangehigh = 108   * FREQ_MUL;
		break;
	/* 2: 76   -  90 MHz (Japan) */
	default:
		tuner->rangelow  =  76   * FREQ_MUL;
		tuner->rangehigh =  90   * FREQ_MUL;
		break;
	};
	tuner->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
	tuner->capability = V4L2_TUNER_CAP_LOW;

	/* Stereo indicator == Stereo (instead of Mono) */
	if (radio->registers[IOCTRL] & IOC_MON_STR)
		tuner->audmode = V4L2_TUNER_MODE_STEREO;
	else
	  tuner->audmode = V4L2_TUNER_MODE_MONO;

	/* automatic frequency control: -1: freq to low, 1 freq to high */
	tuner->afc = 0;

	return 0;
}


/*
 * tavarua_vidioc_s_tuner - set tuner attributes
 * Used to set mono/stereo mode
 */
static int tavarua_vidioc_s_tuner(struct file *file, void *priv,
		struct v4l2_tuner *tuner)
{
	struct tavarua_device *radio = video_get_drvdata(video_devdata(file));
	int retval;
	int audmode;
	if (tuner->index > 0)
		return -EINVAL;

	if (tuner->audmode == V4L2_TUNER_MODE_MONO)
		/* Mono */
		audmode = (radio->registers[IOCTRL] | IOC_MON_STR);
	else
		/* Stereo */
		audmode = (radio->registers[IOCTRL] & ~IOC_MON_STR);

	retval = tavarua_write_register(radio, IOCTRL, audmode);
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": set tuner failed with %d\n", retval);

	return retval;
}


/*
 * tavarua_vidioc_g_frequency - get tuner or modulator radio frequency
 */
static int tavarua_vidioc_g_frequency(struct file *file, void *priv,
		struct v4l2_frequency *freq)
{
	struct tavarua_device *radio = video_get_drvdata(video_devdata(file));

	freq->type = V4L2_TUNER_RADIO;
	return tavarua_get_freq(radio, freq);

}


/*
 * tavarua_vidioc_s_frequency - set tuner or modulator radio frequency
 */
static int tavarua_vidioc_s_frequency(struct file *file, void *priv,
					struct v4l2_frequency *freq)
{
	struct tavarua_device *radio = video_get_drvdata(video_devdata(file));
	int retval;
	FMDBG("%s\n", __func__);

	if (freq->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	retval = tavarua_set_freq(radio, freq->frequency);
	if (retval < 0)
		printk(KERN_WARNING DRIVER_NAME
			": set frequency failed with %d\n", retval);
	msleep(500);
	return retval;
}


/*
 * Our main buffer function, in essense its equivalent to a blocking read call
 */
static int tavarua_vidioc_dqbuf(struct file *file, void *priv,
				struct v4l2_buffer *buffer)
{

	struct tavarua_device  *radio = video_get_drvdata(video_devdata(file));
	enum tavarua_buf_t buf_type = buffer->index;
	struct kfifo *data_fifo;
	unsigned char *buf = (unsigned char *)buffer->m.userptr;
	unsigned int len = buffer->length;
	FMDBG("%s: requesting buffer %d\n", __func__, buf_type);
	/* check if we can access the user buffer */
	if (!access_ok(VERIFY_WRITE, buf, len))
		return -EFAULT;
	if ((buf_type < TAVARUA_BUF_MAX) && (buf_type >= 0)) {
		data_fifo = radio->data_buf[buf_type];
		if (buf_type == TAVARUA_BUF_EVENTS) {
			if (wait_event_interruptible(radio->event_queue,
			__kfifo_len(data_fifo)) < 0) {
				return -EINTR;
			}
		}
	} else {
		FMDBG("invalid buffer type\n");
		return -1;
	}
	buffer->bytesused = __kfifo_get(data_fifo, buf, len);

	return 0;
}

/*
 * This function is here to make the v4l2 framework happy.
 * We cannot use private buffers without it
 */
static int tavarua_vidioc_try_fmt_type_private(struct file *file, void *priv,
						struct v4l2_format *f)
{
	return 0;

}

static int tavarua_vidioc_s_hw_freq_seek(struct file *file, void *priv,
					struct v4l2_hw_freq_seek *seek)
{
	struct tavarua_device  *radio = video_get_drvdata(video_devdata(file));
	int mode = radio->registers[SRCHCTRL] & SRCH_MODE;
	int dwell = radio->registers[SRCHCTRL] & SCAN_DWELL >> 4;
	unsigned char srch;

	FMDBG("starting seek in mode:%d with dwell of:%d\n", mode, dwell);
	srch =  radio->registers[SRCHCTRL] | SRCH_ON | (seek->seek_upward << 3);
	return tavarua_write_register(radio, SRCHCTRL, srch);
}

/*
 * tavarua_viddev_tamples - video device interface
 */
static const struct v4l2_ioctl_ops tavarua_ioctl_ops = {
	.vidioc_querycap              = tavarua_vidioc_querycap,
	.vidioc_queryctrl             = tavarua_vidioc_queryctrl,
	.vidioc_g_ctrl                = tavarua_vidioc_g_ctrl,
	.vidioc_s_ctrl                = tavarua_vidioc_s_ctrl,
	.vidioc_g_tuner               = tavarua_vidioc_g_tuner,
	.vidioc_s_tuner               = tavarua_vidioc_s_tuner,
	.vidioc_g_frequency           = tavarua_vidioc_g_frequency,
	.vidioc_s_frequency           = tavarua_vidioc_s_frequency,
	.vidioc_s_hw_freq_seek        = tavarua_vidioc_s_hw_freq_seek,
	.vidioc_dqbuf                 = tavarua_vidioc_dqbuf,
	.vidioc_try_fmt_type_private  = tavarua_vidioc_try_fmt_type_private,
};

static struct video_device tavarua_viddev_template = {
	.fops                   = &tavarua_fops,
	.ioctl_ops              = &tavarua_ioctl_ops,
	.name                   = DRIVER_NAME,
	.release                = video_device_release,
};


static int tavarua_start(struct tavarua_device *radio,
				enum radio_state_t state)
{

	int retval;
	char int_init[3];
	int_init[0] = (READY | TUNE | SEARCH | SCANNEXT | SIGNAL |
						INTF | SYNC | AUDIO);
	int_init[1] = (RDSRT | RDSPS);
	int_init[2] = (TRANSFER | ERROR);

	FMDBG("%s <%d>\n", __func__, state);

	/* set geographic zone */
	radio->zone = TAVARUA_ZONE_0;

	/* set radio mode */
	retval = tavarua_write_register(radio, RDCTRL, state);
	if (retval < 0)
		return retval;

	/* enable interrupts */
	retval = tavarua_write_register(radio, STATUS_REG1, READY | TUNE
			| SEARCH | SCANNEXT | SIGNAL | INTF | SYNC | AUDIO);
	if (retval < 0)
		return retval;

	if (state == FM_RECV) {
		retval = tavarua_write_register(radio, STATUS_REG2, RDSRT);
	} else {
		retval = tavarua_write_register(radio, STATUS_REG2, RDSRT |
						TXRDSDAT | TXRDSDONE);
	}
	if (retval < 0)
		return retval;

	retval = tavarua_write_register(radio, STATUS_REG3, TRANSFER | ERROR);
	if (retval < 0)
		return retval;

	if (state == FM_RECV) {
		retval = tavarua_write_register(radio, ADVCTRL, RDSRTEN);
		if (retval < 0)
			return retval;
	}

	return 0;
}


static int tavarua_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tavarua_device *radio = platform_get_drvdata(pdev);
	int retval = 0;
	if (!device_may_wakeup(radio->dev))
		retval = disable_irq_wake(radio->irqpin);
	return retval;
}

static int tavarua_resume(struct platform_device *pdev)
{
	struct tavarua_device *radio = platform_get_drvdata(pdev);
	int retval = 0;
	if (!device_may_wakeup(radio->dev))
		retval = enable_irq_wake(radio->irqpin);
	return retval;
}

static int  __init tavarua_probe(struct platform_device *pdev)
{

	struct marimba_fm_platform_data *tavarua_pdata;
	struct tavarua_device *radio;
	unsigned int buf_size;
	int retval;
	int i;
	FMDBG("%s: probe called\n", __func__);
	/* private data allocation */
	radio = kzalloc(sizeof(struct tavarua_device), GFP_KERNEL);
	if (!radio) {
		retval = -ENOMEM;
	goto err_initial;
	}

	radio->marimba = platform_get_drvdata(pdev);
	tavarua_pdata = pdev->dev.platform_data;
	radio->irqpin        = tavarua_pdata->gpioirq;
	radio->irqpin_setup  = tavarua_pdata->gpio_setup;
	radio->irqpin_teardown  = tavarua_pdata->gpio_shutdown;
	radio->pdata = tavarua_pdata;
	radio->dev = &pdev->dev;
	platform_set_drvdata(pdev, radio);

	device_init_wakeup(radio->dev, 0);
	/* video device allocation */
	radio->videodev = video_device_alloc();
	if (!radio->videodev)
		goto err_radio;

	/* initial configuration */
	memcpy(radio->videodev, &tavarua_viddev_template,
	  sizeof(tavarua_viddev_template));

	/*allocate internal buffers for decoded rds and event buffer*/
	buf_size = 64;
	for (i = 0; i < TAVARUA_BUF_MAX; i++) {
		if (i == TAVARUA_BUF_RAW_RDS)
			radio->data_buf[i] = kfifo_alloc(rds_buf*3,
						GFP_KERNEL, NULL);
		else
			radio->data_buf[i] = kfifo_alloc(buf_size,
						GFP_KERNEL, NULL);

		if (radio->data_buf[i] < 0)
			goto err_bufs;
	}

	radio->users = 0;
	/* init lock */
	mutex_init(&radio->lock);
	/* initialize wait queue for event read */
	init_waitqueue_head(&radio->event_queue);

	video_set_drvdata(radio->videodev, radio);
	INIT_WORK(&radio->work, read_int_stat);

	/* register video device */
	if (video_register_device(radio->videodev, VFL_TYPE_RADIO, radio_nr)) {
		printk(KERN_WARNING DRIVER_NAME
				": Could not register video device\n");
		goto err_all;
	}

	return 0;

err_all:
	video_device_release(radio->videodev);
err_bufs:
	for (; i > -1; i--)
		kfifo_free(radio->data_buf[i]);
err_radio:
	kfree(radio);
err_initial:
	return retval;
}

static int __devexit tavarua_remove(struct platform_device *pdev)
{
	int i;
	struct tavarua_device *radio = platform_get_drvdata(pdev);

	/* disable irq */
	tavarua_disable_irq(radio);

	video_unregister_device(radio->videodev);

	/* free internal buffers */
	for (i = 0; i < TAVARUA_BUF_MAX; i++)
		kfifo_free(radio->data_buf[i]);

	radio->irqpin_teardown(radio->pdata);

	/* free state struct */
	kfree(radio);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver tavarua_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "marimba_fm",
	},
	.probe = tavarua_probe,
	.remove = __devexit_p(tavarua_remove),
	.suspend = tavarua_suspend,
	.resume = tavarua_resume,
};


/*************************************************************************
 * Module Interface
 ************************************************************************/

static int __init radio_module_init(void)
{
	printk(KERN_INFO DRIVER_DESC ", Version " DRIVER_VERSION "\n");
	return platform_driver_register(&tavarua_driver);
}


/*
 * tavarua_module_exit - module exit
 */
static void __exit radio_module_exit(void)
{
  platform_driver_unregister(&tavarua_driver);
}

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);

module_init(radio_module_init);
module_exit(radio_module_exit);

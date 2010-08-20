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
 *
 *  Driver for QWERTY keyboard with I/O communications via
 *  the I2C Interface. The keyboard hardware is a reference design supporting
 *  the standard XT/PS2 scan codes (sets 1&2).
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/msm_i2ckbd.h>

#define i2ckybd_name "msm-i2ckbd"

static int __devinit qi2ckybd_probe(struct i2c_client *client,
				    const struct i2c_device_id *id);

/* constants relating to keyboard i2c transactions */
enum qkbd_protocol {
	QKBD_REG_CNTRL  = 0,		/* control register offset */
	QKBD_REG_STATUS = 1,		/* status register offset  */
	QKBD_CMD_RESET  = 0xFF,		/* keyboard reset command  */
	QKBD_CMD_ENABLE = 0xF4,		/* keyboard enable command */
	QKBD_CMD_SREP   = 0xF3,		/* set keyboard autorepeat */
	QKBD_CMD_SLSET  = 0xF0,		/* select scan set         */
	QKBD_CMD_READID = 0xF2,		/* read keyboard id info   */
	QKBD_CMD_SLED   = 0xED,		/* set/reset status leds   */
	QKBD_RSP_ACK    = 0xFA		/* keyboard acknowledge    */
};

/* constants relating to events sent into the input core */
enum kbd_inevents {
	QKBD_IN_KEYPRESS        = 1,
	QKBD_IN_KEYRELEASE      = 0,
	QKBD_IN_MXKYEVTS        = 256
};

/* special keyboard key codes */
enum kbd_codes {
	QKBD_KCODE_PREF1	= 0xE0,	/* extended code prefix */
	QKBD_KCODE_PREF2	= 0xE1,	/* extended code prefix */
	QKBD_KCODE_PRTPFX1	= 0x2A,	/* set 1 PrtScr prefix  */
	QKBD_KCODE_PRTPFX2	= 0xB7,	/* set 1 PrtScr prefix  */
	QKBD_KCODE_BRKPFX	= 0xF0,	/* set 2 break prefix   */
	QKBD_KCODE_OVRFL	= 0xFF,	/* overflow error       */
	QKBD_KCODE_INTERR	= 0xFC, /* keybd internal error */
	QKBD_KCODE_F7		= 0x83  /* F7 = special case    */

};

/* keyboard status register bit definitions */
enum kbd_statusbits {
	QKBD_SRBIT_CMDIP	= 1<<0,	/* cmd in progress      */
	QKBD_SRBIT_KDATA	= 1<<1,	/* key code available   */
	QKBD_SRBIT_RSPRDY	= 1<<2,	/* response ready       */
	QKBD_SRBIT_SCANEN	= 1<<3,	/* keybd enabled        */
	QKBD_SRBIT_SSET		= 1<<4	/* scan set             */
};

/* default constants */
enum kbd_defaults {
	QKBD_TYPO_DELAY		= 1000,
	QKBD_TYPO_PERIOD	= 50
};

enum kbd_constants {
	QKBD_PHYSLEN		= 128,
	QCVENDOR_ID		= 0x5143
};

/*
 * The i2ckybd_record structure consolates all the data/variables
 * specific to managing the single instance of the keyboard.
 */
struct i2ckybd_record {
	struct i2c_client *mykeyboard;
	struct input_dev *i2ckbd_idev;
	int	product_info;
	char	physinfo[QKBD_PHYSLEN];
	int     mclrpin;
	int     irqpin;
	int     (*extpin_setup) (void);
	void    (*extpin_teardown)(void);

	void (*i2c_hw_reset)(int);

	uint8_t cmd;
	uint8_t noargs;
	uint8_t cargs[2];
	uint8_t scanset;
	uint8_t kybd_exists;
	uint8_t kybd_connected;
	uint8_t prefix;
	uint8_t pfxstk[2];
	uint8_t kcnt;
	uint8_t evcnt;
	uint8_t hwrep_enabled;
	struct delayed_work kb_cmdq;
	struct work_struct qkybd_irqwork;
	u32 (*xlf)(struct i2ckybd_record *kbdrec, s32 code,
		   s32 *kstate, s32 *i2cerr);
	unsigned char xltable[QKBD_IN_MXKYEVTS];
};

#define KBDIRQNO(kbdrec)  (MSM_GPIO_TO_INT(kbdrec->irqpin))

/* translation table for scan set 2 */
static uint8_t qi2ckybd_set2dec[QKBD_IN_MXKYEVTS] = {
   KEY_RESERVED,	 KEY_F9,	 KEY_F7,	 KEY_F5,  /* 0 */
	 KEY_F3,	 KEY_F1,	 KEY_F2,	KEY_F12,  /* 4 */
	KEY_F13,	KEY_F10,	 KEY_F8,	 KEY_F6,  /* 8 */
	 KEY_F4,	KEY_TAB,      KEY_GRAVE,    KEY_KPEQUAL,  /* C */
	KEY_F14,    KEY_LEFTALT,  KEY_LEFTSHIFT,	      0,  /* 10 */
   KEY_LEFTCTRL,	  KEY_Q,	  KEY_1,	      0,  /* 14 */
	KEY_F15,	      0,	  KEY_Z,	  KEY_S,  /* 18 */
	  KEY_A,	  KEY_W,	  KEY_2,	      0,  /* 1C */
	KEY_F16,	  KEY_C,	  KEY_X,	  KEY_D,  /* 20 */
	  KEY_E,	  KEY_4,	  KEY_3,	      0,  /* 24 */
	KEY_F17,      KEY_SPACE,	  KEY_V,	  KEY_F,  /* 28 */
	  KEY_T,	  KEY_R,	  KEY_5,	      0,  /* 2C */
	KEY_F18,	  KEY_N,	  KEY_B,	  KEY_H,  /* 30 */
	  KEY_G,	  KEY_Y,	  KEY_6,	      0,  /* 34 */
	KEY_F19,	      0,	  KEY_M,	  KEY_J,  /* 38 */
	  KEY_U,	  KEY_7,	  KEY_8,	      0,  /* 3C */
	KEY_F20,      KEY_COMMA,	  KEY_K,	  KEY_I,  /* 40 */
	  KEY_O,	  KEY_0,	  KEY_9,	      0,  /* 44 */
	KEY_F21,	KEY_DOT,      KEY_SLASH,	  KEY_L,  /* 48 */
  KEY_SEMICOLON,	  KEY_P,      KEY_MINUS,	      0,  /* 4C */
	KEY_F22,	 KEY_RO, KEY_APOSTROPHE,	      0,  /* 50 */
  KEY_LEFTBRACE,      KEY_EQUAL,	      0,	KEY_F23,  /* 54 */
   KEY_CAPSLOCK, KEY_RIGHTSHIFT,      KEY_ENTER, KEY_RIGHTBRACE,  /* 58 */
	      0,  KEY_BACKSLASH,	      0,	KEY_F24,  /* 5C */
	      0,      KEY_102ND,   KEY_HIRAGANA,   KEY_KATAKANA,  /* 60 */
     KEY_HENKAN,	      0,  KEY_BACKSPACE,   KEY_MUHENKAN,  /* 64 */
	      0,	KEY_KP1,	KEY_YEN,	KEY_KP4,  /* 68 */
	KEY_KP7,	      0,	      0,	      0,  /* 6C */
	KEY_KP0,      KEY_KPDOT,	KEY_KP2,	KEY_KP5,  /* 70 */
	KEY_KP6,	KEY_KP8,	KEY_ESC,    KEY_NUMLOCK,  /* 74 */
	KEY_F11,     KEY_KPPLUS,	KEY_KP3,    KEY_KPMINUS,  /* 78 */
 KEY_KPASTERISK,	KEY_KP9, KEY_SCROLLLOCK,	      0,

	      0,	      0,	      0,	      0,  /* 80 */
	      0,	      0,	      0,	      0,  /* 84 */
	      0,	      0,	      0,	      0,  /* 88 */
	      0,	      0,	      0,	      0,  /* 8C */
	      0,   KEY_RIGHTALT,	      0,	      0,  /* 90 */
  KEY_RIGHTCTRL,	      0,	      0,	      0,  /* 94 */
	      0,	      0,	      0,	      0,  /* 98 */
	      0,	      0,	      0,   KEY_LEFTMETA,  /* 9C */
	      0, KEY_VOLUMEDOWN,	      0,       KEY_MUTE,  /* A0 */
	      0,	      0,	      0,       KEY_STOP,  /* A4 */
	      0,	      0,	      0,	      0,  /* A8 */
	      0,	      0,	      0,       KEY_MENU,  /* AC */
	      0,	      0,   KEY_VOLUMEUP,	      0,  /* B0 */
  KEY_PLAYPAUSE,	      0,	      0,      KEY_POWER,  /* B4 */
	      0,	      0,	      0,	      0,  /* B8 */
	      0,	      0,	      0,	      0,  /* BC */
	      0,	      0,	      0,	      0,  /* C0 */
	      0,	      0,	      0,	      0,  /* C4 */
	      0,	      0,    KEY_KPSLASH,	      0,  /* C8 */
	      0,	      0,	      0,	      0,  /* CC */
	      0,	      0,	      0,	      0,  /* D0 */
	      0,	      0,	      0,	      0,  /* D4 */
	      0,	      0,    KEY_KPENTER,	      0,  /* D8 */
	      0,	      0,	      0,	      0,  /* DC */
	      0,	      0,	      0,	      0,  /* E0 */
	      0,	      0,	      0,	      0,  /* E4 */
	      0,	KEY_END,	      0,       KEY_LEFT,  /* E8 */
       KEY_HOME,	      0,	      0,	      0,  /* EC */
     KEY_INSERT,     KEY_DELETE,       KEY_DOWN,	      0,  /* F0 */
      KEY_RIGHT,	 KEY_UP,	      0,      KEY_PAUSE,  /* F4 */
	      0,	      0,   KEY_PAGEDOWN,	      0,  /* F8 */
      KEY_PRINT,     KEY_PAGEUP,      KEY_SYSRQ,	      0,
};

/* translation table scan set 1 */
static uint8_t qi2ckybd_set1dec[QKBD_IN_MXKYEVTS] = {
   KEY_RESERVED,	KEY_ESC,	  KEY_1,	  KEY_2,  /* 0 */
	  KEY_3,	  KEY_4,	  KEY_5,	  KEY_6,  /* 4 */
	  KEY_7,	  KEY_8,	  KEY_9,	  KEY_0,  /* 8 */
      KEY_MINUS,      KEY_EQUAL,  KEY_BACKSPACE,	KEY_TAB,  /* C */
	  KEY_Q,	  KEY_W,	  KEY_E,	  KEY_R,  /* 10 */
	  KEY_T,	  KEY_Y,	  KEY_U,	  KEY_I,  /* 14 */
	  KEY_O,	  KEY_P,  KEY_LEFTBRACE, KEY_RIGHTBRACE,  /* 18 */
      KEY_ENTER,   KEY_LEFTCTRL,	  KEY_A,	  KEY_S,  /* 1C */
	  KEY_D,	  KEY_F,	  KEY_G,	  KEY_H,  /* 20 */
	  KEY_J,	  KEY_K,	  KEY_L,  KEY_SEMICOLON,  /* 24 */
 KEY_APOSTROPHE,      KEY_GRAVE,  KEY_LEFTSHIFT,  KEY_BACKSLASH,  /* 28 */
	  KEY_Z,	  KEY_X,	  KEY_C,	  KEY_V,  /* 2C */
	  KEY_B,	  KEY_N,	  KEY_M,      KEY_COMMA,  /* 30 */
	KEY_DOT,      KEY_SLASH, KEY_RIGHTSHIFT, KEY_KPASTERISK,  /* 34 */
    KEY_LEFTALT,      KEY_SPACE,   KEY_CAPSLOCK,	 KEY_F1,  /* 38 */
	 KEY_F2,	 KEY_F3,	 KEY_F4,	 KEY_F5,  /* 3C */
	 KEY_F6,	 KEY_F7,	 KEY_F8,	 KEY_F9,  /* 40 */
	KEY_F10,    KEY_NUMLOCK, KEY_SCROLLLOCK,	KEY_KP7,  /* 44 */
	KEY_KP8,	KEY_KP9,    KEY_KPMINUS,	KEY_KP4,  /* 48 */
	KEY_KP5,	KEY_KP6,     KEY_KPPLUS,	KEY_KP1,  /* 4C */
	KEY_KP2,	KEY_KP3,	KEY_KP0,      KEY_KPDOT,  /* 50 */
	      0,	      0,      KEY_102ND,	KEY_F11,  /* 54 */
	KEY_F12,	      0,	      0,	      0,  /* 58 */
	      0,	      0,	      0,	      0,  /* 5C */
	      0,	      0,	      0,	      0,  /* 60 */
	KEY_F13,	      0,	KEY_F15,	KEY_F16,  /* 64 */
	KEY_F17,	KEY_F18,	      0,	      0,  /* 68 */
	      0,	      0,	      0,	      0,  /* 6C */
	      0,	      0,	      0,	 KEY_RO,  /* 70 */
	      0,	      0,	      0,   KEY_HIRAGANA,  /* 74 */
   KEY_KATAKANA,     KEY_HENKAN,	      0,   KEY_MUHENKAN,  /* 78 */
	      0,	KEY_YEN,	      0,	      0,

	      0,	      0,	      0,	      0,  /* 80 */
	      0,	      0,	      0,	      0,  /* 84 */
	      0,	      0,	      0,	      0,  /* 88 */
	      0,	      0,	      0,	      0,  /* 8C */
	      0,	      0,	      0,	      0,  /* 90 */
	      0,	      0,	      0,	      0,  /* 94 */
	      0,	      0,	      0,	      0,  /* 98 */
    KEY_KPENTER,  KEY_RIGHTCTRL,	      0,	      0,  /* 9C */
       KEY_MUTE,	      0,  KEY_PLAYPAUSE,	      0,  /* A0 */
	      0,	      0,	      0,	      0,  /* A4 */
	      0,	      0,	      0,	      0,  /* A8 */
	      0,	      0, KEY_VOLUMEDOWN,	      0,  /* AC */
   KEY_VOLUMEUP,	      0,	      0,	      0,  /* B0 */
	      0,    KEY_KPSLASH,	      0,      KEY_PRINT,  /* B4 */
   KEY_RIGHTALT,	      0,	      0,	      0,  /* B8 */
	      0,	      0,	      0,	      0,  /* BC */
	      0,	      0,	      0,	      0,  /* C0 */
	      0,      KEY_PAUSE,      KEY_SYSRQ,       KEY_HOME,  /* C4 */
	 KEY_UP,     KEY_PAGEUP,	      0,       KEY_LEFT,  /* C8 */
	      0,      KEY_RIGHT,	      0,	KEY_END,  /* CC */
       KEY_DOWN,   KEY_PAGEDOWN,     KEY_INSERT,     KEY_DELETE,  /* D0 */
	      0,	      0,	      0,	      0,  /* D4 */
	      0,	      0,	      0,   KEY_LEFTMETA,  /* D8 */
       KEY_STOP,       KEY_MENU,      KEY_POWER,	      0,  /* DC */
	      0,	      0,	      0,	      0,  /* E0 */
	      0,	      0,	      0,	      0,  /* E4 */
	      0,	      0,	      0,	      0,  /* E8 */
	      0,	      0,	      0,	      0,  /* EC */
	      0,	      0,	      0,	      0,  /* F0 */
	      0,	      0,	      0,	      0,  /* F4 */
	      0,	      0,	      0,	      0,  /* F8 */
	      0,	      0,	      0,	      0,
};


static irqreturn_t qi2ckybd_irqhandler(int irq, void *dev_id)
{
	struct i2ckybd_record *kbdrec = dev_id;

	if (kbdrec->kybd_connected)
		schedule_work(&kbdrec->qkybd_irqwork);
	return IRQ_HANDLED;
}

static int qi2ckybd_irqsetup(struct i2ckybd_record *kbdrec)
{
	unsigned long flags;
	int rc;
	struct input_dev *dev = kbdrec->i2ckbd_idev;

	flags = (dev->id.version >= 0x12) ?
		IRQF_TRIGGER_FALLING | IRQF_SAMPLE_RANDOM :
		IRQF_TRIGGER_RISING | IRQF_SAMPLE_RANDOM;

	rc = request_irq(KBDIRQNO(kbdrec), &qi2ckybd_irqhandler,
			 flags, i2ckybd_name, kbdrec);

	if (rc < 0) {
		printk(KERN_ERR
		       "Could not register for  %s interrupt "
		       "(rc = %d)\n", i2ckybd_name, rc);
		rc = -EIO;
	}
	return rc;
}

static int qi2ckybd_release_gpio(struct i2ckybd_record *kbrec)
{
	if (kbrec == NULL)
		return -EINVAL;

	dev_info(&kbrec->mykeyboard->dev,
		 "releasing keyboard gpio pins %d,%d\n",
		 kbrec->irqpin, kbrec->mclrpin);
	kbrec->extpin_teardown();
	return 0;
}

/*
 * Configure the (2) external gpio pins connected to the keyboard.
 * interrupt(input), reset(output).
 */
static int qi2ckybd_config_gpio(struct i2ckybd_record *kbrec)
{
	if (kbrec == NULL)
		return -EINVAL;

	return kbrec->extpin_setup();
}

/* read keyboard via i2c address + register offset, return # bytes read */
static int kybd_read(struct i2c_client *kbd, uint8_t regaddr,
		     uint8_t *buf, uint32_t rdlen)
{
	uint8_t ldat = regaddr;
	struct i2c_msg msgs[] = {
		[0] = {
			.addr	= kbd->addr,
			.flags	= 0,
			.buf	= (void *)&ldat,
			.len	= 1
		},
		[1] = {
			.addr	= kbd->addr,
			.flags	= I2C_M_RD,
			.buf	= (void *)buf,
			.len	= 1
		}
	};

	return i2c_transfer(kbd->adapter, msgs, 2);
}

/* Write the specified data to the  control reg */
static int kybd_wrcntrl(struct i2c_client *kbd, uint8_t *data, int dlen)
{
	int i;
	uint8_t rpd[4];
	struct i2c_msg msgs[] = {
		[0] = {
			.addr	= kbd->addr,
			.flags	= 0,
			.buf	= (void *)rpd,
			.len	= (dlen + 1)
		}
	};

	rpd[0] = QKBD_REG_CNTRL;
	for (i = 0; i < dlen; i++)
		rpd[i+1] = data[i];

	return i2c_transfer(kbd->adapter, msgs, 1);
}

/* issue command to control register and read the ACK */
static int qi2ckybd_issuecmd(struct i2c_client *kbd, uint8_t cmd,
				uint8_t *parms, int pcnt)
{
	int i, rc = -EIO, xfcnt;
	uint8_t rdat[4];

	rdat[0] = cmd;
	for (i = 0; i < pcnt; i++)
		rdat[i+1] = parms[i];

	xfcnt = pcnt + 1;
	rc = kybd_wrcntrl(kbd, rdat, xfcnt);
	if (rc > 0) {
		dev_dbg(&kbd->dev, "Write cmd (0x%x) to keybd control register "
			"w/ %d params via adapter(%s), (rc=%d)\n",
			rdat[0], pcnt, kbd->adapter->name, rc);
		rdat[0] = i2c_smbus_read_byte(kbd);
		if (rdat[0] == QKBD_RSP_ACK)
			rc = 0;
		else {
			dev_dbg(&kbd->dev, "Read from keybd control "
				"register yields (rc=%d), (rdat=0x%x)\n",
				rc, rdat[0]);
		}
	}
	return rc;
}

/* submit cmd to keyboard via work queue */
static void qi2ckybd_submitcmd(struct work_struct *work)
{
	struct i2ckybd_record *kbdrec =
		container_of(work, struct i2ckybd_record, kb_cmdq.work);

	qi2ckybd_issuecmd(kbdrec->mykeyboard, kbdrec->cmd, kbdrec->cargs,
			  kbdrec->noargs);
}

/* send the enable cmd and verify proper state in status register */
static int qi2ckybd_enablekybd(struct i2c_client *kbd)
{
	uint8_t rdat;
	int rc = qi2ckybd_issuecmd(kbd, QKBD_CMD_ENABLE, NULL, 0);

	if (rc) {
		dev_err(&kbd->dev, "FAILED: enable cmd(rc=%d)\n", rc);
		return rc;
	}
	rc = kybd_read(kbd, QKBD_REG_STATUS, &rdat, sizeof(rdat));
	if (rc == 2) {
		if (QKBD_SRBIT_SCANEN & rdat) {
			dev_info(&kbd->dev, "keyboard enabled\n");
			rc = 0;
		} else {
			dev_err(&kbd->dev, "keyboard enable "
				"FAILED: status register yields (rc=%d),"
				" (rdat=0x%x)\n", rc, rdat);
			rc = -EIO;
		}
	} else {
		dev_err(&kbd->dev, "FAILED: read status (rc=%d)\n", rc);
		rc = -EIO;
	}
	return rc;
}

static int qi2ckybd_setrepeat(struct i2c_client *kbd, int period, int delay)
{
	/* set up the keyboard parameter
	 *  the lower 5 bits is the period in cps
	 *  bits 5,6 is the initial delay in units of 250 ms
	 */
	uint8_t parm = (uint8_t)((period & 0x1F) | ((delay/250)<<5));

	return qi2ckybd_issuecmd(kbd, QKBD_CMD_SREP, &parm, 1);
}

static int qi2ckybd_selscanset(struct i2c_client *kbd, int setno)
{
	int rc;
	uint8_t bdata = (uint8_t)setno;
	uint8_t xstat = (setno == 2) ? QKBD_SRBIT_SSET : 0;

	dev_dbg(&kbd->dev, "ENTRY: select scan set %d\n", setno);
	rc = qi2ckybd_issuecmd(kbd, QKBD_CMD_SLSET, &bdata, 1);
	if (!rc) {
		if (2 == kybd_read(kbd, QKBD_REG_STATUS, &bdata,
				   sizeof(bdata))) {
			if (xstat == (QKBD_SRBIT_SSET & bdata)) {
				dev_info(&kbd->dev, "scan set %d enabled\n",
					 setno);
				rc = 0;
			} else {
				dev_err(&kbd->dev, "FAILED: select scan set"
				       " 2 (status=%d)\n", bdata);
				rc = -EIO;
			}
		} else {
			dev_err(&kbd->dev, "FAILED: read status (rc=%d)\n",
				rc);
			rc = -EIO;
		}
	}
	return rc;
}

/* query sw version and product id from keyboard */
static int qi2ckybd_getkbinfo(struct i2c_client *kbd, int *info)
{
	int rc;
	uint8_t rdat[2] = { 0xFF, 0xFF };

	dev_dbg(&kbd->dev, "ENTRY: read keyboard id info\n");
	rc = qi2ckybd_issuecmd(kbd, QKBD_CMD_READID, NULL, 0);
	if (!rc) {
		rdat[0] = i2c_smbus_read_byte(kbd);	/* LSB: SW-release */
		rdat[1] = i2c_smbus_read_byte(kbd);	/* MSB: product-id */
		*info = rdat[1] << 8 | rdat[0];
		rc = 0;

		dev_info(&kbd->dev, "Received keyboard ID info: "
			"MSB: 0x%x LSB 0x%x\n", rdat[1], rdat[0]);
	}
	return rc;
}

/* keyboard has been reset */
static void qi2ckybd_recoverkbd(struct work_struct *work)
{
	int rc;
	struct i2ckybd_record *kbdrec =
		container_of(work, struct i2ckybd_record, kb_cmdq.work);
	struct i2c_client *kbd = kbdrec->mykeyboard;

	INIT_DELAYED_WORK(&kbdrec->kb_cmdq, qi2ckybd_submitcmd);
	dev_info(&kbd->dev, "keyboard recovery requested\n");

	rc = qi2ckybd_enablekybd(kbd);
	if (!rc) {
		kbdrec->kybd_connected = 1;
		if (kbdrec->scanset == 2)
			qi2ckybd_selscanset(kbd, 2);
		qi2ckybd_irqsetup(kbdrec);
		/* make sure we don't drop irq */
		schedule_work(&kbdrec->qkybd_irqwork);
	} else
		dev_err(&kbd->dev,
			"recovery failed with (rc=%d)\n", rc);
}

static void qi2ckybd_hwreset(struct i2ckybd_record *rd)
{
	if (rd != NULL) {
		if (rd->i2c_hw_reset != NULL)
			rd->i2c_hw_reset(rd->mclrpin);
	}
}


/* translate from set 1 scan code(s) to intermediate xlate-lookup code */
static inline void qi2ckybd_xlscancode(s32 rawcode, uint8_t prefix,
					  u32 *xlcode, s32 *keystate)
{
	*keystate = (rawcode & 0x80) ? QKBD_IN_KEYRELEASE : QKBD_IN_KEYPRESS;
	*xlcode = (rawcode & 0x7F) | ((prefix) ? 0x80 : 0);
}

/*
 * process set 1 scan codes
 * This function conditions the code sequences read from the
 * keyboard into a intermediate lookup code. The calling
 * function takes the returned code (if non-zero) and
 * reports it as a key event into the input subsystem.
 *
 * Special cases:
 *   PAUSE :  make=0xE1,0x1D,0x45,0xE1,0x9D,0xC5
 *            break=null
 *   PRTSCR:  make=0xE0,0x2A,0xE0,0x37
 *            break=0xE0,0xB7,0xE0,0xAA
 *
 * The PAUSE key is the only code sequence using 0xE1 prefix.
 * The resulting lookup code is 0x45 since the 0xE0 0x45 sequence
 * is unassigned (never used).
 */
static u32 pr_set1code(struct i2ckybd_record *kbdrec,
		       s32 code, s32 *kevent, s32 *i2cerr)
{
	u32 xlkcode = 0;
	struct input_dev *idev = kbdrec->i2ckbd_idev;
	struct device *dev = &kbdrec->mykeyboard->dev;
	bool dolookup = false;

	switch (code) {
	case QKBD_KCODE_PREF1:
		kbdrec->prefix = 1;
		kbdrec->kcnt = 0;
		break;
	case QKBD_KCODE_PREF2:
		kbdrec->prefix = 2;
		kbdrec->kcnt = 0;
		break;
	case QKBD_KCODE_PRTPFX1:
	case QKBD_KCODE_PRTPFX2:
		if (kbdrec->prefix) {
			/* skip these codes as part of processing PrtScr key */
			kbdrec->prefix = 2;
			kbdrec->kcnt = 0;
		} else
			dolookup = true;
		break;
	case QKBD_KCODE_OVRFL:
		dev_err(dev,
			"I2C Keyboard Sync Failure: keyboard overflow\n");
		xlkcode = 0;
		kbdrec->prefix = 0;
		break;
	case QKBD_KCODE_INTERR:
		dev_err(dev, "Sync Failure: keyboard internal error\n");
		xlkcode = 0;
		kbdrec->prefix = 0;
		/* force a recovery */
		*i2cerr = -EIO;
		break;
	default:
		if (code >= 0)
			dolookup = true;
		else
			*i2cerr = code;
	}
	if (dolookup) {
		if (kbdrec->prefix <= 1) {
			uint8_t *tbl = (uint8_t *) (idev->keycode);
			if (kbdrec->prefix &&
			    (code == (QKBD_KCODE_PRTPFX1 | 0x80)))
				code = QKBD_KCODE_PRTPFX2;

			qi2ckybd_xlscancode(code, kbdrec->prefix,
					       &xlkcode, kevent);
			xlkcode = tbl[xlkcode];
		} else
			kbdrec->prefix--;
	}
	return xlkcode;
}

/* translate from set 2 scan code(s) to intermediate xlate-lookup code */
static inline void qi2ckybd_xlset2code(s32 rawcode, uint8_t *prefix,
					  u32 *xlcode, s32 *keystate)
{
	uint8_t *pfxptr = prefix;

	if (*prefix == QKBD_KCODE_BRKPFX) {
		*keystate = QKBD_IN_KEYRELEASE;
		pfxptr++;
	} else
		*keystate = QKBD_IN_KEYPRESS;
	*xlcode = (rawcode & 0x7F) | ((*pfxptr) ? 0x80 : 0);
}

/*
 * process set 2 scan codes
 * This function conditions the set 2 code sequences read from the
 * keyboard into a intermediate lookup code. The calling
 * function takes the returned code (if non-zero) and
 * reports it as a key event into the input subsystem.
 *
 * Special case:
 *   PAUSE :  make=0xE1,0x14,0x77,0xE1,0xF0,0x14,0xF0,0x77
 *            break=null
 *
 * The PAUSE key is the only code sequence using 0xE1 prefix.
 * The resulting lookup code is 0x77.
 */
static u32 pr_set2code(struct i2ckybd_record *kbdrec,
		       s32 code, s32 *kevent, s32 *i2cerr)
{
	u32 xlkcode = 0;
	struct input_dev *idev = kbdrec->i2ckbd_idev;
	struct device *dev = &kbdrec->mykeyboard->dev;

	switch (code) {
	case QKBD_KCODE_PREF1:
	case QKBD_KCODE_PREF2:
		kbdrec->prefix = code;
		kbdrec->kcnt = 0;
		break;
	case QKBD_KCODE_BRKPFX:
		kbdrec->pfxstk[1] = kbdrec->pfxstk[0];
		kbdrec->pfxstk[0] = kbdrec->prefix;
		kbdrec->prefix = code;
		kbdrec->kcnt = 0;
		break;
	case QKBD_KCODE_OVRFL:
		dev_err(dev, "Sync Failure: keyboard overflow\n");
		xlkcode = 0;
		kbdrec->prefix = kbdrec->pfxstk[0] = kbdrec->pfxstk[1] = 0;
		break;
	case QKBD_KCODE_INTERR:
		dev_err(dev, "Sync Failure: keyboard internal error\n");
		xlkcode = 0;
		kbdrec->prefix = kbdrec->pfxstk[0] = kbdrec->pfxstk[1] = 0;
		/* force a recovery */
		*i2cerr = -EIO;
		break;
	case QKBD_KCODE_F7:
		/* F7 conflict with F5 key (0x83 vs. 0x03) */
		code--;
	default:
		if (code < 0) {
			*i2cerr = code;
		} else {
			uint8_t *tbl = (uint8_t *) (idev->keycode);
			qi2ckybd_xlset2code(code, &kbdrec->prefix,
					    &xlkcode, kevent);
			xlkcode = tbl[xlkcode];
		}
	}
	return xlkcode;
}

/*
 * handler function called via work queue
 *
 * This function reads from the keyboard via I2C until the
 * keyboard's output queue of scan codes is empty.
 *
 * If running on scan set 1, we support the "RAW" keyboard mode
 * directly. The RAW mode is required if using X11. Currently,
 * RAW mode is not supported for scan set 2.
 */
static void qi2ckybd_fetchkeys(struct work_struct *work)
{
	struct i2ckybd_record *kbdrec =
		container_of(work, struct i2ckybd_record, qkybd_irqwork);
	struct i2c_client *kbdcl = kbdrec->mykeyboard;
	struct input_dev *idev = kbdrec->i2ckbd_idev;
	s32 rdat, kevent = QKBD_IN_KEYRELEASE, i2c_error = 0;
	u32 xlkcode = 0;

	if (!kbdrec->kybd_connected) {
		do {
			rdat = i2c_smbus_read_byte(kbdcl);
		} while (rdat > 0);
		return;
	}

	/* keep reading from the keyboard until its queue is empty */
	do {
		rdat = i2c_smbus_read_byte(kbdcl);
		dev_dbg(&kbdcl->dev, "scan code = 0x%x\n", rdat);
		if (rdat == 0)
			continue;

		/* deliver the raw codes for RAW keyboard mode & X11 */
		if (kbdrec->scanset == 1) {
			input_event(idev, EV_MSC, MSC_RAW, rdat);
			kbdrec->evcnt++;
		}

		xlkcode = kbdrec->xlf(kbdrec, rdat, &kevent, &i2c_error);
		/*
		 * error received from i2c driver
		 * do a hard reset of the keyboard, and attempt to recover
		 */
		if (i2c_error) {
			dev_err(&kbdcl->dev, "Failed read from keyboard"
			       " (rc = 0x%2x)\n", i2c_error);
			kbdrec->kybd_connected = 0;
			free_irq(KBDIRQNO(kbdrec), kbdrec);
			qi2ckybd_hwreset(kbdrec);
			INIT_DELAYED_WORK(&kbdrec->kb_cmdq,
					  qi2ckybd_recoverkbd);
			schedule_delayed_work(&kbdrec->kb_cmdq,
					      msecs_to_jiffies(600));
			kbdrec->kcnt = 0;
			break;
		}

		/* we have a translated code to feed into the input system */
		if (xlkcode) {
			input_report_key(idev, xlkcode, kevent);
			kbdrec->kcnt++;
			kbdrec->prefix = 0;
			kbdrec->pfxstk[0] = 0;
			kbdrec->pfxstk[1] = 0;
			xlkcode = 0;
		}
	} while (rdat != 0);

	if (kbdrec->kcnt || kbdrec->evcnt) {
		input_sync(idev);
		kbdrec->evcnt = kbdrec->kcnt = 0;
	} else
		dev_dbg(&kbdcl->dev, "0 keys processed after interrupt\n");
}

static void qi2ckybd_shutdown(struct i2ckybd_record *rd)
{
	if (rd->kybd_connected) {
		dev_info(&rd->mykeyboard->dev, "disconnecting keyboard\n");
		rd->kybd_connected = 0;
		free_irq(KBDIRQNO(rd), rd);
		flush_work(&rd->qkybd_irqwork);
		qi2ckybd_hwreset(rd);
	}
}

static int qi2ckybd_eventcb(struct input_dev *dev, unsigned int type,
			    unsigned int code, int value)
{
	int rc = -EPERM;
	struct i2ckybd_record *kbdrec = input_get_drvdata(dev);
	struct device *kbdev = &kbdrec->mykeyboard->dev;

	if (!kbdrec->kybd_connected)
		return rc;
	switch (type) {
	case EV_MSC:
		/* raw events are forwarded to keyboard handler */
		break;
	case EV_REP:
		if (kbdrec->hwrep_enabled) {
			kbdrec->cmd = QKBD_CMD_SREP;
			kbdrec->noargs = 1;
			kbdrec->cargs[0] = (dev->rep[REP_PERIOD] & 0x1F) |
				((dev->rep[REP_DELAY]/250)<<5);
			schedule_delayed_work(&kbdrec->kb_cmdq,
					      msecs_to_jiffies(600));
			rc = 0;
		} else
			dev_warn(kbdev, "attempt to set typematic rate "
				 "but hw autorepeat not enabled\n");
		break;
	case EV_LED:
		if ((kbdrec->product_info & 0xFF) > 0x10) {
			kbdrec->cmd = QKBD_CMD_SLED;
			kbdrec->noargs = 1;
			kbdrec->cargs[0] =
				(test_bit(LED_SCROLLL, dev->led) ? 1 : 0) |
				(test_bit(LED_NUML,  dev->led) ? 2 : 0) |
				(test_bit(LED_CAPSL, dev->led) ? 4 : 0);
			schedule_delayed_work(&kbdrec->kb_cmdq,
					      msecs_to_jiffies(10));
			dev_dbg(kbdev, "submitted set LEDs cmd\n");
			rc = 0;
		}

		break;
	default:
		dev_warn(kbdev, "rcv'd unrecognized command (%d)\n", type);
	}

	return rc;
}

static int qi2ckybd_opencb(struct input_dev *dev)
{
	int rc;
	struct i2ckybd_record *kbdrec = input_get_drvdata(dev);
	struct i2c_client *kbd = kbdrec->mykeyboard;

	dev_dbg(&kbd->dev, "ENTRY: input_dev open callback\n");
	qi2ckybd_getkbinfo(kbd, &kbdrec->product_info);
	rc = qi2ckybd_enablekybd(kbd);
	if (!rc) {
		if (kbdrec->hwrep_enabled)
			qi2ckybd_setrepeat(kbd, QKBD_TYPO_PERIOD,
					   QKBD_TYPO_DELAY);
		if (kbdrec->scanset == 2)
			qi2ckybd_selscanset(kbd, 2);
		dev->id.version = kbdrec->product_info & 0xFF;
		dev->id.product = (kbdrec->product_info & ~0xFF) |
			kbdrec->scanset;
		rc = qi2ckybd_irqsetup(kbdrec);
		if (!rc)
			kbdrec->kybd_connected = 1;
	} else
		rc = -EIO;
	return rc;
}

static void qi2ckybd_closecb(struct input_dev *idev)
{
	struct i2ckybd_record *kbdrec = input_get_drvdata(idev);
	struct device *dev = &kbdrec->mykeyboard->dev;

	dev_dbg(dev, "ENTRY: close callback\n");
	qi2ckybd_shutdown(kbdrec);
}

static struct input_dev *create_inputdev_instance(struct i2ckybd_record *kbdrec)
{
	struct device *dev = &kbdrec->mykeyboard->dev;
	struct input_dev *idev = 0;
	unsigned char kidx;

	idev = input_allocate_device();
	if (idev) {
		idev->name = i2ckybd_name;
		idev->phys = kbdrec->physinfo;
		idev->id.bustype = BUS_I2C;
		idev->id.vendor  = QCVENDOR_ID;
		idev->id.product = 1;
		idev->id.version = 1;
		idev->open = qi2ckybd_opencb;
		idev->close = qi2ckybd_closecb;
		idev->event = qi2ckybd_eventcb;
		idev->keycode = kbdrec->xltable;
		idev->keycodesize = sizeof(uint8_t);
		idev->keycodemax = QKBD_IN_MXKYEVTS;
		if (kbdrec->hwrep_enabled)
			idev->evbit[0] = BIT(EV_KEY) | BIT(EV_LED);
		else
			idev->evbit[0] = BIT(EV_KEY) | BIT(EV_LED) |
				BIT(EV_REP);
		idev->ledbit[0] = BIT(LED_SCROLLL) | BIT(LED_NUML) |
			BIT(LED_CAPSL);
		__clear_bit(LED_SCROLLL, idev->led);
		__clear_bit(LED_NUML, idev->led);
		__clear_bit(LED_CAPSL, idev->led);
		memset(idev->keycode, 0, QKBD_IN_MXKYEVTS);

		if (kbdrec->scanset == 1) {
			/* support raw mode for scan set 1 */
			idev->evbit[0] |= BIT(EV_MSC);
			idev->mscbit[0] = BIT(MSC_RAW);
			kbdrec->xlf = pr_set1code;
			memcpy(idev->keycode, qi2ckybd_set1dec,
			       QKBD_IN_MXKYEVTS);
		} else {
			idev->mscbit[0] = 0;
			kbdrec->xlf = pr_set2code;
			memcpy(idev->keycode, qi2ckybd_set2dec,
			       QKBD_IN_MXKYEVTS);
		}
		/* the first 83 keycodes are supported */
		for (kidx = 0; kidx <= KEY_KPDOT; kidx++)
			__set_bit(kidx, idev->keybit);

		/* a few more misc keys */
		__set_bit(KEY_F11, idev->keybit);
		__set_bit(KEY_F12, idev->keybit);
		__set_bit(KEY_KPENTER, idev->keybit);
		__set_bit(KEY_KPSLASH, idev->keybit);
		__set_bit(KEY_RIGHTCTRL, idev->keybit);
		__set_bit(KEY_RIGHTALT, idev->keybit);
		__set_bit(KEY_HOME, idev->keybit);
		__set_bit(KEY_UP, idev->keybit);
		__set_bit(KEY_PAGEUP, idev->keybit);
		__set_bit(KEY_LEFT, idev->keybit);
		__set_bit(KEY_RIGHT, idev->keybit);
		__set_bit(KEY_END, idev->keybit);
		__set_bit(KEY_DOWN, idev->keybit);
		__set_bit(KEY_PAGEDOWN, idev->keybit);
		__set_bit(KEY_INSERT, idev->keybit);
		__set_bit(KEY_DELETE, idev->keybit);
		__set_bit(KEY_PAUSE, idev->keybit);
		__set_bit(KEY_LEFTMETA, idev->keybit);
		__set_bit(KEY_MENU, idev->keybit);
		__set_bit(KEY_PRINT, idev->keybit);

		input_set_drvdata(idev, kbdrec);
	} else {
		dev_err(dev,
			"Failed to allocate input device for %s\n",
			i2ckybd_name);
	}
	return idev;
}

static void qi2ckybd_connect2inputsys(struct work_struct *work)
{
	struct i2ckybd_record *kbdrec =
		container_of(work, struct i2ckybd_record, kb_cmdq.work);
	struct device *dev = &kbdrec->mykeyboard->dev;

	kbdrec->i2ckbd_idev = create_inputdev_instance(kbdrec);
	if (kbdrec->i2ckbd_idev) {
		if (input_register_device(kbdrec->i2ckbd_idev) != 0) {
			dev_err(dev, "Failed to register with"
				" input system\n");
			input_free_device(kbdrec->i2ckbd_idev);
		} else {
			INIT_DELAYED_WORK(&kbdrec->kb_cmdq, qi2ckybd_submitcmd);
		}
	}
}

/* utility function used by probe */
static int testfor_keybd(struct i2c_client *new_kbd)
{
	int rc = 0;
	struct i2ckybd_record *rd = i2c_get_clientdata(new_kbd);

	if (!rd->kybd_exists) {
		qi2ckybd_hwreset(rd);
		mdelay(500);
		rc = qi2ckybd_issuecmd(new_kbd, QKBD_CMD_RESET, NULL, 0);
		if (!rc) {
			dev_info(&new_kbd->dev,
				 "Detected %s, attempting to initialize "
				 "keyboard\n", i2ckybd_name);
			snprintf(rd->physinfo, QKBD_PHYSLEN,
				 "%s/%s/event0",
				 new_kbd->adapter->dev.bus_id,
				 new_kbd->dev.bus_id);
			rd->kybd_exists = 1;
			INIT_DELAYED_WORK(&rd->kb_cmdq,
					  qi2ckybd_connect2inputsys);
			schedule_delayed_work(&rd->kb_cmdq,
					      msecs_to_jiffies(600));
		}
	}
	return rc;
}

static int __devexit qi2ckybd_remove(struct i2c_client *kbd)
{
	struct i2ckybd_record *rd = i2c_get_clientdata(kbd);

	dev_info(&kbd->dev, "removing keyboard driver\n");
	device_init_wakeup(&kbd->dev, 0);

	if (rd->i2ckbd_idev) {
		dev_dbg(&kbd->dev, "deregister from input system\n");
		input_unregister_device(rd->i2ckbd_idev);
		rd->i2ckbd_idev = 0;
	}
	qi2ckybd_shutdown(rd);
	qi2ckybd_release_gpio(rd);
	kfree(rd);

	return 0;
}

#ifdef CONFIG_PM
static int qi2ckybd_suspend(struct i2c_client *kbd, pm_message_t mesg)
{
	struct i2ckybd_record *kbdrec = i2c_get_clientdata(kbd);

	if (device_may_wakeup(&kbd->dev))
		enable_irq_wake(KBDIRQNO(kbdrec));

	return 0;
}

static int qi2ckybd_resume(struct i2c_client *kbd)
{
	struct i2ckybd_record *kbdrec = i2c_get_clientdata(kbd);

	if (device_may_wakeup(&kbd->dev))
		disable_irq_wake(KBDIRQNO(kbdrec));

	return 0;
}
#else
# define qi2ckybd_suspend NULL
# define qi2ckybd_resume  NULL
#endif

static const struct i2c_device_id i2ckybd_idtable[] = {
       { i2ckybd_name, 0 },
       { }
};

MODULE_DEVICE_TABLE(i2c, i2ckybd_idtable);

static struct i2c_driver i2ckbd_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = i2ckybd_name,
	},
	.probe	  = qi2ckybd_probe,
	.remove	  = __devexit_p(qi2ckybd_remove),
	.suspend  = qi2ckybd_suspend,
	.resume   = qi2ckybd_resume,
	.id_table = i2ckybd_idtable,
};

static int __devinit qi2ckybd_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct msm_i2ckbd_platform_data *setup_data;
	int                              rc = -ENOMEM;
	struct i2ckybd_record           *rd = 0;


	if (!client->dev.platform_data) {
		dev_err(&client->dev,
			"keyboard platform device data is required\n");
		return -ENODEV;
	}

	rd = kzalloc(sizeof(struct i2ckybd_record), GFP_KERNEL);

	if (!rd)
		return rc;

	client->driver = &i2ckbd_driver;
	i2c_set_clientdata(client, rd);
	rd->mykeyboard    = client;
	setup_data        = client->dev.platform_data;
	rd->scanset       = setup_data->scanset1 ? 1 : 2;
	rd->hwrep_enabled = setup_data->hwrepeat ? 1 : 0;
	rd->mclrpin       = setup_data->gpioreset;
	rd->irqpin        = setup_data->gpioirq;
	rd->extpin_setup  = setup_data->gpio_setup;
	rd->extpin_teardown  = setup_data->gpio_shutdown;
	rd->i2c_hw_reset = setup_data->hw_reset;

	rc = qi2ckybd_config_gpio(rd);
	if (rc)
		goto failexit1;

	rc = testfor_keybd(client);
	if (!rc)
		device_init_wakeup(&client->dev, 1);
	else
		goto failexit1;

	INIT_WORK(&rd->qkybd_irqwork, qi2ckybd_fetchkeys);

	return 0;

 failexit1:
	qi2ckybd_release_gpio(rd);
	kfree(rd);
	return rc;
}

static int __init qi2ckybd_init(void)
{
	return i2c_add_driver(&i2ckbd_driver);
}


static void __exit qi2ckybd_exit(void)
{
	i2c_del_driver(&i2ckbd_driver);
}

module_init(qi2ckybd_init);
module_exit(qi2ckybd_exit);
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("I2C QWERTY keyboard driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm-i2ckbd");

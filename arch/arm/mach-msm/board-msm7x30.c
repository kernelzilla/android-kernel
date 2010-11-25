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

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#include <linux/usb/mass_storage_function.h>
#include <linux/spi/spi.h>
#include <linux/bma150.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/input.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_spi.h>
#include <linux/android_pmem.h>
#include <mach/pmic8058-keypad.h>
#include <mach/qdsp5v2/snddev_icodec.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#include "pm.h"
#include <linux/msm_kgsl.h>

#define MSM_PMEM_SF_SIZE	0x1000000
#define MSM_FB_SIZE		0x200000
#define MSM_PMEM_GPU1_SIZE      0x1000000
#define MSM_GPU_PHYS_SIZE       SZ_2M
#define MSM_PMEM_ADSP_SIZE      0x2000000
#define PMEM_KERNEL_EBI1_SIZE   0x2000000



#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900

static const unsigned int surf_keymap[] = {
	KEY(0, 0, KEY_7),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_UP),
	KEY(0, 3, KEY_RIGHT),
	KEY(0, 4, KEY_ENTER),
	KEY(0, 5, KEY_L),
	KEY(0, 6, KEY_BACK),
	KEY(0, 7, KEY_M),

	KEY(1, 0, KEY_LEFT),
	KEY(1, 1, KEY_SEND),
	KEY(1, 2, KEY_1),
	KEY(1, 3, KEY_4),
	KEY(1, 4, KEY_CLEAR),
	KEY(1, 5, KEY_MSDOS),
	KEY(1, 6, KEY_SPACE),
	KEY(1, 7, KEY_COMMA),

	KEY(2, 0, KEY_6),
	KEY(2, 1, KEY_5),
	KEY(2, 2, KEY_8),
	KEY(2, 3, KEY_3),
	KEY(2, 4, KEY_NUMERIC_STAR),
	KEY(2, 5, KEY_UP),
	KEY(2, 6, KEY_DOWN), /* SYN */
	KEY(2, 7, KEY_LEFTSHIFT),

	KEY(3, 0, KEY_9),
	KEY(3, 1, KEY_NUMERIC_POUND),
	KEY(3, 2, KEY_0),
	KEY(3, 3, KEY_2),
	KEY(3, 4, KEY_END),
	KEY(3, 5, KEY_F1),
	KEY(3, 6, KEY_F2),
	KEY(3, 7, KEY_F3),

	KEY(4, 0, KEY_BACK),
	KEY(4, 1, KEY_HOME),
	KEY(4, 2, KEY_MENU),
	KEY(4, 3, KEY_VOLUMEUP),
	KEY(4, 4, KEY_VOLUMEDOWN),
	KEY(4, 5, KEY_F4),
	KEY(4, 6, KEY_F5),
	KEY(4, 7, KEY_F6),

	KEY(5, 0, KEY_R),
	KEY(5, 1, KEY_T),
	KEY(5, 2, KEY_Y),
	KEY(5, 3, KEY_LEFTALT),
	KEY(5, 4, KEY_HOME),
	KEY(5, 5, KEY_Q),
	KEY(5, 6, KEY_W),
	KEY(5, 7, KEY_E),

	KEY(6, 0, KEY_F),
	KEY(6, 1, KEY_G),
	KEY(6, 2, KEY_H),
	KEY(6, 3, KEY_CAPSLOCK),
	KEY(6, 4, KEY_PAGEUP),
	KEY(6, 5, KEY_A),
	KEY(6, 6, KEY_S),
	KEY(6, 7, KEY_D),

	KEY(7, 0, KEY_V),
	KEY(7, 1, KEY_B),
	KEY(7, 2, KEY_N),
	KEY(7, 3, KEY_MENU), /* REVISIT - SYM */
	KEY(7, 4, KEY_PAGEDOWN),
	KEY(7, 5, KEY_Z),
	KEY(7, 6, KEY_X),
	KEY(7, 7, KEY_C),

	KEY(8, 0, KEY_P),
	KEY(8, 1, KEY_J),
	KEY(8, 2, KEY_K),
	KEY(8, 3, KEY_INSERT),
	KEY(8, 4, KEY_END),
	KEY(8, 5, KEY_U),
	KEY(8, 6, KEY_I),
	KEY(8, 7, KEY_O),

	KEY(9, 0, KEY_4),
	KEY(9, 1, KEY_5),
	KEY(9, 2, KEY_6),
	KEY(9, 3, KEY_7),
	KEY(9, 4, KEY_8),
	KEY(9, 5, KEY_1),
	KEY(9, 6, KEY_2),
	KEY(9, 7, KEY_3),

	KEY(10, 0, KEY_F7),
	KEY(10, 1, KEY_F8),
	KEY(10, 2, KEY_F9),
	KEY(10, 3, KEY_F10),
	KEY(10, 4, KEY_FN),
	KEY(10, 5, KEY_9),
	KEY(10, 6, KEY_0),
	KEY(10, 7, KEY_DOT),

	KEY(11, 0, KEY_LEFTCTRL),
	KEY(11, 1, KEY_F11),  /* START */
	KEY(11, 2, KEY_ENTER),
	KEY(11, 3, KEY_SEARCH),
	KEY(11, 4, KEY_DELETE),
	KEY(11, 5, KEY_RIGHT),
	KEY(11, 6, KEY_LEFT),
	KEY(11, 7, KEY_RIGHTSHIFT),
};

/* REVISIT - this needs to be done through add_subdevice
 * API
 */
static struct resource resources_keypad[] = {
	{
		.start	= PM8058_IRQ_KEYPAD,
		.end	= PM8058_IRQ_KEYPAD,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_IRQ_KEYSTUCK,
		.end	= PM8058_IRQ_KEYSTUCK,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct pmic8058_keypad_data surf_keypad_data = {
	.input_name		= "surf-keypad",
	.input_phys_device	= "surf-keypad/input0",
	.num_rows		= 12,
	.num_cols		= 8,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.keymap_size		= ARRAY_SIZE(surf_keymap),
	.keymap			= surf_keymap,
	.debounce_ms		= 8,
	.scan_delay_ms		= 32,
	.row_hold_us		= 125,
	.wakeup			= 1,
};

static struct platform_device surf_keypad_device = {
	.name		= "pmic8058_keypad",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(resources_keypad),
	.resource       = resources_keypad,
	.dev		= {
		.platform_data = &surf_keypad_data,
	},
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.pm_irqs = {
		[PM8058_IRQ_KEYPAD - PM8058_FIRST_IRQ] = 74,
		[PM8058_IRQ_KEYSTUCK - PM8058_FIRST_IRQ] = 75,
	}
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0),
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data = &pm8058_7x30_data,
	},
};

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_INPUT,
				  GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	i2c_register_board_info(6 /* I2C_SSBI ID */, pm8058_boardinfo,
				ARRAY_SIZE(pm8058_boardinfo));

	return 0;
}

static struct vreg *vreg_marimba_1;
static struct vreg *vreg_marimba_2;

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}

out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
};

static int msm_fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	uint32_t irqcfg = GPIO_CFG(147, 0, GPIO_INPUT, GPIO_PULL_UP,
					GPIO_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
		return -EIO;
	}
	printk(KERN_INFO "%s\n", __func__);
	return 0;
};

static int msm_fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc;
	uint32_t irqcfg = GPIO_CFG(147, 0, GPIO_INPUT, GPIO_NO_PULL,
					GPIO_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
		return -EIO;
	}
	printk(KERN_INFO "%s\n", __func__);
	return 0;
};

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.gpio_shutdown = msm_fm_radio_shutdown,
	.gpio_setup =  msm_fm_radio_setup,
	.gpioirq = MSM_GPIO_TO_INT(147),
	.vreg_fm = NULL,
};


/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

static int msm_marimba_tsadc_power(int vreg_on)
{
	struct vreg *vreg_tsadc;
	int rc = 0;

	vreg_tsadc = vreg_get(NULL, "gp12");
	/* LDO18 */
	if (IS_ERR(vreg_tsadc)) {
		printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_tsadc));
		return PTR_ERR(vreg_tsadc);
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_tsadc);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
	} else {
		rc = vreg_disable(vreg_tsadc);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
	}

	return rc;
}

static struct marimba_tsadc_platform_data marimba_tsadc_pdata = {
	.marimba_tsadc_power = msm_marimba_tsadc_power,
};


static struct marimba_platform_data marimba_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.fm = &marimba_fm_pdata,
	.tsadc = &marimba_tsadc_pdata,
};

static void __init msm7x30_init_marimba(void)
{
	vreg_marimba_1 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
}

static struct adie_codec_action_unit iearpiece_8KHz_osr256_actions[] = {
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_OFF},
	{ ADIE_CODEC_ACTION_DELAY_WAIT, 0xbb8},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x02, 0x02)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x02, 0x00)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_READY },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x24, 0x6F, 0x44)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x04, 0x5F, 0xBC)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x81, 0xFF, 0x4E)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x25, 0x0F, 0x0E)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x26, 0xfc, 0xfc)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x36, 0xc0, 0x80)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x3A, 0xFF, 0x2B)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x3d, 0xFF, 0xD5)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x83, 0x21, 0x21)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x33, 0x80, 0x80)},
	{ ADIE_CODEC_ACTION_DELAY_WAIT,  0x2710},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x33, 0x40, 0x40)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x84, 0xff, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x05, 0x04)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_ANALOG_READY},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8a, 0x01, 0x01)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x36, 0xc0, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x33, 0x40, 0x00)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED,  ADIE_CODEC_ANALOG_OFF},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x33, 0x80, 0x00)}
};

static struct adie_codec_action_unit imic_8KHz_osr256_actions[] = {
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_OFF },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x01, 0x01)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x80, 0x01, 0x00) },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x30, 0x30)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x11, 0xfc, 0xfc)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x13, 0xfc, 0x58)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x14, 0xff, 0x65)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x15, 0xff, 0x64)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x82, 0xff, 0x5C)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_READY },
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x0D, 0xF0, 0xb0)},
	{ ADIE_CODEC_ACTION_DELAY_WAIT, 0xbb8},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x83, 0x14, 0x14)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x86, 0xff, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x50, 0x40)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_DIGITAL_ANALOG_READY},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x8A, 0x10, 0x30)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x0D, 0xFF, 0x00)},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x83, 0x14, 0x00)},
	{ ADIE_CODEC_ACTION_STAGE_REACHED, ADIE_CODEC_ANALOG_OFF},
	{ ADIE_CODEC_ACTION_ENTRY,
	ADIE_CODEC_PACK_ENTRY(0x11, 0xff, 0x00)}
};

static struct adie_codec_hwsetting_entry iearpiece_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = iearpiece_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_8KHz_osr256_actions),
	}
};

static struct adie_codec_hwsetting_entry imic_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_settings),
};

static struct adie_codec_dev_profile imic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings,
	.setting_sz = ARRAY_SIZE(imic_settings),
};

static struct snddev_icodec_data snddev_iearpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = 1,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
};

static struct snddev_icodec_data snddev_imic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = 2,
	.profile = &imic_profile,
	.channel_mode = 1,
};

struct platform_device msm_iearpiece_device = {
	.name = "msm_snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data },
};

struct platform_device msm_imic_device = {
	.name = "msm_snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data },
};

static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)), 0,
	0, 0, 0,
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x8A000300,
		.end = 0x8A0003ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(156),
		.end = MSM_GPIO_TO_INT(156),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
};

static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
};

static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},

};
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.version	= 0x0100,
	.phy_info	= USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM,
	.vendor_id	= 0x5c6,
	.product_name	= "Qualcomm HSUSB Device",
	.serial_number	= "1234567890ABCDEF",
	.manufacturer_name
			= "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions
			= ARRAY_SIZE(usb_func_composition),
	.function_map	= usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.core_clk	= 1,
};

static struct msm_gpio bma_spi_gpio_config_data[] = {
	{ GPIO_CFG(51, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "bma_irq" },
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = "7k_handset",
	},
};

static int msm_bma_gpio_setup(struct device *dev)
{
	int rc;

	rc = msm_gpios_request_enable(bma_spi_gpio_config_data,
		ARRAY_SIZE(bma_spi_gpio_config_data));

	return rc;
}

static void msm_bma_gpio_teardown(struct device *dev)
{
	msm_gpios_disable_free(bma_spi_gpio_config_data,
		ARRAY_SIZE(bma_spi_gpio_config_data));
}

static struct bma150_platform_data bma_pdata = {
	.setup    = msm_bma_gpio_setup,
	.teardown = msm_bma_gpio_teardown,
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device qsd_device_spi = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

static struct spi_board_info msm_spi_board_info[] __initdata = {
	{
		.modalias	= "bma150",
		.mode		= SPI_MODE_3,
		.irq		= MSM_GPIO_TO_INT(51),
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 10000000,
		.platform_data	= &bma_pdata,
	}
};

static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{ GPIO_CFG(45, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_clk" },
	{ GPIO_CFG(46, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA), "spi_mosi" },
	{ GPIO_CFG(48, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_miso" },
};

static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26000000,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if (!strncmp(name, "mddi_toshiba_wvga_pt", 20))
		return -EPERM;
	else if (!strncmp(name, "lcdc_toshiba_wvga_pt", 20))
		return 0;
	else
		return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};


static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
       .name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	* the only valid choice at this time. The board structure is
	* set to all zeros by the C runtime initialization and that is now
	* the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	* include/linux/android_pmem.h.
	*/
       .cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
};

static struct platform_device android_pmem_kernel_ebi1_device = {
       .name = "android_pmem",
       .id = 1,
       .dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};


static struct kgsl_platform_data kgsl_pdata = {
	.max_axi_freq = 0, /*7x30 bringup, no request to made now*/
};

static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA3500000, /* 3D GRP address */
		.end = 0xA351ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "kgsl_phys_memory",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_GRP_3D,
		.end = INT_GRP_3D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

static unsigned wega_reset_gpio =
	GPIO_CFG(180, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);

static int toshiba_wega_power(int on)
{
	int rc;
	struct vreg *vreg_ldo12, *vreg_ldo15;
	struct vreg *vreg_ldo20, *vreg_ldo16;

	if (on) {
		/* reset Toshiba WeGA chip -- toggle reset pin -- gpio_180 */
		rc = gpio_tlmm_config(wega_reset_gpio, GPIO_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, wega_reset_gpio, rc);
			return rc;
		}

		gpio_set_value(180, 0);	/* bring reset line low to hold reset*/
	}

	/* Toshiba WeGA power -- has 3 power source */
	/* 1.5V -- LDO20*/
	vreg_ldo20 = vreg_get(NULL, "gp13");

	if (IS_ERR(vreg_ldo20)) {
		pr_err("%s: gp13 vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_ldo20));
		return rc;
	}

	/* 1.8V -- LDO12 */
	vreg_ldo12 = vreg_get(NULL, "gp9");

	if (IS_ERR(vreg_ldo12)) {
		pr_err("%s: gp9 vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_ldo12));
		return rc;
	}

	/* 2.6V -- LDO16 */
	vreg_ldo16 = vreg_get(NULL, "gp10");

	if (IS_ERR(vreg_ldo16)) {
		pr_err("%s: gp10 vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_ldo16));
		return rc;
	}

	/* lcd panel power */
	/* 3.1V -- LDO15 */
	vreg_ldo15 = vreg_get(NULL, "gp6");

	if (IS_ERR(vreg_ldo15)) {
		pr_err("%s: gp6 vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_ldo15));
		return rc;
	}

	rc = vreg_set_level(vreg_ldo20, 1500);
	if (rc) {
		pr_err("%s: vreg LDO20 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo12, 1800);
	if (rc) {
		pr_err("%s: vreg LDO12 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo16, 2600);
	if (rc) {
		pr_err("%s: vreg LDO16 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo15, 3100);
	if (rc) {
		pr_err("%s: vreg LDO15 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo20);
	else
		rc = vreg_disable(vreg_ldo20);

	if (rc) {
		pr_err("%s: LDO20 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo12);
	else
		rc = vreg_disable(vreg_ldo12);

	if (rc) {
		pr_err("%s: LDO12 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo16);
	else
		rc = vreg_disable(vreg_ldo16);

	if (rc) {
		pr_err("%s: LDO16 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo15);
	else
		rc = vreg_disable(vreg_ldo15);

	if (rc) {
		pr_err("%s: LDO15 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	mdelay(5);		/* ensure power is stable */

	if (on) {
		gpio_set_value(180, 1);	/* bring reset line high */
		mdelay(10);		/* 10 msec before IO can be accessed */
	}

	return 0;
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_on = toshiba_wega_power,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 30,
};

static int lcd_panel_spi_gpio_num[] = {
			45, /* spi_clk */
			46, /* spi_cs  */
			47, /* spi_mosi */
			48, /* spi_miso */
		};

static struct msm_gpio lcd_panel_gpios[] = {
	{ GPIO_CFG(18, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_grn0" },
	{ GPIO_CFG(19, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_grn1" },
	{ GPIO_CFG(20, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_blu0" },
	{ GPIO_CFG(21, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_blu1" },
	{ GPIO_CFG(22, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_blu2" },
	{ GPIO_CFG(23, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_red0" },
	{ GPIO_CFG(24, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_red1" },
	{ GPIO_CFG(25, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_red2" },
	{ GPIO_CFG(45, 0, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_clk" },
	{ GPIO_CFG(46, 0, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 0, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_mosi" },
	{ GPIO_CFG(48, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_miso" },
	{ GPIO_CFG(90, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_pclk" },
	{ GPIO_CFG(91, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_en" },
	{ GPIO_CFG(92, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_vsync" },
	{ GPIO_CFG(93, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_hsync" },
	{ GPIO_CFG(94, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_grn2" },
	{ GPIO_CFG(95, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_grn3" },
	{ GPIO_CFG(96, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_grn4" },
	{ GPIO_CFG(97, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_grn5" },
	{ GPIO_CFG(98, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_grn6" },
	{ GPIO_CFG(99, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_grn7" },
	{ GPIO_CFG(100, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_blu3" },
	{ GPIO_CFG(101, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_blu4" },
	{ GPIO_CFG(102, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_blu5" },
	{ GPIO_CFG(103, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_blu6" },
	{ GPIO_CFG(104, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_blu7" },
	{ GPIO_CFG(105, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_red3" },
	{ GPIO_CFG(106, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_red4" },
	{ GPIO_CFG(107, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_red5" },
	{ GPIO_CFG(108, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_red6" },
	{ GPIO_CFG(109, 1, GPIO_OUTPUT,  GPIO_NO_PULL, GPIO_2MA), "lcdc_red7" },
};

static void lcdc_toshiba_panel_power(int on)
{
	int rc;

	toshiba_wega_power(on);

	if (on) {
		rc = msm_gpios_enable(lcd_panel_gpios,
				ARRAY_SIZE(lcd_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio config failed: %d\n",
				__func__, rc);
		}
	} else
		msm_gpios_disable(lcd_panel_gpios,
				ARRAY_SIZE(lcd_panel_gpios));
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_toshiba_panel_power,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);
}

static struct msm_panel_common_pdata lcdc_toshiba_panel_data = {
	.gpio_num          = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_toshiba_panel_device = {
	.name   = "lcdc_toshiba_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_toshiba_panel_data,
	}
};

#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
};

static struct msm_gpio bt_config_power_on[] = {
	{ GPIO_CFG(134, 1, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(135, 1, GPIO_INPUT,  GPIO_NO_PULL,   GPIO_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(136, 1, GPIO_INPUT,  GPIO_NO_PULL,   GPIO_2MA),
		"UART1DM_Rx" },
	{ GPIO_CFG(137, 1, GPIO_OUTPUT, GPIO_NO_PULL,   GPIO_2MA),
		"UART1DM_Tx" }
};
static struct msm_gpio bt_config_power_off[] = {
	{ GPIO_CFG(134, 0, GPIO_INPUT,  GPIO_NO_PULL,   GPIO_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(135, 0, GPIO_INPUT,  GPIO_NO_PULL,   GPIO_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(136, 0, GPIO_INPUT,  GPIO_NO_PULL,   GPIO_2MA),
		"UART1DM_Rx" },
	{ GPIO_CFG(137, 0, GPIO_INPUT,  GPIO_NO_PULL,   GPIO_2MA),
		"UART1DM_Tx" }
};

static int marimba_bt(int on)
{
	int rc;
	int i;
	struct marimba config = { .mod_id = MARIMBA_SLAVE_ID_MARIMBA };

	struct marimba_config_register {
		u8 reg;
		u8 value;
		u8 mask;
	} *p;

	size_t config_size;

	struct marimba_config_register bt_on[] = {
		{ 0xE5, 0x0B, 0x0F },
		{ 0x05, 0x02, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
	};

	struct marimba_config_register bt_off[] = {
		{ 0xE5, 0x0B, 0x0F },
		{ 0x05, 0x02, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
	};

	p = on ? bt_on : bt_off;
	config_size = on ? ARRAY_SIZE(bt_on) : ARRAY_SIZE(bt_off);

	for (i = 0; i < config_size; i++) {
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&((p+i)->value),
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			printk(KERN_ERR
				"%s: reg %d write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
	}
	return 0;
}

static int bluetooth_power(int on)
{
	int rc;
	struct vreg *vreg_wlan;

	vreg_wlan = vreg_get(NULL, "wlan");

	if (IS_ERR(vreg_wlan)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_wlan));
		return PTR_ERR(vreg_wlan);
	}

	if (on) {
		rc = msm_gpios_enable(bt_config_power_on,
			ARRAY_SIZE(bt_config_power_on));

		if (rc < 0) {
			printk(KERN_ERR
				"%s: gpio config failed: %d\n",
				__func__, rc);
			return rc;
		}

		rc = vreg_set_level(vreg_wlan, PMIC_VREG_WLAN_LEVEL);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan set level failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
		rc = vreg_enable(vreg_wlan);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan enable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
		rc = marimba_bt(on);
		if (rc)
			return -EIO;
	} else {
		rc = marimba_bt(on);
		if (rc)
			return -EIO;
		rc = vreg_disable(vreg_wlan);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan disable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}

		rc = msm_gpios_enable(bt_config_power_off,
					ARRAY_SIZE(bt_config_power_off));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: gpio config failed: %d\n",
				__func__, rc);
			return rc;
		}
	}

	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

	return 0;
}

static void __init bt_power_init(void)
{
	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif

static struct platform_device msm_device_pmic_leds = {
	.name   = "pmic-leds",
	.id = -1,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
	&smc91x_device,
	&msm_device_nand,
	&msm_device_hsusb_peripheral,
	&mass_storage_device,
	&qsd_device_spi,
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi6,
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&lcdc_toshiba_panel_device,
	&android_pmem_kernel_ebi1_device,
	&android_pmem_adsp_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&msm_device_uart_dm1,
	&hs_device,
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_iearpiece_device,
	&msm_imic_device,
	&msm_device_adspdec,
	&qup_device_i2c,
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	&msm_bt_power_device,
#endif
	&android_pmem_gpu1_device,
	&msm_device_kgsl,
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	&msm_device_uart2,
#endif
	&msm_device_pmic_leds,
};

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), "i2c_sda" },
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(16, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(16, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA), "qup_sda" },
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}

static struct vreg *qup_vreg;
static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	if (qup_vreg) {
		int rc = vreg_set_level(qup_vreg, 1800);
		if (rc) {
			pr_err("%s: vreg LVS1 set level failed (%d)\n",
			__func__, rc);
		}
		rc = vreg_enable(qup_vreg);
		if (rc) {
			pr_err("%s: vreg_enable() = %d \n",
			__func__, rc);
		}
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.pri_clk = 70,
	.pri_dat = 71,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 100000,
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	qup_vreg = vreg_get(NULL, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(qup_vreg));
	}
}

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

struct vreg *vreg_s3;
struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
};

static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(38, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), "sdc1_clk"},
	{GPIO_CFG(39, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_cmd"},
	{GPIO_CFG(40, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_3"},
	{GPIO_CFG(41, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_2"},
	{GPIO_CFG(42, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_1"},
	{GPIO_CFG(43, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_0"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_0"},
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(58, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
};

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}

	return rc;
}

static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;
	static int enabled_once[] = {0, 0, 0, 0};

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (!enable || enabled_once[dev_id - 1])
		return 0;

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc) {
			printk(KERN_ERR "%s: vreg_set_level() = %d \n",
					__func__, rc);
		}
		rc = vreg_enable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		}
		enabled_once[dev_id - 1] = 1;
	} else {
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		}
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
};
#endif

static void __init msm7x30_init_mmc(void)
{
	vreg_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_s3));
		return;
	}

	vreg_mmc = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	sdcc_vreg_data[0].vreg_data = vreg_s3;
	sdcc_vreg_data[0].level = 1800;
	msm_add_sdcc(1, &msm7x30_sdc1_data, 0, 0);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	sdcc_vreg_data[1].vreg_data = vreg_s3;
	sdcc_vreg_data[1].level = 1800;
	msm_add_sdcc(2, &msm7x30_sdc2_data, 0, 0);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_s3;
	sdcc_vreg_data[2].level = 1800;
	msm_add_sdcc(3, &msm7x30_sdc3_data, 0, 0);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &msm7x30_sdc4_data, 0, 0);
#endif

}

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 10000,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 0,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 500,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

#ifdef CONFIG_SERIAL_MSM_CONSOLE
static struct msm_gpio uart2_config_data[] = {
	{ GPIO_CFG(49, 2, GPIO_OUTPUT,  GPIO_PULL_DOWN, GPIO_2MA), "UART2_RFR"},
	{ GPIO_CFG(50, 2, GPIO_INPUT,   GPIO_PULL_DOWN, GPIO_2MA), "UART2_CTS"},
	{ GPIO_CFG(51, 2, GPIO_INPUT,   GPIO_PULL_DOWN, GPIO_2MA), "UART2_Rx"},
	{ GPIO_CFG(52, 2, GPIO_OUTPUT,  GPIO_PULL_DOWN, GPIO_2MA), "UART2_Tx"},
};

static void msm7x30_init_uart2(void)
{
	msm_gpios_request_enable(uart2_config_data,
			ARRAY_SIZE(uart2_config_data));

}
#endif

static void __init msm7x30_init(void)
{
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm7x30_init_mmc();
	msm_qsd_spi_init();
	spi_register_board_info(msm_spi_board_info,
		ARRAY_SIZE(msm_spi_board_info));
	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data);
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	buses_init();
	msm7x30_init_marimba();
	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));

	platform_device_register(&surf_keypad_device);
	bt_power_init();
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	msm7x30_init_uart2();
#endif
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
	pmem_sf_size = memparse(*p, p);
}
__early_param("pmem_sf_size=", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static void __init fb_size_setup(char **p)
{
	fb_size = memparse(*p, p);
}
__early_param("fb_size=", fb_size_setup);

static unsigned pmem_gpu1_size = MSM_PMEM_GPU1_SIZE;
static void __init pmem_gpu1_size_setup(char **p)
{
	pmem_gpu1_size = memparse(*p, p);
}
__early_param("pmem_gpu1_size=", pmem_gpu1_size_setup);

static unsigned gpu_phys_size = MSM_GPU_PHYS_SIZE;
static void __init gpu_phys_size_setup(char **p)
{
	gpu_phys_size = memparse(*p, p);
}
__early_param("gpu_phys_size=", gpu_phys_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = pmem_gpu1_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_gpu1_pdata.start = __pa(addr);
		android_pmem_gpu1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for gpu1 "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = gpu_phys_size;
	if (size) {
		addr = alloc_bootmem(size);
		kgsl_resources[1].start = __pa(addr);
		kgsl_resources[1].end = kgsl_resources[1].start + size - 1;
		pr_info("allocating %lu bytes at %p (%lx physical) for "
			"KGSL\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

}

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x00100000;
	msm_map_msm7x30_io();
	msm7x30_allocate_memory_regions();
	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);
}

MACHINE_START(MSM7X30_SURF, "QCT MSM7X30 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = 0x00200100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

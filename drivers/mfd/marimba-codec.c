/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mfd/marimba-codec.h>
#include <linux/mfd/marimba.h>
#include <linux/err.h>

#define MARIMBA_CDC_RX_CTL 0x81
#define MARIMBA_CDC_RX_CTL_ST_EN_MASK 0x20
#define MARIMBA_CDC_RX_CTL_ST_EN_SHFT 0x5

static struct adie_codec_register adie_codec_tx_regs[] =
{
	{ 0x04, 0xc0, 0x8C },
	{ 0x0D, 0xFF, 0x00 },
	{ 0x0E, 0xFF, 0x00 },
	{ 0x0F, 0xFF, 0x00 },
	{ 0x10, 0xF8, 0x68 },
	{ 0x11, 0xFE, 0x00 },
	{ 0x12, 0xFE, 0x00 },
	{ 0x13, 0xFF, 0x58 },
	{ 0x14, 0xFF, 0x00 },
	{ 0x15, 0xFE, 0x00 },
	{ 0x16, 0xFF, 0x00 },
	{ 0x1A, 0xFF, 0x00 },
	{ 0x80, 0x01, 0x00 },
	{ 0x82, 0x7F, 0x18 },
	{ 0x83, 0x1C, 0x00 },
	{ 0x86, 0xFF, 0xAC },
	{ 0x87, 0xFF, 0xAC },
	{ 0x89, 0xFF, 0xFF },
	{ 0x8A, 0xF0, 0x30 }
};



static struct adie_codec_register adie_codec_rx_regs[] =
{
	{ 0x23, 0xF8, 0x00 },
	{ 0x24, 0x6F, 0x00 },
	{ 0x25, 0x7F, 0x00 },
	{ 0x26, 0xFC, 0x00 },
	{ 0x28, 0xFE, 0x00 },
	{ 0x29, 0xFE, 0x00 },
	{ 0x33, 0xFF, 0x00 },
	{ 0x34, 0xFF, 0x00 },
	{ 0x35, 0xFC, 0x00 },
	{ 0x36, 0xFE, 0x00 },
	{ 0x37, 0xFE, 0x00 },
	{ 0x38, 0xFE, 0x00 },
	{ 0x39, 0xF0, 0x00 },
	{ 0x3A, 0xFF, 0x0A },
	{ 0x3B, 0xFC, 0xAC },
	{ 0x3C, 0xFC, 0xAC },
	{ 0x3D, 0xFF, 0x55 },
	{ 0x3E, 0xFF, 0x55 },
	{ 0x3F, 0xCF, 0x00 },
	{ 0x40, 0x3F, 0x00 },
	{ 0x41, 0x3F, 0x00 },
	{ 0x42, 0xFF, 0x00 },
	{ 0x43, 0xF7, 0x00 },
	{ 0x43, 0xF7, 0x00 },
	{ 0x43, 0xF7, 0x00 },
	{ 0x43, 0xF7, 0x00 },
	{ 0x44, 0xF7, 0x00 },
	{ 0x45, 0xFF, 0x00 },
	{ 0x46, 0xFF, 0x00 },
	{ 0x47, 0xF7, 0x00 },
	{ 0x48, 0xF7, 0x00 },
	{ 0x49, 0xFF, 0x00 },
	{ 0x4A, 0xFF, 0x00 },
	{ 0x80, 0x02, 0x00 },
	{ 0x81, 0xFF, 0x4C },
	{ 0x83, 0x23, 0x00 },
	{ 0x84, 0xFF, 0xAC },
	{ 0x85, 0xFF, 0xAC },
	{ 0x88, 0xFF, 0xFF },
	{ 0x8A, 0x0F, 0x03 },
	{ 0x8B, 0xFF, 0xAC },
	{ 0x8C, 0x03, 0x01 },
	{ 0x8D, 0xFF, 0x00 },
	{ 0x8E, 0xFF, 0x00 }
};

static struct adie_codec_register adie_codec_lb_regs[] =
{
	{ 0x2B, 0x8F, 0x02 },
	{ 0x2C, 0x8F, 0x02 }
};

struct adie_codec_state {
	struct adie_codec_path path[ADIE_CODEC_MAX];
	u32 ref_cnt;
	struct marimba *pdrv_ptr;
	struct mutex lock;
};

static struct adie_codec_state adie_codec;

static int adie_codec_write(u8 reg, u8 mask, u8 val)
{
	int rc;
	u8 cur_val;

	rc = marimba_read(adie_codec.pdrv_ptr, reg, &cur_val, 1);
	if (IS_ERR_VALUE(rc)) {
		pr_err("%s: fail to read reg %x\n", __func__, reg);
		return -EIO;
	}
	cur_val = (cur_val & ~mask) | (val & mask);
	rc = marimba_write(adie_codec.pdrv_ptr, reg,  &cur_val, 1);
	if (IS_ERR_VALUE(rc)) {
		pr_err("%s: fail to write reg %x\n", __func__, reg);
		return -EIO;
	}

	pr_debug("%s: write reg %x val %x\n", __func__, reg, cur_val);

	return 0;
}

int adie_codec_setpath(struct adie_codec_path *path_ptr, u32 freq_plan, u32 osr)
{
	int rc = 0;
	u32 i, freq_idx = 0, freq = 0;

	if (path_ptr->curr_stage != ADIE_CODEC_DIGITAL_OFF) {
		rc = -EBUSY;
		goto error;
	}

	for (i = 0; i < path_ptr->profile->setting_sz; i++) {
		if (path_ptr->profile->settings[i].osr == osr) {
			if (path_ptr->profile->settings[i].freq_plan >=
				freq_plan) {
				if (freq == 0) {
					freq = path_ptr->profile->settings[i].
								freq_plan;
					freq_idx = i;
				} else if (path_ptr->profile->settings[i].
					freq_plan < freq) {
					freq = path_ptr->profile->settings[i].
								freq_plan;
					freq_idx = i;
				}
			}
		}
	}

	if (freq_idx >= path_ptr->profile->setting_sz)
		rc = -ENODEV;
	else {
		path_ptr->hwsetting_idx = freq_idx;
		path_ptr->stage_idx = 0;
	}

error:
	return rc;
}
EXPORT_SYMBOL(adie_codec_setpath);

u32 adie_codec_freq_supported(struct adie_codec_dev_profile *profile,
	u32 requested_freq)
{
	u32 i, rc = -EINVAL;

	for (i = 0; i < profile->setting_sz; i++) {
		if (profile->settings[i].freq_plan >= requested_freq) {
			rc = 0;
			break;
		}
	}
	return rc;
}
EXPORT_SYMBOL(adie_codec_freq_supported);

int adie_codec_enable_sidetone(struct adie_codec_path *rx_path_ptr,
	u32 enable)
{
	int rc = 0;

	pr_debug("%s()\n", __func__);

	mutex_lock(&adie_codec.lock);

	if (!rx_path_ptr || &adie_codec.path[ADIE_CODEC_RX] != rx_path_ptr) {
		pr_err("%s: invalid path pointer\n", __func__);
		rc = -EINVAL;
		goto error;
	} else if (rx_path_ptr->curr_stage !=
		ADIE_CODEC_DIGITAL_ANALOG_READY) {
		pr_err("%s: bad state\n", __func__);
		rc = -EPERM;
		goto error;
	}

	if (enable)
		rc = adie_codec_write(MARIMBA_CDC_RX_CTL,
		MARIMBA_CDC_RX_CTL_ST_EN_MASK,
		(0x1 << MARIMBA_CDC_RX_CTL_ST_EN_SHFT));
	else
		rc = adie_codec_write(MARIMBA_CDC_RX_CTL,
		MARIMBA_CDC_RX_CTL_ST_EN_MASK, 0);

error:
	mutex_unlock(&adie_codec.lock);
	return rc;
}
EXPORT_SYMBOL(adie_codec_enable_sidetone);

static void adie_codec_reach_stage_action(struct adie_codec_path *path_ptr,
	u32 stage)
{
	u32 iter;
	struct adie_codec_register *reg_info;

	if (stage == ADIE_CODEC_DIGITAL_OFF) {
		/* perform reimage */
		for (iter = 0; iter < path_ptr->img.img_sz; iter++) {
			reg_info = &path_ptr->img.regs[iter];
			adie_codec_write(reg_info->reg,
			reg_info->mask, reg_info->val);
		}
	}
}

int adie_codec_proceed_stage(struct adie_codec_path *path_ptr, u32 state)
{
	int rc = 0, loop_exit = 0;
	struct adie_codec_action_unit *curr_action;
	struct adie_codec_hwsetting_entry *setting;
	u8 reg, mask, val;

	mutex_lock(&adie_codec.lock);
	setting = &path_ptr->profile->settings[path_ptr->hwsetting_idx];
	while (!loop_exit) {
		curr_action = &setting->actions[path_ptr->stage_idx];
		switch (curr_action->type) {
		case ADIE_CODEC_ACTION_ENTRY:
			ADIE_CODEC_UNPACK_ENTRY(curr_action->action,
			reg, mask, val);
			adie_codec_write(reg, mask, val);
			break;
		case ADIE_CODEC_ACTION_DELAY_WAIT:
			udelay(curr_action->action);
			break;
		case ADIE_CODEC_ACTION_STAGE_REACHED:
			adie_codec_reach_stage_action(path_ptr,
				curr_action->action);
			if (curr_action->action == state) {
				path_ptr->curr_stage = state;
				loop_exit = 1;
			}
			break;
		default:
			BUG();
		}

		path_ptr->stage_idx++;
		if (path_ptr->stage_idx == setting->action_sz)
			path_ptr->stage_idx = 0;
	}
	mutex_unlock(&adie_codec.lock);
	return rc;
}
EXPORT_SYMBOL(adie_codec_proceed_stage);

int adie_codec_open(struct adie_codec_dev_profile *profile,
	struct adie_codec_path **path_pptr)
{
	int rc = 0;

	if (!profile || !path_pptr) {
		rc = -EINVAL;
		goto error;
	}

	mutex_lock(&adie_codec.lock);
	if (adie_codec.path[profile->path_type].profile) {
		rc = -EBUSY;
		goto error;
	}

	if (!adie_codec.ref_cnt) {
		/* bring up sequence for Marimba codec core
		 * ensure RESET_N = 0 and GDFS_CLAMP_EN=1 -
		 * set GDFS_EN_FEW=1 then GDFS_EN_REST=1 then
		 * GDFS_CLAMP_EN = 0 and finally RESET_N = 1
		 * Marimba codec bring up should use the Marimba
		 * slave address after which the codec slave
		 * address can be used
		 */

		/* Bring up codec */
		adie_codec_write(0xFF, 0xFF, 0x08);

		/* set GDFS_EN_FEW=1 */
		adie_codec_write(0xFF, 0xFF, 0x0a);

		/* set GDFS_EN_REST=1 */
		adie_codec_write(0xFF, 0xFF, 0x0e);

		/* set RESET_N=1 */
		adie_codec_write(0xFF, 0xFF, 0x07);

		adie_codec_write(0xFF, 0xFF, 0x17);

		/* enable band gap */
		adie_codec_write(0x03, 0xFF, 0x04);

		/* dither delay selected and dmic gain stage bypassed */
		adie_codec_write(0x8F, 0xFF, 0x44);

	}

	adie_codec.path[profile->path_type].profile = profile;
	*path_pptr = (void *) &adie_codec.path[profile->path_type];
	adie_codec.ref_cnt++;
	adie_codec.path[profile->path_type].hwsetting_idx = 0;
	adie_codec.path[profile->path_type].curr_stage = ADIE_CODEC_DIGITAL_OFF;
	adie_codec.path[profile->path_type].stage_idx = 0;

	mutex_unlock(&adie_codec.lock);

error:
	return rc;
}
EXPORT_SYMBOL(adie_codec_open);

int adie_codec_close(struct adie_codec_path *path_ptr)
{
	int rc = 0;


	if (!path_ptr) {
		rc = -EINVAL;
		goto error;
	}
	adie_codec_proceed_stage(path_ptr, ADIE_CODEC_DIGITAL_OFF);

	mutex_lock(&adie_codec.lock);

	BUG_ON(!adie_codec.ref_cnt);

	path_ptr->profile = NULL;
	adie_codec.ref_cnt--;

	if (!adie_codec.ref_cnt) {

		adie_codec_write(0xFF, 0xFF, 0x07);
		adie_codec_write(0xFF, 0xFF, 0x06);
		adie_codec_write(0xFF, 0xFF, 0x0e);
		adie_codec_write(0xFF, 0xFF, 0x08);
		adie_codec_write(0x03, 0xFF, 0x00);
	}
	mutex_unlock(&adie_codec.lock);
error:
	return rc;
}
EXPORT_SYMBOL(adie_codec_close);

static int marimba_codec_probe(struct platform_device *pdev)
{
	adie_codec.pdrv_ptr = platform_get_drvdata(pdev);

	return 0;
}

static struct platform_driver marimba_codec_driver = {
	.probe = marimba_codec_probe,
	.driver = {
		.name = "marimba_codec",
		.owner = THIS_MODULE,
	},
};

static int __init marimba_codec_init(void)
{
	s32 rc;

	rc = platform_driver_register(&marimba_codec_driver);
	if (IS_ERR_VALUE(rc))
		goto error;

	adie_codec.path[ADIE_CODEC_TX].img.regs = adie_codec_tx_regs;
	adie_codec.path[ADIE_CODEC_TX].img.img_sz =
	ARRAY_SIZE(adie_codec_tx_regs);
	adie_codec.path[ADIE_CODEC_RX].img.regs = adie_codec_rx_regs;
	adie_codec.path[ADIE_CODEC_RX].img.img_sz =
	ARRAY_SIZE(adie_codec_rx_regs);
	adie_codec.path[ADIE_CODEC_LB].img.regs = adie_codec_lb_regs;
	adie_codec.path[ADIE_CODEC_LB].img.img_sz =
	ARRAY_SIZE(adie_codec_lb_regs);
	mutex_init(&adie_codec.lock);
error:
	return rc;
}

static void __exit marimba_codec_exit(void)
{
	platform_driver_unregister(&marimba_codec_driver);
}

module_init(marimba_codec_init);
module_exit(marimba_codec_exit);

MODULE_DESCRIPTION("Marimba codec driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");

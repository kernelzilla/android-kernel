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
#include <linux/slab.h>

#include <mach/pmic.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_ard_adiei.h>
#include <mach/qdsp6/msm8k_adie_codec_rpc.h>
#include <mach/qdsp6/msm8k_adie_codec_dev.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_ard_clk.h>

struct adie_state_struct_type	adie_state;
static int pmic_is_stereo;
static int pmic_initialized;

static u32 adie_spkr_mono_ref1_cnt;
static u32 adie_spkr_stereo_ref1_cnt;

static u32 adie_spkr_mono_ref2_cnt;
static u32 adie_spkr_stereo_ref2_cnt;

#if 0
#define D(fmt, args...) printk(KERN_INFO "adie: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

u32 get_path_id(u32 dev_id)
{
	u32 adie_path_id = 0;

	switch (dev_id) {
	case CAD_HW_DEVICE_ID_HANDSET_MIC:
		adie_path_id = DAL_ADIE_CODEC_HANDSET_TX;
		break;
	case CAD_HW_DEVICE_ID_HANDSET_SPKR:
		adie_path_id = DAL_ADIE_CODEC_HANDSET_RX;
		break;
	case CAD_HW_DEVICE_ID_HEADSET_MIC:
		adie_path_id = DAL_ADIE_CODEC_HEADSET_MONO_TX;
		break;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO:
		adie_path_id = DAL_ADIE_CODEC_HEADSET_MONO_RX;
		break;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO:
		adie_path_id = DAL_ADIE_CODEC_HEADSET_STEREO_RX;
		break;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MIC:
		adie_path_id = DAL_ADIE_CODEC_SPEAKER_TX;
		break;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MONO:
		adie_path_id = DAL_ADIE_CODEC_SPEAKER_RX;
		break;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO:
		adie_path_id = DAL_ADIE_CODEC_SPEAKER_STEREO_RX;
		break;
	case CAD_HW_DEVICE_ID_TTY_HEADSET_MIC:
		adie_path_id = DAL_ADIE_CODEC_TTY_HEADSET_TX;
		break;
	case CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR:
		adie_path_id = DAL_ADIE_CODEC_TTY_HEADSET_RX;
		break;
	case CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_STEREO_RX:
		adie_path_id = DAL_ADIE_CODEC_SPKR_STEREO_HDPH_MONO_RX;
		break;
	case CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX:
		adie_path_id = DAL_ADIE_CODEC_SPKR_MONO_HDPH_MONO_RX;
		break;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO_LB:
		adie_path_id = DAL_ADIE_CODEC_AUXPGA_HDPH_STEREO_LB;
		break;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO_LB:
		adie_path_id = DAL_ADIE_CODEC_AUXPGA_HDPH_MONO_LB;
		break;
	case CAD_HW_DEVICE_ID_SPEAKER_SPKR_STEREO_LB:
		adie_path_id = DAL_ADIE_CODEC_AUXPGA_LINEOUT_STEREO_LB;
		break;
	case CAD_HW_DEVICE_ID_SPEAKER_SPKR_MONO_LB:
		adie_path_id = DAL_ADIE_CODEC_AUXPGA_LINEOUT_MONO_LB;
		break;
	case CAD_HW_DEVICE_ID_BT_SCO_MIC:
	case CAD_HW_DEVICE_ID_BT_SCO_SPKR:
	case CAD_HW_DEVICE_ID_BT_A2DP_SPKR:
	case CAD_HW_DEVICE_ID_BT_A2DP_TX:
	default:
		pr_err("ARD ADIE Paths not supported for dev_id %d\n", dev_id);
		break;
	}
	return adie_path_id;
}

u32 get_path_type(u32 cad_dev_type)
{
	u32 adie_dev_type;

	switch (cad_dev_type) {
	case CAD_RX_DEVICE:
		adie_dev_type = ADIE_CODEC_RX;
		break;
	case CAD_TX_DEVICE:
		adie_dev_type = ADIE_CODEC_TX;
		break;
	default:
		adie_dev_type = ADIE_CODEC_LB;
		break;
	}
	D("ARD ADIE DEV TYPE = %d\n", adie_dev_type);

	return adie_dev_type;
}

void pmic_init(void)
{
	s32	dal_rc;
	u8	dev_type;
	u32	stereo;

	dal_rc = CAD_RES_SUCCESS;
	dev_type = 0;

	if (pmic_spkr_is_stereo_en(&stereo))
		pmic_is_stereo = 0;
	else
		pmic_is_stereo = 1;

	D("pmic stereo out val = 0x%08x\n", stereo);

	/* Initialize the PMIC MIC and SPKR */
	if (pmic_is_stereo) {
		pmic_spkr_set_gain(LEFT_SPKR, SPKR_GAIN_PLUS12DB);
		pmic_spkr_set_gain(RIGHT_SPKR, SPKR_GAIN_PLUS12DB);
	} else
		pmic_set_speaker_gain(SPKR_GAIN_PLUS12DB);

	pmic_mic_set_volt(MIC_VOLT_1_80V);

	pmic_initialized = 1;
}

s32 adie_init(void)
{
	s32	dal_rc;
	u8	dev_type;

	dal_rc = CAD_RES_SUCCESS;
	dev_type = 0;

	dal_rc = daldevice_attach(DALDEVICEID_ADIE_CODEC, ADIE_CODEC_PORT_NAME,
			ADIE_CODEC_CPU, &adie_state.adie_handle);
	if (CAD_RES_SUCCESS != dal_rc)
		pr_err("ARD: ADIE Device Attach failed\n");

	adie_state.adie_opened = ADIE_FALSE;

	/* Initialize the rest of the state variables */
	for (dev_type = 0; dev_type < MAX_ADIE_PATH_TYPES; dev_type++) {
		adie_state.adie_path_type[dev_type].enabled = ADIE_FALSE;
		adie_state.adie_path_type[dev_type].enable_request = ADIE_FALSE;
		adie_state.adie_path_type[dev_type].state = ADIE_STATE_RESET;
	}

	return dal_rc;
}

s32 adie_dinit(void)
{
	s32	dal_rc;
	u8	dev_type;

	dal_rc = CAD_RES_SUCCESS;
	dev_type = 0;

	dal_rc = daldevice_detach(adie_state.adie_handle);
	if (CAD_RES_SUCCESS != dal_rc)
		pr_err("ARD: ADIE Device Detach failed\n");

	for (dev_type = 0; dev_type < MAX_ADIE_PATH_TYPES; dev_type++) {
		adie_state.adie_path_type[dev_type].enabled = ADIE_FALSE;
		adie_state.adie_path_type[dev_type].enable_request = ADIE_FALSE;
		adie_state.adie_path_type[dev_type].state = ADIE_STATE_RESET;
	}

	adie_state.adie_opened = ADIE_FALSE;

	return dal_rc;
}

s32 adie_open(u32 dev_type)
{
	s32 cad_rc, dal_rc;

	cad_rc = dal_rc = CAD_RES_SUCCESS;

	if (!pmic_initialized)
		pmic_init();

	if (adie_state.adie_opened != ADIE_TRUE) {
		dal_rc = daldevice_open(adie_state.adie_handle, 0);

		if (dal_rc != CAD_RES_SUCCESS) {
			pr_err("ARD ADIE DAL Open failed, dev_type = %d\n",
				dev_type);
			cad_rc = CAD_RES_FAILURE;
		} else {
			adie_state.adie_opened = ADIE_TRUE;
			D("ARD ADIE DRIVER OPENED\n");
		}
	}
	return cad_rc;
}


s32 adie_close(u32 dev_type)
{
	s32 rc;

	rc = CAD_RES_SUCCESS;

	if (adie_state.adie_opened == ADIE_TRUE) {
		if (((adie_state.adie_path_type[CAD_RX_DEVICE]).enabled
					== ADIE_TRUE) ||
			((adie_state.adie_path_type[CAD_TX_DEVICE]).enabled
					== ADIE_TRUE) ||
			((adie_state.adie_path_type[CAD_AUXPGA_DEVICE]).enabled
					== ADIE_TRUE)) {
			/* Do not close if at least 1 path is still enabled */
		} else {
			rc = daldevice_close(adie_state.adie_handle);
			adie_state.adie_opened = ADIE_FALSE;
		}
	}

	return rc;
}

s32 adie_enable(u32 dev_type, u32 dev_id)
{
	s32 rc = CAD_RES_SUCCESS;

	if (adie_state.adie_path_type[dev_type].enabled != ADIE_TRUE) {
		adie_state.adie_path_type[dev_type].enable_request = ADIE_TRUE;
		rc = adie_state_control(dev_type, dev_id);
		if (rc != CAD_RES_SUCCESS)
			pr_err("ADIE STATE M/C FAILED, dev_id = %d\n", dev_id);
		else
			D("ADIE ENABLED - PATH %d\n", dev_type);
	} else
		rc = CAD_RES_FAILURE;

	return rc;
}

s32 adie_disable(u32 dev_type, u32 dev_id)
{
	s32		rc = CAD_RES_SUCCESS;

	if (adie_state.adie_path_type[dev_type].enabled != ADIE_FALSE) {
		adie_state.adie_path_type[dev_type].enabled = ADIE_FALSE;

		rc = adie_state_control(dev_type, dev_id);
		if (rc != CAD_RES_SUCCESS)
			pr_err("ADIE STATE M/C FAILED, dev_id = %d\n", dev_id);
		else
			D("ADIE DISABLED\n");
	} else {
		pr_err("ADIE ALREADY DISABLED, dev_id = %d\n", dev_id);
		rc = CAD_RES_FAILURE;
	}

	return rc;
}

u32 adie_state_control(u32 dev_type, u32 dev_id)
{
	s32				cad_res;
	enum adie_state_ret_enum_type	rc;

	cad_res = CAD_RES_SUCCESS;
	rc = ADIE_STATE_RC_SUCCESS;

	do {
		D("ARD ADIE dev_type %d, adie state %d\n",
			dev_type, adie_state.adie_path_type[dev_type].state);

		switch (adie_state.adie_path_type[dev_type].state) {
		case ADIE_STATE_RESET:
			rc = adie_state_reset(dev_type, dev_id);
			break;
		case ADIE_STATE_DIGITAL_ACTIVE:
			rc = adie_state_digital_active(dev_type, dev_id);
			break;
		case ADIE_STATE_DIGITAL_ANALOG_ACTIVE:
			rc = adie_state_digital_analog_active(dev_type, dev_id);
			break;
		default:
			break;
		}
	} while (rc == ADIE_STATE_RC_CONTINUE);

	if (rc == ADIE_STATE_RC_FAILURE)
		cad_res = CAD_RES_FAILURE;

	return cad_res;
}

enum adie_state_ret_enum_type adie_state_reset(u32 dev_type, u32 dev_id)
{
	enum adie_state_ret_enum_type	rc;
	s32				dal_rc;
	enum adie_codec_path_type_enum	path_type;
	u32				path_id;
	u32				freq_plan;

	rc = ADIE_STATE_RC_SUCCESS;
	dal_rc = CAD_RES_SUCCESS;
	if (adie_state.adie_path_type[dev_type].enable_request != ADIE_TRUE) {
		rc = ADIE_STATE_RC_SUCCESS;
		adie_state.adie_path_type[dev_type].enabled = ADIE_FALSE;
		goto done;
	}

	path_id = get_path_id(dev_id);
	path_type = get_path_type(dev_type);

	if (path_type == ADIE_CODEC_RX)
		freq_plan = 48000;
	else {
		if (g_clk_info.tx_clk_freq > 16000)
			freq_plan = 48000;
		else if (g_clk_info.tx_clk_freq > 8000)
			freq_plan = 16000;
		else
			freq_plan = 8000;
	}

	/* Set the path */
	dal_rc = rpc_adie_codec_set_path(adie_state.adie_handle, path_id,
			path_type);

	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD ADIE Set Path failed for dev_type %d\n", dev_type);
		rc = ADIE_STATE_RC_FAILURE;
		goto done;
	}

	/* Set the freq plan */
	dal_rc = rpc_adie_codec_set_path_freq_plan(adie_state.adie_handle,
			path_type, freq_plan);

	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD ADIE Set Path Freq Plan failed for dev_type %d\n",
			dev_type);
		rc = ADIE_STATE_RC_FAILURE;
		goto done;
	}

	/* Proceed to stage */
	dal_rc = rpc_adie_codec_proceed_to_stage(adie_state.adie_handle,
			path_type, ADIE_CODEC_DIGITAL_READY);

	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD ADIE Proceed to Stage failed for dev_type %d\n",
			dev_type);
		rc = ADIE_STATE_RC_FAILURE;
		goto done;
	}

	adie_state.adie_path_type[dev_type].state = ADIE_STATE_DIGITAL_ACTIVE;
	rc = ADIE_STATE_RC_CONTINUE;
done:
	return rc;
}

static int is_speaker_mono(u32 dev_id)
{
	if ((dev_id == CAD_HW_DEVICE_ID_SPKR_PHONE_MONO) ||
		(dev_id == CAD_HW_DEVICE_ID_SPEAKER_SPKR_MONO_LB) ||
		(dev_id == CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX))
		return 1;
	else
		return 0;
}

static int is_speaker_stereo(u32 dev_id)
{
	if ((CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO == dev_id) ||
		(CAD_HW_DEVICE_ID_SPEAKER_SPKR_STEREO_LB == dev_id) ||
		(CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_STEREO_RX
				== dev_id))
		return 1;
	else
		return 0;
}

enum adie_state_ret_enum_type adie_state_digital_active(u32 dev_type,
							u32 dev_id)
{
	enum adie_state_ret_enum_type	rc;
	enum adie_codec_path_type_enum	path_type;
	s32				dal_rc;

	rc = ADIE_STATE_RC_SUCCESS;
	dal_rc = CAD_RES_SUCCESS;

	path_type = get_path_type(dev_type);
	if (adie_state.adie_path_type[dev_type].enable_request == ADIE_TRUE) {

		/* Prepare the PMIC, if necessary. Configure and power on,
		   but mute it until codec output is ready. */
		if (path_type == ADIE_CODEC_TX)
			if ((dev_id == CAD_HW_DEVICE_ID_HANDSET_MIC) ||
			    (dev_id == CAD_HW_DEVICE_ID_HEADSET_MIC) ||
			    (dev_id == CAD_HW_DEVICE_ID_SPKR_PHONE_MIC))
				pmic_mic_en(ON_CMD);
			else
				/* need to turn off MIC bias
				   for TTY_HEADSET_MIC */
				pmic_mic_en(OFF_CMD);
		else if ((path_type == ADIE_CODEC_RX) ||
			(path_type == ADIE_CODEC_LB)) {
			struct spkr_config_mode scm;
			memset(&scm, 0, sizeof(struct spkr_config_mode));

			if (is_speaker_mono(dev_id)) {
				if (!adie_spkr_mono_ref1_cnt) {
					if (pmic_is_stereo) {

						scm.is_right_chan_en = 0;
						scm.is_left_chan_en = 1;
						scm.is_stereo_en = 0;
						scm.is_hpf_en = 1;

						pmic_spkr_en_mute(
							LEFT_SPKR, 0);
						pmic_spkr_en_mute(
							RIGHT_SPKR, 0);

						pmic_set_spkr_configuration(
								&scm);

						pmic_spkr_en(LEFT_SPKR, 1);
						pmic_spkr_en(RIGHT_SPKR, 0);
					} else {
						pmic_speaker_cmd(SPKR_MUTE_ON);
						pmic_speaker_cmd(SPKR_ENABLE);
					}
				}

				/* keep a reference for stereo speaker.
				   case when Rx spkr is disabled when LB speaker
				   is already enabled. */
				adie_spkr_mono_ref1_cnt++;
			} else if (is_speaker_stereo(dev_id)) {
				if (!adie_spkr_stereo_ref1_cnt) {
					if (pmic_is_stereo) {

						scm.is_right_chan_en = 1;
						scm.is_left_chan_en = 1;
						scm.is_stereo_en = 1;
						scm.is_hpf_en = 1;

						pmic_spkr_en_mute(
							LEFT_SPKR, 0);
						pmic_spkr_en_mute(
							RIGHT_SPKR, 0);

						pmic_set_spkr_configuration(
								&scm);

						pmic_spkr_en(LEFT_SPKR, 1);
						pmic_spkr_en(RIGHT_SPKR, 1);
					} else {
						pmic_speaker_cmd(SPKR_MUTE_ON);
						pmic_speaker_cmd(SPKR_ENABLE);
					}
				}

				/* keep a reference for stereo speaker.
				   case when Rx spkr is disabled when LB speaker
				   is already enabled. */
				adie_spkr_stereo_ref1_cnt++;
			}
		} else {
			pr_err("bad path type\n");
		}

		/* Proceed to next stage */
		dal_rc = rpc_adie_codec_proceed_to_stage(adie_state.adie_handle,
				path_type, ADIE_CODEC_DIGITAL_ANALOG_READY);

		if (dal_rc != CAD_RES_SUCCESS) {
			pr_err("ARD ADIE Proceed to Stage failed,"
				" dev_type %d\n", dev_type);
			rc = ADIE_STATE_RC_FAILURE;
			goto done;
		}

		adie_state.adie_path_type[dev_type].state =
			ADIE_STATE_DIGITAL_ANALOG_ACTIVE;
		adie_state.adie_path_type[dev_type].enabled = ADIE_TRUE;
		adie_state.adie_path_type[dev_type].enable_request = ADIE_FALSE;
		rc = ADIE_STATE_RC_CONTINUE;
	} else {

		if (path_type == ADIE_CODEC_TX)
			pmic_mic_en(OFF_CMD);
		else if ((path_type == ADIE_CODEC_RX) ||
				(path_type == ADIE_CODEC_LB)) {
			if (is_speaker_mono(dev_id)) {

				/* disable a speaker LB or RX */
				adie_spkr_mono_ref1_cnt--;

				/* if no active speaker ref then disable pmic */
				if (!adie_spkr_mono_ref1_cnt) {
					if (pmic_is_stereo) {
						pmic_spkr_en(LEFT_SPKR, 0);
						pmic_spkr_en(RIGHT_SPKR, 0);
					} else
						pmic_speaker_cmd(SPKR_DISABLE);
				}

			} else if (is_speaker_stereo(dev_id)) {

				/* disable a speaker LB or RX */
				adie_spkr_stereo_ref1_cnt--;

				/* if no active speaker ref then disable pmic */
				if (!adie_spkr_stereo_ref1_cnt) {
					if (pmic_is_stereo) {
						pmic_spkr_en(LEFT_SPKR, 0);
						pmic_spkr_en(RIGHT_SPKR, 0);
					} else
						pmic_speaker_cmd(SPKR_DISABLE);
				}
			}
		}




		/* Proceed to digital off stage */
		dal_rc = rpc_adie_codec_proceed_to_stage(adie_state.adie_handle,
				path_type, ADIE_CODEC_DIGITAL_OFF);

		if (dal_rc != CAD_RES_SUCCESS) {
			pr_err("ARD ADIE Proceed to Stage failed,"
				" dev_type %d\n", dev_type);
			rc = ADIE_STATE_RC_FAILURE;
			goto done;
		}

		adie_state.adie_path_type[dev_type].state = ADIE_STATE_RESET;
		rc = ADIE_STATE_RC_CONTINUE;
	}
done:
	return rc;
}

enum adie_state_ret_enum_type adie_state_digital_analog_active(u32 dev_type,
							       u32 dev_id)
{
	s32				dal_rc;
	enum adie_state_ret_enum_type   rc;
	enum adie_codec_path_type_enum	path_type;

	dal_rc = CAD_RES_SUCCESS;
	rc = ADIE_STATE_RC_SUCCESS;
	path_type = get_path_type(dev_type);

	if (adie_state.adie_path_type[dev_type].enabled == ADIE_TRUE) {
		/* Stay in this state till teardown or reconfigure */

		if ((path_type == ADIE_CODEC_RX) ||
			(path_type == ADIE_CODEC_LB)) {
			if (is_speaker_mono(dev_id)) {
				/* make sure its not already on */
				if (!adie_spkr_mono_ref2_cnt) {
					if (pmic_is_stereo)
						pmic_spkr_en_mute(
							LEFT_SPKR, 1);
					else
						pmic_speaker_cmd(
							SPKR_MUTE_OFF);
				}

				/* increment ref count for LB and RX devices */
				adie_spkr_mono_ref2_cnt++;

			} else if (is_speaker_stereo(dev_id)) {

				/* make sure its not already on */
				if (!adie_spkr_stereo_ref2_cnt) {
					if (pmic_is_stereo) {
						pmic_spkr_en_mute(
							LEFT_SPKR, 1);
						pmic_spkr_en_mute(
							RIGHT_SPKR, 1);
					} else {
						pmic_speaker_cmd(
							SPKR_MUTE_OFF);
					}
				}

				/* increment ref count for LB and RX devices */
				adie_spkr_stereo_ref2_cnt++;
			}
		} else {
			D("ARD ADIE Loopback Device\n");
		}

	} else {

		if ((path_type == ADIE_CODEC_RX) ||
			(path_type == ADIE_CODEC_LB)) {
			if (is_speaker_mono(dev_id)) {

				/* decrement ref count for LB or RX device */
				adie_spkr_mono_ref2_cnt--;

				if (!adie_spkr_mono_ref2_cnt) {
					if (pmic_is_stereo)
						pmic_spkr_en_mute(
							LEFT_SPKR, 0);
					else
						pmic_speaker_cmd(SPKR_MUTE_ON);
				}
			} else if (is_speaker_stereo(dev_id)) {

				/* decrement ref count for LB/RX device */
				adie_spkr_stereo_ref2_cnt--;

				if (!adie_spkr_stereo_ref2_cnt) {
					if (pmic_is_stereo) {
						pmic_spkr_en_mute(
							LEFT_SPKR, 0);
						pmic_spkr_en_mute(
							RIGHT_SPKR, 0);
					} else {
						pmic_speaker_cmd(SPKR_MUTE_ON);
					}
				}
			}
		} else {
			D("ARD ADIE Loopback Device\n");
		}

		/* Proceed to digital off stage */
		dal_rc = rpc_adie_codec_proceed_to_stage(adie_state.adie_handle,
				path_type, ADIE_CODEC_ANALOG_OFF);

		if (dal_rc != CAD_RES_SUCCESS) {
			pr_err("ARD ADIE Proceed to Stage failed,"
				" dev_type %d\n", dev_type);
			rc = ADIE_STATE_RC_FAILURE;
			goto done;
		}

		adie_state.adie_path_type[dev_type].state =
			ADIE_STATE_DIGITAL_ACTIVE;
		adie_state.adie_path_type[dev_type].enable_request = ADIE_FALSE;
		rc = ADIE_STATE_RC_CONTINUE;
	}
done:
	return rc;
}


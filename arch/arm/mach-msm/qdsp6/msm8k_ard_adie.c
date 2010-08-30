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

#include <mach/pmic.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_ard_adiei.h>
#include <mach/qdsp6/msm8k_adie_codec_rpc.h>
#include <mach/qdsp6/msm8k_adie_codec_dev.h>
#include <mach/qdsp6/msm8k_cad_devices.h>

struct adie_state_struct_type	adie_state;

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
	case CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO:
		adie_path_id = DAL_ADIE_CODEC_SPEAKER_RX;
		break;
	case CAD_HW_DEVICE_ID_TTY_HEADSET_MIC:
		adie_path_id = DAL_ADIE_CODEC_TTY_HEADSET_TX;
		break;
	case CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR:
		adie_path_id = DAL_ADIE_CODEC_TTY_HEADSET_RX;
		break;
	case CAD_HW_DEVICE_ID_BT_SCO_MIC:
	case CAD_HW_DEVICE_ID_BT_SCO_SPKR:
	case CAD_HW_DEVICE_ID_BT_A2DP_SPKR:
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

	/* Initialize the PMIC MIC and SPKR */
	set_speaker_gain(SPKR_GAIN_PLUS04DB);
	mic_set_volt(MIC_VOLT_1_80V);
	speaker_cmd(SPKR_ENABLE);

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

	speaker_cmd(SPKR_DISABLE);

	adie_state.adie_opened = ADIE_FALSE;

	return dal_rc;
}

s32 adie_open(u32 dev_type)
{
	s32 cad_rc, dal_rc;

	cad_rc = dal_rc = CAD_RES_SUCCESS;


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

	if (adie_state.adie_opened != ADIE_TRUE)
		goto done;

	/* Do not close if at least 1 path is still enabled */
	if (((adie_state.adie_path_type[0]).enabled != ADIE_TRUE) &&
		((adie_state.adie_path_type[1]).enabled != ADIE_TRUE)) {

		daldevice_close(adie_state.adie_handle);
		adie_state.adie_opened = ADIE_FALSE;
	}
done:
	return rc;
}

s32 adie_enable(u32 dev_type, u32 dev_id)
{
	s32 rc = CAD_RES_SUCCESS;

	if (adie_state.adie_path_type[dev_type].enabled == ADIE_TRUE) {
		rc = CAD_RES_FAILURE;
		goto done;
	}
	adie_state.adie_path_type[dev_type].enable_request = ADIE_TRUE;

	rc = adie_state_control(dev_type, dev_id);
	if (rc != CAD_RES_SUCCESS)
		pr_err("ADIE STATE M/C FAILED, dev_id = %d\n", dev_id);
	else
		D("ADIE ENABLED - PATH %d\n", dev_type);
done:
	return rc;
}

s32 adie_disable(u32 dev_type, u32 dev_id)
{
	s32		rc = CAD_RES_SUCCESS;

	if (adie_state.adie_path_type[dev_type].enabled == ADIE_FALSE) {
		pr_err("ADIE ALREADY DISABLED, dev_id = %d\n", dev_id);
		rc = CAD_RES_FAILURE;
		goto done;
	}

	adie_state.adie_path_type[dev_type].enabled = ADIE_FALSE;

	rc = adie_state_control(dev_type, dev_id);
	if (rc != CAD_RES_SUCCESS)
		pr_err("ADIE STATE M/C FAILED, dev_id = %d\n", dev_id);
	else
		D("ADIE DISABLED\n");
done:
	return rc;
}

u32 adie_state_control(u32 dev_type, u32 dev_id)
{
	s32				cad_res;
	enum adie_state_ret_enum_type	rc;

	cad_res = CAD_RES_SUCCESS;
	rc = ADIE_STATE_RC_SUCCESS;

	do {
		D("ARD ADIE dev_type %d, adie state %d",
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
	else
		freq_plan = 8000;

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
		if (path_type == ADIE_CODEC_TX) {
			mic_en(1);
		} else if ((path_type == ADIE_CODEC_RX) &&
			(dev_id == CAD_HW_DEVICE_ID_SPKR_PHONE_MONO)) {

			speaker_cmd(SPKR_ON);
			spkr_en_left_chan(1);
		} else {
			D("ARD ADIE Loopback Device\n");
		}

	} else {
		/*Turn off the PMIC if it's a TX*/
		if (path_type == ADIE_CODEC_TX) {
			mic_en(0);
		} else if ((path_type == ADIE_CODEC_RX) &&
			(dev_id == CAD_HW_DEVICE_ID_SPKR_PHONE_MONO)) {

			speaker_cmd(SPKR_OFF);
			spkr_en_left_chan(0);
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


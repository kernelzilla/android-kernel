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

#include <linux/slab.h>
#include <mach/qdsp6/msm8k_ard_q6.h>
#include <mach/qdsp6/msm8k_ardi.h>
#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_itypes.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_q6dec_drvi.h>
#include <mach/qdsp6/msm8k_cad_q6enc_drvi.h>
#include <mach/qdsp6/msm8k_q6_api_flip_utils.h>
#include <mach/qdsp6/msm8k_adsp_audio_error.h>
#include <mach/qdsp6/msm8k_adsp_audio_device.h>
#include <mach/qdsp6/msm8k_adsp_audio_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_stream_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_command.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

s32 qdsp6_open(s32 session_id)
{
	s32					rc, dal_rc;
	struct cadi_open_struct_type		*ref_cadr = NULL;
	struct cad_stream_device_struct_type	*ref_cadr_device = NULL;
	struct cad_stream_info_struct_type	*ref_cadr_stream = NULL;

	struct adsp_audio_open_command		*cadr = NULL;
	union adsp_audio_event			*evt_buf = NULL;

	rc = dal_rc = CAD_RES_SUCCESS;


	ref_cadr = ardsession[session_id]->sess_open_info;
	ref_cadr_stream = &(ref_cadr->cad_stream);
	ref_cadr_device = &(ref_cadr->cad_device);

	if (ref_cadr_stream->app_type == CAD_STREAM_APP_VOICE) {
		D("ARD Q6 OPEN VOICE Session, so do nothing %d\n",
			session_id);
		goto done;
	}

	/* Allocate memory for the event payload */
	evt_buf = kmalloc(sizeof(*evt_buf),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	cadr = kmalloc(sizeof(*cadr),
		GFP_KERNEL);

	if (cadr == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD kmalloc failed\n");
		goto done;
	}

	memset(cadr, 0, sizeof(*cadr));
	cadr->cmd.op_code = q6_open_op_mapping(ref_cadr->cad_open.op_code);
	cadr->cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;
	if (ref_cadr->cad_open.op_code != CAD_OPEN_OP_DEVICE_CTRL) {
		cadr->open_data.stream_context =
			q6_stream_context_mapping(
			ref_cadr->cad_stream.app_type,
			&(cadr->open_data.mode));

		cadr->open_data.buf_max_size =
			ref_cadr->cad_stream.ses_buf_max_size;

		if (ref_cadr->cad_device.device_len >
			ADSP_AUDIO_MAX_DEVICES) {

			pr_err("CAD:ARD device number is too large\n");
			rc = CAD_RES_FAILURE;
			goto done;
		}

		if (ref_cadr->cad_device.device == NULL) {
			pr_err("CAD:ARD no device for control session\n");
			rc = CAD_RES_FAILURE;
			goto done;
		}

		cadr->open_data.device = q6_device_id_mapping
			(ref_cadr->cad_device.device[0]);

		D("ARD device id = %d \n", cadr->open_data.device);

		rc = convert_format_block(session_id, cadr, ref_cadr);
		if (rc != CAD_RES_SUCCESS) {
			pr_err("ARD converting format block failed\n");
			rc = CAD_RES_FAILURE;
			goto done;
		}
	}

	if (ardsession[session_id]->qdsp6_opened == ARD_FALSE) {
		/* Only ARD will open a session with Q6 */
		D("ARD Sending RPC Open, session %d\n", session_id);
		dal_rc = cad_rpc_control(session_id,
			ardsession[session_id]->group_id, (void *)cadr,
			sizeof(*cadr), evt_buf);

		if (dal_rc != CAD_RES_SUCCESS) {
			rc = CAD_RES_FAILURE;
			pr_err("ARD RPC Open failed %d\n", session_id);
			goto done;
		}

		if ((evt_buf->no_payload.source.minor == (u32)session_id) &&
			(evt_buf->no_payload.status != ADSP_AUDIO_SUCCESS)) {

			pr_err("ARD Open RPC failed for ses: %d, status %d\n",
				session_id, evt_buf->no_payload.status);
			rc = CAD_RES_FAILURE;
			goto done;
		}

		ardsession[session_id]->qdsp6_opened = ARD_TRUE;
	}

done:
	if (ardsession[session_id]->local_format_block)
		ardsession[session_id]->local_format_block = NULL;

	kfree(evt_buf);
	kfree(cadr);

	return rc;
}


s32 qdsp6_start(s32 session_id)
{
	s32					rc, dal_rc;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	union adsp_audio_event			*evt_buf = NULL;
	struct adsp_audio_no_payload_command	q6_cmd;

	rc = dal_rc = CAD_RES_SUCCESS;


	memset(&q6_cmd, 0, sizeof(q6_cmd));

	cadr = ardsession[session_id]->sess_open_info;
	cadr_stream = &(cadr->cad_stream);

	if (cadr_stream->app_type == CAD_STREAM_APP_VOICE) {
		D("ARD Q6 START VOICE Session, so do nothing %d\n",
			session_id);
		goto done;
	}

	/* Allocate memory for the event payload */
	evt_buf = kmalloc(sizeof(*evt_buf),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	D("ARD Sending RPC CADI_IOCTL_CMD_DSP_START, session %d\n",
		session_id);

	if (cadr->cad_open.op_code != CAD_OPEN_OP_DEVICE_CTRL) {
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_SESSION_START;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		/* Send START IOCTL CMD - This command has no payload */
		dal_rc = cad_rpc_control(session_id,
			ardsession[session_id]->group_id,
			(void *)&q6_cmd,
			sizeof(q6_cmd), evt_buf);
	} else {
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		/* Send DEVICE_SWITCH_COMMIT - This command has no payload */
		dal_rc = cad_rpc_control(session_id,
			ardsession[session_id]->group_id,
			(void *)&q6_cmd,
			sizeof(q6_cmd), evt_buf);
	}


	if (dal_rc != CAD_RES_SUCCESS) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD CADI_IOCTL_CMD_DSP_START failed %d\n",
			session_id);
		goto done;
	}

	if ((evt_buf->no_payload.source.minor == (u32)session_id) &&
		(evt_buf->no_payload.status != ADSP_AUDIO_SUCCESS)) {
		pr_err("ARD CADI_IOCTL_CMD_DSP_START failed ses: %d, "
			"status %d\n", session_id,
			evt_buf->no_payload.status);
		rc = CAD_RES_FAILURE;
		goto done;
	}

done:
	kfree(evt_buf);

	return rc;
}


s32 qdsp6_close(s32 session_id)
{
	s32					rc, dal_rc;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_info_struct_type      *cadr_stream = NULL;
	union adsp_audio_event			*evt_buf = NULL;
	struct adsp_audio_no_payload_command	q6_cmd;

	rc = dal_rc = CAD_RES_SUCCESS;

	memset(&q6_cmd, 0, sizeof(q6_cmd));

	cadr = ardsession[session_id]->sess_open_info;
	cadr_stream = &(cadr->cad_stream);

	if (cadr_stream->app_type == CAD_STREAM_APP_VOICE) {
		D("ARD Q6 CLOSE VOICE Session, so do nothing %d\n",
			session_id);
		goto done;
	}

	/* Allocate memory for the event payload */
	evt_buf = kmalloc(sizeof(*evt_buf),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	if (ardsession[session_id]->qdsp6_opened == ARD_TRUE) {
		D("ARD Sending RPC Close, session %d\n", session_id);
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_CLOSE;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		/* Send IOCTL_CMD_CLOSE - This command has no payload */
		dal_rc = cad_rpc_control(session_id,
			ardsession[session_id]->group_id,
			(void *)&q6_cmd,
			sizeof(q6_cmd), evt_buf);

		if (dal_rc != CAD_RES_SUCCESS) {
			pr_err("ARD RPC Close failed %d\n", session_id);
			rc = CAD_RES_FAILURE;
			goto done;
		}

		ardsession[session_id]->qdsp6_opened = ARD_FALSE;
		ardsession[session_id]->qdsp6_started = ARD_FALSE;

		if ((evt_buf->no_payload.source.minor == (u32)session_id) &&
			(evt_buf->no_payload.status != ADSP_AUDIO_SUCCESS)) {

			pr_err("ARD Close RPC failed for session %d,"
				" status %d\n", session_id,
				evt_buf->no_payload.status);
			rc = CAD_RES_FAILURE;
			goto done;
		}
	}
done:
	kfree(evt_buf);

	return rc;
}



s32 qdsp6_devchg_notify(s32 session_id, u32 dev_id)
{
	s32					rc, dal_rc;
	struct adsp_audio_device_switch_command	dev_chg;
	struct ard_state_struct_type		*local_ard_state = NULL;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	union adsp_audio_event			*evt_buf = NULL;

	rc = dal_rc = CAD_RES_SUCCESS;

	cadr = ardsession[session_id]->sess_open_info;
	cadr_stream = &(cadr->cad_stream);

	local_ard_state = &ard_state;


	/* Check if handle opened with the QDSP6 */
	if (ardsession[session_id]->qdsp6_opened != ARD_TRUE)
		qdsp6_open(session_id);

	if (cadr_stream->app_type == CAD_STREAM_APP_VOICE) {
		pr_err("ARD Q6 DEV NOTIFY VOICE Session, so do nothing %d\n",
			session_id);
		goto done;
	}

	/* Allocate memory for the event payload */
	evt_buf = kmalloc(sizeof(*evt_buf),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	dev_chg.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE;
	dev_chg.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

	/*0 - Default - Default Device*/
	dev_chg.device_class = 0;
	dev_chg.device_type = q6_device_direction_mapping
		(local_ard_state->ard_device[dev_id].device_type);

	switch (dev_chg.device_type) {
	case ADSP_AUDIO_RX_DEVICE:
		dev_chg.new_device =
			q6_device_id_mapping(local_ard_state->new_rx_device);
		dev_chg.old_device =
			q6_device_id_mapping(local_ard_state->def_rx_device);
		break;
	case ADSP_AUDIO_TX_DEVICE:
		dev_chg.new_device =
			q6_device_id_mapping(local_ard_state->new_tx_device);
		dev_chg.old_device =
			q6_device_id_mapping(local_ard_state->def_tx_device);
		break;
	default:
		pr_err("ARD DAL RPC IOCTL failed %d\n", session_id);
		break;
	}

	D("ARD Sending RPC CMD_DEVICE_SWITCH_PREPARE, session %d\n",
		session_id);

	dal_rc = cad_rpc_control(session_id,
			ardsession[session_id]->group_id,
			(void *)&dev_chg,
			sizeof(dev_chg), evt_buf);

	if (dal_rc != CAD_RES_SUCCESS) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD CADI_IOCTL_CMD_DSP_PREP_DEV_CHG failed %d\n",
			session_id);
		goto done;
	}

	if ((evt_buf->no_payload.source.minor == (u32)session_id) &&
		(evt_buf->no_payload.status != ADSP_AUDIO_SUCCESS)) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD DSP_PREP_DEV_CHG failed, ses %d, status %d\n",
			session_id, evt_buf->no_payload.status);
		goto done;
	}
done:
	kfree(evt_buf);

	return rc;
}


s32 qdsp6_standby(s32 session_id)
{
	/*Send IOCTL to Q6*/
	s32					rc, dal_rc;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	union adsp_audio_event			*evt_buf = NULL;
	struct adsp_audio_no_payload_command	q6_cmd;

	rc = dal_rc = CAD_RES_SUCCESS;


	memset(&q6_cmd, 0, sizeof(q6_cmd));

	cadr = ardsession[session_id]->sess_open_info;
	cadr_stream = &(cadr->cad_stream);

	/*Check if handle opened with the QDSP6*/
	if (ardsession[session_id]->qdsp6_opened != ARD_TRUE)
		qdsp6_open(session_id);

	if (cadr_stream->app_type == CAD_STREAM_APP_VOICE) {
		D("ARD Q6 Standby VOICE Session, so do nothing %d\n",
			session_id);
		goto done;
	}

	/* Allocate memory for the event payload */
	evt_buf = kmalloc(sizeof(*evt_buf),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	if (ardsession[session_id]->session_type == DEVICE_CTRL_TYPE) {
		D("ARD Sending RPC CADI_IOCTL_CMD_DSP_STANDBY,"
			" session %d\n", session_id);

		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		/* Send START IOCTL CMD - This command has no payload */
		dal_rc = cad_rpc_control(session_id,
			ardsession[session_id]->group_id,
			(void *)&q6_cmd,
			sizeof(q6_cmd), evt_buf);

		if (dal_rc != CAD_RES_SUCCESS) {
			rc = CAD_RES_FAILURE;
			pr_err("ARD CADI_IOCTL_CMD_DSP_STANDBY failed %d\n",
				session_id);
			goto done;
		}

		if ((evt_buf->no_payload.source.minor == session_id) &&
			(evt_buf->no_payload.status != ADSP_AUDIO_SUCCESS)) {
			rc = CAD_RES_FAILURE;
			pr_err("ARD DSP_STANDBY failed, ses %d, status %d\n",
				session_id, evt_buf->no_payload.status);
			goto done;
		}
	}
done:
	kfree(evt_buf);

	return rc;
}

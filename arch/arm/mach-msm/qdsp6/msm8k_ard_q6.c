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

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

s32 qdsp6_open(s32 session_id)
{
	s32                 rc, dal_rc;
	struct cadi_open_struct_type		*ref_cadr = NULL;
	struct cad_stream_device_struct_type	*ref_cadr_device = NULL;
	struct cad_stream_info_struct_type	*ref_cadr_stream = NULL;

	struct cadi_open_struct_type		*cadr = NULL;
	struct cadi_evt_struct_type		*evt_buf = NULL;

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
	evt_buf = kmalloc(sizeof(struct cadi_evt_struct_type),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	cadr = kmalloc(sizeof(struct cadi_open_struct_type),
		GFP_KERNEL);

	if (cadr == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD kmalloc failed\n");
		goto done;
	}

	memset(cadr, 0, sizeof(struct cadi_open_struct_type));
	memcpy(cadr, ref_cadr, sizeof(struct cadi_open_struct_type));

	/* reference the stream dev pointer */
	if (cadr->cad_device.device != NULL)
		cadr->cad_device.device = (u32 *)(g_audio_base + session_id *
			sizeof(struct cad_stream_device_struct_type) + 4096 +
			(Q6_ENC_BUF_PER_SESSION *
			Q6_ENC_BUF_MAX_SIZE + 4096) +
			(Q6_DEC_BUFFER_NUM_PER_STREAM *
			Q6_DEC_BUFFER_SIZE_MAX + 4096));

	if (ardsession[session_id]->qdsp6_opened == ARD_FALSE) {
		/* Only ARD will open a session with Q6 */
		D("ARD Sending RPC Open, session %d\n", session_id);
		dal_rc = cad_rpc_open(session_id, 1, cadr, evt_buf);

		if (dal_rc != CAD_RES_SUCCESS) {
			rc = CAD_RES_FAILURE;
			pr_err("ARD RPC Open failed %d\n", session_id);
			goto done;
		}

		ardsession[session_id]->qdsp6_opened = ARD_TRUE;

		if ((evt_buf->cad_event_header.cad_handle == session_id) &&
			(evt_buf->cad_event_header.status != CAD_RES_SUCCESS)) {

			rc = CAD_RES_FAILURE;
			pr_err("ARD Open RPC failed for ses: %d, status %d\n",
				session_id, evt_buf->cad_event_header.status);
			goto done;
		}
	}
	kfree(evt_buf);
	kfree(cadr);

done:
	  return rc;
}


s32 qdsp6_start(s32 session_id)
{
	s32					rc, dal_rc;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	struct cadi_evt_struct_type		*evt_buf = NULL;

	rc = dal_rc = CAD_RES_SUCCESS;


	cadr = ardsession[session_id]->sess_open_info;
	cadr_stream = &(cadr->cad_stream);

	if (cadr_stream->app_type == CAD_STREAM_APP_VOICE) {
		D("ARD Q6 START VOICE Session, so do nothing %d\n",
			session_id);
		goto done;
	}

	/* Allocate memory for the event payload */
	evt_buf = kmalloc(sizeof(struct cadi_evt_struct_type),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	D("ARD Sending RPC CADI_IOCTL_CMD_DSP_START, session %d\n",
		session_id);

	/* Send START IOCTL CMD - This command has no payload */
	dal_rc = cad_rpc_ioctl(session_id, 1, CADI_IOCTL_CMD_DSP_START,
		NULL, 0, evt_buf);

	if (dal_rc != CAD_RES_SUCCESS) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD CADI_IOCTL_CMD_DSP_START failed %d\n",
			session_id);
		goto done;
	}

	if ((evt_buf->cad_event_header.cad_handle == session_id) &&
		(evt_buf->cad_event_header.status != CAD_RES_SUCCESS)) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD CADI_IOCTL_CMD_DSP_START failed ses: %d, "
			"status %d\n", session_id,
			evt_buf->cad_event_header.status);
		goto done;
	}
	kfree(evt_buf);

done:
	return rc;
}


s32 qdsp6_close(s32 session_id)
{
	s32					rc, dal_rc;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_info_struct_type      *cadr_stream = NULL;
	struct cadi_evt_struct_type		*evt_buf = NULL;

	rc = dal_rc = CAD_RES_SUCCESS;


	cadr = ardsession[session_id]->sess_open_info;
	cadr_stream = &(cadr->cad_stream);

	if (cadr_stream->app_type == CAD_STREAM_APP_VOICE) {
		D("ARD Q6 CLOSE VOICE Session, so do nothing %d\n",
			session_id);
		goto done;
	}

	/* Allocate memory for the event payload */
	evt_buf = kmalloc(sizeof(struct cadi_evt_struct_type),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	if (ardsession[session_id]->qdsp6_opened == ARD_TRUE) {
		pr_err("ARD Sending RPC Close, session %d\n", session_id);
		dal_rc = cad_rpc_close(session_id, 1, evt_buf);

		if (dal_rc != CAD_RES_SUCCESS) {
			rc = CAD_RES_FAILURE;
			pr_err("ARD RPC Close failed %d\n", session_id);
			goto done;
		}

		ardsession[session_id]->qdsp6_opened = ARD_FALSE;
		ardsession[session_id]->qdsp6_started = ARD_FALSE;

		if ((evt_buf->cad_event_header.cad_handle == session_id) &&
			(evt_buf->cad_event_header.status != CAD_RES_SUCCESS)) {

			rc = CAD_RES_FAILURE;
			pr_err("ARD Close RPC failed for session %d,"
				" status %d\n", session_id,
				evt_buf->cad_event_header.status);
			goto done;
		}
	}
	kfree(evt_buf);
done:
	  return rc;
}



s32 qdsp6_devchg_notify(s32 session_id, u32 dev_id)
{
	s32					rc, dal_rc;
	struct cadi_dev_chg_struct_type		dev_chg;
	struct ard_state_struct_type		*local_ard_state = NULL;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	struct cadi_evt_struct_type		*evt_buf = NULL;

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
	evt_buf = kmalloc(sizeof(struct cadi_evt_struct_type),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	/*0 - Default - Default Device*/
	dev_chg.cad_dev_class = 0;
	dev_chg.cad_dev_type = local_ard_state->ard_device[dev_id].device_type;

	switch (dev_chg.cad_dev_type) {
	case CAD_RX_DEVICE:
		dev_chg.cad_new_device = local_ard_state->new_rx_device;
		break;
	case CAD_TX_DEVICE:
		dev_chg.cad_new_device = local_ard_state->new_tx_device;
		break;
	default:
		pr_err("ARD DAL RPC IOCTL failed %d\n", session_id);
		break;
	}

	dev_chg.cad_old_device = local_ard_state->ard_device[dev_id]
					.device_inuse;

	D("ARD Sending RPC CADI_IOCTL_CMD_DSP_PREP_DEV_CHG, session %d\n",
		session_id);

	dal_rc = cad_rpc_ioctl(session_id, 1, CADI_IOCTL_CMD_DSP_PREP_DEV_CHG,
		(void *)&dev_chg, sizeof(struct cadi_dev_chg_struct_type),
		evt_buf);

	if (dal_rc != CAD_RES_SUCCESS) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD CADI_IOCTL_CMD_DSP_PREP_DEV_CHG failed %d\n",
			session_id);
		goto done;
	}

	if ((evt_buf->cad_event_header.cad_handle == session_id) &&
		(evt_buf->cad_event_header.status != CAD_RES_SUCCESS)) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD DSP_PREP_DEV_CHG failed, ses %d, status %d\n",
			session_id, evt_buf->cad_event_header.status);
		goto done;
	}
	kfree(evt_buf);
done:
	return rc;
}


s32 qdsp6_standby(s32 session_id)
{
	/*Send IOCTL to Q6*/
	s32					rc, dal_rc;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	struct cadi_evt_struct_type		*evt_buf = NULL;

	rc = dal_rc = CAD_RES_SUCCESS;

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
	evt_buf = kmalloc(sizeof(struct cadi_evt_struct_type),
		GFP_KERNEL);

	if (evt_buf == NULL) {
		rc = CAD_RES_FAILURE;
		pr_err("ARD Malloc failed\n");
		goto done;
	}

	if (ardsession[session_id]->session_type == DEVICE_CTRL_TYPE) {
		D("ARD Sending RPC CADI_IOCTL_CMD_DSP_STANDBY,"
			" session %d\n", session_id);

		/* Send START IOCTL CMD - This command has no payload */
		dal_rc = cad_rpc_ioctl(session_id, 1,
			CADI_IOCTL_CMD_DSP_STANDBY, NULL, 0, evt_buf);

		if (dal_rc != CAD_RES_SUCCESS) {
			rc = CAD_RES_FAILURE;
			pr_err("ARD CADI_IOCTL_CMD_DSP_STANDBY failed %d\n",
				session_id);
			goto done;
		}

		if ((evt_buf->cad_event_header.cad_handle == session_id) &&
			(evt_buf->cad_event_header.status != CAD_RES_SUCCESS)) {
			rc = CAD_RES_FAILURE;
			pr_err("ARD DSP_STANDBY failed, ses %d, status %d\n",
				session_id, evt_buf->cad_event_header.status);
			goto done;
		}
	}
	kfree(evt_buf);

done:
	return rc;
}

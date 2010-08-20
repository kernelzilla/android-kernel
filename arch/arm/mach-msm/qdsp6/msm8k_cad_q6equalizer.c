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

#include <linux/module.h>
#include <linux/uaccess.h>

#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_ard_helper.h>
#include <mach/qdsp6/msm8k_ardi.h>
#include <mach/qdsp6/msm8k_cad_q6eq_drvi.h>
#include <mach/qdsp6/msm8k_adsp_audio_error.h>


#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_eqlzr: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

static struct cad_filter_eq_driver_struct cad_filter_eq_data;

s32 cad_filter_eq_send_stream_config(u32 sess_id)
{
	static union adsp_audio_event	result;
	s32				rc = CAD_RES_SUCCESS;

	if (ardsession[sess_id]->group_id == 0) {
		pr_err("CAD::EQ=>Can not get group id");
		return CAD_RES_FAILURE;
	}

	rc = cad_rpc_control(sess_id, ardsession[sess_id]->group_id,
		(void *)&cad_filter_eq_data.eq_stream_data[sess_id],
		sizeof(cad_filter_eq_data.eq_stream_data[sess_id]),
		&result);

	if ((rc != CAD_RES_SUCCESS) &&
			(result.no_payload.status != ADSP_AUDIO_SUCCESS)) {
		pr_err("%s: failed to set eq config\n", __func__);
		return CAD_RES_FAILURE;
	}

	return CAD_RES_SUCCESS;
}

s32 cad_filter_eq_process_stream_config(s32 sess_id,
		struct cad_audio_eq_cfg *fesc)
{
	s32 rc = CAD_RES_SUCCESS;
	u32 i;

	/* cache the data */
	memset(&(cad_filter_eq_data.eq_stream_data[sess_id]), 0,
		sizeof(cad_filter_eq_data.eq_stream_data[sess_id]));

	cad_filter_eq_data.eq_stream_data[sess_id].enable =
		fesc->enable;
	cad_filter_eq_data.eq_stream_data[sess_id].num_bands =
		fesc->num_bands;

	for (i = 0; i < fesc->num_bands; i++) {
		cad_filter_eq_data.eq_stream_data[sess_id].eq_bands[i].
			band_idx = fesc->eq_bands[i].band_idx;
		cad_filter_eq_data.eq_stream_data[sess_id].eq_bands[i].
			filter_type = fesc->eq_bands[i].filter_type;
		cad_filter_eq_data.eq_stream_data[sess_id].eq_bands[i].
			center_freq_hz = fesc->eq_bands[i].center_freq_hz;
		cad_filter_eq_data.eq_stream_data[sess_id].eq_bands[i].
			filter_gain = fesc->eq_bands[i].filter_gain;
		cad_filter_eq_data.eq_stream_data[sess_id].eq_bands[i].
			q_factor = fesc->eq_bands[i].q_factor;
	}

	/* check the stream status */
	if (!ardsession[sess_id]->active) {
		D("%s: stream is not active.\n", __func__);
		return CAD_RES_SUCCESS;
	}

	/* send it if stream is active */
	rc = cad_filter_eq_send_stream_config(sess_id);
	return CAD_RES_SUCCESS;
}

s32 cad_filter_eq_process_stream_start(s32 sess_id)
{
	struct cadi_open_struct_type *open_struct =
		ardsession[sess_id]->sess_open_info;

	/* store the device control session */
	if (open_struct->cad_open.op_code == CAD_OPEN_OP_DEVICE_CTRL) {
		cad_filter_eq_data.device_session_id = sess_id;
		D("%s: stored device control session\n", __func__);
		return CAD_RES_SUCCESS;
	}
	/* skip if this is TX session */
	if (open_struct->cad_open.op_code != CAD_OPEN_OP_WRITE) {
		D("%s: skip TX stream session\n", __func__);
		return CAD_RES_SUCCESS;
	}
	/* skip if no stream eq data */
	if (cad_filter_eq_data.eq_stream_data[sess_id].enable ==
			CAD_EQ_INVALID_DATA) {
		D("%s: no valid stream eq data for session (%d)\n",
				__func__, sess_id);
		return CAD_RES_SUCCESS;
	}
	/* send stream based eq*/
	return cad_filter_eq_send_stream_config(sess_id);
}

static s32 cad_filter_eq_close(s32 sess_id)
{
	/* reset the device control session */
	if ((u32)sess_id == cad_filter_eq_data.device_session_id) {
		cad_filter_eq_data.device_session_id = 0;
		return CAD_RES_SUCCESS;
	}

	/* this is stream session */
	/* reset the stream eq table since we don't cache the stream eq data */
	cad_filter_eq_data.eq_stream_data[sess_id].enable =
		CAD_EQ_INVALID_DATA;
	return CAD_RES_SUCCESS;
}

static s32 cad_filter_eq_ioctl(s32 sess_id, u32 cmd, void *cmd_buf,
		u32 cmd_buf_len)
{
	s32 rc = CAD_RES_SUCCESS;
	struct cad_filter_struct *filt =
			(struct cad_filter_struct *)cmd_buf;

	D("%s: %d\n", __func__, cmd);

	switch (cmd) {
	case CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG:
	case CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG:
	{
		if (cmd_buf_len != sizeof(*filt))
			break;

		/* check if this is IOCTL for EQ filter */
		if (filt->filter_type != CAD_DEVICE_FILTER_TYPE_EQ)
			break;

		switch (filt->cmd) {
		case CAD_FILTER_EQ_DEVICE_CONFIG:
			if (filt->format_block_len !=
					sizeof(struct cad_audio_eq_cfg)) {
				D("%s: wrong device config format block\n",
					__func__);
				break;
			}
			/* NO OP */
			break;
		case CAD_FILTER_EQ_STREAM_CONFIG:
			if (filt->format_block_len !=
					sizeof(struct cad_audio_eq_cfg)) {

				D("%s: wrong stream config format block.\n",
					__func__);
				break;
			}
			rc = cad_filter_eq_process_stream_config(
				sess_id,
				(struct cad_audio_eq_cfg *)
					filt->format_block);
			break;
		}
		break;
	}
	case CAD_IOCTL_CMD_STREAM_START:
		rc = cad_filter_eq_process_stream_start(sess_id);
		break;
	}
	return rc;
}

s32 cad_filter_eq_init(struct cad_func_tbl_type **func_tbl)
{
	u32 i;
	static struct cad_func_tbl_type vtable = {
		NULL,
		cad_filter_eq_close,
		NULL,
		NULL,
		cad_filter_eq_ioctl
	};

	*func_tbl = &vtable;

	/* init the stream/device eq tables */
	/* set stream data to invalid */
	for (i = 0; i < CAD_MAX_SESSION; i++) {
		cad_filter_eq_data.eq_stream_data[i].enable =
			CAD_EQ_INVALID_DATA;

		cad_filter_eq_data.eq_stream_data[i].cmd.op_code =
			ADSP_AUDIO_IOCTL_SET_SESSION_EQ_CONFIG;
		cad_filter_eq_data.eq_stream_data[i].cmd.response_type =
			ADSP_AUDIO_RESPONSE_COMMAND;
	}

	return CAD_RES_SUCCESS;
}

s32 cad_filter_eq_dinit(void)
{
	return CAD_RES_SUCCESS;
}


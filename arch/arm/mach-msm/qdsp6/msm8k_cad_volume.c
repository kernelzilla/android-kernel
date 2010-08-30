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
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_cad_itypes.h>
#include <mach/qdsp6/msm8k_cad_volume.h>

struct cad_device_volume_cache
		qdsp6_volume_cache_tbl[QDSP6VOLUME_MAX_DEVICE_COUNT];


#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif


/* This computes linear mapping device volume. */
s32 qdsp6_volume_mapping(u32 device_id, s32 percentage)
{
	s32 max_gain = qdsp6_volume_cache_tbl[device_id].max_gain;
	s32 min_gain = qdsp6_volume_cache_tbl[device_id].min_gain;
	return min_gain + (((max_gain - min_gain) * percentage) / 100);
}

s32 qdsp6_volume_open(s32 session_id,
	struct cad_open_struct_type *open_param)
{
	return CAD_RES_SUCCESS;
}

s32 qdsp6_volume_close(s32 session_id)
{
	return CAD_RES_SUCCESS;
}

s32 qdsp6_volume_write(s32 session_id,
	struct cad_buf_struct_type *buf_ptr)
{
	return CAD_RES_SUCCESS;
}

s32 qdsp6_volume_read(s32 session_id,
	struct cad_buf_struct_type *buf_ptr)
{
	return CAD_RES_SUCCESS;
}

s32 qdsp6_volume_ioctl(s32 session_id, u32 cmd_code,
	void *cmd_buf, u32 cmd_len)
{
	struct cad_stream_filter_struct_type *strm_flt = NULL;
	struct cad_flt_cfg_dev_vol *dev_vol_buf = NULL;
	struct cad_flt_cfg_strm_vol *stream_vol_buf = NULL;
	struct cad_flt_cfg_dev_mute *dev_mute_buf = NULL;
	struct cad_flt_cfg_strm_mute *stream_mute_buf = NULL;

	struct qdsp_set_device_volume *q6_set_dev_vol = NULL;
	struct qdsp_set_stream_volume *q6_set_strm_vol = NULL;
	struct qdsp_set_device_mute *q6_set_dev_mute = NULL;
	struct qdsp_set_stream_mute *q6_set_strm_mute = NULL;

	int rc = CAD_RES_SUCCESS;
	s32 device_volume = 0;
	s32 rpc_cmd_code = 0;
	u8 *rpc_cmd_buf = NULL;
	u32 rpc_cmd_buf_len = 0;
	struct cadi_evt_struct_type event_payload;

	memset(&event_payload, 0, sizeof(struct cadi_evt_struct_type));

	/* Not handle request other than the following two. */
	if (cmd_code != CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG &&
		cmd_code != CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG) {
		/* Just silently succeed unrecognized IOCTLs. */
		return CAD_RES_SUCCESS;
	}

	/* Defensive programming. */
	if (session_id < 1 || session_id >= CAD_MAX_SESSION || cmd_buf == NULL
		|| cmd_len != sizeof(struct cad_stream_filter_struct_type))
		return CAD_RES_FAILURE;

	/* Find the appropriate command type. */
	strm_flt = (struct cad_stream_filter_struct_type *)cmd_buf;
	if (cmd_code == CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG
		&& strm_flt->filter_type == CAD_FILTER_CONFIG_DEVICE_VOLUME
		&& strm_flt->format_block_len ==
			sizeof(struct cad_flt_cfg_dev_vol))
		dev_vol_buf =
			(struct cad_flt_cfg_dev_vol *)
					strm_flt->format_block;
	else if (CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG == cmd_code
		&& CAD_FILTER_CONFIG_DEVICE_MUTE == strm_flt->filter_type
		&& strm_flt->format_block_len ==
			sizeof(struct cad_flt_cfg_dev_mute))
		dev_mute_buf = (struct cad_flt_cfg_dev_mute *)
						strm_flt->format_block;

	/* stream session */
	else if (cmd_code == CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG
		&& strm_flt->filter_type == CAD_FILTER_CONFIG_STREAM_VOLUME
		&& strm_flt->format_block_len ==
			sizeof(struct cad_flt_cfg_strm_vol))
		stream_vol_buf = (struct cad_flt_cfg_strm_vol *)(
					strm_flt->format_block);

	else if (cmd_code == CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG
		&& CAD_FILTER_CONFIG_STREAM_MUTE == strm_flt->filter_type
		&& strm_flt->format_block_len ==
			sizeof(struct cad_flt_cfg_strm_mute))
		stream_mute_buf = (struct cad_flt_cfg_strm_mute *)
						(strm_flt->format_block);

	else {
		pr_err("CAD:VOL: Error: wrong type id.\n");
		return CAD_RES_FAILURE;
	}

	/* Handle volume control command. */
	switch (strm_flt->filter_type) {
	case CAD_FILTER_CONFIG_DEVICE_VOLUME:
		D("CAD:VOL: Device Volume\n");

		/* For volume != 0%: send unmute command. */
		if (dev_vol_buf->volume != 0) {
			/* Construct QDSP6 device mute command. */
			/* 1. Allocate memory for command buffer. */
			q6_set_dev_mute = kmalloc(
				sizeof(struct qdsp_set_device_mute),
				GFP_KERNEL);
			if (!q6_set_dev_mute)
				return CAD_RES_FAILURE;

			memset(q6_set_dev_mute, 0,
				sizeof(struct qdsp_set_device_mute));
			/* 2. Assign values to command buffer. */
			q6_set_dev_mute->device_id = dev_vol_buf->device_id;
			q6_set_dev_mute->path = dev_vol_buf->path;
			q6_set_dev_mute->mute = 0;
			rpc_cmd_buf = (u8 *)q6_set_dev_mute;
			rpc_cmd_buf_len = sizeof(struct qdsp_set_device_mute);
			rpc_cmd_code = QDSP_IOCTL_CMD_SET_DEVICE_MUTE;
			/* 3. Send command to Q6. */
			rc = cad_rpc_ioctl(
				session_id,
				1,
				rpc_cmd_code,
				rpc_cmd_buf,
				rpc_cmd_buf_len,
				&event_payload);
			if (rc != CAD_RES_SUCCESS) {
				pr_err("%s: cad_rpc_ioctl() failure\n",
					__func__);
				return rc;
			}

		}

		/* Map the device volume to QDSP6. */
		device_volume = qdsp6_volume_mapping(
			dev_vol_buf->device_id,
			dev_vol_buf->volume);

		/* Cache the device volume. */
		qdsp6_volume_cache_tbl[dev_vol_buf->device_id]
			.current_volume = device_volume;
		qdsp6_volume_cache_tbl[dev_vol_buf->device_id]
			.valid_current_volume = 1;

		/* Construct QDSP6 device volume command:	*/
		/* 1. Allocate memory for command buffer.	*/
		q6_set_dev_vol = kmalloc(
			sizeof(struct cad_flt_cfg_dev_vol),
			GFP_KERNEL);
		if (!q6_set_dev_vol)
			return CAD_RES_FAILURE;

		memset(q6_set_dev_vol, 0,
			sizeof(struct qdsp_set_device_volume));

		/* 2. Assign values to command buffer. */
		q6_set_dev_vol->device_id = dev_vol_buf->device_id;
		q6_set_dev_vol->path = dev_vol_buf->path;
		q6_set_dev_vol->volume = device_volume;
		rpc_cmd_buf = (u8 *)q6_set_dev_vol;
		rpc_cmd_buf_len =
			sizeof(struct qdsp_set_device_volume);
		rpc_cmd_code = QDSP_IOCTL_CMD_SET_DEVICE_VOL;

		/* HACK: for volume = 0%: send mute command instead. */
		if (dev_vol_buf->volume == 0) {
			/* Construct QDSP6 device mute command. */
			/* 1. Allocate memory for command buffer. */
			q6_set_dev_mute = kmalloc(
				sizeof(struct qdsp_set_device_mute),
				GFP_KERNEL);
			if (!q6_set_dev_mute)
				return CAD_RES_FAILURE;

			memset(q6_set_dev_mute, 0,
				sizeof(struct qdsp_set_device_mute));
			/* 2. Assign values to command buffer. */
			q6_set_dev_mute->device_id = q6_set_dev_vol->device_id;
			q6_set_dev_mute->path = q6_set_dev_vol->path;
			q6_set_dev_mute->mute = 1; /* mute */
			rpc_cmd_buf = (u8 *)q6_set_dev_mute;
			rpc_cmd_buf_len = sizeof(struct qdsp_set_device_mute);
			rpc_cmd_code = QDSP_IOCTL_CMD_SET_DEVICE_MUTE;
		}

		break;
	case CAD_FILTER_CONFIG_DEVICE_MUTE:
		D("CAD:VOL: Device Mute\n");

		qdsp6_volume_cache_tbl[dev_mute_buf->device_id].mute =
			dev_mute_buf->mute;

		/* Construct QDSP6 device mute command. */
		/* 1. Allocate memory for command buffer. */
		q6_set_dev_mute = kmalloc(
			sizeof(struct qdsp_set_device_mute), GFP_KERNEL);
		if (!q6_set_dev_mute)
			return CAD_RES_FAILURE;

		memset(q6_set_dev_mute, 0,
			sizeof(struct qdsp_set_device_mute));
		/* 2. Assign values to command buffer. */
		q6_set_dev_mute->device_id = dev_mute_buf->device_id;
		q6_set_dev_mute->path = dev_mute_buf->path;
		q6_set_dev_mute->mute = dev_mute_buf->mute;
		rpc_cmd_buf = (u8 *)q6_set_dev_mute;
		rpc_cmd_buf_len = sizeof(struct qdsp_set_device_mute);
		rpc_cmd_code = QDSP_IOCTL_CMD_SET_DEVICE_MUTE;

		break;
	case CAD_FILTER_CONFIG_STREAM_VOLUME:
		D("CAD:VOL: Stream Volume\n");

		/* For volume != min: send unmute command. */
		if (stream_vol_buf->volume != CAD_STREAM_MIN_GAIN) {
			/* Construct QDSP6 stream mute command. */
			/* 1. Allocate memory for command buffer. */
			q6_set_strm_mute = kmalloc(
				sizeof(struct qdsp_set_stream_mute),
				GFP_KERNEL);
			if (!q6_set_strm_mute)
				return CAD_RES_FAILURE;

			/* 2. Assign values to command buffer.	*/
			q6_set_strm_mute->mute = 0;
			rpc_cmd_buf = (u8 *)q6_set_strm_mute;
			rpc_cmd_buf_len = sizeof(struct qdsp_set_stream_mute);
			rpc_cmd_code = QDSP_IOCTL_CMD_SET_STREAM_MUTE;
			/* 3. Send command to Q6. */
			rc = cad_rpc_ioctl(session_id,
					     1,
					     rpc_cmd_code,
					     rpc_cmd_buf,
					     rpc_cmd_buf_len,
					     &event_payload);
			if (rc != CAD_RES_SUCCESS) {
				pr_err("%s: cad_rpc_ioctl() failure\n",
					__func__);
				return rc;
			}
		}

		/* Construct QDSP6 stream volume command. */
		/* 1. Allocate memory for command buffer. */
		q6_set_strm_vol = kmalloc(
			sizeof(struct qdsp_set_stream_volume),
			GFP_KERNEL);
		if (!q6_set_strm_vol)
			return CAD_RES_FAILURE;

		/* 2. Assign values to command buffer. */
		q6_set_strm_vol->volume = stream_vol_buf->volume;
		rpc_cmd_buf = (u8 *)q6_set_strm_vol;
		rpc_cmd_buf_len = sizeof(struct qdsp_set_stream_volume);
		rpc_cmd_code = QDSP_IOCTL_CMD_SET_STREAM_VOL;

		/* For volume = min: send mute command instead. */
		if (stream_vol_buf->volume == CAD_STREAM_MIN_GAIN) {
			/* Construct QDSP6 stream mute command. */
			/* 1. Allocate memory for command buffer. */
			q6_set_strm_mute = kmalloc(
					sizeof(struct qdsp_set_stream_mute),
					GFP_KERNEL);

			if (!q6_set_strm_mute)
				return CAD_RES_FAILURE;

			/* 2. Assign values to command buffer. */
			q6_set_strm_mute->mute = 1;
			rpc_cmd_buf = (u8 *)q6_set_strm_mute;
			rpc_cmd_buf_len = sizeof(struct qdsp_set_stream_mute);
			rpc_cmd_code = QDSP_IOCTL_CMD_SET_STREAM_MUTE;
		}

		break;
	case CAD_FILTER_CONFIG_STREAM_MUTE:
		D("CAD:VOL: Stream Mute\n");

		/* Construct QDSP6 stream mute command. */
		/* 1. Allocate memory for command buffer. */
		q6_set_strm_mute = kmalloc(
					sizeof(struct qdsp_set_stream_mute),
					GFP_KERNEL);
		if (!q6_set_strm_mute)
			return CAD_RES_FAILURE;

		/* 2. Assign values to command buffer. */
		q6_set_strm_mute->mute = stream_mute_buf->mute;
		rpc_cmd_buf = (u8 *)q6_set_strm_mute;
		rpc_cmd_buf_len = sizeof(struct qdsp_set_stream_mute);
		rpc_cmd_code = QDSP_IOCTL_CMD_SET_STREAM_MUTE;

		break;
	default:
		/* Just return without error. */
		return CAD_RES_SUCCESS;
	}

	/* Always send device/stream volume command to Q6 for now. */
	rc = cad_rpc_ioctl(
			session_id,
			1,
			rpc_cmd_code,
			rpc_cmd_buf,
			rpc_cmd_buf_len,
			&event_payload);
	if (rc != CAD_RES_SUCCESS)
		pr_err("%s: cad_rpc_ioctl() failure\n", __func__);

	D("%s: ioctl() processed.\n", __func__);

	kfree(q6_set_dev_vol);
	kfree(q6_set_strm_vol);
	kfree(q6_set_dev_mute);
	kfree(q6_set_strm_mute);

	return rc;
}


int cad_volume_dinit(void)
{
	memset(qdsp6_volume_cache_tbl, 0,
		sizeof(struct cad_device_volume_cache) *
			QDSP6VOLUME_MAX_DEVICE_COUNT);
	return CAD_RES_SUCCESS;
}


int cad_volume_init(struct cad_func_tbl_type **func_tbl)
{

	static struct cad_func_tbl_type vtable = {
		qdsp6_volume_open,
		qdsp6_volume_close,
		qdsp6_volume_write,
		qdsp6_volume_read,
		qdsp6_volume_ioctl
	};

	*func_tbl = &vtable;

	/* Set up the volume cache table by default values. */
	memset(qdsp6_volume_cache_tbl, 0,
		sizeof(struct cad_device_volume_cache)
			* QDSP6VOLUME_MAX_DEVICE_COUNT);

	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HANDSET_SPKR].max_gain
		= CAD_DEVICE_HANDSET_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HANDSET_SPKR].min_gain
		= CAD_DEVICE_HANDSET_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO].max_gain
		= CAD_DEVICE_HEADSET_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO].min_gain
		= CAD_DEVICE_HEADSET_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO].max_gain
		= CAD_DEVICE_HEADSET_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO].min_gain
		= CAD_DEVICE_HEADSET_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_MONO].max_gain
		= CAD_DEVICE_SPEAKER_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_MONO].min_gain
		= CAD_DEVICE_SPEAKER_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO].max_gain
		= CAD_DEVICE_SPEAKER_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO].min_gain
		= CAD_DEVICE_SPEAKER_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_SCO_SPKR].max_gain
		= CAD_DEVICE_BT_SCO_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_SCO_SPKR].min_gain
		= CAD_DEVICE_BT_SCO_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_A2DP_SPKR].max_gain
		= CAD_DEVICE_BT_A2DP_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_BT_A2DP_SPKR].min_gain
		= CAD_DEVICE_BT_A2DP_MIN_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR].max_gain
		= CAD_DEVICE_TTY_MAX_GAIN;
	qdsp6_volume_cache_tbl[CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR].min_gain
		= CAD_DEVICE_TTY_MIN_GAIN;

	return CAD_RES_SUCCESS;
}

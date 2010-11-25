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
#include <mach/qdsp6/msm8k_q6_api_flip_utils.h>
#include <mach/qdsp6/msm8k_cad_q6dec_drvi.h>
#include <mach/qdsp6/msm8k_adsp_audio_device.h>
#include <mach/qdsp6/msm8k_adsp_audio_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_media_format.h>

#define AAC_FORMAT_SIZE			8

/* Supported AAC object types */
/* #define AAC_OBJECT_LC		2 */
/* #define AAC_OBJECT_SSR		3 */
/* #define AAC_OBJECT_LTP		4 */
/* #define AAC_OBJECT_HE		5 */
/* #define AAC_OBJECT_SCALABLE		6 */
#define AAC_OBJECT_ER_LC		17
#define AAC_OBJECT_ER_LTP		19
#define AAC_OBJECT_ER_SCALABLE		20
#define AAC_OBJECT_BSAC			22
#define AAC_OBJECT_ER_LD		23
/* #define AAC_OBJECT_HE_PS		29 */


#if 0
#define D(fmt, args...) printk(KERN_INFO "ARD: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif


/* Utility function */
u32 q6_stream_context_mapping(enum cad_stream_app_enum_type app_type,
			      u32 *mode)
{
	*mode = ADSP_AUDIO_OPEN_STREAM_MODE_NONE;

	switch (app_type) {
	case CAD_STREAM_APP_VOICE:
		/* voice */
		return ADSP_AUDIO_DEVICE_CONTEXT_VOICE;
	case CAD_STREAM_APP_RECORD:
		/* record */
		return ADSP_AUDIO_DEVICE_CONTEXT_RECORD;
	case CAD_STREAM_APP_A2DP_MIX_MODE:
		/* loopback */
		return ADSP_AUDIO_DEVICE_CONTEXT_PCM_LOOPBACK;
	case CAD_STREAM_APP_PLAYBACK:
	case CAD_STREAM_APP_DTMF:
	case CAD_STREAM_APP_RINGER:
	case CAD_STREAM_APP_SYSTEM_SOUND:
		/* playback */
		return ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK;
	case CAD_STREAM_APP_AUDIO_VIDEO:
		*mode = ADSP_AUDIO_OPEN_STREAM_MODE_AVSYNC;
		return ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK;
	case CAD_STREAM_APP_MIXED_RECORD:
		return ADSP_AUDIO_DEVICE_CONTEXT_MIXED_RECORD;
	case CAD_STREAM_APP_UNKNOWN:
	default:
		/* error */
		return 0xFFFFFFFF;
	}
}

u32 q6_open_op_mapping(u32 op_code)
{
	switch (op_code) {
	case CAD_OPEN_OP_READ:
		/* voice */
		return ADSP_AUDIO_IOCTL_CMD_OPEN_READ;
	case CAD_OPEN_OP_WRITE:
		/* record */
		return ADSP_AUDIO_IOCTL_CMD_OPEN_WRITE;
	case CAD_OPEN_OP_DEVICE_CTRL:
		/* loopback */
		return ADSP_AUDIO_IOCTL_CMD_OPEN_DEVICE;
	default:
		/* error */
		return 0xFFFFFFFF;
	}
}

u32 q6_device_id_mapping(u32 device)
{
	switch (device) {
	case CAD_HW_DEVICE_ID_HANDSET_MIC:
		return ADSP_AUDIO_DEVICE_ID_HANDSET_MIC;
	case CAD_HW_DEVICE_ID_HANDSET_SPKR:
		return ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR;
	case CAD_HW_DEVICE_ID_HEADSET_MIC:
		return ADSP_AUDIO_DEVICE_ID_HEADSET_MIC;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO:
		return ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_MONO;
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO:
		return ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MIC:
		return ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MONO:
		return ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO;
	case CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO:
		return ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO;
	case CAD_HW_DEVICE_ID_BT_SCO_MIC:
		return ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC;
	case CAD_HW_DEVICE_ID_BT_SCO_SPKR:
		return ADSP_AUDIO_DEVICE_ID_BT_SCO_SPKR;
	case CAD_HW_DEVICE_ID_TTY_HEADSET_MIC:
		return ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC;
	case CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR:
		return ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_SPKR;
	case CAD_HW_DEVICE_ID_I2S_RX:
		return ADSP_AUDIO_DEVICE_ID_I2S_SPKR;
	case CAD_HW_DEVICE_ID_I2S_TX:
		return ADSP_AUDIO_DEVICE_ID_I2S_MIC;

	case CAD_HW_DEVICE_ID_BT_A2DP_SPKR:
		return ADSP_AUDIO_DEVICE_ID_BT_A2DP_SPKR;
	case CAD_HW_DEVICE_ID_BT_A2DP_TX:
		return ADSP_AUDIO_DEVICE_ID_MIXED_PCM_LOOPBACK_TX;

	case CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX:
		return ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_MONO_HEADSET;
	case CAD_HW_DEVICE_ID_HEADSET_MONO_PLUS_SPKR_STEREO_RX:
		return ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_MONO_HEADSET;

	case CAD_HW_DEVICE_ID_NULL_RX:
		return ADSP_AUDIO_DEVICE_ID_NULL_SINK;

	case CAD_HW_DEVICE_ID_VOICE:
		return ADSP_AUDIO_DEVICE_ID_VOICE;

	case CAD_HW_DEVICE_ID_DEFAULT_RX:
	case CAD_HW_DEVICE_ID_DEFAULT_TX:
		return ADSP_AUDIO_DEVICE_ID_DEFAULT;

	default:
		return 0xFFFFFFFF;
	}
}

u8 q6_device_direction_mapping(u8 device)
{
	switch (device) {
	case CAD_TX_DEVICE:
		return ADSP_AUDIO_TX_DEVICE;
	case CAD_RX_DEVICE:
		return ADSP_AUDIO_RX_DEVICE;
	default:
		return 0xFF;
	}
}

s32 convert_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32 result = CAD_RES_SUCCESS;

	if ((cad_open_struct->cad_config.format_block_len != 0) &&
		(cad_open_struct->cad_config.format_block == NULL))
		return CAD_RES_FAILURE;

	/* Translate format type and format block. */
	switch (cad_open_struct->cad_open.format) {
	case CAD_FORMAT_PCM:
		result = convert_pcm_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_DTMF:
		result = convert_dtmf_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_YADPCM:
		result = convert_yadpcm_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_ADPCM:
		result = convert_adpcm_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_AAC:
		result = convert_aac_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_AMRWB:
	case CAD_FORMAT_AMRNB:
		result = convert_amr_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_EVRCB:
	case CAD_FORMAT_EVRC:
		result = convert_evrc_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_QCELP13K:
		result = convert_v13k_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_MIDI:
		result = convert_midi_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
		return CAD_RES_SUCCESS;

	case CAD_FORMAT_MP3:
		result = convert_mp3_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_SBC:
		result = convert_sbc_format_block(session_id, q6_open_struct,
			cad_open_struct);
		break;
	case CAD_FORMAT_WMA:
	case CAD_FORMAT_WMA_PRO:
		result = convert_wma_std_format_block(session_id,
			q6_open_struct, cad_open_struct);
		break;
	default:
		result = CAD_RES_FAILURE;
		return result;
	}

	return result;
}

s32 convert_pcm_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_write_pcm_struct_type	*cad_format = NULL;
	struct adsp_audio_standard_format	*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(struct cad_write_pcm_format_struct_type))) {

		pr_err("No format block provided for pcm\n");
		return CAD_RES_FAILURE;
	}


	/* Save and translate the cmdbuff passed in */
	cad_format = &(((struct cad_write_pcm_format_struct_type *)
		(cad_open_struct->cad_config.format_block))->pcm);
	q6_format = (struct adsp_audio_standard_format *)
		&(q6_open_struct->open_data.format_block);

	q6_format->format = ADSP_AUDIO_FORMAT_PCM;
	if (cad_open_struct->cad_open.op_code == CAD_OPEN_OP_WRITE) {
		/* PCM Playback assumed interleaved from Host */
		q6_format->is_interleaved = true;
	} else {
		/* PCM coming out of Device Ses is always non-interleaved */
		q6_format->is_interleaved = false;
	}
	switch (cad_format->us_sample_rate) {
	case 0:
		q6_format->sampling_rate = 9600;
		break;
	case 1:
		q6_format->sampling_rate = 88200;
		break;
	case 2:
		q6_format->sampling_rate = 64000;
		break;
	case 3:
		q6_format->sampling_rate = 48000;
		break;
	case 4:
		q6_format->sampling_rate = 44100;
		break;
	case 5:
		q6_format->sampling_rate = 32000;
		break;
	case 6:
		q6_format->sampling_rate = 24000;
		break;
	case 7:
		q6_format->sampling_rate = 22050;
		break;
	case 8:
		q6_format->sampling_rate = 16000;
		break;
	case 9:
		q6_format->sampling_rate = 12000;
		break;
	case 10:
		q6_format->sampling_rate = 11025;
		break;
	case 11:
		q6_format->sampling_rate = 8000;
		break;
	default:
		return CAD_RES_FAILURE;
	}

	q6_format->channels = cad_format->us_channel_config;

	switch (cad_format->us_width) {
	case 0:
		q6_format->bits_per_sample = 8;
		break;
	case 1:
		q6_format->bits_per_sample = 16;
		break;
	case 2:
		q6_format->bits_per_sample = 24;
		break;
	default:
		return CAD_RES_FAILURE;
	}

	if (cad_format->us_sign == 0)
		q6_format->is_signed = true;


	return result;
}


s32 convert_dtmf_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct adsp_audio_standard_format	*q6_format = NULL;

	/* Save and translate the cmdbuff passed in */
	q6_format = (struct adsp_audio_standard_format *)
		&(q6_open_struct->open_data.format_block);

	q6_format->format = ADSP_AUDIO_FORMAT_DTMF;
	q6_format->sampling_rate = 48000;
	q6_format->channels = 2;
	q6_format->bits_per_sample = 16;
	q6_format->is_signed = true;
	q6_format->is_interleaved = false;

	return result;
}



s32 convert_yadpcm_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_adpcm_format_struct		*cad_format = NULL;
	struct adsp_audio_standard_format	*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(*cad_format))) {

		pr_err("No format block provided for yadpcm\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = (struct cad_adpcm_format_struct *)
		(cad_open_struct->cad_config.format_block);
	q6_format = (struct adsp_audio_standard_format *)
		&(q6_open_struct->open_data.format_block);

	q6_format->format = ADSP_AUDIO_FORMAT_YADPCM;
	q6_format->sampling_rate = cad_format->sampling_rate;
	q6_format->channels = (u16)(cad_format->channels);
	q6_format->bits_per_sample = (u16)(cad_format->bit_per_sample);

	if (cad_format->flags == CAD_ADPCM_SIGNED)
		q6_format->is_signed = true;

	if (cad_format->flags == CAD_ADPCM_INTERLEAVED)
		q6_format->is_interleaved = true;

	return result;
}


s32 convert_adpcm_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_adpcm_format_struct		*cad_format = NULL;
	struct adsp_audio_adpcm_format		*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(*cad_format))) {

		pr_err("No format block provided for adpcm\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = (struct cad_adpcm_format_struct *)
		(cad_open_struct->cad_config.format_block);
	q6_format = (struct adsp_audio_adpcm_format *)
		&(q6_open_struct->open_data.format_block);

	q6_format->format = ADSP_AUDIO_FORMAT_ADPCM;
	q6_format->block_size = cad_format->block_size;
	q6_format->sampling_rate = cad_format->sampling_rate;
	q6_format->channels = (u16)(cad_format->channels);
	q6_format->bits_per_sample = (u16)(cad_format->bit_per_sample);

	if (cad_format->flags == CAD_ADPCM_SIGNED)
		q6_format->is_signed = true;

	if (cad_format->flags == CAD_ADPCM_INTERLEAVED)
		q6_format->is_interleaved = true;

	return result;
}

s32 convert_aac_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_write_aac_struct_type	*cad_format = NULL;
	struct adsp_audio_binary_format		*q6_format = NULL;
	u32					*aac_type = NULL;
	s32					index = sizeof(u32);
	u32		op_code = cad_open_struct->cad_open.op_code;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(struct cad_write_aac_format_struct_type))) {

		pr_err("No format block provided for aac\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = &(((struct cad_write_aac_format_struct_type *)
		(cad_open_struct->cad_config.format_block))->aac);
	q6_format = (struct adsp_audio_binary_format *)
		&(q6_open_struct->open_data.format_block);


	q6_format->format = ADSP_AUDIO_FORMAT_MPEG4_AAC;
	aac_type = (u32 *)(q6_format->data);
	switch (cad_format->block_formats) {
	case 0xFFFF:
		if (op_code == CAD_OPEN_OP_READ) {
			/* AAC Encoder expect MPEG4_ADTS media type */
			*aac_type = ADSP_AUDIO_AAC_MPEG4_ADTS;
		} else {
			*aac_type = ADSP_AUDIO_AAC_ADTS;
		}
		break;
	case 0:
		if (op_code == CAD_OPEN_OP_READ) {
			/* for ADIF recording */
			D("AAC Recording set AAC_ADIF to AAC_RAW for encoder");
			*aac_type = ADSP_AUDIO_AAC_RAW;
		} else {
			*aac_type = ADSP_AUDIO_AAC_ADIF;
		}
		break;
	case 1:
		/* pseudo raw */
		*aac_type = ADSP_AUDIO_AAC_RAW;
		break;
	case 2:
		*aac_type = ADSP_AUDIO_AAC_LOAS;
		break;
	case 3:
		/* RAW format, at least one full frame per buffer */
		/* coming from QTV parser */
		*aac_type = ADSP_AUDIO_AAC_FRAMED_RAW;
		break;
	case 4:
		/* raw */
		*aac_type = ADSP_AUDIO_AAC_RAW;
		break;
	default:
		return CAD_RES_FAILURE;
	}

	q6_format->data[index++] = (u8)(
			((cad_format->audio_object_type & 0x1F) << 3) |
			((cad_format->sample_rate >> 1) & 0x7));
	q6_format->data[index] = (u8)(
			((cad_format->sample_rate & 0x1) << 7) |
			((cad_format->channel_config & 0x7) << 3));

	switch (cad_format->audio_object_type) {
	case AAC_OBJECT_ER_LC:
	case AAC_OBJECT_ER_LTP:
	case AAC_OBJECT_ER_LD:
		/* extension flag */
		q6_format->data[index++] |= 0x1;
		q6_format->data[index] = (u8)(
			((cad_format->aac_section_data_resilience_flag
			& 0x1) << 7) |
			((cad_format->aac_scalefactor_data_resilience_flag
			& 0x1) << 6) |
			((cad_format->aac_spectral_data_resilience_flag
			& 0x1) << 5) |
			((cad_format->ep_config & 0x3) << 2));
		break;

	case AAC_OBJECT_ER_SCALABLE:
		q6_format->data[index++] |= 0x1;
		/* extension flag */
		q6_format->data[index++] = (u8)(
			((cad_format->aac_section_data_resilience_flag
			& 0x1) << 4) |
			((cad_format->aac_scalefactor_data_resilience_flag
			& 0x1) << 3) |
			((cad_format->aac_spectral_data_resilience_flag
			& 0x1) << 2) |
			((cad_format->ep_config >> 1) & 0x1));
		q6_format->data[index] = (u8)((cad_format->ep_config & 0x1)
			<< 7);
		break;

	case AAC_OBJECT_BSAC:
		q6_format->data[++index] = (u8)((cad_format->ep_config & 0x3)
			<< 6);
		break;

	default:
		break;
	}

	D("CAD:ARD AAC format %x%x%x%x%x%x%x%x, len %d",
		q6_format->data[0], q6_format->data[1], q6_format->data[2],
		q6_format->data[3], q6_format->data[4], q6_format->data[5],
		q6_format->data[6], q6_format->data[7],
		index+1);

	q6_format->num_bytes = index + 1;


	/* Fill in encoder config. */
	q6_open_struct->open_data.config.aac_cfg.bit_rate =
		cad_format->bit_rate;

	if ((cad_format->sbr_on_flag == 0) &&
		(cad_format->sbr_ps_on_flag == 0)) {

		q6_open_struct->open_data.config.aac_cfg.encoder_mode =
			ADSP_AUDIO_ENC_AAC_LC_ONLY_MODE;

	} else if ((cad_format->sbr_on_flag == 1) &&
		(cad_format->sbr_ps_on_flag == 0)) {

		q6_open_struct->open_data.config.aac_cfg.encoder_mode =
			ADSP_AUDIO_ENC_AAC_PLUS_MODE;

	} else if ((cad_format->sbr_on_flag == 1) &&
		(cad_format->sbr_ps_on_flag == 1)) {

		q6_open_struct->open_data.config.aac_cfg.encoder_mode =
			ADSP_AUDIO_ENC_ENHANCED_AAC_PLUS_MODE;

	} else {
		pr_err("CAD:ARD: cad format block Sbr flags are undefined");
		return CAD_RES_FAILURE;
	}

	return result;
}

s32 convert_amr_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_amr_format			*cad_format = NULL;
	struct adsp_audio_standard_format	*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(*cad_format))) {

		pr_err("No format block provided for amr\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = (struct cad_amr_format *)
		(cad_open_struct->cad_config.format_block);
	q6_format = (struct adsp_audio_standard_format *)
		&(q6_open_struct->open_data.format_block);


	if (cad_open_struct->cad_open.format == CAD_FORMAT_AMRWB)
		q6_format->format = ADSP_AUDIO_FORMAT_AMRWB_FS;
	else
		q6_format->format = ADSP_AUDIO_FORMAT_AMRNB_FS;


	/* AMR NB by definition */
	q6_format->sampling_rate = 8000;
	q6_format->channels = 1;
	q6_format->bits_per_sample = 16;
	q6_format->is_signed = true;
	q6_format->is_interleaved = false;


	if (cad_open_struct->cad_open.op_code == CAD_OPEN_OP_WRITE)
		return CAD_RES_SUCCESS;

	/* Fill in encoder config. */
	q6_open_struct->open_data.config.amr_cfg.mode =
		q6_band_mode_mapping(cad_format->amr_band_mode);
	if (q6_open_struct->open_data.config.amr_cfg.mode == 0xFFFFFFFF) {
		pr_err("CAD:ARD: unsupported amr_band_mode!");
		return CAD_RES_FAILURE;
	}
	q6_open_struct->open_data.config.amr_cfg.dtx_mode =
		q6_dtx_mode_mapping(cad_format->amr_dtx_mode);
	if (q6_open_struct->open_data.config.amr_cfg.dtx_mode ==
		0xFFFFFFFF) {

		pr_err("CAD:ARD: unsupported amr_dtx_mode!");
		return CAD_RES_FAILURE;
	}
	q6_open_struct->open_data.config.amr_cfg.enable = true;

	return result;
}



s32 convert_evrc_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_evrc_format			*cad_format = NULL;
	struct adsp_audio_standard_format	*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(*cad_format))) {

		pr_err("No format block provided for evrc\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = (struct cad_evrc_format *)
		(cad_open_struct->cad_config.format_block);
	q6_format = (struct adsp_audio_standard_format *)
		&(q6_open_struct->open_data.format_block);


	if (cad_open_struct->cad_open.format == CAD_FORMAT_EVRCB)
		q6_format->format = ADSP_AUDIO_FORMAT_EVRCB_FS;
	else
		q6_format->format = ADSP_AUDIO_FORMAT_EVRC_FS;

	q6_format->sampling_rate = 8000;
	q6_format->channels = 1;
	q6_format->bits_per_sample = 16;
	q6_format->is_signed = true;
	q6_format->is_interleaved = false;


	/* Fill in encoder config. */
	q6_open_struct->open_data.config.evrc_cfg.min_rate =
		(u16)(cad_format->min_bit_rate);
	q6_open_struct->open_data.config.evrc_cfg.max_rate =
		(u16)(cad_format->max_bit_rate);

	return result;
}



s32 convert_v13k_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_qcelp13k_format		*cad_format = NULL;
	struct adsp_audio_standard_format	*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(*cad_format))) {

		pr_err("No format block provided for qcelp13k\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = (struct cad_qcelp13k_format *)
		(cad_open_struct->cad_config.format_block);
	q6_format = (struct adsp_audio_standard_format *)
		&(q6_open_struct->open_data.format_block);

	q6_format->format = ADSP_AUDIO_FORMAT_V13K_FS;
	q6_format->sampling_rate = 8000;
	q6_format->channels = 1;
	q6_format->bits_per_sample = 16;
	q6_format->is_signed = true;
	q6_format->is_interleaved = false;


	/* Fill in encoder config. */
	q6_open_struct->open_data.config.qcelp13k_cfg.min_rate =
		(u16)(cad_format->min_bit_rate);
	q6_open_struct->open_data.config.qcelp13k_cfg.max_rate =
		(u16)(cad_format->max_bit_rate);

	return result;
}

s32 convert_midi_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_write_midi_struct_type	*cad_format = NULL;
	struct adsp_audio_midi_format		*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(struct cad_write_midi_format_struct_type))) {

		pr_err("No format block provided for midi\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = &(((struct cad_write_midi_format_struct_type *)
		(cad_open_struct->cad_config.format_block))->midi);
	q6_format = (struct adsp_audio_midi_format *)
		&(q6_open_struct->open_data.format_block);

	q6_format->format = ADSP_AUDIO_FORMAT_MIDI;
	q6_format->sampling_rate = 48000;
	q6_format->channels = cad_format->midi_stereo;
	q6_format->mode = cad_format->volume_lookup_index;

	return result;
}




s32 convert_mp3_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct adsp_audio_standard_format	*q6_format = NULL;


	/* Save and translate the cmdbuff passed in */
	q6_format = (struct adsp_audio_standard_format *)
		&(q6_open_struct->open_data.format_block);

	q6_format->format = ADSP_AUDIO_FORMAT_MP3;
	q6_format->sampling_rate = 48000;
	q6_format->channels = 2;
	q6_format->bits_per_sample = 16;
	q6_format->is_signed = true;
	q6_format->is_interleaved = false;

	return result;
}


s32 convert_sbc_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_sbc_enc_cfg_struct_type	*cad_format = NULL;
	struct adsp_audio_standard_format	*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(*cad_format))) {

		pr_err("No format block provided for sbc\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = (struct cad_sbc_enc_cfg_struct_type *)
		(cad_open_struct->cad_config.format_block);
	q6_format = (struct adsp_audio_standard_format *)
		&(q6_open_struct->open_data.format_block);

	q6_format->format = ADSP_AUDIO_FORMAT_SBC;
	q6_format->sampling_rate = 48000;
	q6_format->channels = 2;
	q6_format->bits_per_sample = 16;
	q6_format->is_signed = true;
	q6_format->is_interleaved = false;


	/* Fill in encoder config. */
	q6_open_struct->open_data.config.sbc_cfg.num_subbands =
		cad_format->num_subbands;
	q6_open_struct->open_data.config.sbc_cfg.block_len =
		cad_format->block_len;
	q6_open_struct->open_data.config.sbc_cfg.channel_mode =
		cad_format->channel_mode;
	q6_open_struct->open_data.config.sbc_cfg.allocation_method =
		cad_format->allocation_method;
	q6_open_struct->open_data.config.sbc_cfg.bit_rate =
		cad_format->bit_rate;

	return result;
}


s32 convert_wma_std_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct)
{
	s32					result = CAD_RES_SUCCESS;
	struct cad_wma_format			*cad_format = NULL;
	struct adsp_audio_wma_pro_format	*q6_format = NULL;

	if ((cad_open_struct->cad_config.format_block == NULL) ||
		(cad_open_struct->cad_config.format_block_len !=
		sizeof(*cad_format))) {

		pr_err("No format block provided for WMA std\n");
		return CAD_RES_FAILURE;
	}

	/* Save and translate the cmdbuff passed in */
	cad_format = (struct cad_wma_format *)
		(cad_open_struct->cad_config.format_block);
	q6_format = (struct adsp_audio_wma_pro_format *)
		&(q6_open_struct->open_data.format_block);

	if (cad_open_struct->cad_open.format == CAD_FORMAT_WMA_PRO)
		q6_format->format = ADSP_AUDIO_FORMAT_WMA_V9PRO;
	else
		q6_format->format = ADSP_AUDIO_FORMAT_WMA_V9;

	q6_format->format_tag = cad_format->format_tag;
	q6_format->channels = cad_format->channels;
	q6_format->samples_per_sec = cad_format->samples_per_sec;
	q6_format->avg_bytes_per_sec = cad_format->avg_bytes_per_sec;
	q6_format->block_align = cad_format->block_align;
	q6_format->valid_bits_per_sample = cad_format->valid_bits_per_sample;
	q6_format->channel_mask = cad_format->channel_mask;
	q6_format->encode_opt = cad_format->encode_opt;
	q6_format->advanced_encode_opt = cad_format->advanced_encode_opt;
	q6_format->advanced_encode_opt2 = cad_format->advanced_encode_opt2;
	q6_format->drc_peak_reference = cad_format->drc_peak_reference;
	q6_format->drc_peak_target = cad_format->drc_peak_target;
	q6_format->drc_average_reference = cad_format->drc_average_reference;
	q6_format->drc_average_target = cad_format->drc_average_target;

	return result;
}

u32 q6_dtx_mode_mapping(u32 dtx_mode)
{
	switch (dtx_mode) {
	case CAD_AMR_DTX_OFF:
		return ADSP_AUDIO_AMR_DTX_MODE_OFF;
	case CAD_AMR_DTX_VAD1:
		return ADSP_AUDIO_AMR_DTX_MODE_ON_VAD1;
	case CAD_AMR_DTX_VAD2:
		return ADSP_AUDIO_AMR_DTX_MODE_ON_VAD2;
	case CAD_AMR_DTX_AUTO:
		return ADSP_AUDIO_AMR_DTX_MODE_ON_AUTO;
	case CAD_AMR_DTX_EFR:
	default:
		/* error */
		return 0xFFFFFFFF;
	}
}

u32 q6_band_mode_mapping(u32 mode)
{
	switch (mode) {
	case CAD_AMR_BM_NB0:
	case CAD_AMR_BM_WB0:
		return ADSP_AUDIO_AMR_MR475;
	case CAD_AMR_BM_NB1:
	case CAD_AMR_BM_WB1:
		return ADSP_AUDIO_AMR_MR515;
	case CAD_AMR_BM_NB2:
	case CAD_AMR_BM_WB2:
		return ADSP_AUDIO_AMR_MMR59;
	case CAD_AMR_BM_NB3:
	case CAD_AMR_BM_WB3:
		return ADSP_AUDIO_AMR_MMR67;
	case CAD_AMR_BM_NB4:
	case CAD_AMR_BM_WB4:
		return ADSP_AUDIO_AMR_MMR74;
	case CAD_AMR_BM_NB5:
	case CAD_AMR_BM_WB5:
		return ADSP_AUDIO_AMR_MMR795;
	case CAD_AMR_BM_NB6:
	case CAD_AMR_BM_WB6:
		return ADSP_AUDIO_AMR_MMR102;
	case CAD_AMR_BM_NB7:
	case CAD_AMR_BM_WB7:
		return ADSP_AUDIO_AMR_MMR122;
	case CAD_AMR_BM_WB8:
	default:
		/* error */
		return 0xFFFFFFFF;
	}
}


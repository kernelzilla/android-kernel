/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#include "vcd_ddl.h"
#include "vcd_ddl_shared_mem.h"
#include "vcd_core.h"

#if defined(PIX_CACHE_DISABLE)
#define DDL_PIX_CACHE_ENABLE  FALSE
#else
#define DDL_PIX_CACHE_ENABLE  TRUE
#endif

void ddl_vidc_core_init(struct ddl_context_type *p_ddl_context)
{
	struct vidc_1080P_pix_cache_config_type pixel_cache_config;

	vidc_1080p_do_sw_reset(VIDC_1080P_RESET_IN_SEQ_FIRST_STAGE);
	msleep(DDL_SW_RESET_SLEEP);
	vidc_1080p_do_sw_reset(VIDC_1080P_RESET_IN_SEQ_SECOND_STAGE);
	vidc_1080p_init_memory_controller(
		(u32) p_ddl_context->dram_base_a.p_align_physical_addr,
		(u32) p_ddl_context->dram_base_b.p_align_physical_addr);
	vidc_1080p_clear_returned_channel_inst_id();
	p_ddl_context->vidc_decode_seq_start[0] =
		vidc_1080p_decode_seq_start_ch0;
	p_ddl_context->vidc_decode_seq_start[1] =
		vidc_1080p_decode_seq_start_ch1;
	p_ddl_context->vidc_decode_init_buffers[0] =
		vidc_1080p_decode_init_buffers_ch0;
	p_ddl_context->vidc_decode_init_buffers[1] =
		vidc_1080p_decode_init_buffers_ch1;
	p_ddl_context->vidc_decode_frame_start[0] =
		vidc_1080p_decode_frame_start_ch0;
	p_ddl_context->vidc_decode_frame_start[1] =
		vidc_1080p_decode_frame_start_ch1;
	p_ddl_context->vidc_set_divx3_resolution[0] =
		vidc_1080p_set_divx3_resolution_ch0;
	p_ddl_context->vidc_set_divx3_resolution[1] =
		vidc_1080p_set_divx3_resolution_ch1;
	p_ddl_context->vidc_encode_seq_start[0] =
		vidc_1080p_encode_seq_start_ch0;
	p_ddl_context->vidc_encode_seq_start[1] =
		vidc_1080p_encode_seq_start_ch1;
	p_ddl_context->vidc_encode_frame_start[0] =
		vidc_1080p_encode_frame_start_ch0;
	p_ddl_context->vidc_encode_frame_start[1] =
		vidc_1080p_encode_frame_start_ch1;
	vidc_1080p_release_sw_reset();
	p_ddl_context->b_pix_cache_enable = DDL_PIX_CACHE_ENABLE;
	if (p_ddl_context->b_pix_cache_enable) {
		vidc_pix_cache_sw_reset();
		pixel_cache_config.b_cache_enable = TRUE;
		pixel_cache_config.b_prefetch_en = TRUE;
		pixel_cache_config.e_port_select = VIDC_1080P_PIX_CACHE_PORT_B;
		pixel_cache_config.b_statistics_off = TRUE;
		pixel_cache_config.e_page_size =
			VIDC_1080P_PIX_CACHE_PAGE_SIZE_1K;
		vidc_pix_cache_init_config(&pixel_cache_config);
	}
}

void ddl_vidc_core_term(struct ddl_context_type *p_ddl_context)
{
	if (p_ddl_context->b_pix_cache_enable) {
		u32 b_pix_cache_idle = FALSE;
		u32 n_counter = 0;

		vidc_pix_cache_set_halt(TRUE);

		do {
			msleep(DDL_SW_RESET_SLEEP);
			vidc_pix_cache_get_status_idle(&b_pix_cache_idle);
			n_counter++;
		} while (!b_pix_cache_idle &&
			n_counter < DDL_PIXEL_CACHE_STATUS_READ_RETRY);

		if (!b_pix_cache_idle) {
			p_ddl_context->n_cmd_err_status =
				DDL_PIXEL_CACHE_NOT_IDLE;
			ddl_handle_core_errors(p_ddl_context);
		}
	}
}

void ddl_vidc_channel_set(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	enum vcd_codec_type *p_codec;
	enum vidc_1080p_codec_type e_codec = VIDC_1080P_H264_DECODE;
	const enum vidc_1080p_decode_p_cache_enable_type
	e_dec_p_cache = VIDC_1080P_DECODE_PCACHE_DISABLE;

	const enum vidc_1080p_encode_p_cache_enable_type
	e_enc_p_cache = VIDC_1080P_ENCODE_PCACHE_ENABLE;
	u32 n_p_cache_ctrl, n_ctxt_mem_offset, n_ctxt_mem_size;

	if (p_ddl->b_decoding) {
		p_codec = &(p_ddl->codec_data.decoder.codec_type.e_codec);
		n_p_cache_ctrl = (u32)e_dec_p_cache;
		n_ctxt_mem_offset = DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
		p_ddl->codec_data.decoder.hw_bufs.context) >> 11;
		n_ctxt_mem_size =
			p_ddl->codec_data.decoder.hw_bufs.context.n_buffer_size;
	} else {
		p_codec = &(p_ddl->codec_data.encoder.codec_type.e_codec);
		n_p_cache_ctrl = (u32)e_enc_p_cache;
		n_ctxt_mem_offset = DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
			p_ddl->codec_data.encoder.hw_bufs.context) >> 11;
		n_ctxt_mem_size =
			p_ddl->codec_data.encoder.hw_bufs.context.n_buffer_size;
	}
	switch (*p_codec) {
	default:
	case VCD_CODEC_MPEG4:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_MPEG4_DECODE;
		else
			e_codec = VIDC_1080P_MPEG4_ENCODE;
	break;
	case VCD_CODEC_H264:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_H264_DECODE;
		else
			e_codec = VIDC_1080P_H264_ENCODE;
	break;
	case VCD_CODEC_DIVX_3:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_DIVX311_DECODE;
	break;
	case VCD_CODEC_DIVX_4:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_DIVX412_DECODE;
	break;
	case VCD_CODEC_DIVX_5:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_DIVX502_DECODE;
	break;
	case VCD_CODEC_DIVX_6:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_DIVX503_DECODE;
	break;
	case VCD_CODEC_XVID:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_MPEG4_DECODE;
	break;
	case VCD_CODEC_H263:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_H263_DECODE;
		else
			e_codec = VIDC_1080P_H263_ENCODE;
	break;
	case VCD_CODEC_MPEG1:
	case VCD_CODEC_MPEG2:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_MPEG2_DECODE;
	break;
	case VCD_CODEC_VC1:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_VC1_DECODE;
	break;
	case VCD_CODEC_VC1_RCV:
		if (p_ddl->b_decoding)
			e_codec = VIDC_1080P_VC1_RCV_DECODE;
	break;
	}
	p_ddl->e_cmd_state = DDL_CMD_CHANNEL_SET;
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_CHDONE",
	ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_CHDONE;
	vidc_1080p_set_host2risc_cmd(VIDC_1080P_HOST2RISC_CMD_OPEN_CH,
		(u32)e_codec, n_p_cache_ctrl, n_ctxt_mem_offset,
		n_ctxt_mem_size);
}

void ddl_vidc_decode_init_codec(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type  *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vidc_1080p_dec_seq_start_param_type seq_start_param;
	u32 seq_size;

	if ((p_decoder->codec_type.e_codec == VCD_CODEC_DIVX_3))
		p_ddl_context->vidc_set_divx3_resolution
		[p_ddl->n_command_channel](p_decoder->client_frame_size.n_width,
		p_decoder->client_frame_size.n_height);
	else
	p_ddl_context->vidc_set_divx3_resolution
	[p_ddl->n_command_channel](0x0, 0x0);
	DDL_MSG_LOW("HEADER-PARSE-START");
	DDL_MSG_LOW("ddl_state_transition: %s ~~> \
	DDL_CLIENT_WAIT_FOR_INITCODECDONE",
	ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_INITCODECDONE;
	p_ddl->e_cmd_state = DDL_CMD_HEADER_PARSE;
	seq_start_param.n_cmd_seq_num = ++p_ddl_context->n_cmd_seq_num;
	seq_start_param.n_inst_id = p_ddl->n_instance_id;
	seq_start_param.n_shared_mem_addr_offset = DDL_ADDR_OFFSET(
	p_ddl_context->dram_base_a, p_ddl->shared_mem
		[p_ddl->n_command_channel]);
	seq_start_param.n_stream_buffer_addr_offset =
	DDL_OFFSET(p_ddl_context->dram_base_a.p_align_physical_addr,
	p_decoder->decode_config.p_sequence_header);
	seq_start_param.n_stream_buffersize =
		p_decoder->client_input_buf_req.n_size;
	seq_size = p_decoder->decode_config.n_sequence_header_len +
		DDL_LINEAR_BUFFER_ALIGN_BYTES + VCD_SEQ_HDR_PADDING_BYTES;
	if (seq_start_param.n_stream_buffersize < seq_size)
		seq_start_param.n_stream_buffersize = seq_size;
	seq_start_param.n_stream_frame_size =
		p_decoder->decode_config.n_sequence_header_len;
	seq_start_param.n_descriptor_buffer_addr_offset =
		DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
		p_decoder->hw_bufs.desc),
	seq_start_param.n_descriptor_buffer_size =
		p_decoder->hw_bufs.desc.n_buffer_size;
	p_ddl_context->vidc_decode_seq_start[p_ddl->n_command_channel](
		&seq_start_param);
}

void ddl_vidc_decode_dynamic_property(struct ddl_client_context_type *p_ddl,
	u32 b_enable)
{
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vcd_frame_data_type *p_bit_stream =
		&(p_ddl->input_frame.vcd_frm);
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;

	if (!b_enable) {
		if (p_decoder->b_dynmic_prop_change_req)
			p_decoder->b_dynmic_prop_change_req = FALSE;
		return;
	}
	if ((p_decoder->n_dynamic_prop_change & DDL_DEC_REQ_OUTPUT_FLUSH)) {
		p_decoder->b_dynmic_prop_change_req = TRUE;
		p_decoder->n_dynamic_prop_change &= ~(DDL_DEC_REQ_OUTPUT_FLUSH);
		p_decoder->dpb_mask.n_hw_mask = 0;
		p_decoder->b_flush_pending = TRUE;
	}
	if (((p_decoder->n_meta_data_enable_flag & VCD_METADATA_PASSTHROUGH)) &&
		((VCD_FRAME_FLAG_EXTRADATA & p_bit_stream->n_flags))) {
		u32 b_extradata_presence = TRUE;
		u8* p_tmp = ((u8 *) p_bit_stream->p_physical +
				p_bit_stream->n_offset +
				p_bit_stream->n_data_len + 3);
		u32 n_extra_data_start = (u32) ((u32)p_tmp & ~3);

		n_extra_data_start = n_extra_data_start -
			(u32)p_ddl_context->dram_base_a.p_align_physical_addr;
		p_decoder->b_dynmic_prop_change_req = TRUE;
		vidc_sm_set_extradata_addr(&p_ddl->shared_mem
			[p_ddl->n_command_channel], n_extra_data_start);
		vidc_sm_set_extradata_presence(&p_ddl->shared_mem
			[p_ddl->n_command_channel], b_extradata_presence);
	}
}

void ddl_vidc_encode_dynamic_property(struct ddl_client_context_type *p_ddl,
	u32 b_enable)
{
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	u32 b_frame_rate_change = FALSE, b_bit_rate_change = FALSE;
	u32 b_i_period_change = FALSE, b_reset_req = FALSE;

	if (!b_enable) {
		if (p_encoder->b_dynmic_prop_change_req) {
			b_reset_req = TRUE;
			p_encoder->b_dynmic_prop_change_req = FALSE;
		}
	} else {
		if ((p_encoder->n_dynamic_prop_change & DDL_ENC_REQ_IFRAME)) {
			p_encoder->b_intra_frame_insertion = TRUE;
			p_encoder->n_dynamic_prop_change &=
				~(DDL_ENC_REQ_IFRAME);
		}
		if ((p_encoder->n_dynamic_prop_change &
			DDL_ENC_CHANGE_BITRATE)) {
			b_bit_rate_change = TRUE;
			vidc_sm_set_encoder_new_bit_rate(
				&p_ddl->shared_mem[p_ddl->n_command_channel],
				p_encoder->target_bit_rate.n_target_bitrate);
			p_encoder->n_dynamic_prop_change &=
				~(DDL_ENC_CHANGE_BITRATE);
		}
		if ((p_encoder->n_dynamic_prop_change
			& DDL_ENC_CHANGE_IPERIOD)) {
			b_i_period_change = TRUE;
			vidc_sm_set_encoder_new_i_period(
				&p_ddl->shared_mem[p_ddl->n_command_channel],
				p_encoder->i_period.n_p_frames);
			p_encoder->n_dynamic_prop_change &=
				~(DDL_ENC_CHANGE_IPERIOD);
		}
		if ((p_encoder->n_dynamic_prop_change
			& DDL_ENC_CHANGE_FRAMERATE)) {
			b_frame_rate_change = TRUE;
			vidc_sm_set_encoder_new_frame_rate(
				&p_ddl->shared_mem[p_ddl->n_command_channel],
				(u32)(DDL_FRAMERATE_SCALE(p_encoder->\
				frame_rate.n_fps_numerator) /
				p_encoder->frame_rate.n_fps_denominator));
			p_encoder->n_dynamic_prop_change &=
				~(DDL_ENC_CHANGE_FRAMERATE);
		}
	}
	if ((b_enable) || (b_reset_req)) {
		vidc_sm_set_encoder_param_change(
			&p_ddl->shared_mem[p_ddl->n_command_channel],
			b_bit_rate_change, b_frame_rate_change,
			b_i_period_change);
	}
}

static void ddl_vidc_encode_set_profile_level(
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	u32  n_encode_profile, n_level = 0;

	switch (p_encoder->profile.e_profile) {
	default:
	case VCD_PROFILE_MPEG4_SP:
		n_encode_profile = VIDC_1080P_PROFILE_MPEG4_SIMPLE;
	break;
	case VCD_PROFILE_MPEG4_ASP:
		n_encode_profile = VIDC_1080P_PROFILE_MPEG4_ADV_SIMPLE;
	break;
	case VCD_PROFILE_H264_BASELINE:
		n_encode_profile = VIDC_1080P_PROFILE_H264_BASELINE;
	break;
	case VCD_PROFILE_H264_MAIN:
		n_encode_profile = VIDC_1080P_PROFILE_H264_MAIN;
	break;
	case VCD_PROFILE_H264_HIGH:
		n_encode_profile = VIDC_1080P_PROFILE_H264_HIGH;
	break;
	}
	switch (p_encoder->level.e_level) {
	default:
	case VCD_LEVEL_MPEG4_0:
		n_level = VIDC_1080P_MPEG4_LEVEL0;
	break;
	case VCD_LEVEL_MPEG4_0b:
		n_level = VIDC_1080P_MPEG4_LEVEL0b;
	break;
	case VCD_LEVEL_MPEG4_1:
		n_level = VIDC_1080P_MPEG4_LEVEL1;
	break;
	case VCD_LEVEL_MPEG4_2:
		n_level = VIDC_1080P_MPEG4_LEVEL2;
	break;
	case VCD_LEVEL_MPEG4_3:
		n_level = VIDC_1080P_MPEG4_LEVEL3;
	break;
	case VCD_LEVEL_MPEG4_3b:
		n_level = VIDC_1080P_MPEG4_LEVEL3b;
	break;
	case VCD_LEVEL_MPEG4_4:
		n_level = VIDC_1080P_MPEG4_LEVEL4;
	break;
	case VCD_LEVEL_MPEG4_4a:
		n_level = VIDC_1080P_MPEG4_LEVEL4a;
	break;
	case VCD_LEVEL_MPEG4_5:
		n_level = VIDC_1080P_MPEG4_LEVEL5;
	break;
	case VCD_LEVEL_MPEG4_6:
		n_level = VIDC_1080P_MPEG4_LEVEL6;
	break;
	case VCD_LEVEL_MPEG4_7:
		n_level = VIDC_1080P_MPEG4_LEVEL7;
	break;
	case VCD_LEVEL_H264_1:
		n_level = VIDC_1080P_H264_LEVEL1;
	break;
	case VCD_LEVEL_H264_1b:
		n_level = VIDC_1080P_H264_LEVEL1b;
	break;
	case VCD_LEVEL_H264_1p1:
		n_level = VIDC_1080P_H264_LEVEL1p1;
	break;
	case VCD_LEVEL_H264_1p2:
		n_level = VIDC_1080P_H264_LEVEL1p2;
	break;
	case VCD_LEVEL_H264_1p3:
		n_level = VIDC_1080P_H264_LEVEL1p3;
	break;
	case VCD_LEVEL_H264_2:
		n_level = VIDC_1080P_H264_LEVEL2;
	break;
	case VCD_LEVEL_H264_2p1:
		n_level = VIDC_1080P_H264_LEVEL2p1;
	break;
	case VCD_LEVEL_H264_2p2:
		n_level = VIDC_1080P_H264_LEVEL2p2;
	break;
	case VCD_LEVEL_H264_3:
		n_level = VIDC_1080P_H264_LEVEL3;
	break;
	case VCD_LEVEL_H264_3p1:
		n_level = VIDC_1080P_H264_LEVEL3p1;
	break;
	case VCD_LEVEL_H264_3p2:
		n_level = VIDC_1080P_H264_LEVEL3p2;
	break;
	case VCD_LEVEL_H264_4:
		n_level = VIDC_1080P_H264_LEVEL4;
	break;
	case VCD_LEVEL_H263_10:
		n_level = VIDC_1080P_H263_LEVEL10;
	break;
	case VCD_LEVEL_H263_20:
		n_level = VIDC_1080P_H263_LEVEL20;
	break;
	case VCD_LEVEL_H263_30:
		n_level = VIDC_1080P_H263_LEVEL30;
	break;
	case VCD_LEVEL_H263_40:
		n_level = VIDC_1080P_H263_LEVEL40;
	break;
	case VCD_LEVEL_H263_45:
		n_level = VIDC_1080P_H263_LEVEL45;
	break;
	case VCD_LEVEL_H263_50:
		n_level = VIDC_1080P_H263_LEVEL50;
	break;
	case VCD_LEVEL_H263_60:
		n_level = VIDC_1080P_H263_LEVEL60;
	break;
	case VCD_LEVEL_H263_70:
		n_level = VIDC_1080P_H263_LEVEL70;
	break;
	}
	vidc_1080p_set_encode_profile_level(n_encode_profile, n_level);
}

void ddl_vidc_encode_init_codec(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_encoder_data_type *p_encoder = &(p_ddl->codec_data.encoder);
	struct ddl_enc_buffers_type *p_enc_buffers = &p_encoder->hw_bufs;
	struct vidc_1080p_enc_seq_start_param_type seq_start_param;
	enum vidc_1080p_memory_access_method_type mem_access_method;
	enum vidc_1080p_DBConfig_type e_db_config;
	enum vidc_1080p_MSlice_selection_type e_m_slice_sel;
	enum VIDC_SM_frame_skip_type e_r_cframe_skip =
		VIDC_SM_FRAME_SKIP_DISABLE;
	u32 i_multi_slice_size = 0, i_multi_slice_byte = 0;
	u32 n_index, a_luma[4], a_chroma[4], b_hdr_ext_control = FALSE;
	const u32 n_recon_bufs = 4;

	ddl_vidc_encode_set_profile_level(p_ddl);
	vidc_1080p_set_encode_frame_size(p_encoder->frame_size.n_width,
		p_encoder->frame_size.n_height);
	vidc_1080p_encode_set_qp_params(p_encoder->qp_range.n_max_qp,
		p_encoder->qp_range.n_min_qp);
	vidc_1080p_encode_set_rc_config(p_encoder->rc_level.b_frame_level_rc,
		p_encoder->rc_level.b_mb_level_rc,
		p_encoder->session_qp.n_i_frame_qp);
	if (p_encoder->n_hdr_ext_control > 0)
		b_hdr_ext_control = TRUE;
	if (p_encoder->n_r_cframe_skip > 0)
		e_r_cframe_skip = VIDC_SM_FRAME_SKIP_ENABLE_LEVEL;
	vidc_sm_set_extended_encoder_control(&p_ddl->shared_mem
		[p_ddl->n_command_channel], b_hdr_ext_control,
		e_r_cframe_skip, FALSE, 0);
	vidc_sm_set_encoder_hec_period(&p_ddl->shared_mem
		[p_ddl->n_command_channel], p_encoder->n_hdr_ext_control);
	if ((p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4) &&
		(p_encoder->vop_timing.n_vop_time_resolution > 0)) {
		vidc_sm_set_encoder_vop_time(&p_ddl->shared_mem
			[p_ddl->n_command_channel], TRUE,
			p_encoder->vop_timing.n_vop_time_resolution, 0);
	}
	if (p_encoder->rc_level.b_frame_level_rc)
		vidc_1080p_encode_set_frame_level_rc_params((
			DDL_FRAMERATE_SCALE(p_encoder->\
			frame_rate.n_fps_numerator) /
			p_encoder->frame_rate.n_fps_denominator),
			p_encoder->target_bit_rate.n_target_bitrate,
			p_encoder->frame_level_rc.n_reaction_coeff);
	if (p_encoder->rc_level.b_mb_level_rc)
		vidc_1080p_encode_set_mb_level_rc_params(
			p_encoder->adaptive_rc.b_dark_region_as_flag,
			p_encoder->adaptive_rc.b_smooth_region_as_flag,
			p_encoder->adaptive_rc.b_static_region_as_flag,
			p_encoder->adaptive_rc.b_activity_region_flag);
	if ((!p_encoder->rc_level.b_frame_level_rc) &&
		(!p_encoder->rc_level.b_mb_level_rc))
		vidc_sm_set_pand_b_frame_qp(
			&p_ddl->shared_mem[p_ddl->n_command_channel],
			p_encoder->session_qp.n_b_frame_qp,
			p_encoder->session_qp.n_p_frame_qp);
	if (p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4) {
		vidc_1080p_set_mpeg4_encode_quarter_pel_control(FALSE);
		vidc_1080p_set_encode_field_picture_structure(FALSE);
	}
	if (p_encoder->codec_type.e_codec == VCD_CODEC_H264) {
		enum vidc_1080p_entropy_sel_type e_entropy_sel;
		switch (p_encoder->entropy_control.e_entropy_sel) {
		default:
		case VCD_ENTROPY_SEL_CAVLC:
			e_entropy_sel = VIDC_1080P_ENTROPY_SEL_CAVLC;
		break;
		case VCD_ENTROPY_SEL_CABAC:
			e_entropy_sel = VIDC_1080P_ENTROPY_SEL_CABAC;
		break;
	}
	vidc_1080p_set_h264_encode_entropy(e_entropy_sel);
	switch (p_encoder->db_control.e_db_config) {
	default:
	case VCD_DB_ALL_BLOCKING_BOUNDARY:
		e_db_config = VIDC_1080P_DB_ALL_BLOCKING_BOUNDARY;
	break;
	case VCD_DB_DISABLE:
		e_db_config = VIDC_1080P_DB_DISABLE;
	break;
	case VCD_DB_SKIP_SLICE_BOUNDARY:
		e_db_config = VIDC_1080P_DB_SKIP_SLICE_BOUNDARY;
	break;
	}
	vidc_1080p_set_h264_encode_loop_filter(e_db_config,
		p_encoder->db_control.n_slice_alpha_offset,
		p_encoder->db_control.n_slice_beta_offset);
	vidc_1080p_set_h264_encoder_ref_count(p_encoder->\
		n_num_references_for_p_frame);
	if (p_encoder->profile.e_profile == VCD_PROFILE_H264_HIGH)
		vidc_1080p_set_h264_encode_8x8transform_control(TRUE);
	}
	vidc_1080p_set_encode_picture_type(p_encoder->i_period.n_p_frames,
		p_encoder->i_period.n_b_frames);
	vidc_1080p_set_encode_circular_intra_refresh(
		p_encoder->intra_refresh.n_cir_mb_number);
	switch (p_encoder->multi_slice.e_m_slice_sel) {
	default:
	case VCD_MSLICE_OFF:
		e_m_slice_sel = VIDC_1080P_MSLICE_DISABLE;
	break;
	case VCD_MSLICE_BY_MB_COUNT:
		e_m_slice_sel = VIDC_1080P_MSLICE_BY_MB_COUNT;
		i_multi_slice_size = p_encoder->multi_slice.n_m_slice_size;
	break;
	case VCD_MSLICE_BY_BYTE_COUNT:
		e_m_slice_sel = VIDC_1080P_MSLICE_BY_BYTE_COUNT;
		i_multi_slice_byte = p_encoder->multi_slice.n_m_slice_size;
	break;
	}
	vidc_1080p_set_encode_multi_slice_control(e_m_slice_sel,
		i_multi_slice_size, i_multi_slice_byte);
	a_luma[0] = DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
			p_enc_buffers->dpb_y[0]);
	a_luma[1] = DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
			p_enc_buffers->dpb_y[1]);
	a_luma[2] = DDL_ADDR_OFFSET(p_ddl_context->dram_base_b,
			p_enc_buffers->dpb_y[2]);
	a_luma[3] = DDL_ADDR_OFFSET(p_ddl_context->dram_base_b,
			p_enc_buffers->dpb_y[3]);
	for (n_index = 0; n_index < n_recon_bufs; n_index++)
		a_chroma[n_index] = DDL_ADDR_OFFSET(p_ddl_context->dram_base_b,
					p_enc_buffers->dpb_c[n_index]);
	vidc_1080p_set_encode_recon_buffers(n_recon_bufs, a_luma, a_chroma);
	switch (p_encoder->codec_type.e_codec) {
	case VCD_CODEC_MPEG4:
		vidc_1080p_set_mpeg4_encode_work_buffers(
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->col_zero),
				DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->acdc_coef),
				DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->mv));
	break;
	case VCD_CODEC_H263:
		vidc_1080p_set_h263_encode_work_buffers(
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->mv),
				DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->acdc_coef));
	break;
	case VCD_CODEC_H264:
		vidc_1080p_set_h264_encode_work_buffers(
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->mv),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->col_zero),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->md),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_b,
				p_enc_buffers->pred),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->nbor_info),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_enc_buffers->mb_info));
	break;
	default:
	break;
	}
	if (p_encoder->buf_format.e_buffer_format ==
		VCD_BUFFER_FORMAT_NV12_16M2KA)
		mem_access_method = VIDC_1080P_TILE_LINEAR;
	else
		mem_access_method = VIDC_1080P_TILE_64x32;
	vidc_1080p_set_encode_input_frame_format(mem_access_method);
	vidc_1080p_set_encode_padding_control(0, 0, 0, 0);
	DDL_MSG_LOW("ddl_state_transition: %s ~~> \
		DDL_CLIENT_WAIT_FOR_INITCODECDONE",
		ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_INITCODECDONE;
	p_ddl->e_cmd_state = DDL_CMD_INIT_CODEC;
	vidc_1080p_set_encode_field_picture_structure(FALSE);
	seq_start_param.n_cmd_seq_num = ++p_ddl_context->n_cmd_seq_num;
	seq_start_param.n_inst_id = p_ddl->n_instance_id;
	seq_start_param.n_shared_mem_addr_offset = DDL_ADDR_OFFSET(
		p_ddl_context->dram_base_a, p_ddl->shared_mem
		[p_ddl->n_command_channel]);
	seq_start_param.n_stream_buffer_addr_offset = DDL_ADDR_OFFSET(
		p_ddl_context->dram_base_a, p_encoder->seq_header);
	seq_start_param.n_stream_buffer_size =
		p_encoder->seq_header.n_buffer_size;
	p_encoder->n_seq_header_length = 0;
	p_ddl_context->vidc_encode_seq_start[p_ddl->n_command_channel](
		&seq_start_param);
}

void ddl_vidc_channel_end(struct ddl_client_context_type *p_ddl)
{
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_CHEND",
	ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_CHEND;
	p_ddl->e_cmd_state = DDL_CMD_CHANNEL_END;
	vidc_1080p_set_host2risc_cmd(VIDC_1080P_HOST2RISC_CMD_CLOSE_CH,
		p_ddl->n_instance_id, 0, 0, 0);
}

void ddl_vidc_encode_frame_run(struct ddl_client_context_type *p_ddl)
{
	struct vidc_1080p_enc_frame_start_param_type enc_param;
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_encoder_data_type  *p_encoder = &(p_ddl->codec_data.encoder);
	struct ddl_enc_buffers_type *p_enc_buffers = &(p_encoder->hw_bufs);
	struct vcd_frame_data_type *p_stream = &(p_ddl->output_frame.vcd_frm);
	struct vcd_frame_data_type *p_input_vcd_frm =
		&(p_ddl->input_frame.vcd_frm);
	u32 a_dpb_addr_y[4], a_dpb_addr_c[4];
	u32 n_index, n_y_addr, n_c_addr;

	n_y_addr = DDL_OFFSET(p_ddl_context->dram_base_b.p_align_physical_addr,
			p_input_vcd_frm->p_physical);
	n_c_addr = (n_y_addr + p_encoder->input_buf_size.n_size_y);
	if (p_input_vcd_frm->n_flags & VCD_FRAME_FLAG_EOS) {
		enc_param.e_encode_type = VIDC_1080P_ENC_TYPE_LAST_FRAME_DATA;
		DDL_MSG_LOW("ddl_state_transition: %s ~~>\
			DDL_CLIENT_WAIT_FOR_EOS_DONE",
			ddl_get_state_string(p_ddl->e_client_state));
		p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_EOS_DONE;
	} else {
		enc_param.e_encode_type = VIDC_1080P_ENC_TYPE_FRAME_DATA;
		DDL_MSG_LOW("ddl_state_transition: %s ~~> \
			DDL_CLIENT_WAIT_FOR_FRAME_DONE",
			ddl_get_state_string(p_ddl->e_client_state));
		p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_FRAME_DONE;
	}
	p_ddl->e_cmd_state = DDL_CMD_ENCODE_FRAME;
	if (p_encoder->n_dynamic_prop_change) {
		p_encoder->b_dynmic_prop_change_req = TRUE;
		ddl_vidc_encode_dynamic_property(p_ddl, TRUE);
	}
	enc_param.n_cmd_seq_num = ++p_ddl_context->n_cmd_seq_num;
	enc_param.n_inst_id = p_ddl->n_instance_id;
	enc_param.n_shared_mem_addr_offset = DDL_ADDR_OFFSET(
			p_ddl_context->dram_base_a,
			p_ddl->shared_mem[p_ddl->n_command_channel]);
	enc_param.n_current_y_addr_offset = n_y_addr;
	enc_param.n_current_c_addr_offset = n_c_addr;
	enc_param.n_stream_buffer_addr_offset = DDL_OFFSET(
	p_ddl_context->dram_base_a.p_align_physical_addr, p_stream->p_physical);
	enc_param.n_stream_buffer_size =
		p_encoder->client_output_buf_req.n_size;

	enc_param.b_intra_frame = p_encoder->b_intra_frame_insertion;
	if (p_encoder->b_intra_frame_insertion)
		p_encoder->b_intra_frame_insertion = FALSE;
	enc_param.b_input_flush = FALSE;
	if ((p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4) &&
		(p_encoder->vop_timing.n_vop_time_resolution > 0)) {
		vidc_sm_set_encoder_vop_time(
			&p_ddl->shared_mem[p_ddl->n_command_channel], TRUE,
			p_encoder->vop_timing.n_vop_time_resolution,
			p_ddl->input_frame.n_frm_delta);
	}
	vidc_sm_set_frame_tag(&p_ddl->shared_mem[p_ddl->n_command_channel],
	p_ddl->input_frame.vcd_frm.n_ip_frm_tag);
	if (p_ddl_context->b_pix_cache_enable) {
		for (n_index = 0; n_index < p_enc_buffers->n_dpb_count;
			n_index++) {
			a_dpb_addr_y[n_index] = (u32) p_enc_buffers->dpb_y
				[n_index].p_align_physical_addr;
			a_dpb_addr_c[n_index] = (u32) p_enc_buffers->dpb_c
				[n_index].p_align_physical_addr;
		}
		vidc_pix_cache_init_luma_chroma_base_addr(
			p_enc_buffers->n_dpb_count, a_dpb_addr_y, a_dpb_addr_c);
		vidc_pix_cache_set_frame_range(p_enc_buffers->sz_dpb_y,
			p_enc_buffers->sz_dpb_c);
		vidc_pix_cache_clear_cache_tags();
	}
	p_ddl_context->vidc_encode_frame_start[p_ddl->n_command_channel] (
		&enc_param);
}

u32 ddl_vidc_decode_set_buffers(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	u32 vcd_status = VCD_S_SUCCESS;
	struct vidc_1080p_dec_init_buffers_param_type init_buf_param;

	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPB)) {
		DDL_MSG_ERROR("STATE-CRITICAL");
		return VCD_ERR_FAIL;
	}
	if (p_decoder->dp_buf.n_no_of_dec_pic_buf <
		p_decoder->client_output_buf_req.n_actual_count)
		return VCD_ERR_BAD_STATE;
	if (p_decoder->codec_type.e_codec == VCD_CODEC_H264) {
		vcd_status = ddl_allocate_h264_dec_mv_buffer(p_ddl);
		vidc_sm_set_allocated_h264_mv_size(
			&p_ddl->shared_mem[p_ddl->n_command_channel],
			p_decoder->hw_bufs.h264_mv[0].n_buffer_size);
	}
	if (vcd_status)
		return vcd_status;
#ifdef DDL_BUF_LOG
	ddl_list_buffers(p_ddl);
#endif
	ddl_decoder_dpb_transact(p_decoder, NULL, DDL_DPB_OP_INIT);
	ddl_decoder_dpb_init(p_ddl);
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_DPBDONE",
	ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_DPBDONE;
	p_ddl->e_cmd_state = DDL_CMD_DECODE_SET_DPB;
	vidc_sm_set_allocated_dpb_size(
		&p_ddl->shared_mem[p_ddl->n_command_channel],
		p_decoder->dpb_buf_size.n_size_y,
		p_decoder->dpb_buf_size.n_size_c);
	init_buf_param.n_cmd_seq_num = ++p_ddl_context->n_cmd_seq_num;
	init_buf_param.n_inst_id = p_ddl->n_instance_id;
	init_buf_param.n_shared_mem_addr_offset = DDL_ADDR_OFFSET(
				p_ddl_context->dram_base_a, p_ddl->shared_mem
				[p_ddl->n_command_channel]);
	init_buf_param.n_dpb_count = p_decoder->dp_buf.n_no_of_dec_pic_buf;
	p_ddl_context->vidc_decode_init_buffers[p_ddl->n_command_channel] (
		&init_buf_param);
	return VCD_S_SUCCESS;
}

void ddl_vidc_decode_frame_run(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vcd_frame_data_type *p_bit_stream =
		&(p_ddl->input_frame.vcd_frm);
	struct ddl_dec_buffers_type *p_dec_buffers = &p_decoder->hw_bufs;
	struct ddl_mask_type *p_dpb_mask = &p_ddl->codec_data.decoder.dpb_mask;
	struct vidc_1080p_dec_frame_start_param_type dec_param;
	u32 a_dpb_addr_y[32], n_index;

	if ((!p_bit_stream->n_data_len) || (!p_bit_stream->p_physical)) {
		ddl_vidc_decode_eos_run(p_ddl);
		return;
	}
	DDL_MSG_LOW("ddl_state_transition: %s ~~>\
		DDL_CLIENT_WAIT_FOR_FRAME_DONE",
		ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_FRAME_DONE;
	ddl_vidc_decode_dynamic_property(p_ddl, TRUE);
	ddl_decoder_dpb_transact(p_decoder, NULL, DDL_DPB_OP_SET_MASK);
	p_ddl->e_cmd_state = DDL_CMD_DECODE_FRAME;
	dec_param.n_cmd_seq_num = ++p_ddl_context->n_cmd_seq_num;
	dec_param.n_inst_id = p_ddl->n_instance_id;
	dec_param.n_shared_mem_addr_offset = DDL_ADDR_OFFSET(
				p_ddl_context->dram_base_a, p_ddl->shared_mem
				[p_ddl->n_command_channel]);
	dec_param.n_stream_buffer_addr_offset = DDL_OFFSET(
			p_ddl_context->dram_base_a.p_align_physical_addr,
			p_bit_stream->p_physical);
	dec_param.n_stream_frame_size = p_bit_stream->n_data_len;
	dec_param.n_stream_buffersize = p_decoder->client_input_buf_req.n_size;
	dec_param.n_descriptor_buffer_addr_offset = DDL_ADDR_OFFSET(
	p_ddl_context->dram_base_a, p_dec_buffers->desc);
	dec_param.n_descriptor_buffer_size = p_dec_buffers->desc.n_buffer_size;
	dec_param.n_release_dpb_bit_mask = p_dpb_mask->n_hw_mask;
	dec_param.e_decode_type = VIDC_1080P_DEC_TYPE_FRAME_DATA;
	dec_param.n_dpb_count = p_decoder->dp_buf.n_no_of_dec_pic_buf;
	if (p_decoder->b_flush_pending) {
		dec_param.b_dpb_flush = TRUE;
		p_decoder->b_flush_pending = FALSE;
	} else
		dec_param.b_dpb_flush = FALSE;
	vidc_sm_set_frame_tag(&p_ddl->shared_mem[p_ddl->n_command_channel],
		p_bit_stream->n_ip_frm_tag);
	if (p_ddl_context->b_pix_cache_enable) {
		for (n_index = 0; n_index <
			p_decoder->dp_buf.n_no_of_dec_pic_buf; n_index++) {
			a_dpb_addr_y[n_index] = (u32)
			p_decoder->dp_buf.a_dec_pic_buffers
				[n_index].vcd_frm.p_physical;
		}
		vidc_pix_cache_init_luma_chroma_base_addr(
			p_decoder->dp_buf.n_no_of_dec_pic_buf,
			a_dpb_addr_y, NULL);
		vidc_pix_cache_set_frame_range(p_decoder->dpb_buf_size.n_size_y,
			p_decoder->dpb_buf_size.n_size_c);
		vidc_pix_cache_clear_cache_tags();
	}
	p_ddl_context->vidc_decode_frame_start[p_ddl->n_command_channel] (
		&dec_param);
}

void ddl_vidc_decode_eos_run(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vcd_frame_data_type *p_bit_stream =
		&(p_ddl->input_frame.vcd_frm);
	struct ddl_dec_buffers_type *p_dec_buffers = &(p_decoder->hw_bufs);
	struct ddl_mask_type *p_dpb_mask =
		&(p_ddl->codec_data.decoder.dpb_mask);
	struct vidc_1080p_dec_frame_start_param_type dec_param;

	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_EOS_DONE",
	ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_EOS_DONE;
	ddl_vidc_decode_dynamic_property(p_ddl, TRUE);
	ddl_decoder_dpb_transact(p_decoder, NULL, DDL_DPB_OP_SET_MASK);
	p_decoder->b_dynmic_prop_change_req = TRUE;
	p_ddl->e_cmd_state = DDL_CMD_EOS;
	memset(&dec_param, 0, sizeof(dec_param));
	dec_param.n_cmd_seq_num = ++p_ddl_context->n_cmd_seq_num;
	dec_param.n_inst_id = p_ddl->n_instance_id;
	dec_param.n_shared_mem_addr_offset = DDL_ADDR_OFFSET(
			p_ddl_context->dram_base_a,
			p_ddl->shared_mem[p_ddl->n_command_channel]);
	dec_param.n_descriptor_buffer_addr_offset = DDL_ADDR_OFFSET(
	p_ddl_context->dram_base_a, p_dec_buffers->desc);
	dec_param.n_descriptor_buffer_size = p_dec_buffers->desc.n_buffer_size;
	dec_param.n_release_dpb_bit_mask = p_dpb_mask->n_hw_mask;
	dec_param.e_decode_type = VIDC_1080P_DEC_TYPE_LAST_FRAME_DATA;
	dec_param.n_dpb_count = p_decoder->dp_buf.n_no_of_dec_pic_buf;
	if (p_decoder->b_flush_pending) {
		dec_param.b_dpb_flush = TRUE;
		p_decoder->b_flush_pending = FALSE;
	} else
		dec_param.b_dpb_flush = FALSE;
	vidc_sm_set_frame_tag(&p_ddl->shared_mem[p_ddl->n_command_channel],
	p_bit_stream->n_ip_frm_tag);
	p_ddl_context->vidc_decode_frame_start[p_ddl->n_command_channel] (
		&dec_param);
}

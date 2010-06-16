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
#include <linux/delay.h>

static void ddl_decoder_input_done_callback(
	struct ddl_client_context_type *p_ddl, u32 b_frame_transact_end);
static u32 ddl_decoder_ouput_done_callback(
	struct ddl_client_context_type *p_ddl, u32 b_frame_transact_end);
static u32 ddl_get_decoded_frame_type(struct vcd_frame_data_type  *p_frame,
	enum vidc_1080p_decode_frame_type e_frame_type);
static u32 ddl_get_encoded_frame_type(struct vcd_frame_data_type *p_frame,
	enum vcd_codec_type e_codec,
	enum vidc_1080p_encode_frame_type e_frame_type);
static void ddl_get_dec_profile_level(struct ddl_decoder_data_type *p_decoder,
	u32 n_profile, u32 n_level);

static void ddl_fw_status_done_callback(struct ddl_context_type *p_ddl_context)
{
	DDL_MSG_MED("ddl_fw_status_done_callback");
	if (!DDLCOMMAND_STATE_IS(p_ddl_context, DDL_CMD_DMA_INIT)) {
		DDL_MSG_ERROR("UNKWN_DMADONE");
	} else {
		DDL_MSG_LOW("FW_STATUS_DONE");
		vidc_1080p_set_host2risc_cmd(VIDC_1080P_HOST2RISC_CMD_SYS_INIT,
			p_ddl_context->n_fw_ctxt_memory_size, 0, 0, 0);
	}
}

static void ddl_sys_init_done_callback(struct ddl_context_type *p_ddl_context,
	u32 n_fw_size)
{
	u32 vcd_status = VCD_S_SUCCESS;

	DDL_MSG_MED("ddl_sys_init_done_callback");
	if (!DDLCOMMAND_STATE_IS(p_ddl_context, DDL_CMD_DMA_INIT)) {
		DDL_MSG_ERROR("UNKNOWN_SYS_INIT_DONE");
	} else {
		p_ddl_context->e_cmd_state = DDL_CMD_INVALID;
		DDL_MSG_LOW("SYS_INIT_DONE");
		vidc_1080p_get_fw_version(&p_ddl_context->n_fw_version);
		if (p_ddl_context->n_fw_memory_size >= n_fw_size) {
			p_ddl_context->n_device_state = DDL_DEVICE_INITED;
			vcd_status = VCD_S_SUCCESS;
		} else
			vcd_status = VCD_ERR_FAIL;
		p_ddl_context->ddl_callback(VCD_EVT_RESP_DEVICE_INIT,
			vcd_status, NULL, 0, NULL,
			p_ddl_context->p_client_data);
		DDL_IDLE(p_ddl_context);
	}
}

static void ddl_decoder_eos_done_callback(
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;

	if (!p_ddl->b_decoding) {
		DDL_MSG_ERROR("STATE-CRITICAL-EOSDONE");
		ddl_client_fatal_cb(p_ddl);
	} else {
		p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_FRAME;
		DDL_MSG_LOW("EOS_DONE");
		p_ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE,
			VCD_S_SUCCESS, NULL, 0, (u32 *)p_ddl,
			p_ddl->p_client_data);
		ddl_release_command_channel(p_ddl_context,
			p_ddl->n_command_channel);
	}
}

static u32 ddl_channel_set_callback(struct ddl_context_type *p_ddl_context,
	u32 n_instance_id)
{
	struct ddl_client_context_type *p_ddl;
	u32 b_ret = FALSE;

	DDL_MSG_MED("ddl_channel_open_callback");
	p_ddl = ddl_get_current_ddl_client_for_command(p_ddl_context,
			DDL_CMD_CHANNEL_SET);
	if (p_ddl) {
		p_ddl->e_cmd_state = DDL_CMD_INVALID;
		if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_CHDONE)) {
			DDL_MSG_ERROR("STATE-CRITICAL-CHSET");
			ddl_release_command_channel(p_ddl_context,
			p_ddl->n_command_channel);
		} else {
			DDL_MSG_LOW("CH_SET_DONE");
			DDL_MSG_LOW("ddl_state_transition: %s ~~>\
				DDL_CLIENT_WAIT_FOR_INITCODEC",
				ddl_get_state_string(p_ddl->e_client_state));
			p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_INITCODEC;
			p_ddl->n_channel_id = n_instance_id;
			if (p_ddl->b_decoding) {
				if (p_ddl->codec_data.decoder.b_header_in_start)
					ddl_vidc_decode_init_codec(p_ddl);
				else {
					p_ddl_context->ddl_callback(
						VCD_EVT_RESP_START,
						VCD_S_SUCCESS, NULL, 0,
						(u32 *)p_ddl,
						p_ddl->p_client_data);
					ddl_release_command_channel(
						p_ddl_context,
						p_ddl->n_command_channel);
					b_ret = TRUE;
				}
			} else
				ddl_vidc_encode_init_codec(p_ddl);
		}
	}
	return b_ret;
}

static u32 ddl_encoder_seq_done_callback(struct ddl_context_type *p_ddl_context,
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_encoder_data_type *p_encoder;

	DDL_MSG_MED("ddl_encoder_seq_done_callback");
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODECDONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-INITCODEC");
		ddl_client_fatal_cb(p_ddl);
		return TRUE;
	}
	p_ddl->e_cmd_state = DDL_CMD_INVALID;
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_FRAME",
	ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_FRAME;
	DDL_MSG_LOW("INIT_CODEC_DONE");
	p_encoder = &p_ddl->codec_data.encoder;
	vidc_1080p_get_encoder_sequence_header_size(
		&p_encoder->n_seq_header_length);
	p_ddl_context->ddl_callback(VCD_EVT_RESP_START, VCD_S_SUCCESS,
		NULL, 0, (u32 *) p_ddl, p_ddl->p_client_data);
	ddl_release_command_channel(p_ddl_context,
		p_ddl->n_command_channel);
	return TRUE;
}

static u32 ddl_decoder_seq_done_callback(struct ddl_context_type *p_ddl_context,
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_decoder_data_type *p_decoder = &p_ddl->codec_data.decoder;
	struct vidc_1080p_seq_hdr_info_type seq_hdr_info;
	u32 b_process_further = TRUE;

	DDL_MSG_MED("ddl_decoder_seq_done_callback");
	if (!p_ddl->b_decoding ||
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODECDONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-HDDONE");
		ddl_client_fatal_cb(p_ddl);
	} else {
		p_ddl->e_cmd_state = DDL_CMD_INVALID;
		DDL_MSG_LOW("ddl_state_transition: %s ~~>\
			DDL_CLIENT_WAIT_FOR_DPB",\
			ddl_get_state_string(p_ddl->e_client_state));
		p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_DPB;
		DDL_MSG_LOW("HEADER_DONE");
		vidc_1080p_get_decode_seq_start_result(&seq_hdr_info);
		p_decoder->frame_size.n_width = seq_hdr_info.n_img_size_x;
		p_decoder->frame_size.n_height = seq_hdr_info.n_img_size_y;
		p_decoder->n_min_dpb_num = seq_hdr_info.n_min_num_dpb;
		vidc_sm_get_min_yc_dpb_sizes(
			&p_ddl->shared_mem[p_ddl->n_command_channel],
			&seq_hdr_info.n_min_luma_dpb_size,
			&seq_hdr_info.n_min_chroma_dpb_size);
		p_decoder->n_y_cb_cr_size = seq_hdr_info.n_min_luma_dpb_size +
			seq_hdr_info.n_min_chroma_dpb_size;
		p_decoder->dpb_buf_size.n_size_yuv = p_decoder->n_y_cb_cr_size;
		p_decoder->dpb_buf_size.n_size_y =
			seq_hdr_info.n_min_luma_dpb_size;
		p_decoder->dpb_buf_size.n_size_c =
			seq_hdr_info.n_min_chroma_dpb_size;
		p_decoder->n_progressive_only = 1 - seq_hdr_info.n_progressive;
		if (!seq_hdr_info.n_img_size_x || !seq_hdr_info.n_img_size_y) {
			DDL_MSG_ERROR("FATAL:ZeroImageSize");
			ddl_client_fatal_cb(p_ddl);
			return b_process_further;
		}
		vidc_sm_get_profile_info(&p_ddl->shared_mem
			[p_ddl->n_command_channel],
			&seq_hdr_info.n_profile, &seq_hdr_info.n_level);
		ddl_get_dec_profile_level(p_decoder, seq_hdr_info.n_profile,
			seq_hdr_info.n_level);
		ddl_calculate_stride(&p_decoder->frame_size,
			!p_decoder->n_progressive_only);
		vidc_sm_get_crop_info(
			&p_ddl->shared_mem[p_ddl->n_command_channel],
			&seq_hdr_info.n_crop_left_offset,
			&seq_hdr_info.n_crop_right_offset,
			&seq_hdr_info.n_crop_top_offset,
			&seq_hdr_info.n_crop_bottom_offset);
		seq_hdr_info.n_crop_exists = (seq_hdr_info.n_crop_left_offset ||
			seq_hdr_info.n_crop_right_offset ||
			seq_hdr_info.n_crop_top_offset ||
			seq_hdr_info.n_crop_bottom_offset);
		if (seq_hdr_info.n_crop_exists) {
			p_decoder->frame_size.n_width -=
				seq_hdr_info.n_crop_right_offset +
				seq_hdr_info.n_crop_left_offset;
			p_decoder->frame_size.n_height -=
				seq_hdr_info.n_crop_top_offset +
				seq_hdr_info.n_crop_bottom_offset;
		}
		ddl_set_default_decoder_buffer_req(p_decoder, FALSE);
		if (p_decoder->b_header_in_start) {
			p_decoder->client_frame_size = p_decoder->frame_size;
			p_decoder->client_output_buf_req =
				p_decoder->actual_output_buf_req;
			if ((p_decoder->frame_size.n_width *
				p_decoder->frame_size.n_height) >=
				 VCD_DDL_WVGA_BUF_SIZE) {
				if ((p_decoder->actual_output_buf_req.\
					n_actual_count + 2) < 10)
					p_decoder->client_output_buf_req.\
						n_actual_count = 10;
				else
					p_decoder->client_output_buf_req.\
						n_actual_count += 2;
			} else
				p_decoder->client_output_buf_req.\
					n_actual_count = p_decoder->\
					actual_output_buf_req.\
					n_actual_count + 5;
			p_decoder->client_input_buf_req =
				p_decoder->actual_input_buf_req;
			p_ddl_context->ddl_callback(VCD_EVT_RESP_START,
				VCD_S_SUCCESS, NULL, 0, (u32 *) p_ddl,
				p_ddl->p_client_data);
			ddl_release_command_channel(p_ddl_context,
				p_ddl->n_command_channel);
		} else {
			u32 b_seq_hdr_only_frame = FALSE;
			u32 b_need_reconfig = TRUE;
			struct vcd_frame_data_type *p_input_vcd_frm =
				&p_ddl->input_frame.vcd_frm;

			if ((p_decoder->frame_size.n_width <=
				p_decoder->client_frame_size.n_stride) &&
				(p_decoder->frame_size.n_height <=
				p_decoder->client_frame_size.n_scan_lines) &&
				(p_decoder->actual_output_buf_req.n_size <=
				p_decoder->client_output_buf_req.n_size) &&
				(p_decoder->n_min_dpb_num <= p_decoder->\
				client_output_buf_req.n_actual_count))
				b_need_reconfig = FALSE;
			if (((p_input_vcd_frm->n_flags &
				VCD_FRAME_FLAG_CODECCONFIG) &&
				(!(p_input_vcd_frm->n_flags &
				VCD_FRAME_FLAG_SYNCFRAME))) ||
				p_input_vcd_frm->n_data_len ==
				seq_hdr_info.n_dec_frm_size) {
				b_seq_hdr_only_frame = TRUE;
				p_input_vcd_frm->n_offset +=
					seq_hdr_info.n_dec_frm_size;
				p_input_vcd_frm->n_data_len -=
					seq_hdr_info.n_dec_frm_size;
				p_input_vcd_frm->n_flags |=
					VCD_FRAME_FLAG_CODECCONFIG;
				p_ddl->input_frame.b_frm_trans_end =
					!b_need_reconfig;
				p_ddl_context->ddl_callback(
					VCD_EVT_RESP_INPUT_DONE,
					VCD_S_SUCCESS, &p_ddl->input_frame,
					sizeof(struct ddl_frame_data_type_tag),
					(u32 *) p_ddl, p_ddl->p_client_data);
			}
			if (b_need_reconfig) {
				struct ddl_frame_data_type_tag *p_payload =
					&p_ddl->input_frame;
				u32 n_payload_size =
					sizeof(struct ddl_frame_data_type_tag);

				p_decoder->client_frame_size =
					p_decoder->frame_size;
				p_decoder->client_output_buf_req =
					p_decoder->actual_output_buf_req;
				p_decoder->client_input_buf_req =
					p_decoder->actual_input_buf_req;
				if (b_seq_hdr_only_frame) {
					p_payload = NULL;
					n_payload_size = 0;
				}
				p_ddl_context->ddl_callback(
					VCD_EVT_IND_OUTPUT_RECONFIG,
					VCD_S_SUCCESS, p_payload,
					n_payload_size, (u32 *) p_ddl,
					p_ddl->p_client_data);
			}
			if (!b_need_reconfig && !b_seq_hdr_only_frame) {
				if (!ddl_vidc_decode_set_buffers(p_ddl))
					b_process_further = FALSE;
				else {
					DDL_MSG_ERROR("ddl_vidc_decode_set_\
						buffers failed");
					ddl_client_fatal_cb(p_ddl);
				}
			} else
				ddl_release_command_channel(p_ddl_context,
					p_ddl->n_command_channel);
		}
	}
	return b_process_further;
}

static u32 ddl_sequence_done_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl;
	u32 n_channel_inst_id, b_ret;

	vidc_1080p_get_returned_channel_inst_id(&n_channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	p_ddl = ddl_get_current_ddl_client_for_channel_id(p_ddl_context,
			p_ddl_context->n_response_cmd_ch_id);
	if (!p_ddl) {
		DDL_MSG_ERROR("UNKWN_SEQ_DONE");
		b_ret = TRUE;
	} else {
		if (p_ddl->b_decoding)
			b_ret = ddl_decoder_seq_done_callback(p_ddl_context,
					p_ddl);
		else
			b_ret = ddl_encoder_seq_done_callback(p_ddl_context,
					p_ddl);
	}
	return b_ret;
}

static u32 ddl_dpb_buffers_set_done_callback(
	struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl;
	u32 n_channel_inst_id, b_ret_status = TRUE;

	DDL_MSG_MED("ddl_dpb_buffers_set_done_callback");
	vidc_1080p_get_returned_channel_inst_id(&n_channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	p_ddl = ddl_get_current_ddl_client_for_command(p_ddl_context,
			DDL_CMD_DECODE_SET_DPB);
	if (p_ddl) {
		p_ddl->e_cmd_state = DDL_CMD_INVALID;
		if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPBDONE)) {
			DDL_MSG_ERROR("STATE-CRITICAL-DPBDONE");
			ddl_client_fatal_cb(p_ddl);
		} else {
			DDL_MSG_LOW("INTR_DPBDONE");
			DDL_MSG_LOW("ddl_state_transition: %s ~~>\
				DDL_CLIENT_WAIT_FOR_FRAME",\
				ddl_get_state_string(p_ddl->e_client_state));
			p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_FRAME;
			p_ddl->codec_data.decoder.dec_disp_info.\
				n_img_size_x = 0;
			p_ddl->codec_data.decoder.dec_disp_info.\
				n_img_size_y = 0;
			ddl_vidc_decode_frame_run(p_ddl);
			b_ret_status = FALSE;
		}
	}
	return b_ret_status;
}

static void ddl_encoder_frame_run_callback(
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_encoder_data_type *p_encoder =
		&(p_ddl->codec_data.encoder);
	struct vcd_frame_data_type *p_output_frame =
		&(p_ddl->output_frame.vcd_frm);
	u32 n_bottom_frame_tag;

	DDL_MSG_MED("ddl_encoder_frame_run_callback");
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-ENCFRMRUN");
		ddl_client_fatal_cb(p_ddl);
	} else {
		DDL_MSG_LOW("ENC_FRM_RUN_DONE");
		p_ddl->e_cmd_state = DDL_CMD_INVALID;
		vidc_1080p_get_encode_frame_info(&p_encoder->enc_frame_info);
		vidc_sm_get_frame_tags(&p_ddl->shared_mem
			[p_ddl->n_command_channel],
			&p_output_frame->n_ip_frm_tag, &n_bottom_frame_tag);
		if (p_encoder->enc_frame_info.n_enc_frame_size ||
			(p_encoder->enc_frame_info.e_enc_frame_type ==
			VIDC_1080P_ENCODE_FRAMETYPE_SKIPPED) ||
			DDLCLIENT_STATE_IS(p_ddl,
			DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
			u8 *p_input_buffer_address = NULL;
#ifdef DDL_PROFILE
			ddl_calc_core_time(1);
#endif
			p_output_frame->n_data_len =
				p_encoder->enc_frame_info.n_enc_frame_size;
			p_output_frame->n_flags |= VCD_FRAME_FLAG_ENDOFFRAME;
			ddl_get_encoded_frame_type(p_output_frame,
				p_encoder->codec_type.e_codec,
				p_encoder->enc_frame_info.e_enc_frame_type);
			ddl_vidc_encode_dynamic_property(p_ddl, FALSE);
			p_ddl->input_frame.b_frm_trans_end = FALSE;
			p_input_buffer_address = p_ddl_context->dram_base_a.\
				p_align_virtual_addr +
				p_encoder->enc_frame_info.n_enc_luma_address;
			ddl_get_input_frame_from_pool(p_ddl,
				p_input_buffer_address);
			p_ddl_context->ddl_callback(VCD_EVT_RESP_INPUT_DONE,
				VCD_S_SUCCESS, &(p_ddl->input_frame),
				sizeof(struct ddl_frame_data_type_tag),
				(u32 *)p_ddl, p_ddl->p_client_data);
			p_ddl->output_frame.b_frm_trans_end =
				DDLCLIENT_STATE_IS(p_ddl,
				DDL_CLIENT_WAIT_FOR_EOS_DONE) ? FALSE : TRUE;
			p_ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE,
				VCD_S_SUCCESS, &(p_ddl->output_frame),
				sizeof(struct ddl_frame_data_type_tag),
				(u32 *)p_ddl, p_ddl->p_client_data);
			if (DDLCLIENT_STATE_IS(p_ddl,
				DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
				struct vidc_1080p_enc_frame_start_param_type
					enc_param;

				if (p_encoder->i_period.n_b_frames) {
					if (p_ddl->n_extra_output_buf_count >=
						0) {
						p_ddl->output_frame = p_ddl->\
						extra_output_frame[p_ddl->\
						n_extra_output_buf_count];
						p_ddl->\
						n_extra_output_buf_count--;
						p_output_frame =
							&p_ddl->output_frame.\
							vcd_frm;
					} else
						p_output_frame = NULL;
				} else
					p_output_frame = NULL;

				memset(&enc_param, 0, sizeof(enc_param));
				enc_param.n_cmd_seq_num =
					++p_ddl_context->n_cmd_seq_num;
				enc_param.n_inst_id = p_ddl->n_instance_id;
				enc_param.n_shared_mem_addr_offset =
					DDL_ADDR_OFFSET(p_ddl_context->\
						dram_base_a, p_ddl->shared_mem
						[p_ddl->n_command_channel]);
				if (p_output_frame) {
					enc_param.n_stream_buffer_addr_offset =
						DDL_OFFSET(p_ddl_context->\
						dram_base_a.\
						p_align_physical_addr,
						p_output_frame->p_physical);
				}
				enc_param.n_stream_buffer_size =
					p_encoder->client_output_buf_req.n_size;
				enc_param.e_encode_type =
					VIDC_1080P_ENC_TYPE_LAST_FRAME_DATA;
				p_ddl->e_cmd_state = DDL_CMD_ENCODE_FRAME;
				p_ddl_context->vidc_encode_frame_start
				[p_ddl->n_command_channel]
					(&enc_param);
			} else {
				DDL_MSG_LOW("ddl_state_transition: %s ~~> \
					DDL_CLIENT_WAIT_FOR_FRAME",
					ddl_get_state_string(
					p_ddl->e_client_state));
				p_ddl->e_client_state =
					DDL_CLIENT_WAIT_FOR_FRAME;
				ddl_release_command_channel(p_ddl_context,
				p_ddl->n_command_channel);
			}
		} else {
			p_ddl_context->ddl_callback(
				VCD_EVT_RESP_TRANSACTION_PENDING,
				VCD_S_SUCCESS, NULL, 0, (u32 *)p_ddl,
				p_ddl->p_client_data);
			DDL_MSG_LOW("ddl_state_transition: %s ~~> \
				DDL_CLIENT_WAIT_FOR_FRAME",
			ddl_get_state_string(p_ddl->e_client_state));
			p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_FRAME;
			ddl_release_command_channel(p_ddl_context,
			p_ddl->n_command_channel);
		}
	}
}

static u32 ddl_decoder_frame_run_callback(
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct vidc_1080p_dec_disp_info_type *p_dec_disp_info =
		&p_ddl->codec_data.decoder.dec_disp_info;
	u32 b_callback_end = FALSE, b_ret_status = TRUE, b_eos_present = FALSE;

	DDL_MSG_MED("ddl_decoder_frame_run_callback");
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-DECFRMRUN");
		ddl_client_fatal_cb(p_ddl);
	} else {
		DDL_MSG_LOW("DEC_FRM_RUN_DONE");
		p_ddl->e_cmd_state = DDL_CMD_INVALID;
		vidc_1080p_get_display_frame_result(p_dec_disp_info);
		ddl_vidc_decode_dynamic_property(p_ddl, FALSE);
		if (p_dec_disp_info->n_resl_change) {
			DDL_MSG_ERROR("DEC_RECONFIG_NOT_SUPPORTED");
			ddl_client_fatal_cb(p_ddl);
		} else {
			if ((VCD_FRAME_FLAG_EOS &
				p_ddl->input_frame.vcd_frm.n_flags)) {
				b_callback_end = FALSE;
				b_eos_present = TRUE;
			}
			if (p_dec_disp_info->e_display_status ==
				VIDC_1080P_DISPLAY_STATUS_DECODE_ONLY ||
				p_dec_disp_info->e_display_status ==
				VIDC_1080P_DISPLAY_STATUS_DECODE_AND_DISPLAY) {
				if (!b_eos_present)
					b_callback_end =
					(p_dec_disp_info->e_display_status ==
					VIDC_1080P_DISPLAY_STATUS_DECODE_ONLY);
				ddl_decoder_input_done_callback(p_ddl,
					b_callback_end);
			}
			if (p_dec_disp_info->e_display_status ==
				VIDC_1080P_DISPLAY_STATUS_DECODE_AND_DISPLAY ||
				p_dec_disp_info->e_display_status ==
				VIDC_1080P_DISPLAY_STATUS_DISPLAY_ONLY) {
				u32 vcd_status;
				if (!b_eos_present)
					b_callback_end = (p_dec_disp_info->\
					e_display_status ==
				VIDC_1080P_DISPLAY_STATUS_DECODE_AND_DISPLAY);

				vcd_status = ddl_decoder_ouput_done_callback(
					p_ddl, b_callback_end);
				if (vcd_status)
					return TRUE;
			}
			if (p_dec_disp_info->e_display_status ==
				VIDC_1080P_DISPLAY_STATUS_DISPLAY_ONLY) {
				ddl_vidc_decode_frame_run(p_ddl);
				b_ret_status = FALSE;
			} else if (b_eos_present) {
				ddl_vidc_decode_eos_run(p_ddl);
				b_ret_status = FALSE;
			} else {
				p_ddl->e_client_state =
					DDL_CLIENT_WAIT_FOR_FRAME;
				ddl_release_command_channel(p_ddl_context,
					p_ddl->n_command_channel);
			}
		}
	}
	return b_ret_status;
}

static u32 ddl_eos_frame_done_callback(
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_decoder_data_type *p_decoder = &p_ddl->codec_data.decoder;
	struct vidc_1080p_dec_disp_info_type *p_dec_disp_info =
		&p_decoder->dec_disp_info;
	struct ddl_mask_type *p_dpb_mask = &p_decoder->dpb_mask;
	u32 b_ret_status = TRUE;

	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-EOSFRMRUN");
		ddl_client_fatal_cb(p_ddl);
	} else {
		DDL_MSG_LOW("EOS_FRM_RUN_DONE");
		p_ddl->e_cmd_state = DDL_CMD_INVALID;
		vidc_1080p_get_display_frame_result(p_dec_disp_info);
		ddl_vidc_decode_dynamic_property(p_ddl, FALSE);
		if (p_dec_disp_info->e_display_status ==
			VIDC_1080P_DISPLAY_STATUS_DPB_EMPTY) {
			ddl_decoder_eos_done_callback(p_ddl);
		} else {
			struct vidc_1080p_dec_frame_start_param_type dec_param;
			if (p_dec_disp_info->e_display_status ==
				VIDC_1080P_DISPLAY_STATUS_DISPLAY_ONLY) {
				u32 vcd_status;

				vcd_status = ddl_decoder_ouput_done_callback(
					p_ddl, FALSE);
				if (vcd_status)
					return TRUE;
			} else
				DDL_MSG_ERROR("EOS-STATE-CRITICAL-\
					WRONG-DISP-STATUS");

			ddl_decoder_dpb_transact(p_decoder, NULL,
				DDL_DPB_OP_SET_MASK);
			p_ddl->e_cmd_state = DDL_CMD_EOS;

			memset(&dec_param, 0, sizeof(dec_param));

			dec_param.n_cmd_seq_num =
				++p_ddl_context->n_cmd_seq_num;
			dec_param.n_inst_id = p_ddl->n_instance_id;
			dec_param.n_shared_mem_addr_offset =
				DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_ddl->shared_mem[p_ddl->n_command_channel]);
			dec_param.n_release_dpb_bit_mask =
				p_dpb_mask->n_hw_mask;
			dec_param.e_decode_type =
				VIDC_1080P_DEC_TYPE_LAST_FRAME_DATA;

			p_ddl_context->vidc_decode_frame_start[p_ddl->\
				n_command_channel](&dec_param);
			b_ret_status = FALSE;
		}
	}
	return b_ret_status;
}

static u32 ddl_frame_run_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl;
	u32 n_channel_inst_id;
	u32 b_return_status = TRUE;

	vidc_1080p_get_returned_channel_inst_id(&n_channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	p_ddl = ddl_get_current_ddl_client_for_channel_id(p_ddl_context,
			p_ddl_context->n_response_cmd_ch_id);
	if (p_ddl) {
		if (p_ddl->e_cmd_state == DDL_CMD_DECODE_FRAME)
			b_return_status = ddl_decoder_frame_run_callback(p_ddl);
		else if (p_ddl->e_cmd_state == DDL_CMD_ENCODE_FRAME)
			ddl_encoder_frame_run_callback(p_ddl);
		else if (p_ddl->e_cmd_state == DDL_CMD_EOS)
			b_return_status = ddl_eos_frame_done_callback(p_ddl);
		else {
			DDL_MSG_ERROR("UNKWN_FRAME_DONE");
			b_return_status = FALSE;
		}
	} else
		b_return_status = FALSE;

	return b_return_status;
}

static void ddl_channel_end_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl;

	DDL_MSG_MED("ddl_channel_end_callback");
	p_ddl = ddl_get_current_ddl_client_for_command(p_ddl_context,
			DDL_CMD_CHANNEL_END);
	if (p_ddl) {
		p_ddl->e_cmd_state = DDL_CMD_INVALID;
		if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_CHEND)) {
			DDL_MSG_LOW("STATE-CRITICAL-CHEND");
		} else {
			DDL_MSG_LOW("CH_END_DONE");
			ddl_release_client_internal_buffers(p_ddl);
			p_ddl_context->ddl_callback(VCD_EVT_RESP_STOP,
				VCD_S_SUCCESS, NULL, 0, (u32 *)p_ddl,
				p_ddl->p_client_data);
			DDL_MSG_LOW("ddl_state_transition: %s ~~>\
				DDL_CLIENT_OPEN",\
				ddl_get_state_string(p_ddl->e_client_state));
			p_ddl->e_client_state = DDL_CLIENT_OPEN;
		}
		ddl_release_command_channel(p_ddl_context,
			p_ddl->n_command_channel);
	}
}

static void ddl_edfu_callback(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl;
	u32 n_channel_inst_id;

	DDL_MSG_MED("ddl_edfu_callback");
	vidc_1080p_get_returned_channel_inst_id(&n_channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	p_ddl = ddl_get_current_ddl_client_for_channel_id(p_ddl_context,
			p_ddl_context->n_response_cmd_ch_id);
	if (p_ddl) {
		if (p_ddl->e_cmd_state != DDL_CMD_ENCODE_FRAME)
			DDL_MSG_LOW("UNKWN_EDFU");
	}
}

static void ddl_encoder_eos_done(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl;
	u32 n_channel_inst_id;

	DDL_MSG_LOW("encoder_eos_done");
	vidc_1080p_get_returned_channel_inst_id(&n_channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	p_ddl = ddl_get_current_ddl_client_for_channel_id(p_ddl_context,
			p_ddl_context->n_response_cmd_ch_id);
	if (p_ddl) {
		if (DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
			DDL_MSG_LOW("ENC_EOS_DONE");
			p_ddl->e_cmd_state = DDL_CMD_INVALID;
			DDL_MSG_LOW("ddl_state_transition: %s ~~> \
				DDL_CLIENT_WAIT_FOR_FRAME",
				ddl_get_state_string(p_ddl->e_client_state));
			p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_FRAME;
			DDL_MSG_LOW("EOS_DONE");
			p_ddl->output_frame.b_frm_trans_end = TRUE;
			p_ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE,
				VCD_S_SUCCESS, &(p_ddl->output_frame),
				sizeof(struct ddl_frame_data_type_tag),
				(u32 *)p_ddl, p_ddl->p_client_data);
		} else {
			DDL_MSG_LOW("STATE-CRITICAL-EOSDONE");
		}
		ddl_release_command_channel(p_ddl_context,
			p_ddl->n_command_channel);
	} else {
		DDL_MSG_LOW("Invalid Client DDL context");
	}
}

static u32 ddl_process_intr_status(struct ddl_context_type *p_ddl_context,
	u32 intr_status)
{
	u32 b_return_status = TRUE;
	switch (intr_status) {
	case VIDC_1080P_RISC2HOST_CMD_OPEN_CH_RET:
		b_return_status = ddl_channel_set_callback(p_ddl_context,
			p_ddl_context->n_response_cmd_ch_id);
	break;
	case VIDC_1080P_RISC2HOST_CMD_CLOSE_CH_RET:
		ddl_channel_end_callback(p_ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_SEQ_DONE_RET:
		b_return_status = ddl_sequence_done_callback(p_ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_FRAME_DONE_RET:
		b_return_status = ddl_frame_run_callback(p_ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_SYS_INIT_RET:
		ddl_sys_init_done_callback(p_ddl_context,
			p_ddl_context->n_response_cmd_ch_id);
	break;
	case VIDC_1080P_RISC2HOST_CMD_FW_STATUS_RET:
		ddl_fw_status_done_callback(p_ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_EDFU_INT_RET:
		ddl_edfu_callback(p_ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_ENC_COMPLETE_RET:
		ddl_encoder_eos_done(p_ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_ERROR_RET:
		DDL_MSG_ERROR("OP_FAILED_INTR");
		b_return_status = ddl_handle_core_errors(p_ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_INIT_BUFFERS_RET:
		b_return_status =
			ddl_dpb_buffers_set_done_callback(p_ddl_context);
	break;
	default:
		DDL_MSG_LOW("UNKWN_INTR");
	break;
	}
	return b_return_status;
}

void ddl_read_and_clear_interrupt(void)
{
	struct ddl_context_type *p_ddl_context;
	struct ddl_hw_interface_type  *ddl_hw_response;

	p_ddl_context = ddl_get_context();
	if (!p_ddl_context->p_core_virtual_base_addr) {
		DDL_MSG_LOW("SPURIOUS_INTERRUPT");
	} else {
		ddl_hw_response = &p_ddl_context->ddl_hw_response;
		vidc_1080p_get_risc2host_cmd(&ddl_hw_response->cmd,
			&ddl_hw_response->arg1, &ddl_hw_response->arg2,
			&ddl_hw_response->arg3, &ddl_hw_response->arg4);
		vidc_1080p_clear_risc2host_cmd();
		vidc_1080p_clear_interrupt();
		p_ddl_context->n_cmd_err_status =
			ddl_hw_response->arg2 & 0xffff;
		p_ddl_context->n_disp_pic_err_status =
			(ddl_hw_response->arg2 & 0xffff0000) >> 16;
		p_ddl_context->n_response_cmd_ch_id = ddl_hw_response->arg1;
	}
}

u32 ddl_process_core_response(void)
{
	struct ddl_context_type *p_ddl_context;
	struct ddl_hw_interface_type *ddl_hw_response;
	u32 b_return_status = FALSE;

	p_ddl_context = ddl_get_context();
	if (!p_ddl_context->p_core_virtual_base_addr) {
		DDL_MSG_LOW("SPURIOUS_INTERRUPT");
	} else {
		ddl_hw_response = &p_ddl_context->ddl_hw_response;
		if (ddl_hw_response->cmd == DDL_INVALID_INTR_STATUS) {
			DDL_MSG_ERROR("INTERRUPT_NOT_READ");
		} else {
			b_return_status = ddl_process_intr_status(p_ddl_context,
				ddl_hw_response->cmd);
			if (p_ddl_context->pf_interrupt_clr)
				(*p_ddl_context->pf_interrupt_clr)();
			ddl_hw_response->cmd = DDL_INVALID_INTR_STATUS;
		}
	}
	return b_return_status;
}

static void ddl_decoder_input_done_callback(
	struct ddl_client_context_type *p_ddl, u32 b_frame_transact_end)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vidc_1080p_dec_disp_info_type *p_dec_disp_info =
		&p_decoder->dec_disp_info;
	struct vcd_frame_data_type *p_input_vcd_frm =
		&(p_ddl->input_frame.vcd_frm);

	vidc_1080p_get_decoded_frame_size(
		&p_dec_disp_info->n_input_bytes_consumed);
	vidc_1080p_get_decode_frame_type(&p_dec_disp_info->e_input_frame_type);
	ddl_get_decoded_frame_type(p_input_vcd_frm,
		p_dec_disp_info->e_input_frame_type);
	vidc_1080p_get_decode_frame_result(p_dec_disp_info);
	p_input_vcd_frm->b_interlaced = (p_dec_disp_info->e_display_coding !=
		VIDC_1080P_DISPLAY_CODING_PROGRESSIVE_SCAN);
	p_input_vcd_frm->n_offset += p_dec_disp_info->n_input_bytes_consumed;
	p_input_vcd_frm->n_data_len -= p_dec_disp_info->n_input_bytes_consumed;
	p_ddl->input_frame.b_frm_trans_end = b_frame_transact_end;
	p_ddl_context->ddl_callback(VCD_EVT_RESP_INPUT_DONE, VCD_S_SUCCESS,
		&p_ddl->input_frame, sizeof(struct ddl_frame_data_type_tag),
		(u32 *)p_ddl, p_ddl->p_client_data);
}

static u32 ddl_decoder_ouput_done_callback(
	struct ddl_client_context_type *p_ddl, u32 b_frame_transact_end)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_decoder_data_type *p_decoder = &(p_ddl->codec_data.decoder);
	struct vidc_1080p_dec_disp_info_type *p_dec_disp_info =
		&(p_decoder->dec_disp_info);
	struct ddl_frame_data_type_tag *p_output_frame = &(p_ddl->output_frame);
	struct vcd_frame_data_type *p_output_vcd_frm =
		&(p_output_frame->vcd_frm);
	u32 vcd_status, n_free_luma_dpb = 0, n_disp_pict_type = 0;

	p_output_vcd_frm->p_physical =
		(u8 *) (p_dec_disp_info->n_display_y_addr << 11);

	vidc_sm_get_displayed_picture_frame_type(&p_ddl->shared_mem
		[p_ddl->n_command_channel], &n_disp_pict_type);
	if (!n_disp_pict_type)
		p_output_vcd_frm->e_frame_type = VCD_FRAME_NOTCODED;
	else
		p_output_vcd_frm->e_frame_type = VCD_FRAME_YUV;

	if (p_decoder->codec_type.e_codec == VCD_CODEC_MPEG4 ||
		p_decoder->codec_type.e_codec == VCD_CODEC_VC1 ||
		p_decoder->codec_type.e_codec == VCD_CODEC_VC1_RCV ||
		(p_decoder->codec_type.e_codec >= VCD_CODEC_DIVX_3 &&
		p_decoder->codec_type.e_codec <= VCD_CODEC_XVID)) {
		if (p_output_vcd_frm->e_frame_type == VCD_FRAME_NOTCODED) {
			vidc_sm_get_available_luma_dpb_address(
				&p_ddl->shared_mem[p_ddl->n_command_channel],
				&n_free_luma_dpb);
			if (n_free_luma_dpb)
				p_output_vcd_frm->p_physical =
					(u8 *)(n_free_luma_dpb << 11);
		}
	}
	vcd_status = ddl_decoder_dpb_transact(p_decoder, p_output_frame,
			DDL_DPB_OP_MARK_BUSY);
	if (vcd_status) {
		DDL_MSG_ERROR("CORRUPTED_OUTPUT_BUFFER_ADDRESS");
		ddl_hw_fatal_cb(p_ddl);
	} else {
		vidc_sm_get_frame_tags(&p_ddl->shared_mem
			[p_ddl->n_command_channel],
			&p_dec_disp_info->n_tag_top,
			&p_dec_disp_info->n_tag_bottom);
		p_output_vcd_frm->n_ip_frm_tag = p_dec_disp_info->n_tag_top;

		vidc_sm_get_picture_times(&p_ddl->shared_mem
			[p_ddl->n_command_channel],
			&p_dec_disp_info->n_pic_time_top,
			&p_dec_disp_info->n_pic_time_bottom);

		vidc_sm_get_crop_info(&p_ddl->shared_mem
			[p_ddl->n_command_channel],
			&p_dec_disp_info->n_crop_left_offset,
			&p_dec_disp_info->n_crop_right_offset,
			&p_dec_disp_info->n_crop_top_offset,
			&p_dec_disp_info->n_crop_bottom_offset);

		if (p_dec_disp_info->n_crop_left_offset ||
			p_dec_disp_info->n_crop_right_offset ||
			p_dec_disp_info->n_crop_top_offset ||
			p_dec_disp_info->n_crop_bottom_offset)
			p_dec_disp_info->n_crop_exists = TRUE;
		else
			p_dec_disp_info->n_crop_exists = FALSE;

		if (p_dec_disp_info->n_crop_exists) {
			p_output_vcd_frm->dec_op_prop.disp_frm.n_left =
				p_dec_disp_info->n_crop_left_offset;
			p_output_vcd_frm->dec_op_prop.disp_frm.n_top =
				p_dec_disp_info->n_crop_top_offset;
			p_output_vcd_frm->dec_op_prop.disp_frm.n_right =
				p_decoder->frame_size.n_width -
				p_dec_disp_info->n_crop_right_offset;
			p_output_vcd_frm->dec_op_prop.disp_frm.n_bottom =
				p_decoder->frame_size.n_height -
				p_dec_disp_info->n_crop_bottom_offset;
		} else {
			p_output_vcd_frm->dec_op_prop.disp_frm.n_left = 0;
			p_output_vcd_frm->dec_op_prop.disp_frm.n_top = 0;
			p_output_vcd_frm->dec_op_prop.disp_frm.n_right =
				p_decoder->frame_size.n_width;
			p_output_vcd_frm->dec_op_prop.disp_frm.n_bottom =
				p_decoder->frame_size.n_height;
		}
		if (p_dec_disp_info->e_display_coding ==
			VIDC_1080P_DISPLAY_CODING_PROGRESSIVE_SCAN) {
			p_output_vcd_frm->b_interlaced = FALSE;
			p_output_vcd_frm->n_intrlcd_ip_frm_tag =
				VCD_FRAMETAG_INVALID;
		} else {
			p_output_vcd_frm->b_interlaced = TRUE;
			if (!p_dec_disp_info->n_tag_bottom)
				p_output_vcd_frm->n_intrlcd_ip_frm_tag =
					VCD_FRAMETAG_INVALID;
			else
				p_output_vcd_frm->n_intrlcd_ip_frm_tag =
					p_dec_disp_info->n_tag_bottom;
		}
		p_output_vcd_frm->n_offset = 0;
		p_output_vcd_frm->n_data_len = p_decoder->n_y_cb_cr_size;
		if (n_free_luma_dpb) {
			p_output_vcd_frm->n_data_len = 0;
			p_output_vcd_frm->n_flags |= VCD_FRAME_FLAG_DECODEONLY;
		}
		p_output_vcd_frm->n_flags |= VCD_FRAME_FLAG_ENDOFFRAME;
		p_output_frame->b_frm_trans_end = b_frame_transact_end;
#ifdef DDL_PROFILE
		ddl_calc_core_time(0);
#endif
		p_ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE,
			vcd_status, p_output_frame,
			sizeof(struct ddl_frame_data_type_tag),
			(u32 *)p_ddl, p_ddl->p_client_data);
	}
	return vcd_status;
}

static u32 ddl_get_decoded_frame_type(struct vcd_frame_data_type  *p_frame,
	enum vidc_1080p_decode_frame_type e_frame_type)
{
	u32 b_status = TRUE;

	switch (e_frame_type) {
	case VIDC_1080P_DECODE_FRAMETYPE_I:
		p_frame->n_flags |= VCD_FRAME_FLAG_SYNCFRAME;
		p_frame->e_frame_type = VCD_FRAME_I;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_P:
		p_frame->e_frame_type = VCD_FRAME_P;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_B:
		p_frame->e_frame_type = VCD_FRAME_B;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_NOT_CODED:
		p_frame->e_frame_type = VCD_FRAME_NOTCODED;
		p_frame->n_data_len = 0;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_OTHERS:
		p_frame->e_frame_type = VCD_FRAME_YUV;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_32BIT:
	default:
		DDL_MSG_ERROR("UNKNOWN-FRAMETYPE");
		b_status = FALSE;
	break;
	}
	return b_status;
}

static u32 ddl_get_encoded_frame_type(struct vcd_frame_data_type *p_frame,
	enum vcd_codec_type e_codec,
	enum vidc_1080p_encode_frame_type e_frame_type)
{
	u32 b_status = TRUE;

	if (e_codec == VCD_CODEC_H264) {
		switch (e_frame_type) {
		case VIDC_1080P_ENCODE_FRAMETYPE_NOT_CODED:
			p_frame->e_frame_type = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_I:
			p_frame->n_flags |= VCD_FRAME_FLAG_SYNCFRAME;
			p_frame->e_frame_type = VCD_FRAME_I;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_P:
			p_frame->e_frame_type = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_B:
			p_frame->e_frame_type = VCD_FRAME_B;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_SKIPPED:
			p_frame->e_frame_type = VCD_FRAME_NOTCODED;
			p_frame->n_data_len = 0;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_OTHERS:
			DDL_MSG_LOW("FRAMETYPE-OTHERS");
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_32BIT:
		default:
			DDL_MSG_LOW("UNKNOWN-FRAMETYPE");
			b_status = FALSE;
		break;
		}
	} else if (e_codec == VCD_CODEC_MPEG4) {
		switch (e_frame_type) {
		case VIDC_1080P_ENCODE_FRAMETYPE_NOT_CODED:
			p_frame->e_frame_type = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_I:
			p_frame->n_flags |= VCD_FRAME_FLAG_SYNCFRAME;
			p_frame->e_frame_type = VCD_FRAME_I;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_P:
			p_frame->e_frame_type = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_B:
			p_frame->e_frame_type = VCD_FRAME_B;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_SKIPPED:
			p_frame->e_frame_type = VCD_FRAME_NOTCODED;
			p_frame->n_data_len = 0;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_OTHERS:
			DDL_MSG_LOW("FRAMETYPE-OTHERS");
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_32BIT:
		default:
			DDL_MSG_LOW("UNKNOWN-FRAMETYPE");
			b_status = FALSE;
		break;
		}
	} else if (e_codec == VCD_CODEC_H263) {
		switch (e_frame_type) {
		case VIDC_1080P_ENCODE_FRAMETYPE_NOT_CODED:
			p_frame->e_frame_type = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_I:
			p_frame->n_flags |= VCD_FRAME_FLAG_SYNCFRAME;
			p_frame->e_frame_type = VCD_FRAME_I;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_P:
			p_frame->e_frame_type = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_SKIPPED:
			p_frame->e_frame_type = VCD_FRAME_NOTCODED;
			p_frame->n_data_len = 0;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_OTHERS:
			DDL_MSG_LOW("FRAMETYPE-OTHERS");
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_32BIT:
		default:
			DDL_MSG_LOW("UNKNOWN-FRAMETYPE");
			b_status = FALSE;
		break;
		}
	} else
		b_status = FALSE;
	return b_status;
}

static void ddl_get_mpeg4_dec_level(enum vcd_codec_level_type *p_level,
	u32 n_level, enum vcd_codec_profile_type e_mpeg4_profile)
{
	switch (n_level) {
	case VIDC_1080P_MPEG4_LEVEL0:
		*p_level = VCD_LEVEL_MPEG4_0;
	break;
	case VIDC_1080P_MPEG4_LEVEL0b:
		*p_level = VCD_LEVEL_MPEG4_0b;
	break;
	case VIDC_1080P_MPEG4_LEVEL1:
		*p_level = VCD_LEVEL_MPEG4_1;
	break;
	case VIDC_1080P_MPEG4_LEVEL2:
		*p_level = VCD_LEVEL_MPEG4_2;
	break;
	case VIDC_1080P_MPEG4_LEVEL3:
		*p_level = VCD_LEVEL_MPEG4_3;
	break;
	case VIDC_1080P_MPEG4_LEVEL3b:
		if (e_mpeg4_profile == VCD_PROFILE_MPEG4_SP)
			*p_level = VCD_LEVEL_MPEG4_7;
		else
			*p_level = VCD_LEVEL_MPEG4_3b;
	break;
	case VIDC_1080P_MPEG4_LEVEL4a:
		*p_level = VCD_LEVEL_MPEG4_4a;
	break;
	case VIDC_1080P_MPEG4_LEVEL5:
		*p_level = VCD_LEVEL_MPEG4_5;
	break;
	case VIDC_1080P_MPEG4_LEVEL6:
		*p_level = VCD_LEVEL_MPEG4_6;
	break;
	default:
		*p_level = VCD_LEVEL_UNKNOWN;
	break;
	}
}

static void ddl_get_h264_dec_level(enum vcd_codec_level_type *p_level,
	u32 n_level)
{
	switch (n_level) {
	case VIDC_1080P_H264_LEVEL1:
		*p_level = VCD_LEVEL_H264_1;
	break;
	case VIDC_1080P_H264_LEVEL1b:
		*p_level = VCD_LEVEL_H264_1b;
	break;
	case VIDC_1080P_H264_LEVEL1p1:
		*p_level = VCD_LEVEL_H264_1p1;
	break;
	case VIDC_1080P_H264_LEVEL1p2:
		*p_level = VCD_LEVEL_H264_1p2;
	break;
	case VIDC_1080P_H264_LEVEL1p3:
		*p_level = VCD_LEVEL_H264_1p3;
	break;
	case VIDC_1080P_H264_LEVEL2:
		*p_level = VCD_LEVEL_H264_2;
	break;
	case VIDC_1080P_H264_LEVEL2p1:
		*p_level = VCD_LEVEL_H264_2p1;
	break;
	case VIDC_1080P_H264_LEVEL2p2:
		*p_level = VCD_LEVEL_H264_2p2;
	break;
	case VIDC_1080P_H264_LEVEL3:
		*p_level = VCD_LEVEL_H264_3;
	break;
	case VIDC_1080P_H264_LEVEL3p1:
		*p_level = VCD_LEVEL_H264_3p1;
	break;
	case VIDC_1080P_H264_LEVEL3p2:
		*p_level = VCD_LEVEL_H264_3p2;
	break;
	case VIDC_1080P_H264_LEVEL4:
		*p_level = VCD_LEVEL_H264_4;
	break;
	default:
		*p_level = VCD_LEVEL_UNKNOWN;
	break;
	}
}

static void ddl_get_h263_dec_level(enum vcd_codec_level_type *p_level,
	u32 n_level)
{
	switch (n_level) {
	case VIDC_1080P_H263_LEVEL10:
		*p_level = VCD_LEVEL_H263_10;
	break;
	case VIDC_1080P_H263_LEVEL20:
		*p_level = VCD_LEVEL_H263_20;
	break;
	case VIDC_1080P_H263_LEVEL30:
		*p_level = VCD_LEVEL_H263_30;
	break;
	case VIDC_1080P_H263_LEVEL40:
		*p_level = VCD_LEVEL_H263_40;
	break;
	case VIDC_1080P_H263_LEVEL45:
		*p_level = VCD_LEVEL_H263_45;
	break;
	case VIDC_1080P_H263_LEVEL50:
		*p_level = VCD_LEVEL_H263_50;
	break;
	case VIDC_1080P_H263_LEVEL60:
		*p_level = VCD_LEVEL_H263_60;
	break;
	case VIDC_1080P_H263_LEVEL70:
		*p_level = VCD_LEVEL_H263_70;
	break;
	default:
		*p_level = VCD_LEVEL_UNKNOWN;
	break;
	}
}

static void ddl_get_vc1_dec_level(enum vcd_codec_level_type *p_level,
	u32 n_level, enum vcd_codec_profile_type e_vc1_profile)
{
	if (e_vc1_profile == VCD_PROFILE_VC1_ADVANCE) {
		switch (n_level) {
		case VIDC_SM_LEVEL_VC1_ADV_0:
			*p_level = VCD_LEVEL_VC1_0;
		break;
		case VIDC_SM_LEVEL_VC1_ADV_1:
			*p_level = VCD_LEVEL_VC1_1;
		break;
		case VIDC_SM_LEVEL_VC1_ADV_2:
			*p_level = VCD_LEVEL_VC1_2;
		break;
		case VIDC_SM_LEVEL_VC1_ADV_3:
			*p_level = VCD_LEVEL_VC1_3;
		break;
		case VIDC_SM_LEVEL_VC1_ADV_4:
			*p_level = VCD_LEVEL_VC1_4;
		break;
		default:
			*p_level = VCD_LEVEL_UNKNOWN;
		break;
		}
	} else {
		switch (n_level) {
		case VIDC_SM_LEVEL_VC1_LOW:
			*p_level = VCD_LEVEL_VC1_LOW;
		break;
		case VIDC_SM_LEVEL_VC1_MEDIUM:
			*p_level = VCD_LEVEL_VC1_MEDIUM;
		break;
		case VIDC_SM_LEVEL_VC1_HIGH:
			*p_level = VCD_LEVEL_VC1_HIGH;
		break;
		default:
			*p_level = VCD_LEVEL_UNKNOWN;
		break;
		}
	}
}

static void ddl_get_mpeg2_dec_level(enum vcd_codec_level_type *p_level,
	u32 n_level)
{
	switch (n_level) {
	case VIDC_SM_LEVEL_MPEG2_LOW:
		*p_level = VCD_LEVEL_MPEG2_LOW;
	break;
	case VIDC_SM_LEVEL_MPEG2_MAIN:
		*p_level = VCD_LEVEL_MPEG2_MAIN;
	break;
	case VIDC_SM_LEVEL_MPEG2_HIGH_1440:
		*p_level = VCD_LEVEL_MPEG2_HIGH_14;
	break;
	case VIDC_SM_LEVEL_MPEG2_HIGH:
		*p_level = VCD_LEVEL_MPEG2_HIGH;
	break;
	default:
		*p_level = VCD_LEVEL_UNKNOWN;
	break;
	}
}

static void ddl_get_dec_profile_level(struct ddl_decoder_data_type *p_decoder,
	u32 n_profile, u32 n_level)
{
	enum vcd_codec_profile_type profile = VCD_PROFILE_UNKNOWN;
	enum vcd_codec_level_type level = VCD_LEVEL_UNKNOWN;

	switch (p_decoder->codec_type.e_codec) {
	case VCD_CODEC_MPEG4:
	case VCD_CODEC_XVID:
		if (n_profile == VIDC_SM_PROFILE_MPEG4_SIMPLE)
			profile = VCD_PROFILE_MPEG4_SP;
		else if (n_profile == VIDC_SM_PROFILE_MPEG4_ADV_SIMPLE)
			profile = VCD_PROFILE_MPEG4_ASP;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_mpeg4_dec_level(&level, n_level, n_profile);
	break;
	case VCD_CODEC_H264:
		if (n_profile == VIDC_SM_PROFILE_H264_BASELINE)
			profile = VCD_PROFILE_H264_BASELINE;
		else if (n_profile == VIDC_SM_PROFILE_H264_MAIN)
			profile = VCD_PROFILE_H264_MAIN;
		else if (n_profile == VIDC_SM_PROFILE_H264_HIGH)
			profile = VCD_PROFILE_H264_HIGH;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_h264_dec_level(&level, n_level);
	break;
	case VCD_CODEC_H263:
		if (n_profile == VIDC_SM_PROFILE_H263_BASELINE)
			profile = VCD_PROFILE_H263_BASELINE;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_h263_dec_level(&level, n_level);
	break;
	case VCD_CODEC_MPEG2:
		if (n_profile == VIDC_SM_PROFILE_MPEG2_MAIN)
			profile = VCD_PROFILE_MPEG2_MAIN;
		else if (n_profile == VIDC_SM_PROFILE_MPEG2_SIMPLE)
			profile = VCD_PROFILE_MPEG2_SIMPLE;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_mpeg2_dec_level(&level, n_level);
	break;
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		if (n_profile == VIDC_SM_PROFILE_VC1_SIMPLE)
			profile = VCD_PROFILE_VC1_SIMPLE;
		else if (n_profile == VIDC_SM_PROFILE_VC1_MAIN)
			profile = VCD_PROFILE_VC1_MAIN;
		else if (n_profile == VIDC_SM_PROFILE_VC1_ADVANCED)
			profile = VCD_PROFILE_VC1_ADVANCE;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_vc1_dec_level(&level, n_level, n_profile);
	break;
	default:
		if (!n_profile)
			profile = VCD_PROFILE_UNKNOWN;
		if (!n_level)
			level = VCD_LEVEL_UNKNOWN;
	break;
	}
	p_decoder->profile.e_profile = profile;
	p_decoder->level.e_level = level;
}

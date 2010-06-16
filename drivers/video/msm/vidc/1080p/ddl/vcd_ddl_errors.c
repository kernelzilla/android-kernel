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
#include "vidc.h"

static u32 ddl_handle_hw_fatal_errors(struct ddl_client_context_type *p_ddl);
static u32 ddl_handle_client_fatal_errors(
	struct ddl_client_context_type *p_ddl);
static void ddl_input_failed_cb(struct ddl_client_context_type *p_ddl,
	u32 vcd_event, u32 vcd_status);
static u32 ddl_handle_core_recoverable_errors(
	struct ddl_client_context_type *p_ddl);
static u32 ddl_handle_core_warnings(u32 n_error_code);
static void ddl_handle_npf_decoding_error(
	struct ddl_client_context_type *p_ddl);
static void print_core_errors(u32 n_error_code);

void ddl_hw_fatal_cb(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	u32 n_error_code = p_ddl_context->n_cmd_err_status;

	DDL_MSG_FATAL("VIDC_HW_FATAL");
	p_ddl->e_cmd_state = DDL_CMD_INVALID;
	p_ddl_context->n_device_state = DDL_DEVICE_HWFATAL;

	p_ddl_context->ddl_callback(VCD_EVT_IND_HWERRFATAL, VCD_ERR_HW_FATAL,
		&n_error_code, sizeof(n_error_code),
		(u32 *)p_ddl, p_ddl->p_client_data);

	ddl_release_command_channel(p_ddl_context, p_ddl->n_command_channel);
}

static u32 ddl_handle_hw_fatal_errors(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	u32 b_status = FALSE, error_code = p_ddl_context->n_cmd_err_status;

	switch (error_code) {
	case VIDC_1080P_ERROR_INVALID_CHANNEL_NUMBER:
	case VIDC_1080P_ERROR_INVALID_COMMAND_ID:
	case VIDC_1080P_ERROR_CHANNEL_ALREADY_IN_USE:
	case VIDC_1080P_ERROR_CHANNEL_NOT_OPEN_BEFORE_CHANNEL_CLOSE:
	case VIDC_1080P_ERROR_OPEN_CH_ERROR_SEQ_START:
	case VIDC_1080P_ERROR_SEQ_START_ALREADY_CALLED:
	case VIDC_1080P_ERROR_OPEN_CH_ERROR_INIT_BUFFERS:
	case VIDC_1080P_ERROR_SEQ_START_ERROR_INIT_BUFFERS:
	case VIDC_1080P_ERROR_INIT_BUFFER_ALREADY_CALLED:
	case VIDC_1080P_ERROR_OPEN_CH_ERROR_FRAME_START:
	case VIDC_1080P_ERROR_SEQ_START_ERROR_FRAME_START:
	case VIDC_1080P_ERROR_INIT_BUFFERS_ERROR_FRAME_START:
	case VIDC_1080P_ERROR_CODEC_LIMIT_EXCEEDED:
	case VIDC_1080P_ERROR_MEM_ALLOCATION_FAILED:
	case VIDC_1080P_ERROR_INSUFFICIENT_CONTEXT_SIZE:
	case VIDC_1080P_ERROR_DIVIDE_BY_ZERO:
	case VIDC_1080P_ERROR_DESCRIPTOR_BUFFER_EMPTY:
	case VIDC_1080P_ERROR_DMA_TX_NOT_COMPLETE:
	case VIDC_1080P_ERROR_VSP_NOT_READY:
	case VIDC_1080P_ERROR_BUFFER_FULL_STATE:
		ddl_hw_fatal_cb(p_ddl);
		b_status = TRUE;
	break;
	default:
	break;
	}
	return b_status;
}

void ddl_client_fatal_cb(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;

	if (p_ddl->e_cmd_state == DDL_CMD_DECODE_FRAME)
		ddl_vidc_decode_dynamic_property(p_ddl, FALSE);
	else if (p_ddl->e_cmd_state == DDL_CMD_ENCODE_FRAME)
		ddl_vidc_encode_dynamic_property(p_ddl, FALSE);
	p_ddl->e_cmd_state = DDL_CMD_INVALID;
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_FAVIDC_ERROR",
		ddl_get_state_string(p_ddl->e_client_state));
	p_ddl->e_client_state = DDL_CLIENT_FAVIDC_ERROR;
	p_ddl_context->ddl_callback(VCD_EVT_IND_HWERRFATAL,
		VCD_ERR_CLIENT_FATAL, NULL, 0, (u32 *)p_ddl,
		p_ddl->p_client_data);
	ddl_release_command_channel(p_ddl_context, p_ddl->n_command_channel);
}

static u32 ddl_handle_client_fatal_errors(
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	u32 b_status = FALSE;

	switch (p_ddl_context->n_cmd_err_status) {
	case VIDC_1080P_ERROR_UNSUPPORTED_FEATURE_IN_PROFILE:
	case VIDC_1080P_ERROR_RESOLUTION_NOT_SUPPORTED:
	case VIDC_1080P_ERROR_FRAME_RATE_NOT_SUPPORTED:
	case VIDC_1080P_ERROR_INVALID_QP_VALUE:
	case VIDC_1080P_ERROR_INVALID_RC_REACTION_COEFFICIENT:
	case VIDC_1080P_ERROR_INVALID_CPB_SIZE_AT_GIVEN_LEVEL:
	case VIDC_1080P_ERROR_ALLOC_DPB_SIZE_NOT_SUFFICIENT:
	case VIDC_1080P_ERROR_NUM_DPB_OUT_OF_RANGE:
	case VIDC_1080P_ERROR_NULL_METADATA_INPUT_POINTER:
	case VIDC_1080P_ERROR_NULL_DPB_POINTER:
	case VIDC_1080P_ERROR_NULL_OTH_EXT_BUFADDR:
	case VIDC_1080P_ERROR_NULL_MV_POINTER:
		b_status = TRUE;
		DDL_MSG_ERROR("VIDC_CLIENT_FATAL!!");
	break;
	default:
	break;
	}
	if (!b_status)
		DDL_MSG_ERROR("VIDC_UNKNOWN_OP_FAILED");
	ddl_client_fatal_cb(p_ddl);
	return TRUE;
}

static void ddl_input_failed_cb(struct ddl_client_context_type *p_ddl,
	u32 vcd_event, u32 vcd_status)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	u32 n_payload_size = sizeof(struct ddl_frame_data_type_tag);

	p_ddl->e_cmd_state = DDL_CMD_INVALID;
	if (p_ddl->b_decoding)
		ddl_vidc_decode_dynamic_property(p_ddl, FALSE);
	else
		ddl_vidc_encode_dynamic_property(p_ddl, FALSE);
	if (p_ddl->e_client_state == DDL_CLIENT_WAIT_FOR_INITCODECDONE) {
		n_payload_size = 0;
		DDL_MSG_LOW("ddl_state_transition: %s ~~> "
			"DDL_CLIENT_WAIT_FOR_INITCODEC",
			ddl_get_state_string(p_ddl->e_client_state));
		p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_INITCODEC;
	} else {
		DDL_MSG_LOW("ddl_state_transition: %s ~~> "
			"DDL_CLIENT_WAIT_FOR_FRAME",
			ddl_get_state_string(p_ddl->e_client_state));
		p_ddl->e_client_state = DDL_CLIENT_WAIT_FOR_FRAME;
	}
	p_ddl_context->ddl_callback(vcd_event, vcd_status, &p_ddl->input_frame,
		n_payload_size, (u32 *)p_ddl, p_ddl->p_client_data);
}

static u32 ddl_handle_core_recoverable_errors(
	struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	u32 vcd_status = VCD_S_SUCCESS;
	u32 vcd_event = VCD_EVT_RESP_INPUT_DONE;
	u32 b_eos = FALSE, b_status = FALSE;

	if ((p_ddl->e_cmd_state == DDL_CMD_DECODE_FRAME) ||
		(p_ddl->e_cmd_state == DDL_CMD_ENCODE_FRAME) ||
		(p_ddl->e_cmd_state == DDL_CMD_HEADER_PARSE)) {
		if (p_ddl_context->n_cmd_err_status ==
			VIDC_1080P_ERROR_NON_PAIRED_FIELD_NOT_SUPPORTED) {
			ddl_handle_npf_decoding_error(p_ddl);
			b_status = TRUE;
		} else {
			switch (p_ddl_context->n_cmd_err_status) {
			case VIDC_1080P_ERROR_HEADER_NOT_FOUND:
				if (p_ddl->e_client_state !=
					DDL_CLIENT_WAIT_FOR_INITCODECDONE){
					DDL_MSG_ERROR("VIDC_CRITICAL_HEADER");
					DDL_MSG_ERROR("Unrecoverable error");
				} else {
					DDL_MSG_ERROR("VIDC_HDR_PARSE_FAIL");
					vcd_event = VCD_EVT_RESP_START;
					vcd_status = VCD_ERR_NO_SEQ_HDR;
					p_ddl->codec_data.decoder.\
						b_header_in_start = FALSE;
				}
			break;
			case VIDC_1080P_ERROR_SYNC_POINT_NOT_RECEIVED:
				vcd_status = VCD_ERR_IFRAME_EXPECTED;
			break;
			case VIDC_1080P_ERROR_NO_BUFFER_RELEASED_FROM_HOST:
			{
				u32 n_pending_display = 0, n_release_mask;

				n_release_mask =
					p_ddl->codec_data.decoder.\
					dpb_mask.n_hw_mask;
				while (n_release_mask > 0) {
					if (n_release_mask & 0x1)
						n_pending_display++;
					n_release_mask >>= 1;
				}
				if (n_pending_display >= p_ddl->codec_data.\
					decoder.n_min_dpb_num) {
					DDL_MSG_ERROR("VIDC_FW_ISSUE_REQ_BUF");
					ddl_client_fatal_cb(p_ddl);
					b_status = TRUE;
				} else {
					vcd_event = VCD_EVT_RESP_OUTPUT_REQ;
					DDL_MSG_LOW("VIDC_OUTPUT_BUF_REQ!!");
				}
			}
			break;
			case VIDC_1080P_ERROR_BIT_STREAM_BUF_EXHAUST:
			case VIDC_1080P_ERROR_MB_HEADER_NOT_DONE:
			case VIDC_1080P_ERROR_MB_COEFF_NOT_DONE:
			case VIDC_1080P_ERROR_CODEC_SLICE_NOT_DONE:
			case VIDC_1080P_ERROR_VIDC_CORE_TIME_OUT:
			case VIDC_1080P_ERROR_VC1_BITPLANE_DECODE_ERR:
			case VIDC_1080P_ERROR_RESOLUTION_MISMATCH:
			case VIDC_1080P_ERROR_NV_QUANT_ERR:
			case VIDC_1080P_ERROR_SYNC_MARKER_ERR:
			case VIDC_1080P_ERROR_FEATURE_NOT_SUPPORTED:
			case VIDC_1080P_ERROR_MEM_CORRUPTION:
			case VIDC_1080P_ERROR_INVALID_REFERENCE_FRAME:
			case VIDC_1080P_ERROR_PICTURE_CODING_TYPE_ERR:
			case VIDC_1080P_ERROR_MV_RANGE_ERR:
			case VIDC_1080P_ERROR_PICTURE_STRUCTURE_ERR:
			case VIDC_1080P_ERROR_SLICE_ADDR_INVALID:
			case VIDC_1080P_ERROR_NON_FRAME_DATA_RECEIVED:
			case VIDC_1080P_ERROR_INCOMPLETE_FRAME:
			case VIDC_1080P_ERROR_NALU_HEADER_ERROR:
			case VIDC_1080P_ERROR_SPS_PARSE_ERROR:
			case VIDC_1080P_ERROR_PPS_PARSE_ERROR:
			case VIDC_1080P_ERROR_SLICE_PARSE_ERROR:
				vcd_status = VCD_ERR_BITSTREAM_ERR;
				DDL_MSG_ERROR("VIDC_BIT_STREAM_ERR");
			break;
			default:
			break;
			}
			if (((!vcd_status) || (vcd_event !=
				VCD_EVT_RESP_INPUT_DONE)) && !b_status) {
				p_ddl->input_frame.b_frm_trans_end = TRUE;
				b_eos = ((vcd_event ==
				VCD_EVT_RESP_INPUT_DONE) &&
				(p_ddl->input_frame.vcd_frm.n_flags &
				VCD_FRAME_FLAG_EOS));
				if ((p_ddl->b_decoding && b_eos) ||
					!p_ddl->b_decoding)
					p_ddl->input_frame.b_frm_trans_end =
						FALSE;
				if ((vcd_event == VCD_EVT_RESP_INPUT_DONE ||
					vcd_event == VCD_EVT_RESP_START) &&
					p_ddl->b_decoding &&
					!p_ddl->codec_data.decoder.\
					b_header_in_start &&
					!p_ddl->codec_data.decoder.\
					dec_disp_info.n_img_size_x &&
					!p_ddl->codec_data.decoder.\
					dec_disp_info.n_img_size_y) {
					vcd_status = VCD_S_SUCCESS;
					p_ddl->input_frame.vcd_frm.n_flags |=
					VCD_FRAME_FLAG_CODECCONFIG;
					p_ddl->input_frame.b_frm_trans_end =
						!b_eos;
					p_ddl->codec_data.decoder.\
						dec_disp_info.n_img_size_x =
							0xff;
				}
				ddl_input_failed_cb(p_ddl, vcd_event,
					vcd_status);
				if (!p_ddl->b_decoding) {
					p_ddl->output_frame.b_frm_trans_end =
						!b_eos;
					p_ddl->output_frame.vcd_frm.\
						n_data_len = 0;
					p_ddl_context->ddl_callback(
						VCD_EVT_RESP_OUTPUT_DONE,
						VCD_ERR_FAIL,
						&p_ddl->output_frame,
						sizeof(struct
						ddl_frame_data_type_tag),
						(u32 *)p_ddl,
						p_ddl->p_client_data);
					if (b_eos) {
						DDL_MSG_LOW(
							"VIDC_ENC_EOS_DONE");
						p_ddl_context->ddl_callback(
							VCD_EVT_RESP_EOS_DONE,
							VCD_S_SUCCESS,
							NULL, 0, (u32 *)p_ddl,
							p_ddl->p_client_data);
					}
				}
				if (p_ddl->b_decoding && b_eos)
					ddl_vidc_decode_eos_run(p_ddl);
				else
					ddl_release_command_channel(
						p_ddl_context,
						p_ddl->n_command_channel);
				b_status = TRUE;
			}
		}
	}
	return b_status;
}

static u32 ddl_handle_core_warnings(u32 n_err_status)
{
	u32 b_status = FALSE;

	switch (n_err_status) {
	case VIDC_1080P_WARN_COMMAND_FLUSHED:
	case VIDC_1080P_WARN_FRAME_RATE_UNKNOWN:
	case VIDC_1080P_WARN_ASPECT_RATIO_UNKNOWN:
	case VIDC_1080P_WARN_COLOR_PRIMARIES_UNKNOWN:
	case VIDC_1080P_WARN_TRANSFER_CHAR_UNKNOWN:
	case VIDC_1080P_WARN_MATRIX_COEFF_UNKNOWN:
	case VIDC_1080P_WARN_NON_SEQ_SLICE_ADDR:
	case VIDC_1080P_WARN_BROKEN_LINK:
	case VIDC_1080P_WARN_FRAME_CONCEALED:
	case VIDC_1080P_WARN_PROFILE_UNKNOWN:
	case VIDC_1080P_WARN_LEVEL_UNKNOWN:
	case VIDC_1080P_WARN_BIT_RATE_NOT_SUPPORTED:
	case VIDC_1080P_WARN_COLOR_DIFF_FORMAT_NOT_SUPPORTED:
	case VIDC_1080P_WARN_NULL_EXTRA_METADATA_POINTER:
	case VIDC_1080P_ERROR_NULL_FW_DEBUG_INFO_POINTER:
	case VIDC_1080P_ERROR_ALLOC_DEBUG_INFO_SIZE_INSUFFICIENT:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_NUM_CONCEAL_MB:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_QP:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_CONCEAL_MB:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_VC1_PARAM:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_SEI:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_VUI:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_EXTRA:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_DATA_NONE:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_MB_INFO:
	case VIDC_1080P_WARN_METADATA_NO_SPACE_SLICE_SIZE:
	case VIDC_1080P_WARN_RESOLUTION_WARNING:
		b_status = TRUE;
		DDL_MSG_ERROR("VIDC_WARNING_IGNORED");
	break;
	default:
	break;
	}
	return b_status;
}

u32 ddl_handle_core_errors(struct ddl_context_type *p_ddl_context)
{
	struct ddl_client_context_type *p_ddl;
	u32 n_channel_inst_id, b_status = FALSE;

	if (!p_ddl_context->n_cmd_err_status &&
		!p_ddl_context->n_disp_pic_err_status) {
		DDL_MSG_ERROR("VIDC_NO_ERROR");
	} else {
		vidc_1080p_get_returned_channel_inst_id(&n_channel_inst_id);
		vidc_1080p_clear_returned_channel_inst_id();
		p_ddl = ddl_get_current_ddl_client_for_channel_id(p_ddl_context,
			p_ddl_context->n_response_cmd_ch_id);
		if (!p_ddl) {
			DDL_MSG_ERROR("VIDC_SPURIOUS_INTERRUPT_ERROR");
			b_status = TRUE;
		} else {
			u32 b_disp_status;

			if (p_ddl_context->n_cmd_err_status)
				print_core_errors(
					p_ddl_context->n_cmd_err_status);
			if (p_ddl_context->n_disp_pic_err_status)
				print_core_errors(
					p_ddl_context->n_disp_pic_err_status);
			b_status = ddl_handle_core_warnings(
				p_ddl_context->n_cmd_err_status);
			b_disp_status = ddl_handle_core_warnings(
				p_ddl_context->n_disp_pic_err_status);
			if (!b_status && !b_disp_status) {
				DDL_MSG_ERROR("ddl_warning:Unknown");
				b_status = ddl_handle_hw_fatal_errors(p_ddl);
				if (!b_status)
					b_status =
					ddl_handle_core_recoverable_errors(
						p_ddl);
				if (!b_status)
					b_status =
					ddl_handle_client_fatal_errors(p_ddl);
			}
		}
	}
	return b_status;
}

static void ddl_handle_npf_decoding_error(struct ddl_client_context_type *p_ddl)
{
	struct vidc_1080p_dec_disp_info_type *p_dec_disp_info =
		&(p_ddl->codec_data.decoder.dec_disp_info);

	if (!p_ddl->b_decoding) {
		DDL_MSG_ERROR("VIDC_FW_ISSUE_ENC_NPF");
		ddl_client_fatal_cb(p_ddl);
	} else {
		vidc_sm_get_frame_tags(
			&p_ddl->shared_mem[p_ddl->n_command_channel],
			&p_dec_disp_info->n_tag_top,
			&p_dec_disp_info->n_tag_bottom);
		ddl_vidc_decode_dynamic_property(p_ddl, FALSE);
		p_ddl->output_frame.vcd_frm.n_ip_frm_tag =
			p_dec_disp_info->n_tag_top;
		p_ddl->output_frame.vcd_frm.p_physical = NULL;
		p_ddl->output_frame.vcd_frm.p_virtual = NULL;
		p_ddl->output_frame.b_frm_trans_end = FALSE;
		p_ddl->p_ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE,
			VCD_ERR_INTRLCD_FIELD_DROP, &(p_ddl->output_frame),
			sizeof(struct ddl_frame_data_type_tag),
			(u32 *) p_ddl, p_ddl->p_client_data);
		ddl_vidc_decode_frame_run(p_ddl);
	}
}

void print_core_errors(u32 n_error_code)
{
	s8 *p_string = NULL;

	switch (n_error_code) {
	case VIDC_1080P_ERROR_INVALID_CHANNEL_NUMBER:
		p_string = "VIDC_1080P_ERROR_INVALID_CHANNEL_NUMBER";
	break;
	case VIDC_1080P_ERROR_INVALID_COMMAND_ID:
		p_string = "VIDC_1080P_ERROR_INVALID_COMMAND_ID";
	break;
	case VIDC_1080P_ERROR_CHANNEL_ALREADY_IN_USE:
		p_string = "VIDC_1080P_ERROR_CHANNEL_ALREADY_IN_USE";
	break;
	case VIDC_1080P_ERROR_CHANNEL_NOT_OPEN_BEFORE_CHANNEL_CLOSE:
		p_string =
		"VIDC_1080P_ERROR_CHANNEL_NOT_OPEN_BEFORE_CHANNEL_CLOSE";
	break;
	case VIDC_1080P_ERROR_OPEN_CH_ERROR_SEQ_START:
		p_string = "VIDC_1080P_ERROR_OPEN_CH_ERROR_SEQ_START";
	break;
	case VIDC_1080P_ERROR_SEQ_START_ALREADY_CALLED:
		p_string = "VIDC_1080P_ERROR_SEQ_START_ALREADY_CALLED";
	break;
	case VIDC_1080P_ERROR_OPEN_CH_ERROR_INIT_BUFFERS:
		p_string = "VIDC_1080P_ERROR_OPEN_CH_ERROR_INIT_BUFFERS";
	break;
	case VIDC_1080P_ERROR_SEQ_START_ERROR_INIT_BUFFERS:
		p_string = "VIDC_1080P_ERROR_SEQ_START_ERROR_INIT_BUFFERS";
	break;
	case VIDC_1080P_ERROR_INIT_BUFFER_ALREADY_CALLED:
		p_string = "VIDC_1080P_ERROR_INIT_BUFFER_ALREADY_CALLED";
	break;
	case VIDC_1080P_ERROR_OPEN_CH_ERROR_FRAME_START:
		p_string = "VIDC_1080P_ERROR_OPEN_CH_ERROR_FRAME_START";
	break;
	case VIDC_1080P_ERROR_SEQ_START_ERROR_FRAME_START:
		p_string = "VIDC_1080P_ERROR_SEQ_START_ERROR_FRAME_START";
	break;
	case VIDC_1080P_ERROR_INIT_BUFFERS_ERROR_FRAME_START:
		p_string = "VIDC_1080P_ERROR_INIT_BUFFERS_ERROR_FRAME_START";
	break;
	case VIDC_1080P_ERROR_CODEC_LIMIT_EXCEEDED:
		p_string = "VIDC_1080P_ERROR_CODEC_LIMIT_EXCEEDED";
	break;
	case VIDC_1080P_ERROR_MEM_ALLOCATION_FAILED:
		p_string = "VIDC_1080P_ERROR_MEM_ALLOCATION_FAILED";
	break;
	case VIDC_1080P_ERROR_INSUFFICIENT_CONTEXT_SIZE:
		p_string = "VIDC_1080P_ERROR_INSUFFICIENT_CONTEXT_SIZE";
	break;
	case VIDC_1080P_ERROR_DIVIDE_BY_ZERO:
		p_string = "VIDC_1080P_ERROR_DIVIDE_BY_ZERO";
	break;
	case VIDC_1080P_ERROR_DESCRIPTOR_BUFFER_EMPTY:
		p_string = "VIDC_1080P_ERROR_DESCRIPTOR_BUFFER_EMPTY";
	break;
	case VIDC_1080P_ERROR_DMA_TX_NOT_COMPLETE:
		p_string = "VIDC_1080P_ERROR_DMA_TX_NOT_COMPLETE";
	break;
	case VIDC_1080P_ERROR_VSP_NOT_READY:
		p_string = "VIDC_1080P_ERROR_VSP_NOT_READY";
	break;
	case VIDC_1080P_ERROR_BUFFER_FULL_STATE:
		p_string = "VIDC_1080P_ERROR_BUFFER_FULL_STATE";
	break;
	case VIDC_1080P_ERROR_UNSUPPORTED_FEATURE_IN_PROFILE:
		p_string = "VIDC_1080P_ERROR_UNSUPPORTED_FEATURE_IN_PROFILE";
	break;
	case VIDC_1080P_ERROR_RESOLUTION_NOT_SUPPORTED:
		p_string = "VIDC_1080P_ERROR_RESOLUTION_NOT_SUPPORTED";
	break;
	case VIDC_1080P_ERROR_FRAME_RATE_NOT_SUPPORTED:
		p_string = "VIDC_1080P_ERROR_FRAME_RATE_NOT_SUPPORTED";
	break;
	case VIDC_1080P_ERROR_INVALID_QP_VALUE:
		p_string = "VIDC_1080P_ERROR_INVALID_QP_VALUE";
	break;
	case VIDC_1080P_ERROR_INVALID_RC_REACTION_COEFFICIENT:
		p_string = "VIDC_1080P_ERROR_INVALID_RC_REACTION_COEFFICIENT";
	break;
	case VIDC_1080P_ERROR_INVALID_CPB_SIZE_AT_GIVEN_LEVEL:
		p_string = "VIDC_1080P_ERROR_INVALID_CPB_SIZE_AT_GIVEN_LEVEL";
	break;
	case VIDC_1080P_ERROR_ALLOC_DPB_SIZE_NOT_SUFFICIENT:
		p_string = "VIDC_1080P_ERROR_ALLOC_DPB_SIZE_NOT_SUFFICIENT";
	break;
	case VIDC_1080P_ERROR_NUM_DPB_OUT_OF_RANGE:
		p_string = "VIDC_1080P_ERROR_NUM_DPB_OUT_OF_RANGE";
	break;
	case VIDC_1080P_ERROR_NULL_METADATA_INPUT_POINTER:
		p_string = "VIDC_1080P_ERROR_NULL_METADATA_INPUT_POINTER";
	break;
	case VIDC_1080P_ERROR_NULL_DPB_POINTER:
		p_string = "VIDC_1080P_ERROR_NULL_DPB_POINTER";
	break;
	case VIDC_1080P_ERROR_NULL_OTH_EXT_BUFADDR:
		p_string = "VIDC_1080P_ERROR_NULL_OTH_EXT_BUFADDR";
	break;
	case VIDC_1080P_ERROR_NULL_MV_POINTER:
		p_string = "VIDC_1080P_ERROR_NULL_MV_POINTER";
	break;
	case VIDC_1080P_ERROR_NON_PAIRED_FIELD_NOT_SUPPORTED:
		p_string = "VIDC_1080P_ERROR_NON_PAIRED_FIELD_NOT_SUPPORTED";
	break;
	case VIDC_1080P_ERROR_HEADER_NOT_FOUND:
		p_string = "VIDC_1080P_ERROR_HEADER_NOT_FOUND";
	break;
	case VIDC_1080P_ERROR_SYNC_POINT_NOT_RECEIVED:
		p_string = "VIDC_1080P_ERROR_SYNC_POINT_NOT_RECEIVED";
	break;
	case VIDC_1080P_ERROR_NO_BUFFER_RELEASED_FROM_HOST:
		p_string = "VIDC_1080P_ERROR_NO_BUFFER_RELEASED_FROM_HOST";
	break;
	case VIDC_1080P_ERROR_BIT_STREAM_BUF_EXHAUST:
		p_string = "VIDC_1080P_ERROR_BIT_STREAM_BUF_EXHAUST";
	break;
	case VIDC_1080P_ERROR_MB_HEADER_NOT_DONE:
		p_string = "VIDC_1080P_ERROR_MB_HEADER_NOT_DONE";
	break;
	case VIDC_1080P_ERROR_MB_COEFF_NOT_DONE:
		p_string = "VIDC_1080P_ERROR_MB_COEFF_NOT_DONE";
	break;
	case VIDC_1080P_ERROR_CODEC_SLICE_NOT_DONE:
		p_string = "VIDC_1080P_ERROR_CODEC_SLICE_NOT_DONE";
	break;
	case VIDC_1080P_ERROR_VIDC_CORE_TIME_OUT:
		p_string = "VIDC_1080P_ERROR_VIDC_CORE_TIME_OUT";
	break;
	case VIDC_1080P_ERROR_VC1_BITPLANE_DECODE_ERR:
		p_string = "VIDC_1080P_ERROR_VC1_BITPLANE_DECODE_ERR";
	break;
	case VIDC_1080P_ERROR_RESOLUTION_MISMATCH:
		p_string = "VIDC_1080P_ERROR_RESOLUTION_MISMATCH";
	break;
	case VIDC_1080P_ERROR_NV_QUANT_ERR:
		p_string = "VIDC_1080P_ERROR_NV_QUANT_ERR";
	break;
	case VIDC_1080P_ERROR_SYNC_MARKER_ERR:
		p_string = "VIDC_1080P_ERROR_SYNC_MARKER_ERR";
	break;
	case VIDC_1080P_ERROR_FEATURE_NOT_SUPPORTED:
		p_string = "VIDC_1080P_ERROR_FEATURE_NOT_SUPPORTED";
	break;
	case VIDC_1080P_ERROR_MEM_CORRUPTION:
		p_string = "VIDC_1080P_ERROR_MEM_CORRUPTION";
	break;
	case VIDC_1080P_ERROR_INVALID_REFERENCE_FRAME:
		p_string = "VIDC_1080P_ERROR_INVALID_REFERENCE_FRAME";
	break;
	case VIDC_1080P_ERROR_PICTURE_CODING_TYPE_ERR:
		p_string = "VIDC_1080P_ERROR_PICTURE_CODING_TYPE_ERR";
	break;
	case VIDC_1080P_ERROR_MV_RANGE_ERR:
		p_string = "VIDC_1080P_ERROR_MV_RANGE_ERR";
	break;
	case VIDC_1080P_ERROR_PICTURE_STRUCTURE_ERR:
		p_string = "VIDC_1080P_ERROR_PICTURE_STRUCTURE_ERR";
	break;
	case VIDC_1080P_ERROR_SLICE_ADDR_INVALID:
		p_string = "VIDC_1080P_ERROR_SLICE_ADDR_INVALID";
	break;
	case VIDC_1080P_ERROR_NON_FRAME_DATA_RECEIVED:
		p_string = "VIDC_1080P_ERROR_NON_FRAME_DATA_RECEIVED";
	break;
	case VIDC_1080P_ERROR_INCOMPLETE_FRAME:
		p_string = "VIDC_1080P_ERROR_INCOMPLETE_FRAME";
	break;
	case VIDC_1080P_ERROR_NALU_HEADER_ERROR:
		p_string = "VIDC_1080P_ERROR_NALU_HEADER_ERROR";
	break;
	case VIDC_1080P_ERROR_SPS_PARSE_ERROR:
		p_string = "VIDC_1080P_ERROR_SPS_PARSE_ERROR";
	break;
	case VIDC_1080P_ERROR_PPS_PARSE_ERROR:
		p_string = "VIDC_1080P_ERROR_PPS_PARSE_ERROR";
	break;
	case VIDC_1080P_ERROR_SLICE_PARSE_ERROR:
		p_string = "VIDC_1080P_ERROR_SLICE_PARSE_ERROR";
	break;
	case VIDC_1080P_WARN_COMMAND_FLUSHED:
		p_string = "VIDC_1080P_WARN_COMMAND_FLUSHED";
	break;
	case VIDC_1080P_WARN_FRAME_RATE_UNKNOWN:
		p_string = "VIDC_1080P_WARN_FRAME_RATE_UNKNOWN";
	break;
	case VIDC_1080P_WARN_ASPECT_RATIO_UNKNOWN:
		p_string = "VIDC_1080P_WARN_ASPECT_RATIO_UNKNOWN";
	break;
	case VIDC_1080P_WARN_COLOR_PRIMARIES_UNKNOWN:
		p_string = "VIDC_1080P_WARN_COLOR_PRIMARIES_UNKNOWN";
	break;
	case VIDC_1080P_WARN_TRANSFER_CHAR_UNKNOWN:
		p_string = "VIDC_1080P_WARN_TRANSFER_CHAR_UNKNOWN";
	break;
	case VIDC_1080P_WARN_MATRIX_COEFF_UNKNOWN:
		p_string = "VIDC_1080P_WARN_MATRIX_COEFF_UNKNOWN";
	break;
	case VIDC_1080P_WARN_NON_SEQ_SLICE_ADDR:
		p_string = "VIDC_1080P_WARN_NON_SEQ_SLICE_ADDR";
	break;
	case VIDC_1080P_WARN_BROKEN_LINK:
		p_string = "VIDC_1080P_WARN_BROKEN_LINK";
	break;
	case VIDC_1080P_WARN_FRAME_CONCEALED:
		p_string = "VIDC_1080P_WARN_FRAME_CONCEALED";
	break;
	case VIDC_1080P_WARN_PROFILE_UNKNOWN:
		p_string = "VIDC_1080P_WARN_PROFILE_UNKNOWN";
	break;
	case VIDC_1080P_WARN_LEVEL_UNKNOWN:
		p_string = "VIDC_1080P_WARN_LEVEL_UNKNOWN";
	break;
	case VIDC_1080P_WARN_BIT_RATE_NOT_SUPPORTED:
		p_string = "VIDC_1080P_WARN_BIT_RATE_NOT_SUPPORTED";
	break;
	case VIDC_1080P_WARN_COLOR_DIFF_FORMAT_NOT_SUPPORTED:
		p_string = "VIDC_1080P_WARN_COLOR_DIFF_FORMAT_NOT_SUPPORTED";
	break;
	case VIDC_1080P_WARN_NULL_EXTRA_METADATA_POINTER:
		p_string = "VIDC_1080P_WARN_NULL_EXTRA_METADATA_POINTER";
	break;
	case VIDC_1080P_ERROR_NULL_FW_DEBUG_INFO_POINTER:
		p_string = "VIDC_1080P_ERROR_NULL_FW_DEBUG_INFO_POINTER";
	break;
	case VIDC_1080P_ERROR_ALLOC_DEBUG_INFO_SIZE_INSUFFICIENT:
		p_string =
		"VIDC_1080P_ERROR_ALLOC_DEBUG_INFO_SIZE_INSUFFICIENT";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_NUM_CONCEAL_MB:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_NUM_CONCEAL_MB";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_QP:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_QP";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_CONCEAL_MB:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_CONCEAL_MB";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_VC1_PARAM:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_VC1_PARAM";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_SEI:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_SEI";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_VUI:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_VUI";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_EXTRA:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_EXTRA";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_DATA_NONE:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_DATA_NONE";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_MB_INFO:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_MB_INFO";
	break;
	case VIDC_1080P_WARN_METADATA_NO_SPACE_SLICE_SIZE:
		p_string = "VIDC_1080P_WARN_METADATA_NO_SPACE_SLICE_SIZE";
	break;
	case VIDC_1080P_WARN_RESOLUTION_WARNING:
		p_string = "VIDC_1080P_WARN_RESOLUTION_WARNING";
	break;
	}
	if (p_string)
		DDL_MSG_ERROR("Error code = 0x%x : %s", n_error_code,
			p_string);
}

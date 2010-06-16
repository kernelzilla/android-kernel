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

struct ddl_context_type *ddl_get_context(void)
{
	static struct ddl_context_type ddl_context;
	return &ddl_context;
}

#ifdef DDL_MSG_LOG
s8 *ddl_get_state_string(enum ddl_client_state_type e_client_state)
{
	s8 *ptr;

	switch (e_client_state) {
	case DDL_CLIENT_INVALID:
		ptr = "INVALID        ";
	break;
	case DDL_CLIENT_OPEN:
		ptr = "OPEN   ";
	break;
	case DDL_CLIENT_WAIT_FOR_CHDONE:
		ptr = "WAIT_FOR_CHDONE       ";
	break;
	case DDL_CLIENT_WAIT_FOR_INITCODEC:
		ptr = "WAIT_FOR_INITCODEC    ";
	break;
	case DDL_CLIENT_WAIT_FOR_INITCODECDONE:
		ptr = "WAIT_FOR_INITCODECDONE";
	break;
	case DDL_CLIENT_WAIT_FOR_DPB:
		ptr = "WAIT_FOR_DPB   ";
	break;
	case DDL_CLIENT_WAIT_FOR_DPBDONE:
		ptr = "WAIT_FOR_DPBDONE";
	break;
	case DDL_CLIENT_WAIT_FOR_FRAME:
		ptr = "WAIT_FOR_FRAME ";
	break;
	case DDL_CLIENT_WAIT_FOR_FRAME_DONE:
		ptr = "WAIT_FOR_FRAME_DONE   ";
	break;
	case DDL_CLIENT_WAIT_FOR_EOS_DONE:
		ptr = "WAIT_FOR_EOS_DONE     ";
	break;
	case DDL_CLIENT_WAIT_FOR_CHEND:
		ptr = "WAIT_FOR_CHEND ";
	break;
	default:
		ptr = "UNKNOWN        ";
	break;
	}
	return ptr;
}
#endif

u32 ddl_client_transact(u32 operation,
	struct ddl_client_context_type **pddl_client)
{
	struct ddl_context_type *p_ddl_context;
	u32 ret_status = VCD_ERR_FAIL;
	s32 n_counter;

	p_ddl_context = ddl_get_context();
	switch (operation) {
	case DDL_FREE_CLIENT:
		ret_status = VCD_ERR_MAX_CLIENT;
		for (n_counter = 0; (n_counter < VCD_MAX_NO_CLIENT) &&
			(ret_status == VCD_ERR_MAX_CLIENT); ++n_counter) {
			if (*pddl_client == p_ddl_context->a_ddl_clients
				[n_counter]) {
					kfree(*pddl_client);
					*pddl_client = NULL;
					p_ddl_context->a_ddl_clients[n_counter]
						= NULL;
				ret_status = VCD_S_SUCCESS;
			}
		}
	break;
	case DDL_GET_CLIENT:
		ret_status = VCD_ERR_MAX_CLIENT;
		for (n_counter = (VCD_MAX_NO_CLIENT - 1); (n_counter >= 0) &&
			(ret_status == VCD_ERR_MAX_CLIENT); --n_counter) {
			if (!p_ddl_context->a_ddl_clients[n_counter]) {
				*pddl_client =
					(struct ddl_client_context_type *)
					kmalloc(sizeof(struct
					ddl_client_context_type), GFP_KERNEL);
				if (!*pddl_client)
					ret_status = VCD_ERR_ALLOC_FAIL;
				else {
					memset(*pddl_client, 0,
						sizeof(struct
						ddl_client_context_type));
					p_ddl_context->a_ddl_clients
						[n_counter] = *pddl_client;
					(*pddl_client)->p_ddl_context =
						p_ddl_context;
					ret_status = VCD_S_SUCCESS;
				}
			}
		}
	break;
	case DDL_INIT_CLIENTS:
		for (n_counter = 0; n_counter < VCD_MAX_NO_CLIENT; ++n_counter)
			p_ddl_context->a_ddl_clients[n_counter] = NULL;
		ret_status = VCD_S_SUCCESS;
	break;
	case DDL_ACTIVE_CLIENT:
		for (n_counter = 0; n_counter < VCD_MAX_NO_CLIENT;
			++n_counter) {
			if (p_ddl_context->a_ddl_clients[n_counter]) {
				ret_status = VCD_S_SUCCESS;
				break;
			}
		}
	break;
	default:
		ret_status = VCD_ERR_ILLEGAL_PARM;
	break;
	}
	return ret_status;
}

u32 ddl_decoder_dpb_transact(struct ddl_decoder_data_type *p_decoder,
	struct ddl_frame_data_type_tag  *p_in_out_frame, u32 n_operation)
{
	struct ddl_frame_data_type_tag *p_found_frame = NULL;
	struct ddl_mask_type *p_dpb_mask = &p_decoder->dpb_mask;
	u32 vcd_status = VCD_S_SUCCESS, n_loopc;

	switch (n_operation) {
	case DDL_DPB_OP_MARK_BUSY:
	case DDL_DPB_OP_MARK_FREE:
		for (n_loopc = 0; !p_found_frame && n_loopc <
			p_decoder->dp_buf.n_no_of_dec_pic_buf; ++n_loopc) {
			if (p_in_out_frame->vcd_frm.p_physical ==
				p_decoder->dp_buf.a_dec_pic_buffers[n_loopc].
				vcd_frm.p_physical) {
				p_found_frame = &(p_decoder->dp_buf.
					a_dec_pic_buffers[n_loopc]);
			break;
			}
		}
		if (p_found_frame) {
			if (n_operation == DDL_DPB_OP_MARK_BUSY) {
				p_dpb_mask->n_hw_mask &=
					(~(u32)(0x1 << n_loopc));
				*p_in_out_frame = *p_found_frame;
			} else if (n_operation == DDL_DPB_OP_MARK_FREE) {
				p_dpb_mask->n_client_mask |= (0x1 << n_loopc);
				*p_found_frame = *p_in_out_frame;
			}
		} else {
			p_in_out_frame->vcd_frm.p_physical = NULL;
			p_in_out_frame->vcd_frm.p_virtual = NULL;
			vcd_status = VCD_ERR_BAD_POINTER;
			DDL_MSG_ERROR("BUF_NOT_FOUND");
		}
	break;
	case DDL_DPB_OP_SET_MASK:
		p_dpb_mask->n_hw_mask |= p_dpb_mask->n_client_mask;
		p_dpb_mask->n_client_mask = 0;
	break;
	case DDL_DPB_OP_INIT:
	{
		u32 n_dpb_size;
		n_dpb_size = (!p_decoder->n_meta_data_offset) ?
		p_decoder->dp_buf.a_dec_pic_buffers[0].vcd_frm.n_alloc_len :
			p_decoder->n_meta_data_offset;
	}
	break;
	case DDL_DPB_OP_RETRIEVE:
	{
		u32 n_position;
		if (p_dpb_mask->n_client_mask) {
			n_position = 0x1;
			for (n_loopc = 0; n_loopc <
				p_decoder->dp_buf.n_no_of_dec_pic_buf &&
				!p_found_frame; ++n_loopc) {
				if (p_dpb_mask->n_client_mask & n_position) {
					p_found_frame = &p_decoder->dp_buf.
						a_dec_pic_buffers[n_loopc];
					p_dpb_mask->n_client_mask &=
						~(n_position);
				}
				n_position <<= 1;
			}
		} else if (p_dpb_mask->n_hw_mask) {
			n_position = 0x1;
			for (n_loopc = 0; n_loopc <
				p_decoder->dp_buf.n_no_of_dec_pic_buf &&
				!p_found_frame; ++n_loopc) {
				if (p_dpb_mask->n_hw_mask & n_position) {
					p_found_frame = &p_decoder->dp_buf.
						a_dec_pic_buffers[n_loopc];
					p_dpb_mask->n_hw_mask &= ~(n_position);
				}
				n_position <<= 1;
			}
		}
		if (p_found_frame)
			*p_in_out_frame = *p_found_frame;
		else {
			p_in_out_frame->vcd_frm.p_physical = NULL;
			p_in_out_frame->vcd_frm.p_virtual = NULL;
		}
	}
	break;
	default:
	break;
	}
	return vcd_status;
}

u32 ddl_decoder_dpb_init(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context = p_ddl->p_ddl_context;
	struct ddl_decoder_data_type *p_decoder = &p_ddl->codec_data.decoder;
	struct ddl_dec_buffers_type *p_dec_buffers = &p_decoder->hw_bufs;
	struct ddl_frame_data_type_tag *p_frame;

	u32 n_luma_size, i, n_dpb;
	u32 a_luma[32], a_chroma[32], a_mv[32];

	p_frame = &p_decoder->dp_buf.a_dec_pic_buffers[0];
	n_luma_size = ddl_get_yuv_buf_size(p_decoder->frame_size.n_width,
			p_decoder->frame_size.n_height, DDL_YUV_BUF_TYPE_TILE);
	n_dpb = p_decoder->dp_buf.n_no_of_dec_pic_buf;

	for (i = 0; i < n_dpb; i++) {
		a_luma[i] = DDL_OFFSET(p_ddl_context->dram_base_a.
			p_align_physical_addr, p_frame[i].vcd_frm.p_physical);
		a_chroma[i] = a_luma[i] + n_luma_size;
	}
	switch (p_decoder->codec_type.e_codec) {
	case VCD_CODEC_MPEG1:
	case VCD_CODEC_MPEG2:
		vidc_1080p_set_decode_recon_buffers(n_dpb, a_luma, a_chroma);
	break;
	case VCD_CODEC_DIVX_3:
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
	case VCD_CODEC_XVID:
	case VCD_CODEC_MPEG4:
		vidc_1080p_set_decode_recon_buffers(n_dpb, a_luma, a_chroma);
		vidc_1080p_set_mpeg4_divx_decode_work_buffers(
		DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
			p_dec_buffers->nb_dcac),
		DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
			p_dec_buffers->upnb_mv),
		DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
			p_dec_buffers->sub_anchor_mv),
		DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
			p_dec_buffers->overlay_xform),
		DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
			p_dec_buffers->stx_parser));
	break;
	case VCD_CODEC_H263:
		vidc_1080p_set_decode_recon_buffers(n_dpb, a_luma, a_chroma);
		vidc_1080p_set_h263_decode_work_buffers(
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->nb_dcac),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->upnb_mv),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->sub_anchor_mv),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->overlay_xform));
	break;
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		vidc_1080p_set_decode_recon_buffers(n_dpb, a_luma, a_chroma);
		vidc_1080p_set_vc1_decode_work_buffers(
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->nb_dcac),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->upnb_mv),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->sub_anchor_mv),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->overlay_xform),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->bit_plane1),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->bit_plane2),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->bit_plane3));
	break;
	case VCD_CODEC_H264:
		for (i = 0; i < n_dpb; i++)
			a_mv[i] = DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
					p_dec_buffers->h264_mv[i]);
		vidc_1080p_set_h264_decode_buffers(n_dpb,
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->h264_vert_nb_mv),
			DDL_ADDR_OFFSET(p_ddl_context->dram_base_a,
				p_dec_buffers->h264_nb_ip),
			a_luma, a_chroma, a_mv);
	break;
	default:
	break;
	}
	return VCD_S_SUCCESS;
}

void ddl_release_context_buffers(struct ddl_context_type *p_ddl_context)
{
	ddl_pmem_free(&p_ddl_context->dram_base_a);
	ddl_pmem_free(&p_ddl_context->dram_base_b);
	ddl_fw_release();
}

void ddl_release_client_internal_buffers(struct ddl_client_context_type *p_ddl)
{
	if (p_ddl->b_decoding) {
		struct ddl_decoder_data_type *p_decoder =
			&(p_ddl->codec_data.decoder);
		kfree(p_decoder->dp_buf.a_dec_pic_buffers);
		p_decoder->dp_buf.a_dec_pic_buffers = NULL;
		ddl_vidc_decode_dynamic_property(p_ddl, FALSE);
		p_decoder->decode_config.n_sequence_header_len = 0;
		p_decoder->decode_config.p_sequence_header = NULL;
		p_decoder->dpb_mask.n_client_mask = 0;
		p_decoder->dpb_mask.n_hw_mask = 0;
		p_decoder->dp_buf.n_no_of_dec_pic_buf = 0;
		p_decoder->n_dynamic_prop_change = 0;
		ddl_free_dec_hw_buffers(p_ddl);
	} else {
		struct ddl_encoder_data_type *p_encoder =
			&(p_ddl->codec_data.encoder);
		ddl_pmem_free(&p_encoder->seq_header);
		ddl_vidc_encode_dynamic_property(p_ddl, FALSE);
		p_encoder->n_dynamic_prop_change = 0;
		ddl_free_enc_hw_buffers(p_ddl);
	}
}

u32 ddl_codec_type_transact(struct ddl_client_context_type *p_ddl,
	u32 b_remove, enum vcd_codec_type e_requested_codec)
{
	if (e_requested_codec > VCD_CODEC_VC1_RCV ||
		e_requested_codec < VCD_CODEC_H264)
		return FALSE;
	if (!p_ddl->b_decoding && e_requested_codec != VCD_CODEC_MPEG4 &&
		e_requested_codec != VCD_CODEC_H264 &&
		e_requested_codec != VCD_CODEC_H263)
		return FALSE;

	return TRUE;
}

u32 ddl_take_command_channel(struct ddl_context_type *p_ddl_context,
	struct ddl_client_context_type *p_ddl, void *p_client_data)
{
	u32  b_status = TRUE;

	if (!p_ddl_context->p_current_ddl[0]) {
		p_ddl_context->p_current_ddl[0] = p_ddl;
		p_ddl->p_client_data = p_client_data;
		p_ddl->n_command_channel = 0;
	} else if (!p_ddl_context->p_current_ddl[1]) {
		p_ddl_context->p_current_ddl[1] = p_ddl;
		p_ddl->p_client_data = p_client_data;
		p_ddl->n_command_channel = 1;
	} else
		b_status = FALSE;
	if (b_status) {
		if (p_ddl_context->p_current_ddl[0] &&
			p_ddl_context->p_current_ddl[1])
			DDL_BUSY(p_ddl_context);
		else
			DDL_RUN(p_ddl_context);
	}
	return b_status;
}

void ddl_release_command_channel(struct ddl_context_type *p_ddl_context,
	u32 n_command_channel)
{
	p_ddl_context->p_current_ddl[n_command_channel]->p_client_data = NULL;
	p_ddl_context->p_current_ddl[n_command_channel] = NULL;
	if (!p_ddl_context->p_current_ddl[0] &&
		!p_ddl_context->p_current_ddl[1])
		DDL_IDLE(p_ddl_context);
	else
		DDL_RUN(p_ddl_context);
}

struct ddl_client_context_type *ddl_get_current_ddl_client_for_channel_id(
	struct ddl_context_type *p_ddl_context, u32 n_channel_id)
{
	struct ddl_client_context_type *p_ddl;

	if (p_ddl_context->p_current_ddl[0] && n_channel_id ==
		p_ddl_context->p_current_ddl[0]->n_command_channel)
		p_ddl = p_ddl_context->p_current_ddl[0];
	else if (p_ddl_context->p_current_ddl[1] && n_channel_id ==
		p_ddl_context->p_current_ddl[1]->n_command_channel)
		p_ddl = p_ddl_context->p_current_ddl[1];
	else {
		DDL_MSG_LOW("STATE-CRITICAL-FRMRUN");
		DDL_MSG_ERROR("Unexpected channel ID = %d", n_channel_id);
		p_ddl = NULL;
	}
	return p_ddl;
}

struct ddl_client_context_type *ddl_get_current_ddl_client_for_command(
	struct ddl_context_type *p_ddl_context,
	enum ddl_cmd_state_type e_cmd_state)
{
	struct ddl_client_context_type *p_ddl;

	if (p_ddl_context->p_current_ddl[0] &&
		e_cmd_state == p_ddl_context->p_current_ddl[0]->e_cmd_state)
		p_ddl = p_ddl_context->p_current_ddl[0];
	else if (p_ddl_context->p_current_ddl[1] &&
		e_cmd_state == p_ddl_context->p_current_ddl[1]->e_cmd_state)
		p_ddl = p_ddl_context->p_current_ddl[1];
	else {
		DDL_MSG_LOW("STATE-CRITICAL-FRMRUN");
		DDL_MSG_ERROR("Error: Unexpected e_cmd_state = %d",
			e_cmd_state);
		p_ddl = NULL;
	}
	return p_ddl;
}

u32 ddl_get_yuv_buf_size(u32 width, u32 height, u32 format)
{
	u32 mem_size, width_round_up, height_round_up, align;

	width_round_up  = width;
	height_round_up = height;
	if (format == DDL_YUV_BUF_TYPE_TILE) {
		width_round_up  = DDL_ALIGN(width, DDL_TILE_ALIGN_WIDTH);
		height_round_up = DDL_ALIGN(height, DDL_TILE_ALIGN_HEIGHT);
		align = DDL_TILE_MULTIPLY_FACTOR;
	}
	if (format == DDL_YUV_BUF_TYPE_LINEAR) {
		width_round_up = DDL_ALIGN(width, DDL_LINEAR_ALIGN_WIDTH);
		align = DDL_LINEAR_MULTIPLY_FACTOR;
	}
	mem_size = (width_round_up * height_round_up);
	mem_size = DDL_ALIGN(mem_size, align);
	return mem_size;
}
void ddl_free_dec_hw_buffers(struct ddl_client_context_type *p_ddl)
{
	struct ddl_dec_buffers_type *p_dec_bufs =
		&p_ddl->codec_data.decoder.hw_bufs;
	u32 i;

	for (i = 0; i < 32; i++)
		ddl_pmem_free(&p_dec_bufs->h264_mv[i]);
	ddl_pmem_free(&p_dec_bufs->h264_nb_ip);
	ddl_pmem_free(&p_dec_bufs->h264_vert_nb_mv);
	ddl_pmem_free(&p_dec_bufs->nb_dcac);
	ddl_pmem_free(&p_dec_bufs->upnb_mv);
	ddl_pmem_free(&p_dec_bufs->sub_anchor_mv);
	ddl_pmem_free(&p_dec_bufs->overlay_xform);
	ddl_pmem_free(&p_dec_bufs->bit_plane3);
	ddl_pmem_free(&p_dec_bufs->bit_plane2);
	ddl_pmem_free(&p_dec_bufs->bit_plane1);
	ddl_pmem_free(&p_dec_bufs->stx_parser);
	ddl_pmem_free(&p_dec_bufs->desc);
	ddl_pmem_free(&p_dec_bufs->context);
	memset(p_dec_bufs, 0, sizeof(struct ddl_dec_buffers_type));
}

void ddl_free_enc_hw_buffers(struct ddl_client_context_type *p_ddl)
{
	struct ddl_enc_buffers_type *p_enc_bufs =
		&p_ddl->codec_data.encoder.hw_bufs;
	u32 i;

	for (i = 0; i < p_enc_bufs->n_dpb_count; i++) {
		ddl_pmem_free(&p_enc_bufs->dpb_y[i]);
		ddl_pmem_free(&p_enc_bufs->dpb_c[i]);
	}
	ddl_pmem_free(&p_enc_bufs->mv);
	ddl_pmem_free(&p_enc_bufs->col_zero);
	ddl_pmem_free(&p_enc_bufs->md);
	ddl_pmem_free(&p_enc_bufs->pred);
	ddl_pmem_free(&p_enc_bufs->nbor_info);
	ddl_pmem_free(&p_enc_bufs->acdc_coef);
	ddl_pmem_free(&p_enc_bufs->context);
	memset(p_enc_bufs, 0, sizeof(struct ddl_enc_buffers_type));
}

u32 ddl_get_input_frame_from_pool(struct ddl_client_context_type *p_ddl,
	u8 *p_input_buffer_address)
{
	u32 vcd_status = VCD_S_SUCCESS, i, b_found = FALSE;

	for (i = 0; i < DDL_MAX_NUM_IN_INPUTFRAME_POOL && !b_found; i++) {
		if (p_input_buffer_address ==
			p_ddl->input_frame_pool[i].vcd_frm.p_physical) {
			b_found = TRUE;
			p_ddl->input_frame = p_ddl->input_frame_pool[i];
			memset(&p_ddl->input_frame_pool[i], 0,
				sizeof(struct ddl_frame_data_type_tag));
		}
	}
	if (!b_found)
		vcd_status = VCD_ERR_FAIL;

	return vcd_status;
}

u32 ddl_insert_input_frame_to_pool(struct ddl_client_context_type *p_ddl,
	struct ddl_frame_data_type_tag *p_ddl_input_frame)
{
	u32 vcd_status = VCD_S_SUCCESS, i, b_found = FALSE;

	for (i = 0; i < DDL_MAX_NUM_IN_INPUTFRAME_POOL && !b_found; i++) {
		if (!p_ddl->input_frame_pool[i].vcd_frm.p_physical) {
			b_found = TRUE;
			p_ddl->input_frame_pool[i] = *p_ddl_input_frame;
		}
	}
	if (!b_found)
		vcd_status = VCD_ERR_FAIL;

	return vcd_status;
}

void ddl_calc_dec_hw_buffers_size(enum vcd_codec_type e_codec, u32 width,
	u32 height, u32 n_dpb, struct ddl_dec_buffer_size_type *p_buf_size)
{
	u32 sz_dpb0 = 0, sz_dpb1 = 0, sz_mv = 0;
	u32 sz_luma = 0, sz_chroma = 0, sz_nb_dcac = 0, sz_upnb_mv = 0;
	u32 sz_sub_anchor_mv = 0, sz_overlap_xform = 0, sz_bit_plane3 = 0;
	u32 sz_bit_plane2 = 0, sz_bit_plane1 = 0, sz_stx_parser = 0;
	u32 sz_desc, sz_cpb, sz_context, sz_vert_nb_mv = 0, sz_nb_ip = 0;

	if (e_codec == VCD_CODEC_H264) {
		sz_mv = ddl_get_yuv_buf_size(width,
			height>>2, DDL_YUV_BUF_TYPE_TILE);
		sz_nb_ip = DDL_KILO_BYTE(32);
		sz_vert_nb_mv = DDL_KILO_BYTE(16);
	} else {
		if ((e_codec == VCD_CODEC_MPEG4) ||
			(e_codec == VCD_CODEC_DIVX_3) ||
			(e_codec == VCD_CODEC_DIVX_4) ||
			(e_codec == VCD_CODEC_DIVX_5) ||
			(e_codec == VCD_CODEC_DIVX_6) ||
			(e_codec == VCD_CODEC_XVID) ||
			(e_codec == VCD_CODEC_H263)) {
			sz_nb_dcac = DDL_KILO_BYTE(16);
			sz_upnb_mv = DDL_KILO_BYTE(68);
			sz_sub_anchor_mv = DDL_KILO_BYTE(136);
			sz_overlap_xform = DDL_KILO_BYTE(32);
			if (e_codec != VCD_CODEC_H263)
				sz_stx_parser = DDL_KILO_BYTE(68);
		} else if ((e_codec == VCD_CODEC_VC1) ||
			(e_codec == VCD_CODEC_VC1_RCV)) {
			sz_nb_dcac = DDL_KILO_BYTE(16);
			sz_upnb_mv = DDL_KILO_BYTE(68);
			sz_sub_anchor_mv = DDL_KILO_BYTE(136);
			sz_overlap_xform = DDL_KILO_BYTE(32);
			sz_bit_plane3 = DDL_KILO_BYTE(2);
			sz_bit_plane2 = DDL_KILO_BYTE(2);
			sz_bit_plane1 = DDL_KILO_BYTE(2);
		}
	}
	sz_desc = DDL_KILO_BYTE(128);
	sz_cpb = VCD_DEC_CPB_SIZE;
	if (e_codec == VCD_CODEC_H264)
		sz_context = DDL_FW_H264DEC_CONTEXT_SPACE_SIZE;
	else
		sz_context = DDL_FW_OTHER_CONTEXT_SPACE_SIZE;
	if (p_buf_size) {
		p_buf_size->sz_dpb0           = sz_dpb0;
		p_buf_size->sz_dpb1           = sz_dpb1;
		p_buf_size->sz_mv             = sz_mv;
		p_buf_size->sz_vert_nb_mv     = sz_vert_nb_mv;
		p_buf_size->sz_nb_ip          = sz_nb_ip;
		p_buf_size->sz_luma           = sz_luma;
		p_buf_size->sz_chroma         = sz_chroma;
		p_buf_size->sz_nb_dcac        = sz_nb_dcac;
		p_buf_size->sz_upnb_mv        = sz_upnb_mv;
		p_buf_size->sz_sub_anchor_mv  = sz_sub_anchor_mv;
		p_buf_size->sz_overlap_xform  = sz_overlap_xform;
		p_buf_size->sz_bit_plane3     = sz_bit_plane3;
		p_buf_size->sz_bit_plane2     = sz_bit_plane2;
		p_buf_size->sz_bit_plane1     = sz_bit_plane1;
		p_buf_size->sz_stx_parser     = sz_stx_parser;
		p_buf_size->sz_desc           = sz_desc;
		p_buf_size->sz_cpb            = sz_cpb;
		p_buf_size->sz_context        = sz_context;
	}
}

u32 ddl_allocate_h264_dec_mv_buffer(struct ddl_client_context_type *p_ddl)
{
	struct ddl_dec_buffers_type *p_dec_bufs;
	u32 status = VCD_S_SUCCESS, sz_mv, n_dpb, width, height, i;
	u8 *ptr;

	p_dec_bufs = &p_ddl->codec_data.decoder.hw_bufs;
	n_dpb = p_ddl->codec_data.decoder.dp_buf.n_no_of_dec_pic_buf;
	if (p_ddl->codec_data.decoder.frame_size.n_width > 0) {
		width = p_ddl->codec_data.decoder.frame_size.n_width;
		height = p_ddl->codec_data.decoder.frame_size.n_height;
	} else {
		width = p_ddl->codec_data.decoder.client_frame_size.n_width;
		height = p_ddl->codec_data.decoder.client_frame_size.n_height;
	}
	sz_mv = ddl_get_yuv_buf_size(width, height>>2, DDL_YUV_BUF_TYPE_TILE);
	if (sz_mv > 0) {
		for (i = 0; i < n_dpb; i++) {
			ptr = ddl_pmem_alloc(&p_dec_bufs->h264_mv[i],
				sz_mv, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
		}
	}
	return status;
}

u32 ddl_allocate_dec_hw_buffers(struct ddl_client_context_type *p_ddl)
{
	struct ddl_dec_buffers_type *p_dec_bufs;
	struct ddl_dec_buffer_size_type buf_size;
	u32 status = VCD_S_SUCCESS, n_dpb = 0;
	u32 width = 0, height = 0;
	u8 *ptr;

	p_dec_bufs = &p_ddl->codec_data.decoder.hw_bufs;
	ddl_calc_dec_hw_buffers_size(p_ddl->codec_data.decoder.
		codec_type.e_codec, width, height, n_dpb, &buf_size);
	if (buf_size.sz_context > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->context, buf_size.sz_context,
			DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_nb_ip > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->h264_nb_ip, buf_size.sz_nb_ip,
			DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_vert_nb_mv > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->h264_vert_nb_mv,
			buf_size.sz_vert_nb_mv, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_nb_dcac > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->nb_dcac, buf_size.sz_nb_dcac,
			DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_upnb_mv > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->upnb_mv, buf_size.sz_upnb_mv,
			DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_sub_anchor_mv > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->sub_anchor_mv,
			buf_size.sz_sub_anchor_mv, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_overlap_xform > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->overlay_xform,
			buf_size.sz_overlap_xform, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_bit_plane3 > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->bit_plane3,
			buf_size.sz_bit_plane3, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_bit_plane2 > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->bit_plane2,
			buf_size.sz_bit_plane2, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_bit_plane1 > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->bit_plane1,
			buf_size.sz_bit_plane1, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_stx_parser > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->stx_parser,
			buf_size.sz_stx_parser, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (buf_size.sz_desc > 0) {
		ptr = ddl_pmem_alloc(&p_dec_bufs->desc, buf_size.sz_desc,
			DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
	}
	if (status)
		ddl_free_dec_hw_buffers(p_ddl);
	return status;
}

u32 ddl_calc_enc_hw_buffers_size(enum vcd_codec_type e_codec, u32 width,
	u32 height, enum vcd_yuv_buffer_format_type e_input_format,
	struct ddl_client_context_type *p_ddl,
	struct ddl_enc_buffer_size_type *p_buf_size)
{
	u32 status = VCD_S_SUCCESS, n_mb_x, n_mb_y;
	u32 sz_cur_y, sz_cur_c, sz_dpb_y, sz_dpb_c, sz_strm = 0, sz_mv;
	u32 sz_md = 0, sz_pred = 0, sz_nbor_info = 0 , sz_acdc_coef = 0;
	u32 sz_mb_info = 0, sz_context, sz_col_zero = 0;

	n_mb_x = (width + 15) / 16;
	n_mb_y = (height + 15) / 16;
	sz_dpb_y = ddl_get_yuv_buf_size(width,
		height, DDL_YUV_BUF_TYPE_TILE);
	sz_dpb_c = ddl_get_yuv_buf_size(width, height>>1,
		DDL_YUV_BUF_TYPE_TILE);
	if (e_input_format ==
		VCD_BUFFER_FORMAT_NV12_16M2KA) {
		sz_cur_y = ddl_get_yuv_buf_size(width, height,
			DDL_YUV_BUF_TYPE_LINEAR);
		sz_cur_c = ddl_get_yuv_buf_size(width, height>>1,
			DDL_YUV_BUF_TYPE_LINEAR);
	} else if (VCD_BUFFER_FORMAT_TILE_4x2 == e_input_format) {
		sz_cur_y = sz_dpb_y;
		sz_cur_c = sz_dpb_c;
	} else
		status = VCD_ERR_NOT_SUPPORTED;
	if (!status) {
		sz_strm = DDL_ALIGN(ddl_get_yuv_buf_size(width, height,
			DDL_YUV_BUF_TYPE_LINEAR) + ddl_get_yuv_buf_size(width,
			height/2, DDL_YUV_BUF_TYPE_LINEAR), DDL_KILO_BYTE(4));
		sz_mv = DDL_ALIGN(2 * n_mb_x * 8, DDL_KILO_BYTE(2));
		if ((e_codec == VCD_CODEC_MPEG4) ||
			(e_codec == VCD_CODEC_H264)) {
			sz_col_zero = DDL_ALIGN(((n_mb_x * n_mb_y + 7) / 8) *
					8, DDL_KILO_BYTE(2));
		}
		if ((e_codec == VCD_CODEC_MPEG4) ||
			(e_codec == VCD_CODEC_H263)) {
			sz_acdc_coef = DDL_ALIGN((width / 2) * 8,
						DDL_KILO_BYTE(2));
		} else if (e_codec == VCD_CODEC_H264) {
			sz_md = DDL_ALIGN(n_mb_x * 48, DDL_KILO_BYTE(2));
			sz_pred = DDL_ALIGN(2 * 8 * 1024, DDL_KILO_BYTE(2));
			if (p_ddl) {
				if (p_ddl->codec_data.encoder.
					entropy_control.e_entropy_sel ==
					VCD_ENTROPY_SEL_CAVLC)
					sz_nbor_info = DDL_ALIGN(8 * 8 * n_mb_x,
						DDL_KILO_BYTE(2));
				else if (p_ddl->codec_data.encoder.
					entropy_control.e_entropy_sel ==
					VCD_ENTROPY_SEL_CABAC)
					sz_nbor_info = DDL_ALIGN(8 * 24 *
						n_mb_x, DDL_KILO_BYTE(2));
				if ((p_ddl->codec_data.encoder.
					b_mb_info_enable) &&
					(e_codec == VCD_CODEC_H264)) {
					sz_mb_info = DDL_ALIGN(n_mb_x * n_mb_y *
						6 * 8, DDL_KILO_BYTE(2));
				}
			}
		} else {
			sz_nbor_info = DDL_ALIGN(8 * 24 * n_mb_x,
						DDL_KILO_BYTE(2));
			sz_mb_info = DDL_ALIGN(n_mb_x * n_mb_y * 6 * 8,
					DDL_KILO_BYTE(2));
		}
		sz_context = DDL_FW_OTHER_CONTEXT_SPACE_SIZE;
		if (p_buf_size) {
			p_buf_size->sz_cur_y      = sz_cur_y;
			p_buf_size->sz_cur_c      = sz_cur_c;
			p_buf_size->sz_dpb_y      = sz_dpb_y;
			p_buf_size->sz_dpb_c      = sz_dpb_c;
			p_buf_size->sz_strm       = sz_strm;
			p_buf_size->sz_mv         = sz_mv;
			p_buf_size->sz_col_zero   = sz_col_zero;
			p_buf_size->sz_md         = sz_md;
			p_buf_size->sz_pred       = sz_pred;
			p_buf_size->sz_nbor_info  = sz_nbor_info;
			p_buf_size->sz_acdc_coef  = sz_acdc_coef;
			p_buf_size->sz_mb_info    = sz_mb_info;
			p_buf_size->sz_context    = sz_context;
		}
	}
	return status;
}

u32 ddl_allocate_enc_hw_buffers(struct ddl_client_context_type *p_ddl)
{
	struct ddl_enc_buffers_type *p_enc_bufs;
	struct ddl_enc_buffer_size_type buf_size;
	void *ptr;
	u32 status = VCD_S_SUCCESS, i, size;

	p_enc_bufs = &p_ddl->codec_data.encoder.hw_bufs;
	p_enc_bufs->n_dpb_count = DDL_ENC_MIN_DPB_BUFFERS;

	if ((p_ddl->codec_data.encoder.i_period.n_b_frames >
		DDL_MIN_NUM_OF_B_FRAME) ||
		(p_ddl->codec_data.encoder.n_num_references_for_p_frame
		> DDL_MIN_NUM_REF_FOR_P_FRAME))
		p_enc_bufs->n_dpb_count = DDL_ENC_MAX_DPB_BUFFERS;
		DDL_MSG_HIGH("Encoder num DPB buffers allocated = %d",
			p_enc_bufs->n_dpb_count);

	status = ddl_calc_enc_hw_buffers_size(
		p_ddl->codec_data.encoder.codec_type.e_codec,
		p_ddl->codec_data.encoder.frame_size.n_width,
		p_ddl->codec_data.encoder.frame_size.n_height,
		p_ddl->codec_data.encoder.buf_format.e_buffer_format,
		p_ddl, &buf_size);
	buf_size.sz_strm = p_ddl->codec_data.encoder.
		client_output_buf_req.n_size;
	if (!status) {
		if (buf_size.sz_dpb_y > 0 && buf_size.sz_dpb_c > 0) {
			size = buf_size.sz_dpb_y + buf_size.sz_dpb_c;
			for (i = 0; i < p_enc_bufs->n_dpb_count; i++) {
				ptr = ddl_pmem_alloc(&p_enc_bufs->dpb_y[i],
					size, DDL_KILO_BYTE(8));
				p_enc_bufs->dpb_c[i].p_align_virtual_addr  =
				p_enc_bufs->dpb_y[i].p_align_virtual_addr +
					buf_size.sz_dpb_y;
				p_enc_bufs->dpb_c[i].p_align_physical_addr =
				p_enc_bufs->dpb_y[i].p_align_physical_addr +
					buf_size.sz_dpb_y;
				if (!ptr)
					status = VCD_ERR_ALLOC_FAIL;
			}
		}
		if (!status) {
			for (i = p_enc_bufs->n_dpb_count; i <
				DDL_ENC_MAX_DPB_BUFFERS; i++) {
				p_enc_bufs->dpb_y[i] = p_enc_bufs->dpb_y
					[i - p_enc_bufs->n_dpb_count];
				p_enc_bufs->dpb_c[i] = p_enc_bufs->dpb_y
					[i - p_enc_bufs->n_dpb_count];
			}
		}
		p_enc_bufs->sz_dpb_y = buf_size.sz_dpb_y;
		p_enc_bufs->sz_dpb_c = buf_size.sz_dpb_c;
		if (buf_size.sz_mv > 0) {
			ptr = ddl_pmem_alloc(&p_enc_bufs->mv, buf_size.sz_mv,
				DDL_KILO_BYTE(2));
			if (!ptr)
				status = VCD_ERR_ALLOC_FAIL;
		}
		if (buf_size.sz_col_zero > 0) {
			ptr = ddl_pmem_alloc(&p_enc_bufs->col_zero,
				buf_size.sz_col_zero, DDL_KILO_BYTE(2));
		if (!ptr)
			status = VCD_ERR_ALLOC_FAIL;
		}
		if (buf_size.sz_md > 0) {
			ptr = ddl_pmem_alloc(&p_enc_bufs->md, buf_size.sz_md,
				DDL_KILO_BYTE(2));
			if (!ptr)
				status = VCD_ERR_ALLOC_FAIL;
		}
		if (buf_size.sz_pred > 0) {
			ptr = ddl_pmem_alloc(&p_enc_bufs->pred,
				buf_size.sz_pred, DDL_KILO_BYTE(2));
			if (!ptr)
				status = VCD_ERR_ALLOC_FAIL;
		}
		if (buf_size.sz_nbor_info > 0) {
			ptr = ddl_pmem_alloc(&p_enc_bufs->nbor_info,
				buf_size.sz_nbor_info, DDL_KILO_BYTE(2));
			if (!ptr)
				status = VCD_ERR_ALLOC_FAIL;
		}
		if (buf_size.sz_acdc_coef > 0) {
			ptr = ddl_pmem_alloc(&p_enc_bufs->acdc_coef,
				buf_size.sz_acdc_coef, DDL_KILO_BYTE(2));
			if (!ptr)
				status = VCD_ERR_ALLOC_FAIL;
		}
		if (buf_size.sz_mb_info > 0) {
			ptr = ddl_pmem_alloc(&p_enc_bufs->mb_info,
				buf_size.sz_mb_info, DDL_KILO_BYTE(2));
			if (!ptr)
				status = VCD_ERR_ALLOC_FAIL;
		}
		if (buf_size.sz_context > 0) {
			ptr = ddl_pmem_alloc(&p_enc_bufs->context,
				buf_size.sz_context, DDL_KILO_BYTE(2));
			if (!ptr)
				status = VCD_ERR_ALLOC_FAIL;
		}
		if (status)
			ddl_free_enc_hw_buffers(p_ddl);
	}
	return status;
}

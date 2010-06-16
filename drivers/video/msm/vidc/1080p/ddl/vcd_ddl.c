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

u32 ddl_device_init(struct ddl_init_config_type *p_ddl_init_config,
	void *p_client_data)
{
	struct ddl_context_type *p_ddl_context;
	u32 status = VCD_S_SUCCESS;
	void *ptr;
	DDL_MSG_HIGH("ddl_device_init");

	if ((!p_ddl_init_config) || (!p_ddl_init_config->ddl_callback) ||
		(!p_ddl_init_config->p_core_virtual_base_addr)) {
		DDL_MSG_ERROR("ddl_dev_init:Bad_argument");
		return VCD_ERR_ILLEGAL_PARM;
	}
	p_ddl_context = ddl_get_context();
	if (DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dev_init:Multiple_init");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!DDL_IS_IDLE(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dev_init:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	memset(p_ddl_context, 0, sizeof(struct ddl_context_type));
	DDL_BUSY(p_ddl_context);
	p_ddl_context->ddl_callback = p_ddl_init_config->ddl_callback;
	if (p_ddl_init_config->pf_interrupt_clr)
		p_ddl_context->pf_interrupt_clr =
			p_ddl_init_config->pf_interrupt_clr;
	p_ddl_context->p_core_virtual_base_addr =
		p_ddl_init_config->p_core_virtual_base_addr;
	p_ddl_context->p_client_data = p_client_data;
	p_ddl_context->ddl_hw_response.arg1 = DDL_INVALID_INTR_STATUS;
	DDL_MSG_LOW("%s() : virtual address of core(%x)\n", __func__,
		(u32) p_ddl_init_config->p_core_virtual_base_addr);
	vidc_1080p_set_device_base_addr(
		p_ddl_context->p_core_virtual_base_addr);
	p_ddl_context->e_cmd_state =	DDL_CMD_INVALID;
	ddl_client_transact(DDL_INIT_CLIENTS, NULL);
	p_ddl_context->n_fw_ctxt_memory_size = DDL_FW_GLOVIDC_CONTEXT_SIZE;
	p_ddl_context->n_fw_memory_size = p_ddl_context->n_fw_ctxt_memory_size +
		DDL_FW_INST_SPACE_SIZE;
	ptr = ddl_pmem_alloc(&p_ddl_context->dram_base_a,
		p_ddl_context->n_fw_memory_size, DDL_KILO_BYTE(128));
	if (!ptr) {
		DDL_MSG_ERROR("Memory Aocation Failed for FW Base");
		status = VCD_ERR_ALLOC_FAIL;
	} else {
		DDL_MSG_LOW("%s() : physical address of base(%x)\n",
			 __func__, (u32) p_ddl_context->dram_base_a.\
			p_align_physical_addr);
		p_ddl_context->dram_base_b.p_align_physical_addr =
			p_ddl_context->dram_base_a.p_align_physical_addr;
		p_ddl_context->dram_base_b.p_align_virtual_addr  =
			p_ddl_context->dram_base_a.p_align_virtual_addr;
	}
	if (!status && !ddl_fw_init(&p_ddl_context->dram_base_a)) {
		DDL_MSG_ERROR("ddl_dev_init:fw_init_failed");
		status = VCD_ERR_ALLOC_FAIL;
	}
	if (!status) {
		p_ddl_context->e_cmd_state = DDL_CMD_DMA_INIT;
		ddl_vidc_core_init(p_ddl_context);
	} else {
		ddl_release_context_buffers(p_ddl_context);
		DDL_IDLE(p_ddl_context);
	}
	return status;
}

u32 ddl_device_release(void *p_client_data)
{
	struct ddl_context_type *p_ddl_context;

	DDL_MSG_HIGH("ddl_device_release");
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_IDLE(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dev_rel:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dev_rel:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!ddl_client_transact(DDL_ACTIVE_CLIENT, NULL)) {
		DDL_MSG_ERROR("ddl_dev_rel:Client_present_err");
		return VCD_ERR_CLIENT_PRESENT;
	}
	DDL_BUSY(p_ddl_context);
	p_ddl_context->n_device_state = DDL_DEVICE_NOTINIT;
	p_ddl_context->p_client_data = p_client_data;
	p_ddl_context->e_cmd_state = DDL_CMD_INVALID;
	ddl_vidc_core_term(p_ddl_context);
	DDL_MSG_LOW("FW_ENDDONE");
	p_ddl_context->p_core_virtual_base_addr = NULL;
	ddl_release_context_buffers(p_ddl_context);
	DDL_IDLE(p_ddl_context);
	return VCD_S_SUCCESS;
}

u32 ddl_open(u32 **p_ddl_handle, u32 b_decoding)
{
	struct ddl_context_type *p_ddl_context;
	struct ddl_client_context_type *p_ddl;
	void *ptr;
	u32 status;

	DDL_MSG_HIGH("ddl_open");
	if (!p_ddl_handle) {
		DDL_MSG_ERROR("ddl_open:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_open:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	status = ddl_client_transact(DDL_GET_CLIENT, &p_ddl);
	if (status) {
		DDL_MSG_ERROR("ddl_open:Client_trasac_failed");
		return status;
	}
	ptr = ddl_pmem_alloc(&p_ddl->shared_mem[0],
			DDL_FW_AUX_HOST_CMD_SPACE_SIZE, sizeof(u32));
	if (!ptr)
		status = VCD_ERR_ALLOC_FAIL;
	if (!status) {
		ptr = ddl_pmem_alloc(&p_ddl->shared_mem[1],
				DDL_FW_AUX_HOST_CMD_SPACE_SIZE, sizeof(u32));
		if (!ptr) {
			ddl_pmem_free(&p_ddl->shared_mem[0]);
			status = VCD_ERR_ALLOC_FAIL;
		}
	}
	if (!status) {
		memset(p_ddl->shared_mem[0].p_align_virtual_addr, 0,
			DDL_FW_AUX_HOST_CMD_SPACE_SIZE);
		memset(p_ddl->shared_mem[1].p_align_virtual_addr, 0,
			DDL_FW_AUX_HOST_CMD_SPACE_SIZE);
		DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_OPEN",
		ddl_get_state_string(p_ddl->e_client_state));
		p_ddl->e_client_state = DDL_CLIENT_OPEN;
		p_ddl->codec_data.hdr.b_decoding = b_decoding;
		p_ddl->b_decoding = b_decoding;
		ddl_set_initial_default_values(p_ddl);
		*p_ddl_handle	= (u32 *) p_ddl;
	}
	return status;
}

u32 ddl_close(u32 **p_ddl_handle)
{
	struct ddl_context_type *p_ddl_context;
	struct ddl_client_context_type **pp_ddl =
		(struct ddl_client_context_type **)p_ddl_handle;

	DDL_MSG_HIGH("ddl_close");
	if (!pp_ddl || !*pp_ddl) {
		DDL_MSG_ERROR("ddl_close:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_close:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!DDLCLIENT_STATE_IS(*pp_ddl, DDL_CLIENT_OPEN)) {
		DDL_MSG_ERROR("ddl_close:Not_in_open_state");
		return VCD_ERR_ILLEGAL_OP;
	}
	ddl_pmem_free(&(*pp_ddl)->shared_mem[0]);
	ddl_pmem_free(&(*pp_ddl)->shared_mem[1]);
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_INVALID",
	ddl_get_state_string((*pp_ddl)->e_client_state));
	(*pp_ddl)->e_client_state = DDL_CLIENT_INVALID;
	ddl_codec_type_transact(*pp_ddl, TRUE, (enum vcd_codec_type)0);
	ddl_client_transact(DDL_FREE_CLIENT, pp_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_encode_start(u32 *ddl_handle, void *p_client_data)
{
	struct ddl_client_context_type *p_ddl =
		(struct ddl_client_context_type *) ddl_handle;
	struct ddl_context_type *p_ddl_context;
	struct ddl_encoder_data_type *p_encoder;
	void *ptr;
	u32 status = VCD_S_SUCCESS;

	DDL_MSG_HIGH("ddl_encode_start");
#ifdef DDL_PROFILE
	ddl_reset_time_variables(1);
#endif
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_enc_start:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_enc_start:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || p_ddl->b_decoding) {
		DDL_MSG_ERROR("ddl_enc_start:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN)) {
		DDL_MSG_ERROR("ddl_enc_start:Not_opened");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!ddl_encoder_ready_to_start(p_ddl)) {
		DDL_MSG_ERROR("ddl_enc_start:Err_param_settings");
		return VCD_ERR_ILLEGAL_OP;
	}
	p_encoder = &p_ddl->codec_data.encoder;
	status = ddl_allocate_enc_hw_buffers(p_ddl);
	if (status)
		return status;
#ifdef DDL_BUF_LOG
	ddl_list_buffers(p_ddl);
#endif
	if ((p_encoder->codec_type.e_codec == VCD_CODEC_MPEG4 &&
		!p_encoder->short_header.b_short_header) ||
		p_encoder->codec_type.e_codec == VCD_CODEC_H264) {
		ptr = ddl_pmem_alloc(&p_encoder->seq_header,
			DDL_ENC_SEQHEADER_SIZE, DDL_LINEAR_BUFFER_ALIGN_BYTES);
		if (!ptr) {
			ddl_free_enc_hw_buffers(p_ddl);
			DDL_MSG_ERROR("ddl_enc_start:Seq_hdr_alloc_failed");
			return VCD_ERR_ALLOC_FAIL;
		}
	} else {
		p_encoder->seq_header.n_buffer_size = 0;
		p_encoder->seq_header.p_virtual_base_addr = 0;
		p_encoder->seq_header.p_align_physical_addr = 0;
		p_encoder->seq_header.p_align_virtual_addr = 0;
	}
	if (!ddl_take_command_channel(p_ddl_context, p_ddl, p_client_data))
		return VCD_ERR_BUSY;
	ddl_vidc_channel_set(p_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_decode_start(u32 *ddl_handle, struct vcd_sequence_hdr_type *p_header,
	void *p_client_data)
{
	struct ddl_client_context_type  *p_ddl =
		(struct ddl_client_context_type *) ddl_handle;
	struct ddl_context_type *p_ddl_context;
	struct ddl_decoder_data_type *p_decoder;
	u32 status = VCD_S_SUCCESS;

	DDL_MSG_HIGH("ddl_decode_start");
#ifdef DDL_PROFILE
	ddl_reset_time_variables(0);
#endif
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dec_start:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dec_start:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || !p_ddl->b_decoding) {
		DDL_MSG_ERROR("ddl_dec_start:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_OPEN)) {
		DDL_MSG_ERROR("ddl_dec_start:Not_in_opened_state");
		return VCD_ERR_ILLEGAL_OP;
	}

	if ((p_header) && ((!p_header->n_sequence_header_len) ||
		(!p_header->p_sequence_header))) {
		DDL_MSG_ERROR("ddl_dec_start:Bad_param_seq_header");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if (!ddl_decoder_ready_to_start(p_ddl, p_header)) {
		DDL_MSG_ERROR("ddl_dec_start:Err_param_settings");
		return VCD_ERR_ILLEGAL_OP;
	}
	p_decoder = &p_ddl->codec_data.decoder;
	status = ddl_allocate_dec_hw_buffers(p_ddl);
	if (status)
		return status;
#ifdef DDL_BUF_LOG
	ddl_list_buffers(p_ddl);
#endif
	if (!ddl_take_command_channel(p_ddl_context, p_ddl, p_client_data))
		return VCD_ERR_BUSY;
	if (p_header) {
		p_decoder->b_header_in_start = TRUE;
		p_decoder->decode_config = *p_header;
	} else {
		p_decoder->b_header_in_start = FALSE;
		p_decoder->decode_config.n_sequence_header_len = 0;
		ddl_set_default_decoder_buffer_req(p_decoder, TRUE);
	}
	ddl_vidc_channel_set(p_ddl);
	return status;
}

u32 ddl_decode_frame(u32 *ddl_handle,
	struct ddl_frame_data_type_tag *p_input_bits, void *p_client_data)
{
	u32 vcd_status = VCD_S_SUCCESS;
	struct ddl_client_context_type *p_ddl =
		(struct ddl_client_context_type *) ddl_handle;
	struct ddl_context_type *p_ddl_context;
	struct ddl_decoder_data_type *p_decoder;

	DDL_MSG_HIGH("ddl_decode_frame");
#ifdef DDL_PROFILE
	ddl_get_core_start_time(0);
#endif
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dec_frame:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dec_frame:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || !p_ddl->b_decoding) {
		DDL_MSG_ERROR("ddl_dec_frame:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!p_input_bits || ((!p_input_bits->vcd_frm.p_physical ||
		!p_input_bits->vcd_frm.n_data_len) &&
		(!(VCD_FRAME_FLAG_EOS &	p_input_bits->vcd_frm.n_flags)))) {
		DDL_MSG_ERROR("ddl_dec_frame:Bad_input_param");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPB)) {
		DDL_MSG_ERROR("Dec_frame:Wrong_state");
		return VCD_ERR_ILLEGAL_OP;
	}
	p_decoder = &(p_ddl->codec_data.decoder);
	if (DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC)	&&
		!p_ddl->codec_data.decoder.dp_buf.n_no_of_dec_pic_buf) {
		DDL_MSG_ERROR("ddl_dec_frame:Dpbs_requied");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!ddl_take_command_channel(p_ddl_context, p_ddl, p_client_data))
		return VCD_ERR_BUSY;

	p_ddl->input_frame = *p_input_bits;
	if (DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME))
		ddl_vidc_decode_frame_run(p_ddl);
	else {
		if (!p_ddl->codec_data.decoder.dp_buf.n_no_of_dec_pic_buf) {
			DDL_MSG_ERROR("ddl_dec_frame:Dpbs_requied");
			vcd_status = VCD_ERR_ILLEGAL_OP;
		} else if (DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPB)) {
			vcd_status = ddl_vidc_decode_set_buffers(p_ddl);
		if (vcd_status)
			ddl_release_command_channel(p_ddl_context,
				p_ddl->n_command_channel);
		} else if (DDLCLIENT_STATE_IS(p_ddl,
			DDL_CLIENT_WAIT_FOR_INITCODEC)) {
			if (p_decoder->codec_type.e_codec == VCD_CODEC_DIVX_3) {
				if ((!p_decoder->client_frame_size.n_width) ||
				(!p_decoder->client_frame_size.n_height))
					return VCD_ERR_ILLEGAL_OP;
		}
		p_ddl->codec_data.decoder.decode_config.p_sequence_header =
			p_ddl->input_frame.vcd_frm.p_physical;
		p_ddl->codec_data.decoder.decode_config.n_sequence_header_len =
			p_ddl->input_frame.vcd_frm.n_data_len;
		ddl_vidc_decode_init_codec(p_ddl);
		} else {
			DDL_MSG_ERROR("Dec_frame:Wrong_state");
			vcd_status = VCD_ERR_ILLEGAL_OP;
		}
		if (vcd_status)
			DDL_IDLE(p_ddl_context);
		}
	return vcd_status;
}

u32 ddl_encode_frame(u32 *ddl_handle,
	struct ddl_frame_data_type_tag *p_input_frame,
	struct ddl_frame_data_type_tag *p_output_bit, void *p_client_data)
{
	struct ddl_client_context_type *p_ddl =
		(struct ddl_client_context_type *) ddl_handle;
	struct ddl_context_type *p_ddl_context;
	u32 vcd_status = VCD_S_SUCCESS;

	DDL_MSG_HIGH("ddl_encode_frame");
#ifdef DDL_PROFILE
	ddl_get_core_start_time(1);
#endif
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_enc_frame:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_enc_frame:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || p_ddl->b_decoding) {
		DDL_MSG_ERROR("ddl_enc_frame:Bad_handle");
	return VCD_ERR_BAD_HANDLE;
	}
	if (!p_input_frame || !p_input_frame->vcd_frm.p_physical	||
		!p_input_frame->vcd_frm.n_data_len) {
		DDL_MSG_ERROR("ddl_enc_frame:Bad_input_params");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if ((((u32) p_input_frame->vcd_frm.p_physical +
		p_input_frame->vcd_frm.n_offset) &
		(DDL_STREAMBUF_ALIGN_GUARD_BYTES))) {
		DDL_MSG_ERROR("ddl_enc_frame:Un_aligned_yuv_start_address");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if (!p_output_bit || !p_output_bit->vcd_frm.p_physical ||
		!p_output_bit->vcd_frm.n_alloc_len) {
		DDL_MSG_ERROR("ddl_enc_frame:Bad_output_params");
		return VCD_ERR_ILLEGAL_PARM;
	}
	if ((p_ddl->codec_data.encoder.output_buf_req.n_size +
		p_output_bit->vcd_frm.n_offset) >
		p_output_bit->vcd_frm.n_alloc_len)
		DDL_MSG_ERROR("ddl_enc_frame:n_offset_large,\
			Exceeds_min_buf_size");
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME)) {
		DDL_MSG_ERROR("ddl_enc_frame:Wrong_state");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!ddl_take_command_channel(p_ddl_context, p_ddl, p_client_data))
		return VCD_ERR_BUSY;

	p_ddl->input_frame = *p_input_frame;
	p_ddl->output_frame = *p_output_bit;
	if (p_ddl->codec_data.encoder.i_period.n_b_frames > 0) {
		if (!p_ddl->n_b_count) {
			p_ddl->b_first_output_frame = *p_output_bit;
			p_ddl->n_b_count++;
		} else if (p_ddl->codec_data.encoder.i_period.n_b_frames >=
			p_ddl->n_b_count) {
			p_ddl->extra_output_frame[p_ddl->n_b_count-1] =
				*p_output_bit;
			p_ddl->output_frame = p_ddl->b_first_output_frame;
			p_ddl->n_b_count++;
		}
	}
	ddl_insert_input_frame_to_pool(p_ddl, p_input_frame);
	if (!vcd_status)
		ddl_vidc_encode_frame_run(p_ddl);
	else
		DDL_MSG_ERROR("insert to frame pool failed %u", vcd_status);
	return vcd_status;
}

u32 ddl_decode_end(u32 *ddl_handle, void *p_client_data)
{
	struct ddl_client_context_type *p_ddl =
		(struct ddl_client_context_type *) ddl_handle;
	struct ddl_context_type *p_ddl_context;

	DDL_MSG_HIGH("ddl_decode_end");
#ifdef DDL_PROFILE
	ddl_reset_time_variables(0);
#endif
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dec_end:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_dec_end:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || !p_ddl->b_decoding) {
		DDL_MSG_ERROR("ddl_dec_end:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_DPB) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_FAVIDC_ERROR)) {
		DDL_MSG_ERROR("ddl_dec_end:Wrong_state");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!ddl_take_command_channel(p_ddl_context, p_ddl, p_client_data))
		return VCD_ERR_BUSY;
	ddl_vidc_channel_end(p_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_encode_end(u32 *ddl_handle, void *p_client_data)
{
	struct ddl_client_context_type  *p_ddl =
		(struct ddl_client_context_type *) ddl_handle;
	struct ddl_context_type *p_ddl_context;

	DDL_MSG_HIGH("ddl_encode_end");
#ifdef DDL_PROFILE
	ddl_reset_time_variables(1);
#endif
	p_ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_enc_end:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (DDL_IS_BUSY(p_ddl_context)) {
		DDL_MSG_ERROR("ddl_enc_end:Ddl_busy");
		return VCD_ERR_BUSY;
	}
	if (!p_ddl || p_ddl->b_decoding) {
		DDL_MSG_ERROR("ddl_enc_end:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_FRAME) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_WAIT_FOR_INITCODEC) &&
		!DDLCLIENT_STATE_IS(p_ddl, DDL_CLIENT_FAVIDC_ERROR)) {
		DDL_MSG_ERROR("ddl_enc_end:Wrong_state");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!ddl_take_command_channel(p_ddl_context, p_ddl, p_client_data))
		return VCD_ERR_BUSY;
	ddl_vidc_channel_end(p_ddl);
	return VCD_S_SUCCESS;
}

u32 ddl_reset_hw(u32 n_mode)
{
	struct ddl_context_type *p_ddl_context;
	struct ddl_client_context_type *p_ddl;
	u32 i_client_num;

	DDL_MSG_HIGH("ddl_reset_hw");
	DDL_MSG_LOW("ddl_reset_hw:called");
	p_ddl_context = ddl_get_context();
	p_ddl_context->e_cmd_state = DDL_CMD_INVALID;
	DDL_BUSY(p_ddl_context);
	if (p_ddl_context->p_core_virtual_base_addr) {
		vidc_1080p_do_sw_reset(VIDC_1080P_RESET_IN_SEQ_FIRST_STAGE);
		msleep(DDL_SW_RESET_SLEEP);
		vidc_1080p_do_sw_reset(VIDC_1080P_RESET_IN_SEQ_SECOND_STAGE);
		vidc_1080p_release_sw_reset();
		p_ddl_context->p_core_virtual_base_addr = NULL;
	}
	p_ddl_context->n_device_state = DDL_DEVICE_NOTINIT;
	for (i_client_num = 0; i_client_num < VCD_MAX_NO_CLIENT;
		++i_client_num) {
		p_ddl = p_ddl_context->a_ddl_clients[i_client_num];
		p_ddl_context->a_ddl_clients[i_client_num] = NULL;
		if (p_ddl) {
			ddl_release_client_internal_buffers(p_ddl);
			ddl_client_transact(DDL_FREE_CLIENT, &p_ddl);
		}
	}
	ddl_release_context_buffers(p_ddl_context);
	memset(p_ddl_context, 0, sizeof(struct ddl_context_type));
	return TRUE;
}

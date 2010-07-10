/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _VCD_DDL_H_
#define _VCD_DDL_H_

#include "vcd_ddl_api.h"
#include "vcd_ddl_core.h"
#include "vcd_ddl_utils.h"
#include "vidc.h"
#include "vidc_hwio.h"
#include "vidc_pix_cache.h"
#include "vidc_type.h"

#define DDL_BUSY_STATE  1
#define DDL_IDLE_STATE  0
#define DDL_RUN_STATE   3

#define DDL_IS_BUSY(p_ddl_context) \
		((p_ddl_context)->n_ddl_busy == DDL_BUSY_STATE)
#define DDL_IS_IDLE(p_ddl_context) \
		((p_ddl_context)->n_ddl_busy == DDL_IDLE_STATE)
#define DDL_BUSY(p_ddl_context) \
		((p_ddl_context)->n_ddl_busy = DDL_BUSY_STATE)
#define DDL_IDLE(p_ddl_context) \
		((p_ddl_context)->n_ddl_busy = DDL_IDLE_STATE)
#define DDL_ERROR(p_ddl_context) \
		((p_ddl_context)->n_ddl_busy = DDL_ERROR_STATE)
#define DDL_RUN(p_ddl_context) \
	((p_ddl_context)->n_ddl_busy = DDL_RUN_STATE)

#define DDL_DEVICE_NOTINIT  0
#define DDL_DEVICE_INITED   1
#define DDL_DEVICE_HWFATAL  2

#define DDL_IS_INITIALIZED(p_ddl_context) \
	(p_ddl_context->n_device_state == DDL_DEVICE_INITED)
#define DDLCOMMAND_STATE_IS(p_ddl_context, command_state) \
	(command_state == (p_ddl_context)->e_cmd_state)
#define DDLCLIENT_STATE_IS(p_ddl, client_state) \
	(client_state == (p_ddl)->e_client_state)

#define DDL_DPB_OP_INIT       1
#define DDL_DPB_OP_MARK_FREE  2
#define DDL_DPB_OP_MARK_BUSY  3
#define DDL_DPB_OP_SET_MASK   4
#define DDL_DPB_OP_RETRIEVE   5

#define DDL_INIT_CLIENTS     0
#define DDL_GET_CLIENT       1
#define DDL_FREE_CLIENT      2
#define DDL_ACTIVE_CLIENT    3

#define DDL_INVALID_CHANNEL_ID  ((u32)~0)
#define DDL_INVALID_CODEC_TYPE  ((u32)~0)
#define DDL_INVALID_INTR_STATUS ((u32)~0)

#define DDL_ENC_REQ_IFRAME        0x1
#define DDL_ENC_CHANGE_IPERIOD    0x2
#define DDL_ENC_CHANGE_BITRATE    0x4
#define DDL_ENC_CHANGE_FRAMERATE  0x8

#define DDL_DEC_REQ_OUTPUT_FLUSH  0x1

#define DDL_FW_INST_SPACE_SIZE            (DDL_MEGA_BYTE(1))
#define DDL_FW_AUX_HOST_CMD_SPACE_SIZE    (DDL_KILO_BYTE(10))
#define DDL_FW_GLOVIDC_CONTEXT_SIZE       (DDL_KILO_BYTE(400))
#define DDL_FW_H264DEC_CONTEXT_SPACE_SIZE (DDL_KILO_BYTE(600))
#define DDL_FW_OTHER_CONTEXT_SPACE_SIZE   (DDL_KILO_BYTE(10))

#define DDL_YUV_BUF_TYPE_LINEAR 0
#define DDL_YUV_BUF_TYPE_TILE   1

#define VCD_DEC_CPB_SIZE   (DDL_KILO_BYTE(512))

#define DDL_MIN_NUM_OF_B_FRAME  0
#define DDL_MAX_NUM_OF_B_FRAME  2
#define DDL_DEFAULT_NUM_OF_B_FRAME  DDL_MIN_NUM_OF_B_FRAME

#define DDL_MIN_NUM_REF_FOR_P_FRAME             1
#define DDL_MAX_NUM_REF_FOR_P_FRAME             2

#define DDL_MAX_NUM_IN_INPUTFRAME_POOL          (DDL_MAX_NUM_OF_B_FRAME + 1)

struct ddl_buf_addr_type{
	u8  *p_virtual_base_addr;
	u8  *p_physical_base_addr;
	u8  *p_align_physical_addr;
	u8  *p_align_virtual_addr;
	u32 n_buffer_size;
};
enum ddl_cmd_state_type{
	DDL_CMD_INVALID         = 0x0,
	DDL_CMD_DMA_INIT        = 0x1,
	DDL_CMD_CPU_RESET       = 0x2,
	DDL_CMD_CHANNEL_SET     = 0x3,
	DDL_CMD_INIT_CODEC      = 0x4,
	DDL_CMD_HEADER_PARSE    = 0x5,
	DDL_CMD_DECODE_SET_DPB  = 0x6,
	DDL_CMD_DECODE_FRAME    = 0x7,
	DDL_CMD_ENCODE_FRAME    = 0x8,
	DDL_CMD_EOS             = 0x9,
	DDL_CMD_CHANNEL_END     = 0xA,
	DDL_CMD_32BIT           = 0x7FFFFFFF
};
enum ddl_client_state_type{
	DDL_CLIENT_INVALID                 = 0x0,
	DDL_CLIENT_OPEN                    = 0x1,
	DDL_CLIENT_WAIT_FOR_CHDONE         = 0x2,
	DDL_CLIENT_WAIT_FOR_INITCODEC      = 0x3,
	DDL_CLIENT_WAIT_FOR_INITCODECDONE  = 0x4,
	DDL_CLIENT_WAIT_FOR_DPB            = 0x5,
	DDL_CLIENT_WAIT_FOR_DPBDONE        = 0x6,
	DDL_CLIENT_WAIT_FOR_FRAME          = 0x7,
	DDL_CLIENT_WAIT_FOR_FRAME_DONE     = 0x8,
	DDL_CLIENT_WAIT_FOR_EOS_DONE       = 0x9,
	DDL_CLIENT_WAIT_FOR_CHEND          = 0xA,
	DDL_CLIENT_FAVIDC_ERROR            = 0xC,
	DDL_CLIENT_32BIT                   = 0x7FFFFFFF
};
struct ddl_hw_interface_type{
	u32 cmd;
	u32 arg1;
	u32 arg2;
	u32 arg3;
	u32 arg4;
};
struct ddl_mask_type{
	u32  n_client_mask;
	u32  n_hw_mask;
};
struct ddl_yuv_buffer_size_type{
	u32  n_size_yuv;
	u32  n_size_y;
	u32  n_size_c;
};
struct ddl_dec_buffer_size_type{
	u32  sz_dpb0;
	u32  sz_dpb1;
	u32  sz_mv;
	u32  sz_vert_nb_mv;
	u32  sz_nb_ip;
	u32  sz_luma;
	u32  sz_chroma;
	u32  sz_nb_dcac;
	u32  sz_upnb_mv;
	u32  sz_sub_anchor_mv;
	u32  sz_overlap_xform;
	u32  sz_bit_plane3;
	u32  sz_bit_plane2;
	u32  sz_bit_plane1;
	u32  sz_stx_parser;
	u32  sz_desc;
	u32  sz_cpb;
	u32  sz_context;
};
struct ddl_dec_buffers_type{
	struct ddl_buf_addr_type desc;
	struct ddl_buf_addr_type nb_dcac;
	struct ddl_buf_addr_type upnb_mv;
	struct ddl_buf_addr_type sub_anchor_mv;
	struct ddl_buf_addr_type overlay_xform;
	struct ddl_buf_addr_type bit_plane3;
	struct ddl_buf_addr_type bit_plane2;
	struct ddl_buf_addr_type bit_plane1;
	struct ddl_buf_addr_type stx_parser;
	struct ddl_buf_addr_type h264_mv[32];
	struct ddl_buf_addr_type h264_vert_nb_mv;
	struct ddl_buf_addr_type h264_nb_ip;
	struct ddl_buf_addr_type context;
};
struct ddl_enc_buffer_size_type{
	u32  sz_cur_y;
	u32  sz_cur_c;
	u32  sz_dpb_y;
	u32  sz_dpb_c;
	u32  sz_strm;
	u32  sz_mv;
	u32  sz_col_zero;
	u32  sz_md;
	u32  sz_pred;
	u32  sz_nbor_info;
	u32  sz_acdc_coef;
	u32  sz_mb_info;
	u32  sz_context;
};
struct ddl_enc_buffers_type{
	struct ddl_buf_addr_type dpb_y[4];
	struct ddl_buf_addr_type dpb_c[4];
	struct ddl_buf_addr_type mv;
	struct ddl_buf_addr_type col_zero;
	struct ddl_buf_addr_type md;
	struct ddl_buf_addr_type pred;
	struct ddl_buf_addr_type nbor_info;
	struct ddl_buf_addr_type acdc_coef;
	struct ddl_buf_addr_type mb_info;
	struct ddl_buf_addr_type context;
	u32  n_dpb_count;
	u32  sz_dpb_y;
	u32  sz_dpb_c;
};
struct ddl_codec_data_hdr_type{
	u32  b_decoding;
};
struct ddl_encoder_data_type{
	struct ddl_codec_data_hdr_type   hdr;
	struct vcd_property_codec_type   codec_type;
	struct vcd_property_frame_size_type  frame_size;
	struct vcd_property_frame_rate_type  frame_rate;
	struct vcd_property_target_bitrate_type  target_bit_rate;
	struct vcd_property_profile_type  profile;
	struct vcd_property_level_type  level;
	struct vcd_property_rate_control_type  rc_type;
	struct vcd_property_multi_slice_type  multi_slice;
	struct ddl_buf_addr_type  meta_data_input;
	struct vcd_property_short_header_type  short_header;
	struct vcd_property_vop_timing_type  vop_timing;
	struct vcd_property_db_config_type  db_control;
	struct vcd_property_entropy_control_type  entropy_control;
	struct vcd_property_i_period_type  i_period;
	struct vcd_property_session_qp_type  session_qp;
	struct vcd_property_qp_range_type  qp_range;
	struct vcd_property_rc_level_type  rc_level;
	struct vcd_property_frame_level_rc_params_type  frame_level_rc;
	struct vcd_property_adaptive_rc_params_type  adaptive_rc;
	struct vcd_property_intra_refresh_mb_number_type  intra_refresh;
	struct vcd_property_buffer_format_type  buf_format;
	struct vcd_property_buffer_format_type  re_con_buf_format;
	struct ddl_buf_addr_type  seq_header;
	struct vcd_buffer_requirement_type  input_buf_req;
	struct vcd_buffer_requirement_type  output_buf_req;
	struct vcd_buffer_requirement_type  client_input_buf_req;
	struct vcd_buffer_requirement_type  client_output_buf_req;
	struct ddl_enc_buffers_type  hw_bufs;
	struct ddl_yuv_buffer_size_type  input_buf_size;
	struct vidc_1080p_enc_frame_info_type enc_frame_info;
	u32  n_meta_data_enable_flag;
	u32  n_suffix;
	u32  n_meta_data_offset;
	u32  n_hdr_ext_control;
	u32  n_r_cframe_skip;
	u32  n_vb_vbuffer_size;
	u32  n_dynamic_prop_change;
	u32  b_dynmic_prop_change_req;
	u32  n_seq_header_length;
	u32  b_intra_frame_insertion;
	u32  b_mb_info_enable;
	u32  n_ext_enc_control_val;
	u32  n_num_references_for_p_frame;
};
struct ddl_decoder_data_type {
	struct ddl_codec_data_hdr_type  hdr;
	struct vcd_property_codec_type  codec_type;
	struct vcd_property_buffer_format_type  buf_format;
	struct vcd_property_frame_size_type  frame_size;
	struct vcd_property_frame_size_type  client_frame_size;
	struct vcd_property_profile_type  profile;
	struct vcd_property_level_type  level;
	struct ddl_buf_addr_type  meta_data_input;
	struct vcd_property_post_filter_type  post_filter;
	struct vcd_sequence_hdr_type  decode_config;
	struct ddl_property_dec_pic_buffers_type  dp_buf;
	struct ddl_mask_type  dpb_mask;
	struct vcd_buffer_requirement_type  actual_input_buf_req;
	struct vcd_buffer_requirement_type  min_input_buf_req;
	struct vcd_buffer_requirement_type  client_input_buf_req;
	struct vcd_buffer_requirement_type  actual_output_buf_req;
	struct vcd_buffer_requirement_type  min_output_buf_req;
	struct vcd_buffer_requirement_type  client_output_buf_req;
	struct ddl_dec_buffers_type  hw_bufs;
	struct ddl_yuv_buffer_size_type  dpb_buf_size;
	struct vidc_1080p_dec_disp_info_type dec_disp_info;
	u32  n_progressive_only;
	u32  n_meta_data_enable_flag;
	u32  n_suffix;
	u32  n_meta_data_offset;
	u32  b_header_in_start;
	u32  n_min_dpb_num;
	u32  n_y_cb_cr_size;
	u32  n_dynamic_prop_change;
	u32  b_dynmic_prop_change_req;
	u32  b_flush_pending;
	u32  b_meta_data_exists;
};
union ddl_codec_data_type{
	struct ddl_codec_data_hdr_type  hdr;
	struct ddl_decoder_data_type   decoder;
	struct ddl_encoder_data_type   encoder;
};
struct ddl_context_type{
	u8 *p_core_virtual_base_addr;
	void *p_client_data;
	u32 n_device_state;
	u32 n_ddl_busy;
	u32 n_cmd_err_status;
	u32 n_disp_pic_err_status;
	u32 b_pix_cache_enable;
	u32 n_fw_version;
	u32 n_fw_memory_size;
	u32 n_fw_ctxt_memory_size;
	u32 n_cmd_seq_num;
	u32 n_response_cmd_ch_id;
	enum ddl_cmd_state_type e_cmd_state;
	struct ddl_client_context_type *p_current_ddl[2];
	struct ddl_buf_addr_type metadata_shared_input;
	struct ddl_client_context_type *a_ddl_clients[VCD_MAX_NO_CLIENT];
	struct ddl_buf_addr_type dram_base_a;
	struct ddl_buf_addr_type dram_base_b;
	struct ddl_hw_interface_type ddl_hw_response;
	void (*ddl_callback) (u32 event, u32 status, void *payload,
		u32 size, u32 *p_ddl_handle, void *const p_client_data);
	void (*pf_interrupt_clr) (void);
	void (*vidc_decode_seq_start[2])
		(struct vidc_1080p_dec_seq_start_param_type *p_param);
	void (*vidc_set_divx3_resolution[2])
		(u32 n_width, u32 n_height);
	void(*vidc_decode_init_buffers[2])
		(struct vidc_1080p_dec_init_buffers_param_type *p_param);
	void(*vidc_decode_frame_start[2])
		(struct vidc_1080p_dec_frame_start_param_type *p_param);
	void(*vidc_encode_seq_start[2])
		(struct vidc_1080p_enc_seq_start_param_type *p_param);
	void(*vidc_encode_frame_start[2])
		(struct vidc_1080p_enc_frame_start_param_type *p_param);
};
struct ddl_client_context_type{
	struct ddl_context_type  *p_ddl_context;
	enum ddl_client_state_type  e_client_state;
	struct ddl_frame_data_type_tag  b_first_output_frame;
	struct ddl_frame_data_type_tag
		extra_output_frame[DDL_MAX_NUM_OF_B_FRAME];
	struct ddl_frame_data_type_tag  input_frame;
	struct ddl_frame_data_type_tag  output_frame;
	struct ddl_frame_data_type_tag
		input_frame_pool[DDL_MAX_NUM_IN_INPUTFRAME_POOL];
	union ddl_codec_data_type  codec_data;
	enum ddl_cmd_state_type  e_cmd_state;
	struct ddl_buf_addr_type  shared_mem[2];
	void *p_client_data;
	u32  b_decoding;
	u32  n_channel_id;
	u32  n_command_channel;
	u32  n_b_count;
	s32  n_extra_output_buf_count;
	u32  n_instance_id;
};

struct ddl_context_type *ddl_get_context(void);
void ddl_vidc_core_init(struct ddl_context_type *);
void ddl_vidc_core_term(struct ddl_context_type *);
void ddl_vidc_channel_set(struct ddl_client_context_type *);
void ddl_vidc_channel_end(struct ddl_client_context_type *);
void ddl_vidc_encode_init_codec(struct ddl_client_context_type *);
void ddl_vidc_decode_init_codec(struct ddl_client_context_type *);
void ddl_vidc_encode_frame_run(struct ddl_client_context_type *);
void ddl_vidc_decode_frame_run(struct ddl_client_context_type *);
void ddl_vidc_decode_eos_run(struct ddl_client_context_type *p_ddl);
void ddl_release_context_buffers(struct ddl_context_type *);
void ddl_release_client_internal_buffers(struct ddl_client_context_type *p_ddl);
u32  ddl_vidc_decode_set_buffers(struct ddl_client_context_type *);
u32  ddl_decoder_dpb_transact(struct ddl_decoder_data_type *p_decoder,
	struct ddl_frame_data_type_tag *p_in_out_frame, u32 n_operation);
u32  ddl_decoder_dpb_init(struct ddl_client_context_type *p_ddl);
u32  ddl_client_transact(u32 , struct ddl_client_context_type **);
void ddl_set_default_decoder_buffer_req(struct ddl_decoder_data_type *p_decoder,
	u32 b_estimate);
void ddl_set_default_encoder_buffer_req(struct ddl_encoder_data_type
	*p_encoder);
void ddl_set_default_dec_property(struct ddl_client_context_type *);
u32  ddl_encoder_ready_to_start(struct ddl_client_context_type *);
u32  ddl_decoder_ready_to_start(struct ddl_client_context_type *,
	struct vcd_sequence_hdr_type *);
u32  ddl_get_yuv_buffer_size(struct vcd_property_frame_size_type *p_frame_size,
	struct vcd_property_buffer_format_type *p_buf_format, u32 b_interlace,
	u32 *pn_c_offset);
void ddl_calculate_stride(struct vcd_property_frame_size_type *p_frame_size,
	u32 b_interlace);
u32  ddl_codec_type_transact(struct ddl_client_context_type *p_ddl,
	u32 b_remove, enum vcd_codec_type e_requested_codec);
void ddl_vidc_encode_dynamic_property(struct ddl_client_context_type *p_ddl,
	u32 b_enable);
void ddl_vidc_decode_dynamic_property(struct ddl_client_context_type *p_ddl,
	u32 b_enable);
void ddl_set_initial_default_values(struct ddl_client_context_type *p_ddl);

u32  ddl_take_command_channel(struct ddl_context_type *p_ddl_context,
	struct ddl_client_context_type *p_ddl, void *p_client_data);
void ddl_release_command_channel(struct ddl_context_type  *p_ddl_context,
	u32 n_command_channel);
struct ddl_client_context_type *ddl_get_current_ddl_client_for_channel_id(
	struct ddl_context_type *p_ddl_context, u32 n_channel_id);
struct ddl_client_context_type *ddl_get_current_ddl_client_for_command(
	struct ddl_context_type *p_ddl_context,
	enum ddl_cmd_state_type e_cmd_state);

u32  ddl_get_yuv_buf_size(u32 width, u32 height, u32 format);
void ddl_free_dec_hw_buffers(struct ddl_client_context_type *p_ddl);
void ddl_free_enc_hw_buffers(struct ddl_client_context_type *p_ddl);
void ddl_calc_dec_hw_buffers_size(enum vcd_codec_type e_codec, u32 width,
	u32 height, u32 n_h264_dpb,
	struct ddl_dec_buffer_size_type *p_buf_size);
u32  ddl_allocate_h264_dec_mv_buffer(struct ddl_client_context_type *p_ddl);
u32  ddl_allocate_dec_hw_buffers(struct ddl_client_context_type *p_ddl);
u32  ddl_calc_enc_hw_buffers_size(enum vcd_codec_type e_codec, u32 width,
	u32 height, enum vcd_yuv_buffer_format_type  e_input_format,
	struct ddl_client_context_type *p_ddl,
	struct ddl_enc_buffer_size_type *p_buf_size);
u32  ddl_allocate_enc_hw_buffers(struct ddl_client_context_type *p_ddl);

u32  ddl_handle_core_errors(struct ddl_context_type *p_ddl_context);
void ddl_client_fatal_cb(struct ddl_client_context_type *p_ddl);
void ddl_hw_fatal_cb(struct ddl_client_context_type *p_ddl);

void *ddl_pmem_alloc(struct ddl_buf_addr_type *p_addr, u32 size, u32 alignment);
void ddl_pmem_free(struct ddl_buf_addr_type *p_addr);

u32 ddl_get_input_frame_from_pool(struct ddl_client_context_type *p_ddl,
	u8 *p_input_buffer_address);
u32 ddl_insert_input_frame_to_pool(struct ddl_client_context_type *p_ddl,
	struct ddl_frame_data_type_tag *p_ddl_input_frame);

#ifdef DDL_BUF_LOG
void ddl_list_buffers(struct ddl_client_context_type *p_ddl);
#endif
#ifdef DDL_MSG_LOG
s8 *ddl_get_state_string(enum ddl_client_state_type e_client_state);
#endif
extern unsigned char *vidc_video_codec_fw;
extern u32 vidc_video_codec_fw_size;

u32 ddl_fw_init(struct ddl_buf_addr_type *p_dram_base);
void ddl_get_fw_info(const unsigned char **p_fw_array_addr,
	unsigned int *p_fw_size);
void ddl_fw_release(void);
#endif

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

#include "vidc.h"
#include "vidc_hwio.h"


#define VIDC_1080P_INIT_CH_INST_ID      0x0000ffff
#define VIDC_1080P_RESET_VI             0x3f7
#define VIDC_1080P_RESET_VI_RISC        0x3f6
#define VIDC_1080P_RESET_VI_VIDC_RISC    0x3f2
#define VIDC_1080P_RESET_ALL            0
#define VIDC_1080P_RESET_RISC           0x3fe
#define VIDC_1080P_RESET_NONE           0x3ff
#define VIDC_1080P_INTERRUPT_CLEAR      0
#define VIDC_1080P_MAX_H264DECODER_DPB  32
#define VIDC_1080P_MAX_DEC_RECON_BUF    32

#define VIDC_1080P_SI_RG7_DISPLAY_STATUS_MASK    0x00000007
#define VIDC_1080P_SI_RG7_DISPLAY_STATUS_SHIFT   0
#define VIDC_1080P_SI_RG7_DISPLAY_CODING_MASK    0x00000008
#define VIDC_1080P_SI_RG7_DISPLAY_CODING_SHIFT   3
#define VIDC_1080P_SI_RG7_DISPLAY_RES_MASK       0x00000030
#define VIDC_1080P_SI_RG7_DISPLAY_RES_SHIFT      4

#define VIDC_1080P_SI_RG8_DECODE_FRAMETYPE_MASK  0x00000007

#define VIDC_1080P_SI_RG10_NUM_DPB_BMSK      0x00003fff
#define VIDC_1080P_SI_RG10_NUM_DPB_SHFT      0
#define VIDC_1080P_SI_RG10_DPB_FLUSH_BMSK    0x00004000
#define VIDC_1080P_SI_RG10_DPB_FLUSH_SHFT    14
#define VIDC_1080P_SI_RG10_DMX_DISABLE_BMSK  0x00008000
#define VIDC_1080P_SI_RG10_DMX_DISABLE_SHFT  15

#define VIDC_1080P_SI_RG11_DECODE_STATUS_MASK    0x00000007
#define VIDC_1080P_SI_RG11_DECODE_STATUS_SHIFT   0
#define VIDC_1080P_SI_RG11_DECODE_CODING_MASK    0x00000008
#define VIDC_1080P_SI_RG11_DECODE_CODING_SHIFT   3

#define VIDC_1080P_BASE_OFFSET_SHIFT         11


#define VIDC_1080P_H264DEC_LUMA_ADDR      HWIO_REG_759068_ADDR
#define VIDC_1080P_H264DEC_CHROMA_ADDR    HWIO_REG_515200_ADDR
#define VIDC_1080P_H264DEC_MV_PLANE_ADDR  HWIO_REG_466192_ADDR

#define VIDC_1080P_DEC_LUMA_ADDR        HWIO_REG_759068_ADDR
#define VIDC_1080P_DEC_CHROMA_ADDR      HWIO_REG_515200_ADDR
#define VIDC_1080P_DEC_TYPE_SEQ_HEADER  0x00010000


#define VIDC_1080P_DEC_TYPE_INIT_BUFFERS  0x00040000
#define VIDC_1080P_ENC_TYPE_SEQ_HEADER    0x00010000



u8 *VIDC_BASE_PTR;

void vidc_1080p_do_sw_reset(enum vidc_1080p_reset_type n_init_flag)
{
	if (n_init_flag == VIDC_1080P_RESET_IN_SEQ_FIRST_STAGE) {
		u32 sw_reset_value = 0;

		VIDC_HWIO_IN(REG_557899, &sw_reset_value);
		sw_reset_value &= (~HWIO_REG_557899_RSTN_VI_BMSK);
		VIDC_HWIO_OUT(REG_557899, sw_reset_value);
		sw_reset_value &= (~HWIO_REG_557899_RSTN_RISC_BMSK);
		VIDC_HWIO_OUT(REG_557899, sw_reset_value);
		sw_reset_value &= (~(HWIO_REG_557899_RSTN_VIDCCORE_BMSK |
					HWIO_REG_557899_RSTN_DMX_BMSK));

		VIDC_HWIO_OUT(REG_557899, sw_reset_value);
	} else if (n_init_flag == VIDC_1080P_RESET_IN_SEQ_SECOND_STAGE) {
		VIDC_HWIO_OUT(REG_557899, VIDC_1080P_RESET_ALL);
		VIDC_HWIO_OUT(REG_557899, VIDC_1080P_RESET_RISC);
	}
}

void vidc_1080p_release_sw_reset(void)
{
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_611794, VIDC_1080P_HOST2RISC_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_557899, VIDC_1080P_RESET_NONE);
}

void vidc_1080p_clear_interrupt(void)
{
	VIDC_HWIO_OUT(REG_575377, VIDC_1080P_INTERRUPT_CLEAR);
}

void vidc_1080p_set_host2risc_cmd(enum vidc_1080p_host2risc_cmd_type
	e_host2risc_command, u32 n_host2risc_arg1, u32 n_host2risc_arg2,
	u32 n_host2risc_arg3, u32 n_host2risc_arg4)
{
	VIDC_HWIO_OUT(REG_611794, VIDC_1080P_HOST2RISC_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_356340, n_host2risc_arg1);
	VIDC_HWIO_OUT(REG_899023, n_host2risc_arg2);
	VIDC_HWIO_OUT(REG_987762, n_host2risc_arg3);
	VIDC_HWIO_OUT(REG_544000, n_host2risc_arg4);
	VIDC_HWIO_OUT(REG_611794, e_host2risc_command);
}

void vidc_1080p_get_risc2host_cmd(u32 *pn_risc2host_command,
	u32 *pn_risc2host_arg1, u32 *pn_risc2host_arg2,
	u32 *pn_risc2host_arg3, u32 *pn_risc2host_arg4)
{
	VIDC_HWIO_IN(REG_695082, pn_risc2host_command);
	VIDC_HWIO_IN(REG_156596, pn_risc2host_arg1);
	VIDC_HWIO_IN(REG_222292, pn_risc2host_arg2);
	VIDC_HWIO_IN(REG_790962, pn_risc2host_arg3);
	VIDC_HWIO_IN(REG_679882, pn_risc2host_arg4);
}

void vidc_1080p_clear_risc2host_cmd(void)
{
	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
}

void vidc_1080p_get_fw_version(u32 *pn_fw_version)
{
	VIDC_HWIO_IN(REG_653206, pn_fw_version);
}

void vidc_1080p_get_fw_status(u32 *pn_fw_status)
{
	VIDC_HWIO_IN(REG_350619, pn_fw_status);
}

void vidc_1080p_init_memory_controller(u32 n_dram_base_addr_a,
	u32 n_dram_base_addr_b)
{
	VIDC_HWIO_OUT(REG_64440, n_dram_base_addr_a);
	VIDC_HWIO_OUT(REG_675915, n_dram_base_addr_b);
}

void vidc_1080p_get_memory_controller_status(u32 *pb_mc_abusy,
	u32 *pb_mc_bbusy)
{
	u32 n_mc_status = 0;

	VIDC_HWIO_IN(REG_399911, &n_mc_status);
	*pb_mc_abusy = (u32) ((n_mc_status &
			HWIO_REG_399911_MC_BUSY_A_BMSK) >>
			HWIO_REG_399911_MC_BUSY_A_SHFT);
	*pb_mc_bbusy = (u32) ((n_mc_status &
			HWIO_REG_399911_MC_BUSY_B_BMSK) >>
			HWIO_REG_399911_MC_BUSY_B_SHFT);
}

void vidc_1080p_set_h264_decode_buffers(u32 n_dpb, u32 n_dec_vert_nb_mv_offset,
	u32 n_dec_nb_ip_offset, u32 *pn_dpb_luma_offset,
	u32 *pn_dpb_chroma_offset, u32 *pn_mv_buffer_offset)
{
	u32 n_count = 0, n_num_dpb_used = n_dpb;
	u8 *p_vidc_dpb_luma_reg = (u8 *) VIDC_1080P_H264DEC_LUMA_ADDR;
	u8 *p_vidc_dpb_chroma_reg = (u8 *) VIDC_1080P_H264DEC_CHROMA_ADDR;
	u8 *p_vidc_mv_buffer_reg = (u8 *) VIDC_1080P_H264DEC_MV_PLANE_ADDR;

	VIDC_HWIO_OUT(REG_931311, (n_dec_vert_nb_mv_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_16277, (n_dec_nb_ip_offset >>
	VIDC_1080P_BASE_OFFSET_SHIFT));
	if (n_num_dpb_used > VIDC_1080P_MAX_H264DECODER_DPB)
		n_num_dpb_used = VIDC_1080P_MAX_H264DECODER_DPB;
	for (n_count = 0; n_count < n_num_dpb_used; n_count++) {
		VIDC_OUT_DWORD(p_vidc_dpb_luma_reg,
			(pn_dpb_luma_offset[n_count] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		VIDC_OUT_DWORD(p_vidc_dpb_chroma_reg,
			(pn_dpb_chroma_offset[n_count] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		VIDC_OUT_DWORD(p_vidc_mv_buffer_reg,
			(pn_mv_buffer_offset[n_count] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		p_vidc_dpb_luma_reg += 4;
		p_vidc_dpb_chroma_reg += 4;
		p_vidc_mv_buffer_reg += 4;
	}
}

void vidc_1080p_set_decode_recon_buffers(u32 n_recon_buffer,
	u32 *pn_dec_luma, u32 *pn_dec_chroma)
{
	u32 n_count = 0, n_recon_buf_to_program = n_recon_buffer;
	u8 *p_dec_recon_luma_reg = (u8 *) VIDC_1080P_DEC_LUMA_ADDR;
	u8 *p_dec_recon_chroma_reg = (u8 *) VIDC_1080P_DEC_CHROMA_ADDR;

	if (n_recon_buf_to_program > VIDC_1080P_MAX_DEC_RECON_BUF)
		n_recon_buf_to_program = VIDC_1080P_MAX_DEC_RECON_BUF;
	for (n_count = 0; n_count < n_recon_buf_to_program; n_count++) {
		VIDC_OUT_DWORD(p_dec_recon_luma_reg, (pn_dec_luma[n_count] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		VIDC_OUT_DWORD(p_dec_recon_chroma_reg,
			(pn_dec_chroma[n_count] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		p_dec_recon_luma_reg += 4;
		p_dec_recon_chroma_reg += 4;
	}
}

void vidc_1080p_set_mpeg4_divx_decode_work_buffers(u32 n_nb_dcac_buffer_offset,
	u32 n_upnb_mv_buffer_offset, u32 n_sub_anchor_buffer_offset,
	u32 n_overlay_transform_buffer_offset, u32 n_stx_parser_buffer_offset)
{
	VIDC_HWIO_OUT(REG_931311, (n_nb_dcac_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_16277, (n_upnb_mv_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_654169, (n_sub_anchor_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_802794,
		(n_overlay_transform_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_252167, (n_stx_parser_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
}

void vidc_1080p_set_h263_decode_work_buffers(u32 n_nb_dcac_buffer_offset,
	u32 n_upnb_mv_buffer_offset, u32 n_sub_anchor_buffer_offset,
	u32 n_overlay_transform_buffer_offset)
{
	VIDC_HWIO_OUT(REG_931311, (n_nb_dcac_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_16277, (n_upnb_mv_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_654169, (n_sub_anchor_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_802794,
		(n_overlay_transform_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
}

void vidc_1080p_set_vc1_decode_work_buffers(u32 n_nb_dcac_buffer_offset,
	u32 n_upnb_mv_buffer_offset, u32 n_sub_anchor_buffer_offset,
	u32 n_overlay_transform_buffer_offset, u32 n_bitplain1Buffer_offset,
	u32 n_bitplain2Buffer_offset, u32 n_bitplain3Buffer_offset)
{
	VIDC_HWIO_OUT(REG_931311, (n_nb_dcac_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_16277, (n_upnb_mv_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_654169, (n_sub_anchor_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_802794,
		(n_overlay_transform_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_724376, (n_bitplain3Buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_551674, (n_bitplain2Buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_115991, (n_bitplain1Buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
}

void vidc_1080p_set_encode_recon_buffers(u32 n_recon_buffer,
	u32 *pn_enc_luma, u32 *pn_enc_chroma)
{
	if (n_recon_buffer > 0) {
		VIDC_HWIO_OUT(REG_294579, (pn_enc_luma[0] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		VIDC_HWIO_OUT(REG_759068, (pn_enc_chroma[0] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
	}
	if (n_recon_buffer > 1) {
		VIDC_HWIO_OUT(REG_61427, (pn_enc_luma[1] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		VIDC_HWIO_OUT(REG_68356, (pn_enc_chroma[1] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
	}
	if (n_recon_buffer > 2) {
		VIDC_HWIO_OUT(REG_616802, (pn_enc_luma[2] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		VIDC_HWIO_OUT(REG_833502, (pn_enc_chroma[2] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
	}
	if (n_recon_buffer > 3) {
		VIDC_HWIO_OUT(REG_23318, (pn_enc_luma[3] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
		VIDC_HWIO_OUT(REG_127855, (pn_enc_chroma[3] >>
			VIDC_1080P_BASE_OFFSET_SHIFT));
	}
}

void vidc_1080p_set_h264_encode_work_buffers(u32 n_up_row_mv_buffer_offset,
	u32 n_direct_colzero_flag_buffer_offset,
	u32 n_upper_intra_md_buffer_offset,
	u32 n_upper_intra_pred_buffer_offset, u32 n_nbor_infor_buffer_offset,
	u32 n_mb_info_offset)
{
	VIDC_HWIO_OUT(REG_515200, (n_up_row_mv_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_69832,
		(n_direct_colzero_flag_buffer_offset>>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_256132,
		(n_upper_intra_md_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_475648,
		(n_upper_intra_pred_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_29510, (n_nbor_infor_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_175929, (n_mb_info_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
}

void vidc_1080p_set_h263_encode_work_buffers(u32 n_up_row_mv_buffer_offset,
	u32 n_up_row_inv_quanti_coeff_buffer_offset)
{
	VIDC_HWIO_OUT(REG_515200, (n_up_row_mv_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_29510, (
		n_up_row_inv_quanti_coeff_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
}

void vidc_1080p_set_mpeg4_encode_work_buffers(u32 n_skip_flag_buffer_offset,
	u32 n_up_row_inv_quanti_coeff_buffer_offset, u32 n_upper_mv_offset)
{
	VIDC_HWIO_OUT(REG_69832, (n_skip_flag_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_29510, (
		n_up_row_inv_quanti_coeff_buffer_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
	VIDC_HWIO_OUT(REG_515200, (n_upper_mv_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT));
}

void vidc_1080p_set_encode_frame_size(u32 n_hori_size, u32 n_vert_size)
{
	VIDC_HWIO_OUT(REG_934655, n_hori_size);
	VIDC_HWIO_OUT(REG_179070, n_vert_size);
}

void vidc_1080p_set_encode_profile_level(u32 n_encode_profile, u32 n_enc_level)
{
	u32 n_profile_level = 0;

	n_profile_level = VIDC_SETFIELD(n_enc_level,
				HWIO_REG_63643_LEVEL_SHFT,
				HWIO_REG_63643_LEVEL_BMSK) |
				VIDC_SETFIELD(n_encode_profile,
				HWIO_REG_63643_PROFILE_SHFT,
				HWIO_REG_63643_PROFILE_BMSK);
	VIDC_HWIO_OUT(REG_63643, n_profile_level);
}

void vidc_1080p_set_encode_field_picture_structure(u32 b_enc_field_picture)
{
	VIDC_HWIO_OUT(REG_786024, b_enc_field_picture);
}

void vidc_1080p_set_encode_deblock_filter(u32 b_lf_enables)
{
	VIDC_HWIO_OUT(REG_152500, b_lf_enables);
}

void vidc_1080p_set_decode_qp_save_control(u32 b_enable_q_pout)
{
	VIDC_HWIO_OUT(REG_143629, b_enable_q_pout);
}

void vidc_1080p_get_returned_channel_inst_id(u32 *pn_rtn_chid)
{
	VIDC_HWIO_IN(REG_607589, pn_rtn_chid);
}

void vidc_1080p_clear_returned_channel_inst_id(void)
{
	VIDC_HWIO_OUT(REG_607589, VIDC_1080P_INIT_CH_INST_ID);
}

void vidc_1080p_get_decode_seq_start_result(
	struct vidc_1080p_seq_hdr_info_type *p_seq_hdr_info)
{
	u32 n_display_result;


	VIDC_HWIO_IN(REG_845544, &p_seq_hdr_info->n_img_size_y);
	VIDC_HWIO_IN(REG_859906, &p_seq_hdr_info->n_img_size_x);
	VIDC_HWIO_IN(REG_490078, &p_seq_hdr_info->n_min_num_dpb);
	VIDC_HWIO_IN(REG_489688, &p_seq_hdr_info->n_dec_frm_size);
	VIDC_HWIO_IN(REG_853667, &n_display_result);
	p_seq_hdr_info->n_progressive = VIDC_GETFIELD(n_display_result,
					VIDC_1080P_SI_RG7_DISPLAY_CODING_MASK,
					VIDC_1080P_SI_RG7_DISPLAY_CODING_SHIFT);
}

void vidc_1080p_get_decoded_frame_size(u32 *pn_decoded_size)
{
	VIDC_HWIO_IN(REG_489688, pn_decoded_size);
}

void vidc_1080p_get_display_frame_result(
	struct vidc_1080p_dec_disp_info_type *p_dec_disp_info)
{
	u32 n_display_result;

	VIDC_HWIO_IN(REG_640904, &p_dec_disp_info->n_display_y_addr);
	VIDC_HWIO_IN(REG_60114, &p_dec_disp_info->n_display_c_addr);
	VIDC_HWIO_IN(REG_853667, &n_display_result);
	p_dec_disp_info->e_display_status =
		(enum vidc_1080p_display_status_type)
		VIDC_GETFIELD(n_display_result,
		VIDC_1080P_SI_RG7_DISPLAY_STATUS_MASK,
		VIDC_1080P_SI_RG7_DISPLAY_STATUS_SHIFT);
	p_dec_disp_info->e_display_coding =
		(enum vidc_1080p_display_coding_type)
	VIDC_GETFIELD(n_display_result, VIDC_1080P_SI_RG7_DISPLAY_CODING_MASK,
		VIDC_1080P_SI_RG7_DISPLAY_CODING_SHIFT);
	p_dec_disp_info->n_resl_change = VIDC_GETFIELD(n_display_result,
		VIDC_1080P_SI_RG7_DISPLAY_RES_MASK,
		VIDC_1080P_SI_RG7_DISPLAY_RES_SHIFT);
}

void vidc_1080p_get_decode_frame_type(
	enum vidc_1080p_decode_frame_type *pe_frame_type)
{
	u32 n_frame_type = 0;

	VIDC_HWIO_IN(REG_760102, &n_frame_type);
	*pe_frame_type = (enum vidc_1080p_decode_frame_type)
		(n_frame_type & VIDC_1080P_SI_RG8_DECODE_FRAMETYPE_MASK);
}

void vidc_1080p_get_decode_frame_result(
	struct vidc_1080p_dec_disp_info_type *p_dec_disp_info)
{
	u32 n_decode_result;

	VIDC_HWIO_IN(REG_378318, &p_dec_disp_info->n_decode_y_addr);
	VIDC_HWIO_IN(REG_203487, &p_dec_disp_info->n_decode_c_addr);
	VIDC_HWIO_IN(REG_692991, &n_decode_result);
	p_dec_disp_info->e_decode_status = (enum vidc_1080p_display_status_type)
				VIDC_GETFIELD(n_decode_result,
				VIDC_1080P_SI_RG11_DECODE_STATUS_MASK,
				VIDC_1080P_SI_RG11_DECODE_STATUS_SHIFT);
	p_dec_disp_info->e_decode_coding = (enum vidc_1080p_display_coding_type)
				VIDC_GETFIELD(n_decode_result,
				VIDC_1080P_SI_RG11_DECODE_CODING_MASK,
				VIDC_1080P_SI_RG11_DECODE_CODING_SHIFT);
}

void vidc_1080p_decode_seq_start_ch0(
	struct vidc_1080p_dec_seq_start_param_type *p_param)
{

	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_117192,
		p_param->n_stream_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_145068, p_param->n_stream_frame_size);
	VIDC_HWIO_OUT(REG_921356,
		p_param->n_descriptor_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_190381,  p_param->n_stream_buffersize);
	VIDC_HWIO_OUT(REG_85655,  p_param->n_descriptor_buffer_size);
	VIDC_HWIO_OUT(REG_889944,  p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_397087, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_DEC_TYPE_SEQ_HEADER |
		p_param->n_inst_id);
}

void vidc_1080p_decode_seq_start_ch1(
	struct vidc_1080p_dec_seq_start_param_type *p_param)
{

	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_980194,
		p_param->n_stream_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_936704, p_param->n_stream_frame_size);
	VIDC_HWIO_OUT(REG_821977,
		p_param->n_descriptor_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_887095, p_param->n_stream_buffersize);
	VIDC_HWIO_OUT(REG_576987, p_param->n_descriptor_buffer_size);
	VIDC_HWIO_OUT(REG_652528, p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_254093, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_DEC_TYPE_SEQ_HEADER |
		p_param->n_inst_id);
}

void vidc_1080p_decode_frame_start_ch0(
	struct vidc_1080p_dec_frame_start_param_type *p_param)
{
	u32 n_dpb_config;

	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_INIT_CH_INST_ID);
	if ((p_param->e_decode_type == VIDC_1080P_DEC_TYPE_LAST_FRAME_DATA) &&
		((!p_param->n_stream_buffer_addr_offset) ||
		(!p_param->n_stream_frame_size))) {
		VIDC_HWIO_OUT(REG_117192, 0);
		VIDC_HWIO_OUT(REG_145068, 0);
		VIDC_HWIO_OUT(REG_190381, 0);
	} else {
		VIDC_HWIO_OUT(REG_117192,
			p_param->n_stream_buffer_addr_offset >>
			VIDC_1080P_BASE_OFFSET_SHIFT);
		VIDC_HWIO_OUT(REG_145068,
			p_param->n_stream_frame_size);
		VIDC_HWIO_OUT(REG_190381,
			p_param->n_stream_buffersize);
	}
	n_dpb_config = VIDC_SETFIELD(p_param->b_dpb_flush,
					VIDC_1080P_SI_RG10_DPB_FLUSH_SHFT,
					VIDC_1080P_SI_RG10_DPB_FLUSH_BMSK) |
				VIDC_SETFIELD(p_param->n_dpb_count,
					VIDC_1080P_SI_RG10_NUM_DPB_SHFT,
					VIDC_1080P_SI_RG10_NUM_DPB_BMSK);
	VIDC_HWIO_OUT(REG_921356,
		p_param->n_descriptor_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_85655, p_param->n_descriptor_buffer_size);
	VIDC_HWIO_OUT(REG_86830, p_param->n_release_dpb_bit_mask);
	VIDC_HWIO_OUT(REG_889944, p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_404623, n_dpb_config);
	VIDC_HWIO_OUT(REG_397087, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_666957, (u32)p_param->e_decode_type |
		p_param->n_inst_id);
}


void vidc_1080p_decode_frame_start_ch1(
	struct vidc_1080p_dec_frame_start_param_type *p_param)
{
	u32 n_dpb_config;

	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_INIT_CH_INST_ID);
	if ((p_param->e_decode_type == VIDC_1080P_DEC_TYPE_LAST_FRAME_DATA) &&
		((!p_param->n_stream_buffer_addr_offset) ||
		(!p_param->n_stream_frame_size))) {
		VIDC_HWIO_OUT(REG_980194, 0);
		VIDC_HWIO_OUT(REG_936704, 0);
		VIDC_HWIO_OUT(REG_887095, 0);
	} else {
		VIDC_HWIO_OUT(REG_980194,
			p_param->n_stream_buffer_addr_offset >>
			VIDC_1080P_BASE_OFFSET_SHIFT);
		VIDC_HWIO_OUT(REG_936704,
			p_param->n_stream_frame_size);
		VIDC_HWIO_OUT(REG_887095,
			p_param->n_stream_buffersize);
	}
	n_dpb_config = VIDC_SETFIELD(p_param->b_dpb_flush,
					VIDC_1080P_SI_RG10_DPB_FLUSH_SHFT,
					VIDC_1080P_SI_RG10_DPB_FLUSH_BMSK) |
				VIDC_SETFIELD(p_param->n_dpb_count,
					VIDC_1080P_SI_RG10_NUM_DPB_SHFT,
					VIDC_1080P_SI_RG10_NUM_DPB_BMSK);
	VIDC_HWIO_OUT(REG_821977,
		p_param->n_descriptor_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_576987, p_param->n_descriptor_buffer_size);
	VIDC_HWIO_OUT(REG_70448, p_param->n_release_dpb_bit_mask);
	VIDC_HWIO_OUT(REG_652528, p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_220637, n_dpb_config);
	VIDC_HWIO_OUT(REG_254093, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_313350, (u32)p_param->e_decode_type |
		p_param->n_inst_id);
}

void vidc_1080p_decode_init_buffers_ch0(
	struct vidc_1080p_dec_init_buffers_param_type *p_param)
{
	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_889944, p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_404623, p_param->n_dpb_count);
	VIDC_HWIO_OUT(REG_397087, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_DEC_TYPE_INIT_BUFFERS |
		p_param->n_inst_id);
}

void vidc_1080p_decode_init_buffers_ch1(
	struct vidc_1080p_dec_init_buffers_param_type *p_param)
{
	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_652528,  p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_220637, p_param->n_dpb_count);
	VIDC_HWIO_OUT(REG_254093, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_DEC_TYPE_INIT_BUFFERS |
		p_param->n_inst_id);
}

void vidc_1080p_set_divx3_resolution_ch0(u32 n_width, u32 n_height)
{
	VIDC_HWIO_OUT(REG_612810, n_height);
	VIDC_HWIO_OUT(REG_175608, n_width);
}

void vidc_1080p_set_divx3_resolution_ch1(u32 n_width, u32 n_height)
{
	VIDC_HWIO_OUT(REG_655721, n_height);
	VIDC_HWIO_OUT(REG_548308, n_width);
}

void vidc_1080p_get_encode_frame_info(
	struct vidc_1080p_enc_frame_info_type *p_frame_info)
{
	VIDC_HWIO_IN(REG_845544, &(p_frame_info->n_enc_frame_size));
	VIDC_HWIO_IN(REG_859906,
		&(p_frame_info->n_enc_picture_count));
	VIDC_HWIO_IN(REG_490078,
		&(p_frame_info->n_enc_write_pointer));
	VIDC_HWIO_IN(REG_640904,
		(u32 *)(&(p_frame_info->e_enc_frame_type)));
	VIDC_HWIO_IN(REG_60114,
		&(p_frame_info->n_enc_luma_address));
	p_frame_info->n_enc_luma_address = p_frame_info->n_enc_luma_address <<
		VIDC_1080P_BASE_OFFSET_SHIFT;
	VIDC_HWIO_IN(REG_489688,
		&(p_frame_info->n_enc_chroma_address));
	p_frame_info->n_enc_chroma_address = p_frame_info->\
		n_enc_chroma_address << VIDC_1080P_BASE_OFFSET_SHIFT;
}

void vidc_1080p_encode_seq_start_ch0(
	struct vidc_1080p_enc_seq_start_param_type *p_param)
{
	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_117192,
		p_param->n_stream_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_921356, p_param->n_stream_buffer_size);
	VIDC_HWIO_OUT(REG_889944, p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_397087, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_ENC_TYPE_SEQ_HEADER |
		p_param->n_inst_id);
}

void vidc_1080p_encode_seq_start_ch1(
	struct vidc_1080p_enc_seq_start_param_type *p_param)
{
	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_980194,
		p_param->n_stream_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_821977, p_param->n_stream_buffer_size);
	VIDC_HWIO_OUT(REG_652528, p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_254093, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_ENC_TYPE_SEQ_HEADER |
		p_param->n_inst_id);
}

void vidc_1080p_encode_frame_start_ch0(
	struct vidc_1080p_enc_frame_start_param_type *p_param)
{
	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_666957, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_117192,
		p_param->n_stream_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_921356, p_param->n_stream_buffer_size);
	VIDC_HWIO_OUT(REG_612810, p_param->n_current_y_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_175608, p_param->n_current_c_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_190381, p_param->b_intra_frame);
	VIDC_HWIO_OUT(REG_889944, p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_404623, p_param->b_input_flush);
	VIDC_HWIO_OUT(REG_397087, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_666957, (u32)p_param->e_encode_type |
		p_param->n_inst_id);
}

void vidc_1080p_encode_frame_start_ch1(
	struct vidc_1080p_enc_frame_start_param_type *p_param)
{

	VIDC_HWIO_OUT(REG_695082, VIDC_1080P_RISC2HOST_CMD_EMPTY);
	VIDC_HWIO_OUT(REG_313350, VIDC_1080P_INIT_CH_INST_ID);
	VIDC_HWIO_OUT(REG_980194,
		p_param->n_stream_buffer_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_821977, p_param->n_stream_buffer_size);
	VIDC_HWIO_OUT(REG_655721, p_param->n_current_y_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_548308,  p_param->n_current_c_addr_offset >>
		VIDC_1080P_BASE_OFFSET_SHIFT);
	VIDC_HWIO_OUT(REG_887095, p_param->b_intra_frame);
	VIDC_HWIO_OUT(REG_652528, p_param->n_shared_mem_addr_offset);
	VIDC_HWIO_OUT(REG_404623, p_param->b_input_flush);
	VIDC_HWIO_OUT(REG_254093, p_param->n_cmd_seq_num);
	VIDC_HWIO_OUT(REG_313350, (u32)p_param->e_encode_type |
		p_param->n_inst_id);
}

void vidc_1080p_set_encode_picture_type(u32 n_ifrm_ctrl, u32 n_number_b)
{
	u32 n_picture_type = VIDC_SETFIELD(1 ,
				HWIO_REG_783891_ENC_PIC_TYPE_USE_SHFT,
				HWIO_REG_783891_ENC_PIC_TYPE_USE_BMSK) |
				VIDC_SETFIELD(n_ifrm_ctrl + 1 ,
					HWIO_REG_783891_I_FRM_CTRL_SHFT,
					HWIO_REG_783891_I_FRM_CTRL_BMSK)
				| VIDC_SETFIELD(n_number_b ,
				HWIO_REG_783891_B_FRM_CTRL_SHFT ,
				HWIO_REG_783891_B_FRM_CTRL_BMSK);
	VIDC_HWIO_OUT(REG_783891, n_picture_type);
}

void vidc_1080p_set_encode_multi_slice_control(
	enum vidc_1080p_MSlice_selection_type e_multiple_slice_selection,
	u32 n_mslice_mb, u32 n_mslice_byte)
{
	VIDC_HWIO_OUT(REG_226332, e_multiple_slice_selection);
	VIDC_HWIO_OUT(REG_696136, n_mslice_mb);
	VIDC_HWIO_OUT(REG_515564, n_mslice_byte);
}

void vidc_1080p_set_encode_circular_intra_refresh(u32 n_cir_num)
{
	VIDC_HWIO_OUT(REG_886210, n_cir_num);
}

void vidc_1080p_set_encode_input_frame_format(
	enum vidc_1080p_memory_access_method_type e_memory_format)
{
	VIDC_HWIO_OUT(REG_645603, e_memory_format);
}

void vidc_1080p_set_encode_padding_control(u32 b_pad_ctrl_on,
	u32 n_cr_pad_val, u32 n_cb_pad_val, u32 n_luma_pad_val)
{
	u32 n_padding = VIDC_SETFIELD(b_pad_ctrl_on ,
				HWIO_REG_811733_PAD_CTRL_ON_SHFT,
				HWIO_REG_811733_PAD_CTRL_ON_BMSK) |
			VIDC_SETFIELD(n_cr_pad_val ,
				HWIO_REG_811733_CR_PAD_VIDC_SHFT ,
				HWIO_REG_811733_CR_PAD_VIDC_BMSK) |
			VIDC_SETFIELD(n_cb_pad_val ,
				HWIO_REG_811733_CB_PAD_VIDC_SHFT ,
				HWIO_REG_811733_CB_PAD_VIDC_BMSK) |
			VIDC_SETFIELD(n_luma_pad_val ,
				HWIO_REG_811733_LUMA_PAD_VIDC_SHFT ,
				HWIO_REG_811733_LUMA_PAD_VIDC_BMSK) ;
	VIDC_HWIO_OUT(REG_811733, n_padding);
}

void vidc_1080p_encode_set_rc_config(u32 b_enable_frame_level_rc,
	u32 b_enable_mb_level_rc_flag, u32 n_frame_qp)
{
   u32 n_rc_config = VIDC_SETFIELD(b_enable_frame_level_rc ,
					HWIO_REG_559908_FR_RC_EN_SHFT ,
					HWIO_REG_559908_FR_RC_EN_BMSK) |
			VIDC_SETFIELD(b_enable_mb_level_rc_flag ,
					HWIO_REG_559908_MB_RC_EN_SHFT,
					HWIO_REG_559908_MB_RC_EN_BMSK) |
			VIDC_SETFIELD(n_frame_qp ,
					HWIO_REG_559908_FRAME_QP_SHFT ,
					HWIO_REG_559908_FRAME_QP_BMSK);
	VIDC_HWIO_OUT(REG_559908, n_rc_config);
}

void vidc_1080p_encode_set_frame_level_rc_params(u32 n_rc_frame_rate,
	u32 n_target_bitrate, u32 n_reaction_coeff)
{
	VIDC_HWIO_OUT(REG_977937, n_rc_frame_rate);
	VIDC_HWIO_OUT(REG_166135, n_target_bitrate);
	VIDC_HWIO_OUT(REG_550322, n_reaction_coeff);
}

void vidc_1080p_encode_set_qp_params(u32 n_max_qp, u32 n_min_qp)
{
	u32 n_qbound = VIDC_SETFIELD(n_max_qp , HWIO_REG_109072_MAX_QP_SHFT,
					HWIO_REG_109072_MAX_QP_BMSK) |
					VIDC_SETFIELD(n_min_qp,
					HWIO_REG_109072_MIN_QP_SHFT ,
					HWIO_REG_109072_MIN_QP_BMSK);
	VIDC_HWIO_OUT(REG_109072, n_qbound);
}

void vidc_1080p_encode_set_mb_level_rc_params(u32 b_dark_region_as_flag,
	u32 b_smooth_region_as_flag , u32 b_static_region_as_flag,
	u32 b_activity_region_flag)
{
	u32 n_rc_active_feature = VIDC_SETFIELD(b_dark_region_as_flag ,
					HWIO_REG_949086_DARK_DISABLE_SHFT ,
					HWIO_REG_949086_DARK_DISABLE_BMSK) |
					VIDC_SETFIELD(b_smooth_region_as_flag ,
					HWIO_REG_949086_SMOOTH_DISABLE_SHFT ,
					HWIO_REG_949086_SMOOTH_DISABLE_BMSK) |
					VIDC_SETFIELD(b_static_region_as_flag ,
					HWIO_REG_949086_STATIC_DISABLE_SHFT ,
					HWIO_REG_949086_STATIC_DISABLE_BMSK) |
					VIDC_SETFIELD(b_activity_region_flag,
					HWIO_REG_949086_ACT_DISABLE_SHFT ,
					HWIO_REG_949086_ACT_DISABLE_BMSK);
	VIDC_HWIO_OUT(REG_949086, n_rc_active_feature);
}

void vidc_1080p_set_h264_encode_entropy(
	enum vidc_1080p_entropy_sel_type e_entropy_sel)
{
	VIDC_HWIO_OUT(REG_447796, e_entropy_sel);
}

void vidc_1080p_set_h264_encode_loop_filter(
	enum vidc_1080p_DBConfig_type e_db_config, u32 n_slice_alpha_offset,
	u32 n_slice_beta_offset)
{
	VIDC_HWIO_OUT(REG_152500, e_db_config);
	VIDC_HWIO_OUT(REG_266285, n_slice_alpha_offset);
	VIDC_HWIO_OUT(REG_964731, n_slice_beta_offset);
}

void vidc_1080p_set_h264_encoder_ref_count(u32 n_max_reference)
{
	u32 n_ref_frames;
	n_ref_frames = VIDC_SETFIELD(n_max_reference,
		HWIO_REG_744348_P_SHFT,
		HWIO_REG_744348_P_BMSK);
	n_ref_frames |= VIDC_SETFIELD(2, HWIO_REG_744348_SHFT,
		HWIO_REG_744348_BMSK);
	VIDC_HWIO_OUT(REG_744348, n_ref_frames);
}

void vidc_1080p_set_h264_encode_8x8transform_control(u32 b_enable_8x8transform)
{
	VIDC_HWIO_OUT(REG_672163, b_enable_8x8transform);
}

void vidc_1080p_set_mpeg4_encode_quarter_pel_control(
	u32 b_enable_mpeg4_quarter_pel)
{
	VIDC_HWIO_OUT(REG_330132, b_enable_mpeg4_quarter_pel);
}

void vidc_1080p_set_device_base_addr(u8 *mapped_va)
{
	VIDC_BASE_PTR = mapped_va;
}

void vidc_1080p_get_intra_bias(u32 *p_intra_bias)
{
	u32 n_intra_bias;

	VIDC_HWIO_IN(REG_676866, &n_intra_bias);
	*p_intra_bias = VIDC_GETFIELD(n_intra_bias,
					HWIO_REG_676866_RMSK,
					HWIO_REG_676866_SHFT);
}

void vidc_1080p_set_intra_bias(u32 intra_bias)
{
	u32 n_intra_bias;

	n_intra_bias = VIDC_SETFIELD(intra_bias,
					HWIO_REG_676866_SHFT,
					HWIO_REG_676866_RMSK);
	VIDC_HWIO_OUT(REG_676866, n_intra_bias);
}

void vidc_1080p_get_bi_directional_bias(u32 *p_bi_directional_bias)
{
	u32 nbi_direct_bias;

	VIDC_HWIO_IN(REG_54267, &nbi_direct_bias);
	*p_bi_directional_bias = VIDC_GETFIELD(nbi_direct_bias,
					HWIO_REG_54267_RMSK,
					HWIO_REG_54267_SHFT);
}

void vidc_1080p_set_bi_directional_bias(u32 bi_directional_bias)
{
	u32 nbi_direct_bias;

	nbi_direct_bias = VIDC_SETFIELD(bi_directional_bias,
					HWIO_REG_54267_SHFT,
					HWIO_REG_54267_RMSK);
	VIDC_HWIO_OUT(REG_54267, nbi_direct_bias);
}

void vidc_1080p_get_encoder_sequence_header_size(u32 *p_seq_header_size)
{
	VIDC_HWIO_IN(REG_845544, p_seq_header_size);
}

void vidc_1080p_get_intermedia_stage_debug_counter(
	u32 *p_intermediate_stage_counter)
{
	VIDC_HWIO_IN(REG_805993, p_intermediate_stage_counter);
}

void vidc_1080p_get_exception_status(u32 *p_exception_status)
{
	VIDC_HWIO_IN(REG_493355, p_exception_status);
}

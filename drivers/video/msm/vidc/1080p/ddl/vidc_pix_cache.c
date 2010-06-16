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

#include "vidc_hwio_reg.h"
#include "vidc_hwio.h"
#include "vidc_pix_cache.h"


#define VIDC_1080P_MAX_DEC_DPB 19
#define VIDC_TILE_MULTIPLY_FACTOR 8192
#define VIDC_1080P_DEC_DPB_RESET_VALUE 0xFFFFFFF8

void vidc_pix_cache_sw_reset(void)
{
	u32 n_sw_reset_value = 0;

	VIDC_HWIO_IN(REG_169013, &n_sw_reset_value);
	n_sw_reset_value |= HWIO_REG_169013_PIX_CACHE_SW_RESET_BMSK;
	VIDC_HWIO_OUT(REG_169013, n_sw_reset_value);
	VIDC_HWIO_IN(REG_169013, &n_sw_reset_value);
	n_sw_reset_value &= (~HWIO_REG_169013_PIX_CACHE_SW_RESET_BMSK);
	VIDC_HWIO_OUT(REG_169013, n_sw_reset_value);
}

void vidc_pix_cache_init_luma_chroma_base_addr(u32 n_dpb,
	u32 *pn_dpb_luma_offset, u32 *pn_dpb_chroma_offset)
{
	u32 n_count, n_num_dpb_used = n_dpb;
	u32 n_dpb_reset_value = VIDC_1080P_DEC_DPB_RESET_VALUE;

	if (n_num_dpb_used > VIDC_1080P_MAX_DEC_DPB)
		n_num_dpb_used = VIDC_1080P_MAX_DEC_DPB;
	for (n_count = 0; n_count < VIDC_1080P_MAX_DEC_DPB; n_count++) {
		if (n_count < n_num_dpb_used) {
			if (pn_dpb_luma_offset) {
				VIDC_HWIO_OUTI(
					REG_804925,
					n_count, pn_dpb_luma_offset[n_count]);
			} else {
				VIDC_HWIO_OUTI(
					REG_804925,
					n_count, n_dpb_reset_value);
			}
			if (pn_dpb_chroma_offset) {
				VIDC_HWIO_OUTI(
					REG_41909,
					n_count, pn_dpb_chroma_offset[n_count]);
			} else {
				VIDC_HWIO_OUTI(
					REG_41909,
					n_count, n_dpb_reset_value);
			}
		} else {
			VIDC_HWIO_OUTI(REG_804925,
				n_count, n_dpb_reset_value);
			VIDC_HWIO_OUTI(REG_41909,
				n_count, n_dpb_reset_value);
		}
	}
}

void vidc_pix_cache_set_frame_range(u32 n_luma_size, u32 n_chroma_size)
{
	u32 n_frame_range;

	n_frame_range =
		(((n_luma_size / VIDC_TILE_MULTIPLY_FACTOR) & 0xFF) << 8)|
		((n_chroma_size / VIDC_TILE_MULTIPLY_FACTOR) & 0xFF);
	VIDC_HWIO_OUT(REG_905239, n_frame_range);
}

void vidc_pix_cache_init_config(
	struct vidc_1080P_pix_cache_config_type *p_config)
{
	u32 n_cfg_reg = 0;

	if (p_config->b_cache_enable)
		n_cfg_reg |= HWIO_REG_22756_CACHE_EN_BMSK;
	else
		n_cfg_reg &= (~HWIO_REG_22756_CACHE_EN_BMSK);
	if (p_config->e_port_select == VIDC_1080P_PIX_CACHE_PORT_A)
		n_cfg_reg &=
			(~HWIO_REG_22756_CACHE_PORT_SELECT_BMSK);
	else
		n_cfg_reg |= HWIO_REG_22756_CACHE_PORT_SELECT_BMSK;
	if (!p_config->b_statistics_off)
		n_cfg_reg |= HWIO_REG_22756_STATISTICS_OFF_BMSK;
	else
		n_cfg_reg &= (~HWIO_REG_22756_STATISTICS_OFF_BMSK);
	if (p_config->b_prefetch_en)
		n_cfg_reg |= HWIO_REG_22756_PREFETCH_EN_BMSK;
	else
		n_cfg_reg &= (~HWIO_REG_22756_PREFETCH_EN_BMSK);
	n_cfg_reg &= (~HWIO_REG_22756_PAGE_SIZE_BMSK);
	n_cfg_reg |= VIDC_SETFIELD(p_config->e_page_size,
			HWIO_REG_22756_PAGE_SIZE_SHFT,
			HWIO_REG_22756_PAGE_SIZE_BMSK);
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
}

void vidc_pix_cache_set_prefetch_page_limit(u32 n_page_size_limit)
{
	u32 n_cfg_reg = 0;

	VIDC_HWIO_IN(REG_22756, &n_cfg_reg);
	n_cfg_reg &= (~HWIO_REG_22756_PAGE_SIZE_BMSK);
	n_cfg_reg |= VIDC_SETFIELD(n_page_size_limit,
			HWIO_REG_22756_PAGE_SIZE_SHFT,
			HWIO_REG_22756_PAGE_SIZE_BMSK);
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
}

void vidc_pix_cache_enable_prefetch(u32 b_prefetch_enable)
{
	u32 n_cfg_reg = 0;

	VIDC_HWIO_IN(REG_22756, &n_cfg_reg);
	if (b_prefetch_enable)
		n_cfg_reg |= HWIO_REG_22756_PREFETCH_EN_BMSK;
	else
		n_cfg_reg &= (~HWIO_REG_22756_PREFETCH_EN_BMSK);
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
}

void vidc_pix_cache_disable_statistics(u32 b_statistics_off)
{
	u32 n_cfg_reg = 0;

	VIDC_HWIO_IN(REG_22756, &n_cfg_reg);
	if (!b_statistics_off)
		n_cfg_reg |= HWIO_REG_22756_STATISTICS_OFF_BMSK;
	else
		n_cfg_reg &= (~HWIO_REG_22756_STATISTICS_OFF_BMSK);
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
}

void vidc_pix_cache_set_port(
	enum vidc_1080P_pix_cache_port_sel_type e_port_select)
{
	u32 n_cfg_reg = 0;

	VIDC_HWIO_IN(REG_22756, &n_cfg_reg);
	if (e_port_select == VIDC_1080P_PIX_CACHE_PORT_A)
		n_cfg_reg &=
			(~HWIO_REG_22756_CACHE_PORT_SELECT_BMSK);
	else
		n_cfg_reg |= HWIO_REG_22756_CACHE_PORT_SELECT_BMSK;
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
}

void vidc_pix_cache_enable_cache(u32 b_cache_enable)
{
	u32 n_cfg_reg = 0;

	VIDC_HWIO_IN(REG_22756, &n_cfg_reg);
	if (b_cache_enable)
		n_cfg_reg |= HWIO_REG_22756_CACHE_EN_BMSK;
	else
		n_cfg_reg &= (~HWIO_REG_22756_CACHE_EN_BMSK);
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
}

void vidc_pix_cache_clear_cache_tags(void)
{
	u32 n_cfg_reg = 0;

	VIDC_HWIO_IN(REG_22756, &n_cfg_reg);
	n_cfg_reg |= HWIO_REG_22756_CACHE_TAG_CLEAR_BMSK;
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
	VIDC_HWIO_IN(REG_22756, &n_cfg_reg);
	n_cfg_reg &= (~HWIO_REG_22756_CACHE_TAG_CLEAR_BMSK);
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
}

void vidc_pix_cache_set_halt(u32 b_halt_enable)
{
	u32 n_cfg_reg = 0;

	VIDC_HWIO_IN(REG_22756, &n_cfg_reg);
	if (b_halt_enable)
		n_cfg_reg |= HWIO_REG_22756_CACHE_HALT_BMSK;
	else
		n_cfg_reg &= (~HWIO_REG_22756_CACHE_HALT_BMSK);
	VIDC_HWIO_OUT(REG_22756, n_cfg_reg);
}

void vidc_pix_cache_get_status_idle(u32 *p_idle_status)
{
	VIDC_HWIO_IN(REG_919904, p_idle_status);
}

void vidc_pix_cache_set_ram(u32 n_ram_select)
{
	u32 n_dmi_cfg_reg = 0;

	VIDC_HWIO_IN(REG_261029, &n_dmi_cfg_reg);
	n_dmi_cfg_reg &= (~HWIO_REG_261029_DMI_RAM_SEL_BMSK);
	n_dmi_cfg_reg |= VIDC_SETFIELD(n_ram_select,
			HWIO_REG_261029_AUTO_INC_EN_SHFT,
			HWIO_REG_261029_DMI_RAM_SEL_BMSK);
	VIDC_HWIO_OUT(REG_261029, n_dmi_cfg_reg);
}

void vidc_pix_cache_set_auto_inc_ram_addr(u32 b_auto_inc_enable)
{
	u32 n_dmi_cfg_reg = 0;

	VIDC_HWIO_IN(REG_261029, &n_dmi_cfg_reg);
	if (b_auto_inc_enable)
		n_dmi_cfg_reg |= HWIO_REG_261029_AUTO_INC_EN_BMSK;
	else
		n_dmi_cfg_reg &= (~HWIO_REG_261029_AUTO_INC_EN_BMSK);
	VIDC_HWIO_OUT(REG_261029, n_dmi_cfg_reg);
}


void vidc_pix_cache_read_ram_data(u32 n_src_ram_address,
	u32 n_ram_size, u32 *p_dest_address)
{
	u32 n_count, n_dmi_cfg_reg = 0;

	VIDC_HWIO_IN(REG_261029, &n_dmi_cfg_reg);
	VIDC_HWIO_OUT(REG_576200, n_src_ram_address);
	vidc_pix_cache_set_auto_inc_ram_addr(1);
	for (n_count = 0; n_count < n_ram_size; n_count++) {
		VIDC_HWIO_IN(REG_556274, p_dest_address);
		p_dest_address++;
		VIDC_HWIO_IN(REG_917583, p_dest_address);
		p_dest_address++;
	}
	VIDC_HWIO_OUT(REG_261029, n_dmi_cfg_reg);
}

void vidc_pix_cache_write_ram_data(u32 *p_src_address,
	u32 n_ram_size, u32 n_dest_ram_address)
{
	u32 n_count, n_dmi_cfg_reg = 0;

	VIDC_HWIO_IN(REG_261029, &n_dmi_cfg_reg);
	VIDC_HWIO_OUT(REG_576200, n_dest_ram_address);
	vidc_pix_cache_set_auto_inc_ram_addr(1);
	for (n_count = 0; n_count < n_ram_size; n_count++) {
		VIDC_HWIO_OUT(REG_917583, *p_src_address);
		p_src_address++;
		VIDC_HWIO_OUT(REG_556274, *p_src_address);
		p_src_address++;
	}
	VIDC_HWIO_OUT(REG_261029, n_dmi_cfg_reg);
}

void vidc_pix_cache_get_statistics(
	struct vidc_1080P_pix_cache_statistics_type *p_statistics)
{
	VIDC_HWIO_IN(REG_278310,
		&p_statistics->n_access_miss);
	VIDC_HWIO_IN(REG_421222,
		&p_statistics->n_access_hit);
	VIDC_HWIO_IN(REG_609607,
		&p_statistics->n_axi_req);
	VIDC_HWIO_IN(REG_395232,
		&p_statistics->n_core_req);
	VIDC_HWIO_IN(REG_450146,
		&p_statistics->n_axi_bus);
	VIDC_HWIO_IN(REG_610651,
		&p_statistics->n_core_bus);
}

void vidc_pix_cache_enable_misr(u32 b_misr_enable)
{
   u32 n_misr_cfg_reg = 0;

	VIDC_HWIO_IN(REG_883784, &n_misr_cfg_reg);
	if (b_misr_enable)
		n_misr_cfg_reg |= HWIO_REG_883784_MISR_EN_BMSK;
	else
		n_misr_cfg_reg &=
			(~HWIO_REG_883784_MISR_EN_BMSK);
	VIDC_HWIO_OUT(REG_261029, n_misr_cfg_reg);
}

void vidc_pix_cache_set_misr_interface(u32 n_input_select)
{
	u32 n_misr_cfg_reg = 0;

	VIDC_HWIO_IN(REG_883784, &n_misr_cfg_reg);
	n_misr_cfg_reg &= (~HWIO_REG_883784_INPUT_SEL_BMSK);
	n_misr_cfg_reg |= VIDC_SETFIELD(n_input_select,
			HWIO_REG_883784_INPUT_SEL_SHFT,
			HWIO_REG_883784_INPUT_SEL_BMSK);
	VIDC_HWIO_OUT(REG_261029, n_misr_cfg_reg);
}

void vidc_pix_cache_set_misr_id_filtering(
	struct vidc_1080P_pix_cache_misr_id_filtering_type *p_filter_id)
{
	u32 n_misr_cfg_reg = 0;

	VIDC_HWIO_IN(REG_883784, &n_misr_cfg_reg);
	if (p_filter_id->b_ignore_id)
		n_misr_cfg_reg |=
			HWIO_REG_883784_IGNORE_ID_BMSK;
	else
		n_misr_cfg_reg &=
			(~HWIO_REG_883784_IGNORE_ID_BMSK);
	n_misr_cfg_reg &= (~HWIO_REG_883784_ID_BMSK);
	n_misr_cfg_reg |= VIDC_SETFIELD(p_filter_id->n_id,
			HWIO_REG_883784_ID_SHFT,
			HWIO_REG_883784_ID_BMSK);
	VIDC_HWIO_OUT(REG_261029, n_misr_cfg_reg);
}

void vidc_pix_cache_set_misr_filter_trans(u32 n_no_of_trans)
{
	u32 n_misr_cfg_reg = 0;

	VIDC_HWIO_IN(REG_883784, &n_misr_cfg_reg);
	n_misr_cfg_reg &= (~HWIO_REG_883784_COUNTER_BMSK);
	n_misr_cfg_reg |= VIDC_SETFIELD(n_no_of_trans,
			HWIO_REG_883784_COUNTER_SHFT,
			HWIO_REG_883784_COUNTER_BMSK);
	VIDC_HWIO_OUT(REG_261029, n_misr_cfg_reg);
}

void vidc_pix_cache_get_misr_signatures(
	struct vidc_1080P_pix_cache_misr_signature_type *p_signatures)
{
	VIDC_HWIO_INI(REG_651391, 0,
		&p_signatures->n_signature0);
	VIDC_HWIO_INI(REG_651391, 1,
		&p_signatures->n_signature1);
}

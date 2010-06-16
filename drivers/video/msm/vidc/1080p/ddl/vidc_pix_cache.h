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

#ifndef _VIDEO_CORE_PIXCACHE_
#define _VIDEO_CORE_PIXCACHE_


#include "vidc_type.h"

enum vidc_1080P_pix_cache_port_sel_type{
	VIDC_1080P_PIX_CACHE_PORT_A = 0,
	VIDC_1080P_PIX_CACHE_PORT_B = 1,
	VIDC_1080P_PIX_CACHE_PORT_32BIT = 0x7FFFFFFF
};
enum vidc_1080P_pix_cache_page_size_type{
	VIDC_1080P_PIX_CACHE_PAGE_SIZE_1K = 0,
	VIDC_1080P_PIX_CACHE_PAGE_SIZE_2K = 1,
	VIDC_1080P_PIX_CACHE_PAGE_SIZE_4K = 2
};
struct vidc_1080P_pix_cache_config_type{
	u32 b_cache_enable;
	u32 b_prefetch_en;
	enum vidc_1080P_pix_cache_port_sel_type e_port_select;
	u32 b_statistics_off;
	enum vidc_1080P_pix_cache_page_size_type e_page_size;
};
struct vidc_1080P_pix_cache_statistics_type{
	u32 n_access_miss;
	u32 n_access_hit;
	u32 n_axi_req;
	u32 n_core_req;
	u32 n_axi_bus;
	u32 n_core_bus;
};
struct vidc_1080P_pix_cache_misr_id_filtering_type{
	u32 b_ignore_id;
	u32 n_id;
};
struct vidc_1080P_pix_cache_misr_signature_type{
	u32 n_signature0;
	u32 n_signature1;
};

void vidc_pix_cache_sw_reset(void);
void vidc_pix_cache_init_luma_chroma_base_addr(u32 n_dpb,
	u32 *pn_dpb_luma_offset, u32 *pn_dpb_chroma_offset);
void vidc_pix_cache_set_frame_range(u32 n_luma_size, u32 n_chroma_size);
void vidc_pix_cache_init_config(
	struct vidc_1080P_pix_cache_config_type *p_config);
void vidc_pix_cache_set_prefetch_page_limit(u32 n_page_size_limit);
void vidc_pix_cache_enable_prefetch(u32 b_prefetch_enable);
void vidc_pix_cache_disable_statistics(u32 b_statistics_off);
void vidc_pix_cache_set_port(
	enum vidc_1080P_pix_cache_port_sel_type e_port_select);
void vidc_pix_cache_enable_cache(u32 b_cache_enable);
void vidc_pix_cache_clear_cache_tags(void);
void vidc_pix_cache_set_halt(u32 b_halt_enable);
void vidc_pix_cache_get_status_idle(u32 *p_idle_status);
void vidc_pix_cache_set_ram(u32 n_ram_select);
void vidc_pix_cache_set_auto_inc_ram_addr(u32 b_auto_inc_enable);
void vidc_pix_cache_read_ram_data(u32 n_src_ram_address, u32 n_ram_size,
	u32 *p_dest_address);
void vidc_pix_cache_write_ram_data(u32 *p_src_address, u32 n_ram_size,
	u32 n_dest_ram_address);
void vidc_pix_cache_get_statistics(
	struct vidc_1080P_pix_cache_statistics_type *p_statistics);
void vidc_pix_cache_enable_misr(u32 b_misr_enable);
void vidc_pix_cache_set_misr_interface(u32 n_input_select);
void vidc_pix_cache_set_misr_id_filtering(
	struct vidc_1080P_pix_cache_misr_id_filtering_type *p_filter_id);
void vidc_pix_cache_set_misr_filter_trans(u32 n_no_of_trans);
void vidc_pix_cache_get_misr_signatures(
	struct vidc_1080P_pix_cache_misr_signature_type *p_signatures);
#endif

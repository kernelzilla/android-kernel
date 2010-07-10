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

#include "vcd_ddl_utils.h"
#include "vcd_ddl.h"

#ifdef DDL_PROFILE
static unsigned int g_ddl_dec_t1, g_ddl_enc_t1;
static unsigned int g_ddl_dec_ttotal, g_ddl_enc_ttotal;
static unsigned int g_ddl_dec_count, g_ddl_enc_count;
#endif

#define DDL_FW_CHANGE_ENDIAN

#ifdef DDL_BUF_LOG
static void ddl_print_buffer(struct ddl_context_type *p_ddl_context,
	struct ddl_buf_addr_type *p_buf, u32 idx, u8 *p_str);
static void ddl_print_port(struct ddl_context_type *p_ddl_context,
	struct ddl_buf_addr_type *p_buf);
static void ddl_print_buffer_port(struct ddl_context_type *p_ddl_context,
	struct ddl_buf_addr_type *p_buf, u32 idx, u8 *p_str);
#endif

void *ddl_pmem_alloc(struct ddl_buf_addr_type *p_addr, u32 size, u32 alignment)
{
	u32 alloc_size, offset = 0;

	alloc_size = (size + alignment);
	p_addr->p_physical_base_addr = (u8 *) pmem_kalloc(alloc_size,
		PMEM_MEMTYPE | PMEM_ALIGNMENT_4K);
	if (!p_addr->p_physical_base_addr) {
		DDL_MSG_ERROR("%s() : pmem alloc failed (%d)\n", __func__,
			alloc_size);
		return NULL;
	}
	DDL_MSG_LOW("%s() : pmem alloc physical base addr/size 0x%x / %d \n",\
		__func__, (u32)p_addr->p_physical_base_addr, alloc_size);
	p_addr->p_virtual_base_addr = (u8 *)ioremap((unsigned long)
		p_addr->p_physical_base_addr, alloc_size);
	if (!p_addr->p_virtual_base_addr) {
		DDL_MSG_ERROR("%s() : ioremap failed, virtual(%x)\n", __func__,
			(u32)p_addr->p_virtual_base_addr);
		return NULL;
	}
	DDL_MSG_LOW("%s() : pmem alloc virtual base addr/size 0x%x / %d \n",\
		__func__, (u32)p_addr->p_virtual_base_addr, alloc_size);
	p_addr->p_align_physical_addr = (u8 *) DDL_ALIGN((u32)
		p_addr->p_physical_base_addr, alignment);
	offset = (u32)(p_addr->p_align_physical_addr -
			p_addr->p_physical_base_addr);
	p_addr->p_align_virtual_addr = p_addr->p_virtual_base_addr + offset;
	p_addr->n_buffer_size = size;
	DDL_MSG_LOW("%s() : pmem alloc physical aligned addr/size 0x%x/ %d \n",\
		__func__, (u32)p_addr->p_align_physical_addr, size);
	DDL_MSG_LOW("%s() : pmem alloc virtual aligned addr/size 0x%x / %d \n",\
		__func__, (u32)p_addr->p_virtual_base_addr, size);
	return p_addr->p_virtual_base_addr;
}

void ddl_pmem_free(struct ddl_buf_addr_type *p_addr)
{
	DDL_MSG_LOW("ddl_pmem_free:");
	if (p_addr->p_virtual_base_addr)
		iounmap((void *)p_addr->p_virtual_base_addr);
	if ((p_addr->p_physical_base_addr) &&
		pmem_kfree((s32) p_addr->p_physical_base_addr)) {
		DDL_MSG_LOW("\n %s(): Error in Freeing Physical Address %p",\
			__func__, p_addr->p_physical_base_addr);
	}
	p_addr->p_physical_base_addr   = NULL;
	p_addr->p_virtual_base_addr    = NULL;
	p_addr->p_align_virtual_addr   = NULL;
	p_addr->p_align_physical_addr  = NULL;
	p_addr->n_buffer_size = 0;
}

#ifdef DDL_BUF_LOG

static void ddl_print_buffer(struct ddl_context_type *p_ddl_context,
	struct ddl_buf_addr_type *p_buf, u32 idx, u8 *p_str)
{
	struct ddl_buf_addr_type *p_base_ram;
	s32  offset;
	u32  size, KB = 0;

	p_base_ram = &p_ddl_context->dram_base_a;
	offset = (s32) DDL_ADDR_OFFSET(*p_base_ram, *p_buf);
	size = p_buf->n_buffer_size;
	if (size > 0) {
		if (!(size % 1024)) {
			size /= 1024;
			KB++;
			if (!(size % 1024)) {
				size /= 1024;
				KB++;
			}
		}
	}
	DDL_MSG_LOW("\n%12s [%2d]:  0x%08x [0x%04x],  0x%08x(%d%s),  %s",
		p_str, idx, (u32) p_buf->p_align_physical_addr,
		(offset > 0) ? offset : 0, p_buf->n_buffer_size, size,
		((2 == KB) ? "MB" : (1 == KB) ? "KB" : ""),
		(((u32) p_buf->p_virtual_base_addr) ? "Alloc" : ""));
}

static void ddl_print_port(struct ddl_context_type *p_ddl_context,
	struct ddl_buf_addr_type *p_buf)
{
	struct ddl_buf_addr_type *p_a = &p_ddl_context->dram_base_a;
	struct ddl_buf_addr_type *p_b = &p_ddl_context->dram_base_b;

	if (!p_buf->p_align_physical_addr || !p_buf->n_buffer_size)
		return;
	if (p_buf->p_align_physical_addr >= p_a->p_align_physical_addr &&
		p_buf->p_align_physical_addr + p_buf->n_buffer_size <=
		p_a->p_align_physical_addr + p_a->n_buffer_size)
		DDL_MSG_LOW(" -A [0x%x]-", DDL_ADDR_OFFSET(*p_a, *p_buf));
	else if (p_buf->p_align_physical_addr >= p_b->p_align_physical_addr &&
		p_buf->p_align_physical_addr + p_buf->n_buffer_size <=
		p_b->p_align_physical_addr + p_b->n_buffer_size)
		DDL_MSG_LOW(" -B [0x%x]-", DDL_ADDR_OFFSET(*p_b, *p_buf));
	else
		DDL_MSG_LOW(" -?-");
}

static void ddl_print_buffer_port(struct ddl_context_type *p_ddl_context,
	struct ddl_buf_addr_type *p_buf, u32 idx, u8 *p_str)
{
	DDL_MSG_LOW("\n");
	ddl_print_buffer(p_ddl_context, p_buf, idx, p_str);
	ddl_print_port(p_ddl_context, p_buf);
}

void ddl_list_buffers(struct ddl_client_context_type *p_ddl)
{
	struct ddl_context_type *p_ddl_context;
	u32 i;

	p_ddl_context = p_ddl->p_ddl_context;
	DDL_MSG_LOW("\n\n");
	DDL_MSG_LOW("\n      Buffer     :     Start    [offs],      Size    \
	(Size),     Alloc/Port");
	DDL_MSG_LOW("\n-------------------------------------------------------\
	-------------------------");
	ddl_print_buffer(p_ddl_context, &p_ddl_context->dram_base_a, 0,
		"dram_base_a");
	ddl_print_buffer(p_ddl_context, &p_ddl_context->dram_base_b, 0,
		"dram_base_b");
	if (p_ddl->codec_data.hdr.b_decoding) {
		struct ddl_dec_buffers_type  *p_dec_bufs =
			&p_ddl->codec_data.decoder.hw_bufs;
		for (i = 0; i < 32; i++)
			ddl_print_buffer_port(p_ddl_context,
				&p_dec_bufs->h264Mv[i], i, "h264Mv");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->h264Vert_nb_mv, 0, "h264Vert_nb_mv");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->h264Nb_ip, 0, "h264Nb_ip");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->nb_dcac, 0, "nb_dcac");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->upnb_mv, 0, "upnb_mv");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->sub_anchor_mv, 0, "sub_anchor_mv");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->overlay_xform, 0, "overlay_xform");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->bit_plane3, 0, "bit_plane3");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->bit_plane2, 0, "bit_plane2");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->bit_plane1, 0, "bit_plane1");
		ddl_print_buffer_port(p_ddl_context,
			p_dec_bufs->stx_parser, 0, "stx_parser");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->desc, 0, "desc");
		ddl_print_buffer_port(p_ddl_context,
			&p_dec_bufs->context, 0, "context");
	} else {
		struct ddl_enc_buffers_type  *p_enc_bufs =
			&p_ddl->codec_data.encoder.hw_bufs;

		for (i = 0; i < 4; i++)
			ddl_print_buffer_port(p_ddl_context,
				&p_enc_bufs->dpb_y[i], i, "dpb_y");
		for (i = 0; i < 4; i++)
			ddl_print_buffer_port(p_ddl_context,
				&p_enc_bufs->dpb_c[i], i, "dpb_c");
		ddl_print_buffer_port(p_ddl_context, &p_enc_bufs->mv, 0, "mv");
		ddl_print_buffer_port(p_ddl_context,
			&p_enc_bufs->col_zero, 0, "col_zero");
		ddl_print_buffer_port(p_ddl_context, &p_enc_bufs->md, 0, "md");
		ddl_print_buffer_port(p_ddl_context,
			&p_enc_bufs->pred, 0, "pred");
		ddl_print_buffer_port(p_ddl_context,
			&p_enc_bufs->nbor_info, 0, "nbor_info");
		ddl_print_buffer_port(p_ddl_context,
			&p_enc_bufs->acdc_coef, 0, "acdc_coef");
		ddl_print_buffer_port(p_ddl_context,
			&p_enc_bufs->context, 0, "context");
	}
}
#endif

#ifdef DDL_FW_CHANGE_ENDIAN
static void ddl_fw_change_endian(u8 *fw, u32 n_fw_size)
{
	u32 i = 0;
	u8  temp;
	for (i = 0; i < n_fw_size; i = i + 4) {
		temp = fw[i];
		fw[i] = fw[i+3];
		fw[i+3] = temp;
		temp = fw[i+1];
		fw[i+1] = fw[i+2];
		fw[i+2] = temp;
	}
	return;
}
#endif

u32 ddl_fw_init(struct ddl_buf_addr_type *p_dram_base)
{

	u8 *p_dest_addr;

	p_dest_addr = DDL_GET_ALIGNED_VITUAL(*p_dram_base);
	if (vidc_video_codec_fw_size > p_dram_base->n_buffer_size ||
		!vidc_video_codec_fw)
		return FALSE;
	DDL_MSG_LOW("FW Addr / FW Size : %x/%d", (u32)vidc_video_codec_fw,
		vidc_video_codec_fw_size);
	memcpy(p_dest_addr, vidc_video_codec_fw,
		vidc_video_codec_fw_size);
#ifdef DDL_FW_CHANGE_ENDIAN
	ddl_fw_change_endian(p_dest_addr, vidc_video_codec_fw_size);
#endif
	return TRUE;
}

void ddl_fw_release(void)
{

}

#ifdef DDL_PROFILE
void ddl_get_core_start_time(u8 codec_type)
{
	u32 *p_ddl_t1 = NULL;
	if (!codec_type)
		p_ddl_t1 = &g_ddl_dec_t1;
	else if (codec_type == 1)
		p_ddl_t1 = &g_ddl_enc_t1;

	if (!*p_ddl_t1) {
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		*p_ddl_t1 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
	}
}

void ddl_calc_core_time(u8 codec_type)
{
	u32 *p_ddl_t1 = NULL, *p_ddl_ttotal = NULL,
		*p_ddl_count = NULL;
	if (!codec_type) {
		DDL_MSG_ERROR("\n1080p Core Decode ");
		p_ddl_t1 = &g_ddl_dec_t1;
		p_ddl_ttotal = &g_ddl_dec_ttotal;
		p_ddl_count = &g_ddl_dec_count;
	} else if (codec_type == 1) {
		DDL_MSG_ERROR("\n1080p Core Encode ");
		p_ddl_t1 = &g_ddl_enc_t1;
		p_ddl_ttotal = &g_ddl_enc_ttotal;
		p_ddl_count = &g_ddl_enc_count;
	}

	if (*p_ddl_t1) {
		int ddl_t2;
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		ddl_t2 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
		*p_ddl_ttotal += (ddl_t2 - *p_ddl_t1);
		*p_ddl_count = *p_ddl_count + 1;
		DDL_MSG_ERROR("time %u, average time %u, count %u",
			ddl_t2 - *p_ddl_t1, (*p_ddl_ttotal)/(*p_ddl_count),
			*p_ddl_count);
		*p_ddl_t1 = 0;
	}
}

void ddl_reset_time_variables(u8 codec_type)
{
	if (!codec_type) {
		DDL_MSG_ERROR("\n Reset Decoder time variables");
		g_ddl_dec_t1 = 0;
		g_ddl_dec_ttotal = 0;
		g_ddl_dec_count = 0;
	} else if (codec_type == 1) {
		DDL_MSG_ERROR("\n Reset Encoder time variables ");
		g_ddl_enc_t1 = 0;
		g_ddl_enc_ttotal = 0;
		g_ddl_enc_count = 0;
	}
}
#endif

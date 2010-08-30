/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef MT9T012_H
#define MT9T012_H

#define NEW_MCLK_PCLK

#include <mach/board.h>

extern struct mt9p012_reg_t mt9p012_regs;	/* from mt9p012_reg.c */

struct reg_struct {
	uint16_t vt_pix_clk_div;     /* 0x0300 */
	uint16_t vt_sys_clk_div;     /* 0x0302 */
	uint16_t pre_pll_clk_div;    /* 0x0304 */
	uint16_t pll_multiplier;     /* 0x0306 */
	uint16_t op_pix_clk_div;     /* 0x0308 */
	uint16_t op_sys_clk_div;     /* 0x030A */
	uint16_t scale_m;            /* 0x0404 */
	uint16_t row_speed;          /* 0x3016 */
	uint16_t x_addr_start;       /* 0x3004 */
	uint16_t x_addr_end;         /* 0x3008 */
	uint16_t y_addr_start;       /* 0x3002 */
	uint16_t y_addr_end;         /* 0x3006 */
	uint16_t read_mode;          /* 0x3040 */
	uint16_t x_output_size ;     /* 0x034C */
	uint16_t y_output_size;      /* 0x034E */
	uint16_t line_length_pck;    /* 0x300C */
	uint16_t frame_length_lines; /* 0x300A */
	uint16_t coarse_int_time;    /* 0x3012 */
	uint16_t fine_int_time;      /* 0x3014 */
	uint16_t skew;               /* 0x309E */
};


struct mt9p012_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};


struct mt9p012_reg_t {
	struct reg_struct const *reg_pat;
	uint16_t reg_pat_size;
	struct mt9p012_i2c_reg_conf const *ttbl;
	uint16_t ttbl_size;
	struct mt9p012_i2c_reg_conf const *lctbl;
	uint16_t lctbl_size;
	struct mt9p012_i2c_reg_conf const *rftbl;
	uint16_t rftbl_size;
};


#endif /* MT9T012_H */

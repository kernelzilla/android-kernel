/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <mach/msm_iomap.h>

#include "clock.h"
#include "clock-7x30.h"

struct clk_freq_tbl {
	uint32_t	freq_hz;
	uint32_t	src;
	uint32_t	md_val;
	uint32_t	ns_val;
	uint32_t	mode;
};

struct clk_local {
	uint32_t	count;
	uint32_t	type;
	uint32_t	md_reg;
	uint32_t	ns_reg;
	uint32_t	freq_mask;
	uint32_t	br_en_mask;
	uint32_t	root_en_mask;
	int		parent;
	uint32_t	*children;
	struct clk_freq_tbl	*freq_tbl;
	struct clk_freq_tbl	*current_freq;
};


enum {
	SRC_PLL0 = 4, /* Modem PLL */
	SRC_PLL1 = 1, /* Global PLL */
	SRC_PLL3 = 3, /* Multimedia/Peripheral PLL or Backup PLL1 */
	SRC_PLL4 = 2, /* Display PLL */
	SRC_LPXO = 6, /* Low power XO. */
	SRC_MAX       /* Used for sources that can't be turned on/off. */
};

struct pll_data {
	int count;
	uint32_t bit_mask;
};

static struct pll_data pll_tbl[SRC_MAX] = {
	/* FIXME: Put in proper values for bit_mask. */
	[SRC_PLL0] = { 0, 0 },
	[SRC_PLL1] = { 0, 0 },
	[SRC_PLL3] = { 0, 0 },
	[SRC_PLL4] = { 0, 0 },
};

#define B(x)	BIT(x)
#define BM(msb, lsb)	(((((uint32_t)-1) << (31-msb)) >> (31-msb+lsb)) << lsb)
#define BVAL(msb, lsb, val)	(((val) << lsb) & BM(msb, lsb))

#define MD8(m, n)		(BVAL(15, 8, m) | BVAL(7, 0, ~(n)))
#define N8(msb, lsb, m, n)	(BVAL(msb, lsb, ~(n-m)))
#define MD16(m, n)		(BVAL(31, 16, m) | BVAL(15, 0, ~(n)))
#define N16(m, n)		(BVAL(31, 16, ~(n-m)))
#define SPDIV(s, d)		(BVAL(4, 3, d-1) | BVAL(2, 0, s))
#define SDIV(s, d)		(BVAL(6, 3, d-1) | BVAL(2, 0, s))
#define F_MASK_BASIC		(BM(6, 3)|BM(2, 0))
#define F_MASK_MND16		(BM(31, 16)|BM(4, 3)|BM(2, 0))
#define F_MASK_MND8(m, l)	(BM(m, l)|BM(4, 3)|BM(2, 0))

#define F_RAW(f, s, m_v, n_v, mde) { \
	.freq_hz = f, \
	.src = s, \
	.md_val = m_v, \
	.ns_val = n_v, \
	.mode = mde, \
	}

#define FREQ_END	0
#define F_BASIC(f, s, div) F_RAW(f, s, 0, SDIV(s, div), 0)
#define F_MND16(f, s, div, m, n) \
	F_RAW(f, s, MD16(m, n), N16(m, n)|SPDIV(s, div), !!(n))
#define F_MND8(f, nmsb, nlsb, s, div, m, n) \
	F_RAW(f, s, MD8(m, n), N8(nmsb, nlsb, m, n)|SPDIV(s, div), !!(n))
#define F_END	F_RAW(FREQ_END, SRC_MAX, 0, 0, 0)

static struct clk_freq_tbl clk_tbl_tcxo[] = {
	F_RAW(19200000, SRC_MAX, 0, 0, 0),
	F_END,
};

static struct clk_freq_tbl clk_tbl_uartdm[] = {
	F_MND16( 3686400, SRC_PLL3, 3,   3, 200),
	F_MND16( 7372800, SRC_PLL3, 3,   3, 100),
	F_MND16(14745600, SRC_PLL3, 3,   3,  50),
	F_MND16(46400000, SRC_PLL3, 3, 145, 768),
	F_MND16(51200000, SRC_PLL3, 3,   5,  24),
	F_MND16(58982400, SRC_PLL3, 3,   6,  25),
	F_MND16(64000000, SRC_PLL1, 4,   1,   3),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdh[] = {
	F_BASIC( 73728000, SRC_PLL3, 10),
	F_BASIC( 92160000, SRC_PLL3, 8),
	F_BASIC(122880000, SRC_PLL3, 6),
	F_BASIC(184320000, SRC_PLL3, 4),
	F_BASIC(245760000, SRC_PLL3, 3),
	F_BASIC(368640000, SRC_PLL3, 2),
	F_BASIC(384000000, SRC_PLL1, 2),
	F_END,
};

static struct clk_freq_tbl clk_tbl_grp[] = {
	F_BASIC( 24576000, SRC_LPXO, 1),
	F_BASIC( 46000000, SRC_PLL3, 16),
	F_BASIC( 49000000, SRC_PLL3, 15),
	F_BASIC( 52000000, SRC_PLL3, 14),
	F_BASIC( 56000000, SRC_PLL3, 13),
	F_BASIC( 61440000, SRC_PLL3, 12),
	F_BASIC( 67000000, SRC_PLL3, 11),
	F_BASIC( 73000000, SRC_PLL3, 10),
	F_BASIC( 81000000, SRC_PLL3,  9),
	F_BASIC( 92000000, SRC_PLL3,  8),
	F_BASIC(105000000, SRC_PLL3,  7),
	F_BASIC(120000000, SRC_PLL3,  6),
	F_BASIC(150000000, SRC_PLL3,  5),
	F_BASIC(183000000, SRC_PLL3,  4),
	F_BASIC(192000000, SRC_PLL1,  4),
	F_BASIC(245760000, SRC_PLL3,  3),
	/* Sync to AXI. Hence this "rate" is not fixed. */
	F_RAW(1, SRC_MAX, 0, B(14), 0),
	F_END,
};

static struct clk_freq_tbl clk_tbl_sdc1_3[] = {
	F_MND8(  144000, 19, 12, SRC_LPXO, 1,   1,  171),
	F_MND8(  400000, 19, 12, SRC_LPXO, 1,   2,  123),
	F_MND8(16000000, 19, 12, SRC_PLL3, 3,  14,  215),
	F_MND8(17000000, 19, 12, SRC_PLL3, 4,  19,  206),
	F_MND8(20000000, 19, 12, SRC_PLL3, 4,  23,  212),
	F_MND8(25000000, 19, 12, SRC_LPXO, 1,   0,    0),
	F_MND8(50000000, 19, 12, SRC_PLL3, 3,   1,    5),
	F_END,
};

static struct clk_freq_tbl clk_tbl_sdc2_4[] = {
	F_MND8(  144000, 20, 13, SRC_LPXO, 1,   1,  171),
	F_MND8(  400000, 20, 13, SRC_LPXO, 1,   2,  123),
	F_MND8(16000000, 20, 13, SRC_PLL3, 3,  14,  215),
	F_MND8(17000000, 20, 13, SRC_PLL3, 4,  19,  206),
	F_MND8(20000000, 20, 13, SRC_PLL3, 4,  23,  212),
	F_MND8(25000000, 20, 13, SRC_LPXO, 1,   0,    0),
	F_MND8(50000000, 20, 13, SRC_PLL3, 3,   1,    5),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_core[] = {
	F_BASIC( 46000000, SRC_PLL3, 16),
	F_BASIC( 49000000, SRC_PLL3, 15),
	F_BASIC( 52000000, SRC_PLL3, 14),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_lcdc[] = {
	F_MND16(25000000, SRC_LPXO, 1,   0,   0),
	F_MND16(30000000, SRC_PLL3, 4,   1,   6),
	F_MND16(40000000, SRC_PLL3, 2,   1,   9),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_vsync[] = {
	F_RAW(24576000, SRC_MAX, 0, 0, 0), /* Initialized to LPXO. */
	F_END,
};

static struct clk_freq_tbl clk_tbl_mi2s_codec[] = {
	F_MND16( 2048000, SRC_LPXO, 4,   1,   3),
	F_MND16(12288000, SRC_LPXO, 2,   0,   0),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mi2s[] = {
	F_MND16(12288000, SRC_LPXO, 2,   0,   0),
	F_END,
};

static struct clk_freq_tbl clk_tbl_midi[] = {
	F_MND8(98304000, 19, 12, SRC_PLL3, 3,  2,  5),
	F_END,
};
static struct clk_freq_tbl clk_tbl_sdac[] = {
	F_MND16( 256000, SRC_LPXO, 4,   1,    24),
	F_MND16( 352800, SRC_LPXO, 1, 147, 10240),
	F_MND16( 384000, SRC_LPXO, 4,   1,    16),
	F_MND16( 512000, SRC_LPXO, 4,   1,    12),
	F_MND16( 705600, SRC_LPXO, 1, 147,  5120),
	F_MND16( 768000, SRC_LPXO, 4,   1,     8),
	F_MND16(1024000, SRC_LPXO, 4,   1,     6),
	F_MND16(1411200, SRC_LPXO, 1, 147,  2560),
	F_MND16(1536000, SRC_LPXO, 4,   1,     4),
	F_END,
};

static struct clk_freq_tbl clk_tbl_tv[] = {
	F_MND8(27000000, 23, 16, SRC_PLL4, 2,  2,  33),
	F_MND8(74250000, 23, 16, SRC_PLL4, 2,  1,   6),
	F_END,
};

static struct clk_freq_tbl clk_tbl_usb[] = {
	F_MND8(60000000, 23, 16, SRC_PLL1, 2,  5,  32),
	F_END,
};

static struct clk_freq_tbl clk_tbl_vfe_jpeg[] = {
	F_MND16( 36000000, SRC_PLL3, 4,   1,   5),
	F_MND16( 46000000, SRC_PLL3, 4,   1,   4),
	F_MND16( 61440000, SRC_PLL3, 4,   1,   3),
	F_MND16( 74000000, SRC_PLL3, 2,   1,   5),
	F_MND16( 82000000, SRC_PLL3, 3,   1,   3),
	F_MND16( 92000000, SRC_PLL3, 4,   1,   2),
	F_MND16( 98000000, SRC_PLL3, 3,   2,   5),
	F_MND16(105000000, SRC_PLL3, 2,   2,   7),
	F_MND16(122880000, SRC_PLL3, 2,   1,   3),
	F_MND16(148000000, SRC_PLL3, 2,   2,   5),
	F_MND16(154000000, SRC_PLL1, 2,   2,   5),
	F_END,
};

static struct clk_freq_tbl clk_tbl_cam[] = {
	F_MND16( 6000000, SRC_PLL1, 4,   1,  32),
	F_MND16( 8000000, SRC_PLL1, 4,   1,  24),
	F_MND16(12000000, SRC_PLL1, 4,   1,  16),
	F_MND16(16000000, SRC_PLL1, 4,   1,  12),
	F_MND16(19000000, SRC_PLL1, 4,   1,  10),
	F_MND16(24000000, SRC_PLL1, 4,   1,   8),
	F_MND16(32000000, SRC_PLL1, 4,   1,   6),
	F_MND16(48000000, SRC_PLL1, 4,   1,   4),
	F_MND16(64000000, SRC_PLL1, 4,   1,   3),
	F_END,
};

static struct clk_freq_tbl clk_tbl_vpe[] = {
	F_MND8( 24576000, 22, 15, SRC_LPXO, 1,   0,   0),
	F_MND8( 30720000, 22, 15, SRC_PLL3, 4,   1,   6),
	F_MND8( 61440000, 22, 15, SRC_PLL3, 4,   1,   3),
	F_MND8( 81920000, 22, 15, SRC_PLL3, 3,   1,   3),
	F_MND8(122880000, 22, 15, SRC_PLL3, 3,   1,   2),
	F_MND8(147000000, 22, 15, SRC_PLL3, 1,   1,   5),
	F_MND8(153600000, 22, 15, SRC_PLL1, 1,   1,   5),
	F_MND8(170667000, 22, 15, SRC_PLL1, 1,   2,   9),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mfc[] = {
	F_MND8( 24576000, 24, 17, SRC_LPXO, 1,   0,   0),
	F_MND8( 30720000, 24, 17, SRC_PLL3, 4,   1,   6),
	F_MND8( 61440000, 24, 17, SRC_PLL3, 4,   1,   3),
	F_MND8( 81920000, 24, 17, SRC_PLL3, 3,   1,   3),
	F_MND8(122880000, 24, 17, SRC_PLL3, 3,   1,   2),
	F_MND8(147000000, 24, 17, SRC_PLL3, 1,   1,   5),
	F_MND8(153600000, 24, 17, SRC_PLL1, 1,   1,   5),
	F_MND8(170667000, 24, 17, SRC_PLL1, 1,   2,   9),
	F_END,
};

static struct clk_freq_tbl clk_tbl_spi[] = {
	F_MND8(10000000, 19, 12, SRC_PLL3, 4,   7,   129),
	F_MND8(26000000, 19, 12, SRC_PLL3, 4,  34,   241),
	F_END,
};

static struct clk_freq_tbl clk_tbl_lpa_codec[] = {
	F_RAW(1, SRC_MAX, 0,  0,  0), /* src = MI2S_CODEC_RX */
	F_RAW(2, SRC_MAX, 0,  1,  0), /* src = ECODEC_CIF */
	F_RAW(3, SRC_MAX, 0,  2,  0), /* src = MI2S */
	F_RAW(4, SRC_MAX, 0,  3,  0), /* src = SDAC */
	F_END,
};

static struct clk_freq_tbl dummy_freq = F_END;

#define MND	1 /* Integer predivider and fractional MN:D divider. */
#define BASIC	2 /* Integer divider. */
#define NORATE	3 /* Just on/off. */

#define C(x) L_7X30_##x##_CLK

#define CLK_LOCAL(id, t, md, ns, f_msk, br, root, tbl, par, chld_lst) \
	[C(id)] = { \
	.type = t, \
	.md_reg = md, \
	.ns_reg = ns, \
	.freq_mask = f_msk, \
	.br_en_mask = br, \
	.root_en_mask = root, \
	.parent = C(par), \
	.children = chld_lst, \
	.freq_tbl = tbl, \
	.current_freq = &dummy_freq, \
	}

#define CLK_BASIC(id, ns, br, root, tbl, par) \
		CLK_LOCAL(id, BASIC, 0, ns, F_MASK_BASIC, br, root, tbl, \
								par, NULL)
#define CLK_MND8_P(id, ns, m, l, br, root, tbl, par, chld_lst) \
		CLK_LOCAL(id, MND, (ns-4), ns, F_MASK_MND8(m, l), br, root, \
							tbl, par, chld_lst)
#define CLK_MND8(id, ns, m, l, br, root, tbl, chld_lst) \
		CLK_MND8_P(id, ns, m, l, br, root, tbl, NONE, chld_lst)
#define CLK_MND16(id, ns, br, root, tbl, par, chld_lst) \
		CLK_LOCAL(id, MND, (ns-4), ns, F_MASK_MND16, br, root, tbl, \
								par, chld_lst)
#define CLK_1RATE(id, ns, br, root, tbl) \
		CLK_LOCAL(id, BASIC, 0, ns, 0, br, root, tbl, NONE, NULL)
#define CLK_SLAVE(id, ns, br, par) \
		CLK_LOCAL(id, NORATE, 0, ns, 0, br, 0, NULL, par, NULL)
#define CLK_NORATE(id, ns, br, root) \
		CLK_LOCAL(id, NORATE, 0, ns, 0, br, root, NULL, NONE, NULL)
#define CLK_GLBL(id, glbl, root) \
		CLK_LOCAL(id, NORATE, 0, glbl, 0, 0, root, NULL, NONE, NULL)
#define CLK_BRIDGE(id, glbl, root, par) \
		CLK_LOCAL(id, NORATE, 0, glbl, 0, 0, root, NULL, par, NULL)

#define REG(off) (MSM_CLK_CTL_BASE + off)
#define MNCNTR_EN_MASK		B(8)
#define MNCNTR_RST_MASK		B(7)
#define MNCNTR_MODE_MASK	BM(6, 5)
#define MNCNTR_MODE		BVAL(6, 5, 0x2) /* Dual-edge mode. */

/* Register offsets used more than once. */
#define USBH_MD			0x02BC
#define USBH_NS			0x02C0
#define USBH2_NS		0x046C
#define USBH3_NS		0x0470
#define CAM_VFE_NS		0x0044
#define GLBL_CLK_ENA_SC		0x03BC
#define GLBL_CLK_ENA_2_SC	0x03C0
#define SDAC_NS			0x009C
#define TV_NS			0x00CC
#define MI2S_RX_NS		0x0070
#define MI2S_TX_NS		0x0078
#define MI2S_NS			0x02E0
#define LPA_NS			0x02E8
#define MDC_NS			0x007C
#define MDP_VSYNC_REG		0x0460
#define PLL_ENA_REG		0x0260

static uint32_t chld_grp_3d_src[] = {C(IMEM), C(GRP_3D), C(NONE)};
static uint32_t chld_mdp_lcdc_p[] = {C(MDP_LCDC_PAD_P), C(NONE)};
static uint32_t chld_mi2s_codec_rx[] = {C(MI2S_CODEC_RX_S), C(NONE)};
static uint32_t chld_mi2s_codec_tx[] = {C(MI2S_CODEC_TX_S), C(NONE)};
static uint32_t chld_mi2s[] = {C(MI2S_S), C(NONE)};
static uint32_t chld_sdac_m[] = {C(SDAC_S), C(NONE)};
static uint32_t chld_tv[] = {C(TV_DAC), C(TV_ENC), C(TSIF_REF), C(NONE)};
static uint32_t chld_usb_src[] = {
	C(USB_HS), C(USB_HS_CORE),
	C(USB_HS2), C(USB_HS2_CORE),
	C(USB_HS3), C(USB_HS3_CORE),
	C(NONE),
};
uint32_t chld_vfe[] = {C(VFE_MDC), C(VFE_CAMIF), C(NONE)};

static struct clk_local clk_local_tbl[] = {
	CLK_NORATE(MDC,		MDC_NS, B(9), B(11)),
	CLK_NORATE(LPA_CORE,	LPA_NS, B(5), 0),

	CLK_1RATE(I2C,		0x0068, B(9), B(11),	clk_tbl_tcxo),
	CLK_1RATE(I2C_2,	0x02D8, B(0), B(2),	clk_tbl_tcxo),
	CLK_1RATE(QUP_I2C,	0x04F0, B(0), B(2),	clk_tbl_tcxo),
	CLK_1RATE(UART1,	0x00E0, B(5), B(4),	clk_tbl_tcxo),
	CLK_1RATE(UART3,	0x0468, B(5), B(4),	clk_tbl_tcxo),

	CLK_BASIC(EMDH,	0x0050,    0, B(11),	clk_tbl_mdh, AXI_LI_ADSP_A),
	CLK_BASIC(PMDH,	0x008C,    0, B(11),	clk_tbl_mdh, AXI_LI_ADSP_A),
	CLK_BASIC(MDP,	0x014C, B(9), B(11),	clk_tbl_mdp_core, AXI_MDP),

	CLK_MND8_P(VPE, 0x015C, 22, 15, B(9), B(11), clk_tbl_vpe,
							AXI_VPE, NULL),
	/* Combining MFC and MFC_DIV2 clocks. */
	CLK_MND8_P(MFC, 0x0154, 24, 17, B(9)|B(15), B(11), clk_tbl_mfc,
							AXI_MFC, NULL),

	CLK_MND8(SDC1,	0x00A4, 19, 12, B(9), B(11),	clk_tbl_sdc1_3,	NULL),
	CLK_MND8(SDC2,	0x00AC, 20, 13, B(9), B(11),	clk_tbl_sdc2_4,	NULL),
	CLK_MND8(SDC3,	0x00B4, 19, 12, B(9), B(11),	clk_tbl_sdc1_3,	NULL),
	CLK_MND8(SDC4,	0x00BC, 20, 13, B(9), B(11),	clk_tbl_sdc2_4,	NULL),
	CLK_MND8(SPI,	0x02C8, 19, 12, B(9), B(11),	clk_tbl_spi,	NULL),
	CLK_MND8(MIDI,	0x02D0, 19, 12, B(9), B(11),	clk_tbl_midi,	NULL),
	CLK_MND8_P(USB_HS_SRC, USBH_NS, 23, 16, 0, B(11), clk_tbl_usb,
					AXI_LI_ADSP_A,	chld_usb_src),
	CLK_SLAVE(USB_HS,	USBH_NS,	B(9),	USB_HS_SRC),
	CLK_SLAVE(USB_HS_CORE,	USBH_NS,	B(13),	USB_HS_SRC),
	CLK_SLAVE(USB_HS2,	USBH2_NS,	B(9),	USB_HS_SRC),
	CLK_SLAVE(USB_HS2_CORE,	USBH2_NS,	B(4),	USB_HS_SRC),
	CLK_SLAVE(USB_HS3,	USBH3_NS,	B(9),	USB_HS_SRC),
	CLK_SLAVE(USB_HS3_CORE,	USBH3_NS,	B(4),	USB_HS_SRC),
	CLK_MND8(TV,	TV_NS, 23, 16, 0, B(11), clk_tbl_tv, chld_tv),
	CLK_SLAVE(TV_DAC,	TV_NS, B(12),		TV),
	CLK_SLAVE(TV_ENC,	TV_NS, B(9),		TV),
	/* Hacking root & branch into one param. */
	CLK_SLAVE(TSIF_REF,	0x00C4, B(9)|B(11),	TV),

	CLK_MND16(UART1DM, 0x00D4, B(9), B(11), clk_tbl_uartdm, NONE, NULL),
	CLK_MND16(UART2DM, 0x00DC, B(9), B(11), clk_tbl_uartdm, NONE, NULL),
	CLK_MND16(JPEG,    0x0164, B(9), B(11), clk_tbl_vfe_jpeg,
							AXI_LI_JPEG, NULL),
	CLK_MND16(CAM, 0x0374, 0, B(9), clk_tbl_cam, NONE, NULL),
	CLK_MND16(VFE, CAM_VFE_NS, B(9), B(13), clk_tbl_vfe_jpeg,
							AXI_LI_VFE, chld_vfe),
	CLK_SLAVE(VFE_MDC, CAM_VFE_NS, B(11), VFE),
	CLK_SLAVE(VFE_CAMIF, CAM_VFE_NS, B(15), VFE),

	CLK_MND16(SDAC_M, SDAC_NS, B(12), B(11), clk_tbl_sdac,
							NONE, chld_sdac_m),
	CLK_SLAVE(SDAC_S, SDAC_NS, B(9), SDAC_M),

	CLK_MND16(MDP_LCDC_P, 0x0390, B(9), B(11), clk_tbl_mdp_lcdc,
							NONE, chld_mdp_lcdc_p),
	CLK_SLAVE(MDP_LCDC_PAD_P, 0x0390, B(12), MDP_LCDC_P),
	CLK_1RATE(MDP_VSYNC, MDP_VSYNC_REG, B(0), 0, clk_tbl_mdp_vsync),

	CLK_MND16(MI2S_CODEC_RX_M, MI2S_RX_NS, B(12), B(11),
				clk_tbl_mi2s_codec, NONE, chld_mi2s_codec_rx),
	CLK_SLAVE(MI2S_CODEC_RX_S, MI2S_RX_NS, B(9), MI2S_CODEC_RX_M),

	CLK_MND16(MI2S_CODEC_TX_M, MI2S_TX_NS, B(12), B(11),
				clk_tbl_mi2s_codec, NONE, chld_mi2s_codec_tx),
	CLK_SLAVE(MI2S_CODEC_TX_S, MI2S_TX_NS, B(9), MI2S_CODEC_TX_M),

	CLK_MND16(MI2S_M, MI2S_NS, B(12), B(11),
				clk_tbl_mi2s, NONE, chld_mi2s),
	CLK_SLAVE(MI2S_S, MI2S_NS, B(9), MI2S_M),

	CLK_LOCAL(GRP_2D, BASIC, 0, 0x0034, F_MASK_BASIC | (7 << 12),
			B(7), B(11), clk_tbl_grp, AXI_GRP_2D, NULL),
	CLK_LOCAL(GRP_3D_SRC, BASIC, 0, 0x0084, F_MASK_BASIC | (7 << 12),
			0, B(11), clk_tbl_grp, AXI_LI_GRP, chld_grp_3d_src),
	CLK_SLAVE(GRP_3D, 0x0084, B(7), GRP_3D_SRC),
	CLK_SLAVE(IMEM, 0x0084, B(9), GRP_3D_SRC),
	CLK_LOCAL(LPA_CODEC, BASIC, 0, LPA_NS, BM(1, 0), B(9), 0,
					clk_tbl_lpa_codec, NONE, NULL),

	/* Peripheral bus clocks. */
	CLK_GLBL(ADM,	 	GLBL_CLK_ENA_SC,	B(5)),
	CLK_GLBL(CAMIF_PAD_P,	GLBL_CLK_ENA_SC,	B(9)),
	CLK_GLBL(EMDH_P,	GLBL_CLK_ENA_2_SC,	B(3)),
	CLK_GLBL(GRP_2D_P,	GLBL_CLK_ENA_SC,	B(24)),
	CLK_GLBL(GRP_3D_P,	GLBL_CLK_ENA_2_SC,	B(17)),
	CLK_GLBL(JPEG_P,	GLBL_CLK_ENA_2_SC,	B(24)),
	CLK_GLBL(LPA_P,		GLBL_CLK_ENA_2_SC,	B(7)),
	CLK_GLBL(MDP_P,		GLBL_CLK_ENA_2_SC,	B(6)),
	CLK_GLBL(MFC_P,		GLBL_CLK_ENA_2_SC,	B(26)),
	CLK_GLBL(PMDH_P,	GLBL_CLK_ENA_2_SC,	B(4)),
	CLK_GLBL(SDC1_H,	GLBL_CLK_ENA_SC,	B(7)),
	CLK_GLBL(SDC2_H,	GLBL_CLK_ENA_SC,	B(8)),
	CLK_GLBL(SDC3_H,	GLBL_CLK_ENA_SC,	B(27)),
	CLK_GLBL(SDC4_H,	GLBL_CLK_ENA_SC,	B(28)),
	CLK_GLBL(SPI_P,		GLBL_CLK_ENA_2_SC,	B(10)),
	CLK_GLBL(TSIF_P,	GLBL_CLK_ENA_SC,	B(18)),
	CLK_GLBL(UART1DM_P,	GLBL_CLK_ENA_SC,	B(17)),
	CLK_GLBL(UART2DM_P,	GLBL_CLK_ENA_SC,	B(26)),
	CLK_GLBL(USB_HS2_P,	GLBL_CLK_ENA_2_SC,	B(8)),
	CLK_GLBL(USB_HS3_P,	GLBL_CLK_ENA_2_SC,	B(9)),
	CLK_GLBL(USB_HS_P,	GLBL_CLK_ENA_SC,	B(25)),
	CLK_GLBL(VFE_P,		GLBL_CLK_ENA_2_SC,	B(27)),

	/* AXI bridge clocks. */
	CLK_BRIDGE(AXI_LI_APPS,	GLBL_CLK_ENA_SC,	B(2),	NONE),
	CLK_BRIDGE(AXI_LI_ADSP_A, GLBL_CLK_ENA_2_SC,	B(14),	AXI_LI_APPS),
	CLK_BRIDGE(AXI_LI_JPEG,	GLBL_CLK_ENA_2_SC,	B(19),	AXI_LI_APPS),
	CLK_BRIDGE(AXI_LI_VFE,	GLBL_CLK_ENA_SC,	B(23),	AXI_LI_APPS),
	CLK_BRIDGE(AXI_MDP,	GLBL_CLK_ENA_2_SC,	B(29),	AXI_LI_APPS),

	CLK_BRIDGE(AXI_IMEM,	GLBL_CLK_ENA_2_SC,	B(18),	NONE),

	CLK_BRIDGE(AXI_LI_VG,	GLBL_CLK_ENA_SC,	B(3),	NONE),
	CLK_BRIDGE(AXI_GRP_2D,	GLBL_CLK_ENA_SC,	B(21),	AXI_LI_VG),
	CLK_BRIDGE(AXI_LI_GRP,	GLBL_CLK_ENA_SC,	B(22),	AXI_LI_VG),
	CLK_BRIDGE(AXI_MFC,	GLBL_CLK_ENA_2_SC,	B(20),	AXI_LI_VG),
	CLK_BRIDGE(AXI_VPE,	GLBL_CLK_ENA_2_SC,	B(21),	AXI_LI_VG),
};

static DEFINE_SPINLOCK(clock_reg_lock);

void pll_enable(uint32_t pll)
{
	struct pll_data *p;
	uint32_t reg_val;

	/* SRC_MAX is used as a placeholder for some freqencies that don't
	 * have any direct PLL dependency. */
	if (pll == SRC_MAX || pll == SRC_LPXO)
		return;

	p = &pll_tbl[pll];
	if (!p->count) {
		reg_val = readl(REG(PLL_ENA_REG));
		reg_val |= p->bit_mask;
		writel(reg_val, REG(PLL_ENA_REG));
	}
	p->count++;
}

void pll_disable(uint32_t pll)
{
	struct pll_data *p;
	uint32_t reg_val;

	/* SRC_MAX is used as a placeholder for some freqencies that don't
	 * have any direct PLL dependency. */
	if (pll == SRC_MAX || pll == SRC_LPXO)
		return;

	p = &pll_tbl[pll];
	if (!p->count) {
		pr_warning("Reference count mismatch in PLL disable!\n");
		return;
	}
	if (p->count)
		p->count--;
	if (p->count == 0) {
		reg_val = readl(REG(PLL_ENA_REG));
		reg_val &= ~(p->bit_mask);
		writel(reg_val, REG(PLL_ENA_REG));
	}
}

/*
 * SoC specific register-based control of clocks.
 */
static int _soc_clk_enable(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	void *ns_reg = REG(t->ns_reg);
	uint32_t reg_val = 0;

	reg_val = readl(ns_reg);
	if (t->type == MND) {
		/* mode can be either 0 or 1. So the R-value of the
		 * expression will evaluate to MNCNTR_EN_MASK or 0. This
		 * avoids the need for a "if(mode == 1)". A "&" will not work
		 * here. */
		reg_val |= (MNCNTR_EN_MASK * t->current_freq->mode);
		writel(reg_val, ns_reg);
	}
	if (t->root_en_mask) {
		reg_val |= t->root_en_mask;
		writel(reg_val, ns_reg);
	}
	if (t->br_en_mask) {
		reg_val |= t->br_en_mask;
		writel(reg_val, ns_reg);
	}
	return 0;
}

static void _soc_clk_disable(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	void *ns_reg = REG(t->ns_reg);
	uint32_t reg_val = 0;

	reg_val = readl(ns_reg);

	if (t->br_en_mask) {
		reg_val &= ~(t->br_en_mask);
		writel(reg_val, ns_reg);
	}
	if (t->root_en_mask) {
		reg_val &= ~(t->root_en_mask);
		writel(reg_val, ns_reg);
	}
	if (t->type == MND) {
		reg_val &= ~MNCNTR_EN_MASK;
		writel(reg_val, ns_reg);
	}
}

static int soc_clk_enable_nolock(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	int ret = 0;

	if (!t->count) {
		if (t->parent != C(NONE))
			soc_clk_enable_nolock(t->parent);
		pll_enable(t->current_freq->src);
		ret = _soc_clk_enable(id);
	}
	t->count++;

	return ret;
}

static void soc_clk_disable_nolock(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];

	if (!t->count) {
		pr_warning("Reference count mismatch in clock disable!\n");
		return;
	}
	if (t->count)
		t->count--;
	if (t->count == 0) {
		_soc_clk_disable(id);
		pll_disable(t->current_freq->src);
		if (t->parent != C(NONE))
			soc_clk_disable_nolock(t->parent);
	}

	return;
}

static int soc_clk_enable(unsigned id)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	ret = soc_clk_enable_nolock(id);
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	return ret;
}

static void soc_clk_disable(unsigned id)
{
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	soc_clk_disable_nolock(id);
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	return;
}

static int soc_clk_set_rate(unsigned id, unsigned rate)
{
	struct clk_local *t = &clk_local_tbl[id];
	struct clk_freq_tbl *cf = t->current_freq;
	struct clk_freq_tbl *nf;
	uint32_t *chld = t->children;
	void *ns_reg = REG(t->ns_reg);
	void *md_reg = REG(t->md_reg);
	uint32_t reg_val = 0;
	int i, ret = 0;
	unsigned long flags;

	if (t->type != MND && t->type != BASIC)
		return -EPERM;

	spin_lock_irqsave(&clock_reg_lock, flags);

	if (rate == cf->freq_hz)
		goto release_lock;

	for (nf = t->freq_tbl; nf->freq_hz != FREQ_END; nf++)
		if (nf->freq_hz == rate)
			break;

	if (nf->freq_hz == FREQ_END) {
		ret = -EINVAL;
		goto release_lock;
	}

	if (t->freq_mask == 0) {
		t->current_freq = nf;
		goto release_lock;
	}

	/* Disable all branches before changing rate to prevent jitter. */
	for (i = 0; chld && chld[i] != C(NONE); i++) {
		struct clk_local *ch = &clk_local_tbl[chld[i]];
		/* Don't bother turning off if it is already off.
		 * Checking ch->count is cheaper (cache) than reading and
		 * writing to a register (uncached/unbuffered). */
		if (ch->count) {
			reg_val = readl(REG(ch->ns_reg));
			reg_val &= ~(ch->br_en_mask);
			writel(reg_val, REG(ch->ns_reg));
		}
	}
	if (t->count)
		_soc_clk_disable(id);

	/* Turn on PLL of the new freq. */
	pll_enable(nf->src);

	/* Some clocks share the same register, so must be careful when
	 * assuming a register doesn't need to be re-read. */
	reg_val = readl(ns_reg);
	if (t->type == MND) {
		reg_val |= MNCNTR_RST_MASK;
		writel(reg_val, ns_reg);
		/* TODO: Currently writing 0's into reserved bits for 8-bit
		 * MND. Can be avoided by adding md_mask. */
		if (nf->mode)
			writel(nf->md_val, md_reg);
		reg_val &= ~MNCNTR_MODE_MASK;
		reg_val |= (MNCNTR_MODE * nf->mode);
	}
	reg_val &= ~(t->freq_mask);
	reg_val |= nf->ns_val;
	writel(reg_val, ns_reg);

	if (t->type == MND) {
		reg_val &= ~MNCNTR_RST_MASK;
		writel(reg_val, ns_reg);
	}

	/* Turn off PLL of the old freq. */
	pll_disable(cf->src);

	/* Current freq must be updated before _soc_clk_enable() is called to
	 * make sure the MNCNTR_E bit is set correctly. */
	t->current_freq = nf;

	if (t->count)
		_soc_clk_enable(id);
	/* Enable only branches that were ON before. */
	for (i = 0; chld && chld[i] != C(NONE); i++) {
		struct clk_local *ch = &clk_local_tbl[chld[i]];
		if (ch->count) {
			reg_val = readl(REG(ch->ns_reg));
			reg_val |= ch->br_en_mask;
			writel(reg_val, REG(ch->ns_reg));
		}
	}

release_lock:
	spin_unlock_irqrestore(&clock_reg_lock, flags);
	return ret;
}

static int soc_clk_set_min_rate(unsigned id, unsigned rate)
{
	return -EPERM;
}

static int soc_clk_set_max_rate(unsigned id, unsigned rate)
{
	return -EPERM;
}

static int soc_clk_set_flags(unsigned id, unsigned flags)
{
	return -EPERM;
}

static unsigned soc_clk_get_rate(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	unsigned long flags;
	unsigned ret = 0;

	spin_lock_irqsave(&clock_reg_lock, flags);
	if (t->type == MND && t->type == BASIC)
		ret = t->current_freq->freq_hz;
	else {
		/* Walk up the tree to see if any parent has a rate. */
		while (t->type == NORATE && t->parent != C(NONE))
			t = &clk_local_tbl[t->parent];
		if (t->type == MND || t->type == BASIC)
			ret = t->current_freq->freq_hz;
	}
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	/* Return 0 if the rate has never been set. Might not be correct,
	 * but it's good enough. */
	if (ret == FREQ_END)
		ret = 0;

	return ret;
}

static unsigned soc_clk_is_enabled(unsigned id)
{
	return !!(clk_local_tbl[id].count);
}

static long soc_clk_round_rate(unsigned id, unsigned rate)
{
	struct clk_local *t = &clk_local_tbl[id];
	struct clk_freq_tbl *f;

	if (t->type != MND && t->type != BASIC)
		return -EINVAL;

	for (f = t->freq_tbl; f->freq_hz != FREQ_END; f++)
		if (f->freq_hz >= rate)
			return f->freq_hz;

	return -EPERM;
}

struct clk_ops clk_ops_7x30 = {
	.enable = soc_clk_enable,
	.disable = soc_clk_disable,
	.set_rate = soc_clk_set_rate,
	.set_min_rate = soc_clk_set_min_rate,
	.set_max_rate = soc_clk_set_max_rate,
	.set_flags = soc_clk_set_flags,
	.get_rate = soc_clk_get_rate,
	.is_enabled = soc_clk_is_enabled,
	.round_rate = soc_clk_round_rate,
};

#if 0
static struct reg_init {
	void *reg;
	uint32_t mask;
	uint32_t val;
} ri_list[] __initdata = {
	/* TODO: Remove next line from commercial code. */
	{REG(PLL_ENA_REG), 0x7F, 0x7F}, /* Turn on all PLLs. */

	/* Enable UMDX_P clock. Known to causes issues, so never turn off. */
	{REG(GLBL_CLK_ENA_2_SC), B(2), B(2)},
	{REG(0x0050), 0x3 << 17, 0x3}, /* EMDH RX div = div-4. */
	{REG(0x008C), 0x3 << 17, 0x3}, /* PMDH RX div = div-4. */
	/* MI2S_CODEC_RX_S src = MI2S_CODEC_RX_M. */
	{REG(MI2S_RX_NS), B(14), 0x0},
	/* MI2S_CODEC_TX_S src = MI2S_CODEC_TX_M. */
	{REG(MI2S_TX_NS), B(14), 0x0},
	{REG(MI2S_NS), B(14), 0x0}, /* MI2S_S src = MI2S_M. */
	{REG(LPA_NS), B(4), B(4)}, /* LPA CORE src = LPA_CODEC. */
	{REG(0x02EC), 0xF, 0xD}, /* MI2S_CODEC_RX_S div = div-8. */
	{REG(0x02F0), 0xF, 0xD}, /* MI2S_CODEC_TX_S div = div-8. */
	{REG(0x02E4), 0xF, 0x3}, /* MI2S_S div = div-4. */
	{REG(MDC_NS), 0x3, 0x3}, /* MDC src = external MDH src. */
	{REG(SDAC_NS), 0x3 << 14, 0x0}, /* SDAC div = div-1. */
	/* Disable sources TCXO/5 & TCXO/6. UART1 src = TCXO*/
	{REG(0x00E0), 0x3 << 25 | 0x7, 0x0},
	{REG(0x0468), 0x7, 0x0}, /* UART3 src = TCXO. */
	{REG(MDP_VSYNC_REG), 0xC, 0x4}, /* MDP VSYNC src = LPXO. */

	/* USBH core clocks src = USB_HS_SRC. */
	{REG(USBH_NS), B(15), B(15)},
	{REG(USBH2_NS), B(6), B(6)},
	{REG(USBH3_NS), B(6), B(6)},
};

#define set_1rate(clk) \
	soc_clk_set_rate(C(clk), clk_local_tbl[C(clk)].freq_tbl->freq_hz)
static __init int soc_clk_init(void)
{
	int i;
	uint32_t val;

	/* Disable all the child clocks of USB_HS_SRC. This needs to be done
	 * before the register init loop since it changes the source of the
	 * USB HS core clocks. */
	for (i = 0; chld_usb_src[i] != C(NONE); i++)
		_soc_clk_disable(chld_usb_src[i]);

	soc_clk_set_rate(C(USB_HS_SRC), clk_tbl_usb[0].freq_hz);

	for (i = 0; i < ARRAY_SIZE(ri_list); i++) {
		val = readl(ri_list[i].reg);
		val &= ~ri_list[i].mask;
		val |= ri_list[i].val;
		writel(val, ri_list[i].reg);
	}

	/* This is just to update the driver data structures. The actual
	 * register set up is taken care of in the register init loop. */
	set_1rate(I2C);
	set_1rate(I2C_2);
	set_1rate(QUP_I2C);
	set_1rate(UART1);
	set_1rate(UART3);

	return 0;
}

arch_initcall(soc_clk_init);
#endif

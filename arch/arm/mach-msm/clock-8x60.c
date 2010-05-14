/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <mach/msm_iomap.h>
#include <mach/clk.h>

#include "clock.h"
#include "clock-8x60.h"

/* MUX source input identifiers. */
#define NONE_SRC	-1
#define BB_PXO_SRC	0
#define	BB_MXO_SRC	1
#define BB_CXO_SRC	BB_PXO_SRC
#define	BB_PLL0_SRC	2
#define	BB_PLL8_SRC	3
#define	BB_PLL6_SRC	4
#define	MM_PXO_SRC	0
#define MM_PLL0_SRC	1
#define	MM_PLL1_SRC	1
#define	MM_PLL2_SRC	3
#define	MM_GPERF_SRC	2
#define	MM_GPLL0_SRC	3
#define	MM_MXO_SRC	4
#define XO_CXO_SRC	0
#define XO_PXO_SRC	1
#define XO_MXO_SRC	2
#define LPA_PXO_SRC	0
#define LPA_CXO_SRC	1

/* Source name to PLL mappings. */
#define NONE_PLL	-1
#define BB_PXO_PLL	NONE_PLL
#define	BB_MXO_PLL	NONE_PLL
#define	BB_CXO_PLL	NONE_PLL
#define	BB_PLL0_PLL	PLL_0
#define	BB_PLL8_PLL	PLL_8
#define	BB_PLL6_PLL	PLL_6
#define	MM_PXO_PLL	NONE_PLL
#define MM_PLL0_PLL	PLL_1
#define	MM_PLL1_PLL	PLL_2
#define	MM_PLL2_PLL	PLL_3
#define	MM_GPERF_PLL	PLL_8
#define	MM_GPLL0_PLL	PLL_0
#define	MM_MXO_PLL	NONE_PLL
#define XO_CXO_PLL	NONE_PLL
#define XO_PXO_PLL	NONE_PLL
#define XO_MXO_PLL	NONE_PLL
#define LPA_PXO_PLL	NONE_PLL
#define LPA_CXO_PLL	NONE_PLL

/* Bit manipulation macros. */
#define B(x)	BIT(x)
#define BM(msb, lsb)	(((((uint32_t)-1) << (31-msb)) >> (31-msb+lsb)) << lsb)
#define BVAL(msb, lsb, val)	(((val) << lsb) & BM(msb, lsb))

#define C(x) L_8X60_##x##_CLK
#define REG(off)	(MSM_CLK_CTL_BASE + off)
#define REG_MM(off)	(MSM_MMSS_CLK_CTL_BASE + off)
#define REG_LPA(off)	(MSM_LPASS_CLK_CTL_BASE + off)

#define MND		1 /* Integer predivider and fractional MN:D divider. */
#define BASIC		2 /* Integer divider. */
#define NORATE		3 /* Just on/off. */
#define RESET		4 /* Reset only. */

#define CLK_LOCAL(id, t, ns_r, cc_r, md_r, r_r, r_m, br, root, n_m, c_m, \
			s_fn, tbl, bmnd, par, chld_lst) \
	[C(id)] = { \
	.type = t, \
	.ns_reg = ns_r, \
	.cc_reg = cc_r, \
	.md_reg = md_r, \
	.reset_reg = r_r, \
	.reset_mask = r_m, \
	.br_en_mask = br, \
	.root_en_mask = root, \
	.ns_mask = n_m, \
	.cc_mask = c_m, \
	.parent = C(par), \
	.children = chld_lst, \
	.set_rate = s_fn, \
	.freq_tbl = tbl, \
	.banked_mnd_masks = bmnd, \
	.current_freq = &dummy_freq, \
	}

#define F_RAW_PLL(f, p, m_v, n_v, c_v, m_m, p_r) { \
	.freq_hz = f, \
	.pll = p, \
	.md_val = m_v, \
	.ns_val = n_v, \
	.cc_val = c_v, \
	.mnd_en_mask = m_m, \
	.pll_rate = p_r, \
	}
#define F_RAW(f, p, m_v, n_v, c_v, m_m) \
		F_RAW_PLL(f, p, m_v, n_v, c_v, m_m, 0)
#define FREQ_END	0
#define F_END	F_RAW(FREQ_END, NONE_PLL, 0, 0, 0, 0)

#define PLL_RATE(r, l, m, n, v, d) { l, m, n, v, (d>>1) }
struct pll_rate {
	uint32_t	l_val;
	uint32_t	m_val;
	uint32_t	n_val;
	uint32_t	vco;
	uint32_t	post_div;
};

struct clk_freq_tbl {
	uint32_t	freq_hz;
	int		pll;
	uint32_t	md_val;
	uint32_t	ns_val;
	uint32_t	cc_val;
	uint32_t	mnd_en_mask;
	struct pll_rate *pll_rate;
};

/* Some clocks have two banks to avoid glitches when switching frequencies.
 * The unused bank is programmed while running on the other bank, and
 * switched to afterwards. The following two structs describe the banks. */
struct bank_mask_info {
	void		*md_reg;
	uint32_t	ns_mask;
	uint32_t	rst_mask;
	uint32_t	mnd_en_mask;
	uint32_t	mode_mask;
};
struct banked_mnd_masks {
	uint32_t		bank_sel_mask;
	struct bank_mask_info	bank0_mask;
	struct bank_mask_info	bank1_mask;
};

struct clk_local {
	uint32_t	count;
	uint32_t	type;
	void		*ns_reg;
	void		*cc_reg;
	void		*md_reg;
	void		*reset_reg;
	uint32_t	reset_mask;
	uint32_t	br_en_mask;
	uint32_t	root_en_mask;
	uint32_t	ns_mask;
	uint32_t	cc_mask;
	int		parent;
	uint32_t	*children;
	void		(*set_rate)(struct clk_local *, struct clk_freq_tbl *);
	struct clk_freq_tbl	*freq_tbl;
	struct banked_mnd_masks	*banked_mnd_masks;
	struct clk_freq_tbl	*current_freq;
};

/* _soc_clk_set_rate() match types. */
enum match_types {
	MATCH_EXACT,
	MATCH_MIN,
};

/*
 * Set-Rate Functions
 */
static void set_rate_basic(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	uint32_t reg_val;

	reg_val = readl(clk->ns_reg);
	reg_val &= ~(clk->ns_mask);
	reg_val |= nf->ns_val;
	writel(reg_val, clk->ns_reg);
}

static void set_rate_mnd(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	uint32_t ns_reg_val, cc_reg_val;

	/* Assert MND reset. */
	ns_reg_val = readl(clk->ns_reg);
	ns_reg_val |= B(7);
	writel(ns_reg_val, clk->ns_reg);

	/* Program M and D values. */
	writel(nf->md_val, clk->md_reg);

	/* Program NS register. */
	ns_reg_val &= ~(clk->ns_mask);
	ns_reg_val |= nf->ns_val;
	writel(ns_reg_val, clk->ns_reg);

	/* If the clock has a separate CC register, program it. */
	if (clk->ns_reg != clk->cc_reg) {
		cc_reg_val = readl(clk->cc_reg);
		cc_reg_val &= ~(clk->cc_mask);
		cc_reg_val |= nf->cc_val;
		writel(cc_reg_val, clk->cc_reg);
	}

	/* Deassert MND reset. */
	ns_reg_val &= ~B(7);
	writel(ns_reg_val, clk->ns_reg);
}

static void set_rate_cam(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	uint32_t ns_reg_val, cc_reg_val;

	/* Assert MND reset. */
	cc_reg_val = readl(clk->cc_reg);
	cc_reg_val |= B(8);
	writel(cc_reg_val, clk->cc_reg);

	/* Program M and D values. */
	writel(nf->md_val, clk->md_reg);

	/* Program MN counter Enable and Mode. */
	cc_reg_val &= ~(clk->cc_mask);
	cc_reg_val |= nf->cc_val;
	writel(cc_reg_val, clk->cc_reg);

	/* Program N value, divider and source. */
	ns_reg_val = readl(clk->ns_reg);
	ns_reg_val &= ~(clk->ns_mask);
	ns_reg_val |= nf->ns_val;
	writel(ns_reg_val, clk->ns_reg);

	/* Deassert MND reset. */
	cc_reg_val &= ~B(8);
	writel(cc_reg_val, clk->cc_reg);
}

/* Unlike other clocks, the TV rate is adjusted through PLL
 * re-programming. It is also routed through an MND divider. */
static void set_rate_tv(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	struct pll_rate *rate = nf->pll_rate;
	uint32_t pll_mode, pll_config;

	/* Assert active-low PLL reset. */
	pll_mode = readl(REG_MM(0x0338));
	pll_mode &= ~B(2);
	writel(pll_mode, REG_MM(0x0338));

	/* Program L, M and N values. */
	writel(rate->l_val, REG_MM(0x033C));
	writel(rate->m_val, REG_MM(0x0340));
	writel(rate->n_val, REG_MM(0x0344));

	/* Configure post-divide and VCO. */
	pll_config = readl(REG_MM(0x0348));
	pll_config &= ~(BM(21, 20) | BM(17, 16));
	pll_config |= (BVAL(21, 20, rate->post_div));
	pll_config |= (BVAL(17, 16, rate->vco));
	writel(pll_config, REG_MM(0x0348));

	/* Configure MND. */
	set_rate_mnd(clk, nf);

	/* De-assert active-low PLL reset. */
	pll_mode |= B(2);
	writel(pll_mode, REG_MM(0x0338));
}

static void set_rate_mnd_banked(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	struct banked_mnd_masks *banks = clk->banked_mnd_masks;
	struct bank_mask_info *new_bank_masks, *old_bank_masks;
	uint32_t ns_reg_val, cc_reg_val;
	uint32_t bank_sel;

	/* Determine active bank and program the other one. If the clock is
	 * off, program the active bank since bank switching won't work if
	 * both banks aren't running. */
	cc_reg_val = readl(clk->cc_reg);
	bank_sel = !!(cc_reg_val & banks->bank_sel_mask);
	 /* If clock is disabled, don't switch banks. */
	bank_sel ^= !(clk->count);
	if (bank_sel == 0) {
		new_bank_masks = &banks->bank1_mask;
		old_bank_masks = &banks->bank0_mask;
	} else {
		new_bank_masks = &banks->bank0_mask;
		old_bank_masks = &banks->bank1_mask;
	}

	ns_reg_val = readl(clk->ns_reg);

	/* Assert bank MND reset. */
	ns_reg_val |= new_bank_masks->rst_mask;
	writel(ns_reg_val, clk->ns_reg);

	writel(nf->md_val, new_bank_masks->md_reg);

	/* Enable counter only if clock is enabled. */
	if (clk->count)
		cc_reg_val |= new_bank_masks->mnd_en_mask;
	else
		cc_reg_val &= ~(new_bank_masks->mnd_en_mask);

	cc_reg_val &= ~(new_bank_masks->mode_mask);
	cc_reg_val |= (nf->cc_val & new_bank_masks->mode_mask);
	writel(cc_reg_val, clk->cc_reg);

	ns_reg_val &= ~(new_bank_masks->ns_mask);
	ns_reg_val |= (nf->ns_val & new_bank_masks->ns_mask);
	writel(ns_reg_val, clk->ns_reg);

	/* Deassert bank MND reset. */
	ns_reg_val &= ~(new_bank_masks->rst_mask);
	writel(ns_reg_val, clk->ns_reg);

	/* Switch to the new bank if clock is on.  If it isn't, then no
	 * switch is necessary since we programmed the active bank. */
	if (clk->count) {
		cc_reg_val ^= banks->bank_sel_mask;
		writel(cc_reg_val, clk->cc_reg);

		/* Disable previous MN counter. */
		cc_reg_val &= ~(old_bank_masks->mnd_en_mask);
		writel(cc_reg_val, clk->cc_reg);
	}

	/* If this freq requires the MN counter to be enabled,
	 * update the enable mask to match the current bank. */
	if (nf->mnd_en_mask)
		nf->mnd_en_mask = new_bank_masks->mnd_en_mask;
}

static void set_rate_div_banked(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	uint32_t ns_reg_val, ns_mask, bank_sel;

	/* Determine active bank and program the other one. If the clock is
	 * off, program the active bank since bank switching won't work if
	 * both banks aren't running. */
	ns_reg_val = readl(clk->ns_reg);
	bank_sel = !!(ns_reg_val & B(30));
	 /* If clock is disabled, don't switch banks. */
	bank_sel ^= !(clk->count);
	if (bank_sel == 0)
		ns_mask = (BM(29, 26) | BM(21, 19));
	else
		ns_mask = (BM(25, 22) | BM(18, 16));

	ns_reg_val &= ~(ns_mask);
	ns_reg_val |= (nf->ns_val & ns_mask);
	writel(ns_reg_val, clk->ns_reg);

	/* Switch to the new bank if clock is on.  If it isn't, then no
	 * switch is necessary since we programmed the active bank. */
	if (clk->count) {
		ns_reg_val ^= B(30);
		writel(ns_reg_val, clk->ns_reg);
	}
}

static void set_rate_nop(struct clk_local *clk, struct clk_freq_tbl *nf)
{
	/* Nothing to do for fixed-rate clocks. */
}

/*
 * Generic clock declaration macros
 */
#define CLK_NORATE(id, reg, br) \
		CLK_LOCAL(id, NORATE, NULL, REG(reg), NULL, NULL, 0, \
				br, 0, 0, 0, NULL, NULL, NULL, NONE, NULL)

#define CLK_SLAVE(id, reg, br, par) \
		CLK_LOCAL(id, NORATE, NULL, REG(reg), NULL, NULL, 0, \
				br, 0, 0, 0, NULL, NULL, NULL, par, NULL)

#define CLK_RESET(id, ns, res) \
		CLK_LOCAL(id, RESET, NULL, NULL, NULL, REG(ns), res, \
				0, 0, 0, 0, NULL, NULL, NULL, NONE, NULL)

#define CLK_SLAVE_RSET(id, reg, br, r_reg, res, par) \
		CLK_LOCAL(id, NORATE, NULL, REG(reg), NULL, REG(r_reg), res, \
				br, 0, 0, 0, NULL, NULL, NULL, par, NULL)

#define CLK_SLAVE_MM(id, reg, br, par) \
		CLK_LOCAL(id, NORATE, NULL, REG_MM(reg), NULL, NULL, 0, \
				br, 0, 0, 0, NULL, NULL, NULL, par, NULL)

#define CLK_SLAVE_LPA(id, reg, br, par) \
		CLK_LOCAL(id, NORATE, NULL, REG_LPA(reg), NULL, NULL, 0, \
				br, 0, 0, 0, NULL, NULL, NULL, par, NULL)

/*
 * Register value macros
 */
#define MN_MODE_DUAL_EDGE 0x2
#define MND_EN(b, n) (b * !!(n))

/* MD Registers */
#define MD4(m_lsb, m, n_lsb, n)	\
		(BVAL((m_lsb+3), m_lsb, m) | BVAL((n_lsb+3), n_lsb, ~(n)))
#define MD8(m_lsb, m, n_lsb, n)	\
		(BVAL((m_lsb+7), m_lsb, m) | BVAL((n_lsb+7), n_lsb, ~(n)))
#define MD16(m, n) (BVAL(31, 16, m) | BVAL(15, 0, ~(n)))

/* NS Registers */
#define NS(n_msb, n_lsb, n, m, mde_lsb, d_msb, d_lsb, d, s_msb, s_lsb, s) \
		(BVAL(n_msb, n_lsb, ~(n-m)) \
		| (BVAL((mde_lsb+1), mde_lsb, MN_MODE_DUAL_EDGE) * !!(n)) \
		| BVAL(d_msb, d_lsb, (d-1)) | BVAL(s_msb, s_lsb, s##_SRC))

#define NS_MM(n_msb, n_lsb, n, m, d_msb, d_lsb, d, s_msb, s_lsb, s) \
		(BVAL(n_msb, n_lsb, ~(n-m)) | BVAL(d_msb, d_lsb, (d-1)) \
		| BVAL(s_msb, s_lsb, s##_SRC))

#define NS_DIVSRC(d_msb , d_lsb, d, s_msb, s_lsb, s) \
		(BVAL(d_msb, d_lsb, (d-1)) | BVAL(s_msb, s_lsb, s##_SRC))

#define NS_DIV(d_msb , d_lsb, d) \
		BVAL(d_msb, d_lsb, (d-1))

#define NS_SRC(s_msb, s_lsb, s) \
		BVAL(s_msb, s_lsb, s##_SRC)

#define NS_MND_BANKED4(n0_lsb, n1_lsb, n, m, s0_lsb, s1_lsb, s) \
		 (BVAL((n0_lsb+3), n0_lsb, ~(n-m)) \
		| BVAL((n1_lsb+3), n1_lsb, ~(n-m)) \
		| BVAL((s0_lsb+2), s0_lsb, s##_SRC) \
		| BVAL((s1_lsb+2), s1_lsb, s##_SRC))

#define NS_MND_BANKED8(n0_lsb, n1_lsb, n, m, s0_lsb, s1_lsb, s) \
		 (BVAL((n0_lsb+7), n0_lsb, ~(n-m)) \
		| BVAL((n1_lsb+7), n1_lsb, ~(n-m)) \
		| BVAL((s0_lsb+2), s0_lsb, s##_SRC) \
		| BVAL((s1_lsb+2), s1_lsb, s##_SRC))

#define NS_DIVSRC_BANKED(d0_msb, d0_lsb, d1_msb, d1_lsb, d, \
	s0_msb, s0_lsb, s1_msb, s1_lsb, s) \
		 (BVAL(d0_msb, d0_lsb, (d-1)) | BVAL(d1_msb, d1_lsb, (d-1)) \
		| BVAL(s0_msb, s0_lsb, s##_SRC) \
		| BVAL(s1_msb, s1_lsb, s##_SRC))

/* CC Registers */
#define CC(mde_lsb, n) (BVAL((mde_lsb+1), mde_lsb, MN_MODE_DUAL_EDGE) * !!(n))
#define CC_BANKED(mde0_lsb, mde1_lsb, n) \
		((BVAL((mde0_lsb+1), mde0_lsb, MN_MODE_DUAL_EDGE) \
		| BVAL((mde1_lsb+1), mde1_lsb, MN_MODE_DUAL_EDGE)) \
		* !!(n))

/*
 * Clock Descriptions
 */

/* BBRX_SSBI */
#define CLK_BBRX_SSBI(id, ns) \
		CLK_LOCAL(id, BASIC, REG(ns), REG(ns), NULL, NULL, 0, B(4), \
				0, 0, 0, set_rate_nop, clk_tbl_bbrx_ssbi, \
				NULL, NONE, NULL)
#define F_BBRX_SSBI(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, 0, 0, 0)
static struct clk_freq_tbl clk_tbl_bbrx_ssbi[] = {
	F_BBRX_SSBI(19200000, NONE, 0, 0, 0),
	F_END,
};

/* GSBI_UART */
#define NS_MASK_GSBI_UART (BM(31, 16) | BM(6, 0))
#define CLK_GSBI_UART(id, ns) \
		CLK_LOCAL(id, MND, REG(ns), REG(ns), REG(ns-4), REG(ns+8), \
				B(0), B(9), B(11), NS_MASK_GSBI_UART, 0, \
				set_rate_mnd, clk_tbl_gsbi_uart, NULL, NONE, \
				NULL)
#define F_GSBI_UART(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD16(m, n), \
			NS(31, 16, n, m, 5, 4, 3, d, 2, 0, s), \
			0, MND_EN(B(8), n))
static struct clk_freq_tbl clk_tbl_gsbi_uart[] = {
	F_GSBI_UART( 3686400, BB_PLL8, 1,  6, 625),
	F_GSBI_UART( 7372800, BB_PLL8, 1, 12, 625),
	F_GSBI_UART(14745600, BB_PLL8, 1, 24, 625),
	F_GSBI_UART(16000000, BB_PLL8, 4,  1,   6),
	F_GSBI_UART(24000000, BB_PLL8, 4,  1,   4),
	F_GSBI_UART(32000000, BB_PLL8, 4,  1,   3),
	F_GSBI_UART(40000000, BB_PLL8, 1,  5,  48),
	F_GSBI_UART(46400000, BB_PLL8, 1, 29, 240),
	F_GSBI_UART(48000000, BB_PLL8, 4,  1,   2),
	F_GSBI_UART(51200000, BB_PLL8, 1,  2,  15),
	F_GSBI_UART(48000000, BB_PLL8, 4,  1,   2),
	F_GSBI_UART(51200000, BB_PLL8, 1,  2,  15),
	F_GSBI_UART(56000000, BB_PLL8, 1,  7,  48),
	F_GSBI_UART(58982400, BB_PLL8, 1, 96, 625),
	F_GSBI_UART(64000000, BB_PLL8, 2,  1,   3),
	F_END,
};

/* GSBI_QUP */
#define NS_MASK_GSBI_QUP (BM(23, 16) | BM(6, 0))
#define CLK_GSBI_QUP(id, ns) \
		CLK_LOCAL(id, MND, REG(ns), REG(ns), REG(ns-4), REG(ns+16), \
				B(0), B(9), B(11), NS_MASK_GSBI_QUP, 0, \
				set_rate_mnd, clk_tbl_gsbi_qup, NULL, NONE, \
				NULL)
#define F_GSBI_QUP(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(16, m, 0, n), \
			NS(23, 16, n, m, 5, 4, 3, d, 2, 0, s), \
			0, MND_EN(B(8), n))
static struct clk_freq_tbl clk_tbl_gsbi_qup[] = {
	F_GSBI_QUP( 1000000, BB_PXO,  1, 2, 49),
	F_GSBI_QUP( 4920000, BB_PXO,  1, 1,  5),
	F_GSBI_QUP( 9830000, BB_PXO,  1, 2,  5),
	F_GSBI_QUP(15060000, BB_PLL8, 1, 2, 51),
	F_GSBI_QUP(24000000, BB_PLL8, 4, 1,  4),
	F_GSBI_QUP(25600000, BB_PLL8, 1, 1, 15),
	F_GSBI_QUP(48000000, BB_PLL8, 4, 1,  2),
	F_GSBI_QUP(51200000, BB_PLL8, 1, 2, 15),
	F_END,
};

/* GSBI_SIM */
#define NS_MASK_GSBI_SIM (BM(6, 3) | BM(1, 0))
#define CLK_GSBI_SIM(id, ns) \
		CLK_LOCAL(id, BASIC, REG(ns), REG(ns), NULL, NULL, 0, 0, \
				B(11), NS_MASK_GSBI_SIM, 0, set_rate_basic, \
				clk_tbl_gsbi_sim, NULL, NONE, chld_gsbi_sim_src)
#define F_GSBI_SIM(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, NS_DIVSRC(6, 3, d, 1, 0, s), 0, 0)
static struct clk_freq_tbl clk_tbl_gsbi_sim[] = {
	F_GSBI_SIM(3510000, XO_PXO, 7, 0, 0),
	F_END,
};

/* PDM */
#define NS_MASK_PDM (BM(1, 0))
#define CLK_PDM(id, ns) \
		CLK_LOCAL(id, BASIC, REG(ns), REG(ns), NULL, REG(ns), B(12), \
				B(9), B(11)|B(15), NS_MASK_PDM, 0, \
				set_rate_basic, clk_tbl_pdm, NULL, NONE, NULL)
#define F_PDM(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, NS_SRC(1, 0, s), 0, 0)
static struct clk_freq_tbl clk_tbl_pdm[] = {
	F_PDM(24576000, XO_PXO, 1, 0, 0),
	F_END,
};

/* PRNG */
#define NS_MASK_PRNG (BM(6, 3) | BM(2, 0))
#define CLK_PRNG(id, ns) \
		CLK_LOCAL(id, BASIC, REG(ns), REG(ns), NULL, REG(ns), B(12), \
				0, B(11), NS_MASK_PRNG, 0, set_rate_basic, \
				clk_tbl_prng, NULL, NONE, NULL)
#define F_PRNG(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, NS_DIVSRC(6, 3, d, 2, 0, s), 0, 0)
static struct clk_freq_tbl clk_tbl_prng[] = {
	F_PRNG(32000000, BB_PLL8, 12, 0, 0),
	F_PRNG(64000000, BB_PLL8,  6, 0, 0),
	F_END,
};

/* PMIC_SSBI2 */
#define NS_MASK_PMIC_SSBI2 (BM(4, 3) | BM(1, 0))
#define CLK_PMIC_SSBI2(id, ns) \
		CLK_LOCAL(id, BASIC, REG(ns), REG(ns), NULL, REG(ns), B(12), \
				0, B(11), NS_MASK_PMIC_SSBI2, 0, \
				set_rate_basic, clk_tbl_pmic_ssbi2, NULL, \
				NONE, NULL)
#define F_PMIC_SSBI2(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, NS_DIVSRC(4, 3, d, 1, 0, s), 0, 0)
static struct clk_freq_tbl clk_tbl_pmic_ssbi2[] = {
	F_PMIC_SSBI2(12288000, XO_PXO, 2, 0, 0),
	F_END,
};

/* SDC */
#define NS_MASK_SDC (BM(23, 16) | BM(6, 0))
#define CLK_SDC(id, ns) \
		CLK_LOCAL(id, MND, REG(ns), REG(ns), REG(ns-4), REG(ns+4), \
				B(0), B(9), B(11), NS_MASK_SDC, 0, \
				set_rate_mnd, clk_tbl_sdc, NULL, NONE, NULL)
#define F_SDC(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(16, m, 0, n), \
			NS(23, 16, n, m, 5, 4, 3, d, 2, 0, s), \
			0, MND_EN(B(8), n))
static struct clk_freq_tbl clk_tbl_sdc[] = {
	F_SDC(  400000, BB_PLL8,  4, 1, 240),
	F_SDC(16000000, BB_PLL8,  4, 1,   6),
	F_SDC(17070000, BB_PLL8,  1, 2,  45),
	F_SDC(20210000, BB_PLL8,  1, 1,  19),
	F_SDC(24000000, BB_PLL8,  4, 1,   4),
	F_SDC(48000000, BB_PLL8,  4, 1,   2),
	F_END,
};

/* TSIF_REF */
#define NS_MASK_TSIF_REF (BM(31, 16) | BM(6, 0))
#define CLK_TSIF_REF(id, ns) \
		CLK_LOCAL(id, MND, REG(ns), REG(ns), REG(ns-4), NULL, 0, B(9), \
				B(11), NS_MASK_TSIF_REF, 0, set_rate_mnd, \
				clk_tbl_tsif_ref, NULL, NONE, NULL)
#define F_TSIF_REF(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD16(m, n), \
			NS(31, 16, n, m, 5, 4, 3, d, 2, 0, s), \
			0, MND_EN(B(8), n))
static struct clk_freq_tbl clk_tbl_tsif_ref[] = {
	F_TSIF_REF(105000, BB_PXO, 1, 1, 233),
	F_END,
};


/* TSSC */
#define NS_MASK_TSSC (BM(1, 0))
#define CLK_TSSC(id, ns) \
		CLK_LOCAL(id, BASIC, REG(ns), REG(ns), NULL, NULL, 0, B(4), \
				B(11), NS_MASK_TSSC, 0, set_rate_basic, \
				clk_tbl_tssc, NULL, NONE, NULL)
#define F_TSSC(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, NS_DIV(1, 0, d), 0, 0)
static struct clk_freq_tbl clk_tbl_tssc[] = {
	F_TSSC(24576000, NONE, 0, 0, 0),
	F_END,
};

/* USB_HS and USB_FS */
#define NS_MASK_USB (BM(23, 16) | BM(6, 0))
#define CLK_USB_HS(id, ns) \
		CLK_LOCAL(id, MND, REG(ns), REG(ns), REG(ns-4), REG(ns+4), \
				B(0), B(9), B(11), NS_MASK_USB, 0, \
				set_rate_mnd, clk_tbl_usb, NULL, NONE, NULL)
#define CLK_USB_FS(id, ns, chld_lst) \
		CLK_LOCAL(id, MND, REG(ns), REG(ns), REG(ns-4), NULL, 0, \
				0, B(11), NS_MASK_USB, 0, set_rate_mnd, \
				clk_tbl_usb, NULL, NONE, chld_lst)
#define F_USB(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(16, m, 0, n), \
			NS(23, 16, n, m, 5, 4, 3, d, 2, 0, s), \
			0, MND_EN(B(8), n))
static struct clk_freq_tbl clk_tbl_usb[] = {
	F_USB(60000000, BB_PLL8, 1, 5, 32),
	F_END,
};

/* CAM */
#define NS_MASK_CAM (BM(31, 24) | BM(15, 14) | BM(2, 0))
#define CC_MASK_CAM (BM(7, 6))
#define CLK_CAM(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-8), REG_MM(ns-4), \
				NULL, 0, B(0), B(2), NS_MASK_CAM, CC_MASK_CAM, \
				set_rate_cam, clk_tbl_cam, NULL, NONE, NULL)
#define F_CAM(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(8, m, 0, n), \
			NS_MM(31, 24, n, m, 15, 14, d, 2, 0, s), \
			CC(6, n), MND_EN(B(5), n))
static struct clk_freq_tbl clk_tbl_cam[] = {
	F_CAM(  6000000, MM_GPERF, 4, 1, 24),
	F_CAM(  8000000, MM_GPERF, 4, 1, 12),
	F_CAM( 12000000, MM_GPERF, 4, 1,  8),
	F_CAM( 16000000, MM_GPERF, 4, 1,  6),
	F_CAM( 19200000, MM_GPERF, 4, 1,  5),
	F_CAM( 24000000, MM_GPERF, 4, 1,  4),
	F_CAM( 32000000, MM_GPERF, 4, 1,  3),
	F_CAM( 48000000, MM_GPERF, 4, 1,  2),
	F_CAM( 64000000, MM_GPERF, 3, 1,  2),
	F_CAM( 96000000, MM_GPERF, 4, 0,  0),
	F_CAM(128000000, MM_GPERF, 3, 0,  0),
	F_END,
};

/* CSI */
#define NS_MASK_CSI (BM(15, 12) | BM(2, 0))
#define CLK_CSI(id, ns) \
		CLK_LOCAL(id, BASIC, REG_MM(ns), REG_MM(ns-8), NULL, NULL, 0, \
				0, B(2), NS_MASK_CSI, 0, set_rate_basic, \
				clk_tbl_csi, NULL, NONE, chld_csi_src)
#define F_CSI(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, NS_DIVSRC(15, 12, d, 2, 0, s), 0, 0)
static struct clk_freq_tbl clk_tbl_csi[] = {
	F_CSI(192000000, MM_GPERF, 2, 0, 0),
	F_CSI(384000000, MM_GPERF, 1, 0, 0),
	F_END,
};

/* GFX2D0 and GFX2D1 */
struct banked_mnd_masks bmdn_info_gfx2d0 = {
	.bank_sel_mask =			B(11),
	.bank0_mask = {
			.md_reg = 		REG_MM(0x0064),
			.ns_mask =	 	BM(23, 20) | BM(5, 3),
			.rst_mask =		B(25),
			.mnd_en_mask =		B(8),
			.mode_mask =		BM(10, 9),
	},
	.bank1_mask = {
			.md_reg = 		REG_MM(0x0068),
			.ns_mask =		BM(19, 16) | BM(2, 0),
			.rst_mask =		B(24),
			.mnd_en_mask =		B(5),
			.mode_mask =		BM(7, 6),
	},
};
#define CLK_GFX2D0(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-16), NULL, NULL, 0, \
				B(0), B(2), 0, 0, set_rate_mnd_banked, \
				clk_tbl_gfx2d, &bmdn_info_gfx2d0, NONE, NULL)
struct banked_mnd_masks bmdn_info_gfx2d1 = {
	.bank_sel_mask =		B(11),
	.bank0_mask = {
			.md_reg = 		REG_MM(0x0078),
			.ns_mask =	 	BM(23, 20) | BM(5, 3),
			.rst_mask =		B(25),
			.mnd_en_mask =		B(8),
			.mode_mask =		BM(10, 9),
	},
	.bank1_mask = {
			.md_reg = 		REG_MM(0x006C),
			.ns_mask =		BM(19, 16) | BM(2, 0),
			.rst_mask =		B(24),
			.mnd_en_mask =		B(5),
			.mode_mask =		BM(7, 6),
	},
};
#define CLK_GFX2D1(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-8), NULL, NULL, 0, \
				B(0), B(2), 0, 0, set_rate_mnd_banked, \
				clk_tbl_gfx2d, &bmdn_info_gfx2d1, NONE, NULL)
#define F_GFX2D(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD4(4, m, 0, n), \
			NS_MND_BANKED4(20, 16, n, m, 3, 0, s), \
			CC_BANKED(9, 6, n), MND_EN((B(8) | B(5)), n))
static struct clk_freq_tbl clk_tbl_gfx2d[] = {
	F_GFX2D( 24576000, MM_PXO,   0, 0,  0),
	F_GFX2D( 27000000, MM_MXO,   0, 0,  0),
	F_GFX2D( 48000000, MM_GPERF, 0, 1,  8),
	F_GFX2D( 54857000, MM_GPERF, 0, 1,  7),
	F_GFX2D( 64000000, MM_GPERF, 0, 1,  6),
	F_GFX2D( 76800000, MM_GPERF, 0, 1,  5),
	F_GFX2D( 96000000, MM_GPERF, 0, 1,  4),
	F_GFX2D(128000000, MM_GPERF, 0, 1,  3),
	F_GFX2D(145455000, MM_PLL1,  0, 2, 11),
	F_GFX2D(160000000, MM_PLL1,  0, 1,  5),
	F_GFX2D(177778000, MM_PLL1,  0, 2,  9),
	F_GFX2D(200000000, MM_PLL1,  0, 1,  4),
	F_GFX2D(228571000, MM_PLL1,  0, 2,  7),
	F_GFX2D(250000000, MM_GPLL0, 0, 1,  4),
	F_END,
};

/* GFX3D */
struct banked_mnd_masks bmdn_info_gfx3d = {
	.bank_sel_mask = 		B(11),
	.bank0_mask = {
			.md_reg = 		REG_MM(0x0084),
			.ns_mask =	 	BM(21, 18) | BM(5, 3),
			.rst_mask =		B(23),
			.mnd_en_mask =		B(8),
			.mode_mask =		BM(10, 9),
	},
	.bank1_mask = {
			.md_reg = 		REG_MM(0x0088),
			.ns_mask =		BM(17, 14) | BM(2, 0),
			.rst_mask =		B(22),
			.mnd_en_mask =		B(5),
			.mode_mask =		BM(7, 6),
	},
};
#define CLK_GFX3D(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-12), NULL, NULL, 0, \
				B(0), B(2), 0, 0, set_rate_mnd_banked, \
				clk_tbl_gfx3d, &bmdn_info_gfx3d, NONE, NULL)
#define F_GFX3D(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD4(4, m, 0, n), \
			NS_MND_BANKED4(18, 14, n, m, 3, 0, s), \
			CC_BANKED(9, 6, n), MND_EN((B(8) | B(5)), n))
static struct clk_freq_tbl clk_tbl_gfx3d[] = {
	F_GFX3D( 27000000, MM_MXO,   0, 0,  0),
	F_GFX3D( 48000000, MM_GPERF, 0, 1,  8),
	F_GFX3D( 54857000, MM_GPERF, 0, 1,  7),
	F_GFX3D( 64000000, MM_GPERF, 0, 1,  6),
	F_GFX3D( 76800000, MM_GPERF, 0, 1,  5),
	F_GFX3D( 96000000, MM_GPERF, 0, 1,  4),
	F_GFX3D(128000000, MM_GPERF, 0, 1,  3),
	F_GFX3D(145455000, MM_PLL1,  0, 2, 11),
	F_GFX3D(160000000, MM_PLL1,  0, 1,  5),
	F_GFX3D(177778000, MM_PLL1,  0, 2,  9),
	F_GFX3D(200000000, MM_PLL1,  0, 1,  4),
	F_GFX3D(228571000, MM_PLL1,  0, 2,  7),
	F_GFX3D(250000000, MM_GPLL0, 0, 1,  4),
	F_END,
};

/* IJPEG */
#define NS_MASK_IJPEG (BM(23, 16) | BM(15, 12) | BM(2, 0))
#define CC_MASK_IJPEG (BM(7, 6))
#define CLK_IJPEG(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-8), REG_MM(ns-4), \
				NULL, 0, B(0), B(2), NS_MASK_IJPEG, \
				CC_MASK_IJPEG, set_rate_mnd, clk_tbl_ijpeg, \
				NULL, NONE, NULL)
#define F_IJPEG(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(8, m, 0, n), \
			NS_MM(23, 16, n, m, 15, 12, d, 2, 0, s), \
			CC(6, n), MND_EN(B(5), n))
static struct clk_freq_tbl clk_tbl_ijpeg[] = {
	F_IJPEG( 36570000, MM_GPERF, 1, 2, 21),
	F_IJPEG( 54860000, MM_GPERF, 7, 0,  0),
	F_IJPEG( 96000000, MM_GPERF, 4, 0,  0),
	F_IJPEG(109710000, MM_GPERF, 1, 2,  7),
	F_IJPEG(128000000, MM_GPERF, 3, 0,  0),
	F_IJPEG(153600000, MM_GPERF, 1, 2,  5),
	F_IJPEG(200000000, MM_PLL1,  4, 0,  0),
	F_IJPEG(228000000, MM_PLL1,  1, 2,  7),
	F_END,
};

/* JPEGD */
#define NS_MASK_JPEGD (BM(15, 12) | BM(2, 0))
#define CLK_JPEGD(id, ns) \
		CLK_LOCAL(id, BASIC, REG_MM(ns), REG_MM(ns-8), NULL, NULL, 0, \
				B(0), B(2), NS_MASK_JPEGD, 0, set_rate_basic, \
				clk_tbl_jpegd, 	NULL, NONE, NULL)
#define F_JPEGD(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, NS_DIVSRC(15, 12, d, 2, 0, s), 0, 0)
static struct clk_freq_tbl clk_tbl_jpegd[] = {
	F_JPEGD( 64000000, MM_GPERF, 6, 0, 0),
	F_JPEGD( 76800000, MM_GPERF, 5, 0, 0),
	F_JPEGD( 96000000, MM_GPERF, 4, 0, 0),
	F_JPEGD(160000000, MM_PLL1,  5, 0, 0),
	F_JPEGD(200000000, MM_PLL1,  4, 0, 0),
	F_END,
};

/* MDP */
struct banked_mnd_masks bmdn_info_mdp = {
	.bank_sel_mask =		B(11),
	.bank0_mask = {
			.md_reg = 		REG_MM(0x00C4),
			.ns_mask =	 	BM(29, 22) | BM(5, 3),
			.rst_mask =		B(31),
			.mnd_en_mask =		B(8),
			.mode_mask =		BM(10, 9),
	},
	.bank1_mask = {
			.md_reg = 		REG_MM(0x00C8),
			.ns_mask =		BM(21, 14) | BM(2, 0),
			.rst_mask =		B(30),
			.mnd_en_mask =		B(5),
			.mode_mask =		BM(7, 6),
	},
};
#define CLK_MDP(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-16), NULL, NULL, 0, \
				B(0), B(2), 0, 0, set_rate_mnd_banked, \
				clk_tbl_mdp, &bmdn_info_mdp, NONE, NULL)
#define F_MDP(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(8, m, 0, n), \
			NS_MND_BANKED8(22, 14, n, m, 3, 0, s), \
			CC_BANKED(9, 6, n), MND_EN((B(8) | B(5)), n))
static struct clk_freq_tbl clk_tbl_mdp[] = {
	F_MDP(  9600000, MM_GPERF, 0, 1, 40),
	F_MDP( 13710000, MM_GPERF, 0, 1, 28),
	F_MDP( 29540000, MM_GPERF, 0, 1, 13),
	F_MDP( 34910000, MM_GPERF, 0, 1, 11),
	F_MDP( 38400000, MM_GPERF, 0, 1, 10),
	F_MDP( 59080000, MM_GPERF, 0, 2, 13),
	F_MDP( 76800000, MM_GPERF, 0, 1,  5),
	F_MDP( 85330000, MM_GPERF, 0, 2,  9),
	F_MDP( 96000000, MM_GPERF, 0, 1,  4),
	F_MDP(128000000, MM_GPERF, 0, 1,  3),
	F_MDP(160000000, MM_PLL1,  0, 1,  5),
	F_MDP(200000000, MM_PLL1,  0, 1,  4),
	F_END,
};

/* MDP VSYNC */
#define CLK_MDP_VSYNC(id, ns) \
		CLK_LOCAL(id, BASIC, REG_MM(ns), REG_MM(ns), NULL, NULL, 0, \
				B(6), 0, 0, 0, set_rate_nop, \
				clk_tbl_mdp_vsync, NULL, NONE, NULL)
#define F_MDP_VSYNC(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, 0, 0, 0)
static struct clk_freq_tbl clk_tbl_mdp_vsync[] = {
	F_MDP_VSYNC(24576000, NONE, 0, 0, 0),
	F_END,
};

/* PIXEL_MDP */
#define NS_MASK_PIXEL_MDP (BM(31, 16) | BM(15, 14) | BM(2, 0))
#define CC_MASK_PIXEL_MDP (BM(7, 6))
#define CLK_PIXEL_MDP(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-8), REG_MM(ns-4), \
				NULL, 0, B(0), B(2), NS_MASK_PIXEL_MDP, \
				CC_MASK_PIXEL_MDP, set_rate_mnd, \
				clk_tbl_pixel_mdp, NULL, NONE, chld_pixel_mdp)
#define F_PIXEL_MDP(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD16(m, n), \
			NS_MM(31, 16, n, m, 15, 14, d, 2, 0, s), \
			CC(6, n), MND_EN(B(5), n))
static struct clk_freq_tbl clk_tbl_pixel_mdp[] = {
	F_PIXEL_MDP(43192000, MM_GPERF, 1,  64, 569),
	F_PIXEL_MDP(48000000, MM_GPERF, 4,   1,   2),
	F_PIXEL_MDP(53990000, MM_GPERF, 2, 169, 601),
	F_END,
};

/* ROT */
#define CLK_ROT(id, ns) \
		CLK_LOCAL(id, BASIC, REG_MM(ns), REG_MM(ns-8), NULL, NULL, 0, \
				B(0), B(2), 0, 0, set_rate_div_banked, \
				clk_tbl_rot, NULL, NONE, NULL)
#define F_ROT(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, \
		NS_DIVSRC_BANKED(29, 26, 25, 22, d, 21, 19, 18, 16, s), 0, 0)
static struct clk_freq_tbl clk_tbl_rot[] = {
	F_ROT( 24580000, MM_PXO,    1, 0, 0),
	F_ROT( 27000000, MM_MXO,    1, 0, 0),
	F_ROT( 29540000, MM_GPERF, 13, 0, 0),
	F_ROT( 32000000, MM_GPERF, 12, 0, 0),
	F_ROT( 38400000, MM_GPERF, 10, 0, 0),
	F_ROT( 48000000, MM_GPERF,  8, 0, 0),
	F_ROT( 54860000, MM_GPERF,  7, 0, 0),
	F_ROT( 64000000, MM_GPERF,  6, 0, 0),
	F_ROT( 76800000, MM_GPERF,  5, 0, 0),
	F_ROT( 96000000, MM_GPERF,  4, 0, 0),
	F_ROT(100000000, MM_PLL1,   8, 0, 0),
	F_ROT(114290000, MM_PLL1,   7, 0, 0),
	F_ROT(133330000, MM_PLL1,   6, 0, 0),
	F_ROT(160000000, MM_PLL1,   5, 0, 0),
};

/* TV */
#define NS_MASK_TV (BM(23, 16) | BM(15, 14) | BM(2, 0))
#define CC_MASK_TV (BM(7, 6))
#define CLK_TV(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-8), REG_MM(ns-4), \
				NULL, 0, 0, B(2), NS_MASK_TV, CC_MASK_TV, \
				set_rate_tv, clk_tbl_tv, NULL, NONE, \
				chld_tv_src)
#define F_TV(f, s, p_r, d, m, n) \
		F_RAW_PLL(f, s##_PLL, MD8(8, m, 0, n), \
			NS_MM(23, 16, n, m, 15, 14, d, 2, 0, s), \
			CC(6, n), MND_EN(B(5), n), p_r)
/* Switching TV freqs requires PLL reconfiguration. */
static struct pll_rate mm_pll2_rate[] = {
	[0] = PLL_RATE( 50400500,  7, 6301, 13500, 0, 4),
	[1] = PLL_RATE( 54000000,  8,    0,     1, 0, 4),
	[2] = PLL_RATE( 54054000,  8,    1,   125, 0, 4),
	[3] = PLL_RATE(148500000, 22,    0,     1, 2, 4),
	[4] = PLL_RATE(297000000, 44,    0,     1, 2, 4),
};
static struct clk_freq_tbl clk_tbl_tv[] = {
	F_TV( 25200000, MM_PLL2, &mm_pll2_rate[0], 2, 0, 0),
	F_TV( 27000000, MM_PLL2, &mm_pll2_rate[1], 2, 0, 0),
	F_TV( 27030000, MM_PLL2, &mm_pll2_rate[2], 2, 0, 0),
	F_TV( 74250000, MM_PLL2, &mm_pll2_rate[3], 2, 0, 0),
	F_TV(148500000, MM_PLL2, &mm_pll2_rate[4], 2, 0, 0),
	F_END,
};

/* VCODEC */
struct banked_mnd_masks bmdn_info_vcodec = {
	.bank_sel_mask =		B(13),
	.bank0_mask = {
			.md_reg = 		REG_MM(0x00FC),
			.ns_mask =	 	BM(18, 11) | BM(2, 0),
			.rst_mask =		B(31),
			.mnd_en_mask =		B(5),
			.mode_mask =		BM(7, 6),
	},
	.bank1_mask = {
			.md_reg = 		REG_MM(0x0128),
			.ns_mask =		BM(29, 27) | BM(26, 19),
			.rst_mask =		B(30),
			.mnd_en_mask =		B(10),
			.mode_mask =		BM(12, 11),
	},
};
#define CLK_VCODEC(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-8), REG_MM(ns-4), \
				NULL, 0, B(0), B(2), 0, 0, \
				set_rate_mnd_banked, clk_tbl_vcodec, \
				&bmdn_info_vcodec, NONE, NULL)
#define F_VCODEC(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(8, m, 0, n), \
			NS_MND_BANKED8(11, 19, n, m, 0, 27, s), \
			CC_BANKED(6, 11, n), MND_EN((B(5) | B(10)), n))
static struct clk_freq_tbl clk_tbl_vcodec[] = {
	F_VCODEC( 24580000, MM_PXO,   0, 0,  0),
	F_VCODEC( 27000000, MM_MXO,   0, 0,  0),
	F_VCODEC( 32000000, MM_GPERF, 0, 1, 12),
	F_VCODEC( 48000000, MM_GPERF, 0, 1,  8),
	F_VCODEC( 54860000, MM_GPERF, 0, 1,  7),
	F_VCODEC( 96000000, MM_GPERF, 0, 1,  4),
	F_VCODEC(133330000, MM_PLL1,  0, 1,  6),
	F_VCODEC(200000000, MM_PLL1,  0, 1,  4),
	F_VCODEC(228570000, MM_PLL1,  0, 2,  7),
	F_VCODEC(250000000, MM_GPLL0, 0, 1,  2),
	F_END,
};

/* VPE */
#define NS_MASK_VPE (BM(15, 12) | BM(2, 0))
#define CLK_VPE(id, ns) \
		CLK_LOCAL(id, BASIC, REG_MM(ns), REG_MM(ns-8), NULL, NULL, 0, \
				B(0), B(2), NS_MASK_VPE, 0, set_rate_basic, \
				clk_tbl_vpe, NULL, NONE, NULL)
#define F_VPE(f, s, d, m, n) \
		F_RAW(f, s##_PLL, 0, NS_DIVSRC(15, 12, d, 2, 0, s), 0, 0)
static struct clk_freq_tbl clk_tbl_vpe[] = {
	F_VPE( 24576000, MM_PXO,    1, 0, 0),
	F_VPE( 27000000, MM_MXO,    1, 0, 0),
	F_VPE( 34909000, MM_GPERF, 11, 0, 0),
	F_VPE( 38400000, MM_GPERF, 10, 0, 0),
	F_VPE( 64000000, MM_GPERF,  6, 0, 0),
	F_VPE( 76800000, MM_GPERF,  5, 0, 0),
	F_VPE( 96000000, MM_GPERF,  4, 0, 0),
	F_VPE(100000000, MM_PLL1,   8, 0, 0),
	F_VPE(160000000, MM_PLL1,   5, 0, 0),
	F_END,
};

/* VFE */
#define NS_MASK_VFE (BM(23, 16) | BM(11, 10) | BM(2, 0))
#define CC_MASK_VFE (BM(7, 6))
#define CLK_VFE(id, ns) \
		CLK_LOCAL(id, MND, REG_MM(ns), REG_MM(ns-8), REG_MM(ns-4), \
				NULL, 0, 0, B(2), NS_MASK_VFE, CC_MASK_VFE, \
				set_rate_mnd, clk_tbl_vfe, NULL, NONE, \
				chld_vfe_src)
#define F_VFE(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(8, m, 0, n), \
			NS_MM(23, 16, n, m, 11, 10, d, 2, 0, s), \
			CC(6, n), MND_EN(B(5), n))
static struct clk_freq_tbl clk_tbl_vfe[] = {
	F_VFE( 13960000, MM_GPERF,  1, 2, 55),
	F_VFE( 36570000, MM_GPERF,  1, 2, 21),
	F_VFE( 38400000, MM_GPERF,  2, 1,  5),
	F_VFE( 45180000, MM_GPERF,  1, 2, 17),
	F_VFE( 48000000, MM_GPERF,  2, 1,  4),
	F_VFE( 54860000, MM_GPERF,  1, 1,  7),
	F_VFE( 64000000, MM_GPERF,  2, 1,  3),
	F_VFE( 76800000, MM_GPERF,  1, 1,  5),
	F_VFE( 96000000, MM_GPERF,  2, 1,  2),
	F_VFE(109710000, MM_GPERF,  1, 2,  7),
	F_VFE(128000000, MM_GPERF,  1, 1,  3),
	F_VFE(153600000, MM_GPERF,  2, 0,  0),
	F_VFE(200000000, MM_PLL1,   2, 1,  2),
	F_VFE(228570000, MM_PLL1,   1, 2,  7),
	F_END,
};

/* Audio Interface */
#define NS_MASK_AIF (BM(31, 24) | BM(6, 0))
#define CLK_AIF(id, ns, chld_lst) \
		CLK_LOCAL(id, MND, REG_LPA(ns), REG_LPA(ns), REG_LPA(ns+4), \
				REG_LPA(ns), B(19), (B(15) | B(17)), B(9), \
				NS_MASK_AIF, 0, set_rate_mnd, clk_tbl_aif, \
				NULL, NONE, chld_lst)
#define F_AIF(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD8(8, m, 0, n), \
			NS(31, 24, n, m, 5, 4, 3, d, 2, 0, s), \
			0, MND_EN(B(8), n))
static struct clk_freq_tbl clk_tbl_aif[] = {
	F_AIF(  512000, LPA_PXO, 1, 1, 48),
	F_AIF(  768000, LPA_PXO, 1, 1, 32),
	F_AIF( 1024000, LPA_PXO, 1, 1, 24),
	F_AIF( 1536000, LPA_PXO, 1, 1, 16),
	F_AIF( 2048000, LPA_PXO, 1, 1, 12),
	F_AIF( 3072000, LPA_PXO, 1, 1,  8),
	F_AIF( 4096000, LPA_PXO, 1, 1,  6),
	F_AIF( 6144000, LPA_PXO, 1, 1,  4),
	F_AIF( 8192000, LPA_PXO, 1, 1,  3),
	F_AIF(12288000, LPA_PXO, 1, 1,  2),
	F_AIF(24580000, LPA_PXO, 1, 0,  0),
	F_END,
};

/* PCM */
#define NS_MASK_PCM (BM(31, 16) | BM(6, 0))
#define CLK_PCM(id, ns) \
		CLK_LOCAL(id, MND, REG_LPA(ns), REG_LPA(ns), REG_LPA(ns+4), \
				REG_LPA(ns), B(13), B(11), B(9), NS_MASK_PCM, \
				0, set_rate_mnd, clk_tbl_pcm, NULL, NONE, NULL)
#define F_PCM(f, s, d, m, n) \
		F_RAW(f, s##_PLL, MD16(m, n), \
			NS(31, 16, n, m, 5, 4, 3, d, 2, 0, s), \
			0, MND_EN(B(8), n))
static struct clk_freq_tbl clk_tbl_pcm[] = {
	F_PCM(  512000, LPA_PXO, 1, 1, 48),
	F_PCM(  768000, LPA_PXO, 1, 1, 32),
	F_PCM( 1024000, LPA_PXO, 1, 1, 24),
	F_PCM( 1536000, LPA_PXO, 1, 1, 16),
	F_PCM( 2048000, LPA_PXO, 1, 1, 12),
	F_PCM( 3072000, LPA_PXO, 1, 1,  8),
	F_PCM( 4096000, LPA_PXO, 1, 1,  6),
	F_PCM( 6144000, LPA_PXO, 1, 1,  4),
	F_PCM( 8192000, LPA_PXO, 1, 1,  3),
	F_PCM(12288000, LPA_PXO, 1, 1,  2),
	F_PCM(24580000, LPA_PXO, 1, 0,  0),
	F_END,
};

static struct clk_freq_tbl dummy_freq = F_END;

/* Register offsets used more than once. */
#define PLL_ENA_REG		0x34C0

static uint32_t pll_count[NUM_PLL];

static uint32_t chld_gsbi_sim_src[] = 	{C(GSBI1_SIM), C(GSBI2_SIM),
					 C(GSBI3_SIM), C(GSBI4_SIM),
					 C(GSBI4_SIM), C(GSBI5_SIM),
					 C(GSBI5_SIM), C(GSBI6_SIM),
					 C(GSBI7_SIM), C(GSBI8_SIM),
					 C(GSBI9_SIM), C(GSBI10_SIM),
					 C(GSBI11_SIM), C(GSBI12_SIM),
					 C(NONE)};
static uint32_t chld_usb_fs1_src[] = 	{C(USB_FS1_XCVR), C(USB_FS1_SYS),
					 C(NONE)};
static uint32_t chld_usb_fs2_src[] = 	{C(USB_FS2_XCVR), C(USB_FS2_SYS),
					 C(NONE)};
static uint32_t chld_csi_src[] = 	{C(CSI0), C(CSI1), C(NONE)};
static uint32_t chld_pixel_mdp[] = 	{C(PIXEL_LCDC), C(NONE)};
static uint32_t chld_tv_src[] =		{C(TV_ENC), C(TV_DAC), C(MDP_TV),
					 C(HDMI_TV), C(DSUB_TV), C(NONE)};
static uint32_t chld_vfe_src[] =	{C(VFE),  C(CSI0_VFE), C(CSI1_VFE),
					 C(NONE)};
static uint32_t chld_mi2s_src[] =	{C(MI2S), C(MI2S_M), C(NONE)};
static uint32_t chld_codec_i2s_mic_src[] =	{C(CODEC_I2S_MIC),
						 C(CODEC_I2S_MIC_M), C(NONE)};
static uint32_t chld_codec_i2s_spkr_src[] =	{C(CODEC_I2S_SPKR),
						 C(CODEC_I2S_SPKR_M), C(NONE)};
static uint32_t chld_spare_i2s_mic_src[] =	{C(SPARE_I2S_MIC),
						 C(SPARE_I2S_MIC_M), C(NONE)};
static uint32_t chld_spare_i2s_spkr_src[] =	{C(SPARE_I2S_SPKR),
						 C(SPARE_I2S_SPKR_M), C(NONE)};

static struct clk_local clk_local_tbl[] = {

	/*
	 * Peripheral Clocks
	 */

	CLK_BBRX_SSBI(BBRX_SSBI, 0x2CE0),

	CLK_GSBI_UART(GSBI1_UART,  0x29D4),
	CLK_GSBI_UART(GSBI2_UART,  0x29F4),
	CLK_GSBI_UART(GSBI3_UART,  0x2A14),
	CLK_GSBI_UART(GSBI4_UART,  0x2A34),
	CLK_GSBI_UART(GSBI5_UART,  0x2A54),
	CLK_GSBI_UART(GSBI6_UART,  0x2A74),
	CLK_GSBI_UART(GSBI7_UART,  0x2A94),
	CLK_GSBI_UART(GSBI8_UART,  0x2AB4),
	CLK_GSBI_UART(GSBI9_UART,  0x2AD4),
	CLK_GSBI_UART(GSBI10_UART, 0x2AF4),
	CLK_GSBI_UART(GSBI11_UART, 0x2B14),
	CLK_GSBI_UART(GSBI12_UART, 0x2B34),

	CLK_GSBI_QUP(GSBI1_QUP,  0x29CC),
	CLK_GSBI_QUP(GSBI2_QUP,  0x29EC),
	CLK_GSBI_QUP(GSBI3_QUP,  0x2A0C),
	CLK_GSBI_QUP(GSBI4_QUP,  0x2A2C),
	CLK_GSBI_QUP(GSBI5_QUP,  0x2A4C),
	CLK_GSBI_QUP(GSBI6_QUP,  0x2A6C),
	CLK_GSBI_QUP(GSBI7_QUP,  0x2A8C),
	CLK_GSBI_QUP(GSBI8_QUP,  0x2AAC),
	CLK_GSBI_QUP(GSBI9_QUP,  0x2ACC),
	CLK_GSBI_QUP(GSBI10_QUP, 0x2AEC),
	CLK_GSBI_QUP(GSBI11_QUP, 0x2B0C),
	CLK_GSBI_QUP(GSBI12_QUP, 0x2B2C),

	CLK_GSBI_SIM(GSBI_SIM_SRC, 0x29A0),
	CLK_SLAVE_RSET(GSBI1_SIM,  0x29D8, B(4), 0x29DC, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI2_SIM,  0x29F8, B(4), 0x29FC, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI3_SIM,  0x2A18, B(4), 0x2A1C, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI4_SIM,  0x2A38, B(4), 0x2A3C, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI5_SIM,  0x2A58, B(4), 0x2A5C, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI6_SIM,  0x2A78, B(4), 0x2A7C, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI7_SIM,  0x2A98, B(4), 0x2A9C, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI8_SIM,  0x2AB8, B(4), 0x2ABC, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI9_SIM,  0x2AD8, B(4), 0x2ADC, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI10_SIM, 0x2AF8, B(4), 0x2AFC, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI11_SIM, 0x2B18, B(4), 0x2B1C, B(0), GSBI_SIM_SRC),
	CLK_SLAVE_RSET(GSBI12_SIM, 0x2B38, B(4), 0x2B3C, B(0), GSBI_SIM_SRC),

	CLK_PDM(PDM, 0x2CC0),

	CLK_PRNG(PRNG, 0x2E80),

	CLK_PMIC_SSBI2(PMIC_SSBI2, 0x280C),

	CLK_SDC(SDC1, 0x282C),
	CLK_SDC(SDC2, 0x284C),
	CLK_SDC(SDC3, 0x286C),
	CLK_SDC(SDC4, 0x288C),
	CLK_SDC(SDC5, 0x28AC),

	CLK_TSIF_REF(TSIF_REF, 0x2710),

	CLK_TSSC(TSSC, 0x2CA0),

	CLK_USB_HS(USB_HS_XCVR,  0x290C),
	CLK_RESET(USB_PHY0, 0x2E20, B(0)),

	CLK_USB_FS(USB_FS1_SRC, 0x2968, chld_usb_fs1_src),
	CLK_SLAVE_RSET(USB_FS1_XCVR,  0x2968, B(9), 0x2974, B(1), USB_FS1_SRC),
	CLK_SLAVE_RSET(USB_FS1_SYS,   0x296C, B(4), 0x2974, B(0), USB_FS1_SRC),

	CLK_USB_FS(USB_FS2_SRC, 0x2988, chld_usb_fs2_src),
	CLK_SLAVE_RSET(USB_FS2_XCVR,  0x2988, B(9), 0x2994, B(1), USB_FS2_SRC),
	CLK_SLAVE_RSET(USB_FS2_SYS,   0x298C, B(4), 0x2994, B(0), USB_FS2_SRC),

	/* Fast Peripheral Bus Clocks */
	CLK_NORATE(GSBI1_P,  0x29C0, B(4)),
	CLK_NORATE(GSBI2_P,  0x29E0, B(4)),
	CLK_NORATE(GSBI3_P,  0x2A00, B(4)),
	CLK_NORATE(GSBI4_P,  0x2A20, B(4)),
	CLK_NORATE(GSBI5_P,  0x2A40, B(4)),
	CLK_NORATE(GSBI6_P,  0x2A60, B(4)),
	CLK_NORATE(GSBI7_P,  0x2A80, B(4)),
	CLK_NORATE(GSBI8_P,  0x2AA0, B(4)),
	CLK_NORATE(GSBI9_P,  0x2AC0, B(4)),
	CLK_NORATE(GSBI10_P, 0x2AE0, B(4)),
	CLK_NORATE(GSBI11_P, 0x2B00, B(4)),
	CLK_NORATE(GSBI12_P, 0x2B20, B(4)),

	CLK_NORATE(TSIF_P, 0x2700, B(4)),

	CLK_NORATE(USB_FS1_P, 0x2960, B(4)),
	CLK_NORATE(USB_FS2_P, 0x2980, B(4)),

	/*
	 * Multimedia Clocks
	 */

	CLK_CAM(CAM, 0x0148),

	CLK_CSI(CSI_SRC, 0x0048),
	CLK_SLAVE_MM(CSI0, 0x0040, B(0), CSI_SRC),
	CLK_SLAVE_MM(CSI1, 0x0040, B(7), CSI_SRC),

	CLK_GFX2D0(GFX2D0, 0x0070),
	CLK_GFX2D1(GFX2D1, 0x007C),
	CLK_GFX3D(GFX3D,   0x008C),

	CLK_IJPEG(IJPEG, 0x00A0),
	CLK_JPEGD(JPEGD, 0x00AC),

	CLK_MDP(MDP, 0x00D0),
	CLK_MDP_VSYNC(MDP_VSYNC, 0x0058),

	CLK_PIXEL_MDP(PIXEL_MDP, 0x00DC),
	CLK_SLAVE_MM(PIXEL_LCDC, 0x00D4, B(8), PIXEL_MDP),

	CLK_ROT(ROT, 0x00E8),

	CLK_TV(TV_SRC, 0x00F4),
	CLK_SLAVE_MM(TV_ENC,  0x00EC, B(8),  TV_SRC),
	CLK_SLAVE_MM(TV_DAC,  0x00EC, B(10), TV_SRC),
	CLK_SLAVE_MM(MDP_TV,  0x00EC, B(0),  TV_SRC),
	CLK_SLAVE_MM(HDMI_TV, 0x00EC, B(12), TV_SRC),

	CLK_VCODEC(VCODEC, 0x0100),

	CLK_VPE(VPE, 0x0118),

	CLK_VFE(VFE_SRC, 0x010C),
	CLK_SLAVE_MM(VFE,      0x0104, B(0),  VFE_SRC),
	CLK_SLAVE_MM(CSI0_VFE, 0x0104, B(12), VFE_SRC),
	CLK_SLAVE_MM(CSI1_VFE, 0x0104, B(10), VFE_SRC),


	/*
	 * Low Power Audio Clocks
	 */

	CLK_AIF(MI2S_SRC, 0x0048, chld_mi2s_src),
	CLK_SLAVE_LPA(MI2S,   0x0048, B(15), MI2S_SRC),
	CLK_SLAVE_LPA(MI2S_M, 0x0048, B(17), MI2S_SRC),

	CLK_AIF(CODEC_I2S_MIC_SRC, 0x0060, chld_codec_i2s_mic_src),
	CLK_SLAVE_LPA(CODEC_I2S_MIC,   0x0060, B(15), CODEC_I2S_MIC_SRC),
	CLK_SLAVE_LPA(CODEC_I2S_MIC_M, 0x0060, B(17), CODEC_I2S_MIC_SRC),

	CLK_AIF(SPARE_I2S_MIC_SRC, 0x0078, chld_spare_i2s_mic_src),
	CLK_SLAVE_LPA(SPARE_I2S_MIC,   0x0078, B(15), SPARE_I2S_MIC_SRC),
	CLK_SLAVE_LPA(SPARE_I2S_MIC_M, 0x0078, B(17), SPARE_I2S_MIC_SRC),

	CLK_AIF(CODEC_I2S_SPKR_SRC, 0x006C, chld_codec_i2s_spkr_src),
	CLK_SLAVE_LPA(CODEC_I2S_SPKR,   0x006C, B(15), CODEC_I2S_SPKR_SRC),
	CLK_SLAVE_LPA(CODEC_I2S_SPKR_M, 0x006C, B(17), CODEC_I2S_SPKR_SRC),

	CLK_AIF(SPARE_I2S_SPKR_SRC, 0x0084, chld_spare_i2s_spkr_src),
	CLK_SLAVE_LPA(SPARE_I2S_SPKR,   0x0084, B(15), SPARE_I2S_SPKR_SRC),
	CLK_SLAVE_LPA(SPARE_I2S_SPKR_M, 0x0084, B(17), SPARE_I2S_SPKR_SRC),

	CLK_PCM(PCM, 0x0054),
};

static DEFINE_SPINLOCK(clock_reg_lock);
static DEFINE_SPINLOCK(pll_vote_lock);

static int pll_is_voteable(int pll)
{
	switch (pll) {
	case PLL_0:
	case PLL_6:
	case PLL_8:
		return 1;
	default:
		return 0;
	}
}

#define PLL_ACTIVE_MASK	B(16)
void pll_enable(int pll)
{
	uint32_t reg_val;
	unsigned long flags;

	if (!pll_is_voteable(pll))
		return;

	spin_lock_irqsave(&pll_vote_lock, flags);
	if (!pll_count[pll]) {
		reg_val = readl(REG(PLL_ENA_REG));
		reg_val |= (1 << pll);
		writel(reg_val, REG(PLL_ENA_REG));
	}
	pll_count[pll]++;
	spin_unlock_irqrestore(&pll_vote_lock, flags);

	/* TODO:
	 * Once PLL voting is supported, wait here until the PLL is enabled.
	 */
}

void pll_disable(int pll)
{
	uint32_t reg_val;
	unsigned long flags;

	if (!pll_is_voteable(pll))
		return;

	spin_lock_irqsave(&pll_vote_lock, flags);
	if (pll_count[pll])
		pll_count[pll]--;
	else
		pr_warning("Reference count mismatch in PLL disable!\n");

	if (pll_count[pll] == 0) {
		reg_val = readl(REG(PLL_ENA_REG));
		reg_val &= ~(1 << pll);
		writel(reg_val, REG(PLL_ENA_REG));
	}
	spin_unlock_irqrestore(&pll_vote_lock, flags);
}

/*
 * SoC specific register-based control of clocks.
 */
static int _soc_clk_enable(unsigned id)
{
	struct clk_local *clk = &clk_local_tbl[id];
	void *reg = clk->cc_reg;
	uint32_t reg_val;

	reg_val = readl(reg);
	if (clk->type == MND) {
		reg_val |= clk->current_freq->mnd_en_mask;
		writel(reg_val, reg);
	}
	if (clk->root_en_mask) {
		reg_val |= clk->root_en_mask;
		writel(reg_val, reg);
	}
	if (clk->br_en_mask) {
		reg_val |= clk->br_en_mask;
		writel(reg_val, reg);
	}
	return 0;
}

static void _soc_clk_disable(unsigned id)
{
	struct clk_local *clk = &clk_local_tbl[id];
	void *reg = clk->cc_reg;
	uint32_t reg_val = 0;

	reg_val = readl(reg);
	if (clk->br_en_mask) {
		reg_val &= ~(clk->br_en_mask);
		writel(reg_val, reg);
	}
	if (clk->root_en_mask) {
		reg_val &= ~(clk->root_en_mask);
		writel(reg_val, reg);
	}
	if (clk->type == MND) {
		reg_val &= ~(clk->current_freq->mnd_en_mask);
		writel(reg_val, reg);
	}
}

static int soc_clk_enable_nolock(unsigned id)
{
	struct clk_local *clk = &clk_local_tbl[id];
	int ret = 0;

	if (clk->type == RESET)
		return -EPERM;

	if (!clk->count) {
		if (clk->parent != C(NONE))
			soc_clk_enable_nolock(clk->parent);
		pll_enable(clk->current_freq->pll);
		ret = _soc_clk_enable(id);
	}
	clk->count++;

	return ret;
}

static void soc_clk_disable_nolock(unsigned id)
{
	struct clk_local *clk = &clk_local_tbl[id];

	if (!clk->count) {
		pr_warning("Reference count mismatch in clock disable!\n");
		return;
	}
	if (clk->count)
		clk->count--;
	if (clk->count == 0) {
		_soc_clk_disable(id);
		pll_disable(clk->current_freq->pll);
		if (clk->parent != C(NONE))
			soc_clk_disable_nolock(clk->parent);
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

static int soc_clk_reset(unsigned id, enum clk_reset_action action)
{
	struct clk_local *clk = &clk_local_tbl[id];
	uint32_t reg_val, ret = 0;
	unsigned long flags;

	if (clk->reset_reg == NULL)
		return -EPERM;

	spin_lock_irqsave(&clock_reg_lock, flags);

	reg_val = readl(clk->reset_reg);
	switch (action) {
	case CLK_RESET_ASSERT:
		reg_val |= clk->reset_mask;
		break;
	case CLK_RESET_DEASSERT:
		reg_val &= ~(clk->reset_mask);
		break;
	default:
		ret = -EINVAL;
	}
	writel(reg_val, clk->reset_reg);

	spin_unlock_irqrestore(&clock_reg_lock, flags);

	return ret;
}

static int _soc_clk_set_rate(unsigned id, unsigned rate, enum match_types match)
{
	struct clk_local *clk = &clk_local_tbl[id];
	struct clk_freq_tbl *cf = clk->current_freq;
	struct clk_freq_tbl *nf;
	uint32_t *chld = clk->children;
	uint32_t reg_val = 0;
	int i, ret = 0;
	unsigned long flags;

	if (clk->type == NORATE || clk->type == RESET)
		return -EPERM;

	spin_lock_irqsave(&clock_reg_lock, flags);

	if (rate == cf->freq_hz)
		goto release_lock;

	/* Find new frequency based on match rule. */
	switch (match) {
	case MATCH_MIN:
		for (nf = clk->freq_tbl; nf->freq_hz != FREQ_END; nf++)
			if (nf->freq_hz >= rate)
				break;
		break;
	default:
	case MATCH_EXACT:
		for (nf = clk->freq_tbl; nf->freq_hz != FREQ_END; nf++)
			if (nf->freq_hz == rate)
				break;
		break;
	}

	if (nf->freq_hz == FREQ_END) {
		ret = -EINVAL;
		goto release_lock;
	}

	/* Disable clocks if clock is not glitch-free banked. */
	if (clk->banked_mnd_masks == NULL) {
		/* Disable all branches to prevent jitter. */
		for (i = 0; chld && chld[i] != C(NONE); i++) {
			struct clk_local *ch = &clk_local_tbl[chld[i]];
			/* Don't bother turning off if it is already off.
			 * Checking ch->count is cheaper (cache) than reading
			 * and writing to a register (uncached/unbuffered). */
			if (ch->count) {
				reg_val = readl(ch->cc_reg);
				reg_val &= ~(ch->br_en_mask);
				writel(reg_val, ch->cc_reg);
			}
		}
		if (clk->count)
			_soc_clk_disable(id);
	}

	if (clk->count) {
		/* Turn on PLL of the new freq. */
		pll_enable(nf->pll);
	}

	/* Perform clock-specific frequency switch operations. */
	BUG_ON(!clk->set_rate);
	clk->set_rate(clk, nf);

	if (clk->count) {
		/* Turn off PLL of the old freq. */
		pll_disable(cf->pll);
	}

	/* Current freq must be updated before _soc_clk_enable() is called to
	 * make sure the MNCNTR_E bit is set correctly. */
	clk->current_freq = nf;

	/* Enable any clocks that were disabled. */
	if (clk->banked_mnd_masks == NULL) {
		if (clk->count)
			_soc_clk_enable(id);
		/* Enable only branches that were ON before. */
		for (i = 0; chld && chld[i] != C(NONE); i++) {
			struct clk_local *ch = &clk_local_tbl[chld[i]];
			if (ch->count) {
				reg_val = readl(ch->cc_reg);
				reg_val |= ch->br_en_mask;
				writel(reg_val, ch->cc_reg);
			}
		}
	}

release_lock:
	spin_unlock_irqrestore(&clock_reg_lock, flags);
	return ret;
}

static int soc_clk_set_rate(unsigned id, unsigned rate)
{
	return _soc_clk_set_rate(id, rate, MATCH_EXACT);
}

static int soc_clk_set_min_rate(unsigned id, unsigned rate)
{
	return _soc_clk_set_rate(id, rate, MATCH_MIN);
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
	struct clk_local *clk = &clk_local_tbl[id];
	unsigned long flags;
	unsigned ret = 0;

	if (clk->type == NORATE || clk->type == RESET)
		return 0;

	spin_lock_irqsave(&clock_reg_lock, flags);
	ret = clk->current_freq->freq_hz;
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
	struct clk_local *clk = &clk_local_tbl[id];
	struct clk_freq_tbl *f;

	if (clk->type == NORATE || clk->type == RESET)
		return -EINVAL;

	for (f = clk->freq_tbl; f->freq_hz != FREQ_END; f++)
		if (f->freq_hz >= rate)
			return f->freq_hz;

	return -EPERM;
}

struct clk_ops clk_ops_8x60 = {
	.enable = soc_clk_enable,
	.disable = soc_clk_disable,
	.reset = soc_clk_reset,
	.set_rate = soc_clk_set_rate,
	.set_min_rate = soc_clk_set_min_rate,
	.set_max_rate = soc_clk_set_max_rate,
	.set_flags = soc_clk_set_flags,
	.get_rate = soc_clk_get_rate,
	.is_enabled = soc_clk_is_enabled,
	.round_rate = soc_clk_round_rate,
};

void __init msm_clk_soc_set_ops(struct clk *clk)
{
	return;
}

static struct reg_init {
	void *reg;
	uint32_t mask;
	uint32_t val;
} ri_list[] __initdata = {

	/* XXX START OF TEMPORARY CODE XXX
	 * The RPM bootloader should take care of this. */

	/* PXO src = PXO */
	{REG(0x2EA0), 0x3, 0x1},

	/* XXX PLL8 (MM_GPERF_PLL) @ 384MHz. */
	{REG(0x3144), 0x3FF, 0xF},   /* LVAL = 15 */
	{REG(0x3148), 0x7FFFF, 0x5}, /* MVAL = 5 */
	{REG(0x314C), 0x7FFFF, 0x8}, /* NVAL = 8 */
	/* Enable MN, set VCO, main out. */
	{REG(0x3154), B(23) | B(22) | 0x3 << 16, B(23) | B(22) | 0x1 << 16},
	 /* Don't bypass, enable outputs, deassert MND reset. */
	{REG(0x3140), 0x7, 0x7},

	/* XXX PLL0 @ (MM_GPLL0) @ 1000MHz. */
	{REG(0x30C4), 0x3FF, 0x28},    /* LVAL = 40 */
	{REG(0x30C8), 0x7FFFF, 0x109}, /* MVAL = 265 */
	{REG(0x30CC), 0x7FFFF, 0x180}, /* NVAL = 384 */
	/* Enable MN, set VCO, main out. */
	{REG(0x30D4), B(23) | B(22) | 0x3 << 20 | 0x3 << 16,
		B(23) | B(22) | 0x1 << 16},
	/* Don't bypass, enable outputs, deassert MND reset. */
	{REG(0x30C0), 0x7, 0x7},

	/* XXX MM_PLL0 (PLL1) @ 1320MHz */
	{REG_MM(0x0304), 0xFF, 0x35},    /* LVAL = 53 */
	{REG_MM(0x0308), 0x7FFFF, 0x5B}, /* MVAL = 91 */
	{REG_MM(0x030C), 0x7FFFF, 0x80}, /* NVAL = 128 */
	/* Enable MN, set VCO, misc config. */
	{REG_MM(0x0310), 0xFFFFFFFF, 0x14580},
	 /* Don't bypass, enable outputs, deassert MND reset. */
	{REG_MM(0x0300), 0xF, 0x7},

	/* XXX MM_PLL1 (PLL2) @ 800MHz */
	{REG_MM(0x0320), 0x3FF, 0x20},   /* LVAL = 32 */
	{REG_MM(0x0324), 0x7FFFF, 0x35}, /* MVAL = 53 */
	{REG_MM(0x0328), 0x7FFFF, 0x60}, /* NVAL = 96 */
	/* Enable MN, set VCO, main out. */
	{REG_MM(0x032C), 0xFFFFFFFF, 0x00C22080},
	 /* Don't bypass, enable outputs, deassert MND reset. */
	{REG_MM(0x031C), 0x7, 0x7},

	/* XXX MM_PLL2 (PLL3) @ <Varies>, 50.4005MHz for now. */
	{REG_MM(0x033C), 0x3FF, 0x7},      /* LVAL = 7 */
	{REG_MM(0x0340), 0x7FFFF, 0x189D}, /* MVAL = 6301 */
	{REG_MM(0x0344), 0x7FFFF, 0x34BC}, /* NVAL = 13500 */
	/* Enable MN, set VCO, main out, postdiv4. */
	{REG_MM(0x0348), 0xFFFFFFFF, 0x00E02080},
	/* MXO reference, don't bypass, enable outputs, deassert MND reset. */
	{REG_MM(0x0338), 0x1F, 0x17},

	/* XXX Turn on all SC0 voteable PLLs (PLL0, PLL6, PLL8). */
	{REG(PLL_ENA_REG), 0x141, 0x141},

	/* XXX END OF TEMPORARY CODE XXX */

	/* Enable locally controlled peripheral HCLKs in software mode. */
	{REG(0x2700), 0x70, 0x10}, /* EN TSIF_HCLK */
	{REG(0x2820), 0x70, 0x10}, /* EN SDC1_HCLK */
	{REG(0x2840), 0x70, 0x10}, /* EN SDC2_HCLK */
	{REG(0x2860), 0x70, 0x10}, /* EN SDC3_HCLK */
	{REG(0x2880), 0x70, 0x10}, /* EN SDC4_HCLK */
	{REG(0x28A0), 0x70, 0x10}, /* EN SDC5_HCLK */
	{REG(0x2900), 0x70, 0x10}, /* EN USB_HS1_HCLK */
	{REG(0x2960), 0x70, 0x10}, /* EN USB_FS1_HCLK */
	{REG(0x2980), 0x70, 0x10}, /* EN USB_FS2_HCLK */
	{REG(0x29C0), 0x70, 0x10}, /* EN GSBI1_HCLK */
	{REG(0x29E0), 0x70, 0x10}, /* EN GSBI2_HCLK */
	{REG(0x2A00), 0x70, 0x10}, /* EN GSBI3_HCLK */
	{REG(0x2A20), 0x70, 0x10}, /* EN GSBI4_HCLK */
	{REG(0x2A40), 0x70, 0x10}, /* EN GSBI5_HCLK */
	{REG(0x2A60), 0x70, 0x10}, /* EN GSBI6_HCLK */
	{REG(0x2A80), 0x70, 0x10}, /* EN GSBI7_HCLK */
	{REG(0x2AA0), 0x70, 0x10}, /* EN GSBI8_HCLK */
	{REG(0x2AC0), 0x70, 0x10}, /* EN GSBI9_HCLK */
	{REG(0x2AE0), 0x70, 0x10}, /* EN GSBI10_HCLK */
	{REG(0x2B00), 0x70, 0x10}, /* EN GSBI11_HCLK */
	{REG(0x2B20), 0x70, 0x10}, /* EN GSBI12_HCLK */

	{REG_MM(0x0204), 0x1, 0x0}, /* MM SW_RESET_ALL */

	/* Enable all MM AHB clocks in software mode. */
	{REG_MM(0x0004), 0x43C7, 0x0241}, /* MM AHB = PLL2/10 */
	{REG_MM(0x0008), 0xFFFFFFFF, 0x93BDFEFF}, /* MM AHB_EN */
	{REG_MM(0x0038), 0xFFFFFFFF, 0x1}, /* MM AHB_EN2 */
	{REG_MM(0x020C), 0xFFFFFFFF, 0x0}, /* MM SW_RESET_AHB */

	/* Enable all MM AXI clocks. */
	{REG_MM(0x0014), 0x0FFFFFFF, 0x4248451}, /* MM AXI_NS */
	{REG_MM(0x0018), 0xFFFFFFFF, 0x17FC0001}, /* MM MAXI_EN */
	{REG_MM(0x0020), 0x7FFFFFFF, 0x75200400}, /* MM MAXI_EN2 */
	{REG_MM(0x002C), 0xFFFFFFFF, 0x200400}, /* MM MAXI_EN3 */
	{REG_MM(0x0030), 0x3FFF, 0x1C7}, /* MM SAXI_EN */
	{REG_MM(0x0208), 0xE37F, 0x0}, /* SW_RESET_AXI */

	/* Deassert all MM core resets. */
	{REG_MM(0x0210), 0x1FFFFFF, 0x0}, /* MM SW_RESET_CORE */
};

#define set_1rate(clk) \
	soc_clk_set_rate(C(clk), clk_local_tbl[C(clk)].freq_tbl->freq_hz)
void __init msm_clk_soc_init(void)
{
	int i;
	uint32_t val;

	for (i = 0; i < ARRAY_SIZE(ri_list); i++) {
		val = readl(ri_list[i].reg);
		val &= ~ri_list[i].mask;
		val |= ri_list[i].val;
		writel(val, ri_list[i].reg);
	}
}

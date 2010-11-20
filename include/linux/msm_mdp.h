/* include/linux/msm_mdp.h
 *
 * Copyright (C) 2007 Google Incorporated
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _MSM_MDP_H_
#define _MSM_MDP_H_

#include <linux/types.h>
#include <linux/fb.h>

#define MSMFB_IOCTL_MAGIC 'm'
#define MSMFB_GRP_DISP          _IOW(MSMFB_IOCTL_MAGIC, 1, unsigned int)
#define MSMFB_BLIT              _IOW(MSMFB_IOCTL_MAGIC, 2, unsigned int)
#define MSMFB_SUSPEND_SW_REFRESHER _IOW(MSMFB_IOCTL_MAGIC, 128, unsigned int)
#define MSMFB_RESUME_SW_REFRESHER _IOW(MSMFB_IOCTL_MAGIC, 129, unsigned int)
#define MSMFB_CURSOR _IOW(MSMFB_IOCTL_MAGIC, 130, struct fb_cursor)
#define MSMFB_SET_LUT _IOW(MSMFB_IOCTL_MAGIC, 131, struct fb_cmap)
#define MSMFB_HISTOGRAM _IOWR(MSMFB_IOCTL_MAGIC, 132, struct mdp_histogram)
/* new ioctls's for set/get ccs matrix */
#define MSMFB_GET_CCS_MATRIX  _IOWR(MSMFB_IOCTL_MAGIC, 133, struct mdp_ccs)
#define MSMFB_SET_CCS_MATRIX  _IOW(MSMFB_IOCTL_MAGIC, 134, struct mdp_ccs)

#define MDP_IMGTYPE2_START 0x10000
enum {
	MDP_RGB_565,      /* RGB 565 planer */
	MDP_XRGB_8888,    /* RGB 888 padded */
	MDP_Y_CBCR_H2V2,  /* Y and CbCr, pseudo planer w/ Cb is in MSB */
	MDP_ARGB_8888,    /* ARGB 888 */
	MDP_RGB_888,      /* RGB 888 planer */
	MDP_Y_CRCB_H2V2,  /* Y and CrCb, pseudo planer w/ Cr is in MSB */
	MDP_YCRYCB_H2V1,  /* YCrYCb interleave */
	MDP_Y_CRCB_H2V1,  /* Y and CrCb, pseduo planer w/ Cr is in MSB */
	MDP_Y_CBCR_H2V1,   /* Y and CrCb, pseduo planer w/ Cr is in MSB */
	MDP_RGBA_8888,    /* ARGB 888 */
	MDP_BGRA_8888,	  /* ABGR 888 */
	MDP_IMGTYPE_LIMIT,
	MDP_BGR_565 = MDP_IMGTYPE2_START,      /* BGR 565 planer */
	MDP_FB_FORMAT,    /* framebuffer format */
	MDP_IMGTYPE_LIMIT2 /* Non valid image type after this enum */
};

enum {
	PMEM_IMG,
	FB_IMG,
};

/* flag values */
#define MDP_ROT_NOP 0
#define MDP_FLIP_LR 0x1
#define MDP_FLIP_UD 0x2
#define MDP_ROT_90 0x4
#define MDP_ROT_180 (MDP_FLIP_UD|MDP_FLIP_LR)
#define MDP_ROT_270 (MDP_ROT_90|MDP_FLIP_UD|MDP_FLIP_LR)
#define MDP_DITHER 0x8
#define MDP_BLUR 0x10

#define MDP_DEINTERLACE 0x80000000
#define MDP_SHARPENING  0x40000000

#define MDP_TRANSP_NOP 0xffffffff
#define MDP_ALPHA_NOP 0xff

struct mdp_rect {
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
};

struct mdp_img {
	uint32_t width;
	uint32_t height;
	uint32_t format;
	uint32_t offset;
	int memory_id;		/* the file descriptor */
};

/*
 * {3x3} + {3} ccs matrix
 */

#define MDP_CCS_RGB2YUV 	0
#define MDP_CCS_YUV2RGB 	1

#define MDP_CCS_SIZE	9
#define MDP_BV_SIZE	3

struct mdp_ccs {
	int direction;			/* MDP_CCS_RGB2YUV or YUV2RGB */
	uint16_t ccs[MDP_CCS_SIZE];	/* 3x3 color coefficients */
	uint16_t bv[MDP_BV_SIZE];	/* 1x3 bias vector */
};

struct mdp_blit_req {
	struct mdp_img src;
	struct mdp_img dst;
	struct mdp_rect src_rect;
	struct mdp_rect dst_rect;
	uint32_t alpha;
	uint32_t transp_mask;
	uint32_t flags;
	int sharpening_strength;  /* -127 <--> 127, default 64 */
};

struct mdp_blit_req_list {
	uint32_t count;
	struct mdp_blit_req req[];
};

struct mdp_histogram {
	uint32_t frame_cnt;
	uint32_t bin_cnt;
	uint32_t *r;
	uint32_t *g;
	uint32_t *b;
};

#endif /*_MSM_MDP_H_*/

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

#ifndef _MSM_VDEC_H_
#define _MSM_VDEC_H_

#include <linux/types.h>

#define VDEC_IOCTL_MAGIC 'v'

#define VDEC_IOCTL_INITIALIZE   _IOWR(VDEC_IOCTL_MAGIC, 1, struct vdec_init)
#define VDEC_IOCTL_SETBUFFERS   _IOW(VDEC_IOCTL_MAGIC, 2, struct vdec_memory)
#define VDEC_IOCTL_QUEUE        _IOW(VDEC_IOCTL_MAGIC, 3, struct vdec_input_buf)
#define VDEC_IOCTL_REUSEFRAMEBUFFER  _IOW(VDEC_IOCTL_MAGIC, 4, unsigned int)
#define VDEC_IOCTL_FLUSH        _IOW(VDEC_IOCTL_MAGIC, 5, unsigned int)
#define VDEC_IOCTL_EOS          _IO(VDEC_IOCTL_MAGIC, 6)
#define VDEC_IOCTL_GETMSG       _IOR(VDEC_IOCTL_MAGIC, 7, struct vdec_msg)
#define VDEC_IOCTL_CLOSE        _IO(VDEC_IOCTL_MAGIC, 8)

enum {
	VDEC_FRAME_DECODE_OK,
	VDEC_FRAME_DECODE_ERR,
	VDEC_FATAL_ERR,
	VDEC_FLUSH_FINISH,
	VDEC_EOS,
};

enum {
	VDEC_FLUSH_INPUT,
	VDEC_FLUSH_OUTPUT,
	VDEC_FLUSH_ALL
};

struct vdec_input_buf {
	u32 offset;
	u32 data;
	u32 size;
	int timestamp;
	int avsync_state;
	u32 flags;
};

struct vdec_buf_desc {
	u32 bufsize;
	u32 bufnum;
};

struct vdec_buf_req {
	struct vdec_buf_desc input;
	struct vdec_buf_desc output;
	struct vdec_buf_desc dec;
};

struct vdec_buf_info {
	u32 src_id;
	u32 offset;
	u32 size;
};

struct vdec_config {
	u32 fourcc;		/* video format */
	u32 width;		/* source width */
	u32 height;		/* source height */
	u32 order;		/* render decoder order */
	u32 notify_enable;	/* enable notify input buffer done event */
	u32 vc1_rowbase;
	u32 h264_startcode_detect;
	u32 h264_nal_len_size;
	u32 postproc_flag;
};

struct vdec_vc1_panscan_regions {
	int num;
	int width[4];
	int height[4];
	int xoffset[4];
	int yoffset[4];
};

struct vdec_frame_info {
	u32 status; 		/* video decode status */
	u32 offset;		/* buffer offset */
	u32 data1;		/* user data field 1 */
	u32 data2;		/* user data field 2 */
	u32 timestamp;		/* timestamp, in msec */
	u32 dec_width;		/* frame roi width */
	u32 dec_height;		/* frame roi height */
	u32 disp_width;		/* frame width */
	u32 disp_height;	/* frame height */
	u32 picture_type[2];	/* picture coding type */
	u32 picture_format;	/* picture coding format */
	u32 vc1_rangeY;		/* luma range mapping */
	u32 vc1_rangeUV;	/* chroma range mapping */
	u32 picture_resolution; /* scaling factor */
	u32 frame_disp_repeat;	/* how often repeated by disp */
	u32 repeat_first_field; /* repeat 1st field after 2nd */
	u32 top_field_first;	/* top field displayed first */
	u32 interframe_interp;	/* not for inter-frame interp */
	struct vdec_vc1_panscan_regions panscan; /* pan region */
	u32 concealed_macblk_num;/* number of concealed macro blk */
	u32 flags;		/* input flags */
};

struct vdec_buffers {
	struct vdec_buf_info out;
	struct vdec_buf_info in;
	struct vdec_buf_info dec;
};

struct vdec_sequence {
	u8 *header;
	u32 len;
};

struct vdec_config_sps {
	struct vdec_config cfg;
	struct vdec_sequence seq;
};

#define VDEC_MSG_REUSEINPUTBUFFER 	1
#define VDEC_MSG_FRAMEDONE 		2

struct vdec_msg {
	u32 id;

	union {
		/* id = VDEC_MSG_REUSEINPUTBUFFER */
		u32 buf_id;
		/* id = VDEC_MSG_FRAMEDONE */
		struct vdec_frame_info vfr_info;
	};
};

struct vdec_init {
	struct vdec_config_sps sps_cfg;
	struct vdec_buf_req *buf_req;
};

struct vdec_memory {
	struct vdec_buffers buf;
	u32 id;
};

#endif /* _MSM_VDEC_H_ */

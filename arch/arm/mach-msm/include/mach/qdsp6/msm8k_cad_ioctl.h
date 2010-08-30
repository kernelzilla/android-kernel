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

#ifndef CADIOCTL_H
#define CADIOCTL_H

#include <linux/types.h>


enum cad_stream_app_enum_type {
	CAD_STREAM_APP_UNKNOWN                  = 0x00,
	CAD_STREAM_APP_VOICE                    = 0x01,
	CAD_STREAM_APP_PLAYBACK                 = 0x02,
	CAD_STREAM_APP_DTMF			= 0x04,
	CAD_STREAM_APP_RINGER                   = 0x08,
	CAD_STREAM_APP_SYSTEM_SOUND             = 0x10,
	CAD_STREAM_APP_RECORD                   = 0x20,
	CAD_STREAM_APP_A2DP_MIX_MODE		= 0x40
};


enum cad_stream_buf_mem_type {
	CAD_STREAM_BUF_MEM_UNKNOWN                 = 0x00,
	CAD_STREAM_BUF_MEM_HEAP                    = 0x01,
	CAD_STREAM_BUF_MEM_TYPE_CONTIGUOUS_HEAP    = 0x02
};


#define CAD_IOCTL_CMD_SET_STREAM_EVENT_LSTR                 0x01075ee6

#define CAD_IOCTL_CMD_STREAM_START                          0x01075ee7

#define CAD_IOCTL_CMD_STREAM_PAUSE                          0x01075ee8

#define CAD_IOCTL_CMD_STREAM_RESUME                         0x01075ee9

#define CAD_IOCTL_CMD_STREAM_FLUSH                          0x01075eea

#define CAD_IOCTL_CMD_SET_STREAM_DEVICE                     0x01075eec

#define CAD_IOCTL_CMD_SET_STREAM_CONFIG                     0x01075eed

#define CAD_IOCTL_CMD_SET_STREAM_INFO                       0x01075eee

#define CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG              0x01075eef

#define CAD_IOCTL_CMD_SET_STREAM_AV_SYNC                    0x0107605c

#define CAD_IOCTL_CMD_GEN_DTMF                              0x01087ea6

#define CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG              0x01075ef0

#define CAD_IOCTL_CMD_DEVICE_SET_CAL                        0x01075ef1

#define CAD_IOCTL_CMD_DEVICE_SET_GLOBAL_DEFAULT             0x01075ef2

#define QDSP_IOCTL_CMD_STREAM_DTMF_STOP                     0x01087554

#define QDSP_IOCTL_CMD_SET_DEVICE_VOL                       0x01087a4a

#define QDSP_IOCTL_CMD_SET_DEVICE_MUTE                      0x01087a4b

#define QDSP_IOCTL_CMD_SET_STREAM_MUTE                      0x01087b0c

#define QDSP_IOCTL_CMD_SET_STREAM_VOL                       0x01087b0b


struct cad_device_struct_type {
	u32	device;
	u32	reserved;
};

typedef void (*cad_evt_cb_func_type)(u32 event, void *evt_packet,
		u32 evt_packet_len, void *client_data);

struct cad_event_struct_type {
	cad_evt_cb_func_type	callback;
	void			*client_data;
};


struct cad_stream_config_struct_type {
	void		*format_block;
	u32		format_block_len;
};

struct cad_stream_info_struct_type {
	enum cad_stream_app_enum_type	app_type;
	u32				ses_buf_max_size;
	enum cad_stream_buf_mem_type	buf_mem_type;
	u32				priority;
};

struct cad_stream_filter_struct_type {
	u32	filter_type;
	void	*format_block;
	u32	format_block_len;
};

struct cad_stream_device_struct_type {
	u32	*device;
	u32	device_len;
};

struct cad_stream_av_sync_struct_type {
	u32	av_sync_interval;
};

struct cad_cmd_gen_dtmf {
	u16	path;
	u16	dtmf_hi;
	u16	dtmf_low;
	u16	duration;
	u16	tx_gain;
	u16	rx_gain;
	u16	mixing;
};

struct qdsp_set_device_volume {
	u32	device_id;
	u32	path;         /*  0 == Rx, 1 == Tx and 2 == both */
	s32	volume;       /* in mB. */
};

struct qdsp_set_device_mute {
	u32	device_id;
	u32	path;         /* 0 == Rx, 1 == Tx and 2 == both */
	u32	mute;         /* 0 == UnMute,  1 == Mute        */
};

struct qdsp_set_stream_mute {
	u32	mute;         /* 0 == UnMute,  1 == Mute */
};

struct qdsp_set_stream_volume {
	s32	volume;       /* in mB */
};

#endif

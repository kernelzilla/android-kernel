/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CADIOCTL_H
#define CADIOCTL_H

#include <linux/types.h>
#include <mach/qdsp6/msm8k_adsp_audio_types.h>

#define CAD_MAX_SESSION			10

enum cad_stream_app_enum_type {
	CAD_STREAM_APP_UNKNOWN                  = 0x00,
	CAD_STREAM_APP_VOICE                    = 0x01,
	CAD_STREAM_APP_PLAYBACK                 = 0x02,
	CAD_STREAM_APP_DTMF			= 0x04,
	CAD_STREAM_APP_RINGER                   = 0x08,
	CAD_STREAM_APP_SYSTEM_SOUND             = 0x10,
	CAD_STREAM_APP_RECORD                   = 0x20,
	CAD_STREAM_APP_A2DP_MIX_MODE		= 0x40,
	CAD_STREAM_APP_AUDIO_VIDEO		= 0x80,
	CAD_STREAM_APP_MIXED_RECORD		= 0x100
};


enum cad_stream_buf_mem_type {
	CAD_STREAM_BUF_MEM_UNKNOWN                 = 0x00,
	CAD_STREAM_BUF_MEM_HEAP                    = 0x01,
	CAD_STREAM_BUF_MEM_TYPE_CONTIGUOUS_HEAP    = 0x02
} __attribute__ ((packed));


#define CAD_IOCTL_CMD_SET_STREAM_EVENT_LSTR		0x01075ee6
#define CAD_IOCTL_CMD_STREAM_START			0x01075ee7
#define CAD_IOCTL_CMD_STREAM_PAUSE			0x01075ee8
#define CAD_IOCTL_CMD_STREAM_RESUME			0x01075ee9
#define CAD_IOCTL_CMD_STREAM_FLUSH			0x01075eea
#define CAD_IOCTL_CMD_STREAM_END_OF_STREAM		0x0108b150
#define CAD_IOCTL_CMD_SET_STREAM_DEVICE			0x01075eec
#define CAD_IOCTL_CMD_SET_STREAM_CONFIG			0x01075eed
#define CAD_IOCTL_CMD_SET_STREAM_INFO			0x01075eee
#define CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG		0x01075eef
#define CAD_IOCTL_CMD_SET_STREAM_AV_SYNC		0x0108d1e2
#define CAD_IOCTL_CMD_GET_AUDIO_TIME			0x0108c26c
#define CAD_IOCTL_CMD_GEN_DTMF				0x01087ea6
#define CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG		0x01075ef0
#define CAD_IOCTL_CMD_DEVICE_SET_CAL			0x01075ef1
#define CAD_IOCTL_CMD_DEVICE_AUXPGA_ENABLE		0x0108cb98
#define CAD_IOCTL_CMD_DEVICE_AUXPGA_DISABLE		0x0108cb99
#define CAD_IOCTL_CMD_SET_STREAM_BITRATE		0x0108cb9a
#define CAD_IOCTL_CMD_SET_AAC_SBR_PS			0x0108d1e6
#define CAD_IOCTL_CMD_DUAL_MONO_REMAP			0x0108d1e4

#define CAD_STREAM_DUALMONO_REMAP_TYPE1		0
#define CAD_STREAM_DUALMONO_REMAP_TYPE2		1
#define CAD_STREAM_DUALMONO_REMAP_TYPE3		2
#define CAD_STREAM_DUALMONO_REMAP_TYPE4		4

#define CAD_IOCTL_CMD_AAC_DO_SAMPLE_SLIP		0x0108d1e3
#define CAD_IOCTL_CMD_DEVICE_SET_GLOBAL_DEFAULT		0x01075ef2



struct cad_device_struct_type {
	u32	device;
	u32	reserved;
} __attribute__ ((packed));

typedef void (*cad_evt_cb_func_type)(u32 event, void *evt_packet,
		u32 evt_packet_len, void *client_data);

struct cad_event_struct_type {
	cad_evt_cb_func_type	callback;
	void			*client_data;
} __attribute__ ((packed));


struct cad_stream_config_struct_type {
	void		*format_block;
	u32		format_block_len;
} __attribute__ ((packed));

struct cad_stream_info_struct_type {
	enum cad_stream_app_enum_type	app_type;
	u32				ses_buf_max_size;
	enum cad_stream_buf_mem_type	buf_mem_type;
	u32				priority;
} __attribute__ ((packed));

struct cad_stream_filter_struct_type {
	u32	filter_type;
	void	*format_block;
	u32	format_block_len;
} __attribute__ ((packed));

struct cad_stream_device_struct_type {
	u32	*device;
	u32	device_len;
} __attribute__ ((packed));

struct cad_stream_av_sync_struct_type {
	u32	av_sync_interval;
} __attribute__ ((packed));

struct cad_cmd_gen_dtmf {
	u16	path;
	u16	dtmf_hi;
	u16	dtmf_low;
	u16	duration;
	u16	tx_gain;
	u16	rx_gain;
	u16	mixing;
} __attribute__ ((packed));

struct qdsp_set_device_volume {
	u32	device_id;
	u32	path;         /*  0 == Rx, 1 == Tx and 2 == both */
	s32	volume;       /* in mB. */
} __attribute__ ((packed));

struct qdsp_set_device_mute {
	u32	device_id;
	u32	path;         /* 0 == Rx, 1 == Tx and 2 == both */
	u32	mute;         /* 0 == UnMute,  1 == Mute        */
} __attribute__ ((packed));

struct qdsp_set_stream_mute {
	u32	mute;         /* 0 == UnMute,  1 == Mute */
} __attribute__ ((packed));

struct qdsp_set_stream_volume {
	s32	volume;       /* in mB */
} __attribute__ ((packed));

struct cad_filter_struct {
	u32 filter_type;
	u32 cmd;
	void *format_block;
	u32 format_block_len;
}  __attribute__ ((packed));

#define CAD_DEVICE_FILTER_TYPE_EQ        0x0108c384

#define CAD_FILTER_EQ_DEVICE_CONFIG                    0x0108b444
#define CAD_FILTER_EQ_STREAM_CONFIG                    0x0108b445


#endif

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

#ifndef CAD_H
#define CAD_H

#include <linux/kernel.h>
#include <linux/msm_audio.h>

#define CAD_RES_SUCCESS		0
#define CAD_RES_FAILURE		-1
#define CAD_RES_UNSUPPORTED	-2
#define CAD_RES_Q6_BUSY		-3
#define CAD_RES_Q6_PREEMPTED	-4
#define CAD_RES_Q6_UNEXPECTED	-5

#define CAD_FORMAT_PCM		0x00
#define CAD_FORMAT_ADPCM	0x01
#define CAD_FORMAT_MP3		0x02
#define CAD_FORMAT_RA		0x03
#define CAD_FORMAT_WMA		0x04	/* WMA std (V9) */
#define CAD_FORMAT_AAC		0x05
#define CAD_FORMAT_MIDI		0x07
#define CAD_FORMAT_YADPCM	0x08
#define CAD_FORMAT_QCELP8K	0x09
#define CAD_FORMAT_AMRWB	0x0A
#define CAD_FORMAT_AMRNB	0x0B
#define CAD_FORMAT_EVRC		0x0C
#define CAD_FORMAT_DTMF		0x0D
#define CAD_FORMAT_QCELP13K	0x0E
#define CAD_FORMAT_SBC		0x0F
#define CAD_FORMAT_WMA_PRO	0x10	/* WMA Pro (V10) */
#define CAD_FORMAT_EVRCB	0x11

#define CAD_OPEN_OP_READ   0x01
#define CAD_OPEN_OP_WRITE  0x02

#define CAD_OPEN_OP_DEVICE_CTRL  0x04


struct cad_open_struct_type {
	u32	op_code;
	u32	format;
	u32	group_id;
	u32	payload_len;
	u32	payload;
};


struct cad_buf_struct_type {
	void    *buffer;
	u32     phys_addr;
	u32     max_size;
	u32     actual_size;
	s64	time_stamp;
};


extern u8 *g_audio_mem;
extern u32 g_audio_base;

s32 cad_open(struct cad_open_struct_type *open_param);

s32 cad_close(s32 driver_handle);

s32 cad_write(s32 driver_handle, struct cad_buf_struct_type *buf);

s32 cad_read(s32 driver_handle, struct cad_buf_struct_type *buf);

s32 cad_ioctl(s32 driver_handle, u32 cmd_code, void *cmd_buf,
	u32 cmd_buf_len);

int audio_switch_device(int new_device);

int audio_set_device_volume(int vol);

int audio_set_device_volume_path(struct msm_vol_info *v);

int audio_set_device_mute(struct msm_mute_info *m);

int audio_resync_afe_clk(void);

#endif

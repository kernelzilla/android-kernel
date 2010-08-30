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

#ifndef CAD_H
#define CAD_H

#include <linux/kernel.h>

#define CAD_RES_SUCCESS       0
#define CAD_RES_FAILURE      -1
#define CAD_RES_UNSUPPORTED  -2

#define CAD_FORMAT_PCM      0x00
#define CAD_FORMAT_ADPCM    0x01
#define CAD_FORMAT_MP3      0x02
#define CAD_FORMAT_RA       0x03
#define CAD_FORMAT_WMA      0x04
#define CAD_FORMAT_AAC      0x05
#define CAD_FORMAT_MIDI     0x07
#define CAD_FORMAT_YADPCM   0x08
#define CAD_FORMAT_QCELP    0x09
#define CAD_FORMAT_AMRWB    0x0A
#define CAD_FORMAT_AMRNB    0x0B
#define CAD_FORMAT_EVRC     0x0C
#define CAD_FORMAT_DTMF     0x0D

#define CAD_OPEN_OP_READ   0x01
#define CAD_OPEN_OP_WRITE  0x02

#define CAD_OPEN_OP_DEVICE_CTRL  0x04


struct cad_open_struct_type {
	u32  op_code;
	u32  format;
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

int audio_set_device_mute(int mute_info);

#endif

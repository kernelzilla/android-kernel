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

#ifndef CADMODULE_H
#define CADMODULE_H

#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_cad_event.h>

#define CAD_MAX_SESSION         10


typedef s32 (*cad_open_func_ptr_type)(s32 session_id,
	struct cad_open_struct_type *open_param);

typedef s32 (*cad_close_func_ptr_type)(s32 session_id);

typedef s32 (*cad_write_func_ptr_type)(s32 session_id,
	struct cad_buf_struct_type *buf);

typedef s32 (*cad_read_func_ptr_type)(s32 session_id,
	struct cad_buf_struct_type *buf);

typedef s32 (*cad_ioctl_func_ptr_type)(s32 session_id, u32 cmd_code,
	void *cmd_buf, u32 cmd_len);

struct cad_func_tbl_type {
	cad_open_func_ptr_type          open;
	cad_close_func_ptr_type         close;
	cad_write_func_ptr_type         write;
	cad_read_func_ptr_type          read;
	cad_ioctl_func_ptr_type         ioctl;
};


s32 cad_resource_init(struct cad_func_tbl_type **func_tbl);

s32 cad_resource_dinit(void);

s32 cad_audio_dec_init(struct cad_func_tbl_type **func_tbl);

s32 cad_audio_dec_dinit(void);

s32 cad_audio_enc_init(struct cad_func_tbl_type **func_tbl);

s32 cad_audio_enc_dinit(void);

s32 cad_voice_dec_init(struct cad_func_tbl_type **func_tbl);

s32 cad_voice_dec_dinit(void);

s32 cad_voice_enc_init(struct cad_func_tbl_type **func_tbl);

s32 cad_voice_enc_dinit(void);

s32 cad_ard_init(struct cad_func_tbl_type **func_tbl);

s32 cad_ard_dinit(void);

s32 cad_dtmf_init(struct cad_func_tbl_type **func_tbl);

s32 cad_dtmf_dinit(void);

#endif

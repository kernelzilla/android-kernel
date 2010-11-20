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

#ifndef ARD_H
#define ARD_H

#include "msm8k_cad_module.h"

enum sample_rate_type {
	SAMPLE_RATE_NONE,
	SAMPLE_RATE_8000,
	SAMPLE_RATE_11025,
	SAMPLE_RATE_12000,
	SAMPLE_RATE_16000,
	SAMPLE_RATE_22050,
	SAMPLE_RATE_24000,
	SAMPLE_RATE_32000,
	SAMPLE_RATE_44100,
	SAMPLE_RATE_48000,
	SAMPLE_RATE_MAX
};


void print_data(u32 session_id);

s32 cad_ard_init(struct cad_func_tbl_type **func_ptr_tbl);

s32 ard_open(s32 session_id,
	struct cad_open_struct_type *open_param);

s32 ard_close(s32 session_id);

s32 ard_ioctl(s32 session_id, u32 cmd_code, void *cmd_buf, u32 cmd_len);

s32 ard_read(s32 session_id, struct cad_buf_struct_type *buf);

s32 ard_write(s32 session_id, struct cad_buf_struct_type *buf);


#endif

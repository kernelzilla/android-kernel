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

#ifndef _MSM8K_CAD_RPC_H_
#define _MSM8K_CAD_RPC_H_

#include <linux/kernel.h>

#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_cad_itypes.h>

#include <mach/qdsp6/msm8k_adsp_audio_event.h>
#include <mach/qdsp6/msm8k_adsp_audio_types.h>


typedef void (*RPC_CB_FCN)(union adsp_audio_event *return_event,
				void *client_data);

s32 cad_rpc_init(u32 processor_id);
s32 cad_rpc_deinit(void);
s32 cad_rpc_reg_callback(u32 stream_id, RPC_CB_FCN cbFCN, void *client_data);
s32 cad_rpc_dereg_callback(u32 stream_id, RPC_CB_FCN cbFCN);

s32 cad_rpc_data(u32 stream_id, u32 group_id, void *data_buf, u32 data_buf_len,
	union adsp_audio_event *ret_evt);

s32 cad_rpc_control(u32 stream_id, u32 group_id, void *cmd_buf,
	u32 cmd_buf_len, union adsp_audio_event *ret_evt);

#endif

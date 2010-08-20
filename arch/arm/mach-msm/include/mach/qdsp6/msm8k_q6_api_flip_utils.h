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

#ifndef QDSP6APIFLIPUTILS_H
#define QDSP6APIFLIPUTILS_H


#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_itypes.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_write_pcm_format.h>
#include <mach/qdsp6/msm8k_cad_write_aac_format.h>
#include <mach/qdsp6/msm8k_cad_write_midi_format.h>
#include <mach/qdsp6/msm8k_cad_adpcm_format.h>
#include <mach/qdsp6/msm8k_cad_amr_format.h>
#include <mach/qdsp6/msm8k_cad_qcelp13k_format.h>
#include <mach/qdsp6/msm8k_cad_evrc_format.h>
#include <mach/qdsp6/msm8k_cad_sbc_format.h>
#include <mach/qdsp6/msm8k_cad_wma_format.h>
#include <mach/qdsp6/msm8k_adsp_audio_command.h>


/* utility functions */
u32 q6_stream_context_mapping(enum cad_stream_app_enum_type app_type,
				u32 *mode);
u32 q6_open_op_mapping(u32 op_code);
u32 q6_device_id_mapping(u32 device);
u8 q6_device_direction_mapping(u8 device);

s32 convert_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_pcm_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_adpcm_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_aac_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_amr_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_dtmf_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_midi_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_v13k_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_evrc_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_mp3_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_wma_std_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_yadpcm_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
s32 convert_sbc_format_block(s32 session_id,
				struct adsp_audio_open_command *q6_open_struct,
				struct cadi_open_struct_type *cad_open_struct);
u32 q6_dtx_mode_mapping(u32 dtx_mode);
u32 q6_band_mode_mapping(u32 mode);

#endif




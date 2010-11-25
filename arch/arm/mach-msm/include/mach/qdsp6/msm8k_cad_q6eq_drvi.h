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

#ifndef _QDSP6EQDRVI_H_
#define _QDSP6EQDRVI_H_

#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_ardi.h>
#include <mach/qdsp6/msm8k_adsp_audio_device_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_stream_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_command.h>


#define CAD_EQ_INVALID_DATA       0xFFFFFFFF


struct cad_audio_eq_cfg {
	u32                             enable;
	/* Number of consequtive bands specified */
	u32                             num_bands;
	struct adsp_audio_eq_band       eq_bands[ADSP_AUDIO_MAX_EQ_BANDS];
} __attribute__ ((packed));


struct cad_filter_eq_driver_struct {
	/* this is the device control session */
	u32     device_session_id;
	/* indexed by session id */
	struct adsp_audio_set_equalizer_command	eq_stream_data[CAD_MAX_SESSION];
};


#endif

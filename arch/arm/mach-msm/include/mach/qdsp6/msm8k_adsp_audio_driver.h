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

#ifndef __ADSP_AUDIO_DRIVER_H
#define __ADSP_AUDIO_DRIVER_H


#include <mach/qdsp6/msm8k_adsp_audio_types.h>
#include <mach/qdsp6/msm8k_adsp_audio_error.h>
#include <mach/qdsp6/msm8k_adsp_audio_device.h>
#include <mach/qdsp6/msm8k_adsp_audio_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_media_format.h>
#include <mach/qdsp6/msm8k_adsp_audio_event.h>
#include <mach/qdsp6/msm8k_adsp_audio_cfg_ioctl.h>


/* Major version numbers track significant api changes that are */
/* not backwards compatible */
#define ADSP_AUDIO_API_MAJOR_VERSION	3

/* Minor version numbers of the same Major version are backward */
/* compatible and generally increment as features are added. */
#define ADSP_AUDIO_API_MINOR_VERSION	0

#define ADSP_AUDIO_DRIVER_SHARED_MEMORY_ACCESS_FLAG_READ	1
#define ADSP_AUDIO_DRIVER_SHARED_MEMORY_ACCESS_FLAG_WRITE	2


/* This function initializes the ADSP Audio Driver Framework. */
/* This function must be called only once at the start of the process */
/* that hosts this driver. */
/* @param[in] cb_data   Client data for asynchronous event notifications */
/* @return    status of the operation */
extern s32 adsp_audio_driver_init(struct adsp_audio_event_cb *cb_data);


/* This function De-initializes the ADSP Audio Driver Framework and releases */
/* all its resources */
/* @return    status of the operation */
extern s32 adsp_audio_driver_release(void);


/* Returns current version of the ADSPAudioDriver interface */
extern s32 adsp_audio_get_version(u32 *major, u32 *minor, char **build);


/* Returns driver status in string format */
extern s32 adsp_audio_get_status(struct adsp_audio_address *addr,
					u8 *status, u32 *len);


/* Command interface to ADSP Audio Driver (control path) */
extern s32 adsp_audio_control(union adsp_audio_command *cmd, u32 len);


/* Send data buffers to/from an audio stream (data path) */
extern s32 adsp_audio_data(union adsp_audio_command *cmd, u32 len);


#endif




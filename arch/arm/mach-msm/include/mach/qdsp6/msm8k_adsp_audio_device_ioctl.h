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

#ifndef __ADSP_AUDIO_DEVICE_IOCTL_H
#define __ADSP_AUDIO_DEVICE_IOCTL_H


#include <mach/qdsp6/msm8k_adsp_audio_stream_ioctl.h>


/* Device control session only IOCTL command definitions */
/* These commands will affect a logical device and all its associated */
/* streams. */


/* Set device volume. */
/* This command has data payload struct adsp_audio_set_dev_volume_command. */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL		0x0107605c


/* Set Device stereo volume. This command has data payload, */
/* struct adsp_audio_set_dev_stereo_volume_command. */

#define ADSP_AUDIO_IOCTL_SET_DEVICE_STEREO_VOL		0x0108df3e


/* Set L, R cross channel gain for a Device. This command has */
/* data payload, struct adsp_audio_set_dev_x_chan_gain_command. */

#define ADSP_AUDIO_IOCTL_SET_DEVICE_XCHAN_GAIN		0x0108df40


/* Set device mute state. */
/* This command has data payload struct adsp_audio_set_dev_mute_command. */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE		0x0107605f


/* Set device RVE state. */
/* This command has data payload struct adsp_audio_set_dev_rve_command. */

#define ADSP_AUDIO_IOCTL_SET_DEVICE_RVE			0x01090832


/* Set device WNR state. */
/* This command has data payload struct adsp_audio_set_dev_wnr_command. */

#define ADSP_AUDIO_IOCTL_SET_DEVICE_WNR			0x01090831


/* Configure Equalizer for a device. */
/* This command has payload struct adsp_audio_set_dev_equalizer_command. */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_EQ_CONFIG	0x0108b10e



#endif



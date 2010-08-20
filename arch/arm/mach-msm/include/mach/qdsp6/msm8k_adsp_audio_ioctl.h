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

#ifndef __ADSP_AUDIO_IOCTL_H
#define __ADSP_AUDIO_IOCTL_H


#include <mach/qdsp6/msm8k_adsp_audio_device_ioctl.h>


/* Control and Stream session IOCTL command definitions */


/* Opcode to open a device stream session to capture audio */
#define ADSP_AUDIO_IOCTL_CMD_OPEN_READ			0x0108dd79


/* Opcode to open a device stream session to render audio */
#define ADSP_AUDIO_IOCTL_CMD_OPEN_WRITE			0x0108dd7a


/* Opcode to open a device session, must open a device */
#define ADSP_AUDIO_IOCTL_CMD_OPEN_DEVICE		0x0108dd7b


/* Close an existing stream or device */
#define ADSP_AUDIO_IOCTL_CMD_CLOSE			0x0108d8bc



/* A device switch requires three IOCTL */
/* commands in the following sequence: */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT */

/* adsp_audio_device_switch_command structure is needed for */
/* DEVICE_SWITCH_PREPARE */

/* Device switch protocol step #1. Pause old device and */
/* generate silence for the old device. */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE	0x010815c4


/* Device switch protocol step #2. Release old device, */
/* create new device and generate silence for the new device. */

/* When client receives ack for this IOCTL, the client can */
/* start sending IOCTL commands to configure, calibrate and */
/* change filter settings on the new device. */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY	0x010815c5


/* Device switch protocol step #3. Start normal operations on new device */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT	0x01075ee7


/* Set Commands */


/* Generic SET command opCode used to set parameters on Audio */
/* Stream and Device Controls. This command has payload */
/* struct adsp_audio_set_command. */
#define ADSP_AUDIO_IOCTL_SET			0x010909f3


/* Generic SET command opCode used to set parameters on Audio */
/* Stream and Device Controls. This command has payload */
/* struct adsp_audio_set_from_memory_command. */
#define ADSP_AUDIO_IOCTL_SET_FROM_MEMORY	0x010909f4


/* Control ID used in Set commands */



/* CODEC Control is the primary Decoder or Encoder in the Audio graph */
#define ADSP_AUDIO_CODEC_CONTROL		0x010909f5
/* GAIN Control is the primary Volume/Gain control in the Audio graph */
#define ADSP_AUDIO_GAIN_CONTROL			0x010909f6
/* EQ Control is the component providing Equalizer settings */
#define ADSP_AUDIO_EQ_CONTROL			0x010909f7


/* Parameter ID used in Set commands */



#define ADSP_AUDIO_PARAM_DLS					0x0108dd78
#define ADSP_AUDIO_PARAM_DUAL_MONO_MAPPING			0x0108d39a
#define ADSP_AUDIO_PARAM_AAC_ADD_DROP_SAMPLE			0x0108d412
#define ADSP_AUDIO_PARAM_AAC_SBR_PS				0x01052310
#define ADSP_AUDIO_PARAM_WMAPRO_CHEX_FEX			0x0108d680
/* Karoke Mode for Ac3 Decoder Expected values 0-3 (default 3) */
#define ADSP_AUDIO_PARAM_AC3_DEC_KAROKE_MODE			0x01090a05
/* DRC Mode for Ac3 Decoder Expected Values 0-3 (default 2) */
#define ADSP_AUDIO_PARAM_AC3_DEC_DRC_MODE			0x01090a06
/* LFE flag 0/1 (default 1) */
#define ADSP_AUDIO_PARAM_AC3_DEC_LFE_ON_FLAG			0x01090a07
/* Channel Configuration Expected Value 1-7 (default 7) */
#define ADSP_AUDIO_PARAM_AC3_DEC_OUTPUT_CHANNEL_CONFIGURATION	0x01090a08
/* Number of Output Channels Expected Values 1-6 (default 6) */
#define ADSP_AUDIO_PARAM_AC3_DEC_NUMBER_OF_CHANNELS		0x01090a0a
/* PCM scale factor Expected value 0-0x7FFFFFFF (default 0x7FFFFFFF) */
#define ADSP_AUDIO_PARAM_AC3_DEC_PCM_SCALE_FACTOR		0x01090a0b

/* Set Stereo Mode and Dual Mono Mode. */
/* First word is for Stereo Mode Expected Values 0-2. (default 0) */
/* Stereo Mode is valid only when OUTPUT_CHANNEL_CONFIGURATION is 2 */
/* Second word is for Dual Mono Expected Values 0-3 (default 0) */
#define ADSP_AUDIO_PARAM_AC3_DEC_STEREO_MODE_DUAL_MONO_MODE	0x01090a0c

/* Set DRC Range Scale Hi or Scale Low. */
/* First 32 bit word is for Scale Hi, Expected Value */
/* 0-0x7FFFFFFF (default 0x7FFFFFFF) */
/* Second 32 bit word is for Scale Low, Expected Value */
/* 0-0x7FFFFFFF (default 0x7FFFFFFF) */
#define ADSP_AUDIO_PARAM_AC3_DEC_DRC_RANGE_SCALE_HI_SCALE_LOW	0x01090a0f

/* Channel Routing, Route arbitrary input channels (L, C, R, l, r, s) to */
/* arbitrary interleaved output channel (0..5) */
/* Default: 0L 1C 2R 3l 4r 5s */
/* Lower 24 bits of 32 bit word are used four channel Routing */
/* Following is the nible sequence for channels */
/* 0x00,s(LFE), r(Right Surr), l(Left Suff), R(Right), C(Center), L(Left) */
/* default value will be 0x00543210 */
#define ADSP_AUDIO_PARAM_AC3_DEC_CHANNEL_ROUTING		0x01090a11

/* It Sets UserId and Key */
/* First 16 bytes are for User Id */
/* Next 4 32 bit words are for 128 bit Key */
#define ADSP_AUDIO_PARAM_AC3_DEC_USER_ID_USER_KEY		0x01090a12

#endif



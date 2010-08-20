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

#ifndef __ADSP_AUDIO_STREAM_IOCTL_H
#define __ADSP_AUDIO_STREAM_IOCTL_H


#include <mach/qdsp6/msm8k_adsp_audio_types.h>
#include <mach/qdsp6/msm8k_adsp_audio_device.h>



/* DATA IOCTLs */
/* Command IOCTLs to Send/Receive data from an audio stream */



/* Transmit (send) Data to audio stream (playback) */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_DATA_TX			0x0108dd7f


/* Receive Data from audio stream (record). */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_DATA_RX			0x0108dd80




/* Stream only IOCTL command definitions. */
/* These commands will affect only a single stream. */



/* Stop stream for audio device. */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_STOP		0x01075c54


/* End of stream reached. Client will not send any more data. */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_EOS			0x0108b150


/* Do sample slipping/stuffing on AAC outputs. The payload of */
/* this command is struct adsp_audio_slip_sample_command. */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_SLIPSAMPLE		0x0108d40e


/* Set stream volume. */
/* This command has data payload, struct adsp_audio_set_volume_command. */

#define ADSP_AUDIO_IOCTL_CMD_SET_STREAM_VOL		0x0108c0de


/* Set stream stereo volume. This command has data payload, */
/* struct adsp_audio_set_stereo_volume_command. */
#define ADSP_AUDIO_IOCTL_SET_STREAM_STEREO_VOL		0x0108dd7c


/* Set L, R cross channel gain for a Stream. This command has */
/* data payload, struct adsp_audio_set_x_chan_gain_command. */
#define ADSP_AUDIO_IOCTL_SET_STREAM_XCHAN_GAIN		0x0108dd7d


/* Set stream mute state. */
/* This command has data payload, struct adsp_audio_set_stream_mute. */

#define ADSP_AUDIO_IOCTL_CMD_SET_STREAM_MUTE		0x0108c0df


/* Reconfigure bit rate information. This command has data */
/* payload, struct adsp_audio_set_bit_rate_command */
#define ADSP_AUDIO_IOCTL_SET_STREAM_BITRATE		0x0108ccf1


/* Set Channel Mapping. This command has data payload, struct */
/* This command has data payload struct adsp_audio_set_channel_map_command. */
#define ADSP_AUDIO_IOCTL_SET_STREAM_CHANNELMAP		0x0108d32a


/* Enable/disable AACPlus SBR. */
/* This command has data payload struct adsp_audio_set_sbr_command */
#define ADSP_AUDIO_IOCTL_SET_STREAM_SBR			0x0108d416


/* Enable/disable WMA Pro Chex and Fex. This command has data payload */
/* struct adsp_audio_set_amrwb_plus_command. */
#define ADSP_AUDIO_IOCTL_SET_STREAM_WMAPRO		0x0108d417


/* Configure AMR-WB+ decoder's sampling rate. This command has data payload */
/* struct adsp_audio_set_wma_command. */
#define ADSP_AUDIO_IOCTL_SET_STREAM_AMRWBPLUS_SAMPLE_RATE	0x0108f3dd


/* Configure AMR-WB+ decoder to do mono decoding even for stereo content.*/
/* This command has data payload struct adsp_audio_set_wma_command. */
#define ADSP_AUDIO_IOCTL_SET_STREAM_AMRWBPLUS_MONO_DEC_STEREO	0x0108f3dc


/* Configure AMR-WB+ decoder to do mono decoding even for stereo content */
/* This command has data payload struct adsp_audio_set_wma_command */
#define ADSP_AUDIO_IOCTL_SET_STREAM_AMRWBPLUS_LIMITER	0x0108f9a3


/* SESSION CMD IOCTLs */
/* These commands affect a group of streams identified by the major address */



/* Start stream for audio device. */
/* This command has no payload, use struct adsp_audio_no_payload_command.  */

#define ADSP_AUDIO_IOCTL_CMD_SESSION_START		0x010815c6


/* Stop all stream(s) for audio session as indicated by major id. */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_SESSION_STOP		0x0108dd7e


/* Pause the data flow for a session as indicated by major id. */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_SESSION_PAUSE		0x01075ee8


/* Resume the data flow for a session as indicated by major id. */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_SESSION_RESUME		0x01075ee9


/* Drop any unprocessed data buffers for a session as indicated by major id. */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_SESSION_FLUSH		0x01075eea


/* Start Stream DTMF tone */
/* This command has payload struct adsp_audio_dtmf_start_command */

#define ADSP_AUDIO_IOCTL_CMD_SESSION_DTMF_START		0x0108c0dd


/* Stop Stream DTMF tone */
/* This command has no payload, use struct adsp_audio_no_payload_command. */

#define ADSP_AUDIO_IOCTL_CMD_SESSION_DTMF_STOP		0x01087554


/* Get Audio Media Session time. */
/* This command has no payload, use struct adsp_audio_no_payload_command. */
/* This command returns the audioTime in adsp_audio_unsigned64_event */

#define ADSP_AUDIO_IOCTL_CMD_GET_AUDIO_TIME		0x0108c26c


/* SESSION SET IOCTLs */
/* These commands affect a group of streams identified by the major address */



/* Set Session volume. */
/* This command has data payload, struct adsp_audio_set_volume_command. */

#define ADSP_AUDIO_IOCTL_SET_SESSION_VOL		0x0108d8bd


/* Set session stereo volume. This command has data payload, */
/* struct adsp_audio_set_stereo_volume_command. */

#define ADSP_AUDIO_IOCTL_SET_SESSION_STEREO_VOL		0x0108df3d


/* Set L, R cross channel gain for a session. This command has */
/* data payload, struct adsp_audio_set_x_chan_gain_command. */
#define ADSP_AUDIO_IOCTL_SET_SESSION_XCHAN_GAIN		0x0108df3f


/* Set Session mute state. */
/* This command has data payload, struct adsp_audio_set_mute_command. */
#define ADSP_AUDIO_IOCTL_SET_SESSION_MUTE		0x0108d8be


/* Configure Equalizer for a stream. */
/* This command has payload struct adsp_audio_set_equalizer_command. */

#define ADSP_AUDIO_IOCTL_SET_SESSION_EQ_CONFIG		0x0108c0e0


/* Set Audio Video sync information. */
/* This command has data payload, struct adsp_audio_set_av_sync_command. */

#define ADSP_AUDIO_IOCTL_SET_SESSION_AVSYNC		0x0108d1e2



#endif



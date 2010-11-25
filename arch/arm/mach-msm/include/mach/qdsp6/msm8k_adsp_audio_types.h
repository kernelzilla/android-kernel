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

#ifndef __ADSP_AUDIO_TYPES_H
#define __ADSP_AUDIO_TYPES_H

#include <mach/qdsp6/msm8k_adsp_audio_media_format.h>


/* only 3 for now - AMR9, AMR11, DSP */
#define ADSP_AUDIO_MAX_DOMAIN	3
/* only 2 for now: Audio, Video */
#define ADSP_AUDIO_MAX_SERVICE	2
/* same as minor (one major per minor) */
#define ADSP_AUDIO_MAX_MAJOR	64
/* max possible audio streams (including device streams) */
#define ADSP_AUDIO_MAX_MINOR	64


/* Valid Domain locations */
#define ADSP_AUDIO_ADDRESS_DOMAIN_APP   0
#define ADSP_AUDIO_ADDRESS_DOMAIN_MODEM 1
#define ADSP_AUDIO_ADDRESS_DOMAIN_DSP   2


/* Valid services */
#define ADSP_AUDIO_ADDRESS_SERVICE_AUDIO	0
#define ADSP_AUDIO_ADDRESS_SERVICE_VIDEO	1


struct adsp_audio_address {
	/* processor (DSP, AMR9,  ARM11, required for CAD routing) */
	u8	domain;
	/* unused (set to zero) - reserved for future use */
	u8	service;
	/* major id, maps to MediaSession, DeviceSession */
	u8	major;
	/* minor id, maps to audio stream (not used for device) */
	u8	minor;
} __attribute__ ((packed));


/* Command/event response types */
#define ADSP_AUDIO_RESPONSE_COMMAND   0
#define ADSP_AUDIO_RESPONSE_ASYNC     1


/* Defines a command id and type used to route the command request */
/* and its event response */
/* Treated as read-only data by the DSP */
struct adsp_audio_command_data {
	/* Command opcode, determines type of payload */
	u32	op_code;
	/* command type, determines response type (used to route the command) */
	/* ADSP_AUDIO_RESPONSE_COMMAND | ADSP_AUDIO_RESPONSE_ASYNC */
	u32	response_type;
	/* sequence number (internal use only) */
	u32	seq_number;
} __attribute__ ((packed));


/* Defines client data that is returned in the event response for */
/* each command sent */
/* Treated as read-only data by the DSP */
struct adsp_audio_client_data {
	u32	context;	/* Clients Context */
	u32	data;		/* Associated data */
} __attribute__ ((packed));


/* Defines an event id and type used to route the command request */
/* and its event response */
/* Set by DSP, read-only data from client */
struct adsp_audio_event_data {
	/* event id, determines type of event payload */
	u32	id;
	/* event type, determines response type (used to route the event) */
	/* ADSP_AUDIO_RESPONSE_COMMAND | ADSP_AUDIO_RESPONSE_ASYNC */
	u32	response_type;
	/* sequence number (internal use only) */
	u32	seq_number;
} __attribute__ ((packed));


/* Some encoders need configuration information in addition to format */
/* block */

/* AAC Encoder modes */
#define ADSP_AUDIO_ENC_AAC_LC_ONLY_MODE		0
#define ADSP_AUDIO_ENC_AAC_PLUS_MODE		1
#define ADSP_AUDIO_ENC_ENHANCED_AAC_PLUS_MODE	2

/* AAC Encoder configuration */

struct adsp_audio_aac_enc_cfg {
	u32	bit_rate;	/* bits per second */
	u32	encoder_mode;	/* ADSP_AUDIO_ENC_* */
} __attribute__ ((packed));


#define ADSP_AUDIO_ENC_SBC_ALLOCATION_METHOD_LOUNDNESS     0
#define ADSP_AUDIO_ENC_SBC_ALLOCATION_METHOD_SNR           1


#define ADSP_AUDIO_ENC_SBC_CHANNEL_MODE_MONO                1
#define ADSP_AUDIO_ENC_SBC_CHANNEL_MODE_STEREO              2
#define ADSP_AUDIO_ENC_SBC_CHANNEL_MODE_DUAL                8
#define ADSP_AUDIO_ENC_SBC_CHANNEL_MODE_JOINT_STEREO        9


struct adsp_audio_sbc_encoder_cfg {
	u32	num_subbands;
	u32	block_len;
	u32	channel_mode;
	u32	allocation_method;
	u32	bit_rate;
} __attribute__ ((packed));


/* AMR NB encoder modes */
#define ADSP_AUDIO_AMR_MR475	0
#define ADSP_AUDIO_AMR_MR515	1
#define ADSP_AUDIO_AMR_MMR59	2
#define ADSP_AUDIO_AMR_MMR67	3
#define ADSP_AUDIO_AMR_MMR74	4
#define ADSP_AUDIO_AMR_MMR795	5
#define ADSP_AUDIO_AMR_MMR102	6
#define ADSP_AUDIO_AMR_MMR122	7


/* The following are valid AMR NB DTX modes */
#define ADSP_AUDIO_AMR_DTX_MODE_OFF		0
#define ADSP_AUDIO_AMR_DTX_MODE_ON_VAD1		1
#define ADSP_AUDIO_AMR_DTX_MODE_ON_VAD2		2
#define ADSP_AUDIO_AMR_DTX_MODE_ON_AUTO		3


/* AMR Encoder configuration */
struct adsp_audio_amr_enc_cfg {
	u32	mode;		/* ADSP_AUDIO_AMR_MR* */
	u32	dtx_mode;	/* ADSP_AUDIO_AMR_DTX_MODE* */
	u32	enable;		/* 1 = enable, 0 = disable */
} __attribute__ ((packed));


struct adsp_audio_qcelp13k_enc_cfg {
	u16	min_rate;
	u16	max_rate;
} __attribute__ ((packed));


struct adsp_audio_evrc_enc_cfg {
	u16	min_rate;
	u16	max_rate;
} __attribute__ ((packed));


union adsp_audio_codec_config {
	struct adsp_audio_amr_enc_cfg		amr_cfg;
	struct adsp_audio_aac_enc_cfg		aac_cfg;
	struct adsp_audio_qcelp13k_enc_cfg	qcelp13k_cfg;
	struct adsp_audio_evrc_enc_cfg		evrc_cfg;
	struct adsp_audio_sbc_encoder_cfg	sbc_cfg;
} __attribute__ ((packed));


/* Bit masks for adsp_audio_open_payload.mode */



/* This is the default value. */
#define  ADSP_AUDIO_OPEN_STREAM_MODE_NONE		0x0000

/* This bit, if set, indicates that the AVSync mode is activated. */
#define  ADSP_AUDIO_OPEN_STREAM_MODE_AVSYNC		0x0001

/* This bit, if set, indicates that the Sample Rate/Channel Mode */
/* Change Notification mode is activated. */
#define  ADSP_AUDIO_OPEN_STREAM_MODE_SR_CM_NOTIFY	0x0002

/* This bit, if set, indicates that the sync clock is enabled */
#define  ADSP_AUDIO_OPEN_STREAM_MODE_ENABLE_SYNC_CLOCK	0x0004



/* The client specifies the media format block as defined in  */
/* ADSPaudio_MediaFormat.h for OPEN_OP_READ/WRITE */


#define ADSP_AUDIO_MAX_DEVICES 1

struct adsp_audio_open_payload {

	/* stream device ID */
	u32				device;
	/* data end point (HostPCM) */
	struct adsp_audio_address	end_point;
	/* Stream usage type */
	u32				stream_context;
	/* 1- indicates AVSync playback mode */
	u32				mode;
	/* Media Format buffer size in bytes */
	u32				buf_max_size;
	/* Media Format Block */
	union adsp_audio_format		format_block;
	/* Encoder configuration for READ op */
	union adsp_audio_codec_config	config;
} __attribute__ ((packed));


/* This flag, if set, indicates that the beginning of the data in the*/
/* buffer is a synchronization point or key frame, meaning no data */
/* before it in the stream is required in order to render the stream */
/* from this point onward. */
#define ADSP_AUDIO_BUFFER_FLAG_SYNC_POINT        0x01


/* This flag, if set, indicates that the buffer object is using valid */
/* physical address used to store the media data */
#define ADSP_AUDIO_BUFFER_FLAG_PHYS_ADDR         0x04


/* This flag, if set, indicates that a media start timestamp has been */
/* set for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_START_SET         0x08


/* This flag, if set, indicates that a media stop timestamp has been set */
/* for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_STOP_SET          0x10


/* This flag, if set, indicates that a preroll timestamp has been set */
/* for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_PREROLL_SET       0x20


/* This flag, if set, indicates that the data in the buffer is a fragment of */
/* a larger block of data, and will be continued by the data in the next */
/* buffer to be delivered. */
#define ADSP_AUDIO_BUFFER_FLAG_CONTINUATION      0x40


struct adsp_audio_data_buffer {
	u32	buffer_addr;	/* Physical Address of buffer */
	u32	max_size;	/* Maximum size of buffer */
	u32	actual_size;	/* Actual size of valid data in the buffer */
	u32	offset;		/* Offset to the first valid byte */
	u32	flags;		/* ADSP_AUDIO_BUFFER_FLAGs that has been set */
	s64	start;		/* Start timestamp, if any */
	s64	stop;		/* Stop timestamp, if any */
	s64	preroll;	/* Preroll timestamp, if any */
} __attribute__ ((packed));



#define ADSP_AUDIO_MAX_EQ_BANDS 12

/* Equalizer filter band types */
#define ADSP_AUDIO_EQUALIZER_TYPE_NONE		0
#define ADSP_AUDIO_EQUALIZER_BASS_BOOST		1
#define ADSP_AUDIO_EQUALIZER_BASS_CUT		2
#define ADSP_AUDIO_EQUALIZER_TREBLE_BOOST	3
#define ADSP_AUDIO_EQUALIZER_TREBLE_CUT		4
#define ADSP_AUDIO_EQUALIZER_BAND_BOOST		5
#define ADSP_AUDIO_EQUALIZER_BAND_CUT		6


/* Definition for any one band of Equalizer. */

struct adsp_audio_eq_band {
	/* The band index, 0 .. 11 */
	u16	band_idx;
	/* Filter band type */
	u32	filter_type;
	/* Filter band center frequency */
	u32	center_freq_hz;
	/* Filter band initial gain (dB) */
	/* Range is +12 dB to -12 dB with 1dB increments. */
	s32	filter_gain;
	/* Filter band quality factor expressed as q-8 number, */
	/* i.e. fixed point number with q factor of 8, */
	/* e.g. 3000/(2^8) */
	s32	q_factor;
} __attribute__ ((packed));



/* Data that defines a physical memory address into shared memory */
struct adsp_phys_mem_type {
	u32	addr;	/* physical address */
	u32	total;	/* Length of allocated memory in bytes */
	u32	used;	/* Actual Length of buffer in bytes */
} __attribute__ ((packed));


union adsp_audio_event;

/* Callback function type for clients to recieve events */
typedef void (*adsp_audio_event_cb_func)(union adsp_audio_event *event,
						void *client_data);


struct adsp_audio_event_cb {
	adsp_audio_event_cb_func	callback;
	void				*client_data;
};


#endif

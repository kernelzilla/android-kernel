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

#ifndef __ADSP_AUDIO_COMMAND_H
#define __ADSP_AUDIO_COMMAND_H


#include <mach/qdsp6/msm8k_adsp_audio_types.h>
#include <mach/qdsp6/msm8k_adsp_audio_event.h>


/* A generic command structure, commands are cast to this structure */
/* to determine the command and then casted into the appropriate */
/* command type  */
struct adsp_audio_no_payload_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;


	/* no payload for this command */
} __attribute__ ((packed));


/* Command structure for OPEN operations */
/* Open command structure, used to open a new stream */
struct adsp_audio_open_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Open for READ/WRITE */
	struct adsp_audio_open_payload		open_data;
} __attribute__ ((packed));


/* Command structure for DATA (Read/Write) operations */
/* Data command structure, used for Read/Write operations on audio stream. */
/* Should always be sent on data path via adsp_audio_data function */
struct adsp_audio_data_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Media data buffer */
	struct adsp_audio_data_buffer		buffer;
} __attribute__ ((packed));


/* Maximum number of bytes allowed in adsp_audio_set_command */
/* 5 u32's */
#define ADSP_AUDIO_SET_CMD_MAX_BYTES	(5 * 4)

/* Generic command structure for setting configuration data */
struct adsp_audio_set_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* unique id of contorl to be set */
	u32					control_id;
	/* unique id of parameter to be set */
	u32					param_id;
	/* number of bytes to be set */
	u32					len;
	/* data to be set */
	u8					data
						[ADSP_AUDIO_SET_CMD_MAX_BYTES];
} __attribute__ ((packed));


/* Generic command structure for setting configuration data */
struct adsp_audio_set_from_memory_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* unique id of contorl to be set */
	u32					control_id;
	/* unique id of parameter to be set */
	u32					param_id;
	/* number of bytes to be set */
	u32					len;
	/* physical address of data to be set */
	u32					address;
} __attribute__ ((packed));


/* Command struct used to set algorithm aspect of device */
/* Used to set configuration data for an algorithm aspect of a device */
struct adsp_audio_set_dev_cfg_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* Algorithm block ID */
	u32					block_id;
	/* Configurable element ID of a Media Module */
	u32					interface_id;
	/* Physical memory */
	struct adsp_phys_mem_type		phys_mem;
} __attribute__ ((packed));


/* Command struct used to set device configuration table */
/* Used to set configuration data for all interfaces of a device. */
struct adsp_audio_set_dev_cfg_table_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* Physical memory */
	struct adsp_phys_mem_type		phys_mem;
} __attribute__ ((packed));


/* Command struct used to get device values for any device */
struct adsp_audio_set_dev_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32					path;
} __attribute__ ((packed));


struct adsp_audio_set_dev_volume_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32					path;
	/* device volume in mB */
	s32					volume;
} __attribute__ ((packed));


struct adsp_audio_set_dev_stereo_volume_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32					path;
	/* Left Channel gain in mB */
	s32					l_chan_gain_mb;
	/* Right Channel gain in mB */
	s32					r_chan_gain_mb;
} __attribute__ ((packed));


/* Command struct used to set L, R cross channel gain for a Device */
struct adsp_audio_set_dev_x_chan_gain_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32					path;
	/* Gain in mB applied to left channel */
	s32					l_chan_src_mb;
	/* Gain in mB applied to right channel, */
	/* when added to the left channel */
	s32					l_chan_r_mb;
	/* Gain in mB applied to right channel */
	s32					r_chan_src_mb;
	/* Gain in mB applied to left channel, */
	/* when added to the right channel */
	s32					r_chan_l_mb;
} __attribute__ ((packed));


struct adsp_audio_set_dev_mute_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32					path;
	/* 0 == UnMute, 1 == Mute */
	u32					mute;
} __attribute__ ((packed));


/* Set device RVE (Received Voice Enhancement) state */
struct adsp_audio_set_dev_rve_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32					path;
	/* 0 == RVE disabled, 1 == RVE enabled */
	u32					rve;
} __attribute__ ((packed));


/* Set device WNR (Wind Noise Reduction) state */
struct adsp_audio_set_dev_wnr_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* ADSP Device ID */
	u32					device_id;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u32					path;
	/* 0 == WNR disabled, 1 == WNR enabled */
	u32					wnr;
} __attribute__ ((packed));


struct adsp_audio_set_dev_equalizer_command {
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* command data, used for processing */
	struct adsp_audio_command_data	cmd;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;

	/* payload */
	/* ADSP Device ID */
	u32				device_id;
	/* 0 == Off, 1 == On */
	u32				enable;
	/* Number of consequtive bands specified */
	u32				num_bands;
	struct adsp_audio_eq_band	eq_bands[ADSP_AUDIO_MAX_EQ_BANDS];
} __attribute__ ((packed));


/* A device switch requires three IOCTL commands in the following sequence: */
/*	ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE */
/*	ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY */
/*	ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT */

/* The structure below is only needed for DEVICE_SWITCH_PREPARE */
/* other IOCTLS do not require a payload and can use */
/* adsp_audio_no_payload_command */
struct adsp_audio_device_switch_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* DeviceID to switch from */
	u32					old_device;
	/* DeviceID to switch to */
	u32					new_device;
	/*  Device Class is defined as: */
	/*	DMA 0 ~V Int RX */
	/*	DMA 1 ~V Int TX */
	/*	DMA 2 ~V Ext RX */
	/*	DMA 3 ~V Ext TX */
	u8					device_class;
	/* 0 == Rx, 1 == Tx and 2 == both */
	u8					device_type;
} __attribute__ ((packed));


struct adsp_audio_dtmf_start_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* First tone in Hz */
	u32					tone1_hz;
	/* Second tone in Hz */
	u32					tone2_hz;
	/* Duration in microseconds */
	u32					duration_usec;
	/* Gain in mB */
	s32					gain_mb;
} __attribute__ ((packed));


struct adsp_audio_set_volume_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* in mB */
	s32					volume;
} __attribute__ ((packed));


struct adsp_audio_set_stereo_volume_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Left Channel gain in mB */
	s32					l_chan_gain_mb;
	/* Right Channel gain in mB */
	s32					r_chan_gain_mb;
} __attribute__ ((packed));


/* Command struct used to set L, R cross channel gain */
struct adsp_audio_set_x_chan_gain_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Gain in mB applied to left channel */
	s32					l_chan_src_mb;
	/* Gain in mB applied to right channel, */
	/* when added to the left channel */
	s32					l_chan_r_mb;
	/* Gain in mB applied to right channel */
	s32					r_chan_src_mb;
	/* Gain in mB applied to left channel, */
	/* when added to the right channel */
	s32					r_chan_l_mb;
} __attribute__ ((packed));


struct adsp_audio_set_mute_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* 0 == UnMute, 1 == Mute */
	u32					mute;
} __attribute__ ((packed));


struct adsp_audio_set_equalizer_command {
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* command data, used for processing */
	struct adsp_audio_command_data	cmd;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;

	/* payload */
	/* 0 == Off, 1 == On */
	u32				enable;
	/* Number of consequtive bands specified */
	u32				num_bands;
	struct adsp_audio_eq_band	eq_bands[ADSP_AUDIO_MAX_EQ_BANDS];
} __attribute__ ((packed));


struct adsp_audio_set_av_sync_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Media time */
	s64					relative_time;
	/* Presentation time */
	s64					absolute_time;
} __attribute__ ((packed));


/* Command struct to set codec bitrate */
struct adsp_audio_set_bit_rate_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Required BitRate */
	u32					bit_rate;
} __attribute__ ((packed));


struct adsp_audio_set_channel_map_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Channel Mapping controls the dual-mono mapping method: */
	/* 0: First SCE (Single Channel Element) to left channel and */
	/*    Second SCE to right channel. */
	/* 1: First SCE to right channel and Second SCE to left channel. */
	/* 2: First SCE to both left and right channels. */
	/* 3: Second SCE to both left and right channels. */
	s32					chan_map;
} __attribute__ ((packed));


/* Command struct to signal the codec to add/drop a sample */

/* Do sample slipping/stuffing on AAC outputs. */
/*	If num_samples > 0, one sample will be added (stuffing) */
/*	If num_samples < 0, one sample will be dropped (slipping) */
/*	If num_samples = 0, normal playback, no need to send this cmd */

struct adsp_audio_slip_sample_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* number of samples to add/drop */
	s32					num_samples;
} __attribute__ ((packed));


/* Command struct to enable/disable AACPlus SBR and PS */

/* sbr_ps_flag - bit 0: SBR (value 0: disable, value 1: enable) */
/*	       - bit 1: PS  (value 0: disable, value 1: enable) */

/* Note: it is illegal to have sbr_ps_flag == 2, i.e., SBR. */
/* disabled and PS enabled, because PS is a superset of SBR, */
/* enabling PS must enable SBR as well. */

struct adsp_audio_set_sbr_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Enable/Disable Flag */
	u32					sbr_ps_flag;
} __attribute__ ((packed));


/* Command struct to Enable/disable WMA Pro Chex and Fex */
/* chex_fex_flag takes the following values */
#define ADSP_AUDIO_WMAPRO_CHEX_OFF_FEX_OFF	0
#define ADSP_AUDIO_WMAPRO_CHEX_OFF_FEX_ON	1
#define ADSP_AUDIO_WMAPRO_CHEX_ON_FEX_OFF	2
#define ADSP_AUDIO_WMAPRO_CHEX_ON_FEX_ON	3

struct adsp_audio_set_wma_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Enable/Disable Flag */
	u32					chex_fex_flag;
} __attribute__ ((packed));


struct adsp_audio_set_amrwb_plus_command {
	/* destination address, used in routing */
	struct adsp_audio_address		dest;
	/* source address, used in routing */
	struct adsp_audio_address		source;
	/* command data, used for processing */
	struct adsp_audio_command_data		cmd;
	/* read-only client data */
	struct adsp_audio_client_data		client_data;
	/* pad header to 64byte aligned/also match event status */
	u32					padding;

	/* payload */
	/* Number of Channels */
	u32					param;
} __attribute__ ((packed));


/* Union of all command types */
union adsp_audio_command {
	/* Use for any command with no payload */
	struct adsp_audio_no_payload_command		no_payload;

	/* Open command structure */
	struct adsp_audio_open_command			open;

	/* Data command structure, used for read/write data buffers */
	struct adsp_audio_data_command			data;

	/* Generic Set Command - for inband data */
	struct adsp_audio_set_command			set;

	/* Generic Set Command - for out-of-band data */
	struct adsp_audio_set_from_memory_command	set_from_memory;

	/* device commands */
	struct adsp_audio_set_dev_cfg_command		set_dev_cfg;
	struct adsp_audio_set_dev_cfg_table_command	set_dev_cfg_table;
	struct adsp_audio_set_dev_command		set_dev;
	struct adsp_audio_set_dev_volume_command	set_dev_volume;
	struct adsp_audio_set_dev_stereo_volume_command	set_dev_stereo_volume;
	struct adsp_audio_set_dev_x_chan_gain_command	set_dev_x_chan_gain;
	struct adsp_audio_set_dev_mute_command		set_dev_mute;
	struct adsp_audio_set_dev_rve_command		set_dev_rve;
	struct adsp_audio_set_dev_wnr_command		set_dev_wnr;
	struct adsp_audio_set_dev_equalizer_command	set_dev_equalizer;

	/* device switch */
	struct adsp_audio_device_switch_command		device_switch;

	/* session/stream commands */
	struct adsp_audio_dtmf_start_command		dtmf_start;
	struct adsp_audio_set_volume_command		set_volume;
	struct adsp_audio_set_stereo_volume_command	set_stereo_volume;
	struct adsp_audio_set_x_chan_gain_command	set_x_chan_gain;
	struct adsp_audio_set_mute_command		set_mute;
	struct adsp_audio_set_equalizer_command		set_equalizer;
	struct adsp_audio_set_av_sync_command		set_av_sync;
	struct adsp_audio_set_bit_rate_command		set_bit_rate;
	struct adsp_audio_set_channel_map_command	set_channel_map;
	struct adsp_audio_slip_sample_command		slip_sample;
	struct adsp_audio_set_sbr_command		set_sbr;
	struct adsp_audio_set_wma_command		set_wma;
	struct adsp_audio_set_amrwb_plus_command	set_amrwb_plus;

	/* Provide space in our command union to hold any audio event */
	/* this way we can use the same memory allocated for the command to */
	/* send an event as well as cast a command to an event and back */
	/* Not intended for direct access */
	union adsp_audio_event				dummy;

};


#endif



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

#ifndef __ADSP_AUDIO_EVENT_H
#define __ADSP_AUDIO_EVENT_H


#include <mach/qdsp6/msm8k_adsp_audio_types.h>


/* All IOCTL commands generate an event with the IOCTL opcode as the */
/* event id after the IOCTL command has been executed. */



/* Asyncronous buffer consumption. This event is generated after a */
/* recived  buffer is consumed during rendering or filled during */
/* capture opeartion. */
#define ADSP_AUDIO_EVT_STATUS_BUF_DONE				0x0108c0d8


/* This event is generated when rendering operation is starving for */
/* data. In order to avoid audio loss at the end of a plauback, the */
/* client should wait for this event before issuing the close command. */
#define ADSP_AUDIO_EVT_STATUS_BUF_UNDERRUN			0x0108c0d9


/* This event is generated during capture operation when there are no */
/* buffers available to copy the captured audio data */
#define ADSP_AUDIO_EVT_STATUS_BUF_OVERFLOW			0x0108c0da


/* This asynchronous event is generated as a result of an input */
/* sample rate change and/or channel mode change detected by the */
/* decoder. The event payload data is an array of 2 uint32 */
/* values containing the sample rate in Hz and channel mode. */

#define ADSP_AUDIO_EVT_SR_CM_CHANGE				0x0108d329


union adsp_audio_event;


struct adsp_audio_no_payload_event {
	/* source address, used in routing */
	struct adsp_audio_address	source;
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* data that identifies this event */
	struct adsp_audio_event_data	event_data;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;
	/* Return status/error code */
	u32				status;

	/* no payload for this event*/
} __attribute__ ((packed));


/* Event struct that returns a signed 32bit value */
struct adsp_audio_signed32_event {
	/* source address, used in routing */
	struct adsp_audio_address	source;
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* data that identifies this event */
	struct adsp_audio_event_data	event_data;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;
	/* Return status/error code */
	u32				status;

	/* payload */
	s32				value;
} __attribute__ ((packed));


/* Event struct that returns a unsigned 32bit value */
struct adsp_audio_unsigned32_event {
	/* source address, used in routing */
	struct adsp_audio_address	source;
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* data that identifies this event */
	struct adsp_audio_event_data	event_data;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;
	/* Return status/error code */
	u32				status;

	/* payload */
	u32				value;
} __attribute__ ((packed));


/* Event struct that returns a unsigned 64bit value */
/* Used to pass media session time in AVSync return events */
struct adsp_audio_unsigned64_event {
	/* source address, used in routing */
	struct adsp_audio_address	source;
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* data that identifies this event */
	struct adsp_audio_event_data	event_data;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;
	/* Return status/error code */
	u32				status;

	/* payload */
	u64				value;
} __attribute__ ((packed));


/* Max number of bytes allowed in event byte stream payload */
#define ADSP_AUDIO_EVT_BYTE_ARRAY_SIZE		32


/* Event struct that returns a byte array */
/* Max 32 bytes, use adsp_audio_data_event & */
/* shared memory for larger arrays */
struct adsp_audio_byte_array_event {
	/* source address, used in routing */
	struct adsp_audio_address	source;
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* data that identifies this event */
	struct adsp_audio_event_data	event_data;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;
	/* Return status/error code */
	u32				status;

	/* payload */
	/* number of bytes used (32 is max) */
	s32				len;
	/* byte array */
	u8				bytes[ADSP_AUDIO_EVT_BYTE_ARRAY_SIZE];
} __attribute__ ((packed));


/* Event struct that returns a byte array in shared memory */
/* Max 32 bytes, use adsp_audio_data_event & */
/* shared memory for larger arrays */
struct adsp_audio_data_event {
	/* source address, used in routing */
	struct adsp_audio_address	source;
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* data that identifies this event */
	struct adsp_audio_event_data	event_data;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;
	/* Return status/error code */
	u32				status;

	/* payload */
	/* number of bytes used */
	s32				len;
	/* pointer to byte array */
	u8				*bytes;
} __attribute__ ((packed));


/* Event struct that returns an audio buffer */
struct adsp_audio_buffer_event {
	/* source address, used in routing */
	struct adsp_audio_address	source;
	/* destination address, used in routing */
	struct adsp_audio_address	dest;
	/* data that identifies this event */
	struct adsp_audio_event_data	event_data;
	/* read-only client data */
	struct adsp_audio_client_data	client_data;
	/* Return status/error code */
	u32				status;

	/* payload */
	/* media data buffer */
	struct adsp_audio_data_buffer	buffer;
} __attribute__ ((packed));


/* Union of all event types */
union adsp_audio_event {
	struct adsp_audio_no_payload_event	no_payload;
	struct adsp_audio_signed32_event	signed32;
	struct adsp_audio_unsigned32_event	unsigned32;
	struct adsp_audio_unsigned64_event	unsigned64;
	struct adsp_audio_byte_array_event	byte_array;
	struct adsp_audio_data_event		data;
	struct adsp_audio_buffer_event		buffer;
};

#endif

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

#ifndef _CAD_ADPCM_FORMAT_H_
#define _CAD_ADPCM_FORMAT_H_


/* cad adpcm version */
#define CAD_ADPCM_VERSION_10		0x10

/* cad ADPCM boolean flags */
#define CAD_ADPCM_SIGNED		0x00000001	/*signed*/
#define CAD_ADPCM_INTERLEAVED		0x00000002	/*interleaved*/
#define CAD_ADPCM_LITTLE_ENDIAN		0x00000004	/*little endian*/


struct cad_adpcm_format_struct {
	/* cad adpcm version */
	u32			version;
	/* number of channels */
	u32			channels;
	/* value is defined by enum cad_sample_bits */
	u32			bit_per_sample;
	/* the value is defined by enum cad_pcm_sampling_rate */
	u32			sampling_rate;
	/* bit map for the boolean maps, a bit 1 indicates true */
	u32			flags;
	u32			block_size;
};

#endif


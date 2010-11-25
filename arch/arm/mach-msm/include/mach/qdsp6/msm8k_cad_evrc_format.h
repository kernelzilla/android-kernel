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

#ifndef CAD_EVRC_FORMAT_H
#define CAD_EVRC_FORMAT_H


#include <mach/qdsp6/msm8k_cad_format_comm.h>

/* cad evric boolean flags */
#define CAD_EVRC_RATE_REDUCTION		0x00000001
#define CAD_EVRC_HI_PASS_FILTER		0x00000002
#define CAD_EVRC_NOISE_SUPPRESSOR	0x00000004
#define CAD_EVRC_POST_FILTER		0x00000008


/*EVRC stream format parameters */
struct cad_evrc_format {
	u32	size;
	u32	version;

	/* number of channels in the stream*/
	u32	channel_count;

	/* value defined in enum cad_cdma_rate*/
	u32	cdma_rate;

	/* minmal rate for the encoder = 1,2,3,4, default = 1*/
	u32	min_bit_rate;

	/* maximal rate for the encoder = 1,2,3,4, default = 4*/
	u32	max_bit_rate;

	/* bit map for evrc flags, a bit 1 indicate true*/
	u32	evrc_bool_flags;
};
#endif




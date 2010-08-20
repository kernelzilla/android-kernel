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

#ifndef CAD_WRITE_SBC_FORMAT_H
#define CAD_WRITE_SBC_FORMAT_H


/* indicating SNR/Loudness */
#define CAD_ENC_SBC_ALLOCATION_METHOD_LOUDNESS		0x0
#define CAD_ENC_SBC_ALLOCATION_METHOD_SNR		0x1


/* four or eight sub-bands are supported for SBC */
#define CAD_SBC_FOUR_SUBBANDS		4
#define CAD_SBC_EIGHT_SUBBANDS		8


/* four different block lengths are supported */
#define CAD_SBC_BLOCK_LEN_FOUR		4
#define CAD_SBC_BLOCK_LEN_EIGHT		8
#define CAD_SBC_BLOCK_LEN_TWELVE	12
#define CAD_SBC_BLOCK_LEN_SIXTEEN	16


/* This structure provides configuration of SBC on QCOM MSM */
struct cad_sbc_enc_cfg_struct_type {
	/* Subband number : ( 4 OR 8 ) */
	u32	num_subbands;

	/* Block length: (4, 8, 12, 16) */
	u32	block_len;

	/* Channel Mode (Mono, Dual, Stereo, or Joint Stereo) */
	u32	channel_mode;

	/* Allocation method: SNR or Loundness */
	u32	allocation_method;

	/* bits per second */
	u32	bit_rate;
};


#endif




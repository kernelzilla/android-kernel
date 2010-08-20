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

#ifndef WMA_FORMAT_H
#define WMA_FORMAT_H


/* Audio content encoded using Windows Media Audio version 9 codec */
#define CAD_MT_MN_WMA_V9STD		0x0108efb4

/* Audio content encoded using Windows Media Audio version 10 codec */
#define CAD_MT_MN_WMA_V10PRO		0x0108efb5


/* Use for both V10 & V9 */
struct cad_wma_format {
	u16	format_tag;
	u16	channels;
	u32	samples_per_sec;
	u32	avg_bytes_per_sec;
	u16	block_align;
	u16	valid_bits_per_sample;
	u32	channel_mask;
	u16	encode_opt;

	/* Only WMA PRO uses these two values */
	u16	advanced_encode_opt;
	u32	advanced_encode_opt2;

	u32	drc_peak_reference;
	u32	drc_peak_target;
	u32	drc_average_reference;
	u32	drc_average_target;
};



#endif




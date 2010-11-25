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

#ifndef _CAD_FORMAT_COMMOM_H_
#define _CAD_FORMAT_COMMOM_H_


#define CAD_WRITE_AMR_VERSION_10	0x10


/* common digital sample rate */
enum cad_sample_rate {
	CAD_SR_VARIABLE = 0,	/* variable rate or unknown bit rate */
	CAD_SR_8000,
	CAD_SR_11025,
	CAD_SR_22050,
	CAD_SR_32000,
	CAD_SR_44056,
	CAD_SR_44100,
	CAD_SR_47250,
	CAD_SR_48000,
	CAD_SR_50000,
	CAD_SR_50400,
	CAD_SR_88200,
	CAD_SR_96000,
	CAD_SR_176400,
	CAD_SR_192000,
	CAD_SR_2822400,
	CAD_SR_MAX = 0x7FFFFFFF
};

/* CDMA Rate types */
enum cad_cdma_rate {
	cad_cdma_rate_blank = 0,	/*CDMA encoded frame is blank*/
	cad_cdma_rate_eighth,		/*CDMA encoded frame in eighth rate*/
	cad_cdma_rate_quarter,		/*CDMA encoded frame in quarter rate*/
	cad_cdma_rate_half,		/*CDMA encoded frame in half rate*/
	cad_cdma_rate_full,		/*CDMA encoded frame in full rate*/
	cad_cdma_rate_erasure,		/*CDMA erasure frame*/
	cad_cdma_rate_max = 0x7FFFFFFF
};

#endif


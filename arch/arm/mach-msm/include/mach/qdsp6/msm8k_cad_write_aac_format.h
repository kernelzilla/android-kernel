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

#ifndef CAD_WRITE_AAC_FORMAT_H
#define CAD_WRITE_AAC_FORMAT_H

#define CAD_WRITE_AAC_VERSION_10	0x10

/*
* 0. 96000, 1. 88200, 2. 64000, 3. 48000, 4. 44100, 5. 32000
* 6. 24000, 7. 22050, 8. 16000, 9. 12000, 10. 11025, 11. 8000
*/
#define CAD_SAMPLE_RATE_96000		0
#define CAD_SAMPLE_RATE_88200		1
#define CAD_SAMPLE_RATE_64000		2
#define CAD_SAMPLE_RATE_48000		3
#define CAD_SAMPLE_RATE_44100		4
#define CAD_SAMPLE_RATE_32000		5
#define CAD_SAMPLE_RATE_24000		6
#define CAD_SAMPLE_RATE_22050		7
#define CAD_SAMPLE_RATE_16000		8
#define CAD_SAMPLE_RATE_12000		9
#define CAD_SAMPLE_RATE_11025		10
#define CAD_SAMPLE_RATE_08000		11

/*
*  1: Mono
*  2: Stereo (interleaved)
*/
#define CAD_CHANNEL_CFG_MONO		1
#define CAD_CHANNEL_CFG_STEREO		2

/*
*  0xffff, ADTS
*  0     , RAW (ADIF) format
*  1     , PSUEDO-RAW format
*  2     , LOAS format
*/
#define CAD_BLK_FMT_ADTS		0xFFFF
#define CAD_BLK_FMT_RAW			0
#define CAD_BLK_FMT_PSEUDO_RAW		1
#define CAD_BLK_FMT_LOAS		2

/*
*  2, bitstream is in AAC LC format
*  4, bitstream is in AAC LTP format
* 17, bitstream is in ER AAC LC format
* 22, bitsream is in  AAC BSAC format
* other audio object types are not supported
*/
#define CAD_AUDIO_OBJ_TYPE_AAC_LC	2
#define CAD_AUDIO_OBJ_TYPE_AAC_LTP	4
#define CAD_AUDIO_OBJ_TYPE_ER_AAC_LC	17
#define CAD_AUDIO_OBJ_TYPE_AAC_BSAC	22

/*
*  ={0,1,2,3}, indicating the configuration of the error
*  protection scheme. This information shall be retrieved from
* the mp4 header. This information is required by the DSP only
* when AOT==17. Currently, only epConfig=0 is supported.
*/
#define CAD_ERR_PROT_SCHEME_0		0
#define CAD_ERR_PROT_SCHEME_1		1
#define CAD_ERR_PROT_SCHEME_2		2
#define CAD_ERR_PROT_SCHEME_3		3


struct cad_write_aac_struct_type {
	u16	sample_rate;
	u16	channel_config;
	u16	block_formats;
	u16	audio_object_type;
	u16	ep_config;
	/*
	* ={0,1}, indicating whether the VCB11 (an error resilience tool) is
	* used. This information shall be retrieved from the mp4 header.
	* Note that this field must be zero if (AOT!=17)
	*/
	u16	aac_section_data_resilience_flag;
	/*
	*  ={0,1}, indicating whether the RVLC (an error resilience tool)
		is used.
	* This information shall be retrieved from the mp4 header. Note that
	* this field must be zero if (AOT!=17)
	*/
	u16	aac_scalefactor_data_resilience_flag;
	/*
	* ={0,1}, indicating whether the HCR (an error resilience tool) is used.
	* This information shall be retrieved from the mp4 header. Note that
	* this field must be zero if (AOT!=17)
	*/
	u16	aac_spectral_data_resilience_flag;
	/*
	* = 1 to turn on SBR if present in the bitstream
	* = 0 to turn off SBR
	*/
	u16	sbr_on_flag;
	/*
	* = 1 to turn on PS if present in the bitstream.
	* = 0 to turn off PS
	*/
	u16	sbr_ps_on_flag;
	u32	bit_rate;
};


struct cad_write_aac_format_struct_type {
	u16 ver_id;
	struct cad_write_aac_struct_type aac;
};

#endif

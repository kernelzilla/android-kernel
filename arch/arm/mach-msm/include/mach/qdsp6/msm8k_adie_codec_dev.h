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

/* defines for the path Id's */
/* populated in the codec database */

/* Path IDs for normal operation. */
#define DAL_ADIE_CODEC_HANDSET_TX             0x010740f6
#define DAL_ADIE_CODEC_HANDSET_RX             0x010740f7
#define DAL_ADIE_CODEC_HEADSET_MONO_TX        0x010740f8
#define DAL_ADIE_CODEC_HEADSET_STEREO_TX      0x010740f9
#define DAL_ADIE_CODEC_HEADSET_MONO_RX        0x010740fa
#define DAL_ADIE_CODEC_HEADSET_STEREO_RX      0x010740fb
#define DAL_ADIE_CODEC_SPEAKER_TX             0x010740fc
#define DAL_ADIE_CODEC_SPEAKER_RX             0x010740fd
#define DAL_ADIE_CODEC_SPEAKER_STEREO_RX      0x01074101

/* Path IDs used for TTY */
#define DAL_ADIE_CODEC_TTY_HEADSET_TX         0x010740fe
#define DAL_ADIE_CODEC_TTY_HEADSET_RX         0x010740ff

/* Path IDs used by Factory Test Mode. */
#define DAL_ADIE_CODEC_FTM_MIC1_TX            0x01074108
#define DAL_ADIE_CODEC_FTM_MIC2_TX            0x01074107
#define DAL_ADIE_CODEC_FTM_HPH_L_RX           0x01074106
#define DAL_ADIE_CODEC_FTM_HPH_R_RX           0x01074104
#define DAL_ADIE_CODEC_FTM_EAR_RX             0x01074103
#define DAL_ADIE_CODEC_FTM_SPKR_RX            0x01074102

/* Path IDs for Loopback */

/* Path IDs used for Line in -> AuxPGA -> Line Out Stereo Mode*/
#define DAL_ADIE_CODEC_AUXPGA_LINEOUT_STEREO_LB      0x01074100

/* Line in -> AuxPGA -> LineOut Mono */
#define DAL_ADIE_CODEC_AUXPGA_LINEOUT_MONO_LB 0x01073d82
/* Line in -> AuxPGA -> Stereo Headphone */
#define DAL_ADIE_CODEC_AUXPGA_HDPH_STEREO_LB  0x01074109
/* Line in -> AuxPGA -> Mono Headphone */
#define DAL_ADIE_CODEC_AUXPGA_HDPH_MONO_LB    0x01073d85

/* Line in -> AuxPGA -> Earpiece */
#define DAL_ADIE_CODEC_AUXPGA_EAP_LB          0x01073d81
/* Line in -> AuxPGA -> AuxOut */
#define DAL_ADIE_CODEC_AUXPGA_AUXOUT_LB       0x01073d86


/* Concurrency Profiles */
#define DAL_ADIE_CODEC_SPKR_STEREO_HDPH_MONO_RX  0x01073d83

#define DAL_ADIE_CODEC_SPKR_MONO_HDPH_MONO_RX    0x01073d84

#define DAL_ADIE_CODEC_SPKR_MONO_HDPH_STEREO_RX  0x01073d88
#define DAL_ADIE_CODEC_SPKR_STEREO_HDPH_STEREO_RX  0x01073d89

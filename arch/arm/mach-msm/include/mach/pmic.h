/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

enum spkr_left_right {
    LEFT_SPKR,
    RIGHT_SPKR,
    SPKR_OUT_OF_RANGE          /* Not valid */
};

enum spkr_gain {
    SPKR_GAIN_MINUS16DB,      /* -16 db */
    SPKR_GAIN_MINUS12DB,      /* -12 db */
    SPKR_GAIN_MINUS08DB,      /* -08 db */
    SPKR_GAIN_MINUS04DB,      /* -04 db */
    SPKR_GAIN_00DB,           /*  00 db */
    SPKR_GAIN_PLUS04DB,       /* +04 db */
    SPKR_GAIN_PLUS08DB,       /* +08 db */
    SPKR_GAIN_PLUS12DB,       /* +12 db */
    SPKR_GAIN_OUT_OF_RANGE    /* Not valid */
};

/* Turn the speaker on or off and enables or disables mute.*/
enum spkr_cmd {
    SPKR_DISABLE,  /* Enable Speaker                                 */
    SPKR_ENABLE,   /* Disable Speaker                                */
    SPKR_MUTE_OFF, /* turn speaker mute off, SOUND ON                */
    SPKR_MUTE_ON,  /* turn speaker mute on, SOUND OFF                */
    SPKR_OFF,      /* turn speaker OFF (speaker disable and mute on) */
    SPKR_ON,        /* turn speaker ON (speaker enable and mute off)  */
    SPKR_SET_FREQ_CMD,    /* set speaker frequency */
    SPKR_GET_FREQ_CMD,    /* get speaker frequency */
    SPKR_SET_GAIN_CMD,    /* set speaker gain */
    SPKR_GET_GAIN_CMD,    /* get speaker gain */
    SPKR_SET_DELAY_CMD,   /* set speaker delay */
    SPKR_GET_DELAY_CMD,   /* get speaker delay */
    SPKR_SET_PDM_MODE,
    SPKR_SET_PWM_MODE,
    SPKR_CMD_OUT_OF_RANGE /* Not valid */
};

struct spkr_config_mode {
    uint32_t is_right_chan_en;
    uint32_t is_left_chan_en;
    uint32_t is_right_left_chan_added;
    uint32_t is_stereo_en;
    uint32_t is_usb_with_hpf_20hz;
    uint32_t is_mux_bypassed;
    uint32_t is_hpf_en;
    uint32_t is_sink_curr_from_ref_volt_cir_en;
};

enum mic_volt {
    MIC_VOLT_2_00V,            /*  2.00 V  */
    MIC_VOLT_1_93V,            /*  1.93 V  */
    MIC_VOLT_1_80V,            /*  1.80 V  */
    MIC_VOLT_1_73V,            /*  1.73 V  */
    MIC_VOLT_OUT_OF_RANGE      /* Not valid */
};

enum ledtype {
	LED_LCD,
	LED_KEYPAD,
	LED_TYPE_OUT_OF_RANGE
};

int spkr_en_right_chan(const unsigned char enable);
int spkr_is_right_chan_en(unsigned char *enabled);
int spkr_en_left_chan(const unsigned char enable);
int spkr_is_left_chan_en(unsigned char *enabled);
int spkr_is_en(const enum spkr_left_right left_right, unsigned char *enabled);
int spkr_get_gain(const enum spkr_left_right left_right, enum spkr_gain *gain);
int set_speaker_gain(const enum spkr_gain speaker_gain);
int speaker_cmd(const enum spkr_cmd cmd);
int set_spkr_configuration(const struct spkr_config_mode *t);
int get_spkr_configuration(struct spkr_config_mode *t);

int mic_en(const unsigned char enable);
int mic_is_en(unsigned char *enabled);
int mic_set_volt(const enum mic_volt type);

/* Cannot use 'current' as the parameter name because 'current' is defined as
 * a macro to get a pointer to the current task.
 */
int flash_led_set_current(const uint16_t milliamps);

int set_led_intensity(const enum ledtype type, int val);

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

#include <mach/qdsp6/msm8k_cad_module.h>

#define CAD_STREAM_MAX_GAIN                     1200
#define CAD_STREAM_MIN_GAIN                     -4000

#define CAD_DEVICE_HANDSET_MAX_GAIN             1100
#define CAD_DEVICE_HANDSET_MIN_GAIN             -700
#define CAD_DEVICE_HEADSET_MAX_GAIN             400
#define CAD_DEVICE_HEADSET_MIN_GAIN             -1400
#define CAD_DEVICE_SPEAKER_MAX_GAIN             500
#define CAD_DEVICE_SPEAKER_MIN_GAIN             -700
#define CAD_DEVICE_BT_SCO_MAX_GAIN              400
#define CAD_DEVICE_BT_SCO_MIN_GAIN              -1400
#define CAD_DEVICE_BT_A2DP_MAX_GAIN             400
#define CAD_DEVICE_BT_A2DP_MIN_GAIN             -1400
#define CAD_DEVICE_TTY_MAX_GAIN                 400
#define CAD_DEVICE_TTY_MIN_GAIN                 -1400

#define CAD_FILTER_CONFIG_DEVICE_VOLUME_VERID   0x1000
#define CAD_FILTER_CONFIG_STREAM_VOLUME_VERID   0x1000

#define CAD_FILTER_CONFIG_DEVICE_MUTE_VERID     0x1000
#define CAD_FILTER_CONFIG_STREAM_MUTE_VERID     0x1000

#define CAD_FILTER_CONFIG_DEVICE_VOLUME         0x01087472
#define CAD_FILTER_CONFIG_DEVICE_MUTE           0x01087473

#define CAD_FILTER_CONFIG_STREAM_VOLUME         0x010874f2
#define CAD_FILTER_CONFIG_STREAM_MUTE           0x010874f3

#define CAD_DEVICE_FILTER_TYPE_VOL		0x0108c383

struct cad_flt_cfg_dev_vol {
	u32 ver_id;
	u32 device_id;
	u32 path;    /* 0 == Rx, 1 == Tx and 2 == both  */
	s32 volume;  /* Per Device: % map to the gain range of each device. */
};

/*
	Per Stream: 0 is max gain, -4000 is min gain.
	Absolute mB value.
	Support for full range volume control only limited by HW capability.
*/
struct cad_flt_cfg_strm_vol {
	u32 ver_id;
	s32 volume;
};

struct cad_flt_cfg_dev_mute {
	u32 ver_id;
	u32 device_id;
	u32 path;   /* 0 == Rx, 1 == Tx and 2 == both */
	u32 mute;   /* 0 == UnMute,  1 == Mute        */
};

struct cad_flt_cfg_strm_mute {
	u32 ver_id;
	u32 mute;    /* 0 == UnMute,  1 == Mute */
};

#define QDSP6VOLUME_MAX_DEVICE_COUNT         0x0F

struct cad_device_volume_cache {
	s32 current_volume;
	s32 default_volume;
	s32 max_gain;
	s32 min_gain;
	u32 mute;                 /* 1: Mute   0: UnMute  */
	u8  valid_current_volume; /* 1: Valid  0: Invalid */
	u8  setup_session;        /* 1: True   0: False   */
};

/*
	where struct cad_device_volume_cache is the type of an array,
	and the array index is device ID.
*/

s32 qdsp6_stream_volume_mapping(s32 percentage);
s32 qdsp6_volume_mapping(u32 deviceId, s32 percentage);


#define QDSP_IOCTL_CMD_STREAM_DTMF_START            0x01087553

struct q6_dtmf_start {
	u32 tone1_hz;      /* First tone in Hz */
	u32 tone2_hz;      /* Second tone in Hz */
	s32 duration_usec; /* Duration in microseconds */
	s32 gain_mb;       /* Gain in millibels */
};

void set_audio_ctrl_handle(u32 handle);

int cad_volume_dinit(void);
int cad_volume_init(struct cad_func_tbl_type **func_tbl);

int cad_apply_cached_vol_on_dev(u32 device_id);
int volume_set_max_vol_all(void);


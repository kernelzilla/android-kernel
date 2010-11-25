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

#ifndef ARDI_H
#define ARDI_H

#include "msm8k_cad_module.h"
#include "msm8k_ard.h"
#include "linux/mutex.h"
#include "msm8k_cad_devices.h"
#include "msm8k_cad_itypes.h"
#include "msm8k_adsp_audio_event.h"

#define ARD_AUDIO_MAX_CLIENT	10
#define MAX_NUM_TX_DEVICE	2
#define MAX_NUM_RX_DEVICE	2
#define MAX_NUM_DEVICES		8


enum ard_state_enum_type {
	ARD_STATE_RESET,
	ARD_STATE_CLK_ACTIVE,
	ARD_STATE_AFE_ACTIVE,
	ARD_STATE_ACTIVE
};

enum codec_enum_type {
	CODEC_INT,
	CODEC_AUX_PCM,
	CODEC_I2S,
	CODEC_ERROR
};

enum ard_state_ret_enum_type {
	ARD_STATE_RC_SUCCESS,
	ARD_STATE_RC_CONTINUE
};

enum ard_ret_enum_type {
	ARD_FALSE = 0,
	ARD_TRUE
};

enum ard_session_enum_type {
	DEVICE_CTRL_TYPE,
	STREAM_TYPE
};

struct ard_global_data_struct_type {
	u32		audio_device;
	u32		voice_device;
};

struct ard_session_info_struct_type {
	enum ard_session_enum_type	session_type;
	u32				group_id;
	struct cadi_open_struct_type	*sess_open_info;
	enum sample_rate_type		rx;
	enum sample_rate_type		tx;
	u8				enabled;
	u8				available;
	u8				active;
	u8				qdsp6_opened;
	u8				qdsp6_started;
	void				*local_format_block;
	struct mutex			session_mutex;
};

struct ard_device_struct_type {
	u32			state;
	u8			device_type;
	u8			device_configured;
	u8			device_config_request;
	u8			device_change_request;
	u32			device_inuse;
	enum ard_ret_enum_type	afe_enabled;
	u8			dsp_started;
	u8			clk_configured;
	u32			stream_count;
	struct mutex		device_mutex;
	u32			device_sample_rate;
};

struct ard_state_struct_type {
	struct ard_device_struct_type	ard_device[MAX_NUM_DEVICES];
	struct mutex			ard_state_machine_mutex;
	u32				def_tx_device;
	u32				def_rx_device;
	u32				new_tx_device;
	u32				new_rx_device;
};


enum ard_ret_enum_type valid_session_present(u32 dev_id);

s32 qdsp6_enable(s32 session_id);
s32 qdsp6_disable(s32 session_id);
s32 qdsp6_start(s32 session_id);

void ard_callback_func(union adsp_audio_event *ev_data,
		       void *client_data);
enum ard_state_ret_enum_type ard_state_control(s32 session_id,
					       u32 dev_id);
enum ard_state_ret_enum_type ard_state_reset(s32 session_id,
					     u32 dev_id);
enum ard_state_ret_enum_type ard_state_clk_active(s32 session_id,
						  u32 dev_id);
enum ard_state_ret_enum_type ard_state_afe_active(s32 session_id,
						  u32 dev_id);
enum ard_state_ret_enum_type ard_state_active(s32 session_id,
					      u32 dev_id);

enum ard_ret_enum_type check_sampling_rate(void);
enum ard_ret_enum_type device_needs_setup(u32 cad_device);

extern struct ard_session_info_struct_type	*ardsession[CAD_MAX_SESSION];
extern struct ard_state_struct_type		ard_state;
extern u32					device_control_session;

#endif

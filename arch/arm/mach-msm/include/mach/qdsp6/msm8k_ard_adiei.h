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

#ifndef ARDIADIEI_H
#define ARDIADIEI_H

#include "msm8k_cad.h"

#define MAX_ADIE_PATH_TYPES   2

enum adie_state_enum_type {
	ADIE_STATE_RESET,
	ADIE_STATE_DIGITAL_ACTIVE,
	ADIE_STATE_DIGITAL_ANALOG_ACTIVE,
};

enum adie_state_ret_enum_type {
	ADIE_STATE_RC_SUCCESS,
	ADIE_STATE_RC_CONTINUE,
	ADIE_STATE_RC_FAILURE
};

enum adie_ret_enum_type {
	ADIE_FALSE = 0,
	ADIE_TRUE
};

struct adie_path_type_struct_type {
	u32		state;
	u8		enable_request;
	u8		enabled;
};

struct adie_state_struct_type {
	void					*adie_handle;
	u8					adie_opened;
	struct adie_path_type_struct_type
					adie_path_type[MAX_ADIE_PATH_TYPES];
};

u32 adie_state_control(u32 dev_type, u32 dev_id);
enum adie_state_ret_enum_type adie_state_reset(u32 dev_type, u32 dev_id);
enum adie_state_ret_enum_type adie_state_digital_active(u32 dev_type,
		u32 dev_id);
enum adie_state_ret_enum_type adie_state_digital_analog_active(u32 dev_type,
		u32 dev_id);

#endif

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

#ifndef _MSM8K_CAD_Q6DEC_DRVI_H_
#define _MSM8K_CAD_Q6DEC_DRVI_H_

#include <linux/mutex.h>
#include <linux/completion.h>

#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_adsp_audio_command.h>


#define Q6_DEC_MAX_STREAM_COUNT			4
#define Q6_DEC_BUFFER_NUM_PER_STREAM		4
#define Q6_DEC_BUFFER_SIZE_MAX			(1024 * 11)

enum q6dec_session_state {
	Q6_DEC_RESET		= 0x0,
	Q6_DEC_INIT		= 0x1,
	Q6_DEC_READY		= 0x2,
	Q6_DEC_FLUSHING		= 0x4,
	Q6_DEC_CLOSING		= 0x8,
	Q6_DEC_VOICE,
	Q6_DEC_STATE_MAX	= 0x7FFFFFFF
};

struct q6dec_sesson_buffer_node;
struct q6dec_sesson_buffer_node {
	u8					*buf;
	u32					phys_addr;
	struct q6dec_sesson_buffer_node		*next;
};

struct q6dec_session_data;
struct q6dec_session_data {
	u8					*shared_buf;
	struct mutex				session_mutex;
	struct mutex				close_mutex;
	u32					session_id;
	u32					group_id;
	u32					buffer_size;
	struct completion			buf_done_compl;
	struct completion			all_buf_done_compl;
	struct q6dec_sesson_buffer_node		*free_buf_list;
	struct q6dec_sesson_buffer_node		*used_buf_list;
	enum q6dec_session_state		session_state;
	struct q6dec_session_data		*next;
	/* debug variables */
	u32					use_counter;
	u32					ret_counter;
	/* flag to tell if we need flush before close */
	u32					need_flush;
	/* flag for buffer done event */
	u32					need_buffer_done;
	u32					need_all_buf_done;
	struct cad_event_struct_type    	cb_data;
	struct adsp_audio_data_command		q6_data_buf;
};

struct q6dec_data {
	u32				used_stream_count;
	struct q6dec_session_data	*free_session_list;
	struct q6dec_session_data	*used_session_list;
};

#endif

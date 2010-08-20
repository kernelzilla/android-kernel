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

#ifndef _QDSP6AUDIOENCDRIVERI_H_
#define _QDSP6AUDIOENCDRIVERI_H_

#include <linux/completion.h>

#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_adsp_audio_command.h>

#define		Q6_ENC_MAX_SESSION_COUNT	4
#define		Q6_ENC_BUF_PER_SESSION		4
#define		Q6_ENC_BUF_MAX_SIZE		(1024 * 11)

enum q6_enc_session_state {
	Q6_ENC_STATE_RESET = 0,
	Q6_ENC_STATE_INIT,
	Q6_ENC_STATE_PROCESS,
	Q6_ENC_STATE_VOICE,
	Q6_ENC_STATE_CLOSING
};

struct q6_enc_session_buf_node {
	u8					*buf;
	u32					phys_addr;
	u32					buf_len;
	struct q6_enc_session_buf_node		*next;
	/* the following is used when smaller user buffer is used */
	u8					*read_ptr;
};

struct q6_enc_session_data {
	struct mutex			session_mutex;
	struct mutex			close_mutex;
	u32				session_id;
	u32				group_id;
	u32				buf_size;
	struct completion		buf_done_compl;
	s32				signal_buf_done;
	struct completion		all_buf_done_compl;
	s32				signal_all_buf_done;
	u32				need_flush;
	struct q6_enc_session_buf_node	*free_nodes;
	struct q6_enc_session_buf_node	*used_nodes;
	struct q6_enc_session_buf_node	*full_nodes_head;
	struct q6_enc_session_buf_node	*full_nodes_tail;
	enum q6_enc_session_state	session_state;
	struct q6_enc_session_data	*next;
	struct cad_event_struct_type	cb_data;
	u8				*shared_buffer;
	struct adsp_audio_data_command	q6_data_buf;
};

struct q6_audio_enc_data {
	struct q6_enc_session_data	*q6_enc_free_sessions;
	struct q6_enc_session_data	*q6_enc_used_sessions;
};

#endif

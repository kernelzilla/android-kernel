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

#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/uaccess.h>

#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_session_ioctl.h>
#include <mach/qdsp6/msm8k_cad_q6dec_session.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_adsp_audio_stream_ioctl.h>


#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif


static void cad_q6dec_session_async_callback(union adsp_audio_event *evt,
							void *data)
{
	struct q6dec_session_data *self = (struct q6dec_session_data *)data;
	struct q6dec_sesson_buffer_node		*node = NULL;
	struct q6dec_sesson_buffer_node		*prev_node = NULL;

	if ((evt->no_payload.event_data.id == ADSP_AUDIO_IOCTL_CMD_STREAM_EOS)
		&& (self->cb_data.callback != NULL)) {

		self->cb_data.callback(CAD_EVT_STATUS_EOS, NULL, 0,
					 self->cb_data.client_data);
		return;
	}


	if (evt->no_payload.event_data.id != ADSP_AUDIO_EVT_STATUS_BUF_DONE)
		/* unknow event, and do nothing */
		return;

	mutex_lock(&self->session_mutex);
	node = self->used_buf_list;
	while (node) {
		if ((u32)node->buf == evt->buffer.client_data.data) {
			if (prev_node == NULL) {
				/* first node */
				self->used_buf_list = node->next;
				break;
			}
			prev_node->next = node->next;
			break;
		}
		prev_node = node;
		node = node->next;
	}

	if (node) {
		D("========>return buffer number %d, 0x%x\n",
			self->ret_counter, (u32) node);
		self->ret_counter++;
		/* find match, add free buffer to the free list */
		node->next = self->free_buf_list;
		self->free_buf_list = node;

		if (self->need_buffer_done) {
			D("Signal buffer done event\n");
			complete(&self->buf_done_compl);
		}

		if ((self->used_buf_list == NULL) && self->need_all_buf_done) {
			D("Signal all buffer done event\n");
			complete(&self->all_buf_done_compl);
		}

		if (self->cb_data.callback != NULL)
			self->cb_data.callback(evt->
				no_payload.event_data.id, NULL, 0,
				self->cb_data.client_data);
	}
	mutex_unlock(&self->session_mutex);

	return;
}


static struct q6dec_sesson_buffer_node *cad_q6dec_session_get_free_buf(
	struct q6dec_session_data *self)
{
	struct q6dec_sesson_buffer_node *node = NULL;

	mutex_lock(&self->session_mutex);
	if (self->free_buf_list) {
		node = self->free_buf_list;
		self->free_buf_list = self->free_buf_list->next;
		node->next = NULL;
	}
	if (node == NULL)
		self->need_buffer_done = 1;

	mutex_unlock(&self->session_mutex);
	return node;
}


static s32 cad_q6dec_session_send_buf(struct q6dec_session_data *self,
				struct q6dec_sesson_buffer_node *node,
				u32 buf_len)
{
	s32				res = CAD_RES_SUCCESS;

	self->q6_data_buf.buffer.flags = ADSP_AUDIO_BUFFER_FLAG_START_SET |
		ADSP_AUDIO_BUFFER_FLAG_PHYS_ADDR;
	self->q6_data_buf.client_data.data = (u32)node->buf;
	self->q6_data_buf.buffer.buffer_addr = (u32)node->phys_addr;
	self->q6_data_buf.buffer.max_size = buf_len;
	self->q6_data_buf.buffer.actual_size = buf_len;
	self->use_counter++;
	D("==========> use buffer number %d, 0x%x\n",
		       self->use_counter, (u32)node->buf);

	/* add to the used list */
	mutex_lock(&self->session_mutex);
	node->next = self->used_buf_list;
	self->used_buf_list = node;
	mutex_unlock(&self->session_mutex);
	D("----> Send %d byte of data with buffer 0x%x\n",
			self->q6_data_buf.buffer.max_size,
			self->q6_data_buf.buffer.buffer_addr);
	/* rpc write*/
	if (cad_rpc_data(self->session_id, self->group_id,
			(void *)&self->q6_data_buf,
			sizeof(self->q6_data_buf), NULL) !=
			CAD_RES_SUCCESS) {

		pr_err("Can not push write buffers to the Q6!!!\n");

		self->use_counter--;
		self->used_buf_list = self->used_buf_list->next;
		node->next = self->free_buf_list;
		self->free_buf_list = node;
		res = CAD_RES_FAILURE;
	}

	return res;
}



static s32 cad_q6dec_session_write_buffers(struct q6dec_session_data *self,
						u8  **input_buf,
						u32 *input_buf_size)
{
	unsigned long				bytes_ret;
	s32					err = CAD_RES_SUCCESS;
	struct q6dec_sesson_buffer_node		*node = NULL;

	while ((node = cad_q6dec_session_get_free_buf(self)) != NULL) {

		/* copy the data */
		if (*input_buf_size > self->buffer_size) {
			/* we need more buffers */
			bytes_ret = copy_from_user(node->buf, *input_buf,
					   self->buffer_size);
			err = cad_q6dec_session_send_buf(
				self, node, self->buffer_size);

			*input_buf += self->buffer_size;
			*input_buf_size -= self->buffer_size;
		} else {
			/* we are done with the current buffer */
			bytes_ret = copy_from_user(node->buf, *input_buf,
				*input_buf_size);
			err = cad_q6dec_session_send_buf(
				self, node, *input_buf_size);
			*input_buf_size = 0;
			D("========> No more buffer data!!!!!!\n");
			break;
		}
		if (bytes_ret)
			pr_err("Copy from user failed to copy %ld bytes\n",
				bytes_ret);
		if (err) {
			pr_err("Error writing data to Q6!\n");
			break;
		}
	}
	return err;
}

static void cad_q6dec_session_delete_buffer(struct q6dec_sesson_buffer_node
	**list)
{
	struct q6dec_sesson_buffer_node	*node = NULL;

	while (*list) {
		node = (*list)->next;
		kfree(*list);
		*list = node;
	}
}

static s32 cad_q6dec_session_create_buffer(struct q6dec_session_data *self,
						void *cmd_buf)
{
	u32					i;
	s32					result = CAD_RES_SUCCESS;
	struct q6dec_sesson_buffer_node		*node = NULL;
	struct cad_stream_info_struct_type	*info;

	info = (struct cad_stream_info_struct_type *)cmd_buf;
	if ((info->ses_buf_max_size == 0) ||
		(info->ses_buf_max_size > Q6_DEC_BUFFER_SIZE_MAX) ||
		(self->session_state != Q6_DEC_RESET)) {

		return CAD_RES_FAILURE;
	}

	cad_q6dec_session_delete_buffer(&self->free_buf_list);
	cad_q6dec_session_delete_buffer(&self->used_buf_list);
	self->buffer_size = info->ses_buf_max_size;

	if (self->shared_buf == NULL) {

		/* Set to virtual address of shared memory */
		/* reserved for this session. */
		/* memory for each session is stored as: */
		/* |b|p|b|p|b|p|b|p|fb|p */
		/* b - buffer, fb - format block, p - padding */
		self->shared_buf = g_audio_mem +
			((Q6_DEC_BUFFER_NUM_PER_STREAM *
			(Q6_DEC_BUFFER_SIZE_MAX + MEMORY_PADDING))
			* self->session_id);


		if (self->shared_buf == NULL)
			return CAD_RES_FAILURE;
	}

	memset(self->shared_buf, 0, (Q6_DEC_BUFFER_NUM_PER_STREAM *
		(self->buffer_size + MEMORY_PADDING)));

	for (i = 0; i < Q6_DEC_BUFFER_NUM_PER_STREAM; i++) {
		node = kmalloc(sizeof(struct q6dec_sesson_buffer_node),
				GFP_KERNEL);

		if (IS_ERR(node)) {
			result = CAD_RES_FAILURE;
			break;
		}
		memset(node, 0, sizeof(struct q6dec_sesson_buffer_node));
		node->next = self->free_buf_list;
		self->free_buf_list = node;
		D("----> create buffer node 0x%x\n", (u32)node);
		/* Set each buffer to next block of buffer memory */
		/* for this session */
		node->buf = self->shared_buf + i * (self->buffer_size +
			MEMORY_PADDING);

		/* Set to the physical address of buffer memory */
		node->phys_addr = g_audio_base +
			(Q6_DEC_BUFFER_NUM_PER_STREAM *
			(Q6_DEC_BUFFER_SIZE_MAX + MEMORY_PADDING))
			* self->session_id + i *
			(self->buffer_size + MEMORY_PADDING);
	}
	return result;

}


s32 cad_q6dec_session_init(struct q6dec_session_data *self)
{
	mutex_init(&self->session_mutex);
	mutex_init(&self->close_mutex);

	self->q6_data_buf.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_DATA_RX;
	self->q6_data_buf.cmd.response_type = ADSP_AUDIO_RESPONSE_ASYNC;

	return CAD_RES_SUCCESS;
}

s32 cad_q6dec_session_deinit(struct q6dec_session_data *self)
{
	if (self->session_state != Q6_DEC_RESET)
		cad_q6dec_session_close(self);

	mutex_destroy(&self->session_mutex);
	mutex_destroy(&self->close_mutex);
	return CAD_RES_SUCCESS;
}


s32 cad_q6dec_session_open(struct q6dec_session_data *self,
				s32 session_id,
				struct cad_open_struct_type *open_param)
{
	init_completion(&self->buf_done_compl);
	init_completion(&self->all_buf_done_compl);

	if (cad_rpc_reg_callback(session_id,
		cad_q6dec_session_async_callback, self) ==
		CAD_RES_FAILURE) {

		return CAD_RES_FAILURE;
	}

	self->session_id = session_id;
	self->group_id = open_param->group_id;
	self->use_counter = 1;
	self->ret_counter = 1;
	self->need_buffer_done = 0;
	self->need_all_buf_done = 0;
	self->cb_data.client_data = NULL;
	self->cb_data.callback = NULL;
	D("Session open successful\n");
	return CAD_RES_SUCCESS;
}

s32 cad_q6dec_session_close(struct q6dec_session_data *self)
{
	union adsp_audio_event			ret_status;
	struct adsp_audio_no_payload_command	q6_cmd;

	init_completion(&self->all_buf_done_compl);

	mutex_lock(&self->session_mutex);

	if (self->session_state == Q6_DEC_CLOSING) {
		pr_err("CAD:Q6DEC ===> Session already closing, "
			"no need to close.\n");
		mutex_unlock(&self->session_mutex);
		return CAD_RES_SUCCESS;
	}

	self->session_state = Q6_DEC_CLOSING;
	if (self->need_buffer_done)
		complete(&self->buf_done_compl);

	mutex_unlock(&self->session_mutex);

	if ((self->session_state != Q6_DEC_VOICE) && self->need_flush) {
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_STREAM_STOP;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		cad_rpc_control(self->session_id, self->group_id,
			(void *)&q6_cmd, sizeof(q6_cmd), &ret_status);
	}

	mutex_lock(&self->close_mutex);
	mutex_lock(&self->session_mutex);

	if (self->used_buf_list)
		self->need_all_buf_done = 1;

	mutex_unlock(&self->session_mutex);

	if (self->need_all_buf_done) {
		D("CAD:Q6DEC ===> wait for all buffer done event\n");
		wait_for_completion(&self->all_buf_done_compl);
	}

	cad_rpc_dereg_callback(self->session_id,
				cad_q6dec_session_async_callback);

	mutex_lock(&self->session_mutex);
	cad_q6dec_session_delete_buffer(&self->free_buf_list);
	cad_q6dec_session_delete_buffer(&self->used_buf_list);
	if (self->shared_buf != NULL)
		self->shared_buf = NULL;
	self->session_id = 0;
	self->session_state = Q6_DEC_RESET;
	self->need_buffer_done = 0;
	self->need_all_buf_done = 0;
	self->need_flush = 0;
	mutex_unlock(&self->session_mutex);
	mutex_unlock(&self->close_mutex);
	D("Decode session close successful\n");

	return CAD_RES_SUCCESS;
}

s32 cad_q6dec_session_ioctl(struct q6dec_session_data *self,
				u32 cmd_code,
				void *cmd_buf,
				u32 cmd_len)
{
	s32					result = CAD_RES_SUCCESS;
	union adsp_audio_event			ret_status;
	struct adsp_audio_no_payload_command	q6_cmd;
	struct cad_event_struct_type		*cb_struct;

	switch (cmd_code) {
	case CAD_IOCTL_CMD_SET_STREAM_EVENT_LSTR:
		if (cmd_buf == NULL) {
			pr_err("Invalid buffer passed to decoder ioctl\n");
			break;
		}

		cb_struct = (struct cad_event_struct_type *)cmd_buf;
		if ((cb_struct->callback == NULL) ||
			(cb_struct->client_data == NULL)) {

			pr_err("Can not set listener function\n");
			break;
		}
		self->cb_data.client_data = cb_struct->client_data;
		self->cb_data.callback = cb_struct->callback;
		break;
	case CAD_IOCTL_CMD_STREAM_START:
		if (self->session_state != Q6_DEC_INIT) {
			if (self->session_state != Q6_DEC_VOICE)
				pr_err("Cannot start, decoder is in "
					"wrong state\n");
		} else {
			self->session_state = Q6_DEC_READY;
			self->need_flush = 1;
		}
		break;
	case CAD_IOCTL_CMD_SET_STREAM_INFO:
		/* don't need Q6 decoder for voice call */
		if (((struct cad_stream_info_struct_type *)cmd_buf)->app_type
				== CAD_STREAM_APP_VOICE) {
			self->session_state = Q6_DEC_VOICE;
			break;
		}
		result = cad_q6dec_session_create_buffer(self, cmd_buf);
		if (result == CAD_RES_SUCCESS)
			self->session_state = Q6_DEC_INIT;
		break;
	case CAD_IOCTL_CMD_STREAM_FLUSH:
	case CAD_IOCTL_CMD_SESSION_FLUSH:
		mutex_lock(&self->session_mutex);

		self->session_state = Q6_DEC_FLUSHING;
		if (self->need_buffer_done)
			complete(&self->buf_done_compl);

		mutex_unlock(&self->session_mutex);


		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_SESSION_FLUSH;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		result = cad_rpc_control(self->session_id, self->group_id,
			(void *)&q6_cmd, sizeof(q6_cmd), &ret_status);

	case CAD_IOCTL_CMD_STREAM_PAUSE:
	case CAD_IOCTL_CMD_SESSION_PAUSE:
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_SESSION_PAUSE;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		result = cad_rpc_control(self->session_id, self->group_id,
			(void *)&q6_cmd, sizeof(q6_cmd), &ret_status);

		if ((!result) && (cmd_buf != NULL) && (cmd_len == sizeof(u64)))
			*((u64 *)cmd_buf) = ret_status.unsigned64.value;
		break;
	case CAD_IOCTL_CMD_STREAM_END_OF_STREAM:
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_STREAM_EOS;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_ASYNC;

		result = cad_rpc_control(self->session_id, self->group_id,
			(void *)&q6_cmd, sizeof(q6_cmd), &ret_status);
		break;
	case CAD_IOCTL_CMD_STREAM_RESUME:
	case CAD_IOCTL_CMD_SESSION_RESUME:
		if ((self->session_state == Q6_DEC_READY) ||
			(self->session_state == Q6_DEC_FLUSHING)) {

			self->session_state = Q6_DEC_READY;

			q6_cmd.cmd.op_code =
				ADSP_AUDIO_IOCTL_CMD_SESSION_RESUME;
			q6_cmd.cmd.response_type =
				ADSP_AUDIO_RESPONSE_COMMAND;

			result = cad_rpc_control(self->session_id,
				self->group_id, (void *)&q6_cmd,
				sizeof(q6_cmd), &ret_status);

			if ((!result) && (cmd_buf != NULL) &&
				(cmd_len == sizeof(u64)))

				*((u64 *)cmd_buf) =
					ret_status.unsigned64.value;
		} else {
			result = CAD_RES_FAILURE;
		}
		break;
	default:
		break;
	}

	return result;
}

s32 cad_q6dec_session_write(struct q6dec_session_data *self,
				struct cad_buf_struct_type *buffer)
{
	u8				*buf;
	u32				buf_size;
	s32				err;

	buf = (u8 *)buffer->buffer;
	buf_size = buffer->max_size;

	self->q6_data_buf.buffer.start = buffer->time_stamp;
	init_completion(&self->buf_done_compl);

	mutex_lock(&self->close_mutex);
	mutex_lock(&self->session_mutex);
	if ((self->session_state != Q6_DEC_READY) &&
		(self->session_state != Q6_DEC_FLUSHING)) {

		pr_err("Received write command in wrong state for ses: %d\n",
			self->session_id);
		mutex_unlock(&self->close_mutex);
		mutex_unlock(&self->session_mutex);
		return CAD_RES_FAILURE;
	}
	self->session_state = Q6_DEC_READY;
	mutex_unlock(&self->session_mutex);

	D("========> Start send %d byte buffer data......\n", buf_size);
	while (buf_size > 0) {
		err = cad_q6dec_session_write_buffers(self, &buf, &buf_size);

		if (err)
			pr_err("Failed write to Q6\n");
		if (buf_size == 0) {
			D("========> Done sending the buffer data!\n");
			break;
		}

		D("-------> Wait for buffer done event\n");
		wait_for_completion(&self->buf_done_compl);

		mutex_lock(&self->session_mutex);
		self->need_buffer_done = 0;
		if (self->session_state != Q6_DEC_READY) {
			mutex_unlock(&self->session_mutex);
			D("CAD:Q6DEC ===> No need to send the data in "
				"state %d\n", self->session_state);
			break;
		}
		mutex_unlock(&self->session_mutex);
	}
	self->need_buffer_done = 0;
	mutex_unlock(&self->close_mutex);
	return CAD_RES_SUCCESS;
}


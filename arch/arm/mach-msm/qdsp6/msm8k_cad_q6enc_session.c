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

#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <mach/qdsp6/msm8k_cad_session_ioctl.h>
#include <mach/qdsp6/msm8k_cad_q6enc_session.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_q6dec_drvi.h>
#include <mach/qdsp6/msm8k_adsp_audio_stream_ioctl.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

/* release the buffer memory */
static void release_buffers(struct q6_enc_session_buf_node **head)
{
	struct q6_enc_session_buf_node *node;
	while (*head) {
		node = (*head)->next;
		kfree(*head);
		*head = node;
	}
}

/* create the buffer node */
static s32 create_buffers(struct q6_enc_session_data *self, void *cmd_buf)
{
	u32					i;
	s32					res;
	struct q6_enc_session_buf_node		*node = NULL;
	struct cad_stream_info_struct_type	*info;

	res = CAD_RES_SUCCESS;
	info = (struct cad_stream_info_struct_type *)cmd_buf;

	/* limit the buffer size and create buf only in reset state */
	if (info->ses_buf_max_size == 0  ||
		info->ses_buf_max_size > Q6_ENC_BUF_MAX_SIZE ||
		self->session_state != Q6_ENC_STATE_RESET) {

		return CAD_RES_FAILURE;
	}

	self->buf_size = info->ses_buf_max_size;
	/* for safty, clean all the buffer lists */
	release_buffers(&self->full_nodes_head);
	self->full_nodes_head = NULL;
	self->full_nodes_tail = NULL;
	release_buffers(&self->used_nodes);
	release_buffers(&self->free_nodes);

	if (self->shared_buffer == NULL) {

		/* Set to virtual address of shared memory */
		/* reserved for this session. */
		/* memory for each session is stored as: */
		/* |b|p|b|p|b|p|b|p|fb|p */
		/* b - buffer, fb - format block, p - padding */
		self->shared_buffer = g_audio_mem +
			((Q6_ENC_BUF_PER_SESSION *
			(Q6_ENC_BUF_MAX_SIZE + MEMORY_PADDING))
			* self->session_id);

		if (self->shared_buffer == NULL)
			return CAD_RES_FAILURE;
	}

	memset(self->shared_buffer, 0, (Q6_ENC_BUF_PER_SESSION *
		(self->buf_size + MEMORY_PADDING)));

	/* it's time the create the read buffers */
	for (i = 0; i < Q6_ENC_BUF_PER_SESSION; i++) {

		node = kmalloc(sizeof(struct q6_enc_session_buf_node),
				GFP_KERNEL);
		if (node == NULL) {
			release_buffers(&self->free_nodes);
			res = CAD_RES_FAILURE;
			break;
		}

		memset(node, 0, sizeof(struct q6_enc_session_buf_node));
		node->next = self->free_nodes;
		self->free_nodes = node;
		D("----> create buffer node 0x%x\n", (u32)node);
		/* Set each buffer to next block of buffer memory */
		/* for this session */
		node->buf = (u8 *)((u32)self->shared_buffer +
			i * (self->buf_size + MEMORY_PADDING));

		/* Set to the physical address of buffer memory */
		node->phys_addr = g_audio_base +
			(Q6_ENC_BUF_PER_SESSION *
			(Q6_ENC_BUF_MAX_SIZE + MEMORY_PADDING))
			* self->session_id + i *
			(self->buf_size + MEMORY_PADDING);
	}
	return res;
}

/* push all the free buffers to the Q6 */
static s32 send_buffers(struct q6_enc_session_data *self)
{
	struct q6_enc_session_buf_node	*node = NULL;
	s32				res = CAD_RES_SUCCESS;

	mutex_lock(&self->session_mutex);
	while (self->free_nodes) {
		node = self->free_nodes;
		self->free_nodes = node->next;

		self->q6_data_buf.client_data.data = (u32)node->buf;
		self->q6_data_buf.buffer.flags =
			ADSP_AUDIO_BUFFER_FLAG_PHYS_ADDR;
		self->q6_data_buf.buffer.buffer_addr = node->phys_addr;
		self->q6_data_buf.buffer.actual_size = self->buf_size;
		self->q6_data_buf.buffer.max_size = self->buf_size;

		/* add to used list */
		node->next = self->used_nodes;
		self->used_nodes = node;
		mutex_unlock(&self->session_mutex);

		if (cad_rpc_data(self->session_id, self->group_id,
			(void *)&self->q6_data_buf,
			sizeof(self->q6_data_buf), NULL) !=
			CAD_RES_SUCCESS) {

			pr_err("Can not push read buffers to the Q6!!!\n");
			mutex_lock(&self->session_mutex);
			self->used_nodes = self->used_nodes->next;
			node->next = self->free_nodes;
			self->free_nodes = node;
			mutex_unlock(&self->session_mutex);
			res = CAD_RES_FAILURE;
			break;
		}
		D("Send Buffer (0x%x) to Q6\n",
			self->q6_data_buf.buffer.buffer_addr);
		/* lock for next loop iteration */
		mutex_lock(&self->session_mutex);
	}
	mutex_unlock(&self->session_mutex);
	return res;
}

static void cad_q6enc_session_handle_async_evt(
	union adsp_audio_event *return_event, void *client_data)
{
	struct q6_enc_session_buf_node	*node = NULL;
	struct q6_enc_session_buf_node	*prev_node = NULL;
	struct q6_enc_session_data	*self = client_data;

	if (return_event->no_payload.event_data.id !=
		ADSP_AUDIO_EVT_STATUS_BUF_DONE) {

		/* unhandled event */
		pr_err("invalid event ID: %d\n",
			return_event->no_payload.event_data.id);
		return;
	}

	mutex_lock(&self->session_mutex);
	node = self->used_nodes;

	while (node) {
		if ((u32)node->buf ==
			return_event->buffer.client_data.data) {

			if (prev_node == NULL) {
				self->used_nodes = node->next;
				break;
			}
			prev_node->next = node->next;
			break;
		}
		prev_node = node;
		node = node->next;
	}

	if (node) {
		node->buf_len = return_event->
			buffer.buffer.actual_size;

		/* put this node into full list */
		D("Get full read buffer(0x%x)!\n",
			return_event->buffer.buffer.buffer_addr);

		if (self->full_nodes_head == NULL) {
			self->full_nodes_head = node;
			self->full_nodes_tail = node;
		} else {
			self->full_nodes_tail->next = node;
			self->full_nodes_tail = node;
		}

		/* signal buf done event */
		if (self->signal_buf_done)
			complete(&self->buf_done_compl);
		/* check if we need all buf done */
		if ((self->used_nodes == NULL) && self->signal_all_buf_done)
			complete(&self->all_buf_done_compl);

		if (self->cb_data.callback != NULL)
			self->cb_data.callback(return_event->no_payload.
			event_data.id, NULL, 0, self->cb_data.client_data);
	}
	mutex_unlock(&self->session_mutex);
	return;
}


s32 cad_q6enc_session_init(struct q6_enc_session_data *self)
{
	mutex_init(&self->session_mutex);
	mutex_init(&self->close_mutex);
	init_completion(&self->all_buf_done_compl);
	init_completion(&self->buf_done_compl);

	self->q6_data_buf.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_DATA_TX;
	self->q6_data_buf.cmd.response_type = ADSP_AUDIO_RESPONSE_ASYNC;

	return CAD_RES_SUCCESS;
}

s32 cad_q6enc_session_dinit(struct q6_enc_session_data *self)
{
	/* just an extra check */
	if (self->session_state != Q6_ENC_STATE_RESET)
		cad_q6enc_session_close(self);

	mutex_unlock(&self->session_mutex);
	mutex_unlock(&self->close_mutex);

	return CAD_RES_SUCCESS;
}


s32 cad_q6enc_session_open(struct q6_enc_session_data *self, s32 session_id,
			struct cad_open_struct_type *open_param)
{
	if (self->session_state != Q6_ENC_STATE_RESET) {
		pr_err("wrong state to open read session! state: %d\n",
			self->session_state);
		return CAD_RES_FAILURE;
	}

	/* delay creation of the buffers until IOCTL STREAM CONFIG */
	/* register async callback for event handle */
	if (cad_rpc_reg_callback(session_id,
		cad_q6enc_session_handle_async_evt, self) != CAD_RES_SUCCESS)
		return CAD_RES_FAILURE;

	self->session_id = session_id;
	self->group_id = open_param->group_id;
	self->signal_buf_done = 0;
	self->signal_all_buf_done = 0;
	self->need_flush = 0;
	self->cb_data.client_data = NULL;
	self->cb_data.callback = NULL;
	D("Session open successful\n");
	return CAD_RES_SUCCESS;
}



s32 cad_q6enc_session_ioctl(struct q6_enc_session_data *self, u32 cmd,
			 void *cmd_buf, u32 cmd_buf_len)
{
	struct cad_event_struct_type		*cb_struct;
	union adsp_audio_event			ret_status;
	struct adsp_audio_no_payload_command	q6_cmd;
	s32					res = CAD_RES_SUCCESS;

	switch (cmd) {
	case CAD_IOCTL_CMD_STREAM_PAUSE:
	case CAD_IOCTL_CMD_SESSION_PAUSE:
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_SESSION_PAUSE;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		res = cad_rpc_control(self->session_id, self->group_id,
			(void *)&q6_cmd, sizeof(q6_cmd), &ret_status);
	case CAD_IOCTL_CMD_STREAM_RESUME:
	case CAD_IOCTL_CMD_SESSION_RESUME:
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_SESSION_RESUME;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		res = cad_rpc_control(self->session_id, self->group_id,
			(void *)&q6_cmd, sizeof(q6_cmd), &ret_status);
	case CAD_IOCTL_CMD_SET_STREAM_EVENT_LSTR:
		/* register the session callback function */
		if (cmd_buf == NULL) {
			pr_err("Invalid buffer passed to encoder ioctl\n");
			break;
		}
		if (cmd_buf_len != sizeof(*cb_struct)) {
			pr_err("Buf len dosen't match cad_event_struct_type"
				" for encoder LSTR event\n");
		}

		cb_struct = (struct cad_event_struct_type *)cmd_buf;
		if ((cb_struct->callback == NULL) ||
			(cb_struct->client_data == NULL)) {

			pr_err("We can not set listener function\n");
			break;
		}
		self->cb_data.client_data = cb_struct->client_data;
		self->cb_data.callback = cb_struct->callback;
		break;
	case CAD_IOCTL_CMD_STREAM_START:
		if ((self->session_state != Q6_ENC_STATE_INIT) ||
			(self->free_nodes == NULL)) {

			if (self->session_state != Q6_ENC_STATE_VOICE)
				pr_err("CAD:Q6ENC ===> can't start in wrong "
					"state!,  state: %d\n",
					self->session_state);
			break;
		}
		/* start to push the read buffers */
		res = send_buffers(self);
		if (res != CAD_RES_SUCCESS)
			pr_err("!!!!!!Problems in pushing the read buffer!\n");

		/* goes to process state no matter if there is problem or not */
		self->session_state = Q6_ENC_STATE_PROCESS;
		self->need_flush = 1;
		break;
	case CAD_IOCTL_CMD_SET_STREAM_INFO:
		/* for voice call, we don't need q6 encoder */
		if (((struct cad_stream_info_struct_type *)cmd_buf)->app_type
			== CAD_STREAM_APP_VOICE) {

			D("ignore stream info for voice call\n");
			self->session_state = Q6_ENC_STATE_VOICE;
			break;
		}

		if (cmd_buf == NULL) {
			pr_err("Invalid buffer passed to encoder ioctl\n");
			break;
		}
		res = create_buffers(self, cmd_buf);
		if (res == CAD_RES_SUCCESS)
			self->session_state = Q6_ENC_STATE_INIT;
		break;
	default:
		break;
	}
    return res;
}

s32 cad_q6enc_session_read(struct q6_enc_session_data *self,
			struct cad_buf_struct_type *read_buf)
{
	struct q6_enc_session_buf_node *node = NULL;

	init_completion(&self->buf_done_compl);

	mutex_lock(&self->close_mutex);
	mutex_lock(&self->session_mutex);
	if (self->session_state != Q6_ENC_STATE_PROCESS) {
		pr_err("wrong state to handle read! state: %d\n",
			self->session_state);
		mutex_unlock(&self->session_mutex);
		mutex_unlock(&self->close_mutex);
		return CAD_RES_FAILURE;
	}

	if (self->full_nodes_head == NULL)
		self->signal_buf_done = 1;

	mutex_unlock(&self->session_mutex);

	if (self->signal_buf_done) {
		D("Waiting for new buffer......\n");
		wait_for_completion(&self->buf_done_compl);
	}

	/* now we will have some buffers in the full node list */
	mutex_lock(&self->session_mutex);
	if (self->session_state != Q6_ENC_STATE_PROCESS) {
		pr_err("wrong state to handle read!\n");
		mutex_unlock(&self->session_mutex);
		mutex_unlock(&self->close_mutex);
		return CAD_RES_FAILURE;
	}

	self->signal_buf_done = 0;

	if (self->full_nodes_head == NULL) {
		D("No Data to Read!\n");
		mutex_unlock(&self->session_mutex);
		mutex_unlock(&self->close_mutex);
		return CAD_RES_FAILURE;
	}

	node = self->full_nodes_head;
	if (read_buf->max_size < node->buf_len) {
		/* copy data from buffer to user data. */
		if (copy_to_user(read_buf->buffer, node->buf, node->buf_len))
			pr_err("Error: Could not copy record data into user "
			"buffer\n");

		read_buf->actual_size = read_buf->max_size;
		node->buf_len -= read_buf->max_size;
		node->read_ptr += read_buf->max_size;
		D("CAD:Q6ENC1 ===> READ %d bytes of data\n",
			read_buf->max_size);
		mutex_unlock(&self->session_mutex);
	} else {
		/* Copy the entire node and remove the node from the list*/
		self->full_nodes_head = self->full_nodes_head->next;
		if (self->full_nodes_head == NULL)
			self->full_nodes_tail = NULL;

		node->next = NULL;

		/* copy data from buffer to user data. */
		if (copy_to_user(read_buf->buffer, node->buf, node->buf_len))
			pr_err("Error: Could not copy record data into user"
				"buffer\n");

		read_buf->actual_size = node->buf_len;
		D("CAD:Q6ENC2 ===> READ %d bytes of data", node->buf_len);
		node->buf_len = 0;
		node->next = self->free_nodes;
		self->free_nodes = node;
		mutex_unlock(&self->session_mutex);

		send_buffers(self);
	}
	mutex_unlock(&self->close_mutex);
	return CAD_RES_SUCCESS;
}

s32 cad_q6enc_session_close(struct q6_enc_session_data *self)
{
	union adsp_audio_event			ret_status;
	struct adsp_audio_no_payload_command	q6_cmd;

	init_completion(&self->all_buf_done_compl);
	mutex_lock(&self->session_mutex);
	if (self->session_state == Q6_ENC_STATE_CLOSING) {
		pr_err("CAD:Q6ENC ===> Session already closing,"
			"no need to close.\n");
		mutex_unlock(&self->session_mutex);
		return CAD_RES_FAILURE;
	}
	self->session_state = Q6_ENC_STATE_CLOSING;
	mutex_unlock(&self->session_mutex);

	if ((self->session_state != Q6_ENC_STATE_VOICE) && self->need_flush) {
		q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_CMD_STREAM_STOP;
		q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

		cad_rpc_control(self->session_id, self->group_id,
			(void *)&q6_cmd, sizeof(q6_cmd), &ret_status);
	}

	mutex_lock(&self->close_mutex);
	mutex_lock(&self->session_mutex);
	if (self->used_nodes) {
		D("CAD:Q6ENC ===> enable all buf done signal");
		self->signal_all_buf_done = 1;
	}
	mutex_unlock(&self->session_mutex);

	if (self->signal_all_buf_done) {
		D("Wait for all buf gets returned\n");
		wait_for_completion(&self->all_buf_done_compl);
	}

	/* deregister the callback function */
	cad_rpc_dereg_callback(self->session_id,
		cad_q6enc_session_handle_async_evt);

	mutex_lock(&self->session_mutex);
	self->signal_buf_done = 0;
	self->signal_all_buf_done = 0;
	/* release all buffers */
	release_buffers(&self->full_nodes_head);
	self->full_nodes_tail = NULL;
	release_buffers(&self->used_nodes);
	release_buffers(&self->free_nodes);

	if (self->shared_buffer)
		self->shared_buffer = NULL;

	self->session_id = 0;
	self->session_state = Q6_ENC_STATE_RESET;
	self->buf_size = 0;
	self->need_flush = 0;
	mutex_unlock(&self->session_mutex);
	mutex_unlock(&self->close_mutex);
	D("Encode session close successful\n");

	return CAD_RES_SUCCESS;
}

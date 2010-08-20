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

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/completion.h>

#include <mach/qdsp6/msm8k_adsp_audio_command.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_rpc_type.h>
#include <mach/dal.h>


/* this will be replace by include file daldeviceid.h later*/
#define DALDEVICEID_AUDIO_QDSP          0x02000028

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

/* rpc table index */
enum {
	ADSP_RPC_CONTROL = DALDEVICE_FIRST_DEVICE_API_IDX,
	ADSP_RPC_DATA,
	ADSP_RPC_INIT
};

struct callback_function_node;
struct callback_function_node {
	RPC_CB_FCN			cb_funct;
	void				*client_data;
	struct callback_function_node	*next;
};

struct cad_rpc_data_struct {
	u32				processor_id;
	struct mutex                    resource_mutex;
	struct mutex                    rpc_cb_mutex;
	u32                             initialized;
	void				*remote_handle;

	/* 0 = async wakeup event, 1...CAD_MAX_SESSION = sync wakeup evnt*/
	struct completion		compl_list[CAD_MAX_SESSION];

	/* sync mutex for each session call */
	struct mutex			remote_mutex_list[CAD_MAX_SESSION];

	union adsp_audio_event		sync_evt_queue[CAD_MAX_SESSION];


	struct callback_function_node   *dal_callback_list[CAD_MAX_SESSION];
};

static struct cad_rpc_data_struct cad_rpc_data_type;

static s32 cad_rpc_async_callback(union adsp_audio_event *evt)
{
	struct callback_function_node	*node = NULL;

	D("<-------------- ARM Async callback function fired.......\n");
	D("Get new async event(0x%x), handle: %d!!!!!!\n",
		evt->buffer.buffer.buffer_addr,
		evt->no_payload.dest.minor);

	mutex_lock(&cad_rpc_data_type.rpc_cb_mutex);
	node = cad_rpc_data_type.dal_callback_list[evt->
		no_payload.dest.minor];

	while (node) {
		node->cb_funct(evt, node->client_data);
		node = node->next;
	}
	mutex_unlock(&cad_rpc_data_type.rpc_cb_mutex);

	return CAD_RES_SUCCESS;
}

static void remote_cb_function(void *context, u32 param,
				void *evt_buf, u32 len)
{
	struct cad_rpc_data_struct	*self = context;
	union adsp_audio_event		*evt = evt_buf;

	if (evt->no_payload.dest.domain !=
		cad_rpc_data_type.processor_id) {

		D("CAD:RPC invalid domain: %d\n",
			evt->no_payload.dest.domain);
		return;
	}

	if ((evt->no_payload.dest.minor >=
		CAD_MAX_SESSION)) {

		pr_err("CAD:RPC invalid minor number: %d\n",
			evt->no_payload.dest.minor);
		return;
	}

	switch (evt->no_payload.event_data.response_type) {
	case ADSP_AUDIO_RESPONSE_ASYNC:
		/* async event */
		D("<-------CB: get async event!!!\n");
		if (cad_rpc_async_callback(evt) != CAD_RES_SUCCESS)
			pr_err("Async callback not fired for session %d\n",
				evt->no_payload.dest.minor);
		break;

	case ADSP_AUDIO_RESPONSE_COMMAND:
		/* sync event */
		memcpy(&self->sync_evt_queue[evt->no_payload.dest.minor],
			evt, sizeof(*evt));

		/* now wake up the blocking thread */
		complete(&cad_rpc_data_type.compl_list
			[evt->no_payload.dest.minor]);
	}
	return;
}


static s32 cad_rpc_get_remote_handle(void)
{
	s32				err = CAD_RES_SUCCESS;
	struct cad_rpc_config_info	info;

	mutex_lock(&cad_rpc_data_type.resource_mutex);
	if (!cad_rpc_data_type.initialized) {
		pr_err("RPC data is not initialized\n");
		mutex_unlock(&cad_rpc_data_type.resource_mutex);
		return CAD_RES_FAILURE;
	}

	/* already exist the handle */
	if (cad_rpc_data_type.remote_handle) {
		mutex_unlock(&cad_rpc_data_type.resource_mutex);
		return CAD_RES_SUCCESS;
	}

	/* get device handle */
	/* <peter> N-way SMD dependent.*/
	err = daldevice_attach(DALDEVICEID_AUDIO_QDSP,
		"DAL_AQ_AUD",
		1/*DALRPC_DEST_QDSP*/,
		&(cad_rpc_data_type.remote_handle));
	if (err) {
		pr_err("RPC call to Q6 attach failed\n");
		mutex_unlock(&cad_rpc_data_type.resource_mutex);
		return err;
	}

	D("Attached for session %d!\n", session_id);
	/*  get physical addresss of the buffer */
	memset(&info, 0, sizeof(struct cad_rpc_config_info));
	info.domain_id = cad_rpc_data_type.processor_id;
	info.cb_evt = dalrpc_alloc_cb(cad_rpc_data_type.
		remote_handle,
		&remote_cb_function, &cad_rpc_data_type);

	D("Try to configure the remote session %d!\n", session_id);
	/* initlize the rpc call */
	err = dalrpc_fcn_5(ADSP_RPC_INIT,
		cad_rpc_data_type.remote_handle,
		(void *)&info,
		sizeof(struct cad_rpc_config_info));
	D("Configured remote session %d!\n", session_id);
	mutex_unlock(&cad_rpc_data_type.resource_mutex);
	return err;
}

/* public functions */
s32 cad_rpc_init(u32 processor_id)
{
	u32 i;

	memset(&cad_rpc_data_type, 0, sizeof(cad_rpc_data_type));

	if (processor_id >= ADSP_AUDIO_MAX_DOMAIN)
		return CAD_RES_FAILURE;

	cad_rpc_data_type.processor_id = processor_id;

	/* create mutex for data resource */
	mutex_init(&cad_rpc_data_type.resource_mutex);
	mutex_init(&cad_rpc_data_type.rpc_cb_mutex);

	for (i = 0; i < CAD_MAX_SESSION; i++) {
		mutex_init(&cad_rpc_data_type.remote_mutex_list[i]);
		init_completion(&cad_rpc_data_type.compl_list[i]);
	}

	cad_rpc_data_type.initialized = 1;

	/* handle zero for async rpc dh */
	if (cad_rpc_get_remote_handle()) {
		pr_err("RPC failed to get Q6 remote handle\n");
		cad_rpc_deinit();
		return CAD_RES_FAILURE;
	}

	D("DALRPC Interface Initialized!!!\n");
	return CAD_RES_SUCCESS;
}


/* this function is called when everything is going away.*/
s32 cad_rpc_deinit()
{
	u32 i;
	struct callback_function_node  *node = NULL;

	D("cad deinit function fired   ....... \n");

	/* release remote device handle*/
	if (cad_rpc_data_type.remote_handle)
		daldevice_detach(cad_rpc_data_type.remote_handle);

	mutex_destroy(&cad_rpc_data_type.resource_mutex);
	mutex_destroy(&cad_rpc_data_type.rpc_cb_mutex);


	for (i = 0; i < CAD_MAX_SESSION; i++) {

		mutex_destroy(&cad_rpc_data_type.remote_mutex_list[i]);

		/* clean callback  function array */
		while (cad_rpc_data_type.dal_callback_list[i]) {
			node = cad_rpc_data_type.dal_callback_list[i];
			cad_rpc_data_type.dal_callback_list[i] =
			(cad_rpc_data_type.dal_callback_list[i])->next;
			kfree(node);
		}
	}

	/*free shared memory first, then close file handle */
	memset(&cad_rpc_data_type, 0, sizeof(cad_rpc_data_type));
	D("DeInit the rpc resource interface!!!!!\n");
	return CAD_RES_SUCCESS;
}

s32 cad_rpc_reg_callback(u32 stream_id, RPC_CB_FCN cbFCN, void *client_data)
{
	s32				err = CAD_RES_SUCCESS;
	struct callback_function_node	*node = NULL;


	if (!cbFCN && (stream_id >= CAD_MAX_SESSION))
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data_type.rpc_cb_mutex);
	if (cad_rpc_data_type.dal_callback_list[stream_id]) {
		/* check if duplicated callback registration */
		node = cad_rpc_data_type.dal_callback_list[stream_id];
		while (node) {
			if (node->cb_funct == cbFCN) {
				/* duplicated */
				err = CAD_RES_FAILURE;
				break;
			}
			node = node->next;
		}
	}
	if (err) {
		mutex_unlock(&cad_rpc_data_type.rpc_cb_mutex);
		return err;
	}
	node = kmalloc(sizeof(struct callback_function_node),
				GFP_KERNEL);
	if (IS_ERR(node)) {
		mutex_unlock(&cad_rpc_data_type.rpc_cb_mutex);
		return CAD_RES_FAILURE;
	}
	memset(node, 0, sizeof(struct callback_function_node));
	node->cb_funct = cbFCN;
	node->client_data = client_data;
	if (cad_rpc_data_type.dal_callback_list[stream_id]) {
		/* not first one */
		node->next = cad_rpc_data_type.dal_callback_list[stream_id];
		cad_rpc_data_type.dal_callback_list[stream_id] = node;
	} else {
		/* first one */
		cad_rpc_data_type.dal_callback_list[stream_id] = node;
	}

	mutex_unlock(&cad_rpc_data_type.rpc_cb_mutex);
	D("Registered the async callback function!!!\n");
	return CAD_RES_SUCCESS;
}

s32 cad_rpc_dereg_callback(u32 stream_id, RPC_CB_FCN cbFCN)
{
	struct callback_function_node	*node = NULL;
	struct callback_function_node	*prev = NULL;

	if (!cbFCN && stream_id >= CAD_MAX_SESSION)
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data_type.rpc_cb_mutex);
	node = cad_rpc_data_type.dal_callback_list[stream_id];
	while (node) {
		if (node->cb_funct == cbFCN) {
			if (prev == NULL) {
				/* first node */
				cad_rpc_data_type.dal_callback_list[stream_id] =
					node->next;
				kfree(node);
				break;
			}
			/* remove the none first node */
			prev->next = node->next;
			kfree(node);
			break;
		}
		prev = node;
		node = node->next;
	}
	mutex_unlock(&cad_rpc_data_type.rpc_cb_mutex);
	D("DeRegistered the async callback function!!!\n");
	return CAD_RES_SUCCESS;
}

/*===========================================================================*/
/* RPC Interface functions: */
/*===========================================================================*/
s32 cad_rpc_data(u32 stream_id,
		u32 group_id,
		void *data_buf,
		u32 data_buf_len,
		union adsp_audio_event *ret_evt)

{
	s32	err = CAD_RES_SUCCESS;

	if (stream_id >= CAD_MAX_SESSION || group_id >= CAD_MAX_SESSION ||
		!data_buf)

		return CAD_RES_FAILURE;

	((union adsp_audio_command *)data_buf)->no_payload.dest.domain =
		ADSP_AUDIO_ADDRESS_DOMAIN_DSP;
	((union adsp_audio_command *)data_buf)->no_payload.dest.service =
		ADSP_AUDIO_ADDRESS_SERVICE_AUDIO;
	((union adsp_audio_command *)data_buf)->no_payload.dest.major =
		(u8)group_id;
	((union adsp_audio_command *)data_buf)->no_payload.dest.minor =
		(u8)stream_id;

	((union adsp_audio_command *)data_buf)->no_payload.source.domain =
		(u8)cad_rpc_data_type.processor_id;
	((union adsp_audio_command *)data_buf)->no_payload.source.service =
		ADSP_AUDIO_ADDRESS_SERVICE_AUDIO;
	((union adsp_audio_command *)data_buf)->no_payload.source.major =
		(u8)group_id;
	((union adsp_audio_command *)data_buf)->no_payload.source.minor =
		(u8)stream_id;

	mutex_lock(&cad_rpc_data_type.remote_mutex_list[stream_id]);

	err = dalrpc_fcn_5(ADSP_RPC_DATA,
		cad_rpc_data_type.remote_handle,
		data_buf,
		data_buf_len);

	mutex_unlock(&cad_rpc_data_type.remote_mutex_list[stream_id]);

	if (err)
		pr_err("CAD:RPC Data rpc call returned err: %d\n", err);

	return err;
}


s32 cad_rpc_control(u32 stream_id,
		u32 group_id,
		void *cmd_buf,
		u32 cmd_buf_len,
		union adsp_audio_event *ret_evt)
{
	s32	err = CAD_RES_SUCCESS;


	if (stream_id >= CAD_MAX_SESSION || group_id >= CAD_MAX_SESSION ||
		!cmd_buf)

		return CAD_RES_FAILURE;

	((union adsp_audio_command *)cmd_buf)->no_payload.dest.domain =
		ADSP_AUDIO_ADDRESS_DOMAIN_DSP;
	((union adsp_audio_command *)cmd_buf)->no_payload.dest.service =
		ADSP_AUDIO_ADDRESS_SERVICE_AUDIO;
	((union adsp_audio_command *)cmd_buf)->no_payload.dest.major =
		(u8)group_id;
	((union adsp_audio_command *)cmd_buf)->no_payload.dest.minor =
		(u8)stream_id;

	((union adsp_audio_command *)cmd_buf)->no_payload.source.domain =
		(u8)cad_rpc_data_type.processor_id;
	((union adsp_audio_command *)cmd_buf)->no_payload.source.service =
		ADSP_AUDIO_ADDRESS_SERVICE_AUDIO;
	((union adsp_audio_command *)cmd_buf)->no_payload.source.major =
		(u8)group_id;
	((union adsp_audio_command *)cmd_buf)->no_payload.source.minor =
		(u8)stream_id;


	D("CAD:RPC Control stream (%d), Cmd(0x%x)\n",
		stream_id,
		((union adsp_audio_command *)cmd_buf)->no_payload.cmd.op_code);

	mutex_lock(&cad_rpc_data_type.remote_mutex_list[stream_id]);
	init_completion(&cad_rpc_data_type.compl_list[stream_id]);

	err = dalrpc_fcn_5(ADSP_RPC_CONTROL,
		cad_rpc_data_type.remote_handle,
		cmd_buf,
		cmd_buf_len);

	if (err)
		pr_err("CAD:RPC Data rpc call returned err: %d\n", err);

	if (!err && ((union adsp_audio_command *)cmd_buf)->no_payload.
		cmd.response_type == ADSP_AUDIO_RESPONSE_COMMAND) {

		D("DALRPC Open function start wait!!!\n");
		wait_for_completion(&cad_rpc_data_type.compl_list[stream_id]);
		if (ret_evt != NULL) {
			mutex_lock(&cad_rpc_data_type.resource_mutex);
			memcpy(ret_evt,
				&cad_rpc_data_type.sync_evt_queue[stream_id],
				sizeof(*ret_evt));
			mutex_unlock(&cad_rpc_data_type.resource_mutex);
		}
	}
	mutex_unlock(&cad_rpc_data_type.remote_mutex_list[stream_id]);

	D("CAD:RPC Control stream (%d), Cmd(0x%x) DONE!\n",
		stream_id,
		((union adsp_audio_command *)cmd_buf)->no_payload.cmd.op_code);

	return err;
}




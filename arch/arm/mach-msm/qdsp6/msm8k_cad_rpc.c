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

#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_rpc_type.h>
#include <mach/dal.h>


#define  SYNC_EVENT_CLEAN_UP_TIMEOUT      1000      /* timeout value in us */
#define  SYNC_EVENT_WAIT_TIMEOUT          50000000 /* timeout value in us */

/* this will be replace by include file daldeviceid.h later*/
#define DALDEVICEID_AUDIO_QDSP          0x02000028

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

/* rpc table index */
enum {
	CAD_RPC_Q6_OPEN = DALDEVICE_FIRST_DEVICE_API_IDX,
	CAD_RPC_Q6_WRITE,
	CAD_RPC_Q6_READ,
	CAD_RPC_Q6_IOCTL,
	CAD_RPC_Q6_INIT,
	CAD_RPC_Q6_CLOSE,
	CAD_RPC_Q6_FLUSH_BUF
};

struct callback_function_node;
struct callback_function_node {
	RPC_CB_FCN                      cb_funct;
	void                            *client_data;
	struct callback_function_node   *next;
};

struct async_evt_node {
	struct cadi_evt_struct_type	*evt;
	struct async_evt_node		*next;
};

struct cad_rpc_data_struct {
	enum cad_rpc_process_type       processor_id;
	struct mutex                    resource_mutex;
	u32                             initialized;
	void				*remote_handle_list[CAD_MAX_SESSION];

	/* 0 = async wakeup event, 1...CAD_MAX_SESSION = sync wakeup evnt*/
	struct completion		compl_list[CAD_MAX_SESSION];

	/* sync mutex for each session call */
	struct mutex			dal_remote_mutex_list[CAD_MAX_SESSION];

	struct cadi_evt_struct_type	sync_evt_queue[CAD_MAX_SESSION];


	struct callback_function_node   *dal_callback_list[CAD_MAX_SESSION];
};

static struct cad_rpc_data_struct cad_rpc_data;

static s32 cad_rpc_async_callback(struct cadi_evt_struct_type *evt)
{
	struct callback_function_node	*node = NULL;

	D("<-------------- ARM Async callback function fired.......\n");
	D("Get new async event(0x%x), handle: %d!!!!!!\n",
		evt->cad_event_data.buf_data.phys_addr,
		evt->cad_event_header.cad_handle);

	if ((evt->cad_event_header.cad_handle <= 0) ||
		(evt->cad_event_header.cad_handle >=
		CAD_MAX_SESSION)) {

		pr_err("%d not a valid can handle\n",
			evt->cad_event_header.cad_handle);
		return CAD_RES_FAILURE;
	}

	mutex_lock(&cad_rpc_data.resource_mutex);
	node = cad_rpc_data.dal_callback_list[evt->
		cad_event_header.cad_handle];

	while (node) {
		node->cb_funct(evt, node->client_data);
		node = node->next;
	}
	mutex_unlock(&cad_rpc_data.resource_mutex);

	return CAD_RES_SUCCESS;
}

static void remote_cb_function(void *context, u32 param,
				void *evt_buf, u32 len)
{
	struct cad_rpc_data_struct *self = context;
	struct cadi_evt_struct_type *evt = evt_buf;

	if (param == (u32)&self->compl_list[0]) {
		/* async event */
		D("<-------CB: get async event!!!\n");
		if (cad_rpc_async_callback(evt) != CAD_RES_SUCCESS)
			pr_err("Async callback not fired for session %d\n",
				evt->cad_event_header.cad_handle);
	} else {
		if (param != (u32)&self->compl_list
			[evt->cad_event_header.cad_handle]) {

			pr_err("wrong sync event received!!!\n");
			return;
		}
		/* sync event */
		mutex_lock(&cad_rpc_data.resource_mutex);
		memcpy(&self->sync_evt_queue[evt->cad_event_header.cad_handle],
			evt, sizeof(*evt));
		mutex_unlock(&cad_rpc_data.resource_mutex);
		complete((struct completion *)param);
	}
	return;
}


static s32 cad_rpc_get_remote_handle(u32 session_id)
{
	s32 err = CAD_RES_SUCCESS;
	struct cad_rpc_config_info info;
	if (session_id >= CAD_MAX_SESSION)
		return CAD_RES_FAILURE;

	memset(&info, 0, sizeof(struct cad_rpc_config_info));

	mutex_lock(&cad_rpc_data.resource_mutex);
	if (!cad_rpc_data.initialized) {
		pr_err("RPC data is not initialized\n");
		mutex_unlock(&cad_rpc_data.resource_mutex);
		return CAD_RES_FAILURE;
	}

	/* already exist the handle */
	if (cad_rpc_data.remote_handle_list[session_id]) {
		mutex_unlock(&cad_rpc_data.resource_mutex);
		return CAD_RES_SUCCESS;
	}

	/* get device handle */
	/* <peter> N-way SMD dependent.*/
	err = daldevice_attach(DALDEVICEID_AUDIO_QDSP,
		"DAL_AQ_AUD",
		1/*DALRPC_DEST_QDSP*/,
		&(cad_rpc_data.remote_handle_list[session_id]));
	if (err) {
		pr_err("RPC call to Q6 attach failed\n");
		mutex_unlock(&cad_rpc_data.resource_mutex);
		return err;
	}

	D("Attached for session %d!\n", session_id);
	/*  get physical addresss of the buffer */
	info.session_id = session_id;
	info.processor_id = cad_rpc_data.processor_id;
	info.cb_evt = dalrpc_alloc_cb(cad_rpc_data.
		remote_handle_list[session_id],
		remote_cb_function, &cad_rpc_data);
	info.local_trigger_evt = (u32) &cad_rpc_data.compl_list[session_id];

	D("Try to configure the remote session %d!\n", session_id);
	/* initlize the rpc call */
	err = dalrpc_fcn_5(CAD_RPC_Q6_INIT,
		cad_rpc_data.remote_handle_list[session_id],
		(void *)&info,
		sizeof(struct cad_rpc_config_info));
	D("Configured remote session %d!\n", session_id);
	mutex_unlock(&cad_rpc_data.resource_mutex);
	return err;
}

/* public functions */
s32 cad_rpc_init(u32 processor_id)
{
	u32 i;
	memset(&cad_rpc_data, 0, sizeof(cad_rpc_data));

	if (processor_id >= CAD_RPC_PROCESSPR_MAX)
		return CAD_RES_FAILURE;

	cad_rpc_data.processor_id = processor_id;

	/* create mutex for data resource */
	mutex_init(&cad_rpc_data.resource_mutex);

	for (i = 0; i < CAD_MAX_SESSION; i++) {
		mutex_init(&cad_rpc_data.dal_remote_mutex_list[i]);
		init_completion(&cad_rpc_data.compl_list[i]);
	}

	cad_rpc_data.initialized = 1;

	/* handle zero for async rpc dh */
	if (cad_rpc_get_remote_handle(0)) {
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

	for (i = 0; i < CAD_MAX_SESSION; i++) {

		/* release remote device handle*/
		if (cad_rpc_data.remote_handle_list[i])
			daldevice_detach(cad_rpc_data.remote_handle_list[i]);

		/* clean callback  function array */
		while (cad_rpc_data.dal_callback_list[i]) {
			node = cad_rpc_data.dal_callback_list[i];
			cad_rpc_data.dal_callback_list[i] =
			(cad_rpc_data.dal_callback_list[i])->next;
			kfree(node);
		}
	}

	/*free shared memory first, then close file handle */

	memset(&cad_rpc_data, 0, sizeof(cad_rpc_data));
	D("DeInit the rpc resource interface!!!!!\n");
	return CAD_RES_SUCCESS;
}

s32 cad_rpc_reg_callback(u32 session_id, RPC_CB_FCN cbFCN, void *client_data)
{
	s32 err = CAD_RES_SUCCESS;
	struct callback_function_node  *node = NULL;
	if (!cbFCN && (session_id >= CAD_MAX_SESSION))
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data.resource_mutex);
	if (cad_rpc_data.dal_callback_list[session_id]) {
		/* check if duplicated callback registration */
		node = cad_rpc_data.dal_callback_list[session_id];
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
		mutex_unlock(&cad_rpc_data.resource_mutex);
		return err;
	}
	node = kmalloc(sizeof(struct callback_function_node),
				GFP_KERNEL);
	if (IS_ERR(node)) {
		mutex_unlock(&cad_rpc_data.resource_mutex);
		return CAD_RES_FAILURE;
	}
	memset(node, 0, sizeof(struct callback_function_node));
	node->cb_funct = cbFCN;
	node->client_data = client_data;
	if (cad_rpc_data.dal_callback_list[session_id]) {
		/* not first one */
		node->next = cad_rpc_data.dal_callback_list[session_id];
		cad_rpc_data.dal_callback_list[session_id] = node;
	} else {
		/* first one */
		cad_rpc_data.dal_callback_list[session_id] = node;
	}

	mutex_unlock(&cad_rpc_data.resource_mutex);
	D("Registered the async callback function!!!\n");
	return CAD_RES_SUCCESS;
}

s32 cad_rpc_dereg_callback(u32 session_id, RPC_CB_FCN cbFCN)
{
	struct callback_function_node *node = NULL;
	struct callback_function_node *prev = NULL;
	if (!cbFCN && session_id >= CAD_MAX_SESSION)
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data.resource_mutex);
	node = cad_rpc_data.dal_callback_list[session_id];
	while (node) {
		if (node->cb_funct == cbFCN) {
			if (prev == NULL) {
				/* first node */
				cad_rpc_data.dal_callback_list[session_id] =
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
	mutex_unlock(&cad_rpc_data.resource_mutex);
	D("DeRegistered the async callback function!!!\n");
	return CAD_RES_SUCCESS;
}

/*===========================================================================*/
/* RPC Interface functions: */
/*===========================================================================*/
s32 cad_rpc_open(u32 session_id,     /* session handle */
		u32 block_flag,     /* 0=none block, 1=block */
		struct cadi_open_struct_type *open_buf,
		struct cadi_evt_struct_type *ret_status)
{
	s32 err = CAD_RES_SUCCESS;
	if ((session_id == 0) || cad_rpc_get_remote_handle(session_id))
		return CAD_RES_FAILURE;

	if (!open_buf && !ret_status)
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data.dal_remote_mutex_list[session_id]);

	D("DALRPC Open function entering!!!\n");

	/* clean up any pending signal */

	/* making rpc open call*/
	err = dalrpc_fcn_5(CAD_RPC_Q6_OPEN,
		cad_rpc_data.remote_handle_list[session_id],
		(void *)open_buf,
		sizeof(struct cadi_open_struct_type));
	if (!err) {
		D("DALRPC Open function start wait!!!\n");
		init_completion(&cad_rpc_data.compl_list[session_id]);
		wait_for_completion(&cad_rpc_data.compl_list[session_id]);

		D("Got wakeup signal for Open blocking!!!\n");
		mutex_lock(&cad_rpc_data.resource_mutex);
		memset(ret_status, 0,
				sizeof(struct cadi_evt_struct_type));
		memcpy(ret_status,
				&cad_rpc_data.sync_evt_queue[session_id],
				sizeof(*ret_status));

		memset(&cad_rpc_data.sync_evt_queue[session_id], 0,
				sizeof(*ret_status));
		mutex_unlock(&cad_rpc_data.resource_mutex);
	}
	mutex_unlock(&cad_rpc_data.dal_remote_mutex_list[session_id]);
	return err;
}

s32 cad_rpc_read(u32 session_id,
			u32 block_flag,    /* none block, 1=block */
			struct cad_buf_struct_type *read_buf,
			struct cadi_evt_struct_type *ret_status)
{
	s32 err = CAD_RES_SUCCESS;

	if ((session_id == 0) || cad_rpc_get_remote_handle(session_id))
		return CAD_RES_FAILURE;

	if (!read_buf && !ret_status)
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data.dal_remote_mutex_list[session_id]);

	D("DALRPC READ function Entering!!!\n");

	/* making rpc read call*/
	err = dalrpc_fcn_5(CAD_RPC_Q6_READ,
			cad_rpc_data.remote_handle_list[session_id],
			(void *)read_buf,
			sizeof(struct cad_buf_struct_type));

	mutex_unlock(&cad_rpc_data.dal_remote_mutex_list[session_id]);

	return err;
}

s32 cad_rpc_write(u32 session_id,
			u32 block_flag,   /* 0=none block, 1=block */
			struct cad_buf_struct_type *write_buf,
			struct cadi_evt_struct_type *ret_status)
{
	s32 err = CAD_RES_SUCCESS;

	if ((session_id == 0) || cad_rpc_get_remote_handle(session_id))
		return CAD_RES_FAILURE;

	if (!write_buf && !ret_status)
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data.dal_remote_mutex_list[session_id]);

	D("DALRPC WRite function Entering!!!\n");

	/* making rpc write call */
	err = dalrpc_fcn_5(CAD_RPC_Q6_WRITE,
			cad_rpc_data.remote_handle_list[session_id],
			(void *)write_buf,
			sizeof(struct cad_buf_struct_type));

	mutex_unlock(&cad_rpc_data.dal_remote_mutex_list[session_id]);
	return err;
}

s32 cad_rpc_ioctl(u32 session_id,
			u32 block_flag,   /* 0=none block, 1=block */
			u32 cmd_code,
			u8  *cmd_buf,
			u32 cmd_buf_len,
			struct cadi_evt_struct_type *ret_status)
{
	s32 err = CAD_RES_SUCCESS;
	if ((session_id == 0) || cad_rpc_get_remote_handle(session_id))
		return CAD_RES_FAILURE;

	if (!cmd_buf && !ret_status)
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data.dal_remote_mutex_list[session_id]);

	D("DALRPC IOCTL function Entering!!!\n");

	/* clean up any delayed signal */

	err = dalrpc_fcn_6(CAD_RPC_Q6_IOCTL,
			cad_rpc_data.remote_handle_list[session_id],
			cmd_code,
			(void *)cmd_buf,
			cmd_buf_len);
	if (!err) {
		D("DALRPC IOCTL function start wait!!!\n");
		init_completion(&cad_rpc_data.compl_list[session_id]);
		wait_for_completion(&cad_rpc_data.compl_list[session_id]);

		D("Got wake up signal for IOCTL blocking!!!\n");
		mutex_lock(&cad_rpc_data.resource_mutex);
		memset(ret_status, 0,
				sizeof(struct cadi_evt_struct_type));
		memcpy(ret_status,
				&cad_rpc_data.sync_evt_queue[session_id],
				sizeof(*ret_status));
		memset(&cad_rpc_data.sync_evt_queue[session_id], 0,
				sizeof(*ret_status));
		mutex_unlock(&cad_rpc_data.resource_mutex);
	}
	mutex_unlock(&cad_rpc_data.dal_remote_mutex_list[session_id]);
	return err;
}

s32 cad_rpc_close(u32 session_id,
		u32 block_flag,   /* 0=none block, 1=block */
		struct cadi_evt_struct_type *ret_status)
{
	s32 err = CAD_RES_SUCCESS;
	if ((session_id == 0) || cad_rpc_get_remote_handle(session_id))
		return CAD_RES_FAILURE;

	if (!ret_status)
		return CAD_RES_FAILURE;

	mutex_lock(&cad_rpc_data.dal_remote_mutex_list[session_id]);

	D("DALRPC CLOSE function Entering!!!\n");

	/* clean up any delayed signal */

	/* making rpc close call */
	err = dalrpc_fcn_0(CAD_RPC_Q6_CLOSE,
			cad_rpc_data.remote_handle_list[session_id],
			session_id);
	if (!err) {
		D("DALRPC Close function start wait!!!\n");
		init_completion(&cad_rpc_data.compl_list[session_id]);
		wait_for_completion(&cad_rpc_data.compl_list[session_id]);

		D("Got wake up signal for Close blocking!!!\n");
		mutex_lock(&cad_rpc_data.resource_mutex);
		memset(ret_status, 0,
				sizeof(struct cadi_evt_struct_type));
		memcpy(ret_status,
				&cad_rpc_data.sync_evt_queue[session_id],
				sizeof(*ret_status));
		memset(&cad_rpc_data.sync_evt_queue[session_id], 0,
				sizeof(*ret_status));
		mutex_unlock(&cad_rpc_data.resource_mutex);
	}
	mutex_unlock(&cad_rpc_data.dal_remote_mutex_list[session_id]);
	return err;
}

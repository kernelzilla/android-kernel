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
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>

#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_ard_clk.h>
#include <mach/qdsp6/msm8k_ard_adie.h>
#include <mach/qdsp6/msm8k_cad_rpc_type.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_ard_q6.h>
#include <mach/qdsp6/msm8k_ard_helper.h>
#include <mach/qdsp6/msm8k_cad_q6dec_drvi.h>
#include <mach/qdsp6/msm8k_cad_q6enc_drvi.h>

static struct ard_session_info_struct_type	ard_session
							[ARD_AUDIO_MAX_CLIENT];
struct ard_session_info_struct_type		*ardsession[CAD_MAX_SESSION];
struct ard_state_struct_type			ard_state;

#define CAD_FORMAT_BLK_OFFSET	0x100000


#if 0
#define D(fmt, args...) printk(KERN_INFO "ARD: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

s32 cad_ard_init(struct cad_func_tbl_type **func_ptr_tbl)
{
	s32				rc, dal_rc, i;
	struct ard_state_struct_type	*local_ard_state = NULL;

	static struct cad_func_tbl_type ard_func = {
		ard_open, ard_close, ard_write, ard_read, ard_ioctl
	};

	rc = dal_rc = CAD_RES_SUCCESS;

	*func_ptr_tbl = &ard_func;
	local_ard_state = &ard_state;

	for (i = 0; i < MAX_NUM_DEVICES; i++) {
		mutex_init(&local_ard_state->ard_device[i].device_mutex);

		local_ard_state->ard_device[i].state = ARD_STATE_RESET;
		local_ard_state->ard_device[i].afe_enabled = ARD_FALSE;
		local_ard_state->ard_device[i].device_configured = ARD_FALSE;
		local_ard_state->ard_device[i].device_config_request
			= ARD_FALSE;
		local_ard_state->ard_device[i].device_change_request
			= ARD_FALSE;
		local_ard_state->ard_device[i].device_inuse = 0;
		local_ard_state->ard_device[i].device_type = CAD_RX_DEVICE;
		local_ard_state->ard_device[i].clk_configured = ARD_FALSE;
		local_ard_state->ard_device[i].stream_count = 0;
	}

	/* Set the default TX and RX devices to */

	/* Device 0 is RX Internal Codec */
	local_ard_state->ard_device[0].device_type = CAD_RX_DEVICE;

	/* Device 1 is TX Internal Codec */
	local_ard_state->ard_device[1].device_type = CAD_TX_DEVICE;

	local_ard_state->def_tx_device = CAD_HW_DEVICE_ID_HANDSET_MIC;
	local_ard_state->def_rx_device = CAD_HW_DEVICE_ID_HANDSET_SPKR;

	/* Create the session mutexes */
	for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++)
		mutex_init(&ard_session[i].session_mutex);

	/* Create a mutex for the state machine */
	mutex_init(&local_ard_state->ard_state_machine_mutex);

	/* Initialize the ADIE */
	dal_rc = adie_init();
	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD ard_init failed\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

	dal_rc = cad_rpc_init(CAD_RPC_ARM11);
	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD RPC Interface Attach Init failed\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

	/* Register ARD Callback function */
	dal_rc = cad_rpc_reg_callback(0, ard_callback_func, NULL);
	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD RPC Register Callback failed\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

done:
	return rc;
}

s32 cad_ard_dinit(void)
{
	s32				rc, dal_rc, i;
	struct ard_state_struct_type	*local_ard_state = NULL;

	rc = dal_rc = CAD_RES_SUCCESS;


	local_ard_state = &ard_state;

	for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++)
		mutex_unlock(&ard_session[i].session_mutex);

	for (i = 0; i < MAX_NUM_DEVICES; i++) {
		mutex_unlock(&local_ard_state->ard_device[i].device_mutex);
		local_ard_state->ard_device[i].state = ARD_STATE_RESET;
		local_ard_state->ard_device[i].afe_enabled = ARD_FALSE;
		local_ard_state->ard_device[i].device_configured = ARD_FALSE;
		local_ard_state->ard_device[i].device_config_request
			= ARD_FALSE;
		local_ard_state->ard_device[i].device_change_request
			= ARD_FALSE;
		local_ard_state->ard_device[i].device_inuse =
			CAD_HW_DEVICE_ID_DEFAULT_RX;
		local_ard_state->ard_device[i].device_type = CAD_RX_DEVICE;
		local_ard_state->ard_device[i].clk_configured = ARD_FALSE;
		local_ard_state->ard_device[i].stream_count = 0;
	}

	/* DeInit the ADIE */
	dal_rc = adie_dinit();
	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD Adie Interface DeAttach Init failed\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

	dal_rc = cad_rpc_deinit();
	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD RPC Interface DeAttach Init failed\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

	dal_rc = cad_rpc_dereg_callback(0, ard_callback_func);
	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD RPC DeRegister Callback failed\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

done:
	return rc;
}

s32 ard_open(s32 session_id, struct cad_open_struct_type *open_param)
{
	s32				i, rc, dal_rc;
	u32				open_parm_len;
	struct cadi_open_struct_type	*op = NULL;

	rc = dal_rc = CAD_RES_SUCCESS;
	open_parm_len = 0;

	D("ARD ard_open()\n");

	for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++) {
		if (ard_session[i].available == ARD_FALSE)
			break;
	}

	if (i == ARD_AUDIO_MAX_CLIENT) {
		pr_err("ARD session not available %d\n", session_id);
		rc = CAD_RES_FAILURE;
		goto done;
	}

	if (open_param == NULL) {
		pr_err("ARD NULL open_param sent.. ses: %d\n", session_id);
		rc = CAD_RES_FAILURE;
		goto done;
	}

	/* Parse the open parameters and check if this is a device control */
	/* session or a stream based session */
	if (open_param->op_code == CAD_OPEN_OP_DEVICE_CTRL)
		ard_session[i].session_type = DEVICE_CTRL_TYPE;
	else if ((open_param->op_code == CAD_OPEN_OP_WRITE) ||
		(open_param->op_code == CAD_OPEN_OP_READ))
		ard_session[i].session_type = STREAM_TYPE;
	else {
		pr_err("ARD unknown session type %d\n", session_id);
		rc = CAD_RES_FAILURE;
		goto done;
	}

	ard_session[i].sess_open_info =
		kmalloc(sizeof(struct cadi_open_struct_type),
			GFP_KERNEL);

	if (ard_session[i].sess_open_info == NULL) {
		pr_err("ARD open kmalloc failed\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

	op = (struct cadi_open_struct_type *)
			ard_session[i].sess_open_info;

	memset(op, 0, sizeof(struct cadi_open_struct_type));

	open_parm_len = sizeof(struct cad_open_struct_type);

	memcpy(&op->cad_open, open_param, open_parm_len);

	/* Init the qdsp6_opened flag */
	ard_session[i].qdsp6_opened = ARD_FALSE;
	ard_session[i].qdsp6_started = ARD_FALSE;

	/* Init the local format block ptr */
	ard_session[i].local_format_block = NULL;

	ardsession[session_id] = &ard_session[i];

	mutex_lock(&ardsession[session_id]->session_mutex);

	/* Commit session */
	ard_session[i].enabled = ARD_TRUE;
	ard_session[i].available = ARD_TRUE;

	mutex_unlock(&ardsession[session_id]->session_mutex);

	D("ARD Opened session_id %d, sess_opn_info(cadr) = %p\n",
		session_id, op);

	print_data(session_id);

done:

	return rc;
}


s32 ard_close(s32 session_id)
{
	s32					rc, dal_rc;
	u32					dev_id, i, cad_device;
	struct cad_stream_device_struct_type	*cadr_strm_device = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_stream_config_struct_type	*cadr_config = NULL;
	struct cad_stream_device_struct_type	*strm_dev = NULL;
	struct ard_state_struct_type            *local_ard_state = NULL;

	rc = dal_rc = CAD_RES_SUCCESS;


	local_ard_state = &ard_state;
	cadr = ardsession[session_id]->sess_open_info;
	strm_dev = &cadr->cad_device;

	D("ARD close ses_id %d, sess_opn_info(cadr) = %p, strm_dev = %p\n",
		session_id, cadr, strm_dev);

	print_data(session_id);

	/* Disable the session */
	ardsession[session_id]->enabled = ARD_FALSE;


	if (ardsession[session_id]->session_type != DEVICE_CTRL_TYPE) {
		for (i = 0; i < strm_dev->device_len; i++) {
			if (strm_dev->device[i] == CAD_HW_DEVICE_ID_DEFAULT_TX)
				cad_device = local_ard_state->def_tx_device;
			else if (strm_dev->device[i] ==
					CAD_HW_DEVICE_ID_DEFAULT_RX)
				cad_device = local_ard_state->def_rx_device;
			else
				/* not asking for default devices */
				cad_device = strm_dev->device[i];

			dev_id = get_device_id(cad_device);


			/* Lock the device data*/
			mutex_lock(&local_ard_state->ard_device[dev_id].
				device_mutex);

			/* Decrement the number of streams on this device */
			local_ard_state->ard_device[dev_id].stream_count--;

			if (local_ard_state->ard_device[dev_id].stream_count
				== 0) {

				mutex_lock(&local_ard_state->
					ard_state_machine_mutex);

				/* No more streams, so teardown device */
				local_ard_state->ard_device[dev_id].
					device_configured = ARD_FALSE;

				/* invoke the state m/c */
				rc = ard_state_control(session_id, dev_id);

				mutex_unlock(&local_ard_state->
					ard_state_machine_mutex);
			} else
				D("Ses Closed no teardown, ses %d, dev %d,"
						 "%d\n", session_id, dev_id,
					local_ard_state->ard_device[dev_id].
					stream_count);

			/* Release the device data*/
			mutex_unlock(&local_ard_state->ard_device[dev_id].
				device_mutex);
		}

		/* If device was torn down, session would've been removed */
		if (ardsession[session_id]->qdsp6_opened == ARD_TRUE) {
			/* No Device teardown, so close the QDSP6 session */
			rc = qdsp6_close(session_id);
			if (rc != CAD_RES_SUCCESS)
				pr_err("ARD DAL RPC CLOSE FAILED %d\n",
				session_id);
		}
	} else {
		rc = qdsp6_close(session_id);
		if (rc != CAD_RES_SUCCESS)
			pr_err("ARD DAL RPC OPEN FAILED %d\n", session_id);
	}

	/* Start Freeing up allocated session resources */
	/* Free the session open buffer and internal buffers */
	cadr_strm_device = &cadr->cad_device;
	cadr_config = &cadr->cad_config;
	cadr_stream = &cadr->cad_stream;

	cadr_strm_device->device = NULL;

	D("ARD Session %d, App Type = %d, Strm Ptr = %p\n",
		session_id, cadr_stream->app_type, cadr_stream);

	if (cadr_stream->app_type != CAD_STREAM_APP_VOICE) {
		if (ardsession[session_id]->local_format_block) {
			D("ARD Freeing PMEM Session %d, Ptr = %p\n",
				session_id,
				ardsession[session_id]->local_format_block);
			kfree(ardsession[session_id]->local_format_block);
		}
		ardsession[session_id]->local_format_block = NULL;
	}

	kfree(ardsession[session_id]->sess_open_info);
	ardsession[session_id]->sess_open_info = NULL;

	mutex_lock(&ardsession[session_id]->session_mutex);
	ardsession[session_id]->available = ARD_FALSE;
	mutex_unlock(&ardsession[session_id]->session_mutex);

	return rc;
}

s32 ard_ioctl(s32 session_id, u32 cmd_code, void *cmd_buf, u32 cmd_len)
{
	s32					rc, dal_rc;
	u32					dev_id, i, old_dev_stream_count;
	u32					cad_device;
	struct cad_stream_device_struct_type	*cadr_strm_device = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	struct cad_stream_config_struct_type	*cadr_config = NULL;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_device_struct_type		*def_device = NULL;
	struct cad_stream_device_struct_type	*strm_dev = NULL;
	struct ard_state_struct_type            *local_ard_state = NULL;
	void					*config_format_block = NULL;


	rc = dal_rc = CAD_RES_SUCCESS;

	D("ARD ard_IOCTL ses %d, cmd 0X%x\n", session_id, cmd_code);

	local_ard_state = &ard_state;

	switch (cmd_code) {
	case CAD_IOCTL_CMD_DEVICE_SET_GLOBAL_DEFAULT:
		/* Command sent to change global default TX or RX device */
		if (ardsession[session_id]->session_type != DEVICE_CTRL_TYPE) {
			/* Log Error and do nothing */
			pr_err("ARD IOCTL SET DEF DEV not supported Stm"
				" ses %d\n", session_id);
			break;
		}

		def_device = (struct cad_device_struct_type *)cmd_buf;

		/* Only one device can be setup at a given time */
		mutex_lock(&local_ard_state->ard_state_machine_mutex);

		/* Set the device requeste */
		if (def_device->reserved == CAD_RX_DEVICE) {
			dev_id = get_device_id
				(local_ard_state->def_rx_device);
			local_ard_state->new_rx_device
				= def_device->device;
		} else {
			dev_id = get_device_id
				(local_ard_state->def_tx_device);
			local_ard_state->new_tx_device
				= def_device->device;
		}


		if ((local_ard_state->ard_device[dev_id].device_inuse
			== def_device->device) ||
			(CAD_HW_DEVICE_ID_DEFAULT_TX ==
			def_device->device) ||
			(CAD_HW_DEVICE_ID_DEFAULT_RX ==
			def_device->device)) {

			/* Do nothing */
			D("ARD Device Change not required\n");
			mutex_unlock(&local_ard_state->
				ard_state_machine_mutex);
			break;
		}

		/* Grab Mutex and set Device In Use */
		mutex_lock(&local_ard_state->ard_device[dev_id].
			device_mutex);

		old_dev_stream_count = local_ard_state->
			ard_device[dev_id].stream_count;

		/* Teardown the current default device */
		local_ard_state->ard_device[dev_id].
			device_change_request = ARD_TRUE;
		local_ard_state->ard_device[dev_id].
			device_config_request = ARD_TRUE;

		/* Check if device needs setup/teardown */
		rc = ard_state_control(session_id, dev_id);


		/* Device Torn Down, clear stream count */
		local_ard_state->ard_device[dev_id].
			stream_count = 0;

		/* Release mutex */
		mutex_unlock(&local_ard_state->
			ard_device[dev_id].device_mutex);


		/* Set device to the default device and */
		/* get new dev_id */
		if (def_device->reserved == CAD_RX_DEVICE) {
			local_ard_state->def_rx_device =
				def_device->device;
			dev_id = get_device_id(
				local_ard_state->
				def_rx_device);

			/* Grab the Device Mutex and set */
			/* Device In Use */
			mutex_lock(&local_ard_state->ard_device
				[dev_id].device_mutex);

			/* set the stream device to the */
			/* requested device */
			local_ard_state->ard_device[dev_id].
				device_inuse =
				local_ard_state->def_rx_device;
		} else {
			local_ard_state->def_tx_device =
				def_device->device;
			dev_id = get_device_id(
				local_ard_state->
				def_tx_device);

			/* Grab the Device Mutex and set */
			/* Device In Use */
			mutex_lock(&local_ard_state->ard_device
				[dev_id].device_mutex);

			/* set the stream device to the */
			/* requested device */
			local_ard_state->ard_device[dev_id].
				device_inuse =
				local_ard_state->def_tx_device;
		}

		/* Begin new device setup */
		if ((device_needs_setup(local_ard_state->
			ard_device[dev_id].device_inuse)
			== ARD_TRUE) &&
			(valid_session_present() == ARD_TRUE)) {
			/* Setup for the new device */
			rc = ard_state_control(session_id,
				dev_id);
			local_ard_state->ard_device[dev_id].
				stream_count =
				old_dev_stream_count;
		} else {
			/* Either the Device is setup and */
			/* being used by stream(s) [OR] */
			/* there are no Streams present */
			/* at this time, so simply */
			/* communicate device change to QDSP6 */
			local_ard_state->ard_device[dev_id].
				stream_count +=
				old_dev_stream_count;

			D("Dev setup or no strms, dev %d\n",
				local_ard_state->ard_device[dev_id].
				device_inuse);

			/* Go ahead and open a Q6 Dev_Ctrl */
			/* Session & send the Q6 dev chg */
			/* notification Open QDSP6 session */
			rc = qdsp6_open(session_id);
			if (rc != CAD_RES_SUCCESS) {
				/* Log Error and do nothing */
				pr_err("Q6 OPEN FAILED %d\n",
					session_id);
				goto done;
			}

			rc = qdsp6_devchg_notify(session_id,
				dev_id);
			if (rc != CAD_RES_SUCCESS) {
				/* Log Error and do nothing */
				pr_err("Q6 DEV_CHG FAILED %d\n",
					session_id);
				qdsp6_close(session_id);
				goto done;
			}

			rc = qdsp6_standby(session_id);
			if (rc != CAD_RES_SUCCESS) {
				/* Log Error and do nothing */
				pr_err("Q6 STANDBY FAILED %d\n",
					session_id);
				qdsp6_close(session_id);
				goto done;
			}

			rc = qdsp6_start(session_id);
			if (rc != CAD_RES_SUCCESS) {
				/* Log Error and do nothing */
				pr_err("Q6 START FAILED %d\n",
					session_id);
				qdsp6_close(session_id);
				goto done;
			}
		}

		/* Release mutex */
		mutex_unlock(&local_ard_state->
			ard_device[dev_id].device_mutex);

		/* Release mutex */
		mutex_unlock(&local_ard_state->
			ard_state_machine_mutex);

		print_data(session_id);

		break;

	case CAD_IOCTL_CMD_STREAM_START:
		if (ardsession[session_id]->session_type != STREAM_TYPE) {
			/* Log Error and do nothing */
			D("STREAM_START called for device session"
				" ses: %d\n", session_id);
			rc = qdsp6_open(session_id);
			if (rc != CAD_RES_SUCCESS)
				pr_err("ARD DAL RPC OPEN FAILED %d\n",
					session_id);
			break;
		}

		/* Check if this stream session has any device setup */
		/* needs. Such as: Sampling Rate change, Devices */
		/* Requested if not, don't call the device setup */
		/* state m/c */
		cadr = ardsession[session_id]->sess_open_info;
		strm_dev = &(cadr->cad_device);

		D("ard_ioctl START cadr %p, strm_dev %p\n", cadr, strm_dev);

		D("ARD STREAM START IOCTL, sess %d, num devices = %d\n",
			session_id, strm_dev->device_len);

		for (i = 0; i < strm_dev->device_len; i++) {
			if (strm_dev->device[i]
				== CAD_HW_DEVICE_ID_DEFAULT_TX){

				cad_device = local_ard_state->
					def_tx_device;

			} else if (strm_dev->device[i]
				== CAD_HW_DEVICE_ID_DEFAULT_RX){

				cad_device = local_ard_state->
					def_rx_device;
			} else {
				/* not asking for default devices */
				cad_device = strm_dev->device[i];
			}

			dev_id = get_device_id(cad_device);

			/* Grab the Device mutex so that no updates */
			/* are allowed to the device data */
			mutex_lock(&local_ard_state->ard_device[dev_id].
				device_mutex);

			/* Increment stream counts for this device */
			local_ard_state->ard_device[dev_id].
				stream_count++;

			if ((check_sampling_rate() == ARD_TRUE) ||
				(device_needs_setup(cad_device)
				== ARD_TRUE)) {
				/* Session has a new sampling rate */
				/* requirement or has new device(s) */
				/* that need to be setup, so go */
				/* ahead and grab the Device Mutex */
				/* and setup Device requested */
				mutex_lock(&local_ard_state->
					ard_state_machine_mutex);

				/* set the stream device to the */
				/* requested device */
				local_ard_state->ard_device[dev_id].
					device_inuse = cad_device;

				rc = ard_state_control(session_id,
					dev_id);

				/* Release mutex */
				mutex_unlock(&local_ard_state->
					ard_state_machine_mutex);
			}

			/* Release the device mutex */
			mutex_unlock(&local_ard_state->ard_device[dev_id].
				device_mutex);

		}
		/* If Device needed setup, the Q6 would've been */
		/* Opened for this session if not, then go  */
		/* ahead and open & send the Q6 info */
		if (ardsession[session_id]->qdsp6_opened != ARD_TRUE) {
			/* Open QDSP6 session */
			rc = qdsp6_open(session_id);

			if (rc != CAD_RES_SUCCESS) {
				/* Log Error and do nothing */
				pr_err("ARD DAL RPC OPEN FAILED %d\n",
					session_id);
				goto done;
			}

			rc = qdsp6_standby(session_id);
			if (rc != CAD_RES_SUCCESS) {
				/* Log Error and do nothing */
				pr_err("ARD IOCTL STANDBY FAILED %d\n",
					session_id);
				qdsp6_close(session_id);
				goto done;
			}

			rc = qdsp6_start(session_id);
			if (rc != CAD_RES_SUCCESS) {
				/* Log Error and do nothing */
				pr_err("ARD IOCTL START FAILED %d\n",
					session_id);
				qdsp6_close(session_id);
				goto done;
			}
		}

		D("ard_ioctl STARTED ses %d, cadr = %p, strm_dev = %p\n",
			session_id, cadr, strm_dev);

		print_data(session_id);

		break;

	case CAD_IOCTL_CMD_SET_STREAM_DEVICE:

		/* At this point Cache/Store the stream Device ID info to
			send later to Q6 */
		cadr = ardsession[session_id]->sess_open_info;
		cadr_strm_device = &(cadr->cad_device);
		strm_dev = (struct cad_stream_device_struct_type *)cmd_buf;
		cadr_strm_device->device = (u32 *)(g_audio_mem + session_id *
			sizeof(struct cad_stream_device_struct_type) + 4096 +
			(Q6_ENC_BUF_PER_SESSION *
			Q6_ENC_BUF_MAX_SIZE + 4096) +
			(Q6_DEC_BUFFER_NUM_PER_STREAM *
			Q6_DEC_BUFFER_SIZE_MAX + 4096));

		/* Save the cmdbuff passed in */
		if ((cmd_buf != NULL) && (cadr_strm_device->device != NULL)) {
			memcpy(cadr_strm_device->device, strm_dev->device,
				(sizeof(u32) * strm_dev->device_len));
			cadr_strm_device->device_len = strm_dev->device_len;
		} else {
			/* Log Error and do nothing */
			pr_err("ARD IOCTL CMD STREAM DEVICE Memory is"
				" NULL %d\n", session_id);
			rc = CAD_RES_FAILURE;
		}

		D("ard_ioctl STRM DEV SET ses %d, cadr = %p, strm_dev = %p\n",
			session_id, cadr, cadr_strm_device);

		print_data(session_id);
		break;

	case CAD_IOCTL_CMD_SET_STREAM_INFO:
		/* Cache/Store the stream Device ID info to send later to Q6 */
		cadr = ardsession[session_id]->sess_open_info;
		cadr_stream = &(cadr->cad_stream);

		/* Save the cmdbuff passed in */
		if (cmd_buf != NULL) {
			memcpy(cadr_stream, cmd_buf, cmd_len);
		} else {
			/* Log Error and do nothing */
			pr_err("ARD IOCTL CMD STREAM INFO cmdBuff is NULL %d\n",
				session_id);
			rc = CAD_RES_FAILURE;
		}

		D("ard_ioctl STRM INFO SET ses %d, sess_opn_info(cadr) = %p\n",
			session_id, cadr);

		print_data(session_id);
		break;

	case CAD_IOCTL_CMD_SET_STREAM_CONFIG:
		cadr = ardsession[session_id]->sess_open_info;
		cadr_config = &(cadr->cad_config);

		cadr_config->format_block =
			(void *)(g_audio_base + CAD_FORMAT_BLK_OFFSET);
		cadr_config->format_block_len = cmd_len;

		config_format_block = (void *)(g_audio_mem
						+ CAD_FORMAT_BLK_OFFSET);
		memcpy(config_format_block, cmd_buf, cmd_len);

		D("ard_ioctl STRM CFG SET ses %d, sess_opn_info(cadr) = %p\n",
			session_id, cadr);

		print_data(session_id);
		break;

	default:
		/* Just silently succeed unrecognized IOCTLs. */
		break;
	}

done:
	return rc;
}


s32 ard_read(s32 session_id, struct cad_buf_struct_type *buf)
{
	return CAD_RES_SUCCESS;
}

s32 ard_write(s32 session_id, struct cad_buf_struct_type *buf)
{
	return CAD_RES_SUCCESS;
}

void ard_callback_func(struct cadi_evt_struct_type *ev_data, void *client_data)
{
}



enum ard_state_ret_enum_type ard_state_control(s32 session_id, u32 dev_id)
{
	enum ard_state_ret_enum_type	rc;
	struct ard_state_struct_type	*local_ard_state = NULL;

	rc = ARD_STATE_RC_SUCCESS;


	local_ard_state = &ard_state;

	do {
		D("ARD SM session_id %d, ard state %d\n",
			session_id, local_ard_state->ard_device[dev_id].state);

		switch (local_ard_state->ard_device[dev_id].state) {
		case ARD_STATE_RESET:
			rc = ard_state_reset(session_id, dev_id);
			break;
		case ARD_STATE_CLK_ACTIVE:
			rc = ard_state_clk_active(session_id, dev_id);
			break;
		case ARD_STATE_AFE_ACTIVE:
			rc = ard_state_afe_active(session_id, dev_id);
			break;
		case ARD_STATE_ACTIVE:
			rc = ard_state_active(session_id, dev_id);
			break;
		default:
			break;
		}
	} while (rc == ARD_STATE_RC_CONTINUE);

	return rc;
}



enum ard_state_ret_enum_type ard_state_reset(s32 session_id, u32 dev_id)
{
	enum ard_state_ret_enum_type	rc;
	struct ard_state_struct_type	*local_ard_state = NULL;

	rc = ARD_STATE_RC_SUCCESS;

	local_ard_state = &ard_state;

	if (valid_session_present()) {
		local_ard_state->ard_device[dev_id].state
			= ARD_STATE_CLK_ACTIVE;

		if (local_ard_state->ard_device[dev_id].clk_configured
			!= ARD_TRUE) {

			ard_clk_enable(dev_id);
			local_ard_state->ard_device[dev_id].clk_configured =
				ARD_TRUE;

			D("ARD - Setup: Enabled clocks dev_id %d\n", dev_id);
		} else
			D("ARD - Setup: CLKs Already Enabled, dev_id %d\n",
				dev_id);


		rc = ARD_STATE_RC_CONTINUE;
	}
	return rc;
}

enum ard_state_ret_enum_type ard_state_clk_active(s32 session_id, u32 dev_id)
{
	s32				res;
	enum ard_state_ret_enum_type	rc;
	enum codec_enum_type		codec_type;
	struct ard_state_struct_type	*local_ard_state = NULL;

	rc = ARD_STATE_RC_SUCCESS;
	codec_type = CODEC_INT;
	res = CAD_RES_SUCCESS;


	local_ard_state = &ard_state;

	codec_type = get_codec_type((u32)local_ard_state->ard_device[dev_id].
		device_inuse);

	if ((!valid_session_present()) ||
		(local_ard_state->ard_device[dev_id].clk_configured
		!= ARD_TRUE)) {

		D("ARD - Teardown: Disabling clocks, ses %d\n", session_id);
		ard_clk_disable(dev_id);
		local_ard_state->ard_device[dev_id].state = ARD_STATE_RESET;
		rc = ARD_STATE_RC_SUCCESS;
		goto done;
	}

	/* Ready to Open and Start Q6(enable AFE). It is */
	/* assumed that this will 0 fill PCM buffers */
	res = qdsp6_open(session_id);
	if (res == CAD_RES_FAILURE) {
		pr_err("ARD - Unable to Open QDSP6 in session %d\n",
			session_id);

		ard_clk_disable(dev_id);
		local_ard_state->ard_device[dev_id].state = ARD_STATE_RESET;
		rc = ARD_STATE_RC_SUCCESS;
		goto done;
	}

	res = qdsp6_standby(session_id);
	if (res == CAD_RES_FAILURE) {
		pr_err("ARD - Unable to STANDBY QDSP6 in session %d\n",
			session_id);
		qdsp6_close(session_id);
		ard_clk_disable(dev_id);

		local_ard_state->ard_device[dev_id].state = ARD_STATE_RESET;
		rc = ARD_STATE_RC_SUCCESS;
		goto done;
	}

	D("ARD - Setup: Standby Q6, ses %d\n", session_id);
	local_ard_state->ard_device[dev_id].afe_enabled = ARD_TRUE;
	local_ard_state->ard_device[dev_id].state = ARD_STATE_AFE_ACTIVE;
	rc = ARD_STATE_RC_CONTINUE;

done:
	return rc;
}

enum ard_state_ret_enum_type ard_state_afe_active(s32 session_id, u32 dev_id)
{
	s32				res;
	enum ard_state_ret_enum_type	rc;
	enum codec_enum_type		codec_type;
	struct ard_state_struct_type	*local_ard_state = NULL;

	rc = ARD_STATE_RC_SUCCESS;
	res = CAD_RES_SUCCESS;

	local_ard_state = &ard_state;

	if ((!valid_session_present()) || (local_ard_state->
		ard_device[dev_id].afe_enabled != ARD_TRUE)) {

		D("ARD - Teardown: Disabling Q6\n");
		qdsp6_close(session_id);
		local_ard_state->ard_device[dev_id].clk_configured = ARD_FALSE;
		local_ard_state->ard_device[dev_id].state
			= ARD_STATE_CLK_ACTIVE;
		rc = ARD_STATE_RC_CONTINUE;
		goto done;
	}

	codec_type = get_codec_type(local_ard_state->
		ard_device[dev_id].device_inuse);

	res = codec_enable(codec_type,
		(u32)local_ard_state->ard_device[dev_id].device_type,
		local_ard_state->ard_device[dev_id].device_inuse);

	if (res == CAD_RES_FAILURE) {
		/* Failed to setup Codec. Go back to previous state */
		pr_err("ARD Codec setup failed, state %d\n",
			local_ard_state->ard_device[dev_id].state);
		/* Disable QDSP6/AFE */
		qdsp6_close(session_id);
		local_ard_state->ard_device[dev_id].clk_configured = ARD_FALSE;
		/* Goto previous state */
		local_ard_state->ard_device[dev_id].state
			= ARD_STATE_CLK_ACTIVE;
		rc = ARD_STATE_RC_CONTINUE;
		goto done;
	}

	D("ARD - Setup: Enabled ADIE\n");
	local_ard_state->ard_device[dev_id].device_configured = ARD_TRUE;
	local_ard_state->ard_device[dev_id].state = ARD_STATE_ACTIVE;
	local_ard_state->ard_device[dev_id].device_config_request = ARD_FALSE;
	rc = ARD_STATE_RC_CONTINUE;

done:
	return rc;
}

enum ard_state_ret_enum_type ard_state_active(s32 session_id, u32 dev_id)
{
	s32				res;
	enum ard_state_ret_enum_type	rc;
	enum codec_enum_type		codec_type;
	struct ard_state_struct_type	*local_ard_state = NULL;

	rc = ARD_STATE_RC_SUCCESS;
	res = CAD_RES_SUCCESS;

	local_ard_state = &ard_state;
	codec_type = get_codec_type(local_ard_state->ard_device[dev_id].
		device_inuse);

	if ((!valid_session_present()) ||
		(local_ard_state->ard_device[dev_id].device_configured
		!= ARD_TRUE)) {

		D("ARD - Teardown: Disabling CODEC, ses %d\n", session_id);
		res = codec_disable(codec_type,
			(u32)local_ard_state->ard_device[dev_id].device_type,
			local_ard_state->ard_device[dev_id].device_inuse);

		local_ard_state->ard_device[dev_id].afe_enabled = ARD_FALSE;
		local_ard_state->ard_device[dev_id].state
			= ARD_STATE_AFE_ACTIVE;
		rc = ARD_STATE_RC_CONTINUE;
		goto done;
	}

	if (local_ard_state->ard_device[dev_id].device_config_request
		== ARD_TRUE) {

		/* see if ADIE needs to be torn down, w/o closing Q6 sessions */
		if (local_ard_state->ard_device[dev_id].device_change_request
			== ARD_TRUE) {
			D("Reconfig: Notify Q6, Disable ADIE & Clocks,"
				" ses %d\n", session_id);

			/* Notify Q6 of impending device change, payload */
			/* can contain new Device ID. Q6 will play out */
			/* buffers and perhaps 0 fill AFE*/
			qdsp6_devchg_notify(session_id, dev_id);

			/* Tear down codec for the current device in use */
			res = codec_disable(codec_type,
				local_ard_state->ard_device[dev_id].device_type,
				local_ard_state->ard_device[dev_id].
				device_inuse);

			local_ard_state->ard_device[dev_id].dsp_started
				= ARD_FALSE;

			/* Clocks need reconfig, so simply disable clocks */
			/* and goto reset state to start from beginning */
			ard_clk_disable(dev_id);
			local_ard_state->ard_device[dev_id].state
				= ARD_STATE_RESET;
			local_ard_state->ard_device[dev_id].
				device_change_request = ARD_FALSE;
			local_ard_state->ard_device[dev_id].device_configured
				= ARD_FALSE;
			local_ard_state->ard_device[dev_id].
				device_config_request = ARD_FALSE;
			local_ard_state->ard_device[dev_id].clk_configured
				= ARD_FALSE;
			local_ard_state->ard_device[dev_id].
				afe_enabled = ARD_FALSE;
			rc = ARD_STATE_RC_SUCCESS;
		} else {
			/* Do Nothing as Devices are not different */
			D("Device Config Request w new & old device"
				" for ses %d\n", session_id);
			local_ard_state->ard_device[dev_id].
				device_config_request = ARD_FALSE;
			rc = ARD_STATE_RC_SUCCESS;
		}
	} else {
		/* No reconfig request */
		D("ARD - No Device (re)config for this session %d\n",
			session_id);

		/* Start the Q6 */
		if (ardsession[session_id]->qdsp6_started != ARD_TRUE) {
			res = qdsp6_start(session_id);
			if (res != CAD_RES_FAILURE) {
				D("ARD-Setup: Started Q6, ses %d\n",
					session_id);
				if (ardsession[session_id]->session_type
					== STREAM_TYPE)

					ardsession[session_id]->qdsp6_started
					= ARD_TRUE;

				rc = ARD_STATE_RC_SUCCESS;
			} else {
				qdsp6_close(session_id);
				res = codec_disable(codec_type,
					local_ard_state->ard_device[dev_id].
					device_type,
					local_ard_state->ard_device[dev_id].
					device_inuse);
					local_ard_state->ard_device[dev_id].
					state = ARD_STATE_AFE_ACTIVE;
				rc = ARD_STATE_RC_CONTINUE;
			}
		}
	}

done:
	return rc;
}

enum ard_ret_enum_type valid_session_present(void)
{
	u8 i = 0;
	enum ard_ret_enum_type rc;

	rc = ARD_FALSE;

	for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++) {
		if (ard_session[i].enabled == ARD_TRUE) {
			if (ard_session[i].session_type != DEVICE_CTRL_TYPE) {
				rc = ARD_TRUE;
				break;
			}
		}
	}

	if (i == ARD_AUDIO_MAX_CLIENT) {
		rc = ARD_FALSE;
		D("ARD No Stream sessions\n");
	}

	return rc;
}

enum ard_ret_enum_type check_sampling_rate(void)
{
	return ARD_FALSE;
}

enum ard_ret_enum_type device_needs_setup(u32 cad_device)
{
	u32				dev_id;
	enum ard_ret_enum_type		rc;
	struct ard_state_struct_type	*local_ard_state = NULL;

	rc = ARD_TRUE;

	local_ard_state = &ard_state;

	for (dev_id = 0; dev_id < MAX_NUM_DEVICES; dev_id++) {
		if ((local_ard_state->ard_device[dev_id].device_configured) &&
			(local_ard_state->ard_device[dev_id].device_inuse ==
			cad_device)) {

			/* Device Is In Use and has been configured*/
			rc = ARD_FALSE;
			break;
		}
	}

	return rc;
}


void print_data(u32 session_id)
{
	struct cadi_open_struct_type            *cadr = NULL;
	struct cad_stream_info_struct_type      *cadr_stream = NULL;
	struct cad_stream_config_struct_type    *cadr_config = NULL;
	struct cad_stream_device_struct_type    *strm_dev = NULL;
	struct cad_open_struct_type             *cad_open = NULL;

	cadr = ardsession[session_id]->sess_open_info;

	cad_open = &(cadr->cad_open);
	D("ARD session_id %d cad_open->op_code %d\n", session_id,
		cad_open->op_code);
	D("ARD session_id %d cad_open->format %d\n", session_id,
		cad_open->format);

	cadr_config = &(cadr->cad_config);
	D("ARD session_id %d cadr_config %p\n", session_id, cadr_config);

	cadr_stream = &(cadr->cad_stream);
	D("ARD session_id %d cadr_stream->app_type %d\n",
		session_id, cadr_stream->app_type);
	D("ARD session_id %d cadr_stream->ses_buf_max_size %d\n",
		session_id, cadr_stream->ses_buf_max_size);
	D("ARD session_id %d cadr_stream->buf_mem_type %d\n",
		session_id, cadr_stream->buf_mem_type);
	D("ARD session_id %d cadr_stream->priority %d\n",
		session_id, cadr_stream->priority);

	strm_dev = &(cadr->cad_device);
	D("ARD session_id %d strm_dev->device %p\n", session_id,
		strm_dev->device);
	D("ARD session_id %d strm_dev->device_len %d\n", session_id,
		strm_dev->device_len);

}

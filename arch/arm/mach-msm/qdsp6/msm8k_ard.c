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
#include <mach/qdsp6/msm8k_ard_acdb.h>
#include <mach/qdsp6/msm8k_cad_q6dec_drvi.h>
#include <mach/qdsp6/msm8k_cad_q6enc_drvi.h>
#include <mach/qdsp6/msm8k_cad_write_pcm_format.h>
#include <mach/qdsp6/msm8k_cad_write_aac_format.h>
#include <mach/qdsp6/msm8k_adsp_audio_types.h>

static struct ard_session_info_struct_type	ard_session
							[ARD_AUDIO_MAX_CLIENT];
struct ard_session_info_struct_type		*ardsession[CAD_MAX_SESSION];
struct ard_state_struct_type			ard_state;
struct clk_info 				g_clk_info = {8000, 0};
u32						device_control_session;


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

	/* Initialize the sessions. */
	for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++)
		memset(&ard_session[i], 0,
			sizeof(struct ard_session_info_struct_type));

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

	local_ard_state->ard_device[2].device_type = CAD_RX_DEVICE;

	local_ard_state->ard_device[3].device_type = CAD_TX_DEVICE;

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

	dal_rc = ard_acdb_init();
	if (dal_rc != CAD_RES_SUCCESS) {
		pr_err("ARD ard_acdb_init failed\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

	dal_rc = cad_rpc_init(ADSP_AUDIO_ADDRESS_DOMAIN_APP);
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

	mutex_destroy(&local_ard_state->ard_state_machine_mutex);

	for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++)
		mutex_unlock(&ard_session[i].session_mutex);

	for (i = 0; i < MAX_NUM_DEVICES; i++) {
		mutex_unlock(&local_ard_state->ard_device[i].device_mutex);
		mutex_destroy(&local_ard_state->ard_device[i].device_mutex);
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
	ard_acdb_dinit();
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

	open_param->group_id = session_id;

	ard_session[i].group_id = open_param->group_id;

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
	if (ardsession[session_id]->enabled == ARD_FALSE)
		return rc;

	/*PCM recording specific change*/
	if (ardsession[session_id]->sess_open_info->cad_open.op_code ==
		CAD_OPEN_OP_READ && (
		ardsession[session_id]->sess_open_info->cad_open.format ==
		CAD_FORMAT_PCM ||
		ardsession[session_id]->sess_open_info->cad_open.format ==
		CAD_FORMAT_AAC)) {

		g_clk_info.open_rec_sessions -= 1;
		g_clk_info.tx_clk_freq = 8000;
	}

	local_ard_state = &ard_state;
	cadr = ardsession[session_id]->sess_open_info;
	strm_dev = &cadr->cad_device;

	D("ARD close ses_id %d, sess_opn_info(cadr) = %p, strm_dev = %p\n",
		session_id, cadr, strm_dev);

	print_data(session_id);


	if (ardsession[session_id]->session_type != DEVICE_CTRL_TYPE
		&& ardsession[session_id]->active == ARD_TRUE) {

		mutex_lock(&local_ard_state->ard_state_machine_mutex);

		/* Disable the session */
		ardsession[session_id]->enabled = ARD_FALSE;

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
			if (dev_id == CAD_HW_DEVICE_ID_INVALID) {
				pr_err("%s: unsupported device\n", __func__);
				return CAD_RES_FAILURE;
			}


			/* Lock the device data*/
			mutex_lock(&local_ard_state->ard_device[dev_id].
				device_mutex);

			/* Decrement the number of streams on this device */
			local_ard_state->ard_device[dev_id].stream_count--;

			if (local_ard_state->ard_device[dev_id].stream_count
				== 0) {


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

		mutex_unlock(&local_ard_state->ard_state_machine_mutex);

		/* If device was torn down, session would've been removed */
		if (ardsession[session_id]->qdsp6_opened == ARD_TRUE) {
			/* No Device teardown, so close the QDSP6 session */
			rc = qdsp6_close(session_id);
			if (rc != CAD_RES_SUCCESS)
				pr_err("ARD DAL RPC CLOSE FAILED %d\n",
				session_id);
		}
	} else if (ardsession[session_id]->session_type == DEVICE_CTRL_TYPE) {
		rc = qdsp6_close(session_id);
		if (rc != CAD_RES_SUCCESS)
			pr_err("ARD DAL RPC OPEN FAILED %d\n", session_id);
	}

	ardsession[session_id]->active = ARD_FALSE;

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
	u32					old_device, new_device;
	u16					sample_rate;
	u32					clk_freq;
	struct cad_stream_device_struct_type	*cadr_strm_device = NULL;
	struct cad_stream_info_struct_type	*cadr_stream = NULL;
	struct cad_stream_config_struct_type	*cadr_config = NULL;
	struct cadi_open_struct_type		*cadr = NULL;
	struct cad_device_struct_type		*def_device = NULL;
	struct cad_stream_device_struct_type	*strm_dev = NULL;
	struct ard_state_struct_type            *local_ard_state = NULL;
	struct cad_write_pcm_format_struct_type *pcm_format_struct;
	struct cad_write_aac_format_struct_type *aac_format_struct;

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

		if (dev_id == CAD_HW_DEVICE_ID_INVALID) {
			pr_err("%s: unsupported device\n", __func__);
			return CAD_RES_FAILURE;
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

		if (old_dev_stream_count != 0) {
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
		}

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
			if (dev_id == CAD_HW_DEVICE_ID_INVALID) {
				pr_err("%s: unsupported device\n", __func__);
				return CAD_RES_FAILURE;
			}

			/* Grab the Device Mutex and set */
			/* Device In Use */
			mutex_lock(&local_ard_state->ard_device
				[dev_id].device_mutex);

			local_ard_state->ard_device[dev_id].device_type
				= CAD_RX_DEVICE;
			/* set the stream device to the */
			/* requested device */
			local_ard_state->ard_device[dev_id].
				device_inuse =
				local_ard_state->new_rx_device;

			old_device = local_ard_state->def_rx_device;
		} else {
			local_ard_state->def_tx_device =
				def_device->device;
			dev_id = get_device_id(
				local_ard_state->
				def_tx_device);
			if (dev_id == CAD_HW_DEVICE_ID_INVALID) {
				pr_err("%s: unsupported device\n", __func__);
				return CAD_RES_FAILURE;
			}

			/* Grab the Device Mutex and set */
			/* Device In Use */
			mutex_lock(&local_ard_state->ard_device
				[dev_id].device_mutex);

			local_ard_state->ard_device[dev_id].device_type
				= CAD_TX_DEVICE;
			/* set the stream device to the */
			/* requested device */
			local_ard_state->ard_device[dev_id].
				device_inuse =
				local_ard_state->new_tx_device;

			old_device = local_ard_state->def_tx_device;
		}
		new_device = def_device->device;

		ard_acdb_send_cal(session_id, new_device, old_device);

		/* Begin new device setup */
		if ((device_needs_setup(local_ard_state->
			ard_device[dev_id].device_inuse)
			== ARD_TRUE) &&
			(valid_session_present(dev_id) == ARD_TRUE)) {
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

		/* New devices setup, so update def device. */
		if (def_device->reserved == CAD_RX_DEVICE)
			local_ard_state->def_rx_device =
					local_ard_state->new_rx_device;
		else
			local_ard_state->def_tx_device =
					local_ard_state->new_tx_device;



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
			else
				device_control_session = session_id;

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

		mutex_lock(&local_ard_state->ard_state_machine_mutex);

		for (i = 0; i < strm_dev->device_len; i++) {
			if (strm_dev->device[i] == CAD_HW_DEVICE_ID_DEFAULT_TX)
				cad_device = local_ard_state->def_tx_device;

			else if (strm_dev->device[i]
					== CAD_HW_DEVICE_ID_DEFAULT_RX)
				cad_device = local_ard_state->def_rx_device;
			else
				/* not asking for default devices */
				cad_device = strm_dev->device[i];

			dev_id = get_device_id(cad_device);
			if (dev_id == CAD_HW_DEVICE_ID_INVALID) {
				pr_err("%s: unsupported device\n", __func__);
				return CAD_RES_FAILURE;
			}

			/* Grab the Device mutex so that no updates */
			/* are allowed to the device data */
			mutex_lock(&local_ard_state->ard_device[dev_id].
				device_mutex);

			if ((check_sampling_rate() == ARD_TRUE) ||
				(device_needs_setup(cad_device) == ARD_TRUE)) {
				/* Session has a new sampling rate */
				/* requirement or has new device(s) */
				/* that need to be setup, so go */
				/* ahead and grab the Device Mutex */
				/* and setup Device requested */

				/* set the stream device to the */
				/* requested device */
				local_ard_state->ard_device[dev_id].
					device_inuse = cad_device;

				rc = ard_state_control(session_id,
					dev_id);

			}

			/* Release the device mutex */
			mutex_unlock(&local_ard_state->ard_device[dev_id].
				device_mutex);

		}

		/* Release mutex */
		mutex_unlock(&local_ard_state->ard_state_machine_mutex);

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

		/* We don't know which of the devices are default device.
		   Also, the stream can request more than one device */
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
			if (dev_id == CAD_HW_DEVICE_ID_INVALID) {
				pr_err("%s: unsupported device\n", __func__);
				return CAD_RES_FAILURE;
			}

			/* Grab the route mutex so that no updates are allowed
			   to the route data. */
			mutex_lock(&local_ard_state->ard_device[dev_id].
				device_mutex);

			ardsession[session_id]->active = ARD_TRUE;
			local_ard_state->ard_device[dev_id].stream_count++;

			mutex_unlock(&local_ard_state->ard_device[dev_id].
				device_mutex);
		}


		D("ard_ioctl STARTED ses %d, cadr = %p, strm_dev = %p\n",
			session_id, cadr, strm_dev);

		print_data(session_id);

		break;

	case CAD_IOCTL_CMD_SET_STREAM_DEVICE:

		/* At this point Cache/Store the stream Device ID info to
			send later to Q6 */
		if (ardsession[session_id]->session_type == DEVICE_CTRL_TYPE) {
			pr_err("ARD: recieved strm start for dev ctrl ses\n");
			rc = CAD_RES_FAILURE;
			goto done;
		}

		cadr = ardsession[session_id]->sess_open_info;
		cadr_strm_device = &(cadr->cad_device);

		if (cmd_buf == NULL) {
			pr_err("ARD bad value passed as stream device\n");
			rc = CAD_RES_FAILURE;
			goto done;
		}

		strm_dev = (struct cad_stream_device_struct_type *)cmd_buf;

		cadr_strm_device->device =
			kmalloc((sizeof(u32) * strm_dev->device_len),
			GFP_KERNEL);

		if (cadr_strm_device->device == NULL) {
			/* Log Error and do nothing */
			pr_err("ARD IOCTL CMD STREAM DEVICE Memory is"
				" NULL %d\n", session_id);
			rc = CAD_RES_FAILURE;
			goto done;
		}

		/* Save the cmdbuff passed in */
		memcpy(cadr_strm_device->device, strm_dev->device,
			(sizeof(u32) * strm_dev->device_len));
		cadr_strm_device->device_len = strm_dev->device_len;


		D("ard_ioctl STRM DEV SET ses %d, cadr = %p, strm_dev = %p\n",
			session_id, cadr, cadr_strm_device);

		if (strm_dev->device[0] == CAD_HW_DEVICE_ID_DEFAULT_TX)
			dev_id = ard_state.def_tx_device;
		else if (strm_dev->device[0] == CAD_HW_DEVICE_ID_DEFAULT_RX)
			dev_id = ard_state.def_rx_device;
		else
			dev_id = strm_dev->device[0];

		mutex_lock(&local_ard_state->ard_state_machine_mutex);
		ard_acdb_send_cal(session_id, dev_id, 0);
		mutex_unlock(&local_ard_state->ard_state_machine_mutex);
		print_data(session_id);
		break;

	case CAD_IOCTL_CMD_SET_STREAM_INFO:
		/* Cache/Store the stream Device ID info to send later to Q6 */
		cadr = ardsession[session_id]->sess_open_info;
		cadr_stream = &(cadr->cad_stream);

		/* Save the cmdbuff passed in */
		if (cmd_buf == NULL) {
			/* Log Error and do nothing */
			pr_err("ARD IOCTL CMD STREAM INFO cmdBuff is NULL %d\n",
				session_id);
			rc = CAD_RES_FAILURE;
		}

		memcpy(cadr_stream, cmd_buf, cmd_len);

		if (cadr_stream->app_type == CAD_STREAM_APP_VOICE &&
			ardsession[session_id]->sess_open_info->cad_open.op_code
			== CAD_OPEN_OP_READ) {

			if (g_clk_info.tx_clk_freq != 8000)
				for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++)
					if (ardsession[i] &&
						ardsession[i]->sess_open_info
						->cad_open.op_code
						== CAD_OPEN_OP_READ
						&& i != session_id)
						ard_close(i);

			g_clk_info.open_rec_sessions += 1;

		}

		D("ard_ioctl STRM INFO SET ses %d, sess_opn_info(cadr) = %p\n",
			session_id, cadr);

		print_data(session_id);
		break;

	case CAD_IOCTL_CMD_SET_STREAM_CONFIG:
		cadr = ardsession[session_id]->sess_open_info;
		cadr_config = &(cadr->cad_config);

		if (cmd_buf == NULL) {
			pr_err("ARD bad value passed as format block\n");
			rc = CAD_RES_FAILURE;
			goto done;
		}

		cadr_config->format_block = kmalloc(cmd_len, GFP_KERNEL);

		if (cadr_config->format_block == NULL) {
			pr_err("ARD format block allocation failed\n");
			rc = CAD_RES_FAILURE;
			goto done;
		}

		memcpy(cadr_config->format_block, cmd_buf, cmd_len);
		cadr_config->format_block_len = cmd_len;

		D("ard_ioctl STRM CFG SET ses %d, sess_opn_info(cadr) = %p\n",
			session_id, cadr);

		if ((ardsession[session_id]->sess_open_info->cad_open.format ==
			CAD_FORMAT_PCM ||
			ardsession[session_id]->sess_open_info->cad_open.format
			== CAD_FORMAT_AAC) &&
			ardsession[session_id]->sess_open_info->cad_open.op_code
			== CAD_OPEN_OP_READ) {

			if (ardsession[session_id]->sess_open_info->
				cad_open.format == CAD_FORMAT_PCM) {

				pcm_format_struct = cmd_buf;
				sample_rate = pcm_format_struct->
							pcm.us_sample_rate;
			} else {

				aac_format_struct = cmd_buf;
				sample_rate = aac_format_struct->
							aac.sample_rate;
			}

			switch (sample_rate) {

			case 0:
				clk_freq = 96000;
				break;
			case 1:
				clk_freq = 88200;
				break;
			case 2:
				clk_freq = 64000;
				break;
			case 3:
				clk_freq = 48000;
				break;
			case 4:
				clk_freq = 44100;
				break;
			case 5:
				clk_freq = 32000;
				break;
			case 6:
				clk_freq = 24000;
				break;
			case 7:
				clk_freq = 22050;
				break;
			case 8:
				clk_freq = 16000;
				break;
			case 9:
				clk_freq = 12000;
				break;
			case 10:
				clk_freq = 11025;
				break;
			case 11:
			default:
				clk_freq = 8000;
				break;
			}

			if (g_clk_info.open_rec_sessions > 0 &&
					g_clk_info.tx_clk_freq != clk_freq) {

				rc = CAD_RES_FAILURE;
				pr_err("clk mismatch with current recording\n");
				break;
			} else
				g_clk_info.tx_clk_freq = clk_freq;

			g_clk_info.open_rec_sessions += 1;

		}
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

void ard_callback_func(union adsp_audio_event *ev_data, void *client_data)
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

	if (valid_session_present(dev_id)) {
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

	if ((!valid_session_present(dev_id)) ||
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

	if ((!valid_session_present(dev_id)) || (local_ard_state->
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

	if (ardsession[session_id]->sess_open_info->cad_open.op_code ==
							CAD_OPEN_OP_WRITE)
		audio_resync_afe_clk();

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

	if ((!valid_session_present(dev_id)) ||
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

enum ard_ret_enum_type valid_session_present(u32 dev_id)
{
	u8 i = 0;
	enum ard_ret_enum_type rc;

	rc = ARD_FALSE;

	/* Check if there are any valid stream sessions. */
	switch (dev_id) {
	case 0:
	case 2:
	case 4:	/* A2DP */
	case 6:	/* I2S */
		for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++) {
			if ((ard_session[i].enabled == ARD_TRUE)
				&& (ard_session[i].sess_open_info->
					cad_open.op_code
						== CAD_OPEN_OP_WRITE)) {
				/* Valid RX stream exists. */
				rc = ARD_TRUE;
				break;
			}
		}
		break;
	case 1:
	case 3:
	case 5:	/* A2DP */
	case 7:	/* I2S */
		for (i = 0; i < ARD_AUDIO_MAX_CLIENT; i++) {
			if ((ard_session[i].enabled == ARD_TRUE)
				&& (ard_session[i].sess_open_info->
					cad_open.op_code
						== CAD_OPEN_OP_READ)) {
				/* Valid TX stream exists. */
				rc = ARD_TRUE;
				break;
			}
		}
		break;
	default:
		pr_err("valid_session_present(): bad device_id %d\n", dev_id);
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

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

#include <mach/dal.h>
#include <mach/qdsp6/msm8k_ard_acdb.h>
#include <mach/qdsp6/msm8k_ard_acdbi.h>
#include <mach/qdsp6/msm8k_ardi.h>
#include <mach/qdsp6/msm8k_ard_helper.h>
#include <mach/qdsp6/msm8k_adsp_audio_cfg_ioctl.h>
#include <mach/qdsp6/msm8k_adsp_audio_error.h>
#include <mach/qdsp6/msm8k_q6_api_flip_utils.h>
#include <mach/qdsp6/msm8k_ard_clk.h>


/* this is the ACDB device ID */
#define DALDEVICEID_ACDB		0x02000069
#define ACDB_PORT_NAME			"DAL_AM_AUD"
#define ACDB_CPU			SMD_APPS_MODEM

/* rpc table index */
enum {
	ACDB_DalACDB_ioctl = DALDEVICE_FIRST_DEVICE_API_IDX
};


/* this is the default acdb data buffer size */
/* Offset at last 4K of mem assigned to audio */
#define ARD_ACDB_DEFAULT_BUF_SIZE	4096
#define ARD_ACDB_BUF_OFFSET		0x7F000

/* this defines the sampel rate */
enum ard_acdb_sample_rate {
	ARD_ACDB_SR_INVALID	= 0,
	ARD_ACDB_SR_8K_HZ	= 8000,
	ARD_ACDB_SR_16K_HZ	= 16000,
	ARD_ACDB_SR_24K_HZ	= 24000,
	ARD_ACDB_SR_48K_HZ	= 48000,
	ARD_ACDB_SR_96K_HZ	= 96000
};


static void		*acdb_handle;
static void		*ard_acdb_buffer;


#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif



/* this function calcuate the sample rate for TX session*/
enum ard_acdb_sample_rate	ard_acdb_calculate_sample_rate(u32 session_id)
{
	enum ard_acdb_sample_rate	sample_rate = ARD_ACDB_SR_INVALID;

	if (g_clk_info.tx_clk_freq > ARD_ACDB_SR_16K_HZ)
		sample_rate = ARD_ACDB_SR_48K_HZ;
	else if (g_clk_info.tx_clk_freq > ARD_ACDB_SR_8K_HZ)
		sample_rate = ARD_ACDB_SR_16K_HZ;
	else
		sample_rate = ARD_ACDB_SR_8K_HZ;

	return sample_rate;
}


/* this is the current sample rate selection for cad device */
/* It will be revisited once we finalize the cad device definition*/
u32 ard_acdb_get_sample_rate(u32 session_id, u32 route_id)
{
	u32	sample_rate = ARD_ACDB_SR_INVALID;

	switch (route_id) {
	case 0:
	case 2:
	case 4: /* A2DP */
	case 5: /* A2DP */
	case 6: /* I2S RX */
		sample_rate = ARD_ACDB_SR_48K_HZ;
		break;
	case 1:
		sample_rate = ard_acdb_calculate_sample_rate(session_id);
		break;
	case 3:
		sample_rate = ARD_ACDB_SR_8K_HZ;
		break;
	/* I2S TX */
	case 7:
		sample_rate = ARD_ACDB_SR_16K_HZ;
		break;
	default:
		pr_err("CAD:ACDB==> Unsupported route_id %d\n", route_id);
		break;
	}
	D("CAD:ACDB=> device sampel rate is %d\n", sample_rate);
	return sample_rate;
}


/* this function is called in once to initialize the acdb interface */
s32 ard_acdb_init(void)
{
	s32 err = CAD_RES_SUCCESS;

	err = daldevice_attach(DALDEVICEID_ACDB, ACDB_PORT_NAME,
			ACDB_CPU, &acdb_handle);

	if (err) {
		pr_err("CAD:ACDB=> Device Attach failed\n");
		goto err_1;
	}

	/*initialize local cache */
	ard_acdb_buffer = g_audio_mem + ARD_ACDB_BUF_OFFSET;

	if (ard_acdb_buffer == NULL) {
		pr_err("CAD:ACDB=> Can not create buffers\n");
		goto err_2;
	}

	memset(ard_acdb_buffer, 0, sizeof(ard_acdb_buffer));
	return CAD_RES_SUCCESS;

err_2:
	daldevice_detach(acdb_handle);
	acdb_handle = NULL;
err_1:
	return CAD_RES_FAILURE;
}


void ard_acdb_dinit(void)
{
	/* release memory */
	if (ard_acdb_buffer)
		ard_acdb_buffer = NULL;

	/* dettach handle */
	if (acdb_handle) {
		daldevice_detach(acdb_handle);
		acdb_handle = NULL;
	}
}

/* this function sends the device id list to q6 */
/* we can not get the default tx/rx device id, */
/* the caller should do the translation */
s32   ard_acdb_send_cal(u32 session_id, u32 new_device, u32 old_device)
{
	struct acbd_cmd_device_table_struct		acdb_cmd;
	struct acdb_result_struct			acdb_cmd_result;
	struct adsp_audio_set_dev_cfg_table_command	q6_cmd;
	union adsp_audio_event				q6_cmd_result;
	u32						route_id;
	s32						err = CAD_RES_SUCCESS;

	if (acdb_handle == NULL) {
		pr_err("CAD:ACDB=> Device has not been initialized\n");
		return CAD_RES_FAILURE;
	}

	if (device_control_session == 0) {
		pr_err("CAD:ACDB=> Device control session does not exist\n");
		return CAD_RES_FAILURE;
	}

	if (new_device == CAD_HW_DEVICE_ID_DEFAULT_TX ||
		new_device == CAD_HW_DEVICE_ID_DEFAULT_RX ||
		old_device == CAD_HW_DEVICE_ID_DEFAULT_TX ||
		old_device == CAD_HW_DEVICE_ID_DEFAULT_RX) {

		pr_err("CAD:ACDB=> Don't know what is the default device\n");
		return CAD_RES_FAILURE;
	}

	/* query the device cal */
	memset(&acdb_cmd, 0, sizeof(acdb_cmd));

	acdb_cmd.command_id = ACDB_GET_DEVICE_TABLE;
	acdb_cmd.device_id = new_device;

	route_id = get_device_id(new_device);
	if (route_id == CAD_HW_DEVICE_ID_INVALID) {
		pr_err("CAD:ACDB=> Unknown devices %d\n", old_device);
		return CAD_RES_FAILURE;
	}

	if (ard_state.ard_device[route_id].stream_count > 0)
		acdb_cmd.sample_rate_id = ard_state.
			ard_device[route_id].device_sample_rate;
	else {
		ard_state.ard_device[route_id].device_sample_rate =
			ard_acdb_get_sample_rate(session_id, route_id);

		acdb_cmd.sample_rate_id =
			ard_state.ard_device[route_id].device_sample_rate;
	}

	if (acdb_cmd.sample_rate_id == ARD_ACDB_SR_INVALID) {
		pr_err("CAD:ACDB=> Can not select device sample rate\n");
		return CAD_RES_FAILURE;
	}

	acdb_cmd.total_bytes = ARD_ACDB_DEFAULT_BUF_SIZE;
	acdb_cmd.unmapped_buf = g_audio_base + ARD_ACDB_BUF_OFFSET;

	D("CAD:ACDB=>Query acdb(dev:%d, sample_rate: %d\n",
		acdb_cmd.device_id, acdb_cmd.sample_rate_id);

	/* query acdb */
	err = dalrpc_fcn_8(ACDB_DalACDB_ioctl, acdb_handle,
		(const void *)&acdb_cmd, sizeof(acdb_cmd),
		&acdb_cmd_result, sizeof(acdb_cmd_result));
	if (err) {
		pr_err("CAD:ACDB=> Can not query acdb\n");
		return CAD_RES_FAILURE;
	}

	if (acdb_cmd_result.result != ACDB_RES_SUCCESS) {
		pr_err("CAD:ACDB=> Failed to query the ACDB (%d)\n",
			acdb_cmd_result.result);
		return CAD_RES_FAILURE;
	}

	/* push the device cal to Q6 */
	memset(&q6_cmd, 0, sizeof(q6_cmd));

	if (ard_state.ard_device[route_id].device_type ==
		CAD_TX_DEVICE)
		q6_cmd.client_data.data = g_clk_info.tx_clk_freq;

	q6_cmd.cmd.op_code = ADSP_AUDIO_IOCTL_SET_DEVICE_CONFIG_TABLE;
	q6_cmd.cmd.response_type = ADSP_AUDIO_RESPONSE_COMMAND;

	q6_cmd.device_id = q6_device_id_mapping(acdb_cmd.device_id);
	q6_cmd.phys_mem.addr = acdb_cmd.unmapped_buf;
	q6_cmd.phys_mem.total = acdb_cmd.total_bytes;
	q6_cmd.phys_mem.used = acdb_cmd_result.used_bytes;

	D("CAD:ACDB=>Push cal data to Q6: %d\n", q6_cmd.device_id);

	err = cad_rpc_control(device_control_session,
		ardsession[device_control_session]->group_id,
		(void *)&q6_cmd, sizeof(q6_cmd), &q6_cmd_result);

	if (err) {
		pr_err("CAD:ACDB=> Can not push the cal data to Q6\n");
		return CAD_RES_FAILURE;
	}

	if ((q6_cmd_result.no_payload.status != ADSP_AUDIO_SUCCESS) &&
		(q6_cmd_result.no_payload.status !=
			ADSP_AUDIO_EALREADYLOADED)) {
		pr_err("CAD:ACDB=> Can not push the cal data to Q6(%d)\n",
			q6_cmd_result.no_payload.status);
		return CAD_RES_FAILURE;
	}
	return CAD_RES_SUCCESS;
}



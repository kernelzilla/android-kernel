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

#include <linux/err.h>
#include <asm/mach-types.h>
#include <mach/board.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_rpcrouter.h>
#include <mach/vreg.h>

#define PM_APP_USB_PROG				0x30000060
#define PM_APP_USB_VERS_1_1			0x00010001
#define PM_APP_USB_VERS_1_2			0x00010002
#define PM_APP_USB_VERS_2_1			0x00020001

#define VBUS_SESS_VALID_CB_PROC			1
#define PM_VOTE_USB_PWR_SEL_SWITCH_APP__HSUSB 	(1 << 2)
#define PM_USB_PWR_SEL_SWITCH_ID 		0

struct rpc_pmapp_ids {
	unsigned long	reg_for_vbus_valid;
	unsigned long	vote_for_vbus_valid_switch;
};

static struct rpc_pmapp_ids rpc_ids;
static struct vreg *boost_vreg, *usb_vreg;
static struct msm_rpc_client *client;
static int ldo_on;

static void rpc_pmapp_init_rpc_ids(unsigned long vers)
{
	if (vers == PM_APP_USB_VERS_1_1) {
		rpc_ids.reg_for_vbus_valid		= 5;
		rpc_ids.vote_for_vbus_valid_switch	= 6;
	} else if (vers == PM_APP_USB_VERS_1_2) {
		rpc_ids.reg_for_vbus_valid		= 16;
		rpc_ids.vote_for_vbus_valid_switch	= 17;
	} else if (vers == PM_APP_USB_VERS_2_1) {
		rpc_ids.reg_for_vbus_valid		= 0; /* NA */
		rpc_ids.vote_for_vbus_valid_switch	= 0; /* NA */
	}
}

struct usb_pwr_sel_switch_args {
	uint32_t cmd;
	uint32_t switch_id;
	uint32_t app_mask;
};

static int usb_pwr_sel_switch_arg_cb(struct msm_rpc_client *client,
					void *buf, void *data)
{
	struct usb_pwr_sel_switch_args *args = buf;

	args->cmd = cpu_to_be32(*(uint32_t *)data);
	args->switch_id = cpu_to_be32(PM_USB_PWR_SEL_SWITCH_ID);
	args->app_mask = cpu_to_be32(PM_VOTE_USB_PWR_SEL_SWITCH_APP__HSUSB);
	return sizeof(struct usb_pwr_sel_switch_args);
}

static int msm_pm_app_vote_usb_pwr_sel_switch(uint32_t cmd)
{
	return msm_rpc_client_req(client,
			rpc_ids.vote_for_vbus_valid_switch,
			usb_pwr_sel_switch_arg_cb,
			&cmd, NULL, NULL, -1);
}

struct vbus_sess_valid_args {
	uint32_t cb_id;
};

static int vbus_sess_valid_arg_cb(struct msm_rpc_client *client,
					void *buf, void *data)
{
	struct vbus_sess_valid_args *args = buf;

	args->cb_id = cpu_to_be32(*(uint32_t *)data);
	return sizeof(struct vbus_sess_valid_args);
}

int msm_pm_app_register_vbus_sn(void (*callback)(int online))
{
	uint32_t cb_id = msm_rpc_add_cb_func(client, (void *)callback);

	/* In case of NULL callback funtion, cb_id would be -1 */
	if ((int) cb_id < -1)
		return cb_id;

	return msm_rpc_client_req(client,
			rpc_ids.reg_for_vbus_valid,
			vbus_sess_valid_arg_cb,
			&cb_id, NULL, NULL, -1);

}
EXPORT_SYMBOL(msm_pm_app_register_vbus_sn);

void msm_pm_app_unregister_vbus_sn(void (*callback)(int online))
{
	msm_rpc_remove_cb_func(client, (void *)callback);
}
EXPORT_SYMBOL(msm_pm_app_unregister_vbus_sn);

int msm_pm_app_enable_usb_ldo(int enable)
{
	int ret;

	if (ldo_on == enable)
		return 0;
	ldo_on = enable;

	if (enable) {
		/* vote to turn ON Boost Vreg_5V */
		ret = vreg_enable(boost_vreg);
		if (ret < 0)
			return ret;
		/* vote to switch it to VREG_5V source */
		ret = msm_pm_app_vote_usb_pwr_sel_switch(1);
		if (ret < 0) {
			vreg_disable(boost_vreg);
			return ret;
		}
		ret = vreg_enable(usb_vreg);
		if (ret < 0) {
			msm_pm_app_vote_usb_pwr_sel_switch(0);
			vreg_disable(boost_vreg);
			return ret;
		}

	} else {
		ret = vreg_disable(usb_vreg);
		if (ret < 0)
			return ret;
		ret = vreg_disable(boost_vreg);
		if (ret < 0)
			return ret;
		/* vote to switch it to VBUS source */
		ret = msm_pm_app_vote_usb_pwr_sel_switch(0);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(msm_pm_app_enable_usb_ldo);

struct vbus_sn_notification_args {
	uint32_t cb_id;
	uint32_t vbus; /* vbus = 0 if VBUS is present */
};

static int vbus_notification_cb(struct msm_rpc_client *client,
				void *buffer, int in_size)
{
	struct vbus_sn_notification_args *args;
	struct rpc_request_hdr *req = buffer;
	int rc;
	uint32_t accept_status;
	void (*cb_func)(int);
	uint32_t cb_id;
	int vbus;

	args = (struct vbus_sn_notification_args *) (req + 1);
	cb_id = be32_to_cpu(args->cb_id);
	vbus = be32_to_cpu(args->vbus);

	cb_func = msm_rpc_get_cb_func(client, cb_id);
	if (cb_func) {
		cb_func(!vbus);
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
	} else
		accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;

	msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
				     accept_status);
	rc = msm_rpc_send_accepted_reply(client, 0);
	if (rc)
		pr_err("%s: send accepted reply failed: %d\n", __func__, rc);

	return rc;
}

static int pm_app_usb_cb_func(struct msm_rpc_client *client,
				void *buffer, int in_size)
{
	int rc;
	struct rpc_request_hdr *req = buffer;

	switch (be32_to_cpu(req->procedure)) {
	case VBUS_SESS_VALID_CB_PROC:
		rc = vbus_notification_cb(client, buffer, in_size);
		break;
	default:
		pr_err("%s: procedure not supported %d\n", __func__,
		       be32_to_cpu(req->procedure));
		msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
					     RPC_ACCEPTSTAT_PROC_UNAVAIL);
		rc = msm_rpc_send_accepted_reply(client, 0);
		if (rc)
			pr_err("%s: sending reply failed: %d\n", __func__, rc);
		break;
	}
	return rc;
}

int msm_pm_app_rpc_init(void)
{

	if (!machine_is_qsd8x50_ffa() && !machine_is_msm7x27_ffa())
		return -ENOTSUPP;

	boost_vreg = vreg_get(NULL, "boost");
	if (IS_ERR(boost_vreg)) {
		pr_err("%s: boost vreg get failed\n", __func__);
		return PTR_ERR(boost_vreg);
	}

	usb_vreg = vreg_get(NULL, "usb");
	if (IS_ERR(usb_vreg)) {
		pr_err("%s: usb vreg get failed\n", __func__);
		vreg_put(usb_vreg);
		return PTR_ERR(usb_vreg);
	}

	client = msm_rpc_register_client("pmapp_usb",
			PM_APP_USB_PROG,
			PM_APP_USB_VERS_2_1, 1,
			pm_app_usb_cb_func);
	if (!IS_ERR(client)) {
		rpc_pmapp_init_rpc_ids(PM_APP_USB_VERS_2_1);
		goto done;
	}

	client = msm_rpc_register_client("pmapp_usb",
			PM_APP_USB_PROG,
			PM_APP_USB_VERS_1_2, 1,
			pm_app_usb_cb_func);
	if (!IS_ERR(client)) {
		rpc_pmapp_init_rpc_ids(PM_APP_USB_VERS_1_2);
		goto done;
	}

	client = msm_rpc_register_client("pmapp_usb",
			PM_APP_USB_PROG,
			PM_APP_USB_VERS_1_1, 1,
			pm_app_usb_cb_func);
	if (!IS_ERR(client))
		rpc_pmapp_init_rpc_ids(PM_APP_USB_VERS_1_1);
	else
		return PTR_ERR(client);

done:
	return 0;
}
EXPORT_SYMBOL(msm_pm_app_rpc_init);

void msm_pm_app_rpc_deinit(void)
{
	if (client) {
		msm_rpc_unregister_client(client);
		msm_pm_app_enable_usb_ldo(0);
		vreg_put(boost_vreg);
		vreg_put(usb_vreg);
	}
}
EXPORT_SYMBOL(msm_pm_app_rpc_deinit);

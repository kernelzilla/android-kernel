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
#include <linux/kthread.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <mach/rpc_pmapp.h>
#include <linux/workqueue.h>

#define RPC_TYPE_REQUEST			0
#define RPC_TYPE_REPLY				1

#define RPC_COMMON_HDR_SZ			(sizeof(uint32_t) * 2)
#define RPC_REQUEST_HDR_SZ 			(sizeof(struct rpc_request_hdr))
#define RPC_REPLY_HDR_SZ			(sizeof(uint32_t) * 3)

#define PM_VOTE_USB_PWR_SEL_SWITCH_APP__HSUSB 	(1 << 2)
#define PM_SWITCH_OFF_CMD			0
#define PM_SWITCH_ON_CMD 			1
#define PM_USB_PWR_SEL_SWITCH_ID 		0
#define PM_VREG_BOOST_ID 			17
#define PM_VREG_USB_ID				16
#define PM_OFF_CMD 				0
#define PM_ON_CMD 				1

static struct msm_rpc_endpoint *pm_app_ep;

static struct task_struct *task;
static uint32_t vreg_boost_id, vreg_usb_id;
static struct work_struct usb_vbus_sn_w;
struct completion rpc_reply_done;
int32_t rpc_reply_status;

struct msm_pm_app_rpc_ids {
	unsigned long   prog;
	unsigned long   vers_comp;
	unsigned        vbus_sn_valid_cb_type;
	unsigned        vote_vreg_switch;
	unsigned        vote_vreg_req;
	unsigned        reg_for_vbus_sn_valid;
	unsigned        vote_usb_pwr_sel_switch;
};

static struct msm_pm_app_rpc_ids pm_app_rpc_ids;

/* rpc connect for pm_app app */
static int msm_pm_app_rpc_connect(void)
{
	if (!machine_is_qsd8x50_ffa())
		return -ENOTSUPP;

	if (pm_app_ep && !IS_ERR(pm_app_ep)) {
		pr_info("%s: pm_app_ep already connected\n", __func__);
		return 0;
	}

	/* Initialize rpc ids with version compatible */
	pm_app_rpc_ids.prog                             = 0x30000060;
	pm_app_rpc_ids.vers_comp                        = 0x00010001;
	pm_app_rpc_ids.vbus_sn_valid_cb_type            = 1;
	pm_app_rpc_ids.vote_vreg_switch                 = 3;
	pm_app_rpc_ids.vote_vreg_req                    = 4;
	pm_app_rpc_ids.reg_for_vbus_sn_valid            = 16;
	pm_app_rpc_ids.vote_usb_pwr_sel_switch          = 17;

	pm_app_ep = msm_rpc_connect_compatible(pm_app_rpc_ids.prog,
				pm_app_rpc_ids.vers_comp, 0);
	if (IS_ERR(pm_app_ep)) {
		pr_err("%s: connect failed vers = %lx\n",
				__func__, pm_app_rpc_ids.vers_comp);
	return PTR_ERR(pm_app_ep);
	}
	pr_info("%s: rpc connect success vers = %lx\n",
				__func__, pm_app_rpc_ids.vers_comp);

       return 0;
}

int msm_pm_app_reg_for_vbus_sn_valid(void)
{
	int rc = 0;
	struct pm_app_start_req {
		struct rpc_request_hdr hdr;
		uint32_t cb_id;
	} req;

	if (!pm_app_ep || IS_ERR(pm_app_ep))
		return -EAGAIN;

	memset(&req, 0, sizeof(struct pm_app_start_req));

	req.cb_id = cpu_to_be32(0x11111111);

	msm_rpc_setup_req(&req.hdr, pm_app_rpc_ids.prog,
				pm_app_rpc_ids.vers_comp,
				pm_app_rpc_ids.reg_for_vbus_sn_valid);

	rc = msm_rpc_write(pm_app_ep, &req, sizeof(req));
	if (rc < 0) {
		pr_err("%s: failed! rc = %d\n",
			__func__, rc);
		return rc;
	}

	rc = wait_for_completion_timeout(&rpc_reply_done, 5*HZ);
	if (rc == 0) {
		pr_err("%s: failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	}

	if (rpc_reply_status != RPC_ACCEPTSTAT_SUCCESS) {
		pr_err("%s: failed! rpc_reply_status = %d\n",
				__func__, rpc_reply_status);
		return -rpc_reply_status;
	}

	return rpc_reply_status;
}

int msm_pm_app_vote_vreg_request(uint32_t cmd_id)
{
	int rc = 0;
	struct pm_app_start_req {
		struct rpc_request_hdr hdr;
		uint32_t cmd;
		uint32_t out_ptr_not_null;
	} req;

	if (!pm_app_ep || IS_ERR(pm_app_ep))
		return -EAGAIN;

	req.cmd = cpu_to_be32(cmd_id);
	req.out_ptr_not_null = cpu_to_be32(1);

	msm_rpc_setup_req(&req.hdr, pm_app_rpc_ids.prog,
				pm_app_rpc_ids.vers_comp,
				pm_app_rpc_ids.vote_vreg_req);

	rc = msm_rpc_write(pm_app_ep, &req, sizeof(req));
	if (rc < 0) {
		pr_err("%s: failed! rc = %d\n",
			__func__, rc);
		return rc;
	}

	rc = wait_for_completion_timeout(&rpc_reply_done, 5*HZ);
	if (rc == 0) {
		pr_err("%s: failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	}

	if (rpc_reply_status != RPC_ACCEPTSTAT_SUCCESS) {
		pr_err("%s: failed! rpc_reply_status = %d\n",
				__func__, rpc_reply_status);
		return -rpc_reply_status;
	}

	if (cmd_id == PM_VREG_BOOST_ID)
		vreg_boost_id = 0x20000000;
	else if (cmd_id == PM_VREG_USB_ID)
		vreg_usb_id = 0x20000000;

	return rpc_reply_status;
}

int msm_pm_app_vote_vreg_switch(uint32_t cmd, uint32_t vreg_id,
						uint32_t app_mask)
{
	int rc = 0;
	struct pm_app_start_req {
		struct rpc_request_hdr hdr;
		uint32_t cmd;
		uint32_t vreg_id;
		uint32_t app_mask;
	} req;

	if (!pm_app_ep || IS_ERR(pm_app_ep))
		return -EAGAIN;

	req.cmd = cpu_to_be32(cmd);
	req.vreg_id = cpu_to_be32(vreg_id);
	req.app_mask = cpu_to_be32(app_mask);

	msm_rpc_setup_req(&req.hdr, pm_app_rpc_ids.prog,
				pm_app_rpc_ids.vers_comp,
				pm_app_rpc_ids.vote_vreg_switch);

	rc = msm_rpc_write(pm_app_ep, &req, sizeof(req));
	if (rc < 0) {
		pr_err("%s: failed! rc = %d\n",
			__func__, rc);
		return rc;
	}

	rc = wait_for_completion_timeout(&rpc_reply_done, 5*HZ);
	if (rc == 0) {
		pr_err("%s: failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	}

	if (rpc_reply_status != RPC_ACCEPTSTAT_SUCCESS) {
		pr_err("%s: failed! rpc_reply_status = %d\n",
				__func__, rpc_reply_status);
		return -rpc_reply_status;
	}

	return rpc_reply_status;
}

int msm_pm_app_vote_usb_pwr_sel_switch(uint32_t cmd, uint32_t switch_id,
							uint32_t app_mask)
{
	int rc = 0;
	struct pm_app_start_req {
		struct rpc_request_hdr hdr;
		uint32_t cmd;
		uint32_t switch_id;
		uint32_t app_mask;
	} req;

	if (!pm_app_ep || IS_ERR(pm_app_ep))
		return -EAGAIN;

	req.cmd = cpu_to_be32(cmd);
	req.switch_id = cpu_to_be32(switch_id);
	req.app_mask = cpu_to_be32((app_mask));

	msm_rpc_setup_req(&req.hdr, pm_app_rpc_ids.prog,
				pm_app_rpc_ids.vers_comp,
				pm_app_rpc_ids.vote_usb_pwr_sel_switch);

	rc = msm_rpc_write(pm_app_ep, &req, sizeof(req));
	if (rc < 0) {
		pr_err("%s: failed! rc = %d\n",
			__func__, rc);
		return rc;
	}

	rc = wait_for_completion_timeout(&rpc_reply_done, 5*HZ);
	if (rc == 0) {
		pr_err("%s: failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	}

	if (rpc_reply_status != RPC_ACCEPTSTAT_SUCCESS) {
		pr_err("%s: failed! rpc_reply_status = %d\n",
				__func__, rpc_reply_status);
		return -rpc_reply_status;
	}

	return rpc_reply_status;
}

/* rpc call to close pm_app connection */
static int msm_pm_app_rpc_close(void)
{
	int rc = 0;

	if (!pm_app_ep || IS_ERR(pm_app_ep)) {
		pr_err("%s: rpc_close failed before call, rc = %ld\n",
			__func__, PTR_ERR(pm_app_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_close(pm_app_ep);
	pm_app_ep = NULL;

	if (rc < 0) {
		pr_err("%s: close rpc failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	} else
		pr_info("rpc close success\n");

	return rc;
}

static void rpc_reply(struct msm_rpc_endpoint *ept, uint32_t xid)
{
	int rc = 0;
	uint8_t reply_buf[sizeof(struct rpc_reply_hdr)];
	struct rpc_reply_hdr *reply = (struct rpc_reply_hdr *)reply_buf;

	reply->xid = cpu_to_be32(xid);
	reply->type = cpu_to_be32(RPC_TYPE_REPLY);
	reply->reply_stat = cpu_to_be32(RPCMSG_REPLYSTAT_ACCEPTED);

	reply->data.acc_hdr.accept_stat = cpu_to_be32(RPC_ACCEPTSTAT_SUCCESS);
	reply->data.acc_hdr.verf_flavor = 0;
	reply->data.acc_hdr.verf_length = 0;

	rc = msm_rpc_write(ept, reply_buf, sizeof(reply_buf));
	if (rc < 0)
		pr_err("%s: could not write RPC response: %d\n", __func__, rc);
}

static int pm_app_rpc_thread(void *data)
{
	struct rpc_request_hdr *hdr = NULL;
	uint32_t type;
	int len;
	struct vbus_sn_notification {
		struct rpc_request_hdr hdr;
		uint32_t cb_id;
		uint32_t vbus_sn_status;
	};
	struct vbus_sn_notification *rep;

	while (!kthread_should_stop()) {
		kfree(hdr);
		len = msm_rpc_read(pm_app_ep, (void **) &hdr, -1, -1);
		if (len < 0) {
			pr_info("%s: rpc read failed\n", __func__);
			break;
		}

		if (len < RPC_COMMON_HDR_SZ)
			continue;

		type = be32_to_cpu(hdr->type);
		if (type == RPC_TYPE_REPLY) {
			struct rpc_reply_hdr *rep = (void *) hdr;

			if (len < RPC_REPLY_HDR_SZ)
				continue;

			rpc_reply_status = be32_to_cpu(rep->reply_stat);
			if (rpc_reply_status == RPCMSG_REPLYSTAT_ACCEPTED)
				rpc_reply_status = be32_to_cpu
					(rep->data.acc_hdr.accept_stat);

			complete(&rpc_reply_done);
			continue;
		}

		if (len < RPC_REQUEST_HDR_SZ)
			continue;

		if (pm_app_rpc_ids.vbus_sn_valid_cb_type ==
				be32_to_cpu(hdr->procedure)) {
			rep = (struct vbus_sn_notification *)hdr;
			if (!(be32_to_cpu(rep->vbus_sn_status)))
				schedule_work(&usb_vbus_sn_w);
		}
		rpc_reply(pm_app_ep, be32_to_cpu(hdr->xid));

	}
	kfree(hdr);
	return 0;
}


static void pm_app_usb_vbus_sn_valid(struct work_struct *w)
{
	pr_info("\n%s: VBUS SN VALID\n", __func__);

	/* Enable USB internal ldo */
	msm_pm_app_enable_usb_ldo(1);

	/* Notify USB driver for connect event */
	msm_hsusb_set_vbus_state(1);
}

int msm_pm_app_rpc_init(void)
{
	int ret;

	ret = msm_pm_app_rpc_connect();
	if (ret)
		return ret;

	init_completion(&rpc_reply_done);

	task = kthread_run(pm_app_rpc_thread, NULL, "pm_app_rpc");
	if (IS_ERR(task))
		return PTR_ERR(task);
	return 0;
}
EXPORT_SYMBOL(msm_pm_app_rpc_init);

int msm_pm_app_register_vbus_sn(void)
{
	int ret;

	/* Register for VBUS change notifications */
	ret = msm_pm_app_reg_for_vbus_sn_valid();
	if (ret < 0)
		return ret;

	msm_pm_app_vote_vreg_request(PM_VREG_BOOST_ID);
	msm_pm_app_vote_vreg_request(PM_VREG_USB_ID);

	INIT_WORK(&usb_vbus_sn_w, pm_app_usb_vbus_sn_valid);

	return 0;
}
EXPORT_SYMBOL(msm_pm_app_register_vbus_sn);

void msm_pm_app_unregister_vbus_sn(void)
{
	cancel_work_sync(&usb_vbus_sn_w);
}
EXPORT_SYMBOL(msm_pm_app_unregister_vbus_sn);

int msm_pm_app_enable_usb_ldo(int enable)
{
	int ret;

	if (!machine_is_qsd8x50_ffa())
		return -ENOTSUPP;

	if (enable) {
		/* vote to turn ON Boost Vreg_5V */
		ret = msm_pm_app_vote_vreg_switch(PM_ON_CMD,
				PM_VREG_BOOST_ID, vreg_boost_id);
		if (ret < 0)
			return ret;
		/* vote to switch it to VREG_5V source */
		ret = msm_pm_app_vote_usb_pwr_sel_switch(PM_SWITCH_ON_CMD,
				PM_USB_PWR_SEL_SWITCH_ID,
				PM_VOTE_USB_PWR_SEL_SWITCH_APP__HSUSB);
		if (ret < 0)
			return ret;
		/* vote to turn ON USB LDO Vreg */
		ret = msm_pm_app_vote_vreg_switch(PM_ON_CMD, PM_VREG_USB_ID,
				vreg_usb_id);
		if (ret < 0)
			return ret;
	} else {
		/* vote to turn OFF USB LDO Vreg */
		ret = msm_pm_app_vote_vreg_switch(PM_OFF_CMD, PM_VREG_USB_ID,
				vreg_usb_id);
		if (ret < 0)
			return ret;
		/* vote to turn OFF Boost Vreg_5V */
		ret = msm_pm_app_vote_vreg_switch(PM_OFF_CMD,
				PM_VREG_BOOST_ID, vreg_boost_id);
		if (ret < 0)
			return ret;
		/* vote to switch it to VBUS source */
		ret = msm_pm_app_vote_usb_pwr_sel_switch(PM_SWITCH_OFF_CMD,
				PM_USB_PWR_SEL_SWITCH_ID,
				PM_VOTE_USB_PWR_SEL_SWITCH_APP__HSUSB);
		if (ret < 0)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL(msm_pm_app_enable_usb_ldo);

void msm_pm_app_rpc_deinit(void)
{
	if (task && !IS_ERR(task))
		kthread_stop(task);

	msm_pm_app_rpc_close();
}
EXPORT_SYMBOL(msm_pm_app_rpc_deinit);

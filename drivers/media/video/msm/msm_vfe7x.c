/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/msm_adsp.h>
#include <linux/uaccess.h>
#if !defined(CONFIG_MACH_PITTSBURGH)
#include <linux/fs.h>
#endif
#include <linux/android_pmem.h>
#include <mach/msm_adsp.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include "msm_vfe7x.h"

#define QDSP_CMDQUEUE 25

#define VFE_RESET_CMD 0
#define VFE_START_CMD 1
#define VFE_STOP_CMD  2
#define VFE_FRAME_ACK 20
#define STATS_AF_ACK  21
#define STATS_WE_ACK  22

#define MSG_STOP_ACK  1
#define MSG_SNAPSHOT  2
#define MSG_OUTPUT1   6
#define MSG_OUTPUT2   7
#define MSG_STATS_AF  8
#define MSG_STATS_WE  9

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
#define VFE_ADSP_EVENT 0xFFFFFFFF
#else
#define VFE_ADSP_EVENT 0xFFFF
#endif

static struct msm_adsp_module *qcam_mod;
static struct msm_adsp_module *vfe_mod;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static struct msm_vfe_resp *resp;
#else
static struct msm_vfe_callback *resp;
#endif
static void *extdata;
static uint32_t extlen;

struct mutex vfe_lock;
static uint32_t vfe_inuse;
static void     *vfe_syncdata;
static uint8_t vfestopped;

static struct stop_event stopevent;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static void vfe_7x_convert(struct msm_vfe_phy_info *pinfo,
	enum vfe_resp_msg_t	type, void *data, void **ext, int32_t *elen)
#else
static void vfe_7x_convert(struct msm_vfe_phy_info *pinfo,
                enum vfe_resp_msg type,
                void *data, void **ext, int32_t *elen)
#endif
{
	switch (type) {
	case VFE_MSG_OUTPUT1:
	case VFE_MSG_OUTPUT2: {
		pinfo->y_phy = ((struct vfe_endframe *)data)->y_address;
		pinfo->cbcr_phy =
			((struct vfe_endframe *)data)->cbcr_address;

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		CVBS("vfe_7x_convert, y_phy = 0x%x, cbcr_phy = 0x%x\n",
				 pinfo->y_phy, pinfo->cbcr_phy);
#else
		CDBG("vfe_7x_convert, y_phy = 0x%x, cbcr_phy = 0x%x\n",
			pinfo->y_phy, pinfo->cbcr_phy);
#endif

		((struct vfe_frame_extra *)extdata)->bl_evencol =
		((struct vfe_endframe *)data)->blacklevelevencolumn;

		((struct vfe_frame_extra *)extdata)->bl_oddcol =
		((struct vfe_endframe *)data)->blackleveloddcolumn;

		((struct vfe_frame_extra *)extdata)->g_def_p_cnt =
		((struct vfe_endframe *)data)->greendefectpixelcount;

		((struct vfe_frame_extra *)extdata)->r_b_def_p_cnt =
		((struct vfe_endframe *)data)->redbluedefectpixelcount;

		*ext  = extdata;
		*elen = extlen;
	}
		break;

	case VFE_MSG_STATS_AF:
	case VFE_MSG_STATS_WE:
		pinfo->sbuf_phy = *(uint32_t *)data;
		break;

	default:
		break;
	} /* switch */
}

static void vfe_7x_ops(void *driver_data, unsigned id, size_t len,
		void (*getevent)(void *ptr, size_t len))
{
	uint32_t evt_buf[3];
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct msm_vfe_resp_t *rp;
#else
	struct msm_vfe_resp *rp;
        void *data;
#endif

#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
    len = (id == VFE_ADSP_EVENT) ? 0 : len;
       data = resp->vfe_alloc(sizeof(struct msm_vfe_resp) + len, vfe_syncdata);

       if (!data) {
               pr_err("rp: cannot allocate buffer\n");
		return;
	}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	rp = kmalloc(sizeof(struct msm_vfe_resp_t), GFP_ATOMIC);
	if (!rp) {
		CDBG("rp: cannot allocate buffer\n");
		return;
	}
#endif
	if (id == VFE_ADSP_EVENT) {
		/* event */
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		getevent(evt_buf, sizeof(evt_buf));
#endif
		rp->type           = VFE_EVENT;
		rp->evt_msg.type   = MSM_CAMERA_EVT;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		rp->evt_msg.len    = 0;
#else
		 getevent(evt_buf, sizeof(evt_buf));
#endif
		rp->evt_msg.msg_id = evt_buf[0];

		resp->vfe_resp(rp, MSM_CAM_Q_VFE_EVT, vfe_syncdata);
	} else {
		/* messages */

		rp->evt_msg.type   = MSM_CAMERA_MSG;
		rp->evt_msg.msg_id = id;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		rp->evt_msg.len    = len;

		rp->evt_msg.data = kmalloc(rp->evt_msg.len, GFP_ATOMIC);
		if (!(rp->evt_msg.data)) {
			kfree(rp);
			CDBG("rp->evt_msg.data: cannot allocate buffer\n");
			return;
		}
#else
		rp->evt_msg.data = rp + 1;
#endif
		getevent(rp->evt_msg.data, len);

		switch (rp->evt_msg.msg_id) {
		case MSG_SNAPSHOT:
			rp->type = VFE_MSG_SNAPSHOT;
			break;

		case MSG_OUTPUT1:
			rp->type = VFE_MSG_OUTPUT1;
			vfe_7x_convert(&(rp->phy), VFE_MSG_OUTPUT1,
				rp->evt_msg.data, &(rp->extdata),
				&(rp->extlen));
			break;

		case MSG_OUTPUT2:
			rp->type = VFE_MSG_OUTPUT2;
			vfe_7x_convert(&(rp->phy), VFE_MSG_OUTPUT2,
					rp->evt_msg.data, &(rp->extdata),
					&(rp->extlen));
			break;

		case MSG_STATS_AF:
			rp->type = VFE_MSG_STATS_AF;
			vfe_7x_convert(&(rp->phy), VFE_MSG_STATS_AF,
					rp->evt_msg.data, NULL, NULL);
			break;

		case MSG_STATS_WE:
			rp->type = VFE_MSG_STATS_WE;
			vfe_7x_convert(&(rp->phy), VFE_MSG_STATS_WE,
					rp->evt_msg.data, NULL, NULL);

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
			CVBS("MSG_STATS_WE: phy = 0x%x\n", rp->phy.sbuf_phy);
#else
			CDBG("MSG_STATS_WE: phy = 0x%x\n", rp->phy.sbuf_phy);
#endif
			break;

		case MSG_STOP_ACK:
			rp->type = VFE_MSG_GENERAL;
			stopevent.state = 1;
			wake_up(&stopevent.wait);
			break;


		default:
			rp->type = VFE_MSG_GENERAL;
			break;
		}

		resp->vfe_resp(rp, MSM_CAM_Q_VFE_MSG, vfe_syncdata);
	}
}

static struct msm_adsp_ops vfe_7x_sync = {
	.event = vfe_7x_ops,
};

#if defined(CONFIG_MACH_PITTSBURGH)
static int vfe_7x_enable(struct camera_enable_cmd_t *enable)
#else
static int vfe_7x_enable(struct camera_enable_cmd *enable)
#endif
{
	int rc = -EFAULT;

	CDBG("%s: %s\n", __func__, enable->name);
    if (!strcmp(enable->name, "QCAMTASK"))
		rc = msm_adsp_enable(qcam_mod);
	else if (!strcmp(enable->name, "VFETASK"))
		rc = msm_adsp_enable(vfe_mod);

	return rc;
}

#if defined(CONFIG_MACH_PITTSBURGH)
static int vfe_7x_disable(struct camera_enable_cmd_t *enable,
	struct platform_device *dev)
#else
static int vfe_7x_disable(struct camera_enable_cmd *enable,
                struct platform_device *dev __attribute__((unused)))
#endif
{
	int rc = -EFAULT;

	CDBG("%s: %s\n", __func__, enable->name);
	if (!strcmp(enable->name, "QCAMTASK"))
		rc = msm_adsp_disable(qcam_mod);
	else if (!strcmp(enable->name, "VFETASK"))
		rc = msm_adsp_disable(vfe_mod);

	return rc;
}

static int vfe_7x_stop(void)
{
	int rc = 0;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	struct vfe_stop_t stopcmd;
	stopcmd.header = VFE_STOP_CMD;
#else
	uint32_t stopcmd = VFE_STOP_CMD;
#endif
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	rc = msm_adsp_write(vfe_mod, QDSP_CMDQUEUE,
				&stopcmd, sizeof(struct vfe_stop_t));
#else
	rc = msm_adsp_write(vfe_mod, QDSP_CMDQUEUE,
				&stopcmd, sizeof(uint32_t));
#endif
	if (rc < 0)
		CDBG("%s:%d:Failed... rc = %d \n", __func__, __LINE__, rc);

	stopevent.state = 0;
	rc = wait_event_timeout(stopevent.wait,
		stopevent.state != 0,
		msecs_to_jiffies(stopevent.timeout));

	return rc;
}
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static void vfe_7x_release(struct platform_device *dev)
#else
static void vfe_7x_release(struct platform_device *pdev)
#endif
{
	mutex_lock(&vfe_lock);
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	vfe_inuse = 0;
#endif
	vfe_syncdata = NULL;
	mutex_unlock(&vfe_lock);

	if (!vfestopped) {
		CDBG("%s:%d:Calling vfe_7x_stop()\n", __func__, __LINE__);
		vfe_7x_stop();
	} else
		vfestopped = 0;

	msm_adsp_disable(qcam_mod);
	msm_adsp_disable(vfe_mod);

	msm_adsp_put(qcam_mod);
	msm_adsp_put(vfe_mod);

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	msm_camio_disable(dev);
#else
	msm_camio_disable(pdev);
#endif

	kfree(extdata);
	extlen = 0;
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static int vfe_7x_init(struct msm_vfe_resp *presp,
	struct platform_device *dev)
#else
static int vfe_7x_init(struct msm_vfe_callback *presp,
        struct platform_device *dev)
#endif
{
	int rc = 0;

	init_waitqueue_head(&stopevent.wait);
	stopevent.timeout = 200;
	stopevent.state = 0;

	if (presp && presp->vfe_resp)
		resp = presp;
	else
		return -EFAULT;

	/* Bring up all the required GPIOs and Clocks */
	rc = msm_camio_enable(dev);
	if (rc < 0)
		return rc;

	msm_camio_camif_pad_reg_reset();

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)

	extlen = sizeof(struct vfe_frame_extra);

	extdata =
		kmalloc(extlen, GFP_ATOMIC);
#else
	 extdata = kmalloc(sizeof(struct vfe_frame_extra), GFP_ATOMIC);

#endif

	if (!extdata) {
		rc = -ENOMEM;
		goto init_fail;
	}

	rc = msm_adsp_get("QCAMTASK", &qcam_mod, &vfe_7x_sync, NULL);
	if (rc) {
		rc = -EBUSY;
		goto get_qcam_fail;
	}

	rc = msm_adsp_get("VFETASK", &vfe_mod, &vfe_7x_sync, NULL);
	if (rc) {
		rc = -EBUSY;
		goto get_vfe_fail;
	}

	return 0;

get_vfe_fail:
	msm_adsp_put(qcam_mod);
get_qcam_fail:
	kfree(extdata);
init_fail:
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
	extlen = 0;
#endif
	return rc;
}

#if defined(CONFIG_MACH_PITTSBURGH)
static int vfe_7x_config_axi(enum vfeoutput_mode_t mode,
	struct axidata_t *ad, struct axiout_t *ao)
#else
static int vfe_7x_config_axi(int mode,
        struct axidata *ad, struct axiout *ao)
#endif
{
	struct msm_pmem_region *regptr;
	unsigned long *bptr;
	int    cnt;

	int rc = 0;

	if (mode == OUTPUT_1 || mode == OUTPUT_1_AND_2) {
		regptr = ad->region;

		CDBG("bufnum1 = %d\n", ad->bufnum1);
		CDBG("config_axi1: O1, phy = 0x%lx, y_off = %d, cbcr_off =%d\n",
			regptr->paddr, regptr->y_off, regptr->cbcr_off);

		bptr = &ao->output1buffer1_y_phy;
		for (cnt = 0; cnt < ad->bufnum1; cnt++) {
			*bptr = regptr->paddr + regptr->y_off;
			bptr++;
			*bptr = regptr->paddr + regptr->cbcr_off;

			bptr++;
			regptr++;
		}

		regptr--;
		for (cnt = 0; cnt < (8 - ad->bufnum1); cnt++) {
			*bptr = regptr->paddr + regptr->y_off;
			bptr++;
			*bptr = regptr->paddr + regptr->cbcr_off;
			bptr++;
		}
	} /* if OUTPUT1 or Both */

	if (mode == OUTPUT_2 || mode == OUTPUT_1_AND_2) {
		regptr = &(ad->region[ad->bufnum1]);

		CDBG("bufnum2 = %d\n", ad->bufnum2);
		CDBG("config_axi2: O2, phy = 0x%lx, y_off = %d, cbcr_off =%d\n",
			regptr->paddr, regptr->y_off, regptr->cbcr_off);

		bptr = &ao->output2buffer1_y_phy;
		for (cnt = 0; cnt < ad->bufnum2; cnt++) {
			*bptr = regptr->paddr + regptr->y_off;
			bptr++;
			*bptr = regptr->paddr + regptr->cbcr_off;

			bptr++;
			regptr++;
		}

		regptr--;
		for (cnt = 0; cnt < (8 - ad->bufnum2); cnt++) {
			*bptr = regptr->paddr + regptr->y_off;
			bptr++;
			*bptr = regptr->paddr + regptr->cbcr_off;
			bptr++;
		}
	}

	return rc;
}
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
static int vfe_7x_config(struct msm_vfe_cfg_cmd *cmd, void *data)
{
	struct msm_pmem_region *regptr;
	unsigned char buf[256];

	struct vfe_stats_ack_t sack;
	struct axidata *axid;
	uint32_t i;
	struct vfe_stats_we_cfg_t *scfg_t = NULL;
	struct vfe_stats_af_cfg_t *sfcfg_t = NULL;

	struct axiout *axio = NULL;
	void   *cmd_data = NULL;
	void   *cmd_data_alloc = NULL;
	long rc = 0;
	struct msm_vfe_command_7k *vfecmd;

	vfecmd =
			kmalloc(sizeof(struct msm_vfe_command_7k),
				GFP_ATOMIC);
	if (!vfecmd) {
		CDBG("vfecmd alloc failed!\n");
		return -ENOMEM;
	}

	if (cmd->cmd_type != CMD_FRAME_BUF_RELEASE &&
	    cmd->cmd_type != CMD_STATS_BUF_RELEASE &&
	    cmd->cmd_type != CMD_STATS_AF_BUF_RELEASE) {
		if (copy_from_user(vfecmd,
				(void __user *)(cmd->value),
				sizeof(struct msm_vfe_command_7k))) {
			rc = -EFAULT;
			goto config_failure;
		}
	}

	switch (cmd->cmd_type) {
	case CMD_STATS_ENABLE:
	case CMD_STATS_AXI_CFG: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}
		scfg_t =
			kmalloc(sizeof(struct vfe_stats_we_cfg_t),
				GFP_ATOMIC);
		if (!scfg_t) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(scfg_t,
					(void __user *)(vfecmd->value),
					vfecmd->length)) {

			rc = -EFAULT;
			goto config_done;
		}
		CVBS("STATS_ENABLE: bufnum = %d, enabling = %d\n",
			axid->bufnum1, scfg_t->wb_expstatsenable);

		if (axid->bufnum1 > 0) {
			regptr = axid->region;

			for (i = 0; i < axid->bufnum1; i++) {

				CVBS("STATS_ENABLE, phy = 0x%lx\n",
					regptr->paddr);
				scfg_t->wb_expstatoutputbuffer[i] =
					(void *)regptr->paddr;

				regptr++;
			}
			cmd_data = scfg_t;

		} else {
			rc = -EINVAL;
			goto config_done;
		}
	}
		break;

	case CMD_STATS_AF_ENABLE:
	case CMD_STATS_AF_AXI_CFG: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}
		sfcfg_t =
			kmalloc(sizeof(struct vfe_stats_af_cfg_t),
				GFP_ATOMIC);

		if (!sfcfg_t) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(sfcfg_t,
					(void __user *)(vfecmd->value),
					vfecmd->length)) {

			rc = -EFAULT;
			goto config_done;
		}
		CVBS("AF_ENABLE: bufnum = %d, enabling = %d\n",
			axid->bufnum1, sfcfg_t->af_enable);
		if (axid->bufnum1 > 0) {
			regptr = axid->region;
		for (i = 0; i < axid->bufnum1; i++) {
				sfcfg_t->af_outbuf[i] =
					(void *)regptr->paddr;

				regptr++;
			}
			cmd_data = sfcfg_t;
		} else {
			rc = -EINVAL;
			goto config_done;
		}
	}
		break;

	case CMD_FRAME_BUF_RELEASE: {
		struct msm_frame *b;
		unsigned long p;
		struct vfe_outputack_t fack;
		if (!data)  {
			rc = -EFAULT;
			goto config_failure;
		}
		b = (struct msm_frame *)(cmd->value);
		p = *(unsigned long *)data;

		fack.header = VFE_FRAME_ACK;

		fack.output2newybufferaddress =
			(void *)(p + b->y_off);

		fack.output2newcbcrbufferaddress =
			(void *)(p + b->cbcr_off);

		vfecmd->queue = QDSP_CMDQUEUE;
		vfecmd->length = sizeof(struct vfe_outputack_t);
		cmd_data = &fack;
	}
		break;

	case CMD_SNAP_BUF_RELEASE:
		break;

	case CMD_STATS_BUF_RELEASE: {
    CVBS("vfe_7x_config: CMD_STATS_BUF_RELEASE\n");
		if (!data) {
			rc = -EFAULT;
			goto config_failure;
		}

		sack.header = STATS_WE_ACK;
		sack.bufaddr = (void *)*(uint32_t *)data;

		vfecmd->queue  = QDSP_CMDQUEUE;
#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
		vfecmd->length = sizeof(struct vfe_stats_ack_t);
#else
		vfecmd->length = sizeof(struct vfe_stats_ack);
#endif
		cmd_data = &sack;
	}
		break;

	case CMD_STATS_AF_BUF_RELEASE: {
    CVBS("vfe_7x_config: CMD_STATS_AF_BUF_RELEASE\n");
    if (!data) {
			rc = -EFAULT;
			goto config_failure;
		}

		sack.header = STATS_AF_ACK;
		sack.bufaddr = (void *)*(uint32_t *)data;

		vfecmd->queue  = QDSP_CMDQUEUE;
		vfecmd->length = sizeof(struct vfe_stats_ack_t);
		cmd_data = &sack;
	}
		break;

	case CMD_GENERAL:
	case CMD_STATS_DISABLE: {
		if (vfecmd->length > 256) {
			cmd_data_alloc =
			cmd_data = kmalloc(vfecmd->length, GFP_ATOMIC);
			if (!cmd_data) {
				rc = -ENOMEM;
				goto config_failure;
			}
		} else
			cmd_data = buf;

		if (copy_from_user(cmd_data,
					(void __user *)(vfecmd->value),
					vfecmd->length)) {

			rc = -EFAULT;
			goto config_done;
		}

		if (vfecmd->queue == QDSP_CMDQUEUE) {
			switch (*(uint32_t *)cmd_data) {
			case VFE_RESET_CMD:
				msm_camio_vfe_blk_reset();
				vfestopped = 0;
				break;

			case VFE_START_CMD:
				msm_camio_camif_pad_reg_reset_2();
				vfestopped = 0;
				break;

			case VFE_STOP_CMD:
				vfestopped = 1;
				goto config_send;

			default:
				break;
			}
		} /* QDSP_CMDQUEUE */
	}
		break;

	case CMD_AXI_CFG_OUT1: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}
		axio = kmalloc(sizeof(struct axiout), GFP_ATOMIC);
		if (!axio) {
			rc = -ENOMEM;
			goto config_failure;
		}
		if (copy_from_user(axio, (void *)(vfecmd->value),
					sizeof(struct axiout))) {
			rc = -EFAULT;
			goto config_done;
		}

		vfe_7x_config_axi(OUTPUT_1, axid, axio);

		cmd_data = axio;
	}
		break;

	case CMD_AXI_CFG_OUT2:
	case CMD_RAW_PICT_AXI_CFG: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}
		axio = kmalloc(sizeof(struct axiout), GFP_ATOMIC);
		if (!axio) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(axio, (void __user *)(vfecmd->value),
					sizeof(struct axiout))) {
			rc = -EFAULT;
			goto config_done;
		}
		vfe_7x_config_axi(OUTPUT_2, axid, axio);
		cmd_data = axio;
	}
		break;

	case CMD_AXI_CFG_SNAP_O1_AND_O2: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}

		axio = kmalloc(sizeof(struct axiout), GFP_ATOMIC);
		if (!axio) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(axio, (void __user *)(vfecmd->value),
					sizeof(struct axiout))) {
			rc = -EFAULT;
			goto config_done;
		}
		vfe_7x_config_axi(OUTPUT_1_AND_2, axid, axio);

		cmd_data = axio;
	}
		break;

	default:
		break;
	} /* switch */

	if (vfestopped)
		goto config_done;

config_send:
	CVBS("send adsp command = %d\n", *(uint32_t *)cmd_data);
	rc = msm_adsp_write(vfe_mod, vfecmd->queue,
				cmd_data, vfecmd->length);

config_done:
	if (cmd_data_alloc != NULL)
		kfree(cmd_data_alloc);

config_failure:
	kfree(scfg_t);
	kfree(axio);
	kfree(vfecmd);
	return rc;
}
#else
static int vfe_7x_config(struct msm_vfe_cfg_cmd *cmd, void *data)
{
	struct msm_pmem_region *regptr;
	unsigned char buf[256];

	struct vfe_stats_ack sack;
	struct axidata *axid;
	uint32_t i;

	struct vfe_stats_we_cfg *scfg = NULL;
	struct vfe_stats_af_cfg *sfcfg = NULL;

	struct axiout *axio = NULL;
	void   *cmd_data = NULL;
	void   *cmd_data_alloc = NULL;
	long rc = 0;
	struct msm_vfe_command_7k *vfecmd;

	vfecmd =
			kmalloc(sizeof(struct msm_vfe_command_7k),
				GFP_ATOMIC);
	if (!vfecmd) {
		pr_err("vfecmd alloc failed!\n");
		return -ENOMEM;
	}

	if (cmd->cmd_type != CMD_FRAME_BUF_RELEASE &&
	    cmd->cmd_type != CMD_STATS_BUF_RELEASE &&
	    cmd->cmd_type != CMD_STATS_AF_BUF_RELEASE) {
		if (copy_from_user(vfecmd,
				(void __user *)(cmd->value),
				sizeof(struct msm_vfe_command_7k))) {
			rc = -EFAULT;
			goto config_failure;
		}
	}

	switch (cmd->cmd_type) {
	case CMD_STATS_AEC_AWB_ENABLE:
	case CMD_STATS_AXI_CFG: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}

		scfg =
			kmalloc(sizeof(struct vfe_stats_we_cfg),
				GFP_ATOMIC);
		if (!scfg) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(scfg,
					(void __user *)(vfecmd->value),
					vfecmd->length)) {

			rc = -EFAULT;
			goto config_done;
		}

		CDBG("STATS_ENABLE: bufnum = %d, enabling = %d\n",
			axid->bufnum1, scfg->wb_expstatsenable);

		if (axid->bufnum1 > 0) {
			regptr = axid->region;

			for (i = 0; i < axid->bufnum1; i++) {

				CDBG("STATS_ENABLE, phy = 0x%lx\n",
					regptr->paddr);

				scfg->wb_expstatoutputbuffer[i] =
					(void *)regptr->paddr;
				regptr++;
			}

			cmd_data = scfg;

		} else {
			rc = -EINVAL;
			goto config_done;
		}
	}
		break;

	case CMD_STATS_AF_ENABLE:
	case CMD_STATS_AF_AXI_CFG: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}

		sfcfg =
			kmalloc(sizeof(struct vfe_stats_af_cfg),
				GFP_ATOMIC);

		if (!sfcfg) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(sfcfg,
					(void __user *)(vfecmd->value),
					vfecmd->length)) {

			rc = -EFAULT;
			goto config_done;
		}

		CDBG("AF_ENABLE: bufnum = %d, enabling = %d\n",
			axid->bufnum1, sfcfg->af_enable);

		if (axid->bufnum1 > 0) {
			regptr = axid->region;

			for (i = 0; i < axid->bufnum1; i++) {

				CDBG("STATS_ENABLE, phy = 0x%lx\n",
					regptr->paddr);

				sfcfg->af_outbuf[i] =
					(void *)regptr->paddr;

				regptr++;
			}

			cmd_data = sfcfg;

		} else {
			rc = -EINVAL;
			goto config_done;
		}
	}
		break;

	case CMD_FRAME_BUF_RELEASE: {
		struct msm_frame *b;
		unsigned long p;
		struct vfe_outputack fack;
		if (!data)  {
			rc = -EFAULT;
			goto config_failure;
		}

		b = (struct msm_frame *)(cmd->value);
		p = *(unsigned long *)data;

		fack.header = VFE_FRAME_ACK;

		fack.output2newybufferaddress =
			(void *)(p + b->y_off);

		fack.output2newcbcrbufferaddress =
			(void *)(p + b->cbcr_off);

		vfecmd->queue = QDSP_CMDQUEUE;
		vfecmd->length = sizeof(struct vfe_outputack);
		cmd_data = &fack;
	}
		break;

	case CMD_SNAP_BUF_RELEASE:
		break;

	case CMD_STATS_BUF_RELEASE: {
		CDBG("vfe_7x_config: CMD_STATS_BUF_RELEASE\n");
		if (!data) {
			rc = -EFAULT;
			goto config_failure;
		}

		sack.header = STATS_WE_ACK;
		sack.bufaddr = (void *)*(uint32_t *)data;

		vfecmd->queue  = QDSP_CMDQUEUE;
		vfecmd->length = sizeof(struct vfe_stats_ack);
		cmd_data = &sack;
	}
		break;

	case CMD_STATS_AF_BUF_RELEASE: {
		CDBG("vfe_7x_config: CMD_STATS_AF_BUF_RELEASE\n");
		if (!data) {
			rc = -EFAULT;
			goto config_failure;
		}

		sack.header = STATS_AF_ACK;
		sack.bufaddr = (void *)*(uint32_t *)data;

		vfecmd->queue  = QDSP_CMDQUEUE;
		vfecmd->length = sizeof(struct vfe_stats_ack);
		cmd_data = &sack;
	}
		break;

	case CMD_GENERAL:
	case CMD_STATS_DISABLE: {
		if (vfecmd->length > sizeof(buf)) {
			cmd_data_alloc =
			cmd_data = kmalloc(vfecmd->length, GFP_ATOMIC);
			if (!cmd_data) {
				rc = -ENOMEM;
				goto config_failure;
			}
		} else
			cmd_data = buf;

		if (copy_from_user(cmd_data,
					(void __user *)(vfecmd->value),
					vfecmd->length)) {

			rc = -EFAULT;
			goto config_done;
		}

		if (vfecmd->queue == QDSP_CMDQUEUE) {
			switch (*(uint32_t *)cmd_data) {
			case VFE_RESET_CMD:
				msm_camio_vfe_blk_reset();
				vfestopped = 0;
				break;

			case VFE_START_CMD:
				msm_camio_camif_pad_reg_reset_2();
				vfestopped = 0;
				break;

			case VFE_STOP_CMD:
				vfestopped = 1;
				goto config_send;

			default:
				break;
			}
		} /* QDSP_CMDQUEUE */
	}
		break;

	case CMD_AXI_CFG_OUT1: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}

		axio = kmalloc(sizeof(struct axiout), GFP_ATOMIC);
		if (!axio) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(axio, (void *)(vfecmd->value),
					sizeof(struct axiout))) {
			rc = -EFAULT;
			goto config_done;
		}

		vfe_7x_config_axi(OUTPUT_1, axid, axio);

		cmd_data = axio;
	}
		break;

	case CMD_AXI_CFG_OUT2:
	case CMD_RAW_PICT_AXI_CFG: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}

		axio = kmalloc(sizeof(struct axiout), GFP_ATOMIC);
		if (!axio) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(axio, (void __user *)(vfecmd->value),
					sizeof(struct axiout))) {
			rc = -EFAULT;
			goto config_done;
		}

		vfe_7x_config_axi(OUTPUT_2, axid, axio);
		cmd_data = axio;
	}
		break;

	case CMD_AXI_CFG_SNAP_O1_AND_O2: {
		axid = data;
		if (!axid) {
			rc = -EFAULT;
			goto config_failure;
		}

		axio = kmalloc(sizeof(struct axiout), GFP_ATOMIC);
		if (!axio) {
			rc = -ENOMEM;
			goto config_failure;
		}

		if (copy_from_user(axio, (void __user *)(vfecmd->value),
					sizeof(struct axiout))) {
			rc = -EFAULT;
			goto config_done;
		}

		vfe_7x_config_axi(OUTPUT_1_AND_2, axid, axio);

		cmd_data = axio;
	}
		break;

	default:
		break;
	} /* switch */

	if (vfestopped)
		goto config_done;

config_send:
	CDBG("send adsp command = %d\n", *(uint32_t *)cmd_data);
	rc = msm_adsp_write(vfe_mod, vfecmd->queue,
				cmd_data, vfecmd->length);

config_done:
	if (cmd_data_alloc != NULL)
		kfree(cmd_data_alloc);

config_failure:
	kfree(scfg);
	kfree(axio);
	kfree(vfecmd);
	return rc;
}
#endif

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
void msm_camvfe_fn_init(struct msm_camvfe_fn_t *fptr)
#else
void msm_camvfe_fn_init(struct msm_camvfe_fn *fptr, void *data)
#endif
{
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
	 mutex_init(&vfe_lock);
#endif
	fptr->vfe_init    = vfe_7x_init;
	fptr->vfe_enable  = vfe_7x_enable;
	fptr->vfe_config  = vfe_7x_config;
	fptr->vfe_disable = vfe_7x_disable;
	fptr->vfe_release = vfe_7x_release;
#if !defined(CONFIG_MACH_MOT) && !defined(CONFIG_MACH_PITTSBURGH)
	vfe_syncdata = data;
#endif
}

#if defined(CONFIG_MACH_MOT) || defined(CONFIG_MACH_PITTSBURGH)
int msm_camvfe_check(void *data)
{
	int rc = 0;

	mutex_lock(&vfe_lock);
	if (!vfe_inuse) {
		vfe_inuse = 1;
		vfe_syncdata = data;
	} else if (data != vfe_syncdata) {
		rc = -EBUSY;
	}
	mutex_unlock(&vfe_lock);

	return rc;
}

void msm_camvfe_init(void)
{
	mutex_init(&vfe_lock);
	vfe_inuse = 0;
	vfe_syncdata = NULL;
}
#endif

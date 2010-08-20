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

#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include "msm_vfe31.h"
#include <mach/camera.h>
#include <linux/io.h>

static struct vfe31_ctrl_type *vfe31_ctrl;

struct vfe31_isr_queue_cmd {
	struct list_head list;
	uint32_t /* struct vfe_interrupt_status_0 */      vfeInterruptStatus0;
	uint32_t /* struct vfe_interrupt_status_1 */      vfeInterruptStatus1;
	struct vfe_frame_asf_info          vfeAsfFrameInfo;
	struct vfe_frame_bpc_info          vfeBpcFrameInfo;
	struct vfe_msg_camif_status        vfeCamifStatusLocal;
};

static struct vfe31_cmd_type vfe31_cmd[] = {
	{V31_DUMMY_0},
	{V31_SET_CLK},
	{V31_RESET},
	{V31_START},
	{V31_TEST_GEN_START},
	{V31_OPERATION_CFG, V31_OPERATION_CFG_LEN},
	{V31_AXI_OUT_CFG, V31_AXI_OUT_LEN, V31_AXI_OUT_OFF, 0xFF},
	{V31_CAMIF_CFG, V31_CAMIF_LEN, V31_CAMIF_OFF, 0xFF},
	{V31_AXI_INPUT_CFG},
	{V31_BLACK_LEVEL_CFG, V31_BLACK_LEVEL_LEN, V31_BLACK_LEVEL_OFF, 0xFF},
	{V31_ROLL_OFF_CFG, V31_ROLL_OFF_CFG_LEN, V31_ROLL_OFF_CFG_OFF, 0xFF},
	{V31_DEMUX_CFG, V31_DEMUX_LEN, V31_DEMUX_OFF, 0xFF},
	{V31_DEMOSAIC_0_CFG, V31_DEMOSAIC_0_LEN, V31_DEMOSAIC_0_OFF, 0xFF},
	{V31_DEMOSAIC_1_CFG, V31_DEMOSAIC_1_LEN, V31_DEMOSAIC_1_OFF, 0xFF},
	{V31_DEMOSAIC_2_CFG, V31_DEMOSAIC_2_LEN, V31_DEMOSAIC_2_OFF, 0xFF},
	{V31_FOV_CFG, V31_FOV_LEN, V31_FOV_OFF, 0xFF},
	{V31_MAIN_SCALER_CFG, V31_MAIN_SCALER_LEN, V31_MAIN_SCALER_OFF, 0xFF},
	{V31_WB_CFG, V31_WB_LEN, V31_WB_OFF, 0xFF},
	{V31_COLOR_COR_CFG, V31_COLOR_COR_LEN, V31_COLOR_COR_OFF, 0xFF},
	{V31_RGB_G_CFG, V31_RGB_G_LEN, V31_RGB_G_OFF, 0xFF},
	{V31_LA_CFG, V31_LA_LEN, V31_LA_OFF, 0xFF },
	{V31_CHROMA_EN_CFG, V31_CHROMA_EN_LEN, V31_CHROMA_EN_OFF, 0xFF},
	{V31_CHROMA_SUP_CFG, V31_CHROMA_SUP_LEN, V31_CHROMA_SUP_OFF, 0xFF},
	{V31_MCE_CFG, V31_MCE_LEN, V31_MCE_OFF, 0xFF},
	{V31_SK_ENHAN_CFG},
	{V31_ASF_CFG},
	{V31_S2Y_CFG, V31_S2Y_LEN, V31_S2Y_OFF, 0xFF},
	{V31_S2CbCr_CFG, V31_S2CbCr_LEN, V31_S2CbCr_OFF, 0xFF},
	{V31_CHROMA_SUBS_CFG, V31_CHROMA_SUBS_LEN, V31_CHROMA_SUBS_OFF, 0xFF},
	{V31_OUT_CLAMP_CFG, V31_OUT_CLAMP_LEN, V31_OUT_CLAMP_OFF, 0xFF},
	{V31_FRAME_SKIP_CFG, V31_FRAME_SKIP_LEN, V31_FRAME_SKIP_OFF, 0xFF},
	{V31_DUMMY_1},
	{V31_DUMMY_2},
	{V31_DUMMY_3},
	{V31_UPDATE},
	{V31_BL_LVL_UPDATE, V31_BLACK_LEVEL_LEN, V31_BLACK_LEVEL_OFF, 0xFF},
	{V31_DEMUX_UPDATE, V31_DEMUX_LEN, V31_DEMUX_OFF, 0xFF},
	{V31_DEMOSAIC_1_UPDATE, V31_DEMOSAIC_1_LEN, V31_DEMOSAIC_1_OFF, 0xFF},
	{V31_DEMOSAIC_2_UPDATE, V31_DEMOSAIC_2_LEN, V31_DEMOSAIC_2_OFF, 0xFF},
	{V31_FOV_UPDATE, V31_FOV_LEN, V31_FOV_OFF, 0xFF},
	{V31_MAIN_SCALER_UPDATE, V31_MAIN_SCALER_LEN, V31_MAIN_SCALER_OFF,
	0xFF},
	{V31_WB_UPDATE, V31_WB_LEN, V31_WB_OFF, 0xFF},
	{V31_COLOR_COR_UPDATE},
	{V31_RGB_G_UPDATE, V31_RGB_G_LEN, V31_CHROMA_EN_OFF, 0xFF},
	{V31_LA_UPDATE, V31_LA_LEN, V31_LA_OFF, 0xFF },
	{V31_CHROMA_EN_UPDATE, V31_CHROMA_EN_LEN, V31_CHROMA_EN_OFF, 0xFF},
	{V31_CHROMA_SUP_UPDATE, V31_CHROMA_SUP_LEN, V31_CHROMA_SUP_OFF, 0xFF},
	{V31_MCE_UPDATE, V31_MCE_LEN, V31_MCE_OFF, 0xFF},
	{V31_SK_ENHAN_UPDATE},
	{V31_S2CbCr_UPDATE, V31_S2CbCr_LEN, V31_S2CbCr_OFF, 0xFF},
	{V31_S2Y_UPDATE, V31_S2Y_LEN, V31_S2Y_OFF, 0xFF},
	{V31_ASF_UPDATE},
	{V31_FRAME_SKIP_UPDATE},
	{V31_CAMIF_FRAME_UPDATE},
	{V31_STATS_AF_UPDATE},
	{V31_STATS_AE_UPDATE},
	{V31_STATS_AWB_UPDATE},
	{V31_STATS_RS_UPDATE},
	{V31_STATS_CS_UPDATE},
	{V31_STATS_SKIN_UPDATE},
	{V31_STATS_IHIST_UPDATE},
	{V31_DUMMY_4},
	{V31_EPOCH1_ACK},
	{V31_EPOCH2_ACK},
	{V31_DUMMY_5},
	{V31_DUMMY_6},
	{V31_CAPTURE},
	{V31_DUMMY_7},
	{V31_STOP},
	{V31_GET_HW_VERSION},
	{V31_GET_FRAME_SKIP_COUNTS},
	{V31_OUTPUT1_BUFFER_ENQ},
	{V31_OUTPUT2_BUFFER_ENQ},
	{V31_OUTPUT3_BUFFER_ENQ},
	{V31_JPEG_OUT_BUF_ENQ},
	{V31_RAW_OUT_BUF_ENQ},
	{V31_RAW_IN_BUF_ENQ},
	{V31_STATS_AF_ENQ},
	{V31_STATS_AE_ENQ},
	{V31_STATS_AWB_ENQ},
	{V31_STATS_RS_ENQ},
	{V31_STATS_CS_ENQ},
	{V31_STATS_SKIN_ENQ},
	{V31_STATS_IHIST_ENQ},
	{V31_DUMMY_8},
	{V31_JPEG_ENC_CFG},
	{V31_DUMMY_9},
	{V31_STATS_AF_START},
	{V31_STATS_AF_STOP},
	{V31_STATS_AE_START},
	{V31_STATS_AE_STOP},
	{V31_STATS_AWB_START},
	{V31_STATS_AWB_STOP},
	{V31_STATS_RS_START},
	{V31_STATS_RS_STOP},
	{V31_STATS_CS_START},
	{V31_STATS_CS_STOP},
	{V31_STATS_SKIN_START},
	{V31_STATS_SKIN_STOP},
	{V31_STATS_IHIST_START},
	{V31_STATS_IHIST_STOP},
	{V31_DUMMY_10},
	{V31_SYNC_TIMER_SETTING},
	{V31_ASYNC_TIMER_SETTING},
};

static void vfe_addr_convert(struct msm_vfe_phy_info *pinfo,
	enum vfe_resp_msg type, void *data, void **ext, int32_t *elen)
{
	switch (type) {
	case VFE_MSG_OUTPUT1: {
		pinfo->output_mode =
			((struct vfe_message *)data)->_u.msgOut.output_mode;
		pinfo->y_phy =
			((struct vfe_message *)data)->_u.msgOut.yBuffer;
		pinfo->cbcr_phy =
			((struct vfe_message *)data)->_u.msgOut.cbcrBuffer;

		((struct vfe_msg_output *)(vfe31_ctrl->extdata))->bpcInfo =
		((struct vfe_message *)data)->_u.msgOut.bpcInfo;

		((struct vfe_msg_output *)(vfe31_ctrl->extdata))->asfInfo =
		((struct vfe_message *)data)->_u.msgOut.asfInfo;

		((struct vfe_msg_output *)(vfe31_ctrl->extdata))->frameCounter =
		((struct vfe_message *)data)->_u.msgOut.frameCounter;

		*ext  = vfe31_ctrl->extdata;
		*elen = vfe31_ctrl->extlen;
	}
		break;

	case VFE_MSG_STATS_AF:
		pinfo->sbuf_phy =
		((struct vfe_message *)data)->_u.msgStatsAf.afBuffer;
		break;

	case VFE_MSG_STATS_WE:
		pinfo->sbuf_phy =
		((struct vfe_message *)data)->_u.msgStatsWbExp.awbBuffer;
		break;

	default:
		break;
	}
}

static void  *vfe_syncdata;

static void vfe31_proc_ops(enum VFE31_MESSAGE_ID id, void *msg, size_t len)
{
	struct msm_vfe_resp *rp;

	rp = vfe31_ctrl->resp->vfe_alloc(sizeof(struct msm_vfe_resp),
		vfe31_ctrl->syncdata, GFP_ATOMIC);
	if (!rp) {
		CDBG("rp: cannot allocate buffer\n");
		return;
	}

	CDBG("vfe31_proc_ops, msgId = %d\n", id);

	rp->evt_msg.type   = MSM_CAMERA_MSG;
	rp->evt_msg.msg_id = id;
	rp->evt_msg.len    = len;
	rp->evt_msg.data   = msg;

	switch (rp->evt_msg.msg_id) {
	case MSG_ID_SNAPSHOT_DONE:
		rp->type = VFE_MSG_SNAPSHOT;
		break;

	case MSG_ID_OUTPUT:
		rp->type = VFE_MSG_OUTPUT1;
		vfe_addr_convert(&(rp->phy), VFE_MSG_OUTPUT1,
			rp->evt_msg.data, &(rp->extdata),
			&(rp->extlen));
		break;

	case MSG_ID_STATS_AUTOFOCUS:
		rp->type = VFE_MSG_STATS_AF;
		vfe_addr_convert(&(rp->phy), VFE_MSG_STATS_AF,
				rp->evt_msg.data, NULL, NULL);
		break;

	case MSG_ID_STATS_WB_EXP:
		rp->type = VFE_MSG_STATS_WE;
		vfe_addr_convert(&(rp->phy), VFE_MSG_STATS_WE,
				rp->evt_msg.data, NULL, NULL);
		break;

	default:
		rp->type = VFE_MSG_GENERAL;
		break;
	}

	vfe31_ctrl->resp->vfe_resp(rp, MSM_CAM_Q_VFE_MSG, vfe31_ctrl->syncdata,
		GFP_ATOMIC);
}

static void vfe_send_outmsg(uint8_t output_mode, uint32_t pyaddr,
	uint32_t pcbcraddr)
{
	struct vfe_message *msg;

	msg = kzalloc(sizeof(struct vfe_message), GFP_ATOMIC);
	if (!msg)
		return;

	msg->_d = MSG_ID_OUTPUT;
	msg->_u.msgOut.output_mode = output_mode;
	msg->_u.msgOut.yBuffer     = pyaddr;
	msg->_u.msgOut.cbcrBuffer  = pcbcraddr;

	vfe31_proc_ops(MSG_ID_OUTPUT, msg, sizeof(struct vfe_message));

	return;
}


static int vfe31_enable(struct camera_enable_cmd *enable)
{
	return 0;
}

void vfe_stop(void)
{
	uint8_t  axiBusyFlag = true;

	/* for reset hw modules, and send msg when reset_irq comes.*/
	vfe31_ctrl->stop_ack_pending = TRUE;

	/* disable all interrupts.  */
	msm_io_w(VFE_DISABLE_ALL_IRQS,
		vfe31_ctrl->vfebase + VFE_IRQ_MASK_0);
	msm_io_w(VFE_DISABLE_ALL_IRQS,
		vfe31_ctrl->vfebase + VFE_IRQ_MASK_1);

	/* clear all pending interrupts*/
	msm_io_w(VFE_CLEAR_ALL_IRQS,
		vfe31_ctrl->vfebase + VFE_IRQ_CLEAR_0);
	msm_io_w(VFE_CLEAR_ALL_IRQS,
		vfe31_ctrl->vfebase + VFE_IRQ_CLEAR_1);
	msm_io_w(1,
		vfe31_ctrl->vfebase + VFE_IRQ_CMD);

	/* in either continuous or snapshot mode, stop command can be issued
	 * at any time. stop camif immediately. */
	msm_io_w(CAMIF_COMMAND_STOP_IMMEDIATELY,
		vfe31_ctrl->vfebase + VFE_CAMIF_COMMAND);

	/* axi halt command. */
	msm_io_w(AXI_HALT,
		vfe31_ctrl->vfebase + VFE_AXI_CMD);
	while (axiBusyFlag) {
		if (msm_io_r(vfe31_ctrl->vfebase + VFE_AXI_STATUS) & 0x1)
			axiBusyFlag = false;
	}
	msm_io_w(AXI_HALT_CLEAR,
		vfe31_ctrl->vfebase + VFE_AXI_CMD);

	/* after axi halt, then ok to apply global reset. */
	/* enable reset_ack and async timer interrupt only while
	stopping the pipeline.*/
	msm_io_w(0xf0000000,
		vfe31_ctrl->vfebase + VFE_IRQ_MASK_0);
	msm_io_w(VFE_IMASK_WHILE_STOPPING_1,
		vfe31_ctrl->vfebase + VFE_IRQ_MASK_1);

	msm_io_w(VFE_RESET_UPON_STOP_CMD,
		vfe31_ctrl->vfebase + VFE_GLOBAL_RESET);
}

static int vfe31_disable(struct camera_enable_cmd *enable,
	struct platform_device *dev)
{
	vfe_stop();

	msm_camio_disable(dev);
	msm_disable_io_gpio_clk(dev);
	return 0;
}

static void vfe31_release(struct platform_device *pdev)
{
	struct resource	*vfemem, *vfeio;

	vfemem = vfe31_ctrl->vfemem;
	vfeio  = vfe31_ctrl->vfeio;

	kfree(vfe31_ctrl->extdata);
	free_irq(vfe31_ctrl->vfeirq, 0);
	iounmap(vfe31_ctrl->vfebase);
	kfree(vfe31_ctrl);
	vfe31_ctrl = NULL;
	release_mem_region(vfemem->start, (vfemem->end - vfemem->start) + 1);
	msm_camio_disable(pdev);

	vfe_syncdata = NULL;
}

static int vfe31_config_axi(int mode, struct axidata *ad, uint32_t *ao)
{
	int i;
	uint32_t *p, *p1;
	struct vfe31_output_ch *outp1, *outp2;
	struct msm_pmem_region *regp1 = NULL;

	outp1 = NULL;
	outp2 = NULL;

	p = ao + 2;
	*p = 0x200;

	CDBG("vfe31_config_axi: mode = %d, bufnum1 = %d, bufnum2 = %d\n",
		mode, ad->bufnum1, ad->bufnum2);

	switch (mode) {
	case OUTPUT_1:
		if (ad->bufnum1 != 3)
			return -EINVAL;

		vfe31_ctrl->outpath.out0.ch0 = 0; /* luma   */
		vfe31_ctrl->outpath.out0.ch1 = 1; /* chroma */
		regp1 = ad->region;
		outp1 = &(vfe31_ctrl->outpath.out0);
		vfe31_ctrl->outpath.output_mode |= VFE31_OUTPUT_MODE_P;
		break;

	case OUTPUT_2:
		if (ad->bufnum2 != 3)
			return -EINVAL;

		vfe31_ctrl->outpath.out0.ch0 = 0; /* luma   */
		vfe31_ctrl->outpath.out0.ch1 = 1; /* chroma */
		regp1 = &(ad->region[ad->bufnum1]);
		outp1 = &(vfe31_ctrl->outpath.out0);
		vfe31_ctrl->outpath.output_mode |= VFE31_OUTPUT_MODE_P;
		break;

	case OUTPUT_1_AND_2:
		if ((ad->bufnum1 != 3) ||
				(ad->bufnum2 != 3))
			return -EINVAL;

		vfe31_ctrl->outpath.out0.ch0 = 0; /* luma   */
		vfe31_ctrl->outpath.out0.ch1 = 1; /* chroma */

		vfe31_ctrl->outpath.out1.ch0 = 3; /* luma   */
		vfe31_ctrl->outpath.out1.ch1 = 4; /* chroma */
		vfe31_ctrl->outpath.output_mode |= VFE31_OUTPUT_MODE_S;
		break;

	default:
		break;
	}

	if (outp1) {
		for (i = 0; i < 2; i++) {
			p1 = ao + 6 + i;
			*p1 = (regp1->paddr + regp1->info.y_off);

			p1 = ao + 12 + i;
			*p1 = (regp1->paddr + regp1->info.cbcr_off);

			regp1++;
		}

		outp1->free_buf.available = 1;
		outp1->free_buf.paddr = regp1->paddr;
		outp1->free_buf.y_off = regp1->info.y_off;
		outp1->free_buf.cbcr_off = regp1->info.cbcr_off;

		CDBG("vfe31_config_axi: free_buf paddr = 0x%x, y_off = %d,"
			" cbcr_off = %d\n",
			outp1->free_buf.paddr, outp1->free_buf.y_off,
			outp1->free_buf.cbcr_off);
	}

	msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[V31_AXI_OUT_CFG].offset,
		ao, vfe31_cmd[V31_AXI_OUT_CFG].length);
	return 0;
}

static void vfe31_reset_internal_variables(void)
{
	unsigned long flags;

	vfe31_ctrl->vfeImaskCompositePacked = 0;

	/* state control variables */
	vfe31_ctrl->start_ack_pending = 0;
	vfe31_ctrl->stop_ack_pending  = 0;
	vfe31_ctrl->reset_ack_pending  = 0;
	vfe31_ctrl->update_ack_pending = 0;

	spin_lock_irqsave(&vfe31_ctrl->state_lock, flags);
	vfe31_ctrl->vstate = VFE_STATE_IDLE;
	spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);

	vfe31_ctrl->operation_mode = 0;

	/* 0 for continuous mode, 1 for snapshot mode */
	vfe31_ctrl->vfe_snapshot_count = 0;

	/* this is unsigned 32 bit integer. */
	vfe31_ctrl->vfeFrameId = 0;
	vfe31_ctrl->output1Pattern = 0xffffffff;
	vfe31_ctrl->output1Period  = 31;
	vfe31_ctrl->output2Pattern = 0xffffffff;
	vfe31_ctrl->output2Period  = 31;
	vfe31_ctrl->vfeFrameSkipCount   = 0;
	vfe31_ctrl->vfeFrameSkipPeriod  = 31;
}

static void vfe31_reset(void)
{
	uint32_t vfe_version;

	vfe31_reset_internal_variables();

	vfe_version = msm_io_r(vfe31_ctrl->vfebase);

	CDBG("vfe_version = 0x%x\n", vfe_version);

	/* disable all interrupts.  vfeImaskLocal is also reset to 0
	* to begin with. */
	msm_io_w(VFE_DISABLE_ALL_IRQS,
		vfe31_ctrl->vfebase + VFE_IRQ_MASK_0);

	msm_io_w(VFE_DISABLE_ALL_IRQS,
		vfe31_ctrl->vfebase + VFE_IRQ_MASK_1);

	/* clear all pending interrupts*/
	msm_io_w(VFE_CLEAR_ALL_IRQS, vfe31_ctrl->vfebase + VFE_IRQ_CLEAR_0);
	msm_io_w(VFE_CLEAR_ALL_IRQS, vfe31_ctrl->vfebase + VFE_IRQ_CLEAR_1);
	msm_io_w(1, vfe31_ctrl->vfebase + VFE_IRQ_CMD);

	/* enable reset_ack interrupt.  */
	msm_io_w(VFE_IMASK_WHILE_STOPPING_1,
		vfe31_ctrl->vfebase + VFE_IRQ_MASK_1);

	/* Write to VFE_GLOBAL_RESET_CMD to reset the vfe hardware. Once reset
	 * is done, hardware interrupt will be generated.  VFE ist processes
	 * the interrupt to complete the function call.  Note that the reset
	 * function is synchronous. */
	msm_io_w(VFE_RESET_UPON_RESET_CMD,
		vfe31_ctrl->vfebase + VFE_GLOBAL_RESET);
}

static int vfe31_operation_config(uint32_t *cmd)
{
	uint32_t *p = cmd;

	vfe31_ctrl->operation_mode = *p;
	vfe31_ctrl->stats_comp = *(++p);

	msm_io_w(*(++p), vfe31_ctrl->vfebase + VFE_CFG_OFF);
	msm_io_w(*(++p), vfe31_ctrl->vfebase + VFE_MODULE_CFG);
	msm_io_w(*(++p), vfe31_ctrl->vfebase + VFE_REALIGN_BUF);
	msm_io_w(*(++p), vfe31_ctrl->vfebase + VFE_CHROMA_UP);
	msm_io_w(*(++p), vfe31_ctrl->vfebase + VFE_STATS_CFG);
	return 0;
}

static int vfe31_start(void)
{
	unsigned long flags;
	uint32_t irq_comp_mask;

	/* start command now is only good for continuous mode. */
	if (vfe31_ctrl->operation_mode == 1)
		return 0;

	vfe31_ctrl->start_ack_pending = 1;

	irq_comp_mask	=
		msm_io_r(vfe31_ctrl->vfebase + VFE_IRQ_COMP_MASK);

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_P) {
		irq_comp_mask |= (0x1 << vfe31_ctrl->outpath.out0.ch0 |
			0x1 << vfe31_ctrl->outpath.out0.ch1);
	}

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S) {
		irq_comp_mask |= (0x1 << (vfe31_ctrl->outpath.out0.ch0 + 7) |
			0x1 << (vfe31_ctrl->outpath.out0.ch1 + 7));
	}

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_V) {
		irq_comp_mask |= (0x1 << (vfe31_ctrl->outpath.out0.ch0 + 14)|
			0x1 << (vfe31_ctrl->outpath.out0.ch1 + 14));
	}

	msm_io_w(irq_comp_mask, vfe31_ctrl->vfebase + VFE_IRQ_COMP_MASK);

	msm_io_w(0x00EFE021, vfe31_ctrl->vfebase + VFE_IRQ_MASK_0);
	msm_io_w(VFE_IMASK_WHILE_STOPPING_1,
		vfe31_ctrl->vfebase + VFE_IRQ_MASK_1);

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_P) {
		msm_io_w(1, vfe31_ctrl->vfebase + V31_AXI_OUT_OFF + 20 +
			24 * (vfe31_ctrl->outpath.out0.ch0));

		msm_io_w(1, vfe31_ctrl->vfebase + V31_AXI_OUT_OFF + 20 +
			24 * (vfe31_ctrl->outpath.out0.ch1));
	}

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S) {
		msm_io_w(1, vfe31_ctrl->vfebase + V31_AXI_OUT_OFF + 20 +
			24 * (vfe31_ctrl->outpath.out1.ch0));

		msm_io_w(1, vfe31_ctrl->vfebase + V31_AXI_OUT_OFF + 20 +
			24 * (vfe31_ctrl->outpath.out1.ch1));
	}

	if (vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_V) {
		msm_io_w(1, vfe31_ctrl->vfebase + V31_AXI_OUT_OFF + 20 +
			24 * (vfe31_ctrl->outpath.out2.ch0));

		msm_io_w(1, vfe31_ctrl->vfebase + V31_AXI_OUT_OFF + 20 +
			24 * (vfe31_ctrl->outpath.out2.ch1));
	}

	msm_io_dump(vfe31_ctrl->vfebase, 0x600);
	dmb();
	msm_io_w(1, vfe31_ctrl->vfebase + VFE_REG_UPDATE_CMD);
	dmb();
	msm_io_w(1, vfe31_ctrl->vfebase + VFE_CAMIF_COMMAND);

	spin_lock_irqsave(&vfe31_ctrl->state_lock, flags);
	vfe31_ctrl->vstate = VFE_STATE_ACTIVE;
	spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);
	return 0;
}
#define CHECKED_COPY_FROM_USER(in) {					\
	if (copy_from_user((in), (void __user *)cmd->value,		\
			cmd->length)) {					\
		rc = -EFAULT;						\
		break;							\
	}								\
}
void vfe31_program_dmi_cfg(enum VFE31_DMI_RAM_SEL bankSel)
{
	/* set bit 8 for auto increment. */
	uint32_t value = VFE_DMI_CFG_DEFAULT;
	value += (uint32_t)bankSel;

	msm_io_w(value, vfe31_ctrl->vfebase + VFE_DMI_CFG);
	/* by default, always starts with offset 0.*/
	msm_io_w(0, vfe31_ctrl->vfebase + VFE_DMI_ADDR);
}
void vfe31_write_gamma_cfg(enum VFE31_DMI_RAM_SEL channel_sel,
						const uint32_t *tbl)
{
	int i;
	uint32_t value, value1, value2;
	vfe31_program_dmi_cfg(channel_sel);
	/* for loop for extracting init table. */
	for (i = 0 ; i < (VFE31_GAMMA_NUM_ENTRIES/2) ; i++) {
		value = *tbl++;
		value1 = value & 0x0000FFFF;
		value2 = (value & 0xFFFF0000)>>16;
		msm_io_w((value1), vfe31_ctrl->vfebase + VFE_DMI_DATA_LO);
		msm_io_w((value2), vfe31_ctrl->vfebase + VFE_DMI_DATA_LO);
	}
	vfe31_program_dmi_cfg(NO_MEM_SELECTED);
}

static int vfe31_proc_general(struct msm_vfe31_cmd *cmd)
{
	int i , rc = 0;
	uint32_t old_val = 0 , new_val = 0;
	uint32_t *cmdp = NULL;
	uint32_t *cmdp_local = NULL;
	switch (cmd->id) {
	case V31_RESET:
		vfe31_reset();
		break;

	case V31_START:
		rc = vfe31_start();
		break;

	case V31_OPERATION_CFG: {
		if (cmd->length != V31_OPERATION_CFG_LEN) {
			rc = -EINVAL;
			goto proc_general_done;
		}

		cmdp = kmalloc(V31_OPERATION_CFG_LEN, GFP_ATOMIC);
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			V31_OPERATION_CFG_LEN)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		rc = vfe31_operation_config(cmdp);
		}
		break;
	case V31_MCE_UPDATE:
	case V31_MCE_CFG:{
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		/* Incrementing with 4 so as to point to the 2nd Register as
		the 2nd register has the mce_enable bit */
		old_val = msm_io_r(vfe31_ctrl->vfebase + V31_CHROMA_EN_OFF + 4);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		cmdp_local = cmdp;
		new_val = *cmdp_local;
		old_val &= MCE_EN_MASK;
		new_val = new_val | old_val;
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_CHROMA_EN_OFF + 4,
					&new_val, 4);
		cmdp_local += 1;

		old_val = msm_io_r(vfe31_ctrl->vfebase + V31_CHROMA_EN_OFF + 8);
		new_val = *cmdp_local;
		old_val &= MCE_Q_K_MASK;
		new_val = new_val | old_val;
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_CHROMA_EN_OFF + 8,
		&new_val, 4);
		cmdp_local += 1;
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
		cmdp_local, (vfe31_cmd[cmd->id].length));
		}
		break;
	case V31_DEMOSAIC_2_UPDATE:
	case V31_DEMOSAIC_2_CFG: {
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		old_val = msm_io_r(vfe31_ctrl->vfebase + V31_DEMOSAIC_0_OFF);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		cmdp_local = cmdp;
		new_val = *cmdp_local;
		old_val &= BPC_MASK;
		new_val = new_val | old_val;
		*cmdp_local = new_val;
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_DEMOSAIC_0_OFF,
					cmdp_local, 4);
		cmdp_local += 1;
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
			cmdp_local, (vfe31_cmd[cmd->id].length));
		}
		break;
	case V31_DEMOSAIC_1_UPDATE:
	case V31_DEMOSAIC_1_CFG: {
		uint32_t temp;
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		cmdp_local = cmdp;
		temp = msm_io_r(vfe31_ctrl->vfebase + V31_DEMOSAIC_0_OFF);
		*cmdp_local = *cmdp_local | temp;
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_DEMOSAIC_0_OFF,
		cmdp_local, 4);
		cmdp_local += 1;
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
		cmdp_local, (vfe31_cmd[cmd->id].length));
		}
		break;
	case V31_ROLL_OFF_CFG: {
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value) , cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		cmdp_local = cmdp;
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
		cmdp_local, 16);
		cmdp_local += 4;
		vfe31_program_dmi_cfg(ROLLOFF_RAM);
		/* for loop for extrcting init table. */
		for (i = 0 ; i < (VFE31_ROLL_OFF_INIT_TABLE_SIZE * 2) ; i++) {
			msm_io_w(*cmdp_local ,
			vfe31_ctrl->vfebase + VFE_DMI_DATA_LO);
			cmdp_local++;
		}
		CDBG("done writing init table \n");
		/* by default, always starts with offset 0. */
		msm_io_w(LENS_ROLL_OFF_DELTA_TABLE_OFFSET,
		vfe31_ctrl->vfebase + VFE_DMI_ADDR);
		/* for loop for extracting delta table. */
		for (i = 0 ; i < (VFE31_ROLL_OFF_DELTA_TABLE_SIZE * 2) ; i++) {
			msm_io_w(*cmdp_local,
			vfe31_ctrl->vfebase + VFE_DMI_DATA_LO);
			cmdp_local++;
		}
		vfe31_program_dmi_cfg(NO_MEM_SELECTED);
		}
		break;

	case V31_LA_CFG: {
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {

			rc = -EFAULT;
			goto proc_general_done;
		}
		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
				cmdp, (vfe31_cmd[cmd->id].length));

		/* If the value is 0x00 then write in to LUT bank0
		if the value is 0x01 then write in to LUT bank1 */
		if (*cmdp == 0x0)
			vfe31_program_dmi_cfg(LUMA_ADAPT_LUT_RAM_BANK0);
		else
			vfe31_program_dmi_cfg(LUMA_ADAPT_LUT_RAM_BANK1);
		cmdp += 1;
		/* for loop for extracting init table. */
		for (i = 0 ; i < VFE31_LA_TABLE_LENGTH ; i++) {
			msm_io_w(*cmdp, vfe31_ctrl->vfebase + VFE_DMI_DATA_LO);
			cmdp++;
		}
		vfe31_program_dmi_cfg(NO_MEM_SELECTED);
		cmdp -= 1;
		}
		break;
	case V31_RGB_G_UPDATE:
	case V31_RGB_G_CFG: {
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {
			rc = -EFAULT;
			goto proc_general_done;
		}
		msm_io_memcpy(vfe31_ctrl->vfebase + V31_RGB_G_OFF,
				cmdp, 4);
		cmdp += 1;
		vfe31_write_gamma_cfg(RGBLUT_RAM_CH0_BANK0 , cmdp);
		vfe31_write_gamma_cfg(RGBLUT_RAM_CH1_BANK0 , cmdp);
		vfe31_write_gamma_cfg(RGBLUT_RAM_CH2_BANK0 , cmdp);
		cmdp -= 1;
		}
		break;
	case V31_STOP:
		vfe_stop();
		break;

	default: {
		if (cmd->length != vfe31_cmd[cmd->id].length)
			return -EINVAL;

		cmdp = kmalloc(vfe31_cmd[cmd->id].length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto proc_general_done;
		}

		CHECKED_COPY_FROM_USER(cmdp);

		msm_io_memcpy(vfe31_ctrl->vfebase + vfe31_cmd[cmd->id].offset,
			cmdp, (vfe31_cmd[cmd->id].length));
	}
	break;

	}

proc_general_done:
	kfree(cmdp);

	return rc;
}

static int vfe31_config(struct msm_vfe_cfg_cmd *cmd, void *data)
{
	struct msm_vfe31_cmd vfecmd;

	long rc = 0;

	if (cmd->cmd_type != CMD_FRAME_BUF_RELEASE &&
		cmd->cmd_type != CMD_STATS_BUF_RELEASE &&
		cmd->cmd_type != CMD_STATS_AF_BUF_RELEASE) {

		if (copy_from_user(&vfecmd,
				(void __user *)(cmd->value),
				sizeof(vfecmd))) {
			pr_err("%s %d: copy_from_user failed\n", __func__,
				__LINE__);
			return -EFAULT;
		}
	}

	CDBG("%s: cmdType = %d\n", __func__, cmd->cmd_type);

	switch (cmd->cmd_type) {
	case CMD_GENERAL:
		rc = vfe31_proc_general(&vfecmd);
		break;

	case CMD_STATS_ENABLE:
	case CMD_STATS_AXI_CFG:
		break;

	case CMD_FRAME_BUF_RELEASE: {
		struct msm_frame *b;
		unsigned long p;
		struct vfe31_free_buf *fbuf = NULL;

		if (!data)
			return -EFAULT;

		b = (struct msm_frame *)(cmd->value);
		p = *(unsigned long *)data;

		CDBG("CMD_FRAME_BUF_RELEASE b->path = %d\n", b->path);

		if ((b->path & VFE31_OUTPUT_MODE_P) &&
			(vfe31_ctrl->outpath.output_mode &
			VFE31_OUTPUT_MODE_P) &&
			(!vfe31_ctrl->outpath.out0.free_buf.available)) {
			CDBG("CMD_FRAME_BUF_RELEASE got free buffer\n");
			fbuf = &vfe31_ctrl->outpath.out0.free_buf;
		} else if ((b->path & VFE31_OUTPUT_MODE_S) &&
			(vfe31_ctrl->outpath.output_mode &
			VFE31_OUTPUT_MODE_S)) {
			fbuf = &vfe31_ctrl->outpath.out1.free_buf;
		} else if ((b->path & VFE31_OUTPUT_MODE_V) &&
			(vfe31_ctrl->outpath.output_mode &
			VFE31_OUTPUT_MODE_V)) {
			fbuf = &vfe31_ctrl->outpath.out2.free_buf;
		} else
			return -EFAULT;

		fbuf->paddr = p;
		fbuf->y_off = b->y_off;
		fbuf->cbcr_off = b->cbcr_off;
		fbuf->available = 1;
	}
		break;

	case CMD_SNAP_BUF_RELEASE: {
	}
		break;

	case CMD_AXI_CFG_OUT2:
	case CMD_RAW_PICT_AXI_CFG: {
		struct axidata *axid;
		uint32_t *axio = NULL;

		axid = data;
		if (!axid)
			return -EFAULT;

		axio =
			kmalloc(vfe31_cmd[V31_AXI_OUT_CFG].length,
				GFP_ATOMIC);
		if (!axio)
			return -ENOMEM;

		if (copy_from_user(axio, (void __user *)(vfecmd.value),
				vfe31_cmd[V31_AXI_OUT_CFG].length)) {
			kfree(axio);
			return -EFAULT;
		}

		vfe31_config_axi(OUTPUT_2, axid, axio);
		kfree(axio);

	}
		break;
	default:
		break;
	}

	CDBG("%s done: rc = %d\n", __func__, (int) rc);
	return rc;
}

static inline void vfe31_read_irq_status(struct vfe31_irq_status *out)
{
	uint32_t *temp;

	memset(out, 0, sizeof(struct vfe31_irq_status));

	temp = (uint32_t *)(vfe31_ctrl->vfebase + VFE_IRQ_STATUS_0);
	out->vfeIrqStatus0 = msm_io_r(temp);

	temp = (uint32_t *)(vfe31_ctrl->vfebase + VFE_IRQ_STATUS_1);
	out->vfeIrqStatus1 = msm_io_r(temp);

	temp = (uint32_t *)(vfe31_ctrl->vfebase + VFE_CAMIF_STATUS);
	out->camifStatus = msm_io_r(temp);
	CDBG("camifStatus  = 0x%x\n", out->camifStatus);

	CDBG("camifPadStatus  = 0x%x\n", msm_camio_read_camif_status());
}

static void vfe31_send_msg_no_payload(enum VFE31_MESSAGE_ID id)
{
	struct vfe_message *msg;

	msg = kzalloc(sizeof(msg), GFP_ATOMIC);
	if (!msg)
		return;

	CDBG("vfe31_send_msg_no_payload\n");
	msg->_d = id;
	vfe31_proc_ops(id, msg, 0);
}

static void vfe31_process_reg_update_irq(void)
{
	CDBG("vfe31_process_reg_update_irq: ackPendingFlag is %d\n",
	vfe31_ctrl->start_ack_pending);
	if (vfe31_ctrl->start_ack_pending == TRUE) {
		vfe31_send_msg_no_payload(MSG_ID_START_ACK);
		vfe31_ctrl->start_ack_pending = FALSE;
	} else
		vfe31_send_msg_no_payload(MSG_ID_UPDATE_ACK);
}

static void vfe31_set_default_reg_values(void)
{
	msm_io_w(0x800080, vfe31_ctrl->vfebase + VFE_DEMUX_GAIN_0);
	msm_io_w(0x800080, vfe31_ctrl->vfebase + VFE_DEMUX_GAIN_1);
	msm_io_w(0xFFFFF, vfe31_ctrl->vfebase + VFE_CGC_OVERRIDE);

	/* default frame drop period and pattern */
	msm_io_w(0x1f, vfe31_ctrl->vfebase + VFE_FRAMEDROP_ENC_Y_CFG);
	msm_io_w(0x1f, vfe31_ctrl->vfebase + VFE_FRAMEDROP_ENC_CBCR_CFG);
	msm_io_w(0xFFFFFFFF, vfe31_ctrl->vfebase + VFE_FRAMEDROP_ENC_Y_PATTERN);
	msm_io_w(0xFFFFFFFF,
		vfe31_ctrl->vfebase + VFE_FRAMEDROP_ENC_CBCR_PATTERN);
	msm_io_w(0x1f, vfe31_ctrl->vfebase + VFE_FRAMEDROP_VIEW_Y);
	msm_io_w(0x1f, vfe31_ctrl->vfebase + VFE_FRAMEDROP_VIEW_CBCR);
	msm_io_w(0xFFFFFFFF,
		vfe31_ctrl->vfebase + VFE_FRAMEDROP_VIEW_Y_PATTERN);
	msm_io_w(0xFFFFFFFF,
		vfe31_ctrl->vfebase + VFE_FRAMEDROP_VIEW_CBCR_PATTERN);
	msm_io_w(0, vfe31_ctrl->vfebase + VFE_CLAMP_MIN);
	msm_io_w(0xFFFFFF, vfe31_ctrl->vfebase + VFE_CLAMP_MAX);
}

static void vfe31_process_reset_irq(void)
{
	if (vfe31_ctrl->stop_ack_pending) {

		vfe31_ctrl->stop_ack_pending = FALSE;
		vfe31_ctrl->vstate = VFE_STATE_IDLE;
		vfe31_send_msg_no_payload(MSG_ID_STOP_ACK);
	} else {
		/* this is from reset command. */
		vfe31_set_default_reg_values();

		/* reload all write masters. (frame & line)*/
		msm_io_w(0x7FFF, vfe31_ctrl->vfebase + VFE_BUS_CMD);
		vfe31_ctrl->vstate = VFE_STATE_IDLE;
		vfe31_send_msg_no_payload(MSG_ID_RESET_ACK);
	}
}

static void vfe31_process_camif_sof_irq(void)
{
	vfe31_ctrl->vfeFrameId++;

	CDBG("camif_sof_irq, frameId = %d\n",
		vfe31_ctrl->vfeFrameId);
}

#define VFE31_AXI_OFFSET 0x0050
#define vfe31_get_ch_ping_addr(chn) \
	(msm_io_r(vfe31_ctrl->vfebase + 0x0050 + 0x18 * (chn)))
#define vfe31_get_ch_pong_addr(chn) \
	(msm_io_r(vfe31_ctrl->vfebase + 0x0050 + 0x18 * (chn) + 4))
#define vfe31_get_ch_addr(ping_pong, chn) \
	(((ping_pong) & (1 << (chn))) == 0 ? \
	vfe31_get_ch_pong_addr(chn) : vfe31_get_ch_ping_addr(chn))

#define vfe31_put_ch_ping_addr(chn, addr) \
	(msm_io_w((addr), vfe31_ctrl->vfebase + 0x0050 + 0x18 * (chn)))
#define vfe31_put_ch_pong_addr(chn, addr) \
	(msm_io_w((addr), vfe31_ctrl->vfebase + 0x0050 + 0x18 * (chn) + 4))
#define vfe31_put_ch_addr(ping_pong, chn, addr) \
	(((ping_pong) & (1 << (chn))) == 0 ?   \
	vfe31_put_ch_pong_addr((chn), (addr)) : \
	vfe31_put_ch_ping_addr((chn), (addr)))

static void vfe31_process_output_path_irq_0(uint32_t *irqstatus)
{
	uint32_t ping_pong;
	uint32_t pyaddr, pcbcraddr;

	if ((vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_P) ||
		(vfe31_ctrl->outpath.output_mode & VFE31_OUTPUT_MODE_S)) {


		if (vfe31_ctrl->outpath.out0.free_buf.available) {

			ping_pong = msm_io_r(vfe31_ctrl->vfebase +
				VFE_BUS_PING_PONG_STATUS);

			/* Y channel */
			pyaddr = vfe31_get_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out0.ch0);
			/* Chroma channel */
			pcbcraddr = vfe31_get_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out0.ch1);

			CDBG("path_irq_0, pyaddr = 0x%x, pcbcraddr = 0x%x\n",
				pyaddr, pcbcraddr);

			/* Y channel */
			vfe31_put_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out0.ch0,
				vfe31_ctrl->outpath.out0.free_buf.paddr +
				vfe31_ctrl->outpath.out0.free_buf.y_off);
			/* Chroma channel */
			vfe31_put_ch_addr(ping_pong,
				vfe31_ctrl->outpath.out0.ch1,
				vfe31_ctrl->outpath.out0.free_buf.paddr +
				vfe31_ctrl->outpath.out0.free_buf.cbcr_off);

			vfe31_ctrl->outpath.out0.free_buf.available = 0;
			vfe_send_outmsg(VFE31_OUTPUT_MODE_P, pyaddr, pcbcraddr);
		} else {
			CDBG("path_irq_0 - no free buffer!\n");
		}
	}
}

static void vfe31_do_tasklet(unsigned long data)
{
	unsigned long flags;

	struct vfe31_isr_queue_cmd *qcmd = NULL;

	CDBG("=== vfe31_do_tasklet start === \n");

	spin_lock_irqsave(&vfe31_ctrl->tasklet_lock, flags);
	qcmd = list_first_entry(&vfe31_ctrl->tasklet_q,
		struct vfe31_isr_queue_cmd, list);

	if (!qcmd) {
		spin_unlock_irqrestore(&vfe31_ctrl->tasklet_lock, flags);
		return;
	}

	list_del(&qcmd->list);
	spin_unlock_irqrestore(&vfe31_ctrl->tasklet_lock, flags);

	/* interrupt to be processed,  *qcmd has the payload.  */
	if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_REG_UPDATE_MASK) {
		CDBG("irq	regUpdateIrq\n");
		vfe31_process_reg_update_irq();
	}

	if (qcmd->vfeInterruptStatus1 & VFE_IMASK_WHILE_STOPPING_1) {
		CDBG("irq	resetAckIrq\n");
		vfe31_process_reset_irq();
	}

	spin_lock_irqsave(&vfe31_ctrl->state_lock, flags);
	if (vfe31_ctrl->vstate == VFE_STATE_ACTIVE) {
		/* irqs below are only valid when in active state. */
		spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);
		/* next, check output path related interrupts. */
		if (qcmd->vfeInterruptStatus0 &
			VFE_IRQ_STATUS0_IMAGE_COMPOSIT_DONE0_MASK) {
			CDBG("Image composite done 0 irq occured.\n");
			vfe31_process_output_path_irq_0(
				&qcmd->vfeInterruptStatus0);
		}

	} else {
		spin_unlock_irqrestore(&vfe31_ctrl->state_lock, flags);
		goto tasklet_done;
	}

	if (qcmd->vfeInterruptStatus0 & VFE_IRQ_STATUS0_CAMIF_SOF_MASK) {
		CDBG("irq	camifSofIrq\n");
		vfe31_process_camif_sof_irq();
	}

tasklet_done:
	kfree(qcmd);
	CDBG("=== vfe31_do_tasklet end === \n");
}

DECLARE_TASKLET(vfe31_tasklet, vfe31_do_tasklet, 0);

static irqreturn_t vfe31_parse_irq(int irq_num, void *data)
{
	unsigned long flags;
	struct vfe31_irq_status irq;
	struct vfe31_isr_queue_cmd *qcmd;

	CDBG("vfe_parse_irq\n");

	vfe31_read_irq_status(&irq);

	if ((irq.vfeIrqStatus0 == 0) && (irq.vfeIrqStatus1 == 0)) {
		CDBG("vfe_parse_irq: vfeIrqStatus0 & 1 are both 0!\n");
		return IRQ_HANDLED;
	}

	qcmd = kzalloc(sizeof(struct vfe31_isr_queue_cmd),
		GFP_ATOMIC);
	if (!qcmd) {
		CDBG("vfe_parse_irq: qcmd malloc failed!\n");
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&vfe31_ctrl->ack_lock, flags);

	if (vfe31_ctrl->stop_ack_pending) {
		irq.vfeIrqStatus0 &= VFE_IMASK_WHILE_STOPPING_0;
		irq.vfeIrqStatus1 &= VFE_IMASK_WHILE_STOPPING_1;
	}

	CDBG("vfe_parse_irq: Irq_status0 = 0x%x, Irq_status1 = 0x%x.\n",
		irq.vfeIrqStatus0, irq.vfeIrqStatus1);

	spin_unlock_irqrestore(&vfe31_ctrl->ack_lock, flags);

	qcmd->vfeInterruptStatus0 = irq.vfeIrqStatus0;
	qcmd->vfeInterruptStatus1 = irq.vfeIrqStatus1;

	spin_lock_irqsave(&vfe31_ctrl->tasklet_lock, flags);
	list_add_tail(&qcmd->list, &vfe31_ctrl->tasklet_q);
	spin_unlock_irqrestore(&vfe31_ctrl->tasklet_lock, flags);
	tasklet_schedule(&vfe31_tasklet);

	/* clear the pending interrupt of the same kind.*/
	msm_io_w(irq.vfeIrqStatus0, vfe31_ctrl->vfebase + VFE_IRQ_CLEAR_0);
	msm_io_w(irq.vfeIrqStatus1, vfe31_ctrl->vfebase + VFE_IRQ_CLEAR_1);
	msm_io_w(1, vfe31_ctrl->vfebase + VFE_IRQ_CMD);

	return IRQ_HANDLED;
}

static int vfe31_resource_init(struct msm_vfe_callback *presp,
	struct platform_device *pdev, void *sdata)
{
	struct resource	*vfemem, *vfeirq, *vfeio;
	int rc;
	struct msm_camera_sensor_info *s_info;
	s_info = pdev->dev.platform_data;

	pdev->resource = s_info->resource;
	pdev->num_resources = s_info->num_resources;

	vfemem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!vfemem) {
		pr_err("%s: no mem resource?\n", __func__);
		return -ENODEV;
	}

	vfeirq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!vfeirq) {
		pr_err("%s: no irq resource?\n", __func__);
		return -ENODEV;
	}

	vfeio = request_mem_region(vfemem->start,
		resource_size(vfemem), pdev->name);
	if (!vfeio) {
		pr_err("%s: VFE region already claimed\n", __func__);
		return -EBUSY;
	}

	vfe31_ctrl = kzalloc(sizeof(struct vfe31_ctrl_type), GFP_KERNEL);
	if (!vfe31_ctrl) {
		rc = -ENOMEM;
		goto cmd_init_failed1;
	}

	vfe31_ctrl->vfeirq = vfeirq->start;

	vfe31_ctrl->vfebase =
		ioremap(vfemem->start, (vfemem->end - vfemem->start) + 1);
	if (!vfe31_ctrl->vfebase) {
		rc = -ENOMEM;
		pr_err("%s: vfe ioremap failed\n", __func__);
		goto cmd_init_failed2;
	}

	rc = request_irq(vfe31_ctrl->vfeirq, vfe31_parse_irq,
		IRQF_TRIGGER_RISING, "vfe", 0);
	if (rc < 0)
		goto cmd_init_failed2;

	if (presp && presp->vfe_resp)
		vfe31_ctrl->resp = presp;
	else {
		rc = -EINVAL;
		goto cmd_init_failed3;
	}

	vfe31_ctrl->extdata =
		kmalloc(sizeof(struct vfe31_frame_extra), GFP_KERNEL);
	if (!vfe31_ctrl->extdata) {
		rc = -ENOMEM;
		goto cmd_init_failed3;
	}

	vfe31_ctrl->extlen = sizeof(struct vfe31_frame_extra);

	spin_lock_init(&vfe31_ctrl->ack_lock);
	spin_lock_init(&vfe31_ctrl->state_lock);
	spin_lock_init(&vfe31_ctrl->io_lock);

	spin_lock_init(&vfe31_ctrl->tasklet_lock);
	INIT_LIST_HEAD(&vfe31_ctrl->tasklet_q);

	vfe31_ctrl->syncdata = sdata;
	vfe31_ctrl->vfemem = vfemem;
	vfe31_ctrl->vfeio  = vfeio;
	return 0;

cmd_init_failed3:
	free_irq(vfe31_ctrl->vfeirq, 0);
	iounmap(vfe31_ctrl->vfebase);
cmd_init_failed2:
	kfree(vfe31_ctrl);
cmd_init_failed1:
	release_mem_region(vfemem->start, (vfemem->end - vfemem->start) + 1);
	return rc;
}

static int vfe31_init(struct msm_vfe_callback *presp,
	struct platform_device *dev)
{
	int rc = 0;
	rc = vfe31_resource_init(presp, dev, vfe_syncdata);
	if (rc < 0)
		return rc;

	/* Bring up all the required GPIOs and Clocks */
	rc = msm_camio_enable(dev);
	return rc;
}

void msm_camvfe_fn_init(struct msm_camvfe_fn *fptr, void *data)
{
	fptr->vfe_init    = vfe31_init;
	fptr->vfe_enable  = vfe31_enable;
	fptr->vfe_config  = vfe31_config;
	fptr->vfe_disable = vfe31_disable;
	fptr->vfe_release = vfe31_release;
	vfe_syncdata = data;
}

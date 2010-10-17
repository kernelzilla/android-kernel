/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */

#include "video_core_type.h"
#include "vcd_power_sm.h"
#include "vcd_core.h"
#include "vcd.h"

u32 vcd_power_event(
	struct vcd_dev_ctxt_type *p_dev_ctxt,
     struct vcd_clnt_ctxt_type_t *p_cctxt, u32 event)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_MED("Device power state = %d", p_dev_ctxt->e_pwr_clk_state);
	VCD_MSG_MED("event = 0x%x", event);
	switch (event) {

	case VCD_EVT_PWR_DEV_INIT_BEGIN:
	case VCD_EVT_PWR_DEV_INIT_END:
	case VCD_EVT_PWR_DEV_INIT_FAIL:
	case VCD_EVT_PWR_DEV_TERM_BEGIN:
	case VCD_EVT_PWR_DEV_TERM_END:
	case VCD_EVT_PWR_DEV_TERM_FAIL:
	case VCD_EVT_PWR_DEV_SLEEP_BEGIN:
	case VCD_EVT_PWR_DEV_SLEEP_END:
	case VCD_EVT_PWR_DEV_SET_PERFLVL:
	case VCD_EVT_PWR_DEV_HWTIMEOUT:
		{
			rc = vcd_device_power_event(p_dev_ctxt, event,
				p_cctxt);
			break;
		}

	case VCD_EVT_PWR_CLNT_CMD_BEGIN:
	case VCD_EVT_PWR_CLNT_CMD_END:
	case VCD_EVT_PWR_CLNT_CMD_FAIL:
	case VCD_EVT_PWR_CLNT_PAUSE:
	case VCD_EVT_PWR_CLNT_RESUME:
	case VCD_EVT_PWR_CLNT_FIRST_FRAME:
	case VCD_EVT_PWR_CLNT_LAST_FRAME:
	case VCD_EVT_PWR_CLNT_ERRFATAL:
		{
			rc = vcd_client_power_event(p_dev_ctxt, p_cctxt, event);
			break;
		}

	}

	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("vcd_power_event: event 0x%x failed", event);


	return rc;

}

u32 vcd_device_power_event(struct vcd_dev_ctxt_type *p_dev_ctxt, u32 event,
	struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_ERR_FAIL;
	u32 n_set_perf_lvl;

	switch (event) {

	case VCD_EVT_PWR_DEV_INIT_BEGIN:
	{
		if (p_dev_ctxt->e_pwr_clk_state ==
			VCD_PWRCLK_STATE_OFF) {
			if (res_trk_get_max_perf_level(&p_dev_ctxt->
				n_max_perf_lvl)) {
				if (res_trk_power_up()) {
					p_dev_ctxt->e_pwr_clk_state =
					VCD_PWRCLK_STATE_ON_NOTCLOCKED;
					p_dev_ctxt->n_curr_perf_lvl = 0;
					p_dev_ctxt->n_reqd_perf_lvl = 0;
					p_dev_ctxt->n_active_clnts = 0;
					p_dev_ctxt->
						b_set_perf_lvl_pending = FALSE;
					rc = vcd_enable_clock(p_dev_ctxt,
						p_cctxt);
					if (VCD_FAILED(rc)) {
						(void)res_trk_power_down();
						p_dev_ctxt->e_pwr_clk_state =
							VCD_PWRCLK_STATE_OFF;
					}
				}
			}
		}

		break;
	}

	case VCD_EVT_PWR_DEV_INIT_END:
	case VCD_EVT_PWR_DEV_TERM_FAIL:
	case VCD_EVT_PWR_DEV_SLEEP_BEGIN:
	case VCD_EVT_PWR_DEV_HWTIMEOUT:
		{
			rc = vcd_gate_clock(p_dev_ctxt);

			break;
		}

	case VCD_EVT_PWR_DEV_INIT_FAIL:
	case VCD_EVT_PWR_DEV_TERM_END:
		{
			if (p_dev_ctxt->e_pwr_clk_state !=
				VCD_PWRCLK_STATE_OFF) {
				(void)vcd_disable_clock(p_dev_ctxt);
				(void)res_trk_power_down();

				p_dev_ctxt->e_pwr_clk_state =
				    VCD_PWRCLK_STATE_OFF;
				p_dev_ctxt->n_curr_perf_lvl = 0;
				p_dev_ctxt->n_reqd_perf_lvl = 0;
				p_dev_ctxt->n_active_clnts = 0;
				p_dev_ctxt->b_set_perf_lvl_pending = FALSE;
				rc = VCD_S_SUCCESS;
			}

			break;
		}

	case VCD_EVT_PWR_DEV_TERM_BEGIN:
	case VCD_EVT_PWR_DEV_SLEEP_END:
		{
			rc = vcd_un_gate_clock(p_dev_ctxt);

			break;
		}

	case VCD_EVT_PWR_DEV_SET_PERFLVL:
		{
			n_set_perf_lvl =
			    p_dev_ctxt->n_reqd_perf_lvl >
			    0 ? p_dev_ctxt->
			    n_reqd_perf_lvl : VCD_MIN_PERF_LEVEL;

			rc = vcd_set_perf_level(p_dev_ctxt, n_set_perf_lvl,
				p_cctxt);

			break;
		}
	}
	return rc;
}

u32 vcd_client_power_event(
	struct vcd_dev_ctxt_type *p_dev_ctxt,
    struct vcd_clnt_ctxt_type_t *p_cctxt, u32 event)
{
	u32 rc = VCD_ERR_FAIL;

	switch (event) {

	case VCD_EVT_PWR_CLNT_CMD_BEGIN:
		{
			rc = vcd_un_gate_clock(p_dev_ctxt);
			break;
		}

	case VCD_EVT_PWR_CLNT_CMD_END:
		{
			rc = vcd_gate_clock(p_dev_ctxt);
			break;
		}

	case VCD_EVT_PWR_CLNT_CMD_FAIL:
		{
			if (!vcd_core_is_busy(p_dev_ctxt))
				rc = vcd_gate_clock(p_dev_ctxt);

			break;
		}

	case VCD_EVT_PWR_CLNT_PAUSE:
	case VCD_EVT_PWR_CLNT_LAST_FRAME:
	case VCD_EVT_PWR_CLNT_ERRFATAL:
		{
			if (p_cctxt) {
				rc = VCD_S_SUCCESS;
				if (p_cctxt->status.b_req_perf_lvl) {
					p_dev_ctxt->n_reqd_perf_lvl -=
						p_cctxt->n_reqd_perf_lvl;
					p_cctxt->status.b_req_perf_lvl = FALSE;

					rc = vcd_set_perf_level(p_dev_ctxt,
						p_dev_ctxt->n_reqd_perf_lvl,
						p_cctxt);
				}
			}

			break;
		}

	case VCD_EVT_PWR_CLNT_RESUME:
	case VCD_EVT_PWR_CLNT_FIRST_FRAME:
		{
			if (p_cctxt) {
				rc = VCD_S_SUCCESS;
				if (!p_cctxt->status.b_req_perf_lvl) {
					p_dev_ctxt->n_reqd_perf_lvl +=
						p_cctxt->n_reqd_perf_lvl;
					p_cctxt->status.b_req_perf_lvl = TRUE;

					rc = vcd_set_perf_level(p_dev_ctxt,
						p_dev_ctxt->n_reqd_perf_lvl,
						p_cctxt);
				}
			}
			break;
		}
	}

	return rc;
}

u32 vcd_enable_clock(struct vcd_dev_ctxt_type *p_dev_ctxt,
	struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	u32 n_set_perf_lvl;

	if (p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_OFF) {
		VCD_MSG_ERROR("vcd_enable_clock(): Already in state "
			"VCD_PWRCLK_STATE_OFF\n");
		vcd_assert();
		rc = VCD_ERR_FAIL;
	} else if (p_dev_ctxt->e_pwr_clk_state ==
		VCD_PWRCLK_STATE_ON_NOTCLOCKED) {

		n_set_perf_lvl =
				p_dev_ctxt->n_reqd_perf_lvl >
				0 ? p_dev_ctxt->
				n_reqd_perf_lvl : VCD_MIN_PERF_LEVEL;

		rc = vcd_set_perf_level(p_dev_ctxt, n_set_perf_lvl,
			p_cctxt);

		if (!VCD_FAILED(rc)) {
			if (res_trk_enable_clocks()) {
				p_dev_ctxt->e_pwr_clk_state =
					VCD_PWRCLK_STATE_ON_CLOCKED;
			}
		} else {
			rc = VCD_ERR_FAIL;
		}

	}

	if (!VCD_FAILED(rc))
		p_dev_ctxt->n_active_clnts++;

	return rc;
}

u32 vcd_disable_clock(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	u32 rc = VCD_S_SUCCESS;

	if (p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_OFF) {
		VCD_MSG_ERROR("vcd_disable_clock(): Already in state "
			"VCD_PWRCLK_STATE_OFF\n");
		vcd_assert();
		rc = VCD_ERR_FAIL;
	} else if (p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_ON_CLOCKED ||
		p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_ON_CLOCKGATED) {
		p_dev_ctxt->n_active_clnts--;

		if (!p_dev_ctxt->n_active_clnts) {
			if (!res_trk_disable_clocks())
				rc = VCD_ERR_FAIL;

			p_dev_ctxt->e_pwr_clk_state =
			    VCD_PWRCLK_STATE_ON_NOTCLOCKED;
			p_dev_ctxt->n_curr_perf_lvl = 0;
		}
	}

	return rc;
}

u32 vcd_set_perf_level(struct vcd_dev_ctxt_type *p_dev_ctxt,
	u32 n_perf_lvl, struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;

	if (!vcd_core_is_busy(p_dev_ctxt)) {
		if (res_trk_set_perf_level(n_perf_lvl,
			&p_dev_ctxt->n_curr_perf_lvl, p_cctxt)) {
			p_dev_ctxt->b_set_perf_lvl_pending = FALSE;
		} else {
			rc = VCD_ERR_FAIL;
			p_dev_ctxt->b_set_perf_lvl_pending = TRUE;
		}

	} else {
		p_dev_ctxt->b_set_perf_lvl_pending = TRUE;
	}

	return rc;
}

u32 vcd_update_clnt_perf_lvl(
	struct vcd_clnt_ctxt_type_t *p_cctxt,
     struct vcd_property_frame_rate_type *p_fps, u32 n_frm_p_units)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_dev_ctxt_type *p_dev_ctxt = p_cctxt->p_dev_ctxt;
	u32 n_new_perf_lvl;

	n_new_perf_lvl =
	    n_frm_p_units * p_fps->n_fps_numerator / p_fps->n_fps_denominator;

	if (p_cctxt->status.b_req_perf_lvl) {
		p_dev_ctxt->n_reqd_perf_lvl =
		    p_dev_ctxt->n_reqd_perf_lvl - p_cctxt->n_reqd_perf_lvl +
		    n_new_perf_lvl;

		rc = vcd_set_perf_level(p_cctxt->p_dev_ctxt,
			p_dev_ctxt->n_reqd_perf_lvl, p_cctxt);
	}

	p_cctxt->n_reqd_perf_lvl = n_new_perf_lvl;

	return rc;
}

u32 vcd_gate_clock(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	u32 rc = VCD_S_SUCCESS;

	if (p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_OFF ||
		p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_ON_NOTCLOCKED) {
		VCD_MSG_ERROR("%s(): Clk is Off or Not Clked yet \n", __func__);
		vcd_assert();
		return VCD_ERR_FAIL;
	}

	if (p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_ON_CLOCKGATED)
		return rc;

	if (res_trk_disable_clocks())
		p_dev_ctxt->e_pwr_clk_state = VCD_PWRCLK_STATE_ON_CLOCKGATED;
	else
		rc = VCD_ERR_FAIL;

	return rc;
}

u32 vcd_un_gate_clock(struct vcd_dev_ctxt_type *p_dev_ctxt)
{
	u32 rc = VCD_S_SUCCESS;

	if (p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_OFF ||
		p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_ON_NOTCLOCKED) {
		VCD_MSG_ERROR("%s(): Clk is Off or Not Clked yet \n", __func__);
		vcd_assert();
		return VCD_ERR_FAIL;
	}

	if (p_dev_ctxt->e_pwr_clk_state == VCD_PWRCLK_STATE_ON_CLOCKED)
		return rc;

	if (res_trk_enable_clocks())
		p_dev_ctxt->e_pwr_clk_state = VCD_PWRCLK_STATE_ON_CLOCKED;
	else
		rc = VCD_ERR_FAIL;

	return rc;
}


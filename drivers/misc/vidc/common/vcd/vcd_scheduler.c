/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "vidc_type.h"
#include "vcd.h"

static const u32 TKNBKT_SIZE_FACTOR = 5;
static const u32 MAX_U32 = 0xffffffff;

inline u32 get_current_time(void)
{
	struct timeval tv;
	u32 actual_time;
	do_gettimeofday(&tv);
	actual_time = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
	return actual_time;
}

u32 vcd_sched_create(u32 n_perf_lvl, struct vcd_sched_ctx_type **p_sched_ctx)
{
	u32 rc = VCD_S_SUCCESS;
	if (!p_sched_ctx || !n_perf_lvl) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else {
		*p_sched_ctx = (struct vcd_sched_ctx_type *)
			vcd_malloc(sizeof(struct vcd_sched_ctx_type));
		if (*p_sched_ctx) {
			memset(*p_sched_ctx, 0,
				sizeof(struct vcd_sched_ctx_type));
			INIT_LIST_HEAD(&(*p_sched_ctx)->clnt_list);
			(*p_sched_ctx)->n_perf_lvl = n_perf_lvl;
		} else {
			VCD_MSG_ERROR("%s(): vcd_malloc failed!", __func__);
			rc = VCD_ERR_ALLOC_FAIL;
		}
	}
	return rc;
}

void vcd_sched_destroy(struct vcd_sched_ctx_type *p_sched_ctx)
{
	if (p_sched_ctx) {
		if (!list_empty(&p_sched_ctx->clnt_list))
			VCD_MSG_HIGH("%s(): Destroying NO empty scheduler!",
						__func__);
		memset(p_sched_ctx, 0, sizeof(struct vcd_sched_ctx_type));
		vcd_free(p_sched_ctx);
	}
}

u32 vcd_sched_add_client(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_property_hdr_type prop_hdr;
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt;
	struct vcd_sched_ctx_type *p_sched = NULL;
	u32 rc = VCD_S_SUCCESS;
	if (!p_cctxt) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (p_cctxt->sched_clnt_hdl)
		VCD_MSG_HIGH(
			"%s(): Scheduler client already exists!", __func__);
	else {
		p_sched = p_cctxt->p_dev_ctxt->sched_hdl;
		p_sched_cctxt = (struct vcd_sched_clnt_ctx_type *)
			vcd_malloc(sizeof(struct vcd_sched_clnt_ctx_type));
		if (p_sched_cctxt) {
			p_cctxt->sched_clnt_hdl = p_sched_cctxt;
			memset(p_sched_cctxt, 0,
				sizeof(struct vcd_sched_clnt_ctx_type));
			prop_hdr.prop_id = DDL_I_FRAME_PROC_UNITS;
			prop_hdr.n_size = sizeof(p_cctxt->n_frm_p_units);
			rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr,
						  &p_cctxt->n_frm_p_units);
			VCD_FAILED_RETURN(rc,
				"Failed: Get DDL_I_FRAME_PROC_UNITS");

			if (p_cctxt->b_decoding) {
				p_cctxt->frm_rate.n_fps_numerator =
					VCD_DEC_INITIAL_FRAME_RATE;
				p_cctxt->frm_rate.n_fps_denominator = 1;
			} else {
				prop_hdr.prop_id = VCD_I_FRAME_RATE;
				prop_hdr.n_size = sizeof(p_cctxt->frm_rate);
				rc = ddl_get_property(p_cctxt->ddl_handle,
						&prop_hdr, &p_cctxt->frm_rate);
				VCD_FAILED_RETURN(rc,
					"Failed: Get VCD_I_FRAME_RATE");
			}

			p_cctxt->n_reqd_perf_lvl = p_cctxt->n_frm_p_units *
				p_cctxt->frm_rate.n_fps_numerator /
				p_cctxt->frm_rate.n_fps_denominator;

			(void)vcd_sched_update_config(p_cctxt);
			p_sched_cctxt->n_bkt_tkns = 0;
			p_sched_cctxt->n_o_tkns = 0;
			p_sched_cctxt->n_bkt_lst_sup_time = get_current_time();
			p_sched_cctxt->b_clnt_active = TRUE;
			p_sched_cctxt->p_clnt_data = p_cctxt;
			INIT_LIST_HEAD(&p_sched_cctxt->clnt_frm_list);

			list_add_tail(&p_sched_cctxt->list,
				&p_sched->clnt_list);

			p_sched->n_total_clnt_bw += p_sched_cctxt->n_p_tkn_rate;
			if (p_sched->n_total_clnt_bw > p_sched->n_perf_lvl)
				VCD_MSG_HIGH(
					"Not free bandwidth to support client");
		}
	}
	return rc;
}

u32 vcd_sched_remove_client(struct vcd_sched_ctx_type *p_sched,
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	if (!p_sched || !p_sched_cctxt) {
		VCD_MSG_ERROR("%s(): Invalid handle ptr", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (!list_empty(&p_sched_cctxt->clnt_frm_list)) {
		VCD_MSG_ERROR(
			"%s(): Cannot remove client, queue no empty", __func__);
		rc = VCD_ERR_ILLEGAL_OP;
	} else {
		list_del(&p_sched_cctxt->list);
		p_sched->n_total_clnt_bw -= p_sched_cctxt->n_p_tkn_rate;
		memset(p_sched_cctxt, 0,
			sizeof(struct vcd_sched_clnt_ctx_type));
		vcd_free(p_sched_cctxt);
	}
	return rc;
}

u32 vcd_sched_update_config(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt;
	if (p_cctxt && p_cctxt->sched_clnt_hdl) {
		p_sched_cctxt = p_cctxt->sched_clnt_hdl;
		if (p_sched_cctxt->b_clnt_active)
			vcd_sched_update_tkn_bucket(p_cctxt->sched_clnt_hdl,
				get_current_time());
		p_sched_cctxt->n_p_tkn_per_frm = p_cctxt->n_frm_p_units;
		p_sched_cctxt->n_p_tkn_rate = p_cctxt->n_reqd_perf_lvl;
		p_sched_cctxt->n_bkt_size = TKNBKT_SIZE_FACTOR *
			p_sched_cctxt->n_p_tkn_per_frm;
		if (p_sched_cctxt->n_bkt_tkns > p_sched_cctxt->n_bkt_size)
			p_sched_cctxt->n_bkt_tkns = p_sched_cctxt->n_bkt_size;
	} else {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	}
	return rc;
}

u32 vcd_sched_queue_buffer(
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt,
	struct vcd_buffer_entry_type *p_buffer, u32 b_tail)
{
	u32 rc = VCD_S_SUCCESS;
	if (!p_sched_cctxt || !p_buffer) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (b_tail)
		list_add_tail(&p_buffer->sched_list,
			&p_sched_cctxt->clnt_frm_list);
	else
		list_add(&p_buffer->sched_list, &p_sched_cctxt->clnt_frm_list);
	return rc;
}

u32 vcd_sched_dequeue_buffer(
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt,
	struct vcd_buffer_entry_type **pp_buffer)
{
	u32 rc = VCD_S_SCHED_QEMPTY;
	if (!p_sched_cctxt || !pp_buffer) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else {
		*pp_buffer = NULL;
		if (!list_empty(&p_sched_cctxt->clnt_frm_list)) {
			*pp_buffer = list_first_entry(
					&p_sched_cctxt->clnt_frm_list,
					struct vcd_buffer_entry_type,
					sched_list);
			list_del(&(*pp_buffer)->sched_list);
			rc = VCD_S_SUCCESS;
		}
	}
	return rc;
}

u32 vcd_sched_mark_client_eof(struct vcd_sched_clnt_ctx_type *p_sched_cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry_type *p_buffer = NULL;
	if (!p_sched_cctxt) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (!list_empty(&p_sched_cctxt->clnt_frm_list)) {
		p_buffer = list_entry(p_sched_cctxt->clnt_frm_list.prev,
			struct vcd_buffer_entry_type, sched_list);
		p_buffer->frame.n_flags |= VCD_FRAME_FLAG_EOS;
	} else
		rc = VCD_S_SCHED_QEMPTY;
	return rc;
}

u32 vcd_sched_suspend_resume_clnt(
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt, u32 b_state)
{
	u32 rc = VCD_S_SUCCESS;
	if (!p_sched_cctxt) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (b_state != p_sched_cctxt->b_clnt_active) {
		p_sched_cctxt->b_clnt_active = b_state;
		if (b_state)
			p_sched_cctxt->n_bkt_lst_sup_time =
				get_current_time();
		else
			vcd_sched_update_tkn_bucket(p_sched_cctxt,
				get_current_time());
	}
	return rc;
}

void vcd_sched_update_tkn_bucket(
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt, u32 n_actual_time)
{
	u32 n_delta, n_num_tkns;
	n_delta = n_actual_time;
	if (p_sched_cctxt->n_bkt_lst_sup_time < n_actual_time)
		n_delta -= p_sched_cctxt->n_bkt_lst_sup_time;
	else
		n_delta += MAX_U32 - p_sched_cctxt->n_bkt_lst_sup_time;

	n_num_tkns = n_delta * p_sched_cctxt->n_p_tkn_rate / 1000;
	if (n_num_tkns) {
		p_sched_cctxt->n_bkt_tkns += n_num_tkns;
		if (p_sched_cctxt->n_bkt_tkns > p_sched_cctxt->n_bkt_size)
			p_sched_cctxt->n_bkt_tkns = p_sched_cctxt->n_bkt_size;
		n_delta = n_delta * p_sched_cctxt->n_p_tkn_rate % 1000;
		if (n_actual_time < n_delta)
			p_sched_cctxt->n_bkt_lst_sup_time =
				MAX_U32 - n_delta + n_actual_time;
		else
			p_sched_cctxt->n_bkt_lst_sup_time =
				n_actual_time - n_delta;
	}
}

u32 vcd_sched_get_client_frame(struct vcd_sched_ctx_type *p_sched,
	struct vcd_clnt_ctxt_type_t **pp_cctxt,
	struct vcd_buffer_entry_type **pp_buffer)
{
	u32 rc = VCD_S_SCHED_QEMPTY, n_time;
	struct vcd_sched_clnt_ctx_type *p_sched_clnt,
		*p_sched_cand, *p_conf_clnt;
	if (!p_sched || !pp_cctxt || !pp_buffer) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (!list_empty(&p_sched->clnt_list)) {
		*pp_cctxt = NULL;
		*pp_buffer = NULL;
		n_time = get_current_time();
		p_conf_clnt = p_sched_cand = NULL;
		list_for_each_entry(p_sched_clnt, &p_sched->clnt_list, list) {
			if (p_sched_clnt->b_clnt_active
				&& !list_empty(&p_sched_clnt->clnt_frm_list)
				&& p_sched_clnt->n_o_tkns) {
				vcd_sched_update_tkn_bucket(
					p_sched_clnt, n_time);
				if (!p_conf_clnt &&
					p_sched_clnt->n_bkt_tkns >=
						p_sched_clnt->n_p_tkn_per_frm)
					p_conf_clnt = p_sched_clnt;
				else if (!p_sched_cand)
					p_sched_cand = p_sched_clnt;
			}
		}
		if (p_conf_clnt)
			p_sched_clnt = p_conf_clnt;
		else
			p_sched_clnt = p_sched_cand;
		if (p_sched_clnt) {
			rc = vcd_sched_dequeue_buffer(p_sched_clnt, pp_buffer);
			if (rc == VCD_S_SUCCESS) {
				p_sched_clnt->n_bkt_tkns -=
					p_sched_clnt->n_p_tkn_per_frm;
				p_sched_clnt->n_o_tkns--;
				*pp_cctxt = p_sched_clnt->p_clnt_data;
				list_move_tail(&p_sched_clnt->list,
					&p_sched->clnt_list);
			}
		}
	}
	return rc;
}

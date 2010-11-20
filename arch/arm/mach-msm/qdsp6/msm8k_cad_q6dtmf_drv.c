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

#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_cad_q6dtmf_drv.h>
#include <mach/qdsp6/msm8k_cad_q6dtmf_drvi.h>
#include <mach/qdsp6/msm8k_cad_q6dtmf_session.h>

static struct q6dtmf_driver q6_dtmf_data;

static s32 cad_dtmf_open(s32 session_id,
			struct cad_open_struct_type *open_param)
{
	struct q6dtmf_session		*session = NULL;
	s32				rc = CAD_RES_SUCCESS;

	if (!open_param || (session_id >= CAD_MAX_SESSION) ||
		(session_id <= 0)) {

		pr_err("Invalid prameters to dtmf open, session: %d\n",
				session_id);
		rc = CAD_RES_FAILURE;
		goto done;
	}

	if (open_param->format != CAD_FORMAT_DTMF) {
		pr_err("Format is not DTMF\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

	if (q6_dtmf_data.free_session_list == NULL) {
		pr_err("No more free DTMF sessions\n");
		rc = CAD_RES_FAILURE;
		goto done;
	}

	session = q6_dtmf_data.free_session_list;
	if (cad_dtmf_session_open(session, session_id,
			open_param) != CAD_RES_SUCCESS) {
		cad_dtmf_session_close(session);
		rc = CAD_RES_FAILURE;
		goto done;
	}

	q6_dtmf_data.free_session_list =
		q6_dtmf_data.free_session_list->next;
	session->next = q6_dtmf_data.used_session_list;
	q6_dtmf_data.used_session_list = session;
done:
	return rc;
}

static s32 cad_dtmf_close(s32 session_id)
{
	struct q6dtmf_session	*prev_session = NULL;
	struct q6dtmf_session	*session = q6_dtmf_data.used_session_list;
	s32			rc = CAD_RES_SUCCESS;

	if ((session_id >= CAD_MAX_SESSION) || (session_id <= 0)) {
		pr_err("Invalid session ID for dtmf close, session_id:  %d\n",
			session_id);
		rc = CAD_RES_FAILURE;
		goto done;
	}

	while (session) {
		if (session->session_id == session_id) {
			if (prev_session == NULL) {
				/* first node */
				q6_dtmf_data.used_session_list = session->next;
				break;
			}
			/* not the first node */
			prev_session->next = session->next;
			break;
		}
		/* no match */
		prev_session = session;
		session = session->next;
	}

	if (session) {
		cad_dtmf_session_close(session);
		session->next = q6_dtmf_data.free_session_list;
		q6_dtmf_data.free_session_list = session;
	}
done:
	return rc;
}



static s32 cad_dtmf_ioctl(s32 session_id, u32 cmd_code, void *cmd_buf,
				u32 cmd_len)
{
	struct q6dtmf_session	*session = q6_dtmf_data.used_session_list;
	s32			rc = CAD_RES_SUCCESS;

	while (session) {
		if (session->session_id == session_id) {
			if (cad_dtmf_session_ioctl(session, cmd_code, cmd_buf,
				cmd_len) != CAD_RES_SUCCESS) {

				pr_err("Call to dtmf session ioctl failed\n");
				rc = CAD_RES_FAILURE;
				goto done;
			}
			break;
		}
		session = session->next;
	}
done:
	return rc;
}
s32 cad_dtmf_dinit(void)
{
	struct q6dtmf_session	*session;

	while (q6_dtmf_data.free_session_list) {
		session = q6_dtmf_data.free_session_list->next;
		cad_dtmf_session_dinit(q6_dtmf_data.free_session_list);
		kfree(q6_dtmf_data.free_session_list);
		q6_dtmf_data.free_session_list = session;
	}

	while (q6_dtmf_data.used_session_list) {
		session = q6_dtmf_data.used_session_list->next;
		cad_dtmf_session_dinit(q6_dtmf_data.used_session_list);
		kfree(q6_dtmf_data.used_session_list);
		q6_dtmf_data.used_session_list = session;
	}
	memset(&q6_dtmf_data, 0, sizeof(q6_dtmf_data));
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_init(struct cad_func_tbl_type **func_tbl)
{
	u32				i;
	s32				rc = CAD_RES_SUCCESS;
	struct q6dtmf_session		*session = NULL;

	static struct cad_func_tbl_type		vtable = {
		cad_dtmf_open,
		cad_dtmf_close,
		NULL,
		NULL,
		cad_dtmf_ioctl
	};

	memset(&q6_dtmf_data, 0, sizeof(q6_dtmf_data));

	for (i = 0; i < CAD_DTMF_SESSION_MAX; i++) {
		session = kmalloc(sizeof(*session), GFP_KERNEL);
		if (session == NULL) {
			pr_err("Could not allocate memory for the session\n");
			cad_dtmf_dinit();
			rc = CAD_RES_FAILURE;
			goto done;

		}

		memset(session, 0, sizeof(*session));
		if (cad_dtmf_session_init(session) != CAD_RES_SUCCESS) {
			pr_err("Call to dtmf session init failed\n");
			cad_dtmf_dinit();
			rc = CAD_RES_FAILURE;
			goto done;
		}

		session->next = q6_dtmf_data.free_session_list;
		q6_dtmf_data.free_session_list = session;
	}

	*func_tbl = &vtable;
done:
	return rc;
}


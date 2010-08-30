/* arch/arm/mach-msm/qdsp5/audrec.c
 *
 * common code to deal with the AUDREC dsp task (audio recording)
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Based on the audpp layer in arch/arm/mach-msm/qdsp5/audpp.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2008 HTC Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <mach/msm_adsp.h>

#include <mach/qdsp5/qdsp5audreccmdi.h>
#include <mach/qdsp5/qdsp5audrecmsg.h>

#include "audmgr.h"
/* for queue ids - should be relative to module number*/
#include "adsp.h"

#ifdef DEBUG
#define dprintk(format, arg...) \
printk(KERN_DEBUG format, ## arg)
#else
#define dprintk(format, arg...) do {} while (0)
#endif

static DEFINE_MUTEX(audrec_lock);

#define MAX_ENC_COUNT 8 /* Max encoder supported */

#define ENC_SESSION_FREE 0
#define ENC_SESSION_ACTIVE 1

struct enc_session {
	unsigned enc_type;  /* Param to identify type of encoder */
	unsigned audrec_obj_idx;  /* Param to identify REC_OBJ or Session ID */
	audrec_event_func event_func; /* Event Call back
					routine for the encoder */
	void *private;	/* private data element passed as
				part of Event Call back  routine */
	unsigned state; /* Current state of the encoder session ,
				free, active*/
};

struct audrec_state {
	struct msm_adsp_module *audrec_mod;
	struct enc_session enc_session[MAX_ENC_COUNT];
	struct mutex *lock;
	unsigned enc_count;
};

struct audrec_state the_audrec_state = {
	.lock = &audrec_lock,
};

int audrectask_send_cmdqueue(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audrec_state.audrec_mod,
				QDSP_uPAudRecCmdQueue, cmd, len);
}
EXPORT_SYMBOL(audrectask_send_cmdqueue);

int audrectask_send_bitstreamqueue(void *cmd, unsigned len)
{
	return msm_adsp_write(the_audrec_state.audrec_mod,
				QDSP_uPAudRecBitStreamQueue, cmd, len);
}
EXPORT_SYMBOL(audrectask_send_bitstreamqueue);

static void audrectask_dsp_event(void *data, unsigned id, size_t len,
			    void (*getevent)(void *ptr, size_t len))
{
	struct audrec_state *audrec = data;
	int cnt;
	uint16_t msg[5]; /* Max size of message */
	getevent(msg, len);

	switch (id) {
	case AUDREC_MSG_CMD_CFG_DONE_MSG: {
		dprintk(" %s : CMD CFG DONE %x\n", __func__, msg[1]);
		if (msg[0] & AUDREC_MSG_CFG_DONE_ENC_ENA) {
			for (cnt = 0; cnt < MAX_ENC_COUNT ; cnt++) {
				if (audrec->enc_session[cnt].enc_type ==
					(msg[0] & AUDREC_CMD_ENC_TYPE_MASK)) {
					audrec->enc_session[cnt].audrec_obj_idx
					 = msg[1];
					audrec->enc_session[cnt].event_func(
					audrec->enc_session[cnt].private, id,
					msg);
					break;
				}
			}
		} else {
			for (cnt = 0; cnt < MAX_ENC_COUNT ; cnt++) {
				if (audrec->enc_session[cnt].enc_type ==
					(msg[0] & AUDREC_CMD_ENC_TYPE_MASK)) {
					audrec->enc_session[cnt].event_func(
					audrec->enc_session[cnt].private, id,
					msg);
					audrec->enc_session[cnt].audrec_obj_idx
					= 0xFFFFFFFF;
					audrec->enc_session[cnt].state
					= ENC_SESSION_FREE;
					audrec->enc_session[cnt].enc_type
					= 0xFFFFFFFF;
					audrec->enc_session[cnt].event_func
					= NULL;
					audrec->enc_session[cnt].private
					= NULL;
					break;
				}
			}
		}
		break;
	}
	case AUDREC_MSG_CMD_AREC_MEM_CFG_DONE_MSG: {
		dprintk(" %s : CMD AREC MEM CFG DONE %x\n", __func__, msg[0]);
		for (cnt = 0; cnt < MAX_ENC_COUNT ; cnt++) {
			if (audrec->enc_session[cnt].audrec_obj_idx ==
				msg[0]) {
				audrec->enc_session[cnt].event_func(
				audrec->enc_session[cnt].private, id, msg);
				break;
			}
		}
		break;
	}
	case AUDREC_MSG_CMD_AREC_PARAM_CFG_DONE_MSG: {
		dprintk(" %s : CMD AREC PARAM CFG DONE %x\n", __func__, msg[0]);
		for (cnt = 0; cnt < MAX_ENC_COUNT ; cnt++) {
			if (audrec->enc_session[cnt].audrec_obj_idx ==
				msg[0]) {
				audrec->enc_session[cnt].event_func(
				audrec->enc_session[cnt].private, id, msg);
				break;
			}
		}
		break;
	}
	case AUDREC_MSG_PACKET_READY_MSG: {
		dprintk(" %s : PCK READY %x\n", __func__, msg[0]);
		for (cnt = 0; cnt < MAX_ENC_COUNT ; cnt++) {
			if (audrec->enc_session[cnt].audrec_obj_idx ==
				msg[0]) {
				audrec->enc_session[cnt].event_func(
				audrec->enc_session[cnt].private, id, msg);
				break;
			}
		}
		break;
	}
	case AUDREC_MSG_FATAL_ERR_MSG: {
		pr_err(" %s : ERROR %x\n", __func__, msg[0]);
		if (msg[1] & AUDREC_MSG_FATAL_ERR_TYPE_0) {
			for (cnt = 0; cnt < MAX_ENC_COUNT ; cnt++) {
				if (audrec->enc_session[cnt].audrec_obj_idx ==
					msg[0]) {
					audrec->enc_session[cnt].event_func(
					audrec->enc_session[cnt].private, id,
					msg);
				break;
				}
			}
		} else if (msg[1] & AUDREC_MSG_FATAL_ERR_TYPE_1) {
			cnt = audrec->enc_count-1;
			if (audrec->enc_session[cnt].event_func)
				audrec->enc_session[cnt].event_func(
				audrec->enc_session[cnt].private, id,
				msg);
		}
		break;
	}
	default:
		pr_err("audrectask_dsp_event: unknown event %d\n", id);
	}
}

static struct msm_adsp_ops adsp_ops = {
	.event = audrectask_dsp_event,
};

int audrectask_enable(unsigned enc_type, audrec_event_func func, void *private)
{
	struct audrec_state *audrec = &the_audrec_state;
	int cnt, rc = 0;

	mutex_lock(audrec->lock);

	if (audrec->enc_count++ == 0) {
		dprintk(" %s : enable\n", __func__);
		for (cnt = 0; cnt < MAX_ENC_COUNT ; cnt++) {
			if (audrec->enc_session[cnt].state ==
				ENC_SESSION_FREE) {
				audrec->enc_session[cnt].state =
				ENC_SESSION_ACTIVE;
				audrec->enc_session[cnt].enc_type = enc_type;
				audrec->enc_session[cnt].event_func = func;
				audrec->enc_session[cnt].private = private;
				break;
			}
		}
		rc = msm_adsp_get("AUDRECTASK", &audrec->audrec_mod, &adsp_ops,
					audrec);
		if (rc < 0) {
			pr_err("audrec: cannot open AUDRECTASK\n");
			audrec->enc_count = 0;
			audrec->enc_session[cnt].state = ENC_SESSION_FREE;
			audrec->enc_session[cnt].enc_type = 0xFFFFFFFF;
			audrec->enc_session[cnt].event_func = NULL;
			audrec->enc_session[cnt].private = NULL;
			goto out;
		}
		msm_adsp_enable(audrec->audrec_mod);
	} else {
		for (cnt = 0; cnt < MAX_ENC_COUNT ; cnt++) {
			if (audrec->enc_session[cnt].state ==
				ENC_SESSION_FREE) {
				audrec->enc_session[cnt].state =
				ENC_SESSION_ACTIVE;
				audrec->enc_session[cnt].enc_type = enc_type;
				audrec->enc_session[cnt].event_func = func;
				audrec->enc_session[cnt].private = private;
				break;
			}
		}
	}
	if (cnt == MAX_ENC_COUNT)
		rc = -EBUSY;
	else
		rc = 0;

out:
	mutex_unlock(audrec->lock);
	return rc;
}
EXPORT_SYMBOL(audrectask_enable);

void audrectask_disable(unsigned enc_type, void *private)
{
	struct audrec_state *audrec = &the_audrec_state;

	mutex_lock(audrec->lock);

	if (--audrec->enc_count == 0) {
		dprintk(" %s : disable\n", __func__);
		msm_adsp_disable(audrec->audrec_mod);
		msm_adsp_put(audrec->audrec_mod);
		audrec->audrec_mod = NULL;
	}

	mutex_unlock(audrec->lock);
}
EXPORT_SYMBOL(audrectask_disable);


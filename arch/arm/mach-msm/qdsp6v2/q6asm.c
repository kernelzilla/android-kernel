/*
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <mach/debug_mm.h>
#include <mach/peripheral-loader.h>
#include "q6asm.h"
#include "apr_audio.h"

#define SESSION_MAX 8

static DEFINE_MUTEX(session_lock);
static DEFINE_MUTEX(lock);

static struct audio_client *session[SESSION_MAX];

static int session_alloc(struct audio_client *ac)
{
	int n;

	mutex_lock(&session_lock);
	for (n = 1; n < SESSION_MAX + 1; n++) {
		if (!session[n]) {
			session[n] = ac;
			mutex_unlock(&session_lock);
			return n;
		}
	}
	mutex_unlock(&session_lock);
	return -ENOMEM;
}

static void session_free(int n, struct audio_client *ac)
{
	mutex_lock(&session_lock);
	if (session[n] == ac)
		session[n] = 0;
	mutex_unlock(&session_lock);
}

static void audio_client_free(struct audio_client *ac)
{
	session_free(ac->session, ac);

	/* TODO: Need to update for AIO with multiple buffers */
	if (ac->buf[0].data)
		dma_free_coherent(NULL, ac->buf[0].size,
					ac->buf[0].data, ac->buf[0].phys);
	if (ac->buf[1].data)
		dma_free_coherent(NULL, ac->buf[1].size,
					ac->buf[1].data, ac->buf[1].phys);
	kfree(ac);
}

static struct audio_client *audio_client_alloc(unsigned int bufsz)
{
	struct audio_client *ac;
	int n;
	ac = kzalloc(sizeof(*ac), GFP_KERNEL);
	if (!ac)
		return 0;

	n = session_alloc(ac);
	if (n <= 0)
		goto fail_session;

	ac->session = n;

	if (bufsz > 0) {
		ac->buf[0].data = dma_alloc_coherent(NULL, bufsz,
						&ac->buf[0].phys, GFP_KERNEL);
		if (!ac->buf[0].data)
			goto fail;
		ac->buf[1].data = dma_alloc_coherent(NULL, bufsz,
						&ac->buf[1].phys, GFP_KERNEL);
		if (!ac->buf[1].data)
			goto fail;

		ac->buf[0].size = bufsz;
		ac->buf[1].size = bufsz;
	}
	init_waitqueue_head(&ac->wait);
	init_waitqueue_head(&ac->cmd_wait);
	return ac;

fail:
	session_free(n, ac);
fail_session:
	audio_client_free(ac);
	return 0;
}

/* May need to move out, as different stream have different implementation */
static int32_t q6asm_callback(struct apr_client_data *data, void *priv)
{

	struct audio_client *ac = (struct audio_client *) priv;
	int n;
	uint32_t *payload;
	payload = data->payload;

	pr_info("%s: data->opcode = 0x%x\n", __func__, data->opcode);
	if (ac == NULL) {
		pr_err("%s: ac NULL\n", __func__);
		return 0;
	}

	if (data->payload_size) {
		if (data->opcode != APR_BASIC_RSP_RESULT) {
			switch (data->opcode) {
			case ASM_DATA_EVENT_READ_DONE: {
				uint32_t buffer = payload[1];
				uint32_t nActualSize = payload[2];
				n = data->token;
				ac = session[ac->session];
				ac->buf[n].actual_size = nActualSize;

				if ((void *)ac->buf[n].phys !=
						(void *)buffer) {
					pr_err("Buf expected[%p] \
						rxed[%p]\n",\
						(void *)ac->buf[n].phys,\
						(void *)buffer);
				}
				ac->buf[n].used = 0;
				wake_up(&ac->wait);
				break;
			}
			case ASM_DATA_EVENT_WRITE_DONE: {
				n = data->token;
				ac = session[ac->session];
				ac->buf[ac->dsp_buf].used = 0;
				ac->dsp_buf ^= 1;
				wake_up(&ac->wait);
				break;
			}
			default:
				pr_err("%s: unknown cmd=0x%x\n",
					__func__, data->opcode);
			break;
			}
		} else {
			switch (payload[0]) {
			case ASM_STREAM_CMD_OPEN_READ:
			case ASM_STREAM_CMD_OPEN_WRITE:
			case ASM_STREAM_CMD_CLOSE:
			case ASM_SESSION_CMD_RUN:
				n = data->token;
				ac = session[n];
				ac->state = 0;
				wake_up(&ac->cmd_wait);
				break;
			default:
				pr_err("%s: unknown cmd=0x%x\n",
					__func__, payload[0]);
				break;
			}
		}
	}
	return 0;
}

struct audio_client *q6asm_out_open(uint32_t bufsz,
			  uint32_t rate, uint32_t channels)
{

	struct asm_stream_cmd_open_write open;
	struct audio_client *ac;

	pr_info("%s\n", __func__);

	mutex_lock(&lock);


	ac = audio_client_alloc(bufsz);
	if (!ac) {
		mutex_unlock(&lock);
		return 0;
	}
	if (ac->ref_count == 0) {
		ac->apr = apr_register("ADSP", "ASM", q6asm_callback,
							0x101, ac);
		pr_info("%s: Register ASM\n", __func__);
		if (ac->apr == NULL) {
			pr_err("APR register failed\n");
			goto fail;
		}
	}
	ac->state = 1;

	open.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(20), APR_PKT_VER);
	open.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(open) - APR_HDR_SIZE);
	open.hdr.src_svc = ((struct apr_svc *)ac->apr)->id;
	open.hdr.src_domain = APR_DOMAIN_APPS;
	open.hdr.src_port = ac->session << 8 | 0x0001; /* Same as dest */
	open.hdr.dest_svc = APR_SVC_ASM;
	open.hdr.dest_domain = APR_DOMAIN_ADSP;
	open.hdr.dest_port = ac->session << 8 | 0x0001; /* Always new
						session for each open */
	open.hdr.token = ac->session;
	open.hdr.opcode = ASM_STREAM_CMD_OPEN_WRITE;

	open.uMode = ASM_STREAM_PRIORITY_NORMAL;
	open.sink_endpoint = ASM_END_POINT_DEVICE_MATRIX;
	open.stream_handle = 0;
	open.post_proc_top = DEFAULT_TOPOLOGY;
	open.format = LINEAR_PCM;
	open.cfg_size = sizeof(struct asm_pcm_cfg);
	open.write_cfg.pcm_cfg.ch_cfg = channels;
	open.write_cfg.pcm_cfg.bits_per_sample = 16;
	open.write_cfg.pcm_cfg.sample_rate = rate;
	open.write_cfg.pcm_cfg.is_signed = 1;
	open.write_cfg.pcm_cfg.interleaved = 1;

	if (apr_send_pkt(ac->apr, (uint32_t *) &open) < 0) {
		pr_err("Comamnd open failed\n");
		goto fail_cmd;
	}

	ac->state = 1;
	if (!wait_event_timeout(ac->cmd_wait, (ac->state == 0), 5*HZ)) {
		pr_err("[%s:%s] timeout. waited for OPEN_READ?\n",
						__MM_FILE__, __func__);
	}
	ac->ref_count++;
	mutex_unlock(&lock);
	return ac;
fail_cmd:
	apr_deregister(ac->apr);
fail:
	audio_client_free(ac);
	mutex_unlock(&lock);
	return 0;
}


int q6asm_out_run(struct audio_client *ac)
{

	struct asm_stream_cmd_run run;

	pr_info("%s\n", __func__);

	mutex_lock(&lock);
	run.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(20), APR_PKT_VER);
	run.hdr.pkt_size =  APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(run) - APR_HDR_SIZE);
	run.hdr.src_svc = ((struct apr_svc *)ac->apr)->id;
	run.hdr.src_domain = APR_DOMAIN_APPS;
	run.hdr.src_port = ac->session << 8 | 0x0001;
	run.hdr.dest_svc = APR_SVC_ASM;
	run.hdr.dest_domain = APR_DOMAIN_ADSP;
	run.hdr.dest_port = ac->session << 8 | 0x0001;
	run.hdr.token = ac->session;
	run.hdr.opcode = ASM_SESSION_CMD_RUN;
	run.flags = 0;
	run.msw_ts = 0;
	run.lsw_ts = 0;

	if (apr_send_pkt(ac->apr, (uint32_t *) &run) < 0) {
		pr_err("Command run failed\n");
		mutex_unlock(&lock);
		return -ENODEV;
	}
	mutex_unlock(&lock);
	return 0;
}

struct audio_client *q6asm_in_open(uint32_t bufsz, uint32_t rate,
					uint32_t channels)
{
	struct asm_stream_cmd_open_read open;
	struct audio_client *ac;

	pr_info("%s\n", __func__);

	mutex_lock(&lock);
	ac = audio_client_alloc(bufsz);
	if (!ac) {
		mutex_unlock(&lock);
		return 0;
	}

	if (ac->ref_count == 0) {
		ac->apr = apr_register("ADSP", "ASM", q6asm_callback,
							0x101, ac);
		pr_info("%s: Register ASM\n", __func__);
		if (ac->apr == NULL) {
			pr_err("APR register failed\n");
			goto fail;
		}
	}

	open.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(20), APR_PKT_VER);
	open.hdr.pkt_size =  APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(open) - APR_HDR_SIZE);
	open.hdr.src_svc = ((struct apr_svc *)ac->apr)->id;
	open.hdr.src_domain = APR_DOMAIN_APPS;
	open.hdr.src_port = ac->session << 8 | 0x0001;
	open.hdr.dest_svc = APR_SVC_ASM;
	open.hdr.dest_domain = APR_DOMAIN_ADSP;
	open.hdr.dest_port = ac->session << 8 | 0x0001;
	open.hdr.token = ac->session;
	open.hdr.opcode = ASM_STREAM_CMD_OPEN_READ;
	open.uMode = ASM_STREAM_PRIORITY_NORMAL;
	open.src_endpoint = ASM_END_POINT_DEVICE_MATRIX;
	open.pre_proc_top = DEFAULT_TOPOLOGY;
	open.format = LINEAR_PCM;
	open.cfg_size = sizeof(struct asm_pcm_cfg);
	open.read_cfg.pcm_cfg.ch_cfg = channels;
	open.read_cfg.pcm_cfg.bits_per_sample = 16;
	open.read_cfg.pcm_cfg.sample_rate = rate;
	open.read_cfg.pcm_cfg.is_signed = 1;
	open.read_cfg.pcm_cfg.interleaved = 1;

	if (apr_send_pkt(ac->apr, (uint32_t *) &open) < 0) {
		pr_err("Comamnd open failed\n");
		goto fail_cmd;
	}

	ac->state = 1;
	if (!wait_event_timeout(ac->cmd_wait, (ac->state == 0), 5*HZ))
		pr_err("[%s:%s] timeout. waited for OPEN_READ?\n",
				__MM_FILE__, __func__);

	ac->ref_count++;
	mutex_unlock(&lock);
	return ac;

fail_cmd:
	apr_deregister(ac->apr);
fail:
	audio_client_free(ac);
	mutex_unlock(&lock);
	return 0;
}

int q6asm_in_run(struct audio_client *ac)
{
	struct asm_stream_cmd_run run;
	int ret = 0;

	mutex_lock(&lock);
	run.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(20), APR_PKT_VER);
	run.hdr.pkt_size =  APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(run) - APR_HDR_SIZE);
	run.hdr.src_svc = ((struct apr_svc *)ac->apr)->id;
	run.hdr.src_domain = APR_DOMAIN_APPS;
	run.hdr.src_port = ac->session << 8 | 0x0001;
	run.hdr.dest_svc = APR_SVC_ASM;
	run.hdr.dest_domain = APR_DOMAIN_ADSP;
	run.hdr.dest_port = ac->session << 8 | 0x0001;
	run.hdr.token = ac->session;
	run.hdr.opcode = ASM_SESSION_CMD_RUN;
	run.flags = 0;

	if (apr_send_pkt(ac->apr, (uint32_t *) &run) < 0) {
		pr_err("Command run failed\n");
		ret = -EINVAL;
		goto fail_cmd;
	}
	ac->state = 1;
	if (!wait_event_timeout(ac->cmd_wait, (ac->state == 0), 5*HZ)) {
			pr_err("[%s:%s] timeout. waited for CMD_RUN\n",
						__MM_FILE__, __func__);
	}
fail_cmd:
	mutex_unlock(&lock);
	return ret;
}


int q6asm_close(struct audio_client *ac)
{
	struct apr_hdr close;
	int ret = 0;

	mutex_lock(&lock);

	if (ac->ref_count == 0) {
		pr_err("ASM is already closed\n");
		ret = -EINVAL;
		goto fail;
	}
	close.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(20), APR_PKT_VER);
	close.pkt_size =  APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(close) - APR_HDR_SIZE);
	close.src_svc = ((struct apr_svc *)ac->apr)->id;
	close.src_domain = APR_DOMAIN_APPS;
	close.src_port = ac->session << 8 | 0x0001;
	close.dest_svc = APR_SVC_ASM;
	close.dest_domain = APR_DOMAIN_ADSP;
	close.dest_port = ac->session << 8 | 0x0001;
	close.token = ac->session;
	close.opcode = ASM_STREAM_CMD_CLOSE;

	if (apr_send_pkt(ac->apr, (uint32_t *) &close) < 0)
		pr_err("Commmand close failed\n");
	ac->state = 1;
	if (!wait_event_timeout(ac->cmd_wait, (ac->state == 0), 5*HZ))
		pr_err("[%s:%s] timeout. waited for OPEN_WRITE?\n",
				__MM_FILE__, __func__);
	ac->ref_count--;
	if (ac->ref_count == 0) {
		pr_info("%s: DeRegister ASM\n", __func__);
		apr_deregister(ac->apr);
	}
	audio_client_free(ac);
fail:
	mutex_unlock(&lock);
	return ret;
}

int q6asm_read(struct audio_client *ac, struct audio_buffer *ab)
{
	struct asm_stream_cmd_read read;
	int ret;

	mutex_lock(&lock);
	read.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(20), APR_PKT_VER);
	read.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(read) - APR_HDR_SIZE);
	read.hdr.src_svc = ((struct apr_svc *)ac->apr)->id;
	read.hdr.src_domain = APR_DOMAIN_APPS;
	read.hdr.src_port = ac->session << 8 | 0x0001;
	read.hdr.dest_svc = APR_SVC_ASM;
	read.hdr.dest_domain = APR_DOMAIN_ADSP;
	read.hdr.dest_port = ac->session << 8 | 0x0001;
	read.hdr.opcode = ASM_DATA_CMD_READ;
	read.buf_add = ab->phys;
	read.buf_size = ab->size;

	/* TODO: use as ping pong buffer, will need to change for AIO */
	read.hdr.token = ac->dsp_buf;
	read.uid = ac->dsp_buf;
	memset(ab->data, 0, sizeof(ab->size));

	if (apr_send_pkt(ac->apr, (uint32_t *) &read) < 0) {
		pr_err("Command read failed\n");
		ret = -EINVAL;
		goto fail_cmd;
	}
fail_cmd:
	mutex_unlock(&lock);
	return ret;
}

int q6asm_write(struct audio_client *ac, struct audio_buffer *ab)
{
	struct asm_stream_cmd_write write;
	int ret;

	pr_info("%s\n", __func__);

	mutex_lock(&lock);
	write.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(20), APR_PKT_VER);
	write.hdr.pkt_size =  APR_PKT_SIZE(APR_HDR_SIZE,
						sizeof(write) - APR_HDR_SIZE);
	write.hdr.src_svc = ((struct apr_svc *)ac->apr)->id;
	write.hdr.src_domain = APR_DOMAIN_APPS;
	write.hdr.src_port = ac->session << 8 | 0x0001;
	write.hdr.dest_svc = APR_SVC_ASM;
	write.hdr.dest_domain = APR_DOMAIN_ADSP;
	write.hdr.dest_port = ac->session << 8 | 0x0001;
	write.hdr.opcode = ASM_DATA_CMD_WRITE;

	write.buf_add = ab->phys;
	write.avail_bytes = ab->actual_size;
	write.msw_ts = 0;
	write.lsw_ts = 0;
	write.uflags = 0;
	write.hdr.token = ac->cpu_buf;
	write.uid = ac->cpu_buf;

	/* TODO: use as ping pong buffer, will need to change for AIO */

	if (apr_send_pkt(ac->apr, (uint32_t *) &write) < 0) {
		pr_err("Command read failed\n");
		ret = -EINVAL;
		goto fail_cmd;
	}
fail_cmd:
	mutex_unlock(&lock);
	return ret;
}

static int __init q6asm_init(void)
{
	pr_info("%s\n", __func__);
	memset(session, 0, sizeof(session));
	return 0;
}

device_initcall(q6asm_init);

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
#include <mach/msm_qdsp6_audio.h>

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

struct audio_client *audio_client_alloc(unsigned int bufsz)
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
	return ac;

fail:
	session_free(n, ac);
fail_session:
	audio_client_free(ac);
	return 0;
}

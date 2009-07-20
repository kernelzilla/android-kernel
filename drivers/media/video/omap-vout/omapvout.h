/*
 * drivers/media/video/omap/omapvout.h
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * Based on drivers/media/video/omap24xx/omap24xxvout.c&h
 *
 * Copyright (C) 2005-2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __OMAPVOUT_H__
#define __OMAPVOUT_H__

#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <mach/display.h>
#include <mach/vrfb.h>
#include <media/v4l2-dev.h>

/*#define DEBUG*/
#ifdef DEBUG
#define DBG(format, ...) \
	printk(KERN_DEBUG "OMAPVOUT: " format, ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

#define OMAPVOUT_VID_MAX_FRAMES	(10)

struct omapvout_frm_q_entry {
	u32 flags; /* same a V4L2_buffer.flags */
	u32 size;
	u32 phy_addr;
	u32 virt_addr;
	u32 offset; /* For mmap'ing purposes only */
	u32 seq_num;
};

/* The device structure */

struct omapvout_device {
	struct video_device vdev;
	struct mutex  mtx; /* Lock for all device accesses */
	struct video_device *vfd;

	int opened;
	int id;

	int disp_width;
	int disp_height;

	int max_video_width;
	int max_video_height;
	int max_video_bytespp;

	/* Buffer pool */
	struct omapvout_bp *bp;

	/* DSS data */
	struct omapvout_dss *dss;

	/* V4L2 data */
	bool streaming;
	int rotation;
	u32 colorkey;
	int colorkey_en;
	struct v4l2_pix_format pix;
	struct v4l2_window win;
	struct v4l2_rect crop;

	/* Frame Q */
	int fq_cnt; /* number of frames in the queue */
	int fq_min_size; /* smallest frame in the queue */
	int fq_dq_idx; /* the index to begin dQing from */
	int fq_cur_idx; /* the index of the current displayed frame */
	u32 fq_next_seq; /* a sequence number to maintain frame Q'ing order*/
	struct omapvout_frm_q_entry fq[OMAPVOUT_VID_MAX_FRAMES];
	wait_queue_head_t fq_wait;

	/* Don't allow new buffers when some are still mapped */
	int mmap_cnt;
};

#define vdev_to_omapvout(d) container_of(d, struct omapvout_device, vdev)

#endif /* __OMAPVOUT_H__ */

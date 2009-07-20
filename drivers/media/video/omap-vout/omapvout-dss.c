/*
 * drivers/media/video/omap/omapvout-dss.c
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

#include <linux/mm.h>
#include <mach/display.h>
#include <mach/dma.h>
#include <mach/vrfb.h>

#include "omapvout.h"
#include "omapvout-dss.h"
#include "omapvout-mem.h"

#define DMA_CHAN_ALLOTED	1
#define DMA_CHAN_NOT_ALLOTED	0

#define VRFB_TX_TIMEOUT		1000

#define INVALID_SEQ_NUM		0

/*=== Local Functions ==================================================*/

static int omapvout_dss_format_bytespp(u32 pixelformat)
{
	int bpp = 2;

	switch (pixelformat) {
	case V4L2_PIX_FMT_RGB32:
		bpp = 4;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		bpp = 2;
		break;
	}

	return bpp;
}

static enum omap_color_mode omapvout_dss_color_mode(u32 pixelformat)
{
	enum omap_color_mode mode = OMAP_DSS_COLOR_RGB16;

	switch (pixelformat) {
	case V4L2_PIX_FMT_RGB32:
		mode = OMAP_DSS_COLOR_RGB24U;
		break;
	case V4L2_PIX_FMT_RGB565:
		mode = OMAP_DSS_COLOR_RGB16;
		break;
	case V4L2_PIX_FMT_YUYV:
		mode = OMAP_DSS_COLOR_YUV2;
		break;
	case V4L2_PIX_FMT_UYVY:
		mode = OMAP_DSS_COLOR_UYVY;
		break;
	}

	return mode;
}

static int omapvout_dss_calc_offset(struct omapvout_device *vout)
{
	struct omapvout_dss *dss;
	int rc = 0;
	u32 fmt;
	int bpp;
	int bpp_mult = 1;
	u16 ow, oh;
	int iw, ih;
	int cx, cy, cw, ch;

	/* It is assumed that the caller has locked the vout mutex */

	fmt = vout->pix.pixelformat;
	bpp = omapvout_dss_format_bytespp(fmt);
	if (fmt == V4L2_PIX_FMT_YUYV || fmt == V4L2_PIX_FMT_UYVY)
		bpp_mult = 2;

	iw = vout->pix.width;
	ih = vout->pix.height;
	cx = vout->crop.left;
	cy = vout->crop.top;
	cw = vout->crop.width;
	ch = vout->crop.height;

	ow = iw;
	oh = ih;
	omap_vrfb_adjust_size(&ow, &oh, bpp);
	ow = ow - iw;
	oh = oh - ih;

	dss = vout->dss;

	switch (vout->rotation)	{
	case 1: /* 90 degrees */
		dss->foffset = (cx * OMAP_VRFB_LINE_LEN * bpp * bpp_mult)
				+ ((oh + (ih - cy - ch)) * bpp * bpp_mult);
		break;
	case 2: /* 180 degrees */
		dss->foffset = ((oh + (ih - cy - ch)) * OMAP_VRFB_LINE_LEN
							* bpp * bpp_mult)
				+ ((ow + (iw - cx - cw)) * bpp * bpp_mult);
		break;
	case 3: /* 270 degrees */
		dss->foffset = ((ow + (iw - cx - cw)) * OMAP_VRFB_LINE_LEN
							* bpp * bpp_mult)
				+ (cy * bpp * bpp_mult);
		break;
	default:
	case 0: /* 0 degrees */
		dss->foffset = ((cy * iw) + (cx)) * bpp;
		break;
	}

	return rc;
}

static int omapvout_dss_config_colorkey(struct omapvout_device *vout, bool init)
{
	struct omap_overlay_manager *mgr;
	struct omap_overlay_manager_info m_info;

	mgr = vout->dss->overlay->manager;
	if (mgr == NULL)
		return -EINVAL;

	if (mgr->set_manager_info == NULL || mgr->get_manager_info == NULL)
		return -EINVAL;

	mgr->get_manager_info(mgr, &m_info);
	if (vout->colorkey_en) {
		m_info.alpha_enabled = false;
		if (init) {
			m_info.trans_key_type = OMAP_DSS_COLOR_KEY_GFX_DST;
			m_info.trans_key = vout->colorkey;
			m_info.trans_enabled = true;
		} else {
			m_info.trans_enabled = false;
		}

		mgr->set_manager_info(mgr, &m_info);
	} else {
		m_info.trans_enabled = false;
		m_info.alpha_enabled = false;
		mgr->set_manager_info(mgr, &m_info);
	}

	return 0;
}

/* This functions wakes up the application once the DMA transfer to
 * VRFB space is completed.
 */
static void omapvout_dss_vrfb_dma_cb(int lch, u16 ch_status, void *data)
{
	struct omapvout_dss_vrfb *vrfb;

	vrfb = (struct omapvout_dss_vrfb *) data;

	vrfb->dma_complete = true;
	wake_up_interruptible(&vrfb->wait);
}

static int omapvout_dss_acquire_vrfb(struct omapvout_device *vout)
{
	int rc = 0;
	int size;
	u16 w, h;
	struct omapvout_dss_vrfb *vrfb;

	/* It is assumed that the caller has locked the vout mutex */

	vrfb = &vout->dss->vrfb;
	vrfb->dma_id = OMAP_DMA_NO_DEVICE;
	vrfb->dma_ch = -1;
	vrfb->req_status = DMA_CHAN_NOT_ALLOTED;
	vrfb->next = 0;

	rc = omap_vrfb_request_ctx(&vrfb->ctx[0]);
	if (rc != 0) {
		DBG("VRFB context allocation 0 failed %d\n", rc);
		goto failed_ctx0;
	}

	rc = omap_vrfb_request_ctx(&vrfb->ctx[1]);
	if (rc != 0) {
		DBG("VRFB context allocation 1 failed %d\n", rc);
		goto failed_ctx1;
	}

	w = vout->max_video_width;
	h = vout->max_video_height;
	omap_vrfb_adjust_size(&w, &h, vout->max_video_bytespp);
	size = PAGE_ALIGN(w * h * vout->max_video_bytespp);
	vrfb->size = size;

	rc = omapvout_mem_alloc(size, &vrfb->phy_addr[0], &vrfb->virt_addr[0]);
	if (rc != 0) {
		DBG("VRFB buffer alloc 0 failed %d\n", rc);
		goto failed_mem0;
	}

	rc = omapvout_mem_alloc(size, &vrfb->phy_addr[1], &vrfb->virt_addr[1]);
	if (rc != 0) {
		DBG("VRFB buffer alloc 1 failed %d\n", rc);
		goto failed_mem1;
	}

	rc = omap_request_dma(vrfb->dma_id, "VRFB DMA",
				omapvout_dss_vrfb_dma_cb,
				(void *)vrfb,
				&vrfb->dma_ch);
	if (rc != 0) {
		printk(KERN_INFO "No VRFB DMA channel for %d\n", vout->id);
		goto failed_dma;
	}

	vrfb->req_status = DMA_CHAN_ALLOTED;
	init_waitqueue_head(&vrfb->wait);

	return rc;

failed_dma:
	omapvout_mem_free(vrfb->phy_addr[1], vrfb->virt_addr[1], size);
failed_mem1:
	omapvout_mem_free(vrfb->phy_addr[0], vrfb->virt_addr[0], size);
failed_mem0:
	omap_vrfb_release_ctx(&vrfb->ctx[1]);
failed_ctx1:
	omap_vrfb_release_ctx(&vrfb->ctx[0]);
failed_ctx0:
	return rc;
}

static int omapvout_dss_release_vrfb(struct omapvout_device *vout)
{
	int rc = 0;
	int size;
	struct omapvout_dss_vrfb *vrfb;

	/* It is assumed that the caller has locked the vout mutex */

	vrfb = &vout->dss->vrfb;
	if (vrfb->req_status == DMA_CHAN_ALLOTED) {
		vrfb->req_status = DMA_CHAN_NOT_ALLOTED;
		omap_free_dma(vrfb->dma_ch);
		/* FIXME: de-init the wait queue? */

		size = vrfb->size;
		omapvout_mem_free(vrfb->phy_addr[0], vrfb->virt_addr[0], size);
		omapvout_mem_free(vrfb->phy_addr[1], vrfb->virt_addr[1], size);

		omap_vrfb_release_ctx(&vrfb->ctx[0]);
		omap_vrfb_release_ctx(&vrfb->ctx[1]);
	}

	return rc;
}

static int omapvout_dss_perform_vrfb_dma(struct omapvout_device *vout,
					int buf_idx, bool vrfb_cfg)
{
	int rc = 0;
	struct omapvout_dss_vrfb *vrfb;
	u32 src_paddr;
	u32 dst_paddr;

	/* It is assumed that the caller has locked the vout mutex */

	if (vout->rotation == 0)
		return 0;

	if (vout->dss->vrfb.req_status != DMA_CHAN_ALLOTED)
		return -EINVAL;

	vrfb = &vout->dss->vrfb;

	if (vrfb_cfg) {
		enum omap_color_mode dss_fmt;
		int bytespp;
		int w, h;
		u32 fmt = vout->pix.pixelformat;

		w = vout->crop.width;
		h = vout->crop.height;

		dss_fmt = omapvout_dss_color_mode(vout->pix.pixelformat);
		omap_vrfb_setup(&vrfb->ctx[0], vrfb->phy_addr[0],
							w, h, dss_fmt);
		omap_vrfb_setup(&vrfb->ctx[1], vrfb->phy_addr[1],
							w, h, dss_fmt);

		bytespp = omapvout_dss_format_bytespp(vout->pix.pixelformat);
		vrfb->en = (w * bytespp) / 4; /* 32 bit ES */
		vrfb->fn = h;
		vrfb->dst_ei = 1;
		if (fmt == V4L2_PIX_FMT_YUYV || fmt == V4L2_PIX_FMT_UYVY) {
			vrfb->dst_fi = (OMAP_VRFB_LINE_LEN * bytespp * 2)
							- (vrfb->en * 4) + 1;
		} else {
			vrfb->dst_fi = (OMAP_VRFB_LINE_LEN * bytespp)
							- (vrfb->en * 4) + 1;
		}
	}

	src_paddr = vout->fq[buf_idx].phy_addr;
	dst_paddr = vrfb->ctx[vrfb->next].paddr[0];

	omap_set_dma_transfer_params(vrfb->dma_ch, OMAP_DMA_DATA_TYPE_S32,
				vrfb->en, vrfb->fn, OMAP_DMA_SYNC_ELEMENT,
				vrfb->dma_id, 0x0);
	omap_set_dma_src_params(vrfb->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
				src_paddr, 0, 0);
	omap_set_dma_src_burst_mode(vrfb->dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_params(vrfb->dma_ch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
				dst_paddr, vrfb->dst_ei, vrfb->dst_fi);
	omap_set_dma_dest_burst_mode(vrfb->dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

	vrfb->dma_complete = false;
	omap_start_dma(vrfb->dma_ch);
	wait_event_interruptible_timeout(vrfb->wait, vrfb->dma_complete,
							VRFB_TX_TIMEOUT);

	if (!vrfb->dma_complete) {
		DBG("VRFB DMA timeout\n");
		omap_stop_dma(vrfb->dma_ch);
		return -EINVAL;
	}

	return rc;
}

static int omapvout_dss_update_overlay(struct omapvout_device *vout,
							int buf_idx)
{
	struct omap_overlay_info o_info;
	struct omap_overlay *ovly;
	int rc = 0;
	int rot = vout->rotation;

	/* It is assumed that the caller has locked the vout mutex */

	/* Populate the overlay info struct and set it */
	memset(&o_info, 0, sizeof(o_info));
	o_info.enabled = true;
	if (rot == 0) {
		o_info.paddr = vout->fq[buf_idx].phy_addr;
		o_info.paddr += vout->dss->foffset;
		o_info.vaddr = NULL;
		o_info.screen_width = vout->pix.width;
	} else {
		struct omapvout_dss_vrfb *vrfb;

		vrfb = &vout->dss->vrfb;
		o_info.paddr = vrfb->ctx[vrfb->next].paddr[rot];
		o_info.paddr += vout->dss->foffset;
		o_info.vaddr = NULL;
		o_info.screen_width = OMAP_VRFB_LINE_LEN;

		vrfb->next = (vrfb->next) ? 0 : 1;
	}

	if (rot == 1 || rot == 3) { /* 90 or 270 degree rotation */
		o_info.width = vout->crop.height;
		o_info.height = vout->crop.width;
	} else {
		o_info.width = vout->crop.width;
		o_info.height = vout->crop.height;
	}

	o_info.pos_x = vout->win.w.left & ~1;
	o_info.pos_y = vout->win.w.top & ~1;
	o_info.out_width = vout->win.w.width;
	o_info.out_height = vout->win.w.height;
	o_info.color_mode = omapvout_dss_color_mode(vout->pix.pixelformat);
	if (rot == 0)
		o_info.rotation_type = OMAP_DSS_ROT_DMA;
	else
		o_info.rotation_type = OMAP_DSS_ROT_VRFB;

	o_info.rotation = rot;
	o_info.mirror = false;

	ovly = vout->dss->overlay;
	rc = ovly->set_overlay_info(ovly, &o_info);
	if (rc) {
		DBG("Failed setting the overlay info %d\n", rc);
		return rc;
	}

	rc = ovly->manager->apply(ovly->manager);
	if (rc) {
		DBG("Failed apply to overlay manager %d\n", rc);
		return rc;
	}

	rc = ovly->manager->device->update(ovly->manager->device,
					o_info.pos_x, o_info.pos_y,
					o_info.out_width, o_info.out_height);
	if (rc)
		DBG("Overlay update failed %d\n", rc);

	return rc;
}

static void omapvout_dss_perform_update(struct work_struct *work)
{
	struct omapvout_device *vout;
	struct omapvout_dss *dss;
	struct omap_dss_device *dev;
	int i;
	int rc;
	int idx = 0;
	u32 seqn = INVALID_SEQ_NUM;

	dss = container_of(work, struct omapvout_dss, work);
	vout = dss->vout;

	if (dss->exit_work)
		return;

	if (!dss->enabled)
		return;

	mutex_lock(&vout->mtx);

	/* Find a frame to process */
	for (i = vout->fq_cnt - 1; i >= 0; i--) {
		if (vout->fq[i].seq_num != INVALID_SEQ_NUM) {
			if (seqn == INVALID_SEQ_NUM ||
					vout->fq[i].seq_num < seqn) {
				seqn = vout->fq[i].seq_num;
				idx = i;
			}
		}
	}

	if (seqn == INVALID_SEQ_NUM) {
		DBG("No frame found to process\n");
		goto failed;
	}

	vout->fq[idx].seq_num = INVALID_SEQ_NUM;

	if (dss->need_cfg) {
		rc = omapvout_dss_calc_offset(vout);
		if (rc != 0) {
			DBG("Offset calculation failed %d\n", rc);
			goto failed_w_idx;
		}

		rc = omapvout_dss_config_colorkey(vout, true);
		if (rc != 0) {
			DBG("Alpha config failed %d\n", rc);
			goto failed_w_idx;
		}
	}

	rc = omapvout_dss_perform_vrfb_dma(vout, idx, dss->need_cfg);
	if (rc != 0) {
		DBG("VRFB rotation failed %d\n", rc);
		goto failed_w_idx;
	}

	rc = omapvout_dss_update_overlay(vout, idx);
	if (rc != 0) {
		DBG("DSS update failed %d\n", rc);
		goto failed_w_idx;
	}

	dss->need_cfg = false;

	mutex_unlock(&vout->mtx);

	/* Wait until the new frame is being used.  There is no problem
	 * doing this here since we are in a worker thread.  The mutex
	 * is unlocked since the sync may take some time.
	 */
	dev = dss->overlay->manager->device;
	if (dev->sync)
		dev->sync(dev);

	/* Since the mutex was unlocked, it is possible that the DSS may
	 * be exiting when we return, so check for this and exit if so.
	 */
	if (dss->exit_work)
		return;

	mutex_lock(&vout->mtx);

	if (vout->rotation == 0) {
		if (vout->fq_cur_idx != -1) {
			vout->fq[vout->fq_cur_idx].flags &=
						~(V4L2_BUF_FLAG_QUEUED);
			vout->fq[vout->fq_cur_idx].flags |= V4L2_BUF_FLAG_DONE;
			wake_up_interruptible(&vout->fq_wait);
		}

		vout->fq_cur_idx = idx;
	} else {
		vout->fq[idx].flags &= ~(V4L2_BUF_FLAG_QUEUED);
		vout->fq[idx].flags |= V4L2_BUF_FLAG_DONE;
		wake_up_interruptible(&vout->fq_wait);
	}

	mutex_unlock(&vout->mtx);

	return;

failed_w_idx:
	/* Set the done flag on failures to be sure the buffer can be DQ'd */
	vout->fq[idx].flags &= ~(V4L2_BUF_FLAG_QUEUED);
	vout->fq[idx].flags |= V4L2_BUF_FLAG_DONE;
	wake_up_interruptible(&vout->fq_wait);
failed:
	mutex_unlock(&vout->mtx);
}

/*=== Public Functions =================================================*/

int  omapvout_dss_init(struct omapvout_device *vout, enum omap_plane plane)
{
	struct omap_overlay *ovly;
	int rc = 0;
	int i;
	int cnt;

	vout->dss = kzalloc(sizeof(struct omapvout_dss), GFP_KERNEL);
	if (vout->dss == NULL) {
		rc = -ENOMEM;
		goto failed;
	}

	/* Retrieve the desired DSS overlay object */
	vout->dss->overlay = NULL;
	cnt = omap_dss_get_num_overlays();
	for (i = 0; i < cnt; i++) {
		ovly = omap_dss_get_overlay(i);
		if (ovly->id == plane) {
			vout->dss->overlay = ovly;
			break;
		}
	}

	if (vout->dss->overlay == NULL) {
		DBG("No overlay %d found\n", plane);
		rc = -ENODEV;
		goto failed_mem;
	}

	rc = omapvout_dss_acquire_vrfb(vout);
	if (rc != 0) {
		DBG("VRFB allocation failed\n");
		goto failed_mem;
	}

	vout->dss->vout = vout;

	return rc;

failed_mem:
	kfree(vout->dss);
failed:
	return rc;
}

void omapvout_dss_remove(struct omapvout_device *vout)
{
	if (vout->dss != NULL) {
		omapvout_dss_release_vrfb(vout);
		kfree(vout->dss);
	}
}

int omapvout_dss_open(struct omapvout_device *vout, u16 *disp_w, u16 *disp_h)
{
	struct omap_dss_device *dev;
	int rc = 0;

	if (vout->dss->overlay->manager == NULL) {
		DBG("No manager found\n");
		rc = -ENODEV;
		goto failed;
	}

	if (vout->dss->overlay->manager->device == NULL) {
		DBG("No device found\n");
		rc = -ENODEV;
		goto failed;
	}

	dev = vout->dss->overlay->manager->device;

	/* TODO: Do we need to deal with rotation? */
	dev->get_resolution(dev, disp_w, disp_h);

	vout->dss->workqueue = create_singlethread_workqueue("OMAPVOUT-DSS");
	if (vout->dss->workqueue == NULL) {
		rc = -ENOMEM;
		goto failed;
	}
	INIT_WORK(&vout->dss->work, omapvout_dss_perform_update);

	vout->dss->exit_work = false;
	vout->dss->enabled = false;

failed:
	return rc;
}

void omapvout_dss_release(struct omapvout_device *vout)
{
	vout->dss->exit_work = true;

	flush_workqueue(vout->dss->workqueue);
	destroy_workqueue(vout->dss->workqueue);
}

bool omapvout_dss_is_rotation_supported(struct omapvout_device *vout)
{
	return vout->dss->vrfb.req_status == DMA_CHAN_ALLOTED;
}

int omapvout_dss_enable(struct omapvout_device *vout)
{
	/* It is assumed that the caller has locked the vout mutex */

	/* Force a reconfiguration */
	vout->dss->need_cfg = true;

	vout->dss->enabled = true;

	return 0;
}

void omapvout_dss_disable(struct omapvout_device *vout)
{
	int rc = 0;
	struct omap_overlay_info o_info;
	struct omap_overlay *ovly;

	/* It is assumed that the caller has locked the vout mutex */

	memset(&o_info, 0, sizeof(o_info));
	o_info.enabled = false;

	rc = omapvout_dss_config_colorkey(vout, false);
	if (rc)
		DBG("Disabling alpha failed %d\n", rc);

	ovly = vout->dss->overlay;
	rc = ovly->set_overlay_info(ovly, &o_info);
	if (rc)
		DBG("Setting overlay info failed %d\n", rc);

	rc = ovly->manager->apply(ovly->manager);
	if (rc)
		DBG("Overlay manager apply failed %d\n", rc);

	rc = ovly->manager->device->update(ovly->manager->device,
				0, 0, vout->disp_width, vout->disp_height);
	if (rc)
		DBG("Display update failed %d\n", rc);

	vout->dss->enabled = false;
}

int omapvout_dss_update(struct omapvout_device *vout)
{
	if (!vout->dss->enabled) {
		DBG("DSS overlay is not enabled\n");
		return -EINVAL;
	}

	if (queue_work(vout->dss->workqueue, &vout->dss->work) == 0) {
		DBG("Queuing DSS work failed\n");
		return -EINVAL;
	}

	return 0;
}


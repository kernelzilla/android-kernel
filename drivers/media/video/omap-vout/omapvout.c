/*
 * drivers/media/video/omap/omapvout.c
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>

#include <mach/io.h>

#include "omapvout.h"
#include "omapvout-dss.h"
#include "omapvout-mem.h"

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
#include "omapvout-bp.h"
#endif

/*=====================================================*/
/* These should be defined in a platform specific file */
#ifndef OMAPVOUT_VIDEO_1_DEVICE_ID
#define OMAPVOUT_VIDEO_1_DEVICE_ID	1
#endif

#ifndef OMAPVOUT_VIDEO_2_DEVICE_ID
#define OMAPVOUT_VIDEO_2_DEVICE_ID	2
#endif

#ifndef OMAPVOUT_VIDEO_MAX_WIDTH
#define OMAPVOUT_VIDEO_MAX_WIDTH	864
#endif

#ifndef OMAPVOUT_VIDEO_MAX_HEIGHT
#define OMAPVOUT_VIDEO_MAX_HEIGHT	648
#endif

#ifndef OMAPVOUT_VIDEO_MAX_BPP
#define OMAPVOUT_VIDEO_MAX_BPP		2
#endif

#ifndef OMAPVOUT_VIDEO_BP_BUF_COUNT
#define OMAPVOUT_VIDEO_BP_BUF_COUNT	6
#endif
#ifndef OMAPVOUT_VIDEO_BP_BUF_SIZE
#define OMAPVOUT_VIDEO_BP_BUF_SIZE	\
	PAGE_ALIGN(OMAPVOUT_VIDEO_MAX_WIDTH * \
		   OMAPVOUT_VIDEO_MAX_HEIGHT * 2)
#endif
/*=====================================================*/


#define MODULE_NAME "omapvout"
#define VOUT1_NAME  "omapvout1"
#define VOUT2_NAME  "omapvout2"

#define V4L2_CID_PRIV_OFFSET		0x00530000 /* Arbitrary, semi-unique */
#define V4L2_CID_PRIV_ROTATION		(V4L2_CID_PRIVATE_BASE \
						+ V4L2_CID_PRIV_OFFSET + 0)
#define V4L2_CID_PRIV_COLORKEY		(V4L2_CID_PRIVATE_BASE \
						+ V4L2_CID_PRIV_OFFSET + 1)
#define V4L2_CID_PRIV_COLORKEY_EN	(V4L2_CID_PRIVATE_BASE \
						+ V4L2_CID_PRIV_OFFSET + 2)

/* list of image formats supported by OMAP2 video pipelines */
const static struct v4l2_fmtdesc omap2_formats[] = {
{
	/* Note:  V4L2 defines RGB565 as:
	 *      Byte 0                    Byte 1
	 *      g2 g1 g0 r4 r3 r2 r1 r0   b4 b3 b2 b1 b0 g5 g4 g3
	 *
	 * We interpret RGB565 as:
	 *      Byte 0                    Byte 1
	 *      g2 g1 g0 b4 b3 b2 b1 b0   r4 r3 r2 r1 r0 g5 g4 g3
	 */
	.description = "RGB565, le",
	.pixelformat = V4L2_PIX_FMT_RGB565,
},
{
	/* Note:  V4L2 defines RGB32 as: RGB-8-8-8-8  we use
	 *        this for RGB24 unpack mode, the last 8 bits are ignored
	 */
	.description = "RGB32, le",
	.pixelformat = V4L2_PIX_FMT_RGB32,
},
{
	.description = "YUYV (YUV 4:2:2), packed",
	.pixelformat = V4L2_PIX_FMT_YUYV,
},
{
	.description = "UYVY (YUV 4:2:2), packed",
	.pixelformat = V4L2_PIX_FMT_UYVY,
},
};

#define NUM_OUTPUT_FORMATS (sizeof(omap2_formats)/sizeof(omap2_formats[0]))

/*=== Local Functions ==================================================*/

static int omapvout_try_pixel_format(struct omapvout_device *vout,
				struct v4l2_pix_format *pix)
{
	int ifmt;
	int bpp = 0;

	if (pix->width > OMAPVOUT_VIDEO_MAX_WIDTH)
		pix->width = OMAPVOUT_VIDEO_MAX_WIDTH;

	if (pix->height > OMAPVOUT_VIDEO_MAX_HEIGHT)
		pix->height = OMAPVOUT_VIDEO_MAX_HEIGHT;

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++) {
		if (pix->pixelformat == omap2_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt >= NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = omap2_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->priv = 0;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_RGB565:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = 2;
		break;
	case V4L2_PIX_FMT_RGB32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = 4;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = 2;
		break;
	}

	pix->bytesperline = pix->width * bpp;
	pix->sizeimage = pix->bytesperline * pix->height;

	return 0;
}

/* Given a new render window in new_win, adjust the window to the
 * nearest supported configuration.  The adjusted window parameters are
 * returned in new_win.
 * Returns zero if succesful, or -EINVAL if the requested window is
 * impossible and cannot reasonably be adjusted.
 */
static int omapvout_try_window(struct omapvout_device *vout,
				struct v4l2_window *win)
{
	struct v4l2_rect try_win;

	/* make a working copy of the new_win rectangle */
	try_win = win->w;

	/* adjust the preview window so it fits on the display by clipping any
	 * offscreen areas
	 */
	if (try_win.left < 0) {
		try_win.width += try_win.left;
		try_win.left = 0;
	}
	if (try_win.top < 0) {
		try_win.height += try_win.top;
		try_win.top = 0;
	}

	try_win.width = (try_win.width < vout->disp_width) ?
	    try_win.width : vout->disp_width;
	try_win.height = (try_win.height < vout->disp_height) ?
	    try_win.height : vout->disp_height;

	if (try_win.left + try_win.width > vout->disp_width)
		try_win.width = vout->disp_width - try_win.left;
	if (try_win.top + try_win.height > vout->disp_height)
		try_win.height = vout->disp_height - try_win.top;

	try_win.width &= ~1;
	try_win.height &= ~1;

	if (try_win.width <= 0 || try_win.height <= 0)
		return -EINVAL;

	/* We now have a valid preview window, so go with it */
	win->w = try_win;
	win->field = V4L2_FIELD_NONE;

	return 0;
}

/* Return the default overlay cropping rectangle in crop given the image
 * size in pix and the video display size in fbuf.  The default
 * cropping rectangle is the largest rectangle no larger than the capture size
 * that will fit on the display.  The default cropping rectangle is centered in
 * the image.  All dimensions and offsets are rounded down to even numbers.
 */
void omapvout_default_crop(struct omapvout_device *vout,
			struct v4l2_rect *crop)
{
	crop->width = (vout->pix.width < vout->disp_width) ?
		vout->pix.width : vout->disp_width;
	crop->height = (vout->pix.height < vout->disp_height) ?
		vout->pix.height : vout->disp_height;
	crop->width &= ~1;
	crop->height &= ~1;
	crop->left = ((vout->pix.width - crop->width) >> 1) & ~1;
	crop->top = ((vout->pix.height - crop->height) >> 1) & ~1;
}

/* Given a new cropping rectangle in new_crop, adjust the cropping rectangle to
 * the nearest supported configuration.  The image render window in win will
 * also be adjusted if necessary.  The preview window is adjusted such that the
 * horizontal and vertical rescaling ratios stay constant.  If the render
 * window would fall outside the display boundaries, the cropping rectangle will
 * also be adjusted to maintain the rescaling ratios.  If successful, crop
 * and win are updated.
 * Returns zero if succesful, or -EINVAL if the requested cropping rectangle is
 * impossible and cannot reasonably be adjusted.
 */
int omapvout_try_crop(struct omapvout_device *vout, struct v4l2_rect *crop)
{
	struct v4l2_rect try;

	/* make a working copy of the new_crop rectangle */
	try = *crop;

	/* adjust the cropping rectangle so it fits in the image */
	if (try.left < 0) {
		try.width += try.left;
		try.left = 0;
	}
	if (try.top < 0) {
		try.height += try.top;
		try.top = 0;
	}
	try.width = (try.width < vout->pix.width) ?
		try.width : vout->pix.width;
	try.height = (try.height < vout->pix.height) ?
		try.height : vout->pix.height;
	if (try.left + try.width > vout->pix.width)
		try.width = vout->pix.width - try.left;
	if (try.top + try.height > vout->pix.height)
		try.height = vout->pix.height - try.top;
	try.width &= ~1;
	try.height &= ~1;

	if (try.width <= 0 || try.height <= 0)
		return -EINVAL;

	/* Check for resizing constraints */
	if (try.height / vout->win.w.height >= 16) {
		/* The maximum vertical downsizing ratio is 16:1 */
		try.height = vout->win.w.height * 16;
	}
	if (try.width / vout->win.w.width >= 16) {
		/* The maximum vertical downsizing ratio is 16:1 */
		try.width = vout->win.w.width * 16;
	}

	/* update our cropping rectangle and we're done */
	*crop = try;

	return 0;
}

void omapvout_acquire_frames(struct omapvout_device *vout, int cnt, int size)
{
	int i;
	int fcnt;
	u32 paddr;
	u32 vaddr;
	u32 fs;

	/* It is assumed that the vout->mtx is locked for this call */

	vout->fq_min_size = 0x7FFFFFF; /* Some large value */
	fcnt = 0;
	for (i = 0; i < cnt; i++) {
#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
		if (vout->bp &&	omapvout_bp_alloc(vout, size,
					&paddr, &vaddr, &fs) == 0) {
			DBG("Alloc'd from the pool\n");
		} else {
#else
		{
#endif
			if (omapvout_mem_alloc(size, &paddr, &vaddr)) {
				DBG("Alloc failed %d\n", i);
				break;
			}
			fs = size;
		}

		memset((void *)vaddr, 0, fs);

		vout->fq[fcnt].flags = V4L2_BUF_FLAG_DONE;
		vout->fq[fcnt].size = fs;
		vout->fq[fcnt].phy_addr = paddr;
		vout->fq[fcnt].virt_addr = vaddr;
		/* Using OMAPVOUT_VIDEO_BP_BUF_SIZE is just convenient */
		vout->fq[fcnt].offset = OMAPVOUT_VIDEO_BP_BUF_SIZE * i;

		fcnt++;

		if (fs < vout->fq_min_size)
			vout->fq_min_size = fs;
	}

	vout->fq_cnt = fcnt;
	vout->fq_dq_idx = 0;
}

void omapvout_release_frames(struct omapvout_device *vout)
{
	int i;
	u32 paddr;
	u32 vaddr;
	u32 size;

	/* It is assumed that the vout->mtx is locked for this call */

	for (i = 0; i < vout->fq_cnt; i++) {
		paddr = vout->fq[i].phy_addr;
		vaddr = vout->fq[i].virt_addr;
		size = vout->fq[i].size;

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
		if (omapvout_is_bp_buffer(vout, paddr)) {
			if (omapvout_bp_release(vout, paddr))
				DBG("Error releasing to the pool\n");
		} else {
#else
		{
#endif
			omapvout_mem_free(paddr, vaddr, size);
		}
	}

	vout->fq_cnt = 0;
	vout->fq_min_size = 0;
	vout->fq_dq_idx = 0;
	memset(vout->fq, 0, sizeof(vout->fq));
}

static void omapvout_free_resources(struct omapvout_device *vout)
{
	DBG("free_resources\n");

	if (vout == NULL)
		return;

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
	if (vout->bp != NULL)
		omapvout_bp_destroy(vout);
#endif

	video_set_drvdata(&vout->vdev, NULL);
	kfree(vout);
}

static int omapvout_dqbuf(struct omapvout_device *vout, int *idx)
{
	int i, j;

	j = vout->fq_dq_idx;

	for (i = 0; i < vout->fq_cnt; i++) {
		if (vout->fq[j].flags & V4L2_BUF_FLAG_DONE) {
			DBG("DQ'ing Frame %d\n", j);
			vout->fq_dq_idx = j;
			break;
		}

		j++;
		if (j >= vout->fq_cnt)
			j = 0;
	}

	if (i >= vout->fq_cnt)
		return -EINVAL;

	*idx = j;

	return 0;
}

/*=== V4L2 Interface Functions =========================================*/

static int omapvout_open(struct file *file)
{
	struct omapvout_device *vout;
	u16 w, h;
	int rc;

	DBG("omapvout_open\n");

	vout = video_drvdata(file);

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	/* We only support single open */
	if (vout->opened) {
		DBG("Device already opened\n");
		rc = -EBUSY;
		goto failed;
	}

	rc = omapvout_dss_open(vout, &w, &h);
	if (rc != 0)
		goto failed;

	DBG("Overlay Display %dx%d\n", w, h);

	if (w == 0 || h == 0) {
		DBG("Invalid display resolution\n");
		rc = -EINVAL;
		goto failed;
	}

	vout->disp_width = w;
	vout->disp_height = h;
	vout->opened = 1;

	memset(&vout->pix, 0, sizeof(vout->pix));
	vout->pix.width = w;
	vout->pix.height = h;
	vout->pix.field = V4L2_FIELD_NONE;
	vout->pix.pixelformat = V4L2_PIX_FMT_RGB565; /* Arbitrary */
	vout->pix.colorspace = V4L2_COLORSPACE_SRGB; /* Arbitrary */
	vout->pix.bytesperline = w * 2;
	vout->pix.sizeimage = w * h * 2;

	memset(&vout->win, 0, sizeof(vout->win));
	vout->win.w.width = w;
	vout->win.w.height = h;
	vout->win.field = V4L2_FIELD_NONE;

	memset(&vout->crop, 0, sizeof(vout->crop));
	vout->crop.width = w;
	vout->crop.height = h;

	vout->streaming = 0;
	vout->rotation = 0;
	vout->colorkey = 0;
	vout->colorkey_en = 0;

	memset(&vout->fq, 0, sizeof(vout->fq));
	vout->fq_cnt = 0;
	vout->fq_min_size = 0;
	vout->fq_dq_idx = 0;
	vout->fq_cur_idx = 0;

	vout->mmap_cnt = 0;

	mutex_unlock(&vout->mtx);

	file->private_data = vout;

	return 0;

failed:
	mutex_unlock(&vout->mtx);
	return rc;
}

static int omapvout_release(struct file *file)
{
	struct omapvout_device *vout;

	DBG("omapvout_release\n");

	vout = video_drvdata(file);

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	if (vout->streaming)
		omapvout_dss_disable(vout);

	if (vout->mmap_cnt) {
		vout->mmap_cnt = 0;
		DBG("Releasing with non-zero mmap_cnt\n");
	}

	omapvout_dss_release(vout);

	omapvout_release_frames(vout);

	vout->opened = 0;

	/* Force the DQ waiter (if present) to wakeup */
	wake_up_interruptible(&vout->fq_wait);

	mutex_unlock(&vout->mtx);

	file->private_data = NULL;

	return 0;
}

static void omapvout_vm_open(struct vm_area_struct *vma)
{
	struct omapvout_device *vout = vma->vm_private_data;
	DBG("vm_open [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_cnt++;
}

static void omapvout_vm_close(struct vm_area_struct *vma)
{
	struct omapvout_device *vout = vma->vm_private_data;
	DBG("vm_close [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_cnt--;
}

static struct vm_operations_struct omapvout_vm_ops = {
	.open = omapvout_vm_open,
	.close = omapvout_vm_close,
};

static int omapvout_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omapvout_device *vout;
	int idx;
	int rc;
	u32 offset;

	vout = video_drvdata(file);

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	/* look for the buffer to map */
	offset = (vma->vm_pgoff << PAGE_SHIFT);
	for (idx = 0; idx < vout->fq_cnt; idx++) {
		if (vout->fq[idx].offset == offset)
			break;
	}

	if (idx >= vout->fq_cnt) {
		DBG("Invalid offset 0x%lx\n", (unsigned long) offset);
		mutex_unlock(&vout->mtx);
		rc = -EINVAL;
		goto failed;
	}

	DBG("omapvout_mmap %d\n", idx);

	vma->vm_ops = &omapvout_vm_ops;
	vma->vm_private_data = (void *) vout;

	rc = omapvout_mem_map(vma, vout->fq[idx].phy_addr);
	if (rc != 0) {
		DBG("Failed mem_map %d\n", rc);
		goto failed;
	}

	vout->mmap_cnt++;

	mutex_unlock(&vout->mtx);

	return 0;

failed:
	mutex_unlock(&vout->mtx);
	return rc;
}

static int omapvout_vidioc_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	struct omapvout_device *vout = priv;

	memset(cap, 0, sizeof(*cap));
	strncpy(cap->driver, MODULE_NAME, sizeof(cap->driver));
	strncpy(cap->card, vout->vdev.name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;

	return 0;
}

static int omapvout_vidioc_enum_output(struct file *file, void *priv,
				struct v4l2_output *output)
{
	int index = output->index;

	if (index > 0)
		return -EINVAL;

	memset(output, 0, sizeof(*output));
	output->index = index;

	strncpy(output->name, "video out", sizeof(output->name));
	output->type = V4L2_OUTPUT_TYPE_MODULATOR;

	return 0;
}

static int omapvout_vidioc_g_output(struct file *file, void *priv,
				unsigned int *i)
{
	*i = 0;

	return 0;
}

static int omapvout_vidioc_s_output(struct file *file, void *priv,
				unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static int omapvout_vidioc_enum_fmt_vid_overlay(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	int index = f->index;
	enum v4l2_buf_type type = f->type;

	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	memset(f, 0, sizeof(*f));
	f->index = index;
	f->type = type;
	f->flags = omap2_formats[index].flags;
	strncpy(f->description, omap2_formats[index].description,
					sizeof(f->description));
	f->pixelformat = omap2_formats[index].pixelformat;

	return 0;
}

static int omapvout_vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	/* Same formats as the overlay */
	return omapvout_vidioc_enum_fmt_vid_overlay(file, priv, f);
}

static int omapvout_vidioc_g_fmt_vid_overlay(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_window *win = &f->fmt.win;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	/*
	 * The API has a bit of a problem here. We're returning a v4l2_window
	 * structure, but that structure contains pointers to variable-sized
	 * objects for clipping rectangles and clipping bitmaps.  We will just
	 * return NULLs for those pointers.
	 */

	mutex_lock(&vout->mtx);

	memset(win, 0, sizeof(*win));
	win->w = vout->win.w;
	win->field = vout->win.field;
	win->chromakey = vout->win.chromakey;
	win->global_alpha = vout->win.global_alpha;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	memset(pix, 0, sizeof(*pix));
	*pix = vout->pix;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_try_fmt_vid_overlay(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_window *win = &f->fmt.win;
	int rc;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	rc = omapvout_try_window(vout, win);

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_try_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rc;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	rc = omapvout_try_pixel_format(vout, pix);

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_s_fmt_vid_overlay(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_window *win = &f->fmt.win;
	int rc = 0;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (vout->streaming)
		return -EBUSY;

	mutex_lock(&vout->mtx);

	rc = omapvout_try_window(vout, win);
	if (rc != 0)
		goto failed;

	vout->win.w = win->w;
	vout->win.field = win->field;
	vout->win.chromakey = win->chromakey;
	vout->win.global_alpha = win->global_alpha;

	/* Streaming has to be disabled, so config the hardware
	 * later when streaming is enabled
	 */

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rc = 0;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (vout->streaming)
		return -EBUSY;

	mutex_lock(&vout->mtx);

	rc = omapvout_try_pixel_format(vout, pix);
	if (rc != 0)
		goto failed;

	memcpy(&vout->pix, pix, sizeof(*pix));

	/* Don't allow the crop window to be larger than the video */
	omapvout_try_crop(vout, &vout->crop);

	/* Streaming has to be disabled, so config the hardware
	 * later when streaming is enabled
	 */

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_cropcap(struct file *file, void *priv,
				struct v4l2_cropcap *ccap)
{
	struct omapvout_device *vout = priv;
	enum v4l2_buf_type type = ccap->type;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	mutex_lock(&vout->mtx);

	memset(ccap, 0, sizeof(*ccap));
	ccap->type = type;
	ccap->bounds.width = vout->pix.width & ~1;
	ccap->bounds.height = vout->pix.height & ~1;
	omapvout_default_crop(vout, &ccap->defrect);
	ccap->pixelaspect.numerator = 1;
	ccap->pixelaspect.denominator = 1;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_g_crop(struct file *file, void *priv,
				struct v4l2_crop *crop)
{
	struct omapvout_device *vout = priv;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	mutex_lock(&vout->mtx);

	crop->c = vout->crop;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_s_crop(struct file *file, void *priv,
				struct v4l2_crop *crop)
{
	struct omapvout_device *vout = priv;
	struct v4l2_rect rect = crop->c;
	int rc = 0;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (vout->streaming)
		return -EBUSY;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	mutex_lock(&vout->mtx);

	rc = omapvout_try_crop(vout, &rect);
	if (rc != 0)
		goto failed;

	vout->crop = rect;

	/* Streaming has to be disabled, so config the hardware
	 * later when streaming is enabled
	 */

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_reqbufs(struct file *file, void *priv,
				struct v4l2_requestbuffers *req)
{
	struct omapvout_device *vout = priv;
	int rc;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (req->count > OMAPVOUT_VID_MAX_FRAMES)
		req->count = OMAPVOUT_VID_MAX_FRAMES;

	if (req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	if (req->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	/* FIXME: Really should allow if req->count is zero */
	if (vout->streaming)
		return -EBUSY;

	/* It is assumed that the video out format is correctly configured */
	if (vout->pix.sizeimage == 0)
		return -EINVAL;

	mutex_lock(&vout->mtx);

	if (req->count == 0) {
		omapvout_release_frames(vout);
		goto success;
	}

	/* Don't allow new buffers when some are still mapped */
	if (vout->mmap_cnt) {
		DBG("Buffers are still mapped\n");
		rc = -EBUSY;
		goto failed;
	}

	/* Use the existing frames if possible */
	if (req->count <= vout->fq_cnt &&
			vout->pix.sizeimage <= vout->fq_min_size)
		goto success;

	omapvout_release_frames(vout);
	omapvout_acquire_frames(vout, req->count, vout->pix.sizeimage);

	mutex_unlock(&vout->mtx);

	if (vout->fq_min_size < vout->pix.sizeimage) {
		DBG("Buffer allocation failed\n");
		return -ENOMEM;
	}

	req->count = vout->fq_cnt;

	return 0;

success:
failed:
	mutex_unlock(&vout->mtx);
	return rc;
}

static int omapvout_vidioc_querybuf(struct file *file, void *priv,
				struct v4l2_buffer *b)
{
	struct omapvout_device *vout = priv;
	int rc;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	mutex_lock(&vout->mtx);

	if (b->index >= vout->fq_cnt) {
		DBG("Invalid querybuf index %d\n", b->index);
		rc = -EINVAL;
		goto failed;
	}

	b->memory = V4L2_MEMORY_MMAP;
	b->flags = vout->fq[b->index].flags;
	b->length = vout->fq[b->index].size;
	b->m.offset = vout->fq[b->index].offset;

	mutex_unlock(&vout->mtx);

	return 0;

failed:
	mutex_unlock(&vout->mtx);
	return rc;
}

static int omapvout_vidioc_qbuf(struct file *file, void *priv,
				struct v4l2_buffer *b)
{
	struct omapvout_device *vout = priv;
	int rc = 0;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (!vout->streaming)
		return -EINVAL;

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	if (b->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	if (b->index >= vout->fq_cnt)
		return -EINVAL;

	DBG("Q'ing Frame %d\n", b->index);

	mutex_lock(&vout->mtx);

	vout->fq[b->index].flags |= V4L2_BUF_FLAG_QUEUED|V4L2_BUF_FLAG_MAPPED;
	vout->fq[b->index].flags &= ~(V4L2_BUF_FLAG_DONE);

	/* TODO: Deal with the wrap-around case? */
	vout->fq[b->index].seq_num = vout->fq_next_seq++;

	rc = omapvout_dss_update(vout);

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_dqbuf(struct file *file, void *priv,
				struct v4l2_buffer *b)
{
	struct omapvout_device *vout = priv;
	int rc = 0;
	int idx;
	bool wait;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (b->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	wait = (file->f_flags & O_NONBLOCK) ? false : true;

	mutex_lock(&vout->mtx);

	if (vout->opened == 0) {
		rc = -EINVAL;
		goto failed;
	}

	if (omapvout_dqbuf(vout, &idx) != 0) {
		if (!wait) {
			mutex_unlock(&vout->mtx);
			return -EAGAIN;
		} else {
			wait_event_interruptible(vout->fq_wait,
					((omapvout_dqbuf(vout, &idx) == 0) ||
						(vout->opened == 0)));
		}
	}

	vout->fq[idx].flags &=
		~(V4L2_BUF_FLAG_QUEUED|V4L2_BUF_FLAG_DONE|V4L2_BUF_FLAG_MAPPED);

	memset(b, 0, sizeof(b));
	b->index = idx;
	b->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	b->memory = V4L2_MEMORY_MMAP;
	b->flags = vout->fq[idx].flags;
	b->length = vout->fq[idx].size;
	b->bytesused = 0;

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_streamon(struct file *file, void *priv,
				enum v4l2_buf_type type)
{
	struct omapvout_device *vout = priv;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	if (vout->streaming) {
		DBG("Streaming is already enabled\n");
		mutex_unlock(&vout->mtx);
		return -EBUSY;
	}

	/* Enable the streaming flag.
	 * There can't be any frames queued, so when the first frame
	 * is queued the DSS will be updated.
	 */
	vout->streaming = true;

	omapvout_dss_enable(vout);

	/* Reset the current frame idx & seq num */
	vout->fq_cur_idx = -1;
	vout->fq_next_seq = 1;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type type)
{
	struct omapvout_device *vout = priv;
	int rc = 0;
	int i;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	if (!vout->streaming) {
		DBG("Streaming is already disabled\n");
		mutex_unlock(&vout->mtx);
		return -EINVAL;
	}

	vout->streaming = false;

	omapvout_dss_disable(vout);

	/* Reset the queue and wakeup the DQ waiter */
	for (i = 0; i < vout->fq_cnt; i++) {
		if (vout->fq[i].flags & V4L2_BUF_FLAG_QUEUED) {
			vout->fq[i].flags &= ~(V4L2_BUF_FLAG_QUEUED);
			vout->fq[i].flags |= V4L2_BUF_FLAG_DONE;
		}
	}
	wake_up_interruptible(&vout->fq_wait);

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_queryctrl(struct file *file, void *priv,
				struct v4l2_queryctrl *a)
{
	/* TODO: Add me */
	return 0;
}

static int omapvout_vidioc_g_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct omapvout_device *vout = priv;
	int rc = 0;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	switch (ctrl->id) {
	case V4L2_CID_PRIV_ROTATION:
		ctrl->value = vout->rotation * 90;
		break;
	case V4L2_CID_PRIV_COLORKEY:
		ctrl->value = vout->colorkey;
		break;
	case V4L2_CID_PRIV_COLORKEY_EN:
		ctrl->value = vout->colorkey_en;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct omapvout_device *vout = priv;
	int v = ctrl->value;
	int rc = 0;

	if (vout == NULL) {
		DBG("Invalid device\n");
		return -ENODEV;
	}

	if (vout->streaming)
		return -EBUSY;

	mutex_lock(&vout->mtx);

	switch (ctrl->id) {
	case V4L2_CID_PRIV_ROTATION:
		if (!omapvout_dss_is_rotation_supported(vout) && v != 0) {
			rc = -EINVAL;
		} else if (v == 0 || v == 90 || v == 180 || v == 270) {
			vout->rotation = v / 90;
		} else {
			DBG("Invalid rotation %d\n", v);
			rc = -ERANGE;
		}
		break;
	case V4L2_CID_PRIV_COLORKEY:
		vout->colorkey = v;
		break;
	case V4L2_CID_PRIV_COLORKEY_EN:
		vout->colorkey_en = (v) ? 1 : 0;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	/* Streaming has to be disabled, so config the hardware
	 * later when streaming is enabled
	 */

	mutex_unlock(&vout->mtx);

	return rc;
}

/*=== Driver Functions =================================================*/

static struct v4l2_file_operations omapvout_fops = {
	.owner = THIS_MODULE,
	.open = omapvout_open,
	.release = omapvout_release,
	.mmap = omapvout_mmap,
	.ioctl = video_ioctl2,
};

static const struct v4l2_ioctl_ops omapvout_ioctl_ops = {
	.vidioc_querycap = omapvout_vidioc_querycap,
	.vidioc_enum_output = omapvout_vidioc_enum_output,
	.vidioc_g_output = omapvout_vidioc_g_output,
	.vidioc_s_output = omapvout_vidioc_s_output,
	.vidioc_enum_fmt_vid_overlay = omapvout_vidioc_enum_fmt_vid_overlay,
	.vidioc_enum_fmt_vid_out = omapvout_vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_overlay = omapvout_vidioc_g_fmt_vid_overlay,
	.vidioc_g_fmt_vid_out = omapvout_vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_overlay = omapvout_vidioc_try_fmt_vid_overlay,
	.vidioc_try_fmt_vid_out = omapvout_vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_overlay = omapvout_vidioc_s_fmt_vid_overlay,
	.vidioc_s_fmt_vid_out = omapvout_vidioc_s_fmt_vid_out,
	.vidioc_cropcap = omapvout_vidioc_cropcap,
	.vidioc_g_crop = omapvout_vidioc_g_crop,
	.vidioc_s_crop = omapvout_vidioc_s_crop,
	.vidioc_reqbufs = omapvout_vidioc_reqbufs,
	.vidioc_querybuf = omapvout_vidioc_querybuf,
	.vidioc_qbuf = omapvout_vidioc_qbuf,
	.vidioc_dqbuf = omapvout_vidioc_dqbuf,
	.vidioc_streamon = omapvout_vidioc_streamon,
	.vidioc_streamoff = omapvout_vidioc_streamoff,
	.vidioc_queryctrl = omapvout_vidioc_queryctrl,
	.vidioc_g_ctrl = omapvout_vidioc_g_ctrl,
	.vidioc_s_ctrl = omapvout_vidioc_s_ctrl,
};

static struct video_device omapvout_devdata = {
	.name = MODULE_NAME,
	.fops = &omapvout_fops,
	.ioctl_ops = &omapvout_ioctl_ops,
	.vfl_type = VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY,
	.release = video_device_release,
	.minor = -1,
};

static int __init omapvout_probe(struct platform_device *pdev,
				enum omap_plane plane, int vid)
{
	struct omapvout_device *vout = NULL;
	int rc = 0;

	DBG("omapvout_probe %d %d\n", plane, vid);

	vout = kzalloc(sizeof(struct omapvout_device), GFP_KERNEL);
	if (vout == NULL) {
		rc = -ENOMEM;
		goto err0;
	}

	mutex_init(&vout->mtx);

	vout->max_video_width = OMAPVOUT_VIDEO_MAX_WIDTH;
	vout->max_video_height = OMAPVOUT_VIDEO_MAX_HEIGHT;
	vout->max_video_bytespp = OMAPVOUT_VIDEO_MAX_BPP;

	rc = omapvout_dss_init(vout, plane);
	if (rc != 0) {
		printk(KERN_INFO "DSS init failed\n");
		goto cleanup;
	}

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
	vout->bp = dev_get_drvdata(&pdev->dev);
	omapvout_bp_init(vout);
#endif

	/* register the V4L2 interface */
	vout->vdev = omapvout_devdata;
	video_set_drvdata(&vout->vdev, vout);
	if (video_register_device(&vout->vdev, VFL_TYPE_GRABBER, vid) < 0) {
		printk(KERN_ERR MODULE_NAME": could not register with V4L2\n");
		rc = -EINVAL;
		goto cleanup;
	}

	vout->id = plane;
	init_waitqueue_head(&vout->fq_wait);

	return 0;

cleanup:
	omapvout_free_resources(vout);
err0:
	dev_err(&pdev->dev, "failed to setup omapvout\n");
	return rc;
}

static int __init omapvout1_probe(struct platform_device *pdev)
{
	return omapvout_probe(pdev, OMAP_DSS_VIDEO1,
				OMAPVOUT_VIDEO_1_DEVICE_ID);
}

static int __init omapvout2_probe(struct platform_device *pdev)
{
	return omapvout_probe(pdev, OMAP_DSS_VIDEO2,
				OMAPVOUT_VIDEO_2_DEVICE_ID);
}

static int omapvout_remove(struct platform_device *pdev)
{
	struct omapvout_device *vout = platform_get_drvdata(pdev);

	DBG("omapvout_remove\n");

	omapvout_dss_remove(vout);
	omapvout_free_resources(vout);

	return 0;
}

static struct platform_device omapvout1_dev = {
	.name = VOUT1_NAME,
	.id = 11,
};

static struct platform_device omapvout2_dev = {
	.name = VOUT2_NAME,
	.id = 12,
};

static struct platform_driver omapvout1_driver = {
	.remove         = omapvout_remove,
	.driver         = {
		.name   = VOUT1_NAME,
		.owner  = THIS_MODULE,
	},
};

static struct platform_driver omapvout2_driver = {
	.remove         = omapvout_remove,
	.driver         = {
		.name   = VOUT2_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init omapvout_init(void)
{
	struct omapvout_bp *bp;
	int rc;

	DBG("omapvout_init\n");

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
	/* Create a buffer pool and pass it to both driver probes */
	bp = omapvout_bp_create(OMAPVOUT_VIDEO_BP_BUF_COUNT,
				OMAPVOUT_VIDEO_BP_BUF_SIZE);
	omapvout1_dev.dev.driver_data = bp;
	omapvout2_dev.dev.driver_data = bp;
#endif

	rc = platform_device_register(&omapvout1_dev);
	if (rc != 0) {
		printk(KERN_ERR "failed omapvout1 device register %d\n", rc);
		goto faildev1;
	}

	rc = platform_driver_probe(&omapvout1_driver, omapvout1_probe);
	if (rc != 0) {
		printk(KERN_ERR "failed omapvout1 register/probe %d\n", rc);
		goto faildrv1;
	}

	rc = platform_device_register(&omapvout2_dev);
	if (rc != 0) {
		printk(KERN_ERR "failed omapvout2 device register %d\n", rc);
		goto faildev2;
	}

	rc = platform_driver_probe(&omapvout2_driver, omapvout2_probe);
	if (rc != 0) {
		printk(KERN_ERR "failed omapvout2 register/probe %d\n", rc);
		goto faildrv2;
	}

	return 0;

faildrv2:
	platform_device_unregister(&omapvout2_dev);
faildev2:
	platform_driver_unregister(&omapvout1_driver);
faildrv1:
	platform_device_unregister(&omapvout1_dev);
faildev1:
	return -ENODEV;
}

static void __exit omapvout_exit(void)
{
	DBG("omapvout_exit\n");
	platform_driver_unregister(&omapvout1_driver);
	platform_driver_unregister(&omapvout2_driver);
	platform_device_unregister(&omapvout1_dev);
	platform_device_unregister(&omapvout2_dev);
}

module_init(omapvout_init);
module_exit(omapvout_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("OMAP2/3 Video Out for V4L2");
MODULE_LICENSE("GPL");


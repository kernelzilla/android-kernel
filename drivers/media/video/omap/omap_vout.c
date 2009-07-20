/*
 * drivers/media/video/omap/omap_vout.c
 *
 * Copyright (C) 2005-2009 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Leveraged code from the OMAP2 camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2009 Texas Instruments.
 *
 * History:
 * 20-APR-2006	Khasim		Modified VRFB based Rotation,
 *				The image data is always read from 0 degree
 *				view and written
 *				to the virtual space of desired rotation angle
 * 4-DEC-2006 Jian		Changed to support better memory management
 *
 * 17-Nov-2008 Hardik		Changed to used the new DSS paches by Tomi
 *				Changed driver to use video_ioctl2
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/videodev2.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <media/videobuf-dma-sg.h>
#include <linux/input.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <mach/display.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/semaphore.h>
#include <asm/processor.h>
#include <mach/dma.h>
#include <mach/vrfb.h>
#include <media/v4l2-common.h>
#include <mach/display.h>
#include "omap_voutlib.h"
#include "omap_voutdef.h"

#define OMAP_VIDEO1 0
#define OMAP_VIDEO2 1

/* configuration macros */
#define VOUT_NAME		"omap_vout"

#define QQVGA_WIDTH		160
#define QQVGA_HEIGHT		120

#define NUM_OF_VIDEO_CHANNELS	2

#define VID_MAX_WIDTH		1280	/* Largest width */
#define VID_MAX_HEIGHT		720/* Largest height */

/* Mimimum requirement is 2x2 for DSS */
#define VID_MIN_WIDTH		2
#define VID_MIN_HEIGHT		2

/* 2048 x 2048 is max res supported by OMAP display controller */
#define DMA_CHAN_ALLOTED        1
#define DMA_CHAN_NOT_ALLOTED    0
#define MAX_PIXELS_PER_LINE     2048
#define VRFB_TX_TIMEOUT         1000

#define OMAP_VOUT_MAX_BUF_SIZE (VID_MAX_WIDTH*VID_MAX_HEIGHT*4)

/* IRQ Bits mask of DSS */
#define OMAP_VOUT_IRQ_MASK (DISPC_IRQ_VSYNC | DISPC_IRQ_EVSYNC_EVEN | \
			    DISPC_IRQ_EVSYNC_ODD | DISPC_IRQ_FRAMEDONE)


static struct videobuf_queue_ops video_vbq_ops;

static u32 video1_numbuffers = 3;
static u32 video2_numbuffers = 3;
static u32 video1_bufsize = OMAP_VOUT_MAX_BUF_SIZE;
static u32 video2_bufsize = OMAP_VOUT_MAX_BUF_SIZE;
static u32 vid1_static_vrfb_alloc;
static u32 vid2_static_vrfb_alloc;
static int debug;

/* Module parameters */
module_param(video1_numbuffers, uint, S_IRUGO);
MODULE_PARM_DESC(video1_numbuffers, "Number of buffers to be allocated at \
		init time for Video1 device.");

module_param(video2_numbuffers, uint, S_IRUGO);
MODULE_PARM_DESC(video2_numbuffers, "Number of buffers to be allocated at \
		init time for Video2 device.");

module_param(video1_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(video1_bufsize, "Size of the buffer to be allocated for \
		video1 device");

module_param(video2_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(video2_bufsize, "Size of the buffer to be allocated for \
		video2 device");

module_param(vid1_static_vrfb_alloc, bool, S_IRUGO);
MODULE_PARM_DESC(vid1_static_vrfb_alloc, "Static allocation of the VRFB \
		buffer for video1 device");

module_param(vid2_static_vrfb_alloc, bool, S_IRUGO);
MODULE_PARM_DESC(vid2_static_vrfb_alloc, "Static allocation of the VRFB \
		buffer for video2 device");

module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

/* Local Helper functions */
static int omap_vout_create_video_devices(struct platform_device *pdev);
static int omapvid_apply_changes(struct omap_vout_device *vout);
static int omapvid_init(struct omap_vout_device *vout, u32 addr);
static int omapvid_setup_overlay(struct omap_vout_device *vout,
		struct omap_overlay *ovl, int posx, int posy,
		int outw, int outh, u32 addr);
static enum omap_color_mode video_mode_to_dss_mode(struct omap_vout_device
		*vout);
static void omap_vout_isr(void *arg, unsigned int irqstatus);
static void omap_vout_cleanup_device(struct omap_vout_device *vout);

/*
 * Maximum amount of memory to use for rendering buffers.
 * Default is enough to four (RGB24) DVI 720P buffers.
 */
#define MAX_ALLOWED_VIDBUFFERS            4

/* list of image formats supported by OMAP2 video pipelines */
const static struct v4l2_fmtdesc omap_formats[] = {
	{
	 /* Note:  V4L2 defines RGB565 as:
	  *
	  *      Byte 0                    Byte 1
	  *      g2 g1 g0 r4 r3 r2 r1 r0   b4 b3 b2 b1 b0 g5 g4 g3
	  *
	  * We interpret RGB565 as:
	  *
	  *      Byte 0                    Byte 1
	  *      g2 g1 g0 b4 b3 b2 b1 b0   r4 r3 r2 r1 r0 g5 g4 g3
	  */
	 .description = "RGB565, le",
	 .pixelformat = V4L2_PIX_FMT_RGB565,
	 },
	{
	 /* Note:  V4L2 defines RGB32 as: RGB-8-8-8-8  we use
	  *  this for RGB24 unpack mode, the last 8 bits are ignored
	  * */
	 .description = "RGB32, le",
	 .pixelformat = V4L2_PIX_FMT_RGB32,
	 },
	{
	 /* Note:  V4L2 defines RGB24 as: RGB-8-8-8  we use
	  *        this for RGB24 packed mode
	  *
	  */
	 .description = "RGB24, le",
	 .pixelformat = V4L2_PIX_FMT_RGB24,
	 },
	{
	 .description = "YUYV (YUV 4:2:2), packed",
	 .pixelformat = V4L2_PIX_FMT_YUYV,
	 },
	{
	 .description = "UYVY, packed",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	 },
};

#define NUM_OUTPUT_FORMATS (sizeof(omap_formats)/sizeof(omap_formats[0]))

/* Allocate buffers */
static unsigned long omap_vout_alloc_buffer(u32 buf_size, u32 *phys_addr)
{
	unsigned long virt_addr, addr;
	u32 order, size;

	size = PAGE_ALIGN(buf_size);
	order = get_order(size);
	virt_addr = __get_free_pages(GFP_KERNEL | GFP_DMA, order);
	addr = virt_addr;

	if (virt_addr) {
		while (size > 0) {
			SetPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
	}
	*phys_addr = (u32) virt_to_phys((void *) virt_addr);
	return virt_addr;
}

/* Free buffers */
static void omap_vout_free_buffer(unsigned long virtaddr, u32 phys_addr,
			 u32 buf_size)
{
	unsigned long addr = virtaddr;
	u32 order, size;

	size = PAGE_ALIGN(buf_size);
	order = get_order(size);

	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages((unsigned long) virtaddr, order);
}

/* Function for allocating video buffers */
static int omap_vout_allocate_vrfb_buffers(struct omap_vout_device *vout,
		int count)
{
	int i, j;

	for (i = 0; i < count; i++) {
		if (!vout->smsshado_virt_addr[i]) {
			vout->smsshado_virt_addr[i] =
				omap_vout_alloc_buffer(vout->smsshado_size,
						&vout->smsshado_phy_addr[i]);
		}
		if (!vout->smsshado_virt_addr[i]) {
			for (j = 0; j < i; j++) {
				omap_vout_free_buffer(
						vout->smsshado_virt_addr[j],
						vout->smsshado_phy_addr[j],
						vout->smsshado_size);
				vout->smsshado_virt_addr[j] = 0;
				vout->smsshado_phy_addr[j] = 0;
			}
			count = 0;
			return -ENOMEM;
		}
		memset((void *) vout->smsshado_virt_addr[i], 0,
				vout->smsshado_size);
	}
	return 0;
}

/* Try format */
static int omap_vout_try_format(struct v4l2_pix_format *pix,
				struct v4l2_pix_format *def_pix)
{
	int ifmt, bpp = 0;

	if (pix->width > VID_MAX_WIDTH)
		pix->width = VID_MAX_WIDTH;
	if (pix->height > VID_MAX_HEIGHT)
		pix->height = VID_MAX_HEIGHT;

	if (pix->width <= VID_MIN_WIDTH)
		pix->width = def_pix->width;
	if (pix->height <= VID_MIN_HEIGHT)
		pix->height = def_pix->height;

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++) {
		if (pix->pixelformat == omap_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt == NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = omap_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_ANY;
	pix->priv = 0;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = YUYV_BPP;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB565_BPP;
		break;
	case V4L2_PIX_FMT_RGB24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB24_BPP;
		break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB32_BPP;
		break;
	}
	pix->bytesperline = pix->width * bpp;
	pix->sizeimage = pix->bytesperline * pix->height;
	return bpp;
}

/*
 * omap_vout_uservirt_to_phys: This inline function is used to convert user
 * space virtual address to physical address.
 */
static inline u32 omap_vout_uservirt_to_phys(u32 virtp)
{
	unsigned long physp = 0;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;

	vma = find_vma(mm, virtp);
	/* For kernel direct-mapped memory, take the easy way */
	if (virtp >= PAGE_OFFSET) {
		physp = virt_to_phys((void *) virtp);
	} else if ((vma) && (vma->vm_flags & VM_IO)
			&& (vma->vm_pgoff)) {
		/* this will catch, kernel-allocated,
		   mmaped-to-usermode addresses */
		physp = (vma->vm_pgoff << PAGE_SHIFT) + (virtp - vma->vm_start);
	} else {
		/* otherwise, use get_user_pages() for general userland pages */
		int res, nr_pages = 1;
		struct page *pages;
		down_read(&current->mm->mmap_sem);

		res = get_user_pages(current, current->mm, virtp, nr_pages,
				1, 0, &pages, NULL);
		up_read(&current->mm->mmap_sem);

		if (res == nr_pages) {
			physp =  __pa(page_address(&pages[0]) +
					(virtp & ~PAGE_MASK));
		} else {
			printk(KERN_WARNING VOUT_NAME
				"omap_vout_uservirt_to_phys:\
					get_user_pages failed\n");
			return 0;
		}
	}

	return physp;
}

/* This functions wakes up the application once
 * the DMA transfer to VRFB space is completed. */
static void omap_vout_vrfb_dma_tx_callback(int lch, u16 ch_status, void *data)
{
	struct vid_vrfb_dma *t = (struct vid_vrfb_dma *) data;

	t->tx_status = 1;
	wake_up_interruptible(&t->wait);
}

/* Release the VRFB context once the module exits */
static void omap_vout_release_vrfb(struct omap_vout_device *vout)
{
	int i;

	for (i = 0; i < 4; i++)
		omap_vrfb_release_ctx(&vout->vrfb_context[i]);

	if (vout->vrfb_dma_tx.req_status == DMA_CHAN_ALLOTED) {
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;
		omap_free_dma(vout->vrfb_dma_tx.dma_ch);
	}

}

/* Return true if rotation is 90 or 270 */
static inline int rotate_90_or_270(int rotation)
{
	return rotation == 1 || rotation == 3;
}

/* Return true if rotation is enabled */
static inline int rotation_enabled(int rotation)
{
	return rotation != -1;
}

/* Reverse the rotation degree if mirroring is enabled */
static inline int rotation_with_mirror(int rotation)
{
	return (rotation == 1) ? 3 :
		(rotation == 3) ? 1 :
		(rotation == 2) ? 0 : 2;
}

/* Free the V4L2 buffers */
static void omap_vout_free_buffers(struct omap_vout_device *vout)
{
	int i, numbuffers;

	/* Allocate memory for the buffes */
	numbuffers = (vout->vid) ?  video2_numbuffers : video1_numbuffers;
	vout->buffer_size = (vout->vid) ? video2_bufsize : video1_bufsize;

	for (i = 0; i < numbuffers; i++) {
		omap_vout_free_buffer(vout->buf_virt_addr[i],
			 vout->buf_phy_addr[i], vout->buffer_size);
		vout->buf_phy_addr[i] = 0;
		vout->buf_virt_addr[i] = 0;
	}
}

/* Free VRFB buffers */
static void omap_vout_free_vrfb_buffers(struct omap_vout_device *vout)
{
	int j;

	for (j = 0; j < 4; j++) {
		omap_vout_free_buffer(vout->smsshado_virt_addr[j],
				vout->smsshado_phy_addr[j],
				vout->smsshado_size);
		vout->smsshado_virt_addr[j] = 0;
		vout->smsshado_phy_addr[j] = 0;
	}
}

/* Allocate the buffers for the VRFB space.  Data is copied from V4L2
 * buffers to the VRFB buffers using the DMA engine.*/
static int omap_vout_vrfb_buffer_setup(struct omap_vout_device *vout,
			  unsigned int *count, unsigned int startindex)
{
	int i, j;

	/* Allocate the VRFB buffers only if the buffers are not
	 * allocated during init time.
	 */
	if ((rotation_enabled(vout->rotation)) &&
			!vout->vrfb_static_allocation) {
		for (i = 0; i < *count; i++) {
			if (!vout->smsshado_virt_addr[i]) {
				vout->smsshado_virt_addr[i] =
					omap_vout_alloc_buffer(
						vout->smsshado_size,
						&vout->smsshado_phy_addr[i]);
			}
			if (!vout->smsshado_virt_addr[i]) {
				if (V4L2_MEMORY_MMAP == vout->memory
						&& i >= startindex)
					break;
				for (j = 0; j < i; j++) {
					omap_vout_free_buffer(
						vout->smsshado_virt_addr[j],
						vout->smsshado_phy_addr[j],
						vout->smsshado_size);
					vout->smsshado_virt_addr[j] = 0;
					vout->smsshado_phy_addr[j] = 0;
				}
				*count = 0;
				return -ENOMEM;
			}

			memset((void *) vout->smsshado_virt_addr[i], 0,
					vout->smsshado_size);
		}
	}
	for (i = 0; i < *count; i++) {
			omap_vrfb_setup(&vout->vrfb_context[i],
				vout->smsshado_phy_addr[i],
				vout->pix.width, vout->pix.height,
				vout->dss_mode);
	}
	return 0;
}

/* Convert V4L2 rotation to DSS rotation
 * V4L2 understand 0, 90, 180, 270.
 * convert to 0, 1, 2 and 3 repsectively for DSS */
static int v4l2_rot_to_dss_rot(int v4l2_rotation, int *dss_rotation,
		int mirror)
{
	switch (v4l2_rotation) {
	case 90:
		*dss_rotation = 1;
		return 0;
	case 180:
		*dss_rotation = 2;
		return 0;
	case 270:
		*dss_rotation = 3;
		return 0;
	case 0:
		if (mirror)
			*dss_rotation = 0;
		else
			*dss_rotation = -1;
		return 0;
	default:
		return -EINVAL;
	}

}

/* Calculate the buffer offsets from which the streaming should
 * start.  This offset calculation is mainly required because of
 * the VRFB 32 pixels alignment with rotation
 */
static int omap_vout_calculate_offset(struct omap_vout_device *vout)
{
	struct v4l2_pix_format *pix = &(vout->pix);
	struct v4l2_rect *crop = &(vout->crop);
	struct v4l2_window *win = &(vout->win);
	int rotation;
	int mirroring = vout->mirror;
	int vr_ps = 1, ps = 2, temp_ps = 2;
	int offset = 0, ctop = 0, cleft = 0, line_length = 0;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_dss_device *cur_display;
	int *cropped_offset = &(vout->cropped_offset);

	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];
	/* get the display device attached to the overlay */
	if (!ovl->manager || !ovl->manager->device)
		return -1;
	cur_display = ovl->manager->device;

	if ((cur_display->type == OMAP_DISPLAY_TYPE_VENC) &&
	    ((win->w.width == crop->width)
	     && (win->w.height == crop->height)))
		vout->flicker_filter = 1;
	else
		vout->flicker_filter = 0;

	if (1 == vout->mirror)
		rotation = rotation_with_mirror(vout->rotation);
	else
		rotation = vout->rotation;

	if (V4L2_PIX_FMT_YUYV == pix->pixelformat ||
	    V4L2_PIX_FMT_UYVY == pix->pixelformat) {
		if (rotation_enabled(vout->rotation)) {
			/*
			 * ps    - Actual pixel size for YUYV/UYVY for
			 *              VRFB/Mirroring is 4 bytes
			 * vr_ps - Virtually pixel size for YUYV/UYVY is
			 *              2 bytes
			 */
			ps = 4;
			vr_ps = 2;
		} else {
			ps = 2;	/* otherwise the pixel size is 2 byte */
		}
	} else if (V4L2_PIX_FMT_RGB32 == pix->pixelformat) {
		ps = 4;
	} else if (V4L2_PIX_FMT_RGB24 == pix->pixelformat) {
		ps = 3;
	}
	vout->ps = ps;
	vout->vr_ps = vr_ps;
	if (rotation_enabled(vout->rotation)) {
		line_length = MAX_PIXELS_PER_LINE;
		ctop = (pix->height - crop->height) - crop->top;
		cleft = (pix->width - crop->width) - crop->left;
	} else {
		line_length = pix->width;
	}
	vout->line_length = line_length;
	switch (rotation) {
	case 1:
		offset = vout->vrfb_context[0].yoffset *
			vout->vrfb_context[0].bytespp;
		temp_ps = ps / vr_ps;
		if (mirroring == 0) {
			*cropped_offset = offset + line_length *
				temp_ps * cleft + crop->top * temp_ps;
		} else {
			*cropped_offset = offset + line_length * temp_ps *
				cleft + crop->top * temp_ps + (line_length *
				((crop->width / (vr_ps)) - 1) * ps);
		}
		break;
	case 2:
		offset = ((MAX_PIXELS_PER_LINE * vout->vrfb_context[0].yoffset *
			vout->vrfb_context[0].bytespp) +
			(vout->vrfb_context[0].xoffset *
			vout->vrfb_context[0].bytespp));
		if (mirroring == 0) {
			*cropped_offset = offset + (line_length * ps * ctop) +
				(cleft / vr_ps) * ps;

		} else {
			*cropped_offset = offset + (line_length * ps * ctop) +
				(cleft / vr_ps) * ps + (line_length *
				(crop->height - 1) * ps);
		}
		break;
	case 3:
		offset = MAX_PIXELS_PER_LINE * vout->vrfb_context[0].xoffset *
			vout->vrfb_context[0].bytespp;
		temp_ps = ps / vr_ps;
		if (mirroring == 0) {
			*cropped_offset = offset + line_length *
			    temp_ps * crop->left + ctop * ps;
		} else {
			*cropped_offset = offset + line_length *
				temp_ps * crop->left + ctop * ps +
				(line_length * ((crop->width / vr_ps) - 1) *
				 ps);
		}
		break;
	case 0:
		if (mirroring == 0) {
			*cropped_offset = (line_length * ps) *
				crop->top + (crop->left / vr_ps) * ps;
		} else {
			*cropped_offset = (line_length * ps) *
				crop->top + (crop->left / vr_ps) * ps +
				(line_length * (crop->height - 1) * ps);
		}
		break;
	default:
		if (mirroring == 0) {
			*cropped_offset =
			    line_length * ps * crop->top + crop->left * ps;
		} else {
			*cropped_offset = (line_length * ps * crop->top) /
				vr_ps + (crop->left * ps) / vr_ps +
				((crop->width / vr_ps) - 1) * ps;
		}
		break;
	}
	v4l2_dbg(1, debug, vout->dev->driver,
	"%s Offset:%x\n", __func__, *cropped_offset);
	return 0;
}

/* Initialize the overlay structure */
int omapvid_init(struct omap_vout_device *vout, u32 addr)
{
	int r = 0;
	struct omapvideo_info *ovid = &(vout->vid_info);
	struct omap_overlay *ovl;
	int posx, posy;
	int outw, outh, temp, rotation;
	int i;
	struct v4l2_window *win;
	struct omap_video_timings *timing;

	win = &vout->win;
	rotation = vout->rotation;
	for (i = 0; i < ovid->num_overlays; i++) {
		ovl = ovid->overlays[i];
		if (!ovl->manager || !ovl->manager->device)
			return -EINVAL;

		timing = &ovl->manager->device->panel.timings;

		outw = win->w.width;
		outh = win->w.height;
		switch (rotation) {

		case 1:
			/* Invert the height and widht for 90
			 * and 270 degree rotation
			 */
			temp = outw;
			outw = outh;
			outh = temp;
			posy = (timing->y_res - win->w.width)-
				win->w.left;
			posx = win->w.top;
			break;

		case 2:
			posx = (timing->x_res - win->w.width) -
				win->w.left;
			posy = (timing->y_res - win->w.height) -
				win->w.top;
			break;

		case 3:
			temp = outw;
			outw = outh;
			outh = temp;
			posy = win->w.left;
			posx = (timing->x_res - win->w.height)
				- win->w.top;
			break;

		default:
			posx = win->w.left;
			posy = win->w.top;
			break;
		}

		r = omapvid_setup_overlay(vout, ovl, posx, posy, outw,
				outh, addr);
		if (r)
			goto err;
	}
	return 0;
err:
	printk(KERN_WARNING VOUT_NAME "apply_changes failed\n");
	return r;
}

/*  Apply the changes set the go bit of DSS */
int omapvid_apply_changes(struct omap_vout_device *vout)
{
	struct omapvideo_info *ovid = &(vout->vid_info);
	struct omap_overlay *ovl;
	struct omap_dss_device *device;
	int i;

	for (i = 0; i < ovid->num_overlays; i++) {
		ovl = ovid->overlays[i];
		if (!ovl->manager || !ovl->manager->device)
			return -EINVAL;
		ovl->manager->apply(ovl->manager);
		if (ovl->manager->device->update) {
			device = ovl->manager->device;
			device->update(device, 0, 0,
				       device->panel.timings.x_res,
				       device->panel.timings.y_res);
		}
	}
	return 0;

}

/* Setup the overlay */
int omapvid_setup_overlay(struct omap_vout_device *vout,
		struct omap_overlay *ovl, int posx, int posy, int outw,
		int outh, u32 addr)
{
	int r = 0;
	enum omap_color_mode mode = 0;
	int rotation, mirror;
	int cropheight, cropwidth, pixheight, pixwidth;
	struct omap_overlay_info info;

	if ((ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0 &&
			(outw != vout->pix.width || outh != vout->pix.height)) {
		r = -EINVAL;
		goto err;
	}

	vout->dss_mode = video_mode_to_dss_mode(vout);

	if (mode == -EINVAL) {
		r = -EINVAL;
		goto err;
	}

	rotation = vout->rotation;
	mirror = vout->mirror;

	/* Setup the input plane parameters according to
	 * rotation value selected.
	 */
	if (rotate_90_or_270(vout->rotation)) {
		cropheight = vout->crop.width;
		cropwidth = vout->crop.height;
		pixheight = vout->pix.width;
		pixwidth = vout->pix.height;
	} else {
		cropheight = vout->crop.height;
		cropwidth = vout->crop.width;
		pixheight = vout->pix.height;
		pixwidth = vout->pix.width;
	}

	ovl->get_overlay_info(ovl, &info);
	info.paddr = addr;
	info.vaddr = NULL;
	info.width = cropwidth;
	info.height = cropheight;
	info.color_mode = vout->dss_mode;
	info.mirror = mirror;
	info.pos_x = posx;
	info.pos_y = posy;
	info.out_width = outw;
	info.out_height = outh;
	if (!rotation_enabled(vout->rotation)) {
		info.rotation = 0;
		info.rotation_type = OMAP_DSS_ROT_DMA;
		info.screen_width = pixwidth;
	} else {
		info.rotation = vout->rotation;
		info.rotation_type = OMAP_DSS_ROT_VRFB;
		info.screen_width = 2048;
	}

	v4l2_dbg(1, debug, vout->dev->driver,
	"%s info.enable=%d info.addr=%x info.width=%d\n info.height=%d \
	info.color_mode=%d info.rotation=%d info.mirror=%d\n \
	info.posx=%d info.posy=%d info.out_width = %d info.out_height=%d\n \
	info.rotation_type=%d info.screen_width=%d\n", __func__, info.enabled,
	info.paddr, info.width, info.height, info.color_mode, info.rotation,
	info.mirror, info.pos_x, info.pos_y, info.out_width, info.out_height,
	info.rotation_type, info.screen_width);

	r = ovl->set_overlay_info(ovl, &info);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_WARNING VOUT_NAME "setup_overlay failed\n");
	return r;
}

/* convert V4L2 pixel format to DSS pixel format */
static enum omap_color_mode video_mode_to_dss_mode(struct omap_vout_device
			*vout)
{
	struct v4l2_pix_format *pix = &vout->pix;

	switch (pix->pixelformat) {
	case 0:
		break;
	case V4L2_PIX_FMT_YUYV:
		return OMAP_DSS_COLOR_YUV2;

	case V4L2_PIX_FMT_UYVY:
		return OMAP_DSS_COLOR_UYVY;

	case V4L2_PIX_FMT_RGB565:
		return OMAP_DSS_COLOR_RGB16;

	case V4L2_PIX_FMT_RGB24:
		return OMAP_DSS_COLOR_RGB24P;

	case V4L2_PIX_FMT_RGB32:
		return (vout->vid == OMAP_VIDEO1) ?
			OMAP_DSS_COLOR_RGB24U : OMAP_DSS_COLOR_ARGB32;
	case V4L2_PIX_FMT_BGR32:
		return OMAP_DSS_COLOR_RGBX32;

	default:
		return -EINVAL;
	}
	return -EINVAL;
}

/* Video buffer call backs */

/* Buffer setup function is called by videobuf layer when REQBUF ioctl is
 * called. This is used to setup buffers and return size and count of
 * buffers allocated. After the call to this buffer, videobuf layer will
 * setup buffer queue depending on the size and count of buffers
 */
static int omap_vout_buffer_setup(struct videobuf_queue *q, unsigned int *count,
			  unsigned int *size)
{
	struct omap_vout_device *vout = q->priv_data;
	int startindex = 0, i, j;
	u32 phy_addr = 0, virt_addr = 0;

	if (!vout)
		return -EINVAL;

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != q->type)
		return -EINVAL;

	startindex = (vout->vid == OMAP_VIDEO1) ?
		video1_numbuffers : video2_numbuffers;
	if (V4L2_MEMORY_MMAP == vout->memory && *count < startindex)
		*count = startindex;

	if ((rotation_enabled(vout->rotation)) && *count > 4)
		*count = 4;

	/* If rotation is enabled, allocate memory for VRFB space also */
	if (rotation_enabled(vout->rotation)) {
		if (omap_vout_vrfb_buffer_setup(vout, count, startindex))
			return -ENOMEM;
	}

	if (V4L2_MEMORY_MMAP != vout->memory)
		return 0;

	/* Now allocated the V4L2 buffers */
	*size = vout->buffer_size;
	startindex = (vout->vid == OMAP_VIDEO1) ?
		video1_numbuffers : video2_numbuffers;
	for (i = startindex; i < *count; i++) {
		vout->buffer_size = *size;

		virt_addr = omap_vout_alloc_buffer(vout->buffer_size,
				&phy_addr);
		if (!virt_addr) {
			if (!rotation_enabled(vout->rotation))
				break;
			/* Free the VRFB buffers if no space for V4L2 buffers */
			for (j = i; j < *count; j++) {
				omap_vout_free_buffer(
					vout->smsshado_virt_addr[j],
					vout->smsshado_phy_addr[j],
					vout->smsshado_size);
				vout->smsshado_virt_addr[j] = 0;
				vout->smsshado_phy_addr[j] = 0;
			}
		}
		vout->buf_virt_addr[i] = virt_addr;
		vout->buf_phy_addr[i] = phy_addr;
	}
	*count = vout->buffer_allocated = i;
	return 0;
}

/* Free the V4L2 buffers additionally allocated than default
 * number of buffers and free all the VRFB buffers */
static void omap_vout_free_allbuffers(struct omap_vout_device *vout)
{
	int num_buffers = 0, i;

	num_buffers = (vout->vid == OMAP_VIDEO1) ?
		video1_numbuffers : video2_numbuffers;
	for (i = num_buffers; i < vout->buffer_allocated; i++) {
		if (vout->buf_virt_addr[i]) {
			omap_vout_free_buffer(vout->buf_virt_addr[i],
				vout->buf_phy_addr[i], vout->buffer_size);
		}
		vout->buf_virt_addr[i] = 0;
		vout->buf_phy_addr[i] = 0;
	}
	/* Free the VRFB buffers only if they are allocated
	 * during reqbufs.  Don't free if init time allocated
	 */
	if (!vout->vrfb_static_allocation) {
		for (i = 0; i < 4; i++) {
			if (vout->smsshado_virt_addr[i]) {
				omap_vout_free_buffer(
						vout->smsshado_virt_addr[i],
						vout->smsshado_phy_addr[i],
						vout->smsshado_size);
				vout->smsshado_virt_addr[i] = 0;
				vout->smsshado_phy_addr[i] = 0;
			}
		}
	}
	vout->buffer_allocated = num_buffers;
}

/* This function will be called when VIDIOC_QBUF ioctl is called.
 * It prepare buffers before give out for the display. This function
 * user space virtual address into physical address if userptr memory
 * exchange mechanism is used. If rotation is enabled, it copies entire
 * buffer into VRFB memory space before giving it to the DSS.
 */
static int omap_vout_buffer_prepare(struct videobuf_queue *q,
			    struct videobuf_buffer *vb,
			    enum v4l2_field field)
{
	struct omap_vout_device *vout = q->priv_data;
	u32 dest_frame_index = 0, src_element_index = 0;
	u32 dest_element_index = 0, src_frame_index = 0;
	u32 elem_count = 0, frame_count = 0, pixsize = 2;
	struct videobuf_dmabuf *dmabuf = NULL;
	int rotation;

	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		vb->width = vout->pix.width;
		vb->height = vout->pix.height;
		vb->size = vb->width * vb->height * vout->bpp;
		vb->field = field;
	}
	vb->state = VIDEOBUF_PREPARED;
	/* if user pointer memory mechanism is used, get the physical
	 * address of the buffer
	 */
	if (V4L2_MEMORY_USERPTR == vb->memory) {
		if (0 == vb->baddr)
			return -EINVAL;
		/* Virtual address */
		/* priv points to struct videobuf_pci_sg_memory. But we went
		 * pointer to videobuf_dmabuf, which is member of
		 * videobuf_pci_sg_memory */
		dmabuf = videobuf_to_dma(q->bufs[vb->i]);
		dmabuf->vmalloc = (void *) vb->baddr;

		/* Physical address */
		dmabuf->bus_addr =
			(dma_addr_t) omap_vout_uservirt_to_phys(vb->baddr);
	}

	if (!rotation_enabled(vout->rotation)) {
		dmabuf = videobuf_to_dma(q->bufs[vb->i]);

		vout->queued_buf_addr[vb->i] = (u8 *) dmabuf->bus_addr;
		return 0;
	}
	dmabuf = videobuf_to_dma(q->bufs[vb->i]);
	/* If rotation is enabled, copy input buffer into VRFB
	 * memory space using DMA. We are copying input buffer
	 * into VRFB memory space of desired angle and DSS will
	 * read image VRFB memory for 0 degree angle
	 */
	pixsize = vout->bpp * vout->vrfb_bpp;
	/*
	 * DMA transfer in double index mode
	 */

	/* Frame index */
	dest_frame_index = ((MAX_PIXELS_PER_LINE * pixsize) -
			(vout->pix.width * vout->bpp)) + 1;

	/* Source and destination parameters */
	src_element_index = 0;
	src_frame_index = 0;
	dest_element_index = 1;
	/* Number of elements per frame */
	elem_count = vout->pix.width * vout->bpp;
	frame_count = vout->pix.height;
	vout->vrfb_dma_tx.tx_status = 0;
	omap_set_dma_transfer_params(vout->vrfb_dma_tx.dma_ch,
			OMAP_DMA_DATA_TYPE_S32, (elem_count / 4), frame_count,
			OMAP_DMA_SYNC_ELEMENT, vout->vrfb_dma_tx.dev_id, 0x0);
	/* src_port required only for OMAP1 */
	omap_set_dma_src_params(vout->vrfb_dma_tx.dma_ch, 0,
			OMAP_DMA_AMODE_POST_INC, dmabuf->bus_addr,
			src_element_index, src_frame_index);
	/*set dma source burst mode for VRFB */
	omap_set_dma_src_burst_mode(vout->vrfb_dma_tx.dma_ch,
			OMAP_DMA_DATA_BURST_16);
	if (vout->mirror)
		rotation = rotation_with_mirror(vout->rotation);
	else
		rotation = vout->rotation;

	/* dest_port required only for OMAP1 */
	omap_set_dma_dest_params(vout->vrfb_dma_tx.dma_ch, 0,
			OMAP_DMA_AMODE_DOUBLE_IDX,
			vout->vrfb_context[vb->i].paddr[0],
			dest_element_index, dest_frame_index);
	/*set dma dest burst mode for VRFB */
	omap_set_dma_dest_burst_mode(vout->vrfb_dma_tx.dma_ch,
			OMAP_DMA_DATA_BURST_16);
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

	omap_start_dma(vout->vrfb_dma_tx.dma_ch);
	interruptible_sleep_on_timeout(&vout->vrfb_dma_tx.wait,
			VRFB_TX_TIMEOUT);

	if (vout->vrfb_dma_tx.tx_status == 0) {
		omap_stop_dma(vout->vrfb_dma_tx.dma_ch);
		return -EINVAL;
	}
	/* Store buffers physical address into an array. Addresses
	 * from this array will be used to configure DSS */
	vout->queued_buf_addr[vb->i] = (u8 *)
		vout->vrfb_context[vb->i].paddr[rotation];
	return 0;
}

/* Buffer queue funtion will be called from the videobuf layer when _QBUF
 * ioctl is called. It is used to enqueue buffer, which is ready to be
 * displayed. */
static void omap_vout_buffer_queue(struct videobuf_queue *q,
			  struct videobuf_buffer *vb)
{
	struct omap_vout_device *vout = q->priv_data;

	/* Driver is also maintainig a queue. So enqueue buffer in the driver
	 * queue */
	list_add_tail(&vb->queue, &vout->dma_queue);

	vb->state = VIDEOBUF_PREPARED;
}

/* Buffer release function is called from videobuf layer to release buffer
 * which are already allocated */
static void omap_vout_buffer_release(struct videobuf_queue *q,
			    struct videobuf_buffer *vb)
{
	struct omap_vout_device *vout = q->priv_data;

	vb->state = VIDEOBUF_NEEDS_INIT;

	if (V4L2_MEMORY_MMAP != vout->memory)
		return;
}

/*
 *  File operations
 */
static void omap_vout_vm_open(struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = vma->vm_private_data;

	v4l2_dbg(1, debug, vout->dev->driver,
	"vm_open [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_count++;
}

static void omap_vout_vm_close(struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = vma->vm_private_data;

	v4l2_dbg(1, debug, vout->dev->driver,
	"vm_close [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_count--;
}

static struct vm_operations_struct omap_vout_vm_ops = {
	.open = omap_vout_vm_open,
	.close = omap_vout_vm_close,
};

static int omap_vout_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap_vout_device *vout = file->private_data;
	struct videobuf_queue *q = &vout->vbq;
	unsigned long size = (vma->vm_end - vma->vm_start);
	unsigned long start = vma->vm_start;
	int i;
	void *pos;
	struct videobuf_dmabuf *dmabuf = NULL;

	v4l2_dbg(1, debug, vout->dev->driver,
		" %s pgoff=0x%lx, start=0x%lx, end=0x%lx\n", __func__,
		vma->vm_pgoff, vma->vm_start, vma->vm_end);

	/* look for the buffer to map */
	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (NULL == q->bufs[i])
			continue;
		if (V4L2_MEMORY_MMAP != q->bufs[i]->memory)
			continue;
		if (q->bufs[i]->boff == (vma->vm_pgoff << PAGE_SHIFT))
			break;
	}

	if (VIDEO_MAX_FRAME == i) {
		v4l2_dbg(1, debug, vout->dev->driver,
		"offset invalid [offset=0x%lx]\n",
			(vma->vm_pgoff << PAGE_SHIFT));
		return -EINVAL;
	}
	q->bufs[i]->baddr = vma->vm_start;

	vma->vm_flags |= VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_ops = &omap_vout_vm_ops;
	vma->vm_private_data = (void *) vout;
	dmabuf = videobuf_to_dma(q->bufs[i]);
	pos = dmabuf->vmalloc;
	vma->vm_pgoff = virt_to_phys((void *)pos) >> PAGE_SHIFT;
	while (size > 0) {
		unsigned long pfn;
		pfn = virt_to_phys((void *) pos) >> PAGE_SHIFT;
		if (remap_pfn_range(vma, start, pfn, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	vout->mmap_count++;
	v4l2_dbg(1, debug, vout->dev->driver, "Exiting %s\n", __func__);
	return 0;
}

static int omap_vout_release(struct file *file)
{

	struct omap_vout_device *vout = file->private_data;
	struct videobuf_queue *q;
	unsigned int t;
	struct omapvideo_info *ovid;
	unsigned int r;

	v4l2_dbg(1, debug, vout->dev->driver, "Entering %s\n", __func__);
	ovid = &(vout->vid_info);

	if (!vout)
		return 0;
	q = &vout->vbq;

	/* Disable all the overlay managers connected with this interface */
	for (t = 0; t < ovid->num_overlays; t++) {
			struct omap_overlay *ovl = ovid->overlays[t];
			if (ovl->manager && ovl->manager->device) {
				struct omap_overlay_info info;
				ovl->get_overlay_info(ovl, &info);
				info.enabled = 0;
				ovl->set_overlay_info(ovl, &info);
			}

		}
	/* Turn off the pipeline */
	r = omapvid_apply_changes(vout);
	if (r)
		printk(KERN_WARNING VOUT_NAME "Unable to apply changes\n");

	/* Free all buffers */
	omap_vout_free_allbuffers(vout);
	videobuf_mmap_free(q);

	/* Even if apply changes fails we should continue
	   freeing allocated memeory */
	if (vout->streaming) {
		omap_dispc_unregister_isr(omap_vout_isr, vout,
					  OMAP_VOUT_IRQ_MASK);
		vout->streaming = 0;

		videobuf_streamoff(q);
		videobuf_queue_cancel(q);

	}

	if (vout->mmap_count != 0)
		vout->mmap_count = 0;

	vout->opened -= 1;
	file->private_data = NULL;

	if (vout->buffer_allocated)
		videobuf_mmap_free(q);
	v4l2_dbg(1, debug, vout->dev->driver, "Exiting %s\n", __func__);
	return r;
}

static int omap_vout_open(struct file *file)
{
	struct omap_vout_device *vout = NULL;
	struct videobuf_queue *q;

	vout = video_drvdata(file);
	v4l2_dbg(1, debug, vout->dev->driver, "Entering %s\n", __func__);

	if (vout == NULL)
		return -ENODEV;

	/* for now, we only support single open */
	if (vout->opened)
		return -EBUSY;

	vout->opened += 1;

	file->private_data = vout;
	vout->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	q = &vout->vbq;
	video_vbq_ops.buf_setup = omap_vout_buffer_setup;
	video_vbq_ops.buf_prepare = omap_vout_buffer_prepare;
	video_vbq_ops.buf_release = omap_vout_buffer_release;
	video_vbq_ops.buf_queue = omap_vout_buffer_queue;
	spin_lock_init(&vout->vbq_lock);

	videobuf_queue_sg_init(q, &video_vbq_ops, NULL, &vout->vbq_lock,
			       vout->type, V4L2_FIELD_NONE, sizeof
			       (struct videobuf_buffer), vout);
	v4l2_dbg(1, debug, vout->dev->driver, "Exiting %s\n", __func__);
	return 0;
}

/* V4L2 ioctls */
static int vidioc_querycap(struct file *file, void *fh,
		struct v4l2_capability *cap)
{
	struct omap_vout_device *vout = fh;

	strlcpy(cap->driver, VOUT_NAME,
		sizeof(cap->driver));
	strlcpy(cap->card, vout->vfd->name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
	return 0;
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	fmt->flags = omap_formats[index].flags;
	strlcpy(fmt->description, omap_formats[index].description,
			sizeof(fmt->description));
	fmt->pixelformat = omap_formats[index].pixelformat;
	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;

	struct v4l2_pix_format *pix = &f->fmt.pix;
	*pix = vout->pix;
	return 0;

}

static int vidioc_try_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_video_timings *timing;

	if (vout->streaming)
		return -EBUSY;

	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];

	if (!ovl->manager || !ovl->manager->device)
		return -EINVAL;
	/* get the display device attached to the overlay */
	timing = &ovl->manager->device->panel.timings;

	vout->fbuf.fmt.height = timing->y_res;
	vout->fbuf.fmt.width = timing->x_res;

	omap_vout_try_format(&f->fmt.pix, &vout->fbuf.fmt);
	return 0;
}

static int vidioc_s_fmt_vid_out(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	int bpp;
	int r;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_video_timings *timing;

	if (vout->streaming)
		return -EBUSY;

	mutex_lock(&vout->lock);

	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];

	/* get the display device attached to the overlay */
	if (!ovl->manager || !ovl->manager->device) {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	timing = &ovl->manager->device->panel.timings;

	/* We dont support RGB24-packed mode if vrfb rotation
	 * is enabled*/
	if ((rotation_enabled(vout->rotation)) &&
			f->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}

	/* get the framebuffer parameters */

	if (rotate_90_or_270(vout->rotation)) {
		vout->fbuf.fmt.height = timing->x_res;
		vout->fbuf.fmt.width = timing->y_res;
	} else {
		vout->fbuf.fmt.height = timing->y_res;
		vout->fbuf.fmt.width = timing->x_res;
	}

	/* change to samller size is OK */

	bpp = omap_vout_try_format(&f->fmt.pix, &vout->fbuf.fmt);
	f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height * bpp;

	/* try & set the new output format */
	vout->bpp = bpp;
	vout->pix = f->fmt.pix;
	vout->vrfb_bpp = 1;

	/* If YUYV then vrfb bpp is 2, for  others its 1 */
	if (V4L2_PIX_FMT_YUYV == vout->pix.pixelformat ||
		V4L2_PIX_FMT_UYVY == vout->pix.pixelformat)
		vout->vrfb_bpp = 2;

	/* set default crop and win */
	omap_vout_new_format(&vout->pix, &vout->fbuf, &vout->crop, &vout->win);

	/* Save the changes in the overlay strcuture */
	r = omapvid_init(vout, 0);
	if (r) {
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	mutex_unlock(&vout->lock);
	return 0;
}

static int vidioc_try_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	int err = -EINVAL;
	struct omap_vout_device *vout = fh;
	struct v4l2_window *win = &f->fmt.win;

	err = omap_vout_try_window(&vout->fbuf, win);

	if (err)
		return err;

	if (vout->vid == OMAP_VIDEO1)
		win->global_alpha = 255;
	return 0;
}

static int vidioc_s_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	int err = -EINVAL;
	struct v4l2_window *win = &f->fmt.win;

	mutex_lock(&vout->lock);
	err = omap_vout_new_window(&vout->crop, &vout->win, &vout->fbuf, win);
	if (err) {
		mutex_unlock(&vout->lock);
		return err;
	}
	if (vout->vid == OMAP_VIDEO1)
		vout->win.global_alpha = 255;
	else
		vout->win.global_alpha = f->fmt.win.global_alpha;

	vout->win.chromakey = f->fmt.win.chromakey;
	mutex_unlock(&vout->lock);
	return 0;
}

static int vidioc_enum_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	fmt->index = index;
	fmt->type = type;
	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	fmt->flags = omap_formats[index].flags;
	strlcpy(fmt->description, omap_formats[index].description,
		sizeof(fmt->description));
	fmt->pixelformat = omap_formats[index].pixelformat;
	return 0;
}

static int vidioc_g_fmt_vid_overlay(struct file *file, void *fh,
			struct v4l2_format *f)
{
	struct omap_vout_device *vout = fh;
	struct omap_overlay *ovl;
	struct omapvideo_info *ovid;
	struct v4l2_window *win = &f->fmt.win;
	struct omap_overlay_manager_info info;

	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];

	win->w = vout->win.w;
	win->field = vout->win.field;
	win->global_alpha = vout->win.global_alpha;

	if (!ovl->manager || !ovl->manager->get_manager_info)
		return -EINVAL;

	ovl->manager->get_manager_info(ovl->manager, &info);
	win->chromakey = info.trans_key;

	return 0;
}

static int vidioc_cropcap(struct file *file, void *fh,
			struct v4l2_cropcap *cropcap)
{
	struct omap_vout_device *vout = fh;
	enum v4l2_buf_type type = cropcap->type;
	struct v4l2_pix_format *pix = &vout->pix;

	cropcap->type = type;
	if (type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* Width and height are always even */
	cropcap->bounds.width = pix->width & ~1;
	cropcap->bounds.height = pix->height & ~1;

	omap_vout_default_crop(&vout->pix, &vout->fbuf, &cropcap->defrect);
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

static int vidioc_g_crop(struct file *file, void *fh,
	struct v4l2_crop *crop)
{
	struct omap_vout_device *vout = fh;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	crop->c = vout->crop;
	return 0;
}

static int vidioc_s_crop(struct file *file, void *fh,
			struct v4l2_crop *crop)
{
	struct omap_vout_device *vout = fh;
	int err = -EINVAL;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_video_timings *timing;

	if (vout->streaming)
		return -EBUSY;

	mutex_lock(&vout->lock);
	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];

	if (!ovl->manager || !ovl->manager->device) {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	/* get the display device attached to the overlay */
	timing = &ovl->manager->device->panel.timings;

	if (rotate_90_or_270(vout->rotation)) {
		vout->fbuf.fmt.height = timing->x_res;
		vout->fbuf.fmt.width = timing->y_res;
	} else {
		vout->fbuf.fmt.height = timing->y_res;
		vout->fbuf.fmt.width = timing->x_res;
	}

	if (crop->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		err = omap_vout_new_crop(&vout->pix, &vout->crop, &vout->win,
			&vout->fbuf, &crop->c);
		mutex_unlock(&vout->lock);
		return err;
	} else {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
}

static int vidioc_queryctrl(struct file *file, void *fh,
		struct v4l2_queryctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		v4l2_ctrl_query_fill(ctrl, 0, 270, 90, 0);
		break;
	case V4L2_CID_BG_COLOR:
		v4l2_ctrl_query_fill(ctrl, 0, 0xFFFFFF, 1, 0);
		break;
	case V4L2_CID_VFLIP:
		v4l2_ctrl_query_fill(ctrl, 0, 1, 1, 0);
	default:
		ctrl->name[0] = '\0';
		return -EINVAL;

	}
	return 0;
}

static int vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
	struct omap_vout_device *vout = fh;

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ctrl->value = vout->control[0].value;
		return 0;
	case V4L2_CID_BG_COLOR:
	{
		struct omapvideo_info *ovid;
		struct omap_overlay *ovl;
		struct omap_overlay_manager_info info;

		ovid = &(vout->vid_info);
		ovl = ovid->overlays[0];

		if (!ovl->manager || !ovl->manager->get_manager_info)
			return -EINVAL;

		ovl->manager->get_manager_info(ovl->manager, &info);
		ctrl->value = info.default_color;
		return 0;
	}

	case V4L2_CID_VFLIP:
		ctrl->value = vout->control[2].value;
		return 0;
	default:
		return -EINVAL;
	}
}

static int vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct omap_vout_device *vout = fh;

	switch (a->id) {
	case V4L2_CID_ROTATE:
	{
		int rotation = a->value;

		mutex_lock(&vout->lock);
		if ((v4l2_rot_to_dss_rot(rotation, &vout->rotation,
				vout->mirror))) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}
		if (vout->pix.pixelformat == V4L2_PIX_FMT_RGB24 &&
				rotation_enabled(vout->rotation)) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}
		vout->control[0].value = rotation;
		mutex_unlock(&vout->lock);
		return 0;
	}
	case V4L2_CID_BG_COLOR:
	{
		unsigned int  color = a->value;
		struct omapvideo_info *ovid;
		struct omap_overlay *ovl;
		struct omap_overlay_manager_info info;
		ovid = &(vout->vid_info);
		ovl = ovid->overlays[0];

		mutex_lock(&vout->lock);
		if (!ovl->manager || !ovl->manager->get_manager_info) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}
		ovl->manager->get_manager_info(ovl->manager, &info);
		info.default_color = color;
		ovl->manager->set_manager_info(ovl->manager, &info);

		vout->control[1].value = color;
		mutex_unlock(&vout->lock);
		return 0;
	}
	case V4L2_CID_VFLIP:
	{
		unsigned int  mirror = a->value;
		struct omapvideo_info *ovid;
		struct omap_overlay *ovl;
		ovid = &(vout->vid_info);
		ovl = ovid->overlays[0];

		mutex_lock(&vout->lock);

		if (mirror  && vout->pix.pixelformat == V4L2_PIX_FMT_RGB24) {
			mutex_unlock(&vout->lock);
			return -EINVAL;
		}
		/* Use VRFB if mirroring is enabled */
		if (mirror && !rotation_enabled(vout->rotation))
			vout->rotation = 0;
		/* Disabled VRFB if mirroring is disabled */
		if (!mirror && vout->rotation == 0)
			vout->rotation = -1;

		vout->mirror = mirror;
		vout->control[2].value = mirror;
		mutex_unlock(&vout->lock);
		return 0;
	}

	default:
		return -EINVAL;
	}

}

static int vidioc_reqbufs(struct file *file, void *fh,
			struct v4l2_requestbuffers *req)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &(vout->vbq);
	unsigned int i, num_buffers = 0;
	int ret = 0;
	struct videobuf_dmabuf *dmabuf = NULL;

	if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) || (req->count < 0))
		return -EINVAL;
	/* if memory is not mmp or userptr
	   return error */
	if ((V4L2_MEMORY_MMAP != req->memory) &&
		(V4L2_MEMORY_USERPTR != req->memory))
		return -EINVAL;

	mutex_lock(&vout->lock);
	/* Cannot be requested when streaming is on */
	if (vout->streaming) {
		mutex_unlock(&vout->lock);
		return -EBUSY;
	}

	/* If buffers are already allocated free them */
	if (q->bufs[0] && (V4L2_MEMORY_MMAP == q->bufs[0]->memory)) {
		if (vout->mmap_count) {
			mutex_unlock(&vout->lock);
			return -EBUSY;
		}
		num_buffers = (vout->vid == OMAP_VIDEO1) ?
			video1_numbuffers : video2_numbuffers;
		for (i = num_buffers; i < vout->buffer_allocated; i++) {
			dmabuf = videobuf_to_dma(q->bufs[i]);
			omap_vout_free_buffer((u32)dmabuf->vmalloc,
				dmabuf->bus_addr, vout->buffer_size);
			vout->buf_virt_addr[i] = 0;
			vout->buf_phy_addr[i] = 0;
		}
		vout->buffer_allocated = num_buffers;
		videobuf_mmap_free(q);
	} else if (q->bufs[0] && (V4L2_MEMORY_USERPTR == q->bufs[0]->memory)) {
		if (vout->buffer_allocated) {
			videobuf_mmap_free(q);
			for (i = 0; i < vout->buffer_allocated; i++) {
				kfree(q->bufs[i]);
				q->bufs[i] = NULL;
			}
			vout->buffer_allocated = 0;
		}
	}

	/*store the memory type in data structure */
	vout->memory = req->memory;

	INIT_LIST_HEAD(&vout->dma_queue);

	/* call videobuf_reqbufs api */
	ret = videobuf_reqbufs(q, req);
	if (ret < 0) {
		mutex_unlock(&vout->lock);
		return ret;
	}

	vout->buffer_allocated = req->count;
	for (i = 0; i < req->count; i++) {
		dmabuf = videobuf_to_dma(q->bufs[i]);
		dmabuf->vmalloc = (void *) vout->buf_virt_addr[i];
		dmabuf->bus_addr = (dma_addr_t) vout->buf_phy_addr[i];
		dmabuf->sglen = 1;
	}
	mutex_unlock(&vout->lock);
	return 0;
}

static int vidioc_querybuf(struct file *file, void *fh,
			struct v4l2_buffer *b)
{
	struct omap_vout_device *vout = fh;

	return videobuf_querybuf(&(vout->vbq), b);
}

static int vidioc_qbuf(struct file *file, void *fh,
			struct v4l2_buffer *buffer)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	int ret = 0;

	if ((V4L2_BUF_TYPE_VIDEO_OUTPUT != buffer->type) ||
			(buffer->index >= vout->buffer_allocated) ||
			(q->bufs[buffer->index]->memory != buffer->memory)) {
		return -EINVAL;
	}
	if (V4L2_MEMORY_USERPTR == buffer->memory) {
		if ((buffer->length < vout->pix.sizeimage) ||
			(0 == buffer->m.userptr)) {
			return -EINVAL;
		}
	}

	if ((rotation_enabled(vout->rotation)) &&
			vout->vrfb_dma_tx.req_status == DMA_CHAN_NOT_ALLOTED) {
		printk(KERN_WARNING VOUT_NAME
				"DMA Channel not allocated for Rotation\n");
		return -EINVAL;
	}

	ret = videobuf_qbuf(q, buffer);
	return ret;
}

static int vidioc_dqbuf(struct file *file, void *fh,
			struct v4l2_buffer *b)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	int ret = 0;

	if (!vout->streaming)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK)
		/* Call videobuf_dqbuf for non blocking mode */
		ret = videobuf_dqbuf(q, (struct v4l2_buffer *)b, 1);
	else
		/* Call videobuf_dqbuf for  blocking mode */
		ret = videobuf_dqbuf(q, (struct v4l2_buffer *)b, 0);
	return ret;
}

static int vidioc_streamon(struct file *file, void *fh,
			enum v4l2_buf_type i)
{
	struct omap_vout_device *vout = fh;
	struct videobuf_queue *q = &vout->vbq;
	u32 addr = 0;
	int r = 0;
	int t;
	struct omapvideo_info *ovid = &(vout->vid_info);

	mutex_lock(&vout->lock);

	if (vout->streaming) {
		mutex_unlock(&vout->lock);
		return -EBUSY;
	}

	r = videobuf_streamon(q);
	if (r < 0) {
		mutex_unlock(&vout->lock);
		return r;
	}

	if (list_empty(&vout->dma_queue)) {
		mutex_unlock(&vout->lock);
		return -EIO;
	}
	/* Get the next frame from the buffer queue */
	vout->next_frm = vout->cur_frm = list_entry(vout->dma_queue.next,
				struct videobuf_buffer, queue);
	/* Remove buffer from the buffer queue */
	list_del(&vout->cur_frm->queue);
	/* Mark state of the current frame to active */
	vout->cur_frm->state = VIDEOBUF_ACTIVE;
	/* Initialize field_id and started member */
	vout->field_id = 0;

	/* set flag here. Next QBUF will start DMA */
	vout->streaming = 1;

	vout->first_int = 1;

	if (omap_vout_calculate_offset(vout)) {
		mutex_unlock(&vout->lock);
		return -EINVAL;
	}
	addr = (unsigned long) vout->queued_buf_addr[vout->cur_frm->i]
	+ vout->cropped_offset;

	omap_dispc_register_isr(omap_vout_isr, vout, OMAP_VOUT_IRQ_MASK);

	for (t = 0; t < ovid->num_overlays; t++) {
		struct omap_overlay *ovl = ovid->overlays[t];
		if (ovl->manager && ovl->manager->device) {
			struct omap_overlay_info info;
			ovl->get_overlay_info(ovl, &info);
			info.enabled = 1;
			info.paddr = addr;
			if (ovl->set_overlay_info(ovl, &info)) {
				mutex_unlock(&vout->lock);
				return -EINVAL;
			}
		}
	}

	/* First save the configuration in ovelray structure */
	r = omapvid_init(vout, addr);
	if (r)
		printk(KERN_ERR VOUT_NAME "failed to set overlay info\n");
	/* Enable the pipeline and set the Go bit */
	r = omapvid_apply_changes(vout);
	if (r)
		printk(KERN_ERR VOUT_NAME "failed to change mode\n");

	mutex_unlock(&vout->lock);
	return r;
}

static int vidioc_streamoff(struct file *file, void *fh,
			enum v4l2_buf_type i)
{
	struct omap_vout_device *vout = fh;
	int t, r = 0;
	struct omapvideo_info *ovid = &(vout->vid_info);

	if (!vout->streaming)
		return -EINVAL;
	if (vout->streaming) {
		vout->streaming = 0;

		omap_dispc_unregister_isr(omap_vout_isr, vout,
					  OMAP_VOUT_IRQ_MASK);

		for (t = 0; t < ovid->num_overlays; t++) {
			struct omap_overlay *ovl = ovid->overlays[t];
			if (ovl->manager && ovl->manager->device) {
				struct omap_overlay_info info;

				ovl->get_overlay_info(ovl, &info);
				info.enabled = 0;
				return ovl->set_overlay_info(ovl, &info);
			}
		}

		/* Turn of the pipeline */
		r = omapvid_apply_changes(vout);
		if (r) {
			printk(KERN_ERR VOUT_NAME "failed to change mode\n");
			return r;
		}
		videobuf_streamoff(&vout->vbq);
		videobuf_queue_cancel(&vout->vbq);
	}
	return 0;
}

static int vidioc_s_fbuf(struct file *file, void *fh,
		struct v4l2_framebuffer *a)
{
	struct omap_vout_device *vout = fh;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_overlay_manager_info info;
	enum omap_dss_trans_key_type key_type = OMAP_DSS_COLOR_KEY_GFX_DST;
	int enable = 0;

	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];

	/* OMAP DSS doesn't support Source and Destination color
	   key together */
	if ((a->flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY) &&
			(a->flags & V4L2_FBUF_FLAG_CHROMAKEY))
		return -EINVAL;
	/* OMAP DSS Doesn't support the Destination color key
	   and alpha blending together */
	if (a->flags & V4L2_FBUF_FLAG_CHROMAKEY &&
			(a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA))
		return -EINVAL;

	if ((a->flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY)) {
		vout->fbuf.flags |= V4L2_FBUF_FLAG_SRC_CHROMAKEY;
		key_type =  OMAP_DSS_COLOR_KEY_VID_SRC;
	} else
		vout->fbuf.flags &= ~V4L2_FBUF_FLAG_SRC_CHROMAKEY;

	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY)) {
		vout->fbuf.flags |= V4L2_FBUF_FLAG_CHROMAKEY;
		key_type =  OMAP_DSS_COLOR_KEY_GFX_DST;
	} else
		vout->fbuf.flags &=  ~V4L2_FBUF_FLAG_CHROMAKEY;

	if (a->flags & (V4L2_FBUF_FLAG_CHROMAKEY |
				V4L2_FBUF_FLAG_SRC_CHROMAKEY))
		enable = 1;
	else
		enable = 0;

	if (ovl->manager && ovl->manager->get_manager_info &&
	    ovl->manager->set_manager_info) {
		ovl->manager->get_manager_info(ovl->manager, &info);
		info.trans_enabled = enable;
		info.trans_key = vout->win.chromakey;
		ovl->manager->set_manager_info(ovl->manager, &info);
	}

	if (a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA) {
		vout->fbuf.flags |= V4L2_FBUF_FLAG_LOCAL_ALPHA;
		enable = 1;
	} else if (!(a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA)) {
		vout->fbuf.flags &= ~V4L2_FBUF_FLAG_LOCAL_ALPHA;
		enable = 0;
	}
	if (ovl->manager && ovl->manager->get_manager_info &&
	    ovl->manager->set_manager_info) {
		ovl->manager->get_manager_info(ovl->manager, &info);
		info.alpha_enabled = enable;
		ovl->manager->set_manager_info(ovl->manager, &info);
	}

	return 0;
}

static int vidioc_g_fbuf(struct file *file, void *fh,
		struct v4l2_framebuffer *a)
{
	struct omap_vout_device *vout = fh;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_overlay_manager_info info;

	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];

	a->flags = 0x0;

	a->capability = V4L2_FBUF_CAP_LOCAL_ALPHA | V4L2_FBUF_CAP_CHROMAKEY
		| V4L2_FBUF_CAP_SRC_CHROMAKEY;

	if (ovl->manager && ovl->manager->get_manager_info) {
		ovl->manager->get_manager_info(ovl->manager, &info);
		if (info.trans_enabled) {
			if (info.trans_key_type == OMAP_DSS_COLOR_KEY_VID_SRC)
				a->flags |= V4L2_FBUF_FLAG_SRC_CHROMAKEY;
			if (info.trans_key_type == OMAP_DSS_COLOR_KEY_GFX_DST)
				a->flags |= V4L2_FBUF_FLAG_CHROMAKEY;

		}
		if (info.alpha_enabled)
			a->flags |= V4L2_FBUF_FLAG_LOCAL_ALPHA;
	}

	return 0;
}

static const struct v4l2_ioctl_ops vout_ioctl_ops = {
	.vidioc_querycap      			= vidioc_querycap,
	.vidioc_enum_fmt_vid_out 		= vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out			= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out			= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out			= vidioc_s_fmt_vid_out,
	.vidioc_queryctrl    			= vidioc_queryctrl,
	.vidioc_g_ctrl       			= vidioc_g_ctrl,
	.vidioc_s_fbuf				= vidioc_s_fbuf,
	.vidioc_g_fbuf				= vidioc_g_fbuf,
	.vidioc_s_ctrl       			= vidioc_s_ctrl,
	.vidioc_try_fmt_vid_overlay 		= vidioc_try_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay		= vidioc_s_fmt_vid_overlay,
	.vidioc_enum_fmt_vid_overlay		= vidioc_enum_fmt_vid_overlay,
	.vidioc_g_fmt_vid_overlay		= vidioc_g_fmt_vid_overlay,
	.vidioc_cropcap				= vidioc_cropcap,
	.vidioc_g_crop				= vidioc_g_crop,
	.vidioc_s_crop				= vidioc_s_crop,
	.vidioc_reqbufs				= vidioc_reqbufs,
	.vidioc_querybuf			= vidioc_querybuf,
	.vidioc_qbuf				= vidioc_qbuf,
	.vidioc_dqbuf				= vidioc_dqbuf,
	.vidioc_streamon			= vidioc_streamon,
	.vidioc_streamoff			= vidioc_streamoff,
};

static const struct v4l2_file_operations omap_vout_fops = {
	.owner 		= THIS_MODULE,
	.ioctl 		= video_ioctl2,
	.mmap 		= omap_vout_mmap,
	.open 		= omap_vout_open,
	.release 	= omap_vout_release,
};

/* Init functions used during driver intitalization */
/* Initial setup of video_data */
static int __init omap_vout_setup_video_data(struct omap_vout_device *vout)
{
	struct v4l2_pix_format *pix;
	struct video_device *vfd;
	struct v4l2_control *control;
	struct omap_dss_device *display =
		vout->vid_info.overlays[0]->manager->device;

	/* set the default pix */
	pix = &vout->pix;

	/* Set the default picture of QVGA  */
	pix->width = QQVGA_WIDTH;
	pix->height = QQVGA_HEIGHT;

	/* Default pixel format is RGB 5-6-5 */
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_ANY;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_JPEG;

	vout->bpp = RGB565_BPP;
	vout->fbuf.fmt.width  =  display->panel.timings.x_res;
	vout->fbuf.fmt.height =  display->panel.timings.y_res;

	/* Set the data structures for the overlay parameters*/
	vout->win.global_alpha = 255;
	vout->fbuf.flags = 0;
	vout->fbuf.capability = V4L2_FBUF_CAP_LOCAL_ALPHA |
		V4L2_FBUF_CAP_SRC_CHROMAKEY | V4L2_FBUF_CAP_CHROMAKEY;
	vout->win.chromakey = 0;

	omap_vout_new_format(pix, &vout->fbuf, &vout->crop, &vout->win);

	/*Disable the rotation. */
	control = vout->control;

	control[0].id = V4L2_CID_ROTATE;
	control[0].value = 0;
	vout->rotation = -1;
	vout->mirror = 0;
	vout->vrfb_bpp = 2;

	control[1].id = V4L2_CID_BG_COLOR;
	control[1].value = 0;

	/* initialize the video_device struct */
	vfd = vout->vfd = video_device_alloc();

	if (!vfd) {
		printk(KERN_ERR VOUT_NAME ": could not allocate\
				video device struct\n");
		return -ENOMEM;
	}
	vfd->release = video_device_release;
	vfd->ioctl_ops = &vout_ioctl_ops;

	strlcpy(vfd->name, VOUT_NAME, sizeof(vfd->name));
	vfd->vfl_type = VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;

	/* need to register for a VID_HARDWARE_* ID in videodev.h */
	vfd->fops = &omap_vout_fops;
	mutex_init(&vout->lock);

	vfd->minor = -1;
	return 0;

}

/* Setup video buffers */
static int __init omap_vout_setup_video_bufs(struct platform_device *pdev,
		int vid_num)
{
	struct omap2video_device *vid_dev = platform_get_drvdata(pdev);
	struct omap_vout_device *vout;
	int i, j, r = 0;
	int image_width, image_height;
	unsigned numbuffers;
	struct video_device *vfd;
	int static_vrfb_allocation = 0, vrfb_num_bufs = 4;

	vout = vid_dev->vouts[vid_num];
	vfd = vout->vfd;

	numbuffers = (vid_num == 0) ? video1_numbuffers : video2_numbuffers;
	vout->buffer_size = (vid_num == 0) ? video1_bufsize : video2_bufsize;
	printk(KERN_INFO VOUT_NAME "Buffer Size = %d\n", vout->buffer_size);
	for (i = 0; i < numbuffers; i++) {
		vout->buf_virt_addr[i] =
			omap_vout_alloc_buffer(vout->buffer_size,
					(u32 *) &vout->buf_phy_addr[i]);
		if (!vout->buf_virt_addr[i]) {
			numbuffers = i;
			r = -ENOMEM;
			goto free_buffers;
		}
	}

	for (i = 0; i < 4; i++) {

		if (omap_vrfb_request_ctx(&vout->vrfb_context[i])) {
			printk(KERN_INFO VOUT_NAME ": VRFB Region allocation \
					for rotation failed\n");
			r = -ENOMEM;
			break;
		}
	}
	if (r == -ENOMEM) {
		for (j = 0; j < i; j++)
			omap_vrfb_release_ctx(&vout->vrfb_context[j]);

		goto free_buffers;
	}

	vout->cropped_offset = 0;

	/* Calculate VRFB memory size */
	/* allocate for worst case size */
	image_width = VID_MAX_WIDTH / TILE_SIZE;
	if (VID_MAX_WIDTH % TILE_SIZE)
		image_width++;

	image_width = image_width * TILE_SIZE;
	image_height = VID_MAX_HEIGHT / TILE_SIZE;

	if (VID_MAX_HEIGHT % TILE_SIZE)
		image_height++;

	image_height = image_height * TILE_SIZE;
	vout->smsshado_size = PAGE_ALIGN(image_width * image_height * 2 * 2);

	/*
	 * Request and Initialize DMA, for DMA based VRFB transfer
	 */
	vout->vrfb_dma_tx.dev_id = OMAP_DMA_NO_DEVICE;
	vout->vrfb_dma_tx.dma_ch = -1;
	vout->vrfb_dma_tx.req_status = DMA_CHAN_ALLOTED;
	r = omap_request_dma(vout->vrfb_dma_tx.dev_id, "VRFB DMA TX",
			omap_vout_vrfb_dma_tx_callback,
			(void *) &vout->vrfb_dma_tx, &vout->vrfb_dma_tx.dma_ch);
	if (r < 0) {
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;
		printk(KERN_INFO VOUT_NAME ": DMA Channel not alloted\
				for video%d [v4l2]\n", vfd->minor);
	}
	init_waitqueue_head(&vout->vrfb_dma_tx.wait);

	/* Allocate VRFB buffers if selected through bootargs */
	static_vrfb_allocation = (vid_num == 0) ?
		vid1_static_vrfb_alloc : vid2_static_vrfb_alloc;

	/* statically allocated the VRFB buffer is done through
	   commands line aruments */
	if (static_vrfb_allocation) {
		if (omap_vout_allocate_vrfb_buffers(vout, vrfb_num_bufs)) {
			r =  -ENOMEM;
			goto free_buffers;
		}
		vout->vrfb_static_allocation = 1;
	}
	return 0;

free_buffers:
	for (i = 0; i < numbuffers; i++) {
		omap_vout_free_buffer(vout->buf_virt_addr[i],
				vout->buf_phy_addr[i], vout->buffer_size);
		vout->buf_virt_addr[i] = 0;
		vout->buf_phy_addr[i] = 0;
	}
	return r;

}

/* Create video out devices */
static int __init omap_vout_create_video_devices(struct platform_device *pdev)
{
	int r = 0, k;
	struct omap_vout_device *vout;
	struct video_device *vfd = NULL;
	struct omap2video_device *vid_dev = platform_get_drvdata(pdev);

	for (k = 0; k < pdev->num_resources; k++) {

		vout = kmalloc(sizeof(struct omap_vout_device), GFP_KERNEL);
		if (!vout) {
			printk(KERN_ERR VOUT_NAME ": could not allocate \
					memory\n");
			return -ENOMEM;
		}

		memset(vout, 0, sizeof(struct omap_vout_device));

		vout->dev = &pdev->dev;
		vout->vid = k;
		vid_dev->vouts[k] = vout;
		vout->vid_info.vid_dev = vid_dev;
		vout->vid_info.overlays[0] = vid_dev->overlays[k + 1];
		vout->vid_info.num_overlays = 1;
		vout->vid_info.id = k + 1;
		vid_dev->num_videos++;

		/* Setup the default configuration for the video devices
		 */
		if (omap_vout_setup_video_data(vout) != 0) {
			r = -ENOMEM;
			goto error;
		}

		/* Allocate default number of buffers for the video streaming
		 * and reserve the VRFB space for rotation
		 */
		if (omap_vout_setup_video_bufs(pdev, k) != 0) {
			r = -ENOMEM;
			goto error1;
		}

		/* Register the Video device with V4L2
		 */
		vfd = vout->vfd;
		if (video_register_device(vfd, VFL_TYPE_GRABBER, k + 1) < 0) {
			printk(KERN_ERR VOUT_NAME ": could not register \
					Video for Linux device\n");
			vfd->minor = -1;
			r = -ENODEV;
			goto error2;
		}
		video_set_drvdata(vfd, vout);

		/* Configure the overlay structure */
		r = omapvid_init(vid_dev->vouts[k], 0);

		if (r)
			goto error2;
		else
			goto success;
error2:
	omap_vout_release_vrfb(vout);
	omap_vout_free_buffers(vout);
error1:
	video_device_release(vfd);
error:
	kfree(vout);
	return r;

success:
	printk(KERN_INFO VOUT_NAME ": registered and initialized\
			video device %d [v4l2]\n", vfd->minor);
	if (k == (pdev->num_resources - 1))
		return 0;
	}
	return -ENODEV;

}
/* Driver functions */
static int omap_vout_remove(struct platform_device *pdev)
{

	struct omap2video_device *vid_dev = platform_get_drvdata(pdev);
	int k;

	for (k = 0; k < pdev->num_resources; k++)
		omap_vout_cleanup_device(vid_dev->vouts[k]);

	for (k = 0; k < vid_dev->num_displays; k++) {
		if (vid_dev->displays[k]->state != OMAP_DSS_DISPLAY_DISABLED)
			vid_dev->displays[k]->disable(vid_dev->displays[k]);

		omap_dss_put_device(vid_dev->displays[k]);
	}
	kfree(vid_dev);
	return 0;
}

static int __init omap_vout_probe(struct platform_device *pdev)
{
	int r = 0, i;
	struct omap2video_device *vid_dev = NULL;
	struct omap_overlay *ovl;
	struct omap_dss_device *def_display;
	struct omap_dss_device *display = NULL;

	if (pdev->num_resources == 0) {
		dev_err(&pdev->dev, "probed for an unknown device\n");
		r = -ENODEV;
		return r;
	}

	vid_dev = kzalloc(sizeof(struct omap2video_device), GFP_KERNEL);
	if (vid_dev == NULL) {
		r = -ENOMEM;
		return r;
	}

	platform_set_drvdata(pdev, vid_dev);

	vid_dev->num_displays = 0;
	for_each_dss_dev(display) {
		omap_dss_get_device(display);
		if (!display) {
			dev_err(&pdev->dev, "probed for an unknown device\n");
			r = -EINVAL;
			goto error0;
		}
		vid_dev->displays[vid_dev->num_displays++] = display;
	}

	if (vid_dev->num_displays == 0) {
		dev_err(&pdev->dev, "probed for an unknown device\n");
		r = -EINVAL;
		goto error0;
	}

	vid_dev->num_overlays = omap_dss_get_num_overlays();
	for (i = 0; i < vid_dev->num_overlays; i++)
		vid_dev->overlays[i] = omap_dss_get_overlay(i);

	vid_dev->num_managers = omap_dss_get_num_overlay_managers();
	for (i = 0; i < vid_dev->num_managers; i++)
		vid_dev->managers[i] = omap_dss_get_overlay_manager(i);

	/* Get the Video1 overlay and video2 overlay.
	 * Setup the Display attached to that overlays
	 */
	 for (i = 1; i < 3; i++) {
		ovl = omap_dss_get_overlay(i);
		if (ovl->manager && ovl->manager->device) {
			def_display = ovl->manager->device;
		} else {
			dev_err(&pdev->dev, "probed for an unknown device\n");
			r = -EINVAL;
			goto error0;
		}
		r = def_display->enable(def_display);
		if (r) {
			/* Here we are not considering a error as display may be
			enabled by frame buffer driver */
			printk(KERN_WARNING VOUT_NAME
					"Display already enabled\n");
		}
		/* set the update mode */
		if (def_display->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE) {
#ifdef CONFIG_FB_OMAP2_FORCE_AUTO_UPDATE
			if (def_display->set_update_mode)
				def_display->set_update_mode(def_display,
						OMAP_DSS_UPDATE_AUTO);
			if (def_display->enable_te)
				def_display->enable_te(def_display, 1);
#else
			if (def_display->set_update_mode)
				def_display->set_update_mode(def_display,
						OMAP_DSS_UPDATE_MANUAL);
			if (def_display->enable_te)
				def_display->enable_te(def_display, 0);
#endif
		} else {
			if (def_display->set_update_mode)
				def_display->set_update_mode(def_display,
						OMAP_DSS_UPDATE_AUTO);
		}
	 }

	r = omap_vout_create_video_devices(pdev);
	if (r)
		goto error0;

	for (i = 0; i < vid_dev->num_displays; i++) {
		struct omap_dss_device *display = vid_dev->displays[i];

		if (display->update)
			display->update(display, 0, 0,
			display->panel.timings.x_res,
			display->panel.timings.y_res);
	}
	printk(KERN_INFO VOUT_NAME "display->updated\n");
	return 0;

error0:
	kfree(vid_dev);
	return r;
}

static struct platform_driver omap_vout_driver = {
	.driver = {
		   .name = VOUT_NAME,
		   },
	.probe = omap_vout_probe,
	.remove = omap_vout_remove,
};

void omap_vout_isr(void *arg, unsigned int irqstatus)
{
	int r;
	struct timeval timevalue;
	struct omap_vout_device *vout =
	    (struct omap_vout_device *) arg;
	u32 addr, fid;
	struct omapvideo_info *ovid;
	struct omap_overlay *ovl;
	struct omap_dss_device *cur_display;

	if (!vout->streaming)
		return;

	ovid = &(vout->vid_info);
	ovl = ovid->overlays[0];
	/* get the display device attached to the overlay */
	if (!ovl->manager || !ovl->manager->device)
		return;
	cur_display = ovl->manager->device;

	spin_lock(&vout->vbq_lock);
	do_gettimeofday(&timevalue);
	if ((cur_display->type == OMAP_DISPLAY_TYPE_DSI &&
	     irqstatus & DISPC_IRQ_FRAMEDONE) ||
	    (cur_display->type == OMAP_DISPLAY_TYPE_DPI &&
	     irqstatus & DISPC_IRQ_VSYNC)) {

		if (!vout->first_int && (vout->cur_frm != vout->next_frm)) {
			vout->cur_frm->ts = timevalue;
			vout->cur_frm->state = VIDEOBUF_DONE;
			wake_up_interruptible(&vout->cur_frm->done);
			vout->cur_frm = vout->next_frm;
		}
		vout->first_int = 0;
		if (list_empty(&vout->dma_queue)) {
			/* TODO: only update on new frame */
			omapvid_apply_changes(vout);
			spin_unlock(&vout->vbq_lock);
			return;
		}

		vout->next_frm = list_entry(vout->dma_queue.next,
					struct videobuf_buffer, queue);
		list_del(&vout->next_frm->queue);

		vout->next_frm->state = VIDEOBUF_ACTIVE;

		addr = (unsigned long) vout->queued_buf_addr[vout->next_frm->i]
			+ vout->cropped_offset;

		/* First save the configuration in ovelray structure */
		r = omapvid_init(vout, addr);
		if (r)
			printk(KERN_ERR VOUT_NAME "failed to set overlay info\n");
		/* Enable the pipeline and set the Go bit */
		r = omapvid_apply_changes(vout);
		if (r)
			printk(KERN_ERR VOUT_NAME "failed to change mode\n");
	} else {

		if (vout->first_int) {
			vout->first_int = 0;
			spin_unlock(&vout->vbq_lock);
			return;
		}
		if (irqstatus & DISPC_IRQ_EVSYNC_ODD) {
			fid = 1;
		} else if (irqstatus & DISPC_IRQ_EVSYNC_EVEN) {
			fid = 0;
		} else {
			spin_unlock(&vout->vbq_lock);
			return;
		}
		vout->field_id ^= 1;
		if (fid != vout->field_id) {
			if (0 == fid)
				vout->field_id = fid;

			spin_unlock(&vout->vbq_lock);
			return;
		}
		if (0 == fid) {
			if (vout->cur_frm == vout->next_frm) {
				spin_unlock(&vout->vbq_lock);
				return;
			}
			vout->cur_frm->ts = timevalue;
			vout->cur_frm->state = VIDEOBUF_DONE;
			wake_up_interruptible(&vout->cur_frm->done);
			vout->cur_frm = vout->next_frm;
		} else if (1 == fid) {
			if (list_empty(&vout->dma_queue) ||
			    (vout->cur_frm != vout->next_frm)) {
				spin_unlock(&vout->vbq_lock);
				return;
			}
			vout->next_frm = list_entry(vout->dma_queue.next,
					   struct videobuf_buffer, queue);
			list_del(&vout->next_frm->queue);

			vout->next_frm->state = VIDEOBUF_ACTIVE;
			addr = (unsigned long)
			    vout->queued_buf_addr[vout->next_frm->i] +
			    vout->cropped_offset;
			/* First save the configuration in ovelray structure */
			r = omapvid_init(vout, addr);
			if (r)
				printk(KERN_ERR VOUT_NAME "failed to set overlay info\n");
			/* Enable the pipeline and set the Go bit */
			r = omapvid_apply_changes(vout);
			if (r)
				printk(KERN_ERR VOUT_NAME "failed to change mode\n");
		}

	}
	spin_unlock(&vout->vbq_lock);
}

static void omap_vout_cleanup_device(struct omap_vout_device *vout)
{

	struct video_device *vfd;

	if (!vout)
		return;
	vfd = vout->vfd;

	if (vfd) {
		if (vfd->minor == -1) {
			/*
			 * The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(vfd);
		} else {
			/*
			 * The unregister function will release the video_device
			 * struct as well as unregistering it.
			 */
			video_unregister_device(vfd);
		}
	}

	omap_vout_release_vrfb(vout);

	omap_vout_free_buffers(vout);
	/* Free the VRFB buffer if allocated
	 * init time
	 */
	if (vout->vrfb_static_allocation)
		omap_vout_free_vrfb_buffers(vout);

	kfree(vout);
}

static int __init omap_vout_init(void)
{

	if (platform_driver_register(&omap_vout_driver) != 0) {
		printk(KERN_ERR VOUT_NAME ": could not register \
				Video driver\n");
		return -EINVAL;
	}
	return 0;
}

static void omap_vout_cleanup(void)
{
	platform_driver_unregister(&omap_vout_driver);
}

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("OMAP Video for Linux Video out driver");
MODULE_LICENSE("GPL");

late_initcall(omap_vout_init);
module_exit(omap_vout_cleanup);

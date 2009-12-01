/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/version.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/platform_device.h>

#include <plat/display.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"
#include "pvrmodule.h"

MODULE_SUPPORTED_DEVICE(DEVNAME);

#define unref__ __attribute__ ((unused))

void *OMAPLFBAllocKernelMem(unsigned long ulSize)
{
	return kmalloc(ulSize, GFP_KERNEL);
}

void OMAPLFBFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}


OMAP_ERROR OMAPLFBGetLibFuncAddr (char *szFunctionName, PFN_DC_GET_PVRJTABLE *ppfnFuncTable)
{
	if(strcmp("PVRGetDisplayClassJTable", szFunctionName) != 0)
	{
		return (OMAP_ERROR_INVALID_PARAMS);
	}

	
	*ppfnFuncTable = PVRGetDisplayClassJTable;

	return (OMAP_OK);
}

static struct omap_overlay_manager* lcd_mgr = 0;
static struct omap_overlay*         omap_gfxoverlay = 0;
static struct omap_overlay_info     gfxoverlayinfo;
struct fb_info *fb_info;

void OMAPLFBDisplayInit(void)
{
    struct omap_overlay*         overlayptr = 0;
    unsigned int                 i = 0;
    unsigned int                 numoverlays = 0;

    // there is tv and lcd managers... we only care about lcd at this time.
    lcd_mgr = omap_dss_get_overlay_manager(OMAP_DSS_OVL_MGR_LCD);
    if(!lcd_mgr)
    {
        DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBSetDisplayInfo couldn't find lcd overlay manager\n"));
        return;
    }

    numoverlays = omap_dss_get_num_overlays();
    if( numoverlays == 0)
    {
        DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBSetDisplayInfo couldn't find any overlays\n"));
        return;
    }

    for( i = 0; i < numoverlays; i++ )
    {
        overlayptr = omap_dss_get_overlay(i);
        if( strncmp( overlayptr->name, "gfx", 3 ) == 0)
        {
            DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBSetDisplayInfo found GFX overlay\n"));
            omap_gfxoverlay = overlayptr;
            break;
        }
    }
    if( omap_gfxoverlay == 0 )
    {
        DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBSetDisplayInfo GFX overlay no found\n"));
        return;
    }

    omap_gfxoverlay->get_overlay_info( omap_gfxoverlay, &gfxoverlayinfo );
    fb_info =  registered_fb[0];

}

void OMAPLFBSync(void)
{
	if (lcd_mgr && lcd_mgr->device && omap_gfxoverlay)
		lcd_mgr->device->sync(lcd_mgr->device);
}

void OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long paddr)
{
	u32 pixels;

	if (lcd_mgr && lcd_mgr->device && omap_gfxoverlay) {
		omap_gfxoverlay->get_overlay_info(omap_gfxoverlay,
						  &gfxoverlayinfo);
		gfxoverlayinfo.paddr = paddr;
		/* TODO: plumb vaddr in to this function */
		gfxoverlayinfo.vaddr = (void *)(paddr - 0x81314000 + 0xd2800000);

		omap_gfxoverlay->set_overlay_info(omap_gfxoverlay,
						  &gfxoverlayinfo);
		lcd_mgr->apply(lcd_mgr);

		lcd_mgr->device->update(lcd_mgr->device, 0, 0,
					gfxoverlayinfo.width,
					gfxoverlayinfo.height);

		pixels = (paddr - fb_info->fix.smem_start) /
			(fb_info->var.bits_per_pixel / 8);
		fb_info->var.yoffset = pixels / fb_info->var.xres;
		fb_info->var.xoffset = pixels % fb_info->var.xres;
	}
}

static OMAP_BOOL bDeviceSuspended;

static void OMAPLFBCommonSuspend(void)
{
	if (bDeviceSuspended)
	{
		return;
	}

	OMAPLFBDriverSuspend();

	bDeviceSuspended = OMAP_TRUE;
}

static int OMAPLFBDriverProbe_Entry(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "probe\n");
	if(OMAPLFBInit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: OMAPLFBInit failed\n");
		return -ENODEV;
	}

	return 0;
}

static int OMAPLFBDriverSuspend_Entry(struct platform_device unref__ *pDevice, pm_message_t unref__ state)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverSuspend_Entry\n"));

	OMAPLFBCommonSuspend();

	return 0;
}

static int OMAPLFBDriverResume_Entry(struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverResume_Entry\n"));

	OMAPLFBDriverResume();

	bDeviceSuspended = OMAP_FALSE;

	return 0;
}

static IMG_VOID OMAPLFBDriverShutdown_Entry(struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverShutdown_Entry\n"));

	OMAPLFBCommonSuspend();
}

static int __exit OMAPLFBDriverRemove_Entry(struct platform_device *pdev)
{
	if(OMAPLFBDeinit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: OMAPLFBDeinit failed\n");
	}
	return 0;
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name		= DRVNAME,
	},
	.probe		= OMAPLFBDriverProbe_Entry,
 	.suspend	= OMAPLFBDriverSuspend_Entry,
	.resume		= OMAPLFBDriverResume_Entry,
	.shutdown	= OMAPLFBDriverShutdown_Entry,
	.remove		= __exit_p(OMAPLFBDriverRemove_Entry),

};

static int __init OMAPLFB_Init(void)
{
	int error;

	if ((error = platform_driver_register(&omaplfb_driver)) != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: Unable to register platform driver (%d)\n", error);

		return -ENODEV;
	}

	return 0;
}

static IMG_VOID __exit OMAPLFB_Cleanup(IMG_VOID)
{    
	platform_driver_unregister(&omaplfb_driver);

	if(OMAPLFBDeinit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Cleanup: OMAPLFBDeinit failed\n");
	}
}

late_initcall(OMAPLFB_Init);
module_exit(OMAPLFB_Cleanup);

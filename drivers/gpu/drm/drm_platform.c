/*
 * Derived from drm_pci.c
 *
 * Copyright 2003 José Fonseca.
 * Copyright 2003 Leif Delgass.
 * Copyright (c) 2009, Code Aurora Forum.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <linux/pci.h>
#include "drmP.h"

/* TODO:
 * Add an equivelent for drm_irq_by_busid */

static int drm_fill_in_platform_dev(struct drm_device *dev,
				struct platform_device *pdev,
				struct drm_driver *driver)
{
	dev->platformdev = pdev;
	return drm_fill_in_dev(dev, NULL, driver);
}

/**
 * Platform device device initialization. Called via drm_init at module
 * load time,
 *
 * \return zero on success or a negative number on failure.
 *
 * Initializes a drm_device structure,registering the stubs.
 *
 * Expands the \c DRIVER_PREINIT and \c DRIVER_POST_INIT macros before and
 * after the initialization for driver customization.
 */

int drm_platform_init(struct drm_driver *driver)
{
	struct drm_device *dev;
	int ret;

	DRM_DEBUG("\n");

	dev = drm_calloc(1, sizeof(*dev), DRM_MEM_STUB);
	if (!dev)
		return -ENOMEM;

	ret = drm_fill_in_platform_dev(dev, driver->platform_device,
					driver);
	if (ret) {
		printk(KERN_ERR "DRM: Fill_in_platform_dev failed.\n");
		goto err_g1;
	}

	ret = drm_get_minor(dev, &dev->primary, DRM_MINOR_LEGACY);

	if (ret)
		goto err_g1;

	list_add_tail(&dev->driver_item, &driver->device_list);

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		 driver->name, driver->major, driver->minor, driver->patchlevel,
		 driver->date, dev->primary->index);

	return 0;

err_g1:
	drm_free(dev, sizeof(*dev), DRM_MEM_STUB);
	return ret;
}

/* drm_pci.h -- DMA memory management wrappers for DRM -*- linux-c -*- */
/**
 * \file drm_pci.c
 * \brief Functions and ioctls to manage DMA memory
 *
 * \warning These interfaces aren't stable yet.
 *
 * \todo Implement the remaining ioctl's for the PCI pools.
 * \todo The wrappers here are so thin that they would be better off inlined..
 *
 * \author José Fonseca <jrfonseca@tungstengraphics.com>
 * \author Leif Delgass <ldelgass@retinalburn.net>
 */

/*
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
#include <linux/dma-mapping.h>
#include "drmP.h"

#ifdef CONFIG_PCI

static int drm_fill_in_pci_dev(struct drm_device *dev,
				struct pci_dev *pdev,
				const struct pci_device_id *ent,
				struct drm_driver *driver)
{
	dev->pdev = pdev;
	dev->pci_device = pdev->device;
	dev->pci_vendor = pdev->vendor;

	return drm_fill_in_dev(dev, ent, driver);
}

/**********************************************************************/
/** \name PCI memory */
/*@{*/

/**
 * Register.
 *
 * \param pdev - PCI device structure
 * \param ent entry from the PCI ID table with device type flags
 * \return zero on success or a negative number on failure.
 *
 * Attempt to gets inter module "drm" information. If we are first
 * then register the character device and inter module information.
 * Try and register, if we fail to register, backout previous work.
 */

int drm_get_pci_dev(struct pci_dev *pdev, const struct pci_device_id *ent,
		struct drm_driver *driver)
{
	struct drm_device *dev;
	int ret;

	DRM_DEBUG("\n");

	dev = drm_calloc(1, sizeof(*dev), DRM_MEM_STUB);
	if (!dev)
		return -ENOMEM;

	ret = pci_enable_device(pdev);
	if (ret)
		goto err_g1;

	pci_set_master(pdev);

	ret = drm_fill_in_pci_dev(dev, pdev, ent, driver);
	if (ret) {
		printk(KERN_ERR "DRM: Fill_in_pci_dev failed.\n");
		goto err_g2;
	}

	ret = drm_get_minor(dev, &dev->primary, DRM_MINOR_LEGACY);
	if (ret)
		goto err_g2;

	list_add_tail(&dev->driver_item, &driver->device_list);

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n",
		 driver->name, driver->major, driver->minor, driver->patchlevel,
		 driver->date, dev->primary->index);

	return 0;

err_g2:
	pci_disable_device(pdev);
err_g1:
	drm_free(dev, sizeof(*dev), DRM_MEM_STUB);
	return ret;
}

/**
 * PCI device initialization. Called via drm_init at module load time,
 *
 * \return zero on success or a negative number on failure.
 *
 * Initializes a drm_device structures,registering the
 * stubs and initializing the AGP device.
 *
 * Expands the \c DRIVER_PREINIT and \c DRIVER_POST_INIT macros before and
 * after the initialization for driver customization.
 */

int drm_pci_init(struct drm_driver *driver)
{
	struct pci_dev *pdev = NULL;
	struct pci_device_id *pid;
	int i;

	DRM_DEBUG("\n");

	for (i = 0; driver->pci_driver.id_table[i].vendor != 0; i++) {
		pid = (struct pci_device_id *)&driver->pci_driver.id_table[i];

		pdev = NULL;
		/* pass back in pdev to account for multiple identical cards */
		while ((pdev =
			pci_get_subsys(pid->vendor, pid->device, pid->subvendor,
				       pid->subdevice, pdev)) != NULL) {
			/* stealth mode requires a manual probe */
			pci_dev_get(pdev);
			drm_get_pci_dev(pdev, pid, driver);
		}
	}
	return 0;
}

#else

int drm_pci_init(struct drm_driver *driver)
{
	return -1;
}

#endif

/*@}*/

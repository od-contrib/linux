/*
 * drivers/gpu/xburst/xburst_ion.c
 *
 * Copyright (C) 2011 Google, Inc.
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

#include <linux/err.h>
#include <linux/ion.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <asm/uaccess.h>

#include "../ion_priv.h"

struct ion_device *xburst_ion_device;
EXPORT_SYMBOL(xburst_ion_device);

static struct ion_heap *xburst_system_heap;

static int xburst_ion_probe(struct platform_device *pdev)
{
	int err;

	/* This is just an example. A real platform would have more heaps
	 * than just the system heap, if not all peripherals had an MMU.
	 */
	struct ion_platform_heap xburst_system_heap_data = {
		.type = ION_HEAP_TYPE_SYSTEM,
		.id   = ION_HEAP_TYPE_SYSTEM,
		.name = "system",
	};

	xburst_ion_device = ion_device_create(NULL);
	if (IS_ERR_OR_NULL(xburst_ion_device))
		return PTR_ERR(xburst_ion_device);

	xburst_system_heap = ion_heap_create(&xburst_system_heap_data);
	if (IS_ERR_OR_NULL(xburst_system_heap)) {
		err = PTR_ERR(xburst_system_heap);
		return err;
	}

	ion_device_add_heap(xburst_ion_device, xburst_system_heap);
	platform_set_drvdata(pdev, xburst_ion_device);
	return 0;
}

static int xburst_ion_remove(struct platform_device *pdev)
{
	struct ion_device *idev = platform_get_drvdata(pdev);

	ion_heap_destroy(xburst_system_heap);
	ion_device_destroy(idev);
	return 0;
}

static struct platform_driver ion_driver = {
	.probe = xburst_ion_probe,
	.remove = xburst_ion_remove,
	.driver = { .name = "ion-xburst" }
};

/* Begin stuff that should be in the board files, not hand-coded here */

/* This should be part of the platform device data, and define more than
 * just one ion heap. For example, contiguous memory heaps should be defined,
 * so that the memory can be reserved early, long before ion initialization.
 */

static void fake_ion_device_release(struct device *pdev) { }

static struct platform_device fake_ion_device = {
	.name = "ion-xburst",
	.id   = -1,
	.dev  = {
		.release = fake_ion_device_release,
	},
};

/* End stuff that should be in the board files, not hand-coded here */

static int __init ion_init(void)
{
	int err;

	err = platform_driver_register(&ion_driver);
	if(err)
		return err;

	err = platform_device_register(&fake_ion_device);
	if(err)
		platform_driver_unregister(&ion_driver);

	return err;
}

static void __exit ion_exit(void)
{
	platform_device_unregister(&fake_ion_device);
	platform_driver_unregister(&ion_driver);
}

module_init(ion_init);
module_exit(ion_exit);

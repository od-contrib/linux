/*
 * UR-Board Ethernet setup
 *
 * Copyright (c) 2014 Imagination Technologies
 * Author: Alex Smith <alex.smith@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dm9000.h>

#include <mach/jz4780_efuse.h>

#include <soc/base.h>

#include <gpio.h>

struct ci20_otp {
	uint32_t serial_number;
	uint32_t date;
	char manufacturer[2];
	unsigned char mac[6];
} __packed;

#define DM9000_ETH_RET	GPIO_PF(12)
#define DM9000_ETH_INT	GPIO_PE(19)

static struct resource dm9000_resource[] = {
	[0] = {
		.start = NEMC_CS6_IOBASE,
		.end = NEMC_CS6_IOBASE + 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = NEMC_CS6_IOBASE + 2,
		.end = NEMC_CS6_IOBASE + 5,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = IRQ_GPIO_BASE + GPIO_PE(19),
		.end   = IRQ_GPIO_BASE + GPIO_PE(19),
		.flags = IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
};

static int dm9000_gpio[] = {
	[0] =  DM9000_ETH_RET,
	[1] =  DM9000_ETH_INT,
};

static struct dm9000_plat_data dm9000_platform_data = {
	.gpio = dm9000_gpio,
	.flags = DM9000_PLATF_8BITONLY | DM9000_PLATF_NO_EEPROM,
};

static struct platform_device ci20_dm9000 = {
	.name = "dm9000",
	.id = 0,
	.resource = dm9000_resource,
	.num_resources = ARRAY_SIZE(dm9000_resource),
	.dev = {
		.platform_data = &dm9000_platform_data,
	},
};

/*
 * This function must be called late so that the EFUSE driver is initialized
 * by the time it gets called.
 */
static int __init ci20_eth_init(void)
{
	struct ci20_otp otp;

#ifdef CONFIG_JZ4780_EFUSE
	/* Read out the MAC address from the customer ID area of the EFUSE. */
	jz_efuse_id_read(0, (uint32_t *)&otp);
	memcpy(dm9000_platform_data.dev_addr, otp.mac, sizeof(otp.mac));
#else
	pr_warn("EFUSE driver not enabled, cannot read MAC address\n");
#endif

	platform_device_register(&ci20_dm9000);
	return 0;
}
late_initcall(ci20_eth_init);

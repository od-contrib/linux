/*
 * [board]-pmu.c - This file defines PMU board information.
 *
 * Copyright (C) 2012 Ingenic Semiconductor Co., Ltd.
 * Author: Large Dipper <ykli@ingenic.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/pmu-common.h>
#include <linux/i2c.h>
#include <gpio.h>

/**
 * Core voltage Regulator.
 * Couldn't be modified.
 * Voltage was inited at bootloader.
 */
CORE_REGULATOR_DEF(
	ci20,	1000000,	1400000);

/**
 * I/O Regulator.
 * It's the parent regulator of most of devices regulator on board.
 * Voltage was inited at bootloader.
 */
IO_REGULATOR_DEF(
	ci20_vccio,
	"Vcc-IO",	3300000,	1);

/**
 * USB VBUS Regulators.
 * Switch of USB VBUS. It may be a actual or virtual regulator.
 */
VBUS_REGULATOR_DEF(
	ci20,	"OTG-Vbus");
/**
 * Exclusive Regulators.
 * They are only used by one device each other.
 */
EXCLUSIVE_REGULATOR_DEF(
	ci20_vwifi,
	"Wi-Fi",	"vwifi",	NULL,
	NULL,		3300000,	0);

EXCLUSIVE_REGULATOR_DEF(
	ci20_vcim,
	"Camera1.8",	"vcim",		"Camera2.8",
	NULL,		1500000,	1);

EXCLUSIVE_REGULATOR_DEF(
	ci20_vcim_2_8,
	"Camera2.8",	"vcim_2_8",	NULL,
	NULL,		2800000,	1);

EXCLUSIVE_REGULATOR_DEF(
	ci20_vdrvvbus,
	"OTG-Vbus",	"vdrvvbus",	NULL,
	"jz-vbus",	5000000,	0);
/**
 * Fixed voltage Regulators.
 * GPIO silulator regulators. Everyone is an independent device.
 */

FIXED_REGULATOR_DEF(
	ci20_vcc5,
	"Vcc-5V",	5000000,	GPIO_PA(25),
	HIGH_ENABLE,	EN_AT_BOOT,	0,
	NULL,		"vhdmi",	"jz-hdmi");

FIXED_REGULATOR_DEF(
	ci20_ethnet,
	"ethnet",	3300000,	GPIO_PB(25),
	HIGH_ENABLE,	UN_AT_BOOT,	0,
	NULL,		"vethnet",	NULL);

static struct platform_device *fixed_regulator_devices[] __initdata = {
	&ci20_vcc5_regulator_device,
	&ci20_ethnet_regulator_device,
};

/*
 * Regulators definitions used in PMU.
 *
 * If +5V is supplied by PMU, please define "VBUS" here with init_data NULL,
 * otherwise it should be supplied by a exclusive DC-DC, and you should define
 * it as a fixed regulator.
 */
static struct regulator_info ci20_pmu_regulators[] = {
	{"OUT1", &ci20_vcore_init_data},
	{"OUT3", &ci20_vccio_init_data},
	{"OUT4", &ci20_vdrvvbus_init_data},
	{"OUT6", &ci20_vwifi_init_data},
	{"OUT7", &ci20_vcim_2_8_init_data},
	{"OUT8", &ci20_vcim_init_data},
	{"VBUS", &ci20_vbus_init_data},
};


static struct pmu_platform_data ci20_pmu_pdata = {
	.gpio = GPIO_PA(28),
	.num_regulators = ARRAY_SIZE(ci20_pmu_regulators),
	.regulators = ci20_pmu_regulators,
};

#define PMU_I2C_BUSNUM 0

struct i2c_board_info ci20_pmu_board_info = {
	I2C_BOARD_INFO("act8600", 0x5a),
	.platform_data = &ci20_pmu_pdata,
};

static int __init ci20_pmu_dev_init(void)
{
	struct i2c_adapter *adap;
	struct i2c_client *client;
	int busnum = PMU_I2C_BUSNUM;
	int i;

	adap = i2c_get_adapter(busnum);
	if (!adap) {
		pr_err("failed to get adapter i2c%d\n", busnum);
		return -1;
	}

	client = i2c_new_device(adap, &ci20_pmu_board_info);
	if (!client) {
		pr_err("failed to register pmu to i2c%d\n", busnum);
		return -1;
	}

	i2c_put_adapter(adap);

	for (i = 0; i < ARRAY_SIZE(fixed_regulator_devices); i++)
		fixed_regulator_devices[i]->id = i;

	return platform_add_devices(fixed_regulator_devices,
				    ARRAY_SIZE(fixed_regulator_devices));
}

subsys_initcall_sync(ci20_pmu_dev_init);

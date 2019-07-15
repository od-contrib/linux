// SPDX-License-Identifier: GPL-2.0
/*
 * JZ47xx SoCs TCU MFD driver
 * Copyright (C) 2019 Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/mfd/ingenic-tcu.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

static struct regmap *tcu_regmap __initdata;

static const struct regmap_config ingenic_tcu_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = TCU_REG_OST_CNTHBUF,
};

static const struct of_device_id ingenic_tcu_of_match[] = {
	{ .compatible = "ingenic,jz4740-tcu" },
	{ .compatible = "ingenic,jz4725b-tcu" },
	{ .compatible = "ingenic,jz4770-tcu" },
	{ /* sentinel */ }
};

static struct regmap * __init ingenic_tcu_create_regmap(struct device_node *np)
{
	struct resource res;
	void __iomem *base;
	struct regmap *map;

	if (!of_match_node(ingenic_tcu_of_match, np))
		return ERR_PTR(-EINVAL);

	base = of_io_request_and_map(np, 0, "TCU");
	if (IS_ERR(base))
		return ERR_CAST(base);

	map = regmap_init_mmio(NULL, base, &ingenic_tcu_regmap_config);
	if (IS_ERR(map))
		goto err_iounmap;

	return map;

err_iounmap:
	iounmap(base);
	of_address_to_resource(np, 0, &res);
	release_mem_region(res.start, resource_size(&res));

	return map;
}

static int __init ingenic_tcu_probe(struct platform_device *pdev)
{
	struct regmap *map = ingenic_tcu_get_regmap(pdev->dev.of_node);

	platform_set_drvdata(pdev, map);

	regmap_attach_dev(&pdev->dev, map, &ingenic_tcu_regmap_config);

	return devm_of_platform_populate(&pdev->dev);
}

static struct platform_driver ingenic_tcu_driver = {
	.driver = {
		.name = "ingenic-tcu",
		.of_match_table = ingenic_tcu_of_match,
	},
};

static int __init ingenic_tcu_platform_init(void)
{
	return platform_driver_probe(&ingenic_tcu_driver,
				     ingenic_tcu_probe);
}
subsys_initcall(ingenic_tcu_platform_init);

struct regmap * __init ingenic_tcu_get_regmap(struct device_node *np)
{
	if (!tcu_regmap)
		tcu_regmap = ingenic_tcu_create_regmap(np);

	return tcu_regmap;
}

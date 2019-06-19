// SPDX-License-Identifier: GPL-2.0
//
// Ingenic Smart LCD driver
//
// Copyright (C) 2019, Paul Cercueil <paul@crapouillou.net>

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#define JZ_REG_SLCD_MCFG	0x00
#define JZ_REG_SLCD_MCTRL	0x04
#define JZ_REG_SLCD_MSTATE	0x08
#define JZ_REG_SLCD_MDATA	0x0c

#define JZ_SLCD_MCFG_DWIDTH_OFFSET	10
#define JZ_SLCD_MCFG_DWIDTH_16BIT	(1 << JZ_SLCD_MCFG_DWIDTH_OFFSET)
#define JZ_SLCD_MCFG_DWIDTH_8BIT	(4 << JZ_SLCD_MCFG_DWIDTH_OFFSET)

#define JZ_SLCD_MCFG_CWIDTH_OFFSET	8
#define JZ_SLCD_MCFG_CWIDTH_16BIT	(0 << JZ_SLCD_MCFG_CWIDTH_OFFSET)
#define JZ_SLCD_MCFG_CWIDTH_8BIT	(1 << JZ_SLCD_MCFG_CWIDTH_OFFSET)

#define JZ_SLCD_MCFG_TYPE_SERIAL	BIT(0)

#define JZ_SLCD_MSTATE_BUSY		BIT(0)

#define JZ_SLCD_MDATA_DATA		BIT(31)

struct ingenic_slcd {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;

	struct mipi_dsi_host dsi_host;
};

static bool ingenic_slcd_reg_readable(struct device *dev, unsigned int reg)
{
	/* Reading registers is not supported. */
	return false;
}

static void ingenic_slcd_wait(struct ingenic_slcd *priv)
{
	u16 val;

	dev_warn(priv->dev, "Waiting for ready...");

	readb_poll_timeout(priv->base + JZ_REG_SLCD_MSTATE, val,
			   !(val & JZ_SLCD_MSTATE_BUSY), 4, 0);

	dev_warn(priv->dev, "Ready!");
}

static void ingenic_slcd_send_command(struct ingenic_slcd *priv, u32 cmd)
{
	ingenic_slcd_wait(priv);

	writel(cmd, priv->base + JZ_REG_SLCD_MDATA);
}

static void ingenic_slcd_send_data(struct ingenic_slcd *priv, u32 data)
{
	ingenic_slcd_wait(priv);

	writel(data | JZ_SLCD_MDATA_DATA, priv->base + JZ_REG_SLCD_MDATA);
}

static int ingenic_slcd_reg_write(void *ctx, unsigned int reg, unsigned int val)
{
	struct ingenic_slcd *priv = ctx;
	int ret;

	ret = clk_enable(priv->clk);
	if (ret)
		return ret;

	ingenic_slcd_send_command(priv, reg);
	ingenic_slcd_send_data(priv, val);

	clk_disable(priv->clk);

	return 0;
}

static const struct regmap_config ingenic_slcd_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.readable_reg = ingenic_slcd_reg_readable,
	.reg_write = ingenic_slcd_reg_write,
};

static inline struct ingenic_slcd *dsi_host_get_priv(struct mipi_dsi_host *host)
{
	return container_of(host, struct ingenic_slcd, dsi_host);
}

static int ingenic_slcd_dsi_attach(struct mipi_dsi_host *host,
				   struct mipi_dsi_device *dsi)
{
	struct ingenic_slcd *priv = dsi_host_get_priv(host);
	int ret;
	u16 cfg;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	/* Configure for serial transfer, 8-bit commands and 8-bit data */
	cfg = JZ_SLCD_MCFG_TYPE_SERIAL
		| JZ_SLCD_MCFG_DWIDTH_8BIT
		| JZ_SLCD_MCFG_CWIDTH_8BIT;
	writew(cfg, priv->base + JZ_REG_SLCD_MCFG);

	return 0;
}

static int ingenic_slcd_dsi_detach(struct mipi_dsi_host *host,
				   struct mipi_dsi_device *dsi)
{
	struct ingenic_slcd *priv = dsi_host_get_priv(host);

	clk_disable_unprepare(priv->clk);

	return 0;
}

static ssize_t ingenic_slcd_dsi_transfer(struct mipi_dsi_host *host,
					 const struct mipi_dsi_msg *msg)
{
	struct ingenic_slcd *priv = dsi_host_get_priv(host);
	unsigned int i;
	int ret;

	/* We only support sending messages, not receiving */
	if (msg->rx_len)
		return -ENOTSUPP;

	ret = clk_enable(priv->clk);
	if (ret)
		return ret;

	ingenic_slcd_send_command(priv, msg->type);

	for (i = 0; i < msg->tx_len; i++)
		ingenic_slcd_send_data(priv, ((u8 *)msg->tx_buf)[i]);

	clk_disable(priv->clk);

	return msg->tx_len + 1;
}

static const struct mipi_dsi_host_ops ingenic_slcd_dsi_ops = {
	.attach = ingenic_slcd_dsi_attach,
	.detach = ingenic_slcd_dsi_detach,
	.transfer = ingenic_slcd_dsi_transfer,
};

static int ingenic_slcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ingenic_slcd *priv;
	struct regmap *map;
	u16 cfg;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	priv->clk = devm_clk_get(dev, "slcd");
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "Unable to get SLCD clock");
		return PTR_ERR(priv->clk);
	}

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base)) {
		dev_err(dev, "Unable to get memory resource");
		return PTR_ERR(priv->base);
	}

	priv->dsi_host.dev = dev;
	priv->dsi_host.ops = &ingenic_slcd_dsi_ops;

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(dev, "Unable to enable clock");
		return ret;
	}

	/* Default to parallel, 16-bit commands and 16-bit data */
	cfg = JZ_SLCD_MCFG_DWIDTH_16BIT | JZ_SLCD_MCFG_CWIDTH_16BIT;
	writew(cfg, priv->base + JZ_REG_SLCD_MCFG);

	clk_disable(priv->clk);

	map = devm_regmap_init(dev, NULL, priv, &ingenic_slcd_regmap_config);
	if (IS_ERR(map)) {
		dev_err(dev, "Failed to init regmap");
		ret = PTR_ERR(map);
		goto err_clk_unprepare;
	}

	ret = regmap_attach_dev(dev, map, &ingenic_slcd_regmap_config);
	if (ret) {
		dev_err(dev, "Unable to attach regmap to device");
		goto err_clk_unprepare;
	}

	ret = mipi_dsi_host_register(&priv->dsi_host);
	if (ret) {
		dev_err(dev, "Unable to register DSI host");
		goto err_clk_unprepare;
	}

	ret = devm_of_platform_populate(dev);
	if (ret) {
		dev_err(dev, "Unable to populate children");
		goto err_dsi_host_unregister;
	}

	return 0;

err_dsi_host_unregister:
	mipi_dsi_host_unregister(&priv->dsi_host);
err_clk_unprepare:
	clk_unprepare(priv->clk);
	return ret;
}

static int ingenic_slcd_remove(struct platform_device *pdev)
{
	struct ingenic_slcd *priv = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&priv->dsi_host);
	clk_unprepare(priv->clk);

	return 0;
}

static const struct of_device_id ingenic_slcd_of_match[] = {
	{ .compatible = "ingenic,jz4740-slcd", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ingenic_slcd_of_match);

static struct platform_driver ingenic_slcd_driver = {
	.driver = {
		.name = "ingenic-slcd",
		.of_match_table = of_match_ptr(ingenic_slcd_of_match),
	},
	.probe = ingenic_slcd_probe,
	.remove = ingenic_slcd_remove,
};
module_platform_driver(ingenic_slcd_driver);

MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_DESCRIPTION("Ingenic Smart LCD driver\n");
MODULE_LICENSE("GPL v2");

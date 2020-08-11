// SPDX-License-Identifier: GPL-2.0
/*
 * Sharp Y030XX067A IPS LCD panel driver
 *
 * Copyright (C) 2020, Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct y030xx067a_info {
	const struct drm_display_mode *display_modes;
	unsigned int num_modes;
	u16 width_mm, height_mm;
	u32 bus_format, bus_flags;
};

struct y030xx067a {
	struct drm_panel panel;
	struct spi_device *spi;
	struct regmap *map;

	const struct y030xx067a_info *panel_info;

	struct regulator *supply;
	struct gpio_desc *reset_gpio;
};

static inline struct y030xx067a *to_y030xx067a(struct drm_panel *panel)
{
	return container_of(panel, struct y030xx067a, panel);
}

static const struct reg_sequence y030xx067a_init_sequence[] = {
	{ 0x02, 0x7f, },
	{ 0x03, 0x0a, },
	{ 0x04, 0x80, },
	{ 0x06, 0x90, },
	{ 0x08, 0x28, },
	{ 0x09, 0x20, },
	{ 0x0a, 0x20, },
	{ 0x0c, 0x10, },
	{ 0x0d, 0x10, },
	{ 0x0e, 0x90, },
	{ 0x10, 0x7f, },
	{ 0x11, 0x3f, },
};

static int y030xx067a_prepare(struct drm_panel *panel)
{
	struct y030xx067a *priv = to_y030xx067a(panel);
	struct device *dev = &priv->spi->dev;
	int err;

	err = regulator_enable(priv->supply);
	if (err) {
		dev_err(dev, "Failed to enable power supply: %d\n", err);
		return err;
	}

	/* Reset the chip */
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	usleep_range(1000, 20000);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(1000, 20000);

	err = regmap_multi_reg_write(priv->map, y030xx067a_init_sequence,
				     ARRAY_SIZE(y030xx067a_init_sequence));
	if (err) {
		dev_err(dev, "Failed to init registers: %d\n", err);
		goto err_disable_regulator;
	}

	return 0;

err_disable_regulator:
	regulator_disable(priv->supply);
	return err;
}

static int y030xx067a_unprepare(struct drm_panel *panel)
{
	struct y030xx067a *priv = to_y030xx067a(panel);

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	regulator_disable(priv->supply);

	return 0;
}

static int y030xx067a_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct y030xx067a *priv = to_y030xx067a(panel);
	const struct y030xx067a_info *panel_info = priv->panel_info;
	struct drm_display_mode *mode;
	unsigned int i;

	for (i = 0; i < panel_info->num_modes; i++) {
		mode = drm_mode_duplicate(connector->dev,
					  &panel_info->display_modes[i]);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (panel_info->num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.bpc = 6;
	connector->display_info.width_mm = panel_info->width_mm;
	connector->display_info.height_mm = panel_info->height_mm;

	drm_display_info_set_bus_formats(&connector->display_info,
					 &panel_info->bus_format, 1);
	connector->display_info.bus_flags = panel_info->bus_flags;

	return panel_info->num_modes;
}

static const struct drm_panel_funcs y030xx067a_funcs = {
	.prepare	= y030xx067a_prepare,
	.unprepare	= y030xx067a_unprepare,
	.get_modes	= y030xx067a_get_modes,
};

static const struct regmap_config y030xx067a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x15,
};

static int y030xx067a_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct y030xx067a *priv;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spi = spi;
	spi_set_drvdata(spi, priv);

	priv->map = devm_regmap_init_spi(spi, &y030xx067a_regmap_config);
	if (IS_ERR(priv->map)) {
		dev_err(dev, "Unable to init regmap\n");
		return PTR_ERR(priv->map);
	}

	priv->panel_info = of_device_get_match_data(dev);
	if (!priv->panel_info)
		return -EINVAL;

	priv->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(priv->supply)) {
		dev_err(dev, "Failed to get power supply\n");
		return PTR_ERR(priv->supply);
	}

	priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio)) {
		dev_err(dev, "Failed to get reset GPIO\n");
		return PTR_ERR(priv->reset_gpio);
	}

	err = drm_panel_of_backlight(&priv->panel);
	if (err)
		return err;

	drm_panel_init(&priv->panel, dev, &y030xx067a_funcs,
		       DRM_MODE_CONNECTOR_DPI);

	err = drm_panel_add(&priv->panel);
	if (err < 0) {
		dev_err(dev, "Failed to add panel\n");
		return err;
	}

	return 0;
}

static int y030xx067a_remove(struct spi_device *spi)
{
	struct y030xx067a *priv = spi_get_drvdata(spi);

	drm_panel_remove(&priv->panel);
	drm_panel_disable(&priv->panel);
	drm_panel_unprepare(&priv->panel);

	return 0;
}

static const struct drm_display_mode y030xx067a_modes[] = {
	{
		.clock = 54000,
		.hdisplay = 320,
		.hsync_start = 320 + 128,
		.hsync_end = 320 + 128 + 28,
		.htotal = 320 + 128 + 28 + 25,
		.vdisplay = 480,
		.vsync_start = 480 + 36,
		.vsync_end = 480 + 36 + 1,
		.vtotal = 480 + 36 + 1 + 16,
		.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
	},
};

static const struct y030xx067a_info y030xx067a_info = {
	.display_modes = y030xx067a_modes,
	.num_modes = ARRAY_SIZE(y030xx067a_modes),
	.width_mm = 69,
	.height_mm = 51,
	.bus_format = MEDIA_BUS_FMT_RGB565_2X8_LE,
	.bus_flags = 0,
};

static const struct of_device_id y030xx067a_of_match[] = {
	{ .compatible = "sharp,y030xx067a", .data = &y030xx067a_info },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, y030xx067a_of_match);

static struct spi_driver y030xx067a_driver = {
	.driver = {
		.name = "sharp-y030xx067a",
		.of_match_table = y030xx067a_of_match,
	},
	.probe = y030xx067a_probe,
	.remove = y030xx067a_remove,
};
module_spi_driver(y030xx067a_driver);

MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_LICENSE("GPL v2");

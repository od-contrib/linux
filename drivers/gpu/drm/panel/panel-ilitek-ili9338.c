// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct ili9338 {
	struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;

	struct backlight_device *backlight;
	struct gpio_desc	*reset_gpiod;
	struct gpio_desc	*cs_gpiod;
};

struct ili9338_instr {
	u8 cmd;
	unsigned int payload_size;
	const u8 *payload;
};

#define ILI9338_CMD(_cmd, ...)					\
{ .cmd = _cmd,							\
  .payload_size = ARRAY_SIZE(((const u8 []){__VA_ARGS__})),	\
  .payload = (const u8 []){__VA_ARGS__}				\
}
static const struct ili9338_instr ili9338_init[] = {
	ILI9338_CMD(0xcb, 0x01),
	ILI9338_CMD(0xc0, 0x26, 0x01),
	ILI9338_CMD(0xc1, 0x10),
	ILI9338_CMD(0xc5, 0x10, 0x52),
	ILI9338_CMD(MIPI_DCS_SET_GAMMA_CURVE, 0x01),
	ILI9338_CMD(0xe0, 0x10, 0x10, 0x10,
		    0x08, 0x0e, 0x06, 0x42,
		    0x28, 0x36, 0x03, 0x0e,
		    0x04, 0x13, 0x0e, 0x0c),
	ILI9338_CMD(0xe1, 0x0c, 0x23, 0x26,
		    0x04, 0x0c, 0x04, 0x39,
		    0x24, 0x4b, 0x03, 0x0b,
		    0x0b, 0x33, 0x37, 0x0f),
	ILI9338_CMD(MIPI_DCS_SET_COLUMN_ADDRESS, 0x00, 0x00, 0x01, 0x3f),
	ILI9338_CMD(MIPI_DCS_SET_PAGE_ADDRESS, 0x00, 0x00, 0x00, 0xef),
	ILI9338_CMD(MIPI_DCS_SET_ADDRESS_MODE, 0xe8),
	ILI9338_CMD(MIPI_DCS_SET_PIXEL_FORMAT, 0x05),
};

static inline struct ili9338 *panel_to_ili9338(struct drm_panel *panel)
{
	return container_of(panel, struct ili9338, panel);
}

static int ili9338_prepare(struct drm_panel *panel)
{
	struct ili9338 *ctx = panel_to_ili9338(panel);
	unsigned int i;
	int ret;

	gpiod_set_value(ctx->cs_gpiod, 0);
	gpiod_set_value(ctx->reset_gpiod, 0);
	msleep(1);
	gpiod_set_value(ctx->reset_gpiod, 1);
	msleep(120);

	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
	if (ret)
		goto out_err;

	for (i = 0; i < ARRAY_SIZE(ili9338_init); i++) {
		const struct ili9338_instr *instr = &ili9338_init[i];

		ret = mipi_dsi_dcs_write(ctx->dsi, instr->cmd,
					 instr->payload, instr->payload_size);
		if (ret < 0)
			goto out_err;
	}

	ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	if (ret)
		goto out_err;

	ret = mipi_dsi_dcs_write(ctx->dsi, MIPI_DCS_WRITE_MEMORY_START, NULL, 0);
	if (ret < 0)
		goto out_err;

	return 0;

out_err:
	dev_err(&ctx->dsi->dev, "Unable to prepare: %i\n", ret);

	return ret;
}

static int ili9338_enable(struct drm_panel *panel)
{
	struct ili9338 *ctx = panel_to_ili9338(panel);

	backlight_enable(ctx->backlight);

	return 0;
}

static int ili9338_disable(struct drm_panel *panel)
{
	struct ili9338 *ctx = panel_to_ili9338(panel);

	backlight_disable(ctx->backlight);

	return 0;
}

static int ili9338_unprepare(struct drm_panel *panel)
{
	struct ili9338 *ctx = panel_to_ili9338(panel);
	int ret;

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret)
		dev_warn(&ctx->dsi->dev, "Unable to disable: %i\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	if (ret)
		dev_warn(&ctx->dsi->dev, "Unable to unprepare: %i\n", ret);

	gpiod_set_value(ctx->reset_gpiod, 0);
	gpiod_set_value(ctx->cs_gpiod, 1);

	return 0;
}

static const struct drm_display_mode ili9338_default_mode = {
	.clock		= 6000,
	.vrefresh	= 60,

	.hdisplay	= 320,
	.hsync_start	= 320 + 10,
	.hsync_end	= 320 + 10 + 20,
	.htotal		= 320 + 10 + 20 + 30,

	.vdisplay	= 240,
	.vsync_start	= 240 + 10,
	.vsync_end	= 240 + 10 + 10,
	.vtotal		= 240 + 10 + 10 + 20,
};

static int ili9338_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct ili9338 *ctx = panel_to_ili9338(panel);
	struct drm_display_mode *mode;
	u32 format = MEDIA_BUS_FMT_RGB565_1X16;

	mode = drm_mode_duplicate(panel->drm, &ili9338_default_mode);
	if (!mode) {
		dev_err(&ctx->dsi->dev, "failed to add mode %ux%ux@%u\n",
			ili9338_default_mode.hdisplay,
			ili9338_default_mode.vdisplay,
			ili9338_default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.bpc = 8;
	panel->connector->display_info.width_mm = 71;
	panel->connector->display_info.height_mm = 53;

	drm_display_info_set_bus_formats(&connector->display_info, &format, 1);
	connector->display_info.bus_flags = 0;

	return 1;
}

static const struct drm_panel_funcs ili9338_funcs = {
	.prepare	= ili9338_prepare,
	.unprepare	= ili9338_unprepare,
	.enable		= ili9338_enable,
	.disable	= ili9338_disable,
	.get_modes	= ili9338_get_modes,
};

static int ili9338_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct ili9338 *ctx;
	int ret;

	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dsi = dsi;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = &dsi->dev;
	ctx->panel.funcs = &ili9338_funcs;

	ctx->reset_gpiod = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpiod)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(ctx->reset_gpiod);
	}

	ctx->cs_gpiod = devm_gpiod_get(&dsi->dev, "cs", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->cs_gpiod)) {
		dev_err(&dsi->dev, "Couldn't get our cs GPIO\n");
		return PTR_ERR(ctx->cs_gpiod);
	}

	ctx->backlight = devm_of_find_backlight(&dsi->dev);
	if (IS_ERR(ctx->backlight)) {
		ret = PTR_ERR(ctx->backlight);
		if (ret != -EPROBE_DEFER)
			dev_err(&dsi->dev, "Failed to get backlight handle");
		return ret;
	}

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = 4;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		return ret;

	dev_notice(&dsi->dev, "probed");

	return 0;
}

static int ili9338_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct ili9338 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	if (ctx->backlight)
		put_device(&ctx->backlight->dev);

	return 0;
}

static const struct of_device_id ili9338_of_match[] = {
	{ .compatible = "ilitek,ili9338" },
	{ }
};
MODULE_DEVICE_TABLE(of, ili9338_of_match);

static struct mipi_dsi_driver ili9338_dsi_driver = {
	.probe		= ili9338_dsi_probe,
	.remove		= ili9338_dsi_remove,
	.driver = {
		.name		= "ili9338-dsi",
		.of_match_table	= ili9338_of_match,
	},
};
module_mipi_dsi_driver(ili9338_dsi_driver);

MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_DESCRIPTION("Ilitek ILI9338 Controller Driver");
MODULE_LICENSE("GPL v2");

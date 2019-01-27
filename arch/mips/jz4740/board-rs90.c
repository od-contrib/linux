#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pinctrl/pinconf-generic.h>

#include <asm/mach-jz4740/jz4740_fb.h>
#include <asm/mach-jz4740/platform.h>

/* Display */
static struct fb_videomode rs90_video_modes[] = {
	{
		.name = "240x160",
		/* Use 228 lines of 308 ticks each, like GBA. */
		.hsync_len = 1,
		.left_margin = 1,
		.right_margin = 66,
		.xres = 240,
		.vsync_len = 10,
		.upper_margin = 6,
		.lower_margin = 52,
		.yres = 160,
		.sync = FB_SYNC_HOR_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

static struct jz4740_fb_platform_data rs90_fb_pdata = {
	.width		= 42,
	.height		= 28,
	.num_modes	= ARRAY_SIZE(rs90_video_modes),
	.modes		= rs90_video_modes,
	.bpp		= 16,
	.lcd_type	= JZ_LCD_TYPE_SPECIAL_TFT_2,
	.pixclk_falling_edge = 1,
	.special_tft_config = {
		.spl = (0 << 16) | 1, // ???
		.cls = (8 << 16) | 168, // ???
		.ps = (0 << 16) | 2, // ???
		.rev = (2 << 16), // ???
	},
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4740_framebuffer_device,
};

static struct pinctrl_map pin_map[] __initdata = {
	/* fbdev pin configuration */
	PIN_MAP_MUX_GROUP_DEFAULT("jz4740-fb",
			"10010000.pin-controller", "lcd-8bit", "lcd"),
	PIN_MAP_MUX_GROUP_DEFAULT("jz4740-fb",
			"10010000.pin-controller", "lcd-16bit", "lcd"),
	PIN_MAP_MUX_GROUP_DEFAULT("jz4740-fb",
			"10010000.pin-controller", "lcd-special", "lcd"),
};

static int __init rs90_init_platform_devices(void)
{
	jz4740_framebuffer_device.dev.platform_data = &rs90_fb_pdata;

	pinctrl_register_mappings(pin_map, ARRAY_SIZE(pin_map));

	return platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));

}

static int __init rs90_board_setup(void)
{
	pr_warn("RS-90 platform setup\n");

	if (rs90_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}
arch_initcall(rs90_board_setup);

#include <linux/mmc/host.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <mach/jzmmc.h>
#include "ci20.h"

#define GPIO_WIFI_RST_N			GPIO_PF(7)
#define GPIO_WLAN_PW_EN			GPIO_PB(19)

#define KBYTE				(1024LL)
#define MBYTE				((KBYTE)*(KBYTE))
#define UINT32_MAX			(0xffffffffU)
#define RESET				0
#define NORMAL				1

static struct wifi_data			iw8101_data;

static DEFINE_SPINLOCK(wifi_enable_lock);
static int power_en_count;
static int clk_32k_count;

int iw8101_wlan_init(void);

#ifdef CONFIG_MMC0_JZ4780
/*
 * WARNING:
 * If a GPIO is not used or undefined, it must be set to -1
 * to prevent PA0 from being requested.
 */
static struct card_gpio ci20_tf_gpio = {
	.cd				= {GPIO_PF(20),		LOW_ENABLE},
	.wp				= {-1,			-1},
	.pwr				= {-1,			-1},
};

struct jzmmc_platform_data ci20_tf_pdata = {
	.removal  			= REMOVABLE,
	.sdio_clk			= 0,
	.ocr_avail			= MMC_VDD_32_33 | MMC_VDD_33_34,
	.capacity  			= MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.pm_flags			= 0,
	.max_freq			= CONFIG_MMC0_MAX_FREQ,
	.recovery_info			= NULL,
	.gpio				= &ci20_tf_gpio,
#ifdef CONFIG_MMC0_PIO_MODE
	.pio_mode			= 1,
#else
	.pio_mode			= 0,
#endif
	.private_init			= NULL,
};
#endif

#ifdef CONFIG_MMC1_JZ4780
struct jzmmc_platform_data ci20_sdio_pdata = {
	.removal  			= MANUAL,
	.sdio_clk			= 1,
	.ocr_avail			= MMC_VDD_32_33 | MMC_VDD_33_34,
	.capacity  			= MMC_CAP_4_BIT_DATA,
	.pm_flags			= MMC_PM_IGNORE_PM_NOTIFY,
	.max_freq			= CONFIG_MMC1_MAX_FREQ,
	.recovery_info			= NULL,
	.gpio				= NULL,
#ifdef CONFIG_MMC1_PIO_MODE
	.pio_mode			= 1,
#else
	.pio_mode			= 0,
#endif
	.private_init			= iw8101_wlan_init,
};
#endif

#define PXPIN		0x00   /* PIN Level Register */
#define PXINT		0x10   /* Port Interrupt Register */
#define PXINTS		0x14   /* Port Interrupt Set Register */
#define PXINTC		0x18   /* Port Interrupt Clear Register */
#define PXMSK		0x20   /* Port Interrupt Mask Reg */
#define PXMSKS		0x24   /* Port Interrupt Mask Set Reg */
#define PXMSKC		0x28   /* Port Interrupt Mask Clear Reg */
#define PXPAT1		0x30   /* Port Pattern 1 Set Reg. */
#define PXPAT1S		0x34   /* Port Pattern 1 Set Reg. */
#define PXPAT1C		0x38   /* Port Pattern 1 Clear Reg. */
#define PXPAT0		0x40   /* Port Pattern 0 Register */
#define PXPAT0S		0x44   /* Port Pattern 0 Set Register */
#define PXPAT0C		0x48   /* Port Pattern 0 Clear Register */
#define PXFLG		0x50   /* Port Flag Register */
#define PXFLGC		0x58   /* Port Flag clear Register */
#define PXOENS		0x64   /* Port Output Disable Set Register */
#define PXOENC		0x68   /* Port Output Disable Clear Register */
#define PXPEN		0x70   /* Port Pull Disable Register */
#define PXPENS		0x74   /* Port Pull Disable Set Register */
#define PXPENC		0x78   /* Port Pull Disable Clear Register */
#define PXDSS		0x84   /* Port Drive Strength set Register */
#define PXDSC		0x88   /* Port Drive Strength clear Register */

static unsigned int gpio_bakup[4];

void clk_32k_on(void)
{
	spin_lock(&wifi_enable_lock);

	if(clk_32k_count < 2) {
		if(clk_32k_count++ == 0)
			jzrtc_enable_clk32k();
	}

	spin_unlock(&wifi_enable_lock);
}

void clk_32k_off(void)
{
	spin_lock(&wifi_enable_lock);

	if(clk_32k_count > 0) {
		if(--clk_32k_count == 0)
			jzrtc_disable_clk32k();
	}

	spin_unlock(&wifi_enable_lock);
}

void wlan_pw_en_enable(void)
{
	spin_lock(&wifi_enable_lock);

	if(power_en_count < 2) {
		if(power_en_count++ == 0)
			gpio_set_value(GPIO_WLAN_PW_EN, 1);
	}

	spin_unlock(&wifi_enable_lock);
}

void wlan_pw_en_disable(void)
{
	spin_lock(&wifi_enable_lock);

	if(power_en_count > 0) {
		if(--power_en_count == 0)
			gpio_set_value(GPIO_WLAN_PW_EN, 0);
	}

	spin_unlock(&wifi_enable_lock);
}

int iw8101_wlan_init(void)
{
	static struct wake_lock	*wifi_wake_lock = &iw8101_data.wifi_wake_lock;
	struct regulator *power;
	int reset;

	gpio_bakup[0] = readl((void *)(0xb0010300 + PXINT)) & 0x1f00000;
	gpio_bakup[1] = readl((void *)(0xb0010300 + PXMSK)) & 0x1f00000;
	gpio_bakup[2] = readl((void *)(0xb0010300 + PXPAT1)) & 0x1f00000;
	gpio_bakup[3] = readl((void *)(0xb0010300 + PXPAT0)) & 0x1f00000;

	writel(0x1f00000, (void *)(0xb0010300 + PXINTC));
	writel(0x1f00000, (void *)(0xb0010300 + PXMSKS));
	writel(0x1f00000, (void *)(0xb0010300 + PXPAT1S));

	power = regulator_get(NULL, "vwifi");
	if (IS_ERR(power)) {
		pr_err("wifi regulator missing\n");
		return -EINVAL;
	}
	iw8101_data.wifi_power = power;

	reset = GPIO_WIFI_RST_N;
	if (gpio_request(GPIO_WIFI_RST_N, "wifi_reset")) {
		pr_err("no wifi_reset pin available\n");
		regulator_put(power);

		return -EINVAL;
	} else {
		gpio_direction_output(reset, 1);
	}
	 iw8101_data.wifi_reset = reset;

	if (gpio_request(GPIO_WLAN_PW_EN, "wlan_pw_en")) {
		pr_err("no wlan_pw_en pin available\n");
		regulator_put(power);
		return -EINVAL;
	} else {
		gpio_direction_output(GPIO_WLAN_PW_EN, 0);
	}

	wake_lock_init(wifi_wake_lock, WAKE_LOCK_SUSPEND, "wifi_wake_lock");

	return 0;
}

int IW8101_wlan_power_on(int flag)
{
	static struct wake_lock	*wifi_wake_lock = &iw8101_data.wifi_wake_lock;
	struct regulator *power = iw8101_data.wifi_power;
	int reset = iw8101_data.wifi_reset;

	if (wifi_wake_lock == NULL)
		pr_warn("%s: invalid wifi_wake_lock\n", __func__);
	else if (power == NULL)
		pr_warn("%s: invalid power\n", __func__);
	else if (!gpio_is_valid(reset))
		pr_warn("%s: invalid reset\n", __func__);
	else
		goto start;
	return -ENODEV;
start:
	pr_debug("wlan power on:%d\n", flag);

	writel(gpio_bakup[0] & 0x1f00000, (void *)(0xb0010300 + PXINTS));
	writel(~gpio_bakup[0] & 0x1f00000, (void *)(0xb0010300 + PXINTC));
	writel(gpio_bakup[1] & 0x1f00000, (void *)(0xb0010300 + PXMSKS));
	writel(~gpio_bakup[1] & 0x1f00000, (void *)(0xb0010300 + PXMSKC));
	writel(gpio_bakup[2] & 0x1f00000, (void *)(0xb0010300 + PXPAT1S));
	writel(~gpio_bakup[2] & 0x1f00000, (void *)(0xb0010300 + PXPAT1C));
	writel(gpio_bakup[3] & 0x1f00000, (void *)(0xb0010300 + PXPAT0S));
	writel(~gpio_bakup[3] & 0x1f00000, (void *)(0xb0010300 + PXPAT0C));

	clk_32k_on();
	msleep(200);

	switch(flag) {
		case RESET:
			wlan_pw_en_enable();
			regulator_enable(power);
			jzmmc_clk_ctrl(1, 1);

			gpio_set_value(reset, 0);
			msleep(200);

			gpio_set_value(reset, 1);
			msleep(200);

			break;

		case NORMAL:
			wlan_pw_en_enable();
			regulator_enable(power);

			gpio_set_value(reset, 0);
			msleep(200);

			gpio_set_value(reset, 1);

			msleep(200);
			jzmmc_manual_detect(1, 1);

			break;
	}
	wake_lock(wifi_wake_lock);

	return 0;
}

int IW8101_wlan_power_off(int flag)
{
	static struct wake_lock	*wifi_wake_lock = &iw8101_data.wifi_wake_lock;
	struct regulator *power = iw8101_data.wifi_power;
	int reset = iw8101_data.wifi_reset;

	if (wifi_wake_lock == NULL)
		pr_warn("%s: invalid wifi_wake_lock\n", __func__);
	else if (power == NULL)
		pr_warn("%s: invalid power\n", __func__);
	else if (!gpio_is_valid(reset))
		pr_warn("%s: invalid reset\n", __func__);
	else
		goto start;
	return -ENODEV;
start:
	pr_debug("wlan power off:%d\n", flag);
	switch(flag) {
		case RESET:
			gpio_set_value(reset, 0);

			regulator_disable(power);
			wlan_pw_en_disable();
			jzmmc_clk_ctrl(1, 0);
			break;

		case NORMAL:
			gpio_set_value(reset, 0);

			regulator_disable(power);
			wlan_pw_en_disable();

 			jzmmc_manual_detect(1, 0);
			break;
	}

	wake_unlock(wifi_wake_lock);

	clk_32k_off();

	gpio_bakup[0] = (unsigned int)readl((void *)(0xb0010300 + PXINT)) & 0x1f00000;
	gpio_bakup[1] = (unsigned int)readl((void *)(0xb0010300 + PXMSK)) & 0x1f00000;
	gpio_bakup[2] = (unsigned int)readl((void *)(0xb0010300 + PXPAT1)) & 0x1f00000;
	gpio_bakup[3] = (unsigned int)readl((void *)(0xb0010300 + PXPAT0)) & 0x1f00000;

	writel(0x1f00000, (void *)(0xb0010300 + PXINTC));
	writel(0x1f00000, (void *)(0xb0010300 + PXMSKS));
	writel(0x1f00000, (void *)(0xb0010300 + PXPAT1S));

	return 0;
}

EXPORT_SYMBOL(wlan_pw_en_enable);
EXPORT_SYMBOL(wlan_pw_en_disable);
EXPORT_SYMBOL(clk_32k_on);
EXPORT_SYMBOL(clk_32k_off);
EXPORT_SYMBOL(IW8101_wlan_power_on);
EXPORT_SYMBOL(IW8101_wlan_power_off);

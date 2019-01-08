// SPDX-License-Identifier: GPL-2.0
/*
 * ADC driver for the Ingenic JZ47xx SoCs
 * Copyright (c) 2019 Artur Rojek <contact@artur-rojek.eu>
 *
 * based on drivers/mfd/jz4740-adc.c
 */

#include <linux/clk.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define JZ_ADC_REG_ENABLE	0x00
#define JZ_ADC_REG_CFG		0x04
#define JZ_ADC_REG_CTRL		0x08
#define JZ_ADC_REG_STATUS	0x0c
#define JZ_ADC_REG_ADTCH	0x18
#define JZ_ADC_REG_ADBDAT	0x1c
#define JZ_ADC_REG_ADSDAT	0x20

#define REG_CFG_BAT_MD		BIT(4)

#define ADC_NB_BITS		12
#define ADC_NB_BITS_MD0		10
#define ADSDAT_SCALE_MUL	3300
#define ADBDAT_SCALE_MUL_MD0	7500
#define ADBDAT_SCALE_MUL_MD1	2500

struct ingenic_adc {
	void __iomem *base;
	struct clk *clk;
	spinlock_t lock;
	bool low_vref_mode;
};

static int ingenic_adc_set_config(struct ingenic_adc *adc,
				  uint32_t mask,
				  uint32_t val)
{
	unsigned long flags;
	uint32_t cfg;

	clk_enable(adc->clk);
	spin_lock_irqsave(&adc->lock, flags);

	cfg = readl(adc->base + JZ_ADC_REG_CFG) & ~mask;
	cfg |= val;
	writel(cfg, adc->base + JZ_ADC_REG_CFG);

	spin_unlock_irqrestore(&adc->lock, flags);
	clk_disable(adc->clk);


	return 0;
}

static inline int ingenic_adc_enable(struct ingenic_adc *adc,
				     int engine,
				     bool enabled)
{
	unsigned long flags;
	u8 val;
	int ret = 0;

	spin_lock_irqsave(&adc->lock, flags);
	val = readb(adc->base + JZ_ADC_REG_ENABLE);

	if (enabled)
		val |= BIT(engine);
	else
		val &= ~BIT(engine);

	writeb(val, adc->base + JZ_ADC_REG_ENABLE);
	spin_unlock_irqrestore(&adc->lock, flags);

	if (enabled)
		ret = readb_poll_timeout(adc->base + JZ_ADC_REG_ENABLE, val,
					 !(val & BIT(engine)), 250, 1000);

	return ret;
}

static int ingenic_adc_write_raw(struct iio_dev *iio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long m)
{
	struct ingenic_adc *adc = iio_priv(iio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel) {
		case 1:
			if (val > ADBDAT_SCALE_MUL_MD1) {
				ingenic_adc_set_config(adc,
						       REG_CFG_BAT_MD,
						       0);
				adc->low_vref_mode = false;
			}
			else {
				ingenic_adc_set_config(adc,
						       REG_CFG_BAT_MD,
						       REG_CFG_BAT_MD);
				adc->low_vref_mode = true;
			}
			return 0;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static const int ingenic_adc_battery_raw_avail[] = {
	0, 1, (1 << ADC_NB_BITS) - 1,
};

static const int ingenic_adc_battery_scale_avail[] = {
	ADBDAT_SCALE_MUL_MD0, ADC_NB_BITS_MD0,
	ADBDAT_SCALE_MUL_MD1, ADC_NB_BITS,
};

static int ingenic_adc_read_avail(struct iio_dev *iio_dev,
				  struct iio_chan_spec const *chan,
				  const int **vals,
				  int *type,
				  int *length,
				  long m)
{
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(ingenic_adc_battery_raw_avail);
		*vals = ingenic_adc_battery_raw_avail;
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_SCALE:
		*type = IIO_VAL_FRACTIONAL_LOG2;
		*length = ARRAY_SIZE(ingenic_adc_battery_scale_avail);
		*vals = ingenic_adc_battery_scale_avail;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	};
}

static int ingenic_adc_read_raw(struct iio_dev *iio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long m)
{
	struct ingenic_adc *adc = iio_priv(iio_dev);
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		clk_enable(adc->clk);
		ret = ingenic_adc_enable(adc, chan->channel, true);
		if (ret) {
			ingenic_adc_enable(adc, chan->channel, false);
			clk_disable(adc->clk);
			return ret;
		}

		switch (chan->channel) {
		case 0:
			*val = readw(adc->base + JZ_ADC_REG_ADSDAT);
			break;
		case 1:
			*val = readw(adc->base + JZ_ADC_REG_ADBDAT);
			break;
		}

		clk_disable(adc->clk);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel) {
		case 0:
			*val = ADSDAT_SCALE_MUL;
			*val2 = ADC_NB_BITS;
			break;
		case 1:
			if (adc->low_vref_mode) {
				*val = ADBDAT_SCALE_MUL_MD1;
				*val2 = ADC_NB_BITS;
			} else {
				*val = ADBDAT_SCALE_MUL_MD0;
				*val2 = ADC_NB_BITS_MD0;
			}
			break;
		}

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ingenic_adc_info = {
	.write_raw = ingenic_adc_write_raw,
	.read_raw = ingenic_adc_read_raw,
	.read_avail = ingenic_adc_read_avail,
};

static const struct iio_chan_spec ingenic_channels[] = {
	{
		.extend_name = "aux",
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.indexed = 1,
		.channel = 0,
	},
	{
		.extend_name = "battery",
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_RAW) |
						BIT(IIO_CHAN_INFO_SCALE),
		.indexed = 1,
		.channel = 1,
	},
};

static int ingenic_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *iio_dev;
	struct ingenic_adc *adc;
	struct resource *mem_base;
	int ret;

	iio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
	if (!iio_dev)
		return -ENOMEM;

	adc = iio_priv(iio_dev);
	spin_lock_init(&adc->lock);

	mem_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	adc->base = devm_ioremap_resource(&pdev->dev, mem_base);
	if (IS_ERR(adc->base)) {
		dev_err(&pdev->dev, "Unable to ioremap mmio resource\n");
		return PTR_ERR(adc->base);
	}

	adc->clk = devm_clk_get(&pdev->dev, "adc");
	if (IS_ERR(adc->clk)) {
		dev_err(&pdev->dev, "Unable to get clock\n");
		return PTR_ERR(adc->clk);
	}

	ret = clk_prepare_enable(adc->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		return ret;
	}

	/* Put hardware in a known passive state. */
	writeb(0x00, adc->base + JZ_ADC_REG_ENABLE);
	writeb(0xff, adc->base + JZ_ADC_REG_CTRL);
	clk_disable(adc->clk);

	devm_add_action(&pdev->dev, (void (*)(void *))clk_unprepare, adc->clk);

	iio_dev->dev.parent = &pdev->dev;
	iio_dev->name = "adc";
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->channels = ingenic_channels;
	iio_dev->num_channels = ARRAY_SIZE(ingenic_channels);
	iio_dev->info = &ingenic_adc_info;

	ret = devm_iio_device_register(&pdev->dev, iio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register IIO device\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ingenic_adc_of_match[] = {
	{ .compatible = "ingenic,jz4725b-adc", },
	{ },
};
MODULE_DEVICE_TABLE(of, ingenic_adc_of_match);
#endif

static struct platform_driver ingenic_adc_driver = {
	.driver = {
		.name = "ingenic-adc",
		.of_match_table = of_match_ptr(ingenic_adc_of_match),
	},
	.probe = ingenic_adc_probe,
};
module_platform_driver(ingenic_adc_driver);

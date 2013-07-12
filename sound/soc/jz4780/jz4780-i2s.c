/*
 * Ingenic JZ4780 ASoC I2S driver
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "jz4780-aic.h"

struct jz4780_i2s {
	struct resource *mem;
	void __iomem *base;

	unsigned int irq;

	struct clk *clk_aic;
	struct clk *clk_i2s;

	struct dma_slave_config dma_tx_config;
	struct dma_slave_config dma_rx_config;
};

static uint32_t i2s_readl(struct jz4780_i2s *i2s, uint32_t off)
{
	return readl(i2s->base + off);
}

static void i2s_writel(struct jz4780_i2s *i2s, uint32_t val, uint32_t off)
{
	writel(val, i2s->base + off);
}

static int jz4780_i2s_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	uint32_t conf, ctrl;

	if (dai->active)
		return 0;

	ctrl = i2s_readl(i2s, AIC_AICCR);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ctrl |= AICCR_TFLUSH;
	else
		ctrl |= AICCR_RFLUSH;
	i2s_writel(i2s, ctrl, AIC_AICCR);

	clk_enable(i2s->clk_i2s);

	conf = i2s_readl(i2s, AIC_AICFR);
	conf |= AICFR_ENB;
	i2s_writel(i2s, conf, AIC_AICFR);

	return 0;
}

static void jz4780_i2s_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	uint32_t conf;

	if (dai->active)
		return;

	conf = i2s_readl(i2s, AIC_AICFR);
	conf &= ~AICFR_ENB;
	i2s_writel(i2s, conf, AIC_AICFR);

	clk_disable(i2s->clk_i2s);
}

static int jz4780_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	uint32_t ctrl, mask;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		mask = AICCR_ERPL | AICCR_TDMS;
	else
		mask = AICCR_EREC | AICCR_RDMS;

	ctrl = i2s_readl(i2s, AIC_AICCR);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ctrl |= mask;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ctrl &= ~mask;
		break;
	default:
		return -EINVAL;
	}

	i2s_writel(i2s, ctrl, AIC_AICCR);

	return 0;
}

static int jz4780_i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	uint32_t ctrl, div_reg, sample_size, i2s_cfg;
	enum dma_slave_buswidth dma_width;
	struct dma_slave_config *dma_config;
	int div;

	i2s_cfg = i2s_readl(i2s, AIC_I2SCR);
	i2s_writel(i2s, i2s_cfg | I2SCR_STPBK | I2SCR_ISTPBK, AIC_I2SCR);

	div_reg = i2s_readl(i2s, AIC_I2SDIV);
	div = clk_get_rate(i2s->clk_i2s) / (64 * params_rate(params));

	ctrl = i2s_readl(i2s, AIC_AICCR);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		sample_size = 0;
		dma_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case SNDRV_PCM_FORMAT_S16:
		sample_size = 1;
		dma_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			ctrl |= AICCR_PACK16;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		sample_size = 2;
		dma_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		sample_size = 3;
		dma_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sample_size = 4;
		dma_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ctrl &= ~AICCR_OSS_MASK;
		ctrl |= sample_size << AICCR_OSS_SHIFT;

		ctrl &= ~(AICCR_M2S | AICCR_CHANNEL_MASK);
		switch (params_channels(params)) {
		case 1:
			ctrl |= AICCR_M2S;
			break;
		case 2:
		case 4:
		case 6:
		case 8:
			ctrl |= (params_channels(params) - 1) <<
				AICCR_CHANNEL_SHIFT;
			break;
		default:
			return -EINVAL;
		}

		dma_config = &i2s->dma_tx_config;

		div_reg &= ~I2SDIV_DV_MASK;
		div_reg |= (div - 1) << I2SDIV_DV_SHIFT;
	} else {
		ctrl &= ~AICCR_ISS_MASK;
		ctrl |= sample_size << AICCR_ISS_SHIFT;

		dma_config = &i2s->dma_rx_config;

		div_reg &= ~I2SDIV_IDV_MASK;
		div_reg |= (div - 1) << I2SDIV_IDV_SHIFT;
	}

	i2s_writel(i2s, ctrl, AIC_AICCR);
	i2s_writel(i2s, div_reg, AIC_I2SDIV);
	i2s_writel(i2s, i2s_cfg, AIC_I2SCR);

	dma_config->src_addr_width = dma_width;
	dma_config->dst_addr_width = dma_width;
	snd_soc_dai_set_dma_data(dai, substream, dma_config);

	return 0;
}

static int jz4780_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	uint32_t cfg, i2s_cfg = 0;

	/* stop bit clocks */
	i2s_cfg = i2s_readl(i2s, AIC_I2SCR);
	i2s_writel(i2s, i2s_cfg | I2SCR_STPBK | I2SCR_ISTPBK, AIC_I2SCR);
	i2s_cfg &= ~(I2SCR_STPBK | I2SCR_ISTPBK);

	cfg = i2s_readl(i2s, AIC_AICFR);
	cfg &= ~(AICFR_BCKD | AICFR_SYNCD | AICFR_ICDC);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		cfg |= AICFR_BCKD | AICFR_SYNCD;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		cfg |= AICFR_SYNCD;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		cfg |= AICFR_BCKD;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		cfg |= AICFR_ICDC;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_MSB:
		i2s_cfg |= I2SCR_AMSL;
		break;
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	i2s_writel(i2s, cfg, AIC_AICFR);
	i2s_writel(i2s, i2s_cfg, AIC_I2SCR);

	return 0;
}

static int jz4780_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int freq, int dir)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	clk_set_rate(i2s->clk_i2s, freq);
	return 0;
}

static int jz4780_i2s_suspend(struct snd_soc_dai *dai)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	uint32_t conf;

	if (dai->active) {
		conf = i2s_readl(i2s, AIC_AICFR);
		conf &= ~AICFR_ENB;
		i2s_writel(i2s, conf, AIC_AICFR);

		clk_disable(i2s->clk_i2s);
	}

	clk_disable(i2s->clk_aic);

	return 0;
}

static int jz4780_i2s_resume(struct snd_soc_dai *dai)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	uint32_t conf;

	clk_enable(i2s->clk_aic);

	if (dai->active) {
		clk_enable(i2s->clk_i2s);

		conf = i2s_readl(i2s, AIC_AICFR);
		conf |= AICFR_ENB;
		i2s_writel(i2s, conf, AIC_AICFR);
	}

	return 0;
}

static int jz4780_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	uint32_t reg;

	clk_enable(i2s->clk_aic);

	/* stop bit clocks */
	i2s_writel(i2s, I2SCR_STPBK | I2SCR_ISTPBK, AIC_I2SCR);

	/* use internal codec in I2S mode */
	reg = AICFR_ICDC | AICFR_AUSEL | AICFR_DMODE;
	reg |= 7 << AICFR_RFTH_SHIFT;
	reg |= 7 << AICFR_TFTH_SHIFT;
	i2s_writel(i2s, reg, AIC_AICFR);

	/* I2S operation from SYS_CLK */
	reg = i2s_readl(i2s, AIC_I2SCR);
	reg |= I2SCR_ESCLK;
	i2s_writel(i2s, reg, AIC_I2SCR);

	/* enable codec sysclk */
	clk_set_rate(i2s->clk_i2s, 12000000);
	clk_enable(i2s->clk_i2s);

	/* start bit clocks */
	reg = i2s_readl(i2s, AIC_I2SCR);
	reg &= ~(I2SCR_STPBK | I2SCR_ISTPBK);
	i2s_writel(i2s, reg, AIC_I2SCR);

	/* clear FIFO errors & requests */
	i2s_writel(i2s, 0, AIC_AICSR);

	/* enable */
	reg = i2s_readl(i2s, AIC_AICFR);
	reg |= AICFR_ENB;
	i2s_writel(i2s, reg, AIC_AICFR);

	return 0;
}

static int jz4780_i2s_dai_remove(struct snd_soc_dai *dai)
{
	struct jz4780_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	clk_disable(i2s->clk_aic);

	return 0;
}

static struct snd_soc_dai_ops jz4780_i2s_dai_ops = {
	.startup = jz4780_i2s_startup,
	.shutdown = jz4780_i2s_shutdown,
	.trigger = jz4780_i2s_trigger,
	.hw_params = jz4780_i2s_hw_params,
	.set_fmt = jz4780_i2s_set_fmt,
	.set_sysclk = jz4780_i2s_set_sysclk,
};

static struct snd_soc_dai_driver jz4780_i2s_dai = {
	.probe = jz4780_i2s_dai_probe,
	.remove = jz4780_i2s_dai_remove,
	.playback = {
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
			SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S18_3LE |
			SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
			SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S18_3LE |
			SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &jz4780_i2s_dai_ops,
	.suspend = jz4780_i2s_suspend,
	.resume = jz4780_i2s_resume,
};

static int __devinit jz4780_i2s_dev_probe(struct platform_device *pdev)
{
	struct jz4780_i2s *i2s;
	int err;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2s);

	i2s->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!i2s->mem)
		return -ENOENT;

	i2s->base = devm_ioremap(&pdev->dev, i2s->mem->start,
			resource_size(i2s->mem));
	if (!i2s->base)
		return -EBUSY;

	i2s->clk_aic = clk_get(&pdev->dev, "aic0");
	if (IS_ERR(i2s->clk_aic)) {
		err = PTR_ERR(i2s->clk_aic);
		goto out_err_aic;
	}

	i2s->clk_i2s = clk_get(&pdev->dev, "cgu_aic");
	if (IS_ERR(i2s->clk_i2s)) {
		err = PTR_ERR(i2s->clk_i2s);
		goto out_err_i2s;
	}

	/* output DMA configuration */
	i2s->dma_tx_config.direction = DMA_TO_DEVICE;
	i2s->dma_tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	i2s->dma_tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	i2s->dma_tx_config.src_maxburst = 64;
	i2s->dma_tx_config.dst_maxburst = 64;
	i2s->dma_tx_config.dst_addr = i2s->mem->start + AIC_AICDR;
	i2s->dma_tx_config.src_addr = 0;

	/* input DMA configuration */
	i2s->dma_rx_config.direction = DMA_FROM_DEVICE;
	i2s->dma_rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	i2s->dma_rx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	i2s->dma_rx_config.src_maxburst = 32;
	i2s->dma_rx_config.dst_maxburst = 32;
	i2s->dma_rx_config.dst_addr = 0;
	i2s->dma_rx_config.src_addr = i2s->mem->start + AIC_AICDR;

	err = snd_soc_register_dai(&pdev->dev, &jz4780_i2s_dai);
	if (err) {
		dev_err(&pdev->dev, "Failed to register DAI\n");
		goto out_err_dai;
	}

	return 0;
out_err_dai:
	clk_put(i2s->clk_i2s);
out_err_i2s:
	clk_put(i2s->clk_aic);
out_err_aic:
	return err;
}

static int __devexit jz4780_i2s_dev_remove(struct platform_device *pdev)
{
	struct jz4780_i2s *i2s = platform_get_drvdata(pdev);

	free_irq(i2s->irq, NULL);
	snd_soc_unregister_dai(&pdev->dev);
	clk_put(i2s->clk_aic);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver jz4780_i2s_driver = {
	.probe = jz4780_i2s_dev_probe,
	.remove = __devexit_p(jz4780_i2s_dev_remove),
	.driver = {
		.name = "jz4780-i2s",
		.owner = THIS_MODULE,
	},
};

static int __init jz4780_i2s_init(void)
{
	return platform_driver_register(&jz4780_i2s_driver);
}
module_init(jz4780_i2s_init);

static void __exit jz4780_i2s_exit(void)
{
	platform_driver_unregister(&jz4780_i2s_driver);
}
module_exit(jz4780_i2s_exit);

MODULE_AUTHOR("Paul Burton <paul.burton@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 I2S driver");
MODULE_LICENSE("GPL");

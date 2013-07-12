/*
 * Ingenic JZ4780 ASoC codec driver
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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

enum {
	REG_SR		= 0x00,
	REG_SR2,
	REG_MR		= 0x07,
	REG_AICR_DAC,
	REG_AICR_ADC,
	REG_CR_LO	= 0x0b,
	REG_CR_HP	= 0x0d,
	REG_CR_DMIC	= 0x10,
	REG_CR_MIC1,
	REG_CR_MIC2,
	REG_CR_LI1,
	REG_CR_LI2,
	REG_CR_DAC	= 0x17,
	REG_CR_ADC,
	REG_CR_MIX,
	REG_DR_MIX,
	REG_CR_VIC,
	REG_CR_CK,
	REG_FCR_DAC,
	REG_FCR_ADC	= 0x20,
	REG_CR_TIMER_MSB,
	REG_CR_TIMER_LSB,
	REG_ICR,
	REG_IMR,
	REG_IFR,
	REG_IMR2,
	REG_IFR2,
	REG_GCR_HPL,
	REG_GCR_HPR,
	REG_GCR_LIBYL,
	REG_GCR_LIBYR,
	REG_GCR_DACL,
	REG_GCR_DACR,
	REG_GCR_MIC1,
	REG_GCR_MIC2,
	REG_GCR_ADCL,
	REG_GCR_ADCR,
	REG_GCR_MIXDACL	= 0x34,
	REG_GCR_MIXDACR,
	REG_GCR_MIXADCL,
	REG_GCR_MIXADCR,
	REG_CR_ADC_AGC	= 0x3a,
	REG_DR_ADC_AGC,

	REG_MIXER	= 0x100,
	REG_MIX_0	= REG_MIXER | 0x0,
	REG_MIX_1	= REG_MIXER | 0x1,
	REG_MIX_2	= REG_MIXER | 0x2,
	REG_MIX_3	= REG_MIXER | 0x3,
};

static const uint8_t jz4780_codec_reg_defaults[0x40] = {
	[REG_AICR_DAC]	= 0xd3,
	[REG_AICR_ADC]	= 0xd3,
	[REG_CR_LO]	= 0x90,
	[REG_CR_HP]	= 0x90,
	[REG_CR_MIC1]	= 0xb0,
	[REG_CR_MIC2]	= 0x30,
	[REG_CR_LI1]	= 0x10,
	[REG_CR_LI2]	= 0x10,
	[REG_CR_DAC]	= 0x90,
	[REG_CR_ADC]	= 0x90,
	[REG_CR_VIC]	= 0x03,
	[REG_IMR]	= 0xff,
	[REG_IMR2]	= 0xff,
	[REG_GCR_HPL]	= 0x06,
	[REG_GCR_HPR]	= 0x06,
	[REG_GCR_LIBYL]	= 0x06,
	[REG_GCR_LIBYR]	= 0x06,
};

struct jz4780_codec {
	void __iomem *base;
	struct resource *mem;
	struct clk *clk;
};

static unsigned int jz4780_codec_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	struct jz4780_codec *jzc = snd_soc_codec_get_drvdata(codec);
	uint32_t ret;

	clk_enable(jzc->clk);

	if (reg & REG_MIXER) {
		snd_soc_update_bits(codec, REG_CR_MIX, 0x43, reg & 0x3);
		ret = jz4780_codec_read(codec, REG_DR_MIX);
	} else {
		writel(reg << 8, jzc->base);
		ret = readl(jzc->base + 4) & 0xff;
	}

	clk_disable(jzc->clk);
	return ret;
}

static int jz4780_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	struct jz4780_codec *jzc = snd_soc_codec_get_drvdata(codec);
	uint8_t *cache = codec->reg_cache;

	clk_enable(jzc->clk);

	cache[reg] = val;

	if (reg & REG_MIXER) {
		jz4780_codec_write(codec, REG_DR_MIX, val);
		snd_soc_update_bits(codec, REG_CR_MIX, 0x43,
			0x40 | (reg & 0x3));
	} else {
		writel((1 << 16) | (reg << 8) | (val & 0xff), jzc->base);
		while (readl(jzc->base) & (1 << 16))
			;
	}

	clk_disable(jzc->clk);
	return 0;
}

static const char * const mic1_input_mux_text[] = {
	"AIP1", "AIP2",
};

static const struct soc_enum mic1_input_enum =
	SOC_ENUM_SINGLE(REG_CR_MIC1, 0, 2, mic1_input_mux_text);

static const struct snd_kcontrol_new mic1_input_mux =
	SOC_DAPM_ENUM("Mic Input Mux", mic1_input_enum);

static const struct snd_kcontrol_new jz4780_codec_controls[] = {
	SOC_DOUBLE_R("Master Capture Volume", REG_GCR_ADCL, REG_GCR_ADCR,
		0, 63, 0),

	SOC_SINGLE("Mic Volume", REG_GCR_MIC1, 0, 7, 0),
	SOC_SINGLE("Mic Mute", REG_CR_ADC, 7, 1, 0),

	SOC_DOUBLE_R("Master Playback Volume", REG_GCR_DACL, REG_GCR_DACR,
		0, 31, 1),

	SOC_DOUBLE_R("Headphone Volume", REG_GCR_HPL, REG_GCR_HPR, 0, 31, 1),
	SOC_SINGLE("Headphone Mute", REG_CR_HP, 7, 1, 0),
};

static const struct snd_kcontrol_new jz4780_codec_input_controls[] = {
	SOC_DAPM_SINGLE("Mic Capture Switch", REG_CR_MIC1, 4, 1, 1),
};

static const struct snd_kcontrol_new jz4780_codec_output_controls[] = {
	SOC_DAPM_SINGLE("DAC Switch", REG_CR_DAC, 4, 1, 1),
};

static const struct snd_soc_dapm_widget jz4780_codec_dapm_widgets[] = {
	SND_SOC_DAPM_ADC("ADC", "Capture", REG_AICR_ADC, 4, 1),
	SND_SOC_DAPM_DAC("DAC", "Playback", REG_AICR_DAC, 4, 1),

	SND_SOC_DAPM_MUX("Mic Input Mux", SND_SOC_NOPM, 0, 0, &mic1_input_mux),

	SND_SOC_DAPM_MIXER_NAMED_CTL("Input Mixer", REG_CR_ADC, 4, 1,
			jz4780_codec_input_controls,
			ARRAY_SIZE(jz4780_codec_input_controls)),

	SND_SOC_DAPM_MIXER("Output Mixer", REG_CR_HP, 4, 1,
			jz4780_codec_output_controls,
			ARRAY_SIZE(jz4780_codec_output_controls)),

	SND_SOC_DAPM_MICBIAS("Mic Bias", REG_CR_MIC1, 5, 1),

	SND_SOC_DAPM_INPUT("AIP1"),
	SND_SOC_DAPM_INPUT("AIP2"),
	SND_SOC_DAPM_INPUT("AIP3"),

	SND_SOC_DAPM_OUTPUT("AOHPL"),
	SND_SOC_DAPM_OUTPUT("AOHPR"),
};

static const struct snd_soc_dapm_route jz4780_codec_dapm_routes[] = {
	{"ADC", NULL, "Input Mixer"},

	{"Input Mixer", "Mic Capture Switch", "Mic Input Mux"},

	{"Mic Input Mux", "AIP1", "AIP1"},
	{"Mic Input Mux", "AIP2", "AIP2"},

	{"Output Mixer", "DAC Switch", "DAC"},

	{"AOHPL", NULL, "Output Mixer"},
	{"AOHPR", NULL, "Output Mixer"},
};

static int jz4780_codec_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	uint32_t val;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	switch (params_rate(params)) {
	case 8000:
		val = 0;
		break;
	case 11025:
		val = 1;
		break;
	case 12000:
		val = 2;
		break;
	case 16000:
		val = 3;
		break;
	case 22050:
		val = 4;
		break;
	case 24000:
		val = 5;
		break;
	case 32000:
		val = 6;
		break;
	case 44100:
		val = 7;
		break;
	case 48000:
		val = 8;
		break;
	case 88200:
		val = 9;
		break;
	case 96000:
		val = 10;
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_update_bits(codec, REG_FCR_DAC, 0xf, val);
	else
		snd_soc_update_bits(codec, REG_FCR_ADC, 0xf, val);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16:
		val = 0;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		val = 1;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val = 2;
		break;
	case SNDRV_PCM_FORMAT_S24:
		val = 3;
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_update_bits(codec, REG_AICR_DAC, 0xc0, val << 6);
	else {
		snd_soc_update_bits(codec, REG_AICR_ADC, 0xc0, val << 6);

		snd_soc_update_bits(codec, REG_CR_MIC1, 0x80, 0x00);
		snd_soc_update_bits(codec, REG_CR_ADC, 0x20, 0x20);
	}

	return 0;
}

static int jz4780_codec_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	snd_soc_update_bits(codec, REG_CR_DAC, 0x80, (!!mute) << 7);
	return 0;
}

static struct snd_soc_dai_ops jz4780_codec_dai_ops = {
	.hw_params = jz4780_codec_hw_params,
	.digital_mute = jz4780_codec_mute,
};

static struct snd_soc_dai_driver jz4780_codec_dai = {
	.name = "jz4780-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S18_3LE |
			SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S18_3LE |
			SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &jz4780_codec_dai_ops,
};

static int jz4780_codec_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		/* power-up */
		snd_soc_update_bits(codec, REG_CR_VIC, 0x01, 0x00);
		mdelay(250);
		snd_soc_update_bits(codec, REG_CR_VIC, 0x02, 0x00);
		mdelay(400);
		break;
	case SND_SOC_BIAS_STANDBY:
		snd_soc_update_bits(codec, REG_CR_VIC, 0x02, 0x02);
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_update_bits(codec, REG_CR_VIC, 0x01, 0x01);
		break;
	default:
		break;
	}

	codec->dapm.bias_level = level;

	return 0;
}

static int jz4780_codec_dev_probe(struct snd_soc_codec *codec)
{
	struct jz4780_codec *jzc = snd_soc_codec_get_drvdata(codec);

	/* reset */
	writel(1 << 31, jzc->base);
	udelay(2);
	writel(0, jzc->base);

	jz4780_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* select I2S */
	snd_soc_update_bits(codec, REG_AICR_DAC, 0x03, 0x03);
	snd_soc_update_bits(codec, REG_AICR_ADC, 0x03, 0x03);

	/* select AIP2 input */
	snd_soc_update_bits(codec, REG_CR_MIC1, 0x01, 0x01);
	snd_soc_update_bits(codec, REG_CR_ADC, 0x01, 0x00);

	/* disable mixer */
	snd_soc_update_bits(codec, REG_CR_MIX, 0x80, 0x00);

	return 0;
}

static int jz4780_codec_dev_remove(struct snd_soc_codec *codec)
{
	jz4780_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int jz4780_codec_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	return jz4780_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static int jz4780_codec_resume(struct snd_soc_codec *codec)
{
	return jz4780_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
}

#else
#define jz4780_codec_suspend NULL
#define jz4780_codec_resume NULL
#endif

static struct snd_soc_codec_driver soc_codec_dev_jz4780_codec = {
	.probe = jz4780_codec_dev_probe,
	.remove = jz4780_codec_dev_remove,
	.suspend = jz4780_codec_suspend,
	.resume = jz4780_codec_resume,
	.read = jz4780_codec_read,
	.write = jz4780_codec_write,
	.set_bias_level = jz4780_codec_set_bias_level,
	.reg_cache_default = jz4780_codec_reg_defaults,
	.reg_word_size = sizeof(uint8_t),
	.reg_cache_size	= 0x40,

	.controls = jz4780_codec_controls,
	.num_controls = ARRAY_SIZE(jz4780_codec_controls),
	.dapm_widgets = jz4780_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(jz4780_codec_dapm_widgets),
	.dapm_routes = jz4780_codec_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(jz4780_codec_dapm_routes),
};

static int __devinit jz4780_codec_probe(struct platform_device *pdev)
{
	int ret;
	struct jz4780_codec *jz4780_codec;
	struct resource *mem;

	jz4780_codec = kzalloc(sizeof(*jz4780_codec), GFP_KERNEL);
	if (!jz4780_codec)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "Failed to get mmio memory resource\n");
		ret = -ENOENT;
		goto err_free_codec;
	}

	jz4780_codec->base = ioremap(mem->start, resource_size(mem));
	if (!jz4780_codec->base) {
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		ret = -EBUSY;
		goto err_release_mem_region;
	}
	jz4780_codec->mem = mem;

	jz4780_codec->clk = clk_get(&pdev->dev, "cgu_aic");
	if (IS_ERR(jz4780_codec->clk)) {
		ret = PTR_ERR(jz4780_codec->clk);
		goto out_err_clk;
	}

	platform_set_drvdata(pdev, jz4780_codec);

	ret = snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_jz4780_codec, &jz4780_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register codec\n");
		goto out_err_register;
	}

	return 0;

out_err_register:
	clk_put(jz4780_codec->clk);
out_err_clk:
	iounmap(jz4780_codec->base);
err_release_mem_region:
	release_mem_region(mem->start, resource_size(mem));
err_free_codec:
	kfree(jz4780_codec);

	return ret;
}

static int __devexit jz4780_codec_remove(struct platform_device *pdev)
{
	struct jz4780_codec *jz4780_codec = platform_get_drvdata(pdev);
	struct resource *mem = jz4780_codec->mem;

	snd_soc_unregister_codec(&pdev->dev);

	iounmap(jz4780_codec->base);
	release_mem_region(mem->start, resource_size(mem));

	clk_put(jz4780_codec->clk);

	platform_set_drvdata(pdev, NULL);
	kfree(jz4780_codec);

	return 0;
}

static struct platform_driver jz4780_codec_driver = {
	.probe = jz4780_codec_probe,
	.remove = __devexit_p(jz4780_codec_remove),
	.driver = {
		.name = "jz4780-codec",
		.owner = THIS_MODULE,
	},
};

static int __init jz4780_codec_init(void)
{
	return platform_driver_register(&jz4780_codec_driver);
}
module_init(jz4780_codec_init);

static void __exit jz4780_codec_exit(void)
{
	platform_driver_unregister(&jz4780_codec_driver);
}
module_exit(jz4780_codec_exit);

MODULE_AUTHOR("Paul Burton <paul.burton@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 ASoC codec driver");
MODULE_LICENSE("GPL");

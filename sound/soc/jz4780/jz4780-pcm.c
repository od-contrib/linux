/*
 * Ingenic JZ4780 ASoC PCM driver
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
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/jzdma.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

struct jz4780_runtime_data {
	unsigned long period_bytes;
	unsigned long offset;
	unsigned long size;
	int periods;

	struct dma_chan *dma;
	struct dma_async_tx_descriptor *desc;
};

/* identify hardware playback capabilities */
static const struct snd_pcm_hardware jz4780_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S8,

	.rates			= SNDRV_PCM_RATE_8000_96000,
	.channels_min		= 1,
	.channels_max		= 8,
	.period_bytes_min	= 16,
	.period_bytes_max	= 2 * PAGE_SIZE,
	.periods_min		= 2,
	.periods_max		= 127, /* limitation of dma driver */
	.buffer_bytes_max	= 127 * 2 * PAGE_SIZE,
	.fifo_size		= 32,
};

static void jz4780_pcm_dma_irq(void *data)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4780_runtime_data *prtd = runtime->private_data;

	prtd->offset += prtd->period_bytes;
	prtd->offset %= prtd->period_bytes * prtd->periods;

	snd_pcm_period_elapsed(substream);
}

static int jz4780_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4780_runtime_data *prtd;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	snd_soc_set_runtime_hwparams(substream, &jz4780_pcm_hardware);

	runtime->private_data = prtd;

	return 0;
}

static int jz4780_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4740_runtime_data *prtd = runtime->private_data;

	kfree(prtd);

	return 0;
}

static bool dma_chan_filter(struct dma_chan *chan, void *filter_param)
{
	return (uintptr_t)chan->private == JZDMA_REQ_I2S0;
}

static int jz4780_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4780_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct dma_slave_config *config;
	dma_cap_mask_t mask;

	config = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	if (!config)
		return 0;

	if (!prtd->dma) {
		/* request a DMA channel */
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		prtd->dma = dma_request_channel(mask, dma_chan_filter, NULL);
	}
	if (!prtd->dma) {
		printk(KERN_ERR "failed to allocate DMA channel\n");
		return -EBUSY;
	}

	/* configure it */
	dmaengine_slave_config(prtd->dma, config);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->size = params_buffer_bytes(params);
	prtd->periods = params_periods(params);
	prtd->period_bytes = params_period_bytes(params);
	prtd->offset = 0;

	prtd->desc = prtd->dma->device->device_prep_dma_cyclic(prtd->dma,
				runtime->dma_addr,
				prtd->period_bytes * prtd->periods,
				prtd->period_bytes,
				substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
				DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (!prtd->desc) {
		dev_err(&prtd->dma->dev->device, "cannot prepare slave dma\n");
		return -EINVAL;
	}
	prtd->desc->callback = jz4780_pcm_dma_irq;
	prtd->desc->callback_param = substream;
	dmaengine_submit(prtd->desc);

	return 0;
}

static int jz4780_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct jz4780_runtime_data *prtd = substream->runtime->private_data;

	snd_pcm_set_runtime_buffer(substream, NULL);

	if (prtd->dma) {
		dmaengine_terminate_all(prtd->dma);
		prtd->desc = NULL;
		dma_release_channel(prtd->dma);
		prtd->dma = NULL;
	}

	return 0;
}

static int jz4780_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct jz4780_runtime_data *prtd = substream->runtime->private_data;

	if (!prtd->dma)
		return -EBUSY;

	return 0;
}

static int jz4780_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4780_runtime_data *prtd = runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dma_async_issue_pending(prtd->dma);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dmaengine_terminate_all(prtd->dma);
		prtd->desc = NULL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t jz4780_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct jz4780_runtime_data *prtd = runtime->private_data;

	return bytes_to_frames(substream->runtime, prtd->offset);
}

static int jz4780_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	return remap_pfn_range(vma, vma->vm_start,
			substream->dma_buffer.addr >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static struct snd_pcm_ops jz4780_pcm_ops = {
	.open		= jz4780_pcm_open,
	.close		= jz4780_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= jz4780_pcm_hw_params,
	.hw_free	= jz4780_pcm_hw_free,
	.prepare	= jz4780_pcm_prepare,
	.trigger	= jz4780_pcm_trigger,
	.pointer	= jz4780_pcm_pointer,
	.mmap		= jz4780_pcm_mmap,
};

static u64 jz4780_pcm_dmamask = DMA_BIT_MASK(32);

static int jz4780_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = jz4780_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	buf->area = dma_alloc_noncoherent(pcm->card->dev, size,
					  &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;

	return 0;
}

static int jz4780_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &jz4780_pcm_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->driver->playback.channels_min) {
		ret = jz4780_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto err;
	}

	if (dai->driver->capture.channels_min) {
		ret = jz4780_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto err;
	}

err:
	return ret;
}

static void jz4780_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < SNDRV_PCM_STREAM_LAST; ++stream) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_noncoherent(pcm->card->dev, buf->bytes, buf->area,
				buf->addr);
		buf->area = NULL;
	}
}

static struct snd_soc_platform_driver jz4780_soc_platform = {
	.ops		= &jz4780_pcm_ops,
	.pcm_new	= jz4780_pcm_new,
	.pcm_free	= jz4780_pcm_free,
};

static int __devinit jz4780_pcm_dev_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &jz4780_soc_platform);
}

static int __devexit jz4780_pcm_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver jz4780_pcm_driver = {
	.probe = jz4780_pcm_dev_probe,
	.remove = __devexit_p(jz4780_pcm_dev_remove),
	.driver = {
		.name = "jz4780-pcm",
		.owner = THIS_MODULE,
	},
};

static int __init jz4780_pcm_init(void)
{
	return platform_driver_register(&jz4780_pcm_driver);
}
module_init(jz4780_pcm_init);

static void __exit jz4780_pcm_exit(void)
{
	platform_driver_unregister(&jz4780_pcm_driver);
}
module_exit(jz4780_pcm_exit);

MODULE_AUTHOR("Paul Burton <paul.burton@imgtec.com>");
MODULE_DESCRIPTION("Ingenic JZ4780 PCM driver");
MODULE_LICENSE("GPL");

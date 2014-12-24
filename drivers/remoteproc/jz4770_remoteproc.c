// SPDX-License-Identifier: GPL-2.0
/*
 * JZ4770 remoteproc driver
 * Copyright 2018, Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>

#include "remoteproc_internal.h"

#define REG_AUX_CTRL		0x0
#define REG_AUX_MSG_ACK		0x10
#define REG_AUX_MSG		0x14
#define REG_CORE_MSG_ACK	0x18
#define REG_CORE_MSG		0x1C

#define AUX_CTRL_SLEEP		BIT(31)
#define AUX_CTRL_MSG_IRQ_EN	BIT(3)
#define AUX_CTRL_IRQ_MODE	BIT(2)
#define AUX_CTRL_IRQ		BIT(1)
#define AUX_CTRL_SW_RESET	BIT(0)

struct vpu_tcsm_info {
	const char *name;
	unsigned int da;
	unsigned long len;
};

static const struct vpu_tcsm_info vpu_tcsm_info[] = {
	{ "tcsm0", 0x132b0000, 0x4000 },
	{ "tcsm1", 0xf4000000, 0xc000 },
	{ "sram",  0x132f0000, 0x7000 },
};

/* Device data */
struct vpu {
	int irq;
	struct clk *vpu_clk;
	struct clk *aux_clk;
	void __iomem *aux_base;
	void __iomem *sch_base;
	void __iomem *tcsm_base[ARRAY_SIZE(vpu_tcsm_info)];
	struct device *dev;
};

static int jz4770_rproc_start(struct rproc *rproc)
{
	struct vpu *vpu = rproc->priv;
	u32 ctrl;

	clk_prepare_enable(vpu->vpu_clk);
	clk_prepare_enable(vpu->aux_clk);
	enable_irq(vpu->irq);

	/* Disable TLB (for now) */
	writel(BIT(9), vpu->sch_base + 0x0);

	/* Clear the reset bit, enable IRQ to AUX, disable reset on IRQ */
	ctrl = AUX_CTRL_IRQ_MODE | AUX_CTRL_MSG_IRQ_EN;
	writel(ctrl, vpu->aux_base + REG_AUX_CTRL);
	return 0;
}

static int jz4770_rproc_stop(struct rproc *rproc)
{
	struct vpu *vpu = rproc->priv;

	/* Keep AUX in reset mode */
	writel(AUX_CTRL_SW_RESET, vpu->aux_base + REG_AUX_CTRL);

	disable_irq_nosync(vpu->irq);
	clk_disable_unprepare(vpu->aux_clk);
	clk_disable_unprepare(vpu->vpu_clk);
	return 0;
}

static void jz4770_rproc_kick(struct rproc *rproc, int vqid)
{
	struct vpu *vpu = rproc->priv;

	writel(vqid, vpu->aux_base + REG_CORE_MSG);
}

static void *jz4770_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct vpu *vpu = rproc->priv;
	void __iomem *va = NULL;
	unsigned int i;

	if (len <= 0)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(vpu_tcsm_info); i++) {
		if (da >= vpu_tcsm_info[i].da &&
					(da + len) < (vpu_tcsm_info[i].da + vpu_tcsm_info[i].len)) {
			va = vpu->tcsm_base[i] + (da - vpu_tcsm_info[i].da);
			break;
		}
	}

	return (__force void *)va;
}

static struct rproc_ops jz4770_rproc_ops = {
	.start = jz4770_rproc_start,
	.stop = jz4770_rproc_stop,
	.kick = jz4770_rproc_kick,
	.da_to_va = jz4770_rproc_da_to_va,
};

static irqreturn_t vpu_interrupt(int irq, void *data)
{
	struct vpu *vpu = data;
	struct rproc *rproc = dev_get_drvdata(vpu->dev);
	u32 vring = readl(vpu->aux_base + REG_AUX_MSG);

	writel(0, vpu->aux_base + REG_CORE_MSG_ACK);

	dev_dbg(vpu->dev, "Received message for vring %u\n", vring);
	rproc_vq_interrupt(rproc, vring);
	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static const struct of_device_id jz4770_rproc_of_matches[] = {
	{ .compatible = "ingenic,jz4770-remoteproc", },
	{}
};
MODULE_DEVICE_TABLE(of, jz4770_rproc_of_matches);
#endif

static int jz4770_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vpu *vpu;
	unsigned int i;
	int ret;
	struct rproc *rproc;

	rproc = rproc_alloc(dev, "jz4770-vpu", &jz4770_rproc_ops,
			NULL, sizeof(*vpu));
	if (!rproc)
		return -ENOMEM;

	devm_add_action(dev, (void (*)(void *))rproc_free, rproc);

	platform_set_drvdata(pdev, rproc);
	vpu = rproc->priv;
	vpu->dev = &pdev->dev;

	vpu->aux_base = devm_ioremap_resource(dev,
			platform_get_resource_byname(pdev, IORESOURCE_MEM, "aux"));
	if (IS_ERR(vpu->aux_base)) {
		ret = PTR_ERR(vpu->aux_base);
		dev_err(dev, "Failed to get and remap mmio region: %d\n", ret);
		return ret;
	}

	vpu->sch_base = devm_ioremap_resource(dev,
			platform_get_resource_byname(pdev, IORESOURCE_MEM, "sch"));
	if (IS_ERR(vpu->sch_base)) {
		ret = PTR_ERR(vpu->sch_base);
		dev_err(dev, "Failed to get and remap mmio region: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(vpu_tcsm_info); i++) {
		struct resource *res = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, vpu_tcsm_info[i].name);

		vpu->tcsm_base[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(vpu->tcsm_base[i])) {
			ret = PTR_ERR(vpu->tcsm_base[i]);
			dev_err(dev, "Failed to get and remap mmio region: %d\n", ret);
			return ret;
		}
	}

	vpu->vpu_clk = devm_clk_get(dev, "vpu");
	if (IS_ERR(vpu->vpu_clk)) {
		ret = PTR_ERR(vpu->vpu_clk);
		dev_err(dev, "Failed to get VPU clock: %d\n", ret);
		return ret;
	}

	vpu->aux_clk = devm_clk_get(dev, "aux");
	if (IS_ERR(vpu->aux_clk)) {
		ret = PTR_ERR(vpu->aux_clk);
		dev_err(dev, "Failed to get AUX clock: %d\n", ret);
		return ret;
	}

	vpu->irq = platform_get_irq(pdev, 0);
	if (vpu->irq < 0) {
		dev_err(dev, "Failed to get platform IRQ: %d\n", vpu->irq);
		return vpu->irq;
	}

	ret = devm_request_irq(dev, vpu->irq, vpu_interrupt, 0, "jz4770-vpu", vpu);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	disable_irq_nosync(vpu->irq);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "Failed to register remote processor: %d\n", ret);
		return ret;
	}

	devm_add_action(dev, (void (*)(void *))rproc_del, rproc);
	devm_add_action(dev, (void (*)(void *))rproc_shutdown, rproc);

	return 0;
}

static struct platform_driver jz4770_rproc_driver = {
	.probe = jz4770_rproc_probe,
	.driver = {
		.name = "jz4770-remoteproc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(jz4770_rproc_of_matches),
	},
};

module_platform_driver(jz4770_rproc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ4770 Remote Processor control driver");

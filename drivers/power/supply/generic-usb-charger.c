// SPDX-License-Identifier: GPL-2.0
/*
 * Simple USB charger driver
 * Copyright (c) 2019 Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/usb/phy.h>

struct usb_charger {
	struct usb_phy *phy;
	struct notifier_block nb;
	struct power_supply_desc desc;
	struct power_supply *charger;
};

static enum power_supply_property usb_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int usb_charger_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct usb_charger *charger = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->phy->chg_state == USB_CHARGER_PRESENT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int usb_charger_event(struct notifier_block *nb,
			     unsigned long event, void *d)
{
	struct usb_charger *charger = container_of(nb, struct usb_charger, nb);

	power_supply_changed(charger->charger);

	return 0;
}

static int usb_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_desc *desc;
	struct usb_charger *charger;
	struct power_supply_config cfg = {
		.of_node = dev->of_node,
	};
	int err;

	charger = devm_kzalloc(dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	platform_set_drvdata(pdev, charger);
	charger->nb.notifier_call = usb_charger_event;
	cfg.drv_data = charger;

	if (dev->of_node)
		charger->phy = devm_usb_get_phy_by_phandle(dev, "phys", 0);
	else
		charger->phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (IS_ERR(charger->phy)) {
		err = PTR_ERR(charger->phy);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "No transceiver configured");
		return err;
	}

	desc = &charger->desc;
	desc->name = "usb-charger";
	desc->properties = usb_charger_properties;
	desc->num_properties = ARRAY_SIZE(usb_charger_properties);
	desc->get_property = usb_charger_get_property;

	switch (charger->phy->chg_type) {
	case SDP_TYPE:
		desc->type = POWER_SUPPLY_TYPE_USB;
		break;
	case DCP_TYPE:
		desc->type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case CDP_TYPE:
		desc->type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case ACA_TYPE:
		desc->type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	default:
		desc->type = POWER_SUPPLY_TYPE_UNKNOWN;
	}

	charger->charger = devm_power_supply_register(dev, desc, &cfg);
	if (IS_ERR(charger->charger)) {
		dev_err(dev, "Unable to register charger");
		return PTR_ERR(charger->charger);
	}

	return usb_register_notifier(charger->phy, &charger->nb);
}

static int usb_charger_remove(struct platform_device *pdev)
{
	struct usb_charger *charger = platform_get_drvdata(pdev);

	usb_unregister_notifier(charger->phy, &charger->nb);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id usb_charger_of_match[] = {
	{ .compatible = "usb-charger" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, usb_charger_of_match);
#endif

static struct platform_driver usb_charger_driver = {
	.driver = {
		.name = "usb-charger",
		.of_match_table = of_match_ptr(usb_charger_of_match),
	},
	.probe = usb_charger_probe,
	.remove = usb_charger_remove,
};
module_platform_driver(usb_charger_driver);

MODULE_DESCRIPTION("Simple USB charger driver");
MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_LICENSE("GPL");

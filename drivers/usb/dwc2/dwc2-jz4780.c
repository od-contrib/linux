#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <linux/jz_dwc.h>
#include <soc/base.h>
#include <soc/cpm.h>

#include "core.h"
#include "gadget.h"

#define OTG_CLK_NAME		"otg1"
#define VBUS_REG_NAME		"vbus"

#define USBRDT_VBFIL_LD_EN		25
#define USBPCR_TXPREEMPHTUNE		6
#define USBPCR_POR			22
#define USBPCR_USB_MODE			31
#define USBPCR_COMMONONN		25
#define USBPCR_VBUSVLDEXT		24
#define USBPCR_VBUSVLDEXTSEL		23
#define USBPCR_OTG_DISABLE		20
#define USBPCR_IDPULLUP_MASK		28
#define OPCR_SPENDN0			7
#define USBPCR1_USB_SEL			28
#define USBPCR1_WORD_IF0		19
#define USBPCR1_WORD_IF1		18

struct dwc2_jz4780 {
	struct platform_device  dwc2;
	struct device		*dev;
	struct clk		*clk;

	spinlock_t		lock; /* protect between first power on op and irq op */

#if DWC2_DEVICE_MODE_ENABLE
	int 			dete_irq;
	struct jzdwc_pin 	*dete;
	int			pullup_on;
	struct delayed_work	work;
#endif

#if DWC2_HOST_MODE_ENABLE
	int			id_irq;
	struct jzdwc_pin 	*id_pin;
	struct wake_lock        id_resume_wake_lock;
	struct delayed_work	host_id_work;
	struct timer_list	host_id_timer;
	int 			host_id_dog_count;
#define DWC2_HOST_ID_TIMER_INTERVAL (HZ / 2)
#define DWC2_HOST_ID_MAX_DOG_COUNT  3

	struct regulator 	*vbus;
	struct mutex             vbus_lock;
#ifdef CONFIG_USB_DWC2_INPUT_EVDEV
	struct input_dev	*input;
#endif	/* CONFIG_USB_DWC2_INPUT_EVDEV */
#endif	/* DWC2_HOST_MODE_ENABLE */
};

#if DWC2_DEVICE_MODE_ENABLE
struct jzdwc_pin __attribute__((weak)) dete_pin = {
	.num	      = -1,
	.enable_level = -1,
};
#endif

void (*set_charger_current)(int cur) = NULL;
EXPORT_SYMBOL(set_charger_current);

/* NOTE: the following global variables are only used by special late_initcall */
static struct dwc2_jz4780 *g_dwc2_jz4780 = NULL;

#if !DWC2_HOST_MODE_ENABLE
void jz4780_set_vbus(struct dwc2 *dwc, int is_on) {  }
int dwc2_get_id_level(struct dwc2 *dwc) { return 1; }

#else  /* DWC2_HOST_MODE_ENABLE */

static int g_ignore_vbus_on __read_mostly = 1;
static int g_vbus_on_pending = 0;

struct jzdwc_pin __attribute__((weak)) dwc2_id_pin = {
	.num	      = -1,
	.enable_level = -1
};

extern void dwc2_notifier_call_chain_sync(int state);

/* mA */
#define CHARGER_CURRENT_USB	228
#define CHARGER_CURRENT_CHARGER	500

static void charger_set_current(int cur) {
	if (set_charger_current) {
		set_charger_current(cur);
	}
}

static void __jz4780_set_vbus(struct dwc2_jz4780 *jz4780, int is_on) {
	struct dwc2 *dwc = platform_get_drvdata(&jz4780->dwc2);
	int old_is_on = is_on;

	if (unlikely(g_ignore_vbus_on) && is_on) {
		g_vbus_on_pending = 1;
		return;
	}

	mutex_lock(&jz4780->vbus_lock);

	if (dwc->plugin) {
		old_is_on = is_on;
		if (is_on) {
			dwc->extern_vbus_mode = 1;
			dwc2_notifier_call_chain_sync(1);
		}
		is_on = 0;
	} else {
		int id_level = 0;

		if (gpio_is_valid(jz4780->id_pin->num)) {
			id_level = gpio_get_value(jz4780->id_pin->num);
		}

		if (id_level)
			is_on = 0;
	}

	dev_info(jz4780->dev, "set vbus %s(%s) for %s mode\n",
		is_on ? "on" : "off",
		old_is_on ? "on" : "off",
		dwc2_is_host_mode(dwc) ? "host" : "device");

	if (is_on && dwc2_is_host_mode(dwc)) {
		if (!regulator_is_enabled(jz4780->vbus))
			regulator_enable(jz4780->vbus);
	} else {	      /* off or device mode */
		if (regulator_is_enabled(jz4780->vbus))
			regulator_disable(jz4780->vbus);
	}

	if (dwc->plugin) {
		if (old_is_on) {
			charger_set_current(CHARGER_CURRENT_USB);
		} else {
			charger_set_current(CHARGER_CURRENT_CHARGER);
		}
	}

	mutex_unlock(&jz4780->vbus_lock);
}

void jz4780_set_vbus(struct dwc2 *dwc, int is_on)
{
	struct dwc2_jz4780	*jz4780;

	jz4780 = container_of(dwc->pdev, struct dwc2_jz4780, dwc2);
	if ( (jz4780->vbus == NULL) || IS_ERR(jz4780->vbus) )
		return;

	__jz4780_set_vbus(jz4780, is_on);
}

static ssize_t jz4780_vbus_show(struct device *dev,
				struct device_attribute *attr,
				char *buf) {
	struct dwc2_jz4780 *jz4780 = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n",
		regulator_is_enabled(jz4780->vbus) ? "on" : "off");
}

static ssize_t jz4780_vbus_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dwc2_jz4780 *jz4780 = dev_get_drvdata(dev);
	struct dwc2 *dwc = platform_get_drvdata(&jz4780->dwc2);
	int is_on = 0;

	if (strncmp(buf, "on", 2) == 0) {
		int id_level = 0;

		if (dwc->extern_vbus_mode) {
			charger_set_current(CHARGER_CURRENT_USB);
			return count;
		}

		if (gpio_is_valid(jz4780->id_pin->num)) {
			id_level = gpio_get_value(jz4780->id_pin->num);
		}

		if (id_level) {
			return count;
		}

		is_on = 1;
	}

	__jz4780_set_vbus(jz4780, is_on);

	return count;
}

static DEVICE_ATTR(vbus, S_IWUSR | S_IRUSR |  S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH,
		jz4780_vbus_show, jz4780_vbus_set);

static ssize_t jz4780_id_show(struct device *dev,
				struct device_attribute *attr,
				char *buf) {
	struct dwc2_jz4780 *jz4780 = dev_get_drvdata(dev);
	int id_level = 0;

	if (gpio_is_valid(jz4780->id_pin->num)) {
		id_level = gpio_get_value(jz4780->id_pin->num);
	}

	return sprintf(buf, "%s\n", id_level ? "1" : "0");
}

static DEVICE_ATTR(id, S_IRUSR |  S_IRGRP | S_IROTH,
		jz4780_id_show, NULL);
#endif	/* DWC2_HOST_MODE_ENABLE */

static struct attribute *dwc2_jz4780_attributes[] = {
#if DWC2_HOST_MODE_ENABLE
	&dev_attr_vbus.attr,
	&dev_attr_id.attr,
#endif	/* DWC2_HOST_MODE_ENABLE */
	NULL
};

static const struct attribute_group dwc2_jz4780_attr_group = {
	.attrs = dwc2_jz4780_attributes,
};

#ifdef CONFIG_USB_DWC2_INPUT_EVDEV
int dwc2_jz4780_input_init(struct dwc2_jz4780 *jz4780) {
	struct input_dev *input;
	int ret;

	input = input_allocate_device();
	if (input == NULL)
		return -ENOMEM;

	input->name = "dwc2";
	input->dev.parent = jz4780->dev;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(KEY_POWER2, input->keybit);

	if ((ret = input_register_device(input)) < 0)
		goto error;

	jz4780->input = input;

	return 0;

error:
	input_free_device(input);
	return ret;
}

static void dwc2_jz4780_input_cleanup(struct dwc2_jz4780 *jz4780)
{
	if (jz4780->input)
		input_unregister_device(jz4780->input);
}

static void dwc2_jz4780_input_report_key(struct dwc2_jz4780 *jz4780,
					unsigned int code, int value)
{
	if (jz4780->input) {
		input_report_key(jz4780->input, code, value);
		input_sync(jz4780->input);
	}
}

static void __dwc2_input_report_power2_key(struct dwc2_jz4780 *jz4780) {
	dwc2_jz4780_input_report_key(jz4780, KEY_POWER2, 1);
	msleep(50);
	dwc2_jz4780_input_report_key(jz4780, KEY_POWER2, 0);
}

void dwc2_input_report_power2_key(struct dwc2 *dwc) {
	struct dwc2_jz4780	*jz4780;

	jz4780 = container_of(dwc->pdev, struct dwc2_jz4780, dwc2);

	__dwc2_input_report_power2_key(jz4780);
}

#else
#define dwc2_jz4780_input_init(dev) do {  } while(0)
#define dwc2_jz4780_input_cleanup(dev) do {  } while(0)
#define dwc2_jz4780_input_report_key(dev, code, value) do {  } while(0)
void dwc2_input_report_power2_key(struct dwc2 *dwc) {  }
static void __attribute__((unused)) __dwc2_input_report_power2_key(struct dwc2_jz4780 *jz4780) {  }
#endif /* CONFIG_USB_DWC2_INPUT_EVDEV */

static inline void jz4780_usb_phy_init(struct dwc2_jz4780 *jz4780)
{
	pr_debug("init PHY\n");

	cpm_set_bit(USBPCR_POR, CPM_USBPCR);
	msleep(1);
	cpm_clear_bit(USBPCR_POR, CPM_USBPCR);
	msleep(1);
}

static inline void jz4780_usb_set_device_only_mode(void)
{
	pr_info("DWC IN DEVICE ONLY MODE\n");
	cpm_clear_bit(USBPCR_USB_MODE, CPM_USBPCR);
	cpm_clear_bit(USBPCR_OTG_DISABLE, CPM_USBPCR);
}

static inline void jz4780_usb_set_dual_mode(void)
{
	unsigned int tmp;

	pr_info("DWC IN OTG MODE\n");

	tmp = cpm_inl(CPM_USBPCR);
	tmp |= 1 << USBPCR_USB_MODE;
	tmp |= 1 << USBPCR_VBUSVLDEXT;
	tmp |= 1 << USBPCR_VBUSVLDEXTSEL;
	tmp |= 1 << USBPCR_COMMONONN;

	tmp &= ~(1 << USBPCR_OTG_DISABLE);

	cpm_outl(tmp & ~(0x03 << USBPCR_IDPULLUP_MASK), CPM_USBPCR);
}

static inline void jz4780_usb_set_mode(void) {
#if (DWC2_DEVICE_MODE_ENABLE) && !(DWC2_HOST_MODE_ENABLE)
	jz4780_usb_set_device_only_mode();
#else
	jz4780_usb_set_dual_mode();
#endif
}

#if DWC2_DEVICE_MODE_ENABLE
static int get_pin_status(struct jzdwc_pin *pin)
{
	int val;

	if (pin->num < 0)
		return -1;
	val = gpio_get_value(pin->num);

	if (pin->enable_level == LOW_ENABLE)
		return !val;
	return val;
}

extern void dwc2_gadget_plug_change(int plugin);

static void __usb_plug_change(struct dwc2_jz4780 *jz4780) {
	int insert;

	insert = get_pin_status(jz4780->dete);

	pr_info("USB %s\n", insert ? "connect" : "disconnect");
	dwc2_gadget_plug_change(insert);
}

static void usb_plug_change(struct dwc2_jz4780 *jz4780) {

	spin_lock(&jz4780->lock);
	__usb_plug_change(jz4780);
	spin_unlock(&jz4780->lock);
}

static void usb_detect_work(struct work_struct *work)
{
	struct dwc2_jz4780	*jz4780;

	jz4780 = container_of(work, struct dwc2_jz4780, work.work);

	usb_plug_change(jz4780);

	enable_irq(jz4780->dete_irq);
}

static irqreturn_t usb_detect_interrupt(int irq, void *dev_id)
{
	struct dwc2_jz4780	*jz4780 = (struct dwc2_jz4780 *)dev_id;

	disable_irq_nosync(irq);
	schedule_delayed_work(&jz4780->work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}
#endif	/* DWC2_DEVICE_MODE_ENABLE */

#if DWC2_HOST_MODE_ENABLE
static int __dwc2_get_id_level(struct dwc2_jz4780 *jz4780) {
	int			 id_level = 0;

	if (gpio_is_valid(jz4780->id_pin->num)) {
		id_level = gpio_get_value(jz4780->id_pin->num);
	}

	return id_level;
}

int dwc2_get_id_level(struct dwc2 *dwc) {
	struct dwc2_jz4780	*jz4780;

	jz4780 = container_of(dwc->pdev, struct dwc2_jz4780, dwc2);

	return __dwc2_get_id_level(jz4780);
}

static void usb_host_id_timer(unsigned long _jz4780) {
	struct dwc2_jz4780 *jz4780 = (struct dwc2_jz4780 *)_jz4780;

	if (gpio_get_value(jz4780->id_pin->num) == 0) { /* host */
		cpm_set_bit(7, CPM_OPCR);
	}

	if (jz4780->host_id_dog_count > 0) {
		jz4780->host_id_dog_count --;
		mod_timer(&jz4780->host_id_timer, jiffies + DWC2_HOST_ID_TIMER_INTERVAL);
	}
}

static int __usb_host_id_change(struct dwc2_jz4780 *jz4780) {
	int is_host = 0;

	// printk("==============>enter %s, id pin level=%d\n",
	//	__func__, gpio_get_value(jz4780->id_pin->num));

	if (gpio_get_value(jz4780->id_pin->num) == 0) { /* host */
		cpm_set_bit(7, CPM_OPCR);
		is_host = 1;
	}

	jz4780->host_id_dog_count = DWC2_HOST_ID_MAX_DOG_COUNT;
	mod_timer(&jz4780->host_id_timer, jiffies + DWC2_HOST_ID_TIMER_INTERVAL);

	return is_host;
}

static int usb_host_id_change(struct dwc2_jz4780 *jz4780) {
	int is_host;

	spin_lock(&jz4780->lock);
	is_host = __usb_host_id_change(jz4780);
	spin_unlock(&jz4780->lock);

	return is_host;
}

static void usb_host_id_work(struct work_struct *work) {
	struct dwc2_jz4780	*jz4780;
	int			 report_key = 0;

	jz4780 = container_of(work, struct dwc2_jz4780, host_id_work.work);

	report_key = usb_host_id_change(jz4780);

	enable_irq(jz4780->id_irq);

	if (report_key) {
		__dwc2_input_report_power2_key(jz4780);
	}
}

static irqreturn_t usb_host_id_interrupt(int irq, void *dev_id) {
	struct dwc2_jz4780	*jz4780 = (struct dwc2_jz4780 *)dev_id;

	disable_irq_nosync(irq);
#if 0
	if (gpio_get_value(jz4780->id_pin->num) == 0) { /* host */
		cpm_set_bit(7, CPM_OPCR);
	}

	printk("===>id pin level=%d\n", gpio_get_value(jz4780->id_pin->num));
#endif
	wake_lock_timeout(&jz4780->id_resume_wake_lock, 3 * HZ);

	/* 50ms dither filter */
	schedule_delayed_work(&jz4780->host_id_work, msecs_to_jiffies(50));

	return IRQ_HANDLED;
}
#endif	/* DWC2_HOST_MODE_ENABLE */

static void usb_cpm_init(void) {
	unsigned int ref_clk_div = CONFIG_EXTAL_CLOCK / 24;
	unsigned int usbpcr1;

	/* select dwc otg */
	cpm_set_bit(USBPCR1_USB_SEL, CPM_USBPCR1);

	/* select utmi data bus width of port0 to 16bit/30M */
	cpm_set_bit(USBPCR1_WORD_IF0, CPM_USBPCR1);

	usbpcr1 = cpm_inl(CPM_USBPCR1);
	usbpcr1 &= ~(0x3 << 24);
	usbpcr1 |= (ref_clk_div << 24);
	cpm_outl(usbpcr1, CPM_USBPCR1);

	/* fil */
	cpm_outl(0, CPM_USBVBFIL);

	/* rdt */
	cpm_outl(0x96, CPM_USBRDT);

	/* rdt - filload_en */
	cpm_set_bit(USBRDT_VBFIL_LD_EN, CPM_USBRDT);

	/* TXRISETUNE & TXVREFTUNE. */
	//cpm_outl(0x3f, CPM_USBPCR);
	//cpm_outl(0x35, CPM_USBPCR);

	/* enable tx pre-emphasis */
	//cpm_set_bit(USBPCR_TXPREEMPHTUNE, CPM_USBPCR);

	/* OTGTUNE adjust */
	//cpm_outl(7 << 14, CPM_USBPCR);

	cpm_outl(0x8180385F, CPM_USBPCR);
}

static u64 dwc2_jz4780_dma_mask = DMA_BIT_MASK(32);

static int dwc2_jz4780_probe(struct platform_device *pdev) {
	struct platform_device		*dwc2;
	struct dwc2_jz4780		*jz4780;
	struct dwc2_platform_data	*dwc2_plat_data;
	int				 ret = -ENOMEM;

	jz4780 = kzalloc(sizeof(*jz4780), GFP_KERNEL);
	if (!jz4780) {
		dev_err(&pdev->dev, "not enough memory\n");
		goto out;
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we move to full device tree support this will vanish off.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &dwc2_jz4780_dma_mask;

	platform_set_drvdata(pdev, jz4780);

	dwc2 = &jz4780->dwc2;
	dwc2->name = "dwc2";
	dwc2->id = -1;
	device_initialize(&dwc2->dev);
	dma_set_coherent_mask(&dwc2->dev, pdev->dev.coherent_dma_mask);
	dwc2->dev.parent = &pdev->dev;
	dwc2->dev.dma_mask = pdev->dev.dma_mask;
	dwc2->dev.dma_parms = pdev->dev.dma_parms;

	dwc2->dev.platform_data = kzalloc(sizeof(struct dwc2_platform_data), GFP_KERNEL);
	if (!dwc2->dev.platform_data) {
		goto fail_alloc_dwc2_plat_data;
	}
	dwc2_plat_data = dwc2->dev.platform_data;

	jz4780->dev	= &pdev->dev;
	spin_lock_init(&jz4780->lock);

	jz4780->clk = clk_get(NULL, OTG_CLK_NAME);
	if (IS_ERR(jz4780->clk)) {
		dev_err(&pdev->dev, "clk gate get error\n");
		goto fail_get_clk;
	}
	clk_enable(jz4780->clk);

#if DWC2_DEVICE_MODE_ENABLE
	jz4780->dete = &dete_pin;
	jz4780->dete_irq = -1;
	ret = gpio_request_one(jz4780->dete->num,
			GPIOF_DIR_IN, "usb-charger-detect");
	if (ret == 0) {
		jz4780->dete_irq = gpio_to_irq(jz4780->dete->num);
		INIT_DELAYED_WORK(&jz4780->work, usb_detect_work);
	}
#ifdef CONFIG_BOARD_HAS_NO_DETE_FACILITY
	if (jz4780->dete_irq < 0) {
		dwc2_plat_data->keep_phy_on = 1;
	}
#endif	/* CONFIG_BOARD_HAS_NO_DETE_FACILITY */
#endif	/* DWC2_DEVICE_MODE_ENABLE */

#if DWC2_HOST_MODE_ENABLE
#ifdef CONFIG_REGULATOR
	jz4780->vbus = regulator_get(NULL, VBUS_REG_NAME);

	if (IS_ERR(jz4780->vbus)) {
		dev_err(&pdev->dev, "regulator %s get error\n", VBUS_REG_NAME);
		goto fail_get_vbus_reg;
	}
	if (regulator_is_enabled(jz4780->vbus))
		regulator_disable(jz4780->vbus);
#else
#error DWC OTG driver can NOT work without regulator!
#endif

	mutex_init(&jz4780->vbus_lock);
	jz4780->id_pin = &dwc2_id_pin;
	jz4780->id_irq = -1;
	ret = gpio_request_one(jz4780->id_pin->num,
			GPIOF_DIR_IN, "usb-host-id-detect");
	if (ret == 0) {
		jz4780->id_irq = gpio_to_irq(jz4780->id_pin->num);
		INIT_DELAYED_WORK(&jz4780->host_id_work, usb_host_id_work);
		setup_timer(&jz4780->host_id_timer, usb_host_id_timer,
			(unsigned long)jz4780);
		jz4780->host_id_dog_count = 0;

		wake_lock_init(&jz4780->id_resume_wake_lock, WAKE_LOCK_SUSPEND, "otgid-resume");
	} else {
		dwc2_plat_data->keep_phy_on = 1;
	}
#endif	/* DWC2_HOST_MODE_ENABLE */

	dwc2_jz4780_input_init(jz4780);

	usb_cpm_init();

	jz4780_usb_set_mode();

	jz4780_usb_phy_init(jz4780);

	if (dwc2_plat_data->keep_phy_on)
		cpm_set_bit(7, CPM_OPCR);
	else
		cpm_clear_bit(7, CPM_OPCR);

	/*
	 * Close VBUS detect in DWC-OTG PHY.
	 */
	*(unsigned int*)0xb3500000 |= 0xc;

	ret = platform_device_add_resources(dwc2, pdev->resource,
					pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "couldn't add resources to dwc2 device\n");
		goto fail_add_dwc2_res;
	}

	ret = platform_device_add(dwc2);
	if (ret) {
		dev_err(&pdev->dev, "failed to register dwc2 device\n");
		goto fail_add_dwc2_dev;
	}

#if DWC2_DEVICE_MODE_ENABLE
	if (jz4780->dete_irq >= 0) {
		ret = request_irq(gpio_to_irq(jz4780->dete->num),
				usb_detect_interrupt,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"usb-detect", jz4780);
		if (ret) {
			jz4780->dete_irq = -1;
			dev_err(&pdev->dev, "request usb-detect fail\n");
			goto fail_req_dete_irq;
		} else {
			usb_plug_change(jz4780);
		}
	}

#ifdef CONFIG_BOARD_HAS_NO_DETE_FACILITY
	if (jz4780->dete_irq < 0) {
		dwc2_gadget_plug_change(1);
	}
#endif
#endif	/* DWC2_DEVICE_MODE_ENABLE */

#if DWC2_HOST_MODE_ENABLE
	if (jz4780->id_irq >= 0) {
		ret = request_irq(gpio_to_irq(jz4780->id_pin->num),
				usb_host_id_interrupt,
				IRQF_TRIGGER_FALLING,
				"usb-host-id", jz4780);
		if (ret) {
			jz4780->id_irq = -1;
			dev_err(&pdev->dev, "request host id interrupt fail!\n");
			goto fail_req_id_irq;
		} else {
			usb_host_id_change(jz4780);
		}
	}
#endif	/* DWC2_HOST_MODE_ENABLE */

	ret = sysfs_create_group(&pdev->dev.kobj, &dwc2_jz4780_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create sysfs group\n");
	}

	g_dwc2_jz4780 = jz4780;

	return 0;

#if DWC2_HOST_MODE_ENABLE
fail_req_id_irq:
#endif

#if DWC2_DEVICE_MODE_ENABLE
	free_irq(gpio_to_irq(jz4780->dete->num), jz4780);
fail_req_dete_irq:
#endif	/* DWC2_DEVICE_MODE_ENABLE */

fail_add_dwc2_dev:
fail_add_dwc2_res:
#if DWC2_DEVICE_MODE_ENABLE
	if (gpio_is_valid(jz4780->dete->num))
		gpio_free(jz4780->dete->num);
#endif

#if DWC2_HOST_MODE_ENABLE
	if (gpio_is_valid(jz4780->id_pin->num))
		gpio_free(jz4780->id_pin->num);

#ifdef CONFIG_REGULATOR
	regulator_put(jz4780->vbus);

fail_get_vbus_reg:
#endif	/* CONFIG_REGULATOR */
#endif	/* DWC2_HOST_MODE_ENABLE */
	clk_disable(jz4780->clk);
	clk_put(jz4780->clk);

fail_get_clk:
	kfree(dwc2->dev.platform_data);

fail_alloc_dwc2_plat_data:
	kfree(jz4780);
out:
	return ret;
}

static int dwc2_jz4780_remove(struct platform_device *pdev) {
	struct dwc2_jz4780	*jz4780 = platform_get_drvdata(pdev);

	dwc2_jz4780_input_cleanup(jz4780);

#if DWC2_DEVICE_MODE_ENABLE
	if (jz4780->dete_irq >= 0) {
		free_irq(jz4780->dete_irq, jz4780);
		gpio_free(jz4780->dete->num);
	}
#endif

#if DWC2_HOST_MODE_ENABLE
	if (jz4780->id_irq >= 0) {
		free_irq(jz4780->id_irq, jz4780);
		gpio_free(jz4780->id_pin->num);
	}

	if (!IS_ERR(jz4780->vbus))
		regulator_put(jz4780->vbus);
#endif

	clk_disable(jz4780->clk);
	clk_put(jz4780->clk);

	platform_device_unregister(&jz4780->dwc2);
	kfree(jz4780);

	return 0;
}

static int dwc2_jz4780_suspend(struct platform_device *pdev, pm_message_t state) {
	struct dwc2_jz4780	*jz4780 = platform_get_drvdata(pdev);
#if DWC2_DEVICE_MODE_ENABLE
	if (jz4780->dete_irq >= 0)
		enable_irq_wake(jz4780->dete_irq);
#endif

#if DWC2_HOST_MODE_ENABLE
	if (jz4780->id_irq >= 0)
		enable_irq_wake(jz4780->id_irq);
#endif

	return 0;
}

static int dwc2_jz4780_resume(struct platform_device *pdev) {
	struct dwc2_jz4780	*jz4780 = platform_get_drvdata(pdev);
#if DWC2_DEVICE_MODE_ENABLE
	if (jz4780->dete_irq >= 0)
		disable_irq_wake(jz4780->dete_irq);
#endif

#if DWC2_HOST_MODE_ENABLE
	if (jz4780->id_irq >= 0)
		disable_irq_wake(jz4780->id_irq);
#endif

	return 0;
}

static struct platform_driver dwc2_jz4780_driver = {
	.probe		= dwc2_jz4780_probe,
	.remove		= dwc2_jz4780_remove,
	.suspend	= dwc2_jz4780_suspend,
	.resume		= dwc2_jz4780_resume,
	.driver		= {
		.name	= "jz4780-dwc2",
		.owner =  THIS_MODULE,
	},
};


static int __init dwc2_jz4780_init(void)
{
	return platform_driver_register(&dwc2_jz4780_driver);
}

static void __exit dwc2_jz4780_exit(void)
{
	platform_driver_unregister(&dwc2_jz4780_driver);
}

/* make us init after usbcore and i2c (transceivers, regulators, etc)
 * and before usb gadget and host-side drivers start to register
 */
fs_initcall(dwc2_jz4780_init);
module_exit(dwc2_jz4780_exit);

static int __init dwc2_jz4780_late_init(void) {
	// printk("===>enter %s:%d: jz4780 = %p, pending = %d\n",
	//	__func__, __LINE__, g_dwc2_jz4780, g_vbus_on_pending);
#if DWC2_HOST_MODE_ENABLE
	if (g_dwc2_jz4780) {
		g_ignore_vbus_on = 0;
		if (g_vbus_on_pending) {
			if (__dwc2_get_id_level(g_dwc2_jz4780) == 0) {
				__jz4780_set_vbus(g_dwc2_jz4780, 1);
			}

			g_vbus_on_pending = 0;
		}
	}
#endif

	return 0;
}

late_initcall(dwc2_jz4780_late_init);

MODULE_ALIAS("platform:jz4780-dwc2");
MODULE_AUTHOR("Lutts Cao <slcao@ingenic.cn>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DesignWare USB2.0 JZ4780 Glue Layer");

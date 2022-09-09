// SPDX-License-Identifier: GPL-2.0
/*
 * Pinctrl / GPIO driver for StarFive JH7110 SoC aon controller
 *
 * Copyright (C) 2022 StarFive Technology Co., Ltd.
 */

#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinmux.h"
#include "pinctrl-starfive.h"

/* aon_iomux */
#define AON_GPO_DOEN_CFG		0x0
#define AON_GPO_DOEN_MASK		GENMASK(2, 0)
#define AON_GPO_DOUT_CFG		0x4
#define AON_GPO_DOUT_MASK		GENMASK(3, 0)
#define AON_GPI_DIN_CFG			0x8
#define AON_GPI_DIN_MASK		GENMASK(3, 0)
#define AON_GPIO_DIN_REG		0x2c

/* aon_iomux GPIO CTRL */
#define AON_GPIO_EN_REG			0xc
#define AON_GPIO_IS_REG			0x10
#define AON_GPIO_IC_REG			0x14
#define AON_GPIO_IBE_REG		0x18
#define AON_GPIO_IEV_REG		0x1c
#define AON_GPIO_IE_REG			0x20
#define AON_GPIO_MIS_REG		0x28

/* aon_iomux PIN ioconfig reg */
#define AON_GPO_PDA_0_5_CFG		0x30
#define PADCFG_PAD_GMAC_SYSCON_SHIFT	0x0
#define PADCFG_PAD_GMAC_SYSCON_MASK	GENMASK(1, 0)
#define A0N_GPO_PDA_CFG_OFFSET		0x4
#define AON_GPIO_INPUT_ENABLE_REG	0x34

#define AON_GPIO_NUM			4

enum starfive_jh7110_aon_pads {
	PAD_TESTEN	= 0,
	PAD_RGPIO0	= 1,
	PAD_RGPIO1	= 2,
	PAD_RGPIO2	= 3,
	PAD_RGPIO3	= 4,
	PAD_RSTN	= 5,
	PAD_GMAC0_MDC	= 6,
	PAD_GMAC0_MDIO	= 7,
	PAD_GMAC0_RXD0	= 8,
	PAD_GMAC0_RXD1	= 9,
	PAD_GMAC0_RXD2	= 10,
	PAD_GMAC0_RXD3	= 11,
	PAD_GMAC0_RXDV	= 12,
	PAD_GMAC0_RXC	= 13,
	PAD_GMAC0_TXD0	= 14,
	PAD_GMAC0_TXD1	= 15,
	PAD_GMAC0_TXD2	= 16,
	PAD_GMAC0_TXD3	= 17,
	PAD_GMAC0_TXEN	= 18,
	PAD_GMAC0_TXC	= 19,
};

static const struct pinctrl_pin_desc starfive_jh7110_aon_pinctrl_pads[] = {
	STARFIVE_PINCTRL_PIN(PAD_TESTEN),
	STARFIVE_PINCTRL_PIN(PAD_RGPIO0),
	STARFIVE_PINCTRL_PIN(PAD_RGPIO1),
	STARFIVE_PINCTRL_PIN(PAD_RGPIO2),
	STARFIVE_PINCTRL_PIN(PAD_RGPIO3),
	STARFIVE_PINCTRL_PIN(PAD_RSTN),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_MDC),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_MDIO),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_RXD0),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_RXD1),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_RXD2),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_RXD3),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_RXDV),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_RXC),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_TXD0),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_TXD1),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_TXD2),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_TXD3),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_TXEN),
	STARFIVE_PINCTRL_PIN(PAD_GMAC0_TXC),
};

static int jh7110_aon_pmx_set_one_pin_mux(struct starfive_pinctrl *pctl,
					  struct starfive_pin *pin)
{
	const struct starfive_pinctrl_soc_info *info = pctl->info;
	struct starfive_pin_config *pin_config = &pin->pin_config;
	const struct starfive_pin_reg *pin_reg;
	unsigned int gpio, pin_id;
	int i;
	unsigned long flags;
	int n, shift;

	gpio = pin->pin_config.gpio_num;
	pin_id = pin->pin;
	pin_reg = &pctl->pin_regs[pin_id];

	raw_spin_lock_irqsave(&pctl->lock, flags);
	if (pin_reg->func_sel_reg != -1) {
		pinctrl_set_reg(pctl->padctl_base + pin_reg->func_sel_reg,
				pin_config->pinmux_func,
				pin_reg->func_sel_shift,
				pin_reg->func_sel_mask);
	}

	shift = GET_GPO_CFG_SHIFT(gpio);
	if (pin_reg->gpo_dout_reg != -1) {
		pinctrl_write_reg(pctl->padctl_base + pin_reg->gpo_dout_reg,
				  AON_GPO_DOUT_MASK << shift,
				  pin_config->gpio_dout << shift);
	}

	if (pin_reg->gpo_doen_reg != -1) {
		pinctrl_write_reg(pctl->padctl_base + pin_reg->gpo_doen_reg,
				  AON_GPO_DOEN_MASK << shift,
				  pin_config->gpio_doen << shift);
	}

	for (i = 0; i < pin_config->gpio_din_num; i++) {
		n = pin_config->gpio_din_reg[i] >> 2;
		shift = (pin_config->gpio_din_reg[i] & 3) << 3;
		pinctrl_write_reg(pctl->padctl_base + info->din_reg_base + n * 4,
				  AON_GPI_DIN_MASK << shift,
				  (gpio + 2) << shift);
	}

	if (pin_reg->syscon_reg != -1) {
		pinctrl_set_reg(pctl->padctl_base + pin_reg->syscon_reg,
				pin_config->syscon,
				PADCFG_PAD_GMAC_SYSCON_SHIFT,
				PADCFG_PAD_GMAC_SYSCON_MASK);
	}

	raw_spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static void jh7110_aon_parse_pin_config(struct starfive_pinctrl *pctl,
					unsigned int *pins_id,
					struct starfive_pin *pin_data,
					const __be32 *list_p,
					struct device_node *np)
{
	const struct starfive_pinctrl_soc_info *info = pctl->info;
	struct starfive_pin_reg *pin_reg;
	const __be32 *list = list_p;
	const __be32 *list_din;
	int size;
	int size_din;
	int pin_size;
	u32 value;
	int i;

	pin_size = sizeof(u32);
	*pins_id = be32_to_cpu(*list);
	pin_reg = &pctl->pin_regs[*pins_id];
	pin_data->pin = *pins_id;

	if (pin_data->pin > PAD_GMAC0_TXC) {
		dev_err(pctl->dev, "err pin num = %d\n", pin_data->pin);
		return;
	}

	if (pin_data->pin < PAD_GMAC0_MDC) {
		pin_reg->io_conf_reg = (pin_data->pin * A0N_GPO_PDA_CFG_OFFSET) +
					AON_GPO_PDA_0_5_CFG;
	}

	if (!of_property_read_u32(np, "starfive,pin-ioconfig", &value))
		pin_data->pin_config.io_config = value;

	list = of_get_property(np, "starfive,pinmux", &size);
	if (list) {
		pin_reg->func_sel_reg = be32_to_cpu(*list++);
		pin_reg->func_sel_shift = be32_to_cpu(*list++);
		pin_reg->func_sel_mask = be32_to_cpu(*list++);
		pin_data->pin_config.pinmux_func = be32_to_cpu(*list++);
	}

	list = of_get_property(np, "starfive,pin-syscon", &size);
	if (list) {
		pin_reg->syscon_reg = be32_to_cpu(*list++);
		pin_data->pin_config.syscon = be32_to_cpu(*list++);
	}

	if (pin_data->pin >= PAD_RGPIO0 && pin_data->pin <= PAD_RGPIO3) {
		pin_data->pin_config.gpio_num = pin_data->pin - 1;
		pin_reg->gpo_dout_reg = info->dout_reg_base;
		pin_reg->gpo_doen_reg = info->doen_reg_base;

		if (!of_property_read_u32(np, "starfive,pin-gpio-dout", &value))
			pin_data->pin_config.gpio_dout = value;

		if (!of_property_read_u32(np, "starfive,pin-gpio-doen", &value))
			pin_data->pin_config.gpio_doen = value;

		list_din = of_get_property(np, "starfive,pin-gpio-din", &size_din);
		if (list_din) {
			if (!size_din || size_din % pin_size) {
				dev_err(pctl->dev,
					"Invalid starfive,pin-gpio-din property in node\n");
				return;
			}
			pin_data->pin_config.gpio_din_num = size_din / pin_size;
			pin_data->pin_config.gpio_din_reg =
				devm_kcalloc(pctl->dev,
					     pin_data->pin_config.gpio_din_num,
					     sizeof(s32),
					     GFP_KERNEL);
			for (i = 0; i < pin_data->pin_config.gpio_din_num; i++) {
				value = be32_to_cpu(*list_din++);
				pin_data->pin_config.gpio_din_reg[i] = value;
			}
		}
	}
}

static int jh7110_aon_direction_input(struct gpio_chip *gc,
				      unsigned int gpio)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int shift;
	void __iomem *reg_doen;
	u32 mask;

	if (gpio < 0 || gpio >= gc->ngpio)
		return -EINVAL;

	shift  = GET_GPO_CFG_SHIFT(gpio);
	mask = AON_GPO_DOEN_MASK << shift;
	reg_doen = chip->padctl_base + AON_GPO_DOEN_CFG;

	raw_spin_lock_irqsave(&chip->lock, flags);
	pinctrl_set_reg(reg_doen, 1, shift, mask);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int jh7110_aon_direction_output(struct gpio_chip *gc,
				       unsigned int gpio,
				       int value)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int shift;
	void __iomem *reg_doen, *reg_dout;
	u32 mask_doen, mask_dout;

	if (gpio < 0 || gpio >= gc->ngpio)
		return -EINVAL;

	shift  = GET_GPO_CFG_SHIFT(gpio);
	mask_doen = AON_GPO_DOEN_MASK << shift;
	mask_dout = AON_GPO_DOUT_MASK << shift;
	reg_doen = chip->padctl_base + AON_GPO_DOEN_CFG;
	reg_dout = chip->padctl_base + AON_GPO_DOUT_CFG;

	raw_spin_lock_irqsave(&chip->lock, flags);
	pinctrl_set_reg(reg_doen, 0, shift, mask_doen);

	pinctrl_set_reg(reg_dout, value, shift, mask_dout);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int jh7110_aon_get_direction(struct gpio_chip *gc,
				    unsigned int gpio)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int doen;
	unsigned int shift;
	void __iomem *reg_doen;
	u32 mask;

	if (gpio < 0 || gpio >= gc->ngpio)
		return -EINVAL;

	shift  = GET_GPO_CFG_SHIFT(gpio);
	mask = AON_GPO_DOEN_MASK << shift;
	reg_doen = chip->padctl_base + AON_GPO_DOEN_CFG;

	raw_spin_lock_irqsave(&chip->lock, flags);
	doen = readl_relaxed(reg_doen);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	return !!(doen & mask);
}

static int jh7110_aon_get_value(struct gpio_chip *gc,
				unsigned int gpio)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	int value;

	if (gpio < 0 || gpio >= gc->ngpio)
		return -EINVAL;

	raw_spin_lock_irqsave(&chip->lock, flags);
	value = readl_relaxed(chip->padctl_base + AON_GPIO_DIN_REG);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	return (value >> gpio) & 0x1;
}

static void jh7110_aon_set_value(struct gpio_chip *gc,
				 unsigned int gpio, int value)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int shift;
	void __iomem *reg_dout;
	u32 mask;

	if (gpio < 0 || gpio >= gc->ngpio)
		return;

	shift  = GET_GPO_CFG_SHIFT(gpio);
	mask = AON_GPO_DOUT_MASK << shift;
	reg_dout = chip->padctl_base + AON_GPO_DOUT_CFG;

	raw_spin_lock_irqsave(&chip->lock, flags);
	pinctrl_set_reg(reg_dout, value, shift, mask);
	raw_spin_unlock_irqrestore(&chip->lock, flags);
}

static void jh7110_aon_irq_handler(struct irq_desc *desc)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_desc(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long mis;
	unsigned int pin;

	chained_irq_enter(chip, desc);

	mis = readl_relaxed(sfp->padctl_base + AON_GPIO_MIS_REG);
	for_each_set_bit(pin, &mis, AON_GPIO_NUM)
		generic_handle_domain_irq(sfp->gc.irq.domain, pin);

	chained_irq_exit(chip, desc);
}

static int jh7110_aon_init_hw(struct gpio_chip *gc)
{
	struct starfive_pinctrl *sfp = container_of(gc,
			struct starfive_pinctrl, gc);

	/* mask all GPIO interrupts */
	writel_relaxed(0, sfp->padctl_base + AON_GPIO_IE_REG);
	/* clear edge interrupt flags */
	writel_relaxed(0, sfp->padctl_base + AON_GPIO_IC_REG);
	writel_relaxed(0x0f, sfp->padctl_base + AON_GPIO_IC_REG);
	/* enable GPIO interrupts */
	writel_relaxed(1, sfp->padctl_base + AON_GPIO_EN_REG);
	return 0;
}

static int jh7110_aon_irq_set_type(struct irq_data *d,
				   unsigned int trigger)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *base = sfp->padctl_base;
	u32 mask = BIT(gpio);
	u32 irq_type, edge_both, polarity;
	unsigned long flags;

	switch (trigger) {
	case IRQ_TYPE_LEVEL_HIGH:
		irq_type  = 0;    /* 0: level triggered */
		edge_both = 0;    /* 0: ignored */
		polarity  = 0;    /* 0: high level */
		break;
	case IRQ_TYPE_LEVEL_LOW:
		irq_type  = 0;    /* 0: level triggered */
		edge_both = 0;    /* 0: ignored */
		polarity  = 1;    /* 1: low level */
		break;
	case IRQ_TYPE_EDGE_BOTH:
		irq_type  = mask; /* 1: edge triggered */
		edge_both = mask; /* 1: both edges */
		polarity  = 0;    /* 0: ignored */
		break;
	case IRQ_TYPE_EDGE_RISING:
		irq_type  = mask; /* 1: edge triggered */
		edge_both = 0;    /* 0: single edge */
		polarity  = mask; /* 1: rising edge */
		break;
	case IRQ_TYPE_EDGE_FALLING:
		irq_type  = mask; /* 1: edge triggered */
		edge_both = 0;    /* 0: single edge */
		polarity  = 0;    /* 0: falling edge */
		break;
	}
	if (trigger & IRQ_TYPE_EDGE_BOTH)
		irq_set_handler_locked(d, handle_edge_irq);
	else
		irq_set_handler_locked(d, handle_level_irq);

	raw_spin_lock_irqsave(&sfp->lock, flags);
	irq_type |= readl_relaxed(base + AON_GPIO_IS_REG) & ~mask;
	writel_relaxed(irq_type, base + AON_GPIO_IS_REG);

	edge_both |= readl_relaxed(base + AON_GPIO_IBE_REG) & ~mask;
	writel_relaxed(edge_both, base + AON_GPIO_IBE_REG);

	polarity |= readl_relaxed(base + AON_GPIO_IEV_REG) & ~mask;
	writel_relaxed(polarity, base + AON_GPIO_IEV_REG);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);

	sfp->trigger[gpio] = trigger;
	return 0;
}

static void jh7110_aon_irq_mask(struct irq_data *d)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *ie = sfp->padctl_base + AON_GPIO_IE_REG;
	u32 mask = BIT(gpio);
	unsigned long flags;
	u32 value;

	if (gpio < 0 || gpio >= sfp->gc.ngpio)
		return;

	raw_spin_lock_irqsave(&sfp->lock, flags);
	value = readl_relaxed(ie) & ~mask;
	writel_relaxed(value, ie);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);
}

static void jh7110_aon_irq_unmask(struct irq_data *d)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *ie = sfp->padctl_base + AON_GPIO_IE_REG;
	u32 mask = BIT(gpio);
	unsigned long flags;
	u32 value;

	if (gpio < 0 || gpio >= sfp->gc.ngpio)
		return;

	raw_spin_lock_irqsave(&sfp->lock, flags);
	value = readl_relaxed(ie) | mask;
	writel_relaxed(value, ie);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);
}

static void jh7110_aon_irq_ack(struct irq_data *d)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *ic = sfp->padctl_base + AON_GPIO_IC_REG;
	u32 mask = BIT(gpio);
	unsigned long flags;
	u32 value;

	if (gpio < 0 || gpio >= sfp->gc.ngpio)
		return;

	raw_spin_lock_irqsave(&sfp->lock, flags);
	value = readl_relaxed(ic) & ~mask;
	writel_relaxed(value, ic);
	writel_relaxed(value | mask, ic);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);
}

static void jh7110_aon_irq_mask_ack(struct irq_data *d)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *ie = sfp->padctl_base + AON_GPIO_IE_REG;
	void __iomem *ic = sfp->padctl_base + AON_GPIO_IC_REG;
	u32 mask = BIT(gpio);
	unsigned long flags;
	u32 value;

	if (gpio < 0 || gpio >= sfp->gc.ngpio)
		return;

	raw_spin_lock_irqsave(&sfp->lock, flags);
	value = readl_relaxed(ie) & ~mask;
	writel_relaxed(value, ie);

	value = readl_relaxed(ic) & ~mask;
	writel_relaxed(value, ic);
	writel_relaxed(value | mask, ic);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);
}

static struct irq_chip jh7110_aon_irqchip = {
	.name		= "starfive-jh7110-aon-gpio",
	.irq_ack	= jh7110_aon_irq_ack,
	.irq_mask_ack	= jh7110_aon_irq_mask_ack,
	.irq_set_type	= jh7110_aon_irq_set_type,
	.irq_mask	= jh7110_aon_irq_mask,
	.irq_unmask	= jh7110_aon_irq_unmask,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_SET_TYPE_MASKED,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int jh7110_aon_add_pin_ranges(struct gpio_chip *gc)
{
	struct starfive_pinctrl *sfp = container_of(gc,
			struct starfive_pinctrl, gc);

	sfp->gpios.name = sfp->gc.label;
	sfp->gpios.base = sfp->gc.base;
	/*
	 * sfp->gpios.pin_base depends on the chosen signal group
	 * and is set in starfive_probe()
	 */
	sfp->gpios.npins = AON_GPIO_NUM;
	sfp->gpios.gc = &sfp->gc;
	pinctrl_add_gpio_range(sfp->pctl_dev, &sfp->gpios);
	return 0;
}

static int jh7110_aon_gpio_register(struct platform_device *pdev,
				    struct starfive_pinctrl *pctl)
{
	struct device *dev = &pdev->dev;
	int ret, ngpio;
	int loop;

	ngpio = AON_GPIO_NUM;

	pctl->gc.direction_input = jh7110_aon_direction_input;
	pctl->gc.direction_output = jh7110_aon_direction_output;
	pctl->gc.get_direction = jh7110_aon_get_direction;
	pctl->gc.get = jh7110_aon_get_value;
	pctl->gc.set = jh7110_aon_set_value;
	pctl->gc.add_pin_ranges = jh7110_aon_add_pin_ranges;
	pctl->gc.base = MAX_GPIO;
	pctl->gc.ngpio = ngpio;
	pctl->gc.label = dev_name(dev);
	pctl->gc.parent = dev;
	pctl->gc.owner = THIS_MODULE;

	pctl->enabled = 0;

	platform_set_drvdata(pdev, pctl);

	jh7110_aon_irqchip.name = pctl->gc.label;

	pctl->gc.irq.chip = &jh7110_aon_irqchip;
	pctl->gc.irq.parent_handler = jh7110_aon_irq_handler;
	pctl->gc.irq.num_parents = 1;
	pctl->gc.irq.parents =
		devm_kcalloc(dev, pctl->gc.irq.num_parents,
			     sizeof(*pctl->gc.irq.parents), GFP_KERNEL);
	if (!pctl->gc.irq.parents)
		return -ENOMEM;
	pctl->gc.irq.default_type = IRQ_TYPE_NONE;
	pctl->gc.irq.handler = handle_bad_irq;
	pctl->gc.irq.init_hw = jh7110_aon_init_hw;

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	pctl->gc.irq.parents[0] = ret;

	ret = devm_gpiochip_add_data(dev, &pctl->gc, pctl);
	if (ret)
		return dev_err_probe(dev, ret, "could not register gpiochip\n");

	for (loop = 0; loop < ngpio; loop++) {
		unsigned int v;
		void __iomem *ie_reg = pctl->padctl_base +
				AON_GPIO_INPUT_ENABLE_REG + (loop << 2);

		v = readl_relaxed(ie_reg);
		v |= 0x1;
		writel_relaxed(v, ie_reg);
	}

	dev_info(dev, "StarFive AON GPIO chip registered %d GPIOs\n", ngpio);

	return 0;
}

static int jh7110_aon_pinconf_get(struct pinctrl_dev *pctldev,
				  unsigned int pin_id,
				  unsigned long *config)
{
	struct starfive_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct starfive_pinctrl_soc_info *info = pctl->info;
	const struct starfive_pin_reg *pin_reg = &pctl->pin_regs[pin_id];
	unsigned long flags;
	u32 value;

	if (pin_reg->io_conf_reg == -1) {
		dev_err(pctl->dev,
			"Pin(%s) does not support config function\n",
			info->pins[pin_id].name);
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&pctl->lock, flags);
	value = readl_relaxed(pctl->padctl_base + pin_reg->io_conf_reg);
	*config = value & 0xff;
	raw_spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static int jh7110_aon_pinconf_set(struct pinctrl_dev *pctldev,
				  unsigned int pin_id,
				  unsigned long *configs,
				  unsigned int num_configs)
{
	struct starfive_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct starfive_pinctrl_soc_info *info = pctl->info;
	const struct starfive_pin_reg *pin_reg = &pctl->pin_regs[pin_id];
	int i;
	u32 value;
	unsigned long flags;

	if (pin_reg->io_conf_reg == -1) {
		dev_err(pctl->dev,
			"Pin(%s) does not support config function\n",
			info->pins[pin_id].name);
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&pctl->lock, flags);
	for (i = 0; i < num_configs; i++) {
		value = readl_relaxed(pctl->padctl_base +
				      pin_reg->io_conf_reg);
		value = value | (configs[i] & 0xFF);
		writel_relaxed(value, pctl->padctl_base +
				pin_reg->io_conf_reg);
	}
	raw_spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static const struct starfive_pinctrl_soc_info jh7110_aon_pinctrl_info = {
	.pins = starfive_jh7110_aon_pinctrl_pads,
	.npins = ARRAY_SIZE(starfive_jh7110_aon_pinctrl_pads),
	.flags = 1,
	.dout_reg_base = AON_GPO_DOUT_CFG,
	.doen_reg_base = AON_GPO_DOEN_CFG,
	.din_reg_base = AON_GPI_DIN_CFG,
	.starfive_pinconf_get = jh7110_aon_pinconf_get,
	.starfive_pinconf_set = jh7110_aon_pinconf_set,
	.starfive_pmx_set_one_pin_mux = jh7110_aon_pmx_set_one_pin_mux,
	.starfive_gpio_register = jh7110_aon_gpio_register,
	.starfive_pinctrl_parse_pin = jh7110_aon_parse_pin_config,
};

static const struct of_device_id jh7110_aon_pinctrl_of_match[] = {
	{
		.compatible = "starfive,jh7110-aon-pinctrl",
		.data = &jh7110_aon_pinctrl_info,
	},
	{ /* sentinel */ }
};

static int jh7110_aon_pinctrl_probe(struct platform_device *pdev)
{
	const struct starfive_pinctrl_soc_info *pinctrl_info;

	pinctrl_info = of_device_get_match_data(&pdev->dev);
	if (!pinctrl_info)
		return -ENODEV;

	return starfive_pinctrl_probe(pdev, pinctrl_info);
}

static struct platform_driver jh7110_aon_pinctrl_driver = {
	.driver = {
		.name = "starfive-jh7110-aon-pinctrl",
		.of_match_table = of_match_ptr(jh7110_aon_pinctrl_of_match),
	},
	.probe = jh7110_aon_pinctrl_probe,
};

static int __init jh7110_aon_pinctrl_init(void)
{
	return platform_driver_register(&jh7110_aon_pinctrl_driver);
}
arch_initcall(jh7110_aon_pinctrl_init);

MODULE_DESCRIPTION("Pinctrl driver for StarFive JH7110 SoC aon controller");
MODULE_AUTHOR("Jenny Zhang <jenny.zhang@starfivetech.com>");
MODULE_AUTHOR("Jianlong Huang <jianlong.huang@starfivetech.com>");
MODULE_LICENSE("GPL v2");

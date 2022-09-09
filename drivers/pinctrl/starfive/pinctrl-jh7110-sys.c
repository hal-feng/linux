// SPDX-License-Identifier: GPL-2.0
/*
 * Pinctrl / GPIO driver for StarFive JH7110 SoC sys controller
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

#define SYS_GPO_DOEN_CFG		0x0
#define SYS_GPO_DOEN_MASK		GENMASK(5, 0)
#define SYS_GPO_DOUT_CFG		0x40
#define SYS_GPO_DOUT_MASK		GENMASK(6, 0)
#define SYS_GPI_DIN_CFG			0x80
#define SYS_GPI_DIN_MASK		GENMASK(6, 0)
#define SYS_GPIO_INPUT_ENABLE_REG	0x120

/* sys_iomux PIN 0-74 ioconfig reg */
#define SYS_GPO_PDA_0_74_CFG		0x120
/* sys_iomux PIN 89-94 ioconfig reg */
#define SYS_GPO_PDA_89_94_CFG		0x284
#define SYS_GPO_PDA_CFG_OFFSET		0x4

/* sys_iomux GPIO CTRL */
#define GPIO_EN				0xdc
#define GPIO_IS_LOW			0xe0
#define GPIO_IS_HIGH			0xe4
#define GPIO_IC_LOW			0xe8
#define GPIO_IC_HIGH			0xec
#define GPIO_IBE_LOW			0xf0
#define GPIO_IBE_HIGH			0xf4
#define GPIO_IEV_LOW			0xf8
#define GPIO_IEV_HIGH			0xfc
#define GPIO_IE_LOW			0x100
#define GPIO_IE_HIGH			0x104

/* read only */
#define GPIO_MIS_LOW			0x110
#define GPIO_MIS_HIGH			0x114
#define GPIO_DIN_LOW			0x118
#define GPIO_DIN_HIGH			0x11c

#define PADCFG_PAD_GMAC_SYSCON_SHIFT	0x0
#define PADCFG_PAD_GMAC_SYSCON_MASK	GENMASK(1, 0)

/* one dword include 4 gpios */
#define GPIO_NUM_SHIFT			2
#define GPIO_NUM_PER_REG		32
#define OFFSET_PER_REG			4
#define SYS_GPIO_NUM			64

enum starfive_jh7110_sys_pads {
	PAD_GPIO0	= 0,
	PAD_GPIO1	= 1,
	PAD_GPIO2	= 2,
	PAD_GPIO3	= 3,
	PAD_GPIO4	= 4,
	PAD_GPIO5	= 5,
	PAD_GPIO6	= 6,
	PAD_GPIO7	= 7,
	PAD_GPIO8	= 8,
	PAD_GPIO9	= 9,
	PAD_GPIO10	= 10,
	PAD_GPIO11	= 11,
	PAD_GPIO12	= 12,
	PAD_GPIO13	= 13,
	PAD_GPIO14	= 14,
	PAD_GPIO15	= 15,
	PAD_GPIO16	= 16,
	PAD_GPIO17	= 17,
	PAD_GPIO18	= 18,
	PAD_GPIO19	= 19,
	PAD_GPIO20	= 20,
	PAD_GPIO21	= 21,
	PAD_GPIO22	= 22,
	PAD_GPIO23	= 23,
	PAD_GPIO24	= 24,
	PAD_GPIO25	= 25,
	PAD_GPIO26	= 26,
	PAD_GPIO27	= 27,
	PAD_GPIO28	= 28,
	PAD_GPIO29	= 29,
	PAD_GPIO30	= 30,
	PAD_GPIO31	= 31,
	PAD_GPIO32	= 32,
	PAD_GPIO33	= 33,
	PAD_GPIO34	= 34,
	PAD_GPIO35	= 35,
	PAD_GPIO36	= 36,
	PAD_GPIO37	= 37,
	PAD_GPIO38	= 38,
	PAD_GPIO39	= 39,
	PAD_GPIO40	= 40,
	PAD_GPIO41	= 41,
	PAD_GPIO42	= 42,
	PAD_GPIO43	= 43,
	PAD_GPIO44	= 44,
	PAD_GPIO45	= 45,
	PAD_GPIO46	= 46,
	PAD_GPIO47	= 47,
	PAD_GPIO48	= 48,
	PAD_GPIO49	= 49,
	PAD_GPIO50	= 50,
	PAD_GPIO51	= 51,
	PAD_GPIO52	= 52,
	PAD_GPIO53	= 53,
	PAD_GPIO54	= 54,
	PAD_GPIO55	= 55,
	PAD_GPIO56	= 56,
	PAD_GPIO57	= 57,
	PAD_GPIO58	= 58,
	PAD_GPIO59	= 59,
	PAD_GPIO60	= 60,
	PAD_GPIO61	= 61,
	PAD_GPIO62	= 62,
	PAD_GPIO63	= 63,
	PAD_SD0_CLK	= 64,
	PAD_SD0_CMD	= 65,
	PAD_SD0_DATA0	= 66,
	PAD_SD0_DATA1	= 67,
	PAD_SD0_DATA2	= 68,
	PAD_SD0_DATA3	= 69,
	PAD_SD0_DATA4	= 70,
	PAD_SD0_DATA5	= 71,
	PAD_SD0_DATA6	= 72,
	PAD_SD0_DATA7	= 73,
	PAD_SD0_STRB	= 74,
	PAD_GMAC1_MDC	= 75,
	PAD_GMAC1_MDIO	= 76,
	PAD_GMAC1_RXD0	= 77,
	PAD_GMAC1_RXD1	= 78,
	PAD_GMAC1_RXD2	= 79,
	PAD_GMAC1_RXD3	= 80,
	PAD_GMAC1_RXDV	= 81,
	PAD_GMAC1_RXC	= 82,
	PAD_GMAC1_TXD0	= 83,
	PAD_GMAC1_TXD1	= 84,
	PAD_GMAC1_TXD2	= 85,
	PAD_GMAC1_TXD3	= 86,
	PAD_GMAC1_TXEN	= 87,
	PAD_GMAC1_TXC	= 88,
	PAD_QSPI_SCLK	= 89,
	PAD_QSPI_CSn0	= 90,
	PAD_QSPI_DATA0	= 91,
	PAD_QSPI_DATA1	= 92,
	PAD_QSPI_DATA2	= 93,
	PAD_QSPI_DATA3	= 94,
};

/* Pad names for the pinmux subsystem */
static const struct pinctrl_pin_desc starfive_jh7110_sys_pinctrl_pads[] = {
	STARFIVE_PINCTRL_PIN(PAD_GPIO0),
	STARFIVE_PINCTRL_PIN(PAD_GPIO1),
	STARFIVE_PINCTRL_PIN(PAD_GPIO2),
	STARFIVE_PINCTRL_PIN(PAD_GPIO3),
	STARFIVE_PINCTRL_PIN(PAD_GPIO4),
	STARFIVE_PINCTRL_PIN(PAD_GPIO5),
	STARFIVE_PINCTRL_PIN(PAD_GPIO6),
	STARFIVE_PINCTRL_PIN(PAD_GPIO7),
	STARFIVE_PINCTRL_PIN(PAD_GPIO8),
	STARFIVE_PINCTRL_PIN(PAD_GPIO9),
	STARFIVE_PINCTRL_PIN(PAD_GPIO10),
	STARFIVE_PINCTRL_PIN(PAD_GPIO11),
	STARFIVE_PINCTRL_PIN(PAD_GPIO12),
	STARFIVE_PINCTRL_PIN(PAD_GPIO13),
	STARFIVE_PINCTRL_PIN(PAD_GPIO14),
	STARFIVE_PINCTRL_PIN(PAD_GPIO15),
	STARFIVE_PINCTRL_PIN(PAD_GPIO16),
	STARFIVE_PINCTRL_PIN(PAD_GPIO17),
	STARFIVE_PINCTRL_PIN(PAD_GPIO18),
	STARFIVE_PINCTRL_PIN(PAD_GPIO19),
	STARFIVE_PINCTRL_PIN(PAD_GPIO20),
	STARFIVE_PINCTRL_PIN(PAD_GPIO21),
	STARFIVE_PINCTRL_PIN(PAD_GPIO22),
	STARFIVE_PINCTRL_PIN(PAD_GPIO23),
	STARFIVE_PINCTRL_PIN(PAD_GPIO24),
	STARFIVE_PINCTRL_PIN(PAD_GPIO25),
	STARFIVE_PINCTRL_PIN(PAD_GPIO26),
	STARFIVE_PINCTRL_PIN(PAD_GPIO27),
	STARFIVE_PINCTRL_PIN(PAD_GPIO28),
	STARFIVE_PINCTRL_PIN(PAD_GPIO29),
	STARFIVE_PINCTRL_PIN(PAD_GPIO30),
	STARFIVE_PINCTRL_PIN(PAD_GPIO31),
	STARFIVE_PINCTRL_PIN(PAD_GPIO32),
	STARFIVE_PINCTRL_PIN(PAD_GPIO33),
	STARFIVE_PINCTRL_PIN(PAD_GPIO34),
	STARFIVE_PINCTRL_PIN(PAD_GPIO35),
	STARFIVE_PINCTRL_PIN(PAD_GPIO36),
	STARFIVE_PINCTRL_PIN(PAD_GPIO37),
	STARFIVE_PINCTRL_PIN(PAD_GPIO38),
	STARFIVE_PINCTRL_PIN(PAD_GPIO39),
	STARFIVE_PINCTRL_PIN(PAD_GPIO40),
	STARFIVE_PINCTRL_PIN(PAD_GPIO41),
	STARFIVE_PINCTRL_PIN(PAD_GPIO42),
	STARFIVE_PINCTRL_PIN(PAD_GPIO43),
	STARFIVE_PINCTRL_PIN(PAD_GPIO44),
	STARFIVE_PINCTRL_PIN(PAD_GPIO45),
	STARFIVE_PINCTRL_PIN(PAD_GPIO46),
	STARFIVE_PINCTRL_PIN(PAD_GPIO47),
	STARFIVE_PINCTRL_PIN(PAD_GPIO48),
	STARFIVE_PINCTRL_PIN(PAD_GPIO49),
	STARFIVE_PINCTRL_PIN(PAD_GPIO50),
	STARFIVE_PINCTRL_PIN(PAD_GPIO51),
	STARFIVE_PINCTRL_PIN(PAD_GPIO52),
	STARFIVE_PINCTRL_PIN(PAD_GPIO53),
	STARFIVE_PINCTRL_PIN(PAD_GPIO54),
	STARFIVE_PINCTRL_PIN(PAD_GPIO55),
	STARFIVE_PINCTRL_PIN(PAD_GPIO56),
	STARFIVE_PINCTRL_PIN(PAD_GPIO57),
	STARFIVE_PINCTRL_PIN(PAD_GPIO58),
	STARFIVE_PINCTRL_PIN(PAD_GPIO59),
	STARFIVE_PINCTRL_PIN(PAD_GPIO60),
	STARFIVE_PINCTRL_PIN(PAD_GPIO61),
	STARFIVE_PINCTRL_PIN(PAD_GPIO62),
	STARFIVE_PINCTRL_PIN(PAD_GPIO63),
	STARFIVE_PINCTRL_PIN(PAD_SD0_CLK),
	STARFIVE_PINCTRL_PIN(PAD_SD0_CMD),
	STARFIVE_PINCTRL_PIN(PAD_SD0_DATA0),
	STARFIVE_PINCTRL_PIN(PAD_SD0_DATA1),
	STARFIVE_PINCTRL_PIN(PAD_SD0_DATA2),
	STARFIVE_PINCTRL_PIN(PAD_SD0_DATA3),
	STARFIVE_PINCTRL_PIN(PAD_SD0_DATA4),
	STARFIVE_PINCTRL_PIN(PAD_SD0_DATA5),
	STARFIVE_PINCTRL_PIN(PAD_SD0_DATA6),
	STARFIVE_PINCTRL_PIN(PAD_SD0_DATA7),
	STARFIVE_PINCTRL_PIN(PAD_SD0_STRB),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_MDC),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_MDIO),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_RXD0),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_RXD1),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_RXD2),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_RXD3),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_RXDV),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_RXC),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_TXD0),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_TXD1),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_TXD2),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_TXD3),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_TXEN),
	STARFIVE_PINCTRL_PIN(PAD_GMAC1_TXC),
	STARFIVE_PINCTRL_PIN(PAD_QSPI_SCLK),
	STARFIVE_PINCTRL_PIN(PAD_QSPI_CSn0),
	STARFIVE_PINCTRL_PIN(PAD_QSPI_DATA0),
	STARFIVE_PINCTRL_PIN(PAD_QSPI_DATA1),
	STARFIVE_PINCTRL_PIN(PAD_QSPI_DATA2),
	STARFIVE_PINCTRL_PIN(PAD_QSPI_DATA3),
};

static int jh7110_sys_direction_input(struct gpio_chip *gc,
				      unsigned int gpio)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int offset, shift;
	void __iomem *reg_doen;
	u32 mask;

	if (gpio >= gc->ngpio)
		return -EINVAL;

	offset = GET_GPO_REG_OFFSET(gpio);
	shift  = GET_GPO_CFG_SHIFT(gpio);
	mask = SYS_GPO_DOEN_MASK << shift;
	reg_doen = chip->padctl_base + SYS_GPO_DOEN_CFG + offset;

	raw_spin_lock_irqsave(&chip->lock, flags);
	pinctrl_set_reg(reg_doen, 1, shift, mask);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int jh7110_sys_direction_output(struct gpio_chip *gc,
				       unsigned int gpio,
				       int value)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int offset, shift;
	void __iomem *reg_doen, *reg_dout;
	u32 mask_doen, mask_dout;

	if (gpio >= gc->ngpio)
		return -EINVAL;

	offset = GET_GPO_REG_OFFSET(gpio);
	shift  = GET_GPO_CFG_SHIFT(gpio);
	mask_doen = SYS_GPO_DOEN_MASK << shift;
	mask_dout = SYS_GPO_DOUT_MASK << shift;
	reg_doen = chip->padctl_base + SYS_GPO_DOEN_CFG + offset;
	reg_dout = chip->padctl_base + SYS_GPO_DOUT_CFG + offset;

	raw_spin_lock_irqsave(&chip->lock, flags);
	pinctrl_set_reg(reg_doen, 0, shift, mask_doen);

	pinctrl_set_reg(reg_dout, value, shift, mask_dout);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int jh7110_sys_get_direction(struct gpio_chip *gc,
				    unsigned int gpio)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int doen;
	unsigned int offset, shift;
	void __iomem *reg_doen;
	u32 mask;

	if (gpio >= gc->ngpio)
		return -EINVAL;

	offset = GET_GPO_REG_OFFSET(gpio);
	shift  = GET_GPO_CFG_SHIFT(gpio);
	mask = SYS_GPO_DOEN_MASK << shift;
	reg_doen = chip->padctl_base + SYS_GPO_DOEN_CFG + offset;

	raw_spin_lock_irqsave(&chip->lock, flags);
	doen = readl_relaxed(reg_doen);
	raw_spin_unlock_irqrestore(&chip->lock, flags);

	return !!(doen & mask);
}

static int jh7110_sys_get_value(struct gpio_chip *gc,
				unsigned int gpio)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	int value;
	int tmp;

	if (gpio >= gc->ngpio)
		return -EINVAL;

	if (gpio < GPIO_NUM_PER_REG) {
		value = readl_relaxed(chip->padctl_base + GPIO_DIN_LOW);
		tmp = 0;
	} else {
		value = readl_relaxed(chip->padctl_base + GPIO_DIN_HIGH);
		tmp = GPIO_NUM_PER_REG;
	}
	return (value >> (gpio - tmp)) & 0x1;
}

static void jh7110_sys_set_value(struct gpio_chip *gc,
				 unsigned int gpio,
				 int value)
{
	struct starfive_pinctrl *chip = gpiochip_get_data(gc);
	unsigned long flags;
	unsigned int offset, shift;
	void __iomem *reg_dout;
	u32 mask;

	if (gpio >= gc->ngpio)
		return;

	offset = GET_GPO_REG_OFFSET(gpio);
	shift  = GET_GPO_CFG_SHIFT(gpio);
	mask = SYS_GPO_DOUT_MASK << shift;
	reg_dout = chip->padctl_base + SYS_GPO_DOUT_CFG + offset;

	raw_spin_lock_irqsave(&chip->lock, flags);
	pinctrl_set_reg(reg_dout, value, shift, mask);
	raw_spin_unlock_irqrestore(&chip->lock, flags);
}

static int jh7110_sys_irq_set_type(struct irq_data *d,
				   unsigned int trigger)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *base = sfp->padctl_base +
			OFFSET_PER_REG * (gpio / GPIO_NUM_PER_REG);
	u32 mask = BIT(gpio % GPIO_NUM_PER_REG);
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
	irq_type |= readl_relaxed(base + GPIO_IS_LOW) & ~mask;
	writel_relaxed(irq_type, base + GPIO_IS_LOW);

	edge_both |= readl_relaxed(base + GPIO_IBE_LOW) & ~mask;
	writel_relaxed(edge_both, base + GPIO_IBE_LOW);

	polarity |= readl_relaxed(base + GPIO_IEV_LOW) & ~mask;
	writel_relaxed(polarity, base + GPIO_IEV_LOW);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);

	sfp->trigger[gpio] = trigger;
	return 0;
}

/* chained_irq_{enter,exit} already mask the parent */
static void jh7110_sys_irq_mask(struct irq_data *d)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *ie = sfp->padctl_base + GPIO_IE_LOW +
			OFFSET_PER_REG * (gpio / GPIO_NUM_PER_REG);
	u32 mask = BIT(gpio % GPIO_NUM_PER_REG);
	unsigned long flags;
	u32 value;

	if (gpio < 0 || gpio >= sfp->gc.ngpio)
		return;

	raw_spin_lock_irqsave(&sfp->lock, flags);
	value = readl_relaxed(ie) & ~mask;
	writel_relaxed(value, ie);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);
}

static void jh7110_sys_irq_unmask(struct irq_data *d)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *ie = sfp->padctl_base + GPIO_IE_LOW +
			OFFSET_PER_REG * (gpio / GPIO_NUM_PER_REG);
	u32 mask = BIT(gpio % GPIO_NUM_PER_REG);
	unsigned long flags;
	u32 value;

	if (gpio < 0 || gpio >= sfp->gc.ngpio)
		return;

	raw_spin_lock_irqsave(&sfp->lock, flags);
	value = readl_relaxed(ie) | mask;
	writel_relaxed(value, ie);
	raw_spin_unlock_irqrestore(&sfp->lock, flags);
}

static void jh7110_sys_irq_ack(struct irq_data *d)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *ic = sfp->padctl_base + GPIO_IC_LOW +
			OFFSET_PER_REG * (gpio / GPIO_NUM_PER_REG);
	u32 mask = BIT(gpio % GPIO_NUM_PER_REG);
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

static void jh7110_sys_irq_mask_ack(struct irq_data *d)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_data(d);
	irq_hw_number_t gpio = irqd_to_hwirq(d);
	void __iomem *ie = sfp->padctl_base + GPIO_IE_LOW +
			OFFSET_PER_REG * (gpio / GPIO_NUM_PER_REG);
	void __iomem *ic = sfp->padctl_base + GPIO_IC_LOW +
			OFFSET_PER_REG * (gpio / GPIO_NUM_PER_REG);
	u32 mask = BIT(gpio % GPIO_NUM_PER_REG);
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

static struct irq_chip jh7110_sys_irqchip = {
	.name		= "starfive-jh7110-sys-gpio",
	.irq_ack	= jh7110_sys_irq_ack,
	.irq_mask_ack	= jh7110_sys_irq_mask_ack,
	.irq_set_type	= jh7110_sys_irq_set_type,
	.irq_mask	= jh7110_sys_irq_mask,
	.irq_unmask	= jh7110_sys_irq_unmask,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_SET_TYPE_MASKED,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void jh7110_sys_irq_handler(struct irq_desc *desc)
{
	struct starfive_pinctrl *sfp = starfive_from_irq_desc(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long mis;
	unsigned int pin;

	chained_irq_enter(chip, desc);

	mis = readl_relaxed(sfp->padctl_base + GPIO_MIS_LOW);
	for_each_set_bit(pin, &mis, GPIO_NUM_PER_REG)
		generic_handle_domain_irq(sfp->gc.irq.domain, pin);

	mis = readl_relaxed(sfp->padctl_base + GPIO_MIS_HIGH);
	for_each_set_bit(pin, &mis, GPIO_NUM_PER_REG)
		generic_handle_domain_irq(sfp->gc.irq.domain,
					  pin + GPIO_NUM_PER_REG);

	chained_irq_exit(chip, desc);
}

static int jh7110_sys_init_hw(struct gpio_chip *gc)
{
	struct starfive_pinctrl *sfp = container_of(gc,
			struct starfive_pinctrl, gc);

	/* mask all GPIO interrupts */
	writel_relaxed(0, sfp->padctl_base + GPIO_IE_LOW);
	writel_relaxed(0, sfp->padctl_base + GPIO_IE_HIGH);
	/* clear edge interrupt flags */
	writel_relaxed(0, sfp->padctl_base + GPIO_IC_LOW);
	writel_relaxed(0, sfp->padctl_base + GPIO_IC_HIGH);
	writel_relaxed(~0U, sfp->padctl_base + GPIO_IC_LOW);
	writel_relaxed(~0U, sfp->padctl_base + GPIO_IC_HIGH);
	/* enable GPIO interrupts */
	writel_relaxed(1, sfp->padctl_base + GPIO_EN);
	return 0;
}

static int jh7110_sys_add_pin_ranges(struct gpio_chip *gc)
{
	struct starfive_pinctrl *sfp = container_of(gc,
			struct starfive_pinctrl, gc);

	sfp->gpios.name = sfp->gc.label;
	sfp->gpios.base = sfp->gc.base;
	/*
	 * sfp->gpios.pin_base depends on the chosen signal group
	 * and is set in starfive_probe()
	 */
	sfp->gpios.npins = SYS_GPIO_NUM;
	sfp->gpios.gc = &sfp->gc;
	pinctrl_add_gpio_range(sfp->pctl_dev, &sfp->gpios);
	return 0;
}

static int jh7110_sys_gpio_register(struct platform_device *pdev,
				    struct starfive_pinctrl *pctl)
{
	struct device *dev = &pdev->dev;
	int ret, ngpio;
	int loop;

	ngpio = SYS_GPIO_NUM;

	pctl->gc.direction_input = jh7110_sys_direction_input;
	pctl->gc.direction_output = jh7110_sys_direction_output;
	pctl->gc.get_direction = jh7110_sys_get_direction;
	pctl->gc.get = jh7110_sys_get_value;
	pctl->gc.set = jh7110_sys_set_value;
	pctl->gc.add_pin_ranges = jh7110_sys_add_pin_ranges;
	pctl->gc.base = 0;
	pctl->gc.ngpio = ngpio;
	pctl->gc.label = dev_name(dev);
	pctl->gc.parent = dev;
	pctl->gc.owner = THIS_MODULE;
	pctl->enabled = 0;

	platform_set_drvdata(pdev, pctl);

	jh7110_sys_irqchip.name = pctl->gc.label;

	pctl->gc.irq.chip = &jh7110_sys_irqchip;
	pctl->gc.irq.parent_handler = jh7110_sys_irq_handler;
	pctl->gc.irq.num_parents = 1;
	pctl->gc.irq.parents = devm_kcalloc(dev, pctl->gc.irq.num_parents,
					    sizeof(*pctl->gc.irq.parents),
					    GFP_KERNEL);
	if (!pctl->gc.irq.parents)
		return -ENOMEM;
	pctl->gc.irq.default_type = IRQ_TYPE_NONE;
	pctl->gc.irq.handler = handle_bad_irq;
	pctl->gc.irq.init_hw = jh7110_sys_init_hw;

	if (IS_ENABLED(CONFIG_PM))
		pm_runtime_enable(dev);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	pctl->gc.irq.parents[0] = ret;

	ret = devm_gpiochip_add_data(dev, &pctl->gc, pctl);
	if (ret)
		return dev_err_probe(dev, ret,
				"could not register gpiochip\n");

	for (loop = 0; loop < SYS_GPIO_NUM; loop++) {
		unsigned int v;
		void __iomem *reg_ie = pctl->padctl_base +
			SYS_GPIO_INPUT_ENABLE_REG + (loop << 2);

		v = readl_relaxed(reg_ie);
		v |= 0x1;
		writel_relaxed(v, reg_ie);
	}

	dev_info(dev, "StarFive SYS GPIO chip registered %d GPIOs\n", ngpio);

	return 0;
}

static int jh7110_pinconf_get(struct pinctrl_dev *pctldev,
			      unsigned int pin_id,
			      unsigned long *config)
{
	struct starfive_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	const struct starfive_pinctrl_soc_info *info = pctl->info;
	const struct starfive_pin_reg *pin_reg = &pctl->pin_regs[pin_id];
	u32 value;

	if (pin_reg->io_conf_reg == -1) {
		dev_err(pctl->dev,
			"Pin(%s) does not support config function\n",
			info->pins[pin_id].name);
		return -EINVAL;
	}

	value = readl_relaxed(pctl->padctl_base + pin_reg->io_conf_reg);
	*config = value & 0xff;
	return 0;
}

static int jh7110_pinconf_set(struct pinctrl_dev *pctldev,
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

static int jh7110_sys_pmx_set_one_pin_mux(struct starfive_pinctrl *pctl,
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
				  SYS_GPO_DOUT_MASK << shift, pin_config->gpio_dout << shift);
	}

	if (pin_reg->gpo_doen_reg != -1) {
		pinctrl_write_reg(pctl->padctl_base + pin_reg->gpo_doen_reg,
				  SYS_GPO_DOEN_MASK << shift, pin_config->gpio_doen << shift);
	}

	for (i = 0; i < pin_config->gpio_din_num; i++) {
		n = pin_config->gpio_din_reg[i] >> 2;
		shift = (pin_config->gpio_din_reg[i] & 3) << 3;
		pinctrl_write_reg(pctl->padctl_base + info->din_reg_base + n * 4,
				  SYS_GPI_DIN_MASK << shift, (gpio + 2) << shift);
	}

	if (pin_reg->syscon_reg != -1) {
		pinctrl_set_reg(pctl->padctl_base + pin_reg->syscon_reg,
				pin_config->syscon, PADCFG_PAD_GMAC_SYSCON_SHIFT,
				PADCFG_PAD_GMAC_SYSCON_MASK);
	}

	if (pin_reg->pad_sel_reg != -1) {
		pinctrl_set_reg(pctl->padctl_base + pin_reg->pad_sel_reg,
				pin_config->padmux_func,
				pin_reg->pad_sel_shift,
				pin_reg->pad_sel_mask);
	}
	raw_spin_unlock_irqrestore(&pctl->lock, flags);

	return 0;
}

static void jh7110_sys_parse_pin_config(struct starfive_pinctrl *pctl,
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
	int n;

	pin_size = sizeof(u32);
	*pins_id = be32_to_cpu(*list);
	pin_reg = &pctl->pin_regs[*pins_id];
	pin_data->pin = *pins_id;

	if (pin_data->pin > PAD_QSPI_DATA3) {
		dev_err(pctl->dev, "err pin num = %d\n", pin_data->pin);
		return;
	}

	if (pin_data->pin < PAD_GMAC1_MDC) {
		pin_reg->io_conf_reg = (pin_data->pin * SYS_GPO_PDA_CFG_OFFSET)
			+ SYS_GPO_PDA_0_74_CFG;
	} else if (pin_data->pin > PAD_GMAC1_TXC) {
		pin_reg->io_conf_reg = (pin_data->pin * SYS_GPO_PDA_CFG_OFFSET)
			+ SYS_GPO_PDA_89_94_CFG;
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

	list = of_get_property(np, "starfive,padmux", &size);
	if (list) {
		pin_reg->pad_sel_reg = be32_to_cpu(*list++);
		pin_reg->pad_sel_shift = be32_to_cpu(*list++);
		pin_reg->pad_sel_mask = be32_to_cpu(*list++);
		pin_data->pin_config.padmux_func = be32_to_cpu(*list++);
	}

	list = of_get_property(np, "starfive,pin-syscon", &size);
	if (list) {
		pin_reg->syscon_reg = be32_to_cpu(*list++);
		pin_data->pin_config.syscon = be32_to_cpu(*list++);
	}

	if (pin_data->pin < PAD_SD0_CLK) {
		pin_data->pin_config.gpio_num = pin_data->pin;
		n = pin_data->pin_config.gpio_num >> GPIO_NUM_SHIFT;

		if (!of_property_read_u32(np, "starfive,pin-gpio-dout", &value)) {
			pin_data->pin_config.gpio_dout = value;
			pin_reg->gpo_dout_reg = info->dout_reg_base + n * 4;
		}

		if (!of_property_read_u32(np, "starfive,pin-gpio-doen", &value)) {
			pin_data->pin_config.gpio_doen = value;
			pin_reg->gpo_doen_reg = info->doen_reg_base + n * 4;
		}

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

static const struct starfive_pinctrl_soc_info jh7110_sys_pinctrl_info = {
	.pins = starfive_jh7110_sys_pinctrl_pads,
	.npins = ARRAY_SIZE(starfive_jh7110_sys_pinctrl_pads),
	.flags = 1,
	.dout_reg_base = SYS_GPO_DOUT_CFG,
	.doen_reg_base = SYS_GPO_DOEN_CFG,
	.din_reg_base = SYS_GPI_DIN_CFG,
	.starfive_pinconf_get = jh7110_pinconf_get,
	.starfive_pinconf_set = jh7110_pinconf_set,
	.starfive_pmx_set_one_pin_mux = jh7110_sys_pmx_set_one_pin_mux,
	.starfive_gpio_register = jh7110_sys_gpio_register,
	.starfive_pinctrl_parse_pin = jh7110_sys_parse_pin_config,
};

static const struct of_device_id jh7110_sys_pinctrl_of_match[] = {
	{
		.compatible = "starfive,jh7110-sys-pinctrl",
		.data = &jh7110_sys_pinctrl_info,
	},
	{ /* sentinel */ }
};

static int jh7110_sys_pinctrl_probe(struct platform_device *pdev)
{
	const struct starfive_pinctrl_soc_info *pinctrl_info;

	pinctrl_info = of_device_get_match_data(&pdev->dev);
	if (!pinctrl_info)
		return -ENODEV;

	return starfive_pinctrl_probe(pdev, pinctrl_info);
}

static struct platform_driver jh7110_sys_pinctrl_driver = {
	.driver = {
		.name = "starfive-jh7110-sys-pinctrl",
		.of_match_table = of_match_ptr(jh7110_sys_pinctrl_of_match),
	},
	.probe = jh7110_sys_pinctrl_probe,
};

static int __init jh7110_sys_pinctrl_init(void)
{
	return platform_driver_register(&jh7110_sys_pinctrl_driver);
}
arch_initcall(jh7110_sys_pinctrl_init);

MODULE_DESCRIPTION("Pinctrl driver for StarFive JH7110 SoC sys controller");
MODULE_AUTHOR("Jenny Zhang <jenny.zhang@starfivetech.com>");
MODULE_AUTHOR("Jianlong Huang <jianlong.huang@starfivetech.com>");
MODULE_LICENSE("GPL v2");

/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __CLK_STARFIVE_H
#define __CLK_STARFIVE_H

#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/spinlock.h>

/* register fields */
#define STARFIVE_CLK_ENABLE	BIT(31)
#define STARFIVE_CLK_INVERT	BIT(30)
#define STARFIVE_CLK_MUX_MASK	GENMASK(27, 24)
#define STARFIVE_CLK_MUX_SHIFT	24
#define STARFIVE_CLK_DIV_MASK	GENMASK(23, 0)
#define STARFIVE_CLK_FRAC_MASK	GENMASK(15, 8)
#define STARFIVE_CLK_FRAC_SHIFT	8
#define STARFIVE_CLK_INT_MASK	GENMASK(7, 0)

/* fractional divider min/max */
#define STARFIVE_CLK_FRAC_MIN	100UL
#define STARFIVE_CLK_FRAC_MAX	25599UL

/* clock data */
struct starfive_clk_data {
	const char *name;
	unsigned long flags;
	u32 max;
	u8 parents[4];
};

#define STARFIVE_GATE(_idx, _name, _flags, _parent) [_idx] = {			\
	.name = _name,								\
	.flags = CLK_SET_RATE_PARENT | (_flags),				\
	.max = STARFIVE_CLK_ENABLE,						\
	.parents = { [0] = _parent },						\
}

#define STARFIVE__DIV(_idx, _name, _max, _parent) [_idx] = {			\
	.name = _name,								\
	.flags = 0,								\
	.max = _max,								\
	.parents = { [0] = _parent },						\
}

#define STARFIVE_GDIV(_idx, _name, _flags, _max, _parent) [_idx] = {		\
	.name = _name,								\
	.flags = _flags,							\
	.max = STARFIVE_CLK_ENABLE | (_max),					\
	.parents = { [0] = _parent },						\
}

#define STARFIVE_FDIV(_idx, _name, _parent) [_idx] = {				\
	.name = _name,								\
	.flags = 0,								\
	.max = STARFIVE_CLK_FRAC_MAX,						\
	.parents = { [0] = _parent },						\
}

#define STARFIVE__MUX(_idx, _name, _nparents, ...) [_idx] = {			\
	.name = _name,								\
	.flags = 0,								\
	.max = ((_nparents) - 1) << STARFIVE_CLK_MUX_SHIFT,			\
	.parents = { __VA_ARGS__ },						\
}

#define STARFIVE_GMUX(_idx, _name, _flags, _nparents, ...) [_idx] = {		\
	.name = _name,								\
	.flags = _flags,							\
	.max = STARFIVE_CLK_ENABLE |						\
		(((_nparents) - 1) << STARFIVE_CLK_MUX_SHIFT),			\
	.parents = { __VA_ARGS__ },						\
}

#define STARFIVE_MDIV(_idx, _name, _max, _nparents, ...) [_idx] = {		\
	.name = _name,								\
	.flags = 0,								\
	.max = (((_nparents) - 1) << STARFIVE_CLK_MUX_SHIFT) | (_max),		\
	.parents = { __VA_ARGS__ },						\
}

#define STARFIVE__GMD(_idx, _name, _flags, _max, _nparents, ...) [_idx] = {	\
	.name = _name,								\
	.flags = _flags,							\
	.max = STARFIVE_CLK_ENABLE |						\
		(((_nparents) - 1) << STARFIVE_CLK_MUX_SHIFT) | (_max),		\
	.parents = { __VA_ARGS__ },						\
}

#define STARFIVE__INV(_idx, _name, _parent) [_idx] = {				\
	.name = _name,								\
	.flags = CLK_SET_RATE_PARENT,						\
	.max = STARFIVE_CLK_INVERT,						\
	.parents = { [0] = _parent },						\
}

struct starfive_clk {
	struct clk_hw hw;
	unsigned int idx;
	unsigned int max_div;
};

struct starfive_clk_priv {
	struct device *dev;
	struct regmap *regmap;
	struct clk_hw *pll[3];
	struct starfive_clk reg[];
};

const struct clk_ops *starfive_clk_ops(u32 max);

#endif

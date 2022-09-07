// SPDX-License-Identifier: GPL-2.0
/*
 * StarFive Clock Generator Driver
 *
 * Copyright (C) 2021-2022 Emil Renner Berthing <kernel@esmil.dk>
 */

#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include "clk-starfive.h"

static struct starfive_clk *starfive_clk_from(struct clk_hw *hw)
{
	return container_of(hw, struct starfive_clk, hw);
}

static struct starfive_clk_priv *starfive_priv_from(struct starfive_clk *clk)
{
	return container_of(clk, struct starfive_clk_priv, reg[clk->idx]);
}

static u32 starfive_clk_reg_get(struct starfive_clk *clk)
{
	struct starfive_clk_priv *priv = starfive_priv_from(clk);
	unsigned int reg = sizeof(u32) * clk->idx;
	unsigned int value;
	int ret;

	ret = regmap_read(priv->regmap, reg, &value);
	if (ret) {
		dev_warn(priv->dev, "Failed to read clock register: %d\n", ret);
		value = 0;
	}

	return value;
}

static int starfive_clk_enable(struct clk_hw *hw)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	struct starfive_clk_priv *priv = starfive_priv_from(clk);
	unsigned int reg = sizeof(u32) * clk->idx;

	return regmap_update_bits(priv->regmap, reg,
				  STARFIVE_CLK_ENABLE, STARFIVE_CLK_ENABLE);
}

static void starfive_clk_disable(struct clk_hw *hw)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	struct starfive_clk_priv *priv = starfive_priv_from(clk);
	unsigned int reg = sizeof(u32) * clk->idx;

	regmap_update_bits(priv->regmap, reg, STARFIVE_CLK_ENABLE, 0);
}

static int starfive_clk_is_enabled(struct clk_hw *hw)
{
	struct starfive_clk *clk = starfive_clk_from(hw);

	return !!(starfive_clk_reg_get(clk) & STARFIVE_CLK_ENABLE);
}

static unsigned long starfive_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	u32 div = starfive_clk_reg_get(clk) & STARFIVE_CLK_DIV_MASK;

	return div ? parent_rate / div : 0;
}

static int starfive_clk_determine_rate(struct clk_hw *hw,
				       struct clk_rate_request *req)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	unsigned long parent = req->best_parent_rate;
	unsigned long rate = clamp(req->rate, req->min_rate, req->max_rate);
	unsigned long div = min_t(unsigned long, DIV_ROUND_UP(parent, rate), clk->max_div);
	unsigned long result = parent / div;

	/*
	 * we want the result clamped by min_rate and max_rate if possible:
	 * case 1: div hits the max divider value, which means it's less than
	 * parent / rate, so the result is greater than rate and min_rate in
	 * particular. we can't do anything about result > max_rate because the
	 * divider doesn't go any further.
	 * case 2: div = DIV_ROUND_UP(parent, rate) which means the result is
	 * always lower or equal to rate and max_rate. however the result may
	 * turn out lower than min_rate, but then the next higher rate is fine:
	 *   div - 1 = ceil(parent / rate) - 1 < parent / rate
	 * and thus
	 *   min_rate <= rate < parent / (div - 1)
	 */
	if (result < req->min_rate && div > 1)
		result = parent / (div - 1);

	req->rate = result;
	return 0;
}

static int starfive_clk_set_rate(struct clk_hw *hw,
				 unsigned long rate,
				 unsigned long parent_rate)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	struct starfive_clk_priv *priv = starfive_priv_from(clk);
	unsigned int reg = sizeof(u32) * clk->idx;
	unsigned long div = clamp(DIV_ROUND_CLOSEST(parent_rate, rate),
				  1UL, (unsigned long)clk->max_div);

	return regmap_update_bits(priv->regmap, reg, STARFIVE_CLK_DIV_MASK, div);
}

static unsigned long starfive_clk_frac_recalc_rate(struct clk_hw *hw,
						   unsigned long parent_rate)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	u32 reg = starfive_clk_reg_get(clk);
	unsigned long div100 = 100 * (reg & STARFIVE_CLK_INT_MASK) +
			       ((reg & STARFIVE_CLK_FRAC_MASK) >> STARFIVE_CLK_FRAC_SHIFT);

	return (div100 >= STARFIVE_CLK_FRAC_MIN) ? 100 * parent_rate / div100 : 0;
}

static int starfive_clk_frac_determine_rate(struct clk_hw *hw,
					    struct clk_rate_request *req)
{
	unsigned long parent100 = 100 * req->best_parent_rate;
	unsigned long rate = clamp(req->rate, req->min_rate, req->max_rate);
	unsigned long div100 = clamp(DIV_ROUND_CLOSEST(parent100, rate),
				     STARFIVE_CLK_FRAC_MIN, STARFIVE_CLK_FRAC_MAX);
	unsigned long result = parent100 / div100;

	/* clamp the result as in starfive_clk_determine_rate() above */
	if (result > req->max_rate && div100 < STARFIVE_CLK_FRAC_MAX)
		result = parent100 / (div100 + 1);
	if (result < req->min_rate && div100 > STARFIVE_CLK_FRAC_MIN)
		result = parent100 / (div100 - 1);

	req->rate = result;
	return 0;
}

static int starfive_clk_frac_set_rate(struct clk_hw *hw,
				      unsigned long rate,
				      unsigned long parent_rate)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	struct starfive_clk_priv *priv = starfive_priv_from(clk);
	unsigned int reg = sizeof(u32) * clk->idx;
	unsigned long div100 = clamp(DIV_ROUND_CLOSEST(100 * parent_rate, rate),
				     STARFIVE_CLK_FRAC_MIN, STARFIVE_CLK_FRAC_MAX);
	u32 value = ((div100 % 100) << STARFIVE_CLK_FRAC_SHIFT) | (div100 / 100);

	return regmap_update_bits(priv->regmap, reg, STARFIVE_CLK_DIV_MASK, value);
}

static u8 starfive_clk_get_parent(struct clk_hw *hw)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	u32 value = starfive_clk_reg_get(clk);

	return (value & STARFIVE_CLK_MUX_MASK) >> STARFIVE_CLK_MUX_SHIFT;
}

static int starfive_clk_set_parent(struct clk_hw *hw, u8 index)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	struct starfive_clk_priv *priv = starfive_priv_from(clk);
	unsigned int reg = sizeof(u32) * clk->idx;
	u32 value = (u32)index << STARFIVE_CLK_MUX_SHIFT;

	return regmap_update_bits(priv->regmap, reg, STARFIVE_CLK_MUX_MASK, value);
}

static int starfive_clk_mux_determine_rate(struct clk_hw *hw,
					   struct clk_rate_request *req)
{
	return clk_mux_determine_rate_flags(hw, req, 0);
}

static int starfive_clk_get_phase(struct clk_hw *hw)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	u32 value = starfive_clk_reg_get(clk);

	return (value & STARFIVE_CLK_INVERT) ? 180 : 0;
}

static int starfive_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct starfive_clk *clk = starfive_clk_from(hw);
	struct starfive_clk_priv *priv = starfive_priv_from(clk);
	unsigned int reg = sizeof(u32) * clk->idx;
	u32 value;

	if (degrees == 0)
		value = 0;
	else if (degrees == 180)
		value = STARFIVE_CLK_INVERT;
	else
		return -EINVAL;

	return regmap_update_bits(priv->regmap, reg, STARFIVE_CLK_INVERT, value);
}

#ifdef CONFIG_DEBUG_FS
static void starfive_clk_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	static const struct debugfs_reg32 starfive_clk_reg = {
		.name = "CTRL",
		.offset = 0,
	};
	struct starfive_clk *clk = starfive_clk_from(hw);
	struct starfive_clk_priv *priv = starfive_priv_from(clk);
	struct debugfs_regset32 *regset;
	void __iomem *base;

	regset = devm_kzalloc(priv->dev, sizeof(*regset), GFP_KERNEL);
	if (!regset)
		return;

	regset->regs = &starfive_clk_reg;
	regset->nregs = 1;

	base = of_iomap(priv->dev->of_node, 0);
	if (!base) {
		base = of_iomap(priv->dev->of_node->parent, 0);
		if (!base)
			return;
	}

	regset->base = base + sizeof(u32) * clk->idx;

	debugfs_create_regset32("registers", 0400, dentry, regset);
}
#else
#define starfive_clk_debug_init NULL
#endif

static const struct clk_ops starfive_clk_gate_ops = {
	.enable = starfive_clk_enable,
	.disable = starfive_clk_disable,
	.is_enabled = starfive_clk_is_enabled,
	.debug_init = starfive_clk_debug_init,
};

static const struct clk_ops starfive_clk_div_ops = {
	.recalc_rate = starfive_clk_recalc_rate,
	.determine_rate = starfive_clk_determine_rate,
	.set_rate = starfive_clk_set_rate,
	.debug_init = starfive_clk_debug_init,
};

static const struct clk_ops starfive_clk_fdiv_ops = {
	.recalc_rate = starfive_clk_frac_recalc_rate,
	.determine_rate = starfive_clk_frac_determine_rate,
	.set_rate = starfive_clk_frac_set_rate,
	.debug_init = starfive_clk_debug_init,
};

static const struct clk_ops starfive_clk_gdiv_ops = {
	.enable = starfive_clk_enable,
	.disable = starfive_clk_disable,
	.is_enabled = starfive_clk_is_enabled,
	.recalc_rate = starfive_clk_recalc_rate,
	.determine_rate = starfive_clk_determine_rate,
	.set_rate = starfive_clk_set_rate,
	.debug_init = starfive_clk_debug_init,
};

static const struct clk_ops starfive_clk_mux_ops = {
	.determine_rate = starfive_clk_mux_determine_rate,
	.set_parent = starfive_clk_set_parent,
	.get_parent = starfive_clk_get_parent,
	.debug_init = starfive_clk_debug_init,
};

static const struct clk_ops starfive_clk_gmux_ops = {
	.enable = starfive_clk_enable,
	.disable = starfive_clk_disable,
	.is_enabled = starfive_clk_is_enabled,
	.determine_rate = starfive_clk_mux_determine_rate,
	.set_parent = starfive_clk_set_parent,
	.get_parent = starfive_clk_get_parent,
	.debug_init = starfive_clk_debug_init,
};

static const struct clk_ops starfive_clk_mdiv_ops = {
	.recalc_rate = starfive_clk_recalc_rate,
	.determine_rate = starfive_clk_determine_rate,
	.get_parent = starfive_clk_get_parent,
	.set_parent = starfive_clk_set_parent,
	.set_rate = starfive_clk_set_rate,
	.debug_init = starfive_clk_debug_init,
};

static const struct clk_ops starfive_clk_gmd_ops = {
	.enable = starfive_clk_enable,
	.disable = starfive_clk_disable,
	.is_enabled = starfive_clk_is_enabled,
	.recalc_rate = starfive_clk_recalc_rate,
	.determine_rate = starfive_clk_determine_rate,
	.get_parent = starfive_clk_get_parent,
	.set_parent = starfive_clk_set_parent,
	.set_rate = starfive_clk_set_rate,
	.debug_init = starfive_clk_debug_init,
};

static const struct clk_ops starfive_clk_inv_ops = {
	.get_phase = starfive_clk_get_phase,
	.set_phase = starfive_clk_set_phase,
	.debug_init = starfive_clk_debug_init,
};

const struct clk_ops *starfive_clk_ops(u32 max)
{
	if (max & STARFIVE_CLK_DIV_MASK) {
		if (max & STARFIVE_CLK_MUX_MASK) {
			if (max & STARFIVE_CLK_ENABLE)
				return &starfive_clk_gmd_ops;
			return &starfive_clk_mdiv_ops;
		}
		if (max & STARFIVE_CLK_ENABLE)
			return &starfive_clk_gdiv_ops;
		if (max == STARFIVE_CLK_FRAC_MAX)
			return &starfive_clk_fdiv_ops;
		return &starfive_clk_div_ops;
	}

	if (max & STARFIVE_CLK_MUX_MASK) {
		if (max & STARFIVE_CLK_ENABLE)
			return &starfive_clk_gmux_ops;
		return &starfive_clk_mux_ops;
	}

	if (max & STARFIVE_CLK_ENABLE)
		return &starfive_clk_gate_ops;

	return &starfive_clk_inv_ops;
}
EXPORT_SYMBOL_GPL(starfive_clk_ops);

// SPDX-License-Identifier: GPL-2.0
/*
 * StarFive JH7110 Always-On Clock Driver
 *
 * Copyright (C) 2022 Emil Renner Berthing <kernel@esmil.dk>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/starfive-jh7110-aon.h>

#include "clk-starfive.h"

/* external clocks */
#define JH7110_AONCLK_OSC		(JH7110_AONCLK_END + 0)
#define JH7110_AONCLK_RTC		(JH7110_AONCLK_END + 1)
#define JH7110_AONCLK_GMAC0_RMII_REFIN	(JH7110_AONCLK_END + 2)
#define JH7110_AONCLK_GMAC0_RGMII_RXIN	(JH7110_AONCLK_END + 3)
#define JH7110_AONCLK_STG_AXIAHB	(JH7110_AONCLK_END + 4)
#define JH7110_AONCLK_APB_BUS_FUNC	(JH7110_AONCLK_END + 5)
#define JH7110_AONCLK_GMAC0_GTXCLK	(JH7110_AONCLK_END + 6)

static const struct starfive_clk_data jh7110_aonclk_data[] = {
	/* source */
	STARFIVE__DIV(JH7110_AONCLK_OSC_DIV4, "osc_div4", 4,
		      JH7110_AONCLK_OSC),
	STARFIVE__MUX(JH7110_AONCLK_APB_FUNC, "aon_apb_func", 2,
		      JH7110_AONCLK_OSC_DIV4,
		      JH7110_AONCLK_OSC),
	/* gmac0 */
	STARFIVE_GATE(JH7110_AONCLK_GMAC0_AHB, "gmac0_ahb", 0,
		      JH7110_AONCLK_STG_AXIAHB),
	STARFIVE_GATE(JH7110_AONCLK_GMAC0_AXI, "gmac0_axi", 0,
		      JH7110_AONCLK_STG_AXIAHB),
	STARFIVE__DIV(JH7110_AONCLK_GMAC0_RMII_RTX, "gmac0_rmii_rtx", 30,
		      JH7110_AONCLK_GMAC0_RMII_REFIN),
	STARFIVE_GMUX(JH7110_AONCLK_GMAC0_TX, "gmac0_tx", 0, 2,
		      JH7110_AONCLK_GMAC0_GTXCLK,
		      JH7110_AONCLK_GMAC0_RMII_RTX),
	STARFIVE__INV(JH7110_AONCLK_GMAC0_TX_INV, "gmac0_tx_inv",
		      JH7110_AONCLK_GMAC0_TX),
	STARFIVE__MUX(JH7110_AONCLK_GMAC0_RX, "gmac0_rx", 2,
		      JH7110_AONCLK_GMAC0_RGMII_RXIN,
		      JH7110_AONCLK_GMAC0_RMII_RTX),
	STARFIVE__INV(JH7110_AONCLK_GMAC0_RX_INV, "gmac0_rx_inv",
		      JH7110_AONCLK_GMAC0_RX),
	/* otpc */
	STARFIVE_GATE(JH7110_AONCLK_OTPC_APB, "otpc_apb", CLK_IGNORE_UNUSED,
		      JH7110_AONCLK_APB_BUS_FUNC),
	/* rtc */
	STARFIVE_GATE(JH7110_AONCLK_RTC_APB, "rtc_apb", CLK_IGNORE_UNUSED,
		      JH7110_AONCLK_APB_BUS_FUNC),
	STARFIVE__DIV(JH7110_AONCLK_RTC_INTERNAL, "rtc_internal", 1022,
		      JH7110_AONCLK_OSC),
	STARFIVE__MUX(JH7110_AONCLK_RTC_32K, "rtc_32k", 2,
		      JH7110_AONCLK_RTC,
		      JH7110_AONCLK_RTC_INTERNAL),
	STARFIVE_GATE(JH7110_AONCLK_RTC_CAL, "rtc_cal", 0,
		      JH7110_AONCLK_OSC),
};

static struct clk_hw *jh7110_aonclk_get(struct of_phandle_args *clkspec, void *data)
{
	struct starfive_clk_priv *priv = data;
	unsigned int idx = clkspec->args[0];

	if (idx < JH7110_AONCLK_END)
		return &priv->reg[idx].hw;

	return ERR_PTR(-EINVAL);
}

static int jh7110_aoncrg_probe(struct platform_device *pdev)
{
	struct starfive_clk_priv *priv;
	unsigned int idx;
	int ret;

	priv = devm_kzalloc(&pdev->dev,
			    struct_size(priv, reg, JH7110_AONCLK_END),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	priv->regmap = syscon_node_to_regmap(priv->dev->of_node->parent);
	if (IS_ERR(priv->regmap)) {
		dev_err(priv->dev, "failed to get regmap (error %ld)\n",
			PTR_ERR(priv->regmap));
		return PTR_ERR(priv->regmap);
	}

	for (idx = 0; idx < JH7110_AONCLK_END; idx++) {
		u32 max = jh7110_aonclk_data[idx].max;
		struct clk_parent_data parents[4] = {};
		struct clk_init_data init = {
			.name = jh7110_aonclk_data[idx].name,
			.ops = starfive_clk_ops(max),
			.parent_data = parents,
			.num_parents = ((max & STARFIVE_CLK_MUX_MASK)
					>> STARFIVE_CLK_MUX_SHIFT) + 1,
			.flags = jh7110_aonclk_data[idx].flags,
		};
		struct starfive_clk *clk = &priv->reg[idx];
		unsigned int i;

		for (i = 0; i < init.num_parents; i++) {
			unsigned int pidx = jh7110_aonclk_data[idx].parents[i];

			if (pidx < JH7110_AONCLK_END)
				parents[i].hw = &priv->reg[pidx].hw;
			else if (pidx == JH7110_AONCLK_OSC)
				parents[i].fw_name = "osc";
			else if (pidx == JH7110_AONCLK_RTC)
				parents[i].fw_name = "clk_rtc";
			else if (pidx == JH7110_AONCLK_GMAC0_RMII_REFIN)
				parents[i].fw_name = "gmac0_rmii_refin";
			else if (pidx == JH7110_AONCLK_GMAC0_RGMII_RXIN)
				parents[i].fw_name = "gmac0_rgmii_rxin";
			else if (pidx == JH7110_AONCLK_STG_AXIAHB)
				parents[i].fw_name = "stg_axiahb";
			else if (pidx == JH7110_AONCLK_APB_BUS_FUNC)
				parents[i].fw_name = "apb_bus_func";
		}

		clk->hw.init = &init;
		clk->idx = idx;
		clk->max_div = max & STARFIVE_CLK_DIV_MASK;

		ret = devm_clk_hw_register(&pdev->dev, &clk->hw);
		if (ret)
			return ret;
	}

	return devm_of_clk_add_hw_provider(&pdev->dev, jh7110_aonclk_get, priv);
}

static const struct of_device_id jh7110_aoncrg_match[] = {
	{ .compatible = "starfive,jh7110-clkgen-aon" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, jh7110_aoncrg_match);

static struct platform_driver jh7110_aoncrg_driver = {
	.probe = jh7110_aoncrg_probe,
	.driver = {
		.name = "clk-starfive-jh7110-aon",
		.of_match_table = jh7110_aoncrg_match,
	},
};
module_platform_driver(jh7110_aoncrg_driver);

MODULE_AUTHOR("Emil Renner Berthing");
MODULE_DESCRIPTION("StarFive JH7110 always-on clock driver");
MODULE_LICENSE("GPL v2");

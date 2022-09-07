// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Reset driver for the StarFive JH7100 SoC
 *
 * Copyright (C) 2021 Emil Renner Berthing <kernel@esmil.dk>
 * Copyright (C) 2021-2022 StarFive Technology Co., Ltd.
 */

#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include <dt-bindings/reset/starfive-jh7100.h>

/* register offsets */
#define JH7100_RESET_ASSERT0	0x00
#define JH7100_RESET_ASSERT1	0x04
#define JH7100_RESET_ASSERT2	0x08
#define JH7100_RESET_ASSERT3	0x0c
#define JH7100_RESET_STATUS0	0x10
#define JH7100_RESET_STATUS1	0x14
#define JH7100_RESET_STATUS2	0x18
#define JH7100_RESET_STATUS3	0x1c

/*
 * Writing a 1 to the n'th bit of the m'th ASSERT register asserts
 * line 32m + n, and writing a 0 deasserts the same line.
 * Most reset lines have their status inverted so a 0 bit in the STATUS
 * register means the line is asserted and a 1 means it's deasserted. A few
 * lines don't though, so store the expected value of the status registers when
 * all lines are asserted.
 */
static const u32 jh7100_reset_asserted[4] = {
	/* STATUS0 */
	BIT(JH7100_RST_U74 % 32) |
	BIT(JH7100_RST_VP6_DRESET % 32) |
	BIT(JH7100_RST_VP6_BRESET % 32),
	/* STATUS1 */
	BIT(JH7100_RST_HIFI4_DRESET % 32) |
	BIT(JH7100_RST_HIFI4_BRESET % 32),
	/* STATUS2 */
	BIT(JH7100_RST_E24 % 32),
	/* STATUS3 */
	0,
};

struct jh7100_reset {
	struct reset_controller_dev rcdev;
	struct regmap *regmap;
};

static inline struct jh7100_reset *
jh7100_reset_from(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct jh7100_reset, rcdev);
}

static int jh7100_reset_update(struct reset_controller_dev *rcdev,
			       unsigned long id, bool assert)
{
	struct jh7100_reset *data = jh7100_reset_from(rcdev);
	u32 offset = id / 32;
	u32 mask = BIT(id % 32);
	u32 reg_assert = JH7100_RESET_ASSERT0 + offset * sizeof(u32);
	u32 reg_status = JH7100_RESET_STATUS0 + offset * sizeof(u32);
	u32 done = jh7100_reset_asserted[offset] & mask;
	u32 value;
	int ret;

	if (!assert)
		done ^= mask;

	if (assert)
		ret = regmap_update_bits(data->regmap, reg_assert, mask, mask);
	else
		ret = regmap_update_bits(data->regmap, reg_assert, mask, 0);

	if (ret)
		return ret;

	/* if the associated clock is gated, deasserting might otherwise hang forever */
	ret = regmap_read_poll_timeout_atomic(data->regmap,
					      reg_status,
					      value, (value & mask) == done,
					      0, 1000);
	if (ret)
		dev_warn(rcdev->dev, "id:%ld bank:%d, mask:%#x assert:%#x status:%#x ret:%d\n",
			 id, offset, mask, reg_assert, reg_status, ret);

	return ret;
}

static int jh7100_reset_assert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return jh7100_reset_update(rcdev, id, true);
}

static int jh7100_reset_deassert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return jh7100_reset_update(rcdev, id, false);
}

static int jh7100_reset_reset(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	int ret;

	ret = jh7100_reset_assert(rcdev, id);
	if (ret)
		return ret;

	return jh7100_reset_deassert(rcdev, id);
}

static int jh7100_reset_status(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct jh7100_reset *data = jh7100_reset_from(rcdev);
	u32 offset = id / 32;
	u32 mask = BIT(id % 32);
	u32 reg_status = JH7100_RESET_STATUS0 + offset * sizeof(u32);
	u32 value;
	int ret;

	ret = regmap_read(data->regmap, reg_status, &value);
	if (ret)
		return ret;

	return !((value ^ jh7100_reset_asserted[offset]) & mask);
}

static const struct reset_control_ops jh7100_reset_ops = {
	.assert		= jh7100_reset_assert,
	.deassert	= jh7100_reset_deassert,
	.reset		= jh7100_reset_reset,
	.status		= jh7100_reset_status,
};

static int __init jh7100_reset_probe(struct platform_device *pdev)
{
	struct jh7100_reset *data;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = device_node_to_regmap(pdev->dev.of_node);
	if (IS_ERR(data->regmap)) {
		dev_err(&pdev->dev, "failed to get regmap (error %ld)\n",
			PTR_ERR(data->regmap));
		return PTR_ERR(data->regmap);
	}

	data->rcdev.ops = &jh7100_reset_ops;
	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = JH7100_RSTN_END;
	data->rcdev.dev = &pdev->dev;
	data->rcdev.of_node = pdev->dev.of_node;

	return devm_reset_controller_register(&pdev->dev, &data->rcdev);
}

static const struct of_device_id jh7100_reset_dt_ids[] = {
	{ .compatible = "starfive,jh7100-reset" },
	{ /* sentinel */ }
};

static struct platform_driver jh7100_reset_driver = {
	.driver = {
		.name = "jh7100-reset",
		.of_match_table = jh7100_reset_dt_ids,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver_probe(jh7100_reset_driver, jh7100_reset_probe);

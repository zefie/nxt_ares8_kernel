/*
 * platform_it7260.c: ITE 7260 touch platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/it7260_ts.h>
#include <asm/intel-mid.h>
#include "platform_it7260.h"

void *it7260_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = info;
	static struct it7260_platform_data it7260_platform_data = {
		.irq_type = IRQF_ONESHOT | IRQF_TRIGGER_LOW,
		.gpio = -1,
	};

	if (!i2c_info->irq) { /* not a fast-int */
		it7260_platform_data.gpio = get_gpio_by_name("touch_int");
		if (it7260_platform_data.gpio == -1)
			it7260_platform_data.gpio = 183;
		it7260_platform_data.irq_type |= IRQ_TYPE_EDGE_FALLING;
	}
	return &it7260_platform_data;
}

/*
 * platform_s1222.c: Synaptics touch platform data initilization file
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
#include <asm/intel-mid.h>
#include "platform_s1222.h"
#include <linux/input/synaptics_dsx.h>

#define DSX_I2C_ADDR 0x20
#define DSX_ATTN_GPIO 183
#define DSX_IRQ_ON_STATE 0
#define DSX_IRQ_FLAGS (IRQF_TRIGGER_LOW | IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT)
#define DSX_POWER_GPIO -1
#define DSX_POWER_ON_STATE 1
#define DSX_POWER_DELAY_MS 160
#define DSX_RESET_GPIO -1
#define DSX_RESET_ON_STATE 0
#define DSX_RESET_DELAY_MS 100
#define DSX_RESET_ACTIVE_MS 20
#define DSX_MAX_Y_FOR_2D -1 /* set to -1 if no virtual buttons */
static unsigned char bus_reg_name[] = "";
static unsigned int cap_button_codes[] = {};
static unsigned int vir_button_codes[] = {};

static struct synaptics_dsx_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(cap_button_codes),
	.map = cap_button_codes,
};

static struct synaptics_dsx_button_map vir_button_map = {
	.nbuttons = ARRAY_SIZE(vir_button_codes),
	.map = vir_button_codes,
};

static struct synaptics_dsx_board_data dsx_board_data = {
	.x_flip = 1,
	.swap_axes = 1,
	.irq_gpio = DSX_ATTN_GPIO,
	.irq_on_state = DSX_IRQ_ON_STATE,
	.irq_flags = DSX_IRQ_FLAGS,
	.power_gpio = DSX_POWER_GPIO,
	.power_on_state = DSX_POWER_ON_STATE,
	.power_delay_ms = DSX_POWER_DELAY_MS,
	.reset_gpio = DSX_RESET_GPIO,
	.reset_on_state = DSX_RESET_ON_STATE,
	.reset_delay_ms = DSX_RESET_DELAY_MS,
	.reset_active_ms = DSX_RESET_ACTIVE_MS,
	.max_y_for_2d = DSX_MAX_Y_FOR_2D,
	.bus_reg_name = bus_reg_name,
	.cap_button_map = &cap_button_map,
	.vir_button_map = &vir_button_map,
};

void *s1222_platform_data(void *info)
{
	dsx_board_data.irq_gpio = get_gpio_by_name("touch_int");
	if (dsx_board_data.irq_gpio < 0) {
		pr_err("%s: failed to get from sfi table, use default GP183\n",
			 __func__);
		dsx_board_data.irq_gpio = DSX_ATTN_GPIO;
	}

	return &dsx_board_data;
}

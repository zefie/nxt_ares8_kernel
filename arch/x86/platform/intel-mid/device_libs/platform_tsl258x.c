/*
 * platform_tsl258x.c: TAOS TSL258x light sensor platform data initilization file
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
#include <linux/tsl258x_als.h>
#include <asm/intel-mid.h>
#include <linux/tsl258x_als.h>
#include <asm/intel_scu_flis.h>
#include "platform_tsl258x.h"


static int gpio_conf(void) {
	int ret1, ret2;

	ret1 = config_pin_flis(tng_gp_i2c_6_sda, PULL, NONE);
	ret2 = config_pin_flis(tng_gp_i2c_6_scl, PULL, NONE);

	if (ret1 < 0)
		return ret1;
	if (ret2 < 0)
		return ret2;
	return 0;
}

void *tsl258x_als_platform_data(void *info)
{
	static struct tsl258x_platform_data tsl258x_platform_data;

	tsl258x_platform_data.als_def_odr = TSL258X_ALS_DEF_ODR;
	tsl258x_platform_data.als_def_als_time = TSL258X_ALS_DEF_TIME;
	tsl258x_platform_data.als_def_gain = TSL258X_ALS_DEF_GAIN;
	tsl258x_platform_data.als_def_gain_trim = TSL258X_ALS_DEF_GAIN_TRIM;
	tsl258x_platform_data.als_def_cal_target = TSL258X_ALS_DEF_CAL_TARGET;

	if (INTEL_MID_BOARD(3, PHONE, MRFL, GLC, ENG, 4) ||
			INTEL_MID_BOARD(3, PHONE, MRFL, GLC, PRO, 4)||
			INTEL_MID_BOARD(3, PHONE, MRFL, MVN, ENG, 4)||
			INTEL_MID_BOARD(3, PHONE, MRFL, MVN, PRO, 4)) {
		tsl258x_platform_data.gpio_conf = gpio_conf;
	} else
		tsl258x_platform_data.gpio_conf = NULL;

	return &tsl258x_platform_data;
}

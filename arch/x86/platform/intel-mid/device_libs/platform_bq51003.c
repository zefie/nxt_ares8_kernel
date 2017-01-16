/*
 * platform_bq51003.c: bq51003 platform data initilization file
 *
 * Copyright(c) 2015 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/power/bq51003_power.h>
#include <asm/intel-mid.h>

static struct bq51003_plat_data bq51003_pdata = {
	.wc_chg_n = -1,
	.gpio_en1 = -1,
};

void *bq51003_platform_data(void *info)
{
	/* acquired gpio used to notify if power transmission is ongoing */
	bq51003_pdata.wc_chg_n = get_gpio_by_name("wc_chg_n");

	/* acquired gpio to enable/disable wireless charger */
	bq51003_pdata.gpio_en1 = get_gpio_by_name("wc_en1");

	return &bq51003_pdata;
}

/*
 * platform_lis3dh.c: lis3dh platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/input/lis3dsh.h>
#include "platform_lis3dsh.h"

void *lis3dsh_platform_data(void *info)
{
	static struct lis3dsh_acc_platform_data lis3dsh_pdata;

	lis3dsh_pdata.fs_range = LIS3DSH_ACC_G_2G;
	lis3dsh_pdata.poll_interval = LIS3DSH_ACC_POLL_PERIOD_MS_ODR100;
	lis3dsh_pdata.negate_x = 0;
	lis3dsh_pdata.negate_y = 0;
	lis3dsh_pdata.negate_z = 0;
	lis3dsh_pdata.axis_map_x = 0;
	lis3dsh_pdata.axis_map_y = 1;
	lis3dsh_pdata.axis_map_z = 2;
	lis3dsh_pdata.min_interval = LIS3DSH_ACC_MIN_POLL_PERIOD_MS;
	lis3dsh_pdata.gpio_int1 = get_gpio_by_name("secacc_int1_n");	/* SECACC_INT1_N <-> GP54 */
#if 0 /* int2 is not being used for the moment, but may be needed later */
	lis3dsh_pdata.gpio_int2 = get_gpio_by_name("secacc_int2_n");	/* SECACC_INT1_N <-> GP55 */
#endif
	return &lis3dsh_pdata;

}

/*
 * platform_isl97698.c: isl97698 platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/isl97698.h>

void *isl97698_brightness_platform_data(void *info)
{
	static struct isl97698_platform_data isl97698_pdata;

	isl97698_pdata.bias_en = get_gpio_by_name("disp0_bias_en");

	/*
	 * Base value of isl_brightness_val_max is 0x7F. This value has to be
         * tuned for each platform in order to limit upper bound of current
         * going into LED to 11 mA. This allows the measured display luminance
         * to be less than 500 cd/m2 at max brightness level.
	 */
	if (INTEL_MID_BOARD(2, PHONE, MRFL, MVN, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, MVN, ENG))
		isl97698_pdata.isl_brightness_val_max = 0x6F;
	else if (INTEL_MID_BOARD(2, PHONE, MRFL, GLC, PRO) ||
		INTEL_MID_BOARD(2, PHONE, MRFL, GLC, ENG))
		isl97698_pdata.isl_brightness_val_max = 0x7F;

	return &isl97698_pdata;
}

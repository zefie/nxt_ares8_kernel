/*
 * TSL258x ALS Controller Driver
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

#ifndef _TSL258X_ALS_H_
#define _TSL258X_ALS_H_

enum tsl2584_gain_idex {
	TSL2584_ALS_GAIN1 = 0x0,
	TSL2584_ALS_GAIN8,
	TSL2584_ALS_GAIN16,
	TSL2584_ALS_GAIN111,
	TSL2584_ALS_GAIN_NUM,
};

#define TSL258X_ALS_DEF_ODR		1000
#define TSL258X_ALS_DEF_TIME		50
#define TSL258X_ALS_DEF_GAIN		16
#define TSL258X_ALS_DEF_GAIN_TRIM	1000
#define TSL258X_ALS_DEF_CAL_TARGET	130

struct tsl258x_platform_data {
	int als_def_odr;
	int als_def_als_time;
	int als_def_gain;
	int als_def_gain_trim;
	int als_def_cal_target;
	int (*gpio_conf)(void);
};

#endif /* _TSL258X_ALS_H_ */

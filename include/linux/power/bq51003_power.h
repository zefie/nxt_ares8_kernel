/*
 * bq51003_power.h: platform data structure for bq51003 wireless
 * power supply driver
 *
 * Copyright (c) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __BQ51003_POWER_H__
#define __BQ51003_POWER_H__

#include <linux/types.h>

struct bq51003_plat_data {
	int wc_chg_n;
	int gpio_en1;
	int (*wc_docking_detection)(bool enable, void (*cb)(void*), void*);
};

#endif /* __BQ51003_POWER_H__ */

/*
 * ISL97698 Controller Driver
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

#ifndef _ISL97698_BRIGHTNESS_H_
#define _ISL97698_BRIGHTNESS_H_

struct isl97698_platform_data {
	int bias_en;
	int isl_brightness_val_max;
};

#endif /* _ISL97698_BRIGHTNESS_H_ */

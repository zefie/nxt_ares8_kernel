/*
 * IT7260 Touchscreen Controller Driver
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

#ifndef _IT7260_TS_H_
#define _IT7260_TS_H_

struct it7260_platform_data {
	int irq_type;
	int gpio;
};

#endif /* _IT7260_TS_H_ */

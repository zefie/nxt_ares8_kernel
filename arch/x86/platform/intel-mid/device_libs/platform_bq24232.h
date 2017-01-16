/*
 * platform_bq24232.h: platform data for bq24232 driver
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_BQ24232_H_
#define _PLATFORM_BQ24232_H_

extern void __init *bq24232_charger_platform_data(void *info) __attribute__((weak));
#endif

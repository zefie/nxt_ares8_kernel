/*
 * platform_bq24232.c: platform data for bq24232 driver
 *
 * Copyright (c) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/bq24232_charger.h>
#include <asm/intel-mid.h>
#include <asm/pmic_pdata.h>
#include "platform_bq24232.h"

/*
 * Battery temperature limits in 0.1 °C, voltage limits in uV
 */
#define CHGING_PROF_VOLT_MIN		2800000
#define CHGING_PROF_VOLT_MAX		4500000
#define CHGING_PROF_TEMP_MIN		-200
#define CHGING_PROF_TEMP_MAX		700

#define CHGING_PROF_TEMP_WARM		455
#define CHGING_PROF_TEMP_COOL		110
#define CHGING_PROF_VOLT_WARM		4100000

/*
 * Hysteresis values added to/subtract from battery temperature
 * and voltage limits, to avoid charging oscillations between
 * adjacent charging interval
 */
#define CHGING_PROF_TEMP_HYSIS		10
#define CHGING_PROF_VOLT_HYSIS		100000

static char *bq24232_supplied_to[] = {
				"max17047_battery",
};

static struct bq24232_plat_data bq24232_pdata;

/*
 * Battery temperature limits in 0.1 °C, voltage limits in uV.
 * Charging temperature interval set to be outside the range
 * allowed by charger hw component [0°C, 50°C].
 * Relying on hw to stop charging in case of over/underheat
 */
struct chging_profile chging_profile_tbl[] = {
	{
		CHGING_CURRENT_LOW,
		{ CHGING_PROF_VOLT_MIN, CHGING_PROF_VOLT_MAX },
		{ 0 , 0 },
		{ CHGING_PROF_TEMP_MIN, CHGING_PROF_TEMP_COOL },
		{ 0 , CHGING_PROF_TEMP_HYSIS },
	},
	{
		CHGING_CURRENT_NULL,
		{ CHGING_PROF_VOLT_WARM, CHGING_PROF_VOLT_MAX },
		{ 0 , 0 },
		{ CHGING_PROF_TEMP_WARM, CHGING_PROF_TEMP_MAX },
		{ 0 , 0 },
	},
	{
		CHGING_CURRENT_HIGH,
		{ CHGING_PROF_VOLT_MIN, CHGING_PROF_VOLT_WARM },
		{ 0, CHGING_PROF_VOLT_HYSIS },
		{ CHGING_PROF_TEMP_COOL, CHGING_PROF_TEMP_MAX },
		{ 0, 0 },
	},
	{
		CHGING_CURRENT_HIGH,
		{ CHGING_PROF_VOLT_MIN, CHGING_PROF_VOLT_MAX },
		{ 0, 0 },
		{ CHGING_PROF_TEMP_COOL, CHGING_PROF_TEMP_WARM },
		{ 0, CHGING_PROF_TEMP_HYSIS },
	}
};

/*
 * Battery temperature offset between sw measured value and
 * battery NTC. Expressed in 0.1 °C
 */
#define BQ24232_BATTERY_TEMP_OFFSET		30

void *bq24232_charger_platform_data(void *info)
{
	bq24232_pdata.name = BQ24232_CHRGR_DEV_NAME;

	bq24232_pdata.chg_rate_temp_gpio = get_gpio_by_name("chg_rate_temp");
	bq24232_pdata.pgood_gpio = get_gpio_by_name("chg_pgood");
	bq24232_pdata.charger_ce_n_gpio = get_gpio_by_name("charger_ce_n");
	bq24232_pdata.enable_charging = bq24232_assert_ce_n;
#if CONFIG_PMIC_CCSM
	if (bq24232_pdata.charger_ce_n_gpio < 0)
		bq24232_pdata.enable_charging = pmic_enable_charging;
	bq24232_pdata.enable_vbus = pmic_enable_vbus;
#endif
	if (chging_profile_tbl) {
		bq24232_pdata.chging_profiles = chging_profile_tbl;
		bq24232_pdata.num_chging_profiles = ARRAY_SIZE(chging_profile_tbl);
	}
	bq24232_pdata.supplied_to = bq24232_supplied_to;
	bq24232_pdata.num_supplicants = ARRAY_SIZE(bq24232_supplied_to);
	if  (INTEL_MID_BOARD(2, PHONE, MRFL, RBY, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, RBY, ENG) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, MVN, PRO) ||
			INTEL_MID_BOARD(2, PHONE, MRFL, MVN, ENG))
		bq24232_pdata.wc_direct_support = true;
	else
		bq24232_pdata.wc_direct_support = false;

	bq24232_pdata.bat_temp_offset = BQ24232_BATTERY_TEMP_OFFSET;

	return &bq24232_pdata;
}

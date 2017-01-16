/*
 *  bq24232_charger.h - BQ24232 Charger
 *
 *  Copyright (C) 2015 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_POWER_BQ24232_CHARGER_H__
#define __LINUX_POWER_BQ24232_CHARGER_H__

#include <linux/power_supply.h>
#include <linux/types.h>

#define BQ24232_CHRGR_DEV_NAME	"bq24232_charger"

#define BQ24232_CHARGE_CURRENT_LOW	100
#define BQ24232_CHARGE_CURRENT_HIGH	400

#define BAT_TEMP_MONITOR_DELAY	(60 * HZ)

#ifdef CONFIG_BQ24232_CHARGER
extern int bq24232_get_charger_status(void);
extern void bq24232_set_charging_status(bool chg_stat);
#else
int bq24232_get_charger_status(void)
{
	return 0;
}

void bq24232_set_charging_status(bool chg_stat)
{
	return;
}
#endif
int bq24232_assert_ce_n(bool val);

enum chging_current_mode {
	CHGING_CURRENT_NULL,
	CHGING_CURRENT_LOW,
	CHGING_CURRENT_HIGH
};

struct chging_profile {
	enum chging_current_mode mode;
	int volt_range[2];
	int volt_hysis[2];
	int temp_range[2];
	int temp_hysis[2];
};

enum bat_range {
	RANGE_LOW,
	RANGE_HIGH
};

/**
 * struct gpio_charger_platform_data - platform_data for gpio_charger devices
 * @name:		Name for the chargers power_supply device
 * @supplied_to:	Devices supplied by this charger
 * @num_supplicants:	Number of devices supplied by this charger
 * @chging_profiles:	Charge current in function of battery temperature/voltage
 * @num_chging_profiles:	Number of charging profiles defined
 * @pgood_gpio:	GPIO which is used to indicate the chargers status
 * @chg_rate_temp_gpio:	GPIO which is used to select the charge rate
 * @charger_ce_n_gpio:	GPIO to assert low to charge enable, high to disable it
 * @enable_charging:	Function callback to activate the CE_N signal on the charger
 */
struct bq24232_plat_data {
	const char *name;

	char **supplied_to;
	size_t num_supplicants;
	struct chging_profile *chging_profiles;
	int num_chging_profiles;
	int pgood_gpio;
	int chg_rate_temp_gpio;
	int charger_ce_n_gpio;

	int (*enable_charging) (bool val);
	int (*enable_vbus) (bool val);
	int (*get_charging_status) (bool *charging_status);

	int bat_temp_offset;
	bool wc_direct_support;
};

#endif

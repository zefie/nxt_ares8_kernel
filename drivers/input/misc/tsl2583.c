/*
 * Device driver for monitoring ambient light intensity (lux)
 * within the TAOS tsl258x family of devices (tsl2580, tsl2581).
 *
 * Copyright (c) 2011, TAOS Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/posix-timers.h>
#include <linux/pm_runtime.h>

#include <linux/tsl258x_als.h>
#include <asm/intel-mid.h>

#define TSL258X_NAME	"tsl2584"

#define ID_UNKNOWN		0
#define ID_TSL2580		1 /* the TSL2580 is SMBus version of TSL2581 */
#define ID_TSL2581		2
#define ID_TSL2583		3
#define ID_TSL2584TSV		4

#define TSL258X_CHANNEL0		0x00
#define TSL258X_CHANNEL1		0x01

#define TSL258X_MAX_DEVICE_REGS		32

/* Triton register offsets */
#define	TSL258X_REG_MAX		8

/* Device Registers and Masks */
#define TSL258X_CNTRL			0x00
#define TSL258X_ALS_TIME		0X01
#define TSL258X_INTERRUPT		0x02
#define TSL258X_GAIN			0x07
#define TSL258X_REVID			0x11
#define TSL258X_CHIPID			0x12
#define TSL258X_ALS_CHAN0LO		0x14
#define TSL258X_ALS_CHAN0HI		0x15
#define TSL258X_ALS_CHAN1LO		0x16
#define TSL258X_ALS_CHAN1HI		0x17
#define TSL258X_TMR_LO			0x18
#define TSL258X_TMR_HI			0x19
#define TSL258X_ID2			0x1e

/* tsl258x cmd reg masks */
#define TSL258X_CMD_REG			0x80
#define TSL258X_CMD_SPL_FN		0x60
#define TSL258X_CMD_AUTO_INC	0x20
#define TSL258X_CMD_ALS_INT_CLR	0X01

/* tsl258x cntrl reg masks */
#define TSL258X_CNTL_ADC_ENBL	0x02
#define TSL258X_CNTL_PWR_ON		0x01
#define TSL258X_CNTL_PWR_OFF	0x00

/* tsl258x status reg masks */
#define TSL258X_STA_ADC_VALID	0x01
#define TSL258X_STA_ADC_INTR	0x10

/* Lux calculation constants */
#define	TSL258X_LUX_CALC_OVER_FLOW		65535

/* constants for lux calculations */
#define MPOW                    13
#define MFACT                   (1 << MPOW)

#define ALS_TIME_MIN	50
#define ALS_TIME_MAX	650

#define ALS_TIME_TO_COUNT(ms)		((((ms) * 100 + 135) / 270) ? (((ms) * 100 + 135) / 270) : 1)
#define ALS_COUNT_TO_TIME(count)	(((count) * 27 + 5) / 10)

/*******************************************************************
start TSL2584TSV lux equation defines
The lux equation is of the form:
coff_1 * ((coff_ch0 * ch0) - (coff_ch1 * ch1)) / (atime * again)

which can be rearranged as:

((coff_1 * coff_ch0 * ch0) - (coff_1 * coff_ch1 * ch1)) / (atime * again)

In order to maintain precision using integers, we multiply numerator and denominator by a power of 2

(((coff_1 * coff_ch0 * (2 ^ MPOW)) * ch0) - ((coff_1 * coff_ch1 * (2 ^ MPOW)) * ch1)) / (atime * again * (2 ^ MPOW))

which is the same as:

(((coff_1 * coff_ch0 * (2 ^ MPOW)) * ch0) - ((coff_1 * coff_ch1 * (2 ^ MPOW)) * ch1)) / ((atime * again) << MPOW))

since atime * again is an integer
*******************************************************************/

/* No glass Coefficients */
#define COFF_1_NO_GLASS         105
#define COFF_CH0_NO_GLASS       (u32)(1    * MFACT)
#define COFF_CH1_NO_GLASS       (u32)(1.13 * MFACT)
#define CH0_COFF_NO_GLASS       (u32)((COFF_1_NO_GLASS * COFF_CH0_NO_GLASS))
#define CH1_COFF_NO_GLASS       (u32)((COFF_1_NO_GLASS * COFF_CH1_NO_GLASS))

/* Add new glass coefficients here */

/* set the coefficients the TSL2584TSV equation will use */
#define TSL2584TSV_CH0_COFF	CH0_COFF_NO_GLASS
#define TSL2584TSV_CH1_COFF	CH1_COFF_NO_GLASS

/* end TSL2584TSV lux equation defines */

/******************************************************************
 start TSL2584TSV lux equation defines on Marvin
 The lux equation is of the form:
 Lux1 = 1000*((Ch0-(2.16*Ch1))/(Atime*Again))
 Lux2 = 1000*((0.95*Ch0)-(1.11*Ch1)/(Atime*Again))
 Lux = MAX(Lux1,Lux2)

 in other form:
 lux1 = (1000*ch0 - 2160*ch1) / (Atime*Again)
 lux2 = (950*ch0 - 1110*ch1) / (Atime*Again)
 Lux = MAX(Lux1,Lux2)
******************************************************************/

/* set the coefficients the TSL2584TSV equation will use on Marvin */
#define TSL2584TSV_CH0_COFF0	1000
#define TSL2584TSV_CH1_COFF0	2160
#define TSL2584TSV_CH0_COFF1	950
#define TSL2584TSV_CH1_COFF1	1110

/* end TSL2584TSV lux equation defines on Marvin */

enum {
	TSL258X_CHIP_UNKNOWN = 0,
	TSL258X_CHIP_WORKING = 1,
	TSL258X_CHIP_SUSPENDED = 2
};

struct taos_settings {
	int als_odr;
	int als_time;
	int als_gain_idex;
	int als_gain_trim;
	int als_cal_target;
};

struct tsl258x_chip {
	struct mutex als_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct data_work;
	struct tsl258x_platform_data *pdata;
	struct taos_settings taos_settings;
	u16 lux;
	int als_time_scale;
	int als_saturation;
	int als_status;
	int id;
};

struct taos_lux {
	unsigned int ratio;
	unsigned int ch0;
	unsigned int ch1;
};

/* This structure is intentionally large to accommodate updates via sysfs. */
/* Sized to 11 = max 10 segments + 1 termination segment */
/* Assumption is is one and only one type of glass used  */
static struct taos_lux taos_device_lux[11];

/* tsl2581, tsl2583 */
static struct taos_lux taos_device_lux_tsl258x[] = {
	{  9830,  8520, 15729 },
	{ 12452, 10807, 23344 },
	{ 14746,  6383, 11705 },
	{ 17695,  4063,  6554 }
};

/* tsl2585tsv */
static struct taos_lux taos_device_lux_tsl2584tsv[] = {
	{ 0, CH0_COFF_NO_GLASS, CH1_COFF_NO_GLASS }
};

struct gainadj {
	s16 ch0;
	s16 ch1;
};

/* Index = (0 - 3) Used to validate the gain selection index */
static const struct gainadj gainadj[] = {
	{ 1, 1 },
	{ 8, 8 },
	{ 16, 16 },
	{ 107, 115 }
};

static struct tsl2584_als_gain_tbl {
	int gain_idex;
	int gain_val;
} tsl2584_als_gain_tbl[] = {
	[0] = {
		.gain_idex = TSL2584_ALS_GAIN1,
		.gain_val = 1,
	},
	[1] = {
		.gain_idex = TSL2584_ALS_GAIN8,
		.gain_val = 8,
	},
	[2] = {
		.gain_idex = TSL2584_ALS_GAIN16,
		.gain_val = 16,
	},
	[3] = {
		.gain_idex = TSL2584_ALS_GAIN111,
		.gain_val = 111,
	},
};

static struct workqueue_struct *tsl258x_wq;

static char *tsl2583x_get_name(struct tsl258x_chip *chip);

/*
 * Provides initial operational parameter defaults.
 * These defaults may be changed through the device's sysfs files.
 */
static void taos_defaults(struct tsl258x_chip *chip)
{
	chip->taos_settings.als_odr = chip->pdata->als_def_odr;
	/* assume clear glass as default */
	chip->taos_settings.als_gain_trim = chip->pdata->als_def_gain_trim;
	/* default gain trim to account for aperture effects */
	chip->taos_settings.als_cal_target = chip->pdata->als_def_cal_target;

	/* populate the default lux table */
	if (chip->id == ID_TSL2584TSV) {
		memcpy(&taos_device_lux[0],
			&taos_device_lux_tsl2584tsv[0],
			sizeof(taos_device_lux_tsl2584tsv));
	} else {
		memcpy(&taos_device_lux[0],
			&taos_device_lux_tsl258x[0],
			sizeof(taos_device_lux_tsl258x));
	}

	/* Known external ALS reading used for calibration */
}

/* i2c smbus read access */
static int toas_i2c_smbus_read(struct i2c_client *client)
{
	int ret;

	pm_runtime_get_sync(&client->dev);
	ret = i2c_smbus_read_byte(client);
	pm_runtime_put(&client->dev);

	return ret;
}

/* i2c smbus write access */
static int toas_i2c_smbus_write(struct i2c_client *client, u8 command)
{
	int ret;

	pm_runtime_get_sync(&client->dev);
	ret = i2c_smbus_write_byte(client, command);
	pm_runtime_put(&client->dev);

	return ret;
}

static int toas_i2c_smbus_write_data(struct i2c_client *client, u8 command, u8 value)
{
	int ret;

	pm_runtime_get_sync(&client->dev);
	ret = i2c_smbus_write_byte_data(client, command, value);
	pm_runtime_put(&client->dev);

	return ret;
}

/*
 * Read a number of bytes starting at register (reg) location.
 * Return 0, or toas_i2c_smbus_write ERROR code.
 */
static int
taos_i2c_read(struct i2c_client *client, u8 reg, u8 *val, unsigned int len)
{
	int i, ret;

	for (i = 0; i < len; i++) {
		/* select register to write */
		ret = toas_i2c_smbus_write(client, (TSL258X_CMD_REG | reg));
		if (ret < 0) {
			dev_err(&client->dev, "taos_i2c_read failed to write"
				" register %x\n", reg);
			return ret;
		}
		/* read the data */
		*val = toas_i2c_smbus_read(client);
		val++;
		reg++;
	}
	return 0;
}

static int taos_set_power(struct tsl258x_chip *chip, int on)
{
	u8 cntrl = 0;
	int ret = 0;

	if (!on)
		cntrl = 0x0;
	else
		cntrl = TSL258X_CNTL_PWR_ON;
	ret = toas_i2c_smbus_write_data(chip->client,
					 TSL258X_CMD_REG | TSL258X_CNTRL, cntrl);
	return ret;
}

static int taos_set_enable(struct tsl258x_chip *chip, int en)
{
	u8 cntrl = 0;
	int ret = 0;

	if (!en)
		cntrl = TSL258X_CNTL_PWR_ON;
	else {
		mdelay(3);
		cntrl = TSL258X_CNTL_PWR_ON | TSL258X_CNTL_ADC_ENBL;
	}

	ret = toas_i2c_smbus_write_data(chip->client,
					TSL258X_CMD_REG | TSL258X_CNTRL, cntrl);
	if (!ret && en)
		chip->als_status = TSL258X_CHIP_WORKING;
	else if (!ret && !en)
		chip->als_status = TSL258X_CHIP_SUSPENDED;

	return ret;
}


static int taos_set_als_time(struct tsl258x_chip *chip, int ms)
{
	int ret = 0;
	int als_count = 0;
	int als_time = ms;

	if (chip->taos_settings.als_time == ms)
		return 0;

	if (als_time > ALS_TIME_MAX)
		als_time = ALS_TIME_MAX;
	else if (als_time < ALS_TIME_MIN)
		als_time = ALS_TIME_MIN;

	/* determine als integration regster, at least one cycle */
	als_count = ALS_TIME_TO_COUNT(als_time);
	/* convert back to time (encompasses overrides) */
	als_time = ALS_COUNT_TO_TIME(als_count);

	ret = toas_i2c_smbus_write_data(chip->client,
			TSL258X_CMD_REG | TSL258X_ALS_TIME,
			(256 - als_count));

	if (ret < 0)
		return ret;
	chip->taos_settings.als_time = als_time;
	/* set chip struct re scaling and saturation */
	chip->als_saturation = als_count * 922; /* 90% of full scale */
	chip->als_time_scale = (als_time + 25) / 50;

	return ret;
}

static int taos_set_gain(struct tsl258x_chip *chip, int gain)
{
	int i;
	int ret = 0;

	for (i = 0; i < TSL2584_ALS_GAIN_NUM; i++) {
		if (gain == tsl2584_als_gain_tbl[i].gain_val)
			break;
	}
	if (i >= TSL2584_ALS_GAIN_NUM)
		return -EINVAL;

	ret = toas_i2c_smbus_write_data(chip->client,
			TSL258X_CMD_REG | TSL258X_GAIN, gain);
	if (ret < 0)
		return ret;

	chip->taos_settings.als_gain_idex = tsl2584_als_gain_tbl[i].gain_idex;
	return ret;
}

static int taos_channel_data_valid(struct tsl258x_chip *chip)
{
	u8 cntrl = 0;
	int ret = 0;

	ret = taos_i2c_read(chip->client, TSL258X_CNTRL, &cntrl, 1);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed to read ctrnl\n");
		return ret;
	}
	/* is data new & valid */
	if (!(cntrl & TSL258X_STA_ADC_INTR) || !(cntrl & TSL258X_STA_ADC_VALID)) {
		dev_err(&chip->client->dev, "channel data not valid\n");
		return -ENODATA;
	}
	return ret;
}

static int taos_read_channel_data(struct tsl258x_chip *chip, int channel, u16 *val)
{
	int ret = 0;

	if (channel == TSL258X_CHANNEL0)
		ret = i2c_smbus_read_word_data(chip->client,
			TSL258X_CMD_REG | TSL258X_CMD_AUTO_INC | TSL258X_ALS_CHAN0LO);
	else
		ret = i2c_smbus_read_word_data(chip->client,
			TSL258X_CMD_REG | TSL258X_CMD_AUTO_INC | TSL258X_ALS_CHAN1LO);

	if (ret >= 0) {
		*val = ret;
		ret = 0;
	}

	return ret;
}

static int taos_chip_clear_interrupt(struct tsl258x_chip *chip)
{
	return toas_i2c_smbus_write(chip->client,
		TSL258X_CMD_REG | TSL258X_CMD_SPL_FN | TSL258X_CMD_ALS_INT_CLR);
}

static int taos_init_configure(struct tsl258x_chip *chip,
		struct tsl258x_platform_data *pdata)
{
	int ret = 0;

	ret = taos_set_power(chip, true);
	if (ret)
		return ret;
	ret = taos_set_als_time(chip, pdata->als_def_als_time);
	if (ret)
		return ret;
	ret = taos_set_gain(chip, pdata->als_def_gain);

	return ret;
}

/*
 * Reads and calculates current lux value.
 * The raw ch0 and ch1 values of the ambient light sensed in the last
 * integration cycle are read from the device.
 * Time scale factor array values are adjusted based on the integration time.
 * The raw values are multiplied by a scale factor, and device gain is obtained
 * using gain index. Limit checks are done next, then the ratio of a multiple
 * of ch1 value, to the ch0 value, is calculated. The array taos_device_lux[]
 * declared above is then scanned to find the first ratio value that is just
 * above the ratio we just calculated. The ch0 and ch1 multiplier constants in
 * the array are then used along with the time scale factor array values, to
 * calculate the lux.
 */
static int taos_get_lux(struct tsl258x_chip *chip)
{
	u16 ch0, ch1; /* separated ch0/ch1 data from device */
	u32 lux; /* raw lux calculated from device data */
	u32 ratio;
	struct taos_lux *p;
	int ret;
	u32 ch0lux = 0;
	u32 ch1lux = 0;
	u32 gain;
	int lux1;
	int lux2;

	if (chip->als_status != TSL258X_CHIP_WORKING) {
		/* device is not enabled */
		dev_err(&chip->client->dev, "taos_get_lux device is not enabled\n");
		return -EBUSY;
	}

	ret = taos_channel_data_valid(chip);
	if (ret < 0)
		return ret;

	ret = taos_read_channel_data(chip, TSL258X_CHANNEL0, &ch0);
	if (ret < 0) {
		dev_err(&chip->client->dev, "taos_get_lux failed to read ch0\n");
		return ret;
	}
	ret = taos_read_channel_data(chip, TSL258X_CHANNEL1, &ch1);
	if (ret < 0) {
		dev_err(&chip->client->dev, "taos_get_lux failed to read ch1\n");
		return ret;
	}

	/* clear status, really interrupt status (interrupts are off), but we use the bit anyway */
	ret = taos_chip_clear_interrupt(chip);
	if (ret < 0) {
		dev_err(&chip->client->dev, "taos_chip_clear_interrupt fail\n");
		return ret;
	}

	if ((ch0 >= chip->als_saturation) || (ch1 >= chip->als_saturation)) {
		chip->lux = TSL258X_LUX_CALC_OVER_FLOW;
		lux = chip->lux;
		return lux;
	}

	if (ch0 == 0) {
		/* have no data, so return LAST VALUE */
		dev_info(&chip->client->dev, "ch0 have no data\n");
		chip->lux = 0;
		lux = chip->lux;
		return lux;
	}

	if (chip->id == ID_TSL2584TSV) {
		gain = chip->taos_settings.als_time *
			tsl2584_als_gain_tbl[chip->taos_settings.als_gain_idex].gain_val;
		lux1 = (TSL2584TSV_CH0_COFF0 * ch0 - TSL2584TSV_CH1_COFF0 * ch1) / gain;
		lux2 = (TSL2584TSV_CH0_COFF1 * ch0 - TSL2584TSV_CH1_COFF1 * ch1) / gain;
		if ((lux1 < 0) && (lux2 < 0))
			return -ERANGE;
		lux = (lux1 >= lux2) ? lux1 : lux2;
	} else {
		/* calculate ratio */
		ratio = (ch1 << 15) / ch0;
		/* convert to unscaled lux using the pointer to the table */
		for (p = (struct taos_lux *) taos_device_lux;
		     p->ratio != 0 && p->ratio < ratio; p++)
			;

		if (p->ratio == 0) {
			lux = 0;
		} else {
			ch0lux = ((ch0 * p->ch0) +
				  (gainadj[chip->taos_settings.als_gain_idex].ch0 >> 1))
				 / gainadj[chip->taos_settings.als_gain_idex].ch0;
			ch1lux = ((ch1 * p->ch1) +
				  (gainadj[chip->taos_settings.als_gain_idex].ch1 >> 1))
				 / gainadj[chip->taos_settings.als_gain_idex].ch1;
			lux = ch0lux - ch1lux;
		}

		/* note: lux is 31 bit max at this point */
		if (ch1lux > ch0lux) {
			dev_info(&chip->client->dev, "No Data - Return last value\n");
			chip->lux = 0;
			lux = chip->lux;
			return lux;
		}

		/* adjust for active time scale */
		if (chip->als_time_scale == 0)
			lux = 0;
		else
			lux = (lux + (chip->als_time_scale >> 1)) / chip->als_time_scale;

		/* adjust for active gain scale */
		lux >>= 13; /* tables have factor of 8192 builtin for accuracy */
		lux = (lux * chip->taos_settings.als_gain_trim + 500) / 1000;
	}

	if (lux > TSL258X_LUX_CALC_OVER_FLOW)	/* check for overflow */
		lux = TSL258X_LUX_CALC_OVER_FLOW;

	/* Update the structure with the latest VALID lux. */
	chip->lux = lux;
	return lux;
}

/*
 * Obtain single reading and calculate the als_gain_trim (later used
 * to derive actual lux).
 * Return updated gain_trim value.
 */
static int taos_als_calibrate(struct tsl258x_chip *chip)
{
	u8 reg_val;
	unsigned int gain_trim_val;
	int ret;
	int lux_val;

	ret = toas_i2c_smbus_write(chip->client,
				   (TSL258X_CMD_REG | TSL258X_CNTRL));
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"taos_als_calibrate failed to reach the CNTRL register, ret=%d\n",
			ret);
		return ret;
	}

	reg_val = toas_i2c_smbus_read(chip->client);
	if ((reg_val & (TSL258X_CNTL_ADC_ENBL | TSL258X_CNTL_PWR_ON))
			!= (TSL258X_CNTL_ADC_ENBL | TSL258X_CNTL_PWR_ON)) {
		dev_err(&chip->client->dev,
			"taos_als_calibrate failed: device not enabled\n");
		return -1;
	}

	ret = toas_i2c_smbus_write(chip->client,
				   (TSL258X_CMD_REG | TSL258X_CNTRL));
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"taos_als_calibrate failed to reach the STATUS register, ret=%d\n",
			ret);
		return ret;
	}
	reg_val = toas_i2c_smbus_read(chip->client);

	if ((reg_val & TSL258X_STA_ADC_VALID) != TSL258X_STA_ADC_VALID) {
		dev_err(&chip->client->dev,
			"taos_als_calibrate failed: STATUS - ADC not valid.\n");
		return -ENODATA;
	}
	lux_val = taos_get_lux(chip);
	if (lux_val < 0) {
		dev_err(&chip->client->dev, "taos_als_calibrate failed to get lux\n");
		return lux_val;
	}
	gain_trim_val = (unsigned int) (((chip->taos_settings.als_cal_target)
			* chip->taos_settings.als_gain_trim) / lux_val);

	if ((gain_trim_val < 250) || (gain_trim_val > 4000)) {
		dev_err(&chip->client->dev,
			"taos_als_calibrate failed: trim_val of %d is out of range\n",
			gain_trim_val);
		return -ENODATA;
	}
	chip->taos_settings.als_gain_trim = (int) gain_trim_val;

	return (int) gain_trim_val;
}

static void taos_workqueue_handler(struct work_struct *work)
{
	int ret = 0;
	struct tsl258x_chip *chip =
			container_of(work, struct tsl258x_chip, data_work);

	mutex_lock(&chip->als_mutex);
	if (chip->als_status != TSL258X_CHIP_WORKING) {
		mutex_unlock(&chip->als_mutex);
		return;
	}
	ret = taos_get_lux(chip);
	if (ret > 0) {
		/* report the resultant_lux level */
		input_event(chip->input, EV_MSC, MSC_RAW, ret);
		/* Sync it up */
		input_sync(chip->input);
	}
	mutex_unlock(&chip->als_mutex);
	mod_timer(&chip->timer,
			jiffies + msecs_to_jiffies(chip->taos_settings.als_odr));
}


static void taos_timer_handler(unsigned long private)
{
	struct tsl258x_chip *chip = (struct tsl258x_chip *)private;

	if (chip->als_status == TSL258X_CHIP_WORKING)
		queue_work(tsl258x_wq, &chip->data_work);
}

/* Sysfs Interface Functions */

static ssize_t taos_power_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int enable = 0;

	if (chip->als_status == TSL258X_CHIP_WORKING)
		enable = 1;
	else
		enable = 0;

	return sprintf(buf, "%d\n", enable);
}

static ssize_t taos_power_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	unsigned int value;
	int err;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	if ((value != 0) && (value != 1))
		return -EINVAL;

	mutex_lock(&chip->als_mutex);
	if (value == 1) {
		/* turn on light  sensor */
		if (chip->als_status != TSL258X_CHIP_WORKING) {
			err = taos_set_enable(chip, true);
			if (err) {
				dev_err(&client->dev, "taos_set_enable true failed\n");
				len = err;
				goto enable_als_err;
			} else {
				mod_timer(&chip->timer,
					jiffies + msecs_to_jiffies(chip->taos_settings.als_odr));
			}
		}
	} else {
		/* turn off light sensor */
		err = taos_set_enable(chip, false);
		if (err) {
			dev_err(&client->dev, "taos_set_enable false failed\n");
			len = err;
			goto enable_als_err;
		}
	}

enable_als_err:
	mutex_unlock(&chip->als_mutex);
	return len;
}

static ssize_t taos_als_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%d\n", chip->taos_settings.als_odr);
}

static ssize_t taos_als_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	unsigned int value;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	if (!value)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);
	chip->taos_settings.als_odr = value;
	mutex_unlock(&chip->als_mutex);

	return len;
}


static ssize_t taos_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int value;

	value = tsl2584_als_gain_tbl[chip->taos_settings.als_gain_idex].gain_val;

	return sprintf(buf, "%d\n", value);
}

static ssize_t taos_gain_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int i, err, value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	for (i = 0; i < TSL2584_ALS_GAIN_NUM; i++) {
		if (tsl2584_als_gain_tbl[i].gain_val == value)
			break;
	}
	if (i >= TSL2584_ALS_GAIN_NUM)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);
	err = taos_set_gain(chip, value);
	mutex_unlock(&chip->als_mutex);
	if (err)
		len = err;

	return len;
}

static ssize_t taos_gain_available_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "1 8 16 111");
}

static ssize_t taos_als_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%d\n", chip->taos_settings.als_time);
}

static ssize_t taos_als_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	unsigned int value;
	int err;

	if (kstrtouint(buf, 0, &value))
		return -EINVAL;
	if (!value)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);
	err = taos_set_als_time(chip, value);
	mutex_unlock(&chip->als_mutex);
	if (err)
		len = err;

	return len;
}

static ssize_t taos_als_time_available_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n",
		"50 100 150 200 250 300 350 400 450 500 550 600 650");
}

static ssize_t taos_als_trim_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%d\n", chip->taos_settings.als_gain_trim);
}

static ssize_t taos_als_trim_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (value)
		chip->taos_settings.als_gain_trim = value;

	return len;
}

static ssize_t taos_als_cal_target_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%d\n", chip->taos_settings.als_cal_target);
}

static ssize_t taos_als_cal_target_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (value)
		chip->taos_settings.als_cal_target = value;

	return len;
}

static ssize_t taos_lux_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int ret;
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&chip->als_mutex);
	ret = taos_get_lux(chip);
	mutex_unlock(&chip->als_mutex);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t taos_do_calibrate(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (value == 1) {
		mutex_lock(&chip->als_mutex);
		taos_als_calibrate(chip);
		mutex_unlock(&chip->als_mutex);
	}

	return len;
}

static ssize_t taos_luxtable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int offset = 0;

	for (i = 0; i < ARRAY_SIZE(taos_device_lux); i++) {
		offset += sprintf(buf + offset, "%d,%d,%d,",
				  taos_device_lux[i].ratio,
				  taos_device_lux[i].ch0,
				  taos_device_lux[i].ch1);
		if (taos_device_lux[i].ratio == 0) {
			/* We just printed the first "0" entry.
			 * Now get rid of the extra "," and break. */
			offset--;
			break;
		}
	}

	offset += sprintf(buf + offset, "\n");
	return offset;
}

static ssize_t taos_luxtable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(to_i2c_client(dev));
	int value[ARRAY_SIZE(taos_device_lux)*3 + 1];
	int n;

	get_options(buf, ARRAY_SIZE(value), value);

	/* We now have an array of ints starting at value[1], and
	 * enumerated by value[0].
	 * We expect each group of three ints is one table entry,
	 * and the last table entry is all 0.
	 */
	n = value[0];
	if (chip->id == ID_TSL2584TSV) {
		if (n != 3) {
			dev_info(dev, "LUX TABLE INPUT ERROR "
				"TSL2584TSV input as 0,COFF_CH0,COFF_CH1\n");
			return -EINVAL;
		}

		if (value[(n - 2)] != 0) {
			dev_info(dev, "LUX TABLE INPUT ERROR, "
				"TSL2584TSV ratio must be 0, not %d\n",
				value[(n - 2)]);
			return -EINVAL;
		}
	} else {
		if ((n % 3) || n < 6 || n > ((ARRAY_SIZE(taos_device_lux) - 1) * 3)) {
			dev_info(dev, "LUX TABLE INPUT ERROR 1 Value[0]=%d\n", n);
			return -EINVAL;
		}

		if ((value[(n - 2)] | value[(n - 1)] | value[n]) != 0) {
			dev_info(dev, "LUX TABLE INPUT ERROR 2 Value[0]=%d\n", n);
			return -EINVAL;
		}
	}

	if (chip->als_status == TSL258X_CHIP_WORKING)
		taos_set_enable(chip, false);

	/* Zero out the table */
	memset(taos_device_lux, 0, sizeof(taos_device_lux));
	memcpy(taos_device_lux, &value[1], (value[0] * 4));

	taos_set_enable(chip, true);

	return len;
}

static DEVICE_ATTR(als_enable, S_IRUGO | S_IWUSR,
		taos_power_state_show, taos_power_state_store);

static DEVICE_ATTR(als_delay, S_IRUGO | S_IWUSR,
		taos_als_delay_show, taos_als_delay_store);

static DEVICE_ATTR(illuminance0_calibscale, S_IRUGO | S_IWUSR,
		taos_gain_show, taos_gain_store);
static DEVICE_ATTR(illuminance0_calibscale_available, S_IRUGO,
		taos_gain_available_show, NULL);

static DEVICE_ATTR(illuminance0_integration_time, S_IRUGO | S_IWUSR,
		taos_als_time_show, taos_als_time_store);
static DEVICE_ATTR(illuminance0_integration_time_available, S_IRUGO,
		taos_als_time_available_show, NULL);

static DEVICE_ATTR(illuminance0_calibbias, S_IRUGO | S_IWUSR,
		taos_als_trim_show, taos_als_trim_store);

static DEVICE_ATTR(illuminance0_input_target, S_IRUGO | S_IWUSR,
		taos_als_cal_target_show, taos_als_cal_target_store);

static DEVICE_ATTR(illuminance0_input, S_IRUGO, taos_lux_show, NULL);
static DEVICE_ATTR(illuminance0_calibrate, S_IWUSR, NULL, taos_do_calibrate);
static DEVICE_ATTR(illuminance0_lux_table, S_IRUGO | S_IWUSR,
		taos_luxtable_show, taos_luxtable_store);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_als_enable.attr,
	&dev_attr_als_delay.attr,
	&dev_attr_illuminance0_calibscale.attr,			/* Gain  */
	&dev_attr_illuminance0_calibscale_available.attr,
	&dev_attr_illuminance0_integration_time.attr,	/* I time*/
	&dev_attr_illuminance0_integration_time_available.attr,
	&dev_attr_illuminance0_calibbias.attr,			/* trim  */
	&dev_attr_illuminance0_input_target.attr,
	&dev_attr_illuminance0_input.attr,
	&dev_attr_illuminance0_calibrate.attr,
	&dev_attr_illuminance0_lux_table.attr,
	NULL
};

static struct attribute_group tsl258x_attribute_group = {
	.attrs = sysfs_attrs_ctrl,
};

/* Use the default register values to identify the actual chip */
static int taos_tsl258x_chip_id(unsigned char *bufp)
{
	int id;

	if ((bufp[TSL258X_ID2] & 0x80) == 0x80)
		id = ID_TSL2584TSV;
	else if ((bufp[TSL258X_ID2] & 0x30) == 0x30)
		id = ID_TSL2583;
	else if ((bufp[TSL258X_ID2] & 0x30) == 0)
		id = ID_TSL2581;
	else
		id = ID_UNKNOWN;

	return id;
}

/* Use the default register values to identify the Taos device */
static int taos_tsl258x_device(unsigned char *bufp)
{
	return ((bufp[TSL258X_CHIPID] & 0xf0) == 0x90);
}

/*
 * Client probe function - When a valid device is found, the driver's device
 * data structure is updated, and initialization completes successfully.
 */
static int taos_probe(struct i2c_client *clientp,
		      const struct i2c_device_id *idp)
{
	int i, ret = -ENOMEM;
	unsigned char buf[TSL258X_MAX_DEVICE_REGS];
	struct tsl258x_chip *chip;

	if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&clientp->dev, "taos_probe() i2c smbus functions unsupported\n");
		return -EOPNOTSUPP;
	}

	chip = kzalloc(sizeof(struct tsl258x_chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&clientp->dev, "couldn't allocate memory for chip\n");
		return -ENOMEM;
	}

	chip->client = clientp;
	chip->pdata = clientp->dev.platform_data;
	i2c_set_clientdata(clientp, chip);

	chip->input = input_allocate_device();
	/* couldn't allocate input device */
	if (!chip->input) {
		dev_err(&clientp->dev, "couldn't allocate input device\n");
		ret = -ENOMEM;
		goto err_input_alloc_failed;
	}

	/* set up the input device, this name is the dev-node-name under /dev/input/ */
	chip->input->name = "tsl2584 ambient light sensor";
	input_set_capability(chip->input, EV_MSC, MSC_RAW);

	/* attempt to register the input device */
	ret = input_register_device(chip->input);
	if (ret < 0) {
		dev_err(&clientp->dev, "input device register failed:%d\n", ret);
		goto err_input_register_failed;
	}

	/* Register sysfs hooks */
	ret = sysfs_create_group(&clientp->dev.kobj, &tsl258x_attribute_group);
	if (ret) {
		dev_err(&clientp->dev, "sysfs_create_group failed: %d\n", ret);
		goto err_sysfs_failed;
	}

	tsl258x_wq = create_workqueue("tsl258x");
	if (!tsl258x_wq) {
		dev_err(&clientp->dev, "work queue create failed:%d\n", ret);
		ret = -ENOMEM;
		goto err_tsl_wq_failed;
	}

	INIT_WORK(&chip->data_work, taos_workqueue_handler);
	setup_timer(&chip->timer, taos_timer_handler, (unsigned long)chip);

	mutex_init(&chip->als_mutex);
	chip->als_status = TSL258X_CHIP_UNKNOWN;

	if (((struct tsl258x_platform_data *)chip->pdata)->gpio_conf)
		((struct tsl258x_platform_data *)chip->pdata)->gpio_conf();

	for (i = 0; i < TSL258X_MAX_DEVICE_REGS; i++) {
		ret = toas_i2c_smbus_write(clientp,
				(TSL258X_CMD_REG | (TSL258X_CNTRL + i)));
		if (ret < 0) {
			dev_err(&clientp->dev, "toas_i2c_smbus_write() to cmd "
				"reg failed in taos_probe(), err = %d\n", ret);
			goto err_tsl_hw_failed;
		}
		ret = toas_i2c_smbus_read(clientp);
		if (ret < 0) {
			dev_err(&clientp->dev, "toas_i2c_smbus_read from "
				"reg failed in taos_probe(), err = %d\n", ret);

			goto err_tsl_hw_failed;
		}
		buf[i] = ret;
	}

	if (!taos_tsl258x_device(buf)) {
		dev_info(&clientp->dev, "i2c device found but does not match "
			"expected id in taos_probe()\n");
		goto err_tsl_hw_failed;
	}

	if (idp->driver_data != ID_UNKNOWN)
		chip->id = (int) idp->driver_data;
	else
		chip->id = taos_tsl258x_chip_id(buf);

	ret = toas_i2c_smbus_write(clientp, (TSL258X_CMD_REG | TSL258X_CNTRL));
	if (ret < 0) {
		dev_err(&clientp->dev, "toas_i2c_smbus_write() to cmd reg "
			"failed in taos_probe(), err = %d\n", ret);
		goto err_tsl_hw_failed;
	}

	/* Load up the V2 defaults (these are hard coded defaults for now) */
	taos_defaults(chip);

	/* Make sure the chip is on */
	ret = taos_init_configure(chip, chip->pdata);
	if (ret)
		goto err_tsl_hw_failed;

	pm_runtime_enable(&clientp->dev);

	dev_info(&clientp->dev, "ALS found, %s.\n", tsl2583x_get_name(chip));
	return 0;

err_tsl_hw_failed:
	destroy_workqueue(tsl258x_wq);
err_tsl_wq_failed:
	sysfs_remove_group(&clientp->dev.kobj, &tsl258x_attribute_group);
err_sysfs_failed:
	input_unregister_device(chip->input);
err_input_register_failed:
	/* REVERTME: skip the free device call in case no als is detected */
	// input_free_device(chip->input);
err_input_alloc_failed:
	kfree(chip);
	return ret;
}

static int taos_suspend(struct i2c_client *client, pm_message_t state)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&chip->als_mutex);
	if (chip->als_status == TSL258X_CHIP_WORKING) {
		ret = taos_set_power(chip, false);
		chip->als_status = TSL258X_CHIP_SUSPENDED;
	}
	mutex_unlock(&chip->als_mutex);
	return ret;
}

static int taos_resume(struct i2c_client *client)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&chip->als_mutex);
	if (chip->als_status == TSL258X_CHIP_SUSPENDED)
		ret = taos_set_power(chip, true);
	mutex_unlock(&chip->als_mutex);
	return ret;
}


static int taos_remove(struct i2c_client *client)
{
	struct tsl258x_chip *chip = i2c_get_clientdata(client);

	pm_runtime_disable(&client->dev);

	if (tsl258x_wq)
		destroy_workqueue(tsl258x_wq);
	del_timer(&chip->timer);
	sysfs_remove_group(&client->dev.kobj, &tsl258x_attribute_group);
	if (chip->input) {
		input_unregister_device(chip->input);
		input_free_device(chip->input);
	}
	kfree(chip);

	return 0;
}

static struct i2c_device_id taos_idtable[] = {
	{ "tsl258x", ID_UNKNOWN },
	{ "tsl2580", ID_TSL2580 },
	{ "tsl2581", ID_TSL2581 },
	{ "tsl2583", ID_TSL2583 },
	{ "tsl2584", ID_TSL2584TSV },
	{}
};
MODULE_DEVICE_TABLE(i2c, taos_idtable);

static char *tsl2583x_get_name(struct tsl258x_chip *chip)
{
	int i;

	for (i = 0; i < sizeof(taos_idtable) / sizeof(taos_idtable[0]); i++) {
		if ((int) taos_idtable[i].driver_data ==  chip->id)
			return taos_idtable[i].name;
	}
	return "unknown sensor";
}

/* Driver definition */
static struct i2c_driver taos_driver = {
	.driver = {
		.name = TSL258X_NAME,
	},
	.id_table = taos_idtable,
	.suspend	= taos_suspend,
	.resume		= taos_resume,
	.probe = taos_probe,
	.remove = taos_remove,
};

static int __init taos_init(void)
{
	return i2c_add_driver(&taos_driver);
}

static void __exit taos_exit(void)
{
	i2c_del_driver(&taos_driver);
}

module_init(taos_init);
module_exit(taos_exit);

MODULE_AUTHOR("J. August Brenner<jbrenner@taosinc.com>");
MODULE_DESCRIPTION("TAOS tsl258x ambient light sensor driver");
MODULE_LICENSE("GPL");

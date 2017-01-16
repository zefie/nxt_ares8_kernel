/*
 * STMicroelectronics lsm6ds3 core driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <asm/unaligned.h>
#include <linux/iio/common/st_sensors.h>
#include <linux/lnw_gpio.h>
#include <linux/pm_runtime.h>
#include "st_lsm6ds3.h"

#define MS_TO_NS(msec)				((msec) * 1000 * 1000)

#ifndef MAX
#define MAX(a, b)				(((a) > (b)) ? (a) : (b))
#endif

#define MIN_BNZ(a, b)				(((a) < (b)) ? ((a == 0) ? \
						(b) : (a)) : ((b == 0) ? \
						(a) : (b)))

/* COMMON VALUES FOR ACCEL-GYRO SENSORS */
#define ST_LSM6DS3_WAI_ADDRESS			0x0f
#define ST_LSM6DS3_WAI_EXP			0x69
#define ST_LSM6DS3_AXIS_EN_MASK			0x38
#define ST_LSM6DS3_INT1_ADDR			0x0d
#define ST_LSM6DS3_INT2_ADDR			0x0e
#define ST_LSM6DS3_MD1_ADDR			0x5e
#define ST_LSM6DS3_ODR_LIST_NUM			5
#define ST_LSM6DS3_ODR_POWER_OFF_VAL		0x00
#define ST_LSM6DS3_ODR_26HZ_VAL			0x02
#define ST_LSM6DS3_ODR_52HZ_VAL			0x03
#define ST_LSM6DS3_ODR_104HZ_VAL		0x04
#define ST_LSM6DS3_ODR_208HZ_VAL		0x05
#define ST_LSM6DS3_ODR_416HZ_VAL		0x06
#define ST_LSM6DS3_FS_LIST_NUM			4
#define ST_LSM6DS3_BDU_ADDR			0x12
#define ST_LSM6DS3_BDU_MASK			0x40
#define ST_LSM6DS3_EN_BIT			0x01
#define ST_LSM6DS3_DIS_BIT			0x00
#define ST_LSM6DS3_FUNC_EN_ADDR			0x19
#define ST_LSM6DS3_FUNC_EN_MASK			0x04
#define ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR		0x01
#define ST_LSM6DS3_FUNC_CFG_ACCESS_MASK		0x01
#define ST_LSM6DS3_FUNC_CFG_ACCESS_MASK2	0x04
#define ST_LSM6DS3_FUNC_CFG_REG2_MASK		0x80
#define ST_LSM6DS3_FUNC_CFG_START1_ADDR		0x62
#define ST_LSM6DS3_FUNC_CFG_START2_ADDR		0x63
#define ST_LSM6DS3_PASS_THROUGH_MODE_ADDR	0x1a
#define ST_LSM6DS3_PASS_THROUGH_MODE_MASK	0x04
#define ST_LSM6DS3_INTER_PULLUP_ADDR		0x1a
#define ST_LSM6DS3_INTER_PULLUP_MASK		0x08
#define ST_LSM6DS3_SENSORHUB_ADDR		0x1a
#define ST_LSM6DS3_SENSORHUB_MASK		0x01
#define ST_LSM6DS3_STARTCONFIG_ADDR		0x1a
#define ST_LSM6DS3_STARTCONFIG_MASK		0x10
#define ST_LSM6DS3_SELFTEST_ADDR		0x14
#define ST_LSM6DS3_SELFTEST_ACCEL_MASK		0x03
#define ST_LSM6DS3_SELFTEST_GYRO_MASK		0x0c
#define ST_LSM6DS3_SELF_TEST_DISABLED_VAL	0x00
#define ST_LSM6DS3_SELF_TEST_POS_SIGN_VAL	0x01
#define ST_LSM6DS3_SELF_TEST_NEG_ACCEL_SIGN_VAL	0x02
#define ST_LSM6DS3_SELF_TEST_NEG_GYRO_SIGN_VAL	0x03
#define ST_LSM6DS3_LIR_ADDR			0x58
#define ST_LSM6DS3_LIR_MASK			0x01
#define ST_LSM6DS3_TIMER_EN_ADDR		0x58
#define ST_LSM6DS3_TIMER_EN_MASK		0x80
#define ST_LSM6DS3_PEDOMETER_EN_ADDR		0x58
#define ST_LSM6DS3_PEDOMETER_EN_MASK		0x40
#define ST_LSM6DS3_INT2_ON_INT1_ADDR		0x13
#define ST_LSM6DS3_INT2_ON_INT1_MASK		0x20
#define ST_LSM6DS3_MIN_DURATION_MS		1638
#define ST_LSM6DS3_ROUNDING_ADDR		0x16
#define ST_LSM6DS3_ROUNDING_MASK		0x04
#define ST_LSM6DS3_FIFO_MODE_ADDR		0x0a
#define ST_LSM6DS3_FIFO_MODE_MASK		0x07
#define ST_LSM6DS3_FIFO_MODE_BYPASS		0x00
#define ST_LSM6DS3_FIFO_MODE_CONTINUOS		0x06
#define ST_LSM6DS3_FIFO_THRESHOLD_IRQ_MASK	0x08
#define ST_LSM6DS3_FIFO_ODR_ADDR		0x0a
#define ST_LSM6DS3_FIFO_ODR_MASK		0x78
#define ST_LSM6DS3_FIFO_ODR_MAX			0x08
#define ST_LSM6DS3_FIFO_ODR_OFF			0x00
#define ST_LSM6DS3_FIFO_DECIMATOR_ADDR		0x08
#define ST_LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK	0x07
#define ST_LSM6DS3_FIFO_GYRO_DECIMATOR_MASK	0x38
#define ST_LSM6DS3_FIFO_DECIMATOR2_ADDR		0x09
#define ST_LSM6DS3_FIFO_DS3_DECIMATOR_MASK	0x07
#define ST_LSM6DS3_FIFO_DS4_DECIMATOR_MASK	0x38
#define ST_LSM6DS3_FIFO_THR_L_ADDR		0x06
#define ST_LSM6DS3_FIFO_THR_H_ADDR		0x07
#define ST_LSM6DS3_FIFO_THR_H_MASK		0x0f
#define ST_LSM6DS3_FIFO_THR_IRQ_MASK		0x08
#define ST_LSM6DS3_RESET_ADDR			0x12
#define ST_LSM6DS3_RESET_MASK			0x01
#define ST_LSM6DS3_MAX_FIFO_SIZE		4320
#define ST_LSM6DS3_MAX_FIFO_LENGHT		(ST_LSM6DS3_MAX_FIFO_SIZE / \
					ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE)

#define	ST_LSM6DS3_STOP_ON_FTH_ADDR		0x13
#define	ST_LSM6DS3_STOP_ON_FTH_MASK		0x01

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define ST_LSM6DS3_ACCEL_ODR_ADDR		0x10
#define ST_LSM6DS3_ACCEL_ODR_MASK		0xf0
#define ST_LSM6DS3_ACCEL_FS_ADDR		0x10
#define ST_LSM6DS3_ACCEL_FS_MASK		0x0c
#define ST_LSM6DS3_ACCEL_FS_2G_VAL		0x00
#define ST_LSM6DS3_ACCEL_FS_4G_VAL		0x02
#define ST_LSM6DS3_ACCEL_FS_8G_VAL		0x03
#define ST_LSM6DS3_ACCEL_FS_16G_VAL		0x01
#define ST_LSM6DS3_ACCEL_FS_2G_SENSITIVITY	61	/*  ug/LSB */
#define ST_LSM6DS3_ACCEL_FS_4G_SENSITIVITY	122	/*  ug/LSB */
#define ST_LSM6DS3_ACCEL_FS_8G_SENSITIVITY	244	/*  ug/LSB */
#define ST_LSM6DS3_ACCEL_FS_16G_SENSITIVITY	488	/*  ug/LSB */
#define ST_LSM6DS3_ACCEL_FS_2G_GAIN		IIO_G_TO_M_S_2(ST_LSM6DS3_ACCEL_FS_2G_SENSITIVITY)
#define ST_LSM6DS3_ACCEL_FS_4G_GAIN		IIO_G_TO_M_S_2(ST_LSM6DS3_ACCEL_FS_4G_SENSITIVITY)
#define ST_LSM6DS3_ACCEL_FS_8G_GAIN		IIO_G_TO_M_S_2(ST_LSM6DS3_ACCEL_FS_8G_SENSITIVITY)
#define ST_LSM6DS3_ACCEL_FS_16G_GAIN		IIO_G_TO_M_S_2(ST_LSM6DS3_ACCEL_FS_16G_SENSITIVITY)
#define ST_LSM6DS3_ACCEL_OUT_X_L_ADDR		0x28
#define ST_LSM6DS3_ACCEL_OUT_Y_L_ADDR		0x2a
#define ST_LSM6DS3_ACCEL_OUT_Z_L_ADDR		0x2c
#define ST_LSM6DS3_ACCEL_AXIS_EN_ADDR		0x18
#define ST_LSM6DS3_ACCEL_STD			3

/* CUSTOM VALUES FOR GYRO SENSOR */
#define ST_LSM6DS3_GYRO_ODR_ADDR		0x11
#define ST_LSM6DS3_GYRO_ODR_MASK		0xf0
#define ST_LSM6DS3_GYRO_FS_ADDR			0x11
#define ST_LSM6DS3_GYRO_FS_MASK			0x0c
#define ST_LSM6DS3_GYRO_FS_245_VAL		0x00
#define ST_LSM6DS3_GYRO_FS_500_VAL		0x01
#define ST_LSM6DS3_GYRO_FS_1000_VAL		0x02
#define ST_LSM6DS3_GYRO_FS_2000_VAL		0x03
#define ST_LSM6DS3_GYRO_FS_245_SENSITIVITY	8750	/* udps/LSB */
#define ST_LSM6DS3_GYRO_FS_500_SENSITIVITY	17500	/* udps/LSB */
#define ST_LSM6DS3_GYRO_FS_1000_SENSITIVITY	35000	/* udps/LSB */
#define ST_LSM6DS3_GYRO_FS_2000_SENSITIVITY	70000	/* udps/LSB */
#define ST_LSM6DS3_GYRO_FS_245_GAIN		IIO_DEGREE_TO_RAD(ST_LSM6DS3_GYRO_FS_245_SENSITIVITY)
#define ST_LSM6DS3_GYRO_FS_500_GAIN		IIO_DEGREE_TO_RAD(ST_LSM6DS3_GYRO_FS_500_SENSITIVITY)
#define ST_LSM6DS3_GYRO_FS_1000_GAIN		IIO_DEGREE_TO_RAD(ST_LSM6DS3_GYRO_FS_1000_SENSITIVITY)
#define ST_LSM6DS3_GYRO_FS_2000_GAIN		IIO_DEGREE_TO_RAD(ST_LSM6DS3_GYRO_FS_2000_SENSITIVITY)
#define ST_LSM6DS3_GYRO_OUT_X_L_ADDR		0x22
#define ST_LSM6DS3_GYRO_OUT_Y_L_ADDR		0x24
#define ST_LSM6DS3_GYRO_OUT_Z_L_ADDR		0x26
#define ST_LSM6DS3_GYRO_AXIS_EN_ADDR		0x19
#define ST_LSM6DS3_GYRO_STD			6

/* CUSTOM VALUES FOR SIGNIFICANT MOTION SENSOR */
#define ST_LSM6DS3_SIGN_MOTION_EN_ADDR		0x19
#define ST_LSM6DS3_SIGN_MOTION_EN_MASK		0x01

/* CUSTOM VALUES FOR STEP DETECTOR SENSOR */
#define ST_LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK	0x80

/* CUSTOM VALUES FOR STEP COUNTER SENSOR */
#define ST_LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK	0x80
#define ST_LSM6DS3_STEP_COUNTER_OUT_L_ADDR	0x4b
#define ST_LSM6DS3_STEP_COUNTER_RES_ADDR	0x19
#define ST_LSM6DS3_STEP_COUNTER_RES_MASK	0x06
#define ST_LSM6DS3_STEP_COUNTER_RES_ALL_EN	0x03
#define ST_LSM6DS3_STEP_COUNTER_RES_FUNC_EN	0x02
#define ST_LSM6DS3_STEP_COUNTER_DURATION_ADDR	0x15

#define	ST_LSM6DS3_STEP_COUNTER_THS_MIN_MASK		0x1f
#define	ST_LSM6DS3_STEP_COUNTER_THS_MIN_DEF_VAL		0x19
#define	ST_LSM6DS3_STEP_COUNTER_PEDO_THS_ADDR		0x0f
#define	ST_LSM6DS3_STEP_COUNTER_DEB_STEP_MASK		0x07
#define	ST_LSM6DS3_STEP_COUNTER_DEB_STEP_DEF_VAL	0x07
#define	ST_LSM6DS3_STEP_COUNTER_PEDO_DEB_ADDR		0x14


/* CUSTOM VALUES FOR TILT SENSOR */
#define ST_LSM6DS3_TILT_EN_ADDR			0x58
#define ST_LSM6DS3_TILT_EN_MASK			0x20
#define ST_LSM6DS3_TILT_DRDY_IRQ_MASK		0x02

/* STATUS REGISTER */
#define ST_LSM6DS3_STAT_REG_ADDR		0x1e
#define ST_LSM6DS3_XLDA_MASK			0x01
#define ST_LSM6DS3_GDA_MASK				0x02

#if defined(CONFIG_ST_LSM6DS3_SELFTEST) || defined(CONFIG_ST_LSM6DS3_CAL_SUPPORT)
#define MAX_WHILE_COUNTER	150
#endif

#ifdef CONFIG_ST_LSM6DS3_SELFTEST
/* Macro for manufacturing test  */
#define FACT_SELF_SAMPLE_COUNT	5
#define DELAY_FOR_OUT_STABLE	200	/* 200ms */
#define ACCEL_MIN_DIFF	90000		/* ug */
#define ACCEL_MAX_DIFF	1700000		/* ug */
#define GYRO_MIN_DIFF	150000000	/* udps @2000dps */
#define GYRO_MAX_DIFF	700000000	/* udps @2000dps */
#define ABS(n)			(((n) > 0) ? (n) : (0 - (n)))
#endif

#ifdef CONFIG_ST_LSM6DS3_CAL_SUPPORT
/* Macro for calibrate */
#define	CALIBRATE_SAMPLE_COUNT		50
#define	GRAVITY_ACCEL_LSB_2G		(1000000 / ST_LSM6DS3_ACCEL_FS_2G_SENSITIVITY)
#define	SIGN_X_A			1
#define	SIGN_Y_A			(-1)
#define	SIGN_Z_A			(-1)
#define	SIGN_X_G			1
#define	SIGN_Y_G			(-1)
#define	SIGN_Z_G			(-1)
#endif

#define	IIO_BUFFER_KFIFO_DEFAULT_LEN	2

#define ST_LSM6DS3_ACCEL_SUFFIX_NAME		"accel"
#define ST_LSM6DS3_GYRO_SUFFIX_NAME		"gyro"
#define ST_LSM6DS3_STEP_COUNTER_SUFFIX_NAME	"step_c"
#define ST_LSM6DS3_STEP_DETECTOR_SUFFIX_NAME	"step_d"
#define ST_LSM6DS3_SIGN_MOTION_SUFFIX_NAME	"sign_motion"
#define ST_LSM6DS3_TILT_SUFFIX_NAME		"tilt"
#define ST_LSM6DS3_WAKEUP_SUFFIX_NAME	"wk"

#define ST_LSM6DS3_DEV_ATTR_SAMP_FREQ() \
		IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO, \
			st_lsm6ds3_sysfs_get_sampling_frequency, \
			st_lsm6ds3_sysfs_set_sampling_frequency)

#define ST_LSM6DS3_DEV_ATTR_SAMP_FREQ_AVAIL() \
		IIO_DEV_ATTR_SAMP_FREQ_AVAIL( \
			st_lsm6ds3_sysfs_sampling_frequency_avail)

#define ST_LSM6DS3_DEV_ATTR_SCALE_AVAIL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			st_lsm6ds3_sysfs_scale_avail, NULL , 0);

#ifdef CONFIG_ST_LSM6DS3_CAL_SUPPORT
static int accel_cal_data[3], gyro_cal_data[3];
#endif
static bool stay_wake;

static struct st_lsm6ds3_selftest_table {
	char *string_mode;
	u8 accel_value;
	u8 gyro_value;
	u8 accel_mask;
	u8 gyro_mask;
} st_lsm6ds3_selftest_table[] = {
	[0] = {
		.string_mode = "disabled",
		.accel_value = ST_LSM6DS3_SELF_TEST_DISABLED_VAL,
		.gyro_value = ST_LSM6DS3_SELF_TEST_DISABLED_VAL,
	},
	[1] = {
		.string_mode = "positive-sign",
		.accel_value = ST_LSM6DS3_SELF_TEST_POS_SIGN_VAL,
		.gyro_value = ST_LSM6DS3_SELF_TEST_POS_SIGN_VAL
	},
	[2] = {
		.string_mode = "negative-sign",
		.accel_value = ST_LSM6DS3_SELF_TEST_NEG_ACCEL_SIGN_VAL,
		.gyro_value = ST_LSM6DS3_SELF_TEST_NEG_GYRO_SIGN_VAL
	},
};

struct st_lsm6ds3_odr_reg {
	unsigned int hz;
	u8 value;
};

static struct st_lsm6ds3_odr_table {
	u8 addr[4];
	u8 mask[4];
	struct st_lsm6ds3_odr_reg odr_avl[ST_LSM6DS3_ODR_LIST_NUM];
} st_lsm6ds3_odr_table = {
	.addr[ST_INDIO_DEV_ACCEL] = ST_LSM6DS3_ACCEL_ODR_ADDR,
	.mask[ST_INDIO_DEV_ACCEL] = ST_LSM6DS3_ACCEL_ODR_MASK,
	.addr[ST_INDIO_DEV_ACCEL_WK] = ST_LSM6DS3_ACCEL_ODR_ADDR,
	.mask[ST_INDIO_DEV_ACCEL_WK] = ST_LSM6DS3_ACCEL_ODR_MASK,
	.addr[ST_INDIO_DEV_GYRO] = ST_LSM6DS3_GYRO_ODR_ADDR,
	.mask[ST_INDIO_DEV_GYRO] = ST_LSM6DS3_GYRO_ODR_MASK,
	.addr[ST_INDIO_DEV_GYRO_WK] = ST_LSM6DS3_GYRO_ODR_ADDR,
	.mask[ST_INDIO_DEV_GYRO_WK] = ST_LSM6DS3_GYRO_ODR_MASK,
	.odr_avl[0] = { .hz = 26, .value = ST_LSM6DS3_ODR_26HZ_VAL },
	.odr_avl[1] = { .hz = 52, .value = ST_LSM6DS3_ODR_52HZ_VAL },
	.odr_avl[2] = { .hz = 104, .value = ST_LSM6DS3_ODR_104HZ_VAL },
	.odr_avl[3] = { .hz = 208, .value = ST_LSM6DS3_ODR_208HZ_VAL },
	.odr_avl[4] = { .hz = 416, .value = ST_LSM6DS3_ODR_416HZ_VAL },
};

struct st_lsm6ds3_fs_reg {
	unsigned int gain;
	u8 value;
};

static struct st_lsm6ds3_fs_table {
	u8 addr;
	u8 mask;
	struct st_lsm6ds3_fs_reg fs_avl[ST_LSM6DS3_FS_LIST_NUM];
} st_lsm6ds3_fs_table[ST_INDIO_DEV_NUM] = {
	[ST_INDIO_DEV_ACCEL] = {
		.addr = ST_LSM6DS3_ACCEL_FS_ADDR,
		.mask = ST_LSM6DS3_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3_ACCEL_FS_2G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_2G_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3_ACCEL_FS_4G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_4G_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3_ACCEL_FS_8G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_8G_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3_ACCEL_FS_16G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_16G_VAL },
	},
	[ST_INDIO_DEV_ACCEL_WK] = {
		.addr = ST_LSM6DS3_ACCEL_FS_ADDR,
		.mask = ST_LSM6DS3_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3_ACCEL_FS_2G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_2G_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3_ACCEL_FS_4G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_4G_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3_ACCEL_FS_8G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_8G_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3_ACCEL_FS_16G_GAIN,
					.value = ST_LSM6DS3_ACCEL_FS_16G_VAL },
	},
	[ST_INDIO_DEV_GYRO] = {
		.addr = ST_LSM6DS3_GYRO_FS_ADDR,
		.mask = ST_LSM6DS3_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3_GYRO_FS_245_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_245_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3_GYRO_FS_500_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_500_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3_GYRO_FS_1000_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_1000_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3_GYRO_FS_2000_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_2000_VAL },
	},
	[ST_INDIO_DEV_GYRO_WK] = {
		.addr = ST_LSM6DS3_GYRO_FS_ADDR,
		.mask = ST_LSM6DS3_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = ST_LSM6DS3_GYRO_FS_245_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_245_VAL },
		.fs_avl[1] = { .gain = ST_LSM6DS3_GYRO_FS_500_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_500_VAL },
		.fs_avl[2] = { .gain = ST_LSM6DS3_GYRO_FS_1000_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_1000_VAL },
		.fs_avl[3] = { .gain = ST_LSM6DS3_GYRO_FS_2000_GAIN,
					.value = ST_LSM6DS3_GYRO_FS_2000_VAL },
	}
};

const struct iio_event_spec st_lsm6ds3_event_spec[] = {
	{}
};

static const struct iio_chan_spec st_lsm6ds3_accel_ch[] = {
	ST_LSM6DS3_LSM_CHANNELS(IIO_ACCEL, 1, 0, IIO_MOD_X, IIO_LE,
				16, 16, ST_LSM6DS3_ACCEL_OUT_X_L_ADDR, 's'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_ACCEL, 1, 1, IIO_MOD_Y, IIO_LE,
				16, 16, ST_LSM6DS3_ACCEL_OUT_Y_L_ADDR, 's'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_ACCEL, 1, 2, IIO_MOD_Z, IIO_LE,
				16, 16, ST_LSM6DS3_ACCEL_OUT_Z_L_ADDR, 's'),
	ST_LSM6DS3_FLUSH_CHANNEL(IIO_ACCEL),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_lsm6ds3_gyro_ch[] = {
	ST_LSM6DS3_LSM_CHANNELS(IIO_ANGL_VEL, 1, 0, IIO_MOD_X, IIO_LE,
				16, 16, ST_LSM6DS3_GYRO_OUT_X_L_ADDR, 's'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_ANGL_VEL, 1, 1, IIO_MOD_Y, IIO_LE,
				16, 16, ST_LSM6DS3_GYRO_OUT_Y_L_ADDR, 's'),
	ST_LSM6DS3_LSM_CHANNELS(IIO_ANGL_VEL, 1, 2, IIO_MOD_Z, IIO_LE,
				16, 16, ST_LSM6DS3_GYRO_OUT_Z_L_ADDR, 's'),
	ST_LSM6DS3_FLUSH_CHANNEL(IIO_ANGL_VEL),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_lsm6ds3_sign_motion_ch[] = {
	{
		.type = IIO_SIGN_MOTION,
		.channel = 0,
		.modified = 0,
		.event_spec = st_lsm6ds3_event_spec,
		.num_event_specs = 1,
		.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3_step_c_ch[] = {
	{
		.type = IIO_STEP_COUNTER,
		.modified = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.address = ST_LSM6DS3_STEP_COUNTER_OUT_L_ADDR,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3_step_d_ch[] = {
	{
		.type = IIO_STEP_DETECTOR,
		.channel = 0,
		.modified = 0,
		.event_spec = st_lsm6ds3_event_spec,
		.num_event_specs = 1,
		.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static const struct iio_chan_spec st_lsm6ds3_tilt_ch[] = {
	{
		.type = IIO_TILT,
		.channel = 0,
		.modified = 0,
		.event_spec = st_lsm6ds3_event_spec,
		.num_event_specs = 1,
		.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

int st_lsm6ds3_write_data_with_mask(struct lsm6ds3_data *cdata,
				u8 reg_addr, u8 mask, u8 data, bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}
EXPORT_SYMBOL(st_lsm6ds3_write_data_with_mask);

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
static int st_lsm6ds3_enable_sensor_hub(struct lsm6ds3_data *cdata, bool enable)
{
	int err;

	if (enable) {
		if (cdata->sensors_enabled & ST_LSM6DS3_EXT_SENSORS) {
			err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_STARTCONFIG_ADDR,
						ST_LSM6DS3_STARTCONFIG_MASK,
						ST_LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;
		}

		if (cdata->sensors_enabled & ST_LSM6DS3_EXTRA_DEPENDENCY) {
			err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_FUNC_EN_ADDR,
						ST_LSM6DS3_FUNC_EN_MASK,
						ST_LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		}

		if (cdata->sensors_enabled & ST_LSM6DS3_EXT_SENSORS) {
			err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_SENSORHUB_ADDR,
						ST_LSM6DS3_SENSORHUB_MASK,
						ST_LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		}
	} else {
		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_STARTCONFIG_ADDR,
						ST_LSM6DS3_STARTCONFIG_MASK,
						ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		usleep_range(1500, 4000);

		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_SENSORHUB_ADDR,
						ST_LSM6DS3_SENSORHUB_MASK,
						ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_FUNC_EN_ADDR,
						ST_LSM6DS3_FUNC_EN_MASK,
						ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;
	}

	return 0;
}

int st_lsm6ds3_enable_passthrough(struct lsm6ds3_data *cdata, bool enable)
{
	int err;
	u8 reg_value;

	if (enable)
		reg_value = ST_LSM6DS3_EN_BIT;
	else
		reg_value = ST_LSM6DS3_DIS_BIT;

	if (enable) {
		err = st_lsm6ds3_enable_sensor_hub(cdata, false);
		if (err < 0)
			return err;

#ifdef CONFIG_ST_LSM6DS3_ENABLE_INTERNAL_PULLUP
		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_INTER_PULLUP_ADDR,
						ST_LSM6DS3_INTER_PULLUP_MASK,
						ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;
#endif /* CONFIG_ST_LSM6DS3_ENABLE_INTERNAL_PULLUP */
	}

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_PASS_THROUGH_MODE_ADDR,
					ST_LSM6DS3_PASS_THROUGH_MODE_MASK,
					reg_value, enable);
	if (err < 0)
		return err;

	if (!enable) {
#ifdef CONFIG_ST_LSM6DS3_ENABLE_INTERNAL_PULLUP
		err = st_lsm6ds3_write_data_with_mask(cdata,
						ST_LSM6DS3_INTER_PULLUP_ADDR,
						ST_LSM6DS3_INTER_PULLUP_MASK,
						ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;
#endif /* CONFIG_ST_LSM6DS3_ENABLE_INTERNAL_PULLUP */

		err = st_lsm6ds3_enable_sensor_hub(cdata, true);
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_enable_passthrough);
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

static int st_lsm6ds3_set_fifo_enable(struct lsm6ds3_data *cdata, bool status)
{
	int err;
	u8 reg_value;

	if (status)
		reg_value = ST_LSM6DS3_FIFO_ODR_MAX;
	else
		reg_value = ST_LSM6DS3_FIFO_ODR_OFF;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_ODR_ADDR,
					ST_LSM6DS3_FIFO_ODR_MASK,
					reg_value, true);
	if (err < 0)
		return err;

	cdata->last_timestamp = cdata->timestamp = ktime_to_ns(ktime_get_boottime());
	return 0;
}

int st_lsm6ds3_set_fifo_mode(struct lsm6ds3_data *cdata, enum fifo_mode fm)
{
	int err;
	u8 reg_value;
	bool enable_fifo;

	switch (fm) {
	case BYPASS:
		reg_value = ST_LSM6DS3_FIFO_MODE_BYPASS;
		enable_fifo = false;
		break;
	case CONTINUOS:
		reg_value = ST_LSM6DS3_FIFO_MODE_CONTINUOS;
		enable_fifo = true;
		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3_set_fifo_enable(cdata, enable_fifo);
	if (err < 0)
		return err;

	return st_lsm6ds3_write_data_with_mask(cdata, ST_LSM6DS3_FIFO_MODE_ADDR,
				ST_LSM6DS3_FIFO_MODE_MASK, reg_value, true);
}
EXPORT_SYMBOL(st_lsm6ds3_set_fifo_mode);

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
static int st_lsm6ds3_force_accel_odr(struct lsm6ds3_sensor_data *sdata,
							unsigned int odr)
{
	int i;

	for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
		if (st_lsm6ds3_odr_table.odr_avl[i].hz == odr)
			break;
	}
	if (i == ST_LSM6DS3_ODR_LIST_NUM)
		return -EINVAL;

	return st_lsm6ds3_write_data_with_mask(sdata->cdata,
			st_lsm6ds3_odr_table.addr[sdata->sindex],
			st_lsm6ds3_odr_table.mask[sdata->sindex],
			st_lsm6ds3_odr_table.odr_avl[i].value, true);
}
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

int st_lsm6ds3_set_fifo_decimators_and_threshold(struct lsm6ds3_data *cdata)
{
	int err;
	struct iio_dev *indio_dev;
	u16 min_num_pattern, max_num_pattern;
	unsigned int min_odr = 416, max_odr = 0;
	u8 accel_decimator = 0, gyro_decimator = 0;
	u16 num_pattern_accel = 0, num_pattern_gyro = 0;
	struct lsm6ds3_sensor_data *sdata_accel, *sdata_gyro;
	u16 fifo_len, fifo_threshold, fifo_len_accel = 0, fifo_len_gyro = 0;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	int i;
	bool force_accel_odr = false;
	u8 ext0_decimator = 0, ext1_decimator = 0;
	u16 num_pattern_ext1 = 0, fifo_len_ext1 = 0;
	u16 num_pattern_ext0 = 0, fifo_len_ext0 = 0;
	struct lsm6ds3_sensor_data *sdata_ext0 = NULL, *sdata_ext1 = NULL;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	dev_dbg(cdata->dev, "st_lsm6ds3_set_fifo_decimators_and_threshold: sensors_enabled=0x%2x\n",
				cdata->sensors_enabled);

	cdata->sensors_pattern_en = 0;
	indio_dev = cdata->indio_dev[ST_INDIO_DEV_ACCEL];
	sdata_accel = iio_priv(indio_dev);
	if (((1 << ST_INDIO_DEV_ACCEL) | (1 << ST_INDIO_DEV_ACCEL_WK)) & cdata->sensors_enabled) {
		if (min_odr > sdata_accel->c_odr)
			min_odr = sdata_accel->c_odr;

		if (max_odr < sdata_accel->c_odr)
			max_odr = sdata_accel->c_odr;

		if ((1 << ST_INDIO_DEV_ACCEL) & cdata->sensors_enabled) {
			if ((1 << ST_INDIO_DEV_ACCEL_WK) & cdata->sensors_enabled) {
				fifo_len_accel =
					(indio_dev->buffer->length <
						cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]->buffer->length) ?
					indio_dev->buffer->length :
					cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]->buffer->length;
				fifo_len_accel /= IIO_BUFFER_KFIFO_DEFAULT_LEN;
			} else
				fifo_len_accel = (indio_dev->buffer->length /
							IIO_BUFFER_KFIFO_DEFAULT_LEN);
		} else
			fifo_len_accel = (cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]->buffer->length /
						IIO_BUFFER_KFIFO_DEFAULT_LEN);
	}

	indio_dev = cdata->indio_dev[ST_INDIO_DEV_GYRO];
	sdata_gyro = iio_priv(indio_dev);
	if (((1 << ST_INDIO_DEV_GYRO) | (1 << ST_INDIO_DEV_GYRO_WK)) & cdata->sensors_enabled) {
		if (min_odr > sdata_gyro->c_odr)
			min_odr = sdata_gyro->c_odr;

		if (max_odr < sdata_gyro->c_odr)
			max_odr = sdata_gyro->c_odr;

		if ((1 << ST_INDIO_DEV_GYRO) & cdata->sensors_enabled) {
			if ((1 << ST_INDIO_DEV_GYRO_WK) & cdata->sensors_enabled) {
				fifo_len_gyro =
					(indio_dev->buffer->length <
						cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]->buffer->length) ?
					indio_dev->buffer->length :
					cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]->buffer->length;
				fifo_len_gyro /= IIO_BUFFER_KFIFO_DEFAULT_LEN;
			} else
				fifo_len_gyro = (indio_dev->buffer->length /
							IIO_BUFFER_KFIFO_DEFAULT_LEN);
		} else
			fifo_len_gyro = (cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]->buffer->length /
						IIO_BUFFER_KFIFO_DEFAULT_LEN);
	}

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	if (cdata->ext0_available) {
		indio_dev = cdata->indio_dev[ST_INDIO_DEV_EXT0];
		sdata_ext0 = iio_priv(indio_dev);
		if ((1 << sdata_ext0->sindex) & cdata->sensors_enabled) {
			if (min_odr > sdata_ext0->c_odr)
				min_odr = sdata_ext0->c_odr;

			if (max_odr < sdata_ext0->c_odr) {
				force_accel_odr = true;
				max_odr = sdata_ext0->c_odr;
			}

			fifo_len_ext0 = (indio_dev->buffer->length /
						IIO_BUFFER_KFIFO_DEFAULT_LEN);
		}
	}

	if (cdata->ext1_available) {
		indio_dev = cdata->indio_dev[ST_INDIO_DEV_EXT1];
		sdata_ext1 = iio_priv(indio_dev);
		if ((1 << sdata_ext1->sindex) & cdata->sensors_enabled) {
			if (min_odr > sdata_ext1->c_odr)
				min_odr = sdata_ext1->c_odr;

			if (max_odr < sdata_ext1->c_odr) {
				force_accel_odr = true;
				max_odr = sdata_ext1->c_odr;
			}

			fifo_len_ext1 = (indio_dev->buffer->length /
						IIO_BUFFER_KFIFO_DEFAULT_LEN);
		}
	}

	if (force_accel_odr) {
		err = st_lsm6ds3_force_accel_odr(sdata_accel, max_odr);
		if (err < 0)
			return err;
	} else {
		for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
			if (st_lsm6ds3_odr_table.odr_avl[i].hz ==
							sdata_accel->c_odr)
				break;
		}
		if (i == ST_LSM6DS3_ODR_LIST_NUM)
			return -EINVAL;

		if (cdata->sensors_enabled & ((1 << ST_INDIO_DEV_ACCEL) | (1 << ST_INDIO_DEV_ACCEL_WK))) {
			cdata->accel_samples_to_discard = ST_LSM6DS3_ACCEL_STD;

			err = st_lsm6ds3_write_data_with_mask(cdata,
				st_lsm6ds3_odr_table.addr[sdata_accel->sindex],
				st_lsm6ds3_odr_table.mask[sdata_accel->sindex],
				st_lsm6ds3_odr_table.odr_avl[i].value, true);
			if (err < 0)
				return err;
		}
	}
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	if (((1 << ST_INDIO_DEV_ACCEL) | (1 << ST_INDIO_DEV_ACCEL_WK)) & cdata->sensors_enabled) {
		cdata->accel_samples_in_pattern =
						(sdata_accel->c_odr / min_odr);
		num_pattern_accel = MAX(fifo_len_accel /
					cdata->accel_samples_in_pattern, 1);
		cdata->accel_deltatime = (1000000000ULL / sdata_accel->c_odr);
		accel_decimator = max_odr / sdata_accel->c_odr;
		if ((1 << ST_INDIO_DEV_ACCEL) & cdata->sensors_enabled)
			cdata->sensors_pattern_en |= (1 << ST_INDIO_DEV_ACCEL);
		if ((1 << ST_INDIO_DEV_ACCEL_WK) & cdata->sensors_enabled)
			cdata->sensors_pattern_en |= (1 << ST_INDIO_DEV_ACCEL_WK);
		dev_dbg(cdata->dev, "Accel: sensors_enabled=0x%2x, sensors_pattern_en=0x%2x\n",
					cdata->sensors_enabled, cdata->sensors_pattern_en);
	} else
		cdata->accel_samples_in_pattern = 0;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_DECIMATOR_ADDR,
					ST_LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK,
					accel_decimator, true);
	if (err < 0)
		return err;

	if (((1 << ST_INDIO_DEV_GYRO) | (1 << ST_INDIO_DEV_GYRO_WK)) & cdata->sensors_enabled) {
		cdata->gyro_samples_in_pattern = (sdata_gyro->c_odr / min_odr);
		num_pattern_gyro = MAX(fifo_len_gyro /
					cdata->gyro_samples_in_pattern, 1);
		cdata->gyro_deltatime = (1000000000ULL / sdata_gyro->c_odr);
		gyro_decimator = max_odr / sdata_gyro->c_odr;
		if ((1 << ST_INDIO_DEV_GYRO) & cdata->sensors_enabled)
			cdata->sensors_pattern_en |= (1 << ST_INDIO_DEV_GYRO);
		if ((1 << ST_INDIO_DEV_GYRO_WK) & cdata->sensors_enabled)
			cdata->sensors_pattern_en |= (1 << ST_INDIO_DEV_GYRO_WK);
		dev_dbg(cdata->dev, "Gyro: sensors_enabled=0x%2x, sensors_pattern_en=0x%2x\n",
					cdata->sensors_enabled, cdata->sensors_pattern_en);
	} else
		cdata->gyro_samples_in_pattern = 0;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_DECIMATOR_ADDR,
					ST_LSM6DS3_FIFO_GYRO_DECIMATOR_MASK,
					gyro_decimator, true);
	if (err < 0)
		return err;

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	if (cdata->ext0_available) {
		if ((1 << sdata_ext0->sindex) & cdata->sensors_enabled) {
			cdata->ext0_samples_in_pattern =
						(sdata_ext0->c_odr / min_odr);
			num_pattern_ext0 = MAX(fifo_len_ext0 /
					cdata->ext0_samples_in_pattern, 1);
			cdata->ext0_deltatime =
					(1000000000ULL / sdata_ext0->c_odr);
			ext0_decimator = max_odr / sdata_ext0->c_odr;
		} else
			cdata->ext0_samples_in_pattern = 0;

		err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_DECIMATOR2_ADDR,
					ST_LSM6DS3_FIFO_DS3_DECIMATOR_MASK,
					ext0_decimator, true);
		if (err < 0)
			return err;
	}

	if (cdata->ext1_available) {
		if ((1 << sdata_ext1->sindex) & cdata->sensors_enabled) {
			cdata->ext1_samples_in_pattern =
						(sdata_ext1->c_odr / min_odr);
			num_pattern_ext1 = MAX(fifo_len_ext1 /
					cdata->ext1_samples_in_pattern, 1);
			cdata->ext1_deltatime =
					(1000000000ULL / sdata_ext1->c_odr);
			ext1_decimator = max_odr / sdata_ext1->c_odr;
		} else
			cdata->ext1_samples_in_pattern = 0;

		err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_DECIMATOR2_ADDR,
					ST_LSM6DS3_FIFO_DS4_DECIMATOR_MASK,
					ext1_decimator, true);
		if (err < 0)
			return err;
	}
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	min_num_pattern = MIN_BNZ(MIN_BNZ(MIN_BNZ(num_pattern_gyro,
		num_pattern_accel), num_pattern_ext0), num_pattern_ext1);
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	min_num_pattern = MIN_BNZ(num_pattern_gyro, num_pattern_accel);
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	if ((cdata->accel_samples_in_pattern +
				cdata->gyro_samples_in_pattern +
					cdata->ext0_samples_in_pattern +
					cdata->ext1_samples_in_pattern) > 0) {
		cdata->byte_in_pattern =
					(cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern +
					cdata->ext0_samples_in_pattern +
					cdata->ext1_samples_in_pattern) *
					ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
		max_num_pattern = ST_LSM6DS3_MAX_FIFO_SIZE / cdata->byte_in_pattern;

		if (min_num_pattern > max_num_pattern)
			min_num_pattern = max_num_pattern;
	} else
		cdata->byte_in_pattern = 0;

	fifo_len = min_num_pattern * cdata->byte_in_pattern;
	dev_dbg(cdata->dev, "st_lsm6ds3_set_fifo_decimators_and_threshold: %d-%d-%d-%d-%d-%d\n",
					cdata->accel_samples_in_pattern,
					cdata->gyro_samples_in_pattern,
					cdata->ext0_samples_in_pattern,
					cdata->ext1_samples_in_pattern,
					min_num_pattern, fifo_len);
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	if ((cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern) > 0) {
		cdata->byte_in_pattern =
					(cdata->accel_samples_in_pattern +
					cdata->gyro_samples_in_pattern) *
					ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
		max_num_pattern = ST_LSM6DS3_MAX_FIFO_SIZE / cdata->byte_in_pattern;

		if (min_num_pattern > max_num_pattern)
			min_num_pattern = max_num_pattern;
	} else
		cdata->byte_in_pattern = 0;

	fifo_len = min_num_pattern * cdata->byte_in_pattern;
	dev_dbg(cdata->dev, "st_lsm6ds3_set_fifo_decimators_and_threshold: %d-%d-%d-%d\n",
					cdata->accel_samples_in_pattern,
					cdata->gyro_samples_in_pattern,
					min_num_pattern, fifo_len);

#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	if (fifo_len > 0) {
		fifo_threshold = fifo_len / 2;

		err = cdata->tf->write(cdata, ST_LSM6DS3_FIFO_THR_L_ADDR,
					1, (u8 *)&fifo_threshold, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_FIFO_THR_H_ADDR,
					ST_LSM6DS3_FIFO_THR_H_MASK,
					*(((u8 *)&fifo_threshold) + 1), true);
		if (err < 0)
			return err;

		cdata->fifo_threshold = fifo_len;
	}
	cdata->fifo_data = NULL;

	return fifo_len;
}
EXPORT_SYMBOL(st_lsm6ds3_set_fifo_decimators_and_threshold);

int st_lsm6ds3_reconfigure_fifo(struct lsm6ds3_data *cdata,
						bool disable_irq_and_flush)
{
	int err, fifo_len;

	if (disable_irq_and_flush) {
		disable_irq(cdata->irq);
		st_lsm6ds3_flush_works();
	}

	mutex_lock(&cdata->fifo_lock);

	st_lsm6ds3_read_fifo(cdata, READ_FIFO_IN_COF_FIFO);

	err = st_lsm6ds3_set_fifo_mode(cdata, BYPASS);
	if (err < 0)
		goto reconfigure_fifo_irq_restore;

	fifo_len = st_lsm6ds3_set_fifo_decimators_and_threshold(cdata);
	if (fifo_len < 0) {
		err = fifo_len;
		goto reconfigure_fifo_irq_restore;
	}

	dev_dbg(cdata->dev, "st_lsm6ds3_reconfigure_fifo: fifo_len=%d\n", fifo_len);
	if (fifo_len > 0) {
		err = st_lsm6ds3_set_fifo_mode(cdata, CONTINUOS);
		if (err < 0)
			goto reconfigure_fifo_irq_restore;
	}

reconfigure_fifo_irq_restore:
	mutex_unlock(&cdata->fifo_lock);

	if (disable_irq_and_flush)
		enable_irq(cdata->irq);

	return err;
}
EXPORT_SYMBOL(st_lsm6ds3_reconfigure_fifo);

static int st_lsm6ds3_reset_steps(struct lsm6ds3_data *cdata)
{
	int err;
	u8 reg_value = 0x00;

	err = cdata->tf->read(cdata,
			ST_LSM6DS3_STEP_COUNTER_RES_ADDR, 1, &reg_value, true);
	if (err < 0)
		return err;

	if (reg_value & ST_LSM6DS3_FUNC_EN_MASK)
		reg_value = ST_LSM6DS3_STEP_COUNTER_RES_FUNC_EN;
	else
		reg_value = ST_LSM6DS3_DIS_BIT;

	err = st_lsm6ds3_write_data_with_mask(cdata,
				ST_LSM6DS3_STEP_COUNTER_RES_ADDR,
				ST_LSM6DS3_STEP_COUNTER_RES_MASK,
				ST_LSM6DS3_STEP_COUNTER_RES_ALL_EN, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_write_data_with_mask(cdata,
				ST_LSM6DS3_STEP_COUNTER_RES_ADDR,
				ST_LSM6DS3_STEP_COUNTER_RES_MASK,
				reg_value, true);
	if (err < 0)
		return err;

	cdata->reset_steps = true;

	return 0;
}

int st_lsm6ds3_set_drdy_irq(struct lsm6ds3_sensor_data *sdata, bool state)
{
	u8 reg_addr, mask, value;

	if (state)
		value = ST_LSM6DS3_EN_BIT;
	else
		value = ST_LSM6DS3_DIS_BIT;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		if ((sdata->cdata->sensors_enabled &
				((1 << ST_INDIO_DEV_ACCEL_WK) |
				(1 << ST_INDIO_DEV_GYRO) |
				(1 << ST_INDIO_DEV_GYRO_WK))) ||
					(sdata->cdata->sensors_enabled &
						ST_LSM6DS3_EXT_SENSORS))
			return 0;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		if (sdata->cdata->sensors_enabled &
				((1 << ST_INDIO_DEV_ACCEL_WK) |
				(1 << ST_INDIO_DEV_GYRO) |
				(1 << ST_INDIO_DEV_GYRO_WK)))
			return 0;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case ST_INDIO_DEV_ACCEL_WK:
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		if ((sdata->cdata->sensors_enabled &
				((1 << ST_INDIO_DEV_ACCEL) |
				(1 << ST_INDIO_DEV_GYRO) |
				(1 << ST_INDIO_DEV_GYRO_WK))) ||
					(sdata->cdata->sensors_enabled &
						ST_LSM6DS3_EXT_SENSORS))
			return 0;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		if (sdata->cdata->sensors_enabled &
				((1 << ST_INDIO_DEV_ACCEL) |
				(1 << ST_INDIO_DEV_GYRO) |
				(1 << ST_INDIO_DEV_GYRO_WK)))
			return 0;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case ST_INDIO_DEV_GYRO:
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		if ((sdata->cdata->sensors_enabled &
				((1 << ST_INDIO_DEV_ACCEL) |
				(1 << ST_INDIO_DEV_ACCEL_WK) |
				(1 << ST_INDIO_DEV_GYRO_WK))) ||
					(sdata->cdata->sensors_enabled &
						ST_LSM6DS3_EXT_SENSORS))
			return 0;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		if (sdata->cdata->sensors_enabled &
				((1 << ST_INDIO_DEV_ACCEL) |
				(1 << ST_INDIO_DEV_ACCEL_WK) |
				(1 << ST_INDIO_DEV_GYRO_WK)))
			return 0;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case ST_INDIO_DEV_GYRO_WK:
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		if ((sdata->cdata->sensors_enabled &
				((1 << ST_INDIO_DEV_ACCEL) |
				(1 << ST_INDIO_DEV_ACCEL_WK) |
				(1 << ST_INDIO_DEV_GYRO))) ||
					(sdata->cdata->sensors_enabled &
						ST_LSM6DS3_EXT_SENSORS))
			return 0;
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		if (sdata->cdata->sensors_enabled &
				((1 << ST_INDIO_DEV_ACCEL) |
				(1 << ST_INDIO_DEV_ACCEL_WK) |
				(1 << ST_INDIO_DEV_GYRO)))
			return 0;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case ST_INDIO_DEV_SIGN_MOTION:
		if (sdata->cdata->sensors_enabled &
					(1 << ST_INDIO_DEV_STEP_DETECTOR))
			return 0;

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case ST_INDIO_DEV_STEP_COUNTER:
		reg_addr = ST_LSM6DS3_INT2_ADDR;
		mask = ST_LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK;
		break;
	case ST_INDIO_DEV_STEP_DETECTOR:
		if (sdata->cdata->sensors_enabled &
					(1 << ST_INDIO_DEV_SIGN_MOTION))
			return 0;

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case ST_INDIO_DEV_TILT:
		reg_addr = ST_LSM6DS3_MD1_ADDR;
		mask = ST_LSM6DS3_TILT_DRDY_IRQ_MASK;
		break;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	case ST_INDIO_DEV_EXT0:
		if ((sdata->cdata->sensors_enabled &
						((1 << ST_INDIO_DEV_ACCEL) |
						(1 << ST_INDIO_DEV_ACCEL_WK))) ||
					(sdata->cdata->sensors_enabled &
						((1 << ST_INDIO_DEV_GYRO) |
						(1 << ST_INDIO_DEV_GYRO_WK))) ||
					(sdata->cdata->sensors_enabled &
						(1 << ST_INDIO_DEV_EXT1)))
			return 0;

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
	case ST_INDIO_DEV_EXT1:
		if ((sdata->cdata->sensors_enabled &
						((1 << ST_INDIO_DEV_ACCEL) |
						(1 << ST_INDIO_DEV_ACCEL_WK))) ||
					(sdata->cdata->sensors_enabled &
						((1 << ST_INDIO_DEV_GYRO) |
						(1 << ST_INDIO_DEV_GYRO_WK))) ||
					(sdata->cdata->sensors_enabled &
						(1 << ST_INDIO_DEV_EXT0)))
			return 0;

		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	default:
		return -EINVAL;
	}

	return st_lsm6ds3_write_data_with_mask(sdata->cdata,
						reg_addr, mask, value, true);
}
EXPORT_SYMBOL(st_lsm6ds3_set_drdy_irq);

int st_lsm6ds3_set_axis_enable(struct lsm6ds3_sensor_data *sdata, u8 value)
{
	int err;
	u8 reg_addr;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		if (sdata->cdata->axis_enabled & (1 << ST_INDIO_DEV_ACCEL_WK))
			return 0;
		reg_addr = ST_LSM6DS3_ACCEL_AXIS_EN_ADDR;
		break;
	case ST_INDIO_DEV_ACCEL_WK:
		if (sdata->cdata->axis_enabled & (1 << ST_INDIO_DEV_ACCEL))
			return 0;
		reg_addr = ST_LSM6DS3_ACCEL_AXIS_EN_ADDR;
		break;
	case ST_INDIO_DEV_GYRO:
		if (sdata->cdata->axis_enabled & (1 << ST_INDIO_DEV_GYRO_WK))
			return 0;
		reg_addr = ST_LSM6DS3_GYRO_AXIS_EN_ADDR;
		break;
	case ST_INDIO_DEV_GYRO_WK:
		if (sdata->cdata->axis_enabled & (1 << ST_INDIO_DEV_GYRO))
			return 0;
		reg_addr = ST_LSM6DS3_GYRO_AXIS_EN_ADDR;
		break;
	default:
		return 0;
	}

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				reg_addr, ST_LSM6DS3_AXIS_EN_MASK, value, true);
	if (err < 0)
		return err;

	if (!(value & ST_LSM6DS3_AXIS_EN_MASK))
		sdata->cdata->axis_enabled &= ~(1 << sdata->sindex);
	else
		sdata->cdata->axis_enabled |= (1 << sdata->sindex);

	return 0;

}
EXPORT_SYMBOL(st_lsm6ds3_set_axis_enable);

int st_lsm6ds3_enable_accel_dependency(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err;

	if (!((sdata->cdata->sensors_enabled &
			ST_LSM6DS3_ACCEL_DEPENDENCY) & ~(1 << sdata->sindex))) {
		if (enable) {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[ST_INDIO_DEV_ACCEL],
				st_lsm6ds3_odr_table.mask[ST_INDIO_DEV_ACCEL],
				st_lsm6ds3_odr_table.odr_avl[0].value, true);
			if (err < 0)
				return err;
		} else {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[ST_INDIO_DEV_ACCEL],
				st_lsm6ds3_odr_table.mask[ST_INDIO_DEV_ACCEL],
				ST_LSM6DS3_ODR_POWER_OFF_VAL, true);
			if (err < 0)
				return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_enable_accel_dependency);

static int st_lsm6ds3_set_extra_dependency(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err;

	if (!((sdata->cdata->sensors_enabled &
			ST_LSM6DS3_EXTRA_DEPENDENCY) & ~(1 << sdata->sindex))) {
		if (enable) {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_FUNC_EN_ADDR,
						ST_LSM6DS3_FUNC_EN_MASK,
						ST_LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_FUNC_EN_ADDR,
						ST_LSM6DS3_FUNC_EN_MASK,
						ST_LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;
		}
	}

	return st_lsm6ds3_enable_accel_dependency(sdata, enable);
}

static int st_lsm6ds3_enable_pedometer(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	u8 value = ST_LSM6DS3_DIS_BIT;

	if ((sdata->cdata->sensors_enabled & ~(1 << sdata->sindex)) &
						ST_LSM6DS3_PEDOMETER_DEPENDENCY)
		return 0;

	if (enable)
		value = ST_LSM6DS3_EN_BIT;

	return st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_PEDOMETER_EN_ADDR,
						ST_LSM6DS3_PEDOMETER_EN_MASK,
						value, true);

}

void update_current_odr(struct lsm6ds3_sensor_data *sdata,
							unsigned int odr)
{
		switch (sdata->sindex) {
		case ST_INDIO_DEV_ACCEL:
			{
				struct lsm6ds3_sensor_data *sdata_accel_wk =
					iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]);
				sdata_accel_wk->c_odr = sdata->c_odr = odr;
			}
			break;
		case ST_INDIO_DEV_ACCEL_WK:
			{
				struct lsm6ds3_sensor_data *sdata_accel =
					iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
				sdata_accel->c_odr = sdata->c_odr = odr;
			}
			break;
		case ST_INDIO_DEV_GYRO:
			{
				struct lsm6ds3_sensor_data *sdata_gyro_wk =
					iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]);
				sdata_gyro_wk->c_odr = sdata->c_odr = odr;
			}
			break;
		case ST_INDIO_DEV_GYRO_WK:
			{
				struct lsm6ds3_sensor_data *sdata_gyro =
					iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_GYRO]);
				sdata_gyro->c_odr = sdata->c_odr = odr;
			}
			break;
		}
}

static int st_lsm6ds3_set_odr(struct lsm6ds3_sensor_data *sdata,
							unsigned int odr,
							bool need_set_register)
{
	int err, i;

	for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
		if (st_lsm6ds3_odr_table.odr_avl[i].hz == odr)
			break;
	}
	if (i == ST_LSM6DS3_ODR_LIST_NUM)
		return -EINVAL;

	if (need_set_register) {
		disable_irq(sdata->cdata->irq);
		st_lsm6ds3_flush_works();

		if ((sdata->sindex == ST_INDIO_DEV_ACCEL) ||
				(sdata->sindex == ST_INDIO_DEV_ACCEL_WK))
			sdata->cdata->accel_samples_to_discard =
							ST_LSM6DS3_ACCEL_STD;

		sdata->cdata->gyro_samples_to_discard = ST_LSM6DS3_GYRO_STD;

		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				st_lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);
			return err;
		}

		update_current_odr(sdata, st_lsm6ds3_odr_table.odr_avl[i].hz);

		st_lsm6ds3_reconfigure_fifo(sdata->cdata, false);
		enable_irq(sdata->cdata->irq);
	}
	sdata->odr = st_lsm6ds3_odr_table.odr_avl[i].hz;

	return 0;
}

static int st_lsm6ds3_enable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err, i;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		if ((sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL_WK)) &&
				((sdata->odr <= sdata->c_odr) || (sdata->c_odr != 0)))
			goto OUT;
		else
			goto SET_ODR;

	case ST_INDIO_DEV_ACCEL_WK:
		stay_wake = true;
		if ((sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL)) &&
				((sdata->odr <= sdata->c_odr) || (sdata->c_odr != 0)))
			goto OUT;
		else
			goto SET_ODR;

	case ST_INDIO_DEV_GYRO:
		if ((sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO_WK)) &&
				((sdata->odr <= sdata->c_odr) || (sdata->c_odr != 0)))
			goto OUT;
		else
			goto SET_ODR;

	case ST_INDIO_DEV_GYRO_WK:
		stay_wake = true;
		if ((sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO)) &&
				((sdata->odr <= sdata->c_odr) || (sdata->c_odr != 0)))
			goto OUT;
		else
			goto SET_ODR;

SET_ODR:
		for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
			if (st_lsm6ds3_odr_table.odr_avl[i].hz == sdata->odr)
				break;
		}
		if (i == ST_LSM6DS3_ODR_LIST_NUM)
			return -EINVAL;

		if ((sdata->sindex == ST_INDIO_DEV_ACCEL) ||
				(sdata->sindex == ST_INDIO_DEV_ACCEL_WK)) {
			sdata->cdata->accel_samples_to_discard =
							ST_LSM6DS3_ACCEL_STD;
		}

		sdata->cdata->gyro_samples_to_discard = ST_LSM6DS3_GYRO_STD;

		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				st_lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0)
			return err;

		update_current_odr(sdata, st_lsm6ds3_odr_table.odr_avl[i].hz);
		goto OUT;

	case ST_INDIO_DEV_SIGN_MOTION:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_SIGN_MOTION_EN_ADDR,
					ST_LSM6DS3_SIGN_MOTION_EN_MASK,
					ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		if ((sdata->cdata->sensors_enabled & ~(1 << sdata->sindex)) &
					ST_LSM6DS3_PEDOMETER_DEPENDENCY) {
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_PEDOMETER_EN_ADDR,
						ST_LSM6DS3_PEDOMETER_EN_MASK,
						ST_LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;

			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
						ST_LSM6DS3_PEDOMETER_EN_ADDR,
						ST_LSM6DS3_PEDOMETER_EN_MASK,
						ST_LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = st_lsm6ds3_enable_pedometer(sdata, true);
			if (err < 0)
				return err;
		}

		break;
	case ST_INDIO_DEV_STEP_COUNTER:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_TIMER_EN_ADDR,
					ST_LSM6DS3_TIMER_EN_MASK,
					ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

	case ST_INDIO_DEV_STEP_DETECTOR:
		err = st_lsm6ds3_enable_pedometer(sdata, true);
		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_TILT:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_TILT_EN_ADDR,
					ST_LSM6DS3_TILT_EN_MASK,
					ST_LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3_set_extra_dependency(sdata, true);
	if (err < 0)
		return err;

OUT:
	sdata->cdata->sensors_enabled |= (1 << sdata->sindex);

	return 0;
}

static int st_lsm6ds3_disable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err, i;
	u8 odr = 0;
	struct lsm6ds3_sensor_data *sdata_bak;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL_WK)) {
			sdata_bak = iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]);
			odr = sdata_bak->odr;
		}
		goto SET_ODR;

	case ST_INDIO_DEV_ACCEL_WK:
		if (!(sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO_WK)))
			stay_wake = false;
		if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL)) {
			sdata_bak = iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
			odr = sdata_bak->odr;
		}
		goto SET_ODR;

	case ST_INDIO_DEV_GYRO:
		if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO_WK)) {
			sdata_bak = iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]);
			odr = sdata_bak->odr;
		}
		goto SET_ODR;
	case ST_INDIO_DEV_GYRO_WK:
		if (!(sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL_WK)))
			stay_wake = false;
		if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO)) {
			sdata_bak = iio_priv(sdata->cdata->indio_dev[ST_INDIO_DEV_GYRO]);
			odr = sdata_bak->odr;
		}
		goto SET_ODR;

SET_ODR:
		if (odr) {
			for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
				if (st_lsm6ds3_odr_table.odr_avl[i].hz == odr)
					break;
			}
			if (i == ST_LSM6DS3_ODR_LIST_NUM)
				return -EINVAL;

			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				st_lsm6ds3_odr_table.odr_avl[i].value, true);
			if (err >= 0)
				update_current_odr(sdata, odr);
		} else if (((sdata->sindex == ST_INDIO_DEV_ACCEL) ||
				(sdata->sindex == ST_INDIO_DEV_ACCEL_WK)) &&
					(sdata->cdata->sensors_enabled &
						ST_LSM6DS3_EXTRA_DEPENDENCY)) {
				err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					st_lsm6ds3_odr_table.addr[sdata->sindex],
					st_lsm6ds3_odr_table.mask[sdata->sindex],
					st_lsm6ds3_odr_table.odr_avl[0].value, true);
		} else
			err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				st_lsm6ds3_odr_table.addr[sdata->sindex],
				st_lsm6ds3_odr_table.mask[sdata->sindex],
				ST_LSM6DS3_ODR_POWER_OFF_VAL, true);

		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_SIGN_MOTION:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_SIGN_MOTION_EN_ADDR,
					ST_LSM6DS3_SIGN_MOTION_EN_MASK,
					ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_STEP_COUNTER:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_TIMER_EN_ADDR,
					ST_LSM6DS3_TIMER_EN_MASK,
					ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		err = st_lsm6ds3_reset_steps(sdata->cdata);
		if (err < 0)
			return err;

	case ST_INDIO_DEV_STEP_DETECTOR:
		err = st_lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		break;
	case ST_INDIO_DEV_TILT:
		err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_TILT_EN_ADDR,
					ST_LSM6DS3_TILT_EN_MASK,
					ST_LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3_set_extra_dependency(sdata, false);
	if (err < 0)
		return err;

	sdata->cdata->sensors_enabled &= ~(1 << sdata->sindex);

	return 0;
}

int st_lsm6ds3_set_enable(struct lsm6ds3_sensor_data *sdata, bool enable)
{
	if (enable)
		return st_lsm6ds3_enable_sensors(sdata);
	else
		return st_lsm6ds3_disable_sensors(sdata);
}
EXPORT_SYMBOL(st_lsm6ds3_set_enable);

static int st_lsm6ds3_set_fs(struct lsm6ds3_sensor_data *sdata,
							unsigned int gain)
{
	int err, i;

	for (i = 0; i < ST_LSM6DS3_FS_LIST_NUM; i++) {
		if (st_lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}
	if (i == ST_LSM6DS3_FS_LIST_NUM)
		return -EINVAL;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
		st_lsm6ds3_fs_table[sdata->sindex].addr,
		st_lsm6ds3_fs_table[sdata->sindex].mask,
		st_lsm6ds3_fs_table[sdata->sindex].fs_avl[i].value, true);
	if (err < 0)
		return err;

	sdata->c_gain[0] = gain;

	return 0;
}

static int st_lsm6ds3_read_interrupt_config(struct iio_dev *indio_dev,
				u64 event_code)
{
	u8 int_stat, reg_addr, mask;
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	switch (sdata->sindex) {
	case ST_INDIO_DEV_SIGN_MOTION:
	case ST_INDIO_DEV_STEP_DETECTOR:
		reg_addr = ST_LSM6DS3_INT1_ADDR;
		mask = ST_LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case ST_INDIO_DEV_TILT:
		reg_addr = ST_LSM6DS3_MD1_ADDR;
		mask = ST_LSM6DS3_TILT_DRDY_IRQ_MASK;
		break;
	default:
		return -EINVAL;
	}

	err = sdata->cdata->tf->read(sdata->cdata,
					reg_addr, 1, &int_stat, true);
	if (err < 0) {
		dev_err(sdata->cdata->dev, "failed to read SM_THS register.\n");
		return err;
	}

	return (int_stat & mask) ? true : false;
}

static int st_lsm6ds3_write_interrupt_config(struct iio_dev *indio_dev,
				u64 event_code,
				int state)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = st_lsm6ds3_set_drdy_irq(sdata, state);
	return err;
}

static int st_lsm6ds3_read_thresh(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info, int *val, int *val2)
{
	u8 sm_ths;
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3_read_thresh: index=%d, type=%d, dir=%d\n",
				sdata->sindex, type, dir);
	switch (sdata->sindex) {
	case ST_INDIO_DEV_SIGN_MOTION:
		if (type == IIO_EV_TYPE_THRESH) {
			err = sdata->cdata->tf->read(sdata->cdata,
					ST_LSM6DS3_INT2_ON_INT1_ADDR, 1, &sm_ths, true);
			if (err < 0) {
				dev_err(sdata->cdata->dev, "failed to read SM_THS register.\n");
				return err;
			}
			*val = sm_ths;
			return IIO_VAL_INT;
		}
	}
	return -EINVAL;
}

static int st_lsm6ds3_write_thresh(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info, int val, int val2)
{
	u8 sm_ths;
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3_write_thresh: index=%d, type=%d, dir=%d, val=%d, val2=%d\n",
				sdata->sindex, type, dir, val, val2);

	switch (sdata->sindex) {
	case ST_INDIO_DEV_SIGN_MOTION:
		if (type == IIO_EV_TYPE_THRESH) {
			if (val > (1 << 8))
				return -EINVAL;
			sm_ths = val;
			err = sdata->cdata->tf->write(sdata->cdata,
						ST_LSM6DS3_INT2_ON_INT1_ADDR, 1, &sm_ths, true);
			if (err < 0) {
				dev_err(sdata->cdata->dev, "failed to write SM_THS register.\n");
				return err;
			}
			return IIO_VAL_INT;
		}
	}
	return -EINVAL;
}

static int st_lsm6ds3_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int err;
	u8 outdata[ST_LSM6DS3_BYTE_FOR_CHANNEL];
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = st_lsm6ds3_set_enable(sdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		if ((sdata->sindex == ST_INDIO_DEV_ACCEL) ||
				(sdata->sindex == ST_INDIO_DEV_ACCEL_WK))
			msleep(40);

		if ((sdata->sindex == ST_INDIO_DEV_GYRO) ||
				(sdata->sindex == ST_INDIO_DEV_GYRO_WK))
			msleep(120);

		err = sdata->cdata->tf->read(sdata->cdata, ch->address,
				ST_LSM6DS3_BYTE_FOR_CHANNEL, outdata, true);
		if (err < 0) {
			mutex_unlock(&indio_dev->mlock);
			return err;
		}

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		err = st_lsm6ds3_set_enable(sdata, false);

		mutex_unlock(&indio_dev->mlock);

		return IIO_VAL_INT;
#ifdef CONFIG_ST_LSM6DS3_CAL_SUPPORT
	case IIO_CHAN_INFO_OFFSET:
		if ((sdata->sindex == ST_INDIO_DEV_ACCEL) || (sdata->sindex == ST_INDIO_DEV_ACCEL_WK)) {
			*val = accel_cal_data[ch->scan_index];
			return IIO_VAL_INT;
		}
		if ((sdata->sindex == ST_INDIO_DEV_GYRO) || (sdata->sindex == ST_INDIO_DEV_GYRO_WK)) {
			*val = gyro_cal_data[ch->scan_index];
			return IIO_VAL_INT;
		}
		break;
#endif
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sdata->c_gain[0];
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int st_lsm6ds3_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			return -EBUSY;
		}

		err = st_lsm6ds3_set_fs(sdata, val2);
		mutex_unlock(&indio_dev->mlock);
		break;
#ifdef CONFIG_ST_LSM6DS3_CAL_SUPPORT
	case IIO_CHAN_INFO_OFFSET:
		err = 0;
		if ((sdata->sindex == ST_INDIO_DEV_ACCEL) || (sdata->sindex == ST_INDIO_DEV_ACCEL_WK))
			accel_cal_data[chan->scan_index] = val;
		else if ((sdata->sindex == ST_INDIO_DEV_GYRO) || (sdata->sindex == ST_INDIO_DEV_GYRO_WK))
			gyro_cal_data[chan->scan_index] = val;
		else
			err = -EINVAL;
		break;
#endif
	default:
		return -EINVAL;
	}

	return err;
}

static int st_lsm6ds3_init_sensor(struct lsm6ds3_data *cdata)
{
	int err, i;
	u8 default_reg_value = 0;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->tb.buf_lock);

	cdata->sensors_enabled = 0;
	cdata->reset_steps = false;
	cdata->sign_motion_event_ready = false;
	cdata->system_state = SF_NORMAL;

	err = st_lsm6ds3_write_data_with_mask(cdata, ST_LSM6DS3_RESET_ADDR,
				ST_LSM6DS3_RESET_MASK, ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		sdata = iio_priv(cdata->indio_dev[i]);

		err = st_lsm6ds3_set_enable(sdata, false);
		if (err < 0)
			return err;

		err = st_lsm6ds3_set_drdy_irq(sdata, false);
		if (err < 0)
			return err;

		switch (sdata->sindex) {
		case ST_INDIO_DEV_ACCEL:
		case ST_INDIO_DEV_ACCEL_WK:
		case ST_INDIO_DEV_GYRO:
		case ST_INDIO_DEV_GYRO_WK:
			sdata->num_data_channels =
					ARRAY_SIZE(st_lsm6ds3_accel_ch) - 1;

			err = st_lsm6ds3_set_fs(sdata, sdata->c_gain[0]);
			if (err < 0)
				return err;

			break;
		case ST_INDIO_DEV_STEP_COUNTER:
			sdata->num_data_channels =
					ARRAY_SIZE(st_lsm6ds3_step_c_ch) - 1;
			break;
		default:
			break;
		}
	}

	cdata->gyro_selftest_status = 0;
	cdata->accel_selftest_status = 0;

	err = st_lsm6ds3_write_data_with_mask(cdata, ST_LSM6DS3_LIR_ADDR,
				ST_LSM6DS3_LIR_MASK, ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_write_data_with_mask(cdata, ST_LSM6DS3_BDU_ADDR,
				ST_LSM6DS3_BDU_MASK, ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_set_fifo_enable(sdata->cdata, false);
	if (err < 0)
		return err;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_ROUNDING_ADDR,
					ST_LSM6DS3_ROUNDING_MASK,
					ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_write_data_with_mask(cdata,
					ST_LSM6DS3_INT2_ON_INT1_ADDR,
					ST_LSM6DS3_INT2_ON_INT1_MASK,
					ST_LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	mutex_lock(&cdata->bank_registers_lock);

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_STEP_COUNTER_PEDO_THS_ADDR,
					ST_LSM6DS3_STEP_COUNTER_THS_MIN_MASK,
					ST_LSM6DS3_STEP_COUNTER_THS_MIN_DEF_VAL, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_STEP_COUNTER_PEDO_DEB_ADDR,
					ST_LSM6DS3_STEP_COUNTER_DEB_STEP_MASK,
					ST_LSM6DS3_STEP_COUNTER_DEB_STEP_DEF_VAL, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
					ST_LSM6DS3_STEP_COUNTER_DURATION_ADDR,
					1, &default_reg_value, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_init_sensor_mutex_unlock;

	mutex_unlock(&cdata->bank_registers_lock);

	sdata->c_odr = 0;

	return 0;

st_lsm6ds3_init_sensor_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);
	return err;
}

static int st_lsm6ds3_set_selftest(struct lsm6ds3_sensor_data *sdata, int index)
{
	int err;
	u8 mode, mask;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		mask = ST_LSM6DS3_SELFTEST_ACCEL_MASK;
		mode = st_lsm6ds3_selftest_table[index].accel_value;
		break;
	case ST_INDIO_DEV_GYRO:
		mask = ST_LSM6DS3_SELFTEST_GYRO_MASK;
		mode = st_lsm6ds3_selftest_table[index].gyro_value;
		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
				ST_LSM6DS3_SELFTEST_ADDR, mask, mode, true);
	if (err < 0)
		return err;

	return 0;
}

static ssize_t st_lsm6ds3_sysfs_set_max_delivery_rate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
#define	MAX_DELIVERY_RATE_MS	7000
	u8 duration;
	int err, err2;
	unsigned int max_delivery_rate;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = kstrtouint(buf, 10, &max_delivery_rate);
	if (err < 0)
		return -EINVAL;

	if (max_delivery_rate == sdata->c_odr)
		return size;

	if (sdata->sindex == ST_INDIO_DEV_STEP_COUNTER)
		if (max_delivery_rate > MAX_DELIVERY_RATE_MS)
			max_delivery_rate = MAX_DELIVERY_RATE_MS;

	duration = max_delivery_rate / ST_LSM6DS3_MIN_DURATION_MS;

	mutex_lock(&sdata->cdata->bank_registers_lock);

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_sysfs_set_max_delivery_rate_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
					ST_LSM6DS3_STEP_COUNTER_DURATION_ADDR,
					1, &duration, false);
	if (err < 0)
		goto st_lsm6ds3_sysfs_set_max_delivery_rate_restore_bank;

	err = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto st_lsm6ds3_sysfs_set_max_delivery_rate_restore_bank;

	mutex_unlock(&sdata->cdata->bank_registers_lock);

	sdata->c_odr = max_delivery_rate;

	return size;

st_lsm6ds3_sysfs_set_max_delivery_rate_restore_bank:
	do {
		err2 = st_lsm6ds3_write_data_with_mask(sdata->cdata,
					ST_LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					ST_LSM6DS3_FUNC_CFG_REG2_MASK,
					ST_LSM6DS3_DIS_BIT, false);

		msleep(500);
	} while (err2 < 0);

st_lsm6ds3_sysfs_set_max_delivery_rate_mutex_unlock:
	mutex_unlock(&sdata->cdata->bank_registers_lock);
	return err;
}

static ssize_t st_lsm6ds3_sysfs_get_max_delivery_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->c_odr);
}

static ssize_t st_lsm6ds3_sysfs_reset_counter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	err = st_lsm6ds3_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	return size;
}

static ssize_t st_lsm6ds3_sysfs_get_sampling_frequency(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", sdata->odr);
}

static ssize_t st_lsm6ds3_sysfs_set_sampling_frequency(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int odr;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);
	bool need_set_register = false;

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	if (sdata->odr != odr) {
		switch (sdata->sindex) {
		case ST_INDIO_DEV_ACCEL:
			if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL)) {
				if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL_WK)) {
					if (odr > sdata->c_odr)
						need_set_register = true;
				} else
					need_set_register = true;
			}
			mutex_lock(&indio_dev->mlock);
			err = st_lsm6ds3_set_odr(sdata, odr, need_set_register);
			mutex_unlock(&indio_dev->mlock);
			break;

		case ST_INDIO_DEV_ACCEL_WK:
			if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL_WK)) {
				if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_ACCEL)) {
					if (odr > sdata->c_odr)
						need_set_register = true;
				} else
					need_set_register = true;
			}
			mutex_lock(&indio_dev->mlock);
			err = st_lsm6ds3_set_odr(sdata, odr, need_set_register);
			mutex_unlock(&indio_dev->mlock);
			break;

		case ST_INDIO_DEV_GYRO:
			if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO)) {
				if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO_WK)) {
					if (odr > sdata->c_odr)
						need_set_register = true;
				} else
					need_set_register = true;
			}
			mutex_lock(&indio_dev->mlock);
			err = st_lsm6ds3_set_odr(sdata, odr, need_set_register);
			mutex_unlock(&indio_dev->mlock);
			break;

		case ST_INDIO_DEV_GYRO_WK:
			if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO_WK)) {
				if (sdata->cdata->sensors_enabled & (1 << ST_INDIO_DEV_GYRO)) {
					if (odr > sdata->c_odr)
						need_set_register = true;
				} else
					need_set_register = true;
			}
			mutex_lock(&indio_dev->mlock);
			err = st_lsm6ds3_set_odr(sdata, odr, need_set_register);
			mutex_unlock(&indio_dev->mlock);
			break;
		}
	}

	return err < 0 ? err : size;
}

static ssize_t st_lsm6ds3_sysfs_sampling_frequency_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < ST_LSM6DS3_ODR_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
					st_lsm6ds3_odr_table.odr_avl[i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6ds3_sysfs_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	for (i = 0; i < ST_LSM6DS3_FS_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
			st_lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6ds3_sysfs_get_selftest_available(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s, %s, %s\n",
				st_lsm6ds3_selftest_table[0].string_mode,
				st_lsm6ds3_selftest_table[1].string_mode,
				st_lsm6ds3_selftest_table[2].string_mode);
}

static ssize_t st_lsm6ds3_sysfs_get_selftest_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 index;
	struct lsm6ds3_sensor_data *sdata = iio_priv(dev_get_drvdata(dev));

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		index = sdata->cdata->accel_selftest_status;
		break;
	case ST_INDIO_DEV_GYRO:
		index = sdata->cdata->gyro_selftest_status;
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%s\n",
				st_lsm6ds3_selftest_table[index].string_mode);
}

static ssize_t st_lsm6ds3_sysfs_set_selftest_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err, i;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	for (i = 0; i < ARRAY_SIZE(st_lsm6ds3_selftest_table); i++) {
		if (strncmp(buf, st_lsm6ds3_selftest_table[i].string_mode,
								size - 2) == 0)
			break;
	}
	if (i == ARRAY_SIZE(st_lsm6ds3_selftest_table))
		return -EINVAL;

	err = st_lsm6ds3_set_selftest(sdata, i);
	if (err < 0)
		return err;

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		sdata->cdata->accel_selftest_status = i;
		break;
	case ST_INDIO_DEV_GYRO:
		sdata->cdata->gyro_selftest_status = i;
		break;
	default:
		return -EINVAL;
	}

	return size;
}

#if defined(CONFIG_ST_LSM6DS3_SELFTEST) || defined(CONFIG_ST_LSM6DS3_CAL_SUPPORT)
int st_lsm6ds3_average_sample(struct lsm6ds3_sensor_data *sdata,
				s32 *out_data, int sample_count)
{
	int i, err, counter = 0;
	u8 hw_data[ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE];
	u8 stat_reg;
	struct lsm6ds3_data *cdata = sdata->cdata;
	bool data_avl = false;

	msleep(DELAY_FOR_OUT_STABLE);
	/* first sample will be discarded */
	for (i = 0; i < (sample_count + 1); i++) {
		while (!data_avl) {
			dev_dbg(cdata->dev, "counter=%d\n", counter);
			if (++counter > MAX_WHILE_COUNTER)
				return -EAGAIN;

			if (sdata->c_odr)
				msleep((unsigned long)((1000 / sdata->c_odr) + 1));
			err = cdata->tf->read(cdata, ST_LSM6DS3_STAT_REG_ADDR, 1, &stat_reg, true);
			if (err < 0) {
				dev_err(cdata->dev, "failed to read STATUS_REG.\n");
				return err;
			}
			switch (sdata->sindex) {
			case ST_INDIO_DEV_ACCEL:
				if (stat_reg & ST_LSM6DS3_XLDA_MASK)
					data_avl = true;
				break;
			case ST_INDIO_DEV_GYRO:
				if (stat_reg & ST_LSM6DS3_GDA_MASK)
					data_avl = true;
				break;
			}
		}
		/* Now data is ready */
		err = cdata->tf->read(cdata, cdata->indio_dev[sdata->sindex]->channels->address,
									ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE, hw_data, true);
		if (err < 0) {
			dev_err(cdata->dev, "failed to read out data.\n");
			return err;
		}
		dev_dbg(cdata->dev, "x=%x, y=%x, z=%x\n",
			(s16)((hw_data[1] << 8 | hw_data[0])),
			(s16)((hw_data[3] << 8 | hw_data[2])),
			(s16)((hw_data[5] << 8 | hw_data[4])));
		if (i != 0) {
			out_data[0] += (s16)((hw_data[1] << 8 | hw_data[0]));
			out_data[1] += (s16)((hw_data[3] << 8 | hw_data[2]));
			out_data[2] += (s16)((hw_data[5] << 8 | hw_data[4]));
		}
		data_avl = false;
	}

	out_data[0] /= sample_count;
	out_data[1] /= sample_count;
	out_data[2] /= sample_count;
	dev_dbg(cdata->dev, "x=%d, y=%d, z=%d\n", out_data[0], out_data[1], out_data[2]);

	return 0;
}
#endif

#ifdef CONFIG_ST_LSM6DS3_SELFTEST
u32 st_lsm6ds3_get_sensitivity(struct lsm6ds3_sensor_data *sdata)
{
	u32 sensitivity = 0;
	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		switch (sdata->c_gain[0]) {
		case ST_LSM6DS3_ACCEL_FS_2G_GAIN:
			sensitivity = ST_LSM6DS3_ACCEL_FS_2G_SENSITIVITY;
			break;
		case ST_LSM6DS3_ACCEL_FS_4G_GAIN:
			sensitivity = ST_LSM6DS3_ACCEL_FS_4G_SENSITIVITY;
			break;
		case ST_LSM6DS3_ACCEL_FS_8G_GAIN:
			sensitivity = ST_LSM6DS3_ACCEL_FS_8G_SENSITIVITY;
			break;
		case ST_LSM6DS3_ACCEL_FS_16G_GAIN:
			sensitivity = ST_LSM6DS3_ACCEL_FS_16G_SENSITIVITY;
			break;
		}
		break;
	case ST_INDIO_DEV_GYRO:
		switch (sdata->c_gain[0]) {
		case ST_LSM6DS3_GYRO_FS_245_GAIN:
			sensitivity = ST_LSM6DS3_GYRO_FS_245_SENSITIVITY;
			break;
		case ST_LSM6DS3_GYRO_FS_500_GAIN:
			sensitivity = ST_LSM6DS3_GYRO_FS_500_SENSITIVITY;
			break;
		case ST_LSM6DS3_GYRO_FS_1000_GAIN:
			sensitivity = ST_LSM6DS3_GYRO_FS_1000_SENSITIVITY;
			break;
		case ST_LSM6DS3_GYRO_FS_2000_GAIN:
			sensitivity = ST_LSM6DS3_GYRO_FS_2000_SENSITIVITY;
			break;
		}
		break;
	}

	return sensitivity;
}

ssize_t st_lsm6ds3_sysfs_get_fact_selftest_rlt(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int err;
	s32 out_nost[3] = {0, 0, 0};
	s32 out_st[3] = {0, 0, 0};
	u32 outx_diff, outy_diff, outz_diff;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);
	u32 sensitivity;

	err = st_lsm6ds3_average_sample(sdata, out_nost, FACT_SELF_SAMPLE_COUNT);
	if (err < 0)
		return err;

	/* set self test mode to positive-sign */
	err = st_lsm6ds3_set_selftest(sdata, 1);
	if (err < 0) {
		dev_err(dev, "set self test mode to p-s failed.\n");
		return err;
	}
	err = st_lsm6ds3_average_sample(sdata, out_st, FACT_SELF_SAMPLE_COUNT);
	if (err < 0)
		return err;

	/* disable self test function */
	err = st_lsm6ds3_set_selftest(sdata, 0);
	if (err < 0) {
		dev_err(dev, "disable self test failed.\n");
		return err;
	}

	sensitivity = st_lsm6ds3_get_sensitivity(sdata);
	outx_diff = ABS((out_st[0] - out_nost[0])) * sensitivity;
	outy_diff = ABS((out_st[1] - out_nost[1])) * sensitivity;
	outz_diff = ABS((out_st[2] - out_nost[2])) * sensitivity;
	dev_dbg(dev, "sensitivity=%d, diff x=%d, y=%d, z=%d\n",
		sensitivity, outx_diff, outy_diff, outz_diff);
	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
		if (outx_diff > ACCEL_MIN_DIFF &&
				outx_diff < ACCEL_MAX_DIFF &&
				outy_diff > ACCEL_MIN_DIFF &&
				outy_diff < ACCEL_MAX_DIFF &&
				outz_diff > ACCEL_MIN_DIFF &&
				outz_diff < ACCEL_MAX_DIFF)
			return sprintf(buf, "%s\n", "PASSED");
	case ST_INDIO_DEV_GYRO:
		if (outx_diff > GYRO_MIN_DIFF &&
				outx_diff < GYRO_MAX_DIFF &&
				outy_diff > GYRO_MIN_DIFF &&
				outy_diff < GYRO_MAX_DIFF &&
				outz_diff > GYRO_MIN_DIFF &&
				outz_diff < GYRO_MAX_DIFF)
			return sprintf(buf, "%s\n", "PASSED");
	}

	return sprintf(buf, "%s\n", "FAILED");
}
#endif

#ifdef CONFIG_ST_LSM6DS3_CAL_SUPPORT
ssize_t st_lsm6ds3_sysfs_do_calibrate(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	s32 no_cali[3] = {0};
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = st_lsm6ds3_average_sample(sdata, no_cali, CALIBRATE_SAMPLE_COUNT);
	if (err < 0)
		return err;

	/* The 4th parameter used for data sum check */
	if (sdata->sindex == ST_INDIO_DEV_ACCEL)
		return sprintf(buf, "%d %d %d %d\n",
					0 * SIGN_X_A - no_cali[0],
					0 * SIGN_Y_A - no_cali[1],
					GRAVITY_ACCEL_LSB_2G * SIGN_Z_A - no_cali[2],
					0 * SIGN_X_A + 0 * SIGN_Y_A + GRAVITY_ACCEL_LSB_2G * SIGN_Z_A -
					no_cali[0] - no_cali[1] - no_cali[2]);
	else
		return sprintf(buf, "%d %d %d %d\n",
					0 * SIGN_X_G - no_cali[0],
					0 * SIGN_Y_G - no_cali[1],
					0 * SIGN_Z_G - no_cali[2],
					0 * SIGN_X_G + 0 * SIGN_Y_G + 0 * SIGN_Z_G -
					no_cali[0] - no_cali[1] - no_cali[2]);

}
#endif

ssize_t st_lsm6ds3_sysfs_get_hw_fifo_lenght(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ST_LSM6DS3_MAX_FIFO_LENGHT);
}

ssize_t st_lsm6ds3_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u64 sensor_last_timestamp, event_dir = 0;
	int stype = 0;
	int flags = READ_FIFO_IN_FLUSH;
	u64 timestamp_flush = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);
	int err;
	unsigned int sensor_handle;

	mutex_lock(&indio_dev->mlock);

	err = kstrtouint(buf, 10, &sensor_handle);
	if (err < 0) {
		mutex_unlock(&indio_dev->mlock);
		return -EINVAL;
	}
	dev_info(dev, "sensor_handle = %d", sensor_handle);

	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
		disable_irq(sdata->cdata->irq);
		st_lsm6ds3_flush_works();
	} else {
		mutex_unlock(&indio_dev->mlock);
		return 0;
	}

	switch (sdata->sindex) {
	case ST_INDIO_DEV_ACCEL:
	case ST_INDIO_DEV_GYRO:
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	case ST_INDIO_DEV_EXT0:
	case ST_INDIO_DEV_EXT1:
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		sensor_last_timestamp = sdata->cdata->timestamp;
		break;

	default:
		mutex_unlock(&indio_dev->mlock);
		enable_irq(sdata->cdata->irq);
		return -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	mutex_lock(&sdata->cdata->fifo_lock);
	if (sdata->cdata->system_state & SF_RESUME)
		flags |= READ_FIFO_DISCARD_DATA;
	sdata->cdata->system_state = SF_NORMAL;
	st_lsm6ds3_read_fifo(sdata->cdata, flags);

	switch (sdata->sindex) {
         case ST_INDIO_DEV_ACCEL:
                 if (sensor_last_timestamp == sdata->cdata->timestamp)
			event_dir = IIO_EV_DIR_FIFO_EMPTY;
		 else
			event_dir = IIO_EV_DIR_FIFO_DATA;

		stype = IIO_ACCEL;
		break;

         case ST_INDIO_DEV_GYRO:
                if (sensor_last_timestamp == sdata->cdata->timestamp)
			event_dir = IIO_EV_DIR_FIFO_EMPTY;
		 else
			event_dir = IIO_EV_DIR_FIFO_DATA;

		stype = IIO_ANGL_VEL;
		break;

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	case ST_INDIO_DEV_EXT0:
		if (sensor_last_timestamp == sdata->cdata->timestamp)
			event_dir = IIO_EV_DIR_FIFO_EMPTY;
		 else
			event_dir = IIO_EV_DIR_FIFO_DATA;

		stype = IIO_MAGN;
		break;

	case ST_INDIO_DEV_EXT1:
		if (sensor_last_timestamp == sdata->cdata->timestamp)
			event_dir = IIO_EV_DIR_FIFO_EMPTY;
		 else
			event_dir = IIO_EV_DIR_FIFO_DATA;

		stype = IIO_PRESSURE;

		break;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	}

	timestamp_flush = sdata->cdata->timestamp;
	iio_push_event(indio_dev, IIO_UNMOD_EVENT_CODE(stype,
				sensor_handle, IIO_EV_TYPE_FIFO_FLUSH, event_dir),
				timestamp_flush);

	mutex_unlock(&sdata->cdata->fifo_lock);
	enable_irq(sdata->cdata->irq);

	return size;
}

static ST_LSM6DS3_DEV_ATTR_SAMP_FREQ();
static ST_LSM6DS3_DEV_ATTR_SAMP_FREQ_AVAIL();
static ST_LSM6DS3_DEV_ATTR_SCALE_AVAIL(in_accel_scale_available);
static ST_LSM6DS3_DEV_ATTR_SCALE_AVAIL(in_anglvel_scale_available);
static ST_LSM6DS3_FIFO_LENGHT();
static ST_LSM6DS3_FIFO_FLUSH();

static IIO_DEVICE_ATTR(reset_counter, S_IWUSR,
				NULL, st_lsm6ds3_sysfs_reset_counter, 0);

static IIO_DEVICE_ATTR(max_delivery_rate, S_IWUSR | S_IRUGO,
				st_lsm6ds3_sysfs_get_max_delivery_rate,
				st_lsm6ds3_sysfs_set_max_delivery_rate, 0);

static IIO_DEVICE_ATTR(self_test_available, S_IRUGO,
				st_lsm6ds3_sysfs_get_selftest_available,
				NULL, 0);

static IIO_DEVICE_ATTR(self_test, S_IWUSR | S_IRUGO,
				st_lsm6ds3_sysfs_get_selftest_status,
				st_lsm6ds3_sysfs_set_selftest_status, 0);

#ifdef CONFIG_ST_LSM6DS3_SELFTEST
static IIO_DEVICE_ATTR(fact_selftest, S_IRUGO,
				st_lsm6ds3_sysfs_get_fact_selftest_rlt,
				NULL, 0);
#endif

#ifdef CONFIG_ST_LSM6DS3_CAL_SUPPORT
static IIO_DEVICE_ATTR(do_calibrate, S_IRUGO,
				st_lsm6ds3_sysfs_do_calibrate,
				NULL, 0);
#endif

static struct attribute *st_lsm6ds3_accel_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_self_test_available.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
#ifdef CONFIG_ST_LSM6DS3_SELFTEST
	&iio_dev_attr_fact_selftest.dev_attr.attr,
#endif
#ifdef CONFIG_ST_LSM6DS3_CAL_SUPPORT
	&iio_dev_attr_do_calibrate.dev_attr.attr,
#endif
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static struct attribute *st_lsm6ds3_accel_wk_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3_accel_attribute_group = {
	.attrs = st_lsm6ds3_accel_attributes,
};

static const struct attribute_group st_lsm6ds3_accel_wk_attribute_group = {
	.attrs = st_lsm6ds3_accel_wk_attributes,
};

static const struct iio_info st_lsm6ds3_accel_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_accel_attribute_group,
	.read_raw = &st_lsm6ds3_read_raw,
	.write_raw = &st_lsm6ds3_write_raw,
};

static const struct iio_info st_lsm6ds3_accel_wk_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_accel_wk_attribute_group,
	.read_raw = &st_lsm6ds3_read_raw,
	.write_raw = &st_lsm6ds3_write_raw,
};

static struct attribute *st_lsm6ds3_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_self_test_available.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
#ifdef CONFIG_ST_LSM6DS3_SELFTEST
	&iio_dev_attr_fact_selftest.dev_attr.attr,
#endif
#ifdef CONFIG_ST_LSM6DS3_CAL_SUPPORT
	&iio_dev_attr_do_calibrate.dev_attr.attr,
#endif
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static struct attribute *st_lsm6ds3_gyro_wk_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3_gyro_attribute_group = {
	.attrs = st_lsm6ds3_gyro_attributes,
};

static const struct attribute_group st_lsm6ds3_gyro_wk_attribute_group = {
	.attrs = st_lsm6ds3_gyro_wk_attributes,
};

static const struct iio_info st_lsm6ds3_gyro_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_gyro_attribute_group,
	.read_raw = &st_lsm6ds3_read_raw,
	.write_raw = &st_lsm6ds3_write_raw,
};

static const struct iio_info st_lsm6ds3_gyro_wk_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_gyro_wk_attribute_group,
	.read_raw = &st_lsm6ds3_read_raw,
	.write_raw = &st_lsm6ds3_write_raw,
};

static struct attribute *st_lsm6ds3_sign_motion_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6ds3_sign_motion_attribute_group = {
	.attrs = st_lsm6ds3_sign_motion_attributes,
};

static const struct iio_info st_lsm6ds3_sign_motion_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_sign_motion_attribute_group,
	.read_event_value = &st_lsm6ds3_read_thresh,
	.write_event_value = &st_lsm6ds3_write_thresh,
	.read_event_config = &st_lsm6ds3_read_interrupt_config,
	.write_event_config = &st_lsm6ds3_write_interrupt_config,
};

static struct attribute *st_lsm6ds3_step_c_attributes[] = {
	&iio_dev_attr_reset_counter.dev_attr.attr,
	&iio_dev_attr_max_delivery_rate.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6ds3_step_c_attribute_group = {
	.attrs = st_lsm6ds3_step_c_attributes,
};

static const struct iio_info st_lsm6ds3_step_c_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_step_c_attribute_group,
	.read_raw = &st_lsm6ds3_read_raw,
};

static struct attribute *st_lsm6ds3_step_d_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6ds3_step_d_attribute_group = {
	.attrs = st_lsm6ds3_step_d_attributes,
};

static const struct iio_info st_lsm6ds3_step_d_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_step_d_attribute_group,
	.read_event_value = &st_lsm6ds3_read_thresh,
	.write_event_value = &st_lsm6ds3_write_thresh,
	.read_event_config = &st_lsm6ds3_read_interrupt_config,
	.write_event_config = &st_lsm6ds3_write_interrupt_config,
};

static struct attribute *st_lsm6ds3_tilt_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6ds3_tilt_attribute_group = {
	.attrs = st_lsm6ds3_tilt_attributes,
};

static const struct iio_info st_lsm6ds3_tilt_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6ds3_tilt_attribute_group,
	.read_event_value = &st_lsm6ds3_read_thresh,
	.write_event_value = &st_lsm6ds3_write_thresh,
	.read_event_config = &st_lsm6ds3_read_interrupt_config,
	.write_event_config = &st_lsm6ds3_write_interrupt_config,
};

#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops st_lsm6ds3_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = ST_LSM6DS3_TRIGGER_SET_STATE,
};
#define ST_LSM6DS3_TRIGGER_OPS (&st_lsm6ds3_trigger_ops)
#else
#define ST_LSM6DS3_TRIGGER_OPS NULL
#endif

int st_lsm6ds3_common_probe(struct lsm6ds3_data *cdata, int irq)
{
	u8 wai = 0x00;
	int i, n, err;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->fifo_lock);

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	mutex_init(&cdata->passthrough_lock);
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	cdata->fifo_data = NULL;
	cdata->fifo = devm_kzalloc(cdata->dev, ST_LSM6DS3_RX_MAX_LENGTH,
				   GFP_KERNEL);
	if (!cdata->fifo)
		return -ENOMEM;

	err = cdata->tf->read(cdata, ST_LSM6DS3_WAI_ADDRESS, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return -EPROBE_DEFER;
	}
	if (wai != ST_LSM6DS3_WAI_EXP) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	for (i = 0; i < ST_INDIO_DEV_NUM; i++) {
		cdata->indio_dev[i] = iio_device_alloc(sizeof(*sdata));
		if (cdata->indio_dev[i] == NULL) {
			err = -ENOMEM;
			goto iio_device_free;
		}
		sdata = iio_priv(cdata->indio_dev[i]);
		sdata->cdata = cdata;
		sdata->sindex = i;

		if ((i == ST_INDIO_DEV_ACCEL) ||
				(i == ST_INDIO_DEV_ACCEL_WK) ||
				(i == ST_INDIO_DEV_GYRO) ||
				(i == ST_INDIO_DEV_GYRO_WK)) {
			sdata->odr = st_lsm6ds3_odr_table.odr_avl[0].hz;
			sdata->c_gain[0] =
					st_lsm6ds3_fs_table[i].fs_avl[0].gain;
		}
		cdata->indio_dev[i]->modes = INDIO_DIRECT_MODE;
	}

	if (irq > 0) {
		cdata->irq = irq;
		dev_info(cdata->dev, "driver use DRDY int pin 1\n");
	}

	cdata->indio_dev[ST_INDIO_DEV_ACCEL]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_ACCEL_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_ACCEL]->info = &st_lsm6ds3_accel_info;
	cdata->indio_dev[ST_INDIO_DEV_ACCEL]->channels = st_lsm6ds3_accel_ch;
	cdata->indio_dev[ST_INDIO_DEV_ACCEL]->num_channels =
						ARRAY_SIZE(st_lsm6ds3_accel_ch);

	cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]->name =
		kasprintf(GFP_KERNEL, "%s_%s_%s", cdata->name,
					ST_LSM6DS3_ACCEL_SUFFIX_NAME, ST_LSM6DS3_WAKEUP_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]->info = &st_lsm6ds3_accel_wk_info;
	cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]->channels = st_lsm6ds3_accel_ch;
	cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]->num_channels =
						ARRAY_SIZE(st_lsm6ds3_accel_ch);

	cdata->indio_dev[ST_INDIO_DEV_GYRO]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_GYRO_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_GYRO]->info = &st_lsm6ds3_gyro_info;
	cdata->indio_dev[ST_INDIO_DEV_GYRO]->channels = st_lsm6ds3_gyro_ch;
	cdata->indio_dev[ST_INDIO_DEV_GYRO]->num_channels =
						ARRAY_SIZE(st_lsm6ds3_gyro_ch);

	cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]->name =
		kasprintf(GFP_KERNEL, "%s_%s_%s", cdata->name,
					ST_LSM6DS3_GYRO_SUFFIX_NAME, ST_LSM6DS3_WAKEUP_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]->info = &st_lsm6ds3_gyro_wk_info;
	cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]->channels = st_lsm6ds3_gyro_ch;
	cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]->num_channels =
						ARRAY_SIZE(st_lsm6ds3_gyro_ch);

	cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_SIGN_MOTION_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]->info =
						&st_lsm6ds3_sign_motion_info;
	cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]->channels =
						st_lsm6ds3_sign_motion_ch;
	cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]->num_channels =
					ARRAY_SIZE(st_lsm6ds3_sign_motion_ch);

	cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_STEP_COUNTER_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]->info =
						&st_lsm6ds3_step_c_info;
	cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]->channels =
						st_lsm6ds3_step_c_ch;
	cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]->num_channels =
					ARRAY_SIZE(st_lsm6ds3_step_c_ch);

	cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_STEP_DETECTOR_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]->info =
						&st_lsm6ds3_step_d_info;
	cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]->channels =
						st_lsm6ds3_step_d_ch;
	cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]->num_channels =
					ARRAY_SIZE(st_lsm6ds3_step_d_ch);

	cdata->indio_dev[ST_INDIO_DEV_TILT]->name =
		kasprintf(GFP_KERNEL, "%s_%s", cdata->name,
					ST_LSM6DS3_TILT_SUFFIX_NAME);
	cdata->indio_dev[ST_INDIO_DEV_TILT]->info = &st_lsm6ds3_tilt_info;
	cdata->indio_dev[ST_INDIO_DEV_TILT]->channels = st_lsm6ds3_tilt_ch;
	cdata->indio_dev[ST_INDIO_DEV_TILT]->num_channels =
					ARRAY_SIZE(st_lsm6ds3_tilt_ch);

	err = st_lsm6ds3_init_sensor(cdata);
	if (err < 0)
		goto iio_device_free;

	err = st_lsm6ds3_allocate_rings(cdata);
	if (err < 0)
		goto iio_device_free;

	if (irq > 0) {
		err = st_lsm6ds3_allocate_triggers(cdata,
							ST_LSM6DS3_TRIGGER_OPS);
		if (err < 0)
			goto deallocate_ring;
	}

	for (n = 0; n < ST_INDIO_DEV_NUM; n++) {
		err = iio_device_register(cdata->indio_dev[n]);
		if (err)
			goto iio_device_unregister_and_trigger_deallocate;
	}

	err = st_lsm6ds3_i2c_master_probe(cdata);
	if (err < 0)
		goto iio_device_unregister_and_trigger_deallocate;

	device_init_wakeup(cdata->dev, true);

	pm_runtime_enable(cdata->dev);

	return 0;

iio_device_unregister_and_trigger_deallocate:
	for (n--; n >= 0; n--)
		iio_device_unregister(cdata->indio_dev[n]);

	if (irq > 0)
		st_lsm6ds3_deallocate_triggers(cdata);
deallocate_ring:
	st_lsm6ds3_deallocate_rings(cdata);
iio_device_free:
	for (i--; i >= 0; i--)
		iio_device_free(cdata->indio_dev[i]);

	return err;
}
EXPORT_SYMBOL(st_lsm6ds3_common_probe);

void st_lsm6ds3_common_remove(struct lsm6ds3_data *cdata, int irq)
{
	int i;

	pm_runtime_disable(cdata->dev);

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_device_unregister(cdata->indio_dev[i]);

	if (irq > 0)
		st_lsm6ds3_deallocate_triggers(cdata);

	st_lsm6ds3_deallocate_rings(cdata);

	for (i = 0; i < ST_INDIO_DEV_NUM; i++)
		iio_device_free(cdata->indio_dev[i]);

	st_lsm6ds3_i2c_master_exit(cdata);
}
EXPORT_SYMBOL(st_lsm6ds3_common_remove);

#ifdef CONFIG_PM
int st_lsm6ds3_common_suspend(struct lsm6ds3_data *cdata)
{
	int err, i;
	struct lsm6ds3_sensor_data *sdata;

	if (!stay_wake && (cdata->sensors_enabled & ST_INDIO_DEV_AG_MASK)) {
		for (i = 0; i < (ST_INDIO_DEV_GYRO_WK + 1); i++) {
			sdata = iio_priv(cdata->indio_dev[i]);
			if ((1 << sdata->sindex) & cdata->sensors_enabled) {
				err = st_lsm6ds3_set_drdy_irq(sdata, false);
				if (err < 0)
					return err;
			}
		}
	}

	/* Disable step detector irq */
	if (cdata->sensors_enabled & (1 << ST_INDIO_DEV_STEP_DETECTOR)) {
		dev_dbg(cdata->dev, "st_lsm6ds3_common_suspend disable step detector\n");
		sdata = iio_priv(cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]);
		err = st_lsm6ds3_set_drdy_irq(sdata, false);
		if (err < 0)
			return err;
	}

	/* Disable step counter irq*/
	if (cdata->sensors_enabled & (1 << ST_INDIO_DEV_STEP_COUNTER)) {
		dev_dbg(cdata->dev, "st_lsm6ds3_common_suspend disable step counter\n");
		sdata = iio_priv(cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]);
		err = st_lsm6ds3_set_drdy_irq(sdata, false);
		if (err < 0)
			return err;
	}

	if (stay_wake)
		if (device_may_wakeup(cdata->dev))
			enable_irq_wake(cdata->irq);

	cdata->system_state = SF_SUSPEND;
	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_common_suspend);

int st_lsm6ds3_common_resume(struct lsm6ds3_data *cdata)
{
	int err, i;
	struct lsm6ds3_sensor_data *sdata;

	cdata->system_state = SF_RESUME;
	if (stay_wake) {
		cdata->system_state |= SF_WAKEUP_SENSOR_ENABLED;
		if (device_may_wakeup(cdata->dev))
			disable_irq_wake(cdata->irq);
	} else if (cdata->sensors_enabled & ST_INDIO_DEV_AG_MASK) {
		for (i = 0; i < (ST_INDIO_DEV_GYRO_WK + 1); i++) {
			sdata = iio_priv(cdata->indio_dev[i]);
			if ((1 << sdata->sindex) & cdata->sensors_enabled) {
				err = st_lsm6ds3_set_drdy_irq(sdata, true);
				if (err < 0)
					return err;
			}
		}
	}

	/* Enable step detector irq */
	if (cdata->sensors_enabled & (1 << ST_INDIO_DEV_STEP_DETECTOR)) {
		dev_dbg(cdata->dev, "st_lsm6ds3_common_resume enable step detector\n");

		/* timestamp for step detector should be updated on
		 * system resume. Otherwise, event will be considered invalid.
		 */
		cdata->timestamp = ktime_to_ns(ktime_get_boottime());
		sdata = iio_priv(cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]);
		err = st_lsm6ds3_set_drdy_irq(sdata, true);
		if (err < 0)
			return err;
	}

	/* Enable step counter irq */
	if (cdata->sensors_enabled & (1 << ST_INDIO_DEV_STEP_COUNTER)) {
		dev_dbg(cdata->dev, "st_lsm6ds3_common_resume enable step counter\n");

		/* timestamp for step counter should be updated on
		 * system resume. Otherwise, event will be considered invalid.
		 */
		cdata->timestamp = ktime_to_ns(ktime_get_boottime());
		iio_trigger_poll_chained(
			cdata->trig[ST_INDIO_DEV_STEP_COUNTER], 0);
		sdata = iio_priv(cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]);
		err = st_lsm6ds3_set_drdy_irq(sdata, true);
		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6ds3_common_resume);
#endif /* CONFIG_PM */

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3 core driver");
MODULE_LICENSE("GPL v2");

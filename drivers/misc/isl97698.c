/*
 * isl97698.c - Brightness Driver
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <linux/backlight.h>
#include <linux/isl97698.h>
#include <linux/reboot.h>
#include <linux/pm_runtime.h>

#define BACKLIGHT_NAME		"i2c-bl"

#define ISL97698_REG_LED_H8		0x00
#define ISL97698_REG_LED_L3		0x01
#define ISL97698_REG_CONF		0x02
#define ISL97698_REG_PFM		0x03
#define ISL97698_REG_BOOST		0x04
#define ISL97698_REG_STATUS		0x10

#define ISL97698_ST_FAULT_MASK	0xF0
#define ISL97698_ST_BMOD_MASK	0x08
#define ISL97698_ST_LEDON_MASK	0x04
#define ISL97698_ST_CH1OK_MASK	0x02
#define ISL97698_ST_CH0OK_MASK	0x01

#define ISL97698_PEAK_CUR_MASK		0xF0
#define ISL97698_PEAK_CUR_N			0x00
#define ISL97698_AVERG_CUR_MASK		0X0F
#define ISL97698_AVERG_CUR_N		0x00

#define ISL97698_BST_RATE_MASK		0xC0
#define ISL97698_BST_RATE_SLOWEST	0x00
#define ISL97698_BST_RATE_SLOW		0x40
#define ISL97698_BST_RATE_FAST		0x80
#define ISL97698_BST_RATE_FASTEST	0xC0

#define ISL97698_BST_LOAD_MASK		0x20
#define ISL97698_BST_SYNC_PULSE		0x20
#define ISL97698_BST_PURE_PFM		0x00

#define ISL97698_BST_ABS_MASK		0x10
#define ISL97698_BST_ABS_ENABLE		0x10
#define ISL97698_BST_ABS_DISABLE	0x00

#define ISL97698_BST_FREQ_464KHZ	0x00
#define ISL97698_BST_FREQ_486KHZ	0x01
#define ISL97698_BST_FREQ_510KHZ	0x02
#define ISL97698_BST_FREQ_537KHZ	0x03
#define ISL97698_BST_FREQ_567KHZ	0x04
#define ISL97698_BST_FREQ_600KHZ	0x05
#define ISL97698_BST_FREQ_638KHZ	0x06
#define ISL97698_BST_FREQ_680KHZ	0x07
#define ISL97698_BST_FREQ_729KHZ	0x08
#define ISL97698_BST_FREQ_785KHZ	0x09
#define ISL97698_BST_FREQ_850KHZ	0x0A
#define ISL97698_BST_FREQ_927KHZ	0x0B
#define ISL97698_BST_FREQ_1020KHZ	0x0C
#define ISL97698_BST_FREQ_1133KHZ	0x0D
#define ISL97698_BST_FREQ_1275KHZ	0x0E
#define ISL97698_BST_FREQ_1457KHZ	0x0F

/* Brightness level: Min(0), Max (100), Defalut(50) */
#define ISL_BRIGHTNESS_LEVEL_MIN	0
#define ISL_BRIGHTNESS_LEVEL_MAX	100
#define ISL_BRIGHTNESS_LEVEL_INIT	50

/* Default config:  I2C, Enable Fault(OPCP, OTP), Enable VSC,
 * Use 16V OVP, Disable dither, enable channel0, disable channel 1.
 */
#define ISL_CONF_DEF		0x35

/* PFM peak current: 296mA, Average inductor current to enter PFM mode: 93mA. */
#define ISL_PFM_MODE_DEF	0x83

#define ISL_BST_MODE_DEF	(ISL97698_BST_RATE_FASTEST | \
							ISL97698_BST_SYNC_PULSE | \
							ISL97698_BST_ABS_ENABLE | \
							ISL97698_BST_FREQ_850KHZ)

#define ISL_CHIP_DISABLE	0
#define ISL_CHIP_ENABLE		1

struct isl97698_st {
	struct i2c_client *client;
	struct device *dev;
	struct backlight_device *bl;
	int bias_en;
	int isl_brightness_val_max;
	int enable;
};

static struct isl97698_st *isl97698_st_chip = NULL;

static int isl97698_i2c_write(struct i2c_client *client, u8 addr, u8 val)
{
	int ret;
	u8 txbuf[2];
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 2,
		.buf = txbuf,
	};
	txbuf[0] = addr;
	txbuf[1] = val;

	pm_runtime_get_sync(&client->dev);
	ret = i2c_transfer(client->adapter, &msg, 1);
	pm_runtime_put(&client->dev);

	return ret;
}


static int isl97698_i2c_read(struct i2c_client *client, u8 addr, u8 *val)
{
	int ret;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = I2C_M_NOSTART,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		}
	};

	pm_runtime_get_sync(&client->dev);
	ret = i2c_transfer(client->adapter, msgs, 2);
	pm_runtime_put(&client->dev);

	return ret;
}

static int brightness_get_value(struct isl97698_st *isl, int *level)
{
	int ret;
	u8 regval;

	/* PWMI x I2C mode use reg<0x0> for brightness only */
	ret = isl97698_i2c_read(isl->client, ISL97698_REG_LED_H8, &regval);
	if (ret < 0) {
		dev_err(&isl->client->dev, "Read brightness current H8 reg failed.");
		return ret;
	}
	*level = regval * ISL_BRIGHTNESS_LEVEL_MAX / isl->isl_brightness_val_max;

	return 0;
}

static int brightness_set_value(struct isl97698_st *isl, int level)
{
	int ret;
	u8 regval;

	/* PWMI x I2C mode use reg<0x0> for brightness only */
	regval = (isl->isl_brightness_val_max * level) / ISL_BRIGHTNESS_LEVEL_MAX;
	ret = isl97698_i2c_write(isl->client, ISL97698_REG_LED_H8, regval);
	if (ret < 0) {
		dev_err(&isl->client->dev, "Write brightness current H8 reg failed.");
		return ret;
	}

	return 0;
}

static void brightness_set_chip_enable(struct isl97698_st *isl, int enable)
{
	gpio_set_value(isl->bias_en, enable);
	isl->enable = enable;
}

static int brightness_defconfig(struct isl97698_st *isl)
{
	int ret;

	ret = isl97698_i2c_write(isl->client, ISL97698_REG_CONF, ISL_CONF_DEF);
	if (ret < 0) {
		dev_err(&isl->client->dev, "ISL97698_REG_CONF write failed.");
		return ret;
	}

	ret = isl97698_i2c_write(isl->client, ISL97698_REG_PFM, ISL_PFM_MODE_DEF);
	if (ret < 0) {
		dev_err(&isl->client->dev, "ISL97698_REG_PFM write failed.");
		return ret;
	}

	ret = isl97698_i2c_write(isl->client, ISL97698_REG_BOOST, ISL_BST_MODE_DEF);
	if (ret < 0) {
		dev_err(&isl->client->dev, "ISL97698_REG_BOOST write failed.");
		return ret;
	}

	return 0;
}

static int brightness_chip_check(struct isl97698_st *isl)
{
	int ret;
	u8 val;

	ret = isl97698_i2c_read(isl->client, ISL97698_REG_LED_H8, &val);
	if (ret < 0) {
		dev_err(&isl->client->dev, "ISL97698_REG_LED_H8 read failed.");
		return ret;
	}

	ret = isl97698_i2c_read(isl->client, ISL97698_REG_STATUS, &val);
	if (ret < 0) {
		dev_err(&isl->client->dev, "ISL97698_REG_STATUS read failed.");
		return ret;
	}
	if (val & ISL97698_ST_FAULT_MASK) {
		dev_err(&isl->client->dev,
			"brightness fault(OTP/OVP/VSC/BoostHCL), status=%02x", val);
	}

	return 0;
}

static int isl97698_backlight_get_brightness(struct backlight_device *bl)
{
	int ret;
	struct isl97698_st *isl = bl_get_data(bl);

	ret = brightness_get_value(isl, &(bl->props.brightness));
	if (ret < 0)
		dev_err(isl->dev, "i2c failed to access register\n");

	return bl->props.brightness;
}

static int isl97698_backlight_update_brightness(struct backlight_device *bl)
{
	int ret;
	struct isl97698_st *isl = bl_get_data(bl);

	if (bl->props.brightness < ISL_BRIGHTNESS_LEVEL_MIN)
		bl->props.brightness = ISL_BRIGHTNESS_LEVEL_MIN;

	if (bl->props.brightness > ISL_BRIGHTNESS_LEVEL_MAX)
		bl->props.brightness = ISL_BRIGHTNESS_LEVEL_MAX;

	ret = brightness_set_value(isl, bl->props.brightness);
	if (ret < 0)
		dev_err(isl->dev, "i2c failed to access register\n");

	return bl->props.brightness;
}

static const struct backlight_ops isl97698_backlight_ops = {
	.update_status = isl97698_backlight_update_brightness,
	.get_brightness = isl97698_backlight_get_brightness,
};

static int isl97698_backlight_register(struct isl97698_st *chip)
{
	struct backlight_properties props;

	props.type = BACKLIGHT_RAW;
	props.brightness = ISL_BRIGHTNESS_LEVEL_INIT;
	props.max_brightness = ISL_BRIGHTNESS_LEVEL_MAX;
	chip->bl = backlight_device_register(BACKLIGHT_NAME, chip->dev, chip,
				      &isl97698_backlight_ops, &props);
	if (IS_ERR(chip->bl))
		return PTR_ERR(chip->bl);

	backlight_update_status(chip->bl);

	return 0;
}

static void isl97698_backlight_unregister(struct isl97698_st *chip)
{
	if (chip->bl)
		backlight_device_unregister(chip->bl);
}

static int isl_reboot_notify_sys(struct notifier_block *this, unsigned long code,
							void *unused)
{
	if (isl97698_st_chip)
		isl97698_i2c_write(isl97698_st_chip->client, ISL97698_REG_CONF, 0xff);
	return NOTIFY_OK;
}

/*
 *	The backlight needs to learn about soft shutdowns to
 *	reset the register.
 */

static struct notifier_block isl_reboot_notifier = {
	.notifier_call = isl_reboot_notify_sys,
};
static int  isl97698_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int res;
	struct isl97698_platform_data *pdata = client->dev.platform_data;
	struct isl97698_st *chip = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functions unsupported\n");
		return -EOPNOTSUPP;
	}

	if (pdata->bias_en == -1) {
		dev_err(&client->dev, "bias_en invalid\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(struct isl97698_st), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&client->dev, "struct isl97698_st allocation failed\n");
		return -ENOMEM;
	}
	isl97698_st_chip = chip;
	chip->client = client;
	chip->dev = &client->dev;
	chip->bias_en = pdata->bias_en;
	chip->isl_brightness_val_max = pdata->isl_brightness_val_max;
	i2c_set_clientdata(client, chip);

	brightness_set_chip_enable(chip, ISL_CHIP_ENABLE);

	/* Need at least 100us before first I2C access. */
	mdelay(1);

	res = brightness_chip_check(chip);
	if (res <  0) {
		dev_err(&client->dev, "brightness_chip_check failed\n");
		res = -ENXIO;
		goto isl_probe_fail;
	}

	res = brightness_defconfig(chip);
	if (res <  0) {
		dev_err(&client->dev, "brightness_defconfig failed\n");
		res = -EIO;
		goto isl_probe_fail;
	}

	res = isl97698_backlight_register(chip);
	if (res < 0) {
		dev_err(&client->dev, "register backlight failed\n");
		goto isl_probe_fail;
	}
	pm_runtime_enable(&client->dev);

	dev_info(&client->dev, "Brightness isl97698_probe successed\n");

	res = register_reboot_notifier(&isl_reboot_notifier);
	if (res != 0) {
		pr_err("cannot register reboot notifier (err=%d)\n", res);
		goto isl_probe_fail;
	}
	return res;

isl_probe_fail:
	kfree(chip);
	dev_err(&client->dev, "Brightness isl97698_probe failed\n");
	return res;
}

static int isl97698_remove(struct i2c_client *client)
{
	struct isl97698_st *isl = i2c_get_clientdata(client);

	pm_runtime_disable(&client->dev);

	isl97698_backlight_unregister(isl);
	unregister_reboot_notifier(&isl_reboot_notifier);
	return 0;
}

static struct i2c_device_id isl97698_id[] = {
	{ "isl97698", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, isl97698_id);

#ifdef CONFIG_PM
static int isl97698_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl97698_st *isl = i2c_get_clientdata(client);

	if (isl->bl->props.brightness == 0)
		brightness_set_chip_enable(isl, ISL_CHIP_DISABLE);

	return 0;
}

static int isl97698_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl97698_st *isl = i2c_get_clientdata(client);

	if (isl->enable == ISL_CHIP_DISABLE)
		brightness_set_chip_enable(isl, ISL_CHIP_ENABLE);

	return 0;
}
static SIMPLE_DEV_PM_OPS(isl97698_pm_ops, isl97698_suspend, isl97698_resume);
#define ISL97698_PM_OPS (&isl97698_pm_ops)
#else
#define ISL97698_PM_OPS NULL
#endif


static struct i2c_driver isl97698_driver = {
	.driver = {
		.name = "isl97698",
		.pm = ISL97698_PM_OPS,
	},
	.probe = isl97698_probe,
	.remove = isl97698_remove,
	.id_table = isl97698_id,
};

module_i2c_driver(isl97698_driver);

MODULE_AUTHOR("Tan Baixing <baixingx.tan@intel.com>");
MODULE_DESCRIPTION("Isl97698 brightness Driver");
MODULE_LICENSE("GPL");

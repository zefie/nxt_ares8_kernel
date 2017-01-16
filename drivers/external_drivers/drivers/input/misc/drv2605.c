/*
** =============================================================================
** Copyright (c) 2012  Immersion Corporation.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** File:
**	   drv2605.c
**
** Description:
**	   DRV2605 chip driver
**
** =============================================================================
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/types.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>

#include <linux/syscalls.h>
#include <asm/uaccess.h>

#include <linux/gpio.h>

#include <linux/sched.h>

#include <linux/drv2605_vibra.h>

#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>

#include <linux/workqueue.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>

/*	Current code version: 182 */
MODULE_AUTHOR("Immersion Corp.");
MODULE_DESCRIPTION("Driver for "DEVICE_NAME);

#define GO_BIT_POLL_INTERVAL    15
#define STANDBY_WAKE_DELAY      1

#define MAX_TIMEOUT 10000 /* 10s */

/* define GPIO level */
#define GPIO_LEVEL_HIGH	1
#define GPIO_LEVEL_LOW	0

#define SIZE_OTP_MEMORY	FEEDBACK_CONTROL_REG - RATED_VOLTAGE_REG + 1

static struct drv260x {
	struct class *class;
	struct device *device;
	dev_t version;
	struct i2c_client *client;
	struct semaphore sem;
	struct cdev cdev;
} *drv260x;

static struct vibrator {
	struct wake_lock wklock;
	struct pwm_device *pwm_dev;
	struct hrtimer timer;
	struct mutex lock;
	struct work_struct work;
	struct work_struct work_play_eff;
	unsigned char sequence[8];
	volatile int should_stop;
	unsigned gpio_en;
	bool otp_status;
	int calibrate_status;
	struct drv2605_platform_data *pdata;
} vibdata;

enum calibrate_state {
	NEVER_LAUNCH,
	ONGOING,
	SUCCESSFUL,
	FAILED,
};

static int device_id = -1;

static int drv260x_setup(void);
static int drv260x_calibrate(void);
static void drv260x_change_mode(char mode);

static void drv260x_write_reg_val(const unsigned char *data, unsigned int size)
{
	int i = 0;

	if (size % 2 != 0)
		return;

	pm_runtime_get_sync(&drv260x->client->dev);

	while (i < size) {
/*Check if OTP memory have already been written once.*/
		if ((vibdata.otp_status == false) || (data[i] < RATED_VOLTAGE_REG) || (data[i] > FEEDBACK_CONTROL_REG))
			i2c_smbus_write_byte_data(drv260x->client, data[i], data[i + 1]);
		i += 2;
	}

	pm_runtime_put(&drv260x->client->dev);
}

static void drv260x_set_go_bit(char val)
{
	char go[] = {
		GO_REG, val
	};

	drv260x_write_reg_val(go, sizeof(go));
}

static unsigned char drv260x_read_reg(unsigned char reg)
{
	int ret;

	pm_runtime_get_sync(&drv260x->client->dev);
	ret = i2c_smbus_read_byte_data(drv260x->client, reg);
	pm_runtime_put(&drv260x->client->dev);

	return ret;
}

static ssize_t drv260x_calibration_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	switch (vibdata.calibrate_status) {
	case NEVER_LAUNCH:
		return sprintf(buf, "never_launched\n");
		break;
	case ONGOING:
		return sprintf(buf, "ongoing\n");
		break;
	case SUCCESSFUL:
		return sprintf(buf, "vibrator calibrated\n");
		break;
	case FAILED:
		return sprintf(buf, "failed\n");
		break;
	default:
		return sprintf(buf, "unknown state\n");
	}
}

static ssize_t drv260x_do_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (vibdata.calibrate_status == ONGOING)
		return sprintf(buf, "1");
	else
		return sprintf(buf, "0");
}

ssize_t drv260x_do_calibration_store (struct device *d, struct device_attribute *attr,
		const char *buffer, size_t size)
{
	int i;
	sscanf(buffer, "%d", &i);
	if (i == 1)
		return drv260x_calibrate();

	dev_err(drv260x->device, "value not permited \n");
	return -EINVAL;
}

static ssize_t drv260x_OTP_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (vibdata.otp_status == true)
		return sprintf(buf, "memory lock\n");
	else
		return sprintf(buf, "memory unlock\n");
}

/*
if you launch the save procedure you have to follow this step:
	-your device has to be calibrated.
	-"vsys" has to be between 4V and 4.4V
*/

ssize_t drv260x_OTP_save_store (struct device *d, struct  device_attribute *attr,
		const char *buffer, size_t size)
{
	char save[] = {AUTOCAL_MEM_INTERFACE_REG, OTP_PROGRAM};
	char reset[] = {MODE_REG, MODE_RESET};
	char value_init[SIZE_OTP_MEMORY], value_result[SIZE_OTP_MEMORY];
	int i, j = 0;

	drv260x_change_mode(MODE_SOFT_STANDBY);
	if (vibdata.calibrate_status != SUCCESSFUL) {
		dev_err(drv260x->device, "calibration must be run before saving \n");
		goto fail;
	}

	for (i = RATED_VOLTAGE_REG; i <= FEEDBACK_CONTROL_REG; i++) {
		value_init[j] = drv260x_read_reg(i);
		j++;
	}

	drv260x_write_reg_val(save, sizeof(save));
	msleep(500);
	if ((drv260x_read_reg(AUTOCAL_MEM_INTERFACE_REG) & OTP_STATUS_MASK) == false) {
		dev_err(drv260x->device, "otp save failed\n");
		goto fail;
	}
	vibdata.otp_status = true;

	drv260x_write_reg_val(reset, sizeof(reset));
	while (drv260x_read_reg(MODE_REG) & MODE_RESET)
		schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));

	j = 0;
	for (i = RATED_VOLTAGE_REG; i <= FEEDBACK_CONTROL_REG; i++) {
		value_result[j] = drv260x_read_reg(i);
		j++;
	}

	for (j = 0; j < SIZE_OTP_MEMORY; j++)
		dev_info(drv260x->device, "value saved of calibration %02x->%02x\n", value_init[j], value_result[j]);

	for (j = 0; j < SIZE_OTP_MEMORY; j++)
		if (value_result[j] != value_init[j]) {
			dev_err(drv260x->device, "save failed, value saved is not correct %02x!=%02x\n",
									value_init[j], value_result[j]);
			goto fail;
		}
	drv260x_change_mode(MODE_STANDBY);
	return size;

fail:
	drv260x_change_mode(MODE_STANDBY);
	return -EIO;
}

static DEVICE_ATTR(calibration_status, S_IRUGO, drv260x_calibration_status_show, NULL);
static DEVICE_ATTR(OTP_status, S_IRUGO, drv260x_OTP_status_show, NULL);
static DEVICE_ATTR(do_calibration, S_IWUSR | S_IRUGO, drv260x_do_calibration_show, drv260x_do_calibration_store);
static DEVICE_ATTR(OTP_save, S_IWUSR, NULL, drv260x_OTP_save_store);

static struct attribute *drv260x_attr[] = {
	&dev_attr_calibration_status.attr,
	&dev_attr_OTP_status.attr,
	&dev_attr_do_calibration.attr,
	&dev_attr_OTP_save.attr,
	NULL,
};

static const struct attribute_group drv260x_attr_group  = {
	.attrs = drv260x_attr,
};

static void drv2605_poll_go_bit(void)
{
	while (drv260x_read_reg(GO_REG) == GO)
		schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
}

static void drv2605_select_library(char lib)
{
	char library[] = {
		LIBRARY_SELECTION_REG, lib
	};

	drv260x_write_reg_val(library, sizeof(library));
}

static void drv260x_set_rtp_val(char value)
{
	char rtp_val[] = {
		REAL_TIME_PLAYBACK_REG, value
	};

	drv260x_write_reg_val(rtp_val, sizeof(rtp_val));
}

static void drv2605_set_waveform_sequence(unsigned char *seq, unsigned int size)
{
	unsigned char data[WAVEFORM_SEQUENCER_MAX + 1];

	if (size > WAVEFORM_SEQUENCER_MAX)
		return;

	memset(data, 0, sizeof(data));
	memcpy(&data[1], seq, size);
	data[0] = WAVEFORM_SEQUENCER_REG;

	i2c_master_send(drv260x->client, data, sizeof(data));
}

static void drv260x_change_mode(char mode)
{
	unsigned char tmp[] = {
		MODE_REG, mode
	};

	drv260x_write_reg_val(tmp, sizeof(tmp));
}

/* --------------------------------------------------------------------------------- */
#define YES     1
#define NO      0

static void setAudioHapticsEnabled(int enable);
static int audio_haptics_enabled = NO;
static int vibrator_is_playing = NO;

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibdata.timer)) {
		ktime_t r = hrtimer_get_remaining(&vibdata.timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void vibrator_off(void)
{
	if (vibrator_is_playing) {
		vibrator_is_playing = NO;
		if (audio_haptics_enabled) {
			if ((drv260x_read_reg(MODE_REG) & DRV260X_MODE_MASK) != MODE_AUDIOHAPTIC)
				setAudioHapticsEnabled(YES);
		} else
			drv260x_change_mode(MODE_STANDBY);
	}

	wake_unlock(&vibdata.wklock);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	char mode;

	mutex_lock(&vibdata.lock);
	hrtimer_cancel(&vibdata.timer);
	cancel_work_sync(&vibdata.work);

	if (value) {
		wake_lock(&vibdata.wklock);

		mode = drv260x_read_reg(MODE_REG) & DRV260X_MODE_MASK;
		/* Only change the mode if not already in RTP mode; RTP input already set at init */
		if (mode != MODE_REAL_TIME_PLAYBACK) {
			if (audio_haptics_enabled && mode == MODE_AUDIOHAPTIC)
				setAudioHapticsEnabled(NO);
			drv260x_set_rtp_val(vibdata.pdata->real_time_playback);
			drv260x_change_mode(MODE_REAL_TIME_PLAYBACK);
			vibrator_is_playing = YES;
		}

		if (value > 0) {
			if (value > MAX_TIMEOUT)
				value = MAX_TIMEOUT;
			hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	} else
		vibrator_off();

	mutex_unlock(&vibdata.lock);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	schedule_work(&vibdata.work);
	return HRTIMER_NORESTART;
}

static void vibrator_work(struct work_struct *work)
{
	vibrator_off();
}

/* ----------------------------------------------------------------------------- */

static void play_effect(struct work_struct *work)
{
	if (audio_haptics_enabled &&
	    ((drv260x_read_reg(MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC))
		setAudioHapticsEnabled(NO);

	drv260x_change_mode(MODE_INTERNAL_TRIGGER);
	drv2605_set_waveform_sequence(vibdata.sequence, sizeof(vibdata.sequence));
	drv260x_set_go_bit(GO);

	while (drv260x_read_reg(GO_REG) == GO && !vibdata.should_stop)
		schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));

	wake_unlock(&vibdata.wklock);
	if (audio_haptics_enabled)
		setAudioHapticsEnabled(YES);
	else
		drv260x_change_mode(MODE_STANDBY);
}

static void setAudioHapticsEnabled(int enable)
{
	if (enable) {
		if (vibdata.pdata->effect_library != LIBRARY_F) {
			char audiohaptic_settings[] = {
				Control1_REG, STARTUP_BOOST_ENABLED | AC_COUPLE_ENABLED | AUDIOHAPTIC_DRIVE_TIME,
				Control3_REG, NG_Thresh_2 | INPUT_ANALOG
			};
			/* Chip needs to be brought out of standby to change the registers*/
			drv260x_change_mode(MODE_INTERNAL_TRIGGER);
			schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
			drv260x_write_reg_val(audiohaptic_settings, sizeof(audiohaptic_settings));
		}
		drv260x_change_mode(MODE_AUDIOHAPTIC);
	} else {
		drv260x_change_mode(MODE_STANDBY); /* Disable audio-to-haptics*/
		schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
		/* Chip needs to be brought out of standby to change the registers*/
		drv260x_change_mode(MODE_INTERNAL_TRIGGER);
		if (vibdata.pdata->effect_library != LIBRARY_F) {
			char default_settings[] = {
				Control1_REG, STARTUP_BOOST_ENABLED | DEFAULT_DRIVE_TIME,
				Control3_REG, NG_Thresh_2 | ERM_OpenLoop_Enabled
			};
			schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
			drv260x_write_reg_val(default_settings, sizeof(default_settings));
		}
	}
}

static int drv260x_calibrate(void)
{
	char status;

	if (vibdata.calibrate_status == ONGOING) {
		dev_err(drv260x->device, "calibration already ongoing \n");
		return -EBUSY;
	}
	vibdata.calibrate_status = ONGOING;
	drv260x_write_reg_val(vibdata.pdata->parameter_sequence, vibdata.pdata->size_sequence);

	if (vibdata.otp_status == true) {
		dev_info(drv260x->device, "otp status: memory lock\n");
		vibdata.calibrate_status = SUCCESSFUL;
		status = drv260x_read_reg(STATUS_REG);
		return status;
	}

	dev_info(drv260x->device, "otp status: memory unlock\n");
	drv260x_write_reg_val(vibdata.pdata->parameter_autocal_sequence,
							vibdata.pdata->size_autocal_sequence);
	/* Wait until the procedure is done */
	drv2605_poll_go_bit();

	/* Read status */
	status = drv260x_read_reg(STATUS_REG);

	/* Check result */
	if ((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED) {
		dev_err(drv260x->device, "auto-cal failed.\n");
		if (vibdata.pdata->repeat_autocal_sequence == true) {
			drv260x_write_reg_val(vibdata.pdata->parameter_sequence, vibdata.pdata->size_sequence);
			drv260x_write_reg_val(vibdata.pdata->parameter_autocal_sequence,
							vibdata.pdata->size_autocal_sequence);
			drv2605_poll_go_bit();
			status = drv260x_read_reg(STATUS_REG);
			if ((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED)
				dev_err(drv260x->device, "auto-cal retry failed.\n");
		}
	}

	/* Read calibration results */
	drv260x_read_reg(AUTO_CALI_RESULT_REG);
	drv260x_read_reg(AUTO_CALI_BACK_EMF_RESULT_REG);
	drv260x_read_reg(FEEDBACK_CONTROL_REG);
	if ((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED)
		vibdata.calibrate_status = FAILED;
	else
		vibdata.calibrate_status = SUCCESSFUL;

	return status;
}

static int drv260x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	char status;
	int setup_status;

	struct drv2605_platform_data *pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(drv260x->device, "No vibrator platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(drv260x->device, "probe failed");
		return -ENODEV;
	}

	drv260x->client = client;
	setup_status = drv260x_setup();
	if (setup_status) {
		dev_err(drv260x->device, "setup failed");
		return setup_status;
	}

	vibdata.gpio_en = pdata->gpio_en;

	if (gpio_request(vibdata.gpio_en, "drv2605") < 0) {
		dev_err(drv260x->device, "error requesting gpio\n");
		return -EINVAL;
	}

	/* Enable power to the chip */
	gpio_direction_output(vibdata.gpio_en, GPIO_LEVEL_HIGH);

	/* Wait 30 us */
	udelay(30);

	vibdata.pdata = pdata;
	vibdata.otp_status = drv260x_read_reg(AUTOCAL_MEM_INTERFACE_REG) & OTP_STATUS_MASK;

	/*return value is not checked because we still want to probe if the calibration fails */
	drv260x_calibrate();

	/* Read device ID */
	status = drv260x_read_reg(STATUS_REG);
	device_id = (status & DEV_ID_MASK);
	switch (device_id) {
	case DRV2605:
		dev_err(drv260x->device, "driver found: drv2605.\n");
		break;
	case DRV2604:
		dev_err(drv260x->device, "driver found: drv2604.\n");
		break;
	default:
		dev_err(drv260x->device, "driver found: unknown.\n");
		break;
	}

	pm_runtime_enable(&drv260x->client->dev);

	/* Choose default effect library */
	drv2605_select_library(vibdata.pdata->effect_library);

	/* Put hardware in standby */
	drv260x_change_mode(MODE_STANDBY);

	dev_err(drv260x->device, "probe succeeded");
	dev_err(drv260x->device, "driver version: "DRIVER_VERSION);
	return 0;
}

static int drv260x_remove(struct i2c_client *client)
{
	pm_runtime_disable(&drv260x->client->dev);

	dev_err(drv260x->device, "remove");
	return 0;
}

static struct i2c_device_id drv260x_id_table[] = {
	{ DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, drv260x_id_table);

static struct i2c_driver drv260x_driver = {
	.driver		= {
		.name	= DEVICE_NAME,
	},
	.id_table	= drv260x_id_table,
	.probe		= drv260x_probe,
	.remove		= drv260x_remove
};

static struct timed_output_dev to_dev = {
	.name		= "vibrator",
	.get_time	= vibrator_get_time,
	.enable		= vibrator_enable,
};

static char read_val;

static ssize_t drv260x_read(struct file *filp, char *buff, size_t length, loff_t *offset)
{
	buff[0] = read_val;
	return 1;
}

static ssize_t drv260x_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	mutex_lock(&vibdata.lock);
	hrtimer_cancel(&vibdata.timer);

	vibdata.should_stop = YES;
	cancel_work_sync(&vibdata.work_play_eff);
	cancel_work_sync(&vibdata.work);

	if (vibrator_is_playing) {
		vibrator_is_playing = NO;
		drv260x_change_mode(MODE_STANDBY);
	}

	switch (buff[0]) {
	case HAPTIC_CMDID_PLAY_SINGLE_EFFECT:
	case HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE:
	{
		memset(&vibdata.sequence, 0, sizeof(vibdata.sequence));
		if (!copy_from_user(&vibdata.sequence, &buff[1], len - 1)) {
			vibdata.should_stop = NO;
			wake_lock(&vibdata.wklock);
			schedule_work(&vibdata.work_play_eff);
		}
		break;
	}
	case HAPTIC_CMDID_PLAY_TIMED_EFFECT:
	{
		unsigned int value = 0;
		char mode;

		value = buff[2];
		value <<= 8;
		value |= buff[1];

		if (value) {
			wake_lock(&vibdata.wklock);

			mode = drv260x_read_reg(MODE_REG) & DRV260X_MODE_MASK;
			if (mode != MODE_REAL_TIME_PLAYBACK) {
				if (audio_haptics_enabled && mode == MODE_AUDIOHAPTIC)
					setAudioHapticsEnabled(NO);
				drv260x_set_rtp_val(vibdata.pdata->real_time_playback);
				drv260x_change_mode(MODE_REAL_TIME_PLAYBACK);
				vibrator_is_playing = YES;
			}

			if (value > 0) {
				if (value > MAX_TIMEOUT)
					value = MAX_TIMEOUT;
				hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
			}
		}
		break;
	}
	case HAPTIC_CMDID_STOP:
	{
		if (vibrator_is_playing) {
			vibrator_is_playing = NO;
			if (audio_haptics_enabled)
				setAudioHapticsEnabled(YES);
			else
				drv260x_change_mode(MODE_STANDBY);
		}
		vibdata.should_stop = YES;
		break;
	}
	case HAPTIC_CMDID_GET_DEV_ID:
	{
		/* Dev ID includes 2 parts, upper word for device id, lower word for chip revision */
		int revision = (drv260x_read_reg(SILICON_REVISION_REG) & SILICON_REVISION_MASK);
		read_val = (device_id >> 1) | revision;
		break;
	}
	case HAPTIC_CMDID_RUN_DIAG:
	{
		char diag_seq[] = {
			MODE_REG, MODE_DIAGNOSTICS,
			GO_REG,	  GO
		};
		if (audio_haptics_enabled &&
		    ((drv260x_read_reg(MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC))
			setAudioHapticsEnabled(NO);
		drv260x_write_reg_val(diag_seq, sizeof(diag_seq));
		drv2605_poll_go_bit();
		read_val = (drv260x_read_reg(STATUS_REG) & DIAG_RESULT_MASK) >> 3;
		break;
	}
	case HAPTIC_CMDID_AUDIOHAPTIC_ENABLE:
	{
		if ((drv260x_read_reg(MODE_REG) & DRV260X_MODE_MASK) != MODE_AUDIOHAPTIC) {
			setAudioHapticsEnabled(YES);
			audio_haptics_enabled = YES;
		}
		break;
	}
	case HAPTIC_CMDID_AUDIOHAPTIC_DISABLE:
	{
		if (audio_haptics_enabled) {
			if ((drv260x_read_reg(MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC)
				setAudioHapticsEnabled(NO);
			audio_haptics_enabled = NO;
			drv260x_change_mode(MODE_STANDBY);
		}
		break;
	}
	case HAPTIC_CMDID_AUDIOHAPTIC_GETSTATUS:
	{
		if ((drv260x_read_reg(MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC)
			read_val = 1;
		else
			read_val = 0;
		break;
	}
	default:
		break;
	}

	mutex_unlock(&vibdata.lock);

	return len;
}

static struct file_operations fops = {
	.read	= drv260x_read,
	.write	= drv260x_write
};

static int drv260x_init(void)
{
	int reval = -ENOMEM;

	drv260x = kmalloc(sizeof *drv260x, GFP_KERNEL);
	if (!drv260x) {
		dev_err(drv260x->device, "cannot allocate memory for drv260x driver\n");
		return reval;
	}

	reval = i2c_add_driver(&drv260x_driver);
	if (reval) {
		dev_err(drv260x->device, "driver initialization error \n");
		i2c_unregister_device(drv260x->client);
		return reval;
	}
	return 0;
}

static int drv260x_setup(void)
{
	int reval = -ENOMEM;
	struct i2c_client *client = drv260x->client;

	drv260x->version = MKDEV(0, 0);
	reval = alloc_chrdev_region(&drv260x->version, 0, 1, DEVICE_NAME);
	if (reval < 0) {
		dev_err(drv260x->device, "error getting major number %d\n", reval);
		goto fail0;
	}

	drv260x->class = class_create(THIS_MODULE, DEVICE_NAME);
	if (!drv260x->class) {
		dev_err(drv260x->device, "error creating class\n");
		goto fail1;
	}

	drv260x->device = device_create(drv260x->class, NULL, drv260x->version, NULL, DEVICE_NAME);
	if (!drv260x->device) {
		dev_err(drv260x->device, "error creating device 2605\n");
		goto fail2;
	}

	cdev_init(&drv260x->cdev, &fops);
	drv260x->cdev.owner = THIS_MODULE;
	drv260x->cdev.ops = &fops;
	reval = cdev_add(&drv260x->cdev, drv260x->version, 1);

	if (reval) {
		dev_err(drv260x->device, "fail to add cdev\n");
		goto fail3;
	}

	if (timed_output_dev_register(&to_dev) < 0) {
		dev_err(drv260x->device, "fail to create timed output dev\n");
		goto fail4;
	}

	hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibdata.timer.function = vibrator_timer_func;
	INIT_WORK(&vibdata.work, vibrator_work);
	INIT_WORK(&vibdata.work_play_eff, play_effect);

	wake_lock_init(&vibdata.wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&vibdata.lock);

	reval = sysfs_create_group(&client->dev.kobj, &drv260x_attr_group);
	if (reval) {
		dev_err(drv260x->device, "Failed to create sysfs attributes\n");
		goto fail5;
	}

	dev_err(drv260x->device, "initialized\n");
	return 0;

fail5:
	sysfs_remove_group(&client->dev.kobj, &drv260x_attr_group);
fail4:
	unregister_chrdev_region(drv260x->version, 1);
fail3:
	device_destroy(drv260x->class, drv260x->version);
fail2:
	class_destroy(drv260x->class);
fail1:
	gpio_direction_output(vibdata.gpio_en, GPIO_LEVEL_LOW);
	gpio_free(vibdata.gpio_en);
fail0:
	i2c_del_driver(&drv260x_driver);
	i2c_unregister_device(drv260x->client);
	return reval;
}

static void drv260x_exit(void)
{
	sysfs_remove_group(&drv260x->client->dev.kobj, &drv260x_attr_group);
	gpio_direction_output(vibdata.gpio_en, GPIO_LEVEL_LOW);
	gpio_free(vibdata.gpio_en);
	device_destroy(drv260x->class, drv260x->version);
	class_destroy(drv260x->class);
	unregister_chrdev_region(drv260x->version, 1);
	i2c_unregister_device(drv260x->client);
	kfree(drv260x);
	i2c_del_driver(&drv260x_driver);

	dev_err(drv260x->device, "exit\n");
}

module_init(drv260x_init);
module_exit(drv260x_exit);

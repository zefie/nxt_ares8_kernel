/*
 * STMicroelectronics lsm6ds3 buffer driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "st_lsm6ds3.h"

#define ST_LSM6DS3_ENABLE_AXIS			0x07
#define ST_LSM6DS3_FIFO_DIFF_L			0x3a
#define ST_LSM6DS3_FIFO_DIFF_MASK		0x0fff
#define ST_LSM6DS3_FIFO_DATA_OUT_L		0x3e
#define ST_LSM6DS3_FIFO_DATA_OVR_2REGS		0x4000
#define ST_LSM6DS3_FIFO_DATA_PATTERN_L		0x3c

static void st_lsm6ds3_push_data_with_timestamp(struct lsm6ds3_data *cdata,
					u8 index, u8 *data, int64_t timestamp)
{
	int i, n = 0;
	struct iio_chan_spec const *chs = cdata->indio_dev[index]->channels;
	uint16_t bfch, bfchs_out = 0, bfchs_in = 0;
	struct lsm6ds3_sensor_data *sdata = iio_priv(cdata->indio_dev[index]);

	for (i = 0; i < sdata->num_data_channels; i++) {
		bfch = chs[i].scan_type.storagebits >> 3;

		if (test_bit(i, cdata->indio_dev[index]->active_scan_mask)) {
			memcpy(&sdata->buffer_data[bfchs_out],
							&data[bfchs_in], bfch);
			n++;
			bfchs_out += bfch;
		}

		bfchs_in += bfch;
	}

	if (cdata->indio_dev[index]->scan_timestamp)
		*(s64 *)((u8 *)sdata->buffer_data +
			ALIGN(bfchs_out, sizeof(s64))) = timestamp;

	iio_push_to_buffers(cdata->indio_dev[index], sdata->buffer_data);
}

static int st_lsm6ds3_do_div(struct lsm6ds3_data *cdata,
					u16 read_len,
					bool discard_data,
					int64_t *accel_deltatime, int64_t *gyro_deltatime
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
					, int64_t *ext0_deltatime, int64_t *ext1_deltatime
#endif
					)
{
	u8 gyro_sip, accel_sip;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	u8 ext0_sip, ext1_sip;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	u16 byte_in_pattern = cdata->byte_in_pattern;
	u16 pattern_num;
	int64_t pattern_timestamp;

	gyro_sip = cdata->gyro_samples_in_pattern;
	accel_sip = cdata->accel_samples_in_pattern;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	ext0_sip = cdata->ext0_samples_in_pattern;
	ext1_sip = cdata->ext1_samples_in_pattern;
#endif

	if (byte_in_pattern)
		pattern_num = read_len / byte_in_pattern;
	else {
		dev_err(cdata->dev, "st_lsm6ds3_do_div byte_in_pattern equal 0\n");
		return -EINVAL;
	}
	if (pattern_num) {
		if (discard_data) {
			pattern_timestamp = accel_sip ? (*accel_deltatime * accel_sip) :
						(gyro_sip ? (*gyro_deltatime * gyro_sip) :
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
						(ext0_sip ? (*ext0_deltatime * ext0_sip) :
						(ext1_sip ? (*ext1_deltatime * ext1_sip) : 0)));
#else
						0);
#endif
			cdata->last_timestamp = cdata->timestamp - pattern_timestamp * pattern_num;
		} else {
			pattern_timestamp = (cdata->timestamp - cdata->last_timestamp) / pattern_num;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
			if (ext0_sip)
				*ext0_deltatime = pattern_timestamp / ext0_sip;
			if (ext1_sip)
				*ext1_deltatime = pattern_timestamp / ext1_sip;
#endif
			if (gyro_sip)
				*gyro_deltatime = pattern_timestamp / gyro_sip;
			if (accel_sip)
				*accel_deltatime = pattern_timestamp / accel_sip;
		}
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		cdata->ext0_timestamp = cdata->ext1_timestamp =
#endif
				cdata->gyro_timestamp = cdata->accel_timestamp = cdata->last_timestamp;

		dev_dbg(cdata->dev, "st_lsm6ds3_do_div"
					"[%d] [%d] [%d] [%d] [%d] [%lld] [%lld] [%lld] [%lld]\n",
					discard_data, gyro_sip, accel_sip, byte_in_pattern, read_len,
					*gyro_deltatime, *accel_deltatime,
					cdata->last_timestamp, cdata->timestamp);
	}

	return 0;
}

static void st_lsm6ds3_parse_fifo_data(struct lsm6ds3_data *cdata, u16 read_len, bool discard_data)
{
	u16 fifo_offset = 0;
	u8 gyro_sip, accel_sip;
	int64_t accel_deltatime = cdata->accel_deltatime;
	int64_t gyro_deltatime = cdata->gyro_deltatime;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
	u8 ext0_sip, ext1_sip;
	int64_t ext0_deltatime = cdata->ext0_deltatime;
	int64_t ext1_deltatime = cdata->ext1_deltatime;
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */

	dev_dbg(cdata->dev, "st_lsm6ds3_parse_fifo_data: sensors_enabled=0x%2x, sensors_pattern_en=0x%2x\n",
				cdata->sensors_enabled, cdata->sensors_pattern_en);
	if (read_len)
		if (st_lsm6ds3_do_div(cdata, read_len, discard_data,
				&accel_deltatime, &gyro_deltatime
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
				, &ext0_deltatime, &ext1_deltatime
#endif
				) < 0)
			return;

	while (fifo_offset < read_len) {
		gyro_sip = cdata->gyro_samples_in_pattern;
		accel_sip = cdata->accel_samples_in_pattern;
#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
		ext0_sip = cdata->ext0_samples_in_pattern;
		ext1_sip = cdata->ext1_samples_in_pattern;
#endif
		do {
			if (gyro_sip > 0) {
				if (cdata->gyro_samples_to_discard > 0)
					cdata->gyro_samples_to_discard--;
				else {
					if (cdata->sensors_pattern_en & (1 << ST_INDIO_DEV_GYRO)) {
						st_lsm6ds3_push_data_with_timestamp(
							cdata, ST_INDIO_DEV_GYRO,
							&cdata->fifo_data[fifo_offset],
							cdata->gyro_timestamp);
						dev_dbg(cdata->dev, "non-wakeup gyro [%lld]\n",
								cdata->gyro_timestamp);
					}
					if (cdata->sensors_pattern_en & (1 << ST_INDIO_DEV_GYRO_WK)) {
						st_lsm6ds3_push_data_with_timestamp(
							cdata, ST_INDIO_DEV_GYRO_WK,
							&cdata->fifo_data[fifo_offset],
							cdata->gyro_timestamp);
						dev_dbg(cdata->dev, "wakeup gyro [%lld]\n",
								cdata->gyro_timestamp);
					}
				}

				cdata->gyro_timestamp += gyro_deltatime;
				fifo_offset += ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				gyro_sip--;
			}

			if (accel_sip > 0) {
				if (cdata->accel_samples_to_discard > 0)
					cdata->accel_samples_to_discard--;
				else {
					if (cdata->sensors_pattern_en & (1 << ST_INDIO_DEV_ACCEL)) {
						st_lsm6ds3_push_data_with_timestamp(
							cdata, ST_INDIO_DEV_ACCEL,
							&cdata->fifo_data[fifo_offset],
							cdata->accel_timestamp);
						dev_dbg(cdata->dev, "non-wakeup accel [%lld]\n",
								cdata->accel_timestamp);
					}
					if (cdata->sensors_pattern_en & (1 << ST_INDIO_DEV_ACCEL_WK)) {
						st_lsm6ds3_push_data_with_timestamp(
							cdata, ST_INDIO_DEV_ACCEL_WK,
							&cdata->fifo_data[fifo_offset],
							cdata->accel_timestamp);
						dev_dbg(cdata->dev, "wakeup accel [%lld]\n",
								cdata->accel_timestamp);
					}
				}

				cdata->accel_timestamp += accel_deltatime;
				fifo_offset += ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				accel_sip--;
			}

#ifdef CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT
			if (ext0_sip > 0) {
				if (cdata->ext_samples_to_discard[0] > 0)
					cdata->ext_samples_to_discard[0]--;
				else {
					st_lsm6ds3_push_data_with_timestamp(
						cdata, ST_INDIO_DEV_EXT0,
						&cdata->fifo_data[fifo_offset],
						cdata->ext0_timestamp);
				}

				cdata->ext0_timestamp += ext0_deltatime;
				fifo_offset += ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				ext0_sip--;
			}

			if (ext1_sip > 0) {
				if (cdata->ext_samples_to_discard[1] > 0)
					cdata->ext_samples_to_discard[1]--;
				else {
					st_lsm6ds3_push_data_with_timestamp(
						cdata, ST_INDIO_DEV_EXT1,
						&cdata->fifo_data[fifo_offset],
						cdata->ext1_timestamp);
				}

				cdata->ext1_timestamp += ext1_deltatime;
				fifo_offset += ST_LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				ext1_sip--;
			}

		} while ((accel_sip > 0) || (gyro_sip > 0) ||
					(ext0_sip > 0) || (ext1_sip > 0));
#else /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
		} while ((accel_sip > 0) || (gyro_sip > 0));
#endif /* CONFIG_ST_LSM6DS3_IIO_MASTER_SUPPORT */
	}

	return;
}

void st_lsm6ds3_read_fifo(struct lsm6ds3_data *cdata, int flags)
{
	bool overrun_flag = false;
	bool want_to_discard = flags & READ_FIFO_DISCARD_DATA;
	bool discard_data = false;
	int err;
	u16 pattern, offset;
	u16 read_len = cdata->fifo_threshold, discard_len;
	u16 byte_in_pattern = cdata->byte_in_pattern;
	dev_dbg(cdata->dev, "st_lsm6ds3_read_fifo, flags=0x%2x\n", flags);

	if (!cdata->fifo)
		return;
	cdata->fifo_data = cdata->fifo;

	if (flags != READ_FIFO_IN_INTERRUPT) {
		if (byte_in_pattern == 0)
			return;

		if (!(flags & READ_FIFO_IN_INTERRUPT)) {
			cdata->last_timestamp = cdata->timestamp;
			cdata->timestamp = ktime_to_ns(ktime_get_boottime());
		}
		err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DIFF_L,
						2, (u8 *)&read_len, true);
		if (err < 0)
			return;

		if (read_len & ST_LSM6DS3_FIFO_DATA_OVR_2REGS) {
			want_to_discard = true;
			overrun_flag = true;
			dev_err(cdata->dev,
				"data fifo overrun, read_len=%d.\n", read_len);

			if ((read_len & ST_LSM6DS3_FIFO_DIFF_MASK) == 0)
				read_len = ST_LSM6DS3_FIFO_DIFF_MASK;
		}

		if (want_to_discard) {
			dev_dbg(cdata->dev,
				"want_to_discard %d, %d.\n", read_len, cdata->fifo_threshold);
			read_len &= ST_LSM6DS3_FIFO_DIFF_MASK;
			if (read_len > (cdata->fifo_threshold + byte_in_pattern) / ST_LSM6DS3_BYTE_FOR_CHANNEL) {
				discard_len = read_len - cdata->fifo_threshold / ST_LSM6DS3_BYTE_FOR_CHANNEL;
				discard_data = true;
			} else
				goto read_fifo_report;

			discard_len *= ST_LSM6DS3_BYTE_FOR_CHANNEL;
			discard_len = (discard_len / byte_in_pattern) * byte_in_pattern;

			dev_dbg(cdata->dev,
				"prepare to discard %d data.\n", discard_len);
			err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DATA_OUT_L,
							discard_len, cdata->fifo_data, true);
			if (err < 0)
				return;

			cdata->last_timestamp = cdata->timestamp;
			cdata->timestamp = ktime_to_ns(ktime_get_boottime());
			err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DIFF_L,
							2, (u8 *)&read_len, true);
			if (err < 0) {
				if (overrun_flag) {
					st_lsm6ds3_set_fifo_mode(cdata, BYPASS);
					st_lsm6ds3_set_fifo_mode(cdata, CONTINUOS);
				}
				return;
			}
			dev_dbg(cdata->dev,
				"after discard data, read_len=%d.\n", read_len);
		}

read_fifo_report:
		read_len &= ST_LSM6DS3_FIFO_DIFF_MASK;
		read_len *= ST_LSM6DS3_BYTE_FOR_CHANNEL;
		read_len = (read_len / byte_in_pattern) * byte_in_pattern;

	}

	if (read_len == 0) {
		dev_err(cdata->dev, "read_len=%d.\n", read_len);
		return;
	}
	err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DATA_OUT_L,
					read_len, cdata->fifo_data, true);
	if (err < 0) {
		dev_err(cdata->dev, " read %d err=%d.\n", read_len, err);
		return;
	}

	if (overrun_flag) {
		err = cdata->tf->read(cdata, ST_LSM6DS3_FIFO_DATA_PATTERN_L,
					2, (u8 *)&pattern, true);
		if (err < 0)
			return;
		pattern &= 0x03FF;
		/* byte_in_pattern/ST_LSM6DS3_BYTE_FOR_CHANNEL is the number
		 * of axis in a "pattern".
		 * offset is the number of bytes to be discarded when an
		 * overrun condition happens.
		 */
		offset = ((byte_in_pattern/ST_LSM6DS3_BYTE_FOR_CHANNEL) - pattern)
			* ST_LSM6DS3_BYTE_FOR_CHANNEL;
		if (offset != byte_in_pattern) {
			read_len -= byte_in_pattern;
			cdata->fifo_data += offset;
		}
		dev_info(cdata->dev, "FIFO overrun, offset=%d", offset);
		st_lsm6ds3_set_fifo_mode(cdata, BYPASS);
		st_lsm6ds3_set_fifo_mode(cdata, CONTINUOS);
	}

	st_lsm6ds3_parse_fifo_data(cdata, read_len, discard_data);
}

static irqreturn_t st_lsm6ds3_step_counter_trigger_handler(int irq, void *p)
{
	int err;
	int64_t timestamp = 0;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3_step_counter_trigger_handler\n");
	if (!sdata->cdata->reset_steps) {
		err = sdata->cdata->tf->read(sdata->cdata,
					(u8)indio_dev->channels[0].address,
					ST_LSM6DS3_BYTE_FOR_CHANNEL,
					sdata->buffer_data, true);
		if (err < 0)
			goto st_lsm6ds3_step_counter_done;

		timestamp = sdata->cdata->timestamp;
	} else {
		memset(sdata->buffer_data, 0, ST_LSM6DS3_BYTE_FOR_CHANNEL);
		timestamp = ktime_to_ns(ktime_get_boottime());
		sdata->cdata->reset_steps = false;
	}

	if (indio_dev->scan_timestamp)
		*(s64 *)((u8 *)sdata->buffer_data +
				ALIGN(ST_LSM6DS3_BYTE_FOR_CHANNEL,
						sizeof(s64))) = timestamp;
	iio_push_to_buffers(indio_dev, sdata->buffer_data);

st_lsm6ds3_step_counter_done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static inline irqreturn_t st_lsm6ds3_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}

int st_lsm6ds3_trig_set_state(struct iio_trigger *trig, bool state)
{
	int err;
	struct lsm6ds3_sensor_data *sdata;

	sdata = iio_priv(iio_trigger_get_drvdata(trig));

	err = st_lsm6ds3_set_drdy_irq(sdata, state);

	return err < 0 ? err : 0;
}

static int st_lsm6ds3_buffer_preenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	dev_dbg(sdata->cdata->dev, "st_lsm6ds3_buffer_preenable: index=%d\n", sdata->sindex);

	err = st_lsm6ds3_set_enable(sdata, true);
	if (err < 0)
		return err;

	err = st_lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	return iio_sw_buffer_preenable(indio_dev);
}

static int st_lsm6ds3_buffer_postenable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	if ((1 << sdata->sindex) & ST_LSM6DS3_USE_BUFFER) {
		sdata->buffer_data = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
		if (sdata->buffer_data == NULL)
			return -ENOMEM;
	}

	if ((1 << sdata->sindex) & ST_INDIO_DEV_AG_MASK) {
		err = st_lsm6ds3_set_axis_enable(sdata,
					(u8)indio_dev->active_scan_mask[0]);
		if (err < 0)
			goto free_buffer_data;
	}

	err = iio_triggered_buffer_postenable(indio_dev);
	if (err < 0)
		goto free_buffer_data;

	if (sdata->sindex == ST_INDIO_DEV_STEP_COUNTER) {
		sdata->cdata->timestamp = ktime_to_ns(ktime_get_boottime());
		iio_trigger_poll_chained(
			sdata->cdata->trig[ST_INDIO_DEV_STEP_COUNTER], 0);
	}

	if (sdata->sindex == ST_INDIO_DEV_SIGN_MOTION)
		sdata->cdata->sign_motion_event_ready = true;

	return 0;

free_buffer_data:
	if ((1 << sdata->sindex) & ST_LSM6DS3_USE_BUFFER)
		kfree(sdata->buffer_data);

	return err;
}

static int st_lsm6ds3_buffer_predisable(struct iio_dev *indio_dev)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = iio_priv(indio_dev);

	err = iio_triggered_buffer_predisable(indio_dev);
	if (err < 0)
		return err;

	if ((1 << sdata->sindex) & ST_INDIO_DEV_AG_MASK) {
		err = st_lsm6ds3_set_axis_enable(sdata, ST_LSM6DS3_ENABLE_AXIS);
		if (err < 0)
			return err;
	}

	if (sdata->sindex == ST_INDIO_DEV_SIGN_MOTION)
		sdata->cdata->sign_motion_event_ready = false;

	err = st_lsm6ds3_set_enable(sdata, false);
	if (err < 0)
		return err;

	err = st_lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;

	if ((1 << sdata->sindex) & ST_LSM6DS3_USE_BUFFER)
		kfree(sdata->buffer_data);

	return 0;
}

static const struct iio_buffer_setup_ops st_lsm6ds3_buffer_setup_ops = {
	.preenable = &st_lsm6ds3_buffer_preenable,
	.postenable = &st_lsm6ds3_buffer_postenable,
	.predisable = &st_lsm6ds3_buffer_predisable,
};

int st_lsm6ds3_allocate_rings(struct lsm6ds3_data *cdata)
{
	int err;

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_INDIO_DEV_ACCEL],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		return err;

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_accel;

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_INDIO_DEV_GYRO],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_accel_wk;

	err = iio_triggered_buffer_setup(cdata->indio_dev[ST_INDIO_DEV_GYRO_WK],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_gyro;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_gyro_wk;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER],
				NULL,
				&st_lsm6ds3_step_counter_trigger_handler,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_sign_motion;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_step_counter;

	err = iio_triggered_buffer_setup(
				cdata->indio_dev[ST_INDIO_DEV_TILT],
				&st_lsm6ds3_handler_empty, NULL,
				&st_lsm6ds3_buffer_setup_ops);
	if (err < 0)
		goto buffer_cleanup_step_detector;

	return 0;

buffer_cleanup_step_detector:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]);
buffer_cleanup_step_counter:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]);
buffer_cleanup_sign_motion:
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]);
buffer_cleanup_gyro_wk:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]);
buffer_cleanup_gyro:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_GYRO]);
buffer_cleanup_accel_wk:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]);
buffer_cleanup_accel:
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
	return err;
}

void st_lsm6ds3_deallocate_rings(struct lsm6ds3_data *cdata)
{
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_TILT]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_DETECTOR]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_STEP_COUNTER]);
	iio_triggered_buffer_cleanup(
				cdata->indio_dev[ST_INDIO_DEV_SIGN_MOTION]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_GYRO_WK]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_GYRO]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_ACCEL_WK]);
	iio_triggered_buffer_cleanup(cdata->indio_dev[ST_INDIO_DEV_ACCEL]);
}

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3 buffer driver");
MODULE_LICENSE("GPL v2");

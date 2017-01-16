/*
 * platform_drv2605.c: TI drv2605 vibrator platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/drv2605_vibra.h>
#include <asm/intel-mid.h>
#include "platform_drv2605.h"

/*
*	Rated Voltage:
*	Calculated using the formula r = v * 255 / 5.6
*	where r is what will be written to the register
*	and v is the rated voltage of the actuator
*
*	Overdrive Clamp Voltage:
*	Calculated using the formula o = oc * 255 / 5.6
*	where o is what will be written to the register
*	and oc is the overdrive clamp voltage of the actuator
*/

static const unsigned char ERM_autocal_sequence_prod2[] = {
	MODE_REG,			AUTO_CALIBRATION,
	RATED_VOLTAGE_REG,		0x80,
	OVERDRIVE_CLAMP_VOLTAGE_REG,	0x92,
	AUTO_CALI_RESULT_REG,		DEFAULT_ERM_AUTOCAL_COMPENSATION,
	AUTO_CALI_BACK_EMF_RESULT_REG,	DEFAULT_ERM_AUTOCAL_BACKEMF,
	FEEDBACK_CONTROL_REG,		FB_BRAKE_FACTOR_3X | LOOP_RESPONSE_MEDIUM | FEEDBACK_CONTROL_BEMF_ERM_GAIN2,
	AUTOCAL_MEM_INTERFACE_REG,	AUTOCAL_TIME_1000MS,
	GO_REG,				GO,
};

static const unsigned char ERM_sequence_prod2[] = {
	REAL_TIME_PLAYBACK_REG,		0x7F,
	LIBRARY_SELECTION_REG,		LIBRARY_A,
	WAVEFORM_SEQUENCER_REG,		WAVEFORM_SEQUENCER_DEFAULT,
	WAVEFORM_SEQUENCER_REG2,	WAVEFORM_SEQUENCER_DEFAULT,
	WAVEFORM_SEQUENCER_REG3,	WAVEFORM_SEQUENCER_DEFAULT,
	WAVEFORM_SEQUENCER_REG4,	WAVEFORM_SEQUENCER_DEFAULT,
	WAVEFORM_SEQUENCER_REG5,	WAVEFORM_SEQUENCER_DEFAULT,
	WAVEFORM_SEQUENCER_REG6,	WAVEFORM_SEQUENCER_DEFAULT,
	WAVEFORM_SEQUENCER_REG7,	WAVEFORM_SEQUENCER_DEFAULT,
	WAVEFORM_SEQUENCER_REG8,	WAVEFORM_SEQUENCER_DEFAULT,
	OVERDRIVE_TIME_OFFSET_REG,	0x00,
	SUSTAIN_TIME_OFFSET_POS_REG,	0x00,
	SUSTAIN_TIME_OFFSET_NEG_REG,	0x00,
	BRAKE_TIME_OFFSET_REG,		0x00,
	AUDIO_HAPTICS_CONTROL_REG,	AUDIO_HAPTICS_RECT_20MS | AUDIO_HAPTICS_FILTER_125HZ,
	AUDIO_HAPTICS_MIN_INPUT_REG,	AUDIO_HAPTICS_MIN_INPUT_VOLTAGE,
	AUDIO_HAPTICS_MAX_INPUT_REG,	AUDIO_HAPTICS_MAX_INPUT_VOLTAGE,
	AUDIO_HAPTICS_MIN_OUTPUT_REG,	AUDIO_HAPTICS_MIN_OUTPUT_VOLTAGE,
	AUDIO_HAPTICS_MAX_OUTPUT_REG,	AUDIO_HAPTICS_MAX_OUTPUT_VOLTAGE,
	Control1_REG,			STARTUP_BOOST_ENABLED | DEFAULT_DRIVE_TIME,
	Control2_REG,			BIDIRECT_INPUT | AUTO_RES_GAIN_MEDIUM | BLANKING_TIME_SHORT | IDISS_TIME_SHORT,
	Control3_REG,			ERM_OpenLoop_Enabled | NG_Thresh_DISABLED,
};

static const unsigned char LRA_autocal_sequence_prod1[] = {
	MODE_REG,			AUTO_CALIBRATION,
	RATED_VOLTAGE_REG,		0x5D,
	OVERDRIVE_CLAMP_VOLTAGE_REG,	0x92,
	FEEDBACK_CONTROL_REG,		FEEDBACK_CONTROL_MODE_LRA | FB_BRAKE_FACTOR_4X | LOOP_RESPONSE_FAST,
	Control3_REG,			NG_Thresh_2,
	GO_REG,				GO,
};

static void setup_parameter_sequence(struct drv2605_platform_data *pdata)
{
if (INTEL_MID_BOARD(2, PHONE, MRFL, RBY, PRO) ||
	INTEL_MID_BOARD(2, PHONE, MRFL, RBY, ENG)) {
	pdata->effect_library = LIBRARY_F; /* >> LRA close loop */
	pdata->real_time_playback = 0x7F;/* 100% of rated voltage (closed loop)*/
	pdata->repeat_autocal_sequence = true;
	pdata->size_autocal_sequence = ARRAY_SIZE(LRA_autocal_sequence_prod1);
	pdata->parameter_autocal_sequence = LRA_autocal_sequence_prod1;
} else if (INTEL_MID_BOARD(2, PHONE, MRFL, MVN, PRO) ||
	INTEL_MID_BOARD(2, PHONE, MRFL, MVN, ENG) ||
	INTEL_MID_BOARD(2, PHONE, MRFL, GLC, PRO) ||
	INTEL_MID_BOARD(2, PHONE, MRFL, GLC, ENG)) {
	pdata->effect_library = LIBRARY_A; /* ERM open loop */
	pdata->real_time_playback = 0x7F; /* ~100% of overdrive voltage (open loop)*/
	pdata->repeat_autocal_sequence = true;
	pdata->size_autocal_sequence = ARRAY_SIZE(ERM_autocal_sequence_prod2);
	pdata->parameter_autocal_sequence = ERM_autocal_sequence_prod2;
	pdata->size_sequence = ARRAY_SIZE(ERM_sequence_prod2);
	pdata->parameter_sequence = ERM_sequence_prod2;
} else { /* default setting like prod1 */
	pdata->effect_library = LIBRARY_F;
	pdata->real_time_playback = 0x7F;
	pdata->repeat_autocal_sequence = true;
	pdata->size_autocal_sequence = ARRAY_SIZE(LRA_autocal_sequence_prod1);
	pdata->parameter_autocal_sequence = LRA_autocal_sequence_prod1;
}
}

void *drv2605_platform_data(void *info)
{
	static struct drv2605_platform_data drv2605_platform_data = {
		.gpio_en = -1,
		.gpio_pwm = -1,
	};

	drv2605_platform_data.gpio_en = get_gpio_by_name("haptics_en");
	drv2605_platform_data.gpio_pwm = get_gpio_by_name("haptics_pwm");
	setup_parameter_sequence(&drv2605_platform_data);
	return &drv2605_platform_data;
}

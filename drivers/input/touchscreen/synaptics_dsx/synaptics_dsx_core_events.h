/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM synaptics_dsx

#if !defined(_SYNAPTICS_DSX_RMI4_EVENTS_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _SYNAPTICS_DSX_RMI4_EVENTS_H_

#include <linux/tracepoint.h>

TRACE_EVENT(synaptics_rmi4_wakeup_gesture,

	TP_PROTO(struct synaptics_rmi4_data *data, bool enable),

	TP_ARGS(data, enable),

	TP_STRUCT__entry(
		__field(bool, f11_wakeup_gesture)
		__field(bool, f12_wakeup_gesture)
		__field(bool, enable)
	),

	TP_fast_assign(
		__entry->f11_wakeup_gesture = data->f11_wakeup_gesture;
		__entry->f12_wakeup_gesture = data->f12_wakeup_gesture;
		__entry->enable = enable;
	),

	TP_printk("f11_wakeup_gesture=%u f12_wakeup_gesture=%u enable=%d",
		__entry->f11_wakeup_gesture,
		__entry->f12_wakeup_gesture,
		__entry->enable)
);

TRACE_EVENT(synaptics_rmi4_irq,

	TP_PROTO(int gpio_value, int irq_on_state),

	TP_ARGS(gpio_value, irq_on_state),

	TP_STRUCT__entry(
		__field(int, gpio_value)
		__field(int, irq_on_state)
	),

	TP_fast_assign(
		__entry->gpio_value = gpio_value;
		__entry->irq_on_state = irq_on_state;
	),

	TP_printk("gpio_value=%d irq_on_state=%d",
		__entry->gpio_value,
		__entry->irq_on_state)
);

TRACE_EVENT(synaptics_rmi4_report_touch,

	TP_PROTO(struct synaptics_rmi4_data *data, int fn_number),

	TP_ARGS(data, fn_number),

	TP_STRUCT__entry(
		__field(bool, fingers_on_2d)
		__field(int, fn_number)
	),

	TP_fast_assign(
		__entry->fingers_on_2d = data->fingers_on_2d;
		__entry->fn_number = fn_number;
	),

	TP_printk("fn_number=%d fingers_on_2d=%d",
		__entry->fn_number,
		__entry->fingers_on_2d)
);

DECLARE_EVENT_CLASS(synaptics_rmi4_abs_report_finger_template,

	TP_PROTO(int finger_status, int x, int y, int wx, int wy),

	TP_ARGS(finger_status, x, y, wx, wy),

	TP_STRUCT__entry(
		__field(int, finger_status)
		__field(int, x)
		__field(int, y)
		__field(int, wx)
		__field(int, wy)
	),

	TP_fast_assign(
		__entry->finger_status = finger_status;
		__entry->x = x;
		__entry->y = y;
		__entry->wx = wx;
		__entry->wy = wy;
	),

	TP_printk("finger_status=%d x=%d y=%d wx=%d wy=%d",
		__entry->finger_status,
		__entry->x,
		__entry->y,
		__entry->wx,
		__entry->wy)
);

DEFINE_EVENT(synaptics_rmi4_abs_report_finger_template, synaptics_rmi4_f11_abs_report_finger,

	TP_PROTO(int finger_status, int x, int y, int wx, int wy),

	TP_ARGS(finger_status, x, y, wx, wy)

);

DEFINE_EVENT(synaptics_rmi4_abs_report_finger_template, synaptics_rmi4_f12_abs_report_finger,

	TP_PROTO(int finger_status, int x, int y, int wx, int wy),

	TP_ARGS(finger_status, x, y, wx, wy)

);

TRACE_EVENT(synaptics_key_press,

	TP_PROTO(int key, int pressed),

	TP_ARGS(key, pressed),

	TP_STRUCT__entry(
		__field(int, key)
		__field(int, pressed)
	),

	TP_fast_assign(
		__entry->key = key;
		__entry->pressed = pressed;
	),

	TP_printk("key=%d pressed=%d",
		__entry->key,
		__entry->pressed)
);

TRACE_EVENT(synaptics_abs,

	TP_PROTO(int code, int value),

	TP_ARGS(code, value),

	TP_STRUCT__entry(
		__field(int, code)
		__field(int, value)
	),

	TP_fast_assign(
		__entry->code = code;
		__entry->value = value;
	),

	TP_printk("code=%d value=%d",
		__entry->code,
		__entry->value)
);

#endif

/* This part must be outside protection */
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE synaptics_dsx_core_events

#include <trace/define_trace.h>

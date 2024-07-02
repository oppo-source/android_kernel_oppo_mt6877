/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __REAR_ALS_H__
#define __REAR_ALS_H__

#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/uaccess.h>
//#include <linux/wakelock.h>
#include "rear_als_factory.h"
#include "sensor_attr.h"
#include "sensor_event.h"
#include <hwmsensor.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sensors_io.h>

#define REAR_ALS_TAG "<REAR_ALS> "

#define OP_REAR_ALS_DELAY 0X01
#define OP_REAR_ALS_ENABLE 0X02
#define OP_REAR_ALS_GET_DATA 0X04

#define REAR_ALS_INVALID_VALUE -1

#define EVENT_TYPE_ALS_VALUE ABS_X
#define EVENT_TYPE_ALS_STATUS ABS_WHEEL


#define REAR_ALS_VALUE_MAX (32767)
#define REAR_ALS_VALUE_MIN (-32768)
#define REAR_ALS_STATUS_MIN (0)
#define REAR_ALS_STATUS_MAX (64)
#define REAR_ALS_DIV_MAX (32767)
#define REAR_ALS_DIV_MIN (1)

#define MAX_CHOOSE_REAR_ALS_NUM 5

struct rear_als_control_path {
	int (*open_report_data)(int open); /* open data rerport to HAL */
	int (*enable_nodata)(int en); /* only enable not report event to HAL */
	int (*set_delay)(u64 delay);
	int (*batch)(int flag, int64_t samplingPeriodNs,
		int64_t maxBatchReportLatencyNs);
	int (*flush)(void);	    /* open data rerport to HAL */
	int (*set_cali)(uint8_t *data, uint8_t count);
	int (*access_data_fifo)(void);
	bool is_report_input_direct;
	bool is_support_batch;
	bool is_polling_mode;
	bool is_use_common_factory;
};


struct rear_als_data_path {
	int (*get_data)(int *rear_als_value, int *status);
	int (*rear_als_get_raw_data)(int *rear_als_value);
	int vender_div;
};


struct rear_als_init_info {
	char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver *platform_diver_addr;
};

struct rear_als_data {
	struct hwm_sensor_data rear_als_data;
	int data_updata;
};

struct rear_als_drv_obj {
	void *self;
	int polling;
	int (*rear_als_operate)(void *self, uint32_t command, void *buff_in,
			     int size_in, void *buff_out, int size_out,
			     int *actualout);
};

struct rear_als_context {
	struct input_dev *idev;
	struct sensor_attr_t rear_als_mdev;
	struct work_struct report_rear_als;
	struct mutex rear_als_op_mutex;
	struct timer_list timer_rear_als; /*rear_als polling timer */

	atomic_t trace;
	atomic_t delay_rear_als; /*rear_als polling period for reporting input event*/
	atomic_t wake;      /*user-space request to wake-up, used with stop*/

	atomic_t early_suspend;

	struct rear_als_data drv_data;
	struct rear_als_control_path rear_als_ctl;
	struct rear_als_data_path rear_als_data;
	/* Active, but HAL don't need data sensor.such as orientation need */
	bool is_rear_als_active_nodata;
	bool is_rear_als_active_data;   /* Active and HAL need data . */
	/* Active, but HAL don't need data sensor.such as orientation need */

	bool is_rear_als_first_data_after_enable;
	bool is_rear_als_polling_run;
	/* v2.judging whether sensor is in batch mode */
	bool is_rear_als_batch_enable;
	bool is_get_valid_rear_als_data_after_enable;
	int rear_als_power;
	int rear_als_enable;
	int64_t rear_als_delay_ns;
	int64_t rear_als_latency_ns;
};


/* for auto detect */
extern int rear_als_driver_add(struct rear_als_init_info *obj);
extern int rear_als_data_report(int value, int status);
extern int rear_als_data_report_t(int value, int status, int64_t time_stamp);
extern int rear_als_cali_report(int *value);
extern int rear_als_flush_report(void);
extern int rear_als_register_control_path(struct rear_als_control_path *ctl);
extern int rear_als_register_data_path(struct rear_als_data_path *data);
extern struct platform_device *get_rear_als_platformdev(void);
#endif

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __REAR_ALS_FACTORY_H__
#define __REAR_ALS_FACTORY_H__

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <hwmsen_helper.h>
#include <hwmsensor.h>
#include <sensors_io.h>

/*#include <mach/mt_typedefs.h>*/
/*#include <mach/mt_gpio.h>*/
/*#include <mach/mt_pm_ldo.h>*/

#include "rear_als.h"

struct rear_als_factory_fops {
	int (*rear_als_enable_sensor)(bool enable_disable,
				 int64_t sample_periods_ms);
	int (*rear_als_get_data)(int32_t *data);
	int (*rear_als_get_raw_data)(int32_t *data);
	int (*rear_als_enable_calibration)(void);
	int (*rear_als_clear_cali)(void);
	int (*rear_als_set_cali)(int32_t offset);
	int (*rear_als_get_cali)(int32_t offset[6]);
};

struct rear_als_factory_public {
	uint32_t gain;
	uint32_t sensitivity;
	struct rear_als_factory_fops *fops;
};
int rear_als_factory_device_register(struct rear_als_factory_public *dev);
int rear_als_factory_device_deregister(struct rear_als_factory_public *dev);
#endif

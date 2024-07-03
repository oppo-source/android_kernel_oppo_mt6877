/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 * Definitions for REAR_ALS  sensor chip.
 */
#ifndef __REAR_ALSHUB_H__
#define __REAR_ALSHUB_H__

#include <linux/ioctl.h>


/*REAR_ALS related driver tag macro*/
#define REAR_ALS_SUCCESS                         0
#define REAR_ALS_ERR_I2C                        -1
#define REAR_ALS_ERR_STATUS                     -3
#define REAR_ALS_ERR_SETUP_FAILURE              -4
#define REAR_ALS_ERR_GETGSENSORDATA             -5
#define REAR_ALS_ERR_IDENTIFICATION             -6

#endif


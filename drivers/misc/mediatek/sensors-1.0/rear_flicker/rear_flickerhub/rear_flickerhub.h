/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 * Definitions for REAR_FLICKER  sensor chip.
 */
#ifndef __REAR_FLICKERHUB_H__
#define __REAR_FLICKERHUB_H__

#include <linux/ioctl.h>


/*REAR_FLICKER related driver tag macro*/
#define REAR_FLICKER_SUCCESS                         0
#define REAR_FLICKER_ERR_I2C                        -1
#define REAR_FLICKER_ERR_STATUS                     -3
#define REAR_FLICKER_ERR_SETUP_FAILURE              -4
#define REAR_FLICKER_ERR_GETGSENSORDATA             -5
#define REAR_FLICKER_ERR_IDENTIFICATION             -6

#endif


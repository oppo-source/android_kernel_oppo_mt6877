/* SPDX-License-Identifier: GPL-2.0 */
/*
* Copyright (c) 2019 MediaTek Inc.
*/

#ifndef _OCP2130_DRV_H_
#define _OCP2130_DRV_H_

//extern int ocp2130_read_byte(unsigned char cmd, unsigned char *returnData);
extern int _bias_ic_i2c_write_bytes(unsigned char cmd, unsigned char writeData);
extern int _bias_ic_i2c_read_bytes(unsigned char cmd, unsigned char *returnData);
extern void _bias_ic_i2c_panel_bias_enable(unsigned int power_status);

#endif

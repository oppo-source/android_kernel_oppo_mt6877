/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 HI1634mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define PFX "HI1634Q_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imgsensor_eeprom.h"
#include "imgsensor_common.h"
#include "imgsensor_hwcfg_custom.h"
#include "imgsensor_i2c.h"

#include "hi1634qmipiraw_Sensor.h"

#define USE_BURST_MODE 1
#define ENABLE_PDAF 0

#define FOUR_CELL_QSC_START 0x0E1E
#define FOUR_CELL_QSC_END 0x0FFF
#define FOUR_CELL_QSC_CHECKSUM 0x1001
#define FOUR_CELL_QSC_SIZE 480

#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */

extern struct CAMERA_DEVICE_INFO gImgEepromInfo;
static kal_uint8 deviceInfo_register_value = 0x00;
extern enum IMGSENSOR_RETURN Eeprom_DataInit(
            enum IMGSENSOR_SENSOR_IDX sensor_idx,
            kal_uint32 sensorID);

static kal_uint16 test_pattern_save = 0;

#if USE_BURST_MODE
static kal_uint16 hi1634_table_write_cmos_sensor(
		kal_uint16 *para, kal_uint32 len);
#endif
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI1634Q_SENSOR_ID23687,

	.checksum_value = 0xffffffff,
	.pre = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3754,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 339200000,	//(848*4/10)
	},
	.cap = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3754,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 339200000,	//(848*4/10)
	},
	.normal_video = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3754,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1300,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 339200000,	//(848*4/10)
	},
	.hs_video = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 938,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
		.mipi_pixel_rate = 339200000,	//(848*4/10)
	},
    .slim_video = {
		.pclk = 80000000,
		.linelength = 710,
		.framelength = 3754,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 339200000,	//(848*4/10)
    },
     .custom1 = {//Bining 2304x1728@30fps
		 .pclk = 80000000,
		 .linelength = 710,
		 .framelength = 3754,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 2304,
		 .grabwindow_height = 1728,
		 .mipi_data_lp2hs_settle_dc = 85,
		 .max_framerate = 300,
		 .mipi_pixel_rate = 339200000,	 //(848*4/10)
     },
      .custom2 = {//Bining 2304x1728@30fps
		 .pclk = 80000000,
		 .linelength = 710,
		 .framelength = 3754,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 2304,
		 .grabwindow_height = 1728,
		 .mipi_data_lp2hs_settle_dc = 85,
		 .max_framerate = 300,
		 .mipi_pixel_rate = 339200000,	 //(848*4/10)
     },
      .custom3 = {//Quad 4656x3504@30fps
		 .pclk = 80000000,
		 .linelength = 710,
		 .framelength = 3754,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 4608,
		 .grabwindow_height = 3456,
		 .mipi_data_lp2hs_settle_dc = 85,
		 .max_framerate = 300,
		 .mipi_pixel_rate = 678400000,	 //(1696*4/10)
     },

	.margin = 4,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */
	.min_gain = 66,	// 66*16=1056
	.max_gain = 16 * BASEGAIN,
	.min_gain_iso = 100,
	.exp_step = 1,
	.gain_step = 4, // 4*16=64
	.gain_type = 3,
	.max_frame_length = 0xffffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 0,/* 1, support; 0,not support */
	.sensor_mode_num = 8,	/* support sensor mode num */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
	.frame_time_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_8MA, //ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2, // MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x40, 0xff},
	.i2c_speed = 1000,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x100,	/* current shutter */
	.gain = 0xe0,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x40, /* record current sensor's i2c write id */
};

static kal_uint16 read_eeprom(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA2);
	return get_byte;
}

static void read_4cell_data_from_eeprom(char *data)
{
	int i = 0x0;
	int j = 0;
	int sum = 0 , checksum = 0;

	for(i = FOUR_CELL_QSC_START,j = 0; i <= FOUR_CELL_QSC_END; i++,j++) {
		data[j] = read_eeprom(i);
		sum = (sum + data[j]) % 255;
	}
#if 0
	printk("-----------QSC_START-----------");
	for(j = 0; j <= FOUR_CELL_QSC_END - FOUR_CELL_QSC_START; j++) {
		printk("QSC_NO_%d = %d" , j , data[j]);
	}
	printk("-----------QSC_END-----------");
#endif
	checksum = read_eeprom(FOUR_CELL_QSC_CHECKSUM);
	printk("QSC_TAG: QSC data sum = %d , QSC checksum in reg = %d", sum , checksum);
}

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] = {
	{ 4656, 3504,	0,	 0,	4656, 3504,  2328, 1752,  12,   12,	 2304, 1728, 0, 0, 2304, 1728}, 	// preview (2304 x 1728)
	{ 4656, 3504,	0,	 0,	4656, 3504,  2328, 1752,  12,   12,	 2304, 1728, 0, 0, 2304, 1728},   	// capture (2304 x 1728)
	{ 4656, 3504,	0,	 0,	4656, 3504,  2328, 1752,   0,  214,	 2304, 1300, 0, 0, 2304, 1300}, 	// video (2304 x 1300)
	{ 4656, 3504,	0,   0,	4656, 3504,  2328, 1752, 524,  516,	 1280,  720, 0, 0, 1280,  720},  	// hs video (1280 x 720)
    { 4656, 3504,	0,   0,	4656, 3504,  2328, 1752, 204,  336,	 1920, 1080, 0, 0, 1920, 1080}, 	// slim video (1920 x 1080)
	{ 4656, 3504,	0,	 0,	4656, 3504,  2328, 1752,  12,   12,	 2304, 1728, 0, 0, 2304, 1728},  	// custom1 (2304 x 1728)
	{ 4656, 3504,	0,	 0,	4656, 3504,  2328, 1752,  12,   12,	 2304, 1728, 0, 0, 2304, 1728},   	// custom2 (2304 x 1728)
	{ 4656, 3504,	0,	 0,	4656, 3504,  4656, 3504,  24,   24,  4608, 3456, 0, 0, 4608, 3456}, 	// custom3 (4608 x 3456)

};


static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_word = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_word, 1, imgsensor.i2c_write_id);
	return get_word;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF),
			      (char)(para & 0xFF) };

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
			     (char)(para >> 8), (char)(para & 0xFF)};

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/
	/* Add this func to set i2c speed by each sensor */
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	pr_debug("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x020e, imgsensor.frame_length);
	write_cmos_sensor(0x0206, imgsensor.line_length);

}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	pr_debug(
		"framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
				/ imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x020e, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length*/
		write_cmos_sensor(0x020e, imgsensor.frame_length);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x020D, (shutter & 0xFF0000) >> 16);
	write_cmos_sensor(0x020A, shutter);

	pr_debug("shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}	/*	write_shutter  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
} /* set_shutter */


/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter,
				     kal_uint16 frame_length,
				     kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;

	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
				imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x020e, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length */
			write_cmos_sensor(0x020e, imgsensor.frame_length);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x020D, (shutter & 0xFF0000) >> 16 );
	write_cmos_sensor(0x020A, shutter);

    pr_debug("frame_length = %d , shutter = %d \n", imgsensor.frame_length, shutter);

}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0000;
    reg_gain = gain / 4 - 16;

    return (kal_uint16)reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain, max_gain = 16 * BASEGAIN;

	if (gain < BASEGAIN || gain > max_gain) {
		pr_debug("Error max gain setting: %d\n", max_gain);

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > max_gain)
			gain = max_gain;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("gain = %d, reg_gain = 0x%x, max_gain:0x%x\n ",
		gain, reg_gain, max_gain);

	reg_gain = reg_gain & 0x00FF;
	write_cmos_sensor_8(0x0213, reg_gain);

	return gain;
} /* set_gain */

/*
static void set_mirror_flip(kal_uint8 image_mirror)
{
	pr_debug("image_mirror = %d", image_mirror);

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0C34, 0x0000);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0C34, 0x0100);

		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0C34, 0x0200);

		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0C34, 0x0300);

		break;
	default:
		pr_debug("Error image_mirror setting");
		break;
	}

}
*/

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable)
		write_cmos_sensor(0x0b00, 0x0100); // stream on
	else
		write_cmos_sensor(0x0b00, 0x0000); // stream off

	mdelay(10);
	return ERROR_NONE;
}

#if USE_BURST_MODE
#define MULTI_WRITE 1

#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 hi1634_table_write_cmos_sensor(kal_uint16 *para,
						 kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
		/* Write when remain buffer size is less than 3 bytes
		 * or reach end of data
		 */
#if MULTI_WRITE
		if ((I2C_BUFFER_LEN - tosend) < 4
			|| IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
						tosend,
						imgsensor.i2c_write_id,
						4,
						imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 4, imgsensor.i2c_write_id);
		tosend = 0;
#endif

	}
	return 0;
}
#endif

//Hi1634Q_2.0.11.13_gain_cal_1696Mbps_20221107_MTK_MIPI.tsf
static kal_uint16 hi1634_init_setting_tacoo[] = {
0x0790, 0x0100,
0x2000, 0x1001,
0x2002, 0x0000,
0x2006, 0x40B2,
0x2008, 0xB038,
0x200A, 0x8430,
0x200C, 0x40B2,
0x200E, 0xB082,
0x2010, 0x8476,
0x2012, 0x40B2,
0x2014, 0xB0F4,
0x2016, 0x847A,
0x2018, 0x40B2,
0x201A, 0xB128,
0x201C, 0x8434,
0x201E, 0x40B2,
0x2020, 0xB19C,
0x2022, 0x8488,
0x2024, 0x40B2,
0x2026, 0xB236,
0x2028, 0x8486,
0x202A, 0x40B2,
0x202C, 0xB43E,
0x202E, 0x871E,
0x2030, 0x40B2,
0x2032, 0xB3B8,
0x2034, 0x86C8,
0x2036, 0x4130,
0x2038, 0x120B,
0x203A, 0x120A,
0x203C, 0x403B,
0x203E, 0x0261,
0x2040, 0x4B6A,
0x2042, 0xC3EB,
0x2044, 0x0000,
0x2046, 0x1292,
0x2048, 0xD000,
0x204A, 0x4ACB,
0x204C, 0x0000,
0x204E, 0xB3EB,
0x2050, 0x0000,
0x2052, 0x2411,
0x2054, 0x421F,
0x2056, 0x85DA,
0x2058, 0xD21F,
0x205A, 0x85D8,
0x205C, 0x930F,
0x205E, 0x2404,
0x2060, 0x40F2,
0x2062, 0xFF80,
0x2064, 0x0619,
0x2066, 0x3C07,
0x2068, 0x90F2,
0x206A, 0x0011,
0x206C, 0x0619,
0x206E, 0x2803,
0x2070, 0x50F2,
0x2072, 0xFFF0,
0x2074, 0x0619,
0x2076, 0x40B2,
0x2078, 0xB3BE,
0x207A, 0x86D0,
0x207C, 0x413A,
0x207E, 0x413B,
0x2080, 0x4130,
0x2082, 0x120B,
0x2084, 0x120A,
0x2086, 0x8231,
0x2088, 0x430A,
0x208A, 0x93C2,
0x208C, 0x0C0A,
0x208E, 0x2404,
0x2090, 0xB3D2,
0x2092, 0x0B05,
0x2094, 0x2401,
0x2096, 0x431A,
0x2098, 0x403B,
0x209A, 0x8438,
0x209C, 0x422D,
0x209E, 0x403E,
0x20A0, 0x192A,
0x20A2, 0x403F,
0x20A4, 0x86EC,
0x20A6, 0x12AB,
0x20A8, 0x422D,
0x20AA, 0x403E,
0x20AC, 0x86EC,
0x20AE, 0x410F,
0x20B0, 0x12AB,
0x20B2, 0x930A,
0x20B4, 0x2003,
0x20B6, 0xD3D2,
0x20B8, 0x1921,
0x20BA, 0x3C09,
0x20BC, 0x403D,
0x20BE, 0x0200,
0x20C0, 0x422E,
0x20C2, 0x403F,
0x20C4, 0x86EC,
0x20C6, 0x1292,
0x20C8, 0x8448,
0x20CA, 0xC3D2,
0x20CC, 0x1921,
0x20CE, 0x1292,
0x20D0, 0xD046,
0x20D2, 0x403B,
0x20D4, 0x8438,
0x20D6, 0x422D,
0x20D8, 0x410E,
0x20DA, 0x403F,
0x20DC, 0x86EC,
0x20DE, 0x12AB,
0x20E0, 0x422D,
0x20E2, 0x403E,
0x20E4, 0x86EC,
0x20E6, 0x403F,
0x20E8, 0x192A,
0x20EA, 0x12AB,
0x20EC, 0x5231,
0x20EE, 0x413A,
0x20F0, 0x413B,
0x20F2, 0x4130,
0x20F4, 0x4382,
0x20F6, 0x052C,
0x20F8, 0x4F0D,
0x20FA, 0x930D,
0x20FC, 0x3402,
0x20FE, 0xE33D,
0x2100, 0x531D,
0x2102, 0xF03D,
0x2104, 0x07F0,
0x2106, 0x4D0E,
0x2108, 0xC312,
0x210A, 0x100E,
0x210C, 0x110E,
0x210E, 0x110E,
0x2110, 0x110E,
0x2112, 0x930F,
0x2114, 0x3803,
0x2116, 0x4EC2,
0x2118, 0x052C,
0x211A, 0x3C04,
0x211C, 0x4EC2,
0x211E, 0x052D,
0x2120, 0xE33D,
0x2122, 0x531D,
0x2124, 0x4D0F,
0x2126, 0x4130,
0x2128, 0x120B,
0x212A, 0x425F,
0x212C, 0x0205,
0x212E, 0xC312,
0x2130, 0x104F,
0x2132, 0x114F,
0x2134, 0x114F,
0x2136, 0x114F,
0x2138, 0x114F,
0x213A, 0x114F,
0x213C, 0x4F0B,
0x213E, 0xF31B,
0x2140, 0x5B0B,
0x2142, 0x5B0B,
0x2144, 0x5B0B,
0x2146, 0x503B,
0x2148, 0xD1CC,
0x214A, 0x1292,
0x214C, 0xD004,
0x214E, 0x93C2,
0x2150, 0x86BF,
0x2152, 0x240B,
0x2154, 0xB2E2,
0x2156, 0x0400,
0x2158, 0x2008,
0x215A, 0x425F,
0x215C, 0x86BB,
0x215E, 0xD36F,
0x2160, 0xF37F,
0x2162, 0x5F0F,
0x2164, 0x5F0B,
0x2166, 0x4BA2,
0x2168, 0x0402,
0x216A, 0x93C2,
0x216C, 0x86C1,
0x216E, 0x2414,
0x2170, 0x421F,
0x2172, 0x86C6,
0x2174, 0x4FA2,
0x2176, 0x8606,
0x2178, 0x425F,
0x217A, 0x86BD,
0x217C, 0x425E,
0x217E, 0x86BA,
0x2180, 0x5F0F,
0x2182, 0x5E0F,
0x2184, 0x5F0F,
0x2186, 0x4F0E,
0x2188, 0x521E,
0x218A, 0x86CA,
0x218C, 0x4EA2,
0x218E, 0x8600,
0x2190, 0x521F,
0x2192, 0x86CC,
0x2194, 0x4FA2,
0x2196, 0x8602,
0x2198, 0x413B,
0x219A, 0x4130,
0x219C, 0x8231,
0x219E, 0xD3D2,
0x21A0, 0x7A12,
0x21A2, 0xC3D2,
0x21A4, 0x0F00,
0x21A6, 0x422D,
0x21A8, 0x403E,
0x21AA, 0x06D6,
0x21AC, 0x410F,
0x21AE, 0x1292,
0x21B0, 0x8438,
0x21B2, 0x93C2,
0x21B4, 0x86C1,
0x21B6, 0x243B,
0x21B8, 0x421F,
0x21BA, 0x0402,
0x21BC, 0x0B00,
0x21BE, 0x7304,
0x21C0, 0x0000,
0x21C2, 0x4F82,
0x21C4, 0x0402,
0x21C6, 0x421F,
0x21C8, 0x7100,
0x21CA, 0xF03F,
0x21CC, 0x0003,
0x21CE, 0x0800,
0x21D0, 0x7A10,
0x21D2, 0x931F,
0x21D4, 0x2425,
0x21D6, 0x931F,
0x21D8, 0x281C,
0x21DA, 0x932F,
0x21DC, 0x2414,
0x21DE, 0x903F,
0x21E0, 0x0003,
0x21E2, 0x240B,
0x21E4, 0x425E,
0x21E6, 0x86BB,
0x21E8, 0xEE0F,
0x21EA, 0xF31F,
0x21EC, 0x5F0F,
0x21EE, 0x4F1F,
0x21F0, 0xB466,
0x21F2, 0x9382,
0x21F4, 0x7112,
0x21F6, 0x27E2,
0x21F8, 0x3C1C,
0x21FA, 0x41A2,
0x21FC, 0x06D6,
0x21FE, 0x4192,
0x2200, 0x0002,
0x2202, 0x06D8,
0x2204, 0x3FEF,
0x2206, 0x4192,
0x2208, 0x0002,
0x220A, 0x06DA,
0x220C, 0x41A2,
0x220E, 0x06DC,
0x2210, 0x3FE9,
0x2212, 0x4192,
0x2214, 0x0004,
0x2216, 0x06DA,
0x2218, 0x4192,
0x221A, 0x0006,
0x221C, 0x06DC,
0x221E, 0x3FE2,
0x2220, 0x4192,
0x2222, 0x0006,
0x2224, 0x06D6,
0x2226, 0x4192,
0x2228, 0x0004,
0x222A, 0x06D8,
0x222C, 0x3FDB,
0x222E, 0x1292,
0x2230, 0xD058,
0x2232, 0x5231,
0x2234, 0x4130,
0x2236, 0x93C2,
0x2238, 0x86C1,
0x223A, 0x2427,
0x223C, 0x430C,
0x223E, 0x4C0F,
0x2240, 0x5F0F,
0x2242, 0x5F0F,
0x2244, 0x5F0F,
0x2246, 0x5F0F,
0x2248, 0x5F0F,
0x224A, 0x4F1D,
0x224C, 0x84C4,
0x224E, 0x4F1E,
0x2250, 0x84C6,
0x2252, 0x4F9F,
0x2254, 0x84C0,
0x2256, 0x84C4,
0x2258, 0x4F9F,
0x225A, 0x84C2,
0x225C, 0x84C6,
0x225E, 0x4D8F,
0x2260, 0x84C0,
0x2262, 0x4E8F,
0x2264, 0x84C2,
0x2266, 0x4F1D,
0x2268, 0x84CC,
0x226A, 0x4F1E,
0x226C, 0x84CE,
0x226E, 0x4F9F,
0x2270, 0x84C8,
0x2272, 0x84CC,
0x2274, 0x4F9F,
0x2276, 0x84CA,
0x2278, 0x84CE,
0x227A, 0x4D8F,
0x227C, 0x84C8,
0x227E, 0x4E8F,
0x2280, 0x84CA,
0x2282, 0x531C,
0x2284, 0x903C,
0x2286, 0x0005,
0x2288, 0x3BDA,
0x228A, 0x1292,
0x228C, 0xD056,
0x228E, 0x4130,
0x2290, 0x7400,
0x2292, 0x8058,
0x2294, 0x1807,
0x2296, 0x00E0,
0x2298, 0x7002,
0x229A, 0x17C7,
0x229C, 0x7000,
0x229E, 0x1305,
0x22A0, 0x0006,
0x22A2, 0x001F,
0x22A4, 0x0055,
0x22A6, 0x00DB,
0x22A8, 0x0012,
0x22AA, 0x1754,
0x22AC, 0x206F,
0x22AE, 0x009E,
0x22B0, 0x00DD,
0x22B2, 0x5023,
0x22B4, 0x00DE,
0x22B6, 0x005B,
0x22B8, 0x0119,
0x22BA, 0x0390,
0x22BC, 0x00D1,
0x22BE, 0x0055,
0x22C0, 0x0040,
0x22C2, 0x0553,
0x22C4, 0x0456,
0x22C6, 0x5041,
0x22C8, 0x700D,
0x22CA, 0x2F99,
0x22CC, 0x2318,
0x22CE, 0x005C,
0x22D0, 0x7000,
0x22D2, 0x1586,
0x22D4, 0x0001,
0x22D6, 0x2032,
0x22D8, 0x0012,
0x22DA, 0x0008,
0x22DC, 0x0343,
0x22DE, 0x0148,
0x22E0, 0x2123,
0x22E2, 0x0046,
0x22E4, 0x05DD,
0x22E6, 0x00DE,
0x22E8, 0x00DD,
0x22EA, 0x00DC,
0x22EC, 0x00DE,
0x22EE, 0x07D6,
0x22F0, 0x5061,
0x22F2, 0x704F,
0x22F4, 0x2F99,
0x22F6, 0x005C,
0x22F8, 0x5080,
0x22FA, 0x4D90,
0x22FC, 0x50A1,
0x22FE, 0x2122,
0x2300, 0x7800,
0x2302, 0xC08C,
0x2304, 0x0001,
0x2306, 0x9038,
0x2308, 0x59F7,
0x230A, 0x903B,
0x230C, 0x121C,
0x230E, 0x9034,
0x2310, 0x1218,
0x2312, 0x8C34,
0x2314, 0x0180,
0x2316, 0x8DC0,
0x2318, 0x01C0,
0x231A, 0x7400,
0x231C, 0x8058,
0x231E, 0x1807,
0x2320, 0x00E0,
0x2322, 0x00DF,
0x2324, 0x0047,
0x2326, 0x7000,
0x2328, 0x17C5,
0x232A, 0x0046,
0x232C, 0x0095,
0x232E, 0x7000,
0x2330, 0x148C,
0x2332, 0x005B,
0x2334, 0x0014,
0x2336, 0x001D,
0x2338, 0x216F,
0x233A, 0x005E,
0x233C, 0x00DD,
0x233E, 0x2244,
0x2340, 0x001C,
0x2342, 0x00DE,
0x2344, 0x005B,
0x2346, 0x0519,
0x2348, 0x0150,
0x234A, 0x0091,
0x234C, 0x00D5,
0x234E, 0x0040,
0x2350, 0x0393,
0x2352, 0x0356,
0x2354, 0x5021,
0x2356, 0x700D,
0x2358, 0x2F99,
0x235A, 0x2318,
0x235C, 0x005C,
0x235E, 0x0006,
0x2360, 0x0016,
0x2362, 0x425A,
0x2364, 0x0012,
0x2366, 0x0008,
0x2368, 0x0403,
0x236A, 0x01C8,
0x236C, 0x2123,
0x236E, 0x0046,
0x2370, 0x095D,
0x2372, 0x00DE,
0x2374, 0x00DD,
0x2376, 0x00DC,
0x2378, 0x00DE,
0x237A, 0x04D6,
0x237C, 0x5041,
0x237E, 0x704F,
0x2380, 0x2F99,
0x2382, 0x7000,
0x2384, 0x1702,
0x2386, 0x202C,
0x2388, 0x0016,
0x238A, 0x5060,
0x238C, 0x2122,
0x238E, 0x7800,
0x2390, 0xC08C,
0x2392, 0x0001,
0x2394, 0x903B,
0x2396, 0x121C,
0x2398, 0x9034,
0x239A, 0x1218,
0x239C, 0x8DC0,
0x239E, 0x01C0,
0x23A0, 0x0000,
0x23A2, 0xB290,
0x23A4, 0x0000,
0x23A6, 0xB290,
0x23A8, 0xB302,
0x23AA, 0x0002,
0x23AC, 0x0000,
0x23AE, 0xB31A,
0x23B0, 0x0000,
0x23B2, 0xB31A,
0x23B4, 0xB390,
0x23B6, 0x0002,
0x23B8, 0xB3A0,
0x23BA, 0xB3AC,
0x23BC, 0xFCE0,
0x23BE, 0x0040,
0x23C0, 0x0040,
0x23C2, 0x0040,
0x23C4, 0x0045,
0x23C6, 0x004C,
0x23C8, 0x0050,
0x23CA, 0x005A,
0x23CC, 0x005D,
0x23CE, 0x0064,
0x23D0, 0x0066,
0x23D2, 0x0068,
0x23D4, 0x0071,
0x23D6, 0x0078,
0x23D8, 0x007D,
0x23DA, 0x0087,
0x23DC, 0x008C,
0x23DE, 0x0094,
0x23E0, 0x0098,
0x23E2, 0x00AD,
0x23E4, 0x00B0,
0x23E6, 0x00C3,
0x23E8, 0x00C4,
0x23EA, 0x00D9,
0x23EC, 0x00DE,
0x23EE, 0x00F0,
0x23F0, 0x00FF,
0x23F2, 0x0106,
0x23F4, 0x011A,
0x23F6, 0x0117,
0x23F8, 0x0133,
0x23FA, 0x0126,
0x23FC, 0x0126,
0x23FE, 0x0040,
0x2400, 0x0040,
0x2402, 0x0040,
0x2404, 0x0045,
0x2406, 0x004C,
0x2408, 0x0050,
0x240A, 0x005A,
0x240C, 0x005D,
0x240E, 0x0064,
0x2410, 0x0066,
0x2412, 0x006B,
0x2414, 0x0071,
0x2416, 0x0078,
0x2418, 0x007D,
0x241A, 0x0087,
0x241C, 0x008C,
0x241E, 0x0094,
0x2420, 0x0098,
0x2422, 0x00AD,
0x2424, 0x00B0,
0x2426, 0x00C3,
0x2428, 0x00C4,
0x242A, 0x00D9,
0x242C, 0x00DE,
0x242E, 0x00F0,
0x2430, 0x00FF,
0x2432, 0x0106,
0x2434, 0x011A,
0x2436, 0x0117,
0x2438, 0x0133,
0x243A, 0x0126,
0x243C, 0x0126,
0x243E, 0x0041,
0x2440, 0x0060,
0x2442, 0x0124,
0x2444, 0x023B,
0x2446, 0x05FC,
0x2448, 0x0041,
0x244A, 0x0060,
0x244C, 0x0124,
0x244E, 0x023B,
0x2450, 0x05FC,
0x2452, 0x0041,
0x2454, 0x004E,
0x2456, 0x007D,
0x2458, 0x01B8,
0x245A, 0x05FC,
0x245C, 0x0041,
0x245E, 0x004B,
0x2460, 0x0078,
0x2462, 0x019D,
0x2464, 0x05FC,
0x2466, 0x72D8,
0x2468, 0x278D,
0x0262, 0x0600,
0x026A, 0xFFFF,
0x026C, 0x00FF,
0x026E, 0x0000,
0x0360, 0x0E8E,
0x040C, 0x01EB,
0x0600, 0x1132,
0x0604, 0x8008,
0x0644, 0x07FE,
0x0676, 0x07FF,
0x0678, 0x0002,
0x06A8, 0x0350,
0x06AA, 0x0160,
0x06AC, 0x0041,
0x06AE, 0x03FC,
0x06B4, 0x3FFF,
0x06CC, 0x00FF,
0x06E2, 0xFF00,
0x052A, 0x0000,
0x052C, 0x0000,
0x0F00, 0x0000,
0x0B20, 0x0100,
0x1102, 0x0008,
0x1106, 0x0124,
0x11C2, 0x0400,
0x0902, 0x0003,
0x0904, 0x0003,
0x0912, 0x0303,
0x0914, 0x0300,
0x0A04, 0xB4C5,
0x0A06, 0xC400,
0x0A08, 0xA881,
0x0A0E, 0xFEC0,
0x0A12, 0x0000,
0x0A18, 0x0010,
0x0A20, 0x0015,
0x070C, 0x0000,
0x0780, 0x010E,
0x1202, 0x1E00,
0x1204, 0xD700,
0x1210, 0x8028,
0x1216, 0xA0A0,
0x1218, 0x00A0,
0x121A, 0x0000,
0x121C, 0x4128,
0x121E, 0x0000,
0x1220, 0x0000,
0x1222, 0x28FA,
0x105C, 0x0F0B,
0x1958, 0x0041,
0x195A, 0x0060,
0x195C, 0x0124,
0x195E, 0x023B,
0x1960, 0x05FC,
0x1962, 0x0041,
0x1964, 0x0060,
0x1966, 0x0124,
0x1968, 0x023B,
0x196A, 0x05FC,
0x196C, 0x0041,
0x196E, 0x004E,
0x1970, 0x007D,
0x1972, 0x01B8,
0x1974, 0x05FC,
0x1976, 0x0041,
0x1978, 0x004B,
0x197A, 0x0078,
0x197C, 0x019D,
0x197E, 0x05FC,
0x1980, 0x0082,
0x1982, 0x001F,
0x1984, 0x2006,
0x1986, 0x0031,
0x1988, 0x0308,
0x198A, 0x0000,
0x198C, 0x0F86,
0x198E, 0x0000,
0x1990, 0x310D,
0x1992, 0x0000,
0x1994, 0x3E88,
0x1996, 0x0002,
0x19C0, 0x0082,
0x19C2, 0x001F,
0x19C4, 0x2006,
0x19C6, 0x0031,
0x19C8, 0x0308,
0x19CA, 0x0000,
0x19CC, 0x0F86,
0x19CE, 0x0000,
0x19D0, 0x310D,
0x19D2, 0x0000,
0x19D4, 0x3E88,
0x19D6, 0x0002,
0x1A00, 0x0071,
0x1A02, 0x0053,
0x1A04, 0x0000,
0x1A06, 0x0016,
0x1A08, 0x01B7,
0x1A0A, 0x0000,
0x1A0C, 0x034B,
0x1A0E, 0x0000,
0x1A10, 0x171E,
0x1A12, 0x0000,
0x1A14, 0x0999,
0x1A16, 0x0002,
0x1A40, 0x008A,
0x1A42, 0x005E,
0x1A44, 0x0000,
0x1A46, 0x0012,
0x1A48, 0x01E5,
0x1A4A, 0x0000,
0x1A4C, 0x03CE,
0x1A4E, 0x0000,
0x1A50, 0x1853,
0x1A52, 0x0000,
0x1A54, 0x0030,
0x1A56, 0x0000,
0x19BC, 0x2000,
0x19FC, 0x2000,
0x1A3C, 0x2000,
0x1A7C, 0x2000,
0x361C, 0x0000,
0x027E, 0x0100,
};


static kal_uint16 hi1634_capture_setting_tacoo[] = {
//DISP_WIDTH = 2304
//DISP_HEIGHT = 1728
//DISP_NOTE = "Q2SUM_2304x1728_30fps"
//MIPI_SPEED = 848.00
//MIPI_LANE = 4
//DISP_DATAORDER = GR
//MODE_INDEX = 4

////////////////////////////////////////////
// VT CLK: 80.00MHz
// Line length: 710
// Frame length: 3754
// Frame rate: 30.01
// ISP CLK: 169.60MHz
// V-Blank: 17980.75us
// Output bitwidth: 10 bits
// PD output bitwidth: 10 bits
////////////////////////////////////////////

0x0B00, 0x0000,
0x0204, 0x0408,
0x0206, 0x02C6,	// 2: tg/line_length_pck
0x020A, 0x0EA6,	// 3: tg/coarse_integ_time
0x020E, 0x0EAA,	// 4: tg/frame_length_lines
0x0224, 0x0044,
0x022A, 0x0016,
0x022C, 0x0E2A,
0x022E, 0x0DCA,
0x0234, 0x2222,
0x0236, 0x2222,
0x0238, 0x2222,
0x023A, 0x2222,
0x0268, 0x0108,
0x0400, 0x0E10,
0x0404, 0x0008,
0x0406, 0x1244,
0x0408, 0x0001,
0x040E, 0x0200,
0x0440, 0x011D,
0x0D00, 0x4000,
0x0D28, 0x0004,
0x0D2A, 0x0923,
0x0602, 0x3112,
0x0608, 0x0248,
0x067A, 0x0303,
0x067C, 0x0303,
0x06DE, 0x0303,
0x06E0, 0x0303,
0x06E4, 0x8A00,
0x06E6, 0x8A00,
0x06E8, 0x8A00,
0x06EA, 0x8A00,
0x0524, 0x5858,
0x0526, 0x5858,
0x0F04, 0x0010,
0x0F06, 0x0002,
0x0F08, 0x0011,
0x0F0A, 0x2233,
0x0B04, 0x009C,
0x0B12, 0x0900,
0x0B14, 0x06C0,
0x0B30, 0x0000,
0x1100, 0x1100,
0x1108, 0x0002,
0x1116, 0x0000,
0x1118, 0x0014,
0x0A0A, 0x8388,
0x0A10, 0xB040,
0x0A1E, 0x0013,
0x0C00, 0x0021,
0x0C14, 0x0010,
0x0C16, 0x0002,
0x0C18, 0x0900,
0x0C1A, 0x0700,
0x0708, 0x6F81,
0x0736, 0x0050,
0x0738, 0x0002,
0x073C, 0x0700,
0x0746, 0x00D4,
0x0748, 0x0002,
0x074A, 0x0900,
0x074C, 0x0100,
0x074E, 0x0100,
0x1200, 0x4946,
0x1206, 0x1800,
0x122E, 0x0490,
0x1230, 0x0248,
0x1000, 0x0300,
0x1002, 0xC311,
0x1004, 0x2BB0,
0x1010, 0x06CC,
0x1012, 0x0098,
0x1020, 0xC107,
0x1022, 0x081E,
0x1024, 0x0508,
0x1026, 0x080B,
0x1028, 0x1309,
0x102A, 0x0C0A,
0x102C, 0x1500,
0x1066, 0x06EF,
0x1600, 0x0400,
};

static kal_uint16 hi1634_preview_setting_tacoo[] = {
//DISP_WIDTH = 2304
//DISP_HEIGHT = 1728
//DISP_NOTE = "Q2SUM_2304x1728_30fps"
//MIPI_SPEED = 848.00
//MIPI_LANE = 4
//DISP_DATAORDER = GR
//MODE_INDEX = 4

////////////////////////////////////////////
// VT CLK: 80.00MHz
// Line length: 710
// Frame length: 3754
// Frame rate: 30.01
// ISP CLK: 169.60MHz
// V-Blank: 17980.75us
// Output bitwidth: 10 bits
// PD output bitwidth: 10 bits
////////////////////////////////////////////

0x0B00, 0x0000,
0x0204, 0x0408,
0x0206, 0x02C6,	// 2: tg/line_length_pck
0x020A, 0x0EA6,	// 3: tg/coarse_integ_time
0x020E, 0x0EAA,	// 4: tg/frame_length_lines
0x0224, 0x0044,
0x022A, 0x0016,
0x022C, 0x0E2A,
0x022E, 0x0DCA,
0x0234, 0x2222,
0x0236, 0x2222,
0x0238, 0x2222,
0x023A, 0x2222,
0x0268, 0x0108,
0x0400, 0x0E10,
0x0404, 0x0008,
0x0406, 0x1244,
0x0408, 0x0001,
0x040E, 0x0200,
0x0440, 0x011D,
0x0D00, 0x4000,
0x0D28, 0x0004,
0x0D2A, 0x0923,
0x0602, 0x3112,
0x0608, 0x0248,
0x067A, 0x0303,
0x067C, 0x0303,
0x06DE, 0x0303,
0x06E0, 0x0303,
0x06E4, 0x8A00,
0x06E6, 0x8A00,
0x06E8, 0x8A00,
0x06EA, 0x8A00,
0x0524, 0x5858,
0x0526, 0x5858,
0x0F04, 0x0010,
0x0F06, 0x0002,
0x0F08, 0x0011,
0x0F0A, 0x2233,
0x0B04, 0x009C,
0x0B12, 0x0900,
0x0B14, 0x06C0,
0x0B30, 0x0000,
0x1100, 0x1100,
0x1108, 0x0002,
0x1116, 0x0000,
0x1118, 0x0014,
0x0A0A, 0x8388,
0x0A10, 0xB040,
0x0A1E, 0x0013,
0x0C00, 0x0021,
0x0C14, 0x0010,
0x0C16, 0x0002,
0x0C18, 0x0900,
0x0C1A, 0x0700,
0x0708, 0x6F81,
0x0736, 0x0050,
0x0738, 0x0002,
0x073C, 0x0700,
0x0746, 0x00D4,
0x0748, 0x0002,
0x074A, 0x0900,
0x074C, 0x0100,
0x074E, 0x0100,
0x1200, 0x4946,
0x1206, 0x1800,
0x122E, 0x0490,
0x1230, 0x0248,
0x1000, 0x0300,
0x1002, 0xC311,
0x1004, 0x2BB0,
0x1010, 0x06CC,
0x1012, 0x0098,
0x1020, 0xC107,
0x1022, 0x081E,
0x1024, 0x0508,
0x1026, 0x080B,
0x1028, 0x1309,
0x102A, 0x0C0A,
0x102C, 0x1500,
0x1066, 0x06EF,
0x1600, 0x0400,
};


static kal_uint16 hi1634_normal_video_setting_tacoo[] = {
//DISP_WIDTH = 2304
//DISP_HEIGHT = 1300
//DISP_NOTE = "Q2SUM_24fps"
//MIPI_SPEED = 848.00
//MIPI_LANE = 4
//DISP_DATAORDER = GR
//MODE_INDEX = 4

////////////////////////////////////////////
// VT CLK: 80.00MHz
// Line length: 710
// Frame length: 3754
// Frame rate: 30.01
// ISP CLK: 169.60MHz
// V-Blank: 30104.00us
// Output bitwidth: 10 bits
// PD output bitwidth: 10 bits
////////////////////////////////////////////
0x0B00, 0x0000,
0x0204, 0x0408,
0x0206, 0x02C6,
0x020A, 0x1250,
0x020E, 0x0EAA,
0x0224, 0x01F0,
0x022A, 0x0016,
0x022C, 0x0E2A,
0x022E, 0x0C1E,
0x0234, 0x2222,
0x0236, 0x2222,
0x0238, 0x2222,
0x023A, 0x2222,
0x0268, 0x0108,
0x0400, 0x0E10,
0x0404, 0x0008,
0x0406, 0x1244,
0x0408, 0x0001,
0x040E, 0x0200,
0x0440, 0x011D,
0x0D00, 0x4000,
0x0D28, 0x0004,
0x0D2A, 0x0923,
0x0602, 0x3112,
0x0608, 0x0248,
0x067A, 0x0303,
0x067C, 0x0303,
0x06DE, 0x0303,
0x06E0, 0x0303,
0x06E4, 0x8A00,
0x06E6, 0x8A00,
0x06E8, 0x8A00,
0x06EA, 0x8A00,
0x0524, 0x5858,
0x0526, 0x5858,
0x0F04, 0x0010,
0x0F06, 0x0002,
0x0F08, 0x0011,
0x0F0A, 0x2233,
0x0B04, 0x009C,
0x0B12, 0x0900,
0x0B14, 0x0514,
0x0B30, 0x0000,
0x1100, 0x1100,
0x1108, 0x0002,
0x1116, 0x0000,
0x1118, 0x029C,
0x0A0A, 0x8388,
0x0A10, 0xB040,
0x0A1E, 0x0013,
0x0C00, 0x0021,
0x0C14, 0x0010,
0x0C16, 0x0002,
0x0C18, 0x0900,
0x0C1A, 0x0580,
0x0708, 0x6F81,
0x0736, 0x0050,
0x0738, 0x0002,
0x073C, 0x0700,
0x0746, 0x00D4,
0x0748, 0x0002,
0x074A, 0x0900,
0x074C, 0x0100,
0x074E, 0x0100,
0x1200, 0x4946,
0x1206, 0x1800,
0x122E, 0x0490,
0x1230, 0x0248,
0x1000, 0x0300,
0x1002, 0xC311,
0x1004, 0x2BB0,
0x1010, 0x06CC,
0x1012, 0x0098,
0x1020, 0xC107,
0x1022, 0x081E,
0x1024, 0x0508,
0x1026, 0x080B,
0x1028, 0x1309,
0x102A, 0x0C0A,
0x102C, 0x1500,
0x1066, 0x06EF,
0x1600, 0x0400,

};

static kal_uint16 hi1634_slim_video_setting_tacoo[] = {
/*
[SENSOR_RES_MOD]
DISP_WIDTH = 1920
DISP_HEIGHT = 1080
DISP_NOTE = "FHD_2SUM"
MIPI_SPEED = 848.00
MIPI_LANE = 4
DISP_DATAORDER = GR
*/
////////////////////////////////////////////
// VT CLK: 80.00MHz
// Line length: 710
// Frame length: 3754
// Frame rate: 30.01
// ISP CLK: 169.60MHz
// V-Blank: 23731.75us
// Output bitwidth: 10 bits
// PD output bitwidth: 10 bits
////////////////////////////////////////////
0x0B00, 0x0000,
0x0204, 0x0408,
0x0206, 0x02C6,
0x020A, 0x0EA6,
0x020E, 0x0EAA,
0x0224, 0x02CC,
0x022A, 0x0016,
0x022C, 0x0E2A,
0x022E, 0x0B42,
0x0234, 0x2222,
0x0236, 0x2222,
0x0238, 0x2222,
0x023A, 0x2222,
0x0268, 0x0108,
0x0400, 0x0E10,
0x0404, 0x0008,
0x0406, 0x1244,
0x0408, 0x0001,
0x040E, 0x0200,
0x0440, 0x011D,
0x0D00, 0x4000,
0x0D28, 0x0004,
0x0D2A, 0x0923,
0x0602, 0x3112,
0x0608, 0x0248,
0x067A, 0x0303,
0x067C, 0x0303,
0x06DE, 0x0303,
0x06E0, 0x0303,
0x06E4, 0x8A00,
0x06E6, 0x8A00,
0x06E8, 0x8A00,
0x06EA, 0x8A00,
0x0524, 0x5858,
0x0526, 0x5858,
0x0F04, 0x00D0,
0x0F06, 0x0002,
0x0F08, 0x0011,
0x0F0A, 0x2233,
0x0B04, 0x009C,
0x0B12, 0x0780,
0x0B14, 0x0438,
0x0B30, 0x0000,
0x1100, 0x1100,
0x1108, 0x0002,
0x1116, 0x0000,
0x1118, 0x0454,
0x0A0A, 0x8388,
0x0A10, 0xB040,
0x0A1E, 0x0013,
0x0C00, 0x0021,
0x0C14, 0x00D0,
0x0C16, 0x0002,
0x0C18, 0x0780,
0x0C1A, 0x0480,
0x0708, 0x6F81,
0x0736, 0x0050,
0x0738, 0x0002,
0x073C, 0x0700,
0x0746, 0x00D4,
0x0748, 0x0002,
0x074A, 0x0900,
0x074C, 0x0100,
0x074E, 0x0100,
0x1200, 0x4946,
0x1206, 0x1800,
0x122E, 0x0490,
0x1230, 0x0248,
0x1000, 0x0300,
0x1002, 0xC311,
0x1004, 0x2BB0,
0x1010, 0x06CC,
0x1012, 0x0110,
0x1020, 0xC107,
0x1022, 0x081E,
0x1024, 0x0508,
0x1026, 0x080B,
0x1028, 0x1309,
0x102A, 0x0C0A,
0x102C, 0x1500,
0x1066, 0x06EF,
0x1600, 0x0400,
};

static kal_uint16 hi1634_hs_video_setting_tacoo[] = {
/*
[SENSOR_RES_MOD]
DISP_WIDTH = 1280
DISP_HEIGHT = 720
DISP_NOTE = "HD_2SUM"
MIPI_SPEED = 848.00
MIPI_LANE = 4
DISP_DATAORDER = GR
*/
////////////////////////////////////////////
// VT CLK: 80.00MHz
// Line length: 710
// Frame length: 938
// Frame rate: 120.12
// ISP CLK: 169.60MHz
// V-Blank: 1934.75us
// Output bitwidth: 10 bits
// PD output bitwidth: 10 bits
////////////////////////////////////////////
0x0B00, 0x0000,
0x0204, 0x0408,
0x0206, 0x02C6,
0x020A, 0x03A6,
0x020E, 0x03AA,
0x0224, 0x0434,
0x022A, 0x0016,
0x022C, 0x0E2A,
0x022E, 0x09DA,
0x0234, 0x2222,
0x0236, 0x2222,
0x0238, 0x2222,
0x023A, 0x2222,
0x0268, 0x0108,
0x0400, 0x0E10,
0x0404, 0x0008,
0x0406, 0x1244,
0x0408, 0x0001,
0x040E, 0x0200,
0x0440, 0x011D,
0x0D00, 0x4000,
0x0D28, 0x0004,
0x0D2A, 0x0923,
0x0602, 0x3112,
0x0608, 0x0248,
0x067A, 0x0303,
0x067C, 0x0303,
0x06DE, 0x0303,
0x06E0, 0x0303,
0x06E4, 0x8A00,
0x06E6, 0x8A00,
0x06E8, 0x8A00,
0x06EA, 0x8A00,
0x0524, 0x5858,
0x0526, 0x5858,
0x0F04, 0x0210,
0x0F06, 0x0002,
0x0F08, 0x0011,
0x0F0A, 0x2233,
0x0B04, 0x009C,
0x0B12, 0x0500,
0x0B14, 0x02D0,
0x0B30, 0x0000,
0x1100, 0x1100,
0x1108, 0x0002,
0x1116, 0x0000,
0x1118, 0x0698,
0x0A0A, 0x8388,
0x0A10, 0xB040,
0x0A1E, 0x0013,
0x0C00, 0x0021,
0x0C14, 0x0210,
0x0C16, 0x0002,
0x0C18, 0x0500,
0x0C1A, 0x0300,
0x0708, 0x6F81,
0x0736, 0x0050,
0x0738, 0x0002,
0x073C, 0x0700,
0x0746, 0x00D4,
0x0748, 0x0002,
0x074A, 0x0900,
0x074C, 0x0100,
0x074E, 0x0100,
0x1200, 0x4946,
0x1206, 0x1800,
0x122E, 0x0490,
0x1230, 0x0248,
0x1000, 0x0300,
0x1002, 0xC311,
0x1004, 0x2BB0,
0x1010, 0x06CC,
0x1012, 0x01D8,
0x1020, 0xC107,
0x1022, 0x081E,
0x1024, 0x0508,
0x1026, 0x080B,
0x1028, 0x1309,
0x102A, 0x0C0A,
0x102C, 0x1500,
0x1066, 0x06EF,
0x1600, 0x0400,
};

static kal_uint16 hi1634_custom1_setting_tacoo[] = {
//DISP_WIDTH = 2304
//DISP_HEIGHT = 1728
//DISP_NOTE = "Q2SUM_2304x1728_30fps"
//MIPI_SPEED = 848.00
//MIPI_LANE = 4
//DISP_DATAORDER = GR
//MODE_INDEX = 4

////////////////////////////////////////////
// VT CLK: 80.00MHz
// Line length: 710
// Frame length: 3754
// Frame rate: 30.01
// ISP CLK: 169.60MHz
// V-Blank: 17980.75us
// Output bitwidth: 10 bits
// PD output bitwidth: 10 bits
////////////////////////////////////////////

0x0B00, 0x0000,
0x0204, 0x0408,
0x0206, 0x02C6,	// 2: tg/line_length_pck
0x020A, 0x0EA6,	// 3: tg/coarse_integ_time
0x020E, 0x0EAA,	// 4: tg/frame_length_lines
0x0224, 0x0044,
0x022A, 0x0016,
0x022C, 0x0E2A,
0x022E, 0x0DCA,
0x0234, 0x2222,
0x0236, 0x2222,
0x0238, 0x2222,
0x023A, 0x2222,
0x0268, 0x0108,
0x0400, 0x0E10,
0x0404, 0x0008,
0x0406, 0x1244,
0x0408, 0x0001,
0x040E, 0x0200,
0x0440, 0x011D,
0x0D00, 0x4000,
0x0D28, 0x0004,
0x0D2A, 0x0923,
0x0602, 0x3112,
0x0608, 0x0248,
0x067A, 0x0303,
0x067C, 0x0303,
0x06DE, 0x0303,
0x06E0, 0x0303,
0x06E4, 0x8A00,
0x06E6, 0x8A00,
0x06E8, 0x8A00,
0x06EA, 0x8A00,
0x0524, 0x5858,
0x0526, 0x5858,
0x0F04, 0x0010,
0x0F06, 0x0002,
0x0F08, 0x0011,
0x0F0A, 0x2233,
0x0B04, 0x009C,
0x0B12, 0x0900,
0x0B14, 0x06C0,
0x0B30, 0x0000,
0x1100, 0x1100,
0x1108, 0x0002,
0x1116, 0x0000,
0x1118, 0x0014,
0x0A0A, 0x8388,
0x0A10, 0xB040,
0x0A1E, 0x0013,
0x0C00, 0x0021,
0x0C14, 0x0010,
0x0C16, 0x0002,
0x0C18, 0x0900,
0x0C1A, 0x0700,
0x0708, 0x6F81,
0x0736, 0x0050,
0x0738, 0x0002,
0x073C, 0x0700,
0x0746, 0x00D4,
0x0748, 0x0002,
0x074A, 0x0900,
0x074C, 0x0100,
0x074E, 0x0100,
0x1200, 0x4946,
0x1206, 0x1800,
0x122E, 0x0490,
0x1230, 0x0248,
0x1000, 0x0300,
0x1002, 0xC311,
0x1004, 0x2BB0,
0x1010, 0x06CC,
0x1012, 0x0098,
0x1020, 0xC107,
0x1022, 0x081E,
0x1024, 0x0508,
0x1026, 0x080B,
0x1028, 0x1309,
0x102A, 0x0C0A,
0x102C, 0x1500,
0x1066, 0x06EF,
0x1600, 0x0400,

};

static kal_uint16 hi1634_custom2_setting_tacoo[] = {
//DISP_WIDTH = 2304
//DISP_HEIGHT = 1728
//DISP_NOTE = "Q2SUM_2304x1728_30fps"
//MIPI_SPEED = 848.00
//MIPI_LANE = 4
//DISP_DATAORDER = GR
//MODE_INDEX = 4

////////////////////////////////////////////
// VT CLK: 80.00MHz
// Line length: 710
// Frame length: 3754
// Frame rate: 30.01
// ISP CLK: 169.60MHz
// V-Blank: 17980.75us
// Output bitwidth: 10 bits
// PD output bitwidth: 10 bits
////////////////////////////////////////////

0x0B00, 0x0000,
0x0204, 0x0408,
0x0206, 0x02C6,	// 2: tg/line_length_pck
0x020A, 0x0EA6,	// 3: tg/coarse_integ_time
0x020E, 0x0EAA,	// 4: tg/frame_length_lines
0x0224, 0x0044,
0x022A, 0x0016,
0x022C, 0x0E2A,
0x022E, 0x0DCA,
0x0234, 0x2222,
0x0236, 0x2222,
0x0238, 0x2222,
0x023A, 0x2222,
0x0268, 0x0108,
0x0400, 0x0E10,
0x0404, 0x0008,
0x0406, 0x1244,
0x0408, 0x0001,
0x040E, 0x0200,
0x0440, 0x011D,
0x0D00, 0x4000,
0x0D28, 0x0004,
0x0D2A, 0x0923,
0x0602, 0x3112,
0x0608, 0x0248,
0x067A, 0x0303,
0x067C, 0x0303,
0x06DE, 0x0303,
0x06E0, 0x0303,
0x06E4, 0x8A00,
0x06E6, 0x8A00,
0x06E8, 0x8A00,
0x06EA, 0x8A00,
0x0524, 0x5858,
0x0526, 0x5858,
0x0F04, 0x0010,
0x0F06, 0x0002,
0x0F08, 0x0011,
0x0F0A, 0x2233,
0x0B04, 0x009C,
0x0B12, 0x0900,
0x0B14, 0x06C0,
0x0B30, 0x0000,
0x1100, 0x1100,
0x1108, 0x0002,
0x1116, 0x0000,
0x1118, 0x0014,
0x0A0A, 0x8388,
0x0A10, 0xB040,
0x0A1E, 0x0013,
0x0C00, 0x0021,
0x0C14, 0x0010,
0x0C16, 0x0002,
0x0C18, 0x0900,
0x0C1A, 0x0700,
0x0708, 0x6F81,
0x0736, 0x0050,
0x0738, 0x0002,
0x073C, 0x0700,
0x0746, 0x00D4,
0x0748, 0x0002,
0x074A, 0x0900,
0x074C, 0x0100,
0x074E, 0x0100,
0x1200, 0x4946,
0x1206, 0x1800,
0x122E, 0x0490,
0x1230, 0x0248,
0x1000, 0x0300,
0x1002, 0xC311,
0x1004, 0x2BB0,
0x1010, 0x06CC,
0x1012, 0x0098,
0x1020, 0xC107,
0x1022, 0x081E,
0x1024, 0x0508,
0x1026, 0x080B,
0x1028, 0x1309,
0x102A, 0x0C0A,
0x102C, 0x1500,
0x1066, 0x06EF,
0x1600, 0x0400,

};

static kal_uint16 hi1634_custom3_setting_tacoo[] = {
/*
[SENSOR_RES_MOD]
DISP_WIDTH = 4608
DISP_HEIGHT = 3456
DISP_NOTE = "QUAD_FULL_Crop"
MIPI_SPEED = 1696.00
MIPI_LANE = 4
DISP_DATAORDER = GR_QUAD
*/
////////////////////////////////////////////
// VT CLK: 80.00MHz
// Line length: 710
// Frame length: 3754
// Frame rate: 30.01
// ISP CLK: 169.60MHz
// V-Blank: 2644.75us
// Output bitwidth: 10 bits
// PD output bitwidth: 10 bits
////////////////////////////////////////////
0x0B00, 0x0000,
0x0204, 0x0008,
0x0206, 0x02C6,
0x020A, 0x0EA6,
0x020E, 0x0EAA,
0x0224, 0x0048,
0x022A, 0x0017,
0x022C, 0x0E1F,
0x022E, 0x0DC7,
0x0234, 0x1111,
0x0236, 0x1111,
0x0238, 0x1111,
0x023A, 0x1111,
0x0268, 0x0108,
0x0400, 0x0A10,
0x0404, 0x0008,
0x0406, 0x1244,
0x0408, 0x0001,
0x040E, 0x0200,
0x0440, 0x011D,
0x0D00, 0x400B,
0x0D28, 0x0008,
0x0D2A, 0x1247,
0x0602, 0x3712,
0x0608, 0x0490,
0x067A, 0x0900,
0x067C, 0x0009,
0x06DE, 0x0900,
0x06E0, 0x0009,
0x06E4, 0x8300,
0x06E6, 0x0100,
0x06E8, 0x0100,
0x06EA, 0x8300,
0x0524, 0x5858,
0x0526, 0x5858,
0x0F04, 0x0020,
0x0F06, 0x0000,
0x0F08, 0x0022,
0x0F0A, 0x1133,
0x0B04, 0x0094,
0x0B12, 0x1200,
0x0B14, 0x0D80,
0x0B30, 0x0001,
0x1100, 0x1100,
0x1108, 0x0002,
0x1116, 0x0000,
0x1118, 0x0018,
0x0A0A, 0x8388,
0x0A10, 0xB040,
0x0A1E, 0x0013,
0x0C00, 0x0023,
0x0C14, 0x0020,
0x0C16, 0x0000,
0x0C18, 0x1200,
0x0C1A, 0x0D80,
0x0708, 0x6F80,
0x0736, 0x0050,
0x0738, 0x0002,
0x073C, 0x0700,
0x0746, 0x00D4,
0x0748, 0x0002,
0x074A, 0x0900,
0x074C, 0x0000,
0x074E, 0x0100,
0x1200, 0x0946,
0x1206, 0x0800,
0x122E, 0x0490,
0x1230, 0x0248,
0x1000, 0x0300,
0x1002, 0xC311,
0x1004, 0x2BB0,
0x1010, 0x0DBC,
0x1012, 0x0149,
0x1020, 0xC10D,
0x1022, 0x0F39,
0x1024, 0x060F,
0x1026, 0x1013,
0x1028, 0x1F0A,
0x102A, 0x170A,
0x102C, 0x2800,
0x1066, 0x03C0,
0x1600, 0x0000,

};

static void sensor_init(void)
{
	pr_debug("%s +\n", __func__);

    hi1634_table_write_cmos_sensor(hi1634_init_setting_tacoo,
		sizeof(hi1634_init_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
}	/*	  sensor_init  */

static void preview_setting(void)
{
	pr_debug("%s +\n", __func__);

	hi1634_table_write_cmos_sensor(hi1634_preview_setting_tacoo,
		sizeof(hi1634_preview_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
} /* preview_setting */


static void capture_setting(kal_uint16 currefps)
{
	pr_debug("%s +\n", __func__);

	hi1634_table_write_cmos_sensor(hi1634_capture_setting_tacoo,
		sizeof(hi1634_capture_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	pr_debug("%s +\n", __func__);

	hi1634_table_write_cmos_sensor(hi1634_normal_video_setting_tacoo,
		sizeof(hi1634_normal_video_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
}

static void hs_video_setting(void)
{
	pr_debug("%s +\n", __func__);

	hi1634_table_write_cmos_sensor(hi1634_hs_video_setting_tacoo,
		sizeof(hi1634_hs_video_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
}

static void slim_video_setting(void)
{
	pr_debug("%s +\n", __func__);

	hi1634_table_write_cmos_sensor(hi1634_slim_video_setting_tacoo,
		sizeof(hi1634_slim_video_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
}

static void custom1_setting(void)
{
	pr_debug("%s +\n", __func__);

	hi1634_table_write_cmos_sensor(hi1634_custom1_setting_tacoo,
		sizeof(hi1634_custom1_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
}

static void custom2_setting(void)
{
	pr_debug("%s +\n", __func__);

	hi1634_table_write_cmos_sensor(hi1634_custom2_setting_tacoo,
		sizeof(hi1634_custom2_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
}

static void custom3_setting(void)
{
	pr_debug("%s +\n", __func__);

	hi1634_table_write_cmos_sensor(hi1634_custom3_setting_tacoo,
		sizeof(hi1634_custom3_setting_tacoo)/sizeof(kal_uint16));

    pr_debug("%s -\n", __func__);
}


/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/*sensor have two i2c address 0x34 & 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		gImgEepromInfo.i4CurSensorIdx = 1;
    	gImgEepromInfo.i4CurSensorId = imgsensor_info.sensor_id;
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = (read_cmos_sensor_8(0x0716) << 8) | read_cmos_sensor_8(0x0717);
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
                if (deviceInfo_register_value == 0x00) {
                    Eeprom_DataInit(1, HI1634Q_SENSOR_ID23687);
                    deviceInfo_register_value = 0x01;
                }
				return ERROR_NONE;
			}
			pr_debug("Read sensor id fail,imgsensor_info.sensor_id:0x%x, read:0x%x id: 0x%x\n", imgsensor_info.sensor_id, *sensor_id, imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct,
		 *Must set *sensor_id to 0xFFFFFFFF
		 */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	pr_debug("%s +\n", __func__);
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = (read_cmos_sensor_8(0x0716) << 8) | read_cmos_sensor_8(0x0717);
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("Read sensor id fail, id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	pr_debug("%s HI-1634 initial setting start \n", __func__);
	sensor_init();
	pr_debug("%s HI-1634 initial setting end \n", __func__);

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x100;
	imgsensor.gain = 0xe0;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;

	spin_unlock(&imgsensor_drv_lock);
	pr_debug("%s -\n", __func__);

	return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	pr_debug("E\n");
	/* No Need to implement this function */
	streaming_control(KAL_FALSE);
	return ERROR_NONE;
} /* close */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();

	return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
//	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
//	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
//	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 720P@240FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
//	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* slim_video */


static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 2328x1752@30FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	/*imgsensor.custom1 = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();
//	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* slim_video */


static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 2328x1752@30FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	/*imgsensor.custom2 = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
//	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* slim_video */


static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 2328x1752@30FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	/*imgsensor.custom3 = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();
//	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* slim_video */

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	pr_debug("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width =
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width =
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width =
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;



	return ERROR_NONE;
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 0;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;

		 case MSDK_SCENARIO_ID_CUSTOM1:
			 sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
			 sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
			 sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
	         imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
			 break;
		 case MSDK_SCENARIO_ID_CUSTOM2:
			 sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
			 sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
			 sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
	         imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

			 break;
		 case MSDK_SCENARIO_ID_CUSTOM3:
			 sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
			 sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
			 sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
	         imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
		 break;



	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		Custom1(image_window, sensor_config_data); // Custom1
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		Custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		Custom3(image_window, sensor_config_data);
		break;


	default:
		pr_debug("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	pr_debug("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/ {
		imgsensor.autoflicker_en = KAL_TRUE;
		pr_debug("enable! fps = %d", framerate);
	} else {
		 /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	pr_debug("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength
				+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
	break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;



	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		pr_debug("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	pr_debug("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	     case MSDK_SCENARIO_ID_CUSTOM1:
         *framerate = imgsensor_info.custom1.max_framerate;
     break;
     case MSDK_SCENARIO_ID_CUSTOM2:
         *framerate = imgsensor_info.custom2.max_framerate;
         break;
     case MSDK_SCENARIO_ID_CUSTOM3:
         *framerate = imgsensor_info.custom3.max_framerate;
         break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	pr_debug("set_test_pattern_mode enable: %d", enable);
	test_pattern_save = (read_cmos_sensor_8(0x0B04) << 8) | read_cmos_sensor_8(0x0B05);

	if (enable) {

		write_cmos_sensor(0x0B04, test_pattern_save+1);
		write_cmos_sensor(0x0C0A, 0x0104); //Solid color
	}
	else {
		write_cmos_sensor(0x0B04, test_pattern_save);
		write_cmos_sensor(0x0C0A, 0x0000);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
#if ENABLE_PDAF
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
#endif
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */
//	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
//		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*pr_debug("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MINT32 *)(signed long)(*(feature_data + 1)) = 2500000;
		break;
	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{
		/*get 4 cell data from eeprom*/
		int type = (kal_uint16)(*feature_data);
		char *data = (char *)(uintptr_t)(*(feature_data+1));

		/*only copy QSC calibration data*/
		if (type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
			memset(data, 0, FOUR_CELL_QSC_SIZE);
			read_4cell_data_from_eeprom(data);
		}
		break;
	}
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;

	    case MSDK_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom1.pclk;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom2.pclk;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom3.pclk;
            break;

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;



		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		 set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		 /* night_mode((BOOL) *feature_data); */
		break;
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
//		write_cmos_sensor_8(sensor_reg_data->RegAddr,
//				    sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
//		sensor_reg_data->RegData =
//			read_cmos_sensor_8(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*get the lens driver ID from EEPROM
		 * or just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,
				      *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 set_max_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		 get_default_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", (UINT32)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	#if 0
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
	#endif
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		 case MSDK_SCENARIO_ID_CUSTOM1:
             memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[5],
             sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
         break;
         case MSDK_SCENARIO_ID_CUSTOM2:
             memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[6],
             sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
         break;
         case MSDK_SCENARIO_ID_CUSTOM3:
              memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[7],
             sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
         break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		break;

	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)),
					(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CUSTOM3:
			*feature_return_para_32 = 1; /* NON */
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*feature_return_para_32 = 2; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;

		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.mipi_pixel_rate;
			break;

	    case MSDK_SCENARIO_ID_CUSTOM1:
             *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                 = imgsensor_info.custom1.mipi_pixel_rate;
             break;
         case MSDK_SCENARIO_ID_CUSTOM2:
             *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                 = imgsensor_info.custom2.mipi_pixel_rate;
             break;
         case MSDK_SCENARIO_ID_CUSTOM3:
             *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                 = imgsensor_info.custom3.mipi_pixel_rate;
             break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
break;


#if ENABLE_PDAF
	case SENSOR_FEATURE_GET_PDAF_DATA:
		pr_debug("SENSOR_FEATURE_GET_PDAF_DATA\n");
		read_eeprom_hi1634((kal_uint16)(*feature_data),
			(char*)(uintptr_t)(*(feature_data + 1)),
			(kal_uint32)(*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16)*feature_data);
		PDAFinfo =
			(struct SET_PD_BLOCK_INFO_T*)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void*)PDAFinfo,
				(void*)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void*)PDAFinfo,
				(void*)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			break;
		}

		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		pr_debug(
			"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
			(UINT16)*feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0; // type2 - VC enable
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32*)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		pr_debug("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		pr_debug("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_debug("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;


	case SENSOR_FEATURE_GET_VC_INFO:
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT*)(uintptr_t)(*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void*)pvcinfo, (void*)&SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void*)pvcinfo, (void*)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void*)pvcinfo, (void*)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		default:
			pr_info("error: get wrong vc_INFO id = %d",
				*feature_data_32);
			//memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
#endif
	default:
		break;
	}
	return ERROR_NONE;
} /* feature_control() */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI1634Q_MIPI_RAW23687_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /* HI1634_MIPI_RAW_SensorInit */

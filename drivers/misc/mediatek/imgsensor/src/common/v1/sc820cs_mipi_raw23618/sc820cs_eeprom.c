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

#include "sc820cs_eeprom.h"

#define LOG_TAG "SC820CS_EEPROM"
#define LOG_ERR(format, ...) pr_err(LOG_TAG "Line:%d  FUNC: %s:  "format, __LINE__, __func__, ## __VA_ARGS__)
#define LOG_INFO(format, ...) pr_info(LOG_TAG "  %s:  "format, __func__, ## __VA_ARGS__)
#define LOG_DEBUG(format, ...) pr_debug(LOG_TAG "  %s:  "format, __func__, ## __VA_ARGS__)

#define READ_SUCCESS	0
#define ERROR_I2C	   1
#define ERROR_CHECKSUM  2
#define ERROR_READ_FLAG 3
#define ERROR_GROUP	 4
#define OTP_THREAHOLD   3

#define READ_FLAG	   1

static int g_channel;

/* add for linux-4.4 */
#ifndef I2C_WR_FLAG
#define I2C_WR_FLAG		(0x1000)
#define I2C_MASK_FLAG	(0x00ff)
#endif

/*This sensor only have two group otp data.in diffrent page in sensor*/
int sc820cs_read_data_8(u16 addr, u8 *data);
OTP_MAP sc820cs_otp_map_ark = {
		.module_name = "SC820CS_ARK",
		.group_num = 4,
		.group_addr_info_num = 2,
		.readFunc = sc820cs_read_data_8,
		.group_info = {
						{"info",
							{
								{0x827A, 0x8289, 0x828A, 0x828B},
								{0x8C7A, 0x8C89, 0x8C8A, 0x8C8B},
							},
						},
						{"awb",
							{
								{0x828C, 0x829B, 0x829C, 0x829D},
								{0x8C8C, 0x8C9B, 0x8C9C, 0x8C9D},
							 },
						},
						{"sn",
							{
								{0x82A4, 0x82BB, 0x82BC, 0x82BD},
								{0x8CA4, 0x8CBB, 0x8CBC, 0x8CBD},
							},
						},
						{"lsc",
							{
								{0x82BE, 0x8BF1, 0x8BF2, 0x8BF3},
								{0x8CBE, 0x95F1, 0x95F2, 0x95F3},
							},
						},
			},
};

static void write_reg16_data8(u16 addr, u8 data)
{
	char pu_send_cmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(data & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, 0x20);
}

static bool sc820cs_sensor_otp_init(u16 threshold, int pagenumber)
{
	/*0x4412, 0xxx, read 0x8000-0x87FF data£¬0x4412,0x01
	 read 0x8800-0x8FFF£¬0x4412,0x03
	 0x4407, 0xxx, read 0x8000-0x87FF £¬0x4407,0x0c
	 read 0x8800-0x8FFF £¬0x4407,0x00
	*/
	int pageaddr = pagenumber * 2 - 1;
	static int startaddr = 0;
	static int endaddr = 0;
	if (pagenumber < 2 || pagenumber > 12)
		return false;
	startaddr = 0x8200 + (0x200 * (pagenumber - 2));
	endaddr = 0x83FF + (0x200 * (pagenumber - 2));
	write_reg16_data8(0x36b0, 0x48);
	write_reg16_data8(0x36b1, threshold);                     /* set threshold */
	write_reg16_data8(0x36b2, 0x41);
	write_reg16_data8(0x4408, (startaddr >> 8)&0xff);        /* High byte start address for OTP load Group1 */
	write_reg16_data8(0x4409, (startaddr)&0xff);             /* Low byte start address for OTP load  Group1 */
	write_reg16_data8(0x440a, (endaddr >> 8)&0xff);          /* High byte end address for OTP load Group1 */
	write_reg16_data8(0x440b, (endaddr)&0xff);              /* Low byte end address for OTP load Group1 */
	write_reg16_data8(0x4401, 0x13);
	write_reg16_data8(0x4412, pageaddr);
	write_reg16_data8(0x4407, 0x00);
	write_reg16_data8(0x4400, 0x11);                         /* 0x4400, 0x11 manual load */
	mdelay(20);
	return true;
}

int sc820cs_read_data_8(u16 addr, u8 *data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, data, 1, 0x20);
	return 0;
}

static int parse_otp_map_data_sc820cs(OTP_MAP * map, char * data, int temp)
{
	int group_type = 0, channel_type = 0;
	int addr = 0, size = 0, curr_addr = 0;
	int  ret = 0;
	char readbyte = 0;
	int checksum = -1;
	int pagenumber = 0;
	int pageflag = 0;
	int headeraddr;
	LOG_INFO("module: %s ......", map->module_name);

	if (g_channel == 1) {
		pagenumber = startpage1;
		headeraddr  = startaddr1;
		channel_type = 0;
	} else if (g_channel == 2) {
		pagenumber = startpage2;
		headeraddr  = startaddr2;
		channel_type = 1;
	}

	pageflag = pagenumber;

	ret = sc820cs_sensor_otp_init(temp, pagenumber);
	if (ret == false) {
		if (g_channel == 2) {
			ret = ERROR_GROUP;
		} else {
			ret = ERROR_READ_FLAG;
		}
		return ret;
	}
	for (group_type = 0; group_type < map->group_num; group_type++) {
		checksum = 0;
		size = 0;

		LOG_INFO("groupinfo: %s, start_addr 0x%0x(0x%0x)", map->group_info[group_type].group_name, curr_addr, curr_addr);
		if (!strcmp(map->group_info[group_type].group_name, "lsc")) {
			for (addr = map->group_info[group_type].group_addr_info[channel_type].group_start_addr;
				addr <=  map->group_info[group_type].group_addr_info[channel_type].group_end_addr; addr++) {
				if (sc820cs_check_page_head(&headeraddr, addr, &pageflag) == true) {
						addr = addr + 0x7A;
					ret = sc820cs_sensor_otp_init(temp, pageflag);
					if (ret == false) {
						if (g_channel > 2) {
							ret = ERROR_GROUP;
						} else {
							ret = ERROR_READ_FLAG;
						}
						return ret;
					}
				}
				ret = map->readFunc(addr, data);

				if (ret < 0) {
					LOG_ERR("read data error");
				}
				if (addr == map->group_info[group_type].group_addr_info[channel_type].group_flag_addr) {
					if (*data != READ_FLAG) {
						g_channel++;
						if (g_channel > 2) {
							ret = ERROR_GROUP;
							return ret;
						}
					}
				}
				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}

		} else {	/*There is no cross-page stitching to read, go here*/
			for (addr = map->group_info[group_type].group_addr_info[channel_type].group_start_addr;
					addr <= map->group_info[group_type].group_addr_info[channel_type].group_end_addr;
					addr++) {
				ret = map->readFunc(addr, data);

				if (ret < 0) {
					LOG_ERR("read data error");
				}
				if (addr == map->group_info[group_type].group_addr_info[channel_type].group_flag_addr) {
					if (*data != READ_FLAG) {
						g_channel++;
						if (g_channel > 2) {
							ret = ERROR_GROUP;
							return ret;
						}
					}
				}

				checksum += *data;
				curr_addr++;
				size++;
				data++;
			}
		}

		checksum = checksum % 0xFF;
		ret = map->readFunc(map->group_info[group_type].group_addr_info[channel_type].group_checksum_addr, &readbyte);
		if (checksum == readbyte) {
			ret = READ_SUCCESS;
			LOG_INFO("groupinfo: %s, checksum OK c(0x%04x) r(0x%04x)  checkdum 0x%04x  ret%d",
			map->group_info[group_type].group_name, checksum, readbyte, map->group_info[group_type].group_addr_info[channel_type].group_checksum_addr, ret);
		} else {
			LOG_ERR("groupinfo: %s, checksum ERROR ret=%d, checksum=%04d readbyte=%04d 0x%04x",
			map->group_info[group_type].group_name, ret, checksum, readbyte, map->group_info[group_type].group_addr_info[channel_type].group_checksum_addr);
			ret = ERROR_CHECKSUM;
			return ret;
		}
		LOG_INFO("groupinfo: %s, end_addr 0x%04x(%04d) size 0x%04x(%04d)",
						map->group_info[group_type].group_name, curr_addr-1, curr_addr-1, size, size);
	}
	return ret;
}

unsigned int sc820cs_read_region_insensor(void) {
	int ret = 0;
	int readtimes;
	int otpThreshold[3] = {0x38, 0x18, 0x58}; /* the Threshold for sc820cs*/
	int tempotpthreshold =  0;
	g_channel = 1;
	if (g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR]) {
		LOG_INFO("otp data has already been read and saved as g_otp_buf\n");
		return 0;
	}
    for (readtimes = 0; readtimes < OTP_THREAHOLD; readtimes++) {
		tempotpthreshold = otpThreshold[readtimes];
		ret = parse_otp_map_data_sc820cs(&sc820cs_otp_map_ark, &g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][0], tempotpthreshold);
		if (ret == ERROR_READ_FLAG || ret == ERROR_CHECKSUM) {
			if(readtimes >= 2 && ret == ERROR_CHECKSUM) {
				break;
			} else {
				continue;
			}
		} else if (ret == ERROR_GROUP) {
			LOG_ERR("bad group or bad check sum, read sc820cs err!!!\n");
			break;
		}

		if(ret == READ_SUCCESS) {
			g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][CHECKSUM_FLAG_ADDR] = OTP_DATA_GOOD_FLAG;
			g_otp_buf[IMGSENSOR_SENSOR_IDX_SUB][READ_FLAG_ADDR] = 1;
			break;
		}
	}
	return ret;
}

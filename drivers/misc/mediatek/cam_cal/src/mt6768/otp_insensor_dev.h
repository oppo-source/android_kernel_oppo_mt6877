/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __OTP_INSENSOR_H
#define __OTP_INSENSOR_H

#define MAX_NAME_LENGTH 20
#define MAX_GROUP_NUM 8
#define MAX_GROUP_ADDR_NUM 2
#define MAX_PAGE_NUMBER 5
#define MAX_EEPROM_BYTE 0x1FFF
#define CHECKSUM_FLAG_ADDR MAX_EEPROM_BYTE-1
#define READ_FLAG_ADDR MAX_EEPROM_BYTE-2
#define OTP_DATA_GOOD_FLAG 0x88

extern char g_otp_buf[3][MAX_EEPROM_BYTE];
extern enum
{
	startpage1 = 2,
	startpage2 = 7,
} frontpage;
extern enum
{
	startaddr1 = 0x8200,
	startaddr2 = 0x8C00,
} pageheaderaddr;
extern bool sc820cs_check_page_head(int *headeraddr, int curr_addr, int *pagenumber);
typedef struct
{
	int group_start_addr;
	int group_end_addr;
	int group_flag_addr;
	int group_checksum_addr;
}GROUP_ADDR_INFO;

typedef struct
{
	char group_name[MAX_NAME_LENGTH];
	GROUP_ADDR_INFO group_addr_info[MAX_GROUP_ADDR_NUM];
}GROUP_INFO;

typedef struct
{
	char module_name[MAX_NAME_LENGTH];
	int group_num;
	int group_addr_info_num;
	GROUP_INFO group_info[MAX_GROUP_NUM];
	int  (* readFunc) (u16 addr, u8 *data);
}OTP_MAP;
#endif

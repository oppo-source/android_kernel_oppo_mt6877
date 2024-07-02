/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*/

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include <mt-plat/mtk_boot_common.h>
#endif

#include "lcm_drv.h"

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
//#include "data_hw_roundedpattern.h"
#endif

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
//#include <mach/mt_pm_ldo.h>
//#include <mach/mt_gpio.h>
#endif

//#include <cust_gpio_usage.h>

#ifndef MACH_FPGA
//#include <cust_i2c.h>
#endif

#include "../../gate_ic/ocp2130_drv.h"
#include <soc/oplus/device_info.h>
#include "oplus23703_ili9883c_hkc_hdp_dsi_vdo_bl_map.h"

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(ALWAYS, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(INFO, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define SET_GPIO_LCD_ENP_BIAS(v)	(lcm_util.set_gpio_lcd_enp_bias((v)))
#define SET_GPIO_LCD_ENN_BIAS(v)	(lcm_util.set_gpio_lcd_enn_bias((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "lcm_i2c.h"
#endif

#define FRAME_WIDTH				(720)
#define FRAME_HEIGHT			(1604)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH		(69401)
#define LCM_PHYSICAL_HEIGHT		(154610)
#define LCM_DENSITY				(264)

#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF

#define MAX_NORMAL_BRIGHTNESS   3210
#define MIN_NORMAL_BRIGHTNESS   16
#define MULTIPLE_BRIGHTNESS   1070
#define LUX_VAL 41

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

static int cabc_status = 3;
/* static int esd_flag = 0; */
extern int tp_control_reset_gpio(bool enable);
extern int __attribute__((weak)) tp_gesture_enable_flag(void) {
	return 0;
}

extern void __attribute((weak)) lcd_queue_load_tp_fw(void)
{
	return;
};

extern unsigned int __attribute__((weak)) get_boot_mode(void)
{
	return 0;
}

/*extern unsigned int esd_recovery_backlight_level;
extern void __attribute__((weak)) tp_gpio_current_leakage_handler(bool normal)
{
	return;
};
*/

static int esd_last_level;

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{REGFLAG_DELAY, 8, {}},
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 90, {}},
};

/*static struct LCM_setting_table lcm_suspend_dbst_setting[] = {
	{0x6D, 2, {0x25,0x00} },
	{0x28, 2, {0x00,0x00} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 2, {0x00,0x00} },
	{REGFLAG_DELAY, 100, {} },
};*/

static struct LCM_setting_table lcm_init_setting_vdo[] = {
	//------------- Init_code -------------//
	{0xFF, 0x03, {0x98, 0x83, 0x01}},

	{0x00, 0x01, {0x4D}},
	{0x01, 0x01, {0x35}},
	{0x02, 0x01, {0x00}},
	{0x03, 0x01, {0x00}},
	{0x04, 0x01, {0xC9}},
	{0x05, 0x01, {0x15}},
	{0x06, 0x01, {0x00}},
	{0x07, 0x01, {0x00}},

	{0x08, 0x01, {0x89}},
	{0x09, 0x01, {0x06}},
	{0x0A, 0x01, {0xB4}},
	{0x0B, 0x01, {0x00}},

	{0x0C, 0x01, {0x53}},
	{0x0D, 0x01, {0x53}},
	{0x0E, 0x01, {0x00}},
	{0x0F, 0x01, {0x00}},

	{0x16, 0x01, {0x89}},
	{0x17, 0x01, {0x06}},
	{0x18, 0x01, {0x34}},
	{0x19, 0x01, {0x00}},

	{0x1A, 0x01, {0x53}},
	{0x1B, 0x01, {0x53}},
	{0x1C, 0x01, {0x00}},
	{0x1D, 0x01, {0x00}},

	{0x28, 0x01, {0x4F}},
	{0x2A, 0x01, {0x4F}},
	{0x29, 0x01, {0x91}},
	{0x2B, 0x01, {0x91}},

	{0x31, 0x01, {0x2A}},
	{0x32, 0x01, {0x2A}},
	{0x33, 0x01, {0x2A}},
	{0x34, 0x01, {0x2A}},
	{0x35, 0x01, {0x22}},
	{0x36, 0x01, {0x22}},
	{0x37, 0x01, {0x23}},
	{0x38, 0x01, {0x2A}},
	{0x39, 0x01, {0x2A}},
	{0x3A, 0x01, {0x2A}},
	{0x3B, 0x01, {0x23}},
	{0x3C, 0x01, {0x10}},
	{0x3D, 0x01, {0x12}},
	{0x3E, 0x01, {0x14}},
	{0x3F, 0x01, {0x16}},
	{0x40, 0x01, {0x18}},
	{0x41, 0x01, {0x1A}},
	{0x42, 0x01, {0x0C}},
	{0x43, 0x01, {0x0A}},
	{0x44, 0x01, {0x08}},
	{0x45, 0x01, {0x07}},
	{0x46, 0x01, {0x07}},

	{0x47, 0x01, {0x2A}},
	{0x48, 0x01, {0x2A}},
	{0x49, 0x01, {0x2A}},
	{0x4A, 0x01, {0x2A}},
	{0x4B, 0x01, {0x22}},
	{0x4C, 0x01, {0x22}},
	{0x4D, 0x01, {0x23}},
	{0x4E, 0x01, {0x2A}},
	{0x4F, 0x01, {0x2A}},
	{0x50, 0x01, {0x2A}},
	{0x51, 0x01, {0x23}},
	{0x52, 0x01, {0x11}},
	{0x53, 0x01, {0x13}},
	{0x54, 0x01, {0x15}},
	{0x55, 0x01, {0x17}},
	{0x56, 0x01, {0x19}},
	{0x57, 0x01, {0x1B}},
	{0x58, 0x01, {0x0D}},
	{0x59, 0x01, {0x0B}},
	{0x5A, 0x01, {0x09}},
	{0x5B, 0x01, {0x07}},
	{0x5C, 0x01, {0x07}},

	{0x61, 0x01, {0x2A}},
	{0x62, 0x01, {0x2A}},
	{0x63, 0x01, {0x22}},
	{0x64, 0x01, {0x22}},
	{0x65, 0x01, {0x2A}},
	{0x66, 0x01, {0x2A}},
	{0x67, 0x01, {0x23}},
	{0x68, 0x01, {0x2A}},
	{0x69, 0x01, {0x2A}},
	{0x6A, 0x01, {0x2A}},
	{0x6B, 0x01, {0x23}},
	{0x6C, 0x01, {0x17}},
	{0x6D, 0x01, {0x15}},
	{0x6E, 0x01, {0x13}},
	{0x6F, 0x01, {0x11}},
	{0x70, 0x01, {0x1B}},
	{0x71, 0x01, {0x19}},
	{0x72, 0x01, {0x0D}},
	{0x73, 0x01, {0x09}},
	{0x74, 0x01, {0x0B}},
	{0x75, 0x01, {0x07}},
	{0x76, 0x01, {0x07}},

	{0x77, 0x01, {0x2A}},
	{0x78, 0x01, {0x2A}},
	{0x79, 0x01, {0x22}},
	{0x7A, 0x01, {0x22}},
	{0x7B, 0x01, {0x2A}},
	{0x7C, 0x01, {0x2A}},
	{0x7D, 0x01, {0x23}},
	{0x7E, 0x01, {0x2A}},
	{0x7F, 0x01, {0x2A}},
	{0x80, 0x01, {0x2A}},
	{0x81, 0x01, {0x23}},
	{0x82, 0x01, {0x16}},
	{0x83, 0x01, {0x14}},
	{0x84, 0x01, {0x12}},
	{0x85, 0x01, {0x10}},
	{0x86, 0x01, {0x1A}},
	{0x87, 0x01, {0x18}},
	{0x88, 0x01, {0x0C}},
	{0x89, 0x01, {0x08}},
	{0x8A, 0x01, {0x0A}},
	{0x8B, 0x01, {0x07}},
	{0x8C, 0x01, {0x07}},

	{0xD1, 0x01, {0x10}},
	{0xD5, 0x01, {0x5D}},
	{0xD6, 0x01, {0x91}},
	{0xE6, 0x01, {0x22}},
	{0xE7, 0x01, {0x54}},

	{0xC3, 0x01, {0x00}},

	{0xFF, 0x03, {0x98, 0x83, 0x02}},
	{0x06, 0x01, {0x58}},
	{0x0A, 0x01, {0x5B}},
	{0x0C, 0x01, {0x00}},
	{0x0D, 0x01, {0x14}},
	{0x0E, 0x01, {0x68}},
	{0x39, 0x01, {0x05}},
	{0x3A, 0x01, {0x14}},
	{0x3B, 0x01, {0x68}},
	{0x3C, 0x01, {0xAB}},
	{0xF0, 0x01, {0x05}},
	{0xF1, 0x01, {0x47}},
	{0x48, 0x01, {0x01}},
	{0x44, 0x01, {0x68}},

	{0xFF, 0x03, {0x98, 0x83, 0x03}},
	{0x84, 0x01, {0x01}},
	{0x20, 0x01, {0x01}},
	{0x22, 0x01, {0xFA}},
	{0x80, 0x01, {0x04}},
	{0x81, 0x01, {0x04}},
	{0x82, 0x01, {0x04}},
	{0xAF, 0x01, {0x18}},
	{0xAD, 0x01, {0xDD}},
	{0x8C, 0x01, {0x59}},
	{0x90, 0x01, {0xA9}},
	{0x92, 0x01, {0xB1}},
	{0x93, 0x01, {0xB2}},
	{0xB5, 0x01, {0xD3}},

	/*3% MODE1*/
	{0xB6, 0x01, {0x84}},
	{0x88, 0x01, {0xEE}},
	{0x89, 0x01, {0xEE}},
	{0x8A, 0x01, {0xEF}},
	{0x8B, 0x01, {0xF1}},
	{0xA6, 0x01, {0xEF}},
	{0xAC, 0x01, {0xEF}},

	/*7% MODE2*/
	{0xB7, 0x01, {0x84}},
	{0x8C, 0x01, {0xE4}},
	{0x8D, 0x01, {0xE4}},
	{0x8E, 0x01, {0xE5}},
	{0x8F, 0x01, {0xE5}},
	{0x90, 0x01, {0xE5}},
	{0x91, 0x01, {0xE6}},
	{0x92, 0x01, {0xE6}},
	{0x93, 0x01, {0xE6}},
	{0x94, 0x01, {0xE6}},
	{0x95, 0x01, {0xE6}},
	{0xA7, 0x01, {0xE6}},
	{0xAD, 0x01, {0xE6}},

	/*11.5% MODE3*/
	{0xB8, 0x01, {0x84}},
	{0x96, 0x01, {0x60}},
	{0x97, 0x01, {0x9B}},
	{0x98, 0x01, {0x9C}},
	{0x99, 0x01, {0x9F}},
	{0x9A, 0x01, {0xC6}},
	{0x9B, 0x01, {0xC9}},
	{0x9C, 0x01, {0xCE}},
	{0x9D, 0x01, {0xCE}},
	{0x9E, 0x01, {0xCF}},
	{0x9F, 0x01, {0xE1}},
	{0xA8, 0x01, {0xF0}},
	{0xAE, 0x01, {0xD9}},

	{0xFF, 0x03, {0x98, 0x83, 0x05}},
	{0x69, 0x01, {0x9C}},
	{0x6A, 0x01, {0x9C}},
	{0x6D, 0x01, {0x8D}},
	{0x73, 0x01, {0x93}},
	{0x79, 0x01, {0x8D}},
	{0x7F, 0x01, {0x7F}},
	{0x68, 0x01, {0x3E}},
	{0x66, 0x01, {0x33}},
	{0x61, 0x01, {0x11}},
	{0x5D, 0x01, {0x5D}},

	{0xA5, 0x01, {0x7B}},
	{0x4F, 0x01, {0x0A}},

	{0x6E, 0x01, {0xC9}},
	{0x6F, 0x01, {0x8D}},
	{0x70, 0x01, {0x8D}},
	{0x71, 0x01, {0x8D}},
	{0x74, 0x01, {0xCF}},
	{0x75, 0x01, {0x93}},
	{0x76, 0x01, {0x93}},
	{0x77, 0x01, {0x93}},

	{0xFF, 0x03, {0x98, 0x83, 0x06}},
	{0x06, 0x01, {0xA4}},
	{0xD9, 0x01, {0x1F}},
	{0xC0, 0x01, {0x44}},
	{0xC1, 0x01, {0x16}},
	{0x0A, 0x01, {0x50}},

	{0xFF, 0x03, {0x98, 0x83, 0x07}},
	{0xC0, 0x01, {0x01}},
	{0xCB, 0x01, {0xB5}},
	{0xCC, 0x01, {0xAC}},
	{0xCD, 0x01, {0x9D}},
	{0xCE, 0x01, {0x7E}},
	{0xD1, 0x01, {0x20}},

	{0xFF, 0x03, {0x98, 0x83, 0x0A}},
	{0xE0, 0x01, {0x01}},
	{0xE1, 0x01, {0x0B}},
	{0xE2, 0x01, {0x01}},

	{0xFF, 0x03, {0x98, 0x83, 0x0B}},
	{0x9A, 0x01, {0x85}},
	{0x9B, 0x01, {0x9A}},
	{0x9C, 0x01, {0x04}},
	{0x9D, 0x01, {0x04}},
	{0x9E, 0x01, {0x8C}},
	{0x9F, 0x01, {0x8C}},
	{0xAA, 0x01, {0x22}},
	{0xAB, 0x01, {0xE0}},

	{0xFF, 0x03, {0x98, 0x83, 0x0E}},
	{0x11, 0x01, {0x4B}},
	{0x12, 0x01, {0x02}},
	{0x13, 0x01, {0x14}},
	{0x00, 0x01, {0xA0}},

	{0xFF, 0x03, {0x98, 0x83, 0x00}},
	{0x35, 0x01, {0x00}},
	{0x51, 0x02, {0x00, 0x00}},
	{0x53, 0x01, {0x2c}},
	{0x55, 0x01, {0x00}},
	{0x68, 0x01, {0x05}},
	{0x11, 0x02, {0x00, 0x00}},
	{REGFLAG_DELAY, 65, {}},
	{0x29, 0x02, {0x00,0x00}},
	{REGFLAG_DELAY, 5, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_enter_setting[] = {
	{0x55, 1, {0x03} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static int first_set_bl;
static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_diming_off[] = {
	{0x53, 0x01, {0x24} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static int map_exp[4096] = {0};

static void init_global_exp_backlight(void)
{
	int lut_index[LUX_VAL] = {0, 64, 144, 154, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680, 702,
								723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1070};
	int lut_value1[LUX_VAL] = {0, 3,  11,  13,  21,  32,  45, 59,  76,  92, 111, 130,  150, 172, 194, 219, 244, 268, 293, 320, 347, 373, 402,
								430, 459, 487, 517, 549, 579, 611, 642, 675, 707, 740, 775, 808, 843, 878, 911, 947, 1070};

	int index_start = 0, index_end = 0;
	int value1_start = 0, value1_end = 0;
	int i, j;
	int index_len = sizeof(lut_index) / sizeof(int);
	int value_len = sizeof(lut_value1) / sizeof(int);
	if (index_len == value_len) {
		for (i = 0; i < index_len - 1; i++) {
			index_start = lut_index[i] * MAX_NORMAL_BRIGHTNESS / MULTIPLE_BRIGHTNESS;
			index_end = lut_index[i+1] * MAX_NORMAL_BRIGHTNESS / MULTIPLE_BRIGHTNESS;
			value1_start = lut_value1[i] * MAX_NORMAL_BRIGHTNESS / MULTIPLE_BRIGHTNESS;
			value1_end = lut_value1[i+1] * MAX_NORMAL_BRIGHTNESS / MULTIPLE_BRIGHTNESS;
			for (j = index_start; j <= index_end; j++) {
					map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
			}
		}
		for (i = MAX_NORMAL_BRIGHTNESS; i < 4096; i++) {
			map_exp[i] = i;
		}
	}

	/*for (i=0; i<4096; i++) {
		LCM_LOGI("%s, ili9883c_hkc backlight: map_exp[%d]=%d\n", __func__, i, map_exp[i]);
	}*/

}

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
					 table[i].para_list, force_update);
			if (table[i].count > 1)
				MDELAY(1);
			break;
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	struct dfps_info *dfps_params = dsi->dfps_params;

	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 9000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/
	/* traversing array must less than DFPS_LEVELS */
	/* DPFS_LEVEL0 */
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	//dfps_params[0].PLL_CLOCK = 500;
	/* dfps_params[0].data_rate = xx; */
	/* if vfp solution */
	dfps_params[0].vertical_frontporch = 1325;	//1350
	//dfps_params[0].vertical_frontporch_for_low_power = 2300;	//45Hz

	/* DPFS_LEVEL1 */
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	/* dfps_params[1].PLL_CLOCK = 500;*/
	/* dfps_params[1].data_rate = xx; */
	/* if vfp solution */
	dfps_params[1].vertical_frontporch = 350;
	//dfps_params[1].vertical_frontporch_for_low_power = 2300; //45hz

	/* DPFS_LEVEL2 */
	dfps_params[2].level = DFPS_LEVEL2;
	dfps_params[2].fps = 5000;/*real fps * 100, to support float*/
	dfps_params[2].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/* if mipi clock solution */
	/* dfps_params[2].PLL_CLOCK = 500;*/
	/* dfps_params[2].data_rate = xx; */
	/* if vfp solution */
	dfps_params[2].vertical_frontporch = 1900;
	//dfps_params[2].vertical_frontporch_for_low_power = 2300; //45hz

	dsi->dfps_num = 3;
}
#endif

static void lcm_get_params(struct LCM_PARAMS *params)
{
	// unsigned int i = 0;
	// unsigned int dynamic_fps_levels = 0;

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH / 1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT / 1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
	LCM_LOGI("%s: lcm_dsi_mode %d\n", __func__, lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 18;
	params->dsi.vertical_frontporch = 350;	//358
	//params->dsi.vertical_frontporch_for_low_power = 750;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch = 4;
	params->dsi.horizontal_frontporch = 150;	//114
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;

	params->dsi.PLL_CLOCK = 515;	/* this value must be in MTK suggested table */
	params->dsi.data_rate = 1030;

	/* mipi hopping params */
	params->dsi.dynamic_switch_mipi = 1;
	params->dsi.data_rate_dyn = 990;
	params->dsi.PLL_CLOCK_dyn = 495;
	params->dsi.horizontal_frontporch_dyn = 114;

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x09;
	params->dsi.lcm_esd_check_table[0].count = 3;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x03;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;

	params->brightness_min = MIN_NORMAL_BRIGHTNESS;
	params->brightness_max = 4095;
	/* for ARR 2.0 */
	// params->max_refresh_rate = 60;
	// params->min_refresh_rate = 45;

/*#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 0;
	params->corner_pattern_height = ROUND_CORNER_H_TOP;
	params->corner_pattern_height_bot = ROUND_CORNER_H_BOT;
	params->corner_pattern_tp_size = sizeof(top_rc_pattern);
	params->corner_pattern_lt_addr = (void *)top_rc_pattern;
#endif*/

	#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/****DynFPS start****/
	lcm_dfps_int(&(params->dsi));
	/****DynFPS end****/
	#endif
	init_global_exp_backlight();
	params->blmap = map_exp;
	params->blmap_size = sizeof(map_exp) / sizeof(map_exp[0]);
	params->backlight_diversion_map = diversion_value_map;
	params->backlight_diversion_map_size = sizeof(diversion_value_map)/sizeof(diversion_value_map[0]);
	register_device_proc("lcd", "ili9883c", "truly_hkc");
}

/* turn on gate ic & control voltage to 5.5V */
static void lcm_init_power(void)
{
	_bias_ic_i2c_panel_bias_enable(1);
	SET_GPIO_LCD_ENP_BIAS(1);
	MDELAY(5);
	SET_GPIO_LCD_ENN_BIAS(1);
	MDELAY(2);
	LCM_LOGI("ili9883c_hkc power_on\n");
}

static void lcm_suspend_power(void)
{
	if(0 == tp_gesture_enable_flag()) {
		LCM_LOGI("ili9883c_hkc suspend_power\n");
		SET_GPIO_LCD_ENN_BIAS(0);
		MDELAY(5);
		SET_GPIO_LCD_ENP_BIAS(0);
		//_bias_ic_i2c_panel_bias_enable(0);
	}
}

/* turn on gate ic & control voltage to 5.5V */
static void lcm_resume_power(void)
{
	SET_RESET_PIN(0);
	MDELAY(2);
	lcm_init_power();
}

#ifdef LCM_SET_DISPLAY_ON_DELAY
static U32 lcm_init_tick = 0;
static int is_display_on = 0;
#endif

static void lcm_init_set_cabc(int cabc_mode) {

	lcm_cabc_enter_setting[0].para_list[0] = cabc_mode;
	LCM_LOGI("%s [ark] hkc init cabc_mode is %d \n", __func__, cabc_mode);
	push_table(NULL, lcm_cabc_enter_setting, sizeof(lcm_cabc_enter_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_init(void)
{
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(5);

	SET_RESET_PIN(1);
	lcd_queue_load_tp_fw();
	MDELAY(15);
	first_set_bl = 1;
	push_table(NULL, lcm_init_setting_vdo, ARRAY_SIZE(lcm_init_setting_vdo), 1);
	lcm_init_set_cabc(cabc_status);
	LCM_LOGI("ili9883c_hkc ----init----\n");
}

static void lcm_suspend(void)
{
	push_table(NULL, lcm_suspend_setting,
		   ARRAY_SIZE(lcm_suspend_setting), 1);
}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_shudown_power(void)
{
	LCM_LOGI("ili9883c_hkc shudown_power\n");
	/*tp_control_reset_gpio(false);
	MDELAY(12);
	SET_RESET_PIN(0);
	MDELAY(2);*/
	SET_GPIO_LCD_ENN_BIAS(0);
	MDELAY(2);
	SET_GPIO_LCD_ENP_BIAS(0);
	MDELAY(2);
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

#ifdef LCM_SET_DISPLAY_ON_DELAY
	lcm_set_display_on();
#endif

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}

/* return TRUE: need recovery */
/* return FALSE: No need recovery */
static unsigned int lcm_esd_recover(void)
{
	/*char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);

	if (buffer[0] != 0x9c) {
		LCM_LOGI("[LCM ERROR] [0x0a]=0x%02x\n", buffer[0]);
		return TRUE;
	} else {
		LCM_LOGI("[LCM NORMAL] [0x0a]=0x%02x\n", buffer[0]);
		return FALSE;
	}*/

	int level = 0;
	lcm_resume_power();
	lcm_resume();

	level = esd_last_level;
	LCM_LOGI("ili9883c_hkc esd_recovery level = %d\n", level);
	bl_level[0].para_list[0] = (level >> 8) & 0x0f;
	bl_level[0].para_list[1] = level & 0xff;
	if (first_set_bl) {
		MDELAY(12);
		first_set_bl = 0;
	}
	push_table(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	return 0;
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	LCM_LOGI("ATA check size = 0x%x,0x%x,0x%x,0x%x\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00043700;	/* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
	    && (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	return ret;
#else
	return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	int lvl = 0;

	if (level == 0) {
		LCM_LOGI("ili9883c_hkc backlight diming off\n");
		push_table(NULL, bl_diming_off, sizeof(bl_diming_off) / sizeof(struct LCM_setting_table), 1);
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		lvl = 2047;
	else
		lvl = level;

	esd_last_level = level;

	LCM_LOGI("ili9883c_hkc backlight: lvl=%d, level=%d, esd_last_level=%d\n",
		lvl, level, esd_last_level);
	bl_level[0].para_list[0] = (lvl >> 8) & 0x0f;
	bl_level[0].para_list[1] = lvl & 0xff;

	if (first_set_bl) {
		MDELAY(12);
		first_set_bl = 0;
	}

	push_table(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}

static void hkc_ili9883c_lcm_set_cabc_mode(void *handle, unsigned int mode)
{
	int mapped_level = 0;

	if (mode == 1) {
		mapped_level = 0x01;
	} else if (mode == 2) {
		mapped_level = 0x02;
	} else if ( mode==3 ){
		mapped_level = 0x03;
	} else {
		mapped_level = 0x00;
	}
	lcm_cabc_enter_setting[0].para_list[0] = mapped_level;
	LCM_LOGI("%s [ark] cabc_mode is %d \n", __func__, mapped_level);
	push_table(NULL, lcm_cabc_enter_setting, sizeof(lcm_cabc_enter_setting) / sizeof(struct LCM_setting_table), 1);
	cabc_status = mode;
}

struct LCM_DRIVER oplus23703_ili9883c_hkc_hdp_dsi_vdo_drv = {
	.name = "oplus23703_ili9883c_hkc_hdp_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.shutdown_power = lcm_shudown_power,
	.compare_id = lcm_compare_id,
	.esd_recover = lcm_esd_recover,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.set_cabc_mode_cmdq = hkc_ili9883c_lcm_set_cabc_mode,
};

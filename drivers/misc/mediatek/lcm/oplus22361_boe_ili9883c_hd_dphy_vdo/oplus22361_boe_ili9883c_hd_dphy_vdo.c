/***********************************************************
** Copyright (C), 2008-2016, OPPO Mobile Comm Corp., Ltd.
** ODM_WT_EDIT
** File: - nt36525b_hlt_even_boe_hdp_dsi_vdo_lcm.c
** Description: source file for lcm nt36525b+hlt in kernel stage
**
** Version: 1.0
** Date : 2019/9/25
**
** ------------------------------- Revision History: -------------------------------
**<author><data><version ><desc>
**  lianghao       2019/9/25     1.0     source file for lcm nt36525b+hlt in kernel stage
**
****************************************************************/
#define pr_fmt(fmt) "dsi_cmd: %s: " fmt, __func__

#define LOG_TAG "LCM"
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include "../mt65xx_lcm_list.h"
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#endif
/*#include <linux/update_tpfw_notifier.h>*/
#include "disp_cust.h"
/*#include <soc/oppo/device_info.h>*/
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_LCM_VDD18_PIN(v)    (lcm_util.set_gpio_lcm_vddio_ctl((v)))
#define SET_LCM_VSP_PIN(v)  (lcm_util.set_gpio_lcd_enp_bias((v)))
#define SET_LCM_VSN_PIN(v)  (lcm_util.set_gpio_lcd_enn_bias((v)))
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)   (lcm_util.udelay(n))

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
#endif

#define LCM_DSI_CMD_MODE        0
#define FRAME_WIDTH             (720)
#define FRAME_HEIGHT            (1612)
#define LCM_PHYSICAL_WIDTH      (67932)
#define LCM_PHYSICAL_HEIGHT     (152092)
#define REGFLAG_DELAY   0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
/*extern unsigned int esd_recovery_backlight_level;*/
extern int gesture_flag;
extern int tp_gesture_enable_flag(void);
/*extern void ili_resume_by_ddi(void);*/
/*extern void core_config_sleep_ctrl(bool out);*/

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[200];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};
static unsigned int bl_old_level = 0;

#if 1
static int blmap_table[] = {
                36, 8,
                16, 11,
                17, 12,
                19, 13,
                19, 15,
                20, 14,
                22, 14,
                22, 14,
                24, 10,
                24, 8 ,
                26, 4 ,
                27, 0 ,
                29, 9 ,
                29, 9 ,
                30, 14,
                33, 25,
                34, 30,
                36, 44,
                37, 49,
                40, 65,
                40, 69,
                43, 88,
                46, 109,
                47, 112,
                50, 135,
                53, 161,
                53, 163,
                60, 220,
                60, 223,
                64, 257,
                63, 255,
                71, 334,
                71, 331,
                75, 375,
                80, 422,
                84, 473,
                89, 529,
                88, 518,
                99, 653,
                98, 640,
                103, 707,
                117, 878,
                115, 862,
                122, 947,
                128, 1039,
                135, 1138,
                132, 1102,
                149, 1355,
                157, 1478,
                166, 1611,
                163, 1563,
                183, 1900,
                180, 1844,
                203, 2232,
                199, 2169,
                209, 2344,
                236, 2821,
                232, 2742,
                243, 2958,
                255, 3188,
                268, 3433,
                282, 3705,
                317, 4400,
                176, 1555};
#endif

/*
static struct LCM_setting_table init_setting_cmd[] = {
	{ 0xFF, 0x03, {0x98, 0x07, 0x00} },
};
*/

static struct LCM_setting_table bl_level[] = {
	/* { 0xFF, 0x03, {0x98, 0x81, 0x00} }, */
	{0x51, 2, {0x00, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table init_setting_vdo[] = {
		{0xFF, 03, {0x98, 0x83, 0x06}},
		{0x06, 01, {0xA4}},
		{0x3E, 01, {0xE2}},
		{0xFF, 03, {0x98, 0x83, 0x03}},
		{0x83, 01, {0x20}},
		{0x84, 01, {0x00}},
		{0xFF, 03, {0x98, 0x83, 0x03}},
		{0x86, 01, {0x6C}},
		{0x88, 01, {0xE1}},
		{0x89, 01, {0xe8}},
		{0x8A, 01, {0xF0}},
		{0x8B, 01, {0xF7}},
		{0x8C, 01, {0xBF}},
		{0x8D, 01, {0xC5}},
		{0x8E, 01, {0xC8}},
		{0x8F, 01, {0xCE}},
		{0x90, 01, {0xD1}},
		{0x91, 01, {0xD6}},
		{0x92, 01, {0xDC}},
		{0x93, 01, {0xE3}},
		{0x94, 01, {0xED}},
		{0x95, 01, {0xFA}},
		{0xAF, 01, {0x18}},

		{0xFF, 03, {0x98, 0x83, 0x07}},
		{0x00, 01, {0x00}},
		{0x01, 01, {0x00}},

		{0xFF, 03, {0x98, 0x83, 0x00}},
		{0x53, 01, {0x24}},
		{0x51, 02, {0x0F, 0xFF}},
		{0x35, 01, {0x00}},
		{0x11, 01, {0x00}},
		{REGFLAG_DELAY, 80, {}},
		{0x29, 01, {0x00}},
		{REGFLAG_DELAY, 20, {}},
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
    unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;
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
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
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
	dsi->dfps_default_fps = 6000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/

	/*traversing array must less than DFPS_LEVELS*/
	/*DPFS_LEVEL0*/
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[0].PLL_CLOCK = xx;*/
	/*dfps_params[0].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[0].horizontal_frontporch = xx;*/
	dfps_params[0].vertical_frontporch = 1360;
	dfps_params[0].vertical_frontporch_for_low_power = 0;

	/*DPFS_LEVEL1*/
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/
	dfps_params[1].vertical_frontporch = 370;
	dfps_params[1].vertical_frontporch_for_low_power = 0;

	dsi->dfps_num = 2;
}
#endif

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));
	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	lcm_dsi_mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
	pr_debug("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
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
	params->dsi.vertical_backporch = 20;
	params->dsi.vertical_frontporch = 370;
	/*params->dsi.vertical_frontporch_for_low_power = 540;*/
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch = 10;
	params->dsi.horizontal_frontporch = 20;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
	params->dsi.dynamic_switch_mipi = 1;
	params->dsi.horizontal_sync_active_dyn = 4;
	params->dsi.horizontal_backporch_dyn = 24;
	params->dsi.data_rate_dyn = 914;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 360;	/* this value must be in MTK suggested table */
#else
	params->dsi.data_rate = 898;	/* this value must be in MTK suggested table */
#endif

	params->blmap = blmap_table;
	params->blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]);
	params->brightness_max = 4096;
	params->brightness_min = 4;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        params->round_corner_en = 1;
        params->full_content = 1;
        params->corner_pattern_width = 720;
        params->corner_pattern_height = 106;
        params->corner_pattern_height_bot = 106;
#endif

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
        /****DynFPS start****/
        lcm_dfps_int(&(params->dsi));
        /****DynFPS end****/
#endif

	register_device_proc("lcd", "ili9883c", "boe");
}


static void lcm_init_power(void)
{
	/*pr_debug("lcm_init_power\n");*/
	pr_debug("lcm_init_power\n");
	MDELAY(1);
	SET_LCM_VDD18_PIN(1);
	MDELAY(3);
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
	MDELAY(15);
}

static void lcm_suspend_power(void)
{
	pr_err("lcm_suspend_power\n");
	if (!tp_gesture_enable_flag()) {
		printk("lcm_tp_suspend_power_on\n");
		SET_LCM_VSN_PIN(0);
		MDELAY(2);
		SET_LCM_VSP_PIN(0);
	}
}

static void lcm_shudown_power(void)
{
	printk("nt36672c lcm_shudown_power\n");
	MDELAY(12);
	SET_RESET_PIN(0);
	MDELAY(2);
	SET_LCM_VSN_PIN(0);
	MDELAY(2);
	SET_LCM_VSP_PIN(0);
	MDELAY(2);
	SET_LCM_VDD18_PIN(0);
}

static void lcm_resume_power(void)
{
	pr_err("lcm_resume_power\n");
	SET_LCM_VSP_PIN(1);
	MDELAY(3);
	SET_LCM_VSN_PIN(1);
	/*base voltage = 4.0 each step = 100mV; 4.0+20 * 0.1 = 6.0v;*/
	/* #if defined(MTK_LCM_DEVICE_TREE_SUPPORT_PASCAL_E) */
	if (display_bias_setting(0x15)) {
		pr_err("fatal error: lcd gate ic setting failed \n");
	}
	/* #endif */
	MDELAY(10);
}

extern void lcd_queue_load_tp_fw(void);
static void lcm_init(void)
{
	pr_err("lcm_init\n");
	SET_RESET_PIN(0);
	MDELAY(3);
	SET_RESET_PIN(1);
	MDELAY(15);
	lcd_queue_load_tp_fw();
	MDELAY(10);
	push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
	pr_err("csot_ili7807s_lcm_mode = vdo mode :%d\n", lcm_dsi_mode);
}

static void lcm_suspend(void)
{
	pr_err("lcm_suspend\n");
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	pr_err("lcm_resume\n");
	lcm_init();
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	if (level > 4095)
		level = 4095;

	if (0 == level) {
		bl_level[0].para_list[0] = 0;
		bl_level[0].para_list[1] = 0;
		pr_err("[ HW backlight nt36672c]level = %d para_list[0] = %x, \
		para_list[1]=%x\n", level, bl_level[0].para_list[0], bl_level[0].para_list[1]);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
		pr_err("close backlight level==%u ", level);
	} else {
		bl_level[0].para_list[0] = 0x000F&(level >> 8);
		bl_level[0].para_list[1] = 0x00FF&(level);
		pr_err("[ HW  backlight nt36672c]level = %d para_list[0] = %x, \
		para_list[1]=%x\n", level, bl_level[0].para_list[0], bl_level[0].para_list[1]);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	}
	bl_old_level = level;
}

static struct LCM_setting_table set_cabc_off[] = {
	{0xFF, 3, {0x98, 0x83, 0x00}},
	{0x55, 1, {0x00}},
};
static struct LCM_setting_table set_cabc_1[] = {
	{0xFF, 3, {0x98, 0x83, 0x00}},
	{0x55, 1, {0x01}},
};
static struct LCM_setting_table set_cabc_2[] = {
	{0xFF, 3, {0x98, 0x83, 0x00}},
	{0x55, 1, {0x02}},
};

static void lcm_set_cabc_cmdq(void *handle, unsigned int level) {
	pr_info("[lcm] cabc set level %d\n", level);
        if (level == 3) {
                level = 2;
                pr_info("[lcm] cabc set level_2 %d\n", level);
        }
	if (level == 0) {
		push_table(handle, set_cabc_off, sizeof(set_cabc_off) / sizeof(struct LCM_setting_table), 1);
	} else if (level == 1) {
		push_table(handle, set_cabc_1, sizeof(set_cabc_1) / sizeof(struct LCM_setting_table), 1);
	} else if (level == 2) {
		push_table(handle, set_cabc_2, sizeof(set_cabc_2) / sizeof(struct LCM_setting_table), 1);
	} else {
		pr_info("[lcm]  level %d is not support\n", level);
	}
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK
	lcm_resume_power();
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

	push_table(NULL, init_setting_vdo, sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table), 1);
	pr_debug("nt36672c_tm_lcm_mode = vdo mode esd recovery :%d----\n", lcm_dsi_mode);
	pr_debug("lcm_esd_recovery\n");
	push_table(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	/*push_table_cust(NULL, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table_V3), 0);*/
	return FALSE;
#else
	return FALSE;
#endif
}

struct LCM_DRIVER oplus22361_boe_ili9883c_hd_dphy_vdo_lcm_drv = {
	.name = "oplus22361_boe_ili9883c_hd_dphy_vdo_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.shutdown_power = lcm_shudown_power,
	.esd_recover = lcm_esd_recover,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.set_cabc_mode_cmdq = lcm_set_cabc_cmdq,
};


/* SPDX-License-Identifier: GPL-2.0 */
/*
* Copyright (c) 2019 MediaTek Inc.
*/

#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "ocp2130_drv.h"


/*****************************************************************************
 * Define
 *****************************************************************************/
#define BIAS_I2C_ID_NAME "bias_ic_i2c_ocp2130"

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME " %s(%d) :[GATE][I2C] " fmt, __func__, __LINE__

/*****************************************************************************
 * Define Register
 *****************************************************************************/
#define VPOS_BIAS			0x00
#define VNEG_BIAS			0x01
#define DISPLAY_BIAS_DISCHARGE	0x03
#define LCM_BIAS_ID			0x04

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static const struct of_device_id _bias_ic_i2c_of_match[] = {
	{
		.compatible = "mediatek,i2c_lcd_bias",
	 },
};

static struct i2c_client *_bias_ic_i2c_client;

struct bias_ic_client {
	struct i2c_client *i2c_client;
	struct device *dev;
};

/*****************************************************************************
 * Driver Functions
 *****************************************************************************/
int _bias_ic_i2c_read_bytes(unsigned char addr, unsigned char *returnData)
{
	char cmd_buf[2] = { 0x00, 0x00 };
	char readData = 0;
	int ret = 0;

	struct i2c_client *client = _bias_ic_i2c_client;

	if (client == NULL) {
		pr_info("ERROR!! _bias_ic_i2c_client is null\n");
		return 0;
	}

	cmd_buf[0] = addr;
	ret = i2c_master_send(client, &cmd_buf[0], 1);
	ret = i2c_master_recv(client, &cmd_buf[1], 1);
	if (ret < 0)
		pr_info("ERROR %d!! i2c read data 0x%0x fail !!\n", ret, addr);

	readData = cmd_buf[1];
	*returnData = readData;

	return ret;
}
EXPORT_SYMBOL_GPL(_bias_ic_i2c_read_bytes);

int _bias_ic_i2c_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = _bias_ic_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_info("ERROR!! _bias_ic_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("ERROR %d!! i2c write data fail 0x%0x, 0x%0x !!\n",
				ret, addr, value);

	return ret;
}
EXPORT_SYMBOL_GPL(_bias_ic_i2c_write_bytes);

void _bias_ic_i2c_panel_bias_enable(unsigned int power_status)
{

	pr_info("Panel bias enable\n");

	if (power_status) {
		/*set bias to 5.4v*/
		_bias_ic_i2c_write_bytes(VPOS_BIAS, 0x14);
		_bias_ic_i2c_write_bytes(VNEG_BIAS, 0x14);

		/*bias enable*/
		_bias_ic_i2c_write_bytes(DISPLAY_BIAS_DISCHARGE, 0x33);
	} else {
		_bias_ic_i2c_write_bytes(DISPLAY_BIAS_DISCHARGE, 0x30);
	}
}
EXPORT_SYMBOL_GPL(_bias_ic_i2c_panel_bias_enable);

/*****************************************************************************
 * Function
 *****************************************************************************/
static int _bias_ic_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct bias_ic_client *bias_client;

	pr_info("%s NT: info==>name=%s addr=0x%x\n",
		__func__, client->name, client->addr);

	bias_client = devm_kzalloc(&client->dev, sizeof(struct bias_ic_client), GFP_KERNEL);

	if (!bias_client)
		return -ENOMEM;

	bias_client->dev = &client->dev;
	bias_client->i2c_client = client;

	i2c_set_clientdata(client, bias_client);
	_bias_ic_i2c_client = client;

	pr_info("Probe success!");

	return 0;
}

static int _bias_ic_i2c_remove(struct i2c_client *client)
{
	struct bias_ic_client *bias_client;

	pr_info("Bias ic remove\n");
	bias_client = i2c_get_clientdata(client);

	i2c_unregister_device(client);

	kfree(bias_client);
	bias_client = NULL;
	_bias_ic_i2c_client = NULL;
	i2c_unregister_device(client);

	return 0;
}

/*****************************************************************************
 * Data Structure
 *****************************************************************************/
static const struct i2c_device_id _bias_ic_i2c_id[] = {
	{BIAS_I2C_ID_NAME, 0},
	{}
};

static struct i2c_driver _bias_ic_i2c_driver = {
	.id_table = _bias_ic_i2c_id,
	.probe = _bias_ic_i2c_probe,
	.remove = _bias_ic_i2c_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = BIAS_I2C_ID_NAME,
		   .of_match_table = _bias_ic_i2c_of_match,
		   },
};

module_i2c_driver(_bias_ic_i2c_driver);

MODULE_AUTHOR("Mediatek Corporation");
MODULE_DESCRIPTION("MTK OCP2130 I2C Driver");
MODULE_LICENSE("GPL");



#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <kd_camera_feature.h>
#include <imgsensor_hw.h>
#include <camera_hw/imgsensor_cfg_table.h>

void wl2868c_AF_poweron(int status);
int wl2868c_voltage_output_t(enum IMGSENSOR_SENSOR_IDX sensor_idx, enum IMGSENSOR_HW_PIN pin, enum IMGSENSOR_HW_POWER_STATUS pwr_status);
void enable_wl2868c_gpio(int status);

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) "[REAR_FLICKER] " fmt

#include "rear_flickerhub.h"
#include <rear_flicker.h>
#include <hwmsensor.h>
#include <SCP_sensorHub.h>
#include "SCP_power_monitor.h"
#include <linux/pm_wakeup.h>
#ifdef OPLUS_FEATURE_SENSOR
#include "../../oplus_sensor_devinfo/sensor_devinfo.h"
#endif
#define REAR_FLICKERHUB_DEV_NAME     "rear_flicker_hub_pl"

struct rear_flickerhub_ipi_data {
	struct work_struct init_done_work;
	atomic_t first_ready_after_boot;
	/*misc */
	atomic_t	rear_flicker_suspend;
	atomic_t	trace;
	atomic_t	scp_init_done;

	/*data */
	u16		rear_flicker;
	int		rear_flicker_factor;
	atomic_t	rear_flicker_cali;
	ulong		enable;
	ulong		pending_intr;
	bool rear_flicker_factory_enable;
	bool rear_flicker_android_enable;
};

static struct rear_flickerhub_ipi_data *obj_ipi_data;


static int rear_flickerhub_local_init(void);
static int rear_flickerhub_local_remove(void);
static int rear_flickerhub_init_flag = -1;

static struct rear_flicker_init_info rear_flickerhub_init_info = {
	.name = "rear_flicker_hub",
	.init = rear_flickerhub_local_init,
	.uninit = rear_flickerhub_local_remove,

};

static DEFINE_MUTEX(rear_flickerhub_mutex);
static DEFINE_SPINLOCK(calibration_lock);

long rear_flickerhub_read_flicker(u16 *flicker)
{
	long res = 0;
	struct data_unit_t data_t;
	res = sensor_get_data_from_hub(ID_REAR_REAR_FLICKER, &data_t);
	if (res < 0) {
		*flicker = -1;
		pr_err("sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_REAR_REAR_FLICKER, CUST_ACTION_GET_RAW_DATA);
		return -1;
	}
	*flicker = data_t.data[0];//flicker raw data;
	pr_err("rear_flickerhub_read_flicker:flicker_raw data = %d\n",*flicker);
	return 0;
}

static ssize_t trace_show(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;

	if (!obj_ipi_data) {
		pr_err("obj_ipi_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t trace_store(struct device_driver *ddri,
				const char *buf, size_t count)
{
	int trace = 0;
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;
	int res = 0;
	int ret = 0;

	if (!obj) {
		pr_err("obj_ipi_data is null!!\n");
		return 0;
	}
	ret = sscanf(buf, "0x%x", &trace);
	if (ret != 1) {
		pr_err("invalid content: '%s', length = %zu\n", buf, count);
		return count;
	}
	atomic_set(&obj->trace, trace);
	res = sensor_set_cmd_to_hub(ID_REAR_REAR_FLICKER, CUST_ACTION_SET_TRACE, &trace);
	if (res < 0) {
		pr_err("sensor_set_cmd_to_hub fail,(ID: %d),(action: %d)\n",
			ID_REAR_REAR_FLICKER, CUST_ACTION_SET_TRACE);
		return 0;
	}
	return count;
}

static ssize_t rear_flicker_show(struct device_driver *ddri, char *buf)
{
	int res = 0;
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;

	if (!obj) {
		pr_err("obj_ipi_data is null!!\n");
		return 0;
	}
	res = rear_flickerhub_read_flicker(&obj->rear_flicker);
	if (res)
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	else
#ifndef OPLUS_FEATURE_SENSOR
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->rear_flicker);
#else
		return snprintf(buf, PAGE_SIZE, "%u\n", obj->rear_flicker);
#endif
}

static ssize_t flickerlv_show(struct device_driver *ddri, char *buf)
{
	int res = 0;

	res = sensor_set_cmd_to_hub(ID_REAR_REAR_FLICKER, CUST_ACTION_SHOW_ALSLV, buf);
	if (res < 0) {
		pr_err("sensor_set_cmd_to_hub fail,(ID: %d),(action: %d)\n",
			ID_LIGHT, CUST_ACTION_SHOW_ALSLV);
		return 0;
	}

	return res;
}

static ssize_t flickerval_show(struct device_driver *ddri, char *buf)
{
	int res = 0;

	res = sensor_set_cmd_to_hub(ID_REAR_REAR_FLICKER, CUST_ACTION_SHOW_ALSVAL, buf);
	if (res < 0) {
		pr_err("sensor_set_cmd_to_hub fail,(ID: %d),(action: %d)\n",
			ID_REAR_REAR_FLICKER, CUST_ACTION_SHOW_ALSVAL);
		return 0;
	}

	return res;
}

static DRIVER_ATTR_RO(rear_flicker);
static DRIVER_ATTR_RO(flickerlv);
static DRIVER_ATTR_RO(flickerval);
static DRIVER_ATTR_RW(trace);

static struct driver_attribute *rear_flickerhub_attr_list[] = {
	&driver_attr_rear_flicker,
	&driver_attr_trace,	/*trace log */
	&driver_attr_flickerlv,
	&driver_attr_flickerval,
};

static int rear_flickerhub_create_attr(struct device_driver *driver)
{
	int idx = 0, err = 0;
	int num = (int)(ARRAY_SIZE(rear_flickerhub_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, rear_flickerhub_attr_list[idx]);
		if (err) {
			pr_err("driver_create_file (%s) = %d\n",
				rear_flickerhub_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int rear_flickerhub_delete_attr(struct device_driver *driver)
{
	int idx = 0, err = 0;
	int num = (int)(ARRAY_SIZE(rear_flickerhub_attr_list));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, rear_flickerhub_attr_list[idx]);

	return err;
}

static void rear_flickerhub_init_done_work(struct work_struct *work)
{
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;

	if (atomic_read(&obj->scp_init_done) == 0) {
		pr_err("wait for nvram to set calibration\n");
		return;
	}
	if (atomic_xchg(&obj->first_ready_after_boot, 1) == 0)
		return;
}

static int flicker_recv_data(struct data_unit_t *event, void *reserved)
{
	int err = 0;
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;
	/* pr_err("%s\n", __func__); */
	if (!obj)
		return 0;

	if (event->flush_action == FLUSH_ACTION)
		err = rear_flicker_flush_report();
	else if ((event->flush_action == DATA_ACTION) &&
			READ_ONCE(obj->rear_flicker_android_enable) == true)
		err = rear_flicker_data_report_t(event->data,
				SENSOR_STATUS_ACCURACY_MEDIUM,
				(int64_t)event->time_stamp);
	return err;
}

/*
static int flickerhub_factory_enable_sensor(bool enable_disable,
				int64_t sample_periods_ms)
{
	int err = 0;
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;

	if (enable_disable == true)
		WRITE_ONCE(obj->rear_flicker_factory_enable, true);
	else
		WRITE_ONCE(obj->rear_flicker_factory_enable, false);

	if (enable_disable == true) {
		err = sensor_set_delay_to_hub(ID_REAR_REAR_FLICKER, sample_periods_ms);
		if (err) {
			pr_err("sensor_set_delay_to_hub failed!\n");
			return -1;
		}
	}
	err = sensor_enable_to_hub(ID_REAR_REAR_FLICKER, enable_disable);
	if (err) {
		pr_err("sensor_enable_to_hub failed!\n");
		return -1;
	}
	return 0;
}
static int flickerhub_factory_get_data(int32_t *data)
{
	int err = 0;
	struct data_unit_t data_t;

	err = sensor_get_data_from_hub(ID_REAR_REAR_FLICKER, &data_t);
	if (err < 0)
		return -1;
	*data = data_t.light;
	return 0;
}
static int flickerhub_factory_get_raw_data(int32_t *data)
{
	return flickerhub_factory_get_data(data);
}
static int flickerhub_factory_enable_calibration(void)
{
	return sensor_calibration_to_hub(ID_REAR_REAR_FLICKER);
}
static int flickerhub_factory_clear_cali(void)
{
	return 0;
}

static int flickerhub_factory_set_cali(int32_t flicker_factor)
{
	int ret = 0;
	int cfg_data = 0;
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;

	update_sensor_parameter();
	obj->rear_flicker_factor = flicker_factor;
	cfg_data = flicker_factor;
	sensor_cfg_to_hub(ID_REAR_REAR_FLICKER,(uint8_t *)&cfg_data, sizeof(cfg_data));

	pr_err("flicker_factor = %d\n", obj->rear_flicker_factor);
	return ret;
}
static int flickerhub_factory_get_cali(int32_t data[6])
{
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;
	spin_lock(&calibration_lock);
	data[0] = obj->rear_flicker_factor;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	spin_unlock(&calibration_lock);
	return 0;
}

static struct rear_flicker_factory_fops rear_flickerhub_factory_fops = {
	.rear_flicker_enable_sensor = flickerhub_factory_enable_sensor,
	.rear_flicker_get_data = flickerhub_factory_get_data,
	.rear_flicker_get_raw_data = flickerhub_factory_get_raw_data,
	.rear_flicker_enable_calibration = flickerhub_factory_enable_calibration,
	.rear_flicker_clear_cali = flickerhub_factory_clear_cali,
	.rear_flicker_set_cali = flickerhub_factory_set_cali,
	.rear_flicker_get_cali = flickerhub_factory_get_cali,
};

static struct rear_flicker_factory_public rear_flickerhub_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &rear_flickerhub_factory_fops,
};
*/
static int flicker_open_report_data(int open)
{
	return 0;
}


static int flicker_enable_nodata(int en)
{
	int res = 0;
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;

	pr_debug("obj_ipi_data flicker enable value = %d\n", en);

	if (en == true)
		WRITE_ONCE(obj->rear_flicker_android_enable, true);
	else
		WRITE_ONCE(obj->rear_flicker_android_enable, false);

	res = sensor_enable_to_hub(ID_REAR_REAR_FLICKER, en);
	if (res < 0) {
		pr_err("%s is failed!!\n", __func__);
		return -1;
	}

	return 0;
}

static int flicker_set_delay(u64 ns)
{
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	int err = 0;
	unsigned int delayms = 0;

	delayms = (unsigned int)ns / 1000 / 1000;
	err = sensor_set_delay_to_hub(ID_REAR_REAR_FLICKER, delayms);
	if (err) {
		pr_err("%s fail!\n", __func__);
		return err;
	}
	pr_debug("%s (%d)\n", __func__, delayms);
	return 0;
#elif defined CONFIG_NANOHUB
	return 0;
#else
	return 0;
#endif
}
static int flicker_batch(int flag,
	int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	flicker_set_delay(samplingPeriodNs);
#endif
	return sensor_batch_to_hub(ID_REAR_REAR_FLICKER, flag,
		samplingPeriodNs, maxBatchReportLatencyNs);
}

static int flicker_flush(void)
{
	return sensor_flush_to_hub(ID_REAR_REAR_FLICKER);
}

static int flicker_set_cali(uint8_t *data, uint8_t count)
{
	int32_t *buf = (int32_t *)data;
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;

	spin_lock(&calibration_lock);
	atomic_set(&obj->rear_flicker_cali, buf[0]);
	spin_unlock(&calibration_lock);
	return sensor_cfg_to_hub(ID_REAR_REAR_FLICKER, data, count);
}

static int flicker_get_data(int *value, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_REAR_REAR_FLICKER, &data);
	if (err) {
		pr_err("sensor_get_data_from_hub fail!\n");
	} else {
		time_stamp = data.time_stamp;
		*value = data.data[0];
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
		pr_err("rear_flickerhub_read_flicker:rear_flicker data = %d,%d,%d,%d,%d,%d,%d,%d\n",
			value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]);
	}

	return 0;
}

static int scp_ready_event(uint8_t event, void *ptr)
{
	struct rear_flickerhub_ipi_data *obj = obj_ipi_data;

	switch (event) {
	case SENSOR_POWER_UP:
	    atomic_set(&obj->scp_init_done, 1);
		schedule_work(&obj->init_done_work);
		break;
	case SENSOR_POWER_DOWN:
	    atomic_set(&obj->scp_init_done, 0);
		break;
	}
	return 0;
}

static struct scp_power_monitor scp_ready_notifier = {
	.name = "rear_flicker",
	.notifier_call = scp_ready_event,
};

static int rear_flickerhub_probe(struct platform_device *pdev)
{
	struct rear_flickerhub_ipi_data *obj;
	struct platform_driver *paddr =
			rear_flickerhub_init_info.platform_diver_addr;

	int err = 0;
	struct rear_flicker_control_path flicker_ctl = { 0 };
	struct rear_flicker_data_path flicker_data = { 0 };

	pr_err("%s\n", __func__);
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(*obj));
	obj_ipi_data = obj;

	INIT_WORK(&obj->init_done_work, rear_flickerhub_init_done_work);

	platform_set_drvdata(pdev, obj);


	atomic_set(&obj->rear_flicker_suspend, 0);
	atomic_set(&obj->scp_init_done, 0);
	atomic_set(&obj->first_ready_after_boot, 0);

	obj->pending_intr = 0;
	WRITE_ONCE(obj->rear_flicker_factory_enable, false);
	WRITE_ONCE(obj->rear_flicker_android_enable, false);

	scp_power_monitor_register(&scp_ready_notifier);

	err = scp_sensorHub_data_registration(ID_REAR_REAR_FLICKER, flicker_recv_data);
	if (err < 0) {
		pr_err("scp_sensorHub_data_registration failed\n");
		goto exit_kfree;
	}
	/*
	err = rear_flicker_factory_device_register(&rear_flickerhub_factory_device);
	if (err) {
		pr_err("flickerps_factory_device_register register failed\n");
		goto exit_kfree;
	}
	*/
	pr_debug("rear_flickerhub_misc_device misc_register OK!\n");
	flicker_ctl.is_use_common_factory = false;
	err = rear_flickerhub_create_attr(&paddr->driver);
	if (err) {
		pr_err("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	flicker_ctl.open_report_data = flicker_open_report_data;
	flicker_ctl.enable_nodata = flicker_enable_nodata;
	flicker_ctl.set_delay = flicker_set_delay;
	flicker_ctl.batch = flicker_batch;
	flicker_ctl.flush = flicker_flush;
	flicker_ctl.set_cali = flicker_set_cali;
	flicker_ctl.is_report_input_direct = false;

	flicker_ctl.is_support_batch = false;

	err = rear_flicker_register_control_path(&flicker_ctl);
	if (err) {
		pr_err("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	flicker_data.get_data = flicker_get_data;
	flicker_data.vender_div = 100;
	err = rear_flicker_register_data_path(&flicker_data);
	if (err) {
		pr_err("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	rear_flickerhub_init_flag = 0;
	pr_debug("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	rear_flickerhub_delete_attr(&(rear_flickerhub_init_info.platform_diver_addr->driver));
exit_kfree:
	kfree(obj);
	obj_ipi_data = NULL;
exit:
	pr_err("%s: err = %d\n", __func__, err);
	rear_flickerhub_init_flag = -1;
	return err;
}

static int rear_flickerhub_remove(struct platform_device *pdev)
{
	int err = 0;
	struct platform_driver *paddr =
			rear_flickerhub_init_info.platform_diver_addr;

	err = rear_flickerhub_delete_attr(&paddr->driver);
	if (err)
		pr_err("rear_flickerhub_delete_attr fail: %d\n", err);
	// rear_flicker_factory_device_deregister(&rear_flickerhub_factory_device);
	obj_ipi_data = NULL;
	kfree(platform_get_drvdata(pdev));
	return 0;

}

static int rear_flickerhub_suspend(struct platform_device *pdev, pm_message_t msg)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int rear_flickerhub_resume(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	return 0;
}
static struct platform_device rear_flickerhub_device = {
	.name = REAR_FLICKERHUB_DEV_NAME,
	.id = -1,
};

static struct platform_driver rear_flickerhub_driver = {
	.probe = rear_flickerhub_probe,
	.remove = rear_flickerhub_remove,
	.suspend = rear_flickerhub_suspend,
	.resume = rear_flickerhub_resume,
	.driver = {
		.name = REAR_FLICKERHUB_DEV_NAME,
	},
};

static int rear_flickerhub_local_init(void)
{

	if (platform_driver_register(&rear_flickerhub_driver)) {
		pr_err("add driver error\n");
		return -1;
	}
	if (-1 == rear_flickerhub_init_flag)
		return -1;
	return 0;
}
static int rear_flickerhub_local_remove(void)
{

	platform_driver_unregister(&rear_flickerhub_driver);
	return 0;
}

static int __init rear_flickerhub_init(void)
{
	if (platform_device_register(&rear_flickerhub_device)) {
		pr_err("flickerps platform device error\n");
		return -1;
	}
	rear_flicker_driver_add(&rear_flickerhub_init_info);
	return 0;
}

static void __exit rear_flickerhub_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(rear_flickerhub_init);
module_exit(rear_flickerhub_exit);
MODULE_AUTHOR("hongxu.zhao@mediatek.com");
MODULE_DESCRIPTION("rear_flickerhub driver");
MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) "[REAR_ALS] " fmt

#include "rear_alshub.h"
#include <rear_als.h>
#include <hwmsensor.h>
#include <SCP_sensorHub.h>
#include "SCP_power_monitor.h"
#include <linux/pm_wakeup.h>
#ifdef OPLUS_FEATURE_SENSOR
#include "../../oplus_sensor_devinfo/sensor_devinfo.h"
#endif
#define REAR_ALSHUB_DEV_NAME     "rear_als_hub_pl"

struct rear_alshub_ipi_data {
	struct work_struct init_done_work;
	atomic_t first_ready_after_boot;
	/*misc */
	atomic_t	rear_als_suspend;
	atomic_t	trace;
	atomic_t	scp_init_done;

	/*data */
	u16		rear_als;
	int		rear_als_factor;
	atomic_t	rear_als_cali;
	ulong		enable;
	ulong		pending_intr;
	bool rear_als_factory_enable;
	bool rear_als_android_enable;
};

static struct rear_alshub_ipi_data *obj_ipi_data;


static int rear_alshub_local_init(void);
static int rear_alshub_local_remove(void);
static int rear_alshub_init_flag = -1;
static struct rear_als_init_info rear_alshub_init_info = {
	.name = "rear_als_hub",
	.init = rear_alshub_local_init,
	.uninit = rear_alshub_local_remove,

};

static DEFINE_MUTEX(rear_alshub_mutex);
static DEFINE_SPINLOCK(calibration_lock);

long rear_alshub_read_als(u16 *als)
{
	long res = 0;
	struct data_unit_t data_t;
	res = sensor_get_data_from_hub(ID_REAR_ALS, &data_t);
	if (res < 0) {
		*als = -1;
		pr_err("sensor_set_cmd_to_hub fail, (ID: %d),(action: %d)\n", ID_REAR_ALS, CUST_ACTION_GET_RAW_DATA);
		return -1;
	}
	*als = data_t.data[0];//als raw data;
	pr_err("rear_alshub_read_als:als_raw data = %d\n",*als);
	return 0;
}

static ssize_t trace_show(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;
	struct rear_alshub_ipi_data *obj = obj_ipi_data;

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
	struct rear_alshub_ipi_data *obj = obj_ipi_data;
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
	res = sensor_set_cmd_to_hub(ID_REAR_ALS, CUST_ACTION_SET_TRACE, &trace);
	if (res < 0) {
		pr_err("sensor_set_cmd_to_hub fail,(ID: %d),(action: %d)\n",
			ID_REAR_ALS, CUST_ACTION_SET_TRACE);
		return 0;
	}
	return count;
}

static ssize_t rear_als_show(struct device_driver *ddri, char *buf)
{
	int res = 0;
	struct rear_alshub_ipi_data *obj = obj_ipi_data;

	if (!obj) {
		pr_err("obj_ipi_data is null!!\n");
		return 0;
	}
	res = rear_alshub_read_als(&obj->rear_als);
	if (res)
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	else
#ifndef OPLUS_FEATURE_SENSOR
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->rear_als);
#else
		return snprintf(buf, PAGE_SIZE, "%u\n", obj->rear_als);
#endif
}

static ssize_t alslv_show(struct device_driver *ddri, char *buf)
{
	int res = 0;

	res = sensor_set_cmd_to_hub(ID_REAR_ALS, CUST_ACTION_SHOW_ALSLV, buf);
	if (res < 0) {
		pr_err("sensor_set_cmd_to_hub fail,(ID: %d),(action: %d)\n",
			ID_LIGHT, CUST_ACTION_SHOW_ALSLV);
		return 0;
	}

	return res;
}

static ssize_t alsval_show(struct device_driver *ddri, char *buf)
{
	int res = 0;

	res = sensor_set_cmd_to_hub(ID_REAR_ALS, CUST_ACTION_SHOW_ALSVAL, buf);
	if (res < 0) {
		pr_err("sensor_set_cmd_to_hub fail,(ID: %d),(action: %d)\n",
			ID_REAR_ALS, CUST_ACTION_SHOW_ALSVAL);
		return 0;
	}

	return res;
}

#ifdef OPLUS_FEATURE_SENSOR
static ssize_t gain_als_show(struct device_driver *ddri, char *buf)
{
	struct cali_data c_data;
	get_sensor_parameter(&c_data);
	return scnprintf(buf, PAGE_SIZE, "%d\n",c_data.als_factor);
}
#endif /* OPLUS_FEATURE_SENSOR */
static DRIVER_ATTR_RO(rear_als);
static DRIVER_ATTR_RO(alslv);
static DRIVER_ATTR_RO(alsval);
static DRIVER_ATTR_RW(trace);
#ifdef OPLUS_FEATURE_SENSOR
static DRIVER_ATTR_RO(gain_als);
#endif /* OPLUS_FEATURE_SENSOR */
static struct driver_attribute *rear_alshub_attr_list[] = {
	&driver_attr_rear_als,
	&driver_attr_trace,	/*trace log */
	&driver_attr_alslv,
	&driver_attr_alsval,
#ifdef OPLUS_FEATURE_SENSOR
	&driver_attr_gain_als,
#endif /* OPLUS_FEATURE_SENSOR */
};

static int rear_alshub_create_attr(struct device_driver *driver)
{
	int idx = 0, err = 0;
	int num = (int)(ARRAY_SIZE(rear_alshub_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, rear_alshub_attr_list[idx]);
		if (err) {
			pr_err("driver_create_file (%s) = %d\n",
				rear_alshub_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int rear_alshub_delete_attr(struct device_driver *driver)
{
	int idx = 0, err = 0;
	int num = (int)(ARRAY_SIZE(rear_alshub_attr_list));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, rear_alshub_attr_list[idx]);

	return err;
}

static void rear_alshub_init_done_work(struct work_struct *work)
{
	struct rear_alshub_ipi_data *obj = obj_ipi_data;
	int err = 0;
#ifndef MTK_OLD_FACTORY_CALIBRATION
#ifdef OPLUS_FEATURE_SENSOR
	struct cali_data c_data;
	int32_t cfg_data[6] = {0};
#else
	int32_t cfg_data[2] = {0};
#endif /*OPLUS_FEATURE_SENSOR*/
#endif

	if (atomic_read(&obj->scp_init_done) == 0) {
		pr_err("wait for nvram to set calibration\n");
		return;
	}
	if (atomic_xchg(&obj->first_ready_after_boot, 1) == 0)
		return;
#ifdef OPLUS_FEATURE_SENSOR
	update_sensor_parameter();
	get_sensor_parameter(&c_data);
	if (c_data.als_factor > 0) {
		memset(cfg_data, 0, sizeof(int) * 6);
		cfg_data[0] = c_data.als_factor;
		err = sensor_cfg_to_hub(ID_REAR_ALS,(uint8_t *)&cfg_data, sizeof(cfg_data));
		pr_err("set als factory cali %d, res = %d\n",
				cfg_data[0], err);
	}
#else
	spin_lock(&calibration_lock);
	cfg_data[0] = atomic_read(&obj->rear_als_cali);
	spin_unlock(&calibration_lock);
	err = sensor_cfg_to_hub(ID_REAR_ALS,
		(uint8_t *)cfg_data, sizeof(cfg_data));
	if (err < 0)
		pr_err("sensor_cfg_to_hub als fail\n");
#endif
}

static int als_recv_data(struct data_unit_t *event, void *reserved)
{
	int err = 0;
	struct rear_alshub_ipi_data *obj = obj_ipi_data;

	if (!obj)
		return 0;

	if (event->flush_action == FLUSH_ACTION)
		err = rear_als_flush_report();
	else if ((event->flush_action == DATA_ACTION) &&
			READ_ONCE(obj->rear_als_android_enable) == true)
		err = rear_als_data_report_t(event->light,
				SENSOR_STATUS_ACCURACY_MEDIUM,
				(int64_t)event->time_stamp);
	else if (event->flush_action == CALI_ACTION) {
		spin_lock(&calibration_lock);
		atomic_set(&obj->rear_als_cali, event->data[0]);
		spin_unlock(&calibration_lock);
		err = rear_als_cali_report(event->data);
	}
	return err;
}

static int alshub_factory_enable_sensor(bool enable_disable,
				int64_t sample_periods_ms)
{
	int err = 0;
	struct rear_alshub_ipi_data *obj = obj_ipi_data;

	if (enable_disable == true)
		WRITE_ONCE(obj->rear_als_factory_enable, true);
	else
		WRITE_ONCE(obj->rear_als_factory_enable, false);

	if (enable_disable == true) {
		err = sensor_set_delay_to_hub(ID_REAR_ALS, sample_periods_ms);
		if (err) {
			pr_err("sensor_set_delay_to_hub failed!\n");
			return -1;
		}
	}
	err = sensor_enable_to_hub(ID_REAR_ALS, enable_disable);
	if (err) {
		pr_err("sensor_enable_to_hub failed!\n");
		return -1;
	}
	return 0;
}
static int alshub_factory_get_data(int32_t *data)
{
	int err = 0;
	struct data_unit_t data_t;

	err = sensor_get_data_from_hub(ID_REAR_ALS, &data_t);
	if (err < 0)
		return -1;
	*data = data_t.light;
	return 0;
}
static int alshub_factory_get_raw_data(int32_t *data)
{
	return alshub_factory_get_data(data);
}
static int alshub_factory_enable_calibration(void)
{
	return sensor_calibration_to_hub(ID_REAR_ALS);
}
static int alshub_factory_clear_cali(void)
{
	return 0;
}

static int alshub_factory_set_cali(int32_t als_factor)
{
	int ret = 0;
	int cfg_data = 0;
	struct rear_alshub_ipi_data *obj = obj_ipi_data;

	update_sensor_parameter();
	obj->rear_als_factor = als_factor;
	cfg_data = als_factor;
	sensor_cfg_to_hub(ID_REAR_ALS,(uint8_t *)&cfg_data, sizeof(cfg_data));

	pr_err("als_factor = %d\n", obj->rear_als_factor);
	return ret;
}
static int alshub_factory_get_cali(int32_t data[6])
{
	struct rear_alshub_ipi_data *obj = obj_ipi_data;
	spin_lock(&calibration_lock);
	data[0] = obj->rear_als_factor;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	spin_unlock(&calibration_lock);
	return 0;
}


static struct rear_als_factory_fops rear_alshub_factory_fops = {
	.rear_als_enable_sensor = alshub_factory_enable_sensor,
	.rear_als_get_data = alshub_factory_get_data,
	.rear_als_get_raw_data = alshub_factory_get_raw_data,
	.rear_als_enable_calibration = alshub_factory_enable_calibration,
	.rear_als_clear_cali = alshub_factory_clear_cali,
	.rear_als_set_cali = alshub_factory_set_cali,
	.rear_als_get_cali = alshub_factory_get_cali,
};

static struct rear_als_factory_public rear_alshub_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &rear_alshub_factory_fops,
};
static int als_open_report_data(int open)
{
	return 0;
}


static int als_enable_nodata(int en)
{
	int res = 0;
	struct rear_alshub_ipi_data *obj = obj_ipi_data;

	pr_debug("obj_ipi_data als enable value = %d\n", en);

	if (en == true)
		WRITE_ONCE(obj->rear_als_android_enable, true);
	else
		WRITE_ONCE(obj->rear_als_android_enable, false);

	res = sensor_enable_to_hub(ID_REAR_ALS, en);
	if (res < 0) {
		pr_err("%s is failed!!\n", __func__);
		return -1;
	}

	return 0;
}

static int als_set_delay(u64 ns)
{
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	int err = 0;
	unsigned int delayms = 0;

	delayms = (unsigned int)ns / 1000 / 1000;
	err = sensor_set_delay_to_hub(ID_REAR_ALS, delayms);
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
static int als_batch(int flag,
	int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	als_set_delay(samplingPeriodNs);
#endif
	return sensor_batch_to_hub(ID_REAR_ALS, flag,
		samplingPeriodNs, maxBatchReportLatencyNs);
}

static int als_flush(void)
{
	return sensor_flush_to_hub(ID_REAR_ALS);
}

static int als_set_cali(uint8_t *data, uint8_t count)
{
	int32_t *buf = (int32_t *)data;
	struct rear_alshub_ipi_data *obj = obj_ipi_data;

	spin_lock(&calibration_lock);
	atomic_set(&obj->rear_als_cali, buf[0]);
	spin_unlock(&calibration_lock);
	return sensor_cfg_to_hub(ID_REAR_ALS, data, count);
}

static int als_get_data(int *value, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_REAR_ALS, &data);
	if (err) {
		pr_err("sensor_get_data_from_hub fail!\n");
	} else {
		time_stamp = data.time_stamp;
		*value = data.light;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return 0;
}

static int scp_ready_event(uint8_t event, void *ptr)
{
	struct rear_alshub_ipi_data *obj = obj_ipi_data;

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
	.name = "rear_als",
	.notifier_call = scp_ready_event,
};

static int rear_alshub_probe(struct platform_device *pdev)
{
	struct rear_alshub_ipi_data *obj;
	struct platform_driver *paddr =
			rear_alshub_init_info.platform_diver_addr;

	int err = 0;
	struct rear_als_control_path als_ctl = { 0 };
	struct rear_als_data_path als_data = { 0 };

	pr_err("%s\n", __func__);
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(*obj));
	obj_ipi_data = obj;

	INIT_WORK(&obj->init_done_work, rear_alshub_init_done_work);

	platform_set_drvdata(pdev, obj);


	atomic_set(&obj->rear_als_suspend, 0);
	atomic_set(&obj->scp_init_done, 0);
	atomic_set(&obj->first_ready_after_boot, 0);

	obj->pending_intr = 0;
	WRITE_ONCE(obj->rear_als_factory_enable, false);
	WRITE_ONCE(obj->rear_als_android_enable, false);

	scp_power_monitor_register(&scp_ready_notifier);

	err = scp_sensorHub_data_registration(ID_REAR_ALS, als_recv_data);
	if (err < 0) {
		pr_err("scp_sensorHub_data_registration failed\n");
		goto exit_kfree;
	}
	err = rear_als_factory_device_register(&rear_alshub_factory_device);
	if (err) {
		pr_err("alsps_factory_device_register register failed\n");
		goto exit_kfree;
	}
	pr_debug("rear_alshub_misc_device misc_register OK!\n");
	als_ctl.is_use_common_factory = false;
	err = rear_alshub_create_attr(&paddr->driver);
	if (err) {
		pr_err("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.set_cali = als_set_cali;
	als_ctl.is_report_input_direct = false;

	als_ctl.is_support_batch = false;

	err = rear_als_register_control_path(&als_ctl);
	if (err) {
		pr_err("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = rear_als_register_data_path(&als_data);
	if (err) {
		pr_err("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	rear_alshub_init_flag = 0;
	pr_debug("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	rear_alshub_delete_attr(&(rear_alshub_init_info.platform_diver_addr->driver));
exit_kfree:
	kfree(obj);
	obj_ipi_data = NULL;
exit:
	pr_err("%s: err = %d\n", __func__, err);
	rear_alshub_init_flag = -1;
	return err;
}

static int rear_alshub_remove(struct platform_device *pdev)
{
	int err = 0;
	struct platform_driver *paddr =
			rear_alshub_init_info.platform_diver_addr;

	err = rear_alshub_delete_attr(&paddr->driver);
	if (err)
		pr_err("rear_alshub_delete_attr fail: %d\n", err);
	rear_als_factory_device_deregister(&rear_alshub_factory_device);
	obj_ipi_data = NULL;
	kfree(platform_get_drvdata(pdev));
	return 0;

}

static int rear_alshub_suspend(struct platform_device *pdev, pm_message_t msg)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int rear_alshub_resume(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	return 0;
}
static struct platform_device rear_alshub_device = {
	.name = REAR_ALSHUB_DEV_NAME,
	.id = -1,
};

static struct platform_driver rear_alshub_driver = {
	.probe = rear_alshub_probe,
	.remove = rear_alshub_remove,
	.suspend = rear_alshub_suspend,
	.resume = rear_alshub_resume,
	.driver = {
		.name = REAR_ALSHUB_DEV_NAME,
	},
};

static int rear_alshub_local_init(void)
{

	if (platform_driver_register(&rear_alshub_driver)) {
		pr_err("add driver error\n");
		return -1;
	}
	if (-1 == rear_alshub_init_flag)
		return -1;
	return 0;
}
static int rear_alshub_local_remove(void)
{

	platform_driver_unregister(&rear_alshub_driver);
	return 0;
}

static int __init rear_alshub_init(void)
{
	if (platform_device_register(&rear_alshub_device)) {
		pr_err("alsps platform device error\n");
		return -1;
	}
	rear_als_driver_add(&rear_alshub_init_info);
	return 0;
}

static void __exit rear_alshub_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(rear_alshub_init);
module_exit(rear_alshub_exit);
MODULE_AUTHOR("hongxu.zhao@mediatek.com");
MODULE_DESCRIPTION("rear_alshub driver");
MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) "<REAR_FLICKER> " fmt

#include "inc/rear_flicker.h"
struct rear_flicker_context *rear_flicker_context_obj /* = NULL*/;
struct platform_device *rear_flicker_pltfm_dev;
int last_rear_flicker_report_data = -1;

/* AAL default delay timer(nano seconds)*/
#define AAL_DELAY 200000000

static struct rear_flicker_init_info *rear_flicker_init_list[MAX_CHOOSE_REAR_FLICKER_NUM] = {0};

int rear_flicker_data_report_t(int *value, int status, int64_t time_stamp)
{
	int err = 0;
	struct rear_flicker_context *cxt = NULL;
	struct sensor_event event;

	memset(&event, 0, sizeof(struct sensor_event));

	cxt = rear_flicker_context_obj;
	event.time_stamp = time_stamp;
	{
		event.handle = ID_REAR_REAR_FLICKER;
		event.flush_action = DATA_ACTION;
		event.word[0] = value[0];
		event.word[1] = value[1];
		event.word[2] = value[2];
		event.word[3] = value[3];
		event.word[4] = value[4];
		event.word[5] = value[5];
		event.word[6] = value[6];
		event.word[7] = value[7];
		event.status = status;
		err = sensor_input_event(cxt->rear_flicker_mdev.minor, &event);
	}
	return err;
}
int rear_flicker_data_report(int *value, int status)
{
	return rear_flicker_data_report_t(value, status, 0);
}
int rear_flicker_cali_report(int *value)
{
	int err = 0;
	struct sensor_event event;

	memset(&event, 0, sizeof(struct sensor_event));
	event.handle = ID_REAR_REAR_FLICKER;
	event.flush_action = CALI_ACTION;
	event.word[0] = value[0];
	err = sensor_input_event(rear_flicker_context_obj->rear_flicker_mdev.minor, &event);
	return err;
}

int rear_flicker_flush_report(void)
{
	struct sensor_event event;
	int err = 0;

	memset(&event, 0, sizeof(struct sensor_event));

	event.handle = ID_REAR_REAR_FLICKER;
	event.flush_action = FLUSH_ACTION;
	err = sensor_input_event(rear_flicker_context_obj->rear_flicker_mdev.minor, &event);
	pr_debug_ratelimited("flush\n");
	return err;
}

static void rear_flicker_work_func(struct work_struct *work)
{
	struct rear_flicker_context *cxt = NULL;
	int value, status;
	int64_t nt;
	struct timespec time;
	int err;
	pr_err("%s start\n", __func__);

	cxt = rear_flicker_context_obj;
	if (cxt->rear_flicker_data.get_data == NULL) {
		pr_err("rear_flicker driver not register data path\n");
		return;
	}

	time.tv_sec = time.tv_nsec = 0;
	time = get_monotonic_coarse();
	nt = time.tv_sec * 1000000000LL + time.tv_nsec;
	/* add wake lock to make sure data can be read before system suspend */
	err = cxt->rear_flicker_data.get_data(&value, &status);
	if (err) {
		pr_err("get rear_flicker data fails!!\n");
		goto rear_flicker_loop;
	} else {
		cxt->drv_data.rear_flicker_data.values[0] = value;
		cxt->drv_data.rear_flicker_data.status = status;
		cxt->drv_data.rear_flicker_data.time = nt;
	}

	if (true == cxt->is_rear_flicker_first_data_after_enable) {
		cxt->is_rear_flicker_first_data_after_enable = false;
		/* filter -1 value */
		if (cxt->drv_data.rear_flicker_data.values[0] == REAR_FLICKER_INVALID_VALUE) {
			pr_debug(" read invalid data\n");
			goto rear_flicker_loop;
		}
	}
	/* pr_debug(" flicker data[%d]\n" , cxt->drv_data.rear_flicker_data.values[0]); */
	rear_flicker_data_report(cxt->drv_data.rear_flicker_data.values,
			cxt->drv_data.rear_flicker_data.status);

rear_flicker_loop:
	if (true == cxt->is_rear_flicker_polling_run)
		mod_timer(&cxt->timer_rear_flicker,
			  jiffies + atomic_read(&cxt->delay_rear_flicker) / (1000 / HZ));
}

static void rear_flicker_poll(struct timer_list *t)
{
	struct rear_flicker_context *obj = from_timer(obj, t, timer_rear_flicker);

	if ((obj != NULL) && (obj->is_rear_flicker_polling_run))
		schedule_work(&obj->report_rear_flicker);
}

static struct rear_flicker_context *rear_flicker_context_alloc_object(void)
{
	struct rear_flicker_context *obj = kzalloc(sizeof(*obj), GFP_KERNEL);

	pr_err("%s start\n", __func__);
	if (!obj) {
		pr_err("Alloc rear flicker object error!\n");
		return NULL;
	}
	atomic_set(&obj->delay_rear_flicker,
		   200); /*5Hz, set work queue delay time 200ms */
	atomic_set(&obj->wake, 0);
	INIT_WORK(&obj->report_rear_flicker, rear_flicker_work_func);
	timer_setup(&obj->timer_rear_flicker, rear_flicker_poll, 0);
	obj->timer_rear_flicker.expires =
		jiffies + atomic_read(&obj->delay_rear_flicker) / (1000 / HZ);

	obj->is_rear_flicker_first_data_after_enable = false;
	obj->is_rear_flicker_polling_run = false;
	mutex_init(&obj->rear_flicker_op_mutex);
	obj->is_rear_flicker_batch_enable = false; /* for batch mode init */
	obj->rear_flicker_power = 0;
	obj->rear_flicker_enable = 0;
	obj->rear_flicker_delay_ns = -1;
	obj->rear_flicker_latency_ns = -1;

	pr_err("%s end\n", __func__);
	return obj;
}

#if !defined(CONFIG_NANOHUB) || !defined(CONFIG_MTK_REAR_FLICKERHUB)
static int rear_flicker_enable_and_batch(void)
{
	struct rear_flicker_context *cxt = rear_flicker_context_obj;
	int err;

	/* rear_flicker_power on -> power off */
	if (cxt->rear_flicker_power == 1 && cxt->rear_flicker_enable == 0) {
		pr_debug("REAR FLICKER disable\n");
		/* stop polling firstly, if needed */
		if (cxt->rear_flicker_ctl.is_report_input_direct == false &&
		    cxt->is_rear_flicker_polling_run == true) {
			smp_mb(); /* for memory barrier */
			del_timer_sync(&cxt->timer_rear_flicker);
			smp_mb(); /* for memory barrier */
			cancel_work_sync(&cxt->report_rear_flicker);
			cxt->drv_data.rear_flicker_data.values[0] = REAR_FLICKER_INVALID_VALUE;
			cxt->is_rear_flicker_polling_run = false;
			pr_debug("rear flicker stop polling done\n");
		}
		/* turn off the rear_flicker_power */
		err = cxt->rear_flicker_ctl.enable_nodata(0);
		if (err) {
			pr_err("flicker turn off rear_flicker_power err = %d\n", err);
			return -1;
		}
		pr_debug("flicker turn off rear_flicker_power done\n");

		cxt->rear_flicker_power = 0;
		cxt->rear_flicker_delay_ns = -1;
		pr_debug("REAR FLICKER disable done\n");
		return 0;
	}
	/* rear_flicker_power off -> power on */
	if (cxt->rear_flicker_power == 0 && cxt->rear_flicker_enable == 1) {
		pr_debug("rear_flicker_power on\n");
		err = cxt->rear_flicker_ctl.enable_nodata(1);
		if (err) {
			pr_err("flicker turn on rear_flicker_power err = %d\n", err);
			return -1;
		}
		pr_debug("rear flicker turn on rear_flicker_power done\n");

		cxt->rear_flicker_power = 1;
		pr_debug("REAR FLICKER rear_flicker_power on done\n");
	}
	/* rate change */
	if (cxt->rear_flicker_power == 1 && cxt->rear_flicker_delay_ns >= 0) {
		pr_debug("REAR FLICKER set batch\n");
		/* set ODR, fifo timeout latency */
		if (cxt->rear_flicker_ctl.is_support_batch)
			err = cxt->rear_flicker_ctl.batch(0, cxt->rear_flicker_delay_ns,
						 cxt->rear_flicker_latency_ns);
		else
			err = cxt->rear_flicker_ctl.batch(0, cxt->rear_flicker_delay_ns, 0);
		if (err) {
			pr_err("flicker set batch(ODR) err %d\n", err);
			return -1;
		}
		pr_debug("flicker set ODR, fifo latency done\n");
		/* start polling, if needed */
		if (cxt->rear_flicker_ctl.is_report_input_direct == false) {
			uint64_t mdelay = cxt->rear_flicker_delay_ns;

			do_div(mdelay, 1000000);
			/* defaut max polling delay */
			if (mdelay < 10)
				mdelay = 10;
			atomic_set(&cxt->delay_rear_flicker, mdelay);
			/* the first sensor start polling timer */
			if (cxt->is_rear_flicker_polling_run == false) {
				mod_timer(&cxt->timer_rear_flicker,
					  jiffies +
						  atomic_read(&cxt->delay_rear_flicker) /
							  (1000 / HZ));
				cxt->is_rear_flicker_polling_run = true;
				cxt->is_rear_flicker_first_data_after_enable = true;
			}
			pr_debug("flicker set polling delay %d ms\n",
				  atomic_read(&cxt->delay_rear_flicker));
		}
		pr_debug("REAR FLICKER batch done\n");
	}
	return 0;
}
#endif

static ssize_t rear_flickeractive_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct rear_flicker_context *cxt = rear_flicker_context_obj;
	int err = 0;

	pr_debug("%s buf=%s\n", __func__, buf);
	mutex_lock(&rear_flicker_context_obj->rear_flicker_op_mutex);
	if (!strncmp(buf, "1", 1)) {
		cxt->rear_flicker_enable = 1;
		last_rear_flicker_report_data = -1;
	} else if (!strncmp(buf, "0", 1)) {
		cxt->rear_flicker_enable = 0;
	} else {
		pr_err("rear_flicker_store_active error !!\n");
		err = -1;
		goto err_out;
	}
#if defined(CONFIG_NANOHUB) && defined(CONFIG_MTK_REAR_FLICKERHUB)
	if (cxt->rear_flicker_enable) {
		err = cxt->rear_flicker_ctl.enable_nodata(cxt->rear_flicker_enable);
		if (err) {
			pr_err("flicker turn on err = %d\n", err);
			goto err_out;
		}
	} else {
		err = cxt->rear_flicker_ctl.enable_nodata(cxt->rear_flicker_enable);
		if (err) {
			pr_err("flicker turn off err = %d\n", err);
			goto err_out;
		}
	}
#else
		err = rear_flicker_enable_and_batch();
#endif
err_out:
	mutex_unlock(&rear_flicker_context_obj->rear_flicker_op_mutex);
	pr_debug("%s done\n", __func__);
	if (err)
		return err;
	else
		return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t rear_flickeractive_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct rear_flicker_context *cxt = NULL;
	int div = 0;

	cxt = rear_flicker_context_obj;
	div = cxt->rear_flicker_data.vender_div;
	pr_debug("rear_flicker vender_div value: %d\n", div);
	return snprintf(buf, PAGE_SIZE, "%d\n", div);
}

static ssize_t rear_flickerbatch_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct rear_flicker_context *cxt = rear_flicker_context_obj;
	int handle = 0, flag = 0, err = 0;
	#ifndef OPLUS_FEATURE_SENSOR_WISELIGHT
	int64_t delay_ns = 0;
	int64_t latency_ns = 0;
	#endif

	pr_debug("%s %s\n", __func__, buf);
	err = sscanf(buf, "%d,%d,%lld,%lld", &handle, &flag, &cxt->rear_flicker_delay_ns,
		     &cxt->rear_flicker_latency_ns);
	if (err != 4) {
		pr_err("%s param error: err = %d\n", __func__, err);
		return -1;
	}

	mutex_lock(&rear_flicker_context_obj->rear_flicker_op_mutex);
#if defined(CONFIG_NANOHUB) && defined(CONFIG_MTK_REAR_FLICKERHUB)
		if (cxt->rear_flicker_ctl.is_support_batch)
			err = cxt->rear_flicker_ctl.batch(0, cxt->rear_flicker_delay_ns,
				cxt->rear_flicker_latency_ns);
		else
			err = cxt->rear_flicker_ctl.batch(0, cxt->rear_flicker_delay_ns, 0);
	if (err)
		pr_err("rear_flicker set batch(ODR) err %d\n", err);
#else
		err = rear_flicker_enable_and_batch();
#endif
	mutex_unlock(&rear_flicker_context_obj->rear_flicker_op_mutex);
	pr_debug("%s done: %d\n", __func__, cxt->is_rear_flicker_batch_enable);
	if (err)
		return err;
	else
		return count;
}

static ssize_t rear_flickerbatch_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t rear_flickerflush_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct rear_flicker_context *cxt = NULL;
	int handle = 0, err = 0;

	err = kstrtoint(buf, 10, &handle);
	if (err != 0)
		pr_err("%s param error: err = %d\n", __func__, err);

	pr_debug("%s param: handle %d\n", __func__, handle);

	mutex_lock(&rear_flicker_context_obj->rear_flicker_op_mutex);
	cxt = rear_flicker_context_obj;
	if (cxt->rear_flicker_ctl.flush != NULL)
		err = cxt->rear_flicker_ctl.flush();
	else
		pr_err("DON'T SUPPORT FLICKER COMMON VERSION FLUSH\n");
	if (err < 0)
		pr_err("rear_flicker enable flush err %d\n", err);
	mutex_unlock(&rear_flicker_context_obj->rear_flicker_op_mutex);
	if (err)
		return err;
	else
		return count;
}

static ssize_t rear_flickerflush_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}
/* need work around again */
static ssize_t rear_flickerdevnum_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}
static ssize_t rear_flickercali_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct rear_flicker_context *cxt = NULL;
	int err = 0;
	uint8_t *cali_buf = NULL;

	cali_buf = vzalloc(count);
	if (!cali_buf)
		return -ENOMEM;
	memcpy(cali_buf, buf, count);

	mutex_lock(&rear_flicker_context_obj->rear_flicker_op_mutex);
	cxt = rear_flicker_context_obj;
	if (cxt->rear_flicker_ctl.set_cali != NULL)
		err = cxt->rear_flicker_ctl.set_cali(cali_buf, count);
	if (err < 0)
		pr_err("flicker set cali err %d\n", err);
	mutex_unlock(&rear_flicker_context_obj->rear_flicker_op_mutex);
	vfree(cali_buf);
	return count;
}


static int rear_light_remove(struct platform_device *pdev)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int rear_light_probe(struct platform_device *pdev)
{
	pr_err("%s\n", __func__);
	rear_flicker_pltfm_dev = pdev;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rear_flicker_of_match[] = {
	{
		.compatible = "mediatek,rear_flicker",
	},
	{},
};
#endif

static struct platform_driver rear_flicker_driver = {
	.probe = rear_light_probe,
	.remove = rear_light_remove,
	.driver = {

		.name = "rear_flicker",
#ifdef CONFIG_OF
		.of_match_table = rear_flicker_of_match,
#endif
	}
};

static int rear_flicker_real_driver_init(void)
{
	int i = 0;
	int err = 0;

	pr_err("%s start\n", __func__);
	for (i = 0; i < MAX_CHOOSE_REAR_FLICKER_NUM; i++) {
		pr_err("%s i=%d\n", __func__, i);
		if (rear_flicker_init_list[i] != 0) {
			pr_err(" rear flicker try to init driver %s\n",
				  rear_flicker_init_list[i]->name);
			err = rear_flicker_init_list[i]->init();
			if (err == 0) {
				pr_err(" rear flicker real driver %s probe ok\n",
					  rear_flicker_init_list[i]->name);
				break;
			}
		}
	}

	if (i == MAX_CHOOSE_REAR_FLICKER_NUM) {
		pr_err("%s fail\n", __func__);
		err = -1;
	}

	return err;
}

int rear_flicker_driver_add(struct rear_flicker_init_info *obj)
{
	int err = 0;
	int i = 0;

	pr_err("%s\n", __func__);

	if (!obj) {
		pr_err(
			"REAR FLICKER driver add fail, rear_flicker_init_info is NULL\n");
		return -1;
	}

	for (i = 0; i < MAX_CHOOSE_REAR_FLICKER_NUM; i++) {
		if ((i == 0) && (rear_flicker_init_list[0] == NULL)) {
			pr_debug("register rear flicker driver for the first time\n");
			if (platform_driver_register(&rear_flicker_driver))
				pr_err(
					"failed to register rear flicker driver already exist\n");
		}

		if (rear_flicker_init_list[i] == NULL) {
			obj->platform_diver_addr = &rear_flicker_driver;
			rear_flicker_init_list[i] = obj;
			break;
		}
	}
	if (i >= MAX_CHOOSE_REAR_FLICKER_NUM) {
		pr_err("REAR FLICKER driver add err\n");
		err = -1;
	}

	return err;
}
EXPORT_SYMBOL_GPL(rear_flicker_driver_add);
struct platform_device *get_rear_flicker_platformdev(void)
{
	return rear_flicker_pltfm_dev;
}

/*----------------------------------------------------------------------------*/
DEVICE_ATTR_RW(rear_flickeractive);
DEVICE_ATTR_RW(rear_flickerbatch);
DEVICE_ATTR_RW(rear_flickerflush);
DEVICE_ATTR_RO(rear_flickerdevnum);
DEVICE_ATTR_WO(rear_flickercali);


static struct attribute *rear_flicker_attributes[] = {
	&dev_attr_rear_flickeractive.attr,
	&dev_attr_rear_flickerbatch.attr,
	&dev_attr_rear_flickerflush.attr,
	&dev_attr_rear_flickerdevnum.attr,
	&dev_attr_rear_flickercali.attr,
	NULL
};


static struct attribute_group rear_flicker_attribute_group = {
	.attrs = rear_flicker_attributes
};

static int light_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t light_read(struct file *file, char __user *buffer, size_t count,
			  loff_t *ppos)
{
	ssize_t read_cnt = 0;

	read_cnt = sensor_event_read(rear_flicker_context_obj->rear_flicker_mdev.minor, file,
				     buffer, count, ppos);

	return read_cnt;
}

static unsigned int light_poll(struct file *file, poll_table *wait)
{
	return sensor_event_poll(rear_flicker_context_obj->rear_flicker_mdev.minor, file, wait);
}

static const struct file_operations light_fops = {
	.owner = THIS_MODULE,
	.open = light_open,
	.read = light_read,
	.poll = light_poll,
};

static int rear_flicker_misc_init(struct rear_flicker_context *cxt)
{
	int err = 0;

	cxt->rear_flicker_mdev.minor = ID_REAR_REAR_FLICKER;
	cxt->rear_flicker_mdev.name = REAR_FLICKER_MISC_DEV_NAME;
	cxt->rear_flicker_mdev.fops = &light_fops;
	err = sensor_attr_register(&cxt->rear_flicker_mdev);
	if (err)
		pr_err("unable to register rear_flicker misc device!!\n");

	return err;
}

int rear_flicker_register_data_path(struct rear_flicker_data_path *data)
{
	struct rear_flicker_context *cxt = NULL;
	/* int err =0; */
	cxt = rear_flicker_context_obj;
	cxt->rear_flicker_data.get_data = data->get_data;
	cxt->rear_flicker_data.vender_div = data->vender_div;
	cxt->rear_flicker_data.rear_flicker_get_raw_data = data->rear_flicker_get_raw_data;
	pr_err("rear_flicker register data path vender_div: %d\n",
		  cxt->rear_flicker_data.vender_div);
	if (cxt->rear_flicker_data.get_data == NULL) {
		pr_err("rear_flicker register data path fail\n");
		return -1;
	}
	return 0;
}


int rear_flicker_register_control_path(struct rear_flicker_control_path *ctl)
{
	struct rear_flicker_context *cxt = NULL;
	int err = 0;

	cxt = rear_flicker_context_obj;
	cxt->rear_flicker_ctl.set_delay = ctl->set_delay;
	cxt->rear_flicker_ctl.open_report_data = ctl->open_report_data;
	cxt->rear_flicker_ctl.enable_nodata = ctl->enable_nodata;
	cxt->rear_flicker_ctl.batch = ctl->batch;
	cxt->rear_flicker_ctl.flush = ctl->flush;
	cxt->rear_flicker_ctl.set_cali = ctl->set_cali;
	cxt->rear_flicker_ctl.is_support_batch = ctl->is_support_batch;
	cxt->rear_flicker_ctl.is_report_input_direct = ctl->is_report_input_direct;
	cxt->rear_flicker_ctl.is_use_common_factory = ctl->is_use_common_factory;

	if (cxt->rear_flicker_ctl.enable_nodata == NULL || cxt->rear_flicker_ctl.batch == NULL ||
	    cxt->rear_flicker_ctl.flush == NULL) {
		pr_err("flicker register control path fail\n");
		return -1;
	}

	/* add misc dev for sensor hal control cmd */
	err = rear_flicker_misc_init(rear_flicker_context_obj);
	if (err) {
		pr_err("unable to register rear_flicker misc device!!\n");
		return -2;
	}
	err = sysfs_create_group(&rear_flicker_context_obj->rear_flicker_mdev.this_device->kobj,
				 &rear_flicker_attribute_group);
	if (err < 0) {
		pr_err("unable to create rear_flicker attribute file\n");
		return -3;
	}
	kobject_uevent(&rear_flicker_context_obj->rear_flicker_mdev.this_device->kobj,
		       KOBJ_ADD);
	return 0;
}


static int rear_flicker_probe(void)
{
	int err;

	pr_err("%s start!!\n", __func__);
	rear_flicker_context_obj = rear_flicker_context_alloc_object();
	if (!rear_flicker_context_obj) {
		err = -ENOMEM;
		pr_err("unable to allocate devobj!\n");
		goto exit_alloc_data_failed;
	}
	/* init real rear_flicker driver */
	err = rear_flicker_real_driver_init();
	if (err) {
		pr_err("rear_flicker real driver init fail\n");
		goto real_driver_init_fail;
	}
	pr_err("%s OK !!\n", __func__);
	return 0;

real_driver_init_fail:
	kfree(rear_flicker_context_obj);
	rear_flicker_context_obj = NULL;
exit_alloc_data_failed:
	pr_err("%s fail !!!\n", __func__);
	return err;
}

static int rear_flicker_remove(void)
{
	int err = 0;

	pr_err("%s\n", __func__);
	sysfs_remove_group(&rear_flicker_context_obj->rear_flicker_mdev.this_device->kobj,
			   &rear_flicker_attribute_group);

	err = sensor_attr_deregister(&rear_flicker_context_obj->rear_flicker_mdev);
	if (err)
		pr_err("misc_deregister fail: %d\n", err);
	kfree(rear_flicker_context_obj);

	platform_driver_unregister(&rear_flicker_driver);

	return 0;
}

static int __init rear_flicker_init(void)
{
	pr_err("%s\n", __func__);

	if (rear_flicker_probe()) {
		pr_err("failed to register rear_flicker driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit rear_flicker_exit(void)
{
	rear_flicker_remove();
	platform_driver_unregister(&rear_flicker_driver);
}
late_initcall(rear_flicker_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("REAR FLICKER device driver");
MODULE_AUTHOR("Mediatek");

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) "<REAR_ALS> " fmt

#include "inc/rear_als_factory.h"

struct rear_als_factory_private {
	uint32_t gain;
	uint32_t sensitivity;
	struct rear_als_factory_fops *fops;
};

static struct rear_als_factory_private rear_als_factory;

static int rear_als_factory_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int rear_als_factory_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long rear_als_factory_unlocked_ioctl(struct file *file, unsigned int cmd,
					 unsigned long arg)
{
	long err = 0;
	void __user *ptr = (void __user *)arg;
	int data = 0;
	uint32_t enable = 0;
	#ifndef OPLUS_FEATURE_SENSOR
	int als_cali = 0;
	#else
	int32_t data_buf[6] = {0};
	#endif

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg,
				 _IOC_SIZE(cmd));

	if (err) {
		pr_debug("access error: %08X, (%2d, %2d)\n", cmd,
			  _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case REAR_ALS_SET_ALS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable)))
			return -EFAULT;
		if (rear_als_factory.fops != NULL &&
		    rear_als_factory.fops->rear_als_enable_sensor != NULL) {
			err = rear_als_factory.fops->rear_als_enable_sensor(enable,
								    200);
			if (err < 0) {
				pr_err("REAR_ALS_SET_ALS_MODE fail!\n");
				return -EINVAL;
			}
			pr_debug(
				"REAR_ALS_SET_ALS_MODE, enable: %d, sample_period:%dms\n",
				enable, 200);
		} else {
			pr_err("REAR_ALS_SET_ALS_MODE NULL\n");
			return -EINVAL;
		}
		return 0;
	case REAR_ALS_GET_ALS_RAW_DATA:
		if (rear_als_factory.fops != NULL &&
		    rear_als_factory.fops->rear_als_get_raw_data != NULL) {
			err = rear_als_factory.fops->rear_als_get_raw_data(&data);
			if (err < 0) {
				pr_err(
					"REAR_ALS_GET_ALS_RAW_DATA read data fail!\n");
				return -EINVAL;
			}
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("REAR_ALS_GET_ALS_RAW_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
#ifdef OPLUS_FEATURE_SENSOR
	case REAR_ALS_IOCTL_ALS_GET_CALI:
		if (rear_als_factory.fops != NULL && rear_als_factory.fops->rear_als_get_cali != NULL) {
			err = rear_als_factory.fops->rear_als_get_cali(data_buf);
			if (err < 0) {
				pr_err("REAR_ALS_IOCTL_ALS_GET_CALI fail!\n");
				return -EINVAL;
			}
			pr_err("REAR_ALS_IOCTL_ALS_GET_CALI, (%d, %d, %d)\n", data_buf[0], data_buf[1], data_buf[2]);
			if (copy_to_user(ptr, data_buf, sizeof(data_buf)))
				return -EFAULT;
		} else {
			pr_err("REAR_ALS_IOCTL_ALS_GET_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	case REAR_ALS_SET_CALI:
		if (copy_from_user(&data, ptr, sizeof(data)))
		{
			pr_err("REAR_ALS_ALS_SET_CALI, copy from user err.\n");
			return -EFAULT;
		}
		pr_err("REAR_ALS_ALS_SET_CALI, data = %d\n", data);
		if (rear_als_factory.fops != NULL && rear_als_factory.fops->rear_als_set_cali != NULL) {
			err = rear_als_factory.fops->rear_als_set_cali(data);
			if (err < 0) {
				pr_err("REAR_ALS_GET_ALS_RAW_DATA read data fail!\n");
				return -EINVAL;
			}
		} else {
			pr_err("REAR_ALS_GET_ALS_RAW_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
#endif /* OPLUS_FEATURE_SENSOR */
	case REAR_ALS_ENABLE_CALI:
		if (rear_als_factory.fops != NULL &&
		    rear_als_factory.fops->rear_als_enable_calibration != NULL) {
			err = rear_als_factory.fops->rear_als_enable_calibration();
			if (err < 0) {
				pr_err("REAR_ALS_ALS_ENABLE_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("REAR_ALS_ALS_ENABLE_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	#ifndef OPLUS_FEATURE_SENSOR
	case REAR_ALS_SET_CALI:
		if (copy_from_user(&als_cali, ptr, sizeof(als_cali)))
			return -EFAULT;
		if (rear_als_factory.fops != NULL &&
		    rear_als_factory.fops->rear_als_set_cali != NULL) {
			err = rear_als_factory.fops->rear_als_set_cali(als_cali);
			if (err < 0) {
				pr_err("REAR_ALS_ALS_SET_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("REAR_ALS_ALS_SET_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	#endif

	#ifndef OPLUS_FEATURE_SENSOR
	case REAR_ALS_IOCTL_ALS_GET_CALI:
		if (rear_als_factory.fops != NULL &&
			rear_als_factory.fops->rear_als_get_cali != NULL) {
			err = rear_als_factory.fops->rear_als_get_cali(&data);
			if (err < 0) {
				pr_err("REAR_ALS_IOCTL_ALS_GET_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("REAR_ALS_IOCTL_ALS_GET_CALI NULL\n");
			return -EINVAL;
		}
		if (copy_to_user(ptr, &data, sizeof(data)))
			return -EFAULT;
		return 0;
	#endif
	default:
		pr_err("unknown IOCTL: 0x%08x\n", cmd);
		return -ENOIOCTLCMD;
	}
	return 0;
}

static const struct file_operations _rear_als_factory_fops = {
	.open = rear_als_factory_open,
	.release = rear_als_factory_release,
	.unlocked_ioctl = rear_als_factory_unlocked_ioctl,
};

static struct miscdevice rear_als_factory_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rear_als",
	.fops = &_rear_als_factory_fops,
};

int rear_als_factory_device_register(struct rear_als_factory_public *dev)
{
	int err = 0;

	if (!dev || !dev->fops)
		return -1;
	rear_als_factory.gain = dev->gain;
	rear_als_factory.sensitivity = dev->sensitivity;
	rear_als_factory.fops = dev->fops;
	err = misc_register(&rear_als_factory_device);
	if (err) {
		pr_err("rear_als_factory_device register failed\n");
		err = -1;
	}
	return err;
}

int rear_als_factory_device_deregister(struct rear_als_factory_public *dev)
{
	rear_als_factory.fops = NULL;
	misc_deregister(&rear_als_factory_device);
	return 0;
}

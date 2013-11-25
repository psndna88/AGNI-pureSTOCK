/*
 * gpu_voltage_control.c -- gpu voltage control interface for the sgs2/3
 *
 *  Copyright (C) 2011 Michael Wodkins
 *  twitter - @xdanetarchy
 *  XDA-developers - netarchy
 *
 *  Modified for SiyahKernel
 *
 *  Modified by Andrei F. for Galaxy S3 / Perseus kernel (June 2012)
 *  Modified by Simone R. for Note 2 / NEAK Kernel with refactorings (Oct 2012)
 *  Modified by Jean-Pierre Rasquin for SGS3 / Yank555.lu Kernel with refactorings (Jul 2013)
 *
 *                                   - added full freq. range voltage table
 *                                   - added single-value sysfs interface
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of the GNU General Public License as published by the
 *  Free Software Foundation;
 *
 */

#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include "gpu_voltage_control.h"

extern unsigned int exynos_result_of_asv;

typedef struct mali_dvfs_tableTag{
    unsigned int clock;
    unsigned int freq;
    unsigned int vol;
}mali_dvfs_table;

typedef struct mali_dvfs_thresholdTag{
	unsigned int downthreshold;
	unsigned int upthreshold;
}mali_dvfs_threshold_table;

extern mali_dvfs_table mali_dvfs[MALI_DVFS_STEPS];
extern mali_dvfs_threshold_table mali_dvfs_threshold[MALI_DVFS_STEPS];

static ssize_t gpu_voltage_show(struct device *dev, struct device_attribute *attr, char *buf) {

	int i, len = 0;

	if(buf) {
		for(i = 0; i < MALI_DVFS_STEPS; i++)
			// Yank555.lu : Use steps 0-4 as above to keep this consistent with gpu_clock_control
			//               and display the freq. of the step since we can change them
			len += sprintf(buf + len, "Step%d (%dMHz): %d\n", i, mali_dvfs[i].clock, mali_dvfs[i].vol);
	}
	return len;
}

static ssize_t gpu_voltage_store(struct device *dev, struct device_attribute *attr, const char *buf,
									size_t count) {
	unsigned int ret = -EINVAL;
	int i = 0;
	unsigned int gv[MALI_DVFS_STEPS];

	ret = sscanf(buf, "%d %d %d %d %d", &gv[0], &gv[1], &gv[2], &gv[3], &gv[4]);

	if(ret != MALI_DVFS_STEPS)
		return -EINVAL;

	/* safety floor and ceiling - netarchy */
	for( i = 0; i < MALI_DVFS_STEPS; i++ ) {
		if (gv[i] < MIN_VOLTAGE_GPU) {
		    gv[i] = MIN_VOLTAGE_GPU;
		}
		else if (gv[i] > MAX_VOLTAGE_GPU) {
		    gv[i] = MAX_VOLTAGE_GPU;
		}
		mali_dvfs[i].vol = gv[i];
	}
	return count;
}

static DEVICE_ATTR(gpu_control, S_IRUGO | S_IWUGO, gpu_voltage_show, gpu_voltage_store);

// Yank555.lu : add sysfs entry to display the ASV level used to determinate the voltage
static ssize_t asv_level_show(struct device *dev, struct device_attribute *attr, char *buf) {

	return sprintf(buf, "ASV level used for GPU voltage : %d\n",exynos_result_of_asv);

}

static DEVICE_ATTR(asv_level, S_IRUGO, asv_level_show, NULL);

// Yank555.lu : add voltage table reset according to ASV level and default table
static ssize_t mali_dvfs_table_update_store(struct device *dev, struct device_attribute *attr, const char *buf,
									size_t count) {
	unsigned int data;
	unsigned int ret;

	ret = sscanf(buf, "%u\n", &data);

	if (!ret) {
		return -EINVAL;
	}

	if (data == 1) {
		// Yank555.lu : reupdate mali dvfs table
		mali_dvfs_table_update();
		return count;
	}

	return -EINVAL;

}

static DEVICE_ATTR(mali_dvfs_table_update, S_IWUGO, NULL, mali_dvfs_table_update_store);

// GPU voltage steps

#define expose_gpu_voltage(step)									\
static ssize_t show_gpu_voltage_##step									\
(struct device *dev, struct device_attribute *attr, char *buf) {					\
													\
	return sprintf(buf, "%d\n", mali_dvfs[step].vol);						\
													\
}													\
													\
static ssize_t store_gpu_voltage_##step									\
(struct device *dev, struct device_attribute *attr,							\
			       const char *buf, size_t count) {						\
													\
	unsigned int input;										\
													\
	if (sscanf(buf, "%u", &input) != 1)								\
		return -EINVAL;										\
													\
	if (input < MIN_VOLTAGE_GPU || input > MAX_VOLTAGE_GPU)						\
		return -EINVAL;										\
													\
	mali_dvfs[step].vol = input;									\
	return count;											\
}													\
													\
static DEVICE_ATTR(gpu_voltage_##step									\
		 , S_IRUGO | S_IWUGO									\
		 , show_gpu_voltage_##step								\
		 , store_gpu_voltage_##step								\
);

expose_gpu_voltage(0);
expose_gpu_voltage(1);
expose_gpu_voltage(2);
expose_gpu_voltage(3);
expose_gpu_voltage(4);

static struct attribute *gpu_voltage_control_attributes[] = {
	&dev_attr_gpu_control.attr,
	&dev_attr_asv_level.attr,
	&dev_attr_mali_dvfs_table_update.attr,
	// Yank555.lu : new GPU voltage steps interface
	&dev_attr_gpu_voltage_0.attr,
	&dev_attr_gpu_voltage_1.attr,
	&dev_attr_gpu_voltage_2.attr,
	&dev_attr_gpu_voltage_3.attr,
	&dev_attr_gpu_voltage_4.attr,
	NULL
};

static struct attribute_group gpu_voltage_control_group = {
	.attrs = gpu_voltage_control_attributes,
};

static struct miscdevice gpu_voltage_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpu_voltage_control",
};

void gpu_voltage_control_start()
{
	printk("Initializing gpu voltage control interface\n");

	misc_register(&gpu_voltage_control_device);
	if (sysfs_create_group(&gpu_voltage_control_device.this_device->kobj,
				&gpu_voltage_control_group) < 0) {
		printk("%s sysfs_create_group failed\n", __FUNCTION__);
		pr_err("Unable to create group for %s\n", gpu_voltage_control_device.name);
	}
}


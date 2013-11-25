/*
 * gpu_clock_control.c -- a clock control interface for the sgs2/3
 *
 *  Copyright (C) 2011 Michael Wodkins
 *  twitter - @xdanetarchy
 *  XDA-developers - netarchy
 *  modified by gokhanmoral
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

#include "gpu_clock_control.h"

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

typedef struct mali_dvfs_staycount{
	unsigned int staycount;
}mali_dvfs_staycount_table;

extern mali_dvfs_staycount_table mali_dvfs_staycount[MALI_DVFS_STEPS];

static ssize_t gpu_clock_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf, "Step0: %d\n"
			    "Step1: %d\n"
			    "Step2: %d\n"
			    "Step3: %d\n"
			    "Step4: %d\n"
			    "Threshold0-1/up-down: %d%% %d%%\n"
			    "Threshold1-2/up-down: %d%% %d%%\n"
			    "Threshold2-3/up-down: %d%% %d%%\n"
			    "Threshold3-4/up-down: %d%% %d%%\n",
			    mali_dvfs[0].clock,
			    mali_dvfs[1].clock,
			    mali_dvfs[2].clock,
			    mali_dvfs[3].clock,
			    mali_dvfs[4].clock,
			    mali_dvfs_threshold[0].upthreshold*100/255,
			    mali_dvfs_threshold[1].downthreshold*100/255,
			    mali_dvfs_threshold[1].upthreshold*100/255,
			    mali_dvfs_threshold[2].downthreshold*100/255,
			    mali_dvfs_threshold[2].upthreshold*100/255,
			    mali_dvfs_threshold[3].downthreshold*100/255,
			    mali_dvfs_threshold[3].upthreshold*100/255,
			    mali_dvfs_threshold[4].downthreshold*100/255
		);
}

static ssize_t gpu_clock_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count) {
	unsigned int ret = -EINVAL;
	int i = 0;
	unsigned int g[8];

	if ((ret=sscanf(buf, "%d%% %d%% %d%% %d%% %d%% %d%% %d%% %d%%",
			 &g[0], &g[1], &g[2], &g[3], &g[4], &g[5], &g[6], &g[7])) == 8 ) i = 1;

	if(i) {
		if(g[1]<0 || g[0]>100 || g[3]<0 || g[2]>100 || g[5]<0 || g[4]>100 || g[7]<0 || g[6]>100)
			return -EINVAL;

		mali_dvfs_threshold[0].upthreshold = ((int)((255*g[0])/100));
		mali_dvfs_threshold[1].downthreshold = ((int)((255*g[1])/100));
		mali_dvfs_threshold[1].upthreshold = ((int)((255*g[2])/100));
		mali_dvfs_threshold[2].downthreshold = ((int)((255*g[3])/100));
		mali_dvfs_threshold[2].upthreshold = ((int)((255*g[4])/100));
		mali_dvfs_threshold[3].downthreshold = ((int)((255*g[5])/100));
		mali_dvfs_threshold[3].upthreshold = ((int)((255*g[6])/100));
		mali_dvfs_threshold[4].downthreshold = ((int)((255*g[7])/100));
	} else {
		if ((ret=sscanf(buf, "%d %d %d %d %d", &g[0], &g[1], &g[2], &g[3], &g[4])) != MALI_DVFS_STEPS)
			return -EINVAL;

		/* safety floor and ceiling - netarchy */
		for( i = 0; i < MALI_DVFS_STEPS; i++ ) {
			if (g[i] < GPU_MIN_CLOCK) {
				g[i] = GPU_MIN_CLOCK;
			}
			else if (g[i] > GPU_MAX_CLOCK) {
				g[i] = GPU_MAX_CLOCK;
			}
			mali_dvfs[i].clock = g[i];
		}

		// Yank555.lu : reupdate mali dvfs table
		mali_dvfs_table_update();

	}

	return count;
}

static ssize_t gpu_staycount_show(struct device *dev, struct device_attribute *attr, char *buf) {

	int i, len = 0;

	if(buf) {
		for(i = 0; i < MALI_DVFS_STEPS; i++)
			len += sprintf(buf + len, "Step%d: %d\n", i, mali_dvfs_staycount[i].staycount); // Yank555.lu : Use steps 0-4 as above to keep this consistent
	}
	return len;
}

static ssize_t gpu_staycount_store(struct device *dev, struct device_attribute *attr, const char *buf,
									size_t count) {
	unsigned int ret = -EINVAL;
	int i[5],j = 0;

    if ((ret=sscanf(buf, "%d %d %d %d %d", &i[0], &i[1], &i[2], &i[3], &i[4])) != MALI_DVFS_STEPS)
		return -EINVAL;
	else {
		for(j = 0; j < MALI_DVFS_STEPS; j++)
			mali_dvfs_staycount[j].staycount = i[j];
	}
	return count;
}

// Yank555.lu : Add available frequencies sysfs entry (similar to what we have for CPU)
static ssize_t available_frequencies_show(struct device *dev, struct device_attribute *attr, char *buf) {

	int i, len = 0;

	if(buf) {

		for (i = 0; gpu_freq_table[i] != GPU_FREQ_END_OF_TABLE; i++)
			len += sprintf(buf + len, "%d ", gpu_freq_table[i]);

		len += sprintf(buf + len, "\n"); // Don't forget to go to newline
		
	}

	return len;

}

static DEVICE_ATTR(gpu_control, S_IRUGO | S_IWUGO, gpu_clock_show, gpu_clock_store);
static DEVICE_ATTR(gpu_staycount, S_IRUGO | S_IWUGO, gpu_staycount_show, gpu_staycount_store);
static DEVICE_ATTR(available_frequencies, S_IRUGO, available_frequencies_show, NULL);

// Yank555.lu : Add a new sysfs interface, 1 tunable per step, easier to use from an app

// GPU frequency steps

#define expose_gpu_freq(step)										\
static ssize_t show_gpu_freq_##step									\
(struct device *dev, struct device_attribute *attr, char *buf) {					\
													\
	return sprintf(buf, "%d\n", mali_dvfs[step].clock);						\
													\
}													\
													\
static ssize_t store_gpu_freq_##step									\
(struct device *dev, struct device_attribute *attr,							\
			       const char *buf, size_t count) {						\
													\
	unsigned int input;										\
	int ret;											\
	int i=0;											\
													\
	ret = sscanf(buf, "%u", &input);								\
	if (ret != 1)											\
		return -EINVAL;										\
													\
	for (i = 0; (gpu_freq_table[i] != GPU_FREQ_END_OF_TABLE); i++)					\
		if (gpu_freq_table[i] == input) {							\
			mali_dvfs[step].clock = input;							\
			mali_dvfs_table_update();							\
			return count;									\
		}											\
													\
	return -EINVAL;											\
}													\
													\
static DEVICE_ATTR(gpu_freq_##step									\
		 , S_IRUGO | S_IWUGO									\
		 , show_gpu_freq_##step									\
		 , store_gpu_freq_##step								\
);

expose_gpu_freq(0);
expose_gpu_freq(1);
expose_gpu_freq(2);
expose_gpu_freq(3);
expose_gpu_freq(4);

// GPU staycount steps

#define expose_gpu_staycount(step)									\
static ssize_t show_gpu_staycount_##step								\
(struct device *dev, struct device_attribute *attr, char *buf) {					\
													\
	return sprintf(buf, "%d\n", mali_dvfs_staycount[step].staycount);				\
													\
}													\
													\
static ssize_t store_gpu_staycount_##step								\
(struct device *dev, struct device_attribute *attr,							\
			       const char *buf, size_t count) {						\
													\
	unsigned int input;										\
													\
	if (sscanf(buf, "%u", &input) != 1)								\
		return -EINVAL;										\
													\
	if (input < 0 || input > 255)									\
		return -EINVAL;										\
													\
	mali_dvfs_staycount[step].staycount = input;							\
	return count;											\
}													\
													\
static DEVICE_ATTR(gpu_staycount_##step									\
		 , S_IRUGO | S_IWUGO									\
		 , show_gpu_staycount_##step								\
		 , store_gpu_staycount_##step								\
);

expose_gpu_staycount(0);
expose_gpu_staycount(1);
expose_gpu_staycount(2);
expose_gpu_staycount(3);
expose_gpu_staycount(4);

// New GPU up/down thresholds interface

#define expose_gpu_upthreshold(step)									\
static ssize_t show_gpu_upthreshold_##step								\
(struct device *dev, struct device_attribute *attr, char *buf) {					\
													\
	return sprintf(buf, "%d\n", mali_dvfs_threshold[step].upthreshold*100/255);			\
													\
}													\
													\
static ssize_t store_gpu_upthreshold_##step								\
(struct device *dev, struct device_attribute *attr,							\
			       const char *buf, size_t count) {						\
													\
	int input;											\
													\
	if (sscanf(buf, "%d", &input) != 1)								\
		return -EINVAL;										\
													\
	if (input < 0 || input > 100)									\
		return -EINVAL;										\
													\
	mali_dvfs_threshold[step].upthreshold = ((int)((255*input)/100));				\
	return count;											\
}													\
													\
static DEVICE_ATTR(gpu_upthreshold_##step								\
		 , S_IRUGO | S_IWUGO									\
		 , show_gpu_upthreshold_##step								\
		 , store_gpu_upthreshold_##step								\
);

#define expose_gpu_downthreshold(step)									\
static ssize_t show_gpu_downthreshold_##step								\
(struct device *dev, struct device_attribute *attr, char *buf) {					\
													\
	return sprintf(buf, "%d\n", mali_dvfs_threshold[step].downthreshold*100/255);			\
													\
}													\
													\
static ssize_t store_gpu_downthreshold_##step								\
(struct device *dev, struct device_attribute *attr,							\
			       const char *buf, size_t count) {						\
													\
	int input;											\
													\
	if (sscanf(buf, "%d", &input) != 1)								\
		return -EINVAL;										\
													\
	if (input < 0 || input > 100)									\
		return -EINVAL;										\
													\
	mali_dvfs_threshold[step].downthreshold = ((int)((255*input)/100));				\
	return count;											\
}													\
													\
static DEVICE_ATTR(gpu_downthreshold_##step								\
		 , S_IRUGO | S_IWUGO									\
		 , show_gpu_downthreshold_##step							\
		 , store_gpu_downthreshold_##step							\
);

expose_gpu_upthreshold(0);
expose_gpu_upthreshold(1);
expose_gpu_upthreshold(2);
expose_gpu_upthreshold(3);
expose_gpu_downthreshold(1);
expose_gpu_downthreshold(2);
expose_gpu_downthreshold(3);
expose_gpu_downthreshold(4);

static struct attribute *gpu_clock_control_attributes[] = {
	&dev_attr_gpu_control.attr,
	&dev_attr_gpu_staycount.attr,
	&dev_attr_available_frequencies.attr,
	// Yank555.lu : new GPU frequency steps interface
	&dev_attr_gpu_freq_0.attr,
	&dev_attr_gpu_freq_1.attr,
	&dev_attr_gpu_freq_2.attr,
	&dev_attr_gpu_freq_3.attr,
	&dev_attr_gpu_freq_4.attr,
	// Yank555.lu : new GPU staycount steps interface
	&dev_attr_gpu_staycount_0.attr,
	&dev_attr_gpu_staycount_1.attr,
	&dev_attr_gpu_staycount_2.attr,
	&dev_attr_gpu_staycount_3.attr,
	&dev_attr_gpu_staycount_4.attr,
	// Yank555.lu : new GPU up/down thresholds interface
	&dev_attr_gpu_upthreshold_0.attr,
	&dev_attr_gpu_upthreshold_1.attr,
	&dev_attr_gpu_upthreshold_2.attr,
	&dev_attr_gpu_upthreshold_3.attr,
	&dev_attr_gpu_downthreshold_1.attr,
	&dev_attr_gpu_downthreshold_2.attr,
	&dev_attr_gpu_downthreshold_3.attr,
	&dev_attr_gpu_downthreshold_4.attr,
	NULL
};

static struct attribute_group gpu_clock_control_group = {
	.attrs = gpu_clock_control_attributes,
};

static struct miscdevice gpu_clock_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpu_clock_control",
};

void gpu_clock_control_start()
{
	printk("Initializing gpu clock control interface\n");

	misc_register(&gpu_clock_control_device);
	if (sysfs_create_group(&gpu_clock_control_device.this_device->kobj,
				&gpu_clock_control_group) < 0) {
		printk("%s sysfs_create_group failed\n", __FUNCTION__);
		pr_err("Unable to create group for %s\n", gpu_clock_control_device.name);
	}
}


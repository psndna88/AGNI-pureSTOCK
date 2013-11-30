/*
 * gpu_control.c -- a clock control interface for the sgs2/3
 *
 *  Copyright (C) 2011 Michael Wodkins
 *  twitter - @xdanetarchy
 *  XDA-developers - netarchy
 *  modified by gokhanmoral
 *
 *  Modified by Andrei F. for Galaxy S3 / Perseus kernel (June 2012)
 *
 *  Modified by DerTeufel to make it work with malir3p2 (November 2013)
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of the GNU General Public License as published by the
 *  Free Software Foundation;
 *
 */

#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include "gpu_control.h"

#define GPU_MAX_CLOCK 800
#define GPU_MIN_CLOCK 54

typedef struct mali_dvfs_tableTag{
    unsigned int clock;
    unsigned int freq;
    unsigned int vol;
    unsigned int downthreshold;
    unsigned int upthreshold;
}mali_dvfs_table;

extern mali_dvfs_table mali_dvfs[MALI_DVFS_STEPS];
unsigned int gv[MALI_DVFS_STEPS];
extern int step0_clk;
extern int step1_clk;
extern int step2_clk;
extern int step3_clk;
extern int step4_clk;

extern int step0_vol;
extern int step1_vol;
extern int step2_vol;
extern int step3_vol;
extern int step4_vol;

extern int step0_up;
extern int step1_up;
extern int step2_up;
extern int step3_up;

extern int step1_down;
extern int step2_down;
extern int step3_down;
extern int step4_down;

int gpu_voltage_delta[MALI_DVFS_STEPS] = {
#if defined(CONFIG_CPU_EXYNOS4212) || defined(CONFIG_CPU_EXYNOS4412)
	0,
	0,
	0,
	0,
	0
#else
	0,
	0,
	0
#endif
};

static ssize_t gpu_voltage_show(struct device *dev, struct device_attribute *attr, char *buf) {
	int i, j = 0;
   	for (i = 0; i < MALI_DVFS_STEPS; i++)
	{
	    j += sprintf(&buf[j], "Step%d: %d\n", i, mali_dvfs[i].vol);
	}
   return j;
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
		gpu_voltage_delta[i] = gv[i] - gpu_voltage_default[i];                
        }
	mali_dvfs_table_update();

        step0_vol = mali_dvfs[0].vol;
        step1_vol = mali_dvfs[1].vol;
        step2_vol = mali_dvfs[2].vol;
        step3_vol = mali_dvfs[3].vol;
        step4_vol = mali_dvfs[4].vol;

        return count;
}

static ssize_t gpu_clock_show(struct device *dev, struct device_attribute *attr, char *buf) {
	int i, j = 0;
   	for (i = 0; i < MALI_DVFS_STEPS; i++)
	{
	    j += sprintf(&buf[j], "Step%d: %d\n", i, mali_dvfs[i].clock);
	}

   	for (i = 0; i < MALI_DVFS_STEPS - 1; i++)
	{
	    j += sprintf(&buf[j], "Threshold%d-%d/up-down: %d%% %d%%\n", i, i+1, mali_dvfs[i].upthreshold, mali_dvfs[i+1].downthreshold);
	}
   return j;
}

unsigned int g[(MALI_DVFS_STEPS-1)*2];

static ssize_t gpu_clock_store(struct device *dev, struct device_attribute *attr,
                               const char *buf, size_t count) {
        unsigned int ret = -EINVAL;
        int i = 0;
        int j = 0;
        unsigned int g[8];

        if ((ret=sscanf(buf, "%d%% %d%% %d%% %d%% %d%% %d%% %d%% %d%%",
                         &g[0], &g[1], &g[2], &g[3], &g[4], &g[5], &g[6], &g[7])) == 8 ) i = 1;

        if(i) {
                if(g[1]<0 || g[0]>100 || g[3]<0 || g[2]>100 || g[5]<0 || g[4]>100 || g[7]<0 || g[6]>100)
                        return -EINVAL;
		
                mali_dvfs[0].upthreshold = (int)(g[0]);
                mali_dvfs[1].downthreshold = (int)(g[1]);

                mali_dvfs[1].upthreshold = (int)(g[2]);
                mali_dvfs[2].downthreshold = (int)(g[3]);
                mali_dvfs[2].upthreshold = (int)(g[4]);
                mali_dvfs[3].downthreshold = (int)(g[5]);
                mali_dvfs[3].upthreshold = (int)(g[6]);
                mali_dvfs[4].downthreshold = (int)(g[7]);
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
			/* only apply valid freq. - DerTeufel */
                        for (j = 0; (gpu_freq_table[j] != GPU_FREQ_END_OF_TABLE); j++) {
			    if (gpu_freq_table[j] == g[i]) {  
                                mali_dvfs[i].clock=g[i];
			    }
			}
                }
    	// Yank555.lu : reupdate mali dvfs table
    	mali_dvfs_table_update();

        }

        step0_clk = mali_dvfs[0].clock;
        step1_clk = mali_dvfs[1].clock;
        step2_clk = mali_dvfs[2].clock;
        step3_clk = mali_dvfs[3].clock;
        step4_clk = mali_dvfs[4].clock;

	step0_up = mali_dvfs[0].upthreshold;
	step1_up = mali_dvfs[1].upthreshold;
	step2_up = mali_dvfs[2].upthreshold;
	step3_up = mali_dvfs[3].upthreshold;

	step1_down = mali_dvfs[1].downthreshold;
	step2_down = mali_dvfs[2].downthreshold;
	step3_down = mali_dvfs[3].downthreshold;
	step4_down = mali_dvfs[4].downthreshold;

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

// Yank555.lu : add a global voltage delta to be applied to all automatic voltage resets
static ssize_t gpu_voltage_delta_show(struct device *dev, struct device_attribute *attr, char *buf) {

	int i, j = 0;
   	for (i = 0; i < MALI_DVFS_STEPS; i++)
	{
	    j += sprintf(&buf[j], "Step%d: %d\n", i, gpu_voltage_delta[i]);
	}
   return j;

}

static ssize_t gpu_voltage_delta_store(struct device *dev, struct device_attribute *attr, const char *buf,
                  size_t count) {
  int data;
  unsigned int ret;

  ret = sscanf(buf, "%d\n", &data);

  if (!ret) {
    return -EINVAL;
  }

  if (data == 1) { // DerTeufel: reset all voltage deltas
	int i;
   	for (i = 0; i < MALI_DVFS_STEPS; i++)
	{
    	    gpu_voltage_delta[i] = 0;
	}
    // Yank555.lu : update mali dvfs table
    mali_dvfs_table_update();
    return count;
  }

  return -EINVAL;

}

static DEVICE_ATTR(gpu_voltage_delta, S_IRUGO | S_IWUGO, gpu_voltage_delta_show, gpu_voltage_delta_store);

static DEVICE_ATTR(gpu_voltage_control, S_IRUGO | S_IWUGO, gpu_voltage_show, gpu_voltage_store);
static DEVICE_ATTR(gpu_clock_control, S_IRUGO | S_IWUGO, gpu_clock_show, gpu_clock_store);
static DEVICE_ATTR(available_frequencies, S_IRUGO, available_frequencies_show, NULL);

static struct attribute *gpu_control_attributes[] = {
        &dev_attr_gpu_voltage_control.attr,
        &dev_attr_gpu_clock_control.attr,
	&dev_attr_available_frequencies.attr,
	&dev_attr_gpu_voltage_delta.attr,
        NULL
};

static struct attribute_group gpu_control_group = {
        .attrs = gpu_control_attributes,
};

static struct miscdevice gpu_control_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "gpu_control",
};

void gpu_control_start()
{
        printk("Initializing gpu control interface\n");

        misc_register(&gpu_control_device);
        if (sysfs_create_group(&gpu_control_device.this_device->kobj,
                                &gpu_control_group) < 0) {
                printk("%s sysfs_create_group failed\n", __FUNCTION__);
                pr_err("Unable to create group for %s\n", gpu_control_device.name);
        }
}

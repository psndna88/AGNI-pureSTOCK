/*
 * abb_control.c - Adaptive Body Bias control for Exynos 4412
 *
 * @Author	: Andrei F. <https://github.com/AndreiLux>
 * @Date	: January 2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/sysfs_helpers.h>
#include <linux/io.h>

#include <mach/asv.h>
#include <mach/regs-pmu.h>
#include <plat/cpu.h>

#define SLICE_TYPE_NR		4

#define ARM_FREQ_SLICE_NR	4
#define G3D_FREQ_SLICE_NR	3
#define MIF_FREQ_SLICE_NR	2
#define INT_FREQ_SLICE_NR	2

#define FREQ_ALL		0
#define FREQ_MAX		9999999

#define volt_to_regval(volt)	((volt - 600) / 50)
#define regval_to_volt(regval)	((regval * 50) + 600)

extern struct samsung_asv *exynos_asv;

enum attr_type {
	FREQUENCY,
	VOLTAGE,
};

struct device_attr_value {
	const struct device_attribute	attr;
	int				value;
	const enum attr_type		type;
};

struct abb_slice {
	struct device_attr_value	freq;
	struct device_attr_value	volt;
};

static ssize_t show_abb_property(struct device *dev,
				    struct device_attribute *attr, char *buf);

static ssize_t store_abb_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count);

#define _abb_slice(_name, _freq, _volt)	\
{ 										\
	.freq = {								\
			.attr = {						\
				.attr = {					\
					  .name = #_name"_freq",		\
					  .mode = S_IRUGO | S_IWUSR | S_IWGRP,	\
					},					\
				.show = show_abb_property,			\
				.store = store_abb_property,			\
				},						\
			.value = _freq,						\
			.type = FREQUENCY,					\
		},								\
	.volt = {								\
			.attr = {						\
				.attr = {					\
					  .name = #_name"_volt",		\
					  .mode = S_IRUGO | S_IWUSR | S_IWGRP,	\
					},					\
				.show = show_abb_property,			\
				.store = store_abb_property,			\
		 		},						\
			.value = _volt,						\
			.type = VOLTAGE,					\
		},								\
}

/* Frequecy thresholds are inclusive : ..<..slice]..<..slice]..<..slice]..<.. */

static struct abb_slice int_abb_slices[INT_FREQ_SLICE_NR] = {
	_abb_slice(int_slice_1,	110110	, ABB_MODE_100V),
	_abb_slice(int_slice_2,	FREQ_MAX, ABB_MODE_100V),
};

static struct abb_slice mif_abb_slices[MIF_FREQ_SLICE_NR] = {
	_abb_slice(mif_slice_1,	110110	, ABB_MODE_100V),
	_abb_slice(mif_slice_2,	FREQ_MAX, ABB_MODE_100V),
};

static struct abb_slice g3d_abb_slices[G3D_FREQ_SLICE_NR] = {
	_abb_slice(g3d_slice_1,	160000	, ABB_MODE_100V),
	_abb_slice(g3d_slice_2,	533000	, ABB_MODE_100V),
	_abb_slice(g3d_slice_3,	FREQ_MAX, ABB_MODE_075V),
};

static struct abb_slice arm_abb_slices[ARM_FREQ_SLICE_NR] = {
	_abb_slice(arm_slice_1,	200000	, ABB_MODE_075V),
	_abb_slice(arm_slice_2,	800000	, ABB_MODE_075V),
	_abb_slice(arm_slice_3,	1600000	, ABB_MODE_075V),
	_abb_slice(arm_slice_4,	FREQ_MAX, ABB_MODE_075V),
};

/* Index lookup tables
 * Keep declarations in exynos4x12_abb_member enum order */

static int abb_targets[]		= { ABB_INT, ABB_MIF, ABB_G3D, ABB_ARM };
static char* abb_label[]		= { "int", "mif", "g3d", "arm" };

static struct abb_slice* abb_slices[]   = { &int_abb_slices[0], &mif_abb_slices[0],
					    &g3d_abb_slices[0], &arm_abb_slices[0] };

static int abb_slice_size[]		= { INT_FREQ_SLICE_NR, MIF_FREQ_SLICE_NR,
					    G3D_FREQ_SLICE_NR, ARM_FREQ_SLICE_NR };

static void set_slice_voltage(enum exynos4x12_abb_member target,
				int target_freq, int voltage)
{
	struct abb_slice* slice;
	struct abb_slice* candidate = NULL;
	int slice_size, i, tmp_volt;

	slice = abb_slices[target];
	slice_size = abb_slice_size[target];

	tmp_volt = volt_to_regval(voltage);
	sanitize_min_max(tmp_volt, ABB_MODE_060V, ABB_MODE_160V);

	if (target_freq != FREQ_ALL) {
		for (i = 0; i < slice_size; i++) {
			if(target_freq > slice->freq.value) {
				slice++;
				continue;	
			}

			candidate = slice;
		}

		if (candidate == NULL)
			return;

		candidate->volt.value = tmp_volt;
	} else {
		for (i = 0; i < slice_size; i++)
			slice++->volt.value = tmp_volt;
	}
}

static int get_slice_voltage(enum exynos4x12_abb_member target,
				int target_freq)
{
	struct abb_slice* slice;
	int slice_size, ret_volt;

	slice_size = abb_slice_size[target];
	slice = abb_slices[target] + slice_size - 1;

	do {
		ret_volt = slice->volt.value;
	} while ( (slice > abb_slices[target]) &&
		  (target_freq <= (--slice)->freq.value) );
	
	return ret_volt;
}

void abb_target(enum exynos4x12_abb_member target, int new_freq)
{
	int voltage = get_slice_voltage(target, new_freq);
/*
	printk("%s on %s by freq %d voltage: %d\n",
		__func__, abb_label[target], new_freq, voltage);
*/
	exynos4x12_set_abb_member(target, voltage);
}


static ssize_t show_abb_property(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct device_attr_value *container = (struct device_attr_value*)(attr);

	switch (container->type) {
		case FREQUENCY:	
			return sprintf(buf, "%d", container->value);
		case VOLTAGE:	
			return sprintf(buf, "%d", regval_to_volt(container->value));
		default:
			return 0;
	}
}

static ssize_t store_abb_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct device_attr_value *container = (struct device_attr_value*)(attr);
	int val;

	if(sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	switch (container->type) {
		case FREQUENCY:	
			sanitize_min_max(val, 0, FREQ_MAX);
			break;
		case VOLTAGE:
			val = volt_to_regval(val);
			sanitize_min_max(val, ABB_MODE_060V, ABB_MODE_160V);
			break;
		default:
			return -EINVAL;
	}

	container->value = val;

	return count;
}

static inline int get_live_voltage(enum exynos4x12_abb_member target)
{
	return regval_to_volt(exynos4x12_get_abb_member(target));
}

static ssize_t show_abb_info(struct sysdev_class * cls, 
			     struct sysdev_class_attribute *attr, char *buf)
{
	int len = 0;

	len += sprintf(len + buf, "ARM: %d\n", get_live_voltage(ABB_ARM));
	len += sprintf(len + buf, "G3D: %d\n", get_live_voltage(ABB_G3D));
	len += sprintf(len + buf, "MIF: %d\n", get_live_voltage(ABB_MIF));
	len += sprintf(len + buf, "INT: %d\n", get_live_voltage(ABB_INT));

	len += sprintf(len + buf, "\n");

	len += sprintf(len + buf, "HPM: %d\n", exynos_asv->hpm_result);
	len += sprintf(len + buf, "IDS: %d\n", exynos_asv->ids_result);
	len += sprintf(len + buf, "ASV: %d\n", exynos_result_of_asv);

	return len;
}

static SYSDEV_CLASS_ATTR(abb_info, S_IRUGO, show_abb_info, NULL);

struct sysdev_class abb_sysclass = {
	.name	= "abb",
};

void abb_control_init()
{
	static bool initialized = false;
	int i, ret = 0;

	printk("ABB-Control: %s\n", __func__);

	if (initialized)
		return;

	if (samsung_rev() >= EXYNOS4412_REV_2_0) {
		switch (exynos_result_of_asv) {
			case 0:
			case 1:
				/* Default variable values */
				break;
			case 2:
				set_slice_voltage(ABB_ARM, FREQ_ALL, 1000);
				set_slice_voltage(ABB_G3D, FREQ_ALL, 1000);
				set_slice_voltage(ABB_INT, FREQ_ALL, 1000);
				set_slice_voltage(ABB_MIF, FREQ_ALL, 1000);
				break;
			case 3:
				set_slice_voltage(ABB_ARM, FREQ_ALL, 1300);
				set_slice_voltage(ABB_ARM, 200000, 1000);
				set_slice_voltage(ABB_G3D, FREQ_ALL, 1000);
				set_slice_voltage(ABB_INT, FREQ_ALL, 1000);
				set_slice_voltage(ABB_MIF, FREQ_ALL, 1400);
				break;
			case 4:
			case 5:
			case 6:
			case 7:
				set_slice_voltage(ABB_ARM, FREQ_ALL, 1300);
				set_slice_voltage(ABB_G3D, FREQ_ALL, 1000);
				set_slice_voltage(ABB_INT, FREQ_ALL, 1400);
				set_slice_voltage(ABB_MIF, FREQ_ALL, 1400);
				break;
			case 8:
			case 9:
			case 10:
			case 11:
			case 12:
				set_slice_voltage(ABB_ARM, FREQ_ALL, 1300);
				set_slice_voltage(ABB_G3D, FREQ_ALL, 1300);
				set_slice_voltage(ABB_INT, FREQ_ALL, 1300);
				set_slice_voltage(ABB_MIF, FREQ_ALL, 1400);
			default:
				set_slice_voltage(ABB_ARM, FREQ_ALL, 1300);
				set_slice_voltage(ABB_G3D, FREQ_ALL, 1300);
				set_slice_voltage(ABB_INT, FREQ_ALL, 1300);
				set_slice_voltage(ABB_MIF, FREQ_ALL, 1300);
		}
	} else {
		switch (exynos_result_of_asv) {
			case 0:
			case 1:
			case 2:
			case 3:
				set_slice_voltage(ABB_ARM, FREQ_ALL, 1000);
				break;
			case 4:
			case 5:
			case 6:
			case 7:
				set_slice_voltage(ABB_MIF, FREQ_ALL, 1300);
				set_slice_voltage(ABB_MIF, 110110, 1000);
				set_slice_voltage(ABB_INT, FREQ_ALL, 1300);
				set_slice_voltage(ABB_INT, 110110, 1000);
				set_slice_voltage(ABB_ARM, FREQ_ALL, 1300);
				set_slice_voltage(ABB_ARM, 200000, 1000);
				break;
			default:
				set_slice_voltage(ABB_ARM, FREQ_ALL, 1300);
		}

		set_slice_voltage(ABB_G3D, FREQ_ALL, 1300);
		set_slice_voltage(ABB_G3D, 160000, 1000);
	}

	sysdev_class_register(&abb_sysclass);

	ret += sysfs_create_file(&abb_sysclass.kset.kobj, &attr_abb_info.attr);

	for (i = 0; i < SLICE_TYPE_NR; i++) {
		int target = abb_targets[i];
		struct abb_slice* slice = abb_slices[target];
		struct kobject *sub = kobject_create_and_add(abb_label[target],
							&abb_sysclass.kset.kobj);
		do {
			ret += sysfs_create_file(sub, &slice->volt.attr.attr);
			ret += sysfs_create_file(sub, &slice->freq.attr.attr);

			printk("ABB-Control; %s:\t%d KHz %d mV\n",
				abb_label[target], slice->freq.value,
				regval_to_volt(slice->volt.value));

		} while ((++slice - abb_slices[target]) < abb_slice_size[target]);
	}

	initialized = true;
}

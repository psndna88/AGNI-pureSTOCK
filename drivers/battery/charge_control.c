/*
 * charge_control.c - Battery charge control for MAX77693 on MIDAS
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

#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/sysfs_helpers.h>
#include <linux/mfd/max77693-private.h>
#include <linux/battery/samsung_battery.h>

#define MIN_SAFETY_CURR				100
#define MAX_SAFETY_CURR				2500

#define MIN_SOFT_VOLT				3600000
#define MAX_SOFT_VOLT				4500000

#define MAX77693_CHG_CV_PRM_MASK		0x1F
#define MAX77693_CHG_CV_PRM_4_20V		0x16
#define MAX77693_CHG_CV_PRM_4_35V		0x1D
#define MAX77693_CHG_CV_PRM_4_40V		0x1F

static ssize_t show_charge_property(struct device *dev,
				    struct device_attribute *attr, char *buf);

static ssize_t store_charge_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count);

#define CHARGE_ATTR(_name)				\
{							\
	.attr = {					\
		  .name = #_name,			\
		  .mode = S_IRUGO | S_IWUSR | S_IWGRP,	\
		},					\
	.show = show_charge_property,			\
	.store = store_charge_property,			\
}

static struct device_attribute charge_control_attrs[] = {
	CHARGE_ATTR(dcp_ac_input_curr),
	CHARGE_ATTR(dcp_ac_chrg_curr),
	CHARGE_ATTR(sdp_chrg_curr),
	CHARGE_ATTR(sdp_input_curr),
	CHARGE_ATTR(cdp_chrg_curr),
	CHARGE_ATTR(cdp_input_curr),
	CHARGE_ATTR(batt_chrg_hard_volt),
	CHARGE_ATTR(batt_chrg_soft_volt),
	CHARGE_ATTR(ignore_unstable_power),
	CHARGE_ATTR(ignore_stable_margin),
};

enum {
	DCP_AC_INPUT_CURR = 0,
	DCP_AC_CHRG_CURR,
	SDP_CHRG_CURR,
	SDP_INPUT_CURR,
	CDP_CHRG_CURR,
	CDP_INPUT_CURR,

	BATT_CHRG_HARD_VOLT,
	BATT_CHRG_SOFT_VOLT,

	IGNORE_UNSTABLE,
	IGNORE_MARGIN
};

struct max77693_dev *max77693;
static bool chrg_ctrl_flags[CHRG_CTRL_FLAGS] = { 0 };
static int batt_chrg_volt = 2;

static void update_battery_charge_voltage(int volt_t)
{
	u8 reg_data;
	int new_volt;

	if(batt_chrg_volt == volt_t)
		return;

	max77693_read_reg(max77693->i2c, MAX77693_CHG_REG_CHG_CNFG_04, &reg_data);
	reg_data &= (~MAX77693_CHG_CV_PRM_MASK);

	switch (volt_t) {
		case 1:  new_volt = MAX77693_CHG_CV_PRM_4_20V; break;
		case 2:  new_volt = MAX77693_CHG_CV_PRM_4_35V; break;
		case 3:  new_volt = MAX77693_CHG_CV_PRM_4_40V; break;
		default: return;
	}

	batt_chrg_volt = volt_t;

	reg_data |= new_volt;
	max77693_write_reg(max77693->i2c, MAX77693_CHG_REG_CHG_CNFG_04, reg_data);
}

int charge_control_is_flag(int flag)
{
	return chrg_ctrl_flags[flag];
}

static ssize_t show_charge_property(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct battery_info *info = dev_get_drvdata(dev);
	const ptrdiff_t offset = attr - charge_control_attrs;
	
	switch (offset) {
		case DCP_AC_INPUT_CURR:
			return sprintf(buf, "%d", info->pdata->in_curr_limit);
		case DCP_AC_CHRG_CURR:
			return sprintf(buf, "%d", info->pdata->chg_curr_ta);
		case SDP_CHRG_CURR:
			return sprintf(buf, "%d", info->pdata->chg_curr_usb);
		case SDP_INPUT_CURR:
			return sprintf(buf, "%d", info->pdata->in_curr_usb);
		case CDP_CHRG_CURR:
			return sprintf(buf, "%d", info->pdata->chg_curr_cdp);
		case CDP_INPUT_CURR:
			return sprintf(buf, "%d", info->pdata->in_curr_cdp);
		case BATT_CHRG_HARD_VOLT:
			return sprintf(buf, "%d", batt_chrg_volt);
		case BATT_CHRG_SOFT_VOLT:
			return sprintf(buf, "%d", info->pdata->recharge_voltage);
		case IGNORE_UNSTABLE:
			return sprintf(buf, "%d",
				chrg_ctrl_flags[CHRG_CTRL_IGNORE_UNSTABLE]);
		case IGNORE_MARGIN:
			return sprintf(buf, "%d",
				chrg_ctrl_flags[CHRG_CTRL_IGNORE_MARGIN]);
	}

	return -EINVAL;
}

static ssize_t store_charge_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct battery_info *info = dev_get_drvdata(dev);
	const ptrdiff_t offset = attr - charge_control_attrs;
	int ret, val;

	val = 0;

	if((ret = sscanf(buf, "%d", &val)) != 1)
		return -EINVAL;	

	switch (offset) {
		case DCP_AC_INPUT_CURR:
			sanitize_min_max(val, MIN_SAFETY_CURR, MAX_SAFETY_CURR);
			info->pdata->in_curr_limit = val;
			break;
		case DCP_AC_CHRG_CURR:
			sanitize_min_max(val, MIN_SAFETY_CURR, MAX_SAFETY_CURR);
			info->pdata->chg_curr_ta = val;
			break;
		case SDP_CHRG_CURR:
			sanitize_min_max(val, MIN_SAFETY_CURR, MAX_SAFETY_CURR);
			info->pdata->chg_curr_usb = val;
			break;
		case SDP_INPUT_CURR:
			sanitize_min_max(val, MIN_SAFETY_CURR, MAX_SAFETY_CURR);
			info->pdata->in_curr_usb = val;
			break;
		case CDP_CHRG_CURR:
			sanitize_min_max(val, MIN_SAFETY_CURR, MAX_SAFETY_CURR);
			info->pdata->chg_curr_cdp = val;
			break;
		case CDP_INPUT_CURR:
			sanitize_min_max(val, MIN_SAFETY_CURR, MAX_SAFETY_CURR);
			info->pdata->in_curr_cdp = val;
			break;
		case BATT_CHRG_HARD_VOLT:
			update_battery_charge_voltage(val);
			break;
		case BATT_CHRG_SOFT_VOLT:
			sanitize_min_max(val, MIN_SOFT_VOLT, MAX_SOFT_VOLT);
			info->pdata->recharge_voltage = val;
			break;
		case IGNORE_UNSTABLE:
			chrg_ctrl_flags[CHRG_CTRL_IGNORE_UNSTABLE] = !!val;
			break;
		case IGNORE_MARGIN:
			chrg_ctrl_flags[CHRG_CTRL_IGNORE_MARGIN] = !!val;
			break;
			
		default:
			goto invalid_prop;
	}

	cancel_work_sync(&info->monitor_work);
	schedule_work(&info->monitor_work);

	return count;
invalid_prop:

	return -EINVAL;
}

void charger_control_set_charger(struct max77693_dev *dev) { max77693 = dev; }

void charger_control_init(struct battery_info *info)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(charge_control_attrs); i++) {
		if (device_create_file(info->dev, &charge_control_attrs[i]))
			goto create_failed;
	}

	pr_info("%s: charge control interface creation complete\n", __func__);

	return;

create_failed:
	pr_info("%s: charge control interface creation failed\n", __func__);
	while (i--)
		device_remove_file(info->dev, &charge_control_attrs[i]);
}

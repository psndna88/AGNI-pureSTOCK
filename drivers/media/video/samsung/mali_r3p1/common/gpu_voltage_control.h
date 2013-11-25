/*
 * gpu_voltage_control.h -- gpu voltage control interface for the sgs2/3
 *
 *  Copyright (C) 2011 Michael Wodkins
 *  twitter - @xdanetarchy
 *  XDA-developers - netarchy
 *
 *  Modified for SiyahKernel
 *  Modified for Perseus kernel
 *
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

#define MIN_VOLTAGE_GPU 600000
#define MAX_VOLTAGE_GPU 1400000
#define MALI_DVFS_STEPS 5

#include "mali_osk.h"

void gpu_voltage_control_start(void);

mali_bool mali_dvfs_table_update(void);


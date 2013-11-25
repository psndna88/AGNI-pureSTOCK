/*
 * gpu_clock_control.h -- a clock control interface for the sgs2
 *
 *  Copyright (C) 2011 Michael Wodkins
 *  twitter - @xdanetarchy
 *  XDA-developers - netarchy
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

#define GPU_MAX_CLOCK 800
#define GPU_MIN_CLOCK 10
#define MALI_DVFS_STEPS 5

#include "mali_osk.h"
#include "mali_platform.h"

void gpu_clock_control_start(void);

mali_bool mali_dvfs_table_update(void);


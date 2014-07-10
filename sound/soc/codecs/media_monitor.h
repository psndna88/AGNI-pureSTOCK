/*
 * media_monitor.c - Media Handling Status reporting for Exynos 4412 / MIDAS
 *
 * @Author        : Andrei F. <https://github.com/AndreiLux>
 * @Date        : April 2013
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

enum mhs_type {
        MHS_ENCODING = 0,
        MHS_DECODING,
        MHS_CAMERA_STREAM,
        MHS_TYPE_INVALID,
        MHS_TYPE_MAX,
};

extern void mhs_set_status(enum mhs_type type, bool status);
extern int mhs_get_status(enum mhs_type type);

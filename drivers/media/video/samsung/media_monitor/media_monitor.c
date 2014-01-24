/*
 * media_monitor.c - Media Handling Status reporting for Exynos 4412 / MIDAS
 *
 * @Author	: Andrei F. <https://github.com/AndreiLux>
 * @Date	: April 2013
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
#include <mach/media_monitor.h>

struct mhs_context {
	bool	encoding;
	bool	decoding;
	bool	camera_stream;
} mhs_ctx = {
	.encoding = false,
	.decoding = false,
	.camera_stream = false,
};

//TODO Replace this with a proper notifier chain in the future
#ifdef CONFIG_SND_WOLFSON_SOUND_CONTROL
extern void set_mic_level(void);
#endif
extern void do_mdnie_refresh(struct work_struct *work);

void mhs_set_status(enum mhs_type type, bool status)
{
	printk("MHS: type:%d status:%d\n", type, status);
	switch (type) {
		case MHS_ENCODING:	mhs_ctx.encoding = status; break;
		case MHS_DECODING:	mhs_ctx.decoding = status;
//					do_mdnie_refresh(NULL); //TODO see above
					break;
		case MHS_CAMERA_STREAM:	mhs_ctx.camera_stream = status;
#ifdef CONFIG_SND_WOLFSON_SOUND_CONTROL
					set_mic_level(); //TODO see above
#endif
					break;
		default:		return;
	}
}

int mhs_get_status(enum mhs_type type)
{
	switch (type) {
		case MHS_ENCODING:	return mhs_ctx.encoding;
		case MHS_DECODING:	return mhs_ctx.decoding;
		case MHS_CAMERA_STREAM:	return mhs_ctx.camera_stream;
		default: 		return false;
	}
}


/*
 * Author: andip71, 21.12.2012
 *
 * credits: Hardcore for the mdnie settings
 *          Gokhanmoral for some implementation ideas
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/miscdevice.h>
#include <linux/kallsyms.h>

#include "s3cfb.h"
#include "s3cfb_mdnie.h"

#include "mdnie.h"
#include "mdnie_table_c1m0_pr0.h"
#include "mdnie_table_c1m0_pr1.h"


/*****************************************/
// Static variables
/*****************************************/

static int mdnie_preset;


/*****************************************/
// Internal helper functions
/*****************************************/

static void update_mdnie_array (unsigned char *dest_name, const unsigned short source[])
{
	unsigned int i;
	unsigned short *a;

	// get pointer to desired destination table structure
	a = (unsigned short *)(kallsyms_lookup_name(dest_name));

	// in a loop, copy element by element over from the preset table
	// (loop ends after 250 for security reasons if END_SEQ is somehow missing)
	for (i = 0; i <= 250; i+=1)
	{
		if(source[i] == END_SEQ)
			break;
		a[i]=source[i];
	}
}


static void write_mdnie_ui (unsigned char *dest_name,
			const unsigned short source_dynamic[], const unsigned short source_movie[],
			const unsigned short source_standard[],	const unsigned short source_natural[])
{
	struct mdnie_info *m;

	// get pointer to global mdnie control structure
	m = *((void **)kallsyms_lookup_name("g_mdnie"));

	// send the sequence to the mdnie driver to reflect update instantly,
	// based on currently configured screen mode
	switch(m->mode)
	{
		case DYNAMIC:
		{
			mdnie_send_sequence(m, source_dynamic);
			break;
		}

		case MOVIE:
		{
			mdnie_send_sequence(m, source_movie);
			break;
		}

		case STANDARD:
		{
			mdnie_send_sequence(m, source_standard);
			break;
		}

		case NATURAL:
		{
			mdnie_send_sequence(m, source_natural);
			break;
		}

		case MODE_MAX:
		{
			break;
		}
	}

	// print debug info
	printk("Boeffla-kernel: mdnie control device updated for mode: %d\n", m->mode);
}



/*****************************************/
// sysfs interface functions
/*****************************************/

static ssize_t mdnie_preset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	// return value of current preset
	return sprintf(buf, "MDNIE preset: %d", mdnie_preset);
}


static ssize_t mdnie_preset_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int ret = -EINVAL;
	unsigned int val = 0;

	// read value from input buffer
	ret = sscanf(buf, "%d", &val);

	// check received preset value for validity
	if ((val >= 0) && (val <= 1))
	{
		// store preset in global variable
		mdnie_preset = val;

		if (mdnie_preset == 0) // original Samsung mdnie settings
		{
			// update all mdnie control tables with the given preset
			update_mdnie_array("tune_dynamic_gallery", tune_dynamic_gallery_pr0);
			update_mdnie_array("tune_dynamic_ui", tune_dynamic_ui_pr0);
			update_mdnie_array("tune_dynamic_video", tune_dynamic_video_pr0);
			update_mdnie_array("tune_dynamic_vt", tune_dynamic_vt_pr0);
			update_mdnie_array("tune_movie_gallery", tune_movie_gallery_pr0);
			update_mdnie_array("tune_movie_ui", tune_movie_ui_pr0);
			update_mdnie_array("tune_movie_video", tune_movie_video_pr0);
			update_mdnie_array("tune_movie_vt", tune_movie_vt_pr0);
			update_mdnie_array("tune_standard_gallery", tune_standard_gallery_pr0);
			update_mdnie_array("tune_standard_ui", tune_standard_ui_pr0);
			update_mdnie_array("tune_standard_video", tune_standard_video_pr0);
			update_mdnie_array("tune_standard_vt", tune_standard_vt_pr0);
			update_mdnie_array("tune_natural_gallery", tune_natural_gallery_pr0);
			update_mdnie_array("tune_natural_ui", tune_natural_ui_pr0);
			update_mdnie_array("tune_natural_video", tune_natural_video_pr0);
			update_mdnie_array("tune_natural_vt", tune_natural_vt_pr0);
			update_mdnie_array("tune_camera", tune_camera_pr0);
			update_mdnie_array("tune_camera_outdoor", tune_camera_outdoor_pr0);
			update_mdnie_array("tune_cold", tune_cold_pr0);
			update_mdnie_array("tune_cold_outdoor", tune_cold_outdoor_pr0);
			update_mdnie_array("tune_normal_outdoor", tune_normal_outdoor_pr0);
			update_mdnie_array("tune_warm", tune_warm_pr0);
			update_mdnie_array("tune_warm_outdoor", tune_warm_outdoor_pr0);

			// finally write ui preset, based on current scenario to mdnie controller
			write_mdnie_ui("tune_standard_ui", tune_dynamic_ui_pr0, tune_movie_ui_pr0,
							tune_standard_ui_pr0, tune_natural_ui_pr0);

			// print debug info
			printk("Boeffla-kernel: mdnie preset set to original\n");

		}

		if (mdnie_preset == 1) // Hardcore speedmod mdnie settings
		{
			// update all mdnie control tables with the given preset
			update_mdnie_array("tune_dynamic_gallery", tune_dynamic_gallery_pr1);
			update_mdnie_array("tune_dynamic_ui", tune_dynamic_ui_pr1);
			update_mdnie_array("tune_dynamic_video", tune_dynamic_video_pr1);
			update_mdnie_array("tune_dynamic_vt", tune_dynamic_vt_pr1);
			update_mdnie_array("tune_movie_gallery", tune_movie_gallery_pr1);
			update_mdnie_array("tune_movie_ui", tune_movie_ui_pr1);
			update_mdnie_array("tune_movie_video", tune_movie_video_pr1);
			update_mdnie_array("tune_movie_vt", tune_movie_vt_pr1);
			update_mdnie_array("tune_standard_gallery", tune_standard_gallery_pr1);
			update_mdnie_array("tune_standard_ui", tune_standard_ui_pr1);
			update_mdnie_array("tune_standard_video", tune_standard_video_pr1);
			update_mdnie_array("tune_standard_vt", tune_standard_vt_pr1);
			update_mdnie_array("tune_natural_gallery", tune_natural_gallery_pr1);
			update_mdnie_array("tune_natural_ui", tune_natural_ui_pr1);
			update_mdnie_array("tune_natural_video", tune_natural_video_pr1);
			update_mdnie_array("tune_natural_vt", tune_natural_vt_pr1);
			update_mdnie_array("tune_camera", tune_camera_pr1);
			update_mdnie_array("tune_camera_outdoor", tune_camera_outdoor_pr1);
			update_mdnie_array("tune_cold", tune_cold_pr1);
			update_mdnie_array("tune_cold_outdoor", tune_cold_outdoor_pr1);
			update_mdnie_array("tune_normal_outdoor", tune_normal_outdoor_pr1);
			update_mdnie_array("tune_warm", tune_warm_pr1);
			update_mdnie_array("tune_warm_outdoor", tune_warm_outdoor_pr1);

			// finally write ui preset, based on current scenario to mdnie controller
			write_mdnie_ui("tune_standard_ui", tune_dynamic_ui_pr1, tune_movie_ui_pr1,
							tune_standard_ui_pr1, tune_natural_ui_pr1);

			// print debug info
			printk("Boeffla-kernel: mdnie preset set to Hardcore speedmod\n");
		}

	}

	return count;
}



/*****************************************/
// Initialize mdnie preset sysfs folder
/*****************************************/

// define objects
static DEVICE_ATTR(mdnie_preset, S_IRUGO | S_IWUGO, mdnie_preset_show, mdnie_preset_store);

// define attributes
static struct attribute *mdnie_preset_attributes[] = {
	&dev_attr_mdnie_preset.attr,
	NULL
};

// define attribute group
static struct attribute_group mdnie_preset_control_group = {
	.attrs = mdnie_preset_attributes,
};

// define control device
static struct miscdevice mdnie_preset_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mdnie_preset",
};


/*****************************************/
// Driver init and exit functions
/*****************************************/

static int mdnie_preset_init(void)
{
	// register mdnie preset control device
	misc_register(&mdnie_preset_control_device);
	if (sysfs_create_group(&mdnie_preset_control_device.this_device->kobj,
				&mdnie_preset_control_group) < 0) {
		printk("Boeffla-kernel: failed to create sys fs object.\n");
		return 0;
	}

	// Print debug info
	printk("Boeffla-kernel: mdnie control device started\n");

	return 0;
}


static void mdnie_preset_exit(void)
{
	// remove mdnie preset control device
	sysfs_remove_group(&mdnie_preset_control_device.this_device->kobj,
                           &mdnie_preset_control_group);

	// Print debug info
	printk("Boeffla-kernel: mdnie control device stopped\n");
}


/* define driver entry points */

module_init(mdnie_preset_init);
module_exit(mdnie_preset_exit);

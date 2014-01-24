/*
 * Author: Andrei F. <https://github.com/AndreiLux>
 *          Complete re-work and refactoring of andip71's implementation
 *
 * credits: andip71 for Boeffla sound implementation
 *          Supercurio for idea and code from first implementation Voodoo Sound,
 *          Yank555 for great support on problem analysis,
 *          Gokhanmoral for further modifications to the original code
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


/*****************************************/
// External function declarations
/*****************************************/

void sound_control_hook_wm8994_pcm_probe(struct snd_soc_codec *codec_pointer);
unsigned int sound_control_hook_wm8994_write(unsigned int reg, unsigned int value);


/*****************************************/
// Definitions
/*****************************************/

// Sound control general
#define SOUND_CONTROL_DEFAULT 	0
#define SOUND_CONTROL_VERSION 	"Perseus 35"

enum {
	EQ_SP = 0,
	EQ_HP,
	EQ_TYPE_MAX,
};

enum {
	OUTPUT_HP = 0,
	OUTPUT_SPEAKER,
	OUTPUT_RECEIVER,
	OUTPUT_OTHER,
	OUTPUT_MAX
};

// Debug mode

enum {
	DEBUG_OFF = 0,
	DEBUG_NORMAL,
	DEBUG_VERBOSE,
	DEBUG_MAX,
};

#define DEBUG_DEFAULT 		DEBUG_OFF

// Debug register
#define DEBUG_REGISTER_KEY 	66

// EQ mode

enum {
	EQ_ENABLED = 1,
	EQ_SATPREVENT = 2,
};

#define EQ_DEFAULT 		EQ_ENABLED & EQ_SATPREVENT

// EQ gain
#define EQ_GAIN_DEFAULT 	0

#define EQ_GAIN_OFFSET 		12
#define EQ_GAIN_MIN 		-12
#define EQ_GAIN_MAX  		12

// EQ saturation prevention
enum {
	AIF1_DRC1_DEFAULT = 0,
	AIF1_DRC1_PREVENT,
	AIF1_DRC1_STUNING,
};

// Speaker tuning
#define SPEAKER_BOOST_DEFAULT	4
#define SPEAKER_BOOST_TUNED	6

// FLL tuning loop gains
#define FLL_LOOP_GAIN_DEFAULT	0
#define FLL_LOOP_GAIN_TUNED	5

// Stereo 3D
#define STEREO_3D_GAIN_DEFAULT	0
#define STEREO_3D_GAIN_OFF	0
#define STEREO_3D_GAIN_MAX	31

// headphone levels
#define HEADPHONE_DEFAULT 	50

#define HEADPHONE_MAX 		63
#define HEADPHONE_MIN 		0

// speaker levels
#define SPEAKER_DEFAULT 	57

#define SPEAKER_MAX 		63
#define SPEAKER_MIN 		57

// Microphone control
#define MICLEVEL_GENERAL	28
#define MICLEVEL_CAMERA		26
#define MICLEVEL_CALL		25

#define MICLEVEL_MIN		0
#define MICLEVEL_MAX		31

// Register dump
#define REGDUMP_BANKS		4
#define REGDUMP_REGISTERS	300

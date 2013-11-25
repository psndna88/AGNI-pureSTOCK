/*
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_platform_dvfs.c
 * Platform specific Mali driver dvfs functions
 */

#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "mali_platform.h"

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#include <asm/io.h>

#include "mali_device_pause_resume.h"
#include <linux/workqueue.h>

#define MAX_MALI_DVFS_STEPS 5
#define MALI_DVFS_WATING 10 // msec

#ifdef CONFIG_CPU_FREQ
#include <mach/asv.h>
#define EXYNOS4_ASV_ENABLED
#endif

#include <plat/cpu.h>

static int bMaliDvfsRun=0;

static _mali_osk_atomic_t bottomlock_status;
int bottom_lock_step = 0;

typedef struct mali_dvfs_tableTag{
	unsigned int clock;
	unsigned int freq;
	unsigned int vol;
}mali_dvfs_table;

typedef struct mali_dvfs_statusTag{
	unsigned int currentStep;
	mali_dvfs_table * pCurrentDvfs;

}mali_dvfs_currentstatus;

typedef struct mali_dvfs_thresholdTag{
	unsigned int downthreshold;
	unsigned int upthreshold;
}mali_dvfs_threshold_table;

typedef struct mali_dvfs_staycount{
	unsigned int staycount;
}mali_dvfs_staycount_table;

typedef struct mali_dvfs_stepTag{
	int clk;
	int vol;
}mali_dvfs_step;


mali_dvfs_staycount_table mali_dvfs_staycount[MALI_DVFS_STEPS]={
	/*step 0*/{0},
	/*step 1*/{0},
	/*step 2*/{0},
	/*step 3*/{0},
	/*step 4*/{0}
};

/* dvfs information */
// L0 = 533Mhz, 1.075V
// L1 = 440Mhz, 1.025V
// L2 = 350Mhz, 0.95V
// L3 = 266Mhz, 0.90V
// L4 = 160Mhz, 0.875V

mali_dvfs_table mali_dvfs_all[MAX_MALI_DVFS_STEPS]={
	{160   ,1000000   ,  875000},
	{266   ,1000000   ,  900000},
	{350   ,1000000   ,  950000},
	{440   ,1000000   , 1025000},
	{533   ,1000000   , 1075000} };

mali_dvfs_table mali_dvfs[MALI_DVFS_STEPS]={
	{160   ,1000000   , 875000},
	{266   ,1000000   , 900000},
	{350   ,1000000   , 950000},
	{440   ,1000000   ,1025000},
	{533   ,1000000   ,1075000}
};

mali_dvfs_threshold_table mali_dvfs_threshold[MALI_DVFS_STEPS]={
	{0   , 50},
	{42  , 50},
	{65  , 70},
	{70  , 80},
	{80  ,100}
};

#ifdef EXYNOS4_ASV_ENABLED
#define ASV_LEVEL     12	/* ASV0, 1, 11 is reserved */

// Yank555.lu : Introducing a voltage table for all available frequencies and ASV levels instead of only for the 5 standard steps

static unsigned int asv_3d_volt_9_table[GPU_FREQ_STEPS][ASV_LEVEL] = {
//	{   ASV 0,   ASV 1,   ASV 2,   ASV 3,   ASV 4,   ASV 5,   ASV 6,   ASV 7,   ASV 8,   ASV 9,  ASV 10,  ASV 11},  /*  ASV Levels       */
	{  945000,  932500,  920000,  907500,  895000,  882500,  870000,  857500,  870000,  857500,  845000,  845000},  /*  0 ( 54Mhz) - new */
	{  947500,  935000,  922500,  910000,  897500,  885000,  872500,  860000,  872500,  860000,  847500,  847500},  /*  1 (108Mhz) - new */
	{  950000,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  875000,  862500,  850000,  850000},  /*  2 (160Mhz) - old */
	{  962500,  950000,  937500,  925000,  912500,  900000,  887500,  875000,  887500,  875000,  862500,  850000},	/*  3 (200Mhz) - new */
	{  975000,  962500,  950000,  937500,  925000,  912500,  900000,  887500,  900000,  887500,  875000,  862500},	/*  4 (266Mhz) - old */
	{  985000,  972500,  960000,  947500,  935000,  922500,  910000,  897500,  910000,  897500,  885000,  872500},	/*  5 (275Mhz) - new */
	{  995000,  982500,  970000,  957500,  945000,  932500,  920000,  907500,  920000,  907500,  895000,  882500},	/*  6 (300Mhz) - new */
	{ 1007500,  995000,  982500,  970000,  957500,  945000,  932500,  920000,  932500,  920000,  907500,  895000},	/*  7 (333Mhz) - new */
	{ 1025000, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  950000,  937500,  925000,  912500},	/*  8 (350Mhz) - old */
	{ 1055000, 1042500, 1030000, 1017500, 1005000,  992500,  980000,  967500,  980000,  967500,  955000,  942500},	/*  9 (400Mhz) - new */
	{ 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1012500, 1000000, 1012500, 1000000,  987500,  975000},	/* 10 (440Mhz) - old */
	{ 1120000, 1107500, 1095000, 1082500, 1070000, 1057500, 1045000, 1032500, 1045000, 1032500, 1020000, 1007500},	/* 11 (500Mhz) - new */
	{ 1150000, 1137500, 1125000, 1112500, 1100000, 1087500, 1075000, 1062500, 1087500, 1075000, 1062500, 1050000},	/* 12 (533Mhz) - old */
	{ 1177500, 1165000, 1152500, 1140000, 1127500, 1115000, 1102500, 1090000, 1115000, 1102500, 1090000, 1077500},	/* 13 (600Mhz) - new */
	{ 1200000, 1187500, 1175000, 1162500, 1150000, 1137500, 1125000, 1112500, 1137500, 1125000, 1112500, 1100000},	/* 14 (640Mhz) - new */
	{ 1222500, 1210000, 1197500, 1185000, 1172500, 1160000, 1147500, 1135000, 1160000, 1147500, 1135000, 1122500},	/* 15 (666Mhz) - new */
	{ 1242500, 1230000, 1217500, 1205000, 1192500, 1180000, 1167500, 1155000, 1180000, 1167500, 1155000, 1142500},	/* 16 (700Mhz) - new */
	{ 1260000, 1247500, 1235000, 1222500, 1210000, 1197500, 1185000, 1172500, 1197500, 1185000, 1172500, 1160000},	/* 17 (733Mhz) - new */
	{ 1277500, 1265000, 1252500, 1240000, 1227500, 1215000, 1202500, 1190000, 1215000, 1202500, 1190000, 1177500},	/* 18 (750Mhz) - new */
	{ 1295000, 1282500, 1270000, 1257500, 1245000, 1232500, 1220000, 1207500, 1232500, 1220000, 1207500, 1195000} 	/* 19 (800Mhz) - new */
};

// Yank555.lu : Lookup table for possible frequencies
unsigned int gpu_freq_table[GPU_FREQ_STEPS+1] = {
	 54,
	108,
	160,
	200,
	266,
	275,
	300,
	333,
	350,
	400,
	440,
	500,
	533,
	600,
  	640,
  	666,
  	700,
  	733,
  	750,
  	800,
  	GPU_FREQ_END_OF_TABLE
};

// Yank555.lu : Old voltage table kept for reference
//
//static unsigned int asv_3d_volt_9_table[MALI_DVFS_STEPS-1][ASV_LEVEL] = {
//	{  950000,  925000,  900000,  900000,  875000,  875000,  875000,  875000,  850000,  850000,  850000,  850000},  /* L3(160Mhz) */
//	{  975000,  950000,  925000,  925000,  925000,  900000,  900000,  875000,  875000,  875000,  875000,  850000},	/* L2(266Mhz) */
//	{ 1050000, 1025000, 1000000, 1000000,  975000,  950000,  950000,  950000,  925000,  925000,  925000,  900000},	/* L1(350Mhz) */
//	{ 1100000, 1075000, 1050000, 1050000, 1050000, 1025000, 1025000, 1000000, 1000000, 1000000,  975000,  950000},	/* L0(440Mhz) */
//};
//
//static unsigned int asv_3d_volt_9_table_for_prime[MALI_DVFS_STEPS][ASV_LEVEL_PRIME] = {
//	{  950000,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  875000,  862500,  850000,  850000},  /* L4(160Mhz) */
//	{  975000,  962500,  950000,  937500,  925000,  912500,  900000,  887500,  900000,  887500,  875000,  862500},	/* L3(266Mhz) */
//	{ 1025000, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  950000,  937500,  925000,  912500},	/* L2(350Mhz) */
//	{ 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1012500, 1000000, 1012500, 1000000,  987500,  975000},	/* L1(440Mhz) */
//	{ 1150000, 1137500, 1125000, 1112500, 1100000, 1087500, 1075000, 1062500, 1087500, 1075000, 1062500, 1050000},	/* L0(600Mhz) */
//};

#endif /* ASV_LEVEL */

/*dvfs status*/
mali_dvfs_currentstatus maliDvfsStatus;
int mali_dvfs_control=0;

static u32 mali_dvfs_utilization = 255;

static void mali_dvfs_work_handler(struct work_struct *w);

static struct workqueue_struct *mali_dvfs_wq = 0;
extern mali_io_address clk_register_map;
extern _mali_osk_lock_t *mali_dvfs_lock;

int mali_runtime_resumed = -1;

static DECLARE_WORK(mali_dvfs_work, mali_dvfs_work_handler);

static unsigned int get_mali_dvfs_status(void)
{
	return maliDvfsStatus.currentStep;
}
#if MALI_PMM_RUNTIME_JOB_CONTROL_ON
int get_mali_dvfs_control_status(void)
{
	return mali_dvfs_control;
}

mali_bool set_mali_dvfs_current_step(unsigned int step)
{
	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	maliDvfsStatus.currentStep = step % MAX_MALI_DVFS_STEPS;
	if (step >= MAX_MALI_DVFS_STEPS)
		mali_runtime_resumed = maliDvfsStatus.currentStep;
	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	return MALI_TRUE;
}
#endif
static mali_bool set_mali_dvfs_status(u32 step,mali_bool boostup)
{
	u32 validatedStep=step;

#ifdef CONFIG_REGULATOR
	if (mali_regulator_get_usecount() == 0) {
		MALI_DEBUG_PRINT(1, ("regulator use_count is 0 \n"));
		return MALI_FALSE;
	}
#endif

	if (boostup) {
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
	} else {
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
	}

#ifdef EXYNOS4_ASV_ENABLED
	if (samsung_rev() < EXYNOS4412_REV_2_0) {
		if (mali_dvfs[step].clock == 160)
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_100V);
		else
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_130V);
	}
#endif


	set_mali_dvfs_current_step(validatedStep);
	/*for future use*/
	maliDvfsStatus.pCurrentDvfs = &mali_dvfs[validatedStep];

	return MALI_TRUE;
}

static void mali_platform_wating(u32 msec)
{
	/*sample wating
	change this in the future with proper check routine.
	*/
	unsigned int read_val;
	while(1) {
		read_val = _mali_osk_mem_ioread32(clk_register_map, 0x00);
		if ((read_val & 0x8000)==0x0000) break;
			_mali_osk_time_ubusydelay(100); // 1000 -> 100 : 20101218
		}
		/* _mali_osk_time_ubusydelay(msec*1000);*/
}

static mali_bool change_mali_dvfs_status(u32 step, mali_bool boostup )
{

	MALI_DEBUG_PRINT(1, ("> change_mali_dvfs_status: %d, %d \n",step, boostup));

	if (!set_mali_dvfs_status(step, boostup)) {
		MALI_DEBUG_PRINT(1, ("error on set_mali_dvfs_status: %d, %d \n",step, boostup));
		return MALI_FALSE;
	}

	/*wait until clock and voltage is stablized*/
	mali_platform_wating(MALI_DVFS_WATING); /*msec*/

	return MALI_TRUE;
}

#ifdef EXYNOS4_ASV_ENABLED
extern unsigned int exynos_result_of_asv;

mali_bool mali_dvfs_table_update(void)
{
	unsigned int i;
	unsigned int j;
	unsigned int step_num = MALI_DVFS_STEPS;

	// Yank555.lu : Update voltage for all 5 freq. steps based on true freq. table
	for (i = 0; i < step_num; i++) {

		for (j = 0; gpu_freq_table[j] != GPU_FREQ_END_OF_TABLE; j++) {

			if (gpu_freq_table[j] == mali_dvfs[i].clock) { // Yank555.lu : if we have found the right freq. step, use that voltage

				MALI_PRINT((":::exynos_result_of_asv : %d\n", exynos_result_of_asv));
				mali_dvfs[i].vol = asv_3d_volt_9_table[j][exynos_result_of_asv];
				MALI_PRINT(("mali_dvfs[%d].vol = %d (%dMHz)\n", i, mali_dvfs[i].vol, mali_dvfs[i].clock));
				break; // No need to go on

			}

		}

	}

	return MALI_TRUE;
}
#endif

static unsigned int decideNextStatus(unsigned int utilization)
{
	static unsigned int level = 0; // 0:stay, 1:up

	if (mali_runtime_resumed >= 0) {
		level = mali_runtime_resumed;
		mali_runtime_resumed = -1;
		return level;
	}

	if (mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold
			<= mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold) {
		MALI_PRINT(("upthreadshold is smaller than downthreshold: %d < %d\n",
				mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold,
				mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold));
		return level;
	}

	if (utilization > (int)(255 * mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold / 100) &&
			level < MALI_DVFS_STEPS - 1) {
		level++;
	}
	if (utilization < (int)(255 * mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold / 100) &&
			level > 0) {
		level--;
	}

	if (_mali_osk_atomic_read(&bottomlock_status) > 0) {
		if (level < bottom_lock_step)
			level = bottom_lock_step;
	}

	return level;
}

static mali_bool mali_dvfs_status(u32 utilization)
{
	unsigned int nextStatus = 0;
	unsigned int curStatus = 0;
	mali_bool boostup = MALI_FALSE;
	static int stay_count = 0;

	MALI_DEBUG_PRINT(1, ("> mali_dvfs_status: %d \n",utilization));

	/*decide next step*/
	curStatus = get_mali_dvfs_status();
	nextStatus = decideNextStatus(utilization);

	MALI_DEBUG_PRINT(1, ("= curStatus %d, nextStatus %d, maliDvfsStatus.currentStep %d \n", curStatus, nextStatus, maliDvfsStatus.currentStep));

	/*if next status is same with current status, don't change anything*/
	if ((curStatus != nextStatus && stay_count == 0)) {
		/*check if boost up or not*/
		if (nextStatus > maliDvfsStatus.currentStep) boostup = 1;

		/*change mali dvfs status*/
		if (!change_mali_dvfs_status(nextStatus,boostup)) {
			MALI_DEBUG_PRINT(1, ("error on change_mali_dvfs_status \n"));
			return MALI_FALSE;
		}
		stay_count = mali_dvfs_staycount[maliDvfsStatus.currentStep].staycount;
	} else {
		if (stay_count > 0)
			stay_count--;
	}

	return MALI_TRUE;
}



int mali_dvfs_is_running(void)
{
	return bMaliDvfsRun;

}



void mali_dvfs_late_resume(void)
{
	// set the init clock as low when resume
	set_mali_dvfs_status(0,0);
}


static void mali_dvfs_work_handler(struct work_struct *w)
{
	bMaliDvfsRun=1;

	MALI_DEBUG_PRINT(3, ("=== mali_dvfs_work_handler\n"));

	if (!mali_dvfs_status(mali_dvfs_utilization))
		MALI_DEBUG_PRINT(1,( "error on mali dvfs status in mali_dvfs_work_handler"));

	bMaliDvfsRun=0;
}

mali_bool init_mali_dvfs_status(int step)
{
	/*default status
	add here with the right function to get initilization value.
	*/
	if (!mali_dvfs_wq)
		mali_dvfs_wq = create_singlethread_workqueue("mali_dvfs");

	_mali_osk_atomic_init(&bottomlock_status, 0);

	/*add a error handling here*/
	set_mali_dvfs_current_step(step);

#ifdef EXYNOS4_ASV_ENABLED
	mali_dvfs_table_update();
	change_mali_dvfs_status(1, 0);
#endif

	return MALI_TRUE;
}

void deinit_mali_dvfs_status(void)
{
	if (mali_dvfs_wq)
		destroy_workqueue(mali_dvfs_wq);

	_mali_osk_atomic_term(&bottomlock_status);

	mali_dvfs_wq = NULL;
}

mali_bool mali_dvfs_handler(u32 utilization)
{
	mali_dvfs_utilization = utilization;
	queue_work_on(0, mali_dvfs_wq,&mali_dvfs_work);

	/*add error handle here*/
	return MALI_TRUE;
}

void mali_default_step_set(int step, mali_bool boostup)
{
	mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);

	if (maliDvfsStatus.currentStep == 1)
		set_mali_dvfs_status(step, boostup);
}

int mali_dvfs_bottom_lock_push(int lock_step)
{
	int prev_status = _mali_osk_atomic_read(&bottomlock_status);

	if (prev_status < 0) {
		MALI_PRINT(("gpu bottom lock status is not valid for push\n"));
		return -1;
	}
	if (bottom_lock_step < lock_step) {
		bottom_lock_step = lock_step;
		if (get_mali_dvfs_status() < lock_step) {
			mali_regulator_set_voltage(mali_dvfs[lock_step].vol, mali_dvfs[lock_step].vol);
			mali_clk_set_rate(mali_dvfs[lock_step].clock, mali_dvfs[lock_step].freq);
			set_mali_dvfs_current_step(lock_step);
		}
	}
	return _mali_osk_atomic_inc_return(&bottomlock_status);
}

int mali_dvfs_bottom_lock_pop(void)
{
	int prev_status = _mali_osk_atomic_read(&bottomlock_status);
	if (prev_status <= 0) {
		MALI_PRINT(("gpu bottom lock status is not valid for pop\n"));
		return -1;
	} else if (prev_status == 1) {
		bottom_lock_step = 0;
		MALI_PRINT(("gpu bottom lock release\n"));
	}

	return _mali_osk_atomic_dec_return(&bottomlock_status);
}


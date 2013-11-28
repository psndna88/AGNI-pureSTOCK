/* drivers/gpu/mali400/mali/platform/pegasus-m400/exynos4_pmm.c
 *
 * Copyright 2011 by S.LSI. Samsung Electronics Inc.
 * San#24, Nongseo-Dong, Giheung-Gu, Yongin, Korea
 *
 * Samsung SoC Mali400 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/**
 * @file exynos4_pmm.c
 * Platform specific Mali driver functions for the exynos 4XXX based platforms
 */

#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "exynos4_pmm.h"
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#if defined(CONFIG_MALI400_PROFILING)
#include "mali_osk_profiling.h"
#endif

#if defined(CONFIG_PM_RUNTIME)
#include <plat/pd.h>
#endif

#include <asm/io.h>
#include <mach/regs-pmu.h>

#include <linux/workqueue.h>

#ifdef CONFIG_CPU_EXYNOS4210
#define MALI_DVFS_STEPS 3
#define MALI_DVFS_WATING 10 /* msec */
#define MALI_DVFS_DEFAULT_STEP 0
#else
#define MALI_DVFS_STEPS 5
#define MALI_DVFS_WATING 10 /* msec */
#define MALI_DVFS_DEFAULT_STEP 1
#endif

#ifdef CONFIG_CPU_FREQ
#include <mach/asv.h>
#define EXYNOS4_ASV_ENABLED
#endif

#define MALI_DVFS_CLK_DEBUG 0
#define SEC_THRESHOLD 1

#define CPUFREQ_LOCK_DURING_440 0

static int bMaliDvfsRun = 0;
static int needs_update = 0;

typedef struct mali_dvfs_tableTag{
	unsigned int clock;
	unsigned int freq;
	unsigned int vol;
#if SEC_THRESHOLD
	unsigned int downthreshold;
	unsigned int upthreshold;
#endif
}mali_dvfs_table;

typedef struct mali_dvfs_statusTag{
	unsigned int currentStep;
	mali_dvfs_table * pCurrentDvfs;

} mali_dvfs_status_t;

/* dvfs status */
mali_dvfs_status_t maliDvfsStatus;
int mali_dvfs_control;

int step0_clk = 160;
int step0_vol = 875000;
int step1_clk = 266;
int step1_vol = 900000;
int step0_up = 70;
int step1_down = 62;
int step2_clk = 350;
int step2_vol = 950000;
int step1_up = 90;
int step2_down = 85;
int step3_clk = 440;
int step3_vol = 1025000;
int step2_up = 90;
int step3_down = 85;
int step4_clk = 533;
int step4_vol = 1075000;
int step3_up = 90;
int step4_down = 85;

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

typedef struct mali_runtime_resumeTag{
	int clk;
	int vol;
	unsigned int step;
}mali_runtime_resume_table;

mali_runtime_resume_table mali_runtime_resume = {266, 900000, 1};

/* dvfs table */
mali_dvfs_table mali_dvfs[MALI_DVFS_STEPS]={
#if defined(CONFIG_CPU_EXYNOS4212) || defined(CONFIG_CPU_EXYNOS4412)
			/* step 0 */{160  ,1000000	,875000	   , 0   , 70},
			/* step 1 */{266  ,1000000	,900000	   ,62   , 90},
			/* step 2 */{350  ,1000000	,950000	   ,85   , 90},
			/* step 3 */{440  ,1000000	,1025000   ,85   , 90},
			/* step 4 */{533  ,1000000	,1075000   ,85   ,100} };
#else
			/* step 0 */{134  ,1000000	,950000    ,0   , 70},
			/* step 1 */{267  ,1000000	,1050000   ,65   ,90},
			/* step 2 */{400  ,1000000      ,1200000   ,85   ,100} };
#endif

#ifdef EXYNOS4_ASV_ENABLED
#define ASV_LEVEL	 12	/* ASV0, 1, 11 is reserved */
#define ASV_LEVEL_PRIME	 13  /* ASV0, 1, 12 is reserved */
#define ASV_LEVEL_4210_12	8
#define ASV_LEVEL_4210_14	5

#if defined(CONFIG_CPU_EXYNOS4212) || defined(CONFIG_CPU_EXYNOS4412)
static unsigned int asv_3d_volt_9_table_1ghz_type[MALI_DVFS_STEPS-1][ASV_LEVEL] = {
	{  975000,  950000,  950000,  950000,  925000,  925000,  925000,  900000,  900000,  900000,  900000,  875000},  /* L3(160Mhz) */
#if (MALI_DVFS_STEPS > 1)
	{ 1000000,  975000,  975000,  975000,  950000,  950000,  950000,  900000,  900000,  900000,  900000,  875000},  /* L2(266Mhz) */
#if (MALI_DVFS_STEPS > 2)
	{ 1075000, 1050000, 1050000, 1050000, 1000000, 1000000, 1000000,  975000,  975000,  975000,  975000,  925000},  /* L1(350Mhz) */
#if (MALI_DVFS_STEPS > 3)
	{ 1125000, 1100000, 1100000, 1100000, 1075000, 1075000, 1075000, 1025000, 1025000, 1025000, 1025000,  975000},  /* L0(440Mhz) */
#endif
#endif
#endif
};

// Yank555.lu : Introducing a voltage table for all available frequencies and ASV levels instead of only for the 5 standard steps

static unsigned int asv_3d_volt_9_table[GPU_FREQ_STEPS][ASV_LEVEL] = {
//        {   ASV 0,   ASV 1,   ASV 2,   ASV 3,   ASV 4,   ASV 5,   ASV 6,   ASV 7,   ASV 8,   ASV 9,  ASV 10,  ASV 11},  /*  ASV Levels       */
        {  945000,  932500,  920000,  907500,  895000,  882500,  870000,  857500,  870000,  857500,  845000,  845000},  /*  0 ( 54Mhz) - new */
        {  947500,  935000,  922500,  910000,  897500,  885000,  872500,  860000,  872500,  860000,  847500,  847500},  /*  1 (108Mhz) - new */
        {  950000,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  875000,  862500,  850000,  850000},  /*  2 (160Mhz) - old */
        {  962500,  950000,  937500,  925000,  912500,  900000,  887500,  875000,  887500,  875000,  862500,  850000},        /*  3 (200Mhz) - new */
        {  975000,  962500,  950000,  937500,  925000,  912500,  900000,  887500,  900000,  887500,  875000,  862500},        /*  4 (266Mhz) - old */
        {  985000,  972500,  960000,  947500,  935000,  922500,  910000,  897500,  910000,  897500,  885000,  872500},        /*  5 (275Mhz) - new */
        {  995000,  982500,  970000,  957500,  945000,  932500,  920000,  907500,  920000,  907500,  895000,  882500},        /*  6 (300Mhz) - new */
        { 1007500,  995000,  982500,  970000,  957500,  945000,  932500,  920000,  932500,  920000,  907500,  895000},        /*  7 (333Mhz) - new */
        { 1025000, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  950000,  937500,  925000,  912500},        /*  8 (350Mhz) - old */
        { 1055000, 1042500, 1030000, 1017500, 1005000,  992500,  980000,  967500,  980000,  967500,  955000,  942500},        /*  9 (400Mhz) - new */
        { 1087500, 1075000, 1062500, 1050000, 1037500, 1025000, 1012500, 1000000, 1012500, 1000000,  987500,  975000},        /* 10 (440Mhz) - old */
        { 1120000, 1107500, 1095000, 1082500, 1070000, 1057500, 1045000, 1032500, 1045000, 1032500, 1020000, 1007500},        /* 11 (500Mhz) - new */
        { 1150000, 1137500, 1125000, 1112500, 1100000, 1087500, 1075000, 1062500, 1087500, 1075000, 1062500, 1050000},        /* 12 (533Mhz) - old */
        { 1177500, 1165000, 1152500, 1140000, 1127500, 1115000, 1102500, 1090000, 1115000, 1102500, 1090000, 1077500},        /* 13 (600Mhz) - new */
        { 1200000, 1187500, 1175000, 1162500, 1150000, 1137500, 1125000, 1112500, 1137500, 1125000, 1112500, 1100000},        /* 14 (640Mhz) - new */
        { 1222500, 1210000, 1197500, 1185000, 1172500, 1160000, 1147500, 1135000, 1160000, 1147500, 1135000, 1122500},        /* 15 (666Mhz) - new */
        { 1242500, 1230000, 1217500, 1205000, 1192500, 1180000, 1167500, 1155000, 1180000, 1167500, 1155000, 1142500},        /* 16 (700Mhz) - new */
        { 1260000, 1247500, 1235000, 1222500, 1210000, 1197500, 1185000, 1172500, 1197500, 1185000, 1172500, 1160000},        /* 17 (733Mhz) - new */
        { 1277500, 1265000, 1252500, 1240000, 1227500, 1215000, 1202500, 1190000, 1215000, 1202500, 1190000, 1177500},        /* 18 (750Mhz) - new */
        { 1295000, 1282500, 1270000, 1257500, 1245000, 1232500, 1220000, 1207500, 1232500, 1220000, 1207500, 1195000}         /* 19 (800Mhz) - new */
};


/*static unsigned int asv_3d_volt_9_table[MALI_DVFS_STEPS-1][ASV_LEVEL] = {
	{  950000,  925000,  900000,  900000,  875000,  875000,  875000,  875000,  850000,  850000,  850000,  850000},  //L3(160Mhz)
#if (MALI_DVFS_STEPS > 1)
	{  975000,  950000,  925000,  925000,  925000,  900000,  900000,  875000,  875000,  875000,  875000,  850000},  // L2(266Mhz)
#if (MALI_DVFS_STEPS > 2)
	{ 1050000, 1025000, 1000000, 1000000,  975000,  950000,  950000,  950000,  925000,  925000,  925000,  900000},  // L1(350Mhz)
#if (MALI_DVFS_STEPS > 3)
	{ 1100000, 1075000, 1050000, 1050000, 1050000, 1025000, 1025000, 1000000, 1000000, 1000000,  975000,  950000},  // L0(440Mhz)
#endif
#endif
#endif
};
*/
static unsigned int asv_3d_volt_9_table_for_prime[MALI_DVFS_STEPS][ASV_LEVEL_PRIME] = {
	{  962500,  937500,  925000,  912500,  900000,  887500,  875000,  862500,  875000,  862500,  850000,  850000,  850000},  /* L4(160Mhz) */
#if (MALI_DVFS_STEPS > 1)
	{  987500,  962500,  950000,  937500,  925000,  912500,  900000,  887500,  900000,  887500,  875000,  875000,  875000}, /* L3(266Mhz) */
#if (MALI_DVFS_STEPS > 2)
	{ 1037500, 1012500, 1000000,  987500,  975000,  962500,  950000,  937500,  950000,  937500,  912500,  900000,  887500}, /* L2(350Mhz) */
#if (MALI_DVFS_STEPS > 3)
	{ 1100000, 1075000, 1062500, 1050000, 1037500, 1025000, 1012500, 1000000, 1012500, 1000000,  975000,  962500,  950000}, /* L1(440Mhz) */
#if (MALI_DVFS_STEPS > 4)
	{ 1162500, 1137500, 1125000, 1112500, 1100000, 1087500, 1075000, 1062500, 1075000, 1062500, 1037500, 1025000, 1012500}, /* L0(533Mhz) */
#endif
#endif
#endif
#endif
};

#else

static unsigned int asv_3d_volt_4210_12_table[MALI_DVFS_STEPS][ASV_LEVEL_4210_12] = {
	{  1000000,  1000000,  1000000,   950000,   950000,   950000,   950000,   950000},	/* L2(134Mhz) */
#if (MALI_DVFS_STEPS > 1)
	{  1100000,  1100000,  1100000,  1000000,  1000000,  1000000,  1000000,   950000},	/* L1(266Mhz) */
#if (MALI_DVFS_STEPS > 2)
	{  1200000,  1200000,  1200000,  1100000,  1100000,  1100000,  1050000,  1050000},	/* L0(400Mhz) */
#endif
#endif
};

static unsigned int asv_3d_volt_4210_14_table[MALI_DVFS_STEPS][ASV_LEVEL_4210_14] = {
	{  1000000,  1000000,   950000,   950000,   950000},	/* L2(134Mhz) */
#if (MALI_DVFS_STEPS > 1)
	{  1100000,  1100000,  1000000,  1000000,   950000},	/* L1(266Mhz) */
#if (MALI_DVFS_STEPS > 2)
	{  1200000,  1200000,  1100000,  1050000,  1050000},	/* L0(400Mhz) */
#endif
#endif
};
#endif
#endif /* ASV_LEVEL */

#define EXTXTALCLK_NAME  "ext_xtal"
#define VPLLSRCCLK_NAME  "vpll_src"
#define FOUTVPLLCLK_NAME "fout_vpll"
#define SCLVPLLCLK_NAME  "sclk_vpll"
#define GPUMOUT1CLK_NAME "mout_g3d1"

#define MPLLCLK_NAME	 "mout_mpll"
#define GPUMOUT0CLK_NAME "mout_g3d0"
#define GPUCLK_NAME	  "sclk_g3d"
#define CLK_DIV_STAT_G3D 0x1003C62C
#define CLK_DESC		 "clk-divider-status"

static struct clk *ext_xtal_clock	= NULL;
static struct clk *vpll_src_clock	= NULL;
static struct clk *fout_vpll_clock   = NULL;
static struct clk *sclk_vpll_clock   = NULL;

static struct clk *mpll_clock		= NULL;
static struct clk *mali_parent_clock = NULL;
static struct clk  *mali_mout0_clock = NULL;
static struct clk *mali_clock		= NULL;

#if defined(CONFIG_CPU_EXYNOS4412) || defined(CONFIG_CPU_EXYNOS4212)
/* Pegasus */
static const mali_bool bis_vpll = MALI_TRUE;
int mali_gpu_clk = 440;
int mali_gpu_vol = 1025000;
#else
/* Orion */
static const mali_bool bis_vpll = MALI_FALSE;
int mali_gpu_clk = 267;
int mali_gpu_vol = 1050000;
#endif

static unsigned int GPU_MHZ	= 1000000;

int  gpu_power_state;
static int bPoweroff;
atomic_t clk_active;

#define MAX_MALI_DVFS_STEPS 5
static _mali_osk_atomic_t bottomlock_status;
int bottom_lock_step = 0;

#if MALI_VOLTAGE_LOCK
int mali_lock_vol = 0;
static _mali_osk_atomic_t voltage_lock_status;
static mali_bool mali_vol_lock_flag = 0;
#endif

/* Declare for sysfs */
#ifdef CONFIG_MALI_DVFS
module_param(mali_dvfs_control, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP| S_IROTH); /* rw-rw-r-- */
MODULE_PARM_DESC(mali_dvfs_control, "Mali Current DVFS");

DEVICE_ATTR(time_in_state, S_IRUGO|S_IWUSR, show_time_in_state, set_time_in_state);
MODULE_PARM_DESC(time_in_state, "Time-in-state of Mali DVFS");
#endif

module_param(mali_gpu_clk, int, S_IRUSR | S_IRGRP | S_IROTH); /* r--r--r-- */
MODULE_PARM_DESC(mali_gpu_clk, "Mali Current Clock");

module_param(mali_gpu_vol, int, S_IRUSR | S_IRGRP | S_IROTH); /* r--r--r-- */
MODULE_PARM_DESC(mali_gpu_vol, "Mali Current Voltage");

module_param(gpu_power_state, int, S_IRUSR | S_IRGRP | S_IROTH); /* r--r--r-- */
MODULE_PARM_DESC(gpu_power_state, "Mali Power State");

#ifdef CONFIG_REGULATOR
struct regulator *g3d_regulator = NULL;
#endif

mali_io_address clk_register_map = 0;

/* DVFS */
static unsigned int mali_dvfs_utilization = 255;
static void mali_dvfs_work_handler(struct work_struct *w);
static struct workqueue_struct *mali_dvfs_wq = 0;
extern mali_io_address clk_register_map;
_mali_osk_lock_t *mali_dvfs_lock;
int mali_runtime_resumed = -1;
static DECLARE_WORK(mali_dvfs_work, mali_dvfs_work_handler);

#ifdef CONFIG_REGULATOR
void mali_regulator_disable(void)
{
	if(IS_ERR_OR_NULL(g3d_regulator))
	{
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_disable : g3d_regulator is null\n"));
		return;
	}
	regulator_disable(g3d_regulator);
}

void mali_regulator_enable(void)
{
	if(IS_ERR_OR_NULL(g3d_regulator))
	{
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_enable : g3d_regulator is null\n"));
		return;
	}
	regulator_enable(g3d_regulator);
}

void mali_regulator_set_voltage(int min_uV, int max_uV)
{
	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	if(IS_ERR_OR_NULL(g3d_regulator))
	{
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_set_voltage : g3d_regulator is null\n"));
		return;
	}
	MALI_DEBUG_PRINT(1, ("= regulator_set_voltage: %d, %d \n",min_uV, max_uV));
	regulator_set_voltage(g3d_regulator, min_uV, max_uV);
	mali_gpu_vol = regulator_get_voltage(g3d_regulator);
	MALI_DEBUG_PRINT(1, ("Mali voltage: %d\n", mali_gpu_vol));
	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
}
#endif

unsigned long mali_clk_get_rate(void)
{
	return clk_get_rate(mali_clock);
}


static unsigned int get_mali_dvfs_status(void)
{
	return maliDvfsStatus.currentStep;
}

mali_bool mali_clk_get(void)
{
	if (bis_vpll)
	{
		if (ext_xtal_clock == NULL)
		{
			ext_xtal_clock = clk_get(NULL, EXTXTALCLK_NAME);
			if (IS_ERR(ext_xtal_clock)) {
				MALI_PRINT(("MALI Error : failed to get source ext_xtal_clock\n"));
				return MALI_FALSE;
			}
		}

		if (vpll_src_clock == NULL)
		{
			vpll_src_clock = clk_get(NULL, VPLLSRCCLK_NAME);
			if (IS_ERR(vpll_src_clock)) {
				MALI_PRINT(("MALI Error : failed to get source vpll_src_clock\n"));
				return MALI_FALSE;
			}
		}

		if (fout_vpll_clock == NULL)
		{
			fout_vpll_clock = clk_get(NULL, FOUTVPLLCLK_NAME);
			if (IS_ERR(fout_vpll_clock)) {
				MALI_PRINT(("MALI Error : failed to get source fout_vpll_clock\n"));
				return MALI_FALSE;
			}
		}

		if (sclk_vpll_clock == NULL)
		{
			sclk_vpll_clock = clk_get(NULL, SCLVPLLCLK_NAME);
			if (IS_ERR(sclk_vpll_clock)) {
				MALI_PRINT(("MALI Error : failed to get source sclk_vpll_clock\n"));
				return MALI_FALSE;
			}
		}

		if (mali_parent_clock == NULL)
		{
			mali_parent_clock = clk_get(NULL, GPUMOUT1CLK_NAME);

			if (IS_ERR(mali_parent_clock)) {
				MALI_PRINT(( "MALI Error : failed to get source mali parent clock\n"));
				return MALI_FALSE;
			}
		}

		if (mali_mout0_clock == NULL)
		{
			mali_mout0_clock = clk_get(NULL, GPUMOUT0CLK_NAME);

			if (IS_ERR(mali_mout0_clock)) {
				MALI_PRINT( ( "MALI Error : failed to get source mali mout0 clock\n"));
				return MALI_FALSE;
			}
		}
	}
	else /* mpll */
	{
		if (mpll_clock == NULL)
		{
			mpll_clock = clk_get(NULL, MPLLCLK_NAME);

			if (IS_ERR(mpll_clock)) {
				MALI_PRINT(("MALI Error : failed to get source mpll clock\n"));
				return MALI_FALSE;
			}
		}

		if (mali_parent_clock == NULL)
		{
			mali_parent_clock = clk_get(NULL, GPUMOUT0CLK_NAME);

			if (IS_ERR(mali_parent_clock)) {
				MALI_PRINT(( "MALI Error : failed to get source mali parent clock\n"));
				return MALI_FALSE;
			}
		}
	}

	/* mali clock get always. */
	if (mali_clock == NULL)
	{
		mali_clock = clk_get(NULL, GPUCLK_NAME);

		if (IS_ERR(mali_clock)) {
			MALI_PRINT(("MALI Error : failed to get source mali clock\n"));
			return MALI_FALSE;
		}
	}

	return MALI_TRUE;
}

void mali_clk_put(mali_bool binc_mali_clock)
{
	if (mali_parent_clock)
	{
		clk_put(mali_parent_clock);
		mali_parent_clock = NULL;
	}

	if (mali_mout0_clock)
	{
		clk_put(mali_mout0_clock);
		mali_mout0_clock = NULL;
	}

	if (mpll_clock)
	{
		clk_put(mpll_clock);
		mpll_clock = NULL;
	}

	if (sclk_vpll_clock)
	{
		clk_put(sclk_vpll_clock);
		sclk_vpll_clock = NULL;
	}

	if (binc_mali_clock && fout_vpll_clock)
	{
		clk_put(fout_vpll_clock);
		fout_vpll_clock = NULL;
	}

	if (vpll_src_clock)
	{
		clk_put(vpll_src_clock);
		vpll_src_clock = NULL;
	}

	if (ext_xtal_clock)
	{
		clk_put(ext_xtal_clock);
		ext_xtal_clock = NULL;
	}

	if (binc_mali_clock && mali_clock)
	{
		clk_put(mali_clock);
		mali_clock = NULL;
	}
}

void mali_clk_set_rate(unsigned int clk, unsigned int mhz)
{
	int err;
	unsigned long rate = (unsigned long)clk * (unsigned long)mhz;

	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	MALI_DEBUG_PRINT(3, ("Mali platform: Setting frequency to %d mhz\n", clk));

	if (mali_clk_get() == MALI_FALSE)
		return;

	if (bis_vpll)
	{
		clk_set_rate(mali_clock, (clk_get_rate(mali_clock) / 5));
		clk_set_parent(mali_mout0_clock, mpll_clock);
		clk_set_parent(mali_clock, mali_mout0_clock);
		clk_set_rate(mali_clock, 160000000);

		clk_set_rate(fout_vpll_clock, (unsigned int)clk * GPU_MHZ);
		clk_set_parent(mali_parent_clock, sclk_vpll_clock);
		clk_set_parent(mali_clock, mali_parent_clock);
	}
	else
	{
		clk_set_parent(mali_parent_clock, mpll_clock);
		clk_set_parent(mali_clock, mali_parent_clock);
	}

	if (atomic_read(&clk_active) == 0) {
		if (clk_enable(mali_clock) < 0)
			return;
		atomic_set(&clk_active, 1);
	}

	err = clk_set_rate(mali_clock, rate);
	if (err > 0)
		MALI_PRINT_ERROR(("Failed to set Mali clock: %d\n", err));

	rate = mali_clk_get_rate();

	MALI_DEBUG_PRINT(1, ("Mali frequency %d\n", rate / mhz));
	GPU_MHZ = mhz;
	mali_gpu_clk = (int)(rate / mhz);

	mali_clk_put(MALI_FALSE);

	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
}

int get_mali_dvfs_control_status(void)
{
	return mali_dvfs_control;
}

mali_bool set_mali_dvfs_current_step(unsigned int step)
{
	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	maliDvfsStatus.currentStep = step % MALI_DVFS_STEPS;
	if (step >= MALI_DVFS_STEPS)
		mali_runtime_resumed = maliDvfsStatus.currentStep;

	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
	return MALI_TRUE;
}


static mali_bool set_mali_dvfs_status(u32 step,mali_bool boostup)
{
	u32 validatedStep=step;
#if MALI_DVFS_CLK_DEBUG
	unsigned int *pRegMaliClkDiv;
	unsigned int *pRegMaliMpll;
#endif

	if(boostup)	{
#ifdef CONFIG_REGULATOR
		/* change the voltage */
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
		/* change the clock */
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
	} else {
		/* change the clock */
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
#ifdef CONFIG_REGULATOR
		/* change the voltage */
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
	}

#if defined(CONFIG_MALI400_PROFILING)
	_mali_osk_profiling_add_event(MALI_PROFILING_EVENT_TYPE_SINGLE|
			MALI_PROFILING_EVENT_CHANNEL_GPU|
			MALI_PROFILING_EVENT_REASON_SINGLE_GPU_FREQ_VOLT_CHANGE,
			mali_gpu_clk, mali_gpu_vol/1000, 0, 0, 0);
#endif
	mali_clk_put(MALI_FALSE);

#if MALI_DVFS_CLK_DEBUG
	pRegMaliClkDiv = ioremap(0x1003c52c,32);
	pRegMaliMpll = ioremap(0x1003c22c,32);
	MALI_PRINT(("Mali MPLL reg:%d, CLK DIV: %d \n",*pRegMaliMpll, *pRegMaliClkDiv));
#endif

#ifdef EXYNOS4_ASV_ENABLED
	if (samsung_rev() < EXYNOS4412_REV_2_0) {
		if (mali_dvfs[step].clock == 160)
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_100V);
		else
			exynos4x12_set_abb_member(ABB_G3D, ABB_MODE_130V);
	}
#endif

	set_mali_dvfs_current_step(validatedStep);
	/* for future use */
	maliDvfsStatus.pCurrentDvfs = &mali_dvfs[validatedStep];

#if CPUFREQ_LOCK_DURING_440
	/* lock/unlock CPU freq by Mali */
	if (mali_dvfs[step].clock == 440)
		err = cpufreq_lock_by_mali(1200);
	else
		cpufreq_unlock_by_mali();
#endif


	return MALI_TRUE;
}

static void mali_platform_wating(u32 msec)
{
	/*
	* sample wating
	* change this in the future with proper check routine.
	*/
	unsigned int read_val;
	while(1)
	{
		read_val = _mali_osk_mem_ioread32(clk_register_map, 0x00);
		if ((read_val & 0x8000)==0x0000) break;

		_mali_osk_time_ubusydelay(100); /* 1000 -> 100 : 20101218 */
	}
}

static mali_bool change_mali_dvfs_status(u32 step, mali_bool boostup )
{
	MALI_DEBUG_PRINT(4, ("> change_mali_dvfs_status: %d, %d \n",step, boostup));

	if(!set_mali_dvfs_status(step, boostup))
	{
		MALI_DEBUG_PRINT(1, ("error on set_mali_dvfs_status: %d, %d \n",step, boostup));
		return MALI_FALSE;
	}

	/* wait until clock and voltage is stablized */
	mali_platform_wating(MALI_DVFS_WATING); /* msec */

	return MALI_TRUE;
}

#ifdef EXYNOS4_ASV_ENABLED
extern unsigned int exynos_result_of_asv;

mali_bool mali_dvfs_table_update(void)
{
	unsigned int step_num = MALI_DVFS_STEPS;

#if defined(CONFIG_CPU_EXYNOS4212) || defined(CONFIG_CPU_EXYNOS4412)
        unsigned int i;
        unsigned int j;

        // Yank555.lu : Update voltage for all 5 freq. steps based on true freq. table
        for (i = 0; i < step_num; i++) {

                for (j = 0; gpu_freq_table[j] != GPU_FREQ_END_OF_TABLE; j++) {

                        if (gpu_freq_table[j] == mali_dvfs[i].clock) { // Yank555.lu : if we have found the right freq. step, use that voltage

                                MALI_PRINT((":::exynos_result_of_asv : %d\n", exynos_result_of_asv));
                                mali_dvfs[i].vol = max((unsigned int) MIN_VOLTAGE_GPU, min((unsigned int) MAX_VOLTAGE_GPU, asv_3d_volt_9_table[j][exynos_result_of_asv] + gpu_voltage_delta));
                                MALI_PRINT(("mali_dvfs[%d].vol = %d (%dMHz)\n", i, mali_dvfs[i].vol, mali_dvfs[i].clock));
                                break; // No need to go on

                        }

                }

        }
#else
	unsigned int i, exynos_result_of_asv_group, target_asv;

	exynos_result_of_asv_group = exynos_result_of_asv & 0xf;
	target_asv = exynos_result_of_asv >> 28;
	MALI_PRINT(("exynos_result_of_asv_group = 0x%x, target_asv = 0x%x\n", exynos_result_of_asv_group, target_asv));

	for (i = 0; i < step_num; i++) {
		if (target_asv == 0x8) { //SUPPORT_1400MHZ
			mali_dvfs[i].vol = asv_3d_volt_4210_14_table[i][exynos_result_of_asv_group];
		} else if (target_asv == 0x4){ //SUPPORT_1200MHZ
			mali_dvfs[i].vol = asv_3d_volt_4210_12_table[i][exynos_result_of_asv_group];
		}
		MALI_PRINT(("mali_dvfs[%d].vol = %d \n", i, mali_dvfs[i].vol));

		// Update voltage using for resume
		if (mali_runtime_resume.clk == mali_dvfs[i].clock) {
			mali_runtime_resume.vol = mali_dvfs[i].vol;

			MALI_PRINT(("mali_runtime_resume.vol = %d \n", mali_runtime_resume.vol));
		}

		// update voltage using for init timing
		if (mali_gpu_clk == mali_dvfs[i].clock) {
			mali_gpu_vol = mali_dvfs[i].vol;

			MALI_PRINT(("init_gpu_vol = %d \n", mali_gpu_vol));
		}
	}
#endif

	return MALI_TRUE;
}
#endif


static unsigned int decideNextStatus(unsigned int utilization)
{
	static unsigned int level = 0;
	int iStepCount = 0;
	if (mali_runtime_resumed >= 0) {
		level = mali_runtime_resumed;
		mali_runtime_resumed = -1;
	}

	if (mali_dvfs_control == 0 && level == get_mali_dvfs_status()) {
		if (utilization > (int)(255 * mali_dvfs[maliDvfsStatus.currentStep].upthreshold / 100) &&
				level < MALI_DVFS_STEPS - 1) {
			level++;
#if 0 /* this prevents the usage of 5th step -gm */
			if ((samsung_rev() < EXYNOS4412_REV_2_0) && 3 == get_mali_dvfs_status()) {
				level=get_mali_dvfs_status();
			}
#endif
		}
		else if (utilization < (int)(255 * mali_dvfs[maliDvfsStatus.currentStep].downthreshold / 100) &&
				level > 0) {
			level--;
		}
	} else {
		for (iStepCount = MALI_DVFS_STEPS-1; iStepCount >= 0; iStepCount--) {
			if ( mali_dvfs_control >= mali_dvfs[iStepCount].clock ) {
				maliDvfsStatus.currentStep = iStepCount;
				level = iStepCount;
				break;
			}
		}
	}

	return level;
}

static void update_time_in_state(int level);
static mali_bool mali_dvfs_status(unsigned int utilization)
{
	unsigned int nextStatus = 0;
	unsigned int curStatus = 0;
	mali_bool boostup = MALI_FALSE;
/*#ifdef EXYNOS4_ASV_ENABLED
	static mali_bool asv_applied = MALI_FALSE;
#endif

#ifdef EXYNOS4_ASV_ENABLED
	if (asv_applied == MALI_FALSE) {
		mali_dvfs_table_update();
		change_mali_dvfs_status(1, 0);
		asv_applied = MALI_TRUE;

		return MALI_TRUE;
	}
#endif*/

	MALI_DEBUG_PRINT(4, ("> mali_dvfs_status: %d \n",utilization));

	/* decide next step */
	curStatus = get_mali_dvfs_status();
	nextStatus = decideNextStatus(utilization);

	MALI_DEBUG_PRINT(4, ("= curStatus %d, nextStatus %d, maliDvfsStatus.currentStep %d \n", curStatus, nextStatus, maliDvfsStatus.currentStep));
	/* if next status is same with current status, don't change anything */
	if(curStatus!=nextStatus)
	{
		/* check if boost up or not */
		if(nextStatus > maliDvfsStatus.currentStep) boostup = 1;

		update_time_in_state(curStatus);
		/* change mali dvfs status */
		if(!change_mali_dvfs_status(nextStatus,boostup))
		{
			MALI_DEBUG_PRINT(1, ("error on change_mali_dvfs_status \n"));
			return MALI_FALSE;
		}
	}
	return MALI_TRUE;
}


int mali_dvfs_is_running(void)
{
	return bMaliDvfsRun;
}

static int verify_valid (int module_clk, int dvfs_clock) {
    int i, found = 0;
    for (i = 0; (gpu_freq_table[i] != GPU_FREQ_END_OF_TABLE); i++) {
	if (gpu_freq_table[i] == module_clk) {
	    found = 1;
            return module_clk;
	}
    }
// new freq is not valid
    return dvfs_clock;
}


static void mali_dvfs_work_handler(struct work_struct *w)
{
	bMaliDvfsRun=1;
	needs_update=0;

        if (step0_clk != mali_dvfs[0].clock) {
	    step0_clk = verify_valid(step0_clk, mali_dvfs[0].clock);
	    if (step0_clk != mali_dvfs[0].clock) {
                MALI_PRINT(("::: step0_clk change to %d Mhz\n", step0_clk));
                mali_dvfs[0].clock = step0_clk;
		needs_update=1;
	    }
        }
        if (step1_clk != mali_dvfs[1].clock) {
	    step1_clk = verify_valid(step1_clk, mali_dvfs[1].clock);
	    if (step1_clk != mali_dvfs[1].clock) {
                MALI_PRINT(("::: step1_clk change to %d Mhz\n", step1_clk));
                mali_dvfs[1].clock = step1_clk;
		needs_update=1;
	    }
        }
        if (step0_up != mali_dvfs[0].upthreshold) {
                MALI_PRINT(("::: step0_up change to %d %\n", step0_up));
                mali_dvfs[0].upthreshold = step0_up;
        }
        if (step1_down != mali_dvfs[1].downthreshold) {
                MALI_PRINT((":::step1_down change to %d %\n", step1_down));
                mali_dvfs[1].downthreshold = step1_down;
        }
        if (step2_clk != mali_dvfs[2].clock) {
	    step2_clk = verify_valid(step2_clk, mali_dvfs[2].clock);
	    if (step2_clk != mali_dvfs[2].clock) {
                MALI_PRINT(("::: step2_clk change to %d Mhz\n", step2_clk));
                mali_dvfs[2].clock = step2_clk;
		needs_update=1;
	    }
        }
        if (step1_up != mali_dvfs[1].upthreshold) {
                MALI_PRINT((":::step1_up change to %d %\n", step1_up));
                mali_dvfs[1].upthreshold = step1_up;
        }
        if (step2_down != mali_dvfs[2].downthreshold) {
                MALI_PRINT((":::step2_down change to %d %\n", step2_down));
                mali_dvfs[2].downthreshold = step2_down;
        }
        if (step3_clk != mali_dvfs[3].clock) {
	    step3_clk = verify_valid(step3_clk, mali_dvfs[3].clock);
	    if (step3_clk != mali_dvfs[3].clock) {
                MALI_PRINT(("::: step3_clk change to %d Mhz\n", step3_clk));
                mali_dvfs[3].clock = step3_clk;
		needs_update=1;
	    }
        }
        if (step2_up != mali_dvfs[2].upthreshold) {
                MALI_PRINT((":::step2_up change to %d %\n", step2_up));
                mali_dvfs[2].upthreshold = step2_up;
        }
        if (step3_down != mali_dvfs[3].downthreshold) {
                MALI_PRINT((":::step3_down change to %d %\n", step3_down));
                mali_dvfs[3].downthreshold = step3_down;
        }
        if (step4_clk != mali_dvfs[4].clock) {
	    step4_clk = verify_valid(step4_clk, mali_dvfs[4].clock);
	    if (step4_clk != mali_dvfs[4].clock) {
                MALI_PRINT(("::: step4_clk change to %d Mhz\n", step4_clk));
                mali_dvfs[4].clock = step4_clk;
		needs_update=1;
	    }
        }
        if (step3_up != mali_dvfs[3].upthreshold) {
                MALI_PRINT((":::step3_up change to %d %\n", step3_up));
                mali_dvfs[3].upthreshold = step3_up;
        }
        if (step4_down != mali_dvfs[4].downthreshold) {
                MALI_PRINT((":::step4_down change to %d %\n", step4_down));
                mali_dvfs[4].downthreshold = step4_down;
        }

#ifdef DEBUG
        mali_dvfs[0].vol = step0_vol;
        mali_dvfs[1].vol = step1_vol;
        mali_dvfs[2].vol = step2_vol;
        mali_dvfs[3].vol = step3_vol;
        mali_dvfs[4].vol = step4_vol;
#endif
	if(needs_update == 1) {
	mali_dvfs_table_update();
	}

	MALI_DEBUG_PRINT(3, ("=== mali_dvfs_work_handler\n"));

	if(!mali_dvfs_status(mali_dvfs_utilization))
	MALI_DEBUG_PRINT(1, ( "error on mali dvfs status in mali_dvfs_work_handler"));

	bMaliDvfsRun=0;
}


mali_bool init_mali_dvfs_status(void)
{
	/*
	* default status
	* add here with the right function to get initilization value.
	*/

	if (!mali_dvfs_wq)
	{
		mali_dvfs_wq = create_singlethread_workqueue("mali_dvfs");
	}

	/* add a error handling here */
	maliDvfsStatus.currentStep = MALI_DVFS_DEFAULT_STEP;

#ifdef EXYNOS4_ASV_ENABLED
        mali_dvfs_table_update();
        change_mali_dvfs_status(1, 0);
#endif

	return MALI_TRUE;
}

void deinit_mali_dvfs_status(void)
{
	if (mali_dvfs_wq)
	{
		destroy_workqueue(mali_dvfs_wq);
		mali_dvfs_wq = NULL;
	}
}

mali_bool mali_dvfs_handler(unsigned int utilization)
{
	mali_dvfs_utilization = utilization;
	queue_work_on(0, mali_dvfs_wq, &mali_dvfs_work);

	return MALI_TRUE;
}

static mali_bool init_mali_clock(void)
{
	mali_bool ret = MALI_TRUE;
	gpu_power_state = 1;
	bPoweroff = 1;

	if (mali_clock != 0)
		return ret; /* already initialized */

	mali_dvfs_lock = _mali_osk_lock_init(_MALI_OSK_LOCKFLAG_NONINTERRUPTABLE
			| _MALI_OSK_LOCKFLAG_ONELOCK, 0, 0);
	if (mali_dvfs_lock == NULL)
		return _MALI_OSK_ERR_FAULT;

	if (!mali_clk_get())
	{
		MALI_PRINT(("Error: Failed to get Mali clock\n"));
		goto err_clk;
	}

	mali_clk_set_rate((unsigned int)mali_gpu_clk, GPU_MHZ);

	MALI_PRINT(("init_mali_clock mali_clock %x\n", mali_clock));

#ifdef CONFIG_REGULATOR
	g3d_regulator = regulator_get(NULL, "vdd_g3d");

	if (IS_ERR(g3d_regulator))
	{
		MALI_PRINT(("MALI Error : failed to get vdd_g3d\n"));
		ret = MALI_FALSE;
		goto err_regulator;
	}

	regulator_enable(g3d_regulator);
	mali_regulator_set_voltage(mali_gpu_vol, mali_gpu_vol);
#endif

#if defined(CONFIG_MALI400_PROFILING)
	_mali_osk_profiling_add_event(MALI_PROFILING_EVENT_TYPE_SINGLE|
			MALI_PROFILING_EVENT_CHANNEL_GPU|
			MALI_PROFILING_EVENT_REASON_SINGLE_GPU_FREQ_VOLT_CHANGE,
			mali_gpu_clk, mali_gpu_vol/1000, 0, 0, 0);
#endif

	mali_clk_put(MALI_FALSE);

	return MALI_TRUE;

#ifdef CONFIG_REGULATOR
err_regulator:
	regulator_put(g3d_regulator);
#endif
err_clk:
	mali_clk_put(MALI_TRUE);

	return ret;
}

static mali_bool deinit_mali_clock(void)
{
	if (mali_clock == 0)
		return MALI_TRUE;

#ifdef CONFIG_REGULATOR
	if (g3d_regulator)
	{
		regulator_put(g3d_regulator);
		g3d_regulator = NULL;
	}
#endif

	mali_clk_put(MALI_TRUE);

	return MALI_TRUE;
}


static _mali_osk_errcode_t enable_mali_clocks(void)
{
	int err;

	if (atomic_read(&clk_active) == 0) {
		err = clk_enable(mali_clock);
		MALI_DEBUG_PRINT(3,("enable_mali_clocks mali_clock %p error %d \n", mali_clock, err));
		atomic_set(&clk_active, 1);
		if (err >= 0)
			atomic_set(&clk_active, 1);
		gpu_power_state = 1;
	}

	/* set clock rate */
#ifdef CONFIG_MALI_DVFS
	if (get_mali_dvfs_control_status() != 0 || mali_gpu_clk >= mali_runtime_resume.clk) {
		mali_clk_set_rate(mali_gpu_clk, GPU_MHZ);
	} else {
#ifdef CONFIG_REGULATOR
		mali_regulator_set_voltage(mali_runtime_resume.vol, mali_runtime_resume.vol);
#endif
		mali_clk_set_rate(mali_runtime_resume.clk, GPU_MHZ);
		set_mali_dvfs_current_step(mali_runtime_resume.step);
	}
#else
	mali_clk_set_rate((unsigned int)mali_gpu_clk, GPU_MHZ);
	maliDvfsStatus.currentStep = MALI_DVFS_DEFAULT_STEP;
#endif

	MALI_SUCCESS;
}

static _mali_osk_errcode_t disable_mali_clocks(void)
{
	if (atomic_read(&clk_active) == 1) {
		clk_disable(mali_clock);
		atomic_set(&clk_active, 0);
		gpu_power_state = 0;
	}
	MALI_DEBUG_PRINT(3, ("disable_mali_clocks mali_clock %p \n", mali_clock));

	MALI_SUCCESS;
}

/* Some defines changed names in later Odroid-A kernels. Make sure it works for both. */
#ifndef S5P_G3D_CONFIGURATION
#define S5P_G3D_CONFIGURATION S5P_PMU_G3D_CONF
#endif
#ifndef S5P_G3D_STATUS
#define S5P_G3D_STATUS S5P_PMU_G3D_CONF + 0x4
#endif

_mali_osk_errcode_t g3d_power_domain_control(int bpower_on)
{
	if (bpower_on)
	{
		void __iomem *status;
		u32 timeout;
		__raw_writel(S5P_INT_LOCAL_PWR_EN, S5P_G3D_CONFIGURATION);
		status = S5P_G3D_STATUS;

		timeout = 10;
		while ((__raw_readl(status) & S5P_INT_LOCAL_PWR_EN)
			!= S5P_INT_LOCAL_PWR_EN) {
			if (timeout == 0) {
				MALI_PRINTF(("Power domain  enable failed.\n"));
				return -ETIMEDOUT;
			}
			timeout--;
			_mali_osk_time_ubusydelay(100);
		}
	}
	else
	{
		void __iomem *status;
		u32 timeout;
		__raw_writel(0, S5P_G3D_CONFIGURATION);

		status = S5P_G3D_STATUS;
		/* Wait max 1ms */
		timeout = 10;
		while (__raw_readl(status) & S5P_INT_LOCAL_PWR_EN)
		{
			if (timeout == 0) {
				MALI_PRINTF(("Power domain  disable failed.\n" ));
				return -ETIMEDOUT;
			}
			timeout--;
			_mali_osk_time_ubusydelay(100);
		}
	}

	MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_init(struct device *dev)
{
	MALI_CHECK(init_mali_clock(), _MALI_OSK_ERR_FAULT);

	atomic_set(&clk_active, 0);

#ifdef CONFIG_MALI_DVFS
	/* Create sysfs for time-in-state */
	if (device_create_file(dev, &dev_attr_time_in_state)) {
		dev_err(dev, "Couldn't create sysfs file [time_in_state]\n");
	}

	if (!clk_register_map) clk_register_map = _mali_osk_mem_mapioregion( CLK_DIV_STAT_G3D, 0x20, CLK_DESC );
	if (!init_mali_dvfs_status())
		MALI_DEBUG_PRINT(1, ("mali_platform_init failed\n"));
#endif

	mali_platform_power_mode_change(dev, MALI_POWER_MODE_ON);

	MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_deinit(struct device *dev)
{

	mali_platform_power_mode_change(dev, MALI_POWER_MODE_DEEP_SLEEP);
	deinit_mali_clock();

#ifdef CONFIG_MALI_DVFS
	deinit_mali_dvfs_status();
	if (clk_register_map )
	{
		_mali_osk_mem_unmapioregion(CLK_DIV_STAT_G3D, 0x20, clk_register_map);
		clk_register_map = NULL;
	}
#endif

	MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_power_mode_change(struct device *dev, mali_power_mode power_mode)
{
	switch (power_mode)
	{
		case MALI_POWER_MODE_ON:
			MALI_DEBUG_PRINT(3, ("Mali platform: Got MALI_POWER_MODE_ON event, %s\n",
								 bPoweroff ? "powering on" : "already on"));
			if (bPoweroff == 1)
			{
#if !defined(CONFIG_PM_RUNTIME)
				g3d_power_domain_control(1);
#endif
				MALI_DEBUG_PRINT(4, ("enable clock \n"));
				enable_mali_clocks();
#if defined(CONFIG_MALI400_PROFILING)
				_mali_osk_profiling_add_event(MALI_PROFILING_EVENT_TYPE_SINGLE |
						MALI_PROFILING_EVENT_CHANNEL_GPU |
						MALI_PROFILING_EVENT_REASON_SINGLE_GPU_FREQ_VOLT_CHANGE,
						mali_gpu_clk, mali_gpu_vol/1000, 0, 0, 0);

#endif
				bPoweroff=0;
			}
			break;
		case MALI_POWER_MODE_LIGHT_SLEEP:
		case MALI_POWER_MODE_DEEP_SLEEP:
			MALI_DEBUG_PRINT(3, ("Mali platform: Got %s event, %s\n", power_mode ==
						MALI_POWER_MODE_LIGHT_SLEEP ?  "MALI_POWER_MODE_LIGHT_SLEEP" :
						"MALI_POWER_MODE_DEEP_SLEEP", bPoweroff ? "already off" : "powering off"));
			if (bPoweroff == 0)
			{
				disable_mali_clocks();
#if defined(CONFIG_MALI400_PROFILING)
				_mali_osk_profiling_add_event(MALI_PROFILING_EVENT_TYPE_SINGLE |
						MALI_PROFILING_EVENT_CHANNEL_GPU |
						MALI_PROFILING_EVENT_REASON_SINGLE_GPU_FREQ_VOLT_CHANGE,
						0, 0, 0, 0, 0);
#endif

#if !defined(CONFIG_PM_RUNTIME)
				g3d_power_domain_control(0);
#endif
				bPoweroff=1;
			}

			break;
	}
	MALI_SUCCESS;
}

void mali_gpu_utilization_handler(struct mali_gpu_utilization_data *data)
{
	if (bPoweroff==0)
	{
#ifdef CONFIG_MALI_DVFS
		if(!mali_dvfs_handler(data->utilization_gpu))
			MALI_DEBUG_PRINT(1, ("error on mali dvfs status in utilization\n"));
#endif
	}
}

#ifdef CONFIG_CPU_EXYNOS4210
int mali_dvfs_bottom_lock_push()
{
	int prev_status = _mali_osk_atomic_read(&bottomlock_status);

	if (prev_status < 0) {
		MALI_PRINT(("gpu bottom lock status is not valid for push\n"));
		return -1;
	}
	if (prev_status == 0) {
                mali_regulator_set_voltage(mali_dvfs[1].vol, mali_dvfs[1].vol);
                mali_clk_set_rate(mali_dvfs[1].clock, mali_dvfs[1].freq);
                set_mali_dvfs_current_step(1);
    }
	return _mali_osk_atomic_inc_return(&bottomlock_status);
}
#else
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
#endif

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

int mali_dvfs_get_vol(int step)
{
	step = step % MAX_MALI_DVFS_STEPS;
	MALI_DEBUG_ASSERT(step<MAX_MALI_DVFS_STEPS);
	return mali_dvfs[step].vol;
}

#if MALI_VOLTAGE_LOCK
int mali_voltage_lock_push(int lock_vol)
{
	int prev_status = _mali_osk_atomic_read(&voltage_lock_status);

	if (prev_status < 0) {
		MALI_PRINT(("gpu voltage lock status is not valid for push\n"));
		return -1;
	}
	if (prev_status == 0) {
		mali_lock_vol = lock_vol;
		if (mali_gpu_vol < mali_lock_vol)
			mali_regulator_set_voltage(mali_lock_vol, mali_lock_vol);
	} else {
		MALI_PRINT(("gpu voltage lock status is already pushed, current lock voltage : %d\n", mali_lock_vol));
		return -1;
	}

	return _mali_osk_atomic_inc_return(&voltage_lock_status);
}

int mali_voltage_lock_pop(void)
{
	if (_mali_osk_atomic_read(&voltage_lock_status) <= 0) {
		MALI_PRINT(("gpu voltage lock status is not valid for pop\n"));
		return -1;
	}
	return _mali_osk_atomic_dec_return(&voltage_lock_status);
}

int mali_voltage_lock_init(void)
{
	mali_vol_lock_flag = MALI_TRUE;

	MALI_SUCCESS;
}

int mali_vol_get_from_table(int vol)
{
	int i;
	for (i = 0; i < MALI_DVFS_STEPS; i++) {
		if (mali_dvfs[i].vol >= vol)
			return mali_dvfs[i].vol;
	}
	MALI_PRINT(("Failed to get voltage from mali_dvfs table, maximum voltage is %d uV\n", mali_dvfs[MALI_DVFS_STEPS-1].vol));
	return 0;
}
#endif

u64 mali_dvfs_time[MALI_DVFS_STEPS];
static void update_time_in_state(int level)
{
	u64 current_time;
	static u64 prev_time=0;

	if (prev_time ==0)
		prev_time=get_jiffies_64();

	current_time = get_jiffies_64();
	mali_dvfs_time[level] += current_time-prev_time;
	prev_time = current_time;
}

ssize_t show_time_in_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i;

	update_time_in_state(maliDvfsStatus.currentStep);

	for (i = 0; i < MALI_DVFS_STEPS; i++) {
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "%d %llu\n",
				mali_dvfs[i].clock,
				mali_dvfs_time[i]);
	}

	if (ret < PAGE_SIZE - 1) {
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n");
	} else {
		buf[PAGE_SIZE-2] = '\n';
		buf[PAGE_SIZE-1] = '\0';
		ret = PAGE_SIZE-1;
	}

	return ret;
}

ssize_t set_time_in_state(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i;

	for (i = 0; i < MALI_DVFS_STEPS; i++) {
		mali_dvfs_time[i] = 0;
	}

	return count;
}

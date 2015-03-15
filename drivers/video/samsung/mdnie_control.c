/*
 * mdnie_control.c - mDNIe register sequence intercept and control
 *
 * @Author        : Andrei F. <https://github.com/AndreiLux>
 * @Date        : February 2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>

#include "mdnie.h"
#include "s3cfb.h"
#include "s3cfb_mdnie.h"

#include <mach/media_monitor.h>

#define REFRESH_DELAY                HZ / 2
struct delayed_work mdnie_refresh_work;

bool reg_hook = 0;
bool sequence_hook = 0;
struct mdnie_info *mdnie = NULL;

extern struct mdnie_info *g_mdnie;
extern void set_mdnie_value(struct mdnie_info *mdnie, u8 force);

/* Defined as negative deltas */
static int br_reduction = 75;
static int br_takeover_point = 30;
static int br_brightness_delta = 20;

static int br_component_reduction = 0;

enum mdnie_registers {
        EFFECT_MASTER        = 0x08,        /*Dither8 UC4 ABC2 CP1 | CC8 MCM4 SCR2 SCC1 | CS8 DE4 DNR2 HDR1*/
        FA_MASTER        = 0x30,        /*FA cs1 | de8 dnr4 hdr2 fa1*/
        FA_DNR_WEIGHT        = 0x39,        /*FA dnrWeight*/

        DNR_1                = 0x80,        /*DNR dirTh*/
        DNR_2                = 0x81,        /*DNR dirnumTh decon7Th*/
        DNR_3                = 0x82,        /*DNR decon5Th maskTh*/
        DNR_4                = 0x83,        /*DNR blTh*/

        DE_EGTH                = 0x90,        /*DE egth*/
        DE_PE                = 0x92, /*DE pe*/
        DE_PF                = 0x93,        /*DE pf*/
        DE_PB                = 0x94,        /*DE pb*/
        DE_NE                = 0x95,        /*DE ne*/
        DE_NF                = 0x96,        /*DE nf*/
        DE_NB                = 0x97,        /*DE nb*/
        DE_MAX_RATIO        = 0x98,        /*DE max ratio*/
        DE_MIN_RATIO        = 0x99,        /*DE min ratio*/

        CS_HG_RY        = 0xb0,        /*CS hg ry*/
        CS_HG_GC        = 0xb1,        /*CS hg gc*/
        CS_HG_BM        = 0xb2,        /*CS hg bm*/
        CS_WEIGHT_GRTH        = 0xb3,        /*CS weight grayTH*/

        SCR_RR_CR        = 0xe1,        /*SCR RrCr*/
        SCR_RG_CG        = 0xe2,        /*SCR RgCg*/
        SCR_RB_CB        = 0xe3,        /*SCR RbCb*/

        SCR_GR_MR        = 0xe4,        /*SCR GrMr*/
        SCR_GG_MG        = 0xe5,        /*SCR GgMg*/
        SCR_GB_MB        = 0xe6,        /*SCR GbMb*/

        SCR_BR_YR        = 0xe7,        /*SCR BrYr*/
        SCR_BG_YG        = 0xe8,        /*SCR BgYg*/
        SCR_BB_YB        = 0xe9,        /*SCR BbYb*/

        SCR_KR_WR        = 0xea,        /*SCR KrWr*/
        SCR_KG_WG        = 0xeb,        /*SCR KgWg*/
        SCR_KB_WB        = 0xec,        /*SCR KbWb*/

        MCM_TEMPERATURE = 0x01, /*MCM 0x64 10000K 0x28 4000K */
        MCM_9                = 0x09, /*MCM 5cb 1cr W*/
        MCM_B                = 0x0b, /*MCM 4cr 5cr W*/

        UC_Y                = 0xd0,
        UC_CS                = 0xd1,

        CC_CHSEL_STR        = 0x1f,        /*CC chsel strength*/
        CC_0                = 0x20,        /*CC lut r   0*/
        CC_1                = 0x21,        /*CC lut r  16 144*/
        CC_2                = 0x22,        /*CC lut r  32 160*/
        CC_3                = 0x23,        /*CC lut r  48 176*/
        CC_4                = 0x24,        /*CC lut r  64 192*/
        CC_5                = 0x25,        /*CC lut r  80 208*/
        CC_6                = 0x26,        /*CC lut r  96 224*/
        CC_7                = 0x27,        /*CC lut r 112 240*/
        CC_8                = 0x28        /*CC lut r 128 255*/
};

static unsigned short master_sequence[92] = { 
        0x0000, 0x0000,  0x0008, 0x0000,  0x0030, 0x0000,  0x0090, 0x0080,
        0x0092, 0x0030,  0x0093, 0x0060,  0x0094, 0x0060,  0x0095, 0x0030,
        0x0096, 0x0060,  0x0097, 0x0060,  0x0098, 0x1000,  0x0099, 0x0100,
        0x00b0, 0x1010,  0x00b1, 0x1010,  0x00b2, 0x1010,  0x00b3, 0x1804,

        0x00e1, 0xff00,  0x00e2, 0x00ff,  0x00e3, 0x00ff,  0x00e4, 0x00ff,
        0x00e5, 0xff00,  0x00e6, 0x00ff,  0x00e7, 0x00ff,  0x00e8, 0x00ff,
        0x00e9, 0xff00,  0x00ea, 0x00ff,  0x00eb, 0x00ff,  0x00ec, 0x00ff,
        0x0000, 0x0001,  0x0001, 0x0041,  0x0009, 0xa08b,  0x000b, 0x7a7a,

        0x00d0, 0x01c0,  0x00d1, 0x01ff,  0x001f, 0x0080,  0x0020, 0x0000,
        0x0021, 0x1090,  0x0022, 0x20a0,  0x0023, 0x30b0,  0x0024, 0x40c0,
        0x0025, 0x50d0,  0x0026, 0x60e0,  0x0027, 0x70f0,  0x0028, 0x80ff,
        0x00ff, 0x0000,  0xffff, 0x0000,
};

static ssize_t show_mdnie_property(struct device *dev,
                                    struct device_attribute *attr, char *buf);

static ssize_t store_mdnie_property(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count);

#define _effect(name_, reg_, mask_, shift_, regval_)\
{                                                                         \
        .attribute = {                                                        \
                        .attr = {                                        \
                                  .name = name_,                        \
                                  .mode = S_IRUGO | S_IWUSR | S_IWGRP,        \
                                },                                        \
                        .show = show_mdnie_property,                        \
                        .store = store_mdnie_property,                        \
                     },                                                        \
        .reg         = reg_ ,                                                \
        .mask         = mask_ ,                                                \
        .shift         = shift_ ,                                                \
        .delta         = 1 ,                                                        \
        .value         = 0 ,                                                        \
        .regval = regval_                                                \
}

#if defined(CONFIG_FB_S5P_S6EVR02)
#define _model(a, b) b
#else 
#if defined(CONFIG_FB_S5P_S6E8AA0)
#define _model(a, b) a
#endif
#endif

struct mdnie_effect {
        const struct device_attribute        attribute;
        u8                                reg;
        u16                                mask;
        u8                                shift;
        bool                                delta;
        int                                value;
        u16                                regval;
};

struct mdnie_effect mdnie_controls[] = {
        /* Master switches */
        _effect("s_dithering"                , EFFECT_MASTER        , (1 << 11), 11        , _model( 0        , 0        )),
        _effect("s_UC"                        , EFFECT_MASTER        , (1 << 10), 10        , _model( 0        , 0        )),
        _effect("s_adaptive_brightness_control"        , EFFECT_MASTER        , (1 << 9), 9        , _model( 0, 0        )),
        _effect("s_CP"                        , EFFECT_MASTER        , (1 << 8), 8        , _model( 0        , 0        )),

        _effect("s_gamma_curve"                , EFFECT_MASTER        , (1 << 7), 7        , _model( 1        , 1        )),
        _effect("s_MCM"                        , EFFECT_MASTER        , (1 << 6), 6        , _model( 0        , 0        )),
        _effect("s_channel_filters"        , EFFECT_MASTER        , (1 << 5), 5        , _model( 1        , 1        )),
        _effect("s_SCC"                        , EFFECT_MASTER        , (1 << 4), 4        , _model( 0        , 0        )),

        _effect("s_chroma_saturation"        , EFFECT_MASTER        , (1 << 3), 3        , _model( 1        , 1        )),
        _effect("s_edge_enhancement"        , EFFECT_MASTER        , (1 << 2), 2        , _model( 0        , 0        )),
        _effect("s_digital_noise_reduction", EFFECT_MASTER, (1 << 1), 1        , _model( 0        , 0        )),
        _effect("s_high_dynamic_range"        , EFFECT_MASTER        , (1 << 0), 0        , _model( 0        , 0        )),

        /* Factory setting overrides */
        _effect("s_factory_chroma_saturation", FA_MASTER , (1 << 4), 4        , _model( 0        , 0        )),
        _effect("s_factory_edge_enhancement", FA_MASTER        , (1 << 3), 3        , _model( 0        , 0        )),
        _effect("s_factory_digital_noise_reduction", FA_MASTER , (1 << 2), 2, _model( 0        , 0        )),
        _effect("s_factory_high_dynamic_range", FA_MASTER, (1 << 1), 1        , _model( 0        , 0        )),
        _effect("s_FA_fa"                , FA_MASTER         , (1 << 0), 0        , _model( 0        , 0        )),

        _effect("FA_dnr_weight"                , FA_DNR_WEIGHT        , (0xff)  , 0        , _model( 0        , 0        )),
        
        /* Digital noise reduction */
        _effect("dnr_dirTh"                , DNR_1                , (0xffff), 0        , 4095        ),
        _effect("dnr_dirnumTh"                , DNR_2                , (0xff00), 8        , 25        ),
        _effect("dnr_decon7Th"                , DNR_2                , (0x00ff), 0        , 255        ),
        _effect("dnr_decon5Th"                , DNR_3                , (0xff00), 8        , 255        ),
        _effect("dnr_maskTh"                , DNR_3                , (0x00ff), 0        , 22        ),
        _effect("dnr_blTh"                , DNR_4                , (0x00ff), 0        , 0        ),

        /* Ditigal edge enhancement */
        _effect("de_egth"                , DE_EGTH        , (0x00ff), 0        , 128        ),

        _effect("de_positive_e"                , DE_PE                , (0x00ff), 0        , 48        ),
        _effect("de_positive_f"                , DE_PF                , (0x00ff), 0        , 96        ),
        _effect("de_positive_b"                , DE_PB                , (0x00ff), 0        , 96        ),

        _effect("de_negative_e"                , DE_NE                , (0x00ff), 0        , 48        ),
        _effect("de_negative_f"                , DE_NF                , (0x00ff), 0        , 96        ),
        _effect("de_negative_b"                , DE_NB                , (0x00ff), 0        , 96        ),

        _effect("de_min_ratio"                , DE_MIN_RATIO        , (0xffff), 0        , 256        ),
        _effect("de_max_ratio"                , DE_MAX_RATIO        , (0xffff), 0        , 4096        ),

        /* Chroma saturation */
        _effect("cs_weight"                , CS_WEIGHT_GRTH, (0xff00), 8        , 24        ),
        _effect("cs_gray_threshold"        , CS_WEIGHT_GRTH, (0x00ff), 0        , 4        ),

        _effect("cs_red"                , CS_HG_RY        , (0xff00), 8        , _model( 6        , 6        )),
        _effect("cs_green"                , CS_HG_GC        , (0xff00), 8        , _model( 7        , 7        )),
        _effect("cs_blue"                , CS_HG_BM        , (0xff00), 8        , _model( 6        , 7        )),

        _effect("cs_yellow"                , CS_HG_RY        , (0x00ff), 0        , _model( 9        , 6        )),
        _effect("cs_cyan"                , CS_HG_GC        , (0x00ff), 0        , _model( 4        , 2        )),
        _effect("cs_magenta"                , CS_HG_BM        , (0x00ff), 0        , _model( 8        , 4        )),

        /* Colour channel pass-through filters */
        _effect("scr_red_red"                , SCR_RR_CR        , (0xff00), 8        , _model( 237        , 243        )),
        _effect("scr_red_green"                , SCR_RG_CG        , (0xff00), 8        , _model( 15        , 28        )),
        _effect("scr_red_blue"                , SCR_RB_CB        , (0xff00), 8        , _model( 0        , 0        )),

        _effect("scr_cyan_red"                , SCR_RR_CR        , (0x00ff), 0        , _model( 68        , 40        )),
        _effect("scr_cyan_green"        , SCR_RG_CG        , (0x00ff), 0        , _model( 245        , 241        )),
        _effect("scr_cyan_blue"                , SCR_RB_CB        , (0x00ff), 0        , _model( 255        , 255        )),
        
        _effect("scr_green_red"                , SCR_GR_MR        , (0xff00), 8        , _model( 97        , 80        )),
        _effect("scr_green_green"        , SCR_GG_MG        , (0xff00), 8        , _model( 230        , 230        )),
        _effect("scr_green_blue"        , SCR_GB_MB        , (0xff00), 8        , _model( 18        , 16        )),

        _effect("scr_magenta_red"        , SCR_GR_MR        , (0x00ff), 0        , _model( 251        , 255        )),
        _effect("scr_magenta_green"        , SCR_GG_MG        , (0x00ff), 0        , _model( 36        , 5        )),
        _effect("scr_magenta_blue"        , SCR_GB_MB        , (0x00ff), 0        , _model( 255        , 255        )),
        
        _effect("scr_blue_red"                , SCR_BR_YR        , (0xff00), 8        , _model( 0        , 15        )),
        _effect("scr_blue_green"        , SCR_BG_YG        , (0xff00), 8        , _model( 43        , 0        )),
        _effect("scr_blue_blue"                , SCR_BB_YB        , (0xff00), 8        , _model( 255        , 255        )),

        _effect("scr_yellow_red"        , SCR_BR_YR        , (0x00ff), 0        , _model( 255        , 255        )),
        _effect("scr_yellow_green"        , SCR_BG_YG        , (0x00ff), 0        , _model( 239        , 241        )),
        _effect("scr_yellow_blue"        , SCR_BB_YB        , (0x00ff), 0        , _model( 27        , 26        )),

        _effect("scr_black_red"                , SCR_KR_WR        , (0xff00), 8        , _model( 0        , 0        )),
        _effect("scr_black_green"        , SCR_KG_WG        , (0xff00), 8        , _model( 0        , 0        )),
        _effect("scr_black_blue"        , SCR_KB_WB        , (0xff00), 8        , _model( 0        , 0        )),

        _effect("scr_white_red"                , SCR_KR_WR        , (0x00ff), 0        , _model( 255        , 255        )),
        _effect("scr_white_green"        , SCR_KG_WG        , (0x00ff), 0        , _model( 240        , 240        )),
        _effect("scr_white_blue"        , SCR_KB_WB        , (0x00ff), 0        , _model( 240        , 244        )),

        /* MCM */
        _effect("mcm_temperature"        , MCM_TEMPERATURE, (0x00ff), 0        , 65        ),

        _effect("mcm_9_left"                , MCM_9                , (0xff00), 8        , 160        ),
        _effect("mcm_9_right"                , MCM_9                , (0x00ff), 0        , 139        ),

        _effect("mcm_B_left"                , MCM_9                , (0xff00), 8        , 122        ),
        _effect("mcm_B_right"                , MCM_9                , (0x00ff), 0        , 122        ),

        /* UC */
        _effect("UC_y"                        , UC_Y                , (0xffff), 0        , 448        ),
        _effect("UC_chroma_saturation"        , UC_CS                , (0xffff), 0        , 511        ),

        /* Greyscale gamma curve */
        _effect("cc_channel_selection"        , CC_CHSEL_STR        , (0xff00), 8        , 0        ),
        _effect("cc_channel_strength"        , CC_CHSEL_STR        , (0x00ff), 0        , 128        ),
        
        _effect("cc_0"                        , CC_0                , (0x00ff), 0        , _model( 0        , 4        )),
        _effect("cc_16"                        , CC_1                , (0xff00), 8        , _model( 16        , 17        )),
        _effect("cc_32"                        , CC_2                , (0xff00), 8        , _model( 32        , 33        )),
        _effect("cc_48"                        , CC_3                , (0xff00), 8        , _model( 47        , 49        )),
        _effect("cc_64"                        , CC_4                , (0xff00), 8        , _model( 64        , 65        )),
        _effect("cc_80"                        , CC_5                , (0xff00), 8        , _model( 77        , 83        )),
        _effect("cc_96"                        , CC_6                , (0xff00), 8        , _model( 94        , 100        )),
        _effect("cc_112"                , CC_7                , (0xff00), 8        , _model( 112        , 115        )),
        _effect("cc_128"                , CC_8                , (0xff00), 8        , _model( 126        , 133        )),
        _effect("cc_144"                , CC_1                , (0x00ff), 0        , _model( 143        , 148        )),
        _effect("cc_160"                , CC_2                , (0x00ff), 0        , _model( 159        , 163        )),
        _effect("cc_176"                , CC_3                , (0x00ff), 0        , _model( 173        , 180        )),
        _effect("cc_192"                , CC_4                , (0x00ff), 0        , _model( 190        , 193        )),
        _effect("cc_208"                , CC_5                , (0x00ff), 0        , _model( 206        , 212        )),
        _effect("cc_224"                , CC_6                , (0x00ff), 0        , _model( 225        , 224        )),
        _effect("cc_240"                , CC_7                , (0x00ff), 0        , _model( 240        , 240        )),
        _effect("cc_255"                , CC_8                , (0x00ff), 0        , _model( 255        , 255        )),
};

static int is_switch(unsigned int reg)
{
        switch(reg) {
                case EFFECT_MASTER:
                case FA_MASTER:
                        return true;
                default:
                        return false;
        }
}

static int effect_switch_hook(struct mdnie_effect *effect, unsigned short regval)
{
        /* Multi-scenario effect switches; DE, HDR, DNR */
        if(effect->reg == EFFECT_MASTER)
                switch(effect->shift) {
                        case 0 ... 1:
                                return effect->value && mhs_get_status(MHS_DECODING);
                        case 2:
                                return ( ((effect->value & 2) &&  mhs_get_status(MHS_DECODING)) || 
                                             ((effect->value & 1) && !mhs_get_status(MHS_DECODING)) );
                        default: break;
                }

        return effect->value ? !regval : regval;
}

static int secondary_hook(struct mdnie_effect *effect, int val)
{
//        int channel;

        if (likely(effect->delta))
                val += effect->value;
        else 
                val = effect->value;

        if (!(br_reduction))
                return val;

/*        Correct way of decreasing luminance would be to convert the RGB
 *        coordinates into the HSL space, reducing L and going back to RGB,
 *        however, that's pretty outlandish in the current design of things where
 *        channel values are modified independently on a per-register basis.
 *
 *        The effect modifiers need to be tied together to do that, otherwise
 *        you need to do an large amount of ugly to read out the other two
 *        channels outside of the current context (SCR effect).
 *
 *        Luma is L = 0.299R + 0.587G + 0.114B, but you can't use those
 *        weights to independently to reduce luminance because you're
 *        discarding H and S and it'll give you an ugly blue-red-ish hue.
 *
 *        For now, we simply reduce the components of RGB independently,
 *        it reduces brightness but it is no longer colour accurate.
 *        For all practical purposes though, it is good enough for the intent
 *        of this feature.
 *
 *        channel = (effect->reg - SCR_RR_CR) % CI_MAX;
 */

        if (effect->reg >= SCR_RR_CR && effect->reg <= SCR_KB_WB)
                val -= br_component_reduction;

        return val;
}

unsigned short mdnie_reg_hook(unsigned short reg, unsigned short value)
{
        struct mdnie_effect *effect = (struct mdnie_effect*)&mdnie_controls;
        int i;
        int tmp, original;
        unsigned short regval;

        original = value;

//        if(unlikely(!sequence_hook && !reg_hook || mdnie->negative == NEGATIVE_ON))
//                return value;

        for(i = 0; i < ARRAY_SIZE(mdnie_controls); i++) {
            if(unlikely(effect->reg == reg)) {
                if(likely(sequence_hook)) {
                        tmp = regval = effect->regval;
                } else {
                        tmp = regval = (value & effect->mask) >> effect->shift;
                }

                if(likely(reg_hook)) {
                        if (is_switch(reg))
                                tmp = effect_switch_hook(effect, regval);
                        else
                                tmp = secondary_hook(effect, tmp);

                        if(tmp > (effect->mask >> effect->shift))
                                tmp = (effect->mask >> effect->shift);

                        if(tmp < 0)
                                tmp = 0;

                        regval = (unsigned short)tmp;
                }

                value &= ~effect->mask;
                value |= regval << effect->shift;
/*
                printk("mdnie: hook on: 0x%X val: 0x%X -> 0x%X effect:%4d\n",
                        reg, original, value, tmp);
*/
            }
            ++effect;
        }
        
        return value;
}

unsigned short *mdnie_sequence_hook(struct mdnie_info *pmdnie, unsigned short *seq)
{
        if(mdnie == NULL)
                mdnie = pmdnie;

//        if(!sequence_hook || mdnie->negative == NEGATIVE_ON)
//                return seq;

        return (unsigned short *)&master_sequence;
}

//FIXME static // replace with notifier calls in future
void do_mdnie_refresh(struct work_struct *work)
{
        set_mdnie_value(g_mdnie, 1);
}

void mdnie_update_brightness(int brightness, bool is_auto, bool force)
{
        static int prev_brightness = 255;
        static int prev_auto = false;
        int weight, adjusted_brightness;

        if(unlikely(force)) {
                brightness = prev_brightness;
                is_auto = prev_auto;
        }

        adjusted_brightness = brightness + (is_auto ? br_brightness_delta : 0);

        if(unlikely(adjusted_brightness < 1))
                adjusted_brightness = 1;

        if(adjusted_brightness > br_takeover_point) {
                br_component_reduction = 0;

                if(prev_brightness < br_takeover_point)
                        goto do_refresh;

                goto update_previous;
        }

        weight = 1000 - ((adjusted_brightness * 1000) / br_takeover_point);

        br_component_reduction = ((br_reduction) * weight) / 1000;

do_refresh:
        do_mdnie_refresh(NULL);

update_previous:
        prev_brightness = brightness;
        prev_auto = is_auto;

        return;
}

static inline void scheduled_refresh(void)
{
        cancel_delayed_work_sync(&mdnie_refresh_work);
        schedule_delayed_work_on(0, &mdnie_refresh_work, REFRESH_DELAY);
}

static inline void forced_brightness(void)
{ 
        mdnie_update_brightness(0, false, true);
}

/**** Sysfs ****/

static ssize_t show_mdnie_property(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
        struct mdnie_effect *effect = (struct mdnie_effect*)(attr);


        if(is_switch(effect->reg))
                return sprintf(buf, "%d", effect->value);
        
        return sprintf(buf, "%s %d", effect->delta ? "delta" : "override", effect->value);
};

static ssize_t store_mdnie_property(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
        struct mdnie_effect *effect = (struct mdnie_effect*)(attr);
        int val, ret;
        
        if(sscanf(buf, "%d", &val) != 1) {
                char *s = kzalloc(10 * sizeof(char), GFP_KERNEL);
                if(sscanf(buf, "%10c", s) != 1) {
                        ret = -EINVAL;
                } else {
                        printk("inputted: '%s'\n", s);

                        if(strncmp(s, "override", 8)) {
                                effect->delta = 0;
                                ret = count;
                        }
                        
                        if(strncmp(s, "delta", 5)) {
                                effect->delta = 1;
                                ret = count;
                        }

                        ret = -EINVAL;
                }

                kfree(s);
                if(ret != -EINVAL)
                        goto refresh;
                return ret;
        }

        if(is_switch(effect->reg)) {
                effect->value = val;
        } else {
                if(val > (effect->mask >> effect->shift))
                        val = (effect->mask >> effect->shift);

                if(val < -(effect->mask >> effect->shift))
                        val = -(effect->mask >> effect->shift);

                effect->value = val;
        }

refresh:
        scheduled_refresh();

        return count;
};

#define MAIN_CONTROL(_name, _var, _callback) \
static ssize_t show_##_name(struct device *dev,                                        \
                                    struct device_attribute *attr, char *buf)        \
{                                                                                \
        return sprintf(buf, "%d", _var);                                        \
};                                                                                \
static ssize_t store_##_name(struct device *dev,                                \
                                     struct device_attribute *attr,                \
                                     const char *buf, size_t count)                \
{                                                                                \
        int val;                                                                \
                                                                                \
        if(sscanf(buf, "%d", &val) != 1)                                        \
                return -EINVAL;                                                        \
                                                                                \
        _var = val;                                                                \
                                                                                \
        _callback();                                                                \
                                                                                \
        return count;                                                                \
};

MAIN_CONTROL(reg_hook, reg_hook, scheduled_refresh);
MAIN_CONTROL(sequence_intercept, sequence_hook, scheduled_refresh);
MAIN_CONTROL(br_reduction, br_reduction, forced_brightness);
MAIN_CONTROL(br_takeover_point, br_takeover_point, forced_brightness);
MAIN_CONTROL(br_brightness_delta, br_brightness_delta, forced_brightness);

DEVICE_ATTR(hook_intercept, S_IRUGO | S_IWUSR | S_IWGRP, show_reg_hook, store_reg_hook);
DEVICE_ATTR(sequence_intercept, S_IRUGO | S_IWUSR | S_IWGRP, show_sequence_intercept, store_sequence_intercept);
DEVICE_ATTR(brightness_reduction, S_IRUGO | S_IWUSR | S_IWGRP, show_br_reduction, store_br_reduction);
DEVICE_ATTR(brightness_takeover_point, S_IRUGO | S_IWUSR | S_IWGRP, show_br_takeover_point, store_br_takeover_point);
DEVICE_ATTR(brightness_input_delta, S_IRUGO | S_IWUSR | S_IWGRP, show_br_brightness_delta, store_br_brightness_delta);

void init_intercept_control(struct kobject *kobj)
{
        int i, ret;
        struct kobject *subdir;

        subdir = kobject_create_and_add("hook_control", kobj);

        for(i = 0; i < ARRAY_SIZE(mdnie_controls); i++) {
                ret = sysfs_create_file(subdir, &mdnie_controls[i].attribute.attr);
        }

        ret = sysfs_create_file(kobj, &dev_attr_hook_intercept.attr);
        ret = sysfs_create_file(kobj, &dev_attr_sequence_intercept.attr);
        ret = sysfs_create_file(kobj, &dev_attr_brightness_reduction.attr);
        ret = sysfs_create_file(kobj, &dev_attr_brightness_takeover_point.attr);
        ret = sysfs_create_file(kobj, &dev_attr_brightness_input_delta.attr);

        INIT_DELAYED_WORK(&mdnie_refresh_work, do_mdnie_refresh);
}

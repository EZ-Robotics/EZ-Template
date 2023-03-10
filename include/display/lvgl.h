/**
 * @file lvgl.h
 * Include all LVGL related headers
 */

#ifndef LVGL_H
#define LVGL_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************
 * CURRENT VERSION OF LVGL
 ***************************/
#define LVGL_VERSION_MAJOR 8
#define LVGL_VERSION_MINOR 3
#define LVGL_VERSION_PATCH 4
#define LVGL_VERSION_INFO "dev"

/*********************
 *      INCLUDES
 *********************/

#include "display/misc/lv_log.h"
#include "display/misc/lv_timer.h"
#include "display/misc/lv_math.h"
#include "display/misc/lv_mem.h"
#include "display/misc/lv_async.h"
#include "display/misc/lv_anim_timeline.h"
#include "display/misc/lv_printf.h"

#include "display/hal/lv_hal.h"

#include "display/core/lv_obj.h"
#include "display/core/lv_group.h"
#include "display/core/lv_indev.h"
#include "display/core/lv_refr.h"
#include "display/core/lv_disp.h"
#include "display/core/lv_theme.h"

#include "display/font/lv_font.h"
#include "display/font/lv_font_loader.h"
#include "display/font/lv_font_fmt_txt.h"

#include "display/widgets/lv_arc.h"
#include "display/widgets/lv_btn.h"
#include "display/widgets/lv_img.h"
#include "display/widgets/lv_label.h"
#include "display/widgets/lv_line.h"
#include "display/widgets/lv_table.h"
#include "display/widgets/lv_checkbox.h"
#include "display/widgets/lv_bar.h"
#include "display/widgets/lv_slider.h"
#include "display/widgets/lv_btnmatrix.h"
#include "display/widgets/lv_dropdown.h"
#include "display/widgets/lv_roller.h"
#include "display/widgets/lv_textarea.h"
#include "display/widgets/lv_canvas.h"
#include "display/widgets/lv_switch.h"

#include "display/draw/lv_draw.h"

#include "display/lv_api_map.h"

/*-----------------
 * EXTRAS
 *----------------*/
#include "display/extra/lv_extra.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

/** Gives 1 if the x.y.z version is supported in the current version
 * Usage:
 *
 * - Require v6
 * #if LV_VERSION_CHECK(6,0,0)
 *   new_func_in_v6();
 * #endif
 *
 *
 * - Require at least v5.3
 * #if LV_VERSION_CHECK(5,3,0)
 *   new_feature_from_v5_3();
 * #endif
 *
 *
 * - Require v5.3.2 bugfixes
 * #if LV_VERSION_CHECK(5,3,2)
 *   bugfix_in_v5_3_2();
 * #endif
 *
 */
#define LV_VERSION_CHECK(x,y,z) (x == LVGL_VERSION_MAJOR && (y < LVGL_VERSION_MINOR || (y == LVGL_VERSION_MINOR && z <= LVGL_VERSION_PATCH)))

/**
 * Wrapper functions for VERSION macros
 */

static inline int lv_version_major(void)
{
    return LVGL_VERSION_MAJOR;
}

static inline int lv_version_minor(void)
{
    return LVGL_VERSION_MINOR;
}

static inline int lv_version_patch(void)
{
    return LVGL_VERSION_PATCH;
}

static inline const char *lv_version_info(void)
{
    return LVGL_VERSION_INFO;
}

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LVGL_H*/

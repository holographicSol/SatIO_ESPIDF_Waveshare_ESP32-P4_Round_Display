#ifndef ASTRO_CLOCK_H
#define ASTRO_CLOCK_H

#include "lvgl.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void astro_clock_begin(
    lv_obj_t * parent,
    int32_t width,
    int32_t height,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int angle_offset
    );

void astro_clock_end();

void astro_clock_update(void);

void astro_clock_set_target(int target_key);

// Planet target keys
enum {
    ASTRO_TARGET_NONE = 0,
    ASTRO_TARGET_SUN,
    ASTRO_TARGET_MERCURY,
    ASTRO_TARGET_VENUS,
    ASTRO_TARGET_EARTH,
    ASTRO_TARGET_MOON,
    ASTRO_TARGET_MARS,
    ASTRO_TARGET_JUPITER,
    ASTRO_TARGET_SATURN,
    ASTRO_TARGET_URANUS,
    ASTRO_TARGET_NEPTUNE,
    ASTRO_TARGET_MAX
};

extern struct Lv_Astro_Clock lv_astro_clock;

#endif
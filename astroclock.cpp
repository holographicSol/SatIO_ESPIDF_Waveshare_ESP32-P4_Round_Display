#include "astroclock.h"
#include "lvgl.h"
#include <math.h>
#include "./sidereal_helper.h"
#include "./satio.h"
#include "./wtgps300p.h"
#include "./SiderealPlanets.h"
#include "./SiderealObjects.h"
#include <cmath>
#include "esp_heap_caps.h"

// External data - adjust these includes to match your project
extern "C" {
    // Declare your sidereal data structures here or include the proper header
}

LV_FONT_DECLARE(cobalt_alien_17);

// ============================================================================
// CONFIGURATION
// ============================================================================

// Angle offset for positioning
int ANGLE_OFFSET = 90;

// Dimensions for the astro clock display
int32_t ASTRO_WIDTH = 556;
int32_t ASTRO_HEIGHT = 556;

// Center of astro clock display
int32_t SOLAR_CENTER_X = ASTRO_WIDTH / 2;
int32_t SOLAR_CENTER_Y = ASTRO_HEIGHT / 2;

// Max usable orbit radius (leave margin for planet size)
int32_t MAX_ORBIT_RADIUS = (ASTRO_WIDTH < ASTRO_HEIGHT ? ASTRO_WIDTH : ASTRO_HEIGHT) / 2 - 15;

// Orbit step size (8 planetary orbits from Mercury to Neptune)
int32_t ORBIT_STEP = MAX_ORBIT_RADIUS / 8;

// Planet size unit (max planet radius should fit between orbits)
int32_t SIZE_UNIT = ORBIT_STEP / 6;

// Current target selection
int32_t current_target = 0;

// Timer for astro clock updates
lv_timer_t * astro_timer = NULL;

// ============================================================================
// COLORS
// ============================================================================
#define COLOR_ORBIT_BELOW      lv_color_make(  0,   0,  96)
#define COLOR_ORBIT_ABOVE      lv_color_make(  0, 128,   0)
#define COLOR_SUN_BELOW        lv_color_make(128,   0,   0)
#define COLOR_SUN_ABOVE        lv_color_make(128,   0,   0)
#define COLOR_ORBIT_MOON_BELOW lv_color_make(  0,   0,  96)
#define COLOR_ORBIT_MOON_ABOVE lv_color_make(  0, 128,   0)
#define COLOR_TARGET           lv_color_make(255,   0,   0)
#define COLOR_ZODIAC           lv_color_make(  0,   0,  96)

// ============================================================================
// PLANET DATA STRUCTURE
// ============================================================================
typedef struct {
    int orbit_radius;
    int radius;
    int x, y;
    lv_color_t color;
    lv_obj_t * obj;
    lv_obj_t * orbit;
    lv_obj_t * target_box;
    bool tracking;
} Planet;


#define ZODIAC_LINE_WIDTH     2
#define TARGET_BOX_LINE_WIDTH 2

#define ORBIT_ARC_WIDTH_BELOW 2
#define ORBIT_ARC_WIDTH_ABOVE 2

#define MOON_ORBIT_ARC_WIDTH_BELOW 3
#define MOON_ORBIT_ARC_WIDTH_ABOVE 3

#define SUN_ALTITUDE_LINE_WIDTH_BELOW 3
#define SUN_ALTITUDE_LINE_WIDTH_AVOVE 3

// ============================================================================
// PLANETS (orbits and sizes calculated from available space)
// Size scale: Sun=8, Jupiter=6, Saturn/Earth=5, Venus/Mars/Uranus/Neptune=4, Mercury=3, Moon=2
// Scaled by SIZE_UNIT to fit within orbit spacing
// ============================================================================
static Planet sun     = {0,              SIZE_UNIT * 8 / 2,  0, 0, {0}, NULL, NULL, NULL};  // Largest
static Planet mercury = {ORBIT_STEP * 1, SIZE_UNIT * 3 / 2,  0, 0, {0}, NULL, NULL, NULL};  // Smallest planet
static Planet venus   = {ORBIT_STEP * 2, SIZE_UNIT * 4 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet earth   = {ORBIT_STEP * 3, SIZE_UNIT * 5 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet moon    = {ORBIT_STEP / 2, SIZE_UNIT * 3 / 2,  0, 0, {0}, NULL, NULL, NULL};  // Relative to Earth
static Planet mars    = {ORBIT_STEP * 4, SIZE_UNIT * 4 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet jupiter = {ORBIT_STEP * 5, SIZE_UNIT * 6 / 2,  0, 0, {0}, NULL, NULL, NULL};  // Largest planet
static Planet saturn  = {ORBIT_STEP * 6, SIZE_UNIT * 5 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet uranus  = {ORBIT_STEP * 7, SIZE_UNIT * 4 / 2,  0, 0, {0}, NULL, NULL, NULL};
static Planet neptune = {ORBIT_STEP * 8, SIZE_UNIT * 4 / 2,  0, 0, {0}, NULL, NULL, NULL};

// ============================================================================
// LVGL OBJECTS
// ============================================================================
static lv_obj_t * volatile astro_container = NULL;
static lv_obj_t * zodiac_lines[12] = {NULL};
static lv_point_precise_t zodiac_points[12][2];

static lv_obj_t * sun_altitude_line = NULL;
lv_point_precise_t sun_altitude_points[2];

static lv_obj_t * mercury_altitude_line = NULL;
lv_point_precise_t mercury_altitude_points[2];

static lv_obj_t * venus_altitude_line = NULL;
lv_point_precise_t venus_altitude_points[2];

static lv_obj_t * earth_altitude_line = NULL;
lv_point_precise_t earth_altitude_points[2];

static lv_obj_t * moon_altitude_line = NULL;
lv_point_precise_t moon_altitude_points[2];

static lv_obj_t * mars_altitude_line = NULL;
lv_point_precise_t mars_altitude_points[2];

static lv_obj_t * jupiter_altitude_line = NULL;
lv_point_precise_t jupiter_altitude_points[2];

static lv_obj_t * saturn_altitude_line = NULL;
lv_point_precise_t saturn_altitude_points[2];

static lv_obj_t * uranus_altitude_line = NULL;
lv_point_precise_t uranus_altitude_points[2];

static lv_obj_t * neptune_altitude_line = NULL;
lv_point_precise_t neptune_altitude_points[2];

static lv_obj_t * moon_shadow = NULL;  // Shadow overlay for moon phase
static lv_obj_t * saturn_ring = NULL;  // Saturn's rings
static lv_point_precise_t saturn_ring_points[2];

// Target data display objects
static lv_obj_t * target_data_box = NULL;       // Box to display target object data
static lv_obj_t * target_connector_line = NULL; // Line connecting target box to data box
static lv_point_precise_t connector_points[2];

// Data box margin
#define DATA_BOX_MARGIN 10

// ============================================================================
// TO RADIANS
// ============================================================================
static inline float deg2rad(float deg) {return deg * M_PI / 180.0f;}
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI;}

// ============================================================================
// CELESTIAL OBJECT CLICK CALLBACK
// ============================================================================
static void celestial_click_cb(lv_event_t * e) {
    int target = (int)(intptr_t)lv_event_get_user_data(e);
    astro_clock_set_target(target);
}

// ============================================================================
// CONTAINER CLICK CALLBACK (reset target when clicking background)
// ============================================================================
static void container_click_cb(lv_event_t * e) {
    lv_obj_t * target_obj = (lv_obj_t *)lv_event_get_target(e);
    lv_obj_t * current_obj = (lv_obj_t *)lv_event_get_current_target(e);
    // Only reset if clicking directly on container (not on a child object)
    if (target_obj == current_obj) {
        astro_clock_set_target(ASTRO_TARGET_NONE);
    }
}

// ============================================================================
// CREATE PLANET CIRCLE
// ============================================================================
static lv_obj_t * create_planet(lv_obj_t * parent, int radius, lv_color_t color) {
    printf("DEBUG: creating planet\n");
    lv_obj_t * obj = lv_obj_create(parent);
    if (!obj) {printf("ERROR: Failed to create planet\n"); return NULL;}
    lv_obj_remove_style_all(obj);
    lv_obj_set_size(obj, radius * 2, radius * 2);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(obj, color, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN); // Hidden until position update
    return obj;
}

// ============================================================================
// CREATE ORBIT ARC
// ============================================================================
static lv_obj_t * create_orbit(lv_obj_t * parent, int radius, lv_color_t color) {
    printf("DEBUG: creating arc\n");
    lv_obj_t * arc = lv_arc_create(parent);
    if (!arc) {printf("ERROR: Failed to create arc\n"); return NULL;}
    lv_obj_remove_style_all(arc);
    lv_obj_set_size(arc, radius * 2, radius * 2);
    lv_obj_set_style_arc_color(arc, color, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, ORBIT_ARC_WIDTH_BELOW, LV_PART_MAIN);  // Slightly wider for smoother appearance
    lv_obj_set_style_arc_rounded(arc, true, LV_PART_MAIN);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_value(arc, 0);
    lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(arc, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
    return arc;
}

// ============================================================================
// CREATE TARGET BOX
// ============================================================================
static lv_obj_t * create_target_box(lv_obj_t * parent, int size) {
    if (!parent || !lv_obj_is_valid(parent)) {
        printf("ERROR: create_target_box called with invalid parent (ptr=%p)\n", (void*)parent); return NULL;
    }

    printf("DEBUG: create_target_box parent=%p size=%d\n", (void*)parent, size);
    lv_obj_t * box = lv_obj_create(parent);
    if (!box) {printf("ERROR: Failed to create target box\n"); return NULL;}

    lv_obj_remove_style_all(box);
    lv_obj_set_size(box, size + 8, size + 8);
    lv_obj_set_style_border_width(box, TARGET_BOX_LINE_WIDTH, 0);
    lv_obj_set_style_border_color(box, COLOR_TARGET, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_CLICKABLE);
    printf("DEBUG: create_target_box created box=%p\n", (void*)box);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    return box;
}

// ============================================================================
// UPDATE PLANET POSITION
// ============================================================================
static void update_planet_pos(Planet * p, float angle_deg, int cx, int cy) {
    if (!p->obj) return;
    float rad = deg2rad(angle_deg + ANGLE_OFFSET);
    p->x = cx + (int)(p->orbit_radius * sinf(rad)) - p->radius;
    p->y = cy + (int)(p->orbit_radius * cosf(rad)) - p->radius;
    lv_obj_set_pos(p->obj, p->x, p->y);
    if (p->target_box) {
        lv_obj_set_pos(p->target_box, p->x - 4, p->y - 4);
    }
}

// ============================================================================
// ZODIAC
// ============================================================================
static void create_zodiac(lv_obj_t * parent) {
    for (int i = 0; i < 12; i++) {
        zodiac_lines[i] = lv_line_create(parent);
        lv_obj_set_style_line_color(zodiac_lines[i], COLOR_ZODIAC, 0);
        lv_obj_set_style_line_width(zodiac_lines[i], ZODIAC_LINE_WIDTH, 0);
        zodiac_points[i][0].x = 0;
        zodiac_points[i][0].y = 0;
        zodiac_points[i][1].x = 0;
        zodiac_points[i][1].y = 0;
    }
}
static void hide_zodiac() {
    for (int i = 0; i < 12; i++) {
        if (zodiac_lines[i]) {
            lv_obj_add_flag(zodiac_lines[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}
static void show_zodiac() {
    for (int i = 0; i < 12; i++) {
        if (zodiac_lines[i]) {
            lv_obj_clear_flag(zodiac_lines[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}
static void update_zodiac(int earth_x, int earth_y) {
    const float step = 2.0f * M_PI / 12.0f;
    const float neptune_r = (float)neptune.orbit_radius;
    
    // Iterate through 12 zodiac lines
    for (int i = 0; i < 12; i++) {
        float angle = i * step;
        float dx = cosf(angle);
        float dy = sinf(angle);
        
        // Calculate intersection of lines from Earth with Neptune's orbit circle
        float ex = (float)(earth_x - SOLAR_CENTER_X);
        float ey = (float)(earth_y - SOLAR_CENTER_Y);
        
        // Quadratic coefficients: a*t^2 + b*t + c = 0
        float a = dx * dx + dy * dy;  // = 1 for unit direction
        float b = 2.0f * (ex * dx + ey * dy);
        float c = ex * ex + ey * ey - neptune_r * neptune_r;
        
        float discriminant = b * b - 4.0f * a * c;
        float r = 0.0f;
        
        if (discriminant >= 0) {
            float sqrt_disc = sqrtf(discriminant);
            float t1 = (-b + sqrt_disc) / (2.0f * a);
            float t2 = (-b - sqrt_disc) / (2.0f * a);
            // Take positive t (direction we're drawing toward)
            if (t1 > 0 && t2 > 0) {
                r = (t1 < t2) ? t1 : t2;
            } else if (t1 > 0) {
                r = t1;
            } else if (t2 > 0) {
                r = t2;
            }
        }
        // Set points for current zodiac line
        zodiac_points[i][0].x = earth_x;
        zodiac_points[i][0].y = earth_y;
        zodiac_points[i][1].x = earth_x + (int)(dx * r);
        zodiac_points[i][1].y = earth_y + (int)(dy * r);
        
        lv_line_set_points(zodiac_lines[i], zodiac_points[i], 2);
    }
}

// ============================================================================
// Altitude
// ============================================================================
/**
 * Display altitde at users earth cooridantes (approx. direction facing
 * directly outwards into 2D space, relative to the sun).
 * 
 * Should account for extreme latitues (months of daylight/night time)
 * by using sun azimuth which is calculated using current position.
 * 
 * Due to scale, shape & perspective, this line may only accurately reflect
 * altitde direction relative to the sun, rather than also accurately
 * reflecting direction relative to all other planets.
 * 
 * Orbital path lines reflect objects that are actually above the
 * horizon (altitude > 0).
 */
static void update_altitude_line(lv_obj_t * altitude_line, float altitude_angle, lv_point_precise_t * altitude_points, float intersection) {
    if (!altitude_line) return;
    
    float rad = deg2rad(altitude_angle);
    float dx = cosf(rad);
    float dy = sinf(rad);
    
    // Calculate intersection with Neptune's orbit circle (same as zodiac lines)
    // const float neptune_r = (float)neptune.orbit_radius;
    
    float ex = (float)((earth.x + earth.radius) - SOLAR_CENTER_X);
    float ey = (float)((earth.y + earth.radius) - SOLAR_CENTER_Y);
    
    // Quadratic coefficients: a*t^2 + b*t + c = 0
    float a = dx * dx + dy * dy;  // = 1 for unit direction
    float b = 2.0f * (ex * dx + ey * dy);
    float c = ex * ex + ey * ey - intersection * intersection;
    
    float discriminant = b * b - 4.0f * a * c;
    float r = 0.0f;
    
    if (discriminant >= 0) {
        float sqrt_disc = sqrtf(discriminant);
        float t1 = (-b + sqrt_disc) / (2.0f * a);
        float t2 = (-b - sqrt_disc) / (2.0f * a);
        // Take positive t (direction we're drawing toward)
        if (t1 > 0 && t2 > 0) {
            r = (t1 < t2) ? t1 : t2;
        } else if (t1 > 0) {
            r = t1;
        } else if (t2 > 0) {
            r = t2;
        }
    }
    // Set points for altitde line (from Earth's center outwards)
    altitude_points[0].x = (earth.x + earth.radius);
    altitude_points[0].y = (earth.y + earth.radius);
    altitude_points[1].x = (earth.x + earth.radius) + (int)(dx * r);
    altitude_points[1].y = (earth.y + earth.radius) + (int)(dy * r);
    
    lv_line_set_points(altitude_line, altitude_points, 2);
}

// ============================================================================
// UPDATE ASTRO CLOCK
// ============================================================================
void astro_clock_update(void) {

    if (!astro_container) {return;}
    
    // -----------------------------------------------------------------
    //                                                           MERCURY
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_mercury) {
        update_planet_pos(&mercury, siderealPlanetData.mercury_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (mercury.orbit) {
            lv_color_t color = (siderealPlanetData.mercury_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(mercury.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.mercury_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(mercury.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(mercury.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(mercury.obj, LV_OBJ_FLAG_HIDDEN);
    }
    else {
        lv_obj_add_flag(mercury.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(mercury.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                             VENUS
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_venus) {
        update_planet_pos(&venus, siderealPlanetData.venus_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (venus.orbit) {
            lv_color_t color = (siderealPlanetData.venus_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(venus.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.venus_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(venus.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(venus.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(venus.obj, LV_OBJ_FLAG_HIDDEN); 
    }
    else {
        lv_obj_add_flag(venus.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(venus.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                             EARTH
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_earth) {
        float rad = deg2rad(siderealPlanetData.earth_ecliptic_long - ANGLE_OFFSET);
        earth.x = SOLAR_CENTER_X + (int)(earth.orbit_radius * sinf(rad)) - earth.radius;
        earth.y = SOLAR_CENTER_Y + (int)(earth.orbit_radius * cosf(rad)) - earth.radius;
        lv_obj_set_pos(earth.obj, earth.x, earth.y);
        if (earth.target_box) {
            lv_obj_set_pos(earth.target_box, earth.x - 4, earth.y - 4);
        }
        if (earth.orbit) {
            lv_obj_set_style_arc_color(earth.orbit, COLOR_ORBIT_MOON_ABOVE, LV_PART_MAIN); // change to reflect pos alt
        }

        // Draw Zodiac lines from earth
        update_zodiac((earth.x + earth.radius), (earth.y + earth.radius));

        float ecliptic_long;
        float altitde_angle;

        ecliptic_long = -siderealPlanetData.earth_ecliptic_long; // points directly at the sun
        altitde_angle = ecliptic_long - siderealPlanetData.sun_az + 180.0f; // rotate by az = deg relative to local pos, sun pos
        lv_color_t color = (siderealPlanetData.sun_alt <= 0) ? COLOR_SUN_BELOW : COLOR_SUN_ABOVE;
        lv_obj_set_style_line_color(sun_altitude_line, color, 0);
        update_altitude_line(sun_altitude_line, altitde_angle, sun_altitude_points, neptune.orbit_radius);

        // ecliptic_long = -siderealPlanetData.mercury_ecliptic_long;
        // altitde_angle = ecliptic_long - (siderealPlanetData.mercury_az + 180.0f);
        // update_altitude_line(mercury_altitude_line, altitde_angle, mercury_altitude_points, mercury.orbit_radius);

        // ecliptic_long = -siderealPlanetData.venus_ecliptic_long;
        // altitde_angle = ecliptic_long - (siderealPlanetData.venus_az + 180.0f);
        // update_altitude_line(venus_altitude_line, altitde_angle, venus_altitude_points, venus.orbit_radius);

        // ecliptic_long = -siderealPlanetData.moon_ecliptic_long;
        // altitde_angle = ecliptic_long - (siderealPlanetData.moon_az + 180.0f);
        // update_altitude_line(moon_altitude_line, altitde_angle, moon_altitude_points, moon.orbit_radius);

        // ecliptic_long = -siderealPlanetData.mars_ecliptic_long;
        // altitde_angle = ecliptic_long - (siderealPlanetData.mars_az + 180.0f);
        // update_altitude_line(mars_altitude_line, altitde_angle, mars_altitude_points, mars.orbit_radius);

        // ecliptic_long = -siderealPlanetData.jupiter_ecliptic_long;
        // altitde_angle = ecliptic_long - (siderealPlanetData.jupiter_az + 180.0f);
        // update_altitude_line(jupiter_altitude_line, altitde_angle, jupiter_altitude_points, jupiter.orbit_radius);

        // ecliptic_long = -siderealPlanetData.saturn_ecliptic_long;
        // altitde_angle = ecliptic_long - (siderealPlanetData.saturn_az + 180.0f);
        // update_altitude_line(saturn_altitude_line, altitde_angle, saturn_altitude_points, saturn.orbit_radius);

        // ecliptic_long = -siderealPlanetData.uranus_ecliptic_long;
        // altitde_angle = ecliptic_long - (siderealPlanetData.uranus_az + 180.0f);
        // update_altitude_line(uranus_altitude_line, altitde_angle, uranus_altitude_points, uranus.orbit_radius);

        // ecliptic_long = -siderealPlanetData.neptune_ecliptic_long;
        // altitde_angle = ecliptic_long - (siderealPlanetData.neptune_az + 180.0f);
        // update_altitude_line(neptune_altitude_line, altitde_angle, neptune_altitude_points, neptune.orbit_radius);

        lv_obj_clear_flag(earth.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(earth.obj, LV_OBJ_FLAG_HIDDEN);
        show_zodiac();
    }
    else {
        hide_zodiac();
        lv_obj_add_flag(earth.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(earth.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                              MOON
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_moon) {
        
        // Moon uses RA mapped from 0-24 hours to 0-360 degrees
        float moon_angle = (siderealPlanetData.moon_ra / 24.0f) * 360.0f;
        float moon_rad = deg2rad(moon_angle + ANGLE_OFFSET);
        
        // Position moon relative to Earth's center
        moon.x = (earth.x + earth.radius) + (int)(moon.orbit_radius * sinf(moon_rad)) - moon.radius;
        moon.y = (earth.y + earth.radius) + (int)(moon.orbit_radius * cosf(moon_rad)) - moon.radius;
        
        lv_obj_set_pos(moon.obj, moon.x, moon.y);
        if (moon.target_box) {
            lv_obj_set_pos(moon.target_box, moon.x - 4, moon.y - 4);
        }
        
        // Update moon phase visualization
        // moon_p: 0=New, 1=WaxCres, 2=FirstQ, 3=WaxGib, 4=Full, 5=WanGib, 6=ThirdQ, 7=WanCres
        if (moon_shadow) {
            lv_obj_set_pos(moon_shadow, moon.x, moon.y);
            int phase = (int)siderealPlanetData.moon_p;
            if (phase < 0 || phase > 7) phase = 0;  // Default to new moon if invalid
            
            // Phase determines shadow coverage and side
            // Waxing (1-3): shadow on left, shrinking
            // Waning (5-7): shadow on right, growing
            switch (phase) {
                case 0:  // New Moon - full shadow
                    lv_arc_set_bg_angles(moon_shadow, 0, 360);
                    lv_obj_clear_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 1:  // Waxing Crescent - 75% shadow left
                    lv_arc_set_bg_angles(moon_shadow, 45, 315);
                    lv_obj_clear_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 2:  // First Quarter - 50% shadow left
                    lv_arc_set_bg_angles(moon_shadow, 90, 270);
                    lv_obj_clear_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 3:  // Waxing Gibbous - 25% shadow left
                    lv_arc_set_bg_angles(moon_shadow, 135, 225);
                    lv_obj_clear_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 4:  // Full Moon - no shadow
                    lv_obj_add_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 5:  // Waning Gibbous - 25% shadow right
                    lv_arc_set_bg_angles(moon_shadow, 315, 405);  // 315 to 45 (wraps)
                    lv_obj_clear_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 6:  // Third Quarter - 50% shadow right
                    lv_arc_set_bg_angles(moon_shadow, 270, 450);  // 270 to 90 (wraps)
                    lv_obj_clear_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
                case 7:  // Waning Crescent - 75% shadow right
                    lv_arc_set_bg_angles(moon_shadow, 225, 495);  // 225 to 135 (wraps)
                    lv_obj_clear_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
                    break;
            }
        }
        
        // Position moon orbit arc centered on Earth
        if (moon.orbit) {
            lv_obj_align_to(moon.orbit, earth.obj, LV_ALIGN_CENTER, 0, 0);
            lv_color_t color = (siderealPlanetData.moon_alt <= 0) ? COLOR_ORBIT_MOON_BELOW : COLOR_ORBIT_MOON_ABOVE;
            lv_obj_set_style_arc_color(moon.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.moon_alt <= 0) ? MOON_ORBIT_ARC_WIDTH_BELOW : MOON_ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(moon.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(moon.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(moon.obj, LV_OBJ_FLAG_HIDDEN);

    }
    
    else {
        lv_obj_add_flag(moon.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(moon.obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(moon_shadow, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                              MARS
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_mars) {
        update_planet_pos(&mars, siderealPlanetData.mars_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (mars.orbit) {
            lv_color_t color = (siderealPlanetData.mars_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(mars.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.mars_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(mars.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(mars.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(mars.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    else {
        lv_obj_add_flag(mars.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(mars.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                           JUPITER
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_jupiter) {
        update_planet_pos(&jupiter, siderealPlanetData.jupiter_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (jupiter.orbit) {
            lv_color_t color = (siderealPlanetData.jupiter_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(jupiter.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.jupiter_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(jupiter.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(jupiter.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(jupiter.obj, LV_OBJ_FLAG_HIDDEN);
    }

    else {
        lv_obj_add_flag(jupiter.obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(jupiter.orbit, LV_OBJ_FLAG_HIDDEN);
    }

    // -----------------------------------------------------------------
    //                                                            SATURN
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_saturn) {
        update_planet_pos(&saturn, siderealPlanetData.saturn_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (saturn.orbit) {
            lv_color_t color = (siderealPlanetData.saturn_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(saturn.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.saturn_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(saturn.orbit, width, LV_PART_MAIN);
        }
        // Update Saturn rings position
        if (saturn_ring) {
            int ring_extent = saturn.radius + 5;  // Ring extends beyond planet
            saturn_ring_points[0].x = saturn.x + saturn.radius - ring_extent;
            saturn_ring_points[0].y = saturn.y + saturn.radius;
            saturn_ring_points[1].x = saturn.x + saturn.radius + ring_extent;
            saturn_ring_points[1].y = saturn.y + saturn.radius;
            lv_line_set_points(saturn_ring, saturn_ring_points, 2);
        }
        lv_obj_clear_flag(saturn.obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(saturn.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(saturn_ring, LV_OBJ_FLAG_HIDDEN);
    }

    else {
        lv_obj_add_flag(saturn.obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(saturn.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(saturn_ring, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                            URANUS
    // -----------------------------------------------------------------
    
    if (siderealPlanetData.track_uranus) {
        update_planet_pos(&uranus, siderealPlanetData.uranus_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (uranus.orbit) {
            lv_color_t color = (siderealPlanetData.uranus_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(uranus.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.uranus_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(uranus.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(uranus.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(uranus.obj, LV_OBJ_FLAG_HIDDEN);
    }

    else {
        lv_obj_add_flag(uranus.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(uranus.obj, LV_OBJ_FLAG_HIDDEN);
    }
    
    // -----------------------------------------------------------------
    //                                                           NEPTUNE
    // -----------------------------------------------------------------

    if (siderealPlanetData.track_neptune) {
        update_planet_pos(&neptune, siderealPlanetData.neptune_helio_ecliptic_long, SOLAR_CENTER_X, SOLAR_CENTER_Y);
        if (neptune.orbit) {
            lv_color_t color = (siderealPlanetData.neptune_alt <= 0) ? COLOR_ORBIT_BELOW : COLOR_ORBIT_ABOVE;
            lv_obj_set_style_arc_color(neptune.orbit, color, LV_PART_MAIN);
            int32_t width = (siderealPlanetData.neptune_alt <= 0) ? ORBIT_ARC_WIDTH_BELOW : ORBIT_ARC_WIDTH_ABOVE;
            lv_obj_set_style_arc_width(neptune.orbit, width, LV_PART_MAIN);
        }
        lv_obj_clear_flag(neptune.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(neptune.obj, LV_OBJ_FLAG_HIDDEN);
    }

    else {
        lv_obj_add_flag(neptune.orbit, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(neptune.obj, LV_OBJ_FLAG_HIDDEN);
    }
}

// ============================================================================
// UPDATE TARGET DATA BOX CONTENT
// ============================================================================
/**
 * @brief Update the content displayed in the target data box.
 * @param target The target object ID (ASTRO_TARGET_*)
 * 
 * Called when a celestial object is selected. Add your data display code here.
 */
static void update_target_data_content(int target) {
    if (!target_data_box) return;
    
    // ========================================================================
    // USER: ADD YOUR DATA DISPLAY CODE HERE
    // ========================================================================
    // 
    // The target_data_box is an LVGL container. You can add labels, etc.
    // Use lv_obj_clean(target_data_box) to clear previous content first.
    // 
    // Available targets:
    //   ASTRO_TARGET_SUN, ASTRO_TARGET_MERCURY, ASTRO_TARGET_VENUS,
    //   ASTRO_TARGET_EARTH, ASTRO_TARGET_MOON, ASTRO_TARGET_MARS,
    //   ASTRO_TARGET_JUPITER, ASTRO_TARGET_SATURN, ASTRO_TARGET_URANUS,
    //   ASTRO_TARGET_NEPTUNE
    // 
    // Example:
    //   lv_obj_clean(target_data_box);
    //   lv_obj_t * label = lv_label_create(target_data_box);
    //   switch (target) {
    //       case ASTRO_TARGET_SUN:
    //           lv_label_set_text(label, "Sun\nAlt: 45.2°\nAz: 180.5°");
    //           break;
    //       ...
    //   }
    // 
    // ========================================================================
    lv_obj_clean(target_data_box);
    lv_obj_t * label = lv_label_create(target_data_box);
    lv_obj_set_style_text_font(label, &cobalt_alien_17, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_make(0, 255, 0), LV_PART_MAIN);
    
    char buf[256];
    switch (target) {
        case ASTRO_TARGET_SUN:
            snprintf(buf, sizeof(buf),
                "Sun\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.sun_r,
                siderealPlanetData.sun_s,
                siderealPlanetData.sun_distance,
                siderealPlanetData.sun_ra,
                siderealPlanetData.sun_dec,
                siderealPlanetData.sun_helio_ecliptic_lat,
                siderealPlanetData.sun_helio_ecliptic_long,
                siderealPlanetData.sun_ecliptic_lat,
                siderealPlanetData.sun_ecliptic_long,
                siderealPlanetData.sun_az,
                siderealPlanetData.sun_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_MERCURY:
            snprintf(buf, sizeof(buf),
                "Mercury\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.mercury_r,
                siderealPlanetData.mercury_s,
                siderealPlanetData.mercury_distance,
                siderealPlanetData.mercury_ra,
                siderealPlanetData.mercury_dec,
                siderealPlanetData.mercury_helio_ecliptic_lat,
                siderealPlanetData.mercury_helio_ecliptic_long,
                siderealPlanetData.mercury_ecliptic_lat,
                siderealPlanetData.mercury_ecliptic_long,
                siderealPlanetData.mercury_az,
                siderealPlanetData.mercury_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_VENUS:
            snprintf(buf, sizeof(buf),
                "Venus\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.venus_r,
                siderealPlanetData.venus_s,
                siderealPlanetData.venus_distance,
                siderealPlanetData.venus_ra,
                siderealPlanetData.venus_dec,
                siderealPlanetData.venus_helio_ecliptic_lat,
                siderealPlanetData.venus_helio_ecliptic_long,
                siderealPlanetData.venus_ecliptic_lat,
                siderealPlanetData.venus_ecliptic_long,
                siderealPlanetData.venus_az,
                siderealPlanetData.venus_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_EARTH:
            snprintf(buf, sizeof(buf),
                "Earth\n\n"
                "Rise: N/A\n"
                "Set: N/A\n"
                "Distance: %.2fm (to sea level)\n"
                "Right Ascension: N/A\n"
                "Declination: N/A\n"
                "Helio Ecliptic Latitude: N/A\n"
                "Helio Ecliptic Longitude: N/A\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: N/A\n"
                "Altitude: N/A",
                atof(gnggaData.altitude),
                siderealPlanetData.earth_ecliptic_lat,
                siderealPlanetData.earth_ecliptic_long
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_MOON:
            snprintf(buf, sizeof(buf),
                "Moon\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Phase: %s\n"
                "Luminance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.moon_r,
                siderealPlanetData.moon_s,
                siderealPlanetData.moon_p_name[(int)siderealPlanetData.moon_p],
                siderealPlanetData.moon_lum,
                siderealPlanetData.moon_ra,
                siderealPlanetData.moon_dec,
                siderealPlanetData.moon_az,
                siderealPlanetData.moon_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_MARS:
            snprintf(buf, sizeof(buf),
                "Mars\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.mars_r,
                siderealPlanetData.mars_s,
                siderealPlanetData.mars_distance,
                siderealPlanetData.mars_ra,
                siderealPlanetData.mars_dec,
                siderealPlanetData.mars_helio_ecliptic_lat,
                siderealPlanetData.mars_helio_ecliptic_long,
                siderealPlanetData.mars_ecliptic_lat,
                siderealPlanetData.mars_ecliptic_long,
                siderealPlanetData.mars_az,
                siderealPlanetData.mars_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_JUPITER:
            snprintf(buf, sizeof(buf),
                "Jupiter\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.jupiter_r,
                siderealPlanetData.jupiter_s,
                siderealPlanetData.jupiter_distance,
                siderealPlanetData.jupiter_ra,
                siderealPlanetData.jupiter_dec,
                siderealPlanetData.jupiter_helio_ecliptic_lat,
                siderealPlanetData.jupiter_helio_ecliptic_long,
                siderealPlanetData.jupiter_ecliptic_lat,
                siderealPlanetData.jupiter_ecliptic_long,
                siderealPlanetData.jupiter_az,
                siderealPlanetData.jupiter_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_SATURN:
            snprintf(buf, sizeof(buf),
                "Saturn\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.saturn_r,
                siderealPlanetData.saturn_s,
                siderealPlanetData.saturn_distance,
                siderealPlanetData.saturn_ra,
                siderealPlanetData.saturn_dec,
                siderealPlanetData.saturn_helio_ecliptic_lat,
                siderealPlanetData.saturn_helio_ecliptic_long,
                siderealPlanetData.saturn_ecliptic_lat,
                siderealPlanetData.saturn_ecliptic_long,
                siderealPlanetData.saturn_az,
                siderealPlanetData.saturn_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_URANUS:
            snprintf(buf, sizeof(buf),
                "Uranus\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.uranus_r,
                siderealPlanetData.uranus_s,
                siderealPlanetData.uranus_distance,
                siderealPlanetData.uranus_ra,
                siderealPlanetData.uranus_dec,
                siderealPlanetData.uranus_helio_ecliptic_lat,
                siderealPlanetData.uranus_helio_ecliptic_long,
                siderealPlanetData.uranus_ecliptic_lat,
                siderealPlanetData.uranus_ecliptic_long,
                siderealPlanetData.uranus_az,
                siderealPlanetData.uranus_alt
            );
            lv_label_set_text(label, buf);
            break;
        case ASTRO_TARGET_NEPTUNE:
            snprintf(buf, sizeof(buf),
                "Neptune\n\n"
                "Rise: %.2f\n"
                "Set: %.2f\n"
                "Distance: %.2f\n"
                "Right Ascension: %.2f\n"
                "Declination: %.2f\n"
                "Helio Ecliptic Latitude: %.2f\n"
                "Helio Ecliptic Longitude: %.2f\n"
                "Ecliptic Latitude: %.2f\n"
                "Ecliptic Longitude: %.2f\n"
                "Azimuth: %.2f\n"
                "Altitude: %.2f",
                siderealPlanetData.neptune_r,
                siderealPlanetData.neptune_s,
                siderealPlanetData.neptune_distance,
                siderealPlanetData.neptune_ra,
                siderealPlanetData.neptune_dec,
                siderealPlanetData.neptune_helio_ecliptic_lat,
                siderealPlanetData.neptune_helio_ecliptic_long,
                siderealPlanetData.neptune_ecliptic_lat,
                siderealPlanetData.neptune_ecliptic_long,
                siderealPlanetData.neptune_az,
                siderealPlanetData.neptune_alt
            );
            lv_label_set_text(label, buf);
            break;
    }
}

// ============================================================================
// SET TARGET
// ============================================================================
void astro_clock_set_target(int target) {
    // Hide all target boxes
    if (sun.target_box) lv_obj_add_flag(sun.target_box, LV_OBJ_FLAG_HIDDEN);
    if (mercury.target_box) lv_obj_add_flag(mercury.target_box, LV_OBJ_FLAG_HIDDEN);
    if (venus.target_box) lv_obj_add_flag(venus.target_box, LV_OBJ_FLAG_HIDDEN);
    if (earth.target_box) lv_obj_add_flag(earth.target_box, LV_OBJ_FLAG_HIDDEN);
    if (moon.target_box) lv_obj_add_flag(moon.target_box, LV_OBJ_FLAG_HIDDEN);
    if (mars.target_box) lv_obj_add_flag(mars.target_box, LV_OBJ_FLAG_HIDDEN);
    if (jupiter.target_box) lv_obj_add_flag(jupiter.target_box, LV_OBJ_FLAG_HIDDEN);
    if (saturn.target_box) lv_obj_add_flag(saturn.target_box, LV_OBJ_FLAG_HIDDEN);
    if (uranus.target_box) lv_obj_add_flag(uranus.target_box, LV_OBJ_FLAG_HIDDEN);
    if (neptune.target_box) lv_obj_add_flag(neptune.target_box, LV_OBJ_FLAG_HIDDEN);
    
    // Hide data box and connector line
    if (target_data_box) lv_obj_add_flag(target_data_box, LV_OBJ_FLAG_HIDDEN);
    if (target_connector_line) lv_obj_add_flag(target_connector_line, LV_OBJ_FLAG_HIDDEN);
    
    current_target = target;
    
    // If no target selected, we're done
    if (target == ASTRO_TARGET_NONE) return;
    
    // Get target box and object center coordinates
    lv_obj_t * box = NULL;
    int obj_center_x = 0;
    int obj_center_y = 0;
    
    switch (target) {
        case ASTRO_TARGET_SUN:
            box = sun.target_box;
            obj_center_x = SOLAR_CENTER_X;
            obj_center_y = SOLAR_CENTER_Y;
            break;
        case ASTRO_TARGET_MERCURY:
            box = mercury.target_box;
            obj_center_x = mercury.x + mercury.radius;
            obj_center_y = mercury.y + mercury.radius;
            break;
        case ASTRO_TARGET_VENUS:
            box = venus.target_box;
            obj_center_x = venus.x + venus.radius;
            obj_center_y = venus.y + venus.radius;
            break;
        case ASTRO_TARGET_EARTH:
            box = earth.target_box;
            obj_center_x = earth.x + earth.radius;
            obj_center_y = earth.y + earth.radius;
            break;
        case ASTRO_TARGET_MOON:
            box = moon.target_box;
            obj_center_x = moon.x + moon.radius;
            obj_center_y = moon.y + moon.radius;
            break;
        case ASTRO_TARGET_MARS:
            box = mars.target_box;
            obj_center_x = mars.x + mars.radius;
            obj_center_y = mars.y + mars.radius;
            break;
        case ASTRO_TARGET_JUPITER:
            box = jupiter.target_box;
            obj_center_x = jupiter.x + jupiter.radius;
            obj_center_y = jupiter.y + jupiter.radius;
            break;
        case ASTRO_TARGET_SATURN:
            box = saturn.target_box;
            obj_center_x = saturn.x + saturn.radius;
            obj_center_y = saturn.y + saturn.radius;
            break;
        case ASTRO_TARGET_URANUS:
            box = uranus.target_box;
            obj_center_x = uranus.x + uranus.radius;
            obj_center_y = uranus.y + uranus.radius;
            break;
        case ASTRO_TARGET_NEPTUNE:
            box = neptune.target_box;
            obj_center_x = neptune.x + neptune.radius;
            obj_center_y = neptune.y + neptune.radius;
            break;
    }
    
    // Show target box
    if (box) lv_obj_clear_flag(box, LV_OBJ_FLAG_HIDDEN);
    
    // Update data box content FIRST so we can measure its size
    update_target_data_content(target);
    lv_obj_update_layout(target_data_box);  // Force layout update to calculate size
    
    // Get actual data box dimensions after content is set
    int data_box_width = lv_obj_get_width(target_data_box);
    int data_box_height = lv_obj_get_height(target_data_box);
    
    // -----------------------------------------------------------------
    // Position data box based on object location in container
    // -----------------------------------------------------------------
    // Horizontal: Left side → data box on RIGHT, Right side → LEFT
    // Vertical: Top half → data box BELOW, Bottom half → ABOVE
    // -----------------------------------------------------------------
    
    int data_box_x, data_box_y;
    int connector_start_x, connector_start_y;
    int connector_end_x, connector_end_y;
    
    // Determine if object is on left or right side of container
    bool on_right_side = (obj_center_x > SOLAR_CENTER_X);
    // Determine if object is in top or bottom half of container
    bool in_top_half = (obj_center_y < SOLAR_CENTER_Y);
    
    // Horizontal positioning (left/right)
    if (on_right_side) {
        // Object is on right side: place data box on LEFT of object
        data_box_x = obj_center_x - data_box_width - DATA_BOX_MARGIN - 20;
        connector_start_x = obj_center_x - DATA_BOX_MARGIN;
        connector_end_x = data_box_x + data_box_width;
    } else {
        // Object is on left side or center: place data box on RIGHT of object
        data_box_x = obj_center_x + DATA_BOX_MARGIN + 20;
        connector_start_x = obj_center_x + DATA_BOX_MARGIN;
        connector_end_x = data_box_x;
    }
    
    // Vertical positioning (above/below)
    if (in_top_half) {
        // Object is in top half: place data box BELOW object
        data_box_y = obj_center_y + DATA_BOX_MARGIN + 20;
        connector_start_y = obj_center_y + DATA_BOX_MARGIN;
        connector_end_y = data_box_y;
    } else {
        // Object is in bottom half: place data box ABOVE object
        data_box_y = obj_center_y - data_box_height - DATA_BOX_MARGIN - 20;
        connector_start_y = obj_center_y - DATA_BOX_MARGIN;
        connector_end_y = data_box_y + data_box_height;
    }
    
    // Clamp data box X to container bounds
    if (data_box_x < DATA_BOX_MARGIN) data_box_x = DATA_BOX_MARGIN;
    if (data_box_x + data_box_width > ASTRO_WIDTH - DATA_BOX_MARGIN) {
        data_box_x = ASTRO_WIDTH - data_box_width - DATA_BOX_MARGIN;
    }
    
    // Clamp data box Y to container bounds
    if (data_box_y < DATA_BOX_MARGIN) data_box_y = DATA_BOX_MARGIN;
    if (data_box_y + data_box_height > ASTRO_HEIGHT - DATA_BOX_MARGIN) {
        data_box_y = ASTRO_HEIGHT - data_box_height - DATA_BOX_MARGIN;
    }
    
    // Position and show data box
    if (target_data_box) {
        lv_obj_set_pos(target_data_box, data_box_x, data_box_y);
        lv_obj_clear_flag(target_data_box, LV_OBJ_FLAG_HIDDEN);
    }
    
    // Position and show connector line
    if (target_connector_line) {
        connector_points[0].x = connector_start_x;
        connector_points[0].y = connector_start_y;
        connector_points[1].x = connector_end_x;
        connector_points[1].y = connector_end_y;
        lv_line_set_points(target_connector_line, connector_points, 2);
        lv_obj_clear_flag(target_connector_line, LV_OBJ_FLAG_HIDDEN);
    }
}

/** ---------------------------------------------------------------------------------------
 * @brief Astro clock animation callback to update positions.
 */
static void astro_timer_cb(lv_timer_t * timer) {
    astro_clock_update();
}

// ============================================================================
// INIT ASTRO CLOCK
// ============================================================================
void astro_clock_begin(
    lv_obj_t * parent,
    int32_t width,
    int32_t height,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int angle_offset
    ) 
{
    // Guard: ensure parent is valid
    if (!parent || !lv_obj_is_valid(parent)) {
        printf("ERROR: astro_clock_begin called with invalid parent\n");
        return;
    }
    
    astro_clock_end();
    
    // Angle offset for positioning
    ANGLE_OFFSET = angle_offset;

    // Dimensions for the astro clock display
    ASTRO_WIDTH = width;
    ASTRO_HEIGHT = height;

    // Center of astro clock display
    SOLAR_CENTER_X = ASTRO_WIDTH / 2;
    SOLAR_CENTER_Y = ASTRO_HEIGHT / 2;

    // Max usable orbit radius (leave margin for planet size)
    MAX_ORBIT_RADIUS = (ASTRO_WIDTH < ASTRO_HEIGHT ? ASTRO_WIDTH : ASTRO_HEIGHT) / 2 - 15;

    // Orbit step size (8 planetary orbits from Mercury to Neptune)
    ORBIT_STEP = MAX_ORBIT_RADIUS / 8;

    // Planet size unit (max planet radius should fit between orbits)
    SIZE_UNIT = ORBIT_STEP / 6;

    // Current target selection
    current_target = 0;
    
    // Recalculate planet orbit radii and sizes (static initializers used old values)
    sun.orbit_radius = 0;
    sun.radius = SIZE_UNIT * 8 / 2;
    mercury.orbit_radius = ORBIT_STEP * 1;
    mercury.radius = SIZE_UNIT * 3 / 2;
    venus.orbit_radius = ORBIT_STEP * 2;
    venus.radius = SIZE_UNIT * 4 / 2;
    earth.orbit_radius = ORBIT_STEP * 3;
    earth.radius = SIZE_UNIT * 5 / 2;
    moon.orbit_radius = ORBIT_STEP / 2;
    moon.radius = SIZE_UNIT * 3 / 2;
    mars.orbit_radius = ORBIT_STEP * 4;
    mars.radius = SIZE_UNIT * 4 / 2;
    jupiter.orbit_radius = ORBIT_STEP * 5;
    jupiter.radius = SIZE_UNIT * 6 / 2;
    saturn.orbit_radius = ORBIT_STEP * 6;
    saturn.radius = SIZE_UNIT * 5 / 2;
    uranus.orbit_radius = ORBIT_STEP * 7;
    uranus.radius = SIZE_UNIT * 4 / 2;
    neptune.orbit_radius = ORBIT_STEP * 8;
    neptune.radius = SIZE_UNIT * 4 / 2;

    // Set colors
    sun.color = lv_color_make(255, 255, 0);
    mercury.color = lv_color_make(255, 0, 255);
    venus.color = lv_color_make(180, 180, 0);
    earth.color = lv_color_make(0, 0, 255);
    moon.color = lv_color_make(128, 128, 128);
    mars.color = lv_color_make(255, 0, 0);
    jupiter.color = lv_color_make(128, 128, 128);
    saturn.color = lv_color_make(210, 210, 0);
    uranus.color = lv_color_make(0, 255, 255);
    neptune.color = lv_color_make(255, 0, 255);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Container
    printf("DEBUG: Creating astro_container\n");
    astro_container = lv_obj_create(parent);
    if (!astro_container) { printf("ERROR: Failed to create astro_container\n"); return;}
    printf("DEBUG: astro_container done\n");
    // Target boxes - debug print before potentially crashing
    // size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    // printf("DEBUG: Creating target boxes. astro_container=%p sun.radius=%d free_heap=%u\n", 
    //        (void*)astro_container, sun.radius, free_heap);
    // // Validate astro_container before using it
    // if (!astro_container || !lv_obj_is_valid(astro_container)) {
    //     printf("ERROR: astro_container invalid before creating target boxes!\n");
    //     return;
    // }
    lv_obj_remove_style_all(astro_container);
    lv_obj_set_size(astro_container, ASTRO_WIDTH, ASTRO_HEIGHT);
    lv_obj_align(astro_container, alignment, pos_x, pos_y);
    lv_obj_set_style_bg_color(astro_container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(astro_container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(astro_container, 0, 0);
    lv_obj_set_style_border_color(astro_container, COLOR_ZODIAC, 0);
    lv_obj_set_style_outline_width(astro_container, 0, 0);
    lv_obj_set_style_outline_color(astro_container, COLOR_ZODIAC, 0);
    lv_obj_remove_flag(astro_container, LV_OBJ_FLAG_SCROLLABLE);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Zodiac lines
    create_zodiac(astro_container);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Orbits (outer to inner)
    printf("DEBUG: Creating neptune.orbit\n");
    neptune.orbit = create_orbit(astro_container, neptune.orbit_radius, COLOR_ORBIT_BELOW);
    if (!neptune.orbit) { printf("ERROR: Failed to create neptune.orbit\n"); return;}
    printf("DEBUG: neptune.orbit done\n");

    printf("DEBUG: Creating uranus.orbit\n");
    uranus.orbit = create_orbit(astro_container, uranus.orbit_radius, COLOR_ORBIT_BELOW);
    if (!uranus.orbit) { printf("ERROR: Failed to create uranus.orbit\n"); return;}
    printf("DEBUG: uranus.orbit done\n");

    printf("DEBUG: Creating saturn.orbit\n");
    saturn.orbit = create_orbit(astro_container, saturn.orbit_radius, COLOR_ORBIT_BELOW);
    if (!saturn.orbit) { printf("ERROR: Failed to create saturn.orbit\n"); return;}
    printf("DEBUG: saturn.orbit done\n");

    printf("DEBUG: Creating jupiter.orbit\n");
    jupiter.orbit = create_orbit(astro_container, jupiter.orbit_radius, COLOR_ORBIT_BELOW);
    if (!jupiter.orbit) { printf("ERROR: Failed to create jupiter.orbit\n"); return;}
    printf("DEBUG: jupiter.orbit done\n");

    printf("DEBUG: Creating mars.orbit\n");
    mars.orbit = create_orbit(astro_container, mars.orbit_radius, COLOR_ORBIT_BELOW);
    if (!mars.orbit) { printf("ERROR: Failed to create mars.orbit\n"); return;}
    printf("DEBUG: mars.orbit done\n");

    printf("DEBUG: Creating earth.orbit\n");
    earth.orbit = create_orbit(astro_container, earth.orbit_radius, COLOR_ORBIT_MOON_ABOVE);
    if (!earth.orbit) { printf("ERROR: Failed to create earth.orbit\n"); return;}
    printf("DEBUG: earth.orbit done\n");

    printf("DEBUG: Creating venus.orbit\n");
    venus.orbit = create_orbit(astro_container, venus.orbit_radius, COLOR_ORBIT_BELOW);
    if (!venus.orbit) { printf("ERROR: Failed to create venus.orbit\n"); return;}
    printf("DEBUG: venus.orbit done\n");

    printf("DEBUG: Creating mercury.orbit\n");
    mercury.orbit = create_orbit(astro_container, mercury.orbit_radius, COLOR_ORBIT_BELOW);
    if (!mercury.orbit) { printf("ERROR: Failed to create mercury.orbit\n"); return;}
    printf("DEBUG: mercury.orbit done\n");

    vTaskDelay(5 / portTICK_PERIOD_MS);

    // sun.color = lv_color_make(255, 255, 0);
    // mercury.color = lv_color_make(255, 0, 255);
    // venus.color = lv_color_make(180, 180, 0);
    // earth.color = lv_color_make(0, 0, 255);
    // moon.color = lv_color_make(128, 128, 128);
    // mars.color = lv_color_make(255, 0, 0);
    // jupiter.color = lv_color_make(128, 128, 128);
    // saturn.color = lv_color_make(210, 210, 0);
    // uranus.color = lv_color_make(0, 255, 255);
    // neptune.color = lv_color_make(255, 0, 255);

    // Sun altitde line (line from Earth to edge showing local altitde relative to sun)
    printf("DEBUG: Creating sun_altitude_line\n");
    sun_altitude_line = lv_line_create(astro_container);
    if (!sun_altitude_line) { printf("ERROR: Failed to create sun_altitude_line\n"); return;}
    printf("DEBUG: sun_altitude_line done\n");
    lv_obj_set_style_line_color(sun_altitude_line, lv_color_make(128, 0, 0), 0);
    lv_obj_set_style_line_width(sun_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    lv_obj_set_style_line_rounded(sun_altitude_line, true, 0);

    // // Mercury altitde line
    // printf("DEBUG: Creating mercury_altitude_line\n");
    // mercury_altitude_line = lv_line_create(astro_container);
    // if (!mercury_altitude_line) { printf("ERROR: Failed to create mercury_altitude_line\n"); return;}
    // printf("DEBUG: mercury_altitude_line done\n");
    // lv_obj_set_style_line_color(mercury_altitude_line, lv_color_make(128, 128, 0), 0);
    // lv_obj_set_style_line_width(mercury_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    // lv_obj_set_style_line_rounded(mercury_altitude_line, true, 0);

    // // Venus altitde line
    // printf("DEBUG: Creating venus_altitude_line\n");
    // venus_altitude_line = lv_line_create(astro_container);
    // if (!venus_altitude_line) { printf("ERROR: Failed to create venus_altitude_line\n"); return;}
    // printf("DEBUG: venus_altitude_line done\n");
    // lv_obj_set_style_line_color(venus_altitude_line, lv_color_make(0, 0, 128), 0);
    // lv_obj_set_style_line_width(venus_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    // lv_obj_set_style_line_rounded(venus_altitude_line, true, 0);

    // Moon altitde line
    // printf("DEBUG: Creating moon_altitude_line\n");
    // moon_altitude_line = lv_line_create(astro_container);
    // if (!moon_altitude_line) { printf("ERROR: Failed to create moon_altitude_line\n"); return;}
    // printf("DEBUG: moon_altitude_line done\n");
    // lv_obj_set_style_line_color(moon_altitude_line, lv_color_make(128, 128, 128), 0);
    // lv_obj_set_style_line_width(moon_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    // lv_obj_set_style_line_rounded(moon_altitude_line, true, 0);

    // // Mars altitde line (line from Earth to edge showing local altitde relative to mars)
    // printf("DEBUG: Creating mars_altitude_line\n");
    // mars_altitude_line = lv_line_create(astro_container);
    // if (!mars_altitude_line) { printf("ERROR: Failed to create mars_altitude_line\n"); return;}
    // printf("DEBUG: mars_altitude_line done\n");
    // lv_obj_set_style_line_color(mars_altitude_line, lv_color_make(128, 0, 0), 0);
    // lv_obj_set_style_line_width(mars_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    // lv_obj_set_style_line_rounded(mars_altitude_line, true, 0);

    // // Jupiter altitde line
    // printf("DEBUG: Creating jupiter_altitude_line\n");
    // jupiter_altitude_line = lv_line_create(astro_container);
    // if (!jupiter_altitude_line) { printf("ERROR: Failed to create jupiter_altitude_line\n"); return;}
    // printf("DEBUG: jupiter_altitude_line done\n");
    // lv_obj_set_style_line_color(jupiter_altitude_line, lv_color_make(128, 128, 128), 0);
    // lv_obj_set_style_line_width(jupiter_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    // lv_obj_set_style_line_rounded(jupiter_altitude_line, true, 0);

    // // Saturn altitde line
    // printf("DEBUG: Creating saturn_altitude_line\n");
    // saturn_altitude_line = lv_line_create(astro_container);
    // if (!saturn_altitude_line) { printf("ERROR: Failed to create saturn_altitude_line\n"); return;}
    // printf("DEBUG: saturn_altitude_line done\n");
    // lv_obj_set_style_line_color(saturn_altitude_line, lv_color_make(128, 128, 0), 0);
    // lv_obj_set_style_line_width(saturn_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    // lv_obj_set_style_line_rounded(saturn_altitude_line, true, 0);

    // // Uranus altitde line
    // printf("DEBUG: Creating uranus_altitude_line\n");
    // uranus_altitude_line = lv_line_create(astro_container);
    // if (!uranus_altitude_line) { printf("ERROR: Failed to create uranus_altitude_line\n"); return;}
    // printf("DEBUG: uranus_altitude_line done\n");
    // lv_obj_set_style_line_color(uranus_altitude_line, lv_color_make(0, 128, 128), 0);
    // lv_obj_set_style_line_width(uranus_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    // lv_obj_set_style_line_rounded(uranus_altitude_line, true, 0);

    // // Neptune altitde line
    // printf("DEBUG: Creating neptune_altitude_line\n");
    // neptune_altitude_line = lv_line_create(astro_container);
    // if (!neptune_altitude_line) { printf("ERROR: Failed to create neptune_altitude_line\n"); return;}
    // printf("DEBUG: neptune_altitude_line done\n");
    // lv_obj_set_style_line_color(neptune_altitude_line, lv_color_make(128, 0, 128), 0);
    // lv_obj_set_style_line_width(neptune_altitude_line, SUN_ALTITUDE_LINE_WIDTH_BELOW, 0);
    // lv_obj_set_style_line_rounded(neptune_altitude_line, true, 0);

    sun_altitude_points[0].x = 0;
    sun_altitude_points[0].y = 0;
    sun_altitude_points[1].x = 0;
    sun_altitude_points[1].y = 0;

    // mercury_altitude_points[0].x = 0;
    // mercury_altitude_points[0].y = 0;
    // mercury_altitude_points[1].x = 0;
    // mercury_altitude_points[1].y = 0;

    // venus_altitude_points[0].x = 0;
    // venus_altitude_points[0].y = 0;
    // venus_altitude_points[1].x = 0;
    // venus_altitude_points[1].y = 0;

    // earth_altitude_points[0].x = 0;
    // earth_altitude_points[0].y = 0;
    // earth_altitude_points[1].x = 0;
    // earth_altitude_points[1].y = 0;

    // moon_altitude_points[0].x = 0;
    // moon_altitude_points[0].y = 0;
    // moon_altitude_points[1].x = 0;
    // moon_altitude_points[1].y = 0;

    // mars_altitude_points[0].x = 0;
    // mars_altitude_points[0].y = 0;
    // mars_altitude_points[1].x = 0;
    // mars_altitude_points[1].y = 0;

    // jupiter_altitude_points[0].x = 0;
    // jupiter_altitude_points[0].y = 0;
    // jupiter_altitude_points[1].x = 0;
    // jupiter_altitude_points[1].y = 0;

    // saturn_altitude_points[0].x = 0;
    // saturn_altitude_points[0].y = 0;
    // saturn_altitude_points[1].x = 0;
    // saturn_altitude_points[1].y = 0;

    // uranus_altitude_points[0].x = 0;
    // uranus_altitude_points[0].y = 0;
    // uranus_altitude_points[1].x = 0;
    // uranus_altitude_points[1].y = 0;

    // neptune_altitude_points[0].x = 0;
    // neptune_altitude_points[0].y = 0;
    // neptune_altitude_points[1].x = 0;
    // neptune_altitude_points[1].y = 0;

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    int moon_spacing = SIZE_UNIT;  // Gap between Earth surface and Moon orbit
    moon.orbit_radius = earth.radius + moon_spacing + moon.radius;
    moon.orbit = create_orbit(astro_container, moon.orbit_radius, COLOR_ORBIT_MOON_BELOW);
    lv_obj_add_flag(moon.orbit, LV_OBJ_FLAG_HIDDEN);  // Hide until first update positions it

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Sun
    sun.obj = create_planet(astro_container, sun.radius, sun.color);
    lv_obj_set_pos(sun.obj, SOLAR_CENTER_X - sun.radius, SOLAR_CENTER_Y - sun.radius);
    
    // Planets
    neptune.obj = create_planet(astro_container, neptune.radius, neptune.color);
    uranus.obj = create_planet(astro_container, uranus.radius, uranus.color);
    saturn.obj = create_planet(astro_container, saturn.radius, saturn.color);
    saturn_ring = lv_line_create(astro_container);
    lv_obj_set_style_line_color(saturn_ring, lv_color_hex(0xC0A060), LV_PART_MAIN);  // Tan color
    lv_obj_set_style_line_width(saturn_ring, 2, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(saturn_ring, true, LV_PART_MAIN);
    jupiter.obj = create_planet(astro_container, jupiter.radius, jupiter.color);
    mars.obj = create_planet(astro_container, mars.radius, mars.color);
    earth.obj = create_planet(astro_container, earth.radius, earth.color);
    venus.obj = create_planet(astro_container, venus.radius, venus.color);
    mercury.obj = create_planet(astro_container, mercury.radius, mercury.color);
    moon.obj = create_planet(astro_container, moon.radius, moon.color);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    // Moon shadow arc for phase visualization
    moon_shadow = lv_arc_create(astro_container);
    lv_obj_remove_style_all(moon_shadow);
    lv_obj_set_size(moon_shadow, moon.radius * 2, moon.radius * 2);
    lv_obj_set_style_arc_color(moon_shadow, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_arc_width(moon_shadow, moon.radius, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(moon_shadow, false, LV_PART_MAIN);
    lv_arc_set_bg_angles(moon_shadow, 0, 180);
    lv_arc_set_value(moon_shadow, 0);
    lv_obj_set_style_arc_opa(moon_shadow, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(moon_shadow, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(moon_shadow, LV_OBJ_FLAG_CLICKABLE);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    
    printf("DEBUG: Creating sun.target_box\n");
    sun.target_box = create_target_box(astro_container, sun.radius * 2);
    if (!sun.target_box) { printf("ERROR: Failed to create sun.target_box\n"); return;}
    printf("DEBUG: sun.target_box done\n");
    
    printf("DEBUG: Creating mercury.target_box\n");
    mercury.target_box = create_target_box(astro_container, mercury.radius * 2);
    if (!mercury.target_box) { printf("ERROR: Failed to create mercury.target_box\n"); return;}
    printf("DEBUG: mercury.target_box done\n");
    
    printf("DEBUG: Creating venus.target_box\n");
    venus.target_box = create_target_box(astro_container, venus.radius * 2);
    if (!venus.target_box) { printf("ERROR: Failed to create venus.target_box\n"); return;}
    printf("DEBUG: venus.target_box done\n");
    
    printf("DEBUG: Creating earth.target_box\n");
    earth.target_box = create_target_box(astro_container, earth.radius * 2);
    if (!earth.target_box) { printf("ERROR: Failed to create earth.target_box\n"); return;}
    printf("DEBUG: earth.target_box done\n");
    
    printf("DEBUG: Creating moon.target_box\n");
    moon.target_box = create_target_box(astro_container, moon.radius * 2);
    if (!moon.target_box) { printf("ERROR: Failed to create moon.target_box\n"); return;}
    printf("DEBUG: moon.target_box done\n");
    
    printf("DEBUG: Creating mars.target_box\n");
    mars.target_box = create_target_box(astro_container, mars.radius * 2);
    if (!mars.target_box) { printf("ERROR: Failed to create mars.target_box\n"); return;}
    printf("DEBUG: mars.target_box done\n");
    
    printf("DEBUG: Creating jupiter.target_box\n");
    jupiter.target_box = create_target_box(astro_container, jupiter.radius * 2);
    if (!jupiter.target_box) { printf("ERROR: Failed to create jupiter.target_box\n"); return;}
    printf("DEBUG: jupiter.target_box done\n");
    
    printf("DEBUG: Creating saturn.target_box\n");
    saturn.target_box = create_target_box(astro_container, saturn.radius * 2);
    if (!saturn.target_box) { printf("ERROR: Failed to create saturn.target_box\n"); return;}
    printf("DEBUG: saturn.target_box done\n");
    
    printf("DEBUG: Creating uranus.target_box\n");
    uranus.target_box = create_target_box(astro_container, uranus.radius * 2);
    if (!uranus.target_box) { printf("ERROR: Failed to create uranus.target_box\n"); return;}
    printf("DEBUG: uranus.target_box done\n");
    
    printf("DEBUG: Creating neptune.target_box\n");
    neptune.target_box = create_target_box(astro_container, neptune.radius * 2);
    if (!neptune.target_box) { printf("ERROR: Failed to create neptune.target_box\n"); return;}
    printf("DEBUG: neptune.target_box done\n");

    // -----------------------------------------------------------------
    // Target data box (displays object information when selected)
    // -----------------------------------------------------------------
    target_data_box = lv_obj_create(astro_container);
    lv_obj_remove_style_all(target_data_box);
    lv_obj_set_size(target_data_box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_border_width(target_data_box, TARGET_BOX_LINE_WIDTH, 0);
    lv_obj_set_style_border_color(target_data_box, COLOR_TARGET, 0);
    lv_obj_set_style_bg_color(target_data_box, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(target_data_box, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(target_data_box, 4, 0);
    lv_obj_add_flag(target_data_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(target_data_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(target_data_box, LV_OBJ_FLAG_CLICKABLE);
    
    // -----------------------------------------------------------------
    // Connector line (connects target box to data box)
    // -----------------------------------------------------------------
    target_connector_line = lv_line_create(astro_container);
    lv_obj_set_style_line_color(target_connector_line, COLOR_TARGET, 0);
    lv_obj_set_style_line_width(target_connector_line, TARGET_BOX_LINE_WIDTH, 0);
    lv_obj_set_style_line_rounded(target_connector_line, true, 0);
    lv_obj_add_flag(target_connector_line, LV_OBJ_FLAG_HIDDEN);
    connector_points[0].x = 0;
    connector_points[0].y = 0;
    connector_points[1].x = 0;
    connector_points[1].y = 0;

    // Show sun immediately (other objects are shown after positions are set)
    lv_obj_clear_flag(sun.obj, LV_OBJ_FLAG_HIDDEN);

    // Add click event handlers to celestial objects
    lv_obj_add_event_cb(sun.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_SUN);
    lv_obj_add_event_cb(mercury.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_MERCURY);
    lv_obj_add_event_cb(venus.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_VENUS);
    lv_obj_add_event_cb(earth.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_EARTH);
    lv_obj_add_event_cb(moon.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_MOON);
    lv_obj_add_event_cb(mars.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_MARS);
    lv_obj_add_event_cb(jupiter.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_JUPITER);
    lv_obj_add_event_cb(saturn.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_SATURN);
    lv_obj_add_event_cb(uranus.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_URANUS);
    lv_obj_add_event_cb(neptune.obj, celestial_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)ASTRO_TARGET_NEPTUNE);

    // Add click handler to container to reset target when clicking background
    lv_obj_add_flag(astro_container, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(astro_container, container_click_cb, LV_EVENT_CLICKED, NULL);

    // Create timer for astro clock updates (update every second)
    astro_timer = lv_timer_create(astro_timer_cb, 1000, NULL); // use delay time in config.h
}

void astro_clock_end() {

    // // Stop timer first
    // if (astro_timer) {
    //     lv_timer_pause(astro_timer);
    //     lv_timer_del(astro_timer);
    //     astro_timer = NULL;
    // }
    
    // // Delete container (this deletes all children automatically)
    // if (astro_container && lv_obj_is_valid(astro_container)) {
    //     lv_obj_del(astro_container);
    //     astro_container = NULL;
    // }

    if (astro_timer) {
        lv_timer_pause(astro_timer);
        lv_timer_delete(astro_timer);
        astro_timer = NULL;
    }
    
    // // Reset all object pointers to prevent dangling references
    // // Planet objects
    // sun.obj = NULL; sun.orbit = NULL; sun.target_box = NULL;
    // mercury.obj = NULL; mercury.orbit = NULL; mercury.target_box = NULL;
    // venus.obj = NULL; venus.orbit = NULL; venus.target_box = NULL;
    // earth.obj = NULL; earth.orbit = NULL; earth.target_box = NULL;
    // moon.obj = NULL; moon.orbit = NULL; moon.target_box = NULL;
    // mars.obj = NULL; mars.orbit = NULL; mars.target_box = NULL;
    // jupiter.obj = NULL; jupiter.orbit = NULL; jupiter.target_box = NULL;
    // saturn.obj = NULL; saturn.orbit = NULL; saturn.target_box = NULL;
    // uranus.obj = NULL; uranus.orbit = NULL; uranus.target_box = NULL;
    // neptune.obj = NULL; neptune.orbit = NULL; neptune.target_box = NULL;
    
    // // Other global objects
    // altitde_line = NULL;
    // moon_shadow = NULL;
    // saturn_ring = NULL;
    // target_data_box = NULL;
    // target_connector_line = NULL;
    
    // // Zodiac lines
    // for (int i = 0; i < 12; i++) {
    //     zodiac_lines[i] = NULL;
    // }
    
    // Reset target selection
    current_target = 0;
}
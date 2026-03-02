/** -------------------------------------------------------------------------------------
 * 
 */

#include "bsp/esp32_p4_wifi6_touch_lcd_xc.h"
#include <limits.h>
#include "esp_log.h"
#include "lvgl.h"
// #include "./strval.h"
#include <stdlib.h>       // malloc/free
#include "ff.h"           // FatFs core
#include "diskio.h"       // Disk I/O
#include "diskio_impl.h"  // ESP32 disk impl
#include "esp_vfs_fat.h"  // VFS integration
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include <stdio.h>
#include <dirent.h>
#include "bsp/esp-bsp.h"

#include <stdio.h>
#include <limits.h>
#include <string.h>
// #include <iostream>
#include <assert.h>
#include <float.h>
#include <math.h>
#include <Arduino.h>
// #include <Wire.h>

#include "./config.h"
#include "./REG.h"
#include "./strval.h"
#include "./meteors.h"
#include "./wtgps300p.h"
#include "./wt901.h"
#include "./multiplexers.h"
#include "./sidereal_helper.h"
#include "./hextodig.h"
#include "./ins.h"
#include "./satio.h"
#include "./custommapping.h"
#include "./matrix.h"
#include "./serial_infocmd.h"
#include "./system_data.h"
#include "./sdcard_helper.h"
#include "./task_handler.h"
// #include "./multi_display_controller.h"
#include "./i2c_helper.h"
#include "./wit_c_sdk.h"
#include "./astroclock.h"
#include "./satio_file.h"

#include <sys/time.h>
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "esp_pm.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/uart.h"

#include "./satio_lvgl.h"

/** ---------------------------------------------------------------------------------------
 * @brief Screens
 */

lv_timer_t * display_timer=NULL;

lv_obj_t * loading_screen;
lv_obj_t * home_screen;
lv_obj_t * matrix_screen;
lv_obj_t * gps_screen;
lv_obj_t * gyro_screen;
lv_obj_t * disp_screen;
lv_obj_t * system_screen;
lv_obj_t * _screen;

bool flag_display_loading_screen = false;
bool flag_display_home_screen = false;
bool flag_display_matrix_screen = false;
bool flag_display_gps_screen = false;
bool flag_display_gyro_screen = false;
bool flag_display_disp_screen = false;
bool flag_display_system_screen = false;
bool flag_display__screen = false;

bool update_matrix_screen=false;

/** ---------------------------------------------------------------------------------------
 * @brief Global Objects
 */
lv_obj_t * label_enter_str;
lv_obj_t * text_area_1_obj;

lv_obj_t * label_enter_digits;
lv_obj_t * text_area_2_obj;

lv_obj_t * grid_menu_x;
lv_obj_t * drop_down_menu_x;

menu_struct menu_x;

keyboard_t kb_numdec;
keyboard_t kb_alnumsym;

lv_obj_t * slider_brightness;
int32_t slider_brightness_value = 0;
int32_t slider_outline_width = 2;

title_bar_t main_title_bar;
system_tray_t system_tray;

// Loading screen image (freed when switching to home screen)
static sdcard_image_t * loading_image = NULL;

lv_obj_t * title_bar_local_time_label;
lv_obj_t * title_bar_local_date_label;

lv_obj_t * matrix_overview_grid_1;
lv_obj_t * dropdown_menu_matrix_switch_select;
int current_matrix_i = 0;
int current_matrix_function_i = 0;
matrix_function_container_t mfc;
mapping_config_container_t mcc;

button_t matrix_switch_override;
button_t matrix_switch_computer_assist;
button_t matrix_new;
button_t matrix_save;
button_t matrix_load;
button_t matrix_delete;
lv_obj_t * dd_matrix_slot_select;
lv_obj_t * matrix_switch_output_value;

/** ---------------------------------------------------------------------------------------
 * @brief Global Style
 */
// ---------------------------
// Color
// ---------------------------
uint32_t current_hue=0;
lv_color_t menu_bg_color;
lv_color_t menu_border_color;
lv_color_t menu_outline_color;
lv_color_t menu_shadow_color;
lv_color_t menu_text_color;
lv_color_t system_tray_bg_color;
// ---------------------------
// Size
// ---------------------------
int32_t outline_width = 2;
int32_t border_width  = 0;
int32_t shadow_width  = 0;
// ---------------------------
// Radius
// ---------------------------
int32_t radius_square  = 0;
int32_t radius_rounded = 5;
int32_t radius_circle  = 360;
int32_t general_radius = 0;
// ---------------------------
// Font
// ---------------------------
static const lv_font_t * font_menu_title;
static const lv_font_t * font_menu_item;

/* ----------------------------------------------------------------------------------------
 * @brief Custom LVGL log callback to redirect logs to ESP-IDF logging system.
 * @param level LVGL log level.
 * @param buf Log message buffer.
 */
static void my_lv_log_cb(lv_log_level_t level, const char * buf)
{
    static const char * level_prefix[] = {"TRACE", "INFO", "WARN", "ERROR", "USER"};

    const char * prefix = (level < LV_LOG_LEVEL_NUM) ? level_prefix[level] : "???";

    ESP_LOGI("LVGL", "[%s] %s", prefix, buf);
}

/** ---------------------------------------------------------------------------------------
 * @brief Keyboard Targets
 * 
 * Objects to be set by keyboard(s) should be added here to be enumerated so that we don't
 * need seperate callbacks for wach object created that intends to use a keyboard.
 */
typedef enum {
    KB_TARGET_NONE = 0,
    KB_MATRIX_VALUE_X,
    KB_MATRIX_VALUE_Y,
    KB_MATRIX_VALUE_Z,
    KB_MATRIX_FLUX,
    KB_MATRIX_OUTPUT_PWM_0,
    KB_MATRIX_OUTPUT_PWM_1,
    KB_MATRIX_PORT_MAP,
    KB_MAPPING_C1,
    KB_MAPPING_C2,
    KB_MAPPING_C3,
    KB_MAPPING_C4,
    KB_MAPPING_C5,
    /* ... add other objects as required (does not have to be a lv_textarea) */
} kb_target_t;

/** ---------------------------------------------------------------------------------------
 * @brief Keyboard Context
 * 
 * Struct to hold context for keyboard events. This allows us to use a single event handler
 * across multiple keybords and multiple objects.
 * 
 */
typedef struct {
    kb_target_t target;
    strval_type_t strval_type;  // Which strval function to use (enumerated by strval)
} kb_ctx_t;

/** ---------------------------------------------------------------------------------------
 * @brief Create a context for each object that intends to use a keyboard.
 * 
 * Values will be enumerated by set_keyboard_context_cb & keyboard_event_cb.
 * 
 * 1: Add entry to kb_target_t enum for the object with some human name.
 * 2: Create a kb_ctx_t struct below specifying the kb_target_t and strval_type_t.
 * 3: Link the context to the object via user data (lv_obj_set_user_data) when creating the object.
 * 4: In set_keyboard_context_cb, add a case to determine which keyboard to use for object.
 * 5: In keyboard_event_cb, add a case to determine what to do with the input text.
 */
static kb_ctx_t matrix_value_x_ctx = { .target = KB_MATRIX_VALUE_X, .strval_type = STRVAL_DOUBLE };
static kb_ctx_t matrix_value_y_ctx = { .target = KB_MATRIX_VALUE_Y, .strval_type = STRVAL_DOUBLE };
static kb_ctx_t matrix_value_z_ctx = { .target = KB_MATRIX_VALUE_Z, .strval_type = STRVAL_DOUBLE };
static kb_ctx_t matrix_flux_ctx = { .target = KB_MATRIX_FLUX, .strval_type = STRVAL_UINT32 };
static kb_ctx_t matrix_output_pwm_0_ctx = { .target = KB_MATRIX_OUTPUT_PWM_0, .strval_type = STRVAL_UINT32 };
static kb_ctx_t matrix_output_pwm_1_ctx = { .target = KB_MATRIX_OUTPUT_PWM_1, .strval_type = STRVAL_UINT32 };
static kb_ctx_t matrix_port_map_ctx = { .target = KB_MATRIX_PORT_MAP, .strval_type = STRVAL_INT8 };

static kb_ctx_t mapping_c1_ctx = { .target = KB_MAPPING_C1, .strval_type = STRVAL_INT32 };
static kb_ctx_t mapping_c2_ctx = { .target = KB_MAPPING_C2, .strval_type = STRVAL_INT32 };
static kb_ctx_t mapping_c3_ctx = { .target = KB_MAPPING_C3, .strval_type = STRVAL_INT32 };
static kb_ctx_t mapping_c4_ctx = { .target = KB_MAPPING_C4, .strval_type = STRVAL_INT32 };
static kb_ctx_t mapping_c5_ctx = { .target = KB_MAPPING_C5, .strval_type = STRVAL_INT32 };

/* ... add other contexts as required (does not have to be a lv_textarea) */

/** ---------------------------------------------------------------------------------------
 * @brief Set keyboard context and show a keyboard.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void set_keyboard_context_cb(lv_event_t * e)
{
    // Get event code
    lv_event_code_t code = lv_event_get_code(e);
    if(code != LV_EVENT_FOCUSED && code != LV_EVENT_CLICKED) return;
    printf("[set_keyboard_context_cb] Textarea Event Code: %d\n", code);

    // Get event target (Which textarea was clicked?)
    lv_obj_t * target_obj = (lv_obj_t *)lv_event_get_target(e);
    printf("[set_keyboard_context_cb] Target Object: %p\n", (void*)target_obj);
    
    // Get context to know which keyboard to use
    kb_ctx_t * ctx = (kb_ctx_t *)lv_obj_get_user_data(target_obj);
    printf("[set_keyboard_context_cb] Object Context Target: %d\n", ctx ? ctx->target : -1);
    if (!ctx) {printf("[set_keyboard_context_cb] No context found for object.\n"); return;}

    // Determine which keyboard to use based on context
    keyboard_t * kb;
    switch(ctx->target) {

        case KB_MATRIX_VALUE_X: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for MATRIX_VALUE_X\n"); break;
        case KB_MATRIX_VALUE_Y: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for MATRIX_VALUE_Y\n"); break;
        case KB_MATRIX_VALUE_Z: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for MATRIX_VALUE_Z\n"); break;
        case KB_MATRIX_FLUX: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for MATRIX_FLUX\n"); break;
        case KB_MATRIX_OUTPUT_PWM_0: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for MATRIX_OUTPUT_PWM_0\n"); break;
        case KB_MATRIX_OUTPUT_PWM_1: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for MATRIX_OUTPUT_PWM_1\n"); break;
        case KB_MATRIX_PORT_MAP: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for MATRIX_PORT_MAP\n"); break;

        case KB_MAPPING_C1: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for KB_MAPPING_C1\n"); break;
        case KB_MAPPING_C2: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for KB_MAPPING_C2\n"); break;
        case KB_MAPPING_C3: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for KB_MAPPING_C3\n"); break;
        case KB_MAPPING_C4: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for KB_MAPPING_C4\n"); break;
        case KB_MAPPING_C5: kb = &kb_numdec; printf("[set_keyboard_context_cb] Using numeric keyboard for KB_MAPPING_C5\n"); break;

        /* ... add other cases as required */
        default: printf("[set_keyboard_context_cb] Unknown keyboard target: %d\n", ctx->target); return;
    }

    // Link the clicked textarea to the keyboard for later retrieval in the keyboard event handler
    lv_obj_set_user_data(kb->ta, target_obj);
    printf("[set_keyboard_context_cb] Linked keyboard TA: %p to object: %p\n", (void*)kb->ta, (void*)target_obj);
    
    // Show keyboard
    printf("[set_keyboard_context_cb] Bringing keyboard to foreground.\n");
    lv_obj_move_foreground(kb->kb);
    lv_obj_move_foreground(kb->ta);
    lv_obj_clear_flag(kb->kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(kb->ta, LV_OBJ_FLAG_HIDDEN);
}

/** ---------------------------------------------------------------------------------------
 * @brief Event callback for keyboard events.
 * 
 * Intended to be used across multiple keyboards by linking the relevant object to a
 * related keyboard via user data.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void keyboard_event_cb(lv_event_t * e)
{
    // Get event code
    lv_event_code_t code = lv_event_get_code(e);
    printf("[keyboard_event_cb] Keyboard Event Code: %d\n", code);
    if(code != LV_EVENT_VALUE_CHANGED) {
        printf("[keyboard_event_cb] Event code is not VALUE_CHANGED, returning.\n");
        return;
    }

    // Get event target
    lv_obj_t * kb_target = (lv_obj_t *)lv_event_get_target(e);
    printf("[keyboard_event_cb] Keyboard Target: %p\n", (void*)kb_target);
    keyboard_t * kb_user_data = (keyboard_t *)lv_event_get_user_data(e);
    printf("[keyboard_event_cb] Keyboard User Data: %p\n", (void*)kb_user_data);

    // Ensure the event is from an expected keyboard
    if(kb_target != kb_user_data->kb) {
        printf("[keyboard_event_cb] Event target does not match keyboard in user data, returning.\n");
        return;
    }

    // Get the text of the selected object
    uint32_t obj_id = lv_keyboard_get_selected_btn(kb_user_data->kb);
    printf("[keyboard_event_cb] Selected Button ID: %lu\n", obj_id);
    const char * obj_text = lv_keyboard_get_btn_text(kb_user_data->kb, obj_id);
    printf("[keyboard_event_cb] Selected Button Text: %s\n", obj_text);

    // Only proceed if "OK", "Enter", or the LV_SYMBOL_OK button was pressed
    if(strcmp(obj_text, LV_SYMBOL_OK) != 0 &&
       strcmp(obj_text, "Enter") != 0 &&
       strcmp(obj_text, "OK") != 0) {return;}

    // Get the designated textarea that triggered this keyboard
    lv_obj_t * designated_ta = (lv_obj_t *)lv_obj_get_user_data(kb_user_data->ta);
    printf("[keyboard_event_cb] Designated Textarea from User Data: %p\n", (void*)designated_ta);
    if(!designated_ta) {
        printf("[keyboard_event_cb] No designated textarea found in user data, returning.\n");
        return;
    }

    // Get context from designated textarea
    kb_ctx_t * ctx = (kb_ctx_t *)lv_obj_get_user_data(designated_ta);
    printf("[keyboard_event_cb] Context Target from Designated Textarea: %d\n", ctx ? ctx->target : -1);

    // Get the input text from the keyboard's textarea
    const char * input = lv_textarea_get_text(kb_user_data->ta);
    printf("[keyboard_event_cb] Input Text from Keyboard Textarea: %s\n", input);

    // Route to correct destination using context
    switch (ctx ? ctx->target : KB_TARGET_NONE) {

        case KB_MATRIX_VALUE_X:
            if (strval_validate(ctx->strval_type, input)) {
                double val = strtod(input, NULL);
                printf("[keyboard_event_cb] Setting MATRIX_VALUE_X to: %f\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_X] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid double: %s\n", input);
            }
            break;

        case KB_MATRIX_VALUE_Y:
            if (strval_validate(ctx->strval_type, input)) {
                double val = strtod(input, NULL);
                printf("[keyboard_event_cb] Setting MATRIX_VALUE_Y to: %f\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_Y] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid double: %s\n", input);
            }
            break;
        
        case KB_MATRIX_VALUE_Z:
            if (strval_validate(ctx->strval_type, input)) {
                double val = strtod(input, NULL);
                printf("[keyboard_event_cb] Setting MATRIX_VALUE_Z to: %f\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_Z] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid double: %s\n", input);
            }
            break;
        
        case KB_MATRIX_FLUX:
            if (strval_validate(ctx->strval_type, input)) {
                uint32_t val = strtoul(input, NULL, 10);
                printf("[keyboard_event_cb] Setting MATRIX_FLUX to: %lu\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                matrixData.flux_value[0][current_matrix_i] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid uint32_t: %s\n", input);
            }
            break;
        
        case KB_MATRIX_OUTPUT_PWM_0:
            if (strval_validate(ctx->strval_type, input)) {
                uint32_t val = strtoul(input, NULL, 10);
                printf("[keyboard_event_cb] Setting MATRIX_OUTPUT_PWM_0 to: %lu\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                matrixData.output_pwm[0][current_matrix_i][INDEX_MATRIX_SWITCH_PWM_OFF] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid uint32_t: %s\n", input);
            }
            break;
        
        case KB_MATRIX_OUTPUT_PWM_1:
            if (strval_validate(ctx->strval_type, input)) {
                uint32_t val = strtoul(input, NULL, 10);
                printf("[keyboard_event_cb] Setting MATRIX_OUTPUT_PWM_1 to: %lu\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                matrixData.output_pwm[0][current_matrix_i][INDEX_MATRIX_SWITCH_PWM_ON] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid uint32_t: %s\n", input);
            }
            break;
        
        case KB_MATRIX_PORT_MAP:
            if (strval_validate(ctx->strval_type, input)) {
                int8_t val = atoi(input);
                printf("[keyboard_event_cb] Setting MATRIX_PORT_MAP to: %d\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                matrixData.matrix_port_map[0][current_matrix_i] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid int8_t: %s\n", input);
            }
            break;
        
        case KB_MAPPING_C1:
            if (strval_validate(ctx->strval_type, input)) {
                int32_t val = strtol(input, NULL, 10);
                printf("[keyboard_event_cb] Setting KB_MAPPING_C1 to: %ld\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                mappingData.mapping_config[0][matrixData.index_mapped_value[0][current_matrix_i]][INDEX_MAP_C1] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid int32_t: %s\n", input);
            }
            break;
        
        case KB_MAPPING_C2:
            if (strval_validate(ctx->strval_type, input)) {
                int32_t val = strtol(input, NULL, 10);
                printf("[keyboard_event_cb] Setting KB_MAPPING_C2 to: %ld\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                mappingData.mapping_config[0][matrixData.index_mapped_value[0][current_matrix_i]][INDEX_MAP_C2] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid int32_t: %s\n", input);
            }
            break;

        case KB_MAPPING_C3:
            if (strval_validate(ctx->strval_type, input)) {
                int32_t val = strtol(input, NULL, 10);
                printf("[keyboard_event_cb] Setting KB_MAPPING_C3 to: %ld\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                mappingData.mapping_config[0][matrixData.index_mapped_value[0][current_matrix_i]][INDEX_MAP_C3] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid int32_t: %s\n", input);
            }
            break;

        case KB_MAPPING_C4:
            if (strval_validate(ctx->strval_type, input)) {
                int32_t val = strtol(input, NULL, 10);
                printf("[keyboard_event_cb] Setting KB_MAPPING_C4 to: %ld\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                mappingData.mapping_config[0][matrixData.index_mapped_value[0][current_matrix_i]][INDEX_MAP_C4] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid int32_t: %s\n", input);
            }
            break;

        case KB_MAPPING_C5:
            if (strval_validate(ctx->strval_type, input)) {
                int32_t val = strtol(input, NULL, 10);
                printf("[keyboard_event_cb] Setting KB_MAPPING_C5 to: %ld\n", val);
                // lv_textarea_set_text(...); // Update relevant object if needed
                mappingData.mapping_config[0][matrixData.index_mapped_value[0][current_matrix_i]][INDEX_MAP_C5] = val;
            }
            else {
                printf("[keyboard_event_cb] Input is not a valid int32_t: %s\n", input);
            }
            break;
        
        default:
            printf("[keyboard_event_cb] No case for context target: %d\n", ctx->target);
            break;
    }

    // Cleanup
    lv_textarea_set_text(kb_user_data->ta, "");
    lv_obj_add_flag(kb_user_data->kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(kb_user_data->ta, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_user_data(kb_user_data->ta, NULL);  // Clear tracking
}

/** ---------------------------------------------------------------------------------------
 * @brief Event callback for ALL menu_x child objects (buttons, items, containers).
 * @param e Pointer to the LVGL event structure.
 */
static void menu_x_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if(code == LV_EVENT_CLICKED) {
        lv_obj_t * obj = (lv_obj_t *)lv_event_get_target(e);     // Get clicked object
        lv_obj_t * label = lv_obj_get_child(obj, 0); // Get object's label (if any)
        
        // Find which page this object belongs to
        int page_index = -1;
        for(int p = 0; p < menu_x.page_count; p++) {
            uint32_t child_cnt = lv_obj_get_child_cnt(menu_x.pages[p]);
            for(uint32_t i = 0; i < child_cnt; i++) {
                if(lv_obj_get_child(menu_x.pages[p], i) == obj) {
                    page_index = p;
                    uint32_t item_index = i;
                    
                    const char * text = label ? lv_label_get_text(label) : "no label";
                    printf("Menu X: Page %d, Item %lu (%s) clicked!\n", 
                           page_index, item_index, text);
                    
                    // Your custom logic
                    // handle_menu_item_click(page_index, item_index, obj);
                    return;
                }
            }
        }
        
        printf("Menu X: Unknown object clicked\n");
    }
}

/** ---------------------------------------------------------------------------------------
 * @brief Event callback for grid_menu object click event.
 * @param e Pointer to the LVGL event structure.
 */
static void system_tray_grid_menu_1_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {

        lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);     // Get clicked button
        lv_obj_t * grid = (lv_obj_t *)lv_event_get_user_data(e); // Get grid container (if passed)
        lv_obj_t * label = lv_obj_get_child(btn, 0); // Get button's label
        
        uint32_t btn_index = 0;
        uint32_t child_cnt = lv_obj_get_child_cnt(system_tray.grid_menu_1);
        for(uint32_t i = 0; i < child_cnt; i++) {
            if(lv_obj_get_child(system_tray.grid_menu_1, i) == btn) {
                btn_index = i;
                break;
            }
        }
        
        const char * text = lv_label_get_text(label);
        printf("System Tray Grid Menu 1: Button %lu (%s) clicked!\n", btn_index, text);
        
        // Switch logic
        switch(btn_index) {
            case 0:  flag_display_home_screen=true; break;
            case 1:  flag_display_matrix_screen=true; break;
            case 2:  flag_display_gps_screen=true; break;
            case 3:  flag_display_gyro_screen=true; break;
            case 4:  flag_display_disp_screen=true; break;
            case 5:  flag_display_system_screen=true; break;
            case 6:  flag_display__screen=true; break;
            default: break;
        }
    }
}

/** ---------------------------------------------------------------------------------------
 * @brief Event callback for grid_menu_x object click event.
 * @param e Pointer to the LVGL event structure.
 */
static void matrix_overview_grid_1_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {

        lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);     // Get clicked button
        lv_obj_t * grid = (lv_obj_t *)lv_event_get_user_data(e); // Get grid container (if passed)
        lv_obj_t * label = lv_obj_get_child(btn, 0); // Get button's label
        
        uint32_t btn_index = 0;
        uint32_t child_cnt = lv_obj_get_child_cnt(matrix_overview_grid_1);
        for(uint32_t i = 0; i < child_cnt; i++) {
            if(lv_obj_get_child(matrix_overview_grid_1, i) == btn) {
                btn_index = i;
                break;
            }
        }
        
        const char * text = lv_label_get_text(label);
        printf("Matrix Overview Grid 1: Button %lu (%s) clicked!\n", btn_index, text);
        
        if (btn_index <= 44) {current_matrix_i = btn_index;}
    }
}

/** ---------------------------------------------------------------------------------------
 * @brief Event callback for drop_down_menu_x object click event.
 * @param e Pointer to the LVGL event structure.
 */
static void dropdown_menu_x_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if(code == LV_EVENT_CLICKED) {
        lv_obj_t * option = (lv_obj_t *)lv_event_get_target(e);
        lv_obj_t * dropdown = (lv_obj_t *)lv_event_get_user_data(e);
        lv_obj_t * label = lv_obj_get_child(option, 0);
        
        uint32_t option_index = 0;
        uint32_t child_cnt = lv_obj_get_child_cnt(drop_down_menu_x);
        for(uint32_t i = 0; i < child_cnt; i++) {
            if(lv_obj_get_child(drop_down_menu_x, i) == option) {
                option_index = i;
                break;
            }
        }
        
        const char * text = lv_label_get_text(label);
        printf("Dropdown: Option %lu (%s) clicked!\n", option_index, text);
        
        // Custom logic
        switch(option_index) {
            case 0: /* Option 1 */ break;
            case 1: /* Option 2 */ break;
            // ...
        }
    }
}


/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_function_index_select dropdown.
 * 
 * Sets current_matrix_function_i based on dropdown selection.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_function_index_select_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * dd = (lv_obj_t *)lv_event_get_target(e);
        uint32_t sel = lv_dropdown_get_selected(dd);
        current_matrix_function_i = (int)sel;
        printf("[dd_function_index_select_event_cb] Function index set to: %lu\n", sel);
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_function_name dropdown.
 * 
 * Sets matrixData.matrix_function based on dropdown selection.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_function_name_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * dd = (lv_obj_t *)lv_event_get_target(e);
        uint32_t sel = lv_dropdown_get_selected(dd);
        matrixData.matrix_function[0][current_matrix_i][current_matrix_function_i] = (int)sel;
        printf("[dd_function_name_event_cb] Function set to: %lu\n", sel);
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_c0_event_cb dropdown.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_c0_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * dd = (lv_obj_t *)lv_event_get_target(e);
        uint32_t sel = lv_dropdown_get_selected(dd);
        mappingData.mapping_config[0][matrixData.index_mapped_value[0][current_matrix_i]][INDEX_MAP_C0] = (int)sel;
        printf("[dd_c0_event_cb] Set map function: %lu\n", sel);
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_mode_event_cb dropdown.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_mode_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * dd = (lv_obj_t *)lv_event_get_target(e);
        uint32_t sel = lv_dropdown_get_selected(dd);
        mappingData.map_mode[0][matrixData.index_mapped_value[0][current_matrix_i]] = (int)sel;
        printf("[dd_mode_event_cb] Set map mode: %lu\n", sel);
    }
}


/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_operator dropdown.
 * 
 * Sets matrixData.matrix_switch_operator_index based on dropdown selection.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_operator_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * dd = (lv_obj_t *)lv_event_get_target(e);
        uint32_t sel = lv_dropdown_get_selected(dd);
        matrixData.matrix_switch_operator_index[0][current_matrix_i][current_matrix_function_i] = (int)sel;
        printf("[dd_operator_event_cb] Operator set to: %lu\n", sel);
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_output_mode dropdown.
 * 
 * Sets matrixData.output_mode based on dropdown selection.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_output_mode_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * dd = (lv_obj_t *)lv_event_get_target(e);
        uint32_t sel = lv_dropdown_get_selected(dd);
        matrixData.output_mode[0][current_matrix_i] = (int)sel;
        printf("[dd_output_mode_event_cb] Output mode set to: %lu\n", sel);
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_matrix_slot_select_event_cb dropdown.
 * 
 * Sets current matrix file path based on dropdown selection.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_matrix_slot_select_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath));
        lv_dropdown_get_selected_str(dd_matrix_slot_select, satioFileData.current_matrix_filepath, sizeof(satioFileData.current_matrix_filepath));
        // strcpy(satioFileData.current_matrix_filepath, lv_dropdown_get_selected_str(dd_matrix_slot_select));
        printf("[dd_matrix_slot_select_event_cb] Matrix slot set to: %s\n", satioFileData.current_matrix_filepath);
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_map_slot_event_cb dropdown.
 * 
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_map_slot_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * dd = (lv_obj_t *)lv_event_get_target(e);
        uint32_t sel = lv_dropdown_get_selected(dd);
        matrixData.index_mapped_value[0][current_matrix_i] = (int)sel;
        printf("[dd_output_mode_event_cb] Matrix %d will use map slot: %ld\n", current_matrix_i, sel);
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for matrix_new_event_cb switch.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void matrix_new_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if
    (
        code == LV_EVENT_CLICKED ||
        code == LV_EVENT_PRESSED ||
        code == LV_EVENT_RELEASED
    ) {printf("[matrix_new_event_cb] event code: %d\n", code);}

    if(code == LV_EVENT_CLICKED) {
        printf("[matrix_new_event_cb] Setting new matrix.\n");
        override_all_computer_assists();
        set_all_matrix_default();
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for matrix_save_event_cb switch.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void matrix_save_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if
    (
        code == LV_EVENT_CLICKED ||
        code == LV_EVENT_PRESSED ||
        code == LV_EVENT_RELEASED
    ) {printf("[matrix_save_event_cb] event code: %d\n", code);}

    if(code == LV_EVENT_CLICKED) {
        printf("[matrix_save_event_cb] Saving matrix to slot: %s\n", satioFileData.current_matrix_filepath);
        sdcardFlagData.save_matrix=true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sdcardFlagData.save_mapping=true;
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for matrix_load_event_cb switch.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void matrix_load_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if
    (
        code == LV_EVENT_CLICKED ||
        code == LV_EVENT_PRESSED ||
        code == LV_EVENT_RELEASED
    ) {printf("[matrix_load_event_cb] event code: %d\n", code);}

    if(code == LV_EVENT_CLICKED) {
        printf("[matrix_load_event_cb] Loading matrix from slot: %s\n", satioFileData.current_matrix_filepath);
        sdcardFlagData.load_mapping=true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sdcardFlagData.load_matrix=true;
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for matrix_delete_event_cb switch.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void matrix_delete_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if
    (
        code == LV_EVENT_CLICKED ||
        code == LV_EVENT_PRESSED ||
        code == LV_EVENT_RELEASED
    ) {printf("[matrix_delete_event_cb] event code: %d\n", code);}

    if(code == LV_EVENT_CLICKED) {
        printf("[matrix_delete_event_cb] Deleting matrix in slot: %s\n", satioFileData.current_matrix_filepath);
        sdcardFlagData.delete_matrix=true;
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for dd_inverted_logic_event_cb switch.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void dd_inverted_logic_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * dd = (lv_obj_t *)lv_event_get_target(e);
        uint32_t sel = lv_dropdown_get_selected(dd);
        matrixData.matrix_switch_inverted_logic[0][current_matrix_i][current_matrix_function_i] = (int)sel;
        printf("[dd_inverted_logic_event_cb] Inverted logic set to: %s\n", (int)sel ? "true" : "false");
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for current_matrix_computer_assist_event_cb switch.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void current_matrix_computer_assist_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if
    (
        code == LV_EVENT_CLICKED ||
        code == LV_EVENT_PRESSED ||
        code == LV_EVENT_RELEASED
    ) {printf("[current_matrix_computer_assist_event_cb] event code: %d\n", code);}

    if(code == LV_EVENT_CLICKED) {
        bool toggle = !matrixData.computer_assist[0][current_matrix_i];
        matrixData.computer_assist[0][current_matrix_i] = toggle;
        printf("[current_matrix_computer_assist_event_cb] Setting computer assist for matrix switch %d to: %s\n", current_matrix_i, toggle ? "true" : "false");
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Event callback for current_matrix_override_off_event_cb switch.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void current_matrix_override_off_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if
    (
        code == LV_EVENT_CLICKED ||
        code == LV_EVENT_PRESSED ||
        code == LV_EVENT_RELEASED
    ) {printf("[current_matrix_override_off_event_cb] event code: %d\n", code);}

    if(code == LV_EVENT_CLICKED) {
        printf("[current_matrix_override_off_event_cb] OVERRIDING matrix switch %d\n", current_matrix_i);
        setOverrideOutputValue((int)current_matrix_i, (uint32_t)0);
    }
}

/** -------------------------------------------------------------------------------------
 * @brief Create Slider
 */
lv_obj_t * create_slider(
    lv_obj_t * scr,
    int32_t size_w_px,
    int32_t size_h_px,
    int32_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t range_min,
    int32_t range_max,
    int32_t range_value
    )
{
    /*----------------------------------------------- SLIDER -----------------------------------------------*/

    // Create slider
    lv_obj_t * slider = lv_slider_create(scr);

    // Set range and initial value
    lv_slider_set_range(slider, range_min, range_max);
    lv_slider_set_value(slider, range_value, LV_ANIM_OFF);

    // Size and position
    lv_obj_set_size(slider, size_w_px, size_h_px);
    lv_obj_align(slider, (lv_align_t)alignment, pos_x, pos_y);

    /*---------------------------------------- SLIDER LV_PART_MAIN -----------------------------------------*/

    // Main style: radius
    lv_obj_set_style_radius(slider, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(slider, slider_outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(slider, menu_outline_color, LV_PART_MAIN);
    
    // Main style: border
    lv_obj_set_style_border_width(slider, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(slider, menu_border_color, LV_PART_MAIN);
    
    // Main style: background
    lv_obj_set_style_bg_color(slider, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(slider, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(slider, menu_shadow_color, LV_PART_MAIN);

    // ---------------------------------------- SLIDER LV_PART_INDICATOR -----------------------------------------*/

    // Indicator style: outline
    lv_obj_set_style_outline_width(slider, slider_outline_width, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(slider, menu_outline_color, LV_PART_INDICATOR);
    
    // Indicator style: border
    lv_obj_set_style_border_width(slider, border_width, LV_PART_INDICATOR);
    lv_obj_set_style_border_color(slider, menu_border_color, LV_PART_INDICATOR);

    // Indicator style: background
    lv_obj_set_style_bg_color(slider, menu_bg_color, LV_PART_INDICATOR);

    // Indicator style: shadow
    lv_obj_set_style_shadow_width(slider, shadow_width, LV_PART_INDICATOR);
    lv_obj_set_style_shadow_color(slider, menu_shadow_color, LV_PART_INDICATOR);

    // Indicator style: radius (set last to square off indicator so main outline does not bleed though on left edge)
    lv_obj_set_style_radius(slider, general_radius, LV_PART_INDICATOR);

    // ----------------------------------------- SLIDER LV_PART_KNOB -----------------------------------------*/

    // Indicator style: radius
    lv_obj_set_style_radius(slider, radius_circle, LV_PART_KNOB);

    // Knob style: outline
    lv_obj_set_style_outline_width(slider, slider_outline_width, LV_PART_KNOB);
    lv_obj_set_style_outline_color(slider, menu_outline_color, LV_PART_KNOB);
    
    // Knob style: border
    lv_obj_set_style_border_width(slider, border_width, LV_PART_KNOB);
    lv_obj_set_style_border_color(slider, menu_border_color, LV_PART_KNOB);

    // Knob style: background
    lv_obj_set_style_bg_color(slider, menu_bg_color, LV_PART_KNOB);

    // Knob style: shadow
    lv_obj_set_style_shadow_width(slider, shadow_width, LV_PART_KNOB);
    lv_obj_set_style_shadow_color(slider, menu_shadow_color, LV_PART_KNOB);


    return slider;   
}

/** ---------------------------------------------------------------------------------------
 * @brief Event callback for brightness slider value change.
 * 
 * Updates the global brightness value and applies it to the display.
 * 
 * @param e Pointer to the LVGL event structure.
 */
void slider_brightness_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
    slider_brightness_value = lv_slider_get_value(slider);
    printf("Brightness set to: %lu\n", slider_brightness_value);
    bsp_display_brightness_set(slider_brightness_value);
}

/** ---------------------------------------------------------------------------------------
 * @brief Event callback for system tray swipe gestures.
 * 
 * Swipe down from top to open, swipe up from anywhere to close.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void tray_close_ready_cb(lv_anim_t * a)
{
    lv_obj_t * panel = (lv_obj_t *)a->var;
    lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN);
}

/** ---------------------------------------------------------------------------------------
 * @brief Event callback for system tray swipe gestures.
 * 
 * Add gestures here as required.
 * 
 * @param e Pointer to the LVGL event structure.
 */
static void screen_swipe_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t *)lv_event_get_target(e);
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());

    // Get gesture start position
    lv_point_t gesture_point;
    lv_indev_get_point(lv_indev_get_act(), &gesture_point);
    printf("[screen_swipe_cb] x=%lu, y=%lu\n", gesture_point.x, gesture_point.y);

    // ---- Handle special cases first ----

    // Hide keyboard if gesture outside keyboard area
    // if (
    //         target != kb_alnumsym.kb &&
    //         target != kb_alnumsym.ta &&
    //         target != kb_numdec.kb   &&
    //         target != kb_numdec.ta
    //     ) {
    //     printf("[screen_swipe_cb] attempting to hide keyboard(s)\n");
    //     lv_obj_add_flag(kb_alnumsym.kb, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_add_flag(kb_alnumsym.ta, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_add_flag(kb_numdec.kb, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_add_flag(kb_numdec.ta, LV_OBJ_FLAG_HIDDEN);
    // }

    // ---- Handle gestures ----
    
    // Gesture: Swipe down from top of screen
    if(dir == LV_DIR_BOTTOM && gesture_point.y < 200) {
        if(!system_tray.is_open) {
            printf("[screen_swipe_cb] Swipe down detected, opening system tray.\n");
            lv_obj_clear_flag(system_tray.panel, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(system_tray.panel);  // Ensure on top of all objects
            lv_obj_set_y(system_tray.panel, -290);
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, system_tray.panel);
            lv_anim_set_values(&a, -290, 0);
            lv_anim_set_time(&a, 200);
            lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
            lv_anim_start(&a);
            
            system_tray.is_open = true;
        }
    }
    // Gesture: Swipe up from anywhere on screen
    else if(dir == LV_DIR_TOP && system_tray.is_open) {
        // Animate slide up
        printf("[screen_swipe_cb] Swipe up detected, closing system tray.\n");
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, system_tray.panel);
        lv_anim_set_values(&a, 0, -290);
        lv_anim_set_time(&a, 200);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
        lv_anim_set_ready_cb(&a, tray_close_ready_cb);  // Hide when animation finishes
        lv_anim_start(&a);
        
        system_tray.is_open = false;
    }
}

static void screen_tap_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * target = (lv_obj_t *)lv_event_get_target(e);
    if(code != LV_EVENT_CLICKED) return;

    // Get gesture start position
    lv_point_t gesture_point;
    lv_indev_get_point(lv_indev_get_act(), &gesture_point);
    printf("[screen_tap_cb] x=%lu, y=%lu\n", gesture_point.x, gesture_point.y);
    
    // ---- Handle special cases first ----

    // Hide keyboard if gesture outside keyboard area
    // if (
    //         target != kb_alnumsym.kb &&
    //         target != kb_alnumsym.ta &&
    //         target != kb_numdec.kb   &&
    //         target != kb_numdec.ta
    //     ) {
    //     printf("[screen_tap_cb] attempting to hide keyboard(s)\n");
    //     lv_obj_add_flag(kb_alnumsym.kb, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_add_flag(kb_alnumsym.ta, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_add_flag(kb_numdec.kb, LV_OBJ_FLAG_HIDDEN);
    //     lv_obj_add_flag(kb_numdec.ta, LV_OBJ_FLAG_HIDDEN);
    // }

    // // Hide system tray if gesture outside system tray
    // if (system_tray.is_open && target != system_tray.panel) {
    //     printf("[screen_tap_cb] attempting to hide system tray\n");
    //     lv_obj_add_flag(system_tray.panel, LV_OBJ_FLAG_HIDDEN);
    //     system_tray.is_open = false;
    // }
}

/** ---------------------------------------------------------------------------------------
 * Title Bar
 */
title_bar_t create_title_bar(
    lv_obj_t * scr,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    bool show_scrollbar,
    bool enable_scrolling
    )
{
    // Initialize struct
    title_bar_t title_bar = {0};
    
    // Create title bar
    title_bar.panel = lv_obj_create(scr);

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(title_bar.panel, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(title_bar.panel, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(title_bar.panel, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(title_bar.panel, LV_DIR_NONE);}

    // Size and position
    lv_obj_set_size(title_bar.panel, size_w_px, size_h_px);
    lv_obj_align(title_bar.panel, alignment, pos_x, pos_y);

    /* ------------------------------- LV_PART_MAIN -------------------------------- */

    // Main style: radius
    lv_obj_set_style_radius(title_bar.panel, general_radius, LV_PART_MAIN);
    
    // Main style: outline
    lv_obj_set_style_outline_width(title_bar.panel, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(title_bar.panel, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(title_bar.panel, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(title_bar.panel, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(title_bar.panel, system_tray_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(title_bar.panel, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(title_bar.panel, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(title_bar.panel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(title_bar.panel, font_menu_item, LV_PART_MAIN);
    lv_obj_set_style_text_color(title_bar.panel, menu_text_color, LV_PART_MAIN);

    // -------------------------------- Objects --------------------------------- //

    // Time
    title_bar.time_label = create_label(
        title_bar.panel,      // parent
        128,                  // width
        24,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        0,                    // pos x
        0,                    // pos y
        "00:00:00",           // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_25,     // font
        true,                 // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        2,                    // outline width
        general_radius,       // outline radius
        1
    );

    // Date
    title_bar.date_label = create_label(
        title_bar.panel,      // parent
        128,                  // width
        24,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        0,                    // pos x
        24,                   // pos y
        "00/00/00",           // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_25,     // font
        true,                 // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        2,                    // outline width
        general_radius,       // outline radius
        1
    );

    // Datetime Sync
    title_bar.datetime_sync = create_label(
        title_bar.panel,      // parent
        90,                   // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        120,                  // pos x
        24,                   // pos y
        "GPS SYNC",           // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_17,     // font
        false,                // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    // GPS Signal
    title_bar.gps_signal_strength = create_label(
        title_bar.panel,      // parent
        90,                   // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        120,                  // pos x
        24,                   // pos y
        "0:0",                // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_17,     // font
        false,                // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    // SDCard Status
    title_bar.sdcard_mounted = create_label(
        title_bar.panel,      // parent
        40,                   // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        -140,                 // pos x
        24,                   // pos y
        "SD",                 // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_17,     // font
        false,                // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    return title_bar;
}

/** ---------------------------------------------------------------------------------------
 * @brief Create System Tray
 * 
 * Creates a system tray panel that slides down from the top of the screen when swiped down and hides when swiped up.
 * Contains a brightness slider as an example item.
 * 
 * @param scr Pointer to the parent screen object.
 * @return A struct containing references to the system tray objects and state.
 */
system_tray_t create_system_tray(lv_obj_t * scr)
{
    
    /* ------------------------------------ TRAY --------------------------------------- */

    // Initialize struct
    system_tray_t tray = {0};
    
    // Create system tray
    tray.panel = lv_obj_create(scr);

    // Size and position
    lv_obj_set_size(tray.panel, 720, 300);
    lv_obj_align(tray.panel, LV_ALIGN_TOP_MID, 0, -290);  // Start off-screen
    lv_obj_add_flag(tray.panel, LV_OBJ_FLAG_HIDDEN);
    tray.is_open = false;

    /* ------------------------------- TRAY LV_PART_MAIN -------------------------------- */

    // Main style: radius
    lv_obj_set_style_radius(tray.panel, general_radius, LV_PART_MAIN);
    
    // Main style: outline
    lv_obj_set_style_outline_width(tray.panel, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(tray.panel, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(tray.panel, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(tray.panel, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(tray.panel, system_tray_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(tray.panel, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(tray.panel, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(tray.panel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(tray.panel, font_menu_item, LV_PART_MAIN);
    lv_obj_set_style_text_color(tray.panel, menu_text_color, LV_PART_MAIN);

    /* ---------------------------------- TRAY BRIGHTNESS ------------------------------- */
    
    // Create brightness slider
    tray.slider_brightness = create_slider(
        tray.panel,             // parent: tray panel
        200,                    // width
        8,                      // height  
        LV_ALIGN_BOTTOM_MID,    // alignment
        0,                      // x offset
        0,                      // y offset
        21,                     // min value
        100,                    // max value
        slider_brightness_value // initial value
    );
    
    // Add brightness slider event callback
    lv_obj_add_event_cb(tray.slider_brightness, slider_brightness_event_cb, 
                       LV_EVENT_VALUE_CHANGED, NULL);
    
    // -------------------------------- Objects --------------------------------- //

    // Time
    tray.local_time = create_label(
        tray.panel,           // parent
        128,                  // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        0,                    // pos x
        10,                   // pos y
        "00:00:00",           // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_25,     // font
        true,                 // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    // Date
    tray.local_date = create_label(
        tray.panel,           // parent
        128,                  // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        0,                    // pos x
        30,                   // pos y
        "00/00/00",           // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_25,     // font
        true,                 // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    // Human Date
    tray.human_date = create_label(
        tray.panel,           // parent
        340,                  // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        0,                    // pos x
        50,                   // pos y
        "",                   // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_17,     // font
        true,                 // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    // Datetime Sync
    tray.datetime_sync = create_label(
        tray.panel,      // parent
        90,                   // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        120,                  // pos x
        24,                   // pos y
        "GPS SYNC",           // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_17,     // font
        false,                // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    // GPS Signal
    tray.gps_signal_strength = create_label(
        tray.panel,      // parent
        90,                   // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        120,                  // pos x
        24,                   // pos y
        "0:0",                // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_17,     // font
        false,                // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    // SDCard Status
    tray.sdcard_mounted = create_label(
        tray.panel,           // parent
        40,                   // width
        20,                   // height
        LV_ALIGN_TOP_MID,     // parent alignment
        -140,                 // pos x
        24,                   // pos y
        "SD",                 // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_17,     // font
        false,                // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        0,                    // outline width
        general_radius,       // outline radius
        1
    );

    // Grid Menu 1
    tray.grid_menu_1 = create_menu_grid(
        tray.panel,          // lv_obj_t
        7,                   // cols
        1,                   // rows
        56,                  // cell size px
        12,                  // outer padding
        12,                  // inner padding
        0,                   // pos x
        0,                   // pos y
        LV_ALIGN_CENTER,     // alignment
        radius_rounded,      // item radius
        7,                   // Max visbilble columns. Equal or less than cols
        1,                   // Max visible rows. Equal or less than rows
        false,               // show scrollbar
        false,               // enable scrolling
        LV_TEXT_ALIGN_CENTER,// font alignment
        &cobalt_alien_17     // font
    );
    // Grid Menu 1 Configuration
    uint32_t grid_child_cnt = lv_obj_get_child_cnt(tray.grid_menu_1);
    for(uint32_t i = 0; i < grid_child_cnt; i++) {
        lv_obj_t * btn = lv_obj_get_child(tray.grid_menu_1, i);
        // Add callback
        lv_obj_add_event_cb(btn, system_tray_grid_menu_1_event_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_set_style_outline_color(btn, lv_color_hsv_to_rgb((current_hue + 300) % 360, 100, 100), LV_PART_MAIN);
        // Get label
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        if(label && lv_obj_has_class(label, &lv_label_class)) {
            // Set label text
            switch (i) {
                case 0: lv_label_set_text(label, "Home"); break; // Home
                case 1: lv_label_set_text(label, "Mtx"); break; // Matrix
                case 2: lv_label_set_text(label, "GPS"); break; // GPS
                case 3: lv_label_set_text(label, "Gyro"); break; // Gyro
                case 4: lv_label_set_text(label, "Dis"); break; // Display
                case 5: lv_label_set_text(label, "Sys"); break; // System
                case 6: lv_label_set_text(label, "?"); break; // ?
                default: break;
            }
        }
    }

    // Ensure system tray is always on top of other objects
    lv_obj_move_foreground(tray.panel);
    
    return tray;
}

/** ---------------------------------------------------------------------------------------
 * @brief Create Label
 * 
 * Intention is to be resusable, flexible & while providing a consistent style.
 * 
 * @param scr Pointer to the parent screen object.
 * @param size_width_px Width of the label in pixels.
 * @param size_height_px Height of the label in pixels.
 * @param alignment Alignment of the label within its parent/screen.
 * @param pos_x X position of the label within its parent/screen.
 * @param pos_y Y position of the label within its parent/screen.
 * @param text Text to display in the label.
 * @param text_align Alignment of the text within the label (e.g., LV_TEXT_ALIGN_CENTER).
 * @param font Font to use for the label text.
 * @param transparent_bg If true, the label background will be transparent.
 * @param show_scrollbar If true, a scrollbar will be shown.
 * @param enable_scrolling If true, scrolling will be enabled.
 * @return Pointer to the created label object.
 */
lv_obj_t * create_label(
    lv_obj_t * scr,
    int32_t size_width_px,
    int32_t size_height_px,
    int alignment,
    int32_t pos_x,
    int32_t pos_y,
    char * text,
    lv_text_align_t text_align,
    const lv_font_t * font,
    bool transparent_bg,
    bool show_scrollbar,
    bool enable_scrolling,
    int32_t outline_width,
    int32_t radius,
    int32_t expected_number_of_lines
    )
{
    /*----------------------------------------------- LABEL -----------------------------------------------*/

    // Create label
    lv_obj_t * result = lv_label_create(scr);

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(result, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(result, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(result, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(result, LV_DIR_NONE);}

    // Size and position
    lv_obj_set_size(result, size_width_px, size_height_px);
    lv_obj_align(result, (lv_align_t)alignment, pos_x, pos_y);

    /*---------------------------------------- LABEL LV_PART_MAIN -----------------------------------------*/

    // Main style: radius
    lv_obj_set_style_radius(result, radius, LV_PART_MAIN);

    // Vertical centering: calculate top padding based on font height
    int32_t font_line_height = lv_font_get_line_height(font) * expected_number_of_lines;
    int32_t pad_top = (size_height_px - font_line_height) / 2;
    if (pad_top > 0) {
        lv_obj_set_style_pad_top(result, pad_top, LV_PART_MAIN);
    }

    if (transparent_bg) {
        // Main style: outline
        lv_obj_set_style_outline_width(result, 0, LV_PART_MAIN);
        lv_obj_set_style_outline_color(result, menu_outline_color, LV_PART_MAIN);
        
        // Main style: border
        lv_obj_set_style_border_width(result, 0, LV_PART_MAIN);
        lv_obj_set_style_border_color(result, menu_border_color, LV_PART_MAIN);

        // Main style: background
        lv_obj_set_style_bg_opa(result, LV_OPA_TRANSP, LV_PART_MAIN);

        // Main style: shadow
        lv_obj_set_style_shadow_width(result, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_color(result, menu_shadow_color, LV_PART_MAIN);
    }
    else {
        // Main style: outline
        lv_obj_set_style_outline_width(result, outline_width, LV_PART_MAIN);
        lv_obj_set_style_outline_color(result, menu_outline_color, LV_PART_MAIN);
        
        // Main style: border
        lv_obj_set_style_border_width(result, 0, LV_PART_MAIN);
        lv_obj_set_style_border_color(result, menu_border_color, LV_PART_MAIN);

        // Main style: background
        lv_obj_set_style_bg_color(result, menu_bg_color, LV_PART_MAIN);

        // Main style: shadow
        lv_obj_set_style_shadow_width(result, shadow_width, LV_PART_MAIN);
        lv_obj_set_style_shadow_color(result, menu_shadow_color, LV_PART_MAIN);
    }

    // Main style: text
    lv_obj_set_style_text_align(result, text_align, LV_PART_MAIN);
    lv_obj_set_style_text_font(result, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(result, menu_text_color, LV_PART_MAIN);
    lv_label_set_text(result, text);

    return result;
}

/** ---------------------------------------------------------------------------------------
 * @brief Text Area
 * 
 * Intention is to be resusable, flexible & while providing a consistent style.
 * 
 * @param scr Pointer to the parent screen object.
 * @param size_w_px Width of the textarea in pixels.
 * @param size_h_px Height of the textarea in pixels.
 * @param alignment Alignment of the textarea within its parent/screen.
 * @param pos_x X position of the textarea within its parent/screen.
 * @param pos_y Y position of the textarea within its parent/screen.
 * @param one_line Boolean indicating if the textarea should be single line or multi-line.
 * @param accepted_chars Specify characters the textarea should accept.
 * @param placeholder_text Placeholder text to display when the textarea is empty.
 * @param show_scrollbar If true, a scrollbar will be shown.
 * @param enable_scrolling If true, scrolling will be enabled.
 * @return Pointer to the created textarea object.
 */
lv_obj_t * create_textarea(
    lv_obj_t * scr,
    int32_t size_w_px,
    int32_t size_h_px,
    int32_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    bool one_line,
    char * accepted_chars,
    char * placeholder_text,
    bool transparent_bg,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font
    )
{
    /* ----------------------------------- TEXTAREA ------------------------------------ */

    // Create textarea
    lv_obj_t * ta = lv_textarea_create(scr);

    // Set single line mode if specified
    lv_textarea_set_one_line(ta, one_line);
    
    // Hide cursor to prevent text bounce from cursor animation
    lv_obj_set_style_anim_time(ta, 0, LV_PART_CURSOR);
    lv_obj_set_style_opa(ta, LV_OPA_TRANSP, LV_PART_CURSOR);
    
    // Enable cursor click position
    lv_textarea_set_cursor_click_pos(ta, true);

    // Set accepted characters
    lv_textarea_set_accepted_chars(ta, accepted_chars);

    // Set placeholder text and style
    lv_textarea_set_placeholder_text(ta, "");

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(ta, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(ta, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(ta, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(ta, LV_DIR_NONE);}

    // Size, position & shape
    lv_obj_set_size(ta, size_w_px, size_h_px);
    lv_obj_align(ta, (lv_align_t)alignment, pos_x, pos_y);

    /*----------------------------------- LV_PART_MAIN --------------------------------- */

    // Main style: radius
    lv_obj_set_style_radius(ta, general_radius, LV_PART_MAIN);

    if (transparent_bg) {
        // Main style: outline
        lv_obj_set_style_outline_width(ta, 0, LV_PART_MAIN);
        lv_obj_set_style_outline_color(ta, menu_outline_color, LV_PART_MAIN);
        
        // Main style: border
        lv_obj_set_style_border_width(ta, 0, LV_PART_MAIN);
        lv_obj_set_style_border_color(ta, menu_border_color, LV_PART_MAIN);

        // Main style: background
        lv_obj_set_style_bg_opa(ta, LV_OPA_TRANSP, LV_PART_MAIN);

        // Main style: shadow
        lv_obj_set_style_shadow_width(ta, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_color(ta, menu_shadow_color, LV_PART_MAIN);
    }
    else {
        // Main style: outline
        lv_obj_set_style_outline_width(ta, outline_width, LV_PART_MAIN);
        lv_obj_set_style_outline_color(ta, menu_outline_color, LV_PART_MAIN);
        
        // Main style: border
        lv_obj_set_style_border_width(ta, border_width, LV_PART_MAIN);
        lv_obj_set_style_border_color(ta, menu_border_color, LV_PART_MAIN);

        // Main style: background
        lv_obj_set_style_bg_color(ta, menu_bg_color, LV_PART_MAIN);

        // Main style: shadow
        lv_obj_set_style_shadow_width(ta, shadow_width, LV_PART_MAIN);
        lv_obj_set_style_shadow_color(ta, menu_shadow_color, LV_PART_MAIN);
    }

    // Main style: text
    lv_obj_set_style_text_align(ta, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(ta, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(ta, menu_text_color, LV_PART_MAIN);

    return ta;
}

/** -------------------------------------------------------------------------------------
 * @brief Keyboard
 * 
 * Intention is to be resusable, flexible & while providing a consistent style.
 * 
 * @param scr Pointer to the parent screen object.
 * @param size_w_px Width of the keyboard in pixels.
 * @param size_h_px Height of the keyboard in pixels.
 * @param alignment Alignment of the keyboard within its parent/screen.
 * @param pos_x X position of the keyboard within its parent/screen.
 * @param pos_y Y position of the keyboard within its parent/screen.
 * @param kb_ta_padding_px Padding between keyboard and textarea in pixels.
 * @param ta_height_px Height of the textarea in pixels.
 * @param keyboard_mode Set keyboard mode:
 *                      LV_KEYBOARD_MODE_TEXT_LOWER,
                        LV_KEYBOARD_MODE_TEXT_UPPER,
                        LV_KEYBOARD_MODE_SPECIAL,
                        LV_KEYBOARD_MODE_NUMBER,
                        LV_KEYBOARD_MODE_USER_1,
                        LV_KEYBOARD_MODE_USER_2,
                        LV_KEYBOARD_MODE_USER_3,
                        LV_KEYBOARD_MODE_USER_4
 * @return Keyboard struct containing keyboard and textarea objects.
 * 
 */
keyboard_t create_keyboard(
    lv_obj_t * scr,
    int32_t size_w_px,
    int32_t size_h_px,
    int32_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t kb_ta_padding_px,
    int32_t ta_height_px,
    lv_keyboard_mode_t keyboard_mode
    )
{
    /*----------------------------------------------- KEYBOARD --------------------------------------------*/

    // Allocate keyboard struct
    keyboard_t result = {0};
    result.kb = (lv_obj_t *)malloc(sizeof(lv_obj_t *));
    result.ta = (lv_obj_t *)malloc(sizeof(lv_obj_t *));

    // Create keyboard
    result.kb = lv_keyboard_create(scr);

    // Keyboard mode
    lv_keyboard_set_mode(result.kb, keyboard_mode);

    // Size and position
    lv_obj_set_size(result.kb, size_w_px, size_h_px);
    lv_obj_align(result.kb, (lv_align_t)alignment, pos_x, pos_y);

    /*---------------------------------------- KEYBOARD LV_PART_MAIN ---------------------------------------*/

    // Main style: radius
    lv_obj_set_style_radius(result.kb, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.kb, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.kb, menu_outline_color, LV_PART_MAIN);
    
    // Main style: border
    lv_obj_set_style_border_width(result.kb, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.kb, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.kb, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.kb, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.kb, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.kb, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.kb, font_menu_item, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.kb, menu_text_color, LV_PART_MAIN);

    /*-------------------------------------- KEYBOARD LV_PART_ITEMS ---------------------------------------*/

    // Item style: radius
    lv_obj_set_style_radius(result.kb, general_radius, LV_PART_ITEMS);

    // Item style: outline
    lv_obj_set_style_outline_width(result.kb, outline_width, LV_PART_ITEMS);
    lv_obj_set_style_outline_color(result.kb, menu_outline_color, LV_PART_ITEMS);

    // Item style: border
    lv_obj_set_style_border_width(result.kb, border_width, LV_PART_ITEMS);
    lv_obj_set_style_border_color(result.kb, menu_border_color, LV_PART_ITEMS);

    // Item style: background
    lv_obj_set_style_bg_color(result.kb, menu_bg_color, LV_PART_ITEMS);

    // Item style: shadow
    lv_obj_set_style_shadow_width(result.kb, shadow_width, LV_PART_ITEMS);
    lv_obj_set_style_shadow_color(result.kb, menu_shadow_color, LV_PART_ITEMS);

    // Item style: text
    lv_obj_set_style_text_align(result.kb, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.kb, font_menu_item, LV_PART_ITEMS);
    lv_obj_set_style_text_color(result.kb, menu_text_color, LV_PART_ITEMS);
    
    // Item style: background checked
    lv_obj_set_style_bg_color(result.kb, menu_border_color, LV_PART_ITEMS | LV_STATE_CHECKED);
    
    // Item style: text checked
    lv_obj_set_style_text_align(result.kb, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.kb, font_menu_item, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(result.kb, menu_text_color, LV_PART_ITEMS | LV_STATE_CHECKED);

    /*---------------------------------------------- TEXTAREA -----------------------------------------------*/

    // Create text area
    result.ta = lv_textarea_create(scr);

    // Connect keyboard to textarea
    lv_keyboard_set_textarea(result.kb, result.ta);

    // Get keyboard coordinates
    lv_area_t kb_coords;
    lv_obj_get_coords(result.kb, &kb_coords);
    lv_coord_t ta_x = kb_coords.x1;
    lv_coord_t ta_y = kb_coords.y1 - ta_height_px;

    // Size and position
    lv_obj_set_size(result.ta, size_w_px, ta_height_px);
    lv_obj_set_pos(result.ta, ta_x, ta_y-kb_ta_padding_px);

    /*--------------------------------------- TEXTAREA LV_PART_MAIN -----------------------------------------*/

    // Main style: radius
    lv_obj_set_style_radius(result.ta, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.ta, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.ta, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(result.ta, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.ta, menu_border_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.ta, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.ta, menu_shadow_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.ta, menu_bg_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.ta, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.ta, font_menu_item, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.ta, menu_text_color, LV_PART_MAIN);

    return result;
}

/** -----------------------------------------------------------------------------------------------
 * @brief Create Menu Item.
 * 
 * Intention is to be resusable, flexible & while providing a consistent style.
 * 
 * @param menu_page Pointer to the parent menu_x object.
 * @param title Title of the menu_x page.
 * @return Pointer to the created menu_x page object.
 */
void create_menu_item(menu_struct * menu, int page_index, const char * title) {

    /*--------------------------------------------- CONTAINER -----------------------------------------------*/

    // Create container
    lv_obj_t * container = lv_menu_cont_create(menu->pages[page_index]);

    /*-------------------------------------- CONTAINER LV_PART_MAIN -----------------------------------------*/
    
    // Size and position
    lv_obj_set_style_pad_hor(container, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(container, 4, LV_PART_MAIN);
    lv_obj_set_width(container, lv_pct(80));

    // Main style: radius
    lv_obj_set_style_radius(container, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(container, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(container, lv_color_make(0,0,0), LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(container, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(container, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(container, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(container, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(container, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(container, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_font(container, font_menu_item, LV_PART_MAIN);
    lv_obj_set_style_text_color(container, menu_text_color, LV_PART_MAIN);

    /*----------------------------------------------- LABEL -----------------------------------------------*/
    
    // Create label
    lv_obj_t * label = lv_label_create(container);

    /* --- LABEL LV_PART_MAIN ---------------------------------------------------------- */

    // Size and position
    lv_obj_set_style_pad_hor(label, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(label, 8, LV_PART_MAIN);
    lv_obj_set_width(label, lv_pct(100));

    // Main style: radius
    lv_obj_set_style_radius(label, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(label, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_color(label, lv_color_make(0,0,0), LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(label, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(label, menu_border_color, LV_PART_MAIN);
    
    // Main style: background
    lv_obj_set_style_bg_color(label, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(label, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(label, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font_menu_item, LV_PART_MAIN);
    lv_label_set_text(label, title);

    /* --- ADD TO MENU ----------------------------------------------------------------- */
    
    // Add to menu struct
    if(menu->page_item_counts[page_index] < menu->max_items_per_page) {
        menu->page_items[page_index][menu->page_item_counts[page_index]] = container;
        menu->page_item_counts[page_index]++;
    }
}

/** -----------------------------------------------------------------------------------------------
 * @brief Create Menu Page.
 * 
 * Intention is to be resusable, flexible & while providing a consistent style.
 * 
 * @param menu_x Pointer to the parent menu_x object.
 * @param title Title of the menu_x page.
 * @return Pointer to the created menu_x page object.
 */
lv_obj_t * create_menu_page(lv_obj_t * menu_x, const char * title) {

    /* --- MENU ------------------------------------------------------------------------ */

    // Create menu
    lv_obj_t * menu_page = lv_menu_page_create(menu_x, title);

    /* --- MENU LV_PART_MAIN ----------------------------------------------------------- */

    // Size and position
    lv_obj_set_style_pad_row(menu_page, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_column(menu_page, 2, LV_PART_MAIN);

    // Main style: radius
    lv_obj_set_style_radius(menu_page, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(menu_page, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_color(menu_page, lv_color_make(0,0,0), LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(menu_page, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(menu_page, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(menu_page, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(menu_page, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(menu_page, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(menu_page, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(menu_page, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_font(menu_page, font_menu_item, LV_PART_MAIN);

    return menu_page;
}

/** -----------------------------------------------------------------------------------------------
 * @brief Create Menu
 * 
 * Intention is to be resusable, flexible & while providing a consistent style.
 * 
 * @param scr Pointer to the parent screen object.
 * @param max_pages Maximum number of pages in the menu.
 * @param page_titles Array of page titles (NULL-terminated).
 * @param main_menu_items Array of main menu item titles (NULL-terminated).
 * @param size_w_px Width of the menu in pixels.
 * @param size_h_px Height of the menu in pixels.
 * @param alignment Alignment of the menu within its parent/screen.
 * @param pos_x X position of the menu within its parent/screen.
 * @param pos_y Y position of the menu within its parent/screen.
 * 
 */
menu_struct create_menu(
    lv_obj_t *scr,
    int max_pages,
    const char ** page_titles,
    const char ** main_menu_items,
    int32_t size_w_px,
    int32_t size_h_px,
    int32_t alignment,
    int32_t pos_x,
    int32_t pos_y
    )
{
    /* --- MENU ------------------------------------------------------------------------ */

    // Create a new menu_struct
    menu_struct result = {0};
    
    // Allocate pages arrays
    result.max_pages = max_pages;
    result.pages = (lv_obj_t **)malloc(max_pages * sizeof(lv_obj_t *));
    result.page_items = (lv_obj_t ***)malloc(max_pages * sizeof(lv_obj_t **));
    result.page_item_counts = (uint32_t *)calloc(max_pages, sizeof(uint32_t));
    
    // Allocate items arrays for each page
    result.max_items_per_page = 10;  // Max items per page
    for(int p = 0; p < max_pages; p++) {
        result.page_items[p] = (lv_obj_t **)malloc(result.max_items_per_page * sizeof(lv_obj_t *));
    }

    // Create menu
    result.menu = lv_menu_create(scr);

    // Size and position
    lv_obj_set_size(result.menu, size_w_px, size_h_px);
    lv_obj_align(result.menu, (lv_align_t)alignment, pos_x, pos_y);

    /* --- MENU LV_PART_MAIN ----------------------------------------------------------- */

    // Size and position
    lv_obj_set_style_transform_width(result.menu, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(result.menu, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_column(result.menu, 2, LV_PART_MAIN);

    // Main style: radius
    lv_obj_set_style_radius(result.menu, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.menu, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.menu, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(result.menu, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.menu, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.menu, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.menu, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.menu, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.menu, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.menu, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.menu, font_menu_item, LV_PART_MAIN);

    /* --- MENU LV_PART_ITEMS ---------------------------------------------------------- */

    lv_obj_set_style_border_width(result.menu, border_width, LV_PART_ITEMS);

    /* --- MAIN HEADER ----------------------------------------------------------------- */

    // Create main header
    result.main_header = lv_menu_get_main_header(result.menu);

    /* --- MAIN HEADER LV_PART_MAIN ---------------------------------------------------- */

    // Size and position
    lv_obj_set_style_pad_all(result.main_header, 12, LV_PART_MAIN);

    // Main style: radius
    lv_obj_set_style_radius(result.main_header, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.main_header, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.main_header, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(result.main_header, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.main_header, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.main_header, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.main_header, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.main_header, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.main_header, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.main_header, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.main_header, font_menu_title, LV_PART_MAIN);

    /* --- ROOT BACK BUTTON ------------------------------------------------------------ */
    
    // Enable root back button
    lv_menu_set_mode_root_back_button(result.menu, LV_MENU_ROOT_BACK_BUTTON_ENABLED);

    // Create back button
    result.back_button = lv_menu_get_main_header_back_button(result.menu);

    // Create back button label
    result.back_label = lv_obj_get_child(result.back_button, 0);

    /* --- ROOT BACK BUTTON LV_PART_MAIN ------------------------------------------------- */

    // Size and position
    lv_obj_set_style_pad_all(result.back_button, 8, LV_PART_MAIN);

    // Main style: radius
    lv_obj_set_style_radius(result.back_button, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.back_button, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.back_button, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(result.back_button, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.back_button, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.back_button, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.back_button, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.back_button, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.back_button, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.back_button, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.back_button, font_menu_item, LV_PART_MAIN);

    /* --- ROOT BACK BUTTON LABEL LV_PART_MAIN ----------------------------------------- */

    // Main style: radius
    lv_obj_set_style_radius(result.back_label, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.back_label, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.back_label, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(result.back_label, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.back_label, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.back_label, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.back_label, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.back_label, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.back_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.back_label, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.back_label, font_menu_item, LV_PART_MAIN);

    /* --- CREATE PAGES ----------------------------------------------------------------- */

    // Create Pages
    for(int i = 0; i < max_pages && page_titles[i] != NULL; i++) {
        result.pages[i] = create_menu_page(result.menu, page_titles[i]);
        result.page_count++;
    }

    // Add main menu items to first page if provided
    if(main_menu_items && result.pages[0]) {
        for(int i = 0; main_menu_items[i] != NULL; i++) {
            create_menu_item(&result, 0, main_menu_items[i]);
        }
    }
    
    // Set current page
    lv_menu_set_page(result.menu, result.pages[0]);

    return result;
}

/** ----------------------------------------------------------------------------------------
 * @brief Creates Menu Grid.
 * 
 * Intention is to be resusable, flexible & while providing a consistent style.
 * 
 * @param scr Pointer to the parent screen object.
 * @param menu_grid Pointer to the parent menu_grid object.
 * @param cols Number of columns in the grid.
 * @param rows Number of rows in the grid.
 * @param cell_size_px Size of each cell in pixels.
 * @param outer_padding Padding around the grid in pixels.
 * @param inner_padding Padding between cells in pixels.
 * @param pos_x X position of the grid within its parent/screen.
 * @param pos_y Y position of the grid within its parent/screen.
 * @param lv_alignment Alignment of the grid within its parent/screen.
 * @param item_radius Radius of the grid items in pixels.
 * @param max_cols_visible Maximum number of columns visible.
 * @param max_rows_visible Maximum number of rows visible.
 * @param show_scrollbar If true, scrollbars will always be shown.
 * @param enable_scrolling If true, scrolling will be enabled for the grid.
 * @return Pointer to the created grid_menu object.
 */
lv_obj_t * create_menu_grid(
    lv_obj_t *scr,
    // lv_obj_t * grid_menu,
    const int32_t cols,
    const int32_t rows,
    const int32_t cell_size_px,
    const int32_t outer_padding,
    const int32_t inner_padding,
    const int32_t pos_x,
    const int32_t pos_y,
    const int lv_alignment,
    int32_t item_radius,
    int32_t max_cols_visible,
    int32_t max_rows_visible,
    bool show_scrollbar,
    bool enable_scrolling,
    lv_text_align_t text_align,
    const lv_font_t * font
    )
{
    /* ---- GRID MENU CONFIGURATION ---------------------------------------------------- */

    const int32_t GRID_MENU_X_CELL_SIZE_PX = cell_size_px; // Set cell size in pixels
    const int32_t GRID_MENU_X_MAX_COLS = cols;
    const int32_t GRID_MENU_X_MAX_ROWS = rows;
    #define GRID_MENU_X_OUTER_PADDING outer_padding // Set space around grid
    #define GRID_MENU_X_INNER_PADDING inner_padding // Set space between cells
    #define GRID_MENU_X_SCROLLBAR_OFFSET 12 // Set scrollbar offset when scrolling enabled

    // Set arrays
    #define GRID_MENU_X_TOTAL_CELLS (GRID_MENU_X_MAX_COLS * GRID_MENU_X_MAX_ROWS)
    lv_coord_t * grid_menu_x_col_dsc = (lv_coord_t *)malloc((GRID_MENU_X_MAX_COLS + 1) * sizeof(lv_coord_t));
    lv_coord_t * grid_menu_x_row_dsc = (lv_coord_t *)malloc((GRID_MENU_X_MAX_ROWS + 1) * sizeof(lv_coord_t));

    // Create grid object
    lv_obj_t * grid_menu = lv_obj_create(scr);

    // Automatically set size according to configuration
    if (show_scrollbar == false) {
        #define GRID_MENU_X_MAX_DYNAMIC_COLS max_cols_visible // limit displayed columns (does not limit total columns, scrollbar will appear)
        #define GRID_MENU_X_MAX_DYNAMIC_ROWS max_rows_visible // limit displayed rows (does not limit total rows, scrollbar will appear)
        #define DYNAMIC_GRID_WIDTH  (GRID_MENU_X_MAX_DYNAMIC_COLS * GRID_MENU_X_CELL_SIZE_PX + (GRID_MENU_X_MAX_DYNAMIC_COLS-1) * GRID_MENU_X_INNER_PADDING + 2*GRID_MENU_X_OUTER_PADDING)
        #define DYNAMIC_GRID_HEIGHT (GRID_MENU_X_MAX_DYNAMIC_ROWS * GRID_MENU_X_CELL_SIZE_PX + (GRID_MENU_X_MAX_DYNAMIC_ROWS-1) * GRID_MENU_X_INNER_PADDING + 2*GRID_MENU_X_OUTER_PADDING)
        lv_obj_set_scrollbar_mode(grid_menu, LV_SCROLLBAR_MODE_OFF); // remove scrollbars
        lv_obj_set_size(grid_menu, DYNAMIC_GRID_WIDTH, DYNAMIC_GRID_HEIGHT);
        lv_obj_set_style_pad_all(grid_menu, GRID_MENU_X_OUTER_PADDING, LV_PART_MAIN); // Only padding, no spacing here
        lv_obj_set_style_pad_gap(grid_menu, GRID_MENU_X_INNER_PADDING, LV_PART_MAIN); // Gap between cells
    }

    // Automatically set size according to configuration (+ scrollbar offset)
    else if (show_scrollbar == true) {
        #define GRID_MENU_X_MAX_DYNAMIC_COLS max_cols_visible // limit displayed columns (does not limit total columns, scrollbar will appear)
        #define GRID_MENU_X_MAX_DYNAMIC_ROWS max_rows_visible // limit displayed rows (does not limit total rows, scrollbar will appear)
        #define DYNAMIC_GRID_WIDTH  (GRID_MENU_X_MAX_DYNAMIC_COLS * GRID_MENU_X_CELL_SIZE_PX + (GRID_MENU_X_MAX_DYNAMIC_COLS-1) * GRID_MENU_X_INNER_PADDING + 2*GRID_MENU_X_OUTER_PADDING) + GRID_MENU_X_SCROLLBAR_OFFSET
        #define DYNAMIC_GRID_HEIGHT (GRID_MENU_X_MAX_DYNAMIC_ROWS * GRID_MENU_X_CELL_SIZE_PX + (GRID_MENU_X_MAX_DYNAMIC_ROWS-1) * GRID_MENU_X_INNER_PADDING + 2*GRID_MENU_X_OUTER_PADDING) + GRID_MENU_X_SCROLLBAR_OFFSET
        lv_obj_set_scrollbar_mode(grid_menu, LV_SCROLLBAR_MODE_ON); // show scrollbars
        lv_obj_set_size(grid_menu, DYNAMIC_GRID_WIDTH, DYNAMIC_GRID_HEIGHT);
        lv_obj_set_style_pad_all(grid_menu, GRID_MENU_X_OUTER_PADDING, LV_PART_MAIN); // Only padding, no spacing here
        lv_obj_set_style_pad_gap(grid_menu, GRID_MENU_X_INNER_PADDING, LV_PART_MAIN); // Gap between cells
    }

    // enable scrolling in all directions
    if (enable_scrolling) {lv_obj_set_scroll_dir(grid_menu, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(grid_menu, LV_DIR_NONE);}

    /* ---- GRID MENU ------------------------------------------------------------------ */

    // Size and position
    lv_obj_align(grid_menu, (lv_align_t)lv_alignment, pos_x, pos_y);

    /* ---- GRID MENU LV_PART_MAIN ----------------------------------------------------- */

    // Main style: radius
    lv_obj_set_style_radius(grid_menu, general_radius, LV_PART_MAIN);


    // Main style: outline
    lv_obj_set_style_outline_width(grid_menu, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(grid_menu, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(grid_menu, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(grid_menu, menu_border_color, LV_PART_MAIN);
    
    // Main style: background
    lv_obj_set_style_bg_color(grid_menu, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(grid_menu, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(grid_menu, menu_shadow_color, LV_PART_MAIN);
    
    // Set layout to grid
    lv_obj_set_layout(grid_menu, LV_LAYOUT_GRID);

    // Add cells to colmns array
    for(int i = 0; i < GRID_MENU_X_MAX_COLS; i++) {grid_menu_x_col_dsc[i] = GRID_MENU_X_CELL_SIZE_PX;}
    grid_menu_x_col_dsc[GRID_MENU_X_MAX_COLS] = LV_GRID_TEMPLATE_LAST;

    // Add cells to rows array
    for(int i = 0; i < GRID_MENU_X_MAX_ROWS; i++) {grid_menu_x_row_dsc[i] = GRID_MENU_X_CELL_SIZE_PX;}
    grid_menu_x_row_dsc[GRID_MENU_X_MAX_ROWS] = LV_GRID_TEMPLATE_LAST;

    // Apply arrays to grid
    lv_obj_set_style_grid_column_dsc_array(grid_menu, grid_menu_x_col_dsc, LV_PART_MAIN);
    lv_obj_set_style_grid_row_dsc_array(grid_menu, grid_menu_x_row_dsc, LV_PART_MAIN);
    lv_obj_update_layout(grid_menu);

    // Add buttons to grid
    for(int i = 0; i < GRID_MENU_X_TOTAL_CELLS; i++) {

        /* ---- CELL BUTTON ----------------------------------------------------------------- */

        // Create button for cell
        lv_obj_t * grid_menu_x_btn = lv_btn_create(grid_menu);

        // Size and position
        lv_obj_set_size(grid_menu_x_btn, GRID_MENU_X_CELL_SIZE_PX, GRID_MENU_X_CELL_SIZE_PX);

        /* ---- CELL BUTTON LV_PART_MAIN ------------------------------------------------------ */

        // Size and position
        lv_obj_set_style_pad_all(grid_menu_x_btn, 0, LV_PART_MAIN);

        // Button style: radius
        lv_obj_set_style_radius(grid_menu_x_btn, item_radius, LV_PART_MAIN);

        // Button style: outline
        lv_obj_set_style_outline_width(grid_menu_x_btn, outline_width, LV_PART_MAIN);
        lv_obj_set_style_outline_color(grid_menu_x_btn, menu_outline_color, LV_PART_MAIN);

        // Button style: border
        lv_obj_set_style_border_width(grid_menu_x_btn, 0, LV_PART_MAIN);
        lv_obj_set_style_border_color(grid_menu_x_btn, menu_border_color, LV_PART_MAIN);

        // Button style: background
        lv_obj_set_style_bg_color(grid_menu_x_btn, menu_bg_color, LV_PART_MAIN);

        // Button style: shadow
        lv_obj_set_style_shadow_width(grid_menu_x_btn, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_color(grid_menu_x_btn, menu_shadow_color, LV_PART_MAIN);

        /* --- CELL LABEL ----------------------------------------------------------------------- */
        
        // Create label for button
        lv_obj_t * grid_menu_x_label = lv_label_create(grid_menu_x_btn);

        // Set label text to cell index
        lv_label_set_text_fmt(grid_menu_x_label, "%d", i);

        /* --- CELL LABEL LV_PART_MAIN ---------------------------------------------------------- */

        // Size and position
        lv_obj_center(grid_menu_x_label);
        lv_obj_set_grid_cell(grid_menu_x_btn,
            LV_GRID_ALIGN_CENTER, (i % GRID_MENU_X_MAX_COLS), 1,  // Column index
            LV_GRID_ALIGN_CENTER, (i / GRID_MENU_X_MAX_COLS), 1); // Row Index

        // Label style: text
        lv_obj_set_style_text_align(grid_menu_x_label, text_align, LV_PART_MAIN);
        lv_obj_set_style_text_font(grid_menu_x_label, font, LV_PART_MAIN);
        lv_obj_set_style_text_color(grid_menu_x_label, menu_text_color, LV_PART_MAIN);
        
    }
    return grid_menu;
}

/** ---------------------------------------------------------------------------------------
 * @brief Dop Down Menu X
 * 
 * Intention is to be resusable, flexible & while providing a consistent style.
 * 
 * @param scr Pointer to the parent screen object.
 * @param options Array of option strings.
 * @param option_count Number of options in the array.
 * @param width_px Width of the dropdown menu in pixels.
 * @param height_px Height of the dropdown menu in pixels.
 * @param pos_x X position of the dropdown menu within its parent/screen.
 * @param pos_y Y position of the dropdown menu within its parent/screen.
 * 
 */
lv_obj_t * create_dropdown_menu(
    lv_obj_t * scr,
    char options[][MAX_GLOBAL_ELEMENT_SIZE],
    int option_count,
    int32_t width_px,
    int32_t height_px,
    int alignment,
    int32_t pos_x,
    int32_t pos_y,
    const lv_font_t * font
    )
{
    /* --- DROPDOWN -------------------------------------------------------------------- */

    // Create dropdown
    lv_obj_t * ddlist = lv_dropdown_create(scr);


    // Add options if provided
    if (options != NULL && option_count > 0) {
        // Initialize with first option
        lv_dropdown_set_options_static(ddlist, options[0]);

        // Add remaining options
        for(int i = 1; i < option_count; i++) {
            lv_dropdown_add_option(ddlist, options[i], LV_DROPDOWN_POS_LAST);
        }
    }
    else {
        // Clear default options when none provided
        lv_dropdown_set_options(ddlist, "");
    }

    // Size and position
    lv_obj_set_size(ddlist, width_px, height_px);
    lv_obj_align(ddlist, (lv_align_t)alignment, pos_x, pos_y);

    /* --- DROPDOWN LV_PART_MAIN ------------------------------------------------------- */

    // Main style: radius
    lv_obj_set_style_radius(ddlist, general_radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(ddlist, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(ddlist, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(ddlist, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(ddlist, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(ddlist, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(ddlist, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(ddlist, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(ddlist, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_text_font(ddlist, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(ddlist, menu_text_color, LV_PART_MAIN);

    /* --- DROPDOWN LIST --------------------------------------------------------------- */

    // Get dropdown list object
    lv_obj_t * list = lv_dropdown_get_list(ddlist);

    /* --- DROPDOWN LIST LV_PART_MAIN -------------------------------------------------- */

    // List style: radius
    lv_obj_set_style_radius(list, general_radius, LV_PART_MAIN);

    // List style: outline
    lv_obj_set_style_outline_width(list, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(list, menu_outline_color, LV_PART_MAIN);

    // List style: border
    lv_obj_set_style_border_width(list, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(list, menu_border_color, LV_PART_MAIN);

    // List style: background
    lv_obj_set_style_bg_color(list, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(list, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(list, menu_shadow_color, LV_PART_MAIN);

    // List style: text
    lv_obj_set_style_text_align(list, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_text_font(list, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(list, menu_text_color, LV_PART_MAIN);

    // List style: background checked
    lv_obj_set_style_bg_color(list, menu_border_color, LV_PART_SELECTED | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(list, menu_border_color, LV_PART_SELECTED | LV_STATE_DEFAULT);

    return ddlist;
}

lv_obj_t * create_switch(
    lv_obj_t *scr,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y
    )
{
    lv_obj_t * sw = lv_switch_create(scr);
    lv_obj_set_size(sw, size_w_px, size_h_px);
    lv_obj_align(sw, alignment, pos_x, pos_y);

    lv_obj_set_style_bg_color(sw, lv_color_make(58, 58, 58), LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, lv_color_make(58, 58, 58), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sw, lv_color_make(0, 64, 0), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw, lv_color_make(128, 128, 128), LV_PART_KNOB);
    
    return sw;
}

button_t create_button(
    lv_obj_t *scr,
    int32_t size_width_px,
    int32_t size_height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    char * text,
    lv_text_align_t text_align,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font,
    int32_t radius
    )
{
    button_t result = {0};

    // ---- Panel Style ----

    result.panel = lv_obj_create(scr);

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(result.panel, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(result.panel, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(result.panel, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(result.panel, LV_DIR_NONE);}

    // Size and position
    lv_obj_set_size(result.panel, size_width_px, size_height_px);
    lv_obj_align(result.panel, alignment, pos_x, pos_y);

    // Main style: radius
    lv_obj_set_style_radius(result.panel, radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.panel, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.panel, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(result.panel, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.panel, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.panel, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.panel, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.panel, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.panel, text_align, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.panel, font_menu_item, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.panel, menu_text_color, LV_PART_MAIN);

    // ---- Button Style ----

    result.button = lv_btn_create(result.panel);

    // Size and position
    lv_obj_set_size(result.button, size_width_px, size_height_px);
    lv_obj_center(result.button);

    // Main style: radius
    lv_obj_set_style_radius(result.button, radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.button, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.button, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(result.button, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.button, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.button, menu_bg_color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(result.button, LV_OPA_0, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.button, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.button, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.button, text_align, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.button, font_menu_item, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.button, menu_text_color, LV_PART_MAIN);

    // ---- Label Style ----
    
    result.label = lv_label_create(result.button);

    // Set text first
    lv_label_set_text(result.label, text);

    // Main style: radius
    lv_obj_set_style_radius(result.label, radius, LV_PART_MAIN);

    // Main style: outline
    lv_obj_set_style_outline_width(result.label, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.label, menu_outline_color, LV_PART_MAIN);

    // Main style: border
    lv_obj_set_style_border_width(result.label, 0, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.label, menu_border_color, LV_PART_MAIN);

    // Main style: background
    lv_obj_set_style_bg_color(result.label, menu_bg_color, LV_PART_MAIN);

    // Main style: shadow
    lv_obj_set_style_shadow_width(result.label, shadow_width, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(result.label, menu_shadow_color, LV_PART_MAIN);

    // Main style: text
    lv_obj_set_style_text_align(result.label, text_align, LV_PART_MAIN);
    lv_obj_set_style_text_font(result.label, font_menu_item, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label, menu_text_color, LV_PART_MAIN);

    // Size and position (Center the (now auto-sized) label inside panel)
    lv_obj_center(result.label);

    // Label Flags & Events
    lv_obj_clear_flag(result.label, LV_OBJ_FLAG_CLICKABLE); // delegate clicks to lower layer (btn)
    lv_obj_clear_flag(result.label, LV_OBJ_FLAG_CLICK_FOCUSABLE); // delegate focus to lower layer (btn)

    // Button Flags & Events
    lv_obj_add_flag(result.button, LV_OBJ_FLAG_CLICKABLE);

    return result;
}

/** ----------------------------------------------------------------------------------------
 * @brief Create Matrix Function Container
 * 
 * Creates a container panel with all elements needed to display/edit matrix function data:
 * - Function Slot
 * - Function Name
 * - XYZ Coordinates (3 values)
 * - Operator Name
 * - Flux Value
 * - Output Mode
 * - Output PWM (2 values)
 * - Inverted Logic
 * - Port Map
 * 
 * @param scr Pointer to the parent screen object.
 * @param width_px Width of the container in pixels.
 * @param height_px Height of the container in pixels.
 * @param alignment Alignment of the container within its parent/screen.
 * @param pos_x X position of the container.
 * @param pos_y Y position of the container.
 * @return matrix_function_container_t struct with all UI element pointers.
 */
matrix_function_container_t create_matrix_function_container(
    lv_obj_t * scr,
    int32_t width_px,
    int32_t height_px,
    int32_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
) {
    matrix_function_container_t result = {0};
    
    // Layout constants
    const int32_t padding = 0;
    const int32_t row_spacing = 0;
    const int32_t row_height = 30;
    const int32_t label_width = 80;
    const int32_t value_width = width_px-label_width-2*padding;
    
    /* --- MAIN PANEL ------------------------------------------------------------------ */
    
    result.panel = lv_obj_create(scr);
    lv_obj_set_size(result.panel, width_px, height_px);
    lv_obj_align(result.panel, (lv_align_t)alignment, pos_x, pos_y);

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(result.panel, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(result.panel, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(result.panel, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(result.panel, LV_DIR_NONE);}
    
    // Panel style
    lv_obj_set_style_radius(result.panel, general_radius, LV_PART_MAIN);
    lv_obj_set_style_outline_width(result.panel, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.panel, menu_outline_color, LV_PART_MAIN);
    lv_obj_set_style_border_width(result.panel, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.panel, menu_border_color, LV_PART_MAIN);
    lv_obj_set_style_bg_color(result.panel, menu_bg_color, LV_PART_MAIN);
    lv_obj_set_style_pad_all(result.panel, padding, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(result.panel, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(result.panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(result.panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(result.panel, row_spacing, LV_PART_MAIN);

    /* --- ROW 1: Function Slot ------------------------------------------------------- */
    lv_obj_t * row1 = lv_obj_create(result.panel);
    lv_obj_set_size(row1, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row1, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row1, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row1,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row1, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row1, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row1, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row1, LV_DIR_NONE);}
    
    result.label_function_index_select = lv_label_create(row1);
    lv_label_set_text(result.label_function_index_select, "Function:");
    lv_obj_set_size(result.label_function_index_select, label_width, row_height);
    lv_obj_set_style_text_font(result.label_function_index_select, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_function_index_select, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_function_index_select, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.dd_function_index_select = create_dropdown_menu(
        row1,
        NULL,
        0,
        value_width,
        row_height,
        LV_ALIGN_CENTER,
        0,
        0,
        font_sub
    );
    char dd_function_index_select_name[MAX_GLOBAL_ELEMENT_SIZE];
    for (int i = 0; i < MAX_MATRIX_SWITCH_FUNCTIONS; i++) {
        snprintf(dd_function_index_select_name, sizeof(dd_function_index_select_name), "%d", i);
        lv_dropdown_add_option(result.dd_function_index_select, dd_function_index_select_name, LV_DROPDOWN_POS_LAST);
    }
    lv_dropdown_set_selected(result.dd_function_index_select, matrixData.matrix_function[0][current_matrix_i][current_matrix_function_i]);
    lv_obj_add_event_cb(result.dd_function_index_select, dd_function_index_select_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Critical for alignment
    lv_obj_set_size(result.label_function_index_select, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.dd_function_index_select, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 2: Function Name ------------------------------------------------------- */
    
    lv_obj_t * row2 = lv_obj_create(result.panel);
    lv_obj_set_size(row2, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row2, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row2, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row2,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row2, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row2, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row2, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row2, LV_DIR_NONE);}
    
    result.label_function_name = lv_label_create(row2);
    lv_label_set_text(result.label_function_name, "Function:");
    lv_obj_set_size(result.label_function_name, label_width, row_height);
    lv_obj_set_style_text_font(result.label_function_name, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_function_name, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_function_name, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.dd_function_name = create_dropdown_menu(
        row2,
        NULL,
        0,
        value_width,
        row_height,
        LV_ALIGN_CENTER,
        0,
        0,
        font_sub
    );
    char dd_function_name[MAX_GLOBAL_ELEMENT_SIZE];
    for (int i = 0; i < MAX_MATRIX_FUNCTION_NAMES; i++) {
        snprintf(dd_function_name, sizeof(dd_function_name), "%s", matrixData.matrix_function_names[i]);
        lv_dropdown_add_option(result.dd_function_name, dd_function_name, LV_DROPDOWN_POS_LAST);
    }
    lv_dropdown_set_selected(result.dd_function_name, matrixData.matrix_function[0][current_matrix_i][current_matrix_function_i]);
    lv_obj_add_event_cb(result.dd_function_name, dd_function_name_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Critical for alignment
    lv_obj_set_size(result.label_function_name, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.dd_function_name, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 3: X Value ------------------------------------------------------------- */
    
    lv_obj_t * row3 = lv_obj_create(result.panel);
    lv_obj_set_size(row3, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row3, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row3, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row3, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row3, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row3, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row3,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row3, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row3, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row3, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row3, LV_DIR_NONE);}
    
    result.label_x = lv_label_create(row3);
    lv_label_set_text(result.label_x, "X:");
    lv_obj_set_size(result.label_x, label_width, row_height);
    lv_obj_set_style_text_font(result.label_x, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_x, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_x, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_x = create_textarea(
        row3,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_x, String(matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_X]).c_str());
    lv_obj_add_event_cb(result.ta_x, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_x, &matrix_value_x_ctx);

    // Critical for alignment
    lv_obj_set_size(result.label_x, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_x, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 4: Y Value ------------------------------------------------------------- */
    
    lv_obj_t * row4 = lv_obj_create(result.panel);
    lv_obj_set_size(row4, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row4, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row4, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row4, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row4, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row4, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row4,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row4, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row4, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row4, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row4, LV_DIR_NONE);}
    
    result.label_y = lv_label_create(row4);
    lv_label_set_text(result.label_y, "Y:");
    lv_obj_set_size(result.label_y, label_width, row_height);
    lv_obj_set_style_text_font(result.label_y, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_y, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_y, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_y = create_textarea(
        row4,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_y, String(matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_Y]).c_str());
    lv_obj_add_event_cb(result.ta_y, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_y, &matrix_value_y_ctx);

    // Critical for alignment
    lv_obj_set_size(result.label_y, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_y, value_width, LV_SIZE_CONTENT);

    /* --- ROW 5: Z Value ------------------------------------------------------------- */
    
    lv_obj_t * row5 = lv_obj_create(result.panel);
    lv_obj_set_size(row5, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row5, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row5, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row5, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row5, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row5, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row5,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row5, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row5, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row5, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row5, LV_DIR_NONE);}
    
    result.label_z = lv_label_create(row5);
    lv_label_set_text(result.label_z, "Z:");
    lv_obj_set_size(result.label_z, label_width, row_height);
    lv_obj_set_style_text_font(result.label_z, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_z, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_z, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_z = create_textarea(
        row5,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_z, String(matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_Z]).c_str());
    lv_obj_add_event_cb(result.ta_z, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_z, &matrix_value_z_ctx);

    // Critical for alignment
    lv_obj_set_size(result.label_z, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_z, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 6: Operator ------------------------------------------------------------ */
    
    lv_obj_t * row6 = lv_obj_create(result.panel);
    lv_obj_set_size(row6, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row6, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row6, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row6, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row6, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row6, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row6,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row6, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row6, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row6, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row6, LV_DIR_NONE);}
    
    result.label_operator = lv_label_create(row6);
    lv_label_set_text(result.label_operator, "Operator:");
    lv_obj_set_size(result.label_operator, label_width, row_height);
    lv_obj_set_style_text_font(result.label_operator, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_operator, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_operator, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.dd_operator = create_dropdown_menu(
        row6,
        NULL,
        0,
        value_width,
        row_height,
        LV_ALIGN_CENTER,
        0,
        0,
        font_sub
    );
    char dd_operator_name[MAX_GLOBAL_ELEMENT_SIZE];
    for (int i = 0; i < MAX_MATRIX_OPERATORS; i++) {
        snprintf(dd_operator_name, sizeof(dd_operator_name), "%s", matrixData.matrix_function_operator_name[i]);
        lv_dropdown_add_option(result.dd_operator, dd_operator_name, LV_DROPDOWN_POS_LAST);
    }
    // lv_dropdown_set_selected(result.label_operator, matrixData.matrix_switch_operator_index[0][current_matrix_i][current_matrix_function_i]);
    lv_obj_add_event_cb(result.dd_operator, dd_operator_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Critical for alignment
    lv_obj_set_size(result.label_operator, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.dd_operator, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 7: Flux Value ---------------------------------------------------------- */
    
    lv_obj_t * row7 = lv_obj_create(result.panel);
    lv_obj_set_size(row7, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row7, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row7, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row7, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row7, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row7, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row7,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row7, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row7, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row7, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row7, LV_DIR_NONE);}

    result.label_flux = lv_label_create(row7);
    lv_label_set_text(result.label_flux, "Flux:");
    lv_obj_set_size(result.label_flux, label_width, row_height);
    lv_obj_set_style_text_font(result.label_flux, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_flux, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_flux, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_flux = create_textarea(
        row7,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_flux, String(matrixData.flux_value[0][current_matrix_i]).c_str());
    lv_obj_add_event_cb(result.ta_flux, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_flux, &matrix_flux_ctx);

    // Critical for alignment
    lv_obj_set_size(result.label_flux, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_flux, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 8: Output Mode --------------------------------------------------------- */
    
    lv_obj_t * row8 = lv_obj_create(result.panel);
    lv_obj_set_size(row8, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row8, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row8, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row8, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row8, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row8, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row8,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row8, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row8, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row8, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row8, LV_DIR_NONE);}
    
    result.label_output_mode = lv_label_create(row8);
    lv_label_set_text(result.label_output_mode, "Output:");
    lv_obj_set_size(result.label_output_mode, label_width, row_height);
    lv_obj_set_style_text_font(result.label_output_mode, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_output_mode, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_output_mode, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.dd_output_mode = create_dropdown_menu(
        row8,
        NULL,
        0,
        value_width,
        row_height,
        LV_ALIGN_CENTER,
        0,
        0,
        font_sub
    );
    char dd_output_mode_name[MAX_GLOBAL_ELEMENT_SIZE];
    for (int i = 0; i < MAX_MATRIX_OUTPUT_MODES; i++) {
        snprintf(dd_output_mode_name, sizeof(dd_output_mode_name), "%s", matrixData.output_mode_names[i]);
        lv_dropdown_add_option(result.dd_output_mode, dd_output_mode_name, LV_DROPDOWN_POS_LAST);
    }
    lv_dropdown_set_selected(result.dd_output_mode, matrixData.output_mode[0][current_matrix_i]);
    lv_obj_add_event_cb(result.dd_output_mode, dd_output_mode_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Critical for alignment
    lv_obj_set_size(result.label_output_mode, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.dd_output_mode, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 9: Output PWM ---------------------------------------------------------- */
    
    lv_obj_t * row9 = lv_obj_create(result.panel);
    lv_obj_set_size(row9, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row9, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row9, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row9, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row9, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row9, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row9,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row9, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row9, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row9, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row9, LV_DIR_NONE);}

    result.label_output_pwm_0 = lv_label_create(row9);
    lv_label_set_text(result.label_output_pwm_0, "PWM0:");
    lv_obj_set_size(result.label_output_pwm_0, label_width, row_height);
    lv_obj_set_style_text_font(result.label_output_pwm_0, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_output_pwm_0, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_output_pwm_0, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_pwm_0 = create_textarea(
        row9,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_pwm_0, String(matrixData.output_pwm[0][current_matrix_i][INDEX_MATRIX_SWITCH_PWM_OFF]).c_str());
    lv_obj_add_event_cb(result.ta_pwm_0, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_pwm_0, &matrix_output_pwm_0_ctx);

    // Critical for alignment
    lv_obj_set_size(result.label_output_pwm_0, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_pwm_0, value_width, LV_SIZE_CONTENT);

    /* --- ROW 10: Output PWM ---------------------------------------------------------- */
    
    lv_obj_t * row10 = lv_obj_create(result.panel);
    lv_obj_set_size(row10, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row10, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row10, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row10, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row10, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row10, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row10,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row10, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row10, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row10, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row10, LV_DIR_NONE);}

    result.label_output_pwm_1 = lv_label_create(row10);
    lv_label_set_text(result.label_output_pwm_1, "PWM1:");
    lv_obj_set_size(result.label_output_pwm_1, label_width, row_height);
    lv_obj_set_style_text_font(result.label_output_pwm_1, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_output_pwm_1, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_output_pwm_1, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_pwm_1 = create_textarea(
        row10,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_pwm_1, String(matrixData.output_pwm[0][current_matrix_i][INDEX_MATRIX_SWITCH_PWM_ON]).c_str());
    lv_obj_add_event_cb(result.ta_pwm_1, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_pwm_1, &matrix_output_pwm_1_ctx);

    // Critical for alignment
    lv_obj_set_size(result.label_output_pwm_1, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_pwm_1, value_width, LV_SIZE_CONTENT);

    /* --- ROW 11: Inverted Logic ------------------------------------------------------ */
    
    lv_obj_t * row11 = lv_obj_create(result.panel);
    lv_obj_set_size(row11, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row11, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row11, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row11, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row11, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row11, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row11,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row11, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row11, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row11, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row11, LV_DIR_NONE);}

    result.label_inverted_logic = lv_label_create(row11);
    lv_label_set_text(result.label_inverted_logic, "Inverted:");
    lv_obj_set_size(result.label_inverted_logic, label_width, row_height);
    lv_obj_set_style_text_font(result.label_inverted_logic, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_inverted_logic, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_inverted_logic, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.dd_inverted_logic = create_dropdown_menu(
        row11,
        NULL,
        0,
        value_width,
        row_height,
        LV_ALIGN_CENTER,
        0,
        0,
        font_sub
    );
    char dd_inverted_logic_name[MAX_GLOBAL_ELEMENT_SIZE];
    snprintf(dd_inverted_logic_name, sizeof(dd_inverted_logic_name), "%s", "False");
    lv_dropdown_add_option(result.dd_inverted_logic, dd_inverted_logic_name, LV_DROPDOWN_POS_LAST);
    snprintf(dd_inverted_logic_name, sizeof(dd_inverted_logic_name), "%s", "True");
    lv_dropdown_add_option(result.dd_inverted_logic, dd_inverted_logic_name, LV_DROPDOWN_POS_LAST);
    lv_dropdown_set_selected(result.dd_inverted_logic, matrixData.matrix_switch_inverted_logic[0][current_matrix_i][current_matrix_function_i]);
    lv_obj_add_event_cb(result.dd_inverted_logic, dd_inverted_logic_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Critical for alignment
    lv_obj_set_size(result.label_inverted_logic, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.dd_inverted_logic, value_width, LV_SIZE_CONTENT);

    /* --- ROW 12: Index Map Slot ------------------------------------------------ */
    
    lv_obj_t * row12 = lv_obj_create(result.panel);
    lv_obj_set_size(row12, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row12, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row12, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row12, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row12, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row12, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row12,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row12, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row12, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row12, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row12, LV_DIR_NONE);}
    
    result.label_map_slot = lv_label_create(row12);
    lv_label_set_text(result.label_map_slot, "Map Slot:");
    lv_obj_set_size(result.label_map_slot, label_width, row_height);
    lv_obj_set_style_text_font(result.label_map_slot, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_map_slot, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_map_slot, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.dd_map_slot = create_dropdown_menu(
        row12,
        NULL,
        0,
        value_width,
        row_height,
        LV_ALIGN_CENTER,
        0,
        0,
        font_sub
    );
    char dd_map_slot_name[MAX_GLOBAL_ELEMENT_SIZE];
    for (int i = 0; i < MAX_MAP_SLOTS; i++) {
        snprintf(dd_map_slot_name, sizeof(dd_map_slot_name), "%s", String(i).c_str());
        lv_dropdown_add_option(result.dd_map_slot, dd_map_slot_name, LV_DROPDOWN_POS_LAST);
    }
    lv_dropdown_set_selected(result.dd_map_slot, matrixData.index_mapped_value[0][current_matrix_i]);
    lv_obj_add_event_cb(result.dd_map_slot, dd_map_slot_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Critical for alignment
    lv_obj_set_size(result.label_map_slot, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.dd_map_slot, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 13: Port Map ----------------------------------------------------------- */
    
    lv_obj_t * row13 = lv_obj_create(result.panel);
    lv_obj_set_size(row13, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row13, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row13, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row13, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row13, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row13, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row13,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row13, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row13, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row13, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row13, LV_DIR_NONE);}

    result.label_port_map = lv_label_create(row13);
    lv_label_set_text(result.label_port_map, "Port:");
    lv_obj_set_size(result.label_port_map, label_width, row_height);
    lv_obj_set_style_text_font(result.label_port_map, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_port_map, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_port_map, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_port_map = create_textarea(
        row13,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_port_map, String(matrixData.matrix_port_map[0][current_matrix_i]).c_str());
    lv_obj_add_event_cb(result.ta_port_map, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_port_map, &matrix_port_map_ctx);

    // Critical for alignment
    lv_obj_set_size(result.label_port_map, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_port_map, value_width, LV_SIZE_CONTENT);
    
    return result;
}

/** ----------------------------------------------------------------------------------------
 * @brief Create Mapping Configuration Container
 * 
 * Creates a container panel with all elements needed to display/edit matrix function data:
 * - Map Slot
 * - Configuration 0
 * - Configuration 1
 * - Configuration 2
 * - Configuration 3
 * - Configuration 4
 * - Configuration 5
 * - Map Mode
 * 
 * @param scr Pointer to the parent screen object.
 * @param width_px Width of the container in pixels.
 * @param height_px Height of the container in pixels.
 * @param alignment Alignment of the container within its parent/screen.
 * @param pos_x X position of the container.
 * @param pos_y Y position of the container.
 * @return mapping_config_container_t struct with all UI element pointers.
 */
mapping_config_container_t create_mapping_config_container(
    lv_obj_t * scr,
    int32_t width_px,
    int32_t height_px,
    int32_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
) {
    mapping_config_container_t result = {0};
    
    // Layout constants
    const int32_t padding = 0;
    const int32_t row_spacing = 0;
    const int32_t row_height = 34;
    const int32_t label_width = 30;
    const int32_t value_width = width_px-label_width-2*padding;
    
    /* --- MAIN PANEL ------------------------------------------------------------------ */
    
    result.panel = lv_obj_create(scr);
    lv_obj_set_size(result.panel, width_px, height_px);
    lv_obj_align(result.panel, (lv_align_t)alignment, pos_x, pos_y);

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(result.panel, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(result.panel, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(result.panel, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(result.panel, LV_DIR_NONE);}
    
    // Panel style
    lv_obj_set_style_radius(result.panel, general_radius, LV_PART_MAIN);
    lv_obj_set_style_outline_width(result.panel, outline_width, LV_PART_MAIN);
    lv_obj_set_style_outline_color(result.panel, menu_outline_color, LV_PART_MAIN);
    lv_obj_set_style_border_width(result.panel, border_width, LV_PART_MAIN);
    lv_obj_set_style_border_color(result.panel, menu_border_color, LV_PART_MAIN);
    lv_obj_set_style_bg_color(result.panel, menu_bg_color, LV_PART_MAIN);
    lv_obj_set_style_pad_all(result.panel, padding, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(result.panel, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(result.panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(result.panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(result.panel, row_spacing, LV_PART_MAIN);

    /* --- ROW 1: Function Slot ------------------------------------------------------- */
    lv_obj_t * row1 = lv_obj_create(result.panel);
    lv_obj_set_size(row1, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row1, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row1, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row1,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row1, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row1, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row1, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row1, LV_DIR_NONE);}
    
    result.slot = lv_label_create(row1);
    lv_label_set_text(result.slot, "S:");
    lv_obj_set_size(result.slot, label_width, row_height);
    lv_obj_set_style_text_font(result.slot, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.slot, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.slot, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);

    result.label_slot = lv_label_create(row1);
    lv_label_set_text(result.label_slot, String(matrixData.index_mapped_value[0][current_matrix_i]).c_str());
    lv_obj_set_size(result.label_slot, value_width, row_height);
    lv_obj_set_style_text_font(result.label_slot, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.label_slot, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.label_slot, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_size(result.slot, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.label_slot, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 2: Function Name ------------------------------------------------------- */
    
    lv_obj_t * row2 = lv_obj_create(result.panel);
    lv_obj_set_size(row2, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row2, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row2, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row2,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row2, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row2, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row2, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row2, LV_DIR_NONE);}
    
    // C0
    result.c0 = lv_label_create(row2);
    lv_label_set_text(result.c0, "C0:");
    lv_obj_set_size(result.c0, label_width, row_height);
    lv_obj_set_style_text_font(result.c0, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.c0, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.c0, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    // Select C0
    result.dd_c0 = create_dropdown_menu(
        row2,
        NULL,
        0,
        value_width,
        row_height,
        LV_TEXT_ALIGN_RIGHT,
        0,
        0,
        font_sub
    );
    char dd_c0_name[MAX_GLOBAL_ELEMENT_SIZE];
    for (int i = 0; i < MAX_MAPPABLE_VALUES; i++) {
        snprintf(dd_c0_name, sizeof(dd_c0_name), "%s", mappingData.char_map_value[i]);
        lv_dropdown_add_option(result.dd_c0, dd_c0_name, LV_DROPDOWN_POS_LAST);
    }
    lv_dropdown_set_selected(result.dd_c0, mappingData.mapping_config[0][matrixData.index_mapped_value[0][current_matrix_i]][INDEX_MAP_C0]);
    lv_obj_add_event_cb(result.dd_c0, dd_c0_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Critical for alignment
    lv_obj_set_size(result.c0, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.dd_c0, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 3: C1 Value ------------------------------------------------------------- */
    
    lv_obj_t * row3 = lv_obj_create(result.panel);
    lv_obj_set_size(row3, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row3, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row3, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row3, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row3, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row3, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row3,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row3, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row3, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row3, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row3, LV_DIR_NONE);}
    
    result.c1 = lv_label_create(row3);
    lv_label_set_text(result.c1, "C1:");
    lv_obj_set_size(result.c1, label_width, row_height);
    lv_obj_set_style_text_font(result.c1, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.c1, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.c1, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_c1 = create_textarea(
        row3,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_c1, String(mappingData.mapping_config[0][current_matrix_i][INDEX_MAP_C1]).c_str());
    lv_obj_add_event_cb(result.ta_c1, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_c1, &mapping_c1_ctx);

    // Critical for alignment
    lv_obj_set_size(result.c1, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_c1, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 4: C2 Value ------------------------------------------------------------- */
    
    lv_obj_t * row4 = lv_obj_create(result.panel);
    lv_obj_set_size(row4, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row4, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row4, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row4, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row4, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row4, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row4,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row4, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row4, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row4, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row4, LV_DIR_NONE);}
    
    result.c2 = lv_label_create(row4);
    lv_label_set_text(result.c2, "C2:");
    lv_obj_set_size(result.c2, label_width, row_height);
    lv_obj_set_style_text_font(result.c2, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.c2, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.c2, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_c2 = create_textarea(
        row4,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_c2, String(mappingData.mapping_config[0][current_matrix_i][INDEX_MAP_C2]).c_str());
    lv_obj_add_event_cb(result.ta_c2, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_c2, &mapping_c2_ctx);

    // Critical for alignment
    lv_obj_set_size(result.c2, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_c2, value_width, LV_SIZE_CONTENT);

    /* --- ROW 5: C3 Value ------------------------------------------------------------- */
    
    lv_obj_t * row5 = lv_obj_create(result.panel);
    lv_obj_set_size(row5, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row5, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row5, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row5, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row5, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row5, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row5,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row5, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row5, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row5, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row5, LV_DIR_NONE);}
    
    result.c3 = lv_label_create(row5);
    lv_label_set_text(result.c3, "C3:");
    lv_obj_set_size(result.c3, label_width, row_height);
    lv_obj_set_style_text_font(result.c3, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.c3, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.c3, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_c3 = create_textarea(
        row5,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_c3, String(mappingData.mapping_config[0][current_matrix_i][INDEX_MAP_C3]).c_str());
    lv_obj_add_event_cb(result.ta_c3, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_c3, &mapping_c3_ctx);

    // Critical for alignment
    lv_obj_set_size(result.c3, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_c3, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 6: C4 Value ------------------------------------------------------------ */
    
    lv_obj_t * row6 = lv_obj_create(result.panel);
    lv_obj_set_size(row6, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row6, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row6, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row6, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row6, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row6, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row6,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row6, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row6, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row6, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row6, LV_DIR_NONE);}
    
    result.c4 = lv_label_create(row6);
    lv_label_set_text(result.c4, "C4:");
    lv_obj_set_size(result.c4, label_width, row_height);
    lv_obj_set_style_text_font(result.c4, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.c4, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.c4, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_c4 = create_textarea(
        row6,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_c4, String(mappingData.mapping_config[0][current_matrix_i][INDEX_MAP_C4]).c_str());
    lv_obj_add_event_cb(result.ta_c4, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_c4, &mapping_c4_ctx);

    // Critical for alignment
    lv_obj_set_size(result.c4, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_c4, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 7: Map Mode ---------------------------------------------------------- */
    
    lv_obj_t * row7 = lv_obj_create(result.panel);
    lv_obj_set_size(row7, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row7, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row7, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row7, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row7, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row7, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row7,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row7, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row7, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row7, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row7, LV_DIR_NONE);}

    result.c5 = lv_label_create(row7);
    lv_label_set_text(result.c5, "C5:");
    lv_obj_set_size(result.c5, label_width, row_height);
    lv_obj_set_style_text_font(result.c5, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.c5, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.c5, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.ta_c5 = create_textarea(
        row7,     // lv_obj_t
        value_width,             // width px
        row_height,              // height px
        LV_ALIGN_CENTER, // alignment
        0,            // pos x
        0,              // pos y
        true,            // one line
        LV_TXT_ALNUMDEC, // accepted char*
        "",              // placeholder text
        false,           // transparent bg
        false,           // show scrollbars
        false,           // enable scrolling
        font_sub // font for labels,
    );
    lv_textarea_set_text(result.ta_c5, String(mappingData.mapping_config[0][matrixData.index_mapped_value[0][current_matrix_i]][INDEX_MAP_C5]).c_str());
    lv_obj_add_event_cb(result.ta_c5, set_keyboard_context_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_user_data(result.ta_c5, &mapping_c5_ctx);

    // Critical for alignment
    lv_obj_set_size(result.c5, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.ta_c5, value_width, LV_SIZE_CONTENT);
    
    /* --- ROW 8: Map Mode --------------------------------------------------------- */
    
    lv_obj_t * row8 = lv_obj_create(result.panel);
    lv_obj_set_size(row8, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row8, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row8, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row8, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row8, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row8, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row8,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );

    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row8, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row8, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row8, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row8, LV_DIR_NONE);}
    
    // Map Mode
    result.mode = lv_label_create(row8);
    lv_label_set_text(result.mode, "M:");
    lv_obj_set_size(result.mode, label_width, row_height);
    lv_obj_set_style_text_font(result.mode, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.mode, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.mode, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    // Select Map Mode
    result.dd_mode = create_dropdown_menu(
        row8,
        NULL,
        0,
        value_width,
        row_height,
        LV_ALIGN_CENTER,
        0,
        0,
        font_sub
    );
    char dd_mode_name[MAX_GLOBAL_ELEMENT_SIZE];
    for (int i = 0; i < MAX_MAP_MODES; i++) {
        snprintf(dd_mode_name, sizeof(dd_mode_name), "%s", String(i).c_str()); // todo: make human map mode name array in mapping.cpp
        lv_dropdown_add_option(result.dd_mode, dd_mode_name, LV_DROPDOWN_POS_LAST);
    }
    lv_dropdown_set_selected(result.dd_mode, mappingData.map_mode[0][current_matrix_i]);
    lv_obj_add_event_cb(result.dd_mode, dd_mode_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Critical for alignment
    lv_obj_set_size(result.mode, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.dd_mode, value_width, LV_SIZE_CONTENT);

    /* --- ROW 9: Map Input Value --------------------------------------------------- */
    
    lv_obj_t * row9 = lv_obj_create(result.panel);
    lv_obj_set_size(row9, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row9, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row9, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row9, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row9, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row9, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row9,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row9, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row9, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row9, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row9, LV_DIR_NONE);}

    result.input_value = lv_label_create(row9);
    lv_label_set_text(result.input_value, "I:");
    lv_obj_set_size(result.input_value, label_width, row_height);
    lv_obj_set_style_text_font(result.input_value, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.input_value, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.input_value, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.value_input = lv_label_create(row9);
    lv_label_set_text(result.value_input, "- - -");
    lv_obj_set_size(result.value_input, value_width, row_height);
    lv_obj_set_style_text_font(result.value_input, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.value_input, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.value_input, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_size(result.input_value, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.value_input, value_width, LV_SIZE_CONTENT);

    /* --- ROW 10: Map Result ---------------------------------------------------------- */
    
    lv_obj_t * row10 = lv_obj_create(result.panel);
    lv_obj_set_size(row10, LV_PCT(100), row_height);
    lv_obj_set_flex_flow(row10, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(row10, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row10, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row10, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row10, padding, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_flex_align(
        row10,
        LV_FLEX_ALIGN_START,   // main axis (left‑right)
        LV_FLEX_ALIGN_CENTER,  // cross axis (top‑bottom)
        LV_FLEX_ALIGN_CENTER   // track alignment
    );
    
    // Show scrollbar
    if (show_scrollbar) {lv_obj_set_scrollbar_mode(row10, LV_SCROLLBAR_MODE_AUTO);}
    else {lv_obj_set_scrollbar_mode(row10, LV_SCROLLBAR_MODE_OFF);}

    // Enable scrolling
    if (enable_scrolling) {lv_obj_set_scroll_dir(row10, LV_DIR_ALL);}
    else {lv_obj_set_scroll_dir(row10, LV_DIR_NONE);}

    result.map_result = lv_label_create(row10);
    lv_label_set_text(result.map_result, "O:");
    lv_obj_set_size(result.map_result, label_width, row_height);
    lv_obj_set_style_text_font(result.map_result, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.map_result, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.map_result, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    
    result.value_map_result = lv_label_create(row10);
    lv_label_set_text(result.value_map_result, String(mappingData.mapped_value[0][matrixData.index_mapped_value[0][current_matrix_i]]).c_str());
    lv_obj_set_size(result.value_map_result, value_width, row_height);
    lv_obj_set_style_text_font(result.value_map_result, font_title, LV_PART_MAIN);
    lv_obj_set_style_text_color(result.value_map_result, menu_text_color, LV_PART_MAIN);
    lv_obj_set_style_text_align(result.value_map_result, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);

    // Critical for alignment
    lv_obj_set_size(result.map_result, label_width, LV_SIZE_CONTENT);
    lv_obj_set_size(result.value_map_result, value_width, LV_SIZE_CONTENT);

    return result;
}

/** ----------------------------------------------------------------------------------------
 * @brief Display image from SD card.
 * 
 * @param scr Pointer to the parent screen object.
 * @param filename Path to the image file on the SD card.
 * @param width_px Width of the image in pixels.
 * @param height_px Height of the image in pixels.
 * @param color_depth_bits Color depth of the image in bits (e.g., 16 for RGB565).
 * @param pos_x X position of the image within its parent/screen.
 * @param pos_y Y position of the image within its parent/screen.
 * @param alignment Alignment of the image within its parent/screen.
 * 
 * @note Requires sufficient external RAM (PSRAM) for all file bytes.
 * 
 */
sdcard_image_t * create_image_from_sdcard(
    lv_obj_t * scr,
    const char * filename,
    uint32_t width_px,
    uint32_t height_px,
    uint32_t color_depth_bits,
    uint32_t pos_x,
    uint32_t pos_y,
    int32_t alignment,
    bool discard_after_display
) {

    // Allocate sdcard_image_t structure
    sdcard_image_t * sdcard_image = (sdcard_image_t *)heap_caps_malloc(sizeof(sdcard_image_t), MALLOC_CAP_SPIRAM);
    if (!sdcard_image) {
        printf("ERROR: PSRAM allocation for sdcard_image_t failed\n");
        return NULL;
    }

    // Get file size
    sdcard_image->f_size = get_file_size(filename);
    if (sdcard_image->f_size == 0) {
        printf("ERROR: Cannot get size of %s\n", filename);
        heap_caps_free(sdcard_image);
        return NULL;
    }

    // Load image to PSRAM
    sdcard_image->bytes_in_psram = load_file_bytes_to_psram(filename, sdcard_image->f_size);
    if (!sdcard_image->bytes_in_psram) {
        printf("ERROR: Failed to load %s to PSRAM\n", filename);
        heap_caps_free(sdcard_image);
        return NULL;
    }

    // Create descriptor from loaded data
    sdcard_image->dsc.header.cf = color_depth_bits == 24 ? LV_COLOR_FORMAT_RGB565A8 : LV_COLOR_FORMAT_RGB565;
    sdcard_image->dsc.header.w = width_px;
    sdcard_image->dsc.header.h = height_px;
    sdcard_image->dsc.data_size = sdcard_image->f_size;
    sdcard_image->dsc.data = sdcard_image->bytes_in_psram; // Pointer to PSRAM data

    printf("Created image descriptor: cf=%d %ldx%ld size=%ld\n", 
           sdcard_image->dsc.header.cf, width_px, height_px, sdcard_image->f_size);

    // Create LVGL image object
    sdcard_image->lv_image_obj = lv_img_create(scr);
    if (!sdcard_image->lv_image_obj) {
        printf("ERROR: Failed to create LVGL image\n");
        heap_caps_free(sdcard_image->bytes_in_psram);
        heap_caps_free(sdcard_image);
        return NULL;
    }

    // Position and size
    lv_obj_set_pos(sdcard_image->lv_image_obj, pos_x, pos_y);
    if (alignment == LV_ALIGN_CENTER) {
        lv_obj_center(sdcard_image->lv_image_obj);
    }
    lv_obj_set_size(sdcard_image->lv_image_obj, width_px, height_px);
    
    // Set image source
    lv_img_set_src(sdcard_image->lv_image_obj, &sdcard_image->dsc);

    // Discard or return
    if (discard_after_display) {
        heap_caps_free(sdcard_image->bytes_in_psram);
        heap_caps_free(sdcard_image);
        printf("Discarded image data from PSRAM\n");
        return NULL;  // Return NULL when discarding
    }
    printf("Image created successfully at %p\n", sdcard_image);
    return sdcard_image;  // Return full structure
}

void cleanup_loading_image() {
    if (loading_image) {
        // Delete LVGL image object
        if (loading_image->lv_image_obj) {
            lv_obj_del(loading_image->lv_image_obj);
        }
        // Free PSRAM image data
        if (loading_image->bytes_in_psram) {
            heap_caps_free(loading_image->bytes_in_psram);
        }
        // Free the struct itself
        heap_caps_free(loading_image);
        loading_image = NULL;
        printf("Loading screen image cleaned up\n");
    }
}

void default_screen_objects(lv_obj_t * scr) {

    cleanup_loading_image();

    // Clean up title bar
    if (main_title_bar.panel) {
        // lv_obj_del(main_title_bar.panel);
        main_title_bar = {0};  // Reset struct
    }
    // Clean up system tray
    if (system_tray.panel) {
        // lv_obj_del(system_tray.panel);
        system_tray = {0};  // Reset struct
    }
    // Clean up keyboards
    if (kb_numdec.kb) {
        // lv_obj_del(kb_numdec.kb);
        kb_numdec = {0};  // Reset struct
    }
    if (kb_alnumsym.kb) {
        // lv_obj_del(kb_alnumsym.kb);
        kb_alnumsym = {0};  // Reset struct
    }

    // Clean up astro clock
    astro_clock_end();

    // Set background color for main part of the screen
    lv_obj_set_style_bg_color(scr, lv_color_make(0, 0, 0), LV_PART_MAIN);

    // -------------------------------- Keypad Num ------------------------------------ //

    // Create keyboard (bootstrapped)
    kb_numdec = create_keyboard(
        scr,            // lv_obj_t
        600,                    // width px
        250,                    // height px
        LV_ALIGN_CENTER,        // alignment
        0,                      // pos x
        23,                     // pos y
        10,                     // padding between kb and text area
        36,                     // text area height
        LV_KEYBOARD_MODE_NUMBER // keyboard mode
    );
    
    // Plug in keyboard callback for kb_numdec
    lv_obj_add_event_cb(kb_numdec.kb, keyboard_event_cb, LV_EVENT_VALUE_CHANGED, &kb_numdec);
    lv_obj_add_flag(kb_numdec.kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(kb_numdec.ta, LV_OBJ_FLAG_HIDDEN);

    // -------------------------------- Keyboard ----------------------------------- //

    // Create keyboard (bootstrapped)
    kb_alnumsym = create_keyboard(
        scr,            // lv_obj_t
        600,                    // width px
        250,                    // height px
        LV_ALIGN_CENTER,        // alignment
        0,                      // pos x
        23,                     // pos y
        10,                     // padding between kb and text area
        36,                     // text area height
        LV_KEYBOARD_MODE_USER_1 // keyboard mode
    );
    
    // Plug in keyboard callback for kb_alnumsym
    lv_obj_add_event_cb(kb_alnumsym.kb, keyboard_event_cb, LV_EVENT_VALUE_CHANGED, &kb_alnumsym);
    lv_obj_add_flag(kb_alnumsym.kb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(kb_alnumsym.ta, LV_OBJ_FLAG_HIDDEN);

    // -------------------------------- Title Bar --------------------------------- //

    main_title_bar = create_title_bar(
        scr, // parent
        720, // width px
        80,  // height px
        LV_ALIGN_TOP_MID,
        0, // pos x
        0, // pos y
        false, // show scrollbar
        false  // enable scrollbar
    );

    // ------------------------------ System Tray --------------------------------- //

    // Create system tray
    system_tray = create_system_tray(scr);

    // Plug in event callback for screen click events
    lv_obj_add_event_cb(scr, screen_tap_cb, LV_EVENT_CLICKED, NULL);

    // Plug in event callback for screen swipe events
    lv_obj_add_event_cb(scr, screen_swipe_cb, LV_EVENT_GESTURE, NULL);
}

void display_loading_screen() {

    lv_obj_t * current_screen = lv_scr_act();

    if (!lv_obj_is_valid(loading_screen)) {loading_screen = lv_obj_create(NULL);}
    if (current_screen == loading_screen) return;

    lv_obj_set_style_bg_color(loading_screen, lv_color_make(0, 0, 0), LV_PART_MAIN);
    
    // Create image from sdcard and store reference for cleanup
    loading_image = create_image_from_sdcard(
        loading_screen,      // parent object
        "/sdcard/logo1.bin", // filepath
        600,                 // width_px
        600,                 // height_px
        16,                  // color_depth_bits
        0,                   // pos_x
        0,                   // pos_y
        LV_ALIGN_CENTER,     // alignment
        false                // keep in memory until cleanup
    );
    
    lv_scr_load(loading_screen);
}

void display_home_screen() {

    lv_obj_t * current_screen = lv_scr_act();
    if (current_screen == home_screen) {return;}

    // Always create a fresh screen - the old home_screen may have been 
    // auto-deleted when switching away, making the pointer stale
    home_screen = lv_obj_create(NULL);

    default_screen_objects(home_screen);

    // -------------------------------- Astro Clock ----------------------------------- //

    // Initialize astro clock on main screen
    astro_clock_begin(
        home_screen,
        556,             // width px
        556,             // height px
        LV_ALIGN_CENTER, // alignment
        0,               // pos x
        0,               // pos y
        90               // angle offset
    );

    lv_scr_load_anim(home_screen, LV_SCR_LOAD_ANIM_NONE, 300, 0, true);  // last param = auto_del

    flag_display_home_screen = false;
}

void display_matrix_screen() {
    lv_obj_t * current_screen = lv_scr_act();
    if (current_screen == matrix_screen) {return;}

    // Always create a fresh screen - the old screen may have been auto-deleted
    matrix_screen = lv_obj_create(NULL);

    default_screen_objects(matrix_screen);

    matrix_overview_grid_1 = create_menu_grid(
        matrix_screen,        // parent screen
        15,                   // cols
        3,                    // rows
        30,                   // cell size px
        8,                    // outer padding
        8,                    // inner padding
        0,                    // pos x
        145,                  // pos y
        LV_ALIGN_TOP_MID,     // alignment
        radius_circle,        // item radius px
        15,                   // max cols visible (for scrollbar)
        3,                    // max rows visible (for scrollbar)
        false,                // show scrollbar
        false,                // enable scrolling
        LV_TEXT_ALIGN_CENTER, // text align
        &cobalt_alien_17      // font for labels
    );
    // Plug in general event callback handler
    uint32_t grid_menu_x_count = lv_obj_get_child_cnt(matrix_overview_grid_1);
    for(uint32_t i = 0; i < grid_menu_x_count; i++) {
        lv_obj_t * btn = lv_obj_get_child(matrix_overview_grid_1, i);
        lv_obj_add_event_cb(btn, matrix_overview_grid_1_event_cb, LV_EVENT_CLICKED, NULL);
    }

    // Create Function Panel
    mfc = create_matrix_function_container(
        matrix_screen,    // parent
        240,              // width px
        400,              // height px (fits 8 rows)
        LV_ALIGN_CENTER,  // alignment
        -50,              // pos x
        120,              // pos y
        false,            // show scrollbar
        false,            // enable scrolling
        &cobalt_alien_17, // font for titles,
        &cobalt_alien_17  // font for text,
    );

    // Create Mapping Panel
    mcc = create_mapping_config_container(
        matrix_screen,    // parent
        200,              // width px
        350,              // height px
        LV_ALIGN_RIGHT_MID, // alignment
        -60,              // pos x
        90,               // pos y
        false,            // show scrollbar
        false,            // enable scrolling
        &cobalt_alien_17, // font for titles,
        &cobalt_alien_17  // font for text,
    );

    // Computer Assist Toggle
    matrix_switch_computer_assist = create_button(
        matrix_screen,        // parent
        120,                  // width px
        32,                   // height px
        LV_ALIGN_LEFT_MID,    // alignment
        40,                   // pos x
        -60,                  // pos y
        "ASSIST",             // label text
        LV_TEXT_ALIGN_CENTER, // text align
        false,                // show scrollbar
        false,                // enable scrolling
        &cobalt_alien_17,     // font for labels,
        radius_rounded
    );
    lv_obj_add_event_cb(matrix_switch_computer_assist.button, current_matrix_computer_assist_event_cb, LV_EVENT_ALL, NULL);

    // Matrix Switch Override
    matrix_switch_override = create_button(
        matrix_screen,        // parent
        120,                  // width px
        32,                   // height px
        LV_ALIGN_LEFT_MID,    // alignment
        40,                   // pos x
        -10,                  // pos y
        "OVERRIDE",           // label text
        LV_TEXT_ALIGN_CENTER, // text align
        false,                // show scrollbar
        false,                // enable scrolling
        &cobalt_alien_25,     // font for labels,
        radius_rounded
    );
    lv_obj_add_event_cb(matrix_switch_override.button, current_matrix_override_off_event_cb, LV_EVENT_ALL, NULL);

    // Output Value
    matrix_switch_output_value = create_label(
        matrix_screen,        // parent
        120,                  // width
        32,                   // height
        LV_ALIGN_LEFT_MID,    // parent alignment
        40,                   // pos x
        60,                   // pos y
        "0",                  // initial text
        LV_TEXT_ALIGN_CENTER, // font alignment
        &cobalt_alien_17,     // font
        false,                // transparent background
        false,                // show scrollbar
        false,                // enable scrolling
        2,                    // outline width
        radius_rounded,       // outline radius
        1
    );

    // Select Matrix Slot
    dd_matrix_slot_select = create_dropdown_menu(
        matrix_screen,     // parent
        satioFileData.matix_filepaths, // menu items
        10,                // number of menu items
        240,               // width px
        34,                // height px
        LV_ALIGN_TOP_MID,  // alignment
        0,                 // pos x
        100,               // pos y
        &cobalt_alien_17   // font
    );
    // lv_dropdown_set_selected(dd_matrix_slot_select, matrixData.matrix_function[0][current_matrix_i][current_matrix_function_i]);
    lv_obj_add_event_cb(dd_matrix_slot_select, dd_matrix_slot_select_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // New Matrix
    matrix_new = create_button(
        matrix_screen,        // parent
        34,                   // width px
        34,                   // height px
        LV_ALIGN_TOP_LEFT,    // alignment
        130,                  // pos x
        100,                  // pos y
        "N",                  // label text
        LV_TEXT_ALIGN_CENTER, // text align
        false,                // show scrollbar
        false,                // enable scrolling
        &cobalt_alien_17,     // font for labels,
        radius_rounded
    );
    lv_obj_add_event_cb(matrix_new.button, matrix_new_event_cb, LV_EVENT_ALL, NULL);

    // Save Matrix
    matrix_save = create_button(
        matrix_screen,        // parent
        34,                   // width px
        34,                   // height px
        LV_ALIGN_TOP_LEFT,    // alignment
        184,                  // pos x
        100,                  // pos y
        "S",                  // label text
        LV_TEXT_ALIGN_CENTER, // text align
        false,                // show scrollbar
        false,                // enable scrolling
        &cobalt_alien_17,     // font for labels,
        radius_rounded
    );
    lv_obj_add_event_cb(matrix_save.button, matrix_save_event_cb, LV_EVENT_ALL, NULL);

    // Load Matrix
    matrix_load = create_button(
        matrix_screen,        // parent
        34,                   // width px
        34,                   // height px
        LV_ALIGN_TOP_RIGHT,   // alignment
        -184,                 // pos x
        100,                  // pos y
        "L",                  // label text
        LV_TEXT_ALIGN_CENTER, // text align
        false,                // show scrollbar
        false,                // enable scrolling
        &cobalt_alien_17,     // font for labels,
        radius_rounded
    );
    lv_obj_add_event_cb(matrix_load.button, matrix_load_event_cb, LV_EVENT_ALL, NULL);

    // Delete Matrix
    matrix_delete = create_button(
        matrix_screen,        // parent
        34,                   // width px
        34,                   // height px
        LV_ALIGN_TOP_RIGHT,   // alignment
        -130,                 // pos x
        100,                  // pos y
        "D",                  // label text
        LV_TEXT_ALIGN_CENTER, // text align
        false,                // show scrollbar
        false,                // enable scrolling
        &cobalt_alien_17,     // font for labels,
        radius_rounded
    );
    lv_obj_add_event_cb(matrix_delete.button, matrix_delete_event_cb, LV_EVENT_ALL, NULL);

    // current matrix switch output value
    // current matrix switch override value
    // current matrix switch mapped value

    // Map Slot

    // Map C0 - Map C5

    // Map Mode

    lv_scr_load_anim(matrix_screen, LV_SCR_LOAD_ANIM_NONE, 300, 0, true);  // last param = auto_del
    flag_display_matrix_screen = false;
}

void display_gps_screen() {

    lv_obj_t * current_screen = lv_scr_act();
    if (current_screen == gps_screen) return;

    // Always create a fresh screen
    gps_screen = lv_obj_create(NULL);

    default_screen_objects(gps_screen);

    lv_scr_load_anim(gps_screen, LV_SCR_LOAD_ANIM_NONE, 300, 0, true);  // last param = auto_del
    flag_display_gps_screen = false;
}

void display_gyro_screen() {

    lv_obj_t * current_screen = lv_scr_act();
    if (current_screen == gyro_screen) return;

    // Always create a fresh screen
    gyro_screen = lv_obj_create(NULL);

    default_screen_objects(gyro_screen);

    lv_scr_load_anim(gyro_screen, LV_SCR_LOAD_ANIM_NONE, 300, 0, true);  // last param = auto_del
    flag_display_gyro_screen = false;
}

void display_disp_screen() {

    lv_obj_t * current_screen = lv_scr_act();
    if (current_screen == disp_screen) return;

    // Always create a fresh screen
    disp_screen = lv_obj_create(NULL);

    default_screen_objects(disp_screen);

    lv_scr_load_anim(disp_screen, LV_SCR_LOAD_ANIM_NONE, 300, 0, true);  // last param = auto_del
    flag_display_disp_screen = false;
}

void display_system_screen() {

    lv_obj_t * current_screen = lv_scr_act();
    if (current_screen == system_screen) return;

    // Always create a fresh screen
    system_screen = lv_obj_create(NULL);

    default_screen_objects(system_screen);

    lv_scr_load_anim(system_screen, LV_SCR_LOAD_ANIM_NONE, 300, 0, true);  // last param = auto_del
    flag_display_system_screen = false;
}

void display__screen() {

    lv_obj_t * current_screen = lv_scr_act();
    if (current_screen == _screen) return;

    // Always create a fresh screen
    _screen = lv_obj_create(NULL);

    default_screen_objects(_screen);

    lv_scr_load_anim(_screen, LV_SCR_LOAD_ANIM_NONE, 300, 0, true);  // last param = auto_del
    flag_display__screen = false;
}

/** ---------------------------------------------------------------------------------------
 * @brief Update display
 */
char ui_buf_matrix[256];

void update_display_on_timer(lv_timer_t * timer) {
    LV_UNUSED(timer);
    update_display();
};

void update_display() {

    // pause timer...
    lv_timer_pause(display_timer);
    
    if (kb_alnumsym.kb != NULL && lv_obj_is_valid(kb_alnumsym.kb)) {
        if (!lv_obj_has_flag(kb_alnumsym.kb, LV_OBJ_FLAG_HIDDEN)) {
            // Rainbow keyboard full outline
            lv_obj_set_style_outline_color(kb_alnumsym.kb, lv_color_hsv_to_rgb((current_hue + 300) % 360, 100, 100), LV_PART_MAIN);
            // Rainbow keyboard full keys
            lv_obj_set_style_text_color(kb_alnumsym.kb, lv_color_hsv_to_rgb((current_hue + 75) % 360, 100, 100), LV_PART_ITEMS);
            // Rainbow keyboard full checked keys
            lv_obj_set_style_text_color(kb_alnumsym.kb, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_ITEMS | LV_STATE_CHECKED);
            // Rainbow keyboard full text area outline
            lv_obj_set_style_outline_color(kb_alnumsym.ta, lv_color_hsv_to_rgb((current_hue + 300) % 360, 100, 100), LV_PART_MAIN);
            // Rainbow keyboard full text area text
            lv_obj_set_style_text_color(kb_alnumsym.ta, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);
        }
    }

    if (kb_numdec.kb != NULL && lv_obj_is_valid(kb_numdec.kb)) {
        if (!lv_obj_has_flag(kb_numdec.kb, LV_OBJ_FLAG_HIDDEN)) {
            // Rainbow keyboard numdec full outline
            lv_obj_set_style_outline_color(kb_numdec.kb, lv_color_hsv_to_rgb((current_hue + 300) % 360, 100, 100), LV_PART_MAIN);
            // Rainbow keyboard numdec full keys
            lv_obj_set_style_text_color(kb_numdec.kb, lv_color_hsv_to_rgb((current_hue + 75) % 360, 100, 100), LV_PART_ITEMS);
            // Rainbow keyboard numdec full checked keys
            lv_obj_set_style_text_color(kb_numdec.kb, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_ITEMS | LV_STATE_CHECKED);
            // Rainbow keyboard numdec full text area outline
            lv_obj_set_style_outline_color(kb_numdec.ta, lv_color_hsv_to_rgb((current_hue + 300) % 360, 100, 100), LV_PART_MAIN);
            // Rainbow keyboard numdec full text area text
            lv_obj_set_style_text_color(kb_numdec.ta, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);
        }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // ---------------------
    // Title Bar
    // ---------------------

    if (main_title_bar.panel) {

        // Title Bar Outline
        lv_obj_set_style_outline_color(main_title_bar.panel, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_MAIN);

        // Title Bar Local Time
        lv_label_set_text(main_title_bar.time_label, satioData.formatted_local_time_HHMMSS);
        lv_obj_set_style_text_color(main_title_bar.time_label, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);

        // Title Bar Local Date
        lv_label_set_text(main_title_bar.date_label, satioData.formatted_local_short_date_DDMMYY);
        lv_obj_set_style_text_color(main_title_bar.date_label, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);

        // GPS Sync
        if (satioData.gps_sync) {
            lv_obj_add_flag(main_title_bar.gps_signal_strength, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(main_title_bar.datetime_sync, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_outline_color(main_title_bar.datetime_sync, lv_color_make(0, 255, 0), LV_PART_MAIN);
            lv_obj_set_style_text_color(main_title_bar.datetime_sync, lv_color_make(0, 255, 0), LV_PART_MAIN);
        }
        // GPS Signal
        else {
            lv_obj_add_flag(main_title_bar.datetime_sync, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(main_title_bar.gps_signal_strength, LV_OBJ_FLAG_HIDDEN);
            String gps_signal_text = "" + String(gnggaData.satellite_count) + ":" + String(gnggaData.gps_precision_factor) + "";
            lv_obj_set_style_outline_color(main_title_bar.gps_signal_strength, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(main_title_bar.gps_signal_strength, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);
            lv_label_set_text(main_title_bar.gps_signal_strength, gps_signal_text.c_str());
        }

        // SD Card Mounted / Success Flag
        if (sdcardFlagData.success_flag==2) {
            lv_obj_set_style_outline_color(main_title_bar.sdcard_mounted, lv_color_make(0, 255, 0), LV_PART_MAIN);
            lv_obj_set_style_text_color(main_title_bar.sdcard_mounted, lv_color_make(0, 255, 0), LV_PART_MAIN);
            lv_label_set_text(main_title_bar.sdcard_mounted, "ok");
        }
        else if (sdcardFlagData.success_flag==1) {
            lv_obj_set_style_outline_color(main_title_bar.sdcard_mounted, lv_color_make(255, 0, 0), LV_PART_MAIN);
            lv_obj_set_style_text_color(main_title_bar.sdcard_mounted, lv_color_make(255, 0, 0), LV_PART_MAIN);
            lv_label_set_text(main_title_bar.sdcard_mounted, "!");
        }
        else {
            if (sdcardData.sdcard_mounted) {
                lv_obj_set_style_outline_color(main_title_bar.sdcard_mounted, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_MAIN);
                lv_obj_set_style_text_color(main_title_bar.sdcard_mounted, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);
                lv_label_set_text(main_title_bar.sdcard_mounted, "SD");
            }
            else {
                lv_obj_set_style_outline_color(main_title_bar.sdcard_mounted, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_MAIN);
                lv_obj_set_style_text_color(main_title_bar.sdcard_mounted, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);
                lv_label_set_text(main_title_bar.sdcard_mounted, "SD!");
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // ---------------------
    // System Tray
    // ---------------------
    if (system_tray.is_open) {

        // Rainbow System Tray Outline
        lv_obj_set_style_outline_color(system_tray.panel, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_MAIN);

        // Rainbow System Tray Brightness Slider Outline
        lv_obj_set_style_outline_color(system_tray.slider_brightness, lv_color_hsv_to_rgb((current_hue + 300) % 360, 100, 100), LV_PART_MAIN);

        // Rainbow System Tray Brightness Slider Knob
        lv_obj_set_style_bg_color(system_tray.slider_brightness, lv_color_hsv_to_rgb((current_hue + 200) % 360, 100, 100), LV_PART_KNOB);

        // Rainbow System Tray Brightness Slider Indicator
        lv_obj_set_style_outline_color(system_tray.slider_brightness, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_INDICATOR);

        // System Tray Local Time
        lv_label_set_text(system_tray.local_time, satioData.formatted_local_time_HHMMSS);
        lv_obj_set_style_text_color(system_tray.local_time, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);

        // System Tray Local Date
        lv_label_set_text(system_tray.local_date, satioData.formatted_local_short_date_DDMMYY);
        lv_obj_set_style_text_color(system_tray.local_date, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);

        // System Tray Human Date
        String human_date = String(satioData.local_wday_name) + " " + String(satioData.local_mday) + " " + String(satioData.local_month_name);
        // printf("human date: %s\n", human_date.c_str());
        lv_label_set_text(system_tray.human_date, human_date.c_str());
        lv_obj_set_style_text_color(system_tray.human_date, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);

        // GPS Sync
        if (satioData.gps_sync) {
            lv_obj_add_flag(system_tray.gps_signal_strength, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(system_tray.datetime_sync, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_outline_color(system_tray.datetime_sync, lv_color_make(0, 255, 0), LV_PART_MAIN);
            lv_obj_set_style_text_color(system_tray.datetime_sync, lv_color_make(0, 255, 0), LV_PART_MAIN);
        }
        // GPS Signal
        else {
            lv_obj_add_flag(system_tray.datetime_sync, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(system_tray.gps_signal_strength, LV_OBJ_FLAG_HIDDEN);
            String gps_signal_text = "" + String(gnggaData.satellite_count) + ":" + String(gnggaData.gps_precision_factor) + "";
            lv_obj_set_style_outline_color(system_tray.gps_signal_strength, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(system_tray.gps_signal_strength, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);
            lv_label_set_text(system_tray.gps_signal_strength, gps_signal_text.c_str());
        }

        // SD Card Mounted
        if (sdcardData.sdcard_mounted) {
            lv_obj_set_style_outline_color(system_tray.sdcard_mounted, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(system_tray.sdcard_mounted, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);
            lv_label_set_text(system_tray.sdcard_mounted, "SD");
        }
        else {
            lv_obj_set_style_outline_color(system_tray.sdcard_mounted, lv_color_hsv_to_rgb((current_hue + 100) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(system_tray.sdcard_mounted, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_MAIN);
            lv_label_set_text(system_tray.sdcard_mounted, "SD!");
        }

        // Grid Menu 1
        if (system_tray.grid_menu_1) {
            uint32_t grid_child_cnt = lv_obj_get_child_cnt(system_tray.grid_menu_1);
            for(uint32_t i = 0; i < grid_child_cnt; i++) {
                lv_obj_t * btn = lv_obj_get_child(system_tray.grid_menu_1, i);
                lv_obj_set_style_outline_color(btn, lv_color_hsv_to_rgb((current_hue + 300) % 360, 100, 100), LV_PART_MAIN);
                // Get label
                lv_obj_t * label = lv_obj_get_child(btn, 0);
                if(label && lv_obj_has_class(label, &lv_label_class)) {
                    // Color label
                    lv_obj_set_style_text_color(label, 
                        lv_color_hsv_to_rgb((current_hue + (i * 60)) % 360, 100, 100), LV_PART_MAIN);
                }
            }
        }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (flag_display_home_screen==true) {display_home_screen(); vTaskDelay(10 / portTICK_PERIOD_MS);}
    else if (flag_display_matrix_screen==true) {display_matrix_screen(); vTaskDelay(10 / portTICK_PERIOD_MS);}
    else if (flag_display_gps_screen==true) {display_gps_screen(); vTaskDelay(10 / portTICK_PERIOD_MS);}
    else if (flag_display_gyro_screen==true) {display_gyro_screen(); vTaskDelay(10 / portTICK_PERIOD_MS);}
    else if (flag_display_disp_screen==true) {display_disp_screen(); vTaskDelay(10 / portTICK_PERIOD_MS);}
    else if (flag_display_system_screen==true) {display_system_screen(); vTaskDelay(10 / portTICK_PERIOD_MS);}
    else if (flag_display__screen==true) {display__screen(); vTaskDelay(10 / portTICK_PERIOD_MS);}

    // loading screen
    if (lv_scr_act() == loading_screen) {
    }

    // main screen
    else if (lv_scr_act() == home_screen) {
    }

    // matrix screen
    else if (lv_scr_act() == matrix_screen) {

        // Matrix Overview Grid 1
        if (matrix_overview_grid_1) {
            uint32_t grid_child_cnt = lv_obj_get_child_cnt(matrix_overview_grid_1);
            for(uint32_t i = 0; i < grid_child_cnt; i++) {
                lv_obj_t * btn = lv_obj_get_child(matrix_overview_grid_1, i);

                /**
                 * Outline: Computer Assisst
                 * level 0: level 1 + level 2  = false (dark gray)
                 * level 1: computer assist (red)
                 * level 2: computer intention (yellow)
                 * level 3: level 1 + level 2  = true (blue)
                */

                if (matrixData.computer_assist[0][i] == true && matrixData.computer_intention[0][i] == true) {
                    lv_obj_set_style_outline_color(btn, lv_color_make(0, 0, 255), LV_PART_MAIN);     // level 3: blue
                }
                else if (matrixData.computer_intention[0][i] == true) {
                    lv_obj_set_style_outline_color(btn, lv_color_make(255, 255, 0), LV_PART_MAIN);   // level 2: yellow
                }
                else if (matrixData.computer_assist[0][i] == true) {
                    lv_obj_set_style_outline_color(btn, lv_color_make(255, 0, 0), LV_PART_MAIN);     // level 1: red
                }
                else {
                    lv_obj_set_style_outline_color(btn, lv_color_make(58, 58, 58), LV_PART_MAIN);    // level 0: dark gray
                }
                
                // Get label
                lv_obj_t * label = lv_obj_get_child(btn, 0);
                if(label && lv_obj_has_class(label, &lv_label_class)) {

                    /**
                     * Text Color: Switch Intention 
                     * Level 4: switch intention (green)
                     */
                    if (matrixData.switch_intention[0][i]==true) {lv_obj_set_style_text_color(label, lv_color_make(0, 255, 0), LV_PART_MAIN);}
                    else {lv_obj_set_style_text_color(label, lv_color_make(58, 58, 58), LV_PART_MAIN);}
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // Matrix Slot
        if (dd_matrix_slot_select) {
            lv_obj_set_style_outline_color(dd_matrix_slot_select, lv_color_hsv_to_rgb((current_hue + 250) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(dd_matrix_slot_select, lv_color_hsv_to_rgb((current_hue + 150) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(dd_matrix_slot_select, lv_color_hsv_to_rgb((current_hue + 0) % 360, 100, 100), LV_PART_SELECTED);
            // lv_dropdown_set_selected(dd_matrix_slot_select, 0); // change to current index (modify load/save/new/delete)
        }

        // Matrix New
        if (matrix_new.panel) {
            lv_obj_set_style_outline_color(matrix_new.panel, lv_color_hsv_to_rgb((current_hue + 250) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(matrix_new.label, lv_color_hsv_to_rgb((current_hue + 150) % 360, 100, 100), LV_PART_MAIN);
        }

        // Matrix Save
        if (matrix_save.panel) {
            lv_obj_set_style_outline_color(matrix_save.panel, lv_color_hsv_to_rgb((current_hue + 250) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(matrix_save.label, lv_color_hsv_to_rgb((current_hue + 150) % 360, 100, 100), LV_PART_MAIN);
        }

        // Matrix Load
        if (matrix_load.panel) {
            lv_obj_set_style_outline_color(matrix_load.panel, lv_color_hsv_to_rgb((current_hue + 250) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(matrix_load.label, lv_color_hsv_to_rgb((current_hue + 150) % 360, 100, 100), LV_PART_MAIN);
        }

        // Matrix Delete
        if (matrix_delete.panel) {
            lv_obj_set_style_outline_color(matrix_delete.panel, lv_color_hsv_to_rgb((current_hue + 250) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(matrix_delete.label, lv_color_hsv_to_rgb((current_hue + 150) % 360, 100, 100), LV_PART_MAIN);
        }

        // Computer Assist
        if (matrix_switch_computer_assist.panel) {
            if (matrixData.computer_assist[0][current_matrix_i]==true) {
                lv_obj_set_style_outline_color(matrix_switch_computer_assist.panel, lv_color_make(0, 255, 0), LV_PART_MAIN);
                lv_obj_set_style_text_color(matrix_switch_computer_assist.label, lv_color_make(255, 255, 0), LV_PART_MAIN);
            }
            else {
                lv_obj_set_style_outline_color(matrix_switch_computer_assist.panel, lv_color_make(58, 58, 58), LV_PART_MAIN);
                lv_obj_set_style_text_color(matrix_switch_computer_assist.label, lv_color_make(58, 58, 58), LV_PART_MAIN);
            }
        }

        // Override
        if (matrix_switch_override.panel) {
            lv_obj_set_style_outline_color(matrix_switch_override.panel, lv_color_make(255, 0, 0), LV_PART_MAIN);
            lv_obj_set_style_text_color(matrix_switch_override.label, lv_color_make(255, 0, 0), LV_PART_MAIN);
        }

        // Output Value
        if (matrix_switch_output_value) {
            lv_obj_set_style_outline_color(matrix_switch_output_value, lv_color_hsv_to_rgb((current_hue + 250) % 360, 100, 100), LV_PART_MAIN);
            lv_obj_set_style_text_color(matrix_switch_output_value, lv_color_make(0, 255, 0), LV_PART_MAIN);
            lv_label_set_text(matrix_switch_output_value, String(String("") + String(matrixData.output_value[0][current_matrix_i])).c_str());
        }

        // Matrix Configuration Panel
        if (mfc.panel) {
            lv_dropdown_set_selected(mfc.dd_function_index_select, current_matrix_function_i);

            lv_dropdown_set_selected(mfc.dd_function_name, matrixData.matrix_function[0][current_matrix_i][current_matrix_function_i]);

            lv_textarea_set_text(mfc.ta_x, String(matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_X]).c_str());

            lv_textarea_set_text(mfc.ta_y, String(matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_Y]).c_str());

            lv_textarea_set_text(mfc.ta_z, String(matrixData.matrix_function_xyz[0][current_matrix_i][current_matrix_function_i][INDEX_MATRIX_FUNTION_Z]).c_str());

            lv_dropdown_set_selected(mfc.dd_operator, matrixData.matrix_switch_operator_index[0][current_matrix_i][current_matrix_function_i]);

            lv_textarea_set_text(mfc.ta_flux, String(matrixData.flux_value[0][current_matrix_i]).c_str());

            lv_dropdown_set_selected(mfc.dd_output_mode, matrixData.output_mode[0][current_matrix_i]);

            lv_textarea_set_text(mfc.ta_pwm_0, String(matrixData.output_pwm[0][current_matrix_i][0]).c_str());

            lv_textarea_set_text(mfc.ta_pwm_1, String(matrixData.output_pwm[0][current_matrix_i][1]).c_str());

            lv_dropdown_set_selected(mfc.dd_inverted_logic, matrixData.matrix_switch_inverted_logic[0][current_matrix_i][current_matrix_function_i]);

            lv_dropdown_set_selected(mfc.dd_map_slot, matrixData.index_mapped_value[0][current_matrix_i]);

            lv_textarea_set_text(mfc.ta_port_map, String(matrixData.matrix_port_map[0][current_matrix_i]).c_str());
        }

        // Mapping Configuration Panel
        if (mcc.panel) {
            // determine which map slot is used for current matrix
            int current_map_slot = matrixData.index_mapped_value[0][current_matrix_i];

            lv_label_set_text(mcc.label_slot, String(current_map_slot).c_str());

            lv_dropdown_set_selected(mcc.dd_c0, (int)mappingData.mapping_config[0][current_map_slot][INDEX_MAP_C0]);

            lv_textarea_set_text(mcc.ta_c1, String(mappingData.mapping_config[0][current_map_slot][INDEX_MAP_C1]).c_str());

            lv_textarea_set_text(mcc.ta_c2, String(mappingData.mapping_config[0][current_map_slot][INDEX_MAP_C2]).c_str());

            lv_textarea_set_text(mcc.ta_c3, String(mappingData.mapping_config[0][current_map_slot][INDEX_MAP_C3]).c_str());

            lv_textarea_set_text(mcc.ta_c4, String(mappingData.mapping_config[0][current_map_slot][INDEX_MAP_C4]).c_str());

            lv_textarea_set_text(mcc.ta_c5, String(mappingData.mapping_config[0][current_map_slot][INDEX_MAP_C5]).c_str());

            lv_dropdown_set_selected(mcc.dd_mode, mappingData.map_mode[0][current_map_slot]);

            lv_label_set_text(mcc.value_input, String(get_mapping_input_value(current_map_slot)).c_str());
            // lv_label_set_text(mcc.value_input, "foo");

            lv_label_set_text(mcc.value_map_result, String(mappingData.mapped_value[0][matrixData.index_mapped_value[0][current_matrix_i]]).c_str());
        }
    }

    // Increment
    current_hue = (current_hue + 1) % 360;

    lv_timer_resume(display_timer);
}

void initSatIOUI() {
    // --------------------------------------------------------------
    // LVGL Initialization
    // --------------------------------------------------------------
    ESP_LOGI("LVGL", "Version: %d.%d.%d", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    
    // Initialize LVGL display object via BSP
    lv_display_t *disp = bsp_display_start();
    if (!disp) {ESP_LOGE("APP", "Failed to initialize display");}
    ESP_LOGI("APP", "Display initialized successfully");
    
    // Give BSP task time to fully initialize LVGL
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Set LVGL tick period
    lv_timer_set_period(lv_timer_get_next(NULL), 50);  // ms

    // Set screens
    loading_screen = lv_obj_create(NULL);
    home_screen = lv_obj_create(NULL);
    matrix_screen = lv_obj_create(NULL);
    gps_screen = lv_obj_create(NULL);
    gyro_screen = lv_obj_create(NULL);
    disp_screen = lv_obj_create(NULL);
    system_screen = lv_obj_create(NULL);
    _screen = lv_obj_create(NULL);
    
    // Initialize display brightness and backlight
    bsp_display_brightness_init();
    bsp_display_backlight_on();
    slider_brightness_value = 80;
    bsp_display_brightness_set(slider_brightness_value);

    // Menu styling (menus are bootstrapped for consistant reproducibility so minimal styling is needed here)
    font_menu_title = &cobalt_alien_25;
    font_menu_item = &cobalt_alien_17;
    system_tray_bg_color = lv_color_make(10,10,10);
    menu_bg_color        = lv_color_make(0,0,0);
    menu_border_color    = lv_color_make(0,0,0); // hidden by making same as bg
    menu_outline_color   = lv_color_make(0,0,0);    // used instead of border
    menu_shadow_color    = lv_color_make(0,0,0);
    menu_text_color      = lv_color_make(0,255,0);

    // --------------------------------------------------------------
    // SD Card Initialization
    // --------------------------------------------------------------
    mount_sd();
    
    is_fs_ready("/sdcard"); // check mount
    
    list_directory("/sdcard"); // check read sdcard
    
    delay(1000);

    sd_lvgl_register(); // register file related callback functions with LVGL

    // --------------------------------------------------------------
    // Display Loading Screen
    // --------------------------------------------------------------
    display_loading_screen();
}

void satio_ui_begin() {
    display_timer = lv_timer_create(update_display_on_timer, 50, NULL);
}

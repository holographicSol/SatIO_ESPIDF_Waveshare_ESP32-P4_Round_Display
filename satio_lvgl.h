/** -------------------------------------------------------------------------------------
 * 
 */

#ifndef SATIO_LVGL_H
#define SATIO_LVGL_H

#include "lvgl.h"

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

/* Enable complex draw engine (required for anti-aliasing) */
#define LV_USE_DRAW_SW              1

#if LV_USE_DRAW_SW
    /* Enable complex shapes (arcs, lines, polygons, etc.) */
    #define LV_DRAW_SW_COMPLEX      1
    
    /* Set to 1 to enable anti-aliasing */
    #define LV_DRAW_SW_ASM          LV_DRAW_SW_ASM_NONE
#endif

// LV_FONT_DECLARE(active_space_10);
// LV_FONT_DECLARE(space_ranger_15);
LV_FONT_DECLARE(cobalt_alien_17);
// LV_FONT_DECLARE(space_ranger_19);
// LV_FONT_DECLARE(space_ranger_21);
// LV_FONT_DECLARE(space_ranger_23);
LV_FONT_DECLARE(cobalt_alien_25);

#define LV_TXT_ALPHA_LC "abcdefghijklmnopqrstuvwxyz"
#define LV_TXT_ALPHA_UC "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
#define LV_TXT_NUM      "0123456789"
#define LV_TXT_NUMDEC  "0123456789.-"
#define LV_TXT_ALNUMDEC "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789.-"

/** ---------------------------------------------------------------------------------------
 * @brief System Tray Struct
 * 
 * @param tray Pointer to the clickable tray bar object (top-right, no icons).
 * @param panel Pointer to the dropdown panel object.
 * @param slider Pointer to the brightness slider object.
 * @param is_open Boolean flag indicating whether the system tray is currently open or closed.
 */
typedef struct {
    lv_obj_t * tray;     // Clickable tray bar (top-right, no icons)
    lv_obj_t * panel;    // Dropdown panel  
    bool is_open;
    lv_obj_t * slider_brightness;
    lv_obj_t * local_time;
    lv_obj_t * local_date;
    lv_obj_t * human_date;

    // Indicators
    lv_obj_t * datetime_sync; // SYN
    lv_obj_t * serial_cmommand_enabled; // CMD
    lv_obj_t * gps_signal_strength; // SIG
    lv_obj_t * sdcard_mounted; // SD

    // System tray menu
    lv_obj_t * grid_menu_1;

} system_tray_t;

/** ---------------------------------------------------------------------------------------
 * Title Bar Struct
 */
typedef struct {
    lv_obj_t * panel; // The title bar object

    // datetime
    lv_obj_t * time_label; // Label to display local time
    lv_obj_t * date_label; // Label to display local date

    // Indicators
    lv_obj_t * datetime_sync; // SYN
    lv_obj_t * serial_cmommand_enabled; // CMD
    lv_obj_t * gps_signal_strength; // SIG
    lv_obj_t * sdcard_mounted; // SD
} title_bar_t;

/** ----------------------------------------------------------------------------------------
 * @brief Structure to hold image data from SD card and associated LVGL objects for display.
 * 
 * This structure allows us to keep track of the PSRAM pointer, file size, LVGL image object, 
 * and descriptor together.
 */
typedef struct {
    uint32_t f_size;
    uint8_t * bytes_in_psram;
    lv_obj_t * lv_image_obj;
    lv_image_dsc_t dsc;
} sdcard_image_t;

/** ---------------------------------------------------------------------------------------
 * @brief Keyboard Struct
 * 
 * @param kb Pointer to the LVGL keyboard object.
 * @param ta Pointer to the LVGL textarea object associated with the keyboard.
 */

typedef struct {
    lv_obj_t * kb;
    lv_obj_t * ta;
} keyboard_t;

/** ---------------------------------------------------------------------------------------
 * @brief Menu Struct
 * 
 * @param main_header Pointer to the main header object of the menu.
 * @param back_button Pointer to the back button object of the menu.
 * @param back_label Pointer to the label object of the back button.
 * @param menu Pointer to the menu object.
 * @param pages Pointer to an array of page objects in the menu.
 * @param max_pages The maximum number of pages allocated for the menu.
 * @param page_count The current number of pages in the menu.
 * @param page_items Pointer to a 2D array of item objects for each page in the menu.
 * @param page_item_counts Pointer to an array that holds the count of items for each page in the menu.
 * @param max_items_per_page The maximum number of items allocated for each page in the menu.
 */
typedef struct {
    lv_obj_t *   main_header;
    lv_obj_t *   back_button;
    lv_obj_t *   back_label;
    lv_obj_t *   menu;
    lv_obj_t **  pages;
    int          max_pages;
    int          page_count;
    lv_obj_t *** page_items;
    uint32_t *   page_item_counts;
    uint32_t     max_items_per_page;
} menu_struct;

/** ---------------------------------------------------------------------------------------
 * @brief Matrix Function Container Struct
 * 
 * Container for displaying and editing matrix function parameters.
 */
typedef struct {
    lv_obj_t * panel;                    // Main container panel

    // Function selection
    lv_obj_t * label_function_index_select; // Function index select label
    lv_obj_t * dd_function_index_select;     // Matrix function index select
    
    // Function info
    lv_obj_t * label_function_name;      // Function name label
    lv_obj_t * dd_function_name;         // Function name dropdown
    
    // XYZ values (textareas for input)
    lv_obj_t * label_x;                  // X label
    lv_obj_t * ta_x;                     // X textarea
    lv_obj_t * label_y;                  // Y label
    lv_obj_t * ta_y;                     // Y textarea
    lv_obj_t * label_z;                  // Z label
    lv_obj_t * ta_z;                     // Z textarea
    
    // Operator
    lv_obj_t * label_operator;           // Operator label
    lv_obj_t * dd_operator;              // Operator dropdown
    
    // Flux (textarea for input)
    lv_obj_t * label_flux;               // Flux label
    lv_obj_t * ta_flux;                  // Flux textarea
    
    // Output mode (dropdown)
    lv_obj_t * label_output_mode;        // Output mode label
    lv_obj_t * dd_output_mode;           // Output mode dropdown
    
    // PWM values (textareas for input)
    lv_obj_t * label_output_pwm_0;       // Output PWM label 0
    lv_obj_t * label_output_pwm_1;       // Output PWM label 1
    lv_obj_t * ta_pwm_0;                 // PWM 0 textarea
    lv_obj_t * ta_pwm_1;                 // PWM 1 textarea
    
    // Invert Function Logic
    lv_obj_t * label_inverted_logic;     // Inverted logic label
    lv_obj_t * dd_inverted_logic;    // Inverted logic switch
    
    // Map Slot
    lv_obj_t * label_map_slot;           // Map slot label
    lv_obj_t * dd_map_slot;              // Map slot dropdown

    // Port
    lv_obj_t * label_port_map;           // Port map label
    lv_obj_t * ta_port_map;              // Port map textarea
    
} matrix_function_container_t;

/** ---------------------------------------------------------------------------------------
 * @brief Matrix Mapping Configuration Container Struct
 */
typedef struct {
    lv_obj_t * panel; // Main container panel
    lv_obj_t * slot;
    lv_obj_t * label_slot;
    lv_obj_t * c0;
    lv_obj_t * dd_c0;
    lv_obj_t * c1;
    lv_obj_t * ta_c1;
    lv_obj_t * c2;
    lv_obj_t * ta_c2;
    lv_obj_t * c3;
    lv_obj_t * ta_c3;
    lv_obj_t * c4;
    lv_obj_t * ta_c4;
    lv_obj_t * c5;
    lv_obj_t * ta_c5;
    lv_obj_t * mode;
    lv_obj_t * dd_mode;
    lv_obj_t * input_value;
    lv_obj_t * value_input;
    lv_obj_t * map_result;
    lv_obj_t * value_map_result;
} mapping_config_container_t;

typedef struct {
    lv_obj_t * panel; // Main container panel
    lv_obj_t * label;
    lv_obj_t * button;
} button_t;

/** --------------------------------------------------------------------------------------- 
 * Function initializations.
*/
static void set_keyboard_context_cb(lv_event_t * e);

static void keyboard_event_cb(lv_event_t * e);

static void menu_x_event_cb(lv_event_t * e);

static void system_tray_grid_menu_1_event_cb(lv_event_t * e);

static void dropdown_menu_x_event_cb(lv_event_t * e);

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
    );

void slider_brightness_event_cb(lv_event_t * e);

static void tray_close_ready_cb(lv_anim_t * a);

static void screen_swipe_cb(lv_event_t * e);

static void screen_tap_cb(lv_event_t * e);

system_tray_t create_system_tray(lv_obj_t * scr);

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
    int32_t expected_number_of_lines=1
    );

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
    );

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
    );

void create_menu_item(menu_struct * menu, int page_index, const char * title);

lv_obj_t * create_menu_page(lv_obj_t * menu_x, const char * title);

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
    );

lv_obj_t * create_menu_grid(
    lv_obj_t *scr,
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
    ) ;

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
    );

sdcard_image_t * create_image_from_sdcard(
    lv_obj_t * scr,
    const char * filename,
    uint32_t width_px,
    uint32_t height_px,
    uint32_t color_depth_bits,
    uint32_t pos_x,
    uint32_t pos_y,
    int32_t alignment,
    bool discard_after_display);

static void uart0_event_task(void *pvParameters);

void intervalBreach1Second(void);

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
);
    
void cleanup_loading_image();
void delete_screen_and_nullify(lv_obj_t * screen);
void display_loading_screen();
void display_home_screen();
void display_matrix_screen();
void display_gps_screen();
void display_gyro_screen();
void display_disp_screen();
void display_system_screen();
void display__screen();

// update display
void update_display();
// timer update display
void update_display_on_timer(lv_timer_t * timer);

void initSatIOUI();
void satio_ui_begin();

#endif // SATIO_LVGL_H
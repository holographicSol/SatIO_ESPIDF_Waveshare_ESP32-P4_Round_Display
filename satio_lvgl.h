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

typedef struct {
    lv_obj_t * panel; // Main container panel
    lv_obj_t * label;
    lv_obj_t * button;
} button_t;

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

    // Switch selection
    lv_obj_t * label_switch_index_select;   // Function index select label
    lv_obj_t * dd_switch_index_select;      // Matrix function index select

    // Function selection
    lv_obj_t * label_function_index_select; // Function index select label
    lv_obj_t * dd_function_index_select;    // Matrix function index select
    
    // Function info
    lv_obj_t * label_function_name;      // Function name label
    lv_obj_t * dd_function_name;         // Function name dropdown
    
    // XYZ values (textareas for input)
    lv_obj_t * label_x;                  // X label
    lv_obj_t * ta_x;                     // X textarea
    lv_obj_t * dd_x;
    lv_obj_t * label_mode_x;
    lv_obj_t * dd_mode_x;
    
    lv_obj_t * label_y;                  // Y label
    lv_obj_t * ta_y;                     // Y textarea
    lv_obj_t * dd_y;
    lv_obj_t * label_mode_y;
    lv_obj_t * dd_mode_y;
    
    lv_obj_t * label_z;                  // Z label
    lv_obj_t * ta_z;                     // Z textarea
    lv_obj_t * dd_z;
    lv_obj_t * label_mode_z;
    lv_obj_t * dd_mode_z;
    
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
    lv_obj_t * dd_inverted_logic;        // Inverted logic switch
    
    // Map Slot
    lv_obj_t * label_map_slot;           // Map slot label
    lv_obj_t * dd_map_slot;              // Map slot dropdown

    // Port
    lv_obj_t * label_port_map;           // Port map label
    lv_obj_t * ta_port_map;              // Port map textarea

    button_t matrix_switch_computer_assist;
    button_t matrix_switch_override;
    lv_obj_t * matrix_switch_output_value;
    
} matrix_function_container_t;

/** ---------------------------------------------------------------------------------------
 * @brief Matrix Mapping Configuration Container Struct
 */
typedef struct {
    lv_obj_t * panel; // Main container panel
    lv_obj_t * slot;
    lv_obj_t * dd_slot;
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

/** ---------------------------------------------------------------------------------------
 * @brief GPS Switch Panel Container Struct
 */
typedef struct {
    lv_obj_t * panel; // Main container panel
    button_t switch_satio_panel;
    button_t switch_gngga_panel;
    button_t switch_gnrmc_panel;
    button_t switch_gpatt_panel;
} gps_switch_container_t;

/** ---------------------------------------------------------------------------------------
 * @brief GNGGA Container Struct
 */
typedef struct {
    lv_obj_t * panel; // Main container panel
    lv_obj_t * lbl_utc_time;
    lv_obj_t * val_utc_time;
    lv_obj_t * lbl_latitude;
    lv_obj_t * val_latitude;
    lv_obj_t * lbl_longitude;
    lv_obj_t * val_longitude;
    lv_obj_t * lbl_solution_status;
    lv_obj_t * val_solution_status;
    lv_obj_t * lbl_sat_count;
    lv_obj_t * val_sat_count;
    lv_obj_t * lbl_gps_precision_factor;
    lv_obj_t * val_gps_precision_factor;
    lv_obj_t * lbl_altitude;
    lv_obj_t * val_altitude;
    lv_obj_t * lbl_geoidal;
    lv_obj_t * val_geoidal;
    lv_obj_t * lbl_differential_delay;
    lv_obj_t * val_differential_delay;
    lv_obj_t * lbl_bad_element_count;
    lv_obj_t * val_bad_element_count;
} gngga_container_t;

/** ---------------------------------------------------------------------------------------
 * @brief GNRMC Container Struct
 */
typedef struct {
    lv_obj_t * panel; // Main container panel
    lv_obj_t * lbl_utc_time;
    lv_obj_t * val_utc_time;
    lv_obj_t * lbl_positioning_status;
    lv_obj_t * val_positioning_status;
    lv_obj_t * lbl_latitude;
    lv_obj_t * val_latitude;
    lv_obj_t * lbl_longitude;
    lv_obj_t * val_longitude;
    lv_obj_t * lbl_ground_speed;
    lv_obj_t * val_ground_speed;
    lv_obj_t * lbl_ground_heading;
    lv_obj_t * val_ground_heading;
    lv_obj_t * lbl_utc_date;
    lv_obj_t * val_utc_date;
    lv_obj_t * lbl_installation_angle;
    lv_obj_t * val_installation_angle;
    lv_obj_t * lbl_installation_angle_direction;
    lv_obj_t * val_installation_angle_direction;
    lv_obj_t * lbl_mode_indication;
    lv_obj_t * val_mode_indication;
    lv_obj_t * lbl_bad_element_count;
    lv_obj_t * val_bad_element_count;
} gnrmc_container_t;

/** ---------------------------------------------------------------------------------------
 * @brief GPATT Container Struct
 */
typedef struct {
    lv_obj_t * panel; // Main container panel
    lv_obj_t * lbl_pitch;
    lv_obj_t * val_pitch;
    lv_obj_t * lbl_roll;
    lv_obj_t * val_roll;
    lv_obj_t * lbl_yaw;
    lv_obj_t * val_yaw;
    lv_obj_t * lbl_software_version;
    lv_obj_t * val_software_version;
    lv_obj_t * lbl_product_id;
    lv_obj_t * val_product_id;
    lv_obj_t * lbl_ins;
    lv_obj_t * val_ins;
    lv_obj_t * lbl_hardware_version;
    lv_obj_t * val_hardware_version;
    lv_obj_t * lbl_run_state_flag;
    lv_obj_t * val_run_state_flag;
    lv_obj_t * lbl_mis_angle_num;
    lv_obj_t * val_mis_angle_num;
    lv_obj_t * lbl_static_flag;
    lv_obj_t * val_static_flag;
    lv_obj_t * lbl_user_code;
    lv_obj_t * val_user_code;
    lv_obj_t * lbl_gst_data;
    lv_obj_t * val_gst_data;
    lv_obj_t * lbl_line_flag;
    lv_obj_t * val_line_flag;
    lv_obj_t * lbl_mis_att_flag;
    lv_obj_t * val_mis_att_flag;
    lv_obj_t * lbl_imu_kind;
    lv_obj_t * val_imu_kind;
    lv_obj_t * lbl_ubi_car_kind;
    lv_obj_t * val_ubi_car_kind;
    lv_obj_t * lbl_mileage;
    lv_obj_t * val_mileage;
    lv_obj_t * lbl_run_inetial_flag;
    lv_obj_t * val_run_inetial_flag;
    lv_obj_t * lbl_speed_num;
    lv_obj_t * val_speed_num;
    lv_obj_t * lbl_scalable;
    lv_obj_t * val_scalable;
    lv_obj_t * lbl_bad_element_count;
    lv_obj_t * val_bad_element_count;
} gpatt_container_t;

/** ---------------------------------------------------------------------------------------
 * @brief SATIO Container Struct
 */
typedef struct {
    lv_obj_t * panel; // Main container panel

    /* ---------------------------------------------------------- */
    /* Title Positioning                                          */
    /* ---------------------------------------------------------- */
    lv_obj_t * lbl_title_location;

    lv_obj_t * lbl_deg_lat;      // GPS Latitude
    lv_obj_t * val_deg_lat;      // GPS Latitude
    lv_obj_t * lbl_user_deg_lat; // User Latitude
    lv_obj_t * ta_user_deg_lat;  // User Latitude
    lv_obj_t * lbl_sys_deg_lat;  // System Latitude
    lv_obj_t * val_sys_deg_lat;  // System Latitude

    lv_obj_t * lbl_deg_lon;      // GPS Longitude
    lv_obj_t * val_deg_lon;      // GPS Longitude
    button_t btn_auto_set_user_lat;
    lv_obj_t * lbl_user_deg_lon; // User Longitude
    lv_obj_t * ta_user_deg_lon;  // User Longitude
    button_t btn_auto_set_user_lon;
    lv_obj_t * lbl_sys_deg_lon;  // System Longitude
    lv_obj_t * val_sys_deg_lon;  // System Longitude

    lv_obj_t * lbl_location_mode; 
    button_t btn_location_mode_gps;
    button_t btn_location_mode_user;

    /* ---------------------------------------------------------- */
    /* Title Altitude                                             */
    /* ---------------------------------------------------------- */
    lv_obj_t * lbl_title_altitude;

    lv_obj_t * lbl_altitude;      // GPS Altitude
    lv_obj_t * val_altitude;      // GPS Altitude    
    lv_obj_t * lbl_user_altitude; // User Altitude
    button_t btn_auto_set_user_altitude;
    lv_obj_t * ta_user_altitude;  // User Altitude
    lv_obj_t * lbl_sys_altitude;  // System Altitude
    lv_obj_t * val_sys_altitude;  // System Altitude

    lv_obj_t * lbl_altitude_mode; 
    button_t btn_altitude_mode_gps;
    button_t btn_altitude_mode_user;

    /* ---------------------------------------------------------- */
    /* Title Speed                                                */
    /* ---------------------------------------------------------- */
    lv_obj_t * lbl_title_speed;

    lv_obj_t * lbl_speed;      // GPS Speed
    lv_obj_t * val_speed;      // GPS Speed
    lv_obj_t * lbl_user_speed; // User Speed
    button_t btn_auto_set_user_speed;
    lv_obj_t * ta_user_speed;  // User Speed
    lv_obj_t * lbl_sys_speed;  // System Speed
    lv_obj_t * val_sys_speed;  // System Speed

    lv_obj_t * lbl_speed_mode; 
    button_t btn_speed_mode_gps;
    button_t btn_speed_mode_user;

    /* ---------------------------------------------------------- */
    /* Title Heading                                              */
    /* ---------------------------------------------------------- */
    lv_obj_t * lbl_title_heading;

    lv_obj_t * lbl_ground_heading_name; // NNW, etc.
    lv_obj_t * val_ground_heading_name; // NNW, etc
    lv_obj_t * lbl_ground_heading;      // GPS Ground Heading
    lv_obj_t * val_ground_heading;      // GPS Ground Heading
    lv_obj_t * lbl_user_ground_heading; // User Ground Heading
    lv_obj_t * ta_user_ground_heading;  // User Ground Heading
    button_t btn_auto_set_user_ground_heading;
    lv_obj_t * lbl_sys_ground_heading;  // System Ground Heading
    lv_obj_t * val_sys_ground_heading;  // System Ground Heading

    lv_obj_t * lbl_ground_heading_mode; 
    button_t btn_ground_heading_mode_gps;
    button_t btn_ground_heading_mode_user;

    /* ---------------------------------------------------------- */
    /* Title Mileage                                              */
    /* ---------------------------------------------------------- */
    lv_obj_t * lbl_title_mileage;
    lv_obj_t * lbl_mileage;
    lv_obj_t * val_mileage;

    /* ---------------------------------------------------------- */
    /* Title Local Time                                           */
    /* ---------------------------------------------------------- */
    lv_obj_t * lbl_title_local_time;

    lv_obj_t * lbl_local_unixtime_us;
    lv_obj_t * val_local_unixtime_us;
    lv_obj_t * lbl_local_yday;
    lv_obj_t * val_local_yday;
    lv_obj_t * lbl_local_wday_name;
    lv_obj_t * val_local_wday_name;
    lv_obj_t * lbl_local_month_name;
    lv_obj_t * val_local_month_name;
    lv_obj_t * lbl_formatted_local_time;
    lv_obj_t * val_formatted_local_time;
    lv_obj_t * lbl_formatted_local_date;
    lv_obj_t * val_formatted_local_date;

    lv_obj_t * lbl_utc_second_offset;
    lv_obj_t * ta_utc_second_offset;
    
    lv_obj_t * lbl_utc_auto_offset_flag;
    lv_obj_t * val_utc_auto_offset_flag;
    lv_obj_t * lbl_set_time_automatically;
    lv_obj_t * val_set_time_automatically;

    /* ---------------------------------------------------------- */
    /* Title RTC Time                                             */
    /* ---------------------------------------------------------- */
    lv_obj_t * lbl_title_rtc_time;

    lv_obj_t * lbl_rtc_unixtime;
    lv_obj_t * val_rtc_unixtime;
    lv_obj_t * lbl_formatted_rtc_time;
    lv_obj_t * val_formatted_rtc_time;
    lv_obj_t * lbl_formatted_rtc_date;
    lv_obj_t * val_formatted_rtc_date;

    /* ---------------------------------------------------------- */
    /* Title RTC Sync                                             */
    /* ---------------------------------------------------------- */
    lv_obj_t * lbl_title_rtc_sync;

    lv_obj_t * lbl_formatted_rtc_sync_time;
    lv_obj_t * val_formatted_rtc_sync_time;
    lv_obj_t * lbl_formatted_rtc_sync_date;
    lv_obj_t * val_formatted_rtc_sync_date;
    lv_obj_t * lbl_rtcsync_latitude;
    lv_obj_t * val_rtcsync_latitude;
    lv_obj_t * lbl_rtcsync_longitude;
    lv_obj_t * val_rtcsync_longitude;
    lv_obj_t * lbl_rtcsync_altitude;
    lv_obj_t * val_rtcsync_altitude;

} satio_container_t;

/** ---------------------------------------------------------------------------------------
 * @brief GYRO Container Struct
 */
typedef struct {
    lv_obj_t * panel; // Main container panel
    
    lv_obj_t * lbl_gyro_0_ang_x;
    lv_obj_t * val_gyro_0_ang_x;
    // lv_obj_t * lbl_gyro_0_ang_y;
    lv_obj_t * val_gyro_0_ang_y;
    // lv_obj_t * lbl_gyro_0_ang_z;
    lv_obj_t * val_gyro_0_ang_z;
    
    lv_obj_t * lbl_gyro_0_acc_x;
    lv_obj_t * val_gyro_0_acc_x;
    // lv_obj_t * lbl_gyro_0_acc_y;
    lv_obj_t * val_gyro_0_acc_y;
    // lv_obj_t * lbl_gyro_0_acc_z;
    lv_obj_t * val_gyro_0_acc_z;
    
    lv_obj_t * lbl_gyro_0_gyr_x;
    lv_obj_t * val_gyro_0_gyr_x;
    // lv_obj_t * lbl_gyro_0_gyr_y;
    lv_obj_t * val_gyro_0_gyr_y;
    // lv_obj_t * lbl_gyro_0_gyr_z;
    lv_obj_t * val_gyro_0_gyr_z;
    
    lv_obj_t * lbl_gyro_0_mag_x;
    lv_obj_t * val_gyro_0_mag_x;
    // lv_obj_t * lbl_gyro_0_mag_y;
    lv_obj_t * val_gyro_0_mag_y;
    // lv_obj_t * lbl_gyro_0_mag_z;
    lv_obj_t * val_gyro_0_mag_z;
    
    lv_obj_t * lbl_gyro_0_current_uiBaud;
    lv_obj_t * val_gyro_0_current_uiBaud;
} gyro_0_container_t;

/** --------------------------------------------------------------------------------------- 
 * Function initializations.
*/

static void tray_close_ready_cb(lv_anim_t * a);
static void set_keyboard_context_cb(lv_event_t * e);
static void keyboard_event_cb(lv_event_t * e);
static void screen_swipe_cb(lv_event_t * e);
static void screen_tap_cb(lv_event_t * e);
void slider_brightness_event_cb(lv_event_t * e);
static void tray_close_ready_cb(lv_anim_t * a);
static void system_tray_grid_menu_1_event_cb(lv_event_t * e);
static void matrix_overview_grid_1_event_cb(lv_event_t * e);
static void dd_function_index_select_event_cb(lv_event_t * e);
static void dd_switch_index_select_event_cb(lv_event_t * e);
static void dd_current_map_slot_event_cb(lv_event_t * e);
static void dd_function_name_event_cb(lv_event_t * e);
static void dd_c0_event_cb(lv_event_t * e);
static void dd_mode_event_cb(lv_event_t * e);
static void dd_mode_x_event_cb(lv_event_t * e);
static void dd_mode_y_event_cb(lv_event_t * e);
static void dd_mode_z_event_cb(lv_event_t * e);
static void dd_inverted_logic_event_cb(lv_event_t * e);
static void dd_x_event_cb(lv_event_t * e);
static void dd_y_event_cb(lv_event_t * e);
static void dd_z_event_cb(lv_event_t * e);
static void dd_operator_event_cb(lv_event_t * e);
static void dd_output_mode_event_cb(lv_event_t * e);
static void dd_matrix_file_slot_select_event_cb(lv_event_t * e);
static void dd_link_map_slot_event_cb(lv_event_t * e);
static void matrix_new_event_cb(lv_event_t * e);
static void matrix_save_event_cb(lv_event_t * e);
static void matrix_load_event_cb(lv_event_t * e);
static void matrix_delete_event_cb(lv_event_t * e);
static void current_matrix_computer_assist_event_cb(lv_event_t * e);
static void switch_matrix_mapping_panel_event_cb(lv_event_t * e);
static void current_matrix_override_off_event_cb(lv_event_t * e);
static void switch_gps_panel_event_cb(lv_event_t * e);

/** -------------------------------------------------------------------------------------
 * @brief Create System Tray.
 * 
 * @param parent Specify parent object.
 */
system_tray_t create_system_tray(
    lv_obj_t * parent
);

/** -------------------------------------------------------------------------------------
 * @brief Create Title Bar.
 * 
 * @param parent Specify parent object.
 * @param size_w_px Panel width.
 * @param size_h_px Panel height
 * @param alignment Panel alignment on parent object.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @return title_bar_t.
 */
title_bar_t create_title_bar (
    lv_obj_t * parent,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    bool show_scrollbar,
    bool enable_scrolling
    );

/** -------------------------------------------------------------------------------------
 * @brief Create Slider.
 * 
 * @param parent Specify parent object.
 * @param size_w_px Panel width.
 * @param size_h_px Panel height
 * @param alignment Panel alignment on parent object.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param range_min Specify minimum value.
 * @param range_max Specify maximum value.
 * @param value Specify initial value.
 * @return lv_obj_t.
 */
lv_obj_t * create_slider(
    lv_obj_t * parent,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t range_min,
    int32_t range_max,
    int32_t range_value
    );

/** -------------------------------------------------------------------------------------
 * @brief Create Label.
 * 
 * @param parent Specify parent object.
 * @param size_w_px Panel width.
 * @param size_h_px Panel height
 * @param alignment Panel alignment on parent object.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param text Specify initial text.
 * @param font Specify text font.
 * @param text_align Text alignment on label.
 * @param transparent_bg Tranparent background.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param outline_width Specify panel outline width.
 * @param radius Specify panel outline radius.
 * @param expected_number_of_lines Specify expected number of lines (used for alignment).
 * @param color_bg Background color.
 * @param color_text Text color.
 * @return lv_obj_t.
 */
lv_obj_t * create_label(
    lv_obj_t * parent,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
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
    int32_t expected_number_of_lines,
    lv_color_t color_bg,
    lv_color_t color_text
    );

/** -------------------------------------------------------------------------------------
 * @brief Create Text Area.
 * 
 * @param parent Specify parent object.
 * @param size_w_px Panel width.
 * @param size_h_px Panel height
 * @param alignment Panel alignment on parent object.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param one_line Enable/disables multiline.
 * @param accepted_chars Specify accepted chars.
 * @param placeholder_text Specify placeholder text.
 * @param transparent_bg Tranparent background.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font Specify text font.
 * @param text_align Text alignment on label.
 * @return lv_obj_t.
 */
lv_obj_t * create_textarea(
    lv_obj_t * parent,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    bool one_line,
    char * accepted_chars,
    char * placeholder_text,
    bool transparent_bg,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font,
    lv_text_align_t text_align
    );

/** -------------------------------------------------------------------------------------
 * @brief Create Text Area.
 * 
 * @param parent Specify parent object.
 * @param size_w_px Panel width.
 * @param size_h_px Panel height
 * @param alignment Panel alignment on parent object.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param kb_ta_padding_px Distance between bottom of textarea and top of keyboard.
 * @param ta_height_px Height of text area.
 * @param keyboard_mode Set keyboard mode:
 *                      LV_KEYBOARD_MODE_TEXT_LOWER
 *                      LV_KEYBOARD_MODE_TEXT_UPPER
 *                      LV_KEYBOARD_MODE_SPECIAL
 *                      LV_KEYBOARD_MODE_NUMBER
 *                      LV_KEYBOARD_MODE_USER_1
 *                      LV_KEYBOARD_MODE_USER_2
 *                      LV_KEYBOARD_MODE_USER_3
 *                      LV_KEYBOARD_MODE_USER_4
 * @return keyboard_t.
 */
keyboard_t create_keyboard(
    lv_obj_t * parent,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t kb_ta_padding_px,
    int32_t ta_height_px,
    lv_keyboard_mode_t keyboard_mode
    );

/** -------------------------------------------------------------------------------------
 * @brief Create Menu Item
 * 
 * @param menu Specify menu_struct object.
 * @param num_pages Specify number of pages.
 * @param title Specify menu title.
 * @return Void. A menu must be specified as the first parameter.
 */
void create_menu_item(
    menu_struct * menu,
    int num_pages,
    const char * title
    );

/** -------------------------------------------------------------------------------------
 * @brief Create Menu Page
 * 
 * @param menu Specify menu_struct object.
 * @param title Specify menu title.
 * @return lv_obj_t.
 */
lv_obj_t * create_menu_page(
    lv_obj_t * menu,
    const char * title
    );

/** -------------------------------------------------------------------------------------
 * @brief Create Menu.
 * 
 * @param parent Specify parent object.
 * @param max_pages Specify max pages.
 * @param page_titles Specify page titles array.
 * @param main_menu_items
 * @param size_w_px Panel width.
 * @param size_h_px Panel height
 * @param alignment Panel alignment on parent object.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @return menu_struct.
 */
menu_struct create_menu(
    lv_obj_t *parent,
    int max_pages,
    const char ** page_titles,
    const char ** main_menu_items,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y
    );

/** -------------------------------------------------------------------------------------
 * @brief Create Menu Grid Layout.
 *
 * @param parent Specify parent object.
 * @param cols Number of columns.
 * @param rows Number of rows.
 * @param cell_size_px Size of each cell (square).
 * @param outer_padding Outer padding around the grid.
 * @param inner_padding Inner padding between cells.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param lv_alignment Grid alignment on parent.
 * @param item_radius Radius of individual item panels.
 * @param max_cols_visible Maximum visible columns (for scrolling).
 * @param max_rows_visible Maximum visible rows (for scrolling).
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param text_align Text alignment inside cells.
 * @param font Font used in cells.
 * @param transparent_bg Transparent background for cells.
 * @param transparent_outline Transparent outline for cells.
 * @return lv_obj_t* Pointer to the created grid container.
 */
lv_obj_t * create_menu_grid(
    lv_obj_t *parent,
    const int32_t cols,
    const int32_t rows,
    const int32_t cell_size_px,
    const int32_t outer_padding,
    const int32_t inner_padding,
    const int32_t pos_x,
    const int32_t pos_y,
    lv_align_t lv_alignment,
    int32_t item_radius,
    int32_t max_cols_visible,
    int32_t max_rows_visible,
    bool show_scrollbar,
    bool enable_scrolling,
    lv_text_align_t text_align,
    const lv_font_t * font,
    bool transparent_bg,
    bool transparent_outline
);

/** -------------------------------------------------------------------------------------
 * @brief Create Dropdown Menu.
 *
 * @param parent Specify parent object.
 * @param options Array of option strings.
 * @param option_count Number of options in the array.
 * @param width_px Dropdown width.
 * @param height_px Dropdown height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param font Font used in dropdown.
 * @return lv_obj_t* Pointer to the created dropdown object.
 */
lv_obj_t * create_dropdown_menu(
    lv_obj_t * parent,
    char options[][MAX_GLOBAL_ELEMENT_SIZE],
    int option_count,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    const lv_font_t * font
);

/** -------------------------------------------------------------------------------------
 * @brief Create Switch.
 *
 * @param parent Specify parent object.
 * @param size_w_px Switch width.
 * @param size_h_px Switch height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @return lv_obj_t* Pointer to the created switch object.
 */
lv_obj_t * create_switch(
    lv_obj_t *parent,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y
);

/** -------------------------------------------------------------------------------------
 * @brief Create Custom Button (with panel + transparent button + label).
 *
 * @param parent Specify parent object.
 * @param size_w_px Button width.
 * @param size_h_px Button height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param text Button display text.
 * @param text_align Text alignment.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font Font used for button text.
 * @param radius Corner radius.
 * @param color_bg Background color.
 * @param color_text Text color.
 * @return button_t structure containing panel, button and label objects.
 */
button_t create_button(
    lv_obj_t *parent,
    int32_t size_w_px,
    int32_t size_h_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    char * text,
    lv_text_align_t text_align,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font,
    int32_t radius,
    lv_color_t color_bg,
    lv_color_t color_text
);

/** -------------------------------------------------------------------------------------
 * @brief Create Horizontal Row Container.
 *
 * @param parent Specify parent object.
 * @param sub_row_width Width of each sub-row.
 * @param sub_row_height Height of each sub-row.
 * @param inner_pad_all Uniform inner padding.
 * @param sub_row_padding Padding between sub-rows.
 * @param sub_column_padding Padding between sub-columns.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @return lv_obj_t* Pointer to the created row container.
 */
lv_obj_t * create_row(
    lv_obj_t * parent,
    int32_t sub_row_width,
    int32_t sub_row_height,
    int32_t inner_pad_all,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    bool show_scrollbar,
    bool enable_scrolling
);

/** -------------------------------------------------------------------------------------
 * @brief Create Matrix Function Container.
 *
 * Creates a structured container typically used for matrix-style function key layouts.
 *
 * @param parent Specify parent object.
 * @param width_px Container width.
 * @param height_px Container height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param radius Corner radius.
 * @param outer_pad_all Outer padding.
 * @param inner_pad_all Inner uniform padding.
 * @param outline_padding Padding for outline.
 * @param main_row_padding Main row padding.
 * @param main_column_padding Main column padding.
 * @param sub_row_padding Sub-row padding.
 * @param sub_column_padding Sub-column padding.
 * @param row_height Height of each row.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font_title Title font.
 * @param font_sub Subtitle/font for smaller text.
 * @return matrix_function_container_t structure.
 */
matrix_function_container_t create_matrix_function_container(
    lv_obj_t * parent,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t radius,
    int32_t outer_pad_all,
    int32_t inner_pad_all,
    int32_t outline_padding,
    int32_t main_row_padding,
    int32_t main_column_padding,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    int32_t row_height,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
);

/** -------------------------------------------------------------------------------------
 * @brief Create Mapping Configuration Container.
 *
 * @param parent Specify parent object.
 * @param width_px Container width.
 * @param height_px Container height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param radius Corner radius.
 * @param outer_pad_all Outer padding.
 * @param inner_pad_all Inner uniform padding.
 * @param outline_padding Padding for outline.
 * @param main_row_padding Main row padding.
 * @param main_column_padding Main column padding.
 * @param sub_row_padding Sub-row padding.
 * @param sub_column_padding Sub-column padding.
 * @param row_height Height of each row.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font_title Title font.
 * @param font_sub Subtitle/font for smaller text.
 * @return mapping_config_container_t structure.
 */
mapping_config_container_t create_mapping_config_container(
    lv_obj_t * parent,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t radius,
    int32_t outer_pad_all,
    int32_t inner_pad_all,
    int32_t outline_padding,
    int32_t main_row_padding,
    int32_t main_column_padding,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    int32_t row_height,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
);

/** -------------------------------------------------------------------------------------
 * @brief Create Gyro Panel Container.
 *
 * @param parent Specify parent object.
 * @param width_px Container width.
 * @param height_px Container height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param radius Corner radius.
 * @param outer_pad_all Outer padding.
 * @param inner_pad_all Inner uniform padding.
 * @param outline_padding Padding for outline.
 * @param main_row_padding Main row padding.
 * @param main_column_padding Main column padding.
 * @param sub_row_padding Sub-row padding.
 * @param sub_column_padding Sub-column padding.
 * @param row_height Height of each row.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font_title Title font.
 * @param font_sub Subtitle/font for smaller text.
 * @return gyro_0_container_t structure.
 */
gyro_0_container_t create_gyro_panel(
    lv_obj_t * parent,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t radius,
    int32_t outer_pad_all,
    int32_t inner_pad_all,
    int32_t outline_padding,
    int32_t main_row_padding,
    int32_t main_column_padding,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    int32_t row_height,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
);

/** -------------------------------------------------------------------------------------
 * @brief Create Satio Panel Container.
 *
 * @param parent Specify parent object.
 * @param width_px Container width.
 * @param height_px Container height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param radius Corner radius.
 * @param outer_pad_all Outer padding.
 * @param inner_pad_all Inner uniform padding.
 * @param outline_padding Padding for outline.
 * @param main_row_padding Main row padding.
 * @param main_column_padding Main column padding.
 * @param sub_row_padding Sub-row padding.
 * @param sub_column_padding Sub-column padding.
 * @param row_height Height of each row.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font_title Title font.
 * @param font_sub Subtitle/font for smaller text.
 * @return satio_container_t structure.
 */
satio_container_t create_satio_panel(
    lv_obj_t * parent,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t radius,
    int32_t outer_pad_all,
    int32_t inner_pad_all,
    int32_t outline_padding,
    int32_t main_row_padding,
    int32_t main_column_padding,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    int32_t row_height,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
);

/** -------------------------------------------------------------------------------------
 * @brief Create GNGGA NMEA Panel Container.
 *
 * @param parent Specify parent object.
 * @param width_px Container width.
 * @param height_px Container height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param radius Corner radius.
 * @param outer_pad_all Outer padding.
 * @param inner_pad_all Inner uniform padding.
 * @param outline_padding Padding for outline.
 * @param main_row_padding Main row padding.
 * @param main_column_padding Main column padding.
 * @param sub_row_padding Sub-row padding.
 * @param sub_column_padding Sub-column padding.
 * @param row_height Height of each row.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font_title Title font.
 * @param font_sub Subtitle/font for smaller text.
 * @return gngga_container_t structure.
 */
gngga_container_t create_gngga_panel(
    lv_obj_t * parent,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t radius,
    int32_t outer_pad_all,
    int32_t inner_pad_all,
    int32_t outline_padding,
    int32_t main_row_padding,
    int32_t main_column_padding,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    int32_t row_height,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
);

/** -------------------------------------------------------------------------------------
 * @brief Create GNRMC NMEA Panel Container.
 *
 * @param parent Specify parent object.
 * @param width_px Container width.
 * @param height_px Container height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param radius Corner radius.
 * @param outer_pad_all Outer padding.
 * @param inner_pad_all Inner uniform padding.
 * @param outline_padding Padding for outline.
 * @param main_row_padding Main row padding.
 * @param main_column_padding Main column padding.
 * @param sub_row_padding Sub-row padding.
 * @param sub_column_padding Sub-column padding.
 * @param row_height Height of each row.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font_title Title font.
 * @param font_sub Subtitle/font for smaller text.
 * @return gnrmc_container_t structure.
 */
gnrmc_container_t create_gnrmc_panel(
    lv_obj_t * parent,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t radius,
    int32_t outer_pad_all,
    int32_t inner_pad_all,
    int32_t outline_padding,
    int32_t main_row_padding,
    int32_t main_column_padding,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    int32_t row_height,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
);

/** -------------------------------------------------------------------------------------
 * @brief Create GPATT Panel Container.
 *
 * @param parent Specify parent object.
 * @param width_px Container width.
 * @param height_px Container height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param radius Corner radius.
 * @param outer_pad_all Outer padding.
 * @param inner_pad_all Inner uniform padding.
 * @param outline_padding Padding for outline.
 * @param main_row_padding Main row padding.
 * @param main_column_padding Main column padding.
 * @param sub_row_padding Sub-row padding.
 * @param sub_column_padding Sub-column padding.
 * @param row_height Height of each row.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font_title Title font.
 * @param font_sub Subtitle/font for smaller text.
 * @return gpatt_container_t structure.
 */
gpatt_container_t create_gpatt_panel(
    lv_obj_t * parent,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t radius,
    int32_t outer_pad_all,
    int32_t inner_pad_all,
    int32_t outline_padding,
    int32_t main_row_padding,
    int32_t main_column_padding,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    int32_t row_height,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
);

/** -------------------------------------------------------------------------------------
 * @brief Create GPS Switch Panel Container.
 *
 * @param parent Specify parent object.
 * @param width_px Container width.
 * @param height_px Container height.
 * @param alignment Alignment on parent.
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param radius Corner radius.
 * @param outer_pad_all Outer padding.
 * @param inner_pad_all Inner uniform padding.
 * @param outline_padding Padding for outline.
 * @param main_row_padding Main row padding.
 * @param main_column_padding Main column padding.
 * @param sub_row_padding Sub-row padding.
 * @param sub_column_padding Sub-column padding.
 * @param row_height Height of each row.
 * @param show_scrollbar Show/hide scrollbar.
 * @param enable_scrolling Enable/disable scrolling.
 * @param font_title Title font.
 * @param font_sub Subtitle/font for smaller text.
 * @return gps_switch_container_t structure.
 */
gps_switch_container_t create_gps_switch_panel(
    lv_obj_t * parent,
    int32_t width_px,
    int32_t height_px,
    lv_align_t alignment,
    int32_t pos_x,
    int32_t pos_y,
    int32_t radius,
    int32_t outer_pad_all,
    int32_t inner_pad_all,
    int32_t outline_padding,
    int32_t main_row_padding,
    int32_t main_column_padding,
    int32_t sub_row_padding,
    int32_t sub_column_padding,
    int32_t row_height,
    bool show_scrollbar,
    bool enable_scrolling,
    const lv_font_t * font_title,
    const lv_font_t * font_sub
);

/** -------------------------------------------------------------------------------------
 * @brief Create Image Loaded from SD Card.
 *
 * @param parent Specify parent object.
 * @param filename Path/filename on SD card.
 * @param width_px Display width.
 * @param height_px Display height.
 * @param color_depth_bits Color depth (16, 32, etc.).
 * @param pos_x Offset from alignment.
 * @param pos_y Offset from alignment.
 * @param alignment Image alignment on parent.
 * @param discard_after_display Free image data after first display (memory optimization).
 * @return sdcard_image_t* Pointer to image info structure (or NULL on failure).
 */
sdcard_image_t * create_image_from_sdcard(
    lv_obj_t * parent,
    const char * filename,
    uint32_t width_px,
    uint32_t height_px,
    uint32_t color_depth_bits,
    uint32_t pos_x,
    uint32_t pos_y,
    lv_align_t alignment,
    bool discard_after_display
);

/** -------------------------------------------------------------------------------------
 * @brief Create Default Screen Objects.
 *
 * Initializes and places all default UI elements on the given parent screen.
 *
 * @param parent Specify parent object (usually the active screen).
 * @return Void.
 */
void create_default_screen_objects(
    lv_obj_t * parent
);

/* Free's loading image from memory */
void cleanup_loading_image();

/* Sets global color scheme to default color scheme */
void setColorsDefault();

/* Sets global color scheme to custom color scheme */
void setColorsCustom();

/* Display screen's */
void display_loading_screen();
void display_home_screen();
void display_matrix_screen();
void display_gps_screen();
void display_gyro_screen();
void display_disp_screen();
void display_system_screen();
void display_uap_screen();

/* Flags to trigger scrren loading */
extern bool flag_display_loading_screen;
extern bool flag_display_home_screen;
extern bool flag_display_matrix_screen;
extern bool flag_display_gps_screen;
extern bool flag_display_gyro_screen;
extern bool flag_display_disp_screen;
extern bool flag_display_system_screen;
extern bool flag_display_uap_screen;

/* Main function to update screen objects and load screens */
void update_display();

/* Timer runs update_display function */
void update_display_on_timer(lv_timer_t * timer);

/* Initialize LVGL for this device */
void initSatIOUI();

/* Start's update_display_on_timer */
void satio_ui_begin();

#endif // SATIO_LVGL_H
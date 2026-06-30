/*
    TaskHandler - Written By Benjamin Jack Cullen.

    MISRA conventions used throughout this library:
    (1) Every if/else/while/for body is a braced compound statement (MISRA C 2012 Rule 15.6).
    (2) Boolean objects are tested directly instead of being compared to true/false
        (MISRA C 2012 Rule 14.4 - essentially Boolean controlling expressions).
    (3) Null pointers are written as nullptr, the type-safe C++ null pointer constant.

    Intended to be MISRA Compliant (untested, unverified, in-progress).
*/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <rtc_wdt.h>
#include <esp_task_wdt.h>
#include "./config.h"
#include "./REG.h"
#include "./strval.h"
#include "./meteors.h"
#include "./wtgps300p.h"
#include "./wt901.h"
#include "./multiplexers.h"
#include <SiderealPlanets.h>
#include <SiderealObjects.h>
#include "./sidereal_helper.h"
#include "./hextodig.h"
#include "./ins.h"
#include "./satio.h"
#include "./satio_file.h"
#include "./custommapping.h"
#include "./matrix.h"
#include "./serial_infocmd.h"
#include "./system_data.h"
#include "./sdcard_helper.h"
#include "./task_handler.h"
#include "i2c_helper.h"
#include "./satio_lvgl.h"

TaskHandle_t TaskGPS;
TaskHandle_t TaskGyro;
TaskHandle_t TaskMultiplexers;
TaskHandle_t TaskSerialInfoCMD;
TaskHandle_t TaskSwitches;
TaskHandle_t TaskStorage;
TaskHandle_t TaskUniverse;
TaskHandle_t TaskDisplayUpdate;

// PRIORITY
#define TASK_GPS_PRIORITY                   5    // High: Time/location sync critical
#define TASK_GYRO_PRIORITY                  4    // High: Sensor reading
#define TASK_MULTIPLEXERS_PRIORITY          4    // High: Analog multiplexing
#define TASK_SWITCHES_PRIORITY              4    // High: Logic processing
#define TASK_SERIALINFOCMD_PRIORITY         3    // High: User interaction & debugging
#define TASK_UNIVERSE_PRIORITY              3    // LOW: Computational, non-critical delay
#define TASK_STORAGE_PRIORITY               3    // LOW: I/O operations, can wait

// CORE ASSIGNMENT
#define TASK_SERIALINFOCMD_CORE             0    // Core 0: Keep on main (timing-sensitive)
#define TASK_GYRO_CORE                      0    // Core 0: Sensor reading (less critical)
#define TASK_MULTIPLEXERS_CORE              0    // Core 0: ADC/multiplexing
#define TASK_SWITCHES_CORE                  0    // Core 0: Outputs need responsiveness
#define TASK_UNIVERSE_CORE                  0    // Core 0: Heavy compute, can defer
#define TASK_STORAGE_CORE                   1    // Core 1: Deferred to Core 1 while Core 0 is too busy
#define TASK_GPS_CORE                       0    // Critical for system timing regardless of gps

// STACK SIZES (Adjusted for task complexity)
#define TASK_STORAGE_STACK_SIZE             6144    // +50%: SDMMC operations
#define TASK_SERIALINFOCMD_STACK_SIZE       16384   // Keep: Complex serial processing
#define TASK_GPS_STACK_SIZE                 5120    // +25%: NMEA parsing + INS updates
#define TASK_GYRO_STACK_SIZE                4608    // +12%: Math operations
#define TASK_MULTIPLEXERS_STACK_SIZE        4096    // Keep: Simple ADC reading
#define TASK_SWITCHES_STACK_SIZE            5120    // +25%: Matrix calculations, mappings
#define TASK_UNIVERSE_STACK_SIZE            20480   // +25%: Expensive float math (planets, etc.)

/* Display task configuration — defined here so satio_lvgl.cpp can reference them
   alongside the other task constants in task_handler.cpp. */
#define TASK_DISPLAY_PRIORITY    2       // LOWEST: preempted by every other user task
#define TASK_DISPLAY_CORE        0       // Core 1: isolated from Core 0 time-critical tasks
#define TASK_DISPLAY_STACK_SIZE  32768   // Large: LVGL screen building + snprintf buffers

/** ----------------------------------------------------------------------------
 * 
 * @brief Notify all Tasks.
 * 
 * Macro for notifying all tasks.
 * 
 * Tasks delayed by xTaskNotifyWait, can receive notifications while waiting.
 * 
 */
static void notifyAllTasks(void) {
  xTaskNotifyGive(TaskSerialInfoCMD);
  xTaskNotifyGive(TaskStorage);
  xTaskNotifyGive(TaskMultiplexers);
  xTaskNotifyGive(TaskGyro);
  xTaskNotifyGive(TaskGPS);
  xTaskNotifyGive(TaskUniverse);
  xTaskNotifyGive(TaskSwitches);
  if (TaskDisplayUpdate != nullptr) { xTaskNotifyGive(TaskDisplayUpdate); }
}

void setTasksDelayLowPower() {
  pwrConfigCurrent = pwrConfigLowPower;
  notifyAllTasks();
}

void setTasksDelayBalanced() {
  pwrConfigCurrent = pwrConfigBalanced;
  notifyAllTasks();
}

void setTasksDelayUltimatePerformance() {
  pwrConfigCurrent = pwrConfigUltimatePerformance;
  notifyAllTasks();
}

void setTick(TaskHandle_t task_handle, bool *tick_delay, bool use_tick) {
  *tick_delay = use_tick;
  xTaskNotifyGive(task_handle);
}

void setDelay(TaskHandle_t task_handle, uint32_t *task_delay, uint32_t time_delay) {
  *task_delay = time_delay;
  xTaskNotifyGive(task_handle);
}

bool isTaskDelayed(TaskHandle_t taskHandle) {
  // Compound statement required for the if-body (MISRA C 2012 Rule 15.6).
  bool res = false;
  if (taskHandle == nullptr) {
    res = false;
  }
  else {
    eTaskState state = eTaskGetState(taskHandle);
    res = (state == eBlocked) || (state == eSuspended);
  }
  return res;
}

/** ----------------------------------------------------------------------------
 * Syncronize Tasks.
 *
 * @brief Time syncronize tasks. Main loop and some tasks will not begin until
 *        this function has completed.
 *
 *        Inital synchronization trigger is GPS milliseconds 00 (any second).
 *
 *        After initial synchronization the system will attempt to synchronize
 *        every GPS seconds zero (minutely).
 */
void syncTasks() {
  Serial.println("[syncronizing system] please wait");
  global_task_sync = false;
  // Boolean object used directly as the controlling expression (MISRA C 2012 Rule 14.4).
  while (satioData.sync_rtc_immediately_flag) {
    getSystemTime();
    system_sync_retry_max--;
    if (system_sync_retry_max <= 0) {
      Serial.println("[sync] took too long!");
      break;
    }
    delayMicroseconds(1);
  }
  global_task_sync = true;
}

/** ----------------------------------------------------------------------------------------
 * @brief Interval breach (System counters).
 *
 *        Referenced only inside this translation unit, so internal linkage keeps them out
 *        of the global symbol table (MISRA C 2012 Rule 8.7).
 */
static int64_t prev_tv_sec;
static int64_t prev_tv_uS_track_planets;
static int64_t prev_tv_uS_star_navigation;

/** ----------------------------------------------------------------------------
 * Interval Breach: 1 Second.
 *
 * @brief Stores time-of-day snapshots, rolls every per-task counter into its
 *        "total" field, resets the counter for the next second, and raises the
 *        1-second output flag.
 */
static void intervalBreach1Second(void) {
  storeLocalTime();
  storeRTCTime();
  storeLMST();

  systemData.total_loops_a_second = systemData.loops_a_second;
  systemData.loops_a_second = 0;

  systemData.total_gps = systemData.i_count_read_gps;
  systemData.i_count_read_gps = 0;

  systemData.total_ins = systemData.i_count_read_ins;
  systemData.i_count_read_ins = 0;

  systemData.total_gyro_0 = systemData.i_count_read_gyro_0;
  systemData.i_count_read_gyro_0 = 0;

  systemData.total_mplex_0 = systemData.i_count_read_mplex_0;
  systemData.i_count_read_mplex_0 = 0;

  systemData.total_matrix = systemData.i_count_matrix;
  systemData.i_count_matrix = 0;

  systemData.total_portcontroller_output = systemData.i_count_port_controller_output;
  systemData.i_count_port_controller_output = 0;

  systemData.total_universe = systemData.i_count_track_planets;
  systemData.i_count_track_planets = 0;

  systemData.total_infocmd = systemData.i_count_read_serial_commands;
  systemData.i_count_read_serial_commands = 0;

  systemData.total_portcontroller_input = systemData.i_count_portcontroller_input;
  systemData.i_count_portcontroller_input = 0;

  systemData.total_display = systemData.i_count_display;
  systemData.i_count_display = 0;

  systemData.interval_breach_track_planets_output = true;

  systemData.uptime_seconds++;
  // uptime_seconds is int32_t, so the wrap check uses the signed 32-bit limit
  // matching its essential type (MISRA C 2012 Rule 10.4).
  if (systemData.uptime_seconds >= INT32_MAX - 2) {
    systemData.uptime_seconds = 0;
    printf("[reset uptime_seconds] %ld\n", systemData.uptime_seconds);
  }
}

/** ----------------------------------------------------------------------------
 * System Timing.
 *
 * @brief Refreshes the local time-of-day, then raises interval-breach flags
 *        whenever the planet-tracking interval, the star-navigation interval,
 *        or a calendar second has elapsed since it was last raised.
 */
static void system_timing() {
  gettimeofday(&tv_now, NULL);
  timeinfo = localtime(&tv_now.tv_sec); // Assumes localtime works
  satioData.local_unixtime_uS = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

  // Track Planets interval breach.
  if (satioData.local_unixtime_uS >= prev_tv_uS_track_planets + POWER_CONFIG_TRACK_PLANTETS_TIMING_uS ||
      satioData.local_unixtime_uS <= prev_tv_uS_track_planets - POWER_CONFIG_TRACK_PLANTETS_TIMING_uS) {
    prev_tv_uS_track_planets = satioData.local_unixtime_uS;
    systemData.interval_breach_track_planets = true;
  }

  // StarNavigation interval breach.
  if (satioData.local_unixtime_uS >= prev_tv_uS_star_navigation + POWER_CONFIG_STAR_NAVIGATION_TIMING_uS ||
      satioData.local_unixtime_uS <= prev_tv_uS_star_navigation - POWER_CONFIG_STAR_NAVIGATION_TIMING_uS) {
    prev_tv_uS_star_navigation = satioData.local_unixtime_uS;
    systemData.interval_breach_star_navigation = true;
  }

  // 1-second interval breach.
  if (tv_now.tv_sec != prev_tv_sec) {
    prev_tv_sec = tv_now.tv_sec;
    systemData.interval_breach_1_second_output = true;
    intervalBreach1Second();
  }
}

/** ----------------------------------------------------------------------------
 * Storage Task.
 *
 * @brief Performs many operations including:
 *  (1) Card insertion checks.
 *  (2) Mount automatically.
 *  (3) Unmount automatically.
 *  (4) Read/write operations.
 *  (5) Other storage operations.
 *  (6) Powers down the sdcard when not in use.
 */
static void taskStorage(void *pvParameters) {
  (void)pvParameters; // FreeRTOS task signature requires the parameter; it is unused here (MISRA C 2012 Rule 2.7).
  esp_task_wdt_add(nullptr);
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // SDCard Detect/Mount
    // ------------------------------------------------
    sdcard_mount();
    esp_task_wdt_reset();

    // ------------------------------------------------
    // Check Flags
    // ------------------------------------------------
    if (systemData.logging_enabled) {
      Serial.printf("[log] setting write flag true\n");
      sdcardFlagData.write_log = true;
    }
    sdcardFlagHandler();
    esp_task_wdt_reset();

    // ------------------------------------------------
    // SDCard Power Down / Unmount
    // ------------------------------------------------
    sdcard_unmount();
    esp_task_wdt_reset();

    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (!pwrConfigCurrent.TASK_USE_TICKS_STORAGE) {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_STORAGE / portTICK_PERIOD_MS);
    } else {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_STORAGE);
    }
  }
}
void createTaskStorage() {
  xTaskCreatePinnedToCore(
    taskStorage,             /* Function to implement the task */
    "TaskStorage",           /* Name of the task */
    TASK_STORAGE_STACK_SIZE, /* Stack size in words */
    nullptr,                 /* Task input parameter */
    TASK_STORAGE_PRIORITY,   /* Priority of the task */
    &TaskStorage,            /* Task handle. */
    TASK_STORAGE_CORE);      /* Core where the task should run */
}

/** ----------------------------------------------------------------------------
 * Info Command Task.
 *
 * @brief Processes a serial TXD and RXD operations:
 *  (1) Information out for other system and debug.
 *  (2) Commands in.
 *
 *        System statistics are produced by the main loop; this task is
 *        responsible only for serial command input and status output.
 */
static void taskSerialInfoCMD(void *pvParameters) {
  (void)pvParameters; // FreeRTOS task signature requires the parameter; it is unused here (MISRA C 2012 Rule 2.7).
  esp_task_wdt_add(nullptr);
  while (!global_task_sync) {
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  for (;;) {
    esp_task_wdt_reset();
    outputSentences();
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (!pwrConfigCurrent.TASK_USE_TICKS_INFOCMD) {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_INFOCMD / portTICK_PERIOD_MS);
    } else {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_INFOCMD);
    }
  }
}
void createTaskSerialInfoCMD() {
  xTaskCreatePinnedToCore(
    taskSerialInfoCMD,             /* Function to implement the task */
    "TaskSerialInfoCMD",           /* Name of the task */
    TASK_SERIALINFOCMD_STACK_SIZE, /* Stack size in words */
    nullptr,                       /* Task input parameter */
    TASK_SERIALINFOCMD_PRIORITY,   /* Priority of the task */
    &TaskSerialInfoCMD,            /* Task handle. */
    TASK_SERIALINFOCMD_CORE);      /* Core where the task should run */
}

/** ----------------------------------------------------------------------------
 * GPS Task.
 *
 * @brief Reads and validates GPS sentences, commits the resulting position
 *        and timing data, and updates the inertial navigation system.
 */
static void taskGPS(void *pvParameters) {
  (void)pvParameters; // FreeRTOS task signature requires the parameter; it is unused here (MISRA C 2012 Rule 2.7).
  esp_task_wdt_add(nullptr);
  for (;;) {
    esp_task_wdt_reset();

    system_timing();
    // ------------------------------------------------
    // Get, check and set gps data.
    // ------------------------------------------------
    gnggaData.valid_checksum = false;
    gnrmcData.valid_checksum = false;
    gpattData.valid_checksum = false;
    readGPS();
    esp_task_wdt_reset();
    validateGPSData();
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Set SatIO data once every sentence checksum is valid, or when an RTC
    // time sync has been explicitly requested.
    // ------------------------------------------------
    if ((gnggaData.valid_checksum && gnrmcData.valid_checksum && gpattData.valid_checksum) ||
        satioData.set_rtc_datetime_flag) {

      // -> syncRTC -- > setRTCDateTime --> setSystemTime, storeRTCSYNCTime
      satioData.set_rtc_datetime_flag = true;
      setSatIOData();
      esp_task_wdt_reset();

      // --------------------------------------------
      // Set INS data. (Can be used without GPS)
      // --------------------------------------------
      set_ins(satioData.system_degrees_latitude,
              satioData.system_degrees_latitude,
              satioData.system_altitude,
              satioData.system_ground_heading,
              satioData.system_speed,
              atof(gnggaData.gps_precision_factor),
              gyroData.gyro_0_ang_z);
      esp_task_wdt_reset();

      // --------------------------------------------
      // Counters.
      // --------------------------------------------
      systemData.i_count_read_gps++;
      systemData.interval_breach_gps_output = true;
      // i_count_read_gps is int32_t, so the wrap check uses the signed 32-bit
      // limit matching its essential type (MISRA C 2012 Rule 10.4).
      if (systemData.i_count_read_gps >= INT32_MAX - 2) {
        systemData.i_count_read_gps = 0;
      }
      esp_task_wdt_reset();
    }
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (!pwrConfigCurrent.TASK_USE_TICKS_GPS) {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_GPS / portTICK_PERIOD_MS);
    } else {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_GPS);
    }
  }
}
void createTaskGPS() {
  xTaskCreatePinnedToCore(
    taskGPS,             /* Function to implement the task */
    "TaskGPS",           /* Name of the task */
    TASK_GPS_STACK_SIZE, /* Stack size in words */
    nullptr,             /* Task input parameter */
    TASK_GPS_PRIORITY,   /* Priority of the task */
    &TaskGPS,            /* Task handle. */
    TASK_GPS_CORE);      /* Core where the task should run */
}

/** ----------------------------------------------------------------------------
 * Gyro Task.
 *
 * @brief Reads and stores gyroscopic data, and feeds it into the inertial
 *        navigation position estimate.
 */
static void taskGyro(void *pvParameters) {
  (void)pvParameters; // FreeRTOS task signature requires the parameter; it is unused here (MISRA C 2012 Rule 2.7).
  esp_task_wdt_add(nullptr);
  while (!global_task_sync) {
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  for (;;) {
    esp_task_wdt_reset();
    if (readGyro()) {
      esp_task_wdt_reset();
      systemData.i_count_read_gyro_0++;
      systemData.interval_breach_gyro_0_output = true;
      // i_count_read_gyro_0 is int32_t, so the wrap check uses the signed
      // 32-bit limit matching its essential type (MISRA C 2012 Rule 10.4).
      if (systemData.i_count_read_gyro_0 >= INT32_MAX - 2) {
        systemData.i_count_read_gyro_0 = 0;
      }
      esp_task_wdt_reset();
      // ----------------------------------------------
      // Estimate INS data. (Can be used without GPS)
      // INS data is fed back into INS.
      // ----------------------------------------------
      if (systemData.interval_breach_gyro_0_output) {
        if (ins_estimate_position(gyroData.gyro_0_ang_y,
                                   gyroData.gyro_0_ang_z,
                                   satioData.system_ground_heading,
                                   satioData.system_speed,
                                   satioData.local_unixtime_uS)) {
          systemData.i_count_read_ins++;
          systemData.interval_breach_ins_output = true;
          // i_count_read_ins is int32_t, so the wrap check uses the signed
          // 32-bit limit matching its essential type (MISRA C 2012 Rule 10.4).
          if (systemData.i_count_read_ins >= INT32_MAX - 2) {
            systemData.i_count_read_ins = 0;
          }
        }
        esp_task_wdt_reset();
      }
    }
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (!pwrConfigCurrent.TASK_USE_TICKS_GYRO) {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_GYRO / portTICK_PERIOD_MS);
    } else {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_GYRO);
    }
  }
}
void createTaskGyro() {
  xTaskCreatePinnedToCore(
    taskGyro,             /* Function to implement the task */
    "TaskGyro",           /* Name of the task */
    TASK_GYRO_STACK_SIZE, /* Stack size in words */
    nullptr,              /* Task input parameter */
    TASK_GYRO_PRIORITY,   /* Priority of the task */
    &TaskGyro,            /* Task handle. */
    TASK_GYRO_CORE);      /* Core where the task should run */
}

/** ----------------------------------------------------------------------------
 * Multiplexer Task.
 *
 * @brief Reads all analog/digital multiplexer channels.
 */
static void taskMultiplexers(void *pvParameters) {
  (void)pvParameters; // FreeRTOS task signature requires the parameter; it is unused here (MISRA C 2012 Rule 2.7).
  esp_task_wdt_add(nullptr);
  while (!global_task_sync) {
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Read multiplexer channels (customize as required).
    // ------------------------------------------------
    setReadModeADMultiplexer(ad_mux_0);
    for (uint8_t i_chan = 0; i_chan < MAX_AD_MUX_CHANNELS; i_chan++) {
      readADMultiplexerAnalogChannel(ad_mux_0, i_chan);
      vTaskDelay(1);
    }
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Counters
    // ------------------------------------------------
    systemData.i_count_read_mplex_0++;
    systemData.interval_breach_mplex_0_output = true;
    // i_count_read_mplex_0 is int32_t, so the wrap check uses the signed
    // 32-bit limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_count_read_mplex_0 >= INT32_MAX - 2) {
      systemData.i_count_read_mplex_0 = 0;
    }
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (!pwrConfigCurrent.TASK_USE_TICKS_MULTIPLEXERS) {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_MULTIPLEXERS / portTICK_PERIOD_MS);
    } else {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_MULTIPLEXERS);
    }
  }
}
void createTaskMultiplexers() {
  xTaskCreatePinnedToCore(
    taskMultiplexers,             /* Function to implement the task */
    "TaskMultiplexers",           /* Name of the task */
    TASK_MULTIPLEXERS_STACK_SIZE, /* Stack size in words */
    nullptr,                      /* Task input parameter */
    TASK_MULTIPLEXERS_PRIORITY,   /* Priority of the task */
    &TaskMultiplexers,            /* Task handle. */
    TASK_MULTIPLEXERS_CORE);      /* Core where the task should run */
}

/** ----------------------------------------------------------------------------
 * Switch Task.
 *
 * @brief Performs various operations including:
 *  (1) Matrix calculations.
 *  (2) Mapping values.
 *  (3) Sets output values.
 *  (4) Instructing the portcontroller accordingly.
 */
static void taskSwitches(void *pvParameters) {
  (void)pvParameters; // FreeRTOS task signature requires the parameter; it is unused here (MISRA C 2012 Rule 2.7).
  esp_task_wdt_add(nullptr);
  while (!global_task_sync) {
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  for (;;) {
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Calculate.
    // ------------------------------------------------
    if (matrixSwitch()) {
      esp_task_wdt_reset();
      systemData.i_count_matrix++;
      systemData.interval_breach_matrix_output = true;
      // i_count_matrix is int32_t, so the wrap check uses the signed 32-bit
      // limit matching its essential type (MISRA C 2012 Rule 10.4).
      if (systemData.i_count_matrix >= INT32_MAX - 2) {
        systemData.i_count_matrix = 0;
      }
    }
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Mapping.
    // ------------------------------------------------
    map_values();
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Output.
    // ------------------------------------------------
    setOutputValues();
    esp_task_wdt_reset();
    writeOutputPortControllerSetPins(iic_2, I2C_ADDR_OUTPUT_PORTCONTROLLER);
    esp_task_wdt_reset();
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (!pwrConfigCurrent.TASK_USE_TICKS_SWITCHES) {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_SWITCHES / portTICK_PERIOD_MS);
    } else {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_SWITCHES);
    }
  }
}
void createTaskSwitches() {
  xTaskCreatePinnedToCore(
    taskSwitches,             /* Function to implement the task */
    "TaskSwitches",           /* Name of the task */
    TASK_SWITCHES_STACK_SIZE, /* Stack size in words */
    nullptr,                  /* Task input parameter */
    TASK_SWITCHES_PRIORITY,   /* Priority of the task */
    &TaskSwitches,            /* Task handle. */
    TASK_SWITCHES_CORE);      /* Core where the task should run */
}

/** ----------------------------------------------------------------------------
 * Universe Task.
 *
 * @brief Stores various information about the universe!
 *
 *        Tracks planets and meteor showers once per the configured interval,
 *        and continuously evaluates the nearest catalogue object to both the
 *        system zenith and the zenith offset by the gyroscope attitude.
 */
static void taskUniverse(void *pvParameters) {
  (void)pvParameters; // FreeRTOS task signature requires the parameter; it is unused here (MISRA C 2012 Rule 2.7).
  esp_task_wdt_add(nullptr);
  while (!global_task_sync) {
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  for (;;) {
    esp_task_wdt_reset();

    // ------------------------------------------------
    // Set Sidereal Data for Planet/Object Tracking.
    // ------------------------------------------------
    setSiderealData(
      satioData.system_degrees_latitude,
      satioData.system_degrees_longitude,
      satioData.rtc_year,
      satioData.rtc_month,
      satioData.rtc_mday,
      satioData.rtc_hour,
      satioData.rtc_minute,
      satioData.rtc_second,
      satioData.local_hour,
      satioData.local_minute,
      satioData.local_second,
      satioData.system_altitude);

    // ------------------------------------------------
    // Track Planets/Meteors Every Interval (see config.h)
    // ------------------------------------------------
    if (systemData.interval_breach_track_planets) {
      systemData.interval_breach_track_planets = false;

      esp_task_wdt_reset();
      trackPlanets();
      esp_task_wdt_reset();
      setMeteorShowerWarning(satioData.local_month, satioData.local_mday);
      esp_task_wdt_reset();
    }

    // ------------------------------------------------
    // Set RA & Dec for system zenith. (add to matrix)
    // ------------------------------------------------
    siderealExtraData.local_zenith_ra_dec = myAstro.getRADecFromLSTLat(
      siderealExtraData.local_sidereal_time,
      satioData.system_degrees_latitude);
    esp_task_wdt_reset();

    // ------------------------------------------------
    // Set RA & Dec for system zenith +- Gyro. (add to matrix)
    // ------------------------------------------------
    siderealExtraData.gyro_0_ra_dec = gyroOffsetZenithRADec(gyroData.gyro_0_ang_z, gyroData.gyro_0_ang_y);
    esp_task_wdt_reset();

    // ------------------------------------------------
    // StarNav Dynamic Test Zenith Every Interval
    // ------------------------------------------------
    setStarNav(
      siderealExtraData.local_zenith_ra_dec.ra_h,
      siderealExtraData.local_zenith_ra_dec.ra_m,
      siderealExtraData.local_zenith_ra_dec.ra_s,
      siderealExtraData.local_zenith_ra_dec.dec_d,
      siderealExtraData.local_zenith_ra_dec.dec_m,
      siderealExtraData.local_zenith_ra_dec.dec_s
    );
    esp_task_wdt_reset();

    // ------------------------------------------------
    // StarNav Dynamic Test Zenith+-Gyro Offset
    // ------------------------------------------------
    setStarNav(
      siderealExtraData.gyro_0_ra_dec.ra_h,
      siderealExtraData.gyro_0_ra_dec.ra_m,
      siderealExtraData.gyro_0_ra_dec.ra_s,
      siderealExtraData.gyro_0_ra_dec.dec_d,
      siderealExtraData.gyro_0_ra_dec.dec_m,
      siderealExtraData.gyro_0_ra_dec.dec_s
    );
    esp_task_wdt_reset();

    // ------------------------------------------------
    // Set counters and flags
    // ------------------------------------------------
    systemData.i_count_track_planets++;
    systemData.interval_breach_track_planets_output = true;
    // i_count_track_planets is int32_t, so the wrap check uses the signed
    // 32-bit limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_count_track_planets >= INT32_MAX - 2) {
      systemData.i_count_track_planets = 0;
    }
    esp_task_wdt_reset();

    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (!pwrConfigCurrent.TASK_USE_TICKS_UNIVERSE) {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_UNIVERSE / portTICK_PERIOD_MS);
    } else {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_UNIVERSE);
    }
  }
}
void createTaskUniverse() {
  xTaskCreatePinnedToCore(
    taskUniverse,             /* Function to implement the task */
    "TaskUniverse",           /* Name of the task */
    TASK_UNIVERSE_STACK_SIZE, /* Stack size in words */
    nullptr,                  /* Task input parameter */
    TASK_UNIVERSE_PRIORITY,   /* Priority of the task */
    &TaskUniverse,            /* Task handle. */
    TASK_UNIVERSE_CORE);      /* Core where the task should run */
}

/** ----------------------------------------------------------------------------
 * Display Update Task.
 *
 * @brief Drives LVGL screen updates at the lowest user-task priority so it is
 *        preempted by every other task regardless of how long a frame takes.
 *
 *        Acquires the BSP display lock before calling update_display() so all
 *        LVGL API calls are thread-safe.  The lock is released between frames
 *        so the BSP LVGL task can service touch events and DMA completions
 *        during the idle window.
 */
static void taskDisplayUpdate(void *pvParameters) {
  bool locked = false;
  (void)pvParameters;
  for (;;) {
    locked = bsp_display_lock(portMAX_DELAY);
    if (locked) {
      update_display();
      bsp_display_unlock();
    }
    // ------------------------------------------------
    // Set counters and flags
    // ------------------------------------------------
    systemData.i_count_display++;
    // i_count_track_planets is int32_t, so the wrap check uses the signed
    // 32-bit limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_count_display >= INT32_MAX - 2) {
      systemData.i_count_display = 0;
    }
    // ------------------------------------------------
    // Delay next iteration of task.
    // ------------------------------------------------
    if (!pwrConfigCurrent.TASK_USE_TICKS_DISPLAY) {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_DISPLAY / portTICK_PERIOD_MS);
    } else {
      xTaskNotifyWait(0x00, 0x00, nullptr, pwrConfigCurrent.TASK_DELAY_DISPLAY);
    }
  }
}

void createTaskDisplayUpdate() {
  (void)xTaskCreatePinnedToCore(
    taskDisplayUpdate,          /* Function to implement the task */
    "TaskDisplayUpdate",        /* Name of the task */
    TASK_DISPLAY_STACK_SIZE,    /* Stack size in words */
    nullptr,                    /* Task input parameter */
    TASK_DISPLAY_PRIORITY,      /* Priority of the task */
    &TaskDisplayUpdate,         /* Task handle. */
    TASK_DISPLAY_CORE);         /* Core where the task should run */
}

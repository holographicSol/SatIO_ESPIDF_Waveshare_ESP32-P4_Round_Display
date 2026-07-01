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
#define TASK_GPS_PRIORITY                   5
#define TASK_GYRO_PRIORITY                  4
#define TASK_MULTIPLEXERS_PRIORITY          4
#define TASK_SWITCHES_PRIORITY              4
#define TASK_SERIALINFOCMD_PRIORITY         3
#define TASK_UNIVERSE_PRIORITY              3
#define TASK_STORAGE_PRIORITY               3

// CORE ASSIGNMENT
#define TASK_SERIALINFOCMD_CORE             1
#define TASK_GYRO_CORE                      1
#define TASK_MULTIPLEXERS_CORE              1
#define TASK_SWITCHES_CORE                  1
#define TASK_UNIVERSE_CORE                  1
#define TASK_STORAGE_CORE                   1
#define TASK_GPS_CORE                       1

// STACK SIZES
#define TASK_STORAGE_STACK_SIZE             6144
#define TASK_SERIALINFOCMD_STACK_SIZE       16384
#define TASK_GPS_STACK_SIZE                 5120
#define TASK_GYRO_STACK_SIZE                4608
#define TASK_MULTIPLEXERS_STACK_SIZE        4096
#define TASK_SWITCHES_STACK_SIZE            5120
#define TASK_UNIVERSE_STACK_SIZE            20480

/* Display task configuration */
#define TASK_DISPLAY_PRIORITY    5
#define TASK_DISPLAY_CORE        0
#define TASK_DISPLAY_STACK_SIZE  32768

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

void setDelay(TaskHandle_t task_handle, uint32_t delay_in, uint32_t *delay_out) {
  if (delay_in > 0) {
    *delay_out = delay_in;
    xTaskNotifyGive(task_handle);
  }
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

  systemData.total_task_freq_hz_gps = systemData.i_task_freq_hz_gps;
  systemData.i_task_freq_hz_gps = 0;

  systemData.total_ins = systemData.i_count_read_ins;
  systemData.i_count_read_ins = 0;

  systemData.total_gyro_0 = systemData.i_count_read_gyro_0;
  systemData.i_count_read_gyro_0 = 0;

  systemData.total_task_freq_hz_gyro = systemData.i_task_freq_hz_gyro;
  systemData.i_task_freq_hz_gyro = 0;

  systemData.total_mplex_0 = systemData.i_count_read_mplex_0;
  systemData.i_count_read_mplex_0 = 0;

  systemData.total_task_freq_hz_mlx = systemData.i_task_freq_hz_mlx;
  systemData.i_task_freq_hz_mlx = 0;

  systemData.total_matrix = systemData.i_count_matrix;
  systemData.i_count_matrix = 0;

  systemData.total_task_freq_hz_switches = systemData.i_task_freq_hz_switches;
  systemData.i_task_freq_hz_switches = 0;

  systemData.total_portcontroller_output = systemData.i_count_port_controller_output;
  systemData.i_count_port_controller_output = 0;

  systemData.total_universe = systemData.i_count_track_planets;
  systemData.i_count_track_planets = 0;

  systemData.total_task_freq_hz_uni = systemData.i_task_freq_hz_uni;
  systemData.i_task_freq_hz_uni = 0;

  systemData.total_infocmd = systemData.i_count_read_serial_commands;
  systemData.i_count_read_serial_commands = 0;

  systemData.total_portcontroller_input = systemData.i_count_portcontroller_input;
  systemData.i_count_portcontroller_input = 0;

  systemData.total_display = systemData.i_count_display;
  systemData.i_count_display = 0;

  systemData.total_task_freq_hz_dsp = systemData.i_task_freq_hz_dsp;
  systemData.i_task_freq_hz_dsp = 0;

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
void system_timing(void) {
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

// Drift-free, notification-responsive task frequency gate.
// Tracks an absolute xLastWakeTime like vTaskDelayUntil, but passes only the
// remaining ticks to xTaskNotifyWait so a notification (e.g. Hz change via
// notifyAllTasks) unblocks the task immediately. The loop then re-reads the
// current Hz/ms setting and recomputes the remaining time before the deadline.
// When the deadline is reached, xLastWakeTime advances by exactly one period.
#define TASK_FREQ_WAIT(delay_field)                                            \
  do {                                                                      \
    static TickType_t xLastWakeTime = 0;                                    \
    static bool initialized = false;                                        \
    if (!initialized) {                                                     \
      xLastWakeTime = xTaskGetTickCount();                                  \
      initialized = true;                                                   \
    }                                                                       \
    TickType_t xPeriod;                                                     \
    TickType_t xElapsed;                                                    \
    do {                                                                    \
      xPeriod  = pdMS_TO_TICKS(pwrConfigCurrent.delay_field);          \
      xElapsed = xTaskGetTickCount() - xLastWakeTime;                       \
      if (xElapsed < xPeriod) {                                             \
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, nullptr, xPeriod - xElapsed); \
      }                                                                     \
    } while ((xTaskGetTickCount() - xLastWakeTime) < xPeriod);             \
    xLastWakeTime += xPeriod;                                               \
  } while (0)

bool taskFrequencyGPS()         { TASK_FREQ_WAIT(TASK_MAX_FREQ_MS_GPS);         return true; }
bool taskFrequencyGyro()        { TASK_FREQ_WAIT(TASK_MAX_FREQ_MS_GYRO);        return true; }
bool taskFrequencySwitches()    { TASK_FREQ_WAIT(TASK_MAX_FREQ_MS_SWITCHES);    return true; }
bool taskFrequencyStorage()     { TASK_FREQ_WAIT(TASK_MAX_FREQ_MS_STORAGE);     return true; }
bool taskFrequencyInfoCMD()     { TASK_FREQ_WAIT(TASK_MAX_FREQ_MS_INFOCMD);     return true; }
bool taskFrequencyMultiplexers(){ TASK_FREQ_WAIT(TASK_MAX_FREQ_MS_MULTIPLEXERS);return true; }
bool taskFrequencyUniverse()    { TASK_FREQ_WAIT(TASK_MAX_FREQ_MS_UNIVERSE);    return true; }
bool taskFrequencyDisplay()     { TASK_FREQ_WAIT(TASK_MAX_FREQ_MS_DISPLAY);     return true; }

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

    // Delay Task
    if (taskFrequencyGPS() == true) {

      if (readGPS() == true)
      {
        if (validateGPSData() == true)
        {
          // --------------------------------------------
          // Set SatIO Data
          // --------------------------------------------
          setSatIOData();

          // --------------------------------------------
          // Set INS data
          // --------------------------------------------
          set_ins(satioData.system_degrees_latitude,
                  satioData.system_degrees_latitude,
                  satioData.system_altitude,
                  satioData.system_ground_heading,
                  satioData.system_speed,
                  atof(gnggaData.gps_precision_factor),
                  gyroData.gyro_0_ang_z);;

          // --------------------------------------------
          // Read Counters.
          // --------------------------------------------
          systemData.i_count_read_gps++;
          systemData.interval_breach_gps_output = true;
          // i_count_read_gps is int32_t, so the wrap check uses the signed 32-bit
          // limit matching its essential type (MISRA C 2012 Rule 10.4).
          if (systemData.i_count_read_gps >= INT32_MAX - 2) {
            systemData.i_count_read_gps = 0;
          }
        }
      }
      // /**
      //  * System Timing.
      //  * Settle for a system time resolution of TASK_FREQ_HZ_GPS, if no task is
      //  * performance capable of providing every ms, or better uS, anyway.
      //  * Otherwise we could move this call elsewhere.
      //  */
      // system_timing();
    }
    // --------------------------------------------
    // Task Iter Counters.
    // --------------------------------------------
    systemData.i_task_freq_hz_gps++;
    // i_task_freq_hz_gps is int32_t, so the wrap check uses the signed 32-bit
    // limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_task_freq_hz_gps >= INT32_MAX - 2) {
      systemData.i_task_freq_hz_gps = 0;
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

    // Delay Task
    if (taskFrequencyStorage() == true) {
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

    // Delay Task
    if (taskFrequencyInfoCMD() == true) {
      esp_task_wdt_reset();
      outputSentences();
      esp_task_wdt_reset();
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

    // Delay Task
    if (taskFrequencyGyro() == true) {

      if (readGyro() == true) {
        esp_task_wdt_reset();
        systemData.i_count_read_gyro_0++;
        systemData.interval_breach_gyro_0_output = true;
        // i_count_read_gyro_0 is int32_t, so the wrap check uses the signed
        // 32-bit limit matching its essential type (MISRA C 2012 Rule 10.4).
        if (systemData.i_count_read_gyro_0 >= INT32_MAX - 2) {
          systemData.i_count_read_gyro_0 = 0;
        }
        // // ----------------------------------------------
        // // Estimate INS data. (Can be used without GPS)
        // // INS data is fed back into INS.
        // // ----------------------------------------------
        // if (ins_estimate_position(gyroData.gyro_0_ang_y,
        //                             gyroData.gyro_0_ang_z,
        //                             satioData.system_ground_heading,
        //                             satioData.system_speed,
        //                             satioData.local_unixtime_uS)) {
        //   systemData.i_count_read_ins++;
        //   systemData.interval_breach_ins_output = true;
        //   // i_count_read_ins is int32_t, so the wrap check uses the signed
        //   // 32-bit limit matching its essential type (MISRA C 2012 Rule 10.4).
        //   if (systemData.i_count_read_ins >= INT32_MAX - 2) {
        //     systemData.i_count_read_ins = 0;
        //   }
          // esp_task_wdt_reset();
        }
      }
    // --------------------------------------------
    // Task Iter Counters.
    // --------------------------------------------
    systemData.i_task_freq_hz_gyro++;
    // i_task_freq_hz_gyro is int32_t, so the wrap check uses the signed 32-bit
    // limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_task_freq_hz_gyro >= INT32_MAX - 2) {
      systemData.i_task_freq_hz_gyro = 0;
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

    // Delay Task
    if (taskFrequencyMultiplexers() == true) {
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
    }

    // --------------------------------------------
    // Task Iter Counters.
    // --------------------------------------------
    systemData.i_task_freq_hz_mlx++;
    // i_task_freq_hz_mlx is int32_t, so the wrap check uses the signed 32-bit
    // limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_task_freq_hz_mlx >= INT32_MAX - 2) {
      systemData.i_task_freq_hz_mlx = 0;
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

    // Delay Task
    if (taskFrequencySwitches() == true) {
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
    }

    // --------------------------------------------
    // Task Iter Counters.
    // --------------------------------------------
    systemData.i_task_freq_hz_switches++;
    // i_task_freq_hz_switches is int32_t, so the wrap check uses the signed 32-bit
    // limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_task_freq_hz_switches >= INT32_MAX - 2) {
      systemData.i_task_freq_hz_switches = 0;
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

    // Delay Task
    if (taskFrequencyUniverse() == true) {
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
    }

    // --------------------------------------------
    // Task Iter Counters.
    // --------------------------------------------
    systemData.i_task_freq_hz_uni++;
    // i_task_freq_hz_uni is int32_t, so the wrap check uses the signed 32-bit
    // limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_task_freq_hz_uni >= INT32_MAX - 2) {
      systemData.i_task_freq_hz_uni = 0;
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

    // Delay Task
    if (taskFrequencyDisplay() == true) {
      locked = bsp_display_lock(portMAX_DELAY);
      if (locked) {
        update_display();
        bsp_display_unlock();
      }
      // ------------------------------------------------
      // Set counters and flags
      // ------------------------------------------------
      systemData.i_count_display++;
      // i_count_display is int32_t, so the wrap check uses the signed
      // 32-bit limit matching its essential type (MISRA C 2012 Rule 10.4).
      if (systemData.i_count_display >= INT32_MAX - 2) {
        systemData.i_count_display = 0;
      }
    }

    // --------------------------------------------
    // Task Iter Counters.
    // --------------------------------------------
    systemData.i_task_freq_hz_dsp++;
    // i_task_freq_hz_dsp is int32_t, so the wrap check uses the signed 32-bit
    // limit matching its essential type (MISRA C 2012 Rule 10.4).
    if (systemData.i_task_freq_hz_dsp >= INT32_MAX - 2) {
      systemData.i_task_freq_hz_dsp = 0;
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

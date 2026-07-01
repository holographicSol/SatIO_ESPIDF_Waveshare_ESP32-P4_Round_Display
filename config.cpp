/**
 * Config - Written by Benjamin Jack Cullen.
 *
*/

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

bool global_task_sync      = false;
long system_sync_retry_max = 10000;

/**
 * @brief Global IIC wire instances and data struictures for each wire channel.
 */
// TwoWire iic_0(0); // wire channel 0
// TwoWire iic_1(1); // wire channel 1
// TwoWire iic_2(2); // wire channel 2

struct PwrConfig pwrConfigLowPower = {
    .name = {"Low Power"},

    .TASK_MAX_FREQ_HZ_GPS = TASK_MAX_FREQ_HZ_LOW_GPS,

    .TASK_MAX_FREQ_HZ_INFOCMD = TASK_MAX_FREQ_HZ_LOW_SERIAL_INFOCMD,

    .TASK_MAX_FREQ_HZ_MULTIPLEXERS = TASK_MAX_FREQ_HZ_LOW_MULTIPLEXERS,

    .TASK_MAX_FREQ_HZ_GYRO = TASK_MAX_FREQ_HZ_LOW_GYRO,

    .TASK_MAX_FREQ_HZ_UNIVERSE = TASK_MAX_FREQ_HZ_LOW_UNIVERSE,

    .TASK_MAX_FREQ_HZ_SWITCHES = TASK_MAX_FREQ_HZ_LOW_SWITCHES,

    .TASK_MAX_FREQ_HZ_PORTCONTROLLER_INPUT = TASK_MAX_FREQ_HZ_LOW_PORTCONTROLLER_INPUT,

    .TASK_MAX_FREQ_HZ_STORAGE = TASK_MAX_FREQ_HZ_LOW_STORAGE,

    .TASK_MAX_FREQ_HZ_DISPLAY = TASK_MAX_FREQ_HZ_LOW_DISPLAY,
};

struct PwrConfig pwrConfigBalanced = {
    .name = {"Balanced"},

    .TASK_MAX_FREQ_HZ_GPS = TASK_MAX_FREQ_HZ_BALANCED_GPS,

    .TASK_MAX_FREQ_HZ_INFOCMD = TASK_MAX_FREQ_HZ_BALANCED_SERIAL_INFOCMD,

    .TASK_MAX_FREQ_HZ_MULTIPLEXERS = TASK_MAX_FREQ_HZ_BALANCED_MULTIPLEXERS,

    .TASK_MAX_FREQ_HZ_GYRO = TASK_MAX_FREQ_HZ_BALANCED_GYRO,

    .TASK_MAX_FREQ_HZ_UNIVERSE = TASK_MAX_FREQ_HZ_BALANCED_UNIVERSE,

    .TASK_MAX_FREQ_HZ_SWITCHES = TASK_MAX_FREQ_HZ_BALANCED_SWITCHES,

    .TASK_MAX_FREQ_HZ_PORTCONTROLLER_INPUT = TASK_MAX_FREQ_HZ_BALANCED_PORTCONTROLLER_INPUT,

    .TASK_MAX_FREQ_HZ_STORAGE = TASK_MAX_FREQ_HZ_BALANCED_STORAGE,

    .TASK_MAX_FREQ_HZ_DISPLAY = TASK_MAX_FREQ_HZ_BALANCED_DISPLAY,
};

struct PwrConfig pwrConfigUltimatePerformance = {
    .name = {"Ultimate Performance"},

    .TASK_MAX_FREQ_HZ_GPS = TASK_MAX_FREQ_HZ_HIGH__GPS,

    .TASK_MAX_FREQ_HZ_INFOCMD = TASK_MAX_FREQ_HZ_HIGH__SERIAL_INFOCMD,

    .TASK_MAX_FREQ_HZ_MULTIPLEXERS = TASK_MAX_FREQ_HZ_HIGH__MULTIPLEXERS,

    .TASK_MAX_FREQ_HZ_GYRO = TASK_MAX_FREQ_HZ_HIGH__GYRO,

    .TASK_MAX_FREQ_HZ_UNIVERSE = TASK_MAX_FREQ_HZ_HIGH__UNIVERSE,

    .TASK_MAX_FREQ_HZ_SWITCHES = TASK_MAX_FREQ_HZ_HIGH__SWITCHES,

    .TASK_MAX_FREQ_HZ_PORTCONTROLLER_INPUT = TASK_MAX_FREQ_HZ_HIGH__PORTCONTROLLER_INPUT,

    .TASK_MAX_FREQ_HZ_STORAGE = TASK_MAX_FREQ_HZ_HIGH__STORAGE,

    .TASK_MAX_FREQ_HZ_DISPLAY = TASK_MAX_FREQ_HZ_HIGH__DISPLAY,
};

/**
 * @brief Current power configuration for the system.
 * @note Setup as required (default set to Balanced).
 */
struct PwrConfig pwrConfigCurrent = pwrConfigBalanced;
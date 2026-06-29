/*
    TaskHandler - Written By Benjamin Jack Cullen.

    MISRA conventions used throughout this library:
    (1) Every if/else/while/for body is a braced compound statement (MISRA C 2012 Rule 15.6).
    (2) Boolean objects are tested directly instead of being compared to true/false
        (MISRA C 2012 Rule 14.4 - essentially Boolean controlling expressions).
    (3) Null pointers are written as nullptr, the type-safe C++ null pointer constant.

    Intended to be MISRA Compliant (untested, unverified, in-progress).
*/

#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <Arduino.h>
#include "i2c_helper.h"

/* Handles of the FreeRTOS tasks created by this library, exposed so that other
   translation units can query task state or retarget a task's delay/tick setting. */
extern TaskHandle_t TaskSerialInfoCMD;
extern TaskHandle_t TaskStorage;
extern TaskHandle_t TaskMultiplexers;
extern TaskHandle_t TaskGyro;
extern TaskHandle_t TaskGPS;
extern TaskHandle_t TaskUniverse;
extern TaskHandle_t TaskSwitches;

void createTaskSerialInfoCMD();
void createTaskStorage();
void createTaskMultiplexers();
void createTaskGyro();
void createTaskGPS();
void createTaskUniverse();
void createTaskSwitches();

/** ----------------------------------------------------------------------------
 * Syncronize Tasks.
 *
 * @brief Time syncronize tasks.
 */
void syncTasks();

/** ----------------------------------------------------------------------------
 * Is Task Delayed.
 *
 * @brief Returns bool for task delayed.
 */
bool isTaskDelayed(TaskHandle_t taskHandle);

/** ----------------------------------------------------------------------------
 * PowerCfg: Ultimate Performance.
 *
 * @brief Sets all task delay timings to optimum performance.
 */
void setTasksDelayUltimatePerformance();

/** ----------------------------------------------------------------------------
 * PowerCfg: Power Saving.
 *
 * @brief Sets all task delay timings to power saving.
 */
void setTasksDelayPowerSaving();

/** ----------------------------------------------------------------------------
 * Set Tick.
 *
 * @brief Manually override use of millisecond or ticks for delays.
 */
void setTick(TaskHandle_t task_handle, bool *tick_delay, bool use_tick);

/** ----------------------------------------------------------------------------
 * Set Delay.
 *
 * @brief Manually override delay timing.
 */
void setDelay(TaskHandle_t task_handle, long *task_delay, long time_delay);

#endif // TASK_HANDLER_H

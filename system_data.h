/*
    System Data. Written by Benjamin Jack Cullen.

    Declares the single global systemData instance: per-subsystem interval
    flags, output-enable flags, loop counters, and the per-second totals
    derived from them. Other modules read and write these fields directly
    to learn, or change, what the system is currently doing.

    MISRA-relevant conventions used throughout this module:
    - Every counter and total uses a fixed-width type from <stdint.h>, so
      its size and signedness are explicit (Dir 4.6).
    - Every flag uses bool from <stdbool.h> rather than int (Rule 10.1).
    - systemData has exactly one extern declaration, here, and exactly one
      definition, in system_data.cpp, so its external linkage is
      unambiguous (Rule 8.5).
    - restore_system_defaults() declares its parameter list as (void), so
      a call site cannot pass it arguments it does not use (Rule 8.2).

    Intended to be MISRA Compliant (untested, unverified, in-progress).
*/

#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "config.h"

/**
 * @struct systemStruct
 * @brief Single global instance (systemData) holding system-wide flags,
 *        loop counters, and per-second totals.
 */
struct systemStruct {
  // ------------------------------------------------------------------------
  // Output interval-breach flags: true once the configured output interval
  // has elapsed for the named subsystem, signalling this loop iteration to
  // send that subsystem's output.
  // ------------------------------------------------------------------------
  bool interval_breach_gps_output;
  bool interval_breach_ins_output;
  bool interval_breach_gyro_0_output;
  bool interval_breach_mplex_0_output;
  bool interval_breach_matrix_output;
  bool interval_breach_track_planets_output;
  bool interval_breach_logging;
  bool interval_breach_1_second_output;

  // ------------------------------------------------------------------------
  // Compute interval-breach flags: true once the configured recompute
  // interval has elapsed. Distinct from the output flags above, which gate
  // sending a result that has already been computed.
  // ------------------------------------------------------------------------
  bool interval_breach_track_planets;
  bool interval_breach_star_navigation;

  // ------------------------------------------------------------------------
  // Diagnostics and command processing.
  // ------------------------------------------------------------------------
  bool debug;            // Diagnostic-output toggle.
  bool output_stat;       // Enables periodic system-statistics output.
  bool output_stat_v;      // Adds the first level of statistics detail.
  bool output_stat_vv;     // Adds the second level of statistics detail.
  bool serial_command;     // Enables processing of incoming serial commands.
  bool logging_enabled;    // Enables writing log entries to storage.

  // ------------------------------------------------------------------------
  // Per-sentence/per-subsystem output-enable flags: each true value
  // permits its corresponding output to be sent this loop iteration.
  // ------------------------------------------------------------------------
  bool output_satio_all;   // Mirrors the most recent "enable all" bulk toggle.
  bool output_satio_enabled;
  bool output_gngga_enabled;
  bool output_gnrmc_enabled;
  bool output_gpatt_enabled;
  bool output_ins_enabled;
  bool output_matrix_enabled;
  bool output_input_portcontroller;
  bool output_config_matrix_enabled;
  bool output_config_mapping_enabled;
  bool output_admplex0_enabled;
  bool output_gyro_0_enabled;
  bool output_sun_enabled;
  bool output_mercury_enabled;
  bool output_venus_enabled;
  bool output_earth_enabled;
  bool output_luna_enabled;
  bool output_mars_enabled;
  bool output_jupiter_enabled;
  bool output_saturn_enabled;
  bool output_uranus_enabled;
  bool output_neptune_enabled;
  bool output_meteors_enabled;

  // Seconds elapsed since startup, or since the last reset of uptime_seconds.
  int32_t uptime_seconds;

  // ------------------------------------------------------------------------
  // Loop counters: each is incremented as its named operation runs. Most
  // are sampled into the matching total_* field below once per second,
  // then reset to zero there.
  // ------------------------------------------------------------------------
  int32_t i_count_read_gps;
  int32_t i_task_freq_hz_gps;

  int32_t i_count_read_ins;
  int32_t i_count_read_gyro_0;
  int32_t i_task_freq_hz_gyro;
  int32_t i_count_read_mplex_0;
  int32_t i_task_freq_hz_mlx;
  int32_t i_count_matrix;
  int32_t i_task_freq_hz_switches;
  int32_t i_count_portcontroller_input;
  int32_t i_count_port_controller_output;
  int32_t i_count_track_planets;
  int32_t i_task_freq_hz_uni;
  int32_t i_count_read_serial_commands;
  int32_t i_count_logging;
  int32_t i_count_display;
  int32_t i_task_freq_hz_dsp;

  int32_t loops_a_second;       // Main loop iterations since the last 1-second sample.
  int32_t total_loops_a_second; // Main loop iterations per second, at the last sample.

  // ------------------------------------------------------------------------
  // Per-second totals: each holds the matching i_count_* counter above, as
  // sampled and reset once per second, i.e. that operation's rate per
  // second.
  // ------------------------------------------------------------------------
  int32_t total_gps;
  int32_t total_task_freq_hz_gps;

  int32_t total_ins;
  int32_t total_gyro_0;
  int32_t total_task_freq_hz_gyro;
  int32_t total_mplex_0;
  int32_t total_task_freq_hz_mlx;
  int32_t total_matrix;
  int32_t total_task_freq_hz_switches;
  int32_t total_universe;
  int32_t total_task_freq_hz_uni;
  int32_t total_task_freq_hz_dsp;
  int32_t total_infocmd;
  int32_t total_portcontroller_output;
  int32_t total_portcontroller_input;
  int32_t total_display;
};
extern struct systemStruct systemData;

/**
 * Restores the system to default configuration values: clears every
 * output flag and the debug flag, then restores SATIO time-sync and INS
 * threshold/mode settings to their startup values.
 */
void restore_system_defaults(void);

#endif /* SYSTEM_DATA_H */
/*
    System Data. Written by Benjamin Jack Cullen.

    Defines the single systemData instance declared in system_data.h, and
    restore_system_defaults(), which clears every output flag and restores
    SATIO/INS settings to their startup values.

    MISRA-relevant conventions used throughout this module:
    - systemData has exactly one definition, here, matching the single
      extern declaration in system_data.h (Rule 8.5).
    - restore_system_defaults() assigns insData.INS_REQ_HEADING_RANGE_DIFF
      a double literal (1.0) rather than an int literal, matching that
      field's declared type, so no implicit int-to-double conversion
      occurs (Rule 10.3/10.4).
    - restore_system_defaults() has a single point of exit (Rule 15.5).

    Intended to be MISRA Compliant (untested, unverified, in-progress).
*/

#include "system_data.h"
#include <Arduino.h>
#include "satio.h"
#include "serial_infocmd.h"
#include "ins.h"

/**
 * @struct systemStruct
 * @brief Single global instance of the system-wide flags, loop counters,
 *        and per-second totals declared in system_data.h.
 */
struct systemStruct systemData = {
  // Output interval-breach flags.
  .interval_breach_gps_output = false,
  .interval_breach_ins_output = false,
  .interval_breach_gyro_0_output = false,
  .interval_breach_mplex_0_output = false,
  .interval_breach_matrix_output = false,
  .interval_breach_track_planets_output = false,
  .interval_breach_logging = false,
  .interval_breach_1_second_output = false,

  // Compute interval-breach flags.
  .interval_breach_track_planets = false,
  .interval_breach_star_navigation = false,

  // Diagnostics and command processing.
  .debug = false,
  .output_stat = false,
  .output_stat_v = false,
  .output_stat_vv = false,
  .serial_command = true,
  .logging_enabled=false,

  // Per-sentence/per-subsystem output-enable flags.
  .output_satio_all = false,
  .output_satio_enabled = false,
  .output_gngga_enabled = false,
  .output_gnrmc_enabled = false,
  .output_gpatt_enabled = false,
  .output_ins_enabled = false,
  .output_matrix_enabled = false,
  .output_input_portcontroller = false,
  .output_config_matrix_enabled = false,
  .output_admplex0_enabled = false,
  .output_gyro_0_enabled = false,
  .output_sun_enabled = false,
  .output_mercury_enabled = false,
  .output_venus_enabled = false,
  .output_earth_enabled = false,
  .output_luna_enabled = false,
  .output_mars_enabled = false,
  .output_jupiter_enabled = false,
  .output_saturn_enabled = false,
  .output_uranus_enabled = false, 
  .output_neptune_enabled = false,
  .output_meteors_enabled = false,

  // Uptime and loop counters.
  .uptime_seconds = 0,

  .i_count_read_gps = 0,
  .i_task_freq_hz_gps = 0,

  .i_count_read_ins = 0,
  .i_count_read_gyro_0 = 0,
  .i_task_freq_hz_gyro = 0,
  .i_count_read_mplex_0 = 0,
  .i_task_freq_hz_mlx = 0,
  .i_count_matrix = 0,
  .i_task_freq_hz_switches = 0,
  .i_count_portcontroller_input = 0,
  .i_count_port_controller_output = 0,
  .i_count_track_planets = 0,
  .i_task_freq_hz_uni = 0,
  .i_count_read_serial_commands = 0,
  .i_count_logging = 0,
  .i_count_display = 0,
  .i_task_freq_hz_dsp = 0,
  
  .total_system_time = 0,
  .i_count_system_time = 0,

  .loops_a_second = 0,
  .total_loops_a_second = 0,

  // Per-second totals.
  .total_gps = 0,
  .total_task_freq_hz_gps = 0,
  
  .total_ins = 0,
  .total_gyro_0 = 0,
  .total_task_freq_hz_gyro = 0,
  .total_mplex_0 = 0,
  .total_task_freq_hz_mlx = 0,
  .total_matrix = 0,
  .total_task_freq_hz_switches = 0,
  .total_universe = 0,
  .total_task_freq_hz_uni = 0,
  .total_task_freq_hz_dsp = 0,
  .total_infocmd = 0,
  .total_portcontroller_output = 0,
  .total_portcontroller_input = 0,
  .total_display = 0,
};

void restore_system_defaults(void) {
  // Clear the debug flag and every per-sentence/per-subsystem output flag.
  systemData.debug = false;
  systemData.output_satio_all = false;
  systemData.output_satio_enabled = false; 
  systemData.output_gngga_enabled = false;
  systemData.output_gnrmc_enabled = false;
  systemData.output_gpatt_enabled = false;
  systemData.output_ins_enabled = false;
  systemData.output_matrix_enabled = false;
  systemData.output_input_portcontroller = false;
  systemData.output_config_matrix_enabled = false;
  systemData.output_config_mapping_enabled = false;
  systemData.output_admplex0_enabled = false;
  systemData.output_gyro_0_enabled = false;
  systemData.output_sun_enabled = false;
  systemData.output_luna_enabled = false;
  systemData.output_mercury_enabled = false;
  systemData.output_venus_enabled = false;
  systemData.output_mars_enabled = false;
  systemData.output_jupiter_enabled = false;
  systemData.output_saturn_enabled = false;
  systemData.output_uranus_enabled = false;
  systemData.output_neptune_enabled = false;
  systemData.output_meteors_enabled = false;

  // Restore SATIO time-sync settings to their startup values.
  satioData.utc_second_offset = 0;
  satioData.utc_auto_offset_flag = false;
  satioData.set_time_automatically = true;
  satioData.sync_rtc_immediately_flag = true;

  // Restore INS thresholds and mode to their startup values.
  insData.INS_REQ_GPS_PRECISION = DEFAULT_INS_REQ_GPS_PRECISION;
  insData.INS_REQ_MIN_SPEED = DEFAULT_INS_REQ_MIN_SPEED;
  insData.INS_REQ_HEADING_RANGE_DIFF = DEFAULT_INS_REQ_HEADING_RANGE_DIFF;
  insData.INS_USE_GYRO_HEADING = DEFAULT_INS_USE_GYRO_HEADING;
  insData.INS_MODE = DEFAULT_INS_MODE;
  insData.INS_FORCED_ON_FLAG = DEFAULT_INS_FORCED_ON_FLAG;

  Serial.println("[restore_system_defaults] done.");
}
/*
    INS Library. Written by Benjamin Jack Cullen.

    Estimate location using gyro data and or dead reckoning.
*/

#include "./ins.h"
#include <Arduino.h>
#include "satio.h"

/**
 * @struct InsData
 */
struct InsData insData = {
  .INS_ENABLED = true,
  .INS_REQ_GPS_PRECISION = 0.5,
  .INS_REQ_MIN_SPEED = 0.1,
  .INS_REQ_HEADING_RANGE_DIFF=1,
  .INS_USE_GYRO_HEADING = true,
  .INS_FORCED_ON_FLAG = false,
  .INS_MODE = INS_MODE_DYNAMIC,
  .char_ins_mode = {"INS OFF", "INS DYNAMIC", "INS FORCED ON"},
  .INS_INITIALIZATION_FLAG = INS_INITIALIZATION_FLAG_0,
  .ins_latitude = 0.0,
  .ins_longitude = 0.0,
  .ins_altitude = 0.0,
  .ins_heading = 0.0,
  .ins_dt_prev = 0,
};

/**
 * Reports whether angle1 and angle2 (degrees) are within range of each
 * other, accounting for wraparound at the 0/360 boundary (e.g. 1 and 359
 * are 2 degrees apart, not 358).
 *
 * Rule 10.x: fmod()/fabs() (not fmodf()/fabsf()) and double literals are
 * used throughout, matching this function's double parameters; mixing in
 * the float-precision forms here would silently narrow and re-widen every
 * value, losing precision for no reason.
 * Rule 15.6: braces are used on every if/while body, including the
 * single-statement angle normalization below.
 */
bool angles_are_close(double angle1,
                      double angle2,
                      double range) {
  // -------------------------------------------------------------------------------
  // Normalize angles to [0, 360).
  // -------------------------------------------------------------------------------
  angle1 = fmod(angle1, 360.0);
  angle2 = fmod(angle2, 360.0);
  if (angle1 < 0.0) {angle1 += 360.0;}
  if (angle2 < 0.0) {angle2 += 360.0;}
  // -------------------------------------------------------------------------------
  // Calculate the absolute difference.
  // -------------------------------------------------------------------------------
  double diff = fabs(angle1 - angle2);
  // -------------------------------------------------------------------------------
  // Consider wraparound (e.g., 0 and 360 are close).
  // -------------------------------------------------------------------------------
  if (diff > 180.0) {diff = 360.0 - diff;}
  // -------------------------------------------------------------------------------
  // Check if difference is within the specified range.
  // -------------------------------------------------------------------------------
  return diff <= range;
}

/**
 * Re-evaluates the INS initialization level from scratch: starts at 0 and
 * adds 1 for each of the 4 independent readiness conditions currently
 * met (GPS precision, speed, heading agreement, master enable), so the
 * result reaches MAX_INS_INITIALIZATION_FLAG only when every condition
 * holds simultaneously.
 */
void ins_init(double gps_precision_factor,
              double ground_heading,
              double ground_speed,
              double gyro_heading) {
  // -------------------------------------------------------------------------------
  // 0 : Default.
  // -------------------------------------------------------------------------------
  insData.tmp_ins_initialization_flag=0;
  // -------------------------------------------------------------------------------
  // 1 : Check GPS precision.
  // -------------------------------------------------------------------------------
  if (gps_precision_factor<=insData.INS_REQ_GPS_PRECISION) {
    insData.tmp_ins_initialization_flag++;
  }
  // -----------------------------------------------------------------------------
  // 2 : Check Speed.
  // -----------------------------------------------------------------------------
  if (ground_speed>=insData.INS_REQ_MIN_SPEED) {
    insData.tmp_ins_initialization_flag++;
  }
  // ---------------------------------------------------------------------------
  // 3 : Check Heading.
  // ---------------------------------------------------------------------------
  if ((angles_are_close(ground_heading,
      gyro_heading,
      insData.INS_REQ_HEADING_RANGE_DIFF)==true) ||
      (insData.INS_USE_GYRO_HEADING==false)) {
    insData.tmp_ins_initialization_flag++;
  }
  // -------------------------------------------------------------------------
  // 4 : Check enabled.
  // -------------------------------------------------------------------------
  if (insData.INS_ENABLED==true) {
    insData.tmp_ins_initialization_flag++;
  }
  // -------------------------------------------------------------------------
  // Set INS initialization
  // -------------------------------------------------------------------------
  insData.INS_INITIALIZATION_FLAG=insData.tmp_ins_initialization_flag;
}

/**
 * Rule 8.7: internal linkage; only called from set_ins() in this file.
 */
static void set_ins_data_as_gps(double gps_latitude,
                         double gps_longitude,
                         double gps_altitude,
                         double ground_heading) {
  insData.ins_latitude=gps_latitude;
  insData.ins_longitude=gps_longitude;
  insData.ins_altitude=gps_altitude;
  insData.ins_heading=ground_heading;
}

/**
 * Sets INS data and INS_ENABLED according to the current INS_MODE: OFF and
 * DYNAMIC both take GPS data directly every call (OFF leaves INS_ENABLED
 * false so ins_estimate_position() never runs; DYNAMIC leaves it true so
 * estimation fills the gaps between GPS updates). HOLD_THE_LINE takes GPS
 * data once, on the first call after initialization completes, then
 * leaves INS_FORCED_ON_FLAG set so it is never overwritten by GPS again
 * until that flag is cleared elsewhere.
 */
void set_ins(double gps_latitude,
             double gps_longitude,
             double gps_altitude,
             double ground_heading,
             double ground_speed,
             double gps_precision_factor,
             double gyro_heading) {
  // -------------------------------------------------------------------------------
  // Off.
  // -------------------------------------------------------------------------------
  if (insData.INS_MODE==INS_MODE_OFF) {
    insData.INS_ENABLED=false;
    set_ins_data_as_gps(gps_latitude, gps_longitude, gps_altitude, ground_heading);
    ins_init(gps_precision_factor, ground_heading, ground_speed, gyro_heading);
  }
  // -------------------------------------------------------------------------------
  // Dynamic.
  // -------------------------------------------------------------------------------
  else if (insData.INS_MODE==INS_MODE_DYNAMIC) {
    insData.INS_ENABLED=true;
    set_ins_data_as_gps(gps_latitude, gps_longitude, gps_altitude, ground_heading);
    ins_init(gps_precision_factor, ground_heading, ground_speed, gyro_heading);
  }
  // -------------------------------------------------------------------------------
  // Forced (Hold the Line).
  // -------------------------------------------------------------------------------
  else if (insData.INS_MODE==INS_MODE_HOLD_THE_LINE) {
    insData.INS_ENABLED=true;
    // Check initialization flag because INS_FORCED_ON_FLAG overrides.
    if (insData.INS_INITIALIZATION_FLAG==MAX_INS_INITIALIZATION_FLAG &&
        insData.INS_FORCED_ON_FLAG==false) {
      ins_init(gps_precision_factor, ground_heading, ground_speed, gyro_heading);
      // Continue once or try again next time.
      if (insData.INS_FORCED_ON_FLAG==false) {
        set_ins_data_as_gps(gps_latitude, gps_longitude, gps_altitude, ground_heading);
        insData.INS_FORCED_ON_FLAG=true;
      }
    }
  }
}

/**
 * Dead-reckons forward from the last known INS position using gyro pitch
 * and yaw plus GPS ground speed, over the elapsed time since the previous
 * call (insData.ins_dt_prev). Only estimates while the INS is fully
 * initialized or held forced-on; otherwise leaves insData untouched and
 * reports that no estimate was produced.
 *
 * @return true if insData was updated with a new estimate, false if the
 *         INS was not ready and nothing changed.
 *
 * Rule 8.9: temp_lat/temp_lon are declared here at block scope rather than
 * at file scope, since they are scratch values used only within this
 * function.
 * Rule 15.5: single point of exit via a result variable.
 * Rule 15.6: braces are used on every if/while body, including the
 * single-statement angle/latitude normalization loops below.
 */
bool ins_estimate_position(double pitch,
                           double yaw,
                           double ground_heading,
                           double ground_speed,
                           int64_t dt) {
  bool position_updated = false;

  if (insData.INS_INITIALIZATION_FLAG==MAX_INS_INITIALIZATION_FLAG ||
      insData.INS_FORCED_ON_FLAG==true) {
    double temp_lat;
    double temp_lon;
    // -------------------------------------------------------------------------------
    // Calculate time interval in seconds from microseconds dt.
    // -------------------------------------------------------------------------------
    double dt_interval = (double)(dt - insData.ins_dt_prev) / 1000000.0;
    // -------------------------------------------------------------------------------
    // Ensure positive time interval; fallback to 0.001s if invalid.
    // -------------------------------------------------------------------------------
    if (dt_interval <= 0.0) {dt_interval = 0.001;}
    // -------------------------------------------------------------------------------
    // Normalize yaw to [0, 360°).
    // -------------------------------------------------------------------------------
    while (yaw >= 360.0) {yaw -= 360.0;}
    while (yaw < 0.0) {yaw += 360.0;}
    // -------------------------------------------------------------------------------
    // Convert angles to radians.
    // -------------------------------------------------------------------------------
    double yaw_rad = yaw * DEG_TO_RAD;
    double pitch_rad = pitch * DEG_TO_RAD;
    // -------------------------------------------------------------------------------
    // Horizontal and vertical speed components.
    // -------------------------------------------------------------------------------
    double v_horizontal = ground_speed * cos(pitch_rad);
    double v_vertical = ground_speed * sin(pitch_rad);
    // -------------------------------------------------------------------------------
    // Calculate horizontal distance traveled.
    // -------------------------------------------------------------------------------
    double distance_horizontal = v_horizontal * dt_interval;
    // -------------------------------------------------------------------------------
    // North-South and East-West displacements.
    // -------------------------------------------------------------------------------
    double delta_y = distance_horizontal * cos(yaw_rad); // North-South (m)
    double delta_x = distance_horizontal * sin(yaw_rad); // East-West (m)
    // -------------------------------------------------------------------------------
    // Altitude change.
    // -------------------------------------------------------------------------------
    double delta_alt = v_vertical * dt_interval;
    // -------------------------------------------------------------------------------
    // Convert initial latitude to radians.
    // -------------------------------------------------------------------------------
    double lat0_rad = insData.ins_latitude * DEG_TO_RAD;
    // -------------------------------------------------------------------------------
    // Latitude change: Δφ = Δy / R.
    // -------------------------------------------------------------------------------
    double delta_lat = (delta_y / EARTH_MEAN_RADIUS) * RAD_TO_DEG;
    // -------------------------------------------------------------------------------
    // Longitude change: Δλ = Δx / (R * cos(φ₀)).
    // -------------------------------------------------------------------------------
    double delta_lon = (delta_x / (EARTH_MEAN_RADIUS * cos(lat0_rad))) * RAD_TO_DEG;
    // -------------------------------------------------------------------------------
    // normalize and update lat & long.
    // -------------------------------------------------------------------------------
    temp_lat = insData.ins_latitude + delta_lat;
    temp_lon = insData.ins_longitude + delta_lon;
    // -------------------------------------------------------------------------------
    // Normalize latitude to [-90°, 90°], reflecting at poles.
    // -------------------------------------------------------------------------------
    while (temp_lat > 90.0) {temp_lat = 180.0 - temp_lat;}
    while (temp_lat < -90.0) {temp_lat = -180.0 - temp_lat;}
    // -------------------------------------------------------------------------------
    // Normalize longitude to [-180°, 180°].
    // -------------------------------------------------------------------------------
    temp_lon = fmod(temp_lon + 180.0, 360.0) - 180.0;
    // -------------------------------------------------------------------------------
    // Update INS data with normalized values.
    // -------------------------------------------------------------------------------
    insData.ins_latitude = temp_lat;
    insData.ins_longitude = temp_lon;
    // -------------------------------------------------------------------------------
    // update other INS data.
    // -------------------------------------------------------------------------------
    insData.ins_altitude = insData.ins_altitude + delta_alt;
    if (insData.INS_USE_GYRO_HEADING==true) {insData.ins_heading=yaw;}
    else {insData.ins_heading=ground_heading;}
    // -------------------------------------------------------------------------------
    // Ensure previous datetime is set to current datetime.
    // -------------------------------------------------------------------------------
    insData.ins_dt_prev=dt;
    position_updated = true;
  }

  return position_updated;
}

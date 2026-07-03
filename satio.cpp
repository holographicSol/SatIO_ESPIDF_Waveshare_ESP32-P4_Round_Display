/*
    SATIO Library. Written by Benjamin Jack Cullen.

    Intended to be MISRA Compliant (untested, unverified, in-progress).
*/

#include "./satio.h"
#include <Arduino.h>
#include <esp_timer.h>
#include <time.h>
#include <Wire.h>
#include "wtgps300p.h"
#include "./task_handler.h"
#include "freertos/semphr.h"
#include <SiderealPlanets.h>
#include <SiderealObjects.h>
#include "./sidereal_helper.h"

// ------------------------------------------------------------------------------------
// Global Time
// ------------------------------------------------------------------------------------
struct tm *timeinfo;
struct timeval tv_now;
// ------------------------------------------------------------------------------------


SemaphoreHandle_t systemTimeMutex = nullptr;

void initSystemTimeMutex(void) {
  systemTimeMutex = xSemaphoreCreateMutex();
}

SemaphoreHandle_t dataMutex = nullptr;

void initDataMutex(void) {
  dataMutex = xSemaphoreCreateMutex();
}


struct SATIOStruct satioData = {
    // ------------------------------------------------------------------------------------
    // INTERNAL
    // ------------------------------------------------------------------------------------
    .satio_sentence = {0},
    .latitude_meter = 0.0000100,
    .longitude_meter = 0.0000100,
    .latitude_mile = 0.0000100 * 1609.34,
    .longitude_mile = 0.0000100 * 1609.34,
    .abs_latitude_gngga_0 = 0.0,
    .abs_longitude_gngga_0 = 0.0,
    .abs_latitude_gnrmc_0 = 0.0,
    .abs_longitude_gnrmc_0 = 0.0,
    .temp_latitude_gngga = 0.0,
    .temp_longitude_gngga = 0.0,
    .temp_latitude_gnrmc = 0.0,
    .temp_longitude_gnrmc = 0.0,
    .minutesLat = 0.0,
    .minutesLong = 0.0,
    .secondsLat = 0.0,
    .secondsLong = 0.0,
    .millisecondsLat = 0.0,
    .millisecondsLong = 0.0,
    .degreesLat = 0.0,
    .degreesLong = 0.0,
    .tmp_year_int = 0,
    .tmp_month_int = 0,
    .tmp_day_int = 0,
    .tmp_hour_int = 0,
    .tmp_minute_int = 0,
    .tmp_second_int = 0,
    .tmp_millisecond_int = 0,
    .tmp_year = {0},
    .tmp_month = {0},
    .tmp_day = {0},
    .tmp_hour = {0},
    .tmp_minute = {0},
    .tmp_second = {0},
    .tmp_millisecond = {0},
    .week_day_names = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"},
    .month_names = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"},
    .abbrev_month_names = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"},

    // ------------------------------------------------------------------------------------
    // LOCATION
    // ------------------------------------------------------------------------------------
    .degrees_latitude=0.0,                            // Converted latitude in degrees
    .degrees_longitude=0.0,                           // Converted longitude in degrees
    .user_degrees_latitude=0.0,
    .user_degrees_longitude=0.0,
    .system_degrees_latitude=0.0,
    .system_degrees_longitude=0.0,
    .location_value_mode = SATIO_MODE_GPS,
    // ------------------------------------------------------------------------------------
    // ALTITUDE SETTINGS
    // ------------------------------------------------------------------------------------
    .altitude = 0.0,
    .user_altitude = 0.0,
    .system_altitude=0.0,
    .altitude_value_mode = SATIO_MODE_GPS,
    // ------------------------------------------------------------------------------------
    // SPEED SETTINGS
    // ------------------------------------------------------------------------------------
    .speed = 0.0,
    .user_speed = 0.0,
    .system_speed = 0.0,
    .speed_value_mode = SATIO_MODE_GPS,
    // ------------------------------------------------------------------------------------
    // HEADING SETTINGS
    // ------------------------------------------------------------------------------------
    .ground_heading = 0.0,
    .user_ground_heading = 0.0,
    .system_ground_heading = 0.0,
    .ground_heading_value_mode = SATIO_MODE_GPS,
    .ground_heading_name = {0},
    .course_heading = 0.0,
    // ------------------------------------------------------------------------------------
    // MILEAGE
    // ------------------------------------------------------------------------------------
    .mileage = "pending",

    // ------------------------------------------------------------------------------------
    // Date Time
    // ------------------------------------------------------------------------------------
    .GPSTime = {},
    .systemTime = {},
    .localTime = {},
    .localMeanSolarTime = {},
};

LocPoint loc_point1_gps = {0.0, 0.0, 0.0, 0};
LocPoint loc_point2_gps = {0.0, 0.0, 0.0, 0};
LocPoint loc_point1_ins = {0.0, 0.0, 0.0, 0};
LocPoint loc_point2_ins = {0.0, 0.0, 0.0, 0};

struct SpeedStruct speedData = {
    .lat1_rad = 0.0,
    .lon1_rad = 0.0,
    .lat2_rad = 0.0,
    .lon2_rad = 0.0,
    .delta_lat = 0.0,
    .delta_lon = 0.0,
    .delta_alt = 0.0,
    .a = 0.0,
    .c = 0.0,
    .distance_2d = 0.0,
    .distance_3d = 0.0,
    .delta_time = 0.0,
    .speed = 0.0
};

/**
 * Set SatIO Altitude According To Update Mode. (The following should either be set or not set. If not set then conditions the be checked elsewhere)
 *
 * Rule 8.7: internal linkage; only called from setSatIOData() in this file.
 */
void setSatIOAltitude(void) {
  satioData.altitude = atof(gnggaData.altitude);
  // ---------------------------------------------------------------------
  // Select which value to use from the system.
  // ---------------------------------------------------------------------
  if      (satioData.altitude_value_mode==SATIO_MODE_GPS)  {satioData.system_altitude = satioData.altitude;}
  else if (satioData.altitude_value_mode==SATIO_MODE_USER) {satioData.system_altitude = satioData.user_altitude;}
}

/**
 * Set SatIO Speed According To Update Mode. (The following should either be set or not set. If not set then conditions the be checked elsewhere)
 *
 * Rule 8.7: internal linkage; only called from setSatIOData() in this file.
 */
void setSatIOSpeed(void) {
  satioData.speed = atof(gnrmcData.ground_speed);
  // ---------------------------------------------------------------------
  // Select which value to use from the system.
  // ---------------------------------------------------------------------
  if      (satioData.speed_value_mode==SATIO_MODE_GPS)  {satioData.system_speed = satioData.speed;}
  else if (satioData.speed_value_mode==SATIO_MODE_USER) {satioData.system_speed = satioData.user_speed;}
}

/**
 * Set SatIO Ground Heading According To Update Mode. (The following should either be set or not set. If not set then conditions the be checked elsewhere)
 *
 * Rule 8.7: internal linkage; only called from setSatIOData() in this file.
 */
void setSatIOGroundHeading(void) {
  satioData.ground_heading = atof(gnrmcData.ground_heading);
  // ---------------------------------------------------------------------
  // Select which value to use from the system.
  // ---------------------------------------------------------------------
  if      (satioData.ground_heading_value_mode==SATIO_MODE_GPS)  {satioData.system_ground_heading = satioData.ground_heading;}
  else if (satioData.ground_heading_value_mode==SATIO_MODE_USER) {satioData.system_ground_heading = satioData.user_ground_heading;}
}

// ----------------------------------------------------------------------------------------
// groundHeadingDegreesToNESW.
// ----------------------------------------------------------------------------------------
/* Rule 8.7: internal linkage; only called from setGroundHeadingName() in
   this file. Rule 15.5: single point of exit via a result variable instead
   of one return per compass direction. */
static String groundHeadingDegreesToNESW(float num) {
  String direction;

  if (num == 0 || num == 360)      {direction = "N";}
  else if (num > 0 && num < 45)    {direction = "NNE";}
  else if (num == 45)              {direction = "NE";}
  else if (num > 45 && num < 90)   {direction = "ENE";}
  else if (num == 90)              {direction = "E";}
  else if (num > 90 && num < 135)  {direction = "ESE";}
  else if (num == 135)             {direction = "SE";}
  else if (num > 135 && num < 180) {direction = "SSE";}
  else if (num == 180)             {direction = "S";}
  else if (num > 180 && num < 225) {direction = "SSW";}
  else if (num == 225)             {direction = "SW";}
  else if (num > 225 && num < 270) {direction = "WSW";}
  else if (num == 270)             {direction = "W";}
  else if (num > 270 && num < 315) {direction = "WNW";}
  else if (num == 315)             {direction = "NW";}
  else if (num > 315 && num < 360) {direction = "NNW";}
  else                              {direction = "";}

  return direction;
}
void setGroundHeadingName(float num) {
  /* Rule 21.x: bounded replacement for strcpy(); String::c_str() can never
     exceed the longest literal above, but strncpy keeps the destination
     write provably within ground_heading_name's bounds. */
  String direction = groundHeadingDegreesToNESW(num);

  memset(satioData.ground_heading_name, 0, sizeof(satioData.ground_heading_name));
  strncpy(satioData.ground_heading_name, direction.c_str(), sizeof(satioData.ground_heading_name) - 1U);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                         CONVERT COORDINTE DATA
// ------------------------------------------------------------------------------------------------------------------------------
/* Rule 8.7: internal linkage; only called from setSatIOData() in this file. */
void setSatioCoordinates(void) {
  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                  GNGGA COORDINATE CONVERSION
  // ----------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------
  // Convert GNGGA latitude & longitude strings to decimal degrees and format into hours, minutes, seconds, milliseconds.
  // ----------------------------------------------------------------------------------------------------------------------------
  // -----------------------------------------------------------------------------------------
  // Extract absolute latitude value from GNGGA data as decimal degrees.
  // -----------------------------------------------------------------------------------------
  satioData.abs_latitude_gngga_0=atof(String(gnggaData.latitude).c_str());
  // -----------------------------------------------------------------------------------------
  // Store absolute latitude in temporary variable for further processing.
  // -----------------------------------------------------------------------------------------
  satioData.temp_latitude_gngga=satioData.abs_latitude_gngga_0;
  // -----------------------------------------------------------------------------------------
  // Separate the integer degrees value from the fractional part.
  // -----------------------------------------------------------------------------------------
  satioData.degreesLat=trunc(satioData.temp_latitude_gngga / 100);
  // -----------------------------------------------------------------------------------------
  // Calculate minutes and seconds values based on remaining fractional part.
  // -----------------------------------------------------------------------------------------
  satioData.minutesLat=satioData.temp_latitude_gngga - (satioData.degreesLat * 100);
  // -----------------------------------------------------------------------------------------
  // Convert excess fractional part to seconds.
  // -----------------------------------------------------------------------------------------
  satioData.secondsLat=(satioData.minutesLat - trunc(satioData.minutesLat)) * 60;
  // -----------------------------------------------------------------------------------------
  // Convert excess seconds to milliseconds.
  // -----------------------------------------------------------------------------------------
  satioData.millisecondsLat=(satioData.secondsLat - trunc(satioData.secondsLat)) * 1000;
  // -----------------------------------------------------------------------------------------
  // Round off minutes and seconds values to nearest integer.
  // -----------------------------------------------------------------------------------------
  satioData.minutesLat=trunc(satioData.minutesLat);
  satioData.secondsLat=trunc(satioData.secondsLat);
  // -----------------------------------------------------------------------------------------
  // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
  // -----------------------------------------------------------------------------------------
  satioData.degrees_latitude =
  satioData.degreesLat + satioData.minutesLat / 60 + satioData.secondsLat / 3600 + satioData.millisecondsLat / 3600000;
  // -----------------------------------------------------------------------------------------
  // Negate latitude value if it's in the Southern hemisphere (make negative value).
  // -----------------------------------------------------------------------------------------
  if (strcmp(gnggaData.latitude_hemisphere, "S")==0) {
    satioData.degrees_latitude=0 - satioData.degrees_latitude;
  }
  // -----------------------------------------------------------------------------------------
  // Extract absolute longitude value from GNGGA data as decimal degrees.
  // -----------------------------------------------------------------------------------------
  satioData.abs_longitude_gngga_0=atof(String(gnggaData.longitude).c_str());
  // -----------------------------------------------------------------------------------------
  // Store absolute latitude in temporary variable for further processing.
  // -----------------------------------------------------------------------------------------
  satioData.temp_longitude_gngga=satioData.abs_longitude_gngga_0;
  // -----------------------------------------------------------------------------------------
  // Separate the integer degrees value from the fractional part.
  // -----------------------------------------------------------------------------------------
  satioData.degreesLong=trunc(satioData.temp_longitude_gngga / 100);
  // -----------------------------------------------------------------------------------------
  // Calculate minutes and seconds values based on remaining fractional part.
  // -----------------------------------------------------------------------------------------
  satioData.minutesLong=satioData.temp_longitude_gngga - (satioData.degreesLong * 100);
  // -----------------------------------------------------------------------------------------
  // Convert excess fractional part to seconds.
  // -----------------------------------------------------------------------------------------
  satioData.secondsLong=(satioData.minutesLong - trunc(satioData.minutesLong)) * 60;
  // -----------------------------------------------------------------------------------------
  // Convert excess seconds to milliseconds.
  // -----------------------------------------------------------------------------------------
  satioData.millisecondsLong=(satioData.secondsLong - trunc(satioData.secondsLong)) * 1000;
  // -----------------------------------------------------------------------------------------
  // Round off minutes and seconds values to nearest integer.
  // -----------------------------------------------------------------------------------------
  satioData.minutesLong=trunc(satioData.minutesLong);
  satioData.secondsLong=trunc(satioData.secondsLong);
  // -----------------------------------------------------------------------------------------
  // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
  // -----------------------------------------------------------------------------------------
  satioData.degrees_longitude =
  satioData.degreesLong + satioData.minutesLong / 60 + satioData.secondsLong / 3600 + satioData.millisecondsLong / 3600000;
  // -----------------------------------------------------------------------------------------
  // Negate latitude value if it's in the Southern hemisphere (make negative value).
  // -----------------------------------------------------------------------------------------
  if (strcmp(gnggaData.longitude_hemisphere, "W")==0) {
    satioData.degrees_longitude=0 - satioData.degrees_longitude;
  }
  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                     USER DEFINED COORDINATES
  // ----------------------------------------------------------------------------------------------------------------------------
  if (satioData.location_value_mode==SATIO_MODE_GPS)  {
    satioData.system_degrees_latitude = satioData.degrees_latitude;
    satioData.system_degrees_longitude = satioData.degrees_longitude;
  }
  else if (satioData.location_value_mode==SATIO_MODE_USER) {
    satioData.system_degrees_latitude = satioData.user_degrees_latitude;
    satioData.system_degrees_longitude = satioData.user_degrees_longitude;

  }
}

 double calculateSpeedFromLocationData(LocPoint p1, LocPoint p2) {
    // -------------------------------------------------------------------------------
    // Convert latitude and longitude from degrees to radians for calculations.
    // -------------------------------------------------------------------------------
    double lat1_rad = p1.latitude * M_PI / 180.0;
    double lon1_rad = p1.longitude * M_PI / 180.0;
    double lat2_rad = p2.latitude * M_PI / 180.0;
    double lon2_rad = p2.longitude * M_PI / 180.0;
    // -------------------------------------------------------------------------------
    // Calculate the change in coordinates.
    // -------------------------------------------------------------------------------
    double delta_lat = lat2_rad - lat1_rad;
    double delta_lon = lon2_rad - lon1_rad;
    // -------------------------------------------------------------------------------
    // Calculate the change in altitude.
    // -------------------------------------------------------------------------------
    double delta_alt = p2.altitude - p1.altitude;
    // -------------------------------------------------------------------------------
    // Haversine formula to calculate the 2D distance.
    // -------------------------------------------------------------------------------
    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    // -------------------------------------------------------------------------------
    // Calculate the 2D distance (great-circle distance).
    // -------------------------------------------------------------------------------
    double distance_2d = EARTH_MEAN_RADIUS * c;
    // -------------------------------------------------------------------------------
    // Calculate the total 3D distance using the altitude change.
    // -------------------------------------------------------------------------------
    double distance_3d = sqrt(distance_2d * distance_2d + delta_alt * delta_alt);
    // -------------------------------------------------------------------------------
    // Calculate the change in time in seconds.
    // -------------------------------------------------------------------------------
    double delta_time = (p2.time - p1.time) / 1000000.0;
    // -------------------------------------------------------------------------------
    // Handle the case of zero time difference to avoid division by zero.
    // -------------------------------------------------------------------------------
    double speed=0;
    if (delta_time == 0.0) {speed=0.0;}
    // -------------------------------------------------------------------------------
    // The result is in meters per second, as distance is in meters and time is in seconds.
    // -------------------------------------------------------------------------------
    else {speed = distance_3d / delta_time;}
    return speed;
}

// ----------------------------------------------------------------------------------------
// Shared time/date formatting (Rule 8.7: internal linkage; storeTimeFields()
// needs the same colon/slash-separated and compact-padded strings built for
// every SatIOTimeData instance, so the snprintf() calls live here once
// instead of being repeated per clock domain). "%02d"/"%04d" zero-pad
// directly, so no separate digit-padding helper is needed.
// ----------------------------------------------------------------------------------------
static void formatTimeStrings(uint8_t hour, uint8_t minute, uint8_t second,
                               char *formatted_time, size_t formatted_time_size,
                               char *padded_time, size_t padded_time_size) {
    snprintf(formatted_time, formatted_time_size, "%02d:%02d:%02d", hour, minute, second);
    snprintf(padded_time, padded_time_size, "%02d%02d%02d", hour, minute, second);
}

static void formatDateStrings(uint8_t day, uint8_t month, uint16_t year,
                               char *formatted_date, size_t formatted_date_size,
                               char *padded_date, size_t padded_date_size,
                               char *formatted_short_date, size_t formatted_short_date_size,
                               char *padded_short_date, size_t padded_short_date_size) {
    uint16_t short_year = year % 100U;

    snprintf(formatted_date, formatted_date_size, "%02d/%02d/%04d", day, month, year);
    snprintf(padded_date, padded_date_size, "%02d%02d%04d", day, month, year);
    snprintf(formatted_short_date, formatted_short_date_size, "%02d/%02d/%02d", day, month, short_year);
    snprintf(padded_short_date, padded_short_date_size, "%02d%02d%02d", day, month, short_year);
}

// ----------------------------------------------------------------------------------------
// Shared SatIOTimeData population (Rule 8.7: internal linkage; every clock
// domain -- GPSTime, systemTime, localTime, localMeanSolarTime -- fills the
// same set of calendar/formatted/padded fields from its own hour..wday
// values, so that logic lives here once instead of once per domain).
// ----------------------------------------------------------------------------------------
static void storeTimeFields(SatIOTimeData &t, uint8_t hour, uint8_t minute, uint8_t second,
                             uint16_t year, uint8_t month, uint8_t mday, uint16_t yday, uint8_t wday) {
  t.hour = hour;
  t.minute = minute;
  t.second = second;
  t.year = year;
  t.month = month;
  t.mday = mday;
  t.yday = yday;
  t.wday = wday;

  memset(t.wday_name, 0, sizeof(t.wday_name));
  strncpy(t.wday_name, satioData.week_day_names[wday], sizeof(t.wday_name) - 1U);
  memset(t.month_name, 0, sizeof(t.month_name));
  strncpy(t.month_name, satioData.month_names[month - 1U], sizeof(t.month_name) - 1U);

  formatTimeStrings(hour, minute, second,
                     t.formatted_time_HHMMSS, sizeof(t.formatted_time_HHMMSS),
                     t.padded_time_HHMMSS, sizeof(t.padded_time_HHMMSS));

  formatDateStrings(mday, month, year,
                     t.formatted_date_DDMMYYYY, sizeof(t.formatted_date_DDMMYYYY),
                     t.padded_date_DDMMYYYY, sizeof(t.padded_date_DDMMYYYY),
                     t.formatted_date_DDMMYY, sizeof(t.formatted_date_DDMMYY),
                     t.padded_date_DDMMYY, sizeof(t.padded_date_DDMMYY));

  snprintf(t.padded_hour_HH, sizeof(t.padded_hour_HH), "%02d", hour);
  snprintf(t.padded_minute_MM, sizeof(t.padded_minute_MM), "%02d", minute);
  snprintf(t.padded_second_SS, sizeof(t.padded_second_SS), "%02d", second);
  snprintf(t.padded_day_DD, sizeof(t.padded_day_DD), "%02d", mday);
  snprintf(t.padded_month_MM, sizeof(t.padded_month_MM), "%02d", month);
  snprintf(t.padded_year_YY, sizeof(t.padded_year_YY), "%02d", year % 100U);
  snprintf(t.padded_year_YYYY, sizeof(t.padded_year_YYYY), "%04d", year);
}

/* Rule 8.7: internal linkage; convenience wrapper around storeTimeFields()
   for the common case of a populated struct tm (as produced by mktime()/
   gmtime_r()/makeLMST()) plus a microsecond unix timestamp. */
static void storeTimeFromTm(SatIOTimeData &t, const struct tm &src, uint64_t unixtime_uS) {
  t.time_struct = src;
  t.unixtime_uS = unixtime_uS;
  storeTimeFields(t,
                   (uint8_t)src.tm_hour, (uint8_t)src.tm_min, (uint8_t)src.tm_sec,
                   (uint16_t)(src.tm_year + LAST_EPOCH), (uint8_t)(src.tm_mon + 1),
                   (uint8_t)src.tm_mday, (uint16_t)(src.tm_yday + 1), (uint8_t)src.tm_wday);
}

// ----------------------------------------------------------------------------------------
// storeLocalTime.
// ----------------------------------------------------------------------------------------
/**
 * Fills satioData.localTime as systemTime shifted by systemTime.second_offset
 * (the user-configured +-seconds offset from UTC).
 */
void storeLocalTime(void) {
  time_t local_sec = (time_t)((int64_t)tv_now.tv_sec + satioData.systemTime.second_offset);
  struct tm local_tm = {};
  gmtime_r(&local_sec, &local_tm);
  uint64_t local_unixtime_uS = ((uint64_t)local_sec * 1000000ULL) + (uint64_t)tv_now.tv_usec;
  storeTimeFromTm(satioData.localTime, local_tm, local_unixtime_uS);
}

/* Rule 8.7: internal linkage; only called from storeLMST() in this file. */
static void setPhotoPeriodSchedule_LMST(void) {
  satioData.localMeanSolarTime.photo_period_schedule = getPhotoPeriodData(
    satioData.system_degrees_latitude,
    satioData.system_degrees_longitude,
    satioData.localMeanSolarTime.time_struct,
    satioData.localMeanSolarTime.hour,
    satioData.localMeanSolarTime.minute,
    satioData.localMeanSolarTime.second
  );
}

void storeLMST(void) {
  struct tm lmst_tm = makeLMST(
    satioData.systemTime.year,
    satioData.systemTime.month,
    satioData.systemTime.mday,
    satioData.systemTime.hour,
    satioData.systemTime.minute,
    satioData.systemTime.second,
    satioData.system_degrees_latitude,
    satioData.system_degrees_longitude
  );
  storeTimeFromTm(satioData.localMeanSolarTime, lmst_tm, 0U);

  // Store photo period schedule for LMST
  setPhotoPeriodSchedule_LMST();
}

// ----------------------------------------------------------------------------------------
// storeSyncTime.
// ----------------------------------------------------------------------------------------
/* Rule 8.7: internal linkage; only called from storeSyncTime() in this
   file. Copies a SatIOTimeData's live fields into its own sync_* fields, so
   each clock domain remembers what it read at the moment of the last sync
   alongside what it reads live. All sync_* char buffers are declared with
   the exact same size as their live counterpart, so a straight memcpy of
   the whole (already null-terminated) buffer is safe. */
/**
 * @brief Sync time stored is derived from each SatIOTimeData unixtime_uS.
 *        Therefore sync time reflects the SatIOTimeData unixtime_uS arg, rather than being
 *        strictly UTC0/local/etc.
 *        This allows for unique sync times for each instance of SatIOTimeData.
 *        Example:
 *          systemTime sync time is in system time.
 *          localTime sync time is in local time.
 *
 */
static void snapshotSyncFields(SatIOTimeData &t) {
  t.sync_unixtime_uS = t.unixtime_uS;
  t.sync_hour   = t.hour;
  t.sync_minute = t.minute;
  t.sync_second = t.second;
  t.sync_year   = t.year;
  t.sync_month  = t.month;
  t.sync_mday   = t.mday;
  t.sync_yday   = t.yday;
  t.sync_wday   = t.wday;

  memcpy(t.sync_wday_name, t.wday_name, sizeof(t.wday_name));
  memcpy(t.sync_month_name, t.month_name, sizeof(t.month_name));

  memcpy(t.sync_formatted_time_HHMMSS, t.formatted_time_HHMMSS, sizeof(t.formatted_time_HHMMSS));
  memcpy(t.sync_formatted_date_DDMMYY, t.formatted_date_DDMMYY, sizeof(t.formatted_date_DDMMYY));
  memcpy(t.sync_formatted_date_DDMMYYYY, t.formatted_date_DDMMYYYY, sizeof(t.formatted_date_DDMMYYYY));

  memcpy(t.sync_padded_time_HHMMSS, t.padded_time_HHMMSS, sizeof(t.padded_time_HHMMSS));
  memcpy(t.sync_padded_hour_HH, t.padded_hour_HH, sizeof(t.padded_hour_HH));
  memcpy(t.sync_padded_minute_MM, t.padded_minute_MM, sizeof(t.padded_minute_MM));
  memcpy(t.sync_padded_second_SS, t.padded_second_SS, sizeof(t.padded_second_SS));

  memcpy(t.sync_padded_date_DDMMYY, t.padded_date_DDMMYY, sizeof(t.padded_date_DDMMYY));
  memcpy(t.sync_padded_date_DDMMYYYY, t.padded_date_DDMMYYYY, sizeof(t.padded_date_DDMMYYYY));
  memcpy(t.sync_padded_day_DD, t.padded_day_DD, sizeof(t.padded_day_DD));
  memcpy(t.sync_padded_month_MM, t.padded_month_MM, sizeof(t.padded_month_MM));
  memcpy(t.sync_padded_year_YY, t.padded_year_YY, sizeof(t.padded_year_YY));
  memcpy(t.sync_padded_year_YYYY, t.padded_year_YYYY, sizeof(t.padded_year_YYYY));

  t.sync = true;
}

/**
 * Snapshots every clock domain's live fields into its own sync_* fields.
 * Called once a sync (manual or GPS) has been applied to the system clock
 * and every domain refreshed from it, so e.g. satioData.systemTime.hour is
 * "now" and satioData.systemTime.sync_hour is "at the last sync".
 */
void storeSyncTime(void) {
  snapshotSyncFields(satioData.GPSTime);
  snapshotSyncFields(satioData.systemTime);
  snapshotSyncFields(satioData.localTime);
  snapshotSyncFields(satioData.localMeanSolarTime);
}

/* Rule 8.7: internal linkage; only called from setSystemTime() and
   extractDateTimeFromGPSData() in this file. GPS NMEA dates and the CLI's
   --set-datetime command both give a 2-digit year (e.g. 25 for 2025). */
static uint16_t normalizeTwoDigitYear(uint16_t year) {
  return (year < 100U) ? (year + 2000U) : year;
}

// ----------------------------------------------------------------------------------------
// setSystemTime.
// ----------------------------------------------------------------------------------------
/**
 * Applies a datetime pending in temporary values.
 */
void setSystemTime(long usec) {
  uint16_t full_year = normalizeTwoDigitYear((uint16_t)satioData.tmp_year_int);

  struct tm tmpti = {};
  tmpti.tm_year = (int)full_year - LAST_EPOCH; // Years since 1900
  tmpti.tm_mon = (int)satioData.tmp_month_int - 1; // Months 0-11
  tmpti.tm_mday = (int)satioData.tmp_day_int;
  tmpti.tm_hour = (int)satioData.tmp_hour_int;
  tmpti.tm_min = (int)satioData.tmp_minute_int;
  tmpti.tm_sec = (int)satioData.tmp_second_int;
  tmpti.tm_isdst = -1; // No DST
  time_t now = mktime(&tmpti);
  tv_now = {
      .tv_sec = now, // systemTime is always UTC.
      .tv_usec = usec
  };
  /* Rule 15.7: a terminating else with no empty if-branch -- the failure
     case is the one that needs handling, so it is the if, not the else. */
  if (settimeofday(&tv_now, NULL) != 0) {Serial.println("[settimeofday] failed");}
}

// ----------------------------------------------------------------------------------------
// getSystemTime.
// ----------------------------------------------------------------------------------------
void getSystemTime(void) {
  // --------------------------------------------------------
  // This function must be called in order to update timeinfo.
  // More calls means higher resolution of system time, and can therefore decrease performance.
  // All time is derived from tv_now/timeinfo.
  // --------------------------------------------------------
  gettimeofday(&tv_now, NULL);
  timeinfo = localtime(&tv_now.tv_sec);
  // --------------------------------------------------------
  // Keep this function quick by only storing unixtime uS.
  // --------------------------------------------------------
  satioData.systemTime.unixtime_uS = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

// ----------------------------------------------------------------------------------------
// storeSystemTime.
// ----------------------------------------------------------------------------------------
/**
 * Fills satioData.systemTime from timeinfo.
 * System time should always be UTC+-0.
 */
void storeSystemTime(void) {
  storeTimeFromTm(satioData.systemTime, *timeinfo, satioData.systemTime.unixtime_uS);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                              SYNC SYSTEM TIME FROM GPS
// ------------------------------------------------------------------------------------------------------------------------------
void extractDateTimeFromGPSData(void) {
  memset(satioData.tmp_day, 0, sizeof(satioData.tmp_day));
  satioData.tmp_day[0]=gnrmcData.utc_date[0];
  satioData.tmp_day[1]=gnrmcData.utc_date[1];
  memset(satioData.tmp_month, 0, sizeof(satioData.tmp_month));
  satioData.tmp_month[0]=gnrmcData.utc_date[2];
  satioData.tmp_month[1]=gnrmcData.utc_date[3];
  memset(satioData.tmp_year, 0, sizeof(satioData.tmp_year));
  satioData.tmp_year[0]=gnrmcData.utc_date[4];
  satioData.tmp_year[1]=gnrmcData.utc_date[5];
  memset(satioData.tmp_hour, 0, sizeof(satioData.tmp_hour));
  satioData.tmp_hour[0]=gnrmcData.utc_time[0];
  satioData.tmp_hour[1]=gnrmcData.utc_time[1];
  memset(satioData.tmp_minute, 0, sizeof(satioData.tmp_minute));
  satioData.tmp_minute[0]=gnrmcData.utc_time[2];
  satioData.tmp_minute[1]=gnrmcData.utc_time[3];
  memset(satioData.tmp_second, 0, sizeof(satioData.tmp_second));
  satioData.tmp_second[0]=gnrmcData.utc_time[4];
  satioData.tmp_second[1]=gnrmcData.utc_time[5];
  memset(satioData.tmp_millisecond, 0, sizeof(satioData.tmp_millisecond));
  satioData.tmp_millisecond[0]=gnrmcData.utc_time[7];
  satioData.tmp_millisecond[1]=gnrmcData.utc_time[8];

  satioData.tmp_day_int=atoi(satioData.tmp_day);
  satioData.tmp_month_int=atoi(satioData.tmp_month);
  satioData.tmp_year_int=atoi(satioData.tmp_year);
  satioData.tmp_hour_int=atoi(satioData.tmp_hour);
  satioData.tmp_minute_int=atoi(satioData.tmp_minute);
  satioData.tmp_second_int=atoi(satioData.tmp_second);
  satioData.tmp_millisecond_int=atoi(satioData.tmp_millisecond);

  // ----------------------------------------------------------------------------
  // Reflect what GPS is currently reporting into satioData.GPSTime, distinct
  // from the tmp_* scratch fields above (which feed setSystemTime()).
  // ----------------------------------------------------------------------------
  uint16_t full_year = normalizeTwoDigitYear((uint16_t)satioData.tmp_year_int);
  struct tm gps_tm = {};
  gps_tm.tm_year = (int)full_year - LAST_EPOCH;
  gps_tm.tm_mon = satioData.tmp_month_int - 1;
  gps_tm.tm_mday = satioData.tmp_day_int;
  gps_tm.tm_hour = satioData.tmp_hour_int;
  gps_tm.tm_min = satioData.tmp_minute_int;
  gps_tm.tm_sec = satioData.tmp_second_int;
  gps_tm.tm_isdst = -1;
  time_t gps_epoch_sec = mktime(&gps_tm); // Normalizes tm_wday/tm_yday too.
  uint64_t gps_unixtime_uS = ((uint64_t)gps_epoch_sec * 1000000ULL) +
                              ((uint64_t)satioData.tmp_millisecond_int * 10000ULL); // centiseconds -> uS
  storeTimeFromTm(satioData.GPSTime, gps_tm, gps_unixtime_uS);
}

void applyPendingDateTimeStore(void) {
  storeSystemTime();
  storeLocalTime();
  storeLMST();
}

/* */
static void applyPendingDateTime(void) {
  /**
   * For efficiency, just update system datetime and sync datetime here.
   *
   */
  int64_t entry_uS = esp_timer_get_time();
  int64_t handoff_uS = entry_uS - gps_read_done_uS;

  int64_t t_setSystemTime_uS = esp_timer_get_time();
  setSystemTime(0);
  t_setSystemTime_uS = esp_timer_get_time() - t_setSystemTime_uS;

  int64_t t_getSystemTime_uS = esp_timer_get_time();
  xSemaphoreTake(systemTimeMutex, portMAX_DELAY);
  getSystemTime();
  xSemaphoreGive(systemTimeMutex);
  t_getSystemTime_uS = esp_timer_get_time() - t_getSystemTime_uS;

  int64_t t_storeSyncTime_uS = esp_timer_get_time();
  storeSyncTime();
  t_storeSyncTime_uS = esp_timer_get_time() - t_storeSyncTime_uS;

  /**
   * [ Utlimately this time includes ]
   *  Through  readGPS()
   *           validateGPSData()
   *           syncTimeGPS()
   *           extractDateTimeFromGPSData()
   *  Finally: applyPendingDateTime()
   * 
   * Sample ( 0.4 milliseconds = 400 microseconds ):
   * 
   * SATIO_DISPLAY_OPTION_HEADLESS + Balanced Power: total=389 uS  handoff=239 uS  setSystemTime=54 uS  getSystemTime=38 uS  storeSyncTime=50 uS
   * 
   * SATIO_DISPLAY_OPTION_HEADLESS + Ultimate Perf: total=501 uS  handoff=345 uS  setSystemTime=60 uS  getSystemTime=38 uS  storeSyncTime=50 uS (^ -100 uS)
   * 
   * SATIO_DISPLAY_OPTION_LVGL + Balanced Power: total=1338 uS  handoff=1183 uS  setSystemTime=63 uS  getSystemTime=38 uS  storeSyncTime=49 uS
   * 
   * SATIO_DISPLAY_OPTION_LVGL + Ultimate Perf: total=1410 uS  handoff=1260 uS  setSystemTime=58 uS  getSystemTime=38 uS  storeSyncTime=49 uS (^ -100 uS)
   */
  int64_t readGPS_to_applyPendingDateTime_uS = esp_timer_get_time() - gps_read_done_uS;
  printf("readGPS->applyPendingDateTime: total=%lld uS  handoff=%lld uS  setSystemTime=%lld uS  getSystemTime=%lld uS  storeSyncTime=%lld uS\n",
         (long long)readGPS_to_applyPendingDateTime_uS,
         (long long)handoff_uS,
         (long long)t_setSystemTime_uS,
         (long long)t_getSystemTime_uS,
         (long long)t_storeSyncTime_uS);
}

// Reset GPSTime.sync after 1 second
int64_t GPS_SYNC_TIMEOUT_uS = 1000000;

void syncTimeGPS(void) {

  // Clear sync flag (used for gps sync indicators)
  if (satioData.GPSTime.sync == true) {
    if ((satioData.systemTime.unixtime_uS >= satioData.GPSTime.sync_unixtime_uS + GPS_SYNC_TIMEOUT_uS) ||
        (satioData.systemTime.unixtime_uS < satioData.GPSTime.sync_unixtime_uS)) {
      satioData.GPSTime.sync = false;
    }
  }

  /**
   * Automatically set system time with GPS data.
   */
  if (satioData.systemTime.set_time_automatically==true) {
    // ----------------------------------------------------------------------------------------------
    /*                                 SYNC SYSTEM TIME FROM GPS                                    */
    // ----------------------------------------------------------------------------------------------
    if ((atoi(gnggaData.satellite_count)>3) && (atoi(gnggaData.gps_precision_factor)<=3)) {
      // ----------------------------------------------------------------------------
      // Extract just what we need to perform a timing check.
      // ----------------------------------------------------------------------------
      extractDateTimeFromGPSData();
      if (satioData.systemTime.sync_immediately_flag==true) {
        // ----------------------------------------------------------------------------
        // Sync within the first 100 milliseconds of any second.
        // ----------------------------------------------------------------------------
        if (satioData.tmp_millisecond_int==0) {
          applyPendingDateTime();
          satioData.systemTime.sync_immediately_flag=false;
          satioData.GPSTime.sync = true;
          satioData.GPSTime.sync_unixtime_uS = satioData.systemTime.unixtime_uS;
          printf("syn: 0\n");
        }
      }
      else {
        // ----------------------------------------------------------------------------
        // Sync within the first 100 milliseconds of any minute.
        // ----------------------------------------------------------------------------
        if (satioData.tmp_second_int==0 && satioData.tmp_millisecond_int==0) {
          applyPendingDateTime();
          satioData.systemTime.sync_immediately_flag=false;
          satioData.GPSTime.sync = true;
          satioData.GPSTime.sync_unixtime_uS = satioData.systemTime.unixtime_uS;
          printf("syn: 1\n");
        }
      }
    }
  }
}

// ----------------------------------------------------------------------------------------
// initSystemTime.
// ----------------------------------------------------------------------------------------
void initSystemTime(void) {
  Serial.println("[SYNC] initializing system time");
  // No external RTC chip to seed from: read whatever the system clock
  // currently holds and populate the derived domains. syncTime() takes over
  // once a GPS fix is available.
  getSystemTime();
  storeSystemTime();
  storeLocalTime();
  storeLMST();
  Serial.println("[SYNC] system datetime: " +
                 String(satioData.systemTime.padded_time_HHMMSS) + " " +
                 String(satioData.systemTime.padded_date_DDMMYYYY) +
                 " (+- offset seconds " +
                 String((long)satioData.systemTime.second_offset) + ")");
}

// ----------------------------------------------------------------------------------------
// printAllTimes.
// ----------------------------------------------------------------------------------------
/* Rule 8.7: internal linkage; only called from printAllTimes() in this file. */
static void printTimeDomain(const char *label, const SatIOTimeData &t) {
  printf("[%s] %s %s  (sync: %s %s)\n",
         label,
         t.padded_time_HHMMSS, t.padded_date_DDMMYYYY,
         t.sync_padded_time_HHMMSS, t.sync_padded_date_DDMMYYYY);
}

void printAllTimes(void) {
  printTimeDomain("GPSTime", satioData.GPSTime);
  printTimeDomain("systemTime", satioData.systemTime);
  printTimeDomain("localTime", satioData.localTime);
  printTimeDomain("localMeanSolarTime", satioData.localMeanSolarTime);
}
/*
    SATIO Library. Written by Benjamin Jack Cullen.

    Intended to be MISRA Compliant (untested, unverified, in-progress).
*/

#include "./satio.h"
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>  // https://github.com/adafruit/RTClib
#include "wtgps300p.h"
#include "./task_handler.h"
#include "freertos/semphr.h"
#include <SiderealPlanets.h>
#include <SiderealObjects.h>
#include "./sidereal_helper.h"

// ----------------------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------------------
RTC_DS3231 rtc;

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
    // LOCAL TIME ITEMS
    // ------------------------------------------------------------------------------------
    .local_unixtime_uS = 0,
    .local_hour = 0,
    .local_minute = 0,
    .local_second = 0,
    .local_year = 2000,
    .local_month = 1,
    .local_mday = 1,
    .local_yday = 1,
    .local_wday = 1,
    .local_wday_name = {0},
    .local_month_name = {0},
    .utc_second_offset = 0,
    .utc_auto_offset_flag = false,
    .set_time_automatically = true,
    // ------------------------------------------------------------------------------------
    // LOCAL TIME FORMATTED (FOR USER)
    // ------------------------------------------------------------------------------------
    .formatted_local_time_HHMMSS = "00:00:00",
    .formatted_local_date_DDMMYYYY = "00/00/0000",
    .formatted_local_short_date_DDMMYY = "00/00/00",
    // ------------------------------------------------------------------------------------
    // LOCAL TIME PADDED (FOR CALC)
    // ------------------------------------------------------------------------------------
    .padded_local_time_HHMMSS = "000000",
    .padded_local_hour = "00",
    .padded_local_minute = "00",
    .padded_local_second = "00",
    .padded_local_date_DDMMYYYY = "00000000",
    .padded_local_short_date_DDMMYY = "000000",
    .padded_local_day = "00",
    .padded_local_month = "00",
    .padded_local_year = "00",
    // ------------------------------------------------------------------------------------
    // RTC ITEMS
    // ------------------------------------------------------------------------------------
    .rtc_unixtime = 0,
    .rtc_hour = 0,
    .rtc_minute = 0,
    .rtc_second = 0,
    .rtc_year = 2000,
    .rtc_month = 1,
    .rtc_mday = 1,
    .rtc_wday = 1,
    .rtc_wday_name = {0},
    // ------------------------------------------------------------------------------------
    // RTC FORMATTED (FOR USER)
    // ------------------------------------------------------------------------------------
    .formatted_rtc_time = "00:00:00",
    .formatted_rtc_date = "00/00/00",
    .padded_rtc_time_HHMMSS = "000000",
    .padded_rtc_date_DDMMYYYY = "00000000",
    // ------------------------------------------------------------------------------------
    // RTC SYNC ITEMS
    // ------------------------------------------------------------------------------------
    .rtcsync_unixtime = 0,
    .rtcsync_hour = 0,
    .rtcsync_minute = 0,
    .rtcsync_second = 0,
    .rtcsync_year = 0,
    .rtcsync_month = 0,
    .rtcsync_day = 0,
    .rtcsync_latitude = "0.0",
    .rtcsync_longitude = "0.0",
    .rtcsync_altitude = "0.0",
    // ------------------------------------------------------------------------------------
    // RTC SYNC FORMATTED (FOR USER)
    // ------------------------------------------------------------------------------------
    .formatted_rtc_sync_time = "00:00:00",
    .formatted_rtc_sync_date_DDMMYYYY = "00/00/00",
    .formatted_rtc_sync_short_date_DDMMYY = "00/00/00",
    // ------------------------------------------------------------------------------------
    // RTC SYNC PADDED (FOR CALC)
    // ------------------------------------------------------------------------------------
    .padded_rtc_sync_time_HHMMSS = "000000",
    .padded_rtc_sync_date_DDMMYYYY = "00000000",
    // ------------------------------------------------------------------------------------
    // LMST Time
    // ------------------------------------------------------------------------------------
    .LMST_tm = {},

    .LMST_hour = 0,
    .LMST_minute = 0,
    .LMST_second = 0,
    .LMST_millisecond = 0,
    .LMST_year = 0,
    .LMST_month = 0,
    .LMST_day = 0,

    .formatted_LMST_time = "00:00:00",
    .formatted_LMST_date_DDMMYYYY = "00/00/0000",
    .formatted_LMST_short_date_DDMMYY = "00/00/00",

    .padded_LMST_time_HHMMSS = "000000",
    .padded_LMST_date_DDMMYYYY = "00000000",

    /*
      schedule according to LMST time.
      { zoneName, dusk_start, dusk_end, dawn_start, dawn_end } in HH.MM format; -1.0 if N/A.
    */
    .LMST_photo_period_schedule = {},

    // ------------------------------------------------------------------------------------
    // FLAGS
    // ------------------------------------------------------------------------------------
    .set_rtc_datetime_flag = false,
    .sync_rtc_immediately_flag = true, // attempt sync immediately on starttup
    .gps_sync = false,
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
static void setSatIOAltitude(void) {
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
static void setSatIOSpeed(void) {
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
static void setSatIOGroundHeading(void) {
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
static void setSatioCoordinates(void){
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
// printAllTimes.
// ----------------------------------------------------------------------------------------
struct tm *timeinfo;
struct timeval tv_now;
SemaphoreHandle_t systemTimeMutex = nullptr;

void initSystemTimeMutex(void) {
  systemTimeMutex = xSemaphoreCreateMutex();
}

SemaphoreHandle_t dataMutex = nullptr;

void initDataMutex(void) {
  dataMutex = xSemaphoreCreateMutex();
}

void printAllTimes(void) {
  /*     UTC     */
  Serial.println("-----------------------------------------");
  Serial.println("[gnrmcData.utc_date]      " + String(gnrmcData.utc_date));
  Serial.println("[gnrmcData.utc_time]      " + String(gnrmcData.utc_time));
  Serial.println("-----------------------------------------");
  Serial.println("[satioData.tmp_year_int]    " + String(satioData.tmp_year_int));
  Serial.println("[satioData.tmp_month_int]   " + String(satioData.tmp_month_int));
  Serial.println("[satioData.tmp_day_int]     " + String(satioData.tmp_day_int));
  Serial.println("[satioData.tmp_hour_int]    " + String(satioData.tmp_hour_int));
  Serial.println("[satioData.tmp_minute_int]  " + String(satioData.tmp_minute_int));
  Serial.println("[satioData.tmp_second_int]  " + String(satioData.tmp_second_int));
  Serial.println("[satioData.tmp_msecond_int] " + String(satioData.tmp_millisecond_int));
  Serial.println("-----------------------------------------");
  Serial.println("[satioData.rtc_year]        " + String(satioData.rtc_year));
  Serial.println("[satioData.rtc_month]       " + String(satioData.rtc_month));
  Serial.println("[satioData.rtc_mday]        " + String(satioData.rtc_mday));
  Serial.println("[satioData.rtc_hour]        " + String(satioData.rtc_hour));
  Serial.println("[satioData.rtc_minute]      " + String(satioData.rtc_minute));
  Serial.println("[satioData.rtc_second]      " + String(satioData.rtc_second));
  Serial.println("[satioData.rtc_unixtime]    " + String(satioData.rtc_unixtime));
  Serial.println("[satioData.rtc wday]        " + String(rtc.now().dayOfTheWeek()));
  Serial.println("-----------------------------------------");
  Serial.println("[satioData.rtcsync_year]    " + String(satioData.rtcsync_year));
  Serial.println("[satioData.rtcsync_month]   " + String(satioData.rtcsync_month));
  Serial.println("[satioData.rtc_mday]        " + String(satioData.rtc_mday));
  Serial.println("[satioData.rtcsync_hour]    " + String(satioData.rtcsync_hour));
  Serial.println("[satioData.rtcsync_minute]  " + String(satioData.rtcsync_minute));
  Serial.println("[satioData.rtcsync_second]  " + String(satioData.rtcsync_second));
  /*    SYSTEM/LOCAL    */
  Serial.println("-----------------------------------------");
  Serial.println("[timeinfo->tm_year+last_epoch] " + String(timeinfo->tm_year+LAST_EPOCH));
  Serial.println("[timeinfo->tm_mon+1]           " + String(timeinfo->tm_mon+1));
  Serial.println("[timeinfo->tm_mday]            " + String(timeinfo->tm_mday));
  Serial.println("[timeinfo->tm_hour]            " + String(timeinfo->tm_hour));
  Serial.println("[timeinfo->tm_min]             " + String(timeinfo->tm_min));
  Serial.println("[timeinfo->tm_sec]             " + String(timeinfo->tm_sec));
  Serial.println("[tv_now.tv_sec]                " + String(tv_now.tv_sec));
  Serial.println("[tv_now.tv_usec]               " + String(tv_now.tv_usec));
  Serial.println("[timeinfo->wday]               " + String(timeinfo->tm_wday));
  Serial.println("[timeinfo->mday]               " + String(timeinfo->tm_mday));
  Serial.println("[timeinfo->yday]               " + String(timeinfo->tm_yday));
  Serial.println("-----------------------------------------");

}

// ----------------------------------------------------------------------------------------
// Shared time/date formatting (Rule 8.7: internal linkage; storeRTCTime(),
// storeLocalTime(), storeLMST(), and storeRTCSYNCTime() each need the same
// colon/slash-separated and compact-padded strings built from their own
// clock source, so the snprintf() calls live here once instead of being
// repeated per function). "%02d"/"%04d" zero-pad directly, so no separate
// digit-padding helper is needed.
// ----------------------------------------------------------------------------------------
static void formatTimeStrings(uint8_t hour, uint8_t minute, uint8_t second,
                               char *formatted_time, size_t formatted_time_size,
                               char *padded_time, size_t padded_time_size) {
    snprintf(formatted_time, formatted_time_size, "%02d:%02d:%02d", hour, minute, second);
    snprintf(padded_time, padded_time_size, "%02d%02d%02d", hour, minute, second);
}

/* formatted_short_date/padded_short_date may be NULL where a caller has no
   short-date field to fill (storeRTCTime has neither). */
static void formatDateStrings(uint8_t day, uint8_t month, uint16_t year,
                               char *formatted_date, size_t formatted_date_size,
                               char *padded_date, size_t padded_date_size,
                               char *formatted_short_date, size_t formatted_short_date_size,
                               char *padded_short_date, size_t padded_short_date_size) {
    uint16_t short_year = year % 100U;

    snprintf(formatted_date, formatted_date_size, "%02d/%02d/%04d", day, month, year);
    snprintf(padded_date, padded_date_size, "%02d%02d%04d", day, month, year);

    if (formatted_short_date != NULL) {
        snprintf(formatted_short_date, formatted_short_date_size, "%02d/%02d/%02d", day, month, short_year);
    }
    if (padded_short_date != NULL) {
        snprintf(padded_short_date, padded_short_date_size, "%02d%02d%02d", day, month, short_year);
    }
}

// ----------------------------------------------------------------------------------------
// storeRTCTime.
// ----------------------------------------------------------------------------------------
void storeRTCTime(void) {
    // Store RTC time (UTC) to avoid multiple calls to rtc.now()
    satioData.rtc_hour = rtc.now().hour();
    satioData.rtc_minute = rtc.now().minute();
    satioData.rtc_second = rtc.now().second();
    satioData.rtc_year = rtc.now().year();
    satioData.rtc_month = rtc.now().month();
    satioData.rtc_wday = rtc.now().dayOfTheWeek();
    satioData.rtc_mday = rtc.now().day();
    satioData.rtc_unixtime = rtc.now().unixtime();

    // Copy weekday name
    memset(satioData.rtc_wday_name, 0, sizeof(satioData.rtc_wday_name));
    strncpy(satioData.rtc_wday_name, satioData.week_day_names[satioData.rtc_wday], sizeof(satioData.rtc_wday_name) - 1U);

    formatTimeStrings(satioData.rtc_hour, satioData.rtc_minute, satioData.rtc_second,
                       satioData.formatted_rtc_time, sizeof(satioData.formatted_rtc_time),
                       satioData.padded_rtc_time_HHMMSS, sizeof(satioData.padded_rtc_time_HHMMSS));

    formatDateStrings(satioData.rtc_mday, satioData.rtc_month, satioData.rtc_year,
                       satioData.formatted_rtc_date, sizeof(satioData.formatted_rtc_date),
                       satioData.padded_rtc_date_DDMMYYYY, sizeof(satioData.padded_rtc_date_DDMMYYYY),
                       NULL, 0, NULL, 0);
}

// ----------------------------------------------------------------------------------------
// storeLocalTime.
// ----------------------------------------------------------------------------------------
void storeLocalTime(void) {
    // Store system time (Local Time) to avoid multiple calls to timeinfo
    satioData.local_hour = timeinfo->tm_hour;
    satioData.local_minute = timeinfo->tm_min;
    satioData.local_second = timeinfo->tm_sec;
    satioData.local_year = timeinfo->tm_year + LAST_EPOCH; // Adjust from timeinfo's year (since 1900)
    satioData.local_month = timeinfo->tm_mon + 1; // Adjust from 0
    satioData.local_wday = timeinfo->tm_wday;
    satioData.local_mday = timeinfo->tm_mday;
    satioData.local_yday = timeinfo->tm_yday + 1; // Adjust from 0
    // printf("local_yday: %d\n", satioData.local_yday);

    // Copy weekday name
    memset(satioData.local_wday_name, 0, sizeof(satioData.local_wday_name));
    strncpy(satioData.local_wday_name, satioData.week_day_names[satioData.local_wday], sizeof(satioData.local_wday_name) - 1U);

    // Copy month name
    memset(satioData.local_month_name, 0, sizeof(satioData.local_month_name));
    strncpy(satioData.local_month_name, satioData.month_names[satioData.local_month-1], sizeof(satioData.local_month_name) - 1U);

    formatTimeStrings(satioData.local_hour, satioData.local_minute, satioData.local_second,
                       satioData.formatted_local_time_HHMMSS, sizeof(satioData.formatted_local_time_HHMMSS),
                       satioData.padded_local_time_HHMMSS, sizeof(satioData.padded_local_time_HHMMSS));

    formatDateStrings(satioData.local_mday, satioData.local_month, satioData.local_year,
                       satioData.formatted_local_date_DDMMYYYY, sizeof(satioData.formatted_local_date_DDMMYYYY),
                       satioData.padded_local_date_DDMMYYYY, sizeof(satioData.padded_local_date_DDMMYYYY),
                       satioData.formatted_local_short_date_DDMMYY, sizeof(satioData.formatted_local_short_date_DDMMYY),
                       satioData.padded_local_short_date_DDMMYY, sizeof(satioData.padded_local_short_date_DDMMYY));

    // The individually-padded fields below are unique to local time (no
    // other clock source exposes its components separately), so they are
    // formatted directly here rather than through the shared helpers.
    snprintf(satioData.padded_local_hour, sizeof(satioData.padded_local_hour), "%02d", satioData.local_hour);
    snprintf(satioData.padded_local_minute, sizeof(satioData.padded_local_minute), "%02d", satioData.local_minute);
    snprintf(satioData.padded_local_second, sizeof(satioData.padded_local_second), "%02d", satioData.local_second);
    snprintf(satioData.padded_local_day, sizeof(satioData.padded_local_day), "%02d", satioData.local_mday);
    snprintf(satioData.padded_local_month, sizeof(satioData.padded_local_month), "%02d", satioData.local_month);
    snprintf(satioData.padded_local_year, sizeof(satioData.padded_local_year), "%02d", satioData.local_year % 100U);
}

/* Rule 8.7: internal linkage; only called from storeLMST() in this file. */
static void setPhotoPeriodSchedule_LMST(void) {
  satioData.LMST_photo_period_schedule = getPhotoPeriodData(
    satioData.system_degrees_latitude,
    satioData.system_degrees_longitude,
    satioData.LMST_tm,
    satioData.LMST_hour,
    satioData.LMST_minute,
    satioData.LMST_second
  );
}


void storeLMST(void) {

  satioData.LMST_tm = makeLMST(
    satioData.rtc_year,
    satioData.rtc_month,
    satioData.rtc_mday,
    satioData.rtc_hour,
    satioData.rtc_minute,
    satioData.rtc_second,
    satioData.degrees_latitude,
    satioData.degrees_longitude
  );
  /* LMST_year/month/day/hour/minute/second are uint16_t/uint8_t fields
     (Rule 10.x: assigning through an unrelated floating type is misleading
     and unnecessary when the destination is already an integer type). */
  satioData.LMST_year        = satioData.LMST_tm.tm_year + LAST_EPOCH;
  satioData.LMST_month       = satioData.LMST_tm.tm_mon + 1;
  satioData.LMST_day         = satioData.LMST_tm.tm_mday;
  satioData.LMST_hour        = satioData.LMST_tm.tm_hour;
  satioData.LMST_minute      = satioData.LMST_tm.tm_min;
  satioData.LMST_second      = satioData.LMST_tm.tm_sec;

  formatTimeStrings(satioData.LMST_hour, satioData.LMST_minute, satioData.LMST_second,
                     satioData.formatted_LMST_time, sizeof(satioData.formatted_LMST_time),
                     satioData.padded_LMST_time_HHMMSS, sizeof(satioData.padded_LMST_time_HHMMSS));

  formatDateStrings(satioData.LMST_day, satioData.LMST_month, satioData.LMST_year,
                     satioData.formatted_LMST_date_DDMMYYYY, sizeof(satioData.formatted_LMST_date_DDMMYYYY),
                     satioData.padded_LMST_date_DDMMYYYY, sizeof(satioData.padded_LMST_date_DDMMYYYY),
                     satioData.formatted_LMST_short_date_DDMMYY, sizeof(satioData.formatted_LMST_short_date_DDMMYY),
                     NULL, 0);

  // Store photo period schedule for LMST
  setPhotoPeriodSchedule_LMST();
}

// ----------------------------------------------------------------------------------------
// storeRTCSYNCTime.
// ----------------------------------------------------------------------------------------
void storeRTCSYNCTime(void) {
    // Store RTC sync time (based on local time and RTC)
    satioData.rtcsync_hour = satioData.local_hour;
    satioData.rtcsync_minute = satioData.local_minute;
    satioData.rtcsync_second = satioData.local_second;
    satioData.rtcsync_year = satioData.local_year;
    satioData.rtcsync_month = satioData.local_month;
    satioData.rtcsync_day = satioData.local_mday;
    satioData.rtcsync_unixtime = rtc.now().unixtime();

    formatTimeStrings(satioData.rtcsync_hour, satioData.rtcsync_minute, satioData.rtcsync_second,
                       satioData.formatted_rtc_sync_time, sizeof(satioData.formatted_rtc_sync_time),
                       satioData.padded_rtc_sync_time_HHMMSS, sizeof(satioData.padded_rtc_sync_time_HHMMSS));

    formatDateStrings(satioData.rtcsync_day, satioData.rtcsync_month, satioData.rtcsync_year,
                       satioData.formatted_rtc_sync_date_DDMMYYYY, sizeof(satioData.formatted_rtc_sync_date_DDMMYYYY),
                       satioData.padded_rtc_sync_date_DDMMYYYY, sizeof(satioData.padded_rtc_sync_date_DDMMYYYY),
                       satioData.formatted_rtc_sync_short_date_DDMMYY, sizeof(satioData.formatted_rtc_sync_short_date_DDMMYY),
                       NULL, 0);

    // RTC sync latitude/longitude/altitude (unique to this function).
    snprintf(satioData.rtcsync_latitude, sizeof(satioData.rtcsync_latitude), "%.7f", satioData.degrees_latitude);
    snprintf(satioData.rtcsync_longitude, sizeof(satioData.rtcsync_longitude), "%.7f", satioData.degrees_longitude);

    memset(satioData.rtcsync_altitude, 0, sizeof(satioData.rtcsync_altitude));
    strncpy(satioData.rtcsync_altitude, gnggaData.altitude, sizeof(satioData.rtcsync_altitude) - 1U);
}

// ----------------------------------------------------------------------------------------
// setSystemTime.
// ----------------------------------------------------------------------------------------
void setSystemTime(long usec) {
  // -----------------------------------------------------
  // System time = Local Time.
  // -----------------------------------------------------
  struct tm tmpti = {};
  tmpti.tm_year = int(rtc.now().year()) - LAST_EPOCH; // Years since 1900 (since last epoch)
  tmpti.tm_mon = rtc.now().month() - 1; // Months 0-11
  tmpti.tm_mday = rtc.now().day();
  tmpti.tm_hour = rtc.now().hour();
  tmpti.tm_min = rtc.now().minute();
  tmpti.tm_sec = rtc.now().second();
  tmpti.tm_isdst = -1; // No DST
  time_t now = mktime(&tmpti);
  tv_now = {
      .tv_sec = now + satioData.utc_second_offset, // negative utc_second_offset will be deducted.
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
  // System time = Local Time
  // This function must be called in order to update timeinfo
  // Only use when required and alternatively, use other
  // stored times when a lower time resolution is required.
  // --------------------------------------------------------
  xSemaphoreTake(systemTimeMutex, portMAX_DELAY);
  gettimeofday(&tv_now, NULL);
  timeinfo = localtime(&tv_now.tv_sec); // Assumes localtime works
  // --------------------------------------------------------
  // Keep this function quick by only storing unixtime uS.
  // --------------------------------------------------------
  satioData.local_unixtime_uS = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
  xSemaphoreGive(systemTimeMutex);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                              SYNC RTC FROM GPS
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
}

/* Rule 8.7: internal linkage; only called from syncRTC() in this file. */
static void setRTCDateTime(void) {
  rtc.adjust(DateTime((uint16_t)satioData.tmp_year_int,
            (uint8_t)satioData.tmp_month_int,
            (uint8_t)satioData.tmp_day_int,
            (uint8_t)satioData.tmp_hour_int,
            (uint8_t)satioData.tmp_minute_int,
            (uint8_t)satioData.tmp_second_int));
  // Serial.println("[ " + String(satioData.local_unixtime_uS) + " ] sync time in setRTCDateTime");
  storeRTCSYNCTime();
  satioData.sync_rtc_immediately_flag=false;
}

// GPS sync timeout timer
int64_t gps_sync_timestamp = 0;
const int64_t GPS_SYNC_TIMEOUT_uS = 1000000; // Reset gps_sync after 1 second

void syncRTC(void) {

  // Set GPS sync flag false
  if (satioData.gps_sync == true) {
    if ((satioData.local_unixtime_uS >= gps_sync_timestamp + GPS_SYNC_TIMEOUT_uS) ||
        (satioData.local_unixtime_uS < gps_sync_timestamp)) {
      satioData.gps_sync = false;
    }
  }

  /**
   * Manually set RTC datetime.
   * 
   * (1) Set set_time_automatically false.
   * (1) Set temporary datetime values.
   * (2) Set satioData.set_rtc_datetime_flag true.
   */
  if (satioData.set_time_automatically==false && satioData.set_rtc_datetime_flag==true)
  {
    satioData.set_rtc_datetime_flag=false;
    setRTCDateTime();
    setSystemTime(0);
    getSystemTime();
    storeLocalTime();
    storeLMST();
    printf("[rtc] sync 2: %s\n", String(rtc.now().timestamp()).c_str());
  }

  /**
   * Automatically set RTC datetime with GPS data.
   */
  else if (satioData.set_time_automatically==true && satioData.set_rtc_datetime_flag==true) {
    satioData.set_rtc_datetime_flag=false;
    // ----------------------------------------------------------------------------------------------
    /*                                 SYNC RTC TIME & DATE FROM GPS                               */
    // ----------------------------------------------------------------------------------------------
    if ((atoi(gnggaData.satellite_count)>3) && (atoi(gnggaData.gps_precision_factor)<=3)) {
      // ----------------------------------------------------------------------------
      // Extract just what we need to perform a timing check.
      // ----------------------------------------------------------------------------
      extractDateTimeFromGPSData();
      if (satioData.sync_rtc_immediately_flag==true) {
        // ----------------------------------------------------------------------------
        // Sync within the first 100 milliseconds of any second.
        // ----------------------------------------------------------------------------
        if (satioData.tmp_millisecond_int==0) {
          // --------------------------------------------------------------------------
          // Sync RTC to UTC.
          // --------------------------------------------------------------------------
          setRTCDateTime();
          setSystemTime(0);
          getSystemTime();
          storeLocalTime();
          storeLMST();
          satioData.gps_sync=true;
          gps_sync_timestamp = satioData.local_unixtime_uS;
          printf("[rtc] sync 0: %s\n", String(rtc.now().timestamp()).c_str());
        }
      }
      else {
        // ----------------------------------------------------------------------------
        // Sync within the first 100 milliseconds of any minute.
        // ----------------------------------------------------------------------------
        if (satioData.tmp_second_int==0 && satioData.tmp_millisecond_int==0) {
          // --------------------------------------------------------------------------
          // Sync RTC to UTC.
          // --------------------------------------------------------------------------
          setRTCDateTime();
          setSystemTime(0);
          getSystemTime();
          storeLocalTime();
          storeLMST();
          satioData.gps_sync=true;
          gps_sync_timestamp = satioData.local_unixtime_uS;
          printf("[rtc] sync 1: %s\n", String(rtc.now().timestamp()).c_str());
        }
      }
    }
  }
}

// ----------------------------------------------------------------------------------------
// setSatIOData.
// ----------------------------------------------------------------------------------------
void setSatIOData(void) {
    syncRTC();
    setSatioCoordinates();
    setSatIOAltitude();
    setSatIOSpeed();
    setSatIOGroundHeading();
    setGroundHeadingName(atof(gnrmcData.ground_heading));
}

// ----------------------------------------------------------------------------------------
// initSystemTime.
// ----------------------------------------------------------------------------------------
void initSystemTime(void) {
  Serial.println("[SYNC] synchronizing system time with RTC");
  int rtc_second_now=rtc.now().second();
  getSystemTime();
  while (rtc_second_now==rtc.now().second()) { // wait to sync
    setSystemTime(0);
  }
  getSystemTime();
  storeLocalTime();
  storeRTCTime();
  storeLMST();
  Serial.println("[SYNC] RTC datetime:    " +
                  String(satioData.padded_rtc_time_HHMMSS) + " " +
                  String(satioData.padded_rtc_date_DDMMYYYY));
  Serial.println("[SYNC] system datetime: " +
                 String(satioData.padded_local_time_HHMMSS) + " " +
                 String(satioData.padded_local_date_DDMMYYYY) +
                 " (+- offset seconds " +
                 String(satioData.utc_second_offset) + ")");
}
/*
    SATIO Library. Written by Benjamin Jack Cullen.

*/

#include "./satio.h"
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>  // https://github.com/adafruit/RTClib
#include "wtgps300p.h"
#include "./task_handler.h"
#include "freertos/semphr.h"

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
    // Geo-Positional Time
    // ------------------------------------------------------------------------------------
    .geo_positional_lat_weight = 0.0,

    .geo_positional_hour = 0,
    .geo_positional_minute = 0,
    .geo_positional_second = 0,
    .geo_positional_millisecond = 0,
    .geo_positional_year = 0,
    .geo_positional_month = 0,
    .geo_positional_day = 0,

    .formatted_geo_positional_time = "00:00:00",
    .formatted_geo_positional_date_DDMMYYYY = "00/00/0000",
    .formatted_geo_positional_short_date_DDMMYY = "00/00/00",

    .padded_geo_positional_time_HHMMSS = "000000",
    .padded_geo_positional_date_DDMMYYYY = "00000000",

    .geo_positional_day_hours = 0.0,
    .geo_positional_night_hours = 0.0,
    .geo_positional_anomaly = 0.0,
    .geo_positional_sunrise = 0.0,
    .geo_positional_sunset = 0.0,

    .geo_cache_last_updated = 0,
    .geo_cache_lat = 0.0,
    .geo_cache_lon = 0.0,
    .geo_noon_last = 0,
    .geo_noon_next = 0,
    .geo_noon_sharpness_last = 0.0,
    .geo_noon_sharpness_next = 0.0,

    // ── Proportional day ─────────────────────────────────────────────────────────
    .geo_abs_day_span_sec = 0.0,
    .geo_abs_day_elapsed_sec = 0.0,
    .geo_abs_day_fraction = 0.0,
    .geo_abs_prop_second_si = 0.0,
    .geo_abs_prop_hour = 0.0,
    .geo_abs_prop_minute = 0.0,
    .geo_abs_prop_second = 0.0,
    .geo_abs_wobble = 0.0,
    .geo_abs_anomaly = 0.0,
    .geo_abs_noon_sharpness = 0.0,

    // ── Geo calendar (J2000.0 epoch, vernal equinox = month 1) ───────────────────
    .geo_abs_year_j2000 = 0.0,
    .geo_abs_year_position = 0.0,
    .geo_abs_cal_month = 0,
    .geo_abs_cal_day = 0,
    .geo_abs_solar_quarter = 0,
    .geo_abs_days_to_next_quarter = 0.0,

    .formatted_geo_abs_prop_time = "00:00:00",
    .formatted_geo_abs_cal_date = "00/00/00",
    .geo_abs_year_decimal = "0.0",
    .geo_abs_quarter_str = "",
    .geo_abs_polar_status = "",

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
 */
void setSatIOAltitude() {
  /* set converted values */
  // ...
  satioData.altitude = atoi(gnggaData.altitude);
  // ---------------------------------------------------------------------
  // Select which value to use from the system.
  // ---------------------------------------------------------------------
  if      (satioData.altitude_value_mode==SATIO_MODE_GPS)  {satioData.system_altitude = satioData.altitude;}
  else if (satioData.altitude_value_mode==SATIO_MODE_USER) {satioData.system_altitude = satioData.user_altitude;}
}

/**
 * Set SatIO Speed According To Update Mode. (The following should either be set or not set. If not set then conditions the be checked elsewhere)
 */
void setSatIOSspeed() {
  /* set converted values */
  // ...
  satioData.speed = atof(gnrmcData.ground_speed);
  // ---------------------------------------------------------------------
  // Select which value to use from the system.
  // ---------------------------------------------------------------------
  if      (satioData.speed_value_mode==SATIO_MODE_GPS)  {satioData.system_speed = satioData.speed;}
  else if (satioData.speed_value_mode==SATIO_MODE_USER) {satioData.system_speed = satioData.user_speed;}
}

/**
 * Set SatIO Ground Heading According To Update Mode. (The following should either be set or not set. If not set then conditions the be checked elsewhere)
 */
void setSatIOGroundHeading() {
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
String groundHeadingDegreesToNESW(float num) {
  if (num == 0 || num == 360)      {return String("N");}
  else if (num > 0 && num < 45)    {return String("NNE");}
  else if (num == 45)              {return String("NE");}
  else if (num > 45 && num < 90)   {return String("ENE");}
  else if (num == 90)              {return String("E");}
  else if (num > 90 && num < 135)  {return String("ESE");}
  else if (num == 135)             {return String("SE");}
  else if (num > 135 && num < 180) {return String("SSE");}
  else if (num == 180)             {return String("S");}
  else if (num > 180 && num < 225) {return String("SSW");}
  else if (num == 225)             {return String("SW");}
  else if (num > 225 && num < 270) {return String("WSW");}
  else if (num == 270)             {return String("W");}
  else if (num > 270 && num < 315) {return String("WNW");}
  else if (num == 315)             {return String("NW");}
  else if (num > 315 && num < 360) {return String("NNW");}
  return String("");
}
void setGroundHeadingName(float num) {
  memset(satioData.ground_heading_name, 0, sizeof(satioData.ground_heading_name));
  strcpy(satioData.ground_heading_name, groundHeadingDegreesToNESW(num).c_str());
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                         CONVERT COORDINTE DATA
// ------------------------------------------------------------------------------------------------------------------------------
void setSatioCoordinates(){
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
  // Save formatted latitude value as a string for later use.
  // -----------------------------------------------------------------------------------------
  scanf("%lf17", &satioData.degrees_latitude);
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
  // -----------------------------------------------------------------------------------------
  // Save formatted latitude value as a string for later use.
  // -----------------------------------------------------------------------------------------
  scanf("%lf17", &satioData.degrees_longitude);
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
// padDigitsZero.
// ----------------------------------------------------------------------------------------
void padDigitsZero(int digits, char* output, size_t output_size) {
    // Prepends a zero to pad a string of digits evenly
    memset(output, 0, output_size);
    if (digits < 10) {
        strcpy(output, "0");
    }
    char temp[12]; // Enough for int32_t in base 10
    itoa(digits, temp, 10);
    strncat(output, temp, output_size - strlen(output) - 1);
}

// ----------------------------------------------------------------------------------------
// storeRTCTime.
// ----------------------------------------------------------------------------------------
void storeRTCTime(void) {
    // Store RTC time (UTC) to avoid multiple calls to rtc.now()
    // Serial.printf("[writeI2C] storeRTCTime\n");
    // xSemaphoreTake(i2c_bus0_mutex, 1000 / portTICK_PERIOD_MS);
    satioData.rtc_hour = rtc.now().hour();
    satioData.rtc_minute = rtc.now().minute();
    satioData.rtc_second = rtc.now().second();
    satioData.rtc_year = rtc.now().year();
    satioData.rtc_month = rtc.now().month();
    satioData.rtc_wday = rtc.now().dayOfTheWeek();
    satioData.rtc_mday = rtc.now().day();
    satioData.rtc_unixtime = rtc.now().unixtime();
    // xSemaphoreGive(i2c_bus0_mutex);

    // Debug output without String
    // char debug_str[MAX_GLOBAL_ELEMENT_SIZE];
    // snprintf(debug_str, MAX_GLOBAL_ELEMENT_SIZE, "RTC: %d", satioData.rtc_hour);
    // Serial.println(debug_str);

    // Copy weekday name
    memset(satioData.rtc_wday_name, 0, sizeof(satioData.rtc_wday_name));
    strcpy(satioData.rtc_wday_name, satioData.week_day_names[satioData.rtc_wday]);

    // Format time (HH:MM:SS)
    char hour_str[3], min_str[3], sec_str[3];
    padDigitsZero(satioData.rtc_hour, hour_str, sizeof(hour_str));
    padDigitsZero(satioData.rtc_minute, min_str, sizeof(min_str));
    padDigitsZero(satioData.rtc_second, sec_str, sizeof(sec_str));
    memset(satioData.formatted_rtc_time, 0, sizeof(satioData.formatted_rtc_time));
    snprintf(satioData.formatted_rtc_time, sizeof(satioData.formatted_rtc_time), "%s:%s:%s", hour_str, min_str, sec_str);

    // Debug formatted time
    // snprintf(debug_str, MAX_GLOBAL_ELEMENT_SIZE, "formatted_rtc_time: %s", satioData.formatted_rtc_time);
    // Serial.println(debug_str);

    // Format date (DD/MM/YYYY)
    char day_str[3], month_str[3], year_str[5];
    padDigitsZero(satioData.rtc_mday, day_str, sizeof(day_str));
    padDigitsZero(satioData.rtc_month, month_str, sizeof(month_str));
    padDigitsZero(satioData.rtc_year, year_str, sizeof(year_str));

    memset(satioData.formatted_rtc_date, 0, sizeof(satioData.formatted_rtc_date));
    snprintf(satioData.formatted_rtc_date, sizeof(satioData.formatted_rtc_date), "%s/%s/%s", day_str, month_str, year_str);

    // Format padded time (HHMMSS)
    memset(satioData.padded_rtc_time_HHMMSS, 0, sizeof(satioData.padded_rtc_time_HHMMSS));
    snprintf(satioData.padded_rtc_time_HHMMSS, sizeof(satioData.padded_rtc_time_HHMMSS), "%s%s%s", hour_str, min_str, sec_str);

    // Format padded date (DDMMYYYY)
    memset(satioData.padded_rtc_date_DDMMYYYY, 0, sizeof(satioData.padded_rtc_date_DDMMYYYY));
    snprintf(satioData.padded_rtc_date_DDMMYYYY, sizeof(satioData.padded_rtc_date_DDMMYYYY), "%s%s%s", day_str, month_str, year_str);
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

    // Copy weekday name
    memset(satioData.local_wday_name, 0, sizeof(satioData.local_wday_name));
    strcpy(satioData.local_wday_name, satioData.week_day_names[satioData.local_wday]);

    // Copy month name
    memset(satioData.local_month_name, 0, sizeof(satioData.local_month_name));
    strcpy(satioData.local_month_name, satioData.month_names[satioData.local_month-1]);

    // Format time (HH:MM:SS)
    char hour_str[3], min_str[3], sec_str[3];
    padDigitsZero(satioData.local_hour, hour_str, sizeof(hour_str));
    padDigitsZero(satioData.local_minute, min_str, sizeof(min_str));
    padDigitsZero(satioData.local_second, sec_str, sizeof(sec_str));
    memset(satioData.formatted_local_time_HHMMSS, 0, sizeof(satioData.formatted_local_time_HHMMSS));
    snprintf(satioData.formatted_local_time_HHMMSS, sizeof(satioData.formatted_local_time_HHMMSS), "%s:%s:%s", hour_str, min_str, sec_str);

    char day_str[3], month_str[3], year_str[5];
    padDigitsZero(satioData.local_mday, day_str, sizeof(day_str));
    padDigitsZero(satioData.local_month, month_str, sizeof(month_str));
    padDigitsZero(satioData.local_year, year_str, sizeof(year_str));
    memset(satioData.formatted_local_date_DDMMYYYY, 0, sizeof(satioData.formatted_local_date_DDMMYYYY));
    snprintf(satioData.formatted_local_date_DDMMYYYY, sizeof(satioData.formatted_local_date_DDMMYYYY), "%s/%s/%s", day_str, month_str, year_str);

    char short_year_str[3] = { year_str[2], year_str[3], '\0' };
    memset(satioData.formatted_local_short_date_DDMMYY, 0, sizeof(satioData.formatted_local_short_date_DDMMYY));
    snprintf(satioData.formatted_local_short_date_DDMMYY, sizeof(satioData.formatted_local_short_date_DDMMYY), "%s/%s/%s", day_str, month_str, short_year_str);

    memset(satioData.padded_local_time_HHMMSS, 0, sizeof(satioData.padded_local_time_HHMMSS));
    snprintf(satioData.padded_local_time_HHMMSS, sizeof(satioData.padded_local_time_HHMMSS), "%s%s%s", hour_str, min_str, sec_str);

    memset(satioData.padded_local_date_DDMMYYYY, 0, sizeof(satioData.padded_local_date_DDMMYYYY));
    snprintf(satioData.padded_local_date_DDMMYYYY, sizeof(satioData.padded_local_date_DDMMYYYY), "%s%s%s", day_str, month_str, year_str);

    memset(satioData.padded_local_short_date_DDMMYY, 0, sizeof(satioData.padded_local_short_date_DDMMYY));
    snprintf(satioData.padded_local_short_date_DDMMYY, sizeof(satioData.padded_local_short_date_DDMMYY), "%s%s%s", day_str, month_str, short_year_str);

    // Format padded hour (HH)
    memset(satioData.padded_local_hour, 0, sizeof(satioData.padded_local_hour));
    snprintf(satioData.padded_local_hour, MAX_GLOBAL_ELEMENT_SIZE, "%s", hour_str);

    // Format padded minute (MM)
    memset(satioData.padded_local_minute, 0, sizeof(satioData.padded_local_minute));
    snprintf(satioData.padded_local_minute, MAX_GLOBAL_ELEMENT_SIZE, "%s", min_str);

    // Format padded second (SS)
    memset(satioData.padded_local_second, 0, sizeof(satioData.padded_local_second));
    snprintf(satioData.padded_local_second, MAX_GLOBAL_ELEMENT_SIZE, "%s", sec_str);

    // Format padded day (DD)
    memset(satioData.padded_local_day, 0, sizeof(satioData.padded_local_day));
    snprintf(satioData.padded_local_day, MAX_GLOBAL_ELEMENT_SIZE, "%s", day_str);

    // Format padded month (MM)
    memset(satioData.padded_local_month, 0, sizeof(satioData.padded_local_month));
    snprintf(satioData.padded_local_month, MAX_GLOBAL_ELEMENT_SIZE, "%s", month_str);

    // Format padded year (YY)
    memset(satioData.padded_local_year, 0, sizeof(satioData.padded_local_year));
    snprintf(satioData.padded_local_year, MAX_GLOBAL_ELEMENT_SIZE, "%s", String(String(year_str[2]) + String(year_str[3])).c_str());

    updateGeoPositionalTime();
  }

// ----------------------------------------------------------------------------------------
// updateGeoPositionalTime - With latitude weighting for true break down of time (time anomaly at poles).
// Computes true solar (geo-positional) time by snapshotting the RTC (UTC) and
// offsetting by longitude, weighted by latitude.
//
// Longitude offset: 1 degree = 240 seconds (15 deg/h).
// Latitude weight:  cos(lat) — scales the offset from full (equator) to zero (poles).
//
// At the poles, longitude converges and solar time becomes geometrically undefined.
// The weight smoothly collapses the offset to zero as latitude approaches ±90°,
// mirroring the physical breakdown of solar timekeeping at extreme latitudes.

/*
  Anomaly Thresholds (Optional Extension)
  These thresholds can be used to categorize the severity of the time anomaly based on the
  latitude weight:

  0.9< ~26° Normal solar time
  0.5 – 0.926°–60°Partial solar validity
  0.1 – 0.560°–84°High anomaly zone
  0.1<  84°Time undefined

  The genuine breakdown of time at the poles isn't in the time offset formula — it's in the
  meaningfulness of solar time as a concept.

  This real local time should be legitimately perspective building over manufactured fudged,
  authoritative time. For example I realize now that there are equal hours of daylight either
  side of noon, at any position on Earth, if using this real local time. We can start to see
  the elegance of real local time, and possibly apply it to ancient sites, built around
  the movement of the sun and celestial objects.

  Time should now change as you move around the globe, even just a little, and not be artificially
  fixed to an imaginary timezone. The time should be a function of your position on the globe, and not a
  function of political boundaries. This is the real local time.

  Now you may know and keep the real time, anywhere on earth.
*/

// ----------------------------------------------------------------------------------------
// hoursToHHMM
// Converts a fractional hour value where the decimal part represents minutes, not
// hundredths of an hour.
// ----------------------------------------------------------------------------------------
static inline double hoursToHHMM(double frac_hours) {
    int    h   = (int)frac_hours;
    int    m   = (int)((frac_hours - h) * 60.0);
    return (double)h + (double)m / 100.0;
}

// ----------------------------------------------------------------------------------------
// updateGeoPositionalTime.
// ----------------------------------------------------------------------------------------
void updateGeoPositionalTime(void) {
    // Build UTC time_t from stored RTC values (RTC always holds UTC).
    struct tm utc_tm = {0};
    utc_tm.tm_year  = satioData.rtc_year  - LAST_EPOCH;
    utc_tm.tm_mon   = satioData.rtc_month - 1;
    utc_tm.tm_mday  = satioData.rtc_mday;
    utc_tm.tm_hour  = satioData.rtc_hour;
    utc_tm.tm_min   = satioData.rtc_minute;
    utc_tm.tm_sec   = satioData.rtc_second;
    utc_tm.tm_isdst = 0;
    time_t utc_sec  = mktime(&utc_tm);

    // True solar time offset is purely longitudinal — no latitude factor.
    double lon_offset_sec_f = satioData.system_degrees_longitude * 240.0;
    time_t lon_offset_sec   = (time_t)lon_offset_sec_f;

    // Geo-positional unix time.
    time_t geo_sec = utc_sec + lon_offset_sec;

    // Decompose into calendar fields.
    struct tm geo_tm;
    gmtime_r(&geo_sec, &geo_tm);

    satioData.geo_positional_hour        = (double)geo_tm.tm_hour;
    satioData.geo_positional_minute      = (double)geo_tm.tm_min;
    satioData.geo_positional_second      = (double)geo_tm.tm_sec;
    satioData.geo_positional_year        = (double)(geo_tm.tm_year + LAST_EPOCH);
    satioData.geo_positional_month       = (double)(geo_tm.tm_mon + 1);
    satioData.geo_positional_day         = (double)geo_tm.tm_mday;

    // ------------------------------------------------------------------------
    // Photoperiod calculation.
    // Day of year (1-365) from the geo-positional date.
    // ------------------------------------------------------------------------
    int day_of_year = geo_tm.tm_yday + 1;  // tm_yday is 0-based

    // Solar declination (degrees): where the sun sits relative to the equator.
    // +23.45° at summer solstice, -23.45° at winter solstice.
    double decl_deg = -23.45 * cos((360.0 / 365.0) * (day_of_year + 10) * M_PI / 180.0);
    double decl_rad = decl_deg * M_PI / 180.0;

    double lat_rad  = satioData.system_degrees_latitude * M_PI / 180.0;

    // Hour angle at sunrise/sunset: arccos(-tan(lat) * tan(decl)).
    // The argument can exceed ±1 at the poles — clamp to handle polar day/night.
    double cos_omega = -tan(lat_rad) * tan(decl_rad);

    double day_hours;
    if (cos_omega <= -1.0) {
        // Sun never sets — polar day (midnight sun).
        day_hours = 24.0;
    } else if (cos_omega >= 1.0) {
        // Sun never rises — polar night.
        day_hours = 0.0;
    } else {
        double omega_deg = acos(cos_omega) * 180.0 / M_PI;
        day_hours = 2.0 * omega_deg / 15.0;
    }

    double night_hours = 24.0 - day_hours;

    // Anomaly flag: how far into polar conditions we are.
    // 0.0 = normal photoperiod, 1.0 = fully at the pole.
    double abs_lat = fabs(satioData.system_degrees_latitude);
    double anomaly = 0.0;
    if (abs_lat > 66.5) {
        anomaly = (abs_lat - 66.5) / (90.0 - 66.5);
    }

    // Sunrise and sunset as fractional hours from midnight (solar noon = 12.0).
    // Only meaningful when 0 < day_hours < 24.
    double sunrise_h = 12.0 - (day_hours / 2.0);
    double sunset_h  = 12.0 + (day_hours / 2.0);

    satioData.geo_positional_day_hours   = hoursToHHMM(day_hours);
    satioData.geo_positional_night_hours = hoursToHHMM(night_hours);
    satioData.geo_positional_anomaly     = anomaly;
    satioData.geo_positional_sunrise     = hoursToHHMM(sunrise_h);
    satioData.geo_positional_sunset      = hoursToHHMM(sunset_h);

    printf("Geo-positional Time : %02d:%02d:%02d %02d/%02d/%04d\n",
           (int)satioData.geo_positional_hour,
           (int)satioData.geo_positional_minute,
           (int)satioData.geo_positional_second,
           (int)satioData.geo_positional_day,
           (int)satioData.geo_positional_month,
           (int)satioData.geo_positional_year);

    printf("Photoperiod         : day=%05.2f  night=%05.2f  sunrise=%05.2f  sunset=%05.2f  anomaly=%.4f%s\n",
           satioData.geo_positional_day_hours,
           satioData.geo_positional_night_hours,
           satioData.geo_positional_sunrise,
           satioData.geo_positional_sunset,
           anomaly,
           cos_omega <= -1.0 ? "  [POLAR DAY]"  :
           cos_omega >=  1.0 ? "  [POLAR NIGHT]" : "");

    // Format geo-positional time (HH:MM:SS)
    char hour_str[3], min_str[3], sec_str[3];
    padDigitsZero(satioData.geo_positional_hour, hour_str, sizeof(hour_str));
    padDigitsZero(satioData.geo_positional_minute, min_str, sizeof(min_str));
    padDigitsZero(satioData.geo_positional_second, sec_str, sizeof(sec_str));
    memset(satioData.formatted_geo_positional_time, 0, sizeof(satioData.formatted_geo_positional_time));
    snprintf(satioData.formatted_geo_positional_time, sizeof(satioData.formatted_geo_positional_time), "%s:%s:%s", hour_str, min_str, sec_str);

    // Format geo-positional date (DD/MM/YYYY)
    char day_str[3], month_str[3], year_str[5];
    padDigitsZero(satioData.geo_positional_day, day_str, sizeof(day_str));
    padDigitsZero(satioData.geo_positional_month, month_str, sizeof(month_str));
    padDigitsZero(satioData.geo_positional_year, year_str, sizeof(year_str));
    memset(satioData.formatted_geo_positional_date_DDMMYYYY, 0, sizeof(satioData.formatted_geo_positional_date_DDMMYYYY));
    snprintf(satioData.formatted_geo_positional_date_DDMMYYYY, sizeof(satioData.formatted_geo_positional_date_DDMMYYYY), "%s/%s/%s", day_str, month_str, year_str);

    // Format geo-positional short date (DD/MM/YY)
    char short_year_str[3] = { year_str[2], year_str[3], '\0' };
    memset(satioData.formatted_geo_positional_short_date_DDMMYY, 0, sizeof(satioData.formatted_geo_positional_short_date_DDMMYY));
    snprintf(satioData.formatted_geo_positional_short_date_DDMMYY, sizeof(satioData.formatted_geo_positional_short_date_DDMMYY), "%s/%s/%s", day_str, month_str, short_year_str);

    // Format padded geo-positional time (HHMMSS)
    memset(satioData.padded_geo_positional_time_HHMMSS, 0, sizeof(satioData.padded_geo_positional_time_HHMMSS));
    snprintf(satioData.padded_geo_positional_time_HHMMSS, sizeof(satioData.padded_geo_positional_time_HHMMSS), "%s%s%s", hour_str, min_str, sec_str);

    // Format padded geo-positional date (DDMMYYYY)
    memset(satioData.padded_geo_positional_date_DDMMYYYY, 0, sizeof(satioData.padded_geo_positional_date_DDMMYYYY));
    snprintf(satioData.padded_geo_positional_date_DDMMYYYY, sizeof(satioData.padded_geo_positional_date_DDMMYYYY), "%s%s%s", day_str, month_str, year_str);

}

// // ============================================================================
// // Geo-Positional Time — Complete Implementation
// // ============================================================================
// //
// // LAYERS:
// //   UTC          → ground truth, never touched
// //   Civil local  → geo-political convenience, never touched
// //   Geo-pos mean → longitude-offset solar time (existing, preserved)
// //   Geo-pos abs  → proportional time anchored to true solar noon (new)
// //
// // GEO-POSITIONAL ABSOLUTE (ABS) SPECIFICATION:
// //   Epoch        : J2000.0 (1 Jan 2000 12:00 TT = unix 946728000)
// //   Year         : elapsed tropical years from J2000.0 as decimal
// //   Year boundary: vernal equinox moment at observer's longitude
// //   Month        : 1-12 counted from vernal equinox (March = 1)
// //                  boundaries at fixed tropical year fractions
// //                  quarters anchored to solstices/equinoxes
// //   Day boundary : solar noon at observer's longitude
// //   Day unit     : true noon-to-noon interval (Option C)
// //                  stretches/compresses with true solar day
// //                  merges across SI days when noon is too flat to distinguish
// //                  polar day: one geo-day spans entire continuous daylight period
// //   Clock        : 00:00:00 at solar noon, counts up through proportional units
// //                  proportional second = noon_span_SI / 86400
// //                  hours/minutes/seconds are fractions of the actual day span
// //   Wobble       : noon sharpness degrades toward poles
// //                  uncertainty in proportional second encoded explicitly
// //   Anomaly      : 0.0 (normal) → 1.0 (pole, time undefined)
// //
// // CACHE:
// //   Noon positions recomputed when:
// //     - elapsed SI time since last cache > 3600s (1 hour), OR
// //     - position has moved > 1km from cached position
// //   200-day forward scan cap for polar noon merging
// //
// // ============================================================================

// #include <math.h>
// #include <time.h>
// #include <stdio.h>
// #include <string.h>
// #include <stdbool.h>

// // J2000.0 as unix epoch seconds (1 Jan 2000 12:00 UTC)
// #define J2000_UNIX          946728000.0

// // Tropical year in seconds
// #define TROPICAL_YEAR_SEC   31556925.2

// // Noon sharpness threshold below which consecutive SI noons merge
// // into one extended geo-positional day (degrees/hour)
// #define NOON_MERGE_THRESHOLD    0.1

// // Maximum forward scan for next distinguishable noon (200 SI days in seconds)
// #define NOON_SCAN_CAP_SEC       (200 * 86400)

// // Cache invalidation thresholds
// #define CACHE_INTERVAL_SEC      3600        // 1 hour
// #define CACHE_DISTANCE_METRES   1000.0      // 1 km

// // Vernal equinox offset in the declination formula:
// // day_of_year + 10 shifts the cosine so that doy=1 (1 Jan) maps correctly
// // to the tropical year, placing the vernal equinox at position 0.0
// #define DECL_DOY_OFFSET         10

// // Quarter boundaries as tropical year fractions
// // 0.00 = vernal equinox   (March,     month 1)
// // 0.25 = summer solstice  (June,      month 4)
// // 0.50 = autumnal equinox (September, month 7)
// // 0.75 = winter solstice  (December,  month 10)
// static const double QUARTER_FRACTIONS[4] = { 0.00, 0.25, 0.50, 0.75 };
// static const char*  QUARTER_NAMES[4]     = { "SPRING", "SUMMER", "AUTUMN", "WINTER" };

// // ============================================================================
// // hoursToHHMM
// // Converts fractional hours (e.g. 16.283) to HH.MM display double (e.g. 16.17)
// // where the decimal part is minutes, not hundredths of an hour.
// // ============================================================================
// static inline double hoursToHHMM(double frac_hours) {
//     int h = (int)frac_hours;
//     int m = (int)((frac_hours - h) * 60.0);
//     return (double)h + (double)m / 100.0;
// }

// // ============================================================================
// // approxDistanceMetres
// // Fast flat-earth approximation — sufficient for a 1km cache threshold.
// // ============================================================================
// static double approxDistanceMetres(double lat1, double lon1,
//                                    double lat2, double lon2) {
//     double dlat = (lat2 - lat1) * 111320.0;
//     double dlon = (lon2 - lon1) * 111320.0 * cos(lat1 * M_PI / 180.0);
//     return sqrt(dlat * dlat + dlon * dlon);
// }

// // ============================================================================
// // solarDeclination
// // Returns solar declination in degrees for a given day-of-year.
// // Uses standard cosine approximation: accurate to ~0.5°.
// // ============================================================================
// static double solarDeclination(int doy) {
//     return -23.45 * cos((360.0 / 365.0) * (doy + DECL_DOY_OFFSET) * M_PI / 180.0);
// }

// // ============================================================================
// // solarElevation
// // Returns solar elevation in degrees at a given SI unix time and position.
// // Elevation > 0 = above horizon, < 0 = below horizon.
// // ============================================================================
// static double solarElevation(time_t t, double lat_deg, double lon_deg) {
//     struct tm ut;
//     gmtime_r(&t, &ut);
//     int    doy      = ut.tm_yday + 1;
//     double hour_ut  = ut.tm_hour + ut.tm_min / 60.0 + ut.tm_sec / 3600.0;

//     double decl_deg = solarDeclination(doy);
//     double decl_rad = decl_deg * M_PI / 180.0;
//     double lat_rad  = lat_deg  * M_PI / 180.0;

//     // Solar noon in UT at this longitude (mean solar, no equation of time)
//     double solar_noon_ut   = 12.0 - (lon_deg / 15.0);
//     double hour_angle_deg  = (hour_ut - solar_noon_ut) * 15.0;
//     double hour_angle_rad  = hour_angle_deg * M_PI / 180.0;

//     double sin_elev = sin(lat_rad) * sin(decl_rad)
//                     + cos(lat_rad) * cos(decl_rad) * cos(hour_angle_rad);

//     // Clamp to [-1,1] before asin to guard floating point edge cases
//     if (sin_elev >  1.0) sin_elev =  1.0;
//     if (sin_elev < -1.0) sin_elev = -1.0;

//     return asin(sin_elev) * 180.0 / M_PI;
// }

// // ============================================================================
// // noonSharpness
// // Rate of change of solar elevation at noon, degrees per hour.
// // = cos(lat) * cos(decl) * 15  (15 = Earth rotation degrees/hour)
// // High value → sharp noon peak → stable proportional time
// // Zero       → flat arc → time undefined (geographic pole at solstice)
// // ============================================================================
// static double noonSharpness(double lat_deg, double decl_deg) {
//     double lat_rad  = lat_deg  * M_PI / 180.0;
//     double decl_rad = decl_deg * M_PI / 180.0;
//     return fabs(cos(lat_rad) * cos(decl_rad) * 15.0);
// }

// // ============================================================================
// // findSolarNoon
// // Finds the SI unix time of the solar noon (elevation peak) nearest to
// // search_centre within a ±24h window.
// //
// // Strategy:
// //   1. Coarse scan: 10-minute steps across ±24h  (~288 elevation calls)
// //   2. Fine scan:   1-second steps across ±30min around coarse peak (~3600 calls)
// //
// // At the poles in summer the elevation curve is nearly flat — the coarse scan
// // lands near the centre of the flat region which is the honest noon estimate.
// // Returns the SI time of the peak, and optionally its elevation and sharpness
// // via output pointers (pass NULL to ignore).
// // ============================================================================
// static time_t findSolarNoon(time_t      search_centre,
//                             double      lat_deg,
//                             double      lon_deg,
//                             double     *out_elevation,
//                             double     *out_sharpness) {
//     // Coarse scan: ±144 steps of 600s = ±24h
//     time_t best_t    = search_centre;
//     double best_elev = -91.0;

//     // for (int step = -144; step <= 144; step++) {
//     for (int step = -72; step <= 72; step++) {
//         time_t t    = search_centre + (time_t)(step * 600);
//         double elev = solarElevation(t, lat_deg, lon_deg);
//         if (elev > best_elev) {
//             best_elev = elev;
//             best_t    = t;
//         }
//     }

//     // Fine scan: ±1800 steps of 1s = ±30min around coarse peak
//     time_t fine_t    = best_t;
//     double fine_elev = best_elev;

//     for (int s = -1800; s <= 1800; s++) {
//         time_t t    = best_t + (time_t)s;
//         double elev = solarElevation(t, lat_deg, lon_deg);
//         if (elev > fine_elev) {
//             fine_elev = elev;
//             fine_t    = t;
//         }
//     }

//     if (out_elevation) *out_elevation = fine_elev;

//     if (out_sharpness) {
//         // Compute declination at found noon for sharpness calculation
//         struct tm ut;
//         gmtime_r(&fine_t, &ut);
//         double decl = solarDeclination(ut.tm_yday + 1);
//         *out_sharpness = noonSharpness(lat_deg, decl);
//     }

//     return fine_t;
// }

// // ============================================================================
// // findNextDistinguishableNoon
// // Starting from last_noon, walks forward in ~86400s steps looking for the
// // next solar noon whose sharpness exceeds NOON_MERGE_THRESHOLD.
// //
// // At normal latitudes this returns the very next SI day's noon.
// // At high latitudes during polar day/night, this may skip many SI days —
// // the returned noon is the next one that forms a meaningful day boundary.
// // The span last_noon → returned value is one geo-positional day.
// //
// // Cap: will not scan beyond NOON_SCAN_CAP_SEC (200 days) from last_noon.
// // If no distinguishable noon is found within the cap, returns
// // last_noon + NOON_SCAN_CAP_SEC and sets out_sharpness = 0 (anomaly).
// // ============================================================================
// static time_t findNextDistinguishableNoon(time_t  last_noon,
//                                           double  lat_deg,
//                                           double  lon_deg,
//                                           double *out_sharpness) {
//     time_t cap      = last_noon + (time_t)NOON_SCAN_CAP_SEC;
//     time_t search   = last_noon + 86400;   // start one SI day ahead

//     while (search < cap) {
//         double elev, sharpness;
//         time_t candidate = findSolarNoon(search, lat_deg, lon_deg,
//                                          &elev, &sharpness);

//         if (sharpness >= NOON_MERGE_THRESHOLD) {
//             if (out_sharpness) *out_sharpness = sharpness;
//             return candidate;
//         }

//         // Noon too flat — merge: advance one more SI day and keep scanning
//         search += 86400;
//     }

//     // Cap reached — pole or extreme case, time undefined
//     if (out_sharpness) *out_sharpness = 0.0;
//     return cap;
// }

// // ============================================================================
// // updateGeoPositionalTime
// // ============================================================================
// void updateGeoPositionalTime(void) {

//     // -------------------------------------------------------------------------
//     // 1. Build UTC time_t from RTC (RTC always holds UTC)
//     // -------------------------------------------------------------------------
//     struct tm utc_tm = {0};
//     utc_tm.tm_year  = satioData.rtc_year   - LAST_EPOCH;
//     utc_tm.tm_mon   = satioData.rtc_month  - 1;
//     utc_tm.tm_mday  = satioData.rtc_mday;
//     utc_tm.tm_hour  = satioData.rtc_hour;
//     utc_tm.tm_min   = satioData.rtc_minute;
//     utc_tm.tm_sec   = satioData.rtc_second;
//     utc_tm.tm_isdst = 0;
//     time_t utc_sec  = mktime(&utc_tm);

//     double lat = satioData.system_degrees_latitude;
//     double lon = satioData.system_degrees_longitude;

//     // -------------------------------------------------------------------------
//     // 2. Mean solar (geo-positional) time — existing behaviour, unchanged
//     //    Longitude offset only, no latitude factor.
//     //    Day boundary: midnight. Clock: standard 00:00-23:59.
//     // -------------------------------------------------------------------------
//     double lon_offset_sec_f = lon * 240.0;
//     time_t lon_offset_sec   = (time_t)lon_offset_sec_f;
//     time_t geo_sec          = utc_sec + lon_offset_sec;

//     struct tm geo_tm;
//     gmtime_r(&geo_sec, &geo_tm);

//     satioData.geo_positional_hour   = (double)geo_tm.tm_hour;
//     satioData.geo_positional_minute = (double)geo_tm.tm_min;
//     satioData.geo_positional_second = (double)geo_tm.tm_sec;
//     satioData.geo_positional_year   = (double)(geo_tm.tm_year + LAST_EPOCH);
//     satioData.geo_positional_month  = (double)(geo_tm.tm_mon  + 1);
//     satioData.geo_positional_day    = (double)geo_tm.tm_mday;

//     // -------------------------------------------------------------------------
//     // 3. Photoperiod — latitude + date, unchanged
//     // -------------------------------------------------------------------------
//     int day_of_year = geo_tm.tm_yday + 1;

//     double decl_deg  = solarDeclination(day_of_year);
//     double decl_rad  = decl_deg * M_PI / 180.0;
//     double lat_rad   = lat      * M_PI / 180.0;
//     double cos_omega = -tan(lat_rad) * tan(decl_rad);

//     double day_hours;
//     if      (cos_omega <= -1.0) day_hours = 24.0;
//     else if (cos_omega >=  1.0) day_hours =  0.0;
//     else {
//         double omega_deg = acos(cos_omega) * 180.0 / M_PI;
//         day_hours = 2.0 * omega_deg / 15.0;
//     }
//     double night_hours = 24.0 - day_hours;

//     double abs_lat = fabs(lat);
//     double anomaly_photo = (abs_lat > 66.5)
//                          ? (abs_lat - 66.5) / (90.0 - 66.5)
//                          : 0.0;

//     double sunrise_h = 12.0 - (day_hours / 2.0);
//     double sunset_h  = 12.0 + (day_hours / 2.0);

//     satioData.geo_positional_day_hours   = hoursToHHMM(day_hours);
//     satioData.geo_positional_night_hours = hoursToHHMM(night_hours);
//     satioData.geo_positional_anomaly     = anomaly_photo;
//     satioData.geo_positional_sunrise     = hoursToHHMM(sunrise_h);
//     satioData.geo_positional_sunset      = hoursToHHMM(sunset_h);

//     // -------------------------------------------------------------------------
//     // 4. Noon cache — recompute if stale or position moved > 1km
//     // -------------------------------------------------------------------------
//     double dist_from_cache = approxDistanceMetres(lat, lon,
//                                                    satioData.geo_cache_lat,
//                                                    satioData.geo_cache_lon);
//     double age_sec = difftime(utc_sec, satioData.geo_cache_last_updated);

//     bool cache_stale = (age_sec  > CACHE_INTERVAL_SEC)
//                     || (dist_from_cache > CACHE_DISTANCE_METRES)
//                     || (satioData.geo_noon_last == 0);  // first run

//     if (cache_stale) {
//         // Find last noon: search centred 12h before now so we look backward
//         double sharpness_last, sharpness_next;

//         time_t noon_last = findSolarNoon(utc_sec - 43200, lat, lon,
//                                          NULL, &sharpness_last);

//         // If found noon is actually in the future, step back one SI day
//         if (noon_last > utc_sec) {
//             noon_last = findSolarNoon(utc_sec - 43200 - 86400, lat, lon,
//                                       NULL, &sharpness_last);
//         }

//         // Find next distinguishable noon (may span many SI days at poles)
//         time_t noon_next = findNextDistinguishableNoon(noon_last, lat, lon,
//                                                         &sharpness_next);

//         satioData.geo_noon_last           = noon_last;
//         satioData.geo_noon_next           = noon_next;
//         satioData.geo_noon_sharpness_last = sharpness_last;
//         satioData.geo_noon_sharpness_next = sharpness_next;
//         satioData.geo_cache_last_updated  = utc_sec;
//         satioData.geo_cache_lat           = lat;
//         satioData.geo_cache_lon           = lon;
//     }

//     // -------------------------------------------------------------------------
//     // 5. Proportional day computation (Option C)
//     //
//     //    One geo-positional day = last distinguishable noon → next.
//     //    This span may be ~86400s (normal) or weeks/months (polar).
//     //
//     //    Clock runs 00:00:00 at noon, counts up proportionally.
//     //    One proportional second = span_SI / 86400.
//     //    Proportional hours/minutes/seconds are fractions of the span.
//     //
//     //    Wobble = uncertainty in proportional second, derived from sharpness.
//     //    Anomaly = 0.0 (normal) → 1.0 (pole, time undefined).
//     // -------------------------------------------------------------------------
//     double span_si      = difftime(satioData.geo_noon_next,
//                                    satioData.geo_noon_last);
//     double elapsed_si   = difftime(utc_sec, satioData.geo_noon_last);

//     // Clamp elapsed to [0, span] — guards against cache edge cases
//     if (elapsed_si < 0.0)       elapsed_si = 0.0;
//     if (elapsed_si > span_si)   elapsed_si = span_si;

//     double day_fraction = (span_si > 0.0) ? (elapsed_si / span_si) : 0.0;

//     // Proportional second duration in SI seconds
//     double prop_second_si = (span_si > 0.0) ? (span_si / 86400.0) : 1.0;

//     // Proportional elapsed seconds, minutes, hours within this geo-day
//     double prop_elapsed_seconds_total = elapsed_si / prop_second_si;
//     int    prop_h = (int)(prop_elapsed_seconds_total / 3600.0);
//     int    prop_m = (int)((prop_elapsed_seconds_total - prop_h * 3600.0) / 60.0);
//     int    prop_s = (int)(prop_elapsed_seconds_total
//                         - prop_h * 3600.0 - prop_m * 60.0);

//     // Wobble: how uncertain is prop_second_si due to noon flatness?
//     // As sharpness → 0, wobble → prop_second_si (the entire second is uncertain)
//     double max_sharpness = 15.0;  // cos(0)*cos(0)*15 = equator on equinox
//     double sharpness_now = satioData.geo_noon_sharpness_last;
//     double wobble_factor = 1.0 - (sharpness_now / max_sharpness);
//     if (wobble_factor < 0.0) wobble_factor = 0.0;
//     if (wobble_factor > 1.0) wobble_factor = 1.0;
//     double wobble_si = wobble_factor * prop_second_si;

//     // // Absolute anomaly: 0.0 normal, 1.0 pole undefined
//     // // Driven by sharpness — when sharpness = 0 anomaly = 1
//     // double abs_anomaly = (sharpness_now < max_sharpness)
//     //                    ? (1.0 - sharpness_now / max_sharpness)
//     //                    : 0.0;
    
//     // = 1.0 - (8.64 / 15.0) = 0.424
//     // double abs_anomaly = 1.0 - (sharpness_now / max_sharpness);
//     // if (abs_anomaly < 0.0) abs_anomaly = 0.0;
//     // if (abs_anomaly > 1.0) abs_anomaly = 1.0;


//     #define SHARPNESS_ARCTIC    5.98   // cos(66.5°)*cos(0°)*15

//     double abs_anomaly = (sharpness_now < SHARPNESS_ARCTIC)
//                       ? (1.0 - sharpness_now / SHARPNESS_ARCTIC)
//                       : 0.0;
//     if (abs_anomaly < 0.0) abs_anomaly = 0.0;
//     if (abs_anomaly > 1.0) abs_anomaly = 1.0;

//     // Polar status string
//     const char *polar_status = "";
//     if (sharpness_now < NOON_MERGE_THRESHOLD) {
//         if (day_hours >= 24.0)       polar_status = "POLAR DAY";
//         else if (day_hours <= 0.0)   polar_status = "POLAR NIGHT";
//         else                         polar_status = "UNDEFINED";
//     }

//     // Store proportional day values
//     satioData.geo_abs_day_span_sec    = span_si;
//     satioData.geo_abs_day_elapsed_sec = elapsed_si;
//     satioData.geo_abs_day_fraction    = day_fraction;
//     satioData.geo_abs_prop_second_si  = prop_second_si;
//     satioData.geo_abs_prop_hour       = (double)prop_h;
//     satioData.geo_abs_prop_minute     = (double)prop_m;
//     satioData.geo_abs_prop_second     = (double)prop_s;
//     satioData.geo_abs_wobble          = wobble_si;
//     satioData.geo_abs_anomaly         = abs_anomaly;
//     satioData.geo_abs_noon_sharpness  = sharpness_now;

//     // -------------------------------------------------------------------------
//     // 6. Geo-positional calendar (J2000.0 epoch, vernal equinox = month 1)
//     //
//     //    Primary value: elapsed tropical years from J2000.0
//     //    Year position: fractional position 0.0-1.0 within tropical year
//     //                   0.00 = vernal equinox  (March,     month 1)
//     //                   0.25 = summer solstice (June,      month 4)
//     //                   0.50 = autumnal equinox(September, month 7)
//     //                   0.75 = winter solstice (December,  month 10)
//     //
//     //    Month: 1-12 from vernal equinox (no fixed day counts —
//     //           each month is 1/12 of the tropical year)
//     //    Day:   integer solar day within the geo month
//     //           counted from last noon boundary within that month's window
//     // -------------------------------------------------------------------------
//     double elapsed_from_j2000  = (utc_sec - J2000_UNIX) / TROPICAL_YEAR_SEC;
//     // double year_j2000          = elapsed_from_j2000;

//     // // Fractional position within current tropical year
//     // // (day_of_year + DECL_DOY_OFFSET) / 365.24219 gives position
//     // // where 0.0 = vernal equinox
//     // double year_position = fmod((day_of_year + DECL_DOY_OFFSET)
//     //                             / 365.24219, 1.0);
//     // if (year_position < 0.0) year_position += 1.0;

//     double year_j2000    = (utc_sec - J2000_UNIX) / TROPICAL_YEAR_SEC;
//     double year_position = fmod(year_j2000, 1.0);
//     if (year_position < 0.0) year_position += 1.0;

//     // Solar quarter (0=spring 1=summer 2=autumn 3=winter)
//     int solar_quarter = 0;
//     for (int q = 3; q >= 0; q--) {
//         if (year_position >= QUARTER_FRACTIONS[q]) {
//             solar_quarter = q;
//             break;
//         }
//     }

//     // Geo month: 12 equal divisions of tropical year from vernal equinox
//     // Month 1 starts at year_position 0.0 (vernal equinox, March)
//     int    geo_cal_month = (int)(year_position * 12.0) + 1;
//     if (geo_cal_month > 12) geo_cal_month = 12;

//     // Geo day: position within the current geo month
//     // Each geo month = 365.24219 / 12 = 30.437 SI days
//     double month_length_days = 365.24219 / 12.0;
//     double pos_in_month      = fmod(year_position * 12.0, 1.0);
//     int    geo_cal_day       = (int)(pos_in_month * month_length_days) + 1;

//     // Days to next quarter boundary
//     double next_quarter_frac  = QUARTER_FRACTIONS[(solar_quarter + 1) % 4];
//     if (next_quarter_frac <= year_position)
//         next_quarter_frac += 1.0;  // wraps across year boundary
//     double days_to_next_quarter = (next_quarter_frac - year_position)
//                                 * 365.24219;

//     satioData.geo_abs_year_j2000         = year_j2000;
//     satioData.geo_abs_year_position      = year_position;
//     satioData.geo_abs_cal_month          = geo_cal_month;
//     satioData.geo_abs_cal_day            = geo_cal_day;
//     satioData.geo_abs_solar_quarter      = solar_quarter;
//     satioData.geo_abs_days_to_next_quarter = days_to_next_quarter;

//     // -------------------------------------------------------------------------
//     // 7. Format existing mean solar strings (preserved, unchanged)
//     // -------------------------------------------------------------------------
//     char hour_str[3], min_str[3], sec_str[3];
//     padDigitsZero(satioData.geo_positional_hour,   hour_str, sizeof(hour_str));
//     padDigitsZero(satioData.geo_positional_minute, min_str,  sizeof(min_str));
//     padDigitsZero(satioData.geo_positional_second, sec_str,  sizeof(sec_str));

//     memset(satioData.formatted_geo_positional_time, 0,
//            sizeof(satioData.formatted_geo_positional_time));
//     snprintf(satioData.formatted_geo_positional_time,
//              sizeof(satioData.formatted_geo_positional_time),
//              "%s:%s:%s", hour_str, min_str, sec_str);

//     char day_str[3], month_str[3], year_str[5];
//     padDigitsZero(satioData.geo_positional_day,   day_str,   sizeof(day_str));
//     padDigitsZero(satioData.geo_positional_month, month_str, sizeof(month_str));
//     padDigitsZero(satioData.geo_positional_year,  year_str,  sizeof(year_str));

//     memset(satioData.formatted_geo_positional_date_DDMMYYYY, 0,
//            sizeof(satioData.formatted_geo_positional_date_DDMMYYYY));
//     snprintf(satioData.formatted_geo_positional_date_DDMMYYYY,
//              sizeof(satioData.formatted_geo_positional_date_DDMMYYYY),
//              "%s/%s/%s", day_str, month_str, year_str);

//     char short_year_str[3] = { year_str[2], year_str[3], '\0' };
//     memset(satioData.formatted_geo_positional_short_date_DDMMYY, 0,
//            sizeof(satioData.formatted_geo_positional_short_date_DDMMYY));
//     snprintf(satioData.formatted_geo_positional_short_date_DDMMYY,
//              sizeof(satioData.formatted_geo_positional_short_date_DDMMYY),
//              "%s/%s/%s", day_str, month_str, short_year_str);

//     memset(satioData.padded_geo_positional_time_HHMMSS, 0,
//            sizeof(satioData.padded_geo_positional_time_HHMMSS));
//     snprintf(satioData.padded_geo_positional_time_HHMMSS,
//              sizeof(satioData.padded_geo_positional_time_HHMMSS),
//              "%s%s%s", hour_str, min_str, sec_str);

//     memset(satioData.padded_geo_positional_date_DDMMYYYY, 0,
//            sizeof(satioData.padded_geo_positional_date_DDMMYYYY));
//     snprintf(satioData.padded_geo_positional_date_DDMMYYYY,
//              sizeof(satioData.padded_geo_positional_date_DDMMYYYY),
//              "%s%s%s", day_str, month_str, year_str);

//     // -------------------------------------------------------------------------
//     // 8. Format new absolute (proportional + calendar) strings
//     // -------------------------------------------------------------------------

//     // Proportional time: HH:MM:SS from solar noon
//     // Hours may exceed 23 for extended polar days — no upper clamp
//     char abs_hour_str[8], abs_min_str[3], abs_sec_str[3];
//     snprintf(abs_hour_str, sizeof(abs_hour_str), "%02d", prop_h);
//     snprintf(abs_min_str,  sizeof(abs_min_str),  "%02d", prop_m);
//     snprintf(abs_sec_str,  sizeof(abs_sec_str),  "%02d", prop_s);

//     memset(satioData.formatted_geo_abs_prop_time, 0,
//            sizeof(satioData.formatted_geo_abs_prop_time));
//     snprintf(satioData.formatted_geo_abs_prop_time,
//              sizeof(satioData.formatted_geo_abs_prop_time),
//              "%s:%s:%s", abs_hour_str, abs_min_str, abs_sec_str);

//     // Geo calendar date: DD/MM (geo) / YY (J2000 elapsed, 2-digit)
//     char geo_day_str[12], geo_mon_str[12], geo_yr_str[12];
//     snprintf(geo_day_str, sizeof(geo_day_str), "%02d", geo_cal_day);
//     snprintf(geo_mon_str, sizeof(geo_mon_str), "%02d", geo_cal_month);
//     // J2000 year: integer elapsed tropical years, shown as 2-digit offset
//     int j2000_year_int = (int)year_j2000;
//     snprintf(geo_yr_str, sizeof(geo_yr_str), "%02d", j2000_year_int);

//     memset(satioData.formatted_geo_abs_cal_date, 0,
//            sizeof(satioData.formatted_geo_abs_cal_date));
//     snprintf(satioData.formatted_geo_abs_cal_date,
//              sizeof(satioData.formatted_geo_abs_cal_date),
//              "%s/%s/J%s", geo_day_str, geo_mon_str, geo_yr_str);

//     // Full decimal year string: "26.4763"
//     memset(satioData.geo_abs_year_decimal, 0,
//            sizeof(satioData.geo_abs_year_decimal));
//     snprintf(satioData.geo_abs_year_decimal,
//              sizeof(satioData.geo_abs_year_decimal),
//              "%.4f", year_j2000);

//     // Quarter string
//     memset(satioData.geo_abs_quarter_str, 0,
//            sizeof(satioData.geo_abs_quarter_str));
//     snprintf(satioData.geo_abs_quarter_str,
//              sizeof(satioData.geo_abs_quarter_str),
//              "%s", QUARTER_NAMES[solar_quarter]);

//     // Polar status string
//     memset(satioData.geo_abs_polar_status, 0,
//            sizeof(satioData.geo_abs_polar_status));
//     snprintf(satioData.geo_abs_polar_status,
//              sizeof(satioData.geo_abs_polar_status),
//              "%s", polar_status);

//     // -------------------------------------------------------------------------
//     // 9. Console output
//     // -------------------------------------------------------------------------
//     printf("Geo-pos mean        : %s  %s\n",
//            satioData.formatted_geo_positional_time,
//            satioData.formatted_geo_positional_date_DDMMYYYY);

//     printf("Photoperiod         : day=%05.2f  night=%05.2f  "
//            "sunrise=%05.2f  sunset=%05.2f  anomaly=%.4f%s%s\n",
//            satioData.geo_positional_day_hours,
//            satioData.geo_positional_night_hours,
//            satioData.geo_positional_sunrise,
//            satioData.geo_positional_sunset,
//            anomaly_photo,
//            cos_omega <= -1.0 ? "  [POLAR DAY]"   :
//            cos_omega >=  1.0 ? "  [POLAR NIGHT]" : "",
//            "");

//     printf("Geo-pos abs time    : %s  (prop_sec=%.6fs  wobble=±%.6fs  "
//            "sharpness=%.4f°/h)\n",
//            satioData.formatted_geo_abs_prop_time,
//            prop_second_si,
//            wobble_si,
//            sharpness_now);

//     printf("Geo-pos abs date    : %s  %s  next_quarter=%.1fd  "
//            "year_pos=%.4f%s%s\n",
//            satioData.formatted_geo_abs_cal_date,
//            satioData.geo_abs_quarter_str,
//            days_to_next_quarter,
//            year_position,
//            polar_status[0] ? "  [" : "",
//            polar_status[0] ? (strcat(satioData.geo_abs_polar_status, "]"),
//                               satioData.geo_abs_polar_status) : "");

//     printf("Geo-pos abs year    : J%s  (day_span=%.1fs  "
//            "anomaly=%.4f)\n",
//            satioData.geo_abs_year_decimal,
//            span_si,
//            abs_anomaly);
// }

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
    // Serial.printf("[writeI2C] storeRTCSYNCTime\n");
    // xSemaphoreTake(i2c_bus0_mutex, 1000 / portTICK_PERIOD_MS);
    satioData.rtcsync_unixtime = rtc.now().unixtime();
    // xSemaphoreGive(i2c_bus0_mutex);

    // Format sync time (HH:MM:SS)
    char hour_str[3], min_str[3], sec_str[3];
    padDigitsZero(satioData.rtcsync_hour, hour_str, sizeof(hour_str));
    padDigitsZero(satioData.rtcsync_minute, min_str, sizeof(min_str));
    padDigitsZero(satioData.rtcsync_second, sec_str, sizeof(sec_str));
    memset(satioData.formatted_rtc_sync_time, 0, sizeof(satioData.formatted_rtc_sync_time));
    snprintf(satioData.formatted_rtc_sync_time, sizeof(satioData.formatted_rtc_sync_time), "%s:%s:%s", hour_str, min_str, sec_str);

    char day_str[3], month_str[3], year_str[5];
    padDigitsZero(satioData.rtcsync_day, day_str, sizeof(day_str));
    padDigitsZero(satioData.rtcsync_month, month_str, sizeof(month_str));
    padDigitsZero(satioData.rtcsync_year, year_str, sizeof(year_str));

    memset(satioData.formatted_rtc_sync_date_DDMMYYYY, 0, sizeof(satioData.formatted_rtc_sync_date_DDMMYYYY));
    snprintf(satioData.formatted_rtc_sync_date_DDMMYYYY, sizeof(satioData.formatted_rtc_sync_date_DDMMYYYY), "%s/%s/%s", day_str, month_str, year_str);

    char short_year_str[3] = { year_str[2], year_str[3], '\0' };
    memset(satioData.formatted_rtc_sync_short_date_DDMMYY, 0, sizeof(satioData.formatted_rtc_sync_short_date_DDMMYY));
    snprintf(satioData.formatted_rtc_sync_short_date_DDMMYY, sizeof(satioData.formatted_rtc_sync_short_date_DDMMYY), "%s/%s/%s", day_str, month_str, short_year_str);

    memset(satioData.padded_rtc_sync_time_HHMMSS, 0, sizeof(satioData.padded_rtc_sync_time_HHMMSS));
    snprintf(satioData.padded_rtc_sync_time_HHMMSS, sizeof(satioData.padded_rtc_sync_time_HHMMSS), "%s%s%s", hour_str, min_str, sec_str);

    memset(satioData.padded_rtc_sync_date_DDMMYYYY, 0, sizeof(satioData.padded_rtc_sync_date_DDMMYYYY));
    snprintf(satioData.padded_rtc_sync_date_DDMMYYYY, sizeof(satioData.padded_rtc_sync_date_DDMMYYYY), "%s%s%s", day_str, month_str, year_str);

    // RTC sync latitude
    char temp_str[MAX_GLOBAL_ELEMENT_SIZE];
    snprintf(temp_str, MAX_GLOBAL_ELEMENT_SIZE, "%.7f", satioData.degrees_latitude);
    memset(satioData.rtcsync_latitude, 0, sizeof(satioData.rtcsync_latitude));
    strcpy(satioData.rtcsync_latitude, temp_str);

    // RTC sync longitude
    snprintf(temp_str, MAX_GLOBAL_ELEMENT_SIZE, "%.7f", satioData.degrees_longitude);
    memset(satioData.rtcsync_longitude, 0, sizeof(satioData.rtcsync_longitude));
    strcpy(satioData.rtcsync_longitude, temp_str);

    // RTC sync altitude
    memset(satioData.rtcsync_altitude, 0, sizeof(satioData.rtcsync_altitude));
    strcpy(satioData.rtcsync_altitude, gnggaData.altitude);
}

// ----------------------------------------------------------------------------------------
// setSystemTime.
// ----------------------------------------------------------------------------------------
void setSystemTime(long usec) {
  // -----------------------------------------------------
  // System time = Local Time.
  // -----------------------------------------------------
  struct tm tmpti = {0};
  memset(&tmpti, 0, sizeof(tmpti));
  // Serial.printf("[writeI2C] setSystemTime\n");
  // xSemaphoreTake(i2c_bus0_mutex, 1000 / portTICK_PERIOD_MS);
  tmpti.tm_year = int(rtc.now().year()) - LAST_EPOCH; // Years since 1900 (since last epoch)
  tmpti.tm_mon = rtc.now().month() - 1; // Months 0-11
  tmpti.tm_mday = rtc.now().day();
  tmpti.tm_hour = rtc.now().hour();
  tmpti.tm_min = rtc.now().minute();
  tmpti.tm_sec = rtc.now().second();
  // xSemaphoreGive(i2c_bus0_mutex);
  tmpti.tm_isdst = -1; // No DST
  time_t now = mktime(&tmpti);
  tv_now = {
      .tv_sec = now + satioData.utc_second_offset, // negative utc_second_offset will be deducted.
      .tv_usec = usec
  };
  if (settimeofday(&tv_now, NULL) == 0) {}
  else {Serial.println("[settimeofday] failed");}
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
  gettimeofday(&tv_now, NULL);
  timeinfo = localtime(&tv_now.tv_sec); // Assumes localtime works
  // --------------------------------------------------------
  // Keep this function quick by only storing unixtime uS.
  // --------------------------------------------------------
  satioData.local_unixtime_uS = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
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

void setRTCDateTime() {
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

void syncRTC() {
  // Set GPS sync flag false
  if (satioData.gps_sync) {
    if
      (
      satioData.local_unixtime_uS >= gps_sync_timestamp + GPS_SYNC_TIMEOUT_uS
      ||
      satioData.local_unixtime_uS < gps_sync_timestamp
      )
    {
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
    printf("[rtc] sync 2: %s\n", String(rtc.now().timestamp()).c_str());
  }

  /**
   * Automatically set RTC datetime with GPS data.
   */
  else if (satioData.set_time_automatically==true) {
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
    // Serial.println("[setSatIOData]");
      syncRTC();
      setSatioCoordinates();
      setSatIOAltitude();
      setSatIOSspeed();
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
  while (rtc_second_now==rtc.now().second()) // wait to sync
  setSystemTime(0);
  getSystemTime();
  storeLocalTime();
  storeRTCTime();
  Serial.println("[SYNC] RTC datetime:    " +
                  String(satioData.padded_rtc_time_HHMMSS) + " " +
                  String(satioData.padded_rtc_date_DDMMYYYY));
  Serial.println("[SYNC] system datetime: " +
                 String(satioData.padded_local_time_HHMMSS) + " " +
                 String(satioData.padded_local_date_DDMMYYYY) +
                 " (+- offset seconds " +
                 String(satioData.utc_second_offset) + ")");
}

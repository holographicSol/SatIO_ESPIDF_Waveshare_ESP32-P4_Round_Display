/**
 * @file LMST.h Written By Benjamin Jack Cullen.
 * @brief Various functions for:
 *       - Calculating LMST (Local Mean Solar Time)
 *       - Creating a photo period schedule based on LMST (Local Mean Solar Time), latitude, and longitude.
 *       - Further development may include photo periods for other time standards, converted from LMST.
 */

#include <Arduino.h>
#include "./LMST.h"
#include <time.h>

// Twilight zone solar altitude ranges (degrees)
float tzonesar[MAX_TWILIGHT_ZONES][2]={
    {  6.0f,  999.0f }, // 0 Full Daylight
    {  0.0f,    6.0f }, // 1 Golden Hour
    {  0.0f,    0.0f }, // 2 Sunrise/Sunset
    { -6.0f,    0.0f }, // 3 Civil Twilight
    {-12.0f,   -6.0f }, // 4 Nautical Twilight
    {-18.0f,  -12.0f }, // 5 Astronomical Twilight
    {-999.0f, -18.0f }  // 6 Astronomical Night
};

char twilight_zone_names[MAX_TWILIGHT_ZONES][22]={
    "Full Daylight",
    "Golden Hour",
    "Sunrise/Sunset",
    "Civil Twilight",
    "Nautical Twilight",
    "Astronomical Twilight",
    "Astronomical Night"
};

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

// ──────────────────────────────────────────────────────────────────────────────
// sunCrossingMorning / sunCrossingEvening
// Returns the HH.MM solar time at which the sun crosses alt_deg (ascending /
// descending). Returns -1.0 (N/A) when the sun never reaches that altitude.
// ──────────────────────────────────────────────────────────────────────────────
static inline double sunCrossingMorning(float alt_deg, double lat_rad, double decl_rad) {
    double denom = cos(lat_rad) * cos(decl_rad);
    if (fabs(denom) < 1e-9) return NAN;
    double cos_ha = (sin((double)alt_deg * M_PI / 180.0) - sin(lat_rad) * sin(decl_rad)) / denom;
    if (cos_ha < -1.0 || cos_ha > 1.0) return NAN;
    return hoursToHHMM(12.0 - (acos(cos_ha) * 180.0 / M_PI) / 15.0);
}

static inline double sunCrossingEvening(float alt_deg, double lat_rad, double decl_rad) {
    double denom = cos(lat_rad) * cos(decl_rad);
    if (fabs(denom) < 1e-9) return NAN;
    double cos_ha = (sin((double)alt_deg * M_PI / 180.0) - sin(lat_rad) * sin(decl_rad)) / denom;
    if (cos_ha < -1.0 || cos_ha > 1.0) return NAN;
    return hoursToHHMM(12.0 + (acos(cos_ha) * 180.0 / M_PI) / 15.0);
}

/**
 * @brief Get the current twilight zone index based on the sun's altitude in degrees.
*/
int getCurrentTwilightZoneIndex(float sunAltDeg) {
  for (int i=0; i<MAX_TWILIGHT_ZONES; i++) {
    if (sunAltDeg >= tzonesar[i][0] && sunAltDeg <= tzonesar[i][1]) {
      return i;
    }
  }
  return -1; // Return -1 if no matching twilight zone is found
}

// ----------------------------------------------------------------------------------------
// makeLMST.
// ----------------------------------------------------------------------------------------
tm makeLMST(
    uint16_t utc_year, uint8_t utc_month, uint8_t utc_day,
    uint8_t utc_hour, uint8_t utc_minute, uint8_t utc_second,
    double latitude_degrees,
    double longitude_degrees)
    {
    // Build UTC time_t from stored RTC values (RTC always holds UTC).
    struct tm utc_tm = {0};
    utc_tm.tm_year  = utc_year  - LAST_EPOCH;
    utc_tm.tm_mon   = utc_month - 1;
    utc_tm.tm_mday  = utc_day;
    utc_tm.tm_hour  = utc_hour;
    utc_tm.tm_min   = utc_minute;
    utc_tm.tm_sec   = utc_second;
    utc_tm.tm_isdst = 0;
    time_t utc_sec  = mktime(&utc_tm);

    // True solar time offset is purely longitudinal — no latitude factor.
    double lon_offset_sec_f = longitude_degrees * 240.0;
    time_t lon_offset_sec   = (time_t)lon_offset_sec_f;

    // LMST unix time.
    time_t lmst_sec = utc_sec + lon_offset_sec;

    // Decompose into calendar fields.
    struct tm lmst_tm;
    gmtime_r(&lmst_sec, &lmst_tm);

    return lmst_tm;
}

PhotoPeriodSchedulele getPhotoPeriodData(
    double latitude_degrees,
    double longitude_degrees,
    tm lmst_tm,
    uint8_t LMST_hour,
    uint8_t LMST_minute,
    uint8_t LMST_second
    )
    {

    PhotoPeriodSchedulele schedule = {0};
    // ------------------------------------------------------------------------
    // Photoperiod calculation.
    // Day of year (1-365) from the LMST date.
    // ------------------------------------------------------------------------
    int day_of_year = lmst_tm.tm_yday + 1;  // tm_yday is 0-based

    // Solar declination (degrees): where the sun sits relative to the equator.
    // +23.45° at summer solstice, -23.45° at winter solstice.
    double decl_deg = -23.45 * cos((360.0 / 365.0) * (day_of_year + 10) * M_PI / 180.0);
    double decl_rad = decl_deg * M_PI / 180.0;

    double lat_rad  = latitude_degrees * M_PI / 180.0;

    // Hour angle at sunrise/sunset: arccos(-tan(lat) * tan(decl)).
    // The argument can exceed ±1 at the poles — clamp to handle polar day/night.
    double cos_omega = -tan(lat_rad) * tan(decl_rad);

    double day_hours;

    // Sun never sets — polar day (midnight sun).
    if (cos_omega <= -1.0) {day_hours = 24.0;}
    
    // Sun never rises — polar night.
    else if (cos_omega >= 1.0) {day_hours = 0.0;}
    
    else {
        double omega_deg = acos(cos_omega) * 180.0 / M_PI;
        day_hours = 2.0 * omega_deg / 15.0;
    }

    double night_hours = 24.0 - day_hours;

    // Anomaly flag: how far into polar conditions we are.
    // 0.0 = normal photoperiod, 1.0 = fully at the pole.
    double abs_lat = fabs(latitude_degrees);
    double anomaly = 0.0;
    if (abs_lat > 66.5) {anomaly = (abs_lat - 66.5) / (90.0 - 66.5);}

    // Store photoperiod data in satioData structure.
    schedule.LMST_day_hours   = hoursToHHMM(day_hours);
    schedule.LMST_night_hours = hoursToHHMM(night_hours);
    schedule.LMST_anomaly     = anomaly;

    // printf("Photoperiod         : daylight_hours=%05.2f  night_hours=%05.2f anomaly=%.4f%s\n",
    //        schedule.LMST_day_hours,
    //        schedule.LMST_night_hours,
    //        schedule.LMST_anomaly,
    //        cos_omega <= -1.0 ? "  [POLAR DAY]"  :
    //        cos_omega >=  1.0 ? "  [POLAR NIGHT]" : "");
    

    // ──────────────────────────────────────────────────────────────────────────
    // Twilight stage: classify current sun altitude
    // LMST means solar noon == 12:00
    // ──────────────────────────────────────────────────────────────────────────
    double solar_hour = (double)LMST_hour
                      + (double)LMST_minute / 60.0
                      + (double)LMST_second / 3600.0;
    double ha_now_rad = (solar_hour - 12.0) * 15.0 * M_PI / 180.0;
    double sin_alt_now = sin(lat_rad) * sin(decl_rad)
                       + cos(lat_rad) * cos(decl_rad) * cos(ha_now_rad);
    // clamp for asin domain safety
    if (sin_alt_now >  1.0) sin_alt_now =  1.0;
    if (sin_alt_now < -1.0) sin_alt_now = -1.0;
    float sun_alt_deg = (float)(asin(sin_alt_now) * 180.0 / M_PI);

    // ──────────────────────────────────────────────────────────────────────────
    // Twilight schedule: crossing times (HH.MM) for each of the 10 zones.
    // dusk_start / dusk_end : evening entry / exit of zone (sun descending).
    // dawn_start / dawn_end : morning entry / exit of zone (sun ascending).
    // -1.0 = N/A (sun never crosses that altitude, or no finite bound).
    // ──────────────────────────────────────────────────────────────────────────

    // FullDaylight
    // Note reuse of Dawn is intentional (6 degrees to 6 degrees, while retaining 999 degrees for bounds).
    schedule.dawn_start[FullDaylight] = sunCrossingMorning(tzonesar[FullDaylight][Dawn], lat_rad, decl_rad);
    schedule.dawn_end[FullDaylight]   = NAN;
    schedule.dusk_start[FullDaylight] = NAN;
    schedule.dusk_end[FullDaylight]   = sunCrossingEvening(tzonesar[FullDaylight][Dawn], lat_rad, decl_rad);

    // GoldenHour
    schedule.dawn_start[GoldenHour] = sunCrossingMorning(tzonesar[GoldenHour][Dawn], lat_rad, decl_rad);
    schedule.dawn_end[GoldenHour]   = sunCrossingMorning(tzonesar[GoldenHour][Dusk], lat_rad, decl_rad);
    schedule.dusk_start[GoldenHour] = sunCrossingEvening(tzonesar[GoldenHour][Dusk], lat_rad, decl_rad);
    schedule.dusk_end[GoldenHour]   = sunCrossingEvening(tzonesar[GoldenHour][Dawn], lat_rad, decl_rad);

    // Sunset/Sunrise
    schedule.dawn_start[SunriseSunset] = sunCrossingMorning(tzonesar[SunriseSunset][Dawn], lat_rad, decl_rad);
    schedule.dawn_end[SunriseSunset]   = sunCrossingMorning(tzonesar[SunriseSunset][Dusk], lat_rad, decl_rad);
    schedule.dusk_start[SunriseSunset] = sunCrossingEvening(tzonesar[SunriseSunset][Dusk], lat_rad, decl_rad);
    schedule.dusk_end[SunriseSunset]   = sunCrossingEvening(tzonesar[SunriseSunset][Dawn], lat_rad, decl_rad);

    // Civil Twilight
    schedule.dawn_start[CivilTwilight] = sunCrossingMorning(tzonesar[CivilTwilight][Dawn], lat_rad, decl_rad);
    schedule.dawn_end[CivilTwilight]   = sunCrossingMorning(tzonesar[CivilTwilight][Dusk], lat_rad, decl_rad);
    schedule.dusk_start[CivilTwilight] = sunCrossingEvening(tzonesar[CivilTwilight][Dusk], lat_rad, decl_rad);
    schedule.dusk_end[CivilTwilight]   = sunCrossingEvening(tzonesar[CivilTwilight][Dawn], lat_rad, decl_rad);

    // Nautical Twilight
    schedule.dawn_start[NauticalTwilight] = sunCrossingMorning(tzonesar[NauticalTwilight][Dawn], lat_rad, decl_rad);
    schedule.dawn_end[NauticalTwilight]   = sunCrossingMorning(tzonesar[NauticalTwilight][Dusk], lat_rad, decl_rad);
    schedule.dusk_start[NauticalTwilight] = sunCrossingEvening(tzonesar[NauticalTwilight][Dusk], lat_rad, decl_rad);
    schedule.dusk_end[NauticalTwilight]   = sunCrossingEvening(tzonesar[NauticalTwilight][Dawn], lat_rad, decl_rad);

    // Astronomical Twilight
    schedule.dawn_start[AstronomicalTwilight] = sunCrossingMorning(tzonesar[AstronomicalTwilight][Dawn], lat_rad, decl_rad);
    schedule.dawn_end[AstronomicalTwilight]   = sunCrossingMorning(tzonesar[AstronomicalTwilight][Dusk], lat_rad, decl_rad);
    schedule.dusk_start[AstronomicalTwilight] = sunCrossingEvening(tzonesar[AstronomicalTwilight][Dusk], lat_rad, decl_rad);
    schedule.dusk_end[AstronomicalTwilight]   = sunCrossingEvening(tzonesar[AstronomicalTwilight][Dawn], lat_rad, decl_rad);

    // Astronomical Night
    // Note reuse of Dusk is intentional (-18 degrees to -18 degrees, while retaining 999 degrees for bounds).
    schedule.dawn_start[AstronomicalNight] = NAN;
    schedule.dawn_end[AstronomicalNight]   = sunCrossingMorning(tzonesar[AstronomicalNight][Dusk], lat_rad, decl_rad);
    schedule.dusk_start[AstronomicalNight] = sunCrossingEvening(tzonesar[AstronomicalNight][Dusk], lat_rad, decl_rad);
    schedule.dusk_end[AstronomicalNight]   = NAN;

    // printf("Twilight Schedule:\n");
    // for (int i = 0; i < MAX_TWILIGHT_ZONES; ++i) {
    //     const auto& entry = schedule;
    //     printf("  Zone %d: %s | dusk_start=%.2f, dusk_end=%.2f, dawn_start=%.2f, dawn_end=%.2f\n",
    //            i, twilight_zone_names[i], entry.dusk_start[i], entry.dusk_end[i], entry.dawn_start[i], entry.dawn_end[i]);
    // }

    // Set current twilight zone index number
    schedule.current_zone = getCurrentTwilightZoneIndex(sun_alt_deg);
    // printf(
    //   "  Current Zone %d: %s | dusk_start=%.2f, dusk_end=%.2f, dawn_start=%.2f, dawn_end=%.2f\n",
    //   schedule.current_zone,
    //   twilight_zone_names[schedule.current_zone],
    //   schedule.dusk_start[schedule.current_zone],
    //   schedule.dusk_end[schedule.current_zone],
    //   schedule.dawn_start[schedule.current_zone],
    //   schedule.dawn_end[schedule.current_zone]
    // );

    return schedule;
}

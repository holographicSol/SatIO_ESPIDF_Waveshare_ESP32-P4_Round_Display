/**
 * @file LMST.h Written By Benjamin Jack Cullen.
 * @brief Various functions for:
 *       - Calculating LMST (Local Mean Solar Time)
 *       - Creating a photo period schedule based on LMST (Local Mean Solar Time), latitude, and longitude.
 *       - Further development may include photo periods for other time standards, converted from LMST.
 */

#ifndef LMST_H
#define LMST_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

// ── Twilight stage types ─────────────────────────────────────────────────────
#define MAX_TWILIGHT_ZONES 7

enum {
    FullDaylight         = 0,
    GoldenHour           = 1,
    SunriseSunset        = 2,
    CivilTwilight        = 3,
    NauticalTwilight     = 4,
    AstronomicalTwilight = 5,
    AstronomicalNight    = 6
};

enum {
    Dawn = 0,
    Dusk = 1
};

extern float tzonesar[MAX_TWILIGHT_ZONES][2];
extern char twilight_zone_names[MAX_TWILIGHT_ZONES][22];

typedef struct {
    int current_zone; // Current twilight zone index
	double dawn_start[MAX_TWILIGHT_ZONES];
    double dawn_end[MAX_TWILIGHT_ZONES];
    double dusk_start[MAX_TWILIGHT_ZONES];
    double dusk_end[MAX_TWILIGHT_ZONES];
    double LMST_day_hours;
    double LMST_night_hours;
    double LMST_anomaly;
} PhotoPeriodSchedulele;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------

/**
 * @brief Calculate Local Mean Solar Time from UTC+-0 and observer's latitude and longitude.
 * 
 * @param utc_year Year in UTC
 * @param utc_month Month in UTC
 * @param utc_day Day in UTC
 * @param utc_hour Hour in UTC
 * @param utc_minute Minute in UTC
 * @param utc_second Second in UTC
 * @param latitude_degrees Observer's latitude in degrees
 * @param longitude_degrees Observer's longitude in degrees
 * @return tm Local Mean Solar Time as a struct tm
 */
tm makeLMST(
    uint16_t utc_year, uint8_t utc_month, uint8_t utc_day,
    uint8_t utc_hour, uint8_t utc_minute, uint8_t utc_second,
    double latitude_degrees,
    double longitude_degrees);

/**
 * @brief Calculate a photo period schedule for given coordinates and LMST time.
 * 
 * @param latitude_degrees latitude in degrees 
 * @param longitude_degrees longitude in degrees
 * @param lmst_tm Local Mean Solar Time as a struct tm
 * @param LMST_hour LMST hour
 * @param LMST_minute LMST minute
 * @param LMST_second LMST second
 * @return PhotoPeriodSchedulele Photo period schedule
 */
PhotoPeriodSchedulele getPhotoPeriodData(
    double latitude_degrees,
    double longitude_degrees,
    tm lmst_tm,
    uint8_t LMST_hour,
    uint8_t LMST_minute,
    uint8_t LMST_second
    );

#endif // LMST_H
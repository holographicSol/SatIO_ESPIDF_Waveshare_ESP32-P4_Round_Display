/*
  SatioFile - Written By Benjamin Jack Cullen.
*/

#include "satio_file.h"
#include <Arduino.h>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "matrix.h"
#include "strval.h"
#include "satio.h"
#include "system_data.h"
#include "matrix.h"
#include "custommapping.h"
#include "ins.h"
#include "config.h"
#include "multiplexers.h"
#include "wtgps300p.h"
#include "wt901.h"
#include "sdcard_helper.h"

struct satioFileStruct satioFileData = {
    .i_token=0,
    .token={},
    .tmp_chars="",

    .matrix_tags=
    {
        "SWITCH_PORT",        // 0
        "SWITCH_FUNCTION",    // 1
        "FUNCTION_X",         // 2
        "FUNCTION_Y",         // 3
        "FUNCTION_Z",         // 4
        "FUNCTION_OPERATOR",  // 5
        "FUNCTION_INVERT",    // 6
        "SWITCH_OUTPUT_MODE", // 7
        "SWITCH_PWM_VALUE_0", // 8
        "SWITCH_PWM_VALUE_1", // 9
        "SWITCH_FLUX",        // 10
        "COMPUTER_ASSIST",    // 11
        "MAP_SLOT",           // 12            
        
    },
    .matix_filepaths=
    {
        "/MATRIX/MATRIX_0.csv",
        "/MATRIX/MATRIX_1.csv",
        "/MATRIX/MATRIX_2.csv",
        "/MATRIX/MATRIX_3.csv",
        "/MATRIX/MATRIX_4.csv",
        "/MATRIX/MATRIX_5.csv",
        "/MATRIX/MATRIX_6.csv",
        "/MATRIX/MATRIX_7.csv",
        "/MATRIX/MATRIX_8.csv",
        "/MATRIX/MATRIX_9.csv",
    },
    .matrix_file_slots={0},
    .current_matrix_filepath="/MATRIX/MATRIX_0.csv", // default
    .mapping_tags=
    {
        "MAP_MODE",     // 0
        "FUNCTION_N",   // 1
        "MAP_CONFIG_1", // 2
        "MAP_CONFIG_2", // 3
        "MAP_CONFIG_3", // 4
        "MAP_CONFIG_4", // 5
        "MAP_CONFIG_5", // 6   
    },
    .mapping_filepath="/MAPPING/mapping_conf.csv",
    .system_tags=
    {

        "SERIAL_COMMAND",               // 0
        "OUTPUT_ALL",                   // 1
        "OUTPUT_SATIO",                 // 2
        "OUTPUT_INS",                   // 3
        "OUTPUT_GNGGA",                 // 4
        "OUTPUT_GNRMC",                 // 5
        "OUTPUT_GPATT",                 // 6
        "OUTPUT_MATRIX",                // 7
        "OUTPUT_ADMPLEX0",              // 8
        "OUTPUT_GYRO0",                 // 9
        "OUTPUT_SUN",                   // 10
        "OUTPUT_MOON",                  // 11
        "OUTPUT_MERCURY",               // 12
        "OUTPUT_VENUS",                 // 13
        "OUTPUT_MARS",                  // 14
        "OUTPUT_JUPITER",               // 15
        "OUTPUT_SATURN",                // 16
        "OUTPUT_URANUS",                // 17
        "OUTPUT_NEPTUNE",               // 18
        "OUTPUT_METEORS",               // 19
        "COORDINATE_CONVERSION_MODE",   // 20
        "SPEED_CONVERSION_MODE",        // 21
        "ALTITUDE_CONVERSION_MODE",     // 22
        "UTC_SECOND_OFFSET",            // 23
        "UTC_AUTO_OFFSET_FLAG",         // 24
        "SET_DATETIME_AUTOMATICALLY",   // 25
        "INS_REQ_GPS_PRECISION",        // 26
        "INS_REQ_MIN_SPEED",            // 27
        "INS_REQ_HEADING_RANGE_DIFF",   // 28
        "INS_MODE",                     // 29
        "INS_USE_GYRO_HEADING",         // 30
        "MATRIX_FILE",                  // 31
        "LOAD_MATRIX_ON_STARTUP",       // 32
        "SPEED_UNIT_MODE",              // 33
        "ALTITUDE_UNIT_MODE",           // 34
        "GROUND_HEADING_MODE",          // 35
        "ALTITUDE",                     // 36
        "COORDINATES_LATITUDE",         // 37
        "COORDINATES_LONGITUDE",        // 38
        "GROUND_SPEED",                 // 39
        "GROUND_HEADING",               // 40

        "DELAY_TASK_SERIAL_INFOCMD",        // 41
        "TICK_DELAY_TASK_SERIAL_INFOCMD",   // 42
        "DELAY_TASK_MULTIPLEXERS",          // 43
        "TICK_DELAY_TASK_MULTIPLEXERS",     // 44
        "DELAY_TASK_GYRO0",                 // 45
        "TICK_DELAY_TASK_GYRO0",            // 46
        "DELAY_TASK_UNIVERSE",              // 47
        "TICK_DELAY_TASK_UNIVERSE",         // 48
        "DELAY_TASK_GPS",                   // 49
        "TICK_DELAY_TASK_GPS",              // 50
        "DELAY_TASK_SWITCHES",              // 51
        "TICK_DELAY_TASK_SWITCHES",         // 52
        "DELAY_TASK_PORTCONTROLLER_INPUT",      // 53
        "TICK_DELAY_TASK_PORTCONTROLLER_INPUT", // 54
        "DELAY_TASK_STORAGE",               // 55
        "TICK_DELAY_TASK_STORAGE",          // 56
        "DELAY_TASK_LOGGING",               // 57
        "TICK_DELAY_TASK_LOGGING",          // 58

        "LOGGING",          // 59
    },
    .system_filepath="/SYSTEM/system_conf.csv",
    .log_dir="/LOG/",
    .log_files = {},
    .log_filepath = "",
    .unixtimestamp=0,
    .tmp_unixtimestamp=0,
    .number_of_log_files=0,
};

/**
 * @brief Yield for other tasks.
 * @note Current setup yields on every N lines loaded/saved, with no special cases.
 * @warning This increases loading/saving time.
 * 
 * yield_every_n_lines   0: very slow read/write (prioritize other tasks).
 * yield_every_n_lines > 0: faster read/write (more time blocking other tasks).
 */
int yield_every_n_lines=8;
int yield_counter=0;

void yieldForTasks() {
    if (yield_every_n_lines <= 0) { 
        // Always yield if yield_every_n_lines set to 0 or negative
        vTaskDelay(1);
        esp_task_wdt_reset();
        return;
    }
    yield_counter++;
    if (yield_counter >= yield_every_n_lines) {
        // Perform the actual yield
        vTaskDelay(1);
        esp_task_wdt_reset();
        yield_counter = 0; // Reset counter after yielding
    }
    // Otherwise, just pass through
}

void set_rw_running_true() {sdcardFlagData.rw_running=true;}
void set_rw_running_false() {sdcardFlagData.rw_running=false;}

/**
 * @brief Set Read/Write Success Flag.
 * 
 * Sustains flag for a period of time while also non-blocking before setting flag back to NULL.
 * 
 * @param flag Specify result of a read/write operation.
 */
void set_storage_success_flag(bool flag) {
    if (flag==true) {sdcardFlagData.success_flag=2; vTaskDelay(1000 / portTICK_PERIOD_MS);}
    else            {sdcardFlagData.success_flag=1; vTaskDelay(1000 / portTICK_PERIOD_MS);}
                     sdcardFlagData.success_flag=0;
}

// ----------------------------------------------------------------------------------------
// SD Card File Helper Functions (using standard C I/O with /sdcard prefix)
// ----------------------------------------------------------------------------------------

/**
 * @brief Create full path with /sdcard prefix
 */
static void sd_fullpath(const char* path, char* fullpath, size_t size) {
    snprintf(fullpath, size, "/sdcard%s", path);
}

/**
 * @brief Check if file exists
 */
static bool sd_exists(const char* path) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    struct stat st;
    return (stat(fullpath, &st) == 0);
}

/**
 * @brief Remove file
 */
static bool sd_remove(const char* path) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    return (remove(fullpath) == 0);
}

/**
 * @brief Get file size
 */
static uint64_t sd_file_size(const char* path) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    struct stat st;
    if (stat(fullpath, &st) == 0) {
        return (uint64_t)st.st_size;
    }
    return 0;
}

/**
 * @brief Create directory recursively
 */
static bool sd_mkdir(const char* path) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    
    char tmp[256];
    strncpy(tmp, fullpath, sizeof(tmp));
    size_t len = strlen(tmp);
    
    // Remove trailing slash
    if (tmp[len - 1] == '/') {
        tmp[len - 1] = '\0';
    }
    
    // Create directories recursively
    for (char* p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    return (mkdir(tmp, 0755) == 0 || errno == EEXIST);
}

/**
 * @brief Open file with /sdcard prefix
 */
static FILE* sd_fopen(const char* path, const char* mode) {
    char fullpath[256];
    sd_fullpath(path, fullpath, sizeof(fullpath));
    printf("[sd_fopen] fullpath: %s, mode: %s\n", fullpath, mode);
    fflush(stdout);
    
    // Create parent directory if writing
    if (strchr(mode, 'w') || strchr(mode, 'a')) {
        char dirpath[256];
        strncpy(dirpath, fullpath, sizeof(dirpath) - 1);
        dirpath[sizeof(dirpath) - 1] = '\0';
        char* lastSlash = strrchr(dirpath, '/');
        if (lastSlash && lastSlash != dirpath) {
            *lastSlash = '\0';
            printf("[sd_fopen] creating dir: %s\n", dirpath);
            fflush(stdout);
            // Create directory directly with full path (already has /sdcard)
            char tmp[256];
            strncpy(tmp, dirpath, sizeof(tmp) - 1);
            tmp[sizeof(tmp) - 1] = '\0';
            for (char* p = tmp + 1; *p; p++) {
                if (*p == '/') {
                    *p = '\0';
                    int ret = mkdir(tmp, 0755);
                    if (ret != 0 && errno != EEXIST) {
                        printf("[sd_fopen] mkdir(%s) failed: %s\n", tmp, strerror(errno));
                    }
                    *p = '/';
                }
            }
            int ret = mkdir(tmp, 0755);
            if (ret != 0 && errno != EEXIST) {
                printf("[sd_fopen] final mkdir(%s) failed: %s\n", tmp, strerror(errno));
            }
            fflush(stdout);
        }
    }
    
    // Create file
    FILE* f = fopen(fullpath, mode);
    if (!f) {
        printf("[sd_fopen] fopen(%s) failed: errno=%d (%s)\n", fullpath, errno, strerror(errno));
        fflush(stdout);
    }
    return f;
}

/** 
 * @brief Write string to file with newline
 */
static bool sd_fprintln(FILE* f, const char* str) {
    if (!f) return false;
    fputs(str, f);
    fputc('\n', f);
    return true;
}

// bool isOpen(File root) {
//  if (!root) {
//     Serial.println("[getLogFiles] Failed to open directory");
//     return false;
//   }
//   if (!root.isDirectory()) {
//     Serial.println("[getLogFiles] Not a directory");
//     root.close();
//     return false;
//   }
//   return true;
// }

/**
 * @brief Get a log filename.
 * @param mode Specify mode: 0=oldest 1=latest.
 */
bool getLogFile(int mode) {
  // printf("[getLogFiles] Listing directory: %s\n", satioFileData.log_dir);

  // Open dir (and create dir if not exist)
  char fullpath[256];
  sd_fullpath(satioFileData.log_dir, fullpath, sizeof(fullpath));
  sd_mkdir(satioFileData.log_dir); // ensure directory exists
  
  DIR* root = opendir(fullpath);
  if (!root) {return false;}

  // Set target
  if (mode==0) {satioFileData.unixtimestamp=INT64_MAX;} // for finding oldest
  else {satioFileData.unixtimestamp=0;} // for finding latest
  
  // Iterate through files in directory
  satioFileData.number_of_log_files=0;
  struct dirent* entry;
  while ((entry = readdir(root)) != NULL) {
    // Skip . and .. and directories
    if (entry->d_name[0] == '.') continue;
    if (entry->d_type == DT_DIR) continue;
    
    // printf("  FILE : %s\n", entry->d_name);

    // Create a copy of filename
    memset(satioFileData.tmp_chars, 0, sizeof(satioFileData.tmp_chars));
    strncpy(satioFileData.tmp_chars, entry->d_name, sizeof(satioFileData.tmp_chars));

    // Tokenize
    int i_san=0;
    satioFileData.tmp_unixtimestamp=0;
    satioFileData.i_token = 0;
    satioFileData.token = strtok(satioFileData.tmp_chars, ".");
    while (satioFileData.token != NULL) {
      switch (satioFileData.i_token) {
          case 0: if (satioFileData.token != NULL && str_is_int64(satioFileData.token)) {
              i_san++; char *endptr; satioFileData.tmp_unixtimestamp = strtoll(satioFileData.token, &endptr, 10);} break;
          case 1: if (strcmp(satioFileData.token, "csv")==0) {i_san++;} break;
      }
      satioFileData.token = strtok(NULL, ".");
      satioFileData.i_token++;
    }
    // Update unixtimestamp
    if (i_san==2) {
      satioFileData.number_of_log_files++;
      if      (mode==0 && satioFileData.tmp_unixtimestamp<satioFileData.unixtimestamp) {satioFileData.unixtimestamp=satioFileData.tmp_unixtimestamp;}
      else if (mode==1 && satioFileData.tmp_unixtimestamp>satioFileData.unixtimestamp) {satioFileData.unixtimestamp=satioFileData.tmp_unixtimestamp;}
    }
  }
  closedir(root);
  // printf("[getLogFiles] number_of_log_files: %llu\n", satioFileData.number_of_log_files);
  if (satioFileData.unixtimestamp==0 || satioFileData.unixtimestamp==INT64_MIN) {return false;} 

  // Store filename of interest
  memset(satioFileData.log_filepath, 0, sizeof(satioFileData.log_filepath));
  int written = snprintf(satioFileData.log_filepath, sizeof(satioFileData.log_filepath), "%s%lld.csv", satioFileData.log_dir, (long long)satioFileData.unixtimestamp);
  if (written < 0 || (size_t)written >= sizeof(satioFileData.log_filepath)) {
    printf("[getLogFiles] log_filepath truncated\n");
  }
  // printf("[getLogFiles] current log file: %s\n", satioFileData.log_filepath);
  return true;
}

void createNewLogFilename() {
  memset(satioFileData.log_filepath, 0, sizeof(satioFileData.log_filepath));
  // Use local_unixtime_uS for filename
  int written = snprintf(satioFileData.log_filepath, sizeof(satioFileData.log_filepath), "%s%lld.csv", satioFileData.log_dir, (long long)satioData.local_unixtime_uS);
  if (written < 0 || (size_t)written >= sizeof(satioFileData.log_filepath)) {
    printf("[createNewLogFilename] log_filepath truncated\n");
  }
  // printf("[createNewLogFilename] current log file: %s\n", satioFileData.log_filepath);
}

void deleteOldestLogFile() {
  if (getLogFile(0)==true && satioFileData.number_of_log_files>MAX_LOG_FILES) {
    // printf("[deleteOldestLogFile] attempting to delete log file: %s\n", satioFileData.log_filepath);
    sd_remove(satioFileData.log_filepath);
    if (!sd_exists(satioFileData.log_filepath)) {
        printf("[deleteOldestLogFile] log file deleted successfully.\n");
    }
    else {printf("[deleteOldestLogFile] failed to delete log file.\n");}
  }
}

void printLogLine(const char* line_str) {
    // Critical! This may take a moment, yield for other tasks!
    yieldForTasks();

    // printf("[printLogLine]\n");
    size_t line_len = strlen(line_str);

    // Check disk space
    if (!isAvailableBytes(line_len + 1)) {printf("No more diskspace available!\n"); return;}

    // Select log filename
    deleteOldestLogFile();
    if (getLogFile(1)==false) {createNewLogFilename();}
    
    // Ensure file exists (create if needed)
    FILE* f = sd_fopen(satioFileData.log_filepath, "a");
    if (f) fclose(f);

    // Check file size
    uint64_t file_size = sd_file_size(satioFileData.log_filepath);
    // printf("[printLogLine] line_size=%zu, file_size=%llu\n", line_len, file_size);
    if (file_size + line_len + 1 > MAX_LOG_FILE_SIZE) {
      printf("[printLogLine] creating new log file..\n");
      deleteOldestLogFile();
      createNewLogFilename();
    }

    // Write to log file
    // printf("[printLogLine] writing to log file: %s\n", satioFileData.log_filepath);
    f = sd_fopen(satioFileData.log_filepath, "a");
    if (f) {
      fputs(line_str, f);
      fputc('\n', f);
      fclose(f);
    }
    else {printf("[printLogLine] file does not exist: %s\n", satioFileData.log_filepath);}
}

bool writeLog() {
    // printf("--------------------------------------\n");
    printf("[writeLog] writing log\n");

    // --------------------------------
    // Log Line: Timestamp & Basic Stat
    // --------------------------------
    String line="";
    // --------------------------------
    // Log Line: Satio
    // --------------------------------
    line = "$SATIO,";
    line=line+ String(satioData.padded_rtc_time_HHMMSS) + ",";
    line=line+ String(satioData.padded_rtc_date_DDMMYYYY) + ",";
    line=line+ String(satioData.padded_rtc_sync_time_HHMMSS) + ",";
    line=line+ String(satioData.padded_rtc_sync_date_DDMMYYYY) + ",";
    line=line+ String(satioData.padded_local_time_HHMMSS) + ",";
    line=line+ String(satioData.padded_local_date_DDMMYYYY) + ",";
    line=line+ String(systemData.uptime_seconds) + ",";
    line=line+ String(satioData.char_coordinate_conversion_mode[satioData.coordinate_conversion_mode]) + ",";
    line=line+ String(satioData.degrees_latitude) + ",";
    line=line+ String(satioData.degrees_longitude) + ",";
    line=line+ String(satioData.char_altitude_conversion_mode[satioData.altitude_conversion_mode]) + ",";
    line=line+ String(satioData.altitude_converted) + ",";
    line=line+ String(satioData.char_altitude_unit_mode[satioData.altitude_unit_mode]) + ",";
    line=line+ String(satioData.char_speed_conversion_mode[satioData.speed_conversion_mode]) + ",";
    line=line+ String(satioData.speed_converted, 7) + ",";
    line=line+ String(satioData.char_speed_unit_mode[satioData.speed_unit_mode]) + ",";
    line=line+ String(satioData.char_ground_heading_mode[satioData.ground_heading_mode]) + ",";
    line=line+ String(satioData.ground_heading, 7) + ",";
    line=line+ String(insData.ins_latitude, 7) + ",";
    line=line+ String(insData.ins_longitude, 7) + ",";
    line=line+ String(insData.ins_altitude) + ",";
    line=line+ String(insData.ins_heading) + ",";
    line=line+ String(insData.INS_INITIALIZATION_FLAG) + ",";
    line=line+ String(insData.INS_MODE) + ",";
    line=line+ String(insData.INS_FORCED_ON_FLAG) + ",";
    line=line+ String(insData.INS_REQ_GPS_PRECISION) + ",";
    line=line+ String(insData.INS_REQ_HEADING_RANGE_DIFF) + ",";
    line=line+ String(insData.INS_REQ_MIN_SPEED) + ",";
    line=line+ String(insData.INS_USE_GYRO_HEADING) + ",";
    line=line+ String(insData.INS_ENABLED) + ",";
    printLogLine(line.c_str());

    // --------------------------------
    // Log Line: Analog/Digital
    // --------------------------------
    line="$MPLEX0,";
    for (int i=0; i<MAX_AD_MUX_CHANNELS; i++) {line=line+String(ad_mux_0.data[i])+",";}
    printLogLine(line.c_str());
    // --------------------------------
    // Log Line: Port Controller Input
    // --------------------------------
    line="$PCINPT,";
    for (int i=0; i<MAX_MATRIX_SWITCHES; i++) {line=line+String(matrixData.input_value[0][i])+",";}
    printLogLine(line.c_str());
    // --------------------------------
    // Log Line: Gyro0
    // --------------------------------
    line="$GYRO0,";
    line=line+ String(gyroData.gyro_0_acc_x) + ",";
    line=line+ String(gyroData.gyro_0_acc_y) + ",";
    line=line+ String(gyroData.gyro_0_acc_z) + ",";
    line=line+ String(gyroData.gyro_0_ang_x) + ",";
    line=line+ String(gyroData.gyro_0_ang_y) + ",";
    line=line+ String(gyroData.gyro_0_ang_z) + ",";
    line=line+ String(gyroData.gyro_0_gyr_x) + ",";
    line=line+ String(gyroData.gyro_0_gyr_y) + ",";
    line=line+ String(gyroData.gyro_0_gyr_z) + ",";
    line=line+ String(gyroData.gyro_0_mag_x) + ",";
    line=line+ String(gyroData.gyro_0_mag_y) + ",";
    line=line+ String(gyroData.gyro_0_mag_z) + ",";
    printLogLine(line.c_str());

    return true;
}

void printLine(FILE* f, const char* line) {
    if (!f) return;
    if (!isAvailableBytes(strlen(line) + 1)) {printf("No more diskspace available!\n"); return;}
    fputs(line, f);
    fputc('\n', f);
    // Critical! This may take a moment, yield for other tasks!
    yieldForTasks();
}

void printLineCStr(FILE* f, const char* line) {
    if (!f) return;
    if (!isAvailableBytes(strlen(line) + 1)) {printf("No more diskspace available!\n"); return;}
    fputs(line, f);
    fputc('\n', f);
    // Critical! This may take a moment, yield for other tasks!
    yieldForTasks();
}

char *endptr;

bool saveMappingFile(const char *filepath) {
    printf("$MAPPINGSAVING\n");
    FILE* f = sd_fopen(filepath, "w");
    if (!f) { printf("$MAPPINGSAVEFAILED\n"); return false; }
    
    char lineBuf[256];
    for (int i_tag=0; i_tag<MAX_MAPPING_TAGS; i_tag++) {

        for (int i_map=0; i_map<MAX_MAP_SLOTS; i_map++) {
            if      (i_tag==0) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", satioFileData.mapping_tags[i_tag], i_map, (int)mappingData.map_mode[0][i_map]); }
            else if (i_tag==1) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", satioFileData.mapping_tags[i_tag], i_map, (long)mappingData.mapping_config[0][i_map][0]); }
            else if (i_tag==2) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", satioFileData.mapping_tags[i_tag], i_map, (long)mappingData.mapping_config[0][i_map][1]); }
            else if (i_tag==3) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", satioFileData.mapping_tags[i_tag], i_map, (long)mappingData.mapping_config[0][i_map][2]); }
            else if (i_tag==4) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", satioFileData.mapping_tags[i_tag], i_map, (long)mappingData.mapping_config[0][i_map][3]); }
            else if (i_tag==5) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", satioFileData.mapping_tags[i_tag], i_map, (long)mappingData.mapping_config[0][i_map][4]); }
            else if (i_tag==6) { snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", satioFileData.mapping_tags[i_tag], i_map, (long)mappingData.mapping_config[0][i_map][5]); }
            printLine(f, lineBuf);
        }
    }
    
    fclose(f);
    printf("$MAPPINGSAVED\n");
    return true;
}

bool loadMappingFile(const char *filepath) {
    printf("$MAPPINGLOADING\n");
    if (!sd_exists(filepath)) { printf("$MAPPINGLOADFAILED\n");
        printf("[loadMappingFile] no mapping file found. attempting to create new file..\n");
        FILE* f = sd_fopen(filepath, "w");
        if (!f) {printf("[loadMappingFile] failed to create new mapping file.\n");}
        else {printf("[loadMappingFile] created new mapping file.\n");}
        fclose(f);
    }
    
    FILE* f = sd_fopen(filepath, "r");
    if (!f) { printf("$MAPPINGLOADFAILED\n"); return false; }
    
    override_all_computer_assists();
    set_all_mapping_default(); // avoid mixing current values with loaded values
    char lineBuffer[1024];
    int currentTag = 0;
    while (fgets(lineBuffer, sizeof(lineBuffer), f) != NULL) {
        // Critical! This may take a moment, yield for other tasks!
        yieldForTasks();

        // Strip newline
        size_t len = strlen(lineBuffer);
        while (len > 0 && (lineBuffer[len-1] == '\n' || lineBuffer[len-1] == '\r')) { lineBuffer[--len] = '\0'; }
        if (len == 0) continue;
        // printf("Processing Tag Token Number: %d (data: %s)\n", currentTag, lineBuffer); // uncomment to debug
        
        char *commaToken = strtok(lineBuffer, ",");
        int tokenCount = 0;
        signed int tag_index=-1;
        for (int i_find_tag=0; i_find_tag<MAX_MAPPING_TAGS; i_find_tag++) {if (strcmp(satioFileData.mapping_tags[i_find_tag], commaToken)==0) {tag_index=i_find_tag; break;}}
        if (tag_index==-1) {
            printf("Unrecognized tag found in mapping file: %s\n", commaToken);
            continue;
        }
        String data_0; String data_1; String data_2;
        commaToken = strtok(NULL, ","); // remove tag
        while (commaToken != NULL) {if (tokenCount==0) {data_0=commaToken;} else if (tokenCount==1) {data_1=commaToken;} else if (tokenCount==2) {data_2=commaToken;} commaToken = strtok(NULL, ","); tokenCount++;}
        if      (tag_index==0) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {mappingData.map_mode[0][atoi(data_0.c_str())]=atoi(data_1.c_str());}}
        else if (tag_index==1) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][0]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==2) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][1]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==3) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][2]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==4) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][3]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==5) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][4]=strtol(data_1.c_str(), &endptr, 10);}}
        else if (tag_index==6) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {mappingData.mapping_config[0][atoi(data_0.c_str())][5]=strtol(data_1.c_str(), &endptr, 10);}}
        currentTag++;
    }
    fclose(f);
    if (currentTag == 0) {printf("$MAPPINGLOADFAILED\n"); return false;}
    printf("$MAPPINGLOADED\n");
    return true;
}

bool deleteMappingFile(const char *filepath) {
    if (sd_exists(filepath)) {if (sd_remove(filepath)) {printf("$MAPPINGDELETED\n"); return true;}}
    printf("$MAPPINGDELETEFAILED\n");
    return false;
}

// todo: mapping data will saved with matrix data
bool saveMatrixFile(const char *filepath) {    
    printf("$MATRIXSAVING\n");
    FILE* f = sd_fopen(filepath, "w");
    if (!f) { printf("$MATRIXSAVEFAILED\n"); return false; }
    
    char lineBuf[256];
    for (int i_tag=0; i_tag<MAX_MATRIX_TAGS; i_tag++) {

        if (i_tag==0) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", satioFileData.matrix_tags[i_tag], i_switch, (int)matrixData.matrix_port_map[0][i_switch]);
                printLine(f, lineBuf);
            }
        }
        else if (i_tag==1) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                    snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", satioFileData.matrix_tags[i_tag], i_switch, i_func, (int)matrixData.matrix_function[0][i_switch][i_func]);
                    printLine(f, lineBuf);
                }
            }
        }
        else if (i_tag==2) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                    snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%.10f", satioFileData.matrix_tags[i_tag], i_switch, i_func, matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_X]);
                    printLine(f, lineBuf);
                }
            }
        }
        else if (i_tag==3) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                    snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%.10f", satioFileData.matrix_tags[i_tag], i_switch, i_func, matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_Y]);
                    printLine(f, lineBuf);
                }
            }
        }
        else if (i_tag==4) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                    snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%.10f", satioFileData.matrix_tags[i_tag], i_switch, i_func, matrixData.matrix_function_xyz[0][i_switch][i_func][INDEX_MATRIX_FUNTION_Z]);
                    printLine(f, lineBuf);
                }
            }
        }
        else if (i_tag==5) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                    snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", satioFileData.matrix_tags[i_tag], i_switch, i_func, (int)matrixData.matrix_switch_operator_index[0][i_switch][i_func]);
                    printLine(f, lineBuf);
                }
            }
        }
        else if (i_tag==6) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                for (int i_func=0; i_func<MAX_MATRIX_SWITCH_FUNCTIONS; i_func++) {
                    snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d,%d", satioFileData.matrix_tags[i_tag], i_switch, i_func, (int)matrixData.matrix_switch_inverted_logic[0][i_switch][i_func]);
                    printLine(f, lineBuf);
                }
            }
        }
        else if (i_tag==7) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", satioFileData.matrix_tags[i_tag], i_switch, (int)matrixData.output_mode[0][i_switch]);
                printLine(f, lineBuf);
            }
        }
        else if (i_tag==8) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%lu", satioFileData.matrix_tags[i_tag], i_switch, (unsigned long)matrixData.output_pwm[0][i_switch][0]);
                printLine(f, lineBuf);
            }
        }
        else if (i_tag==9) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%lu", satioFileData.matrix_tags[i_tag], i_switch, (unsigned long)matrixData.output_pwm[0][i_switch][1]);
                printLine(f, lineBuf);
            }
        }
        else if (i_tag==10) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%ld", satioFileData.matrix_tags[i_tag], i_switch, (long)matrixData.flux_value[0][i_switch]);
                printLine(f, lineBuf);
            }
        }
        else if (i_tag==11) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", satioFileData.matrix_tags[i_tag], i_switch, (int)matrixData.computer_assist[0][i_switch]);
                printLine(f, lineBuf);
            }
        }
        else if (i_tag==12) {
            for (int i_switch=0; i_switch<MAX_MATRIX_SWITCHES; i_switch++) {
                snprintf(lineBuf, sizeof(lineBuf), "%s,%d,%d", satioFileData.matrix_tags[i_tag], i_switch, (int)matrixData.index_mapped_value[0][i_switch]);
                printLine(f, lineBuf);
            }
        }
    }
    
    fclose(f);
    printf("$MATRIXSAVED\n");
    return true;
}

bool loadMatrixFile(const char *filepath) {
    printf("$MATRIXLOADING\n");
    if (!sd_exists(filepath)) { printf("$MATRIXLOADFAILED\n"); return false; }
    
    FILE* f = sd_fopen(filepath, "r");
    if (!f) { printf("$MATRIXLOADFAILED\n"); return false; }
    
    override_all_computer_assists();
    set_all_matrix_default(); // avoid mixing current values with loaded values
    char lineBuffer[1024];
    int currentTag = 0;
    while (fgets(lineBuffer, sizeof(lineBuffer), f) != NULL) {
        // Critical! This may take a moment, yield for other tasks!
        yieldForTasks();

        // Strip newline
        size_t len = strlen(lineBuffer);
        while (len > 0 && (lineBuffer[len-1] == '\n' || lineBuffer[len-1] == '\r')) { lineBuffer[--len] = '\0'; }
        if (len == 0) continue;
        // printf("Processing Tag Token Number: %d (data: %s)\n", currentTag, lineBuffer); // uncomment to debug
        
        char *commaToken = strtok(lineBuffer, ",");
        int tokenCount = 0;
        signed int tag_index = -1;
        for (int i_find_tag=0; i_find_tag<MAX_MATRIX_TAGS; i_find_tag++) {if (strcmp(satioFileData.matrix_tags[i_find_tag], commaToken)==0) {tag_index=i_find_tag; break;}}
        if (tag_index==-1) {
            printf("Unrecognized tag found in matrix file: %s\n", commaToken);
            continue;
        }
        String data_0; String data_1; String data_2;
        commaToken = strtok(NULL, ","); // remove tag
        while (commaToken != NULL) {if (tokenCount==0) {data_0=commaToken;} else if (tokenCount==1) {data_1=commaToken;} else if (tokenCount==2) {data_2=commaToken;} commaToken = strtok(NULL, ","); tokenCount++;}
        if      (tag_index==0) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {matrixData.matrix_port_map[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==1) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_function[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==2) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_X]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==3) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_Y]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==4) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_double(data_2.c_str())) {matrixData.matrix_function_xyz[0][atoi(data_0.c_str())][atoi(data_1.c_str())][INDEX_MATRIX_FUNTION_Z]=strtod(data_2.c_str(), &endptr);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==5) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_switch_operator_index[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==6) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str()) && str_is_int8(data_2.c_str())) {matrixData.matrix_switch_inverted_logic[0][atoi(data_0.c_str())][atoi(data_1.c_str())]=atoi(data_2.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==7) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {matrixData.output_mode[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==8) {if (str_is_int8(data_0.c_str()) && str_is_uint32(data_1.c_str())) {matrixData.output_pwm[0][atoi(data_0.c_str())][0]=strtoul(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==9) {if (str_is_int8(data_0.c_str()) && str_is_uint32(data_1.c_str())) {matrixData.output_pwm[0][atoi(data_0.c_str())][1]=strtoul(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==10) {if (str_is_int8(data_0.c_str()) && str_is_long(data_1.c_str())) {matrixData.flux_value[0][atoi(data_0.c_str())]=strtol(data_1.c_str(), &endptr, 10);} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==11) {if (str_is_int8(data_0.c_str()) && str_is_bool(data_1.c_str())) {matrixData.computer_assist[0][atoi(data_0.c_str())]=atoi(data_1.c_str());} matrixData.matrix_switch_write_required[0][atoi(data_0.c_str())]=true;}
        else if (tag_index==12) {if (str_is_int8(data_0.c_str()) && str_is_int8(data_1.c_str())) {matrixData.index_mapped_value[0][atoi(data_0.c_str())]=atoi(data_1.c_str());}}
        currentTag++;
    }
    fclose(f);
    if (currentTag == 0) {printf("$MATRIXLOADFAILED\n"); return false;}
    printf("$MATRIXLOADED\n");
    return true;
}

bool deleteMatrixFile(const char *filepath) {
    if (sd_exists(filepath)) {if (sd_remove(filepath)) {printf("$MATRIXDELETED\n"); return true;}}
    printf("$MATRIXDELETEFAILED\n");
    return false;
}

bool saveSystemFile(const char *filepath) {
    printf("$SYSTEMSAVING\n");
    printf("[saveSystemFile] filepath: %s\n", filepath);
    fflush(stdout);
    FILE* f = sd_fopen(filepath, "w");
    if (!f) { 
        printf("[saveSystemFile] fopen failed, errno: %d (%s)\n", errno, strerror(errno));
        fflush(stdout);
        printf("$SYSTEMSAVEFAILED\n"); 
        return false; 
    }
    
    // Use heap-allocated buffer to avoid stack overflow
    char* lineBuf = (char*)malloc(256);
    if (!lineBuf) { fclose(f); printf("$SYSTEMSAVEFAILED\n"); return false; }
    
    #define WRITE_INT_TAG(idx, val) snprintf(lineBuf, 256, "%s,%d", satioFileData.system_tags[idx], (int)(val)); printLineCStr(f, lineBuf)
    #define WRITE_LONG_TAG(idx, val) snprintf(lineBuf, 256, "%s,%ld", satioFileData.system_tags[idx], (long)(val)); printLineCStr(f, lineBuf)
    #define WRITE_DBL_TAG(idx, val) snprintf(lineBuf, 256, "%s,%.6f", satioFileData.system_tags[idx], (double)(val)); printLineCStr(f, lineBuf)
    #define WRITE_STR_TAG(idx, val) snprintf(lineBuf, 256, "%s,%s", satioFileData.system_tags[idx], (val)); printLineCStr(f, lineBuf)
    
    WRITE_INT_TAG(0, systemData.serial_command);
    WRITE_INT_TAG(1, systemData.output_satio_all);
    WRITE_INT_TAG(2, systemData.output_satio_enabled);
    WRITE_INT_TAG(3, systemData.output_ins_enabled);
    WRITE_INT_TAG(4, systemData.output_gngga_enabled);
    WRITE_INT_TAG(5, systemData.output_gnrmc_enabled);
    WRITE_INT_TAG(6, systemData.output_gpatt_enabled);
    WRITE_INT_TAG(7, systemData.output_matrix_enabled);
    WRITE_INT_TAG(8, systemData.output_admplex0_enabled);
    WRITE_INT_TAG(9, systemData.output_gyro_0_enabled);
    WRITE_INT_TAG(10, systemData.output_sun_enabled);
    WRITE_INT_TAG(11, systemData.output_moon_enabled);
    WRITE_INT_TAG(12, systemData.output_mercury_enabled);
    WRITE_INT_TAG(13, systemData.output_venus_enabled);
    WRITE_INT_TAG(14, systemData.output_mars_enabled);
    WRITE_INT_TAG(15, systemData.output_jupiter_enabled);
    WRITE_INT_TAG(16, systemData.output_saturn_enabled);
    WRITE_INT_TAG(17, systemData.output_uranus_enabled);
    WRITE_INT_TAG(18, systemData.output_neptune_enabled);
    WRITE_INT_TAG(19, systemData.output_meteors_enabled);
    WRITE_INT_TAG(20, satioData.coordinate_conversion_mode);
    WRITE_INT_TAG(21, satioData.speed_conversion_mode);
    WRITE_INT_TAG(22, satioData.altitude_conversion_mode);
    WRITE_LONG_TAG(23, satioData.utc_second_offset);
    WRITE_INT_TAG(24, satioData.utc_auto_offset_flag);
    WRITE_INT_TAG(25, satioData.set_time_automatically);
    WRITE_INT_TAG(26, insData.INS_REQ_GPS_PRECISION);
    WRITE_DBL_TAG(27, insData.INS_REQ_MIN_SPEED);
    WRITE_DBL_TAG(28, insData.INS_REQ_HEADING_RANGE_DIFF);
    WRITE_INT_TAG(29, insData.INS_MODE);
    WRITE_INT_TAG(30, insData.INS_USE_GYRO_HEADING);
    WRITE_STR_TAG(31, satioFileData.current_matrix_filepath);
    WRITE_INT_TAG(32, matrixData.load_matrix_on_startup);
    WRITE_INT_TAG(33, satioData.speed_unit_mode);
    WRITE_INT_TAG(34, satioData.altitude_unit_mode);
    WRITE_INT_TAG(35, satioData.ground_heading_mode);
    WRITE_DBL_TAG(36, satioData.altitude);
    WRITE_DBL_TAG(37, satioData.degrees_latitude);
    WRITE_DBL_TAG(38, satioData.degrees_longitude);
    WRITE_DBL_TAG(39, satioData.speed);
    WRITE_DBL_TAG(40, satioData.ground_heading);
    WRITE_LONG_TAG(41, DELAY_TASK_SERIAL_INFOCMD);
    WRITE_INT_TAG(42, TICK_DELAY_TASK_SERIAL_INFOCMD);
    WRITE_LONG_TAG(43, DELAY_TASK_MULTIPLEXERS);
    WRITE_INT_TAG(44, TICK_DELAY_TASK_MULTIPLEXERS);
    WRITE_LONG_TAG(45, DELAY_TASK_GYRO0);
    WRITE_INT_TAG(46, TICK_DELAY_TASK_GYRO0);
    WRITE_LONG_TAG(47, DELAY_TASK_UNIVERSE);
    WRITE_INT_TAG(48, TICK_DELAY_TASK_UNIVERSE);
    WRITE_LONG_TAG(49, DELAY_TASK_GPS);
    WRITE_INT_TAG(50, TICK_DELAY_TASK_GPS);
    WRITE_LONG_TAG(51, DELAY_TASK_SWITCHES);
    WRITE_INT_TAG(52, TICK_DELAY_TASK_SWITCHES);
    WRITE_LONG_TAG(53, DELAY_TASK_PORTCONTROLLER_INPUT);
    WRITE_INT_TAG(54, TICK_DELAY_TASK_PORTCONTROLLER_INPUT);
    WRITE_LONG_TAG(55, DELAY_TASK_STORAGE);
    WRITE_INT_TAG(56, TICK_DELAY_TASK_STORAGE);
    WRITE_LONG_TAG(57, DELAY_TASK_LOGGING);
    WRITE_INT_TAG(58, TICK_DELAY_TASK_LOGGING);
    WRITE_INT_TAG(59, systemData.logging_enabled);
    
    #undef WRITE_INT_TAG
    #undef WRITE_LONG_TAG
    #undef WRITE_DBL_TAG
    #undef WRITE_STR_TAG
    
    free(lineBuf);
    fclose(f);
    printf("$SYSTEMSAVED\n");
    return true;
}

bool loadSystemFile(const char *filepath) {
    printf("$SYSTEMLOADING\n");
    if (!sd_exists(filepath)) { printf("$SYSTEMLOADFAILED\n"); return false; }
    
    FILE* f = sd_fopen(filepath, "r");
    if (!f) { printf("$SYSTEMLOADFAILED\n"); return false; }
    
    char lineBuffer[1024];
    int currentTag = 0;
    while (fgets(lineBuffer, sizeof(lineBuffer), f) != NULL) {
        // Critical! This may take a moment, yield for other tasks!
        yieldForTasks();

        // Strip newline
        size_t len = strlen(lineBuffer);
        while (len > 0 && (lineBuffer[len-1] == '\n' || lineBuffer[len-1] == '\r')) { lineBuffer[--len] = '\0'; }
        if (len == 0) continue;
            printf("Processing Tag Token Number: %d (data: %s)\n", currentTag, lineBuffer); // uncomment to debug
            char *commaToken = strtok(lineBuffer, ",");
            int tokenCount = 0;
            signed int tag_index = -1;
            for (int i_find_tag=0; i_find_tag<MAX_SYSTEM_TAGS; i_find_tag++) {if (strcmp(satioFileData.system_tags[i_find_tag], commaToken)==0) {tag_index=i_find_tag; break;}}
            if (tag_index==-1) {
                printf("Unrecognized tag found in system file: %s\n", commaToken);
                continue;
            }
            // Serial.println("Tag Found: " + String(satioFileData.system_tags[tag_index]));
            String data_0; String data_1; String data_2;
            commaToken = strtok(NULL, ","); // remove tag
            while (commaToken != NULL) {if (tokenCount==0) {data_0=commaToken;} else if (tokenCount==1) {data_1=commaToken;} else if (tokenCount==2) {data_2=commaToken;} commaToken = strtok(NULL, ","); tokenCount++;}
            if      (tag_index==0) {if (str_is_bool(data_0.c_str())) {systemData.serial_command=atoi(data_0.c_str());}}
            else if (tag_index==1) {if (str_is_bool(data_0.c_str())) {systemData.output_satio_all=atoi(data_0.c_str());}}
            else if (tag_index==2) {if (str_is_bool(data_0.c_str())) {systemData.output_satio_enabled=atoi(data_0.c_str());}}
            else if (tag_index==3) {if (str_is_bool(data_0.c_str())) {systemData.output_ins_enabled=atoi(data_0.c_str());}}
            else if (tag_index==4) {if (str_is_bool(data_0.c_str())) {systemData.output_gngga_enabled=atoi(data_0.c_str());}}
            else if (tag_index==5) {if (str_is_bool(data_0.c_str())) {systemData.output_gnrmc_enabled=atoi(data_0.c_str());}}
            else if (tag_index==6) {if (str_is_bool(data_0.c_str())) {systemData.output_gpatt_enabled=atoi(data_0.c_str());}}
            else if (tag_index==7) {if (str_is_bool(data_0.c_str())) {systemData.output_matrix_enabled=atoi(data_0.c_str());}}
            else if (tag_index==8) {if (str_is_bool(data_0.c_str())) {systemData.output_admplex0_enabled=atoi(data_0.c_str());}}
            else if (tag_index==9) {if (str_is_bool(data_0.c_str())) {systemData.output_gyro_0_enabled=atoi(data_0.c_str());}}
            else if (tag_index==10) {if (str_is_bool(data_0.c_str())) {systemData.output_sun_enabled=atoi(data_0.c_str());}}
            else if (tag_index==11) {if (str_is_bool(data_0.c_str())) {systemData.output_moon_enabled=atoi(data_0.c_str());}}
            else if (tag_index==12) {if (str_is_bool(data_0.c_str())) {systemData.output_mercury_enabled=atoi(data_0.c_str());}}
            else if (tag_index==13) {if (str_is_bool(data_0.c_str())) {systemData.output_venus_enabled=atoi(data_0.c_str());}}
            else if (tag_index==14) {if (str_is_bool(data_0.c_str())) {systemData.output_mars_enabled=atoi(data_0.c_str());}}
            else if (tag_index==15) {if (str_is_bool(data_0.c_str())) {systemData.output_jupiter_enabled=atoi(data_0.c_str());}}
            else if (tag_index==16) {if (str_is_bool(data_0.c_str())) {systemData.output_saturn_enabled=atoi(data_0.c_str());}}
            else if (tag_index==17) {if (str_is_bool(data_0.c_str())) {systemData.output_uranus_enabled=atoi(data_0.c_str());}}
            else if (tag_index==18) {if (str_is_bool(data_0.c_str())) {systemData.output_neptune_enabled=atoi(data_0.c_str());}}
            else if (tag_index==19) {if (str_is_bool(data_0.c_str())) {systemData.output_meteors_enabled=atoi(data_0.c_str());}}
            else if (tag_index==20) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_COORDINATE_CONVERSION_CONVERSION_MODES) {satioData.coordinate_conversion_mode=atoi(data_0.c_str());}}}
            else if (tag_index==21) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_SPEED_CONVERSIO_MODES) {satioData.speed_conversion_mode=atoi(data_0.c_str());}}}
            else if (tag_index==22) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_ALTITUDE_CONVERSION_MODES) {satioData.altitude_conversion_mode=atoi(data_0.c_str());}}}
            else if (tag_index==23) {if (str_is_long(data_0.c_str())) {satioData.utc_second_offset=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==24) {if (str_is_bool(data_0.c_str())) {satioData.utc_auto_offset_flag=atoi(data_0.c_str());}}
            else if (tag_index==25) {if (str_is_bool(data_0.c_str())) {satioData.set_time_automatically=atoi(data_0.c_str());}}
            else if (tag_index==26) {if (str_is_double(data_0.c_str())) {insData.INS_REQ_GPS_PRECISION=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==27) {if (str_is_double(data_0.c_str())) {insData.INS_REQ_MIN_SPEED=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==28) {if (str_is_double(data_0.c_str())) {insData.INS_REQ_HEADING_RANGE_DIFF=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==29) {if (str_is_double(data_0.c_str())) {insData.INS_MODE=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==30) {if (str_is_double(data_0.c_str())) {insData.INS_USE_GYRO_HEADING=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==31) {memset(satioFileData.current_matrix_filepath, 0, sizeof(satioFileData.current_matrix_filepath)); strcpy(satioFileData.current_matrix_filepath, data_0.c_str());}
            else if (tag_index==32) {if (str_is_bool(data_0.c_str())) {matrixData.load_matrix_on_startup=atoi(data_0.c_str());}}
            else if (tag_index==33) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_SPEED_UNIT_MODES) {satioData.speed_unit_mode=atoi(data_0.c_str());}}}
            else if (tag_index==34) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_ALTITUDE_UNIT_MODES) {satioData.altitude_unit_mode=atoi(data_0.c_str());}}}
            else if (tag_index==35) {if (str_is_uint8(data_0.c_str())) {if (atoi(data_0.c_str())<MAX_GROUND_HEADING_MODES) {satioData.ground_heading_mode=atoi(data_0.c_str());}}}
            else if (tag_index==36) {if (str_is_double(data_0.c_str())) {satioData.altitude=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==37) {if (str_is_double(data_0.c_str())) {satioData.degrees_latitude=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==38) {if (str_is_double(data_0.c_str())) {satioData.degrees_longitude=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==39) {if (str_is_double(data_0.c_str())) {satioData.speed=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==40) {if (str_is_double(data_0.c_str())) {satioData.ground_heading=strtod(data_0.c_str(), &endptr);}}
            else if (tag_index==41) {if (str_is_long(data_0.c_str())) {DELAY_TASK_SERIAL_INFOCMD=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==42) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_SERIAL_INFOCMD=atoi(data_0.c_str());}}
            else if (tag_index==43) {if (str_is_long(data_0.c_str())) {DELAY_TASK_MULTIPLEXERS=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==44) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_MULTIPLEXERS=atoi(data_0.c_str());}}
            else if (tag_index==45) {if (str_is_long(data_0.c_str())) {DELAY_TASK_GYRO0=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==46) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_GYRO0=atoi(data_0.c_str());}}
            else if (tag_index==47) {if (str_is_long(data_0.c_str())) {DELAY_TASK_UNIVERSE=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==48) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_UNIVERSE=atoi(data_0.c_str());}}
            else if (tag_index==49) {if (str_is_long(data_0.c_str())) {DELAY_TASK_GPS=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==50) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_GPS=atoi(data_0.c_str());}}
            else if (tag_index==51) {if (str_is_long(data_0.c_str())) {DELAY_TASK_SWITCHES=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==52) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_SWITCHES=atoi(data_0.c_str());}}
            else if (tag_index==53) {if (str_is_long(data_0.c_str())) {DELAY_TASK_PORTCONTROLLER_INPUT=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==54) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_PORTCONTROLLER_INPUT=atoi(data_0.c_str());}}
            else if (tag_index==55) {if (str_is_long(data_0.c_str())) {DELAY_TASK_STORAGE=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==56) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_STORAGE=atoi(data_0.c_str());}}
            else if (tag_index==57) {if (str_is_long(data_0.c_str())) {DELAY_TASK_LOGGING=strtol(data_0.c_str(), &endptr, 10);}}
            else if (tag_index==58) {if (str_is_bool(data_0.c_str())) {TICK_DELAY_TASK_LOGGING=atoi(data_0.c_str());}}
            else if (tag_index==59) {if (str_is_bool(data_0.c_str())) {systemData.logging_enabled=atoi(data_0.c_str());}}
            currentTag++;
    }
    fclose(f);
    if (currentTag == 0) {printf("$SYSTEMLOADFAILED\n"); return false;}
    printf("$SYSTEMLOADED\n");
    return true;
}

bool deleteSystemFile(const char *filepath) {
    if (sd_exists(filepath)) {if (sd_remove(filepath)) {printf("$SYSTEMDELETED\n"); return true;}}
    printf("$SYSTEMDELETEFAILED\n");
    return false;
}

bool ran_startup_file_operations = false;

void sdcardFlagHandler() {

  // ---- MOUNT ----

//   if (sdcardFlagData.unmount_sdcard_flag==true) {
//     printf("card unmount flag detected\n");
//     sdcardData.allow_mount=false; // override allow_mount (prevent remounting on unmount flag).
//     sdcardFlagData.no_delay_flag=false; // reset flag
//     sdcardFlagData.unmount_sdcard_flag=false; // reset flag
//     sdcard_unmount();
//   }
//   else if (sdcardFlagData.mount_sdcard_flag==true) {
//     printf("card mount flag detected\n");
//     sdcardData.allow_mount=true; // override allow_mount on mount flag.
//     sdcardFlagData.no_delay_flag=false;
//     sdcardFlagData.mount_sdcard_flag=false;
//     sdcard_mount();
//   }

  if (sdcardData.sdcard_mounted==false) {
    
  }

  else if (sdcardData.sdcard_mounted==true) {

    // ---- MAPPING ----

    if (sdcardFlagData.save_mapping) {
      printf("saving mapping ...\n");
      if (saveMappingFile(satioFileData.mapping_filepath))
        {printf("saved mapping successfully.\n"); set_storage_success_flag(true);}
      else {printf("save mapping failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.save_mapping=false;
    }
    else if (sdcardFlagData.load_mapping) {
      printf("loading mapping...\n");
      if (loadMappingFile(satioFileData.mapping_filepath))
        {printf("loaded mapping successfully.\n"); set_storage_success_flag(true);}
      else {printf("load mapping failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.load_mapping=false;
    }
    else if (sdcardFlagData.delete_mapping) {
      printf("deleting mapping...\n");
      if (deleteMappingFile(satioFileData.mapping_filepath))
        {printf("deleted mapping successfully.\n"); set_storage_success_flag(true);}
      else {printf("delete mapping failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.delete_mapping=false;
    }

    // ---- MATRIX ----

    else if (sdcardFlagData.save_matrix) {
      printf("saving matrix ...\n");
      if (saveMatrixFile(satioFileData.current_matrix_filepath))
        {printf("saved matrix successfully.\n"); set_storage_success_flag(true);}
      else {printf("save matrix failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.save_matrix=false;
    }
    else if (sdcardFlagData.load_matrix) {
      printf("loading matrix...\n");
      if (loadMatrixFile(satioFileData.current_matrix_filepath))
        {printf("loaded matrix successfully.\n"); set_storage_success_flag(true);}
      else {printf("load matrix failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.load_matrix=false;
    }
    else if (sdcardFlagData.delete_matrix) {
      printf("deleting matrix...\n");
      if (deleteMatrixFile(satioFileData.current_matrix_filepath))
        {printf("deleted matrix successfully.\n"); set_storage_success_flag(true);}
      else {printf("delete matrix failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.delete_matrix=false;
    }

    // ---- SYSTEM ----

    else if (sdcardFlagData.save_system) {
      printf("saving system...\n");
      if (saveSystemFile(satioFileData.system_filepath))
        {printf("saved system successfully.\n"); set_storage_success_flag(true);}
      else {printf("save system failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.save_system=false;
    }
    else if (sdcardFlagData.delete_system) {
      printf("deleting system...\n");
      if (deleteSystemFile(satioFileData.system_filepath))
        {printf("deleted system successfully.\n"); set_storage_success_flag(true);}
      else {printf("delete system failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.delete_system=false;
    }

    else if (sdcardFlagData.load_system) {
      // load system
      printf("loading system...\n");
      if (loadSystemFile(satioFileData.system_filepath))
        {printf("loaded system successfully.\n"); set_storage_success_flag(true);}
      else {printf("load system failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.load_system=false;

      if (ran_startup_file_operations==false && matrixData.load_matrix_on_startup==true) {
        // load mapping
        printf("loading mapping...\n");
        if (loadMappingFile(satioFileData.mapping_filepath))
            {printf("loaded mapping successfully.\n"); set_storage_success_flag(true);}
        else {printf("load mapping failed.\n"); set_storage_success_flag(false);}
        sdcardFlagData.load_mapping=false;

        // load matrix
        printf("loading matrix on startup...\n");
        if (loadMatrixFile(satioFileData.current_matrix_filepath))
          {printf("loaded matrix successfully.\n"); set_storage_success_flag(true);}
        else {printf("load matrix failed.\n"); set_storage_success_flag(false);}
        sdcardFlagData.load_matrix=false;

        // end
        ran_startup_file_operations = true;
      }
      sdcardFlagData.no_delay_flag=false;
    }

    // ---- LOG ----

    else if (sdcardFlagData.write_log) {
      if (writeLog())
        {printf("[log] successfull.\n"); set_storage_success_flag(true);}
      else {printf("[log] failed.\n"); set_storage_success_flag(false);}
      sdcardFlagData.no_delay_flag=false;
      sdcardFlagData.write_log=false;
    }
  }
//   clearSDMMCArgStruct();
}
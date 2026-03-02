/*
  SatioFile - Written By Benjamin Jack Cullen.
*/

#ifndef SATIO_FILE_HELPER_H
#define SATIO_FILE_HELPER_H

#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
#include "config.h"

#define MAX_MATRIX_TAGS   13
#define MAX_MATRIX_SLOTS  10
#define MAX_MAPPING_TAGS  7
#define MAX_SYSTEM_TAGS   60
#define MAX_LOG_FILES     3     // maximum number of log files to be kept.
/*
  Maximum log file size in bytes.
  Each record is approximately 1KB in size.
  1KB * Seconds in a day = 3686400 bytes = approx. 3MB daily.
  1KB * Milliseconds in a day = 3686400000 bytes = approx. 3GB daily.
*/
#define DEAFAULT_MAX_LOG_FILE_SIZE 3686400
#define MAX_LOG_FILE_SIZE DEAFAULT_MAX_LOG_FILE_SIZE

struct satioFileStruct {

    long i_token;
    char* token;
    char tmp_chars[MAX_GLOBAL_ELEMENT_SIZE];

    char matrix_tags[MAX_MATRIX_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    char matix_filepaths[MAX_MATRIX_SLOTS][MAX_GLOBAL_ELEMENT_SIZE];
    int matrix_file_slots[MAX_MATRIX_SLOTS];
    char current_matrix_filepath[MAX_GLOBAL_ELEMENT_SIZE];

    char mapping_tags[MAX_MAPPING_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    char mapping_filepath[MAX_GLOBAL_ELEMENT_SIZE];

    char system_tags[MAX_SYSTEM_TAGS][MAX_GLOBAL_ELEMENT_SIZE];
    char system_filepath[MAX_GLOBAL_ELEMENT_SIZE];

    char log_dir[MAX_GLOBAL_ELEMENT_SIZE];
    char log_files[MAX_LOG_FILES][MAX_GLOBAL_ELEMENT_SIZE];
    char log_filepath[MAX_GLOBAL_ELEMENT_SIZE];
    int64_t unixtimestamp;
    int64_t tmp_unixtimestamp;
    int64_t number_of_log_files;
};
extern struct satioFileStruct satioFileData;

// system -log -e
// system -log -d

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------

/**
 * Save current mapping data.
 * @param filepath Specify path to file
 */
bool saveMappingFile(const char *filepath);

/**
 * Load mapping data.
 * @param filepath Specify path to file
 */
bool loadMappingFile(const char *filepath);

/**
 * Delete mapping data.
 * @param filepath Specify path to file
 */
bool deleteMappingFile(const char *filepath);

/**
 * Save current matrix data.
 * @param filepath Specify path to file
 */
bool saveMatrixFile(const char *filepath);

/**
 * Load matrix data.
 * @param filepath Specify path to file
 */
bool loadMatrixFile(const char *filepath);

/**
 * Delete matrix data.
 * @param filepath Specify path to file
 */
bool deleteMatrixFile(const char *filepath);

/**
 * Save system data.
 * @param filepath Specify path to file
 */
bool saveSystemFile(const char *filepath);

/**
 * Load system data.
 * @param filepath Specify path to file
 */
bool loadSystemFile(const char *filepath);

/**
 * Delete matrix data.
 * @param filepath Specify path to file
 */
bool deleteSystemFile(const char *filepath);

/**
 * Write Log.
 */
bool writeLog();

/**
 * @brief Print SD some information.
 */
void statSDCard();

/**
 * @brief Get free disk space.
 * @return Free space in bytes.
 */
uint64_t getFreeBytes();

/**
 * @brief Check if bytes available on disk.
 * @return Bool true if space available.
 */
bool isAvailableBytes(uint64_t num_bytes);

void sdcardFlagHandler();

void getLogFiles();

#endif
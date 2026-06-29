/*
    Serial Information Command - Written By Benjamin Jack Cullen

    Intended to be MISRA Compliant (untested, unverified, in-progress).
*/

#ifndef SERIAL_INFOCMD_H
#define SERIAL_INFOCMD_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/**
 * @struct Serial0Struct
 *
 * Working state for the serial command line. BUFFER holds the most recent
 * raw line (filled by main.cpp, consumed by CmdProcess()/outputSentences());
 * checksum holds the hex checksum most recently produced by
 * createChecksumSerial0() for the sentence currently being built. All other
 * checksum/XOR scratch state is local to the functions that compute it.
 */
struct Serial0Struct {
  char BUFFER_RX[MAX_GLOBAL_SERIAL_BUFFER_SIZE];
  char BUFFER_TX[MAX_GLOBAL_SERIAL_BUFFER_SIZE]; // serial buffer.
  char checksum[MAX_CHECKSUM_SIZE];           // hex checksum for the sentence currently being built.
};
extern struct Serial0Struct serial0Data;

/**
 * Serial Output.
 * 
 * @brief Checks flags to see if data should be output over a serial port. 
 */
void outputSentences(void);

/**
 * Command Process.
 * 
 * @brief Checks a serial port to see if commands have been recieved. 
 */
void CmdProcess(void);


void setOverrideOutputValue(int switch_idx, uint32_t override_value);


/**
 * Output Stat.
 * 
 * @brief Prints various infomation for debugging purposes only. 
 */
void outputStat(void);

void setAllSentenceOutput(bool enable);

#endif
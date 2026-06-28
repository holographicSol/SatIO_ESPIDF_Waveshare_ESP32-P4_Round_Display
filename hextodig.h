/*
    HexToDig Library. Written by Benjamin Jack Cullen.

    A small library for converting hex to digits.

    Intended to be MISRA C++ complient.
*/

#ifndef HEX_TO_DIG_H
#define HEX_TO_DIG_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* Sentinel value returned when an invalid hex character is supplied. */
#define H2D_INVALID_VALUE ((uint8_t)0xFFU)

/**
 * Hex to digit.
 *
 * Convert a single hexadecimal character ('0'-'9', 'A'-'F', 'a'-'f')
 * to its decimal equivalent (0-15).
 *
 * @param hex Specify hex value
 * @return Returns uint8_t in range 0-15, or H2D_INVALID_VALUE if hex
 *         is not a valid hexadecimal character.
 */
uint8_t h2d(char hex);

/**
 * 2 Hex to 2 digit.
 *
 * Convert two hexadecimal characters to a single byte.
 *
 * @param h1 Specify hex value 1 (most significant nibble)
 * @param h2 Specify hex value 2 (least significant nibble)
 * @return Returns uint8_t containing the combined byte, or
 *         H2D_INVALID_VALUE if either input is invalid.
 */
uint8_t h2d2(char h1, char h2);

#endif
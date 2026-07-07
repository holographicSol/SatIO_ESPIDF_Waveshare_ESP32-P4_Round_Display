/*
Written by Benjamin Jack Cullen.

GPIO Port Expander - IIC I/O device.

  - Sends pin reads to master.

  - Sets pins from master.

  - PWM.

  For perfomance reasons it is recommnended for single use as either input
  or output device but can be used as both.
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <Wire.h>
#include "./i2c_helper.h"
#include "./gpio_port_expander.h"

#ifdef GPIO_PORT_EXPANDER_WRITE_MODE 
// ------------------------------------------------------------
// Output modulator
// ------------------------------------------------------------
void modulator(GPIOPortExpander *expander) {
  // ------------------------------------------------------------
  // Logic modulator
  // Modulate output only if a switch state is already true.
  // Modulator values: time high, time low.
  // ------------------------------------------------------------
  // Walk only the pins actually modulating (compact list maintained by
  // CMD_WRITE_PIN_PWM/CMD_CLEAR_DATA in gpio_port_expander.cpp) instead of
  // scanning all max_pins slots every pass - the scan itself, not the
  // per-pin logic below, was the measured ~184us fixed cost.
  // One clock read shared by every pin this pass instead of one per
  // active pin - micros() is a syscall-ish read on AVR (briefly disables
  // interrupts), so this cuts up to max_pins reads down to 1.
  const unsigned long now = micros();
  int8_t count;
  const int8_t *active = modulatedPinIndices(&count);
  // Iterate backward: deactivateModulatedPin() below swap-removes the
  // *current* index, moving the list's tail into this slot. Walking high-to-low
  // guarantees that tail element (always at a position we've already visited,
  // or this same one) is never revisited - a forward loop would skip it.
  for (int8_t k=count-1; k>=0; k--) {
    const int8_t i = active[k];
    const int32_t out_val = expander->output_value[i];

    // cache this pin's row instead of re-indexing modulation_time[i] up to 4x
    unsigned long *mt = expander->modulation_time[i];

    const int16_t pin = expander->port_map[i];

    // ------------------------------------------------------
    // handle currently low
    // ------------------------------------------------------
    if (expander->switch_state[i]==false) {
      // ----------------------------------
      // modulate on
      // ----------------------------------
      if ((now - mt[2]) >= mt[0]) {
        // Serial.println("[t0 exceeded (mod on)] idx: " + String(i));
        if (pin<54) {digitalWrite(pin, HIGH);}
        else {analogWrite(pin, out_val);}
        mt[2]=now;
        expander->switch_state[i]=true;
      }
    }
    // -------------------------------------------------------
    // handle currently high
    // -------------------------------------------------------
    else {
      // ----------------------------------
      // remain off
      // ----------------------------------
      if (mt[1]==0) {
        if ((now - mt[2]) >= mt[0]) {
          // Serial.println("[t1 exceeded (remain off)] idx: " + String(i));
          if (pin<54) {digitalWrite(pin, LOW);}
          else {analogWrite(pin, 0);}
          mt[2]=now;
          expander->switch_state[i]=false;
          // change parent state off
          expander->output_value[i]=0;
          // pin is done modulating for good (output_value<=0 now) - stop
          // scanning it. Safe mid-loop: see the backward-iteration note above.
          deactivateModulatedPin(expander, i);
        }
      }
      // ----------------------------------
      // modulate off
      // ----------------------------------
      else {
        if ((now - mt[2]) >= mt[1]) {
          // Serial.println("[t1 exceeded (mod off)] idx: " + String(i));
          if (pin<54) {digitalWrite(pin, LOW);}
          else {analogWrite(pin, 0);}
          mt[2]=now;
          expander->switch_state[i]=false;
        }
      }
    }
  }
  // Serial.println("T " + String(micros()-now));
}
#endif

#ifdef GPIO_PORT_EXPANDER_READ_MODE
#endif
// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP
// ------------------------------------------------------------------------------------------------------------------

void setup() {
  // ------------------------------------------------------------
  // ADC prescaler
  // ------------------------------------------------------------
  // Arduino core default is /128 (125kHz ADC clock, ~104us/conversion) -
  // the datasheet's recommended range for full 10-bit accuracy. /16
  // (1MHz ADC clock) cuts that to ~13us/conversion at the cost of some
  // resolution/noise margin - validate readings against known references
  // if analog precision matters for this build.
  ADCSRA = (ADCSRA & ~0x07) | 0x04; // ADPS2:0 = 100 -> /16
  // ------------------------------------------------------------
  // Serial
  // ------------------------------------------------------------
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200);  while(!Serial);

  // ------------------------------------------------------------
  // I2C
  // ------------------------------------------------------------
  GPIOPortExpander_ATMEGA2560_Default.wire->begin(GPIOPortExpander_ATMEGA2560_Default.address);
  Serial.println("[IIC] Starting IIC as slave address: " + String(GPIOPortExpander_ATMEGA2560_Default.address));
  // ------------------------------------------------------------
  // Function to run when data requested from master
  // ------------------------------------------------------------
  GPIOPortExpander_ATMEGA2560_Default.wire->onRequest(requestEventBus0Bin);
  // ------------------------------------------------------------
  // Function to run when data received from master
  // ------------------------------------------------------------
  GPIOPortExpander_ATMEGA2560_Default.wire->onReceive(receiveEventBus0Bin);

  Serial.println("[READY] Waiting for instructions");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                          MAIN LOOP
// ------------------------------------------------------------------------------------------------------------------

void loop() {
  #ifdef GPIO_PORT_EXPANDER_WRITE_MODE
  modulator(&GPIOPortExpander_ATMEGA2560_Default); // for output: uncomment if required
  #endif
}
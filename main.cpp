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
  for (int i=0; i<expander->max_pins; i++) {
    if (expander->output_value[i]>0) {
      if (expander->modulation_time[i][0] != 0 || expander->modulation_time[i][1] != 0) {
        // ------------------------------------------------------
        // handle currently low
        // ------------------------------------------------------
        if (expander->switch_state[i]==false) {
          // ----------------------------------
          // modulate on
          // ----------------------------------
          if ((micros() - expander->modulation_time[i][2]) >= expander->modulation_time[i][0]) {
            // Serial.println("[t0 exceeded (mod on)] idx: " + String(i));
            if (expander->port_map[i]<54) {digitalWrite(expander->port_map[i], HIGH);}
            else {analogWrite(expander->port_map[i], expander->output_value[i]);}
            expander->modulation_time[i][2]=micros();
            expander->switch_state[i]=true;
          }
        }
        // -------------------------------------------------------
        // handle currently high
        // -------------------------------------------------------
        else if (expander->switch_state[i]==true) {
          // ----------------------------------
          // remain off
          // ----------------------------------
          if (expander->modulation_time[i][1]==0) {
            if ((micros() - expander->modulation_time[i][2]) >= expander->modulation_time[i][0]) {
              // Serial.println("[t1 exceeded (remain off)] idx: " + String(i));
              if (expander->port_map[i]<54) {digitalWrite(expander->port_map[i], LOW);}
              else {analogWrite(expander->port_map[i], 0);}
              expander->modulation_time[i][2]=micros();
              expander->switch_state[i]=false;
              // change parent state off
              expander->output_value[i]=0;
            }
          }
          // ----------------------------------
          // modulate off
          // ----------------------------------
          else {
            if ((micros() - expander->modulation_time[i][2]) >= expander->modulation_time[i][1]) {
              // Serial.println("[t1 exceeded (mod off)] idx: " + String(i));
              if (expander->port_map[i]<54) {digitalWrite(expander->port_map[i], LOW);}
              else {analogWrite(expander->port_map[i], 0);}
              expander->modulation_time[i][2]=micros();
              expander->switch_state[i]=false;
            }
          }
        }
      }
    }
  }
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
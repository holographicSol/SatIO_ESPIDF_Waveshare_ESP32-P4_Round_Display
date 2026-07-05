/*
Written by Benjamin Jack Cullen.

PortController - IIC I/O device.

                 Can send pin readings to master.

                 Can set pin levels according to master.

                 For perfomance reasons it is recommnended for single use as either input
                 or output device but can be used as both.

                 Modulation is currently done here but modulation values may be forwarded to
                 external MCU's on each IO in the future.
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <Wire.h>
#include "./i2c_helper.h"
#include "./gpio_portcontroller.h"

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

// ------------------------------------------------------------
// Reads all analog and digital pins
// ------------------------------------------------------------
void readPins(GPIOPortExpander *expander) {
  int i_counter=0;
  for (int i=0; i<expander->num_digital_pins; i++) {
    expander->input_value[i_counter]=digitalRead(expander->digital_pins[i]);
    i_counter++;
  }
  for (int i=0; i<expander->num_analog_pins; i++) {
    expander->input_value[i_counter]=analogRead(expander->analog_pins[i]);
    i_counter++;
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP
// ------------------------------------------------------------------------------------------------------------------

void setup() {
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
  modulator(&GPIOPortExpander_ATMEGA2560_Default); // for output: uncomment if required
  // readPins(&GPIOPortExpander_ATMEGA2560_Default);  // for input: // uncomment if required
  // Serial.println("loop"+String(millis()));
}
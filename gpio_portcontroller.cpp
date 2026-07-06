/**
 * GPIO Portcontroller - Written by benjamin Jack Cullen.
 * 
 * Intention is to keep everything in one place and in a dedicated library for
 * custom port controllers, while being as generally agnostic as possible.
 * 
 * Dynamically allocating memory has been avoided, with safety as a plus, and extra
 * static code on the downside.
 * 
 * (1) Ensure gpio_portcontroller exists on both master and slave.
 * (2) On the slave, create a new GPIOPortExpander instance, configured for the slave or use the default.
 * (3) Copy the custom/default GPIOPortExpander instance to the masters gpio_portcontroller lib.
 */

#include <string.h>
#include "./gpio_portcontroller.h"

// ------------------------------------------------------------
// Debug
// ------------------------------------------------------------
/**
 * @brief Uncomment to enable debug prints.
 * 
 * GPIO_PORTCONTROLLER_DEBUG_0: error conditions only (unrecognized REQUEST_ID/command, bad packet length).
 * 
 * GPIO_PORTCONTROLLER_DEBUG_1: receiveEventBus0Bin's entry line only (command byte + bytes received).
 * 
 * GPIO_PORTCONTROLLER_DEBUG_2: every other print (per-command state).
 * 
 * @warning Only enable when required, or errors may be received.
 */

#define GPIO_PORTCONTROLLER_DEBUG_0
// #define GPIO_PORTCONTROLLER_DEBUG_1
// #define GPIO_PORTCONTROLLER_DEBUG_2

#ifdef GPIO_PORT_EXPANDER_SLAVE_MODE
// ------------------------------------------------------------
/**
 * @brief GPIOPortExpander_ATMEGA2560 (DEFAULT)
 *        This is how to create a GPIOPortExpander instance.
 *        To setup gpio_portcontroller using a different MCU,
 *        create a custom instance.
 */
GPIOPortExpander GPIOPortExpander_ATMEGA2560_Default = {
    "GPIOPortExpander_ATMEGA2560_Default",
    &iic_0,
    I2CLinkBus0,
    11, // address
    0,  // current_pin
    0,  // pin_min
    69, // pin_max
    MAX_GPIOPortExpander_ATMEGA2560_Default_PINS, // max_pins
    16, // number of analog pins
    54, // number of digital pins
    (int8_t[]){54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69}, // analog_pins
    (int8_t[]){ // digital_pins
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
        10,11,12,13,14,15,16,17,18,19,
        20,21,22,23,24,25,26,27,28,29,
        30,31,32,33,34,35,36,37,38,39,
        40,41,42,43,44,45,46,47,48,49,
        50,51,52,53
    },
    (unsigned long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS][3]){}, // modulation_time
    (long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // input_value
    (long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // output_value
    (int[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){ // port_map (default no port)
      0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,
      21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,
      39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,
      57,58,59,60,61,62,63,64,65,66,67,68,69
    },
    (bool[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // switch_state
    0             // query_cursor
};
#endif

#ifdef GPIO_PORT_EXPANDER_MASTER_MODE
// ------------------------------------------------------------
/**
 * @brief GPIOPortExpander_ATMEGA2560 (Custom)
 */
IICLink IICLinkPCI; // IIC link data structure for the GPIOPortExpander.
GPIOPortExpander GPIOPortExpander_ATMEGA2560_Input_0 = {
    "GPIOPortExpander_ATMEGA2560_Input_0",
    &iic_0,
    IICLinkPCI,
    11, // address
    0,  // current_pin
    0,  // pin_min
    69, // pin_max
    MAX_GPIOPortExpander_ATMEGA2560_Default_PINS, // max_pins
    16, // number of analog pins
    54, // number of digital pins
    (int8_t[]){54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69}, // analog_pins
    (int8_t[]){ // digital_pins
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
        10,11,12,13,14,15,16,17,18,19,
        20,21,22,23,24,25,26,27,28,29,
        30,31,32,33,34,35,36,37,38,39,
        40,41,42,43,44,45,46,47,48,49,
        50,51,52,53
    },
    (unsigned long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS][3]){}, // modulation_time
    (long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // input_value
    (long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // output_value
    (int[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){ // port_map (default no port)
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 0-9
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 10-19
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 20-29
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 30-39
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 40-49
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 50-59
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 60-69
    },
    (bool[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // switch_state
    0 // query_cursor
};

// ------------------------------------------------------------
/**
 * @brief GPIOPortExpander_ATMEGA2560 (Custom)
 */
IICLink IICLinkPCO_0; // IIC link data structure for the GPIOPortExpander.
GPIOPortExpander GPIOPortExpander_ATMEGA2560_Output_0 = {
    "GPIOPortExpander_ATMEGA2560_Output_0",
    &iic_2,
    IICLinkPCO_0,
    9,  // address
    0,  // current_pin
    0,  // pin_min
    69, // pin_max
    MAX_GPIOPortExpander_ATMEGA2560_Default_PINS, // max_pins
    16, // number of analog pins
    54, // number of digital pins
    (int8_t[]){54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69}, // analog_pins
    (int8_t[]){ // digital_pins
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
        10,11,12,13,14,15,16,17,18,19,
        20,21,22,23,24,25,26,27,28,29,
        30,31,32,33,34,35,36,37,38,39,
        40,41,42,43,44,45,46,47,48,49,
        50,51,52,53
    },
    (unsigned long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS][3]){}, // modulation_time
    (long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // input_value
    (long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // output_value
    (int[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){ // port_map (default no port)
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 0-9
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 10-19
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 20-29
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 30-39
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 40-49
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 50-59
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 60-69
    },
    (bool[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // switch_state
    0 // query_cursor
};

// ------------------------------------------------------------
/**
 * @brief GPIOPortExpander_ATMEGA2560 (Custom)
 */
IICLink IICLinkPCO_1; // IIC link data structure for the GPIOPortExpander.
GPIOPortExpander GPIOPortExpander_ATMEGA2560_Output_1 = {
    "GPIOPortExpander_ATMEGA2560_Output_1",
    &iic_2,
    IICLinkPCO_1,
    10, // address
    0,  // current_pin
    0,  // pin_min
    69, // pin_max
    MAX_GPIOPortExpander_ATMEGA2560_Default_PINS, // max_pins
    16, // number of analog pins
    54, // number of digital pins
    (int8_t[]){54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69}, // analog_pins
    (int8_t[]){ // digital_pins
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
        10,11,12,13,14,15,16,17,18,19,
        20,21,22,23,24,25,26,27,28,29,
        30,31,32,33,34,35,36,37,38,39,
        40,41,42,43,44,45,46,47,48,49,
        50,51,52,53
    },
    (unsigned long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS][3]){}, // modulation_time
    (long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // input_value
    (long[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // output_value
    (int[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){ // port_map (default no port)
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 0-9
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 10-19
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 20-29
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 30-39
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 40-49
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 50-59
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 60-69
    },
    (bool[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS]){}, // switch_state
    0 // query_cursor
};
#endif

#ifdef GPIO_PORT_EXPANDER_SLAVE_MODED
// --------------------------------------------------------------------
// Inline binary search – compiles to ~10–15 instructions
// --------------------------------------------------------------------
inline bool isAnalogPin(int8_t analog_pins[], int8_t num_analog_pins, uint8_t pin) {
  for (int i=0; i<num_analog_pins; i++) {
    if (pin==(uint8_t)analog_pins[i]) {return true;}
  }
  return false;
}
inline bool isDigitalPin(int8_t digital_pins[], int8_t num_digital_pins, uint8_t pin) {
  for (int i=0; i<num_digital_pins; i++) {
    if (pin==(uint8_t)digital_pins[i]) {return true;}
  }
  return false;
}
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
// --------------------------------------------------------------------
// O(1) pin-kind lookup, keyed directly by physical pin number - built once
// from analog_pins/digital_pins so readPin() doesn't re-scan up to 70
// entries (isAnalogPin + isDigitalPin) on every single I2C request.
// --------------------------------------------------------------------
constexpr int8_t PIN_KIND_UNKNOWN = 0;
constexpr int8_t PIN_KIND_ANALOG  = 1;
constexpr int8_t PIN_KIND_DIGITAL = 2;
static int8_t pin_kind_lookup[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static bool pin_kind_lookup_built = false;

void buildPinKindLookup(GPIOPortExpander *gpio_expander) {
  memset(pin_kind_lookup, PIN_KIND_UNKNOWN, sizeof(pin_kind_lookup));
  for (int i=0; i<gpio_expander->num_analog_pins; i++) {
    uint8_t pin = (uint8_t)gpio_expander->analog_pins[i];
    if (pin < MAX_GPIOPortExpander_ATMEGA2560_Default_PINS) {pin_kind_lookup[pin] = PIN_KIND_ANALOG;}
  }
  for (int i=0; i<gpio_expander->num_digital_pins; i++) {
    uint8_t pin = (uint8_t)gpio_expander->digital_pins[i];
    if (pin < MAX_GPIOPortExpander_ATMEGA2560_Default_PINS) {pin_kind_lookup[pin] = PIN_KIND_DIGITAL;}
  }
  pin_kind_lookup_built = true;
}

inline void readPin(GPIOPortExpander *gpio_expander) {
  if (!pin_kind_lookup_built) {buildPinKindLookup(gpio_expander);}
  int mapped_pin = gpio_expander->port_map[gpio_expander->current_pin];
  if (mapped_pin < 0 || mapped_pin >= MAX_GPIOPortExpander_ATMEGA2560_Default_PINS) {return;}
  switch (pin_kind_lookup[mapped_pin]) {
    case PIN_KIND_ANALOG:
      gpio_expander->input_value[gpio_expander->current_pin] = analogRead((uint8_t)mapped_pin);
      break;
    case PIN_KIND_DIGITAL:
      gpio_expander->input_value[gpio_expander->current_pin] = digitalRead((uint8_t)mapped_pin);
      break;
  }
}

/** ----------------------------------------------------------------------------
 * @brief Request binary event handler for an I2C Bus.
 *
 * @note Registered directly with Wire.onRequest(), which requires this exact
 *       void() signature, so it can't take a GPIOPortExpander* parameter -
 *       it operates on a GPIOPortExpander inside its function.
 * 
 *       This function is bus agnostic, however the functions name conveys a bus
 *       for clarity, and can be reproduced (copy/paste) as many times as required,
 *       with same/differently configured GPIOPortExpander, for any required
 *       compatible I2C bus and MCU.
 */
void requestEventBus0Bin() {
  GPIOPortExpander &gpio_expander = GPIOPortExpander_ATMEGA2560_Default;

  switch (gpio_expander.i2cLink.REQUEST_ID) {

    // Commands 0-69 - Send reading for the directly-addressed pin (current_pin already set by matching receive command)
    case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: // 0-9
    case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: case 19: // 10-19
    case 20: case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 28: case 29: // 20-29
    case 30: case 31: case 32: case 33: case 34: case 35: case 36: case 37: case 38: case 39: // 30-39
    case 40: case 41: case 42: case 43: case 44: case 45: case 46: case 47: case 48: case 49: // 40-49
    case 50: case 51: case 52: case 53: case 54: case 55: case 56: case 57: case 58: case 59: // 50-59
    case 60: case 61: case 62: case 63: case 64: case 65: case 66: case 67: case 68: case 69: { // 60-69
        #ifdef GPIO_PORTCONTROLLER_DEBUG_2
        Serial.println("[CASE 0-N] sending: pin=" + String(gpio_expander.current_pin)  + "  value=" + String(gpio_expander.input_value[gpio_expander.current_pin]));
        #endif
        clearI2CLinkOutputPacket(gpio_expander.i2cLink);
        write_float_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 0, (float)gpio_expander.input_value[gpio_expander.current_pin]);
        writeI2CToMasterBin(Wire, gpio_expander.i2cLink, 4, 0);
        break;
      }

    // Command 120 - Send pin reading to master and step current pin
    case CMD_RESET_CURRENT_PIN: {
        #ifdef GPIO_PORTCONTROLLER_DEBUG_2
        Serial.println("[CMD_RESET_CURRENT_PIN] sending: pin=" + String(gpio_expander.current_pin)  + "  value=" + String(gpio_expander.input_value[gpio_expander.current_pin]));
        #endif

        #ifdef GPIO_PORTCONTROLLER_BENCH
        int32_t t0 = micros();
        #endif
        readPin(&gpio_expander);
        #ifdef GPIO_PORTCONTROLLER_BENCH
        Serial.println("RP " + String(micros()-t0)); // read pin time uS
        #endif

        #ifdef GPIO_PORTCONTROLLER_BENCH
        t0 = micros();
        #endif
        // (offsets 0-4) are fully overwritten by the two writes below, so
        // zeroing the 32-byte buffer first bought nothing but ~6-9us.
        clearI2CLinkOutputPacket(gpio_expander.i2cLink); // dropped here: the 5 bytes actually sent
        write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 0, gpio_expander.current_pin);
        write_float_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 1, (float)gpio_expander.input_value[gpio_expander.current_pin]);
        writeI2CToMasterBin(Wire, gpio_expander.i2cLink, 5, 0);
        #ifdef GPIO_PORTCONTROLLER_BENCH
        Serial.println("SN " + String(micros()-t0)); // write time uS
        #endif
        if (++gpio_expander.current_pin >= gpio_expander.num_analog_pins + gpio_expander.num_digital_pins) {gpio_expander.current_pin = 0;}
        break;
      }

    // Command 130 - Send expander header: pin_min, pin_max, max_pins, n_analog, n_digital
    case CMD_GET_EXPANDER_INFO: {
        #ifdef GPIO_PORTCONTROLLER_DEBUG_2
        Serial.println(
          "[CMD_GET_EXPANDER_INFO]  pin_min=" + String(gpio_expander.pin_min) +
          "  pin_max=" + String(gpio_expander.pin_max) +
          "  max_pins=" + String((uint8_t)gpio_expander.max_pins) +
          "  num_analog_pins=" + String(gpio_expander.num_analog_pins) +
          "  num_digital_pins=" + String(gpio_expander.num_digital_pins)
        );
        #endif
        clearI2CLinkOutputPacket(gpio_expander.i2cLink);
        write_int8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 0, gpio_expander.pin_min);
        write_int8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 1, gpio_expander.pin_max);
        write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 2, (uint8_t)gpio_expander.max_pins);
        write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 3, gpio_expander.num_analog_pins);
        write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 4, gpio_expander.num_digital_pins);
        writeI2CToMasterBin(Wire, gpio_expander.i2cLink, 5, 0);
        break;
      }

    // Command 140 - Send one (is_analog, pin) entry at query_cursor, then advance it
    case CMD_GET_EXPANDER_PIN_LIST: {
        clearI2CLinkOutputPacket(gpio_expander.i2cLink);
        if (gpio_expander.query_cursor < gpio_expander.num_analog_pins) {
          #ifdef GPIO_PORTCONTROLLER_DEBUG_2
          Serial.println("[CMD_GET_EXPANDER_PIN_LIST]  is_analog=1  pin=" + String(gpio_expander.analog_pins[gpio_expander.query_cursor]));
          #endif
          write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 0, 1);
          write_int8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 1, gpio_expander.analog_pins[gpio_expander.query_cursor]);
        } else {
          #ifdef GPIO_PORTCONTROLLER_DEBUG_2
          Serial.println("[CMD_GET_EXPANDER_PIN_LIST]  is_analog=0  pin=" + String(gpio_expander.digital_pins[gpio_expander.query_cursor - gpio_expander.num_analog_pins]));
          #endif
          write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 0, 0);
          write_int8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 1, gpio_expander.digital_pins[gpio_expander.query_cursor - gpio_expander.num_analog_pins]);
        }
        writeI2CToMasterBin(Wire, gpio_expander.i2cLink, 2, 0);
        if (++gpio_expander.query_cursor >= gpio_expander.num_analog_pins + gpio_expander.num_digital_pins) {gpio_expander.query_cursor = 0;}
        break;
      }

    default: {
        #ifdef GPIO_PORTCONTROLLER_DEBUG_0
        Serial.println("[requestEventBus0Bin] event id is not defined: " + String(gpio_expander.i2cLink.REQUEST_ID));
        #endif
        while (Wire.available()) {Wire.read();} // drain
        break;
    }
  }
}

/** ----------------------------------------------------------------------------
 * @brief Receive binary event handler for an I2C Bus
 *
 * @note Registered directly with Wire.onReceive(), which requires this exact
 *       void(int) signature, so it can't take a GPIOPortExpander* parameter -
 *       it operates on this slave's own GPIOPortExpander_ATMEGA2560 by name.
 * 
 *       This function is bus agnostic, however the functions name conveys a bus
 *       for clarity, and can be reproduced (copy/paste) as many times as required,
 *       with same/differently configured GPIOPortExpander, for any required
 *       compatible I2C bus and MCU.
*/
void receiveEventBus0Bin(int n_bytes_received) {
  GPIOPortExpander &gpio_expander = GPIOPortExpander_ATMEGA2560_Default;
  if (n_bytes_received < 1) return;
  uint8_t cmd = gpio_expander.wire->read(); // expects uint8 command byte (up to 255 unique commands can be accepted).
  #ifdef GPIO_PORTCONTROLLER_DEBUG_1
  Serial.println("[receiveEventBus0Bin] " + String(cmd) + " (" + String(n_bytes_received) + " bytes)");
  #endif
  switch (cmd) {

    /**
     * @brief Command 100 - Clear Data
     */
    case CMD_CLEAR_DATA: {
      #ifdef GPIO_PORTCONTROLLER_DEBUG_2
      Serial.println("[CMD_CLEAR_DATA]");
      #endif
      for (int i = 0; i < gpio_expander.max_pins; i++) {
        gpio_expander.port_map[i] = -1;
        gpio_expander.output_value[i] = 0;
        gpio_expander.modulation_time[i][0] = 0;
        gpio_expander.modulation_time[i][1] = 0;
        gpio_expander.modulation_time[i][2] = 0;
        gpio_expander.switch_state[i] = false;
      }
      gpio_expander.current_pin = 0;
      while (gpio_expander.wire->available()) gpio_expander.wire->read();  // flush
      break;
    }

    /**
     * @brief Command 110 - Write value to pin & Set PWM
     */
    case CMD_WRITE_PIN_PWM: {
      if (n_bytes_received != 15) {
        while (gpio_expander.wire->available()) gpio_expander.wire->read();
        #ifdef GPIO_PORTCONTROLLER_DEBUG_0
        Serial.println("!=15");
        #endif
        return;
      }
      
      uint8_t idx;
      read_uint8_FromWire(*gpio_expander.wire, idx);
      
      int8_t pin;
      read_int8_FromWire(*gpio_expander.wire, pin);
      
      int32_t value;
      read_int32_FromWire(*gpio_expander.wire, value);
      
      uint32_t off_time;
      read_uint32_FromWire(*gpio_expander.wire, off_time);
      
      uint32_t on_time;
      read_uint32_FromWire(*gpio_expander.wire, on_time);
      
      #ifdef GPIO_PORTCONTROLLER_DEBUG_2
      Serial.println(
        "[CMD_WRITE_PIN_PWM]  idx=" + String(idx) +
        "  pin=" + String(pin) +
        "  value=" + String(value) +
        "  off_time=" + String(off_time) +
        "  on_time=" + String(on_time)
      );
      #endif

      if (idx >= gpio_expander.max_pins) return;

      gpio_expander.port_map[idx]           = pin;
      gpio_expander.output_value[idx]       = value;
      gpio_expander.modulation_time[idx][0] = off_time;
      gpio_expander.modulation_time[idx][1] = on_time;

      // Set digital pin
      if (isDigitalPin(gpio_expander.digital_pins, gpio_expander.num_digital_pins, gpio_expander.port_map[idx])) {
        int current_input_value = digitalRead(gpio_expander.port_map[idx]);
        if ( (current_input_value==1) && (gpio_expander.output_value[idx]==0) ) {
          pinMode(gpio_expander.port_map[idx], OUTPUT); // new
          digitalWrite(gpio_expander.port_map[idx], LOW);
          gpio_expander.modulation_time[idx][2]=0;
        }
        else if ( (current_input_value==0) && (gpio_expander.output_value[idx]==1) ) {
          pinMode(gpio_expander.port_map[idx], OUTPUT); // new
          digitalWrite(gpio_expander.port_map[idx], HIGH);
          gpio_expander.modulation_time[idx][2]=0;
        }
      }
      // Set analog pin
      else if (isAnalogPin(gpio_expander.analog_pins, gpio_expander.num_analog_pins, gpio_expander.port_map[idx])) {
        pinMode(gpio_expander.port_map[idx], OUTPUT); // new
        analogWrite(gpio_expander.port_map[idx], gpio_expander.output_value[idx]);
      }
      break;
    }

    /**
     * @brief Command 120 - Reset current pin & set request ID 120 ready for a request.
     * @note This is useful if master does not need any specific value right now, but
     *       does need to know which value it is receiving.
     */
    case CMD_RESET_CURRENT_PIN: {
      #ifdef GPIO_PORTCONTROLLER_DEBUG_2
      Serial.println("[CMD_RESET_CURRENT_PIN]");
      #endif
      gpio_expander.current_pin = 0;
      gpio_expander.i2cLink.REQUEST_ID = CMD_RESET_CURRENT_PIN;
      while (gpio_expander.wire->available()) {gpio_expander.wire->read();}
      break;
    }

    /**
     * @brief Command 130 - Reset expander-info cursor & set request ID ready for a request.
     */
    case CMD_GET_EXPANDER_INFO: {
      #ifdef GPIO_PORTCONTROLLER_DEBUG_2
      Serial.println("[CMD_GET_EXPANDER_INFO]");
      #endif
      gpio_expander.query_cursor = 0;
      gpio_expander.i2cLink.REQUEST_ID = CMD_GET_EXPANDER_INFO;
      while (gpio_expander.wire->available()) {gpio_expander.wire->read();}
      break;
    }

    /**
     * @brief Command 140 - Reset expander pin-list cursor & set request ID ready for a request.
     */
    case CMD_GET_EXPANDER_PIN_LIST: {
      #ifdef GPIO_PORTCONTROLLER_DEBUG_2
      Serial.println("[CMD_GET_EXPANDER_PIN_LIST]");
      #endif
      gpio_expander.query_cursor = 0;
      gpio_expander.i2cLink.REQUEST_ID = CMD_GET_EXPANDER_PIN_LIST;
      while (gpio_expander.wire->available()) {gpio_expander.wire->read();}
      break;
    }

    /**
     * @brief Commands 0-69 - Directly set current pin & matching request ID, ready for a request.
     */
    case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: // 0-9
    case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: case 19: // 10-19
    case 20: case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 28: case 29: // 20-29
    case 30: case 31: case 32: case 33: case 34: case 35: case 36: case 37: case 38: case 39: // 30-39
    case 40: case 41: case 42: case 43: case 44: case 45: case 46: case 47: case 48: case 49: // 40-49
    case 50: case 51: case 52: case 53: case 54: case 55: case 56: case 57: case 58: case 59: // 50-59
    case 60: case 61: case 62: case 63: case 64: case 65: case 66: case 67: case 68: case 69: { // 60-69
      gpio_expander.current_pin = cmd;
      gpio_expander.i2cLink.REQUEST_ID = cmd;
      #ifdef GPIO_PORTCONTROLLER_DEBUG_2
      Serial.println("[CASE 0-N]  current_pin=" + String(gpio_expander.current_pin) + "  REQUEST_ID=" + String(gpio_expander.i2cLink.REQUEST_ID));
      #endif
      while (gpio_expander.wire->available()) {gpio_expander.wire->read();}
      break;
    }

    // Default: flush
    default: {
      while (gpio_expander.wire->available()) {gpio_expander.wire->read();}
      break;
    }
  }
}
#endif

#ifdef GPIO_PORT_EXPANDER_MASTER_MODE
// ------------------------------------------------------------
// Master-side: query a GPIOPortExpander's configuration live over I2C.
// Reused sequentially across calls, same convention as IICLinkPCO in matrix.cpp.
// ------------------------------------------------------------
IICLink IICLinkGPIOExpander;

bool queryGPIOPortExpander(TwoWire &wire, int address, const char *name, GPIOPortExpander *out,
                            int8_t *analog_buf, int8_t *digital_buf,
                            unsigned long (*modulation_buf)[3],
                            long *input_buf, long *output_buf,
                            int *port_map_buf, bool *switch_state_buf) {
  // Phase 1: header - pin_min, pin_max, max_pins, n_analog, n_digital
  clearI2CLinkOutputPacket(IICLinkGPIOExpander);
  write_uint8_ToPacket(IICLinkGPIOExpander.OUTPUT_PACKET, 0, CMD_GET_EXPANDER_INFO);
  writeI2CToSlaveBin(wire, IICLinkGPIOExpander, address, 1, 0, "queryGPIOPortExpander");

  if (!requestFromSlaveBinNoID(wire, IICLinkGPIOExpander, address, 5, 0, "queryGPIOPortExpander")) {
    return false;
  }

  int8_t pin_min, pin_max;
  uint8_t max_pins, n_analog, n_digital;
  read_int8_FromWire(wire, pin_min);
  read_int8_FromWire(wire, pin_max);
  read_uint8_FromWire(wire, max_pins);
  read_uint8_FromWire(wire, n_analog);
  read_uint8_FromWire(wire, n_digital);

  memset(out->name, 0, sizeof(out->name));
  strncpy(out->name, name, sizeof(out->name) - 1);
  out->pin_min = pin_min;
  out->pin_max = pin_max;
  out->max_pins = max_pins;
  out->num_analog_pins = n_analog;
  out->num_digital_pins = n_digital;
  out->analog_pins     = analog_buf;
  out->digital_pins    = digital_buf;
  out->modulation_time = modulation_buf;
  out->input_value     = input_buf;
  out->output_value    = output_buf;
  out->port_map        = port_map_buf;
  out->switch_state    = switch_state_buf;
  memset(analog_buf, 0, MAX_GPIOPortExpander_ATMEGA2560_Default_PINS * sizeof(int8_t));
  memset(digital_buf, 0, MAX_GPIOPortExpander_ATMEGA2560_Default_PINS * sizeof(int8_t));
  memset(modulation_buf, 0, MAX_GPIOPortExpander_ATMEGA2560_Default_PINS * sizeof(*modulation_buf));
  memset(input_buf, 0, MAX_GPIOPortExpander_ATMEGA2560_Default_PINS * sizeof(long));
  memset(output_buf, 0, MAX_GPIOPortExpander_ATMEGA2560_Default_PINS * sizeof(long));
  memset(switch_state_buf, 0, MAX_GPIOPortExpander_ATMEGA2560_Default_PINS * sizeof(bool));
  for (int i = 0; i < MAX_GPIOPortExpander_ATMEGA2560_Default_PINS; i++) {port_map_buf[i] = -1;} // -1 = unmapped

  // Phase 2: pin list - n_analog + n_digital entries, 2 bytes each (is_analog, pin)
  clearI2CLinkOutputPacket(IICLinkGPIOExpander);
  write_uint8_ToPacket(IICLinkGPIOExpander.OUTPUT_PACKET, 0, CMD_GET_EXPANDER_PIN_LIST);
  writeI2CToSlaveBin(wire, IICLinkGPIOExpander, address, 1, 0, "queryGPIOPortExpander");

  uint8_t i_analog = 0, i_digital = 0;
  for (uint8_t i = 0; i < (uint8_t)(n_analog + n_digital); i++) {
    if (!requestFromSlaveBinNoID(wire, IICLinkGPIOExpander, address, 2, 0, "queryGPIOPortExpander")) {
      continue;
    }
    uint8_t is_analog;
    int8_t pin;
    read_uint8_FromWire(wire, is_analog);
    read_int8_FromWire(wire, pin);
    if (is_analog && i_analog < n_analog && i_analog < MAX_GPIOPortExpander_ATMEGA2560_Default_PINS) {
      out->analog_pins[i_analog++] = pin;
    } else if (!is_analog && i_digital < n_digital && i_digital < MAX_GPIOPortExpander_ATMEGA2560_Default_PINS) {
      out->digital_pins[i_digital++] = pin;
    }
  }

  return true;
}

bool readGPIOPortExapander_All(GPIOPortExpander &gpio_expander) {

  bool ok = true; // assume true except even just one bad read (replace with counter)

  // Send event ID once to set a mode on receiver.
  clearI2CLinkOutputPacket(gpio_expander.i2cLink);
  gpio_expander.i2cLink.OUTPUT_PACKET[0] = 0x78; // command 120 (reset and iter current pin)
  // printf("writeI2CToSlaveBin\n");
  writeI2CToSlaveBin(*gpio_expander.wire, gpio_expander.i2cLink, gpio_expander.address, 1, 0, "readInputPortControllerM1");

  for (uint8_t i = 0; i < (uint8_t)gpio_expander.max_pins; i++) {
    // printf("requestFromSlaveBinNoID\n");
    if (requestFromSlaveBinNoID(*gpio_expander.wire, gpio_expander.i2cLink, gpio_expander.address, 5, 0, "readInputPortControllerM1")) {
      uint8_t pin;
      // printf("read_uint8_FromWire\n");
      read_uint8_FromWire(*gpio_expander.wire, pin);
      float value;
      // printf("read_float_FromWire\n");
      read_float_FromWire(*gpio_expander.wire, value);
      // Bound against the device's own discovered pin count (matches the
      // input_portcontroller_0.input_value array's real capacity), not
      // MAX_MATRIX_SWITCHES - a physical pin can exceed the switch count.
      if (pin < gpio_expander.max_pins) {
        gpio_expander.input_value[pin] = (long)value;
      }
      // printf("readInputPortControllerReadPins: pin=%d   value=%f\n", pin, value);
    }
    else {
      ok = false;
      printf("ERROR: readInputPortControllerReadPins (pin_index=%d)\n", i);
    }
  }
  return ok;
}

void clearGPIOPortController(GPIOPortExpander &gpio_expander) {
  clearI2CLinkOutputPacket(gpio_expander.i2cLink);
  write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 0, 0x64); // command 100
  writeI2CToSlaveBin(*gpio_expander.wire, gpio_expander.i2cLink, gpio_expander.address, 1, 0, "clearGPIOPortController");
}

/**
 * @brief A placeholder for writing to a GPIOPortExpander.
 */
int32_t writeGPIOPortExapander_All(GPIOPortExpander &gpio_expander) {
  int32_t count_write=0;

  for (int32_t Mi = 0; Mi < gpio_expander.max_pins; Mi++) {

    int address = gpio_expander.address;
    int32_t value_to_send = 0;
    uint32_t off_time = 0;
    uint32_t on_time = 0;

    // Build binary packet with human readable helper functions.
    write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 0, 0x6E); // command 110
    write_uint8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 1, (uint8_t)Mi);
    write_int8_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 2, (int8_t)gpio_expander.port_map[Mi]);
    write_int32_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 3, value_to_send);
    write_uint32_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 7, off_time);
    write_uint32_ToPacket(gpio_expander.i2cLink.OUTPUT_PACKET, 11, on_time);
    // Write to slave.
    writeI2CToSlaveBin(*gpio_expander.wire, gpio_expander.i2cLink, address, 15, 0, gpio_expander.name);

    count_write++;

  }
  return count_write;
}

// ------------------------------------------------------------
// Master-side: named instances for the known physical devices, each with
// its own dedicated static backing storage (MISRA Rule 21.3: no dynamic
// allocation - a buffer that must outlive a single call is a static
// instead of a heap allocation).
// ------------------------------------------------------------

static int8_t        GPIOPortExpander_ATMEGA2560_Output_0_analog_pins[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static int8_t        GPIOPortExpander_ATMEGA2560_Output_0_digital_pins[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static unsigned long GPIOPortExpander_ATMEGA2560_Output_0_modulation_time[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS][3];
static long          GPIOPortExpander_ATMEGA2560_Output_0_input_value[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static long          GPIOPortExpander_ATMEGA2560_Output_0_output_value[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static int           GPIOPortExpander_ATMEGA2560_Output_0_port_map[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static bool          GPIOPortExpander_ATMEGA2560_Output_0_switch_state[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];

static int8_t        GPIOPortExpander_ATMEGA2560_Output_1_analog_pins[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static int8_t        GPIOPortExpander_ATMEGA2560_Output_1_digital_pins[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static unsigned long GPIOPortExpander_ATMEGA2560_Output_1_modulation_time[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS][3];
static long          GPIOPortExpander_ATMEGA2560_Output_1_input_value[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static long          GPIOPortExpander_ATMEGA2560_Output_1_output_value[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static int           GPIOPortExpander_ATMEGA2560_Output_1_port_map[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static bool          GPIOPortExpander_ATMEGA2560_Output_1_switch_state[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];

static int8_t        GPIOPortExpander_ATMEGA2560_Input_0_analog_pins[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static int8_t        GPIOPortExpander_ATMEGA2560_Input_0_digital_pins[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static unsigned long GPIOPortExpander_ATMEGA2560_Input_0_modulation_time[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS][3];
static long          GPIOPortExpander_ATMEGA2560_Input_0_input_value[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static long          GPIOPortExpander_ATMEGA2560_Input_0_output_value[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static int           GPIOPortExpander_ATMEGA2560_Input_0_port_map[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];
static bool          GPIOPortExpander_ATMEGA2560_Input_0_switch_state[MAX_GPIOPortExpander_ATMEGA2560_Default_PINS];

void initGPIOPortExpanders(
  TwoWire &wire_output_0,
  TwoWire &wire_input_0,
  int addr_output_0, int addr_output_1,
  int addr_input_0
)
{
  queryGPIOPortExpander(wire_output_0, addr_output_0, "GPIOPortExpander_ATMEGA2560_Output_0", &GPIOPortExpander_ATMEGA2560_Output_0,
                        GPIOPortExpander_ATMEGA2560_Output_0_analog_pins, GPIOPortExpander_ATMEGA2560_Output_0_digital_pins,
                        GPIOPortExpander_ATMEGA2560_Output_0_modulation_time, GPIOPortExpander_ATMEGA2560_Output_0_input_value,
                        GPIOPortExpander_ATMEGA2560_Output_0_output_value,
                        GPIOPortExpander_ATMEGA2560_Output_0_port_map, GPIOPortExpander_ATMEGA2560_Output_0_switch_state);

  queryGPIOPortExpander(wire_output_0, addr_output_1, "GPIOPortExpander_ATMEGA2560_Output_1", &GPIOPortExpander_ATMEGA2560_Output_1,
                        GPIOPortExpander_ATMEGA2560_Output_1_analog_pins, GPIOPortExpander_ATMEGA2560_Output_1_digital_pins,
                        GPIOPortExpander_ATMEGA2560_Output_1_modulation_time, GPIOPortExpander_ATMEGA2560_Output_1_input_value,
                        GPIOPortExpander_ATMEGA2560_Output_1_output_value,
                        GPIOPortExpander_ATMEGA2560_Output_1_port_map, GPIOPortExpander_ATMEGA2560_Output_1_switch_state);

  queryGPIOPortExpander(wire_input_0, addr_input_0, "GPIOPortExpander_ATMEGA2560_Input_0", &GPIOPortExpander_ATMEGA2560_Input_0,
                        GPIOPortExpander_ATMEGA2560_Input_0_analog_pins, GPIOPortExpander_ATMEGA2560_Input_0_digital_pins,
                        GPIOPortExpander_ATMEGA2560_Input_0_modulation_time, GPIOPortExpander_ATMEGA2560_Input_0_input_value,
                        GPIOPortExpander_ATMEGA2560_Input_0_output_value,
                        GPIOPortExpander_ATMEGA2560_Input_0_port_map, GPIOPortExpander_ATMEGA2560_Input_0_switch_state);
}
#endif
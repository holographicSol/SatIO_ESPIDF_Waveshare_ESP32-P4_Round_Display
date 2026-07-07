/**
 * GPIO Port Expander - Written by benjamin Jack Cullen.
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

#ifndef __GPIO_PORT_EXPANDER__
#define __GPIO_PORT_EXPANDER__

#include <stdint.h>
#include <stdbool.h>
#include "./i2c_helper.h"

/**
 * @brief GPIOPortExpander - A type to setup a gpio_portcontroller instance.
 * @note Can be created as required for matching a specific MCU's specifications.
 *       Default type included is configured for ATMEGA2560.
 *       If this device will be a slave then create an instance and copy it to master,
 *       for gpio_portcontroller to use on the master side too.
 */
typedef struct GPIOPortExpander {
    char name[56];
    TwoWire *wire; // no default value: a default member initializer would
                   // disqualify this struct as an aggregate under C++11
                   // (this toolchain's default standard), breaking every
                   // positional brace-initializer below
    IICLink i2cLink;
    int8_t address;
    int16_t current_pin;
    int16_t pin_min;
    int16_t pin_max;
    int16_t max_pins;
    int16_t num_analog_pins;
    int16_t num_digital_pins;
    int16_t *analog_pins;
    int16_t *digital_pins;
    unsigned long (*modulation_time)[3];
    int32_t *input_value;
    int32_t *output_value;
    int16_t *port_map;        // logical index -> physical pin, -1 = unmapped
    bool *switch_state;   // per-pin modulation on/off tracking
    uint64_t *chan_freq_uS; // per-pin minimum microseconds between accepted
                            // reads (see setGPIOPortExpanderChannelFreq());
                            // 0 = no floor, i.e. accept every read. Master-side
                            // only - the slave never reads this field.
    int8_t query_cursor;  // cursor for CMD_GET_EXPANDER_PIN_LIST streaming,
                          // kept separate from current_pin so a discovery
                          // query never interferes with the normal
                          // pin-value-read protocol (CMD_RESET_CURRENT_PIN)
} GPIOPortExpander;

// ------------------------------------------------------------
// Pointer members above are sized per-instance via compound literals
// (see the definition in gpio_portcontroller.cpp) - exactly as many
// elements as listed, so a board with fewer pins never pays for a
// MAX_GPIO_PORTCONTROLLER_PINS-sized allocation.
// ------------------------------------------------------------

// ------------------------------------------------------------
// Master-side: named GPIOPortExpander instances for the known physical
// devices, auto-populated over I2C. Addresses are passed in (not hardcoded
// here) so this file stays identical across projects regardless of which
// project-specific I2C_ADDR_* macros exist. On a slave build these are
// simply unused and get dead-stripped.
// ------------------------------------------------------------

// ------------------------------------------------------------
/**
 * @brief BUILD OPTIONS
 */
// ------------------------------------------------------------
/**
 * @brief GPIO_PORT_EXPANDER_MASTER_MODE - Setup to control a GPIO Expander module.
 */
// #define GPIO_PORT_EXPANDER_MASTER_MODE

/**
 * @brief GPIO_PORT_EXPANDER_SLAVE_MODE - Setup to be controlled by a master.
 */
#define GPIO_PORT_EXPANDER_SLAVE_MODE

/**
 * @brief GPIO_PORT_EXPANDER_READ_MODE read pins for master
 */
// #define GPIO_PORT_EXPANDER_READ_MODE

/**
 * @brief GPIO_PORT_EXPANDER_READ_MODE write to pins for master
 */
#define GPIO_PORT_EXPANDER_WRITE_MODE
// ------------------------------------------------------------
// DEFAULT INSTANCES
// ------------------------------------------------------------
#define MAX_GPIOPortExpander_ATMEGA2560_Default_PINS 70
extern GPIOPortExpander GPIOPortExpander_ATMEGA2560_Default;
// ------------------------------------------------------------
// CUSTOM INSTANCES
// ------------------------------------------------------------
extern GPIOPortExpander GPIOPortExpander_ATMEGA2560_Input_0;
extern GPIOPortExpander GPIOPortExpander_ATMEGA2560_Output_0;
// ------------------------------------------------------------

// ------------------------------------------------------------
// Bus control commands.
// Command bytes 0-69 directly address a pin (see MAX_GPIO_PORTCONTROLLER_PINS).
// Control commands below are pushed well above the pin range so they can
// never collide with a pin number, even on boards with more pins than this one.
// Brace-init enforces (at compile time) that each value actually fits a uint8_t.
// ------------------------------------------------------------
// COMMAND
// ------------------------------------------------------------
constexpr uint8_t CMD_CLEAR_DATA            {100}; // 0x64 - nice round start of the control-command range
constexpr uint8_t CMD_WRITE_PIN_PWM         {110}; // 0x6E
constexpr uint8_t CMD_RESET_CURRENT_PIN     {120}; // 0x78
constexpr uint8_t CMD_GET_EXPANDER_INFO     {130}; // 0x82 - one-shot: pin_min,pin_max,max_pins,n_analog,n_digital
constexpr uint8_t CMD_GET_EXPANDER_PIN_LIST {140}; // 0x8C - highest command in use, must fit uint8_t

// ------------------------------------------------------------
// Slave-side I2C event handlers for this device (Bus 0).
// Register with Wire.onRequest()/Wire.onReceive() from the slave's setup().
// On a master build these are simply unused and get dead-stripped.
// ------------------------------------------------------------
void requestEventBus0Bin();
void receiveEventBus0Bin(int n_bytes_received);
bool readGPIOPortExapander_All(GPIOPortExpander gpio_expander);

/**
 * Reads a single pin fresh, via the direct pin-addressing commands (0-69)
 * rather than the bulk CMD_RESET_CURRENT_PIN sequential pass used by
 * readGPIOPortExapander_All(). Master-side only.
 * @param gpio_expander Specify GPIOPortExpander instance
 * @param pin Specify pin index (bounds-checked against max_pins)
 * @return false if pin is out of range or the I2C request failed
 */
bool readGPIOPortExapander_Pin(GPIOPortExpander gpio_expander, uint8_t pin);

void clearGPIOPortController(GPIOPortExpander gpio_expander);

/**
 * Set a pin's minimum accepted-read period in microseconds. Master-side only
 * - the slave never reads chan_freq_uS.
 * @param gpio_expander Specify GPIOPortExpander instance
 * @param pin Specify pin index (bounds-checked against max_pins)
 * @param freq_uS Minimum microseconds between reads of this pin
 * @return None
 */
void setGPIOPortExpanderChannelFreq(GPIOPortExpander gpio_expander, uint8_t pin, uint64_t freq_uS);
// ------------------------------------------------------------
// Master-side: query a GPIOPortExpander's configuration live over I2C
// (CMD_GET_EXPANDER_INFO + CMD_GET_EXPANDER_PIN_LIST) and point its pointer
// members at caller-supplied backing storage (MISRA Rule 21.3: no dynamic
// allocation - a buffer that must outlive a single call is a static
// instead of a heap allocation). Each *_buf must have at least
// MAX_GPIO_PORTCONTROLLER_PINS elements (modulation_buf: that many [3]
// rows). On a slave build this is simply unused and gets dead-stripped.
// ------------------------------------------------------------
bool queryGPIOPortExpander(TwoWire &wire, int address, const char *name, GPIOPortExpander *out,
                            int8_t *analog_buf, int8_t *digital_buf,
                            unsigned long (*modulation_buf)[3],
                            long *input_buf, long *output_buf,
                            int *port_map_buf, bool *switch_state_buf);

// ------------------------------------------------------------
// Master-side: named GPIOPortExpander instances for the known physical
// devices, auto-populated over I2C. Addresses are passed in (not hardcoded
// here) so this file stays identical across projects regardless of which
// project-specific I2C_ADDR_* macros exist. On a slave build these are
// simply unused and get dead-stripped.
// ------------------------------------------------------------

void initGPIOPortExpanders(
  TwoWire &wire_output_0,
  int addr_output_0
);

#endif // __GPIO_PORTCONTROLLER__

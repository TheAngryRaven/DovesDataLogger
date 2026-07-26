#pragma once

#include "Arduino.h"

///////////////////////////////////////////
// nrf_gpio shim — sim build. The only caller (System OFF wake config in
// shutdownSystemOff()) is #ifndef SIM'd out, but the include must resolve
// and the symbols are kept for completeness.
///////////////////////////////////////////

typedef enum {
  NRF_GPIO_PIN_NOPULL = 0,
  NRF_GPIO_PIN_PULLDOWN = 1,
  NRF_GPIO_PIN_PULLUP = 3,
} nrf_gpio_pin_pull_t;

typedef enum {
  NRF_GPIO_PIN_NOSENSE = 0,
  NRF_GPIO_PIN_SENSE_LOW = 3,
  NRF_GPIO_PIN_SENSE_HIGH = 2,
} nrf_gpio_pin_sense_t;

static inline void nrf_gpio_cfg_sense_input(uint32_t, nrf_gpio_pin_pull_t,
                                            nrf_gpio_pin_sense_t) {}
static inline void nrf_gpio_cfg_sense_set(uint32_t, nrf_gpio_pin_sense_t) {}
// Idle-high is the sim's stand-in for an unconnected wake pin.
static inline uint32_t nrf_gpio_pin_read(uint32_t) { return 1; }

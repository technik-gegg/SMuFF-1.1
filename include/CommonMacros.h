#pragma once

#define COUNT(a)    (sizeof(a) / sizeof(a[0]))

#if defined (__AVR__)
  typedef uint8_t WiringPinMode;
  #define INPUT_PULLDOWN  INPUT   // Pulldown not supported, default to input

  typedef uint16_t timerVal_t;

  #define noInterrupts    cli
  #define interrupts      sei

#elif defined (__STM32F1__)
  typedef uint16_t timerVal_t;

  #define I2C_SUCCESS SUCCESS

#elif defined (__ESP32__)
  typedef uint8_t WiringPinMode;

  typedef uint64_t timerVal_t;

  #define I2C_SUCCESS I2C_ERROR_OK
#endif

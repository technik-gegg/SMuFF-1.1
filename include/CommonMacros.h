#pragma once

#if defined (__AVR__)
  typedef uint8_t WiringPinMode;
  #define INPUT_PULLDOWN  INPUT   // Pulldown not supported, default to input

  #typedef uint16_t timerVal_t;

  #define noInterrupts    cli
  #define interrupts      sei

#elif defined (__STM32F1__)
  #typedef uint16_t timerVal_t;

#elif defined (__ESP32__)
  typedef uint8_t WiringPinMode;
  #define INPUT_PULLDOWN  INPUT   // Pulldown not supported, default to input

  #typedef uint64_t timerVal_t;

  #define noInterrupts    cli
  #define interrupts      sei
#endif

#pragma once

#if defined (__STM32F1__)
  #define I2C_SUCCESS SUCCESS

#elif defined (__ESP32__)
  typedef uint8_t WiringPinMode;

  #define I2C_SUCCESS I2C_ERROR_OK
#endif

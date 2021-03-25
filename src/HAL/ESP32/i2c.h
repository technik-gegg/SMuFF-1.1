#pragma once

#define I2C_SUCCESS   I2C_ERROR_OK

#if !defined(USE_SW_TWI)
  #include <Wire.h>

  extern Wire I2CBus;
#else
  #error "Soft I2C is not supported!"
#endif

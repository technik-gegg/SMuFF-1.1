#pragma once

#define I2C_SUCCESS     SUCCESS

#if !defined(USE_SW_TWI)
#include <Wire.h>
  extern TwoWire I2CBus;
  extern TwoWire I2CBus2;
#else
  #include <U8g2lib.h>
  #include <SoftWire.h>

  uint8_t u8x8_byte_smuff_sw_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);

  class SMUFF_SH1106_128X64_NONAME_F_SW_I2C : public U8G2 {
    public: SMUFF_SH1106_128X64_NONAME_F_SW_I2C(const u8g2_cb_t *rotation, uint8_t clock, uint8_t data, uint8_t reset = U8X8_PIN_NONE) : U8G2() {
      u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, rotation, u8x8_byte_smuff_sw_i2c, u8x8_gpio_and_delay_arduino);
    }
  };

  extern SoftWire I2CBus;
  extern SoftWire I2CBus2;
#endif

#include "SMuFF.h"

#if !defined(USE_SW_TWI)
  TwoWire I2CBus(1);
  TwoWire I2CBus2(2);
#else
  SoftWire I2CBus(DSP_SCL, DSP_SDA);
  SoftWire I2CBus2(PCF857x_SCL, PCF857x_SDA);

  uint8_t u8x8_byte_smuff_sw_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr) {
    switch(msg) {
      case U8X8_MSG_BYTE_SEND:
        I2CBus.write((uint8_t *)arg_ptr, (int)arg_int);
        break;

      case U8X8_MSG_BYTE_INIT:
        if (u8x8->bus_clock == 0)
	        u8x8->bus_clock = u8x8->display_info->i2c_bus_clock_100kHz * 100000UL;
        I2CBus.begin();
        break;

      case U8X8_MSG_BYTE_SET_DC:
        break;

      case U8X8_MSG_BYTE_START_TRANSFER:
        I2CBus.setClock(u8x8->bus_clock);
        I2CBus.beginTransmission(u8x8_GetI2CAddress(u8x8)>>1);
        break;

      case U8X8_MSG_BYTE_END_TRANSFER:
        I2CBus.endTransmission();
        break;

      default:
        return 0;
    }

    return 1;
  }
#endif

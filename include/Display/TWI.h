#include "U8g2lib.h"
#include "HAL/HAL.h"
#if defined(USE_SW_TWI)
typedef SMUFF_SSD1306_128X64_NONAME_F_SW_I2C    dspDriver;
#define INIT_DSP(...) dspDriver(U8G2_R0, /* clock */ DSP_SCL, /* data */ DSP_SDA, /* reset=*/U8X8_PIN_NONE)
#else
typedef U8G2_SSD1306_128X64_NONAME_F_HW_I2C     dspDriver;
#define INIT_DSP(...) dspDriver(U8G2_R0, /* reset=*/U8X8_PIN_NONE)
#endif
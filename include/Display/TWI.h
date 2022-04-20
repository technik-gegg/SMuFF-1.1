#include "HAL/HAL.h"

typedef SMUFF_SSD1306_128X64_NONAME_F_I2C    dspDriver;
#define INIT_DSP(...) dspDriver(U8G2_R0, /* reset= */U8X8_PIN_NONE, /* clock= */ DSP_SCL, /* data= */ DSP_SDA)

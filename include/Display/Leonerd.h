#include "HAL/HAL.h"

typedef SMUFF_SH1106_128X64_NONAME_F_I2C     dspDriver;
#define INIT_DSP(...) dspDriver(U8G2_R2,  /* reset=*/ U8X8_PIN_NONE, /* clock= */ DSP_SCL, /* data= */ DSP_SDA)


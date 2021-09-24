#include "U8g2lib.h"
#if defined(CREALITY_HW_SPI)
typedef U8G2_ST7920_128X64_F_HW_SPI dspDriver;
#define INIT_DSP(...)               dspDriver(U8G2_R0, /* cs=*/DSP_CS_PIN, /* reset=*/DSP_RESET_PIN);
#else
typedef U8G2_ST7920_128X64_F_SW_SPI dspDriver;
#define INIT_DSP(...)               dspDriver(U8G2_R0, /* clock=*/DSP_DC_PIN, /* data=*/DSP_DATA_PIN, /* cs=*/DSP_CS_PIN, /* reset=*/DSP_RESET_PIN);
#endif
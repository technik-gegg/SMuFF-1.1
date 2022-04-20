#include <U8g2lib.h>
typedef U8G2_ST7920_128X64_F_2ND_HW_SPI dspDriver;
#define INIT_DSP(...)               dspDriver(U8G2_R0, /* cs=*/DSP_CS_PIN, /* reset=*/DSP_RESET_PIN);

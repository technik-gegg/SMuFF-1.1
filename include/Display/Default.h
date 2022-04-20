#include <U8g2lib.h>
typedef U8G2_UC1701_MINI12864_F_4W_HW_SPI   dspDriver;
#define INIT_DSP(...)                       dspDriver(U8G2_R0, /* cs=*/DSP_CS_PIN, /* dc=*/DSP_DC_PIN, /* reset=*/DSP_RESET_PIN);

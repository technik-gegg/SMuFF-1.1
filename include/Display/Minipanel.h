#include <U8g2lib.h>
typedef U8G2_ST7567_JLX12864_F_4W_HW_SPI    dspDriver;
#define INIT_DSP(...)                       dspDriver(U8G2_R2, /* cs=*/DSP_CS_PIN, /* dc=*/DSP_DC_PIN, /* reset=*/DSP_RESET_PIN);

#define NUM_LEDS            3       // number of Neopixel LEDS
#define BRIGHTNESS          127
#define COLOR_ORDER         NEO_RGB + NEO_KHZ800

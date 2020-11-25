/**
 * SMuFF Firmware
 * Copyright (C) 2019 Technik Gegg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "SMuFF.h"

#if defined(USE_FASTLED_BACKLIGHT)
CRGB leds[NUM_LEDS];
static CRGB ColorsFastLED[8] = { CRGB::Black, CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Cyan, CRGB::Magenta, CRGB::Yellow, CRGB::White };
#endif

// simulate beeps with different LEDs on LeoNerd display
void showLed(uint8_t mode, uint8_t count) {
#if defined(USE_LEONERD_DISPLAY)
  switch(mode) {
    case 1: encoder.setLED(1, true); break;
    case 2: encoder.setLED(2, true); break;
    case 3: encoder.setLED(1, true); encoder.setLED(2, true); break;
    case 4:
      encoder.setLED(LED_GREEN, false);
      encoder.setLED(LED_RED, false);
      encoder.setLED(LED_GREEN, true);
      delay(500);
      encoder.setLED(LED_GREEN, false);
      encoder.setLED(LED_RED, true);
      delay(500);
      encoder.setLED(LED_RED, false);
      break;
    default: encoder.setLED(LED_GREEN, false); encoder.setLED(LED_RED, false); break;
  }
#endif
}

uint8_t colorMap[8] PROGMEM = { 0, 1, 2, 4, 6, 5, 3, 7 };

void setBacklightRGB(uint8_t color) {
  if(color >= 0 && color <= 7) {
    #if defined(RGB_LED_R_PIN)
    pinMode(RGB_LED_R_PIN, OUTPUT);
    digitalWrite(RGB_LED_R_PIN, colorMap[color] & 1);
    #endif
    #if defined(RGB_LED_G_PIN)
    pinMode(RGB_LED_G_PIN, OUTPUT);
    digitalWrite(RGB_LED_G_PIN, colorMap[color] & 2);
    #endif
    #if defined(RGB_LED_B_PIN)
    pinMode(RGB_LED_B_PIN, OUTPUT);
    digitalWrite(RGB_LED_B_PIN, colorMap[color] & 4);
    #endif
  }
}

void setBacklightRGB(byte R, byte G, byte B) {
  #if defined(RGB_LED_R_PIN) && defined(RGB_LED_G_PIN) && defined(RGB_LED_B_PIN)
  pinMode(RGB_LED_R_PIN, OUTPUT);
  pinMode(RGB_LED_G_PIN, OUTPUT);
  pinMode(RGB_LED_B_PIN, OUTPUT);
  digitalWrite(RGB_LED_R_PIN, (R&1));
  digitalWrite(RGB_LED_G_PIN, (G&1));
  digitalWrite(RGB_LED_B_PIN, (B&1));
  #endif
}

void setBacklightCRGB(CRGB color) {
#if defined(USE_FASTLED_BACKLIGHT)
  FastLED.showColor(color);
#endif
}

void setFastLED(uint8_t index, CRGB color) {
#if defined(USE_FASTLED_BACKLIGHT)
  leds[index] = color;
  FastLED.show();
#endif
}

void setFastLEDIndex(uint8_t index, uint8_t color) {
#if defined(USE_FASTLED_BACKLIGHT)
    leds[index] = ColorsFastLED[color];
    FastLED.show();
#endif
}

void setFastLEDIntensity(uint8_t intensity) {
#if defined(USE_FASTLED_BACKLIGHT)
    FastLED.setBrightness(intensity);
#endif
}

void setBacklightIndex(int color) {
#if defined(USE_RGB_BACKLIGHT)
  setBacklightRGB(color);
#elif defined(USE_FASTLED_BACKLIGHT)
  setBacklightCRGB(ColorsFastLED[color]);
#endif
}

void testFastLED() {
#if defined(USE_FASTLED_BACKLIGHT)
  for(uint8_t i=0; i< NUM_LEDS; i++) {
    leds[i] = ColorsFastLED[1];
    FastLED.show();
    delay(250);
    leds[i] = ColorsFastLED[0];
  }
#endif
}

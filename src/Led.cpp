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
#include "Config.h"
#include "FastLED.h"

CRGB leds[NUM_LEDS];

void showLed(int mode, int count) {
  
  CRGB color;
  switch(mode) {
    case 0: // off
      color = CRGB::Black;
      break;
    case 1: // beep
      color = CRGB::Red;
      break;
    case 2: // longBeep
      color = CRGB::Cyan;
      break;
    case 3: // userBeep
      color = CRGB::Orange;
      break;
    case 4: // initBeep
      color = CRGB::Pink;
      break;
    default: // unknown
      color = CRGB::Yellow;
      break;
  }
  for(int i=0; i< NUM_LEDS; i++)
    leds[i] = color;
  #if defined(__STM32F1__)
  FastLED.show();
  #endif
}

void setBacklightRGB(byte R, byte G, byte B) {
  setBacklightRGB((R&1) | (G&1)<<1 | (B&1)<<2);
}

void setBacklightRGB(byte color) {
#if defined(RGB_LED_R_PIN)
    pinMode(RGB_LED_R_PIN, OUTPUT);
    digitalWrite(RGB_LED_R_PIN, color & 1);
#endif
#if defined(RGB_LED_G_PIN)
    pinMode(RGB_LED_G_PIN, OUTPUT);
    digitalWrite(RGB_LED_G_PIN, color & 2);
#endif
#if defined(RGB_LED_B_PIN)
    pinMode(RGB_LED_B_PIN, OUTPUT);
    digitalWrite(RGB_LED_B_PIN, color & 4);
#endif
}

void setBacklightFastLED(CRGB color) {
    for(int i=0; i< NUM_LEDS; i++)
        leds[i] = color;
    FastLED.show();
}
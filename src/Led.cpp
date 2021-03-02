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

#define FADE_SPEED          50
#define FADE_SPEED_MARQUEE  25
#define MARQUEE_BPM         6

uint8_t lastFastLedIndex = 255, lastFastLedColor = 0;
volatile uint8_t lastFastLedStatus = 0;
volatile bool fastLedStatus = false;
uint8_t colorMap[8] PROGMEM = {0, 1, 2, 4, 6, 5, 3, 7};

#if defined(USE_FASTLED_BACKLIGHT)
CRGB leds[NUM_LEDS];
#endif
#if defined(USE_FASTLED_TOOLS)
CRGB ledsTool[MAX_TOOLS];
#endif
#if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
const CRGB ColorsFastLED[8] PROGMEM = {CRGB::Black, CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Cyan, CRGB::Magenta, CRGB::Yellow, CRGB::White};
#endif

// simulate beeps with different LEDs on LeoNerd display
void showLed(uint8_t mode, uint8_t count)
{
#if defined(USE_LEONERD_DISPLAY)
  switch (mode)
  {
  case 1:
    encoder.setLED(1, true);
    break;
  case 2:
    encoder.setLED(2, true);
    break;
  case 3:
    encoder.setLED(1, true);
    encoder.setLED(2, true);
    break;
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
  default:
    encoder.setLED(LED_GREEN, false);
    encoder.setLED(LED_RED, false);
    break;
  }
#endif
}

void setBacklightRGB(uint8_t color) {
  if (color >= 0 && color <= 7)
  {
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
  digitalWrite(RGB_LED_R_PIN, (R & 1));
  digitalWrite(RGB_LED_G_PIN, (G & 1));
  digitalWrite(RGB_LED_B_PIN, (B & 1));
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
#endif
}

void setFastLEDIndex(uint8_t index, uint8_t color) {
#if defined(USE_FASTLED_BACKLIGHT)
  leds[index] = ColorsFastLED[color];
#endif
}

void setFastLEDStatus() {
  setFastLEDStatus(lastFastLedStatus);
}

volatile uint32_t lastStatusUpdate;
volatile bool settingFastLedStatus = false;

void setFastLEDStatus(uint8_t status) {
  if(settingFastLedStatus)
    return;
  if ((millis() - lastStatusUpdate) < 20) {
    lastFastLedStatus = status;
    return;
  }
  lastStatusUpdate = millis();
  fastLedStatus = true;
  settingFastLedStatus = true;
  switch (status) {
    case FASTLED_STAT_NONE:
      fastLedStatus = false;
      setFastLEDTools();
      break;
    case FASTLED_STAT_MARQUEE:
      setFastLEDToolsMarquee();
      break;
    case FASTLED_STAT_RAINBOW:
      setFastLEDToolsRainbow();
      break;
    case FASTLED_STAT_ERROR:
      setFastLEDToolsError();
      break;
    case FASTLED_STAT_WARNING:
      setFastLEDToolsWarning();
      break;
    case FASTLED_STAT_OK:
      setFastLEDToolsOk();
      break;
  }
  lastFastLedStatus = status;
  settingFastLedStatus = false;
  refreshFastLED();
}

void setFastLEDToolsRainbow() {
#if defined(USE_FASTLED_TOOLS)
  // FastLED's built-in rainbow generator
  fill_rainbow(ledsTool, smuffConfig.toolCount, fastLedHue);
#endif
}

void setFastLEDToolsError() {
#if defined(USE_FASTLED_TOOLS)
  fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_RED, 255, fastLedBrightness));
#endif
}

void setFastLEDToolsWarning() {
#if defined(USE_FASTLED_TOOLS)
  fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_ORANGE, 255, fastLedBrightness));
#endif
}

void setFastLEDToolsOk() {
#if defined(USE_FASTLED_TOOLS)
  fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_GREEN, 255, fastLedBrightness));
#endif
}

void setFastLEDToolsMarquee()
{
#if defined(USE_FASTLED_TOOLS)
  // taken from: https://github.com/FastLED/FastLED/blob/master/examples/DemoReel100/DemoReel100.ino
  //fadeToBlackBy(ledsTool, smuffConfig.toolCount, FADE_SPEED_MARQUEE);
  FastLED.clear(false);
  int8_t pos = beatsin8(MARQUEE_BPM, 0, smuffConfig.toolCount-1);
  ledsTool[pos] += CHSV(fastLedHue, 255, 200);
#endif
}

void setFastLEDTools(){
  setFastLEDToolIndex(lastFastLedIndex, lastFastLedColor);
}

void setFastLEDToolIndex(uint8_t index, uint8_t color) {
#if defined(USE_FASTLED_TOOLS)
  if (fastLedStatus) {
    lastFastLedIndex = index;
    lastFastLedColor = color;
    return;
  }
  FastLED.clear(true);
  //fadeToBlackBy(ledsTool, smuffConfig.toolCount, FADE_SPEED);
  if (index >= 0 && index < smuffConfig.toolCount) {
    ledsTool[smuffConfig.toolCount - index - 1] = ColorsFastLED[color];
    lastFastLedIndex = index;
    lastFastLedColor = color;
  }
  refreshFastLED();   // refresh Neopixels

#endif
}

void setFastLEDIntensity(uint8_t intensity)
{
#if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
  FastLED.setBrightness(intensity);
#endif
}

void setBacklightIndex(int color)
{
#if defined(USE_RGB_BACKLIGHT)
  setBacklightRGB(color);
#elif defined(USE_FASTLED_BACKLIGHT)
  setBacklightCRGB(ColorsFastLED[color]);
#endif
}

void setToolColorIndex(int color)
{
#if defined(USE_FASTLED_TOOLS)
  smuffConfig.toolColor = color;
  setFastLEDToolIndex(lastFastLedIndex, color);
  fill_solid(ledsTool, smuffConfig.toolCount, ColorsFastLED[color]);
#endif
}

void testFastLED(bool tools)
{
#if defined(NEOPIXEL_PIN) && defined(USE_FASTLED_BACKLIGHT)
  if(!tools) {
    __debugS(PSTR("Testing backlight FastLED"));
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = ColorsFastLED[1];
      FastLED.show();
      delay(250);
      leds[i] = ColorsFastLED[0];
    }
  }
#endif
#if defined(NEOPIXEL_TOOL_PIN) && defined(USE_FASTLED_TOOLS)
  if(tools) {
    __debugS(PSTR("Testing tools FastLED"));
    // set black as active tool color
    for (uint8_t i = 0; i < smuffConfig.toolCount; i++)
    {
      ledsTool[i] = ColorsFastLED[3];
      FastLED.show();
      delay(250);
      ledsTool[i] = ColorsFastLED[0];
    }
  }
#endif
}

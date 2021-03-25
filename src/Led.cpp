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

#define FADE_SPEED          100
#define FADE_SPEED_MARQUEE  80

uint8_t lastFastLedIndex = 255, lastFastLedColor = 0;
volatile bool fastLedFadeFlag = false;
volatile uint8_t lastFastLedStatus = 0;
volatile bool fastLedStatus = false;
uint8_t colorMap[8] PROGMEM = {0, 1, 2, 4, 6, 5, 3, 7};

#if defined(USE_FASTLED_BACKLIGHT)
CLEDController* cBackLight;
CRGB leds[NUM_LEDS];
#endif
#if defined(USE_FASTLED_TOOLS)
CLEDController* cTools;
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
  fill_solid(leds, NUM_LEDS, color);
  cBackLight->showLeds();
  //__debugS(PSTR("Backlight set"));
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
}

void setFastLEDToolsRainbow() {
#if defined(USE_FASTLED_TOOLS)
  // FastLED's built-in rainbow generator
  fill_rainbow(ledsTool, smuffConfig.toolCount, fastLedHue);
#endif
}

void setFastLEDToolsError() {
#if defined(USE_FASTLED_TOOLS)
  uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
  fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_RED, 255, brightness));
#endif
}

void setFastLEDToolsWarning() {
#if defined(USE_FASTLED_TOOLS)
  uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
  fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_ORANGE, 255, brightness));
#endif
}

void setFastLEDToolsOk() {
#if defined(USE_FASTLED_TOOLS)
  uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
  fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_GREEN, 255, brightness));
#endif
}

void setFastLEDToolsMarquee()
{
#if defined(USE_FASTLED_TOOLS)
  // taken from: https://github.com/FastLED/FastLED/blob/master/examples/DemoReel100/DemoReel100.ino
  fadeToBlackBy(ledsTool, smuffConfig.toolCount, FADE_SPEED_MARQUEE);
  int8_t pos = beatsin8(smuffConfig.animationBPM, 0, smuffConfig.toolCount-1);
  uint32_t color = smuffConfig.materialColors[smuffConfig.toolCount - pos - 1];
  // if the material color value is 0, set a random color
  if(color == 0)
    ledsTool[pos] += CHSV(fastLedHue, 255, 200);
  else
    ledsTool[pos] = CRGB(color);
#endif
}

void setFastLEDTools(){
  setFastLEDToolIndex(lastFastLedIndex, lastFastLedColor, false);
}

void setFastLEDToolIndex(uint8_t index, uint8_t color, bool setFlag) {
#if defined(USE_FASTLED_TOOLS)
  if (fastLedStatus && fastLedStatus != FASTLED_STAT_MARQUEE) {
    lastFastLedIndex = index;
    lastFastLedColor = color;
    return;
  }
  uint8_t ndx = lastFastLedIndex;
  fadeToBlackBy(ledsTool, smuffConfig.toolCount, FADE_SPEED);
  if (index >= 0 && index < smuffConfig.toolCount) {
    ledsTool[smuffConfig.toolCount - index - 1] = ColorsFastLED[color];
    lastFastLedIndex = index;
    lastFastLedColor = color;
  }
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
  setFastLEDToolIndex(lastFastLedIndex, color, false);
  cTools->showLeds();
#endif
}

void testFastLED(bool tools)
{
#if defined(NEOPIXEL_PIN) && defined(USE_FASTLED_BACKLIGHT)
  if(!tools) {
    __debugS(PSTR("Testing backlight FastLED"));
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = ColorsFastLED[(uint8_t)random(1,7)];
      cBackLight->showLeds();
      delay(250);
      leds[i] = CRGB::Black;
    }
    setBacklightCRGB(ColorsFastLED[smuffConfig.backlightColor]);
  }
#endif
#if defined(NEOPIXEL_TOOL_PIN) && defined(USE_FASTLED_TOOLS)
  if(tools) {
    __debugS(PSTR("Testing tools FastLED"));
    // set black as active tool color
    for (uint8_t i = 0; i < smuffConfig.toolCount; i++)
    {
      ledsTool[i] = ColorsFastLED[(uint8_t)random(1,7)];
      cTools->showLeds();
      delay(250);
      ledsTool[i] = CRGB::Black;
    }
    cTools->showLeds();
  }
#endif
}

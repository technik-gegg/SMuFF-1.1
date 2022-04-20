/**
 * SMuFF Firmware
 * Copyright (C) 2019-2022 Technik Gegg
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

uint8_t           lastFastLedIndex = 255, lastFastLedColor = 0;
volatile uint8_t  lastFastLedStatus = 0;
volatile bool     fastLedStatus = false;
volatile uint8_t  fastLedHue = 0;

typedef enum {
  AliceBlue=0xF0F8FF,
  Amethyst=0x9966CC,
  AntiqueWhite=0xFAEBD7,
  Aqua=0x00FFFF,
  Aquamarine=0x7FFFD4,
  Azure=0xF0FFFF,
  Beige=0xF5F5DC,
  Bisque=0xFFE4C4,
  Black=0x000000,
  BlanchedAlmond=0xFFEBCD,
  Blue=0x0000FF,
  BlueViolet=0x8A2BE2,
  Brown=0xA52A2A,
  BurlyWood=0xDEB887,
  CadetBlue=0x5F9EA0,
  Chartreuse=0x7FFF00,
  Chocolate=0xD2691E,
  Coral=0xFF7F50,
  CornflowerBlue=0x6495ED,
  Cornsilk=0xFFF8DC,
  Crimson=0xDC143C,
  Cyan=0x00FFFF,
  DarkBlue=0x00008B,
  DarkCyan=0x008B8B,
  DarkGoldenrod=0xB8860B,
  DarkGray=0xA9A9A9,
  DarkGrey=0xA9A9A9,
  DarkGreen=0x006400,
  DarkKhaki=0xBDB76B,
  DarkMagenta=0x8B008B,
  DarkOliveGreen=0x556B2F,
  DarkOrange=0xFF8C00,
  DarkOrchid=0x9932CC,
  DarkRed=0x8B0000,
  DarkSalmon=0xE9967A,
  DarkSeaGreen=0x8FBC8F,
  DarkSlateBlue=0x483D8B,
  DarkSlateGray=0x2F4F4F,
  DarkSlateGrey=0x2F4F4F,
  DarkTurquoise=0x00CED1,
  DarkViolet=0x9400D3,
  DeepPink=0xFF1493,
  DeepSkyBlue=0x00BFFF,
  DimGray=0x696969,
  DimGrey=0x696969,
  DodgerBlue=0x1E90FF,
  FireBrick=0xB22222,
  FloralWhite=0xFFFAF0,
  ForestGreen=0x228B22,
  Fuchsia=0xFF00FF,
  Gainsboro=0xDCDCDC,
  GhostWhite=0xF8F8FF,
  Gold=0xFFD700,
  Goldenrod=0xDAA520,
  Gray=0x808080,
  Grey=0x808080,
  Green=0x008000,
  GreenYellow=0xADFF2F,
  Honeydew=0xF0FFF0,
  HotPink=0xFF69B4,
  IndianRed=0xCD5C5C,
  Indigo=0x4B0082,
  Ivory=0xFFFFF0,
  Khaki=0xF0E68C,
  Lavender=0xE6E6FA,
  LavenderBlush=0xFFF0F5,
  LawnGreen=0x7CFC00,
  LemonChiffon=0xFFFACD,
  LightBlue=0xADD8E6,
  LightCoral=0xF08080,
  LightCyan=0xE0FFFF,
  LightGoldenrodYellow=0xFAFAD2,
  LightGreen=0x90EE90,
  LightGrey=0xD3D3D3,
  LightPink=0xFFB6C1,
  LightSalmon=0xFFA07A,
  LightSeaGreen=0x20B2AA,
  LightSkyBlue=0x87CEFA,
  LightSlateGray=0x778899,
  LightSlateGrey=0x778899,
  LightSteelBlue=0xB0C4DE,
  LightYellow=0xFFFFE0,
  Lime=0x00FF00,
  LimeGreen=0x32CD32,
  Linen=0xFAF0E6,
  Magenta=0xFF00FF,
  Maroon=0x800000,
  MediumAquamarine=0x66CDAA,
  MediumBlue=0x0000CD,
  MediumOrchid=0xBA55D3,
  MediumPurple=0x9370DB,
  MediumSeaGreen=0x3CB371,
  MediumSlateBlue=0x7B68EE,
  MediumSpringGreen=0x00FA9A,
  MediumTurquoise=0x48D1CC,
  MediumVioletRed=0xC71585,
  MidnightBlue=0x191970,
  MintCream=0xF5FFFA,
  MistyRose=0xFFE4E1,
  Moccasin=0xFFE4B5,
  NavajoWhite=0xFFDEAD,
  Navy=0x000080,
  OldLace=0xFDF5E6,
  Olive=0x808000,
  OliveDrab=0x6B8E23,
  Orange=0xFFA500,
  OrangeRed=0xFF4500,
  Orchid=0xDA70D6,
  PaleGoldenrod=0xEEE8AA,
  PaleGreen=0x98FB98,
  PaleTurquoise=0xAFEEEE,
  PaleVioletRed=0xDB7093,
  PapayaWhip=0xFFEFD5,
  PeachPuff=0xFFDAB9,
  Peru=0xCD853F,
  Pink=0xFFC0CB,
  Plaid=0xCC5533,
  Plum=0xDDA0DD,
  PowderBlue=0xB0E0E6,
  Purple=0x800080,
  Red=0xFF0000,
  RosyBrown=0xBC8F8F,
  RoyalBlue=0x4169E1,
  SaddleBrown=0x8B4513,
  Salmon=0xFA8072,
  SandyBrown=0xF4A460,
  SeaGreen=0x2E8B57,
  Seashell=0xFFF5EE,
  Sienna=0xA0522D,
  Silver=0xC0C0C0,
  SkyBlue=0x87CEEB,
  SlateBlue=0x6A5ACD,
  SlateGray=0x708090,
  SlateGrey=0x708090,
  Snow=0xFFFAFA,
  SpringGreen=0x00FF7F,
  SteelBlue=0x4682B4,
  Tan=0xD2B48C,
  Teal=0x008080,
  Thistle=0xD8BFD8,
  Tomato=0xFF6347,
  Turquoise=0x40E0D0,
  Violet=0xEE82EE,
  Wheat=0xF5DEB3,
  White=0xFFFFFF,
  WhiteSmoke=0xF5F5F5,
  Yellow=0xFFFF00,
  YellowGreen=0x9ACD32
} HTMLColorCode;

uint8_t           colorMap[8] PROGMEM = {0, 1, 2, 4, 6, 5, 3, 7};
#if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
  const CRGB LEDColors[8] PROGMEM = { Black, Red, Green, Blue, Cyan, Magenta, Yellow, White };
#endif

#if defined(USES_ADAFRUIT_NPX)
typedef enum {
  HUE_RED = 0,
  HUE_ORANGE = 32,
  HUE_YELLOW = 64,
  HUE_GREEN = 96,
  HUE_AQUA = 128,
  HUE_BLUE = 160,
  HUE_PURPLE = 192,
  HUE_PINK = 224
} HSVHue;
#endif 

#if defined(USE_FASTLED_BACKLIGHT)
  #if defined(USES_ADAFRUIT_NPX)
    Adafruit_NeoPixel* cBackLight;
  #else
    CLEDController* cBackLight;
    CRGB leds[NUM_LEDS];
  #endif
#endif

#if defined(USE_FASTLED_TOOLS)
  #if defined(USES_ADAFRUIT_NPX)
    Adafruit_NeoPixel* cTools;
  #else
    CLEDController* cTools;
    CRGB ledsTool[MAX_TOOLS];
  #endif
#endif

#if defined(USES_ADAFRUIT_NPX)
  #if defined(USE_FASTLED_TOOLS) || defined(USE_FASTLED_BACKLIGHT)
    /* some FastLED wrapper functions for Adafruit NeoPixel library */
    uint32_t ColorRGB(uint8_t r, uint8_t g, uint8_t b) {
      return ((r << 16) | (g << 8) | b);
    }

    void ColorToRGB(uint32_t color, uint8_t* r, uint8_t* g, uint8_t* b) {
      *r = (color >> 16) & 0xff;
      *g = (color >> 8)  & 0xff;
      *b = (color >> 0 ) & 0xff;
    }

    void nscale8(Adafruit_NeoPixel* instance, uint16_t num_leds, uint8_t scale) {
      if(instance == nullptr)
        return;
      uint8_t r, g, b;
      for(uint16_t i = 0; i < num_leds; ++i) {
        ColorToRGB(instance->getPixelColor(i), &r, &g, &b);
        nscale8x3( &r, &g, &b, scale);
        instance->setPixelColor(i, ColorRGB(r, g, b));
      }
    }

    void fadeToBlackBy(Adafruit_NeoPixel* instance, uint16_t num_leds, uint8_t fadeBy) {
      if(instance == nullptr)
        return;
      nscale8(instance, num_leds, 255 - fadeBy);
    }

    void addHSV(Adafruit_NeoPixel* instance, uint16_t index, uint32_t rgb) {
      if(instance == nullptr)
        return;
      uint8_t r1, r2, g1, g2, b1, b2;
      ColorToRGB(rgb, &r1, &g1, &b1);
      ColorToRGB(instance->getPixelColor(index), &r2, &g2, &b2);
      instance->setPixelColor(index, ColorRGB(qadd8(r1, r2), qadd8(g1, g2), qadd8(b1, b2)));
    }
  #endif
#endif

void showToolLeds() {
#if defined(USE_FASTLED_TOOLS)
  if(cTools == nullptr)
    return;
  #if !defined(USES_ADAFRUIT_NPX)
    cTools->showLeds();
  #else
    cTools->show();
  #endif
#endif
}

void showBacklightLeds() {
#if defined(USE_FASTLED_BACKLIGHT)
  if(cBackLight == nullptr)
    return;
  #if !defined(USES_ADAFRUIT_NPX)
    cBackLight->showLeds();
  #else
    cBackLight->show();
  #endif
#endif
}

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
    encoder.setLED(LN_LED_GREEN, false);
    encoder.setLED(LN_LED_RED, false);
    encoder.setLED(LN_LED_GREEN, true);
    delay(500);
    encoder.setLED(LN_LED_GREEN, false);
    encoder.setLED(LN_LED_RED, true);
    delay(500);
    encoder.setLED(LN_LED_RED, false);
    break;
  default:
    encoder.setLED(LN_LED_GREEN, false);
    encoder.setLED(LN_LED_RED, false);
    break;
  }
#endif
}

void setBacklightRGB(uint8_t color) {
  if (color >= 0 && color <= 7) {
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
  #if !defined(USES_ADAFRUIT_NPX)
    fill_solid(leds, NUM_LEDS, color);
    //__debugS(I, PSTR("Backlight set"));
  #else
    if(cBackLight != nullptr)
      cBackLight->fill(color, 0, NUM_LEDS);
  #endif
  showBacklightLeds();
#endif
}

void setFastLED(uint8_t index, CRGB color) {
#if defined(USE_FASTLED_BACKLIGHT)
  #if !defined(USES_ADAFRUIT_NPX)
    leds[index] = color;
  #else
    if(cBackLight != nullptr)
      cBackLight->setPixelColor(index, color);
  #endif
#endif
}

void setFastLEDIndex(uint8_t index, uint8_t color) {
#if defined(USE_FASTLED_BACKLIGHT)
  #if !defined(USES_ADAFRUIT_NPX)
    leds[index] = LEDColors[color];
  #else
    if(cBackLight != nullptr)
      cBackLight->setPixelColor(index, LEDColors[color]);
  #endif
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
  #if !defined(USES_ADAFRUIT_NPX)
    // FastLED's built-in rainbow generator
    fill_rainbow(ledsTool, smuffConfig.toolCount, fastLedHue);
  #else
    if(cTools != nullptr)
      cTools->rainbow(fastLedHue);
  #endif
#endif
}

void setFastLEDToolsError() {
#if defined(USE_FASTLED_TOOLS)
  #if !defined(USES_ADAFRUIT_NPX)
    uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
    fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_RED, 255, brightness));
  #else
    uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
    if(cTools != nullptr)
      cTools->fill(cTools->gamma32(cTools->ColorHSV(HUE_RED, 255, brightness)));
  #endif
#endif
}

void setFastLEDToolsWarning() {
#if defined(USE_FASTLED_TOOLS)
  #if !defined(USES_ADAFRUIT_NPX)
    uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
    fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_ORANGE, 255, brightness));
  #else
    uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
    if(cTools != nullptr)
      cTools->fill(cTools->gamma32(cTools->ColorHSV(HUE_ORANGE, 255, brightness)));
  #endif
#endif
}

void setFastLEDToolsOk() {
#if defined(USE_FASTLED_TOOLS)
  #if !defined(USES_ADAFRUIT_NPX)
    uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
    fill_solid(ledsTool, smuffConfig.toolCount, CHSV(HUE_GREEN, 255, brightness));
  #else
    uint8_t brightness = beatsin8(smuffConfig.statusBPM, 0, 255);
    if(cTools != nullptr)
      cTools->fill(cTools->gamma32(cTools->ColorHSV(HUE_GREEN, 255, brightness)));
  #endif
#endif
}

void setFastLEDToolsMarquee() {
#if defined(USE_FASTLED_TOOLS)
  int8_t pos = beatsin8(smuffConfig.animationBPM, 0, smuffConfig.toolCount-1);
  uint32_t color = smuffConfig.materialColors[smuffConfig.toolCount - pos - 1];
  #if !defined(USES_ADAFRUIT_NPX)
    /*
     * taken from: https://github.com/FastLED/FastLED/blob/master/examples/DemoReel100/DemoReel100.ino
     */
    fadeToBlackBy(ledsTool, smuffConfig.toolCount, FADE_SPEED_MARQUEE);
    // if the material color value is 0, set a random color
    if(color == 0)
      ledsTool[pos] += CHSV(fastLedHue, 255, 200);
    else
      ledsTool[pos] = CRGB(color);
  #else
    fadeToBlackBy(cTools, smuffConfig.toolCount, FADE_SPEED_MARQUEE);
    if(cTools != nullptr) {
      if(color == 0)
        addHSV(cTools, pos, cTools->gamma32(cTools->ColorHSV((uint16_t)fastLedHue<<8, 255, 200)));
      else
        cTools->setPixelColor(pos, color);  
    }
  #endif
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
  #if !defined(USES_ADAFRUIT_NPX)
    fadeToBlackBy(ledsTool, smuffConfig.toolCount, FADE_SPEED);
    if (index >= 0 && index < smuffConfig.toolCount) {
      ledsTool[smuffConfig.toolCount - ndx - 1] = LEDColors[color];
    }
  #else
    fadeToBlackBy(cTools, smuffConfig.toolCount, FADE_SPEED);
    if (index >= 0 && index < smuffConfig.toolCount) {
      if(cTools != nullptr)
        cTools->setPixelColor(smuffConfig.toolCount - ndx - 1, LEDColors[color]);
    }
  #endif
  lastFastLedIndex = index;
  lastFastLedColor = color;
#endif
}

void setFastLEDIntensity(uint8_t intensity) {
#if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
  #if !defined(USES_ADAFRUIT_NPX)
    FastLED.setBrightness(intensity);
  #else
    #if defined(USE_FASTLED_BACKLIGHT)
    if(cBackLight != nullptr)
      cBackLight->setBrightness(intensity);
    #endif
    #if defined(USE_FASTLED_TOOLS)
    if(cTools != nullptr)
      cTools->setBrightness(intensity);
    #endif
  #endif
#endif
}

void setBacklightIndex(int color) {
#if defined(USE_RGB_BACKLIGHT)
  setBacklightRGB(color);
#elif defined(USE_FASTLED_BACKLIGHT)
  setBacklightCRGB(LEDColors[color]);
#endif
}

void setContrast(int contrast) {
  display.setContrast((uint8_t)contrast);
}

void setToolColorIndex(int color) {
#if defined(USE_FASTLED_TOOLS)
  setFastLEDToolIndex(lastFastLedIndex, color, false);
  showToolLeds();
#endif
}

void testFastLED(bool tools) {
#if defined(NEOPIXEL_PIN) && defined(USE_FASTLED_BACKLIGHT)
  if(!tools) {
    __debugS(D, PSTR("[\ttesting Backlight LEDs ]"));
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
      #if !defined(USES_ADAFRUIT_NPX)
        leds[i] = LEDColors[(uint8_t)random(1,7)];
        if(cBackLight != nullptr)
          cBackLight->showLeds();
        delay(250);
        leds[i] = CRGB::Black;
      #else
        if(cBackLight != nullptr) {
          cBackLight->setPixelColor(i, LEDColors[(uint8_t)random(1,7)]);
          cBackLight->show();
          delay(250);
          cBackLight->setPixelColor(i, LEDColors[0]);
        }
      #endif
    }
    setBacklightCRGB(LEDColors[smuffConfig.backlightColor]);
  }
#endif
#if defined(NEOPIXEL_TOOL_PIN) && defined(USE_FASTLED_TOOLS)
  if(tools) {
    __debugS(D, PSTR("[\ttesting Tools LEDs ]"));
    for (uint8_t i = 0; i < smuffConfig.toolCount; i++)
    {
      #if !defined(USES_ADAFRUIT_NPX)
        ledsTool[i] = LEDColors[(uint8_t)random(1,7)];
        if(cTools != nullptr)
          cTools->showLeds();
        delay(250);
        ledsTool[i] = CRGB::Black;
      #else
        if(cTools != nullptr) {
          cTools->setPixelColor(i, LEDColors[(uint8_t)random(1,7)]);
          cTools->show();
          delay(250);
          cTools->setPixelColor(i, LEDColors[0]);
        }
      #endif
    }
    showToolLeds();
  }
#endif
}

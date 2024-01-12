/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
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

uint16_t        sequence[MAX_SEQUENCE][3];    // store for tune sequence for background playing

char PROGMEM    tuneStartup[MAX_TUNE1] = {"F1760D90.F1975D90.F2093D90.F1975D90.F1760D200P50."}; // the "traditional" tune
char PROGMEM    tuneUser[MAX_TUNE2] = {"F1760D90P90.F440D90P90.F440D90P90."};
char PROGMEM    tuneBeep[MAX_TUNE3] = {"F1760D90P200."};
char PROGMEM    tuneLongBeep[MAX_TUNE3] = {"F1760D450P500."};
#if defined(USE_LEONERD_DISPLAY)
char PROGMEM    tuneEncoder[MAX_TUNE3] = {"F330D10P10."};
#else
char PROGMEM    tuneEncoder[MAX_TUNE3] = {"F1440D3."};
#endif
uint8_t         sequenceSerial;
const char*     tuneNames[] = { "", "Beep", "LongBeep", "Startup", "User" };

#define _F_ 0     // Frequency index
#define _D_ 1     // Duration index
#define _P_ 2     // Pause index

#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
/*
  Simple wrapper for tone()
*/
void playTone(pin_t pin, int16_t freq, int16_t duration) {
#if defined(USE_LEONERD_DISPLAY)
  UNUSED(pin);
  encoder.playFrequency(freq, duration);
#else
  if (pin != 0)
    tone(pin, freq, duration);
#endif
}

void muteTone(pin_t pin) {
#if defined(USE_LEONERD_DISPLAY)
  UNUSED(pin);
  encoder.muteTone();
#else
  if (pin != 0)
    pinMode(pin, INPUT);
#endif
}
#endif

void beep(uint8_t count) {
  prepareSequence(tuneBeep, false);
  for (uint8_t i = 0; i < count; i++)
    playSequence();
}

void longBeep(uint8_t count) {
#if defined(USE_LEONERD_DISPLAY)
  encoder.setLED(LN_LED_RED, true);
#endif
  prepareSequence(tuneLongBeep, false);
  for (uint8_t i = 0; i < count; i++)
    playSequence();
}

void userBeep() {
#if defined(USE_LEONERD_DISPLAY)
  encoder.setLED(LN_LED_RED, true);
#endif
  prepareSequence(tuneUser, false);
  playSequence();
}

void encoderBeep(uint8_t count) {
  prepareSequence(tuneEncoder, false);
  playSequence();
}

void startupBeep() {
#if defined(USE_LEONERD_DISPLAY)
  showLed(4, 1);
#endif
  prepareSequence(tuneStartup, true);
}

/*
  Prepares a tune sequence from string to an array of notes to be played.
  The format is: F{frequency} D{duration} [P{pause}].F{frequency}D{duration}[P{pause}]. ...
  Example: "F440D120P80." plays an A (440Hz) with a duration of 120mS and pauses 80mS
           after the tone has played.
           The '.' at the end of a tone is mandatory and must not be omitted.
*/
void prepareSequence(const char *seq, bool autoPlay /* =true */) {
#if defined(USE_SERIAL_DISPLAY)
  sequenceSerial = 0;
  if(seq == tuneBeep)           sequenceSerial = 1;
  else if(seq == tuneLongBeep)  sequenceSerial = 2;
  else if(seq == tuneStartup)   sequenceSerial = 3; 
  else if(seq == tuneUser)      sequenceSerial = 4;
  if (autoPlay)
    playSequence();
  return;
#endif
#if !defined(USE_LEONERD_DISPLAY) 
  if (BEEPER_PIN <= 0)
    return;
#endif
  if (seq == nullptr || *seq == 0)
    return;

  int f = 0, d = 0, p = 0;
  uint8_t n = 0;
  while (*seq) {
    if (*seq == '"' || *seq == ' ' || *seq=='\r' || *seq=='\n' || *seq=='\t') // skip quotes, spaces and newlines
      seq++;
    switch(toupper(*seq)) {
      case 'F': f = atoi(++seq); break;
      case 'D': d = atoi(++seq); break;
      case 'P': p = atoi(++seq); break;
    }
    if (*seq == '.') {
      if (f && d) {
        sequence[n][_F_] = (uint16_t)f;
        sequence[n][_D_] = (uint16_t)d;
      }
      sequence[n][_P_] = (uint16_t)p;
      f = d = p = 0;
      if (n < MAX_SEQUENCE - 1)
        n++;
      else
        break;
    }
    seq++;
  }
  // mark end-of-sequence
  memset(&sequence[n], 0, 3*sizeof(uint16_t));
  if (autoPlay)
    playSequence();
}

void playSequence() {
  #if defined(USE_SERIAL_DISPLAY)
    char msg[80];
    if(sequenceSerial > 0) {
      snprintf(msg, ArraySize(msg)-1, "{\"PlaySequence\": \"%s\"}\n", tuneNames[sequenceSerial]);
      printResponse(msg, smuffConfig.displaySerial);
    }
  #else
    if(BEEPER_PIN <= 0)
      return;
    for (uint8_t i = 0; i < MAX_SEQUENCE; i++) {
      if (sequence[i][_F_] == 0)
        return;
      _tone(sequence[i][_F_], sequence[i][_D_]);
      delay(sequence[i][_D_]);
      if (sequence[i][_P_] > 0)
        delay(sequence[i][_P_]);
    }
  #endif
}


// ----------------------------------------------------------------------------
// Rotary Encoder Driver with Acceleration
// Supports Click, DoubleClick, Long Click
//
// (c) 2010 karl@pitrich.com
// (c) 2014 karl@pitrich.com
//
// Timer-based rotary encoder logic by Peter Dannegger
// http://www.mikrocontroller.net/articles/Drehgeber
// Modified by Technik Gegg - added resetButton() method
// ----------------------------------------------------------------------------

#ifndef __have__ClickEncoder_h__
#define __have__ClickEncoder_h__

// ----------------------------------------------------------------------------

#include <CommonMacros.h>
#include <stdint.h>
#ifdef __AVR__
  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include <avr/pgmspace.h>
#endif
#include "Arduino.h"

// ----------------------------------------------------------------------------

#define ENC_NORMAL        (1 << 1)   // use Peter Danneger's decoder
#define ENC_FLAKY         (1 << 2)   // use Table-based decoder

// ----------------------------------------------------------------------------

#ifndef ENC_DECODER
  #define ENC_DECODER   ENC_NORMAL
#endif

#if ENC_DECODER == ENC_FLAKY
  #ifndef ENC_HALFSTEP
    #define ENC_HALFSTEP  1   // use table for half step per default
  #endif
#endif

// ----------------------------------------------------------------------------

class ClickEncoder {
  public:
    typedef enum Button_e {
      Open = 0,
      Closed,

      Pressed,
      Held,
      Released,

      Clicked,
      DoubleClicked

    } Button;

  private:
    uint8_t pinA;
    uint8_t pinB;
    uint8_t pinBTN;
    bool pinsActive;
    volatile int16_t delta = 0;
    volatile int16_t last = 0;
    volatile uint16_t acceleration = 0;
    bool accelerationEnabled;
    uint8_t steps = 0;
    bool enableSound;

    #ifndef WITHOUT_BUTTON
      volatile Button button;
      unsigned long lastButtonCheck = 0;
      uint8_t doubleClickTicks = 0;
      bool doubleClickEnabled;
      uint16_t keyDownTicks = 0;
    #endif

    #if ENC_DECODER != ENC_NORMAL
      static const int8_t table[16];
    #endif

  public:
    ClickEncoder(uint8_t A, uint8_t B, uint8_t BTN, uint8_t stepsPerNotch, bool active = false);

    void service(void);
    int16_t getValue(void);

    #ifndef WITHOUT_BUTTON
      Button getButton(void);
      void resetButton(void) { button = Open; lastButtonCheck = millis(); doubleClickTicks = 0; }
    #endif

    #ifndef WITHOUT_BUTTON
      void setDoubleClickEnabled(const bool &d) {
        doubleClickEnabled = d;
      }

      const bool getDoubleClickEnabled() {
        return doubleClickEnabled;
      }
    #endif

    void setEnableSound(const bool &d) {
      enableSound = d;
    }

    const bool getEnableSound() {
      return enableSound;
    }

    void setAccelerationEnabled(const bool &a) {
      accelerationEnabled = a;
      if (!accelerationEnabled) acceleration = 0;
    }

    const bool getAccelerationEnabled() {
      return accelerationEnabled;
    }
};

// ----------------------------------------------------------------------------

#endif // __have__ClickEncoder_h__

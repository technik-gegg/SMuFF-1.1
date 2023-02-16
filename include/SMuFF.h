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
#pragma once

#include <Arduino.h>
#include "avr/dtostrf.h"
#include "Config.h"
#include "Strings.h"
#include "GCodes.h"

#define SM_SERIAL_PORT_NULL -1
#ifdef USE_SERIAL_DISPLAY
  #define SM_SHOULD_SHOW_MESSAGE(__serial__) \
    __serial__ != SM_SERIAL_PORT_NULL \
    && smuffConfig.displaySerial != SM_SERIAL_PORT_NULL \
    && __serial__ == smuffConfig.displaySerial
#else
  #define SM_SHOULD_SHOW_MESSAGE(__serial__) false
#endif

#ifndef USE_SERIAL_DISPLAY
#include <U8g2lib.h>
#include "Menus.h"
#include "InputDialogs.h"
#define DISPLAY_SERIAL_PORT SM_SERIAL_PORT_NULL
#endif

#if defined(USE_LEONERD_DISPLAY)
#include "LeoNerdEncoder.h"
#else
#include "ClickEncoder.h"
#endif

#include <SPI.h>

#if defined(USE_SDFAT)
#include <SdFat.h>
#define _File SdFile
extern SdFat SD;
#else
#include "STM32SD.h"
#define _File File
#endif

#include "DataStore.h"


#if defined(USE_FASTLED_BACKLIGHT) || defined(USE_FASTLED_TOOLS)
  #if defined(USES_ADAFRUIT_NPX)
    #include <Adafruit_NeoPixel.h>
    #include "lib8tion/lib8tion.h"
  #else
    #define FASTLED_ALLOW_INTERRUPTS      0
    #define FASTLED_INTERRUPT_RETRY_COUNT 1
    #define FASTLED_USE_PROGMEM           0
    //#define FASTLED_FORCE_SOFTWARE_PINS   1
    #include <FastLED.h>
  #endif
#endif
#include "HAL/HAL.h"
#include "ZStepperLib.h"

#if defined(USE_ZSERVO)
#include "ZServo.h"
#else
#undef TIMER_SERVO
#define TIMER_SERVO   TIM7
#include "Servo.h"
#endif

#include "ZFan.h"
#include "ZEStopMux.h"
#include "DuetLaserSensor.h"

#if defined(HAS_TMC_SUPPORT)
#include <TMCStepper.h>
#endif

#undef TIMER_SERIAL
#if defined(__STM32F1XX) || defined(_STM32F4XX)
  #define TIMER_SERIAL TIM5
#elif defined(__STM32G0XX)
  #define TIMER_SERIAL TIM17
#endif
#include <SoftwareSerial.h>

#if defined(USE_MULTISERVO)
#include <Adafruit_PWMServoDriver.h>
#endif

#include "Debug.h"

#if !defined(USE_FASTLED_BACKLIGHT) && !defined(USE_FASTLED_TOOLS)
  #define CRGB uint32_t
#else
  #if defined(USES_ADAFRUIT_NPX)
    #define CRGB uint32_t
  #endif
#endif

#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
  #include <wiring.h>
  #include "pinconfig.h"
  
  #ifndef _BV
    #define _BV(bit) (1 << (bit))
  #endif

  #if defined(__LIBMAPLE__)
    #undef sprintf_P
    #define sprintf_P(s, f, ...) sprintf(s, f, ##__VA_ARGS__)
    #define vsnprintf_P vsnprintf
  #endif
#endif

#if defined(USE_TWI_DISPLAY) || defined(USE_LEONERD_DISPLAY) || defined(USE_SW_TWI)
#define USE_I2C
#endif

#define FEEDER_SIGNAL           1
#define SELECTOR_SIGNAL         2
#define REVOLVER_SIGNAL         3
#define LED_SIGNAL              4

#define NOT_LOADED              0x00
#define SPL_LOADED_TO_SPLITTER  0x01
#define SPL_LOADED_TO_NOZZLE    0x02
#define LOADED_TO_SELECTOR      0x10
#define LOADED_TO_NOZZLE        0x20
#define LOADED_TO_DDE           0x40

#define INTERNAL                1
#define EXTERNAL                0

#define MAX_SEQUENCE            512
#define MAX_TUNE1               512
#define MAX_TUNE2               128
#define MAX_TUNE3               64

#define SERVO_OPEN              0
#define SERVO_CLOSED            1


#if defined(__HW_DEBUG__) && defined(DEBUG_PIN)
// used for internal hardware debugging only - will produce by default a 500Hz signal on the output pin
  #if defined(DEBUG_PIN_NAME)
    #define FLIPDBG        \
      if (DEBUG_PIN > 0) { \
        digitalToggleFast(DEBUG_PIN_NAME); \
      }
  #else
    #define FLIPDBG        \
      if (DEBUG_PIN > 0) { \
        digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN)); \
      }
  #endif
#endif

#define ArraySize(arr) (sizeof(arr) / sizeof(arr[0]))

#if defined(USE_SDFAT)
#include <SdFat.h>
#define _File SdFile
#define __fopen(file_handle, file_name, file_mode) file_handle.open(file_name, file_mode)
extern SdFat SD;
#else
#include "STM32SD.h"
#define _File File
#define __fopen(file_handle, file_name, file_mode) (file_handle = SD.open(file_name, file_mode))
#endif

typedef enum {
  ABSOLUTE,
  RELATIVE
} PositionMode;

typedef struct {
  uint16_t  period;
  uint16_t  val;
  void      (*func)();
} IntervalHandler;

// SMuFF runtime configuration storage
typedef struct {
  unsigned long serialBaudrates[4]                    = { 57600, 57600, 57600, 57600 };
  uint8_t       toolCount                             = 5;
  double        bowdenLength                          = 400.0f;
  double        selectorDistance                      = 23.0f;
  double        selectorUnloadDist                    = 23.0f;
  uint8_t       i2cAddress                            = 0x58;
  uint8_t       lcdContrast                           = DSP_CONTRAST;
  uint8_t       menuAutoClose                         = 20;
  uint8_t       fanSpeed                              = 0;
  uint16_t      powerSaveTimeout                      = 300;
  bool          sendActionCmds                        = false;
  bool          prusaMMU2                             = true;
  char          unloadCommand[MAX_UNLOAD_COMMAND]     = { 0 };
  uint16_t      servoMinPwm                           = 800;
  uint16_t      servoMaxPwm                           = 2400;
  char          wipeSequence[MAX_WIPE_SEQUENCE]       = { 0 };
  uint8_t       backlightColor                        = 0x4;      // Cyan by default
  uint8_t       hasPanelDue                           = 0;        // Serial Port for PanelDue (0=None)
  uint8_t       duet3Dport                            = 0;        // Serial Port for Duet3D (0=none)
  bool          encoderTickSound                      = false;
  bool          sendPeriodicalStats                   = true;
  char          lButtonDown[MAX_BUTTON_LEN]           = { 0 };
  char          lButtonHold[MAX_BUTTON_LEN]           = { 0 };
  char          rButtonDown[MAX_BUTTON_LEN]           = { 0 };
  char          rButtonHold[MAX_BUTTON_LEN]           = { 0 };
  bool          speedsInMMS                           = true;
  bool          runoutDetection                       = false;
  bool          useCutter                             = false;
  bool          usePurge                              = false;
  uint16_t      cutterOpen                            = 90;
  uint16_t      cutterClose                           = 50;
  double        cutterLength                          = 0;
  bool          cutterOnTop                           = false;

  // ALL STEPPERS
  uint16_t      stepsPerMM[NUM_STEPPERS+1]            = { 80, 0, 410 };
  long          maxSteps[NUM_STEPPERS+1]              = { 68000, 9600, 0 };
  uint16_t      maxSpeed[NUM_STEPPERS+1]              = { 10, 10, 10 };
  uint16_t      accelSpeed[NUM_STEPPERS+1]            = { 10, 10, 10 };
  bool          invertDir[NUM_STEPPERS+1]             = { false, false, false };
  uint8_t       endstopTrg[NUM_STEPPERS + 1]          = { HIGH, HIGH, HIGH, LOW };
  uint8_t       stepDelay[NUM_STEPPERS+1]             = { 3, 3, 3 };
  uint8_t       accelDist[NUM_STEPPERS+1]             = { 21, 5, 5 };
  int8_t        ms3config[NUM_STEPPERS+1]             = { -1, -1, -1 };
  double        speedAdjust[NUM_STEPPERS+1]           = { 1, 1, 1 };
  // TMC drivers via UART or SPI
  uint16_t      stepperPower[NUM_STEPPERS + 1]        = { 700, 700, 700, 700 };
  uint8_t       stepperMode[NUM_STEPPERS + 1]         = { 0, 0, 0, 0 };                   // 0 = NONE, 1 = UART, 2 = SPI
  bool          stepperStealth[NUM_STEPPERS + 1]      = { false, false, false, false };   // true = StealthChop, false = SpreadCycle mode (for Stall Detection)
  double        stepperRSense[NUM_STEPPERS + 1]       = { 0.11, 0.11, 0.11, 0.11 };
  uint16_t      stepperMicrosteps[NUM_STEPPERS + 1]   = { 16, 16, 16, 16 };
  int8_t        stepperStall[NUM_STEPPERS + 1]        = { 0, 0, 0, 0 };
  int8_t        stepperCSmin[NUM_STEPPERS + 1]        = { 0, 0, 0, 0 };
  int8_t        stepperCSmax[NUM_STEPPERS + 1]        = { 0, 0, 0, 0 };
  int8_t        stepperCSdown[NUM_STEPPERS + 1]       = { 0, 0, 0, 0 };
  int8_t        stepperAddr[NUM_STEPPERS + 1]         = { 0, 0, 0, 0 };
  int8_t        stepperToff[NUM_STEPPERS + 1]         = { -1, -1, -1, -1 };
  bool          stepperStopOnStall[NUM_STEPPERS + 1]  = { false, false, false, false};
  int8_t        stepperMaxStallCnt[NUM_STEPPERS + 1]  = { 5, 5, 5, 5 };
  // REVOLVER specific settings
  uint16_t      firstRevolverOffset                   = FIRST_REVOLVER_OFFSET;
  uint16_t      revolverSpacing                       = REVOLVER_SPACING;
  long          stepsPerRevolution                    = 9600;
  bool          resetBeforeFeed                       = true;
  bool          homeAfterFeed                         = true;
  bool          wiggleRevolver                        = false;
  bool          revolverIsServo                       = false;
  uint8_t       revolverOffPos                        = 0;
  uint8_t       revolverOnPos                         = 90;
  uint8_t       servoCycles1                          = 0;
  uint8_t       servoCycles2                          = 0;
  // SELECTOR specific settings
  double        firstToolOffset                       = FIRST_TOOL_OFFSET;
  double        toolSpacing                           = TOOL_SPACING;
  // FEEDER specific settings
  bool          extControlFeeder                      = false;
  uint16_t      insertSpeed                           = 1000;
  uint8_t       feedChunks                            = 20;
  bool          enableChunks                          = false;
  double        insertLength                          = 5.0;
  double        unloadRetract                         = -20.0f;
  double        unloadPushback                        = 5.0f;
  double        pushbackDelay                         = 1.5f;
  double        reinforceLength                       = 3.0f;
  bool          isSharedStepper                       = false;
  bool          externalStepper                       = false;
  bool          useDuet                               = false;
  uint16_t      purgeSpeed                            = 5000;
  double        purgeLength                           = 0;
  bool          useEndstop2                           = false;

  char          materials[MAX_TOOLS][MAX_MATERIAL_LEN+1];
  char          materialNames[MAX_TOOLS][MAX_MATERIAL_NAME_LEN+1];
  uint32_t      materialColors[MAX_TOOLS];
  uint16_t      purges[MAX_TOOLS];
  bool          wipeBeforeUnload                      = false;
  uint8_t       toolColor                             = 5;
  bool          useIdleAnimation                      = false;
  uint8_t       animationBPM                          = 6;
  uint8_t       statusBPM                             = 20;
  bool          invertRelay                           = false;
  bool          menuOnTerminal                        = false;
  bool          webInterface                          = false;
  double        revolverClose                         = 0;
  bool          useSplitter                           = false;
  double        splitterDist                          = 0;
  uint8_t       feedLoadState[MAX_TOOLS];
  bool          useDDE                                = false;
  double        ddeDist                               = 0;
  bool          purgeDDE                              = false;
  bool          traceUSBTraffic                       = false;
  char          deviceName[MAX_BUTTON_LEN]            = {0};
  uint16_t      dbgFreq                               = 500;
  bool          invertDuet                            = false;
  bool          allowSyncSteppers                     = true;
  uint8_t       dbgLevel                              = W|I|SP;
  bool          useDebugColoring                      = true;
  int8_t        displaySerial                         = DISPLAY_SERIAL_PORT;         // Serial Port for USE_SERIAL_DISPLAY

} SMuFFConfig;

extern SMuFFConfig              smuffConfig;
extern dspDriver                display;
extern GCodeFunctions           gCodeFuncsM[];
extern GCodeFunctions           gCodeFuncsG[];

extern ZStepper                 steppers[];
extern ZTimer                   stepperTimer;
extern ZTimer                   gpTimer;
#if defined(USE_FASTLED_TOOLS)
extern ZTimer                   ledTimer;
#endif
#if defined(USE_ZSERVO)
extern ZTimer                   servoTimer;
extern ZServo                   servoWiper;
extern ZServo                   servoLid;
extern ZServo                   servoCutter;
#else
extern Servo                    servoWiper;
extern Servo                    servoLid;
extern Servo                    servoCutter;
#endif
extern ZFan                     fan;
extern ZEStopMux                splitterMux;

#if defined(USE_LEONERD_DISPLAY)
extern LeoNerdEncoder           encoder;
#else
extern ClickEncoder             encoder;
#endif

#if defined(USE_FASTLED_BACKLIGHT)
  #if !defined(USES_ADAFRUIT_NPX)
    extern CLEDController*       cBackLight;
    extern CRGB                  leds[];
  #else
    extern Adafruit_NeoPixel*    cBackLight;
  #endif
#endif

#if defined(USE_FASTLED_TOOLS)
  #if !defined(USES_ADAFRUIT_NPX)
    extern CLEDController*       cTools;
    extern CRGB                  ledsTool[];
  #else
    extern Adafruit_NeoPixel*   cTools;
  #endif
#endif

#if defined(USE_MULTISERVO)
extern Adafruit_PWMServoDriver  servoPwm;
extern int8_t                   servoMapping[];
extern int8_t                   outputMode[];
#endif
extern uint8_t                  servoPosClosed[];
extern double                    stepperPosClosed[];


extern const char               brand[];
extern uint8_t                  swapTools[];
extern volatile byte            nextStepperFlag;
extern volatile byte            remainingSteppersFlag;
extern volatile unsigned long   lastEncoderButtonTime;
extern int8_t                   toolSelected;
extern int8_t                   currentSerial;
extern PositionMode             positionMode;
extern String                   serialBuffer0, serialBuffer2, serialBuffer9, traceSerial2;
extern char                     tuneStartup[];
extern char                     tuneUser[];
extern char                     tuneBeep[];
extern char                     tuneLongBeep[];
extern char                     tuneEncoder[];
extern bool                     displayingUserMessage;
extern uint16_t                 userMessageTime;
extern bool                     feederJammed;
extern volatile bool            parserBusy;
extern volatile bool            isPwrSave;
extern volatile bool            actionOk;
extern volatile bool            sendingResponse;
extern volatile bool            showMenu;
extern bool                     maintainingMode;
extern volatile double          lastDuetPos;
extern String                   wirelessHostname;
extern volatile bool            initDone;
extern volatile bool            leoNerdBlinkGreen;
extern volatile bool            leoNerdBlinkRed;
extern bool                     forceStopMenu;
extern uint16_t                 sequence[][3];
extern uint8_t                  remoteKey;
extern bool                     settingsChanged;
extern Stream                   *logSerial;
extern Stream                   *debugSerial;
extern Stream                   *terminalSerial;
extern bool                     isWarning;
extern bool                     lidOpen;
extern uint16_t                 mmsMin, mmsMax;
extern uint16_t                 speedIncrement;
extern volatile uint8_t         fastLedHue;
extern volatile bool            fastLedStatus;
extern volatile bool            fastLedRefresh;
extern volatile uint8_t         lastFastLedStatus;
extern volatile bool            isIdle;
extern bool                     tmcWarning;
extern bool                     isTestrun;
extern bool                     isTestPending;
extern char                     testToRun[];
extern bool                     isUsingTmc;
extern volatile bool            sdRemoved;
extern char                     firmware[];
extern bool                     gotFirmware;
extern int32_t                  uploadLen;
extern _File                    upload;
extern volatile bool            isUpload;
extern bool                     splitterEndstopChanged;
extern bool                     asyncDDE;
extern volatile bool            refreshingDisplay;
extern volatile uint16_t        flipDbgCnt;
extern volatile bool            sendingStatesToggle;
extern uint32_t                 waitForDlgId;
extern bool                     gotDlgId;
extern uint8_t                  dlgButton;

#ifdef HAS_TMC_SUPPORT
extern TMC2209Stepper           *drivers[];
#endif


#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
extern void                     playTone(pin_t pin, int16_t frequency, int16_t duration);
extern void                     muteTone(pin_t pin);
#define _tone(freq, duration)   playTone(BEEPER_PIN, freq, duration)
#define _noTone()               muteTone(BEEPER_PIN)
#else
#define _tone(freq, duration)   tone(BEEPER_PIN, freq, duration)
#define _noTone()               noTone(BEEPER_PIN)
#endif

extern void setupSerial();
extern void setupSwSerial0();
extern void setupDisplay();
extern void setupTimers();
extern void runTimers();
extern void setupSteppers();
extern void setupTMCDrivers();
extern void setupServos();
extern void setupFan();
extern void setupEStopMux();
extern void setupRelay();
extern void setupI2C();
extern void setupSerialBT();
extern void setupBuzzer();
extern void setupEncoder();
extern void setupBacklight();
extern void setupDuetSignals();
extern void setupDuetLaserSensor();
extern void setupHBridge();
extern void initHwDebug();
extern void initFastLED();
extern void initUSB();
extern void drawLogo();
extern void drawStatus();
extern void drawSelectingMessage(uint8_t tool);
extern void drawPurgingMessage(uint16_t len, uint8_t tool);
extern void drawUpload(uint32_t remain);
extern void drawUserMessage(String message, bool smallFont = false, bool center = true, void (*drawCallbackFunc)() = nullptr);
extern void drawSDStatus(int8_t stat);
extern void drawFeed(bool updateBuffer = true);
extern void drawSDRemoved(bool removed);
extern void resetDisplay();
extern void setDisplayPowerSave(bool state);
extern bool selectorEndstop();
extern bool revolverEndstop();
extern bool feederEndstop(int8_t index = 1);
extern bool ddeEndstop();
extern bool showFeederLoadedMessage();
extern bool showFeederLoadMessage();
extern bool showFeederFailedMessage(int8_t opt);
extern bool showFeederBlockedMessage();
extern uint8_t showDialog(PGM_P title, PGM_P message, PGM_P addMessage, PGM_P buttons, int32_t timeout = -1);
extern bool moveHome(int8_t index, bool showMessage = true, bool checkFeeder = true);
extern bool loadFilament(char* errmsg, bool showMessage = true);
extern bool loadFilamentPMMU2(char* errmsg, bool showMessage = true);
extern bool unloadFilament(char* errmsg);
extern bool unloadFromSelector(char* errmsg);
extern void wipeNozzle();
extern void cutFilament(bool keepClosed = true);
extern bool handleFeederStall(uint16_t *speed, int8_t *retries);
extern bool nudgeBackFilament();
extern void handleStall(int8_t axis);
extern void runAndWait(int8_t index);
extern void runNoWait(int8_t index);
extern bool selectTool(int8_t ndx, char* errmsg, bool showMessage = true);
extern void setStepperSteps(int8_t index, long steps, bool ignoreEndstop);
extern void prepSteppingAbs(int8_t index, long steps, bool ignoreEndstop = false);
extern void prepSteppingAbsMillimeter(int8_t index, double millimeter, bool ignoreEndstop = false);
extern void prepSteppingRel(int8_t index, long steps, bool ignoreEndstop = false);
extern void prepSteppingRelMillimeter(int8_t index, double millimeter, bool ignoreEndstop = false);
extern void resetRevolver();
extern void serialEvent();
extern void serialEvent1();
extern void serialEvent2();
extern void serialEvent3();
extern void beep(uint8_t count);
extern void longBeep(uint8_t count);
extern void userBeep();
extern void encoderBeep(uint8_t count);
extern void startupBeep();
extern void setSignalPort(uint8_t port, bool state);
extern void signalNoTool();
extern void signalDuetBusy();
extern void signalDuetReady();
#if defined(USE_ZSERVO)
extern ZServo* getServoInstance(int8_t servoNum);
#else
extern Servo* getServoInstance(int8_t servoNum);
#endif
extern void attachServo(int8_t servoNum, pin_t pin);
extern void detachServo(int8_t servoNum);
extern void setServoMaxCycles(int8_t servoNum, uint8_t cycles);
extern bool isServoPulseComplete(int8_t servoNum);
extern bool isServoDisabled(int8_t servoNum);
extern bool setServoPos(int8_t servoNum, uint8_t degree);
extern bool setServoMS(int8_t servoNum, uint16_t microseconds);
extern void setServoLid(uint8_t pos);
extern void setServoMinPwm(int8_t servoNum, uint16_t pwm);
extern void setServoMaxPwm(int8_t servoNum, uint16_t pwm);
extern void disableServo(int8_t servoNum);
extern void enableServo(int8_t servoNum);
extern void setServoTickResolution(int8_t servoNum);
extern void getStoredData();
extern bool readTune(const char *filename, char* buffer, size_t length);
extern void readSequences();
extern bool readDebugLevel();
extern bool readConfig();
extern bool readTmcConfig();
extern bool readMaterials();
extern bool readServoMapping();
extern bool readRevolverMapping();
extern bool writeConfig(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool writeMainConfig(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool writeSteppersConfig(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool writeTmcConfig(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool writeServoMapping(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool writeMaterials(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool writeSwapTools(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool writeRevolverMapping(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool writefeedLoadState(Print* dumpTo = nullptr, bool useWebInterface = false);
extern bool deserializeSwapTools(const char* cfg);
extern bool saveConfig(String& buffer);
extern bool serializeTMCStats(Print* out, uint8_t axis, int8_t version, bool isStealth, uint16_t powerCfg, uint16_t powerRms, uint16_t microsteps, bool ms1, bool ms2, const char* uart, const char* diag, const char* ola, const char* olb, const char* s2ga, const char* s2gb, const char* ot_stat);
extern _File* openCfgFileWrite(const char* filename);
extern void closeCfgFile();
extern bool checkAutoClose();
extern void resetAutoClose();
extern bool checkUserMessage();
extern void setPwrSave(int8_t state);
extern void __log(const char *fmt, ...);
extern void __terminal(const char *fmt, ...);
extern void setAbortRequested(bool state);
extern void resetSerialBuffer(int8_t serial);
extern void checkSerialPending();
extern void drawSwapTool(uint8_t from, uint8_t with);
extern uint8_t swapTool(uint8_t index);
extern int8_t getToolSelected();
extern void positionRevolver();
extern bool feedToEndstop(char* errmsg, bool showMessage);
extern bool feedToNozzle(char* errmsg, bool showMessage);
extern bool unloadFromNozzle(char* errmsg, bool showMessage);
extern bool unloadFromDDE();
extern uint8_t splitStringLines(char *lines[], uint8_t maxLines, const char *message, const char *token = "\n");
extern void debounceButton();
extern bool checkStopMenu(unsigned startTime);
extern void drawTestrunMessage(unsigned long loop, char *msg);
extern bool getFiles(const char *rootFolder PROGMEM, const char *pattern PROGMEM, uint8_t maxFiles, bool cutExtension, char *files);
extern void testRun(const char *fname);
extern void moveFeeder(double distanceMM);
extern void overrideStepX(pin_t pin);
extern void overrideStepY(pin_t pin);
extern void overrideStepZ(pin_t pin);
extern void endstopEventX();
extern void endstopEventY();
extern void endstopEventZ();
extern void endstopEventZ2();
extern void isrEndstopX();
extern void isrEndstopY();
extern void isrEndstopZ();
extern void isrEndstopZ2();
extern bool checkDuetEndstop();
extern bool checkSplitterEndstop();
extern void readSplitterEndstops();
extern void isrSplitterEndstops();
extern void setToneTimerChannel(uint8_t ntimer, uint8_t channel);
extern void isrStepperTimerHandler();
extern void isrGPTimerHandler();
extern void isrLedTimerHandler();
extern void isrServoTimerHandler();
extern void isrStallDetectedX();
extern void isrStallDetectedY();
extern void isrStallDetectedZ();
extern void refreshStatus(bool feedOnly = false);
extern void every10ms();
extern void every20ms();
extern void every50ms();
extern void every100ms();
extern void every250ms();
extern void every500ms();
extern void every1s();
extern void every2s();
extern void every5s();
extern void blinkLED();
#ifdef HAS_TMC_SUPPORT
extern void setDriverSpreadCycle(TMC2209Stepper *driver, bool spread, uint8_t stallThrs, uint8_t csmin = 0, uint8_t csmax = 0, uint8_t csdown = 0, uint8_t toff = 3);
#endif
extern void terminalSend(uint8_t y, uint8_t x, const char* str, bool isCenter, uint8_t isInvert, bool clearLine = false);
extern void terminalClear(bool drawFrame = false);
extern void terminalDrawFrame(bool clear = false);
extern void terminalDrawSeparator(uint8_t y, uint8_t x, uint8_t color);
extern uint8_t terminalSendLines(uint8_t y, uint8_t x, const char* str, bool isCenter, uint8_t isInvert, bool clearLine);
extern uint8_t terminalSendButtons(uint8_t y, uint8_t x, const char* str, bool isCenter, uint8_t isInvert, bool clearLine);
extern void enumI2cDevices(uint8_t bus);

extern void printEndstopState(int8_t serial);
extern void printAcceleration(int8_t serial);
extern void printSpeedAdjust(int8_t serial);
extern void printSpeeds(int8_t serial);
extern void printDuetSignalStates(int8_t serial);
extern void sendGList(int8_t serial);
extern void sendMList(int8_t serial);
extern void sendM205List(int8_t serial);
extern void sendToolResponse(int8_t serial);
extern void sendStartResponse(int8_t serial);
extern void sendOkResponse(int8_t serial);
extern void sendBusyResponse(int8_t serial);
extern void sendErrorResponse(int8_t serial, const char *msg = nullptr);
extern void sendXon(Stream* serial);
extern void sendXoff(Stream* serial);
extern void parseGcode(const String &serialBuffer, int8_t serial);
extern bool parse_G(const String &buf, int8_t serial, char* errmsg);
extern bool parse_M(const String &buf, int8_t serial, char* errmsg);
extern bool parse_T(const String &buf, int8_t serial, char* errmsg);
extern bool parse_PMMU2(char cmd, const String &buf, int8_t serial, char* errmsg);
extern bool parse_Action(const String &buf, int8_t serial, char* errmsg);
extern int  getParam(String buf, const char *token);
extern long getParamL(String buf, const char *token);
extern double getParamF(String buf, const char *token);
extern bool hasParam(String buf, const char *token);
extern bool getParamString(String buf, const char *token, char *dest, int16_t bufLen);
extern void prepStepping(int8_t index, long param, bool Millimeter = true, bool ignoreEndstop = false);
extern void saveSettings(int8_t serial);
extern void reportSettings(int8_t serial);
extern void printResponse(const char *response);
extern void printResponse(const char *response, int8_t serial);
extern void printResponseP(const char *response, int8_t serial);
extern void printOffsets(int8_t serial);
extern void printDriverMode(int8_t serial);
extern void printDriverRms(int8_t serial);
extern void printDriverMS(int8_t serial);
extern void printDriverStallThrs(int8_t serial);
extern bool maintainTool(char* errmsg);
extern void printPeriodicalState(int8_t serial);
extern void prepareSequence(const char *sequence, bool autoPlay = true);
extern void playSequence();
extern void setParserBusy();
extern void setParserReady();
extern void runHomeAfterFeed();
extern void purgeFilament();

extern void showLed(uint8_t mode, uint8_t count);
extern void setBacklightIndex(int color);
extern void setToolColorIndex(int color);
extern void setContrast(int contrast);
extern void setFastLED(uint8_t index, CRGB color);
extern void setFastLEDIndex(uint8_t index, uint8_t color);
extern void setFastLEDToolIndex(uint8_t index, uint8_t color, bool setFlag = true);
extern void setFastLEDTools();
extern void setFastLEDIntensity(uint8_t intensity);
extern void testFastLED(bool tools);
extern void setFastLEDStatus();
extern void setFastLEDStatus(uint8_t status);
extern void setFastLEDToolsMarquee();
extern void setFastLEDToolsRainbow();
extern void setFastLEDToolsError();
extern void setFastLEDToolsWarning();
extern void setFastLEDToolsOk();

extern void switchFeederStepper(uint8_t stepper);
extern void removeFirmwareBin();
extern void showMemInfo(int8_t serial);
extern uint8_t scanI2CDevices(uint8_t *devices, uint8_t);
extern uint8_t scanI2C2Devices(uint8_t *devices, uint8_t);
extern bool initSD(bool showStatus = true);

extern unsigned long translateSpeed(uint16_t speed, uint8_t axis, bool forceTranslation = false);
extern bool getEncoderButton(bool encoderOnly);
extern void getEncoderButton(int16_t *turn, uint8_t *button, bool *isHeld, bool *isClicked);

extern void listTextFile(const char* filename PROGMEM, const char* filter, int8_t serial);
extern void listHelpFile(const char* filename PROGMEM, const char* filter, int8_t serial);
extern const char *loadMenu(const char* filename PROGMEM, int ordinals[], size_t maxLen);
extern const char *loadOptions(const char* filename PROGMEM, size_t maxLen);
extern bool loadReport(const char* filename PROGMEM, char* buffer, const char* ext, uint16_t maxLen);

extern void sendTMCStatus(uint8_t axis, int8_t port);
extern Print* getSerialInstance(int8_t serial);
extern void sendStates(bool override = false);
extern void setTestRunPending(const char* testfile);
extern void resetUpload();

extern bool loadToSplitter(char* errmsg, bool showMessage);
extern bool unloadFromSplitter(char* errmsg, bool showMessage);

extern void calcHwDebugCounter();

extern void showToolLeds();
extern void showBacklightLeds();
#if defined(USES_ADAFRUIT_NPX)
  #if defined(USE_FASTLED_TOOLS) || defined(USE_FASTLED_BACKLIGHT)
    extern uint32_t ColorRGB(uint8_t r, uint8_t g, uint8_t b);
    extern void ColorToRGB(uint32_t color, uint8_t* r, uint8_t* g, uint8_t* b);
    extern void nscale8(Adafruit_NeoPixel* instance, uint16_t num_leds, uint8_t scale);
    extern void fadeToBlackBy(Adafruit_NeoPixel* instance, uint16_t num_leds, uint8_t fadeBy);
    extern void assHSV(Adafruit_NeoPixel* instance, uint16_t index, uint32_t rgb);
  #endif
#endif 
extern void fastFlipDbg();
extern uint32_t parseJson(String& data);
extern void changeDebugPort(int param, char* errmsg, bool noMsg = false);


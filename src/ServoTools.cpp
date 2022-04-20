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

#if defined(USE_ZSERVO)
ZServo* getServoInstance(int8_t servoNum) {
#else
Servo* getServoInstance(int8_t servoNum) {
#endif
  switch(servoNum) {
    case SERVO_WIPER:   return &servoWiper;
    case SERVO_LID:     return &servoLid;
    case SERVO_CUTTER:  return &servoCutter;
  }
  return nullptr;
}

bool isServoPulseComplete(int8_t servoNum) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr)
    #if defined(USE_ZSERVO)
    return instance->isPulseComplete();
    #else
    return true;
    #endif
  return false;
}

void setServoMaxCycles(int8_t servoNum, uint8_t cycles) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    #if defined(USE_ZSERVO)
    instance->setMaxCycles(cycles);
    #endif
  }
}

void setServoMinPwm(int8_t servoNum, uint16_t pwm) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    #if defined(USE_ZSERVO)
    instance->setPulseWidthMin(pwm);
    #endif
  }
}

void setServoMaxPwm(int8_t servoNum, uint16_t pwm) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    #if defined(USE_ZSERVO)
    instance->setPulseWidthMax(pwm);
    #endif
  }
}

void setServoTickResolution(int8_t servoNum) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    #if defined(USE_ZSERVO)
    instance->setTickRes(SERVO_RESOLUTION);
    #endif
  }
}

void attachServo(int8_t servoNum, pin_t pin) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    #if defined(USE_ZSERVO)
    instance->attach(pin, true, servoNum);
    #else
    instance->attach(pin, smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
    #endif
  }
}

void detachServo(int8_t servoNum) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr)
    instance->detach();
}

void disableServo(int8_t servoNum) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    #if defined(USE_ZSERVO)
    instance->disable();
    #endif
  }
}

void enableServo(int8_t servoNum) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    #if defined(USE_ZSERVO)
    instance->enable();
    #endif
  }
}

uint8_t getServoDegree(int8_t servoNum) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    return instance->read();
  }
  return 0;
}

void setServoDelay(int8_t servoNum) {
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    #if defined(USE_ZSERVO)
    instance->setDelay();
    #else
    delay(300);
    #endif
  }
}

bool setServoPos(int8_t servoNum, uint8_t degree) {
#if defined(MULTISERVO)
  uint16_t pulseLen = map(degree, 0, 180, smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
  setServoMS(servoNum, pulseLen);
#else
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    instance->write(degree);
    setServoDelay(servoNum);
    return true;
  }
#endif
  return false;
}

bool setServoMS(int8_t servoNum, uint16_t microseconds) {
#if defined(MULTISERVO)
  int8_t index = -1;
  switch(servoNum) {
    case SERVO_WIPER:   index = 16; break;
    case SERVO_CUTTER:  index = 17; break;
    default:
      if(servoNum >= 10 && servoNum <= 26)
        index = servoNum - 10;
      break;
  }
  if(index != -1) {
    //__debugS(D, PSTR("Servo mapping: %d -> %d (pulse len: %d ms)"), servoNum-10, index, microseconds);
    if(servoMapping[index] != -1)
      servoPwm.writeMicroseconds(servoMapping[index], microseconds);
    return true;
  }
#else
  #if defined(USE_ZSERVO)
  ZServo* instance = getServoInstance(servoNum);
  #else
  Servo* instance = getServoInstance(servoNum);
  #endif
  if(instance != nullptr) {
    instance->writeMicroseconds(microseconds);
    setServoDelay(servoNum);
    return true;
  }
#endif
  return false;
}

void setServoLid(uint8_t pos)
{
#if !defined(SMUFF_V6S)
  #if !defined(MULTISERVO)
  uint8_t posForTool = (toolSelected < 0 || toolSelected > smuffConfig.toolCount-1) ? 0 : servoPosClosed[toolSelected];
  uint8_t p = (pos == SERVO_OPEN) ? smuffConfig.revolverOffPos : (posForTool == 0) ? smuffConfig.revolverOnPos : posForTool;
  if(getServoDegree(SERVO_LID) != p) {
    //__debugS(D, PSTR("setServoLid called with %d (posForTool: %d)"), pos, posForTool);
    setServoPos(SERVO_LID, p);
  }
  #else
  uint8_t p = (pos == SERVO_OPEN) ? servoPosClosed[toolSelected] - SERVO_CLOSED_OFS : servoPosClosed[toolSelected];
  //__debugS(D, PSTR("Tool%d = %d"), toolSelected, p);
  setServoPos(toolSelected + 10, p);
  #endif
  lidOpen = pos == SERVO_OPEN;
#else
  if(pos == SERVO_OPEN)
    moveHome(REVOLVER, false, false);
  else
    positionRevolver();
#endif
}


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

#if !defined(USE_MULTISERVO)
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
#else
#endif

bool isServoPulseComplete(int8_t servoNum) {
  #if !defined(USE_MULTISERVO)
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
  #endif
  return true;
}

bool isServoDisabled(int8_t servoNum) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      ZServo* instance = getServoInstance(servoNum);
      if(instance != nullptr)
          return instance->isDisabled();
    #endif
  #endif
  return false;
}

void setServoMaxCycles(int8_t servoNum, uint8_t cycles) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      ZServo* instance = getServoInstance(servoNum);
      if(instance != nullptr) {
        instance->setMaxCycles(cycles);
      }
    #endif
  #endif
}

void setServoMinPwm(int8_t servoNum, uint16_t pwm) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      ZServo* instance = getServoInstance(servoNum);
      if(instance != nullptr) {
        instance->setPulseWidthMin(pwm);
      }
    #endif
  #endif
}

void setServoMaxPwm(int8_t servoNum, uint16_t pwm) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      ZServo* instance = getServoInstance(servoNum);
      if(instance != nullptr) {
        instance->setPulseWidthMax(pwm);
      }
    #endif
  #endif
}

void setServoTickResolution(int8_t servoNum) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      ZServo* instance = getServoInstance(servoNum);
      if(instance != nullptr) {
        instance->setTickRes(SERVO_RESOLUTION);
      }
    #endif
  #endif
}

void attachServo(int8_t servoNum, pin_t pin) {
  #if !defined(USE_MULTISERVO)
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
  #endif
}

void detachServo(int8_t servoNum) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      ZServo* instance = getServoInstance(servoNum);
    #else
      Servo* instance = getServoInstance(servoNum);
    #endif
    if(instance != nullptr)
      instance->detach();
  #endif
}

void disableServo(int8_t servoNum) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      #if !defined(NEVER_DISABLE_SERVOS)
        ZServo* instance = getServoInstance(servoNum);
        if(instance != nullptr) {
            instance->disable();
        }
      #endif
    #endif
  #endif
}

void enableServo(int8_t servoNum) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      ZServo* instance = getServoInstance(servoNum);
      if(instance != nullptr) {
          instance->enable();
      }
    #endif
  #endif
}

uint8_t getServoDegree(int8_t servoNum) {
  #if !defined(USE_MULTISERVO)
    #if defined(USE_ZSERVO)
      ZServo* instance = getServoInstance(servoNum);
    #else
      Servo* instance = getServoInstance(servoNum);
    #endif
    if(instance != nullptr) {
      return instance->read();
    }
  #endif
  return 0;
}

void setServoDelay(int8_t servoNum) {
  #if !defined(USE_MULTISERVO)
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
  #endif
}

bool setServoPos(int8_t servoNum, uint8_t degree) {
  #if defined(USE_MULTISERVO)
    uint16_t pulseLen = map(degree, 0, 180, smuffConfig.servoMinPwm, smuffConfig.servoMaxPwm);
    setServoMS(servoNum, pulseLen);
    return true;
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
  #if defined(USE_MULTISERVO)
    //__debugS(D, PSTR("Servo mapping: %d -> %d (pulse len: %d ms)"), servoNum-10, index, microseconds);
    if(servoMapping[servoNum] != -1) {
      servoPwm.writeMicroseconds(servoMapping[servoNum], microseconds);
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

void setServoLid(uint8_t pos) {
  #if !defined(SMUFF_V6S)
    uint8_t posForTool = (toolSelected < 0 || toolSelected > smuffConfig.toolCount-1) ? 0 : servoPosClosed[toolSelected];
    uint8_t p = (pos == SERVO_OPEN) ? smuffConfig.revolverOffPos : (posForTool == 0) ? smuffConfig.revolverOnPos : posForTool;
    //__debugS(D, PSTR("setServoLid called with %d (posForTool: %d)"), pos, posForTool);
    setServoPos(SERVO_LID, p);
    lidOpen = pos == SERVO_OPEN;
  #else
    if(pos == SERVO_OPEN)
      moveHome(REVOLVER, false, false);
    else
      positionRevolver();
  #endif
}


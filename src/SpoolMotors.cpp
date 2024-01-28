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

#if defined(USE_SPOOLMOTOR)

extern int8_t motorPins[][3];

void testSpoolMotors() {
}

void setOutputs(Adafruit_PWMServoDriver* instance, uint8_t motor, uint8_t direction, uint8_t speed) {
    uint16_t pulse = speed * (4095/255);
    __debugS(DEV3, PSTR("Setting Motor Output; Motor: M%c Speed=%d, Pulse=%d"), 'A'+motor, speed, pulse);

    int pwm = motorPins[motor][0];
    if(direction == MOTOR_DIR_CW) {
        instance->setPWM(pwm, 0, pulse);                 // PWM = pulse
        instance->setPin(motorPins[motor][1], 0);        // IN1 = LOW
        instance->setPin(motorPins[motor][2], 4095);     // IN2 =  HIGH
    }
    else if(direction == MOTOR_DIR_CCW) {
        instance->setPWM(pwm, 0, pulse);                 // PWM = pulse
        instance->setPin(motorPins[motor][1], 4095);     // IN1 =  HIGH
        instance->setPin(motorPins[motor][2], 0);        // IN2 =  LOW
    }
    else {
        instance->setPin(pwm, 4095);                     // PWM = HIGH
        instance->setPin(motorPins[motor][1], 4095);     // IN1 = HIGH
        instance->setPin(motorPins[motor][2], 4095);     // IN2 = HIGH
    }
}

void doWind(uint8_t direction, int8_t motorIndex, uint8_t speed) {
    uint8_t board = (motorIndex == 0) ? 0 : motorIndex / MOTORS_PER_CTRL;
    uint8_t motor = motorIndex % MOTORS_PER_CTRL;

    __debugS(DEV3, PSTR("Winding spool motor; Direction %s; Board=%d, Motor=M%c, Speed=%d"), (direction == MOTOR_DIR_CW ? "CW" : (direction == MOTOR_DIR_CCW ? "CCW" : "STOP")), board+1, 'A'+motor, speed);
    Adafruit_PWMServoDriver* instance = nullptr;
    switch(board) {
        case 0:
            instance = &motor1Pwm;
            break;
        case 1:
            instance = &motor2Pwm;
            break;
        case 2: 
            instance = &motor3Pwm;
            break;
        default:
            break;
    }
    if(instance != nullptr)
        setOutputs(instance, motor, direction, speed);
}

void startRewindingSpool(int8_t tool) {
    if(tool == -1 || tool > MAX_TOOLS-1)
        return;
    if(spoolDirectionCCW[tool])
        windSpoolMotorCCW(tool, smuffConfig.spoolRewindSpeed, 0);
    else
        windSpoolMotorCW(tool, smuffConfig.spoolRewindSpeed, 0);
}

void stopRewindingSpool(int8_t tool) {
    if(tool == -1 || tool > MAX_TOOLS-1)
        return;
    stopWindingSpoolMotor(tool);
}

void windSpoolMotorCW(int8_t tool, uint8_t speed, uint16_t duration) {
    spoolDuration[tool] = 0;
    if(duration > 0) {
        spoolDuration[tool] = millis()+duration;
    }
    int8_t motor = spoolMappings[tool];
    if(motor != -1) {
        doWind(MOTOR_DIR_CW, motor, (uint8_t)speed);
    }
    else {
        __debugS(D, PSTR("No spool motor assigned for tool %d!"),tool);
    }
}

void windSpoolMotorCCW(int8_t tool, uint8_t speed, uint16_t duration) {
    spoolDuration[tool] = 0;
    if(duration > 0) {
        spoolDuration[tool] = millis()+duration;
    }
    int8_t motor = spoolMappings[tool];
    if(motor != -1) {
        doWind(MOTOR_DIR_CCW, motor, (uint8_t)speed);
    }
    else {
        __debugS(D, PSTR("No spool motor assigned for tool %d!"),tool);
    }
}

void stopWindingSpoolMotor(int8_t tool) {
    int8_t motor = spoolMappings[tool];
    if(motor != -1) {
        spoolDuration[tool] = 0;
        doWind(MOTOR_DIR_STOP, motor, 0);
    }
}
#endif

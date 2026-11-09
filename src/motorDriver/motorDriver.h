#pragma once
#ifndef __motorDriver_h__
#define __motorDriver_h__
#include <Arduino.h>
enum MotorDirection
{
    FORWARD = 1,
    REVERSE = -1,
};
enum BrakeMode
{
    BRAKE,
    FLOAT,
};
class MotorDriver
{
private:
    uint8_t int1Pin;
    uint8_t int2Pin;
    uint8_t int3Pin;
    uint8_t int4Pin;

public:
    MotorDriver(uint8_t int1, uint8_t int2, uint8_t int3, uint8_t int4)
    {
        int1Pin = int1;
        int2Pin = int2;
        int3Pin = int3;
        int4Pin = int4;
        pinMode(int1Pin, OUTPUT);
        pinMode(int2Pin, OUTPUT);
        pinMode(int3Pin, OUTPUT);
        pinMode(int4Pin, OUTPUT);
    }
    uint8_t getPositivePin(uint8_t motorNum)
    {
        switch (motorNum)
        {
        case 1:
            return int1Pin;
        case 2:
            return int3Pin;
        default:
            return 0;
        }
    }
    uint8_t getNegativePin(uint8_t motorNum)
    {
        switch (motorNum)
        {
        case 1:
            return int2Pin;
        case 2:
            return int4Pin;
        default:
            return 0;
        }
    }
};
class Motor
{
private:
    uint8_t positivePin;
    uint8_t negativePin;
    int16_t speed;
    MotorDirection direction;
    BrakeMode brakeMode;
    uint8_t minPWM = 40;
    void update()
    {
        if (speed == 0)
        {
            if (brakeMode == BRAKE)
            {
                digitalWrite(positivePin, HIGH);
                digitalWrite(negativePin, HIGH);
            }
            else
            {
                digitalWrite(positivePin, LOW);
                digitalWrite(negativePin, LOW);
            }
        }
        else
        {
            if (speed * direction > 0)
            {
                digitalWrite(negativePin, LOW);
                analogWrite(positivePin, compensateMotor(speed));
            }
            else
            {
                digitalWrite(positivePin, LOW);
                analogWrite(negativePin, compensateMotor(speed));
            }
        }
    }
    uint8_t compensateMotor(float command)
    {
        command = constrain(command, -255.0f, 255.0f);

        if (command == 0.0f)
            return 0;

        float pwm = minPWM + (255.0f - minPWM) * (fabs(command) / 255.0f);

        return abs((int)pwm);
    }

public:
    Motor(MotorDriver &driver, uint8_t num)
    {
        positivePin = driver.getPositivePin(num);
        negativePin = driver.getNegativePin(num);
        speed = 0;
        direction = FORWARD;
        brakeMode = FLOAT;
    }
    Motor &setDirection(MotorDirection dir)
    {
        direction = dir;
        update();
        return *this;
    }
    Motor &setSpeed(int16_t spd)
    {
        speed = spd;
        update();
        return *this;
    }
    Motor &setBreakingMode(BrakeMode mode)
    {
        brakeMode = mode;
        update();
        return *this;
    }
};

#endif // __motorDriver_h__

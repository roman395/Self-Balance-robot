#pragma once
#ifndef __mpu6500_h__
#define __mpu6500_h__
#include <Wire.h>
enum GyroRange
{
    GYRO_RANGE_250DPS = 0b00000000,
    GYRO_RANGE_500DPS = 0b00001000,
    GYRO_RANGE_1000DPS = 0b00010000,
    GYRO_RANGE_2000DPS = 0b0011000
};
enum AccelRange
{
    ACCEL_RANGE_2G = 0b00000000,
    ACCEL_RANGE_4G = 0b00001000,
    ACCEL_RANGE_8G = 0b00010000,
    ACCEL_RANGE_16G = 0b0011000
};
class acceleration
{
private:
    float xa;
    float ya;
    float za;

public:
    acceleration(float x, float y, float z) : xa(x), ya(y), za(z) {};
    acceleration &operator=(const acceleration &other)
    {
        if (this != &other)
        {
            xa = other.xa;
            ya = other.ya;
            za = other.za;
        }
        return *this;
    }
    float getXa() { return xa; }
    float getYa() { return ya; }
    float getZa() { return za; }
    String toString()
    {
        return "Xa: " + String(xa) + ", Ya: " + String(ya) + ", Za: " + String(za);
    }
};
class angleSpeed
{
private:
    float xg;
    float yg;
    float zg;

public:
    angleSpeed(float x, float y, float z) : xg(x), yg(y), zg(z) {}
    angleSpeed &operator=(const angleSpeed &other)
    {
        if (this != &other)
        {
            xg = other.xg;
            yg = other.yg;
            zg = other.zg;
        }
        return *this;
    }
    float getXg() { return xg; }
    float getYg() { return yg; }
    float getZg() { return zg; }
    String toString()
    {
        return "Xg: " + String(xg) + ", Yg: " + String(yg) + ", Zg: " + String(zg);
    }
};
class MPU6500
{
private:
    const uint8_t MPU6500_ADDRESS = 0x69;
    const uint8_t MPU6500_PWR_MGMT_1 = 0x6B;
    const uint8_t MPU6500_ACCEL_XOUT_H = 0x3B;
    const uint8_t MPU6500_ACCEL_YOUT_H = 0x3D;
    const uint8_t MPU6500_ACCEL_ZOUT_H = 0x3F;
    const uint8_t MPU6500_GYRO_XOUT_H = 0x43;
    const uint8_t MPU6500_GYRO_YOUT_H = 0x45;
    const uint8_t MPU6500_GYRO_ZOUT_H = 0x47;
    const uint8_t MPU6500_TEMP_OUT_H = 0x41;
    const uint8_t MPU6500_GYRO_CONFIG = 0x1B;
    const uint8_t MPU6500_ACCEL_CONFIG = 0x1C;
    TwoWire &i2c = Wire;
    uint8_t address = MPU6500_ADDRESS;
    acceleration lastAcceleration = acceleration(0, 0, 0);
    angleSpeed lastAngleSpeed = angleSpeed(0, 0, 0);
    float lastTemperature = 0;
    float accelDivider = 16384.0f;
    float gyroDivider = 131.0f;

public:
    MPU6500();
    MPU6500(uint8_t addr);
    MPU6500(uint8_t addr, TwoWire &wire);
    bool begin();
    acceleration getAcceleration();
    angleSpeed getAngleSpeed();
    float getTemperature();
    bool setGyroRange(GyroRange range);
    bool setAccelRange(AccelRange range);
};
#endif // __mpu6500_h__
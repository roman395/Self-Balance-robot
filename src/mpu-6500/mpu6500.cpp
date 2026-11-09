#include "mpu6500.h"
MPU6500::MPU6500() : i2c(Wire), address(MPU6500_ADDRESS) {}
MPU6500::MPU6500(uint8_t addr) : i2c(Wire), address(addr) {}
MPU6500::MPU6500(uint8_t addr, TwoWire &wire) : i2c(wire), address(addr) {}
bool MPU6500::begin()
{
    i2c.begin();
    i2c.beginTransmission(address);
    i2c.write(MPU6500_PWR_MGMT_1);
    i2c.write(0x00);
    setGyroRange(GYRO_RANGE_250DPS);
    setAccelRange(ACCEL_RANGE_2G);
    return i2c.endTransmission() == 0;
}

bool MPU6500::setGyroRange(GyroRange range)
{
    i2c.beginTransmission(address);
    i2c.write(MPU6500_GYRO_CONFIG);
    if (i2c.endTransmission(false) != 0)
        return false;
    if (i2c.requestFrom(address, (uint8_t)1) != 1)
        return false;
    uint8_t currentConfig = i2c.read();
    currentConfig = (currentConfig & 0b11100111) | range;
    i2c.beginTransmission(address);
    i2c.write(MPU6500_GYRO_CONFIG);
    i2c.write(currentConfig);
    if (i2c.endTransmission() != 0)
        return false;

    switch (range)
    {
    case GYRO_RANGE_250DPS:
        gyroDivider = 131.0f;
        break;
    case GYRO_RANGE_500DPS:
        gyroDivider = 65.5f;
        break;
    case GYRO_RANGE_1000DPS:
        gyroDivider = 32.8f;
        break;
    case GYRO_RANGE_2000DPS:
        gyroDivider = 16.4f;
        break;
    }

    return true;
}
bool MPU6500::setAccelRange(AccelRange range)
{
    i2c.beginTransmission(address);
    i2c.write(MPU6500_ACCEL_CONFIG);
    if (i2c.endTransmission(false) != 0)
        return false;
    if (i2c.requestFrom(address, (uint8_t)1) != 1)
        return false;
    uint8_t currentConfig = i2c.read();
    currentConfig = (currentConfig & 0b11100111) | range;
    i2c.beginTransmission(address);
    i2c.write(MPU6500_ACCEL_CONFIG);
    i2c.write(currentConfig);
    if (i2c.endTransmission() != 0)
        return false;

    switch (range)
    {
    case ACCEL_RANGE_2G:
        accelDivider = 16384.0f;
        break;
    case ACCEL_RANGE_4G:
        accelDivider = 8192.0f;
        break;
    case ACCEL_RANGE_8G:
        accelDivider = 4096.0f;
        break;
    case ACCEL_RANGE_16G:
        accelDivider = 2048.0f;
        break;
    }

    return true;
}
acceleration MPU6500::getAcceleration()
{
    i2c.beginTransmission(address);
    i2c.write(MPU6500_ACCEL_XOUT_H);
    if (i2c.endTransmission(false) != 0)
        return lastAcceleration;
    uint8_t count = i2c.requestFrom(address, (uint8_t)6);
    if (count != 6)
        return lastAcceleration;
    uint8_t axh = i2c.read();
    uint8_t axl = i2c.read();
    uint8_t ayh = i2c.read();
    uint8_t ayl = i2c.read();
    uint8_t azh = i2c.read();
    uint8_t azl = i2c.read();
    int16_t ax = (axh << 8) | axl;
    int16_t ay = (ayh << 8) | ayl;
    int16_t az = (azh << 8) | azl;
    lastAcceleration = acceleration((float)ax / accelDivider, (float)ay / accelDivider, (float)az / accelDivider);
    return lastAcceleration;
}
angleSpeed MPU6500::getAngleSpeed()
{
    i2c.beginTransmission(address);
    i2c.write(MPU6500_GYRO_XOUT_H);
    if (i2c.endTransmission(false) != 0)
        return lastAngleSpeed;
    uint8_t count = i2c.requestFrom(address, (uint8_t)6);
    if (count != 6)
        return lastAngleSpeed;
    uint8_t axh = i2c.read();
    uint8_t axl = i2c.read();
    uint8_t ayh = i2c.read();
    uint8_t ayl = i2c.read();
    uint8_t azh = i2c.read();
    uint8_t azl = i2c.read();
    int16_t ax = (axh << 8) | axl;
    int16_t ay = (ayh << 8) | ayl;
    int16_t az = (azh << 8) | azl;
    lastAngleSpeed = angleSpeed((float)ax / gyroDivider, (float)ay / gyroDivider, (float)az / gyroDivider);
    return lastAngleSpeed;
}
float MPU6500::getTemperature()
{
    i2c.beginTransmission(address);
    i2c.write(MPU6500_TEMP_OUT_H);
    if (i2c.endTransmission(false) != 0)
        return lastTemperature;
    uint8_t count = i2c.requestFrom(address, (uint8_t)2);
    if (count != 2)
        return lastTemperature;
    uint8_t th = i2c.read();
    uint8_t tl = i2c.read();
    int16_t t = (int16_t)(((uint16_t)th << 8) | tl);
    lastTemperature = t / 333.87f + 21.0f;
    return lastTemperature;
}
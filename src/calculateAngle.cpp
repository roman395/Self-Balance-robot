#include "calculations.h"
#include <math.h>
#include <Arduino.h>
float angle = 0;
float gyroOffsetY = 0; // Подставь среднее gy неподвижного датчика
unsigned long previousTime = 0;
bool initialized = false;

void updateAngle(float ax, float az, float gy)
{
    uint32_t now = micros();
    float accelAngle = atan2(-ax, az) * RAD_TO_DEG;

    if (!initialized)
    {
        angle = accelAngle; // Начальный угол, даже если робот наклонён
        previousTime = now;
        initialized = true;
        return;
    }

    float dt = (uint32_t)(now - previousTime) * 1e-6f;
    previousTime = now;

    const float tau = 0.5f; // Начальная настройка фильтра, секунды
    float alpha = tau / (tau + dt);

    float gyroAngle = angle + (gy - gyroOffsetY) * dt;
    angle = alpha * gyroAngle + (1.0f - alpha) * accelAngle;
}
float getAngle()
{
    return angle;
}
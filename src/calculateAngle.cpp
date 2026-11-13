#include "calculations.h"
#include <math.h>
#include <Arduino.h>
float angle = 0;
float gyroOffsetY = 0; // Подставь среднее gy неподвижного датчика
unsigned long previousTime = 0;
bool initialized = false;

void updateAngle(float axForward, float azUp, float gyLeft)
{
    uint32_t now = micros();
    // Forward lean gives negative X acceleration and positive rotation about +Y.
    float accelAngle = atan2(-axForward, azUp) * RAD_TO_DEG;

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

    float gyroAngle = angle + (gyLeft - gyroOffsetY) * dt;
    angle = alpha * gyroAngle + (1.0f - alpha) * accelAngle;
}
float getAngle()
{
    return angle;
}

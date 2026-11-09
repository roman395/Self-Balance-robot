#pragma once
#ifndef __pid_h__
#define __pid_h__
class PIDCoefficients
{
private:
    float kp;
    float ki;
    float kd;

public:
    PIDCoefficients(float p, float i, float d) : kp(p), ki(i), kd(d) {}
    PIDCoefficients() : kp(0), ki(0), kd(0) {}
    PIDCoefficients &operator=(const PIDCoefficients &other)
    {
        if (this != &other)
        {
            kp = other.kp;
            ki = other.ki;
            kd = other.kd;
        }
        return *this;
    }
    float getKp() { return kp; }
    float getKi() { return ki; }
    float getKd() { return kd; }
};

class PIDController
{
private:
    PIDCoefficients coefficients;
    float target = 0;
    float lastOutput = 0;
    bool initialized = false;
    float errorSum;
    float lastError;
    float lastTime;
    float maxOutput = 255;
    float minOutput = -255;
    float maxIntegral = 255;
    float minIntegral = -255;

public:
    PIDController(PIDCoefficients coeffs) : coefficients(coeffs), errorSum(0), lastError(0), lastTime(0) {}
    PIDController() : coefficients(), errorSum(0), lastError(0), lastTime(0) {}
    void setCoefficients(PIDCoefficients coeffs)
    {
        coefficients = coeffs;
    }
    PIDCoefficients getCoefficients() const
    {
        return coefficients;
    }
    float compute(float setpoint, float measuredValue, float currentTime)
    {
        float error = setpoint - measuredValue;
        // currentTime is in seconds. Start without a derivative kick.
        if (!initialized)
        {
            lastError = error;
            lastTime = currentTime;
            initialized = true;
            lastOutput = clamp(coefficients.getKp() * error, minOutput, maxOutput);
            return lastOutput;
        }
        float deltaTime = currentTime - lastTime;

        if (deltaTime <= 0.0f)
            return lastOutput;

        errorSum += error * deltaTime;
        float dError = (error - lastError) / deltaTime;

        float output = coefficients.getKp() * error + clamp(coefficients.getKi() * errorSum, minIntegral, maxIntegral) + coefficients.getKd() * dError;

        lastError = error;
        lastTime = currentTime;

        lastOutput = clamp(output, minOutput, maxOutput);
        return lastOutput;
    }
    float compute(float measuredValue, float currentTime)
    {
        return compute(target, measuredValue, currentTime);
    }
    void setSetpoint(float setpoint)
    {
        target = setpoint;
        lastError = 0;
        errorSum = 0;
        lastTime = 0;
        lastOutput = 0;
        initialized = false;
    }
    float clamp(float value, float min, float max)
    {
        if (value < min)
            return min;
        else if (value > max)
            return max;
        else
            return value;
    }
    void setOutputLimits(float min, float max)
    {
        minOutput = min;
        maxOutput = max;
    }
    void setIntegralLimits(float min, float max)
    {
        minIntegral = min;
        maxIntegral = max;
    }
    float getError() const
    {
        return lastError;
    }
};
#endif // __pid_h__

#include <Arduino.h>
#include "mpu-6500/mpu6500.h"
#include "calculations.h"
#include "motorDriver/motorDriver.h"
#include "pid/pid.h"
#include "pid/pidTerminal.h"

MPU6500 mpu = MPU6500();
MotorDriver motorDriver = MotorDriver(3, 9, 10, 11);
Motor motorLeft = Motor(motorDriver, 1);
Motor motorRight = Motor(motorDriver, 2);
PIDCoefficients pidCoeffs = PIDCoefficients(-20.0f, 0.0f, -0.5f);
PIDController pid = PIDController(pidCoeffs);
PIDTerminal pidTerminal = PIDTerminal(Serial, pid);
void setup()
{
  Serial.begin(115200);
  if (!mpu.begin())
    Serial.println("MPU6500 не отвечает");
  motorLeft.setDirection(REVERSE).setSpeed(0).setBreakingMode(BRAKE);
  motorRight.setDirection(FORWARD).setSpeed(0).setBreakingMode(BRAKE);
  pid.setSetpoint(0.3f);
  pidTerminal.begin();
}

void loop()
{
  pidTerminal.update();
  updateAngle(mpu.getAcceleration().getXa(), mpu.getAcceleration().getZa(), mpu.getAngleSpeed().getYg());
  float angle = getAngle();
  float power = pid.compute(angle, millis() * 0.001f);
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500)
  {
    lastPrint = millis();
    Serial.print("Angle: ");
    Serial.println(angle);
    Serial.print("Power: ");
    Serial.println(power);
  }
  motorLeft.setSpeed(power);
  motorRight.setSpeed(power);
}

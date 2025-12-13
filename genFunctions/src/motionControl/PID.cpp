#include "motionControl/PID.h"


MotionPID::MotionPID(AsymptoticGains& Kp, float Ki, float Kd, float integralDeadband, bool integralSignReset)
  //constructor for pid object

  : Kp(Kp)
  , Ki(Ki)
  , Kd(Kd)
  , integralDeadband(integralDeadband)
  , integralSignReset(integralSignReset)
{}


float MotionPID::tick(float error) {
  //calculates one pid iteration and returns output

  this->error = error;
  if (fabs(error) > integralDeadband) totalError = 0;
  else if (integralSignReset && std::signbit(error) != std::signbit(lastError)) totalError = 0;
  else totalError += error * 0.01;
  derivative = (error - lastError) * 100.0;
  lastError = error;

  return Kp.getGain() * error + Ki * totalError + Kd * derivative;
}


void MotionPID::reset(float initialError) {
  //resets variables between movements

  lastError = initialError;
  totalError = 0;
}


void MotionPID::setKp(float setpoint) {
  //sets the Kp based on the initial error

  Kp.setGain(setpoint);
}


float MotionPID::getError() {
  //returns the error of the PID controller

  return error;
}
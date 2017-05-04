//
// Created by Fan Jiang on 2017/5/3.
//
#include "math.h"
#include "bsp/pid.h"


PID::PID(float p, float i, float d, float deltat) {
  Kp = p;
  Ki = i;
  Kd = d;
  maxIntegral = -1;
  dt = deltat;
}

void PID::initialize(float error) {
  lastError = error;
  integral = 0;
}

float PID::run(float error) {
  float result;

  if (maxIntegral > 0) {
    integral += error * dt;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
  } else {
    integral += error * dt;
  }

  result = Kp * error + Kd * (error - lastError) / dt + Ki * integral;

  lastError = error;

  return result;
}
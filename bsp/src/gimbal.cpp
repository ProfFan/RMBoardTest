//
// Created by Fan Jiang on 2017/5/3.
//

#include "bsp/gimbal.h"
#include "bsp/param.h"
#include "bsp/imu.h"
#include "cmsis_os.h"

osThreadId gimbalTaskHandle;

Gimbal *gimbal = new Gimbal(can2);

Gimbal::Gimbal(CAN* hcan) :
    pid_pitch(params->raw.PIDPitchKp, params->raw.PIDPitchKi, params->raw.PIDPitchKd, 0.001),
    pid_pitch_v(params->raw.PIDPitchKpV, 0, params->raw.PIDPitchKdV, 0.001),

    pid_yaw(params->raw.PIDYawKp, params->raw.PIDYawKi, params->raw.PIDYawKd, 0.001),
    pid_yaw_v(params->raw.PIDYawKp, 0, params->raw.PIDYawKdV, 0.001),
    pitch_motor(CAN_PIT_FEEDBACK_ID,hcan),
    yaw_motor(CAN_YAW_FEEDBACK_ID,hcan)
{
  this->hcan = hcan;
}

void Gimbal::Run(){
  if(armed){

  } else {

  }
}

void Gimbal::Arm(){
  armed = true;
}

void Gimbal::Disarm() {
  armed = false;
}

extern "C" void StartGimbalTask(void const *argument) {
  while(!ahrs->healthy) osDelay(100);

  for(;;){
    gimbal->Run();
    osDelay(1);
  }
}
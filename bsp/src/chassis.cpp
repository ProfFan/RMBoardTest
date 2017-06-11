//
// Created by Fan Jiang on 2017/5/17.
//

#include "bsp/chassis.h"
#include "bsp/gimbal.h"
#include "bsp/imu.h"
#include "cmsis_os.h"

osThreadId chassisTaskHandle;

Chassis *chassis = new Chassis(can1);

Chassis::Chassis(CAN *hcan) :
    motorLF(CAN_MotorLF_ID, hcan),
    motorRF(CAN_MotorRF_ID, hcan),
    motorLB(CAN_MotorLB_ID, hcan),
    motorRB(CAN_MotorRB_ID, hcan),
    pidLF(1, 0, 0, 0.01), pidRF(1, 0, 0, 0.01), pidLB(1, 0, 0, 0.01), pidRB(1, 0, 0, 0.01)
{

}

extern "C" void StartChassisTask(void const *argument) {
  while(!ahrs->healthy) osDelay(100);

  for(;;){
    osDelay(1);
  }
}
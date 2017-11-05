//
// Created by Fan Jiang on 2017/5/17.
//

#include "bsp/chassis.h"
#include "bsp/gimbal.h"
#include "bsp/imu.h"
#include "cmsis_os.h"

osThreadId chassisTaskHandle;

Chassis *chassis;

Chassis::Chassis(CAN *hcan) :
    motorLF(CAN_MotorLF_ID, hcan),
    motorRF(CAN_MotorRF_ID, hcan),
    motorLB(CAN_MotorLB_ID, hcan),
    motorRB(CAN_MotorRB_ID, hcan),
    pidLF(1, 0, 0, 0.01), pidRF(1, 0, 0, 0.005), pidLB(1, 0, 0, 0.005), pidRB(1, 0, 0, 0.005) {

  this->hcan = hcan;
}

void Chassis::Run(float lf, float rf, float lb, float rb) {

  int16_t iq[4] = {
      pidLF.run(lf),
      pidRF.run(rf),
      pidLB.run(lb),
      pidRB.run(rb),
  };

  CAN_HandleTypeDef *hrawcan = hcan->hcan;

  hrawcan->pTxMsg->StdId = 0x200;
  hrawcan->pTxMsg->IDE = CAN_ID_STD;
  hrawcan->pTxMsg->RTR = CAN_RTR_DATA;
  hrawcan->pTxMsg->DLC = 0x08;
  hrawcan->pTxMsg->Data[0] = (uint8_t)(iq[0] >> 8);
  hrawcan->pTxMsg->Data[1] = (uint8_t)iq[0];
  hrawcan->pTxMsg->Data[2] = (uint8_t)(iq[1] >> 8);
  hrawcan->pTxMsg->Data[3] = (uint8_t)iq[1];
  hrawcan->pTxMsg->Data[4] = (uint8_t)(iq[2] >> 8);
  hrawcan->pTxMsg->Data[5] = (uint8_t)iq[2];
  hrawcan->pTxMsg->Data[6] = (uint8_t)(iq[3] >> 8);
  hrawcan->pTxMsg->Data[7] = (uint8_t)iq[3];
  HAL_CAN_Transmit(hrawcan, 1);

}

void Chassis::Arm() {

}

void Chassis::Disarm() {

}

extern "C" void StartChassisTask(void const *argument) {
  while (!ahrs->healthy) osDelay(100);

  while (can1 == nullptr) osDelay(100);

  chassis = new Chassis(can1);

  for (;;) {
    //chassis->Run();

    CAN_HandleTypeDef *hrawcan;

    hrawcan = chassis->hcan->hcan;

    hrawcan->pTxMsg->StdId = 0x200;
    hrawcan->pTxMsg->IDE = CAN_ID_STD;
    hrawcan->pTxMsg->RTR = CAN_RTR_DATA;
    hrawcan->pTxMsg->DLC = 0x08;
    hrawcan->pTxMsg->Data[0] = (uint8_t)(chassis->motorLF.targetCurrent >> 8);
    hrawcan->pTxMsg->Data[1] = (uint8_t)chassis->motorLF.targetCurrent;
    hrawcan->pTxMsg->Data[2] = (uint8_t)(chassis->motorLB.targetCurrent >> 8);
    hrawcan->pTxMsg->Data[3] = (uint8_t)chassis->motorLB.targetCurrent;
    hrawcan->pTxMsg->Data[4] = (uint8_t)(chassis->motorRF.targetCurrent >> 8);
    hrawcan->pTxMsg->Data[5] = (uint8_t)chassis->motorRF.targetCurrent;
    hrawcan->pTxMsg->Data[6] = (uint8_t)(chassis->motorRB.targetCurrent >> 8);
    hrawcan->pTxMsg->Data[7] = (uint8_t)chassis->motorRB.targetCurrent;
    HAL_CAN_Transmit(hrawcan, 1);

    osDelay(2);
  }
}
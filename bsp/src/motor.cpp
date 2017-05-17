//
// Created by Fan Jiang on 2017/5/4.
//

#include "bsp/bsp_can.h"
#include "bsp/motor.h"

Motor::Motor(uint32_t id, CAN* bus) {
  this->bus = bus;
  canID = id;
  current = 0;
  angle = 0;
  targetCurrent = 0;
  round = 0;
  totalAngle = 0;
  offsetAngle = 0;
  bus->registerCallback(id, std::bind(&Motor::UpdateSensorData, this, std::placeholders::_1));
}

void Motor::UpdateSensorData(CanRxMsgTypeDef *message) {
  lastAngle = angle;
  angle = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
  current  = (int16_t)(message->Data[2] << 8 | message->Data[3]);
  speedRPM = current; //2 moto driver have diff info.
  targetCurrent = (int16_t)(message->Data[4] << 8 | message->Data[5]) / -5;
  // ptr->hall          = message->Data[6];
  if (angle - lastAngle > 4096)
    round--;
  else if (angle - lastAngle < -4096)
    round++;
  totalAngle = round * 8192 + angle - offsetAngle;
}

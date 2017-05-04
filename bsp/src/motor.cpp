//
// Created by Fan Jiang on 2017/5/4.
//

#include "bsp/motor.h"

Motor::Motor(CAN_HandleTypeDef *hcan) {
  this->hcan = hcan;
  current = 0;
  angle = 0;
}

void Motor::UpdateSensors() {

}

void Motor::SetCurrent() {

}
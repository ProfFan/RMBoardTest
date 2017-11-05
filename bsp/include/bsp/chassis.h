//
// Created by Fan Jiang on 2017/5/17.
//

#ifndef CHASSIS_CHASSIS_H
#define CHASSIS_CHASSIS_H

#include "bsp/pid.h"
#include "bsp/motor.h"
#include "bsp/sbus.h"

#ifdef __cplusplus
extern "C" {
#endif

void StartChassisTask(void const *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "can.h"

class Chassis {
public:

  explicit Chassis(CAN *hcan);

  void Run(float lf, float rf, float lb, float rb);

  void Arm();

  void Disarm();

  Motor motorLF, motorRF, motorLB, motorRB;

  CAN* hcan;
private:
  bool armed;
  PID pidLF, pidRF, pidLB, pidRB;
};

extern Chassis *chassis;
#endif

#endif //CHASSIS_CHASSIS_H

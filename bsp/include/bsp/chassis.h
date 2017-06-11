//
// Created by Fan Jiang on 2017/5/17.
//

#ifndef CHASSIS_CHASSIS_H
#define CHASSIS_CHASSIS_H

#include "bsp/pid.h"
#include "bsp/motor.h"

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

  Chassis(CAN *hcan);

  float pitch_encoder = 0;
  float pitch_encoder_offset = 0;

  float yaw_encoder = 0;
  float yaw_encoder_offset = 0;

  void Run();

  void Arm();

  void Disarm();

  Motor motorLF, motorRF, motorLB, motorRB;

private:
  bool armed;
  PID pidLF, pidRF, pidLB, pidRB;

  CAN* hcan;
};

extern Chassis *chassis;
#endif

#endif //CHASSIS_CHASSIS_H

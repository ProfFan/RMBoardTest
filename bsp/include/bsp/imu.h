//
// Created by Fan Jiang on 2017/5/2.
//

#ifndef CHASSIS_IMU_H
#define CHASSIS_IMU_H

#include <stdint.h>

#include "bsp/mpu6500.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifdef __cplusplus
extern "C" {
#endif
void StartIMUTask(void const * argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class AHRS {
public:
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  AHRS();
  uint64_t lastUpdate, currentUpdate;

  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
private:
  float beta;
};

extern AHRS* ahrs;
extern MPU6500* mpu;
#endif
#endif //CHASSIS_IMU_H

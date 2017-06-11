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
void StartIMUTask(void const *argument);

#ifdef __cplusplus
}
#endif

#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)

#ifdef __cplusplus

class AHRS {
public:
  void
  MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

  void MahonyAHRSUpdate(float gx, float gy, float gz,
                        bool useAcc, float ax, float ay, float az,
                        bool useMag, float mx, float my, float mz,
                        bool useYaw, float yawError);

  void ComputeRotationMatrix(void);

  void UpdateRPY();

  AHRS();

  uint64_t lastUpdate, currentUpdate;

  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float rMat[3][3];
  float dcm_kp = 1;
  float dcm_ki = 0.0;
  bool healthy = false;
  float beta = 1.753340;

  float roll = 0, pitch = 0, yaw = 0;
private:

  float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki
};

extern AHRS *ahrs;
extern MPU6500 *mpu;
#endif
#endif //CHASSIS_IMU_H

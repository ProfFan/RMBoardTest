//
// Created by Fan Jiang on 2017/5/2.
//

#include <bsp/param.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "math.h"
#include "spi.h"
#include "bsp/imu.h"
#include "bsp/hptimer.h"
#include "bsp/param.h"

osThreadId imuTaskHandle;

MPU6500 *mpu = new MPU6500(&hspi5);

AHRS *ahrs = new AHRS();

AHRS::AHRS() {
  // parameters for 6 DoF sensor fusion calculations
//  float GyroMeasError = M_PI * (45.0f /
//                                180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
  //beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
  //float GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  ComputeRotationMatrix();
}

void AHRS::MadgwickQuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my,
                                    float mz) {
  float deltat = (float) (ahrs->currentUpdate - ahrs->lastUpdate) / SystemCoreClock;
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) -
       _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
       (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
       _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) -
       4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) +
       _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
       (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
       (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) -
       4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) +
       (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
       (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
       (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) +
       (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
       (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
       _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}

#define sq(x) x*x
#define SPIN_RATE_LIMIT 20

void AHRS::ComputeRotationMatrix(void) {
  float q1q1 = sq(q[1]);
  float q2q2 = sq(q[2]);
  float q3q3 = sq(q[3]);

  float q0q1 = q[0] * q[1];
  float q0q2 = q[0] * q[2];
  float q0q3 = q[0] * q[3];
  float q1q2 = q[1] * q[2];
  float q1q3 = q[1] * q[3];
  float q2q3 = q[2] * q[3];

  rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
  rMat[0][1] = 2.0f * (q1q2 + -q0q3);
  rMat[0][2] = 2.0f * (q1q3 - -q0q2);

  rMat[1][0] = 2.0f * (q1q2 - -q0q3);
  rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
  rMat[1][2] = 2.0f * (q2q3 + -q0q1);

  rMat[2][0] = 2.0f * (q1q3 + -q0q2);
  rMat[2][1] = 2.0f * (q2q3 - -q0q1);
  rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;

}

void AHRS::MahonyAHRSUpdate(float gx, float gy, float gz,
                            bool useAcc, float ax, float ay, float az,
                            bool useMag, float mx, float my, float mz,
                            bool useYaw, float yawError) {
  float dt = (float) (ahrs->currentUpdate - ahrs->lastUpdate) / SystemCoreClock;
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];   // short name local variable for readability

  beta = sqrt(3.0f / 4.0f) * M_PI * (1.0f / 180.0f) * (HAL_GetTick() < 100000) ? 60.0f : 1.0f;

  // Calculate general spin rate (rad/s)
  const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

  // Use raw heading error (from GPS or whatever else)
  float ez = 0;
  if (useYaw) {
    while (yawError > M_PI) yawError -= (2.0f * M_PI);
    while (yawError < -M_PI) yawError += (2.0f * M_PI);

    ez += sin(yawError / 2.0f);
  }

  // Use measured magnetic field vector
  float ex = 0, ey = 0;
  float recipNorm = sq(mx) + sq(my) + sq(mz);
  if (useMag && recipNorm > 0.01f) {
    // Normalise magnetometer measurement
    recipNorm = 1.0f / sqrtf(recipNorm);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
    // This way magnetic field will only affect heading and wont mess roll/pitch angles

    // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
    // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
    const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
    const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
    const float bx = sqrtf(hx * hx + hy * hy);

    // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
    const float ez_ef = -(hy * bx);

    // Rotate mag error vector back to BF and accumulate
    ex += rMat[2][0] * ez_ef;
    ey += rMat[2][1] * ez_ef;
    ez += rMat[2][2] * ez_ef;
  }

  // Use measured acceleration vector
  recipNorm = sq(ax) + sq(ay) + sq(az);
  if (useAcc && recipNorm > 0.01f) {
    // Normalise accelerometer measurement
    recipNorm = 1.0f / sqrtf(recipNorm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Error is sum of cross product between estimated direction and measured direction of gravity
    ex += (ay * rMat[2][2] - az * rMat[2][1]);
    ey += (az * rMat[2][0] - ax * rMat[2][2]);
    ez += (ax * rMat[2][1] - ay * rMat[2][0]);
  }

  // Compute and apply integral feedback if enabled
  if (dcm_ki > 0.0f) {
    // Stop integrating if spinning beyond the certain limit
    if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
      const float dcmKiGain = dcm_ki;
      integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
      integralFBy += dcmKiGain * ey * dt;
      integralFBz += dcmKiGain * ez * dt;
    }
  } else {
    integralFBx = 0.0f;    // prevent integral windup
    integralFBy = 0.0f;
    integralFBz = 0.0f;
  }

  // Calculate kP gain. If we are acquiring initial attitude (not armed and within 20 sec from powerup) scale the kP to converge faster
  const float dcmKpGain = dcm_kp * (HAL_GetTick() < 10000) ? 10.0f : 1.0f;

  // Apply proportional and integral feedback
  gx += dcmKpGain * ex + integralFBx;
  gy += dcmKpGain * ey + integralFBy;
  gz += dcmKpGain * ez + integralFBz;

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);

  const float qa = q0;
  const float qb = q1;
  const float qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0f / sqrtf(sq(q0) + sq(q1) + sq(q2) + sq(q3));
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
  // Pre-compute rotation matrix from quaternion
  ComputeRotationMatrix();
}

void AHRS::UpdateRPY() {
  yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}

#undef sq

float lastPitch = 1000;

void StartIMUTask(void const *argument) {
  float dt;
  while (mpu->initialize()) osDelay(100);
  ahrs->lastUpdate = GetCycleCount64();
  /* Infinite loop */
  for (;;) {
    ahrs->currentUpdate = GetCycleCount64();
    mpu->readRawData();
    dt = (float) (ahrs->currentUpdate - ahrs->lastUpdate) / SystemCoreClock;
//    ahrs->MahonyAHRSUpdate(mpu->data.wx - params->raw.GyroXOffset * dt, mpu->data.wy - params->raw.GyroYOffset * dt,
//                           mpu->data.wz - params->raw.GyroZOffset * dt,
//                           true, mpu->data.ax, mpu->data.ay, mpu->data.az,
//                           true, mpu->data.mx, mpu->data.my, mpu->data.mz,
//                           false, 0.0f);
    ahrs->MadgwickQuaternionUpdate(mpu->data.wx - params->raw.GyroXOffset * dt,
                                   mpu->data.wy - params->raw.GyroYOffset * dt,
                                   mpu->data.wz - params->raw.GyroZOffset * dt,
                                   mpu->data.ax, mpu->data.ay, mpu->data.az,
                                   mpu->data.mx, mpu->data.my, mpu->data.mz);

    ahrs->UpdateRPY();

    if (!ahrs->healthy) {
      if ((lastPitch - ahrs->pitch) < 0.001) {
        ahrs->healthy = true;
      }
      lastPitch = ahrs->pitch;
    }

    ahrs->lastUpdate = ahrs->currentUpdate;

    osDelay(1);
  }
}

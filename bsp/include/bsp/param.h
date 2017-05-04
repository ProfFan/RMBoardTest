//
// Created by Fan Jiang on 2017/5/3.
//

#ifndef CHASSIS_PARAM_H
#define CHASSIS_PARAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx_hal.h>

typedef struct {
  uint16_t checksum;
  float GyroXOffset;
  float GyroYOffset;
  float GyroZOffset;

  float MagXOffset;
  float MagYOffset;
  float MagZOffset;

  float MagXRatio;
  float MagYRatio;
  float MagZRatio;

  float PIDPitchKp, PIDPitchKi, PIDPitchKd;
  float PIDPitchKpV, PIDPitchKdV;

  float PIDYawKp, PIDYawKi, PIDYawKd;
  float PIDYawKpV, PIDYawKdV;

  uint16_t guard;
} __attribute__((aligned(2), packed)) RawParams;

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

class ParamStore {
public:

  RawParams raw;

  ParamStore();

  void ReadParams();

  void SaveParams();

  void LoadPresets();

private:
};

extern ParamStore *params;

#endif

#endif //CHASSIS_PARAM_H

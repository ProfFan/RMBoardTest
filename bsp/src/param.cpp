//
// Created by Fan Jiang on 2017/5/3.
//
#include <bsp/param.h>
#include "bsp/param.h"

extern "C" {
#include "bsp/eeprom.h"
}

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

uint16_t VirtAddVarTab[NB_OF_VAR] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};

ParamStore *params = new ParamStore;

void ParamStorageSanityCheck() {
  BUILD_BUG_ON(sizeof(RawParams) / 2 > sizeof(VirtAddVarTab));
}

ParamStore::ParamStore() {
  ReadParams();
}

void ParamStore::ReadParams() {
  uint16_t i;
  uint16_t *ptr;

  if (EE_Init() != HAL_OK)
    return;

  ptr = &raw.checksum;

  for (i = 0; i < sizeof(raw) / 2; i++) {
    EE_ReadVariable(i, ptr);
    ptr++;
  }

  if (raw.checksum != 0xBEEF) {
    LoadPresets();
  }
}

void ParamStore::SaveParams() {
  uint16_t i;
  uint16_t *ptr = &raw.checksum;

  HAL_FLASH_Unlock();

  EE_Init();

  for (i = 0; i < sizeof(raw) / 2; i++) {
    EE_WriteVariable(i, (uint16_t) *ptr);
    ptr++;
  }

  HAL_FLASH_Lock();
}

void ParamStore::LoadPresets() {
  raw.checksum = 0xBEEF;
  raw.guard = 0xDEAD;
  raw.GyroXOffset = -0.58247298f;
  raw.GyroYOffset = -0.57712272f;
  raw.GyroZOffset = -2.64872325f;

  raw.MagXOffset = 7.15559368;
  raw.MagYOffset = -196.75254773;
  raw.MagZOffset = 282.78573528;

  raw.MagEllipsoidAInv[0][0] = 7.78583592e-03;
  raw.MagEllipsoidAInv[0][1] = -3.65206706e-04;
  raw.MagEllipsoidAInv[0][2] = 5.52328167e-04;

  raw.MagEllipsoidAInv[1][0] = -3.65206706e-04;
  raw.MagEllipsoidAInv[1][1] = 7.21988342e-03;
  raw.MagEllipsoidAInv[1][2] = 3.79955892e-06;

  raw.MagEllipsoidAInv[2][0] = 5.52328167e-04;
  raw.MagEllipsoidAInv[2][1] = 3.79955892e-06;
  raw.MagEllipsoidAInv[2][2] = 7.74409334e-03;

  raw.PIDPitchKp = 0;
  raw.PIDPitchKi = 0;
  raw.PIDPitchKd = 0;
  raw.PIDPitchKpV = 0;
  raw.PIDPitchKdV = 0;

  raw.PIDYawKp = 0;
  raw.PIDYawKi = 0;
  raw.PIDYawKd = 0;
  raw.PIDYawKpV = 0;
  raw.PIDYawKdV = 0;

  SaveParams();
}
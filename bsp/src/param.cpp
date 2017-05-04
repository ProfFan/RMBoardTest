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
  BUILD_BUG_ON(sizeof(RawParams) > sizeof(VirtAddVarTab));
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
  raw.GyroXOffset = 1.42f;
  raw.GyroYOffset = 1.30f;
  raw.GyroZOffset = -0.3428f;

  raw.MagXOffset = 0;
  raw.MagYOffset = 0;
  raw.MagZOffset = 0;

  raw.MagXRatio = 0;
  raw.MagYRatio = 0;
  raw.MagZRatio = 0;

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
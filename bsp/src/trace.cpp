//
// Created by Fan Jiang on 2017/6/11.
//

#include "bsp/trace.h"
#include "stm32f4xx_hal.h"

extern "C" int ITM_Send(uint32_t port, uint32_t ch) {
  uint32_t tickstart = HAL_GetTick();

  while (ITM->PORT[port].u32 == 0) {
    if (((HAL_GetTick() - tickstart) >= 2)) {
      return HAL_TIMEOUT;
    }
  }
  ITM->PORT[port].u8 = (uint8_t) ch;
  return 0;
}

Trace::Trace() {

}

int Trace::sendChar(uint32_t port, uint32_t ch) {
  uint32_t tickstart = HAL_GetTick();

  while (ITM->PORT[port].u32 == 0) {
    if (((HAL_GetTick() - tickstart) >= 2)) {
      return HAL_TIMEOUT;
    }
  }
  ITM->PORT[port].u8 = (uint8_t) ch;
  return 0;
}
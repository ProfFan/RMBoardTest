//
// Created by Fan Jiang on 2017/5/2.
//

#ifndef CHASSIS_HPTIMER_H
#define CHASSIS_HPTIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Call at least every 2^32 cycles (every 59.6 seconds @ 72 MHz).
uint64_t GetCycleCount64();

uint64_t GetCycleCount64_NoISR();

#ifdef __cplusplus
}
#endif

#endif //CHASSIS_HPTIMER_H

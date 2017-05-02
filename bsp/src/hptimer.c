//
// Created by Fan Jiang on 2017/5/2.
//
#include "stm32f4xx_hal.h"
#include "bsp/hptimer.h"

volatile uint64_t last_cycle_count_64 = 0;

// Call at least every 2^32 cycles (every 59.6 seconds @ 72 MHz).
uint64_t GetCycleCount64() {
  uint32_t primask;
  asm volatile ("mrs %0, PRIMASK" : "=r"(primask));
  asm volatile ("cpsid i");  // Disable interrupts.
  int64_t r = last_cycle_count_64;
  r += DWT->CYCCNT - (uint32_t)(r);
  last_cycle_count_64 = r;
  asm volatile ("msr PRIMASK, %0" : : "r"(primask));  // Restore interrupts.
  return r;
}

uint64_t GetCycleCount64_NoISR() {
  last_cycle_count_64 += DWT->CYCCNT - (uint32_t)(last_cycle_count_64);
  return last_cycle_count_64;
}
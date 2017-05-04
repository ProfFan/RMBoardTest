//
// Created by Fan Jiang on 2017/5/1.
//

#ifndef CHASSIS_SBUS_H
#define CHASSIS_SBUS_H

#include "FreeRTOS.h"
#include "usart.h"

#define SBUS_DMA_BUFFER_SIZE 64

typedef struct {
  uint16_t CH1:11;  //each ch value from -364 -- +364
  uint16_t CH2:11;
  uint16_t CH3:11;
  uint16_t CH4:11;

  uint8_t SW1:2;  //3 value
  uint8_t SW2:2;

  struct {
    int16_t X;
    int16_t Y;
    int16_t Z;

    uint8_t L;
    uint8_t R;
  } __attribute__((packed)) Mouse;

  union {
    uint16_t key_code;
    struct {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    } __attribute__((packed)) bits;
/**********************************************************************************
   * KEY_CHAN:15  14  13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V   C   X    Z    G    F    R   E   Q   CTRL  SHIFT D   A   S   W
************************************************************************************/
  } Key;
  uint8_t Resv[2];
} __attribute__((packed)) RC_Type;

extern RC_Type hsbus1;

extern uint8_t sbus_buffer[SBUS_DMA_BUFFER_SIZE];

extern volatile int sbusStatus;

void SBUS_Reset_DMA_Rx(UART_HandleTypeDef *huart);

void StartSBUSTask(void const *argument);

#endif //CHASSIS_SBUS_H
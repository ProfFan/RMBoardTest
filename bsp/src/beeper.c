//
// Created by Fan Jiang on 2017/4/27.
//

#include "bsp/beeper.h"

osThreadId beepTaskHandle;

QueueHandle_t beepQueue;

const uint16_t note_arr_tab[] = {
    //382, 341, 303, 286, 255, 227, 202, // Low C
    191, 170, 152, 143, 128, 114, 101, // Mid C
    // 96, 85, 76, 72, 64, 57, 51,        // Hi  C
    1 // Silence
};
const int note_count = sizeof(note_arr_tab) / sizeof(*note_arr_tab);

Note_t current;

#define MAX_DURATION 2000

void StartBeepTask(void const *argument) {
  beepQueue = xQueueCreate(10, sizeof(Note_t));
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* Infinite loop */
  for (;;) {

    if (beepQueue != 0) {
      if (xQueueReceive(beepQueue, &current, (TickType_t) 1000)) {
        if ((current.pitch >= note_count) || (current.pitch < 0)) current.pitch = note_count - 1;
        if (current.duration >= MAX_DURATION) current.duration = MAX_DURATION;
        if (current.duration < 0) current.duration = 0;
        __HAL_TIM_SET_AUTORELOAD(&htim3, note_arr_tab[current.pitch]);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, note_arr_tab[current.pitch] / 10);
        osDelay(current.duration);
        __HAL_TIM_SET_AUTORELOAD(&htim3, 1);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        //vTaskSuspend(beepTaskHandle);
      }
    } else {
      osDelay(300);
    }

  }
}

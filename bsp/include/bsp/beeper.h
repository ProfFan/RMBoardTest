//
// Created by Fan Jiang on 2017/4/27.
//

#ifndef CHASSIS_BEEPER_H
#define CHASSIS_BEEPER_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "tim.h"

void StartBeepTask(void const *argument);

typedef struct {
  unsigned short pitch;
  unsigned short duration;
} Note_t;

#endif //CHASSIS_BEEPER_H

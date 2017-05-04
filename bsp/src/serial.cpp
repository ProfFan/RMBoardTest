//
// Created by Fan Jiang on 2017/5/1.
//
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "bsp/serial.h"
#include "bsp/beeper.h"
#include "bsp/sbus.h"
#include "bsp/hptimer.h"
}

#include <bsp/param.h>
#include "bsp/imu.h"
#include "bsp/param.h"

osThreadId serialTaskHandle;

QueueHandle_t serialQueue;

extern QueueHandle_t beepQueue;

Note_t currNote;

uint8_t print_buf[256];

//float mx_sum = 0, my_sum = 0, mz_sum = 0;

void StartSerialTask(void const *argument) {
  int size;

  serialQueue = xQueueCreate(10, sizeof(uint8_t));
  currNote.duration = 200;
  currNote.pitch = 0;

  params->LoadPresets();
  /* Infinite loop */
  for (;;) {
    currNote.pitch = (short) ((currNote.pitch > 6) ? 0 : (currNote.pitch + 1));

    size = sprintf((char *) print_buf, "\033[1;1H\033[KRC: %d,RC_CH1: %d, RC_CH2: %d, RC_CH3: %d, RC_CH4: %d\r\n",
                   HAL_UART_GetState(&huart1), hsbus1.CH1 - 1024, hsbus1.CH2 - 1024, hsbus1.CH3 - 1024,
                   hsbus1.CH4 - 1024);
    CDC_Try_Send(print_buf, size, 1);

    if (ahrs->ready) {

//      mx_sum += mpu->data.wx;
//      my_sum += mpu->data.wy;
//      mz_sum += mpu->data.wz;

      //float time = HAL_GetTick()/1000.0f;
//      size = sprintf((char *) print_buf, "\033[KGx: %f, Gy: %f, Gz: %f, Tim: %f, Tmp: %f\r\n",
//                     mx_sum/time, my_sum/time, mz_sum/time, time, mpu->data.temp);

      size = sprintf((char *) print_buf, "\033[KP: %f, R: %f, Y: %f, Kp: %f, Tmp: %f\r\n",
                     ahrs->pitch * 180.0f / M_PI, ahrs->roll * 180.0f / M_PI, ahrs->yaw * 180.0f / M_PI, ahrs->beta,
                     mpu->data.temp);

      CDC_Try_Send(print_buf, size, 1);

    }

    size = sprintf((char *) print_buf, "\033[KParam: %f\r\n",
                   params->raw.GyroYOffset);

    CDC_Try_Send(print_buf, size, 1);

//    if(USBD_OK == CDC_Transmit_FS(print_buf, size)){
//
//    }

    osDelay(10);

  }
}

extern "C" void CDC_Receive_Hook(USBD_HandleTypeDef *husb) {
//  static int error = 1;
//  float GyroMeasError = M_PI * (error /
//                                180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
//  ahrs->beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
//  error = error + 1;
}
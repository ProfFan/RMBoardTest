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
}

#include "bsp/imu.h"

osThreadId serialTaskHandle;

QueueHandle_t serialQueue;

extern QueueHandle_t beepQueue;

Note_t currNote;

uint8_t print_buf[256];

extern int resp;

float yaw, pitch, roll;
void StartSerialTask(void const *argument) {
  serialQueue = xQueueCreate(10, sizeof(uint8_t));
  currNote.duration = 200;
  currNote.pitch = 0;
  /* Infinite loop */
  for (;;) {
    currNote.pitch=(short)((currNote.pitch>6)?0:(currNote.pitch+1));

    int size;
    size = sprintf((char*)print_buf, "\033[H\033[KRC: %d,RC_CH1: %d, RC_CH2: %d, RC_CH3: %d, RC_CH4: %d\r\n",
                   HAL_UART_GetState(&huart1), hsbus1.CH1 - 1024, hsbus1.CH2 - 1024, hsbus1.CH3 - 1024, hsbus1.CH4 - 1024);
    CDC_Transmit_FS(print_buf, size);

    float q[4];
    memcpy(q, ahrs->q, sizeof(q));
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / M_PI;
    yaw   *= 180.0f / M_PI;
    roll  *= 180.0f / M_PI;

    size = sprintf((char*)print_buf, "\033[KGx: %f, Gy: %f, Gz: %f, Mx: %f, Ax: %f\r\n",
                   pitch, roll, yaw, mpu->data.mx, mpu->data.ax);
    CDC_Transmit_FS(print_buf, size);

//    if(USBD_OK == CDC_Transmit_FS(print_buf, size)){
//
//    }

    osDelay(10);

  }
}

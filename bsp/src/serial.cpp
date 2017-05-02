//
// Created by Fan Jiang on 2017/5/1.
//
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "bsp/serial.h"
#include "bsp/beeper.h"
#include "bsp/sbus.h"
}

#include <bsp/mpu6500.h>
#include "bsp/mpu6500.h"

osThreadId serialTaskHandle;

QueueHandle_t serialQueue;

extern QueueHandle_t beepQueue;

Note_t currNote;

uint8_t print_buf[256];

MPU6500 *mpu = new MPU6500(&hspi5);

int resp;

void StartSerialTask(void const *argument) {
  serialQueue = xQueueCreate(10, sizeof(uint8_t));
  currNote.duration = 200;
  currNote.pitch = 0;
  resp = mpu->initialize();
  /* Infinite loop */
  for (;;) {
    mpu->readRawData();
    currNote.pitch=(short)((currNote.pitch>6)?0:(currNote.pitch+1));

    int size;
    size = sprintf((char*)print_buf, "\033[H\033[KRC: %d,RC_CH1: %d, RC_CH2: %d, RC_CH3: %d, RC_CH4: %d\r\n",
                   HAL_UART_GetState(&huart1), hsbus1.CH1 - 1024, hsbus1.CH2 - 1024, hsbus1.CH3 - 1024, hsbus1.CH4 - 1024);
    CDC_Transmit_FS(print_buf, size);

    size = sprintf((char*)print_buf, "\033[KResp: %d, ID: 0x%2x Gx: %f, Gy: %f, Gz: %f, Temp: %f\r\n",
                   resp, mpu->readReg(MPU6500_WHO_AM_I), mpu->data.ax, mpu->data.ay, mpu->data.az, mpu->data.temp);
    CDC_Transmit_FS(print_buf, size);

//    if(USBD_OK == CDC_Transmit_FS(print_buf, size)){
//
//    }

    osDelay(10);

  }
}

//
// Created by Fan Jiang on 2017/5/1.
//
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "bsp/serial.h"
#include "bsp/beeper.h"
#include <stdio.h>

osThreadId serialTaskHandle;

QueueHandle_t serialQueue;

extern QueueHandle_t beepQueue;

// extern RC_Type SBUSData;

Note_t currNote;

uint8_t print_buf[256];
void StartSerialTask(void const *argument) {
  serialQueue = xQueueCreate(10, sizeof(uint8_t));
  currNote.duration = 200;
  currNote.pitch = 0;
  /* Infinite loop */
  for (;;) {

    currNote.pitch=(short)((currNote.pitch>6)?0:(currNote.pitch+1));
    //uint8_t str[] = "\033[1;31mHello!\033[0m\r\n";
    int size;
//    size = sprintf(print_buf, "RC_CH1: %d, RC_CH2: %d, RC_CH3: %d, RC_CH4: %d, BUSY: %u\r\n",
//                   (int)SBUSData.CH1, (int)SBUSData.CH2, (int)SBUSData.CH3, (int)SBUSData.CH4, (unsigned int)SBUS_Rx_GetLengthReceived());
    size = sprintf(print_buf, "Hello!\r\n");
    if(USBD_OK == CDC_Transmit_FS(print_buf, size)){
      xQueueSend(beepQueue, &currNote, 0);
    }

    osDelay(300);

  }
}

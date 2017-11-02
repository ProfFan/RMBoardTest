//
// Created by Fan Jiang on 2017/5/1.
//
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "cmsis_os.h"
#include "bsp/beeper.h"
}

#include "bsp/serial.h"
#include <bsp/param.h>
#include <bsp/sbus.h>
#include "bsp/imu.h"
#include "bsp/param.h"
#include "bsp/gimbal.h"
#include "bsp/chassis.h"

#include "ros.h"

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

osThreadId serialTaskHandle;

QueueHandle_t serialQueue;

extern QueueHandle_t beepQueue;

Note_t currNote;

uint8_t print_buf[256];

uint8_t usbTxBuf[1024];

USBSerial *USBSerial1;

void StartSerialTask(void const *argument) {
  int size;

  serialQueue = xQueueCreate(10, sizeof(uint8_t));
  currNote.duration = 200;
  currNote.pitch = 0;

  //params->LoadPresets();

  USBSerial1 = new USBSerial(&hUsbDeviceFS);

  ros::NodeHandle nh;

  std_msgs::String str_msg;
  ros::Publisher chatter("chatter", &str_msg);

  std_msgs::Float32 encLF_msg;
  ros::Publisher pub_encLF("encoder/leftfront", &encLF_msg);

  char hello[13] = "hello world!";

  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(pub_encLF);
  /* Infinite loop */
  for (;;) {

    str_msg.data = hello;
    chatter.publish(&str_msg);

    // AHRS Attitude
    if (ahrs->healthy) {
      encLF_msg.data = chassis->motorLF.angle;
      pub_encLF.publish(&encLF_msg);
    }

    nh.spinOnce();

    USBSerial1->txService();

    osDelay(10);

  }
}

extern "C" void CDC_Receive_Hook(USBD_HandleTypeDef *husb, uint8_t *pbuf, uint32_t *Len) {

  if (USBSerial1 != nullptr) {
    USBSerial1->rxISR(pbuf, *Len);
  };

}

int USBSerial::write(const uint8_t *buffer, int length) {
  for (int i = 0; i < length; i++) {
    this->txBuffer.put(buffer[i]);
  }
  return length;
}

int USBSerial::read(uint8_t *buffer, int length) {
  if (this->rxBuffer.available() >= length) {
    for (int i = 0; i < length; i++) {
      buffer[i] = this->rxBuffer.get();
    }
    return length;
  }
  return -1;
}

int USBSerial::readable() {
  return this->rxBuffer.available();
}

void USBSerial::rxISR(const uint8_t *buffer, int length) {
  for (int i = 0; i < length; i++) {
    this->rxBuffer.put(buffer[i]);
  }
}

void USBSerial::txService() {
  int i = 0;
  while (this->txBuffer.available()) {
    usbTxBuf[i] = this->txBuffer.get();
    i = i + 1;
  }
  if (i)
    CDC_Try_Send(usbTxBuf, i, 1);
}

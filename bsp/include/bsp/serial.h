//
// Created by Fan Jiang on 2017/5/1.
//

#ifndef CHASSIS_SERIAL_H
#define CHASSIS_SERIAL_H

#include "usart.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#ifdef __cplusplus

#include "BufferedSerial/Buffer/Buffer.h"

extern "C" {
#endif

void StartSerialTask(void const *argument);

#ifdef __cplusplus
};

class Serial {
public:
  virtual int write(const uint8_t *buffer, int length) = 0;
  virtual int read(uint8_t *buffer, int length) = 0;
  virtual int readable() = 0;
};

class USBSerial: public Serial {
public:
  explicit USBSerial(USBD_HandleTypeDef *_hcdc): rxBuffer(4096), txBuffer(4096){
    hcdc = _hcdc;
  }

  ~USBSerial() = default;

  int write(const uint8_t *buffer, int length) override;

  int read(uint8_t *buffer, int length) override;

  char read();

  int readable() override;

  void rxISR(const uint8_t *buffer, uint32_t length);

  void txService();
private:
  USBD_HandleTypeDef *hcdc;

protected:
  Buffer <char> rxBuffer;
  Buffer <char> txBuffer;
};

extern USBSerial* USBSerial1;

#endif
#endif //CHASSIS_SERIAL_H
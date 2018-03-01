/*
 * MbedHardware
 *
 *  Created on: Aug 17, 2011
 *      Author: nucho
 */

#ifndef ROS_MBED_HARDWARE_H_
#define ROS_MBED_HARDWARE_H_

#include "bsp/serial.h"

class STM32Hardware {
  public:
    STM32Hardware() {
        baud_ = 921600;
        //t.start();
    }

    void setBaud(long baud){
      this->baud_= baud;
    }

    int getBaud(){return baud_;}

    void init(){

    }

    int read(){
      //uint8_t chr;
      if(USBSerial1->readable()){
        return USBSerial1->read();
      } else {
        return -1;
      }
    };

    void write(uint8_t* data, int length) {
      USBSerial1->write(data, length);
    }

    unsigned long time(){return HAL_GetTick();}

protected:
    long baud_;
    //Timer t;
};


#endif /* ROS_MBED_HARDWARE_H_ */

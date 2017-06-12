//
// Created by Fan Jiang on 2017/6/11.
//

#ifndef CHASSIS_TRACE_H
#define CHASSIS_TRACE_H

#include "main.h"
#include "stm32f4xx.h"
#include "core_cm4.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

int ITM_Send(uint32_t port, uint32_t ch);

#ifdef __cplusplus
}
#endif

class Trace {
public:

  Trace();

  int sendChar(uint32_t port, uint32_t ch);
private:

};

extern Trace *trace;

#endif //CHASSIS_TRACE_H

/****************************************************************************
 *
 *  Copyright (c) 2017, Michael Becker (michael.f.becker@gmail.com)
 *
 *  This file is part of the FreeRTOS C++ Wrappers project.
 *  
 *  Source Code:
 *  https://github.com/michaelbecker/freertos-addons
 *
 *  Project Page:
 *  http://michaelbecker.github.io/freertos-addons/
 *
 *  On-line Documentation:
 *  http://michaelbecker.github.io/freertos-addons/docs/html/index.html
 *
 *  The FreeRTOS C++ Wrappers project is free software: you can redistribute
 *  it and/or modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 2
 *  of the License, or (at your option) any later version.
 *
 *  The FreeRTOS C++ Wrappers project is distributed in the hope that it will
 *  be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the FreeRTOS C++ Wrappers project.
 *  If not, see <http://www.gnu.org/licenses/>.
 *
 *  NOTE: A modification to the GPL is included to allow you to distribute a
 *  combined work that includes FreeRTOS C++ Wrappers project
 *  without being obliged to provide the source
 *  code for proprietary components.
 * 
 *  The FreeRTOS C++ Wrappers project GPL exception text follows:
 * 
 *  Any FreeRTOS C++ Wrapper *source code*, whether modified or in it's 
 *  original release form, or whether in whole or in part, can only be 
 *  distributed by you under the terms of the GNU General Public License plus
 *  this exception.  An independent module is a module which is not derived
 *  from or based on FreeRTOS C++ Wrappers project.
 * 
 *  Clause 1:
 * 
 *  Linking FreeRTOS C++ Wrappers project  with other modules is making a 
 *  combined work based on FreeRTOS C++ Wrappers project. Thus, the terms 
 *  and conditions of the GNU General Public License V2 cover the
 *  whole combination.
 * 
 *  As a special exception, the copyright holders of FreeRTOS C++ Wrappers 
 *  project give you permission to link FreeRTOS C++ Wrappers project with 
 *  independent modules to produce a statically linked executable, regardless
 *  of the license terms of these independent modules, and to copy and 
 *  distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the 
 *  terms and conditions of the license of that module.  An independent 
 *  module is a module which is not derived from or based on FreeRTOS C++ 
 *  Wrappers project.
 * 
 *  Clause 2:
 * 
 *  FreeRTOS C++ Wrappers project may not be used for any competitive or 
 *  comparative purpose, including the publication of any form of run time 
 *  or compile time metric, without the express permission of the copyright 
 *  holder(s) (this is the norm within the industry and is intended to ensure
 *  information accuracy).
 *  
 ***************************************************************************/
#include "os/tasklet.hpp"


using namespace cpp_freertos;


Tasklet::Tasklet()
{
    DtorLock = xSemaphoreCreateBinary();

    if (DtorLock == NULL) {
#ifndef CPP_FREERTOS_NO_EXCEPTIONS
        throw TaskletCreateException("Create DtorLock Failed");
#else
        configASSERT(!"Tasklet Constructor Failed");
#endif
    }

    xSemaphoreGive(DtorLock);
}


Tasklet::~Tasklet()
{
}


void Tasklet::CheckForSafeDelete()
{
    xSemaphoreTake(DtorLock, portMAX_DELAY);
    vSemaphoreDelete(DtorLock);
}


void Tasklet::TaskletAdapterFunction(void *reference, uint32_t parameter)
{
    Tasklet *tasklet = static_cast<Tasklet *>(reference);
    tasklet->Run(parameter);
    xSemaphoreGive(tasklet->DtorLock);
}


bool Tasklet::Schedule( uint32_t parameter,
                        TickType_t CmdTimeout)
{
    BaseType_t rc;

    xSemaphoreTake(DtorLock, portMAX_DELAY);

    rc = xTimerPendFunctionCall(TaskletAdapterFunction,
                                this,
                                parameter,
                                CmdTimeout);

    if (rc == pdPASS) {
        return true;
    }
    else {
        xSemaphoreGive(DtorLock);
        return false;
    }
}


bool Tasklet::ScheduleFromISR(  uint32_t parameter,
                                BaseType_t *pxHigherPriorityTaskWoken)
{
    BaseType_t rc;

    rc = xSemaphoreTakeFromISR(DtorLock, pxHigherPriorityTaskWoken);

    if (rc != pdTRUE) {
        return false;
    }
    
    rc = xTimerPendFunctionCallFromISR( TaskletAdapterFunction,
                                        this,
                                        parameter,
                                        pxHigherPriorityTaskWoken);

    if (rc == pdPASS) {
        return true;
    }
    else {
        xSemaphoreGive(DtorLock);
        return false;
    }
}


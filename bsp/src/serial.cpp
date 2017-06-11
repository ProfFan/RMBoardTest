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
#include <bsp/sbus.h>
#include "bsp/imu.h"
#include "bsp/param.h"
#include "bsp/gimbal.h"
#include "bsp/chassis.h"

#include "mavlink/standard/mavlink.h"

osThreadId serialTaskHandle;

QueueHandle_t serialQueue;

extern QueueHandle_t beepQueue;

Note_t currNote;

uint8_t print_buf[256];

mavlink_system_t mavlinkSystem{
    .sysid = 20,
    .compid = MAV_COMP_ID_ALL
};

mavlink_message_t mavlinkMessage;
uint8_t mavlinkTxBuffer[MAVLINK_MAX_PACKET_LEN];

static int packet_drops = 0;
static int mode = MAVLINK_PARSE_STATE_UNINIT; /* Defined in mavlink_types.h, which is included by mavlink.h */

// int messageCount;
// float mx_sum = 0, my_sum = 0, mz_sum = 0;

void StartSerialTask(void const *argument) {
  int size;

  serialQueue = xQueueCreate(10, sizeof(uint8_t));
  currNote.duration = 200;
  currNote.pitch = 0;

  params->LoadPresets();

  /* Infinite loop */
  for (;;) {

    // System Status
    if(HAL_GetTick()%1000) { // Every second
      if (!ahrs->healthy) {
        mavlink_msg_heartbeat_pack(mavlinkSystem.sysid, mavlinkSystem.compid, &mavlinkMessage, MAV_TYPE_GROUND_ROVER,
                                   MAV_AUTOPILOT_GENERIC, MAV_MODE_PREFLIGHT, 0, MAV_STATE_BOOT);

        CDC_Try_Send(mavlinkTxBuffer, mavlink_msg_to_send_buffer(mavlinkTxBuffer, &mavlinkMessage), 1);
      } else {
        mavlink_msg_heartbeat_pack(mavlinkSystem.sysid, mavlinkSystem.compid, &mavlinkMessage, MAV_TYPE_GROUND_ROVER,
                                   MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE_BOOT);

        CDC_Try_Send(mavlinkTxBuffer, mavlink_msg_to_send_buffer(mavlinkTxBuffer, &mavlinkMessage), 1);
      }
    }

    // currNote.pitch = (short) ((currNote.pitch > 6) ? 0 : (currNote.pitch + 1));

    // Raw RC Values
    mavlink_msg_rc_channels_pack(mavlinkSystem.sysid, mavlinkSystem.compid, &mavlinkMessage, HAL_GetTick(), 6,
                                 hsbus1.CH1, hsbus1.CH2, hsbus1.CH3, hsbus1.CH4, hsbus1.SW1, hsbus1.SW2,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    );

    CDC_Try_Send(mavlinkTxBuffer, mavlink_msg_to_send_buffer(mavlinkTxBuffer, &mavlinkMessage), 1);

    // AHRS Attitude
    if (ahrs->healthy) {
//      mx_sum += mpu->data.wx;
//      my_sum += mpu->data.wy;
//      mz_sum += mpu->data.wz;

      //float time = HAL_GetTick()/1000.0f;
//      size = sprintf((char *) print_buf, "\033[KGx: %f, Gy: %f, Gz: %f, Tim: %f, Tmp: %f\r\n",
//                     mx_sum/time, my_sum/time, mz_sum/time, time, mpu->data.temp);

//      size = sprintf((char *) print_buf, "\033[KP: %f, Y: %f, C: %d, Kp: %d, Tmp: %f\r\n",
//                     ahrs->pitch * 180.0f / M_PI, ahrs->yaw * 180.0f / M_PI, chassis->motorLF.angle,
//                     gimbal->pitch_motor.angle,
//                     mpu->data.temp);
//
//      CDC_Try_Send(print_buf, size, 1);
      mavlink_msg_attitude_pack(mavlinkSystem.sysid, mavlinkSystem.compid, &mavlinkMessage, HAL_GetTick(),
                                           ahrs->roll, ahrs->pitch, ahrs->yaw, 0, 0, 0);

      CDC_Try_Send(mavlinkTxBuffer, mavlink_msg_to_send_buffer(mavlinkTxBuffer, &mavlinkMessage), 1);

      mavlink_msg_attitude_pack(mavlinkSystem.sysid, mavlinkSystem.compid, &mavlinkMessage, HAL_GetTick(),
                                mpu->data.mx, mpu->data.my, mpu->data.mz, 0, 0, 0);

      CDC_Try_Send(mavlinkTxBuffer, mavlink_msg_to_send_buffer(mavlinkTxBuffer, &mavlinkMessage), 1);
    }

//    size = sprintf((char *) print_buf, "\033[KParam: %f\r\n",
//                   params->raw.GyroYOffset);
//
//    CDC_Try_Send(print_buf, size, 1);

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
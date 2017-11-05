//
// Created by Fan Jiang on 2017/5/4.
//

#ifndef CHASSIS_MOTOR_H
#define CHASSIS_MOTOR_H

#define CAN_TxGimbal_ID             0x1FF  //云台发送ID
#define CAN_YAW_FEEDBACK_ID         0x205  //云台Yaw24v
#define CAN_PIT_FEEDBACK_ID         0x206  //云台Yaw24v
#define CAN_POKE_FEEDBACK_ID        0x207
#define CAN_ZGYRO_RST_ID            0x404
#define CAN_ZGYRO_FEEDBACK_MSG_ID   0x401
#define CAN_MotorLF_ID              0x201  //左前
#define CAN_MotorRF_ID              0x202  //右前
#define CAN_MotorLB_ID              0x203  //左后
#define CAN_MotorRB_ID              0x204  //右后
#define CAN_4Moto_Target_Speed_ID   0x046  //
#define CAN_GyroRecev_ID            0x011  //陀螺仪接收
#define CAN_GyroReset_ID            0x012  //陀螺仪复位
#define CAN_EC60_All_ID             0x200  //EC60接收程序
#define CAN_backLeft_EC60_ID        0x203  //ec60
#define CAN_frontLeft_EC60_ID       0x201  //ec60
#define CAN_backRight_EC60_ID       0x202  //ec60
#define CAN_frontRight_EC60_ID      0x204  //ec60

#define CAN_3510MotoAll_ID   0x200
#define CAN_3510Moto1_ID     0x201
#define CAN_3510Moto2_ID     0x202
#define CAN_3510Moto3_ID     0x203
#define CAN_3510Moto4_ID     0x204
#define CAN_DriverPower_ID   0x80

#define CAN_HeartBeat_ID   0x156

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <std_msgs/Float32.h>
#include "bsp/bsp_can.h"

class Motor {
public:

  Motor(uint32_t id, CAN* bus);

  void UpdateSensorData(CanRxMsgTypeDef *message);

  int current = 0;
  int speedRPM = 0;
  int angle = 0;
  int lastAngle = 0;
  int targetCurrent = 0;
  long round = 0;
  long totalAngle = 0;
  int offsetAngle = 0;
private:
  uint32_t canID;
  CAN* bus;
};

#endif

#endif //CHASSIS_MOTOR_H

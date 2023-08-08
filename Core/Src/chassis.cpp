//
// Created by yejia on 2023/8/8.
//

#include "chassis.h"

extern "C" {
#include "../BUPT_RobotTeam_Libraries/MotorLib/dji_boardv2_can.h"
};
#define DRIVE_WHEEL_MAX_SPEED (0.2)
#define WHEEL2CHASSISCENTER_SQUARE (0.31*0.31)
#define WHEEL_FRONT2BACK_DISTANCE (0.41)
#define DRIVE_WHEEL_RADIUS (0.075)
#define DJIV2_BOARD_ID (2)
#define PI (3.1415926535)


void chassis::chassis_init(CAN_HandleTypeDef *hcan) {
    for (int i = 0; i < 4; i++)
    {
        DJIBoard_MotorOn(hcan,DJIV2_BOARD_ID,i+1);  // 使能电机
        DJIBoard_VelCfg(hcan,DJIV2_BOARD_ID,i+1);  // 调整为速度环控制模式
    }
}

void chassis::chassis_move(CAN_HandleTypeDef *hcan, SerialVelMsgTypeDef msg) {
    double target_speed=msg.raw_msg[0],target_omega=msg.raw_msg[5];

    if (target_speed > DRIVE_WHEEL_MAX_SPEED)
    {
        target_speed = DRIVE_WHEEL_MAX_SPEED;
    }
    else if (target_speed < -DRIVE_WHEEL_MAX_SPEED)
    {
        target_speed = -DRIVE_WHEEL_MAX_SPEED;
    }
    //限制速度大小

    float driveWheelSpeed[4];
    driveWheelSpeed[0]=target_speed  // 线速度解算
                       - target_omega * WHEEL2CHASSISCENTER_SQUARE / WHEEL_FRONT2BACK_DISTANCE * 2 ; // 角速度解算
    driveWheelSpeed[1]=target_speed  // 线速度解算
                       - target_omega * WHEEL2CHASSISCENTER_SQUARE / WHEEL_FRONT2BACK_DISTANCE * 2 ; // 角速度解算
    driveWheelSpeed[2]=target_speed  // 线速度解算
                       + target_omega * WHEEL2CHASSISCENTER_SQUARE / WHEEL_FRONT2BACK_DISTANCE * 2 ; // 角速度解算
    driveWheelSpeed[3]=target_speed  // 线速度解算
                       + target_omega * WHEEL2CHASSISCENTER_SQUARE / WHEEL_FRONT2BACK_DISTANCE * 2 ; // 角速度解算

    // 传入指令
    for (uint8_t i = 0 ; i < 4 ; i++) {
        DJIBoard_VelCtrl(hcan, DJIV2_BOARD_ID, i + 1, driveWheelSpeed[i] / DRIVE_WHEEL_RADIUS * 2 * PI / 1000 * 8192 );
        osDelay(1);
    }
}

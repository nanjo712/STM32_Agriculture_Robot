/*!
 * @author LiNY2 
 * @date 2023/03/31 0031
 */


#ifndef CTRLGO2POINT_H
#define CTRLGO2POINT_H

/**
 * Log default configuration for EasyLogger.
 * NOTE: Must defined before including the <elog.h>
 */
#if !defined(LOG_TAG)
#define LOG_TAG "Go2PT"
#endif
#undef LOG_LVL
#define CHASSIS_LOG_LVL ELOG_LVL_INFO
#if defined(CHASSIS_LOG_LVL)
#define LOG_LVL CHASSIS_LOG_LVL
#endif

#include "base_chassis.h"


#ifdef __cplusplus
extern "C"
{
#endif
///<CStyle-Link inc,func and var begin
#include "../SimpleLib/utils/utils.h"
#include "../SimpleLib/utils/vec.h"
#include "BUPTLib_port.h"


    ///<CStyle-Link func and var end
#ifdef __cplusplus
}
#endif


class Go2Point
{
public:
    explicit Go2Point(BaseChassis *chassisPtr);
    enum Go2PointFlag{
        Reset = 0,
        Running,
        Arrived
    };
    uint8_t disable_yaw_ctrl = {false};
    uint8_t enable_always_yaw_ctrl = {false};
    void SetTarget(Point2D_s target_pos,float target_yaw,float start_spd,float end_spd);
    void Ctrl_exe();
    void ResetStatus();
    void SetyawPID(float kp,float ki,float kd);
    void SetlockPID(float kp,float ki,float kd);
    Go2PointFlag getStatusFlag() const
    {
        return status_flag;
    }
    void setMinSpeed(float minSpeed)
    {
        min_speed = minSpeed;
    }
    void setMaxSpeed(float maxSpeed)
    {
        max_speed = maxSpeed;
    }
    void setAccRatio(float accRatio)
    {
        acc_ratio = accRatio;
    }
    void setDecRatio(float decRatio)
    {
        dec_ratio = decRatio;
    }

protected:
    BaseChassis *chassis_ptr{nullptr};
    float arrived_Circie_Th = {0.007};///< m 到达半径阈值
    float lock_Circie_Th = {0.2};     ///< m 启用原地锁定PID的距离半径阈值
    float yaw_Ctrl_Th = {0.01745};
    PID_s yawPID = {
            .Kp = 3.00,
            .Kd = 2.0,
            .Ki = 0.00,
            .int_max = 10.0,
            .int_duty = 10.0,
            .ctrl_max = 5.0};///< 跑点模式角度环PID
    PID_s lockPID = {
            .Kp = 0.7,
            .Kd = 0.35,
            .Ki = 0.00,
            .int_max = 1.0,
            .int_duty = 5.0,
            .ctrl_max = 1.0
    };///< 跑点模式锁止PID
    Point2D_s start_point{0}; ///< 起始点，应当为小车坐标
    Point2D_s target_point{0}; ///< 终点
    Go2PointFlag status_flag{Reset};
//    uint8_t arrived {false};///< 到达
//    uint8_t enable {false};//< 运行中
    float target_yaw{1.5708}; ///< 目标偏航角
    float start_speed{0}; ///< 起始速度
    float final_speed{0}; ///< 目标速度
    float min_speed{DRIVE_WHEEL_MIN_SPEED}; ///< 最小速度
    float max_speed{DRIVE_WHEEL_MAX_SPEED}; ///< 最大速度
    float acc_ratio{0.2};
    float dec_ratio{0.4};
    float total_distance{0.00};
    float Plan2PointSpeed(PostureStatus_s nowpos,float acc_ratio, float dec_ratio);

};

#endif//CTRLGO2POINT_H

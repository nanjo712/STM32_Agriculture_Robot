//
// Created by yejia on 2023/8/8.
//

#ifndef TEST_CHASSIS_H
#define TEST_CHASSIS_H

extern "C"{
#include "oslib.h"
};


union SerialVelMsgTypeDef
{
    double raw_msg[6];
    uint8_t ui8[48];
};

class chassis {
private:

public:
    void chassis_init(CAN_HandleTypeDef *hcan);
    void chassis_move(CAN_HandleTypeDef *hcan,SerialVelMsgTypeDef msg);
};


#endif //TEST_CHASSIS_H

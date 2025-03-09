#ifndef __CONTROL_LAW_H__
#define __CONTROL_LAW_H__

#include "common.h"

#define MAX_MOTOR_VOLT 12.0
static const float MIN2MOVE_MOTOR_VOLT = MAX_MOTOR_VOLT * 0.2;

void init_motor(motor m);

void control_law_callback(const void *msgin);
void control_motor1(float volt);
void control_motor2(float volt);

#endif // __CONTROL_LAW_H__ 
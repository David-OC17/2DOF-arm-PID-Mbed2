#ifndef __CONTROL_LAW_H__
#define __CONTROL_LAW_H__

#include <std_msgs/msg/float32_multi_array.h>

#include "common.h"

#define MAX_MOTOR_VOLT 12.0
static const float MIN2MOVE_MOTOR_VOLT = MAX_MOTOR_VOLT * 0.2;

extern rcl_subscription_t _control_law_subscriber;
extern rcl_node_t _control_law_node;
extern rcl_timer_t _control_law_timer;

extern std_msgs__msg__Float32MultiArray _control_law_msg;

void init_motor(motor m);
void init_control_law();

void control_law_callback(const void *msgin);
void control_motor1(float volt);
void control_motor2(float volt);

#endif // __CONTROL_LAW_H__ 
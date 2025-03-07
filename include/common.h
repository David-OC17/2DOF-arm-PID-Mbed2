#ifndef __COMMON_H__
#define __COMMON_H__

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define LINK1_LENGTH_MM 120.0 // Axis to axis
#define LINK2_LENGTH_MM 88.0  // Axis to planar end-effector phase

#define ENCODER1_INITAL_POS_DEG 0.0
#define ENCODER2_INITAL_POS_DEG 0.0

#define ERROR_LED_PIN 30

#define MOTOR1_PWM 11
#define MOTOR1_DIR1 25 
#define MOTOR1_DIR2 25
#define MOTOR1_ENCODERA 2
#define MOTOR1_ENCODERB 3

#define MOTOR2_PWM 12
#define MOTOR2_DIR1 20
#define MOTOR2_DIR2 19
#define MOTOR2_ENCODERA 4
#define MOTOR2_ENCODERB 5 

typedef struct {
  uint8_t pwm;
  uint8_t dir1;
  uint8_t dir2;

  uint8_t encoder_a;
  uint8_t encoder_b;

  volatile uint32_t encoder_pos;
} motor;

typedef struct {
  float _pos_x;
  float _pos_y;

  float _vel_x;
  float _vel_y;
} end_efector_state;

// Consider jointN to be the Nth joint starting from the base
typedef struct {
  float _joint1_theta;
  float _joint2_theta;
} joint_state;

extern rclc_executor_t _joint_state_executor;
extern rclc_executor_t _control_law_executor;

extern rclc_support_t _support;
extern rcl_allocator_t _allocator;

extern motor motor1;
extern motor motor2;

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }

#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

void init_micro_ros_nodes();

void error_loop();

#endif // __COMMON_H__
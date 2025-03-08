#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdio.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define LINK1_LENGTH_MM 120.0 // Axis to axis
#define LINK2_LENGTH_MM 88.0  // Axis to planar end-effector phase

#define ENCODER1_INITAL_POS_DEG 0.0
#define ENCODER2_INITAL_POS_DEG 0.0

#define REACH_AGENT_TIMEOUT_MS 1000
#define REACH_AGENT_MAX_ATTEMPTS 150

#define CALIBRATE_KALMAN_ITERS 10
#define LOOP_SPIN_CHECK_TIMEOUT_MS 20
#define CONTROL_LAW_EXECUTOR_SPIN_TIMEOUT_MS 5 * LOOP_SPIN_CHECK_TIMEOUT_MS
#define JOINT_STATE_EXECUTOR_SPIN_TIMEOUT_MS 5 * LOOP_SPIN_CHECK_TIMEOUT_MS

#define MOTOR1_PWM 27
#define MOTOR1_DIR1 26
#define MOTOR1_DIR2 24
#define MOTOR1_ENCODERA 22
#define MOTOR1_ENCODERB 21

#define MOTOR2_PWM 15
#define MOTOR2_DIR1 16
#define MOTOR2_DIR2 17
#define MOTOR2_ENCODERA 19 
#define MOTOR2_ENCODERB 20

#define DEBUG_TX 6
#define DEBUG_RX 7
#define DEBUG_BAUD_RATE 9600

#define DEFAULT_ERROR_MSG "ERROR"
#define DEFAULT_ERROR_WAIT_MS 1000

#define LED_PIN 25

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

#define RCCHECK(fn, file, line)                                                \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop(file, line);                                                  \
    }                                                                          \
  }

#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

void error_loop(char *file, int32_t line);

void init_comms();

void print_debug(const char *msg);

void start_end_blink();

#endif // __COMMON_H__
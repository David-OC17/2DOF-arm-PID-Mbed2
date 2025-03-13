#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdio.h>

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>


#define DEBUG // Comment to enter release mode and disable debug features

#define LINK1_LENGTH_MM 120.0 // Axis to axis
#define LINK2_LENGTH_MM 88.0  // Axis to planar end-effector phase
#define ENCODER_REVS_PER_ROT 470

#define ENCODER1_INITAL_POS_DEG 0.0
#define ENCODER2_INITAL_POS_DEG 0.0

#define REACH_AGENT_TIMEOUT_MS 1000
#define REACH_AGENT_MAX_ATTEMPTS 150

#define CALIBRATE_KALMAN_ITERS 10
#define LOOP_SPIN_CHECK_TIMEOUT_MS 20

#define MOTOR1_PWM 11
#define MOTOR1_DIR1 12
#define MOTOR1_DIR2 13
#define MOTOR1_ENCODERA 14
#define MOTOR1_ENCODERB 15

#define MOTOR2_PWM 28
#define MOTOR2_DIR1 21
#define MOTOR2_DIR2 27
#define MOTOR2_ENCODERA 26
#define MOTOR2_ENCODERB 22

#define LED_PIN 25

#define SPI_PORT spi0
#define SPI_BAUD
#define SPI_CS_PIN 17
#define SPI_CLK_PIN 18
#define SPI_MOSI_PIN 19

#define DEBUG_BUF_LEN sizeof("CHECKPOINT 00")
#define DEBUG_PRINT_SUBSCRIBER_BUF_LEN 128
#define DEBUG_PRINT_PUBLISHER_BUF_LEN 128

static uint8_t debug_out_buf[DEBUG_BUF_LEN] = "CHECKPOINT ";
static uint8_t debug_print_subscriber_buf[DEBUG_PRINT_SUBSCRIBER_BUF_LEN];
static uint8_t debug_print_publisher_buf[DEBUG_PRINT_PUBLISHER_BUF_LEN];
static uint8_t debug_response_buf[DEBUG_PRINT_SUBSCRIBER_BUF_LEN];

typedef struct {
  uint pwm;
  uint dir1;
  uint dir2;

  uint encoder_a;
  uint encoder_b;

  volatile int32_t encoder_pos;
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

extern rclc_executor_t _executor;
extern rclc_support_t _support;
extern rcl_allocator_t _allocator;
extern rcl_node_t _node;

extern rcl_publisher_t _joint_state_publisher;
extern std_msgs__msg__Float32MultiArray _joint_state_msg;

extern rcl_subscription_t _control_law_subscriber;
extern std_msgs__msg__Float32MultiArray _control_law_msg;
extern rmw_message_info_t _control_law_msg_info;

extern motor motor1;
extern motor motor2;

const float clicks2angle(uint16_t clicks);
const int16_t angle2clicks(float angle);

void error_loop();

void init_spi();

void print_debug_checkpoint(uint8_t val);
void print_debug_subscriber();
void print_debug_publisher();

#ifdef DEBUG
#define DEBUG_CHECKPOINT(val) print_debug_checkpoint(val)
#define DEBUG_SUBSCRIBER_PRINT() print_debug_subscriber()
#define DEBUG_PUBLISHER_PRINT() print_debug_publisher()
#else
#define DEBUG_CHECKPOINT(file, line)
#define DEBUG_PRINT(msg)
#define DEBUG_SUBSCRIBER_PRINT()
#define DEBUG_PUBLISHER_PRINT()
#endif

extern void control_law_callback(const void *msgin);

void init_ros_nodes();

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


#endif // __COMMON_H__
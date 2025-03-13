/**
 * Testing routines for the followings individual tasks:
 * - Giving a variable PWM output and moving motors (print to debug)
 * - Counting encoder pulses and publishing position to a topic /encoderTicks
 * - Receiving voltage control input (print to debug)
 * - Stabilize Kalman filter after N iterations (print to debug)
 */

#include <math.h>

#include "common.h"
#include "control_law.h"
#include "joint_state.h"

#include <std_msgs/msg/int32_multi_array.h>

rclc_executor_t _test_executor;
rclc_support_t _test_support;
rcl_allocator_t _test_allocator;
rcl_node_t _test_node;
std_msgs__msg__Int32MultiArray _encoder_pos_msg;
rcl_publisher_t _encoder_pos_publisher;

void TEST_pwm_output();
void TEST_encoder_count_print();
void TEST_voltage_control_ros_subscribe();

/* Test that the GPIO can output a good PWM to both motors */
void TEST_pwm_output() {
  init_spi();
  stdio_init_all();

  // Init motors with their interrupts for encoders
  motor1.pwm = MOTOR1_PWM;
  motor1.dir1 = MOTOR1_DIR1;
  motor1.dir2 = MOTOR1_DIR2;
  motor1.encoder_a = MOTOR1_ENCODERA;
  motor1.encoder_b = MOTOR1_ENCODERB;
  init_motor(motor1);

  motor2.pwm = MOTOR2_PWM;
  motor2.dir1 = MOTOR2_DIR1;
  motor2.dir2 = MOTOR2_DIR2;
  motor2.encoder_a = MOTOR2_ENCODERA;
  motor2.encoder_b = MOTOR2_ENCODERB;
  init_motor(motor2);

  const uint16_t MAX_DUTY_CYCLE = 255;
  bool dir = true;

  // Work at 1/2 duty cycle
  while (1) {
    gpio_put(motor1.dir1, dir);
    gpio_put(motor1.dir2, !dir);
    pwm_set_gpio_level(motor1.pwm, MAX_DUTY_CYCLE / 2);
    pwm_set_gpio_level(motor2.pwm, MAX_DUTY_CYCLE / 2);
    sleep_ms(10);

    gpio_put(motor2.dir1, dir);
    gpio_put(motor2.dir2, !dir);
    pwm_set_gpio_level(motor1.pwm, MAX_DUTY_CYCLE / 2);
    pwm_set_gpio_level(motor2.pwm, MAX_DUTY_CYCLE / 2);
    sleep_ms(10);

    dir = !dir;
  }
}

void TEST_encoder_count_print() {
  init_spi();
  stdio_init_all();

  // Init motors with their interrupts for encoders
  motor1.pwm = MOTOR1_PWM;
  motor1.dir1 = MOTOR1_DIR1;
  motor1.dir2 = MOTOR1_DIR2;
  motor1.encoder_a = MOTOR1_ENCODERA;
  motor1.encoder_b = MOTOR1_ENCODERB;
  init_motor(motor1);

  motor2.pwm = MOTOR2_PWM;
  motor2.dir1 = MOTOR2_DIR1;
  motor2.dir2 = MOTOR2_DIR2;
  motor2.encoder_a = MOTOR2_ENCODERA;
  motor2.encoder_b = MOTOR2_ENCODERB;
  init_motor(motor2);

  init_encoder_interrupt();

  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  rcl_ret_t ret =
      rmw_uros_ping_agent(REACH_AGENT_TIMEOUT_MS, REACH_AGENT_MAX_ATTEMPTS);
  if (ret != RCL_RET_OK) {
    // Unreachable agent
    error_loop();
  }

  _test_allocator = rcl_get_default_allocator();
  rclc_support_init(&_test_support, 0, NULL, &_test_allocator);

  // Create node
  RCCHECK(rclc_node_init_default(&_test_node, "_test_encoder_node", "", &_test_support));

  std_msgs__msg__Int32MultiArray__init(&_encoder_pos_msg);
  _encoder_pos_msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 2);
  _encoder_pos_msg.data.size = 2;
  _encoder_pos_msg.data.capacity = 2;

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &_encoder_pos_publisher, &_test_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "test_encoder_ticks"));

  RCCHECK(rclc_executor_init(&_test_executor, &_test_support.context, 1,
                             &_test_allocator));

  while (1) {
      _encoder_pos_msg.data.data[0] = motor1.encoder_pos;
      _encoder_pos_msg.data.data[1] = motor2.encoder_pos;

      RCSOFTCHECK(rcl_publish(&_encoder_pos_publisher, &_encoder_pos_msg, NULL));

      DEBUG_CHECKPOINT(99);
  }
}

void TEST_voltage_control_ros_subscribe() {
  init_spi();
  stdio_init_all();

  motor1.pwm = MOTOR1_PWM;
  motor1.dir1 = MOTOR1_DIR1;
  motor1.dir2 = MOTOR1_DIR2;
  motor1.encoder_a = MOTOR1_ENCODERA;
  motor1.encoder_b = MOTOR1_ENCODERB;
  init_motor(motor1);

  motor2.pwm = MOTOR2_PWM;
  motor2.dir1 = MOTOR2_DIR1;
  motor2.dir2 = MOTOR2_DIR2;
  motor2.encoder_a = MOTOR2_ENCODERA;
  motor2.encoder_b = MOTOR2_ENCODERB;
  init_motor(motor2);

  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  rcl_ret_t ret =
      rmw_uros_ping_agent(REACH_AGENT_TIMEOUT_MS, REACH_AGENT_MAX_ATTEMPTS);
  if (ret != RCL_RET_OK) {
    // Unreachable agent
    error_loop();
  }

  init_encoder_interrupt();
  init_ros_nodes();

  // For both motors in encoder_a
  static uint8_t iter = 0;
  while (1) {
    if (rcl_take(&_control_law_subscriber, &_control_law_msg,
                 &_control_law_msg_info, NULL) == RCL_RET_OK) {
      control_motor1(_control_law_msg.data.data[0]);
      control_motor2(_control_law_msg.data.data[1]);
    }
    sleep_ms(1000);
  }
}
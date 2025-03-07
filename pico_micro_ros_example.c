#include <stdio.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "common.h"
#include "control_law.h"
#include "joint_state.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  msg.data++;
}

int main() {
  rcl_timer_t timer;
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;

  allocator = rcl_get_default_allocator();

  rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

  if (ret != RCL_RET_OK) {
    // Unreachable agent, exiting program.
    return ret;
  }

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "pico_node", "", &support);

  rclc_publisher_init_default(&publisher, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                              "pico_publisher");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
  return 0;
}

////////////

#define WAIT_ROS_INIT 2000
#define CALIBRATE_KALMAN_ITERS 10
#define LOOP_SPIN_CHECK_TIMEOUT_MS 20
#define CONTROL_LAW_EXECUTOR_SPIN_TIMEOUT_MS 5 * LOOP_SPIN_CHECK_TIMEOUT_MS
#define JOINT_STATE_EXECUTOR_SPIN_TIMEOUT_MS 5 * LOOP_SPIN_CHECK_TIMEOUT_MS

static HardwareSerial debugSerial(1);

#define DEBUG_TX 22
#define DEBUG_RX 21

#define REACH_AGENT_TIMEOUT_MS 1000
#define REACH_AGENT_MAX_ATTEMPTS 150

void setup() {
  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  // Check connection to agent
  rcl_ret_t ret = rmw_uros_ping_agent(REACH_AGENT_TIMEOUT_MS, REACH_AGENT_MAX_ATTEMPTS);
  if (ret != RCL_RET_OK) {
    // Unreachable agent, exiting program.
    return ret;
  }

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

  // Set to default values
  init_joint_state_values(LINK1_LENGTH_MM, LINK2_LENGTH_MM);

  // Init Kalman filter and configure first values
  KF_2DOF_init(&joint_state_kalman_filter, KALMAN_DT_MS, LINK1_LENGTH_MM,
               LINK2_LENGTH_MM, KALMAN_Q_NOISE, KALMAN_R_NOISE);
  init_joint_state_kalman(ENCODER1_INITAL_POS_DEG, ENCODER2_INITAL_POS_DEG);
  calibrate_joint_state_kalman(CALIBRATE_KALMAN_ITERS);

  // Control law config node and timer
  init_control_law();

  // Joint state config node and timer
  init_joint_state();

  // Call executors to start ROS2 nodes
  init_micro_ros_nodes();

  // For both motors in encoder_a
  init_encoder_interrupt();
}

void loop() {
  // Spin _control_law_executor to update control law from subscription and send
  // to drivers
  RCSOFTCHECK(rclc_executor_spin_some(
      &_control_law_executor,
      RCL_MS_TO_NS(CONTROL_LAW_EXECUTOR_SPIN_TIMEOUT_MS)));

  // Spin _joint_state_executor to state (with Kalman belief) and publish
  RCSOFTCHECK(rclc_executor_spin_some(
      &_joint_state_executor,
      RCL_MS_TO_NS(JOINT_STATE_EXECUTOR_SPIN_TIMEOUT_MS)));
}
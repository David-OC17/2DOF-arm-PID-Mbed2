#include <stdio.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "common.h"
#include "control_law.h"
#include "joint_state.h"
/////////////////
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define BUF_LEN 0x100

void setup();
void loop();

int main() {
  setup();
  loop();

  return EXIT_SUCCESS;
}

void setup() {
  init_spi();

  rmw_uros_set_custom_transport(
      true, NULL, pico_serial_transport_open, pico_serial_transport_close,
      pico_serial_transport_write, pico_serial_transport_read);

  rcl_ret_t ret =
      rmw_uros_ping_agent(REACH_AGENT_TIMEOUT_MS, REACH_AGENT_MAX_ATTEMPTS);
  if (ret != RCL_RET_OK) {
    // Unreachable agent, ERR
    error_loop();
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
  // calibrate_joint_state_kalman(CALIBRATE_KALMAN_ITERS);

  init_ros_nodes();

  // For both motors in encoder_a
  init_encoder_interrupt();
}

void loop() {
  static uint8_t iter = 0;
  while (1) {

    // Spin to get new voltages and apply them to drivers
    // RCSOFTCHECK(rclc_executor_spin_some(
    //     &_executor, RCL_MS_TO_NS(LOOP_SPIN_CHECK_TIMEOUT_MS)));

    joint_state_callback();

    // Print to notify update
    DEBUG_CHECKPOINT(iter++);
    sleep_ms(1000);
  }
}
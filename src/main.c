#include "common.h"
#include "control_law.h"
#include "joint_state.h"
#include "testing_routines.h"

void setup();
void loop();

int main() {
  // TEST_pwm_output();
  // TEST_encoder_count_print();
  // TEST_voltage_control_ros_subscribe();

  setup();
  loop();

  return 0;
}

void setup() {
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

  // Set to default values
  init_joint_state_values(LINK1_LENGTH_MM, LINK2_LENGTH_MM);

  // Init Kalman filter and configure first values
  init_joint_state_kalman(ENCODER1_INITAL_POS_DEG, ENCODER2_INITAL_POS_DEG);
  calibrate_joint_state_kalman(CALIBRATE_KALMAN_ITERS);
}

void loop() {
  static uint8_t iter = 0;
  while (1) {
    RCCHECK(rclc_executor_spin_some(&_executor, 10));

    joint_state_callback();
  }
}
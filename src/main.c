#include "common.h"
#include "control_law.h"
#include "joint_state.h"
#include "testing_routines.h"

void setup();
void loop();

const float setpoint = 90;
// static const float Kp = 5, Ki = 0.11, Kd = 0.3;
static const float Kp = 1.7, Ki = 0.8, Kd = 0.1;
float integral = 0.0;
uint64_t lastTime = 0;

// Anti-windup Integral Limits
const float integralMax = 100.0f;
const float integralMin = -100.0f;

const float LOWER_LIMIT = -255.0f;
const float UPPER_LIMIT = 255.0f;

float constrain(float num, float lower, float upper) {
  return (num < lower) ? lower : (num > upper) ? upper : num;
}

void update_PID() {
  // float Kp = 1.7, Ki = 0.8, Kd = 0.001;
  // float Kp = 2.3, Ki = 1.3, Kd = 0.015;
  uint64_t now = get_absolute_time();
  float deltaTime =
      (float)(now - lastTime) / 1000000.0f; // Ensure float division
  lastTime = now;

  float error = setpoint - _approximate_joint_state.joint1_theta;

  integral += error * deltaTime;
  integral = constrain(integral, integralMin, integralMax);

  float output =
      Kp * error + Ki * integral + Kd * _approximate_joint_state.joint1_omega;

  _control_law_msg.data.data[0] = constrain(output, LOWER_LIMIT, UPPER_LIMIT);
}

int main() {
  // TEST_pwm_output();
  // TEST_control_motor();
  // TEST_encoder_count_print();
  // TEST_voltage_control_ros_subscribe();
  // TEST_kf_generic_update();

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

  control_motor1(0);
  control_motor2(0);

  // rmw_uros_set_custom_transport(
  //     true, NULL, pico_serial_transport_open, pico_serial_transport_close,
  //     pico_serial_transport_write, pico_serial_transport_read);

  // rcl_ret_t ret =
  //     rmw_uros_ping_agent(REACH_AGENT_TIMEOUT_MS, REACH_AGENT_MAX_ATTEMPTS);
  // if (ret != RCL_RET_OK) {
  //   // Unreachable agent
  //   error_loop();
  // }

  init_encoder_interrupt();

  // init_ros_nodes();

  // Set to default values
  init_joint_state_values();

  // Init Kalman filter and configure first values
  init_joint_state_kalman(ENCODER1_INITAL_POS_DEG, ENCODER2_INITAL_POS_DEG);

  // TODO change calibration period to being determined by continuity in a GPIO,
  // then start running
  // calibrate_joint_state_kalman(200);
}

void loop() {
  // while (1) {
  //   update_PID();

  //   control_motor1(_control_law_msg.data.data[0]);

  //   joint_state_callback();
  // }

  while (1) {
    uint64_t now = get_absolute_time();
    float deltaTime =
        (float)(now - lastTime) / 1000000.0f; // Ensure float division
    lastTime = now;

    float error = setpoint - _approximate_joint_state.joint2_theta/-2.2;
    // printf("SP: %f, ERROR: %f, THETA: %f", setpoint, error, _approximate_joint_state.joint2_theta/360*470*2);
    printf("error: %f", error/-2.2);
    integral += error * deltaTime;
    integral = constrain(integral, integralMin, integralMax);

    float output =
        Kp * error + Ki * integral + Kd * _approximate_joint_state.joint2_omega;

    _control_law_msg.data.data[1] = constrain(output, LOWER_LIMIT, UPPER_LIMIT);

    control_motor2(_control_law_msg.data.data[1]);

    _measured_joint_state = take_measurement_encoders();

    KF_Update(&_joint2_kalman_filter, _control_law_msg.data.data[1],
              _measured_joint_state.joint2_theta);

    _approximate_joint_state.joint2_theta =
        KF_get_pos_estimate(&_joint2_kalman_filter);
    _approximate_joint_state.joint2_omega =
        KF_get_vel_estimate(&_joint2_kalman_filter);
  }

  // while (1) {
  //   uint64_t now = get_absolute_time();
  //   float deltaTime =
  //       (float)(now - lastTime) / 1000000.0f; // Ensure float division
  //   lastTime = now;

  //   float error = setpoint - _approximate_joint_state.joint1_theta/2.2;
  //   printf("ERROR: %f, THETA: %f", error, _approximate_joint_state.joint1_theta);

  //   integral += error * deltaTime;
  //   integral = constrain(integral, integralMin, integralMax);

  //   float output =
  //       Kp * error + Ki * integral + Kd * _approximate_joint_state.joint1_omega;

  //   _control_law_msg.data.data[0] = constrain(output, LOWER_LIMIT, UPPER_LIMIT);

  //   control_motor1(_control_law_msg.data.data[0]);

  //   _measured_joint_state = take_measurement_encoders();

  //   KF_Update(&_joint1_kalman_filter, _control_law_msg.data.data[0],
  //             _measured_joint_state.joint1_theta);

  //   _approximate_joint_state.joint1_theta =
  //       KF_get_pos_estimate(&_joint1_kalman_filter);
  //   _approximate_joint_state.joint1_omega =
  //       KF_get_vel_estimate(&_joint1_kalman_filter);
  // }
}

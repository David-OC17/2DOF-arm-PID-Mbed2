#include "common.h"
#include "control_law.h"
#include "joint_state.h"
#include "testing_routines.h"

void setup();
void loop();

const float setpoint1 = 90;
const float setpoint2 = 90;
static const float Kp1 = 5, Ki1 = 0.11, Kd1 = 0.3;
static const float Kp2 = 1.7, Ki2 = 0.8, Kd2 = 0.1;
float integral1 = 0.0;
float integral2 = 0.0;
uint64_t lastTime = 0;

// Anti-windup Integral Limits
const float integralMax = 100.0f;
const float integralMin = -100.0f;

const float LOWER_LIMIT = -255.0f;
const float UPPER_LIMIT = 255.0f;

float constrain(float num, float lower, float upper) {
  return (num < lower) ? lower : (num > upper) ? upper : num;
}

int main() {
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

  init_encoder_interrupt();

  // Set to default values
  init_joint_state_values();

  // Init Kalman filter and configure first values
  init_joint_state_kalman(ENCODER1_INITAL_POS_DEG, ENCODER2_INITAL_POS_DEG);
}

void loop() {
  while (1) {
    uint64_t now = get_absolute_time();
    float deltaTime =
        (float)(now - lastTime) / 1000000.0f;
    lastTime = now;

    float error1 = setpoint1 - _approximate_joint_state.joint1_theta/2.2;
    float error2 = setpoint2 - _approximate_joint_state.joint2_theta/-2.2;
    
    integral1 += error1 * deltaTime;
    integral2 += error2 * deltaTime;
    integral1 = constrain(integral1, integralMin, integralMax);
    integral2 = constrain(integral2, integralMin, integralMax);

    float output1 =
        Kp1 * error1 + Ki1 * integral1 + Kd1 * _approximate_joint_state.joint1_omega;

    float output2 =
        Kp2 * error2 + Ki2 * integral2 + Kd2 * _approximate_joint_state.joint2_omega;

    output1 = constrain(output1, LOWER_LIMIT, UPPER_LIMIT);
    output2 = constrain(output2, LOWER_LIMIT, UPPER_LIMIT);
    
    control_motor2(output2);
    control_motor1(output1);

    _measured_joint_state = take_measurement_encoders();

    KF_Update(&_joint1_kalman_filter, _control_law_msg.data.data[0],
              _measured_joint_state.joint1_theta);
    KF_Update(&_joint2_kalman_filter, _control_law_msg.data.data[1],
              _measured_joint_state.joint2_theta);

    _approximate_joint_state.joint2_theta =
        KF_get_pos_estimate(&_joint2_kalman_filter);
    _approximate_joint_state.joint1_theta =
        KF_get_pos_estimate(&_joint1_kalman_filter);
    _approximate_joint_state.joint2_omega =
        KF_get_vel_estimate(&_joint2_kalman_filter);
    _approximate_joint_state.joint1_omega =
        KF_get_vel_estimate(&_joint1_kalman_filter);
  }
}

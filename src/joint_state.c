#include "joint_state.h"
#include "control_law.h"
#include "kf_2dof.h"

KF_2DOF_t _joint1_kalman_filter;
KF_2DOF_t _joint2_kalman_filter;

joint_state _measured_joint_state;
full_joint_state _approximate_joint_state;

joint_state take_measurement_encoders() {
  joint_state temp_state;

  temp_state.joint1_theta = clicks2angle(motor1.encoder_pos);
  temp_state.joint2_theta = clicks2angle(motor2.encoder_pos);

  return temp_state;
}

void init_joint_state_kalman(float encoder1_init_pos, float encoder2_init_pos) {
  _measured_joint_state.joint1_theta = encoder1_init_pos;
  _measured_joint_state.joint2_theta = encoder2_init_pos;

  KF_Init(&_joint1_kalman_filter);
  KF_Init(&_joint2_kalman_filter);

  _joint1_kalman_filter.x_hat_prev_data[0] = encoder1_init_pos;
  _joint2_kalman_filter.x_hat_prev_data[0] = encoder2_init_pos;

  _joint_state_msg.data.data[0] = _approximate_joint_state.joint1_theta;
  _joint_state_msg.data.data[1] = _approximate_joint_state.joint1_omega;

  _joint_state_msg.data.data[2] = _approximate_joint_state.joint2_theta;
  _joint_state_msg.data.data[3] = _approximate_joint_state.joint2_omega;
}

/* Vibrate motors and update-predict Kalman filter multiple times to stabilize K
 * matrix */
void calibrate_joint_state_kalman(uint8_t iterations) {
  float sign = -1.0;

  for (uint8_t i = 0; i < iterations; i++) {
    // TODO change this input voltages
    control_motor1(sign * MIN2MOVE_MOTOR_VOLT);
    control_motor2(-sign * MIN2MOVE_MOTOR_VOLT);
    sleep_ms(50);

    _measured_joint_state = take_measurement_encoders();

    KF_Update(&_joint1_kalman_filter, _control_law_msg.data.data[0],
              _measured_joint_state.joint2_theta);

    KF_Update(&_joint2_kalman_filter, _control_law_msg.data.data[1],
              _measured_joint_state.joint2_theta);

    sign *= -1;
  }

  _approximate_joint_state.joint1_theta =
      KF_get_pos_estimate(&_joint1_kalman_filter);
  _approximate_joint_state.joint2_omega =
      KF_get_pos_estimate(&_joint2_kalman_filter);

  _approximate_joint_state.joint2_theta =
      KF_get_pos_estimate(&_joint2_kalman_filter);
  _approximate_joint_state.joint2_omega =
      KF_get_pos_estimate(&_joint2_kalman_filter);

  _joint_state_msg.data.data[0] = _approximate_joint_state.joint1_theta;
  _joint_state_msg.data.data[1] = _approximate_joint_state.joint1_omega;

  _joint_state_msg.data.data[2] = _approximate_joint_state.joint2_theta;
  _joint_state_msg.data.data[3] = _approximate_joint_state.joint2_omega;
}

/* Encoder interrupts */
void init_encoder_interrupt() {
  gpio_set_irq_enabled_with_callback(motor1.encoder_a,
                                     GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                                     true, &update_encoder_motor);

  gpio_set_irq_enabled(motor2.encoder_a,
                       GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
}

/* Encoder interrupts callbacks */
void update_encoder_motor(uint gpio, uint32_t events) {
  if (gpio == motor1.encoder_a) {
    bool encoder_a_state = gpio_get(motor1.encoder_a);
    bool encoder_b_state = gpio_get(motor1.encoder_b);

    motor1.encoder_pos += (encoder_a_state == encoder_b_state) ? 1 : -1;
  } else if (gpio == motor2.encoder_a) {
    bool encoder_a_state = gpio_get(motor2.encoder_a);
    bool encoder_b_state = gpio_get(motor2.encoder_b);

    motor2.encoder_pos += (encoder_a_state == encoder_b_state) ? 1 : -1;
  }
}

/* Update Kalman prediction and publish joint state ROS callback*/
void joint_state_callback() {
  _measured_joint_state = take_measurement_encoders();

  KF_Update(&_joint1_kalman_filter, _control_law_msg.data.data[0],
            _measured_joint_state.joint1_theta);

  KF_Update(&_joint2_kalman_filter, _control_law_msg.data.data[1],
            _measured_joint_state.joint2_theta);

  _approximate_joint_state.joint1_theta =
      KF_get_pos_estimate(&_joint1_kalman_filter);
  _approximate_joint_state.joint1_omega =
      KF_get_vel_estimate(&_joint1_kalman_filter);

  _approximate_joint_state.joint2_theta =
      KF_get_pos_estimate(&_joint2_kalman_filter);
  _approximate_joint_state.joint2_omega =
      KF_get_vel_estimate(&_joint2_kalman_filter);

  _joint_state_msg.data.data[0] = _approximate_joint_state.joint1_theta;
  _joint_state_msg.data.data[1] = _approximate_joint_state.joint1_omega;

  _joint_state_msg.data.data[2] = _approximate_joint_state.joint2_theta;
  _joint_state_msg.data.data[3] = _approximate_joint_state.joint2_omega;

  RCSOFTCHECK(rcl_publish(&_joint_state_publisher, &_joint_state_msg, NULL));

  DEBUG_CHECKPOINT(11);
}

/* Init joint state to stationary and horizontal right*/
void init_joint_state_values() {
  _measured_joint_state.joint1_theta = 0;
  _measured_joint_state.joint2_theta = 0;

  // Initially both motors at 90deg and stationary
  _approximate_joint_state.joint1_theta = 0;
  _approximate_joint_state.joint2_theta = 0;

  _approximate_joint_state.joint1_omega = 0; // Stationary
  _approximate_joint_state.joint2_omega = 0; // Stationary

  // Init value of msg
  _joint_state_msg.data.data[0] = _approximate_joint_state.joint1_theta;
  _joint_state_msg.data.data[1] = _approximate_joint_state.joint1_omega;

  _joint_state_msg.data.data[2] = _approximate_joint_state.joint2_theta;
  _joint_state_msg.data.data[3] = _approximate_joint_state.joint2_omega;
}

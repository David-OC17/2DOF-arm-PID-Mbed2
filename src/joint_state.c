#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "control_law.h"
#include "joint_state.h"

joint_state _measured_joint_state;
end_efector_state _estimated_end_efector_state;

/* Kalman filter "wrappers" */
joint_state take_measurement_encoders() {
  joint_state temp_state;

  temp_state._joint1_theta = clicks2angle(motor1.encoder_pos);
  temp_state._joint2_theta = clicks2angle(motor2.encoder_pos);

  return temp_state;
}

void init_joint_state_kalman(float encoder1_init_pos, float encoder2_init_pos) {
  // Right horizontal position
  float encoder1_theta = 0.0;
  float encoder2_theta = 0.0;

  KF_2DOF_update(&joint_state_kalman_filter, encoder1_theta, encoder2_theta);
  KF_2DOF_predict(&joint_state_kalman_filter);
}

/* Vibrate motors and update-predict Kalman filter multiple times to stabilize K
 * matrix */
void calibrate_joint_state_kalman(uint8_t iterations) {
  int8_t sign = -1;

  for (uint8_t i = 0; i < iterations; i++) {
    control_motor1(sign * MIN2MOVE_MOTOR_VOLT);
    control_motor2(-sign * MIN2MOVE_MOTOR_VOLT);
    sleep_ms(100); // Allow movement for 10 MS

    KF_2DOF_update(&joint_state_kalman_filter,
                   _measured_joint_state._joint1_theta,
                   _measured_joint_state._joint2_theta);
    KF_2DOF_predict(&joint_state_kalman_filter);

    sign *= -1;
  }

  // Update estimated joint state after calibration
  _estimated_end_efector_state._pos_x =
      KF_2DOF_getX(&joint_state_kalman_filter);
  _estimated_end_efector_state._pos_y =
      KF_2DOF_getY(&joint_state_kalman_filter);

  // Update velocity
  _estimated_end_efector_state._vel_x =
      KF_2DOF_getVX(&joint_state_kalman_filter);
  _estimated_end_efector_state._vel_y =
      KF_2DOF_getVY(&joint_state_kalman_filter);

  // Update ROS msg and publish
  _joint_state_msg.data.data[0] =
      angle2clicks(_estimated_end_efector_state._pos_x);
  _joint_state_msg.data.data[1] =
      angle2clicks(_estimated_end_efector_state._pos_y);

  _joint_state_msg.data.data[2] =
      angle2clicks(_estimated_end_efector_state._vel_x);
  _joint_state_msg.data.data[3] =
      angle2clicks(_estimated_end_efector_state._vel_y);
}

/* Encoder interrupts */
void init_encoder_interrupt() {
  // TODO check IRQ EDGE behaviour
  gpio_set_irq_enabled_with_callback(motor1.encoder_a,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true, &update_encoder_motor1);
  gpio_set_irq_enabled_with_callback(motor2.encoder_a,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true, &update_encoder_motor2);
}

/* Encoder interrupts callbacks */
void update_encoder_motor1() {
  int encoder_a_state = gpio_get(motor1.encoder_a);
  int encoder_b_state = gpio_get(motor1.encoder_b);

  // Determine direction of rotation
  if (encoder_a_state == 1) {   // HIGH
    if (encoder_b_state == 0) { // LOW
      motor1.encoder_pos++;
    } else {
      motor1.encoder_pos--;
    }
  } else {
    if (encoder_b_state == 0) { // LOW
      motor1.encoder_pos--;
    } else {
      motor1.encoder_pos++;
    }
  }
}

void update_encoder_motor2() {
  int encoder_a_state = gpio_get(motor2.encoder_a);
  int encoder_b_state = gpio_get(motor2.encoder_b);

  // Determine direction of rotation
  if (encoder_a_state == 1) {   // HIGH
    if (encoder_b_state == 0) { // LOW
      motor2.encoder_pos++;
    } else {
      motor2.encoder_pos--;
    }
  } else {
    if (encoder_b_state == 0) { // LOW
      motor2.encoder_pos--;
    } else {
      motor2.encoder_pos++;
    }
  }
}

/* Update Kalman prediction and publish joint state ROS callback*/
void joint_state_callback() {
  // NOTE the frequency of this callback should match KALMAN_DT_MS timestep
  _measured_joint_state = take_measurement_encoders();

  KF_2DOF_update(&joint_state_kalman_filter,
                 _measured_joint_state._joint1_theta,
                 _measured_joint_state._joint2_theta);
  KF_2DOF_predict(&joint_state_kalman_filter);

  // Update position
  _estimated_end_efector_state._pos_x =
      KF_2DOF_getX(&joint_state_kalman_filter);
  _estimated_end_efector_state._pos_y =
      KF_2DOF_getY(&joint_state_kalman_filter);

  // Update velocity
  _estimated_end_efector_state._vel_x =
      KF_2DOF_getVX(&joint_state_kalman_filter);
  _estimated_end_efector_state._vel_y =
      KF_2DOF_getVY(&joint_state_kalman_filter);

  // Update ROS msg and publish
  _joint_state_msg.data.data[0] =
      angle2clicks(_estimated_end_efector_state._pos_x);
  _joint_state_msg.data.data[1] =
      angle2clicks(_estimated_end_efector_state._pos_y);

  _joint_state_msg.data.data[2] =
      angle2clicks(_estimated_end_efector_state._vel_x);
  _joint_state_msg.data.data[3] =
      angle2clicks(_estimated_end_efector_state._vel_y);

  RCSOFTCHECK(rcl_publish(&_joint_state_publisher, &_joint_state_msg, NULL));
}

/* Init joint state */
void init_joint_state_values(float link1_len, float link2_len) {
  // Initially both motors at 90deg and stationary
  _estimated_end_efector_state._pos_x = link1_len + link2_len;
  _estimated_end_efector_state._pos_y = 0; // Altitude 0 at horizontal

  _estimated_end_efector_state._vel_x = 0; // Stationary
  _estimated_end_efector_state._vel_y = 0; // Stationary

  // Init value of msg
  _joint_state_msg.data.data[0] =
      angle2clicks(_estimated_end_efector_state._pos_x);
  _joint_state_msg.data.data[1] =
      angle2clicks(_estimated_end_efector_state._pos_y);

  _joint_state_msg.data.data[2] =
      angle2clicks(_estimated_end_efector_state._vel_x);
  _joint_state_msg.data.data[3] =
      angle2clicks(_estimated_end_efector_state._vel_y);
}

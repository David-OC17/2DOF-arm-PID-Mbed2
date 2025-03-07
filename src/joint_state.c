#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "joint_state.h"
#include "control_law.h"

rcl_publisher_t _joint_state_publisher;
rcl_node_t _joint_state_node;
rcl_timer_t _joint_state_timer;
std_msgs__msg__Float32MultiArray _joint_state_msg;

joint_state _measured_joint_state;
end_efector_state _estimated_end_efector_state;

#define ENCODER_REVS_PER_ROT 470

/* Kalman filter "wrappers" */
joint_state take_measurement_encoders() {
  joint_state temp_state;

  temp_state._joint1_theta = motor1.encoder_pos * 360 / ENCODER_REVS_PER_ROT;
  temp_state._joint2_theta = motor2.encoder_pos * 360 / ENCODER_REVS_PER_ROT;

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
    control_motor1(sign * MIN_2MOVE_MOTOR_VOLT);
    control_motor2(-sign * MIN_2MOVE_MOTOR_VOLT);
    delay(10); // Allow movement for 10 MS

    KF_2DOF_update(&joint_state_kalman_filter,
                   _measured_joint_state._joint1_theta,
                   _measured_joint_state._joint2_theta);
    KF_2DOF_predict(&joint_state_kalman_filter);

    sign *= -1;
  }

  // Update estimated joint state after calibration
  _estimated_end_efector_state._pos_x = KF_2DOF_getX(&joint_state_kalman_filter);
  _estimated_end_efector_state._pos_y = KF_2DOF_getY(&joint_state_kalman_filter);

  // Update velocity
  _estimated_end_efector_state._vel_x = KF_2DOF_getVX(&joint_state_kalman_filter);
  _estimated_end_efector_state._vel_y = KF_2DOF_getVY(&joint_state_kalman_filter);

  // Update ROS msg and publish
  _joint_state_msg.data.data[0] = _estimated_end_efector_state._pos_x;
  _joint_state_msg.data.data[1] = _estimated_end_efector_state._pos_y;

  _joint_state_msg.data.data[2] = _estimated_end_efector_state._vel_x;
  _joint_state_msg.data.data[3] = _estimated_end_efector_state._vel_y;
}

/* Encoder interrupts */
void init_encoder_interrupt() {
    gpio_set_irq_enabled_with_callback(motor1.encoder_a, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &update_encoder_motor1);
    gpio_set_irq_enabled_with_callback(motor2.encoder_a, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &update_encoder_motor2);
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

/* Publish joint state ROS callback*/
void joint_state_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  // NOTE the frequency of this callback should match KALMAN_DT_MS timestep
  RCLC_UNUSED(last_call_time);

  _measured_joint_state = take_measurement_encoders();

  // TODO change to new Kalman filter interface
  // joint_state_kalman_filter.update(_measured_joint_state._joint1_theta,
  //                                  _measured_joint_state._joint2_theta);
  // joint_state_kalman_filter.predict();

  // Update position
  // _estimated_end_efector_state._pos_x = joint_state_kalman_filter.getX();
  // _estimated_end_efector_state._pos_y = joint_state_kalman_filter.getY();

  // Update velocity
  // _estimated_end_efector_state._vel_x = joint_state_kalman_filter.getVX();
  // _estimated_end_efector_state._vel_y = joint_state_kalman_filter.getVY();

  // Update ROS msg and publish
  _joint_state_msg.data.data[0] = _estimated_end_efector_state._pos_x;
  _joint_state_msg.data.data[1] = _estimated_end_efector_state._pos_y;

  _joint_state_msg.data.data[2] = _estimated_end_efector_state._vel_x;
  _joint_state_msg.data.data[3] = _estimated_end_efector_state._vel_y;

  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&_joint_state_publisher, &_joint_state_msg, NULL));
  }
}

/* Init joint state */
void init_joint_state_values(float link1_len, float link2_len) {
  // Initially both motors at 90deg and stationary
  _estimated_end_efector_state._pos_x = link1_len + link2_len;
  _estimated_end_efector_state._pos_y = 0; // Altitude 0 at horizontal

  _estimated_end_efector_state._vel_x = 0; // Stationary
  _estimated_end_efector_state._vel_y = 0; // Stationary

  // Init value of msg
  _joint_state_msg.data.data[0] = _estimated_end_efector_state._pos_x;
  _joint_state_msg.data.data[1] = _estimated_end_efector_state._pos_y;

  _joint_state_msg.data.data[2] = _estimated_end_efector_state._vel_x;
  _joint_state_msg.data.data[3] = _estimated_end_efector_state._vel_y;
}

/* Create and start start joint state publisher node to spin with timer*/
void init_joint_state() {
  // Create node
  RCCHECK(rclc_node_init_default(&_joint_state_node, "joint_state_node", "",
                                 &_support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &_joint_state_publisher, &_joint_state_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "joint_state_node_publisher"));

  // Create timer
  RCCHECK(rclc_timer_init_default(&_joint_state_timer, &_support,
                                   RCL_MS_TO_NS(JOINT_STATE_TIMEOUT_MS),
                                   joint_state_timer_callback));

  RCCHECK(rclc_executor_init(&_joint_state_executor, &_support.context, 1,
                             &_allocator));

  RCCHECK(rclc_executor_add_timer(&_joint_state_executor, &_joint_state_timer));
}

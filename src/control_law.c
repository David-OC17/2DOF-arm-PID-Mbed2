#include "control_law.h"
#include "joint_state.h"

motor motor1;
motor motor2;

/* Configure motor controller pins */
void init_motor(motor m) {
  gpio_set_function(m.pwm, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(m.pwm);

  gpio_init(m.dir1);
  gpio_set_dir(m.dir1, GPIO_OUT);

  gpio_init(m.dir2);
  gpio_set_dir(m.dir2, GPIO_OUT);

  gpio_init(m.encoder_a);
  gpio_set_dir(m.encoder_a, GPIO_IN);
  // gpio_pull_up(m.encoder_a); // Enable pull-up resistor
  gpio_disable_pulls(m.encoder_a);

  gpio_init(m.encoder_b);
  gpio_set_dir(m.encoder_b, GPIO_IN);
  // gpio_pull_up(m.encoder_b); // Enable pull-up resistor
  gpio_disable_pulls(m.encoder_b);

  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 4.0f); // TODO review clkdiv
  pwm_config_set_wrap(&config, 255);

  pwm_init(slice_num, &config, true);

  m.encoder_pos = 1;
}

/* Control law callback on new voltage for motors received from ROS topic */
void control_law_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *control_voltages =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  DEBUG_SUBSCRIBER_PRINT();

  // Apply new voltages (new PWM)
  control_motor1(control_voltages->data.data[0]);
  control_motor2(control_voltages->data.data[1]);
}

/* Call to motor driver action */
void control_motor1(float volt) {
  // Positive voltage
  if (volt > 0) {
    gpio_put(motor1.dir1, true);
    gpio_put(motor1.dir2, false);
  } else {
    // Negative voltage
    gpio_put(motor1.dir1, false);
    gpio_put(motor1.dir2, true);
  }
  const int16_t duty_cycle = 0; // TODO calculate duty cycle
  pwm_set_gpio_level(motor1.pwm, duty_cycle);
}

void control_motor2(float volt) {
  // Positive voltage
  if (volt > 0) {
    gpio_put(motor1.dir1, true);
    gpio_put(motor1.dir2, false);
  } else {
    // Negative voltage
    gpio_put(motor1.dir1, false);
    gpio_put(motor1.dir2, true);
  }
  const int16_t duty_cycle = 0; // TODO calculate duty cycle
  pwm_set_gpio_level(motor1.pwm, duty_cycle);
}
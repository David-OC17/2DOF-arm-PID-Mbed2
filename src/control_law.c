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

  control_motor1(control_voltages->data.data[0]);
  control_motor2(control_voltages->data.data[1]);

  DEBUG_CHECKPOINT(99);
}

/* Call to motor driver action */
void control_motor1(float volt) {
  if (volt < -200 || volt > 200) {
    if (volt < 0) { // Negative rotation
      gpio_put(motor1.dir1, false);
      gpio_put(motor1.dir2, true);
      pwm_set_gpio_level(motor1.pwm, 255);
    } else {
      gpio_put(motor1.dir1, true);
      gpio_put(motor1.dir2, false);
      pwm_set_gpio_level(motor1.pwm, 255);
    }
  } else {
    bool sign = volt < 0 ? 0 : 1;
    volt = abs(volt);

    gpio_put(motor1.dir1, sign);
    gpio_put(motor1.dir2, !sign);

    uint16_t duty_cycle = volt / 200 * 255;
    pwm_set_gpio_level(motor1.pwm, duty_cycle);
  }
  sleep_ms(1);
}

void control_motor2(float volt) {
  if (volt < -200 || volt > 200) {
    if (volt < 0) { // Negative rotation
      gpio_put(motor2.dir1, false);
      gpio_put(motor2.dir2, true);
      pwm_set_gpio_level(motor2.pwm, 255);
    } else {
      gpio_put(motor2.dir1, true);
      gpio_put(motor2.dir2, false);
      pwm_set_gpio_level(motor2.pwm, 255);
    }
  } else {
    bool sign = volt < 0 ? 0 : 1;
    volt = abs(volt);

    gpio_put(motor2.dir1, sign);
    gpio_put(motor2.dir2, !sign);

    uint16_t duty_cycle = volt / 200 * 255;
    pwm_set_gpio_level(motor2.pwm, duty_cycle);
  }
  sleep_ms(1);
}
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "control_law.h"
#include "joint_state.h"

motor motor1;
motor motor2;

void set_pwm(uint pin, int volt) {
  volt = volt * 255 / 12;
  volt = volt < 0 ? 0 : (volt > 255 ? 255 : volt); // Constrain manual
  uint16_t duty = (volt * 65535) / 255;            // Escalar a 16 bits

  uint slice = pwm_gpio_to_slice_num(pin);
  pwm_set_gpio_level(pin, duty);
}

/* Configure motor controller pins */
void init_motor(motor m) {
  gpio_set_function(m.pwm, GPIO_FUNC_PWM);

  gpio_init(m.dir1);
  gpio_set_dir(m.dir1, GPIO_OUT);

  gpio_init(m.dir2);
  gpio_set_dir(m.dir2, GPIO_OUT);

  gpio_init(m.encoder_a);
  gpio_set_dir(m.encoder_a, GPIO_IN);
  gpio_pull_up(m.encoder_a); // Enable pull-up resistor

  gpio_init(m.encoder_b);
  gpio_set_dir(m.encoder_b, GPIO_IN);
  gpio_pull_up(m.encoder_b); // Enable pull-up resistor

  init_encoder_interrupt();

  uint slice = pwm_gpio_to_slice_num(m.pwm);
  pwm_set_enabled(slice, true);

  m.encoder_pos = 1;
}

/* Control law callback on new voltage for motors received from ROS topic */
void control_law_callback(const void *msgin) {
  const std_msgs__msg__Int16MultiArray* control_voltages = (const std_msgs__msg__Int16MultiArray *)msgin;

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
  set_pwm(motor1.pwm, volt);
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
  set_pwm(motor1.pwm, volt);
}
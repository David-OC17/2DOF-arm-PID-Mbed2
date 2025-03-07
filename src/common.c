#include "common.h"

rclc_executor_t _joint_state_executor;
rclc_executor_t _control_law_executor;

rclc_support_t _support;
rcl_allocator_t _allocator;

void error_loop() {
  while (1) {
    // TODO change to pico-sdk
    Serial2.print("ERROR\n");
    delay(100);
  }
}

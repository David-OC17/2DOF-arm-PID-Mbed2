#include "common.h"

rclc_executor_t _joint_state_executor;
rclc_executor_t _control_law_executor;

rclc_support_t _support;
rcl_allocator_t _allocator;

void init_micro_ros_nodes() {
  // TODO check if one needs only one allocator and support for both, or if one is OK
  _allocator = rcl_get_default_allocator();

  rcl_ret_t ret = rclc_support_init(&_support, 0, NULL, &_allocator);
  if (ret != RCL_RET_OK) {
    error_loop();
  }

  // Create init_options
  RCCHECK(rclc_support_init(&_support, 0, NULL, &_allocator));

}

void error_loop() {
  while (1) {
    Serial2.print("ERROR\n");
    delay(100);
  }
}

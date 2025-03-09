#include "common.h"

rclc_executor_t _joint_state_executor;
rclc_executor_t _control_law_executor;

rclc_executor_t _executor;
rclc_support_t _support;
rcl_allocator_t _allocator;
rcl_node_t _node;

rcl_subscription_t _control_law_subscriber;
rmw_message_info_t _control_law_msg_info;
std_msgs__msg__Float32MultiArray _control_law_msg;

rcl_publisher_t _joint_state_publisher;
std_msgs__msg__Float32MultiArray _joint_state_msg;

const float clicks2angle(uint16_t clicks) {
  return (float)(clicks * (uint16_t)360 / (uint16_t)ENCODER_REVS_PER_ROT);
}

const int16_t angle2clicks(float angle) {
  return (int16_t)(angle / (float)360.0 * (float)ENCODER_REVS_PER_ROT);
}

void init_spi() {
  spi_init(spi_default, 1000 * 1000);
  gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
}

void print_debug_checkpoint(uint8_t val) {
  // Convert the checkpoint_count to a string
  char checkpoint_str[3]; // Temporary buffer for the count (2 digits max)
  snprintf(checkpoint_str, sizeof(checkpoint_str), "%02d", val);

  for (size_t i = 0; i < 2; ++i) {
    debug_out_buf[10 + i] = checkpoint_str[i]; // Position after "CHECKPOINT "
  }

  spi_write_read_blocking(spi_default, debug_out_buf, debug_response_buf,
                          DEBUG_BUF_LEN);
}

void print_debug_publisher() {
    float pos_x = _joint_state_msg.data.data[0];
    float pos_y = _joint_state_msg.data.data[1];
    float vel_x = _joint_state_msg.data.data[2];
    float vel_y = _joint_state_msg.data.data[3];

    snprintf((char*)debug_print_publisher_buf, DEBUG_PRINT_SUBSCRIBER_BUF_LEN, 
             "JOINT STATE - POSX: %.2f, POSY: %.2f, VELX: %.2f, VELY: %.2f", 
             pos_x, pos_y, vel_x, vel_y);

    spi_write_read_blocking(spi_default, debug_print_publisher_buf, debug_response_buf, DEBUG_PRINT_SUBSCRIBER_BUF_LEN);
}


void print_debug_subscriber() {
    float volt_1 = _control_law_msg.data.data[0];
    float volt_2 = _control_law_msg.data.data[1];

    snprintf((char*)debug_print_subscriber_buf, DEBUG_PRINT_SUBSCRIBER_BUF_LEN, 
             "SUBSCRIBER VOLT1: %.2f, VOLT2: %.2f", 
             volt_1, volt_2);

    spi_write_read_blocking(spi_default, debug_print_subscriber_buf, debug_response_buf, DEBUG_PRINT_SUBSCRIBER_BUF_LEN);
}

void error_loop() {
  while (1) {
    gpio_put(LED_PIN, 1);
    sleep_ms(200);
    gpio_put(LED_PIN, 0);
    sleep_ms(200);
  }
}

#define CONTROL_LAW_MSG_ARRAY_SIZE 2
#define JOINT_STATE_MSG_ARRAY_SIZE 4

void init_ros_nodes() {
  _allocator = rcl_get_default_allocator();
  rclc_support_init(&_support, 0, NULL, &_allocator);

  // Create node
  RCCHECK(rclc_node_init_default(&_node, "control_law_node", "", &_support));

  std_msgs__msg__Float32MultiArray__init(&_control_law_msg);
  std_msgs__msg__Float32MultiArray__init(&_joint_state_msg);

  // Init messages to wanted configuration
  _joint_state_msg.data.data =
      (float *)malloc(sizeof(float) * JOINT_STATE_MSG_ARRAY_SIZE);
  _joint_state_msg.data.size = JOINT_STATE_MSG_ARRAY_SIZE;
  _joint_state_msg.data.capacity = JOINT_STATE_MSG_ARRAY_SIZE;

  _control_law_msg.data.data =
      (float *)malloc(sizeof(float) * CONTROL_LAW_MSG_ARRAY_SIZE);
  _control_law_msg.data.size = CONTROL_LAW_MSG_ARRAY_SIZE;
  _control_law_msg.data.capacity = CONTROL_LAW_MSG_ARRAY_SIZE;

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &_joint_state_publisher, &_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "joint_state_topic"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
      &_control_law_subscriber, &_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "control_law_topic"));

  // Init executor for 2 nodes
  RCCHECK(rclc_executor_init(&_executor, &_support.context, 2, &_allocator));

  // Create callback on receive to topic
  RCCHECK(rclc_executor_add_subscription(&_executor, &_control_law_subscriber,
                                         &_control_law_msg,
                                         &control_law_callback, ON_NEW_DATA));
}

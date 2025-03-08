#include "common.h"

rclc_executor_t _joint_state_executor;
rclc_executor_t _control_law_executor;

rclc_support_t _support;
rcl_allocator_t _allocator;

void init_comms() {
  gpio_set_function(DEBUG_RX, UART_FUNCSEL_NUM(uart1, 0));
  gpio_set_function(DEBUG_TX, UART_FUNCSEL_NUM(uart1, 1));
  uart_init(uart1, DEBUG_BAUD_RATE);
}

void print_debug(const char *msg) { uart_puts(uart1, msg); }

void error_loop(char *file, int32_t line) {
  while (1) {
    print_debug(DEFAULT_ERROR_MSG);
    sleep_ms(DEFAULT_ERROR_WAIT_MS);
  }
}

void start_end_blink() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (uint8_t i = 0; i < 20; i++) {
        gpio_put(LED_PIN, 1);  // Turn LED ON
        sleep_ms(200);         // Wait 500ms
        gpio_put(LED_PIN, 0);  // Turn LED OFF
        sleep_ms(200);         // Wait 500ms
    }
}

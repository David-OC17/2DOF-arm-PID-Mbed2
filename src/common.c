#include "common.h"

rclc_executor_t _joint_state_executor;
rclc_executor_t _control_law_executor;

rclc_support_t _support;
rcl_allocator_t _allocator;

void init_comms() {
  uart_init(DEBUG_UART_ID, DEBUG_BAUD_RATE);
  gpio_set_function(DEBUG_UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(DEBUG_UART_RX_PIN, GPIO_FUNC_UART);
}

void print_debug(const char *msg) { uart_puts(DEBUG_UART_ID, msg); }

void init_spi() {
  spi_init(spi_default, 1000 * 1000);
  gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
}

void print_debug_checkpoint(uint8_t val) {
  // Convert the checkpoint_count to a string
  char checkpoint_str[3];  // Temporary buffer for the count (2 digits max)
  snprintf(checkpoint_str, sizeof(checkpoint_str), "%02d", val);

  for (size_t i = 0; i < 2; ++i) {
    debug_out_buf[10 + i] = checkpoint_str[i];  // Position after "CHECKPOINT "
  }

  spi_write_read_blocking(spi_default, debug_out_buf, debug_in_buf, DEBUG_BUF_LEN);
}

void error_loop() {
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

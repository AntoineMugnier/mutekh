#ifndef MCR12_H_
#define MCR12_H_

#include <hexo/types.h>
#include <device/class/char.h>
#include <device/class/uart.h>
#include <device/class/gpio.h>

struct mcr12_s;

struct mcr12_handler_s {
  void (*code_received)(struct mcr12_s *mcr, const uint8_t *code, size_t size);
};

enum mcr12_state_e {
  MCR_STATE_IDLE,
  MCR_STATE_BARCODE,
  MCR_STATE_COMMAND,
  MCR_STATE_COMMAND_DATA_LEN,
  MCR_STATE_COMMAND_DATA,
  MCR_STATE_COMMAND_ETX,
};

struct mcr12_s {
  const struct mcr12_handler_s *handler;
  struct device_gpio_s gpio;
  struct device_char_s serial_port;
  struct device_uart_s uart;
  uint32_t baudrate;
  size_t barcode_size;
  enum mcr12_state_e rx_state;

  struct dev_char_rq_s read_req;

  uint8_t barcode[64];
  uint8_t rx_cmd;
  size_t rx_data_len;
  uint8_t rx_data[256];

  bool_t power;
};

void mcr12_init(struct mcr12_s *mcr,
                const struct mcr12_handler_s *handler,
                uint32_t baudrate);
void mcr12_power(struct mcr12_s *mcr, bool_t power);
void mcr12_trigger(struct mcr12_s *mcr);

#endif

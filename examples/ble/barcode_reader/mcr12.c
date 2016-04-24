#include <mutek/printk.h>
#include "mcr12.h"

#define dprintk(...) do{}while(0)

enum mcr_framing_e
{
  MCR_FRAMING_STX  = 0x02,
  MCR_FRAMING_ETX  = 0x03,
  MCR_FRAMING_ACK  = 0x06,
  MCR_FRAMING_NACK = 0x15,
};

enum mcr_opcode_e
{
  MCR_OP_NO_DATA      = 0x80,

  MCR_OP_CONFIG_TEST  = 0x00,
  MCR_OP_CONFIG_SET   = 0x01,
  MCR_OP_STATUS       = 0x02,
  MCR_OP_DEVICE_SET   = 0x03,
  MCR_OP_PKB_DOWNLOAD = 0x04,
  MCR_OP_PKB_UPLOAD   = 0x05,
  MCR_OP_PC_UPLOAD    = MCR_OP_NO_DATA | 0x00,
  MCR_OP_PC_DOWNLOAD  = MCR_OP_NO_DATA | 0x01,
  MCR_OP_TRIGGER      = MCR_OP_NO_DATA | 0x02,
  MCR_OP_STATUS_GET   = MCR_OP_NO_DATA | 0x03,
  MCR_OP_ISP          = MCR_OP_NO_DATA | 0x04,
  MCR_OP_VERSION_GET  = MCR_OP_NO_DATA | 0x05,
};

static uint8_t mcr_command_checksum(uint8_t cmd, const uint8_t *data, size_t len)
{
  uint8_t ret = MCR_FRAMING_STX ^ MCR_FRAMING_ETX ^ cmd;

  for (size_t i = 0; i < len; ++i)
    ret ^= data[i];

  return ret;
}

static void mcr12_read_next(struct mcr12_s *mcr)
{
  mcr->read_req.type = DEV_CHAR_READ;

  switch (mcr->rx_state) {
  case MCR_STATE_IDLE:
  case MCR_STATE_BARCODE:
    mcr->read_req.data = mcr->barcode + mcr->barcode_size;
    mcr->read_req.size = 1;
    break;

  case MCR_STATE_COMMAND:
    mcr->read_req.data = &mcr->rx_cmd;
    mcr->read_req.size = 1;
    break;

  case MCR_STATE_COMMAND_DATA:
    mcr->read_req.data = mcr->rx_data;
    mcr->read_req.size = mcr->rx_data_len;
    break;

  case MCR_STATE_COMMAND_DATA_LEN:
    mcr->read_req.data = mcr->barcode;
    mcr->read_req.size = 1;
    break;

  case MCR_STATE_COMMAND_ETX:
    mcr->read_req.data = mcr->barcode;
    mcr->read_req.size = 2;
    break;
  }

  dprintk("%s %d %d\n", __FUNCTION__, mcr->rx_state, mcr->read_req.size);
  
  DEVICE_OP(&mcr->serial_port, request, &mcr->read_req);
}

static void mcr12_write(struct mcr12_s *mcr, const uint8_t *data, size_t size)
{
  dprintk("%s %P\n", __FUNCTION__, data, size);

  dev_char_wait_op(&mcr->serial_port, DEV_CHAR_WRITE, (void*)data, size);
}

static KROUTINE_EXEC(read_done)
{
  struct mcr12_s *mcr = KROUTINE_CONTAINER(kr, *mcr, read_req.base.kr);

  dprintk("%s\n", __FUNCTION__);

  switch (mcr->rx_state) {
  case MCR_STATE_IDLE:
    switch (mcr->barcode[0]) {
    case MCR_FRAMING_STX:
      dprintk("STX\n");
      mcr->rx_state = MCR_STATE_COMMAND;
      break;

    case MCR_FRAMING_ACK:
      dprintk("ACK\n");
      break;

    case MCR_FRAMING_NACK:
      dprintk("NACK\n");
      break;
      
    case '\r':
    case '\n':
      mcr->barcode_size = 0;
      break;

    default:
      dprintk("Code\n");
      mcr->rx_state = MCR_STATE_BARCODE;
      mcr->barcode_size = 1;
      break;
    }
    break;
    
  case MCR_STATE_BARCODE:
    switch (mcr->barcode[mcr->barcode_size]) {
    case '\r':
    case '\n':
      mcr->handler->code_received(mcr, mcr->barcode, mcr->barcode_size);
      mcr->barcode_size = 0;
      goto ack;

    default:
      mcr->barcode_size++;
      break;
    }
    break;

  case MCR_STATE_COMMAND:
    mcr->rx_data_len = 0;
    mcr->rx_state = (mcr->rx_cmd & MCR_OP_NO_DATA)
      ? MCR_STATE_COMMAND_ETX
      : MCR_STATE_COMMAND_DATA_LEN;
    break;

  case MCR_STATE_COMMAND_DATA_LEN:
    mcr->rx_data_len = mcr->barcode[0] + 1;
    mcr->rx_state = MCR_STATE_COMMAND_DATA;
    break;

  case MCR_STATE_COMMAND_DATA:
    mcr->rx_state = MCR_STATE_COMMAND_ETX;
    break;

  case MCR_STATE_COMMAND_ETX:
    if (mcr->barcode[0] != MCR_FRAMING_ETX)
      goto nack;

    if (mcr_command_checksum(mcr->rx_cmd, mcr->rx_data, mcr->rx_data_len)
        == mcr->barcode[1])
      goto ack;
    else
      goto nack;
    break;
  }
 again:
  mcr12_read_next(mcr);
  return;

 ack:
  mcr12_write(mcr, (const uint8_t []){MCR_FRAMING_ACK}, 1);
  mcr->rx_state = MCR_STATE_IDLE;
  goto again;
  
 nack:
  mcr12_write(mcr, (const uint8_t []){MCR_FRAMING_NACK}, 1);
  mcr->rx_state = MCR_STATE_IDLE;
  goto again;
}

static
error_t mcr12_uart_config(struct mcr12_s *mcr,
                          uint32_t baudrate)
{
  struct dev_uart_config_s config = {
    .baudrate = baudrate,
    .data_bits = 8,
    .stop_bits = 1,
    .flow_ctrl = 0,
    .half_duplex = 0,
    .parity = DEV_UART_PARITY_NONE,
  };

  return dev_uart_config(&mcr->uart, &config);
}

static
error_t mcr12_command_send(struct mcr12_s *mcr, uint8_t command,
                           const uint8_t *data, size_t data_len)
{
  uint8_t chk;
  uint8_t tmp[2];

  chk = mcr_command_checksum(command, data, data_len);

  tmp[0] = MCR_FRAMING_STX;
  tmp[1] = command;
  mcr12_write(mcr, tmp, 2);

  if (!(command & MCR_OP_NO_DATA)) {
    assert(data_len && data_len <= 256);

    tmp[0] = data_len - 1;
    mcr12_write(mcr, tmp, 1);
    mcr12_write(mcr, data, data_len);
  }

  tmp[0] = MCR_FRAMING_ETX;
  tmp[1] = chk;
  mcr12_write(mcr, tmp, 2);

  return 0;
}

void mcr12_init(struct mcr12_s *mcr,
                const struct mcr12_handler_s *handler,
                uint32_t baudrate)
{
  int ret;

  ret = device_get_accessor_by_path(&mcr->gpio.base, NULL,
                                    "/gpio*", DRIVER_CLASS_GPIO);
  ensure(ret == 0);

  ret = device_get_accessor_by_path(&mcr->serial_port.base, NULL,
                                    "/uart*", DRIVER_CLASS_CHAR);
  ensure(ret == 0);

  ret = device_get_accessor_by_path(&mcr->uart.base, NULL,
                                    "/uart*", DRIVER_CLASS_UART);
  ensure(ret == 0);

  ret = mcr12_uart_config(mcr, baudrate);
  ensure(ret == 0);

  mcr->handler = handler;
  mcr->rx_state = MCR_STATE_IDLE;
  
  kroutine_init_deferred(&mcr->read_req.base.kr, read_done);

  mcr->power = 0;
  
  DEVICE_OP(&mcr->gpio, set_output, 14, 14, dev_gpio_mask0, dev_gpio_mask0);
  DEVICE_OP(&mcr->gpio, set_mode, 14, 14, dev_gpio_mask1, DEV_PIN_PUSHPULL);

  mcr12_read_next(mcr);
}

void mcr12_power(struct mcr12_s *mcr, bool_t power)
{
  if (mcr->power == power)
    return;

  mcr->power = power;

  if (power) {
    device_start(&mcr->serial_port.base);
  } else {
    device_stop(&mcr->serial_port.base);
  }

  const uint8_t *mask = power ? dev_gpio_mask1 : dev_gpio_mask0;
  DEVICE_OP(&mcr->gpio, set_output, 14, 14, mask, mask);
}

void mcr12_trigger(struct mcr12_s *mcr)
{
  mcr12_command_send(mcr, MCR_OP_TRIGGER, NULL, 0);
}

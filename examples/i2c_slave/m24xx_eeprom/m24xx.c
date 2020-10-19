#define LOGK_MODULE_ID "m24x"

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/startup.h>
#include <device/class/i2c_slave.h>

enum slave_state_e
{
  SLAVE_IDLE,
  SLAVE_ADDR_RX,
  SLAVE_DATA_RX,
  SLAVE_DATA_TX,
};

struct ctx_s
{
  struct device_i2c_slave_s slave;
  struct dev_i2c_slave_rq_s ssel_rq;
  struct dev_i2c_slave_rq_s data_rq;
  enum slave_state_e state;
  uint16_t addr;
  uint8_t data[18];
  uint8_t memory[16384];
};

STRUCT_COMPOSE(ctx_s, ssel_rq);
STRUCT_COMPOSE(ctx_s, data_rq);

static
void ssel_rq_enqueue(struct ctx_s *ctx)
{
  ctx->ssel_rq.type = DEV_I2C_SLAVE_SELECTION;
  ctx->ssel_rq.selection.saddr = 0x2a;
  ctx->ssel_rq.selection.saddr_mask = 0x7f;

  logk_trace("Pushing ssel rq");

  DEVICE_OP(&ctx->slave, request, &ctx->ssel_rq);
}

static
void data_rq_enqueue(struct ctx_s *ctx)
{
  switch (ctx->state) {
  case SLAVE_IDLE:
    return;

  case SLAVE_ADDR_RX:
    ctx->data_rq.type = DEV_I2C_SLAVE_RECEIVE;
    ctx->data_rq.transfer.data = ctx->data;
    ctx->data_rq.transfer.size = sizeof(ctx->data);
    ctx->data_rq.transfer.end_ack = 1;

    logk_trace("Pushing address RX");

    DEVICE_OP(&ctx->slave, request, &ctx->data_rq);
    break;

  case SLAVE_DATA_RX:
    ctx->data_rq.type = DEV_I2C_SLAVE_RECEIVE;
    ctx->data_rq.transfer.data = ctx->data;
    ctx->data_rq.transfer.size = sizeof(ctx->data);
    ctx->data_rq.transfer.end_ack = 1;

    logk_trace("Pushing data RX at %04x", ctx->addr);

    DEVICE_OP(&ctx->slave, request, &ctx->data_rq);
    break;

  case SLAVE_DATA_TX: {
    size_t s1 = __MIN(sizeof(ctx->memory) - ctx->addr, sizeof(ctx->data));
    size_t s2 = sizeof(ctx->data) - s1;

    memcpy(ctx->data, ctx->memory + ctx->addr, s1);
    if (s2)
      memcpy(ctx->data + s1, ctx->memory, s2);

    ctx->data_rq.type = DEV_I2C_SLAVE_TRANSMIT;
    ctx->data_rq.transfer.data = ctx->data;
    ctx->data_rq.transfer.size = sizeof(ctx->data);
    ctx->data_rq.transfer.end_ack = 1;

    logk_trace("Pushing data TX at %04x", ctx->addr);
    
    DEVICE_OP(&ctx->slave, request, &ctx->data_rq);
    break;
  }
  }
}

static
KROUTINE_EXEC(ssel_done)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, ssel_rq.base.kr);
  error_t err = ctx->ssel_rq.error;
  uint8_t saddr = ctx->ssel_rq.selection.saddr;
  bool_t read = ctx->ssel_rq.selection.read;
  
  logk_trace("%s", __func__);

  ssel_rq_enqueue(ctx);

  if (err) {
    logk_trace("Slave selection ended in error %d", err);

    ctx->state = SLAVE_IDLE;
  } else {
    logk_debug("Slave at address %02x selected for %s",
         saddr,
         read ? "read" : "write");

    if (read)
      ctx->state = SLAVE_DATA_TX;
    else
      ctx->state = SLAVE_ADDR_RX;

    data_rq_enqueue(ctx);
  }
}

static
KROUTINE_EXEC(data_done)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, data_rq.base.kr);
  size_t size;
  uint8_t *data;

  logk_trace("%s", __func__);

  if (ctx->data_rq.error) {
    logk_trace("Data request ended in error %d", ctx->data_rq.error);

    ctx->state = SLAVE_IDLE;
    return;
  }
  
  switch (ctx->state) {
  case SLAVE_IDLE:
    return;

  case SLAVE_ADDR_RX:
    size = ctx->data_rq.transfer.data - ctx->data;
    data = ctx->data;

    logk_debug("Address received: %P", data, size);

    if (size < sizeof(ctx->addr))
      return;

    ctx->addr = endian_be16_na_load(data);
    logk_debug("Address received: %04x", ctx->addr);
    ctx->state = SLAVE_DATA_RX;

    size -= sizeof(ctx->addr);
    data += sizeof(ctx->addr);
    goto data_rx;

  case SLAVE_DATA_RX:
    size = ctx->data_rq.transfer.data - ctx->data;
    data = ctx->data;

  data_rx: {
    logk_debug("Rx data @%04x: %P", ctx->addr, data, size);

    size_t s1 = __MIN(sizeof(ctx->memory) - ctx->addr, size);
    size_t s2 = size - s1;

    memcpy(ctx->memory + ctx->addr, data, s1);
    if (s2)
      memcpy(ctx->memory, data + s1, s2);

    ctx->addr += size;
    if (ctx->addr >= sizeof(ctx->memory))
      ctx->addr = s2;

    data_rq_enqueue(ctx);
    return;
  }

  case SLAVE_DATA_TX: {
    size_t size = ctx->data_rq.transfer.data - ctx->data;
    size_t ack = ctx->data_rq.transfer.data - ctx->data;

    size_t s1 = __MIN(sizeof(ctx->memory) - ctx->addr, size);
    size_t s2 = size - s1;

    ctx->addr += size;
    if (ctx->addr >= sizeof(ctx->memory))
      ctx->addr = s2;

    logk_debug("Tx data done with %s, next addr: %04x",
         ack ? "ACK" : "NACK", ctx->addr);

    if (ack)
      data_rq_enqueue(ctx);
    return;
  }
  }
}

void app_start(void)
{
  error_t err;
  struct ctx_s *ctx;

  ctx = mem_alloc(sizeof(*ctx), mem_scope_sys);
  assert(ctx);

  memset(ctx, 0, sizeof(*ctx));
  
  err = device_get_accessor_by_path(&ctx->slave.base, NULL, "i2cs0", DRIVER_CLASS_I2C_SLAVE);
  ensure(!err && "Could not get I2C device instance");

  dev_i2c_slave_rq_init(&ctx->ssel_rq, ssel_done);
  dev_i2c_slave_rq_init(&ctx->data_rq, data_done);

  ctx->state = SLAVE_IDLE;
  ssel_rq_enqueue(ctx);
}

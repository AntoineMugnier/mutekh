#include <mutek/printk.h>
#include <mutek/startup.h>
#include <device/class/char.h>
#include <device/class/crypto.h>
#include <device/class/timer.h>
#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <hexo/decls.h>

enum mode_e
{
  BASE,
  SPEEDTEST_RX,
  SPEEDTEST_TX,
};

struct ctx_s {
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  struct device_crypto_s crc;
  struct dev_crypto_context_s crc_ctx;
  struct dev_crypto_rq_s crc_rq;
  uint32_t crc_state[4];
  uint8_t crc_data[4];
  struct device_char_s tag;
  struct dev_char_rq_s tag_rq;
  uint8_t buffer[64];
  uint16_t ctr;
  enum mode_e mode;
  size_t count;
};

STRUCT_COMPOSE(ctx_s, tag_rq);

static
void tag_write(struct ctx_s *ctx);
static
void tag_read(struct ctx_s *ctx);

static void timeout_reset(struct ctx_s *ctx)
{
  error_t err;
  
  printk("Tag timeout reset\n");

  if (ctx->timer_rq.base.pvdata) {
    err = DEVICE_OP(&ctx->timer, cancel, &ctx->timer_rq);
    if (err) {
      printk("Cancel failed: %d\n", err);
      return;
    }
  }
  
  ctx->timer_rq.base.pvdata = ctx;
  printk("Timeout: %d\n", ctx->timer_rq.delay);
  err = DEVICE_OP(&ctx->timer, request, &ctx->timer_rq);
  if (err) {
    printk("Timeout setup failed: %d\n", err);
  }
}

static void crc_update(struct ctx_s *ctx)
{
  printk("CRC update %d bytes, op %x\n", ctx->crc_rq.ad_len, ctx->crc_rq.op);
  DEVICE_OP(&ctx->crc, request, &ctx->crc_rq);
}

static void speedtest_tx_next(struct ctx_s *ctx)
{
  printk("Speedtest TX %d\n", ctx->count);
  
  ctx->count--;
  endian_le16_na_store(ctx->buffer, ctx->count);
  
  if (ctx->count == 0) {
    memcpy(ctx->buffer, "finish_S", 8);
    ctx->crc_rq.ad_len = 60;
    ctx->crc_rq.op |= DEV_CRYPTO_FINALIZE;
  }
  
  crc_update(ctx);
}

static
KROUTINE_EXEC(tag_timeout)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, timer_rq.base.kr);
  error_t err;
  
  printk("Tag timeout !\n");

  ctx->timer_rq.base.pvdata = NULL;

  switch (ctx->mode) {
  case BASE:
    return;

  case SPEEDTEST_RX:
  case SPEEDTEST_TX:
    ctx->mode = BASE;
    err = DEVICE_OP(&ctx->tag, cancel, &ctx->tag_rq);
    printk("tag rq cancel: %d\n", err);
    if (!err)
      tag_read(ctx);
    break;
  }  
}

static
KROUTINE_EXEC(tag_write_done)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, tag_rq.base.kr);

  printk("Write done %d\n", ctx->tag_rq.error);

  if (ctx->tag_rq.error) {
    ctx->mode = BASE;
    tag_read(ctx);
    return;
  }

  switch (ctx->mode) {
  case BASE:
    tag_read(ctx);
    break;

  case SPEEDTEST_TX:
    if (ctx->count == 0) {
      ctx->mode = BASE;
      tag_read(ctx);
    } else {
      speedtest_tx_next(ctx);
    }
    break;

  case SPEEDTEST_RX:
    assert(0);
    break;
  }
}

static
KROUTINE_EXEC(crc_done)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, crc_rq.base.kr);

  printk("CRC done %d\n", ctx->crc_rq.error);
  ctx->crc_rq.op = 0;

  switch (ctx->mode) {
  case BASE:
    break;
    
  case SPEEDTEST_RX:
    if (!memcmp(ctx->buffer, "finish_S", 8)) {
      printk("Speedtest RX done\n");
      uint32_t computed = ~endian_be32_na_load(ctx->crc_data);
      uint32_t in_data = endian_le32_na_load(ctx->buffer + 60);
      printk("CRC final %08x, expected %08x\n", computed, in_data);
      ctx->mode = SPEEDTEST_TX;
      ctx->crc_rq.op = DEV_CRYPTO_INIT;
      ctx->crc_rq.ad_len = 64;

      memset(ctx->buffer, 0, 64);
      ctx->buffer[59] = computed != in_data;

      speedtest_tx_next(ctx);
    } else {
      printk("Speedtest RX next\n");
      timeout_reset(ctx);
      tag_read(ctx);
    }
    break;

  case SPEEDTEST_TX:
    if (ctx->count == 0) {
      uint32_t computed = ~endian_be32_na_load(ctx->crc_data);
      endian_le32_na_store(ctx->buffer + 60, computed);
    }
    printk("Speedtest TX %d\n", ctx->count);
    tag_write(ctx);
    break;
  }
}

static
KROUTINE_EXEC(tag_read_done)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, tag_rq.base.kr);

  printk("Read done %d\n", ctx->tag_rq.error);

  if (ctx->tag_rq.error) {
    ctx->mode = BASE;
    tag_read(ctx);
    return;
  }

  switch (ctx->mode) {
  case BASE:
    switch (ctx->buffer[60]) {
    case 'S':
      printk("Speedtest start\n");
      ctx->mode = SPEEDTEST_RX;
      ctx->count = 0;
      ctx->crc_rq.ad_len = 64;
      ctx->crc_rq.op = DEV_CRYPTO_INIT;
      tag_read(ctx);
      break;

    case 'L':
      printk("LED Demo %c\n", "-RBG"[ctx->buffer[61] & 0x3]);
      printk("Temp sensor %s\n", ctx->buffer[53] == 'E' ? "on" : "off");
      printk("LCD %s, %s message\n",
             ctx->buffer[54] == 'E' ? "on" : "off",
             ctx->buffer[53] == 'E' ? "NDEF" : "Default");

      endian_be16_na_store(&ctx->buffer[58], ((ctx->ctr % 64) + 273) << 2);
      endian_le16_na_store(&ctx->buffer[56], (ctx->ctr % 8) << 7);

      ctx->buffer[62] =  ctx->ctr % 8;
      tag_write(ctx);
      break;
      
    case 'R':
      printk("Factory reset\n");
      tag_read(ctx);
      break;

    case 'C':
      printk("Cube demo\n");
      printk("Leds: R %02x, G %02x, B %02x\n",
             ctx->buffer[57], ctx->buffer[58], ctx->buffer[59]);
      // Should write back buttons in
      //   ctx->buffer[57]
      //   ctx->buffer[58]
      //   ctx->buffer[59]
      tag_write(ctx);
      break;

    case 'V':
      printk("Get version\n");
      strcpy((void*)ctx->buffer,
             "Board Ver.: 4.3\n"
             "FW    Ver.: lol\n");
      ctx->ctr++;

      tag_write(ctx);
      break;

    default:
      printk("Unhandled action %c\n", ctx->buffer[60]);
      tag_write(ctx);
      break;
    }
    break;

  case SPEEDTEST_RX:
    printk("Speedtest RX %d, %d\n", ctx->count, endian_le16_na_load(ctx->buffer));
    ctx->count++;
    if (!memcmp(ctx->buffer, "finish_S", 8)) {
      ctx->crc_rq.op |= DEV_CRYPTO_FINALIZE;
      ctx->crc_rq.ad_len = 60;
    }
    crc_update(ctx);
    break;

  case SPEEDTEST_TX:
    assert(0);
    break;
  }
}

static
void tag_write(struct ctx_s *ctx)
{
  printk("Writing...\n");
  
  dev_char_rq_init(&ctx->tag_rq, tag_write_done);
  ctx->tag_rq.data = ctx->buffer;
  ctx->tag_rq.size = 64;
  ctx->tag_rq.type = DEV_CHAR_WRITE_FRAME;
  timeout_reset(ctx);
  DEVICE_OP(&ctx->tag, request, &ctx->tag_rq);
}

static
void tag_read(struct ctx_s *ctx)
{
  printk("Reading...\n");

  dev_char_rq_init(&ctx->tag_rq, tag_read_done);
  ctx->tag_rq.data = ctx->buffer;
  ctx->tag_rq.size = 64;
  ctx->tag_rq.type = DEV_CHAR_READ_FRAME;
  DEVICE_OP(&ctx->tag, request, &ctx->tag_rq);
}

void app_start(void)
{
  struct ctx_s *ctx = mem_alloc(sizeof(*ctx), mem_scope_sys);
  error_t err;

  memset(ctx, 0, sizeof(*ctx));
  
  err = device_get_accessor_by_path(&ctx->tag.base, NULL, "ntag", DRIVER_CLASS_CHAR);
  ensure(!err && "Could not get NTAG accessor");

  err = device_get_accessor_by_path(&ctx->crc.base, NULL, "crc_soft[1]", DRIVER_CLASS_CRYPTO);
  ensure(!err && "Could not get CRC accessor");

  err = device_get_accessor_by_path(&ctx->timer.base, NULL, "rtc* timer*", DRIVER_CLASS_TIMER);
  ensure(!err && "Could not get timer accessor");

  ctx->mode = BASE;
  ctx->crc_ctx.mode = DEV_CRYPTO_MODE_HASH;
  ctx->crc_rq.ctx = &ctx->crc_ctx;
  ctx->crc_rq.ad = ctx->buffer;
  ctx->crc_rq.out = ctx->crc_data;
  ctx->crc_rq.len = 4;
  ctx->crc_ctx.state_data = ctx->crc_state;

  ctx->timer_rq.base.pvdata = NULL;

  dev_timer_rq_init(&ctx->timer_rq, &tag_timeout);
  dev_timer_init_sec(&ctx->timer, &ctx->timer_rq.delay, 0, 1, 2);

  dev_char_rq_init(&ctx->crc_rq, crc_done);
  
  tag_read(ctx);
}


#include <mutek/printk.h>
#include <mutek/startup.h>
#include <device/class/char.h>
#include <device/class/crypto.h>
#include <device/class/timer.h>
#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <hexo/decls.h>

enum op_e
{
  OP_IDLE,
  OP_TX_RESUME,
  OP_TX_SPEC,
  OP_TX_DONE,
};

struct ctx_s {
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  struct device_char_s tag;
  struct dev_char_rq_s tag_rq;
  struct device_crypto_s crc;
  struct dev_crypto_context_s crc_ctx;
  struct dev_crypto_rq_s crc_rq;
  uint8_t buffer[64];
  uint8_t crc_data[4];
  uint16_t next_block;
  uint32_t firmware_size;
  uint32_t chunk_size;
  enum op_e op;
};

STRUCT_COMPOSE(ctx_s, tag_rq);

static KROUTINE_EXEC(tag_write_done);
static KROUTINE_EXEC(tag_read_done);
static void timeout_reset(struct ctx_s *ctx);

static
void tag_write(struct ctx_s *ctx)
{
  printk("Writing...\n");
  
  kroutine_init_deferred(&ctx->tag_rq.base.kr, tag_write_done);
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

  kroutine_init_deferred(&ctx->tag_rq.base.kr, tag_read_done);
  ctx->tag_rq.data = ctx->buffer;
  ctx->tag_rq.size = 64;
  ctx->tag_rq.type = DEV_CHAR_READ_FRAME;
  DEVICE_OP(&ctx->tag, request, &ctx->tag_rq);
}

static
void tag_state_send(struct ctx_s *ctx)
{
  endian_le16_na_store(ctx->buffer, ctx->op);
  endian_le16_na_store(ctx->buffer + 2, ctx->next_block);
  memset(ctx->buffer + 4, 0, 64 - 4);
  
  printk("State send op %d next %d\n",
         ctx->op, ctx->next_block);
  
  tag_write(ctx);
}

static
void tag_packet_check_done(struct ctx_s *ctx, bool_t ok)
{
  enum op_e next_op;

  if (!ok) {
    next_op = OP_TX_RESUME;
  } else {
    next_op = OP_TX_SPEC;
    ctx->next_block = ctx->next_block + 1;

    if (ctx->next_block * ctx->chunk_size >= ctx->firmware_size) {
      next_op = OP_TX_DONE;
      ctx->next_block = 0;
    }
  }

  if (next_op == ctx->op) {
    tag_read(ctx);
    return;
  }

  ctx->op = next_op;
  
  tag_state_send(ctx);
}

static
void tag_packet_check(struct ctx_s *ctx)
{
  printk("Packet check. No %d, next block %d\n",
         endian_le16_na_load(ctx->buffer + 2), ctx->next_block);
  
  if (endian_le16_na_load(ctx->buffer + 2) < ctx->next_block)
    return tag_read(ctx);

  if (endian_le16_na_load(ctx->buffer + 2) != ctx->next_block)
    return tag_packet_check_done(ctx, 0);

  if (ctx->buffer[2] != ctx->buffer[8])
    return tag_packet_check_done(ctx, 0);

  if (memcstcmp(ctx->buffer + 8, ctx->buffer[8], 64 - 8))
    return tag_packet_check_done(ctx, 0);

  ctx->crc_rq.ad = ctx->buffer + 8;
  ctx->crc_rq.ad_len = 64 - 8;

  DEVICE_OP(&ctx->crc, request, &ctx->crc_rq);
}

static void timeout_reset(struct ctx_s *ctx)
{
  error_t err;
  
  //printk("Tag timeout reset\n");

  if (ctx->timer_rq.rq.pvdata) {
    err = DEVICE_OP(&ctx->timer, cancel, &ctx->timer_rq);
    if (err) {
      //printk("Cancel failed: %d\n", err);
      return;
    }
  }
  
  ctx->timer_rq.rq.pvdata = ctx;
  //printk("Timeout: %d\n", ctx->timer_rq.delay);
  err = DEVICE_OP(&ctx->timer, request, &ctx->timer_rq);
  if (err) {
    //printk("Timeout setup failed: %d\n", err);
  }
}

static
KROUTINE_EXEC(tag_timeout)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, timer_rq.rq.kr);
  error_t err;
  
  printk("Tag timeout !\n");

  ctx->timer_rq.rq.pvdata = NULL;

  err = DEVICE_OP(&ctx->tag, cancel, &ctx->tag_rq);
  
  if (err == 0 || err == -ENOENT) {
    ctx->op = OP_IDLE;
    tag_read(ctx);
  }
}

static
KROUTINE_EXEC(crc_done)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, crc_rq.base.kr);
  uint32_t got = endian_le32_na_load(ctx->buffer + 4);
  uint32_t expected = ~endian_be32_na_load(ctx->crc_data);
  
  //printk("CRC done %d, got %08x, expected %08x\n", ctx->crc_rq.err,
  //       got, expected);

  tag_packet_check_done(ctx, got == expected);
}

static
KROUTINE_EXEC(tag_write_done)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, tag_rq.base.kr);

  //printk("Write done %d\n", ctx->tag_rq.error);

  if (ctx->tag_rq.error)
    ctx->op = OP_IDLE;

  tag_read(ctx);
}

static
KROUTINE_EXEC(tag_read_done)
{
  struct ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, tag_rq.base.kr);

  //printk("Read done %d\n", ctx->tag_rq.error);

  timeout_reset(ctx);

  //hexdumpk(0, ctx->buffer, 64);
  
  if (ctx->tag_rq.error)
    goto stop;

  printk("Current op: %d, received %d\n", ctx->op, endian_le16_na_load(ctx->buffer));

  switch (ctx->op) {
  case OP_IDLE:
    switch (endian_le16_na_load(ctx->buffer)) {
    case OP_IDLE:
    restart:
      printk("Restarting download\n");
      ctx->op = OP_TX_RESUME;
      ctx->firmware_size = endian_le32_na_load(ctx->buffer + 4);
      ctx->chunk_size = endian_le32_na_load(ctx->buffer + 8);
      tag_state_send(ctx);
      return;

    default:
      tag_read(ctx);
      return;
    }

  case OP_TX_RESUME:
  case OP_TX_SPEC:
    switch (endian_le16_na_load(ctx->buffer)) {
    case OP_IDLE:
      goto restart;

    case OP_TX_RESUME:
    case OP_TX_SPEC:
      tag_packet_check(ctx);
      return;

    case OP_TX_DONE:
    default:
      goto stop;
    }
    
  case OP_TX_DONE:
  stop:
    ctx->op = OP_IDLE;
    tag_read(ctx);
    return;
  }
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

  ctx->op = OP_IDLE;
  ctx->crc_ctx.mode = DEV_CRYPTO_MODE_HASH;
  ctx->crc_rq.ctx = &ctx->crc_ctx;
  ctx->crc_rq.out = ctx->crc_data;
  ctx->crc_rq.len = 4;
  ctx->crc_rq.op = DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE;

  ctx->timer_rq.rq.pvdata = NULL;

  kroutine_init_deferred(&ctx->timer_rq.rq.kr, &tag_timeout);
  dev_timer_init_sec(&ctx->timer, &ctx->timer_rq.delay, 0, 1, 2);

  kroutine_init_deferred(&ctx->crc_rq.base.kr, crc_done);
  
  tag_read(ctx);
}

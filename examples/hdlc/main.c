#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>
#include <drivers/char/hdlc.h>

static DRIVER_HDLC_RX_MAP(hdlc_dev_rx_map, 0x00, 0x03);

DEV_DECLARE_STATIC(pipe_dev, "pipe", 0, char_pipe_drv,
                   DEV_STATIC_RES_UINT_ARRAY_PARAM("fifos", 128, 128)
                   );

DEV_DECLARE_STATIC(hdlc0_dev, "hdlc0", 0, hdlc_drv,
                   DEV_STATIC_RES_DEV_PARAM("io", "/pipe[0]"),
                   );

DEV_DECLARE_STATIC(hdlc1_dev, "hdlc1", 0, hdlc_drv,
                   DEV_STATIC_RES_DEV_PARAM("io", "/pipe[1]"),
                   DEV_STATIC_RES_BLOB_PARAM("rx_map", hdlc_dev_rx_map),
                   );

struct hdlc_rx_context_s
{
  uint8_t frame[32];
  struct device_char_s input_pipe;
  struct dev_char_rq_s rq;
  const char *prefix;
};

STRUCT_COMPOSE(hdlc_rx_context_s, rq);

static void hdlc_rx_submit(struct hdlc_rx_context_s *ctx)
{
  ctx->rq.type = DEV_CHAR_READ_FRAME;
  ctx->rq.data = ctx->frame;
  ctx->rq.size = sizeof(ctx->frame);
  DEVICE_OP(&ctx->input_pipe, request, &ctx->rq);
}

static KROUTINE_EXEC(hdlc_rx_done)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct hdlc_rx_context_s *ctx = hdlc_rx_context_s_from_rq(rq);

  if (rq->base.error) {
    logk("%s: read error: %s", ctx->prefix, strerror(rq->base.error));
  } else {
    logk("%s: %P", ctx->prefix, ctx->frame, rq->data - ctx->frame);
  }
  
  hdlc_rx_submit(ctx);
}

static void hdlc_rx_start(const char *dev_name)
{
  struct hdlc_rx_context_s *ctx = mem_alloc(sizeof(*ctx), mem_scope_sys);
  error_t err;

  memset(ctx, 0, sizeof(*ctx));

  ctx->prefix = dev_name;
  
  err = device_get_accessor_by_path(&ctx->input_pipe.base, NULL, dev_name, DRIVER_CLASS_CHAR);
  ensure(!err);

  dev_char_rq_init(&ctx->rq, hdlc_rx_done);
  hdlc_rx_submit(ctx);
}

static
void _send(struct device_char_s *device, enum dev_char_rq_type_e op, const uint8_t *data, size_t size)
{
  error_t err;
  logk("TX %s: %P", op == DEV_CHAR_WRITE_FRAME ? "frame" : "bytes", data, size);

  err = dev_char_wait_op(device, op, (uint8_t *)data, size);

  if (err < 0) {
    logk("%s: write error: %s", __func__, strerror(err));
  }
}

#define frame_send(dev, data) _send(dev, DEV_CHAR_WRITE_FRAME, (const uint8_t *)(data), sizeof(data)-1)
#define bytes_send(dev, data) _send(dev, DEV_CHAR_WRITE, (const uint8_t *)(data), sizeof(data)-1)

static CONTEXT_ENTRY(hdlc0_tx)
{
  error_t err;
  struct device_char_s hdlc_dev, pipe_dev;

  err = device_get_accessor_by_path(&hdlc_dev.base, NULL, "hdlc0[0]", DRIVER_CLASS_CHAR);
  ensure(!err);

  err = device_get_accessor_by_path(&pipe_dev.base, NULL, "pipe", DRIVER_CLASS_CHAR);
  ensure(!err);
  
  bytes_send(&pipe_dev, "\x7e\x00\x00\x7e");
  bytes_send(&pipe_dev, "\x7e\x00\x00\x00\x7e");
  frame_send(&hdlc_dev, "\x00\x01Some data");
  frame_send(&hdlc_dev, "\x00"); // Short yet valid data
  frame_send(&hdlc_dev, "\x03\x02\x03\x7e\x13\x11\x02\x7f Escaped");
  frame_send(&hdlc_dev, "\x00\x03""012345678901234567890123456789 Too Long");
  frame_send(&hdlc_dev, "\x03\x04Valid");
  frame_send(&hdlc_dev, "\xff\x05To someone else");
  frame_send(&hdlc_dev, "\x00\x06""Also valid");
  frame_send(&hdlc_dev, "\x03\x07""Data");
  frame_send(&hdlc_dev, "\x00\x08""Data");
  frame_send(&hdlc_dev, "\x03\x09""Data");
  // Inject errors in the stream
  bytes_send(&pipe_dev, "\x7e\x00\x0a\x77\x77\x7e");
  bytes_send(&pipe_dev, "\x7e\x00\x0a\x77\x77\x7e");
  bytes_send(&pipe_dev, "\x7e\x00\x0a\x77\x77\x7e");
  bytes_send(&pipe_dev, "\x7e\x00\x0a\x77\x77\x7e");
  frame_send(&hdlc_dev, "\x00\x0a""Data");
}

void app_start(void)
{
  hdlc_rx_start("hdlc1[0]");
  hdlc_rx_start("hdlc1[0]");
  hdlc_rx_start("hdlc1[1]");
  hdlc_rx_start("hdlc1[1]");
  thread_create(hdlc0_tx, NULL, NULL);
}

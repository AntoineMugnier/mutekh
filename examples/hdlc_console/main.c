#include <stdio.h>
#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>
#include <device/class/timer.h>
#include <drivers/char/hdlc.h>

static DRIVER_HDLC_RX_MAP(hdlc0_rx_map, 0x00, 0x01);

DEV_DECLARE_STATIC(hdlc0_dev, "hdlc0", 0, hdlc_drv,
                   DEV_STATIC_RES_DEV_PARAM("io", "/uart*"),
                   DEV_STATIC_RES_BLOB_PARAM("rx_map", hdlc0_rx_map),
                   );

static const uint8_t console0_tx_header[] = { 0x01, 0x03 };

DEV_DECLARE_STATIC(console0, "console0", 0, char_framer_drv,
                   DEV_STATIC_RES_DEV_PARAM("io", "/hdlc0[1]"),
                   DEV_STATIC_RES_UINT_PARAM("header_size", sizeof(console0_tx_header)),
                   DEV_STATIC_RES_BLOB_PARAM("tx_header", console0_tx_header),
                   DEV_STATIC_RES_UINT_PARAM("mtu", 64),
                   );

struct device_timer_s timer;
struct device_char_s console;
struct dev_timer_rq_s timer_rq;
struct dev_timer_rq_s timer_rq2;
struct dev_char_rq_s char_rq;

static KROUTINE_EXEC(delay_done)
{
  printk("Hello from printk\n");
  dev_timer_init_sec(&timer, &timer_rq.delay, 0, 1, 1);
  timer_rq.deadline = 0;
  DEVICE_OP(&timer, request, &timer_rq);
}

static KROUTINE_EXEC(delay2_done)
{
  char_rq.data = "Hello from write\n";
  char_rq.size = strlen("Hello from write\n");
  char_rq.type = DEV_CHAR_WRITE;
  DEVICE_OP(&console, request, &char_rq);
}

static KROUTINE_EXEC(char_done)
{
  dev_timer_init_sec(&timer, &timer_rq2.delay, 0, 1, 1);
  timer_rq2.deadline = 0;
  DEVICE_OP(&timer, request, &timer_rq2);
}

void app_start(void)
{
  error_t err;
  err = device_get_accessor_by_path(&timer.base, NULL, "/timer* /rtc*", DRIVER_CLASS_TIMER);
  if (err) {
    logk_error("Error getting timer device: %d", err);
    return;
  }

  err = device_get_accessor(&console.base, &console0, DRIVER_CLASS_CHAR, 0);
  if (err) {
    logk_error("Error getting console device: %d", err);
    return;
  }
  
  device_start(&timer.base);


  dev_timer_rq_init(&timer_rq, delay_done);
  dev_timer_init_sec(&timer, &timer_rq.delay, 0, 1, 1);
  timer_rq.deadline = 0;
  //  DEVICE_OP(&timer, request, &timer_rq);

  dev_timer_rq_init(&timer_rq2, delay2_done);
  dev_timer_init_sec(&timer, &timer_rq2.delay, 0, 1, 1);
  timer_rq2.deadline = 0;
  //  DEVICE_OP(&timer, request, &timer_rq2);

  dev_char_rq_init(&char_rq, char_done);
  
  printk("Zou\n");
}

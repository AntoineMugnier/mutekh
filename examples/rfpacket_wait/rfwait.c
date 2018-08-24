/*
  This example shows how to access a rfpacket device from a thread
  using a set of blocking helper functions.
*/

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>

#include <device/class/timer.h>
#include <device/class/rfpacket.h>

static const struct dev_rfpacket_pk_cfg_basic_s pkcfg = {
  .base = {
    .format = DEV_RFPACKET_FMT_SLPC,
    .encoding = DEV_RFPACKET_CLEAR,
  },
  .crc = 0x04c11db7,
  .sw_value = 0xabba,
  .sw_len = 15,
  .pb_pattern = 0xa,
  .pb_pattern_len = 1,
  .tx_pb_len = 64,
  .rx_pb_len = 20,
};

static const struct dev_rfpacket_rf_cfg_fsk_s rfcfg =
{
  .base = {
    .mod = DEV_RFPACKET_GFSK,
    .drate = 38400,
    .jam_rssi = (-90) << 3,
    .frequency = 865050000,
    .chan_spacing = 100000,
    .rx_bw = 0,
    .freq_err = 868 * 20 /* ppm */,
  },
  .deviation = 19200,
  .symbols = 2,
};

static CONTEXT_ENTRY(rf_thread)
{
  struct device_rfpacket_s rf_dev;
  struct device_timer_s timer_dev;

  if (device_get_accessor_by_path(&rf_dev.base, NULL, "rf*",
                                  DRIVER_CLASS_RFPACKET))
    {
      logk_error("no rf device");
      return;
    }

  /* use the time base from the rf device */
  if (device_get_accessor(&timer_dev.base, rf_dev.dev,
                          DRIVER_CLASS_TIMER, 0))
    {
      logk_error("no timer device");
      return;
    }

  dev_timer_delay_t msec;
  if (dev_timer_init_sec(&timer_dev, &msec, 0, 1, 1000))
    {
      logk_error("timer config error");
      return;
    }

  /* initialize our blocking rfpacket context */
  struct dev_rfpacket_wait_ctx_s w;
  dev_rfpacket_wait_init(&w, &rf_dev, &rfcfg.base, &pkcfg.base);

  while (1)
    {
      uint8_t tx_buf[] = "transmitted test payload";

      logk("TX ...");

      /* suspend the thread, waiting for packet transmission end */
      dev_rfpacket_wait_tx(&w, tx_buf, sizeof(tx_buf), 0, 8 /* 1 dbm */, 0);

      /* start listening for 5 seconds */
      logk("RX on");
      if (dev_rfpacket_start_rx(&w, 0, msec * 5000))
        {
          logk("Unable to start RX");
          break;
        }

      uint8_t rx_buf[64];
      int count = 10;

      while (1)
        {
          size_t size = sizeof(rx_buf);

          /* suspend the thread, waiting for incoming packets */
          error_t err = dev_rfpacket_wait_rx(&w, rx_buf, &size);

          if (err)
            {
              logk("RX err: %i", err);
              if (err == -EBUSY || err == -ETIMEDOUT)
                break;    /* listen terminated by timeout or error */

              /* other errors do not terminate the RX */
            }
          else
            {
              /* dump the packet content */
              logk("RX (%u bytes) : %P", size, rx_buf, size);
            }

          if (count-- == 0)
            {
              /* early stop listening after some packets */
              dev_rfpacket_stop_rx(&w);
              break;
            }
        }

      logk("RX off");
    }

  dev_rfpacket_wait_cleanup(&w);
}

void app_start()
{
  thread_create(rf_thread, NULL, NULL);
}

#include <mutek/startup.h>
#include <mutek/thread.h>
#include <mutek/printk.h>
#include <stdio.h>

#include <device/class/nfc.h>
#include <device/class/timer.h>

static CONTEXT_ENTRY(nfc_main)
{
  error_t err;
  struct device_timer_s timer;
  dev_timer_delay_t sec;
  dev_timer_delay_t msec;
  struct device_nfc_s nfc;
  struct dev_nfc_rq_s req;
  struct dev_nfc_peer_s peer;
  uint8_t data[16];

  req.peer = &peer;
  peer.side = DEV_NFC_PASSIVE_PICC;
  peer.protocol = DEV_NFC_14443A;

  err = device_get_accessor_by_path(&nfc.base, NULL, "/nfc0", DRIVER_CLASS_NFC);
  assert(!err);

  err = device_get_accessor_by_path(&timer.base, NULL, "/rtc* /timer*", DRIVER_CLASS_TIMER);
  assert(!err);

  dev_timer_init_sec(&timer, &sec, 0, 1, 1);
  dev_timer_init_sec(&timer, &msec, 0, 1, 1000);

  size_t n;

  while (1) {
    req.type = DEV_NFC_POWEROFF;
    dev_nfc_wait_request(&nfc, &req);

    dev_timer_wait_delay(&timer, msec * 200, 0);

    req.type = DEV_NFC_SELECT_ANY;
    err = dev_nfc_wait_request(&nfc, &req);

    printk("UID %P selected err %d\n", peer.uid, peer.uid_size, err);

    if (err)
      continue;

    for (size_t err_count = 0; err_count < 5; err_count++) {
      size_t size;

      dev_timer_wait_delay(&timer, msec * 100, 0);

      if (err_count) {
        req.type = DEV_NFC_SELECT_ANY;
        err = dev_nfc_wait_request(&nfc, &req);

        if (err)
          continue;
      }

      for (uint8_t offset = 0; offset < 1024/8; offset += 16) {
        size = sizeof(data);
        err = dev_nfc_wait_transceive_std(&nfc, &peer,
                                          (const uint8_t []){0x30, offset / 4}, 2,
                                          data, &size);
        if (err) {
          printk("transceive error %d\n", err);
          continue;
        }

        hexdumpk(offset, data, 16);
      }

      err_count = 0;
    }
  }
}

void app_start(void)
{
  struct thread_attr_s attr = {
    .stack_size = 2048,
  };

  thread_create(nfc_main, 0, &attr);
}

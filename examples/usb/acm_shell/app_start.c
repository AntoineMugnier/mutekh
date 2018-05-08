#define LOGK_MODULE_ID "main"

#include <mutek/printk.h>
#include <mutek/startup.h>

#include "usb.h"

void app_start(void)
{
  usb_dev_init();

  logk("ACM Console example");
}

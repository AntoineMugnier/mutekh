#define LOGK_MODULE_ID "usbi"

#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/class/usbdev.h>
#include <device/class/char.h>
#include <device/class/cmu.h>

#include "usb.h"

DEV_DECLARE_STATIC(acm0, "acm0", 0, usbdev_acm_drv,
                   DEV_STATIC_RES_USBDEV_EP_MAP(0, 0x21, 0x01),
                   DEV_STATIC_RES_DEVCLASS_PARAM("usb-ctrl", "/usb", DRIVER_CLASS_USBDEV),
                   DEV_STATIC_RES_STR_PARAM("function", "Console"),
                   );

void usb_dev_init(void)
{
  static const struct usbdev_device_info_s usbdevinfo = {
    .desc = {
      .head.bLength = sizeof(struct usb_device_descriptor_s),
      .head.bDescriptorType = USB_DESC_DEVICE,
      .bcdUSB = endian_le16(CONFIG_USBDEV_USB_REVISION),
      .bDeviceClass = 0,
      .bDeviceSubClass = 0,
      .bDeviceProtocol = 0,
      .bMaxPacketSize0 = 64,
      .idVendor  = endian_le16(0x1234),
      .idProduct = endian_le16(0xdead),
      .bcdDevice = endian_le16(0x0200),
      .iManufacturer = 1,
      .iProduct = 2,
      .iSerialNumber = 0,
      .bNumConfigurations = 1
    },

    .configuration = 0,
    .iconfig = 0,
    .power = 30,
    .str_cnt = 2,
    .string =
    "MutekH\0"
    "Console"
  };

  error_t err;

#ifndef CONFIG_DEVICE_CLOCK_THROTTLE
  {
    struct device_cmu_s clock;
    err = device_get_accessor_by_path(&clock.base, NULL, "recmu", DRIVER_CLASS_CMU);
    ensure(!err);
    /* Use 48MHz clock */
    DEVICE_OP(&clock, app_configid_set, 3);
    device_put_accessor(&clock.base);
  }
#endif

  struct device_usbdev_s usb;

  err = device_get_accessor_by_path(&usb.base, NULL, "usb", DRIVER_CLASS_USBDEV);
  ensure(!err);
  
  /* Attach description to USB device */
  usbdev_stack_set_device_info(&usb, &usbdevinfo);
  device_start(&usb.base);
  device_put_accessor(&usb.base);

  logk("OK");
}

#define LOGK_MODULE_ID "usbi"

#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/class/usbdev.h>
#include <device/class/char.h>
#include <device/class/cmu.h>

#include "usb.h"

DEV_DECLARE_STATIC(dfu0, "dfu0", 0, usbdev_dfu_drv,
                   DEV_STATIC_RES_DEVCLASS_PARAM("usb-ctrl", "/usb", DRIVER_CLASS_USBDEV),
                   DEV_STATIC_RES_DEVCLASS_PARAM("storage", "/mem[1]", DRIVER_CLASS_MEM),
                   DEV_STATIC_RES_MEM(CONFIG_LOAD_ROM_RO_SIZE, 0),
                   );

#define DEVICE_VENDOR "MutekH"
#define DEVICE_PRODUCT "DFU Demo"
#define DEVICE_VID 0x4242
#define DEVICE_PID 0xdead

static char usb_strings[sizeof(DEVICE_VENDOR) + sizeof(DEVICE_PRODUCT) + 32] =
    DEVICE_VENDOR "\0"
    DEVICE_PRODUCT "\0"
    "";

void usb_dev_init(const char *serial_number)
{
  static const struct usbdev_device_info_s usbdevinfo = {
    .desc = {
      .head.bLength = sizeof(struct usb_device_descriptor_s),
      .head.bDescriptorType = USB_DESC_DEVICE,
      .bcdUSB = endian_le16(0x0200),
      .bDeviceClass = 0,
      .bDeviceSubClass = 0,
      .bDeviceProtocol = 0,
      .bMaxPacketSize0 = 64,
      .idVendor  = endian_le16(DEVICE_VID),
      .idProduct = endian_le16(DEVICE_PID),
      .bcdDevice = endian_le16(0x0200),
      .iManufacturer = 1,
      .iProduct = 2,
      .iSerialNumber = 3,
      .bNumConfigurations = 1
    },

    .configuration = 0,
    .iconfig = 0,
    .power = 30,
    .str_cnt = 3,
    .string = usb_strings,
  };

  error_t err;

  strcpy(usb_strings + sizeof(DEVICE_VENDOR) + sizeof(DEVICE_PRODUCT), serial_number);
  
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

  logk("USB OK");
}

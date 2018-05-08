
#include <mutek/printk.h>
#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>
#include <hexo/endian.h>
#include <mutek/startup.h>

#include <device/class/usbdev.h>
#include <device/class/cmu.h>
#include <device/class/gpio.h>

#ifdef CONFIG_ARCH_EFM32
# include <arch/efm32/irq.h>
# include <arch/efm32/pin.h>
#endif


#ifdef CONFIG_ARCH_EFM32

DEV_DECLARE_STATIC(max3420_dev, "max3420", 0, max3420_drv,

  DEV_STATIC_RES_DEV_PARAM("spi", "/spi*"),
  DEV_STATIC_RES_DEV_PARAM("gpio", "/gpio"),
  DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),

  DEV_STATIC_RES_IRQ(0, EFM32_PC5, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
  DEV_STATIC_RES_GPIO("rst",  EFM32_PD6 , 1),
  DEV_STATIC_RES_GPIO("nirq", EFM32_PC5 , 1),
  DEV_STATIC_RES_GPIO("gpx",  EFM32_PC4 , 1),
  DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", EFM32_PD3),

);

DEV_DECLARE_STATIC(usbdev_acm0, "console", 0, usbdev_acm_drv,
                   DEV_STATIC_RES_USBDEV_EP_MAP(0, 0x32, 0x01),
                   DEV_STATIC_RES_DEV_PARAM("usb-ctrl", "/max3420")
);

#else
extern struct device_s usbdev_acm0;
#endif

static const struct usbdev_device_info_s usbdevinfo =
{
  .desc =
    {
      .head.bLength = sizeof(struct usb_device_descriptor_s),
      .head.bDescriptorType = USB_DESC_DEVICE,
      .bcdUSB = endian_le16(CONFIG_USBDEV_USB_REVISION),
      .bDeviceClass = 0,
      .bDeviceSubClass = 0,
      .bDeviceProtocol = 0,
      .bMaxPacketSize0 = 64,
      .idVendor = endian_le16(0x5a5a),
      .idProduct = endian_le16(0),
      .bcdDevice = endian_le16(0),
      .iManufacturer = 1,
      .iProduct = 2,
      .iSerialNumber = 0,
      .bNumConfigurations = 1
    },

  .configuration = 0,
  .iconfig = 0,
  .power = 50,
  .str_cnt = 2,
  .string = "MutekH\0\Test\0"
};

void app_start(void)
{
  struct device_usbdev_s usb;
  if (!device_get_accessor_by_path(&usb.base, NULL, "max3420", DRIVER_CLASS_USBDEV))
    {
      /* Attach description to USB device */
      usbdev_stack_set_device_info(&usb, &usbdevinfo);
      device_start(&usb.base);
      device_put_accessor(&usb.base);
    }
}


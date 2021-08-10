#define LOGK_MODULE_ID "usbi"

#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/class/usbdev.h>
#include <device/class/char.h>
#include <device/class/icu.h>
#include <device/class/iomux.h>
#include <device/class/dma.h>
#include <device/class/cmu.h>
#include <device/resource/uart.h>

#if defined(CONFIG_ARCH_EFM32)
# include <arch/efm32/pin.h>
# include <arch/efm32/clock.h>
# include <arch/efm32/irq.h>
# include <arch/efm32/cmu.h>
# include <arch/efm32/devaddr.h>
# include <arch/efm32/dma_source.h>
#endif

#include "usb.h"

DEV_DECLARE_STATIC(acm0, "acm0", 0, usbdev_acm_drv,
  DEV_STATIC_RES_USBDEV_EP_MAP(0, 0x21, 0x01),
  DEV_STATIC_RES_DEVCLASS_PARAM("usb-ctrl", "/usb", DRIVER_CLASS_USBDEV),
  DEV_STATIC_RES_UART(921600, 8, 0, 1, 0),
#if defined(CONFIG_ARCH_EFM32)
  DEV_STATIC_RES_STR_PARAM("function", "Tx: PD0, Rx: PD1"),
#elif defined(CONFIG_ARCH_NRF5X)
  DEV_STATIC_RES_STR_PARAM("function", "Tx: P0.6, Rx: P0.8"),
#endif
  );

#if defined(CONFIG_ARCH_EFM32)
DEV_DECLARE_STATIC(
  serial0_dev, "serial0", 0,
#if defined(CONFIG_DRIVER_EFM32_USART_CHAR_DMA)
  efm32_usart_dma_drv,
#else
  efm32_usart_drv,
#endif
  DEV_STATIC_RES_MEM(0x4000c400, 0x4000c800),
  DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_USART1, 0),
  
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART1_RX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

#if defined(CONFIG_DRIVER_EFM32_USART_CHAR_DMA)
    DEV_STATIC_RES_DEV_PARAM("dma", "/dma"),
    /* Read channel must have higher priority than write channel */
    DEV_STATIC_RES_DMA((1 << 0), (EFM32_DMA_SOURCE_USART1)),
    DEV_STATIC_RES_DMA((1 << 1), (EFM32_DMA_SOURCE_USART1)),

    /* Timer 2 for RX timeout */
    DEV_STATIC_RES_MEM(0x40010800, 0x40010C00),
    DEV_STATIC_RES_IRQ(1, EFM32_IRQ_TIMER2, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
    DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_TIMER2, 1),
    DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_PRS, 2),
#else
    DEV_STATIC_RES_IRQ(1, EFM32_IRQ_USART1_TX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
#endif
  
  DEV_STATIC_RES_DEV_IOMUX("/gpio"),
  DEV_STATIC_RES_IOMUX("rx", EFM32_LOC1, EFM32_PD1, 0, 0),
  DEV_STATIC_RES_IOMUX("tx", EFM32_LOC1, EFM32_PD0, 0, 0),
 
  DEV_STATIC_RES_UART(115200, 8, 0, 1, 0)
  );
#endif

DEV_DECLARE_STATIC(
  forwarder_dev, "forwarder", 0, uart_forwarder_drv,
  DEV_STATIC_RES_DEV_PARAM("master", "/acm0"),
#if defined(CONFIG_ARCH_EFM32)
  DEV_STATIC_RES_DEV_PARAM("slave", "/serial0"),
#elif defined(CONFIG_ARCH_NRF5X)
  DEV_STATIC_RES_DEV_PARAM("slave", "/uart0"),
#endif


  DEV_STATIC_RES_UINT_PARAM("m_read_size", 16),
  DEV_STATIC_RES_UINT_PARAM("m2s_size", 64),
  DEV_STATIC_RES_UINT_PARAM("s_write_size", 16),

  DEV_STATIC_RES_UINT_PARAM("s_read_size", 64),
  DEV_STATIC_RES_UINT_PARAM("s2m_size", 1024),
  DEV_STATIC_RES_UINT_PARAM("m_write_size", 64),
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
    "Serial Port"
  };

  error_t err;

#if !defined(CONFIG_DEVICE_CLOCK_THROTTLE) && defined(CONFIG_ARCH_EFM32)
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
  struct device_accessor_s forwarder;

  err = device_get_accessor_by_path(&usb.base, NULL, "usb", DRIVER_CLASS_USBDEV);
  ensure(!err);
  
  /* Attach description to USB device */
  usbdev_stack_set_device_info(&usb, &usbdevinfo);
  device_start(&usb.base);
  device_put_accessor(&usb.base);

  /* Start forwarder */
  err = device_get_accessor_by_path(&forwarder, NULL,
                                    "forwarder", DRIVER_CLASS_NONE);
  if (err) {
    logk_error("uart_forwarder error: %d", err);
    return;
  }

  device_start(&forwarder);

  logk("OK");
}

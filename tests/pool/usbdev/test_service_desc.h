#ifndef TEST_SERVICE_DESC_H_
# define TEST_SERVICE_DESC_H_

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/usb/usb.h>
#include <device/class/char.h>
#include <device/class/usbdev.h>
#include <device/class/cmu.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#define USBDEV_SERV_TEST_CONTROL_0_ENDPOINT 1
#define USBDEV_SERV_TEST_BULK_ENDPOINT 1
#define USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT 1
#define USBDEV_SERV_TEST_INTERRUPT_ENDPOINT 1
#define USBDEV_SERV_TEST_CONTROL_N_ENDPOINT 1

#define USBDEV_SERV_TEST_BULK_INTF  0
#define USBDEV_SERV_TEST_ISO_INTF   (USBDEV_SERV_TEST_BULK_INTF + USBDEV_SERV_TEST_BULK_ENDPOINT)
#define USBDEV_SERV_TEST_IRQ_INTF   (USBDEV_SERV_TEST_ISO_INTF + USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT)
#define USBDEV_SERV_TEST_CTRL_INTF  (USBDEV_SERV_TEST_IRQ_INTF + USBDEV_SERV_TEST_INTERRUPT_ENDPOINT)

#define USBDEV_TEST_DEVICE_PATH "usb"
#define USBDEV_TEST_VENDOR_ID 0x5A5A
#define USBDEV_TEST_PRODUCT_ID 0x0000

#define USBDEV_TEST_BULK_BUFFER_SIZE 64

//#define CONFIG_USBDEV_TEST_DEBUG

#ifdef CONFIG_USBDEV_TEST_DEBUG
# define usbdev_test_printk(...) do { printk(__VA_ARGS__); } while(0)
#else
# define usbdev_test_printk(...) do { } while(0)
#endif

#define USBDEV_TEST_EP0_MPS 64

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
      .bMaxPacketSize0 = USBDEV_TEST_EP0_MPS,
      .idVendor = endian_le16(USBDEV_TEST_VENDOR_ID),
      .idProduct = endian_le16(USBDEV_TEST_PRODUCT_ID),
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
  .string =
    "MutekH\0"
    "UsbTestDevice\0"
};

#ifdef USBDEV_SERV_TEST_BULK_ENDPOINT

#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)

#define USBDEV_TEST_BULK_ALT_SIZE 8

static const struct usb_endpoint_descriptor_s ep_in_1 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_IN | 0,
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(USBDEV_TEST_BULK_ALT_SIZE),
  .bInterval = 0
};

static const struct usb_endpoint_descriptor_s ep_out_1 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_OUT | 0,
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(USBDEV_TEST_BULK_ALT_SIZE),
  .bInterval = 0
};

static const struct usbdev_interface_s interface_test_a1 =
{
  .desc = {
    .head.bLength = sizeof(struct usb_interface_descriptor_s),
    .head.bDescriptorType = USB_DESC_INTERFACE,
    .bInterfaceNumber = USBDEV_SERV_TEST_BULK_INTF,
    .bAlternateSetting = 1,
    .bNumEndpoints = 2,
    .bInterfaceClass = 0,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface =2
  },
  USBDEV_ENDPOINT(&ep_in_1, &ep_out_1)
};

#endif

#define USBDEV_TEST_BULK_SIZE 32

static const struct usb_endpoint_descriptor_s ep_in_0 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_IN | 0,
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(USBDEV_TEST_BULK_SIZE),
  .bInterval = 0
};

static const struct usb_endpoint_descriptor_s ep_out_0 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_OUT | 0,
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(USBDEV_TEST_BULK_SIZE),
  .bInterval = 0
};

static const struct usbdev_interface_default_s interface_test_a0 =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = USBDEV_SERV_TEST_BULK_INTF,
      .bAlternateSetting = 0,
      .bNumEndpoints = 2,
      .bInterfaceClass = 0,
      .bInterfaceSubClass = 0,
      .bInterfaceProtocol = 0,
      .iInterface =1
    },
    USBDEV_ENDPOINT(&ep_in_0, &ep_out_0)
  },
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  USBDEV_INTERFACE_ALTERNATE(&interface_test_a1)
#else
  USBDEV_INTERFACE_ALTERNATE()
#endif
};

#endif

#ifdef USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT

#define USB_TEST_ISOCHONOUS_SIZE 128

/* Isochronous endpoint test */

static const struct usb_endpoint_descriptor_s ep_in_2 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_IN | USBDEV_SERV_TEST_BULK_ENDPOINT,
  .bmAttributes = USB_EP_ISOCHRONOUS,
  .wMaxPacketSize = endian_le16((0 << 11) |USB_TEST_ISOCHONOUS_SIZE),
  .bInterval = 1
};

static const struct usb_endpoint_descriptor_s ep_out_2 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_OUT | USBDEV_SERV_TEST_BULK_ENDPOINT,
  .bmAttributes = USB_EP_ISOCHRONOUS,
  .wMaxPacketSize = endian_le16((0 << 11) | USB_TEST_ISOCHONOUS_SIZE),
  .bInterval = 1
};

static const struct usbdev_interface_default_s interface_test_b0 =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = USBDEV_SERV_TEST_ISO_INTF,
      .bAlternateSetting = 0,
      .bNumEndpoints = 2,
      .bInterfaceClass = 0,
      .bInterfaceSubClass = 0,
      .bInterfaceProtocol = 0,
      .iInterface = 3
    },
    USBDEV_ENDPOINT(&ep_in_2, &ep_out_2)
  },
  /* No alternate settings */
  USBDEV_INTERFACE_ALTERNATE()
};

#endif

#ifdef USBDEV_SERV_TEST_INTERRUPT_ENDPOINT

#define USB_TEST_INTERRUPT_SIZE 8

/* Interrupt endpoints test */

static const struct usb_endpoint_descriptor_s ep_in_3 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_IN | (USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT +
                                   USBDEV_SERV_TEST_BULK_ENDPOINT),
  .bmAttributes = USB_EP_INTERRUPT,
  .wMaxPacketSize = endian_le16(USB_TEST_INTERRUPT_SIZE),
  .bInterval = 1
};

static const struct usb_endpoint_descriptor_s ep_out_3 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_OUT | (USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT +
                                    USBDEV_SERV_TEST_BULK_ENDPOINT),
  .bmAttributes = USB_EP_INTERRUPT,
  .wMaxPacketSize = endian_le16(USB_TEST_INTERRUPT_SIZE),
  .bInterval = 1
};

static const struct usbdev_interface_default_s interface_test_c0 =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = USBDEV_SERV_TEST_IRQ_INTF,
      .bAlternateSetting = 0,
      .bNumEndpoints = 2,
      .bInterfaceClass = 0,
      .bInterfaceSubClass = 0,
      .bInterfaceProtocol = 0,
      .iInterface = 4
    },
    USBDEV_ENDPOINT(&ep_in_3, &ep_out_3)
  },
  /* No alternate settings */
  USBDEV_INTERFACE_ALTERNATE()
};

#endif

#ifdef USBDEV_SERV_TEST_CONTROL_N_ENDPOINT

#define USB_TEST_CONTROL_SIZE 32

/* Control N endpoints test */

static const struct usb_endpoint_descriptor_s ep_4 =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = (USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT +
                       USBDEV_SERV_TEST_BULK_ENDPOINT +
                       USBDEV_SERV_TEST_INTERRUPT_ENDPOINT),
  .bmAttributes = USB_EP_CONTROL,
  .wMaxPacketSize = endian_le16(USB_TEST_CONTROL_SIZE),
  .bInterval = 0
};

static const struct usbdev_interface_default_s interface_test_d0 =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = USBDEV_SERV_TEST_CTRL_INTF,
      .bAlternateSetting = 0,
      .bNumEndpoints = 1,
      .bInterfaceClass = 0,
      .bInterfaceSubClass = 0,
      .bInterfaceProtocol = 0,
      .iInterface = 5
    },
    USBDEV_ENDPOINT(&ep_4)
  },
  /* No alternate settings */
  USBDEV_INTERFACE_ALTERNATE()
};

#endif
/* USB test service */
static const struct usbdev_service_descriptor_s usb_test_service =
{
  /* Local descriptor */
  USBDEV_SERVICE_DESCRIPTOR(
#if USBDEV_SERV_TEST_BULK_ENDPOINT
      &interface_test_a0.intf.desc.head,
      &ep_in_0.head,
      &ep_out_0.head,
# if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
      &interface_test_a1.desc.head,
      &ep_in_1.head,
      &ep_out_1.head,
# endif
#endif
#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
      &interface_test_b0.intf.desc.head,
      &ep_in_2.head,
      &ep_out_2.head,
#endif
#if USBDEV_SERV_TEST_INTERRUPT_ENDPOINT
      &interface_test_c0.intf.desc.head,
      &ep_in_3.head,
      &ep_out_3.head,
#endif
#if USBDEV_SERV_TEST_CONTROL_N_ENDPOINT
      &interface_test_d0.intf.desc.head,
      &ep_4.head,
#endif
    ),

  USBDEV_INTERFACE(
#if USBDEV_SERV_TEST_BULK_ENDPOINT
    &interface_test_a0,
#endif
#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
    &interface_test_b0,
#endif
#if USBDEV_SERV_TEST_INTERRUPT_ENDPOINT
    &interface_test_c0,
#endif
#if USBDEV_SERV_TEST_CONTROL_N_ENDPOINT
    &interface_test_d0,
#endif
    ),
  .str_cnt = 5,
  .string =
    "Bulk interface\0"
    "Bulk alternate interface\0"
    "Isochronous interfaxe\0"
    "Interrupt interface\0"
    "Control interface\0",
};

#endif /* !TEST_SERVICE_DESC_H_ */

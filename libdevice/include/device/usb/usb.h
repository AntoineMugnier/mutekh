/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2016

*/

/**
   @file
   @module {Core::Devices support library}
   @short USB spec related definitions
*/

#ifndef __USB_H__
#define __USB_H__

#include <hexo/endian.h>
#include <hexo/enum.h>

enum usb_configuration_parameter_e
{
  USB_SELF_POWERED  = 1 << 6,
  USB_REMOTE_WAKEUP = 1 << 5,
};

enum usb_feature_selector_e
{
  USB_ENDPOINT_HALT,
  USB_DEVICE_REMOTE_WAKEUP,
  USB_TEST_MODE,
};

enum usb_endpoint_type_e
{
  USB_EP_CONTROL,
  USB_EP_ISOCHRONOUS,
  USB_EP_BULK,
  USB_EP_INTERRUPT,
};

enum usb_control_status_e
{
  USB_CTRL_STATUS_ACK,
  USB_CTRL_STATUS_STALL,
  USB_CTRL_STATUS_NACK,
};

enum usb_endpoint_dir_e
{
  USB_EP_OUT  = 0,
  USB_EP_IN   = 0x80
};

enum usb_descriptor_type_e
{
  USB_DESC_DEVICE                = 1,
  USB_DESC_CONFIGURATION         = 2,
  USB_DESC_STRING                = 3,
  USB_DESC_INTERFACE             = 4,
  USB_DESC_ENDPOINT              = 5,
  USB_DESC_DEVICE_QUALIFIER      = 6,
  USB_DESC_OTHER_SPEED_CONFIG    = 7, 
  USB_DESC_INTERFACE_POWER       = 8,
  USB_DESC_OTG                   = 9,
  USB_DESC_DEBUG                 = 10,
  USB_DESC_INTERFACE_ASSOCIATION = 11,
};

enum usb_device_class_code_e
{
  USB_DEV_CLASS_SEE_INTERFACE        = 0x00,
  USB_DEV_CLASS_AUDIO                = 0x01,
  USB_DEV_CLASS_COMM                 = 0x02,
  USB_DEV_CLASS_HID                  = 0x03,
  USB_DEV_CLASS_PHYSICAL             = 0x05,
  USB_DEV_CLASS_IMAGE                = 0x06,
  USB_DEV_CLASS_PRINTER              = 0x07,
  USB_DEV_CLASS_MASS_STORAGE         = 0x08,
  USB_DEV_CLASS_HUB                  = 0x09,
  USB_DEV_CLASS_COMM_DATA            = 0x0A,
  USB_DEV_CLASS_SMART_CARD           = 0x0B,
  USB_DEV_CLASS_CONTENT_SECURITY     = 0x0D,
  USB_DEV_CLASS_VIDEO                = 0x0E,
  USB_DEV_CLASS_PERSONAL_HEALTHCARE  = 0x0F,
  USB_DEV_CLASS_AUDIO_VIDEO          = 0x10,
  USB_DEV_CLASS_BILLBOARD            = 0x11,
  USB_DEV_CLASS_TYPEC_BRIDGE         = 0x12,
  USB_DEV_CLASS_DIAGNOSTIC           = 0xDC,
  USB_DEV_CLASS_WIRELESS             = 0xE0,
  USB_DEV_CLASS_MISCELLANEOUS        = 0xEF,
  USB_DEV_CLASS_APPLICATION_SPECIFIC = 0xFE,
  USB_DEV_CLASS_VENDOR_SPECIFIC      = 0xFF,
};

enum usb_device_misc_subclass_e
{
  USB_DEV_MISC_SUBCLASS_SYNC  = 0x01,
  USB_DEV_MISC_SUBCLASS_MULTI = 0x02,
  USB_DEV_MISC_SUBCLASS_CABLE = 0x03,
  USB_DEV_MISC_SUBCLASS_RNDIS = 0x04,
  USB_DEV_MISC_SUBCLASS_USB3  = 0x05,
  USB_DEV_MISC_SUBCLASS_STEP  = 0x06,
  USB_DEV_MISC_SUBCLASS_CICAM = 0x07,
};

enum usb_device_application_subclass_e
{
  USB_DEV_APP_SUBCLASS_DFU  = 0x01,
  USB_DEV_APP_SUBCLASS_IRDA = 0x02,
  USB_DEV_APP_SUBCLASS_TEST = 0x03,
};

enum usb_device_misc_multi_protocol_e
{
  USB_DEV_MISC_MULTI_PROTO_IAD  = 0x01,
  USB_DEV_MISC_MULTI_PROTO_WAMP = 0x02,
};

ENUM_DESCRIPTOR(dev_usbdev_state_e, strip:DEV_USBDEV_, upper);

enum dev_usbdev_state_e
{
/** Detached */
  DEV_USBDEV_DETACHED,
/** Attached */
  DEV_USBDEV_ATTACHED,
/** Attached and powered */
  DEV_USBDEV_POWERED,
/** Reset is on-going */
  DEV_USBDEV_POWERED_TO_DEFAULT,
  DEV_USBDEV_WAIT_SERIVCE_READY,
/** Respond to default address */
  DEV_USBDEV_DEFAULT,
/** Unique address assigned */
  DEV_USBDEV_ADDRESS,
  DEV_USBDEV_ADDRESS_TO_DEFAULT,
  DEV_USBDEV_ADDRESS_TO_DETACHED,
  DEV_USBDEV_ADDRESS_TO_POWERED,
/** Configuration done */
  DEV_USBDEV_CONFIGURED,
  DEV_USBDEV_CONFIGURED_TO_DETACHED,
  DEV_USBDEV_CONFIGURED_TO_POWERED,
  DEV_USBDEV_CONFIGURED_TO_DEFAULT,
/** No bus traffic for at least 3 ms */
  DEV_USBDEV_SUSPENDED,
};

struct usb_descriptor_header_s
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
}__attribute__((packed));

struct usb_class_descriptor_header_s
{
  struct usb_descriptor_header_s head;
  uint8_t  bDescriptorSubtype;
};

struct usb_string_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint16_t wData[0];
}__attribute__((packed));

struct usb_endpoint_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t  bEndpointAddress;
  uint8_t  bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t  bInterval;
}__attribute__((packed));

ALWAYS_INLINE
uint8_t usb_ep_dir_get(const struct usb_endpoint_descriptor_s *ep)
{
  return ep->bEndpointAddress & 0x80;
}

ALWAYS_INLINE
uint8_t usb_ep_num_get(const struct usb_endpoint_descriptor_s *ep)
{
  return ep->bEndpointAddress & 0xf;
}

ALWAYS_INLINE
uint16_t usb_ep_size_get(const struct usb_endpoint_descriptor_s *ep)
{
  return endian_le16(ep->wMaxPacketSize);
}

ALWAYS_INLINE
uint16_t usb_ep_mps_get(const struct usb_endpoint_descriptor_s *ep)
{
  return usb_ep_size_get(ep) & 0x7ff;
}

ALWAYS_INLINE
enum usb_endpoint_type_e usb_ep_type_get(const struct usb_endpoint_descriptor_s *ep)
{
  return ep->bmAttributes & 0x3;
}

ALWAYS_INLINE
uint8_t usb_ep_interval_get(const struct usb_endpoint_descriptor_s *ep)
{
  return ep->bInterval;
}

struct usb_interface_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t  bInterfaceNumber;
  uint8_t  bAlternateSetting;
  uint8_t  bNumEndpoints;
  uint8_t  bInterfaceClass;
  uint8_t  bInterfaceSubClass;
  uint8_t  bInterfaceProtocol;
  uint8_t  iInterface;
}__attribute__((packed));

ALWAYS_INLINE
uint8_t usb_interface_ep_count_get(const struct usb_interface_descriptor_s *intf)
{
  return intf->bNumEndpoints;
}

ALWAYS_INLINE
uint8_t usb_interface_alt_get(const struct usb_interface_descriptor_s *intf)
{
  return intf->bAlternateSetting;
}

ALWAYS_INLINE
uint8_t usb_interface_number_get(const struct usb_interface_descriptor_s *intf)
{
  return intf->bInterfaceNumber;
}

struct usb_configuration_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint16_t wTotalLength;
  uint8_t  bNumInterfaces;
  uint8_t  bConfigurationValue;
  uint8_t  iConfiguration;
  uint8_t  bmAttributes;
  uint8_t  bMaxPower;
}__attribute__((packed));

// See IAD
struct usb_interface_association_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t  bFirstInterface;
  uint8_t  bInterfaceCount;
  uint8_t  bFunctionClass;
  uint8_t  bFunctionSubClass;
  uint8_t  bFunctionProtocol;
  uint8_t  iFunction;
}__attribute__((packed));

/** String descriptor for LANGID */
struct usb_string_descriptor_zero_s
{
  struct usb_descriptor_header_s head;
  uint16_t wlangid[0];
};

struct usb_device_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint16_t bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t  iManufacturer;
  uint8_t  iProduct;
  uint8_t  iSerialNumber;
  uint8_t  bNumConfigurations;
}__attribute__((packed));


struct usb_ctrl_setup_s
{
  uint8_t  bRequestType;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
}__attribute__((packed));

enum usb_standard_request_e
{
  USB_GET_STATUS        = 0,
  USB_CLEAR_FEATURE     = 1,
  USB_SET_FEATURE       = 3,
  USB_SET_ADDRESS       = 5,
  USB_GET_DESCRIPTOR    = 6,
  USB_SET_DESCRIPTOR    = 7,
  USB_GET_CONFIGURATION = 8,
  USB_SET_CONFIGURATION = 9,
  USB_GET_INTERFACE     = 10,
  USB_SET_INTERFACE     = 11,
  USB_SYNCH_FRAME       = 12,
};

ALWAYS_INLINE
bool_t usb_ep_is_in(const struct usb_endpoint_descriptor_s *desc)
{
  return usb_ep_type_get(desc) == USB_EP_CONTROL
    || usb_ep_dir_get(desc) == USB_EP_IN;
}

ALWAYS_INLINE
bool_t usb_ep_is_out(const struct usb_endpoint_descriptor_s *desc)
{
  return usb_ep_type_get(desc) == USB_EP_CONTROL
    || usb_ep_dir_get(desc) == USB_EP_OUT;
}

enum usb_transfert_direction_e
{
  USB_HOST_TO_DEVICE,
  USB_DEVICE_TO_HOST,
};

enum usb_tranfert_type_e
{
  USB_STANDARD,
  USB_CLASS,
  USB_VENDOR,
};

enum usb_tranfert_recipient_e
{
  USB_DEVICE,
  USB_INTERFACE,
  USB_ENDPOINT,
  USB_OTHER,
};

ALWAYS_INLINE
uint8_t usb_setup_reqtype_get(const struct usb_ctrl_setup_s *s)
{
  return s->bRequestType;
}

ALWAYS_INLINE
enum usb_tranfert_type_e usb_setup_type_get(const struct usb_ctrl_setup_s *s)
{
  return (s->bRequestType >> 5) & 3;
}

ALWAYS_INLINE
enum usb_tranfert_recipient_e usb_setup_recipient_get(const struct usb_ctrl_setup_s *s)
{
  return s->bRequestType & 0x1F;
}

ALWAYS_INLINE
enum usb_transfert_direction_e usb_setup_direction_get(const struct usb_ctrl_setup_s *s)
{
  return (s->bRequestType >> 7) & 1;
}

ALWAYS_INLINE
uint8_t usb_setup_request_get(const struct usb_ctrl_setup_s *s)
{
  return s->bRequest;
}

ALWAYS_INLINE
uint16_t usb_setup_value_get(const struct usb_ctrl_setup_s *s)
{
  return endian_le16(s->wValue);
}

ALWAYS_INLINE
uint16_t usb_setup_index_get(const struct usb_ctrl_setup_s *s)
{
  return endian_le16(s->wIndex);
}

ALWAYS_INLINE
uint16_t usb_setup_length_get(const struct usb_ctrl_setup_s *s)
{
  return endian_le16(s->wLength);
}

#define USBDEV_DESC_CONFIGURATION(length, ifcount, id, icfg, attr, power)    \
  .desc = (const uint8_t[]){                                                 \
    sizeof(struct usb_config_descriptor_s),                                  \
    USB_CONFIG_DESCRIPTOR,                                                   \
    (length) & 0xFF,                                                         \
    ((length) >> 8) & 0xFF,                                                  \
    ifcount,                                                                 \
    icfg,                                                                    \
    attr,                                                                    \
    power                                                                    \
  }

#define USBDEV_DESC_DEVICE(version, class, subclass, prot, packetsize, vendor,   \
                           product, release, imanufacturer, iproduct, iserial,   \
                           cfgcount)                                             \
  .desc = (const uint8_t[]){                  \
    sizeof(struct usb_device_descriptor_s),   \
    USB_DEVICE_DESCRIPTOR,                    \
    (version) & 0xFF,                         \
    ((version) >> 8) & 0xFF,                  \
    class,                                    \
    subclass,                                 \
    prot,                                     \
    packetsize,                               \
    (vendor) & 0xFF,                          \
    ((vendor) >> 8) & 0xFF,                   \
    (product) & 0xFF,                         \
    ((product) >> 8) & 0xFF,                  \
    (release) & 0xFF,                         \
    ((release) >> 8) & 0xFF,                  \
    imanufacturer,                            \
    iproduct,                                 \
    iserial,                                  \
    cfgcount                                  \
   }
   
#define USBDEV_STRINGS(...)   \
   .strings = ((const uint16_t *[]){__VA_ARGS__}),  \
   .strcount = sizeof((uint16_t *[]){__VA_ARGS__})                

#define USBDEV_CONFIGURATIONS(...)  \
   .config = ((const struct usbdev_configuration_s *[]){__VA_ARGS__}),  \
   .cfgcount = sizeof((const struct usbdev_configuration_s *[]){__VA_ARGS__})

#define USBDEV_LANGID(code)   \
  (code) & 0xFF,              \
  ((code) >> 8) & 0xFF

#define USBDEV_LANGID_TABLE(...)   \
   .langid = (const uint8_t []){2 + sizeof((uint8_t []){__VA_ARGS__}), USB_STRING_DESCRIPTOR, __VA_ARGS__}

#endif

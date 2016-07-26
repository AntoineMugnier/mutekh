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
  USB_DEVICE_DESCRIPTOR              = 1,
  USB_CONFIGURATION_DESCRIPTOR       = 2,
  USB_STRING_DESCRIPTOR              = 3,
  USB_INTERFACE_DESCRIPTOR           = 4,
  USB_ENDPOINT_DESCRIPTOR            = 5,
  USB_DEVICE_QUALIFIER_DESCRIPTOR    = 6,
  USB_OTHER_SPEED_CONFIG_DESCRIPTOR  = 7, 
  USB_INTERFACE_POWER_DESCRIPTOR     = 8
};

enum usb_device_class_code_e
{
  /** CDC */
  USB_CLASS_CDC                  = 0x02,
  USB_CLASS_HID                  = 0x03,
  USB_CLASS_HUB                  = 0x09,  
  USB_CLASS_MSD                  = 0x08,  
  USB_CLASS_CDC_DATA             = 0x0A,
};

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

#define USB_GET_EDP_DIR(desc)  ((desc)->bEndpointAddress & 0x80)
#define USB_GET_EDP_NUM(desc)  ((desc)->bEndpointAddress & 0x0F)
#define USB_GET_EDP_SIZE(desc) (endian_le16((desc)->wMaxPacketSize))
#define USB_GET_EDP_TYPE(desc) ((desc)->bmAttributes & 0x3)
#define USB_GET_EDP_INTV(desc) ((desc)->bInterval)

#define USB_GET_EDP_MPS(desc)  (USB_GET_EDP_SIZE(desc) & 0x7FF)

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

#define USB_GET_ITF_EP_CNT(desc)  ((desc)->bNumEndpoints)
#define USB_GET_ITF_ALT(desc)     ((desc)->bAlternateSetting)
#define USB_GET_ITF_NUM(desc)     ((desc)->bInterfaceNumber)

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

ALWAYS_INLINE bool_t
usbdev_is_endpoint_in(const struct usb_endpoint_descriptor_s *desc)
{
  return (USB_GET_EDP_TYPE(desc) == USB_EP_CONTROL || USB_GET_EDP_DIR(desc) == USB_EP_IN);
}

ALWAYS_INLINE bool_t
usbdev_is_endpoint_out(const struct usb_endpoint_descriptor_s *desc)
{
  return (USB_GET_EDP_TYPE(desc) == USB_EP_CONTROL || USB_GET_EDP_DIR(desc) == USB_EP_OUT);
}

#define USB_REQUEST_SETUP_CAST(s)    ((struct usb_ctrl_setup_s *)(s))
#define USB_REQUEST_REQTYPE_GET(s)   ((USB_REQUEST_SETUP_CAST(s)->bRequestType))
#define USB_REQUEST_TYPE_GET(s)      ((USB_REQUEST_SETUP_CAST(s)->bRequestType) >> 5  & 3)
#define USB_REQUEST_RECIPIENT_GET(s) ((USB_REQUEST_SETUP_CAST(s)->bRequestType) >> 0  & 0x1F)
#define USB_REQUEST_DIRECTION_GET(s) ((USB_REQUEST_SETUP_CAST(s)->bRequestType) >> 7  & 1)
#define USB_REQUEST_REQUEST_GET(s)   (USB_REQUEST_SETUP_CAST(s)->bRequest)
#define USB_REQUEST_VALUE_GET(s)     (endian_le16(USB_REQUEST_SETUP_CAST(s)->wValue))
#define USB_REQUEST_INDEX_GET(s)     (endian_le16(USB_REQUEST_SETUP_CAST(s)->wIndex))
#define USB_REQUEST_LENGTH_GET(s)    (endian_le16(USB_REQUEST_SETUP_CAST(s)->wLength))

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

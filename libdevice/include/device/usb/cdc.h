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
   @short USB CDC class spec related definitions
*/

#ifndef __USB_CDC_H__
#define __USB_CDC_H__

#include <device/usb/usb.h>
#include <device/class/usbdev.h>

enum usb_cdc_subclass_e
{
  /* Direct Line Control Model */
  USB_CDC_SUBCLASS_DCM              = 0x01,
  /* Abstract Control Model */
  USB_CDC_SUBCLASS_ACM              = 0x02,
  /* Telephone Control Model */
  USB_CDC_SUBCLASS_TCM              = 0x03,
  /* Multi-Channel Control Model */
  USB_CDC_SUBCLASS_CCM              = 0x04,
  /* CAPI Control Model */
  USB_CDC_SUBCLASS_CAPI             = 0x05,
  /* Ethernet Networking Control Mode */
  USB_CDC_SUBCLASS_ECM              = 0x06,
  /* ATM Networking Control Model */
  USB_CDC_SUBCLASS_ATM              = 0x07,
  /* Wireless Handset Control Model */ 
  USB_CDC_SUBCLASS_WCM              = 0x08,
  /* Device Management */
  USB_CDC_SUBCLASS_DEVICE           = 0x09,
  /* Mobile Direct Line Model */
  USB_CDC_SUBCLASS_MOBILE           = 0x0A,
  /* OBEX */
  USB_CDC_SUBCLASS_OBEX             = 0x0B,
  /* Ethernet Emulation Model */
  USB_CDC_SUBCLASS_EEM              = 0x0C,
  /* Network Control Model */
  USB_CDC_SUBCLASS_NCM              = 0x0D,
};

enum usb_cdc_protocol_e
{
  /* No class specific protocol required */
  USB_CDC_PROTOCOL_NONE             = 0x00,
  /* AT Commands: V.250 etc */
  USB_CDC_PROTOCOL_AT               = 0x01,
  /* AT Commands defined by PCCA-101 */
  USB_CDC_PROTOCOL_ATPCCA           = 0x02,
  /* AT Commands defined by PCCA-101 & Annex O */
  USB_CDC_PROTOCOL_ATPCCAO          = 0x03,
  /* AT Commands defined by GSM 07.07 */
  USB_CDC_PROTOCOL_ATGSM            = 0x04,
  /* AT Commands defined by 3GPP 27.007 */
  USB_CDC_PROTOCOL_AT3GPP           = 0x05,
  /* AT Commands defined by TIA for CDMA */
  USB_CDC_PROTOCOL_ATCDMA           = 0x06,
  /* Ethernet Emulation Model */ 
  USB_CDC_PROTOCOL_EEM              = 0x07,
  /* External Protocol: Commands defined by Command Set Functional Descriptor */
  USB_CDC_PROTOCOL_EXT              = 0xFE,
  /* Vendor-specific */
  USB_CDC_PROTOCOL_VENDOR           = 0xFF,
};

/* Fonctionnal descriptor */

enum usb_functional_desc_subtype_e
{
  USB_CDC_FUNC_HEADER               = 0x00,
  USB_CDC_FUNC_CALL_MGMT            = 0x01,
  USB_CDC_FUNC_ACM                  = 0x02,
  USB_CDC_FUNC_DLM                  = 0x03,
  USB_CDC_FUNC_TEL_RING             = 0x04,
  USB_CDC_FUNC_TEL_CALL             = 0x05,
  USB_CDC_FUNC_UNION                = 0x06,
  USB_CDC_FUNC_COUNTRY_SEL          = 0x07,
  USB_CDC_FUNC_TEL_OP_MODE          = 0x08,
  USB_CDC_FUNC_USB_TERM             = 0x09,
  USB_CDC_FUNC_NETWORK              = 0x0a,
  USB_CDC_FUNC_PROTOCOL_UNIT        = 0x0b,
  USB_CDC_FUNC_EXTENSION_UNIT       = 0x0c,
  USB_CDC_FUNC_CHANNEL_MGMT         = 0x0d,
  USB_CDC_FUNC_CAPI                 = 0x0e,
  USB_CDC_FUNC_ETHERNET             = 0x0f,
  USB_CDC_FUNC_ATM                  = 0x10,
};

enum usb_cdc_fctl_desc_type_e
{
  USB_CDC_INTERFACE_DESCRIPTOR = 0x24,
  USB_CDC_ENDPOINT_DESCRIPTOR  = 0x25
};

struct usb_cdc_header_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t  bDescriptorSubtype;
  uint16_t bcdCDC;
}__attribute__((packed));

struct usb_cdc_call_mgmt_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t bDescriptorSubtype;
  uint8_t bmCapabilities;
  uint8_t bDataInterface;
}__attribute__((packed));
  
struct usb_cdc_acm_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t bDescriptorSubtype;
  uint8_t bmCapabilities;
}__attribute__((packed));

struct usb_cdc_union_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t bDescriptorSubtype;
  uint8_t bMasterInterface;
  uint8_t bSlaveInterface;
}__attribute__((packed));

struct usbdev_class_cdc_func_info_s
{
  usbdev_replace_t * f_replace;

  union{
    struct usb_cdc_union_descriptor_s un;
    struct usb_cdc_header_descriptor_s hdr;
    struct usb_cdc_call_mgmt_descriptor_s call;
    struct usb_cdc_acm_descriptor_s acm;
  };
};

config_depend(CONFIG_DEVICE_USBDEV)
USBDEV_REPLACE(usbdev_cdc_desc_update);

enum usb_cdc_request_code_e
{
  USB_CDC_SET_LINE_CODING          = 0x20,
  USB_CDC_GET_LINE_CODING          = 0x21,
  USB_CDC_SET_CONTROL_LINE_STATE   = 0x22,
};

struct usbdev_cdc_line_coding_s
{
  uint32_t dwDTERate;
  uint8_t bCharFormat;
  uint8_t bParityType;
  uint8_t bDataBits;
}__attribute__((packed));


#endif

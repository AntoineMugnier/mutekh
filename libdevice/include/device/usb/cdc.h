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

enum usb_cdc_desc_functional_subtype_e
{
  USB_CDC_DESC_FUNC_HEADER               = 0x00,
  USB_CDC_DESC_FUNC_CALL_MGMT            = 0x01,
  USB_CDC_DESC_FUNC_ACM                  = 0x02,
  USB_CDC_DESC_FUNC_DLM                  = 0x03,
  USB_CDC_DESC_FUNC_TEL_RING             = 0x04,
  USB_CDC_DESC_FUNC_TEL_CALL             = 0x05,
  USB_CDC_DESC_FUNC_UNION                = 0x06,
  USB_CDC_DESC_FUNC_COUNTRY_SEL          = 0x07,
  USB_CDC_DESC_FUNC_TEL_OP_MODE          = 0x08,
  USB_CDC_DESC_FUNC_USB_TERM             = 0x09,
  USB_CDC_DESC_FUNC_NETWORK              = 0x0a,
  USB_CDC_DESC_FUNC_PROTOCOL_UNIT        = 0x0b,
  USB_CDC_DESC_FUNC_EXTENSION_UNIT       = 0x0c,
  USB_CDC_DESC_FUNC_CHANNEL_MGMT         = 0x0d,
  USB_CDC_DESC_FUNC_CAPI                 = 0x0e,
  USB_CDC_DESC_FUNC_ETHERNET             = 0x0f,
  USB_CDC_DESC_FUNC_ATM                  = 0x10,
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

#define USB_CDC_UNION_DESCRIPTOR_LEN(x) (sizeof(struct usb_cdc_union_descriptor_s) + (x))

struct usb_cdc_union_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t bDescriptorSubtype;
  uint8_t bControlInterface;
  uint8_t bSubordinateInterface;
}__attribute__((packed));

enum usb_cdc_request_code_e
{
  /* Generic */
  USB_CDC_SEND_ENCAPSULATED_COMMAND                    = 0x00,
  USB_CDC_GET_ENCAPSULATED_RESPONSE                    = 0x01,

  /* PSTN */
  USB_CDC_SET_COMM_FEATURE                             = 0x02,
  USB_CDC_GET_COMM_FEATURE                             = 0x03,
  USB_CDC_CLEAR_COMM_FEATURE                           = 0x04,
  USB_CDC_SET_AUX_LINE_STATE                           = 0x10,
  USB_CDC_SET_HOOK_STATE                               = 0x11,
  USB_CDC_PULSE_SETUP                                  = 0x12,
  USB_CDC_SEND_PULSE                                   = 0x13,
  USB_CDC_SET_PULSE_TIME                               = 0x14,
  USB_CDC_RING_AUX_JACK                                = 0x15,
  USB_CDC_SET_LINE_CODING                              = 0x20,
  USB_CDC_GET_LINE_CODING                              = 0x21,
  USB_CDC_SET_CONTROL_LINE_STATE                       = 0x22,
  USB_CDC_SEND_BREAK                                   = 0x23,
  USB_CDC_SET_RINGER_PARMS                             = 0x30,
  USB_CDC_GET_RINGER_PARMS                             = 0x31,
  USB_CDC_SET_OPERATION_PARMS                          = 0x32,
  USB_CDC_GET_OPERATION_PARMS                          = 0x33,
  USB_CDC_SET_LINE_PARMS                               = 0x34,
  USB_CDC_GET_LINE_PARMS                               = 0x35,
  USB_CDC_DIAL_DIGITS                                  = 0x36,

  /* ISDN */
  USB_CDC_SET_UNIT_PARAMETER                           = 0x37,
  USB_CDC_GET_UNIT_PARAMETER                           = 0x38,
  USB_CDC_CLEAR_UNIT_PARAMETER                         = 0x39,
  USB_CDC_GET_PROFILE                                  = 0x3A,

  /* ECM */
  USB_CDC_SET_ETHERNET_MULTICAST_FILTERS               = 0x40,
  USB_CDC_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER = 0x41,
  USB_CDC_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER = 0x42,
  USB_CDC_SET_ETHERNET_PACKET_FILTER                   = 0x43,
  USB_CDC_GET_ETHERNET_STATISTIC                       = 0x44,

  /* ATM */
  USB_CDC_SET_ATM_DATA_FORMAT                          = 0x50,
  USB_CDC_GET_ATM_DEVICE_STATISTICS                    = 0x51,
  USB_CDC_SET_ATM_DEFAULT_VC                           = 0x52,
  USB_CDC_GET_ATM_VC_STATISTICS                        = 0x53,

  /* NCM */
  USB_CDC_GET_NTB_PARAMETERS                           = 0x80,
  USB_CDC_GET_NET_ADDRESS                              = 0x81,
  USB_CDC_SET_NET_ADDRESS                              = 0x82,
  USB_CDC_GET_NTB_FORMAT                               = 0x83,
  USB_CDC_SET_NTB_FORMAT                               = 0x84,
  USB_CDC_GET_NTB_INPUT_SIZE                           = 0x85,
  USB_CDC_SET_NTB_INPUT_SIZE                           = 0x86,
  USB_CDC_GET_MAX_DATAGRAM_SIZE                        = 0x87,
  USB_CDC_SET_MAX_DATAGRAM_SIZE                        = 0x88,
  USB_CDC_GET_CRC_MODE                                 = 0x89,
  USB_CDC_SET_CRC_MODE                                 = 0x8A,
};

enum usb_cdc_notification_e
{
  /* Generic */
  USB_CDC_NETWORK_CONNECTION      = 0x00,
  USB_CDC_RESPONSE_AVAILABLE      = 0x01,

  /* PSTN */
  USB_CDC_AUX_JACK_HOOK_STATE     = 0x08,
  USB_CDC_RING_DETECT             = 0x09,
  USB_CDC_SERIAL_STATE            = 0x20,
  USB_CDC_CALL_STATE_CHANGE       = 0x28,
  USB_CDC_LINE_STATE_CHANGE       = 0x29,

  /* Generic */
  USB_CDC_CONNECTION_SPEED_CHANGE = 0x2A,
};

/*
  PSTN/ACM Subclass
 */

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

struct usbdev_cdc_line_coding_s
{
  uint32_t dwDTERate;
  uint8_t bCharFormat;
  uint8_t bParityType;
  uint8_t bDataBits;
}__attribute__((packed));

/*
  ECM Subclass
 */

struct usb_cdc_ecm_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t bDescriptorSubtype;
  uint8_t iMACAddress;
  uint32_t bmEthernetStatistics;
  uint16_t wMaxSegmentSize;
  uint16_t wNumberMCFilters;
  uint8_t bNumberPowerFilters;
}__attribute__((packed));

enum usb_cdc_ecm_statistic_e
{
  USB_CDC_ECM_STAT_XMIT_OK                = 0x01,
  USB_CDC_ECM_STAT_RCV_OK                 = 0x02,
  USB_CDC_ECM_STAT_XMIT_ERROR             = 0x03,
  USB_CDC_ECM_STAT_RCV_ERROR              = 0x04,
  USB_CDC_ECM_STAT_RCV_NO_BUFFER          = 0x05,
  USB_CDC_ECM_STAT_DIRECTED_BYTES_XMIT    = 0x06,
  USB_CDC_ECM_STAT_DIRECTED_FRAMES_XMIT   = 0x07,
  USB_CDC_ECM_STAT_MULTICAST_BYTES_XMIT   = 0x08,
  USB_CDC_ECM_STAT_MULTICAST_FRAMES_XMIT  = 0x09,
  USB_CDC_ECM_STAT_BROADCAST_BYTES_XMIT   = 0x0A,
  USB_CDC_ECM_STAT_BROADCAST_FRAMES_XMIT  = 0x0B,
  USB_CDC_ECM_STAT_DIRECTED_BYTES_RCV     = 0x0C,
  USB_CDC_ECM_STAT_DIRECTED_FRAMES_RCV    = 0x0D,
  USB_CDC_ECM_STAT_MULTICAST_BYTES_RCV    = 0x0E,
  USB_CDC_ECM_STAT_MULTICAST_FRAMES_RCV   = 0x0F,
  USB_CDC_ECM_STAT_BROADCAST_BYTES_RCV    = 0x10,
  USB_CDC_ECM_STAT_BROADCAST_FRAMES_RCV   = 0x11,
  USB_CDC_ECM_STAT_RCV_CRC_ERROR          = 0x12,
  USB_CDC_ECM_STAT_TRANSMIT_QUEUE_LENGTH  = 0x13,
  USB_CDC_ECM_STAT_RCV_ERROR_ALIGNMENT    = 0x14,
  USB_CDC_ECM_STAT_XMIT_ONE_COLLISION     = 0x15,
  USB_CDC_ECM_STAT_XMIT_MORE_COLLISIONS   = 0x16,
  USB_CDC_ECM_STAT_XMIT_DEFERRED          = 0x17,
  USB_CDC_ECM_STAT_XMIT_MAX_COLLISIONS    = 0x18,
  USB_CDC_ECM_STAT_RCV_OVERRUN            = 0x19,
  USB_CDC_ECM_STAT_XMIT_UNDERRUN          = 0x1A,
  USB_CDC_ECM_STAT_XMIT_HEARTBEAT_FAILURE = 0x1B,
  USB_CDC_ECM_STAT_XMIT_TIMES_CRS_LOST    = 0x1C,
  USB_CDC_ECM_STAT_XMIT_LATE_COLLISIONS   = 0x1D,
};

#define USB_CDC_ECM_STAT_CAP(x) (1 << ((USB_CDC_ECM_STAT_ ## x) - 1))

enum usb_cdc_ecm_filter_e
{
  USB_CDC_ECM_PACKET_TYPE_PROMISCUOUS   = (1 << 0),
  USB_CDC_ECM_PACKET_TYPE_ALL_MULTICAST = (1 << 1),
  USB_CDC_ECM_PACKET_TYPE_DIRECTED      = (1 << 2),
  USB_CDC_ECM_PACKET_TYPE_BROADCAST     = (1 << 3),
  USB_CDC_ECM_PACKET_TYPE_MULTICAST     = (1 << 4),
};

/*
  Generic descriptor union
*/

config_depend(CONFIG_DEVICE_USBDEV)
USBDEV_REPLACE(usbdev_cdc_desc_update);

#endif

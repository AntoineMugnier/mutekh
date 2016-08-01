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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

/**
   @file
   @module {Core::Devices support library}
   @short USB HID class spec related definitions
*/

#ifndef __USB_HID_H__
#define __USB_HID_H__

#include <device/usb/usb.h>
#include <device/class/usbdev.h>

enum usb_hid_subclass_e
{
  USB_HID_SUBCLASS_REPORT   = 0x00,
  USB_HID_SUBCLASS_BOOT     = 0x01,
};

enum usb_hid_boot_protocol_e
{
  USB_HID_PROTOCOL_NONE     = 0x00,
  USB_HID_PROTOCOL_KEYBOARD = 0x01,
  USB_HID_PROTOCOL_MOUSE    = 0x02,
};

enum usb_hid_lang_e
{
  USB_HID_LANG_NOT_SUPPORTED      = 0,
  USB_HID_LANG_ARABIC             = 1,
  USB_HID_LANG_BELGIAN            = 2,
  USB_HID_LANG_CANADIAN_BILINGUAL = 3,
  USB_HID_LANG_CANADIAN_FRENCH    = 4,
  USB_HID_LANG_CZECH_REPUBLIC     = 5,
  USB_HID_LANG_DANISH             = 6,
  USB_HID_LANG_FINNISH            = 7,
  USB_HID_LANG_FRENCH             = 8,
  USB_HID_LANG_GERMAN             = 9,
  USB_HID_LANG_GREEK              = 10,
  USB_HID_LANG_HEBREW             = 11,
  USB_HID_LANG_HUNGARY            = 12,
  USB_HID_LANG_INTERNATIONAL_ISO  = 13,
  USB_HID_LANG_ITALIAN            = 14,
  USB_HID_LANG_JAPAN_KATAKANA     = 15,
  USB_HID_LANG_KOREAN             = 16,
  USB_HID_LANG_LATIN_AMERICAN     = 17,
  USB_HID_LANG_NETHERLANDS_DUTCH  = 18,
  USB_HID_LANG_NORWEGIAN          = 19,
  USB_HID_LANG_PERSIAN_FARSI      = 20,
  USB_HID_LANG_POLAND             = 21,
  USB_HID_LANG_PORTUGUESE         = 22,
  USB_HID_LANG_RUSSIA             = 23,
  USB_HID_LANG_SLOVAKIA           = 24,
  USB_HID_LANG_SPANISH            = 25,
  USB_HID_LANG_SWEDISH            = 26,
  USB_HID_LANG_SWISS_FRENCH       = 27,
  USB_HID_LANG_SWISS_GERMAN       = 28,
  USB_HID_LANG_SWITZERLAND        = 29,
  USB_HID_LANG_TAIWAN             = 30,
  USB_HID_LANG_TURKISH_Q          = 31,
  USB_HID_LANG_UK                 = 32,
  USB_HID_LANG_US                 = 33,
  USB_HID_LANG_YUGOSLAVIA         = 34,
  USB_HID_LANG_TURKISH_F          = 35,
};

enum usb_hid_descriptor_type_e
{
  USB_HID_DESCRIPTOR_HID          = 0x21,
  USB_HID_DESCRIPTOR_REPORT       = 0x22,
  USB_HID_DESCRIPTOR_PHYSICAL     = 0x23,
};

enum usb_hid_report_type_e
{
  USB_HID_REPORT_INPUT   = 0x01,
  USB_HID_REPORT_OUTPUT  = 0x02,
  USB_HID_REPORT_FEATURE = 0x03,
};

enum usb_hid_request_e
{
  USB_HID_GET_REPORT   = 0x1,
  USB_HID_GET_IDLE     = 0x2,
  USB_HID_GET_PROTOCOL = 0x3,
  USB_HID_SET_REPORT   = 0x9,
  USB_HID_SET_IDLE     = 0xa,
  USB_HID_SET_PROTOCOL = 0xb,
};

#define USB_HID_DESCRIPTOR_LENGTH(bNumDescriptors) (6 + 3 * (bNumDescriptors))

struct usb_hid_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint16_t bcdHID;
  uint8_t  bCountryCode;
  uint8_t  bNumDescriptors;

  struct usb_hid_descriptor_element_s
  {
    uint8_t bDescriptorType;
    uint16_t wDescriptorLength;
  } __attribute__((packed)) aDescriptor[];
}__attribute__((packed));

#endif

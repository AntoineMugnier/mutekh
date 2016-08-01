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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014

*/

#include <hexo/types.h>
#include <device/usb/usb.h>
#include <device/class/usbdev.h>
#include <device/usb/cdc.h>

USBDEV_REPLACE(usbdev_descriptor_replace)
{
  size_t offset;

  switch (src->bDescriptorType) {
  case USB_ENDPOINT_DESCRIPTOR: {
    const struct usb_endpoint_descriptor_s *orig = (const void *)src;
    struct usb_endpoint_descriptor_s *to_patch = (void *)dst;

    /* Replace endpoint number */
    offset = offsetof(struct usb_endpoint_descriptor_s, bEndpointAddress);
    if (begin <= offset && offset < end)
      to_patch->bEndpointAddress = usb_ep_dir_get(orig) |
        usbdev_stack_get_ep_addr(orig, it->service->start.epi[it->intf_index],
                                 it->service->start.epo[it->intf_index]);
    break;
  }

  case USB_INTERFACE_DESCRIPTOR: {
    /* Store alternate setting number for endpoint descriptor */
    struct usb_interface_descriptor_s *to_patch = (void *)dst;

    /* Replace interface number */
    offset = offsetof(struct usb_interface_descriptor_s, bInterfaceNumber);
    if (begin <= offset && offset < end)
      to_patch->bInterfaceNumber += it->service->start.intf;

    /* Replace interface string index */
    offset = offsetof(struct usb_interface_descriptor_s, iInterface);
    if (begin <= offset && offset < end && to_patch->iInterface)
      to_patch->iInterface += it->service->start.str;
    break;
  }
  }
}

USBDEV_REPLACE(usbdev_cdc_descriptor_replace)
{
  const struct usb_class_descriptor_header_s *cd = (const void *)src;
  size_t offset;

  switch (src->bDescriptorType) {
  case USB_CDC_INTERFACE_DESCRIPTOR: {
    switch (cd->bDescriptorSubtype) {
    case USB_CDC_DESC_FUNC_CALL_MGMT: {
      struct usb_cdc_call_mgmt_descriptor_s *to_patch = (void *)dst;

      /* Replace interface number */
      offset = offsetof(struct usb_cdc_call_mgmt_descriptor_s, bDataInterface);

      if (begin <= offset && offset < end)
        to_patch->bDataInterface += it->service->start.intf;
      break;
    }

    case USB_CDC_DESC_FUNC_UNION: {
      const struct usb_cdc_union_descriptor_s *orig = (const void *)src;

      /* Replace interface numbers */
      offset = offsetof(struct usb_cdc_union_descriptor_s, bControlInterface);
      for (size_t i = __MAX(offset, begin); i < __MIN(orig->head.bLength, end); ++i)
        ((uint8_t *)dst)[i] += it->service->start.intf;
      break;
    }

    case USB_CDC_DESC_FUNC_ETHERNET: {
      struct usb_cdc_ecm_descriptor_s *to_patch = (void *)dst;

      /* Replace iMACAddress */
      offset = offsetof(struct usb_cdc_ecm_descriptor_s, iMACAddress);
      if (begin <= offset && offset < end)
        to_patch->iMACAddress += it->service->start.str;
      break;
    }
    }
    break;
  }

  default:
    break;
  }
}

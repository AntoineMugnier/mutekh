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

USBDEV_REPLACE(usbdev_cdc_desc_update)
{
  size_t offset;
  uint8_t *p = (uint8_t *)hdr;

  switch (p[sizeof(struct usb_descriptor_header_s)])
    {
    case USB_CDC_DESC_FUNC_CALL_MGMT:
      /* Replace interface number */
      offset = offsetof(struct usb_cdc_call_mgmt_descriptor_s,
                        bDataInterface) - bidx;

      if (offset >= 0 && cnt > offset)
        dst[offset] += index->intf;
      break;
    case USB_CDC_DESC_FUNC_UNION:{
      offset = offsetof(struct usb_cdc_union_descriptor_s,
                        bControlInterface);
      size_t nbr = hdr->bLength - offset; 
      /* Replace interface number */
      for (uint8_t i=0; i<nbr; i++)
        {
          size_t idx = i + offset - bidx;
          if (idx >= 0 && cnt > idx)
            dst[idx] += index->intf;
        }
      break;
    }
    default:
      return;
    }
}

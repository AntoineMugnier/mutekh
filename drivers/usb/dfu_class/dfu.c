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

    Copyright (c) 2018 Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "dfus"

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>
#include <hexo/power.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <device/class/usbdev.h>
#include <device/class/mem.h>
#include <device/usb/usb.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

#include "dfu.h"

#if defined(CONFIG_DRIVER_USBDEV_DFU_FUNCTIONAL)
# error TODO
#endif

enum string_id_e
{
  STRING_ID_INTF_DFU = 1,
  STRING_COUNT = STRING_ID_INTF_DFU,
};

#define STRING_CONSTANT "Firmware Upgrade"
#define DFU_TRANSFER_SIZE 128

static const struct usbdev_interface_default_s dfu_intf =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = 0,
      .bAlternateSetting = 0,
      .bNumEndpoints = 0,
      .bInterfaceClass = USB_DEV_CLASS_APPLICATION_SPECIFIC,
      .bInterfaceSubClass = USB_DEV_APP_SUBCLASS_DFU,
      .bInterfaceProtocol = 0x02,
      .iInterface = STRING_ID_INTF_DFU,
    },
  },
  USBDEV_INTERFACE_ALTERNATE(),
};

static const struct usb_dfu_functional_descriptor_s dfu_func =
{
  .head.bLength = sizeof(struct usb_dfu_functional_descriptor_s),
  .head.bDescriptorType = 0x21,
  .bmAttributes = 0x01,
  .wDetachTimeOut = endian_le16(2000),
  .wTransferSize = endian_le16(DFU_TRANSFER_SIZE),
  .bcdDFUVersion = endian_le16(0x0101),
};

static USBDEV_REPLACE(dfu_descriptor_replace);

static const struct usbdev_service_descriptor_s dfu_service_desc =
{
  USBDEV_SERVICE_DESCRIPTOR(
    &dfu_intf.intf.desc.head,
    &dfu_func.head,
  ),

  USBDEV_INTERFACE(
    &dfu_intf,
  ),

  .str_cnt = STRING_COUNT,
  .string = STRING_CONSTANT,
  .replace = dfu_descriptor_replace,
};

struct dfu_private_s
{
  struct device_usbdev_s usb;
  struct usbdev_service_s service;
  struct usbdev_service_rq_s usb_rq;
  struct device_mem_s mem;
  struct dev_mem_rq_s mem_rq;
  struct dev_mem_page_sc_s mem_sc;

  enum usb_dfu_status_e status;
  enum usb_dfu_state_e state;
  
  uint8_t *data;

  bool_t mem_busy;
  
  size_t page_log2;
  size_t block_base;
  size_t block_count;
  
  size_t offset;
  size_t block;
  size_t to_transfer;
};

STRUCT_COMPOSE(dfu_private_s, service);
STRUCT_COMPOSE(dfu_private_s, usb_rq);
STRUCT_COMPOSE(dfu_private_s, mem_rq);

DRIVER_PV(struct dfu_private_s);

static KROUTINE_EXEC(dfu_ctrl_done);
static KROUTINE_EXEC(dfu_tx_done);
static KROUTINE_EXEC(dfu_upload_done);
static KROUTINE_EXEC(dfu_download_done);
static KROUTINE_EXEC(dfu_mem_read_done);
static KROUTINE_EXEC(dfu_mem_write_done);
static
void dfu_upload_next(struct dfu_private_s *pv);

static USBDEV_REPLACE(dfu_descriptor_replace)
{
  const struct dfu_private_s *pv = const_dfu_private_s_from_service(it->service);
  size_t offset;

  switch (src->bDescriptorType) {
  case 0x21: {
    struct usb_dfu_functional_descriptor_s *to_patch = (void *)dst;

    /* Replace endpoint number */
    offset = offsetof(struct usb_dfu_functional_descriptor_s, wTransferSize);

    if (begin <= offset && offset+1 < end)
      to_patch->wTransferSize = endian_le16(1 << pv->page_log2);
    break;
  }
  }

  usbdev_descriptor_replace(it, src, dst, begin, end);
}


static
void dfu_state_reset(struct dfu_private_s *pv)
{
  pv->status = USB_DFU_STATUS_OK;
  pv->state = USB_DFU_STATE_IDLE;
  pv->offset = 0;
  pv->block = 0;
}

static void dfu_ep0_stall(struct dfu_private_s *pv)
{
  logk("Issuing STALL");
  pv->usb_rq.error = -EINVAL;
  pv->usb_rq.type = USBDEV_GET_COMMAND;

  kroutine_init_deferred(&pv->usb_rq.kr, &dfu_ctrl_done);
  usbdev_stack_request(&pv->usb, &pv->service, &pv->usb_rq);
}

static void dfu_ep0_get_command(struct dfu_private_s *pv)
{
  pv->usb_rq.error = 0;
  pv->usb_rq.type = USBDEV_GET_COMMAND;

  kroutine_init_deferred(&pv->usb_rq.kr, &dfu_ctrl_done);
  usbdev_stack_request(&pv->usb, &pv->service, &pv->usb_rq);
}

static void dfu_ep0_transfer(struct dfu_private_s *pv,
                             kroutine_exec_t *kr,
                             size_t size)
{
  pv->usb_rq.type = USBDEV_TRANSFER_DATA;
  pv->usb_rq.ctrl.size = size;
  pv->usb_rq.error = 0;
  kroutine_init_deferred(&pv->usb_rq.kr, kr);
  usbdev_stack_request(&pv->usb, &pv->service, &pv->usb_rq);
}

static void dfu_ep0_zlp(struct dfu_private_s *pv)
{
  logk("Issuing ZLP");
  return dfu_ep0_transfer(pv, dfu_tx_done, 0);
}

static
void dfu_upload_block(struct dfu_private_s *pv)
{
  size_t block = pv->block + pv->block_base;

  if (pv->block >= pv->block_count) {
    logk("Block overflow");
    pv->status = USB_DFU_STATUS_ERROR_ADDRESS;
    pv->state = USB_DFU_STATE_ERROR;
    return dfu_ep0_stall(pv);
  }

  pv->mem_busy = 1;

  pv->mem_rq.type = DEV_MEM_OP_PAGE_READ;
  pv->mem_rq.band_mask = 1;
  pv->mem_rq.error = 0;
  pv->mem_sc.addr = block << pv->page_log2;
  pv->mem_sc.data = pv->data;
  pv->mem_rq.page.sc = &pv->mem_sc;
  pv->mem_rq.page.sc_count = 1;
  pv->mem_rq.page.page_log2 = pv->page_log2;
  dev_mem_rq_init(&pv->mem_rq, &dfu_mem_read_done);
  DEVICE_OP(&pv->mem, request, &pv->mem_rq);
}

static KROUTINE_EXEC(dfu_mem_read_done)
{
  struct dev_mem_rq_s *mem_rq =  dev_mem_rq_from_kr(kr);
  struct dfu_private_s *pv = dfu_private_s_from_mem_rq(mem_rq);

  logk("Upload mem read done %d", mem_rq->error);

  pv->mem_busy = 0;

  if (mem_rq->error) {
    // There is not read error code
    pv->status = USB_DFU_STATUS_ERROR_UNKNOWN;
    pv->state = USB_DFU_STATE_ERROR;
    return dfu_ep0_zlp(pv);
  }

  pv->offset = 0;
  pv->to_transfer = 1 << pv->page_log2;

  dfu_upload_next(pv);
}

static
void dfu_upload_next(struct dfu_private_s *pv)
{
  size_t size = pv->to_transfer;

  if (size > CONFIG_USBDEV_EP0_BUFFER_SIZE)
    size = CONFIG_USBDEV_EP0_BUFFER_SIZE;
  memcpy(pv->usb_rq.ctrl.buffer, pv->data + pv->offset, size);

  logk("Upload next mem read, block 0x%08x, offset %04x, to transfer 0x%04x, actual 0x%x",
       pv->block, pv->offset, pv->to_transfer, size);

  return dfu_ep0_transfer(pv, dfu_upload_done, size);
}

static KROUTINE_EXEC(dfu_upload_done)
{
  struct usbdev_service_rq_s *usb_rq =  KROUTINE_CONTAINER(kr, *usb_rq, kr);
  struct dfu_private_s *pv = dfu_private_s_from_usb_rq(usb_rq);

  logk("USB upload chunk done");

  size_t len = pv->usb_rq.ctrl.size;
  pv->offset += len;
  pv->to_transfer -= len;

  if (pv->to_transfer)
    dfu_upload_next(pv);
  else
    dfu_ep0_get_command(pv);
}

static
void dfu_download_next(struct dfu_private_s *pv)
{
  size_t  size = pv->to_transfer;
  if (size > CONFIG_USBDEV_EP0_BUFFER_SIZE)
    size = CONFIG_USBDEV_EP0_BUFFER_SIZE;

  logk_trace("Download next mem read, block 0x%08x, offset %04x, to transfer 0x%04x, actual 0x%x",
             pv->block, pv->offset, pv->to_transfer, size);

  return dfu_ep0_transfer(pv, dfu_download_done, size);
}

static KROUTINE_EXEC(dfu_download_done)
{
  struct usbdev_service_rq_s *usb_rq =  KROUTINE_CONTAINER(kr, *usb_rq, kr);
  struct dfu_private_s *pv = dfu_private_s_from_usb_rq(usb_rq);
  size_t block = pv->block + pv->block_base;

  memcpy(pv->data + pv->offset, pv->usb_rq.ctrl.buffer, pv->usb_rq.ctrl.size);
  pv->offset += pv->usb_rq.ctrl.size;
  pv->to_transfer -= pv->usb_rq.ctrl.size;

  if (pv->to_transfer)
    return dfu_download_next(pv);

  if (pv->offset < (1 << pv->page_log2))
    memset(pv->data + pv->offset, 0, (1 << pv->page_log2) - pv->offset);
  
  pv->state = USB_DFU_STATE_DOWNLOAD_BUSY;
  pv->mem_busy = 1;

  dfu_ep0_get_command(pv);

  pv->mem_rq.type = DEV_MEM_OP_PAGE_ERASE | DEV_MEM_OP_PAGE_WRITE_THROUGH;
  pv->mem_rq.band_mask = 1;
  pv->mem_rq.error = 0;
  pv->mem_sc.addr = block << pv->page_log2;
  pv->mem_sc.data = pv->data;
  pv->mem_rq.page.sc = &pv->mem_sc;
  pv->mem_rq.page.sc_count = 1;
  pv->mem_rq.page.page_log2 = pv->page_log2;
  dev_mem_rq_init(&pv->mem_rq, &dfu_mem_write_done);
  DEVICE_OP(&pv->mem, request, &pv->mem_rq);
}

static KROUTINE_EXEC(dfu_mem_write_done)
{
  struct dev_mem_rq_s *mem_rq =  dev_mem_rq_from_kr(kr);
  struct dfu_private_s *pv = dfu_private_s_from_mem_rq(mem_rq);

  pv->mem_busy = 0;

  // Host aborted transfer in the mean time ?
  if (pv->state != USB_DFU_STATE_DOWNLOAD_BUSY)
    return;

  pv->state = USB_DFU_STATE_DOWNLOAD_SYNC;

  if (mem_rq->error) {
    pv->status = USB_DFU_STATUS_ERROR_WRITE;
    pv->state = USB_DFU_STATE_ERROR;
    return;
  }
}

static KROUTINE_EXEC(dfu_tx_done)
{
  struct usbdev_service_rq_s *usb_rq =  KROUTINE_CONTAINER(kr, *usb_rq, kr);
  struct dfu_private_s *pv = dfu_private_s_from_usb_rq(usb_rq);

  logk("Transfer done");

  usb_rq->type = USBDEV_GET_COMMAND;
  usb_rq->error = 0;

  kroutine_init_deferred(&usb_rq->kr, &dfu_ctrl_done);
  usbdev_stack_request(&pv->usb, &pv->service, &pv->usb_rq);
}

static KROUTINE_EXEC(dfu_ctrl_done)
{
  struct usbdev_service_rq_s *usb_rq =  KROUTINE_CONTAINER(kr, *usb_rq, kr);
  struct dfu_private_s *pv = dfu_private_s_from_usb_rq(usb_rq);
  uint16_t len = usb_setup_length_get(usb_rq->ctrl.setup);

  usb_rq->error = 0;

  switch (usb_rq->cmd) {
  case USBDEV_ENABLE_SERVICE:
    logk_trace("Enabled");
    return dfu_ep0_get_command(pv);
    
  case USBDEV_DISABLE_SERVICE:
    logk_trace("Disabled");
    return dfu_ep0_get_command(pv);

  case USBDEV_CHANGE_INTERFACE:
    logk_trace("Interface changed to %d", usb_rq->alternate);
    return dfu_ep0_get_command(pv);
    
  case USBDEV_PROCESS_CONTROL:
    logk_trace("Control %02x", usb_setup_request_get(usb_rq->ctrl.setup));
    /* We use provided buffer to send/retrieve USB data */
       
    switch (usb_setup_request_get(usb_rq->ctrl.setup)) {
    case USB_DFU_REQUEST_UPLOAD: {
      if (usb_setup_direction_get(usb_rq->ctrl.setup) != USB_DEVICE_TO_HOST)
        break;
      uint16_t block = usb_setup_value_get(usb_rq->ctrl.setup);

      if (pv->state != USB_DFU_STATE_UPLOAD_IDLE
          && pv->state != USB_DFU_STATE_IDLE) {
        logk_error("Upload request from other state");
        return dfu_ep0_stall(pv);
      }

      if (pv->mem_busy) {
        logk_error("Upload request while memory is busy");
        return dfu_ep0_stall(pv);
      }
      
      pv->state = USB_DFU_STATE_UPLOAD_IDLE;

      if (len != (1 << pv->page_log2)) {
        pv->status = USB_DFU_STATUS_ERROR_FIRMWARE;
        pv->state = USB_DFU_STATE_ERROR;
        logk_error("Transaction not using page size, aborting");
        return dfu_ep0_stall(pv);
      }
      
      pv->to_transfer = len;
      if (((pv->block & 0xffff) == 0xffff) && block == 0) {
        pv->block &= ~0xffff;
        pv->block += 0x10000;
      } else {
        pv->block &= ~0xffff;
        pv->block |= block;
      }

      logk("Upload request, block 0x%04x, length 0x%04x -> block 0x%08x",
           block, len, pv->block);

      return dfu_upload_block(pv);
    }

    case USB_DFU_REQUEST_GETSTATUS: {
      if (usb_setup_value_get(usb_rq->ctrl.setup)
          || usb_setup_direction_get(usb_rq->ctrl.setup) != USB_DEVICE_TO_HOST
          || len != sizeof(struct usb_dfu_status_s))
        break;

      uint32_t to = 100;
      
      switch (pv->state) {
      case USB_DFU_STATE_DOWNLOAD_SYNC:
        pv->state = USB_DFU_STATE_DOWNLOAD_IDLE;
        to = 1;
        break;
      case USB_DFU_STATE_MANIFEST_SYNC:
        pv->state = USB_DFU_STATE_MANIFEST;
        to = 1;
        break;
      case USB_DFU_STATE_MANIFEST:
        pv->state = USB_DFU_STATE_MANIFEST_WAIT_RESET;
        power_reboot();
        to = 1;
        break;
      default:
        break;
      }

      struct usb_dfu_status_s *status = (struct usb_dfu_status_s*)usb_rq->ctrl.buffer;
      status->bStatus = pv->status;
      status->bwPollTimeoutLow = endian_le16(to & 0xffff);
      status->bwPollTimeoutHigh = to >> 16;
      status->bState = pv->state;
      status->bStatus = pv->status;
      status->iString = 0;

      return dfu_ep0_transfer(pv, dfu_tx_done, sizeof(struct usb_dfu_status_s));
    }

    case USB_DFU_REQUEST_CLRSTATUS:
    case USB_DFU_REQUEST_ABORT:
       if (usb_setup_value_get(usb_rq->ctrl.setup)
          || usb_setup_direction_get(usb_rq->ctrl.setup) != USB_HOST_TO_DEVICE
          || len)
        break;
       dfu_state_reset(pv);
       return dfu_ep0_get_command(pv);

    case USB_DFU_REQUEST_GETSTATE:
      if (usb_setup_value_get(usb_rq->ctrl.setup)
          || usb_setup_direction_get(usb_rq->ctrl.setup) != USB_DEVICE_TO_HOST
          || len != 1)
        break;

      return dfu_ep0_transfer(pv, dfu_tx_done, 1);

    case USB_DFU_REQUEST_DNLOAD: {
      if (usb_setup_direction_get(usb_rq->ctrl.setup) != USB_HOST_TO_DEVICE)
        break;
      uint16_t block = usb_setup_value_get(usb_rq->ctrl.setup);

      if (pv->state != USB_DFU_STATE_DOWNLOAD_IDLE
          && pv->state != USB_DFU_STATE_IDLE) {
        logk_error("Download request from other state");
        return dfu_ep0_stall(pv);
      }

      if (pv->mem_busy) {
        logk_error("Download request while memory is busy");
        return dfu_ep0_stall(pv);
      }

      // Put error here preventively, it should be cleared by successful upload
      pv->state = USB_DFU_STATE_ERROR;

      if (len == 0) {
        pv->state = USB_DFU_STATE_MANIFEST_SYNC;
        logk_error("Download done");
        return dfu_ep0_get_command(pv);
      }

      if (len > (1 << pv->page_log2)) {
        pv->status = USB_DFU_STATUS_ERROR_FIRMWARE;
        pv->state = USB_DFU_STATE_ERROR;
        logk_error("Transaction above page size, aborting");
        return dfu_ep0_stall(pv);
      }

      pv->to_transfer = len;
      pv->offset = 0;

      if (((pv->block & 0xffff) == 0xffff) && block == 0) {
        pv->block &= ~0xffff;
        pv->block += 0x10000;
      } else {
        pv->block &= ~0xffff;
        pv->block |= block;
      }

      logk("Download request, block 0x%04x, length 0x%04x -> block 0x%08x",
           block, len, pv->block);

      return dfu_download_next(pv);
    }
      
    case USB_DFU_REQUEST_DETACH:
      return dfu_ep0_stall(pv);

    default:
      break;
    }
    break;

  default:
    break;
  }

  logk_error("DFU unhandled cmd 0x%02x/0x%02x",
             usb_rq->cmd,
             usb_setup_request_get(usb_rq->ctrl.setup));
  return dfu_ep0_stall(pv);
}

#define dfu_use dev_use_generic

static DEV_INIT(dfu_init)
{
  error_t err = 0;
  struct dfu_private_s *pv;
  struct dev_mem_info_s info;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "usb-ctrl", &pv->usb.base, DRIVER_CLASS_USBDEV);
  if (err)
    goto err_pv;

  err = device_get_param_dev_accessor(dev, "storage", &pv->mem.base, DRIVER_CLASS_MEM);
  if (err)
    goto err_usb;

  err = DEVICE_OP(&pv->mem, info, &info, 0);
  if (err) {
    logk_error("Cannot get memory info");
    goto err_mem;
  }

  pv->page_log2 = info.page_log2;

  uintptr_t base, size;
  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &base, &size);
  if (err) {
    base = 0;
    size = info.size << info.page_log2;
  }

  pv->block_base = (base + (1 << info.page_log2) - 1) >> info.page_log2;
  size_t block_end = (base + size + (1 << info.page_log2) - 1) >> info.page_log2;

  if (pv->block_base >= info.size) {
    logk_error("DFU Zone starts beyond end of backing storage");
    goto err_mem;
  }

  pv->data = mem_alloc(1 << pv->page_log2, (mem_scope_sys));
  if (!pv->data) {
    logk_error("Cannot allocate page store");
    goto err_mem;
  }

  if (size == 0)
    pv->block_count = info.size - pv->block_base;
  else
    pv->block_count = block_end - pv->block_base;
    
  if (pv->block_base + pv->block_count > info.size) {
    pv->block_count = info.size - pv->block_base;
    logk_warning("DFU Zone truncated to %d blocks", pv->block_count);
  }
  
  pv->service.desc = &dfu_service_desc;
  pv->service.pv = dev;

  kroutine_init_deferred(&pv->usb_rq.kr, &dfu_ctrl_done);

  err = usbdev_stack_service_register(&pv->usb, &pv->service);
  if (err)
    goto err_data;

  logk_error("DFU service ready, exposing %d blocks of %d bytes",
             pv->block_count, 1 << pv->page_log2);

  pv->usb_rq.type = USBDEV_GET_COMMAND; 
  pv->usb_rq.error = 0;

  pv->mem_busy = 0;

  dfu_state_reset(pv);
  
  usbdev_stack_request(&pv->usb, &pv->service, &pv->usb_rq);

  return 0;

 err_data:
  mem_free(pv->data);
 err_mem:
  device_put_accessor(&pv->mem.base);
 err_usb:
  device_put_accessor(&pv->usb.base);
 err_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(dfu_cleanup)
{
  struct dfu_private_s *pv = dev->drv_pv;

  device_put_accessor(&pv->usb.base);
  device_put_accessor(&pv->mem.base);
  mem_free(pv->data);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(usbdev_dfu_drv, 0, "USB DFU", dfu,
               NULL);

DRIVER_REGISTER(usbdev_dfu_drv);

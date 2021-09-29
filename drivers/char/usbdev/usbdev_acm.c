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

#define LOGK_MODULE_ID "uAcm"

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/resource/uart.h>
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
# include <device/class/valio.h>
# include <device/valio/uart_config.h>
#endif

#include <device/usb/usb.h>
#include <device/usb/cdc.h>

#include <device/class/usbdev.h>
#include <device/class/char.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

/*
  This is a CHAR class driver implementing the USB communications device
  class (CDC ACM) as an USB device service.

  This USB service can be used with others services on the same USB device
  controller.

  This is an example of char service device instantiation:

  DEV_DECLARE_STATIC(usbdev_acm0, "usbdev-char0", 0, usbdev_acm_drv,
                     DEV_STATIC_RES_USBDEV_EP_MAP(0, 0x32, 0x01),
                     DEV_STATIC_RES_DEV_PARAM("usb-ctrl", "/max3420")
  );

*/

#define USBDEV_SERV_CHAR_BUFFER_SIZE 128
/* Bulk endpoint max packet size */
#define USBDEV_SERV_CHAR_BULK_SIZE 64

enum usbdev_serv_char_intf_e
{
  USBDEV_SERV_CHAR_INTF_CTRL,
  USBDEV_SERV_CHAR_INTF_DATA,
  USBDEV_SERV_CHAR_INTF_COUNT,
};

static const struct usb_interface_association_descriptor_s iad_acm =
{
  .head.bLength = sizeof(iad_acm),
  .head.bDescriptorType = USB_DESC_INTERFACE_ASSOCIATION,
  .bFirstInterface = USBDEV_SERV_CHAR_INTF_CTRL, // will be offsetted
  .bInterfaceCount = USBDEV_SERV_CHAR_INTF_COUNT,
  .bFunctionClass = USB_DEV_CLASS_COMM,
  .bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
  .bFunctionProtocol = USB_CDC_PROTOCOL_NONE,
  .iFunction = 3,
};

/* Functionnal descriptor */

static const struct usb_cdc_header_descriptor_s cdc_header =
{
  .head.bLength = sizeof(struct usb_cdc_header_descriptor_s),
  .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
  .bDescriptorSubtype = USB_CDC_DESC_FUNC_HEADER,
  .bcdCDC = endian_le16(0x0120),
};

static const struct usb_cdc_call_mgmt_descriptor_s cdc_call_mgmt =
{
  .head.bLength = sizeof(struct usb_cdc_call_mgmt_descriptor_s),
  .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
  .bDescriptorSubtype = USB_CDC_DESC_FUNC_CALL_MGMT,
  .bmCapabilities = USBDEV_SERV_CHAR_INTF_CTRL,
  .bDataInterface = USBDEV_SERV_CHAR_INTF_DATA,
};

static const struct usb_cdc_acm_descriptor_s cdc_acm =
{
  .head.bLength = sizeof(struct usb_cdc_acm_descriptor_s),
  .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
  .bDescriptorSubtype = USB_CDC_DESC_FUNC_ACM,
  .bmCapabilities = 2,
};

static const struct usb_cdc_union_descriptor_s cdc_union =
{
  .head.bLength = sizeof(struct usb_cdc_union_descriptor_s),
  .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
  .bDescriptorSubtype = USB_CDC_DESC_FUNC_UNION,
  .bControlInterface = USBDEV_SERV_CHAR_INTF_CTRL,
  .bSubordinateInterface = USBDEV_SERV_CHAR_INTF_DATA,
};

/* Standart descriptors */

static struct usb_endpoint_descriptor_s ep_bulk_in =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_IN | 0,
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(USBDEV_SERV_CHAR_BULK_SIZE),
  .bInterval = 0
};

static struct usb_endpoint_descriptor_s ep_bulk_out =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_OUT | 0,
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(USBDEV_SERV_CHAR_BULK_SIZE),
  .bInterval = 0
};

static const struct usbdev_interface_default_s interface_cdc_data0 =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = USBDEV_SERV_CHAR_INTF_DATA,
      .bAlternateSetting = 0,
      .bNumEndpoints = 2,
      .bInterfaceClass = USB_DEV_CLASS_COMM_DATA,
      .bInterfaceSubClass = 0,
      .bInterfaceProtocol = 0,
      .iInterface = 1
    },
    USBDEV_ENDPOINT(&ep_bulk_in, &ep_bulk_out)
  },
  USBDEV_INTERFACE_ALTERNATE()
};

static struct usb_endpoint_descriptor_s ep_irq_in =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_IN | 1,
  .bmAttributes = USB_EP_INTERRUPT,
  .wMaxPacketSize = 8,
  .bInterval = 0xFF
};

static const struct usbdev_interface_default_s interface_cdc_ctrl =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = USBDEV_SERV_CHAR_INTF_CTRL,
      .bAlternateSetting = 0,
      .bNumEndpoints = 1,
      .bInterfaceClass = USB_DEV_CLASS_COMM,
      .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
      .bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
      .iInterface = 2
    },
    USBDEV_ENDPOINT(&ep_irq_in)
  },
  USBDEV_INTERFACE_ALTERNATE()
};

static
USBDEV_REPLACE(usbdev_acm_replace)
{
  usbdev_descriptor_replace(it, src, dst, begin, end);
  usbdev_cdc_descriptor_replace(it, src, dst, begin, end);
}

struct usbdev_acm_private_s
{
  struct usbdev_service_s service;
  /* Associated USB controller device */
  struct device_usbdev_s usb;
  /* Read, write queue */
  dev_request_queue_root_t read_q;
  dev_request_queue_root_t write_q;
  /* Sequence for kroutine */
  struct kroutine_sequence_s seq;
  /* Usb stack request endpoint 0 */
  struct usbdev_service_rq_s rq;

  struct usbdev_cdc_line_coding_s coding;

  struct dev_usbdev_rq_s wtr;
  uint8_t *wbuffer;

  struct dev_usbdev_rq_s rtr;
  uint8_t *rbuffer;

  bool_t read_started;
  bool_t write_started;
  bool_t zlp;

  size_t remaining;

  bool_t service_enabled;
  /* Notify next request of service disabling */
  bool_t disconnection;

  /* Service mapping */
  uint8_t interface_start_index;
  dev_usbdev_ep_map_t epi_map;
  dev_usbdev_ep_map_t epo_map;

  const char *function;
  
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
  dev_request_queue_root_t coding_notify_queue;
#endif
};

STRUCT_COMPOSE(usbdev_acm_private_s, service);

DRIVER_PV(struct usbdev_acm_private_s);

static
void cdc_line_coding_set(struct usbdev_cdc_line_coding_s *coding,
                         const struct dev_uart_config_s *cfg)
{
  coding->dwDTERate = endian_le32(cfg->baudrate);
  coding->bCharFormat = (cfg->stop_bits == 1) ? 0 : 2;
  coding->bParityType = cfg->parity;
  coding->bDataBits = cfg->data_bits;
}

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
static
void cdc_line_coding_parse(const struct usbdev_cdc_line_coding_s *coding,
                           struct dev_uart_config_s *cfg)
{
  cfg->baudrate = endian_le32(coding->dwDTERate);
  cfg->stop_bits = (coding->bCharFormat == 0) ? 1 : 2;
  cfg->parity = coding->bParityType;
  cfg->data_bits = coding->bDataBits;
}
#endif

static
void usbdev_service_char_read(struct device_s *dev);
static
void usbdev_service_char_write(struct device_s *dev, bool_t flush);

static
void usbdev_service_end_request(struct device_s *dev, bool_t read, error_t err)
{
  struct usbdev_acm_private_s *pv = dev->drv_pv;
  struct dev_char_rq_s *rq;

  if (read)
    {
      pv->read_started = 0;
      rq = dev_char_rq_pop(&pv->read_q);
      if (rq)
        rq->base.drvdata = NULL;
    }
  else
    {
      pv->write_started = 0;
      rq = dev_char_rq_pop(&pv->write_q);
      if (rq)
        rq->base.drvdata = NULL;
    }

  if (rq == 0)
    return;

  rq->error = err;
  dev_char_rq_done(rq);
}

static
void usbdev_service_try_read(struct device_s *dev)
{
  struct usbdev_acm_private_s *pv = dev->drv_pv;
  struct dev_usbdev_rq_s *tr = &pv->rtr;
  struct dev_char_rq_s *rq;

  while(1)
    {
      rq = dev_char_rq_head(&pv->read_q);

      pv->read_started = 1;

      if (rq == NULL)
        {
          pv->read_started = 0;
          break;
        }

      if (!pv->remaining)
        /* No data or all data used */
        {
          usbdev_service_char_read(dev);
          break;
        }

      if (rq->type != DEV_CHAR_READ_POLL)
        {
          size_t min = pv->remaining < rq->size ? pv->remaining : rq->size;
          memcpy(rq->data, tr->data - pv->remaining, min);

          pv->remaining -= min;
          rq->data += min;
          rq->size -= min;
        }

      if (rq->type == DEV_CHAR_READ_PARTIAL ||
          rq->type == DEV_CHAR_READ_POLL ||
          rq->size == 0)
        usbdev_service_end_request(dev, 1, 0);
   }
}

static
void usbdev_service_try_write(struct device_s *dev)
{
  struct usbdev_acm_private_s *pv = dev->drv_pv;
  struct dev_usbdev_rq_s *tr = &pv->wtr;

  uint8_t * p = tr->data = pv->wbuffer;

  tr->size = 0;
  bool_t flush = 0;

  while(1)
    {
      struct dev_char_rq_s *rq = dev_char_rq_head(&pv->write_q);

      pv->write_started = 1;

      if (rq == NULL || tr->size == USBDEV_SERV_CHAR_BUFFER_SIZE || flush)
        {
          if (tr->size)
            usbdev_service_char_write(dev, flush);
          else
            pv->write_started = 0;
          break;
        }


      size_t s = USBDEV_SERV_CHAR_BUFFER_SIZE - tr->size;
      size_t min = s < rq->size ? s : rq->size;

      memcpy(p, rq->data, min);

      tr->size += min;
      p += min;
      rq->data += min;
      rq->size -= min;

      flush = (rq->size == 0) && (rq->type & _DEV_CHAR_FLUSH);

      if (rq->type == DEV_CHAR_WRITE_PARTIAL || rq->size == 0)
        usbdev_service_end_request(dev, 0, 0);
   }
}

static
KROUTINE_EXEC(usbdev_acm_write_cb)
{
  struct dev_usbdev_rq_s *tr = KROUTINE_CONTAINER(kr, *tr, base.kr);
  struct device_s *dev = tr->pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (tr->error)
    {
      /* USB reset or disconnected */
      logk_debug("Write -EPIPE");
      usbdev_service_end_request(dev, 0, tr->error);
      return;
    }

  struct usbdev_acm_private_s *pv = dev->drv_pv;

  if (pv->zlp)
  /* A zero length packet must be sent */
    {
      tr->size = 0;
      pv->zlp = 0;
      usbdev_service_char_write(dev, 0);
    }
  else
    usbdev_service_try_write(dev);
}

static
KROUTINE_EXEC(usbdev_acm_read_cb)
{
  struct dev_usbdev_rq_s *tr = KROUTINE_CONTAINER(kr, *tr, base.kr);
  struct device_s *dev = tr->pvdata;
  struct usbdev_acm_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (tr->error)
    {
      /* USB reset or disconnected */
      logk_debug("Read -EPIPE");
      usbdev_service_end_request(dev, 1, tr->error);
    }
  else
    {
      pv->remaining = USBDEV_SERV_CHAR_BUFFER_SIZE - tr->size;
      usbdev_service_try_read(dev);
    }
}

static
void usbdev_service_char_write(struct device_s *dev, bool_t flush)
{
  struct usbdev_acm_private_s *pv = dev->drv_pv;
  struct dev_usbdev_rq_s *tr = &pv->wtr;


  tr->type = DEV_USBDEV_DATA_IN;
  tr->error = 0;
  tr->pvdata = dev;

  pv->zlp = 0;
  /* ZLP when size is multiple of MPS and _DEV_CHAR_FLUSH request */
  if (tr->size % USBDEV_SERV_CHAR_BULK_SIZE == 0)
    pv->zlp = flush;

  /* Push transfer to stack */
  error_t err = usbdev_stack_transfer(&pv->usb, &pv->service, tr, &ep_bulk_in);

  if (err)
    {
      /* USB reset or disconnected */
      usbdev_service_end_request(dev, 0, err);
    }
}

static
void usbdev_service_char_read(struct device_s *dev)
{
  struct usbdev_acm_private_s *pv = dev->drv_pv;
  struct dev_usbdev_rq_s *tr = &pv->rtr;

  tr->size = USBDEV_SERV_CHAR_BUFFER_SIZE;
  tr->type = DEV_USBDEV_DATA_OUT;
  tr->data = pv->rbuffer;
  tr->error = 0;

  /* Push transfer to stack */
  error_t err = usbdev_stack_transfer(&pv->usb, &pv->service, tr, &ep_bulk_out);

  if (err)
    {
      /* USB reset or disconnected */
      usbdev_service_end_request(dev, 1, err);
    }
}

static KROUTINE_EXEC(usbdev_acm_ctrl_cb);

static
KROUTINE_EXEC(usbdev_acm_transfer_cb)
{
  struct usbdev_service_rq_s *rq =  KROUTINE_CONTAINER(kr, *rq, kr);
  struct device_s *dev = rq->pvdata;
  struct usbdev_acm_private_s *pv = dev->drv_pv;

  if (memcmp(&pv->coding, rq->ctrl.buffer,
             sizeof(struct usbdev_cdc_line_coding_s))) {
    /* Update coding parameters */
    memcpy(&pv->coding, rq->ctrl.buffer, sizeof(struct usbdev_cdc_line_coding_s));

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
    struct dev_uart_config_s config;

    cdc_line_coding_parse(&pv->coding, &config);

    /* Notify listeners */
    GCT_FOREACH(dev_request_queue, &pv->coding_notify_queue, item, {
        struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);

        dev_valio_rq_remove(&pv->coding_notify_queue, rq);
        rq->error = 0;
        memcpy(rq->data, &config, sizeof(config));
        dev_valio_rq_done(rq);
      });
#endif
  }

  rq->type = USBDEV_GET_COMMAND;
  rq->error = 0;

  kroutine_init_deferred_seq(&rq->kr, &usbdev_acm_ctrl_cb, &pv->seq);

  /* Push request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}

static
KROUTINE_EXEC(usbdev_acm_ctrl_cb)
{
  struct usbdev_service_rq_s *rq =  KROUTINE_CONTAINER(kr, *rq, kr);
  struct device_s *dev = rq->pvdata;
  struct usbdev_acm_private_s *pv = dev->drv_pv;

  {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    rq->error = 0;

    switch (rq->cmd)
      {
      case USBDEV_ENABLE_SERVICE:
        logk_debug("ACM Service enable");

        /* Enable service and check queues */
        pv->service_enabled = 1;

        /* Start a USB bulk write transfer */
        usbdev_service_try_write(dev);
        usbdev_service_try_read(dev);

        break;

      case USBDEV_DISABLE_SERVICE:
        logk_debug("ACM Service disable");

        pv->service_enabled = 0;
        if (!pv->read_started)
          pv->disconnection = 1;

        break;

      case USBDEV_PROCESS_CONTROL: {
        /* Only CDC control interface support specific request*/
        ensure(rq->intf == USBDEV_SERV_CHAR_INTF_CTRL);

        /* We use provided buffer to send/retrieve USB data */
        const struct usb_ctrl_setup_s *setup = (const void *)rq->ctrl.setup;
        uint16_t len = usb_setup_length_get(setup);

        ensure(len <= CONFIG_USBDEV_EP0_BUFFER_SIZE);

        rq->error = -EINVAL;

        switch (usb_setup_request_get(setup))
          {
          case USB_CDC_GET_LINE_CODING:
            logk_debug("Get Line Coding req");
            if (usb_setup_value_get(setup) ||
                usb_setup_direction_get(setup) != USB_DEVICE_TO_HOST)
              break;

            /* Copy data in provided buffer */
            memcpy(rq->ctrl.buffer, &pv->coding, len);

            rq->type = USBDEV_TRANSFER_DATA;
            rq->ctrl.size = len;
            rq->error = 0;
            kroutine_init_deferred_seq(&rq->kr, &usbdev_acm_transfer_cb, &pv->seq);
            break;

          case USB_CDC_SET_LINE_CODING:
            logk_debug("Set Line Coding req");
            if (usb_setup_value_get(setup) ||
                usb_setup_direction_get(setup) != USB_HOST_TO_DEVICE)
              break;

            rq->type = USBDEV_TRANSFER_DATA;
            rq->ctrl.size = len;
            rq->error = 0;
            kroutine_init_deferred_seq(&rq->kr, &usbdev_acm_transfer_cb, &pv->seq);
            break;

          case USB_CDC_SET_CONTROL_LINE_STATE:
            logk_debug("Set Line State req");
            rq->error = 0;
            break;

          default:
            logk_debug("Unsupported request type %d", usb_setup_request_get(setup));
            break;
          }
      }

      default:
        break;
      }

    if (rq->error)
      logk_debug("Request error %d", rq->error);
  }

  /* Push request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}

static USBDEV_GET_STRING(usbdev_acm_get_string)
{
  const struct usbdev_acm_private_s *pv = const_usbdev_acm_private_s_from_service(service);

  if (idx == 3)
    return pv->function;

  return NULL;
}

/* USB CDC ACM service */
static const struct usbdev_service_descriptor_s usb_cdc_acm_service =
{
  /* Local descriptor */
  USBDEV_SERVICE_DESCRIPTOR(
      &iad_acm.head,
      &interface_cdc_ctrl.intf.desc.head,
      &cdc_header.head,
      &cdc_call_mgmt.head,
      &cdc_acm.head,
      &cdc_union.head,
      &ep_irq_in.head,
      &interface_cdc_data0.intf.desc.head,
      &ep_bulk_out.head,
      &ep_bulk_in.head
    ),

  USBDEV_INTERFACE(
    &interface_cdc_ctrl,
    &interface_cdc_data0
    ),

  .str_cnt = 3,
  .string =
    "CDC ACM Data\0"
    "CDC ACM Control\0",

  .get_string = usbdev_acm_get_string,
  .replace = usbdev_acm_replace,
};

static
DEV_CHAR_CANCEL(usbdev_acm_cancel)
{
  struct device_s *dev = accessor->dev;
  struct usbdev_acm_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_POLL:
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
      if (rq->base.drvdata != pv)
        return -EBUSY;

      rq->base.drvdata = NULL;
      dev_char_rq_remove(&pv->read_q, rq);
      return 0;

    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
      if (rq->base.drvdata != pv)
        return -EBUSY;

      rq->base.drvdata = NULL;
      dev_char_rq_remove(&pv->write_q, rq);
      return 0;

    default:
      return -ENOTSUP;
    }
}

static
DEV_CHAR_REQUEST(usbdev_acm_request)
{
  struct device_s               *dev = accessor->dev;
  struct usbdev_acm_private_s   *pv = dev->drv_pv;

  if (rq->size == 0)
    {
      dev_char_rq_done(rq);
      return;
    }

  LOCK_SPIN_IRQ(&dev->lock);

  rq->error = 0;
  rq->base.drvdata = pv;

  switch (rq->type)
    {
    case DEV_CHAR_READ_POLL:
      if (rq->size > 1)
        {
          rq->error = -ENOTSUP;
          break;
        }
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
      if (pv->disconnection)
        {
          pv->disconnection = 0;
          rq->error = -EPIPE;
          break;
        }
      dev_char_rq_pushback(&pv->read_q, rq);
      /* Start a USB bulk read transfer */
      if (!pv->read_started && pv->service_enabled)
        usbdev_service_try_read(dev);
      break;

    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
      dev_char_rq_pushback(&pv->write_q, rq);
      /* Start a USB bulk write transfer */
      if (!pv->write_started && pv->service_enabled)
        usbdev_service_try_write(dev);
      break;

    default:
      rq->error = -ENOTSUP;
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (rq->error)
    dev_char_rq_done(rq);
}

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
static
DEV_VALIO_CANCEL(usbdev_acm_valio_cancel)
{
  struct device_s *dev = accessor->dev;
  struct usbdev_acm_private_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->base.drvdata == dev)
    {
      dev_valio_rq_remove(&pv->coding_notify_queue, rq);
      rq->base.drvdata = NULL;
      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
DEV_VALIO_REQUEST(usbdev_acm_valio_request)
{
  struct device_s               *dev = accessor->dev;
  struct usbdev_acm_private_s   *pv = dev->drv_pv;

  if (rq->attribute != VALIO_UART_CONFIG) {
    rq->error = -ENOTSUP;
    dev_valio_rq_done(rq);
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->type) {
  case DEVICE_VALIO_WRITE:
    cdc_line_coding_set(&pv->coding, rq->data);
    goto done;

  case DEVICE_VALIO_READ:
    cdc_line_coding_parse(&pv->coding, rq->data);

  done:
    rq->error = 0;
    dev_valio_rq_done(rq);
    return;

  case DEVICE_VALIO_WAIT_EVENT:
    dev_valio_rq_pushback(&pv->coding_notify_queue, rq);
    rq->base.drvdata = dev;
    return;
  }
}
#endif

#define usbdev_acm_use dev_use_generic

static
DEV_INIT(usbdev_acm_init)
{
  error_t err = 0;
  struct dev_uart_config_s cfg = {
    .baudrate = 115200,
    .data_bits = 8,
    .parity = DEV_UART_PARITY_NONE,
    .stop_bits = 1,
  };

  struct usbdev_acm_private_s *pv;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "usb-ctrl", &pv->usb.base, DRIVER_CLASS_USBDEV);
  if (err)
    goto err_pv;

  /* Default setting */
  pv->wbuffer = usbdev_stack_allocate(&pv->usb, USBDEV_SERV_CHAR_BUFFER_SIZE);

  if (!pv->wbuffer)
    goto err_ctrl;

  pv->rbuffer = usbdev_stack_allocate(&pv->usb, USBDEV_SERV_CHAR_BUFFER_SIZE);
  if (!pv->rbuffer)
    goto err_wbuffer;

  pv->wtr.pvdata = dev;
  pv->rtr.pvdata = dev;

  device_get_res_uart(dev, &cfg);
  cdc_line_coding_set(&pv->coding, &cfg);

  pv->service.desc = &usb_cdc_acm_service;
  pv->service.pv = dev;

  device_get_param_str_default(dev, "function", &pv->function, "serial port");
  
  /* Get endpoint map from ressources */
  dev_res_get_usbdev_epmap(dev, &pv->service);

  err = usbdev_stack_service_register(&pv->usb, &pv->service);
  if (err)
    goto err_rbuffer;

  dev_rq_queue_init(&pv->read_q);
  dev_rq_queue_init(&pv->write_q);
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
  dev_rq_queue_init(&pv->coding_notify_queue);
#endif

  struct usbdev_service_rq_s *rq = &pv->rq;

  rq->pvdata = dev;
  rq->type = USBDEV_GET_COMMAND;
  rq->error = 0;

  kroutine_seq_init(&pv->seq);

  kroutine_init_deferred_seq(&rq->kr, &usbdev_acm_ctrl_cb, &pv->seq);
  dev_usbdev_rq_init_seq(&pv->wtr, &usbdev_acm_write_cb, &pv->seq);
  dev_usbdev_rq_init_seq(&pv->rtr, &usbdev_acm_read_cb, &pv->seq);

  /* Push initial request on stack */
  err = usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
  if (err)
    goto err_rbuffer;

  return 0;

err_rbuffer:
  usbdev_stack_free(&pv->usb, pv->rbuffer);
err_wbuffer:
  usbdev_stack_free(&pv->usb, pv->wbuffer);
err_ctrl:
  device_put_accessor(&pv->usb.base);
err_pv:
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(usbdev_acm_cleanup)
{
  struct usbdev_acm_private_s *pv = dev->drv_pv;

  if (usbdev_stack_service_unregister(&pv->usb, &pv->service))
    return -EBUSY;

  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
  dev_rq_queue_destroy(&pv->coding_notify_queue);
#endif

  usbdev_stack_free(&pv->usb, pv->rbuffer);
  usbdev_stack_free(&pv->usb, pv->wbuffer);

  device_put_accessor(&pv->usb.base);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(usbdev_acm_drv, 0, "CDC-ACM Dev", usbdev_acm,
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
               DRIVER_VALIO_METHODS(usbdev_acm_valio),
#endif
               DRIVER_CHAR_METHODS(usbdev_acm));

DRIVER_REGISTER(usbdev_acm_drv);

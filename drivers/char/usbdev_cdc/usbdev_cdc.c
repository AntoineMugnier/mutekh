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

  DEV_DECLARE_STATIC(usbdev_cdc0, "usbdev-char0", 0, usbdev_cdc_drv,
                     DEV_STATIC_RES_USBDEV_EP_MAP(0, 0x32, 0x01),
                     DEV_STATIC_RES_DEV_PARAM("usb-ctrl", "/max3420")
  );

*/

#define USBDEV_SERV_CHAR_BUFFER_SIZE 128
/* Bulk endpoint max packet size */
#define USBDEV_SERV_CHAR_BULK_SIZE 64

#define USBDEV_SERV_CHAR_INTF_CTRL 0
#define USBDEV_SERV_CHAR_INTF_DATA 1

//#define CONFIG_USBDEV_CDC_DEBUG

#ifdef CONFIG_USBDEV_CDC_DEBUG
# define usbdev_cdc_printk(...) do { printk(__VA_ARGS__); } while(0)
#else
# define usbdev_cdc_printk(...) do { } while(0)
#endif

/* Functionnal descriptor */

static const struct usbdev_class_cdc_func_info_s cdc_header =
{
  .f_replace = NULL,
  .hdr = {
    .head.bLength = sizeof(struct usb_cdc_header_descriptor_s),
    .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype = USB_CDC_FUNC_HEADER,
    .bcdCDC = endian_le16(0x0120),
  }
};
static const struct usbdev_class_cdc_func_info_s cdc_call_mgmt = 
{
  .f_replace = usbdev_cdc_desc_update,
  .call = {
    .head.bLength = sizeof(struct usb_cdc_call_mgmt_descriptor_s),
    .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype = USB_CDC_FUNC_CALL_MGMT,
    .bmCapabilities = USBDEV_SERV_CHAR_INTF_CTRL,
    .bDataInterface = USBDEV_SERV_CHAR_INTF_DATA
  }
};
static const struct usbdev_class_cdc_func_info_s cdc_acm =
{
  .f_replace = NULL,
  .acm = {
    .head.bLength = sizeof(struct usb_cdc_acm_descriptor_s),
    .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype = USB_CDC_FUNC_ACM,
    .bmCapabilities = 2,
  }
};
static const struct usbdev_class_cdc_func_info_s cdc_union =
{
  .f_replace = usbdev_cdc_desc_update,
  .un = {
    .head.bLength = sizeof(struct usb_cdc_union_descriptor_s),
    .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype = USB_CDC_FUNC_UNION,
    .bMasterInterface = USBDEV_SERV_CHAR_INTF_CTRL,
    .bSlaveInterface = USBDEV_SERV_CHAR_INTF_DATA
  }
};

/* Standart descriptors */

static struct usb_endpoint_descriptor_s ep_bulk_in =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_ENDPOINT_DESCRIPTOR,
  .bEndpointAddress = USB_EP_IN | 0, 
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(USBDEV_SERV_CHAR_BULK_SIZE),
  .bInterval = 0
};

static struct usb_endpoint_descriptor_s ep_bulk_out =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_ENDPOINT_DESCRIPTOR,
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
      .head.bDescriptorType = USB_INTERFACE_DESCRIPTOR,
      .bInterfaceNumber = USBDEV_SERV_CHAR_INTF_DATA,
      .bAlternateSetting = 0,
      .bNumEndpoints = 2,
      .bInterfaceClass = USB_CLASS_CDC_DATA,
      .bInterfaceSubClass = 0,
      .bInterfaceProtocol = 0,
      .iInterface =1
    },
    USBDEV_ENDPOINT(&ep_bulk_in, &ep_bulk_out)
  },
  USBDEV_INTERFACE_ALTERNATE()
};

static struct usb_endpoint_descriptor_s ep_irq_in =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_ENDPOINT_DESCRIPTOR,
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
      .head.bDescriptorType = USB_INTERFACE_DESCRIPTOR,
      .bInterfaceNumber = USBDEV_SERV_CHAR_INTF_CTRL,
      .bAlternateSetting = 0,
      .bNumEndpoints = 1,
      .bInterfaceClass = USB_CLASS_CDC,
      .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
      .bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
      .iInterface = 2
    },
    USBDEV_ENDPOINT(&ep_irq_in)
  },
  USBDEV_INTERFACE_ALTERNATE()
};

DRIVER_PV(struct usbdev_cdc_private_s
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

  struct dev_usbdev_request_s wtr;
  uint8_t * wbuffer;

  struct dev_usbdev_request_s rtr;
  uint8_t * rbuffer;

  bool_t read_started;
  bool_t write_started;

  size_t remaining;

  bool_t service_enabled;
  /* Notify next request of service disabling */
  bool_t disconnection;

  /* Service mapping */
  uint8_t interface_start_index;
  dev_usbdev_ep_map_t epi_map;
  dev_usbdev_ep_map_t epo_map;
});

static void usbdev_service_char_read(struct device_s *dev);
static void usbdev_service_char_write(struct device_s *dev);

static void usbdev_service_end_request(struct device_s *dev, bool_t read,
                                       error_t err)
{
  struct usbdev_cdc_private_s *pv = dev->drv_pv;
  struct dev_char_rq_s *rq;

  if (read)
    {
      pv->read_started = 0;
      rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q));
      if (rq)
        {
          rq->base.drvdata = NULL;
          dev_request_queue_pop(&pv->read_q);
        }
    }
  else
    {
      pv->write_started = 0;
      rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q));
      if (rq)
        {
          rq->base.drvdata = NULL;
          dev_request_queue_pop(&pv->write_q);
        }
    }

  if (rq == 0)
    return;

  rq->error = err;
  kroutine_exec(&rq->base.kr);
}

static void usbdev_service_try_read(struct device_s *dev)
{
  struct usbdev_cdc_private_s *pv = dev->drv_pv;
  struct dev_usbdev_request_s *tr = &pv->rtr;
  struct dev_char_rq_s *rq;

  while(1)
    {
      rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q));

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

static void usbdev_service_try_write(struct device_s *dev)
{
  struct usbdev_cdc_private_s *pv = dev->drv_pv;
  struct dev_usbdev_request_s *tr = &pv->wtr;

  uint8_t * p = tr->data = pv->wbuffer;

  tr->size = 0;

  while(1)
    {
      struct dev_char_rq_s *rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q));

      pv->write_started = 1;

      if (rq == NULL || tr->size == USBDEV_SERV_CHAR_BUFFER_SIZE)
        {
          if (tr->size)
            usbdev_service_char_write(dev);
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
  
      if (rq->type == DEV_CHAR_WRITE_PARTIAL || rq->size == 0)
        usbdev_service_end_request(dev, 0, 0);
   }
}

static KROUTINE_EXEC(usbdev_cdc_write_cb)
{
  struct dev_usbdev_request_s *tr = KROUTINE_CONTAINER(kr, *tr, base.kr);
  struct device_s *dev = tr->base.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);


  if (tr->error)
  /* USB reset or disconnected */
    {
      usbdev_cdc_printk("cb -EPIPE\n");
      usbdev_service_end_request(dev, 0, tr->error);
    }
  else
    usbdev_service_try_write(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static KROUTINE_EXEC(usbdev_cdc_read_cb)
{
  struct dev_usbdev_request_s *tr = KROUTINE_CONTAINER(kr, *tr, base.kr);
  struct device_s *dev = tr->base.pvdata;
  struct usbdev_cdc_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  if (tr->error)
  /* USB reset or disconnected */
    {
      usbdev_cdc_printk("cb -EPIPE\n");
      usbdev_service_end_request(dev, 1, tr->error);
    }
  else
    {
      pv->remaining = USBDEV_SERV_CHAR_BUFFER_SIZE - tr->size;
      usbdev_service_try_read(dev);
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static void usbdev_service_char_write(struct device_s *dev)
{
  struct usbdev_cdc_private_s *pv = dev->drv_pv;
  struct dev_usbdev_request_s *tr = &pv->wtr;

  tr->type = DEV_USBDEV_DATA_IN;
  tr->error = 0;
  tr->base.pvdata = dev;
        
  /* Push transfer to stack */
  error_t err = usbdev_stack_transfer(&pv->usb, &pv->service, tr, &ep_bulk_in);
   
  if (err)
  /* USB reset or disconnected */
    usbdev_service_end_request(dev, 0, err);
}

static void usbdev_service_char_read(struct device_s *dev)
{
  struct usbdev_cdc_private_s *pv = dev->drv_pv;
  struct dev_usbdev_request_s *tr = &pv->rtr;

  tr->size = USBDEV_SERV_CHAR_BUFFER_SIZE;
  tr->type = DEV_USBDEV_DATA_OUT;
  tr->data = pv->rbuffer;
  tr->error = 0;

  /* Push transfer to stack */
  error_t err = usbdev_stack_transfer(&pv->usb, &pv->service, tr, &ep_bulk_out);

  if (err)
  /* USB reset or disconnected */
    usbdev_service_end_request(dev, 1, err);
}

static KROUTINE_EXEC(usbdev_cdc_ctrl_cb);

static KROUTINE_EXEC(usbdev_cdc_transfer_cb)
{
  struct usbdev_service_rq_s *rq =  KROUTINE_CONTAINER(kr, *rq, kr);
  struct device_s *dev = rq->pvdata;
  struct usbdev_cdc_private_s *pv = dev->drv_pv;

  /* Update coding parameters */
  memcpy(&pv->coding, rq->ctrl.buffer, sizeof(struct usbdev_cdc_line_coding_s));

  rq->type = USBDEV_GET_COMMAND; 
  rq->error = 0;

  kroutine_init_deferred_seq(&rq->kr, &usbdev_cdc_ctrl_cb, &pv->seq);
  /* Push request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}

static KROUTINE_EXEC(usbdev_cdc_ctrl_cb)
{
  struct usbdev_service_rq_s *rq =  KROUTINE_CONTAINER(kr, *rq, kr);
  struct device_s *dev = rq->pvdata;
  struct usbdev_cdc_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  rq->error = 0;

  switch (rq->cmd)
    {
    case USBDEV_ENABLE_SERVICE:
      usbdev_cdc_printk("USB CHAR SERVICE ENABLE\n");
      /* Enable service and check queues */
      pv->service_enabled = 1;
 
      /* Start a USB bulk write transfer */
      usbdev_service_try_write(dev);
      usbdev_service_try_read(dev);

      break;
    
    case USBDEV_DISABLE_SERVICE:
      usbdev_cdc_printk("USB CHAR SERVICE DISABLED\n");
      pv->service_enabled = 0;
    
      if (!pv->read_started)
        pv->disconnection = 1;

      break;
    
    case USBDEV_PROCESS_CONTROL:
      {
        /* Only CDC control interface support specific request*/
        ensure (rq->intf == USBDEV_SERV_CHAR_INTF_CTRL);
   
        /* We use provided buffer to send/retrieve USB data */
        const struct usb_ctrl_setup_s *setup = (const void *)rq->ctrl.setup;
        uint16_t len = usb_setup_length_get(setup);

        ensure(len <= CONFIG_USBDEV_EP0_BUFFER_SIZE);

        rq->error = -EINVAL;
       
        switch (usb_setup_request_get(setup))
          {
          case USB_CDC_GET_LINE_CODING:
            usbdev_cdc_printk("USB REQUEST: GET LINE CODING\n");
            if (usb_setup_value_get(setup) ||
                usb_setup_direction_get(setup) != USB_DEVICE_TO_HOST)
              break;

            /* Copy data in provided buffer */
            memcpy(rq->ctrl.buffer, &pv->coding, len);
       
            rq->type = USBDEV_TRANSFER_DATA;
            rq->ctrl.size = len;
            rq->error = 0;
            kroutine_init_deferred_seq(&rq->kr, &usbdev_cdc_transfer_cb, &pv->seq);
            break;
       
          case USB_CDC_SET_LINE_CODING:
            usbdev_cdc_printk("USB REQUEST: SET LINE CODING\n");
            if (usb_setup_value_get(setup) ||
                usb_setup_direction_get(setup) != USB_HOST_TO_DEVICE)
              break;
       
            rq->type = USBDEV_TRANSFER_DATA;
            rq->ctrl.size = len;
            rq->error = 0;
            kroutine_init_deferred_seq(&rq->kr, &usbdev_cdc_transfer_cb, &pv->seq);
            break;
     
          case USB_CDC_SET_CONTROL_LINE_STATE:
            usbdev_cdc_printk("USB REQUEST: SET LINE STATE\n");
            rq->error = 0;
            break;
       
          default:
            usbdev_cdc_printk("USB CHAR SERVICE UNSUPPORTED REQUEST TYPE %d\n", usb_setup_request_get(setup));
            break;
          }
      }

    default:
      break;
  }

  if (rq->error)
    usbdev_cdc_printk("USB CHAR REQUEST ERROR\n");

  LOCK_RELEASE_IRQ(&dev->lock);
  /* Push request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}

/* USB CDC service */
static const struct usbdev_service_descriptor_s usb_cdc_service =
{
  /* Local descriptor */
  USBDEV_SERVICE_DESCRIPTOR(
      &interface_cdc_ctrl.intf.desc.head,
      &cdc_header.hdr.head,
      &cdc_call_mgmt.call.head,
      &cdc_acm.acm.head,
      &cdc_union.un.head,
      &ep_irq_in.head,
      &interface_cdc_data0.intf.desc.head,
      &ep_bulk_out.head,
      &ep_bulk_in.head
    ),

  USBDEV_INTERFACE(
    &interface_cdc_ctrl,
    &interface_cdc_data0
    ),

  .str_cnt = 2,
  .string = 
    "CDC Data Interface\0"
    "CDC Control Interface\0",
};

static DEV_CHAR_CANCEL(usbdev_cdc_cancel)
{
  struct device_s *dev = accessor->dev;
  struct usbdev_cdc_private_s   *pv = dev->drv_pv;

  error_t err = -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_POLL:
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
      err = -EBUSY;
      if (rq->base.drvdata == pv)
        {
          err = 0;
          rq->base.drvdata = NULL;
          dev_request_queue_remove(&pv->read_q, dev_char_rq_s_base(rq));
        }
      break;
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
      err = -EBUSY;
      if (rq->base.drvdata == pv)
        {
          err = 0;
          rq->base.drvdata = NULL;
          dev_request_queue_remove(&pv->write_q, dev_char_rq_s_base(rq));
        }
      break;
    default:
      break;                 
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_CHAR_REQUEST(usbdev_cdc_request)
{
  struct device_s               *dev = accessor->dev;
  struct usbdev_cdc_private_s   *pv = dev->drv_pv;

  assert(rq->size);

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
      dev_request_queue_pushback(&pv->read_q, dev_char_rq_s_base(rq));
      /* Start a USB bulk read transfer */
      if (!pv->read_started && pv->service_enabled)
        usbdev_service_try_read(dev);
      break;

    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
      dev_request_queue_pushback(&pv->write_q, dev_char_rq_s_base(rq));
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
    kroutine_exec(&rq->base.kr);
}

#define usbdev_cdc_use dev_use_generic

static DEV_INIT(usbdev_cdc_init)
{
  error_t err = 0;

  struct usbdev_cdc_private_s *pv;

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

  pv->wtr.base.pvdata = dev;
  pv->rtr.base.pvdata = dev;

  pv->coding.dwDTERate = endian_le32(115200);
  /* 1 Stop bit */
  pv->coding.bCharFormat = 0;   
  /* No parity */
  pv->coding.bParityType = 0;
  /* Data bits */
  pv->coding.dwDTERate = 8;

  pv->service.desc = &usb_cdc_service;
  pv->service.pv = dev;
  
  /* Get endpoint map from ressources */
  dev_res_get_usbdev_epmap(dev, &pv->service);

  err = usbdev_stack_service_register(&pv->usb, &pv->service);

  if (err)
    goto err_rbuffer;

  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

  struct usbdev_service_rq_s *rq = &pv->rq;

  rq->pvdata = dev;
  rq->type = USBDEV_GET_COMMAND; 
  rq->error = 0;

  kroutine_seq_init(&pv->seq);

  kroutine_init_deferred_seq(&rq->kr, &usbdev_cdc_ctrl_cb, &pv->seq);
  kroutine_init_deferred_seq(&pv->wtr.base.kr, &usbdev_cdc_write_cb, &pv->seq);
  kroutine_init_deferred_seq(&pv->rtr.base.kr, &usbdev_cdc_read_cb, &pv->seq);

  /* Push initial request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);

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

static DEV_CLEANUP(usbdev_cdc_cleanup)
{
  struct usbdev_cdc_private_s *pv = dev->drv_pv;

  if (usbdev_stack_service_unregister(&pv->usb, &pv->service))
    return -EBUSY;

  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);

  usbdev_stack_free(&pv->usb, pv->rbuffer);
  usbdev_stack_free(&pv->usb, pv->wbuffer);

  device_put_accessor(&pv->usb.base);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(usbdev_cdc_drv, 0, "USBDEV CDC", usbdev_cdc,
               DRIVER_CHAR_METHODS(usbdev_cdc));

DRIVER_REGISTER(usbdev_cdc_drv);


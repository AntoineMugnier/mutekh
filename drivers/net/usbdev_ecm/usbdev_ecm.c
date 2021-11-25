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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <device/class/usbdev.h>
#include <device/class/net.h>
#include <device/usb/usb.h>
#include <device/usb/cdc.h>

#include <mutek/buffer_pool.h>
#include <net/scheduler.h>
#include <net/layer.h>
#include <net/layer/ethernet.h>
#include <net/task.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

enum interface_id_e
{
  INTF_ID_CONTROL,
  INTF_ID_DATA,
};

enum string_id_e
{
  STRING_ID_INTF_CTRL = 1,
  STRING_ID_INTF_DUMB,
  STRING_ID_INTF_DATA,
  STRING_ID_MAC, // Dynamic
  STRING_COUNT = STRING_ID_MAC,
};

#define STRING_CONSTANT "CDC Control\0""Dumb\0""Ethernet"

//#define dprintk printk
#ifndef dprintk
# define dprintk(...) do{}while(0)
#endif

/* Functionnal descriptor */

static const struct usb_cdc_header_descriptor_s cdc_header =
{
  .head.bLength = sizeof(struct usb_cdc_header_descriptor_s),
  .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
  .bDescriptorSubtype = USB_CDC_DESC_FUNC_HEADER,
  .bcdCDC = endian_le16(0x0120),
};

static const struct usb_cdc_union_descriptor_s cdc_union =
{
  .head.bLength = sizeof(struct usb_cdc_union_descriptor_s),
  .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
  .bDescriptorSubtype = USB_CDC_DESC_FUNC_UNION,
  .bControlInterface = INTF_ID_CONTROL,
  .bSubordinateInterface = INTF_ID_DATA,
};

static const struct usb_cdc_ecm_descriptor_s cdc_ecm_desc =
{
  .head.bLength = sizeof(struct usb_cdc_ecm_descriptor_s),
  .head.bDescriptorType = USB_CDC_INTERFACE_DESCRIPTOR,
  .bDescriptorSubtype = USB_CDC_DESC_FUNC_ETHERNET,
  .iMACAddress = STRING_ID_MAC,
  .bmEthernetStatistics = 0,
  .wMaxSegmentSize = endian_le16(1500),
  .wNumberMCFilters = 0,
  .bNumberPowerFilters = 0,
};

/* Standart descriptors */

static const struct usb_endpoint_descriptor_s ep_bulk_in =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_IN | 0, 
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(CONFIG_DRIVER_USBDEV_ECM_DATA_MPS),
  .bInterval = 0,
};

static const struct usb_endpoint_descriptor_s ep_bulk_out =
{
  .head.bLength = sizeof(struct usb_endpoint_descriptor_s),
  .head.bDescriptorType = USB_DESC_ENDPOINT,
  .bEndpointAddress = USB_EP_OUT | 0, 
  .bmAttributes = USB_EP_BULK,
  .wMaxPacketSize = endian_le16(CONFIG_DRIVER_USBDEV_ECM_DATA_MPS),
  .bInterval = 0,
};

static const struct usbdev_interface_default_s ecm_intf_control =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = INTF_ID_CONTROL,
      .bAlternateSetting = 0,
      .bNumEndpoints = 0,
      .bInterfaceClass = USB_DEV_CLASS_COMM,
      .bInterfaceSubClass = USB_CDC_SUBCLASS_ECM,
      .bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
      .iInterface = STRING_ID_INTF_CTRL,
    },
  },
  USBDEV_INTERFACE_ALTERNATE(),
};

static const struct usbdev_interface_s ecm_intf_data;

static const struct usbdev_interface_default_s ecm_intf_data_nop =
{
  .intf = {
    .desc = {
      .head.bLength = sizeof(struct usb_interface_descriptor_s),
      .head.bDescriptorType = USB_DESC_INTERFACE,
      .bInterfaceNumber = INTF_ID_DATA,
      .bAlternateSetting = 0,
      .bNumEndpoints = 0,
      .bInterfaceClass = USB_DEV_CLASS_COMM_DATA,
      .bInterfaceSubClass = 0,
      .bInterfaceProtocol = 0,
      .iInterface = STRING_ID_INTF_DUMB,
    },
    USBDEV_ENDPOINT(),
  },
  USBDEV_INTERFACE_ALTERNATE(&ecm_intf_data),
};

static const struct usbdev_interface_s ecm_intf_data =
{
  .desc = {
    .head.bLength = sizeof(struct usb_interface_descriptor_s),
    .head.bDescriptorType = USB_DESC_INTERFACE,
    .bInterfaceNumber = INTF_ID_DATA,
    .bAlternateSetting = 1,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_DEV_CLASS_COMM_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = STRING_ID_INTF_DATA,
  },
  USBDEV_ENDPOINT(&ep_bulk_in, &ep_bulk_out),
};

static const struct usb_descriptor_header_s *service_desc_array[] =
{
  &ecm_intf_control.intf.desc.head,
  &cdc_header.head,
  &cdc_union.head,
  &cdc_ecm_desc.head,
  &ecm_intf_data_nop.intf.desc.head,
  &ecm_intf_data.desc.head,
  &ep_bulk_out.head,
  &ep_bulk_in.head,
};

static const struct usbdev_interface_default_s *interface_desc_array[] =
{
  &ecm_intf_control,
  &ecm_intf_data_nop,
};

struct ecm_private_s
{
  struct device_usbdev_s usb;

  struct net_layer_s *target;

  struct net_layer_s layer;
  net_task_queue_root_t txq;
  struct net_task_s *tx_current;
  struct buffer_s *rx_packet;
  uint8_t rx_ether_header[14];

  struct usbdev_service_s service;

  struct kroutine_sequence_s seq;
  struct usbdev_service_rq_s rq;
  struct dev_usbdev_rq_s tx_rq;
  struct dev_usbdev_rq_s rx_rq;
  uint8_t * tx_buffer;
  uint8_t * rx_buffer;

  bool_t service_enabled;
  bool_t rx_flush;

  // Device resource parameters
  dev_usbdev_ep_map_t epi_map;
  dev_usbdev_ep_map_t epo_map;
  uint8_t hwaddr[6];
  uint16_t mtu;

  char macAddressString[13];
};

STRUCT_COMPOSE(ecm_private_s, layer);
STRUCT_COMPOSE(ecm_private_s, service);
STRUCT_COMPOSE(ecm_private_s, rq);
STRUCT_COMPOSE(ecm_private_s, tx_rq);
STRUCT_COMPOSE(ecm_private_s, rx_rq);

DRIVER_PV(struct ecm_private_s);

static
USBDEV_REPLACE(cdc_ecm_desc_replace)
{
  const struct ecm_private_s *pv = const_ecm_private_s_from_service(it->service);
  size_t offset;

  switch (src->bDescriptorType) {
  case USB_CDC_INTERFACE_DESCRIPTOR: {
    const struct usb_class_descriptor_header_s *cd = (const void *)src;

    usbdev_cdc_descriptor_replace(it, src, dst, begin, end);

    switch (cd->bDescriptorSubtype) {
    case USB_CDC_DESC_FUNC_ETHERNET: {
      struct usb_cdc_ecm_descriptor_s *to_patch = (void *)dst;

      /* Replace wMaxSegmentSize */
      offset = offsetof(struct usb_cdc_ecm_descriptor_s, wMaxSegmentSize);
      if (begin <= offset && offset < end)
        *(uint8_t *)&to_patch->wMaxSegmentSize = pv->mtu & 0xff;
      if (begin <= offset + 1 && offset + 1 < end)
        *((uint8_t *)&to_patch->wMaxSegmentSize + 1) = pv->mtu >> 8;

      break;
    }
    }
    break;
  }
  }

  usbdev_descriptor_replace(it, src, dst, begin, end);
}

static
USBDEV_GET_STRING(cdc_ecm_get_string)
{
  const struct ecm_private_s *pv = const_ecm_private_s_from_service(service);

  printk("%s %d\n", __FUNCTION__, idx);

  if (idx == STRING_ID_MAC)
    return pv->macAddressString;

  return NULL;
}

static const struct usbdev_service_descriptor_s service_desc =
{
  .desc = service_desc_array,
  .desc_cnt = ARRAY_SIZE(service_desc_array),
  .string = STRING_CONSTANT,
  .str_cnt = STRING_COUNT,
  .intf = interface_desc_array,
  .intf_cnt = ARRAY_SIZE(interface_desc_array),
  .replace = cdc_ecm_desc_replace,
  .get_string = cdc_ecm_get_string,
};

static KROUTINE_EXEC(ecm_ctrl_cb)
{
  struct usbdev_service_rq_s *rq =  KROUTINE_CONTAINER(kr, *rq, kr);
  struct device_s *dev = rq->pvdata;
  struct ecm_private_s *pv = dev->drv_pv;

  rq->error = 0;

  switch (rq->cmd) {
  case USBDEV_ENABLE_SERVICE:
    printk("ECM service enabled\n");
    break;
    
  case USBDEV_DISABLE_SERVICE:
    printk("ECM service disabled\n");
    pv->service_enabled = 0;
    break;

  case USBDEV_CHANGE_INTERFACE:
    printk("ECM service interface changed to %d\n", rq->alternate);
    pv->service_enabled = rq->alternate;
    if (pv->service_enabled) {
      pv->rx_rq.size = CONFIG_DRIVER_USBDEV_ECM_DATA_MPS;
      pv->rx_rq.data = pv->rx_buffer;
      usbdev_stack_transfer(&pv->usb, &pv->service, &pv->rx_rq, &ep_bulk_out);
    }
    break;
    
  case USBDEV_PROCESS_CONTROL: {
    assert(rq->intf == INTF_ID_CONTROL);
   
    /* We use provided buffer to send/retrieve USB data */
    uint16_t len = usb_setup_length_get(rq->ctrl.setup);

    ensure(len <= CONFIG_USBDEV_EP0_BUFFER_SIZE);

    rq->error = -EINVAL;
       
    switch (usb_setup_request_get(rq->ctrl.setup)) {
    case USB_CDC_SET_ETHERNET_PACKET_FILTER:
      rq->error = 0;
      break;
       
    default:
      printk("ECM unsupported setup request 0x%x\n",
             usb_setup_request_get(rq->ctrl.setup));
      break;
    }
  }

  default:
    break;
  }

  if (rq->error)
    dprintk("ECM service request error %d\n", rq->error);

  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}

static
void ecm_usb_tx_next(struct ecm_private_s *pv);

static
void ecm_usb_link_notify(struct ecm_private_s *pv, bool_t up)
{
}

static KROUTINE_EXEC(ecm_bulk_in_done)
{
  struct dev_usbdev_rq_s *tx_rq =  KROUTINE_CONTAINER(kr, *tx_rq, base.kr);
  struct ecm_private_s *pv = ecm_private_s_from_tx_rq(tx_rq);

  if (pv->tx_rq.error && pv->tx_current) {
    net_task_destroy(pv->tx_current);
    pv->tx_current = NULL;
  }

  ecm_usb_tx_next(pv);
}

static KROUTINE_EXEC(ecm_bulk_out_done)
{
  struct dev_usbdev_rq_s *rx_rq =  KROUTINE_CONTAINER(kr, *rx_rq, base.kr);
  struct ecm_private_s *pv = ecm_private_s_from_rx_rq(rx_rq);
  struct net_task_s *task;
  struct net_addr_s src, dst;
  size_t used = CONFIG_DRIVER_USBDEV_ECM_DATA_MPS - pv->rx_rq.size;

  dprintk("%s %d %P\n", __FUNCTION__, used, pv->rx_buffer, used);

  if (pv->rx_flush) {
    dprintk(" ... flushing\n");

    if (used < CONFIG_DRIVER_USBDEV_ECM_DATA_MPS)
      pv->rx_flush = 0;
    goto again;
  }

  if (!pv->rx_packet) {
    dprintk(" ... no packet\n");

    if (used)
      dprintk(" ... no rx packet buffer available\n");

    pv->rx_flush = 1;
    goto again;
  }

  if (pv->rx_rq.error) {
    dprintk(" ... error %d\n", pv->rx_rq.error);
    pv->rx_packet->end = 0;
    pv->rx_flush = 1;
    goto again;
  }

  if (used > buffer_available(pv->rx_packet)) {
    dprintk(" ... rx packet buffer overflow\n");
    pv->rx_flush = 1;
    pv->rx_packet->end = 0;
    goto again;
  }

  if (pv->rx_packet->end == 0) {
    if (used < 14) {
      dprintk(" ... short initial packet in sequence\n");
      goto packet_reset;
    }

    memcpy(pv->rx_ether_header, pv->rx_buffer, 14);
    memcpy(pv->rx_packet->data, pv->rx_buffer + 14, used - 14);
    pv->rx_packet->end = used - 14;
  } else {
    memcpy(pv->rx_packet->data + pv->rx_packet->end, pv->rx_buffer, used);
    pv->rx_packet->end += used;
  }

  if (used == CONFIG_DRIVER_USBDEV_ECM_DATA_MPS)
    goto again;

  if (!pv->target) {
    dprintk("No target for packet %P\n", pv->rx_packet->data + pv->rx_packet->begin,
           pv->rx_packet->end - pv->rx_packet->begin);
    pv->rx_packet->end = 0;
    goto again;
  }

  task = net_scheduler_task_alloc(pv->layer.scheduler);

  if (!task)
    goto packet_reset;

  dprintk("Forwarding packet %P\n", pv->rx_packet->data + pv->rx_packet->begin,
          pv->rx_packet->end - pv->rx_packet->begin);

  memcpy(&dst.mac, pv->rx_ether_header, 6);
  memcpy(&src.mac, pv->rx_ether_header + 6, 6);
  dst.ethertype = endian_be16_na_load(pv->rx_ether_header + 12);

  net_task_inbound_push(task, pv->target, &pv->layer,
                        0, &src, &dst, pv->rx_packet);

  buffer_refdec(pv->rx_packet);
  pv->rx_packet = net_scheduler_packet_alloc(pv->layer.scheduler);
  if (!pv->rx_packet)
    goto again;

 packet_reset:
  pv->rx_packet->begin = pv->rx_packet->end = 0;

 again:
  if (!pv->service_enabled)
    return;

  pv->rx_rq.size = CONFIG_DRIVER_USBDEV_ECM_DATA_MPS;
  pv->rx_rq.data = pv->rx_buffer;
  usbdev_stack_transfer(&pv->usb, &pv->service, &pv->rx_rq, &ep_bulk_out);
}

static
void ecm_usb_tx_next(struct ecm_private_s *pv)
{
  size_t buffer_offset = 0;

  if (!pv->tx_current) {
  again:
    pv->tx_current = net_task_queue_pop(&pv->txq);

    if (!pv->tx_current)
      return;

    if (!pv->service_enabled) {
      net_task_destroy(pv->tx_current);
      pv->tx_current = NULL;
      goto again;
    }

    memcpy(pv->tx_buffer, pv->tx_current->packet.dst_addr.mac, 6);
    memcpy(pv->tx_buffer + 6, pv->tx_current->packet.src_addr.mac, 6);
    endian_be16_na_store(pv->tx_buffer + 12, pv->tx_current->packet.dst_addr.ethertype);
    buffer_offset = 14;
  }

  struct buffer_s *p = pv->tx_current->packet.buffer;
  size_t chunk_size = __MIN(CONFIG_DRIVER_USBDEV_ECM_DATA_MPS - buffer_offset,
                            p->end - p->begin);

  memcpy(pv->tx_buffer + buffer_offset, p->data + p->begin, chunk_size);
  p->begin += chunk_size;
  pv->tx_rq.data = pv->tx_buffer;
  pv->tx_rq.size = chunk_size + buffer_offset;

  if (p->begin == p->end && pv->tx_rq.size < CONFIG_DRIVER_USBDEV_ECM_DATA_MPS) {
    net_task_destroy(pv->tx_current);
    pv->tx_current = NULL;
  }

  pv->tx_rq.error = 0;

  usbdev_stack_transfer(&pv->usb, &pv->service, &pv->tx_rq, &ep_bulk_in);
}

static
void ecm_usb_outbound_enqueue(struct ecm_private_s *pv,
                              struct net_task_s *task)
{
  net_task_queue_pushback(&pv->txq, task);

  if (!pv->tx_current)
    ecm_usb_tx_next(pv);
}

static
void ecm_layer_destroyed(struct net_layer_s *layer)
{
  struct ecm_private_s *pv = ecm_private_s_from_layer(layer);

  memset(&pv->layer, 0, sizeof(pv->layer));

  ecm_usb_link_notify(pv, 0);
}

static
void ecm_layer_task_handle(struct net_layer_s *layer,
                      struct net_task_s *task)
{
  struct ecm_private_s *pv = ecm_private_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    ecm_usb_outbound_enqueue(pv, task);
    return;

  default:
    break;
  }

  net_task_destroy(task);
}

static
void ecm_layer_child_context_adjust(const struct net_layer_s *layer,
                                    struct net_layer_context_s *ctx)
{
  const struct ecm_private_s *pv = const_ecm_private_s_from_layer(layer);

  (void)pv;
}

static
void ecm_layer_context_changed(struct net_layer_s *layer)
{
  struct ecm_private_s *pv = ecm_private_s_from_layer(layer);

  (void)pv;
}

static
void ecm_layer_dangling(struct net_layer_s *layer)
{
  struct ecm_private_s *pv = ecm_private_s_from_layer(layer);

  if (pv->rx_packet) {
    buffer_refdec(pv->rx_packet);
    pv->rx_packet = NULL;
  }
}

static
error_t ecm_layer_bound(struct net_layer_s *layer,
                        void *addr,
                        struct net_layer_s *child)
{
  struct ecm_private_s *pv = ecm_private_s_from_layer(layer);

  if (pv->target)
    return -EBUSY;

  pv->target = child;

  return 0;
}

static
void ecm_layer_unbound(struct net_layer_s *layer,
                       struct net_layer_s *child)
{
  struct ecm_private_s *pv = ecm_private_s_from_layer(layer);

  assert(child == pv->target);

  pv->target = NULL;
}

static const struct net_layer_handler_s ecm_layer_handler =
{
  .destroyed = ecm_layer_destroyed,
  .task_handle = ecm_layer_task_handle,
  .child_context_adjust = ecm_layer_child_context_adjust,
  .context_changed = ecm_layer_context_changed,
  .dangling = ecm_layer_dangling,
  .bound = ecm_layer_bound,
  .unbound = ecm_layer_unbound,
};

#define ecm_use dev_use_generic

static DEV_NET_LAYER_CREATE(ecm_layer_create)
{
  struct device_s *dev = accessor->dev;
  struct ecm_private_s *pv = dev->drv_pv;
  error_t err;

  if (type != NET_LAYER_ETHERNET)
    return -ENOTSUP;

  if (pv->layer.scheduler)
    return -EBUSY;

  err = net_layer_init(&pv->layer,
                       &ecm_layer_handler, scheduler,
                       delegate, delegate_vtable);

  if (!err)
    *layer = &pv->layer;

  pv->rx_packet = net_scheduler_packet_alloc(pv->layer.scheduler);
  if (pv->rx_packet)
    pv->rx_packet->begin = pv->rx_packet->end = 0;

  ecm_usb_link_notify(pv, 1);

  return err;
}

static DEV_NET_GET_INFO(ecm_get_info)
{
  struct device_s *dev = accessor->dev;
  struct ecm_private_s *pv = dev->drv_pv;

  info->implemented_layers = bit(NET_LAYER_ETHERNET);
  memcpy(info->addr.mac, pv->hwaddr, 6);
  info->prefix_size = 0;
  info->mtu = 0;

  return 0;
}

static DEV_INIT(ecm_init)
{
  error_t err = 0;
  struct ecm_private_s *pv;
  const void *hwaddr;
  uintptr_t mtu;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "usb-ctrl", &pv->usb.base, DRIVER_CLASS_USBDEV);
  if (err)
    goto err_pv;

  pv->tx_buffer = usbdev_stack_allocate(&pv->usb, CONFIG_DRIVER_USBDEV_ECM_DATA_MPS);
  if (!pv->tx_buffer)
    goto err_ctrl;

  pv->rx_buffer = usbdev_stack_allocate(&pv->usb, CONFIG_DRIVER_USBDEV_ECM_DATA_MPS);
  if (!pv->rx_buffer)
    goto err_tx_buffer;

  err = device_get_param_blob(dev, "hwaddr", 0, &hwaddr);
  if (err)
    hwaddr = "\x02\x00\x00\x00\x00"/*\x00*/;
  memcpy(pv->hwaddr, hwaddr, 6);

  err = device_get_param_uint(dev, "mtu", &mtu);
  if (err)
    pv->mtu = 1500;
  else
    pv->mtu = mtu;

  for (uint_fast8_t i = 0; i < 6; ++i) {
    static const char *hex = "0123456789abcdef";
    pv->macAddressString[i * 2] = hex[pv->hwaddr[i] >> 4];
    pv->macAddressString[i * 2 + 1] = hex[pv->hwaddr[i] & 0xf];
  }
  pv->macAddressString[12] = 0;

  pv->service.desc = &service_desc;
  pv->service.pv = dev;
  
  dev_res_get_usbdev_epmap(dev, &pv->service);

  err = usbdev_stack_service_register(&pv->usb, &pv->service);

  net_task_queue_init(&pv->txq);

  if (err)
    goto err_rx_buffer;

  pv->pvdata = dev;
  pv->rq.type = USBDEV_GET_COMMAND; 
  pv->rq.error = 0;

  pv->tx_rq.pvdata = dev;
  pv->tx_rq.type = DEV_USBDEV_DATA_IN;
  pv->tx_rq.data = pv->tx_buffer;
  pv->tx_rq.error = 0;
  pv->tx_rq.rev = 1;

  pv->rx_rq.pvdata = dev;
  pv->rx_rq.type = DEV_USBDEV_DATA_OUT;
  pv->rx_rq.data = pv->rx_buffer;
  pv->rx_rq.error = 0;
  pv->rx_rq.rev = 1;

  kroutine_seq_init(&pv->seq);
  dev_timer_rq_init_seq(pv, &ecm_ctrl_cb, &pv->seq);
  dev_usbdev_rq_init_seq(&pv->tx_rq, &ecm_bulk_in_done, &pv->seq);
  dev_usbdev_rq_init_seq(&pv->rx_rq, &ecm_bulk_out_done, &pv->seq);

  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);

  return 0;

err_rx_buffer:
  usbdev_stack_free(&pv->usb, pv->rx_buffer);
err_tx_buffer:
  usbdev_stack_free(&pv->usb, pv->tx_buffer);
err_ctrl:
  device_put_accessor(&pv->usb.base);
err_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(ecm_cleanup)
{
  struct ecm_private_s *pv = dev->drv_pv;

  if (usbdev_stack_service_unregister(&pv->usb, &pv->service))
    return -EBUSY;

  usbdev_stack_free(&pv->usb, pv->rx_buffer);
  usbdev_stack_free(&pv->usb, pv->tx_buffer);

  device_put_accessor(&pv->usb.base);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(usbdev_ecm_drv, 0, "USB-ECM Device", ecm,
               DRIVER_NET_METHODS(ecm));

DRIVER_REGISTER(usbdev_ecm_drv);

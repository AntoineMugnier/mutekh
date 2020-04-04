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

#undef LOGK_MODULE_ID
#define LOGK_MODULE_ID "usbd"

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/usb/usb.h>
#include <device/class/usbdev.h>

#include <mutek/mem_alloc.h>

#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
#endif
#include <hexo/lock.h>
#include <hexo/interrupt.h>
#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/endian.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

GCT_CONTAINER_PROTOTYPES(usbdev_service, extern inline, usbdev_service,
                         init, destroy, push, remove, head, next);

/* Return string and size at index idx */
static bool_t usbdev_get_string(const char ** str, uint8_t idx, size_t *len)
{
  assert(*str);

  for (uint8_t i=0; i<idx + 1; i++)
    {
      if (i)
        *str += *len + 1;
      *len = strlen(*str);
      if (!*len)
        return 1;
    }
  return 0;
}

static void usbdev_set_ep0_state(struct dev_usbdev_context_s *ctx, 
                                 enum dev_usbdev_ctrl_ep_state_e state)
                             
{
  logk_trace("ep0 state: %N", state, ENUM_DESC_DEV_USBDEV_CTRL_EP_STATE_E);
  ctx->ep0_state = state;
}

__unused__
static void usbdev_dump_setup(const struct usb_ctrl_setup_s *setup)
{
  logk("  bRequestType: %x", usb_setup_reqtype_get(setup));
  logk("  bRequest: %x", usb_setup_request_get(setup));
  logk("  wValue: %x", usb_setup_value_get(setup));
  logk("  wIndex: %x", usb_setup_index_get(setup));
  logk("  wLength: %x", usb_setup_length_get(setup));
}

static size_t usbdev_copy_buffer(struct dev_usbdev_context_s *ctx,
                                 const struct usb_descriptor_header_s * hd)
{
  size_t idx = ctx->it.data_pkt_size % CONFIG_USBDEV_EP0_BUFFER_SIZE;

  uint8_t *d = (uint8_t*)ctx->data + idx;
  size_t dz = CONFIG_USBDEV_EP0_BUFFER_SIZE - idx;

  uint8_t *s = (uint8_t *)hd + ctx->it.desc_offset;
  size_t sz =  hd->bLength - ctx->it.desc_offset;

  size_t size = dz > sz ? sz : dz;

  ctx->it.desc_offset = size == sz ? 0 : ctx->it.desc_offset + size;
  ctx->it.data_pkt_size += size;

  memcpy(d, s, size);

  return size;
}

static inline bool_t usbdev_is_included(size_t index, size_t start, size_t cnt)
{
  return (index >= start && index < (start + cnt));
}

static error_t usbdev_is_valid_transfer(struct dev_usbdev_context_s *ctx,
                                        struct usbdev_endpoint_s *ep,
                                        struct dev_usbdev_rq_s *tr)
{
  if (ctx->state != DEV_USBDEV_CONFIGURED)
  /* Device not configured */
    return -EPIPE;
  else if (tr->rev != ep->rev)
  /* Endpoint configuration has changed */
    {
      tr->rev = ep->rev;
      return -EAGAIN;
    }
  return 0;
}

/* Return 0 if transfer is transfer is terminated  */
static bool_t usbdev_start_transfer(struct dev_usbdev_context_s *ctx,
                                    struct usbdev_endpoint_s *ep,
                                    struct dev_usbdev_rq_s *tr)
{
  error_t ret = ctx->ops->f_transfer(ctx, tr);

  switch (ret)
    {
    case 0:
      /* Transfer is terminated */
      break;
    case -EAGAIN:
      ep->busy = 1;
      /* Transfer will terminate later */
      return 1;
    default:
      /* An error has occured */
      tr->error = ret;
      break;
    }

  /* Remove from queue */
  dev_usbdev_rq_pop(&ep->queue);
  dev_usbdev_rq_done(tr);
  return 0;
}

static void usbdev_process_queue(struct dev_usbdev_context_s *ctx,
                                 struct usbdev_endpoint_s *ep)
{
  struct dev_usbdev_rq_s * tr;

  while(1)
    {
      tr = dev_usbdev_rq_head(&ep->queue);

      if (tr == NULL || ep->busy || ep->disabled)
        break;

      tr->error = usbdev_is_valid_transfer(ctx, ep, tr);

      if (tr->error)
        {
          /* Remove from queue */
          dev_usbdev_rq_pop(&ep->queue);
          dev_usbdev_rq_done(tr);
        }
      else if (usbdev_start_transfer(ctx, ep, tr))
        break;
    }
}

static void usbdev_disable_all_endpoints(struct dev_usbdev_context_s *ctx,
                                         enum usb_endpoint_dir_e dir)
{
  struct usbdev_endpoint_s *ep;
  uint16_t msk = dir == USB_EP_IN ? ctx->epi_msk : ctx->epo_msk;
  uint8_t idx;

  while(msk)
    {
      idx = bit_ctz(msk);
      ep = ctx->ops->f_endpoint(ctx, dir, idx);
      assert(ep != NULL);
      ep->disabled = 1;
      msk ^= 1 << idx;
    }
}

static void usbdev_disable_intf_endpoints(struct dev_usbdev_context_s *ctx,
                                         struct dev_usbdev_interface_cfg_s *cfg,
                                         uint8_t rev)
{
  struct usbdev_endpoint_s *ep;

  USBDEV_FOREACH_ENDPOINT(cfg->i, cfg->epi, cfg->epo,
    {
      if (usb_ep_is_in(epdesc))
        {
          ep = ctx->ops->f_endpoint(ctx, USB_EP_IN, epaddr);
          ep->disabled = 1;
          ep->rev = rev;
        }
      if (usb_ep_is_out(epdesc))
        {
          ep = ctx->ops->f_endpoint(ctx, USB_EP_OUT, epaddr);
          ep->disabled = 1;
          ep->rev = rev;
        }
    });
}

static void usbdev_disable_endpoint(struct dev_usbdev_context_s *ctx,
                                    struct dev_usbdev_interface_cfg_s *cfg,
                                    uint8_t rev)
{
  LOCK_SPIN_IRQ(&ctx->dev->lock);

  if (cfg)
  /* Lock old and new interface */
    {
      usbdev_disable_intf_endpoints(ctx, cfg + DEV_USBDEV_INTF_DISABLE, rev);
      usbdev_disable_intf_endpoints(ctx, cfg + DEV_USBDEV_INTF_ENABLE, rev);
    }
  else
    /* Disable all endpoint */
    {
      usbdev_disable_all_endpoints(ctx, USB_EP_IN);
      usbdev_disable_all_endpoints(ctx, USB_EP_OUT);
    }

  LOCK_RELEASE_IRQ(&ctx->dev->lock);
}

static void usbdev_enable_all_endpoints(struct dev_usbdev_context_s *ctx,
                                        enum usb_endpoint_dir_e dir)
{
  struct usbdev_endpoint_s *ep;
  uint16_t msk = dir == USB_EP_IN ? ctx->epi_msk : ctx->epo_msk;
  uint8_t idx = 0;

  while(msk)
    {
      idx = bit_ctz(msk);
      ep = ctx->ops->f_endpoint(ctx, dir, idx);
      assert(ep != NULL);
      ep->disabled = 0;
      /* Process pending request */
      usbdev_process_queue(ctx, ep);
      msk ^= 1 << idx;
    }
}

static void usbdev_enable_intf_endpoints(struct dev_usbdev_context_s *ctx,
                                        struct dev_usbdev_interface_cfg_s *cfg)
{
  struct usbdev_endpoint_s *ep;

  USBDEV_FOREACH_ENDPOINT(cfg->i, cfg->epi, cfg->epo,
    {
      if (usb_ep_is_in(epdesc))
        {
          ep = ctx->ops->f_endpoint(ctx, USB_EP_IN, epaddr);
          ep->disabled = 0;
          /* Process pending request */
          usbdev_process_queue(ctx, ep);
        }
      if (usb_ep_is_out(epdesc))
        {
          ep = ctx->ops->f_endpoint(ctx, USB_EP_OUT, epaddr);
          ep->disabled = 0;
          /* Process pending request */
          usbdev_process_queue(ctx, ep);
        }
    });
}

static void usbdev_enable_endpoint(struct dev_usbdev_context_s *ctx,
                                   struct dev_usbdev_interface_cfg_s *cfg)
{
  LOCK_SPIN_IRQ(&ctx->dev->lock);

  if (cfg)
  /* Unlock old and new interface */
    {
      usbdev_enable_intf_endpoints(ctx, cfg + DEV_USBDEV_INTF_DISABLE);
      usbdev_enable_intf_endpoints(ctx, cfg + DEV_USBDEV_INTF_ENABLE);
    }
  else
  /* Unlock all endpoint */
    {
      usbdev_enable_all_endpoints(ctx, USB_EP_IN);
      usbdev_enable_all_endpoints(ctx, USB_EP_OUT);
    }

  LOCK_RELEASE_IRQ(&ctx->dev->lock);
}

static void usbdev_service_ctrl(struct dev_usbdev_context_s *ctx,
                                enum usbdev_service_cmd_type_e type)
{
  struct usbdev_service_rq_s *rq = ctx->it.service->rq;

  ensure(rq);

  struct usb_ctrl_setup_s *setup = (void *)ctx->setup;

  rq->cmd = type;
  rq->error = 0;

  switch (rq->type)
    {
    case USBDEV_GET_COMMAND:
      switch (type)
        {
        case USBDEV_PROCESS_CONTROL:
          rq->intf = ctx->it.intf_index;
          rq->ctrl.setup = (void *)setup;
          rq->ctrl.buffer = ctx->data;
          rq->ctrl.size = CONFIG_USBDEV_EP0_BUFFER_SIZE;
          break;
        case USBDEV_CHANGE_INTERFACE:
          rq->intf = ctx->it.intf_index;
          rq->alternate = usb_setup_value_get(setup);
        default:
          break;
        }
      break;
    case USBDEV_TRANSFER_DATA:
      switch (type)
        {
        case USBDEV_DISABLE_SERVICE:
          rq->error = -EPIPE;
          break;
        default:
          abort();
        }
      break;
    }
  ctx->it.service->rq = NULL;
  /* Trigger service */
  kroutine_exec(&rq->kr);
}


static void usbdev_disable_service(struct dev_usbdev_context_s *ctx)
{
  GCT_FOREACH(usbdev_service, &ctx->service, service, {
    /* A request must be posted */
    ensure(service->rq);
    ctx->it.service = service;
    usbdev_service_ctrl(ctx, USBDEV_DISABLE_SERVICE);
    });
}

static error_t usbdev_ctrl_transaction(struct dev_usbdev_context_s *ctx,
                                       enum dev_usbdev_rq_type_e type)
{
  struct dev_usbdev_rq_s *tr = &ctx->tr;
  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;

  tr->ep = 0;
  tr->type = type;
  tr->data = ctx->data;
  tr->error = 0;
  tr->size = 0;
  tr->rem = 0;
  tr->event = 0;

  switch (type)
    {
    case DEV_USBDEV_CTRL_SETUP:
      /* Push setup request on endpoint 0 */
      tr->data = (uint8_t*)setup;
      tr->size = sizeof(struct usb_ctrl_setup_s);
      break;
    case DEV_USBDEV_DATA_IN:
      tr->rem = usb_setup_length_get(setup) - ctx->it.data_pkt_size;
      tr->size = ctx->it.data_pkt_size % CONFIG_USBDEV_EP0_BUFFER_SIZE;
      if (ctx->it.data_pkt_size && !tr->size)
        tr->size = CONFIG_USBDEV_EP0_BUFFER_SIZE;
      tr->rem += tr->size;
      break;
    case DEV_USBDEV_DATA_OUT:
      tr->size = CONFIG_USBDEV_EP0_BUFFER_SIZE;
      tr->rem = usb_setup_length_get(setup) - ctx->it.data_pkt_size;
    default:
      break;
    }

  error_t err = 0;

  LOCK_SPIN_IRQ(&ctx->dev->lock);

  err = ctx->ops->f_transfer(ctx, &ctx->tr);

  LOCK_RELEASE_IRQ(&ctx->dev->lock);

  return err;
}

static bool_t usbdev_is_service_setup(const struct usb_ctrl_setup_s *setup)
{
  if (usb_setup_type_get(setup) != USB_STANDARD)
    return 1;

  switch (usb_setup_request_get(setup))
    {
      case USB_GET_DESCRIPTOR:
        switch (usb_setup_recipient_get(setup))
          {
            case USB_INTERFACE:
              return 1;
            default:
              break;
          }
        break;

      default:
        break;
    }
  return 0;
}

static void usbdev_ep0_idle(struct dev_usbdev_context_s *ctx);

static inline void usbdev_ep0_status_done(struct dev_usbdev_context_s *ctx)
{
  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;

  ensure(ctx->tr.type == DEV_USBDEV_CTRL_STATUS_OUT ||
         ctx->tr.type == DEV_USBDEV_CTRL_STATUS_IN);

  if (!usbdev_is_service_setup(setup))
  /* Change device state if necessary */
    {
      switch (usb_setup_request_get(setup))
        {
        case USB_SET_ADDRESS:
          ctx->state = DEV_USBDEV_ADDRESS;
          break;
        case USB_SET_CONFIGURATION:
          if (usb_setup_value_get(setup))
            ctx->state = DEV_USBDEV_CONFIGURED;
          else
            ctx->state = DEV_USBDEV_ADDRESS;
          /* Process pending transfer */
          usbdev_enable_endpoint(ctx, NULL);
          break;
        default:
          break;
        }
    }
  return usbdev_ep0_idle(ctx);
}

static void usbdev_fsm_handle_event(struct dev_usbdev_context_s *ctx, uint8_t event);

static inline void usbdev_ep0_status_in(struct dev_usbdev_context_s *ctx)
{
  usbdev_set_ep0_state(ctx, EP0_STATUS_WAIT);

  error_t err = usbdev_ctrl_transaction(ctx, DEV_USBDEV_CTRL_STATUS_IN);

  switch (err)
    {
    case 0:
      return usbdev_ep0_status_done(ctx);
    case -EIO:
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static void usbdev_enable_service(struct dev_usbdev_context_s *ctx)
{
  if (ctx->service_cnt == 0)
    return usbdev_ep0_status_in(ctx);

  usbdev_set_ep0_state(ctx, EP0_SRVC_ENABLE_WAIT);

  GCT_FOREACH(usbdev_service, &ctx->service, service, {
    /* A request must be posted */
    ensure(service->rq);

    ctx->it.service = service;
    usbdev_service_ctrl(ctx, USBDEV_ENABLE_SERVICE);
    });
}


void * usbdev_stack_allocate(const struct device_usbdev_s *dev, size_t size)
{
  struct dev_usbdev_context_s *ctx = device_usbdev_context(dev);
  return ctx->ops->f_alloc(ctx, size);
}

void  usbdev_stack_free(const struct device_usbdev_s *dev, void * ptr)
{
  struct dev_usbdev_context_s *ctx = device_usbdev_context(dev);
  ctx->ops->f_free(ctx, ptr);
}

error_t usbdev_stack_service_register(const struct device_usbdev_s *dev,
                                      struct usbdev_service_s *service)
{
  struct dev_usbdev_context_s *ctx = device_usbdev_context(dev);
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->dev->lock);

  if (ctx->state > DEV_USBDEV_POWERED)
    {
      err = -EBUSY;
    }
  else
    {
      service->rq = NULL;
      usbdev_service_push(&ctx->service, service);
    }

  LOCK_RELEASE_IRQ(&dev->dev->lock);

  return err;
}

/* Remove a USB service on a USB controller */

error_t usbdev_stack_service_unregister(const struct device_usbdev_s *dev,
                                        struct usbdev_service_s *service)
{
  struct dev_usbdev_context_s *ctx = device_usbdev_context(dev);
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->dev->lock);

  if (ctx->state > DEV_USBDEV_POWERED)
    err = -EBUSY;
  else
    usbdev_service_remove(&ctx->service, service);

  LOCK_RELEASE_IRQ(&dev->dev->lock);

  return err;
}

/* This returns a global endpoint address from an endpoint descriptor */

uint8_t usbdev_stack_get_ep_addr(const struct usb_endpoint_descriptor_s *desc,
                                  dev_usbdev_ep_map_t mapi,
                                  dev_usbdev_ep_map_t mapo)
{
  uint8_t ret;

  if (usb_ep_type_get(desc) == USB_EP_CONTROL)
    {
      ret = (mapi >> (usb_ep_num_get(desc) << 2)) & 0xF;
      assert(ret == (mapo >> ((usb_ep_num_get(desc) << 2)) & 0xF));
    }
  else if ((usb_ep_dir_get(desc) == USB_EP_IN))
    ret = (mapi >> (usb_ep_num_get(desc) << 2)) & 0xF;
  else
    ret = (mapo >> (usb_ep_num_get(desc) << 2)) & 0xF;

  /* Endpoint address can not be 0 */
  assert(ret);

  return ret;
}

/* This set information on USB device */

void usbdev_stack_set_device_info(const struct device_usbdev_s *dev,
                                  const struct usbdev_device_info_s *info)
{
  struct dev_usbdev_context_s *ctx = device_usbdev_context(dev);
  ctx->devinfo = info;
  assert(!(CONFIG_USBDEV_EP0_BUFFER_SIZE % usbdev_stack_get_ep0_mps(ctx)));
}

static void usbdev_wait_service_ready(struct dev_usbdev_context_s *ctx)
{
  /* Ensure that all service are ready */
  GCT_FOREACH(usbdev_service, &ctx->service, service, {
      if (service->rq == NULL)
      /* A service is not ready yet */
        {
          ctx->state = DEV_USBDEV_WAIT_SERIVCE_READY;
          return;
        }
    });

  ctx->state = DEV_USBDEV_DEFAULT;
  return usbdev_ep0_idle(ctx);
}

/* All service must be registered when this function is called. */

static void usbdev_service_parse(struct dev_usbdev_context_s *ctx)
{
  uint8_t intf_cnt = 0;

  ctx->service_cnt = 0;

  GCT_FOREACH(usbdev_service, &ctx->service, service, {
      service->start.id = ctx->service_cnt++;
      /* Retrieve total number of interface */
      intf_cnt += service->desc->intf_cnt;
    });

  ensure(intf_cnt <= CONFIG_USBDEV_MAX_INTERFACE_COUNT);

  uint8_t str = ctx->devinfo->str_cnt;
  uint8_t intf = 0;
  const struct usbdev_service_descriptor_s * s;

  GCT_FOREACH(usbdev_service, &ctx->service, service, {

    service->start.intf = intf;
    service->start.str = str;

    s = service->desc;

    intf +=  s->intf_cnt;
    str +=  s->str_cnt;
  });

  return usbdev_wait_service_ready(ctx);
}

static void usbdev_service_data(struct dev_usbdev_context_s *ctx, error_t err)
{
  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;
  struct usbdev_service_rq_s *rq = ctx->it.service->rq;
  ensure(rq && rq->type == USBDEV_TRANSFER_DATA);

  rq = ctx->it.service->rq;
  rq->error = err;
  rq->ctrl.buffer = ctx->data;
  rq->ctrl.size = CONFIG_USBDEV_EP0_BUFFER_SIZE;

  if (usb_setup_direction_get(setup) == USB_HOST_TO_DEVICE)
    {
      uint32_t r = ctx->it.data_pkt_size % CONFIG_USBDEV_EP0_BUFFER_SIZE;
      if (r)
        rq->ctrl.size = r;
    }

  ctx->it.service->rq = NULL;
  /* Trigger service */
  kroutine_exec(&rq->kr);
}

/* This is always executed from an interrupt handler * */

static void usbdev_ep0_service_setup(struct dev_usbdev_context_s *ctx)
{
  logk_trace("Service setup");

  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;

  //  usbdev_dump_setup(setup);

  usbdev_set_ep0_state(ctx, EP0_SRVC_STATUS_IN);

  if (usb_setup_length_get(setup))
    {
      if (usb_setup_direction_get(setup) == USB_DEVICE_TO_HOST)
        usbdev_set_ep0_state(ctx, EP0_SRVC_DATA_IN);
      else
        usbdev_set_ep0_state(ctx, EP0_SRVC_DATA_OUT);
    }

  uint8_t intf = usb_setup_index_get(setup);

  GCT_FOREACH(usbdev_service, &ctx->service, service, {
    if (usbdev_is_included(intf, service->start.intf, service->desc->intf_cnt))
      {
        ctx->it.service = service;
        ctx->it.intf_index = intf - service->start.intf;
        /* Call service kroutine */
        return usbdev_service_ctrl(ctx, USBDEV_PROCESS_CONTROL);
      }
    });

  abort();
}

static void usbdev_ep0_standard_setup(struct dev_usbdev_context_s *ctx);

static void usbdev_fsm_reset_ep0(struct dev_usbdev_context_s *ctx)
{
  /* Enable endpoint 0 */
  ctx->cfg.type = DEV_USBDEV_CONFIGURE;
  ctx->cfg.intf = NULL;
  ctx->cfg.error = 0;

  error_t err;

  LOCK_SPIN_IRQ(&ctx->dev->lock);

  err = ctx->ops->f_config(ctx, &ctx->cfg);

  LOCK_RELEASE_IRQ(&ctx->dev->lock);

  switch (err)
    {
    case -EAGAIN:
      return;
    case 0:
      /* Ready to process setup packet */
      return usbdev_service_parse(ctx);
    default:
      abort();
    }

}

static void usbdev_fsm_reset_address(struct dev_usbdev_context_s *ctx)
{
  ctx->state = DEV_USBDEV_POWERED_TO_DEFAULT;
  ctx->cfg.type = DEV_USBDEV_SET_ADDRESS;
  ctx->cfg.addr = 0;
  ctx->cfg.error = 0;

  error_t err;

  LOCK_SPIN_IRQ(&ctx->dev->lock);

  err = ctx->ops->f_config(ctx, &ctx->cfg);

  LOCK_RELEASE_IRQ(&ctx->dev->lock);

  switch (err)
    {
    case -EAGAIN:
      return;
    case 0:
      /* Configure endpoint 0 */
      return usbdev_fsm_reset_ep0(ctx);
    default:
      abort();
    }
}

static void usbdev_get_event(struct dev_usbdev_context_s *ctx)
{
  /* Get any event on device */
  ctx->tr.type = DEV_USBDEV_EVENT;
  ctx->tr.error = 0;
  ctx->tr.ep = 0;
  error_t err = ctx->ops->f_transfer(ctx, &ctx->tr);

  switch (err)
    {
    case -EIO:
    case 0:
      /* An event was pending */
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static void usbdev_fsm_attached(struct dev_usbdev_context_s *ctx);

static void usbdev_fsm_detached(struct dev_usbdev_context_s *ctx)
{
  logk_trace("Detached %d", ctx->event);

  ctx->state = DEV_USBDEV_DETACHED;

  if (ctx->event & USBDEV_EVENT_CONNECT)
    {
      ctx->event &= ~USBDEV_EVENT_CONNECT;
      return usbdev_fsm_attached(ctx);
    }

  return usbdev_get_event(ctx);
}

static void usbdev_fsm_attached(struct dev_usbdev_context_s *ctx)
{
  logk_trace("Attached %d", ctx->event);

  ctx->state = DEV_USBDEV_ATTACHED;

  if (ctx->event & USBDEV_EVENT_DISCONNECT)
    {
      ctx->event &= ~USBDEV_EVENT_DISCONNECT;
      return usbdev_fsm_detached(ctx);
    }

  if (ctx->event & USBDEV_EVENT_RESET)
    {
      ctx->event &= ~USBDEV_EVENT_RESET;
      return usbdev_fsm_reset_address(ctx);
    }

  if (ctx->event & USBDEV_EVENT_IDLE)
    ctx->event &= ~USBDEV_EVENT_IDLE;

  return usbdev_get_event(ctx);
}

static void usbdev_fsm_default(struct dev_usbdev_context_s *ctx)
{
  logk_trace("Default %N", ctx->event, ENUM_DESC_DEV_USBDEV_EVENT_E);

  ctx->state = DEV_USBDEV_DEFAULT;

  switch (ctx->ep0_state)
    {
    case EP0_SETUP_WAIT:
    case EP0_CTRL_SET_INTERFACE_WAIT:
    case EP0_SET_INTERFACE_WAIT:
    case EP0_CTRL_CONFIGURATION_WAIT:
    case EP0_CTRL_UNCONFIGURATION_WAIT:
    case EP0_SRVC_DATA_OUT_WAIT:
    case EP0_DATA_OUT_WAIT:
    case EP0_SRVC_DATA_IN_WAIT:
    case EP0_DATA_IN_WAIT:
    case EP0_DATA_IN_ZERO_WAIT:
    case EP0_STATUS_WAIT:
    case EP0_STALL_WAIT:
      if (ctx->event & USBDEV_EVENT_DISCONNECT)
        {
          ctx->event &= ~USBDEV_EVENT_DISCONNECT;
          return usbdev_fsm_detached(ctx);
        }
      if (ctx->event & USBDEV_EVENT_STOP)
        {
          ctx->event &= ~USBDEV_EVENT_DISCONNECT;
          return usbdev_fsm_attached(ctx);
        }
      if (ctx->event & USBDEV_EVENT_RESET)
        {
          ctx->event &= ~USBDEV_EVENT_RESET;
          return usbdev_fsm_reset_address(ctx);
        }
      if (ctx->event & USBDEV_EVENT_IDLE)
        {
          ctx->event &= ~USBDEV_EVENT_IDLE;
          return usbdev_ep0_idle(ctx);
        }
      break;
    default:
      abort();
    }
}

static void usbdev_fsm_unconfiguration_done(struct dev_usbdev_context_s *ctx)
{
  /* Terminate transfers */
  usbdev_enable_endpoint(ctx, NULL);

  switch (ctx->state)
    {
    case DEV_USBDEV_CONFIGURED_TO_POWERED:
    case DEV_USBDEV_ADDRESS_TO_POWERED:
      return usbdev_fsm_attached(ctx);
    case DEV_USBDEV_CONFIGURED_TO_DEFAULT:
    case DEV_USBDEV_ADDRESS_TO_DEFAULT:
    case DEV_USBDEV_POWERED_TO_DEFAULT:
      return usbdev_fsm_reset_address(ctx);
    case DEV_USBDEV_ADDRESS_TO_DETACHED:
    case DEV_USBDEV_CONFIGURED_TO_DETACHED:
      return usbdev_fsm_detached(ctx);
    default:
      abort();
    }
}

static void usbdev_fsm_unconfiguration(struct dev_usbdev_context_s *ctx)
{
  /* Unconfigure controller */
  ctx->cfg.type = DEV_USBDEV_UNCONFIGURE;
  ctx->cfg.error = 0;

  error_t err;

  LOCK_SPIN_IRQ(&ctx->dev->lock);

  err = ctx->ops->f_config(ctx, &ctx->cfg);

  LOCK_RELEASE_IRQ(&ctx->dev->lock);

  switch (err)
    {
    case 0:
      usbdev_fsm_unconfiguration_done(ctx);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static void usbdev_fsm_addressed(struct dev_usbdev_context_s *ctx)
{
  logk_trace("Addressed");

  ctx->state = DEV_USBDEV_ADDRESS;

  switch (ctx->ep0_state)
    {
    case EP0_SETUP_WAIT:
    case EP0_CTRL_SET_INTERFACE_WAIT:
    case EP0_SET_INTERFACE_WAIT:
    case EP0_CTRL_CONFIGURATION_WAIT:
    case EP0_CTRL_UNCONFIGURATION_WAIT:
    case EP0_SRVC_DATA_OUT_WAIT:
    case EP0_SRVC_DATA_IN_WAIT:
    case EP0_DATA_OUT_WAIT:
    case EP0_DATA_IN_WAIT:
    case EP0_DATA_IN_ZERO_WAIT:
    case EP0_STATUS_WAIT:
    case EP0_STALL_WAIT:
      if (ctx->event & USBDEV_EVENT_DISCONNECT)
        {
          ctx->event &= ~USBDEV_EVENT_DISCONNECT;
          ctx->state = DEV_USBDEV_ADDRESS_TO_DETACHED;
          /* Disable endpoints  */
          usbdev_disable_endpoint(ctx, NULL, 0);
          if (ctx->service_state)
            goto disable;
          return usbdev_fsm_detached(ctx);
        }
      if (ctx->event & USBDEV_EVENT_STOP)
        {
          ctx->event &= ~USBDEV_EVENT_STOP;
          ctx->state = DEV_USBDEV_ADDRESS_TO_POWERED;
          /* Disable endpoints  */
          usbdev_disable_endpoint(ctx, NULL, 0);
          if (ctx->service_state)
            goto disable;
          return usbdev_fsm_attached(ctx);
        }
      if (ctx->event & USBDEV_EVENT_RESET)
        {
          ctx->event &= ~USBDEV_EVENT_RESET;
          ctx->state = DEV_USBDEV_ADDRESS_TO_DEFAULT;
          /* Disable endpoints  */
          usbdev_disable_endpoint(ctx, NULL, 0);
          if (ctx->service_state)
            goto disable;
          return usbdev_fsm_reset_address(ctx);
        }
      if (ctx->event & USBDEV_EVENT_IDLE)
        ctx->event &= ~USBDEV_EVENT_IDLE;
      return;
    default:
      abort();
    }

disable:
  /* Disable all services */
  return usbdev_disable_service(ctx);
}

static void usbdev_fsm_configured(struct dev_usbdev_context_s *ctx)
{
  logk_trace("Configured %N", ctx->ep0_state, ENUM_DESC_DEV_USBDEV_CTRL_EP_STATE_E);

  ctx->state = DEV_USBDEV_CONFIGURED;

  switch (ctx->ep0_state)
    {
    case EP0_SETUP_WAIT:
    case EP0_CTRL_SET_INTERFACE_WAIT:
    case EP0_SET_INTERFACE_WAIT:
    case EP0_CTRL_CONFIGURATION_WAIT:
    case EP0_CTRL_UNCONFIGURATION_WAIT:
    case EP0_SRVC_DATA_OUT_WAIT:
    case EP0_SRVC_DATA_IN_WAIT:
    case EP0_DATA_OUT_WAIT:
    case EP0_DATA_IN_WAIT:
    case EP0_DATA_IN_ZERO_WAIT:
    case EP0_STATUS_WAIT:
    case EP0_STALL_WAIT:
      if (ctx->event & USBDEV_EVENT_DISCONNECT)
        {
          ctx->state = DEV_USBDEV_CONFIGURED_TO_DETACHED;
          ctx->event &= ~USBDEV_EVENT_DISCONNECT;
          break;
        }
      if (ctx->event & USBDEV_EVENT_STOP)
        {
          ctx->state = DEV_USBDEV_CONFIGURED_TO_POWERED;
          ctx->event &= ~USBDEV_EVENT_STOP;
          break;
        }
      if (ctx->event & USBDEV_EVENT_RESET)
        {
          ctx->state = DEV_USBDEV_CONFIGURED_TO_DEFAULT;
          ctx->event &= ~USBDEV_EVENT_RESET;
          break;
        }
      if (ctx->event & USBDEV_EVENT_IDLE)
        ctx->event &= ~USBDEV_EVENT_IDLE;
      return;
    default:
      abort();
    }

  /* Disable endpoints  */
  usbdev_disable_endpoint(ctx, NULL, 0);
  /* Disable all services */
  if (ctx->service_state)
  /* Disable services */
    return usbdev_disable_service(ctx);
  else
    return usbdev_fsm_unconfiguration(ctx);
}
/* This is always executed from a kroutine deferred or from a service request * */

static void usbdev_fsm_handle_event(struct dev_usbdev_context_s *ctx, uint8_t event)
{
  ctx->event |= event;

  logk_trace("handle_event state %N events %N",
             ctx->state, ENUM_DESC_DEV_USBDEV_STATE_E,
             ctx->event, ENUM_DESC_DEV_USBDEV_EVENT_E);
  
  switch (ctx->state)
    {
      case DEV_USBDEV_DETACHED:
        return usbdev_fsm_detached(ctx);
      case DEV_USBDEV_ATTACHED:
      case DEV_USBDEV_POWERED:
        return usbdev_fsm_attached(ctx);
      case DEV_USBDEV_DEFAULT:
        return usbdev_fsm_default(ctx);
      case DEV_USBDEV_ADDRESS:
        return usbdev_fsm_addressed(ctx);
      case DEV_USBDEV_CONFIGURED:
        return usbdev_fsm_configured(ctx);
      default:
        abort();
    }
}

static void usbdev_ep0_idle(struct dev_usbdev_context_s *ctx)
{
  struct usb_ctrl_setup_s *setup = (void *)ctx->setup;

  /* Reset iterator and setup */
  memset(&ctx->it, 0, sizeof(ctx->it));
  memset(setup, 0, 8);

  usbdev_set_ep0_state(ctx, EP0_SETUP_WAIT);

  /* Get next control packet */
  error_t err = usbdev_ctrl_transaction(ctx, DEV_USBDEV_CTRL_SETUP);

  switch (err)
    {
    case 0:
      if (usbdev_is_service_setup(setup))
      /* Non standard setup packet */
        return usbdev_ep0_service_setup(ctx);
      else
        return usbdev_ep0_standard_setup(ctx);
    case -EIO:
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static inline void usbdev_ep0_status_out(struct dev_usbdev_context_s *ctx)
{
  usbdev_set_ep0_state(ctx, EP0_STATUS_WAIT);

  error_t err = usbdev_ctrl_transaction(ctx, DEV_USBDEV_CTRL_STATUS_OUT);

  switch (err)
    {
    case 0:
      return usbdev_ep0_idle(ctx);
    case -EIO:
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static bool_t usbdev_data_in_stage_done(struct dev_usbdev_context_s *ctx)
{
  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;

  if (ctx->it.data_pkt_size == usb_setup_length_get(setup) ||
     (ctx->it.data_pkt_size % usbdev_stack_get_ep0_mps(ctx)))
    /* No need of zero-lenght packet */
    return 1;

  return 0;
}

static void usbdev_ep0_zero_len_packet(struct dev_usbdev_context_s *ctx)
{
  if (usbdev_data_in_stage_done(ctx))
    /* Data stage is terminated */
    return usbdev_ep0_status_out(ctx);

  /* Zero length packet must be send */
  usbdev_set_ep0_state(ctx, EP0_DATA_IN_ZERO_WAIT);
  ctx->it.data_pkt_size = 0;

  error_t err = usbdev_ctrl_transaction(ctx, DEV_USBDEV_DATA_IN);

  switch (err)
    {
    case 0:
      return usbdev_ep0_status_out(ctx);
    case -EIO:
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static void usbdev_ep0_data_in_done(struct dev_usbdev_context_s *ctx)
{
  if (ctx->ep0_state == EP0_SRVC_DATA_IN_WAIT)
    {
      usbdev_set_ep0_state(ctx, EP0_SRVC_DATA_IN);
      /* Call service kroutine */
      return usbdev_service_data(ctx, 0);
    }

  if(ctx->it.done)
    return usbdev_ep0_zero_len_packet(ctx);
  else
    /* Standard data stage not terminated */
    return usbdev_ep0_standard_setup(ctx);
}

static void usbdev_ep0_data_in(struct dev_usbdev_context_s *ctx)
{
  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;

  /* Send data to host */
  if (ctx->it.data_pkt_size >= usb_setup_length_get(setup))
    {
      ctx->it.data_pkt_size = usb_setup_length_get(setup);
      ctx->it.done = 1;
    }

  if (ctx->ep0_state == EP0_SRVC_DATA_IN)
    usbdev_set_ep0_state(ctx, EP0_SRVC_DATA_IN_WAIT);
  else
    usbdev_set_ep0_state(ctx, EP0_DATA_IN_WAIT);

  error_t err = usbdev_ctrl_transaction(ctx, DEV_USBDEV_DATA_IN);

  switch (err)
    {
    case 0:
      return usbdev_ep0_data_in_done(ctx);
    case -EIO:
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static inline void usbdev_ep0_stall(struct dev_usbdev_context_s *ctx, enum dev_usbdev_rq_type_e type)
{
  usbdev_set_ep0_state(ctx, EP0_STALL_WAIT);

  error_t err = usbdev_ctrl_transaction(ctx, type);

  switch (err)
    {
    case 0:
      return usbdev_ep0_idle(ctx);
    case -EIO:
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static void usbdev_ep0_unconfigure(struct dev_usbdev_context_s *ctx)
{
/* Unconfigure whole controller except endpoint 0 */

  ctx->ep0_state = EP0_CTRL_UNCONFIGURATION_WAIT;

  ctx->cfg.type = DEV_USBDEV_UNCONFIGURE;
  ctx->cfg.error = 0;

  error_t err;

  LOCK_SPIN_IRQ(&ctx->dev->lock);

  err = ctx->ops->f_config(ctx, &ctx->cfg);

  LOCK_RELEASE_IRQ(&ctx->dev->lock);

  switch (err)
    {
    case 0:
      return usbdev_ep0_status_in(ctx);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

/* Data stage is terminated or input buffer is full */
static void usbdev_ep0_data_out_stage_done(struct dev_usbdev_context_s *ctx)
{
  ctx->it.data_pkt_size += CONFIG_USBDEV_EP0_BUFFER_SIZE - ctx->tr.size;

  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;

  ctx->it.done = (usb_setup_length_get(setup) == ctx->it.data_pkt_size);

  if (ctx->tr.size)
    assert(ctx->it.done);
}

static void usbdev_ep0_data_out_done(struct dev_usbdev_context_s *ctx)
{
  /* Check if data stage is valid */
  usbdev_ep0_data_out_stage_done(ctx);

  if (ctx->ep0_state == EP0_SRVC_DATA_OUT_WAIT)
    {
      usbdev_set_ep0_state(ctx, EP0_SRVC_DATA_OUT);
      /* Call service kroutine */
      return usbdev_service_data(ctx, 0);
    }

  /* Standard data stage not terminated */
  return usbdev_ep0_standard_setup(ctx);
}

static void usbdev_ep0_data_out(struct dev_usbdev_context_s *ctx)
{
  if (ctx->ep0_state == EP0_SRVC_DATA_OUT)
    usbdev_set_ep0_state(ctx, EP0_SRVC_DATA_OUT_WAIT);
  else
    usbdev_set_ep0_state(ctx, EP0_DATA_OUT_WAIT);

  error_t err = usbdev_ctrl_transaction(ctx, DEV_USBDEV_DATA_OUT);

  switch (err)
    {
    case 0:
      return usbdev_ep0_data_out_done(ctx);
    case -EIO:
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EAGAIN:
    default:
      break;
    }
}

static void usbdev_set_configuration(struct dev_usbdev_context_s *ctx, const struct usb_ctrl_setup_s *setup)
{

  uint16_t cfg_number = usb_setup_value_get(setup);
  uint8_t i = 0;
  error_t err;

  logk_trace("Set Configuration %d", cfg_number);

  /* Only one configuration */
  if (cfg_number > 1)
    return usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_IN_STALL);

  switch (ctx->state)
    {
    case DEV_USBDEV_CONFIGURED:
      /* Nothing to do device is already configured */
      if (cfg_number == 1)
        return usbdev_ep0_status_in(ctx);

      usbdev_set_ep0_state(ctx, EP0_SRVC_DISABLE_WAIT);
      usbdev_disable_endpoint(ctx, NULL, 0);

      return usbdev_disable_service(ctx);

    case DEV_USBDEV_ADDRESS:
      usbdev_disable_endpoint(ctx, NULL, 0);
      GCT_FOREACH(usbdev_service, &ctx->service, service, {
        /* Build table of interface */
        for (uint8_t j = 0; j< service->desc->intf_cnt; j++)
          {
            ctx->intf[i + j].i = &service->desc->intf[j]->intf;
            ctx->intf[i + j].epi = service->start.epi;
            ctx->intf[i + j].epo = service->start.epo;
          }
        i += service->desc->intf_cnt;
      });

      /* Termination */
      if (i < CONFIG_USBDEV_MAX_INTERFACE_COUNT)
        ctx->intf[i].i = NULL;

      /* Start configuration on controller */
      ctx->cfg.intf = ctx->intf;
      ctx->cfg.type = DEV_USBDEV_CONFIGURE;
      ctx->cfg.error = 0;

      usbdev_set_ep0_state(ctx, EP0_CTRL_CONFIGURATION_WAIT);

      LOCK_SPIN_IRQ(&ctx->dev->lock);

      err = ctx->ops->f_config(ctx, &ctx->cfg);

      LOCK_RELEASE_IRQ(&ctx->dev->lock);

      switch (err)
        {
        case 0:
          return usbdev_enable_service(ctx);
        case -ENOTSUP:
          /* Unsupported configuration */
          logk_error("ERROR: Configuration not supported by hardware");
          return usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_IN_STALL);
        case -EAGAIN:
          /* Configuration is on-going */
          return;
        default:
          abort();
        }
      break;

    default:
      abort();
    }
  return;
}

static bool_t usbdev_configuration_out_desc(struct dev_usbdev_context_s *ctx)
{
  size_t len = sizeof(struct usb_configuration_descriptor_s);
  size_t intf = 0;

  const struct usbdev_service_descriptor_s *s;
  const struct usbdev_device_info_s *d = ctx->devinfo;

  GCT_FOREACH(usbdev_service, &ctx->service, service, {
    s = service->desc;
    for (size_t j=0; j<s->desc_cnt; j++)
      len += s->desc[j]->bLength;
    intf += s->intf_cnt;
    });

  struct usb_configuration_descriptor_s desc = {
      .head.bLength = sizeof(struct usb_configuration_descriptor_s),
      .head.bDescriptorType = USB_DESC_CONFIGURATION,
      .wTotalLength = endian_le16(len),
      .bNumInterfaces = intf,
      .bConfigurationValue = 1,
      .iConfiguration = d->iconfig,
      .bmAttributes = 0x80 | d->configuration,
      .bMaxPower = d->power
  };

  usbdev_copy_buffer(ctx, &desc.head);

  /* Whole descriptor sent */
  return (ctx->it.desc_offset == 0);
}

static bool_t usbdev_service_out_desc(struct dev_usbdev_context_s *ctx)
{
  const struct usbdev_service_descriptor_s * s = ctx->it.service->desc;

  while (ctx->it.desc_index < s->desc_cnt)
    {
      size_t begin, end;
      const struct usb_descriptor_header_s *src = s->desc[ctx->it.desc_index];
      struct usb_descriptor_header_s *dst;

      begin = ctx->it.desc_offset;
      dst = (void *)((uint8_t*)ctx->data
                     + (ctx->it.data_pkt_size % CONFIG_USBDEV_EP0_BUFFER_SIZE)
                     - begin);
      end = begin + usbdev_copy_buffer(ctx, src);

      if (src->bDescriptorType == USB_DESC_INTERFACE)
        ctx->it.intf_index = usb_interface_alt_get((const struct usb_interface_descriptor_s *)src);

      if (s->replace)
        s->replace(&ctx->it, src, dst, begin, end);
      else
        usbdev_descriptor_replace(&ctx->it, src, dst, begin, end);

      if (ctx->it.desc_offset == 0)
        ctx->it.desc_index++;

      if (!(ctx->it.data_pkt_size % CONFIG_USBDEV_EP0_BUFFER_SIZE))
        break;
    }

  if ((ctx->it.desc_offset) || (ctx->it.desc_index != s->desc_cnt))
    return 0;

  /* Whole service sent */
  ctx->it.desc_index = 0;
  return 1;
}

static inline void usbdev_string_out_desc(struct dev_usbdev_context_s *ctx, size_t index)
{
  size_t len = 0;
  const char * str = NULL;
  size_t idx = 0;

  logk_trace("Get String Descriptor");

  /* Langid table */
  if (index == 0)
    {
      assert(CONFIG_USBDEV_USB_LANGID == 0x0409);
      /* Only one language supported */
      uint32_t d;
      struct usb_string_descriptor_s *desc = (struct usb_string_descriptor_s*)&d;

      desc->head.bLength = 2 + sizeof(struct usb_descriptor_header_s);
      desc->head.bDescriptorType = USB_DESC_STRING;
      desc->wData[0] = endian_le16(CONFIG_USBDEV_USB_LANGID);

      usbdev_copy_buffer(ctx, &desc->head);

      ctx->it.done = (ctx->it.desc_offset == 0);

      return;
    }

  if (index < ctx->devinfo->str_cnt + 1)
    {
      str = ctx->devinfo->string;
      idx = index - 1;
      usbdev_get_string(&str, idx, &len);
    }
  else
    {
      const struct usbdev_service_s *service = NULL;

      GCT_FOREACH(usbdev_service, &ctx->service, s, {
          if (usbdev_is_included(index, s->start.str + 1, s->desc->str_cnt))
            {
              service = s;
              idx = index - s->start.str - 1;
              GCT_FOREACH_BREAK;
            }
        });

      if (service)
        {
          if (service->desc->get_string) {
            str = service->desc->get_string(service, idx + 1);
            if (str)
              len = strlen(str);
          }

          if (!str) {
            str = service->desc->string;
            usbdev_get_string(&str, idx, &len);
          }
        }
    }

  uint8_t s = 2 * len + sizeof(struct usb_descriptor_header_s);
  uint8_t desc[s];

  desc[0] = s;
  desc[1] = USB_DESC_STRING;

  for (size_t i = 0; i < len; i++)
    {
      size_t j = 2 + (i << 1);
      desc[j] = str[i];
      desc[j + 1] = 0;
    }

  usbdev_copy_buffer(ctx, (struct usb_descriptor_header_s*)desc);

  ctx->it.done = (ctx->it.desc_offset == 0);
}

static inline void usbdev_out_desc(struct dev_usbdev_context_s *ctx, const struct usb_ctrl_setup_s *setup)
{

  if (usb_setup_recipient_get(setup) != USB_DEVICE ||
     (usb_setup_direction_get(setup) != USB_DEVICE_TO_HOST))
    return usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_IN_STALL);

  uint8_t dindex = usb_setup_value_get(setup) & 0xFF;

  switch (usb_setup_value_get(setup) >> 8)
    {
      case USB_DESC_DEVICE :
        {
          logk_trace("Get Device Descriptor ");
          if (dindex)
            return usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_IN_STALL);

          assert(ctx->devinfo);

          usbdev_copy_buffer(ctx, &ctx->devinfo->desc.head);
          ctx->it.done = (ctx->it.desc_offset == 0);
          break;
        }
      case USB_DESC_STRING :
        usbdev_string_out_desc(ctx, dindex);
        break;
      case USB_DESC_CONFIGURATION:
        /* Return configuration, interfaces and endpoints descriptors
           in a single response */
        logk_trace("Get Config Descriptor");
        while(!ctx->it.done)
          {
            if (ctx->it.service == NULL)
              {
                /* Configuration descriptor */
                if (usbdev_configuration_out_desc(ctx))
                  {
                    ctx->it.service = usbdev_service_head(&ctx->service);
                    if (ctx->it.service == NULL)
                      ctx->it.done = 1;
                  }
              }
            else
              {
                /* Service descriptor */
                if (usbdev_service_out_desc(ctx))
                  {
                    ctx->it.service = usbdev_service_next(&ctx->service,
                                                          ctx->it.service);
                    if (ctx->it.service == NULL)
                      ctx->it.done = 1;
                  }
              }

            if (!(ctx->it.data_pkt_size % CONFIG_USBDEV_EP0_BUFFER_SIZE))
              break;
          };
        break;
      default:
        return usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_IN_STALL);
    }
  return usbdev_ep0_data_in(ctx);
}

static inline void usbdev_set_address(struct dev_usbdev_context_s *ctx, const struct usb_ctrl_setup_s *setup)
{
  logk_trace("Set Address");

  if (usb_setup_value_get(setup) > 127 ||
      usb_setup_index_get(setup) ||
      usb_setup_length_get(setup))
    abort();

  ctx->cfg.type = DEV_USBDEV_SET_ADDRESS;
  ctx->cfg.addr = usb_setup_value_get(setup);
  ctx->cfg.error = 0;

  error_t err;

  LOCK_SPIN_IRQ(&ctx->dev->lock);

  err = ctx->ops->f_config(ctx, &ctx->cfg);

  LOCK_RELEASE_IRQ(&ctx->dev->lock);

  usbdev_set_ep0_state(ctx, EP0_CTRL_SET_ADDRESS_WAIT);

  switch (err)
    {
    case 0:
      return usbdev_ep0_status_in(ctx);
    case -EINVAL:
      return usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_IN_STALL);
    case -EAGAIN:
      return;
    default:
      abort();
    }
}

static inline
void usbdev_get_status(struct dev_usbdev_context_s *ctx,
                       const struct usb_ctrl_setup_s *setup)
{
  logk_trace("Get status");

  if (usb_setup_length_get(setup) != 2 ||
      usb_setup_value_get(setup))
    return usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_IN_STALL);

  ctx->it.data_pkt_size = usb_setup_length_get(setup);
  ctx->it.done = 1;
  ctx->data[0] = 0;
  ctx->data[1] = 0;
  return usbdev_ep0_data_in(ctx);
}

static inline void usbdev_get_interface(struct dev_usbdev_context_s *ctx, const struct usb_ctrl_setup_s *setup)
{
  logk_trace("Get Interface");

  if (usb_setup_length_get(setup) != 1 ||
      usb_setup_value_get(setup))
    abort();

  switch (ctx->state)
    {
      case DEV_USBDEV_CONFIGURED:{
        ctx->it.data_pkt_size = usb_setup_length_get(setup);
        ctx->it.done = 1;
        ctx->data[0] = 0;
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
        uint8_t intf = usb_setup_index_get(setup);
        const struct usbdev_service_descriptor_s * s;

        GCT_FOREACH(usbdev_service, &ctx->service, service, {
          s = service->desc;
          if (usbdev_is_included(intf, service->start.intf, s->intf_cnt))
            {
              ctx->data[0] = ctx->intf_cfg[intf];
              return usbdev_ep0_data_in(ctx);
            }
          });
        return usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_IN_STALL);
#endif
        break;}

      case DEV_USBDEV_ADDRESS:
        return usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_IN_STALL);

      default:
        abort();
    }
  return usbdev_ep0_data_in(ctx);
}

static inline void usbdev_get_configuration(struct dev_usbdev_context_s *ctx)
{
  uint8_t *data = ctx->data;

  switch (ctx->state)
    {
      case DEV_USBDEV_ADDRESS:
        data[0] = 0;
        break;
      case DEV_USBDEV_CONFIGURED:
        /* Only one configuration */
        data[0] = 1;
        break;
      default:
        abort();
    }

  ctx->it.done = 1;

  return usbdev_ep0_data_in(ctx);
}

static void usbdev_set_interface(struct dev_usbdev_context_s *ctx, const struct usb_ctrl_setup_s *setup)
{
  uint8_t alt = usb_setup_value_get(setup);
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  uint8_t intf = usb_setup_index_get(setup);
  const struct usbdev_service_descriptor_s * s;
  uint8_t idx, old;

  const struct usbdev_interface_s * i;

  logk_trace("Set Interface");

  GCT_FOREACH(usbdev_service, &ctx->service, service,
    {
      s = service->desc;
      if (usbdev_is_included(intf, service->start.intf, s->intf_cnt))
        {
          ctx->it.intf_index = intf;
          ctx->it.service = service;

          old = ctx->intf_cfg[intf];
          /* Local interface number */
          idx = intf - service->start.intf;

          if (old)
            i = s->intf[idx]->alt[old - 1];
          else
            i = &s->intf[idx]->intf;

          ctx->cfg.intf[DEV_USBDEV_INTF_DISABLE].i = i;
          ctx->cfg.intf[DEV_USBDEV_INTF_DISABLE].epi = service->start.epi;
          ctx->cfg.intf[DEV_USBDEV_INTF_DISABLE].epo = service->start.epo;

          if (alt)
            i = s->intf[idx]->alt[alt - 1];
          else
            i = &s->intf[idx]->intf;

          ctx->cfg.intf[DEV_USBDEV_INTF_ENABLE].i = i;
          ctx->cfg.intf[DEV_USBDEV_INTF_ENABLE].epi = service->start.epi;
          ctx->cfg.intf[DEV_USBDEV_INTF_ENABLE].epo = service->start.epo;

          /* Disable endpoints and change configuration number */
          usbdev_disable_endpoint(ctx, ctx->cfg.intf, alt);

          /* Notify service */
          ctx->it.intf_index = ctx->it.intf_index - ctx->it.service->start.intf;
          usbdev_service_ctrl(ctx, USBDEV_CHANGE_INTERFACE);

          ctx->cfg.type = DEV_USBDEV_CHANGE_INTERFACE;
          ctx->cfg.error = 0;

          usbdev_set_ep0_state(ctx, EP0_SET_INTERFACE_WAIT);

          error_t err;

          LOCK_SPIN_IRQ(&ctx->dev->lock);

          err = ctx->ops->f_config(ctx, &ctx->cfg);

          LOCK_RELEASE_IRQ(&ctx->dev->lock);

          switch (err)
            {
            case 0:
              usbdev_set_ep0_state(ctx, EP0_SRVC_SET_INTERFACE_WAIT);
              return;
            case -EAGAIN:
              /* Configuration is on-going */
              return;
            default:
              abort();
            }
      }
    });
  return usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_IN_STALL);
#endif
  if (alt)
    return usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_IN_STALL);
  return usbdev_ep0_status_in(ctx);
}


static void usbdev_ep0_standard_setup(struct dev_usbdev_context_s *ctx)
{
  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;

  //  usbdev_dump_setup(setup);

  switch (usb_setup_request_get(setup))
    {
      case USB_GET_CONFIGURATION:
        return usbdev_get_configuration(ctx);

      case USB_GET_DESCRIPTOR:
        return usbdev_out_desc(ctx, setup);

      case USB_GET_INTERFACE:
        return usbdev_get_interface(ctx, setup);

      case USB_GET_STATUS:
        return usbdev_get_status(ctx, setup);

      case USB_SET_ADDRESS:
        return usbdev_set_address(ctx, setup);

      case USB_SET_CONFIGURATION:
        return usbdev_set_configuration(ctx, setup);

      case USB_SET_INTERFACE:
        return usbdev_set_interface(ctx, setup);

      default:
        logk_error("Unsupported USB request:");
        logk_error("  bRequestType: %x", usb_setup_reqtype_get(setup));
        logk_error("  bRequest: %x", usb_setup_request_get(setup));
        logk_error("  wValue: %x", usb_setup_value_get(setup));
        logk_error("  wIndex: %x", usb_setup_index_get(setup));
        logk_error("  wLength: %x", usb_setup_length_get(setup));

        if (usb_setup_length_get(setup))
          {
            if (usb_setup_direction_get(setup) == USB_HOST_TO_DEVICE)
              return usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_OUT_STALL);
            else
              return usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_IN_STALL);
          }
        else
          return usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_IN_STALL);
    }
}

#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
static void usbdev_set_interface_end_transfer(struct dev_usbdev_context_s *ctx)
{
  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;
  /* Reenable old and new interface endpoints */
  usbdev_enable_endpoint(ctx, ctx->cfg.intf);
  /* Update interface number */
  ctx->intf_cfg[ctx->it.intf_index] = usb_setup_value_get(setup);

  return usbdev_ep0_status_in(ctx);
}
#endif

/* This is executed defered */

static KROUTINE_EXEC(usbdev_config_done)
{
  struct dev_usbdev_context_s * ctx = dev_usbdev_context_s_from_kr(kr);

  switch (ctx->cfg.error)
    {
    case 0:
      switch (ctx->state)
        {
        case DEV_USBDEV_ADDRESS_TO_DETACHED:
        case DEV_USBDEV_CONFIGURED_TO_DETACHED:
        case DEV_USBDEV_CONFIGURED_TO_DEFAULT:
        case DEV_USBDEV_CONFIGURED_TO_POWERED:
          switch (ctx->cfg.type)
            {
            case DEV_USBDEV_UNCONFIGURE:
              return usbdev_fsm_unconfiguration_done(ctx);
            default:
              abort();
            }
        case DEV_USBDEV_POWERED_TO_DEFAULT:
          switch (ctx->cfg.type)
            {
            case DEV_USBDEV_CONFIGURE:
            /* Ready to process setup packet */
              return usbdev_service_parse(ctx);
            case DEV_USBDEV_SET_ADDRESS:
              return usbdev_fsm_reset_ep0(ctx);
            default:
              abort();
            }
        case DEV_USBDEV_DEFAULT:
          switch (ctx->ep0_state)
            {
            case EP0_CTRL_SET_ADDRESS_WAIT:
              return usbdev_ep0_status_in(ctx);
            default:
              abort();
            }
        case DEV_USBDEV_ADDRESS:
          switch (ctx->ep0_state)
            {
            case EP0_CTRL_SET_ADDRESS_WAIT:
            case EP0_CTRL_CONFIGURATION_WAIT:
              return usbdev_enable_service(ctx);
            default:
              abort();
            }
        case DEV_USBDEV_CONFIGURED:
          switch (ctx->ep0_state)
            {
            case EP0_CTRL_UNCONFIGURATION_WAIT:
              return usbdev_ep0_status_in(ctx);
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
            case EP0_SET_INTERFACE_WAIT:
              /* Wait for service acknoledge */
              usbdev_set_ep0_state(ctx, EP0_SRVC_SET_INTERFACE_WAIT);
              return;
            case EP0_CTRL_SET_INTERFACE_WAIT:
              /* Process pending transfer */
              return usbdev_set_interface_end_transfer(ctx);
#endif
            default:
              abort();
            }
        default:
          abort();
        }
      break;
    default:
      logk_error("Error on configuration type %d", ctx->cfg.type);
      break;
    }
}

/* This is executed defered */

static KROUTINE_EXEC(usbdev_transfer_done)
{
  struct dev_usbdev_context_s * ctx = dev_usbdev_context_s_from_kr(kr);

  switch (ctx->tr.error)
    {
    case 0:
      switch (ctx->state)
        {
        case DEV_USBDEV_CONFIGURED:
        case DEV_USBDEV_ADDRESS:
        case DEV_USBDEV_DEFAULT:
          switch (ctx->ep0_state)
            {
            case EP0_SETUP_WAIT:
              return usbdev_ep0_standard_setup(ctx);

            case EP0_STALL_WAIT:
              return usbdev_ep0_idle(ctx);

            case EP0_STATUS_WAIT:
              return usbdev_ep0_status_done(ctx);

            case EP0_DATA_OUT_WAIT:
            case EP0_SRVC_DATA_OUT_WAIT:
              return usbdev_ep0_data_out_done(ctx);

            case EP0_DATA_IN_WAIT:
            case EP0_SRVC_DATA_IN_WAIT:
              return usbdev_ep0_data_in_done(ctx);

            case EP0_DATA_IN_ZERO_WAIT:
              return usbdev_ep0_status_out(ctx);

            default:
              abort();
            }
          break;
        default:
          abort();
        }
      break;
    case -EIO:
      /* An event was pending */
      return usbdev_fsm_handle_event(ctx, ctx->tr.event);
    case -EPIPE:
      switch (ctx->state)
        {
        case DEV_USBDEV_CONFIGURED:
        case DEV_USBDEV_ADDRESS:
        case DEV_USBDEV_DEFAULT:
          switch (ctx->ep0_state)
            {
            case EP0_DATA_IN_ZERO_WAIT:
            case EP0_SETUP_WAIT:
            case EP0_STALL_WAIT:
            case EP0_STATUS_WAIT:
            case EP0_DATA_OUT_WAIT:
            case EP0_DATA_IN_WAIT:
              return usbdev_ep0_idle(ctx);
            case EP0_SRVC_DATA_OUT_WAIT:
            case EP0_SRVC_DATA_IN_WAIT:
            default:
              abort();
            }
          break;
        default:
          abort();
        }
      logk("Control tranfer 0 aborted");
      /* Unexpected end of control transfer */
    default:
      logk_error("Error on transfer 0");
      abort();
    }
}

/* This is executed from an interrupt handler */

static void usbdev_stack_transfer_0_done(struct dev_usbdev_context_s *ctx,
                                         struct dev_usbdev_rq_s *tr)
{
  if (tr->error)
    goto kroutine;

  const struct usb_ctrl_setup_s *setup = (const void *)ctx->setup;

  switch (ctx->state)
    {
    case DEV_USBDEV_CONFIGURED:
    case DEV_USBDEV_ADDRESS:
    case DEV_USBDEV_DEFAULT:
      switch (ctx->ep0_state)
        {
        case EP0_SETUP_WAIT:
          ensure(tr->type == DEV_USBDEV_CTRL_SETUP);
          if (usbdev_is_service_setup(setup))
          /* Non standard setup packet */
            return usbdev_ep0_service_setup(ctx);
          break;

        case EP0_SRVC_DATA_OUT_WAIT:
          usbdev_set_ep0_state(ctx, EP0_SRVC_DATA_OUT);
          /* Check if data stage is valid */
          usbdev_ep0_data_out_stage_done(ctx);
          /* Call service kroutine */
          return usbdev_service_data(ctx, 0);

        case EP0_DATA_IN_WAIT:
        case EP0_DATA_OUT_WAIT:
          break;

        case EP0_SRVC_DATA_IN_WAIT:
          usbdev_set_ep0_state(ctx, EP0_SRVC_DATA_IN);
          /* Call service kroutine */
          return usbdev_service_data(ctx, 0);

        case EP0_DATA_IN_ZERO_WAIT:
        case EP0_STATUS_WAIT:
        case EP0_STALL_WAIT:
         break;

        default:
          abort();
        }
      break;
    default:
      abort();
    }

kroutine:
  kroutine_init_deferred(&ctx->kr, &usbdev_transfer_done);
  kroutine_exec(&ctx->kr);
}

static KROUTINE_EXEC(usbdev_init)
{
  struct dev_usbdev_context_s * ctx = dev_usbdev_context_s_from_kr(kr);
  /* Get any event on device */
  usbdev_get_event(ctx);
}

/* This is executed with lock */

void usbdev_stack_config_done(struct dev_usbdev_context_s *ctx)
{
  kroutine_init_deferred(&ctx->kr, &usbdev_config_done);
  kroutine_exec(&ctx->kr);
}

/* This is executed with lock */

void usbdev_stack_request_done(struct dev_usbdev_context_s *ctx,
                                struct dev_usbdev_rq_s *tr)
{
  assert(tr != NULL);

  if (tr->ep == 0)
    return usbdev_stack_transfer_0_done(ctx, tr);

  enum usb_endpoint_dir_e dir;

  if (dev_usbdev_get_transfer_dir(tr) == USB_HOST_TO_DEVICE)
    dir = USB_EP_OUT;
  else
    dir = USB_EP_IN;

  struct usbdev_endpoint_s *ep = ctx->ops->f_endpoint(ctx, dir, tr->ep);

  assert(ep);

  ep->busy = 0;
  /* Remove from queue */
  dev_usbdev_rq_pop(&ep->queue);

  /* Update Endpoint revision only when revision error */
  if (tr->error == -EAGAIN)
    tr->rev = ep->rev;

  dev_usbdev_rq_done(tr);

  /* Process next transfer */
  usbdev_process_queue(ctx, ep);
}

/* This function is called by services to retrieve a ctrl command or to
 * read/wite on control endpoint 0 */

error_t usbdev_stack_request(const struct device_usbdev_s *dev,
                             struct usbdev_service_s *service,
                             struct usbdev_service_rq_s *rq)
{
  ensure(service->rq == NULL);
  service->rq = rq;

  struct dev_usbdev_context_s *ctx = device_usbdev_context(dev);

  switch (ctx->state)
    {
    case DEV_USBDEV_WAIT_SERIVCE_READY:
      usbdev_wait_service_ready(ctx);
      break;
    case DEV_USBDEV_CONFIGURED_TO_POWERED:
    case DEV_USBDEV_CONFIGURED_TO_DETACHED:
    case DEV_USBDEV_CONFIGURED_TO_DEFAULT:
    case DEV_USBDEV_ADDRESS_TO_DEFAULT:
    case DEV_USBDEV_ADDRESS_TO_DETACHED:
    case DEV_USBDEV_ADDRESS_TO_POWERED:
      ctx->service_state &= ~(1 << service->start.id);
      /* All service are disabled */
      if (!ctx->service_state)
        usbdev_fsm_unconfiguration(ctx);
      break;
    case DEV_USBDEV_CONFIGURED:
      switch (ctx->ep0_state)
        {
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
          case EP0_SET_INTERFACE_WAIT:
            /* Wait for controller acknoledge */
            usbdev_set_ep0_state(ctx, EP0_CTRL_SET_INTERFACE_WAIT);
            break;
          case EP0_SRVC_SET_INTERFACE_WAIT:
            /* Reconfiguration is done */
            usbdev_set_interface_end_transfer(ctx);
            break;
#endif
          case EP0_SRVC_DISABLE_WAIT:
            ctx->service_state &= ~(1 << service->start.id);
            /* All service are disabled */
            if (ctx->service_state == 0)
              usbdev_ep0_unconfigure(ctx);
            break;
          case EP0_SRVC_DATA_OUT:
            switch (rq->type)
              {
                case USBDEV_GET_COMMAND:
                  if (rq->error)
                    {
                      if (usbdev_data_in_stage_done(ctx))
                      /* Host is waiting status */
                        usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_IN_STALL);
                      else
                      /* Host is sending data */
                        usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_OUT_STALL);
                    }
                  else
                    /* This terminates the data stage of service */
                    usbdev_ep0_status_in(ctx);
                  break;
                case USBDEV_TRANSFER_DATA:
                  usbdev_ep0_data_out(ctx);
                  break;
                default:
                  abort();
              }
            break;
          case EP0_SRVC_DATA_IN:
            switch (rq->type)
              {
                case USBDEV_GET_COMMAND:
                  if (rq->error)
                    {
                      if (usbdev_data_in_stage_done(ctx))
                      /* Host is sending status */
                        usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_OUT_STALL);
                      else
                      /* Host is waiting data */
                        usbdev_ep0_stall(ctx, DEV_USBDEV_DATA_IN_STALL);
                    }
                  else
                    /* This terminates the data stage of service */
                    usbdev_ep0_zero_len_packet(ctx);
                  break;
                case USBDEV_TRANSFER_DATA:
                  ctx->it.data_pkt_size += rq->ctrl.size;
                  usbdev_ep0_data_in(ctx);
                  break;
                default:
                  abort();
              }
            break;
          case EP0_SRVC_STATUS_IN:
            /* No data stage */
            if (rq->error)
              usbdev_ep0_stall(ctx, DEV_USBDEV_CTRL_STATUS_IN_STALL);
            else
              usbdev_ep0_status_in(ctx);
            break;
          default:
            logk_error("Bad State %d %d", ctx->state, ctx->ep0_state);
            abort();
        }
      break;
    case DEV_USBDEV_ADDRESS:
      switch (ctx->ep0_state)
        {
          case EP0_SRVC_ENABLE_WAIT:
            ctx->service_state |= 1 << service->start.id;
            /* All service are enabled */
            if (ctx->service_state == ((1 << ctx->service_cnt) - 1))
              usbdev_ep0_status_in(ctx);
            break;
          default:
            abort();
        }
      break;
    default:
      break;
    }
  return 0;
}

/* Service entry point. Executed with lock */

error_t usbdev_stack_transfer(const struct device_usbdev_s *dev,
                              struct usbdev_service_s *service,
                              struct dev_usbdev_rq_s *tr,
                              const struct usb_endpoint_descriptor_s *desc)
{
  struct dev_usbdev_context_s *ctx = device_usbdev_context(dev);
  error_t err = 0;

  LOCK_SPIN_IRQ(&ctx->dev->lock);

  tr->error = 0;

  dev_usbdev_ep_map_t mapi = service->start.epi[tr->rev];
  dev_usbdev_ep_map_t mapo = service->start.epo[tr->rev];

  /* Retrieve global address */
  tr->ep = usbdev_stack_get_ep_addr(desc, mapi, mapo);

  struct usbdev_endpoint_s *ep;
  enum usb_endpoint_dir_e dir = USB_EP_IN;

  if (dev_usbdev_get_transfer_dir(tr) == USB_HOST_TO_DEVICE)
    dir = USB_EP_OUT;

  ep = ctx->ops->f_endpoint(ctx, dir, tr->ep);

  if (!ep->disabled)
    {
      err = usbdev_is_valid_transfer(ctx, ep, tr);
      if (err)
        goto done;
    }

  bool_t empty = dev_rq_queue_isempty(&ep->queue);
  dev_usbdev_rq_pushback(&ep->queue, tr);

  /* Queue is empty and endpoint is neither disabled or busy */
  if (empty && !ep->disabled && !ep->busy)
    usbdev_start_transfer(ctx, ep, tr);

done:
  LOCK_RELEASE_IRQ(&ctx->dev->lock);
  return err;
}

/* This is executed with lock */

static void usbdev_ep_init_cleanup(struct dev_usbdev_context_s *ctx,
                                   enum usb_endpoint_dir_e dir,
                                   bool_t init)
{
  struct usbdev_endpoint_s *ep;
  uint16_t msk = dir == USB_EP_IN ? ctx->epi_msk : ctx->epo_msk;
  uint8_t idx;

  while(msk)
    {
      idx = bit_ctz(msk);

      ep = ctx->ops->f_endpoint(ctx, dir, idx);
      assert(ep != NULL);

      if (init)
        {
          ep->disabled = 0;
          ep->busy = 0;
          dev_rq_queue_init(&ep->queue);
        }
      else
        dev_rq_queue_destroy(&ep->queue);

      msk ^= 1 << idx;
    }
}

#ifdef CONFIG_USBDEV_DEFAULT_DEVICE_INFO

/* This is the default device information structure. */

static const struct usbdev_device_info_s usb_default_devinfo =
{
  .desc =
    {
      .head.bLength = sizeof(struct usb_device_descriptor_s),
      .head.bDescriptorType = USB_DESC_DEVICE,
      .bcdUSB = endian_le16(CONFIG_USBDEV_USB_REVISION),
      .bDeviceClass = 0,
      .bDeviceSubClass = 0,
      .bDeviceProtocol = 0,
      .bMaxPacketSize0 = 64,
      .idVendor = endian_le16(0x5a5a),
      .idProduct = endian_le16(0),
      .bcdDevice = endian_le16(0),
      .iManufacturer = 1,
      .iProduct = 2,
      .iSerialNumber = 0,
      .bNumConfigurations = 1
    },

  .configuration = 0,
  .iconfig = 0,
  .power = 50,
  .str_cnt = 2,
  .string = "MutekH\0Test\0"
};

#endif

/* This is executed with lock */

error_t usbdev_stack_init(struct device_s *dev,
                          struct dev_usbdev_context_s *ctx,
                          uint16_t epi_msk, uint16_t epo_msk,
                          const struct dev_usbdev_driver_ops_s *ops)
{
  usbdev_service_init(&ctx->service);

#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  ensure(CONFIG_USBDEV_MAX_INTERFACE_COUNT > 1);
#endif

  ctx->dev = dev;
  ctx->ops = ops;
#ifdef CONFIG_USBDEV_DEFAULT_DEVICE_INFO
  ctx->devinfo = &usb_default_devinfo;
#else
  ctx->devinfo = NULL;
#endif

  ctx->epi_msk = epi_msk & 0xFE;
  ctx->epo_msk = epo_msk & 0xFE;

  /* Initialise endpoints */
  usbdev_ep_init_cleanup(ctx, USB_EP_IN, 1);
  usbdev_ep_init_cleanup(ctx, USB_EP_OUT, 1);

  /* Intialise device state */
  ctx->state = DEV_USBDEV_DETACHED;
  ctx->service_cnt = 0;

  /* Endpoint 0 */
  ctx->data = ctx->ops->f_alloc(ctx, CONFIG_USBDEV_EP0_BUFFER_SIZE);
  usbdev_set_ep0_state(ctx, EP0_IDLE);
  ctx->tr.ep = 0;

  kroutine_init_deferred(&ctx->kr, &usbdev_init);

  /* Defer initialisation */
  kroutine_exec(&ctx->kr);

  return 0;
}

/* This is executed with lock */

error_t usbdev_stack_cleanup(struct dev_usbdev_context_s *ctx)
{
  if (ctx->state > DEV_USBDEV_POWERED)
    return -EBUSY;

  usbdev_service_destroy(&ctx->service);

  /* Free endpoint 0 buffer */
  ctx->ops->f_free(ctx, ctx->data);

  /* Destroy endpoint queue */
  usbdev_ep_init_cleanup(ctx, USB_EP_IN, 0);
  usbdev_ep_init_cleanup(ctx, USB_EP_OUT, 0);

  return 0;
}

enum usb_transfert_direction_e dev_usbdev_get_transfer_dir(struct dev_usbdev_rq_s *tr)
{
  switch (tr->type)
    {
    case DEV_USBDEV_CTRL_STATUS_OUT:
    case DEV_USBDEV_CTRL_STATUS_OUT_STALL:
    case DEV_USBDEV_DATA_OUT:
    case DEV_USBDEV_PARTIAL_DATA_OUT:
    case DEV_USBDEV_DATA_OUT_STALL:
    case DEV_USBDEV_CTRL_SETUP:
    case DEV_USBDEV_EVENT:
      return USB_HOST_TO_DEVICE;
    default:
      return USB_DEVICE_TO_HOST;
    }
}

error_t dev_res_get_usbdev_epmap(struct device_s *dev, struct usbdev_service_s * service)
{
  uint8_t config;
  uint8_t found = 0;

  for (uint8_t i = 0; i < CONFIG_USBDEV_MAX_ALTERNATE_COUNT + 1; i++)
    {
      service->start.epi[i] = 0;
      service->start.epo[i] = 0;
    }

  DEVICE_RES_FOREACH(dev, r, {
      if (r->type == DEV_RES_USBDEV)
        {
          config = r->u.usbdev.config;
          assert(config <= CONFIG_USBDEV_MAX_ALTERNATE_COUNT);
          service->start.epi[config] = r->u.usbdev.mapin;
          service->start.epo[config] = r->u.usbdev.mapout;
          found++;
        }
    });

  if (!found)
    return -ENOENT;

  return 0;
}

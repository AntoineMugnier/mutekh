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

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2018
*/

#define LOGK_MODULE_ID "ufwd"

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/class/char.h>
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
#include <device/class/valio.h>
#include <device/valio/uart_config.h>
#endif

#include <gct_platform.h>
#include <gct/container_dring.h>

#if defined(DRIVER_USAGE_TEMPLATE) && 0

#define LOGK_MODULE_ID "main"

#include <device/resources.h>
#include <mutek/printk.h>
#include <mutek/startup.h>

DEV_DECLARE_STATIC(
  uart_forwarder_dev, "ufwd", 0, uart_forwarder_drv,
  DEV_STATIC_RES_DEV_PARAM("master", "/uart0"),
  DEV_STATIC_RES_UINT_PARAM("m_read_size", 16),
  DEV_STATIC_RES_UINT_PARAM("m2s_size", 64),
  DEV_STATIC_RES_UINT_PARAM("s_write_size", 16),

  DEV_STATIC_RES_DEV_PARAM("slave", "/uart1"),
  DEV_STATIC_RES_UINT_PARAM("s_read_size", 64),
  DEV_STATIC_RES_UINT_PARAM("s2m_size", 512),
  DEV_STATIC_RES_UINT_PARAM("m_write_size", 64),
  );

void app_start(void)
{
  error_t err;
  struct device_accessor_s forwarder;

  err = device_get_accessor_by_path(&forwarder, NULL,
                                    "/ufwd", DRIVER_CLASS_NONE);
  if (err) {
    logk_error("/ufwd error: %d", err);
  } else {
    device_start(&forwarder);
    device_put_accessor(&forwarder);
  }
}

#endif

#define GCT_CONTAINER_ALGO_uart_forwarder_buffer  DRING

GCT_CONTAINER_TYPES(uart_forwarder_buffer, uint8_t, 0);
GCT_CONTAINER_FCNS(uart_forwarder_buffer, static inline, uart_forwarder_buffer,
		   init, destroy, storage, count,
                   pushback_array, pop_array);

enum ep_id_e
{
  EP_MASTER,
  EP_SLAVE,
  EP_COUNT,
};

__unused__
static const char *ep_flow[EP_COUNT] = {"M->S", "S->M"};

__unused__
static const char *ep_name[EP_COUNT] = {"master", "slave"};

struct pipe_s;

struct endpoint_s
{
  struct dev_char_rq_s rq;
  uint8_t *data;
  size_t size;
  struct pipe_s *pipe;
};

struct pipe_s
{
  struct device_char_s cdev;
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
  struct device_valio_s vdev;
#endif
  struct endpoint_s in[2];
  struct endpoint_s out;
  uart_forwarder_buffer_root_t buffer;
  enum ep_id_e src;
};

struct uart_forwarder_pv_s
{
  struct pipe_s pipe[EP_COUNT];
  struct kroutine_sequence_s seq;
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
  struct dev_uart_config_s uart_config;
  struct dev_valio_rq_s uart_rq;
#endif
};

DRIVER_PV(struct uart_forwarder_pv_s);

static
struct pipe_s *pipe_peer(struct pipe_s *pipe)
{
  if (pipe->src == EP_MASTER)
    return pipe + 1;
  else
    return pipe - 1;
}

static
void endpoint_read_schedule(struct endpoint_s *ep)
{
  struct pipe_s *pipe = ep->pipe;

  ep->rq.data = ep->data;
  ep->rq.size = ep->size;

  DEVICE_OP(&pipe->cdev, request, &ep->rq);
}

static
void endpoint_read_cancel(struct endpoint_s *ep)
{
  DEVICE_OP(&ep->pipe->cdev, cancel, &ep->rq);
}

static
void endpoint_write_schedule(struct endpoint_s *ep)
{
  struct pipe_s *pipe = ep->pipe;
  struct pipe_s *peer = pipe_peer(pipe);

  if (ep->rq.base.pvuint)
    return;

  logk_debug("%s %d bytes left",
             ep_flow[ep->pipe->src],
             uart_forwarder_buffer_count(&ep->pipe->buffer));

  ep->rq.data = ep->data;
  ep->rq.size = uart_forwarder_buffer_pop_array(&ep->pipe->buffer, ep->data, ep->size);

  if (ep->rq.size) {
    ep->rq.base.pvuint = 1;
    DEVICE_OP(&peer->cdev, request, &ep->rq);
  }
}

static
void endpoint_write_cancel(struct endpoint_s *ep)
{
  struct pipe_s *pipe = ep->pipe;
  struct pipe_s *peer = pipe_peer(pipe);

  if (!ep->rq.base.pvuint)
    return;

  if (DEVICE_OP(&peer->cdev, cancel, &ep->rq) == 0)
    ep->rq.base.pvuint = 0;
}

static KROUTINE_EXEC(endpoint_write_done)
{
  struct endpoint_s *ep = KROUTINE_CONTAINER(kr, *ep, rq.base.kr);
  struct pipe_s *pipe = ep->pipe;

  (void)pipe;
  logk_debug("%s %d bytes out", ep_flow[pipe->src], ep->rq.data - ep->data);

  ep->rq.base.pvuint = 0;

  endpoint_write_schedule(ep);
}

static KROUTINE_EXEC(endpoint_read_done)
{
  struct endpoint_s *ep = KROUTINE_CONTAINER(kr, *ep, rq.base.kr);
  struct pipe_s *pipe = ep->pipe;
  size_t size = ep->rq.data - ep->data;

  if (ep->rq.error) {
    logk_error("read done err %d", ep->rq.error);
    return endpoint_read_schedule(ep);
  }

  logk_debug("%s %d bytes in", ep_flow[pipe->src], size);

  size_t written = uart_forwarder_buffer_pushback_array(
      &pipe->buffer, ep->data, size);

  if (written != size)
    logk_warning("%s buffer overflowed by %d bytes",
               ep_flow[pipe->src], size - written);

  endpoint_read_schedule(ep);

  endpoint_write_schedule(&pipe->out);
}

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
static KROUTINE_EXEC(forwarder_cfg_changed);

static KROUTINE_EXEC(forwarder_cfg_applied)
{
  struct uart_forwarder_pv_s *pv = KROUTINE_CONTAINER(kr, *pv, uart_rq.base.kr);

  dev_valio_rq_init(&pv->uart_rq, forwarder_cfg_changed);
  pv->uart_rq.type = DEVICE_VALIO_WAIT_EVENT;
  DEVICE_OP(&pv->pipe[EP_MASTER].vdev, request, &pv->uart_rq);
}

static KROUTINE_EXEC(forwarder_cfg_changed)
{
  struct uart_forwarder_pv_s *pv = KROUTINE_CONTAINER(kr, *pv, uart_rq.base.kr);

  if (pv->uart_rq.error) {
    logk_error("UART Config get error: %d", pv->uart_rq.error);
    logk_error("%s", pv->pipe[EP_MASTER].vdev.dev->node.name);
    return;
  }
  
  logk("UART Config now %d %d-%c-%d",
       pv->uart_config.baudrate, pv->uart_config.data_bits,
       "NOE"[pv->uart_config.parity], pv->uart_config.stop_bits);
  
  dev_valio_rq_init(&pv->uart_rq, forwarder_cfg_applied);
  pv->uart_rq.type = DEVICE_VALIO_WRITE;
  DEVICE_OP(&pv->pipe[EP_SLAVE].vdev, request, &pv->uart_rq);
}
#endif

static DEV_USE(uart_forwarder_use)
{
  switch (op) {
  case DEV_USE_START: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;
    struct uart_forwarder_pv_s *pv = dev->drv_pv;

    for (enum ep_id_e src = 0; src < EP_COUNT; ++src)
      for (size_t i = 0; i < ARRAY_SIZE(pv->pipe[src].in); ++i)
        endpoint_read_schedule(&pv->pipe[src].in[i]);

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
    if (device_check_accessor(&pv->pipe[EP_MASTER].vdev.base)) {
      dev_valio_rq_init(&pv->uart_rq, forwarder_cfg_changed);

      pv->uart_rq.type = DEVICE_VALIO_READ;
      pv->uart_rq.attribute = VALIO_UART_CONFIG;
      pv->uart_rq.data = &pv->uart_config;
      
      DEVICE_OP(&pv->pipe[EP_MASTER].vdev, request, &pv->uart_rq);
    }
#endif
    return 0;
  }

  case DEV_USE_STOP: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;
    struct uart_forwarder_pv_s *pv = dev->drv_pv;

    for (enum ep_id_e src = 0; src < EP_COUNT; ++src) {
      for (size_t i = 0; i < ARRAY_SIZE(pv->pipe[src].in); ++i)
        endpoint_read_cancel(&pv->pipe[src].in[i]);
      endpoint_write_cancel(&pv->pipe[src].out);
    }

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
    if (device_check_accessor(&pv->pipe[EP_MASTER].vdev.base))
      DEVICE_OP(&pv->pipe[EP_MASTER].vdev, cancel, &pv->uart_rq);
#endif
    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_INIT(uart_forwarder_init)
{
  error_t ret;
  struct uart_forwarder_pv_s *pv;
  uintptr_t rsize[EP_COUNT];
  uintptr_t wsize[EP_COUNT];
  uintptr_t bsize[EP_COUNT];
  struct device_char_s cdev[EP_COUNT];
  size_t alloc_size = sizeof(*pv);

  device_get_param_uint_default(dev, "m_read_size", &rsize[EP_MASTER], 16);
  device_get_param_uint_default(dev, "m2s_size", &bsize[EP_MASTER], 64);
  device_get_param_uint_default(dev, "s_write_size", &wsize[EP_SLAVE], 16);

  device_get_param_uint_default(dev, "s_read_size", &rsize[EP_SLAVE], 16);
  device_get_param_uint_default(dev, "s2m_size", &bsize[EP_SLAVE], 64);
  device_get_param_uint_default(dev, "m_write_size", &wsize[EP_MASTER], 16);

  for (enum ep_id_e src = 0; src < EP_COUNT; ++src) {
    ret = device_get_param_dev_accessor(dev, ep_name[src],
                                        &cdev[src].base,
                                        DRIVER_CLASS_CHAR);

    if (ret) {
      logk_fatal("Cannot get %s device", ep_name[src]);

      if (src == EP_SLAVE)
        device_put_accessor(&cdev[EP_MASTER].base);

      return ret;
    }

    alloc_size += 0
      + rsize[src] * ARRAY_SIZE(pv->pipe[EP_MASTER].in)
      + wsize[src]
      + bsize[src]
      ;
  }

  logk_debug("PV data size (inc. buffers): %d bytes", alloc_size);

  pv = mem_alloc(alloc_size, mem_scope_sys);

  if (!pv) {
    logk_fatal("Cannot allocate memory");
    device_put_accessor(&cdev[EP_MASTER].base);
    device_put_accessor(&cdev[EP_SLAVE].base);
    return -ENOMEM;
  }

  memset(pv, 0, sizeof(*pv));

  kroutine_seq_init(&pv->seq);

  uint8_t *data = (uint8_t*)(pv + 1);

  for (enum ep_id_e src = 0; src < EP_COUNT; ++src) {
    struct pipe_s *p = &pv->pipe[src];

    device_copy_accessor(&p->cdev.base, &cdev[src].base);
    p->src = src;

    uart_forwarder_buffer_init(&p->buffer);
    uart_forwarder_buffer_storage(&p->buffer, bsize[src], data);
    data += bsize[src];

    logk_debug("%s in rq: %dx%d, buffer: %d, out rq: %d bytes",
               ep_flow[src], ARRAY_SIZE(p->in), rsize[src],
               bsize[src], wsize[!src]);

    for (size_t i = 0; i < ARRAY_SIZE(p->in); ++i) {
      struct endpoint_s *ep = &p->in[i];
      dev_char_rq_init_seq(&ep->rq, endpoint_read_done, &pv->seq);
      ep->rq.type = DEV_CHAR_READ_PARTIAL;
      
      ep->data = data;
      ep->size = rsize[src];
      data += rsize[src];

      ep->pipe = p;
    }

    {
      struct endpoint_s *ep = &p->out;

      dev_char_rq_init_seq(&ep->rq, endpoint_write_done, &pv->seq);
      ep->rq.type = DEV_CHAR_WRITE;

      ep->data = data;
      ep->size = wsize[src];
      data += wsize[src];

      ep->pipe = p;
    }
    
    assert(data <= (uint8_t*)pv + alloc_size);

    device_put_accessor(&cdev[src].base);
  }

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
  for (enum ep_id_e src = 0; src < EP_COUNT; ++src) {
    ret = device_get_param_dev_accessor(
        dev, ep_name[src],
        &pv->pipe[src].vdev.base,
        DRIVER_CLASS_VALIO);

    if (ret) {
      logk_warning("Valio API not supported by %s, no config will be forwarded",
                   ep_name[src]);

      if (src == EP_SLAVE)
        device_put_accessor(&pv->pipe[EP_MASTER].vdev.base);
      break;
    }
  }
#endif
  
  dev->drv_pv = pv;
  
  return 0;
}

static DEV_CLEANUP(uart_forwarder_cleanup)
{
  struct uart_forwarder_pv_s *pv = dev->drv_pv;
  
  for (enum ep_id_e src = 0; src < EP_COUNT; ++src) {
    device_put_accessor(&pv->pipe[src].cdev.base);
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
    if (device_check_accessor(&pv->pipe[src].vdev.base))
      device_put_accessor(&pv->pipe[src].vdev.base);
#endif
  }

  kroutine_seq_cleanup(&pv->seq);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(uart_forwarder_drv, 0, "UART Forwarder", uart_forwarder,
               0);

DRIVER_REGISTER(uart_forwarder_drv,
                DEV_ENUM_FDTNAME_ENTRY("uart_forwarder"));


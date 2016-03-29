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

    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/class/spi.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>

#include "mcp23s17.h"

/*
  This implements a driver for the Microchip MCP23S17 device,
  a 16 bit IO expander with SPI interface.

  It implements the asynchronous API of the GPIO class and the API of the ICU
  class (when #CONFIG_DRIVER_MCP23S17_ICU is defined).

  When used as an ICU, #CONFIG_DRIVER_MCP23S17_IRQ_COUNT defines the
  the number of available IRQs on the device (between 1 and 16).

  For proper use, the sink end-point linked with the source end-point of the
  device has to be configured with a RISING_EDGE sensibility.

  The device's sink endpoints sensibilities are HIGH_LEVEL, LOW_LEVEL and
  ANY_EDGE. Due to some race conditions inherent to the device hardware, the
  driver uses a sofware implementation for the ANY_EDGE trigger. It uses
  HIGH_LEVEL and LOW_LEVEL alternatively by reconfiguring the sink end-point
  sensitivity on the fly, during the interruption. Altough it works well with
  long pulses (like push button pressed and released), a too short pulse could
  be masked by another one when they appear simultaneously !

  The address of the device could be specified with an integer resource named
  'addr'. If not, the default address is 0.

  Here is a sample static mcp23s17 device declaration:

  DEV_DECLARE_STATIC(mcp23s17_dev, "gpio_ext", 0, mcp23s17_drv,
                      DEV_STATIC_RES_UINT_PARAM("addr", 0),
                      DEV_STATIC_RES_DEV_SPI("/spi"),
                      DEV_STATIC_RES_DEV_ICU("/gpio"),
                      DEV_STATIC_RES_IRQ(0, 1, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                    );

*/

static void mcp23s17_rq_serve(struct device_s *dev, struct dev_gpio_rq_s *req);

#ifdef CONFIG_DRIVER_MCP23S17_ICU
static void mcp23s17_spi_irq_process(struct mcp23s17_private_s *pv);
static void mcp23s17_spi_irq_setup(struct mcp23s17_private_s *pv);
#endif

static void mcp23s17_next(struct device_s *dev)
{
  struct mcp23s17_private_s *pv = dev->drv_pv;
  struct dev_gpio_rq_s      *req;

#ifdef CONFIG_DRIVER_MCP23S17_ICU
  if (pv->pending_op & MCP23S17_IRQ_PROCESS_OP)
    {
      pv->current_op = MCP23S17_IRQ_PROCESS_OP;
      pv->pending_op &= ~MCP23S17_IRQ_PROCESS_OP;
      mcp23s17_spi_irq_process(pv);
      return;
    }
  if (pv->pending_op & MCP23S17_IRQ_SETUP_OP)
    {
      pv->current_op = MCP23S17_IRQ_SETUP_OP;
      pv->pending_op &= ~MCP23S17_IRQ_SETUP_OP;
      mcp23s17_spi_irq_setup(pv);
      return;
    }
#endif
  if ((req = dev_gpio_rq_s_cast(dev_request_queue_head(&pv->rq_pending))))
    {
      pv->current_op = MCP23S17_REQUEST_OP;
      pv->pending_op &= ~MCP23S17_REQUEST_OP;
      mcp23s17_rq_serve(dev, req);
      return;
    }
  pv->current_op = 0;
  return;
}

static KROUTINE_EXEC(mcp23s17_spi_done)
{
  struct mcp23s17_private_s *pv = KROUTINE_CONTAINER(kr, *pv, spi_req.base.base.kr);
  struct device_s           *dev = pv->spi_req.base.base.pvdata;
  struct dev_gpio_rq_s      *req;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (pv->current_op)
    {
      case MCP23S17_REQUEST_OP:
        req = dev_gpio_rq_s_cast(dev_request_queue_head(&pv->rq_pending));
        req->error = pv->spi_req.base.err;
        switch (req->type)
          {
            case DEV_GPIO_GET_INPUT:
              endian_le16_na_store(req->input.data, pv->gpio_cache >> req->io_first);
            case DEV_GPIO_MODE:
            case DEV_GPIO_SET_OUTPUT:
            case DEV_GPIO_INPUT_IRQ_RANGE:
              break;
          }
        dev_request_queue_pop(&pv->rq_pending);
        req->base.drvdata = NULL;
        kroutine_exec(&req->base.kr);
        break;
#ifdef CONFIG_DRIVER_MCP23S17_ICU
      case MCP23S17_IRQ_PROCESS_OP:
        if (pv->intf_cache)
          {
            while (pv->intf_cache)
              {
                uint_fast8_t pin_id = __builtin_ctz(pv->intf_cache);
                struct dev_irq_sink_s *sink = pv->sinks_ep + pv->sinks_map[pin_id];
                device_irq_sink_process(sink, 0);
                pv->intf_cache ^= 1 << pin_id;
              }
            mcp23s17_spi_irq_process(pv);
            return;
          }
        break;
      case MCP23S17_IRQ_SETUP_OP:
#endif
      case MCP23S17_INIT_OP:
      default:
        break;
    }
    mcp23s17_next(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

#ifdef CONFIG_DRIVER_MCP23S17_ICU
static void mcp23s17_spi_irq_setup(
  struct mcp23s17_private_s *pv)
{
  bc_set_reg(&pv->spi_req.vm, BC_REG_0, pv->int_update_mask);
  bc_set_reg(&pv->spi_req.vm, BC_REG_1, pv->int_update_en);
  pv->int_update_mask = 0;
  pv->int_update_en = 0;
  dev_spi_bytecode_start(&pv->spi, &pv->spi_req, &mcp23s17_bc_setup_irq);
}

static inline void mcp23s17_spi_irq_process(
  struct mcp23s17_private_s *pv)
{
  dev_spi_bytecode_start(&pv->spi, &pv->spi_req, &mcp23s17_bc_read_irq);
}
#endif

static void mcp23s17_rq_serve(
  struct device_s *dev,
  struct dev_gpio_rq_s *req)
{
  struct mcp23s17_private_s *pv = dev->drv_pv;

  if (req->io_last >= MCP23S17_PIN_NB || req->io_last < req->io_first)
    {
      req->error = -ERANGE;
      kroutine_exec(&pv->spi_req.base.base.kr);
      return;
    }

  uint16_t range_mask
    = ((1 << (req->io_last - req->io_first + 1)) - 1) << req->io_first;

  __unused__ uint16_t mask;
  __unused__ uint16_t set_mask;
  __unused__ uint16_t clr_mask;

  switch (req->type)
    {
      case DEV_GPIO_MODE:
        mask = (endian_le16_na_load(req->mode.mask) << req->io_first) & range_mask;
        switch (req->mode.mode)
          {
            case DEV_PIN_PUSHPULL:
              if (pv->iodir_cache & mask)
                {
                  pv->iodir_cache &= ~mask;
                  dev_spi_bytecode_start(&pv->spi, &pv->spi_req, &mcp23s17_bc_write_output_mode);
                  return;
                }
              break;
            case DEV_PIN_INPUT_PULLUP:
              if (~pv->gppu_cache & mask || ~pv->iodir_cache & mask)
                {
                  pv->gppu_cache |= mask;
                  pv->iodir_cache |= mask;
                  dev_spi_bytecode_start(&pv->spi, &pv->spi_req,
                                         &mcp23s17_bc_write_input_mode);
                  return;
                }
              break;
            case DEV_PIN_INPUT:
              if (pv->gppu_cache & mask || ~pv->iodir_cache & mask)
                {
                  pv->gppu_cache &= ~mask;
                  pv->iodir_cache |= mask;
                  dev_spi_bytecode_start(&pv->spi, &pv->spi_req,
                                         &mcp23s17_bc_write_input_mode);
                  return;
                }
              break;
            default:
              req->error = -ENOTSUP;
              kroutine_exec(&pv->spi_req.base.base.kr);
              return;
          }
        break;

      case DEV_GPIO_SET_OUTPUT:
        set_mask = endian_le16_na_load(req->output.set_mask) << req->io_first;
        clr_mask = endian_le16_na_load(req->output.clear_mask) << req->io_first;
        mask = set_mask ^ (pv->olat_cache & (set_mask ^ clr_mask));
        if ((pv->olat_cache ^ mask) & range_mask)
          {
            pv->olat_cache |= mask & range_mask;
            pv->olat_cache &= mask | ~range_mask;
            dev_spi_bytecode_start(&pv->spi, &pv->spi_req,
                                   &mcp23s17_bc_write_output);
            return;
          }
        break;

      case DEV_GPIO_GET_INPUT:
        dev_spi_bytecode_start(&pv->spi, &pv->spi_req,
                               &mcp23s17_bc_read_input);
        return;

      case DEV_GPIO_INPUT_IRQ_RANGE:
        req->error = -ENOTSUP;
        kroutine_exec(&pv->spi_req.base.base.kr);
        return;
    }
    req->error = 0;
    kroutine_exec(&pv->spi_req.base.base.kr);
    return;
}

static DEV_GPIO_REQUEST(mcp23s17_request)
{
  struct device_s *dev = gpio->dev;
  struct mcp23s17_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  dev_request_queue_pushback(&pv->rq_pending, &req->base);

  pv->pending_op |= MCP23S17_REQUEST_OP;
  if (!pv->current_op)
    {
      pv->current_op = MCP23S17_REQUEST_OP;
      pv->pending_op &= ~MCP23S17_REQUEST_OP;
      mcp23s17_rq_serve(dev, req);
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

#ifdef CONFIG_DRIVER_MCP23S17_ICU

static DEV_IRQ_SRC_PROCESS(mcp23s17_source_process)
{
  struct mcp23s17_private_s *pv = ep->base.dev->drv_pv;

  pv->pending_op |= MCP23S17_IRQ_PROCESS_OP;
  if (!pv->current_op && pv->pending_op == MCP23S17_IRQ_PROCESS_OP)
    {
      pv->current_op = MCP23S17_IRQ_PROCESS_OP;
      pv->pending_op &= ~MCP23S17_IRQ_PROCESS_OP;
      mcp23s17_spi_irq_process(pv);
    }
}

static DEV_IRQ_SINK_UPDATE(mcp23s17_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct mcp23s17_private_s *pv = dev->drv_pv;

  uint_fast8_t pin_id = sink->icu_pv;
  uint16_t     pin_mask = 1 << pin_id;

  pv->int_update_mask |= pin_mask;
  switch (sense)
    {
      case DEV_IRQ_SENSE_ANY_EDGE:
        pv->intcon_cache |= pin_mask;
        pv->int_update_en |= pin_mask;
        pv->edge_sensitive |= pin_mask;
        break;
      case DEV_IRQ_SENSE_HIGH_LEVEL:
        pv->intcon_cache |= pin_mask;
        pv->defval_cache &= ~pin_mask;
        pv->int_update_en |= pin_mask;
        pv->edge_sensitive &= ~pin_mask;
        break;
      case DEV_IRQ_SENSE_LOW_LEVEL:
        pv->intcon_cache |= pin_mask;
        pv->defval_cache |= pin_mask;
        pv->int_update_en |= pin_mask;
        pv->edge_sensitive &= ~pin_mask;
        break;
      case DEV_IRQ_SENSE_NONE:
      default:
        pv->edge_sensitive &= ~pin_mask;
        pv->int_update_en &= ~pin_mask;
        break;
    }

  pv->pending_op |= MCP23S17_IRQ_SETUP_OP;
  if (!pv->current_op)
    {
      pv->current_op = MCP23S17_IRQ_SETUP_OP;
      pv->pending_op &= ~MCP23S17_IRQ_SETUP_OP;
      mcp23s17_spi_irq_setup(pv);
    }
}

static DEV_ICU_LINK(mcp23s17_icu_link)
{
  struct device_s *dev = sink->base.dev;
  struct mcp23s17_private_s *pv = dev->drv_pv;

  uint_fast8_t pin_id = sink->icu_pv;
  uint16_t     pin_mask = 1 << pin_id;

#ifdef CONFIG_DEVICE_IRQ_SHARING
  if (sink->base.link_count > 1)
    return 0;
#endif

  if (*bypass)
    return 0;

  if (route_mask)
    {
      pv->gppu_cache &= ~pin_mask;
      pv->iodir_cache |= pin_mask;
    }

  pv->int_update_mask |= pin_mask;
  pv->int_update_en &= ~pin_mask;

  pv->pending_op |= MCP23S17_IRQ_SETUP_OP;
  if (!pv->current_op)
    {
      pv->current_op = MCP23S17_IRQ_SETUP_OP;
      pv->pending_op &= ~MCP23S17_IRQ_SETUP_OP;
      mcp23s17_spi_irq_setup(pv);
    }

  return 0;
}

static DEV_ICU_GET_SINK(mcp23s17_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct mcp23s17_private_s *pv = dev->drv_pv;

  if (id >= MCP23S17_PIN_NB)
    return NULL;

  for (uint_fast8_t i = 0; i < CONFIG_DRIVER_MCP23S17_IRQ_COUNT; i++)
    {
      if (!pv->sinks_ep[i].base.link_count)
        {
          pv->sinks_map[id] = i;
          pv->sinks_ep[i].icu_pv = id;
          return pv->sinks_ep + i;
        }
    }
  return NULL;
}

#endif

static DEV_INIT(mcp23s17_init);
static DEV_CLEANUP(mcp23s17_cleanup);

#define mcp23s17_set_mode (dev_gpio_set_mode_t *)dev_driver_notsup_fcn
#define mcp23s17_set_output (dev_gpio_set_output_t *)dev_driver_notsup_fcn
#define mcp23s17_get_input (dev_gpio_get_input_t *)dev_driver_notsup_fcn
#define mcp23s17_input_irq_range (dev_gpio_input_irq_range_t *)dev_driver_notsup_fcn
#define mcp23s17_use  dev_use_generic

DRIVER_DECLARE(mcp23s17_drv, 0, "mcp23s17", mcp23s17,
  DRIVER_GPIO_METHODS(mcp23s17)
#ifdef CONFIG_DRIVER_MCP23S17_ICU
  ,DRIVER_ICU_METHODS(mcp23s17_icu)
#endif
);

static error_t spi_config(
  struct device_s *dev,
  struct mcp23s17_private_s *pv)
{
  if (dev_drv_spi_bytecode_init(dev, &pv->spi_req, &pv->spi, NULL, NULL))
    return -ENOTSUP;

  pv->spi_req.base.config.bit_rate = 1000000;
  pv->spi_req.base.config.word_width = 8;
  pv->spi_req.base.config.bit_order = DEV_SPI_MSB_FIRST;
  pv->spi_req.base.config.ck_mode = DEV_SPI_CK_MODE_0;
  pv->spi_req.base.base.pvdata = dev;

  bc_init(&pv->spi_req.vm, &mcp23s17_bytecode, 1, pv);

  kroutine_init_immediate(&pv->spi_req.base.base.kr, &mcp23s17_spi_done);

  return 0;
}

static DEV_INIT(mcp23s17_init)
{
  struct mcp23s17_private_s *pv;


  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

 if (spi_config(dev, pv))
    goto err_mem;

  uintptr_t addr;
  if (device_get_param_uint(dev, "addr", &addr))
    addr = 0;
  if (addr > 7)
    goto err_mem;
  pv->device_opcode = 0x40 + (addr << 1);

  pv->icon_cache = MCP23S17_REG_IOCON_MIRROR
                    | MCP23S17_REG_IOCON_HAEN
                    | MCP23S17_REG_IOCON_INTPOL;

  pv->iodir_cache = -1;

  dev_request_queue_init(&pv->rq_pending);

#ifdef CONFIG_DRIVER_MCP23S17_ICU
  device_irq_source_init(dev, &pv->src_ep, 1, &mcp23s17_source_process);

  if (device_irq_source_link(dev, &pv->src_ep, 1, 1))
    goto err_unlink;

  enum dev_irq_sense_modes_e sink_mode;
  device_irq_modes(&pv->src_ep, &sink_mode, NULL);
  if (sink_mode != DEV_IRQ_SENSE_RISING_EDGE)
    goto err_unlink;

  device_irq_sink_init(dev, pv->sinks_ep, CONFIG_DRIVER_MCP23S17_IRQ_COUNT,
    &mcp23s17_icu_sink_update, DEV_IRQ_SENSE_HIGH_LEVEL
                                | DEV_IRQ_SENSE_LOW_LEVEL
                                | DEV_IRQ_SENSE_ANY_EDGE);

  memset(pv->sinks_map, 0, sizeof(*(pv->sinks_map)));
#endif

  dev->drv_pv = pv;

  pv->current_op = MCP23S17_INIT_OP;
  dev_spi_bytecode_start(&pv->spi, &pv->spi_req, &mcp23s17_bc_sync_cache);

  return 0;

#ifdef CONFIG_DRIVER_MCP23S17_ICU
 err_unlink:
  device_irq_source_unlink(dev, &pv->src_ep, 1);
#endif

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(mcp23s17_cleanup)
{
  struct mcp23s17_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_MCP23S17_ICU
  device_irq_source_unlink(dev, &pv->src_ep, 1);
#endif

  dev_request_queue_destroy(&pv->rq_pending);

  mem_free(pv);
  return 0;
}


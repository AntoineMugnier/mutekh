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

    Copyright (c) 2014 Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include <device/class/gpio.h>
#include <device/device.h>
#include <device/driver.h>

#include <stdarg.h>

__attribute__((aligned(8)))
const uint8_t dev_gpio_mask1[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
__attribute__((aligned(8)))
const uint8_t dev_gpio_mask0[8] = { };

extern inline error_t dev_gpio_spin_rq(struct device_gpio_s *accessor,
                                       struct dev_gpio_rq_s *rq);

#ifdef CONFIG_MUTEK_SCHEDULER

extern inline error_t dev_gpio_wait_rq(struct device_gpio_s *accessor,
                                       struct dev_gpio_rq_s *rq);

#endif

error_t device_gpio_map_set_mode(struct device_gpio_s *accessor,
                                 const gpio_id_t *map, const gpio_width_t *wmap,
                                 uint_fast8_t count, /* enum dev_pin_driving_e */ ...)
{
  va_list ap;
  uint_fast8_t i;
  error_t err = 0;

  va_start(ap, count);

  for (i = 0; i < count; i++)
    {
      gpio_id_t id = map[i];
      enum dev_pin_driving_e mode = va_arg(ap, __compiler_sint_t);

      if (id != GPIO_INVALID_ID)
        if (DEVICE_OP(accessor, set_mode, id, id + wmap[i] - 1, dev_gpio_mask1, mode))
          {
            err = -EINVAL;
            break;
          }
    }

  va_end(ap);
  return err;
}

error_t device_res_gpio_map(struct device_s *dev, const char *pin_list,
                            gpio_id_t *map, gpio_width_t *wmap)
{
  while (*pin_list)
    {
      struct dev_resource_s *r = device_res_get_from_name(dev, DEV_RES_GPIO, 0, pin_list);

      while (*pin_list && *pin_list != ':' && *pin_list != '?')
        pin_list++;

      if (*pin_list == '?')
        pin_list++;
      else if (!r)
        return -ENOENT;

      *map++ = r ? r->u.gpio.id : GPIO_INVALID_ID;
      if (wmap)
        *wmap++ = r ? r->u.gpio.width : 0;

      if (*pin_list == ':')
        {
          uint_fast8_t w = strtoul(pin_list + 1, (char**)&pin_list, 0);
          if (r && r->u.gpio.width != w)
            return -ERANGE;
        }

      while (*pin_list == ' ')
        pin_list++;
    }

  return 0;
}

DEV_GPIO_REQUEST(dev_gpio_request_async_to_sync)
{
  switch (req->type) {
  case DEV_GPIO_MODE:
    req->error = DEVICE_OP(gpio, set_mode,
                          req->io_first, req->io_last,
                          req->mode.mask, req->mode.mode);
    break;

  case DEV_GPIO_SET_OUTPUT:
    req->error = DEVICE_OP(gpio, set_output,
                          req->io_first, req->io_last,
                          req->output.set_mask, req->output.clear_mask);
    break;

  case DEV_GPIO_GET_INPUT:
    req->error = DEVICE_OP(gpio, get_input,
                          req->io_first, req->io_last,
                          req->input.data);
    break;

  case DEV_GPIO_INPUT_IRQ_RANGE:
    req->error = DEVICE_OP(gpio, input_irq_range,
                           req->io_first, req->io_last,
                           req->input_irq_range.mask,
                           req->input_irq_range.mode,
                           req->input_irq_range.ep_id);
    break;
  }

  kroutine_exec(&req->base.kr);
}

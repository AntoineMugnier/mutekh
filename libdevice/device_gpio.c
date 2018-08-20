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

#ifdef CONFIG_MUTEK_CONTEXT_SCHED

extern inline error_t dev_gpio_wait_rq(const struct device_gpio_s *accessor,
                                       struct dev_gpio_rq_s *rq);

extern inline error_t dev_gpio_wait_mode(const struct device_gpio_s *accessor, gpio_id_t id,
                                         enum dev_pin_driving_e mode);

extern inline error_t dev_gpio_wait_out(const struct device_gpio_s *accessor, gpio_id_t id,
                                        bool_t x);

extern inline bool_t dev_gpio_wait_input(const struct device_gpio_s *accessor, gpio_id_t id,
                                         error_t *err);

#endif

error_t device_gpio_map_set_mode(const struct device_gpio_s *accessor,
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
  return device_gpio_setup(NULL, dev, pin_list, map, wmap);
}

error_t device_gpio_setup(struct device_gpio_s *gpio,
                          struct device_s *dev, const char *pin_list,
                          gpio_id_t *map, gpio_width_t *wmap)
{
  while (*pin_list)
    {
      enum dev_pin_driving_e dir = device_io_mode_symbol(*pin_list);
      if (dir)
        pin_list++;

      struct dev_resource_s *r = device_res_get_from_name(dev, DEV_RES_GPIO, 0, pin_list);

      while (*pin_list && *pin_list != ':' && *pin_list != '?' && *pin_list != ' ')
        pin_list++;

      if (*pin_list == '?')
        pin_list++;
      else if (!r)
        return -ENOENT;

      gpio_id_t id = GPIO_INVALID_ID;
      gpio_width_t w = 0;

      if (r)
        {
          id = r->u.gpio.id;
          w = r->u.gpio.width;

          if (dir != DEV_PIN_DISABLED &&
              DEVICE_OP(gpio, set_mode, id, id + w - 1, dev_gpio_mask1, dir))
            return -EIO;
        }

      *map++ = id;
      if (wmap)
        *wmap++ = w;

      if (*pin_list == ':')
        {
          if (r && r->u.gpio.width !=
              strtoul(pin_list + 1, (char**)&pin_list, 0))
            return -ERANGE;
        }

      while (*pin_list == ' ')
        pin_list++;
    }

  return 0;
}

error_t device_gpio_get_setup(struct device_gpio_s *gpio,
                              struct device_s *dev, const char *pin_list,
                              gpio_id_t *map, gpio_width_t *wmap)
{
  if (device_get_param_dev_accessor(dev, "gpio", &gpio->base, DRIVER_CLASS_GPIO))
    return -ENOENT;

  error_t err = device_gpio_setup(gpio, dev, pin_list, map, wmap);
  if (err)
    device_put_accessor(&gpio->base);

  return err;
}

DEV_GPIO_REQUEST(dev_gpio_request_async_to_sync)
{
  switch (rq->type) {
  case DEV_GPIO_MODE:
    rq->error = DEVICE_OP(gpio, set_mode,
                          rq->io_first, rq->io_last,
                          rq->mode.mask, rq->mode.mode);
    break;

  case DEV_GPIO_SET_OUTPUT:
    rq->error = DEVICE_OP(gpio, set_output,
                          rq->io_first, rq->io_last,
                          rq->output.set_mask, rq->output.clear_mask);
    break;

  case DEV_GPIO_GET_INPUT:
    rq->error = DEVICE_OP(gpio, get_input,
                          rq->io_first, rq->io_last,
                          rq->input.data);
    break;

  case DEV_GPIO_UNTIL:
    rq->error = -ENOTSUP;
    break;
  }

  dev_gpio_rq_done(rq);
}

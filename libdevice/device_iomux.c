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

#include <device/class/iomux.h>
#include <device/device.h>
#include <device/driver.h>

#include <stdarg.h>

static enum dev_pin_driving_e device_iomux_mode(char l)
{
  struct switch_s { char c; char n; };
  static const struct switch_s sw[10] = {
    { '^', DEV_PIN_OPENSOURCE },
    { '_', DEV_PIN_OPENDRAIN },
    { '`', DEV_PIN_OPENSOURCE_PULLDOWN },
    { '+', DEV_PIN_INPUT_PULLUP },
    { ',', DEV_PIN_OPENDRAIN_PULLUP },
    { '-', DEV_PIN_INPUT_PULLDOWN },
    { 0, 0 },
    { '<', DEV_PIN_INPUT },
    { '=', DEV_PIN_INPUT_PULL },
    { '>', DEV_PIN_PUSHPULL },
  };

  /* decode direction symbol using a perfect hash */
  uint32_t x = ((319838000U * (uint32_t)l) >> 28);
  if (x < 10 && sw[x].c == l)
    return sw[x].n;

  return DEV_PIN_DISABLED;
}

error_t device_iomux_fetch(struct device_s *dev,
                           struct device_iomux_s *iomux, const char *io_list,
                           iomux_demux_t *demux, iomux_io_id_t *io_id,
                           iomux_config_t *config)
{
  while (*io_list)
    {
      enum dev_pin_driving_e dir = device_iomux_mode(*io_list);
      if (dir)
        io_list++;

      /* lookup io name in resources */
      struct dev_resource_s *r = device_res_get_from_name(dev, DEV_RES_IOMUX, 0, io_list);
      iomux_demux_t d = IOMUX_INVALID_DEMUX;
      iomux_io_id_t i = IOMUX_INVALID_ID;
      iomux_config_t c = 0;

      /* skip io name */
      while (*io_list > ' ')
        {
          io_list++;
          if (io_list[-1] == '?' && !r)
            goto done;
        }

      if (!r)
        return -ENOENT;

      /* direction in resource label overrides the one in driver string */
      enum dev_pin_driving_e dir2 = device_iomux_mode(*r->u.iomux.label);
      if (dir2)
        dir = dir2;

      if (iomux)
        {
          /* configure io */
          error_t err = DEVICE_OP(iomux, setup, r->u.iomux.io_id, dir,
                                      r->u.iomux.mux, r->u.iomux.config);
          if (err)
            return err;
        }

      d = r->u.iomux.demux;
      i = r->u.iomux.io_id;
      c = r->u.iomux.config;

    done:
      /* fill arrays */
      if (demux)
        *demux++ = d;
      if (io_id)
        *io_id++ = i;
      if (config)
        *config++ = c;

      while (*io_list == ' ')
        io_list++;
    }

  return 0;
}

error_t device_iomux_setup(struct device_s *dev, const char *io_list,
                           iomux_demux_t *demux, iomux_io_id_t *io_id,
                           iomux_config_t *config)
{
  struct device_iomux_s iomux;

  error_t err = device_get_param_dev_accessor(dev, "iomux", &iomux.base, DRIVER_CLASS_IOMUX);
  if (err)
    return err;

  err = device_iomux_fetch(dev, &iomux, io_list, demux, io_id, config);

  device_put_accessor(&iomux.base);

  return err;
}

void device_iomux_cleanup2(struct device_s *dev, struct device_iomux_s *iomux)
{
  DEVICE_RES_FOREACH(dev, r, {
      if (r->type == DEV_RES_IOMUX)
        DEVICE_OP(iomux, setup, r->u.iomux.io_id, DEV_PIN_DISABLED, IOMUX_INVALID_MUX, 0);
  });
}

void device_iomux_cleanup(struct device_s *dev)
{
  struct device_iomux_s iomux;

  if (!device_get_param_dev_accessor(dev, "iomux", &iomux.base, DRIVER_CLASS_IOMUX))
    {
      device_iomux_cleanup2(dev, &iomux);
      device_put_accessor(&iomux.base);
    }
}

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

    Copyright (c) 2016-2019, Nicolas Pouillon, <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "smib"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/smi.h>
#include <device/class/gpio.h>

enum smi_gpio_id_e
{
  GPIO_ID_MDC,
  GPIO_ID_MDIO,
  GPIO_ID_COUNT,
};
  
struct smi_bb_ctx_s
{
  struct device_gpio_s gpio;
  gpio_id_t gpio_map[GPIO_ID_COUNT];

  struct dev_smi_drv_ctx_s smi_context;
};

STRUCT_COMPOSE(smi_bb_ctx_s, smi_context);

DRIVER_PV(struct smi_bb_ctx_s);

static
void smi_bb_mdio_oe(struct smi_bb_ctx_s *pv, bool_t en)
{
  dev_gpio_mode(&pv->gpio, pv->gpio_map[GPIO_ID_MDIO],
                en ? DEV_PIN_PUSHPULL : DEV_PIN_INPUT);
}

static
bool_t smi_bb_bit(struct smi_bb_ctx_s *pv, bool_t out)
{
  bool_t ret;

  dev_gpio_out(&pv->gpio, pv->gpio_map[GPIO_ID_MDIO], out);
  dev_gpio_out(&pv->gpio, pv->gpio_map[GPIO_ID_MDC], 0);
  ret = dev_gpio_input(&pv->gpio, pv->gpio_map[GPIO_ID_MDIO], NULL);
  dev_gpio_out(&pv->gpio, pv->gpio_map[GPIO_ID_MDC], 1);

  return ret;
}

static
uint32_t smi_bb_shift(struct smi_bb_ctx_s *pv, uint32_t value, uint_fast8_t bit_count)
{
  uint32_t mask = bit(bit_count - 1);
  uint32_t ret = 0;

  for (uint_fast8_t i = 0; i < bit_count; ++i) {
    bool_t in;

    in = smi_bb_bit(pv, !!(value & mask));
    if (in)
      ret |= mask;

    mask >>= 1;
  }

  return ret;
}

static
error_t smi_bb_c22_read(struct smi_bb_ctx_s *pv,
                         uint_fast8_t phyad,
                         uint_fast8_t regad,
                         uint16_t *value)
{
  uint16_t turn;

  smi_bb_mdio_oe(pv, 1);
  smi_bb_shift(pv, -1, 32);
  smi_bb_shift(pv, 0b0110, 4);
  smi_bb_shift(pv, phyad, 5);
  smi_bb_shift(pv, regad, 5);
  smi_bb_mdio_oe(pv, 0);
  turn = smi_bb_shift(pv, 0, 2);
  *value = smi_bb_shift(pv, 0, 16);
  smi_bb_shift(pv, 0, 1);

  return turn == 0x2 ? 0 : -EIO;
}

static
void smi_bb_c22_write(struct smi_bb_ctx_s *pv,
                      uint_fast8_t phyad,
                      uint_fast8_t regad,
                      uint16_t value)
{
  smi_bb_mdio_oe(pv, 1);
  smi_bb_shift(pv, -1, 32);
  smi_bb_shift(pv, 0b0101, 4);
  smi_bb_shift(pv, phyad, 5);
  smi_bb_shift(pv, regad, 5);
  smi_bb_shift(pv, 0b10, 2);
  smi_bb_shift(pv, value, 16);
  smi_bb_mdio_oe(pv, 0);
  smi_bb_shift(pv, 0, 1);
}

static
void smi_bb_c45_address(struct smi_bb_ctx_s *pv,
                        uint_fast8_t prtad,
                        uint_fast8_t devad,
                        uint16_t address)
{
  smi_bb_mdio_oe(pv, 1);
  smi_bb_shift(pv, -1, 32);
  smi_bb_shift(pv, 0b0000, 4);
  smi_bb_shift(pv, prtad, 5);
  smi_bb_shift(pv, devad, 5);
  smi_bb_shift(pv, 0b10, 2);
  smi_bb_shift(pv, address, 16);
  smi_bb_mdio_oe(pv, 0);
  smi_bb_shift(pv, 0, 1);
}

static
error_t smi_bb_c45_read(struct smi_bb_ctx_s *pv,
                        uint_fast8_t prtad,
                        uint_fast8_t devad,
                        uint16_t *value)
{
  uint16_t turn;

  smi_bb_mdio_oe(pv, 1);
  smi_bb_shift(pv, -1, 32);
  smi_bb_shift(pv, 0b0011, 4);
  smi_bb_shift(pv, prtad, 5);
  smi_bb_shift(pv, devad, 5);
  smi_bb_mdio_oe(pv, 0);
  turn = smi_bb_shift(pv, 0, 2);
  *value = smi_bb_shift(pv, 0, 16);
  smi_bb_shift(pv, 0, 1);

  return turn == 0x2 ? 0 : -EIO;
}

static
void smi_bb_c45_write(struct smi_bb_ctx_s *pv,
                      uint_fast8_t prtad,
                      uint_fast8_t devad,
                      uint16_t value)
{
  smi_bb_mdio_oe(pv, 1);
  smi_bb_shift(pv, -1, 32);
  smi_bb_shift(pv, 0b0001, 4);
  smi_bb_shift(pv, prtad, 5);
  smi_bb_shift(pv, devad, 5);
  smi_bb_shift(pv, 0b10, 2);
  smi_bb_shift(pv, value, 16);
  smi_bb_mdio_oe(pv, 0);
  smi_bb_shift(pv, 0, 1);
}

static
DEV_SMI_REQUEST(smi_bb_request)
{
  struct device_s *dev = accessor->dev;
  struct smi_bb_ctx_s *pv = dev->drv_pv;

  dev_smi_drv_request_push(&pv->smi_context, rq);
}

static
DEV_SMI_TRANSFER(smi_bb_transfer)
{
  struct smi_bb_ctx_s *pv = smi_bb_ctx_s_from_smi_context(ctx);

  switch (data->clause) {
  case DEV_SMI_C22:
    if (data->op == DEV_SMI_READ)
      return smi_bb_c22_read(pv, data->prtad, data->address & 0x1f, &data->value);

    smi_bb_c22_write(pv, data->prtad, data->address & 0x1f, data->value);
    return 0;

  case DEV_SMI_C22X:
    smi_bb_c22_write(pv, data->prtad, 13, data->devad);
    smi_bb_c22_write(pv, data->prtad, 14, data->address);
    smi_bb_c22_write(pv, data->prtad, 13, 0x4000 | data->devad);
    if (data->op == DEV_SMI_READ)
      return smi_bb_c22_read(pv, data->prtad, 14, &data->value);

    smi_bb_c22_write(pv, data->prtad, 14, data->value);
    return 0;

  case DEV_SMI_C45:
    smi_bb_c45_address(pv, data->prtad, data->devad, data->address);
    if (data->op == DEV_SMI_READ)
      return smi_bb_c45_read(pv, data->prtad, data->devad, &data->value);

    smi_bb_c45_write(pv, data->prtad, data->devad, data->value);
    return 0;
  }

  return -EINVAL;
}

#define smi_bb_use dev_use_generic

DRIVER_SMI_OPS_DECLARE(smi_bb);

static DEV_INIT(smi_bb_init)
{
  struct smi_bb_ctx_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = device_gpio_get_setup(&pv->gpio, dev,
                              ">mdc:1 >mdio:1", pv->gpio_map, NULL);
  if (err)
    goto err_gpio;

  dev_smi_drv_init(dev, &pv->smi_context, &smi_bb_smi_ops);
  
  dev->drv_pv = pv;

  return 0;

 err_gpio:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(smi_bb_cleanup)
{
  struct smi_bb_ctx_s *pv = dev->drv_pv;

  error_t err;

  err = dev_smi_drv_cleanup(&pv->smi_context);
  if (err)
    return err;

  device_put_accessor(&pv->gpio.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(smi_bitbang_drv, 0, "SMI Bitbang", smi_bb,
               DRIVER_SMI_METHODS(smi_bb));

DRIVER_REGISTER(smi_bitbang_drv);


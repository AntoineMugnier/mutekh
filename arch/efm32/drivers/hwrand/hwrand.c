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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2018
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <arch/efm32/cmu.h>
#include <arch/efm32/rtc.h>
#include <arch/efm32/devaddr.h>
#include <arch/efm32/hwrand.h>

#include <mutek/mem_alloc.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>

DRIVER_PV(struct efm32_hwrand_private_s
{
  uint16_t ptr;
  uint16_t size;
  uint8_t pool[0];
});

static DEV_CRYPTO_INFO(efm32_hwrand_info)
{
  memset(info, 0, sizeof(*info));

  if (accessor->number)
    return -ENOENT;

  info->name = "hwrand";
  info->modes_mask = 1 << DEV_CRYPTO_MODE_RANDOM;

  return 0;
}

static DEV_CRYPTO_REQUEST(efm32_hwrand_request)
{
  struct device_s *dev = accessor->dev;

  LOCK_SPIN_IRQ(&dev->lock);

  struct efm32_hwrand_private_s * __restrict__ pv = dev->drv_pv;

  rq->error = -ENOENT;
  if (pv)
    {
      struct dev_crypto_context_s * __restrict__ ctx = rq->ctx;

      rq->error = -ENOTSUP;
      if (ctx->mode == DEV_CRYPTO_MODE_RANDOM &&
          (rq->op & DEV_CRYPTO_FINALIZE))
        {
          size_t l = pv->size - pv->ptr;
          size_t rl = rq->len;

          rq->error = -ENOENT;
          if (rl <= l)
            {
              uint_fast8_t ptr = pv->ptr;
              uint8_t *r = pv->pool + ptr;
              memcpy(rq->out, r, rl);
              memset(r, 0, rl);
              pv->ptr = ptr + rl;
              rq->error = 0;

              if (rl == l)
                {
                  mem_free(pv);
                  dev->drv_pv = NULL;
                }
            }
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  dev_crypto_rq_done(rq);
}

static DEV_INIT(efm32_hwrand_init)
{
  error_t err;

  uintptr_t size;
  device_get_param_uint_default(dev, "size", &size, 32);
  size = align_pow2_up(size, 4);
  if (!size || size > 256)
    return -EINVAL;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  /* Check that HFRC is still selected */
  if (!(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_HFRCOSEL))
    {
      err = -EBUSY;
      goto done;
    }

  /* Enable LE clock */
  uint32_t coreclken = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKEN0_ADDR);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKEN0_ADDR, coreclken | EFM32_CMU_HFCORECLKEN0_LE);

  /* Enable LFRCO */
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_LFRCOEN);
  while (!(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_LFRCORDY))
    ;

  uint32_t lfsel = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFCLKSEL_ADDR);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFCLKSEL_ADDR, EFM32_CMU_LFCLKSEL_LFA(LFRCO));

  /* Enable RTC clock */
  uint32_t lfen = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFACLKEN0_ADDR);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFACLKEN0_ADDR, lfen | EFM32_CMU_LFACLKEN0_RTC);

  /* Start RTC */
  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_FREEZE_ADDR, 0);
  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_CTRL_ADDR, EFM32_RTC_CTRL_EN_COUNT);

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  /* Check that HFRC is still selected */
  if (cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFCLKSTATUS_ADDR)
      != EFM32_CMU_HFCLKSTATUS_SELECTED_HFRCO)
    {
      err = -EBUSY;
      goto done;
    }

  /* Enable LFRCO */
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_LFRCOEN);
  while (!(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_LFRCORDY))
    ;

  /* Select as source for RTCC */
  uint32_t lfeclksel = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFECLKSEL_ADDR);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFECLKSEL_ADDR, EFM32_CMU_LFECLKSEL_LFE(LFRCO));

  /* Enable RTCC clock */
  uint32_t lfclken = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFECLKEN0_ADDR);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFECLKEN0_ADDR, EFM32_CMU_LFECLKEN0_RTCC);

  /* Start RTCC */
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CTRL_ADDR, EFM32_RTCC_CTRL_ENABLE);
#else
# error
#endif

  /* Start collecting random data */
  err = -EIO;
  uint32_t x = efm32_hw_rand32();
  if (x != HWRAND_CRC32_ALL1 &&
      x != HWRAND_CRC32_ALL0)
    {
      err = -ENOMEM;
      struct efm32_hwrand_private_s *pv = mem_alloc(sizeof (*pv) + size, (mem_scope_sys));

      if (pv)
        {
          dev->drv_pv = pv;
          pv->ptr = 0;
          pv->size = size;

          uint8_t *p = pv->pool;
          endian_le32_na_store(p, x);
          for (p += 4; size > 4; (p += 4), (size -= 4))
            endian_le32_na_store(p, efm32_hw_rand32());

          err = 0;
        }
    }

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  /* Stop RTC */
  cpu_mem_write_32(EFM32_RTC_ADDR + EFM32_RTC_CTRL_ADDR, 0);

  /* Restore clocks state */
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFACLKEN0_ADDR, lfen);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKEN0_ADDR, coreclken);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFCLKSEL_ADDR, lfsel);

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  /* Stop RTCC */
  cpu_mem_write_32(EFM32_RTCC_ADDR + EFM32_RTCC_CTRL_ADDR, 0);

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFECLKSEL_ADDR, lfeclksel);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFECLKEN0_ADDR, lfclken);
#else
# error
#endif

 done:;
  CPU_INTERRUPT_RESTORESTATE;
  return err;
}

static DEV_CLEANUP(efm32_hwrand_cleanup)
{
  struct efm32_hwrand_private_s  *pv = dev->drv_pv;
  if (pv)
    mem_free(pv);
  return 0;
}

#define efm32_hwrand_use dev_use_generic

DRIVER_DECLARE(efm32_hwrand_drv, DRIVER_FLAGS_EARLY_INIT,
               "Hardware random seed", efm32_hwrand,
               DRIVER_CRYPTO_METHODS(efm32_hwrand));

DRIVER_REGISTER(efm32_hwrand_drv);

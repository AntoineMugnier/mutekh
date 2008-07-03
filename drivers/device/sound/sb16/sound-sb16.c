/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#include <hexo/types.h>

#include <device/sound.h>
#include <hexo/device.h>
#include <device/driver.h>
#include <hexo/alloc.h>

#include <arch/dma-8237.h>

#include "sound-sb16.h"

#include "sound-sb16-private.h"

DEVSOUND_READ(sound_sb16_read)
{
  return 0;
}

DEVSOUND_WRITE(sound_sb16_write)
{
  return 0;
}

DEVSOUND_MODE(sound_sb16_mode)
{
  return 0;
}

DEV_IRQ(sound_sb16_irq)
{
  return 0;
}

DEV_CLEANUP(sound_sb16_cleanup)
{
  struct sound_sb16_context_s	*pv = dev->drv_pv;

  mem_free(pv);
}

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	sound_sb16_drv =
{
  .class		= device_class_sound,
  .f_init		= sound_sb16_init,
  .f_cleanup		= sound_sb16_cleanup,
  .f_irq		= sound_sb16_irq,
  .f.sound = {
    .f_read		= sound_sb16_read,
    .f_write		= sound_sb16_write,
    .f_mode		= sound_sb16_mode,
  }
};
#endif

DEV_INIT(sound_sb16_init)
{
  struct sound_sb16_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &sound_sb16_drv;
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  return 0;
}


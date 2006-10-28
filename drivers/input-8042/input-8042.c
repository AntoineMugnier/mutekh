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

#include <hexo/device/input.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include <assert.h>

#include "input-8042.h"

#include "input-8042-private.h"

#include "8042.h"

/**************************************************************/

/* 
 * device info query
 */

DEVINPUT_INFO(input_8042_info)
{
  info->name = "8042 PC Keyboard controller";
  info->ctrl_button_count = INPUT_8042_KEYCOUNT;
  info->ctrl_axe_count = 0;
}

/* 
 * device read operation
 */

DEVINPUT_READ(input_8042_read)
{
  struct input_8042_context_s	*pv = dev->drv_pv;

  assert(id < INPUT_8042_KEYCOUNT);

  return input_state_get(&pv->key_state, id);
}

/* 
 * device write operations
 */

DEVINPUT_WRITE(input_8042_write)
{
  // struct input_8042_context_s	*pv = dev->drv_pv;

  assert(id < INPUT_8042_KEYCOUNT);

  /* FIXME handle led status change here */

  return -ERANGE;
}

/* 
 * device read operation
 */

DEVINPUT_SETCALLBACK(input_8042_setcallback)
{
  struct input_8042_context_s	*pv = dev->drv_pv;

  assert((id == DEVINPUT_CTRLID_ALL) || (id < INPUT_8042_KEYCOUNT));

  if (id == DEVINPUT_CTRLID_ALL)
    {
      devinput_ctrlid_t	i;

      for (i = 0; i < INPUT_8042_KEYCOUNT; i++)
	{
	  pv->events[i].callback = callback;
	  pv->events[i].private = private;
	  pv->events[i].type = type;
	}
    }
  else
    {
      pv->events[id].callback = callback;
      pv->events[id].private = private;
      pv->events[id].type = type;
    }

  return 0;
}

static inline void
input_8042_keyevent(struct input_8042_context_s *pv, devinput_ctrlid_t keyid, bool_t down)
{
  input_state_set(&pv->key_state, keyid, down);

  if (pv->events[keyid].callback != NULL)
    pv->events[keyid].callback(keyid, down, pv->events[keyid].private);

  /* FIXME handle led status change here depending on key pressed */
}

static void
input_8042_scancode_default(struct device_s *dev, uint8_t scancode);

static void
input_8042_scancode_led(struct device_s *dev, uint8_t scancode)
{
  struct input_8042_context_s	*pv = dev->drv_pv;

  if (scancode == 0xfa)
    cpu_io_write_8(dev->addr[0] + KEYB_8042_REG1, pv->led_state);

  pv->scancode = input_8042_scancode_default;
}

static void
input_8042_scancode_ext(struct device_s *dev, uint8_t scancode)
{
  struct input_8042_context_s	*pv = dev->drv_pv;
  static const devinput_ctrlid_t keycodes[128] =
    {
      /* FIXME */
    };

  pv->scancode = input_8042_scancode_default;
  input_8042_keyevent(pv, keycodes[scancode & 0x7f], !(scancode & 0x80));
}

static void
input_8042_scancode_default(struct device_s *dev, uint8_t scancode)
{
  struct input_8042_context_s	*pv = dev->drv_pv;
  static const devinput_ctrlid_t keycodes[128] =
    {
      /* FIXME */
    };

  switch (scancode)
    {
    case (0xe0):		/* extended scancode */
      pv->scancode = input_8042_scancode_ext;
      break;

    case (0xfa):
      break;			/* command ack */

    default:
      input_8042_keyevent(pv, keycodes[scancode & 0x7f], !(scancode & 0x80));
      break;
    }
}

static void input_8042_updateleds(struct device_s *dev)
{
  struct input_8042_context_s	*pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&pv->lock);

  pv->scancode = input_8042_scancode_led;
  cpu_io_write_8(dev->addr[0] + KEYB_8042_REG1, 0xed);

  LOCK_RELEASE_IRQ(&pv->lock);
}

/*
 * IRQ handler
 */

DEV_IRQ(input_8042_irq)
{
  struct input_8042_context_s	*pv = dev->drv_pv;
  bool_t			res = 0;

  lock_spin(&pv->lock);

  while (cpu_io_read_8(dev->addr[0] + KEYB_8042_REG2) & KEYB_8042_VALUE1)
    {
      uint8_t	inbyte = cpu_io_read_8(dev->addr[0] + KEYB_8042_REG1);

      pv->scancode(dev, inbyte);

      res = 1;
    }

  lock_release(&pv->lock);

  return res;
}

/* 
 * device close operation
 */

DEV_CLEANUP(input_8042_cleanup)
{
  struct input_8042_context_s	*pv = dev->drv_pv;

  lock_destroy(&pv->lock);
  mem_free(pv);
}

/* 
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	input_8042_drv =
{
  .f_init		= input_8042_init,
  .f_cleanup		= input_8042_cleanup,
  .f_irq		= input_8042_irq,
  .f.input = {
    .f_info		= input_8042_info,
    .f_read		= input_8042_read,
    .f_setcallback	= input_8042_setcallback,
  }
};
#endif

DEV_INIT(input_8042_init)
{
  struct input_8042_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &input_8042_drv;
#endif

#if defined(CONFIG_ARCH_IBMPC)
  assert(dev->addr[0] == 0x60);
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  lock_init(&pv->lock);
  input_state_init(&pv->key_state);

  pv->led_state = KEYB_8042_LED_NUM;
  input_8042_updateleds(dev);

  return 0;
}


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

#include <hexo/device/char.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include "tty-vga.h"

#include "tty-vga-private.h"

/**************************************************************/

/*
 * Write to VGA adapter register
 */

static inline void
vga_reg_write(uintptr_t addr, uint8_t index, uint8_t data)
{
  cpu_io_write_8(addr, index);  
  cpu_io_write_8(addr + 1, data);
}

/*
 * Read from VGA adapter register
 */

static inline uint8_t
vga_reg_read(uintptr_t addr, uint8_t index)
{
  cpu_io_write_8(addr, index);  
  return cpu_io_read_8(addr + 1);
}

/**************************************************************/

/*
 * Move cursor position on vga text mode screen
 */

inline void tty_vga_updatecursor(struct device_s *dev)
{
  struct tty_vga_context_s		*pv = dev->drv_pv;
  uint_fast16_t				cur_addr;

  cur_addr = (pv->ypos * pv->width + pv->xpos);

  /* move cursor on screen */
  vga_reg_write(dev->addr[VGA_TTY_ADDR_CRTC], VGA_CRTCREG_CURPOS_HI, cur_addr >> 8);
  vga_reg_write(dev->addr[VGA_TTY_ADDR_CRTC], VGA_CRTCREG_CURPOS_LO, cur_addr & 0xff);
}

/*
 * Set curosr postion with bound checking
 */

inline bool_t
tty_vga_setcursor(struct device_s *dev, int_fast8_t x, int_fast8_t y)
{
  struct tty_vga_context_s	*pv = dev->drv_pv;
  bool_t			res = 0;

  /* check cursor position boundary */
  if (x < 0)
    {
      x = 0;
      res = 1;
    }

  if (x >= pv->width)
    {
      x = pv->width - 1;
      res = 1;
    }

  if (y < 0)
    {
      y = 0;
      res = 1;
    }

  if (y >= pv->height)
    {
      y = pv->height - 1;
      res = 1;
    }

  /* set new position */
  pv->xpos = x;
  pv->ypos = y;

  tty_vga_updatecursor(dev);

  return res;
}

/*
 * Clear rows on text screen
 */

void
tty_vga_clear(struct device_s *dev, int_fast8_t rowstart, int_fast8_t rowend)
{
  vga_text_buf_t		buf = (vga_text_buf_t)(dev->addr[VGA_TTY_ADDR_BUFFER]);
  struct tty_vga_context_s	*pv = dev->drv_pv;
  int_fast16_t			i;

  for (i = rowstart * pv->width; i < rowend * pv->width; i++)
    {
      buf[i].c = ' ';
      buf[i].attrs = 0x07;
#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI
      buf[i].foreground = pv->forecolor;
      buf[i].background = pv->backcolor;
#endif
    }
}

/*
 * Reset terminal to default settings
 */

void
tty_vga_reset(struct device_s *dev)
{
  struct tty_vga_context_s	*pv = dev->drv_pv;
  pv->width = 80;
  pv->height = 25;

#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI
  pv->forecolor = 7;
  pv->backcolor = 0;
  pv->blink = 0;
  pv->bright = 0;
  pv->reverse = 0;
  pv->linewrap = 1;
  pv->nlmode = 1;
  pv->insert = 0;
  pv->xsave = 0;
  pv->xsave = 0;
#endif

  tty_vga_clear(dev, 0, pv->height);
  tty_vga_setcursor(dev, 0, 0);
}

/*
 * Clear a single text row
 */

inline void
tty_vga_clear_row(struct device_s *dev, uint_fast8_t row,
		  int_fast8_t colstart, int_fast8_t colend)
{
  vga_text_buf_t		buf = (vga_text_buf_t)(dev->addr[VGA_TTY_ADDR_BUFFER]);
  struct tty_vga_context_s	*pv = dev->drv_pv;
  int_fast16_t	i;

  buf += pv->width * row;

  for (i = colstart; i < colend; i++)
    {
      buf[i].c = ' ';
      buf[i].attrs = 0x07;
#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI
      buf[i].foreground = pv->forecolor;
      buf[i].background = pv->backcolor;
#endif
    }
}

#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI

#if 0

/* 
 * Shift row block down
 */

inline void
tty_vga_shift_rows_down(struct device_s *dev, uint_fast8_t rowstart, uint_fast8_t count)
{
  vga_text_buf_t		buf = (vga_text_buf_t)(dev->addr[VGA_TTY_ADDR_BUFFER]);
  struct tty_vga_context_s	*pv = dev->drv_pv;
  int_fast16_t			i;

  rowstart *= pv->width;
  count *= pv->width;

  /* scroll buffer down */
  for (i = rowstart + count; i >= rowstart; i--)
    buf[i + count] = buf[i];
}

/* 
 * shift row block up
 */

inline void
tty_vga_shift_rows_up(struct device_s *dev, uint_fast8_t rowstart, uint_fast8_t count)
{
  vga_text_buf_t		buf = (vga_text_buf_t)(dev->addr[VGA_TTY_ADDR_BUFFER]);
  struct tty_vga_context_s	*pv = dev->drv_pv;
  int_fast16_t			i;

  rowstart *= pv->width;
  count *= pv->width;

  /* scroll buffer down */
  for (i = rowstart; i < rowstart + count; i++)
    buf[i] = buf[i + count];
}
#endif
/* 
 * scroll one row up
 */

inline void
tty_vga_scroll_down(struct device_s *dev, uint_fast8_t count)
{
  vga_text_buf_t		buf = (vga_text_buf_t)(dev->addr[VGA_TTY_ADDR_BUFFER]);
  struct tty_vga_context_s	*pv = dev->drv_pv;
  int_fast16_t			i;

  /* scroll buffer down */
  for (i = pv->width * pv->height - 1; i >= pv->width; i--)
    buf[i] = buf[i - pv->width];

  /* clear first line */
  tty_vga_clear_row(dev, 0, 0, pv->width);
}

/* 
 * scroll one row down
 */

#endif

inline void
tty_vga_scroll_up(struct device_s *dev, uint_fast8_t count)
{
  vga_text_buf_t		buf = (vga_text_buf_t)(dev->addr[VGA_TTY_ADDR_BUFFER]);
  struct tty_vga_context_s	*pv = dev->drv_pv;
  int_fast16_t			i;

  /* scroll buffer up */
  for (i = 0; i < pv->width * (pv->height - 1); i++)
    buf[i] = buf[i + pv->width];

  /* clear last line */
  tty_vga_clear_row(dev, pv->height - 1, 0, pv->width);
}

/* 
 * new line with scrolling
 */

static inline void
tty_vga_newline(struct device_s *dev)
{
  struct tty_vga_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI
  if (tty_vga_setcursor(dev, pv->nlmode ? 0 : pv->xpos, pv->ypos + 1))
    tty_vga_scroll_up(dev, 1);
#else
  if (tty_vga_setcursor(dev, 0, pv->ypos + 1))
    tty_vga_scroll_up(dev, 1);
#endif
}

/* 
 * put char at cursor position
 */

static void
tty_vga_putchar(struct device_s *dev, uint8_t data)
{
  vga_text_buf_t		buf = (vga_text_buf_t)(dev->addr[VGA_TTY_ADDR_BUFFER]);
  struct tty_vga_context_s	*pv = dev->drv_pv;  

  if (pv->xpos >= pv->width)
    {
#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI
      if (!pv->linewrap)
	pv->xpos = pv->width - 1;
      else
#endif
	tty_vga_newline(dev);
    }
#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI
  else
    {
      if (pv->insert)
	tty_vga_ansi_insert(dev);
    }
#endif

  buf += pv->width * pv->ypos + pv->xpos++;

  buf->c = data;

#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI
  if (pv->reverse)
    {
      buf->background = pv->forecolor;
      buf->foreground = pv->backcolor;
    }
  else
    {
      buf->background = pv->backcolor;
      buf->foreground = pv->forecolor;
    }

  buf->blink = pv->blink;
  buf->bright = pv->bright;
#else
  buf->attrs = 0x07;
#endif
}

/**************************************************************/

/* 
 * process char in default mode
 */

void
tty_vga_process_default(struct device_s *dev, uint8_t c)
{
  struct tty_vga_context_s	*pv = dev->drv_pv;

  switch (c)
    {
    case(12):
    case(11):
    case(10):
      tty_vga_newline(dev);
      break;

    case(13):
      tty_vga_setcursor(dev, 0, pv->ypos);
      break;

    case ('\b'):
      tty_vga_setcursor(dev, pv->xpos - 1, pv->ypos);
      break;

#ifdef CONFIG_DRIVER_CHAR_VGATTY_ANSI
    case(27):		/* ESC */
      //if (pv->key_state & VGA_KS_SCROLL)
	pv->process = &tty_vga_process_ansi;
      break;
#endif

    default:
      if (c > 31)
	tty_vga_putchar(dev, c);
    }
}

/**************************************************************/

/* 
 * device read operation
 */

DEVCHAR_READ(tty_vga_read)
{
#ifdef DRIVER_CHAR_VGATTY_HAS_FIFO
  struct tty_vga_context_s	*pv = dev->drv_pv;
  size_t			res;

  res = tty_fifo_pop_array(&pv->read_fifo, data, size);

  return res;
#else
  return 0;
#endif
}

/* 
 * device write operation
 */

DEVCHAR_WRITE(tty_vga_write)
{
  struct tty_vga_context_s	*pv = dev->drv_pv;
  uint_fast16_t			i;

  lock_spin_irq(&pv->lock);

  for (i = 0; i < size; i++)
    /* process each char */
    pv->process(dev, data[i]);

  tty_vga_updatecursor(dev);

  lock_release_irq(&pv->lock);

  return size;
}

/* 
 * device close operation
 */

DEV_CLEANUP(tty_vga_cleanup)
{
  struct tty_vga_context_s	*pv = dev->drv_pv;

#ifdef DRIVER_CHAR_VGATTY_HAS_FIFO
  tty_fifo_destroy(&pv->read_fifo);
#endif
  lock_destroy(&pv->lock);

  mem_free(pv);
}

/* 
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	tty_vga_drv =
{
  .f_init		= tty_vga_init,
  .f_cleanup		= tty_vga_cleanup,
#ifdef CONFIG_DRIVER_CHAR_VGATTY_KEYBOARD
  .f_irq		= tty_vga_irq,
#endif
  .f.chr = {
    .f_read		= tty_vga_read,
    .f_write		= tty_vga_write,
  }
};
#endif

DEV_INIT(tty_vga_init)
{
  struct tty_vga_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &tty_vga_drv;
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

#ifdef DRIVER_CHAR_VGATTY_HAS_FIFO
  /* init tty input fifo */
  tty_fifo_init(&pv->read_fifo);
#endif

  /* set parser automata initial state */
  pv->process = &tty_vga_process_default;

  /* reset terminal */
  tty_vga_reset(dev);

#ifdef CONFIG_DRIVER_CHAR_VGATTY_KEYBOARD
  /* setup keyboard leds */
  pv->key_state = VGA_KS_SCROLL;
  pv->scancode = &tty_vga_scancode_led;
  cpu_io_write_8(0x60, 0xed);
#endif

  return 0;
}


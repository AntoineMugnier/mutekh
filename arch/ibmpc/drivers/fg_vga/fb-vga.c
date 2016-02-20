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

    This driver is based on svgalib code (http://www.svgalib.org/)

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <hexo/types.h>

#include <device/class/fb.h>
#include <device/device.h>
#include <device/driver.h>

#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/lock.h>
#include <string.h>

#include "fb-vga.h"

#include "fb-vga-private.h"

/* 
 * device close operation
 */

DEV_CLEANUP(fb_vga_cleanup)
{
  struct fb_vga_context_s	*pv = dev->drv_pv;

  lock_destroy(&pv->lock);

  mem_free(pv);
}

DEV_FB_GETBUFFER(fb_vga_getbuffer)
{
  struct fb_vga_context_s	*pv = dev->drv_pv;

  return FB_VGA_FB_ADDRESS + page * pv->mode->xres * pv->mode->yres;
}

/* 
 * device open operation
 */

#define fb_vga_use dev_use_generic

DRIVER_DECLARE(fb_vga_drv, 0, "VGA FB", fb_vga,
               DRIVER_FB_METHODS(fb_vga));

DRIVER_REGISTER(fb_vga_drv);

DEV_INIT(fb_vga_init)
{
  struct fb_vga_context_s	*pv;


  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  return 0;
}


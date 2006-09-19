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

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/device.h>

/** device structure identification informations. wildcard values are
    enum driver dependent */
struct devenum_ident_s
{
  uint_fast16_t		vendor;
  uint_fast16_t		device;
};

/** device driver object structure */

#define DRV_MAX_FUNC_COUNT	4

struct driver_s
{
  const struct devenum_ident_s	*id_table;

  dev_init_t			*f_init;
  dev_cleanup_t			*f_cleanup;
  dev_irq_t			*f_irq;

  union {
    void			*ptrs[DRV_MAX_FUNC_COUNT];

#ifdef __DEVICE_CHAR_H__
    /** char devices */
    struct dev_class_char_s	chr;
#endif

#ifdef __DEVICE_ICU_H__
    /** icu devices */
    struct dev_class_icu_s	icu;
#endif

#ifdef __DEVICE_FB_H__
    /** frame buffer devices */
    struct dev_class_fb_s	fb;
#endif


#ifdef __DEVICE_TIMER_H__
    /** timer devices */
    struct dev_class_timer_s	timer;
#endif

#ifdef __DEVICE_ENUM_H__
    /** device enumerator class */
    struct dev_class_enum_s	denum;
#endif

#ifdef __DEVICE_NET_H__
    /** network devices */
    struct dev_class_net_s	net;
#endif
  } f;
};

#endif


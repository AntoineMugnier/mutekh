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


#if !defined(DEVICE_H) || defined(DEVICE_FB_H_)
#error This file can not be included directly
#else

#define DEVICE_FB_H_

#include "../types.h"
#include "../error.h"


/** pixel color packing is text mode */
#define FB_PACK_TEXT	0
/** pixel color packing is palette indexed colors */
#define FB_PACK_INDEX	1
/** pixel color packing is planar */
#define FB_PACK_PLANE	2
/** pixel color packing is RGB */
#define FB_PACK_RGB	3



/** Fb device class graphic mode setup function tempate. */
#define DEVFB_SETMODE(n)	error_t  (n) (struct device_s *dev, uint_fast16_t xres, uint_fast16_t yres, \
					      uint_fast8_t bpp, uint_fast8_t packing)

/** 
    Fb device class setmode() function type.
    Setup a graphic mode for the frame buffer device.

    @param dev pointer to device descriptor
    @param xres frame buffer horizontal pixel resolution
    @param yres frame buffer vertical pixel resolution
    @param bbp bits per pixels
    @param packing pixel color information packing
    @return error code
*/
typedef DEVFB_SETMODE(devfb_setmode_t);

/** Fb device class setmode() methode shortcut */
#define dev_fb_setmode(dev, ...) (dev)->drv->f.chr.f_setmode(dev, __VA_ARGS__ )




/** Fb device class getbuffer() function tempate. */
#define DEVFB_GETBUFFER(n)	uintptr_t  (n) (struct device_s *dev, uint_fast8_t page)

/** 
    Fb device class getbuffer() function type.
    Get frame buffer address

    @param dev pointer to device descriptor
    @param page page index for multiple pages frame buffers
    @return frame buffer address
*/
typedef DEVFB_GETBUFFER(devfb_getbuffer_t);

/** Fb device class getbuffer() methode shortcut */
#define dev_fb_getbuffer(dev, ...) (dev)->drv->f.chr.f_getbuffer(dev, __VA_ARGS__ )



/** Fb device class flippage() function tempate. */
#define DEVFB_FLIPPAGE(n)	error_t  (n) (struct device_s *dev, uint_fast8_t page)

/** 
    Fb device class flippage() function type.
    Set current displayed page

    @param dev pointer to device descriptor
    @param page page index
    @return error code
*/
typedef DEVFB_FLIPPAGE(devfb_flippage_t);

/** Fb device class flippage() methode shortcut */
#define dev_fb_flippage(dev, ...) (dev)->drv->f.chr.f_flippage(dev, __VA_ARGS__ )



/** Fb device class methodes */
struct dev_class_fb_s
{
  devfb_setmode_t	*f_setmode;
  devfb_getbuffer_t	*f_getbuffer;
  devfb_flippage_t	*f_flippage;
};


#endif


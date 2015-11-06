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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2009

*/

/**
 * @file
 * @module{Devices support library}
 * @short Liquide cristal display driver API
 */

#ifndef __DEVICE_LCD_H__
#define __DEVICE_LCD_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <gct_platform.h>
#include <gpct/cont_clist.h>

#include <device/driver.h>

struct device_s;
struct driver_s;
struct device_lcd_s;
struct driver_lcd_s;


/** pixel color packing is RGB */
#define LCD_PACK_RGB    0
/** pixel color packing is palette indexed colors */
#define LCD_PACK_INDEX	1

#define LCD_FLIP_HORIZ 1
#define LCD_FLIP_VERT  2
#define LCD_INVERT     4

typedef uint16_t lcd_coord_t;

struct lcd_info_s
{
	uint8_t packing;
	uint8_t r_bpp;
	uint8_t g_bpp;
	uint8_t b_bpp;
	lcd_coord_t width;
	lcd_coord_t height;
	uint_fast8_t flags;
};

struct		lcd_pal_s
{
  uint8_t	r;
  uint8_t	g;
  uint8_t	b;
};


struct lcd_req_s;

#define DEV_LCD_CALLBACK(x) void (x)(void *context, struct lcd_req_s *req)

typedef DEV_LCD_CALLBACK(dev_lcd_callback_t);

enum lcd_req_type_e
{
	LCD_REQ_BLIT,
	LCD_REQ_SET_PALETTE,
	LCD_REQ_SET_MODE,
	LCD_REQ_SET_CONTRAST,
};

GCT_CONTAINER_TYPES(dev_lcd_queue, CLIST,
struct lcd_req_s
{
	enum lcd_req_type_e type;
	union {
		struct {
			lcd_coord_t xmin;
			lcd_coord_t xmax;
			lcd_coord_t ymin;
			lcd_coord_t ymax;
			const uint8_t *src;
		} blit;
		struct {
			struct lcd_pal_s *pal;
			size_t count;
		} palette;
		struct {
			uint_fast8_t bpp;
			uint_fast8_t packing;
			uint_fast8_t flags;
		} mode;
		struct {
			uint_fast8_t value;
		} contrast;
	};
	dev_lcd_callback_t *callback;
	void *callback_data;
	dev_lcd_queue_entry_t	queue_entry; /* used by driver to enqueue requests */
}, queue_entry);

GCT_CONTAINER_FCNS(dev_lcd_queue, CLIST, ALWAYS_INLINE, dev_lcd_queue);

/** Lcd device class request() function tempate. */
#define DEV_LCD_REQUEST(n)	error_t  (n) (struct device_lcd_s *accessor, struct lcd_req_s *req)

typedef DEV_LCD_REQUEST(dev_lcd_request_t);



/** Lcd device class getinfo() function tempate. */
#define DEV_LCD_GETINFO(n)	const struct lcd_info_s * (n) (struct device_lcd_s *accessor)

/**
    Lcd device class getinfo() function type.  Get a device
    information structure.

    @param dev pointer to device descriptor
    @return the information struct
*/
typedef DEV_LCD_GETINFO(dev_lcd_getinfo_t);



DRIVER_CLASS_TYPES(lcd,
                    dev_lcd_request_t	*f_request;
                    dev_lcd_getinfo_t	*f_getinfo;
                    );


#define DRIVER_LCD_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_lcd_s){   \
    .class_ = DRIVER_CLASS_LCD,                                  \
    .f_request = prefix ## _request,                             \
    .f_getinfo = prefix ## _getinfo,                             \
  })

ssize_t dev_lcd_set_palette(struct device_lcd_s *accessor, struct lcd_pal_s *palette, size_t count);

/**
    Lcd device class blit() function type.  Blit the rectangle from
    xmin,ymin to xmax,ymax. [xy]max is not in the blit.  This function
    expects a buffer of (xmax-xmin)*(ymax-ymin) pixels.  If bpp is not
    on a byte boundary (eg 12bpp), pixels must be packed (eg 2 pixels
    in 3 bytes).

    @param dev pointer to device descriptor
	@param xmin min x column
	@param xmax max x column -- excluded from blit
	@param ymin min y line
	@param ymax max y line -- excluded from blit
	@param src pixel data
    @return error level
*/

ssize_t dev_lcd_blit(struct device_lcd_s *accessor,
					 lcd_coord_t xmin,
					 lcd_coord_t ymin,
					 lcd_coord_t xmax,
					 lcd_coord_t ymax,
					 const uint8_t *src );
/**
    Lcd device class setmode() function type.
    Setup a graphic mode for the frame buffer device.

    @param dev pointer to device descriptor
    @param bbp bits per pixels
    @param packing pixel color information packing
    @return error code
*/
ssize_t dev_lcd_setmode(struct device_lcd_s *accessor,
						uint_fast8_t bpp, uint_fast8_t packing,
						uint_fast8_t flags);

/**
    Lcd device class setcontrast() function type.
    Setup a graphic contrast

    @param dev pointer to device descriptor
    @param contrast contrast value
    @return error code
*/
ssize_t dev_lcd_setcontrast(struct device_lcd_s *accessor, uint_fast8_t contrast);

#endif


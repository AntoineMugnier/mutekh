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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2017
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2017
*/

/**
   @file
   @module {Core::Devices support library}
   @short Display device driver API
   @index {Display device} {Device classes}
   @csee DRIVER_CLASS_DISPLAY
*/

#ifndef __DEVICE_DISPLAY_H__
#define __DEVICE_DISPLAY_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/request.h>

struct driver_s;
struct dev_display_rq_s;
struct driver_display_s;
struct device_display_s;

typedef uint16_t dev_display_coord_t;
typedef uint64_t dev_display_date_t;

enum dev_display_pixfmt_e
{
  /** 1 bit per pixel monochrome, pixel 0 on least significant bit */
  DEV_DISP_PIXFMT_1BPP_LSB,
  /** 1 bit per pixel monochrome, pixel 0 on most significant bit */
  DEV_DISP_PIXFMT_1BPP_MSB,
  /** 2 bit per pixel monochrome, pixel 0 on least significant bit */
  DEV_DISP_PIXFMT_2BPP_LSB,
  /** 2 bit per pixel monochrome, pixel 0 on most significant bit */
  DEV_DISP_PIXFMT_2BPP_MSB,
  /** 2 bit per pixel palette, pixel 0 on least significant bit */
  DEV_DISP_PIXFMT_2BPP_PAL_LSB,
  /** 2 bit per pixel palette, pixel 0 on most significant bit */
  DEV_DISP_PIXFMT_2BPP_PAL_MSB,
  /** 4 bit per pixel monochrome, pixel 0 on least significant bit */
  DEV_DISP_PIXFMT_4BPP_LSB,
  /** 4 bit per pixel monochrome, pixel 0 on most significant bit */
  DEV_DISP_PIXFMT_4BPP_MSB,
  /** 4 bit per pixel palette, pixel 0 on least significant bit */
  DEV_DISP_PIXFMT_4BPP_PAL_LSB,
  /** 4 bit per pixel palette, pixel 0 on most significant bit */
  DEV_DISP_PIXFMT_4BPP_PAL_MSB,
  /** 8 bit per pixel monochrome */
  DEV_DISP_PIXFMT_8BPP,
  /** 8 bit per pixel palette */
  DEV_DISP_PIXFMT_8BPP_PAL,
  /** 16 bit per pixel 0RGB 1:5:5:5, little endian */
  DEV_DISP_PIXFMT_16BPP_0RGB_555_LE,
  /** 16 bit per pixel ARGB 1:5:5:5, little endian */
  DEV_DISP_PIXFMT_16BPP_ARGB_555_LE,
  /** 16 bit per pixel RGB 5:6:5, little endian */
  DEV_DISP_PIXFMT_16BPP_RGB_565_LE,
  /** 16 bit per pixel ARGB 4:4:4:4, little endian */
  DEV_DISP_PIXFMT_16BPP_ARGB_444_LE,
  /** 24 bit per pixel RGB 8:8:8, little endian */
  DEV_DISP_PIXFMT_24BPP_RGB_LE,
  /** 32 bit per pixel ARGB 8:8:8:8, little endian */
  DEV_DISP_PIXFMT_32BPP_ARGB_LE,
};


/** @This specified surface flip and rotate operations. */
enum dev_display_transform_e
{
  DEV_DISPLAY_HR_FLIP = (1 << 0),
  DEV_DISPLAY_VR_FLIP = (1 << 1),
  DEV_DISPLAY_ROT_90  = (1 << 2)
};

/** @This specifies fields of the @ref dev_display_surface_s structure. */
enum dev_display_changes_e
{
  DEV_DISPLAY_CH_DATA      = (1 << 0),
  DEV_DISPLAY_CH_PALETTE   = (1 << 1),
  DEV_DISPLAY_CH_XSTART    = (1 << 2),
  DEV_DISPLAY_CH_YSTART    = (1 << 3),
  DEV_DISPLAY_CH_XEND      = (1 << 4),
  DEV_DISPLAY_CH_YEND      = (1 << 5),
  DEV_DISPLAY_CH_FORMAT    = (1 << 6),
  DEV_DISPLAY_CH_ALPHA     = (1 << 7),
  DEV_DISPLAY_CH_STRIDE    = (1 << 8),
  DEV_DISPLAY_CH_TRANSFORM = (1 << 9),
};

enum dev_display_op_e
{
  DEV_DISP_OP_OVERLAY,
  DEV_DISP_OP_NOP,
  /* ... */
};

/** @see dev_display_info_s */
struct dev_display_mode_info_s
{
  /** width of the screen in pixels */
  dev_display_coord_t width;
  /**  height of the screen in pixels */
  dev_display_coord_t height;

  /** Mask of surface pixel formats which can be used when this mode
      is in use. When the bit at @em index is set in the mask, a
      surface with the pixel format specified at index in the array
      @ref dev_display_fmt_info_s can be used to display in this
      mode. */
  uint32_t pix_conv_mask;

  /** Allowed pixel alignment of @ref dev_display_surface_s::xsrc. */
  dev_display_coord_t xsrc_align;

  /** Allowed pixel alignment of @ref dev_display_surface_s::ysrc. */
  dev_display_coord_t ysrc_align;

  /** Allowed pixel alignment of @ref dev_display_surface_s::xstart and
      dev_display_surface_s::xend + 1. Zero can be used to indicates
      that surfaces width must span the entire screen. */
  dev_display_coord_t x_align;

  /** Allowed pixel alignment of @ref dev_display_surface_s::ystart and
      dev_display_surface_s::yend + 1. Zero can be used to indicates
      that surfaces height must span the entire screen. */
  dev_display_coord_t y_align;

  /** Byte alignment of any pixel row in surface data. */
  uint8_t log2_row_align:2;

  /** Native pixel format used by the display in this mode. Pixel
      format index is device specific and is described in @ref
      dev_display_fmt_info_s */
  uint8_t pix_fmt_index:5;

  /** Specifies if padding is allowed between rows in surface data. */
  uint8_t padding:1;

  /** Specifies some supported refresh rates in frames per
      second. Zero padded as needed. */
  uint8_t rates[7];
};

/** @see dev_display_info_s */
struct dev_display_fmt_info_s
{
  /** number of bits per pixel minus one. */
  uint8_t bpp_m1:5;

  /** number of bits per palette entry minus one.
      Zero indicates that no palette is used. */
  uint8_t pal_bpp_m1:5;

  /** pixel format descriptor */
  enum dev_display_pixfmt_e fmt;
};

/** @see dev_display_info_t */
struct dev_display_info_s
{
  const struct dev_display_mode_info_s *modes;
  const struct dev_display_fmt_info_s  *pixfmts;

  /** Mask of surface fields which can be updated by the @ref
      dev_display_callback_t function. */
  enum dev_display_changes_e  changes:16;

  /** Number of descriptors in the @ref modes array */
  uint8_t modes_count;
  /** Number of descriptors in the @ref pixfmt array */
  uint8_t pixfmts_count;

  /** Maximum number of surfaces handled by a request. */
  uint8_t max_surface_count;

  /** Mask of supported transforms */
  enum dev_display_transform_e transform_mask;

  /** Mask of supported surface operations @see dev_display_op_e */
  uint8_t op_mask;
};

/** @csee dev_display_info_t */
#define DEV_DISPLAY_INFO(n)                                \
  const struct dev_display_info_s * (n)(                   \
    const struct device_display_s *accessor)

/** @This returns the display device capabilities. */
typedef DEV_DISPLAY_INFO(dev_display_info_t);


/** @csee dev_display_alloc_t */
#define DEV_DISPLAY_ALLOC(n) void * (n)(struct device_display_s *dev, void *ptr, size_t size)

/** @This allocates memory used to store surface data.

    Depending on the hardware, this allocates a buffer in the display
    controller internal memory or with specific alignment constraints.

    The usage is similar to the standard @tt realloc
    function. Resizing may not be supported.
*/
typedef DEV_DISPLAY_ALLOC(dev_display_alloc_t);



/** @see dev_display_request_t
    @see dev_display_rq_s */
struct dev_display_surface_s
{
  /** Pixel data. layout depends on pixel format.
      The bytes size of the buffer is @em {stride * (ysrc + yend - ystart + 1)}.

      The memory must be allocated by the @ref dev_display_alloc_t function. */
  void *data;

  /** Palette data. Buffer size and entries layout depend on pixel
      format.  Not used when the pixel format does not rely on a
      palette or when the hardware does not support changing the
      palette. */
  void *palette;

  /** When the request callback returns 1, it must also updates this
      field in order to indicate which fields have been updated. */
  enum dev_display_changes_e  changes:16;

  /** @multiple coordinates of the pixel in the surface shown at
      (xstart, ystart) on the display. */
  dev_display_coord_t xsrc, ysrc;

  /** @multiple coordinates of the surface on the display */
  dev_display_coord_t xstart, ystart;
  dev_display_coord_t xend, yend;

  /** pixel format. pixel format index is device specific and is
      described in @ref dev_display_fmt_info_s */
  uint8_t format;

  /** 0 is transparent, 1 is transparent */
  uint8_t alpha;

  /** surface operator. */
  enum dev_display_op_e op;

  /** size of a single row in bytes. */
  uint16_t stride;

  /** This applies some transform to the scan order. The scan order
      and orientation of the whole display is device specific. */
  enum dev_display_transform_e transform;
};

/** @csee dev_display_callback_t */
#define DEV_DISPLAY_CALLBACK(n) bool_t (n)(struct dev_display_rq_s *rq, \
                                           dev_display_date_t next_date, uint16_t next_id)

/** @This is called when the display has been refreshed. The ownership
    of the surface buffer is transferred back to the user.

    The display request terminates if this callback returns 0. If this
    callback returns 1, the ownership of the buffer is transferred
    again to the driver and an other refresh cycle is scheduled.

    The callback function may update the following fields of the
    request object before returning:
    @list
      @item The @ref dev_display_rq_s::deadline field.
      @item The @ref dev_display_rq_s::surfaces_count field.
      @item Any field of the @ref dev_display_surface_s entries
        allowed in the capabilities.
    @end list

    The @tt next_date parameter gives an estimate of the date of the
    next call to this function, assuming the @ref deadline field of
    the request will be zero. The @tt next_id parameter is useful for
    calling the @ref dev_display_deadline_t function.

    It is not permitted to call the dev_display_request_t function in
    order to start a new operation from this callback. This callback
    is potentially invoked from an interrupt handler. */
typedef DEV_DISPLAY_CALLBACK(dev_display_callback_t);

/** @see dev_display_request_t */
struct dev_display_rq_s
{
  /** This can be used to delay the invocation of the @ref f_done
      callback function. The callback will be called immediately after
      the first display if zero. The deadline can be set far in the
      future and then changed by calling the @ref dev_display_deadline_t
      function of the driver.

      If the display hardware refreshes periodically, the callback
      will be called at the end of the first refresh after the
      deadline.

      The driver should also implement the timer device class in order
      to provide access to the time base of the display */
  dev_display_date_t deadline; 

  /** This function is called when the display has been refreshed and
      the deadline has been reached. See @ref dev_display_callback_t
      for details. */
  dev_display_callback_t *f_done;

  /** This points to an array of surface descriptors involved in
     refresh of the display. This can be changed during execution of
     the request. In this case, the @tt changes field of the
     new descriptors must be updated accordingly. */
  const struct dev_display_surface_s *surfaces;

  /** This specifies the number of surfaces involved in refresh of the
     display. This can be changed during execution of the request. */
  uint8_t surfaces_count;

  /** index of the display mode. Display mode index is device specific
      and is described in @ref dev_display_mode_info_s. */
  uint8_t mode;

  /** Display refresh rate in frames per second. Some supported rates
      are specified in @ref dev_display_mode_info_s. */
  uint8_t rate;
};

/** @see dev_display_request_t */
#define DEV_DISPLAY_REQUEST(n)                                         \
  error_t (n)(                                                         \
    const struct device_display_s *accessor,                           \
    struct dev_display_rq_s *rq)

/** @This starts a display refresh request. When the display has been
    refreshed, the @ref dev_display_callback_t function provided in
    the request is invoked. The callback has to decide if the request
    terminates or continues with an other refresh cycle.

    This function may return the following error codes:
    @list
      @item @tt -EBUSY if a request is already running.
      @item @tt -ENOTSUP if some attributes of the request are
        not supported by the hardware. The @ref dev_display_info_t
        function can be used to check what is supported.
      @item @tt 0 on success.
    @end list

    The display may switch off when no request is running. */
typedef DEV_DISPLAY_REQUEST(dev_display_request_t);

/** @see dev_display_deadline_t */
#define DEV_DISPLAY_DEADLINE(n)                                       \
  error_t (n)(                                                        \
    const struct device_display_s *accessor,                          \
    struct dev_display_rq_s *rq,                                      \
    dev_display_date_t deadline, int16_t frame_id)

/** @This changes the deadline of the request before the callback is
    invoked. This can be used to reschedule the change of the
    currently displayed frame.

    The identifier provided by the last callback must be passed. The
    identifier of the first frame before the first invocation of the
    callback is 0.

    The function returns @tt -ENOENT if the frame identifier does not
    match the index of the frame currently displayed. */
typedef DEV_DISPLAY_DEADLINE(dev_display_deadline_t);


/** @see dev_display_config_t */
struct dev_display_config_s
{
  /** Display brightness */
  uint8_t brightness;
};

struct dev_display_screeninfo_s
{
  /** Horizontal resolution of the screen. */
  uint32_t xres;

/** Vertical resolution of the screen. */
  uint32_t yres;

/** PPI of the screen. */
  uint16_t ppi;
};


/** @see dev_display_config_t */
#define DEV_DISPLAY_CONFIG(n)                                          \
  error_t (n)(                                                         \
    const struct device_display_s *accessor,                           \
    struct dev_display_config_s *cfg)

/** @This changes the configuration of the display.
    This can be used while a request is running. */
typedef DEV_DISPLAY_CONFIG(dev_display_config_t);



DRIVER_CLASS_TYPES(DRIVER_CLASS_DISPLAY, display,
                   dev_display_info_t              *f_info;
                   dev_display_config_t            *f_config;
                   dev_display_alloc_t             *f_alloc;
                   dev_display_request_t           *f_request;
                   dev_display_deadline_t          *f_deadline;
                   );

/** @see driver_display_s */
#define DRIVER_DISPLAY_METHODS(prefix)                                 \
  ((const struct driver_class_s*)&(const struct driver_display_s){     \
    .class_ = DRIVER_CLASS_DISPLAY,                                    \
      .f_info     = prefix ## _info,                                   \
      .f_config   = prefix ## _config,                                 \
      .f_alloc    = prefix ## _alloc,                                  \
      .f_request  = prefix ## _request,                                \
      .f_deadline = prefix ## _deadline,                               \
  })

ALWAYS_INLINE error_t device_get_res_display(const struct device_s *dev,
                                          struct dev_display_screeninfo_s *cfg)
{
  struct dev_resource_s *r;

  r = device_res_get(dev, DEV_RES_DISPLAY, 0);
  if (r == NULL)
    return -ENOENT;

  cfg->xres = r->u.display.xres;
  cfg->yres = r->u.display.yres;
  cfg->ppi  = r->u.display.ppi;

  return 0;
}

# define DEV_STATIC_RES_DISPLAY(width_, height_, ppi_)               \
  {                                                                  \
    .type = DEV_RES_DISPLAY,                                         \
       .u = { .display = {                                           \
      .xres = (width_),                                              \
      .yres = (height_),                                             \
      .ppi = (ppi_),                                                 \
    } }                                                              \
  }


#endif

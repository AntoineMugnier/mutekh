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
#include <device/resources.h>

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

/** @This specifies fields of the @ref dev_display_surface_s structure
    that have been modified by the @ref dev_display_callback_t
    function. */
enum dev_display_surface_changes_e
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
  DEV_DISPLAY_CH_SINIT     = 0x3ff,
};

/** @This specifies fields of the @ref dev_display_rq_s structure
    that have been modified by the @ref dev_display_callback_t
    function. */
enum dev_display_rq_changes_e
{
  DEV_DISPLAY_CH_RATE      = (1 << 0),
  DEV_DISPLAY_CH_SYNC      = (1 << 1),
  DEV_DISPLAY_CH_SCOUNT    = (1 << 2),
  DEV_DISPLAY_CH_RINIT     = 0x7,
};

enum dev_display_rq_sync_e
{
  /** Display the next frame based on the current frame rate. */
  DEV_DISPLAY_VSYNC,
  /** Schedule display of the next frame at the deadline from the
      request structure. */
  DEV_DISPLAY_SCHEDULE,
  /** Keep the current frame displayed until the @ref
      dev_display_resume_t function is called. */
  DEV_DISPLAY_PERSIST,
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
  /** height of the screen in pixels */
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
  enum dev_display_surface_changes_e  schanges:16;

  /** Mask of request fields which can be updated by the @ref
      dev_display_callback_t function. */
  enum dev_display_rq_changes_e  rchanges:8;

  /** Number of descriptors in the @ref modes array */
  uint8_t modes_count;
  /** Number of descriptors in the @ref pixfmt array */
  uint8_t pixfmts_count;

  /** Maximum number of surfaces handled by a request. */
  uint8_t max_surface_count;

  /** Mask of supported transforms */
  enum dev_display_transform_e transform_mask;

  /** Specifies if parts of the display not covered by a surface are
      persistent between frames. */
  uint8_t persistent:1;

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
      field in order to indicate which fields have been updated. When
      the surface is to be displayed for the first time in a request,
      @ref DEV_DISPLAY_CH_SINIT must be used.

      The driver will clear all bits except @ref DEV_DISPLAY_CH_DATA
      after display of the surface. When the display is persistent (see
      @ref dev_display_mode_info_s), the driver will also clear @ref
      DEV_DISPLAY_CH_DATA. */
  enum dev_display_surface_changes_e  changes:16;

  /** @multiple coordinates of the pixel in the surface shown at
      (xstart, ystart) on the display. */
  dev_display_coord_t xsrc, ysrc;

  /** @multiple coordinates of the surface on the display */
  dev_display_coord_t xstart, ystart;
  dev_display_coord_t xend, yend;

  /** pixel format. pixel format index is device specific and is
      described in @ref dev_display_fmt_info_s */
  uint8_t format;

  /** 0 is transparent, 255 is opaque */
  uint8_t alpha;

  /** surface operator. */
  enum dev_display_op_e op;

  /** storage size of a single row in bytes. */
  uint16_t stride;

  /** This applies some transform to the scan order. The scan order
      and orientation of the whole display is device specific. */
  enum dev_display_transform_e transform;
};

/** @csee dev_display_callback_t */
#define DEV_DISPLAY_CALLBACK(n)                                         \
  bool_t (n)(struct dev_display_rq_s *rq, dev_display_date_t date)

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

    It may also update most fields of the surfaces. The @ref
    dev_display_surface_s::changes field needs to be updated as well.

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
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  union {
    /** This can be used to schedule the next display at the specified
        absolute time. @see DEV_DISPLAY_SCHEDULE */
    dev_display_date_t deadline;

    /** Display refresh rate in frames per second.
        @see DEV_DISPLAY_KEEP_RATE @see DEV_DISPLAY_NEW_RATE */
    struct {
      uint16_t num, denom;
    } rate;
  };

  /** This function is called when the display has been refreshed.
      See @ref dev_display_callback_t for details. */
  dev_display_callback_t *f_done;

  /** This points to an array of surface descriptors involved in
     refresh of the display. This can be changed during execution of
     the request. In this case, the @tt changes field of the
     new descriptors must be updated accordingly. */
  struct dev_display_surface_s *surfaces;

  /** When the request callback returns 1, it must also updates this
      field in order to indicate which fields have been updated. When
      the request is pushed, @ref DEV_DISPLAY_CH_RINIT must be used.

      The driver will clear all bits after display of the surface. */
  enum dev_display_rq_changes_e changes;

  /** This specifies when the @ref dev_display_callback_t function is
      invoked. */
  enum dev_display_rq_sync_e sync;

  /** This specifies the number of surfaces involved in refresh of the
     display. */
  uint8_t surfaces_count;

  /** index of the display mode. Display mode index is device specific
      and is described in @ref dev_display_mode_info_s. */
  uint8_t mode;
};

DEV_REQUEST_INHERIT(display);

/** @see dev_display_request_t */
#define DEV_DISPLAY_REQUEST(n)                                         \
  error_t (n)(                                                         \
    const struct device_display_s *accessor,                           \
    struct dev_display_rq_s *rq)

/** @This starts a display request. When the display has been
    refreshed, the @ref dev_display_callback_t function provided along
    with the request is invoked. The callback has to decide if the
    request terminates or continues with an other refresh cycle.

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

/** @see dev_display_resume_t */
#define DEV_DISPLAY_RESUME(n)                                         \
  error_t (n)(                                                        \
    const struct device_display_s *accessor,                          \
    struct dev_display_rq_s *rq)

/** When the @ref dev_display_callback_t function has been called and
    the request use the @ref DEV_DISPLAY_PERSIST sync mode, this
    function must be called in order to trigger the next refresh. */
typedef DEV_DISPLAY_RESUME(dev_display_resume_t);


DRIVER_CLASS_TYPES(DRIVER_CLASS_DISPLAY, display,
                   dev_display_info_t              *f_info;
                   dev_display_alloc_t             *f_alloc;
                   dev_display_request_t           *f_request;
                   dev_display_resume_t            *f_resume;
                   );

/** @see driver_display_s */
#define DRIVER_DISPLAY_METHODS(prefix)                                 \
  ((const struct driver_class_s*)&(const struct driver_display_s){     \
    .class_ = DRIVER_CLASS_DISPLAY,                                    \
      .f_info     = prefix ## _info,                                   \
      .f_alloc    = prefix ## _alloc,                                  \
      .f_request  = prefix ## _request,                                \
      .f_resume   = prefix ## _resume,                                 \
  })

#endif

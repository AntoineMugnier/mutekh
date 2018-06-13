
#include <gfx/pixel.h>
#include <gfx/line.h>
#include <gfx/circle.h>
#include <gfx/arc.h>
#include <gfx/rect.h>
#include <gfx/blit.h>

#include <gfx/bytecode.h>

#ifdef __MUTEKH__
#include <mutek/bytecode.h>
#endif

#include <string.h>

void gfx_bc_init(struct gfx_bc_context_s *ctx)
{
  ctx->attr = 0;

  uint_fast8_t i;
  for (i = 0; i < 4; i++)
    gfx_surface_dummy(&ctx->s[i]);
  gfx_tilemap_init(&ctx->tilemap, &ctx->s[0], 1, 1, 0);
}

static inline void *
gfx_translate_addr(struct bc_context_s *vm, bc_reg_t addr,
                   size_t size, bool_t writable)
{
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  if (vm->sandbox)
    return bc_translate_addr(vm, addr, size, writable);
#endif
  return (void*)addr;
}

error_t gfx_bc_run(struct bc_context_s *vm,
		   struct gfx_bc_context_s *ctx,
		   uint16_t op)
{
  gfx_pixel_t attr = ctx->attr;
  struct gfx_surface_s *draw = &ctx->s[0];

  switch (op & 0x7f00)
    {
    case 0x1000:
    case 0x1100:
    case 0x1200:
    case 0x1300: {          /* gfx_surface */
      uint32_t d = bc_get_reg(vm, op & 15);
      bc_reg_t p = bc_get_reg(vm, (op >> 4) & 15);
      uint_fast8_t n = (op >> 8) & 3;
      gfx_pos_t w = (d >> 20) & 0xfff;
      gfx_pos_t h = (d >> 8) & 0xfff;
      enum gfx_surface_format fmt = d & 0x1f;

      size_t s;
      if (/* enforce 32 bits alignment */ (p & 3) ||
	  gfx_surface_bytes(&s, w, h, fmt))
	return -ERANGE;

      void *data = gfx_translate_addr(vm, p, s, 0);

      if (!data || gfx_surface_init(ctx->s + n, data,
				    s, w, h, fmt))
	return -ERANGE;

      if (!n)
	ctx->attr = gfx_fmt_desc[fmt].pm;
      break;
    }

    case 0x1400:
    case 0x1500:
    case 0x1600:
    case 0x1700: {          /* gfx_tilemap */
      uint32_t d = bc_get_reg(vm, op & 15);
      uint_fast8_t n = (op >> 4) & 3;
      gfx_pos_t tw = (d >> 20) & 0xfff;
      gfx_pos_t th = (d >> 8) & 0xfff;

      if (gfx_tilemap_init(&ctx->tilemap, &ctx->s[n], tw, th, d & 0xff))
	return -ERANGE;

      break;
    }

    case 0x1800: {          /* gfx_attr_l8 */
      gfx_pixel_t attr = op & 0xff;
      uint_fast8_t bpp = 1 << gfx_fmt_desc[draw->fmt].l2bpp;
      if (bpp < 8)
	attr >>= 8 - bpp;
      else
	attr *= 0x010101 & gfx_fmt_desc[draw->fmt].pm;
      ctx->attr = attr;
      break;
    }

    case 0x1b00: {          /* gfx_swap */
      uint_fast8_t a = (op >> 6) & 3;
      uint_fast8_t b = (op >> 4) & 3;
      struct gfx_surface_s tmp = ctx->s[b];
      ctx->s[b] = ctx->s[a];
      ctx->s[a] = tmp;
      break;
    }

    case 0x2000:
    case 0x2100:
    case 0x2200: {          /* gfx_circle* */
      uint32_t xyc = bc_get_reg(vm, op & 15);
      uint_fast16_t r = bc_get_reg(vm, 15) & 2047;
      uint_fast16_t xc = gfx_vector_xint(xyc);
      uint_fast16_t yc = gfx_vector_yint(xyc);
      uint_fast8_t oct = (op >> 4) & 0xf;
      switch (op & 0x0300)
	{
	case 0x0000:
	  gfx_draw_circle_safe(draw, xc, yc, r, oct, attr);
	  break;
	case 0x0100:
	  gfx_draw_circle_infill_safe(draw, xc, yc, r, oct, attr);
	  break;
	case 0x0200:
	  gfx_draw_circle_outfill_safe(draw, xc, yc, r, oct, attr);
	  break;
	}
      break;
    }

    case 0x2800:
    case 0x2900: {          /* gfx_arc* */
      uint32_t angles = bc_get_reg(vm, op & 15);
      uint32_t xyc = bc_get_reg(vm, (op >> 4) & 15);
      uint_fast16_t r = bc_get_reg(vm, 15) & 2047;
      gfx_draw_arc_angles_safe(draw,
			       gfx_vector_xint(xyc), gfx_vector_yint(xyc),
			       gfx_vector_xint(angles), gfx_vector_yint(angles),
			       r, (op >> 8) & 1, attr);
      break;
    }
#if 0
    case 0x2c00:
    case 0x2d00: {          /* gfx_xyarc* */
      uint32_t xy0 = bc_get_reg(vm, (op >> 4) & 15);
      uint32_t xy1 = bc_get_reg(vm, op & 15);
      uint_fast16_t r = bc_get_reg(vm, 15) & 2047;
      gfx_draw_arc_xy_safe(draw,
			   gfx_vector_xint(xy0), gfx_vector_yint(xy0),
			   gfx_vector_xint(xy1), gfx_vector_yint(xy1),
			   r, (op >> 8) & 1, attr);
      break;
    }
#endif

    case 0x3000: {          /* gfx_line / gfx_point */
      uint32_t p0 = bc_get_reg(vm, op & 15);
      uint32_t p1 = bc_get_reg(vm, (op >> 4) & 15);
      uint_fast16_t x0 = gfx_vector_xint(p0);
      uint_fast16_t y0 = gfx_vector_yint(p0);
      if (p0 == p1)
	{
	  gfx_put_pixel_safe(draw, x0, y0, attr);
	}
      else
	{
	  uint_fast16_t x1 = gfx_vector_xint(p1);
	  uint_fast16_t y1 = gfx_vector_yint(p1);
	  gfx_draw_line_safe(draw, x0, y0, x1, y1, attr);
	}
      break;
    }

    case 0x3200:
    case 0x3300: {          /* gfx_tile */
      uint32_t s = bc_get_reg(vm, (op >> 4) & 15);
      uint32_t t = bc_get_reg(vm, op & 15);
      gfx_pos_t x = gfx_vector_xint(s);
      gfx_pos_t y = gfx_vector_yint(s);

      gfx_draw_tile(draw, &ctx->tilemap, t, x, y, !!(op & 0x100));
      break;
    }

    case 0x3400:
    case 0x3500:
    case 0x3600:
    case 0x3700: {          /* gfx_rect* */
      uint32_t p0 = bc_get_reg(vm, op & 15);
      uint32_t p1 = bc_get_reg(vm, (op >> 4) & 15);
      uint_fast16_t x0 = gfx_vector_xint(p0);
      uint_fast16_t y0 = gfx_vector_yint(p0);
      uint_fast16_t x1 = gfx_vector_xint(p1);
      uint_fast16_t y1 = gfx_vector_yint(p1);
      bool_t filled = (op >> 8) & 1;
      bool_t round = (op >> 9) & 1;
      uint_fast16_t r = round ? bc_get_reg(vm, 15) & 2047 : 0;
      if (filled)
	gfx_draw_rect_fr_safe(draw, x0, y0, x1, y1, r, attr);
      else
	gfx_draw_rect_r_safe(draw, x0, y0, x1, y1, r, attr);
      break;
    }

    case 0x3800:
    case 0x3900:
    case 0x3a00:
    case 0x3b00:
    case 0x3c00:
    case 0x3d00:
    case 0x3e00:
    case 0x3f00:{          /* gfx_tilestr */
      uint32_t len = bc_get_reg(vm, 15);

      uint32_t s = bc_get_reg(vm, (op >> 4) & 15);
      gfx_pos_t x = gfx_vector_xint(s);
      gfx_pos_t y = gfx_vector_yint(s);

      bc_reg_t p = bc_get_reg(vm, op & 15);
      const uint8_t *str = gfx_translate_addr(vm, p, len, 0);

      enum gfx_direction_e dir = (op >> 8) & 3;
      if (str != NULL)
	gfx_draw_tile_string(draw, &ctx->tilemap, str, len,
			     x, y, dir, !!(op & 0x400));
      break;
    }

    case 0x4200: {          /* gfx_clear_l8 */
      gfx_pixel_t attr = op & 0xff;
      uint_fast8_t bpp = 1 << gfx_fmt_desc[draw->fmt].l2bpp;
      if (bpp < 8)
	attr >>= 8 - bpp;
      else
	attr *= 0x010101 & gfx_fmt_desc[draw->fmt].pm;
      gfx_clear(draw, attr);
#if 0
      srand(0);
      unsigned n = 10000;
      while (n--)
	((uint8_t*)draw->ptr)[rand() & draw->mask] = rand();
#endif
      break;
    }

    case 0x4800:
    case 0x4900:
    case 0x4a00:
    case 0x4b00: {          /* gfx_blit */
      struct gfx_surface_s *src = &ctx->s[(op >> 8) & 3];
      uint32_t p0 = bc_get_reg(vm, op & 15);
      uint32_t p1 = bc_get_reg(vm, (op >> 4) & 15);
      uint32_t p15 = bc_get_reg(vm, 15);
      uint_fast16_t x0 = gfx_vector_xint(p0);
      uint_fast16_t y0 = gfx_vector_yint(p0);
      uint_fast16_t x2 = gfx_vector_xint(p1);
      uint_fast16_t y2 = gfx_vector_yint(p1);
      gfx_pos_t w = gfx_vector_xint(p15);
      gfx_pos_t h = gfx_vector_yint(p15);
      gfx_blit_safe(draw, x2, y2,
		    src, x0, y0,
		    w ? w : gfx_width(src),
		    h ? h : gfx_height(src));
      break;
    }

    case 0x4c00: {          /* gfx_blit_overlap */
      uint32_t p0 = bc_get_reg(vm, op & 15);
      uint32_t p1 = bc_get_reg(vm, (op >> 4) & 15);
      uint32_t p15 = bc_get_reg(vm, 15);
      uint_fast16_t x0 = gfx_vector_xint(p0);
      uint_fast16_t y0 = gfx_vector_yint(p0);
      uint_fast16_t x2 = gfx_vector_xint(p1);
      uint_fast16_t y2 = gfx_vector_yint(p1);
      gfx_pos_t w = gfx_vector_xint(p15);
      gfx_pos_t h = gfx_vector_yint(p15);
      gfx_blit_overlap_safe(draw, x2, y2,
			    x0, y0, w, h);
      break;
    }

    case 0x5800:
    case 0x5900:
    case 0x5a00:
    case 0x5b00:
    case 0x5c00:
    case 0x5d00:
    case 0x5e00:
    case 0x5f00:		/* gfx_addyi */
    case 0x5000:
    case 0x5100:
    case 0x5200:
    case 0x5300:
    case 0x5400:
    case 0x5500:
    case 0x5600:
    case 0x5700: {		/* gfx_addxi */
      uint_fast8_t d = op & 3;
      uint32_t p0 = bc_get_reg(vm, d);
      uint32_t x = gfx_vector_x(p0);
      uint32_t y = gfx_vector_y(p0);
      uint32_t i = (op << 3) & 0x3fe0;
      i = (0x2000 ^ i) - 0x2000;
      if (op & 0x0800)
	y += i;
      else
	x += i;
      bc_set_reg(vm, d, gfx_vector_xy_2p(x, y));
      break;
    }

    case 0x6000:		/* gfx_addv */
    case 0x6100: {		/* gfx_subv */
      uint_fast8_t d = (op >> 4) & 15;
      uint32_t p1 = gfx_vector_p2xy(bc_get_reg(vm, d)) & 0xfffefffe;
      uint32_t p0 = gfx_vector_p2xy(bc_get_reg(vm, op & 15));
      if (op & 0x0100)
	p0 = ~p0 + 0x00020002;
      p0 &= 0xfffefffe;
      bc_set_reg(vm, d, gfx_vector_xy2p((p1 + p0) & 0xfffefffe));
      break;
    }

    case 0x6200: {		/* gfx_neg* gfx_swp* */
      uint_fast8_t d = op & 15;
      uint32_t p0 = bc_get_reg(vm, d);
      gfx_pos_t x = gfx_vector_x(p0);
      gfx_pos_t y = gfx_vector_y(p0);
      if (op & 0x0010)
	x = -x;
      if (op & 0x0020)
	y = -y;
      if (op & 0x0040)
	_GFX_SWAP(x, y);
      bc_set_reg(vm, d, gfx_vector_xy_2p(x, y));
      break;
    }

    case 0x6300: {		/* gfx_mul */
      uint_fast8_t d = op & 15;
      uint_fast8_t e = (op >> 4) & 15;
      bc_set_reg(vm, e, ((int64_t)(int32_t)bc_get_reg(vm, d)
			 * (int32_t)bc_get_reg(vm, e)) >> 5);
      break;
    }

    case 0x6400: 		/* gfx_mulxy */
    case 0x6500: 		/* gfx_addx */
    case 0x6600: 		/* gfx_addy */
    case 0x6700: {		/* gfx_divxy */
      uint_fast8_t d = (op >> 4) & 15;
      gfx_pos_t r = bc_get_reg(vm, op & 15);
      uint32_t p0 = bc_get_reg(vm, d);
      gfx_pos_t x = gfx_vector_x(p0);
      gfx_pos_t y = gfx_vector_y(p0);
      switch (op & 0x0300)
	{
	case 0x0000:
	  x = (x * r) >> 5;
	  y = (y * r) >> 5;
	  break;
	case 0x0100:
	  x += r;
	  break;
	case 0x0200:
	  y += r;
	  break;
	case 0x0300:
	  x = (x << 5) / r;
	  y = (y << 5) / r;
	  break;
	}
      bc_set_reg(vm, d, gfx_vector_xy_2p(x, y));
      break;
    }

    case 0x6800: {		/* gfx_unpackxy */
      uint_fast8_t d = (op >> 4) & 14;
      uint32_t p0 = bc_get_reg(vm, op & 15);
      uint32_t x = gfx_vector_x(p0);
      uint32_t y = gfx_vector_y(p0);
      if (op & 0x0010)
	{
	  x = (int32_t)(int16_t)x;
	  y = (int32_t)(int16_t)y;
	}
      bc_set_reg(vm, d, x);
      bc_set_reg(vm, d | 1, y);
      break;
    }

    case 0x6900: 		/* gfx_packx */
    case 0x6a00: 		/* gfx_packy */
    case 0x6b00: {		/* gfx_pack */
      uint_fast8_t d = (op >> 4) & 14;
      uint_fast8_t e = op & 15;
      uint32_t p0 = bc_get_reg(vm, e);
      uint32_t x = 0, y = 0;
      if (op & 0x0010)
	{
	  x = gfx_vector_x(p0);
	  y = gfx_vector_y(p0);
	}
      if (op & 0x0100)
	x = bc_get_reg(vm, d);
      if (op & 0x0200)
	y = bc_get_reg(vm, d | 1);
      bc_set_reg(vm, e, gfx_vector_xy_2p(x, y));
      break;
    }

    case 0x6c00: {
      uint_fast8_t d = op & 15;

      switch (op & 0x00f0)
	{
	case 0x0000:
	case 0x0010:
	case 0x0020:
	case 0x0030: {		/* gfx_size */
	  uint_fast8_t n = (op >> 4) & 3;
	  struct gfx_surface_s *s = ctx->s + n;
	  bc_set_reg(vm, d, gfx_vector_xy_2p(gfx_width(s) << 5,
					     gfx_height(s) << 5));
	  break;
	}

	case 0x0040: {		/* gfx_hypot */
	  uint32_t p0 = bc_get_reg(vm, d);
	  gfx_pos_t x = (int16_t)gfx_vector_x(p0);
	  gfx_pos_t y = (int16_t)gfx_vector_y(p0);
	  x = (x * x) >> 2;
	  y = (y * y) >> 2;
	  bc_set_reg(vm, d, gfx_sqrt32(x + y) << 1);
	  break;
	}

	case 0x0050: {		/* gfx_sqrt */
	  bc_set_reg(vm, d, gfx_sqrt32(bc_get_reg(vm, d) << 1) << 2);
	  break;
	}

	case 0x0060: {		/* gfx_sincos */
	  int32_t a = bc_get_reg(vm, d);
	  int32_t s = gfx_sin(a >> 5);
	  int32_t c = gfx_cos(a >> 5);
	  int64_t p1 = (int32_t)bc_get_reg(vm, 15);
	  c = (c * p1) >> 30;
	  s = (s * p1) >> 30;
	  bc_set_reg(vm, d, gfx_vector_xy_2p(c, s));
	  break;
	}

	case 0x0080: 		/* gfx_sin */
	case 0x0090: {		/* gfx_cos */
	  int32_t a = bc_get_reg(vm, d);
	  int32_t s = gfx_sin((a >> 5) + ((op & 0x0010) << 3));
	  int64_t p1 = (int32_t)bc_get_reg(vm, 15);
	  s = (s * p1) >> 30;
	  bc_set_reg(vm, d, s);
	  break;
	}

	}
      break;
    }

    default:
      return -ENOTSUP;
    }

  return 0;
}

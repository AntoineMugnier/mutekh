
#include <SDL/SDL.h>
#include <stdint.h>

#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "bytecode.h"
#include <gfx/gfx.h>
#include <gfx/pixel.h>
#include <gfx/bytecode.h>

#include "font_8x11.xbm"

int main(int argc, char **argv)
{
  static const uint32_t scale = 4;

  if (argc <= 1)
    {
      fprintf(stderr, "usage: %s bytecode.out\n", argv[0]);
      return -1;
    }

  int fd = open(argv[1], O_RDONLY);
  if (fd < 0)
    {
      fprintf(stderr, "bytecode file opened failed\n");
      return -1;
    }

  /* Start SDL */
  SDL_Init(SDL_INIT_EVERYTHING);

  /* Set up screen */
  SDL_Surface *screen;
  screen = SDL_SetVideoMode(64 * scale, 128 * scale, 32, SDL_SWSURFACE);
  assert(screen->format->BitsPerPixel == 32);

  int quit = 0;

  struct stat s;
  if (fstat(fd, &s) < 0)
    {
      fprintf(stderr, "bytecode file stat failed\n");
      return -1;
    }

  size_t blob_len = s.st_size;
  void *blob = mmap(NULL, blob_len, PROT_READ, MAP_SHARED, fd, 0);
  if (blob == MAP_FAILED)
    {
      fprintf(stderr, "bytecode mmap failed: %s\n", strerror(errno));
      return -1;
    }

  struct bc_descriptor_s desc;

  if (bc_load(&desc, blob, blob_len))
    {
      fprintf(stderr, "bytecode loading failed\n");
      return -1;
    }

  struct bc_context_s vm;
  bc_init(&vm, &desc);
  bc_set_trace(&vm, 1, 1);

  static struct gfx_bc_context_s ctx;
  size_t size = 0x100000;
  void *data = malloc(size);
  gfx_bc_init(&ctx);

  memcpy(data, font_8x11_bits, font_8x11_width * font_8x11_height / 8);
  bc_set_reg(&vm, 0, (uintptr_t)data);

  while (!quit)
    {
      SDL_Event event;
      if (SDL_PollEvent(&event))
	{
	  switch (event.type)
	    {
	    case SDL_KEYDOWN:
	      if (event.key.keysym.sym != SDLK_ESCAPE)
		break;
	    case SDL_QUIT:
	      quit = 1;
	    }
	}

      uint16_t op = bc_run(&vm);

      if (!BC_STATUS_CUSTOM(op))
	{
	  switch (op)
	    {
	    case BC_RUN_STATUS_FAULT:
	    case BC_RUN_STATUS_END:
	      SDL_Delay(3000 /* ms */);
	      quit = 1;
	    case BC_RUN_STATUS_BREAK:
	    case BC_RUN_STATUS_CYCLES:
	      break;
	    }
	  continue;
	}

      if (GFX_BC_IS_GFX_OP(op))
	{
	  error_t err = gfx_bc_run(&vm, &ctx, op);
	  if (err)
	    {
	      fprintf(stderr, "gfx bytecode %04x error %i, before:\n", op, err);
	      bc_dump(&vm, 1);
	      quit = 1;
	      continue;
	    }
	}

      switch (op & 0x7c00)
        {
        case 0x0800: {            /* display */
          uint_fast8_t n = (op >> 4) & 3;

          struct gfx_surface_s *s = ctx.s + n;

          SDL_LockSurface(screen);
          uint32_t *px = (uint32_t*)screen->pixels;
          uint_fast32_t stride = screen->pitch / screen->format->BytesPerPixel;

          uint_fast32_t yl = 0, yh = screen->h / scale;
          uint_fast32_t xl = 0, xh = screen->w / scale;

          if (op & 0x0040)
            {
              uint32_t p0 = bc_get_reg(&vm, op & 15);
              yl = gfx_vector_yint(p0);
              xl = gfx_vector_xint(p0);

              uint32_t p1 = bc_get_reg(&vm, 15);
              yh = yl + gfx_vector_yint(p1) * scale;
              if (yh > (uint_fast16_t)screen->h)
                yh = screen->h;

              xh = xl + gfx_vector_xint(p1) * scale;
              if (xh > (uint_fast16_t)screen->w)
                xh = screen->w;
            }

          for (uint_fast16_t y = yl * scale; y < yh * scale; y++)
            {
              for (uint_fast16_t x = xl * scale; x < xh * scale; x++)
                {
                  uint32_t *pixel = px + stride * y + x;
                  gfx_pixel_t p = gfx_get_pixel_safe(s,
                                                     x / scale - xl / scale,
                                                     y / scale - yl / scale);
                  uint32_t color = 0;
                  uint8_t r, g, b;

                  switch (CONFIG_GFX_DEFAULT_L2BPP)
                    {
                    case 0 ... 3:
                      color = (0x100 * p) >> (1 << gfx_fmt_desc[s->fmt].l2bpp);
                      color *= 0x10101;
                      break;
                    case 4:
                      /* Fixme */
                      color = p;
                      break;
                    case 5:
                      color = p;
                      break;
                    }
                  *pixel = color;
                }
            }
          SDL_UnlockSurface(screen);
          SDL_Flip(screen);
          SDL_Delay(30 /* ms */);
          break;
        }

        }
    }

  SDL_Quit();

  return 0;
}


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
  switch (CONFIG_GFX_DEFAULT_L2BPP)
    {
    case 0 ... 3:
      screen = SDL_SetVideoMode(512, 256, 8, SDL_SWSURFACE);
      assert(screen->format->BitsPerPixel == 8);
      break;
    case 4:
      screen = SDL_SetVideoMode(512, 256, 16, SDL_SWSURFACE);
      assert(screen->format->BitsPerPixel == 16);
      break;
    case 5:
      screen = SDL_SetVideoMode(512, 256, 32, SDL_SWSURFACE);
      assert(screen->format->BitsPerPixel == 32);
      break;
    }

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
          uint8_t *pdst = (uint8_t*)screen->pixels;
          uint_fast16_t bytes = screen->format->BytesPerPixel;

          uint_fast32_t ys = 0, ye = screen->h;
          uint_fast32_t xs = 0, xe = screen->w;

          if (op & 0x0040)
            {
              uint32_t p0 = bc_get_reg(&vm, op & 15);
              ys = gfx_vector_yint(p0);
              xs = gfx_vector_xint(p0);

              uint32_t p1 = bc_get_reg(&vm, 15);
              ye = ys + gfx_vector_yint(p1);
              if (ye > (uint_fast16_t)screen->h)
                ye = screen->h;

              xs = xs + gfx_vector_xint(p1);
              if (xe > (uint_fast16_t)screen->w)
                xe = screen->w;
            }

          for (uint_fast16_t y = ys; y < ye; y++)
            {
              for (uint_fast16_t x = xs; x < xe; x++)
                {
                  gfx_pixel_t p = gfx_get_pixel_safe(s, x - xs, y - ys);

                  switch (CONFIG_GFX_DEFAULT_L2BPP)
                    {
                    case 0 ... 3: {
                      uint32_t h = 1 << gfx_fmt_desc[s->fmt].l2bpp;
                      *pdst = (0x100 * p) >> h;
                      break;
                    }
                    case 4:
                      *(uint16_t*)pdst = p;
                      break;
                    case 5:
                      *(uint32_t*)pdst = p;
                      break;
                    }
                  pdst += bytes;
                }
              pdst += screen->pitch - screen->w * bytes;
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


/*
 * This file is part of MutekH.
 * 
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (c) UPMC, Lip6, SoC
 *         Nicolas Pouillon <nipo@ssji.net>, 2009-2010
 */

#define GCT_CONFIG_NODEPRECATED

#include <hexo/endian.h>

#include <stdio.h>
#include <mutek/printk.h>
#include <libc/formatter.h>

GCT_CONTAINER_FCNS(printk_backend, static inline, printk_backend,
                   remove, push);

static printk_backend_root_t printk_backends =
  GCT_CONTAINER_ROOT_INITIALIZER(printk_backend);

void printk_register(struct printk_backend_s *s, printk_handler_t *handler)
{
  s->level = CONFIG_MUTEK_PRINTK_RUNTIME_LEVEL;
  s->id = 0;
  s->handler = handler;
  printk_backend_push(&printk_backends, s);
}

void printk_unregister(struct printk_backend_s *s)
{
  printk_backend_remove(&printk_backends, s);
}

#ifdef CONFIG_COMPILE_INSTRUMENT
bool_t mutek_instrument_trace(bool_t state);
#endif

#ifdef CONFIG_MUTEK_PRINTK_LOCK
static lock_t printk_lock = LOCK_INITIALIZER;
#endif

struct logk_filter_s
{
  uint32_t id;
  int8_t level;
};

static PRINTF_OUTPUT_FUNC(logk_output)
{
  const struct logk_filter_s *f = ctx;

  GCT_FOREACH(printk_backend, &printk_backends, s, {
      __unused__ uint32_t backend_id = s->id;
      __unused__ int8_t backend_level = s->level;
      __unused__ uint32_t id = f->id;
      __unused__ int8_t level = f->level;

      if (CONFIG_MUTEK_PRINTK_RUNTIME_EXPR)
        s->handler(s, str, len);
  });
}

inline ssize_t vlogk_(const char *format, va_list ap)
{
#ifdef CONFIG_COMPILE_INSTRUMENT
  bool_t old = mutek_instrument_trace(0);
#endif

  struct logk_filter_s f = {
    .id = endian_be32_na_load(format + 2 + sizeof(LOGK_COLOR(XX)) - 1),
    .level = format[0]
  };

  error_t err = formatter_printf(&f, logk_output, format + 1, ap);

#ifdef CONFIG_COMPILE_INSTRUMENT
  mutek_instrument_trace(old);
#endif
  return err;
}

ssize_t logk_(const char *format, ...)
{
  ssize_t	res;
  va_list	ap;

  va_start(ap, format);
  res = vlogk_(format, ap);
  va_end(ap);

  return res;
}

void logk_set_filter(struct printk_backend_s *backend,
                     const char id[4], enum logk_level_e level)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  GCT_FOREACH(printk_backend, &printk_backends, s, {
      if (backend == NULL || backend == s)
        {
          s->id = id != NULL ? endian_be32_na_load(id) : 0;
          s->level = level;
        }
  });
  CPU_INTERRUPT_RESTORESTATE;
}

static PRINTF_OUTPUT_FUNC(printk_output)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  GCT_FOREACH(printk_backend, &printk_backends, s, {
      s->handler(s, str, len);
  });
  CPU_INTERRUPT_RESTORESTATE;
}

inline ssize_t vprintk(const char *format, va_list ap)
{
#ifdef CONFIG_COMPILE_INSTRUMENT
  bool_t old = mutek_instrument_trace(0);
#endif
  error_t err = formatter_printf(NULL, printk_output, format, ap);

#ifdef CONFIG_COMPILE_INSTRUMENT
  mutek_instrument_trace(old);
#endif
  return err;
}


ssize_t printk(const char *format, ...)
{
  ssize_t	res;
  va_list	ap;

  va_start(ap, format);
  res = vprintk(format, ap);
  va_end(ap);

  return res;
}


void writek(const char *data, size_t len)
{
#ifdef CONFIG_MUTEK_PRINTK_LOCK
    LOCK_SPIN_IRQ(&printk_lock);
#endif
    GCT_FOREACH(printk_backend, &printk_backends, s, {
        s->handler(s, data, len);
    });
#ifdef CONFIG_MUTEK_PRINTK_LOCK
    LOCK_RELEASE_IRQ(&printk_lock);
#endif
}

#ifdef CONFIG_MUTEK_PRINTK_HEXDUMP

void hexdumpk(uintptr_t address, const void *data, size_t len)
{
#ifdef CONFIG_MUTEK_PRINTK_LOCK
  LOCK_SPIN_IRQ(&printk_lock);
#endif
  formatter_hexdump(NULL, printk_output, address, data, len);
#ifdef CONFIG_MUTEK_PRINTK_LOCK
  LOCK_RELEASE_IRQ(&printk_lock);
#endif
}

#endif

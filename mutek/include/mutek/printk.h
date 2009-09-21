#ifndef MUTEK_PRINTK_H_
#define MUTEK_PRINTK_H_

#if defined(CONFIG_MUTEK_PRINTK)

#include <stdarg.h>
#include <mutek/printf_arg.h>

void printk_set_output(printf_output_func_t *f, void *ctx);
ssize_t printk(const char *format, ...);
inline ssize_t vprintk(const char *format, va_list ap);

#else /* no printk */

static inline
void printk_set_output(printf_output_func_t *f, void *ctx)
{}

static inline
ssize_t printk(const char *format, ...)
{
	return 0;
}

static inline
inline ssize_t vprintk(const char *format, va_list ap)
{
	return 0;
}

#endif /* printk */

#endif


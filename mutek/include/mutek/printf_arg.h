
#ifndef MUTEK_PRINTF_ARG_H_
#define MUTEK_PRINTF_ARG_H_

#include <stdarg.h>
#include <hexo/types.h>

#define PRINTF_OUTPUT_FUNC(x) void (x)(void *ctx, const char *str, size_t offset, size_t len)

typedef PRINTF_OUTPUT_FUNC(printf_output_func_t);

ssize_t
mutek_printf_arg(void *ctx, printf_output_func_t * const fcn,
				 const char *format, va_list ap);

void
mutek_hexdump_arg(void *ctx, printf_output_func_t * const fcn,
                  uintptr_t address, const void *base, size_t len);

#endif

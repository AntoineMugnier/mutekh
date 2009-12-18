
#include <stdio.h>
#include <mutek/printk.h>

static printf_output_func_t *printk_output = NULL;
static void *printk_output_arg;

#ifdef CONFIG_COMPILE_INSTRUMENT
bool_t mutek_instrument_trace(bool_t state);
#endif

void printk_set_output(printf_output_func_t *f, void *priv)
{
	printk_output = f;
	printk_output_arg = priv;
}

inline ssize_t vprintk(const char *format, va_list ap)
{
#ifdef CONFIG_COMPILE_INSTRUMENT
	bool_t old = mutek_instrument_trace(0);
#endif
	error_t err = EIO;

#ifdef CONFIG_MUTEK_CONSOLE
	if ( printk_output )
		err = mutek_printf_arg(printk_output_arg, printk_output, format, ap);
#endif

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

void hexdumpk(uintptr_t address, const void *data, size_t len)
{
    mutek_hexdump_arg(printk_output_arg, printk_output, address, data, len);
}


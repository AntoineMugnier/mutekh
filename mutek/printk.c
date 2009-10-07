
#include <stdio.h>
#include <mutek/printk.h>

static printf_output_func_t *printk_output = NULL;
static void *printk_output_arg;

void printk_set_output(printf_output_func_t *f, void *priv)
{
	printk_output = f;
	printk_output_arg = priv;
}

inline ssize_t vprintk(const char *format, va_list ap)
{
#ifdef CONFIG_MUTEK_CONSOLE
	if ( printk_output )
		return mutek_printf_arg(printk_output_arg, printk_output, format, ap);
#endif
	return EIO;
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


#include <hexo/types.h>
#include <hexo/cpu.h>
#include <mutek/startup.h>

#include <assert.h>

#if defined(CONFIG_LIBC_ASSERT)

#ifdef CONFIG_MUTEK_PRINTK
ssize_t printk(const char *format, ...);
#endif

static bool_t already_failed = 0;

# if defined(CONFIG_LIBC_ASSERT_SIMPLE) || !defined(CONFIG_MUTEK_PRINTK)
void __assert_fail(void)
# else
void __assert_fail(const char *file, uint_fast16_t line, const char *expr)
# endif
{
  if (!already_failed)
    {
      already_failed = 1;
# if defined(CONFIG_MUTEK_PRINTK)
#  if defined(CONFIG_LIBC_ASSERT_SIMPLE)
      printk("Assertion failed at pc=%p\n", __builtin_return_address(0));
#  else
      printk("Assertion failed at %s:%u: (%s) is false\n", file, line, expr);
#  endif
# endif
    }

  void CONFIG_MUTEK_FAULT_FINISH(void);
  while (1)
    CONFIG_MUTEK_FAULT_FINISH();
}

#endif

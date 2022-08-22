
/**
  @file
  @module {Core::Kernel services}
  @short Debugging messages output API
*/

#ifndef MUTEK_PRINTK_H_
#define MUTEK_PRINTK_H_

#include <stdarg.h>
#include <libc/formatter.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>
#include <gct/container_slist.h>

#define GCT_CONTAINER_ALGO_printk_backend SLIST
#define GCT_CONTAINER_LOCK_printk_backend NOLOCK

#ifndef LOGK_MODULE_ID
/** A four characters @ref logk module id defined for the current compilation unit. */
# define LOGK_MODULE_ID "none"
#endif

/** @This specifies the logk level.
    @see logk_set_filter */
enum logk_level_e
{
  LOGK_LEVEL_TRACE =   0x10,
  LOGK_LEVEL_DEBUG =   0x20,
  LOGK_LEVEL_NORMAL=   0x30,
  LOGK_LEVEL_WARNING = 0x40,
  LOGK_LEVEL_ERROR =   0x50
};

/** @hidden static assert for logk module id len */
typedef char printk_module_len_isnt_4_t[-(sizeof(LOGK_MODULE_ID) != 5)];

struct printk_backend_s;

/** @see printk_handler_t */
#define PRINTK_HANDLER(n) void (n)(struct printk_backend_s *backend, \
                                   const char *str, size_t len)
/** @This is the printk backend output handler.
    It may be called from interrupt context. */
typedef PRINTK_HANDLER(printk_handler_t);

/** @internalmembers printk backend entry */
struct printk_backend_s
{
  printk_handler_t *handler;
  GCT_CONTAINER_ENTRY(printk_backend, list_entry);
  uint32_t id;
  int8_t level;
};

GCT_CONTAINER_TYPES      (printk_backend, struct printk_backend_s *, list_entry);

#ifdef CONFIG_MUTEK_PRINTK_FATAL_ABORT
# define printk_fatal_abort() do {                      \
    logk_error("printk_fatal_abort reached, aborting"); \
    abort();                                            \
  } while (0)
#else
# define printk_fatal_abort() do {} while (0)
#endif

/** @This registers a printk backend */
config_depend(CONFIG_MUTEK_PRINTK)
void printk_register(struct printk_backend_s *s, printk_handler_t *handler);

/** @This unregisters a printk backend */
config_depend(CONFIG_MUTEK_PRINTK)
void printk_unregister(struct printk_backend_s *s);

/** @This set the logk runtime filtering.
    Filtering by module is disabled if the @tt id pointer is @tt NULL.
    @see #CONFIG_MUTEK_PRINTK_RUNTIME_LEVEL
    @see #CONFIG_MUTEK_PRINTK_RUNTIME_EXPR */
config_depend(CONFIG_MUTEK_PRINTK)
void logk_set_filter(struct printk_backend_s *backend,
                     const char id[4], enum logk_level_e level);

/** @multiple @This outputs a formatted string to the backends.

    Output is not filtered according to the log level.  Use the @ref
    #logk family of functions instead when the intent is to output
    filtered log entries.

    @see formatter_printf */
config_depend(CONFIG_MUTEK_PRINTK)
ssize_t printk(const char *format, ...);

/** @see printk */
config_depend(CONFIG_MUTEK_PRINTK)
ssize_t vprintk(const char *format, va_list ap);

/**
   @This prints a binary memory dump of memory to the
   backends. Output is terminal-protected (all characters between
   ascii(0) and ascii(31) are replaced by "." in output).

   Address appearing on the side of the dump can be defined.

   Output is not filtered according to the log level.
*/
config_depend(CONFIG_MUTEK_PRINTK_HEXDUMP)
void hexdumpk(uintptr_t address, const void *data, size_t len);

/** @This write raw data to the current backends.
    Output is not filtered according to the log level. */
config_depend(CONFIG_MUTEK_PRINTK)
void writek(const char *data, size_t len);

/** @internal @see #vlogk */
config_depend(CONFIG_MUTEK_PRINTK)
ssize_t vlogk_(const char *format, va_list ap);

/** @internal @see #logk */
config_depend(CONFIG_MUTEK_PRINTK)
ssize_t logk_(const char *format, ...);

#ifdef CONFIG_MUTEK_PRINTK_COLOR
/** @internal */
# define LOGK_COLOR(x) "\x1b[" #x "m"
#else
# define LOGK_COLOR(x) ""
#endif

/** @internal */
#define logk_level_(log_func, level_str, format, ...)                   \
  do {                                                                  \
    bool_t __do_log = 0;                                                \
    {                                                                   \
       __unused__ uint8_t level = level_str[0];                         \
       __unused__ uint32_t id   = ((LOGK_MODULE_ID[0] << 24) |          \
                                   (LOGK_MODULE_ID[1] << 16) |          \
                                   (LOGK_MODULE_ID[2] << 8)  |          \
                                    LOGK_MODULE_ID[3]);                 \
       __do_log = !!(CONFIG_MUTEK_PRINTK_COMPILE_EXPR);                 \
    }                                                                   \
    if (__do_log)                                                       \
      log_func(level_str format "\n",                                   \
               ## __VA_ARGS__);                                         \
  } while (0)

#ifdef CONFIG_MUTEK_PRINTK
/** @multiple @This sends a line to the printk backends with the @ref
    LOGK_LEVEL_TRACE level */
# define vlogk_trace(format, ap)   logk_level_(vlogk_, "\x10[" LOGK_COLOR(34) LOGK_MODULE_ID LOGK_COLOR() "] ", format, ap)
# define logk_trace(format...)   logk_level_(logk_, "\x10[" LOGK_COLOR(34) LOGK_MODULE_ID LOGK_COLOR() "] ", format)
/** @multiple @This sends a line to the printk backends with the @ref
    LOGK_LEVEL_DEBUG level */
# define vlogk_debug(format, ap)   logk_level_(vlogk_, "\x20[" LOGK_COLOR(36) LOGK_MODULE_ID LOGK_COLOR() "] ", format, ap)
# define logk_debug(format...)   logk_level_(logk_, "\x20[" LOGK_COLOR(36) LOGK_MODULE_ID LOGK_COLOR() "] ", format)
/** @multiple @This sends a line to the printk backends with the @ref
    LOGK_LEVEL_NORMAL level */
# define vlogk(format, ap)         logk_level_(vlogk_, "\x30[" LOGK_COLOR(97) LOGK_MODULE_ID LOGK_COLOR() "] ", format, ap)
# define logk(format...)         logk_level_(logk_, "\x30[" LOGK_COLOR(97) LOGK_MODULE_ID LOGK_COLOR() "] ", format)
/** @multiple @This sends a line to the printk backends with the @ref
    LOGK_LEVEL_WARNING level */
# define vlogk_warning(format, ap) logk_level_(vlogk_, "\x40[" LOGK_COLOR(93) LOGK_MODULE_ID LOGK_COLOR() "] ", format, ap)
# define logk_warning(format...) logk_level_(logk_, "\x40[" LOGK_COLOR(93) LOGK_MODULE_ID LOGK_COLOR() "] ", format)
/** @multiple @This sends a line to the printk backends with the @ref
    LOGK_LEVEL_ERROR level */
# define vlogk_error(format, ap)   logk_level_(vlogk_, "\x50[" LOGK_COLOR(91) LOGK_MODULE_ID LOGK_COLOR() "] ", format, ap)
# define logk_error(format...)   logk_level_(logk_, "\x50[" LOGK_COLOR(91) LOGK_MODULE_ID LOGK_COLOR() "] ", format)

#else

/** @multiple @hidden
   silently ignore calls to these functions when printk is disabled. */
# define vlogk_trace(format, ap)
# define logk_trace(format...)
# define vlogk_debug(format, ap)
# define logk_debug(format...)
# define vlogk(format, ap)
# define logk(format...)
# define vlogk_warning(format, ap)
# define logk_warning(format...)
# define vlogk_error(format, ap)
# define logk_error(format...)
# define printk(...)
# define vprintk(...)
# define hexdumpk(...)
# define writek(...)

#endif

/** @multiple @This sends a line to the printk backends with the @ref
    LOGK_LEVEL_ERROR level, then aborts if #CONFIG_MUTEK_PRINTK_FATAL_ABORT
    is set */
# define vlogk_fatal(format, ap)   do {         \
    vlogk_error(format, ap);                    \
    printk_fatal_abort();                       \
  } while(0)
# define logk_fatal(format...)   do {           \
    logk_error(format);                         \
    printk_fatal_abort();                       \
  } while(0)

#define PRINTK_RET(val, ...)			\
do {						\
	printk(__VA_ARGS__);			\
	return (val);				\
} while (0)

#endif


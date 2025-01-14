/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef __ASSERT_H_
#define __ASSERT_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

/**
 * @file
 * @module {Core::C library}
 */

#include <hexo/types.h>
#include <hexo/cpu.h>

#ifdef NDEBUG
# warning NDEBUG is deprecated here, use CONFIG_LIBC_ASSERT or CONFIG_DEBUG
#endif

#ifndef LOGK_MODULE_ID
/** A four characters @ref logk module id defined for the current compilation unit. */
# define LOGK_MODULE_ID "none"
#endif

# if defined(CONFIG_LIBC_ASSERT)

#  ifndef MUTEK_CFILE
#   define MUTEK_CFILE ""
#  endif

/** @internal */
#  define __assert_filter() ({                                          \
  __unused__ uint32_t id = ((LOGK_MODULE_ID[0] << 24) |                 \
                            (LOGK_MODULE_ID[1] << 16) |                 \
                            (LOGK_MODULE_ID[2] << 8)  |                 \
                            LOGK_MODULE_ID[3]);                         \
  !(CONFIG_LIBC_ASSERT_FILTER_EXPR); })

#  if defined(CONFIG_LIBC_ASSERT_SIMPLE) || !defined(CONFIG_MUTEK_PRINTK)
__attribute__((noreturn))
void __assert_fail(void);
#   define __assert(expr, str) ((void) ((expr) ? 0 : __assert_fail()))
#  else
__attribute__((noreturn))
void __assert_fail(const char *file, uint_fast16_t line, const char *expr);
#   define __assert(expr, str) ((void) ((expr) ? 0 : __assert_fail(MUTEK_CFILE, __LINE__, str)))
#  endif
/** @this is the standard @tt assert macro */
#  define assert(expr) __assert(__assert_filter() || (expr), #expr)
#  define IFASSERT(...) __VA_ARGS__
/** @this macro does the same as the @ref #assert macro, but
    still execute @tt expr when @ref #CONFIG_LIBC_ASSERT is
    disabled */
#  define ensure(expr) __assert((expr) || __assert_filter(), #expr)

/** @this macro can be used when the control flow can not reach a
    point. When @ref CONFIG_LIBC_ASSERT is defined, reaching this
    point is treated as an assertion failure. In the other case, @tt
    __builtin_unreachable is used. */
#  define UNREACHABLE()  do {                                           \
    if (__assert_filter())                                              \
      __builtin_unreachable();                                          \
    else                                                                \
      __assert(0, "UNREACHABLE()");                                     \
  } while (1)

# else
#  define assert(expr) ((void) 0)
#  define IFASSERT(...)
#  define ensure(expr) ((void) (expr))
#  define UNREACHABLE()  __builtin_unreachable()
# endif

C_HEADER_END

#endif


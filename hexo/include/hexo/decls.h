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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2010

*/

#ifndef HEXO_DECLS_H_
#define HEXO_DECLS_H_

/**
 * @file
 * @module{Hexo}
 * @short Various build system related defs
 * @internal
 */

/* warp C header in C++ */
# define _GNUC_VERSION      (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)

# if __cplusplus &&! defined(__MUTEK_ASM__)
#  define C_HEADER_BEGIN extern "C" {
#  define C_HEADER_END }
# else
#  define C_HEADER_BEGIN
#  define C_HEADER_END
# endif

/* make unavailable functions deprecated */

# define __unused__ __attribute__((unused))

# if _GNUC_VERSION >= 40500
#  define deprecated(message)   __attribute__((deprecated(message)))
# else
#  define deprecated(message)   __attribute__((deprecated))
# endif

# ifndef __MUTEK_ASM__ // mkdoc:skip

#  define _CONFIG_DEPEND_1(name, attr, proto, ...) \
  attr proto __VA_ARGS__

#   define _CONFIG_DEPEND_0(name, attr, proto, ...) \
  deprecated("this symbol depends on " name ", not defined in configuration") proto

#  define _CONFIG_DEPEND_AND_00(name, attr, proto, ...) _CONFIG_DEPEND_0(name, attr, proto, __VA_ARGS__)
#  define _CONFIG_DEPEND_AND_01(name, attr, proto, ...) _CONFIG_DEPEND_0(name, attr, proto, __VA_ARGS__)
#  define _CONFIG_DEPEND_AND_10(name, attr, proto, ...) _CONFIG_DEPEND_0(name, attr, proto, __VA_ARGS__)
#  define _CONFIG_DEPEND_AND_11(name, attr, proto, ...) _CONFIG_DEPEND_1(name, attr, proto, __VA_ARGS__)

#  define _CONFIG_DEPEND_OR_00(name, attr, proto, ...) _CONFIG_DEPEND_0(name, attr, proto, __VA_ARGS__)
#  define _CONFIG_DEPEND_OR_01(name, attr, proto, ...) _CONFIG_DEPEND_1(name, attr, proto, __VA_ARGS__)
#  define _CONFIG_DEPEND_OR_10(name, attr, proto, ...) _CONFIG_DEPEND_1(name, attr, proto, __VA_ARGS__)
#  define _CONFIG_DEPEND_OR_11(name, attr, proto, ...) _CONFIG_DEPEND_1(name, attr, proto, __VA_ARGS__)

#  define _CONFIG_DEPEND_PASTE2(a, b) a ## b
#  define _CONFIG_DEPEND_PASTE3(a, b, c) a ## b ## c

#  define _CONFIG_DEPEND(a, b, attr, proto, ...) \
  _CONFIG_DEPEND_PASTE2(_CONFIG_DEPEND_, b)(a, attr, proto, __VA_ARGS__)
#  define _CONFIG_DEPEND_AND2(a1, a2, b1, b2, attr, proto, ...)            \
  _CONFIG_DEPEND_PASTE3(_CONFIG_DEPEND_AND_, b1, b2)(a1 " and " a2, attr, proto, __VA_ARGS__)
#  define _CONFIG_DEPEND_OR2(a1, a2, b1, b2, attr, proto, ...)            \
  _CONFIG_DEPEND_PASTE3(_CONFIG_DEPEND_OR_, b1, b2)(a1 " or " a2, attr, proto, __VA_ARGS__)

#  define config_depend(token) \
  _CONFIG_DEPEND(#token, _##token, , , )
#  define config_depend_inline(token, proto, ...) \
  _CONFIG_DEPEND(#token, _##token, static inline, proto, __VA_ARGS__)

#  define config_depend_and2(token1, token2) \
  _CONFIG_DEPEND_AND2(#token1, #token2, _##token1, _##token2, , , )
#  define config_depend_and2_inline(token1, token2, proto, ...) \
  _CONFIG_DEPEND_AND2(#token1, #token2, _##token1, _##token2, static inline, proto, __VA_ARGS__)

#  define config_depend_or2(token1, token2) \
  _CONFIG_DEPEND_OR2(#token1, #token2, _##token1, _##token2, , , )
#  define config_depend_or2_inline(token1, token2, proto, ...) \
  _CONFIG_DEPEND_OR2(#token1, #token2, _##token1, _##token2, static inline, proto, __VA_ARGS__)

# endif

#ifdef __MKDOC__
# define config_depend(token)
# define config_depend_inline(token, proto, ...) static inline proto __VA_ARGS__
# define config_depend_and2(token1, token2)
# define config_depend_and2_inline(token1, token2, proto, ...) static inline proto __VA_ARGS__
# define config_depend_or2(token1, token2)
# define config_depend_or2_inline(token1, token2, proto, ...) static inline proto __VA_ARGS__
#endif

#endif /* HEXO_DECLS_H_ */

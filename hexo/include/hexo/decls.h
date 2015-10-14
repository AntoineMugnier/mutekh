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

# if __cplusplus
#  define C_HEADER_BEGIN extern "C" {
#  define C_HEADER_END }
# else
#  define C_HEADER_BEGIN
#  define C_HEADER_END
# endif

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))

/* make unavailable functions deprecated */

# define __unused__ __attribute__((unused))

# if _GNUC_VERSION >= 40500
#  define DEPRECATED(message)   __attribute__((deprecated(message)))
# else
#  define DEPRECATED(message)   __attribute__((deprecated))
# endif

# ifdef CONFIG_RELEASE
#  define UNREACHABLE()  __builtin_unreachable()
# else
#  define UNREACHABLE()  abort()
# endif

# define ALWAYS_INLINE inline __attribute__((always_inline))

# if 1 // mkdoc:skip

#  define _CONFIG_DEPEND_1(name, attr, proto, ...) \
  attr proto __VA_ARGS__

#   define _CONFIG_DEPEND_0(name, attr, proto, ...) \
  DEPRECATED("this symbol depends on " name ", not defined in configuration") proto

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
#  define config_depend_alwaysinline(token, proto, ...) \
  _CONFIG_DEPEND(#token, _##token, ALWAYS_INLINE, proto, __VA_ARGS__) ;
#  define config_depend_inline(token, proto, ...) \
  _CONFIG_DEPEND(#token, _##token, inline, proto, __VA_ARGS__) ;

#  define config_depend_and2(token1, token2) \
  _CONFIG_DEPEND_AND2(#token1, #token2, _##token1, _##token2, , , )
#  define config_depend_and2_alwaysinline(token1, token2, proto, ...) \
  _CONFIG_DEPEND_AND2(#token1, #token2, _##token1, _##token2, ALWAYS_INLINE, proto, __VA_ARGS__) ;
#  define config_depend_and2_inline(token1, token2, proto, ...) \
  _CONFIG_DEPEND_AND2(#token1, #token2, _##token1, _##token2, inline, proto, __VA_ARGS__) ;

#  define config_depend_or2(token1, token2) \
  _CONFIG_DEPEND_OR2(#token1, #token2, _##token1, _##token2, , , )
#  define config_depend_or2_alwaysinline(token1, token2, proto, ...) \
  _CONFIG_DEPEND_OR2(#token1, #token2, _##token1, _##token2, ALWAYS_INLINE, proto, __VA_ARGS__) ;
#  define config_depend_or2_inline(token1, token2, proto, ...) \
  _CONFIG_DEPEND_OR2(#token1, #token2, _##token1, _##token2, inline, proto, __VA_ARGS__) ;

#ifndef STATIC_ASSERT
#define STATIC_ASSERT(error, expr)                              \
  extern char assertion_failure__##error[-(char)!(expr)];
#endif

#ifndef FIRST_FIELD_ASSERT
#define FIRST_FIELD_ASSERT(struct_name, field)                        \
  STATIC_ASSERT(field##_must_be_the_first_field_in_struct,\
                __builtin_offsetof(struct struct_name, field) == 0)
#endif

#define STRUCT_INHERIT(type_s, base_s, field)                           \
                                                                        \
ALWAYS_INLINE struct base_s *                                           \
type_s##_base(struct type_s *x)                                         \
{                                                                       \
  if (__builtin_offsetof(struct type_s, field) && x == NULL)            \
    return NULL;                                                        \
  return &x->field;                                                     \
}                                                                       \
                                                                        \
ALWAYS_INLINE struct type_s *                                           \
type_s##_cast(struct base_s *x)                                         \
{                                                                       \
  if (__builtin_offsetof(struct type_s, field) && x == NULL)            \
    return NULL;                                                        \
  return (void*)((uint8_t*)x - __builtin_offsetof(struct type_s, field)); \
}

#define STRUCT_COMPOSE(cont_s, field)                                   \
                                                                        \
ALWAYS_INLINE struct cont_s *                                           \
cont_s##_from_##field(typeof(((struct cont_s*)0)->field) *x)            \
{                                                                       \
  if (__builtin_offsetof(struct cont_s, field) && x == NULL)            \
    return NULL;                                                        \
  return (void*)((uint8_t*)x - __builtin_offsetof(struct cont_s, field)); \
}


#undef ENUM_DESCRIPTOR
#define ENUM_DESCRIPTOR(name, ...) extern const char name[];

#endif /* 1 */

#ifdef __MKDOC__
# define config_depend(token)
# define config_depend_inline(token, proto, ...) inline proto __VA_ARGS__
# define config_depend_alwaysinline(token, proto, ...) ALWAYS_INLINE proto __VA_ARGS__
# define config_depend_and2(token1, token2)
# define config_depend_and2_inline(token1, token2, proto, ...) inline proto __VA_ARGS__
# define config_depend_and2_alwaysinline(token1, token2, proto, ...) ALWAYS_INLINE proto __VA_ARGS__
# define config_depend_or2(token1, token2)
# define config_depend_or2_inline(token1, token2, proto, ...) inline proto __VA_ARGS__
# define config_depend_or2_alwaysinline(token1, token2, proto, ...) ALWAYS_INLINE proto __VA_ARGS__

#define STRUCT_INHERIT(type_s, base_s, field)                           \
ALWAYS_INLINE struct base_s *                                           \
type_s##_base(struct type_s *x);                                        \
ALWAYS_INLINE struct type_s *                                           \
type_s##_cast(struct base_s *x);

#define STRUCT_COMPOSE(cont_s, field)                                   \
ALWAYS_INLINE struct cont_s *                                           \
cont_s##_from_##field(void *x);

# define STATIC_ASSERT(error, expr)
# define FIRST_FIELD_ASSERT(struct_name, field)
# define ENUM_DESCRIPTOR(...)
#endif

#ifdef CONFIG_COMPILE_NOBITFIELD
# define BITFIELD(name, bits) name
#else
# define BITFIELD(name, bits) name:bits
#endif

#if defined(CONFIG_DEBUG) && !defined(__ASSEMBLER__)

ALWAYS_INLINE void hexo_atomic_scope_check(char *scope_exited_cleanly)
{
  extern void abort(void);

  if (!*scope_exited_cleanly)
    abort();
}

# define HEXO_ATOMIC_SCOPE_BEGIN                                \
  {                                                             \
  __attribute__((cleanup(hexo_atomic_scope_check)))             \
  char __scope_exited_cleanly = 0;

# define HEXO_ATOMIC_SCOPE_END                                \
  __scope_exited_cleanly = 1;                                 \
  }

#else

# define HEXO_ATOMIC_SCOPE_BEGIN {
# define HEXO_ATOMIC_SCOPE_END }

#endif

#endif /* HEXO_DECLS_H_ */

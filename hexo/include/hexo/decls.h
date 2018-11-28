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
 * @module {Core::Hardware abstraction layer}
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
  __attribute__((unused)) extern char assertion_failure__##error[-(char)!(expr)];
#endif

#ifndef FIRST_FIELD_ASSERT
#define FIRST_FIELD_ASSERT(struct_name, field)                        \
  STATIC_ASSERT(field##_must_be_the_first_field_in_struct,\
                __builtin_offsetof(struct struct_name, field) == 0)
#endif

#define STRUCT_DESCRIPTOR(struct_name, ...) \
  __attribute__((weak))                     \
  const char struct_name##_desc = 0 

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
ALWAYS_INLINE const struct base_s *                                     \
const_##type_s##_base(const struct type_s *x)                           \
{                                                                       \
  if (__builtin_offsetof(const struct type_s, field) && x == NULL)      \
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
}                                                                       \
                                                                        \
ALWAYS_INLINE const struct type_s *                                     \
const_##type_s##_cast(const struct base_s *x)                           \
{                                                                       \
  if (__builtin_offsetof(const struct type_s, field) && x == NULL)      \
    return NULL;                                                        \
  return (const void*)((const uint8_t*)x - __builtin_offsetof(const struct type_s, field)); \
}

#define STRUCT_COMPOSE(cont_s, field)                                   \
                                                                        \
ALWAYS_INLINE struct cont_s *                                           \
cont_s##_from_##field(typeof(((struct cont_s*)0)->field) *x)            \
{                                                                       \
  if (__builtin_offsetof(struct cont_s, field) && x == NULL)            \
    return NULL;                                                        \
  return (void*)((uint8_t*)x - __builtin_offsetof(struct cont_s, field)); \
}                                                                       \
                                                                        \
ALWAYS_INLINE const struct cont_s *                                     \
const_##cont_s##_from_##field(typeof(((const struct cont_s*)0)->field) *x) \
{                                                                       \
  if (__builtin_offsetof(const struct cont_s, field) && x == NULL)      \
    return NULL;                                                        \
  return (const void*)((const uint8_t*)x - __builtin_offsetof(const struct cont_s, field)); \
}

#define FIELD_ALIAS(type_s, field_s, type_a, field_a)                   \
  struct {                                                              \
    uint8_t field_a##_padding_[__builtin_offsetof(type_s, field_s)];    \
    type_a field_a;                                                     \
    uint8_t _alias_is_too_large[-(sizeof(((type_s*)0)->field_s)         \
                                  < sizeof(type_a))];                   \
  }

#define FIELD_USING(type_s, field_s)                                    \
  struct {                                                              \
    uint8_t field_s##_padding_[__builtin_offsetof(type_s, field_s)];    \
    typeof(((type_s*)0)->field_s) field_s;                              \
  }

#endif /* 1 */

#ifdef __MKDOC__
# define config_depend(token)                           \
  /*+ This is available when @ref # ## token is defined. */

# define config_depend_inline(token, proto, ...)        \
  config_depend(token)                                  \
  inline proto __VA_ARGS__

# define config_depend_alwaysinline(token, proto, ...)  \
  config_depend(token)                                  \
  ALWAYS_INLINE proto __VA_ARGS__

# define config_depend_and2(token1, token2)             \
  /*+ This is available when @ref # ## token1 and @ref # ## token2 are both defined. */

# define config_depend_and2_inline(token1, token2, proto, ...)  \
  config_depend_and2(token1, token2)                            \
  inline proto __VA_ARGS__

# define config_depend_and2_alwaysinline(token1, token2, proto, ...)    \
  config_depend_and2(token1, token2)                                    \
  ALWAYS_INLINE proto __VA_ARGS__

# define config_depend_or2(token1, token2)                              \
  /*+ This is available when either @ref # ## token1 or @ref # ## token2 is defined. */

# define config_depend_or2_inline(token1, token2, proto, ...)   \
  config_depend_or2(token1, token2)\
  inline proto __VA_ARGS__

# define config_depend_or2_alwaysinline(token1, token2, proto, ...)     \
  config_depend_or2(token1, token2)                                     \
  ALWAYS_INLINE proto __VA_ARGS__

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

/** @This may be used to declare a field named @tt field_a of type @tt
    type_a aliasing a @tt field_s in declared in a stucture @tt
    type_s. The struct and the field alias must be nested in an union.
    @see #FIELD_USING

@code
struct base_s
{
  ...
  size_t size;
  ...
};

struct main_s
{
  union {
    struct base_s                  base;
    FIELD_ALIAS(struct base_s,     size,
      size_t,                      main_size)
  };
};
@end code
*/
#define FIELD_ALIAS(type_s, field_s, type_a, field_a)   \
  type_a field_a;

/** @This may be used to reuse declaration of a field from a base struct.
    The struct and the field alias must be nested in an union.
    @see #FIELD_ALIAS

@code
struct base_s
{
  ...
  size_t size;
  ...
};

struct main_s
{
  union {
    struct base_s                  base;
    FIELD_USING(struct base_s,     size);
  };
};
@end code
*/
#define FIELD_USING(type_s, field_s)            \
  typeof(type_s::field_s) field_s;

#endif

#ifdef CONFIG_COMPILE_NOBITFIELD
# define BITFIELD(name, bits) name
#else
# define BITFIELD(name, bits) name:bits
#endif

/** @This expands to an unsigned integer type large enough to hold the
    constant value @tt x */
#define UINT_FIT_TYPE(x) typeof(                           \
  __builtin_choose_expr((uint8_t)(x)  == (x), (uint8_t)0,  \
  __builtin_choose_expr((uint16_t)(x) == (x), (uint16_t)0, \
  __builtin_choose_expr((uint32_t)(x) == (x), (uint32_t)0, \
                        (uint64_t)0))))

/** @This expands to a signed integer type large enough to hold the
    constant value @tt x */
#define INT_FIT_TYPE(x) typeof(                            \
  __builtin_choose_expr((int8_t)(x)  == (x), (uint8_t)0,   \
  __builtin_choose_expr((int16_t)(x) == (x), (int16_t)0,   \
  __builtin_choose_expr((int32_t)(x) == (x), (int32_t)0,   \
                        (int64_t)0))))

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

#define BUSY_WAITING_FUNCTION                                           \
  /*+ Use of a @url {https://en.wikipedia.org/wiki/Busy_waiting}        \
    {busy-waiting} function is bad programming practice. @This should   \
    be used for test purpose only. */                                   \
  DEPRECATED("Use of a busy-waiting function")

#endif /* HEXO_DECLS_H_ */

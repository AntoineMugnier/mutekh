#ifndef HEXO_DECLS_H_
#define HEXO_DECLS_H_

/* warp C header in C++ */

# if __cplusplus &&! defined(__MUTEK_ASM__)
#  define C_HEADER_BEGIN extern "C" {
#  define C_HEADER_END }
# else
#  define C_HEADER_BEGIN
#  define C_HEADER_END
# endif

/* make unavailable functions deprecated */

# ifndef __MUTEK_ASM__

#  define _CONFIG_DEPEND_1(name, b)
#  if _GCT_GNUC_VERSION >= 40500
#   define _CONFIG_DEPEND_0(name, b) __attribute__((deprecated("this symbol depends on " name " which is not defined in configuration")))
#  else
#   define _CONFIG_DEPEND_0(name, b) __attribute__((deprecated))
#  endif

#  define _CONFIG_DEPEND_PASTE(a, b) a ## b
#  define _CONFIG_DEPEND(a, b) _CONFIG_DEPEND_PASTE(_CONFIG_DEPEND_, b)(#a, a)

#  define config_depend(token) _CONFIG_DEPEND(token, _##token)

# endif

#endif /* HEXO_DECLS_H_ */

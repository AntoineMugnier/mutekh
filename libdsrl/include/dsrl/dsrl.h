#ifndef _DSRL_H_
#define _DSRL_H_

#include <lua/lua.h>
void luaopen_dsrl (lua_State *L);

#if defined(CONFIG_LIBDSRL_DEBUG)
# define _dsrl_debug printk
#else
# define _dsrl_debug(...)
#endif

#endif

#ifndef _DSRL_H_
#define _DSRL_H_

error_t dsrl_init(void);

#if defined (CONFIG_LIBDSRL_LUAAPI)
#include <lua/lua.h>
error_t luaopen_dsrl (lua_State *L);
#else
error_t dsrl (const char *filename);
#endif

#endif

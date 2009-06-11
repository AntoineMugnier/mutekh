#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dsrl/dsrl.h>
#include <dsrl/dsrl-private.h>

#include <lua/lua.h>
#include <lua/lauxlib.h>
#include <lua/lualib.h>

/* internal shared function */
static inline int _dsrl_dofile (const char *filename) {
	_dsrl_debug("_dsrl_dofile\n");
    /* create a new state for the script execution 
     * and open the standard and dsrl libs 
     */
    //FIXME: should create a sub-environment instead of a new state...
	_dsrl_debug("\tnew lua state\n");
    lua_State *L_tmp = luaL_newstate();
	_dsrl_debug("\topen aux libs\n");
    luaL_openlibs(L_tmp);
	_dsrl_debug("\topen dsrl libs\n");
    luaopen_dsrl_libs(L_tmp);

    /* execute the script */
	_dsrl_debug("\tfilename: %s\n", filename);
    if (luaL_dofile(L_tmp, filename) != 0)
    {
        printk("%s\n", lua_tostring(L_tmp, -1));
        lua_pop(L_tmp, 1);
        lua_close(L_tmp);
        return -1;
    }

    /* build the new dsrl object */
    dsrl_desc_s *d = (dsrl_desc_s*)malloc(sizeof(dsrl_desc_s));
    d->state = L_tmp;
    d->script = strdup(filename);
	/* add it the list */
	dsrl_list_pushback(&dsrl_root, d);

    return 0;
}

/* External API */
#if defined (CONFIG_LIBDSRL_LUAAPI)
static int _dsrl (lua_State *L)
{
	const char *filename = luaL_checkstring(L, 1);
	if (_dsrl_dofile(filename) != 0)
		luaL_error(L, "dsrl script %s failed", filename);
	return 0;
}
#else
error_t dsrl (const char *filename)
{
	return _dsrl_dofile(filename);
}
#endif

/*
 * Register this new lib
 */
error_t dsrl_init(void)
{
	_dsrl_debug("dsrl_init\n");
	dsrl_list_init(&dsrl_root);
	return 0;
}
#if defined (CONFIG_LIBDSRL_LUAAPI)
error_t luaopen_dsrl (lua_State *L)
{
	lua_register(L, "dsrl", _dsrl);
    return 1;
}
#endif

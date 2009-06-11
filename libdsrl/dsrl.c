#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dsrl/dsrl.h>
#include <dsrl/dsrl-lua.h>

#define DSRL_TCG_HANDLE "lua.dsrl_tcg"

void check_tcg (lua_State *L, int index)
{
    /* check if the table is a "dsrl_tcg" */
    if ((lua_type(L, index) == LUA_TTABLE) && lua_getmetatable(L, index)) {  /* does it have a metatable? */
        lua_getfield(L, LUA_REGISTRYINDEX, DSRL_TCG_HANDLE);  /* get correct metatable */
        if (lua_rawequal(L, -1, -2)) {  /* does it have the correct mt? */
            lua_pop(L, 2); /* remove both metatables */
            return;
        }
    }
    luaL_argerror(L, index, "`"DSRL_TCG_HANDLE"' expected");  
}

static int new_tcg (lua_State *L)
{
    const char *name = luaL_checkstring(L, 1);

    lua_newtable(L);
    luaL_getmetatable(L, DSRL_TCG_HANDLE);
    lua_setmetatable(L, -2);

    lua_pushvalue(L, 1);
    lua_setfield(L, -2, "name");

    return 1;
}

static int load_tcg (lua_State *L)
{
    const char *scriptfile = luaL_checkstring(L, 1);
    _dsrl_debug("\tscriptfile: %s\n", scriptfile);

    if (luaL_dofile(L, scriptfile) != 0)
    {
        printk("%s\n", lua_tostring(L, -1));
        //luaL_error(L, "dsrl script %s failed", scriptfile);
        lua_pushnil(L);
        return 1;
    }

    /* don't care of what's on stack:
     * could be a lua.dsrl_tcg or a user.dsrl_tcg
     */
    return 1;
}

static int run_tcg (lua_State *L)
{
    /* check everything */
    check_tcg(L, 1);

#define check_tcg_resource(resource)                    \
    lua_getfield(L, 1, #resource);                      \
    if (lua_istable(L, -1)) {                           \
        size_t n##resource = 0;                         \
        size_t i##resource = lua_gettop(L);             \
        lua_pushnil(L);                                 \
        while (lua_next(L, i##resource) != 0){          \
            size_t iresource = lua_gettop(L);           \
            size_t iname = iresource - 1;               \
            if (check_##resource(L, iresource) != 0)    \
            {                                           \
                size_t ierror = lua_gettop(L);          \
                luaL_Buffer b;                          \
                luaL_buffinit(L, &b);                   \
                luaL_addstring(&b, #resource" `");      \
                lua_pushvalue(L, iname);                \
                luaL_addvalue(&b);                      \
                luaL_addstring(&b, "': ");              \
                lua_pushvalue(L, ierror);               \
                luaL_addvalue(&b);                      \
                luaL_pushresult(&b);                    \
                lua_error(L);                           \
            }                                           \
            lua_pop(L, 1);                              \
            n##resource++;                              \
        }                                               \
        _dsrl_debug("#"#resource": %d\n", n##resource); \
    } else if (!lua_isnil(L, -1))                       \
    luaL_error(L, #resource": expected to be a table");


    check_tcg_resource(dsrl_const);
    check_tcg_resource(dsrl_barrier);
    check_tcg_resource(dsrl_file);
    check_tcg_resource(dsrl_memspace);
    check_tcg_resource(dsrl_io_memspace);
    check_tcg_resource(dsrl_mwmr);

    check_tcg_resource(dsrl_task);

    _dsrl_debug("check is ok!\n");
    return 0;
}
static int gc_tcg (lua_State *L)
{
    _dsrl_debug("nothing yet...\n");
    /* TODO */
    return 0;
}

static int list (lua_State *L)
{
    luaL_checktype(L, 1, LUA_TTABLE);
    lua_getglobal(L, "print");
    size_t iprint = lua_gettop(L);
    lua_pushnil(L);  /* first key */
    while (lua_next(L, 1)) {
        lua_pushvalue(L, iprint);
        lua_pushvalue(L, -3);  /* key */
        lua_pushvalue(L, -3);  /* value */
        lua_call(L, 2, 1);
        if (!lua_isnil(L, -1))
            return 1;
        lua_pop(L, 2);  /* remove value and result */
    }
    return 0;
}

static const luaL_Reg tcglib_m[] = {
    {"barrier",       new_dsrl_barrier},
    {"const",         new_dsrl_const},
    {"memspace",      new_dsrl_memspace},
    {"file",          new_dsrl_file},
    {"io_memspace",   new_dsrl_io_memspace},
    {"mwmr",          new_dsrl_mwmr},
    {"task",          new_dsrl_task},
    {"run",           run_tcg},
    {"__gc",          gc_tcg},
    {NULL, NULL}
};

void luaopen_tcg_resources (lua_State *L)
{
    luaL_newmetatable(L, DSRL_TCG_HANDLE);         
    lua_pushstring(L, "__index");        
    lua_pushvalue(L, -2);                
    lua_settable(L, -3);                 
    lua_pushstring(L, "__metatable");        
    lua_pushvalue(L, -2);                
    lua_settable(L, -3);                 
    luaL_openlib(L, NULL, tcglib_m, 0); 

    luaopen_dsrl_resources(L);
}

static const luaL_Reg dsrllib_f[] = {
    {"new_tcg",     new_tcg},
    {"load_tcg",    load_tcg},
    {"list",        list},
    {NULL, NULL}
};

void luaopen_dsrl (lua_State *L)
{
    /* register the two functions which create a new tcg */
    luaL_openlib(L, "dsrl", dsrllib_f, 0);
    /* register the metatables for the subtypes of tcg */
    luaopen_tcg_resources(L); 
}

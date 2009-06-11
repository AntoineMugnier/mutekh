#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dsrl/dsrl-lua.h>

#define CHECK_FIELD(field, type, ires)                                            \
    lua_getfield(L, ires, #field);                                                \
    if (!lua_is##type(L, -1))                                                     \
    {                                                                             \
        if (lua_isnil(L, -1))                                                     \
            lua_pushstring(L, "field `"#field"' not existing");                   \
        else                                                                      \
            lua_pushstring(L, "error on field `"#field"' (expecting `"#type"')"); \
        return 1;                                                                 \
    }                                                                             \
    lua_pop(L, 1);

#define RESOURCE_CHECK_FIELDS(resource) error_t check_fields_##resource(lua_State *L, size_t ires)

/* Barrier */
RESOURCE_CHECK_FIELDS(dsrl_barrier)
{
    CHECK_FIELD(max, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_barrier)
/* Memspace */
RESOURCE_CHECK_FIELDS(dsrl_memspace)
{
    CHECK_FIELD(size, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_memspace)
/* IOmemspace */
RESOURCE_CHECK_FIELDS(dsrl_io_memspace)
{
    CHECK_FIELD(size, number, ires);
    CHECK_FIELD(addr, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_io_memspace)
/* File (this is not really part of original srl api...) */
RESOURCE_CHECK_FIELDS(dsrl_file)
{
    CHECK_FIELD(file, string, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_file)
/* Mwmr */
RESOURCE_CHECK_FIELDS(dsrl_mwmr)
{
    CHECK_FIELD(width, number, ires);
    CHECK_FIELD(depth, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_mwmr)
/* Const */
RESOURCE_CHECK_FIELDS(dsrl_const)
{
    CHECK_FIELD(value, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_const)
/* Task */

#define CHECK_ARG(resource)                         \
    if (check_type_##resource(L, iarg) == 0)        \
    {                                               \
        if (check_fields_##resource(L, iarg) != 0)  \
        {                                           \
            size_t ierror = lua_gettop(L);          \
            luaL_Buffer b;                          \
            luaL_buffinit(L, &b);                   \
            lua_pushfstring(L, "arg #%d: ", nargs); \
            luaL_addvalue(&b);                      \
            lua_pushvalue(L, ierror);               \
            luaL_addvalue(&b);                      \
            luaL_pushresult(&b);                    \
            return 1;                               \
        }                                           \
        lua_pop(L, 1);                              \
        continue;                                   \
    }                                               \
    lua_pop(L, 1); /* pop the error message */

RESOURCE_CHECK_FIELDS(dsrl_task)
{
    CHECK_FIELD(exec, string, ires);
    CHECK_FIELD(func, string, ires);
    CHECK_FIELD(sstack, number, ires);
    CHECK_FIELD(cpuid, number, ires);
    CHECK_FIELD(tty, number, ires);
    CHECK_FIELD(args, table, ires);

    /* check the args table are dsrl resources */
    lua_getfield(L, ires, "args");
    /* args table is numerically indexed */
    size_t iargs = lua_gettop(L);
    size_t nargs = luaL_getn(L, iargs);
    size_t i;
    for (i=1; i<=nargs; i++)
    {
        lua_rawgeti(L, iargs, i);
        size_t iarg = lua_gettop(L);

        CHECK_ARG(dsrl_const);
        CHECK_ARG(dsrl_barrier);
        CHECK_ARG(dsrl_file);
        CHECK_ARG(dsrl_memspace);
        CHECK_ARG(dsrl_io_memspace);
        CHECK_ARG(dsrl_mwmr);

        lua_pushfstring(L, "arg #%d: not a dsrl resource", i);
        return 1;
    }
    /* pop the args table */
    lua_pop(L, 1);
    return 0;
}
DSRL_RESOURCE(dsrl_task)

void luaopen_dsrl_resources(lua_State *L)
{
#define REGISTER_DSRL_RESOURCE(resource) createmeta_##resource(L)
    REGISTER_DSRL_RESOURCE(dsrl_const);
    REGISTER_DSRL_RESOURCE(dsrl_barrier);
    REGISTER_DSRL_RESOURCE(dsrl_file);
    REGISTER_DSRL_RESOURCE(dsrl_io_memspace);
    REGISTER_DSRL_RESOURCE(dsrl_memspace);
    REGISTER_DSRL_RESOURCE(dsrl_mwmr);
    REGISTER_DSRL_RESOURCE(dsrl_task);
}

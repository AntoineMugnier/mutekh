
#include <stdio.h>
#include <errno.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <lua/lua.h>
#include <vfs/vfs.h>

#include <dsrl/dsrl.h>

void init_dsrl_shell(lua_State* luast)
{
    luaopen_dsrl(luast);
}

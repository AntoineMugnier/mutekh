#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dsrl/dsrl-private.h>

#include <lua/lua.h>
#include <lua/lauxlib.h>
#include <lua/lualib.h>

#include <hexo/segment.h>
#include <rtld/rtld.h>
#include <cpu/tls.h>

CPU_SYSCALL_HANDLER(dsrl_syscall_handler);

/* Task */
static CONTEXT_ENTRY(dsrl_run_task)
{
    dsrl_task_t task = param;

    // set the syscall handler for the current CPU
    // that's why migration should be forbidden
    cpu_syscall_sethandler(dsrl_syscall_handler);

    /* copy args on stack */
    /* arg[0] = tty, arg[1] = func */
    /* arg[i..n] = task_resources */
    size_t i;
    uintptr_t args[2 + task->nargs];
    args[0] = task->tty;
    args[1] = task->func;
    for (i = 0; i < task->nargs; i++)
    {
        args[i+2] = task->args[i];
    }

    /* setup tls */
    tls_init_tp(task->tls);

    sched_unlock();
    cpu_interrupt_enable();

    /*
     * Two ways of calling the thread:
     *  - direct call in C: it means the thread must manage to become PIC
     *  itself (via an asm routine)
     *  - call with $25: the thread can begin directly with a C function,
     *  expecting $25 to be set correctly
     */
#if 0
    dsrl_func_t *f = task->entrypoint;
    f(args);
#else
    asm volatile (
            ".set push              \n"
            ".set noat              \n"
            ".set noreorder         \n"
            "   move    $25,    %0  \n"
            "   move    $4,     %1  \n"
            "   jr      $25         \n"
            ".set pop               \n"
            :
            : "r" (task->entrypoint)
            , "r" (args)
            );
#endif


    // should not happen (at least if previously we went in user mode)
    cpu_interrupt_disable();
    sched_lock();
    sched_context_exit();
}

CREATE_DSRL_RESOURCE(dsrl_task)
{
    _dsrl_debug("create_dsrl_task\n");
    /* Fixed args */
    const char *filename = luaL_checkstring(L, 3);
    const char *funcname = luaL_checkstring(L, 4);
    size_t sstack = luaL_checkinteger(L, 5);
    size_t cpuid = luaL_checkinteger(L, 6);
    uintptr_t tty = luaL_checkinteger(L, 7);
    dsrl_task_t task = (dsrl_task_t)malloc(sizeof(dsrl_task_s));
    task->tty = tty;
    task->filename = strdup(filename);
    task->funcname = strdup(funcname);
    task->cpuid = cpuid;
    _dsrl_debug("\tfilename: %s\n", filename);
    _dsrl_debug("\tfuncname: %s\n", funcname);
    _dsrl_debug("\tstack_size: %d bytes\n", sstack);
    _dsrl_debug("\tcpuid: %d\n", cpuid);
    _dsrl_debug("\ttty: %p\n", (void*)tty);

    /* load the exec in memory */
    _dsrl_debug("\tload exec in memory\n");
    if (rtld_user_dlopen(filename, &task->entrypoint, &task->handle) != 0)
        luaL_error(L, "dlopen failed on %s", filename);
    if (rtld_user_dlsym(task->handle, funcname, &task->func) != 0)
        luaL_error(L, "dlsym failed on %s", funcname);
    else
        _dsrl_debug("\tfunc is @%p\n", task->func);
    if (rtld_user_dltls(task->handle, &task->tls) != 0)
        luaL_error(L, "dltls failed on %s", filename);
    else
        _dsrl_debug("\ttls is @%p\n", task->tls);

    /* build the mutekH sched context */
    _dsrl_debug("\tBuild sched context\n");
    uint8_t *stack;
    stack = arch_contextstack_alloc(sstack);

    context_init(&task->context.context,
            stack, stack + sstack,
            dsrl_run_task, task);
    sched_context_init(&task->context);
    sched_affinity_single(&task->context, cpuid);

    _dsrl_debug("\ttask is %p\n", task);
    return task;
}
USTRING_DSRL_RESOURCE(dsrl_task)
{
    lua_pushfstring(L, "\tfilename=%s\n", c->filename);
    luaL_addvalue(b);
    lua_pushfstring(L, "\tfuncname=%s\n", c->funcname);
    luaL_addvalue(b);
    lua_pushfstring(L, "\tcpuid=%d\n", c->cpuid);
    luaL_addvalue(b);
    lua_pushfstring(L, "\ttty=%d", c->tty);
    luaL_addvalue(b);
}

#define DSRL_TASK_HANDLE "type.dsrl_task"

void check_dsrl_task (lua_State *L, int i)
{
    /* check if the table is a "dsrl_task" */
    if ((lua_type(L, index) == LUA_TTABLE) && lua_getmetatable(L, index)) {  /* does it have a metatable? */
        lua_getfield(L, LUA_REGISTRYINDEX, DSRL_TASK_HANDLE);  /* get correct metatable */
        if (lua_rawequal(L, -1, -2)) {  /* does it have the correct mt? */
            lua_pop(L, 2); /* remove both metatables */
            return;
        }
    }
    luaL_argerror(L, index, "`"DSRL_TASK_HANDLE"' expected");  
}
error_t new_dsrl_task (lua_State *L)
{
    /* create a new "dsrl_task" table to store a task and its args */
    lua_newtable(L);
    size_t itask = lua_gettop(L);
    lua_getmetatable(L, DSRL_TASK_HANDLE);
    lua_setmetatable(L, -2);

    /* check the args are proper dsrl userdata */
    lua_pushnil(L);
    while (lua_next(L, 8) != 0)
    {
        void *arg = lua_touserdata(L, -1);
        if (arg == NULL)
            return 1;
        if (lua_getmetatable(L, -1) == 0)
            return 1;

#define check_dsrl_type(resource)                      \
        lua_getfield(L, LUA_REGISTRYINDEX, #resource); \
        if (lua_rawequal(L, -2, -1))                   \
            continue;                                  \
        lua_pop(L, 1)

        check_dsrl_type(dsrl_const);
        check_dsrl_type(dsrl_barrier);
        check_dsrl_type(dsrl_file);
        check_dsrl_type(dsrl_memspace);
        check_dsrl_type(dsrl_io_memspace);
        check_dsrl_type(dsrl_mwmr);

        return 1;
    }
    lua_pushvalue(L, 8);
    /* store the args list in the "args" field of the "dsrl_task" table */
    lua_setfield(L, itask, "args");

    /* create the task object */
    dsrl_task_t *d = (dsrl_task_t*)lua_newuserdata(L, sizeof(dsrl_task_t));
    *d = NULL;
    *d = create_dsrl_task(L);
    if (*d == NULL)
        return 1;
    /* store the object in the "task" field of the "dsrl_task" table */
    lua_setfield(L, itask, "task");

    return 0;
}
static int string_dsrl_task (lua_State *L)
{
    check_dsrl_task(L, 1);
    lua_getfield(L, 1, "task");
    dsrl_task_t c = *(dsrl_task_t*)lua_touserdata(L, -1);
    luaL_Buffer b;
    luaL_buffinit(L, &b);
    lua_pushfstring(L, "type.dsrl_task: %s\n", c->name);
    luaL_addvalue(&b);
    ustring_dsrl_task(L, c, &b);
    luaL_pushresult(&b);
    return 1;
}
static const luaL_reg m_dsrl_task[] = {
    {"__tostring", string_dsrl_task},
    {0, 0}
};
void createmeta_dsrl_task(lua_State *L)
{
    _dsrl_debug("_createmeta_dsrl_task\n");
    luaL_newmetatable(L, "type.dsrl_task");
    lua_pushstring(L, "__index");
    lua_pushvalue(L, -2);
    lua_settable(L, -3);
    lua_pushstring(L, "__metatable");
    lua_pushvalue(L, -2);
    lua_settable(L, -3);
    luaL_openlib(L, NULL, m_dsrl_task, 0);
}

/* we use the same table as other resources */
DSRL_RESOURCE_TABLE(dsrl_task)


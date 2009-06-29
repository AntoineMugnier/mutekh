#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dsrl/dsrl-lua.h>
#include <hexo/segment.h>
#include <rtld/rtld.h>
#include <cpu/tls.h>

/*
 * checking utilities
 */
#define RESOURCE_CHECK_FIELDS(resource) error_t check_fields_##resource(lua_State *L, size_t ires)
#define CHECK_EXTERNAL(ires)       \
    lua_getfield(L, ires, "addr"); \
    if (!lua_isnil(L, -1)){        \
        lua_pop(L, 1);             \
        return 0;                  \
    }                              \
    lua_pop(L, 1);

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

/*
 * building utilities
 */
#define GET_FIELD(field, type, ires)                  \
({                                                    \
    lua_getfield(L, ires, #field);                    \
    lua_to##type(L, -1);                              \
})
#define GET_INTEGER_FIELD(field, ires)                \
({                                                    \
    size_t res = GET_FIELD(field, integer, ires);     \
    lua_pop(L, 1);                                    \
    res;                                              \
})
#define GET_STRING_FIELD(field, ires)                 \
({                                                    \
    const char *res = GET_FIELD(field, string, ires); \
    lua_pop(L, 1);                                    \
    res;                                              \
})

#define RESOURCE_BUILD(resource) void build_##resource(lua_State *L, size_t ires, resource##_t **p)

#define RESOURCE_ALLOC(resource)                                     \
    resource##_t *res = (resource##_t*)malloc(sizeof(resource##_t)); \
    *p = res                                                         \

/* Barrier */
RESOURCE_CHECK_FIELDS(dsrl_barrier)
{
    CHECK_EXTERNAL(ires);
    CHECK_FIELD(max, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_barrier)
RESOURCE_BUILD(dsrl_barrier)
{
    RESOURCE_ALLOC(dsrl_barrier);
    size_t n = GET_INTEGER_FIELD(max, ires);
    res->count = n;
    res->max = n;
    res->serial = 0;
    res->lock = 0; 
}
/* Const */
RESOURCE_CHECK_FIELDS(dsrl_const)
{
    CHECK_EXTERNAL(ires);
    CHECK_FIELD(value, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_const)
RESOURCE_BUILD(dsrl_const)
{
    *p = (dsrl_const_t*)GET_INTEGER_FIELD(value, ires);
}
/* Memspace */
RESOURCE_CHECK_FIELDS(dsrl_memspace)
{
    CHECK_EXTERNAL(ires);
    CHECK_FIELD(size, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_memspace)
RESOURCE_BUILD(dsrl_memspace)
{
    RESOURCE_ALLOC(dsrl_memspace);
    res->size = GET_INTEGER_FIELD(size, ires);
    res->buffer = (dsrl_buffer_t)malloc(res->size);
}
/* IOmemspace */
RESOURCE_CHECK_FIELDS(dsrl_io_memspace)
{
    CHECK_EXTERNAL(ires);
    CHECK_FIELD(size, number, ires);
    CHECK_FIELD(mmap, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_io_memspace)
RESOURCE_BUILD(dsrl_io_memspace)
{
    RESOURCE_ALLOC(dsrl_io_memspace);
    res->size = GET_INTEGER_FIELD(size, ires);
    res->buffer = (dsrl_buffer_t)GET_INTEGER_FIELD(mmap, ires);
}
/* File (this is not really part of original srl api...) */
RESOURCE_CHECK_FIELDS(dsrl_file)
{
    CHECK_EXTERNAL(ires);
    CHECK_FIELD(file, string, ires);
    CHECK_FIELD(size, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_file)
RESOURCE_BUILD(dsrl_file)
{
    RESOURCE_ALLOC(dsrl_file);
    const char* filename = GET_STRING_FIELD(file, ires);
//    struct stat st;
//    if (stat(filename, &st) != 0)
//    {
//        lua_pushfstring(L, "`stat()' failed on file `%s'\n", filename);
//        lua_error(L);
//    }
//    _dsrl_debug("file is %d bytes\n", st.st_size);
//    res->size = st.st_size;
    res->size = GET_INTEGER_FIELD(size, ires);
    res->buffer = (dsrl_buffer_t)malloc(res->size);

    FILE *f = fopen(filename, "r");
    fread(res->buffer, 1, res->size, f);
    fclose(f);
}
/* Mwmr */
RESOURCE_CHECK_FIELDS(dsrl_mwmr)
{
    CHECK_EXTERNAL(ires);
    CHECK_FIELD(width, number, ires);
    CHECK_FIELD(depth, number, ires);
    return 0;
}
DSRL_RESOURCE(dsrl_mwmr)
RESOURCE_BUILD(dsrl_mwmr)
{
    RESOURCE_ALLOC(dsrl_mwmr);
    res->width = GET_INTEGER_FIELD(width, ires);
    res->depth = GET_INTEGER_FIELD(depth, ires);
    size_t size = res->width * res->depth;
    res->gdepth = size;
    res->buffer = (dsrl_buffer_t)malloc(size);

    res->status.free_tail = 0;
    res->status.free_head = 0;
    res->status.free_size = size;
    res->status.data_tail = 0;
    res->status.data_head = 0;
    res->status.data_size = 0;
}

/* Task */
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

static CONTEXT_ENTRY(dsrl_run_task)
{
    dsrl_task_t *task = param;

    // set the syscall handler for the current CPU
    // that's why migration should be forbidden
    CPU_SYSCALL_HANDLER(dsrl_syscall_handler);
    cpu_syscall_sethandler(dsrl_syscall_handler);

    /* copy args on stack */
    /* arg[0] = tty, arg[1] = func */
    /* arg[i..n] = task_resources */
    uintptr_t args[2 + task->nargs];
    args[0] = task->tty;
    args[1] = task->func;
#if 0
    memcpy(&args+2, &task->args, 4*task->nargs);
#else
    size_t i;
    for (i = 0; i < task->nargs; i++)
    {
        args[i+2] = task->args[i];
    }
#endif

    /* setup tls */
    tls_init_tp(task->tp);

    sched_unlock();
    cpu_interrupt_enable();

    /*
     *  direct call in C: it means the thread must manage to become PIC
     *  itself (via an asm routine)
     */
    typedef void* dsrl_func_t (void*);
    dsrl_func_t *f = task->entrypoint;
    f(args);

    // should not happen (at least if previously we went in user mode)
    cpu_interrupt_disable();
    sched_lock();
    sched_context_exit();
}

RESOURCE_BUILD(dsrl_task)
{
    RESOURCE_ALLOC(dsrl_task);
    res->execname = GET_STRING_FIELD(exec, ires);
    res->funcname = GET_STRING_FIELD(func, ires);

    res->cpuid  = GET_INTEGER_FIELD(cpuid, ires);
    res->tty = GET_INTEGER_FIELD(tty, ires);

    size_t sstack = GET_INTEGER_FIELD(sstack, ires);

    /* Fill the args */
    lua_getfield(L, ires, "args");
    /* args table is numerically indexed */
    size_t iargs = lua_gettop(L);
    size_t nargs = luaL_getn(L, iargs);
    res->nargs = nargs;
    res->args = (uintptr_t)malloc(nargs*sizeof(uintptr_t));
    size_t i;
    for (i=0; i<nargs; i++)
    {
        lua_rawgeti(L, iargs, i+1);
        size_t iarg = lua_gettop(L);
        lua_getfield(L, iarg, "addr");
        res->args[i] = lua_tointeger(L, -1);
        lua_pop(L, 2);
    }
    /* pop the args table */
    lua_pop(L, 1);

    /* load the exec in memory */
    _dsrl_debug("\tload exec in memory\n");
    if (rtld_user_dlopen(res->execname, &res->entrypoint, &res->handle) != 0)
        luaL_error(L, "dlopen failed on %s", res->execname);

    if (rtld_user_dlsym(res->handle, res->funcname, &res->func) != 0)
        luaL_error(L, "dlsym failed on %s", res->funcname);
    else
        _dsrl_debug("\tfunc is @%p\n", res->func);

    res->tls = NULL;
    if (rtld_user_dltls(res->handle, &res->tls, &res->tp) != 0)
        luaL_error(L, "dltls failed on %s", res->execname);
    else
        _dsrl_debug("\ttls is @%p\n", res->tls);

    /* build the mutekH sched context */
    _dsrl_debug("\tBuild sched context\n");
    uint8_t *stack;
    stack = arch_contextstack_alloc(sstack);

    context_init(&res->context.context,
            stack, stack + sstack,
            dsrl_run_task, res);
    sched_context_init(&res->context);
    sched_affinity_single(&res->context, res->cpuid);
}

/*
 * Register dsrl resources
 */
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

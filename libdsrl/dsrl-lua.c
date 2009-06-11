#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hexo/segment.h>

#include <dsrl/dsrl-private.h>
#include <dsrl/syscall.h>

#include <lua/lua.h>
#include <lua/lauxlib.h>
#include <lua/lualib.h>

#include <rtld/rtld.h>
#include <cpu/tls.h>

static CONTEXT_ENTRY(dsrl_run_task);

#define DECLARE_LUA_RESOURCES(name)							            \
	static name##_t to_##name (lua_State *L, int i)                     \
	{														            \
    	name##_t *d = (name##_t*)luaL_checkudata(L, i, #name);          \
        if (*d == NULL)                                                 \
            luaL_error(L, "error retrieving "#name" object");           \
    	return *d;          								            \
	}														            \
	static int _new_##name (lua_State *L)	    			            \
	{														            \
		name##_t *d = (name##_t*)lua_newuserdata(L, sizeof(name##_t));	\
		*d = NULL;											            \
		luaL_getmetatable(L, #name);						            \
		lua_setmetatable(L, -2);							            \
		*d = _create_##name(L);	        					            \
        if (*d == NULL) {                                               \
            lua_pushnil(L);                                             \
            lua_pushstring(L, "error creating "#name" object");         \
            return 2;                                                   \
        } else return 1;                                                \
	}														            \
	static const luaL_reg _f_##name[] = {					            \
		{"new",	_new_##name},				        		            \
		{0, 0}												            \
	};														            \
	static int _gc_##name (lua_State *L)					            \
	{														            \
		name##_t d = to_##name(L, 1);                                   \
		_destroy_##name(d);		    						            \
		return 0;											            \
	}														            \
	static const luaL_reg _m_##name[] = {					            \
		{"__gc",	_gc_##name},							            \
		{0, 0}												            \
	};														            \
	static const error_t _luaopen_##name(lua_State *L)		            \
	{														            \
        _dsrl_debug("_luaopen_"#name"\n");                              \
		luaL_newmetatable(L, #name);						            \
		lua_pushstring(L, "__index");						            \
		lua_pushvalue(L, -2);								            \
		lua_settable(L, -3);								            \
		luaL_openlib(L, NULL, _m_##name, 0);				            \
		luaL_openlib(L, #name, _f_##name, 0);				            \
		return 1;											            \
	}

#define CREATE_LUA_RESOURCE(name) static name## _t _create_##name (lua_State *L)
#define DESTROY_LUA_RESOURCE(name) static void _destroy_##name (name##_t d)

/*
 * Barrier
 */
CREATE_LUA_RESOURCE(dsrl_barrier)
{
    _dsrl_debug("_create_dsrl_barrier\n");
    size_t n = luaL_checkinteger(L, 1);

    dsrl_barrier_t barrier = (dsrl_barrier_t)malloc(sizeof(dsrl_barrier_s));
    barrier->count = n;
    barrier->max = n;
    barrier->serial = 0;
    barrier->lock = 0;

    _dsrl_debug("\tmax: %d\n", n);
    _dsrl_debug("\tbarrier is %p\n", barrier);

    return barrier;
}
DESTROY_LUA_RESOURCE(dsrl_barrier)
{
    _dsrl_debug("_destroy_dsrl_barrier\n");
    free(d);
}
DECLARE_LUA_RESOURCES(dsrl_barrier)

/*
 * Memspace
 */
CREATE_LUA_RESOURCE(dsrl_memspace)
{
    _dsrl_debug("_create_dsrl_memspace\n");
	size_t size = luaL_checkinteger(L, 1);
	dsrl_memspace_t mem;
    void *buf;

    /* check if the buffer exists already */
    if ((buf = (void*)lua_tointeger(L, 2)) != 0)
    {
        _dsrl_debug("\tretrieve an existing memspace at %p\n", buf);
        /* typically for framebuffer, the memspace exists */
        mem = (dsrl_memspace_t)malloc(sizeof(dsrl_memspace_s));
        mem->buffer = buf;
    } else {
        _dsrl_debug("\tcreate a new memspace\n");
        mem = (dsrl_memspace_t)malloc(sizeof(dsrl_memspace_s) + size);
        mem->buffer = (uint8_t*)mem + sizeof(dsrl_memspace_s);
    }

    mem->size = size;

    _dsrl_debug("\tsize: %d bytes\n", size);
    _dsrl_debug("\tmem is %p\n", mem);
    return mem;
}
DESTROY_LUA_RESOURCE(dsrl_memspace)
{
    _dsrl_debug("_destroy_dsrl_memspace\n");
    //FIXME
    //free(d->buffer);
    //free(d);
}
DECLARE_LUA_RESOURCES(dsrl_memspace)

/*
 * Mwmr
 */
CREATE_LUA_RESOURCE(dsrl_mwmr)
{
    _dsrl_debug("_create_dsrl_mwmr\n");

	size_t width = luaL_checkinteger(L, 1);
	size_t depth = luaL_checkinteger(L, 2);
	size_t size = width*depth;

	dsrl_mwmr_t mwmr = (dsrl_mwmr_t)malloc(sizeof(dsrl_mwmr_s) + size);

    mwmr->width = width;
    mwmr->depth = depth;
    mwmr->gdepth = size;

    mwmr->status.free_tail = 0;
    mwmr->status.free_head = 0;
    mwmr->status.free_size = size;
    mwmr->status.data_tail = 0;
    mwmr->status.data_head = 0;
    mwmr->status.data_size = 0;

    mwmr->buffer = (uint8_t*)mwmr + sizeof(dsrl_mwmr_s);
    _dsrl_debug("\tmwmr is %p\n", mwmr);
    return mwmr;
}
DESTROY_LUA_RESOURCE(dsrl_mwmr)
{
    _dsrl_debug("_destroy_dsrl_mwmr\n");
    //FIXME
    //free(d->buffer);
    //free(d);
}
DECLARE_LUA_RESOURCES(dsrl_mwmr)

/*
 * Const
 */
CREATE_LUA_RESOURCE(dsrl_const)
{
    _dsrl_debug("_create_dsrl_const\n");
    uintptr_t val = luaL_checkinteger(L, 1);
    /* we are forced to create a const via malloc otherwise return a null const
     * will be interpreted as an error */
    dsrl_const_t c = (dsrl_const_t)malloc(sizeof(uintptr_t));
    *c = val;
    _dsrl_debug("\tval is %d, const is %p\n", val, c);
    return c;
}
DESTROY_LUA_RESOURCE(dsrl_const)
{
    _dsrl_debug("_destroy_dsrl_const\n");
    //FIXME
    //free(d);
}
DECLARE_LUA_RESOURCES(dsrl_const)

/*
 * File (this is not really part of original srl api...)
 */
CREATE_LUA_RESOURCE(dsrl_file)
{
    _dsrl_debug("_create_dsrl_file\n");
	dsrl_file_t mem;
    const char *filename = luaL_checkstring(L, 1);

    struct stat st;
    if (stat(filename, &st) != 0)
    {
        _dsrl_debug("\tstat failed on %s\n", filename);
        return NULL;
    }

    _dsrl_debug("\tcreate a new space of %d bytes\n", st.st_size);

    mem = (dsrl_file_t)malloc(sizeof(dsrl_file_s) + st.st_size);
    mem->buffer = (uint8_t*)mem + sizeof(dsrl_file_s);
    mem->size = st.st_size;

    _dsrl_debug("\tfill the space with %s\n", filename);

    FILE *f = fopen(filename, "r");
    fread(mem->buffer, st.st_size, 1, f);
    fclose(f);

    _dsrl_debug("\tsize: %d bytes\n", st.st_size);
    _dsrl_debug("\tmem is %p\n", mem);
    return mem;
}
DESTROY_LUA_RESOURCE(dsrl_file)
{
    _dsrl_debug("_destroy_dsrl_file\n");
    free(d->buffer);
    free(d);
}
DECLARE_LUA_RESOURCES(dsrl_file)

/*
 * Task
 */
CREATE_LUA_RESOURCE(dsrl_task)
{
    _dsrl_debug("_create_dsrl_task\n");
    /* Fixed args */
    const char *filename = luaL_checkstring(L, 1);
    const char *funcname = luaL_checkstring(L, 2);
    size_t sstack = luaL_checkinteger(L, 3);
    size_t cpuid = luaL_checkinteger(L, 4);
    uintptr_t tty = luaL_checkinteger(L, 5);

    dsrl_task_t task = (dsrl_task_t)malloc(sizeof(dsrl_task_s));
    task->tty = tty;

    _dsrl_debug("\tfilename: %s\n", filename);
    _dsrl_debug("\tfuncname: %s\n", funcname);
    _dsrl_debug("\tstack_size: %d bytes\n", sstack);
    _dsrl_debug("\tcpuid: %d\n", cpuid);
    _dsrl_debug("\ttty: %p\n", (void*)tty);

    /* arguments table */
    size_t i;
    luaL_checktype(L, 6, LUA_TTABLE);
    task->nargs = luaL_getn(L, 6);
    if (task->nargs != 0)
    {
        task->args = malloc(task->nargs*sizeof(uintptr_t));
        for (i=1; i<=task->nargs; i++) 
        {
            lua_rawgeti(L, 6, i);  /* push arg[i] */
            switch(lua_type(L, -1))
            {
                case LUA_TUSERDATA:
                    {
                        void *arg = lua_touserdata(L, -1);
                        if (arg != NULL && lua_getmetatable(L, -1))
                        {
                            /* const */
                            lua_getfield(L, LUA_REGISTRYINDEX, "dsrl_const");
                            if (lua_rawequal(L, -2, -1))
                            {
                                task->args[i-1] = **(dsrl_const_t*)arg;
                                goto userdata_ok;
                            }

                            /* memspace */
                            lua_pop(L, 1);
                            lua_getfield(L, LUA_REGISTRYINDEX, "dsrl_memspace");
                            if (lua_rawequal(L, -2, -1))
                            {
                                task->args[i-1] = *(dsrl_memspace_t*)arg;
                                goto userdata_ok;
                            }

                            /* mwmr */
                            lua_pop(L, 1);
                            lua_getfield(L, LUA_REGISTRYINDEX, "dsrl_mwmr");
                            if (lua_rawequal(L, -2, -1))
                            {
                                task->args[i-1] = *(dsrl_mwmr_t*)arg;
                                goto userdata_ok;
                            }

                            /* barrier */
                            lua_pop(L, 1);
                            lua_getfield(L, LUA_REGISTRYINDEX, "dsrl_barrier");
                            if (lua_rawequal(L, -2, -1))
                            {
                                task->args[i-1] = *(dsrl_barrier_t*)arg;
                                goto userdata_ok;
                            }

                            /* mwmr */
                            lua_pop(L, 1);
                            lua_getfield(L, LUA_REGISTRYINDEX, "dsrl_file");
                            if (lua_rawequal(L, -2, -1))
                            {
                                task->args[i-1] = *(dsrl_file_t*)arg;
                                goto userdata_ok;
                            }

                            luaL_typerror(L, -3, "dsrl_*");
userdata_ok:
                            lua_pop(L, 2);
                        }
                        else
                            luaL_typerror(L, -1, "dsrl_*");

                        _dsrl_debug("\targ[%d]: %p\n", i, (void*)task->args[i-1]);
                    }
                    break;
                default:
                    free(task->args);
                    free(task);
                    luaL_typerror(L, -1, "dsrl_*");
            }
            lua_pop(L, 1); /* pop arg[i] */
        }
    }

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
DESTROY_LUA_RESOURCE(dsrl_task)
{
    _dsrl_debug("_destroy_dsrl_task\n");
    free(d->args);
    free(d);
}
DECLARE_LUA_RESOURCES(dsrl_task)

/*
 * Tcg
 */

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

CREATE_LUA_RESOURCE(dsrl_tcg)
{
    _dsrl_debug("_create_dsrl_tcg\n");
    dsrl_tcg_t tcg = NULL;

    /* arguments table */
    size_t i, nargs;
    luaL_checktype(L, 1, LUA_TTABLE);
    nargs = luaL_getn(L, 1);
    if (nargs != 0)
    {
        tcg = (dsrl_tcg_t)malloc(sizeof(dsrl_tcg_s));
        dsrl_task_list_init(&tcg->task_list);

        for (i=1; i<=nargs; i++) 
        {
            lua_rawgeti(L, 1, i);  /* push arg[i] */

            dsrl_task_list_t t = (dsrl_task_list_t)malloc(sizeof(_dsrl_task_list_s));
#if 1
            t->task = to_dsrl_task(L, -1);
#else
            void *arg = lua_touserdata(L, -1);
            if (arg != NULL && lua_getmetatable(L, -1))
            {
                lua_getfield(L, LUA_REGISTRYINDEX, "dsrl_task");
                if (lua_rawequal(L, -2, -1))
                {
                    t->task = *(dsrl_task_t*)arg;
                    lua_pop(L, 2);
                    goto ok;
                }
                luaL_typerror(L, -3, "dsrl_task");
            }
            luaL_typerror(L, -1, "dsrl_task");
ok:
#endif
            _dsrl_debug("\tlauch task %p\n", t->task);
            dsrl_task_list_pushback(&tcg->task_list, t);

            lua_pop(L, 1); /* pop arg[i] */
        }
    }
    CONTAINER_FOREACH(dsrl_task_list, CLIST, &tcg->task_list,
    {
        CPU_INTERRUPT_SAVESTATE_DISABLE;
        sched_context_start(&item->task->context);
        CPU_INTERRUPT_RESTORESTATE;
    });

    return tcg;
}
DESTROY_LUA_RESOURCE(dsrl_tcg)
{
    _dsrl_debug("_destroy_dsrl_tcg\n");
}
DECLARE_LUA_RESOURCES(dsrl_tcg)

/*
 * External API
 */
#define OPEN_LUA_RESOURCE(name) _luaopen_##name(L)
error_t luaopen_dsrl_libs (lua_State *L)
{
    OPEN_LUA_RESOURCE(dsrl_barrier);
    OPEN_LUA_RESOURCE(dsrl_memspace);
    OPEN_LUA_RESOURCE(dsrl_const);
    OPEN_LUA_RESOURCE(dsrl_mwmr);
    OPEN_LUA_RESOURCE(dsrl_file);
    OPEN_LUA_RESOURCE(dsrl_task);
    OPEN_LUA_RESOURCE(dsrl_tcg);

    return 1;
}

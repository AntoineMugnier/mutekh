#ifndef _DSRL_PRIVATE_H_
#define _DSRL_PRIVATE_H_

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

#include <mutek/scheduler.h>

#include <lua/lua.h>

/*
 * Utils
 */
#if defined(CONFIG_LIBDSRL_DEBUG)
# define _dsrl_debug printk
#else
# define _dsrl_debug(...)
#endif

/* 
 * Dsrl descriptor
 */
typedef struct dsrl_desc_s {
    lua_State *state;
    char *script;

	CONTAINER_ENTRY_TYPE(CLIST) list_entry;
} dsrl_desc_s;

CONTAINER_TYPE(dsrl_list, CLIST, dsrl_desc_s, list_entry);
CONTAINER_FUNC(dsrl_list, CLIST, static inline, dsrl_list, list_entry);

dsrl_list_root_t dsrl_root;

/*
 * Task resource
 */
#define DSRL_FUNC(n)	void* (n) (void *arg)
typedef DSRL_FUNC(dsrl_func_t);

typedef struct {
	/* dynamic loader */
	void *handle;
	dsrl_func_t *entrypoint;
	uintptr_t func;
	uintptr_t tls;

	uintptr_t *args;
	size_t nargs;

	uintptr_t tty;

	/* Hexo context */
	struct sched_context_s context;

	/* Sched wait syscall */
	volatile uint32_t *wait_addr;
	uint32_t	wait_val;
} dsrl_task_s;
typedef dsrl_task_s* dsrl_task_t;

/*
 * TCG resource
 */
typedef struct {
	dsrl_task_t task;
	CONTAINER_ENTRY_TYPE(CLIST)	list_entry;
} _dsrl_task_list_s;
typedef _dsrl_task_list_s* dsrl_task_list_t;

CONTAINER_TYPE(dsrl_task_list, CLIST, _dsrl_task_list_s, list_entry);
CONTAINER_FUNC(dsrl_task_list, CLIST, static inline, dsrl_task_list, list_entry);

typedef struct {
	dsrl_task_list_root_t task_list;
} dsrl_tcg_s;
typedef dsrl_tcg_s* dsrl_tcg_t;

/*
 * Memspace resource
 */
typedef void* dsrl_buffer_t;
typedef struct {
	uint32_t size;
	dsrl_buffer_t buffer;
} dsrl_memspace_s;
typedef dsrl_memspace_s* dsrl_memspace_t;

/*
 * Barrier resource
 */
typedef struct {
    int8_t max;
    int8_t count;
    volatile int32_t lock;
    volatile uint32_t serial;
} dsrl_barrier_s;
typedef dsrl_barrier_s* dsrl_barrier_t;

/*
 * Mwmr resource
 */
typedef struct {
     uint32_t free_tail; // bytes 
     uint32_t free_head; // bytes 
     uint32_t free_size; // bytes 
     uint32_t data_tail; // bytes 
     uint32_t data_head; // bytes 
     uint32_t data_size; // bytes 
} soclib_mwmr_status_s;

typedef struct {
    size_t width;
    size_t depth;
    size_t gdepth;
    soclib_mwmr_status_s status;
    dsrl_buffer_t buffer;
} dsrl_mwmr_s;
typedef dsrl_mwmr_s* dsrl_mwmr_t;

/*
 * Const resource
 */
typedef uintptr_t* dsrl_const_t;

/*
 * File resource
 */
typedef dsrl_memspace_s dsrl_file_s;
typedef dsrl_file_s* dsrl_file_t;

/* 
 * Internal API
 */
error_t luaopen_dsrl_libs (lua_State *L);

#endif /* _DSRL_PRIVATE_H_ */

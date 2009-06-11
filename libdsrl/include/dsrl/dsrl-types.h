#ifndef _DSRL_TYPES_H_
#define _DSRL_TYPES_H_

#include <mutek/scheduler.h>

/*
 * Task resource
 */
typedef struct {
    /* dynamic loader */
    void *handle;
    uintptr_t entrypoint;
    uintptr_t func;
    uintptr_t tls;

    uintptr_t *args;
    size_t nargs;

    const char *filename;
    const char *funcname;
    uintptr_t tty;
    size_t cpuid;

    /* Hexo context */
    struct sched_context_s context;

    /* Sched wait syscall */
    volatile uint32_t *wait_addr;
    uint32_t    wait_val;
} dsrl_task_s;
typedef dsrl_task_s* dsrl_task_t;

/**********************
 * standard resources *
 **********************/

/*
 * Memspace resource
 */
typedef void* dsrl_buffer_t;
typedef struct {
    uint32_t size;
    dsrl_buffer_t buffer;
} dsrl_memspace_s;
typedef dsrl_memspace_s* dsrl_memspace_t;

typedef dsrl_memspace_s* dsrl_io_memspace_t;

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
typedef struct {
    uint32_t val;
} dsrl_const_s;
typedef dsrl_const_s* dsrl_const_t;

/*
 * File resource
 */
typedef dsrl_memspace_s dsrl_file_s;
typedef dsrl_file_s* dsrl_file_t;


#endif

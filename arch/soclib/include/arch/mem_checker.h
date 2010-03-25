
#ifndef SOCLIB_MEM_CHECKER_H_
#define SOCLIB_MEM_CHECKER_H_

/** Magic value must be stored here to enable other registers. 0 must
    be stored to exit magix state. */
#define SOCLIB_MC_MAGIC (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 0)
/** Magic value */
# define SOCLIB_MC_MAGIC_VAL 0x4d656d63

/** Data register 1, depends on action performed */
#define SOCLIB_MC_R1 (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 4)

/** Data register 2, depends on action performed */
#define SOCLIB_MC_R2 (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 8)

/** Creates a new context: R1 = base, R2 = size, value = id */
#define SOCLIB_MC_CTX_CREATE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 12)

/** Delete a context: value = id */
#define SOCLIB_MC_CTX_DELETE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 16)

/** Change a context id: R1 = old_id, value = new_id */
#define SOCLIB_MC_CTX_CHANGE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 20)

/** Set current context id */
#define SOCLIB_MC_CTX_SET (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 24)

/** Set allocated region state: R1 = base, R2 = size, value = state */
#define SOCLIB_MC_REGION_UPDATE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 28)
/** Allocatable region state is free */
# define SOCLIB_MC_REGION_FREE 1
/** Allocatable region state is allocated */
# define SOCLIB_MC_REGION_ALLOC 2

/** Set enabled checks */
#define SOCLIB_MC_ENABLE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 32)
/** Set disabled checks */
#define SOCLIB_MC_DISABLE (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 36)

/** Mark region as intialized */
#define SOCLIB_MC_INITIALIZED (CONFIG_SOCLIB_MEMCHECK_ADDRESS + 40)

/** Stack pointer range must be checked */
# define SOCLIB_MC_CHECK_SP 1
/** Frame pointer range must be checked */
# define SOCLIB_MC_CHECK_FP 2
/** Memory initialization status must be checked */
# define SOCLIB_MC_CHECK_INIT 4
/** Memory allocation status must be checked */
# define SOCLIB_MC_CHECK_REGIONS 8

# if defined(CONFIG_COMPILE_FRAMEPTR) && !defined(__OPTIMIZE__)
#  define SOCLIB_MC_CHECK_SPFP 3
# else
#  define SOCLIB_MC_CHECK_SPFP 1
# endif

#define ASM_STR_(x) #x
#define ASM_STR(x) ASM_STR_(x)

#include <hexo/iospace.h>
#include <hexo/interrupt.h>

static inline __attribute__ ((always_inline)) void
soclib_mem_check_change_id(uint32_t old_id, uint32_t new_id)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_R1, old_id);
  cpu_mem_write_32(SOCLIB_MC_CTX_CHANGE, new_id);
  cpu_mem_write_32(SOCLIB_MC_CTX_SET, new_id);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
  CPU_INTERRUPT_RESTORESTATE;
}

static inline __attribute__ ((always_inline)) void
soclib_mem_check_create_ctx(uint32_t ctx_id, void *stack_start, void *stack_end)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_R1, (uint32_t)stack_start);
  cpu_mem_write_32(SOCLIB_MC_R2, (uint32_t)(stack_end - stack_start));
  cpu_mem_write_32(SOCLIB_MC_CTX_CREATE, ctx_id);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
  CPU_INTERRUPT_RESTORESTATE;
}

static inline __attribute__ ((always_inline)) void
soclib_mem_check_delete_ctx(uint32_t ctx_id)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_CTX_DELETE, ctx_id);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
  CPU_INTERRUPT_RESTORESTATE;
}

static inline __attribute__ ((always_inline)) void
soclib_mem_check_region_status(void *region, size_t size, uint32_t status)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_R1, (uint32_t)region);
  cpu_mem_write_32(SOCLIB_MC_R2, size);
  cpu_mem_write_32(SOCLIB_MC_REGION_UPDATE, status);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
  CPU_INTERRUPT_RESTORESTATE;
}

static inline __attribute__ ((always_inline)) void
soclib_mem_check_disable(uint32_t flags)
{
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_DISABLE, flags);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
  __asm__ __volatile__("" ::: "memory");
}

static inline __attribute__ ((always_inline)) void
soclib_mem_check_enable(uint32_t flags)
{
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_ENABLE, flags);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
  __asm__ __volatile__("" ::: "memory");
}

#endif


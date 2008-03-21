#include <stddef.h>
#include <hexo/endian.h>
#include <hexo/scheduler.h>
#include <srl_private_types.h>
#include <srl/srl_sched_wait.h>

static inline srl_task_s *context_to_srl_task( struct sched_context_s *ctx )
{
    uintptr_t p = (uintptr_t)ctx;
    p -= offsetof(srl_task_s, context);
    return (srl_task_s *)p;
}

#define DECLARE_WAIT(endianness, name, cmp)								\
                                                                        \
    static SCHED_CANDIDATE_FCN(wait_##name##endianness##_f)				\
    {                                                                   \
        srl_task_s *task = context_to_srl_task(sched_ctx);              \
                                                                        \
        cpu_dcache_invld(task->wait_addr);                              \
        return (endian_##endianness##32(*task->wait_addr) cmp task->wait_val); \
    }                                                                   \
                                                                        \
    void srl_sched_wait_##name##_##endianness( uint32_t*addr, uint32_t val )           \
    {                                                                   \
        if ( endian_##endianness##32(*addr) cmp val )					\
            return;                                                     \
        srl_task_s *current = context_to_srl_task(sched_get_current()); \
        current->wait_val = val;                                        \
        current->wait_addr = addr;                                      \
        sched_context_candidate_fcn(&current->context, wait_##name##endianness##_f); \
        cpu_interrupt_disable();                                        \
        sched_context_switch();                                         \
        cpu_interrupt_enable();                                         \
        sched_context_candidate_fcn(&current->context, NULL);           \
    }

DECLARE_WAIT(le, eq, ==)
DECLARE_WAIT(le, ne, !=)
DECLARE_WAIT(le, le, <=)
DECLARE_WAIT(le, ge, >=)
DECLARE_WAIT(le, lt, <)
DECLARE_WAIT(le, gt, >)

DECLARE_WAIT(be, eq, ==)
DECLARE_WAIT(be, ne, !=)
DECLARE_WAIT(be, le, <=)
DECLARE_WAIT(be, ge, >=)
DECLARE_WAIT(be, lt, <)
DECLARE_WAIT(be, gt, >)

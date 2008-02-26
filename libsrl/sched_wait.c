#include <stddef.h>
#include <hexo/scheduler.h>
#include <srl_private_types.h>
#include <srl/srl_sched_wait.h>

static inline srl_task_s *context_to_srl_task( struct sched_context_s *ctx )
{
    uintptr_t p = (uintptr_t)ctx;
    p -= offsetof(srl_task_s, context);
    return (srl_task_s *)p;
}

#define DECLARE_WAIT(name, cmp)                                         \
                                                                        \
    static SCHED_CANDIDATE_FCN(wait_##name##_f)                         \
    {                                                                   \
        srl_task_s *task = context_to_srl_task(sched_ctx);              \
                                                                        \
        cpu_dcache_invld(task->wait_addr);                              \
        return (*task->wait_addr cmp task->wait_val);                   \
    }                                                                   \
                                                                        \
    void srl_sched_wait_##name( uint32_t*addr, uint32_t val )           \
    {                                                                   \
        if ( *addr cmp val )                                            \
            return;                                                     \
        srl_task_s *current = context_to_srl_task(sched_get_current()); \
        current->wait_val = val;                                        \
        current->wait_addr = addr;                                      \
        sched_context_candidate_fcn(&current->context, wait_##name##_f); \
        cpu_interrupt_disable();                                        \
        sched_context_switch();                                         \
        cpu_interrupt_enable();                                         \
        sched_context_candidate_fcn(&current->context, NULL);           \
    }

DECLARE_WAIT(eq, ==)
DECLARE_WAIT(ne, !=)
DECLARE_WAIT(le, <=)
DECLARE_WAIT(ge, >=)
DECLARE_WAIT(lt, <)
DECLARE_WAIT(gt, >)

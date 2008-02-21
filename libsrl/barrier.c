
#include <hexo/scheduler.h>
#include <hexo/error.h>
#include <assert.h>
#include <srl/srl_public_types.h>
#include <srl_private_types.h>

void srl_barrier_wait(srl_barrier_t barrier)
{
	CPU_INTERRUPT_SAVESTATE_DISABLE;
	sched_queue_wrlock(&barrier->wait);

	assert(barrier->count >= 1);

	if (barrier->count == 1)
    {
		while (sched_wake(&barrier->wait) != NULL)
			barrier->count++;
		sched_queue_unlock(&barrier->wait);
    }
	else
    {
		barrier->count--;
		sched_wait_unlock(&barrier->wait);
    }

	CPU_INTERRUPT_RESTORESTATE;
}


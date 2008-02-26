#ifndef SRL_SCHED_WAIT_H
#define SRL_SCHED_WAIT_H

#include <stdint.h>

#define DECLARE_WAIT(name, cmp)											\
	void srl_sched_wait_##name( uint32_t*addr, uint32_t val );


DECLARE_WAIT(eq, ==)
DECLARE_WAIT(ne, !=)
DECLARE_WAIT(le, <=)
DECLARE_WAIT(ge, >=)
DECLARE_WAIT(lt, <)
DECLARE_WAIT(gt, >)

#undef DECLARE_WAIT

#endif

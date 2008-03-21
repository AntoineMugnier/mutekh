#ifndef SRL_SCHED_WAIT_H
#define SRL_SCHED_WAIT_H

#include <stdint.h>

#define DECLARE_WAIT(endianness, name, cmp)								\
	void srl_sched_wait_##name##_##endianness( uint32_t*addr, uint32_t val );


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

#undef DECLARE_WAIT

#endif

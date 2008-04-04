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

DECLARE_WAIT(cpu, eq, ==)
DECLARE_WAIT(cpu, ne, !=)
DECLARE_WAIT(cpu, le, <=)
DECLARE_WAIT(cpu, ge, >=)
DECLARE_WAIT(cpu, lt, <)
DECLARE_WAIT(cpu, gt, >)

#undef DECLARE_WAIT

typedef int8_t srl_callback_t( uint32_t val );

void srl_sched_wait_priv( srl_callback_t *cb, uint32_t val );

#endif

#ifndef SRL_SCHED_WAIT_H
#define SRL_SCHED_WAIT_H

/**
 * @file
 * @module{SRL}
 * @short Smart waiting tools
 */

#include <stdint.h>

#define DECLARE_WAIT(endianness, name, cmp)								\
	void srl_sched_wait_##name##_##endianness( volatile uint32_t*addr, uint32_t val );

/**
   @multiple{18}
   @this is of the form:

   @code
   srl_sched_wait_CMP_END(addr, val)
   @end code

   where @em CMP can be eq, ne, le, ge, lt, or gt, and @em END can be le, be, or cpu.

   They make the current task sleep until the valut pointed at @tt addr
   asserts the following test:
   @code
   (*addr CMP val)
   @end code

   @tt addr is taken with the END endianness (little, big, or the currrent cpu's one)

   @param addr The address to poll
   @param val The value to compare to
*/
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

/**
 @internal
*/
typedef int8_t srl_callback_t( uint32_t val );

/**
 @internal
*/
void srl_sched_wait_priv( srl_callback_t *cb, uint32_t val );

#endif

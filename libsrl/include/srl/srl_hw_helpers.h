#ifndef SRL_HW_HELPERS_H
#define SRL_HW_HELPERS_H

#include <hexo/types.h>
#include <hexo/cpu.h>
#include <stdlib.h>

#define BASE_ADDR_OF(id)											   \
	({																   \
		extern __ldscript_symbol_t _dsx_##id##_region_begin;		   \
		(void*)&_dsx_##id##_region_begin;								   \
	})

#define srl_busy_cycles(n) do{}while(0)
#define srl_dcache_flush_addr cpu_dcache_invld
#define srl_dcache_flush_zone cpu_dcache_invld_buf

void srl_sleep_cycles( uint32_t n );

static inline uint32_t srl_cycle_count()
{
	return cpu_cycle_count();
}

static inline void srl_abort()
{
	abort();
}

#endif

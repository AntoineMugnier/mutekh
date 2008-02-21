#ifndef SRL_HW_HELPERS_H
#define SRL_HW_HELPERS_H

#include <hexo/types.h>
#include <hexo/cpu.h>

#define BASE_ADDR_OF(id)											   \
	({																   \
		extern __ldscript_symbol_t _dsx_##id##_base_addr;			   \
		_dsx_##id##_base_addr;										   \
	})

#define srl_busy_cycles(n) do{}while(0)
#define srl_dcache_flush_addr cpu_dcache_invld
#define srl_dcache_flush_zone cpu_dcache_invld_buf

#endif

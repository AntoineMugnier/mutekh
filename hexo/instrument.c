
#include <hexo/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <hexo/alloc.h>

void __cyg_profile_func_enter (void *this_fn,
			       void *call_site);

void __cyg_profile_func_exit  (void *this_fn,
			       void *call_site);

static bool_t hexo_instrument_trace_flag = 0;

#ifdef CONFIG_HEXO_MEMALLOC_GUARD_INSTRUMENT
static bool_t hexo_instrument_memalloc_guard = 0;
#endif

__attribute__ ((no_instrument_function))
void __cyg_profile_func_enter (void *this_fn,
			       void *call_site)
{
  if (hexo_instrument_trace_flag)
    {
      hexo_instrument_trace_flag = 0;
      printf(">>>>> [f:%p]   Called from [f:%p]\n", this_fn, call_site);
      hexo_instrument_trace_flag = 1;
    }

#ifdef CONFIG_HEXO_MEMALLOC_GUARD_INSTRUMENT
  if (hexo_instrument_memalloc_guard && mem_guard_check())
    {
      printf("Memory guard check failed on function call [f:%p] called from [f:%p]",
	     this_fn, call_site);
      abort();
    }
#endif
}

__attribute__ ((no_instrument_function))
void __cyg_profile_func_exit  (void *this_fn,
			       void *call_site)
{
#ifdef CONFIG_HEXO_MEMALLOC_GUARD_INSTRUMENT
  if (hexo_instrument_memalloc_guard && mem_guard_check())
    {
      printf("Memory guard check failed on function return [f:%p] called from [f:%p]",
	     this_fn, call_site);
      abort();
    }
#endif

  if (hexo_instrument_trace_flag)
    {
      hexo_instrument_trace_flag = 0;
      printf("  <<< [f:%p]   Called from [f:%p]\n", this_fn, call_site);
      hexo_instrument_trace_flag = 1;
    }
}

void
hexo_instrument_trace(bool_t state)
{
  hexo_instrument_trace_flag = state;  
}

void
hexo_instrument_alloc_guard(bool_t state)
{
#ifdef CONFIG_HEXO_MEMALLOC_GUARD_INSTRUMENT
  hexo_instrument_trace_flag = state;  
#endif
}


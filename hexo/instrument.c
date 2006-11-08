
#include <hexo/types.h>
#include <stdio.h>

void __cyg_profile_func_enter (void *this_fn,
			       void *call_site);

void __cyg_profile_func_exit  (void *this_fn,
			       void *call_site);

static bool_t hexo_instrument_trace_flag = 0;

__attribute__ ((no_instrument_function))
void __cyg_profile_func_enter (void *this_fn,
			       void *call_site)
{
  if (hexo_instrument_trace_flag)
    {
      hexo_instrument_trace_flag = 0;
      printf(">>> f:%p Called from f:%p\n", this_fn, call_site);
      hexo_instrument_trace_flag = 1;
    }
}

__attribute__ ((no_instrument_function))
void __cyg_profile_func_exit  (void *this_fn,
			       void *call_site)
{
  if (hexo_instrument_trace_flag)
    {
      hexo_instrument_trace_flag = 0;
      printf("<<< f:%p Called from f:%p\n", this_fn, call_site);
      hexo_instrument_trace_flag = 1;
    }
}

void
hexo_instrument_trace(bool_t state)
{
  hexo_instrument_trace_flag = state;  
}


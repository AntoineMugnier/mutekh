#include <mutek/mem_alloc.h>
#include <mutek/mem_region.h>
#include <mutek/memory_allocator.h>

extern __ldscript_symbol_t __system_uncached_heap_start, __system_uncached_heap_end;

#define INIT_RAM(ram_tab, nb, s, e, cl, ca) do {                        \
    ram_tab[nb].start = s;                                              \
    ram_tab[nb].end = e;                                                \
    ram_tab[nb].cluster = cl;                                           \
    ram_tab[nb].cached = ca;                                            \
  }while(0);                                                            \

struct ram_desc_s
{
  void *start;
  void *end;
  uint32_t cluster;
  bool_t cached;
  void *desc;
};

struct ram_desc_s ram_tab[2];

void mem_init(void)
{
  uint_fast16_t ram = 0;

  INIT_RAM(ram_tab, 0,
	   &__system_uncached_heap_start,
	   (void*)((uintptr_t)&__system_uncached_heap_end - 1024 * CONFIG_CPU_MAXCOUNT),
	   0,
	   1);
  INIT_RAM(ram_tab, 1, 0x6f000000, 0x70000000, 1, 1);

  for (ram=0; ram<2; ram++)
    {
      ram_tab[ram].desc = memory_allocator_init(NULL, ram_tab[ram].start, ram_tab[ram].end);
    }

  default_region = ram_tab[0].desc;
}

void mem_region_init(void)
{
  uint_fast16_t scope,ram = 0;
  cpu_id_t cpu = 0;
  for (cpu=0; cpu<4; cpu++)
    mem_region_id_init(cpu);

  for (ram=0; ram<2 ; ram++) {
    for (cpu=0; cpu<4; cpu++)
      {
        for (scope=0; scope<mem_scope_e_count; scope++)
          {
            switch(scope)
              {
              case mem_scope_sys:
                if (ram == 0)
                  mem_region_id_add(cpu, scope, ram_tab[ram].desc, 0);
                else
                  mem_region_id_add(cpu, scope, ram_tab[ram].desc, 200);
                break;
              
              case mem_scope_cluster:
              case mem_scope_default:
                if (ram_tab[ram].cluster == (cpu>>1))
                  mem_region_id_add(cpu, scope, ram_tab[ram].desc, 0);
                else
                  mem_region_id_add(cpu, scope, ram_tab[ram].desc, 200);
                break;
		  
              case mem_scope_cpu:
                if (ram_tab[ram].cluster == (cpu>>1))
                  mem_region_id_add(cpu, scope, ram_tab[ram].desc, 0);
                else
                  mem_region_id_add(cpu, scope, ram_tab[ram].desc, 200);
                break;
		  
              case mem_scope_context:
                if (ram_tab[ram].cluster == (cpu>>1))
                  mem_region_id_add(cpu, scope, ram_tab[ram].desc, 0);
                else
                  mem_region_id_add(cpu, scope, ram_tab[ram].desc, 200);
                break;
		  
              }
          }
      }
  }
  default_region = NULL;
}

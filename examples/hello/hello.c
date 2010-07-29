
#include <pthread.h>
#include <mutek/printk.h>
#include <arch/lm3s/lm3s8962.h>

#define THREAD_COUNT 4

pthread_mutex_t m;
pthread_t pthread[THREAD_COUNT];

void *f(void *param)
{
  while (1)
    { 
      pthread_mutex_lock(&m);
      printk("(%s:%i) %s", cpu_type_name(), cpu_id(), param);
      pthread_mutex_unlock(&m);
      //      cpu_cycle_wait(10000);
      size_t i;
      for(i=0; i<2000000; i++)
        asm volatile("nop");
      uint32_t pf0 = ~LM3S_BASE_GPIOF->GPIO_DATAR[1];
      LM3S_BASE_GPIOF->GPIO_DATAR[1] = pf0;
      pthread_yield();
    }
}

void app_start()
{
  size_t i;

  pthread_mutex_init(&m, NULL);
  for ( i = 0; i < THREAD_COUNT; ++i )
    pthread_create(&pthread[i], NULL, f, "Hello world\n");
}


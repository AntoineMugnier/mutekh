
#include <pthread.h>
#include <mutek/printk.h>
#include <hexo/power.h>

#define THREAD_COUNT 4
#define TEST_CYCLES 100

pthread_mutex_t m;
pthread_t pthread[THREAD_COUNT];

size_t count = 0;

void *f(void *param)
{
  int i;
  for (i = 0; i < TEST_CYCLES; i++)
    { 
      pthread_mutex_lock(&m);
      printk("(%i:%i) %s", i, cpu_id(), param);
      count++;
      pthread_mutex_unlock(&m);

      if (rand() % 2)
        pthread_yield();
    }

  return NULL;
}

void main()
{
  size_t i;

  pthread_mutex_init(&m, NULL);

  for ( i = 0; i < THREAD_COUNT; ++i )
    pthread_create(&pthread[i], NULL, f, "Hello world\n");

  for ( i = 0; i < THREAD_COUNT; ++i )
    pthread_join(pthread[i], NULL);

  printk("++SUCCESS++%i++\n", count);

  power_shutdown();
  power_reboot();
}


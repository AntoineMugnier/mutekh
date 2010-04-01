
#include <pthread.h>
#include <mutek/printk.h>
#include <hexo/interrupt.h>

#define N 8

pthread_spinlock_t m;
pthread_t a[N];
volatile int d;
void *f(void *param)
{
  while (1)
    {
      pthread_spin_lock(&m);
      printk("(%s:%i) %i\n", cpu_type_name(), cpu_id(), (uintptr_t)param);
      pthread_spin_unlock(&m);
      //      *(uint32_t*)(0x5a) = 0;
      //      pthread_yield();
    }
}

void app_start()
{
  uint_fast8_t i;
  pthread_spin_init(&m, 0);
  for (i = 0; i < N; i++)
    pthread_create(&a[i], NULL, f, i);
}


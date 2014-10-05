
#include <pthread.h>
#include <mutek/printk.h>

#define THREAD_COUNT 4

pthread_mutex_t m;

void main()
{
  pthread_mutex_init(&m, NULL);
  pthread_mutex_lock(&m);

  printk("(%s:%i) %s\n", cpu_type_name(), cpu_id(), "Hello World!");

  pthread_mutex_unlock(&m);

  pthread_yield();
}


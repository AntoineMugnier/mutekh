
#include <pthread.h>
#include <mutek/printk.h>

pthread_mutex_t m;
pthread_t a, b;

void *f(void *param)
{
  while (1)
    { 
      pthread_mutex_lock(&m);
      printk("(%s:%i) %s", cpu_type_name(), cpu_id(), param);
      pthread_mutex_unlock(&m);
      pthread_yield();
    }
}

void app_start()
{
  switch (cpu_id())
    {
    case 0:
    case 1:
      pthread_mutex_init(&m, NULL);
      pthread_create(&a, NULL, f, "Hello\n");
      break;

    case 2:
    case 3:
      pthread_create(&b, NULL, f, "World\n");
      break;
    }
}


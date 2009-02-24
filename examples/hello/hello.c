
#include <pthread.h>

pthread_mutex_t m;
pthread_t a, b;

void *f(void *param)
{
  while (1)
    { 
      pthread_mutex_lock(&m);
      printk("(%i) %s", cpu_id(), param);
      pthread_mutex_unlock(&m);
      pthread_yield();
    }
}
int main()
{
  pthread_mutex_init(&m, NULL);
  pthread_create(&a, NULL, f, "Hello ");
  pthread_create(&b, NULL, f, "World\n");
}


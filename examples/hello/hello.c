
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
//	*(uint32_t*)0xdeadbeef = 42;
//	asm volatile("swi 3");
	asm volatile(".word ");
	return 0;
  pthread_mutex_init(&m, NULL);
  pthread_create(&a, NULL, f, "Hello world\n");
  pthread_create(&b, NULL, f, "Hello world\n");
}


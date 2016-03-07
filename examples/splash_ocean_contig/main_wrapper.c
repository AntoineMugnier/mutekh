#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <mutek/printk.h>
#include <mutek/startup.h>

pthread_t PThreadWrapper;

extern int main(void);

void app_start(void)
{
  if( cpu_id() == 0)
    {
      pthread_create(&PThreadWrapper, NULL, (void * (*)(void *))(main),NULL);
    }
}

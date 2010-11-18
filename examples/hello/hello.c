
#include <hexo/init.h>
#include <hexo/types.h>
#include <pthread.h>
#include <mutek/printk.h>
#include <arch/lm3s/lm3s8962.h>
#include <drivers/icu/cm3/icu-cm3.h>
#include <drivers/icu/arm/icu-arm.h>

#include <device/device.h>
#include <device/driver.h>

#define THREAD_COUNT 4

pthread_mutex_t m;
pthread_t pthread[THREAD_COUNT];

struct device_s icu_dev;
struct device_s cpu_icu_dev;


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

void say_yo(void *data){
  printk("YO !!!\n");
  LM3S_BASE_GPIOF->GPIO_ICRR = (1<<1); // clear interrupt
}

void app_start()
{
  size_t i;
  device_init(&cpu_icu_dev);
  icu_arm_init(&cpu_icu_dev,NULL);

  cpu_interrupt_sethandler_device(&cpu_icu_dev);

  device_init(&icu_dev);
  icu_dev.addr[0] = (uintptr_t)CM3_BASE_NVIC;
  icu_dev.irq=0;
  icu_dev.icudev = CPU_LOCAL_ADDR(cpu_icu_dev);
  icu_cm3_init(&icu_dev, NULL);
  
  dev_icu_sethndl(&icu_dev,30,say_yo,NULL);
  dev_icu_enable(&icu_dev,30,1,0);

  cpu_interrupt_enable();

  pthread_mutex_init(&m, NULL);
  //  for ( i = 0; i < THREAD_COUNT; ++i )
  pthread_create(&pthread[0], NULL, f, "Hello world, Thread 0\n");
  pthread_create(&pthread[1], NULL, f, "Hello world, Thread 1\n");
  pthread_create(&pthread[2], NULL, f, "Hello world, Thread 2\n");
  pthread_create(&pthread[3], NULL, f, "Hello world, Thread 3\n");
}


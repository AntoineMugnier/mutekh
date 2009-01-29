
#include <hexo/interrupt.h>
#include <drivers/device/icu/8259/icu-8259.h>
#include <drivers/device/block/ata/block-ata.h>
#include <hexo/device.h>
#include <device/driver.h>

#include <stdio.h>

#include <hexo/context.h>
#include <mutek/scheduler.h>

struct sched_context_s a, b, c;
struct device_s *drv[2];
extern struct device_s icu_dev;
struct device_s ata;


static CONTEXT_ENTRY(a_entry)
{
  ssize_t i = -1;
  sched_unlock();

  //  hexo_instrument_trace(1);

  while (i--)
    {
      uint8_t	_data[1024];
      uint8_t	*data[2];
      //uint_fast8_t d = rand() % 2;
      dev_block_lba_t l = ((rand() % 45) << 16) + rand() % 65535;

      printf("(START d%i c%i %s %p)", 1, cpu_id(), param, &sched_get_current()->context);

      cpu_interrupt_enable();

      data[0] = _data;
      data[1] = _data + 512;

      dev_block_wait_read(drv[0], data, l, 2);

      cpu_interrupt_disable();

      printf("(DATA c%i %s %P)", cpu_id(), param, data[0], 2);

      sched_context_switch();
    }

  while(1);
}

int main()
{
  static reg_t stack_bufa[10240];
  static reg_t stack_bufb[10240];
  static reg_t stack_bufc[10240];

  device_init(&ata);

  ata.addr[0] = 0x1f0;
  ata.addr[1] = 0x3f0;
  ata.irq = 14;

  controller_ata_init(&ata, &icu_dev, NULL);
  DEV_ICU_BIND(&icu_dev, &ata);

  drv[0] = device_get_child(&ata, 0);
  //  drv[1] = device_get_child(&ata, 1);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  context_init(&a.context, stack_bufa, stack_bufa + 10240, a_entry, "A");
  sched_context_init(&a);
  sched_context_start(&a);

  context_init(&b.context, stack_bufb, stack_bufb + 10240, a_entry, "B");
  sched_context_init(&b);
  sched_context_start(&b);

  context_init(&c.context, stack_bufc, stack_bufc + 10240, a_entry, "C");
  sched_context_init(&c);
  sched_context_start(&c);
  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}


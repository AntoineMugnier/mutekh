#include <mutek/startup.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#define BC_CUSTOM_PRINTI 0x1000

extern const struct bc_descriptor_s test_bytecode;

void app_start(void)
{
  struct bc_context_s vm;

  bc_init(&vm, &test_bytecode);

  /* provide a buffer for use by our bytecode program */
  uintptr_t buffer[32];
  bc_set_reg(&vm, 0, (uintptr_t)buffer);

  while (1)
    {
      uint16_t r = bc_run(&vm);

      switch (r)
        {
        case BC_RUN_STATUS_END:
          printk("bytecode done");
          return;

        case BC_RUN_STATUS_FAULT:
          printk("bytecode error");
          bc_dump(&vm, 1);
          return;

        default:
          break;
        }

      switch (r & 0x7000)
        {
        case BC_CUSTOM_PRINTI:
          printk("%u\n", bc_get_reg(&vm, r & 0xf));
          break;
        default:
          printk("bad custom bytecode opcode\n");
          return;
        }
    }
}

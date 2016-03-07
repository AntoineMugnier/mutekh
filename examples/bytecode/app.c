#include <mutek/startup.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#define BC_CUSTOM_PRINTI 0x1000
#define BC_CUSTOM_PRINTS 0x2000
#define BC_CUSTOM_SKIPODD 0x3000

static BC_CCALL_FUNCTION(c_func)
{
    return dst * 13;
}

extern const struct bc_descriptor_s test_bytecode;

void app_start(void)
{
  struct bc_context_s vm;
  char buf[128] = "testbarx";

  bc_init(&vm, &test_bytecode, 2, buf, &c_func);
  //  bc_set_addr_range(&vm, (uintptr_t)buf, (uintptr_t)buf + sizeof(buf) - 1);
  //  bc_dump(&vm);

  while (1)
    {
      uint16_t r = bc_run(&vm, -1);

      if (!r)
        {
          printk("done\n");
          return;  /* terminate */
        }
      if (!(r & 0x8000))
        {
          printk("bytecode execution error\n");
          return; /* error */
        }

      switch (r & 0x7000)
        {
        case BC_CUSTOM_PRINTI:
          printk("%u\n", bc_get_reg(&vm, r & 0xf));
          break;
        case BC_CUSTOM_PRINTS:
          printk("%s\n", (char*)bc_get_reg(&vm, r & 0xf));
          break;
        case BC_CUSTOM_SKIPODD:
          if (r & 1)
            bc_skip(&vm);
          break;
        default:
          printk("bad custom bytecode opcode\n");
          return;
        }
    }

  while (1)
   ;
}

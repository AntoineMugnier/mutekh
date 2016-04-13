
#include <hexo/power.h>

#include <mutek/printk.h>
#include <mutek/bytecode.h>

#define BC_CUSTOM_PRINTI 0x1000
#define BC_CUSTOM_PRINTS 0x2000
#define BC_CUSTOM_SKIPODD 0x3000

uint32_t cksum = 5381;

static void cksum_update(uint32_t x)
{
  cksum = (cksum * 33) ^ x;
}

static BC_CCALL_FUNCTION(c_func)
{
  cksum_update(dst);
  return dst * 13;
}

extern const struct bc_descriptor_s test_bytecode;

void app_start()
{
  struct bc_context_s vm;
  char buf[128] = "testbarx";

  bc_init(&vm, &test_bytecode);
  bc_set_regs(&vm, 0b11, buf, &c_func);

#ifdef CONFIG_MUTEK_BYTECODE_CHECKING
  bc_set_addr_range(&vm, (uintptr_t)buf, (uintptr_t)buf + sizeof(buf) - 1);
  bc_sandbox(&vm, 1);
#endif
  //  bc_set_trace(&vm, 1, 1);
  //  bc_dump(&vm, 1);

  while (1)
    {
      uint16_t r = bc_run(&vm, -1);

      if (!r)
        {
          printk("++SUCCESS++%08x++\n", cksum);
          break;
        }
      if (!(r & 0x8000))
        {
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
          bc_dump(&vm, 1);
#endif
          printk("bytecode execution error\n");
          break; /* error */
        }

      switch (r & 0x7000)
        {
        case BC_CUSTOM_PRINTI: {
          uintptr_t x = bc_get_reg(&vm, r & 0xf);
          printk("%u\n", x);
          cksum_update(x);
          break;
        }
        case BC_CUSTOM_PRINTS: {
          const char *s = (char*)bc_get_reg(&vm, r & 0xf);
          printk("%s\n", s);
          while (*s)
            cksum_update(*s++);
          break;
        }
        case BC_CUSTOM_SKIPODD: {
          cksum_update(r & 1);
          if (r & 1)
            bc_skip(&vm);
          break;
        }
        default:
          printk("bad custom bytecode opcode\n");
          goto done;
        }
    }

 done:
  power_shutdown();
  power_reboot();
}

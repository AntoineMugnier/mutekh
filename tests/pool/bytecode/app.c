
#include <hexo/power.h>

#include <mutek/printk.h>
#include <mutek/bytecode.h>
#include <mutek/startup.h>

#define BC_CUSTOM_PRINTI 0x1000
#define BC_CUSTOM_PRINTS 0x2000
#define BC_CUSTOM_SKIPODD 0x3000
#define BC_CUSTOM_LOADSTR 0x4000
#define BC_CUSTOM_CHECKSTR 0x5000

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

extern bytecode_entry_t test_bytecode_entry;

void app_start()
{
  struct bc_context_s vm;
  char buf[128] = "testbarx";

  bc_init(&vm, &test_bytecode);
  bc_set_regs(&vm, 0b11, buf, &c_func);
  bc_set_pc(&vm, &test_bytecode_entry);

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
        case BC_CUSTOM_LOADSTR: {
          uint8_t *t = (uint8_t*)(vm.v + (r & 0xf));
          uint_fast8_t i, c = (r & 0x3f0) >> 4;
          for (i = 0; i < c; i++)
            t[i] = i;
          break;
        }
        case BC_CUSTOM_CHECKSTR: {
          uint8_t *t = (uint8_t*)(vm.v + (r & 0xf));
          uint_fast8_t i, c = (r & 0x3f0) >> 4;
          for (i = 0; i < c; i++)
            if (t[i] != i)
              {
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
                bc_dump(&vm, 1);
#endif
                printk("bad string %P\n", t, c);
                goto done;
              }
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

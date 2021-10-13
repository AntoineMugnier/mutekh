
#include <hexo/power.h>

#include <mutek/printk.h>
#include <mutek/bytecode.h>
#include <mutek/startup.h>

#include "test.o.h"

#define BC_CUSTOM_PRINTI 0x1000
#define BC_CUSTOM_PRINTS 0x2000
#define BC_CUSTOM_SKIPODD 0x3000
#define BC_CUSTOM_LOADSTR 0x4000
#define BC_CUSTOM_CHECKSTR 0x5000
#define BC_CUSTOM_PRINTH 0x6000

uint32_t cksum = 5381;

static void cksum_update(uint32_t x)
{
  cksum = (cksum * 33) ^ x;
}

#ifndef CONFIG_MUTEK_BYTECODE_SANDBOX
static BC_CCALL_FUNCTION(c_func)
{
  assert(ctx->mode == 4);
  bc_reg_t r = bc_get_reg(ctx, 2);
  cksum_update(r);
  bc_set_reg(ctx, 2, r * 13);
}
#endif

extern const struct bc_descriptor_s test_bytecode;

extern bytecode_entry_t test_bytecode_entry;

void app_start()
{
  struct bc_context_s vm;
  __attribute__((aligned(8)))
  char buf[128] = "testbarx";

#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
  bc_init_sandbox(&vm, &test_bytecode, buf, 7, -1);
  bc_set_regs(&vm, 0b1, /* buf */ 0x80000000);
#else
  bc_init(&vm, &test_bytecode);
  bc_set_regs(&vm, 0b11, buf, &c_func);
#endif

  for (uint_fast8_t i = 2; i < 16; i++)
    bc_set_reg(&vm, i, 0x5a5a5a5a);
  bc_set_pc(&vm, &test_bytecode_entry);

  //  bc_set_trace(&vm, BC_TRACE_ALL);
  //  bc_dump(&vm, 1);

  while (1)
    {
      uint16_t r = bc_run(&vm);

      if (!r)
        {
          assert(vm.mode == 2);
          printk("++SUCCESS++%08x++\n", cksum);
          break;
        }
      if (!(r & 0x8000))
        {
#ifdef CONFIG_MUTEK_BYTECODE_DEBUG
          bc_dump(&vm, 1);
#endif
          printk("bytecode execution error at %p\n", bc_get_pc(&vm));
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
          uintptr_t addr = bc_get_reg(&vm, r & 0xf);
#ifdef CONFIG_MUTEK_BYTECODE_SANDBOX
          const char *s = bc_translate_addr(&vm, addr, 0, 0);
#else
          const char *s = (char*)addr;
#endif
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
        case BC_CUSTOM_PRINTH: {
          uint8_t *t = (uint8_t*)(vm.v + (r & 0xf));
          uint_fast8_t c = (r & 0x3f0) >> 4;
          printk("%P\n", t, c);
          while (c--)
            cksum_update(*t++);
          break;
        }
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

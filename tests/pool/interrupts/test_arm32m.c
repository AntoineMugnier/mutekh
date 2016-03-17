
#include <mutek/printk.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <hexo/power.h>

volatile int ok = 0;
volatile int step = 0;
volatile uintptr_t stack;

int fault1(int);
int fault2(int);
uintptr_t fault3(int);
int syscall();
void end_test();

asm(
".text \n"
"fault1:"
    "add      r0, r0, #1 \n"
    "udf                  \n" // fault
    "add      r0, r0, #41 \n"
    "mov      pc, lr \n"
"fault2:"
    "add      r0, r0, #1 \n"
    "udf                  \n" // fault
    "add      r0, r0, #41 \n"
    "add      r0, r0, #17 \n"
    "mov      pc, lr \n"
"fault3:"
    "udf                  \n" // fault
    "mov      r0,  r13   \n"
    "mov      pc, lr \n"
);

static CPU_EXCEPTION_HANDLER(fault_handler)
{
  printk("fault %i %i\n", type, stackptr);

  switch (step++)
    {
    case 0:
      if (type == CPU_EXCEPTION_HARDFAULT &&
          execptr == (uintptr_t)&fault1 + 2)
        ok |= 1;
      return;

    case 1:
      if (type == CPU_EXCEPTION_HARDFAULT &&
          execptr == (uintptr_t)&fault1 + 2)
        ok |= 2;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault1 + 4);
      return;

    case 2:
      if (execptr == (uintptr_t)&fault2 + 2)
        ok |= 8;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault2 + 6);
      return;

    case 3:
      if (execptr == (uintptr_t)&fault3)
        stack = stackptr;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault3 + 2);
      return;

#ifdef CONFIG_HEXO_USERMODE
    case 7:
      if (type == CPU_EXCEPTION_HARDFAULT &&
          execptr == (uintptr_t)&syscall + 20 &&
          stackptr == 1234+33)
        ok |= 512;
      cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 22);
      return;
#endif
    }
}

#ifdef CONFIG_HEXO_USERMODE

asm(
".text \n"
"syscall:"
    "add    r0, r0, #17 \n"
    "svc    0           \n"
    "add    r0, r0, #7  \n"
    "add    r0, r0, #5  \n"
    "svc    0           \n"
    "add    r0, r0, #3  \n"     // skipped
    "add    r0, r0, #8  \n"
    "add    sp, sp, #33 \n" // bad user stack
    "svc    0           \n"
    "nop                \n"
    "udf                \n" // user fault
    "add    r0, r0, #11  \n"
    "svc    0           \n"
);

static CPU_SYSCALL_HANDLER(syscall_handler)
{
  printk("syscall %i, %i\n", number, regs->gpr[0]);

  switch (step++)
    {
      case 4:
        if (number == 0 && regs->gpr[0] == 351+17) // syscall arg
          ok |= 64;
        return;

      case 5:
        if (number == 0 && regs->gpr[0] == 351+17+7+5) // syscall arg
          ok |= 128;
        cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 12);
        return;

      case 6:
        if (number == 0 && regs->gpr[0] == 351+17+7+5+8) // syscall arg
          ok |= 256;
        regs->gpr[0] = 117;
        return;

      case 8:
        if (number == 0 && regs->gpr[0] == 117+11) // syscall arg
          ok |= 1024;
        end_test();
    }

}
#endif

void main()
{
  cpu_exception_sethandler(fault_handler);

  if (fault1(0) == 42)
    ok |= 4;
  if (fault2(0) == 18)
    ok |= 16;

  if (fault3(0) == stack)
    ok |= 32;

#ifdef CONFIG_HEXO_USERMODE
  cpu_syscall_sethandler(syscall_handler);
  cpu_context_set_user(1234, (uintptr_t)&syscall, 351);
#else
  ok |= 64 | 128 | 256 | 512 | 1024;
#endif

  end_test();
}

void end_test()
{
  if (ok == 2047)
    printk("++SUCCESS++%x++\n", ok);
  else
    printk("++FAIL++%x++\n", ok);

  power_shutdown();
  power_reboot();

  while (1)
    ;
}


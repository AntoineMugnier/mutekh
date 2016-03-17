
#include <mutek/printk.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <hexo/power.h>

volatile int ok = 0;
volatile int step = 0;
volatile uintptr_t stack;

int fault1();
int fault2();
uintptr_t fault3();
int syscall();
void end_test();

asm(
".text \n"
"fault1:"
    "addi     3,  0,  1 \n"
    "lwz      0,  1(0)  \n" // bad align
    "addi     3,  3, 41 \n"
    "blr                \n"
"fault2:"
    "addi     3,  0,  1 \n"
    "lwz      0,  1(0)  \n" // bad align
    "addi     3,  3, 41 \n"
    "addi     3,  3, 17 \n"
    "blr                \n"
"fault3:" 
    "lwz      0,  1(0)  \n" // bad align
    "mr       3,  1     \n"
    "blr                \n"
);

static CPU_EXCEPTION_HANDLER(fault_handler)
{

  switch (step++)
    {
    case 0:
      if (type == CPU_EXCEPTION_DATA_ALIGN &&
          execptr == (uintptr_t)&fault1 + 4)
        ok |= 1;
      return;

    case 1:
      if (type == CPU_EXCEPTION_DATA_ALIGN &&
          execptr == (uintptr_t)&fault1 + 4)
        ok |= 2;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault1 + 8);
      return;

    case 2:
      if (execptr == (uintptr_t)&fault2 + 4)
        ok |= 8;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault2 + 12);
      return;

    case 3:
      if (execptr == (uintptr_t)&fault3)
        stack = stackptr;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault3 + 4);
      return;

#ifdef CONFIG_HEXO_USERMODE
    case 7:
      if (type == CPU_EXCEPTION_ILLEGAL_INS &&
          execptr == (uintptr_t)&syscall + 40 &&
          stackptr == 1234+33)
        ok |= 512;
      cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 44);
      return;
#endif
    }
}

#ifdef CONFIG_HEXO_USERMODE

asm(
".text \n"
"syscall:"
    "addi    3, 3, 17 \n"
    "sc               \n"
    "addi    3, 3, 7  \n"
    "addi    3, 3, 5  \n"
    "sc               \n"
    "addi    3, 3, 3  \n"     // skipped
    "addi    3, 3, 8  \n"
    "addi    1, 1, 33 \n" // bad user stack
    "sc               \n"
    "nop              \n"
    "mtmsr   0        \n" // user fault
    "addi    3, 3, 11  \n"
    "sc               \n"
);

static CPU_SYSCALL_HANDLER(syscall_handler)
{
  printk("syscall %i\n", regs->gpr[3]);

  switch (step++)
    {
      case 4:
        if (number == 0 && regs->gpr[3] == 351+17) // syscall arg
          ok |= 64;
        return;

      case 5:
        if (number == 0 && regs->gpr[3] == 351+17+7+5) // syscall arg
          ok |= 128;
        cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 24);
        return;

      case 6:
        if (number == 0 && regs->gpr[3] == 351+17+7+5+8) // syscall arg
          ok |= 256;
        regs->gpr[3] = 117;
        return;

      case 8:
        if (number == 0 && regs->gpr[3] == 117+11) // syscall arg
          ok |= 1024;
        end_test();
    }

}
#endif

void main()
{
  cpu_exception_sethandler(fault_handler);

  if (fault1() == 42)
    ok |= 4;
  if (fault2() == 18)
    ok |= 16;

  if (fault3() == stack)
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
    printk("++SUCCESS++%i++\n", ok);
  else
    printk("++FAIL++%x++\n", ok);

  power_shutdown();
  power_reboot();
}


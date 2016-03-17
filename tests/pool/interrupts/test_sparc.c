
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
    "add      %g0, 1, %o0  \n"
    "ld       [%g0 + 1], %g1 \n" // bad align
    "add      %o0, 41, %o0 \n"
    "retl                  \n"
    "nop                   \n"
"fault2:"
    "add      %g0, 1, %o0  \n"
    "b        1f \n"
    "ld       [%g0 + 1], %g1 \n" // bad align in delay slot
    "add      %o0, 1, %o0 \n"
"1: \n"
    "add      %o0, 42, %o0 \n"
    "retl                  \n"
    "nop                   \n"
"fault3:"
    "ld       [%g0 + 1], %g1 \n" // bad align
    "retl                  \n"
    "mov      %sp, %o0     \n" // return stack pointer
);

static CPU_EXCEPTION_HANDLER(fault_handler)
{

  switch (step++)
    {
    case 0:
      if (type == CPU_EXCEPTION_BAD_ALIGN &&
          execptr == (uintptr_t)&fault1 + 4)
        ok |= 1;
      return;

    case 1:
      if (execptr == (uintptr_t)&fault1 + 4)
        ok |= 2;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault1 + 8);
      return;

    case 2:
      if (execptr == (uintptr_t)&fault2 + 8)
        ok |= 8;
      return;

    case 3:
      if (execptr == (uintptr_t)&fault2 + 8)
        ok |= 16;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault2 + 16);
      return;

    case 4:
      if (execptr == (uintptr_t)&fault3)
        stack = stackptr;
      cpu_exception_resume_pc(regs, (uintptr_t)&fault3 + 4);
      return;

#ifdef CONFIG_HEXO_USERMODE
    case 8:
      if (type == CPU_EXCEPTION_INS_PRIVILEGED &&
          execptr == (uintptr_t)&syscall + 36 &&
          stackptr == 1234+33)
        ok |= 512;
      cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 40);
      return;
#endif
    }
}

#ifdef CONFIG_HEXO_USERMODE

asm(
".text \n"
"syscall:"
    "add     %o0, 17, %o0  \n"
    "ta    55 \n"
    "add     %o0, 7, %o0  \n"
    "add     %o0, 5, %o0  \n"
    "ta    19 \n"
    "add     %o0, 8, %o0  \n"
    "add     %sp, 33, %sp  \n" // bad user stack
    "add     %g0, 1, %g6\n" // destroy cls
    "ta    42 \n"
    "wr      %g0, 1, %wim \n" // user fault
    "add     %o0, 11, %o0  \n"
    "add     %g0, 1, %g7\n" // destroy tls
    "ta      0 \n"
    "nop     \n"
);

static CPU_SYSCALL_HANDLER(syscall_handler)
{
  printk("syscall %i %i\n", number, regs->o[0]);

  switch (step++)
    {
      case 5:
        if (number == 55 && regs->o[0] == 351+17) // syscall arg
          ok |= 64;
        return;

      case 6:
        if (number == 19 && regs->o[0] == 351+17+7+5) // syscall arg
          ok |= 128;
        cpu_exception_resume_pc(regs, (uintptr_t)&syscall + 28);
        return;

      case 7:
        if (number == 42 && regs->o[0] == 351+17+7+5+8) // syscall arg
          ok |= 256;
        regs->o[0] = 117;
        return;

      case 9:
        if (number == 0 && regs->o[0] == 117+11) // syscall arg
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
  if (fault2() == 43)
    ok |= 32;

  if (fault3() == stack)
    ok |= 2048;

#ifdef CONFIG_HEXO_USERMODE
  cpu_interrupt_enable();
  cpu_syscall_sethandler(syscall_handler);
  cpu_context_set_user(1234 + SPARC_STACK_REDZONE, (uintptr_t)&syscall, 351);
#else
  ok |= 64 | 128 | 256 | 512 | 1024;
#endif

  end_test();
}

void end_test()
{
  if (ok == 4095)
    printk("++SUCCESS++%i++\n", ok);
  else
    printk("++FAIL++%x++\n", ok);

  power_shutdown();
  power_reboot();
}


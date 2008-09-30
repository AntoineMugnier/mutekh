#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <hexo/cpu.h>
#include <stdlib.h>
#include <stdio.h>

#include <libunix/syscall.h>
#include <libunix/libunix.h>
#include <libunix/process.h>

extern struct unix_process_s *unix_process_current;

reg_t unix_sys_invalid(void)
{
  puts("invalid syscall\n");
  return 0;
}

reg_t unix_sys_fork(void)
{
    struct unix_process_s *ps_child;
    struct unix_process_s *ps_parent;

#ifdef CONFIG_UNIX_DEBUG
    puts("fork\n");
#endif

    CONTEXT_LOCAL_SET(unix_process_current, ps_parent);

    ps_child = unix_create_process(ps_parent);

    memcpy(&(ps_child->stack_vaddr_start),
	   &(ps_parent->stack_vaddr_start),
	   abs(ps_child->stack_vaddr_end - ps_child->stack_vaddr_start));

    unix_start_process(ps_child);

    return 0;
}

reg_t unix_sys_execve(const char *filename, char *const argv [], char *const envp[])
{
    puts("execve\n");
    return 0;
}

struct unix_syscall_s unix_syscall_table[] =
  {
    { .argc = 0, .call = unix_sys_invalid },
    { .argc = 0, .call = unix_sys_fork },
    { .argc = 3, .call = unix_sys_execve },
  };

#define UNIX_SYSCALLTABLE_SIZE 3

/*
  argc < 6:
  1->ebx
  2->ecx
  3->edx
  4->esi
  5->edi

  argc == 6:
  ptr->ebx
*/

#define CPU_GPREG_EDI   0
#define CPU_GPREG_ESI   1
#define CPU_GPREG_EBP   2
#define CPU_GPREG_ESP   3
#define CPU_GPREG_EBX   4
#define CPU_GPREG_EDX   5
#define CPU_GPREG_ECX   6
#define CPU_GPREG_EAX   7

CPU_SYSCALL_HANDLER(unix_syscall_handler)
{
  puts("yala");
  return;

  if (regtable[CPU_GPREG_EAX] < UNIX_SYSCALLTABLE_SIZE)
    {
      struct unix_syscall_s *f = &unix_syscall_table[regtable[CPU_GPREG_EAX]];

      switch(f->argc)
	{
	case 0:
	  f->call();
	  break;
	case 1:
	  f->call(regtable[CPU_GPREG_EBX]);
	  break;
	case 2:
	  f->call(regtable[CPU_GPREG_EBX],
		  regtable[CPU_GPREG_ECX]);
	  break;
	case 3:
	  f->call(regtable[CPU_GPREG_EBX],
		  regtable[CPU_GPREG_ECX],
		  regtable[CPU_GPREG_EDX]);
	  break;
	case 4:
	  f->call(regtable[CPU_GPREG_EBX],
		  regtable[CPU_GPREG_ECX],
		  regtable[CPU_GPREG_EDX],
		  regtable[CPU_GPREG_ESI]);
	  break;
	case 5:
	  f->call(regtable[CPU_GPREG_EBX],
		  regtable[CPU_GPREG_ECX],
		  regtable[CPU_GPREG_EDX],
		  regtable[CPU_GPREG_ESI],
		  regtable[CPU_GPREG_EDI]);
	  break;
	case 6:
	  {
	    reg_t *args = (reg_t*)regtable[CPU_GPREG_EBX];

	    f->call(args[0], args[1],
		    args[2], args[3],
		    args[4], args[5]);
	  }
	  break;
	default:
	  unix_syscall_table[0].call();
	  break;
	}
    }
  else
	  unix_syscall_table[0].call();
}

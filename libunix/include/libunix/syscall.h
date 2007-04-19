#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/interrupt.h>

struct unix_syscall_s
{
  reg_t (*call)();
  uint_fast8_t argc;
};

CPU_SYSCALL_HANDLER(unix_syscall_handler);

reg_t unix_sys_invalid(void);
reg_t unix_sys_fork(void);
reg_t unix_sys_execve(const char *filename, char *const argv [], char *const envp[]);


#include <mutek/startup.h>

void main();

#ifdef CONFIG_HEXO_CONTEXT
#include <mutek/thread.h>

static CONTEXT_ENTRY(thread)
{
  main();
}
#endif

void app_start()
{
#ifdef CONFIG_HEXO_CONTEXT
  thread_create(thread, NULL, NULL);
#else
  main();
#endif
}

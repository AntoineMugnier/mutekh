#include <mutek/mem_alloc.h>
#include <ble/stack.h>

struct ble_stack_s
{
  struct net_scheduler_s *sched;
};

error_t ble_stack_create(
  size_t sched_stack_size,
  struct ble_stack_s **_stack)
{
  struct ble_stack_s *stack = mem_alloc(sizeof(*stack), mem_scope_sys);

  *_stack = stack;

  return 0;
}

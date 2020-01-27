/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2019, Nicolas Pouillon <nipo@ssji.net>
*/

#undef LOGK_MODULE_ID
#define LOGK_MODULE_ID "smid"

#include <device/device.h>
#include <device/class/smi.h>
#include <device/driver.h>
#include <mutek/scheduler.h>
#include <mutek/printk.h>

const char dev_smi_clause_e[] = ENUM_DESC_DEV_SMI_CLAUSE_E;
const char dev_smi_op_e[] = ENUM_DESC_DEV_SMI_OP_E;
const char dev_smi_rq_type_e[] = ENUM_DESC_DEV_SMI_RQ_TYPE_E;


extern inline
error_t dev_smi_wait_read(
    const struct device_smi_s *accessor,
    enum dev_smi_clause_e clause,
    uint8_t prtad, uint8_t devad, uint16_t address,
    uint16_t *data);

extern inline
error_t dev_smi_wait_write(
    const struct device_smi_s *accessor,
    enum dev_smi_clause_e clause,
    uint8_t prtad, uint8_t devad, uint16_t address,
    uint16_t data);

extern inline
error_t dev_smi_wait_c22_read(
    const struct device_smi_s *accessor,
    uint8_t phy, uint8_t reg,
    uint16_t *data);

extern inline
error_t dev_smi_wait_c22_write(
    const struct device_smi_s *accessor,
    uint8_t phy, uint8_t reg,
    uint16_t data);

extern inline
error_t dev_smi_wait_c22x_read(
    const struct device_smi_s *accessor,
    uint8_t prtad, uint8_t devad,
    uint16_t address, uint16_t *data);

extern inline
error_t dev_smi_wait_c22x_write(
    const struct device_smi_s *accessor,
    uint8_t prtad, uint8_t devad,
    uint16_t address, uint16_t data);

extern inline
error_t dev_smi_wait_c45_read(
    const struct device_smi_s *accessor,
    uint8_t prtad, uint8_t devad,
    uint16_t address, uint16_t *data);

extern inline
error_t dev_smi_wait_c45_write(
    const struct device_smi_s *accessor,
    uint8_t prtad, uint8_t devad,
    uint16_t address, uint16_t data);


static
void _smi_current_done(struct dev_smi_drv_ctx_s *ctx,
                       error_t err)
{
  logk_trace("Current done %d", err);

  LOCK_SPIN_IRQ_SCOPED(&ctx->dev->lock);

  struct dev_smi_rq_s *rq = ctx->current;
  assert(rq);
  rq->error = err;
  ctx->current = NULL;
  ctx->state = DEV_SMI_IDLE;
  dev_smi_rq_done(rq);

  kroutine_exec(&ctx->runner);
}

void dev_smi_drv_transfer_done(struct dev_smi_drv_ctx_s *ctx,
                               error_t err)
{
  assert(ctx->state == DEV_SMI_WAIT_IO);

  ctx->state = DEV_SMI_RUNNING;

  logk_trace("IO Done");

  if (ctx->current->type == DEV_SMI_TRANSFER) {
    struct dev_smi_transfer_rq_s *rq
      = dev_smi_transfer_rq_s_cast(ctx->current);

    if (ctx->data.op == DEV_SMI_READ) {
      logk_trace(" -> Transfer read done %d: 0x%04x", err, ctx->data.value);
      rq->data.value = ctx->data.value;
    } else {
      logk_trace(" -> Transfer write done %d", err);
    }

    return _smi_current_done(ctx, err);
  } else {
    struct dev_smi_bc_rq_s *rq
      = dev_smi_bc_rq_s_cast(ctx->current);

    if (ctx->data.op == DEV_SMI_READ) {
      logk_trace(" -> VM read done %d: 0x%04x", err, ctx->data.value);
      bc_set_reg(&rq->vm, ctx->dest_reg, ctx->data.value);
    } else {
      logk_trace(" -> VM write done %d", err);
    }
    if (err)
      return _smi_current_done(ctx, err);

    kroutine_exec(&ctx->runner);
    return;
  }
}

static
KROUTINE_EXEC(dev_smi_ctx_runner)
{
  struct dev_smi_drv_ctx_s *ctx = KROUTINE_CONTAINER(kr, *ctx, runner);

  switch (ctx->state) {
  case DEV_SMI_WAIT_IO:
    logk_warning("kroutine scheduled while waiting for IO");
    return;

  case DEV_SMI_RUNNING:
    assert(ctx->current);
    goto handle_cur;

  case DEV_SMI_IDLE:
    goto handle_next;
  }

 handle_next:
  {
    LOCK_SPIN_IRQ_SCOPED(&ctx->dev->lock);
    assert(ctx->state == DEV_SMI_IDLE);

    if (!ctx->current)
      ctx->current = dev_smi_rq_pop(&ctx->queue);

    if (!ctx->current) {
      logk_trace("Idle");
      return;
    }
  }

 handle_cur:
  if (ctx->current->type == DEV_SMI_TRANSFER) {
    struct dev_smi_transfer_rq_s *trn
      = dev_smi_transfer_rq_s_cast(ctx->current);
    ctx->data = trn->data;
    ctx->current->error = 0;

    if (trn->data.op == DEV_SMI_READ) {
      logk_trace("Transfer read clause %d prt %d dev %d reg 0x%04x",
                 ctx->data.clause,
                 ctx->data.prtad,
                 ctx->data.devad,
                 ctx->data.address);
    } else {
      logk_trace("Transfer write clause %d prt %d dev %d reg 0x%04x: 0x%04x",
                 ctx->data.clause,
                 ctx->data.prtad,
                 ctx->data.devad,
                 ctx->data.address,
                 ctx->data.value);
    }
    goto transfer_start;
  }

 vm_resume:
  logk_trace("VM Resume, %p", ctx->current);
  struct dev_smi_bc_rq_s *rq
    = dev_smi_bc_rq_s_cast(ctx->current);
  uint16_t op = bc_run(&rq->vm);

  logk_trace("VM Done op 0x%04x", op);

  if (!(op & 0x8000))
    return _smi_current_done(ctx, 0);

  switch (bit_get_mask(op, 8, 3)) {
  case 0: // Yield
    rq->yield_value = bit_get_mask(op, 0, 8);
    return _smi_current_done(ctx, 0);

  case 1: // Reg set
    ctx->data.clause = rq->clause;
    ctx->data.op = DEV_SMI_WRITE;
    ctx->data.prtad = rq->prtad;
    ctx->data.devad = rq->devad;
    ctx->data.address = bc_get_reg(&rq->vm, bit_get_mask(op, 4, 4));
    ctx->data.value = bc_get_reg(&rq->vm, bit_get_mask(op, 0, 4));
    logk_trace("VM write clause %d prt %d dev %d reg 0x%04x: 0x%04x",
               ctx->data.clause,
               ctx->data.prtad,
               ctx->data.devad,
               ctx->data.address,
               ctx->data.value);
    goto transfer_start;

  case 2: // Reg get
    ctx->data.clause = rq->clause;
    ctx->data.op = DEV_SMI_READ;
    ctx->data.prtad = rq->prtad;
    ctx->data.devad = rq->devad;
    ctx->data.address = bc_get_reg(&rq->vm, bit_get_mask(op, 0, 4));
    ctx->dest_reg = bit_get_mask(op, 4, 4);
    logk_trace("VM read clause %d prt %d dev %d r%d=0x%04x -> r%d",
               ctx->data.clause,
               ctx->data.prtad,
               ctx->data.devad,
               bit_get_mask(op, 0, 4),
               ctx->data.address,
               bit_get_mask(op, 4, 4));
    goto transfer_start;
      
  case 3: // prtad
    ctx->data.prtad = bc_get_reg(&rq->vm, bit_get_mask(op, 0, 4));
    goto vm_resume;

  case 4: // devad
    ctx->data.devad = bc_get_reg(&rq->vm, bit_get_mask(op, 0, 4));
    goto vm_resume;

  default:
    return _smi_current_done(ctx, -EINVAL);
  }
  UNREACHABLE();

 transfer_start: {
    logk_trace("Transfer start");
    ctx->state = DEV_SMI_WAIT_IO;
    error_t err = ctx->ops->transfer(ctx, &ctx->data);
    if (err == -EAGAIN)
      return;
    return dev_smi_drv_transfer_done(ctx, err);
  }
}

error_t dev_smi_drv_init(struct device_s *dev,
                         struct dev_smi_drv_ctx_s *ctx,
                         const struct device_smi_ops_s *ops)
{
  ctx->dev = dev;
  ctx->state = DEV_SMI_IDLE;
  dev_rq_queue_init(&ctx->queue);
  ctx->current = NULL;
  ctx->ops = ops;
  kroutine_init_deferred(&ctx->runner, dev_smi_ctx_runner);

  return 0;
}

void dev_smi_drv_request_push(struct dev_smi_drv_ctx_s *ctx,
                              struct dev_smi_rq_s *rq)
{
  LOCK_SPIN_IRQ_SCOPED(&ctx->dev->lock);

  logk_trace("request push, %p", rq);

  dev_smi_rq_pushback(&ctx->queue, rq);
  if (!ctx->current)
    kroutine_exec(&ctx->runner);
}

error_t dev_smi_drv_cleanup(struct dev_smi_drv_ctx_s *ctx)
{
  return -EBUSY;
}

void dev_smi_bc_rq_init(struct dev_smi_bc_rq_s *rq,
                        const struct bc_descriptor_s *desc,
                        enum dev_smi_clause_e clause,
                        uint8_t prtad, uint8_t devad)
{
  rq->base.type = DEV_SMI_BC;
  rq->clause = clause;
  rq->prtad = prtad;
  rq->devad = devad;
  bc_init(&rq->vm, desc);
}

void dev_smi_bc_rq_start(struct device_smi_s *dev,
                         struct dev_smi_bc_rq_s *rq,
                         const void *pc, uint16_t mask, ...)
{
  va_list ap;

  bc_set_pc(&rq->vm, pc);
  va_start(ap, mask);
  bc_set_regs_va(&rq->vm, mask, ap);
  va_end(ap);

  DEVICE_OP(dev, request, &rq->base);
}

void dev_smi_bc_rq_resume(struct device_smi_s *dev,
                         struct dev_smi_bc_rq_s *rq)
{
  DEVICE_OP(dev, request, &rq->base);
}

void dev_smi_bc_rq_cleanup(struct dev_smi_bc_rq_s *rq)
{
}


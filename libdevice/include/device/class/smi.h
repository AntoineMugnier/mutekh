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

    Copyright (c) 2017-2019, Nicolas Pouillon <nipo@ssji.net>
*/

/**
   @file
   @module {Core::Devices support library}
   @short System Management Interface driver class

   @section{Purpose} The @em {SMI} class gives access to Media
   Independant Interface Management bus defined in IEEE802.3.

   @end section
 */

#ifndef __DEVICE_SMI_H__
#define __DEVICE_SMI_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/request.h>

#include <mutek/bytecode.h>
#include <hexo/enum.h>

struct device_s;
struct driver_s;
struct device_smi_s;
struct driver_smi_s;
struct dev_smi_drv_ctx_s;
struct dev_smi_rq_s;

/*
  Request datatypes
 */

ENUM_DESCRIPTOR(dev_smi_clause_e, strip:DEV_SMI_, upper);
ENUM_DESCRIPTOR(dev_smi_op_e, strip:DEV_SMI_, upper);
ENUM_DESCRIPTOR(dev_smi_rq_type_e, strip:DEV_SMI_, upper);

enum dev_smi_clause_e
{
  /** Clause 22 */
  DEV_SMI_C22,
  /** Clause 22 extension to access Clause 45 (802.3ah) */
  DEV_SMI_C22X,
  /** Clause 45 (802.3ae) */
  DEV_SMI_C45,
};

enum dev_smi_op_e
{
  /** Register read */
  DEV_SMI_READ,
  /** Register write */
  DEV_SMI_WRITE,
};

enum dev_smi_rq_type_e
{
  /** Standalone transfer */
  DEV_SMI_TRANSFER,
  /** Bytecode */
  DEV_SMI_BC,
};

struct dev_smi_data_s
{
  /** IEEE 802 Clause */
  enum dev_smi_clause_e BITFIELD(clause,2);

  /** Operation type */
  enum dev_smi_op_e BITFIELD(op,1);

  /** Destination port on bus (clause 22 "PHYAD" or 45 "PRTAD"), 5 bits */
  uint8_t prtad;
  /** Destination device on port, 5 bits, clause 45 only */
  uint8_t devad;
  /** Register number in device (only 5 bits "REGAD" for clause 22) */
  uint16_t address;
  /** Value */
  uint16_t value;
};

struct dev_smi_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** @internal */
  struct device_smi_s *smi;

  /** @internal */
  bool_t BITFIELD(active,1);

  /** Request type */
  enum dev_smi_rq_type_e  BITFIELD(type,1);
};

DEV_REQUEST_INHERIT(smi);
DEV_REQUEST_QUEUE_OPS(smi);

struct dev_smi_transfer_rq_s
{
  union {
    struct dev_smi_rq_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  struct dev_smi_data_s data;
};

STRUCT_INHERIT(dev_smi_transfer_rq_s, dev_smi_rq_s, base);

struct dev_smi_bc_rq_s
{
  union {
    struct dev_smi_rq_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** bytecode virtual machine context */
  struct bc_context_s vm;

  /** Filled when yield is reached in bytecode */
  uint8_t yield_value;

  /** @internal */
  uint8_t prtad;
  /** @internal */
  uint8_t devad;
  /** @internal */
  enum dev_smi_clause_e BITFIELD(clause,2);
};

STRUCT_INHERIT(dev_smi_bc_rq_s, dev_smi_rq_s, base);

/*
  Driver API
 */

/** @see dev_smi_request_t */
#define DEV_SMI_REQUEST(n) void (n) (                              \
    const struct device_smi_s *accessor,                           \
    struct dev_smi_rq_s *rq)

/** @This enqueues a request.

   The kroutine of the request may be executed from within this
   function. Please read @xref {Nested device request completion}. 

   @xsee {Purpose} @see dev_smi_request_type_e
*/
typedef DEV_SMI_REQUEST(dev_smi_request_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_SMI, smi,
    dev_smi_request_t *f_request;
);

/** @see driver_smi_s */
#define DRIVER_SMI_METHODS(prefix)                                    \
  ((const struct driver_class_s*)&(const struct driver_smi_s){        \
    .class_ = DRIVER_CLASS_SMI,                                       \
    .f_request = prefix ## _request,                                  \
  })

/*
  Internal helper API
 */

/** @see dev_smi_transfer_t */
#define DEV_SMI_TRANSFER(n) error_t (n) (                               \
    struct dev_smi_drv_ctx_s *ctx,                                      \
    struct dev_smi_data_s *data)

/** @internal */
typedef DEV_SMI_TRANSFER(dev_smi_transfer_t);

/**
   @internal @this defines an API for a SMI driver. It must be passed
   to libdevice helpers to accomplish various management tasks.
 */
struct device_smi_ops_s
{
  dev_smi_transfer_t *transfer;
};

/**
   @this is a helper to declare a @ref device_smi_ops_s
   structure.
 */
#define DRIVER_SMI_OPS_DECLARE(prefix)                                  \
  static const struct device_smi_ops_s prefix##_smi_ops = {             \
    .transfer = prefix ## _transfer,                                    \
  }

/*
  Driver context
 */

enum dev_smi_runner_state_e
{
  DEV_SMI_IDLE,
  DEV_SMI_RUNNING,
  DEV_SMI_WAIT_IO,
};

struct dev_smi_drv_ctx_s
{
  struct device_s *dev;
  dev_request_queue_root_t queue;
  struct dev_smi_data_s data;
  struct dev_smi_rq_s *current;
  const struct device_smi_ops_s *ops;
  enum dev_smi_runner_state_e state;
  struct kroutine_s runner;
  uint8_t dest_reg;
};

error_t dev_smi_drv_init(struct device_s *dev,
                         struct dev_smi_drv_ctx_s *ctx,
                         const struct device_smi_ops_s *ops);

void dev_smi_drv_request_push(struct dev_smi_drv_ctx_s *ctx,
                              struct dev_smi_rq_s *rq);

void dev_smi_drv_transfer_done(struct dev_smi_drv_ctx_s *ctx,
                               error_t err);

error_t dev_smi_drv_cleanup(struct dev_smi_drv_ctx_s *ctx);

/*
  Client API helpers
 */

DEV_REQUEST_WAIT_FUNC(smi);

/** @This perform a read operation with given clause and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_read(
    const struct device_smi_s *accessor,
    enum dev_smi_clause_e clause,
    uint8_t prtad, uint8_t devad, uint16_t address,
    uint16_t *value),
{
  struct dev_smi_transfer_rq_s rq = {
    .data.clause = clause,
    .data.op = DEV_SMI_READ,
    .data.prtad = prtad,
    .data.devad = devad,
    .data.address = address,
    .base.type = DEV_SMI_TRANSFER,
  };

  error_t err = dev_smi_wait_rq(accessor, &rq.base);

  *value = rq.data.value;

  return err;
});

/** @This perform a read operation with given clause and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_write(
    const struct device_smi_s *accessor,
    enum dev_smi_clause_e clause,
    uint8_t prtad, uint8_t devad, uint16_t address,
    uint16_t value),
{
  struct dev_smi_transfer_rq_s rq = {
    .data.clause = clause,
    .data.op = DEV_SMI_WRITE,
    .data.prtad = prtad,
    .data.devad = devad,
    .data.address = address,
    .data.value = value,
    .base.type = DEV_SMI_TRANSFER,
  };

  return dev_smi_wait_rq(accessor, &rq.base);
});

/** @This perform a Clause 22 read operation and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_c22_read(
    const struct device_smi_s *accessor,
    uint8_t phy, uint8_t reg,
    uint16_t *value),
{
  return dev_smi_wait_read(accessor, DEV_SMI_C22, phy, 0, reg, value);
});

/** @This perform a Clause 22 write operation and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_c22_write(
    const struct device_smi_s *accessor,
    uint8_t phy, uint8_t reg,
    uint16_t value),
{
  return dev_smi_wait_write(accessor, DEV_SMI_C22, phy, 0, reg, value);
});

/** @This perform a Clause 45 read operation and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_c45_read(
    const struct device_smi_s *accessor,
    uint8_t prtad, uint8_t devad,
    uint16_t address, uint16_t *value),
{
  return dev_smi_wait_read(accessor, DEV_SMI_C45, prtad, devad, address, value);
});

/** @This perform a Clause 45 write operation and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_c45_write(
    const struct device_smi_s *accessor,
    uint8_t prtad, uint8_t devad,
    uint16_t address, uint16_t value),
{
  return dev_smi_wait_write(accessor, DEV_SMI_C45, prtad, devad, address, value);
});

/** @This perform a Clause 22X read operation and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_c22x_read(
    const struct device_smi_s *accessor,
    uint8_t prtad, uint8_t devad,
    uint16_t address, uint16_t *value),
{
  return dev_smi_wait_read(accessor, DEV_SMI_C22X, prtad, devad, address, value);
});

/** @This perform a Clause 22 write operation and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_c22x_write(
    const struct device_smi_s *accessor,
    uint8_t prtad, uint8_t devad,
    uint16_t address, uint16_t value),
{
  return dev_smi_wait_write(accessor, DEV_SMI_C22X, prtad, devad, address, value);
});

void dev_smi_bc_rq_init(struct dev_smi_bc_rq_s *rq,
                        const struct bc_descriptor_s *desc,
                        enum dev_smi_clause_e clause,
                        uint8_t prtad, uint8_t devad);

void dev_smi_bc_rq_start(struct device_smi_s *dev,
                         struct dev_smi_bc_rq_s *rq,
                         const void *pc, uint16_t mask, ...);

void dev_smi_bc_rq_resume(struct device_smi_s *dev,
                         struct dev_smi_bc_rq_s *rq);

void dev_smi_bc_rq_cleanup(struct dev_smi_bc_rq_s *rq);
                           

/*
  Resource helpers
 */

#ifdef CONFIG_DEVICE_SMI
# define DEV_STATIC_RES_DEV_SMI(path_)                          \
  DEV_STATIC_RES_DEVCLASS_PARAM("smi", path_, DRIVER_CLASS_SMI)
#else
# define DEV_STATIC_RES_DEV_SMI(path_) { .type = DEV_RES_UNUSED, }
#endif

#endif

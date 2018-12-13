/*
 * libpersist examples, some toughts:
 *
 * 1) blob entries can be used to store custom structures.
 * 2) counters use bitfields to minimize flash use
 *
 * Libpersist implementation enforce the use of at least 2
 * page of flash to pack valid entries when storage is full.
 * You must provide a device address aligned on the page size.
 *
 * Size of strutures used in blob entries must have a size
 * aligned on 32 bits. Use padding if your data fit unaligned
 * size.
 *
 * All blob reads return const because in case of memory
 * mapped flash backend, data can't be edited (use memcpy,
 * to create a modifiable copy).
 *
 * Be careful with descriptor's uid and uid_offset. If an entry
 * has a descriptor of type my_data and an uid = 0x1000, then
 * an other blob entry of the same type with uid_offset = 1 will
 * be set, in the backend, with uid = 0x1001. Hence, an other
 * descriptor type can't have uid = 0x1001! That is why in the example
 * the blob descriptor has uid = 0x1000 and the counter has
 * uid = 0x2000, to allow multiple my_data entries before colliding
 * with the counter descriptor.
 *
 */

#define LOGK_MODULE_ID "pste"

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/mem_alloc.h>

#include <persist/persist.h>

#include <assert.h>

#include "config.h"

#define PRST_PAGE_NUM   2
#define PRST_PAGE_SIZE  CONFIG_PERSIST_RAM_BACKEND_PAGESIZE
#define PRST_DEV_SIZE   (PRST_PAGE_SIZE * PRST_PAGE_NUM)

#define ERROR(...)                                    \
  {                                                   \
    logk_error("ERR: "__VA_ARGS__);                   \
    goto end;                                         \
  }

/* custom data structure to be stored in persist storage */
struct my_data
{
  uint16_t uid_offset;
  uint32_t a;
  uint32_t b;
  uint32_t c;
};

/* counter structure refering to a counter */
struct my_counter
{
  uint16_t uid_offset;
  uint64_t value;
};

/* Persist context required in every storage action */
static struct persist_context_s prst_ctx;

/* descriptors */
static const struct persist_descriptor_s blob_entry =
  {
   .uid = 0x1000,
   .type = PERSIST_BLOB,
   .size = sizeof(struct my_data), /* must be aligned on 32bits */
  };

static const struct persist_descriptor_s counter_entry =
  {
   .uid = 0x2000,
   .type = PERSIST_COUNTER,
   .size = 16,
  };

/* data to be stored using BLOB entry */
static struct my_data d1 =
  {
   .uid_offset = 0,
   .a = 1,
   .b = 2,
   .c = 3,
  };

static struct my_data d2 =
  {
   .uid_offset = 1,
   .a = 0xf1,
   .b = 0xf2,
   .c = 0xf3,
  };

/* counters */
static struct my_counter c1 =
  {
   .uid_offset = 0,
   .value = 0,
  };

static struct my_counter c2 =
  {
   .uid_offset = 1,
   .value = 50,
  };


void app_start()
{
  /* allocate pages in RAM to emulate flash pages */
  void *dev = mem_alloc_align(PRST_DEV_SIZE,
                              PRST_PAGE_SIZE,
                              mem_scope_sys);
  /*
     In the contest of no RAM backend, dev can typically be the address
     of the first page hosting libpersist storage.
  */

  if (dev == NULL)
    ERROR("dev is NULL");

  /* initialize the context, expecting at least 2 pages */
  persist_context_init(&prst_ctx,
		       (uintptr_t) dev,
                       PRST_DEV_SIZE,
		       PRST_PAGE_SIZE);

  error_t err;

  /* BLOB entry WRITE */
  err = persist_wait_write(&prst_ctx,
                           &blob_entry,
                           d1.uid_offset,
                           &d1);
  if (err)
    ERROR("d1 write fail: %d", err);

  err = persist_wait_write(&prst_ctx,
                           &blob_entry,
                           d2.uid_offset,
                           &d2);
  if (err)
    ERROR("d2 write fail: %d", err);

  /* BLOB entries READ */
  const struct my_data *dout;
  err = persist_wait_read(&prst_ctx,
                          &blob_entry,
                          d1.uid_offset,
                          (const void **) &dout);
  if (err)
    ERROR("d1 read fail: %d", err);

  assert(dout->a == 1);
  assert(dout->b == 2);
  assert(dout->c == 3);

  err = persist_wait_read(&prst_ctx,
                          &blob_entry,
                          d2.uid_offset,
                          (const void **) &dout);
  if (err)
    ERROR("d2 read fail: %d", err);

  assert(dout->a == 0xf1);
  assert(dout->b == 0xf2);
  assert(dout->c == 0xf3);

  /* COUNTER entries INC */
  for (uint8_t i = 0; i < 10; i++)
    {
      err = persist_wait_inc(&prst_ctx,
                             &counter_entry,
                             c1.uid_offset);
      if (err)
        ERROR("c1 inc fail: %d (%hhu)", err, i);

      err = persist_wait_inc(&prst_ctx,
                             &counter_entry,
                             c2.uid_offset);
      if (err)
        ERROR("c2 inc fail: %d (%hhu)", err, i);
    }

  /* COUNTER entries READ */
  uint64_t cout = 0;

  err = persist_wait_counter_read(&prst_ctx,
                                  &counter_entry,
                                  c1.uid_offset,
                                  &cout);
  if (err)
    ERROR("c1 read fail: %d)", err);

  c1.value += cout;
  assert(c1.value == 10);

  err = persist_wait_counter_read(&prst_ctx,
                                  &counter_entry,
                                  c2.uid_offset,
                                  &cout);
  if (err)
    ERROR("c2 read fail: %d)", err);

  c2.value += cout;
  assert(c2.value == 60);

  logk_debug("persist example ok");

 end:
  free(dev);
}

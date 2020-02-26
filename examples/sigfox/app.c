
#include <vulpis/vulpis.h>

#include <device/class/rfpacket.h>
#include <device/class/timer.h>
#include <device/class/crypto.h>
#include <persist/persist.h>

#include <mutek/startup.h>
#include <mutek/shell.h>
#include <semaphore.h>

static struct vulpis_context_s vl_ctx;
static struct persist_context_s pr_ctx;
static sem_t vl_sem;

#define PERSIST_ADDR        0   /* XXX */
#define PERSIST_PAGE_COUNT  2
#define PERSIST_PAGE_SIZE   2048

static KROUTINE_EXEC(vulpis_done)
{
  sem_post(&vl_sem);
}

void app_start()
{
  ensure(!device_get_accessor_by_path(&vl_ctx.tm_dev.base, NULL,
				      "/rtc /timer*", DRIVER_CLASS_TIMER));

  ensure(!device_get_accessor_by_path(&vl_ctx.rf_dev.base, NULL,
				      "/rf*", DRIVER_CLASS_RFPACKET));

  ensure(!device_get_accessor_by_path(&vl_ctx.aes_dev.base, NULL,
				      "/aes*", DRIVER_CLASS_CRYPTO));

  sem_init(&vl_sem, 0, 0);

  vl_ctx.pr_ctx = &pr_ctx;

  persist_context_init(&pr_ctx, PERSIST_ADDR,
		       PERSIST_PAGE_COUNT * PERSIST_PAGE_SIZE,
		       PERSIST_PAGE_SIZE);

  vulpis_context_init(&vl_ctx);
  persist_context_init(&pr_ctx, 0x3e000, 2048 * 4, 2048);

  kroutine_init_immediate(&vl_ctx.kr, vulpis_done);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_vulpis_send)
{
  if (argc)
    vulpis_send(&vl_ctx, (const uint8_t*)argv[0], argl[0] * 8);
  else
    vulpis_send(&vl_ctx, (const uint8_t*)"\x00", 1);

  sem_wait(&vl_sem);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_vulpis_set_id_key)
{
  if (argl[0] != 4 + 16)   /* 4 bytes device ID + 16 bytes device key */
    return -EINVAL;

  if (persist_wait_write(&pr_ctx, &vulpis_persist_id_desc, CONFIG_VULPIS_PERSIST_UID, argv[0]))
    return -EINVAL;

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_vulpis_get_id_key)
{
  uint8_t *data;

  if (persist_wait_read(&pr_ctx, &vulpis_persist_id_desc, CONFIG_VULPIS_PERSIST_UID, (const void**)&data))
    return -EINVAL;

  termui_con_printf(con, "%P", data, 4 + 16);

  return 0;
}

TERMUI_CON_GROUP_DECL(shell_vulpis_subgroup) =
{
  TERMUI_CON_ENTRY(shell_vulpis_send, "send",
    TERMUI_CON_ARGS(0, 1)
  )

  TERMUI_CON_ENTRY(shell_vulpis_set_id_key, "set_id_key",
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(shell_vulpis_get_id_key, "get_id_key",
  )

  TERMUI_CON_LIST_END
};

MUTEK_SHELL_ROOT_GROUP(shell_vulpis_subgroup, "vulpis");

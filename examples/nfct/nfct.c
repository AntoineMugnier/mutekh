#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>
#include <device/class/nfc.h>

struct type2_s
{
  struct device_nfc_s nfc;
  struct dev_nfc_rq_s nfc_rq;
  struct dev_nfc_config_s nfc_config;
  uint8_t pending_cmd;
  uint8_t base_addr;

  uint8_t memory[13*4];
};

enum nfc_tag_type2_cmd_e
{
  NFC_TYPE2_READ = 0x30,
  NFC_TYPE2_FAST_READ = 0x3a,
  NFC_14A_HALT = 0x50,
  NFC_14A_REQA = 0x26,
  NFC_14A_ANTICOLL = 0x93,
  NFC_14A_ANTICOLL2 = 0x95,
  NFC_14A_ANTICOLL3 = 0x97,
  NFC_TYPE2_GET_VERSION = 0x60,
  NFC_TYPE2_COMPAT_WRITE = 0xa0,
  NFC_TYPE2_WRITE = 0xa2,
  NFC_TYPE2_SECTOR_SELECT = 0xc2,
};

enum nfc_ack_e
{
  NFC_TYPE2_ACK = 0xa,
  NFC_TYPE2_EINVAL = 0x0, // Invalid argument
  NFC_TYPE2_CRC_ERR = 0x1, // Parity or CRC error
  NFC_TYPE2_ARB_LOCK = 0x3, // Arbitrer locked to IC
  NFC_TYPE2_EOVERFLOW = 0x4, // Counter write overflow
  NFC_TYPE2_EIO2 = 0x5, // EEPROM Write Error
  NFC_TYPE2_EIO = 0x7, // EEPROM Write Error
  NFC_TYPE2_OTHER2 = 0x6, // Other Error
  NFC_TYPE2_OTHER = 0x9, // Other Error
};

enum nfc_manufacturer_e
{
  NFC_MANUF_MOTOROLA                           = 0x01, // Motorola [UK]
  NFC_MANUF_STMICRO                            = 0x02, // STMicroelectronics SA [France]
  NFC_MANUF_HITACHI                            = 0x03, // Hitachi, Ltd [Japan]
  NFC_MANUF_NXP                                = 0x04, // NXP Semiconductors [Germany]
  NFC_MANUF_INFINEON                           = 0x05, // Infineon Technologies AG [Germany]
  NFC_MANUF_CYLINK                             = 0x06, // Cylink [USA]
  NFC_MANUF_TI                                 = 0x07, // Texas Instrument [France]
  NFC_MANUF_FUJITSU                            = 0x08, // Fujitsu Limited [Japan]
  NFC_MANUF_MATSUSHITA                         = 0x09, // Matsushita Electronics Corporation [Japan]
  NFC_MANUF_NEC                                = 0x0A, // NEC [Japan]
  NFC_MANUF_OKI                                = 0x0B, // Oki Electric Industry Co. Ltd [Japan]
  NFC_MANUF_TOSHIBA                            = 0x0C, // Toshiba Corp. [Japan]
  NFC_MANUF_MITSUBISHI                         = 0x0D, // Mitsubishi Electric Corp. [Japan]
  NFC_MANUF_SAMSUNG                            = 0x0E, // Samsung Electronics Co. Ltd [Korea]
  NFC_MANUF_HYNIX                              = 0x0F, // Hynix [Korea]
  NFC_MANUF_LG                                 = 0x10, // LG-Semiconductors Co. Ltd [Korea]
  NFC_MANUF_EMOSYN_EM                          = 0x11, // Emosyn-EM Microelectronics [USA]
  NFC_MANUF_INSIDE                             = 0x12, // INSIDE Technology [France]
  NFC_MANUF_ORGA_KARTENSYSTEME                 = 0x13, // ORGA Kartensysteme GmbH [Germany]
  NFC_MANUF_SHARP                              = 0x14, // SHARP Corporation [Japan]
  NFC_MANUF_ATMEL                              = 0x15, // ATMEL [France]
  NFC_MANUF_EM_MICROELECTRONIC_MARIN           = 0x16, // EM Microelectronic-Marin SA [Switzerland]
  NFC_MANUF_KSW_MICROTEC                       = 0x17, // KSW Microtec GmbH [Germany]
  NFC_MANUF_ZMD                                = 0x18, // ZMD AG [Germany]
  NFC_MANUF_XICOR                              = 0x19, // XICOR, Inc. [USA]
  NFC_MANUF_SONY                               = 0x1A, // Sony Corporation [Japan]
  NFC_MANUF_MALAYSIA_MICROELECTRONIC_SOLUTIONS = 0x1B, // Malaysia Microelectronic Solutions Sdn. Bhd [Malaysia]
  NFC_MANUF_EMOSYN                             = 0x1C, // Emosyn [USA]
  NFC_MANUF_SHANGHAI_FUDAN_MICROELECTRONICS    = 0x1D, // Shanghai Fudan Microelectronics Co. Ltd. P.R. [China]
  NFC_MANUF_MAGELLAN                           = 0x1E, // Magellan Technology Pty Limited [Australia]
  NFC_MANUF_MELEXIS                            = 0x1F, // Melexis NV BO [Switzerland]
  NFC_MANUF_RENESAS_TECHNOLOGY                 = 0x20, // Renesas Technology Corp. [Japan]
  NFC_MANUF_TAGSYS                             = 0x21, // TAGSYS [France]
  NFC_MANUF_TRANSCORE                          = 0x22, // Transcore [USA]
  NFC_MANUF_SHANGHAI_BELLING                   = 0x23, // Shanghai belling corp., ltd. [China]
  NFC_MANUF_MASKTECH                           = 0x24, // Masktech Germany Gmbh [Germany]
  NFC_MANUF_INNOVISION                         = 0x25, // Innovision Research and Technology Plc [UK]
  NFC_MANUF_HITACHI_ULSI                       = 0x26, // Hitachi ULSI Systems Co., Ltd. [Japan]
  NFC_MANUF_CYPAK                              = 0x27, // Cypak AB [Sweden]
  NFC_MANUF_RICOH                              = 0x28, // Ricoh [Japan]
  NFC_MANUF_ASK                                = 0x29, // ASK [France]
  NFC_MANUF_UNICORE_MICROSYSTEMS               = 0x2A, // Unicore Microsystems, LLC [RussianFederation]
  NFC_MANUF_MAXIM                              = 0x2B, // Dallas Semiconductor/Maxim [USA]
  NFC_MANUF_IMPINJ                             = 0x2C, // Impinj, Inc. [USA]
  NFC_MANUF_RIGHTPLUG_ALLIANCE                 = 0x2D, // RightPlug Alliance [USA]
  NFC_MANUF_BROADCOM                           = 0x2E, // Broadcom Corporation [USA]
  NFC_MANUF_MSTAR_SEMICONDUCTOR                = 0x2F, // MStar Semiconductor, Inc Taiwan, [ROC]
  NFC_MANUF_BEEDAR_TECHNOLOGY                  = 0x30, // BeeDar Technology Inc. [USA]
  NFC_MANUF_RFIDSEC                            = 0x31, // RFIDsec [Denmark]
  NFC_MANUF_SCHWEIZER_ELECTRONIC               = 0x32, // Schweizer Electronic AG [Germany]
  NFC_MANUF_AMIC_TECHNOLOGY                    = 0x33, // AMIC Technology Corp [Taiwan]
  NFC_MANUF_MIKRON                             = 0x34, // Mikron JSC [Russia]
  NFC_MANUF_FRAUNHOFER                         = 0x35, // Fraunhofer Institute for Photonic Microsystems [Germany]
  NFC_MANUF_IDS_MICROCHIP                      = 0x36, // IDS Microchip AG [Switzerland]
  NFC_MANUF_KOVIO                              = 0x37, // Kovio [USA]
  NFC_MANUF_HMT_MICROELECTRONIC                = 0x38, // HMT Microelectronic Ltd [Switzerland]
  NFC_MANUF_SILICON_CRAFT_TECHNOLOGY           = 0x39, // Silicon Craft Technology [Thailand]
  NFC_MANUF_ADVANCED_FILM_DEVICE               = 0x3A, // Advanced Film Device Inc. [Japan]
  NFC_MANUF_NITECREST                          = 0x3B, // Nitecrest Ltd [UK]
  NFC_MANUF_VERAYO                             = 0x3C, // Verayo Inc. [USA]
  NFC_MANUF_HID_GLOBAL                         = 0x3D, // HID Global [USA]
  NFC_MANUF_PRODUCTIVITY_ENGINEERING           = 0x3E, // Productivity Engineering Gmbh [Germany]
  NFC_MANUF_AUSTRIAMICROSYSTEMS                = 0x3F, // Austriamicrosystems AG (reserved) [Austria]
  NFC_MANUF_GEMALTO                            = 0x40, // Gemalto SA [France]
  NFC_MANUF_RENESAS_ELECTRONICS                = 0x41, // Renesas Electronics Corporation [Japan]
  NFC_MANUF_3ALOGICS                           = 0x42, // 3Alogics Inc [Korea]
  NFC_MANUF_TOP_TRONIQ                         = 0x43, // Top TroniQ Asia Limited Hong [Kong]
  NFC_MANUF_GENTAG                             = 0x44, // Gentag Inc [USA]
  NFC_MANUF_NORDIC                             = 0x5f, // Nordic Semiconductor [Norway]
};

enum nfc_tag_type_e
{
  NFC_TYPE_MF_ULTRALIGHT  = 0x00, // NXP MIFARE Ultralight | Ultralight C
  NFC_TYPE_MF_DESFIRE     = 0x04, // NXP MIFARE (various !DESFire !DESFire EV1)
  NFC_TYPE_MF_CLASSIC     = 0x08, // NXP MIFARE CLASSIC 1k | Plus 2k
  NFC_TYPE_MF_MINI        = 0x09, // NXP MIFARE Mini 0.3k
  NFC_TYPE_MF_PLUS_2K     = 0x10, // NXP MIFARE Plus 2k
  NFC_TYPE_MF_PLUS_4K     = 0x11, // NXP MIFARE Plus 4k
  NFC_TYPE_MF_CLASSIC_4K  = 0x18, // NXP MIFARE Classic 4k | Plus 4k
  NFC_TYPE_MF_DESFIRE_4K  = 0x20, // NXP MIFARE DESFire 4k | DESFire EV1 2k/4k/8k | Plus 2k/4k | JCOP 31/41
  NFC_TYPE_MF_DESFIRE_EV1 = 0x24, // NXP MIFARE DESFire | DESFire EV1
  NFC_TYPE_JCOP31         = 0x28, // JCOP31 or JCOP41 v2.3.1
  NFC_TYPE_NOKIA_6212     = 0x38, // Nokia 6212 or 6131 MIFARE CLASSIC 4K
  NFC_TYPE_MF_CLASSIC_INF = 0x88, // Infineon MIFARE CLASSIC 1K
  NFC_TYPE_MPCOS          = 0x98, // Gemplus MPCOS
};

static error_t type2_poll(struct type2_s *t2)
{
  error_t err;

  printk("Polling...");

  t2->nfc_rq.config = &t2->nfc_config;
  t2->nfc_config.mode = DEV_NFC_14443A;
  t2->nfc_config.rate = DEV_NFC_106K;
  t2->nfc_config.side = DEV_NFC_TARGET;

  t2->nfc_rq.type = DEV_NFC_POLL;

  err = dev_nfc_wait_request(&t2->nfc, &t2->nfc_rq);
  if (err) {
    printk(" failed %d\n", err);
    return err;
  }

  printk(" OK\n");
  return 0;
}

static ssize_t type2_cmd_read(struct type2_s *t2, uint8_t *cmd, ssize_t size)
{
  error_t err;

  printk(">");

  t2->nfc_rq.type = DEV_NFC_READ;
  t2->nfc_rq.data.auto_parity = 1;
  t2->nfc_rq.data.last_byte_bits = 0;
  t2->nfc_rq.data.data = cmd;
  t2->nfc_rq.data.parity = NULL;
  t2->nfc_rq.data.size = size;

  err = dev_nfc_wait_request(&t2->nfc, &t2->nfc_rq);

  if (err) {
    printk(" failed %d\n", err);
    return err;
  }

  size = size - t2->nfc_rq.data.size;

  printk(" %P\n", cmd, size);

  return size;
}

static error_t type2_reply_send(struct type2_s *t2, const uint8_t *rsp, size_t size)
{
  error_t err;

  printk("< %P", rsp, size);

  t2->nfc_rq.type = DEV_NFC_WRITE;
  t2->nfc_rq.data.auto_parity = 1;
  t2->nfc_rq.data.last_byte_bits = 0;
  t2->nfc_rq.data.data = (uint8_t *)rsp;
  t2->nfc_rq.data.parity = NULL;
  t2->nfc_rq.data.size = size;

  err = dev_nfc_wait_request(&t2->nfc, &t2->nfc_rq);

  if (err) {
    printk(" failed %d\n", err);
    return err;
  }

  printk("\n");

  return 0;
}

static error_t type2_ack_send(struct type2_s *t2, uint8_t value)
{
  error_t err;

  printk("< (%x)", value);

  t2->nfc_rq.type = DEV_NFC_WRITE;
  t2->nfc_rq.data.auto_parity = 0;
  t2->nfc_rq.data.last_byte_bits = 4;
  t2->nfc_rq.data.data = &value;
  t2->nfc_rq.data.parity = NULL;
  t2->nfc_rq.data.size = 1;

  err = dev_nfc_wait_request(&t2->nfc, &t2->nfc_rq);

  if (err) {
    printk(" failed %d\n", err);
    return err;
  }

  printk("\n");

  return 0;
}

static size_t command_handle(struct type2_s *t2, const uint8_t *cmd, size_t cmd_size)
{
  uint8_t rsp[64];

  switch (t2->pending_cmd) {
  case NFC_TYPE2_SECTOR_SELECT:
    t2->base_addr = cmd[0];
    t2->pending_cmd = 0;
    return type2_ack_send(t2, NFC_TYPE2_ACK);

  case NFC_TYPE2_COMPAT_WRITE:
    t2->pending_cmd = 0;
    memcpy(t2->memory + t2->base_addr * 4, cmd, 4);
    return type2_ack_send(t2, NFC_TYPE2_ACK);
  }

  switch (*cmd) {
  case NFC_TYPE2_READ:
    printk("Read page %d\n", cmd[1]);
    if (cmd[1] * 4 > sizeof(t2->memory))
      return type2_ack_send(t2, NFC_TYPE2_EINVAL);
    size_t first = __MIN(sizeof(t2->memory) - 4 * cmd[1], 16);
    memcpy(rsp, t2->memory + 4 * cmd[1], first);
    memcpy(rsp + first, t2->memory, 16 - first);
    return type2_reply_send(t2, rsp, 16);

  case NFC_TYPE2_FAST_READ:
    printk("Read block %d->%d\n", cmd[1], cmd[2]);
    if (cmd[1] * 4 > sizeof(t2->memory) || cmd[2] * 4 > sizeof(t2->memory) || cmd[1] > cmd[2])
      return type2_ack_send(t2, NFC_TYPE2_EINVAL);
    return type2_reply_send(t2, t2->memory + 4 * cmd[1], (cmd[2] - cmd[1] + 1) * 4);

  case NFC_TYPE2_GET_VERSION:
    printk("Get version\n");
    memcpy(rsp, "\x00\x00\x03\x01\x01\x00\x0b\x03", 8);
    rsp[1] = t2->memory[0];
    return type2_reply_send(t2, rsp, 8);

  case NFC_TYPE2_COMPAT_WRITE:
    printk("Write %d\n", cmd[1]);
    if (cmd[1] < 3 || cmd[1] * 4 >= sizeof(t2->memory))
      return type2_ack_send(t2, NFC_TYPE2_EINVAL);
    t2->base_addr = cmd[1];
    t2->pending_cmd = cmd[0];
    return type2_ack_send(t2, NFC_TYPE2_ACK);

  case NFC_TYPE2_WRITE:
    printk("Write %d: %P\n", cmd[1], cmd + 2, 4);
    if (cmd[1] < 3 || cmd[1] * 4 >= sizeof(t2->memory))
      return type2_ack_send(t2, NFC_TYPE2_EINVAL);
    memcpy(t2->memory + cmd[1] * 4, cmd + 2, 4);
    return type2_ack_send(t2, NFC_TYPE2_ACK);

  case NFC_TYPE2_SECTOR_SELECT:
    printk("Sector select\n");
    t2->pending_cmd = cmd[0];
    return type2_ack_send(t2, NFC_TYPE2_ACK);

  case NFC_14A_HALT:
    printk("Halt\n");
    return -EEOF;

  default:
    printk("Other command %P\n", cmd, cmd_size);
    return type2_ack_send(t2, NFC_TYPE2_EINVAL);
  }
}

static CONTEXT_ENTRY(main)
{
  struct type2_s t2[1];
  uint8_t cmd[64];
  error_t err;
  ssize_t size;

  err = device_get_accessor_by_path(&t2->nfc.base, NULL, "nfc0", DRIVER_CLASS_NFC);
  assert(!err && "NFC device not found");

  DEVICE_OP(&t2->nfc, info_get, &t2->nfc_config.local);
  t2->nfc_config.local.uid_size = 7;
  t2->nfc_config.local.uid[0] = 4;

  printk("Emulating a T2 tag with UID %P\n",
         t2->nfc_config.local.uid, t2->nfc_config.local.uid_size);

  memcpy(t2->memory, t2->nfc_config.local.uid, 3);
  memcpy(t2->memory + 4, t2->nfc_config.local.uid + 3, 4);
  t2->memory[3] = t2->memory[0] ^ t2->memory[1] ^ t2->memory[2] ^ 0x88;
  t2->memory[8] = t2->memory[4] ^ t2->memory[5] ^ t2->memory[6] ^ t2->memory[7];
  // Default
  t2->memory[9] = 0x48;
  // Locks
  t2->memory[10] = 0;
  t2->memory[11] = 0;
  // Capability Container
  t2->memory[12] = 0xe1; // NDEF
  t2->memory[13] = 0x10; // 1.0
  t2->memory[14] = sizeof(t2->memory) / 8 - 1;
  t2->memory[15] = 0x0f; // Write protected

  memset(t2->memory + 16, 0, sizeof(t2->memory) - 16);

  for (;;) {
    err = type2_poll(t2);
    if (err)
      continue;

    t2->pending_cmd = 0;
    t2->base_addr = 0;

    while (!err) {
      size = type2_cmd_read(t2, cmd, sizeof(cmd));
      if (size < 0)
        break;

      err = command_handle(t2, cmd, size);
      if (err)
        break;
    }
  }  
}

void app_start(void)
{
  struct thread_attr_s attr = {
    .stack_size = 0x1000,
  };

  printk("Hello\n");

  ensure(thread_create(main, 0, &attr) == 0);
}

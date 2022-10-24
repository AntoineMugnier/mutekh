/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -O arch/efm32/include/arch/efm32/aes.h cdefs_use_reg_mask=1  \
     cdefs_use_field_setval=1 cdefs_use_field_shift=1 cdefs_use_field_set=1
*/

#ifndef _EFM32_AES_BFGEN_DEFS_
#define _EFM32_AES_BFGEN_DEFS_

#define EFM32_AES_CTRL_ADDR                          0x00000000
#define EFM32_AES_CTRL_MASK                          0x00000071
/** Select encryption or decryption. @multiple */
  #define EFM32_AES_CTRL_DECRYPT_SHIFT             0
  #define EFM32_AES_CTRL_DECRYPT(v)                ((EFM32_AES_CTRL_DECRYPT_##v) << 0)
  #define EFM32_AES_CTRL_DECRYPT_SET(x, v)         do { (x) = (((x) & ~0x1) | ((EFM32_AES_CTRL_DECRYPT_##v) << 0)); } while(0)
  #define EFM32_AES_CTRL_DECRYPT_SETVAL(x, v)      do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define EFM32_AES_CTRL_DECRYPT_GET(x)            (((x) >> 0) & 0x1)
    #define EFM32_AES_CTRL_DECRYPT_ENCRYPTION        0x00000000
    #define EFM32_AES_CTRL_DECRYPT_DECRYPTION        0x00000001
# if (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32LG) || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32WG) || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG) || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32G) || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32TG)
/** enable AES with 256 bits key instead of 128 bits @multiple */
  #define EFM32_AES_CTRL_AES256                    0x00000002
  #define EFM32_AES_CTRL_AES256_SHIFT              1
  #define EFM32_AES_CTRL_AES256_SET(x, v)          do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
/** enable key buffer in AES 128 mode @multiple */
  #define EFM32_AES_CTRL_KEYBUFEN                  0x00000004
  #define EFM32_AES_CTRL_KEYBUFEN_SHIFT            2
  #define EFM32_AES_CTRL_KEYBUFEN_SET(x, v)        do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
# endif
/** Set this bit to start encryption/decryption when DATA3 is written through
   AES_DATA. @multiple */
  #define EFM32_AES_CTRL_DATASTART                 0x00000010
  #define EFM32_AES_CTRL_DATASTART_SHIFT           4
  #define EFM32_AES_CTRL_DATASTART_SET(x, v)       do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
/** Set this bit to start encryption/decryption when DATA3 is written through
   AES_XORDATA. @multiple */
  #define EFM32_AES_CTRL_XORSTART                  0x00000020
  #define EFM32_AES_CTRL_XORSTART_SHIFT            5
  #define EFM32_AES_CTRL_XORSTART_SET(x, v)        do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
/** When set, the byte orders in the data and key registers are swapped before
   and after encryption/decryption. @multiple */
  #define EFM32_AES_CTRL_BYTEORDER                 0x00000040
  #define EFM32_AES_CTRL_BYTEORDER_SHIFT           6
  #define EFM32_AES_CTRL_BYTEORDER_SET(x, v)       do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)

#define EFM32_AES_CMD_ADDR                           0x00000004
#define EFM32_AES_CMD_MASK                           0x00000003
/** Set to start encryption/decryption. @multiple */
  #define EFM32_AES_CMD_START                      0x00000001
  #define EFM32_AES_CMD_START_SHIFT                0
  #define EFM32_AES_CMD_START_SET(x, v)            do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
/** Set to stop encryption/decryption. @multiple */
  #define EFM32_AES_CMD_STOP                       0x00000002
  #define EFM32_AES_CMD_STOP_SHIFT                 1
  #define EFM32_AES_CMD_STOP_SET(x, v)             do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)

#define EFM32_AES_STATUS_ADDR                        0x00000008
#define EFM32_AES_STATUS_MASK                        0x00000001
/** This bit indicates that the AES module is running an encryption/decryption.
   @multiple */
  #define EFM32_AES_STATUS_RUNNING                 0x00000001
  #define EFM32_AES_STATUS_RUNNING_SHIFT           0
  #define EFM32_AES_STATUS_RUNNING_SET(x, v)       do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)

#define EFM32_AES_IEN_ADDR                           0x0000000c
#define EFM32_AES_IEN_MASK                           0x00000001
/** Enable/disable interrupt on encryption/decryption done. @multiple */
  #define EFM32_AES_IEN_DONE                       0x00000001
  #define EFM32_AES_IEN_DONE_SHIFT                 0
  #define EFM32_AES_IEN_DONE_SET(x, v)             do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)

#define EFM32_AES_IF_ADDR                            0x00000010
#define EFM32_AES_IF_MASK                            0x00000001
/** Set when an encryption/decryption has finished. @multiple */
  #define EFM32_AES_IF_DONE                        0x00000001
  #define EFM32_AES_IF_DONE_SHIFT                  0
  #define EFM32_AES_IF_DONE_SET(x, v)              do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)

#define EFM32_AES_IFS_ADDR                           0x00000014
#define EFM32_AES_IFS_MASK                           0x00000001
/** Write to 1 to set encryption/decryption done interrupt flag. @multiple */
  #define EFM32_AES_IFS_DONE                       0x00000001
  #define EFM32_AES_IFS_DONE_SHIFT                 0
  #define EFM32_AES_IFS_DONE_SET(x, v)             do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)

#define EFM32_AES_IFC_ADDR                           0x00000018
#define EFM32_AES_IFC_MASK                           0x00000001
/** Write to 1 to clear encryption/decryption done interrupt flag. @multiple */
  #define EFM32_AES_IFC_DONE                       0x00000001
  #define EFM32_AES_IFC_DONE_SHIFT                 0
  #define EFM32_AES_IFC_DONE_SET(x, v)             do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)

#define EFM32_AES_DATA_ADDR                          0x0000001c
#define EFM32_AES_DATA_MASK                          0xffffffff
/** Access data through this register. @multiple */
  #define EFM32_AES_DATA_DATA_SHIFT                0
  #define EFM32_AES_DATA_DATA(v)                   ((v) << 0)
  #define EFM32_AES_DATA_DATA_SET(x, v)            do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define EFM32_AES_DATA_DATA_GET(x)               (((x) >> 0) & 0xffffffff)

#define EFM32_AES_XORDATA_ADDR                       0x00000020
#define EFM32_AES_XORDATA_MASK                       0xffffffff
/** Access data with XOR function through this register. @multiple */
  #define EFM32_AES_XORDATA_XORDATA_SHIFT          0
  #define EFM32_AES_XORDATA_XORDATA(v)             ((v) << 0)
  #define EFM32_AES_XORDATA_XORDATA_SET(x, v)      do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define EFM32_AES_XORDATA_XORDATA_GET(x)         (((x) >> 0) & 0xffffffff)

#define EFM32_AES_KEYL_ADDR(ridx)                    (0x00000030 + (ridx) * 4)
#define EFM32_AES_KEYL_COUNT                         4
#define EFM32_AES_KEYL_MASK                          0xffffffff
/** Access the low key words through this register. @multiple */
  #define EFM32_AES_KEYL_KEYL_SHIFT                0
  #define EFM32_AES_KEYL_KEYL(v)                   ((v) << 0)
  #define EFM32_AES_KEYL_KEYL_SET(x, v)            do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define EFM32_AES_KEYL_KEYL_GET(x)               (((x) >> 0) & 0xffffffff)

# if (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32LG) || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32WG) || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG) || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32G) || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32TG)
#define EFM32_AES_KEYH_ADDR(ridx)                    (0x00000040 + (ridx) * 4)
#define EFM32_AES_KEYH_COUNT                         4
#define EFM32_AES_KEYH_MASK                          0xffffffff
/** Access the high key words through this register. @multiple */
  #define EFM32_AES_KEYH_KEYL_SHIFT                0
  #define EFM32_AES_KEYH_KEYL(v)                   ((v) << 0)
  #define EFM32_AES_KEYH_KEYL_SET(x, v)            do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define EFM32_AES_KEYH_KEYL_GET(x)               (((x) >> 0) & 0xffffffff)
# endif

#endif

/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -I /Users/nipo/projects/bfgen/defs/inet/smi.bf               \
     -O /Users/nipo/projects/mutekh/drivers/include/drivers/phy/smi.h
*/

#ifndef _SMI_BFGEN_DEFS_
#define _SMI_BFGEN_DEFS_

#define SMI_CONTROL_ADDR                             0x00000000
/** When set, transmit from media independent interface regardless of whether the
   PHY has determined that a valid link has been established, else, transmit from
   media independent interface only when the PHY has determined that a valid link
   has been established @multiple */
  #define SMI_CONTROL_UNIDIRECTIONAL               0x0020
  #define SMI_CONTROL_SPEED_SELECT_1(v)            ((SMI_CONTROL_SPEED_SELECT_1_##v) << 6)
  #define SMI_CONTROL_SPEED_SELECT_1_SET(x, v)     do { (x) = (((x) & ~0x40) | ((SMI_CONTROL_SPEED_SELECT_1_##v) << 6)); } while(0)
  #define SMI_CONTROL_SPEED_SELECT_1_GET(x)        (((x) >> 6) & 0x1)
    #define SMI_CONTROL_SPEED_SELECT_1_10_1000       0x00000000
    #define SMI_CONTROL_SPEED_SELECT_1_100_RES       0x00000001
  #define SMI_CONTROL_COLLISION_TEST               0x0080
  #define SMI_CONTROL_DUPLEX_MODE(v)               ((SMI_CONTROL_DUPLEX_MODE_##v) << 8)
  #define SMI_CONTROL_DUPLEX_MODE_SET(x, v)        do { (x) = (((x) & ~0x100) | ((SMI_CONTROL_DUPLEX_MODE_##v) << 8)); } while(0)
  #define SMI_CONTROL_DUPLEX_MODE_GET(x)           (((x) >> 8) & 0x1)
/** Half duplex */
    #define SMI_CONTROL_DUPLEX_MODE_HALF             0x00000000
/** Full duplex */
    #define SMI_CONTROL_DUPLEX_MODE_FULL             0x00000001
  #define SMI_CONTROL_AUTONEG_RESTART              0x0200
  #define SMI_CONTROL_ISOLATE                      0x0400
  #define SMI_CONTROL_POWER_DOWN                   0x0800
  #define SMI_CONTROL_AUTONEG                      0x1000
/** Speed selection register, used with bit 6 @multiple */
  #define SMI_CONTROL_SPEED_SELECT_2(v)            ((SMI_CONTROL_SPEED_SELECT_2_##v) << 13)
  #define SMI_CONTROL_SPEED_SELECT_2_SET(x, v)     do { (x) = (((x) & ~0x2000) | ((SMI_CONTROL_SPEED_SELECT_2_##v) << 13)); } while(0)
  #define SMI_CONTROL_SPEED_SELECT_2_GET(x)        (((x) >> 13) & 0x1)
    #define SMI_CONTROL_SPEED_SELECT_2_10_100        0x00000000
    #define SMI_CONTROL_SPEED_SELECT_2_1000_RES      0x00000001
  #define SMI_CONTROL_LOOPBACK                     0x4000
  #define SMI_CONTROL_RESET                        0x8000

#define SMI_STATUS_ADDR                              0x00000001
/** Extended register capabilities @multiple */
  #define SMI_STATUS_EXTENDED_CAPABILITY           0x0001
/** Jabber condition detected @multiple */
  #define SMI_STATUS_JABBER_DETECT                 0x0002
/** Link is up @multiple */
  #define SMI_STATUS_LINK_UP                       0x0004
/** PHY is able to perform Auto-Negotiation @multiple */
  #define SMI_STATUS_AUTONEG_ABILITY               0x0008
/** Remote fault condition detected @multiple */
  #define SMI_STATUS_REMOTE_FAULT                  0x0010
/** Auto-Negotiation process completed @multiple */
  #define SMI_STATUS_AUTONEG_DONE                  0x0020
/** PHY will accept management frames with preamble suppressed @multiple */
  #define SMI_STATUS_MF_PREAMBLE_SUPPRESSION       0x0040
/** PHY able to transmit from media independent interface regardless of whether
   the PHY has determined that a valid link has been established @multiple */
  #define SMI_STATUS_UNIDIRECTIONAL_ABILITY        0x0080
/** Extended status information in Register 15 @multiple */
  #define SMI_STATUS_EXTENDED_STATUS               0x0100
/** PHY able to perform half duplex 100BASE-T2 @multiple */
  #define SMI_STATUS_CAN_100BASET2HD               0x0200
/** PHY able to perform full duplex 100BASE-T2 @multiple */
  #define SMI_STATUS_CAN_100BASET2FD               0x0400
/** PHY able to operate at 10 Mb/s in half duplex mode @multiple */
  #define SMI_STATUS_CAN_10HD                      0x0800
/** PHY able to operate at 10 Mb/s in full duplex mode @multiple */
  #define SMI_STATUS_CAN_10FD                      0x1000
/** PHY able to perform half duplex 100BASE-X @multiple */
  #define SMI_STATUS_CAN_100BASEXHD                0x2000
/** PHY able to perform full duplex 100BASE-X @multiple */
  #define SMI_STATUS_CAN_100BASEXFD                0x4000
/** PHY able to perform 100BASE-T4 @multiple */
  #define SMI_STATUS_CAN_100BASET4                 0x8000

#define SMI_PHY_ID0_ADDR                             0x00000002
  #define SMI_PHY_ID0_OUI_3_18(v)                  ((v) << 0)
  #define SMI_PHY_ID0_OUI_3_18_SET(x, v)           do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define SMI_PHY_ID0_OUI_3_18_GET(x)              (((x) >> 0) & 0xffff)

#define SMI_PHY_ID1_ADDR                             0x00000003
  #define SMI_PHY_ID1_REVISION_NUMBER(v)           ((v) << 0)
  #define SMI_PHY_ID1_REVISION_NUMBER_SET(x, v)    do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define SMI_PHY_ID1_REVISION_NUMBER_GET(x)       (((x) >> 0) & 0xf)
  #define SMI_PHY_ID1_MODEL_NUMBER(v)              ((v) << 4)
  #define SMI_PHY_ID1_MODEL_NUMBER_SET(x, v)       do { (x) = (((x) & ~0x3f0) | ((v) << 4)); } while(0)
  #define SMI_PHY_ID1_MODEL_NUMBER_GET(x)          (((x) >> 4) & 0x3f)
  #define SMI_PHY_ID1_OUI_19_24(v)                 ((v) << 10)
  #define SMI_PHY_ID1_OUI_19_24_SET(x, v)          do { (x) = (((x) & ~0xfc00) | ((v) << 10)); } while(0)
  #define SMI_PHY_ID1_OUI_19_24_GET(x)             (((x) >> 10) & 0x3f)

#define SMI_AUTONEG_ADV_ADDR                         0x00000004
  #define SMI_AUTONEG_ADV_SELECTOR(v)              ((SMI_AUTONEG_ADV_SELECTOR_##v) << 0)
  #define SMI_AUTONEG_ADV_SELECTOR_SET(x, v)       do { (x) = (((x) & ~0x1f) | ((SMI_AUTONEG_ADV_SELECTOR_##v) << 0)); } while(0)
  #define SMI_AUTONEG_ADV_SELECTOR_GET(x)          (((x) >> 0) & 0x1f)
/** IEEE 802.3 */
    #define SMI_AUTONEG_ADV_SELECTOR_IEEE8023        0x00000001
  #define SMI_AUTONEG_ADV_CAN_10BASET              0x0020
  #define SMI_AUTONEG_ADV_CAN_10BASETFD            0x0040
  #define SMI_AUTONEG_ADV_CAN_100BASETX            0x0080
  #define SMI_AUTONEG_ADV_CAN_100BASETXFD          0x0100
  #define SMI_AUTONEG_ADV_PAUSE(v)                 ((SMI_AUTONEG_ADV_PAUSE_##v) << 10)
  #define SMI_AUTONEG_ADV_PAUSE_SET(x, v)          do { (x) = (((x) & ~0xc00) | ((SMI_AUTONEG_ADV_PAUSE_##v) << 10)); } while(0)
  #define SMI_AUTONEG_ADV_PAUSE_GET(x)             (((x) >> 10) & 0x3)
/** No Pause */
    #define SMI_AUTONEG_ADV_PAUSE_NONE               0x00000000
/** Symmetric Pause */
    #define SMI_AUTONEG_ADV_PAUSE_SYMMETRIC          0x00000001
/** Asymmetric Pause */
    #define SMI_AUTONEG_ADV_PAUSE_ASYMMETRIC         0x00000002
/** Advertise support for both Symmetric PAUSE and Asymmetric PAUSE toward local device */
    #define SMI_AUTONEG_ADV_PAUSE_ADVERTISE          0x00000003
  #define SMI_AUTONEG_ADV_REMOTE_FAULT             0x2000

#define SMI_AUTONEG_PARTNER_ADDR                     0x00000005
  #define SMI_AUTONEG_PARTNER_SELECTOR(v)          ((SMI_AUTONEG_PARTNER_SELECTOR_##v) << 0)
  #define SMI_AUTONEG_PARTNER_SELECTOR_SET(x, v)   do { (x) = (((x) & ~0x1f) | ((SMI_AUTONEG_PARTNER_SELECTOR_##v) << 0)); } while(0)
  #define SMI_AUTONEG_PARTNER_SELECTOR_GET(x)      (((x) >> 0) & 0x1f)
/** IEEE 802.3 */
    #define SMI_AUTONEG_PARTNER_SELECTOR_IEEE8023    0x00000001
  #define SMI_AUTONEG_PARTNER_CAN_10BASET          0x0020
  #define SMI_AUTONEG_PARTNER_CAN_10BASETFD        0x0040
  #define SMI_AUTONEG_PARTNER_CAN_100BASETX        0x0080
  #define SMI_AUTONEG_PARTNER_CAN_100BASETXFD      0x0100
  #define SMI_AUTONEG_PARTNER_PAUSE                0x0400
  #define SMI_AUTONEG_PARTNER_REMOTE_FAULT         0x2000
  #define SMI_AUTONEG_PARTNER_ACKNOWLEDGE          0x4000
  #define SMI_AUTONEG_PARTNER_NEXT_PAGE            0x8000

#endif


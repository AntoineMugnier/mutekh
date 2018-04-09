#include <mutek/startup.h>

#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/mem_alloc.h>

#include <hexo/iospace.h>

#include <arch/cc26xx/memory_map.h>
#include <arch/cc26xx/init.h>
#include <arch/cc26xx/aux_smph.h>
#include <arch/cc26xx/fcfg1.h>
#include <arch/cc26xx/ddi_0_osc.h>
#include <arch/cc26xx/aon_wuc.h>
#include <arch/cc26xx/aux_wuc.h>
#include <arch/cc26xx/ccfg.h>
#include <arch/cc26xx/adi3_refsys.h>
#include <arch/cc26xx/aon_sysctl.h>
#include <arch/cc26xx/adi_4_aux.h>
#include <arch/cc26xx/flash.h>
#include <arch/cc26xx/prcm.h>
#include <arch/cc26xx/vims.h>
#include <arch/cc26xx/aon_rtc.h>

static inline void AuxAdiDdiSafeWrite(uint32_t nAddr, uint32_t nData)
{
    // Acquire semaphore for accessing ADI/DDI in AUX
    while (!(cpu_mem_read_32(CC26XX_AUX_SMPH_BASE +
      CC26XX_AUX_SMPH_SMPH0_ADDR)));
    //perform access
    cpu_mem_write_32(nAddr, nData);

    //release semaphore
    cpu_mem_write_32(CC26XX_AUX_SMPH_BASE + CC26XX_AUX_SMPH_SMPH0_ADDR, 1);
}

static void DDI16BitfieldWrite(uint32_t ui32Reg, uint32_t ui32Mask,
                                uint32_t ui32Shift, uint16_t ui32Data)
{
  uint32_t ui32RegAddr;
  uint32_t ui32WrData;

  //
  // 16-bit target is on 32-bit boundary so double offset.
  //
  ui32RegAddr = CC26XX_AUX_DDI0_OSC_BASE + (ui32Reg << 1) +
    CC26XX_DDI_O_MASK16B;

  //
  // Adjust for target bit in high half of the word.
  //
  if(ui32Shift >= 16)
  {
    ui32Shift = ui32Shift - 16;
    ui32RegAddr += 4;
    ui32Mask = ui32Mask >> 16;
  }

  //
  // Shift data in to position.
  //
  ui32WrData = ui32Data << ui32Shift;

  //
  // Write data.
  //
  AuxAdiDdiSafeWrite(ui32RegAddr, (ui32Mask << 16) | ui32WrData);
}

//******************************************************************************
//
//! \brief Sign extend the VDDR_TRIM setting
//! (special format ranging from -10 to +21)
//!
//! \return
//
//******************************************************************************
static int32_t SignExtendVddrTrimValue(uint32_t ui32VddrTrimVal)
{

  //
  // The VDDR trim value is 5 bits representing the range from -10 to +21
  // (where -10=0x16, -1=0x1F, 0=0x00, 1=0x01 and +21=0x15)
  //

  int32_t i32SignedVddrVal = ui32VddrTrimVal;

  if (i32SignedVddrVal > 0x15)
  {
    i32SignedVddrVal -= 0x20;
  }
  return (i32SignedVddrVal);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used for the ANABYPASS_VALUE1
//! register in OSC_DIG.
//
//******************************************************************************
static uint32_t GetTrimForAnabypassValue1()
{
  uint32_t  ui32Fcfg1Value;
  uint32_t  ui32XoscHfRow;
  uint32_t  ui32XoscHfCol;
  uint32_t  ui32TrimValue;

  // Use device specific trim values located in factory configuration
  // area for the XOSC_HF_COLUMN_Q12 and XOSC_HF_ROW_Q12 bit fields in
  // the ANABYPASS_VALUE1 register. Value for the other bit fields
  // are set to 0.

  ui32Fcfg1Value = cpu_mem_read_32(CC26XX_FCFG1_BASE +
    CC26XX_FCFG1_CONFIG_OSC_TOP_ADDR);

  ui32XoscHfRow =
    (ui32Fcfg1Value >> CC26XX_FCFG1_CONFIG_OSC_TOP_XOSC_HF_ROW_Q12_SHIFT) &
    0xf;

  ui32XoscHfCol =
    (ui32Fcfg1Value >> CC26XX_FCFG1_CONFIG_OSC_TOP_XOSC_HF_COLUMN_Q12_SHIFT) &
     0xf;

  ui32TrimValue = ((ui32XoscHfRow <<
    CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_SHIFT) |
    (ui32XoscHfCol << CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_SHIFT));

  return (ui32TrimValue);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used for the RCOSCLF_RTUNE_TRIM and the
//! RCOSCLF_CTUNE_TRIM bit fields in the XOSCLF_RCOSCLF_CTRL
//! register in OSC_DIG.
//
//******************************************************************************
static uint32_t GetTrimForRcOscLfRtuneCtuneTrim(void)
{
  uint32_t  ui32TrimValue;

  // Use device specific trim values located in factory configuration
  //area

  ui32TrimValue =
    ((cpu_mem_read_32(CC26XX_FCFG1_BASE + CC26XX_FCFG1_CONFIG_OSC_TOP_ADDR) &
    CC26XX_FCFG1_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM) >>
    CC26XX_FCFG1_CONFIG_OSC_TOP_RCOSCLF_CTUNE_TRIM_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_SHIFT;

  ui32TrimValue |=
    ((cpu_mem_read_32(CC26XX_FCFG1_BASE + CC26XX_FCFG1_CONFIG_OSC_TOP_ADDR) &
    CC26XX_FCFG1_CONFIG_OSC_TOP_RCOSCLF_RTUNE_TRIM) >>
    CC26XX_FCFG1_CONFIG_OSC_TOP_RCOSCLF_RTUNE_TRIM_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_SHIFT;

  return(ui32TrimValue);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used for the XOSC_HF_IBIASTHERM
//! bitfield in the ANABYPASS_VALUE2 register in OSC_DIG.
//
//******************************************************************************
static uint32_t GetTrimForXoscHfIbiastherm(void)
{
  uint32_t ui32TrimValue;

  // Use device specific trim value located in factory configuration
  // area

  ui32TrimValue =
    (cpu_mem_read_32(CC26XX_FCFG1_BASE + CC26XX_FCFG1_ANABYPASS_VALUE2_ADDR) &
    CC26XX_FCFG1_ANABYPASS_VALUE2_XOSC_HF_IBIASTHERM) >>
    CC26XX_FCFG1_ANABYPASS_VALUE2_XOSC_HF_IBIASTHERM_SHIFT;

  return(ui32TrimValue);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_TH2
//! register in OSC_DIG.
//
//******************************************************************************
static uint32_t GetTrimForAmpcompTh2(void)
{
  uint32_t  ui32TrimValue;
  uint32_t  ui32Fcfg1Value;

  // Use device specific trim value located in factory configuration
  // area. All defined register bit fields have corresponding trim
  // value in the factory configuration area

  ui32Fcfg1Value = cpu_mem_read_32(CC26XX_FCFG1_BASE +
    CC26XX_FCFG1_AMPCOMP_TH2_ADDR);

  ui32TrimValue = ((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH) >>
    CC26XX_FCFG1_AMPCOMP_TH2_LPMUPDATE_LTH_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_SHIFT;

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM) >>
    CC26XX_FCFG1_AMPCOMP_TH2_LPMUPDATE_HTM_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_SHIFT);

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM) >>
    CC26XX_FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_LPM_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_SHIFT);

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM) >>
    CC26XX_FCFG1_AMPCOMP_TH2_ADC_COMP_AMPTH_HPM_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_SHIFT);

  return(ui32TrimValue);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_TH1
//! register in OSC_DIG.
//
//******************************************************************************
static uint32_t GetTrimForAmpcompTh1(void)
{
  uint32_t  ui32TrimValue;
  uint32_t  ui32Fcfg1Value;

  // Use device specific trim values located in factory configuration
  // area. All defined register bit fields have a corresponding trim
  // value in the factory configuration area

  ui32Fcfg1Value = cpu_mem_read_32(CC26XX_FCFG1_BASE +
    CC26XX_FCFG1_AMPCOMP_TH1_ADDR);

  ui32TrimValue = (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH) >>
    CC26XX_FCFG1_AMPCOMP_TH1_HPMRAMP3_LTH_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_SHIFT);

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH) >>
    CC26XX_FCFG1_AMPCOMP_TH1_HPMRAMP3_HTH_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_SHIFT);

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT) >>
    CC26XX_FCFG1_AMPCOMP_TH1_IBIASCAP_LPTOHP_OL_CNT_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_SHIFT);

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_TH1_HPMRAMP1_TH) >>
    CC26XX_FCFG1_AMPCOMP_TH1_HPMRAMP1_TH_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH_SHIFT);

  return(ui32TrimValue);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used for the AMPCOMP_CTRL
//! register in OSC_DIG.
//
//******************************************************************************
static uint32_t GetTrimForAmpcompCtrl(uint32_t ui32Fcfg1Revision)
{
  uint32_t  ui32TrimValue;
  uint32_t  ui32Fcfg1Value;
  uint32_t  ibiasOffset;
  uint32_t  ibiasInit;

  // Use device specific trim values located in factory configuration
  // area. Register bit fields without trim values in the factory
  // configuration area will be set to the value of 0.

  ui32Fcfg1Value = cpu_mem_read_32(CC26XX_FCFG1_BASE +
    CC26XX_FCFG1_AMPCOMP_CTRL1_ADDR);

  ibiasOffset = (ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET) >>
    CC26XX_FCFG1_AMPCOMP_CTRL1_IBIAS_OFFSET_SHIFT;

  ibiasInit = (ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_CTRL1_IBIAS_INIT) >>
    CC26XX_FCFG1_AMPCOMP_CTRL1_IBIAS_INIT_SHIFT;

  ui32TrimValue = (ibiasOffset <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET_SHIFT) |
    (ibiasInit << CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_INIT_SHIFT);

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL) >>
    CC26XX_FCFG1_AMPCOMP_CTRL1_LPM_IBIAS_WAIT_CNT_FINAL_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_SHIFT);

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_CTRL1_CAP_STEP) >>
    CC26XX_FCFG1_AMPCOMP_CTRL1_CAP_STEP_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_CAP_STEP_SHIFT);

  ui32TrimValue |= (((ui32Fcfg1Value &
    CC26XX_FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT) >>
    CC26XX_FCFG1_AMPCOMP_CTRL1_IBIASCAP_HPTOLP_OL_CNT_SHIFT) <<
    CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_SHIFT);

  if (ui32Fcfg1Revision >= 0x00000022)
  {
    ui32TrimValue |= (((ui32Fcfg1Value &
      CC26XX_FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE) >>
      CC26XX_FCFG1_AMPCOMP_CTRL1_AMPCOMP_REQ_MODE_SHIFT) <<
      CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_REQ_MODE_SHIFT);
  }

  return(ui32TrimValue);
}

//******************************************************************************
//
//! \brief Returns the trim value from FCFG1 to be used as
//! DBLR_LOOP_FILTER_RESET_VOLTAGE setting.
//
//******************************************************************************
static uint32_t GetTrimForDblrLoopFilterResetVoltage(uint32_t ui32Fcfg1Revision)
{
  uint32_t dblrLoopFilterResetVoltageValue = 0; // Reset value

  if (ui32Fcfg1Revision >= 0x00000020)
  {
    dblrLoopFilterResetVoltageValue = (cpu_mem_read_32(CC26XX_FCFG1_BASE +
      CC26XX_FCFG1_MISC_OTP_DATA_1_ADDR) &
      CC26XX_FCFG1_MISC_OTP_DATA_1_DBLR_LOOP_FILTER_RESET_VOLTAGE) >>
      CC26XX_FCFG1_MISC_OTP_DATA_1_DBLR_LOOP_FILTER_RESET_VOLTAGE_SHIFT;
  }

   return (dblrLoopFilterResetVoltageValue);
}

//******************************************************************************
//
//! \brief Returns the trim value from FCFG1 to be used as
//! ADC_SH_MODE_EN setting.
//
//******************************************************************************
static uint32_t GetTrimForAdcShModeEn(uint32_t ui32Fcfg1Revision)
{
  uint32_t getTrimForAdcShModeEnValue = 1; // Recommended default setting

  if (ui32Fcfg1Revision >= 0x00000022)
  {
    getTrimForAdcShModeEnValue = (cpu_mem_read_32(CC26XX_FCFG1_BASE +
      CC26XX_FCFG1_OSC_CONF_ADDR) &
      CC26XX_FCFG1_OSC_CONF_ADC_SH_MODE_EN) >>
      CC26XX_FCFG1_OSC_CONF_ADC_SH_MODE_EN_SHIFT;
  }

  return (getTrimForAdcShModeEnValue);
}

//******************************************************************************
//
//! \brief Returns the trim value from FCFG1 to be used as
//! ADC_SH_VBUF_EN setting.
//
//******************************************************************************
static uint32_t GetTrimForAdcShVbufEn(uint32_t ui32Fcfg1Revision)
{
  uint32_t getTrimForAdcShVbufEnValue = 1; // Recommended default setting

  if (ui32Fcfg1Revision >= 0x00000022)
  {
    getTrimForAdcShVbufEnValue = (cpu_mem_read_32(CC26XX_FCFG1_BASE +
      CC26XX_FCFG1_OSC_CONF_ADDR) &
      CC26XX_FCFG1_OSC_CONF_ADC_SH_VBUF_EN) >>
      CC26XX_FCFG1_OSC_CONF_ADC_SH_VBUF_EN_SHIFT;
  }

  return (getTrimForAdcShVbufEnValue);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used for the XOSCHFCTL
//! register in OSC_DIG.
//
//******************************************************************************
static uint32_t GetTrimForXoscHfCtl(uint32_t ui32Fcfg1Revision)
{
  uint32_t getTrimForXoschfCtlValue = 0; // Recommended default setting
  uint32_t fcfg1Data;

  if (ui32Fcfg1Revision >= 0x00000020)
  {
    fcfg1Data = cpu_mem_read_32(CC26XX_FCFG1_BASE +
      CC26XX_FCFG1_MISC_OTP_DATA_1_ADDR);

    getTrimForXoschfCtlValue =
      (((fcfg1Data & CC26XX_FCFG1_MISC_OTP_DATA_1_PEAK_DET_ITRIM) >>
      CC26XX_FCFG1_MISC_OTP_DATA_1_PEAK_DET_ITRIM_SHIFT) <<
      CC26XX_AUX_DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_SHIFT);

    getTrimForXoschfCtlValue |=
      (((fcfg1Data & CC26XX_FCFG1_MISC_OTP_DATA_1_HP_BUF_ITRIM) >>
      CC26XX_FCFG1_MISC_OTP_DATA_1_HP_BUF_ITRIM_SHIFT) <<
      CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM_SHIFT);

    getTrimForXoschfCtlValue |=
      (((fcfg1Data & CC26XX_FCFG1_MISC_OTP_DATA_1_LP_BUF_ITRIM) >>
      CC26XX_FCFG1_MISC_OTP_DATA_1_LP_BUF_ITRIM_SHIFT) <<
      CC26XX_AUX_DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM_SHIFT);
  }

  return (getTrimForXoschfCtlValue);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used as OSC_DIG:CTL1.XOSC_HF_FAST_START.
//
//******************************************************************************
static uint32_t GetTrimForXoscHfFastStart(void)
{
  uint32_t ui32XoscHfFastStartValue   ;

  // Get value from FCFG1

  ui32XoscHfFastStartValue = (cpu_mem_read_32(CC26XX_FCFG1_BASE +
    CC26XX_FCFG1_OSC_CONF_ADDR) &
    CC26XX_FCFG1_OSC_CONF_XOSC_HF_FAST_START) >>
    CC26XX_FCFG1_OSC_CONF_XOSC_HF_FAST_START_SHIFT;

  return (ui32XoscHfFastStartValue);
}

//******************************************************************************
//
//! \brief Returns the trim value to be used for the RADCEXTCFG
//! register in OSC_DIG.
//
//******************************************************************************
static uint32_t GetTrimForRadcExtCfg(uint32_t ui32Fcfg1Revision)
{
  // Recommended default setting
  uint32_t getTrimForRadcExtCfgValue = 0x403F8000;
  uint32_t fcfg1Data;

  if (ui32Fcfg1Revision >= 0x00000020)
  {
    fcfg1Data = cpu_mem_read_32(CC26XX_FCFG1_BASE +
      CC26XX_FCFG1_MISC_OTP_DATA_1_ADDR);

    getTrimForRadcExtCfgValue =
      (((fcfg1Data & CC26XX_FCFG1_MISC_OTP_DATA_1_HPM_IBIAS_WAIT_CNT) >>
      CC26XX_FCFG1_MISC_OTP_DATA_1_HPM_IBIAS_WAIT_CNT_SHIFT) <<
      CC26XX_AUX_DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_SHIFT);

    getTrimForRadcExtCfgValue |=
      (((fcfg1Data & CC26XX_FCFG1_MISC_OTP_DATA_1_LPM_IBIAS_WAIT_CNT) >>
      CC26XX_FCFG1_MISC_OTP_DATA_1_LPM_IBIAS_WAIT_CNT_SHIFT) <<
      CC26XX_AUX_DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_SHIFT);

    getTrimForRadcExtCfgValue |=
      (((fcfg1Data & CC26XX_FCFG1_MISC_OTP_DATA_1_IDAC_STEP) >>
      CC26XX_FCFG1_MISC_OTP_DATA_1_IDAC_STEP_SHIFT) <<
      CC26XX_AUX_DDI0_OSC_RADCEXTCFG_IDAC_STEP_SHIFT);
  }

  return (getTrimForRadcExtCfgValue);
}

//******************************************************************************
//
//! \brief Returns the FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM.
//
//******************************************************************************
static uint32_t GetTrimForRcOscLfIBiasTrim(uint32_t ui32Fcfg1Revision)
{
  uint32_t trimForRcOscLfIBiasTrimValue = 0; // Default value

  if (ui32Fcfg1Revision >= 0x00000022)
  {
    trimForRcOscLfIBiasTrimValue = (cpu_mem_read_32(CC26XX_FCFG1_BASE +
      CC26XX_FCFG1_OSC_CONF_ADDR) &
      CC26XX_FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM) >>
      CC26XX_FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM_SHIFT;
  }

  return (trimForRcOscLfIBiasTrimValue);
}

//******************************************************************************
//
//! \brief Returns XOSCLF_REGULATOR_TRIM and XOSCLF_CMIRRWR_RATIO as one packet
//! spanning bits [5:0] in the returned value.
//
//******************************************************************************
static uint32_t
GetTrimForXoscLfRegulatorAndCmirrwrRatio(uint32_t ui32Fcfg1Revision)
{
  // Default value for both fields
  uint32_t trimForXoscLfRegulatorAndCmirrwrRatioValue = 0;

  if (ui32Fcfg1Revision >= 0x00000022)
  {
    trimForXoscLfRegulatorAndCmirrwrRatioValue =
      (cpu_mem_read_32(CC26XX_FCFG1_BASE + CC26XX_FCFG1_OSC_CONF_ADDR) &
      (CC26XX_FCFG1_OSC_CONF_XOSCLF_REGULATOR_TRIM |
      CC26XX_FCFG1_OSC_CONF_XOSCLF_CMIRRWR_RATIO)) >>
      CC26XX_FCFG1_OSC_CONF_XOSCLF_CMIRRWR_RATIO_SHIFT;
  }

  return (trimForXoscLfRegulatorAndCmirrwrRatioValue);
}

//******************************************************************************
//
//! Control the power to the AUX domain
//
//******************************************************************************
static void AUXWUCPowerCtrl(void)
{
  cpu_mem_write_32(CC26XX_AUX_WUC_BASE + CC26XX_AUX_WUC_PWROFFREQ_ADDR, 0x0);

  cpu_mem_write_32(CC26XX_AUX_WUC_BASE + CC26XX_AUX_WUC_PWRDWNREQ_ADDR,
    CC26XX_AUX_WUC_PWRDWNREQ_REQ);

  cpu_mem_write_32(CC26XX_AUX_WUC_BASE + CC26XX_AUX_WUC_MCUBUSCTL_ADDR,
    CC26XX_AUX_WUC_MCUBUSCTL_DISCONNECT_REQ);
}

//******************************************************************************
//
//! \brief Sync all accesses to the AON register interface.
//!
//! When this function returns, all writes to the AON register interface is
//! guaranteed to have progressed to hardware.
//!
//! \return None
//
//******************************************************************************
static inline void SysCtrlAonSync(void)
{
  //
  // Sync the AON interface
  //
  cpu_mem_read_32(CC26XX_AON_RTC_BASE + CC26XX_AON_RTC_SYNC_ADDR);
}

//******************************************************************************
//
//!  Configure the oscillator input to the a source clock.
//
//******************************************************************************
static void OSCClockSourceSet(void)
{
  //
  // Configure the low frequency source clock.
  //
  DDI16BitfieldWrite(CC26XX_AUX_DDI0_OSC_CTL0_ADDR,
    CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL,
    CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_SHIFT,
    0x2);
}

static void HapiTrimDeviceShutDown(uint32_t ui32Fcfg1Revision)
{
  uint32_t  ui32Trim;
  uint32_t  ccfg_ModeConfReg;
  int32_t   i32VddrSleepTrim;
  int32_t   i32VddrSleepDelta;

  //
  // Force AUX on and enable clocks
  //
  // No need to save the current status of the power/clock registers.
  // At this point both AUX and AON should have been reset to 0x0.
  //
  cpu_mem_write_32(CC26XX_AON_WUC_BASE + CC26XX_AON_WUC_AUXCTL_ADDR,
    CC26XX_AON_WUC_AUXCTL_AUX_FORCE_ON);

  //
  // Wait for power on on the AUX domain
  //
  while (!(cpu_mem_read_32(BIT_BAND_ADDR(CC26XX_AON_WUC_BASE +
    CC26XX_AON_WUC_PWRSTAT_ADDR, CC26XX_AON_WUC_PWRSTAT_AUX_PD_ON_SHIFT))));

  //
  // Enable the clocks for AUX_DDI0_OSC and AUX_ADI4
  //
  cpu_mem_write_32(CC26XX_AUX_WUC_BASE + CC26XX_AUX_WUC_MODCLKEN0_ADDR,
    CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC |
    CC26XX_AUX_WUC_MODCLKEN0_AUX_ADI4);

  //
  // Enable for JTAG to be powered down
  // (will still be powered on if debugger is connected)
  //
  cpu_mem_write_32(CC26XX_AON_WUC_BASE + CC26XX_AON_WUC_JTAGCFG_ADDR, 1);

  //
  // read the MODE_CONF register in CCFG
  //
  ccfg_ModeConfReg = cpu_mem_read_32(CC26XX_CCFG_BASE +
    CC26XX_CCFG_MODE_CONF_ADDR);

  //
  // Adjust the VDDR_TRIM_SLEEP value with value adjustable by customer
  // (CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA)
  //
  i32VddrSleepTrim = SignExtendVddrTrimValue((cpu_mem_read_32(
    CC26XX_FCFG1_BASE + CC26XX_FCFG1_LDO_TRIM_ADDR) &
    CC26XX_FCFG1_LDO_TRIM_VDDR_TRIM_SLEEP) >>
    CC26XX_FCFG1_LDO_TRIM_VDDR_TRIM_SLEEP_SHIFT);

  // Read and sign extend VddrSleepDelta (in range -8 to +7)
  i32VddrSleepDelta = ((((int32_t)ccfg_ModeConfReg) <<
    (32 - CC26XX_CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_WIDTH -
    CC26XX_CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_SHIFT)) >>
    (32 - CC26XX_CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_WIDTH));

  // Calculate new VDDR sleep trim
  i32VddrSleepTrim = (i32VddrSleepTrim + i32VddrSleepDelta + 1);
  if (i32VddrSleepTrim < -10)
  {
    i32VddrSleepTrim = -10;
  }

  // Write adjusted value using MASKED write (MASK8)
  cpu_mem_write_16(CC26XX_ADI3_BASE + CC26XX_ADI_O_MASK8B +
    (CC26XX_ADI_3_REFSYS_DCDCCTL1_ADDR * 2),
    (CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP << 8) |
    ((i32VddrSleepTrim << CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT) &
    CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP));

  //
  // set the RECHARGE source based upon CCFG:MODE_CONF:DCDC_RECHARGE
  // Note: Inverse polarity
  //
  cpu_mem_write_32(BIT_BAND_ADDR(CC26XX_AON_SYSCTL_BASE +
    CC26XX_AON_SYSCTL_PWRCTL_ADDR, CC26XX_AON_SYSCTL_PWRCTL_DCDC_EN_SHIFT),
    ((ccfg_ModeConfReg >> CC26XX_CCFG_MODE_CONF_DCDC_RECHARGE_SHIFT) & 1) ^ 1);

  //
  // set the ACTIVE source based upon CCFG:MODE_CONF:DCDC_ACTIVE
  // Note: Inverse polarity
  //
  cpu_mem_write_32(BIT_BAND_ADDR(CC26XX_AON_SYSCTL_BASE +
    CC26XX_AON_SYSCTL_PWRCTL_ADDR, CC26XX_AON_SYSCTL_PWRCTL_DCDC_ACTIVE_SHIFT),
    ((ccfg_ModeConfReg >> CC26XX_CCFG_MODE_CONF_DCDC_ACTIVE_SHIFT) & 1) ^ 1);

  //
  // Following sequence is required for using XOSCHF, if not included
  // devices crashes when trying to switch to XOSCHF.
  //
  // Trim CAP settings. Get and set trim value for the ANABYPASS_VALUE1
  // register
  ui32Trim = GetTrimForAnabypassValue1();
  AuxAdiDdiSafeWrite(CC26XX_AUX_DDI0_OSC_BASE +
    CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_ADDR, ui32Trim);

  // Trim RCOSC_LF. Get and set trim values for the RCOSCLF_RTUNE_TRIM and
  // RCOSCLF_CTUNE_TRIM fields in the XOSCLF_RCOSCLF_CTRL register.
  ui32Trim = GetTrimForRcOscLfRtuneCtuneTrim();
  DDI16BitfieldWrite(CC26XX_AUX_DDI0_OSC_LFOSCCTL_ADDR,
    (CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM |
    CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM),
    CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_SHIFT,
    ui32Trim);

  // Trim XOSCHF IBIAS THERM. Get and set trim value for the
  // XOSCHF IBIAS THERM bit field in the ANABYPASS_VALUE2 register. Other
  // register bit fields are set to 0.
  ui32Trim = GetTrimForXoscHfIbiastherm();
  AuxAdiDdiSafeWrite(CC26XX_AUX_DDI0_OSC_BASE +
    CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_ADDR,

  ui32Trim << CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_SHIFT);

  // Trim AMPCOMP settings required before switch to XOSCHF
  ui32Trim = GetTrimForAmpcompTh2();
  AuxAdiDdiSafeWrite(CC26XX_AUX_DDI0_OSC_BASE +
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADDR, ui32Trim);

  ui32Trim = GetTrimForAmpcompTh1();
  AuxAdiDdiSafeWrite(CC26XX_AUX_DDI0_OSC_BASE +
    CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_ADDR, ui32Trim);

  ui32Trim = GetTrimForAmpcompCtrl(ui32Fcfg1Revision);
  AuxAdiDdiSafeWrite(CC26XX_AUX_DDI0_OSC_BASE +
    CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_ADDR, ui32Trim);

  //
  // Set trim for DDI_0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_MODE_EN in accordance to
  // FCFG1 setting.
  // This is bit[5] in the CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADDR
  // register
  // Using MASK4 write + 1 => writing to bits[7:4]
  //
  ui32Trim = GetTrimForAdcShModeEn(ui32Fcfg1Revision);
  cpu_mem_write_8(CC26XX_AUX_DDI0_OSC_BASE + CC26XX_DDI_O_MASK4B +
    (CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADDR * 2) + 1,
    0x20 | (ui32Trim << 1));

  //
  // Set trim for DDI_0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_VBUF_EN in accordance to
  // FCFG1 setting
  // This is bit[4] in the CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADDR
  // register
  // Using MASK4 write + 1 => writing to bits[7:4]
  //
  ui32Trim = GetTrimForAdcShVbufEn(ui32Fcfg1Revision);
  cpu_mem_write_8(CC26XX_AUX_DDI0_OSC_BASE + CC26XX_DDI_O_MASK4B +
    (CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADDR * 2) + 1, 0x10 | (ui32Trim));

  //
  // Set trim for the PEAK_DET_ITRIM, HP_BUF_ITRIM and LP_BUF_ITRIM bit fields
  // in the DDI0_OSC_O_XOSCHFCTL register in accordance to FCFG1 setting.
  // Remaining register bit fields are set to their reset values of 0.
  //
  ui32Trim = GetTrimForXoscHfCtl(ui32Fcfg1Revision);
  AuxAdiDdiSafeWrite(CC26XX_AUX_DDI0_OSC_BASE +
    CC26XX_AUX_DDI0_OSC_XOSCHFCTL_ADDR, ui32Trim);

  //
  // Set trim for DBLR_LOOP_FILTER_RESET_VOLTAGE in accordance to FCFG1 setting
  // (This is bits [18:17] in CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADDR)
  // (Using MASK4 write + 4 => writing to bits[19:16] => (4*4))
  // (Assuming: DDI_0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_RESET_VOLTAGE_S
  // = 17 and that 
  //DDI_0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_RESET_VOLTAGE_M =0x00060000)
  //
  ui32Trim = GetTrimForDblrLoopFilterResetVoltage(ui32Fcfg1Revision);
  cpu_mem_write_8(CC26XX_AUX_DDI0_OSC_BASE + CC26XX_DDI_O_MASK4B +
    (CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADDR * 2) + 4,
    0x60 | (ui32Trim << 1));

  //
  // Update DDI_0_OSC_ATESTCTL_ATESTLF_RCOSCLF_IBIAS_TRIM with data from
  // FCFG1_OSC_CONF_ATESTLF_RCOSCLF_IBIAS_TRIM
  // This is DDI_0_OSC_O_ATESTCTL bit[7]
  // (DDI_0_OSC_O_ATESTCTL is currently hidden (but=0x00000020))
  // Using MASK4 write + 1 => writing to bits[7:4]
  //
  ui32Trim = GetTrimForRcOscLfIBiasTrim(ui32Fcfg1Revision);
  cpu_mem_write_8(CC26XX_AUX_DDI0_OSC_BASE + CC26XX_DDI_O_MASK4B +
    (0x00000020 * 2) + 1, 0x80 | (ui32Trim << 3));

  //
  // Update DDI_0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM and
  //        DDI_0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO in one write
  // This can be simplified since the registers are packed together in the same
  // order both in FCFG1 and in the HW register.
  // This spans DDI_0_OSC_O_LFOSCCTL bits[23:18]
  // Using MASK8 write + 4 => writing to bits[23:16]
  //
  ui32Trim = GetTrimForXoscLfRegulatorAndCmirrwrRatio(ui32Fcfg1Revision);
  cpu_mem_write_16(CC26XX_AUX_DDI0_OSC_BASE + CC26XX_DDI_O_MASK8B +
    (CC26XX_AUX_DDI0_OSC_LFOSCCTL_ADDR * 2) + 4, 0xFC00 | (ui32Trim << 2));

  //
  // Set trim the HPM_IBIAS_WAIT_CNT, LPM_IBIAS_WAIT_CNT and IDAC_STEP bit
  // fields in the DDI0_OSC_O_RADCEXTCFG register in accordance to FCFG1 setting
  // Remaining register bit fields are set to their reset values of 0.
  //
  ui32Trim = GetTrimForRadcExtCfg(ui32Fcfg1Revision);
  AuxAdiDdiSafeWrite(CC26XX_AUX_DDI0_OSC_BASE +
    CC26XX_AUX_DDI0_OSC_RADCEXTCFG_ADDR, ui32Trim);

  // Setting FORCE_KICKSTART_EN (ref. CC26_V1_BUG00261).
  // Should also be done for PG2
  // (This is bit 22 in CC26XX_AUX_DDI0_OSC_CTL0_ADDR)
  cpu_mem_write_8(CC26XX_AUX_DDI0_OSC_BASE + CC26XX_DDI_O_MASK4B +
    (CC26XX_AUX_DDI0_OSC_CTL0_ADDR * 2) + 5, 0x44);

  // XOSC source is a 24 MHz xtal (default)
  // Set bit DDI_0_OSC_CTL0_XTAL_IS_24M
  // (this is bit 31 in CC26XX_AUX_DDI0_OSC_CTL0_ADDR)
  cpu_mem_write_8(CC26XX_AUX_DDI0_OSC_BASE + CC26XX_DDI_O_MASK4B +
    (CC26XX_AUX_DDI0_OSC_CTL0_ADDR * 2) + 7, 0x88);

  // Setting DDI_0_OSC_CTL1_XOSC_HF_FAST_START according to value found in FCFG1
  ui32Trim = GetTrimForXoscHfFastStart();
  cpu_mem_write_8(CC26XX_AUX_DDI0_OSC_BASE + CC26XX_DDI_O_MASK4B +
    (0x00000004 * 2), 0x30 | ui32Trim);

  //
  // setup the LF clock
  //
  OSCClockSourceSet();

  //
  // Update ADI_4_AUX_ADCREF1_VTRIM with value from FCFG1
  //
  cpu_mem_write_8(CC26XX_AUX_ADI4_BASE + CC26XX_AUX_ADI4_ADCREF1_ADDR,
    ((cpu_mem_read_32(CC26XX_FCFG1_BASE +
    CC26XX_FCFG1_SOC_ADC_REF_TRIM_AND_OFFSET_EXT_ADDR) >>
    CC26XX_FCFG1_SOC_ADC_REF_TRIM_AND_OFFSET_EXT_SOC_ADC_REF_VOLTAGE_TRIM_TEMP1_SHIFT) <<
    CC26XX_AUX_ADI4_ADCREF1_VTRIM_SHIFT) &
    CC26XX_AUX_ADI4_ADCREF1_VTRIM);

  //
  // Set ADI_4_AUX:ADC0.SMPL_CYCLE_EXP to it's default minimum value (=3)
  // (Note: Using MASK8B requires that the bits to be modified must be within
  // the same byte boundary which is the case
  // for the ADI_4_AUX_ADC0_SMPL_CYCLE_EXP field)
  //
  cpu_mem_write_16(CC26XX_AUX_ADI4_BASE + CC26XX_ADI_O_MASK8B +
    (CC26XX_AUX_ADI4_ADC0_ADDR * 2),
    (CC26XX_AUX_ADI4_ADC0_SMPL_CYCLE_EXP << 8) |
    (3 << CC26XX_AUX_ADI4_ADC0_SMPL_CYCLE_EXP_SHIFT));

  //
  // Sync with AON
  //
  SysCtrlAonSync();

  //
  // Allow AUX to power down
  //
  AUXWUCPowerCtrl();

  //
  //Leaving on AUX and clock for AUX_DDI0_OSC on but turn off clock for AUX_ADI4
  //
  cpu_mem_write_32(CC26XX_AUX_WUC_BASE + CC26XX_AUX_WUC_MODCLKEN0_ADDR,
    CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC);

  // Disable EFUSE clock
  cpu_mem_write_32(BIT_BAND_ADDR(CC26XX_FLASH_BASE + CC26XX_FLASH_CFG_ADDR,
    CC26XX_FLASH_CFG_DIS_EFUSECLK_SHIFT), 1);
}

void cc26xx_ti_init(void)
{
  uint32_t  ui32Fcfg1Revision;
  uint32_t  ui32AonSysResetctl;
  uint32_t  ui32FlashFpac1;
  uint32_t  ui32VimsCtl;

  //
  // Get layout revision of the factory configuration area
  // (Handle undefined revision as revision = 0)
  //
  ui32Fcfg1Revision = cpu_mem_read_32(CC26XX_FCFG1_BASE +
    CC26XX_FCFG1_FCFG1_REVISION_ADDR);

  if (ui32Fcfg1Revision == 0xFFFFFFFF)
  {
    ui32Fcfg1Revision = 0;
  }

  //
  // Enable standby in flash bank
  //
  cpu_mem_write_32(BIT_BAND_ADDR(CC26XX_FLASH_BASE + CC26XX_FLASH_CFG_ADDR,
    CC26XX_FLASH_CFG_DIS_STANDBY_SHIFT), 0);

  //
  // Clock must always be enabled for the semaphore module
  // (due to ADI/DDI HW workaround)
  //
  cpu_mem_write_32(CC26XX_AUX_WUC_BASE + CC26XX_AUX_WUC_MODCLKEN1_ADDR,
    CC26XX_AUX_WUC_MODCLKEN1_SMPH_EN);

  //
  // Warm resets on CC26XX complicates software design as much of our software
  // expect that initialization is done from a full system reset.
  // This includes RTC setup, oscillator configuration and AUX setup.
  // To ensure a full reset of the device is done when customers
  // get e.g. a Watchdog, reset, the following is set here:
  //
  cpu_mem_write_32(BIT_BAND_ADDR(CC26XX_PRCM_BASE + CC26XX_PRCM_WARMRESET_ADDR,
    CC26XX_PRCM_WARMRESET_WR_TO_PINRESET_SHIFT), 1);

  //
  // Select correct CACHE mode and set correct CACHE configuration
  //
  while (cpu_mem_read_32(BIT_BAND_ADDR(CC26XX_VIMS_BASE + CC26XX_VIMS_STAT_ADDR,
    CC26XX_VIMS_STAT_MODE_CHANGING_SHIFT)))
  {
    // Do nothing - wait for an eventual ongoing mode change to complete.
  }
  ui32VimsCtl = cpu_mem_read_32(CC26XX_VIMS_BASE + CC26XX_VIMS_CTL_ADDR);
  cpu_mem_write_32(CC26XX_VIMS_BASE + CC26XX_VIMS_CTL_ADDR,
    (ui32VimsCtl & ~(CC26XX_VIMS_CTL_MODE)) |
    CC26XX_VIMS_CTL_MODE_CACHE |
    CC26XX_VIMS_CTL_DYN_CG_EN |
    CC26XX_VIMS_CTL_PREF_EN);

  HapiTrimDeviceShutDown(ui32Fcfg1Revision);

  //
  // Set VIMS power domain control.
  // PDCTL1VIMS = 0 ==> VIMS power domain is only powered when CPU power domain
  // is powered
  //
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL1VIMS_ADDR, 0);

  //
  // Configure optimal wait time for flash FSM in cases where flash pump
  // wakes up from sleep
  //
  ui32FlashFpac1 = cpu_mem_read_32(CC26XX_FLASH_BASE + CC26XX_FLASH_FPAC1_ADDR);
  ui32FlashFpac1 &= ~CC26XX_FLASH_FPAC1_PSLEEPTDIS;
  ui32FlashFpac1 |= 0x139 << CC26XX_FLASH_FPAC1_PSLEEPTDIS_SHIFT;
  cpu_mem_write_32(CC26XX_FLASH_BASE + CC26XX_FLASH_FPAC1_ADDR, ui32FlashFpac1);

  //
  // And finally at the end of the flash boot process:
  // SET BOOT_DET bits in AON_SYSCTL to 3 if already found to be 1
  // Note: The BOOT_DET_x_CLR/SET bits must be manually cleared
  //
  if (((cpu_mem_read_32(CC26XX_AON_SYSCTL_BASE +
    CC26XX_AON_SYSCTL_RESETCTL_ADDR) &
    (CC26XX_AON_SYSCTL_RESETCTL_BOOT_DET_1 |
    CC26XX_AON_SYSCTL_RESETCTL_BOOT_DET_0)) >>
    CC26XX_AON_SYSCTL_RESETCTL_BOOT_DET_0_SHIFT) == 1)
  {
    ui32AonSysResetctl = (cpu_mem_read_32(CC26XX_AON_SYSCTL_BASE +
      CC26XX_AON_SYSCTL_RESETCTL_ADDR) &
      ~(CC26XX_AON_SYSCTL_RESETCTL_BOOT_DET1_CLR |
      CC26XX_AON_SYSCTL_RESETCTL_BOOT_DET0_CLR |
      CC26XX_AON_SYSCTL_RESETCTL_BOOT_DET1_SET |
      CC26XX_AON_SYSCTL_RESETCTL_BOOT_DET0_SET));

    cpu_mem_write_32(CC26XX_AON_SYSCTL_BASE + CC26XX_AON_SYSCTL_RESETCTL_ADDR,
      ui32AonSysResetctl | CC26XX_AON_SYSCTL_RESETCTL_BOOT_DET1_SET);

    cpu_mem_write_32(CC26XX_AON_SYSCTL_BASE + CC26XX_AON_SYSCTL_RESETCTL_ADDR,
      ui32AonSysResetctl);
  }

  //
  // Make sure there are no ongoing VIMS mode change when leaving trimDevice()
  // (There should typically be no wait time here, but need to be sure)
  //
  while (cpu_mem_read_32(BIT_BAND_ADDR(CC26XX_VIMS_BASE +
    CC26XX_VIMS_STAT_ADDR, CC26XX_VIMS_STAT_MODE_CHANGING_SHIFT)))
  {
      // Do nothing - wait for an eventual ongoing mode change to complete.
  }
}


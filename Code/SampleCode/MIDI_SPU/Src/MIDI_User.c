#include "GPCM3_FM1.h"
#include "MIDI.h"
#include "MergedRes.h"

uint16_t R_SPU_STS_CH0_15_PRE;
uint16_t R_SPU_STS_CH16_31_PRE;
uint16_t R_SPU_STS_CH32_47_PRE;
uint16_t R_SPU_STS_CH48_63_PRE;

SPU_ChannelInfoStruct SPU_ChannelInfo[SPU_ChannelNumber];

void MIDI_CB_Init()
{
  DAC_Open();
	DAC_CH0_Init(DAC_TRG_SEL_TM0);
	DAC_CH1_Init(DAC_TRG_SEL_TM0);
  DAC_AudioPwm_Open();
  DAC_SetAudPwmGain(0x1F);
  DAC_VoltageDAC_CH0_Enable();
  DAC_VoltageDAC_CH1_Enable();
}

void MIDI_CB_ISR()
{
// clear ISR flag
  WRITE_REG(SPU->BEAT_COUNTER, 0xC001);
}

void MIDI_CB_StartPlay()
{
  SPU_AllChannelStop();
  SPU_Initial();
}

void MIDI_CB_StopPlay()
{
  SPU_AllChannelStop();
}

void MIDI_CB_Pause()
{
  SPU_AllChannelStop();
}

void MIDI_CB_Resume()
{

}

uint64_t MIDI_User_SetSpuMask()
{
#if(SPU_ChannelNumber == 64)
  return 0xFFFFFFFFFFFFFFFF;                    // all channel available
#endif
#if(SPU_ChannelNumber == 32)
  return 0xFFFFFFFF;                          // 32 ch available
#endif
#if(SPU_ChannelNumber == 16)
  return 0xFFFF;                              // 16 ch available
#endif
#if(SPU_ChannelNumber == 8)
  return 0xFF;                                // 8 ch available
#endif
}

uint16_t MIDI_User_GetSpuCtrlFlag()
{
  return (SPU->CTRL_FLAG);
}

uint16_t MIDI_User_GetSpuClock()
{
#if (SPU_CLK == 96000000)
  return  1;                                // SPU clock = 96MHz, Use 12MHz XTAL
#endif
  return  0;                                // SPU clock = 98.304MHz, Use Internal Clock or 12.228MHz XTAL
}

uint32_t MIDI_User_GetResourceAddress()
{
  return (uint32_t)SEC_START_ADDR;          // from FileMerger.h
}

uint32_t MIDI_FindEmptyChannel(uint64_t R_SPU_Available)
{
  uint64_t Temp;
  uint16_t ChannelIndex;
  uint32_t MinDuration;
  int32_t ChannelDuration;
  uint16_t R_SPU_Available_CH0_15;
  uint16_t R_SPU_Available_CH16_31;
  uint16_t R_SPU_Available_CH32_47;
  uint16_t R_SPU_Available_CH48_63;

  R_SPU_Available_CH0_15 = R_SPU_Available & 0xFFFF;
  R_SPU_Available_CH16_31 = (R_SPU_Available >> 16) & 0xFFFF;
  R_SPU_Available_CH32_47 = (R_SPU_Available >> 32) & 0xFFFF;
  R_SPU_Available_CH48_63 = (R_SPU_Available >> 48) & 0xFFFF;

// Find Idle Channel
  Temp = (READ_REG(SPU->CH_STATUS_CH0_15) ^ 0xFFFF) & R_SPU_Available_CH0_15;       // Temp_bit = 1 is idle channel
  if (Temp != 0x0000)
  {
    for(ChannelIndex = 0; ChannelIndex < 16; ChannelIndex++)
    {
      if(Temp & 0x0001)
      {
        return ChannelIndex;
      }
      else
      {
        Temp = Temp >> 1;
      }
    }
  }
  Temp = (READ_REG(SPU->CH_STATUS_CH16_31) ^ 0xFFFF) & R_SPU_Available_CH16_31;       // Temp_bit = 1 is idle channel
  if (Temp != 0x0000)
  {
    for(ChannelIndex = 16; ChannelIndex < 32; ChannelIndex++)
    {
      if(Temp & 0x0001)
      {
        return ChannelIndex;
      }
      else
      {
        Temp = Temp >> 1;
      }
    }
  }
  Temp = (READ_REG(SPU->CH_STATUS_CH32_47) ^ 0xFFFF) & R_SPU_Available_CH32_47;       // Temp_bit = 1 is idle channel
  if (Temp != 0x0000)
  {
    for(ChannelIndex = 32; ChannelIndex < 48; ChannelIndex++)
    {
      if(Temp & 0x0001)
      {
        return ChannelIndex;
      }
      else
      {
        Temp = Temp >> 1;
      }
    }
  }
  Temp = (READ_REG(SPU->CH_STATUS_CH48_63) ^ 0xFFFF) & R_SPU_Available_CH32_47;       // Temp_bit = 1 is idle channel
  if (Temp != 0x0000)
  {
    for(ChannelIndex = 48; ChannelIndex < 64; ChannelIndex++)
    {
      if(Temp & 0x0001)
      {
        return ChannelIndex;
      }
      else
      {
        Temp = Temp >> 1;
      }
    }
  }


//// Find Idle Channel
//  Temp = READ_REG(SPU->CH_STATUS_CH0_15);
//  if (Temp != 0xFFFF)                               // Temp = 0xFFFFFFFF, all channel busy
//  {
//    for(ChannelIndex = 0; ChannelIndex < 16; ChannelIndex++)
//    {
//      if((Temp & 0x0001) == 0)
//      {
//        return ChannelIndex;
//      }
//      else
//      {
//        Temp = Temp >> 1;
//      }
//    }
//  }
//  Temp = READ_REG(SPU->CH_STATUS_CH16_31);
//  if (Temp != 0xFFFF)                               // Temp = 0xFFFFFFFF, all channel busy
//  {
//    for(ChannelIndex = 16; ChannelIndex < 32; ChannelIndex++)
//    {
//      if((Temp & 0x0001) == 0)
//      {
//        return ChannelIndex;
//      }
//      else
//      {
//        Temp = Temp >> 1;
//      }
//    }
//  }
//  Temp = READ_REG(SPU->CH_STATUS_CH32_47);
//  if (Temp != 0xFFFF)                               // Temp = 0xFFFFFFFF, all channel busy
//  {
//    for(ChannelIndex = 32; ChannelIndex < 48; ChannelIndex++)
//    {
//      if((Temp & 0x0001) == 0)
//      {
//        return ChannelIndex;
//      }
//      else
//      {
//        Temp = Temp >> 1;
//      }
//    }
//  }
//  Temp = READ_REG(SPU->CH_STATUS_CH48_63);
//  if (Temp != 0xFFFF)                               // Temp = 0xFFFFFFFF, all channel busy
//  {
//    for(ChannelIndex = 48; ChannelIndex < 64; ChannelIndex++)
//    {
//      if((Temp & 0x0001) == 0)
//      {
//        return ChannelIndex;
//      }
//      else
//      {
//        Temp = Temp >> 1;
//      }
//    }
//  }

// No Idle Channel, Find Release Channel
  Temp = READ_REG(SPU->ENV_RAMPDOWN_CH0_15) & R_SPU_Available_CH0_15;
  if (Temp != 0x0000)                                 // Temp = 0x00000000, all channel release
  {
    for(ChannelIndex = 0; ChannelIndex < 16; ChannelIndex++)
    {
      if(Temp & 0x0001)
      {
        // Close Release CH for Note Play
        MIDI_SetSampleRate(ChannelIndex, 0x40000);
        MIDI_SetChannelDisable(ChannelIndex);
        CLEAR_BIT(SPU->ENV_RAMPDOWN_CH0_15, 0x0001 << ChannelIndex);
        while(READ_REG(SPU->CH_STATUS_CH0_15) & (0x0001 << ChannelIndex));
        return ChannelIndex;
      }
      else
      {
        Temp = Temp >> 1;
      }
    }
  }

  Temp = READ_REG(SPU->ENV_RAMPDOWN_CH16_31) & R_SPU_Available_CH16_31;
  if (Temp != 0x0000)                                 // Temp = 0x00000000, all channel release
  {
    for(ChannelIndex = 16; ChannelIndex < 32; ChannelIndex++)
    {
      if(Temp & 0x0001)
      {
        // Close Release CH for Note Play
        MIDI_SetSampleRate(ChannelIndex, 0x40000);
        MIDI_SetChannelDisable(ChannelIndex);
        CLEAR_BIT(SPU->ENV_RAMPDOWN_CH16_31, 0x0001 << (ChannelIndex - 16));
        while(READ_REG(SPU->CH_STATUS_CH16_31) & (0x0001 << (ChannelIndex - 16)));
        return ChannelIndex;
      }
      else
      {
        Temp = Temp >> 1;
      }
    }
  }

  Temp = READ_REG(SPU->ENV_RAMPDOWN_CH32_47) & R_SPU_Available_CH32_47;
  if (Temp != 0x0000)                                 // Temp = 0x00000000, all channel release
  {
    for(ChannelIndex = 32; ChannelIndex < 48; ChannelIndex++)
    {
      if(Temp & 0x0001)
      {
        // Close Release CH for Note Play
        MIDI_SetSampleRate(ChannelIndex, 0x40000);
        MIDI_SetChannelDisable(ChannelIndex);
        CLEAR_BIT(SPU->ENV_RAMPDOWN_CH32_47, 0x0001 << (ChannelIndex - 32));
        while(READ_REG(SPU->CH_STATUS_CH32_47) & (0x0001 << (ChannelIndex - 32)));
        return ChannelIndex;
      }
      else
      {
        Temp = Temp >> 1;
      }
    }
  }

  Temp = READ_REG(SPU->ENV_RAMPDOWN_CH48_63) & R_SPU_Available_CH48_63;
  if (Temp != 0x0000)                                 // Temp = 0x00000000, all channel release
  {
    for(ChannelIndex = 48; ChannelIndex < 64; ChannelIndex++)
    {
      if(Temp & 0x0001)
      {
        // Close Release CH for Note Play
        MIDI_SetSampleRate(ChannelIndex, 0x40000);
        MIDI_SetChannelDisable(ChannelIndex);
        CLEAR_BIT(SPU->ENV_RAMPDOWN_CH48_63, 0x0001 << (ChannelIndex - 48));
        while(READ_REG(SPU->CH_STATUS_CH48_63) & (0x0001 << (ChannelIndex - 48)));
        return ChannelIndex;
      }
      else
      {
        Temp = Temp >> 1;
      }
    }
  }

// No Release Channel, Find Minimum Duration Channel
  MinDuration = 0xFFFF;
  for(Temp = 0; Temp < 64; Temp++)
  {
    if (R_SPU_Available & (0x0001 << Temp))
    {
      ChannelDuration = GetChannelDuration(Temp);
      if (ChannelDuration == 0)
      {
        ChannelIndex = Temp;
        break;
      }
      if (MinDuration > ChannelDuration)
      {
        MinDuration = ChannelDuration;
        ChannelIndex = Temp;
      }
    }
  }
  MIDI_SetChannelDisable(ChannelIndex);
  if (ChannelIndex < 16)
  {
    while(READ_REG(SPU->CH_STATUS_CH0_15) & (0x0001 << ChannelIndex));
  }
  else if (ChannelIndex < 32)
  {
    while(READ_REG(SPU->CH_STATUS_CH16_31) & (0x0001 << (ChannelIndex - 16)));
  }
  else if (ChannelIndex < 48)
  {
    while(READ_REG(SPU->CH_STATUS_CH32_47) & (0x0001 << (ChannelIndex - 32)));
  }
  else
  {
    while(READ_REG(SPU->CH_STATUS_CH48_63) & (0x0001 << (ChannelIndex - 48)));
  }
  return ChannelIndex;
}

void MIDI_StopExpiredCH()
{
  uint16_t Temp;

  Temp = READ_REG(SPU->CH_STATUS_CH0_15);
  if (Temp != R_SPU_STS_CH0_15_PRE)
  {
    R_SPU_STS_CH0_15_PRE = Temp;
    Temp ^= 0xFFFF;
    MODIFY_REG(SPU->SPU_EN_CH0_15, Temp, 0x0000);
  }

  Temp = READ_REG(SPU->CH_STATUS_CH16_31);
  if (Temp != R_SPU_STS_CH16_31_PRE)
  {
    R_SPU_STS_CH16_31_PRE = Temp;
    Temp ^= 0xFFFF;
    MODIFY_REG(SPU->SPU_EN_CH16_31, Temp, 0x0000);
  }

  Temp = READ_REG(SPU->CH_STATUS_CH32_47);
  if (Temp != R_SPU_STS_CH32_47_PRE)
  {
    R_SPU_STS_CH32_47_PRE = Temp;
    Temp ^= 0xFFFF;
    MODIFY_REG(SPU->SPU_EN_CH32_47, Temp, 0x0000);
  }

  Temp = READ_REG(SPU->CH_STATUS_CH48_63);
  if (Temp != R_SPU_STS_CH48_63_PRE)
  {
    R_SPU_STS_CH48_63_PRE = Temp;
    Temp ^= 0xFFFF;
    MODIFY_REG(SPU->SPU_EN_CH48_63, Temp, 0x0000);
  }
}

void MIDI_SetSampleRate(uint32_t Channel_Number, uint32_t PhaseAdder)
{
  uint32_t PhaseAddrt18_16;
  uint32_t PhaseAdder15_0;
  SPU_CH_TYPE_DEF *SPU_CH;

  ChannelToSPUCH(Channel_Number, SPU_CH);

  WRITE_REG(SPU_CH->PHASE_ACCH, 0x0007);
  WRITE_REG(SPU_CH->PHASE_ACC, 0xFFFF);


  PhaseAdder15_0 = PhaseAdder & 0x0000FFFF;
  PhaseAddrt18_16 = (PhaseAdder & 0x00070000) >> 16;
  WRITE_REG(SPU_CH->PHASE_VALUEH, PhaseAddrt18_16);
  WRITE_REG(SPU_CH->PHASE_VALUE, PhaseAdder15_0);
}

void MIDI_SetNewPhase(uint32_t Channel_Number, uint32_t PhaseAdder)
{
  uint32_t PhaseAddrt18_16;
  uint32_t PhaseAdder15_0;
  SPU_CH_TYPE_DEF *SPU_CH;

  ChannelToSPUCH(Channel_Number, SPU_CH);
  PhaseAdder15_0 = PhaseAdder & 0x0000FFFF;
  PhaseAddrt18_16 = (PhaseAdder & 0x00070000) >> 16;
  WRITE_REG(SPU_CH->PHASE_VALUEH, PhaseAddrt18_16);
  WRITE_REG(SPU_CH->PHASE_VALUE, PhaseAdder15_0);
}


void MIDI_SetInstrumentAddress(uint32_t Channel_Number,uint32_t START_ADDR,uint32_t LOOP_ADDR)
{
  SPU_CH_TYPE_DEF *SPU_CH;
  uint32_t ADDR_Bit30_28;
  uint32_t ADDR_Bit27_22;
  uint32_t ADDR_Bit21_16;
  uint32_t ADDR_Bit15_0;

  ChannelToSPUCH(Channel_Number, SPU_CH);
  WRITE_REG(SPU_CH->WAVE_DATA, 0x8000);  		         //16 bit data
  WRITE_REG(SPU_CH->WAVE_DATA0, 0x8000);  		       //16 bit data
  ADDR_Bit15_0 = START_ADDR & 0x0000FFFF;
  ADDR_Bit21_16 = START_ADDR & 0x003F0000;
  ADDR_Bit27_22 = START_ADDR & 0x0FC00000;
  ADDR_Bit30_28 = START_ADDR & 0x70000000;

  MODIFY_REG(SPU_CH->WAVE_ADDR, 0xFFFF, ADDR_Bit15_0);
  MODIFY_REG(SPU_CH->WAVE_MODE, 0x003F, ADDR_Bit21_16 >> 16);
  MODIFY_REG(SPU_CH->WAVE_ADDRH, 0x703F, (ADDR_Bit27_22 >> 22) + (ADDR_Bit30_28 >> 16));

  ADDR_Bit15_0 = LOOP_ADDR & 0x0000FFFF;
  ADDR_Bit21_16 = LOOP_ADDR & 0x003F0000;
  ADDR_Bit27_22 = LOOP_ADDR & 0x0FC00000;
  ADDR_Bit30_28 = LOOP_ADDR & 0x70000000;

  MODIFY_REG(SPU_CH->WAVE_LOOPADDR, 0xFFFF, ADDR_Bit15_0);
  MODIFY_REG(SPU_CH->WAVE_MODE, 0x0FC0, ADDR_Bit21_16 >> 10);
  MODIFY_REG(SPU_CH->WAVE_ADDRH, 0x0FC0, (ADDR_Bit27_22 >> 16) + (ADDR_Bit30_28 >> 16));
}

void MIDI_SetEnvelopeAddress(uint32_t Channel_Number, uint32_t EnvAddr)
{
  uint16_t *pAddr;
  uint32_t Temp;
  uint32_t Temp1;
  uint32_t ADDR_Bit15_0;
  uint32_t ADDR_Bit21_16;
  uint32_t ADDR_Bit30_22;
  SPU_CH_TYPE_DEF *SPU_CH;

  ChannelToSPUCH(Channel_Number, SPU_CH);

  pAddr = (uint32_t *)EnvAddr;
  Temp = *pAddr;
  Temp1 = *(pAddr + 1);
  EnvAddr = EnvAddr >> 1;
  WRITE_REG(SPU_CH->ENV_CTRL0, Temp);
  WRITE_REG(SPU_CH->ENV_CTRL1, Temp1);
  WRITE_REG(SPU_CH->ENV_DATA, 0x0000);            //7 bit[6:0]

  ADDR_Bit15_0 = EnvAddr & 0x0000FFFF;
  ADDR_Bit21_16 = EnvAddr & 0x003F0000;
  ADDR_Bit30_22 = EnvAddr & 0xFFC00000;
  WRITE_REG(SPU_CH->ENV_ADDR, ADDR_Bit15_0);
  MODIFY_REG(SPU_CH->ENV_INT_CTRL, 0x003F, (ADDR_Bit21_16) >> 16);
  WRITE_REG(SPU_CH->ENV_ADDRH, ADDR_Bit30_22 >> 22);
}

void MIDI_SetEnvReleaseClock(uint32_t Channel_Number, uint32_t R_EnvelopeReleaseClock)
{
  SPU_CH_TYPE_DEF *SPU_CH;
  ChannelToSPUCH(Channel_Number, SPU_CH);
  WRITE_REG(SPU_CH->ENV_RAMPDOWN_CLK, R_EnvelopeReleaseClock);
}

void MIDI_SetEnvReleaseOffset(uint32_t Channel_Number, uint32_t R_EnvelopeReleaseOffset)
{
  SPU_CH_TYPE_DEF *SPU_CH;
  ChannelToSPUCH(Channel_Number, SPU_CH);
  WRITE_REG(SPU_CH->ENV_RAMPDOWN_STEP, R_EnvelopeReleaseOffset << SPU_CH_ENV_RAMPDOWN_STEP_POS);
}

void MIDI_SetChannelEnable(uint32_t Channel_Number)
{
  if (Channel_Number < 16)
  {
    WRITE_REG(SPU->STOP_STATUS_CH0_15, 0x0001 << Channel_Number);	          // clear stop flag
    SET_BIT(SPU->SPU_EN_CH0_15, 0x0001 << Channel_Number);	                // channel enable
    while ( !(READ_REG(SPU->CH_STATUS_CH0_15) & (0x0001 << Channel_Number)));
  }
  else if (Channel_Number < 32)
  {
    WRITE_REG(SPU->STOP_STATUS_CH16_31, 0x0001 << (Channel_Number - 16));	   // clear stop flag
    SET_BIT(SPU->SPU_EN_CH16_31, 0x0001 << (Channel_Number - 16));	         // channel enable
    while ( !(READ_REG(SPU->CH_STATUS_CH16_31) & (0x0001 << Channel_Number - 16)));
  }
  else if (Channel_Number < 48)
  {
    WRITE_REG(SPU->STOP_STATUS_CH32_47, 0x0001 << (Channel_Number - 32));	   // clear stop flag
    SET_BIT(SPU->SPU_EN_CH32_47, 0x0001 << (Channel_Number - 32));	         // channel enable
    while ( !(READ_REG(SPU->CH_STATUS_CH32_47) & (0x0001 << Channel_Number - 32)));
  }
  else
  {
    WRITE_REG(SPU->STOP_STATUS_CH48_63, 0x0001 << (Channel_Number - 48));	   // clear stop flag
    SET_BIT(SPU->SPU_EN_CH48_63, 0x0001 << (Channel_Number - 48));	         // channel enable
    while ( !(READ_REG(SPU->CH_STATUS_CH48_63) & (0x0001 << Channel_Number - 48)));
  }
}

void MIDI_SetChannelDisable(uint32_t Channel_Number)
{
  if (Channel_Number < 16)
  {
    CLEAR_BIT(SPU->SPU_EN_CH0_15, 0x0001 << Channel_Number);	                // channel enable
  }
  else if (Channel_Number < 32)
  {
    CLEAR_BIT(SPU->SPU_EN_CH16_31, 0x0001 << (Channel_Number - 16));	         // channel enable
  }
  else if (Channel_Number < 48)
  {
    CLEAR_BIT(SPU->SPU_EN_CH32_47, 0x0001 << (Channel_Number - 32));	         // channel enable
  }
  else
  {
    CLEAR_BIT(SPU->SPU_EN_CH48_63, 0x0001 << (Channel_Number - 48));	         // channel enable
  }
}

void MIDI_SetPan(uint32_t Channel_Number, uint32_t Pan)
{
  SPU_CH_TYPE_DEF *SPU_CH;
  ChannelToSPUCH(Channel_Number, SPU_CH);
//  Pan &= SPU_CH_PAN_MSK;
  MODIFY_REG(SPU_CH->PAN_VELOCITY, SPU_CH_VOLUME_PAN_MSK, Pan << SPU_CH_VOLUME_PAN_POS);
}


void MIDI_SetVelocity(uint32_t Channel_Number, uint32_t Velocity)
{
  SPU_CH_TYPE_DEF *SPU_CH;
  ChannelToSPUCH(Channel_Number, SPU_CH);
  Velocity &= SPU_CH_VELOCITY_MSK;
  MODIFY_REG(SPU_CH->PAN_VELOCITY, SPU_CH_VELOCITY_MSK, Velocity);
}

void MIDI_SetWaveType(uint32_t Channel_Number, uint32_t WaveType)
{
  SPU_CH_TYPE_DEF *SPU_CH;
  ChannelToSPUCH(Channel_Number, SPU_CH);

  if ((WaveType & BIT6) == 0)                           // PCM
  {
    WRITE_REG(SPU_CH->WAVE_ADPCM_CTRL, 0x0000);
    if ((WaveType & BIT0) == 0)
    {
      MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, SPU_CH_WAVE_ALGORITHM_PCM + SPU_CH_WAVE_PCM_16BIT + SPU_CH_WAVE_PLAYMODE_LOOP);    //16bit PCM + 16bit PCM Loop
    }
    else
    {
      MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, SPU_CH_WAVE_ALGORITHM_PCM + SPU_CH_WAVE_PCM_16BIT + SPU_CH_WAVE_PLAYMODE_AUTOEND);    //16bit PCM + No Loop
    }
  }
  else                                                  // ADPCM
  {
    WRITE_REG(SPU_CH->WAVE_ADPCM_CTRL, 0xBE00);
    if ((WaveType & BIT0) == 0)
    {
      MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, SPU_CH_WAVE_ALGORITHM_ADPCM + SPU_CH_WAVE_PCM_16BIT + SPU_CH_WAVE_PLAYMODE_ADPCM_PCM_LOOP);    //ADPCM + 16bit PCM Loop
    }
    else
    {
      MODIFY_REG(SPU_CH->WAVE_MODE, 0xF000, SPU_CH_WAVE_ALGORITHM_ADPCM + SPU_CH_WAVE_PLAYMODE_AUTOEND);    //ADPCM + No Loop
    }
  }
}

void MIDI_SetEnvelopeManualMode(uint32_t Channel_Number)
{
  if (Channel_Number < 16)
  {
    SET_BIT(SPU->ENV_MODE_CH0_15, 0x0001 << Channel_Number);
  }
  else if (Channel_Number < 32)
  {
    SET_BIT(SPU->ENV_MODE_CH16_31, 0x0001 << (Channel_Number - 16));
  }
  else if (Channel_Number < 48)
  {
    SET_BIT(SPU->ENV_MODE_CH32_47, 0x0001 << (Channel_Number - 32));
  }
  else
  {
    SET_BIT(SPU->ENV_MODE_CH48_63, 0x0001 << (Channel_Number - 48));
  }
}

void MIDI_SetEnvelopeAutoMode(uint32_t Channel_Number)
{
  if (Channel_Number < 16)
  {
    CLEAR_BIT(SPU->ENV_MODE_CH0_15, 0x0001 << Channel_Number);
  }
  else if (Channel_Number < 32)
  {
    CLEAR_BIT(SPU->ENV_MODE_CH16_31, 0x0001 << (Channel_Number - 16));
  }
  else if (Channel_Number < 48)
  {
    CLEAR_BIT(SPU->ENV_MODE_CH32_47, 0x0001 << (Channel_Number - 32));
  }
  else
  {
    CLEAR_BIT(SPU->ENV_MODE_CH48_63, 0x0001 << (Channel_Number - 48));
  }
}

void MIDI_SetEnvelopeValue(uint32_t Channel_Number, uint32_t R_EDD)
{
  SPU_CH_TYPE_DEF *SPU_CH;
  ChannelToSPUCH(Channel_Number, SPU_CH);
  MODIFY_REG(SPU_CH->ENV_DATA, SPU_CH_ENV_DATA_MSK, (R_EDD & 0x7F));
}

void MIDI_EnvRelease_Enable(uint32_t Channel_Number)
{
  if (Channel_Number < 16)
  {
    WRITE_REG(SPU->ENV_RAMPDOWN_CH0_15, 0x0001 << Channel_Number);
  }
  else if (Channel_Number < 32)
  {
    WRITE_REG(SPU->ENV_RAMPDOWN_CH16_31, 0x0001 << (Channel_Number - 16));
  }
  else if (Channel_Number < 48)
  {
    WRITE_REG(SPU->ENV_RAMPDOWN_CH32_47, 0x0001 << (Channel_Number - 32));
  }
  else
  {
    WRITE_REG(SPU->ENV_RAMPDOWN_CH48_63, 0x0001 << (Channel_Number - 48));
  }
}









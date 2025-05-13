/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   SACM_A1801_User.c
 * @Version:
 *   V1.0.3.1
 * @Date:
 *   April 15, 2022
 * @Abstract:
 *
 **************************************************************************************************/


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"

/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/
#define C_BufferLength       512
#define C_BufferLength_X2   C_BufferLength*2        // Right and Left
#define C_Buffer_A          BIT1
#define C_Buffer_B          BIT2
#define C_DataReady         BIT7

// I2S 120000000/64/32 = 58,593.75Hz
uint16_t I2S_DataInBuffer[C_BufferLength_X2*2];
uint16_t DAC_DataBuffer_L[C_BufferLength*2];
uint16_t DAC_DataBuffer_R[C_BufferLength*2];
uint8_t R_I2S_DataReady;

/*---------------------------------------------------------------------------------------
 * User Subroutine Area
 *---------------------------------------------------------------------------------------*/
 /**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void I2S_RX_ISR(void)
{
    GPIOA_OBIT->OBIT17 ^= BIT17;
    if(R_I2S_DataReady == C_Buffer_B)
    {
        R_I2S_DataReady = C_Buffer_A + C_DataReady;       // Data Ready in Buffer_A
    }
    else
    {
        R_I2S_DataReady = C_Buffer_B + C_DataReady;       // Data Ready in Buffer_B
    }
}
/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void I2S_RX_Service(void)
{
    uint16_t i;
    uint16_t j;
	uint16_t *StereoPCM_SPU;
	uint16_t *pDAC_L;
	uint16_t *pDAC_R;

    if (R_I2S_DataReady & C_DataReady)
    {
        R_I2S_DataReady &= ~C_DataReady;
        if (R_I2S_DataReady == C_Buffer_A)
        {
            pDAC_L = DAC_DataBuffer_L;
            pDAC_R = DAC_DataBuffer_R;
            StereoPCM_SPU = I2S_DataInBuffer;
        }
        else
        {
            pDAC_L = DAC_DataBuffer_L + C_BufferLength;
            pDAC_R = DAC_DataBuffer_R + C_BufferLength;
            StereoPCM_SPU = I2S_DataInBuffer + C_BufferLength_X2;
        }
    }

    for (i = 0, j = 0; i < C_BufferLength; i++)
    {
      pDAC_L[i] = StereoPCM_SPU[j++];
      pDAC_R[i] = StereoPCM_SPU[j++];
    /*
      pDAC_L[i] = StereoPCM_SPU[j++] ^ 0x8000;
      pDAC_R[i] = StereoPCM_SPU[j++] ^ 0x8000;
     */
    }
// -- Process,  pDAC_L[i] and pDAC_R[i]--

// --------------------------------------
}

/**
 * @brief
 *
 * @param
 *   None.
 * @return
 *   None.
 */
void I2S_RX_Initial(void)
{
    uint16_t i;
    for (i = 0; i < C_BufferLength*2;i++)
    {
        DAC_DataBuffer_L[i] = 0x0000;
        DAC_DataBuffer_R[i] = 0x0000;
    }

	TIMER_Open(TM0);
	WRITE_REG(TM0->PLOAD, 2047);
	WRITE_REG(TM0->COUNT, 0);

    SET_BIT(DAC->CTRL, DAC_CTRL_DAC_CH0_DMA_ENABLE + DAC_CTRL_CH0_DMA_CONTINUE_ENABLE);
    DMA1->CTRL &= ~DMA_CTRL_DMA_ENABLE;
    DMA1->SRCADR = DAC_DataBuffer_L;
    DMA1->DSTADR = (uint32_t)&DAC->DAC_CH0_DMA_DATA0;
    DMA1->DTN = C_BufferLength;
    DMA_INT->INTSTS = DMA_INTSTS_DMA1_DONE_INT_FLAG;
    DMA1->CTRL = DMA_CTRL_DMA_REQSEL_DAC_CH0 |DMA_CTRL_DMA_SRCSIZE_16B  | DMA_CTRL_DMA_DSTSIZE_16B  | DMA_CTRL_DMA_SRCINC_ENABLE | DMA_CTRL_DMA_DUAL_ENABLE | DMA_CTRL_DMA_CIRC_ENABLE | DMA_CTRL_DMA_CONTINUE_ENABLE;

    SET_BIT(DAC->CTRL, DAC_CTRL_DAC_CH1_DMA_ENABLE + DAC_CTRL_CH1_DMA_CONTINUE_ENABLE);
    DMA2->CTRL &= ~DMA_CTRL_DMA_ENABLE;
    DMA2->SRCADR = DAC_DataBuffer_R;
    DMA2->DSTADR = (uint32_t)&DAC->DAC_CH1_DMA_DATA0;
    DMA2->DTN = C_BufferLength;
    DMA_INT->INTSTS = DMA_INTSTS_DMA2_DONE_INT_FLAG;
    DMA2->CTRL = DMA_CTRL_DMA_REQSEL_DAC_CH1 |DMA_CTRL_DMA_SRCSIZE_16B  | DMA_CTRL_DMA_DSTSIZE_16B  | DMA_CTRL_DMA_SRCINC_ENABLE | DMA_CTRL_DMA_DUAL_ENABLE | DMA_CTRL_DMA_CIRC_ENABLE | DMA_CTRL_DMA_CONTINUE_ENABLE;

    R_I2S_DataReady = C_Buffer_B;
    DMA_InstallIsrService(DMA0, I2S_RX_ISR);
    DMA0->CTRL &= ~DMA_CTRL_DMA_ENABLE;
    DMA0->CTRL = DMA_CTRL_DMA_DUAL_ENABLE | DMA_CTRL_DMA_REQSEL_I2S | DMA_CTRL_DMA_SRCSIZE_16B | DMA_CTRL_DMA_DSTSIZE_16B | DMA_CTRL_DMA_CONTINUE_ENABLE
                | DMA_CTRL_DMA_SRCINC_DISABLE | DMA_CTRL_DMA_DSTINC_ENABLE | DMA_CTRL_DMA_CIRC_ENABLE;
    DMA0->SRCADR = (uint32_t)&I2S->RX_DATA;
	DMA0->DSTADR = I2S_DataInBuffer;
	DMA0->DTN = C_BufferLength_X2;
//	DMA0->DTN = C_BufferLength;
    DMA0->CTRL |= DMA_CTRL_DMA_DONE_INT_ENABLE;
    NVIC_EnableIRQ(DMA0_IRQn);

    I2S_RX_Init();
    I2S_RX_Enable();

    DMA0->CTRL |= DMA_CTRL_DMA_ENABLE;
    DMA1->CTRL |= DMA_CTRL_DMA_ENABLE;
    DMA2->CTRL |= DMA_CTRL_DMA_ENABLE;
}


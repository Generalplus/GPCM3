/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   GPCM3_FM1.h
 * @Version:
 *   V0.9.3
 * @Date:
 *   April 30, 2025
 * @Abstract:
 *   Registers definition for Generalplus GPCM3x Series
 *   2023.03.08 Correct the wrong bit definition of the DAC_CTRL_CH0_POSTWAVE_OUT_SRC_POS
 *   2023.03.28 Add the definition of AUDPWM_CTRL[15]
 *   2023.05.09 Modify the IOFUNC->SPIFC to IOFUNC->SPIFCCFG
 *   2023.05.17 Add the definition of SPIFC_TIMING[12] (tSLCH time)
 *   2023.10.13 Correct the pad definition of ACU_APAD[7:0]
 *   2024.02.05 Update the definitions of I2S
 *   2024.07.19 Update the definitions of CLOCK_SWTRIM_COUNTER_MSK
 *   2024.12.02 Modify the I2S->TX_CTRL[19] Sign/Unsign & [15:14] Stereo LR/RL definition.
 *   2024.12.03 Add the definition of System Tick.
 *   2025.04.30 Add the clock source of CCP1.
 *
 **************************************************************************************************/
#ifndef _GPCM3_FM1_H_
#define _GPCM3_FM1_H_

/**
 * <Device>
 * =================================================================================
 * |Name             |GPCM3/GPFM1
 * |Version          |
 * |Description      |
 * |AddressUnitBits  |8
 * |Width            |32
 * |ResetValue       |0
 * |ResetMask        |0
 * =================================================================================
 * </Device>
 */

/* GPIO Configration
 * =================================================================================
 * CFG[1:0] OBUF    Function Description     Wakeup
 *  0    0    0     Input Pull Low          Yes
 *  0    0    1     Input Pull High         Yes
 *  0    1    x     Input Floating          Yes
 *  1    0    0     Output Open Drain       No
 *  1    0    1     Output Sink mode        No
 *  1    1    0     Output Low              No
 *  1    1    1     Output High             No
 * =================================================================================
 * Default: Input with pull low.
 */

/* Special IO Function
 * =================================================================================
 * IOA[15:0]  IOA00  IOA01  IOA02  IOA03  IOA04  IOA05  IOA06  IOA07  IOA08  IOA09  IOA10  IOA11  IOA12  IOA13  IOA14  IOA15
 * VKey       VKOut0 VKOut1 VKOut2 VKOut3 VKIn0  VKIn1  VKIn2  VKIn3  VKIn4  VKIn5  VKIn6  VKIn7  VKIn8  VKIn9  VKIn10 VKIn11
 * CCP0         -      -      -    CCP_A0 CCP_A1 CCP_A2 CCP_A3   -      -      -      -      -      -    CCP_B0 CCP_B1 CCP_B2
 * CCP1         -      -      -       -      -      -      -      -      -   CCP_A0 CCP_A1 CCP_A2 CCP_A3   -      -      -
 * CTS         YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES
 * I2C        SCK_0  SDA_0    -      -      -      -      -      -      -      -      -      -      -      -      -    SCK_1
 * IOPWM        -      -      -      -      -      -      -      -      -      -      -      -      -    PWM0   PWM1   PWM2
 * I2S_In       -      -      -    MCLK   BCLK    LR    DATA     -      -      -      -      -      -      -      -      -
 * I2S_Out      -      -      -      -      -      -      -      -      -    MCLK   BCLK    LR    DATA     -      -      -
 * IR_Tx        -      -      -      -      -      -      -    Tx_0     -      -      -      -      -      -      -      -
 * UART0      Tx_0   Rx_0     -      -      -      -      -      -      -      -      -      -      -    Tx_1   Rx_1     -
 * UART1        -      -      -      -      -     Tx_0   Rx_0    -      -      -      -      -      -      -      -      -
 * SPI0         -      -      -    SCLK_0 MOSI_0  CS_0  MISO_0   -      -      -      -      -      -      -      -      -
 * SPI1         -      -      -      -      -      -      -      -      -    SCLK_1 MOSI_1 CS_1   MISO_1   -      -      -
 * WakeUp      YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES
 * WakeUp(SD)  YES    YES    YES    YES    YES    YES    YES    YES     -      -      -      -      -      -      -      -
 * Feedback     -      -      -    IN_0   OUT_0    -      -    OUT_1   IN_1    -      -      -      -      -      -      -
 * EXT.INT.    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES
 * PDM_IN       -      -      -      -      -    DATA    CLK     -      -      -      -      -      -      -      -      -
 * TMCAP       YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES
 * X32K         -      -      -      -      -      -      -    X32KO  X32KI    -      -      -      -      -      -      -

 * ----------------------------------------------------------------------------
 * IOA[24:16] IOA16  IOA17  IOA18  IOA19  IOA20  IOA21  IOA22  IOA23  IOA24  IOA25  IOA26  IOA27  IOA28  IOA29  IOA30  IOA31
 * VKey       VKIn12 VKIn13 VKIn14 VKIn15 VKIn16 VKIn17 VKIn18 VKIn19 VKIn20 VKIn21   -      -      -      -      -      -
 * CCP0       CCP_B3   -      -      -      -      -      -      -      -      -      -      -      -      -      -      -
 * CCP1         -      -      -      -      -      -      -      -      -      -    CCP_B0 CCP_B1 CCP_B2 CCP_B3   -      -
 * CTS         YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES
 * I2C        SDA_1    -      -      -    SCK_2  SDA_2    -      -      -      -      -      -      -      -      -      -
 * IOPWM      PWM3   PWM4   PWM5   PWM6   PWM7   PWM8   PWM9   PWM10  PWM11  PWM12  PWM13  PWM14  PWM15    -      -      -
 * I2S_In       -      -      -      -      -      -      -      -      -      -    MCLK   BCLK    LR    DATA     -      -
 * I2S_Out      -      -      -      -      -      -    MCLK   BCLK    LR    DATA     -      -      -      -      -      -
 * IR_Tx        -    Tx_1     -      -      -      -      -      -      -      -      -      -      -      -      -      -
 * UART0        -      -      -      -      -      -      -      -      -      -      -      -      -      -    Tx_2   Rx_2
 * UART1        -      -      -      -    Tx_1   Rx_1     -      -      -      -    Tx_2   Rx_2     -      -      -      -
 * SPI0         -      -      -      -      -      -      -      -      -      -    SCLK_0 MOSI_0 CS_0   MISO_0   -      -
 * SPI1         -      -      -      -      -      -    SCLK_1 MOSI_1 CS_1   MISO_1   -      -      -      -      -      -
 * WakeUp      YES    YES    YES    YES    YES    YES    YES    YES    YES     -      -      -      -      -      -      -
 * WakeUp(SD)   -      -      -      -      -      -      -      -     YES    YES    YES    YES    YES    YES    YES    YES
 * EXT.INT.    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES     -
 * PDM_IN       -      -      -      -      -      -    DATA    CLK     -      -      -      -      -      -      -      -
 * TMCAP       YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES    YES
 * LineIN       -      -      -      -      -      -      -      -     YES    YES    YES    YES    YES    YES    YES    YES
 * QD           -      -    QD0_A  QD0_B  QD1_A  QD1_B    -      -      -      -      -      -      -      -      -      -

 * ----------------------------------------------------------------------------
 * IOB[5:0]   IOB00  IOB01  IOB02  IOB03  IOB04  IOB05
 * SPIFC      SIO3   SCLK   SIO0    CS    SIO1   SIO2
 * SPI0         -    SCLK_0 MOSI_0  CS_0  MISO_0   -
 * WakeUp      YES    YES    YES    YES    YES    YES
 * EXT.INT.    YES    YES    YES    YES    YES    YES
 * TMCAP       YES    YES    YES    YES    YES    YES

 * ----------------------------------------------------------------------------
 * IOC[1:0]   IOC00  IOC01  IOC02
 * ICE I/F      -      -     SDA
 * X12M       X12MI  X12MO    -
 * UART0        -      -    Tx_3

 * ----------------------------------------------------------------------------
 * IOD[1:0]   IOD00  IOD01
 * USB         DP     DM

 * =================================================================================
 */


/*---------------------------------------------------------------------------------------
 * Interrupt Number Definition
 *---------------------------------------------------------------------------------------*/
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ******************************************/
  NonMaskableInt_IRQn           = -14,           /*!<  2 Non Maskable Interrupt          */
  HardFault_IRQn                = -13,           /*!<  3 HardFault Interrupt             */
  SVCall_IRQn                   =  -5,           /*!< 11 SV Call Interrupt               */
  PendSV_IRQn                   =  -2,           /*!< 14 Pend SV Interrupt               */
  SysTick_IRQn                  =  -1,           /*!< 15 System Tick Interrupt           */

/******  GPCM2_CM3 Specific Interrupt Numbers ********************************************/
  USB_IRQn                      = 0,             /*!< USB Interrupt                      */
  VKEY_IRQn                     = 1,             /*!< Velocity Key Interrupt             */
  EXTI_IRQn                     = 2,             /*!< EXT0~3 Interrupt                   */
  MAC_IRQn                      = 3,             /*!< MAC Interrupt                      */
  QD_IRQn                       = 4,             /*!< Quadrature Decoder Interrupt       */
  SAR_ADC_IRQn                  = 5,             /*!< SAR-ADC Interrupt                  */
  DS_ADC_IRQn                   = 6,             /*!< Delta-sigma ADC Interrupt          */
  DAC_CH0_IRQn                  = 7,             /*!< DAC Channel0 Interrupt             */
  DAC_CH1_IRQn                  = 8,             /*!< DAC Channel1 Interrupt             */
  CCP0_IRQn                     = 9,             /*!< CCP0 Interrupt                     */
  CCP1_IRQn                     = 10,            /*!< CCP1 Interrupt                     */
  CTS_TM0_IRQn                  = 11,            /*!< Touch Timer0 Interrupt             */
  CTS_TM1_IRQn                  = 12,            /*!< Touch Timer1 Interrupt             */
  TIMEBASE_IRQn                 = 13,            /*!< Timebase Interrupt                 */
  I2C_IRQn                      = 14,            /*!< I2C Interrupt                      */
  SPU_IRQn                      = 15,            /*!< SPU ENV, Pwspu, SPUfiq Interrupt   */
  UART0_IRQn                    = 16,            /*!< UART0 Interrupt                    */
  UART1_IRQn                    = 17,            /*!< UART1 Interrupt                    */
  I2S_IRQn                      = 18,            /*!< I2S Interrupt                      */
  SPI0_IRQn                     = 19,            /*!< SPI0 Interrupt                     */
  SPI1_IRQn                     = 20,            /*!< SPI1 Interrupt                     */
  DMA0_IRQn                     = 21,            /*!< DMA0 Interrupt                     */
  DMA1_IRQn                     = 22,            /*!< DMA1 Interrupt                     */
  DMA2_IRQn                     = 23,            /*!< DMA2 Interrupt                     */
  DMA3_IRQn                     = 24,            /*!< DMA3 Interrupt                     */
  DMA4_IRQn                     = 25,            /*!< DMA4 Interrupt                     */
  TIMER0_IRQn                   = 26,            /*!< Timer0 Interrupt                   */
  TIMER1_IRQn                   = 27,            /*!< Timer1 Interrupt                   */
  TIMER2_IRQn                   = 28,            /*!< Timer2 Interrupt                   */
  KEYCHG_IRQn                   = 29,            /*!< Key Change Inerrupt                */
  PDM_IRQn                      = 30,            /*!< PDM Interrupt                      */
  SPUBEAT_IRQn                  = 31             /*!< SPU Beat Interrupt                 */
} IRQn_Type;


/*---------------------------------------------------------------------------------------
 * Processor and Core Peripheral Section
 *---------------------------------------------------------------------------------------*/
#define __CM0_REV               0                /*!< Core Revision r0p0                                */
#define __MPU_PRESENT           0                /*!< do not provide MPU                                */
#define __NVIC_PRIO_BITS        2                /*!< 2 Bits for the Priority Levels                    */
#define __Vendor_SysTickConfig  0                /*!< Set to 1 if different SysTick Config is used      */


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <core_cm0.h>                            // Cortex-M0 processor and core peripherals
#include <stdint.h>
#include <string.h>


/*---------------------------------------------------------------------------------------
 * SPI Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;                          // SPI Control Register
    __IO uint32_t STS;                           // SPI Status Register
    __IO uint32_t TX_DATA;                       // SPI Transmit Data Register
    __IO uint32_t RX_DATA;                       // SPI Receive Data Register
    __IO uint32_t CLK_DIV;
} SPI_TYPE_DEF;

/*
 * Bit definition for SPI_CTRL[31] - Manual mode auto read/write enable
 */
#define SPI_CTRL_MASTER_RX_TRIG_POS              (31)
#define SPI_CTRL_MASTER_RX_TRIG_MSK              (0x1UL << SPI_CTRL_MASTER_RX_TRIG_POS)
#define SPI_CTRL_MASTER_RX_TRIG_ENABLE           (0x1UL << SPI_CTRL_MASTER_RX_TRIG_POS)
#define SPI_CTRL_MASTER_RX_TRIG_DISABLE          (0x0UL << SPI_CTRL_MASTER_RX_TRIG_POS)

/*
 * Bit definition for SPI_CTRL[30] - Tx function disable
 */
#define SPI_CTRL_TX_DISABLE_POS                  (30)
#define SPI_CTRL_TX_DISABLE_MSK                  (0x1UL << SPI_CTRL_TX_DISABLE_POS)
#define SPI_CTRL_TX_DISABLE                      (0x1UL << SPI_CTRL_TX_DISABLE_POS)
#define SPI_CTRL_TX_ENABLE                       (0x0UL << SPI_CTRL_TX_DISABLE_POS)

/*
 * Bit definition for SPI_CTRL[29:28] - SPI RX to sel latch data timing for high speed SPI (>20Mhz)
 */
#define SPI_CTRL_LATCH_SEL_DLY_POS               (28)
#define SPI_CTRL_LATCH_SEL_DLY_MSK               (0x3ul << SPI_CTRL_LATCH_SEL_DLY_POS)
#define SPI_CTRL_LATCH_SEL_DLY_3                 (0x3ul << SPI_CTRL_LATCH_SEL_DLY_POS)
#define SPI_CTRL_LATCH_SEL_DLY_2                 (0x2ul << SPI_CTRL_LATCH_SEL_DLY_POS)
#define SPI_CTRL_LATCH_SEL_DLY_1                 (0x1ul << SPI_CTRL_LATCH_SEL_DLY_POS)
#define SPI_CTRL_LATCH_SEL_DLY_0                 (0x0ul << SPI_CTRL_LATCH_SEL_DLY_POS)

/*
 * Bit definition for SPI_CTRL[19] - Tx source selection during DMA Received data
 */
#define SPI_CTRL_MOSI_DASEL_POS                  (19)
#define SPI_CTRL_MOSI_DASEL_MSK                  (0x1UL << SPI_CTRL_MOSI_DASEL_POS)
#define SPI_CTRL_MOSI_DASEL_SRC_TX               (0x1UL << SPI_CTRL_MOSI_DASEL_POS)
#define SPI_CTRL_MOSI_DASEL_SRC_RX               (0x0UL << SPI_CTRL_MOSI_DASEL_POS)

/*
 * Bit definition for SPI_CTRL[18] - Manual mode auto read/write enable
 */
#define SPI_CTRL_MAUT_RW_EN_POS                  (18)
#define SPI_CTRL_MAUT_RW_EN_MSK                  (0x1UL << SPI_CTRL_MAUT_RW_EN_POS)
#define SPI_CTRL_MAUT_RW_EN_ENABLE               (0x1UL << SPI_CTRL_MAUT_RW_EN_POS)
#define SPI_CTRL_MAUT_RW_EN_DISABLE              (0x0UL << SPI_CTRL_MAUT_RW_EN_POS)

/*
 * Bit definition for SPI_CTRL[17] - Rx DMA enable
 */
#define SPI_CTRL_RX_DMA_EN_POS                   (17)
#define SPI_CTRL_RX_DMA_EN_MSK                   (0x1UL << SPI_CTRL_RX_DMA_EN_POS)
#define SPI_CTRL_RX_DMA_ENABLE                   (0x1UL << SPI_CTRL_RX_DMA_EN_POS)
#define SPI_CTRL_RX_DMA_DISABLE                  (0x0UL << SPI_CTRL_RX_DMA_EN_POS)

/*
 * Bit definition for SPI_CTRL[16] - Tx DMA enable
 */
#define SPI_CTRL_TX_DMA_EN_POS                   (16)
#define SPI_CTRL_TX_DMA_EN_MSK                   (0x1UL << SPI_CTRL_TX_DMA_EN_POS)
#define SPI_CTRL_TX_DMA_ENABLE                   (0x1UL << SPI_CTRL_TX_DMA_EN_POS)
#define SPI_CTRL_TX_DMA_DISABLE                  (0x0UL << SPI_CTRL_TX_DMA_EN_POS)

/*
 * Bit definition for SPI_CTRL[15] - SPI CSB control by SW(Bit.11 CSB_GPIO) enable
 */
#define SPI_CTRL_CSB_SW_EN_POS                   (15)
#define SPI_CTRL_CSB_SW_EN_MSK                   (0x1UL << SPI_CTRL_CSB_SW_EN_POS)
#define SPI_CTRL_CSB_SW_ENABLE                   (0x1UL << SPI_CTRL_CSB_SW_EN_POS)
#define SPI_CTRL_CSB_SW_DISABLE                  (0x0UL << SPI_CTRL_CSB_SW_EN_POS)

/*
 * Bit definition for SPI_CTRL[14:12] -  SPI_CLK_real_i_sel : clk delay sel
 */
#define SPI_CTRL_CLK_IN_SEL_DLY_POS              (12)
#define SPI_CTRL_CLK_IN_SEL_DLY_MSK              (0x7ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)
#define SPI_CTRL_CLK_IN_SEL_DLY_7                (0x7ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)
#define SPI_CTRL_CLK_IN_SEL_DLY_6                (0x6ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)
#define SPI_CTRL_CLK_IN_SEL_DLY_5                (0x5ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)
#define SPI_CTRL_CLK_IN_SEL_DLY_4                (0x4ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)
#define SPI_CTRL_CLK_IN_SEL_DLY_3                (0x3ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)
#define SPI_CTRL_CLK_IN_SEL_DLY_2                (0x2ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)
#define SPI_CTRL_CLK_IN_SEL_DLY_1                (0x1ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)
#define SPI_CTRL_CLK_IN_SEL_DLY_0                (0x0ul << SPI_CTRL_CLK_IN_SEL_DLY_POS)

/*
 * Bit definition for SPI_CTRL[11] - SPI CSB control bit
 */
#define SPI_CTRL_CSB_GPIO_POS                    (11)
#define SPI_CTRL_CSB_GPIO_MSK                    (0x1UL << SPI_CTRL_CSB_GPIO_POS)
#define SPI_CTRL_CSB_GPIO_HIGH                   (0x1UL << SPI_CTRL_CSB_GPIO_POS)
#define SPI_CTRL_CSB_GPIO_LOW                    (0x0UL << SPI_CTRL_CSB_GPIO_POS)

/*
 * Bit definition for SPI_CTRL[10] - Error interrupt enable
 */
#define SPI_CTRL_ERR_INT_EN_POS                  (10)
#define SPI_CTRL_ERR_INT_EN_MSK                  (0x1UL << SPI_CTRL_ERR_INT_EN_POS)
#define SPI_CTRL_ERR_INT_ENABLE                  (0x1UL << SPI_CTRL_ERR_INT_EN_POS)
#define SPI_CTRL_ERR_INT_DISABLE                 (0x0UL << SPI_CTRL_ERR_INT_EN_POS)

/*
 * Bit definition for SPI_CTRL[9] - SPI Received complete interrupt enable
 */
#define SPI_CTRL_RX_INT_EN_POS                   (9)
#define SPI_CTRL_RX_INT_EN_MSK                   (0x1UL << SPI_CTRL_RX_INT_EN_POS)
#define SPI_CTRL_RX_INT_ENABLE                   (0x1UL << SPI_CTRL_RX_INT_EN_POS)
#define SPI_CTRL_RX_INT_DISABLE                  (0x0UL << SPI_CTRL_RX_INT_EN_POS)

/*
 * Bit definition for SPI_CTRL[8] - SPI transmitted complete interrupt enable
 */
#define SPI_CTRL_TX_INT_EN_POS                   (8)
#define SPI_CTRL_TX_INT_EN_MSK                   (0x1UL << SPI_CTRL_TX_INT_EN_POS)
#define SPI_CTRL_TX_INT_ENABLE                   (0x1UL << SPI_CTRL_TX_INT_EN_POS)
#define SPI_CTRL_TX_INT_DISABLE                  (0x0UL << SPI_CTRL_TX_INT_EN_POS)

/*
 * Bit definition for SPI_CTRL[6] - SPI CLK Continue mode
 */
#define SPI_CTRL_SPICLK_CONTIUNE_EN_POS          (6)
#define SPI_CTRL_SPICLK_CONTIUNE_EN_MSK          (0x1UL << SPI_CTRL_SPICLK_CONTIUNE_EN_POS)
#define SPI_CTRL_SPICLK_CONTIUNE_ENABLE          (0x1UL << SPI_CTRL_SPICLK_CONTIUNE_EN_POS)
#define SPI_CTRL_SPICLK_CONTIUNE_DISABLE         (0x0UL << SPI_CTRL_SPICLK_CONTIUNE_EN_POS)

/*
 * Bit definition for SPI_CTRL[5] - SPI loop-back enable
 */
#define SPI_CTRL_LOOPBACK_EN_POS                 (5)
#define SPI_CTRL_LOOPBACK_EN_MSK                 (0x1UL << SPI_CTRL_LOOPBACK_EN_POS)
#define SPI_CTRL_LOOPBACK_ENABLE                 (0x1UL << SPI_CTRL_LOOPBACK_EN_POS)
#define SPI_CTRL_LOOPBACK_DISABLE                (0x0UL << SPI_CTRL_LOOPBACK_EN_POS)

/*
 * Bit definition for SPI_CTRL[4] - SPI clock polarity
 */
#define SPI_CTRL_CLK_POL_POS                     (4)
#define SPI_CTRL_CLK_POL_MSK                     (0x1UL << SPI_CTRL_CLK_POL_POS)
#define SPI_CTRL_CLK_POL1                        (0x1UL << SPI_CTRL_CLK_POL_POS)
#define SPI_CTRL_CLK_POL0                        (0x0UL << SPI_CTRL_CLK_POL_POS)

/*
 * Bit definition for SPI_CTRL[3] - SPI clock phase
 */
#define SPI_CTRL_CLK_PHA_POS                     (3)
#define SPI_CTRL_CLK_PHA_MSK                     (0x1UL << SPI_CTRL_CLK_PHA_POS)
#define SPI_CTRL_CLK_PHA_SHIFT                   (0x1UL << SPI_CTRL_CLK_PHA_POS)
#define SPI_CTRL_CLK_PHA_NORMAL                  (0x0UL << SPI_CTRL_CLK_PHA_POS)

/*
 * Bit definition for SPI_CTRL[2] - SPI CSB keeping low control
 */
#define SPI_CTRL_CSB_KEEPL_POS                   (2)
#define SPI_CTRL_CSB_KEEPL_MSK                   (0x1UL << SPI_CTRL_CSB_KEEPL_POS)
#define SPI_CTRL_CSB_KEEPL_ENABLE                (0x1UL << SPI_CTRL_CSB_KEEPL_POS)
#define SPI_CTRL_CSB_KEEPL_DISABLE               (0x0UL << SPI_CTRL_CSB_KEEPL_POS)

/*
 * Bit definition for SPI_CTRL[1] - SPI operating mode select
 */
#define SPI_CTRL_MODE_SEL_POS                    (1)
#define SPI_CTRL_MODE_SEL_MSK                    (0x1UL << SPI_CTRL_MODE_SEL_POS)
#define SPI_CTRL_SLAVE_MODE                      (0x1UL << SPI_CTRL_MODE_SEL_POS)
#define SPI_CTRL_MASTER_MODE                     (0x0UL << SPI_CTRL_MODE_SEL_POS)

/*
 * Bit definition for SPI_CTRL[0] - SPI enable
 */
#define SPI_CTRL_SPI_EN_POS                      (0)
#define SPI_CTRL_SPI_EN_MSK                      (0x1UL << SPI_CTRL_SPI_EN_POS)
#define SPI_CTRL_SPI_ENABLE                      (0x1UL << SPI_CTRL_SPI_EN_POS)
#define SPI_CTRL_SPI_DISABLE                     (0x0UL << SPI_CTRL_SPI_EN_POS)

/*
 * Bit definition for SPI_STS[4] - Receive buffer not empty
 */
#define SPI_CTRL_RX_BUF_NEMPTY_FLAG_POS          (4)
#define SPI_CTRL_RX_BUF_NEMPTY_FLAG_MSK          (0x1UL << SPI_CTRL_RX_BUF_NEMPTY_FLAG_POS)
#define SPI_CTRL_RX_BUF_NEMPTY_FLAG              (0x1UL << SPI_CTRL_RX_BUF_NEMPTY_FLAG_POS)

/*
 * Bit definition for SPI_STS[3] - SPI received over run
 */
#define SPI_CTRL_RX_OVER_RUN_FLAG_POS            (3)
#define SPI_CTRL_RX_OVER_RUN_FLAG_MSK            (0x1UL << SPI_CTRL_RX_OVER_RUN_FLAG_POS)
#define SPI_CTRL_RX_OVER_RUN_FLAG                (0x1UL << SPI_CTRL_RX_OVER_RUN_FLAG_POS)

/*
 * Bit definition for SPI_STS[2] - SPI receive complete flag
 */
#define SPI_CTRL_RX_DONE_FLAG_POS                (2)
#define SPI_CTRL_RX_DONE_FLAG_MSK                (0x1UL << SPI_CTRL_RX_DONE_FLAG_POS)
#define SPI_CTRL_RX_DONE_FLAG                    (0x1UL << SPI_CTRL_RX_DONE_FLAG_POS)

/*
 * Bit definition for SPI_STS[0] - SPI transmission complete flag
 */
#define SPI_CTRL_TX_DONE_FLAG_POS                (0)
#define SPI_CTRL_TX_DONE_FLAG_MSK                (0x1UL << SPI_CTRL_TX_DONE_FLAG_POS)
#define SPI_CTRL_TX_DONE_FLAG                    (0x1UL << SPI_CTRL_TX_DONE_FLAG_POS)

/*
 * Bit definition for SPI_CLK_DIV[23:20] - SPI clock selection
 */
#define SPI_CLK_DIV_PCLK_DIV_POS                 (0)
#define SPI_CLK_DIV_PCLK_DIV_MSK                 (0x3FFUL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_1024                (0x3FFUL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_512                 (0x1FFUL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_256                 (0xFFUL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_128                 (0x7FUL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_64                  (0x3FUL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_32                  (0x1FUL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_16                  (0xFUL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_10                  (0x9UL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_8                   (0x7UL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_6                   (0x5UL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_4                   (0x3UL << SPI_CLK_DIV_PCLK_DIV_POS)
#define SPI_CLK_DIV_PCLK_DIV_2                   (0x0UL << SPI_CLK_DIV_PCLK_DIV_POS)


/*---------------------------------------------------------------------------------------
 * UART Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
 typedef struct
{
    __IO uint32_t CTRL;                          // UART Control Register
    __IO uint32_t STS;                           // UART Status Register
    __IO uint32_t BAUD_RATE;                     // UART Baud Rate Register
    __IO uint32_t DATA;                          // UART Data Register
    __IO uint32_t FIFO;                          // UART FIFO Control Register(UART0 only)
} UART_TYPE_DEF;

/*
 * Bit definition for UART_CTRL[21] - Rx buffer DMA enable
 */
#define UART_CTRL_RX_DMAE_POS                    (21)
#define UART_CTRL_RX_DMAE_MSK                    (0x1UL << UART_CTRL_RX_DMAE_POS)
#define UART_CTRL_RX_DMA_ENABLE                  (0x1UL << UART_CTRL_RX_DMAE_POS)
#define UART_CTRL_RX_DMA_DISABLE                 (0x0UL << UART_CTRL_RX_DMAE_POS)

/*
 * Bit definition for UART_CTRL[20] - Tx buffer DMA enable
 */
#define UART_CTRL_TX_DMAE_POS                    (20)
#define UART_CTRL_TX_DMAE_MSK                    (0x1UL << UART_CTRL_TX_DMAE_POS)
#define UART_CTRL_TX_DMA_ENABLE                  (0x1UL << UART_CTRL_TX_DMAE_POS)
#define UART_CTRL_TX_DMA_DISABLE                 (0x0UL << UART_CTRL_TX_DMAE_POS)

/*
 * Bit definition for UART_CTRL[19] - UART loop-back enable
 */
#define UART_CTRL_LOOPBACK_EN_POS                (19)
#define UART_CTRL_LOOPBACK_EN_MSK                (0x1UL << UART_CTRL_LOOPBACK_EN_POS)
#define UART_CTRL_LOOPBACK_ENABLE                (0x1UL << UART_CTRL_LOOPBACK_EN_POS)
#define UART_CTRL_LOOPBACK_DISABLE               (0x0UL << UART_CTRL_LOOPBACK_EN_POS)

/*
 * Bit definition for UART_CTRL[18] - STOP bits select
 */
#define UART_CTRL_STOP_SEL_POS                   (18)
#define UART_CTRL_STOP_SEL_MSK                   (0x1UL << UART_CTRL_STOP_SEL_POS)
#define UART_CTRL_STOP_SEL_2BIT                  (0x1UL << UART_CTRL_STOP_SEL_POS)
#define UART_CTRL_STOP_SEL_1BIT                  (0x0UL << UART_CTRL_STOP_SEL_POS)

/*
 * Bit definition for UART_CTRL[17] - UART send idle enable
 */
#define UART_CTRL_SEND_IDLE_POS                  (17)
#define UART_CTRL_SEND_IDLE_MSK                  (0x1UL << UART_CTRL_SEND_IDLE_POS)
#define UART_CTRL_SEND_IDLE_ENABLE               (0x1UL << UART_CTRL_SEND_IDLE_POS)
#define UART_CTRL_SEND_IDLE_DISABLE              (0x0UL << UART_CTRL_SEND_IDLE_POS)

/*
 * Bit definition for UART_CTRL[15] - UART received break interrupt enable
 */
#define UART_CTRL_RX_BRK_INT_EN_POS              (15)
#define UART_CTRL_RX_BRK_INT_EN_MSK              (0x1UL << UART_CTRL_RX_BRK_INT_EN_POS)
#define UART_CTRL_RX_BRK_INT_ENABLE              (0x1UL << UART_CTRL_RX_BRK_INT_EN_POS)
#define UART_CTRL_RX_BRK_INT_DISABLE             (0x0UL << UART_CTRL_RX_BRK_INT_EN_POS)

/*
 * Bit definition for UART_CTRL[14] - UART detected idle interrupt enable
 */
#define UART_CTRL_RX_IDLE_INT_EN_POS             (14)
#define UART_CTRL_RX_IDLE_INT_EN_MSK             (0x1UL << UART_CTRL_RX_IDLE_INT_EN_POS)
#define UART_CTRL_RX_IDLE_INT_ENABLE             (0x1UL << UART_CTRL_RX_IDLE_INT_EN_POS)
#define UART_CTRL_RX_IDLE_INT_DISABLE            (0x0UL << UART_CTRL_RX_IDLE_INT_EN_POS)

/*
 * Bit definition for UART_CTRL[13] - UART received a word interrupt enable
 */
#define UART_CTRL_RX_INT_EN_POS                  (13)
#define UART_CTRL_RX_INT_EN_MSK                  (0x1UL << UART_CTRL_RX_INT_EN_POS)
#define UART_CTRL_RX_INT_ENABLE                  (0x1UL << UART_CTRL_RX_INT_EN_POS)
#define UART_CTRL_RX_INT_DISABLE                 (0x0UL << UART_CTRL_RX_INT_EN_POS)

/*
 * Bit definition for UART_CTRL[12] - UART transmission done interrupt enable
 */
#define UART_CTRL_TX_INT_EN_POS                  (12)
#define UART_CTRL_TX_INT_EN_MSK                  (0x1UL << UART_CTRL_TX_INT_EN_POS)
#define UART_CTRL_TX_INT_ENABLE                  (0x1UL << UART_CTRL_TX_INT_EN_POS)
#define UART_CTRL_TX_INT_DISABLE                 (0x0UL << UART_CTRL_TX_INT_EN_POS)

/*
 * Bit definition for UART_CTRL[11] - Parity selection
 */
#define UART_CTRL_PRITY_SEL_POS                  (11)
#define UART_CTRL_PRITY_SEL_MSK                  (0x1UL << UART_CTRL_PRITY_SEL_POS)
#define UART_CTRL_PRITY_SEL_ODD                  (0x1UL << UART_CTRL_PRITY_SEL_POS)
#define UART_CTRL_PRITY_SEL_EVEN                 (0x0UL << UART_CTRL_PRITY_SEL_POS)

/*
 * Bit definition for UART_CTRL[10] - Parity check enable
 */
#define UART_CTRL_PARITY_CHK_EN_POS              (10)
#define UART_CTRL_PARITY_CHK_EN_MSK              (0x1UL << UART_CTRL_PARITY_CHK_EN_POS)
#define UART_CTRL_PARITY_CHK_ENABLE              (0x1UL << UART_CTRL_PARITY_CHK_EN_POS)
#define UART_CTRL_PARITY_CHK_DISABLE             (0x0UL << UART_CTRL_PARITY_CHK_EN_POS)

/*
 * Bit definition for UART_CTRL[9] - Parity bit enable
 */
#define UART_CTRL_PARITY_BIT_POS                 (9)
#define UART_CTRL_PARITY_BIT_MSK                 (0x1UL << UART_CTRL_PARITY_BIT_POS)
#define UART_CTRL_PARITY_BIT_ENABLE              (0x1UL << UART_CTRL_PARITY_BIT_POS)
#define UART_CTRL_PARITY_BIT_DISABLE             (0x0UL << UART_CTRL_PARITY_BIT_POS)

/*
 * Bit definition for UART_CTRL[8] - UART enable
 */
#define UART_CTRL_UART_EN_POS                    (8)
#define UART_CTRL_UART_EN_MSK                    (0x1UL << UART_CTRL_UART_EN_POS)
#define UART_CTRL_UART_ENABLE                    (0x1UL << UART_CTRL_UART_EN_POS)
#define UART_CTRL_UART_DISABLE                   (0x0UL << UART_CTRL_UART_EN_POS)

/*
 * Bit definition for UART_CTRL[7] - UART0 FIFO enable
 */
#define UART_CTRL_UART0_FIFO_EN_POS              (7)
#define UART_CTRL_UART0_FIFO_EN_MSK              (0x1UL << UART_CTRL_UART0_FIFO_EN_POS)
#define UART_CTRL_UART0_FIFO_ENABLE              (0x1UL << UART_CTRL_UART0_FIFO_EN_POS)
#define UART_CTRL_UART0_FIFO_DISABLE             (0x0UL << UART_CTRL_UART0_FIFO_EN_POS)

/*
 * Bit definition for UART_CTRL[1] - UART receiver enable
 */
#define UART_CTRL_UART_RX_EN_POS                 (1)
#define UART_CTRL_UART_RX_EN_MSK                 (0x1UL << UART_CTRL_UART_RX_EN_POS)
#define UART_CTRL_UART_RX_ENABLE                 (0x1UL << UART_CTRL_UART_RX_EN_POS)
#define UART_CTRL_UART_RX_DISABLE                (0x0UL << UART_CTRL_UART_RX_EN_POS)

/*
 * Bit definition for UART_CTRL[0] - UART transmitter enable
 */
#define UART_CTRL_UART_TX_EN_POS                 (0)
#define UART_CTRL_UART_TX_EN_MSK                 (0x1UL << UART_CTRL_UART_TX_EN_POS)
#define UART_CTRL_UART_TX_ENABLE                 (0x1UL << UART_CTRL_UART_TX_EN_POS)
#define UART_CTRL_UART_TX_DISABLE                (0x0UL << UART_CTRL_UART_TX_EN_POS)

/*
 * Bit definition for UART_STS[23] - UART RX Interrupt flag
 */
#define UART_STS_RX_INT_POS                      (23)
#define UART_STS_RX_INT_MSK                      (0x1UL << UART_STS_RX_INT_POS)
#define UART_STS_RX_INT_FLAG                     (0x1UL << UART_STS_RX_INT_POS)

/*
 * Bit definition for UART_STS[22] - UART TX Interrupt flag
 */
#define UART_STS_TX_INT_POS                      (22)
#define UART_STS_TX_INT_MSK                      (0x1UL << UART_STS_TX_INT_POS)
#define UART_STS_TX_INT_FLAG                     (0x1UL << UART_STS_TX_INT_POS)

/*
 * Bit definition for UART_STS[19] - UART TX FIFO Empty flag
 */
#define UART_STS_TX_FIFO_EMPTY_POS               (19)
#define UART_STS_TX_FIFO_EMPTY_MSK               (0x1UL << UART_STS_TX_FIFO_EMPTY_POS)
#define UART_STS_TX_FIFO_EMPTY_FLAG              (0x1UL << UART_STS_TX_FIFO_EMPTY_POS)

/*
 * Bit definition for UART_STS[18] - UART RX FIFO full flag
 */
#define UART_STS_RX_FIFO_FULL_POS                (18)
#define UART_STS_RX_FIFO_FULL_MSK                (0x1UL << UART_STS_RX_FIFO_FULL_POS)
#define UART_STS_RX_FIFO_FULL_FLAG               (0x1UL << UART_STS_RX_FIFO_FULL_POS)

/*
 * Bit definition for UART_STS[17] - UART TX FIFO full flag
 */
#define UART_STS_TX_FIFO_FULL_POS                (17)
#define UART_STS_TX_FIFO_FULL_MSK                (0x1UL << UART_STS_TX_FIFO_FULL_POS)
#define UART_STS_TX_FIFO_FULL_FLAG               (0x1UL << UART_STS_TX_FIFO_FULL_POS)

/*
 * Bit definition for UART_STS[16] - UART RX FIFO empty flag
 */
#define UART_STS_RX_FIFO_EMPTY_POS               (16)
#define UART_STS_RX_FIFO_EMPTY_MSK               (0x1UL << UART_STS_RX_FIFO_EMPTY_POS)
#define UART_STS_RX_FIFO_EMPTY_FLAG              (0x1UL << UART_STS_RX_FIFO_EMPTY_POS)

/*
 * Bit definition for UART_STS[15] - UART receiver break detected flag
 */
#define UART_STS_RX_BRK_POS                      (15)
#define UART_STS_RX_BRK_MSK                      (0x1UL << UART_STS_RX_BRK_POS)
#define UART_STS_RX_BRK_FLAG                     (0x1UL << UART_STS_RX_BRK_POS)

/*
 * Bit definition for UART_STS[14] - UART Rx idle detected flag
 */
#define UART_STS_RX_IDLE_POS                     (14)
#define UART_STS_RX_IDLE_MSK                     (0x1UL << UART_STS_RX_IDLE_POS)
#define UART_STS_RX_IDLE_FLAG                    (0x1UL << UART_STS_RX_IDLE_POS)

/*
 * Bit definition for UART_STS[13] - UART read data register not empty flag
 */
#define UART_STS_RX_DAT_NEMP_POS                 (13)
#define UART_STS_RX_DAT_NEMP_MSK                 (0x1UL << UART_STS_RX_DAT_NEMP_POS)
#define UART_STS_RX_DAT_NEMP_FLAG                (0x1UL << UART_STS_RX_DAT_NEMP_POS)

/*
 * Bit definition for UART_STS[12] - UART transmission complete flag
 */
#define UART_STS_TX_DONE_POS                     (12)
#define UART_STS_TX_DONE_MSK                     (0x1UL << UART_STS_TX_DONE_POS)
#define UART_STS_TX_DONE_FLAG                    (0x1UL << UART_STS_TX_DONE_POS)

/*
 * Bit definition for UART_STS[8] - UART receiver stop bit error flag
 */
#define UART_STS_RX_STP_ERR_POS                  (8)
#define UART_STS_RX_STP_ERR_MSK                  (0x1UL << UART_STS_RX_STP_ERR_POS)
#define UART_STS_RX_STP_ERR_FLAG                 (0x1UL << UART_STS_RX_STP_ERR_POS)

/*
 * Bit definition for UART_STS[7] - UART receiver parity error flag
 */
#define UART_STS_RX_PARITY_ERR_POS               (7)
#define UART_STS_RX_PARITY_ERR_MSK               (0x1UL << UART_STS_RX_PARITY_ERR_POS)
#define UART_STS_RX_PARITY_ERR_FLAG              (0x1UL << UART_STS_RX_PARITY_ERR_POS)

/*
 * Bit definition for UART_STS[6] - UART receiver parity bit
 */
#define UART_STS_RX_PRITY_POS                    (6)
#define UART_STS_RX_PRITY_MSK                    (0x1UL << UART_STS_RX_PRITY_POS)
#define UART_STS_RX_PRITY_BIT                    (0x1UL << UART_STS_RX_PRITY_POS)

/*
 * Bit definition for UART_STS[5] - UART over run error flag
 */
#define UART_STS_RX_OV_RUN_POS                   (5)
#define UART_STS_RX_OV_RUN_MSK                   (0x1UL << UART_STS_RX_OV_RUN_POS)
#define UART_STS_RX_OV_RUN_FLAG                  (0x1UL << UART_STS_RX_OV_RUN_POS)

/*
 * Bit definition for UART_STS[1] - UART transmit data register empty flag
 */
#define UART_STS_TX_DAT_EMP_POS                  (1)
#define UART_STS_TX_DAT_EMP_MSK                  (0x1UL << UART_STS_TX_DAT_EMP_POS)
#define UART_STS_TX_DAT_EMP_FLAG                 (0x1UL << UART_STS_TX_DAT_EMP_POS)

/*
 * Bit definition for UART_BAUD_RATE[20] - Baud rate compensate selection
 */
#define UART_BAUD_RATE_BR_CMP_SEL_POS            (20)
#define UART_BAUD_RATE_BR_CMP_SEL_MSK            (0x1UL << UART_BAUD_RATE_BR_CMP_SEL_POS)
#define UART_BAUD_RATE_BR_CMP_SEL_SUB            (0x1UL << UART_BAUD_RATE_BR_CMP_SEL_POS)
#define UART_BAUD_RATE_BR_CMP_SEL_ADD            (0x0UL << UART_BAUD_RATE_BR_CMP_SEL_POS)

/*
 * Bit definition for UART_BAUD_RATE[19:16] - Baud rate compensate cycles
 */
#define UART_BAUD_RATE_BR_CMP_CYCLE_POS          (16)
#define UART_BAUD_RATE_BR_CMP_CYCLE_MSK          (0xFUL << UART_BAUD_RATE_BR_CMP_CYCLE_POS)

/*
 * Bit definition for UART_BAUD_RATE[11:0] - Baud rate divider
 */
#define UART_BAUD_RATE_BR_DIV_POS                (0)
#define UART_BAUD_RATE_BR_DIV_MSK                (0xFFFUL << UART_BAUD_RATE_BR_DIV_POS)

/*
 * Bit definition for UART_FIFO[14:12] - Transmit FIFO Interrupt Level Register
 */
#define UART_FIFO_TX_LEVEL_POS                   (12)
#define UART_FIFO_TX_LEVEL_MSK                   (0x7UL << UART_FIFO_TX_LEVEL_POS)
#define UART_FIFO_TX_LEVEL_0                     (0x0UL << UART_FIFO_TX_LEVEL_POS)
#define UART_FIFO_TX_LEVEL_1                     (0x1UL << UART_FIFO_TX_LEVEL_POS)
#define UART_FIFO_TX_LEVEL_2                     (0x2UL << UART_FIFO_TX_LEVEL_POS)
#define UART_FIFO_TX_LEVEL_3                     (0x3UL << UART_FIFO_TX_LEVEL_POS)
#define UART_FIFO_TX_LEVEL_4                     (0x4UL << UART_FIFO_TX_LEVEL_POS)
#define UART_FIFO_TX_LEVEL_5                     (0x5UL << UART_FIFO_TX_LEVEL_POS)
#define UART_FIFO_TX_LEVEL_6                     (0x6UL << UART_FIFO_TX_LEVEL_POS)
#define UART_FIFO_TX_LEVEL_7                     (0x7UL << UART_FIFO_TX_LEVEL_POS)

/*
 * Bit definition for UART_FIFO[10:8] - Transmit FIFO Data Level
 */
#define UART_FIFO_TX_FLAG_POS                    (8)
#define UART_FIFO_TX_FLAG_MSK                    (0x7UL << UART_FIFO_TX_FLAG_POS)

/*
 * Bit definition for UART_FIFO[6:4] - Receive FIFO Interrupt Level Register
 */
#define UART_FIFO_RX_LEVEL_POS                   (4)
#define UART_FIFO_RX_LEVEL_MSK                   (0x7UL << UART_FIFO_RX_LEVEL_POS)
#define UART_FIFO_RX_LEVEL_0                     (0x0UL << UART_FIFO_RX_LEVEL_POS)
#define UART_FIFO_RX_LEVEL_1                     (0x1UL << UART_FIFO_RX_LEVEL_POS)
#define UART_FIFO_RX_LEVEL_2                     (0x2UL << UART_FIFO_RX_LEVEL_POS)
#define UART_FIFO_RX_LEVEL_3                     (0x3UL << UART_FIFO_RX_LEVEL_POS)
#define UART_FIFO_RX_LEVEL_4                     (0x4UL << UART_FIFO_RX_LEVEL_POS)
#define UART_FIFO_RX_LEVEL_5                     (0x5UL << UART_FIFO_RX_LEVEL_POS)
#define UART_FIFO_RX_LEVEL_6                     (0x6UL << UART_FIFO_RX_LEVEL_POS)
#define UART_FIFO_RX_LEVEL_7                     (0x7UL << UART_FIFO_RX_LEVEL_POS)

/*
 * Bit definition for UART_FIFO[2:0] - Receive FIFO Data Level
 */
#define UART_FIFO_RX_FLAG_POS                    (0)
#define UART_FIFO_RX_FLAG_MSK                    (0x7UL << UART_FIFO_RX_FLAG_POS)


/*---------------------------------------------------------------------------------------
 * I2C Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;                          // I2C Control Register
    __IO uint32_t STS;                           // I2C Status Register
    __IO uint32_t ADDR;                          // I2C Address Register
    __IO uint32_t DATA;                          // I2C Data Register
} I2C_TYPE_DEF;

/*
 * Bit definition for I2C_CTRL[26] - I2C Rx mode DMA enable
 */
#define I2C_CTRL_RX_DMA_EN_POS                   (26)
#define I2C_CTRL_RX_DMA_EN_MSK                   (0x1UL << I2C_CTRL_RX_DMA_EN_POS)
#define I2C_CTRL_RX_DMA_ENABLE                   (0x1UL << I2C_CTRL_RX_DMA_EN_POS)
#define I2C_CTRL_RX_DMA_DISABLE                  (0x0UL << I2C_CTRL_RX_DMA_EN_POS)

/*
 * Bit definition for I2C_CTRL[25] - I2C Tx mode DMA enable
 */
#define I2C_CTRL_TX_DMA_EN_POS                   (25)
#define I2C_CTRL_TX_DMA_EN_MSK                   (0x1UL << I2C_CTRL_TX_DMA_EN_POS)
#define I2C_CTRL_TX_DMA_ENABLE                   (0x1UL << I2C_CTRL_TX_DMA_EN_POS)
#define I2C_CTRL_TX_DMA_DISABLE                  (0x0UL << I2C_CTRL_TX_DMA_EN_POS)

/*
 * Bit definition for I2C_CTRL[24] - I2C controller operating mode select
 */
#define I2C_CTRL_MODE_SEL_POS                    (24)
#define I2C_CTRL_MODE_SEL_MSK                    (0x1UL << I2C_CTRL_MODE_SEL_POS)
#define I2C_CTRL_MODE_SEL_MASTER                 (0x1UL << I2C_CTRL_MODE_SEL_POS)
#define I2C_CTRL_MODE_SEL_SLAVE                  (0x0UL << I2C_CTRL_MODE_SEL_POS)

/*
 * Bit definition for I2C_CTRL[23] - Slave address error interrupt enable
 */
#define I2C_CTRL_ERR_SADR_INT_EN_POS             (23)
#define I2C_CTRL_ERR_SADR_INT_EN_MSK             (0x1UL << I2C_CTRL_ERR_SADR_INT_EN_POS)
#define I2C_CTRL_ERR_SADR_INT_ENABLE             (0x1UL << I2C_CTRL_ERR_SADR_INT_EN_POS)
#define I2C_CTRL_ERR_SADR_INT_DISABLE            (0x0UL << I2C_CTRL_ERR_SADR_INT_EN_POS)

/*
 * Bit definition for I2C_CTRL[22] - I2C interrupt enable
 */
#define I2C_CTRL_I2C_INT_EN_POS                  (22)
#define I2C_CTRL_I2C_INT_EN_MSK                  (0x1UL << I2C_CTRL_I2C_INT_EN_POS)
#define I2C_CTRL_I2C_INT_ENABLE                  (0x1UL << I2C_CTRL_I2C_INT_EN_POS)
#define I2C_CTRL_I2C_INT_DISABLE                 (0x0UL << I2C_CTRL_I2C_INT_EN_POS)

/*
 * Bit definition for I2C_CTRL[21:16] - SCL / SDA input de-bounce time
 */
#define I2C_CTRL_DB_TIME_POS                     (16)
#define I2C_CTRL_DB_TIME_MSK                     (0x3FUL << I2C_CTRL_DB_TIME_POS)

/*
 * Bit definition for I2C_CTRL[15] - I2C controller issued start command enable
 */
#define I2C_CTRL_MST_STR_POS                     (15)
#define I2C_CTRL_MST_STR_MSK                     (0x1UL << I2C_CTRL_MST_STR_POS)
#define I2C_CTRL_MST_STR                         (0x1UL << I2C_CTRL_MST_STR_POS)

/*
 * Bit definition for I2C_CTRL[14] - I2C controller issued stop command enable
 */
#define I2C_CTRL_MST_STP_POS                     (14)
#define I2C_CTRL_MST_STP_MSK                     (0x1UL << I2C_CTRL_MST_STP_POS)
#define I2C_CTRL_MST_STP                         (0x1UL << I2C_CTRL_MST_STP_POS)

/*
 * Bit definition for I2C_CTRL[13] - I2C controller issued non-acknowledge
 */
#define I2C_CTRL_MST_NACK_POS                    (13)
#define I2C_CTRL_MST_NACK_MSK                    (0x1UL << I2C_CTRL_MST_NACK_POS)
#define I2C_CTRL_MST_NACK                        (0x1UL << I2C_CTRL_MST_NACK_POS)

/*
 * Bit definition for I2C_CTRL[11:9] - I2C controller serial clock select
 */
#define I2C_CTRL_CLK_SEL_POS                     (9)
#define I2C_CTRL_CLK_SEL_MSK                     (0x7UL << I2C_CTRL_CLK_SEL_POS)
#define I2C_CTRL_CLK_SEL_HCLK_DIV_1024           (0x6UL << I2C_CTRL_CLK_SEL_POS)
#define I2C_CTRL_CLK_SEL_HCLK_DIV_768            (0x5UL << I2C_CTRL_CLK_SEL_POS)
#define I2C_CTRL_CLK_SEL_HCLK_DIV_256            (0x4UL << I2C_CTRL_CLK_SEL_POS)
#define I2C_CTRL_CLK_SEL_HCLK_DIV_128            (0x3UL << I2C_CTRL_CLK_SEL_POS)
#define I2C_CTRL_CLK_SEL_HCLK_DIV_64             (0x2UL << I2C_CTRL_CLK_SEL_POS)
#define I2C_CTRL_CLK_SEL_HCLK_DIV_32             (0x1UL << I2C_CTRL_CLK_SEL_POS)
#define I2C_CTRL_CLK_SEL_HCLK_DIV_16             (0x0UL << I2C_CTRL_CLK_SEL_POS)

/*
 * Bit definition for I2C_CTRL[8] - I2C controller enable bit
 */
#define I2C_CTRL_I2C_EN_POS                      (8)
#define I2C_CTRL_I2C_EN_MSK                      (0x1UL << I2C_CTRL_I2C_EN_POS)
#define I2C_CTRL_I2C_ENABLE                      (0x1UL << I2C_CTRL_I2C_EN_POS)
#define I2C_CTRL_I2C_DISABLE                     (0x0UL << I2C_CTRL_I2C_EN_POS)

/*
 * Bit definition for I2C_CTRL[0] - I2C start transmission trigger bit
 */
#define I2C_CTRL_I2C_TRG_POS                     (0)
#define I2C_CTRL_I2C_TRG_MSK                     (0x1UL << I2C_CTRL_I2C_TRG_POS)
#define I2C_CTRL_I2C_TRIGGER                     (0x1UL << I2C_CTRL_I2C_TRG_POS)

/*
 * Bit definition for I2C_STS[11] - I2C Slave address error flag
 */
#define I2C_STS_SLV_ADR_ERR_POS                  (11)
#define I2C_STS_SLV_ADR_ERR_MSK                  (0x1UL << I2C_STS_SLV_ADR_ERR_POS)
#define I2C_STS_SLV_ADR_ERR_FLAG                 (0x1UL << I2C_STS_SLV_ADR_ERR_POS)

/*
 * Bit definition for I2C_STS[10] - I2C controller busy flag
 */
#define I2C_STS_BUSY_POS                         (10)
#define I2C_STS_BUSY_MSK                         (0x1UL << I2C_STS_BUSY_POS)
#define I2C_STS_BUSY_FLAG                        (0x1UL << I2C_STS_BUSY_POS)

/*
 * Bit definition for I2C_STS[9] - I2C general call flag
 */
#define I2C_STS_GEN_CALL_POS                     (9)
#define I2C_STS_GEN_CALL_MSK                     (0x1UL << I2C_STS_GEN_CALL_POS)
#define I2C_STS_GEN_CALL_FLAG                    (0x1UL << I2C_STS_GEN_CALL_POS)

/*
 * Bit definition for I2C_STS[6] - I2C bus arbitration lost flag
 */
#define I2C_STS_ARB_LOST_POS                     (6)
#define I2C_STS_ARB_LOST_MSK                     (0x1UL << I2C_STS_ARB_LOST_POS)
#define I2C_STS_ARB_LOST_FLAG                    (0x1UL << I2C_STS_ARB_LOST_POS)

/*
 * Bit definition for I2C_STS[4] - I2C Slave address received flag
 */
#define I2C_STS_SLV_DATA_DONE_POS                (4)
#define I2C_STS_SLV_DATA_DONE_MSK                (0x1UL << I2C_STS_SLV_DATA_DONE_POS)
#define I2C_STS_SLV_DATA_DONE_FLAG               (0x1UL << I2C_STS_SLV_DATA_DONE_POS)

/*
 * Bit definition for I2C_STS[3] - I2C Tx/Rx Data Done flag
 */
#define I2C_STS_DATA_DONE_POS                    (3)
#define I2C_STS_DATA_DONE_MSK                    (0x1UL << I2C_STS_DATA_DONE_POS)
#define I2C_STS_DATA_DONE_FLAG                   (0x1UL << I2C_STS_DATA_DONE_POS)

/*
 * Bit definition for I2C_STS[2] - I2C Stop command received flag
 */
#define I2C_STS_RX_STOP_CMD_POS                  (2)
#define I2C_STS_RX_STOP_CMD_MSK                  (0x1UL << I2C_STS_RX_STOP_CMD_POS)
#define I2C_STS_RX_STOP_CMD_FLAG                 (0x1UL << I2C_STS_RX_STOP_CMD_POS)

/*
 * Bit definition for I2C_STS[1] - I2C No Ack received flag
 */
#define I2C_STS_RX_NO_ACK_POS                    (1)
#define I2C_STS_RX_NO_ACK_MSK                    (0x1UL << I2C_STS_RX_NO_ACK_POS)
#define I2C_STS_RX_NO_ACK_FLAG                   (0x1UL << I2C_STS_RX_NO_ACK_POS)

/*
 * Bit definition for I2C_STS[0] - I2C transmission complete flag
 */
#define I2C_STS_TRS_DONE_POS                     (0)
#define I2C_STS_TRS_DONE_MSK                     (0x1UL << I2C_STS_TRS_DONE_POS)
#define I2C_STS_TRS_DONE_FLAG                    (0x1UL << I2C_STS_TRS_DONE_POS)

/*
 * Bit definition for I2C_ADDR[16] - I2C address mode control bit
 */
#define I2C_ADDR_MODE_POS                        (16)
#define I2C_ADDR_MODE_10BIT                      (0x1UL << I2C_ADDR_MODE_POS)
#define I2C_ADDR_MODE_7BIT                       (0x0UL << I2C_ADDR_MODE_POS)

/*
 * Bit definition for I2C_ADDR[15:1] - I2C slave address
 */
#define I2C_ADDR_ADDRESS_POS                     (1)
#define I2C_ADDR_ADDRESS_MSK                     (0x7FFFUL << I2C_ADDR_ADDRESS_POS)

/*
 * Bit definition for I2C_ADDR[0] - I2C read/write control bit
 */
#define I2C_RW_SEL_POS                           (0)
#define I2C_RW_SEL_MSK                           (0x1UL << I2C_RW_SEL_POS)
#define I2C_RW_SEL_READ                          (0x1UL << I2C_RW_SEL_POS)
#define I2C_RW_SEL_WRITE                         (0x0UL << I2C_RW_SEL_POS)


/*---------------------------------------------------------------------------------------
 * I2S Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t RX_CTRL;                       // I2S RX Control Register
    __IO uint32_t RX_DATA;                       // I2S Receive Data Register
    __IO uint32_t RX_STS;                        // I2S Status Register
    __IO uint32_t RX_CTRL2;                      // I2S RX Control2 Register
    __IO uint32_t TX_CTRL;                       // I2S TX Control Register
    __IO uint32_t TX_CTRL2;                      // I2S TX Control2 Register
} I2S_TYPE_DEF;

/*
 * Bit definition for RX_CTRL[21] - Mono Mode Enable
 */
#define I2S_RX_CTRL_MONO_MODE_POS                (21)
#define I2S_RX_CTRL_MONO_MODE_MSK                (0x1UL << I2S_RX_CTRL_MONO_MODE_POS)
#define I2S_RX_CTRL_MONO_MODE_ENABLE             (0x1UL << I2S_RX_CTRL_MONO_MODE_POS)
#define I2S_RX_CTRL_MONO_MODE_DISABLE            (0x0UL << I2S_RX_CTRL_MONO_MODE_POS)

/*
 * Bit definition for RX_CTRL[20] - Right Channel At LSB
 */
#define I2S_RX_CTRL_RIGHT_LSB_POS                (20)
#define I2S_RX_CTRL_RIGHT_LSB_MSK                (0x1UL << I2S_RX_CTRL_RIGHT_LSB_POS)
#define I2S_RX_CTRL_RIGHT_LSB_ENABLE             (0x1UL << I2S_RX_CTRL_RIGHT_LSB_POS)
#define I2S_RX_CTRL_RIGHT_LSB_DISABLE            (0x0UL << I2S_RX_CTRL_RIGHT_LSB_POS)

/*
 * Bit definition for RX_CTRL[19] - MERGE
 */
#define I2S_RX_CTRL_MERGE_POS                    (19)
#define I2S_RX_CTRL_MERGE_MSK                    (0x1UL << I2S_RX_CTRL_MERGE_POS)
#define I2S_RX_CTRL_MERGE_ENABLE                 (0x1UL << I2S_RX_CTRL_MERGE_POS)
#define I2S_RX_CTRL_MERGE_DISABLE                (0x0UL << I2S_RX_CTRL_MERGE_POS)

/*
 * Bit definition for RX_CTRL[16] - Enable Half Full Interrupt
 */
#define I2S_RX_CTRL_HALF_FULL_INT_POS            (16)
#define I2S_RX_CTRL_HALF_FULL_INT_MSK            (0x1UL << I2S_RX_CTRL_HALF_FULL_INT_POS)
#define I2S_RX_CTRL_HALF_FULL_INT_ENABLE         (0x1UL << I2S_RX_CTRL_HALF_FULL_INT_POS)
#define I2S_RX_CTRL_HALF_FULL_INT_DISABLE        (0x0UL << I2S_RX_CTRL_HALF_FULL_INT_POS)

/*
 * Bit definition for RX_CTRL[14] - INT Rising or Falling edge triggered.
 */
#define I2S_RX_CTRL_INT_SEL_POS                  (14)
#define I2S_RX_CTRL_INT_SEL_MSK                  (0x1UL << I2S_RX_CTRL_INT_SEL_POS)
#define I2S_RX_CTRL_INT_SEL_RISING               (0x1UL << I2S_RX_CTRL_INT_SEL_POS)
#define I2S_RX_CTRL_INT_SEL_FALLING              (0x0UL << I2S_RX_CTRL_INT_SEL_POS)

/*
 * Bit definition for RX_CTRL[13] - Master Mode
 */
#define I2S_RX_CTRL_MODE_SEL_POS                 (13)
#define I2S_RX_CTRL_MODE_SEL_MSK                 (0x1UL << I2S_RX_CTRL_MODE_SEL_POS)
#define I2S_RX_CTRL_MODE_SEL_MASTER              (0x1UL << I2S_RX_CTRL_MODE_SEL_POS)
#define I2S_RX_CTRL_MODE_SEL_SLAVE               (0x0UL << I2S_RX_CTRL_MODE_SEL_POS)

/*
* Bit definition for RX_CTRL[12:11] - Framing Mode
 */
#define I2S_RX_CTRL_FRAME_MODE_POS               (11)
#define I2S_RX_CTRL_FRAME_MODE_MSK               (0x3UL << I2S_RX_CTRL_FRAME_MODE_POS)
#define I2S_RX_CTRL_FRAME_MODE_NORMAL            (0x1UL << I2S_RX_CTRL_FRAME_MODE_POS)
#define I2S_RX_CTRL_FRAME_MODE_I2S               (0x0UL << I2S_RX_CTRL_FRAME_MODE_POS)

/*
* Bit definition for RX_CTRL[10:9] - Frame size
 */
#define I2S_RX_CTRL_FRAME_SIZE_POS               (9)
#define I2S_RX_CTRL_FRAME_SIZE_MSK               (0x3UL << I2S_RX_CTRL_FRAME_SIZE_POS)
#define I2S_RX_CTRL_FRAME_SIZE_Slave             (0x3UL << I2S_RX_CTRL_FRAME_SIZE_POS)
#define I2S_RX_CTRL_FRAME_SIZE_32BITS            (0x2UL << I2S_RX_CTRL_FRAME_SIZE_POS)
#define I2S_RX_CTRL_FRAME_SIZE_24BITS            (0x1UL << I2S_RX_CTRL_FRAME_SIZE_POS)
#define I2S_RX_CTRL_FRAME_SIZE_16BITS            (0x0UL << I2S_RX_CTRL_FRAME_SIZE_POS)

/*
 * Bit definition for RX_CTRL[8:6] - Valid Data Length
 */
#define I2S_RX_CTRL_DATA_LENGTH_POS              (6)
#define I2S_RX_CTRL_DATA_LENGTH_MSK              (0x7UL << I2S_RX_CTRL_DATA_LENGTH_POS)
#define I2S_RX_CTRL_DATA_LENGTH_32BITS           (0x5UL << I2S_RX_CTRL_DATA_LENGTH_POS)
#define I2S_RX_CTRL_DATA_LENGTH_24BITS           (0x4UL << I2S_RX_CTRL_DATA_LENGTH_POS)
#define I2S_RX_CTRL_DATA_LENGTH_22BITS           (0x3UL << I2S_RX_CTRL_DATA_LENGTH_POS)
#define I2S_RX_CTRL_DATA_LENGTH_20BITS           (0x2UL << I2S_RX_CTRL_DATA_LENGTH_POS)
#define I2S_RX_CTRL_DATA_LENGTH_18BITS           (0x1UL << I2S_RX_CTRL_DATA_LENGTH_POS)
#define I2S_RX_CTRL_DATA_LENGTH_16BITS           (0x0UL << I2S_RX_CTRL_DATA_LENGTH_POS)

/*
 * Bit definition for RX_CTRL[5] - Received Data Left Alignment,
 */
#define I2S_RX_CTRL_ALIGN_POS                    (5)
#define I2S_RX_CTRL_ALIGN_MSK                    (0x1UL << I2S_RX_CTRL_ALIGN_POS)
#define I2S_RX_CTRL_ALIGN_LEFT                   (0x1UL << I2S_RX_CTRL_ALIGN_POS)
#define I2S_RX_CTRL_ALIGN_RIGHT                  (0x0UL << I2S_RX_CTRL_ALIGN_POS)

/*
 * Bit definition for RX_CTRL[4] - I2S RX Data endianness selection
 */
#define I2S_RX_CTRL_DATA_ENDIANNESS_POS          (4)
#define I2S_RX_CTRL_DATA_ENDIANNESS_MSK          (0x1UL << I2S_RX_CTRL_DATA_ENDIANNESS_POS)
#define I2S_RX_CTRL_DATA_FIRST_MSB               (0x0UL << I2S_RX_CTRL_DATA_ENDIANNESS_POS)
#define I2S_RX_CTRL_DATA_FIRST_LSB               (0x1UL << I2S_RX_CTRL_DATA_ENDIANNESS_POS)

/*
 * Bit definition for RX_CTRL[3] - I2S receiving mode selection
 */
#define I2S_RX_CTRL_DATA_LATCH_POS               (3)
#define I2S_RX_CTRL_DATA_LATCH_MSK               (0x1UL << I2S_RX_CTRL_DATA_LATCH_POS)
#define I2S_RX_CTRL_DATA_LATCH_RISING            (0x1UL << I2S_RX_CTRL_DATA_LATCH_POS)
#define I2S_RX_CTRL_DATA_LATCH_FALLING           (0x0UL << I2S_RX_CTRL_DATA_LATCH_POS)

/*
 * Bit definition for RX_CTRL[2] - I2S RX frame polarity,,  0: LRCK=0 is the right frame (Default),    1: LRCK=0 is the left frame
 */
#define I2S_RX_CTRL_FRAME_POL_POS                (2)
#define I2S_RX_CTRL_FRAME_POL_MSK                (0x1UL << I2S_RX_CTRL_FRAME_POL_POS)
#define I2S_RX_CTRL_FRAME_RIGHT                  (0x0UL << I2S_RX_CTRL_FRAME_POL_POS)
#define I2S_RX_CTRL_FRAME_LEFT                   (0x1UL << I2S_RX_CTRL_FRAME_POL_POS)

/*
 * Bit definition for RX_CTRL[1] - I2S RX the first frame polarity,0: Left frame (Default), 1: Right frame
 */
#define I2S_RX_CTRL_1ST_FRAME_POL_POS            (1)
#define I2S_RX_CTRL_1ST_FRAME_POL_MSK            (0x1UL << I2S_RX_CTRL_1ST_FRAME_POL_POS)
#define I2S_RX_CTRL_1ST_FRAME_RIGHT              (0x1UL << I2S_RX_CTRL_1ST_FRAME_POL_POS)
#define I2S_RX_CTRL_1ST_FRAME_LEFT               (0x0UL << I2S_RX_CTRL_1ST_FRAME_POL_POS)

/*
 * Bit definition for RX_CTRL[0] - I2S RX enable bit
 */
#define I2S_RX_CTRL_EN_POS                       (0)
#define I2S_RX_CTRL_EN_MSK                       (0x1UL << I2S_RX_CTRL_EN_POS)
#define I2S_RX_CTRL_ENABLE                       (0x1UL << I2S_RX_CTRL_EN_POS)
#define I2S_RX_CTRL_DISABLE                      (0x0UL << I2S_RX_CTRL_EN_POS)

/*
 * Bit definition for RX_STS[18] - FIFO Clear
 */
#define I2S_RX_STS_CLEAR_FIFO_POS                (18)
#define I2S_RX_STS_CLEAR_FIFO_MSK                (0x1UL << I2S_RX_STS_CLEAR_FIFO_POS)
#define I2S_RX_STS_CLEAR_FIFO_ENABLE             (0x1UL << I2S_RX_STS_CLEAR_FIFO_POS)
#define I2S_RX_STS_CLEAR_FIFO_DISABLE            (0x0UL << I2S_RX_STS_CLEAR_FIFO_POS)

/*
 * Bit definition for RX_STS[17] - HALF FULL FLAG
 */
#define I2S_RX_STS_HALF_FULL_FLAG_POS            (17)
#define I2S_RX_STS_HALF_FULL_FLAG_MSK            (0x1UL << I2S_RX_STS_HALF_FULL_FLAG_POS)
#define I2S_RX_STS_HALF_FULL_FLAG                (0x1UL << I2S_RX_STS_HALF_FULL_FLAG_POS)

/*
 * Bit definition for RX_STS[15] - Overflow Flag
 */
#define I2S_RX_STS_OVERFLOW_FLAG_POS             (15)
#define I2S_RX_STS_OVERFLOW_FLAG_MSK             (0x1UL << I2S_RX_STS_OVERFLOW_FLAG_POS)
#define I2S_RX_STS_OVERFLOW_FLAG                 (0x1UL << I2S_RX_STS_OVERFLOW_FLAG_POS)

/*
 * Bit definition for RX_STS[3:2] - RX FIFO NUMBER
 */
#define I2S_RX_CTRL_FIFO_INT_LEVEL_POS           (2)
#define I2S_RX_CTRL_FIFO_INT_LEVEL_MSK           (0x3FUL << I2S_RX_CTRL_FIFO_INT_LEVEL_POS)

/*
 * Bit definition for RX_STS[1] - RX FIFO Full Flag
 */
#define I2S_RX_CTRL_FIFO_FULL_FLAG_POS           (1)
#define I2S_RX_CTRL_FIFO_FULL_FLAG_MSK           (0x1UL << I2S_RX_CTRL_FIFO_FULL_FLAG_POS)
#define I2S_RX_CTRL_FIFO_FULL_FLAG               (0x1UL << I2S_RX_CTRL_FIFO_FULL_FLAG_POS)

/*
 * Bit definition for RX_STS[0] - RX FIFO Empty Flag
 */
#define I2S_RX_CTRL_FIFO_EMPTY_FLAG_POS          (0)
#define I2S_RX_CTRL_FIFO_EMPTY_FLAG_MSK          (0x1UL << I2S_RX_CTRL_FIFO_EMPTY_FLAG_POS)
#define I2S_RX_CTRL_FIFO_EMPTY_FLAG              (0x1UL << I2S_RX_CTRL_FIFO_EMPTY_FLAG_POS)

/*
 * Bit definition for RX_CTRL2[31:29] - BCLK_DIV
 */
#define I2S_RX_CTRL2_BCLK_DIV_SEL_POS            (29)
#define I2S_RX_CTRL2_BCLK_DIV_SEL_MSK            (0x7UL << I2S_RX_CTRL2_BCLK_DIV_SEL_POS)
#define I2S_RX_CTRL2_BCLK_DIV_2                  (0x2UL << I2S_RX_CTRL2_BCLK_DIV_SEL_POS)
#define I2S_RX_CTRL2_BCLK_DIV_3                  (0x3UL << I2S_RX_CTRL2_BCLK_DIV_SEL_POS)
#define I2S_RX_CTRL2_BCLK_DIV_4                  (0x4UL << I2S_RX_CTRL2_BCLK_DIV_SEL_POS)
#define I2S_RX_CTRL2_BCLK_DIV_6                  (0x6UL << I2S_RX_CTRL2_BCLK_DIV_SEL_POS)
#define I2S_RX_CTRL2_BCLK_DIV_8                  (0x7UL << I2S_RX_CTRL2_BCLK_DIV_SEL_POS)

/*
 * Bit definition for RX_CTRL2[5:0] - MCLK_DIV
 */
#define I2S_RX_CTRL2_MCLK_DIV_POS                (0)
#define I2S_RX_CTRL2_MCLK_DIV_MSK                (0x3FUL << I2S_RX_CTRL2_MCLK_DIV_POS)
#define I2S_RX_CTRL2_MCLK_DIV_2                  (0x0UL << I2S_RX_CTRL2_MCLK_DIV_POS)
#define I2S_RX_CTRL2_MCLK_DIV_3                  (0x2UL << I2S_RX_CTRL2_MCLK_DIV_POS)
#define I2S_RX_CTRL2_MCLK_DIV_4                  (0x3UL << I2S_RX_CTRL2_MCLK_DIV_POS)
#define I2S_RX_CTRL2_MCLK_DIV_5                  (0x4UL << I2S_RX_CTRL2_MCLK_DIV_POS)
#define I2S_RX_CTRL2_MCLK_DIV_6                  (0x5UL << I2S_RX_CTRL2_MCLK_DIV_POS)
#define I2S_RX_CTRL2_MCLK_DIV_7                  (0x6UL << I2S_RX_CTRL2_MCLK_DIV_POS)
#define I2S_RX_CTRL2_MCLK_DIV_8                  (0x7UL << I2S_RX_CTRL2_MCLK_DIV_POS)
#define I2S_RX_CTRL2_MCLK_DIV_9                  (0x8UL << I2S_RX_CTRL2_MCLK_DIV_POS)

/*
 * Bit definition for TX_CTRL[31:26]
 */
#define I2S_TX_CTRL_MCLK_DIV_POS                 (26)
#define I2S_TX_CTRL_MCLK_DIV_MSK                 (0x3FUL << I2S_TX_CTRL_MCLK_DIV_POS)
#define I2S_TX_CTRL_MCLK_DIV_2                   (0x0UL << I2S_TX_CTRL_MCLK_DIV_POS)
#define I2S_TX_CTRL_MCLK_DIV_3                   (0x2UL << I2S_TX_CTRL_MCLK_DIV_POS)
#define I2S_TX_CTRL_MCLK_DIV_4                   (0x3UL << I2S_TX_CTRL_MCLK_DIV_POS)
#define I2S_TX_CTRL_MCLK_DIV_5                   (0x4UL << I2S_TX_CTRL_MCLK_DIV_POS)
#define I2S_TX_CTRL_MCLK_DIV_6                   (0x5UL << I2S_TX_CTRL_MCLK_DIV_POS)
#define I2S_TX_CTRL_MCLK_DIV_7                   (0x6UL << I2S_TX_CTRL_MCLK_DIV_POS)
#define I2S_TX_CTRL_MCLK_DIV_8                   (0x7UL << I2S_TX_CTRL_MCLK_DIV_POS)

/*
 * Bit definition for TX_CTRL[25:23]
 */
#define I2S_TX_CTRL_BCLK_DIV_POS                 (23)
#define I2S_TX_CTRL_BCLK_DIV_MSK                 (0x7UL << I2S_TX_CTRL_BCLK_DIV_POS)
#define I2S_TX_CTRL_BCLK_DIV_2                   (0x2UL << I2S_TX_CTRL_BCLK_DIV_POS)
#define I2S_TX_CTRL_BCLK_DIV_3                   (0x3UL << I2S_TX_CTRL_BCLK_DIV_POS)
#define I2S_TX_CTRL_BCLK_DIV_4                   (0x4UL << I2S_TX_CTRL_BCLK_DIV_POS)
#define I2S_TX_CTRL_BCLK_DIV_6                   (0x6UL << I2S_TX_CTRL_BCLK_DIV_POS)
#define I2S_TX_CTRL_BCLK_DIV_8                   (0x7UL << I2S_TX_CTRL_BCLK_DIV_POS)

/*
 * Bit definition for TX_CTRL[19]
 */
#define I2S_TX_CTRL_DATA_FMT_POS                 (19)
#define I2S_TX_CTRL_DATA_FMT_MSK                 (0x1UL << I2S_TX_CTRL_DATA_FMT_POS)
#define I2S_TX_CTRL_DATA_FMT_UNSIGN              (0x0UL << I2S_TX_CTRL_DATA_FMT_POS)
#define I2S_TX_CTRL_DATA_FMT_SIGN                (0x1UL << I2S_TX_CTRL_DATA_FMT_POS)

/*
 * Bit definition for TX_CTRL[18]
 */
#define I2S_TX_CTRL_MCLK_EN_POS                  (18)
#define I2S_TX_CTRL_MCLK_EN_MSK                  (0x1UL << I2S_TX_CTRL_MCLK_EN_POS)
#define I2S_TX_CTRL_MCLK_ENABLE                  (0x1UL << I2S_TX_CTRL_MCLK_EN_POS)
#define I2S_TX_CTRL_MCLK_DISABLE                 (0x0UL << I2S_TX_CTRL_MCLK_EN_POS)

/*
 * Bit definition for TX_CTRL[17:16]
 */
#define I2S_TX_CTRL_DATA_SRC_POS                 (16)
#define I2S_TX_CTRL_DATA_SRC_MSK                 (0x3UL << I2S_TX_CTRL_DATA_SRC_POS)
#define I2S_TX_CTRL_DATA_SRC_POSTWAVE            (0x0UL << I2S_TX_CTRL_DATA_SRC_POS)
#define I2S_TX_CTRL_DATA_SRC_SPU                 (0x1UL << I2S_TX_CTRL_DATA_SRC_POS)
#define I2S_TX_CTRL_DATA_SRC_SOFTCH              (0x2UL << I2S_TX_CTRL_DATA_SRC_POS)

/*
 * Bit definition for TX_CTRL[15:14]
 */
#define I2S_TX_CTRL_DATA_OUT_POS                 (14)
#define I2S_TX_CTRL_DATA_OUT_MSK                 (0x3UL << I2S_TX_CTRL_DATA_OUT_POS)
#define I2S_TX_CTRL_DATA_OUT_STEREO_RL           (0x0UL << I2S_TX_CTRL_DATA_OUT_POS)
#define I2S_TX_CTRL_DATA_OUT_STEREO_LR           (0x1UL << I2S_TX_CTRL_DATA_OUT_POS)
#define I2S_TX_CTRL_DATA_OUT_MONO_L              (0x2UL << I2S_TX_CTRL_DATA_OUT_POS)
#define I2S_TX_CTRL_DATA_OUT_MONO_R              (0x3UL << I2S_TX_CTRL_DATA_OUT_POS)

/*
 * Bit definition for TX_CTRL[10:9]
 */
#define I2S_TX_CTRL_FRAME_SIZE_POS               (9)
#define I2S_TX_CTRL_FRAME_SIZE_MSK               (0x3UL << I2S_TX_CTRL_FRAME_SIZE_POS)
#define I2S_TX_CTRL_FRAME_SIZE_16BITS            (0x0UL << I2S_TX_CTRL_FRAME_SIZE_POS)
#define I2S_TX_CTRL_FRAME_SIZE_24BITS            (0x1UL << I2S_TX_CTRL_FRAME_SIZE_POS)
#define I2S_TX_CTRL_FRAME_SIZE_32BITS            (0x2UL << I2S_TX_CTRL_FRAME_SIZE_POS)

/*
 * Bit definition for TX_CTRL[5]
 */
#define I2S_TX_CTRL_ALIGN_POS                    (5)
#define I2S_TX_CTRL_ALIGN_MSK                    (0x1UL << I2S_TX_CTRL_ALIGN_POS)
#define I2S_TX_CTRL_ALIGN_RIGHT                  (0x0UL << I2S_TX_CTRL_ALIGN_POS)
#define I2S_TX_CTRL_ALIGN_LEFT                   (0x1UL << I2S_TX_CTRL_ALIGN_POS)

/*
 * Bit definition for TX_CTRL[4]
 */
#define I2S_TX_CTRL_DATA_ENDIANNESS_POS          (4)
#define I2S_TX_CTRL_DATA_ENDIANNESS_MSK          (0x1UL << I2S_TX_CTRL_DATA_ENDIANNESS_POS)
#define I2S_TX_CTRL_DATA_FIRST_MSB               (0x0UL << I2S_TX_CTRL_DATA_ENDIANNESS_POS)
#define I2S_TX_CTRL_DATA_FIRST_LSB               (0x1UL << I2S_TX_CTRL_DATA_ENDIANNESS_POS)

/*
 * Bit definition for TX_CTRL[3]
 */
#define I2S_TX_CTRL_EDGE_MODE_POS                (3)
#define I2S_TX_CTRL_EDGE_MODE_MSK                (0x1UL << I2S_TX_CTRL_EDGE_MODE_POS)
#define I2S_TX_CTRL_EDGE_MODE_RISING             (0x1UL << I2S_TX_CTRL_EDGE_MODE_POS)
#define I2S_TX_CTRL_EDGE_MODE_FALLING            (0x0UL << I2S_TX_CTRL_EDGE_MODE_POS)

/*
 * Bit definition for TX_CTRL[2]
 */
#define I2S_TX_CTRL_FRAME_MODE_POS               (2)
#define I2S_TX_CTRL_FRAME_MODE_MSK               (0x1UL << I2S_TX_CTRL_FRAME_MODE_POS)
#define I2S_TX_CTRL_FRAME_MODE_NORMAL            (0x1UL << I2S_TX_CTRL_FRAME_MODE_POS)
#define I2S_TX_CTRL_FRAME_MODE_I2S               (0x0UL << I2S_TX_CTRL_FRAME_MODE_POS)

/*
 * Bit definition for TX_CTRL[0]
 */
#define I2S_TX_CTRL_EN_POS                       (0)
#define I2S_TX_CTRL_EN_MSK                       (0x1UL << I2S_TX_CTRL_EN_POS)
#define I2S_TX_CTRL_ENABLE                       (0x1UL << I2S_TX_CTRL_EN_POS)
#define I2S_TX_CTRL_DISABLE                      (0x0UL << I2S_TX_CTRL_EN_POS)

/*
 * Bit definition for TX_CTRL2[23:16]
 */
#define I2S_TX_CTRL2_DUMMY_CYCLE_POS             (16)
#define I2S_TX_CTRL2_DUMMY_CYCLE_MSK             (0xFFUL << I2S_TX_CTRL2_DUMMY_CYCLE_POS)

/*
 * Bit definition for TX_CTRL2[1]
 */
#define I2S_TX_CTRL2_DELAY_EN_POS                (1)
#define I2S_TX_CTRL2_DELAY_EN_MSK                (0x1UL << I2S_TX_CTRL2_DELAY_EN_POS)
#define I2S_TX_CTRL2_DELAY_ENABLE                (0x1UL << I2S_TX_CTRL2_DELAY_EN_POS)
#define I2S_TX_CTRL2_DELAY_DISABLE               (0x0UL << I2S_TX_CTRL2_DELAY_EN_POS)

/*
 * Bit definition for TX_CTRL2[0]
 */
#define I2S_TX_CTRL2_DELAY_TYPE_POS              (0)
#define I2S_TX_CTRL2_DELAY_TYPE_MSK              (0x1UL << I2S_TX_CTRL2_DELAY_TYPE_POS)
#define I2S_TX_CTRL2_DELAY_DATA_CHANGE           (0x1UL << I2S_TX_CTRL2_DELAY_TYPE_POS)
#define I2S_TX_CTRL2_DELAY_DUMMY_CYCLE           (0x0UL << I2S_TX_CTRL2_DELAY_TYPE_POS)

/*---------------------------------------------------------------------------------------
 * Watchdog Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t KEYCODE;                       // Watchdog Key Code Register
    __IO uint32_t CTRL;                          // Watchdog Control Register
    __IO uint32_t STS;                           // Watchdog Status Register
} WDG_TYPE_DEF;

/*
 * Bit definition for WDG_KEYCODE[7:0] - Watch dog keycode
 */
#define WDG_KEYCODE_POS                          (0)
#define WDG_KEYCODE_MSK                          (0xFFUL << WDG_KEYCODE_POS)
#define WDG_KEYCODE_WDG_CLEAR                    (0xAAUL << WDG_KEYCODE_POS)
#define WDG_KEYCODE_RESET_DISABLE                (0x99UL << WDG_KEYCODE_POS)
#define WDG_KEYCODE_RESET_ENABLE                 (0x66UL << WDG_KEYCODE_POS)
#define WDG_KEYCODE_ACCESS_ENABLE                (0x55UL << WDG_KEYCODE_POS)
#define WDG_KEYCODE_ACCESS_DISABLE               (0x44UL << WDG_KEYCODE_POS)
#define WDG_KEYCODE_WDG_DISABLE                  (0x00UL << WDG_KEYCODE_POS)

/*
 * Bit definition for WDG_CTRL[27:16] -  Watchdog counter reload value
 */
#define WDG_CTRL_CNT_POS                         (16)
#define WDG_CTRL_CNT_MSK                         (0xFFFUL << WDG_CTRL_CNT_POS)

/*
 * Bit definition for WDG_CTRL[12] -  Illegal keycode reset enable
 */
#define WDG_CTRL_ILL_RST_EN_POS                  (12)
#define WDG_CTRL_ILL_RST_EN_MSK                  (0x1UL << WDG_CTRL_ILL_RST_EN_POS)
#define WDG_CTRL_ILL_RST_ENABLE                  (0x1UL << WDG_CTRL_ILL_RST_EN_POS)
#define WDG_CTRL_ILL_RST_DISABLE                 (0x0UL << WDG_CTRL_ILL_RST_EN_POS)

/*
 * Bit definition for WDG_CTRL[10:8] - Watchdog clock source selection
 */
#define WDG_CTRL_CLK_SEL_POS                     (8)
#define WDG_CTRL_CLK_SEL_MSK                     (0x7UL << WDG_CTRL_CLK_SEL_POS)
#define WDG_CTRL_CLK_SEL_32768_DIV_256           (0x7UL << WDG_CTRL_CLK_SEL_POS)
#define WDG_CTRL_CLK_SEL_32768_DIV_128           (0x6UL << WDG_CTRL_CLK_SEL_POS)
#define WDG_CTRL_CLK_SEL_32768_DIV_64            (0x5UL << WDG_CTRL_CLK_SEL_POS)
#define WDG_CTRL_CLK_SEL_32768_DIV_32            (0x4UL << WDG_CTRL_CLK_SEL_POS)
#define WDG_CTRL_CLK_SEL_32768_DIV_16            (0x3UL << WDG_CTRL_CLK_SEL_POS)
#define WDG_CTRL_CLK_SEL_32768_DIV_8             (0x2UL << WDG_CTRL_CLK_SEL_POS)
#define WDG_CTRL_CLK_SEL_32768_DIV_4             (0x1UL << WDG_CTRL_CLK_SEL_POS)
#define WDG_CTRL_CLK_SEL_32768_DIV_1             (0x0UL << WDG_CTRL_CLK_SEL_POS)

/*
 * Bit definition for WDG_CTRL[5] - Watchdog running without reset function
 */
#define WDG_CTRL_TIMER_MODE_FLAG_POS             (5)
#define WDG_CTRL_TIMER_MODE_FLAG_MSK             (0x1ul << WDG_CTRL_TIMER_MODE_FLAG_POS)
#define WDG_CTRL_TIMER_MODE_FLAG                 (0x1ul << WDG_CTRL_TIMER_MODE_FLAG_POS)

/*
 * Bit definition for WDG_CTRL[4] - Watchdog running with reset function
 */
#define WDG_CTRL_RESET_MODE_FLAG_POS             (4)
#define WDG_CTRL_RESET_MODE_FLAG_MSK             (0x1ul << WDG_CTRL_RESET_MODE_FLAG_POS)
#define WDG_CTRL_RESET_MODE_FLAG                 (0x1ul << WDG_CTRL_RESET_MODE_FLAG_POS)

/*
 * Bit definition for WDG_CTRL[1] - Watchdog timer timeout flag
 */
#define WDG_CTRL_TIMEOUT_FLAG_POS                (1)
#define WDG_CTRL_TIMEOUT_FLAG_MSK                (0x1UL << WDG_CTRL_TIMEOUT_FLAG_POS)
#define WDG_CTRL_TIMEOUT_FLAG                    (0x1UL << WDG_CTRL_TIMEOUT_FLAG_POS)

/*
 * Bit definition for WDG_STS[1] - Watch dog illegal access reset flag
 */
#define WDG_STS_ILL_RESET_FLAG_POS               (1)
#define WDG_STS_ILL_RESET_FLAG_MSK               (0x1UL << WDG_STS_ILL_RESET_FLAG_POS)
#define WDG_STS_ILL_RESET_FLAG                   (0x1UL << WDG_STS_ILL_RESET_FLAG_POS)

/*
 * Bit definition for WDG_STS[0] - Watch dog reset flag
 */
#define WDG_STS_RESET_FLAG_POS                   (0)
#define WDG_STS_RESET_FLAG_MSK                   (0x1ul << WDG_STS_RESET_FLAG_POS)
#define WDG_STS_RESET_FLAG                       (0x1ul << WDG_STS_RESET_FLAG_POS)


/*---------------------------------------------------------------------------------------
 * CTS Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;                          // CTS Control Register
    __IO uint32_t STS;                           // CTS Status Register
    __IO uint32_t TMAPLOAD;                      // CTS Preload Register for TimerA
    __IO uint32_t TMACOUNT;                      // CTS Counter Register for TimerA
    __IO uint32_t TMBPLOAD;                      // CTS Preload Register for TimerB
    __IO uint32_t TMBCOUNT;                      // CTS Counter Register for TimerB
    __IO uint32_t TMBCAP;                        // CTS Capture Register for TimerB
    __IO uint32_t PADSEL;                        // CTS Pad Select Register
} CTS_TYPE_DEF;

/*
 * Bit definition for CTS_CTRL[29] - CTS TMB INT Enable Select
 */
#define CTS_CTRL_TMB_INTEN_SEL_POS               (29)
#define CTS_CTRL_TMB_INTEN_SEL_MSK               (0x1UL << CTS_CTRL_TMB_INTEN_SEL_POS)
#define CTS_CTRL_TMB_INTEN_SEL_ENABLE            (0x1UL << CTS_CTRL_TMB_INTEN_SEL_POS)
#define CTS_CTRL_TMB_INTEN_SEL_DISABLE           (0x0UL << CTS_CTRL_TMB_INTEN_SEL_POS)

/*
 * Bit definition for CTS_CTRL[28] - CTS TMA INT Enable Select
 */
#define CTS_CTRL_TMA_INTEN_SEL_POS               (28)
#define CTS_CTRL_TMA_INTEN_SEL_MSK               (0x1UL << CTS_CTRL_TMA_INTEN_SEL_POS)
#define CTS_CTRL_TMA_INTEN_SEL_ENABLE            (0x1UL << CTS_CTRL_TMA_INTEN_SEL_POS)
#define CTS_CTRL_TMA_INTEN_SEL_DISABLE           (0x0UL << CTS_CTRL_TMA_INTEN_SEL_POS)

/*
 * Bit definition for CTS_CTRL[23:21] - CTS TMB Clock Source Select
 */
#define CTS_CTRL_TMB_CLK_SEL_POS                 (21)
#define CTS_CTRL_TMB_CLK_SEL_MSK                 (0x7UL << CTS_CTRL_TMB_CLK_SEL_POS)
#define CTS_CTRL_TMB_CLK_SEL_IOSC12M             (0x7UL << CTS_CTRL_TMB_CLK_SEL_POS)
#define CTS_CTRL_TMB_CLK_SEL_HCLK_DIV_32         (0x6UL << CTS_CTRL_TMB_CLK_SEL_POS)
#define CTS_CTRL_TMB_CLK_SEL_HCLK_DIV_16         (0x5UL << CTS_CTRL_TMB_CLK_SEL_POS)
#define CTS_CTRL_TMB_CLK_SEL_HCLK_DIV_8          (0x4UL << CTS_CTRL_TMB_CLK_SEL_POS)
#define CTS_CTRL_TMB_CLK_SEL_HCLK_DIV_4          (0x3UL << CTS_CTRL_TMB_CLK_SEL_POS)
#define CTS_CTRL_TMB_CLK_SEL_HCLK_DIV_2          (0x2UL << CTS_CTRL_TMB_CLK_SEL_POS)
#define CTS_CTRL_TMB_CLK_SEL_HCLK                (0x1UL << CTS_CTRL_TMB_CLK_SEL_POS)
#define CTS_CTRL_TMB_CLK_SEL_DISABLE             (0x0UL << CTS_CTRL_TMB_CLK_SEL_POS)

/*
 * Bit definition for CTS_CTRL[20] - CTS TMB dedicated for CTS timer or General timer
 */
#define CTS_CTRL_TMB_MODE_POS                    (20)
#define CTS_CTRL_TMB_MODE_MSK                    (0x1UL << CTS_CTRL_TMB_MODE_POS)
#define CTS_CTRL_TMB_MODE_GENERALTMR             (0x1UL << CTS_CTRL_TMB_MODE_POS)
#define CTS_CTRL_TMB_MODE_CTSTMR                 (0x0UL << CTS_CTRL_TMB_MODE_POS)

/*
 * Bit definition for CTS_CTRL[19:17] - CTS TMA Clock Source Select
 */
#define CTS_CTRL_TMA_CLK_SEL_POS                 (17)
#define CTS_CTRL_TMA_CLK_SEL_MSK                 (0x7UL << CTS_CTRL_TMA_CLK_SEL_POS)
#define CTS_CTRL_TMA_CLK_SEL_HCLK_DIV_16         (0x7UL << CTS_CTRL_TMA_CLK_SEL_POS)
#define CTS_CTRL_TMA_CLK_SEL_HCLK_DIV_8          (0x6UL << CTS_CTRL_TMA_CLK_SEL_POS)
#define CTS_CTRL_TMA_CLK_SEL_HCLK_DIV_4          (0x5UL << CTS_CTRL_TMA_CLK_SEL_POS)
#define CTS_CTRL_TMA_CLK_SEL_HCLK_DIV_2          (0x4UL << CTS_CTRL_TMA_CLK_SEL_POS)
#define CTS_CTRL_TMA_CLK_SEL_HCLK                (0x3UL << CTS_CTRL_TMA_CLK_SEL_POS)
#define CTS_CTRL_TMA_CLK_SEL_CTSMODULE           (0x2UL << CTS_CTRL_TMA_CLK_SEL_POS)
#define CTS_CTRL_TMA_CLK_SEL_IO                  (0x1UL << CTS_CTRL_TMA_CLK_SEL_POS)
#define CTS_CTRL_TMA_CLK_SEL_DISABLE             (0x0UL << CTS_CTRL_TMA_CLK_SEL_POS)

/*
 * Bit definition for CTS_CTRL[16] - CTS TMA dedicated for CTS timer or General timer
 */
#define CTS_CTRL_TMA_MODE_POS                    (16)
#define CTS_CTRL_TMA_MODE_MSK                    (0x1UL << CTS_CTRL_TMA_MODE_POS)
#define CTS_CTRL_TMA_MODE_GENERALTMR             (0x1UL << CTS_CTRL_TMA_MODE_POS)
#define CTS_CTRL_TMA_MODE_CTSTMR                 (0x0UL << CTS_CTRL_TMA_MODE_POS)

/*
 * Bit definition for CTS_CTRL[15:14] - CTS Low Pass Filter selection
 */
#define CTS_CTRL_LPF_SEL_POS                     (14)
#define CTS_CTRL_LPF_SEL_MSK                     (0x3UL << CTS_CTRL_LPF_SEL_POS)
#define CTS_CTRL_LPF_SEL_500KHZ                  (0x2UL << CTS_CTRL_LPF_SEL_POS)
#define CTS_CTRL_LPF_SEL_1MHZ                    (0x1UL << CTS_CTRL_LPF_SEL_POS)
#define CTS_CTRL_LPF_SEL_DISABLE                 (0x0UL << CTS_CTRL_LPF_SEL_POS)

/*
 * Bit definition for CTS_CTRL[13:12] - CTS Charge/Discharge Current Select
 */
#define CTS_CTRL_CHARGE_CURRENT_POS              (12)
#define CTS_CTRL_CHARGE_CURRENT_MSK              (0x3UL << CTS_CTRL_CHARGE_CURRENT_POS)
#define CTS_CTRL_CHARGE_CURRENT_100uA            (0x3UL << CTS_CTRL_CHARGE_CURRENT_POS)
#define CTS_CTRL_CHARGE_CURRENT_50uA             (0x2UL << CTS_CTRL_CHARGE_CURRENT_POS)
#define CTS_CTRL_CHARGE_CURRENT_25uA             (0x1UL << CTS_CTRL_CHARGE_CURRENT_POS)
#define CTS_CTRL_CHARGE_CURRENT_0uA              (0x0UL << CTS_CTRL_CHARGE_CURRENT_POS)

/*
 * Bit definition for CTS_CTRL[2] - CTS Auto Stop bit
 */
#define CTS_CTRL_AUTOSTOP_POS                    (2)
#define CTS_CTRL_AUTOSTOP_MSK                    (0x1UL << CTS_CTRL_AUTOSTOP_POS)
#define CTS_CTRL_AUTOSTOP_ENABLE                 (0x1UL << CTS_CTRL_AUTOSTOP_POS)
#define CTS_CTRL_AUTOSTOP_DISABLE                (0x0UL << CTS_CTRL_AUTOSTOP_POS)

/*
 * Bit definition for CTS_CTRL[1] - CTS Start bit
 */
#define CTS_CTRL_START_POS                       (1)
#define CTS_CTRL_START_MSK                       (0x1UL << CTS_CTRL_START_POS)
#define CTS_CTRL_START_ENABLE                    (0x1UL << CTS_CTRL_START_POS)
#define CTS_CTRL_START_DISABLE                   (0x0UL << CTS_CTRL_START_POS)

/*
 * Bit definition for CTS_CTRL[0] - CTS enable bit
 */
#define CTS_CTRL_CTSEN_POS                       (0)
#define CTS_CTRL_CTSEN_MSK                       (0x1UL << CTS_CTRL_CTSEN_POS)
#define CTS_CTRL_CTSEN_ENABLE                    (0x1UL << CTS_CTRL_CTSEN_POS)
#define CTS_CTRL_CTSEN_DISABLE                   (0x0UL << CTS_CTRL_CTSEN_POS)

/*
 * Bit definition for CTS_STS[29] - CTS TMB Flag
 */
#define CTS_STS_TMB_INTF_POS                     (29)
#define CTS_STS_TMB_INTF_MSK                     (0x1UL << CTS_STS_TMB_INTF_POS)
#define CTS_STS_TMB_INTF_FLAG                    (0x1UL << CTS_STS_TMB_INTF_POS)

/*
 * Bit definition for CTS_STS[28] - CTS TMA Flag
 */
#define CTS_STS_TMA_INTF_POS                     (28)
#define CTS_STS_TMA_INTF_MSK                     (0x1UL << CTS_STS_TMA_INTF_POS)
#define CTS_STS_TMA_INTF_FLAG                    (0x1UL << CTS_STS_TMA_INTF_POS)

/*
 * Bit definition for PADSEL[31:0] - CTS TMA clock source select(From CTS module) when the CTS_CTRL[19:17] = 010
 */
#define CTS_PADSEL_POS                           (0)
#define CTS_PADSEL_MSK                           (0xFFFFFFFFUL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA31_ENABLE                  (0x80000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA31_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA30_ENABLE                  (0x40000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA30_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA29_ENABLE                  (0x20000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA29_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA28_ENABLE                  (0x10000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA28_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA27_ENABLE                  (0x08000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA27_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA26_ENABLE                  (0x04000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA26_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA25_ENABLE                  (0x02000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA25_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA24_ENABLE                  (0x01000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA24_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA23_ENABLE                  (0x00800000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA23_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA22_ENABLE                  (0x00400000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA22_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA21_ENABLE                  (0x00200000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA21_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA20_ENABLE                  (0x00100000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA20_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA19_ENABLE                  (0x00080000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA19_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA18_ENABLE                  (0x00040000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA18_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA17_ENABLE                  (0x00020000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA17_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA16_ENABLE                  (0x00010000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA16_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA15_ENABLE                  (0x00008000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA15_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA14_ENABLE                  (0x00004000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA14_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA13_ENABLE                  (0x00002000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA13_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA12_ENABLE                  (0x00001000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA12_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA11_ENABLE                  (0x00000800UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA11_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA10_ENABLE                  (0x00000400UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA10_DISABLE                 (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA9_ENABLE                   (0x00000200UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA9_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA8_ENABLE                   (0x00000100UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA8_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA7_ENABLE                   (0x00000080UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA7_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA6_ENABLE                   (0x00000040UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA6_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA5_ENABLE                   (0x00000020UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA5_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA4_ENABLE                   (0x00000010UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA4_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA3_ENABLE                   (0x00000008UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA3_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA2_ENABLE                   (0x00000004UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA2_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA1_ENABLE                   (0x00000002UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA1_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA0_ENABLE                   (0x00000001UL << CTS_PADSEL_POS)
#define CTS_PADSEL_IOA0_DISABLE                  (0x00000000UL << CTS_PADSEL_POS)

/*
 * Bit definition for PADSEL[5:0] - CTS TMA clock source select(From IO PAD) if CTS_CTRL[19:17] = 001
 */
#define CTS_PADSEL_TMA_CLKSRC_POS                (0)
#define CTS_PADSEL_TMA_CLKSRC_MSK                (0x3FUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA0          (0x00UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA1          (0x01UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA2          (0x02UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA3          (0x03UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA4          (0x04UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA5          (0x05UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA6          (0x06UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA7          (0x07UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA8          (0x08UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA9          (0x09UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA10         (0x0AUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA11         (0x0BUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA12         (0x0CUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA13         (0x0DUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA14         (0x0EUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA15         (0x0FUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA16         (0x10UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA17         (0x11UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA18         (0x12UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA19         (0x13UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA20         (0x14UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA21         (0x15UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA22         (0x16UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA23         (0x17UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA24         (0x18UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA25         (0x19UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA26         (0x1AUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA27         (0x1BUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA28         (0x1CUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA29         (0x1DUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA30         (0x1EUL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOA31         (0x1FUL << CTS_PADSEL_TMA_CLKSRC_POS)

#define CTS_PADSEL_TMA_CLKSRC_FROM_IOB0          (0x20UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOB1          (0x21UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOB2          (0x22UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOB3          (0x23UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOB4          (0x24UL << CTS_PADSEL_TMA_CLKSRC_POS)
#define CTS_PADSEL_TMA_CLKSRC_FROM_IOB5          (0x25UL << CTS_PADSEL_TMA_CLKSRC_POS)


/*---------------------------------------------------------------------------------------
 * TimeBase Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;                          // TimeBase Control Register
    __IO uint32_t STS;                           // TimeBase Status Register
} TIMEBASE_TYPE_DEF;

/*
 * Bit definition for TIMEBASE_CTRL[17] - TimeBase clear bit
 */
#define TIMEBASE_CTRL_CLRCNT_POS                 (17)
#define TIMEBASE_CTRL_CLRCNT_MSK                 (0x1UL << TIMEBASE_CTRL_CLRCNT_POS)
#define TIMEBASE_CTRL_CLRCNT_ENABLE              (0x1UL << TIMEBASE_CTRL_CLRCNT_POS)
#define TIMEBASE_CTRL_CLRCNT_DISABLE             (0x0UL << TIMEBASE_CTRL_CLRCNT_POS)

/*
 * Bit definition for TIMEBASE_CTRL[16] - TimeBase enable bit
 */
#define TIMEBASE_CTRL_TB_EN_POS                  (16)
#define TIMEBASE_CTRL_TB_EN_MSK                  (0x1UL << TIMEBASE_CTRL_TB_EN_POS)
#define TIMEBASE_CTRL_TB_ENABLE                  (0x1UL << TIMEBASE_CTRL_TB_EN_POS)
#define TIMEBASE_CTRL_TB_DISABLE                 (0x0UL << TIMEBASE_CTRL_TB_EN_POS)

/*
 * Bit definition for TIMEBASE_CTRL[9] - 4KHz INT enable bit
 */
#define TIMEBASE_CTRL_4KHZ_POS                   (9)
#define TIMEBASE_CTRL_4KHZ_MSK                   (0x1UL << TIMEBASE_CTRL_4KHZ_POS)
#define TIMEBASE_CTRL_4KHZ_ENABLE                (0x1UL << TIMEBASE_CTRL_4KHZ_POS)
#define TIMEBASE_CTRL_4KHZ_DISABLE               (0x0UL << TIMEBASE_CTRL_4KHZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[8] - 2KHz INT enable bit
 */
#define TIMEBASE_CTRL_2KHZ_POS                   (8)
#define TIMEBASE_CTRL_2KHZ_MSK                   (0x1UL << TIMEBASE_CTRL_2KHZ_POS)
#define TIMEBASE_CTRL_2KHZ_ENABLE                (0x1UL << TIMEBASE_CTRL_2KHZ_POS)
#define TIMEBASE_CTRL_2KHZ_DISABLE               (0x0UL << TIMEBASE_CTRL_2KHZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[7] - 512Hz INT enable bit
 */
#define TIMEBASE_CTRL_512HZ_POS                  (7)
#define TIMEBASE_CTRL_512HZ_MSK                  (0x1UL << TIMEBASE_CTRL_512HZ_POS)
#define TIMEBASE_CTRL_512HZ_ENABLE               (0x1UL << TIMEBASE_CTRL_512HZ_POS)
#define TIMEBASE_CTRL_512HZ_DISABLE              (0x0UL << TIMEBASE_CTRL_512HZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[6] - 64Hz INT enable bit
 */
#define TIMEBASE_CTRL_64HZ_POS                   (6)
#define TIMEBASE_CTRL_64HZ_MSK                   (0x1UL << TIMEBASE_CTRL_64HZ_POS)
#define TIMEBASE_CTRL_64HZ_ENABLE                (0x1UL << TIMEBASE_CTRL_64HZ_POS)
#define TIMEBASE_CTRL_64HZ_DISABLE               (0x0UL << TIMEBASE_CTRL_64HZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[5] - 16Hz INT enable bit
 */
#define TIMEBASE_CTRL_16HZ_POS                   (5)
#define TIMEBASE_CTRL_16HZ_MSK                   (0x1UL << TIMEBASE_CTRL_16HZ_POS)
#define TIMEBASE_CTRL_16HZ_ENABLE                (0x1UL << TIMEBASE_CTRL_16HZ_POS)
#define TIMEBASE_CTRL_16HZ_DISABLE               (0x0UL << TIMEBASE_CTRL_16HZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[4] - 2Hz INT enable bit
 */
#define TIMEBASE_CTRL_2HZ_POS                    (4)
#define TIMEBASE_CTRL_2HZ_MSK                    (0x1UL << TIMEBASE_CTRL_2HZ_POS)
#define TIMEBASE_CTRL_2HZ_ENABLE                 (0x1UL << TIMEBASE_CTRL_2HZ_POS)
#define TIMEBASE_CTRL_2HZ_DISABLE                (0x0UL << TIMEBASE_CTRL_2HZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[3] - 1Hz INT enable bit
 */
#define TIMEBASE_CTRL_1HZ_POS                    (3)
#define TIMEBASE_CTRL_1HZ_MSK                    (0x1UL << TIMEBASE_CTRL_1HZ_POS)
#define TIMEBASE_CTRL_1HZ_ENABLE                 (0x1UL << TIMEBASE_CTRL_1HZ_POS)
#define TIMEBASE_CTRL_1HZ_DISABLE                (0x0UL << TIMEBASE_CTRL_1HZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[2] - 0.5Hz INT enable bit
 */
#define TIMEBASE_CTRL_0D5HZ_POS                  (2)
#define TIMEBASE_CTRL_0D5HZ_MSK                  (0x1UL << TIMEBASE_CTRL_0D5HZ_POS)
#define TIMEBASE_CTRL_0D5HZ_ENABLE               (0x1UL << TIMEBASE_CTRL_0D5HZ_POS)
#define TIMEBASE_CTRL_0D5HZ_DISABLE              (0x0UL << TIMEBASE_CTRL_0D5HZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[1] - 0.25Hz INT enable bit
 */
#define TIMEBASE_CTRL_0D25HZ_POS                 (1)
#define TIMEBASE_CTRL_0D25HZ_MSK                 (0x1UL << TIMEBASE_CTRL_0D25HZ_POS)
#define TIMEBASE_CTRL_0D25HZ_ENABLE              (0x1UL << TIMEBASE_CTRL_0D25HZ_POS)
#define TIMEBASE_CTRL_0D25HZ_DISABLE             (0x0UL << TIMEBASE_CTRL_0D25HZ_POS)

/*
 * Bit definition for TIMEBASE_CTRL[0] - 0.125Hz INT enable bit
 */
#define TIMEBASE_CTRL_0D125HZ_POS                (0)
#define TIMEBASE_CTRL_0D125HZ_MSK                (0x1UL << TIMEBASE_CTRL_0D125HZ_POS)
#define TIMEBASE_CTRL_0D125HZ_ENABLE             (0x1UL << TIMEBASE_CTRL_0D125HZ_POS)
#define TIMEBASE_CTRL_0D125HZ_DISABLE            (0x0UL << TIMEBASE_CTRL_0D125HZ_POS)

/*
 * Bit definition for TIMEBASE_STS[9] - 4KHz INT Flag
 */
#define TIMEBASE_STS_4KHZ_INTF_POS               (9)
#define TIMEBASE_STS_4KHZ_INTF_MSK               (0x1UL << TIMEBASE_STS_4KHZ_INTF_POS)
#define TIMEBASE_STS_4KHZ_INTF_FLAG              (0x1UL << TIMEBASE_STS_4KHZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[8] - 2KHz INT Flag
 */
#define TIMEBASE_STS_2KHZ_INTF_POS               (8)
#define TIMEBASE_STS_2KHZ_INTF_MSK               (0x1UL << TIMEBASE_STS_2KHZ_INTF_POS)
#define TIMEBASE_STS_2KHZ_INTF_FLAG              (0x1UL << TIMEBASE_STS_2KHZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[7] - 512Hz INT Flag
 */
#define TIMEBASE_STS_512HZ_INTF_POS              (7)
#define TIMEBASE_STS_512HZ_INTF_MSK              (0x1UL << TIMEBASE_STS_512HZ_INTF_POS)
#define TIMEBASE_STS_512HZ_INTF_FLAG             (0x1UL << TIMEBASE_STS_512HZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[6] - 64Hz INT Flag
 */
#define TIMEBASE_STS_64HZ_INTF_POS               (6)
#define TIMEBASE_STS_64HZ_INTF_MSK               (0x1UL << TIMEBASE_STS_64HZ_INTF_POS)
#define TIMEBASE_STS_64HZ_INTF_FLAG              (0x1UL << TIMEBASE_STS_64HZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[5] - 16Hz INT Flag
 */
#define TIMEBASE_STS_16HZ_INTF_POS               (5)
#define TIMEBASE_STS_16HZ_INTF_MSK               (0x1UL << TIMEBASE_STS_16HZ_INTF_POS)
#define TIMEBASE_STS_16HZ_INTF_FLAG              (0x1UL << TIMEBASE_STS_16HZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[4] - 2Hz INT Flag
 */
#define TIMEBASE_STS_2HZ_INTF_POS                (4)
#define TIMEBASE_STS_2HZ_INTF_MSK                (0x1UL << TIMEBASE_STS_2HZ_INTF_POS)
#define TIMEBASE_STS_2HZ_INTF_FLAG               (0x1UL << TIMEBASE_STS_2HZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[3] - 1Hz INT Flag
 */
#define TIMEBASE_STS_1HZ_INTF_POS                (3)
#define TIMEBASE_STS_1HZ_INTF_MSK                (0x1UL << TIMEBASE_STS_1HZ_INTF_POS)
#define TIMEBASE_STS_1HZ_INTF_FLAG               (0x1UL << TIMEBASE_STS_1HZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[2] - 0.5Hz INT Flag
 */
#define TIMEBASE_STS_0D5HZ_INTF_POS              (2)
#define TIMEBASE_STS_0D5HZ_INTF_MSK              (0x1UL << TIMEBASE_STS_0D5HZ_INTF_POS)
#define TIMEBASE_STS_0D5HZ_INTF_FLAG             (0x1UL << TIMEBASE_STS_0D5HZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[1] - 0.25Hz INT Flag
 */
#define TIMEBASE_STS_0D25HZ_INTF_POS             (1)
#define TIMEBASE_STS_0D25HZ_INTF_MSK             (0x1UL << TIMEBASE_STS_0D25HZ_INTF_POS)
#define TIMEBASE_STS_0D25HZ_INTF_FLAG            (0x1UL << TIMEBASE_STS_0D25HZ_INTF_POS)

/*
 * Bit definition for TIMEBASE_STS[0] - 0.125Hz INT Flag
 */
#define TIMEBASE_STS_0D125HZ_INTF_POS            (0)
#define TIMEBASE_STS_0D125HZ_INTF_MSK            (0x1UL << TIMEBASE_STS_0D125HZ_INTF_POS)
#define TIMEBASE_STS_0D125HZ_INTF_FLAG           (0x1UL << TIMEBASE_STS_0D125HZ_INTF_POS)


/*---------------------------------------------------------------------------------------
 * SAR ADC Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t GCTRL;                         // SAR ADC Global Control Register
    __IO uint32_t CTRL;                          // SAR ADC Control Register
    __IO uint32_t STS;                           // SAR ADC Status Register
    __IO uint32_t SMP0;                          // SAR ADC Sample Time Control Register 0
    __IO uint32_t SMP1;                          // SAR ADC Sample Time Control Register 1
    __IO uint32_t REG_COV;                       // SAR ADC Regular Mode Conversion Channel
    __IO uint32_t REG_SEQ;                       // SAR ADC Regular Sequence Register
    __IO uint32_t INJ_SEQ;                       // SAR ADC Injected Sequence Register
    __IO uint32_t WDG_TH;                        // SAR ADC Watch-Dog Threshold Register
    __IO uint32_t REG_DATA;                      // SAR ADC Regular Data Register
    __IO uint32_t INJ0_DATA;                     // SAR ADC Injected0 Data Register
    __IO uint32_t INJ1_DATA;                     // SAR ADC Injected1 Data Register
    __IO uint32_t INJ2_DATA;                     // SAR ADC Injected2 Data Register
    __IO uint32_t INJ3_DATA;                     // SAR ADC Injected3 Data Register
} SAR_ADC_TYPE_DEF;

/*
 * Bit definition for SAR_ADC_GCTRL[30] - Analog watch-dog interrupt enable on injected channel
 */
#define SAR_ADC_GCTRL_ANALOG_WDGJ_INT_EN_POS     (30)
#define SAR_ADC_GCTRL_ANALOG_WDGJ_INT_EN_MSK     (0x1UL << SAR_ADC_GCTRL_ANALOG_WDGJ_INT_EN_POS)
#define SAR_ADC_GCTRL_ANALOG_WDGJ_INT_ENABLE     (0x1UL << SAR_ADC_GCTRL_ANALOG_WDGJ_INT_EN_POS)
#define SAR_ADC_GCTRL_ANALOG_WDGJ_INT_DISABLE    (0x0UL << SAR_ADC_GCTRL_ANALOG_WDGJ_INT_EN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[29] - Analog watch-dog interrupt enable on regular channel
 */
#define SAR_ADC_GCTRL_ANALOG_WDGR_INT_EN_POS     (29)
#define SAR_ADC_GCTRL_ANALOG_WDGR_INT_EN_MSK     (0x1UL << SAR_ADC_GCTRL_ANALOG_WDGR_INT_EN_POS)
#define SAR_ADC_GCTRL_ANALOG_WDGR_INT_ENABLE     (0x1UL << SAR_ADC_GCTRL_ANALOG_WDGR_INT_EN_POS)
#define SAR_ADC_GCTRL_ANALOG_WDGR_INT_DISABLE    (0x0UL << SAR_ADC_GCTRL_ANALOG_WDGR_INT_EN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[28] - Injected conversion interrupt enable of channel 3
 */
#define SAR_ADC_GCTRL_INJ3_INT_EN_POS            (28)
#define SAR_ADC_GCTRL_INJ3_INT_EN_MSK            (0x1UL << SAR_ADC_GCTRL_INJ3_INT_EN_POS)
#define SAR_ADC_GCTRL_INJ3_INT_ENABLE            (0x1UL << SAR_ADC_GCTRL_INJ3_INT_EN_POS)
#define SAR_ADC_GCTRL_INJ3_INT_DISABLE           (0x0UL << SAR_ADC_GCTRL_INJ3_INT_EN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[27] - Injected conversion interrupt enable of channel 2
 */
#define SAR_ADC_GCTRL_INJ2_INT_EN_POS            (27)
#define SAR_ADC_GCTRL_INJ2_INT_EN_MSK            (0x1UL << SAR_ADC_GCTRL_INJ2_INT_EN_POS)
#define SAR_ADC_GCTRL_INJ2_INT_ENABLE            (0x1UL << SAR_ADC_GCTRL_INJ2_INT_EN_POS)
#define SAR_ADC_GCTRL_INJ2_INT_DISABLE           (0x0UL << SAR_ADC_GCTRL_INJ2_INT_EN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[26] - Injected conversion interrupt enable of channel 1
 */
#define SAR_ADC_GCTRL_INJ1_INT_EN_POS            (26)
#define SAR_ADC_GCTRL_INJ1_INT_EN_MSK            (0x1UL << SAR_ADC_GCTRL_INJ1_INT_EN_POS)
#define SAR_ADC_GCTRL_INJ1_INT_ENABLE            (0x1UL << SAR_ADC_GCTRL_INJ1_INT_EN_POS)
#define SAR_ADC_GCTRL_INJ1_INT_DISABLE           (0x0UL << SAR_ADC_GCTRL_INJ1_INT_EN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[25] - Injected conversion interrupt enable of channel 0
 */
#define SAR_ADC_GCTRL_INJ0_INT_EN_POS            (25)
#define SAR_ADC_GCTRL_INJ0_INT_EN_MSK            (0x1UL << SAR_ADC_GCTRL_INJ0_INT_EN_POS)
#define SAR_ADC_GCTRL_INJ0_INT_ENABLE            (0x1UL << SAR_ADC_GCTRL_INJ0_INT_EN_POS)
#define SAR_ADC_GCTRL_INJ0_INT_DISABLE           (0x0UL << SAR_ADC_GCTRL_INJ0_INT_EN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[24] - Regular mode interrupt enable
 */
#define SAR_ADC_GCTRL_REG_MODE_INT_EN_POS        (24)
#define SAR_ADC_GCTRL_REG_MODE_INT_EN_MSK        (0x1UL << SAR_ADC_GCTRL_REG_MODE_INT_EN_POS)
#define SAR_ADC_GCTRL_REG_MODE_INT_ENABLE        (0x1UL << SAR_ADC_GCTRL_REG_MODE_INT_EN_POS)
#define SAR_ADC_GCTRL_REG_MODE_INT_DISABLE       (0x0UL << SAR_ADC_GCTRL_REG_MODE_INT_EN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[12:8] - ADC clock select bits
 */
#define SAR_ADC_GCTRL_ADC_CLK_SEL_POS            (8)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_MSK            (0x1FUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV64     (0x1FUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV62     (0x1EUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV60     (0x1DUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV58     (0x1CUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV56     (0x1BUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV54     (0x1AUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV52     (0x19UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV50     (0x18UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV48     (0x17UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV46     (0x16UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV44     (0x15UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV42     (0x14UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV40     (0x13UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV38     (0x12UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV36     (0x11UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV34     (0x10UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV32     (0x0FUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV30     (0x0EUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV28     (0x0DUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV26     (0x0CUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV24     (0x0BUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV22     (0x0AUL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV20     (0x09UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV18     (0x08UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV16     (0x07UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV14     (0x06UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV12     (0x05UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV10     (0x04UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV8      (0x03UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV6      (0x02UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV4      (0x01UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)
#define SAR_ADC_GCTRL_ADC_CLK_SEL_PCLK_DIV2      (0x00UL << SAR_ADC_GCTRL_ADC_CLK_SEL_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[4] - ADC DMA request enable
 */
#define SAR_ADC_GCTRL_DMA_EN_POS                 (4)
#define SAR_ADC_GCTRL_DMA_EN_MSK                 (0x1UL << SAR_ADC_GCTRL_DMA_EN_POS)
#define SAR_ADC_GCTRL_DMA_ENABLE                 (0x1UL << SAR_ADC_GCTRL_DMA_EN_POS)
#define SAR_ADC_GCTRL_DMA_DISABLE                (0x0UL << SAR_ADC_GCTRL_DMA_EN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[2] - ADC output data alignment
 */
#define SAR_ADC_GCTRL_DAT_ALIGN_POS              (2)
#define SAR_ADC_GCTRL_DAT_ALIGN_MSK              (0x1UL << SAR_ADC_GCTRL_DAT_ALIGN_POS)
#define SAR_ADC_GCTRL_DAT_ALIGN_RIGHT            (0x1UL << SAR_ADC_GCTRL_DAT_ALIGN_POS)
#define SAR_ADC_GCTRL_DAT_ALIGN_LEFT             (0x0UL << SAR_ADC_GCTRL_DAT_ALIGN_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[1] - ADC data signed/unsigned select
 */
#define SAR_ADC_GCTRL_ADC_FMT_SEL_POS            (1)
#define SAR_ADC_GCTRL_ADC_FMT_SEL_MSK            (0x1UL << SAR_ADC_GCTRL_ADC_FMT_SEL_POS)
#define SAR_ADC_GCTRL_ADC_FMT_SEL_SIGN           (0x1UL << SAR_ADC_GCTRL_ADC_FMT_SEL_POS)
#define SAR_ADC_GCTRL_ADC_FMT_SEL_UNSIGN         (0x0UL << SAR_ADC_GCTRL_ADC_FMT_SEL_POS)

/*
 * Bit definition for SAR_ADC_GCTRL[0] - ADC analog block enable
 */
#define SAR_ADC_GCTRL_ADC_EN_POS                 (0)
#define SAR_ADC_GCTRL_ADC_EN_MSK                 (0x1UL << SAR_ADC_GCTRL_ADC_EN_POS)
#define SAR_ADC_GCTRL_ADC_ENABLE                 (0x1UL << SAR_ADC_GCTRL_ADC_EN_POS)
#define SAR_ADC_GCTRL_ADC_DISABLE                (0x0UL << SAR_ADC_GCTRL_ADC_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[31:29] - Injection channel 3 trigger source select bits
 */
#define SAR_ADC_CTRL_INJ3_TRG_SEL_POS            (29)
#define SAR_ADC_CTRL_INJ3_TRG_SEL_MSK            (0x7UL << SAR_ADC_CTRL_INJ3_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ3_TRG_SEL_CTS_TM1        (0x6UL << SAR_ADC_CTRL_INJ3_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ3_TRG_SEL_CTS_TM0        (0x5UL << SAR_ADC_CTRL_INJ3_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ3_TRG_SEL_CCP1_TM        (0x4UL << SAR_ADC_CTRL_INJ3_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ3_TRG_SEL_CCP0_TM        (0x3UL << SAR_ADC_CTRL_INJ3_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ3_TRG_SEL_TM2            (0x2UL << SAR_ADC_CTRL_INJ3_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ3_TRG_SEL_TM1            (0x1UL << SAR_ADC_CTRL_INJ3_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ3_TRG_SEL_TM0            (0x0UL << SAR_ADC_CTRL_INJ3_TRG_SEL_POS)

/*
 * Bit definition for SAR_ADC_CTRL[28] - Injection channel 3 enable bit
 */
#define SAR_ADC_CTRL_INJ3_EN_POS                 (28)
#define SAR_ADC_CTRL_INJ3_EN_MSK                 (0x1UL << SAR_ADC_CTRL_INJ3_EN_POS)
#define SAR_ADC_CTRL_INJ3_ENABLE                 (0x1UL << SAR_ADC_CTRL_INJ3_EN_POS)
#define SAR_ADC_CTRL_INJ3_DISABLE                (0x0UL << SAR_ADC_CTRL_INJ3_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[27:25] - Injection channel 2 trigger source select bits
 */
#define SAR_ADC_CTRL_INJ2_TRG_SEL_POS            (25)
#define SAR_ADC_CTRL_INJ2_TRG_SEL_MSK            (0x7UL << SAR_ADC_CTRL_INJ2_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ2_TRG_SEL_CTS_TM1        (0x6UL << SAR_ADC_CTRL_INJ2_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ2_TRG_SEL_CTS_TM0        (0x5UL << SAR_ADC_CTRL_INJ2_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ2_TRG_SEL_CCP1_TM        (0x4UL << SAR_ADC_CTRL_INJ2_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ2_TRG_SEL_CCP0_TM        (0x3UL << SAR_ADC_CTRL_INJ2_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ2_TRG_SEL_TM2            (0x2UL << SAR_ADC_CTRL_INJ2_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ2_TRG_SEL_TM1            (0x1UL << SAR_ADC_CTRL_INJ2_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ2_TRG_SEL_TM0            (0x0UL << SAR_ADC_CTRL_INJ2_TRG_SEL_POS)

/*
 * Bit definition for SAR_ADC_CTRL[24] - Injection channel 2 enable bit
 */
#define SAR_ADC_CTRL_INJ2_EN_POS                 (24)
#define SAR_ADC_CTRL_INJ2_EN_MSK                 (0x1UL << SAR_ADC_CTRL_INJ2_EN_POS)
#define SAR_ADC_CTRL_INJ2_ENABLE                 (0x1UL << SAR_ADC_CTRL_INJ2_EN_POS)
#define SAR_ADC_CTRL_INJ2_DISABLE                (0x0UL << SAR_ADC_CTRL_INJ2_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[23:21] - Injection channel 1 trigger source select bits
 */
#define SAR_ADC_CTRL_INJ1_TRG_SEL_POS            (21)
#define SAR_ADC_CTRL_INJ1_TRG_SEL_MSK            (0x7UL << SAR_ADC_CTRL_INJ1_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ1_TRG_SEL_CTS_TM1        (0x6UL << SAR_ADC_CTRL_INJ1_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ1_TRG_SEL_CTS_TM0        (0x5UL << SAR_ADC_CTRL_INJ1_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ1_TRG_SEL_CCP1_TM        (0x4UL << SAR_ADC_CTRL_INJ1_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ1_TRG_SEL_CCP0_TM        (0x3UL << SAR_ADC_CTRL_INJ1_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ1_TRG_SEL_TM2            (0x2UL << SAR_ADC_CTRL_INJ1_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ1_TRG_SEL_TM1            (0x1UL << SAR_ADC_CTRL_INJ1_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ1_TRG_SEL_TM0            (0x0UL << SAR_ADC_CTRL_INJ1_TRG_SEL_POS)

/*
 * Bit definition for SAR_ADC_CTRL[20] - Injection channel 1 enable bit
 */
#define SAR_ADC_CTRL_INJ1_EN_POS                 (20)
#define SAR_ADC_CTRL_INJ1_EN_MSK                 (0x1UL << SAR_ADC_CTRL_INJ1_EN_POS)
#define SAR_ADC_CTRL_INJ1_ENABLE                 (0x1UL << SAR_ADC_CTRL_INJ1_EN_POS)
#define SAR_ADC_CTRL_INJ1_DISABLE                (0x0UL << SAR_ADC_CTRL_INJ1_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[19:17] - Injection channel 0 trigger source select bits
 */
#define SAR_ADC_CTRL_INJ0_TRG_SEL_POS            (17)
#define SAR_ADC_CTRL_INJ0_TRG_SEL_MSK            (0x7UL << SAR_ADC_CTRL_INJ0_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ0_TRG_SEL_CTS_TM1        (0x6UL << SAR_ADC_CTRL_INJ0_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ0_TRG_SEL_CTS_TM0        (0x5UL << SAR_ADC_CTRL_INJ0_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ0_TRG_SEL_CCP1_TM        (0x4UL << SAR_ADC_CTRL_INJ0_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ0_TRG_SEL_CCP0_TM        (0x3UL << SAR_ADC_CTRL_INJ0_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ0_TRG_SEL_TM2            (0x2UL << SAR_ADC_CTRL_INJ0_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ0_TRG_SEL_TM1            (0x1UL << SAR_ADC_CTRL_INJ0_TRG_SEL_POS)
#define SAR_ADC_CTRL_INJ0_TRG_SEL_TM0            (0x0UL << SAR_ADC_CTRL_INJ0_TRG_SEL_POS)

/*
 * Bit definition for SAR_ADC_CTRL[16] - Injection channel 0 enable bit
 */
#define SAR_ADC_CTRL_INJ0_EN_POS                 (16)
#define SAR_ADC_CTRL_INJ0_EN_MSK                 (0x1UL << SAR_ADC_CTRL_INJ0_EN_POS)
#define SAR_ADC_CTRL_INJ0_ENABLE                 (0x1UL << SAR_ADC_CTRL_INJ0_EN_POS)
#define SAR_ADC_CTRL_INJ0_DISABLE                (0x0UL << SAR_ADC_CTRL_INJ0_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[15] - Analog watch-dog enable on injected channel
 */
#define SAR_ADC_CTRL_ANALOG_WDGJ_EN_POS          (15)
#define SAR_ADC_CTRL_ANALOG_WDGJ_EN_MSK          (0x1UL << SAR_ADC_CTRL_ANALOG_WDGJ_EN_POS)
#define SAR_ADC_CTRL_ANALOG_WDGJ_ENABLE          (0x1UL << SAR_ADC_CTRL_ANALOG_WDGJ_EN_POS)
#define SAR_ADC_CTRL_ANALOG_WDGJ_DISABLE         (0x0UL << SAR_ADC_CTRL_ANALOG_WDGJ_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[11:8] - Regular sequence gap select bits
 */
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS         (8)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_MSK         (0xFUL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_MANUAL      (0xFUL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_CTS_TM1     (0xEUL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_CTS_TM0     (0xDUL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_CCP1_TM     (0xCUL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_CCP0_TM     (0xBUL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_TM2         (0xAUL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_TM1         (0x9UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_TM0         (0x8UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_7ADCCLK     (0x7UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_6ADCCLK     (0x6UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_5ADCCLK     (0x5UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_4ADCCLK     (0x4UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_3ADCCLK     (0x3UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_2ADCCLK     (0x2UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_1ADCCLK     (0x1UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)
#define SAR_ADC_CTRL_REG_SEQ_GAP_SEL_0ADCCLK     (0x0UL << SAR_ADC_CTRL_REG_SEQ_GAP_SEL_POS)

/*
 * Bit definition for SAR_ADC_CTRL[7:4] - Channel number during regular sequence
 */
#define SAR_ADC_CTRL_REG_CH_NUM_POS              (4)
#define SAR_ADC_CTRL_REG_CH_NUM_MSK              (0xFUL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_VDDIOD4                 (0x9UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_VDD12                   (0x8UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_CH7                     (0x7UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_CH6                     (0x6UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_CH5                     (0x5UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_CH4                     (0x4UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_CH3                     (0x3UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_CH2                     (0x2UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_CH1                     (0x1UL << SAR_ADC_CTRL_REG_CH_NUM_POS)
#define SAR_ADC_CTRL_REG_CH0                     (0x0UL << SAR_ADC_CTRL_REG_CH_NUM_POS)

/*
 * Bit definition for SAR_ADC_CTRL[3] - Analog watch-dog enable on regular channel
 */
#define SAR_ADC_CTRL_ANALOG_WDGR_EN_POS          (3)
#define SAR_ADC_CTRL_ANALOG_WDGR_EN_MSK          (0x1UL << SAR_ADC_CTRL_ANALOG_WDGR_EN_POS)
#define SAR_ADC_CTRL_ANALOG_WDGR_ENABLE          (0x1UL << SAR_ADC_CTRL_ANALOG_WDGR_EN_POS)
#define SAR_ADC_CTRL_ANALOG_WDGR_DISABLE         (0x0UL << SAR_ADC_CTRL_ANALOG_WDGR_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[2] - ADC loop scan enable bit
 */
#define SAR_ADC_CTRL_LOOP_EN_POS                 (2)
#define SAR_ADC_CTRL_LOOP_EN_MSK                 (0x1UL << SAR_ADC_CTRL_LOOP_EN_POS)
#define SAR_ADC_CTRL_LOOP_ENABLE                 (0x1UL << SAR_ADC_CTRL_LOOP_EN_POS)
#define SAR_ADC_CTRL_LOOP_DISABLE                (0x0UL << SAR_ADC_CTRL_LOOP_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[1] - Regular mode enable bit
 */
#define SAR_ADC_CTRL_REG_EN_POS                  (1)
#define SAR_ADC_CTRL_REG_EN_MSK                  (0x1UL << SAR_ADC_CTRL_REG_EN_POS)
#define SAR_ADC_CTRL_REG_ENABLE                  (0x1UL << SAR_ADC_CTRL_REG_EN_POS)
#define SAR_ADC_CTRL_REG_DISABLE                 (0x0UL << SAR_ADC_CTRL_REG_EN_POS)

/*
 * Bit definition for SAR_ADC_CTRL[0] - Software start bit
 */
#define SAR_ADC_CTRL_SFT_STR_POS                 (0)
#define SAR_ADC_CTRL_SFT_STR_MSK                 (0x1UL << SAR_ADC_CTRL_SFT_STR_POS)
#define SAR_ADC_CTRL_SFT_START                   (0x1UL << SAR_ADC_CTRL_SFT_STR_POS)

/*
 * Bit definition for SAR_ADC_STS[30] - Analog watch-dog interrupt flag on injected channel
 */
#define SAR_ADC_STS_ANALOG_WDGJ_INTF_POS         (30)
#define SAR_ADC_STS_ANALOG_WDGJ_INTF_MSK         (0x1UL << SAR_ADC_STS_ANALOG_WDGJ_INTF_POS)
#define SAR_ADC_STS_ANALOG_WDGJ_INT_FLAG         (0x1UL << SAR_ADC_STS_ANALOG_WDGJ_INTF_POS)

/*
 * Bit definition for SAR_ADC_STS[29] - Analog watch-dog interrupt flag on regular channel
 */
#define SAR_ADC_STS_ANALOG_WDGR_INTF_POS         (29)
#define SAR_ADC_STS_ANALOG_WDGR_INTF_MSK         (0x1UL << SAR_ADC_STS_ANALOG_WDGR_INTF_POS)
#define SAR_ADC_STS_ANALOG_WDGR_INT_FLAG         (0x1UL << SAR_ADC_STS_ANALOG_WDGR_INTF_POS)

/*
 * Bit definition for SAR_ADC_STS[28] - Injected conversion interrupt flag of channel 3
 */
#define SAR_ADC_STS_INJ3_INTF_POS                (28)
#define SAR_ADC_STS_INJ3_INTF_MSK                (0x1UL << SAR_ADC_STS_INJ3_INTF_POS)
#define SAR_ADC_STS_INJ3_INT_FLAG                (0x1UL << SAR_ADC_STS_INJ3_INTF_POS)

/*
 * Bit definition for SAR_ADC_STS[27] - Injected conversion interrupt flag of channel 2
 */
#define SAR_ADC_STS_INJ2_INTF_POS                (27)
#define SAR_ADC_STS_INJ2_INTF_MSK                (0x1UL << SAR_ADC_STS_INJ2_INTF_POS)
#define SAR_ADC_STS_INJ2_INT_FLAG                (0x1UL << SAR_ADC_STS_INJ2_INTF_POS)

/*
 * Bit definition for SAR_ADC_STS[26] - Injected conversion interrupt flag of channel 1
 */
#define SAR_ADC_STS_INJ1_INTF_POS                (26)
#define SAR_ADC_STS_INJ1_INTF_MSK                (0x1UL << SAR_ADC_STS_INJ1_INTF_POS)
#define SAR_ADC_STS_INJ1_INT_FLAG                (0x1UL << SAR_ADC_STS_INJ1_INTF_POS)

/*
 * Bit definition for SAR_ADC_STS[25] - Injected conversion interrupt flag of channel 0
 */
#define SAR_ADC_STS_INJ0_INTF_POS                (25)
#define SAR_ADC_STS_INJ0_INTF_MSK                (0x1UL << SAR_ADC_STS_INJ0_INTF_POS)
#define SAR_ADC_STS_INJ0_INT_FLAG                (0x1UL << SAR_ADC_STS_INJ0_INTF_POS)

/*
 * Bit definition for SAR_ADC_STS[24] - Regular conversion interrupt flag
 */
#define SAR_ADC_STS_REG_MODE_INTF_POS            (24)
#define SAR_ADC_STS_REG_MODE_INTF_MSK            (0x1UL << SAR_ADC_STS_REG_MODE_INTF_POS)
#define SAR_ADC_STS_REG_MODE_INT_FLAG            (0x1UL << SAR_ADC_STS_REG_MODE_INTF_POS)

/*
 * Bit definition for SAR_ADC_STS[0] - ADC ready flag
 */
#define SAR_ADC_STS_ADC_RDY_POS                  (0)
#define SAR_ADC_STS_ADC_RDY_MSK                  (0x1UL << SAR_ADC_STS_ADC_RDY_POS)
#define SAR_ADC_STS_ADC_RDY_FLAG                 (0x1UL << SAR_ADC_STS_ADC_RDY_POS)

/*
 * Bit definition for SAR_ADC_SMP0[30:28] - Channel 7 sample time selection
 */
#define SAR_ADC_SMP0_CH7_SMP_SEL_POS             (28)
#define SAR_ADC_SMP0_CH7_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH7_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH7_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH7_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH7_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH7_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH7_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH7_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH7_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP0_CH7_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP0[26:24] - Channel 6 sample time selection
 */
#define SAR_ADC_SMP0_CH6_SMP_SEL_POS             (24)
#define SAR_ADC_SMP0_CH6_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH6_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH6_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH6_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH6_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH6_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH6_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH6_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH6_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP0_CH6_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP0[22:20] - Channel 5 sample time selection
 */
#define SAR_ADC_SMP0_CH5_SMP_SEL_POS             (20)
#define SAR_ADC_SMP0_CH5_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH5_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH5_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH5_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH5_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH5_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH5_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH5_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH5_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP0_CH5_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP0[18:16] - Channel 4 sample time selection
 */
#define SAR_ADC_SMP0_CH4_SMP_SEL_POS             (16)
#define SAR_ADC_SMP0_CH4_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH4_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH4_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH4_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH4_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH4_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH4_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH4_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH4_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP0_CH4_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP0[14:12] - Channel 3 sample time selection
 */
#define SAR_ADC_SMP0_CH3_SMP_SEL_POS             (12)
#define SAR_ADC_SMP0_CH3_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH3_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH3_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH3_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH3_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH3_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH3_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH3_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH3_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP0_CH3_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP0[10:8] - Channel 2 sample time selection
 */
#define SAR_ADC_SMP0_CH2_SMP_SEL_POS             (8)
#define SAR_ADC_SMP0_CH2_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH2_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH2_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH2_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH2_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH2_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH2_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH2_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH2_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP0_CH2_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP0[6:4] - Channel 1 sample time selection
 */
#define SAR_ADC_SMP0_CH1_SMP_SEL_POS             (4)
#define SAR_ADC_SMP0_CH1_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH1_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH1_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH1_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH1_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH1_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH1_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH1_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH1_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP0_CH1_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP0[2:0] - Channel 0 sample time selection
 */
#define SAR_ADC_SMP0_CH0_SMP_SEL_POS             (0)
#define SAR_ADC_SMP0_CH0_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH0_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH0_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH0_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH0_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH0_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH0_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH0_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)
#define SAR_ADC_SMP0_CH0_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP0_CH0_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP1[6:4] - Channel 9 sample time selection
 */
#define SAR_ADC_SMP1_CH9_SMP_SEL_POS             (4)
#define SAR_ADC_SMP1_CH9_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH9_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH9_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH9_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH9_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH9_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH9_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH9_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH9_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP1_CH9_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_SMP1[2:0] - Channel 8 sample time selection
 */
#define SAR_ADC_SMP1_CH8_SMP_SEL_POS             (0)
#define SAR_ADC_SMP1_CH8_SMP_SEL_MSK             (0x7UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH8_SMP_SEL_64ADCCLK        (0x7UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH8_SMP_SEL_48ADCCLK        (0x6UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH8_SMP_SEL_32ADCCLK        (0x5UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH8_SMP_SEL_16ADCCLK        (0x4UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH8_SMP_SEL_8ADCCLK         (0x3UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH8_SMP_SEL_4ADCCLK         (0x2UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH8_SMP_SEL_2ADCCLK         (0x1UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)
#define SAR_ADC_SMP1_CH8_SMP_SEL_1ADCCLK         (0x0UL << SAR_ADC_SMP1_CH8_SMP_SEL_POS)

/*
 * Bit definition for SAR_ADC_REG_SEQ[31:28] - Regular sequence 7 selection
 */
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS           (28)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_MSK           (0xFUL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH9       (0x9UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH8       (0x8UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH7       (0x7UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH6       (0x6UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH5       (0x5UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH4       (0x4UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH3       (0x3UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH2       (0x2UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH1       (0x1UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ7TH_SEL_ADC_CH0       (0x0UL << SAR_ADC_REG_SEQ_SEQ7TH_SEL_POS)

/*
 * Bit definition for SAR_ADC_REG_SEQ[27:24] - Regular sequence 6 selection
 */
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS           (24)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_MSK           (0xFUL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH9       (0x9UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH8       (0x8UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH7       (0x7UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH6       (0x6UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH5       (0x5UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH4       (0x4UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH3       (0x3UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH2       (0x2UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH1       (0x1UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ6TH_SEL_ADC_CH0       (0x0UL << SAR_ADC_REG_SEQ_SEQ6TH_SEL_POS)

/*
 * Bit definition for SAR_ADC_REG_SEQ[23:20] - Regular sequence 5 selection
 */
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS           (20)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_MSK           (0xFUL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH9       (0x9UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH8       (0x8UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH7       (0x7UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH6       (0x6UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH5       (0x5UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH4       (0x4UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH3       (0x3UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH2       (0x2UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH1       (0x1UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ5TH_SEL_ADC_CH0       (0x0UL << SAR_ADC_REG_SEQ_SEQ5TH_SEL_POS)

/*
 * Bit definition for SAR_ADC_REG_SEQ[19:16] - Regular sequence 4 selection
 */
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS           (16)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_MSK           (0xFUL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH9       (0x9UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH8       (0x8UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH7       (0x7UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH6       (0x6UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH5       (0x5UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH4       (0x4UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH3       (0x3UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH2       (0x2UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH1       (0x1UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ4TH_SEL_ADC_CH0       (0x0UL << SAR_ADC_REG_SEQ_SEQ4TH_SEL_POS)

/*
 * Bit definition for SAR_ADC_REG_SEQ[15:12] - Regular sequence 3 selection
 */
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS           (12)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_MSK           (0xFUL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH9       (0x9UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH8       (0x8UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH7       (0x7UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH6       (0x6UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH5       (0x5UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH4       (0x4UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH3       (0x3UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH2       (0x2UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH1       (0x1UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ3RD_SEL_ADC_CH0       (0x0UL << SAR_ADC_REG_SEQ_SEQ3RD_SEL_POS)

/*
 * Bit definition for SAR_ADC_REG_SEQ[11:8] - Regular sequence 2 selection
 */
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS           (8)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_MSK           (0xFUL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH9       (0x9UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH8       (0x8UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH7       (0x7UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH6       (0x6UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH5       (0x5UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH4       (0x4UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH3       (0x3UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH2       (0x2UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH1       (0x1UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ2ND_SEL_ADC_CH0       (0x0UL << SAR_ADC_REG_SEQ_SEQ2ND_SEL_POS)

/*
 * Bit definition for SAR_ADC_REG_SEQ[7:4] - Regular sequence 1 selection
 */
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS           (4)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_MSK           (0xFUL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH9       (0x9UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH8       (0x8UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH7       (0x7UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH6       (0x6UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH5       (0x5UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH4       (0x4UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH3       (0x3UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH2       (0x2UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH1       (0x1UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ1ST_SEL_ADC_CH0       (0x0UL << SAR_ADC_REG_SEQ_SEQ1ST_SEL_POS)

/*
 * Bit definition for SAR_ADC_REG_SEQ[3:0] - Regular sequence 0 selection
 */
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS           (0)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_MSK           (0xFUL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH9       (0x9UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH8       (0x8UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH7       (0x7UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH6       (0x6UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH5       (0x5UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH4       (0x4UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH3       (0x3UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH2       (0x2UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH1       (0x1UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_REG_SEQ_SEQ0TH_SEL_ADC_CH0       (0x0UL << SAR_ADC_REG_SEQ_SEQ0TH_SEL_POS)

/*
 * Bit definition for SAR_ADC_INJ_SEQ[15:12] - Injected sequence 3 selection
 */
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS           (12)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_MSK           (0xFUL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH9       (0x9UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH8       (0x8UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH7       (0x7UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH6       (0x6UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH5       (0x5UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH4       (0x4UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH3       (0x3UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH2       (0x2UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH1       (0x1UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ3RD_SEL_ADC_CH0       (0x0UL << SAR_ADC_INJ_SEQ_SEQ3RD_SEL_POS)

/*
 * Bit definition for SAR_ADC_INJ_SEQ[11:8] - Injected sequence 2 selection
 */
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS           (8)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_MSK           (0xFUL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH9       (0x9UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH8       (0x8UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH7       (0x7UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH6       (0x6UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH5       (0x5UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH4       (0x4UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH3       (0x3UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH2       (0x2UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH1       (0x1UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ2ND_SEL_ADC_CH0       (0x0UL << SAR_ADC_INJ_SEQ_SEQ2ND_SEL_POS)

/*
 * Bit definition for SAR_ADC_INJ_SEQ[7:4] - Injected sequence 1 selection
 */
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS           (4)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_MSK           (0xFUL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH9       (0x9UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH8       (0x8UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH7       (0x7UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH6       (0x6UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH5       (0x5UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH4       (0x4UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH3       (0x3UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH2       (0x2UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH1       (0x1UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ1ST_SEL_ADC_CH0       (0x0UL << SAR_ADC_INJ_SEQ_SEQ1ST_SEL_POS)

/*
 * Bit definition for SAR_ADC_INJ_SEQ[3:0] - Injected sequence 0 selection
 */
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS           (0)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_MSK           (0xFUL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH9       (0x9UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH8       (0x8UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH7       (0x7UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH6       (0x6UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH5       (0x5UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH4       (0x4UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH3       (0x3UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH2       (0x2UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH1       (0x1UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)
#define SAR_ADC_INJ_SEQ_SEQ0TH_SEL_ADC_CH0       (0x0UL << SAR_ADC_INJ_SEQ_SEQ0TH_SEL_POS)

/*
 * Bit definition for SAR_ADC_WDG_TH[27:16] - Analog watch-dog high threshold
 */
#define SAR_ADC_WDG_TH_HIGH_THRESHOLD_POS        (16)
#define SAR_ADC_WDG_TH_HIGH_THRESHOLD_MSK        (0xFFFUL << SAR_ADC_WDG_TH_HIGH_THRESHOLD_POS)

/*
 * Bit definition for SAR_ADC_WDG_TH[11:0] - Analog watch-dog low threshold
 */
#define SAR_ADC_WDG_TH_LOW_THRESHOLD_POS         (0)
#define SAR_ADC_WDG_TH_LOW_THRESHOLD_MSK         (0xFFFUL << SAR_ADC_WDG_TH_LOW_THRESHOLD_POS)


/*---------------------------------------------------------------------------------------
 * Delta-sigma ADC Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;                          // Delta-sigma ADC Control Register
    __IO uint32_t AGC_CTRL0;                     // AGC Control Register 0
    __IO uint32_t AGC_CTRL1;                     // AGC Control Register 1
    __IO uint32_t MUTE_CTRL0;                    // Auto-mute Control Register 0
    __IO uint32_t MUTE_CTRL1;                    // Auto-mute Control Register 1
    __IO uint32_t RESERVED;                      //
    __IO uint32_t STS;                           // Delta-sigma ADC Status Register
    __IO uint32_t DATA;                          // Sigma-Delta ADC Data Register
} DS_ADC_TYPE_DEF;

/*
 * Bit definition for DS_ADC_CTRL[31] - Delta-Sigma ADC interrupt enable
 */
#define DS_ADC_CTRL_PDM_EN_POS                   (31)
#define DS_ADC_CTRL_PDM_EN_MSK                   (0x1UL << DS_ADC_CTRL_PDM_EN_POS)
#define DS_ADC_CTRL_PDM_ENABLE                   (0x1UL << DS_ADC_CTRL_PDM_EN_POS)
#define DS_ADC_CTRL_PDM_DISBLE                   (0x0UL << DS_ADC_CTRL_PDM_EN_POS)

/*
 * Bit definition for DS_ADC_CTRL[30:29] - PDM_SAMP_Sel PDM L/R-Ch Resample Timing Select
 */
#define DS_ADC_CTRL_PDM_RESAMPLE_SEL_POS         (29)
#define DS_ADC_CTRL_PDM_RESAMPLE_SEL_MSK         (0x3UL << DS_ADC_CTRL_PDM_RESAMPLE_SEL_POS)
#define DS_ADC_CTRL_PDM_L_FALLING_R_FALLING      (0x3UL << DS_ADC_CTRL_PDM_RESAMPLE_SEL_POS)
#define DS_ADC_CTRL_PDM_L_RISING_R_RISING        (0x2UL << DS_ADC_CTRL_PDM_RESAMPLE_SEL_POS)
#define DS_ADC_CTRL_PDM_L_FALLING_R_RISING       (0x1UL << DS_ADC_CTRL_PDM_RESAMPLE_SEL_POS)
#define DS_ADC_CTRL_PDM_L_RISING_R_FALLING       (0x0UL << DS_ADC_CTRL_PDM_RESAMPLE_SEL_POS)

/*
 * Bit definition for DS_ADC_CTRL[28] - PDM Mic Right-CH data select
 */
#define DS_ADC_CTRL_PDM_RCH_DATA_SEL_POS         (28)
#define DS_ADC_CTRL_PDM_RCH_DATA_SEL_MSK         (0x1UL << DS_ADC_CTRL_PDM_RCH_DATA_SEL_POS)
#define DS_ADC_CTRL_PDM_RCH_DATA_EQU_LCH         (0x1UL << DS_ADC_CTRL_PDM_RCH_DATA_SEL_POS)
#define DS_ADC_CTRL_PDM_RCH_DATA_ZERO            (0x0UL << DS_ADC_CTRL_PDM_RCH_DATA_SEL_POS)

/*
 * Bit definition for DS_ADC_CTRL[22:20] - Delta-Sigma ADC FIFO level
 */
#define DS_ADC_CTRL_FIFO_LVL_POS                 (20)
#define DS_ADC_CTRL_FIFO_LVL_MSK                 (0x7UL << DS_ADC_CTRL_FIFO_LVL_POS)
#define DS_ADC_CTRL_FIFO_LVL_7                   (0x7UL << DS_ADC_CTRL_FIFO_LVL_POS)
#define DS_ADC_CTRL_FIFO_LVL_6                   (0x6UL << DS_ADC_CTRL_FIFO_LVL_POS)
#define DS_ADC_CTRL_FIFO_LVL_5                   (0x5UL << DS_ADC_CTRL_FIFO_LVL_POS)
#define DS_ADC_CTRL_FIFO_LVL_4                   (0x4UL << DS_ADC_CTRL_FIFO_LVL_POS)
#define DS_ADC_CTRL_FIFO_LVL_3                   (0x3UL << DS_ADC_CTRL_FIFO_LVL_POS)
#define DS_ADC_CTRL_FIFO_LVL_2                   (0x2UL << DS_ADC_CTRL_FIFO_LVL_POS)
#define DS_ADC_CTRL_FIFO_LVL_1                   (0x1UL << DS_ADC_CTRL_FIFO_LVL_POS)
#define DS_ADC_CTRL_FIFO_LVL_0                   (0x0UL << DS_ADC_CTRL_FIFO_LVL_POS)

/*
 * Bit definition for DS_ADC_CTRL[18] - Delta-Sigma ADC interrupt enable
 */
#define DS_ADC_CTRL_INT_EN_POS                   (18)
#define DS_ADC_CTRL_INT_EN_MSK                   (0x1UL << DS_ADC_CTRL_INT_EN_POS)
#define DS_ADC_CTRL_INT_ENABLE                   (0x1UL << DS_ADC_CTRL_INT_EN_POS)
#define DS_ADC_CTRL_INT_DISBLE                   (0x0UL << DS_ADC_CTRL_INT_EN_POS)

/*
 * Bit definition for DS_ADC_CTRL[17] - Delta-Sigma ADC FIFO overwrite enable
 */
#define DS_ADC_CTRL_FIFO_OVWR_EN_POS             (17)
#define DS_ADC_CTRL_FIFO_OVWR_EN_MSK             (0x1UL << DS_ADC_CTRL_FIFO_OVWR_EN_POS)
#define DS_ADC_CTRL_FIFO_OVWR_ENABLE             (0x1UL << DS_ADC_CTRL_FIFO_OVWR_EN_POS)
#define DS_ADC_CTRL_FIFO_OVWR_DISABLE            (0x0UL << DS_ADC_CTRL_FIFO_OVWR_EN_POS)

/*
 * Bit definition for DS_ADC_CTRL[16] - Delta-Sigma ADC FIFO enable
 */
#define DS_ADC_CTRL_FIFO_EN_POS                  (16)
#define DS_ADC_CTRL_FIFO_EN_MSK                  (0x1UL << DS_ADC_CTRL_FIFO_EN_POS)
#define DS_ADC_CTRL_FIFO_ENABLE                  (0x1UL << DS_ADC_CTRL_FIFO_EN_POS)
#define DS_ADC_CTRL_FIFO_DISABLE                 (0x0UL << DS_ADC_CTRL_FIFO_EN_POS)

/*
 * Bit definition for DS_ADC_CTRL[15] - Sigma-Delta ADC Output Data signed/unsigned select
 */
#define DS_ADC_CTRL_FMT_SEL_POS                  (15)
#define DS_ADC_CTRL_FMT_SEL_MSK                  (0x1UL << DS_ADC_CTRL_FMT_SEL_POS)
#define DS_ADC_CTRL_FMT_SEL_SIGN                 (0x1UL << DS_ADC_CTRL_FMT_SEL_POS)
#define DS_ADC_CTRL_FMT_SEL_UNSIGN               (0x0UL << DS_ADC_CTRL_FMT_SEL_POS)

/*
 * Bit definition for DS_ADC_CTRL[12:8] - PGA gain
 */
#define DS_ADC_CTRL_PGA_GAIN_POS                 (8)
#define DS_ADC_CTRL_PGA_GAIN_MSK                 (0x1FUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV0                 (0x1FUL << DS_ADC_CTRL_PGA_GAIN_POS)    // Min. Gain
#define DS_ADC_CTRL_PGA_GAIN_LV1                 (0x1EUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV2                 (0x1DUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV3                 (0x1CUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV4                 (0x1BUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV5                 (0x1AUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV6                 (0x19UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV7                 (0x18UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV8                 (0x17UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV9                 (0x16UL << DS_ADC_CTRL_PGA_GAIN_POS)    // 0db
#define DS_ADC_CTRL_PGA_GAIN_LV10                (0x15UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV11                (0x14UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV12                (0x13UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV13                (0x12UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV14                (0x11UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV15                (0x10UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV16                (0x0FUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV17                (0x0EUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV18                (0x0DUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV19                (0x0CUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV20                (0x0BUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV21                (0x0AUL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV22                (0x09UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV23                (0x08UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV24                (0x07UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV25                (0x06UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV26                (0x05UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV27                (0x04UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV28                (0x03UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV29                (0x02UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV30                (0x01UL << DS_ADC_CTRL_PGA_GAIN_POS)
#define DS_ADC_CTRL_PGA_GAIN_LV31                (0x00UL << DS_ADC_CTRL_PGA_GAIN_POS)    // Max. Gain

/*
 * Bit definition for DS_ADC_CTRL[7:6] - Delta-Sigma ADC input limist range
 */
#define DS_ADC_CTRL_IN_LIMIT_POS                 (6)
#define DS_ADC_CTRL_IN_LIMIT_MSK                 (0x3UL << DS_ADC_CTRL_IN_LIMIT_POS)
#define DS_ADC_CTRL_IN_LIMIT_0P5FULL             (0x3UL << DS_ADC_CTRL_IN_LIMIT_POS)
#define DS_ADC_CTRL_IN_LIMIT_0P6FULL             (0x2UL << DS_ADC_CTRL_IN_LIMIT_POS)
#define DS_ADC_CTRL_IN_LIMIT_0P71FULL            (0x1UL << DS_ADC_CTRL_IN_LIMIT_POS)
#define DS_ADC_CTRL_IN_LIMIT_O084FULL            (0x0UL << DS_ADC_CTRL_IN_LIMIT_POS)

/*
 * Bit definition for DS_ADC_CTRL[5:4] - Delta-Sigma ADC boost gain
 */
#define DS_ADC_CTRL_BOOST_GAIN_POS               (4)
#define DS_ADC_CTRL_BOOST_GAIN_MSK               (0x3UL << DS_ADC_CTRL_BOOST_GAIN_POS)
#define DS_ADC_CTRL_BOOST_GAIN_LV3               (0x3UL << DS_ADC_CTRL_BOOST_GAIN_POS)
#define DS_ADC_CTRL_BOOST_GAIN_LV2               (0x2UL << DS_ADC_CTRL_BOOST_GAIN_POS)
#define DS_ADC_CTRL_BOOST_GAIN_LV1               (0x1UL << DS_ADC_CTRL_BOOST_GAIN_POS)
#define DS_ADC_CTRL_BOOST_GAIN_LV0               (0x0UL << DS_ADC_CTRL_BOOST_GAIN_POS)

/*
 * Bit definition for DS_ADC_CTRL[3] - Delta-Sigma ADC enable
 */
#define DS_ADC_CTRL_DSADC_EN_POS                 (3)
#define DS_ADC_CTRL_DSADC_EN_MSK                 (0x1UL << DS_ADC_CTRL_DSADC_EN_POS)
#define DS_ADC_CTRL_DSADC_ENABLE                 (0x1UL << DS_ADC_CTRL_DSADC_EN_POS)
#define DS_ADC_CTRL_DSADC_DISABLE                (0x0UL << DS_ADC_CTRL_DSADC_EN_POS)

/*
 * Bit definition for DS_ADC_CTRL[2] - Delta-Sigma ADC reset
 */
#define DS_ADC_CTRL_DSADC_RST_POS                (2)
#define DS_ADC_CTRL_DSADC_RST_MSK                (0x1UL << DS_ADC_CTRL_DSADC_RST_POS)
#define DS_ADC_CTRL_DSADC_NORMAL                 (0x1UL << DS_ADC_CTRL_DSADC_RST_POS)
#define DS_ADC_CTRL_DSADC_RESET                  (0x0UL << DS_ADC_CTRL_DSADC_RST_POS)

/*
 * Bit definition for DS_ADC_CTRL[1] - High pass filter enable
 */
#define DS_ADC_CTRL_HPF_EN_POS                   (1)
#define DS_ADC_CTRL_HPF_EN_MSK                   (0x1UL << DS_ADC_CTRL_HPF_EN_POS)
#define DS_ADC_CTRL_HPF_ENABLE                   (0x1UL << DS_ADC_CTRL_HPF_EN_POS)
#define DS_ADC_CTRL_HPF_DISABLE                  (0x0UL << DS_ADC_CTRL_HPF_EN_POS)

/*
 * Bit definition for DS_ADC_CTRL[0] - Mic enable
 */
#define DS_ADC_CTRL_MIC_EN_POS                   (0)
#define DS_ADC_CTRL_MIC_EN_MSK                   (0x1UL << DS_ADC_CTRL_MIC_EN_POS)
#define DS_ADC_CTRL_MIC_ENABLE                   (0x1UL << DS_ADC_CTRL_MIC_EN_POS)
#define DS_ADC_CTRL_MIC_DISABLE                  (0x0UL << DS_ADC_CTRL_MIC_EN_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL0[30:16] - Toggle Threshold
 */
#define DS_ADC_AGC_CTRL0_TOGGLE_THRESHOLD_POS    (16)
#define DS_ADC_AGC_CTRL0_TOGGLE_THRESHOLD_MSK    (0x7FFFUL << DS_ADC_AGC_CTRL0_TOGGLE_THRESHOLD_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL0[14:8] - Threshold
 */
#define DS_ADC_AGC_CTRL0_THRESHOLD_POS           (8)
#define DS_ADC_AGC_CTRL0_THRESHOLD_MSK           (0x7FUL << DS_ADC_AGC_CTRL0_THRESHOLD_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL0[2] - Digital AGC enable
 */
#define DS_ADC_AGC_CTRL0_DAGC_EN_POS             (2)
#define DS_ADC_AGC_CTRL0_DAGC_EN_MSK             (0x1UL << DS_ADC_AGC_CTRL0_DAGC_EN_POS)
#define DS_ADC_AGC_CTRL0_DAGC_ENABLE             (0x1UL << DS_ADC_AGC_CTRL0_DAGC_EN_POS)
#define DS_ADC_AGC_CTRL0_DAGC_DISABLE            (0x0UL << DS_ADC_AGC_CTRL0_DAGC_EN_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL0[1] - Zero Cross Function of Compressor Control
 */
#define DS_ADC_AGC_CTRL0_ZERO_CROSS_POS          (1)
#define DS_ADC_AGC_CTRL0_ZERO_CROSS_MSK          (0x1UL << DS_ADC_AGC_CTRL0_ZERO_CROSS_POS)
#define DS_ADC_AGC_CTRL0_ZERO_CROSS_DISABLE      (0x1UL << DS_ADC_AGC_CTRL0_ZERO_CROSS_POS)
#define DS_ADC_AGC_CTRL0_ZERO_CROSS_ENABLE       (0x0UL << DS_ADC_AGC_CTRL0_ZERO_CROSS_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL0[0] - AGC Mode Control
 */
#define DS_ADC_AGC_CTRL0_MODE_SEL_POS            (0)
#define DS_ADC_AGC_CTRL0_MODE_SEL_MSK            (0x1UL << DS_ADC_AGC_CTRL0_MODE_SEL_POS)
#define DS_ADC_AGC_CTRL0_MODE_SEL_PEAK           (0x1UL << DS_ADC_AGC_CTRL0_MODE_SEL_POS)
#define DS_ADC_AGC_CTRL0_MODE_SEL_RMS            (0x0UL << DS_ADC_AGC_CTRL0_MODE_SEL_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL1[31:20] - Update frequence
 */
#define DS_ADC_AGC_CTRL1_UPDATE_FREQ_POS         (20)
#define DS_ADC_AGC_CTRL1_UPDATE_FREQ_MSK         (0xFFFUL << DS_ADC_AGC_CTRL1_UPDATE_FREQ_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL1[19:18] - Release time scale control
 */
#define DS_ADC_AGC_CTRL1_RELEASE_SCALE_POS       (18)
#define DS_ADC_AGC_CTRL1_RELEASE_SCALE_MSK       (0x3UL << DS_ADC_AGC_CTRL1_RELEASE_SCALE_POS)
#define DS_ADC_AGC_CTRL1_RELEASE_TIME_X64        (0x3UL << DS_ADC_AGC_CTRL1_RELEASE_SCALE_POS)
#define DS_ADC_AGC_CTRL1_RELEASE_TIME_X16        (0x2UL << DS_ADC_AGC_CTRL1_RELEASE_SCALE_POS)
#define DS_ADC_AGC_CTRL1_RELEASE_TIME_X4         (0x1UL << DS_ADC_AGC_CTRL1_RELEASE_SCALE_POS)
#define DS_ADC_AGC_CTRL1_RELEASE_TIME_X1         (0x0UL << DS_ADC_AGC_CTRL1_RELEASE_SCALE_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL1[17:10] - Release time
 */
#define DS_ADC_AGC_CTRL1_RELEASE_TIME_POS        (10)
#define DS_ADC_AGC_CTRL1_RELEASE_TIME_MSK        (0xFFUL << DS_ADC_AGC_CTRL1_RELEASE_TIME_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL1[9:8] - Attack time scale control
 */
#define DS_ADC_AGC_CTRL1_ATTACK_SCALE_POS        (8)
#define DS_ADC_AGC_CTRL1_ATTACK_SCALE_MSK        (0x3UL << DS_ADC_AGC_CTRL1_ATTACK_SCALE_POS)
#define DS_ADC_AGC_CTRL1_ATTACK_TIME_X64         (0x3UL << DS_ADC_AGC_CTRL1_ATTACK_SCALE_POS)
#define DS_ADC_AGC_CTRL1_ATTACK_TIME_X16         (0x2UL << DS_ADC_AGC_CTRL1_ATTACK_SCALE_POS)
#define DS_ADC_AGC_CTRL1_ATTACK_TIME_X4          (0x1UL << DS_ADC_AGC_CTRL1_ATTACK_SCALE_POS)
#define DS_ADC_AGC_CTRL1_ATTACK_TIME_X1          (0x0UL << DS_ADC_AGC_CTRL1_ATTACK_SCALE_POS)

/*
 * Bit definition for DS_ADC_AGC_CTRL1[7:0] - Attack time
 */
#define DS_ADC_AGC_CTRL1_ATTACK_TIME_POS         (0)
#define DS_ADC_AGC_CTRL1_ATTACK_TIME_MSK         (0xFFUL << DS_ADC_AGC_CTRL1_ATTACK_TIME_POS)

/*
 * Bit definition for DS_ADC_MUTE_CTRL0[31:16] - Silence Threshold
 */
#define DS_ADC_MUTE_CTRL0_SILENCE_TH_POS         (16)
#define DS_ADC_MUTE_CTRL0_SILENCE_TH_MSK         (0xFFFFUL << DS_ADC_MUTE_CTRL0_SILENCE_TH_POS)

/*
 * Bit definition for DS_ADC_MUTE_CTRL0[4] - Echo canceling control bit
 */
#define DS_ADC_MUTE_CTRL0_ECHO_CANCEL_EN_POS     (4)
#define DS_ADC_MUTE_CTRL0_ECHO_CANCEL_EN_MSK     (0x1UL << DS_ADC_MUTE_CTRL0_ECHO_CANCEL_EN_POS)
#define DS_ADC_MUTE_CTRL0_ECHO_CANCEL_ENABLE     (0x1UL << DS_ADC_MUTE_CTRL0_ECHO_CANCEL_EN_POS)
#define DS_ADC_MUTE_CTRL0_ECHO_CANCEL_DISABLE    (0x0UL << DS_ADC_MUTE_CTRL0_ECHO_CANCEL_EN_POS)

/*
 * Bit definition for DS_ADC_MUTE_CTRL0[3:2] - Ramp counter step
 */
#define DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_POS       (2)
#define DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_MSK       (0x3UL << DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_POS)
#define DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_3         (0x3UL << DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_POS)
#define DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_2         (0x2UL << DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_POS)
#define DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_1         (0x1UL << DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_POS)
#define DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_0         (0x0UL << DS_ADC_MUTE_CTRL0_RAMPCNT_STEP_POS)

/*
 * Bit definition for DS_ADC_MUTE_CTRL0[1] - MIC AGC signed/unsigned data select
 */
#define DS_ADC_MUTE_CTRL0_FMT_SEL_POS            (1)
#define DS_ADC_MUTE_CTRL0_FMT_SEL_MSK            (0x1UL << DS_ADC_MUTE_CTRL0_FMT_SEL_POS)
#define DS_ADC_MUTE_CTRL0_FMT_SEL_SIGN           (0x1UL << DS_ADC_MUTE_CTRL0_FMT_SEL_POS)
#define DS_ADC_MUTE_CTRL0_FMT_SEL_UNSIGN         (0x0UL << DS_ADC_MUTE_CTRL0_FMT_SEL_POS)

/*
 * Bit definition for DS_ADC_MUTE_CTRL0[0] - MIC auto mute enable
 */
#define DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_EN_POS    (0)
#define DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_EN_MSK    (0x1UL << DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_EN_POS)
#define DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_ENABLE    (0x1UL << DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_EN_POS)
#define DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_DISABLE   (0x0UL << DS_ADC_MUTE_CTRL0_MIC_AUTOMUTE_EN_POS)

/*
 * Bit definition for DS_ADC_MUTE_CTRL1[31:16] - Normal debounce
 */
#define DS_ADC_MUTE_CTRL1_NORMAL_DEBOUNCE_POS    (16)
#define DS_ADC_MUTE_CTRL1_NORMAL_DEBOUNCE_MSK    (0xFFFFUL << DS_ADC_MUTE_CTRL1_NORMAL_DEBOUNCE_POS)

/*
 * Bit definition for DS_ADC_MUTE_CTRL1[15:0] - Silence debounce
 */
#define DS_ADC_MUTE_CTRL1_SILENCE_DEBOUNCE_POS   (0)
#define DS_ADC_MUTE_CTRL1_SILENCE_DEBOUNCE_MSK   (0xFFFFUL << DS_ADC_MUTE_CTRL1_SILENCE_DEBOUNCE_POS)

/*
 * Bit definition for DS_ADC_STS[19] - Delta-Sigma ADC FIFO full flag
 */
#define DS_ADC_STS_FIFO_FULL_POS                 (19)
#define DS_ADC_STS_FIFO_FULL_MSK                 (0x1UL << DS_ADC_STS_FIFO_FULL_POS)
#define DS_ADC_STS_FIFO_FULL_FLAG                (0x1UL << DS_ADC_STS_FIFO_FULL_POS)

/*
 * Bit definition for DS_ADC_STS[18] - Delta-Sigma ADC interrupt flag
 */
#define DS_ADC_STS_INTF_POS                      (18)
#define DS_ADC_STS_INTF_MSK                      (0x1UL << DS_ADC_STS_INTF_POS)
#define DS_ADC_STS_INT_FLAG                      (0x1UL << DS_ADC_STS_INTF_POS)

/*
 * Bit definition for DS_ADC_STS[12:8] - Actually PGAG gain value
 */
#define DS_ADC_STS_CURPGA_GAIN_POS               (8)
#define DS_ADC_STS_CURPGA_GAIN_MSK               (0x1FUL << DS_ADC_STS_CURPGA_GAIN_POS)


/*---------------------------------------------------------------------------------------
 * DAC Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;                          // DAC Control Register
    __IO uint32_t STS;                           // DAC Status Register
    __IO uint32_t AUDPWM_CTRL2;                  // AUDIO PWM Control Register2
    __IO uint32_t AUDPWM_CTRL;                   // AUDIO PWM Control Register
    __IO uint32_t DAC_CH0_DATA;                  // DAC Channel0 Input Data
    __IO uint32_t DAC_CH1_DATA;                  // DAC Channel1 Input Data
    __IO uint32_t MIX_DATA_OUT;                  // DAC Mixed Output Data
    __I  uint32_t RESERVED1;                     // Reserved
    __IO uint32_t DAC_CH0_DMA_DATA0;             // DAC Channel0 DMA fifo data0
    __IO uint32_t DAC_CH0_DMA_DATA1;             // DAC Channel0 DMA fifo data1
    __I  uint32_t RESERVED2[2];                  // Reserved
    __IO uint32_t DAC_CH1_DMA_DATA0;             // DAC Channel1 DMA fifo data0
    __IO uint32_t DAC_CH1_DMA_DATA1;             // DAC Channel1 DMA fifo data1
} DAC_TYPE_DEF;

/*
 * Bit definition for DAC_CTRL[31] - DAC Channel1 DMA fifo Error interrupt enable
 */
#define DAC_CTRL_DAC_CH1_DMAERR_INT_EN_POS       (31)
#define DAC_CTRL_DAC_CH1_DMAERR_INT_EN_MSK       (0x1UL << DAC_CTRL_DAC_CH1_DMAERR_INT_EN_POS)
#define DAC_CTRL_DAC_CH1_DMAERR_INT_ENABLE       (0x1UL << DAC_CTRL_DAC_CH1_DMAERR_INT_EN_POS)
#define DAC_CTRL_DAC_CH1_DMAERR_INT_DISABLE      (0x0UL << DAC_CTRL_DAC_CH1_DMAERR_INT_EN_POS)

/*
 * Bit definition for DAC_CTRL[30] - DAC Channel0 DMA fifo Error interrupt enable
 */
#define DAC_CTRL_DAC_CH0_DMAERR_INT_EN_POS       (30)
#define DAC_CTRL_DAC_CH0_DMAERR_INT_EN_MSK       (0x1UL << DAC_CTRL_DAC_CH0_DMAERR_INT_EN_POS)
#define DAC_CTRL_DAC_CH0_DMAERR_INT_ENABLE       (0x1UL << DAC_CTRL_DAC_CH0_DMAERR_INT_EN_POS)
#define DAC_CTRL_DAC_CH0_DMAERR_INT_DISABLE      (0x0UL << DAC_CTRL_DAC_CH0_DMAERR_INT_EN_POS)

/*
 * Bit definition for DAC_CTRL[29] - DAC Channel1 interrupt enable
 */
#define DAC_CTRL_DAC_CH1_INT_EN_POS              (29)
#define DAC_CTRL_DAC_CH1_INT_EN_MSK              (0x1UL << DAC_CTRL_DAC_CH1_INT_EN_POS)
#define DAC_CTRL_DAC_CH1_INT_ENABLE              (0x1UL << DAC_CTRL_DAC_CH1_INT_EN_POS)
#define DAC_CTRL_DAC_CH1_INT_DISABLE             (0x0UL << DAC_CTRL_DAC_CH1_INT_EN_POS)

/*
 * Bit definition for DAC_CTRL[28] - DAC Channel0 interrupt enable
 */
#define DAC_CTRL_DAC_CH0_INT_EN_POS              (28)
#define DAC_CTRL_DAC_CH0_INT_EN_MSK              (0x1UL << DAC_CTRL_DAC_CH0_INT_EN_POS)
#define DAC_CTRL_DAC_CH0_INT_ENABLE              (0x1UL << DAC_CTRL_DAC_CH0_INT_EN_POS)
#define DAC_CTRL_DAC_CH0_INT_DISABLE             (0x0UL << DAC_CTRL_DAC_CH0_INT_EN_POS)

/*
 * Bit definition for DAC_CTRL[27]
 */
#define DAC_CTRL_CH1_SRC_POS                     (27)
#define DAC_CTRL_CH1_SRC_MSK                     (0x1UL << DAC_CTRL_CH1_SRC_POS)
#define DAC_CTRL_CH1_SRC_MIX_CH01                (0x0UL << DAC_CTRL_CH1_SRC_POS)
#define DAC_CTRL_CH1_SRC_ONLY_CH1                (0x1UL << DAC_CTRL_CH1_SRC_POS)

/*
 * Bit definition for DAC_CTRL[26]
 */
#define DAC_CTRL_CH0_SRC_POS                     (26)
#define DAC_CTRL_CH0_SRC_MSK                     (0x1UL << DAC_CTRL_CH0_SRC_POS)
#define DAC_CTRL_CH0_SRC_MIX_CH01                (0x0UL << DAC_CTRL_CH0_SRC_POS)
#define DAC_CTRL_CH0_SRC_ONLY_CH0                (0x1UL << DAC_CTRL_CH0_SRC_POS)


/*
 * Bit definition for DAC_CTRL[25] - Up sample function Software reset
 */
#define DAC_CTRL_UPSMP_SW_RST_POS                (25)
#define DAC_CTRL_UPSMP_SW_RST_MSK                (0x1UL << DAC_CTRL_UPSMP_SW_RST_POS)
#define DAC_CTRL_UPSMP_SW_RESET                  (0x1UL << DAC_CTRL_UPSMP_SW_RST_POS)

/*
 * Bit definition for DAC_CTRL[24] - DAC software reset
 */
#define DAC_CTRL_DAC_SW_RST_POS                  (24)
#define DAC_CTRL_DAC_SW_RST_MSK                  (0x1UL << DAC_CTRL_DAC_SW_RST_POS)
#define DAC_CTRL_DAC_SW_RESET                    (0x1UL << DAC_CTRL_DAC_SW_RST_POS)

/*
 * Bit definition for DAC_CTRL[23] - Voltage DAC IP enable
 */
#define DAC_CTRL_VOLTAGE_DAC_EN_POS              (23)
#define DAC_CTRL_VOLTAGE_DAC_EN_MSK              (0x1UL << DAC_CTRL_VOLTAGE_DAC_EN_POS)
#define DAC_CTRL_VOLTAGE_DAC_ENABLE              (0x1UL << DAC_CTRL_VOLTAGE_DAC_EN_POS)
#define DAC_CTRL_VOLTAGE_DAC_DISABLE             (0x0UL << DAC_CTRL_VOLTAGE_DAC_EN_POS)

/*
 * Bit definition for SPU->POSTWAVE_CTRL[22]
 */
#define DAC_CTRL_CH1_POSTWAVE_OUT_SRC_POS        (22)
#define DAC_CTRL_CH1_POSTWAVE_OUT_SRC_MSK        (0x1UL << DAC_CTRL_CH1_POSTWAVE_OUT_SRC_POS)
#define DAC_CTRL_CH1_POSTWAVE_OUT_CH1            (0x1UL << DAC_CTRL_CH1_POSTWAVE_OUT_SRC_POS)
#define DAC_CTRL_CH1_POSTWAVE_OUT_CH1ANDSPU      (0x0UL << DAC_CTRL_CH1_POSTWAVE_OUT_SRC_POS)

/*
 * Bit definition for DAC_CTRL[21] - DAC output data signed/unsigned select
 */
#define DAC_CTRL_DACOUT_FMT_SEL_POS              (21)
#define DAC_CTRL_DACOUT_FMT_SEL_MSK              (0x1UL << DAC_CTRL_DACOUT_FMT_SEL_POS)
#define DAC_CTRL_DACOUT_FMT_SEL_SIGN             (0x1UL << DAC_CTRL_DACOUT_FMT_SEL_POS)
#define DAC_CTRL_DACOUT_FMT_SEL_UNSIGN           (0x0UL << DAC_CTRL_DACOUT_FMT_SEL_POS)

/*
 * Bit definition for DAC_CTRL[20] - DAC input data signed/unsigned select
 */
#define DAC_CTRL_DACIN_FMT_SEL_POS               (20)
#define DAC_CTRL_DACIN_FMT_SEL_MSK               (0x1UL << DAC_CTRL_DACIN_FMT_SEL_POS)
#define DAC_CTRL_DACIN_FMT_SEL_UNSIGN            (0x1UL << DAC_CTRL_DACIN_FMT_SEL_POS)
#define DAC_CTRL_DACIN_FMT_SEL_SIGN              (0x0UL << DAC_CTRL_DACIN_FMT_SEL_POS)

/*
 * Bit definition for SPU->POSTWAVE_CTRL[19]
 */
#define DAC_CTRL_CH0_POSTWAVE_OUT_SRC_POS        (19)
#define DAC_CTRL_CH0_POSTWAVE_OUT_SRC_MSK        (0x1UL << DAC_CTRL_CH0_POSTWAVE_OUT_SRC_POS)
#define DAC_CTRL_CH0_POSTWAVE_OUT_CH0            (0x1UL << DAC_CTRL_CH0_POSTWAVE_OUT_SRC_POS)
#define DAC_CTRL_CH0_POSTWAVE_OUT_CH0ANDSPU      (0x0UL << DAC_CTRL_CH0_POSTWAVE_OUT_SRC_POS)

/*
 * Bit definition for DAC_CTRL[18] - DAC CH1 DMA continue mode enable
 */
#define DAC_CTRL_CH1_DMA_CONTINUE_EN_POS         (18)
#define DAC_CTRL_CH1_DMA_CONTINUE_EN_MSK         (0x1UL << DAC_CTRL_CH1_DMA_CONTINUE_EN_POS)
#define DAC_CTRL_CH1_DMA_CONTINUE_ENABLE         (0x1UL << DAC_CTRL_CH1_DMA_CONTINUE_EN_POS)
#define DAC_CTRL_CH1_DMA_CONTINUE_DISABLE        (0x0UL << DAC_CTRL_CH1_DMA_CONTINUE_EN_POS)

/*
 * Bit definition for DAC_CTRL[17] - DAC CH0 DMA continue mode enable
 */
#define DAC_CTRL_CH0_DMA_CONTINUE_EN_POS         (17)
#define DAC_CTRL_CH0_DMA_CONTINUE_EN_MSK         (0x1UL << DAC_CTRL_CH0_DMA_CONTINUE_EN_POS)
#define DAC_CTRL_CH0_DMA_CONTINUE_ENABLE         (0x1UL << DAC_CTRL_CH0_DMA_CONTINUE_EN_POS)
#define DAC_CTRL_CH0_DMA_CONTINUE_DISABLE        (0x0UL << DAC_CTRL_CH0_DMA_CONTINUE_EN_POS)

/*
 * Bit definition for DAC_CTRL[16] - DAC scale enable
 */
#define DAC_CTRL_SCALE_EN_POS                    (16)
#define DAC_CTRL_SCALE_EN_MSK                    (0x1UL << DAC_CTRL_SCALE_EN_POS)
#define DAC_CTRL_SCALE_ENABLE                    (0x1UL << DAC_CTRL_SCALE_EN_POS)
#define DAC_CTRL_SCALE_DISABLE                   (0x0UL << DAC_CTRL_SCALE_EN_POS)

/*
 * Bit definition for DAC_CTRL[15] - DAC Channel1 half volume enable
 */
#define DAC_CTRL_DAC_CH1_HALF_EN_POS             (15)
#define DAC_CTRL_DAC_CH1_HALF_EN_MSK             (0x1UL << DAC_CTRL_DAC_CH1_HALF_EN_POS)
#define DAC_CTRL_DAC_CH1_HALF_ENABLE             (0x1UL << DAC_CTRL_DAC_CH1_HALF_EN_POS)
#define DAC_CTRL_DAC_CH1_HALF_DISABLE            (0x0UL << DAC_CTRL_DAC_CH1_HALF_EN_POS)

/*
 * Bit definition for DAC_CTRL[14:12] - DAC channel1 Trigger source selection
 */
#define DAC_CTRL_DAC_CH1_TRG_SEL_POS             (12)
#define DAC_CTRL_DAC_CH1_TRG_SEL_MSK             (0x7UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH1_TRG_SEL_CTS_TM1         (0x7UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH1_TRG_SEL_CTS_TM0         (0x6UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH1_TRG_SEL_CCP1_TM         (0x5UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH1_TRG_SEL_CCP0_TM         (0x4UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH1_TRG_SEL_TM2             (0x3UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH1_TRG_SEL_TM1             (0x2UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH1_TRG_SEL_TM0             (0x1UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH1_TRG_SEL_Manual          (0x0UL << DAC_CTRL_DAC_CH1_TRG_SEL_POS)

/*
 * Bit definition for DAC_CTRL[11:10] - DAC channel1 UpSampling selection
 */
#define DAC_CTRL_DAC_CH1_UPSMP_MODE_POS          (10)
#define DAC_CTRL_DAC_CH1_UPSMP_MODE_MSK          (0x3UL << DAC_CTRL_DAC_CH1_UPSMP_MODE_POS)
#define DAC_CTRL_DAC_CH1_UPSMP_MODE_4X           (0x2UL << DAC_CTRL_DAC_CH1_UPSMP_MODE_POS)
#define DAC_CTRL_DAC_CH1_UPSMP_MODE_1X           (0x1UL << DAC_CTRL_DAC_CH1_UPSMP_MODE_POS)
#define DAC_CTRL_DAC_CH1_UPSMP_MODE_DISABLE      (0x0UL << DAC_CTRL_DAC_CH1_UPSMP_MODE_POS)

/*
 * Bit definition for DAC_CTRL[9] - DAC Channel1 DMA enable
 */
#define DAC_CTRL_DAC_CH1_DMA_EN_POS              (9)
#define DAC_CTRL_DAC_CH1_DMA_EN_MSK              (0x1UL << DAC_CTRL_DAC_CH1_DMA_EN_POS)
#define DAC_CTRL_DAC_CH1_DMA_ENABLE              (0x1UL << DAC_CTRL_DAC_CH1_DMA_EN_POS)
#define DAC_CTRL_DAC_CH1_DMA_DISABLE             (0x0UL << DAC_CTRL_DAC_CH1_DMA_EN_POS)

/*
 * Bit definition for DAC_CTRL[8] - DAC channel1 enable
 */
#define DAC_CTRL_DAC_CH1_EN_POS                  (8)
#define DAC_CTRL_DAC_CH1_EN_MSK                  (0x1UL << DAC_CTRL_DAC_CH1_EN_POS)
#define DAC_CTRL_DAC_CH1_ENABLE                  (0x1UL << DAC_CTRL_DAC_CH1_EN_POS)
#define DAC_CTRL_DAC_CH1_DISABLE                 (0x0UL << DAC_CTRL_DAC_CH1_EN_POS)

/*
 * Bit definition for DAC_CTRL[7] - DAC Channel0 half volume enable
 */
#define DAC_CTRL_DAC_CH0_HALF_EN_POS             (7)
#define DAC_CTRL_DAC_CH0_HALF_EN_MSK             (0x1UL << DAC_CTRL_DAC_CH0_HALF_EN_POS)
#define DAC_CTRL_DAC_CH0_HALF_ENABLE             (0x1UL << DAC_CTRL_DAC_CH0_HALF_EN_POS)
#define DAC_CTRL_DAC_CH0_HALF_DISABLE            (0x0UL << DAC_CTRL_DAC_CH0_HALF_EN_POS)

/*
 * Bit definition for DAC_CTRL[6:4] - DAC channel0 Trigger source selection
 */
#define DAC_CTRL_DAC_CH0_TRG_SEL_POS             (4)
#define DAC_CTRL_DAC_CH0_TRG_SEL_MSK             (0x7UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH0_TRG_SEL_CTS_TM1         (0x7UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH0_TRG_SEL_CTS_TM0         (0x6UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH0_TRG_SEL_CCP1_TM         (0x5UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH0_TRG_SEL_CCP0_TM         (0x4UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH0_TRG_SEL_TM2             (0x3UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH0_TRG_SEL_TM1             (0x2UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH0_TRG_SEL_TM0             (0x1UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)
#define DAC_CTRL_DAC_CH0_TRG_SEL_MANUAL          (0x0UL << DAC_CTRL_DAC_CH0_TRG_SEL_POS)

/*
 * Bit definition for DAC_CTRL[3:2] - DAC channel0 UpSampling selection
 */
#define DAC_CTRL_DAC_CH0_UPSMP_MODE_POS          (2)
#define DAC_CTRL_DAC_CH0_UPSMP_MODE_MSK          (0x3UL << DAC_CTRL_DAC_CH0_UPSMP_MODE_POS)
#define DAC_CTRL_DAC_CH0_UPSMP_MODE_4X           (0x2UL << DAC_CTRL_DAC_CH0_UPSMP_MODE_POS)
#define DAC_CTRL_DAC_CH0_UPSMP_MODE_1X           (0x1UL << DAC_CTRL_DAC_CH0_UPSMP_MODE_POS)
#define DAC_CTRL_DAC_CH0_UPSMP_MODE_DISABLE      (0x0UL << DAC_CTRL_DAC_CH0_UPSMP_MODE_POS)

/*
 * Bit definition for DAC_CTRL[1] - DAC Channel0 DMA enable
 */
#define DAC_CTRL_DAC_CH0_DMA_EN_POS              (1)
#define DAC_CTRL_DAC_CH0_DMA_EN_MSK              (0x1UL << DAC_CTRL_DAC_CH0_DMA_EN_POS)
#define DAC_CTRL_DAC_CH0_DMA_ENABLE              (0x1UL << DAC_CTRL_DAC_CH0_DMA_EN_POS)
#define DAC_CTRL_DAC_CH0_DMA_DISABLE             (0x0UL << DAC_CTRL_DAC_CH0_DMA_EN_POS)

/*
 * Bit definition for DAC_CTRL[0] - DAC channel0 enable
 */
#define DAC_CTRL_DAC_CH0_EN_POS                  (0)
#define DAC_CTRL_DAC_CH0_EN_MSK                  (0x1UL << DAC_CTRL_DAC_CH0_EN_POS)
#define DAC_CTRL_DAC_CH0_ENABLE                  (0x1UL << DAC_CTRL_DAC_CH0_EN_POS)
#define DAC_CTRL_DAC_CH0_DISABLE                 (0x0UL << DAC_CTRL_DAC_CH0_EN_POS)

/*
 * Bit definition for DAC_STS[31] - DAC Channel1 error flag
 */
#define DAC_STS_DAC_CH1_ERR_POS                  (31)
#define DAC_STS_DAC_CH1_ERR_MSK                  (0x1UL << DAC_STS_DAC_CH1_ERR_POS)
#define DAC_STS_DAC_CH1_ERR_FLAG                 (0x1UL << DAC_STS_DAC_CH1_ERR_POS)

/*
 * Bit definition for DAC_STS[30] - DAC Channel0 error flag
 */
#define DAC_STS_DAC_CH0_ERR_POS                  (30)
#define DAC_STS_DAC_CH0_ERR_MSK                  (0x1UL << DAC_STS_DAC_CH0_ERR_POS)
#define DAC_STS_DAC_CH0_ERR_FLAG                 (0x1UL << DAC_STS_DAC_CH0_ERR_POS)

/*
 * Bit definition for DAC_STS[29] - DAC Channel1 interrupt flag
 */
#define DAC_STS_DAC_CH1_INTF_POS                 (29)
#define DAC_STS_DAC_CH1_INTF_MSK                 (0x1UL << DAC_STS_DAC_CH1_INTF_POS)
#define DAC_STS_DAC_CH1_INT_FLAG                 (0x1UL << DAC_STS_DAC_CH1_INTF_POS)

/*
 * Bit definition for DAC_STS[28] - DAC Channel0 interrupt flag
 */
#define DAC_STS_DAC_CH0_INTF_POS                 (28)
#define DAC_STS_DAC_CH0_INTF_MSK                 (0x1UL << DAC_STS_DAC_CH0_INTF_POS)
#define DAC_STS_DAC_CH0_INT_FLAG                 (0x1UL << DAC_STS_DAC_CH0_INTF_POS)

/*
 * Bit definition for MUTE_OUT_CTRL[0] - DAC Channel0 interrupt flag
 */
#define DAC_STS_DAC_CH0_INTF_POS                 (28)
#define DAC_STS_DAC_CH0_INTF_MSK                 (0x1UL << DAC_STS_DAC_CH0_INTF_POS)
#define DAC_STS_DAC_CH0_INT_FLAG                 (0x1UL << DAC_STS_DAC_CH0_INTF_POS)

/*
 * Bit definition for AUDPWM_CTRL2[8:2]
 */
#define AUDPWM_CTRL2_MUTE_OUT_SEL_POS            (2)
#define AUDPWM_CTRL2_MUTE_OUT_SEL_MSK            (0x7FUL << AUDPWM_CTRL2_MUTE_OUT_SEL_POS)

/*
 * Bit definition for AUDPWM_CTRL2[1:0]
 */
#define AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_POS     (0)
#define AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_MSK     (0x3UL << AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_POS)
#define AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_64US    (0x0UL << AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_POS)
#define AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_128US   (0x1UL << AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_POS)
#define AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_256US   (0x2UL << AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_POS)
#define AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_512US   (0x3UL << AUDPWM_CTRL2_MUTE_OUT_CLEAR_TIME_POS)

/*
 * Bit definition for AUDPWM_CTRL[27:21]
 */
#define AUDPWM_CTRL_MUTE_IN_SEL_POS              (21)
#define AUDPWM_CTRL_MUTE_IN_SEL_MSK              (0x7FUL << AUDPWM_CTRL_MUTE_IN_SEL_POS)

/*
 * Bit definition for AUDPWM_CTRL[20]
 */
#define AUDPWM_CTRL_AUDP_STATE_POS               (20)
#define AUDPWM_CTRL_AUDP_STATE_MSK               (0x1UL << AUDPWM_CTRL_AUDP_STATE_POS)
#define AUDPWM_CTRL_AUDP_STATE_HIGH              (0x1UL << AUDPWM_CTRL_AUDP_STATE_POS)
#define AUDPWM_CTRL_AUDP_STATE_LOW               (0x0UL << AUDPWM_CTRL_AUDP_STATE_POS)

/*
 * Bit definition for AUDPWM_CTRL[19]
 */
#define AUDPWM_CTRL_AUDN_STATE_POS               (19)
#define AUDPWM_CTRL_AUDN_STATE_MSK               (0x1UL << AUDPWM_CTRL_AUDN_STATE_POS)
#define AUDPWM_CTRL_AUDN_STATE_HIGH              (0x1UL << AUDPWM_CTRL_AUDN_STATE_POS)
#define AUDPWM_CTRL_AUDN_STATE_LOW               (0x0UL << AUDPWM_CTRL_AUDN_STATE_POS)

/*
 * Bit definition for AUDPWM_CTRL[18]
 */
#define AUDPWM_CTRL_DAC_CH1_EN_POS               (18)
#define AUDPWM_CTRL_DAC_CH1_EN_MSK               (0x1UL << AUDPWM_CTRL_DAC_CH1_EN_POS)
#define AUDPWM_CTRL_DAC_CH1_ENABLE               (0x1UL << AUDPWM_CTRL_DAC_CH1_EN_POS)
#define AUDPWM_CTRL_DAC_CH1_DISABLE              (0x0UL << AUDPWM_CTRL_DAC_CH1_EN_POS)

/*
 * Bit definition for AUDPWM_CTRL[17]
 */
#define AUDPWM_CTRL_DAC_CH0_EN_POS               (17)
#define AUDPWM_CTRL_DAC_CH0_EN_MSK               (0x1UL << AUDPWM_CTRL_DAC_CH0_EN_POS)
#define AUDPWM_CTRL_DAC_CH0_ENABLE               (0x1UL << AUDPWM_CTRL_DAC_CH0_EN_POS)
#define AUDPWM_CTRL_DAC_CH0_DISABLE              (0x0UL << AUDPWM_CTRL_DAC_CH0_EN_POS)

/*
 * Bit definition for AUDPWM_CTRL[16] - PWM enable control
 */
#define AUDPWM_CTRL_AUDPWM_IP_EN_POS             (16)
#define AUDPWM_CTRL_AUDPWM_IP_EN_MSK             (0x1UL << AUDPWM_CTRL_AUDPWM_IP_EN_POS)
#define AUDPWM_CTRL_AUDPWM_IP_ENABLE             (0x1UL << AUDPWM_CTRL_AUDPWM_IP_EN_POS)
#define AUDPWM_CTRL_AUDPWM_IP_DISABLE            (0x0UL << AUDPWM_CTRL_AUDPWM_IP_EN_POS)

/*
 * Bit definition for AUDPWM_CTRL[15]
 */
#define AUDPWM_CTRL_DS_TYPE_POS                  (15)
#define AUDPWM_CTRL_DS_TYPE_MSK                  (0x1UL << AUDPWM_CTRL_DS_TYPE_POS)
#define AUDPWM_CTRL_DS_TYPE_DS6                  (0x1UL << AUDPWM_CTRL_DS_TYPE_POS)
#define AUDPWM_CTRL_DS_TYPE_DS3                  (0x0UL << AUDPWM_CTRL_DS_TYPE_POS)

/*
 * Bit definition for AUDPWM_CTRL[14]
 */
#define AUDPWM_CTRL_MUTE_TYPE_POS                (14)
#define AUDPWM_CTRL_MUTE_TYPE_MSK                (0x1UL << AUDPWM_CTRL_MUTE_TYPE_POS)
#define AUDPWM_CTRL_MUTE_TYPE_50DUTY             (0x1UL << AUDPWM_CTRL_MUTE_TYPE_POS)
#define AUDPWM_CTRL_MUTE_TYPE_PWMOFF             (0x0UL << AUDPWM_CTRL_MUTE_TYPE_POS)

/*
 * Bit definition for AUDPWM_CTRL[11:10] - Audio PWM Clock mute state selection
 */
#define AUDPWM_CTRL_MUTE_STATE_SEL_POS           (10)
#define AUDPWM_CTRL_MUTE_STATE_SEL_MSK           (0x3UL << AUDPWM_CTRL_MUTE_STATE_SEL_POS)
#define AUDPWM_CTRL_MUTE_STATE_SEL_8MHZ          (0x3UL << AUDPWM_CTRL_MUTE_STATE_SEL_POS)
#define AUDPWM_CTRL_MUTE_STATE_SEL_4MHZ          (0x2UL << AUDPWM_CTRL_MUTE_STATE_SEL_POS)
#define AUDPWM_CTRL_MUTE_STATE_SEL_2MHZ          (0x1UL << AUDPWM_CTRL_MUTE_STATE_SEL_POS)
#define AUDPWM_CTRL_MUTE_STATE_SEL_1MHZ          (0x0UL << AUDPWM_CTRL_MUTE_STATE_SEL_POS)

/*
 * Bit definition for AUDPWM_CTRL[9:4] - Audio PWM Gain
 */
#define AUDPWM_CTRL_AUDPWM_GAIN_POS              (4)
#define AUDPWM_CTRL_AUDPWM_GAIN_MSK              (0x3FUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV63             (0x3FUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV62             (0x3EUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV61             (0x3DUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV60             (0x3CUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV59             (0x3BUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV58             (0x3AUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV57             (0x39UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV56             (0x38UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV55             (0x37UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV54             (0x36UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV53             (0x35UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV52             (0x34UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV51             (0x33UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV50             (0x32UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV49             (0x31UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV48             (0x30UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV47             (0x2FUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV46             (0x2EUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV45             (0x2DUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV44             (0x2CUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV43             (0x2BUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV42             (0x2AUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV41             (0x29UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV40             (0x28UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV39             (0x27UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV38             (0x26UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV37             (0x25UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV36             (0x24UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV35             (0x23UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV34             (0x22UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV33             (0x21UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV32             (0x20UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV31             (0x1FUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV30             (0x1EUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV29             (0x1DUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV28             (0x1CUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV27             (0x1BUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV26             (0x1AUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV25             (0x19UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV24             (0x18UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV23             (0x17UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV22             (0x16UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV21             (0x15UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV20             (0x14UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV19             (0x13UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV18             (0x12UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV17             (0x11UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV16             (0x10UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV15             (0xFUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV14             (0xEUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV13             (0xDUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV12             (0xCUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV11             (0xBUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV10             (0xAUL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV9              (0x9UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV8              (0x8UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV7              (0x7UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV6              (0x6UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV5              (0x5UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV4              (0x4UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV3              (0x3UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV2              (0x2UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV1              (0x1UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)
#define AUDPWM_CTRL_AUDPWM_GAIN_LV0              (0x0UL << AUDPWM_CTRL_AUDPWM_GAIN_POS)

/*
 * Bit definition for AUDPWM_CTRL[3:2] - Auto Mute control
 */
#define AUDPWM_CTRL_MUTE_CTRL_POS                (2)
#define AUDPWM_CTRL_MUTE_CTRL_MSK                (0x3UL << AUDPWM_CTRL_MUTE_CTRL_POS)
#define AUDPWM_CTRL_MUTE_CTRL_BY_REG             (0x2UL << AUDPWM_CTRL_MUTE_CTRL_POS)
#define AUDPWM_CTRL_MUTE_CTRL_BY_DATACHANGE      (0x1UL << AUDPWM_CTRL_MUTE_CTRL_POS)
#define AUDPWM_CTRL_MUTE_CTRL_OFF                (0x0UL << AUDPWM_CTRL_MUTE_CTRL_POS)

/*
 * Bit definition for AUDPWM_CTRL[1] - Audio PWM input data signed/unsigned select
 */
#define AUDPWM_CTRL_DATAIN_FMT_SEL_POS           (1)
#define AUDPWM_CTRL_DATAIN_FMT_SEL_MSK           (0x1UL << AUDPWM_CTRL_DATAIN_FMT_SEL_POS)
#define AUDPWM_CTRL_DATAIN_FMT_SEL_UNSIGN        (0x1UL << AUDPWM_CTRL_DATAIN_FMT_SEL_POS)
#define AUDPWM_CTRL_DATAIN_FMT_SEL_SIGN          (0x0UL << AUDPWM_CTRL_DATAIN_FMT_SEL_POS)

/*
 * Bit definition for AUDPWM_CTRL[0] - Audio PWM enable
 */
#define AUDPWM_CTRL_AUDPWM_EN_POS                (0)
#define AUDPWM_CTRL_AUDPWM_EN_MSK                (0x1UL << AUDPWM_CTRL_AUDPWM_EN_POS)
#define AUDPWM_CTRL_AUDPWM_ENABLE                (0x1UL << AUDPWM_CTRL_AUDPWM_EN_POS)
#define AUDPWM_CTRL_AUDPWM_DISABLE               (0x0UL << AUDPWM_CTRL_AUDPWM_EN_POS)


/*---------------------------------------------------------------------------------------
 * PWMIO Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
 typedef struct
{
    __IO uint32_t CTRL0;                         // PWMIO Control0 Register
    __IO uint32_t CTRL1;                         // PWMIO Control1 Register
    __IO uint32_t TOGGLE_CTRL;                   // PWMIO Toggle Control Register
    __IO uint32_t PERIOD_CTRL;                   // PWMIO Period Control Register
    __IO uint32_t PWMIO0_Duty;                   // PWMIO0 Duty Control Register
    __IO uint32_t PWMIO1_Duty;                   // PWMIO1 Duty Control Register
    __IO uint32_t PWMIO2_Duty;                   // PWMIO2 Duty Control Register
    __IO uint32_t PWMIO3_Duty;                   // PWMIO3 Duty Control Register
    __IO uint32_t PWMIO4_Duty;                   // PWMIO4 Duty Control Register
    __IO uint32_t PWMIO5_Duty;                   // PWMIO5 Duty Control Register
    __IO uint32_t PWMIO6_Duty;                   // PWMIO6 Duty Control Register
    __IO uint32_t PWMIO7_Duty;                   // PWMIO7 Duty Control Register
    __IO uint32_t PWMIO8_Duty;                   // PWMIO8 Duty Control Register
    __IO uint32_t PWMIO9_Duty;                   // PWMIO9 Duty Control Register
    __IO uint32_t PWMIO10_Duty;                  // PWMIO10 Duty Control Register
    __IO uint32_t PWMIO11_Duty;                  // PWMIO11 Duty Control Register
    __IO uint32_t PWMIO12_Duty;                  // PWMIO12 Duty Control Register
    __IO uint32_t PWMIO13_Duty;                  // PWMIO13 Duty Control Register
    __IO uint32_t PWMIO14_Duty;                  // PWMIO14 Duty Control Register
    __IO uint32_t PWMIO15_Duty;                  // PWMIO15 Duty Control Register
    __IO uint32_t RESERVED[8];                   // Reserved
    __IO uint32_t OFFSET;                        // PWMIO7 Duty Control Register
} PWMIO_TYPE_DEF;

/*
 * Bit definition for PWMIO_CTRL0[31] - PWMIO channel 15 enable
 */
#define PWMIO_CTRL_PWM15_EN_POS                   (31)
#define PWMIO_CTRL_PWM15_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM15_EN_POS)
#define PWMIO_CTRL_PWM15_ENABLE                   (0x1UL << PWMIO_CTRL_PWM15_EN_POS)
#define PWMIO_CTRL_PWM15_DISABLE                  (0x0UL << PWMIO_CTRL_PWM15_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[30] - PWMIO channel 14 enable
 */
#define PWMIO_CTRL_PWM14_EN_POS                   (30)
#define PWMIO_CTRL_PWM14_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM14_EN_POS)
#define PWMIO_CTRL_PWM14_ENABLE                   (0x1UL << PWMIO_CTRL_PWM14_EN_POS)
#define PWMIO_CTRL_PWM14_DISABLE                  (0x0UL << PWMIO_CTRL_PWM14_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[29] - PWMIO channel 13 enable
 */
#define PWMIO_CTRL_PWM13_EN_POS                   (29)
#define PWMIO_CTRL_PWM13_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM13_EN_POS)
#define PWMIO_CTRL_PWM13_ENABLE                   (0x1UL << PWMIO_CTRL_PWM13_EN_POS)
#define PWMIO_CTRL_PWM13_DISABLE                  (0x0UL << PWMIO_CTRL_PWM13_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[28] - PWMIO channel 12 enable
 */
#define PWMIO_CTRL_PWM12_EN_POS                   (28)
#define PWMIO_CTRL_PWM12_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM12_EN_POS)
#define PWMIO_CTRL_PWM12_ENABLE                   (0x1UL << PWMIO_CTRL_PWM12_EN_POS)
#define PWMIO_CTRL_PWM12_DISABLE                  (0x0UL << PWMIO_CTRL_PWM12_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[27] - PWMIO channel 11 enable
 */
#define PWMIO_CTRL_PWM11_EN_POS                   (27)
#define PWMIO_CTRL_PWM11_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM11_EN_POS)
#define PWMIO_CTRL_PWM11_ENABLE                   (0x1UL << PWMIO_CTRL_PWM11_EN_POS)
#define PWMIO_CTRL_PWM11_DISABLE                  (0x0UL << PWMIO_CTRL_PWM11_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[26] - PWMIO channel 10 enable
 */
#define PWMIO_CTRL_PWM10_EN_POS                   (26)
#define PWMIO_CTRL_PWM10_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM10_EN_POS)
#define PWMIO_CTRL_PWM10_ENABLE                   (0x1UL << PWMIO_CTRL_PWM10_EN_POS)
#define PWMIO_CTRL_PWM10_DISABLE                  (0x0UL << PWMIO_CTRL_PWM10_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[25] - PWMIO channel 9 enable
 */
#define PWMIO_CTRL_PWM9_EN_POS                   (25)
#define PWMIO_CTRL_PWM9_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM9_EN_POS)
#define PWMIO_CTRL_PWM9_ENABLE                   (0x1UL << PWMIO_CTRL_PWM9_EN_POS)
#define PWMIO_CTRL_PWM9_DISABLE                  (0x0UL << PWMIO_CTRL_PWM9_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[24] - PWMIO channel 8 enable
 */
#define PWMIO_CTRL_PWM8_EN_POS                   (24)
#define PWMIO_CTRL_PWM8_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM8_EN_POS)
#define PWMIO_CTRL_PWM8_ENABLE                   (0x1UL << PWMIO_CTRL_PWM8_EN_POS)
#define PWMIO_CTRL_PWM8_DISABLE                  (0x0UL << PWMIO_CTRL_PWM8_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[23] - PWMIO channel 7 enable
 */
#define PWMIO_CTRL_PWM7_EN_POS                   (23)
#define PWMIO_CTRL_PWM7_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM7_EN_POS)
#define PWMIO_CTRL_PWM7_ENABLE                   (0x1UL << PWMIO_CTRL_PWM7_EN_POS)
#define PWMIO_CTRL_PWM7_DISABLE                  (0x0UL << PWMIO_CTRL_PWM7_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[22] - PWMIO channel 6 enable
 */
#define PWMIO_CTRL_PWM6_EN_POS                   (22)
#define PWMIO_CTRL_PWM6_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM6_EN_POS)
#define PWMIO_CTRL_PWM6_ENABLE                   (0x1UL << PWMIO_CTRL_PWM6_EN_POS)
#define PWMIO_CTRL_PWM6_DISABLE                  (0x0UL << PWMIO_CTRL_PWM6_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[21] - PWMIO channel 5 enable
 */
#define PWMIO_CTRL_PWM5_EN_POS                   (21)
#define PWMIO_CTRL_PWM5_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM5_EN_POS)
#define PWMIO_CTRL_PWM5_ENABLE                   (0x1UL << PWMIO_CTRL_PWM5_EN_POS)
#define PWMIO_CTRL_PWM5_DISABLE                  (0x0UL << PWMIO_CTRL_PWM5_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[20] - PWMIO channel 4 enable
 */
#define PWMIO_CTRL_PWM4_EN_POS                   (20)
#define PWMIO_CTRL_PWM4_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM4_EN_POS)
#define PWMIO_CTRL_PWM4_ENABLE                   (0x1UL << PWMIO_CTRL_PWM4_EN_POS)
#define PWMIO_CTRL_PWM4_DISABLE                  (0x0UL << PWMIO_CTRL_PWM4_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[19] - PWMIO channel 3 enable
 */
#define PWMIO_CTRL_PWM3_EN_POS                   (19)
#define PWMIO_CTRL_PWM3_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM3_EN_POS)
#define PWMIO_CTRL_PWM3_ENABLE                   (0x1UL << PWMIO_CTRL_PWM3_EN_POS)
#define PWMIO_CTRL_PWM3_DISABLE                  (0x0UL << PWMIO_CTRL_PWM3_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[18] - PWMIO channel 2 enable
 */
#define PWMIO_CTRL_PWM2_EN_POS                   (18)
#define PWMIO_CTRL_PWM2_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM2_EN_POS)
#define PWMIO_CTRL_PWM2_ENABLE                   (0x1UL << PWMIO_CTRL_PWM2_EN_POS)
#define PWMIO_CTRL_PWM2_DISABLE                  (0x0UL << PWMIO_CTRL_PWM2_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[17] - PWMIO channel 1 enable
 */
#define PWMIO_CTRL_PWM1_EN_POS                   (17)
#define PWMIO_CTRL_PWM1_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM1_EN_POS)
#define PWMIO_CTRL_PWM1_ENABLE                   (0x1UL << PWMIO_CTRL_PWM1_EN_POS)
#define PWMIO_CTRL_PWM1_DISABLE                  (0x0UL << PWMIO_CTRL_PWM1_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[16] - PWMIO channel 0 enable
 */
#define PWMIO_CTRL_PWM0_EN_POS                   (16)
#define PWMIO_CTRL_PWM0_EN_MSK                   (0x1UL << PWMIO_CTRL_PWM0_EN_POS)
#define PWMIO_CTRL_PWM0_ENABLE                   (0x1UL << PWMIO_CTRL_PWM0_EN_POS)
#define PWMIO_CTRL_PWM0_DISABLE                  (0x0UL << PWMIO_CTRL_PWM0_EN_POS)


/*
 * Bit definition for PWMIO_CTRL0[8] - SYNC enable
 */
#define PWMIO_CTRL0_SYNC_EN_POS                  (8)
#define PWMIO_CTRL0_SYNC_EN_MSK                  (0x1UL << PWMIO_CTRL0_SYNC_EN_POS)
#define PWMIO_CTRL0_SYNC_ENABLE                  (0x1UL << PWMIO_CTRL0_SYNC_EN_POS)
#define PWMIO_CTRL0_SYNC_DISABLE                 (0x0UL << PWMIO_CTRL0_SYNC_EN_POS)

/*
 * Bit definition for PWMIO_CTRL0[7:4] - Set the clock source of PWM I/O
 */
#define PWMIO_CTRL_CLK_SEL_POS                   (4)
#define PWMIO_CTRL_CLK_SEL_MSK                   (0xFUL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_FCPU_DIV_4096         (0xEUL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_FCPU_DIV_1024		     (0xDUL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_FCPU_DIV_256          (0xCUL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_FCPU_DIV_64           (0xBUL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_FCPU_DIV_32           (0xAUL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_FCPU_DIV_16           (0x9UL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_FCPU_DIV_8            (0x8UL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_CCP1                  (0x5UL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_CCP0                  (0x4UL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_TIMER2                (0x3UL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_TIMER1                (0x2UL << PWMIO_CTRL_CLK_SEL_POS)
#define PWMIO_CTRL_CLK_SEL_TIMER0                (0x1UL << PWMIO_CTRL_CLK_SEL_POS)

/*
 * Bit definition for PWMIO_CTRL1[31] - PWMIO channel 15 INV
 */
#define PWMIO_CTRL_PWM15_INV_POS                 (31)
#define PWMIO_CTRL_PWM15_INV_MSK                 (0x1UL << PWMIO_CTRL_PWM15_INV_POS)
#define PWMIO_CTRL_PWM15_INV_ON                  (0x1UL << PWMIO_CTRL_PWM15_INV_POS)
#define PWMIO_CTRL_PWM15_INV_OFF                 (0x0UL << PWMIO_CTRL_PWM15_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[30] - PWMIO channel 14 INV
 */
#define PWMIO_CTRL_PWM14_INV_POS                 (30)
#define PWMIO_CTRL_PWM14_INV_MSK                 (0x1UL << PWMIO_CTRL_PWM14_INV_POS)
#define PWMIO_CTRL_PWM14_INV_ON                  (0x1UL << PWMIO_CTRL_PWM14_INV_POS)
#define PWMIO_CTRL_PWM14_INV_OFF                 (0x0UL << PWMIO_CTRL_PWM14_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[29] - PWMIO channel 13 INV
 */
#define PWMIO_CTRL_PWM13_INV_POS                 (29)
#define PWMIO_CTRL_PWM13_INV_MSK                 (0x1UL << PWMIO_CTRL_PWM13_INV_POS)
#define PWMIO_CTRL_PWM13_INV_ON                  (0x1UL << PWMIO_CTRL_PWM13_INV_POS)
#define PWMIO_CTRL_PWM13_INV_OFF						     (0x0UL << PWMIO_CTRL_PWM13_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[28] - PWMIO channel 12 INV
 */
#define PWMIO_CTRL_PWM12_INV_POS                 (28)
#define PWMIO_CTRL_PWM12_INV_MSK                 (0x1UL << PWMIO_CTRL_PWM12_INV_POS)
#define PWMIO_CTRL_PWM12_INV_ON                  (0x1UL << PWMIO_CTRL_PWM12_INV_POS)
#define PWMIO_CTRL_PWM12_INV_OFF						     (0x0UL << PWMIO_CTRL_PWM12_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[27] - PWMIO channel 11 INV
 */
#define PWMIO_CTRL_PWM11_INV_POS                 (27)
#define PWMIO_CTRL_PWM11_INV_MSK                 (0x1UL << PWMIO_CTRL_PWM11_INV_POS)
#define PWMIO_CTRL_PWM11_INV_ON                  (0x1UL << PWMIO_CTRL_PWM11_INV_POS)
#define PWMIO_CTRL_PWM11_INV_OFF						     (0x0UL << PWMIO_CTRL_PWM11_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[26] - PWMIO channel 10 INV
 */
#define PWMIO_CTRL_PWM10_INV_POS                 (26)
#define PWMIO_CTRL_PWM10_INV_MSK                 (0x1UL << PWMIO_CTRL_PWM10_INV_POS)
#define PWMIO_CTRL_PWM10_INV_ON                  (0x1UL << PWMIO_CTRL_PWM10_INV_POS)
#define PWMIO_CTRL_PWM10_INV_OFF						     (0x0UL << PWMIO_CTRL_PWM10_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[25] - PWMIO channel 9 INV
 */
#define PWMIO_CTRL_PWM9_INV_POS                  (25)
#define PWMIO_CTRL_PWM9_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM9_INV_POS)
#define PWMIO_CTRL_PWM9_INV_ON                   (0x1UL << PWMIO_CTRL_PWM9_INV_POS)
#define PWMIO_CTRL_PWM9_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM9_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[24] - PWMIO channel 8 INV
 */
#define PWMIO_CTRL_PWM8_INV_POS                  (24)
#define PWMIO_CTRL_PWM8_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM8_INV_POS)
#define PWMIO_CTRL_PWM8_INV_ON                   (0x1UL << PWMIO_CTRL_PWM8_INV_POS)
#define PWMIO_CTRL_PWM8_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM8_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[23] - PWMIO channel 7 INV
 */
#define PWMIO_CTRL_PWM7_INV_POS                  (23)
#define PWMIO_CTRL_PWM7_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM7_INV_POS)
#define PWMIO_CTRL_PWM7_INV_ON                   (0x1UL << PWMIO_CTRL_PWM7_INV_POS)
#define PWMIO_CTRL_PWM7_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM7_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[22] - PWMIO channel 6 INV
 */
#define PWMIO_CTRL_PWM6_INV_POS                  (22)
#define PWMIO_CTRL_PWM6_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM6_INV_POS)
#define PWMIO_CTRL_PWM6_INV_ON                   (0x1UL << PWMIO_CTRL_PWM6_INV_POS)
#define PWMIO_CTRL_PWM6_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM6_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[21] - PWMIO channel 5 INV
 */
#define PWMIO_CTRL_PWM5_INV_POS                  (21)
#define PWMIO_CTRL_PWM5_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM5_INV_POS)
#define PWMIO_CTRL_PWM5_INV_ON                   (0x1UL << PWMIO_CTRL_PWM5_INV_POS)
#define PWMIO_CTRL_PWM5_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM5_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[20] - PWMIO channel 4 INV
 */
#define PWMIO_CTRL_PWM4_INV_POS                  (20)
#define PWMIO_CTRL_PWM4_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM4_INV_POS)
#define PWMIO_CTRL_PWM4_INV_ON                   (0x1UL << PWMIO_CTRL_PWM4_INV_POS)
#define PWMIO_CTRL_PWM4_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM4_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[19] - PWMIO channel 3 INV
 */
#define PWMIO_CTRL_PWM3_INV_POS                  (19)
#define PWMIO_CTRL_PWM3_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM3_INV_POS)
#define PWMIO_CTRL_PWM3_INV_ON                   (0x1UL << PWMIO_CTRL_PWM3_INV_POS)
#define PWMIO_CTRL_PWM3_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM3_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[18] - PWMIO channel 2 INV
 */
#define PWMIO_CTRL_PWM2_INV_POS                  (18)
#define PWMIO_CTRL_PWM2_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM2_INV_POS)
#define PWMIO_CTRL_PWM2_INV_ON                   (0x1UL << PWMIO_CTRL_PWM2_INV_POS)
#define PWMIO_CTRL_PWM2_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM2_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[17] - PWMIO channel 1 INV
 */
#define PWMIO_CTRL_PWM1_INV_POS                  (17)
#define PWMIO_CTRL_PWM1_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM1_INV_POS)
#define PWMIO_CTRL_PWM1_INV_ON                   (0x1UL << PWMIO_CTRL_PWM1_INV_POS)
#define PWMIO_CTRL_PWM1_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM1_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[16] - PWMIO channel 0 INV
 */
#define PWMIO_CTRL_PWM0_INV_POS                  (16)
#define PWMIO_CTRL_PWM0_INV_MSK                  (0x1UL << PWMIO_CTRL_PWM0_INV_POS)
#define PWMIO_CTRL_PWM0_INV_ON                   (0x1UL << PWMIO_CTRL_PWM0_INV_POS)
#define PWMIO_CTRL_PWM0_INV_OFF                  (0x0UL << PWMIO_CTRL_PWM0_INV_POS)

/*
 * Bit definition for PWMIO_CTRL1[15] - PWMIO channel 15 mask
 */
#define PWMIO_CTRL_PWM15_MSK_POS                 (15)
#define PWMIO_CTRL_PWM15_MSK_MSK                 (0x1UL << PWMIO_CTRL_PWM15_MSK_POS)
#define PWMIO_CTRL_PWM15_MSK_ON                  (0x1UL << PWMIO_CTRL_PWM15_MSK_POS)
#define PWMIO_CTRL_PWM15_MSK_OFF                 (0x0UL << PWMIO_CTRL_PWM15_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[14] - PWMIO channel 14 mask
 */
#define PWMIO_CTRL_PWM14_MSK_POS                 (14)
#define PWMIO_CTRL_PWM14_MSK_MSK                 (0x1UL << PWMIO_CTRL_PWM14_MSK_POS)
#define PWMIO_CTRL_PWM14_MSK_ON                  (0x1UL << PWMIO_CTRL_PWM14_MSK_POS)
#define PWMIO_CTRL_PWM14_MSK_OFF                 (0x0UL << PWMIO_CTRL_PWM14_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[13] - PWMIO channel 13 mask
 */
#define PWMIO_CTRL_PWM13_MSK_POS                 (13)
#define PWMIO_CTRL_PWM13_MSK_MSK                 (0x1UL << PWMIO_CTRL_PWM13_MSK_POS)
#define PWMIO_CTRL_PWM13_MSK_ON                  (0x1UL << PWMIO_CTRL_PWM13_MSK_POS)
#define PWMIO_CTRL_PWM13_MSK_OFF                 (0x0UL << PWMIO_CTRL_PWM13_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[12] - PWMIO channel 12 mask
 */
#define PWMIO_CTRL_PWM12_MSK_POS                 (12)
#define PWMIO_CTRL_PWM12_MSK_MSK                 (0x1UL << PWMIO_CTRL_PWM12_MSK_POS)
#define PWMIO_CTRL_PWM12_MSK_ON                  (0x1UL << PWMIO_CTRL_PWM12_MSK_POS)
#define PWMIO_CTRL_PWM12_MSK_OFF                 (0x0UL << PWMIO_CTRL_PWM12_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[11] - PWMIO channel 11 mask
 */
#define PWMIO_CTRL_PWM11_MSK_POS                 (11)
#define PWMIO_CTRL_PWM11_MSK_MSK                 (0x1UL << PWMIO_CTRL_PWM11_MSK_POS)
#define PWMIO_CTRL_PWM11_MSK_ON                  (0x1UL << PWMIO_CTRL_PWM11_MSK_POS)
#define PWMIO_CTRL_PWM11_MSK_OFF                 (0x0UL << PWMIO_CTRL_PWM11_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[10] - PWMIO channel 10 mask
 */
#define PWMIO_CTRL_PWM10_MSK_POS                 (10)
#define PWMIO_CTRL_PWM10_MSK_MSK                 (0x1UL << PWMIO_CTRL_PWM10_MSK_POS)
#define PWMIO_CTRL_PWM10_MSK_ON                  (0x1UL << PWMIO_CTRL_PWM10_MSK_POS)
#define PWMIO_CTRL_PWM10_MSK_OFF                 (0x0UL << PWMIO_CTRL_PWM10_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[9] - PWMIO channel 9 mask
 */
#define PWMIO_CTRL_PWM9_MSK_POS                  (9)
#define PWMIO_CTRL_PWM9_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM9_MSK_POS)
#define PWMIO_CTRL_PWM9_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM9_MSK_POS)
#define PWMIO_CTRL_PWM9_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM9_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[8] - PWMIO channel 8 mask
 */
#define PWMIO_CTRL_PWM8_MSK_POS                  (8)
#define PWMIO_CTRL_PWM8_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM8_MSK_POS)
#define PWMIO_CTRL_PWM8_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM8_MSK_POS)
#define PWMIO_CTRL_PWM8_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM8_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[7] - PWMIO channel 7 mask
 */
#define PWMIO_CTRL_PWM7_MSK_POS                  (7)
#define PWMIO_CTRL_PWM7_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM7_MSK_POS)
#define PWMIO_CTRL_PWM7_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM7_MSK_POS)
#define PWMIO_CTRL_PWM7_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM7_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[6] - PWMIO channel 6 mask
 */
#define PWMIO_CTRL_PWM6_MSK_POS                  (6)
#define PWMIO_CTRL_PWM6_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM6_MSK_POS)
#define PWMIO_CTRL_PWM6_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM6_MSK_POS)
#define PWMIO_CTRL_PWM6_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM6_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[5] - PWMIO channel 5 mask
 */
#define PWMIO_CTRL_PWM5_MSK_POS                  (5)
#define PWMIO_CTRL_PWM5_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM5_MSK_POS)
#define PWMIO_CTRL_PWM5_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM5_MSK_POS)
#define PWMIO_CTRL_PWM5_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM5_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[4] - PWMIO channel 4 mask
 */
#define PWMIO_CTRL_PWM4_MSK_POS                  (4)
#define PWMIO_CTRL_PWM4_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM4_MSK_POS)
#define PWMIO_CTRL_PWM4_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM4_MSK_POS)
#define PWMIO_CTRL_PWM4_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM4_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[3] - PWMIO channel 3 mask
 */
#define PWMIO_CTRL_PWM3_MSK_POS                  (3)
#define PWMIO_CTRL_PWM3_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM3_MSK_POS)
#define PWMIO_CTRL_PWM3_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM3_MSK_POS)
#define PWMIO_CTRL_PWM3_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM3_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[2] - PWMIO channel 2 mask
 */
#define PWMIO_CTRL_PWM2_MSK_POS                  (2)
#define PWMIO_CTRL_PWM2_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM2_MSK_POS)
#define PWMIO_CTRL_PWM2_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM2_MSK_POS)
#define PWMIO_CTRL_PWM2_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM2_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[1] - PWMIO channel 1 mask
 */
#define PWMIO_CTRL_PWM1_MSK_POS                  (1)
#define PWMIO_CTRL_PWM1_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM1_MSK_POS)
#define PWMIO_CTRL_PWM1_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM1_MSK_POS)
#define PWMIO_CTRL_PWM1_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM1_MSK_POS)

/*
 * Bit definition for PWMIO_CTRL1[0] - PWMIO channel 0 mask
 */
#define PWMIO_CTRL_PWM0_MSK_POS                  (0)
#define PWMIO_CTRL_PWM0_MSK_MSK                  (0x1UL << PWMIO_CTRL_PWM0_MSK_POS)
#define PWMIO_CTRL_PWM0_MSK_ON                   (0x1UL << PWMIO_CTRL_PWM0_MSK_POS)
#define PWMIO_CTRL_PWM0_MSK_OFF                  (0x0UL << PWMIO_CTRL_PWM0_MSK_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[15] - PWMIO Channel 15 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE15_EN_POS               (15)
#define PWMIO_CTRL_TOGGLE15_EN_MSK               (0x1UL << PWMIO_CTRL_TOGGLE15_EN_POS)
#define PWMIO_CTRL_TOGGLE15_ENABLE               (0x1UL << PWMIO_CTRL_TOGGLE15_EN_POS)
#define PWMIO_CTRL_TOGGLE15_DISABLE              (0x0UL << PWMIO_CTRL_TOGGLE15_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[14] - PWMIO Channel 14 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE14_EN_POS               (14)
#define PWMIO_CTRL_TOGGLE14_EN_MSK               (0x1UL << PWMIO_CTRL_TOGGLE14_EN_POS)
#define PWMIO_CTRL_TOGGLE14_ENABLE               (0x1UL << PWMIO_CTRL_TOGGLE14_EN_POS)
#define PWMIO_CTRL_TOGGLE14_DISABLE              (0x0UL << PWMIO_CTRL_TOGGLE14_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[13] - PWMIO Channel 13 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE13_EN_POS               (13)
#define PWMIO_CTRL_TOGGLE13_EN_MSK               (0x1UL << PWMIO_CTRL_TOGGLE13_EN_POS)
#define PWMIO_CTRL_TOGGLE13_ENABLE               (0x1UL << PWMIO_CTRL_TOGGLE13_EN_POS)
#define PWMIO_CTRL_TOGGLE13_DISABLE              (0x0UL << PWMIO_CTRL_TOGGLE13_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[12] - PWMIO Channel 12 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE12_EN_POS               (12)
#define PWMIO_CTRL_TOGGLE12_EN_MSK               (0x1UL << PWMIO_CTRL_TOGGLE12_EN_POS)
#define PWMIO_CTRL_TOGGLE12_ENABLE               (0x1UL << PWMIO_CTRL_TOGGLE12_EN_POS)
#define PWMIO_CTRL_TOGGLE12_DISABLE              (0x0UL << PWMIO_CTRL_TOGGLE12_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[11] - PWMIO Channel 11 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE11_EN_POS               (11)
#define PWMIO_CTRL_TOGGLE11_EN_MSK               (0x1UL << PWMIO_CTRL_TOGGLE11_EN_POS)
#define PWMIO_CTRL_TOGGLE11_ENABLE               (0x1UL << PWMIO_CTRL_TOGGLE11_EN_POS)
#define PWMIO_CTRL_TOGGLE11_DISABLE              (0x0UL << PWMIO_CTRL_TOGGLE11_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[10] - PWMIO Channel 10 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE10_EN_POS               (10)
#define PWMIO_CTRL_TOGGLE10_EN_MSK               (0x1UL << PWMIO_CTRL_TOGGLE10_EN_POS)
#define PWMIO_CTRL_TOGGLE10_ENABLE               (0x1UL << PWMIO_CTRL_TOGGLE10_EN_POS)
#define PWMIO_CTRL_TOGGLE10_DISABLE              (0x0UL << PWMIO_CTRL_TOGGLE10_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[9] - PWMIO Channel 9 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE9_EN_POS                (9)
#define PWMIO_CTRL_TOGGLE9_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE9_EN_POS)
#define PWMIO_CTRL_TOGGLE9_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE9_EN_POS)
#define PWMIO_CTRL_TOGGLE9_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE9_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[8] - PWMIO Channel 8 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE8_EN_POS                (8)
#define PWMIO_CTRL_TOGGLE8_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE8_EN_POS)
#define PWMIO_CTRL_TOGGLE8_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE8_EN_POS)
#define PWMIO_CTRL_TOGGLE8_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE8_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[7] - PWMIO Channel 7 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE7_EN_POS                (7)
#define PWMIO_CTRL_TOGGLE7_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE7_EN_POS)
#define PWMIO_CTRL_TOGGLE7_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE7_EN_POS)
#define PWMIO_CTRL_TOGGLE7_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE7_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[6] - PWMIO Channel 6 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE6_EN_POS                (6)
#define PWMIO_CTRL_TOGGLE6_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE6_EN_POS)
#define PWMIO_CTRL_TOGGLE6_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE6_EN_POS)
#define PWMIO_CTRL_TOGGLE6_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE6_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[5] - PWMIO Channel 5 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE5_EN_POS                (5)
#define PWMIO_CTRL_TOGGLE5_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE5_EN_POS)
#define PWMIO_CTRL_TOGGLE5_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE5_EN_POS)
#define PWMIO_CTRL_TOGGLE5_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE5_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[4] - PWMIO Channel 4 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE4_EN_POS                (4)
#define PWMIO_CTRL_TOGGLE4_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE4_EN_POS)
#define PWMIO_CTRL_TOGGLE4_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE4_EN_POS)
#define PWMIO_CTRL_TOGGLE4_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE4_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[3] - PWMIO Channel 3 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE3_EN_POS                (3)
#define PWMIO_CTRL_TOGGLE3_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE3_EN_POS)
#define PWMIO_CTRL_TOGGLE3_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE3_EN_POS)
#define PWMIO_CTRL_TOGGLE3_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE3_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[2] - PWMIO Channel 2 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE2_EN_POS                (2)
#define PWMIO_CTRL_TOGGLE2_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE2_EN_POS)
#define PWMIO_CTRL_TOGGLE2_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE2_EN_POS)
#define PWMIO_CTRL_TOGGLE2_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE2_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[1] - PWMIO Channel 1 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE1_EN_POS                (1)
#define PWMIO_CTRL_TOGGLE1_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE1_EN_POS)
#define PWMIO_CTRL_TOGGLE1_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE1_EN_POS)
#define PWMIO_CTRL_TOGGLE1_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE1_EN_POS)

/*
 * Bit definition for PWMIO_TOGGLE_CTRL[0] - PWMIO Channel 0 Toggle Control enable
 */
#define PWMIO_CTRL_TOGGLE0_EN_POS                (0)
#define PWMIO_CTRL_TOGGLE0_EN_MSK                (0x1UL << PWMIO_CTRL_TOGGLE0_EN_POS)
#define PWMIO_CTRL_TOGGLE0_ENABLE                (0x1UL << PWMIO_CTRL_TOGGLE0_EN_POS)
#define PWMIO_CTRL_TOGGLE0_DISABLE               (0x0UL << PWMIO_CTRL_TOGGLE0_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[31:30] - PWMIO Period Control 15
 */
#define PWMIO_CTRL_PERIOD15_EN_POS               (30)
#define PWMIO_CTRL_PERIOD15_EN_MSK               (0x3UL << PWMIO_CTRL_PERIOD15_EN_POS)
#define PWMIO_CTRL_PERIOD15_32                   (0x3UL << PWMIO_CTRL_PERIOD15_EN_POS)
#define PWMIO_CTRL_PERIOD15_64                   (0x2UL << PWMIO_CTRL_PERIOD15_EN_POS)
#define PWMIO_CTRL_PERIOD15_128                  (0x1UL << PWMIO_CTRL_PERIOD15_EN_POS)
#define PWMIO_CTRL_PERIOD15_256                  (0x0UL << PWMIO_CTRL_PERIOD15_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[29:28] - PWMIO Period Control 14
 */
#define PWMIO_CTRL_PERIOD14_EN_POS               (28)
#define PWMIO_CTRL_PERIOD14_EN_MSK               (0x3UL << PWMIO_CTRL_PERIOD14_EN_POS)
#define PWMIO_CTRL_PERIOD14_32                   (0x3UL << PWMIO_CTRL_PERIOD14_EN_POS)
#define PWMIO_CTRL_PERIOD14_64                   (0x2UL << PWMIO_CTRL_PERIOD14_EN_POS)
#define PWMIO_CTRL_PERIOD14_128                  (0x1UL << PWMIO_CTRL_PERIOD14_EN_POS)
#define PWMIO_CTRL_PERIOD14_256                  (0x0UL << PWMIO_CTRL_PERIOD14_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[27:26] - PWMIO Period Control 13
 */
#define PWMIO_CTRL_PERIOD13_EN_POS               (26)
#define PWMIO_CTRL_PERIOD13_EN_MSK               (0x3UL << PWMIO_CTRL_PERIOD13_EN_POS)
#define PWMIO_CTRL_PERIOD13_32                   (0x3UL << PWMIO_CTRL_PERIOD13_EN_POS)
#define PWMIO_CTRL_PERIOD13_64                   (0x2UL << PWMIO_CTRL_PERIOD13_EN_POS)
#define PWMIO_CTRL_PERIOD13_128                  (0x1UL << PWMIO_CTRL_PERIOD13_EN_POS)
#define PWMIO_CTRL_PERIOD13_256                  (0x0UL << PWMIO_CTRL_PERIOD13_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[25:24] - PWMIO Period Control 12
 */
#define PWMIO_CTRL_PERIOD12_EN_POS               (24)
#define PWMIO_CTRL_PERIOD12_EN_MSK               (0x3UL << PWMIO_CTRL_PERIOD12_EN_POS)
#define PWMIO_CTRL_PERIOD12_32                   (0x3UL << PWMIO_CTRL_PERIOD12_EN_POS)
#define PWMIO_CTRL_PERIOD12_64                   (0x2UL << PWMIO_CTRL_PERIOD12_EN_POS)
#define PWMIO_CTRL_PERIOD12_128                  (0x1UL << PWMIO_CTRL_PERIOD12_EN_POS)
#define PWMIO_CTRL_PERIOD12_256                  (0x0UL << PWMIO_CTRL_PERIOD12_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[23:22] - PWMIO Period Control 11
 */
#define PWMIO_CTRL_PERIOD11_EN_POS               (22)
#define PWMIO_CTRL_PERIOD11_EN_MSK               (0x3UL << PWMIO_CTRL_PERIOD11_EN_POS)
#define PWMIO_CTRL_PERIOD11_32                   (0x3UL << PWMIO_CTRL_PERIOD11_EN_POS)
#define PWMIO_CTRL_PERIOD11_64                   (0x2UL << PWMIO_CTRL_PERIOD11_EN_POS)
#define PWMIO_CTRL_PERIOD11_128                  (0x1UL << PWMIO_CTRL_PERIOD11_EN_POS)
#define PWMIO_CTRL_PERIOD11_256                  (0x0UL << PWMIO_CTRL_PERIOD11_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[21:20] - PWMIO Period Control 10
 */
#define PWMIO_CTRL_PERIOD10_EN_POS               (20)
#define PWMIO_CTRL_PERIOD10_EN_MSK               (0x3UL << PWMIO_CTRL_PERIOD10_EN_POS)
#define PWMIO_CTRL_PERIOD10_32                   (0x3UL << PWMIO_CTRL_PERIOD10_EN_POS)
#define PWMIO_CTRL_PERIOD10_64                   (0x2UL << PWMIO_CTRL_PERIOD10_EN_POS)
#define PWMIO_CTRL_PERIOD10_128                  (0x1UL << PWMIO_CTRL_PERIOD10_EN_POS)
#define PWMIO_CTRL_PERIOD10_256                  (0x0UL << PWMIO_CTRL_PERIOD10_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[19:18] - PWMIO Period Control 9
 */
#define PWMIO_CTRL_PERIOD9_EN_POS                (18)
#define PWMIO_CTRL_PERIOD9_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD9_EN_POS)
#define PWMIO_CTRL_PERIOD9_32                    (0x3UL << PWMIO_CTRL_PERIOD9_EN_POS)
#define PWMIO_CTRL_PERIOD9_64                    (0x2UL << PWMIO_CTRL_PERIOD9_EN_POS)
#define PWMIO_CTRL_PERIOD9_128                   (0x1UL << PWMIO_CTRL_PERIOD9_EN_POS)
#define PWMIO_CTRL_PERIOD9_256                   (0x0UL << PWMIO_CTRL_PERIOD9_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[17:16] - PWMIO Period Control 8
 */
#define PWMIO_CTRL_PERIOD8_EN_POS                (16)
#define PWMIO_CTRL_PERIOD8_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD8_EN_POS)
#define PWMIO_CTRL_PERIOD8_32                    (0x3UL << PWMIO_CTRL_PERIOD8_EN_POS)
#define PWMIO_CTRL_PERIOD8_64                    (0x2UL << PWMIO_CTRL_PERIOD8_EN_POS)
#define PWMIO_CTRL_PERIOD8_128                   (0x1UL << PWMIO_CTRL_PERIOD8_EN_POS)
#define PWMIO_CTRL_PERIOD8_256                   (0x0UL << PWMIO_CTRL_PERIOD8_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[15:14] - PWMIO Period Control 7
 */
#define PWMIO_CTRL_PERIOD7_EN_POS                (14)
#define PWMIO_CTRL_PERIOD7_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD7_EN_POS)
#define PWMIO_CTRL_PERIOD7_32                    (0x3UL << PWMIO_CTRL_PERIOD7_EN_POS)
#define PWMIO_CTRL_PERIOD7_64                    (0x2UL << PWMIO_CTRL_PERIOD7_EN_POS)
#define PWMIO_CTRL_PERIOD7_128                   (0x1UL << PWMIO_CTRL_PERIOD7_EN_POS)
#define PWMIO_CTRL_PERIOD7_256                   (0x0UL << PWMIO_CTRL_PERIOD7_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[13:12] - PWMIO Period Control 6
 */
#define PWMIO_CTRL_PERIOD6_EN_POS                (12)
#define PWMIO_CTRL_PERIOD6_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD6_EN_POS)
#define PWMIO_CTRL_PERIOD6_32                    (0x3UL << PWMIO_CTRL_PERIOD6_EN_POS)
#define PWMIO_CTRL_PERIOD6_64                    (0x2UL << PWMIO_CTRL_PERIOD6_EN_POS)
#define PWMIO_CTRL_PERIOD6_128                   (0x1UL << PWMIO_CTRL_PERIOD6_EN_POS)
#define PWMIO_CTRL_PERIOD6_256                   (0x0UL << PWMIO_CTRL_PERIOD6_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[11:10] - PWMIO Period Control 5
 */
#define PWMIO_CTRL_PERIOD5_EN_POS                (10)
#define PWMIO_CTRL_PERIOD5_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD5_EN_POS)
#define PWMIO_CTRL_PERIOD5_32                    (0x3UL << PWMIO_CTRL_PERIOD5_EN_POS)
#define PWMIO_CTRL_PERIOD5_64                    (0x2UL << PWMIO_CTRL_PERIOD5_EN_POS)
#define PWMIO_CTRL_PERIOD5_128                   (0x1UL << PWMIO_CTRL_PERIOD5_EN_POS)
#define PWMIO_CTRL_PERIOD5_256                   (0x0UL << PWMIO_CTRL_PERIOD5_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[9:8] - PWMIO Period Control 4
 */
#define PWMIO_CTRL_PERIOD4_EN_POS                (8)
#define PWMIO_CTRL_PERIOD4_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD4_EN_POS)
#define PWMIO_CTRL_PERIOD4_32                    (0x3UL << PWMIO_CTRL_PERIOD4_EN_POS)
#define PWMIO_CTRL_PERIOD4_64                    (0x2UL << PWMIO_CTRL_PERIOD4_EN_POS)
#define PWMIO_CTRL_PERIOD4_128                   (0x1UL << PWMIO_CTRL_PERIOD4_EN_POS)
#define PWMIO_CTRL_PERIOD4_256                   (0x0UL << PWMIO_CTRL_PERIOD4_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[7:6] - PWMIO Period Control 3
 */
#define PWMIO_CTRL_PERIOD3_EN_POS                (6)
#define PWMIO_CTRL_PERIOD3_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD3_EN_POS)
#define PWMIO_CTRL_PERIOD3_32                    (0x3UL << PWMIO_CTRL_PERIOD3_EN_POS)
#define PWMIO_CTRL_PERIOD3_64                    (0x2UL << PWMIO_CTRL_PERIOD3_EN_POS)
#define PWMIO_CTRL_PERIOD3_128                   (0x1UL << PWMIO_CTRL_PERIOD3_EN_POS)
#define PWMIO_CTRL_PERIOD3_256                   (0x0UL << PWMIO_CTRL_PERIOD3_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[5:4] - PWMIO Period Control 2
 */
#define PWMIO_CTRL_PERIOD2_EN_POS                (4)
#define PWMIO_CTRL_PERIOD2_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD2_EN_POS)
#define PWMIO_CTRL_PERIOD2_32                    (0x3UL << PWMIO_CTRL_PERIOD2_EN_POS)
#define PWMIO_CTRL_PERIOD2_64                    (0x2UL << PWMIO_CTRL_PERIOD2_EN_POS)
#define PWMIO_CTRL_PERIOD2_128                   (0x1UL << PWMIO_CTRL_PERIOD2_EN_POS)
#define PWMIO_CTRL_PERIOD2_256                   (0x0UL << PWMIO_CTRL_PERIOD2_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[3:2] - PWMIO Period Control 1
 */
#define PWMIO_CTRL_PERIOD1_EN_POS                (2)
#define PWMIO_CTRL_PERIOD1_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD1_EN_POS)
#define PWMIO_CTRL_PERIOD1_32                    (0x3UL << PWMIO_CTRL_PERIOD1_EN_POS)
#define PWMIO_CTRL_PERIOD1_64                    (0x2UL << PWMIO_CTRL_PERIOD1_EN_POS)
#define PWMIO_CTRL_PERIOD1_128                   (0x1UL << PWMIO_CTRL_PERIOD1_EN_POS)
#define PWMIO_CTRL_PERIOD1_256                   (0x0UL << PWMIO_CTRL_PERIOD1_EN_POS)

/*
 * Bit definition for PWMIO_PERIOD_CTRL[1:0] - PWMIO Period Control 0
 */
#define PWMIO_CTRL_PERIOD0_EN_POS                (0)
#define PWMIO_CTRL_PERIOD0_EN_MSK                (0x3UL << PWMIO_CTRL_PERIOD0_EN_POS)
#define PWMIO_CTRL_PERIOD0_32                    (0x3UL << PWMIO_CTRL_PERIOD0_EN_POS)
#define PWMIO_CTRL_PERIOD0_64                    (0x2UL << PWMIO_CTRL_PERIOD0_EN_POS)
#define PWMIO_CTRL_PERIOD0_128                   (0x1UL << PWMIO_CTRL_PERIOD0_EN_POS)
#define PWMIO_CTRL_PERIOD0_256                   (0x0UL << PWMIO_CTRL_PERIOD0_EN_POS)

/*
 * Bit definition for PWMIO_OFFSET[15:14] - PWMIO Period Offset Control
 */
#define PWMIO_OFFSET_PWM14_15_POS                (14)
#define PWMIO_OFFSET_PWM14_15_MSK                (0x3UL << PWMIO_OFFSET_PWM14_15_POS)
#define PWMIO_OFFSET_PWM14_15_DELAY3             (0x3UL << PWMIO_OFFSET_PWM14_15_POS)
#define PWMIO_OFFSET_PWM14_15_DELAY2             (0x2UL << PWMIO_OFFSET_PWM14_15_POS)
#define PWMIO_OFFSET_PWM14_15_DELAY1             (0x1UL << PWMIO_OFFSET_PWM14_15_POS)
#define PWMIO_OFFSET_PWM14_15_NODELAY            (0x0UL << PWMIO_OFFSET_PWM14_15_POS)

/*
 * Bit definition for PWMIO_OFFSET[13:12] - PWMIO Period Offset Control
 */
#define PWMIO_OFFSET_PWM12_13_POS                (12)
#define PWMIO_OFFSET_PWM12_13_MSK                (0x3UL << PWMIO_OFFSET_PWM12_13_POS)
#define PWMIO_OFFSET_PWM12_13_DELAY3             (0x3UL << PWMIO_OFFSET_PWM12_13_POS)
#define PWMIO_OFFSET_PWM12_13_DELAY2             (0x2UL << PWMIO_OFFSET_PWM12_13_POS)
#define PWMIO_OFFSET_PWM12_13_DELAY1             (0x1UL << PWMIO_OFFSET_PWM12_13_POS)
#define PWMIO_OFFSET_PWM12_13_NODELAY            (0x0UL << PWMIO_OFFSET_PWM12_13_POS)

/*
 * Bit definition for PWMIO_OFFSET[11:10] - PWMIO Period Offset Control
 */
#define PWMIO_OFFSET_PWM10_11_POS                (10)
#define PWMIO_OFFSET_PWM10_11_MSK                (0x3UL << PWMIO_OFFSET_PWM10_11_POS)
#define PWMIO_OFFSET_PWM10_11_DELAY3             (0x3UL << PWMIO_OFFSET_PWM10_11_POS)
#define PWMIO_OFFSET_PWM10_11_DELAY2             (0x2UL << PWMIO_OFFSET_PWM10_11_POS)
#define PWMIO_OFFSET_PWM10_11_DELAY1             (0x1UL << PWMIO_OFFSET_PWM10_11_POS)
#define PWMIO_OFFSET_PWM10_11_NODELAY            (0x0UL << PWMIO_OFFSET_PWM10_11_POS)

/*
 * Bit definition for PWMIO_OFFSET[9:8] - PWMIO Period Offset Control
 */
#define PWMIO_OFFSET_PWM8_9_POS                  (8)
#define PWMIO_OFFSET_PWM8_9_MSK                  (0x3UL << PWMIO_OFFSET_PWM8_9_POS)
#define PWMIO_OFFSET_PWM8_9_DELAY3               (0x3UL << PWMIO_OFFSET_PWM8_9_POS)
#define PWMIO_OFFSET_PWM8_9_DELAY2               (0x2UL << PWMIO_OFFSET_PWM8_9_POS)
#define PWMIO_OFFSET_PWM8_9_DELAY1               (0x1UL << PWMIO_OFFSET_PWM8_9_POS)
#define PWMIO_OFFSET_PWM8_9_NODELAY              (0x0UL << PWMIO_OFFSET_PWM8_9_POS)

/*
 * Bit definition for PWMIO_OFFSET[7:6] - PWMIO Period Offset Control
 */
#define PWMIO_OFFSET_PWM6_7_POS                  (6)
#define PWMIO_OFFSET_PWM6_7_MSK                  (0x3UL << PWMIO_OFFSET_PWM6_7_POS)
#define PWMIO_OFFSET_PWM6_7_DELAY3               (0x3UL << PWMIO_OFFSET_PWM6_7_POS)
#define PWMIO_OFFSET_PWM6_7_DELAY2               (0x2UL << PWMIO_OFFSET_PWM6_7_POS
#define PWMIO_OFFSET_PWM6_7_DELAY1               (0x1UL << PWMIO_OFFSET_PWM6_7_POS)
#define PWMIO_OFFSET_PWM6_7_NODELAY              (0x0UL << PWMIO_OFFSET_PWM6_7_POS)

/*
 * Bit definition for PWMIO_OFFSET[5:4] - PWMIO Period Offset Control
 */
#define PWMIO_OFFSET_PWM4_5_POS                  (4)
#define PWMIO_OFFSET_PWM4_5_MSK                  (0x3UL << PWMIO_OFFSET_PWM4_5_POS)
#define PWMIO_OFFSET_PWM4_5_DELAY3               (0x3UL << PWMIO_OFFSET_PWM4_5_POS)
#define PWMIO_OFFSET_PWM4_5_DELAY2               (0x2UL << PWMIO_OFFSET_PWM4_5_POS)
#define PWMIO_OFFSET_PWM4_5_DELAY1               (0x1UL << PWMIO_OFFSET_PWM4_5_POS)
#define PWMIO_OFFSET_PWM4_5_NODELAY              (0x0UL << PWMIO_OFFSET_PWM4_5_POS)

/*
 * Bit definition for PWMIO_OFFSET[3:2] - PWMIO Period Offset Control
 */
#define PWMIO_OFFSET_PWM2_3_POS                  (2)
#define PWMIO_OFFSET_PWM2_3_MSK                  (0x3UL << PWMIO_OFFSET_PWM2_3_POS)
#define PWMIO_OFFSET_PWM2_3_DELAY3               (0x3UL << PWMIO_OFFSET_PWM2_3_POS)
#define PWMIO_OFFSET_PWM2_3_DELAY2               (0x2UL << PWMIO_OFFSET_PWM2_3_POS)
#define PWMIO_OFFSET_PWM2_3_DELAY1               (0x1UL << PWMIO_OFFSET_PWM2_3_POS)
#define PWMIO_OFFSET_PWM2_3_NODELAY              (0x0UL << PWMIO_OFFSET_PWM2_3_POS)

/*
 * Bit definition for PWMIO_OFFSET[1:0] - PWMIO Period Offset Control
 */
#define PWMIO_OFFSET_PWM0_1_POS                  (0)
#define PWMIO_OFFSET_PWM0_1_MSK                  (0x3UL << PWMIO_OFFSET_PWM0_1_POS)
#define PWMIO_OFFSET_PWM0_1_DELAY3               (0x3UL << PWMIO_OFFSET_PWM0_1_POS)
#define PWMIO_OFFSET_PWM0_1_DELAY2               (0x2UL << PWMIO_OFFSET_PWM0_1_POS)
#define PWMIO_OFFSET_PWM0_1_DELAY1               (0x1UL << PWMIO_OFFSET_PWM0_1_POS)
#define PWMIO_OFFSET_PWM0_1_NODELAY              (0x0UL << PWMIO_OFFSET_PWM0_1_POS)

/*---------------------------------------------------------------------------------------
 * Quadrature Decoder Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
 typedef struct
{
    __IO uint32_t QD0_CNT;                       // QD0 Counter Register
    __IO uint32_t QD1_CNT;                       // QD1 Counter Register
    __IO uint32_t QD0_CLR;                       // QD0 Clear Register
    __IO uint32_t QD1_CLR;                       // QD1 Clear Register
    __IO uint32_t CTRL;                          // QD Control Register
    __IO uint32_t STS;                           // QD Status Register
    __IO uint32_t FW_TH;                         // QD Forward counter Threshold
    __IO uint32_t BW_TH;                         // QD Backward counter Threshold
} QD_TYPE_DEF;

/*
 * Bit definition for QD_CTRL[11] - QD1 motor reverse interrupt enable
 */
#define QD_CTRL_QD1_REV_EN_POS                   (11)
#define QD_CTRL_QD1_REV_EN_MSK                   (0x1UL << QD_CTRL_QD1_REV_EN_POS)
#define QD_CTRL_QD1_REV_ENABLE                   (0x1UL << QD_CTRL_QD1_REV_EN_POS)
#define QD_CTRL_QD1_REV_DISABLE                  (0x0UL << QD_CTRL_QD1_REV_EN_POS)

/*
 * Bit definition for QD_CTRL[10] - QD1 motor positive interrupt enable
 */
#define QD_CTRL_QD1_POS_EN_POS                   (10)
#define QD_CTRL_QD1_POS_EN_MSK                   (0x1UL << QD_CTRL_QD1_POS_EN_POS)
#define QD_CTRL_QD1_POS_ENABLE                   (0x1UL << QD_CTRL_QD1_POS_EN_POS)
#define QD_CTRL_QD1_POS_DISABLE                  (0x0UL << QD_CTRL_QD1_POS_EN_POS)

/*
 * Bit definition for QD_CTRL[9] - QD0 motor reverse interrupt enable
 */
#define QD_CTRL_QD0_REV_EN_POS                   (9)
#define QD_CTRL_QD0_REV_EN_MSK                   (0x1UL << QD_CTRL_QD0_REV_EN_POS)
#define QD_CTRL_QD0_REV_ENABLE                   (0x1UL << QD_CTRL_QD0_REV_EN_POS)
#define QD_CTRL_QD0_REV_DISABLE                  (0x0UL << QD_CTRL_QD0_REV_EN_POS)

/*
 * Bit definition for QD_CTRL[8] - QD0 motor positive interrupt enable
 */
#define QD_CTRL_QD0_POS_EN_POS                   (8)
#define QD_CTRL_QD0_POS_EN_MSK                   (0x1UL << QD_CTRL_QD0_POS_EN_POS)
#define QD_CTRL_QD0_POS_ENABLE                   (0x1UL << QD_CTRL_QD0_POS_EN_POS)
#define QD_CTRL_QD0_POS_DISABLE                  (0x0UL << QD_CTRL_QD0_POS_EN_POS)

/*
 * Bit definition for QD_CTRL[7] - QD overflow mode2
 */
#define QD_CTRL_OVERFLOW_MODE2_POS               (7)
#define QD_CTRL_OVERFLOW_MODE2_MSK               (0x1UL << QD_CTRL_OVERFLOW_MODE2_POS)
#define QD_CTRL_OVERFLOW_MODE2_ENABLE            (0x1UL << QD_CTRL_OVERFLOW_MODE2_POS)
#define QD_CTRL_OVERFLOW_MODE2_DISABLE           (0x0UL << QD_CTRL_OVERFLOW_MODE2_POS)

/*
* Bit definition for QD_CTRL[6:4] - QD input debounce select
 */
#define QD_CTRL_DEBOUNCE_SEL_POS                 (4)
#define QD_CTRL_DEBOUNCE_SEL_MSK                 (0x7UL << QD_CTRL_DEBOUNCE_SEL_POS)
#define QD_CTRL_DEBOUNCE_SEL_0T                  (0x0UL << QD_CTRL_DEBOUNCE_SEL_POS)
#define QD_CTRL_DEBOUNCE_SEL_4T                  (0x1UL << QD_CTRL_DEBOUNCE_SEL_POS)
#define QD_CTRL_DEBOUNCE_SEL_8T                  (0x2UL << QD_CTRL_DEBOUNCE_SEL_POS)
#define QD_CTRL_DEBOUNCE_SEL_16T                 (0x3UL << QD_CTRL_DEBOUNCE_SEL_POS)
#define QD_CTRL_DEBOUNCE_SEL_32T                 (0x4UL << QD_CTRL_DEBOUNCE_SEL_POS)
#define QD_CTRL_DEBOUNCE_SEL_40T                 (0x5UL << QD_CTRL_DEBOUNCE_SEL_POS)
#define QD_CTRL_DEBOUNCE_SEL_80T                 (0x6UL << QD_CTRL_DEBOUNCE_SEL_POS)
#define QD_CTRL_DEBOUNCE_SEL_128T                (0x7UL << QD_CTRL_DEBOUNCE_SEL_POS)

/*
 * Bit definition for QD_CTRL[3] - QD1 control bit
 */
#define QD_CTRL_QD1_EN_POS                       (3)
#define QD_CTRL_QD1_EN_MSK                       (0x1UL << QD_CTRL_QD1_EN_POS)
#define QD_CTRL_QD1_ENABLE                       (0x1UL << QD_CTRL_QD1_EN_POS)
#define QD_CTRL_QD1_DISABLE                      (0x0UL << QD_CTRL_QD1_EN_POS)

/*
 * Bit definition for QD_CTRL[2] - QD1 interrupt mode
 */
#define QD_CTRL_QD1_INT_MODE_POS                 (2)
#define QD_CTRL_QD1_INT_MODE_MSK                 (0x1UL << QD_CTRL_QD1_INT_MODE_POS)
#define QD_CTRL_QD1_OVERFLOW_MODE                (0x1UL << QD_CTRL_QD1_INT_MODE_POS)
#define QD_CTRL_QD1_ALL_MODE                     (0x0UL << QD_CTRL_QD1_INT_MODE_POS)

/*
 * Bit definition for QD_CTRL[1] - QD0 control bit
 */
#define QD_CTRL_QD0_EN_POS                       (1)
#define QD_CTRL_QD0_EN_MSK                       (0x1UL << QD_CTRL_QD0_EN_POS)
#define QD_CTRL_QD0_ENABLE                       (0x1UL << QD_CTRL_QD0_EN_POS)
#define QD_CTRL_QD0_DISABLE                      (0x0UL << QD_CTRL_QD0_EN_POS)

/*
 * Bit definition for QD_CTRL[0] - QD0 interrupt mode
 */
#define QD_CTRL_QD0_INT_MODE_POS                 (0)
#define QD_CTRL_QD0_INT_MODE_MSK                 (0x1UL << QD_CTRL_QD0_INT_MODE_POS)
#define QD_CTRL_QD0_OVERFLOW_MODE                (0x1UL << QD_CTRL_QD0_INT_MODE_POS)
#define QD_CTRL_QD0_ALL_MODE                     (0x0UL << QD_CTRL_QD0_INT_MODE_POS)

/*
 * Bit definition for QD_STS[11] - QD1 motor reverse interrupt flag
 */
#define QD_CTRL_QD1_REV_INTF_POS                 (11)
#define QD_CTRL_QD1_REV_INTF_MSK                 (0x1UL << QD_CTRL_QD1_REV_INTF_POS)
#define QD_CTRL_QD1_REV_INT_FLAG                 (0x1UL << QD_CTRL_QD1_REV_INTF_POS)

/*
 * Bit definition for QD_STS[10] - QD1 motor positive interrupt flag
 */
#define QD_CTRL_QD1_POS_INTF_POS                 (10)
#define QD_CTRL_QD1_POS_INTF_MSK                 (0x1UL << QD_CTRL_QD1_POS_INTF_POS)
#define QD_CTRL_QD1_POS_INT_FLAG                 (0x1UL << QD_CTRL_QD1_POS_INTF_POS)

/*
 * Bit definition for QD_STS[9] - QD0 motor reverse interrupt flag
 */
#define QD_CTRL_QD0_REV_INTF_POS                 (9)
#define QD_CTRL_QD0_REV_INTF_MSK                 (0x1UL << QD_CTRL_QD0_REV_INTF_POS)
#define QD_CTRL_QD0_REV_INT_FLAG                 (0x1UL << QD_CTRL_QD0_REV_INTF_POS)

/*
 * Bit definition for QD_STS[8] - QD0 motor positive interrupt flag
 */
#define QD_CTRL_QD0_POS_INTF_POS                 (8)
#define QD_CTRL_QD0_POS_INTF_MSK                 (0x1UL << QD_CTRL_QD0_POS_INTF_POS)
#define QD_CTRL_QD0_POS_INT_FLAG                 (0x1UL << QD_CTRL_QD0_POS_INTF_POS)


/*---------------------------------------------------------------------------------------
 * System Management Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t SYSLOCK;                       // System Control Signal Lock Register
    __IO uint32_t PID;                           // Produce ID Register
    __I  uint32_t RESERVED0[3];
    __IO uint32_t LIB_CERTIF;                    // User Defined ID 2 Mirror Register
    __IO uint32_t ID_ALGORITHM;
    __IO uint32_t DUMMY_OPTION;
    __IO uint32_t MODE_FLAG;
    __IO uint32_t OPTION_0;
    __I  uint32_t RESERVED1[2];
    __IO uint32_t CACHE_CTRL;                    // Cache control
    __I  uint32_t RESERVED2[7];
    __IO uint32_t EFUSE_A;
    __IO uint32_t EFUSE_B;
    __IO uint32_t SHUTDOWN_ON;                   // Shut Down Enable Control
    __IO uint32_t SHUTDOWN_OFF;                  // Release from shut down mode
    __IO uint32_t SHUTDOWN_CTRL;                 // Shut Down Control
} SMU_TYPE_DEF;

/*
 * Bit definition for SMU_SYSLOCK[8]- System UnLock Status
 */
#define SMU_SYSLOCK_UNLOCK_STS_POS               (8)
#define SMU_SYSLOCK_UNLOCK_STS_MSK               (0x1UL << SMU_SYSLOCK_UNLOCK_STS_POS)
#define SMU_SYSLOCK_UNLOCK_STS_FLAG              (0x1UL << SMU_SYSLOCK_UNLOCK_STS_POS)

/*
 * Bit definition for SMU_SYSLOCK[7:0]- System UnLock Key
 */
#define SMU_SYSLOCK_UNLOCK_KEY_POS               (0)
#define SMU_SYSLOCK_UNLOCK_KEY_MSK               (0xFFUL << SMU_SYSLOCK_UNLOCK_KEY_POS)
#define SMU_SYSLOCK_UNLOCK_KEY1                  (0xABUL << SMU_SYSLOCK_UNLOCK_KEY_POS)
#define SMU_SYSLOCK_UNLOCK_KEY2                  (0x12UL << SMU_SYSLOCK_UNLOCK_KEY_POS)

/*
 * Bit definition for SMU_CACHE_CTRL[0]- Cache Enable
 */
#define SMU_CACHE_CTRL_CACHE_EN_POS              (0)
#define SMU_CACHE_CTRL_CACHE_EN_MSK              (0x1UL << SMU_CACHE_CTRL_CACHE_EN_POS)
#define SMU_CACHE_CTRL_CACHE_ENABLE              (0x1UL << SMU_CACHE_CTRL_CACHE_EN_POS)
#define SMU_CACHE_CTRL_CACHE_DISABLE             (0x0UL << SMU_CACHE_CTRL_CACHE_EN_POS)

/*
 * Bit definition for SMU_SHUTDOWN_ON[7:0]- Shut down mode enable
 */
#define SMU_SHUTDOWN_ON_SD_ON_POS                (0)
#define SMU_SHUTDOWN_ON_SD_ON_MSK                (0xFFUL << SMU_SHUTDOWN_ON_SD_ON_POS)
#define SMU_SHUTDOWN_ON_SD_ON                    (0x55UL << SMU_SHUTDOWN_ON_SD_ON_POS)

/*
 * Bit definition for SMU_SHUTDOWN_OFF[7:0]- Release from shut down mode
 */
#define SMU_SHUTDOWN_OFF_SD_OFF_POS              (0)
#define SMU_SHUTDOWN_OFF_SD_OFF_MSK              (0xFFUL << SMU_SHUTDOWN_OFF_SD_OFF_POS)
#define SMU_SHUTDOWN_OFF_SD_OFF                  (0xAAUL << SMU_SHUTDOWN_OFF_SD_OFF_POS)

/*
 * Bit definition for SMU_SHUTDOWN_CTRL[0]- Wakeup source in shut down mode
 */
#define SMU_SHUTDOWN_CTRL_WK_SRC_POS             (0)
#define SMU_SHUTDOWN_CTRL_WK_SRC_MSK             (0x1UL << SMU_SHUTDOWN_CTRL_WK_SRC_POS)
#define SMU_SHUTDOWN_CTRL_WK_SRC_I16K            (0x1UL << SMU_SHUTDOWN_CTRL_WK_SRC_POS)
#define SMU_SHUTDOWN_CTRL_WK_SRC_KEY             (0x0UL << SMU_SHUTDOWN_CTRL_WK_SRC_POS)

/*
 * Bit definition for SMU_SHUTDOWN_CTRL[31]- shut down mode flag
 */
#define SMU_SHUTDOWN_CTRL_SD_FLAG_POS            (31)
#define SMU_SHUTDOWN_CTRL_SD_FLAG_MSK            (0x1UL << SMU_SHUTDOWN_CTRL_SD_FLAG_POS)
#define SMU_SHUTDOWN_CTRL_SD_FLAG                (0x1UL << SMU_SHUTDOWN_CTRL_SD_FLAG_POS)


/*---------------------------------------------------------------------------------------
 * eFuse & Body Option Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t BODYOPTLOCK;                   // Body Option Lock Register 0x5000_5040
    __IO uint32_t BODYOPT_0;                     // 0x5000_5044
    __IO uint32_t BODYOPT_1;                     // 0x5000_5048
    __IO uint32_t BODYOPT_2;                     // 0x5000_504C
    __IO uint32_t BODYOPT_3;                     // 0x5000_5050
    __IO uint32_t BODYOPT_4;                     // 0x5000_5054
    __IO uint32_t BODYOPT_5;                     // 0x5000_5058
    __IO uint32_t BODYOPT_6;                     // 0x5000_505C
    __IO uint32_t BODYOPT_7;                     // 0x5000_5060
} BODYOPT_TYPE_DEF;

#define BODYOPT_OPTLOCK_UNLOCK_KEY_POS           (0)
#define BODYOPT_OPTLOCK_UNLOCK_KEY_MSK           (0xFFUL << SMU_SYSLOCK_UNLOCK_KEY_POS)
#define BODYOPT_OPTLOCK_UNLOCK_KEY1              (0x87UL << SMU_SYSLOCK_UNLOCK_KEY_POS)
#define BODYOPT_OPTLOCK_UNLOCK_KEY2              (0x9AUL << SMU_SYSLOCK_UNLOCK_KEY_POS)

typedef struct
{
    __IO uint32_t EFUSE_A;                       // Body Option Lock Register
    __IO uint32_t EFUSE_B;
} EFUSE_TYPE_DEF;


/*---------------------------------------------------------------------------------------
 * Clock Control Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t AHBCKEN;                       // AHB Peripherals Clock Enable Register 0x50001000
    __I  uint32_t RESERVED0[3];                  // Reserved
    __IO uint32_t APBCKEN;                       // APB Peripherals Clock Enable Register 0x50001010
    __I  uint32_t RESERVED1[3];                  // Reserved
    __IO uint32_t AHBCKSEL;                      // AHB Peripherals Clock Selection 0 Register 0x50001020
    __I  uint32_t RESERVED2[3];                  // Reserved
    __IO uint32_t CLKDIV1;                       // Clock Divider Register 0x50001030
    __I  uint32_t RESERVED3[3];                  // Reserved
    __IO uint32_t CLKDIV2;                       // Clock Divider Register 0x50001040
    __I  uint32_t RESERVED4[3];                  // Reserved
    __IO uint32_t CLKSTS;                        // Clock Status Register 0x50001050
    __I  uint32_t RESERVED5[3];                  // Reserved
    __IO uint32_t SWTRIM;                        // Software Trim Register 0x50001060
    __I  uint32_t RESERVED6[3];                  // Reserved
    __IO uint32_t CLKDIV3;                       // Clock Divider Register 0x50001070
} CLOCK_TYPE_DEF;

/*
 * Bit definition for CLOCK_AHBCKEN[15]- CPU clock enable
 */
#define CLOCK_AHBCKEN_CPU_CLK_EN_POS             (15)
#define CLOCK_AHBCKEN_CPU_CLK_EN_MSK             (0x1UL << CLOCK_AHBCKEN_CPU_CLK_EN_POS)
#define CLOCK_AHBCKEN_CPU_CLK_ENABLE             (0x1UL << CLOCK_AHBCKEN_CPU_CLK_EN_POS)
#define CLOCK_AHBCKEN_CPU_CLK_DISABLE            (0x0UL << CLOCK_AHBCKEN_CPU_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[14]- SysTick clock enable
 */
#define CLOCK_AHBCKEN_SYSTICK_CLK_EN_POS         (14)
#define CLOCK_AHBCKEN_SYSTICK_CLK_EN_MSK         (0x1UL << CLOCK_AHBCKEN_SYSTICK_CLK_EN_POS)
#define CLOCK_AHBCKEN_SYSTICK_CLK_ENABLE         (0x1UL << CLOCK_AHBCKEN_SYSTICK_CLK_EN_POS)
#define CLOCK_AHBCKEN_SYSTICK_CLK_DISABLE        (0x0UL << CLOCK_AHBCKEN_SYSTICK_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[13]- Analog Controller clock enable
 */
#define CLOCK_AHBCKEN_ANALOG_CLK_EN_POS          (13)
#define CLOCK_AHBCKEN_ANALOG_CLK_EN_MSK          (0x1UL << CLOCK_AHBCKEN_ANALOG_CLK_EN_POS)
#define CLOCK_AHBCKEN_ANALOG_CLK_ENABLE          (0x1UL << CLOCK_AHBCKEN_ANALOG_CLK_EN_POS)
#define CLOCK_AHBCKEN_ANALOG_CLK_DISABLE         (0x0UL << CLOCK_AHBCKEN_ANALOG_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[12]- Interrupt Controller clock enable
 */
#define CLOCK_AHBCKEN_INT_CLK_EN_POS             (12)
#define CLOCK_AHBCKEN_INT_CLK_EN_MSK             (0x1UL << CLOCK_AHBCKEN_INT_CLK_EN_POS)
#define CLOCK_AHBCKEN_INT_CLK_ENABLE             (0x1UL << CLOCK_AHBCKEN_INT_CLK_EN_POS)
#define CLOCK_AHBCKEN_INT_CLK_DISABLE            (0x0UL << CLOCK_AHBCKEN_INT_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[11]- Internal RAM clock enable
 */
#define CLOCK_AHBCKEN_RAM_CLK_EN_POS             (11)
#define CLOCK_AHBCKEN_RAM_CLK_EN_MSK             (0x1UL << CLOCK_AHBCKEN_RAM_CLK_EN_POS)
#define CLOCK_AHBCKEN_RAM_CLK_ENABLE             (0x1UL << CLOCK_AHBCKEN_RAM_CLK_EN_POS)
#define CLOCK_AHBCKEN_RAM_CLK_DISABLE            (0x0UL << CLOCK_AHBCKEN_RAM_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[10]- APB sub bus clock enable
 */
#define CLOCK_AHBCKEN_APB_CLK_EN_POS             (10)
#define CLOCK_AHBCKEN_APB_CLK_EN_MSK             (0x1UL << CLOCK_AHBCKEN_APB_CLK_EN_POS)
#define CLOCK_AHBCKEN_APB_CLK_ENABLE             (0x1UL << CLOCK_AHBCKEN_APB_CLK_EN_POS)
#define CLOCK_AHBCKEN_APB_CLK_DISABLE            (0x0UL << CLOCK_AHBCKEN_APB_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[9]- AHB bus matrix clock enable
 */
#define CLOCK_AHBCKEN_AHBBUS_CLK_EN_POS          (9)
#define CLOCK_AHBCKEN_AHBBUS_CLK_EN_MSK          (0x1UL << CLOCK_AHBCKEN_AHBBUS_CLK_EN_POS)
#define CLOCK_AHBCKEN_AHBBUS_CLK_ENABLE          (0x1UL << CLOCK_AHBCKEN_AHBBUS_CLK_EN_POS)
#define CLOCK_AHBCKEN_AHBBUS_CLK_DISABLE         (0x0UL << CLOCK_AHBCKEN_AHBBUS_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[8]- Flash clock enable
 */
#define CLOCK_AHBCKEN_FLASH_CLK_EN_POS           (8)
#define CLOCK_AHBCKEN_FLASH_CLK_EN_MSK           (0x1UL << CLOCK_AHBCKEN_FLASH_CLK_EN_POS)
#define CLOCK_AHBCKEN_FLASH_CLK_ENABLE           (0x1UL << CLOCK_AHBCKEN_FLASH_CLK_EN_POS)
#define CLOCK_AHBCKEN_FLASH_CLK_DISABLE          (0x0UL << CLOCK_AHBCKEN_FLASH_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[7]- GPIO clock enable
 */
#define CLOCK_AHBCKEN_GPIO_CLK_EN_POS            (7)
#define CLOCK_AHBCKEN_GPIO_CLK_EN_MSK            (0x1UL << CLOCK_AHBCKEN_GPIO_CLK_EN_POS)
#define CLOCK_AHBCKEN_GPIO_CLK_ENABLE            (0x1UL << CLOCK_AHBCKEN_GPIO_CLK_EN_POS)
#define CLOCK_AHBCKEN_GPIO_CLK_DISABLE           (0x0UL << CLOCK_AHBCKEN_GPIO_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[5]- SPU clock enable
 */
#define CLOCK_AHBCKEN_SPU_CLK_EN_POS             (5)
#define CLOCK_AHBCKEN_SPU_CLK_EN_MSK             (0x1UL << CLOCK_AHBCKEN_SPU_CLK_EN_POS)
#define CLOCK_AHBCKEN_SPU_CLK_ENABLE             (0x1UL << CLOCK_AHBCKEN_SPU_CLK_EN_POS)
#define CLOCK_AHBCKEN_SPU_CLK_DISABLE            (0x0UL << CLOCK_AHBCKEN_SPU_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[4]- SPIFC clock enable
 */
#define CLOCK_AHBCKEN_SPIFC_CLK_EN_POS           (4)
#define CLOCK_AHBCKEN_SPIFC_CLK_EN_MSK           (0x1UL << CLOCK_AHBCKEN_SPIFC_CLK_EN_POS)
#define CLOCK_AHBCKEN_SPIFC_CLK_ENABLE           (0x1UL << CLOCK_AHBCKEN_SPIFC_CLK_EN_POS)
#define CLOCK_AHBCKEN_SPIFC_CLK_DISABLE          (0x0UL << CLOCK_AHBCKEN_SPIFC_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[3]- MAC clock enable
 */
#define CLOCK_AHBCKEN_MAC_CLK_EN_POS             (3)
#define CLOCK_AHBCKEN_MAC_CLK_EN_MSK             (0x1UL << CLOCK_AHBCKEN_MAC_CLK_EN_POS)
#define CLOCK_AHBCKEN_MAC_CLK_ENABLE             (0x1UL << CLOCK_AHBCKEN_MAC_CLK_EN_POS)
#define CLOCK_AHBCKEN_MAC_CLK_DISABLE            (0x0UL << CLOCK_AHBCKEN_MAC_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[2]- CCP1 clock enable
 */
#define CLOCK_AHBCKEN_CCP1_CLK_EN_POS            (2)
#define CLOCK_AHBCKEN_CCP1_CLK_EN_MSK            (0x1UL << CLOCK_AHBCKEN_CCP1_CLK_EN_POS)
#define CLOCK_AHBCKEN_CCP1_CLK_ENABLE            (0x1UL << CLOCK_AHBCKEN_CCP1_CLK_EN_POS)
#define CLOCK_AHBCKEN_CCP1_CLK_DISALBE           (0x0UL << CLOCK_AHBCKEN_CCP1_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[1]- CCP0 clock enable
 */
#define CLOCK_AHBCKEN_CCP0_CLK_EN_POS            (1)
#define CLOCK_AHBCKEN_CCP0_CLK_EN_MSK            (0x1UL << CLOCK_AHBCKEN_CCP0_CLK_EN_POS)
#define CLOCK_AHBCKEN_CCP0_CLK_ENABLE            (0x1UL << CLOCK_AHBCKEN_CCP0_CLK_EN_POS)
#define CLOCK_AHBCKEN_CCP0_CLK_DISALBE           (0x0UL << CLOCK_AHBCKEN_CCP0_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKEN[0]- DMA clock enable
 */
#define CLOCK_AHBCKEN_DMA_CLK_EN_POS             (0)
#define CLOCK_AHBCKEN_DMA_CLK_EN_MSK             (0x1UL << CLOCK_AHBCKEN_DMA_CLK_EN_POS)
#define CLOCK_AHBCKEN_DMA_CLK_ENABLE             (0x1UL << CLOCK_AHBCKEN_DMA_CLK_EN_POS)
#define CLOCK_AHBCKEN_DMA_CLK_DISABLE            (0x0UL << CLOCK_AHBCKEN_DMA_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[15]- Watch dog clock enable
 */
#define CLOCK_APBCKEN_WDG_CLK_EN_POS             (15)
#define CLOCK_APBCKEN_WDG_CLK_EN_MSK             (0x1UL << CLOCK_APBCKEN_WDG_CLK_EN_POS)
#define CLOCK_APBCKEN_WDG_CLK_ENABLE             (0x1UL << CLOCK_APBCKEN_WDG_CLK_EN_POS)
#define CLOCK_APBCKEN_WDG_CLK_DISABLE            (0x0UL << CLOCK_APBCKEN_WDG_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[14]- TimeBase clock enable
 */
#define CLOCK_APBCKEN_TB_CLK_EN_POS              (14)
#define CLOCK_APBCKEN_TB_CLK_EN_MSK              (0x1UL << CLOCK_APBCKEN_TB_CLK_EN_POS)
#define CLOCK_APBCKEN_TB_CLK_ENABLE              (0x1UL << CLOCK_APBCKEN_TB_CLK_EN_POS)
#define CLOCK_APBCKEN_TB_CLK_DISABLE             (0x0UL << CLOCK_APBCKEN_TB_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[13]- UART1 clock enable
 */
#define CLOCK_APBCKEN_UART1_CLK_EN_POS           (13)
#define CLOCK_APBCKEN_UART1_CLK_EN_MSK           (0x1UL << CLOCK_APBCKEN_UART1_CLK_EN_POS)
#define CLOCK_APBCKEN_UART1_CLK_ENABLE           (0x1UL << CLOCK_APBCKEN_UART1_CLK_EN_POS)
#define CLOCK_APBCKEN_UART1_CLK_DISABLE          (0x0UL << CLOCK_APBCKEN_UART1_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[12]- USB clock enable
 */
#define CLOCK_APBCKEN_USB_CLK_EN_POS             (12)
#define CLOCK_APBCKEN_USB_CLK_EN_MSK             (0x1UL << CLOCK_APBCKEN_USB_CLK_EN_POS)
#define CLOCK_APBCKEN_USB_CLK_ENABLE             (0x1UL << CLOCK_APBCKEN_USB_CLK_EN_POS)
#define CLOCK_APBCKEN_USB_CLK_DISABLE            (0x0UL << CLOCK_APBCKEN_USB_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[11]- AUDPWM clock enable
 */
#define CLOCK_APBCKEN_AUDPWM_CLK_EN_POS          (11)
#define CLOCK_APBCKEN_AUDPWM_CLK_EN_MSK          (0x1UL << CLOCK_APBCKEN_AUDPWM_CLK_EN_POS)
#define CLOCK_APBCKEN_AUDPWM_CLK_ENABLE          (0x1UL << CLOCK_APBCKEN_AUDPWM_CLK_EN_POS)
#define CLOCK_APBCKEN_AUDPWM_CLK_DISABLE         (0x0UL << CLOCK_APBCKEN_AUDPWM_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[10]- CTS clock enable
 */
#define CLOCK_APBCKEN_CTS_CLK_EN_POS             (10)
#define CLOCK_APBCKEN_CTS_CLK_EN_MSK             (0x1UL << CLOCK_APBCKEN_CTS_CLK_EN_POS)
#define CLOCK_APBCKEN_CTS_CLK_ENABLE             (0x1UL << CLOCK_APBCKEN_CTS_CLK_EN_POS)
#define CLOCK_APBCKEN_CTS_CLK_DISABLE            (0x0UL << CLOCK_APBCKEN_CTS_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[9]- I2S clock enable
 */
#define CLOCK_APBCKEN_I2S_CLK_EN_POS             (9)
#define CLOCK_APBCKEN_I2S_CLK_EN_MSK             (0x1UL << CLOCK_APBCKEN_I2S_CLK_EN_POS)
#define CLOCK_APBCKEN_I2S_CLK_ENABLE             (0x1UL << CLOCK_APBCKEN_I2S_CLK_EN_POS)
#define CLOCK_APBCKEN_I2S_CLK_DISABLE            (0x0UL << CLOCK_APBCKEN_I2S_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[8]- Delta sigma ADC clock enable
 */
#define CLOCK_APBCKEN_DSADC_CLK_EN_POS           (8)
#define CLOCK_APBCKEN_DSADC_CLK_EN_MSK           (0x1UL << CLOCK_APBCKEN_DSADC_CLK_EN_POS)
#define CLOCK_APBCKEN_DSADC_CLK_ENABLE           (0x1UL << CLOCK_APBCKEN_DSADC_CLK_EN_POS)
#define CLOCK_APBCKEN_DSADC_CLK_DISABLE          (0x0UL << CLOCK_APBCKEN_DSADC_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[7]- SAR ADC clock enable
 */
#define CLOCK_APBCKEN_SARADC_CLK_EN_POS          (7)
#define CLOCK_APBCKEN_SARADC_CLK_EN_MSK          (0x1UL << CLOCK_APBCKEN_SARADC_CLK_EN_POS)
#define CLOCK_APBCKEN_SARADC_CLK_ENABLE          (0x1UL << CLOCK_APBCKEN_SARADC_CLK_EN_POS)
#define CLOCK_APBCKEN_SARADC_CLK_DISABLE         (0x0UL << CLOCK_APBCKEN_SARADC_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[6]- SPI1 clock enable
 */
#define CLOCK_APBCKEN_SPI1_CLK_EN_POS            (6)
#define CLOCK_APBCKEN_SPI1_CLK_EN_MSK            (0x1UL << CLOCK_APBCKEN_SPI1_CLK_EN_POS)
#define CLOCK_APBCKEN_SPI1_CLK_ENABLE            (0x1UL << CLOCK_APBCKEN_SPI1_CLK_EN_POS)
#define CLOCK_APBCKEN_SPI1_CLK_DISABLE           (0x0UL << CLOCK_APBCKEN_SPI1_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[5]- SPI0 clock enable
 */
#define CLOCK_APBCKEN_SPI0_CLK_EN_POS            (5)
#define CLOCK_APBCKEN_SPI0_CLK_EN_MSK            (0x1UL << CLOCK_APBCKEN_SPI0_CLK_EN_POS)
#define CLOCK_APBCKEN_SPI0_CLK_ENABLE            (0x1UL << CLOCK_APBCKEN_SPI0_CLK_EN_POS)
#define CLOCK_APBCKEN_SPI0_CLK_DISABLE           (0x0UL << CLOCK_APBCKEN_SPI0_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[4]- UART0 clock enable
 */
#define CLOCK_APBCKEN_UART0_CLK_EN_POS           (4)
#define CLOCK_APBCKEN_UART0_CLK_EN_MSK           (0x1UL << CLOCK_APBCKEN_UART0_CLK_EN_POS)
#define CLOCK_APBCKEN_UART0_CLK_ENABLE           (0x1UL << CLOCK_APBCKEN_UART0_CLK_EN_POS)
#define CLOCK_APBCKEN_UART0_CLK_DISABLE          (0x0UL << CLOCK_APBCKEN_UART0_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[3]- I2C clock enable
 */
#define CLOCK_APBCKEN_I2C_CLK_EN_POS             (3)
#define CLOCK_APBCKEN_I2C_CLK_EN_MSK             (0x1UL << CLOCK_APBCKEN_I2C_CLK_EN_POS)
#define CLOCK_APBCKEN_I2C_CLK_ENABLE             (0x1UL << CLOCK_APBCKEN_I2C_CLK_EN_POS)
#define CLOCK_APBCKEN_I2C_CLK_DISABLE            (0x0UL << CLOCK_APBCKEN_I2C_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[2]- Timer clock enable
 */
#define CLOCK_APBCKEN_TIMER_CLK_EN_POS           (2)
#define CLOCK_APBCKEN_TIMER_CLK_EN_MSK           (0x1UL << CLOCK_APBCKEN_TIMER_CLK_EN_POS)
#define CLOCK_APBCKEN_TIMER_CLK_ENABLE           (0x1UL << CLOCK_APBCKEN_TIMER_CLK_EN_POS)
#define CLOCK_APBCKEN_TIMER_CLK_DISABLE          (0x0UL << CLOCK_APBCKEN_TIMER_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[1]- DAC clock enable
 */
#define CLOCK_APBCKEN_DAC_CLK_EN_POS             (1)
#define CLOCK_APBCKEN_DAC_CLK_EN_MSK             (0x1UL << CLOCK_APBCKEN_DAC_CLK_EN_POS)
#define CLOCK_APBCKEN_DAC_CLK_ENABLE             (0x1UL << CLOCK_APBCKEN_DAC_CLK_EN_POS)
#define CLOCK_APBCKEN_DAC_CLK_DISABLE            (0x0UL << CLOCK_APBCKEN_DAC_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[0]- QD clock enable
 */
#define CLOCK_APBCKEN_QD_CLK_EN_POS              (0)
#define CLOCK_APBCKEN_QD_CLK_EN_MSK              (0x1UL << CLOCK_APBCKEN_QD_CLK_EN_POS)
#define CLOCK_APBCKEN_QD_CLK_ENABLE              (0x1UL << CLOCK_APBCKEN_QD_CLK_EN_POS)
#define CLOCK_APBCKEN_QD_CLK_DISABLE             (0x0UL << CLOCK_APBCKEN_QD_CLK_EN_POS)

/*
 * Bit definition for CLOCK_APBCKEN[0]- PWMIO clock enable
 */
#define CLOCK_APBCKEN_PWMIO_CLK_EN_POS           (0)
#define CLOCK_APBCKEN_PWMIO_CLK_EN_MSK           (0x1UL << CLOCK_APBCKEN_PWMIO_CLK_EN_POS)
#define CLOCK_APBCKEN_PWMIO_CLK_ENABLE           (0x1UL << CLOCK_APBCKEN_PWMIO_CLK_EN_POS)
#define CLOCK_APBCKEN_PWMIO_CLK_DISABLE          (0x0UL << CLOCK_APBCKEN_PWMIO_CLK_EN_POS)

/*
 * Bit definition for CLOCK_AHBCKSEL[12]- SYS32K Selection
 */
#define CLOCK_AHBCKSEL_SYS32K_SEL_POS            (12)
#define CLOCK_AHBCKSEL_SYS32K_SEL_MSK            (0x1UL << CLOCK_AHBCKSEL_SYS32K_SEL_POS)
#define CLOCK_AHBCKSEL_SYS32K_SEL_IOSC32K        (0x1UL << CLOCK_AHBCKSEL_SYS32K_SEL_POS)
#define CLOCK_AHBCKSEL_SYS32K_SEL_XTAL32K        (0x0UL << CLOCK_AHBCKSEL_SYS32K_SEL_POS)

/*
 * Bit definition for CLOCK_AHBCKSEL[2]- SPI clock source Selection
 */
#define CLOCK_AHBCKSEL_PLL_SPICLK_SEL_POS        (2)
#define CLOCK_AHBCKSEL_PLL_SPICLK_SEL_MSK        (0x1UL << CLOCK_AHBCKSEL_PLL_SPICLK_SEL_POS)
#define CLOCK_AHBCKSEL_PLL_SPICLK_SEL_96M        (0x1UL << CLOCK_AHBCKSEL_PLL_SPICLK_SEL_POS)
#define CLOCK_AHBCKSEL_PLL_SPICLK_SEL_120M       (0x0UL << CLOCK_AHBCKSEL_PLL_SPICLK_SEL_POS)

/*
 * Bit definition for CLOCK_AHBCKSEL[1:0]- HCLK Source Selection
 */
#define CLOCK_AHBCKSEL_HCLK_SEL_POS              (0)
#define CLOCK_AHBCKSEL_HCLK_SEL_MSK              (0x3UL << CLOCK_AHBCKSEL_HCLK_SEL_POS)
#define CLOCK_AHBCKSEL_HCLK_SEL_SYS32K           (0x2UL << CLOCK_AHBCKSEL_HCLK_SEL_POS)
#define CLOCK_AHBCKSEL_HCLK_SEL_PLL              (0x1UL << CLOCK_AHBCKSEL_HCLK_SEL_POS)
#define CLOCK_AHBCKSEL_HCLK_SEL_OSC12M           (0x0UL << CLOCK_AHBCKSEL_HCLK_SEL_POS)

/*
 * Bit definition for CLOCK_CLKDIV1[8]- DS ADC clock source Selection
 */
#define CLOCK_CLKDIV1_DSADC_CLK_SRC_POS          (8)
#define CLOCK_CLKDIV1_DSADC_CLK_SRC_MSK          (0x1UL << CLOCK_CLKDIV1_DSADC_CLK_SRC_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_SRC_96M          (0x0UL << CLOCK_CLKDIV1_DSADC_CLK_SRC_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_SRC_SYS          (0x1UL << CLOCK_CLKDIV1_DSADC_CLK_SRC_POS)

/*
 * Bit definition for CLOCK_CLKDIV1[7:4] - DS ADC Clock divder
 */
#define CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS       (4)
#define CLOCK_CLKDIV1_DSADC_CLK_DIVDER_MSK       (0xFUL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_16           (0xFUL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_15           (0xEUL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_14           (0xDUL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_13           (0xCUL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_12           (0xBUL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_11           (0xAUL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_10           (0x9UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_9            (0x8UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_8            (0x7UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_7            (0x6UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_6            (0x5UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_5            (0x4UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_4            (0x3UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_3            (0x2UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_2            (0x1UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV1_DSADC_CLK_DIV_1            (0x0UL << CLOCK_CLKDIV1_DSADC_CLK_DIVDER_POS)

/*
 * Bit definition for CLOCK_CLKDIV1[3:0] - Audio PWM Clock divder
 */
#define CLOCK_CLKDIV1_AUDCLK_SEL_POS             (0)
#define CLOCK_CLKDIV1_AUDCLK_SEL_MSK             (0xFUL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_16   (0xFUL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_15   (0xEUL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_14   (0xDUL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_13   (0xCUL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_12   (0xBUL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_11   (0xAUL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_10   (0x9UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_9    (0x8UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_8    (0x7UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_7    (0x6UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_6    (0x5UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_5    (0x4UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_4    (0x3UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_3    (0x2UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_2    (0x1UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)
#define CLOCK_CLKDIV1_AUDCLK_SEL_SYSCLK_DIV_1    (0x0UL << CLOCK_CLKDIV1_AUDCLK_SEL_POS)

/*
 * Bit definition for CLOCK_CLKSTS[4]- External 12MHz Crystal Oscillator Ready Flag(XTAL 12MHz)
 */
#define CLOCK_CLKSTS_XTAL12M_POS                 (4)
#define CLOCK_CLKSTS_XTAL12M_MSK                 (0x1UL << CLOCK_CLKSTS_XTAL12M_POS)
#define CLOCK_CLKSTS_XTAL12M_RDY                 (0x1UL << CLOCK_CLKSTS_XTAL12M_POS)
#define CLOCK_CLKSTS_XTAL12M_NOT_RDY             (0x0UL << CLOCK_CLKSTS_XTAL12M_POS)

/*
 * Bit definition for CLOCK_CLKSTS[3]- External Crystal Oscillator Ready Flag(XTAL 32KHz)
 */
#define CLOCK_CLKSTS_XTAL32K_POS                 (3)
#define CLOCK_CLKSTS_XTAL32K_MSK                 (0x1UL << CLOCK_CLKSTS_XTAL32K_POS)
#define CLOCK_CLKSTS_XTAL32K_RDY                 (0x1UL << CLOCK_CLKSTS_XTAL32K_POS)
#define CLOCK_CLKSTS_XTAL32K_NOT_RDY             (0x0UL << CLOCK_CLKSTS_XTAL32K_POS)

/*
 * Bit definition for CLOCK_CLKSTS[2] - PLL Ready Flag
 */
#define CLOCK_CLKSTS_PLL_RDY_POS                 (2)
#define CLOCK_CLKSTS_PLL_RDY_MSK                 (0x1UL << CLOCK_CLKSTS_PLL_RDY_POS)
#define CLOCK_CLKSTS_PLL_RDY                     (0x1UL << CLOCK_CLKSTS_PLL_RDY_POS)
#define CLOCK_CLKSTS_PLL_NOT_RDY                 (0x0UL << CLOCK_CLKSTS_PLL_RDY_POS)

/*
 * Bit definition for CLOCK_CLKSTS[1]- Low Speed Internal RC Ready Flag(Iosc 32KHz)  Kim 2019.08.20
 */
#define CLOCK_CLKSTS_IOSC32K_POS                 (1)
#define CLOCK_CLKSTS_IOSC32K_MSK                 (0x1UL << CLOCK_CLKSTS_IOSC32K_POS)
#define CLOCK_CLKSTS_IOSC32K_RDY                 (0x1UL << CLOCK_CLKSTS_IOSC32K_POS)
#define CLOCK_CLKSTS_IOSC32K_NOT_RDY             (0x0UL << CLOCK_CLKSTS_IOSC32K_POS)

/*
 * Bit definition for CLOCK_CLKSTS[0] - IOSC12M Ready Flag
 */
#define CLOCK_CLKSTS_IOSC12M_RDY_POS             (0)
#define CLOCK_CLKSTS_IOSC12M_RDY_MSK             (0x1UL << CLOCK_CLKSTS_IOSC12M_RDY_POS)
#define CLOCK_CLKSTS_IOSC12M_RDY                 (0x1UL << CLOCK_CLKSTS_IOSC12M_RDY_POS)
#define CLOCK_CLKSTS_IOSC12M_NOT_RDY             (0x0UL << CLOCK_CLKSTS_IOSC12M_RDY_POS)

/*
 * Bit definition for CLOCK_SWTRIM[15:3] - Software Trim Counter Value
 */
#define CLOCK_SWTRIM_COUNTER_POS                 (3)
#define CLOCK_SWTRIM_COUNTER_MSK                 (0x1FFFUL << CLOCK_SWTRIM_COUNTER_POS)

/*
 * Bit definition for CLOCK_SWTRIM[2] - Reload From Efuse
 */
#define CLOCK_SWTRIM_RELOAD_EFUSE_POS            (2)
#define CLOCK_SWTRIM_RELOAD_EFUSE_MSK            (0x1UL << CLOCK_SWTRIM_RELOAD_EFUSE_POS)
#define CLOCK_SWTRIM_RELOAD_EFUSE                (0x1UL << CLOCK_SWTRIM_RELOAD_EFUSE_POS)

/*
 * Bit definition for CLOCK_SWTRIM[1] - Trim Counter Ready Flag
 */
#define CLOCK_SWTRIM_COUNTER_RDY_POS             (1)
#define CLOCK_SWTRIM_COUNTER_RDY_MSK             (0x1UL << CLOCK_SWTRIM_COUNTER_RDY_POS)
#define CLOCK_SWTRIM_COUNTER_RDY                 (0x1UL << CLOCK_SWTRIM_COUNTER_RDY_POS)
#define CLOCK_SWTRIM_COUNTER_NOT_RDY             (0x0UL << CLOCK_SWTRIM_COUNTER_RDY_POS)

/*
 * Bit definition for CLOCK_SWTRIM[0] - Software Trim Enable
 */
#define CLOCK_SWTRIM_EN_POS                      (0)
#define CLOCK_SWTRIM_EN_MSK                      (0x1UL << CLOCK_SWTRIM_EN_POS)
#define CLOCK_SWTRIM_ENABLE                      (0x1UL << CLOCK_SWTRIM_EN_POS)
#define CLOCK_SWTRIM_DISABLE                     (0x0UL << CLOCK_SWTRIM_EN_POS)

/*
 * Bit definition for CLOCK_CLKDIV3[1:0] - DS ADC Clock divder
 */
#define CLOCK_CLKDIV3_DSADC_CLK_DIVDER_POS       (0)
#define CLOCK_CLKDIV3_DSADC_CLK_DIVDER_MSK       (0x3UL << CLOCK_CLKDIV3_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV3_DSADC_CLK_DIV_4            (0x3UL << CLOCK_CLKDIV3_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV3_DSADC_CLK_DIV_2            (0x1UL << CLOCK_CLKDIV3_DSADC_CLK_DIVDER_POS)
#define CLOCK_CLKDIV3_DSADC_CLK_DIV_1            (0x0UL << CLOCK_CLKDIV3_DSADC_CLK_DIVDER_POS)


/*---------------------------------------------------------------------------------------
 * Reset control unit 0x50003000
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;
    __IO uint32_t STS;
} RCU_TYPE_DEF;

/*
 * Bit definition for RCU_CTRL[0] - Master Reset Trigger Register
 */
#define RCU_CTRL_MRST_EN_POS                     (0)
#define RCU_CTRL_MRST_EN_MSK                     (0x1UL << RCU_CTRL_MRST_EN_POS)
#define RCU_CTRL_MRST_EN_TRIGGER                 (0x0UL << RCU_CTRL_MRST_EN_POS)

/*
 * Bit definition for RCU_STS[6] - WatchDog reset
 */
#define RCU_STS_SYS_WDT_POS                      (6)
#define RCU_STS_SYS_WDT_MSK                      (0x1UL << RCU_STS_SYS_WDT_POS)
#define RCU_STS_SYS_WDT_FLAG                     (0x1UL << RCU_STS_SYS_WDT_POS)

/*
 * Bit definition for RCU_STS[4] - LVR reset
 */
#define RCU_STS_LVR_RST_POS                      (4)
#define RCU_STS_LVR_RST_MSK                      (0x1UL << RCU_STS_LVR_RST_POS)
#define RCU_STS_LVR_RST_FLAG                     (0x1UL << RCU_STS_LVR_RST_POS)

/*
 * Bit definition for RCU_STS[3] - System reset
 */
#define RCU_STS_SYS_RST_POS                      (3)
#define RCU_STS_SYS_RST_MSK                      (0x1UL << RCU_STS_SYS_RST_POS)
#define RCU_STS_SYS_RST_FLAG                     (0x1UL << RCU_STS_SYS_RST_POS)

/*
 * Bit definition for RCU_STS[2] - Software reset
 */
#define RCU_STS_SOFT_RST_POS                     (2)
#define RCU_STS_SOFT_RST_MSK                     (0x1UL << RCU_STS_SOFT_RST_POS)
#define RCU_STS_SOFT_RST_FLAG                    (0x1UL << RCU_STS_SOFT_RST_POS)

/*
 * Bit definition for RCU_STS[1] - H/W Key reset
 */
#define RCU_STS_KEY_RST_POS                      (1)
#define RCU_STS_KEY_RST_MSK                      (0x1UL << RCU_STS_KEY_RST_POS)
#define RCU_STS_KEY_RST_FLAG                     (0x1UL << RCU_STS_KEY_RST_POS)

/*
 * Bit definition for RCU_STS[0] - Power on reset
 */
#define RCU_STS_POR_POS                          (0)
#define RCU_STS_POR_MSK                          (0x1UL << RCU_STS_POR_POS)
#define RCU_STS_POR_FLAG                         (0x1UL << RCU_STS_POR_POS)


/*---------------------------------------------------------------------------------------
 * Interrupt control unit
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t NMICTRL;                       // NMI Interrupt Control Register
    __IO uint32_t SWIRQ;                         // Software Interrupt Request Register
    __I  uint32_t RESERVED0[2];                  // Reserved
    __IO uint32_t EXTIEN;                        // External Interrupt Enable Register
    __IO uint32_t EXTEEN;                        // External Event Enable Register
    __IO uint32_t EXTRHT;                        // External Rising Edge / High Level Trigger Register
    __I  uint32_t RESERVED1[1];                  // Reserved
    __IO uint32_t EXTFLT;                        // External Falling Edge / Low Level Trigger Register
    __I  uint32_t RESERVED2[1];                  // Reserved
    __IO uint32_t EXTIFLG;                       // External Interrupt Flag Register
    __IO uint32_t EXTEFLG;                       // External Event Flag Register
    __IO uint32_t EXTSWIE;                       // External Software Trigger Interrupt Event Register
} ITU_TypeDef;

/*
 * Bit definition for 0x10 ITU_NMICTRL[16] - NMI Interrupt Enabe
 */
#define ITU_NMICTRL_INT_EN_POS                   (16)
#define ITU_NMICTRL_INT_EN_MSK                   (0x1UL << ITU_NMICTRL_INT_EN_POS)
#define ITU_NMICTRL_INT_ENABLE                   (0x1UL << ITU_NMICTRL_INT_EN_POS)
#define ITU_NMICTRL_INT_DISABLE                  (0x0UL << ITU_NMICTRL_INT_EN_POS)

/*
 * Bit definition for 0x10 ITU_NMICTRL[4:0] - NMI Source Selection
 */
#define ITU_NMICTRL_SRC_SEL_POS                  (0)
#define ITU_NMICTRL_SRC_SEL_MSK                  (0x1FUL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_BEATspu              (0x1FUL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_PDM                  (0x1EUL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_KEYCHG               (0x1DUL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_TM2                  (0x1CUL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_TM1                  (0x1BUL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_TM0                  (0x1AUL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_DMA4                 (0x19UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_DMA3                 (0x18UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_DMA2                 (0x17UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_DMA1                 (0x16UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_DMA0                 (0x15UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_SPI1                 (0x14UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_SPI0                 (0x13UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_I2S                  (0x12UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_UART1                (0x11UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_UART0                (0x10UL << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_SPU                  (0xFUL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_I2C                  (0xEUL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_TIMEBASE             (0xDUL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_CTS_TMA1             (0xCUL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_CTS_TMA0             (0xBUL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_CCP1                 (0xAUL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_CCP0                 (0x9UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_DAC_CH1              (0x8UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_DAC_CH0              (0x7UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_DS_ADC               (0x6UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_SAR_ADC              (0x5UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_QD                   (0x4UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_MAC                  (0x3UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_EXT                  (0x2UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_VKey                 (0x1UL  << ITU_NMICTRL_SRC_SEL_POS)
#define ITU_NMICTRL_SRC_SEL_USB                  (0x0UL  << ITU_NMICTRL_SRC_SEL_POS)

/*
 * Bit definition for ITU_EXTIEN[3] - External Input 3 Interrupt Enable
 */
#define ITU_EXTIEN_EXT3_INT_EN_POS               (3)
#define ITU_EXTIEN_EXT3_INT_EN_MSK               (0x1UL << ITU_EXTIEN_EXT3_INT_EN_POS)
#define ITU_EXTIEN_EXT3_INT_ENABLE               (0x1UL << ITU_EXTIEN_EXT3_INT_EN_POS)
#define ITU_EXTIEN_EXT3_INT_DISABLE              (0x0UL << ITU_EXTIEN_EXT3_INT_EN_POS)

/*
 * Bit definition for ITU_EXTIEN[2] - External Input 2 Interrupt Enable
 */
#define ITU_EXTIEN_EXT2_INT_EN_POS               (2)
#define ITU_EXTIEN_EXT2_INT_EN_MSK               (0x1UL << ITU_EXTIEN_EXT2_INT_EN_POS)
#define ITU_EXTIEN_EXT2_INT_ENABLE               (0x1UL << ITU_EXTIEN_EXT2_INT_EN_POS)
#define ITU_EXTIEN_EXT2_INT_DISABLE              (0x0UL << ITU_EXTIEN_EXT2_INT_EN_POS)

/*
 * Bit definition for ITU_EXTIEN[1] - External Input 1 Interrupt Enable
 */
#define ITU_EXTIEN_EXT1_INT_EN_POS               (1)
#define ITU_EXTIEN_EXT1_INT_EN_MSK               (0x1UL << ITU_EXTIEN_EXT1_INT_EN_POS)
#define ITU_EXTIEN_EXT1_INT_ENABLE               (0x1UL << ITU_EXTIEN_EXT1_INT_EN_POS)
#define ITU_EXTIEN_EXT1_INT_DISABLE              (0x0UL << ITU_EXTIEN_EXT1_INT_EN_POS)

/*
 * Bit definition for ITU_EXTIEN[0] - External Input 0 Interrupt Enable
 */
#define ITU_EXTIEN_EXT0_INT_EN_POS               (0)
#define ITU_EXTIEN_EXT0_INT_EN_MSK               (0x1UL << ITU_EXTIEN_EXT0_INT_EN_POS)
#define ITU_EXTIEN_EXT0_INT_ENABLE               (0x1UL << ITU_EXTIEN_EXT0_INT_EN_POS)
#define ITU_EXTIEN_EXT0_INT_DISABLE              (0x0UL << ITU_EXTIEN_EXT0_INT_EN_POS)

/*
 * Bit definition for EXTEEN[4] - Key Change Event Enable
 */
#define ITU_EXTEEN_KEYCHG_EVT_EN_POS             (4)
#define ITU_EXTEEN_KEYCHG_EVT_EN_MSK             (0x1UL << ITU_EXTEEN_KEYCHG_EVT_EN_POS)
#define ITU_EXTEEN_KEYCHG_EVT_ENABLE             (0x1UL << ITU_EXTEEN_KEYCHG_EVT_EN_POS)
#define ITU_EXTEEN_KEYCHG_EVT_DISABLE            (0x0UL << ITU_EXTEEN_KEYCHG_EVT_EN_POS)

/*
 * Bit definition for ITU_EXTEEN[3] - External Input 3 Event Enable
 */
#define ITU_EXTEEN_EXT3_EVT_EN_POS               (3)
#define ITU_EXTEEN_EXT3_EVT_EN_MSK               (0x1UL << ITU_EXTEEN_EXT3_EVT_EN_POS)
#define ITU_EXTEEN_EXT3_EVT_ENABLE               (0x1UL << ITU_EXTEEN_EXT3_EVT_EN_POS)
#define ITU_EXTEEN_EXT3_EVT_DISABLE              (0x0UL << ITU_EXTEEN_EXT3_EVT_EN_POS)

/*
 * Bit definition for ITU_EXTEEN[2] - External Input 2 Event Enable
 */
#define ITU_EXTEEN_EXT2_EVT_EN_POS               (2)
#define ITU_EXTEEN_EXT2_EVT_EN_MSK               (0x1UL << ITU_EXTEEN_EXT2_EVT_EN_POS)
#define ITU_EXTEEN_EXT2_EVT_ENABLE               (0x1UL << ITU_EXTEEN_EXT2_EVT_EN_POS)
#define ITU_EXTEEN_EXT2_EVT_DISABLE              (0x0UL << ITU_EXTEEN_EXT2_EVT_EN_POS)

/*
 * Bit definition for ITU_EXTEEN[1] - External Input 1 Event Enable
 */
#define ITU_EXTEEN_EXT1_EVT_EN_POS               (1)
#define ITU_EXTEEN_EXT1_EVT_EN_MSK               (0x1UL << ITU_EXTEEN_EXT1_EVT_EN_POS)
#define ITU_EXTEEN_EXT1_EVT_ENABLE               (0x1UL << ITU_EXTEEN_EXT1_EVT_EN_POS)
#define ITU_EXTEEN_EXT1_EVT_DISABLE              (0x0UL << ITU_EXTEEN_EXT1_EVT_EN_POS)

/*
 * Bit definition for ITU_EXTEEN[0] - External Input 0 Event Enable
 */
#define ITU_EXTEEN_EXT0_EVT_EN_POS               (0)
#define ITU_EXTEEN_EXT0_EVT_EN_MSK               (0x1UL << ITU_EXTEEN_EXT0_EVT_EN_POS)
#define ITU_EXTEEN_EXT0_EVT_ENABLE               (0x1UL << ITU_EXTEEN_EXT0_EVT_EN_POS)
#define ITU_EXTEEN_EXT0_EVT_DISABLE              (0x0UL << ITU_EXTEEN_EXT0_EVT_EN_POS)

/*
 * Bit definition for ITU_EXTRHT[19] - External Input 3 High Level Trigger Enable
 */
#define ITU_EXTRHT_EXT3_HLVL_TRIG_EN_POS         (19)
#define ITU_EXTRHT_EXT3_HLVL_TRIG_EN_MSK         (0x1UL << ITU_EXTRHT_EXT3_HLVL_TRIG_EN_POS)
#define ITU_EXTRHT_EXT3_HLVL_TRIG_ENABLE         (0x1UL << ITU_EXTRHT_EXT3_HLVL_TRIG_EN_POS)
#define ITU_EXTRHT_EXT3_HLVL_TRIG_DISABLE        (0x0UL << ITU_EXTRHT_EXT3_HLVL_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTRHT[18] - External Input 2 High Level Trigger Enable
 */
#define ITU_EXTRHT_EXT2_HLVL_TRIG_EN_POS         (18)
#define ITU_EXTRHT_EXT2_HLVL_TRIG_EN_MSK         (0x1UL << ITU_EXTRHT_EXT2_HLVL_TRIG_EN_POS)
#define ITU_EXTRHT_EXT2_HLVL_TRIG_ENABLE         (0x1UL << ITU_EXTRHT_EXT2_HLVL_TRIG_EN_POS)
#define ITU_EXTRHT_EXT2_HLVL_TRIG_DISABLE        (0x0UL << ITU_EXTRHT_EXT2_HLVL_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTRHT[17] - External Input 1 High Level Trigger Enable
 */
#define ITU_EXTRHT_EXT1_HLVL_TRIG_EN_POS         (17)
#define ITU_EXTRHT_EXT1_HLVL_TRIG_EN_MSK         (0x1UL << ITU_EXTRHT_EXT1_HLVL_TRIG_EN_POS)
#define ITU_EXTRHT_EXT1_HLVL_TRIG_ENABLE         (0x1UL << ITU_EXTRHT_EXT1_HLVL_TRIG_EN_POS)
#define ITU_EXTRHT_EXT1_HLVL_TRIG_DISABLE        (0x0UL << ITU_EXTRHT_EXT1_HLVL_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTRHT[16] - External Input 0 High Level Trigger Enable
 */
#define ITU_EXTRHT_EXT0_HLVL_TRIG_EN_POS         (16)
#define ITU_EXTRHT_EXT0_HLVL_TRIG_EN_MSK         (0x1UL << ITU_EXTRHT_EXT0_HLVL_TRIG_EN_POS)
#define ITU_EXTRHT_EXT0_HLVL_TRIG_ENABLE         (0x1UL << ITU_EXTRHT_EXT0_HLVL_TRIG_EN_POS)
#define ITU_EXTRHT_EXT0_HLVL_TRIG_DISABLE        (0x0UL << ITU_EXTRHT_EXT0_HLVL_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTRHT[3] - External Input 3 Rising Trigger Enable
 */
#define ITU_EXTRHT_EXT3_RISING_TRIG_EN_POS       (3)
#define ITU_EXTRHT_EXT3_RISING_TRIG_EN_MSK       (0x1UL << ITU_EXTRHT_EXT3_RISING_TRIG_EN_POS)
#define ITU_EXTRHT_EXT3_RISING_TRIG_ENABLE       (0x1UL << ITU_EXTRHT_EXT3_RISING_TRIG_EN_POS)
#define ITU_EXTRHT_EXT3_RISING_TRIG_DISABLE      (0x0UL << ITU_EXTRHT_EXT3_RISING_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTRHT[2] - External Input 2 Rising Trigger Enable
 */
#define ITU_EXTRHT_EXT2_RISING_TRIG_EN_POS       (2)
#define ITU_EXTRHT_EXT2_RISING_TRIG_EN_MSK       (0x1UL << ITU_EXTRHT_EXT2_RISING_TRIG_EN_POS)
#define ITU_EXTRHT_EXT2_RISING_TRIG_ENABLE       (0x1UL << ITU_EXTRHT_EXT2_RISING_TRIG_EN_POS)
#define ITU_EXTRHT_EXT2_RISING_TRIG_DISABLE      (0x0UL << ITU_EXTRHT_EXT2_RISING_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTRHT[1] - External Input 1 Rising Trigger Enable
 */
#define ITU_EXTRHT_EXT1_RISING_TRIG_EN_POS       (1)
#define ITU_EXTRHT_EXT1_RISING_TRIG_EN_MSK       (0x1UL << ITU_EXTRHT_EXT1_RISING_TRIG_EN_POS)
#define ITU_EXTRHT_EXT1_RISING_TRIG_ENABLE       (0x1UL << ITU_EXTRHT_EXT1_RISING_TRIG_EN_POS)
#define ITU_EXTRHT_EXT1_RISING_TRIG_DISABLE      (0x0UL << ITU_EXTRHT_EXT1_RISING_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTRHT[0] - External Input 0 Rising Trigger Enable
 */
#define ITU_EXTRHT_EXT0_RISING_TRIG_EN_POS       (0)
#define ITU_EXTRHT_EXT0_RISING_TRIG_EN_MSK       (0x1UL << ITU_EXTRHT_EXT0_RISING_TRIG_EN_POS)
#define ITU_EXTRHT_EXT0_RISING_TRIG_ENABLE       (0x1UL << ITU_EXTRHT_EXT0_RISING_TRIG_EN_POS)
#define ITU_EXTRHT_EXT0_RISING_TRIG_DISABLE      (0x0UL << ITU_EXTRHT_EXT0_RISING_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTFLT[19] - External Input 3 Low Level Trigger Enable
 */
#define ITU_EXTFLT_EXT3_LLVL_TRIG_EN_POS         (19)
#define ITU_EXTFLT_EXT3_LLVL_TRIG_EN_MSK         (0x1UL << ITU_EXTFLT_EXT3_LLVL_TRIG_EN_POS)
#define ITU_EXTFLT_EXT3_LLVL_TRIG_ENABLE         (0x1UL << ITU_EXTFLT_EXT3_LLVL_TRIG_EN_POS)
#define ITU_EXTFLT_EXT3_LLVL_TRIG_DISABLE        (0x0UL << ITU_EXTFLT_EXT3_LLVL_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTFLT[18] - External Input 2 Low Level Trigger Enable
 */
#define ITU_EXTFLT_EXT2_LLVL_TRIG_EN_POS         (18)
#define ITU_EXTFLT_EXT2_LLVL_TRIG_EN_MSK         (0x1UL << ITU_EXTFLT_EXT2_LLVL_TRIG_EN_POS)
#define ITU_EXTFLT_EXT2_LLVL_TRIG_ENABLE         (0x1UL << ITU_EXTFLT_EXT2_LLVL_TRIG_EN_POS)
#define ITU_EXTFLT_EXT2_LLVL_TRIG_DISABLE        (0x0UL << ITU_EXTFLT_EXT2_LLVL_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTFLT[17] - External Input 1 Low Level Trigger Enable
 */
#define ITU_EXTFLT_EXT1_LLVL_TRIG_EN_POS         (17)
#define ITU_EXTFLT_EXT1_LLVL_TRIG_EN_MSK         (0x1UL << ITU_EXTFLT_EXT1_LLVL_TRIG_EN_POS)
#define ITU_EXTFLT_EXT1_LLVL_TRIG_ENABLE         (0x1UL << ITU_EXTFLT_EXT1_LLVL_TRIG_EN_POS)
#define ITU_EXTFLT_EXT1_LLVL_TRIG_DISABLE        (0x0UL << ITU_EXTFLT_EXT1_LLVL_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTFLT[16] - External Input 0 Low Level Trigger Enable
 */
#define ITU_EXTFLT_EXT0_LLVL_TRIG_EN_POS         (16)
#define ITU_EXTFLT_EXT0_LLVL_TRIG_EN_MSK         (0x1UL << ITU_EXTFLT_EXT0_LLVL_TRIG_EN_POS)
#define ITU_EXTFLT_EXT0_LLVL_TRIG_ENABLE         (0x1UL << ITU_EXTFLT_EXT0_LLVL_TRIG_EN_POS)
#define ITU_EXTFLT_EXT0_LLVL_TRIG_DISABLE        (0x0UL << ITU_EXTFLT_EXT0_LLVL_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTFLT[3] - External Input 3 Falling Trigger Enable
 */
#define ITU_EXTFLT_EXT3_FALLING_TRIG_EN_POS      (3)
#define ITU_EXTFLT_EXT3_FALLING_TRIG_EN_MSK      (0x1UL << ITU_EXTFLT_EXT3_FALLING_TRIG_EN_POS)
#define ITU_EXTFLT_EXT3_FALLING_TRIG_ENABLE      (0x1UL << ITU_EXTFLT_EXT3_FALLING_TRIG_EN_POS)
#define ITU_EXTFLT_EXT3_FALLING_TRIG_DISABLE     (0x0UL << ITU_EXTFLT_EXT3_FALLING_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTFLT[2] - External Input 2 Falling Trigger Enable
 */
#define ITU_EXTFLT_EXT2_FALLING_TRIG_EN_POS      (2)
#define ITU_EXTFLT_EXT2_FALLING_TRIG_EN_MSK      (0x1UL << ITU_EXTFLT_EXT2_FALLING_TRIG_EN_POS)
#define ITU_EXTFLT_EXT2_FALLING_TRIG_ENABLE      (0x1UL << ITU_EXTFLT_EXT2_FALLING_TRIG_EN_POS)
#define ITU_EXTFLT_EXT2_FALLING_TRIG_DISABLE     (0x0UL << ITU_EXTFLT_EXT2_FALLING_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTFLT[1] - External Input 1 Falling Trigger Enable
 */
#define ITU_EXTFLT_EXT1_FALLING_TRIG_EN_POS      (1)
#define ITU_EXTFLT_EXT1_FALLING_TRIG_EN_MSK      (0x1UL << ITU_EXTFLT_EXT1_FALLING_TRIG_EN_POS)
#define ITU_EXTFLT_EXT1_FALLING_TRIG_ENABLE      (0x1UL << ITU_EXTFLT_EXT1_FALLING_TRIG_EN_POS)
#define ITU_EXTFLT_EXT1_FALLING_TRIG_DISABLE     (0x0UL << ITU_EXTFLT_EXT1_FALLING_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTFLT[0] - External Input 0 Falling Trigger Enable
 */
#define ITU_EXTFLT_EXT0_FALLING_TRIG_EN_POS      (0)
#define ITU_EXTFLT_EXT0_FALLING_TRIG_EN_MSK      (0x1UL << ITU_EXTFLT_EXT0_FALLING_TRIG_EN_POS)
#define ITU_EXTFLT_EXT0_FALLING_TRIG_ENABLE      (0x1UL << ITU_EXTFLT_EXT0_FALLING_TRIG_EN_POS)
#define ITU_EXTFLT_EXT0_FALLING_TRIG_DISABLE     (0x0UL << ITU_EXTFLT_EXT0_FALLING_TRIG_EN_POS)

/*
 * Bit definition for ITU_EXTIFLG[3] - External Input 3 Interrupt Flag
 */
#define ITU_EXTIFLG_EXT3_INTF_POS                (3)
#define ITU_EXTIFLG_EXT3_INTF_MSK                (0x1UL << ITU_EXTIFLG_EXT3_INTF_POS)
#define ITU_EXTIFLG_EXT3_INT_FLAG                (0x1UL << ITU_EXTIFLG_EXT3_INTF_POS)

/*
 * Bit definition for ITU_EXTIFLG[2] - External Input 2 Interrupt Flag
 */
#define ITU_EXTIFLG_EXT2_INTF_POS                (2)
#define ITU_EXTIFLG_EXT2_INTF_MSK                (0x1UL << ITU_EXTIFLG_EXT2_INTF_POS)
#define ITU_EXTIFLG_EXT2_INT_FLAG                (0x1UL << ITU_EXTIFLG_EXT2_INTF_POS)

/*
 * Bit definition for ITU_EXTIFLG[1] - External Input 1 Interrupt Flag
 */
#define ITU_EXTIFLG_EXT1_INTF_POS                (1)
#define ITU_EXTIFLG_EXT1_INTF_MSK                (0x1UL << ITU_EXTIFLG_EXT1_INTF_POS)
#define ITU_EXTIFLG_EXT1_INT_FLAG                (0x1UL << ITU_EXTIFLG_EXT1_INTF_POS)

/*
 * Bit definition for ITU_EXTIFLG[0] - External Input 0 Interrupt Flag
 */
#define ITU_EXTIFLG_EXT0_INTF_POS                (0)
#define ITU_EXTIFLG_EXT0_INTF_MSK                (0x1UL << ITU_EXTIFLG_EXT0_INTF_POS)
#define ITU_EXTIFLG_EXT0_INT_FLAG                (0x1UL << ITU_EXTIFLG_EXT0_INTF_POS)

/*
 * Bit definition for ITU_EXTEFLG[4] - Key Change Event Flag
 */
#define ITU_EXTEFLG_KEYCHG_EVTF_POS              (4)
#define ITU_EXTEFLG_KEYCHG_EVTF_MSK              (0x1UL << ITU_EXTEFLG_KEYCHG_EVTF_POS)
#define ITU_EXTEFLG_KEYCHG_EVT_FLAG              (0x1UL << ITU_EXTEFLG_KEYCHG_EVTF_POS)

/*
 * Bit definition for ITU_EXTEFLG[3] - External Input 3 Event Flag
 */
#define ITU_EXTEFLG_EXT3_EVTF_POS                (3)
#define ITU_EXTEFLG_EXT3_EVTF_MSK                (0x1UL << ITU_EXTEFLG_EXT3_EVTF_POS)
#define ITU_EXTEFLG_EXT3_EVT_FLAG                (0x1UL << ITU_EXTEFLG_EXT3_EVTF_POS)

/*
 * Bit definition for ITU_EXTEFLG[2] - External Input 2 Event Flag
 */
#define ITU_EXTEFLG_EXT2_EVTF_POS                (2)
#define ITU_EXTEFLG_EXT2_EVTF_MSK                (0x1UL << ITU_EXTEFLG_EXT2_EVTF_POS)
#define ITU_EXTEFLG_EXT2_EVT_FLAG                (0x1UL << ITU_EXTEFLG_EXT2_EVTF_POS)

/*
 * Bit definition for ITU_EXTEFLG[1] - External Input 1 Event Flag
 */
#define ITU_EXTEFLG_EXT1_EVTF_POS                (1)
#define ITU_EXTEFLG_EXT1_EVTF_MSK                (0x1UL << ITU_EXTEFLG_EXT1_EVTF_POS)
#define ITU_EXTEFLG_EXT1_EVT_FLAG                (0x1UL << ITU_EXTEFLG_EXT1_EVTF_POS)

/*
 * Bit definition for ITU_EXTEFLG[0] - External Input 0 Event Flag
 */
#define ITU_EXTEFLG_EXT0_EVTF_POS                (0)
#define ITU_EXTEFLG_EXT0_EVTF_MSK                (0x1UL << ITU_EXTEFLG_EXT0_EVTF_POS)
#define ITU_EXTEFLG_EXT0_EVT_FLAG                (0x1UL << ITU_EXTEFLG_EXT0_EVTF_POS)


/*---------------------------------------------------------------------------------------
 * Analog control unit
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t REG33_CTRL;                    // REG33 Control Register
    __IO uint32_t REG12_CTRL;                    // REG12 Control Register
    __IO uint32_t SYS12M_CTRL;                   // SYS12M Control Register
    __IO uint32_t PLL_CTRL;                      // PLL Control Register
    __IO uint32_t X32K_CTRL;                     // X32K Control Register
    __IO uint32_t I32K_CTRL;                     // IOSC32K Control Register
    __IO uint32_t PLL_SPI_DIV;                   // PLL_SPI Clock Divide Register
    __I  uint32_t RESERVED3[5];                  // Reserved
    __IO uint32_t BVD_CTRL;                      // BVD Control Register
    __IO uint32_t APAD_CTRL;                     // Analog Pad Control Register
    __I  uint32_t RESERVED4[2];                  // Reserved
    __IO uint32_t OPT_KEYCODE;                   // 0x50005040
    __IO uint32_t OPT_LOCK1;                     // 0x50005044
    __IO uint32_t OPT_LOCK2;                     // 0x50005048
    __IO uint32_t OPT_LOCK3;                     // 0x5000504C
    __IO uint32_t OPT_LOCK4;                     // 0x50005050
    __IO uint32_t OPT_LOCK5;                     // 0x50005054
    __IO uint32_t OPT_LOCK6;                     // 0x50005058
    __IO uint32_t OPT_LOCK7;                     // 0x5000505C
    __IO uint32_t OPT_LOCK8;                     // 0x50005060
} ACU_TYPE_DEF;

/*
 * Bit definition for ACU_REG33_CTRL[6] - SPI regulator always enable
 */
#define ACU_REG33_CTRL_SPI_REG_AE_POS            (6)
#define ACU_REG33_CTRL_SPI_REG_AE_MSK            (0x1UL << ACU_REG33_CTRL_SPI_REG_AE_POS)
#define ACU_REG33_CTRL_SPI_REG_AE_ENABLE         (0x1UL << ACU_REG33_CTRL_SPI_REG_AE_POS)
#define ACU_REG33_CTRL_SPI_REG_AE_DISABLE        (0x0UL << ACU_REG33_CTRL_SPI_REG_AE_POS)

/*
 * Bit definition for ACU_REG33_CTRL[5] - REG33 SPI Regulator Sleep Mode Control
 */
#define ACU_REG33_CTRL_SLEEP_EN_POS              (5)
#define ACU_REG33_CTRL_SLEEP_EN_MSK              (0x1UL << ACU_REG33_CTRL_SLEEP_EN_POS)
#define ACU_REG33_CTRL_SLEEP_EN_ENABLE           (0x1UL << ACU_REG33_CTRL_SLEEP_EN_POS)
#define ACU_REG33_CTRL_SLEEP_EN_DISABLE          (0x0UL << ACU_REG33_CTRL_SLEEP_EN_POS)

/*
 * Bit definition for ACU_REG33_CTRL[4] - DS-ADC(Mic) Regulator Enable
 */
#define ACU_REG33_CTRL_DSADC_REG_EN_POS          (4)
#define ACU_REG33_CTRL_DSADC_REG_EN_MSK          (0x1UL << ACU_REG33_CTRL_DSADC_REG_EN_POS)
#define ACU_REG33_CTRL_DSADC_REG_EN_ENABLE       (0x1UL << ACU_REG33_CTRL_DSADC_REG_EN_POS)
#define ACU_REG33_CTRL_DSADC_REG_EN_DISABLE      (0x0UL << ACU_REG33_CTRL_DSADC_REG_EN_POS)

/*
 * Bit definition for ACU_REG12_CTRL[6] - 1.2V Regulator Enable
 */
#define ACU_REG12_CTRL_REG12_EN_POS              (6)
#define ACU_REG12_CTRL_REG12_EN_MSK              (0x1UL << ACU_REG12_CTRL_REG12_EN_POS)
#define ACU_REG12_CTRL_REG12_EN_ENABLE           (0x1UL << ACU_REG12_CTRL_REG12_EN_POS)
#define ACU_REG12_CTRL_REG12_EN_DISABLE          (0x0UL << ACU_REG12_CTRL_REG12_EN_POS)

/*
 * Bit definition for ACU_REG12_CTRL[3] - REG12 Regulator Sleep Mode Control
 */
#define ACU_REG12_CTRL_SLEEP_EN_POS              (3)
#define ACU_REG12_CTRL_SLEEP_EN_MSK              (0x1UL << ACU_REG12_CTRL_SLEEP_EN_POS)
#define ACU_REG12_CTRL_SLEEP_EN_ENABLE           (0x0UL << ACU_REG12_CTRL_SLEEP_EN_POS)
#define ACU_REG12_CTRL_SLEEP_EN_DISABLE          (0x1UL << ACU_REG12_CTRL_SLEEP_EN_POS)

/*
 * Bit definition for ACU_SYS12M_CTRL[14:8] - SYS12M Software Trim Bits
 */
#define ACU_SYS12M_CTRL_IOSC12M_TRIM_POS         (8)
#define ACU_SYS12M_CTRL_IOSC12M_TRIM_MSK         (0x7FUL << ACU_SYS12M_CTRL_IOSC12M_TRIM_POS)

/*
 * Bit definition for ACU_SYS12M_CTRL[2] - 12M X'tal strong mode control
 */
#define ACU_SYS12M_CTRL_STRONG_EN_POS            (2)
#define ACU_SYS12M_CTRL_STRONG_EN_MSK            (0x1UL << ACU_SYS12M_CTRL_STRONG_EN_POS)
#define ACU_SYS12M_CTRL_STRONG_EN_ENABLE         (0x1UL << ACU_SYS12M_CTRL_STRONG_EN_POS)
#define ACU_SYS12M_CTRL_STRONG_EN_DISABLE        (0x0UL << ACU_SYS12M_CTRL_STRONG_EN_POS)

/*
 * Bit definition for ACU_SYS12M_CTRL[1] - 12M X'tal control, low active
 */
#define ACU_SYS12M_CTRL_X12M_EN_POS              (1)
#define ACU_SYS12M_CTRL_X12M_EN_MSK              (0x1UL << ACU_SYS12M_CTRL_X12M_EN_POS)
#define ACU_SYS12M_CTRL_X12M_EN_DISABLE          (0x1UL << ACU_SYS12M_CTRL_X12M_EN_POS)
#define ACU_SYS12M_CTRL_X12M_EN_ENABLE           (0x0UL << ACU_SYS12M_CTRL_X12M_EN_POS)

/*
 * Bit definition for ACU_SYS12M_CTRL[0] - IOSC12M control, low active
 */
#define ACU_SYS12M_CTRL_IOSC12M_EN_POS           (0)
#define ACU_SYS12M_CTRL_IOSC12M_EN_MSK           (0x1UL << ACU_SYS12M_CTRL_IOSC12M_EN_POS)
#define ACU_SYS12M_CTRL_IOSC12M_EN_DISABLE       (0x1UL << ACU_SYS12M_CTRL_IOSC12M_EN_POS)
#define ACU_SYS12M_CTRL_IOSC12M_EN_ENABLE        (0x0UL << ACU_SYS12M_CTRL_IOSC12M_EN_POS)

/*
 * Bit definition for ACU_PLL_CTRL[7:3] - PLL Clock selection
 */
#define ACU_PLL_CTRL_SYS_DIVC_POS                (3)
#define ACU_PLL_CTRL_SYS_DIVC_MSK                (0x1FUL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_122D88M            (0x3UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_98D304M            (0x4UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_81D920M            (0x5UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_70D217M            (0x6UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_61D440M            (0x7UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_54D613M            (0x8UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_49D152M            (0x9UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_44D684M            (0xAUL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_40D960M            (0xBUL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_30D720M            (0xFUL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_24D576M            (0x13UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_20D480M            (0x17UL << ACU_PLL_CTRL_SYS_DIVC_POS)
#define ACU_PLL_CTRL_SYS_DIVC_15D360M            (0x1FUL << ACU_PLL_CTRL_SYS_DIVC_POS)

/*
 * Bit definition for ACU_CTRL[0] - PLL Enable
 */
#define ACU_PLL_CTRL_EN_POS                      (0)
#define ACU_PLL_CTRL_PLL_EN_MSK                  (0x1UL << ACU_PLL_CTRL_EN_POS)
#define ACU_PLL_CTRL_PLL_ENABLE                  (0x1UL << ACU_PLL_CTRL_EN_POS)
#define ACU_PLL_CTRL_PLL_DISABLE                 (0x0UL << ACU_PLL_CTRL_EN_POS)

/*
 * Bit definition for ACU_X32K_CTRL[7:6] - Strong/Enhance/Weak mode Select
 */
#define ACU_X32K_CTRL_MODE_SEL_POS               (6)
#define ACU_X32K_CTRL_MODE_SEL_MSK               (0x3UL << ACU_X32K_CTRL_MODE_SEL_POS)
#define ACU_X32K_CTRL_MODE_STRONG                (0x2UL << ACU_X32K_CTRL_MODE_SEL_POS)
#define ACU_X32K_CTRL_MODE_ENHANCE               (0x1UL << ACU_X32K_CTRL_MODE_SEL_POS)
#define ACU_X32K_CTRL_MODE_WEAK                  (0x0UL << ACU_X32K_CTRL_MODE_SEL_POS)

/*
 * Bit definition for ACU_X32K_CTRL[5] - LDO33(ADC, CTS) sleep control
 */
#define ACU_X32K_CTRL_CTS_ADC_LDO33_SLP_EN_POS   (5)
#define ACU_X32K_CTRL_CTS_ADC_LDO33_SLP_EN_MSK   (0x1UL << ACU_X32K_CTRL_CTS_ADC_LDO33_SLP_EN_POS)
#define ACU_X32K_CTRL_CTS_ADC_LDO33_SLP_ENABLE   (0x1UL << ACU_X32K_CTRL_CTS_ADC_LDO33_SLP_EN_POS)
#define ACU_X32K_CTRL_CTS_ADC_LDO33_SLP_DISABLE  (0x0UL << ACU_X32K_CTRL_CTS_ADC_LDO33_SLP_EN_POS)

/*
 * Bit definition for ACU_X32K_CTRL[4] - LDO12(ADC, CTS) sleep control
 */
#define ACU_X32K_CTRL_CTS_ADC_LDO12_SLP_EN_POS   (4)
#define ACU_X32K_CTRL_CTS_ADC_LDO12_SLP_EN_MSK   (0x1UL << ACU_X32K_CTRL_CTS_ADC_LDO12_SLP_EN_POS)
#define ACU_X32K_CTRL_CTS_ADC_LDO12_SLP_ENABLE   (0x1UL << ACU_X32K_CTRL_CTS_ADC_LDO12_SLP_EN_POS)
#define ACU_X32K_CTRL_CTS_ADC_LDO12_SLP_DISABLE  (0x0UL << ACU_X32K_CTRL_CTS_ADC_LDO12_SLP_EN_POS)

/*
 * Bit definition for ACU_X32K_CTRL[3] - Macro sleep control
 */
#define ACU_X32K_CTRL_MACRO_SLP_EN_POS           (3)
#define ACU_X32K_CTRL_MACRO_SLP_EN_MSK           (0x1UL << ACU_X32K_CTRL_MACRO_SLP_EN_POS)
#define ACU_X32K_CTRL_MACRO_SLP_EN_ENABLE        (0x1UL << ACU_X32K_CTRL_MACRO_SLP_EN_POS)
#define ACU_X32K_CTRL_MACRO_SLP_EN_DISABLE       (0x0UL << ACU_X32K_CTRL_MACRO_SLP_EN_POS)

/*
 * Bit definition for ACU_X32K_CTRL[2] - Iosc32K/X32K ON/OFF in deep sleep mode
 */
#define ACU_X32K_CTRL_32K_SLP_EN_POS             (2)
#define ACU_X32K_CTRL_32K_SLP_EN_MSK             (0x1UL << ACU_X32K_CTRL_32K_SLP_EN_POS)
#define ACU_X32K_CTRL_32K_SLP_ENABLE             (0x1UL << ACU_X32K_CTRL_32K_SLP_EN_POS)
#define ACU_X32K_CTRL_32K_SLP_DISABLE            (0x0UL << ACU_X32K_CTRL_32K_SLP_EN_POS)

/*
 * Bit definition for ACU_X32K_CTRL[0] - Enable X32K , low active
 */
#define ACU_X32K_CTRL_X32K_EN_POS                (0)
#define ACU_X32K_CTRL_X32K_EN_MSK                (0x1UL << ACU_X32K_CTRL_X32K_EN_POS)
#define ACU_X32K_CTRL_X32K_DISABLE               (0x1UL << ACU_X32K_CTRL_X32K_EN_POS)
#define ACU_X32K_CTRL_X32K_ENABLE                (0x0UL << ACU_X32K_CTRL_X32K_EN_POS)

/*
 * Bit definition for ACU_I32K_CTRL[7:1] - IOSC32K Trim Bits
 */
#define ACU_I32K_CTRL_IOSC32K_TRIM_POS           (1)
#define ACU_I32K_CTRL_IOSC32K_TRIM_MSK           (0x7FUL << ACU_I32K_CTRL_IOSC32K_TRIM_POS)

/*
 * Bit definition for ACU_I32K_CTRL[0] - Enable I32K , low active
 */
#define ACU_I32K_CTRL_I32K_EN_POS                (0)
#define ACU_I32K_CTRL_I32K_EN_MSK                (0x1UL << ACU_I32K_CTRL_I32K_EN_POS)
#define ACU_I32K_CTRL_I32K_DISABLE               (0x1UL << ACU_I32K_CTRL_I32K_EN_POS)
#define ACU_I32K_CTRL_I32K_ENABLE                (0x0UL << ACU_I32K_CTRL_I32K_EN_POS)

/*
 * Bit definition for ACU_PLL_SPI_DIV[12:8] - SPI_PLL_CLK divider for SPI falsh controller's PLL clock
 */
#define ACU_PLL_SPI_DIV_DIVC_POS                 (8)
#define ACU_PLL_SPI_DIV_DIVC_MSK                 (0x1FUL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_122D88M             (0x3UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_98D304M             (0x4UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_81D920M             (0x5UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_70D217M             (0x6UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_61D440M             (0x7UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_54D613M             (0x8UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_49D152M             (0x9UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_44D684M             (0xAUL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_40D960M             (0xBUL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_30D720M             (0xFUL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_24D576M             (0x13UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_20D480M             (0x17UL << ACU_PLL_SPI_DIV_DIVC_POS)
#define ACU_PLL_SPI_DIV_DIVC_15D360M             (0x1FUL << ACU_PLL_SPI_DIV_DIVC_POS)

/*
 * Bit definition for ACU_BVD_CTRL[4] - Battery voltage detect enable
 */
#define ACU_BVD_CTRL_EN_POS                      (4)
#define ACU_BVD_CTRL_EN_MSK                      (0x1UL << ACU_BVD_CTRL_EN_POS)
#define ACU_BVD_CTRL_ENABLE                      (0x1UL << ACU_BVD_CTRL_EN_POS)
#define ACU_BVD_CTRL_DISABLE                     (0x0UL << ACU_BVD_CTRL_EN_POS)

/*
 * Bit definition for ACU_BVD_CTRL[3:0] - Battery voltage detect trigger level,tolerance +/-7.5% (Read only)
 */
#define ACU_BVD_CTRL_LVL_SEL_POS                 (0)
#define ACU_BVD_CTRL_LVL_SEL_MSK                 (0xFUL << ACU_BVD_CTRL_LVL_SEL_POS)

/*
 * Bit definition for ACU_APAD[8] - Crystal 32KHz pad (IOA[8:7]) enable
 */
#define ACU_APAD_X32K_PAD_EN_POS                 (8)
#define ACU_APAD_X32K_PAD_EN_MSK                 (0x1UL << ACU_APAD_X32K_PAD_EN_POS)
#define ACU_APAD_X32K_PAD_ENABLE                 (0x1UL << ACU_APAD_X32K_PAD_EN_POS)
#define ACU_APAD_X32K_PAD_DISABLE                (0x0UL << ACU_APAD_X32K_PAD_EN_POS)

/*
 * Bit definition for ACU_APAD[7:0] - Line-in pad enable
 */
#define ACU_APAD_LINE_PAD_EN_POS                 (0)
#define ACU_APAD_LINE_PAD_EN_MSK                 (0xFFUL << ACU_APAD_LINE_PAD_EN_POS)
#define ACU_APAD_LINE_PAD_IOA31                  (0x80UL << ACU_APAD_LINE_PAD_EN_POS)
#define ACU_APAD_LINE_PAD_IOA30                  (0x40UL << ACU_APAD_LINE_PAD_EN_POS)
#define ACU_APAD_LINE_PAD_IOA29                  (0x20UL << ACU_APAD_LINE_PAD_EN_POS)
#define ACU_APAD_LINE_PAD_IOA28                  (0x10UL << ACU_APAD_LINE_PAD_EN_POS)
#define ACU_APAD_LINE_PAD_IOA27                  (0x08UL << ACU_APAD_LINE_PAD_EN_POS)
#define ACU_APAD_LINE_PAD_IOA26                  (0x04UL << ACU_APAD_LINE_PAD_EN_POS)
#define ACU_APAD_LINE_PAD_IOA25                  (0x02UL << ACU_APAD_LINE_PAD_EN_POS)
#define ACU_APAD_LINE_PAD_IOA24                  (0x01UL << ACU_APAD_LINE_PAD_EN_POS)

/*
 * Bit definition for ACU_OPT_KEYCODE[7:0]- System UnLock Key
 */
#define ACU_OPT_UNLOCK_KEY_POS                   (0)
#define ACU_OPT_UNLOCK_KEY_MSK                   (0xFFUL << ACU_OPT_UNLOCK_KEY_POS)
#define ACU_OPT_UNLOCK_KEY_KEY1                  (0x87UL << ACU_OPT_UNLOCK_KEY_POS)
#define ACU_OPT_UNLOCK_KEY_KEY2                  (0x9AUL << ACU_OPT_UNLOCK_KEY_POS)

/*
 * Bit definition for ACU_LOCK1[0] - 12M clock source select
 */
#define ACU_LOCK1_CLKSRC_SEL_POS                 (0)
#define ACU_LOCK1_CLKSRC_SEL_MSK                 (0x1UL << ACU_LOCK1_CLKSRC_SEL_POS)
#define ACU_LOCK1_CLKSRC_SEL_IOSC12M             (0x1UL << ACU_LOCK1_CLKSRC_SEL_POS)
#define ACU_LOCK1_CLKSRC_SEL_XTAL12M             (0x0UL << ACU_LOCK1_CLKSRC_SEL_POS)

/*
 * Bit definition for ACU_OPT_LOCK8[4:0]- CLK source trim control
 */
#define ACU_OPT_LOCK8_CLK_TRIM_EN_POS            (0)
#define ACU_OPT_LOCK8_CLK_TRIM_EN_MSK            (0x1FUL << ACU_OPT_LOCK8_CLK_TRIM_EN_POS)
#define ACU_OPT_LOCK8_CLK_TRIM_IOSC12M           (0x01UL << ACU_OPT_LOCK8_CLK_TRIM_EN_POS)
#define ACU_OPT_LOCK8_CLK_TRIM_PLL48M            (0x02UL << ACU_OPT_LOCK8_CLK_TRIM_EN_POS)
#define ACU_OPT_LOCK8_CLK_TRIM_IOSC32K           (0x04UL << ACU_OPT_LOCK8_CLK_TRIM_EN_POS)
#define ACU_OPT_LOCK8_CLK_TRIM_XTAL32K           (0x08UL << ACU_OPT_LOCK8_CLK_TRIM_EN_POS)
#define ACU_OPT_LOCK8_CLK_TRIM_XTAL12M           (0x10UL << ACU_OPT_LOCK8_CLK_TRIM_EN_POS)


/*---------------------------------------------------------------------------------------
 * Key Scan Controller Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL1;                         //Key Scan Control Register 1
    __IO uint32_t CTRL2;                         //Key Scan Control Register 2
    __IO uint32_t INTSTS;                        //Key Scan Interrupt Status Register
    __IO uint32_t ADDR;                          //Key Scan Velocity Address Register
    __IO uint32_t VELOCITY;                      //Key Scan Velocity Value Register
    __IO uint32_t RESERVED[3];                   //Reserved
    __IO uint32_t DATA[16];                      //Key Scan Sapmle Data Register[16]
} KEYSCAN_TYPE_DEF;

/*
 * Bit definition for KEYSCAN_CTRL1[22:16] - Key scan system clock counter
 */
#define KEYSCAN_CTRL1_ST_CNTR_POS                (16)
#define KEYSCAN_CTRL1_ST_CNTR_MSK                (0x7FUL << KEYSCAN_CTRL1_ST_CNTR_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[14] - Key scan interrupt enable
 */
#define KEYSCAN_CTRL1_INT_EN_POS                 (14)
#define KEYSCAN_CTRL1_INT_EN_MSK                 (0x1UL << KEYSCAN_CTRL1_INT_EN_POS)
#define KEYSCAN_CTRL1_INT_ENABLE                 (0x1UL << KEYSCAN_CTRL1_INT_EN_POS)
#define KEYSCAN_CTRL1_INT_DISABLE                (0x0UL << KEYSCAN_CTRL1_INT_EN_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[13] - Key scan auto mode enable
 */
#define KEYSCAN_CTRL1_AUTO_POS                   (13)
#define KEYSCAN_CTRL1_AUTO_MSK                   (0x1UL << KEYSCAN_CTRL1_AUTO_POS)
#define KEYSCAN_CTRL1_AUTO_ENABLE                (0x1UL << KEYSCAN_CTRL1_AUTO_POS)
#define KEYSCAN_CTRL1_AUTO_DISABLE               (0x0UL << KEYSCAN_CTRL1_AUTO_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[11] - Key scan input inverted control
 */
#define KEYSCAN_CTRL1_INV_POS                    (11)
#define KEYSCAN_CTRL1_INV_MSK                    (0x1UL << KEYSCAN_CTRL1_INV_POS)
#define KEYSCAN_CTRL1_INV_LOW                    (0x1UL << KEYSCAN_CTRL1_INV_POS)
#define KEYSCAN_CTRL1_INV_HIGH                   (0x0UL << KEYSCAN_CTRL1_INV_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[10] - Key scan smart mode enable
 */
#define KEYSCAN_CTRL1_SMART_POS                  (10)
#define KEYSCAN_CTRL1_SMART_MSK                  (0x1UL << KEYSCAN_CTRL1_SMART_POS)
#define KEYSCAN_CTRL1_SMART_ENABLE               (0x1UL << KEYSCAN_CTRL1_SMART_POS)
#define KEYSCAN_CTRL1_SMART_DISABLE              (0x0UL << KEYSCAN_CTRL1_SMART_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[9] - Key scan manual start scan
 */
#define KEYSCAN_CTRL1_MANUAL_STR_POS             (9)
#define KEYSCAN_CTRL1_MANUAL_STR_MSK             (0x1UL << KEYSCAN_CTRL1_MANUAL_STR_POS)
#define KEYSCAN_CTRL1_MANUAL_STR                 (0x1UL << KEYSCAN_CTRL1_MANUAL_STR_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[8] - Key scan busy flag
 */
#define KEYSCAN_CTRL1_BUSY_POS                   (8)
#define KEYSCAN_CTRL1_BUSY_MSK                   (0x1UL << KEYSCAN_CTRL1_BUSY_POS)
#define KEYSCAN_CTRL1_BUSY                       (0x1UL << KEYSCAN_CTRL1_BUSY_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[7] - Key scan force stop flag
 */
#define KEYSCAN_CTRL1_STOP_POS                   (7)
#define KEYSCAN_CTRL1_STOP_MSK                   (0x1UL << KEYSCAN_CTRL1_STOP_POS)
#define KEYSCAN_CTRL1_STOP                       (0x1UL << KEYSCAN_CTRL1_STOP_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[3:2] - Key scan sample time selection
 */
#define KEYSCAN_CTRL1_SMPTIME_SEL_POS            (2)
#define KEYSCAN_CTRL1_SMPTIME_SEL_MSK            (0x3UL << KEYSCAN_CTRL1_SMPTIME_SEL_POS)
#define KEYSCAN_CTRL1_SMPTIME_SEL_128            (0x3UL << KEYSCAN_CTRL1_SMPTIME_SEL_POS)
#define KEYSCAN_CTRL1_SMPTIME_SEL_64             (0x2UL << KEYSCAN_CTRL1_SMPTIME_SEL_POS)
#define KEYSCAN_CTRL1_SMPTIME_SEL_32             (0x1UL << KEYSCAN_CTRL1_SMPTIME_SEL_POS)
#define KEYSCAN_CTRL1_SMPTIME_SEL_16             (0x0UL << KEYSCAN_CTRL1_SMPTIME_SEL_POS)

/*
 * Bit definition for KEYSCAN_CTRL1[1:0] - Key scan timer selection
 */
#define KEYSCAN_CTRL1_TMR_SEL_POS                (0)
#define KEYSCAN_CTRL1_TMR_SEL_MSK                (0x3UL << KEYSCAN_CTRL1_TMR_SEL_POS)
#define KEYSCAN_CTRL1_TMR_SEL_NONE               (0x3UL << KEYSCAN_CTRL1_TMR_SEL_POS)
#define KEYSCAN_CTRL1_TMR_SEL_TMC                (0x2UL << KEYSCAN_CTRL1_TMR_SEL_POS)
#define KEYSCAN_CTRL1_TMR_SEL_TMB                (0x1UL << KEYSCAN_CTRL1_TMR_SEL_POS)
#define KEYSCAN_CTRL1_TMR_SEL_TMA                (0x0UL << KEYSCAN_CTRL1_TMR_SEL_POS)

/*
 * Bit definition for KEYSCAN_CTRL2[6] - Key scan IOA enable
 */
#define KEYSCAN_CTRL2_IOA_EN_POS                 (6)
#define KEYSCAN_CTRL2_IOA_EN_MSK                 (0x1UL << KEYSCAN_CTRL2_IOA_EN_POS)
#define KEYSCAN_CTRL2_IOA_EN_ENABLE              (0x1UL << KEYSCAN_CTRL2_IOA_EN_POS)
#define KEYSCAN_CTRL2_IOA_EN_DISABLE             (0x0UL << KEYSCAN_CTRL2_IOA_EN_POS)

/*
 * Bit definition for KEYSCAN_CTRL2[5] - Key scan read A-key status control
 */
#define KEYSCAN_CTRL2_AKEY_STS_POS               (5)
#define KEYSCAN_CTRL2_AKEY_STS_MSK               (0x1UL << KEYSCAN_CTRL2_AKEY_STS_POS)
#define KEYSCAN_CTRL2_AKEY_STS_ENABLE            (0x1UL << KEYSCAN_CTRL2_AKEY_STS_POS)
#define KEYSCAN_CTRL2_AKEY_STS_DISABLE           (0x0UL << KEYSCAN_CTRL2_AKEY_STS_POS)

/*
 * Bit definition for KEYSCAN_CTRL2[4:3] - Key scan out selection
 */
#define KEYSCAN_CTRL2_OUT_COUNT_SEL_POS          (3)
#define KEYSCAN_CTRL2_OUT_COUNT_SEL_MSK          (0x3UL << KEYSCAN_CTRL2_OUT_COUNT_SEL_POS)
#define KEYSCAN_CTRL2_OUT_COUNT_SEL_0xB          (0x2UL << KEYSCAN_CTRL2_OUT_COUNT_SEL_POS)
#define KEYSCAN_CTRL2_OUT_COUNT_SEL_0x9          (0x1UL << KEYSCAN_CTRL2_OUT_COUNT_SEL_POS)
#define KEYSCAN_CTRL2_OUT_COUNT_SEL_MODE         (0x0UL << KEYSCAN_CTRL2_OUT_COUNT_SEL_POS)

/*
 * Bit definition for KEYSCAN_CTRL2[2:0] - Key scan mode selection
 */
#define KEYSCAN_CTRL2_MODE_SEL_POS               (0)
#define KEYSCAN_CTRL2_MODE_SEL_MSK               (0x7UL << KEYSCAN_CTRL2_MODE_SEL_POS)
#define KEYSCAN_CTRL2_MODE_SEL_88VKEY_176SKEY    (0x7UL << KEYSCAN_CTRL2_MODE_SEL_POS)
#define KEYSCAN_CTRL2_MODE_SEL_64VKEY_128SKEY    (0x5UL << KEYSCAN_CTRL2_MODE_SEL_POS)
#define KEYSCAN_CTRL2_MODE_SEL_88VKEY            (0x3UL << KEYSCAN_CTRL2_MODE_SEL_POS)
#define KEYSCAN_CTRL2_MODE_SEL_88SKEY            (0x2UL << KEYSCAN_CTRL2_MODE_SEL_POS)
#define KEYSCAN_CTRL2_MODE_SEL_64VKEY            (0x1UL << KEYSCAN_CTRL2_MODE_SEL_POS)
#define KEYSCAN_CTRL2_MODE_SEL_64SKEY            (0x0UL << KEYSCAN_CTRL2_MODE_SEL_POS)

/*
 * Bit definition for KEYSCAN_INTSTS[0] - Key scan interrupt flag
 */
#define KEYSCAN_INTSTS_INT_POS                   (0)
#define KEYSCAN_INTSTS_INT_MSK                   (0x1UL << KEYSCAN_INTSTS_INT_POS)
#define KEYSCAN_INTSTS_INT_FLAG                  (0x1UL << KEYSCAN_INTSTS_INT_POS)

/*
 * Bit definition for KEYSCAN_ADDR[15] - Key scan read control
 */
#define KEYSCAN_ADDR_READ_POS                    (15)
#define KEYSCAN_ADDR_READ_MSK                    (0x1UL << KEYSCAN_ADDR_READ_POS)
#define KEYSCAN_ADDR_READ_ENABLE                 (0x1UL << KEYSCAN_ADDR_READ_POS)

/*
 * Bit definition for KEYSCAN_ADDR[6:0] - Key scan read address
 */
#define KEYSCAN_ADDR_ADDRESS_POS                 (0)
#define KEYSCAN_ADDR_ADDRESS_MSK                 (0x7FUL << KEYSCAN_ADDR_ADDRESS_POS)

/*
 * Bit definition for KEYSCAN_VELOCITY[13:0] - Key scan velocity data
 */
#define KEYSCAN_VELOCITY_DATA_POS                (0)
#define KEYSCAN_VELOCITY_DATA_MSK                (0x3FFFUL << KEYSCAN_VELOCITY_DATA_POS)


/*---------------------------------------------------------------------------------------
 * Non-Volatile Memory Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL_UNLOCK;                   // Flash Control Register Unlock Key
    __IO uint32_t RESERVED1;
    __IO uint32_t CTRL;                          // Flash Control Register
    __IO uint32_t ERASE_ADDR;                    // Flash Erase Address Register
    __IO uint32_t STS;
    __IO uint32_t OPTION0_STS;
    __IO uint32_t RESERVED2;
    __IO uint32_t RESERVED3;
    __IO uint32_t ERASE_CTRL;                    // Flash Erase Control Register
} FLASH_TYPE_DEF;

#define UNLOCK_KEY1                              0xABCD5678
#define UNLOCK_KEY2                              0x1234FEDC

/*
 * Bit definition for FLASH_CTRL[11:8] - Flash Erase Type
 */
#define FLASH_CTRL_TYPE_POS                      (8)
#define FLASH_CTRL_TYPE_MSK                      (0x3UL << FLASH_CTRL_TYPE_POS)
#define FLASH_CTRL_TYPE_CHIPERASE                (0x3UL << FLASH_CTRL_TYPE_POS)
#define FLASH_CTRL_TYPE_PAGEERASE                (0x2UL << FLASH_CTRL_TYPE_POS)
#define FLASH_CTRL_TYPE_PROGRAM                  (0x1UL << FLASH_CTRL_TYPE_POS)
#define FLASH_CTRL_TYPE_READ                     (0x0UL << FLASH_CTRL_TYPE_POS)

/*
 * Bit definition for FLASH_CTRL[0] - Lock / Unlock Status Flag
 */
#define FLASH_CTRL_LOCK_POS                      (0)
#define FLASH_CTRL_LOCK_MSK                      (0x1UL << FLASH_CTRL_LOCK_POS)
#define FLASH_CTRL_LOCK_FLAG                     (0x1UL << FLASH_CTRL_LOCK_POS)
#define FLASH_CTRL_LOCK                          (0x1UL << FLASH_CTRL_LOCK_POS)

/*
 * Bit definition for FLASH_STS[0]- Status Flag
 */
#define FLASH_STS_POS                            (0)
#define FLASH_STS_MSK                            (0x1UL << FLASH_STS_POS)
#define FLASH_STS_FLAG                           (0x1UL << FLASH_STS_POS)

/*
 * Bit definition for OPTION0_STS[0]- Security Status Flag
 */
#define FLASH_SECURITY_STS_POS                   (0)
#define FLASH_SECURITY_STS_MSK                   (0x1UL << FLASH_STS_POS)
#define FLASH_SECURITY_STS_FLAG                  (0x1UL << FLASH_STS_POS)

/*
 * Bit definition for FLASH_OPTION0_STS[2:0]- Latency Wait State Configuration
 */
#define FLASH_OPTION0_STS_WAITSTATE_POS          (0)
#define FLASH_OPTION0_STS_WAITSTATE_MSK          (0x7UL << FLASH_OPTION0_STS_WAITSTATE_POS)
#define FLASH_OPTION0_STS_7WAITSTATE             (0x7UL << FLASH_OPTION0_STS_WAITSTATE_POS)
#define FLASH_OPTION0_STS_6WAITSTATE             (0x6UL << FLASH_OPTION0_STS_WAITSTATE_POS)
#define FLASH_OPTION0_STS_5WAITSTATE             (0x5UL << FLASH_OPTION0_STS_WAITSTATE_POS)
#define FLASH_OPTION0_STS_4WAITSTATE             (0x4UL << FLASH_OPTION0_STS_WAITSTATE_POS)
#define FLASH_OPTION0_STS_3WAITSTATE             (0x3UL << FLASH_OPTION0_STS_WAITSTATE_POS)
#define FLASH_OPTION0_STS_2WAITSTATE             (0x2UL << FLASH_OPTION0_STS_WAITSTATE_POS)
#define FLASH_OPTION0_STS_1WAITSTATE             (0x1UL << FLASH_OPTION0_STS_WAITSTATE_POS)

/*
 * Bit definition for ERASE_CTRL[0]- NVM Operation Start
 */
#define FLASH_ERASE_CTRL_NVM_START_POS           (0)
#define FLASH_ERASE_CTRL_NVM_START_MSK           (0x1UL << FLASH_ERASE_CTRL_NVM_START_POS)
#define FLASH_ERASE_CTRL_NVM_START               (0x1UL << FLASH_ERASE_CTRL_NVM_START_POS)


/*---------------------------------------------------------------------------------------
 * MAC Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;                          // MAC Control Register
    __IO uint32_t LENGTH;                        // MAC Length Register
    __IO uint32_t ADDR_X ;                       // MAC Start address X Register
    __IO uint32_t ADDR_Y;                        // MAC Start address Y Register
    __IO uint32_t STEP_XY;                       // MAC Step X/Y Register
    __IO int32_t  OUTPUT;                        // MAC Output Register
    __IO int32_t  OUTPUT_EXT8;                   // MAC Output Extention Register
    __IO int32_t  SATURATION;                    // MAC OUT Saturation Register
    __IO int32_t  INTSTS;                        // MAC Interrupt Status Register
    __I  int32_t  OUTPUT_R;
    __I  int32_t  OUTPUT_EXT8_R;
    __I  int32_t  SATURATION_R;
    __IO int32_t  PRELOAD_DATA;
    __IO int32_t  START_CNT;
    __I  int32_t  MUL_DIV_STS;
    __IO int32_t  MUL1_X32;
    __IO int32_t  MUL2_X32;
    __IO int32_t  MUL12_C16;
    __I  int32_t  MUL1_OUT;
    __I  int32_t  MUL2_OUT;
    __IO int32_t  DIV_DIVIDEND;
    __IO int32_t  DIV_DIVISOR;
    __I  int32_t  MAC_DIV_Q;
    __I  int32_t  MAC_DIV_R;
    __IO int32_t  MAC_EXP_DATA;
    __I  int32_t  MAC_EXP_RESULT;
} MAC_TYPE_DEF;

/*
 * Bit definition for MAC_CTRL[12:8] - Output shift control
 */
#define MAC_CTRL_OUTPUT_SHIFT_POS                (8)
#define MAC_CTRL_OUTPUT_SHIFT_MSK                (0x1FUL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_LEFT_1             (0x1FUL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_16           (0x10UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_15           (0x0FUL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_14           (0x0EUL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_13           (0x0DUL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_12           (0x0CUL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_11           (0x0BUL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_10           (0x0AUL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_9            (0x09UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_8            (0x08UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_7            (0x07UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_6            (0x06UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_5            (0x05UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_4            (0x04UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_3            (0x03UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_2            (0x02UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_1            (0x01UL << MAC_CTRL_OUTPUT_SHIFT_POS)
#define MAC_CTRL_OUTPUT_SHIFT_RIGHT_0            (0x00UL << MAC_CTRL_OUTPUT_SHIFT_POS)

/*
 * Bit definition for MAC_CTRL[7] - MAC interrupt enable
 */
#define MAC_CTRL_INT_EN_POS                      (7)
#define MAC_CTRL_INT_EN_MSK                      (0x1UL << MAC_CTRL_INT_EN_POS)
#define MAC_CTRL_INT_ENABLE                      (0x1UL << MAC_CTRL_INT_EN_POS)
#define MAC_CTRL_INT_DISBLE                      (0x0UL << MAC_CTRL_INT_EN_POS)

/*
 * Bit definition for MAC_CTRL[6] - Operation mode selection
 */
#define MAC_CTRL_OPERATION_MODE_SEL_POS          (6)
#define MAC_CTRL_OPERATION_MODE_SEL_MSK          (0x1UL << MAC_CTRL_OPERATION_MODE_SEL_POS)
#define MAC_CTRL_OPERATION_MODE_SEL_SU           (0x1UL << MAC_CTRL_OPERATION_MODE_SEL_POS)
#define MAC_CTRL_OPERATION_MODE_SEL_SS           (0x0UL << MAC_CTRL_OPERATION_MODE_SEL_POS)

/*
 * Bit definition for MAC_CTRL[5:4] - Saturation mode selection
 */
#define MAC_CTRL_SATURATION_MODE_SEL_POS         (4)
#define MAC_CTRL_SATURATION_MODE_SEL_MSK         (0x3UL << MAC_CTRL_SATURATION_MODE_SEL_POS)
#define MAC_CTRL_SATURATION_MODE_SEL_8B          (0x2UL << MAC_CTRL_SATURATION_MODE_SEL_POS)
#define MAC_CTRL_SATURATION_MODE_SEL_32B         (0x1UL << MAC_CTRL_SATURATION_MODE_SEL_POS)
#define MAC_CTRL_SATURATION_MODE_SEL_16B         (0x0UL << MAC_CTRL_SATURATION_MODE_SEL_POS)

/*
 * Bit definition for MAC_CTRL[1] - MAC reset
 */
#define MAC_CTRL_RESET_POS                       (1)
#define MAC_CTRL_RESET_MSK                       (0x1UL << MAC_CTRL_RESET_POS)
#define MAC_CTRL_RESET                           (0x1UL << MAC_CTRL_RESET_POS)

/*
 * Bit definition for MAC_CTRL[0] - MAC start
 */
#define MAC_CTRL_START_POS                       (0)
#define MAC_CTRL_START_MSK                       (0x1UL << MAC_CTRL_START_POS)
#define MAC_CTRL_START                           (0x1UL << MAC_CTRL_START_POS)
#define MAC_CTRL_BUSY                            (0x1UL << MAC_CTRL_START_POS)

/*
 * Bit definition for MAC_LENGTH[7:0] - MAC length
 */
#define MAC_CTRL_LENGTH_POS                      (0)
#define MAC_CTRL_LENGTH_MSK                      (0xFFUL << MAC_CTRL_LENGTH_POS)

/*
 * Bit definition for MAC_STEP_XY[12:8] - MAC AddressY Step control
 */
#define MAC_STEP_XY_ADDRY_STEP_POS               (8)
#define MAC_STEP_XY_ADDRY_STEP_MSK               (0x1FUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_16          (0x10UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_15          (0x11UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_14          (0x12UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_13          (0x13UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_12          (0x14UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_11          (0x15UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_10          (0x16UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_9           (0x17UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_8           (0x18UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_7           (0x19UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_6           (0x1AUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_5           (0x1BUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_4           (0x1CUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_3           (0x1DUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_2           (0x1EUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_MINUS_1           (0x1FUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_15           (0xFUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_14           (0xEUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_13           (0xDUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_12           (0xCUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_11           (0xBUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_10           (0xAUL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_9            (0x9UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_8            (0x8UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_7            (0x7UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_6            (0x6UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_5            (0x5UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_4            (0x4UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_3            (0x3UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_2            (0x2UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_1            (0x1UL << MAC_STEP_XY_ADDRY_STEP_POS)
#define MAC_STEP_XY_ADDRY_STEP_PLUS_0            (0x0UL << MAC_STEP_XY_ADDRY_STEP_POS)

/*
 * Bit definition for MAC_STEP_XY[4:0] - MAC AddressX Step control
 */
#define MAC_STEP_XY_ADDRX_STEP_POS               (0)
#define MAC_STEP_XY_ADDRX_STEP_MSK               (0x1FUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_16          (0x10UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_15          (0x11UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_14          (0x12UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_13          (0x13UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_12          (0x14UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_11          (0x15UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_10          (0x16UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_9           (0x17UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_8           (0x18UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_7           (0x19UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_6           (0x1AUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_5           (0x1BUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_4           (0x1CUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_3           (0x1DUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_2           (0x1EUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_MINUS_1           (0x1FUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_15           (0xFUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_14           (0xEUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_13           (0xDUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_12           (0xCUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_11           (0xBUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_10           (0xAUL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_9            (0x9UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_8            (0x8UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_7            (0x7UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_6            (0x6UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_5            (0x5UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_4            (0x4UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_3            (0x3UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_2            (0x2UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_1            (0x1UL << MAC_STEP_XY_ADDRX_STEP_POS)
#define MAC_STEP_XY_ADDRX_STEP_PLUS_0            (0x0UL << MAC_STEP_XY_ADDRX_STEP_POS)

/*
 * Bit definition for MAC_STS[0] - MAC interrupt flag
 */
#define MAC_INTSTS_INTF_POS                      (0)
#define MAC_INTSTS_INTF_MSK                      (0x1UL << MAC_INTSTS_INTF_POS)
#define MAC_INTSTS_INT_FLAG                      (0x1UL << MAC_INTSTS_INTF_POS)

/*
 * Bit definition for MAC_CTRL[31] - MAC enhance mode control
 */
#define MAC_CTRL_MODE_SEL_CTRL_POS               (31)
#define MAC_CTRL_MODE_SEL_CTRL_MSK               (0x1UL << MAC_CTRL_MODE_SEL_CTRL_POS)
#define MAC_CTRL_MODE_SEL_CTRL_ENHANCE           (0x1UL << MAC_CTRL_MODE_SEL_CTRL_POS)
#define MAC_CTRL_MODE_SEL_CTRL_NORMAL            (0x0UL << MAC_CTRL_MODE_SEL_CTRL_POS)

/*
 * Bit definition for MAC_CTRL[29] - div interrupt control
 */
#define MAC_CTRL_ENH_DIV_INT_EN_POS              (29)
#define MAC_CTRL_ENH_DIV_INT_EN_MSK              (0x1UL << MAC_CTRL_ENH_DIV_INT_EN_POS)
#define MAC_CTRL_ENH_DIV_INT_ENABLE              (0x1UL << MAC_CTRL_ENH_DIV_INT_EN_POS)
#define MAC_CTRL_ENH_DIV_INT_DISABLE             (0x0UL << MAC_CTRL_ENH_DIV_INT_EN_POS)

/*
 * Bit definition for MAC_CTRL[13:8] - Output shift control
 */
#define MAC_CTRL_ENH_OUTPUT_SHIFT_POS            (8)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_MSK            (0x3FUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_LEFT_4         (0x3CUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_LEFT_3         (0x3DUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_LEFT_2         (0x3EUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_LEFT_1         (0x3FUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_32       (0x20UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_31       (0x1FUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_30       (0x1EUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_29       (0x1DUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_28       (0x1CUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_27       (0x1BUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_26       (0x1AUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_25       (0x19UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_24       (0x18UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_23       (0x17UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_22       (0x16UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_21       (0x15UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_20       (0x14UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_19       (0x13UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_18       (0x12UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_17       (0x11UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_16       (0x10UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_15       (0x0FUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_14       (0x0EUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_13       (0x0DUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_12       (0x0CUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_11       (0x0BUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_10       (0x0AUL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_9        (0x09UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_8        (0x08UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_7        (0x07UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_6        (0x06UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_5        (0x05UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_4        (0x04UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_3        (0x03UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_2        (0x02UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_1        (0x01UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)
#define MAC_CTRL_ENH_OUTPUT_SHIFT_RIGHT_0        (0x00UL << MAC_CTRL_ENH_OUTPUT_SHIFT_POS)

/*
 * Bit definition for enhance mode MAC_STEP_XY[25:16] - MAC AddressY Step control
 */
#define MAC_STEP_XY_ENH_ADDRY_STEP_POS           (16)
#define MAC_STEP_XY_ENH_ADDRY_STEP_MSK           (0x3FFUL << MAC_STEP_XY_ENH_ADDRY_STEP_POS)

/*
 * Bit definition for enhance mode MAC_STEP_XY[9:0] - MAC AddressX Step control
 */
#define MAC_STEP_XY_ENH_ADDRX_STEP_POS           (0)
#define MAC_STEP_XY_ENH_ADDRX_STEP_MSK           (0x3FFUL << MAC_STEP_XY_ENH_ADDRX_STEP_POS)

/*
 * Bit definition for MAC_MUL_DIV_STS[3] - divider working status
 */
#define MAC_MUL_DIV_STS_DIV_BUSY_POS             (3)
#define MAC_MUL_DIV_STS_DIV_BUSY_MSK             (0x1UL << MAC_MUL_DIV_STS_DIV_BUSY_POS)
#define MAC_MUL_DIV_STS_DIV_BUSY                 (0x1UL << MAC_MUL_DIV_STS_DIV_BUSY_POS)

/*
 * Bit definition for MAC_MUL_DIV_STS[2] - divide 0 flag
 */
#define MAC_MUL_DIV_STS_DIV0_FLAG_POS            (2)
#define MAC_MUL_DIV_STS_DIV0_FLAG_MSK            (0x1UL << MAC_MUL_DIV_STS_DIV0_FLAG_POS)
#define MAC_MUL_DIV_STS_DIV0_FLAG                (0x1UL << MAC_MUL_DIV_STS_DIV0_FLAG_POS)

/*
 * Bit definition for MAC_MUL_DIV_STS[1] - Multipier 2 working status
 */
#define MAC_MUL_DIV_STS_MUL2_BUSY_POS            (1)
#define MAC_MUL_DIV_STS_MUL2_BUSY_MSK            (0x1UL << MAC_MUL_DIV_STS_MUL2_BUSY_POS)
#define MAC_MUL_DIV_STS_MUL2_BUSY                (0x1UL << MAC_MUL_DIV_STS_MUL2_BUSY_POS)

/*
 * Bit definition for MAC_MUL_DIV_STS[0] - Multipier 1 working status
 */
#define MAC_MUL_DIV_STS_MUL1_BUSY_POS            (0)
#define MAC_MUL_DIV_STS_MUL1_BUSY_MSK            (0x1UL << MAC_MUL_DIV_STS_MUL1_BUSY_POS)
#define MAC_MUL_DIV_STS_MUL1_BUSY                (0x1UL << MAC_MUL_DIV_STS_MUL1_BUSY_POS)

/*
 * Bit definition for MAC_MUL12_C16[31:16] - MUL2 multiplier
 */
#define MAC_MUL12_C16_MUL2_C16_POS               (16)
#define MAC_MUL12_C16_MUL2_C16_MSK               (0xFFFFUL << MAC_MUL12_C16_MUL2_C16_POS)

/*
 * Bit definition for MAC_MUL12_C16[15:0] - MUL1 multiplier
 */
#define MAC_MUL12_C16_MUL1_C16_POS               (0)
#define MAC_MUL12_C16_MUL1_C16_MSK               (0xFFFFUL << MAC_MUL12_C16_MUL1_C16_POS)


/*---------------------------------------------------------------------------------------
 * CCP Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t TMCMP_CTRL;                    // CCP Timer Control and Compare Mode Control Register
    __IO uint32_t CAP_CTRL;                      // CCP Capture Mode Control Register
    __IO uint32_t PWM_CTRL;                      // CCP PWM Mode Control Register
    __I  uint32_t RESERVED;                      // Reserved
    __IO uint32_t INTEN;                         // CCP Interrupt Enable Register
    __IO uint32_t INTSTS;                        // CCP Interrupt Status Register
    __IO uint32_t TMR_COUNT;                     // CCP Timer Actual Value Register
    __IO uint32_t TMR_PLOAD;                     // CCP Timer Pre-Load Data Register
    __IO uint32_t CCPR0;                         // CCP Compare/Capture/PWM duty Register 0
    __IO uint32_t CCPR1;                         // CCP Compare/Capture/PWM duty Register 1
    __IO uint32_t CCPR2;                         // CCP Compare/Capture/PWM duty Register 2
    __IO uint32_t CCPR3;                         // CCP Compare/Capture/PWM duty Register 3
    __IO uint32_t PWM_DTIME;                     // CCP PWM Dead-Time Register
} CCP_TYPE_DEF;

/*
 * Bit definition for CCPx TMCMP_CTRL[15] - CCPx compare channel 3 enable
 */
#define CCP_TMCMPCTRL_CMP3_EN_POS                (15)
#define CCP_TMCMPCTRL_CMP3_EN_MSK                (0x1UL << CCP_TMCMPCTRL_CMP3_EN_POS)
#define CCP_TMCMPCTRL_CMP3_ENABLE                (0x1UL << CCP_TMCMPCTRL_CMP3_EN_POS)
#define CCP_TMCMPCTRL_CMP3_DISABLE               (0x0UL << CCP_TMCMPCTRL_CMP3_EN_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[14] - CCPx compare channel 2 enable
 */
#define CCP_TMCMPCTRL_CMP2_EN_POS                (14)
#define CCP_TMCMPCTRL_CMP2_EN_MSK                (0x1UL << CCP_TMCMPCTRL_CMP2_EN_POS)
#define CCP_TMCMPCTRL_CMP2_ENABLE                (0x1UL << CCP_TMCMPCTRL_CMP2_EN_POS)
#define CCP_TMCMPCTRL_CMP2_DISABLE               (0x0UL << CCP_TMCMPCTRL_CMP2_EN_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[13] - CCPx compare channel 1 enable
 */
#define CCP_TMCMPCTRL_CMP1_EN_POS                (13)
#define CCP_TMCMPCTRL_CMP1_EN_MSK                (0x1UL << CCP_TMCMPCTRL_CMP1_EN_POS)
#define CCP_TMCMPCTRL_CMP1_ENABLE                (0x1UL << CCP_TMCMPCTRL_CMP1_EN_POS)
#define CCP_TMCMPCTRL_CMP1_DISABLE               (0x0UL << CCP_TMCMPCTRL_CMP1_EN_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[12] - CCPx compare channel 0 enable
 */
#define CCP_TMCMPCTRL_CMP0_EN_POS                (12)
#define CCP_TMCMPCTRL_CMP0_EN_MSK                (0x1UL << CCP_TMCMPCTRL_CMP0_EN_POS)
#define CCP_TMCMPCTRL_CMP0_ENABLE                (0x1UL << CCP_TMCMPCTRL_CMP0_EN_POS)
#define CCP_TMCMPCTRL_CMP0_DISABLE               (0x0UL << CCP_TMCMPCTRL_CMP0_EN_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[11] - CCP1 PWM channel 3 control by CCPx Timer
 */
#define CCP1_PWM3_TIMER_CTRL_POS                 (11)
#define CCP1_PWM3_TIMER_CTRL_MSK                 (0x1UL << CCP1_PWM3_TIMER_CTRL_POS)
#define CCP1_PWM3_TIMER_CCP0                     (0x1UL << CCP1_PWM3_TIMER_CTRL_POS)
#define CCP1_PWM3_TIMER_CCP1                     (0x0UL << CCP1_PWM3_TIMER_CTRL_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[10] - CCP1 PWM channel 2 control by CCPx Timer
 */
#define CCP1_PWM2_TIMER_CTRL_POS                 (10)
#define CCP1_PWM2_TIMER_CTRL_MSK                 (0x1UL << CCP1_PWM2_TIMER_CTRL_POS)
#define CCP1_PWM2_TIMER_CCP0                     (0x1UL << CCP1_PWM2_TIMER_CTRL_POS)
#define CCP1_PWM2_TIMER_CCP1                     (0x0UL << CCP1_PWM2_TIMER_CTRL_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[8] - Timer up/down count selection
 */
#define CCP_TMCMPCTRL_TMR_UDC_SEL_POS            (8)
#define CCP_TMCMPCTRL_TMR_UDC_SEL_MSK            (0x1UL << CCP_TMCMPCTRL_TMR_UDC_SEL_POS)
#define CCP_TMCMPCTRL_TMR_UDC_UP_CNT             (0x1UL << CCP_TMCMPCTRL_TMR_UDC_SEL_POS)
#define CCP_TMCMPCTRL_TMR_UDC_DOWN_CNT           (0x0UL << CCP_TMCMPCTRL_TMR_UDC_SEL_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[7:4] - Timer input slock source selection
 */
#define CCP_TMCMPCTRL_TMR_CLK_SEL_POS            (4)
#define CCP_TMCMPCTRL_TMR_CLK_SEL_MSK            (0xFUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_TMC                (0x0FUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_TMB                (0x0EUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_TMA                (0x0DUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP1_TMCMPCTRL_TMR_CLK_EXT3              (0x0CUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP1_TMCMPCTRL_TMR_CLK_EXT2              (0x0BUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP0_TMCMPCTRL_TMR_CLK_EXT1              (0x0CUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP0_TMCMPCTRL_TMR_CLK_EXT0              (0x0BUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_IOSC_32K           (0x0AUL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_4096      (0x09UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_2048      (0x08UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_1024      (0x07UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_256       (0x06UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_64        (0x05UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_16        (0x04UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_8         (0x03UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_4         (0x02UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK_DIV_2         (0x01UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)
#define CCP_TMCMPCTRL_TMR_CLK_HCLK               (0x00UL << CCP_TMCMPCTRL_TMR_CLK_SEL_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[3:2] - Timer operating mode selection
 */
#define CCP_TMCMPCTRL_TMR_MODE_SEL_POS           (2)
#define CCP_TMCMPCTRL_TMR_MODE_SEL_MSK           (0x3UL << CCP_TMCMPCTRL_TMR_MODE_SEL_POS)
#define CCP_TMCMPCTRL_TMR_MODE_PWM               (0x2UL << CCP_TMCMPCTRL_TMR_MODE_SEL_POS)
#define CCP_TMCMPCTRL_TMR_MODE_CAPTURE           (0x1UL << CCP_TMCMPCTRL_TMR_MODE_SEL_POS)
#define CCP_TMCMPCTRL_TMR_MODE_COMPARE           (0x0UL << CCP_TMCMPCTRL_TMR_MODE_SEL_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[1] - Data direct write enable
 */
#define CCP_TMCMPCTRL_DIRECT_WR_EN_POS           (1)
#define CCP_TMCMPCTRL_DIRECT_WR_EN_MSK           (0x1UL << CCP_TMCMPCTRL_DIRECT_WR_EN_POS)
#define CCP_TMCMPCTRL_DIRECT_WR_ENABLE           (0x1UL << CCP_TMCMPCTRL_DIRECT_WR_EN_POS)
#define CCP_TMCMPCTRL_DIRECT_WR_DISABLE          (0x0UL << CCP_TMCMPCTRL_DIRECT_WR_EN_POS)

/*
 * Bit definition for CCPx TMCMP_CTRL[0] - Timer enable
 */
#define CCP_TMCMPCTRL_TMR_EN_POS                 (0)
#define CCP_TMCMPCTRL_TMR_EN_MSK                 (0x1UL << CCP_TMCMPCTRL_TMR_EN_POS)
#define CCP_TMCMPCTRL_TMR_ENABLE                 (0x1UL << CCP_TMCMPCTRL_TMR_EN_POS)
#define CCP_TMCMPCTRL_TMR_DISABLE                (0x0UL << CCP_TMCMPCTRL_TMR_EN_POS)

/*
 * Bit definition for CCPx CAP_CTRL[14:12] - Capture mode low pass filter selection
 */
#define CCP_CAPCTRL_CAP_LPF_SEL_POS              (12)
#define CCP_CAPCTRL_CAP_LPF_SEL_MSK              (0x7UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)
#define CCP_CAPCTRL_CAP_LPF_SEL_128HCLK          (0x7UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)
#define CCP_CAPCTRL_CAP_LPF_SEL_80HCLK           (0x6UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)
#define CCP_CAPCTRL_CAP_LPF_SEL_40HCLK           (0x5UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)
#define CCP_CAPCTRL_CAP_LPF_SEL_32HCLK           (0x4UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)
#define CCP_CAPCTRL_CAP_LPF_SEL_16HCLK           (0x3UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)
#define CCP_CAPCTRL_CAP_LPF_SEL_8HCLK            (0x2UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)
#define CCP_CAPCTRL_CAP_LPF_SEL_4HCLK            (0x1UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)
#define CCP_CAPCTRL_CAP_LPF_DISABLE              (0x0UL << CCP_CAPCTRL_CAP_LPF_SEL_POS)

/*
 * Bit definition for CCPx CAP_CTRL[11:10] - CCPx CAP3 sample edge selection
 */
#define CCP_CAPCTRL_CAP3_EDGE_POS                (10)
#define CCP_CAPCTRL_CAP3_EDGE_MSK                (0x3UL << CCP_CAPCTRL_CAP3_EDGE_POS)
#define CCP_CAPCTRL_CAP3_EDGE_DUAL               (0x3UL << CCP_CAPCTRL_CAP3_EDGE_POS)
#define CCP_CAPCTRL_CAP3_EDGE_FALL               (0x2UL << CCP_CAPCTRL_CAP3_EDGE_POS)
#define CCP_CAPCTRL_CAP3_EDGE_RISE               (0x1UL << CCP_CAPCTRL_CAP3_EDGE_POS)
#define CCP_CAPCTRL_CAP3_EDGE_NO_ACTIVE          (0x0UL << CCP_CAPCTRL_CAP3_EDGE_POS)

/*
 * Bit definition for CCPx CAP_CTRL[9:8] - CCPx CAP2 sample edge selection
 */
#define CCP_CAPCTRL_CAP2_EDGE_POS                (8)
#define CCP_CAPCTRL_CAP2_EDGE_MSK                (0x3UL << CCP_CAPCTRL_CAP2_EDGE_POS)
#define CCP_CAPCTRL_CAP2_EDGE_DUAL               (0x3UL << CCP_CAPCTRL_CAP2_EDGE_POS)
#define CCP_CAPCTRL_CAP2_EDGE_FALL               (0x2UL << CCP_CAPCTRL_CAP2_EDGE_POS)
#define CCP_CAPCTRL_CAP2_EDGE_RISE               (0x1UL << CCP_CAPCTRL_CAP2_EDGE_POS)
#define CCP_CAPCTRL_CAP2_EDGE_NO_ACTIVE          (0x0UL << CCP_CAPCTRL_CAP2_EDGE_POS)

/*
 * Bit definition for CCPx CAP_CTRL[7:6] - CCPx CAP1 sample edge selection
 */
#define CCP_CAPCTRL_CAP1_EDGE_POS                (6)
#define CCP_CAPCTRL_CAP1_EDGE_MSK                (0x3UL << CCP_CAPCTRL_CAP1_EDGE_POS)
#define CCP_CAPCTRL_CAP1_EDGE_DUAL               (0x3UL << CCP_CAPCTRL_CAP1_EDGE_POS)
#define CCP_CAPCTRL_CAP1_EDGE_FALL               (0x2UL << CCP_CAPCTRL_CAP1_EDGE_POS)
#define CCP_CAPCTRL_CAP1_EDGE_RISE               (0x1UL << CCP_CAPCTRL_CAP1_EDGE_POS)
#define CCP_CAPCTRL_CAP1_EDGE_NO_ACTIVE          (0x0UL << CCP_CAPCTRL_CAP1_EDGE_POS)

/*
 * Bit definition for CCPx CAP_CTRL[5:4] - CCPx CAP0 sample edge selection
 */
#define CCP_CAPCTRL_CAP0_EDGE_POS                (4)
#define CCP_CAPCTRL_CAP0_EDGE_MSK                (0x3UL << CCP_CAPCTRL_CAP0_EDGE_POS)
#define CCP_CAPCTRL_CAP0_EDGE_DUAL               (0x3UL << CCP_CAPCTRL_CAP0_EDGE_POS)
#define CCP_CAPCTRL_CAP0_EDGE_FALL               (0x2UL << CCP_CAPCTRL_CAP0_EDGE_POS)
#define CCP_CAPCTRL_CAP0_EDGE_RISE               (0x1UL << CCP_CAPCTRL_CAP0_EDGE_POS)
#define CCP_CAPCTRL_CAP0_EDGE_NO_ACTIVE          (0x0UL << CCP_CAPCTRL_CAP0_EDGE_POS)

/*
 * Bit definition for CCPx CAP_CTRL[3] - CCPx capture channel 3 enable
 */
#define CCP_CAPCTRL_CAP3_EN_POS                  (3)
#define CCP_CAPCTRL_CAP3_EN_MSK                  (0x1UL << CCP_CAPCTRL_CAP3_EN_POS)
#define CCP_CAPCTRL_CAP3_ENABLE                  (0x1UL << CCP_CAPCTRL_CAP3_EN_POS)
#define CCP_CAPCTRL_CAP3_DISABLE                 (0x0UL << CCP_CAPCTRL_CAP3_EN_POS)

/*
 * Bit definition for CCPx CAP_CTRL[2] - CCPx capture channel 2 enable
 */
#define CCP_CAPCTRL_CAP2_EN_POS                  (2)
#define CCP_CAPCTRL_CAP2_EN_MSK                  (0x1UL << CCP_CAPCTRL_CAP2_EN_POS)
#define CCP_CAPCTRL_CAP2_ENABLE                  (0x1UL << CCP_CAPCTRL_CAP2_EN_POS)
#define CCP_CAPCTRL_CAP2_DISABLE                 (0x0UL << CCP_CAPCTRL_CAP2_EN_POS)

/*
 * Bit definition for CCPx CAP_CTRL[1] - CCPx capture channel 1 enable
 */
#define CCP_CAPCTRL_CAP1_EN_POS                  (1)
#define CCP_CAPCTRL_CAP1_EN_MSK                  (0x1UL << CCP_CAPCTRL_CAP1_EN_POS)
#define CCP_CAPCTRL_CAP1_ENABLE                  (0x1UL << CCP_CAPCTRL_CAP1_EN_POS)
#define CCP_CAPCTRL_CAP1_DISABLE                 (0x0UL << CCP_CAPCTRL_CAP1_EN_POS)

/*
 * Bit definition for CCPx CAP_CTRL[0] - CCPx capture channel 0 enable
 */
#define CCP_CAPCTRL_CAP0_EN_POS                  (0)
#define CCP_CAPCTRL_CAP0_EN_MSK                  (0x1UL << CCP_CAPCTRL_CAP0_EN_POS)
#define CCP_CAPCTRL_CAP0_ENABLE                  (0x1UL << CCP_CAPCTRL_CAP0_EN_POS)
#define CCP_CAPCTRL_CAP0_DISABLE                 (0x0UL << CCP_CAPCTRL_CAP0_EN_POS)

/*
 * Bit definition for CCPx PWM_CTRL[10] - CCP1 PWM DMA Trigger enable
 */
#define CCP_PWMCTRL_DMA_EN_POS                   (10)
#define CCP_PWMCTRL_DMA_EN_MSK                   (0x1UL << CCP_PWMCTRL_DMA_EN_POS)
#define CCP_PWMCTRL_DMA_ENABLE                   (0x1UL << CCP_PWMCTRL_DMA_EN_POS)
#define CCP_PWMCTRL_DMA_DISABLE                  (0x0UL << CCP_PWMCTRL_DMA_EN_POS)

/*
 * Bit definition for CCPx PWM_CTRL[9] - CCPx PWM channel 2/3 operationg mode selection
 */
#define CCP_PWMCTRL_PWM23_CHM_POS                (9)
#define CCP_PWMCTRL_PWM23_CHM_MSK                (0x1UL << CCP_PWMCTRL_PWM23_CHM_POS)
#define CCP_PWMCTRL_PWM23_COMPLEMENT             (0x1UL << CCP_PWMCTRL_PWM23_CHM_POS)
#define CCP_PWMCTRL_PWM23_INDEPENDENT            (0x0UL << CCP_PWMCTRL_PWM23_CHM_POS)

/*
 * Bit definition for CCPx PWM_CTRL[8] - CCPx PWM channel 0/1 operationg mode selection
 */
#define CCP_PWMCTRL_PWM01_CHM_POS                (8)
#define CCP_PWMCTRL_PWM01_CHM_MSK                (0x1UL << CCP_PWMCTRL_PWM01_CHM_POS)
#define CCP_PWMCTRL_PWM01_COMPLEMENT             (0x1UL << CCP_PWMCTRL_PWM01_CHM_POS)
#define CCP_PWMCTRL_PWM01_INDEPENDENT            (0x0UL << CCP_PWMCTRL_PWM01_CHM_POS)

/*
 * Bit definition for CCPx PWM_CTRL[7] - CCPx PWM channel 3 polarity control
 */
#define CCP_PWMCTRL_PWM3_POL_POS                 (7)
#define CCP_PWMCTRL_PWM3_POL_MSK                 (0x1UL << CCP_PWMCTRL_PWM3_POL_POS)
#define CCP_PWMCTRL_PWM3_POL1                    (0x1UL << CCP_PWMCTRL_PWM3_POL_POS)
#define CCP_PWMCTRL_PWM3_POL0                    (0x0UL << CCP_PWMCTRL_PWM3_POL_POS)

/*
 * Bit definition for CCPx PWM_CTRL[6] - CCPx PWM channel 2 polarity control
 */
#define CCP_PWMCTRL_PWM2_POL_POS                 (6)
#define CCP_PWMCTRL_PWM2_POL_MSK                 (0x1UL << CCP_PWMCTRL_PWM2_POL_POS)
#define CCP_PWMCTRL_PWM2_POL1                    (0x1UL << CCP_PWMCTRL_PWM2_POL_POS)
#define CCP_PWMCTRL_PWM2_POL0                    (0x0UL << CCP_PWMCTRL_PWM2_POL_POS)

/*
 * Bit definition for CCPx PWM_CTRL[5] - CCPx PWM channel 1 polarity control
 */
#define CCP_PWMCTRL_PWM1_POL_POS                 (5)
#define CCP_PWMCTRL_PWM1_POL_MSK                 (0x1UL << CCP_PWMCTRL_PWM1_POL_POS)
#define CCP_PWMCTRL_PWM1_POL1                    (0x1UL << CCP_PWMCTRL_PWM1_POL_POS)
#define CCP_PWMCTRL_PWM1_POL0                    (0x0UL << CCP_PWMCTRL_PWM1_POL_POS)

/*
 * Bit definition for CCPx PWM_CTRL[4] - CCPx PWM channel 0 polarity control
 */
#define CCP_PWMCTRL_PWM0_POL_POS                 (4)
#define CCP_PWMCTRL_PWM0_POL_MSK                 (0x1UL << CCP_PWMCTRL_PWM0_POL_POS)
#define CCP_PWMCTRL_PWM0_POL1                    (0x1UL << CCP_PWMCTRL_PWM0_POL_POS)
#define CCP_PWMCTRL_PWM0_POL0                    (0x0UL << CCP_PWMCTRL_PWM0_POL_POS)

/*
 * Bit definition for CCPx PWM_CTRL[3] - CCPx PWM channel 3 enable
 */
#define CCP_PWMCTRL_PWM3_EN_POS                  (3)
#define CCP_PWMCTRL_PWM3_EN_MSK                  (0x1UL << CCP_PWMCTRL_PWM3_EN_POS)
#define CCP_PWMCTRL_PWM3_ENABLE                  (0x1UL << CCP_PWMCTRL_PWM3_EN_POS)
#define CCP_PWMCTRL_PWM3_DISABLE                 (0x0UL << CCP_PWMCTRL_PWM3_EN_POS)

/*
 * Bit definition for CCPx PWM_CTRL[2] - CCPx PWM channel 2 enable
 */
#define CCP_PWMCTRL_PWM2_EN_POS                  (2)
#define CCP_PWMCTRL_PWM2_EN_MSK                  (0x1UL << CCP_PWMCTRL_PWM2_EN_POS)
#define CCP_PWMCTRL_PWM2_ENABLE                  (0x1UL << CCP_PWMCTRL_PWM2_EN_POS)
#define CCP_PWMCTRL_PWM2_DISABLE                 (0x0UL << CCP_PWMCTRL_PWM2_EN_POS)

/*
 * Bit definition for CCPx PWM_CTRL[1] - CCPx PWM channel 1 enable
 */
#define CCP_PWMCTRL_PWM1_EN_POS                  (1)
#define CCP_PWMCTRL_PWM1_EN_MSK                  (0x1UL << CCP_PWMCTRL_PWM1_EN_POS)
#define CCP_PWMCTRL_PWM1_ENABLE                  (0x1UL << CCP_PWMCTRL_PWM1_EN_POS)
#define CCP_PWMCTRL_PWM1_DISABLE                 (0x0UL << CCP_PWMCTRL_PWM1_EN_POS)

/*
 * Bit definition for CCPx PWM_CTRL[0] - CCPx PWM channel 0 enable
 */
#define CCP_PWMCTRL_PWM0_EN_POS                  (0)
#define CCP_PWMCTRL_PWM0_EN_MSK                  (0x1UL << CCP_PWMCTRL_PWM0_EN_POS)
#define CCP_PWMCTRL_PWM0_ENABLE                  (0x1UL << CCP_PWMCTRL_PWM0_EN_POS)
#define CCP_PWMCTRL_PWM0_DISABLE                 (0x0UL << CCP_PWMCTRL_PWM0_EN_POS)

/*
 * Bit definition for CCPx INTEN[8] - CCPx Timer interrupt enable
 */
#define CCP_INTEN_TMR_INT_EN_POS                 (8)
#define CCP_INTEN_TMR_INT_EN_MSK                 (0x1UL << CCP_INTEN_TMR_INT_EN_POS)
#define CCP_INTEN_TMR_INT_ENABLE                 (0x1UL << CCP_INTEN_TMR_INT_EN_POS)
#define CCP_INTEN_TMR_INT_DISABLE                (0x0UL << CCP_INTEN_TMR_INT_EN_POS)

/*
 * Bit definition for CCPx INTEN[7] - CCPx capture channel 3 interrupt enable
 */
#define CCP_INTEN_CAP3_INT_EN_POS                (7)
#define CCP_INTEN_CAP3_INT_EN_MSK                (0x1UL << CCP_INTEN_CAP3_INT_EN_POS)
#define CCP_INTEN_CAP3_INT_ENABLE                (0x1UL << CCP_INTEN_CAP3_INT_EN_POS)
#define CCP_INTEN_CAP3_INT_DISABLE               (0x0UL << CCP_INTEN_CAP3_INT_EN_POS)

/*
 * Bit definition for CCPx INTEN[6] - CCPx capture channel 2 interrupt enable
 */
#define CCP_INTEN_CAP2_INT_EN_POS                (6)
#define CCP_INTEN_CAP2_INT_EN_MSK                (0x1UL << CCP_INTEN_CAP2_INT_EN_POS)
#define CCP_INTEN_CAP2_INT_ENABLE                (0x1UL << CCP_INTEN_CAP2_INT_EN_POS)
#define CCP_INTEN_CAP2_INT_DISABLE               (0x0UL << CCP_INTEN_CAP2_INT_EN_POS)

/*
 * Bit definition for CCPx INTEN[5] - CCPx capture channel 1 interrupt enable
 */
#define CCP_INTEN_CAP1_INT_EN_POS                (5)
#define CCP_INTEN_CAP1_INT_EN_MSK                (0x1UL << CCP_INTEN_CAP1_INT_EN_POS)
#define CCP_INTEN_CAP1_INT_ENABLE                (0x1UL << CCP_INTEN_CAP1_INT_EN_POS)
#define CCP_INTEN_CAP1_INT_DISABLE               (0x0UL << CCP_INTEN_CAP1_INT_EN_POS)

/*
 * Bit definition for CCPx INTEN[4] - CCPx capture channel 0 interrupt enable
 */
#define CCP_INTEN_CAP0_INT_EN_POS                (4)
#define CCP_INTEN_CAP0_INT_EN_MSK                (0x1UL << CCP_INTEN_CAP0_INT_EN_POS)
#define CCP_INTEN_CAP0_INT_ENABLE                (0x1UL << CCP_INTEN_CAP0_INT_EN_POS)
#define CCP_INTEN_CAP0_INT_DISABLE               (0x0UL << CCP_INTEN_CAP0_INT_EN_POS)

/*
 * Bit definition for CCPx INTEN[3] - CCPx compare channel 3 interrupt enable
 */
#define CCP_INTEN_CMP3_INT_EN_POS                (3)
#define CCP_INTEN_CMP3_INT_EN_MSK                (0x1UL << CCP_INTEN_CMP3_INT_EN_POS)
#define CCP_INTEN_CMP3_INT_ENABLE                (0x1UL << CCP_INTEN_CMP3_INT_EN_POS)
#define CCP_INTEN_CMP3_INT_DISABLE               (0x0UL << CCP_INTEN_CMP3_INT_EN_POS)

/*
 * Bit definition for CCPx INTEN[2] - CCPx compare channel 2 interrupt enable
 */
#define CCP_INTEN_CMP2_INT_EN_POS                (2)
#define CCP_INTEN_CMP2_INT_EN_MSK                (0x1UL << CCP_INTEN_CMP2_INT_EN_POS)
#define CCP_INTEN_CMP2_INT_ENABLE                (0x1UL << CCP_INTEN_CMP2_INT_EN_POS)
#define CCP_INTEN_CMP2_INT_DISABLE               (0x0UL << CCP_INTEN_CMP2_INT_EN_POS)

/*
 * Bit definition for CCPx INTEN[1] - CCPx compare channel 1 interrupt enable
 */
#define CCP_INTEN_CMP1_INT_EN_POS                (1)
#define CCP_INTEN_CMP1_INT_EN_MSK                (0x1UL << CCP_INTEN_CMP1_INT_EN_POS)
#define CCP_INTEN_CMP1_INT_ENABLE                (0x1UL << CCP_INTEN_CMP1_INT_EN_POS)
#define CCP_INTEN_CMP1_INT_DISABLE               (0x0UL << CCP_INTEN_CMP1_INT_EN_POS)

/*
 * Bit definition for CCPx INTEN[0] - CCPx compare channel 0 interrupt enable
 */
#define CCP_INTEN_CMP0_INT_EN_POS                (0)
#define CCP_INTEN_CMP0_INT_EN_MSK                (0x1UL << CCP_INTEN_CMP0_INT_EN_POS)
#define CCP_INTEN_CMP0_INT_ENABLE                (0x1UL << CCP_INTEN_CMP0_INT_EN_POS)
#define CCP_INTEN_CMP0_INT_DISABLE               (0x0UL << CCP_INTEN_CMP0_INT_EN_POS)

/*
 * Bit definition for CCPx INTSTS[8] - CCPx Timer interrupt flag
 */
#define CCP_INTEN_TMR_INTF_POS                   (8)
#define CCP_INTEN_TMR_INTF_MSK                   (0x1UL << CCP_INTEN_TMR_INTF_POS)
#define CCP_INTEN_TMR_INT_FLAG                   (0x1UL << CCP_INTEN_TMR_INTF_POS)

/*
 * Bit definition for CCPx INTSTS[7] - CCPx capture channel 3 interrupt flag
 */
#define CCP_INTSTS_CAP3_INTF_POS                 (7)
#define CCP_INTSTS_CAP3_INTF_MSK                 (0x1UL << CCP_INTSTS_CAP3_INTF_POS)
#define CCP_INTSTS_CAP3_INT_FLAG                 (0x1UL << CCP_INTSTS_CAP3_INTF_POS)

/*
 * Bit definition for CCPx INTSTS[6] - CCPx capture channel 2 interrupt flag
 */
#define CCP_INTSTS_CAP2_INTF_POS                 (6)
#define CCP_INTSTS_CAP2_INTF_MSK                 (0x1UL << CCP_INTSTS_CAP2_INTF_POS)
#define CCP_INTSTS_CAP2_INT_FLAG                 (0x1UL << CCP_INTSTS_CAP2_INTF_POS)

/*
 * Bit definition for CCPx INTSTS[5] - CCPx capture channel 1 interrupt flag
 */
#define CCP_INTSTS_CAP1_INTF_POS                 (5)
#define CCP_INTSTS_CAP1_INTF_MSK                 (0x1UL << CCP_INTSTS_CAP1_INTF_POS)
#define CCP_INTSTS_CAP1_INT_FLAG                 (0x1UL << CCP_INTSTS_CAP1_INTF_POS)

/*
 * Bit definition for CCPx INTSTS[4] - CCPx capture channel 0 interrupt flag
 */
#define CCP_INTSTS_CAP0_INTF_POS                 (4)
#define CCP_INTSTS_CAP0_INTF_MSK                 (0x1UL << CCP_INTSTS_CAP0_INTF_POS)
#define CCP_INTSTS_CAP0_INT_FLAG                 (0x1UL << CCP_INTSTS_CAP0_INTF_POS)

/*
 * Bit definition for CCPx INTSTS[3] - CCPx compare channel 3 interrupt flag
 */
#define CCP_INTSTS_CMP3_INTF_POS                 (3)
#define CCP_INTSTS_CMP3_INTF_MSK                 (0x1UL << CCP_INTSTS_CMP3_INTF_POS)
#define CCP_INTSTS_CMP3_INT_FLAG                 (0x1UL << CCP_INTSTS_CMP3_INTF_POS)

/*
 * Bit definition for CCPx INTSTS[2] - CCPx compare channel 2 interrupt flag
 */
#define CCP_INTSTS_CMP2_INTF_POS                 (2)
#define CCP_INTSTS_CMP2_INTF_MSK                 (0x1UL << CCP_INTSTS_CMP2_INTF_POS)
#define CCP_INTSTS_CMP2_INT_FLAG                 (0x1UL << CCP_INTSTS_CMP2_INTF_POS)

/*
 * Bit definition for CCPx INTSTS[1] - CCPx compare channel 1 interrupt flag
 */
#define CCP_INTSTS_CMP1_INTF_POS                 (1)
#define CCP_INTSTS_CMP1_INTF_MSK                 (0x1UL << CCP_INTSTS_CMP1_INTF_POS)
#define CCP_INTSTS_CMP1_INT_FLAG                 (0x1UL << CCP_INTSTS_CMP1_INTF_POS)

/*
 * Bit definition for CCPx INTSTS[0] - CCPx compare channel 0 interrupt flag
 */
#define CCP_INTSTS_CMP0_INTF_POS                 (0)
#define CCP_INTSTS_CMP0_INTF_MSK                 (0x1UL << CCP_INTSTS_CMP0_INTF_POS)
#define CCP_INTSTS_CMP0_INT_FLAG                 (0x1UL << CCP_INTSTS_CMP0_INTF_POS)


/*---------------------------------------------------------------------------------------
 * TIMER Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t PLOAD;                         // Timer Pre-Load Data Register
    __IO uint32_t COUNT;                         // Timer Actual Value Register
    __IO uint32_t CTRL;                          // Timer Baud Control Register
    __IO uint32_t CAPSRC;                        // Timer Capture Mode Source Selection Register
} TIMER_TYPE_DEF;

typedef struct
{
    __IO uint32_t INTEN;                         // Timer Interrupt Enable Register
    __IO uint32_t INTSTS;                        // Timer Interrupt Status Register
} TIMER_INT_TYPE_DEF;

/*
 * Bit definition for TMx_PLOAD[15:0] - Timer Pre-Load Data
 */
#define TIMER_PLOAD_PRELOAD_DATA_POS             (0)
#define TIMER_PLOAD_PRELOAD_DATA_MSK             (0xFFFFUL << TIMER_PLOAD_PRELOAD_DATA_POS)

/*
 * Bit definition for TMx_CTRL[15:12] - Timer Interrupt divider
 */
#define TIMER_CTRL_TMINT_DIV_POS                 (12)
#define TIMER_CTRL_TMINT_DIV_MSK                 (0xFUL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_16                  (0xFUL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_15                  (0xEUL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_14                  (0xDUL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_13                  (0xCUL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_12                  (0xBUL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_11                  (0xAUL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_10                  (0x9UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_9                   (0x8UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_8                   (0x7UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_7                   (0x6UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_6                   (0x5UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_5                   (0x4UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_4                   (0x3UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_3                   (0x2UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_2                   (0x1UL << TIMER_CTRL_TMINT_DIV_POS)
#define TIMER_CTRL_TMINT_DIV_1                   (0x0UL << TIMER_CTRL_TMINT_DIV_POS)

/*
 * Bit definition for TMx_CTRL[11] - Capture end edge switch
 */
#define TIMER_CTRL_CAP_ENDSW_POS                 (11)
#define TIMER_CTRL_CAP_ENDSW_MSK                 (0x1UL << TIMER_CTRL_CAP_ENDSW_POS)
#define TIMER_CTRL_CAP_ENDSW_FALL_EDGE           (0x1UL << TIMER_CTRL_CAP_ENDSW_POS)
#define TIMER_CTRL_CAP_ENDSW_RISE_EDGE           (0x0UL << TIMER_CTRL_CAP_ENDSW_POS)

/*
 * Bit definition for TMx_CTRL[10] - Capture end flag
 */
#define TIMER_CTRL_CAP_END_POS                   (10)
#define TIMER_CTRL_CAP_END_MSK                   (0x1UL << TIMER_CTRL_CAP_END_POS)
#define TIMER_CTRL_CAP_END_FLAG                  (0x1UL << TIMER_CTRL_CAP_END_POS)

/*
 * Bit definition for TMx_CTRL[9] - Capture start
 */
#define TIMER_CTRL_CAP_START_POS                 (9)
#define TIMER_CTRL_CAP_START_MSK                 (0x1UL << TIMER_CTRL_CAP_START_POS)
#define TIMER_CTRL_CAP_START                     (0x1UL << TIMER_CTRL_CAP_START_POS)

/*
 * Bit definition for TMx_CTRL[8] - Capture mode enable
 */
#define TIMER_CTRL_CAPMODE_POS                   (8)
#define TIMER_CTRL_CAPMODE_MSK                   (0x1UL << TIMER_CTRL_CAPMODE_POS)
#define TIMER_CTRL_CAPMODE_ENABLE                (0x1UL << TIMER_CTRL_CAPMODE_POS)
#define TIMER_CTRL_CAPMODE_DISABLE               (0x0UL << TIMER_CTRL_CAPMODE_POS)

/*
 * Bit definition for TMx_CTRL[7:5] - Timer input clock source selection
 */
#define TIMER_CTRL_CLK_SEL_POS                   (5)
#define TIMER_CTRL_CLK_SEL_MSK                   (0x7UL << TIMER_CTRL_CLK_SEL_POS)
#define TIMER_CTRL_CLK_SEL_32K                   (0x7UL << TIMER_CTRL_CLK_SEL_POS)
#define TIMER_CTRL_CLK_SEL_EXTCLK                (0x6UL << TIMER_CTRL_CLK_SEL_POS)
#define TIMER_CTRL_CLK_SEL_HCLK_DIV_32           (0x5UL << TIMER_CTRL_CLK_SEL_POS)
#define TIMER_CTRL_CLK_SEL_HCLK_DIV_16           (0x4UL << TIMER_CTRL_CLK_SEL_POS)
#define TIMER_CTRL_CLK_SEL_HCLK_DIV_8            (0x3UL << TIMER_CTRL_CLK_SEL_POS)
#define TIMER_CTRL_CLK_SEL_HCLK_DIV_4            (0x2UL << TIMER_CTRL_CLK_SEL_POS)
#define TIMER_CTRL_CLK_SEL_HCLK_DIV_2            (0x1UL << TIMER_CTRL_CLK_SEL_POS)
#define TIMER_CTRL_CLK_SEL_HCLK                  (0x0UL << TIMER_CTRL_CLK_SEL_POS)

/*
 * Bit definition for TMx_CTRL[4:2] - Capture mode low pass filter selection
 */
#define TIMER_CTRL_CAP_LPF_SEL_POS               (2)
#define TIMER_CTRL_CAP_LPF_SEL_MSK               (0x7UL << TIMER_CTRL_CAP_LPF_SEL_POS)
#define TIMER_CTRL_CAP_LPF_SEL_128HCLK           (0x7UL << TIMER_CTRL_CAP_LPF_SEL_POS)
#define TIMER_CTRL_CAP_LPF_SEL_80HCLK            (0x6UL << TIMER_CTRL_CAP_LPF_SEL_POS)
#define TIMER_CTRL_CAP_LPF_SEL_40HCLK            (0x5UL << TIMER_CTRL_CAP_LPF_SEL_POS)
#define TIMER_CTRL_CAP_LPF_SEL_32HCLK            (0x4UL << TIMER_CTRL_CAP_LPF_SEL_POS)
#define TIMER_CTRL_CAP_LPF_SEL_16HCLK            (0x3UL << TIMER_CTRL_CAP_LPF_SEL_POS)
#define TIMER_CTRL_CAP_LPF_SEL_8HCLK             (0x2UL << TIMER_CTRL_CAP_LPF_SEL_POS)
#define TIMER_CTRL_CAP_LPF_SEL_4HCLK             (0x1UL << TIMER_CTRL_CAP_LPF_SEL_POS)
#define TIMER_CTRL_CAP_LPF_DISABLE               (0x0UL << TIMER_CTRL_CAP_LPF_SEL_POS)

/*
 * Bit definition for TMx_CTRL[1] - Timer reload
 */
#define TIMER_CTRL_TM_RELOAD_POS                 (1)
#define TIMER_CTRL_TM_RELOAD_MSK                 (0x1UL << TIMER_CTRL_TM_RELOAD_POS)
#define TIMER_CTRL_TM_RELOAD                     (0x1UL << TIMER_CTRL_TM_RELOAD_POS)

/*
 * Bit definition for TMx_CTRL[0] - Timer enable
 */
#define TIMER_CTRL_TM_EN_POS                     (0)
#define TIMER_CTRL_TM_EN_MSK                     (0x1UL << TIMER_CTRL_TM_EN_POS)
#define TIMER_CTRL_TM_ENABLE                     (0x1UL << TIMER_CTRL_TM_EN_POS)
#define TIMER_CTRL_TM_DISABLE                    (0x0UL << TIMER_CTRL_TM_EN_POS)

/*
 * Bit definition for TMx_CAPSRC[4:0] - Timer Capture mode source selection
 */
#define TIMER_CAPSRC_IOSEL_POS                   (0)
#define TIMER_CAPSRC_IOSEL_MSK                   (0x1FUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA31                 (0x1FUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA30                 (0x1EUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA29                 (0x1DUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA28                 (0x1CUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA27                 (0x1BUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA26                 (0x1AUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA25                 (0x19UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA24                 (0x18UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA23                 (0x17UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA22                 (0x16UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA21                 (0x15UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA20                 (0x14UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA19                 (0x13UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA18                 (0x12UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA17                 (0x11UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA16                 (0x10UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA15                 (0xFUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA14                 (0xEUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA13                 (0xDUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA12                 (0xCUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA11                 (0xBUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA10                 (0xAUL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA9                  (0x9UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA8                  (0x8UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA7                  (0x7UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA6                  (0x6UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA5                  (0x5UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA4                  (0x4UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA3                  (0x3UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA2                  (0x2UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA1                  (0x1UL << TIMER_CAPSRC_IOSEL_POS)
#define TIMER_CAPSRC_IOSEL_IOA0                  (0x0UL << TIMER_CAPSRC_IOSEL_POS)

/*
 * Bit definition for TM_INTEN[5] - TM2 capture mode interrupt enable
 */
#define TIMER_INTEN_TM2_CAPINT_EN_POS            (5)
#define TIMER_INTEN_TM2_CAPINT_EN_MSK            (0x1UL << TIMER_INTEN_TM2_CAPINT_EN_POS)
#define TIMER_INTEN_TM2_CAPINT_ENABLE            (0x1UL << TIMER_INTEN_TM2_CAPINT_EN_POS)
#define TIMER_INTEN_TM2_CAPINT_DISABLE           (0x0UL << TIMER_INTEN_TM2_CAPINT_EN_POS)

/*
 * Bit definition for TM_INTEN[4] - TM2 interrupt enable
 */
#define TIMER_INTEN_TM2_INT_EN_POS               (4)
#define TIMER_INTEN_TM2_INT_EN_MSK               (0x1UL << TIMER_INTEN_TM2_INT_EN_POS)
#define TIMER_INTEN_TM2_INT_ENABLE               (0x1UL << TIMER_INTEN_TM2_INT_EN_POS)
#define TIMER_INTEN_TM2_INT_DISABLE              (0x0UL << TIMER_INTEN_TM2_INT_EN_POS)

/*
 * Bit definition for TM_INTEN[3] - TM1 capture mode interrupt enable
 */
#define TIMER_INTEN_TM1_CAPINT_EN_POS            (3)
#define TIMER_INTEN_TM1_CAPINT_EN_MSK            (0x1UL << TIMER_INTEN_TM1_CAPINT_EN_POS)
#define TIMER_INTEN_TM1_CAPINT_ENABLE            (0x1UL << TIMER_INTEN_TM1_CAPINT_EN_POS)
#define TIMER_INTEN_TM1_CAPINT_DISABLE           (0x0UL << TIMER_INTEN_TM1_CAPINT_EN_POS)

/*
 * Bit definition for TM_INTEN[2] - TM1 interrupt enable
 */
#define TIMER_INTEN_TM1_INT_EN_POS               (2)
#define TIMER_INTEN_TM1_INT_EN_MSK               (0x1UL << TIMER_INTEN_TM1_INT_EN_POS)
#define TIMER_INTEN_TM1_INT_ENABLE               (0x1UL << TIMER_INTEN_TM1_INT_EN_POS)
#define TIMER_INTEN_TM1_INT_DISABLE              (0x0UL << TIMER_INTEN_TM1_INT_EN_POS)

/*
 * Bit definition for TM_INTEN[1] - TM0 capture mode interrupt enable
 */
#define TIMER_INTEN_TM0_CAPINT_EN_POS            (1)
#define TIMER_INTEN_TM0_CAPINT_EN_MSK            (0x1UL << TIMER_INTEN_TM0_CAPINT_EN_POS)
#define TIMER_INTEN_TM0_CAPINT_ENABLE            (0x1UL << TIMER_INTEN_TM0_CAPINT_EN_POS)
#define TIMER_INTEN_TM0_CAPINT_DISABLE           (0x0UL << TIMER_INTEN_TM0_CAPINT_EN_POS)

/*
 * Bit definition for TM_INTEN[0] - TM0 interrupt enable
 */
#define TIMER_INTEN_TM0_INT_EN_POS               (0)
#define TIMER_INTEN_TM0_INT_EN_MSK               (0x1UL << TIMER_INTEN_TM0_INT_EN_POS)
#define TIMER_INTEN_TM0_INT_ENABLE               (0x1UL << TIMER_INTEN_TM0_INT_EN_POS)
#define TIMER_INTEN_TM0_INT_DISABLE              (0x0UL << TIMER_INTEN_TM0_INT_EN_POS)

/*
 * Bit definition for TM_INTSTS[5] - TM2 capture mode interrupt flag
 */
#define TIMER_INTSTS_TM2_CAPINTF_POS             (5)
#define TIMER_INTSTS_TM2_CAPINTF_MSK             (0x1UL << TIMER_INTSTS_TM2_CAPINTF_POS)
#define TIMER_INTSTS_TM2_CAPINT_FLAG             (0x1UL << TIMER_INTSTS_TM2_CAPINTF_POS)

/*
 * Bit definition for TM_INTSTS[4] - TM2 interrupt flag
 */
#define TIMER_INTSTS_TM2_INTF_POS                (4)
#define TIMER_INTSTS_TM2_INTF_MSK                (0x1UL << TIMER_INTSTS_TM2_INTF_POS)
#define TIMER_INTSTS_TM2_INT_FLAG                (0x1UL << TIMER_INTSTS_TM2_INTF_POS)

/*
 * Bit definition for TM_INTSTS[3] - TM1 capture mode interrupt flag
 */
#define TIMER_INTSTS_TM1_CAPINTF_POS             (3)
#define TIMER_INTSTS_TM1_CAPINTF_MSK             (0x1UL << TIMER_INTSTS_TM1_CAPINTF_POS)
#define TIMER_INTSTS_TM1_CAPINT_FLAG             (0x1UL << TIMER_INTSTS_TM1_CAPINTF_POS)

/*
 * Bit definition for TM_INTSTS[2] - TM1 interrupt flag
 */
#define TIMER_INTSTS_TM1_INTF_POS                (2)
#define TIMER_INTSTS_TM1_INTF_MSK                (0x1UL << TIMER_INTSTS_TM1_INTF_POS)
#define TIMER_INTSTS_TM1_INT_FLAG                (0x1UL << TIMER_INTSTS_TM1_INTF_POS)

/*
 * Bit definition for TM_INTSTS[1] - TM0 capture mode interrupt flag
 */
#define TIMER_INTSTS_TM0_CAPINTF_POS             (1)
#define TIMER_INTSTS_TM0_CAPINTF_MSK             (0x1UL << TIMER_INTSTS_TM0_CAPINTF_POS)
#define TIMER_INTSTS_TM0_CAPINT_FLAG             (0x1UL << TIMER_INTSTS_TM0_CAPINTF_POS)

/*
 * Bit definition for TM_INTSTS[0] - TM0 interrupt flag
 */
#define TIMER_INTSTS_TM0_INTF_POS                (0)
#define TIMER_INTSTS_TM0_INTF_MSK                (0x1UL << TIMER_INTSTS_TM0_INTF_POS)
#define TIMER_INTSTS_TM0_INT_FLAG                (0x1UL << TIMER_INTSTS_TM0_INTF_POS)


/*---------------------------------------------------------------------------------------
 * SPIFC Control Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL0;                         // SPIFC Control Register0          0
    __IO uint32_t CMD;                           // SPIFC Command Register           4
    __IO uint32_t PARA;                          // SPIFC Control Register1          8
    __IO uint32_t ADDRL;                         // SPIFC Address RegisterL          C
    __IO uint32_t ADDRH;                         // SPIFC Address RegisterH          10
    __IO uint32_t TX_DATA;                       // SPIFC TX Data Register           14
    __IO uint32_t RX_DATA;                       // SPIFC RX Data Register           18
    __IO uint32_t TX_BC;                         // SPIFC TX Byte Count Register     1C
    __IO uint32_t RX_BC;                         // SPIFC RX Byte Count Register     20
    __IO uint32_t TIMING;                        // SPIFC Timing Delay Register      24
    __IO uint32_t Reserve1;                      // SPIFC SPIFC Reserve Register1    28
    __IO uint32_t CTRL1;                         // SPIFC Control Register1          2C
    __IO uint32_t Reserve2;                      // SPIFC SPIFC Reserve Register2    30
    __IO uint32_t CTRL2;                         // SPIFC SPIFC Reserve Register3    34
    __IO uint32_t TX_DATA32;                     // SPIFC TX Word Data Register      38
    __IO uint32_t Reserve3;                      // SPIFC Reserve Register           3C
    __IO uint32_t SCRAMBLE_CTRL;                 // SPIFC Scramble Control Register  40
} SPIFC_TYPE_DEF;

/*
 * Bit definition for SPIFC_CTRL0[15] - SPIFC TX done flag
 */
#define SPIFC_CTRL0_TX_DONE_FLAG_POS             (15)
#define SPIFC_CTRL0_TX_DONE_FLAG_MSK             (1UL << SPIFC_CTRL0_TX_DONE_FLAG_POS)
#define SPIFC_CTRL0_TX_DONE_FLAG                 (1UL << SPIFC_CTRL0_TX_DONE_FLAG_POS)

/*
 * Bit definition for SPIFC_CTRL0[14] - SPIFC RX FIFO empty flag
 */
#define SPIFC_CTRL0_RX_FIFO_EMP_FLAG_POS         (14)
#define SPIFC_CTRL0_RX_FIFO_EMP_FLAG_MSK         (1UL << SPIFC_CTRL0_RX_FIFO_EMP_FLAG_POS)
#define SPIFC_CTRL0_RX_FIFO_EMP_FLAG             (1UL << SPIFC_CTRL0_RX_FIFO_EMP_FLAG_POS)

/*
 * Bit definition for SPIFC_CTRL0[9] - Ignore last clock
 */
#define SPIFC_CTRL0_IGNORE_LAST_CLK_POS          (9)
#define SPIFC_CTRL0_IGNORE_LAST_CLK_MSK          (1UL << SPIFC_CTRL0_IGNORE_LAST_CLK_POS)
#define SPIFC_CTRL0_IGNORE_LAST_CLK_ENABLE       (1UL << SPIFC_CTRL0_IGNORE_LAST_CLK_POS)
#define SPIFC_CTRL0_IGNORE_LAST_CLK_DISABLE      (0UL << SPIFC_CTRL0_IGNORE_LAST_CLK_POS)

/*
 * Bit definition for SPIFC_CTRL0[8] - SPIFC operation mode
 */
#define SPIFC_CTRL0_OPERATION_MODE_POS           (8)
#define SPIFC_CTRL0_OPERATION_MODE_MSK           (1UL << SPIFC_CTRL0_OPERATION_MODE_POS)
#define SPIFC_CTRL0_MANUAL_MODE                  (1UL << SPIFC_CTRL0_OPERATION_MODE_POS)
#define SPIFC_CTRL0_AUTO_MODE                    (0UL << SPIFC_CTRL0_OPERATION_MODE_POS)

/*
 * Bit definition for SPIFC_CTRL0[7:6] - IO bit width definition for command
 */
#define SPIFC_CTRL0_CMIO_POS                     (6)
#define SPIFC_CTRL0_CMIO_MSK                     (3UL << SPIFC_CTRL0_CMIO_POS)
#define SPIFC_CTRL0_CMIO_4BIT                    (2UL << SPIFC_CTRL0_CMIO_POS)
#define SPIFC_CTRL0_CMIO_2BIT                    (1UL << SPIFC_CTRL0_CMIO_POS)
#define SPIFC_CTRL0_CMIO_1BIT                    (0UL << SPIFC_CTRL0_CMIO_POS)

/*
 * Bit definition for SPIFC_CTRL0[5:4] - IO bit width definition for address
 */
#define SPIFC_CTRL0_AMIO_POS                     (4)
#define SPIFC_CTRL0_AMIO_MSK                     (3UL << SPIFC_CTRL0_AMIO_POS)
#define SPIFC_CTRL0_AMIO_4BIT                    (2UL << SPIFC_CTRL0_AMIO_POS)
#define SPIFC_CTRL0_AMIO_2BIT                    (1UL << SPIFC_CTRL0_AMIO_POS)
#define SPIFC_CTRL0_AMIO_1BIT                    (0UL << SPIFC_CTRL0_AMIO_POS)

/*
 * Bit definition for SPIFC_CTRL0[3:2] - Multi-IO mode,IO bit width definition for TX & RX data.
 */
#define SPIFC_CTRL0_MIO_POS                      (2)
#define SPIFC_CTRL0_MIO_MSK                      (0x3UL << SPIFC_CTRL0_MIO_POS)
#define SPIFC_CTRL0_MIO_4BIT                     (0x2UL << SPIFC_CTRL0_MIO_POS)
#define SPIFC_CTRL0_MIO_2BIT                     (0x1UL << SPIFC_CTRL0_MIO_POS)
#define SPIFC_CTRL0_MIO_1BIT                     (0x0UL << SPIFC_CTRL0_MIO_POS)

/*
 * Bit definition for SPIFC_CTRL0[1] - SPI clock state
 */
#define SPIFC_CTRL0_CLK_STATE_POS                (1)
#define SPIFC_CTRL0_CLK_STATE_MSK                (0x1UL << SPIFC_CTRL0_MIO_POS)
#define SPIFC_CTRL0_CLK_STATE_HIGH               (0x1UL << SPIFC_CTRL0_MIO_POS)
#define SPIFC_CTRL0_CLK_STATE_LOW                (0x0UL << SPIFC_CTRL0_MIO_POS)

/*
 * Bit definition for SPIFC_CTRL0[0] - SPIFC pending flag
 */
#define SPIFC_CTRL0_PENDING_FLAG_POS             (0)
#define SPIFC_CTRL0_PENDING_FLAG_MSK             (0x1UL << SPIFC_CTRL0_PENDING_FLAG_POS)
#define SPIFC_CTRL0_PENDING_FLAG                 (0x1UL << SPIFC_CTRL0_PENDING_FLAG_POS)

/*
 * Bit definition for SPIFC_CMD[13] - Only the first transaction has command
 */
#define SPIFC_CMD_1ST_TRANS_WITHCMD_EN_POS       (13)
#define SPIFC_CMD_1ST_TRANS_WITHCMD_EN_MSK       (0x1UL << SPIFC_CMD_1ST_TRANS_WITHCMD_EN_POS)
#define SPIFC_CMD_1ST_TRANS_WITHCMD_ENABLE       (0x1UL << SPIFC_CMD_1ST_TRANS_WITHCMD_EN_POS)
#define SPIFC_CMD_1ST_TRANS_WITHCMD_DISABLE      (0x0UL << SPIFC_CMD_1ST_TRANS_WITHCMD_EN_POS)

/*
 * Bit definition for SPIFC_CMD[9] - A Transaction contains only command information
 */
#define SPIFC_CMD_ONLYCMD_EN_POS                 (9)
#define SPIFC_CMD_ONLYCMD_EN_MSK                 (0x1UL << SPIFC_CMD_ONLYCMD_EN_POS)
#define SPIFC_CMD_ONLYCMD_ENABLE                 (0x1UL << SPIFC_CMD_ONLYCMD_EN_POS)
#define SPIFC_CMD_ONLYCMD_DISABLE                (0x0UL << SPIFC_CMD_ONLYCMD_EN_POS)

/*
 * Bit definition for SPIFC_CMD[8] - A Transaction without command
 */
#define SPIFC_CMD_WITHOUTCMD_EN_POS              (8)
#define SPIFC_CMD_WITHOUTCMD_EN_MSK              (0x1UL << SPIFC_CMD_WITHOUTCMD_EN_POS)
#define SPIFC_CMD_WITHOUTCMD_ENABLE              (0x1UL << SPIFC_CMD_WITHOUTCMD_EN_POS)
#define SPIFC_CMD_WITHOUTCMD_DISABLE             (0x0UL << SPIFC_CMD_WITHOUTCMD_EN_POS)

/*
 * Bit definition for SPIFC_CMD[7:0] - SPI Command
 */
#define SPIFC_CMD_SPICMD_POS                     (0)
#define SPIFC_CMD_SPICMD_MSK                     (0xFFUL << SPIFC_CMD_SPICMD_POS)

/*
 * Bit definition for SPIFC_PARA[14] - A Transaction does not contain enhancement bits
 */
#define SPIFC_PARA_ENHAN_EN_POS                  (14)
#define SPIFC_PARA_ENHAN_EN_MSK                  (0x1UL << SPIFC_PARA_ENHAN_EN_POS)
#define SPIFC_PARA_ENHAN_DISABLE                 (0x1UL << SPIFC_PARA_ENHAN_EN_POS)
#define SPIFC_PARA_ENHAN_ENABLE                  (0x0UL << SPIFC_PARA_ENHAN_EN_POS)

/*
 * Bit definition for SPIFC_PARA[13] - A Transaction only contain address bits
 */
#define SPIFC_PARA_ONLY_ADDR_EN_POS              (13)
#define SPIFC_PARA_ONLY_ADDR_EN_MSK              (0x1UL << SPIFC_PARA_ONLY_ADDR_EN_POS)
#define SPIFC_PARA_ONLY_ADDR_ENABLE              (0x1UL << SPIFC_PARA_ONLY_ADDR_EN_POS)
#define SPIFC_PARA_ONLY_ADDR_DISABLE             (0x0UL << SPIFC_PARA_ONLY_ADDR_EN_POS)

/*
 * Bit definition for SPIFC_PARA[12] - A Transaction does not contain address bits
 */
#define SPIFC_PARA_WITHOUT_ADDR_EN_POS           (12)
#define SPIFC_PARA_WITHOUT_ADDR_EN_MSK           (0x1UL << SPIFC_PARA_WITHOUT_ADDR_EN_POS)
#define SPIFC_PARA_WITHOUT_ADDR_ENABLE           (0x1UL << SPIFC_PARA_WITHOUT_ADDR_EN_POS)
#define SPIFC_PARA_WITHOUT_ADDR_DISABLE          (0x0UL << SPIFC_PARA_WITHOUT_ADDR_EN_POS)

/*
 * Bit definition for SPIFC_PARA[11:8] - Enhancement bits in a transaction
 */
#define SPIFC_PARA_DUMMY_CLK_POS                 (8)
#define SPIFC_PARA_DUMMY_CLK_MSK                 (0xFUL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_9_CYCLE             (0x9UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_8_CYCLE             (0x8UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_7_CYCLE             (0x7UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_6_CYCLE             (0x6UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_5_CYCLE             (0x5UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_4_CYCLE             (0x4UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_3_CYCLE             (0x3UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_2_CYCLE             (0x2UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_1_CYCLE             (0x1UL << SPIFC_PARA_DUMMY_CLK_POS)
#define SPIFC_PARA_DUMMY_CLK_0_CYCLE             (0x0UL << SPIFC_PARA_DUMMY_CLK_POS)

/*
 * Bit definition for SPIFC_PARA[7:0] - Enhancement bits in a transaction
 */
#define SPIFC_PARA_ENHANCE_BIT_POS               (0)
#define SPIFC_PARA_ENHANCE_BIT_MSK               (0xFFUL << SPIFC_PARA_ENHANCE_BIT_POS)
#define SPIFC_PARA_ENHANCE_BIT7                  (0x80UL << SPIFC_PARA_ENHANCE_BIT_POS)
#define SPIFC_PARA_ENHANCE_BIT6                  (0x40UL << SPIFC_PARA_ENHANCE_BIT_POS)
#define SPIFC_PARA_ENHANCE_BIT5                  (0x20UL << SPIFC_PARA_ENHANCE_BIT_POS)
#define SPIFC_PARA_ENHANCE_BIT4                  (0x10UL << SPIFC_PARA_ENHANCE_BIT_POS)
#define SPIFC_PARA_ENHANCE_BIT3                  (0x08UL << SPIFC_PARA_ENHANCE_BIT_POS)
#define SPIFC_PARA_ENHANCE_BIT2                  (0x04UL << SPIFC_PARA_ENHANCE_BIT_POS)
#define SPIFC_PARA_ENHANCE_BIT1                  (0x02UL << SPIFC_PARA_ENHANCE_BIT_POS)
#define SPIFC_PARA_ENHANCE_BIT0                  (0x01UL << SPIFC_PARA_ENHANCE_BIT_POS)

/*
 * Bit definition for SPIFC_TIMING[12] - tSLCH time setting (Timing setting from CS pulling Lo to the first CLK signal)
 */
#define SPIFC_TIMING_SLCH_SEL_POS                (12)
#define SPIFC_TIMING_SLCH_SEL_MSK                (0x1UL << SPIFC_TIMING_SLCH_SEL_POS)
#define SPIFC_TIMING_SLCH_SEL_1D5CLK             (0x1UL << SPIFC_TIMING_SLCH_SEL_POS)
#define SPIFC_TIMING_SLCH_SEL_0D5CLK             (0x0UL << SPIFC_TIMING_SLCH_SEL_POS)

/*
 * Bit definition for SPIFC_TIMING[7:4] - Sample delay for received data
 */
#define SPIFC_TIMING_SMP_DELAY_POS               (4)
#define SPIFC_TIMING_SMP_DELAY_MSK               (0xFUL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_15                (0xFUL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_14                (0xEUL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_13                (0xDUL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_12                (0xCUL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_11                (0xBUL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_10                (0xAUL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_9                 (0x9UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_8                 (0x8UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_7                 (0x7UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_6                 (0x6UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_5                 (0x5UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_4                 (0x4UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_3                 (0x3UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_2                 (0x2UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_1                 (0x1UL << SPIFC_TIMING_SMP_DELAY_POS)
#define SPIFC_TIMING_SMP_DELAY_0                 (0x0UL << SPIFC_TIMING_SMP_DELAY_POS)

/*
 * Bit definition for SPIFC_TIMING[2:0] - Sample timing for received data
 */
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS        (0)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_MSK        (0x7UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_7          (0x7UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_6          (0x6UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_5          (0x5UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_4          (0x4UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_3          (0x3UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_2          (0x2UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_1          (0x1UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)
#define SPIFC_TIMING_SMP_CLK_EDGE_SEL_0          (0x0UL << SPIFC_TIMING_SMP_CLK_EDGE_SEL_POS)

/*
 * Bit definition for SPIFC_CTRL1[15:8] - 4 bytes address enable
 */
#define SPIFC_CTRL1_4BYTES_ADDR_EN_POS           (8)
#define SPIFC_CTRL1_4BYTES_ADDR_EN_MSK           (0xFFUL << SPIFC_CTRL1_4BYTES_ADDR_EN_POS)
#define SPIFC_CTRL1_4BYTES_ADDR_ENABLE           (0xFFUL << SPIFC_CTRL1_4BYTES_ADDR_EN_POS)
#define SPIFC_CTRL1_4BYTES_ADDR_DISABLE          (0x00UL << SPIFC_CTRL1_4BYTES_ADDR_EN_POS)

/*
 * Bit definition for SPIFC_CTRL1[0] - SPI flash controller enable
 */
#define SPIFC_CTRL1_SPIFC_EN_POS                 (0)
#define SPIFC_CTRL1_SPIFC_EN_MSK                 (0x1UL << SPIFC_CTRL1_SPIFC_EN_POS)
#define SPIFC_CTRL1_SPIFC_ENABLE                 (0x1UL << SPIFC_CTRL1_SPIFC_EN_POS)
#define SPIFC_CTRL1_SPIFC_DISABLE                (0x0UL << SPIFC_CTRL1_SPIFC_EN_POS)

/*
 * Bit definition for SPIFC_CTRL2[12] - Asynchronous clock setting
 */
#define SPIFC_CTRL2_CLK_ASYN_POS                 (12)
#define SPIFC_CTRL2_CLK_ASYN_MSK                 (0x1UL << SPIFC_CTRL2_CLK_ASYN_POS)
#define SPIFC_CTRL2_CLK_ASYN_ENABLE              (0x1UL << SPIFC_CTRL2_CLK_ASYN_POS)
#define SPIFC_CTRL2_CLK_ASYN_DISABLE             (0x0UL << SPIFC_CTRL2_CLK_ASYN_POS)

/*
 * Bit definition for SPIFC_CTRL2[9:8] - SPIFC clock divide
 */
#define SPIFC_CTRL2_CLK_SEL_POS                  (8)
#define SPIFC_CTRL2_CLK_SEL_MSK                  (0x3UL << SPIFC_CTRL2_CLK_SEL_POS)
#define SPIFC_CTRL2_CLK_SEL_HCLK_DIV_4           (0x3UL << SPIFC_CTRL2_CLK_SEL_POS)
#define SPIFC_CTRL2_CLK_SEL_HCLK_DIV_3           (0x2UL << SPIFC_CTRL2_CLK_SEL_POS)
#define SPIFC_CTRL2_CLK_SEL_HCLK_DIV_2           (0x1UL << SPIFC_CTRL2_CLK_SEL_POS)
#define SPIFC_CTRL2_CLK_SEL_HCLK_DIV_1           (0x0UL << SPIFC_CTRL2_CLK_SEL_POS)

/*
 * Bit definition for SPIFC_CTRL2[7:0] - SPIFC CS hold count
 */
#define SPIFC_CTRL2_CS_HOLD_CNT_POS              (0)
#define SPIFC_CTRL2_CS_HOLD_CNT_MSK              (0xFFUL << SPIFC_CTRL2_CS_HOLD_CNT_POS)

/*
 * Bit definition for SPIFC_SCRAMBLE_CTRL[1] - SPIFC encrypt control in manual mode
 */
#define SPIFC_SCRAMBLE_CTRL_ENC_EN_POS           (1)
#define SPIFC_SCRAMBLE_CTRL_ENC_EN_MSK           (0x1UL << SPIFC_SCRAMBLE_CTRL_ENC_EN_POS)
#define SPIFC_SCRAMBLE_CTRL_ENC_ENABLE           (0x1UL << SPIFC_SCRAMBLE_CTRL_ENC_EN_POS)
#define SPIFC_SCRAMBLE_CTRL_ENC_DISABLE          (0x0UL << SPIFC_SCRAMBLE_CTRL_ENC_EN_POS)

/*
 * Bit definition for SPIFC_SCRAMBLE_CTRL[0] - SPIFC decrypt control in manual mode
 */
#define SPIFC_SCRAMBLE_CTRL_DEC_EN_POS           (0)
#define SPIFC_SCRAMBLE_CTRL_DEC_EN_MSK           (0x1UL << SPIFC_SCRAMBLE_CTRL_DEC_EN_POS)
#define SPIFC_SCRAMBLE_CTRL_DEC_ENABLE           (0x1UL << SPIFC_SCRAMBLE_CTRL_DEC_EN_POS)
#define SPIFC_SCRAMBLE_CTRL_DEC_DISABLE          (0x0UL << SPIFC_SCRAMBLE_CTRL_DEC_EN_POS)


/*---------------------------------------------------------------------------------------
 * General-purpose I/O Control Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CFG0;                          //GPIOx Mode Control Register 0
    __IO uint32_t CFG1;                          //GPIOx Mode Control Register 1
    __IO uint32_t DRV;                           //GPIOx Driving Control Register
    __IO uint32_t IE;                            //GPIOx Input Enable Control Register
    __IO uint32_t SMT;                           //GPIOx Schmitt Trigger Control Register
    __IO uint32_t OBUF;                          //GPIOx Output Data Buffer Register
    __IO uint32_t IDATA;                         //GPIOx Input Data Status Register
    __IO uint32_t FST;                           //GPIOx IO Function First Register
    __IO uint32_t OBIT[32];                      //GPIOx IOx Bit Operation
} GPIO_TYPE_DEF;

/*
 * Bit definition for GPIO_CFG0[31:30] - GPIOA_15 Mode control
 */
#define GPIO_CFG0_IO15_MODE_POS                  (30)
#define GPIO_CFG0_IO15_MODE_MSK                  (0x3UL << GPIO_CFG0_IO15_MODE_POS)
#define GPIO_CFG0_IO15_MODE_OUTPUT               (0x3UL << GPIO_CFG0_IO15_MODE_POS)
#define GPIO_CFG0_IO15_MODE_OPENDRAIN            (0x2UL << GPIO_CFG0_IO15_MODE_POS)
#define GPIO_CFG0_IO15_MODE_FLOATING             (0x1UL << GPIO_CFG0_IO15_MODE_POS)
#define GPIO_CFG0_IO15_MODE_INPUT                (0x0UL << GPIO_CFG0_IO15_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[29:28] - GPIOA_14 Mode control
 */
#define GPIO_CFG0_IO14_MODE_POS                  (28)
#define GPIO_CFG0_IO14_MODE_MSK                  (0x3UL << GPIO_CFG0_IO14_MODE_POS)
#define GPIO_CFG0_IO14_MODE_OUTPUT               (0x3UL << GPIO_CFG0_IO14_MODE_POS)
#define GPIO_CFG0_IO14_MODE_OPENDRAIN            (0x2UL << GPIO_CFG0_IO14_MODE_POS)
#define GPIO_CFG0_IO14_MODE_FLOATING             (0x1UL << GPIO_CFG0_IO14_MODE_POS)
#define GPIO_CFG0_IO14_MODE_INPUT                (0x0UL << GPIO_CFG0_IO14_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[27:26] - GPIOA_13 Mode control
 */
#define GPIO_CFG0_IO13_MODE_POS                  (26)
#define GPIO_CFG0_IO13_MODE_MSK                  (0x3UL << GPIO_CFG0_IO13_MODE_POS)
#define GPIO_CFG0_IO13_MODE_OUTPUT               (0x3UL << GPIO_CFG0_IO13_MODE_POS)
#define GPIO_CFG0_IO13_MODE_OPENDRAIN            (0x2UL << GPIO_CFG0_IO13_MODE_POS)
#define GPIO_CFG0_IO13_MODE_FLOATING             (0x1UL << GPIO_CFG0_IO13_MODE_POS)
#define GPIO_CFG0_IO13_MODE_INPUT                (0x0UL << GPIO_CFG0_IO13_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[25:24] - GPIOA_12 Mode control
 */
#define GPIO_CFG0_IO12_MODE_POS                  (24)
#define GPIO_CFG0_IO12_MODE_MSK                  (0x3UL << GPIO_CFG0_IO12_MODE_POS)
#define GPIO_CFG0_IO12_MODE_OUTPUT               (0x3UL << GPIO_CFG0_IO12_MODE_POS)
#define GPIO_CFG0_IO12_MODE_OPENDRAIN            (0x2UL << GPIO_CFG0_IO12_MODE_POS)
#define GPIO_CFG0_IO12_MODE_FLOATING             (0x1UL << GPIO_CFG0_IO12_MODE_POS)
#define GPIO_CFG0_IO12_MODE_INPUT                (0x0UL << GPIO_CFG0_IO12_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[23:22] - GPIOA_11 Mode control
 */
#define GPIO_CFG0_IO11_MODE_POS                  (22)
#define GPIO_CFG0_IO11_MODE_MSK                  (0x3UL << GPIO_CFG0_IO11_MODE_POS)
#define GPIO_CFG0_IO11_MODE_OUTPUT               (0x3UL << GPIO_CFG0_IO11_MODE_POS)
#define GPIO_CFG0_IO11_MODE_OPENDRAIN            (0x2UL << GPIO_CFG0_IO11_MODE_POS)
#define GPIO_CFG0_IO11_MODE_FLOATING             (0x1UL << GPIO_CFG0_IO11_MODE_POS)
#define GPIO_CFG0_IO11_MODE_INPUT                (0x0UL << GPIO_CFG0_IO11_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[21:20] - GPIOA_10 Mode control
 */
#define GPIO_CFG0_IO10_MODE_POS                  (20)
#define GPIO_CFG0_IO10_MODE_MSK                  (0x3UL << GPIO_CFG0_IO10_MODE_POS)
#define GPIO_CFG0_IO10_MODE_OUTPUT               (0x3UL << GPIO_CFG0_IO10_MODE_POS)
#define GPIO_CFG0_IO10_MODE_OPENDRAIN            (0x2UL << GPIO_CFG0_IO10_MODE_POS)
#define GPIO_CFG0_IO10_MODE_FLOATING             (0x1UL << GPIO_CFG0_IO10_MODE_POS)
#define GPIO_CFG0_IO10_MODE_INPUT                (0x0UL << GPIO_CFG0_IO10_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[19:18] - GPIOA_9 Mode control
 */
#define GPIO_CFG0_IO9_MODE_POS                   (18)
#define GPIO_CFG0_IO9_MODE_MSK                   (0x3UL << GPIO_CFG0_IO9_MODE_POS)
#define GPIO_CFG0_IO9_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO9_MODE_POS)
#define GPIO_CFG0_IO9_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO9_MODE_POS)
#define GPIO_CFG0_IO9_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO9_MODE_POS)
#define GPIO_CFG0_IO9_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO9_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[17:16] - GPIOA_8 Mode control
 */
#define GPIO_CFG0_IO8_MODE_POS                   (16)
#define GPIO_CFG0_IO8_MODE_MSK                   (0x3UL << GPIO_CFG0_IO8_MODE_POS)
#define GPIO_CFG0_IO8_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO8_MODE_POS)
#define GPIO_CFG0_IO8_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO8_MODE_POS)
#define GPIO_CFG0_IO8_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO8_MODE_POS)
#define GPIO_CFG0_IO8_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO8_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[15:14] - GPIOA_7 Mode control
 */
#define GPIO_CFG0_IO7_MODE_POS                   (14)
#define GPIO_CFG0_IO7_MODE_MSK                   (0x3UL << GPIO_CFG0_IO7_MODE_POS)
#define GPIO_CFG0_IO7_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO7_MODE_POS)
#define GPIO_CFG0_IO7_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO7_MODE_POS)
#define GPIO_CFG0_IO7_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO7_MODE_POS)
#define GPIO_CFG0_IO7_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO7_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[13:12] - GPIOA_6 Mode control
 */
#define GPIO_CFG0_IO6_MODE_POS                   (12)
#define GPIO_CFG0_IO6_MODE_MSK                   (0x3UL << GPIO_CFG0_IO6_MODE_POS)
#define GPIO_CFG0_IO6_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO6_MODE_POS)
#define GPIO_CFG0_IO6_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO6_MODE_POS)
#define GPIO_CFG0_IO6_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO6_MODE_POS)
#define GPIO_CFG0_IO6_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO6_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[11:10] - GPIOA/B_5 Mode control
 */
#define GPIO_CFG0_IO5_MODE_POS                   (10)
#define GPIO_CFG0_IO5_MODE_MSK                   (0x3UL << GPIO_CFG0_IO5_MODE_POS)
#define GPIO_CFG0_IO5_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO5_MODE_POS)
#define GPIO_CFG0_IO5_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO5_MODE_POS)
#define GPIO_CFG0_IO5_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO5_MODE_POS)
#define GPIO_CFG0_IO5_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO5_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[9:8] - GPIOA/B_4 Mode control
 */
#define GPIO_CFG0_IO4_MODE_POS                   (8)
#define GPIO_CFG0_IO4_MODE_MSK                   (0x3UL << GPIO_CFG0_IO4_MODE_POS)
#define GPIO_CFG0_IO4_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO4_MODE_POS)
#define GPIO_CFG0_IO4_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO4_MODE_POS)
#define GPIO_CFG0_IO4_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO4_MODE_POS)
#define GPIO_CFG0_IO4_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO4_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[7:6] - GPIOA/B_3 Mode control
 */
#define GPIO_CFG0_IO3_MODE_POS                   (6)
#define GPIO_CFG0_IO3_MODE_MSK                   (0x3UL << GPIO_CFG0_IO3_MODE_POS)
#define GPIO_CFG0_IO3_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO3_MODE_POS)
#define GPIO_CFG0_IO3_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO3_MODE_POS)
#define GPIO_CFG0_IO3_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO3_MODE_POS)
#define GPIO_CFG0_IO3_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO3_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[5:4] - GPIOA/B/C_2 Mode control
 */
#define GPIO_CFG0_IO2_MODE_POS                   (4)
#define GPIO_CFG0_IO2_MODE_MSK                   (0x3UL << GPIO_CFG0_IO2_MODE_POS)
#define GPIO_CFG0_IO2_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO2_MODE_POS)
#define GPIO_CFG0_IO2_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO2_MODE_POS)
#define GPIO_CFG0_IO2_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO2_MODE_POS)
#define GPIO_CFG0_IO2_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO2_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[3:2] - GPIOA/B/C/D_1 Mode control
 */
#define GPIO_CFG0_IO1_MODE_POS                   (2)
#define GPIO_CFG0_IO1_MODE_MSK                   (0x3UL << GPIO_CFG0_IO1_MODE_POS)
#define GPIO_CFG0_IO1_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO1_MODE_POS)
#define GPIO_CFG0_IO1_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO1_MODE_POS)
#define GPIO_CFG0_IO1_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO1_MODE_POS)
#define GPIO_CFG0_IO1_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO1_MODE_POS)

/*
 * Bit definition for GPIO_CFG0[1:0] - GPIOA/B/C/D_0 Mode control
 */
#define GPIO_CFG0_IO0_MODE_POS                   (0)
#define GPIO_CFG0_IO0_MODE_MSK                   (0x3UL << GPIO_CFG0_IO0_MODE_POS)
#define GPIO_CFG0_IO0_MODE_OUTPUT                (0x3UL << GPIO_CFG0_IO0_MODE_POS)
#define GPIO_CFG0_IO0_MODE_OPENDRAIN             (0x2UL << GPIO_CFG0_IO0_MODE_POS)
#define GPIO_CFG0_IO0_MODE_FLOATING              (0x1UL << GPIO_CFG0_IO0_MODE_POS)
#define GPIO_CFG0_IO0_MODE_INPUT                 (0x0UL << GPIO_CFG0_IO0_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[31:30] - GPIOA_31 Mode control
 */
#define GPIO_CFG1_IO31_MODE_POS                  (30)
#define GPIO_CFG1_IO31_MODE_MSK                  (0x3UL << GPIO_CFG1_IO31_MODE_POS)
#define GPIO_CFG1_IO31_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO31_MODE_POS)
#define GPIO_CFG1_IO31_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO31_MODE_POS)
#define GPIO_CFG1_IO31_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO31_MODE_POS)
#define GPIO_CFG1_IO31_MODE_INPUT                (0x0UL << GPIO_CFG1_IO31_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[29:28] - GPIOA_30 Mode control
 */
#define GPIO_CFG1_IO30_MODE_POS                  (28)
#define GPIO_CFG1_IO30_MODE_MSK                  (0x3UL << GPIO_CFG1_IO30_MODE_POS)
#define GPIO_CFG1_IO30_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO30_MODE_POS)
#define GPIO_CFG1_IO30_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO30_MODE_POS)
#define GPIO_CFG1_IO30_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO30_MODE_POS)
#define GPIO_CFG1_IO30_MODE_INPUT                (0x0UL << GPIO_CFG1_IO30_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[27:26] - GPIOA_29 Mode control
 */
#define GPIO_CFG1_IO29_MODE_POS                  (26)
#define GPIO_CFG1_IO29_MODE_MSK                  (0x3UL << GPIO_CFG1_IO29_MODE_POS)
#define GPIO_CFG1_IO29_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO29_MODE_POS)
#define GPIO_CFG1_IO29_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO29_MODE_POS)
#define GPIO_CFG1_IO29_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO29_MODE_POS)
#define GPIO_CFG1_IO29_MODE_INPUT                (0x0UL << GPIO_CFG1_IO29_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[25:24] - GPIOA_28 Mode control
 */
#define GPIO_CFG1_IO28_MODE_POS                  (24)
#define GPIO_CFG1_IO28_MODE_MSK                  (0x3UL << GPIO_CFG1_IO28_MODE_POS)
#define GPIO_CFG1_IO28_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO28_MODE_POS)
#define GPIO_CFG1_IO28_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO28_MODE_POS)
#define GPIO_CFG1_IO28_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO28_MODE_POS)
#define GPIO_CFG1_IO28_MODE_INPUT                (0x0UL << GPIO_CFG1_IO28_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[23:22] - GPIOA_27 Mode control
 */
#define GPIO_CFG1_IO27_MODE_POS                  (22)
#define GPIO_CFG1_IO27_MODE_MSK                  (0x3UL << GPIO_CFG1_IO27_MODE_POS)
#define GPIO_CFG1_IO27_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO27_MODE_POS)
#define GPIO_CFG1_IO27_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO27_MODE_POS)
#define GPIO_CFG1_IO27_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO27_MODE_POS)
#define GPIO_CFG1_IO27_MODE_INPUT                (0x0UL << GPIO_CFG1_IO27_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[21:20] - GPIOA_26 Mode control
 */
#define GPIO_CFG1_IO26_MODE_POS                  (20)
#define GPIO_CFG1_IO26_MODE_MSK                  (0x3UL << GPIO_CFG1_IO26_MODE_POS)
#define GPIO_CFG1_IO26_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO26_MODE_POS)
#define GPIO_CFG1_IO26_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO26_MODE_POS)
#define GPIO_CFG1_IO26_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO26_MODE_POS)
#define GPIO_CFG1_IO26_MODE_INPUT                (0x0UL << GPIO_CFG1_IO26_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[19:18] - GPIOA_25 Mode control
 */
#define GPIO_CFG1_IO25_MODE_POS                  (18)
#define GPIO_CFG1_IO25_MODE_MSK                  (0x3UL << GPIO_CFG1_IO25_MODE_POS)
#define GPIO_CFG1_IO25_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO25_MODE_POS)
#define GPIO_CFG1_IO25_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO25_MODE_POS)
#define GPIO_CFG1_IO25_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO25_MODE_POS)
#define GPIO_CFG1_IO25_MODE_INPUT                (0x0UL << GPIO_CFG1_IO25_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[17:16] - GPIOA_24 Mode control
 */
#define GPIO_CFG1_IO24_MODE_POS                  (16)
#define GPIO_CFG1_IO24_MODE_MSK                  (0x3UL << GPIO_CFG1_IO24_MODE_POS)
#define GPIO_CFG1_IO24_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO24_MODE_POS)
#define GPIO_CFG1_IO24_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO24_MODE_POS)
#define GPIO_CFG1_IO24_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO24_MODE_POS)
#define GPIO_CFG1_IO24_MODE_INPUT                (0x0UL << GPIO_CFG1_IO24_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[15:14] - GPIOA_23 Mode control
 */
#define GPIO_CFG1_IO23_MODE_POS                  (14)
#define GPIO_CFG1_IO23_MODE_MSK                  (0x3UL << GPIO_CFG1_IO23_MODE_POS)
#define GPIO_CFG1_IO23_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO23_MODE_POS)
#define GPIO_CFG1_IO23_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO23_MODE_POS)
#define GPIO_CFG1_IO23_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO23_MODE_POS)
#define GPIO_CFG1_IO23_MODE_INPUT                (0x0UL << GPIO_CFG1_IO23_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[13:12] - GPIOA_22 Mode control
 */
#define GPIO_CFG1_IO22_MODE_POS                  (12)
#define GPIO_CFG1_IO22_MODE_MSK                  (0x3UL << GPIO_CFG1_IO22_MODE_POS)
#define GPIO_CFG1_IO22_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO22_MODE_POS)
#define GPIO_CFG1_IO22_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO22_MODE_POS)
#define GPIO_CFG1_IO22_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO22_MODE_POS)
#define GPIO_CFG1_IO22_MODE_INPUT                (0x0UL << GPIO_CFG1_IO22_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[11:10] - GPIOA_21 Mode control
 */
#define GPIO_CFG1_IO21_MODE_POS                  (10)
#define GPIO_CFG1_IO21_MODE_MSK                  (0x3UL << GPIO_CFG1_IO21_MODE_POS)
#define GPIO_CFG1_IO21_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO21_MODE_POS)
#define GPIO_CFG1_IO21_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO21_MODE_POS)
#define GPIO_CFG1_IO21_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO21_MODE_POS)
#define GPIO_CFG1_IO21_MODE_INPUT                (0x0UL << GPIO_CFG1_IO21_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[9:8] - GPIOA_20 Mode control
 */
#define GPIO_CFG1_IO20_MODE_POS                  (8)
#define GPIO_CFG1_IO20_MODE_MSK                  (0x3UL << GPIO_CFG1_IO20_MODE_POS)
#define GPIO_CFG1_IO20_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO20_MODE_POS)
#define GPIO_CFG1_IO20_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO20_MODE_POS)
#define GPIO_CFG1_IO20_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO20_MODE_POS)
#define GPIO_CFG1_IO20_MODE_INPUT                (0x0UL << GPIO_CFG1_IO20_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[7:6] - GPIOA_19 Mode control
 */
#define GPIO_CFG1_IO19_MODE_POS                  (6)
#define GPIO_CFG1_IO19_MODE_MSK                  (0x3UL << GPIO_CFG1_IO19_MODE_POS)
#define GPIO_CFG1_IO19_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO19_MODE_POS)
#define GPIO_CFG1_IO19_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO19_MODE_POS)
#define GPIO_CFG1_IO19_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO19_MODE_POS)
#define GPIO_CFG1_IO19_MODE_INPUT                (0x0UL << GPIO_CFG1_IO19_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[5:4] - GPIOA_18 Mode control
 */
#define GPIO_CFG1_IO18_MODE_POS                  (4)
#define GPIO_CFG1_IO18_MODE_MSK                  (0x3UL << GPIO_CFG1_IO18_MODE_POS)
#define GPIO_CFG1_IO18_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO18_MODE_POS)
#define GPIO_CFG1_IO18_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO18_MODE_POS)
#define GPIO_CFG1_IO18_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO18_MODE_POS)
#define GPIO_CFG1_IO18_MODE_INPUT                (0x0UL << GPIO_CFG1_IO18_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[3:2] - GPIOA_17 Mode control
 */
#define GPIO_CFG1_IO17_MODE_POS                  (2)
#define GPIO_CFG1_IO17_MODE_MSK                  (0x3UL << GPIO_CFG1_IO17_MODE_POS)
#define GPIO_CFG1_IO17_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO17_MODE_POS)
#define GPIO_CFG1_IO17_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO17_MODE_POS)
#define GPIO_CFG1_IO17_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO17_MODE_POS)
#define GPIO_CFG1_IO17_MODE_INPUT                (0x0UL << GPIO_CFG1_IO17_MODE_POS)

/*
 * Bit definition for GPIO_CFG1[1:0] - GPIOA_16 Mode control
 */
#define GPIO_CFG1_IO16_MODE_POS                  (0)
#define GPIO_CFG1_IO16_MODE_MSK                  (0x3UL << GPIO_CFG1_IO16_MODE_POS)
#define GPIO_CFG1_IO16_MODE_OUTPUT               (0x3UL << GPIO_CFG1_IO16_MODE_POS)
#define GPIO_CFG1_IO16_MODE_OPENDRAIN            (0x2UL << GPIO_CFG1_IO16_MODE_POS)
#define GPIO_CFG1_IO16_MODE_FLOATING             (0x1UL << GPIO_CFG1_IO16_MODE_POS)
#define GPIO_CFG1_IO16_MODE_INPUT                (0x0UL << GPIO_CFG1_IO16_MODE_POS)

/*
 * Bit definition for GPIOA_DRV[31] - GPIOA_31 Drive Current Select
 */
#define GPIOA_DRV_IO31_SEL_POS                    (31)
#define GPIOA_DRV_IO31_SEL_MSK                    (0x1UL << GPIOA_DRV_IO31_SEL_POS)
#define GPIOA_DRV_IO31_16mA                       (0x1UL << GPIOA_DRV_IO31_SEL_POS)
#define GPIOA_DRV_IO31_8mA                        (0x0UL << GPIOA_DRV_IO31_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[30] - GPIOA_30 Drive Current Select
 */
#define GPIOA_DRV_IO30_SEL_POS                    (30)
#define GPIOA_DRV_IO30_SEL_MSK                    (0x1UL << GPIOA_DRV_IO30_SEL_POS)
#define GPIOA_DRV_IO30_16mA                       (0x1UL << GPIOA_DRV_IO30_SEL_POS)
#define GPIOA_DRV_IO30_8mA                        (0x0UL << GPIOA_DRV_IO30_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[29] - GPIOA_29 Drive Current Select
 */
#define GPIOA_DRV_IO29_SEL_POS                    (29)
#define GPIOA_DRV_IO29_SEL_MSK                    (0x1UL << GPIOA_DRV_IO29_SEL_POS)
#define GPIOA_DRV_IO29_16mA                       (0x1UL << GPIOA_DRV_IO29_SEL_POS)
#define GPIOA_DRV_IO29_8mA                        (0x0UL << GPIOA_DRV_IO29_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[28] - GPIOA_28 Drive Current Select
 */
#define GPIOA_DRV_IO28_SEL_POS                    (28)
#define GPIOA_DRV_IO28_SEL_MSK                    (0x1UL << GPIOA_DRV_IO28_SEL_POS)
#define GPIOA_DRV_IO28_16mA                       (0x1UL << GPIOA_DRV_IO28_SEL_POS)
#define GPIOA_DRV_IO28_8mA                        (0x0UL << GPIOA_DRV_IO28_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[27] - GPIOA_27 Drive Current Select
 */
#define GPIOA_DRV_IO27_SEL_POS                    (27)
#define GPIOA_DRV_IO27_SEL_MSK                    (0x1UL << GPIOA_DRV_IO27_SEL_POS)
#define GPIOA_DRV_IO27_16mA                       (0x1UL << GPIOA_DRV_IO27_SEL_POS)
#define GPIOA_DRV_IO27_8mA                        (0x0UL << GPIOA_DRV_IO27_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[26] - GPIOA_26 Drive Current Select
 */
#define GPIOA_DRV_IO26_SEL_POS                    (26)
#define GPIOA_DRV_IO26_SEL_MSK                    (0x1UL << GPIOA_DRV_IO26_SEL_POS)
#define GPIOA_DRV_IO26_16mA                       (0x1UL << GPIOA_DRV_IO26_SEL_POS)
#define GPIOA_DRV_IO26_8mA                        (0x0UL << GPIOA_DRV_IO26_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[25] - GPIOA_25 Drive Current Select
 */
#define GPIOA_DRV_IO25_SEL_POS                    (25)
#define GPIOA_DRV_IO25_SEL_MSK                    (0x1UL << GPIOA_DRV_IO25_SEL_POS)
#define GPIOA_DRV_IO25_16mA                       (0x1UL << GPIOA_DRV_IO25_SEL_POS)
#define GPIOA_DRV_IO25_8mA                        (0x0UL << GPIOA_DRV_IO25_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[24] - GPIOA_24 Drive Current Select
 */
#define GPIOA_DRV_IO24_SEL_POS                    (24)
#define GPIOA_DRV_IO24_SEL_MSK                    (0x1UL << GPIOA_DRV_IO24_SEL_POS)
#define GPIOA_DRV_IO24_16mA                       (0x1UL << GPIOA_DRV_IO24_SEL_POS)
#define GPIOA_DRV_IO24_8mA                        (0x0UL << GPIOA_DRV_IO24_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[23] - GPIOA_23 Drive Current Select
 */
#define GPIOA_DRV_IO23_SEL_POS                    (23)
#define GPIOA_DRV_IO23_SEL_MSK                    (0x1UL << GPIOA_DRV_IO23_SEL_POS)
#define GPIOA_DRV_IO23_16mA                       (0x1UL << GPIOA_DRV_IO23_SEL_POS)
#define GPIOA_DRV_IO23_8mA                        (0x0UL << GPIOA_DRV_IO23_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[22] - GPIOA_22 Drive Current Select
 */
#define GPIOA_DRV_IO22_SEL_POS                    (22)
#define GPIOA_DRV_IO22_SEL_MSK                    (0x1UL << GPIOA_DRV_IO22_SEL_POS)
#define GPIOA_DRV_IO22_16mA                       (0x1UL << GPIOA_DRV_IO22_SEL_POS)
#define GPIOA_DRV_IO22_8mA                        (0x0UL << GPIOA_DRV_IO22_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[21] - GPIOA_21 Drive Current Select
 */
#define GPIOA_DRV_IO21_SEL_POS                    (21)
#define GPIOA_DRV_IO21_SEL_MSK                    (0x1UL << GPIOA_DRV_IO21_SEL_POS)
#define GPIOA_DRV_IO21_16mA                       (0x1UL << GPIOA_DRV_IO21_SEL_POS)
#define GPIOA_DRV_IO21_8mA                        (0x0UL << GPIOA_DRV_IO21_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[20] - GPIOA_20 Drive Current Select
 */
#define GPIOA_DRV_IO20_SEL_POS                    (20)
#define GPIOA_DRV_IO20_SEL_MSK                    (0x1UL << GPIOA_DRV_IO20_SEL_POS)
#define GPIOA_DRV_IO20_16mA                       (0x1UL << GPIOA_DRV_IO20_SEL_POS)
#define GPIOA_DRV_IO20_8mA                        (0x0UL << GPIOA_DRV_IO20_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[19] - GPIOA_19 Drive Current Select
 */
#define GPIOA_DRV_IO19_SEL_POS                    (19)
#define GPIOA_DRV_IO19_SEL_MSK                    (0x1UL << GPIOA_DRV_IO19_SEL_POS)
#define GPIOA_DRV_IO19_16mA                       (0x1UL << GPIOA_DRV_IO19_SEL_POS)
#define GPIOA_DRV_IO19_8mA                        (0x0UL << GPIOA_DRV_IO19_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[18] - GPIOA_18 Drive Current Select
 */
#define GPIOA_DRV_IO18_SEL_POS                    (18)
#define GPIOA_DRV_IO18_SEL_MSK                    (0x1UL << GPIOA_DRV_IO18_SEL_POS)
#define GPIOA_DRV_IO18_16mA                       (0x1UL << GPIOA_DRV_IO18_SEL_POS)
#define GPIOA_DRV_IO18_8mA                        (0x0UL << GPIOA_DRV_IO18_SEL_POS)
/*
 * Bit definition for GPIOA_DRV[17] - GPIOA_17 Drive Current Select
 */
#define GPIOA_DRV_IO17_SEL_POS                    (17)
#define GPIOA_DRV_IO17_SEL_MSK                    (0x1UL << GPIOA_DRV_IO17_SEL_POS)
#define GPIOA_DRV_IO17_16mA                       (0x1UL << GPIOA_DRV_IO17_SEL_POS)
#define GPIOA_DRV_IO17_8mA                        (0x0UL << GPIOA_DRV_IO17_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[16] - GPIOA_16 Drive Current Select
 */
#define GPIOA_DRV_IO16_SEL_POS                    (16)
#define GPIOA_DRV_IO16_SEL_MSK                    (0x1UL << GPIOA_DRV_IO16_SEL_POS)
#define GPIOA_DRV_IO16_16mA                       (0x1UL << GPIOA_DRV_IO16_SEL_POS)
#define GPIOA_DRV_IO16_8mA                        (0x0UL << GPIOA_DRV_IO16_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[15] - GPIOA_15 Drive Current Select
 */
#define GPIOA_DRV_IO15_SEL_POS                    (15)
#define GPIOA_DRV_IO15_SEL_MSK                    (0x1UL << GPIOA_DRV_IO15_SEL_POS)
#define GPIOA_DRV_IO15_16mA                       (0x1UL << GPIOA_DRV_IO15_SEL_POS)
#define GPIOA_DRV_IO15_8mA                        (0x0UL << GPIOA_DRV_IO15_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[14] - GPIOA_14 Drive Current Select
 */
#define GPIOA_DRV_IO14_SEL_POS                    (14)
#define GPIOA_DRV_IO14_SEL_MSK                    (0x1UL << GPIOA_DRV_IO14_SEL_POS)
#define GPIOA_DRV_IO14_16mA                       (0x1UL << GPIOA_DRV_IO14_SEL_POS)
#define GPIOA_DRV_IO14_8mA                        (0x0UL << GPIOA_DRV_IO14_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[13] - GPIOA_13 Drive Current Select
 */
#define GPIOA_DRV_IO13_SEL_POS                    (13)
#define GPIOA_DRV_IO13_SEL_MSK                    (0x1UL << GPIOA_DRV_IO13_SEL_POS)
#define GPIOA_DRV_IO13_16mA                       (0x1UL << GPIOA_DRV_IO13_SEL_POS)
#define GPIOA_DRV_IO13_8mA                        (0x0UL << GPIOA_DRV_IO13_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[12] - GPIOA_12 Drive Current Select
 */
#define GPIOA_DRV_IO12_SEL_POS                    (12)
#define GPIOA_DRV_IO12_SEL_MSK                    (0x1UL << GPIOA_DRV_IO12_SEL_POS)
#define GPIOA_DRV_IO12_16mA                       (0x1UL << GPIOA_DRV_IO12_SEL_POS)
#define GPIOA_DRV_IO12_8mA                        (0x0UL << GPIOA_DRV_IO12_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[11] - GPIOA_11 Drive Current Select
 */
#define GPIOA_DRV_IO11_SEL_POS                    (11)
#define GPIOA_DRV_IO11_SEL_MSK                    (0x1UL << GPIOA_DRV_IO11_SEL_POS)
#define GPIOA_DRV_IO11_16mA                       (0x1UL << GPIOA_DRV_IO11_SEL_POS)
#define GPIOA_DRV_IO11_8mA                        (0x0UL << GPIOA_DRV_IO11_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[10] - GPIOA_10 Drive Current Select
 */
#define GPIOA_DRV_IO10_SEL_POS                    (10)
#define GPIOA_DRV_IO10_SEL_MSK                    (0x1UL << GPIOA_DRV_IO10_SEL_POS)
#define GPIOA_DRV_IO10_16mA                       (0x1UL << GPIOA_DRV_IO10_SEL_POS)
#define GPIOA_DRV_IO10_8mA                        (0x0UL << GPIOA_DRV_IO10_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[9] - GPIOA_9 Drive Current Select
 */
#define GPIOA_DRV_IO9_SEL_POS                     (9)
#define GPIOA_DRV_IO9_SEL_MSK                     (0x1UL << GPIOA_DRV_IO9_SEL_POS)
#define GPIOA_DRV_IO9_16mA                        (0x1UL << GPIOA_DRV_IO9_SEL_POS)
#define GPIOA_DRV_IO9_8mA                         (0x0UL << GPIOA_DRV_IO9_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[8] - GPIOA_8 Drive Current Select
 */
#define GPIOA_DRV_IO8_SEL_POS                     (8)
#define GPIOA_DRV_IO8_SEL_MSK                     (0x1UL << GPIOA_DRV_IO8_SEL_POS)
#define GPIOA_DRV_IO8_16mA                        (0x1UL << GPIOA_DRV_IO8_SEL_POS)
#define GPIOA_DRV_IO8_8mA                         (0x0UL << GPIOA_DRV_IO8_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[7] - GPIOA_7 Drive Current Select
 */
#define GPIOA_DRV_IO7_SEL_POS                     (7)
#define GPIOA_DRV_IO7_SEL_MSK                     (0x1UL << GPIOA_DRV_IO7_SEL_POS)
#define GPIOA_DRV_IO7_16mA                        (0x1UL << GPIOA_DRV_IO7_SEL_POS)
#define GPIOA_DRV_IO7_8mA                         (0x0UL << GPIOA_DRV_IO7_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[6] - GPIOA_6 Drive Current Select
 */
#define GPIOA_DRV_IO6_SEL_POS                     (6)
#define GPIOA_DRV_IO6_SEL_MSK                     (0x1UL << GPIOA_DRV_IO6_SEL_POS)
#define GPIOA_DRV_IO6_16mA                        (0x1UL << GPIOA_DRV_IO6_SEL_POS)
#define GPIOA_DRV_IO6_8mA                         (0x0UL << GPIOA_DRV_IO6_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[5] - GPIOA_5 Drive Current Select
 */
#define GPIOA_DRV_IO5_SEL_POS                     (5)
#define GPIOA_DRV_IO5_SEL_MSK                     (0x1UL << GPIOA_DRV_IO5_SEL_POS)
#define GPIOA_DRV_IO5_16mA                        (0x1UL << GPIOA_DRV_IO5_SEL_POS)
#define GPIOA_DRV_IO5_8mA                         (0x0UL << GPIOA_DRV_IO5_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[4] - GPIOA_4 Drive Current Select
 */
#define GPIOA_DRV_IO4_SEL_POS                     (4)
#define GPIOA_DRV_IO4_SEL_MSK                     (0x1UL << GPIOA_DRV_IO4_SEL_POS)
#define GPIOA_DRV_IO4_16mA                        (0x1UL << GPIOA_DRV_IO4_SEL_POS)
#define GPIOA_DRV_IO4_8mA                         (0x0UL << GPIOA_DRV_IO4_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[3] - GPIOA_3 Drive Current Select
 */
#define GPIOA_DRV_IO3_SEL_POS                     (3)
#define GPIOA_DRV_IO3_SEL_MSK                     (0x1UL << GPIOA_DRV_IO3_SEL_POS)
#define GPIOA_DRV_IO3_16mA                        (0x1UL << GPIOA_DRV_IO3_SEL_POS)
#define GPIOA_DRV_IO3_8mA                         (0x0UL << GPIOA_DRV_IO3_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[2] - GPIOA_2 Drive Current Select
 */
#define GPIOA_DRV_IO2_SEL_POS                     (2)
#define GPIOA_DRV_IO2_SEL_MSK                     (0x1UL << GPIOA_DRV_IO2_SEL_POS)
#define GPIOA_DRV_IO2_16mA                        (0x1UL << GPIOA_DRV_IO2_SEL_POS)
#define GPIOA_DRV_IO2_8mA                         (0x0UL << GPIOA_DRV_IO2_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[1] - GPIOA_1 Drive Current Select
 */
#define GPIOA_DRV_IO1_SEL_POS                     (1)
#define GPIOA_DRV_IO1_SEL_MSK                     (0x1UL << GPIOA_DRV_IO1_SEL_POS)
#define GPIOA_DRV_IO1_16mA                        (0x1UL << GPIOA_DRV_IO1_SEL_POS)
#define GPIOA_DRV_IO1_8mA                         (0x0UL << GPIOA_DRV_IO1_SEL_POS)

/*
 * Bit definition for GPIOA_DRV[0] - GPIOA_0 Drive Current Select
 */
#define GPIOA_DRV_IO0_SEL_POS                     (0)
#define GPIOA_DRV_IO0_SEL_MSK                     (0x1UL << GPIOA_DRV_IO0_SEL_POS)
#define GPIOA_DRV_IO0_16mA                        (0x1UL << GPIOA_DRV_IO0_SEL_POS)
#define GPIOA_DRV_IO0_8mA                         (0x0UL << GPIOA_DRV_IO0_SEL_POS)

/*
 * Bit definition for GPIOB_DRV[22:20] - GPIOB_5 Drive Current Select
 */
#define GPIOB_DRV_IO5_SEL_POS                    (20)
#define GPIOB_DRV_IO5_SEL_MSK                    (0x7UL << GPIOB_DRV_IO5_SEL_POS)
#define GPIOB_DRV_IO5_16mA                       (0x7UL << GPIOB_DRV_IO5_SEL_POS)
#define GPIOB_DRV_IO5_14mA                       (0x6UL << GPIOB_DRV_IO5_SEL_POS)
#define GPIOB_DRV_IO5_12mA                       (0x5UL << GPIOB_DRV_IO5_SEL_POS)
#define GPIOB_DRV_IO5_10mA                       (0x4UL << GPIOB_DRV_IO5_SEL_POS)
#define GPIOB_DRV_IO5_8mA                        (0x3UL << GPIOB_DRV_IO5_SEL_POS)
#define GPIOB_DRV_IO5_6mA                        (0x2UL << GPIOB_DRV_IO5_SEL_POS)
#define GPIOB_DRV_IO5_4mA                        (0x1UL << GPIOB_DRV_IO5_SEL_POS)
#define GPIOB_DRV_IO5_2mA                        (0x0UL << GPIOB_DRV_IO5_SEL_POS)

/*
 * Bit definition for GPIOB_DRV[18:16] - GPIOB_4 Drive Current Select
 */
#define GPIOB_DRV_IO4_SEL_POS                    (16)
#define GPIOB_DRV_IO4_SEL_MSK                    (0x7UL << GPIOB_DRV_IO4_SEL_POS)
#define GPIOB_DRV_IO4_16mA                       (0x7UL << GPIOB_DRV_IO4_SEL_POS)
#define GPIOB_DRV_IO4_14mA                       (0x6UL << GPIOB_DRV_IO4_SEL_POS)
#define GPIOB_DRV_IO4_12mA                       (0x5UL << GPIOB_DRV_IO4_SEL_POS)
#define GPIOB_DRV_IO4_10mA                       (0x4UL << GPIOB_DRV_IO4_SEL_POS)
#define GPIOB_DRV_IO4_8mA                        (0x3UL << GPIOB_DRV_IO4_SEL_POS)
#define GPIOB_DRV_IO4_6mA                        (0x2UL << GPIOB_DRV_IO4_SEL_POS)
#define GPIOB_DRV_IO4_4mA                        (0x1UL << GPIOB_DRV_IO4_SEL_POS)
#define GPIOB_DRV_IO4_2mA                        (0x0UL << GPIOB_DRV_IO4_SEL_POS)

/*
 * Bit definition for GPIOB_DRV[14:12] - GPIOB_3 Drive Current Select
 */
#define GPIOB_DRV_IO3_SEL_POS                    (12)
#define GPIOB_DRV_IO3_SEL_MSK                    (0x7UL << GPIOB_DRV_IO3_SEL_POS)
#define GPIOB_DRV_IO3_16mA                       (0x7UL << GPIOB_DRV_IO3_SEL_POS)
#define GPIOB_DRV_IO3_14mA                       (0x6UL << GPIOB_DRV_IO3_SEL_POS)
#define GPIOB_DRV_IO3_12mA                       (0x5UL << GPIOB_DRV_IO3_SEL_POS)
#define GPIOB_DRV_IO3_10mA                       (0x4UL << GPIOB_DRV_IO3_SEL_POS)
#define GPIOB_DRV_IO3_8mA                        (0x3UL << GPIOB_DRV_IO3_SEL_POS)
#define GPIOB_DRV_IO3_6mA                        (0x2UL << GPIOB_DRV_IO3_SEL_POS)
#define GPIOB_DRV_IO3_4mA                        (0x1UL << GPIOB_DRV_IO3_SEL_POS)
#define GPIOB_DRV_IO3_2mA                        (0x0UL << GPIOB_DRV_IO3_SEL_POS)

/*
 * Bit definition for GPIOB_DRV[10:8] - GPIOB_2 Drive Current Select
 */
#define GPIOB_DRV_IO2_SEL_POS                    (8)
#define GPIOB_DRV_IO2_SEL_MSK                    (0x7UL << GPIOB_DRV_IO2_SEL_POS)
#define GPIOB_DRV_IO2_16mA                       (0x7UL << GPIOB_DRV_IO2_SEL_POS)
#define GPIOB_DRV_IO2_14mA                       (0x6UL << GPIOB_DRV_IO2_SEL_POS)
#define GPIOB_DRV_IO2_12mA                       (0x5UL << GPIOB_DRV_IO2_SEL_POS)
#define GPIOB_DRV_IO2_10mA                       (0x4UL << GPIOB_DRV_IO2_SEL_POS)
#define GPIOB_DRV_IO2_8mA                        (0x3UL << GPIOB_DRV_IO2_SEL_POS)
#define GPIOB_DRV_IO2_6mA                        (0x2UL << GPIOB_DRV_IO2_SEL_POS)
#define GPIOB_DRV_IO2_4mA                        (0x1UL << GPIOB_DRV_IO2_SEL_POS)
#define GPIOB_DRV_IO2_2mA                        (0x0UL << GPIOB_DRV_IO2_SEL_POS)

/*
 * Bit definition for GPIOB_DRV[6:4] - GPIOB_1 Drive Current Select
 */
#define GPIOB_DRV_IO1_SEL_POS                    (4)
#define GPIOB_DRV_IO1_SEL_MSK                    (0x7UL << GPIOB_DRV_IO1_SEL_POS)
#define GPIOB_DRV_IO1_16mA                       (0x7UL << GPIOB_DRV_IO1_SEL_POS)
#define GPIOB_DRV_IO1_14mA                       (0x6UL << GPIOB_DRV_IO1_SEL_POS)
#define GPIOB_DRV_IO1_12mA                       (0x5UL << GPIOB_DRV_IO1_SEL_POS)
#define GPIOB_DRV_IO1_10mA                       (0x4UL << GPIOB_DRV_IO1_SEL_POS)
#define GPIOB_DRV_IO1_8mA                        (0x3UL << GPIOB_DRV_IO1_SEL_POS)
#define GPIOB_DRV_IO1_6mA                        (0x2UL << GPIOB_DRV_IO1_SEL_POS)
#define GPIOB_DRV_IO1_4mA                        (0x1UL << GPIOB_DRV_IO1_SEL_POS)
#define GPIOB_DRV_IO1_2mA                        (0x0UL << GPIOB_DRV_IO1_SEL_POS)

/*
 * Bit definition for GPIOB_DRV[2:0] - GPIOB_0 Drive Current Select
 */
#define GPIOB_DRV_IO0_SEL_POS                    (0)
#define GPIOB_DRV_IO0_SEL_MSK                    (0x7UL << GPIOB_DRV_IO0_SEL_POS)
#define GPIOB_DRV_IO0_16mA                       (0x7UL << GPIOB_DRV_IO0_SEL_POS)
#define GPIOB_DRV_IO0_14mA                       (0x6UL << GPIOB_DRV_IO0_SEL_POS)
#define GPIOB_DRV_IO0_12mA                       (0x5UL << GPIOB_DRV_IO0_SEL_POS)
#define GPIOB_DRV_IO0_10mA                       (0x4UL << GPIOB_DRV_IO0_SEL_POS)
#define GPIOB_DRV_IO0_8mA                        (0x3UL << GPIOB_DRV_IO0_SEL_POS)
#define GPIOB_DRV_IO0_6mA                        (0x2UL << GPIOB_DRV_IO0_SEL_POS)
#define GPIOB_DRV_IO0_4mA                        (0x1UL << GPIOB_DRV_IO0_SEL_POS)
#define GPIOB_DRV_IO0_2mA                        (0x0UL << GPIOB_DRV_IO0_SEL_POS)

/*
 * Bit definition for GPIOC_DRV[2] - GPIOC_2 Drive Current Select
 */
#define GPIOC_DRV_IO2_SEL_POS                     (2)
#define GPIOC_DRV_IO2_SEL_MSK                     (0x1UL << GPIOC_DRV_IO2_SEL_POS)
#define GPIOC_DRV_IO2_16mA                        (0x1UL << GPIOC_DRV_IO2_SEL_POS)
#define GPIOC_DRV_IO2_8mA                         (0x0UL << GPIOC_DRV_IO2_SEL_POS)

/*
 * Bit definition for GPIOC_DRV[1] - GPIOC_1 Drive Current Select
 */
#define GPIOC_DRV_IO1_SEL_POS                     (1)
#define GPIOC_DRV_IO1_SEL_MSK                     (0x1UL << GPIOC_DRV_IO1_SEL_POS)
#define GPIOC_DRV_IO1_16mA                        (0x1UL << GPIOC_DRV_IO1_SEL_POS)
#define GPIOC_DRV_IO1_8mA                         (0x0UL << GPIOC_DRV_IO1_SEL_POS)

/*
 * Bit definition for GPIOC_DRV[0] - GPIOC_0 Drive Current Select
 */
#define GPIOC_DRV_IO0_SEL_POS                     (0)
#define GPIOC_DRV_IO0_SEL_MSK                     (0x1UL << GPIOC_DRV_IO0_SEL_POS)
#define GPIOC_DRV_IO0_16mA                        (0x1UL << GPIOC_DRV_IO0_SEL_POS)
#define GPIOC_DRV_IO0_8mA                         (0x0UL << GPIOC_DRV_IO0_SEL_POS)

/*
 * Bit definition for GPIOD_DRV[5:3] - GPIOD_1 Drive Current Select
 */
#define GPIOD_DRV_IO1_SEL_POS                    (3)
#define GPIOD_DRV_IO1_SEL_MSK                    (0x7UL << GPIOD_DRV_IO1_SEL_POS)
#define GPIOD_DRV_IO1_16mA                       (0x7UL << GPIOD_DRV_IO1_SEL_POS)
#define GPIOD_DRV_IO1_14mA                       (0x6UL << GPIOD_DRV_IO1_SEL_POS)
#define GPIOD_DRV_IO1_12mA                       (0x5UL << GPIOD_DRV_IO1_SEL_POS)
#define GPIOD_DRV_IO1_10mA                       (0x4UL << GPIOD_DRV_IO1_SEL_POS)
#define GPIOD_DRV_IO1_8mA                        (0x3UL << GPIOD_DRV_IO1_SEL_POS)
#define GPIOD_DRV_IO1_6mA                        (0x2UL << GPIOD_DRV_IO1_SEL_POS)
#define GPIOD_DRV_IO1_4mA                        (0x1UL << GPIOD_DRV_IO1_SEL_POS)
#define GPIOD_DRV_IO1_2mA                        (0x0UL << GPIOD_DRV_IO1_SEL_POS)

/*
 * Bit definition for GPIOD_DRV[2:0] - GPIOD_0 Drive Current Select
 */
#define GPIOD_DRV_IO0_SEL_POS                    (0)
#define GPIOD_DRV_IO0_SEL_MSK                    (0x7UL << GPIOD_DRV_IO0_SEL_POS)
#define GPIOD_DRV_IO0_16mA                       (0x7UL << GPIOD_DRV_IO0_SEL_POS)
#define GPIOD_DRV_IO0_14mA                       (0x6UL << GPIOD_DRV_IO0_SEL_POS)
#define GPIOD_DRV_IO0_12mA                       (0x5UL << GPIOD_DRV_IO0_SEL_POS)
#define GPIOD_DRV_IO0_10mA                       (0x4UL << GPIOD_DRV_IO0_SEL_POS)
#define GPIOD_DRV_IO0_8mA                        (0x3UL << GPIOD_DRV_IO0_SEL_POS)
#define GPIOD_DRV_IO0_6mA                        (0x2UL << GPIOD_DRV_IO0_SEL_POS)
#define GPIOD_DRV_IO0_4mA                        (0x1UL << GPIOD_DRV_IO0_SEL_POS)
#define GPIOD_DRV_IO0_2mA                        (0x0UL << GPIOD_DRV_IO0_SEL_POS)

/*
 * Bit definition for GPIOA_IE[31] - GPIOA_31 Input Enable Control
 */
#define GPIOA_IE_IO31_EN_POS                     (31)
#define GPIOA_IE_IO31_EN_MSK                     (0x1UL << GPIOA_IE_IO31_EN_POS)
#define GPIOA_IE_IO31_ENABLE                     (0x1UL << GPIOA_IE_IO31_EN_POS)
#define GPIOA_IE_IO31_DISABLE                    (0x0UL << GPIOA_IE_IO31_EN_POS)

/*
 * Bit definition for GPIOA_IE[30] - GPIOA_30 Input Enable Control
 */
#define GPIOA_IE_IO30_EN_POS                     (30)
#define GPIOA_IE_IO30_EN_MSK                     (0x1UL << GPIOA_IE_IO30_EN_POS)
#define GPIOA_IE_IO30_ENABLE                     (0x1UL << GPIOA_IE_IO30_EN_POS)
#define GPIOA_IE_IO30_DISABLE                    (0x0UL << GPIOA_IE_IO30_EN_POS)

/*
 * Bit definition for GPIOA_IE[29] - GPIOA_29 Input Enable Control
 */
#define GPIOA_IE_IO29_EN_POS                     (29)
#define GPIOA_IE_IO29_EN_MSK                     (0x1UL << GPIOA_IE_IO29_EN_POS)
#define GPIOA_IE_IO29_ENABLE                     (0x1UL << GPIOA_IE_IO29_EN_POS)
#define GPIOA_IE_IO29_DISABLE                    (0x0UL << GPIOA_IE_IO29_EN_POS)

/*
 * Bit definition for GPIOA_IE[28] - GPIOA_28 Input Enable Control
 */
#define GPIOA_IE_IO28_EN_POS                     (28)
#define GPIOA_IE_IO28_EN_MSK                     (0x1UL << GPIOA_IE_IO28_EN_POS)
#define GPIOA_IE_IO28_ENABLE                     (0x1UL << GPIOA_IE_IO28_EN_POS)
#define GPIOA_IE_IO28_DISABLE                    (0x0UL << GPIOA_IE_IO28_EN_POS)

/*
 * Bit definition for GPIOA_IE[27] - GPIOA_27 Input Enable Control
 */
#define GPIOA_IE_IO27_EN_POS                     (27)
#define GPIOA_IE_IO27_EN_MSK                     (0x1UL << GPIOA_IE_IO27_EN_POS)
#define GPIOA_IE_IO27_ENABLE                     (0x1UL << GPIOA_IE_IO27_EN_POS)
#define GPIOA_IE_IO27_DISABLE                    (0x0UL << GPIOA_IE_IO27_EN_POS)

/*
 * Bit definition for GPIOA_IE[26] - GPIOA_26 Input Enable Control
 */
#define GPIOA_IE_IO26_EN_POS                     (26)
#define GPIOA_IE_IO26_EN_MSK                     (0x1UL << GPIOA_IE_IO26_EN_POS)
#define GPIOA_IE_IO26_ENABLE                     (0x1UL << GPIOA_IE_IO26_EN_POS)
#define GPIOA_IE_IO26_DISABLE                    (0x0UL << GPIOA_IE_IO26_EN_POS)

/*
 * Bit definition for GPIOA_IE[25] - GPIOA_25 Input Enable Control
 */
#define GPIOA_IE_IO25_EN_POS                     (25)
#define GPIOA_IE_IO25_EN_MSK                     (0x1UL << GPIOA_IE_IO25_EN_POS)
#define GPIOA_IE_IO25_ENABLE                     (0x1UL << GPIOA_IE_IO25_EN_POS)
#define GPIOA_IE_IO25_DISABLE                    (0x0UL << GPIOA_IE_IO25_EN_POS)

/*
 * Bit definition for GPIOA_IE[24] - GPIOA_24 Input Enable Control
 */
#define GPIOA_IE_IO24_EN_POS                     (24)
#define GPIOA_IE_IO24_EN_MSK                     (0x1UL << GPIOA_IE_IO24_EN_POS)
#define GPIOA_IE_IO24_ENABLE                     (0x1UL << GPIOA_IE_IO24_EN_POS)
#define GPIOA_IE_IO24_DISABLE                    (0x0UL << GPIOA_IE_IO24_EN_POS)

/*
 * Bit definition for GPIOA_IE[23] - GPIOA_23 Input Enable Control
 */
#define GPIOA_IE_IO23_EN_POS                     (23)
#define GPIOA_IE_IO23_EN_MSK                     (0x1UL << GPIOA_IE_IO23_EN_POS)
#define GPIOA_IE_IO23_ENABLE                     (0x1UL << GPIOA_IE_IO23_EN_POS)
#define GPIOA_IE_IO23_DISABLE                    (0x0UL << GPIOA_IE_IO23_EN_POS)

/*
 * Bit definition for GPIOA_IE[22] - GPIOA_22 Input Enable Control
 */
#define GPIOA_IE_IO22_EN_POS                     (22)
#define GPIOA_IE_IO22_EN_MSK                     (0x1UL << GPIOA_IE_IO22_EN_POS)
#define GPIOA_IE_IO22_ENABLE                     (0x1UL << GPIOA_IE_IO22_EN_POS)
#define GPIOA_IE_IO22_DISABLE                    (0x0UL << GPIOA_IE_IO22_EN_POS)

/*
 * Bit definition for GPIOA_IE[21] - GPIOA_21 Input Enable Control
 */
#define GPIOA_IE_IO21_EN_POS                     (21)
#define GPIOA_IE_IO21_EN_MSK                     (0x1UL << GPIOA_IE_IO21_EN_POS)
#define GPIOA_IE_IO21_ENABLE                     (0x1UL << GPIOA_IE_IO21_EN_POS)
#define GPIOA_IE_IO21_DISABLE                    (0x0UL << GPIOA_IE_IO21_EN_POS)

/*
 * Bit definition for GPIOA_IE[20] - GPIOA_20 Input Enable Control
 */
#define GPIOA_IE_IO20_EN_POS                     (20)
#define GPIOA_IE_IO20_EN_MSK                     (0x1UL << GPIOA_IE_IO20_EN_POS)
#define GPIOA_IE_IO20_ENABLE                     (0x1UL << GPIOA_IE_IO20_EN_POS)
#define GPIOA_IE_IO20_DISABLE                    (0x0UL << GPIOA_IE_IO20_EN_POS)

/*
 * Bit definition for GPIOA_IE[19] - GPIOA_19 Input Enable Control
 */
#define GPIOA_IE_IO19_EN_POS                     (19)
#define GPIOA_IE_IO19_EN_MSK                     (0x1UL << GPIOA_IE_IO19_EN_POS)
#define GPIOA_IE_IO19_ENABLE                     (0x1UL << GPIOA_IE_IO19_EN_POS)
#define GPIOA_IE_IO19_DISABLE                    (0x0UL << GPIOA_IE_IO19_EN_POS)

/*
 * Bit definition for GPIOA_IE[18] - GPIOA_18 Input Enable Control
 */
#define GPIOA_IE_IO18_EN_POS                     (18)
#define GPIOA_IE_IO18_EN_MSK                     (0x1UL << GPIOA_IE_IO18_EN_POS)
#define GPIOA_IE_IO18_ENABLE                     (0x1UL << GPIOA_IE_IO18_EN_POS)
#define GPIOA_IE_IO18_DISABLE                    (0x0UL << GPIOA_IE_IO18_EN_POS)

/*
 * Bit definition for GPIOA_IE[17] - GPIOA_17 Input Enable Control
 */
#define GPIOA_IE_IO17_EN_POS                     (17)
#define GPIOA_IE_IO17_EN_MSK                     (0x1UL << GPIOA_IE_IO17_EN_POS)
#define GPIOA_IE_IO17_ENABLE                     (0x1UL << GPIOA_IE_IO17_EN_POS)
#define GPIOA_IE_IO17_DISABLE                    (0x0UL << GPIOA_IE_IO17_EN_POS)

/*
 * Bit definition for GPIOA_IE[16] - GPIOA_16 Input Enable Control
 */
#define GPIOA_IE_IO16_EN_POS                     (16)
#define GPIOA_IE_IO16_EN_MSK                     (0x1UL << GPIOA_IE_IO16_EN_POS)
#define GPIOA_IE_IO16_ENABLE                     (0x1UL << GPIOA_IE_IO16_EN_POS)
#define GPIOA_IE_IO16_DISABLE                    (0x0UL << GPIOA_IE_IO16_EN_POS)

/*
 * Bit definition for GPIOA_IE[15] - GPIOA_15 Input Enable Control
 */
#define GPIOA_IE_IO15_EN_POS                     (15)
#define GPIOA_IE_IO15_EN_MSK                     (0x1UL << GPIOA_IE_IO15_EN_POS)
#define GPIOA_IE_IO15_ENABLE                     (0x1UL << GPIOA_IE_IO15_EN_POS)
#define GPIOA_IE_IO15_DISABLE                    (0x0UL << GPIOA_IE_IO15_EN_POS)

/*
 * Bit definition for GPIOA_IE[14] - GPIOA_14 Input Enable Control
 */
#define GPIOA_IE_IO14_EN_POS                     (14)
#define GPIOA_IE_IO14_EN_MSK                     (0x1UL << GPIOA_IE_IO14_EN_POS)
#define GPIOA_IE_IO14_ENABLE                     (0x1UL << GPIOA_IE_IO14_EN_POS)
#define GPIOA_IE_IO14_DISABLE                    (0x0UL << GPIOA_IE_IO14_EN_POS)

/*
 * Bit definition for GPIOA_IE[13] - GPIOA_13 Input Enable Control
 */
#define GPIOA_IE_IO13_EN_POS                     (13)
#define GPIOA_IE_IO13_EN_MSK                     (0x1UL << GPIOA_IE_IO13_EN_POS)
#define GPIOA_IE_IO13_ENABLE                     (0x1UL << GPIOA_IE_IO13_EN_POS)
#define GPIOA_IE_IO13_DISABLE                    (0x0UL << GPIOA_IE_IO13_EN_POS)

/*
 * Bit definition for GPIOA_IE[12] - GPIOA_12 Input Enable Control
 */
#define GPIOA_IE_IO12_EN_POS                     (12)
#define GPIOA_IE_IO12_EN_MSK                     (0x1UL << GPIOA_IE_IO12_EN_POS)
#define GPIOA_IE_IO12_ENABLE                     (0x1UL << GPIOA_IE_IO12_EN_POS)
#define GPIOA_IE_IO12_DISABLE                    (0x0UL << GPIOA_IE_IO12_EN_POS)

/*
 * Bit definition for GPIOA_IE[11] - GPIOA_11 Input Enable Control
 */
#define GPIOA_IE_IO11_EN_POS                     (11)
#define GPIOA_IE_IO11_EN_MSK                     (0x1UL << GPIOA_IE_IO11_EN_POS)
#define GPIOA_IE_IO11_ENABLE                     (0x1UL << GPIOA_IE_IO11_EN_POS)
#define GPIOA_IE_IO11_DISABLE                    (0x0UL << GPIOA_IE_IO11_EN_POS)

/*
 * Bit definition for GPIOA_IE[10] - GPIOA_10 Input Enable Control
 */
#define GPIOA_IE_IO10_EN_POS                     (10)
#define GPIOA_IE_IO10_EN_MSK                     (0x1UL << GPIOA_IE_IO10_EN_POS)
#define GPIOA_IE_IO10_ENABLE                     (0x1UL << GPIOA_IE_IO10_EN_POS)
#define GPIOA_IE_IO10_DISABLE                    (0x0UL << GPIOA_IE_IO10_EN_POS)

/*
 * Bit definition for GPIOA_IE[9] - GPIOA_9 Input Enable Control
 */
#define GPIOA_IE_IO9_EN_POS                      (9)
#define GPIOA_IE_IO9_EN_MSK                      (0x1UL << GPIOA_IE_IO9_EN_POS)
#define GPIOA_IE_IO9_ENABLE                      (0x1UL << GPIOA_IE_IO9_EN_POS)
#define GPIOA_IE_IO9_DISABLE                     (0x0UL << GPIOA_IE_IO9_EN_POS)

/*
 * Bit definition for GPIOA_IE[8] - GPIOA_8 Input Enable Control
 */
#define GPIOA_IE_IO8_EN_POS                      (8)
#define GPIOA_IE_IO8_EN_MSK                      (0x1UL << GPIOA_IE_IO8_EN_POS)
#define GPIOA_IE_IO8_ENABLE                      (0x1UL << GPIOA_IE_IO8_EN_POS)
#define GPIOA_IE_IO8_DISABLE                     (0x0UL << GPIOA_IE_IO8_EN_POS)

/*
 * Bit definition for GPIOA_IE[7] - GPIOA_7 Input Enable Control
 */
#define GPIOA_IE_IO7_EN_POS                      (7)
#define GPIOA_IE_IO7_EN_MSK                      (0x1UL << GPIOA_IE_IO7_EN_POS)
#define GPIOA_IE_IO7_ENABLE                      (0x1UL << GPIOA_IE_IO7_EN_POS)
#define GPIOA_IE_IO7_DISABLE                     (0x0UL << GPIOA_IE_IO7_EN_POS)

/*
 * Bit definition for GPIOA_IE[6] - GPIOA_6 Input Enable Control
 */
#define GPIOA_IE_IO6_EN_POS                      (6)
#define GPIOA_IE_IO6_EN_MSK                      (0x1UL << GPIOA_IE_IO6_EN_POS)
#define GPIOA_IE_IO6_ENABLE                      (0x1UL << GPIOA_IE_IO6_EN_POS)
#define GPIOA_IE_IO6_DISABLE                     (0x0UL << GPIOA_IE_IO6_EN_POS)

/*
 * Bit definition for GPIOA_IE[5] - GPIOA_5 Input Enable Control
 */
#define GPIOA_IE_IO5_EN_POS                      (5)
#define GPIOA_IE_IO5_EN_MSK                      (0x1UL << GPIOA_IE_IO5_EN_POS)
#define GPIOA_IE_IO5_ENABLE                      (0x1UL << GPIOA_IE_IO5_EN_POS)
#define GPIOA_IE_IO5_DISABLE                     (0x0UL << GPIOA_IE_IO5_EN_POS)

/*
 * Bit definition for GPIOA_IE[4] - GPIOA_4 Input Enable Control
 */
#define GPIOA_IE_IO4_EN_POS                      (4)
#define GPIOA_IE_IO4_EN_MSK                      (0x1UL << GPIOA_IE_IO4_EN_POS)
#define GPIOA_IE_IO4_ENABLE                      (0x1UL << GPIOA_IE_IO4_EN_POS)
#define GPIOA_IE_IO4_DISABLE                     (0x0UL << GPIOA_IE_IO4_EN_POS)

/*
 * Bit definition for GPIOA_IE[3] - GPIOA_3 Input Enable Control
 */
#define GPIOA_IE_IO3_EN_POS                      (3)
#define GPIOA_IE_IO3_EN_MSK                      (0x1UL << GPIOA_IE_IO3_EN_POS)
#define GPIOA_IE_IO3_ENABLE                      (0x1UL << GPIOA_IE_IO3_EN_POS)
#define GPIOA_IE_IO3_DISABLE                     (0x0UL << GPIOA_IE_IO3_EN_POS)

/*
 * Bit definition for GPIOA_IE[2] - GPIOA_2 Input Enable Control
 */
#define GPIOA_IE_IO2_EN_POS                      (2)
#define GPIOA_IE_IO2_EN_MSK                      (0x1UL << GPIOA_IE_IO2_EN_POS)
#define GPIOA_IE_IO2_ENABLE                      (0x1UL << GPIOA_IE_IO2_EN_POS)
#define GPIOA_IE_IO2_DISABLE                     (0x0UL << GPIOA_IE_IO2_EN_POS)

/*
 * Bit definition for GPIOA_IE[1] - GPIOA_1 Input Enable Control
 */
#define GPIOA_IE_IO1_EN_POS                      (1)
#define GPIOA_IE_IO1_EN_MSK                      (0x1UL << GPIOA_IE_IO1_EN_POS)
#define GPIOA_IE_IO1_ENABLE                      (0x1UL << GPIOA_IE_IO1_EN_POS)
#define GPIOA_IE_IO1_DISABLE                     (0x0UL << GPIOA_IE_IO1_EN_POS)

/*
 * Bit definition for GPIOA_IE[0] - GPIOA_0 Input Enable Control
 */
#define GPIOA_IE_IO0_EN_POS                      (0)
#define GPIOA_IE_IO0_EN_MSK                      (0x1UL << GPIOA_IE_IO0_EN_POS)
#define GPIOA_IE_IO0_ENABLE                      (0x1UL << GPIOA_IE_IO0_EN_POS)
#define GPIOA_IE_IO0_DISABLE                     (0x0UL << GPIOA_IE_IO0_EN_POS)

/*
 * Bit definition for GPIO_SMT[31] - GPIOA_31 schmitt control register.
 */
#define GPIO_SMT_IO31_EN_POS                     (31)
#define GPIO_SMT_IO31_EN_MSK                     (0x1UL << GPIO_SMT_IO31_EN_POS)
#define GPIO_SMT_IO31_EN_ENABLE                  (0x1UL << GPIO_SMT_IO31_EN_POS)
#define GPIO_SMT_IO31_EN_DISABLE                 (0x0UL << GPIO_SMT_IO31_EN_POS)

/*
 * Bit definition for GPIO_SMT[30] - GPIOA_30 schmitt control register.
 */
#define GPIO_SMT_IO30_EN_POS                     (30)
#define GPIO_SMT_IO30_EN_MSK                     (0x1UL << GPIO_SMT_IO30_EN_POS)
#define GPIO_SMT_IO30_EN_ENABLE                  (0x1UL << GPIO_SMT_IO30_EN_POS)
#define GPIO_SMT_IO30_EN_DISABLE                 (0x0UL << GPIO_SMT_IO30_EN_POS)

/*
 * Bit definition for GPIO_SMT[29] - GPIOA_29 schmitt control register.
 */
#define GPIO_SMT_IO29_EN_POS                     (29)
#define GPIO_SMT_IO29_EN_MSK                     (0x1UL << GPIO_SMT_IO29_EN_POS)
#define GPIO_SMT_IO29_EN_ENABLE                  (0x1UL << GPIO_SMT_IO29_EN_POS)
#define GPIO_SMT_IO29_EN_DISABLE                 (0x0UL << GPIO_SMT_IO29_EN_POS)

/*
 * Bit definition for GPIO_SMT[28] - GPIOA_28 schmitt control register.
 */
#define GPIO_SMT_IO28_EN_POS                     (28)
#define GPIO_SMT_IO28_EN_MSK                     (0x1UL << GPIO_SMT_IO28_EN_POS)
#define GPIO_SMT_IO28_EN_ENABLE                  (0x1UL << GPIO_SMT_IO28_EN_POS)
#define GPIO_SMT_IO28_EN_DISABLE                 (0x0UL << GPIO_SMT_IO28_EN_POS)

/*
 * Bit definition for GPIO_SMT[27] - GPIOA_27 schmitt control register.
 */
#define GPIO_SMT_IO27_EN_POS                     (27)
#define GPIO_SMT_IO27_EN_MSK                     (0x1UL << GPIO_SMT_IO27_EN_POS)
#define GPIO_SMT_IO27_EN_ENABLE                  (0x1UL << GPIO_SMT_IO27_EN_POS)
#define GPIO_SMT_IO27_EN_DISABLE                 (0x0UL << GPIO_SMT_IO27_EN_POS)

/*
 * Bit definition for GPIO_SMT[26] - GPIOA_26 schmitt control register.
 */
#define GPIO_SMT_IO26_EN_POS                     (26)
#define GPIO_SMT_IO26_EN_MSK                     (0x1UL << GPIO_SMT_IO26_EN_POS)
#define GPIO_SMT_IO26_EN_ENABLE                  (0x1UL << GPIO_SMT_IO26_EN_POS)
#define GPIO_SMT_IO26_EN_DISABLE                 (0x0UL << GPIO_SMT_IO26_EN_POS)

/*
 * Bit definition for GPIO_SMT[25] - GPIOA_25 schmitt control register.
 */
#define GPIO_SMT_IO25_EN_POS                     (25)
#define GPIO_SMT_IO25_EN_MSK                     (0x1UL << GPIO_SMT_IO25_EN_POS)
#define GPIO_SMT_IO25_EN_ENABLE                  (0x1UL << GPIO_SMT_IO25_EN_POS)
#define GPIO_SMT_IO25_EN_DISABLE                 (0x0UL << GPIO_SMT_IO25_EN_POS)

/*
 * Bit definition for GPIO_SMT[24] - GPIOA_24 schmitt control register.
 */
#define GPIO_SMT_IO24_EN_POS                     (24)
#define GPIO_SMT_IO24_EN_MSK                     (0x1UL << GPIO_SMT_IO24_EN_POS)
#define GPIO_SMT_IO24_EN_ENABLE                  (0x1UL << GPIO_SMT_IO24_EN_POS)
#define GPIO_SMT_IO24_EN_DISABLE                 (0x0UL << GPIO_SMT_IO24_EN_POS)

/*
 * Bit definition for GPIO_SMT[23] - GPIOA_23 schmitt control register.
 */
#define GPIO_SMT_IO23_EN_POS                     (23)
#define GPIO_SMT_IO23_EN_MSK                     (0x1UL << GPIO_SMT_IO23_EN_POS)
#define GPIO_SMT_IO23_EN_ENABLE                  (0x1UL << GPIO_SMT_IO23_EN_POS)
#define GPIO_SMT_IO23_EN_DISABLE                 (0x0UL << GPIO_SMT_IO23_EN_POS)

/*
 * Bit definition for GPIO_SMT[22] - GPIOA_22 schmitt control register.
 */
#define GPIO_SMT_IO22_EN_POS                     (22)
#define GPIO_SMT_IO22_EN_MSK                     (0x1UL << GPIO_SMT_IO22_EN_POS)
#define GPIO_SMT_IO22_EN_ENABLE                  (0x1UL << GPIO_SMT_IO22_EN_POS)
#define GPIO_SMT_IO22_EN_DISABLE                 (0x0UL << GPIO_SMT_IO22_EN_POS)

/*
 * Bit definition for GPIO_SMT[21] - GPIOA_21 schmitt control register.
 */
#define GPIO_SMT_IO21_EN_POS                     (21)
#define GPIO_SMT_IO21_EN_MSK                     (0x1UL << GPIO_SMT_IO21_EN_POS)
#define GPIO_SMT_IO21_EN_ENABLE                  (0x1UL << GPIO_SMT_IO21_EN_POS)
#define GPIO_SMT_IO21_EN_DISABLE                 (0x0UL << GPIO_SMT_IO21_EN_POS)

/*
 * Bit definition for GPIO_SMT[20] - GPIOA_20 schmitt control register.
 */
#define GPIO_SMT_IO20_EN_POS                     (20)
#define GPIO_SMT_IO20_EN_MSK                     (0x1UL << GPIO_SMT_IO20_EN_POS)
#define GPIO_SMT_IO20_EN_ENABLE                  (0x1UL << GPIO_SMT_IO20_EN_POS)
#define GPIO_SMT_IO20_EN_DISABLE                 (0x0UL << GPIO_SMT_IO20_EN_POS)

/*
 * Bit definition for GPIO_SMT[19] - GPIOA_19 schmitt control register.
 */
#define GPIO_SMT_IO19_EN_POS                     (19)
#define GPIO_SMT_IO19_EN_MSK                     (0x1UL << GPIO_SMT_IO19_EN_POS)
#define GPIO_SMT_IO19_EN_ENABLE                  (0x1UL << GPIO_SMT_IO19_EN_POS)
#define GPIO_SMT_IO19_EN_DISABLE                 (0x0UL << GPIO_SMT_IO19_EN_POS)

/*
 * Bit definition for GPIO_SMT[18] - GPIOA_18 schmitt control register.
 */
#define GPIO_SMT_IO18_EN_POS                     (18)
#define GPIO_SMT_IO18_EN_MSK                     (0x1UL << GPIO_SMT_IO18_EN_POS)
#define GPIO_SMT_IO18_EN_ENABLE                  (0x1UL << GPIO_SMT_IO18_EN_POS)
#define GPIO_SMT_IO18_EN_DISABLE                 (0x0UL << GPIO_SMT_IO18_EN_POS)

/*
 * Bit definition for GPIO_SMT[17] - GPIOA_17 schmitt control register.
 */
#define GPIO_SMT_IO17_EN_POS                     (17)
#define GPIO_SMT_IO17_EN_MSK                     (0x1UL << GPIO_SMT_IO17_EN_POS)
#define GPIO_SMT_IO17_EN_ENABLE                  (0x1UL << GPIO_SMT_IO17_EN_POS)
#define GPIO_SMT_IO17_EN_DISABLE                 (0x0UL << GPIO_SMT_IO17_EN_POS)

/*
 * Bit definition for GPIO_SMT[16] - GPIOA_16 schmitt control register.
 */
#define GPIO_SMT_IO16_EN_POS                     (16)
#define GPIO_SMT_IO16_EN_MSK                     (0x1UL << GPIO_SMT_IO16_EN_POS)
#define GPIO_SMT_IO16_EN_ENABLE                  (0x1UL << GPIO_SMT_IO16_EN_POS)
#define GPIO_SMT_IO16_EN_DISABLE                 (0x0UL << GPIO_SMT_IO16_EN_POS)

/*
 * Bit definition for GPIO_SMT[15] - GPIOA_15 schmitt control register.
 */
#define GPIO_SMT_IO15_EN_POS                     (15)
#define GPIO_SMT_IO15_EN_MSK                     (0x1UL << GPIO_SMT_IO15_EN_POS)
#define GPIO_SMT_IO15_EN_ENABLE                  (0x1UL << GPIO_SMT_IO15_EN_POS)
#define GPIO_SMT_IO15_EN_DISABLE                 (0x0UL << GPIO_SMT_IO15_EN_POS)

/*
 * Bit definition for GPIO_SMT[14] - GPIOA_14 schmitt control register.
 */
#define GPIO_SMT_IO14_EN_POS                     (14)
#define GPIO_SMT_IO14_EN_MSK                     (0x1UL << GPIO_SMT_IO14_EN_POS)
#define GPIO_SMT_IO14_EN_ENABLE                  (0x1UL << GPIO_SMT_IO14_EN_POS)
#define GPIO_SMT_IO14_EN_DISABLE                 (0x0UL << GPIO_SMT_IO14_EN_POS)

/*
 * Bit definition for GPIO_SMT[13] - GPIOA_13 schmitt control register.
 */
#define GPIO_SMT_IO13_EN_POS                     (13)
#define GPIO_SMT_IO13_EN_MSK                     (0x1UL << GPIO_SMT_IO13_EN_POS)
#define GPIO_SMT_IO13_EN_ENABLE                  (0x1UL << GPIO_SMT_IO13_EN_POS)
#define GPIO_SMT_IO13_EN_DISABLE                 (0x0UL << GPIO_SMT_IO13_EN_POS)

/*
 * Bit definition for GPIO_SMT[12] - GPIOA_12 schmitt control register.
 */
#define GPIO_SMT_IO12_EN_POS                     (12)
#define GPIO_SMT_IO12_EN_MSK                     (0x1UL << GPIO_SMT_IO12_EN_POS)
#define GPIO_SMT_IO12_EN_ENABLE                  (0x1UL << GPIO_SMT_IO12_EN_POS)
#define GPIO_SMT_IO12_EN_DISABLE                 (0x0UL << GPIO_SMT_IO12_EN_POS)

/*
 * Bit definition for GPIO_SMT[11] - GPIOA_11 schmitt control register.
 */
#define GPIO_SMT_IO11_EN_POS                     (11)
#define GPIO_SMT_IO11_EN_MSK                     (0x1UL << GPIO_SMT_IO11_EN_POS)
#define GPIO_SMT_IO11_EN_ENABLE                  (0x1UL << GPIO_SMT_IO11_EN_POS)
#define GPIO_SMT_IO11_EN_DISABLE                 (0x0UL << GPIO_SMT_IO11_EN_POS)

/*
 * Bit definition for GPIO_SMT[10] - GPIOA_10 schmitt control register.
 */
#define GPIO_SMT_IO10_EN_POS                     (10)
#define GPIO_SMT_IO10_EN_MSK                     (0x1UL << GPIO_SMT_IO10_EN_POS)
#define GPIO_SMT_IO10_EN_ENABLE                  (0x1UL << GPIO_SMT_IO10_EN_POS)
#define GPIO_SMT_IO10_EN_DISABLE                 (0x0UL << GPIO_SMT_IO10_EN_POS)

/*
 * Bit definition for GPIO_SMT[9] - GPIOA_9 schmitt control register.
 */
#define GPIO_SMT_IO9_EN_POS                      (9)
#define GPIO_SMT_IO9_EN_MSK                      (0x1UL << GPIO_SMT_IO9_EN_POS)
#define GPIO_SMT_IO9_EN_ENABLE                   (0x1UL << GPIO_SMT_IO9_EN_POS)
#define GPIO_SMT_IO9_EN_DISABLE                  (0x0UL << GPIO_SMT_IO9_EN_POS)

/*
 * Bit definition for GPIO_SMT[8] - GPIOA_8 schmitt control register.
 */
#define GPIO_SMT_IO8_EN_POS                      (8)
#define GPIO_SMT_IO8_EN_MSK                      (0x1UL << GPIO_SMT_IO8_EN_POS)
#define GPIO_SMT_IO8_EN_ENABLE                   (0x1UL << GPIO_SMT_IO8_EN_POS)
#define GPIO_SMT_IO8_EN_DISABLE                  (0x0UL << GPIO_SMT_IO8_EN_POS)

/*
 * Bit definition for GPIO_SMT[7] - GPIOA_7 schmitt control register.
 */
#define GPIO_SMT_IO7_EN_POS                      (7)
#define GPIO_SMT_IO7_EN_MSK                      (0x1UL << GPIO_SMT_IO7_EN_POS)
#define GPIO_SMT_IO7_EN_ENABLE                   (0x1UL << GPIO_SMT_IO7_EN_POS)
#define GPIO_SMT_IO7_EN_DISABLE                  (0x0UL << GPIO_SMT_IO7_EN_POS)

/*
 * Bit definition for GPIO_SMT[6] - GPIOA_6 schmitt control register.
 */
#define GPIO_SMT_IO6_EN_POS                      (6)
#define GPIO_SMT_IO6_EN_MSK                      (0x1UL << GPIO_SMT_IO6_EN_POS)
#define GPIO_SMT_IO6_EN_ENABLE                   (0x1UL << GPIO_SMT_IO6_EN_POS)
#define GPIO_SMT_IO6_EN_DISABLE                  (0x0UL << GPIO_SMT_IO6_EN_POS)

/*
 * Bit definition for GPIO_SMT[5] - GPIOA/B_5 schmitt control register.
 */
#define GPIO_SMT_IO5_EN_POS                      (5)
#define GPIO_SMT_IO5_EN_MSK                      (0x1UL << GPIO_SMT_IO5_EN_POS)
#define GPIO_SMT_IO5_EN_ENABLE                   (0x1UL << GPIO_SMT_IO5_EN_POS)
#define GPIO_SMT_IO5_EN_DISABLE                  (0x0UL << GPIO_SMT_IO5_EN_POS)

/*
 * Bit definition for GPIO_SMT[4] - GPIOA/B_4 schmitt control register.
 */
#define GPIO_SMT_IO4_EN_POS                      (4)
#define GPIO_SMT_IO4_EN_MSK                      (0x1UL << GPIO_SMT_IO4_EN_POS)
#define GPIO_SMT_IO4_EN_ENABLE                   (0x1UL << GPIO_SMT_IO4_EN_POS)
#define GPIO_SMT_IO4_EN_DISABLE                  (0x0UL << GPIO_SMT_IO4_EN_POS)

/*
 * Bit definition for GPIO_SMT[3] - GPIOA/B_3 schmitt control register.
 */
#define GPIO_SMT_IO3_EN_POS                      (3)
#define GPIO_SMT_IO3_EN_MSK                      (0x1UL << GPIO_SMT_IO3_EN_POS)
#define GPIO_SMT_IO3_EN_ENABLE                   (0x1UL << GPIO_SMT_IO3_EN_POS)
#define GPIO_SMT_IO3_EN_DISABLE                  (0x0UL << GPIO_SMT_IO3_EN_POS)

/*
 * Bit definition for GPIO_SMT[2] - GPIOA/B/C_2 schmitt control register.
 */
#define GPIO_SMT_IO2_EN_POS                      (2)
#define GPIO_SMT_IO2_EN_MSK                      (0x1UL << GPIO_SMT_IO2_EN_POS)
#define GPIO_SMT_IO2_EN_ENABLE                   (0x1UL << GPIO_SMT_IO2_EN_POS)
#define GPIO_SMT_IO2_EN_DISABLE                  (0x0UL << GPIO_SMT_IO2_EN_POS)
/*
 * Bit definition for GPIO_SMT[1] - GPIOA/B/C/D_1 schmitt control register.
 */
#define GPIO_SMT_IO1_EN_POS                      (1)
#define GPIO_SMT_IO1_EN_MSK                      (0x1UL << GPIO_SMT_IO1_EN_POS)
#define GPIO_SMT_IO1_EN_ENABLE                   (0x1UL << GPIO_SMT_IO1_EN_POS)
#define GPIO_SMT_IO1_EN_DISABLE                  (0x0UL << GPIO_SMT_IO1_EN_POS)

/*
 * Bit definition for GPIO_SMT[0] - GPIOA/B/C/D_0 schmitt control register.
 */
#define GPIO_SMT_IO0_EN_POS                      (0)
#define GPIO_SMT_IO0_EN_MSK                      (0x1UL << GPIO_SMT_IO0_EN_POS)
#define GPIO_SMT_IO0_EN_ENABLE                   (0x1UL << GPIO_SMT_IO0_EN_POS)
#define GPIO_SMT_IO0_EN_DISABLE                  (0x0UL << GPIO_SMT_IO0_EN_POS)

typedef struct
{
    __IO uint32_t OBIT00;
    __IO uint32_t OBIT01;
    __IO uint32_t OBIT02;
    __IO uint32_t OBIT03;
    __IO uint32_t OBIT04;
    __IO uint32_t OBIT05;
    __IO uint32_t OBIT06;
    __IO uint32_t OBIT07;
    __IO uint32_t OBIT08;
    __IO uint32_t OBIT09;
    __IO uint32_t OBIT10;
    __IO uint32_t OBIT11;
    __IO uint32_t OBIT12;
    __IO uint32_t OBIT13;
    __IO uint32_t OBIT14;
    __IO uint32_t OBIT15;
    __IO uint32_t OBIT16;
    __IO uint32_t OBIT17;
    __IO uint32_t OBIT18;
    __IO uint32_t OBIT19;
    __IO uint32_t OBIT20;
    __IO uint32_t OBIT21;
    __IO uint32_t OBIT22;
    __IO uint32_t OBIT23;
    __IO uint32_t OBIT24;
    __IO uint32_t OBIT25;
    __IO uint32_t OBIT26;
    __IO uint32_t OBIT27;
    __IO uint32_t OBIT28;
    __IO uint32_t OBIT29;
    __IO uint32_t OBIT30;
    __IO uint32_t OBIT31;
} GPIO_BIT_OPERATION_TYPE_DEF;
/*
 * Bit definition for GPIO_FST[31] - GPIOA_31 IO function priority first enable
 */
#define GPIO_FST_IO31_EN_POS                     (31)
#define GPIO_FST_IO31_EN_MSK                     (0x1UL << GPIO_FST_IO31_EN_POS)
#define GPIO_FST_IO31_EN_ENABLE                  (0x1UL << GPIO_FST_IO31_EN_POS)
#define GPIO_FST_IO31_EN_DISABLE                 (0x0UL << GPIO_FST_IO31_EN_POS)

/*
 * Bit definition for GPIO_FST[30] - GPIOA_30 IO function priority first enable
 */
#define GPIO_FST_IO30_EN_POS                     (30)
#define GPIO_FST_IO30_EN_MSK                     (0x1UL << GPIO_FST_IO30_EN_POS)
#define GPIO_FST_IO30_EN_ENABLE                  (0x1UL << GPIO_FST_IO30_EN_POS)
#define GPIO_FST_IO30_EN_DISABLE                 (0x0UL << GPIO_FST_IO30_EN_POS)

/*
 * Bit definition for GPIO_FST[29] - GPIOA_29 IO function priority first enable
 */
#define GPIO_FST_IO29_EN_POS                     (29)
#define GPIO_FST_IO29_EN_MSK                     (0x1UL << GPIO_FST_IO29_EN_POS)
#define GPIO_FST_IO29_EN_ENABLE                  (0x1UL << GPIO_FST_IO29_EN_POS)
#define GPIO_FST_IO29_EN_DISABLE                 (0x0UL << GPIO_FST_IO29_EN_POS)

/*
 * Bit definition for GPIO_FST[28] - GPIOA_28 IO function priority first enable
 */
#define GPIO_FST_IO28_EN_POS                     (28)
#define GPIO_FST_IO28_EN_MSK                     (0x1UL << GPIO_FST_IO28_EN_POS)
#define GPIO_FST_IO28_EN_ENABLE                  (0x1UL << GPIO_FST_IO28_EN_POS)
#define GPIO_FST_IO28_EN_DISABLE                 (0x0UL << GPIO_FST_IO28_EN_POS)

/*
 * Bit definition for GPIO_FST[27] - GPIOA_27 IO function priority first enable
 */
#define GPIO_FST_IO27_EN_POS                     (27)
#define GPIO_FST_IO27_EN_MSK                     (0x1UL << GPIO_FST_IO27_EN_POS)
#define GPIO_FST_IO27_EN_ENABLE                  (0x1UL << GPIO_FST_IO27_EN_POS)
#define GPIO_FST_IO27_EN_DISABLE                 (0x0UL << GPIO_FST_IO27_EN_POS)

/*
 * Bit definition for GPIO_FST[26] - GPIOA_26 IO function priority first enable
 */
#define GPIO_FST_IO26_EN_POS                     (26)
#define GPIO_FST_IO26_EN_MSK                     (0x1UL << GPIO_FST_IO26_EN_POS)
#define GPIO_FST_IO26_EN_ENABLE                  (0x1UL << GPIO_FST_IO26_EN_POS)
#define GPIO_FST_IO26_EN_DISABLE                 (0x0UL << GPIO_FST_IO26_EN_POS)

/*
 * Bit definition for GPIO_FST[25] - GPIOA_25 IO function priority first enable
 */
#define GPIO_FST_IO25_EN_POS                     (25)
#define GPIO_FST_IO25_EN_MSK                     (0x1UL << GPIO_FST_IO25_EN_POS)
#define GPIO_FST_IO25_EN_ENABLE                  (0x1UL << GPIO_FST_IO25_EN_POS)
#define GPIO_FST_IO25_EN_DISABLE                 (0x0UL << GPIO_FST_IO25_EN_POS)

/*
 * Bit definition for GPIO_FST[24] - GPIOA_24 IO function priority first enable
 */
#define GPIO_FST_IO24_EN_POS                     (24)
#define GPIO_FST_IO24_EN_MSK                     (0x1UL << GPIO_FST_IO24_EN_POS)
#define GPIO_FST_IO24_EN_ENABLE                  (0x1UL << GPIO_FST_IO24_EN_POS)
#define GPIO_FST_IO24_EN_DISABLE                 (0x0UL << GPIO_FST_IO24_EN_POS)

/*
 * Bit definition for GPIO_FST[23] - GPIOA_23 IO function priority first enable
 */
#define GPIO_FST_IO23_EN_POS                     (23)
#define GPIO_FST_IO23_EN_MSK                     (0x1UL << GPIO_FST_IO23_EN_POS)
#define GPIO_FST_IO23_EN_ENABLE                  (0x1UL << GPIO_FST_IO23_EN_POS)
#define GPIO_FST_IO23_EN_DISABLE                 (0x0UL << GPIO_FST_IO23_EN_POS)

/*
 * Bit definition for GPIO_FST[22] - GPIOA_22 IO function priority first enable
 */
#define GPIO_FST_IO22_EN_POS                     (22)
#define GPIO_FST_IO22_EN_MSK                     (0x1UL << GPIO_FST_IO22_EN_POS)
#define GPIO_FST_IO22_EN_ENABLE                  (0x1UL << GPIO_FST_IO22_EN_POS)
#define GPIO_FST_IO22_EN_DISABLE                 (0x0UL << GPIO_FST_IO22_EN_POS)

/*
 * Bit definition for GPIO_FST[21] - GPIOA_21 IO function priority first enable
 */
#define GPIO_FST_IO21_EN_POS                     (21)
#define GPIO_FST_IO21_EN_MSK                     (0x1UL << GPIO_FST_IO21_EN_POS)
#define GPIO_FST_IO21_EN_ENABLE                  (0x1UL << GPIO_FST_IO21_EN_POS)
#define GPIO_FST_IO21_EN_DISABLE                 (0x0UL << GPIO_FST_IO21_EN_POS)

/*
 * Bit definition for GPIO_FST[20] - GPIOA_20 IO function priority first enable
 */
#define GPIO_FST_IO20_EN_POS                     (20)
#define GPIO_FST_IO20_EN_MSK                     (0x1UL << GPIO_FST_IO20_EN_POS)
#define GPIO_FST_IO20_EN_ENABLE                  (0x1UL << GPIO_FST_IO20_EN_POS)
#define GPIO_FST_IO20_EN_DISABLE                 (0x0UL << GPIO_FST_IO20_EN_POS)

/*
 * Bit definition for GPIO_FST[19] - GPIOA_19 IO function priority first enable
 */
#define GPIO_FST_IO19_EN_POS                     (19)
#define GPIO_FST_IO19_EN_MSK                     (0x1UL << GPIO_FST_IO19_EN_POS)
#define GPIO_FST_IO19_EN_ENABLE                  (0x1UL << GPIO_FST_IO19_EN_POS)
#define GPIO_FST_IO19_EN_DISABLE                 (0x0UL << GPIO_FST_IO19_EN_POS)

/*
 * Bit definition for GPIO_FST[18] - GPIOA_18 IO function priority first enable
 */
#define GPIO_FST_IO18_EN_POS                     (18)
#define GPIO_FST_IO18_EN_MSK                     (0x1UL << GPIO_FST_IO18_EN_POS)
#define GPIO_FST_IO18_EN_ENABLE                  (0x1UL << GPIO_FST_IO18_EN_POS)
#define GPIO_FST_IO18_EN_DISABLE                 (0x0UL << GPIO_FST_IO18_EN_POS)

/*
 * Bit definition for GPIO_FST[17] - GPIOA_17 IO function priority first enable
 */
#define GPIO_FST_IO17_EN_POS                     (17)
#define GPIO_FST_IO17_EN_MSK                     (0x1UL << GPIO_FST_IO17_EN_POS)
#define GPIO_FST_IO17_EN_ENABLE                  (0x1UL << GPIO_FST_IO17_EN_POS)
#define GPIO_FST_IO17_EN_DISABLE                 (0x0UL << GPIO_FST_IO17_EN_POS)

/*
 * Bit definition for GPIO_FST[16] - GPIOA_16 IO function priority first enable
 */
#define GPIO_FST_IO16_EN_POS                     (16)
#define GPIO_FST_IO16_EN_MSK                     (0x1UL << GPIO_FST_IO16_EN_POS)
#define GPIO_FST_IO16_EN_ENABLE                  (0x1UL << GPIO_FST_IO16_EN_POS)
#define GPIO_FST_IO16_EN_DISABLE                 (0x0UL << GPIO_FST_IO16_EN_POS)

/*
 * Bit definition for GPIO_FST[15] - GPIOA_15 IO function priority first enable
 */
#define GPIO_FST_IO15_EN_POS                     (15)
#define GPIO_FST_IO15_EN_MSK                     (0x1UL << GPIO_FST_IO15_EN_POS)
#define GPIO_FST_IO15_EN_ENABLE                  (0x1UL << GPIO_FST_IO15_EN_POS)
#define GPIO_FST_IO15_EN_DISABLE                 (0x0UL << GPIO_FST_IO15_EN_POS)

/*
 * Bit definition for GPIO_FST[14] - GPIOA_14 IO function priority first enable
 */
#define GPIO_FST_IO14_EN_POS                     (14)
#define GPIO_FST_IO14_EN_MSK                     (0x1UL << GPIO_FST_IO14_EN_POS)
#define GPIO_FST_IO14_EN_ENABLE                  (0x1UL << GPIO_FST_IO14_EN_POS)
#define GPIO_FST_IO14_EN_DISABLE                 (0x0UL << GPIO_FST_IO14_EN_POS)

/*
 * Bit definition for GPIO_FST[13] - GPIOA_13 IO function priority first enable
 */
#define GPIO_FST_IO13_EN_POS                     (13)
#define GPIO_FST_IO13_EN_MSK                     (0x1UL << GPIO_FST_IO13_EN_POS)
#define GPIO_FST_IO13_EN_ENABLE                  (0x1UL << GPIO_FST_IO13_EN_POS)
#define GPIO_FST_IO13_EN_DISABLE                 (0x0UL << GPIO_FST_IO13_EN_POS)

/*
 * Bit definition for GPIO_FST[12] - GPIOA_12 IO function priority first enable
 */
#define GPIO_FST_IO12_EN_POS                     (12)
#define GPIO_FST_IO12_EN_MSK                     (0x1UL << GPIO_FST_IO12_EN_POS)
#define GPIO_FST_IO12_EN_ENABLE                  (0x1UL << GPIO_FST_IO12_EN_POS)
#define GPIO_FST_IO12_EN_DISABLE                 (0x0UL << GPIO_FST_IO12_EN_POS)

/*
 * Bit definition for GPIO_FST[11] - GPIOA_11 IO function priority first enable
 */
#define GPIO_FST_IO11_EN_POS                     (11)
#define GPIO_FST_IO11_EN_MSK                     (0x1UL << GPIO_FST_IO11_EN_POS)
#define GPIO_FST_IO11_EN_ENABLE                  (0x1UL << GPIO_FST_IO11_EN_POS)
#define GPIO_FST_IO11_EN_DISABLE                 (0x0UL << GPIO_FST_IO11_EN_POS)

/*
 * Bit definition for GPIO_FST[10] - GPIOA_10 IO function priority first enable
 */
#define GPIO_FST_IO10_EN_POS                     (10)
#define GPIO_FST_IO10_EN_MSK                     (0x1UL << GPIO_FST_IO10_EN_POS)
#define GPIO_FST_IO10_EN_ENABLE                  (0x1UL << GPIO_FST_IO10_EN_POS)
#define GPIO_FST_IO10_EN_DISABLE                 (0x0UL << GPIO_FST_IO10_EN_POS)

/*
 * Bit definition for GPIO_FST[9] - GPIOA_9 IO function priority first enable
 */
#define GPIO_FST_IO9_EN_POS                      (9)
#define GPIO_FST_IO9_EN_MSK                      (0x1UL << GPIO_FST_IO9_EN_POS)
#define GPIO_FST_IO9_EN_ENABLE                   (0x1UL << GPIO_FST_IO9_EN_POS)
#define GPIO_FST_IO9_EN_DISABLE                  (0x0UL << GPIO_FST_IO9_EN_POS)

/*
 * Bit definition for GPIO_FST[8] - GPIOA_8 IO function priority first enable
 */
#define GPIO_FST_IO8_EN_POS                      (8)
#define GPIO_FST_IO8_EN_MSK                      (0x1UL << GPIO_FST_IO8_EN_POS)
#define GPIO_FST_IO8_EN_ENABLE                   (0x1UL << GPIO_FST_IO8_EN_POS)
#define GPIO_FST_IO8_EN_DISABLE                  (0x0UL << GPIO_FST_IO8_EN_POS)

/*
 * Bit definition for GPIO_FST[7] - GPIOA_7 IO function priority first enable
 */
#define GPIO_FST_IO7_EN_POS                      (7)
#define GPIO_FST_IO7_EN_MSK                      (0x1UL << GPIO_FST_IO7_EN_POS)
#define GPIO_FST_IO7_EN_ENABLE                   (0x1UL << GPIO_FST_IO7_EN_POS)
#define GPIO_FST_IO7_EN_DISABLE                  (0x0UL << GPIO_FST_IO7_EN_POS)

/*
 * Bit definition for GPIO_FST[6] - GPIOA_6 IO function priority first enable
 */
#define GPIO_FST_IO6_EN_POS                      (6)
#define GPIO_FST_IO6_EN_MSK                      (0x1UL << GPIO_FST_IO6_EN_POS)
#define GPIO_FST_IO6_EN_ENABLE                   (0x1UL << GPIO_FST_IO6_EN_POS)
#define GPIO_FST_IO6_EN_DISABLE                  (0x0UL << GPIO_FST_IO6_EN_POS)

/*
 * Bit definition for GPIO_FST[5] - GPIOA/B_5 IO function priority first enable
 */
#define GPIO_FST_IO5_EN_POS                      (5)
#define GPIO_FST_IO5_EN_MSK                      (0x1UL << GPIO_FST_IO5_EN_POS)
#define GPIO_FST_IO5_EN_ENABLE                   (0x1UL << GPIO_FST_IO5_EN_POS)
#define GPIO_FST_IO5_EN_DISABLE                  (0x0UL << GPIO_FST_IO5_EN_POS)

/*
 * Bit definition for GPIO_FST[4] - GPIOA/B_4 IO function priority first enable
 */
#define GPIO_FST_IO4_EN_POS                      (4)
#define GPIO_FST_IO4_EN_MSK                      (0x1UL << GPIO_FST_IO4_EN_POS)
#define GPIO_FST_IO4_EN_ENABLE                   (0x1UL << GPIO_FST_IO4_EN_POS)
#define GPIO_FST_IO4_EN_DISABLE                  (0x0UL << GPIO_FST_IO4_EN_POS)

/*
 * Bit definition for GPIO_FST[3] - GPIOA/B_3 IO function priority first enable
 */
#define GPIO_FST_IO3_EN_POS                      (3)
#define GPIO_FST_IO3_EN_MSK                      (0x1UL << GPIO_FST_IO3_EN_POS)
#define GPIO_FST_IO3_EN_ENABLE                   (0x1UL << GPIO_FST_IO3_EN_POS)
#define GPIO_FST_IO3_EN_DISABLE                  (0x0UL << GPIO_FST_IO3_EN_POS)

/*
 * Bit definition for GPIO_FST[2] - GPIOA/B_2 IO function priority first enable
 */
#define GPIO_FST_IO2_EN_POS                      (2)
#define GPIO_FST_IO2_EN_MSK                      (0x1UL << GPIO_FST_IO2_EN_POS)
#define GPIO_FST_IO2_EN_ENABLE                   (0x1UL << GPIO_FST_IO2_EN_POS)
#define GPIO_FST_IO2_EN_DISABLE                  (0x0UL << GPIO_FST_IO2_EN_POS)

/*
 * Bit definition for GPIO_FST[1] - GPIOA/B_1 IO function priority first enable
 */
#define GPIO_FST_IO1_EN_POS                      (1)
#define GPIO_FST_IO1_EN_MSK                      (0x1UL << GPIO_FST_IO1_EN_POS)
#define GPIO_FST_IO1_EN_ENABLE                   (0x1UL << GPIO_FST_IO1_EN_POS)
#define GPIO_FST_IO1_EN_DISABLE                  (0x0UL << GPIO_FST_IO1_EN_POS)

/*
 * Bit definition for GPIO_FST[0] - GPIOA/B_0 IO function priority first enable
 */
#define GPIO_FST_IO0_EN_POS                      (0)
#define GPIO_FST_IO0_EN_MSK                      (0x1UL << GPIO_FST_IO0_EN_POS)
#define GPIO_FST_IO0_EN_ENABLE                   (0x1UL << GPIO_FST_IO0_EN_POS)
#define GPIO_FST_IO0_EN_DISABLE                  (0x0UL << GPIO_FST_IO0_EN_POS)

typedef struct
{
    __IO uint32_t CTRL0;                         // GPIO Function Selection Control Register
    __IO uint32_t CTRL1;                         // GPIO IR Function Control Register
    __IO uint32_t CTRL2;                         // GPIO EXT 0~3 Input Pins Select Control Register
    __I  uint32_t RESERVED[1];                   // Reserved
    __IO uint32_t WAKEEN;                        // GPIO Key-Change Wake Up Enable Register 1
    __IO uint32_t WAKEEN2;                       // GPIO Key-Change Wake Up Enable Register 2
    __IO uint32_t STS;                           // GPIO Key-Change Interrupt Status Register
    __IO uint32_t SPIFCCFG;                      // SPIFC IO Select Register
    __IO uint32_t IOASTS;                        // IOA Key Change INT Status
} GPIOFUNC_TYPE_DEF;

/*
 * Bit definition for GPIOFUNC_CTRL0[30] - CCP0 capture input pins select
 */
#define GPIOFUNC_CTRL0_CCP0_IOSEL_POS            (30)
#define GPIOFUNC_CTRL0_CCP0_IOSEL_MSK            (0x1UL << GPIOFUNC_CTRL0_CCP0_IOSEL_POS)
#define GPIOFUNC_CTRL0_CCP0_IOA3_6               (0x0UL << GPIOFUNC_CTRL0_CCP0_IOSEL_POS)
#define GPIOFUNC_CTRL0_CCP0_IOA13_16             (0x1UL << GPIOFUNC_CTRL0_CCP0_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[29] - PDM pins select
 */
#define GPIOFUNC_CTRL0_PDM_IOSEL_POS             (29)
#define GPIOFUNC_CTRL0_PDM_IOSEL_MSK             (0x1UL << GPIOFUNC_CTRL0_PDM_IOSEL_POS)
#define GPIOFUNC_CTRL0_PDM_IOA22_23              (0x1UL << GPIOFUNC_CTRL0_PDM_IOSEL_POS)
#define GPIOFUNC_CTRL0_PDM_IOA5_6                (0x0UL << GPIOFUNC_CTRL0_PDM_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[28] - Feedback pins select
 */
#define GPIOFUNC_CTRL0_FB_IOSEL_POS              (28)
#define GPIOFUNC_CTRL0_FB_IOSEL_MSK              (0x1UL << GPIOFUNC_CTRL0_FB_IOSEL_POS)
#define GPIOFUNC_CTRL0_FB_IOA3_4                 (0x0UL << GPIOFUNC_CTRL0_FB_IOSEL_POS)
#define GPIOFUNC_CTRL0_FB_IOA8_7                 (0x1UL << GPIOFUNC_CTRL0_FB_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[27] - UART1 TX invert Enable
 */
#define GPIOFUNC_CTRL0_UART1_TXINV_POS           (27)
#define GPIOFUNC_CTRL0_UART1_TXINV_MSK           (0x1UL << GPIOFUNC_CTRL0_UART1_TXINV_POS)
#define GPIOFUNC_CTRL0_UART1_TXINV_DISABLE       (0x0UL << GPIOFUNC_CTRL0_UART1_TXINV_POS)
#define GPIOFUNC_CTRL0_UART1_TXINV_ENABLE        (0x1UL << GPIOFUNC_CTRL0_UART1_TXINV_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[26] - UART0 TX invert Enable
 */
#define GPIOFUNC_CTRL0_UART0_TXINV_POS           (26)
#define GPIOFUNC_CTRL0_UART0_TXINV_MSK           (0x1UL << GPIOFUNC_CTRL0_UART0_TXINV_POS)
#define GPIOFUNC_CTRL0_UART0_TXINV_DISABLE       (0x0UL << GPIOFUNC_CTRL0_UART0_TXINV_POS)
#define GPIOFUNC_CTRL0_UART0_TXINV_ENABLE        (0x1UL << GPIOFUNC_CTRL0_UART0_TXINV_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[25] - I2S Out pins Select
 */
#define GPIOFUNC_CTRL0_I2S_OUT_IOSEL_POS         (25)
#define GPIOFUNC_CTRL0_I2S_OUT_IOSEL_MSK         (0x1UL << GPIOFUNC_CTRL0_I2S_OUT_IOSEL_POS)
#define GPIOFUNC_CTRL0_I2S_OUT_IOA9_12           (0x0UL << GPIOFUNC_CTRL0_I2S_OUT_IOSEL_POS)
#define GPIOFUNC_CTRL0_I2S_OUT_IOA22_25          (0x1UL << GPIOFUNC_CTRL0_I2S_OUT_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[24] - I2S In pins Select
 */
#define GPIOFUNC_CTRL0_I2S_IN_IOSEL_POS          (24)
#define GPIOFUNC_CTRL0_I2S_IN_IOSEL_MSK          (0x1UL << GPIOFUNC_CTRL0_I2S_IN_IOSEL_POS)
#define GPIOFUNC_CTRL0_I2S_IN_IOA3_6             (0x0UL << GPIOFUNC_CTRL0_I2S_IN_IOSEL_POS)
#define GPIOFUNC_CTRL0_I2S_IN_IOA26_29           (0x1UL << GPIOFUNC_CTRL0_I2S_IN_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[23] - SPI0 MOSI/MISO swap
 */
#define GPIOFUNC_CTRL0_SPI0_SWAP_POS             (23)
#define GPIOFUNC_CTRL0_SPI0_SWAP_MSK             (0x1UL << GPIOFUNC_CTRL0_SPI0_SWAP_POS)
#define GPIOFUNC_CTRL0_SPI0_SWAP_Keep            (0x0UL << GPIOFUNC_CTRL0_SPI0_SWAP_POS)
#define GPIOFUNC_CTRL0_SPI0_SWAP                 (0x1UL << GPIOFUNC_CTRL0_SPI0_SWAP_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[22:21] - SPI0 pins Select
 */
#define GPIOFUNC_CTRL0_SPI0_IOSEL_POS            (21)
#define GPIOFUNC_CTRL0_SPI0_IOSEL_MSK            (0x3UL << GPIOFUNC_CTRL0_SPI0_IOSEL_POS)
#define GPIOFUNC_CTRL0_SPI0_IOB1_4               (0x2UL << GPIOFUNC_CTRL0_SPI0_IOSEL_POS)
#define GPIOFUNC_CTRL0_SPI0_IOA26_29             (0x1UL << GPIOFUNC_CTRL0_SPI0_IOSEL_POS)
#define GPIOFUNC_CTRL0_SPI0_IOA3_6               (0x0UL << GPIOFUNC_CTRL0_SPI0_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[20] - UART1 TX/RX swap
 */
#define GPIOFUNC_CTRL0_UART1_SWAP_POS            (20)
#define GPIOFUNC_CTRL0_UART1_SWAP_MSK            (0x1UL << GPIOFUNC_CTRL0_UART1_SWAP_POS)
#define GPIOFUNC_CTRL0_UART1_SWAP_Keep           (0x0UL << GPIOFUNC_CTRL0_UART1_SWAP_POS)
#define GPIOFUNC_CTRL0_UART1_SWAP                (0x1UL << GPIOFUNC_CTRL0_UART1_SWAP_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[19:18] - UART1 pins select
 */
#define GPIOFUNC_CTRL0_UART1_IOSEL_POS           (18)
#define GPIOFUNC_CTRL0_UART1_IOSEL_MSK           (0x3UL << GPIOFUNC_CTRL0_UART1_IOSEL_POS)
#define GPIOFUNC_CTRL0_UART1_IOA26_27            (0x2UL << GPIOFUNC_CTRL0_UART1_IOSEL_POS)
#define GPIOFUNC_CTRL0_UART1_IOA20_21            (0x1UL << GPIOFUNC_CTRL0_UART1_IOSEL_POS)
#define GPIOFUNC_CTRL0_UART1_IOA5_6              (0x0UL << GPIOFUNC_CTRL0_UART1_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[17:16] - I2C pins Select
 */
#define GPIOFUNC_CTRL0_I2C_IOSEL_POS             (16)
#define GPIOFUNC_CTRL0_I2C_IOSEL_MSK             (0x3UL << GPIOFUNC_CTRL0_I2C_IOSEL_POS)
#define GPIOFUNC_CTRL0_I2C_IOA20_21              (0x2UL << GPIOFUNC_CTRL0_I2C_IOSEL_POS)
#define GPIOFUNC_CTRL0_I2C_IOA15_16              (0x1UL << GPIOFUNC_CTRL0_I2C_IOSEL_POS)
#define GPIOFUNC_CTRL0_I2C_IOA0_1                (0x0UL << GPIOFUNC_CTRL0_I2C_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[15] - SPI1 MOSI/MISO swap
 */
#define GPIOFUNC_CTRL0_SPI1_SWAP_POS               (15)
#define GPIOFUNC_CTRL0_SPI1_SWAP_MSK               (0x1UL << GPIOFUNC_CTRL0_SPI1_SWAP_POS)
#define GPIOFUNC_CTRL0_SPI1_SWAP_Keep              (0x0UL << GPIOFUNC_CTRL0_SPI1_SWAP_POS)
#define GPIOFUNC_CTRL0_SPI1_SWAP                   (0x1UL << GPIOFUNC_CTRL0_SPI1_SWAP_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[14:13] - SPI1 pins select
 */
#define GPIOFUNC_CTRL0_SPI1_IOSEL_POS            (13)
#define GPIOFUNC_CTRL0_SPI1_IOSEL_MSK            (0x3UL << GPIOFUNC_CTRL0_SPI1_IOSEL_POS)
#define GPIOFUNC_CTRL0_SPI1_IOA22_25_MOSI        (0x3UL << GPIOFUNC_CTRL0_SPI1_IOSEL_POS)
#define GPIOFUNC_CTRL0_SPI1_IOA9_12_MOSI         (0x2UL << GPIOFUNC_CTRL0_SPI1_IOSEL_POS)
#define GPIOFUNC_CTRL0_SPI1_IOA22_25             (0x1UL << GPIOFUNC_CTRL0_SPI1_IOSEL_POS)
#define GPIOFUNC_CTRL0_SPI1_IOA9_12              (0x0UL << GPIOFUNC_CTRL0_SPI1_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[12] - UART0 TX/RX swap
 */
#define GPIOFUNC_CTRL0_UART0_SWAP_POS              (12)
#define GPIOFUNC_CTRL0_UART0_SWAP_MSK              (0x1UL << GPIOFUNC_CTRL0_UART0_SWAP_POS)
#define GPIOFUNC_CTRL0_UART0_SWAP_Keep             (0x0UL << GPIOFUNC_CTRL0_UART0_SWAP_POS)
#define GPIOFUNC_CTRL0_UART0_SWAP                  (0x1UL << GPIOFUNC_CTRL0_UART0_SWAP_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[11: 10] - UART0 pins Select
 */
#define GPIOFUNC_CTRL0_UART0_IOSEL_POS             (10)
#define GPIOFUNC_CTRL0_UART0_IOSEL_MSK             (0x3UL << GPIOFUNC_CTRL0_UART0_IOSEL_POS)
#define GPIOFUNC_CTRL0_UART0_IOC2_RST              (0x3UL << GPIOFUNC_CTRL0_UART0_IOSEL_POS)
#define GPIOFUNC_CTRL0_UART0_IOA30_31              (0x2UL << GPIOFUNC_CTRL0_UART0_IOSEL_POS)
#define GPIOFUNC_CTRL0_UART0_IOA13_14              (0x1UL << GPIOFUNC_CTRL0_UART0_IOSEL_POS)
#define GPIOFUNC_CTRL0_UART0_IOA0_1                (0x0UL << GPIOFUNC_CTRL0_UART0_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[9] - CCP1 capture input pin select
 */
#define GPIOFUNC_CTRL0_CCP1_IOSEL_POS            (9)
#define GPIOFUNC_CTRL0_CCP1_IOSEL_MSK            (0x1UL << GPIOFUNC_CTRL0_CCP1_IOSEL_POS)
#define GPIOFUNC_CTRL0_CCP1_IOA9_12              (0x0UL << GPIOFUNC_CTRL0_CCP1_IOSEL_POS)
#define GPIOFUNC_CTRL0_CCP1_IOA26_29             (0x1UL << GPIOFUNC_CTRL0_CCP1_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[8] - IR Pins Select
 */
#define GPIOFUNC_CTRL0_IR_IOSEL_POS              (8)
#define GPIOFUNC_CTRL0_IR_IOSEL_MSK              (0x1UL << GPIOFUNC_CTRL0_IR_IOSEL_POS)
#define GPIOFUNC_CTRL0_IR_IOA17                  (0x1UL << GPIOFUNC_CTRL0_IR_IOSEL_POS)
#define GPIOFUNC_CTRL0_IR_IOA7                   (0x0UL << GPIOFUNC_CTRL0_IR_IOSEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[5] - SPI1 CS H/W control enable
 */
#define GPIOFUNC_CTRL0_SPI1_HWCS_EN_POS          (5)
#define GPIOFUNC_CTRL0_SPI1_HWCS_EN_MSK          (0x1UL << GPIOFUNC_CTRL0_SPI1_HWCS_EN_POS)
#define GPIOFUNC_CTRL0_SPI1_HWCS_ENABLE          (0x1UL << GPIOFUNC_CTRL0_SPI1_HWCS_EN_POS)
#define GPIOFUNC_CTRL0_SPI1_HWCS_DISABLE         (0x0UL << GPIOFUNC_CTRL0_SPI1_HWCS_EN_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[4] - SPI0 CS H/W control enable
 */
#define GPIOFUNC_CTRL0_SPI0_HWCS_EN_POS          (4)
#define GPIOFUNC_CTRL0_SPI0_HWCS_EN_MSK          (0x1UL << GPIOFUNC_CTRL0_SPI0_HWCS_EN_POS)
#define GPIOFUNC_CTRL0_SPI0_HWCS_ENABLE          (0x1UL << GPIOFUNC_CTRL0_SPI0_HWCS_EN_POS)
#define GPIOFUNC_CTRL0_SPI0_HWCS_DISABLE         (0x0UL << GPIOFUNC_CTRL0_SPI0_HWCS_EN_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[1] - IO feedback enable
 */
#define GPIOFUNC_CTRL0_FB_EN_POS                 (1)
#define GPIOFUNC_CTRL0_FB_EN_MSK                 (0x1UL << GPIOFUNC_CTRL0_FB_EN_POS)
#define GPIOFUNC_CTRL0_FB_ENABLE                 (0x1UL << GPIOFUNC_CTRL0_FB_EN_POS)
#define GPIOFUNC_CTRL0_FB_DISABLE                (0x0UL << GPIOFUNC_CTRL0_FB_EN_POS)

/*
 * Bit definition for GPIOFUNC_CTRL0[0] - ICE enable
 */
#define GPIOFUNC_CTRL0_ICE_EN_POS                (0)
#define GPIOFUNC_CTRL0_ICE_EN_MSK                (0x1UL << GPIOFUNC_CTRL0_ICE_EN_POS)
#define GPIOFUNC_CTRL0_ICE_ENABLE                (0x1UL << GPIOFUNC_CTRL0_ICE_EN_POS)
#define GPIOFUNC_CTRL0_ICE_DISABLE               (0x0UL << GPIOFUNC_CTRL0_ICE_EN_POS)

/*
 * Bit definition for GPIOFUNC_CTRL1[7] - IR TX Polarity control
 */
#define GPIOFUNC_CTRL1_IR_POL_POS                (7)
#define GPIOFUNC_CTRL1_IR_POL_MSK                (0x1UL << GPIOFUNC_CTRL1_IR_POL_POS)
#define GPIOFUNC_CTRL1_IR_POL_NEGATIVE           (0x1UL << GPIOFUNC_CTRL1_IR_POL_POS)
#define GPIOFUNC_CTRL1_IR_POL_POSITIVE           (0x0UL << GPIOFUNC_CTRL1_IR_POL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL1[6] - IR TX Mask
 */
#define GPIOFUNC_CTRL1_IR_MASK_POS                (6)
#define GPIOFUNC_CTRL1_IR_MASK_MSK                (0x1UL << GPIOFUNC_CTRL1_IR_MASK_POS)
#define GPIOFUNC_CTRL1_IR_MASK_ENABLE             (0x1UL << GPIOFUNC_CTRL1_IR_MASK_POS)
#define GPIOFUNC_CTRL1_IR_MASK_DISABLE            (0x0UL << GPIOFUNC_CTRL1_IR_MASK_POS)

/*
 * Bit definition for GPIOFUNC_CTRL1[5: 3] - IR TX CLK Source Select
 */
#define GPIOFUNC_CTRL1_IR_CLK_SEL_POS            (3)
#define GPIOFUNC_CTRL1_IR_CLK_SEL_MSK            (0x7UL << GPIOFUNC_CTRL1_IR_CLK_SEL_POS)
#define GPIOFUNC_CTRL1_IR_CLK_SEL_CTS1           (0x6UL << GPIOFUNC_CTRL1_IR_CLK_SEL_POS)
#define GPIOFUNC_CTRL1_IR_CLK_SEL_CTS0           (0x5UL << GPIOFUNC_CTRL1_IR_CLK_SEL_POS)
#define GPIOFUNC_CTRL1_IR_CLK_SEL_CCP1           (0x4UL << GPIOFUNC_CTRL1_IR_CLK_SEL_POS)
#define GPIOFUNC_CTRL1_IR_CLK_SEL_CCP0           (0x3UL << GPIOFUNC_CTRL1_IR_CLK_SEL_POS)
#define GPIOFUNC_CTRL1_IR_CLK_SEL_TIMER2         (0x2UL << GPIOFUNC_CTRL1_IR_CLK_SEL_POS)
#define GPIOFUNC_CTRL1_IR_CLK_SEL_TIMER1         (0x1UL << GPIOFUNC_CTRL1_IR_CLK_SEL_POS)
#define GPIOFUNC_CTRL1_IR_CLK_SEL_TIMER0         (0x0UL << GPIOFUNC_CTRL1_IR_CLK_SEL_POS)

/*
 * Bit definition for GPIOFUNC_CTRL1[2: 1] - IR TX Duty Select
 */
#define GPIOFUNC_CTRL1_IR_DUTY_POS               (1)
#define GPIOFUNC_CTRL1_IR_DUTY_MSK               (0x3UL << GPIOFUNC_CTRL1_IR_DUTY_POS)
#define GPIOFUNC_CTRL1_IR_DUTY_DIV_5             (0x3UL << GPIOFUNC_CTRL1_IR_DUTY_POS)
#define GPIOFUNC_CTRL1_IR_DUTY_DIV_4             (0x2UL << GPIOFUNC_CTRL1_IR_DUTY_POS)
#define GPIOFUNC_CTRL1_IR_DUTY_DIV_3             (0x1UL << GPIOFUNC_CTRL1_IR_DUTY_POS)
#define GPIOFUNC_CTRL1_IR_DUTY_DIV_2             (0x0UL << GPIOFUNC_CTRL1_IR_DUTY_POS)

/*
 * Bit definition for GPIOFUNC_CTRL1[0] - IR TX Enable
 */
#define GPIOFUNC_CTRL1_IR_TX_EN_POS              (0)
#define GPIOFUNC_CTRL1_IR_TX_EN_MSK              (0x1ul << GPIOFUNC_CTRL1_IR_TX_EN_POS)
#define GPIOFUNC_CTRL1_IR_TX_ENABLE              (0x1ul << GPIOFUNC_CTRL1_IR_TX_EN_POS)
#define GPIOFUNC_CTRL1_IR_TX_DISABLE             (0x0ul << GPIOFUNC_CTRL1_IR_TX_EN_POS)

/*
 * Bit definition for GPIOFUNC_CTRL2[31:29] -  For EXTINT3/ CCP EXTCLK3 Key selection
 */
#define GPIOFUNC_CTRL2_EXT3_PINSEL2_POS          (29)
#define GPIOFUNC_CTRL2_EXT3_PINSEL2_MSK          (0x7UL << GPIOFUNC_CTRL2_EXT3_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT3_IOB_DISABLE          (0x7UL << GPIOFUNC_CTRL2_EXT3_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT3_IOB5                 (0x5UL << GPIOFUNC_CTRL2_EXT3_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT3_IOB4                 (0x4UL << GPIOFUNC_CTRL2_EXT3_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT3_IOB3                 (0x3UL << GPIOFUNC_CTRL2_EXT3_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT3_IOB2                 (0x2UL << GPIOFUNC_CTRL2_EXT3_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT3_IOB1                 (0x1UL << GPIOFUNC_CTRL2_EXT3_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT3_IOB0                 (0x0UL << GPIOFUNC_CTRL2_EXT3_PINSEL2_POS)

/*
 * Bit definition for GPIOFUNC_CTRL2[28:24] -  For EXTINT3/ CCP EXTCLK3 Key selection
 */
#define GPIOFUNC_CTRL2_EXT3_PINSEL1_POS          (24)
#define GPIOFUNC_CTRL2_EXT3_PINSEL1_MSK          (0x1FUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA_DISABLE          (0x1FUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA30                (0x1EUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA29                (0x1DUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA28                (0x1CUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA27                (0x1BUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA26                (0x1AUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA25                (0x19UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA24                (0x18UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA23                (0x17UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA22                (0x16UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA21                (0x15UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA20                (0x14UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA19                (0x13UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA18                (0x12UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA17                (0x11UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA16                (0x10UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA15                (0x0FUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA14                (0x0EUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA13                (0x0DUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA12                (0x0CUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA11                (0x0BUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA10                (0x0AUL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA9                 (0x09UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA8                 (0x08UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA7                 (0x07UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA6                 (0x06UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA5                 (0x05UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA4                 (0x04UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA3                 (0x03UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA2                 (0x02UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA1                 (0x01UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT3_IOA0                 (0x00UL << GPIOFUNC_CTRL2_EXT3_PINSEL1_POS)

/*
 * Bit definition for GPIOFUNC_CTRL2[23:21] -  For EXTINT2/Timer2 ExtCLK/CCP EXTCLK2 Key selection
 */
#define GPIOFUNC_CTRL2_EXT2_PINSEL2_POS          (21)
#define GPIOFUNC_CTRL2_EXT2_PINSEL2_MSK          (0x7UL << GPIOFUNC_CTRL2_EXT2_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT2_IOB_DISABLE          (0x7UL << GPIOFUNC_CTRL2_EXT2_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT2_IOB5                 (0x5UL << GPIOFUNC_CTRL2_EXT2_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT2_IOB4                 (0x4UL << GPIOFUNC_CTRL2_EXT2_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT2_IOB3                 (0x3UL << GPIOFUNC_CTRL2_EXT2_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT2_IOB2                 (0x2UL << GPIOFUNC_CTRL2_EXT2_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT2_IOB1                 (0x1UL << GPIOFUNC_CTRL2_EXT2_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT2_IOB0                 (0x0UL << GPIOFUNC_CTRL2_EXT2_PINSEL2_POS)

/*
 * Bit definition for GPIOFUNC_CTRL2[20:16] -  For EXTINT2/Timer2 ExtCLK/CCP EXTCLK2 Key selection
 */
#define GPIOFUNC_CTRL2_EXT2_PINSEL1_POS          (16)
#define GPIOFUNC_CTRL2_EXT2_PINSEL1_MSK          (0x1FUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA_DISABLE          (0x1FUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA30                (0x1EUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA29                (0x1DUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA28                (0x1CUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA27                (0x1BUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA26                (0x1AUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA25                (0x19UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA24                (0x18UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA23                (0x17UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA22                (0x16UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA21                (0x15UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA20                (0x14UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA19                (0x13UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA18                (0x12UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA17                (0x11UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA16                (0x10UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA15                (0x0FUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA14                (0x0EUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA13                (0x0DUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA12                (0x0CUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA11                (0x0BUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA10                (0x0AUL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA9                 (0x09UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA8                 (0x08UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA7                 (0x07UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA6                 (0x06UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA5                 (0x05UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA4                 (0x04UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA3                 (0x03UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA2                 (0x02UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA1                 (0x01UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT2_IOA0                 (0x00UL << GPIOFUNC_CTRL2_EXT2_PINSEL1_POS)

/*
 * Bit definition for GPIOFUNC_CTRL2[15:13] -  For EXTINT1/Timer1 ExtCLK/CCP EXTCLK1 Key selection
 */
#define GPIOFUNC_CTRL2_EXT1_PINSEL2_POS          (13)
#define GPIOFUNC_CTRL2_EXT1_PINSEL2_MSK          (0x7UL << GPIOFUNC_CTRL2_EXT1_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT1_IOB_DISABLE          (0x7UL << GPIOFUNC_CTRL2_EXT1_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT1_IOB5                 (0x5UL << GPIOFUNC_CTRL2_EXT1_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT1_IOB4                 (0x4UL << GPIOFUNC_CTRL2_EXT1_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT1_IOB3                 (0x3UL << GPIOFUNC_CTRL2_EXT1_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT1_IOB2                 (0x2UL << GPIOFUNC_CTRL2_EXT1_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT1_IOB1                 (0x1UL << GPIOFUNC_CTRL2_EXT1_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT1_IOB0                 (0x0UL << GPIOFUNC_CTRL2_EXT1_PINSEL2_POS)

/*
 * Bit definition for GPIOFUNC_CTRL2[12:8] -  For EXTINT1/Timer1 ExtCLK/CCP EXTCLK1 Key selection
 */
#define GPIOFUNC_CTRL2_EXT1_PINSEL1_POS          (8)
#define GPIOFUNC_CTRL2_EXT1_PINSEL1_MSK          (0x1FUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA30                (0x1EUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA29                (0x1DUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA28                (0x1CUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA27                (0x1BUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA26                (0x1AUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA25                (0x19UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA24                (0x18UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA23                (0x17UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA22                (0x16UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA21                (0x15UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA20                (0x14UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA19                (0x13UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA18                (0x12UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA17                (0x11UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA16                (0x10UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA15                (0x0FUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA14                (0x0EUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA13                (0x0DUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA12                (0x0CUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA11                (0x0BUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA10                (0x0AUL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA9                 (0x09UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA8                 (0x08UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA7                 (0x07UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA6                 (0x06UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA5                 (0x05UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA4                 (0x04UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA3                 (0x03UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA2                 (0x02UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA1                 (0x01UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT1_IOA0                 (0x00UL << GPIOFUNC_CTRL2_EXT1_PINSEL1_POS)

/*
 * Bit definition for GPIOFUNC_CTRL2[7:5] -  For EXTINT0/Timer0 ExtCLK/CCP0 EXTCLK Key selection
 */
#define GPIOFUNC_CTRL2_EXT0_PINSEL2_POS          (5)
#define GPIOFUNC_CTRL2_EXT0_PINSEL2_MSK          (0x7UL << GPIOFUNC_CTRL2_EXT0_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT0_IOB_DISABLE          (0x7UL << GPIOFUNC_CTRL2_EXT0_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT0_IOB5                 (0x5UL << GPIOFUNC_CTRL2_EXT0_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT0_IOB4                 (0x4UL << GPIOFUNC_CTRL2_EXT0_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT0_IOB3                 (0x3UL << GPIOFUNC_CTRL2_EXT0_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT0_IOB2                 (0x2UL << GPIOFUNC_CTRL2_EXT0_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT0_IOB1                 (0x1UL << GPIOFUNC_CTRL2_EXT0_PINSEL2_POS)
#define GPIOFUNC_CTRL2_EXT0_IOB0                 (0x0UL << GPIOFUNC_CTRL2_EXT0_PINSEL2_POS)

/*
 * Bit definition for GPIOFUNC_CTRL2[4:0] -  For EXTINT0/Timer0 ExtCLK/CCP0 EXTCLK Key selection
 */
#define GPIOFUNC_CTRL2_EXT0_PINSEL1_POS          (0)
#define GPIOFUNC_CTRL2_EXT0_PINSEL1_MSK          (0x1FUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA30                (0x1EUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA29                (0x1DUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA28                (0x1CUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA27                (0x1BUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA26                (0x1AUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA25                (0x19UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA24                (0x18UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA23                (0x17UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA22                (0x16UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA21                (0x15UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA20                (0x14UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA19                (0x13UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA18                (0x12UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA17                (0x11UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA16                (0x10UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA15                (0x0FUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA14                (0x0EUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA13                (0x0DUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA12                (0x0CUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA11                (0x0BUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA10                (0x0AUL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA9                 (0x09UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA8                 (0x08UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA7                 (0x07UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA6                 (0x06UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA5                 (0x05UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA4                 (0x04UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA3                 (0x03UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA2                 (0x02UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA1                 (0x01UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)
#define GPIOFUNC_CTRL2_EXT0_IOA0                 (0x00UL << GPIOFUNC_CTRL2_EXT0_PINSEL1_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[31] - IOA31 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA31_EN_POS             (31)
#define GPIOFUNC_WAKEEN_IOA31_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA31_EN_POS)
#define GPIOFUNC_WAKEEN_IOA31_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA31_EN_POS)
#define GPIOFUNC_WAKEEN_IOA31_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA31_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[30] - IOA30 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA30_EN_POS             (30)
#define GPIOFUNC_WAKEEN_IOA30_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA30_EN_POS)
#define GPIOFUNC_WAKEEN_IOA30_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA30_EN_POS)
#define GPIOFUNC_WAKEEN_IOA30_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA30_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[29] - IOA29 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA29_EN_POS             (29)
#define GPIOFUNC_WAKEEN_IOA29_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA29_EN_POS)
#define GPIOFUNC_WAKEEN_IOA29_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA29_EN_POS)
#define GPIOFUNC_WAKEEN_IOA29_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA29_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[28] - IOA28 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA28_EN_POS             (28)
#define GPIOFUNC_WAKEEN_IOA28_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA28_EN_POS)
#define GPIOFUNC_WAKEEN_IOA28_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA28_EN_POS)
#define GPIOFUNC_WAKEEN_IOA28_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA28_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[27] - IOA27 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA27_EN_POS             (27)
#define GPIOFUNC_WAKEEN_IOA27_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA27_EN_POS)
#define GPIOFUNC_WAKEEN_IOA27_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA27_EN_POS)
#define GPIOFUNC_WAKEEN_IOA27_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA27_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[26] - IOA26 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA26_EN_POS             (26)
#define GPIOFUNC_WAKEEN_IOA26_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA26_EN_POS)
#define GPIOFUNC_WAKEEN_IOA26_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA26_EN_POS)
#define GPIOFUNC_WAKEEN_IOA26_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA26_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[25] - IOA25 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA25_EN_POS             (25)
#define GPIOFUNC_WAKEEN_IOA25_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA25_EN_POS)
#define GPIOFUNC_WAKEEN_IOA25_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA25_EN_POS)
#define GPIOFUNC_WAKEEN_IOA25_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA25_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[24] - IOA24 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA24_EN_POS             (24)
#define GPIOFUNC_WAKEEN_IOA24_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA24_EN_POS)
#define GPIOFUNC_WAKEEN_IOA24_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA24_EN_POS)
#define GPIOFUNC_WAKEEN_IOA24_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA24_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[23] - IOA23 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA23_EN_POS             (23)
#define GPIOFUNC_WAKEEN_IOA23_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA23_EN_POS)
#define GPIOFUNC_WAKEEN_IOA23_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA23_EN_POS)
#define GPIOFUNC_WAKEEN_IOA23_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA23_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[22] - IOA22 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA22_EN_POS             (22)
#define GPIOFUNC_WAKEEN_IOA22_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA22_EN_POS)
#define GPIOFUNC_WAKEEN_IOA22_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA22_EN_POS)
#define GPIOFUNC_WAKEEN_IOA22_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA22_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[21] - IOA21 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA21_EN_POS             (21)
#define GPIOFUNC_WAKEEN_IOA21_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA21_EN_POS)
#define GPIOFUNC_WAKEEN_IOA21_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA21_EN_POS)
#define GPIOFUNC_WAKEEN_IOA21_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA21_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[20] - IOA20 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA20_EN_POS             (20)
#define GPIOFUNC_WAKEEN_IOA20_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA20_EN_POS)
#define GPIOFUNC_WAKEEN_IOA20_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA20_EN_POS)
#define GPIOFUNC_WAKEEN_IOA20_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA20_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[19] - IOA19 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA19_EN_POS             (19)
#define GPIOFUNC_WAKEEN_IOA19_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA19_EN_POS)
#define GPIOFUNC_WAKEEN_IOA19_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA19_EN_POS)
#define GPIOFUNC_WAKEEN_IOA19_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA19_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[18] - IOA18 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA18_EN_POS             (18)
#define GPIOFUNC_WAKEEN_IOA18_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA18_EN_POS)
#define GPIOFUNC_WAKEEN_IOA18_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA18_EN_POS)
#define GPIOFUNC_WAKEEN_IOA18_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA18_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[17] - IOA17 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA17_EN_POS             (17)
#define GPIOFUNC_WAKEEN_IOA17_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA17_EN_POS)
#define GPIOFUNC_WAKEEN_IOA17_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA17_EN_POS)
#define GPIOFUNC_WAKEEN_IOA17_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA17_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[16] - IOA16 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA16_EN_POS             (16)
#define GPIOFUNC_WAKEEN_IOA16_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA16_EN_POS)
#define GPIOFUNC_WAKEEN_IOA16_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA16_EN_POS)
#define GPIOFUNC_WAKEEN_IOA16_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA16_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[15] - IOA15 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA15_EN_POS             (15)
#define GPIOFUNC_WAKEEN_IOA15_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA15_EN_POS)
#define GPIOFUNC_WAKEEN_IOA15_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA15_EN_POS)
#define GPIOFUNC_WAKEEN_IOA15_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA15_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[14] - IOA14 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA14_EN_POS             (14)
#define GPIOFUNC_WAKEEN_IOA14_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA14_EN_POS)
#define GPIOFUNC_WAKEEN_IOA14_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA14_EN_POS)
#define GPIOFUNC_WAKEEN_IOA14_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA14_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[13] - IOA13 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA13_EN_POS             (13)
#define GPIOFUNC_WAKEEN_IOA13_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA13_EN_POS)
#define GPIOFUNC_WAKEEN_IOA13_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA13_EN_POS)
#define GPIOFUNC_WAKEEN_IOA13_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA13_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[12] - IOA12 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA12_EN_POS             (12)
#define GPIOFUNC_WAKEEN_IOA12_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA12_EN_POS)
#define GPIOFUNC_WAKEEN_IOA12_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA12_EN_POS)
#define GPIOFUNC_WAKEEN_IOA12_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA12_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[11] - IOA11 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA11_EN_POS             (11)
#define GPIOFUNC_WAKEEN_IOA11_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA11_EN_POS)
#define GPIOFUNC_WAKEEN_IOA11_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA11_EN_POS)
#define GPIOFUNC_WAKEEN_IOA11_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA11_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[10] - IOA10 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA10_EN_POS             (10)
#define GPIOFUNC_WAKEEN_IOA10_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN_IOA10_EN_POS)
#define GPIOFUNC_WAKEEN_IOA10_ENABLE             (0x1UL << GPIOFUNC_WAKEEN_IOA10_EN_POS)
#define GPIOFUNC_WAKEEN_IOA10_DISABLE            (0x0UL << GPIOFUNC_WAKEEN_IOA10_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[9] - IOA9 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA9_EN_POS              (9)
#define GPIOFUNC_WAKEEN_IOA9_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA9_EN_POS)
#define GPIOFUNC_WAKEEN_IOA9_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA9_EN_POS)
#define GPIOFUNC_WAKEEN_IOA9_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA9_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[8] - IOA8 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA8_EN_POS              (8)
#define GPIOFUNC_WAKEEN_IOA8_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA8_EN_POS)
#define GPIOFUNC_WAKEEN_IOA8_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA8_EN_POS)
#define GPIOFUNC_WAKEEN_IOA8_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA8_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[7] - IOA7 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA7_EN_POS              (7)
#define GPIOFUNC_WAKEEN_IOA7_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA7_EN_POS)
#define GPIOFUNC_WAKEEN_IOA7_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA7_EN_POS)
#define GPIOFUNC_WAKEEN_IOA7_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA7_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[6] - IOA6 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA6_EN_POS              (6)
#define GPIOFUNC_WAKEEN_IOA6_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA6_EN_POS)
#define GPIOFUNC_WAKEEN_IOA6_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA6_EN_POS)
#define GPIOFUNC_WAKEEN_IOA6_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA6_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[5] - IOA5 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA5_EN_POS              (5)
#define GPIOFUNC_WAKEEN_IOA5_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA5_EN_POS)
#define GPIOFUNC_WAKEEN_IOA5_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA5_EN_POS)
#define GPIOFUNC_WAKEEN_IOA5_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA5_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[4] - IOA4 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA4_EN_POS              (4)
#define GPIOFUNC_WAKEEN_IOA4_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA4_EN_POS)
#define GPIOFUNC_WAKEEN_IOA4_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA4_EN_POS)
#define GPIOFUNC_WAKEEN_IOA4_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA4_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[3] - IOA3 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA3_EN_POS              (3)
#define GPIOFUNC_WAKEEN_IOA3_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA3_EN_POS)
#define GPIOFUNC_WAKEEN_IOA3_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA3_EN_POS)
#define GPIOFUNC_WAKEEN_IOA3_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA3_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[2] - IOA2 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA2_EN_POS              (2)
#define GPIOFUNC_WAKEEN_IOA2_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA2_EN_POS)
#define GPIOFUNC_WAKEEN_IOA2_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA2_EN_POS)
#define GPIOFUNC_WAKEEN_IOA2_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA2_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[1] - IOA1 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA1_EN_POS              (1)
#define GPIOFUNC_WAKEEN_IOA1_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA1_EN_POS)
#define GPIOFUNC_WAKEEN_IOA1_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA1_EN_POS)
#define GPIOFUNC_WAKEEN_IOA1_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA1_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN[0] - IOA0 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN_IOA0_EN_POS              (0)
#define GPIOFUNC_WAKEEN_IOA0_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN_IOA0_EN_POS)
#define GPIOFUNC_WAKEEN_IOA0_ENABLE              (0x1UL << GPIOFUNC_WAKEEN_IOA0_EN_POS)
#define GPIOFUNC_WAKEEN_IOA0_DISABLE             (0x0UL << GPIOFUNC_WAKEEN_IOA0_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[31] - GPIO key change interrupt enable
 */
#define GPIOFUNC_WAKEEN2_INT_EN_POS              (31)
#define GPIOFUNC_WAKEEN2_INT_EN_MSK              (0x1UL << GPIOFUNC_WAKEEN2_INT_EN_POS)
#define GPIOFUNC_WAKEEN2_INT_ENABLE              (0x1UL << GPIOFUNC_WAKEEN2_INT_EN_POS)
#define GPIOFUNC_WAKEEN2_INT_DISABLE             (0x0UL << GPIOFUNC_WAKEEN2_INT_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[10] - IOD1 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOD1_EN_POS             (10)
#define GPIOFUNC_WAKEEN2_IOD1_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOD1_EN_POS)
#define GPIOFUNC_WAKEEN2_IOD1_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOD1_EN_POS)
#define GPIOFUNC_WAKEEN2_IOD1_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOD1_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[9] - IOD0 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOD0_EN_POS             (9)
#define GPIOFUNC_WAKEEN2_IOD0_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOD0_EN_POS)
#define GPIOFUNC_WAKEEN2_IOD0_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOD0_EN_POS)
#define GPIOFUNC_WAKEEN2_IOD0_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOD0_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[8] - IOC2 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOC2_EN_POS             (8)
#define GPIOFUNC_WAKEEN2_IOC2_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOC2_EN_POS)
#define GPIOFUNC_WAKEEN2_IOC2_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOC2_EN_POS)
#define GPIOFUNC_WAKEEN2_IOC2_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOC2_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[7] - IOC1 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOC1_EN_POS             (7)
#define GPIOFUNC_WAKEEN2_IOC1_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOC1_EN_POS)
#define GPIOFUNC_WAKEEN2_IOC1_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOC1_EN_POS)
#define GPIOFUNC_WAKEEN2_IOC1_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOC1_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[6] - IOC0 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOC0_EN_POS             (6)
#define GPIOFUNC_WAKEEN2_IOC0_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOC0_EN_POS)
#define GPIOFUNC_WAKEEN2_IOC0_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOC0_EN_POS)
#define GPIOFUNC_WAKEEN2_IOC0_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOC0_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[5] - IOB5 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOB5_EN_POS             (5)
#define GPIOFUNC_WAKEEN2_IOB5_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOB5_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB5_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOB5_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB5_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOB5_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[4] - IOB4 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOB4_EN_POS             (4)
#define GPIOFUNC_WAKEEN2_IOB4_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOB4_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB4_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOB4_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB4_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOB4_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[3] - IOB3 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOB3_EN_POS             (3)
#define GPIOFUNC_WAKEEN2_IOB3_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOB3_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB3_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOB3_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB3_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOB3_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[2] - IOB2 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOB2_EN_POS             (2)
#define GPIOFUNC_WAKEEN2_IOB2_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOB2_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB2_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOB2_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB2_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOB2_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[1] - IOB1 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOB1_EN_POS             (1)
#define GPIOFUNC_WAKEEN2_IOB1_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOB1_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB1_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOB1_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB1_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOB1_EN_POS)

/*
 * Bit definition for GPIOFUNC_WAKEEN2[0] - IOB0 key change wakeup function enable
 */
#define GPIOFUNC_WAKEEN2_IOB0_EN_POS             (0)
#define GPIOFUNC_WAKEEN2_IOB0_EN_MSK             (0x1UL << GPIOFUNC_WAKEEN2_IOB0_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB0_ENABLE             (0x1UL << GPIOFUNC_WAKEEN2_IOB0_EN_POS)
#define GPIOFUNC_WAKEEN2_IOB0_DISABLE            (0x0UL << GPIOFUNC_WAKEEN2_IOB0_EN_POS)

/*
 * Bit definition for GPIOFUNC_STS[31] - GPIO key change interrupt flag
 */
#define GPIOFUNC_STS_INTF_POS                    (31)
#define GPIOFUNC_STS_INTF_MSK                    (0x1UL << GPIOFUNC_STS_INTF_POS)
#define GPIOFUNC_STS_INTF_FLAG                   (0x1UL << GPIOFUNC_STS_INTF_POS)

/*
 * Bit definition for GPIOFUNC_SPIFC[30:28] - SPIFC IOB5 select
 */
#define GPIOFUNC_SPIFC_IOB5_SEL_POS              (28)
#define GPIOFUNC_SPIFC_IOB5_SEL_MSK              (0x7UL << GPIOFUNC_SPIFC_IOB5_SEL_POS)
#define GPIOFUNC_SPIFC_IOB5_SEL_WP               (0x0UL << GPIOFUNC_SPIFC_IOB5_SEL_POS)
#define GPIOFUNC_SPIFC_IOB5_SEL_HOLD             (0x1UL << GPIOFUNC_SPIFC_IOB5_SEL_POS)
#define GPIOFUNC_SPIFC_IOB5_SEL_MOSI             (0x2UL << GPIOFUNC_SPIFC_IOB5_SEL_POS)
#define GPIOFUNC_SPIFC_IOB5_SEL_CS               (0x3UL << GPIOFUNC_SPIFC_IOB5_SEL_POS)
#define GPIOFUNC_SPIFC_IOB5_SEL_MISO             (0x4UL << GPIOFUNC_SPIFC_IOB5_SEL_POS)

/*
 * Bit definition for GPIOFUNC_SPIFC[26:24] - SPIFC IOB4 select
 */
#define GPIOFUNC_SPIFC_IOB4_SEL_POS              (24)
#define GPIOFUNC_SPIFC_IOB4_SEL_MSK              (0x7UL << GPIOFUNC_SPIFC_IOB4_SEL_POS)
#define GPIOFUNC_SPIFC_IOB4_SEL_MISO             (0x0UL << GPIOFUNC_SPIFC_IOB4_SEL_POS)
#define GPIOFUNC_SPIFC_IOB4_SEL_HOLD             (0x1UL << GPIOFUNC_SPIFC_IOB4_SEL_POS)
#define GPIOFUNC_SPIFC_IOB4_SEL_MOSI             (0x2UL << GPIOFUNC_SPIFC_IOB4_SEL_POS)
#define GPIOFUNC_SPIFC_IOB4_SEL_CS               (0x3UL << GPIOFUNC_SPIFC_IOB4_SEL_POS)
#define GPIOFUNC_SPIFC_IOB4_SEL_WP               (0x5UL << GPIOFUNC_SPIFC_IOB4_SEL_POS)

/*
 * Bit definition for GPIOFUNC_SPIFC[22:20] - SPIFC IOB3 select
 */
#define GPIOFUNC_SPIFC_IOB3_SEL_POS              (20)
#define GPIOFUNC_SPIFC_IOB3_SEL_MSK              (0x7UL << GPIOFUNC_SPIFC_IOB3_SEL_POS)
#define GPIOFUNC_SPIFC_IOB3_SEL_CS               (0x0UL << GPIOFUNC_SPIFC_IOB3_SEL_POS)
#define GPIOFUNC_SPIFC_IOB3_SEL_HOLD             (0x1UL << GPIOFUNC_SPIFC_IOB3_SEL_POS)
#define GPIOFUNC_SPIFC_IOB3_SEL_MOSI             (0x2UL << GPIOFUNC_SPIFC_IOB3_SEL_POS)
#define GPIOFUNC_SPIFC_IOB3_SEL_MISO             (0x4UL << GPIOFUNC_SPIFC_IOB3_SEL_POS)
#define GPIOFUNC_SPIFC_IOB3_SEL_WP               (0x5UL << GPIOFUNC_SPIFC_IOB3_SEL_POS)

/*
 * Bit definition for GPIOFUNC_SPIFC[18:16] - SPIFC IOB2 select
 */
#define GPIOFUNC_SPIFC_IOB2_SEL_POS              (16)
#define GPIOFUNC_SPIFC_IOB2_SEL_MSK              (0x7UL << GPIOFUNC_SPIFC_IOB2_SEL_POS)
#define GPIOFUNC_SPIFC_IOB2_SEL_MOSI             (0x0UL << GPIOFUNC_SPIFC_IOB2_SEL_POS)
#define GPIOFUNC_SPIFC_IOB2_SEL_HOLD             (0x1UL << GPIOFUNC_SPIFC_IOB2_SEL_POS)
#define GPIOFUNC_SPIFC_IOB2_SEL_CS               (0x3UL << GPIOFUNC_SPIFC_IOB2_SEL_POS)
#define GPIOFUNC_SPIFC_IOB2_SEL_MISO             (0x4UL << GPIOFUNC_SPIFC_IOB2_SEL_POS)
#define GPIOFUNC_SPIFC_IOB2_SEL_WP               (0x5UL << GPIOFUNC_SPIFC_IOB2_SEL_POS)

/*
 * Bit definition for GPIOFUNC_SPIFC[14:12] - SPIFC IOB0 select
 */
#define GPIOFUNC_SPIFC_IOB0_SEL_POS              (12)
#define GPIOFUNC_SPIFC_IOB0_SEL_MSK              (0x7UL << GPIOFUNC_SPIFC_IOB0_SEL_POS)
#define GPIOFUNC_SPIFC_IOB0_SEL_HOLD             (0x0UL << GPIOFUNC_SPIFC_IOB0_SEL_POS)
#define GPIOFUNC_SPIFC_IOB0_SEL_MOSI             (0x2UL << GPIOFUNC_SPIFC_IOB0_SEL_POS)
#define GPIOFUNC_SPIFC_IOB0_SEL_CS               (0x3UL << GPIOFUNC_SPIFC_IOB0_SEL_POS)
#define GPIOFUNC_SPIFC_IOB0_SEL_MISO             (0x4UL << GPIOFUNC_SPIFC_IOB0_SEL_POS)
#define GPIOFUNC_SPIFC_IOB0_SEL_WP               (0x5UL << GPIOFUNC_SPIFC_IOB0_SEL_POS)

/*---------------------------------------------------------------------------------------
 * DMA Control Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t INTSTS;                        // DMA Interrupt Status Register
} DMA_INT_TYPE_DEF;

typedef struct
{
    __IO uint32_t CTRL;                          // DMA Control Register
    __IO uint32_t DTN;                           // DMA Data Transfer Number Register
    __IO uint32_t SRCADR;                        // DMA Source Address Register
    __IO uint32_t DSTADR;                        // DMA Destination Address Register
} DMA_TYPE_DEF;

/*
 * Bit definition for DMAx_CTRL[25] - DMA Double Buffer Full Flag
 */
#define DMA_CTRL_DMA_FULL_FLAG_POS               (25)
#define DMA_CTRL_DMA_FULL_FLAG_MSK               (0x1UL << DMA_CTRL_DMA_FULL_FLAG_POS)
#define DMA_CTRL_DMA_FULL_FLAG                   (0x1UL << DMA_CTRL_DMA_FULL_FLAG_POS)

/*
 * Bit definition for DMAx_CTRL[24] - DMA operation status
 */
#define DMA_CTRL_DMA_BUSY_POS                    (24)
#define DMA_CTRL_DMA_BUSY_MSK                    (0x1UL << DMA_CTRL_DMA_BUSY_POS)
#define DMA_CTRL_DMA_BUSY_FLAG                   (0x1UL << DMA_CTRL_DMA_BUSY_POS)

/*
 * Bit definition for DMAx_CTRL[19] - DMA dual trigger mode enable
 */
#define DMA_CTRL_DMA_DUAL_EN_POS                 (19)
#define DMA_CTRL_DMA_DUAL_EN_MSK                 (0x1UL << DMA_CTRL_DMA_DUAL_EN_POS)
#define DMA_CTRL_DMA_DUAL_ENABLE                 (0x1UL << DMA_CTRL_DMA_DUAL_EN_POS)
#define DMA_CTRL_DMA_DUAL_DISABLE                (0x0UL << DMA_CTRL_DMA_DUAL_EN_POS)

/*
 * Bit definition for DMAx_CTRL[18] - DMA software stop timing control bit
 */
#define DMA_CTRL_DMA_STPS_POS                    (18)
#define DMA_CTRL_DMA_STPS_MSK                    (0x1UL << DMA_CTRL_DMA_STPS_POS)
#define DMA_CTRL_DMA_STPS_WAITCOMP               (0x1UL << DMA_CTRL_DMA_STPS_POS)
#define DMA_CTRL_DMA_STPS_IMMDLY                 (0x0UL << DMA_CTRL_DMA_STPS_POS)

/*
 * Bit definition for DMAx_CTRL[17] - DMA software stop bit
 */
#define DMA_CTRL_DMA_STOP_POS                    (17)
#define DMA_CTRL_DMA_STOP_MSK                    (0x1UL << DMA_CTRL_DMA_STOP_POS)
#define DMA_CTRL_DMA_STOP                        (0x1UL << DMA_CTRL_DMA_STOP_POS)

/*
 * Bit definition for DMAx_CTRL[16] - DMA software start bit
 */
#define DMA_CTRL_DMA_STR_POS                     (16)
#define DMA_CTRL_DMA_STR_MSK                     (0x1UL << DMA_CTRL_DMA_STR_POS)
#define DMA_CTRL_DMA_START                       (0x1UL << DMA_CTRL_DMA_STR_POS)

/*
 * Bit definition for DMAx_CTRL[15:12] - DMA peripheral REQ selection
 */
#define DMA_CTRL_DMA_REQSEL_POS                  (12)
#define DMA_CTRL_DMA_REQSEL_MSK                  (0xFUL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_UART1                (0xDUL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_CCP                  (0xCUL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_MEM                  (0xBUL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_SPUR                 (0xAUL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_SPUL                 (0x9UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_SPI1                 (0x8UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_DSADC                (0x7UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_I2C                  (0x6UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_I2S                  (0x5UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_UART0                (0x4UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_SPI0                 (0x3UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_DAC_CH1              (0x2UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_DAC_CH0              (0x1UL << DMA_CTRL_DMA_REQSEL_POS)
#define DMA_CTRL_DMA_REQSEL_SARADC               (0x0UL << DMA_CTRL_DMA_REQSEL_POS)

/*
 * Bit definition for DMAx_CTRL[11:10] - Source data size
 */
#define DMA_CTRL_DMA_SRCSIZE_POS                 (10)
#define DMA_CTRL_DMA_SRCSIZE_MSK                 (0x3UL << DMA_CTRL_DMA_SRCSIZE_POS)
#define DMA_CTRL_DMA_SRCSIZE_RSV                 (0x3UL << DMA_CTRL_DMA_SRCSIZE_POS)
#define DMA_CTRL_DMA_SRCSIZE_32B                 (0x2UL << DMA_CTRL_DMA_SRCSIZE_POS)
#define DMA_CTRL_DMA_SRCSIZE_16B                 (0x1UL << DMA_CTRL_DMA_SRCSIZE_POS)
#define DMA_CTRL_DMA_SRCSIZE_8B                  (0x0UL << DMA_CTRL_DMA_SRCSIZE_POS)

/*
 * Bit definition for DMAx_CTRL[9:8] - Destination data size
 */
#define DMA_CTRL_DMA_DSTSIZE_POS                 (8)
#define DMA_CTRL_DMA_DSTSIZE_MSK                 (0x3UL << DMA_CTRL_DMA_DSTSIZE_POS)
#define DMA_CTRL_DMA_DSTSIZE_RSV                 (0x3UL << DMA_CTRL_DMA_DSTSIZE_POS)
#define DMA_CTRL_DMA_DSTSIZE_32B                 (0x2UL << DMA_CTRL_DMA_DSTSIZE_POS)
#define DMA_CTRL_DMA_DSTSIZE_16B                 (0x1UL << DMA_CTRL_DMA_DSTSIZE_POS)
#define DMA_CTRL_DMA_DSTSIZE_8B                  (0x0UL << DMA_CTRL_DMA_DSTSIZE_POS)

/*
 * Bit definition for DMAx_CTRL[7] - DMA continue mode enable
 */
#define DMA_CTRL_DMA_CONTINUE_EN_POS             (7)
#define DMA_CTRL_DMA_CONTINUE_EN_MSK             (0x1UL << DMA_CTRL_DMA_CONTINUE_EN_POS)
#define DMA_CTRL_DMA_CONTINUE_ENABLE             (0x1UL << DMA_CTRL_DMA_CONTINUE_EN_POS)
#define DMA_CTRL_DMA_CONTINUE_DISABLE            (0x0UL << DMA_CTRL_DMA_CONTINUE_EN_POS)

/*
 * Bit definition for DMAx_CTRL[6] - DMA burst transfer mode enable
 */
#define DMA_CTRL_DMA_BURST_EN_POS                (6)
#define DMA_CTRL_DMA_BURST_EN_MSK                (0x1UL << DMA_CTRL_DMA_BURST_EN_POS)
#define DMA_CTRL_DMA_BURST_ENABLE                (0x1UL << DMA_CTRL_DMA_BURST_EN_POS)
#define DMA_CTRL_DMA_BURST_DISABLE               (0x0UL << DMA_CTRL_DMA_BURST_EN_POS)

/*
 * Bit definition for DMAx_CTRL[5] - DMA source address increment mode enable
 */
#define DMA_CTRL_DMA_SRCINC_EN_POS               (5)
#define DMA_CTRL_DMA_SRCINC_EN_MSK               (0x1UL << DMA_CTRL_DMA_SRCINC_EN_POS)
#define DMA_CTRL_DMA_SRCINC_ENABLE               (0x1UL << DMA_CTRL_DMA_SRCINC_EN_POS)
#define DMA_CTRL_DMA_SRCINC_DISABLE              (0x0UL << DMA_CTRL_DMA_SRCINC_EN_POS)

/*
 * Bit definition for DMAx_CTRL[4] - DMA destination address increment mode enable
 */
#define DMA_CTRL_DMA_DSTINC_EN_POS               (4)
#define DMA_CTRL_DMA_DSTINC_EN_MSK               (0x1UL << DMA_CTRL_DMA_DSTINC_EN_POS)
#define DMA_CTRL_DMA_DSTINC_ENABLE               (0x1UL << DMA_CTRL_DMA_DSTINC_EN_POS)
#define DMA_CTRL_DMA_DSTINC_DISABLE              (0x0UL << DMA_CTRL_DMA_DSTINC_EN_POS)

/*
 * Bit definition for DMA_CTRL[3] - DMA circular mode enable
 */
#define DMA_CTRL_DMA_CIRC_EN_POS                 (3)
#define DMA_CTRL_DMA_CIRC_EN_MSK                 (0x1UL << DMA_CTRL_DMA_CIRC_EN_POS)
#define DMA_CTRL_DMA_CIRC_ENABLE                 (0x1UL << DMA_CTRL_DMA_CIRC_EN_POS)
#define DMA_CTRL_DMA_CIRC_DISABLE                (0x0UL << DMA_CTRL_DMA_CIRC_EN_POS)

/*
 * Bit definition for DMA_CTRL[2] - DMA transaction error interrupt enable
 */
#define DMA_CTRL_DMA_ERR_INT_EN_POS              (2)
#define DMA_CTRL_DMA_ERR_INT_EN_MSK              (0x1UL << DMA_CTRL_DMA_ERR_INT_EN_POS)
#define DMA_CTRL_DMA_ERR_INT_ENABLE              (0x1UL << DMA_CTRL_DMA_ERR_INT_EN_POS)
#define DMA_CTRL_DMA_ERR_INT_DISABLE             (0x0UL << DMA_CTRL_DMA_ERR_INT_EN_POS)

/*
 * Bit definition for DMA_CTRL[1] - DMA transaction complete interrupt enable
 */
#define DMA_CTRL_DMA_DONE_INT_EN_POS             (1)
#define DMA_CTRL_DMA_DONE_INT_EN_MSK             (0x1UL << DMA_CTRL_DMA_DONE_INT_EN_POS)
#define DMA_CTRL_DMA_DONE_INT_ENABLE             (0x1UL << DMA_CTRL_DMA_DONE_INT_EN_POS)
#define DMA_CTRL_DMA_DONE_INT_DISABLE            (0x0UL << DMA_CTRL_DMA_DONE_INT_EN_POS)

/*
 * Bit definition for DMA_CTRL[0] - DMA channel enable
 */
#define DMA_CTRL_DMA_EN_POS                      (0)
#define DMA_CTRL_DMA_EN_MSK                      (0x1UL << DMA_CTRL_DMA_EN_POS)
#define DMA_CTRL_DMA_ENABLE                      (0x1UL << DMA_CTRL_DMA_EN_POS)
#define DMA_CTRL_DMA_DISABLE                     (0x0UL << DMA_CTRL_DMA_EN_POS)

/*
 * Bit definition for DMA_INTSTS[12] - DMA channel 4 transfer error flag
 */
#define DMA_INTSTS_DMA4_TX_ERR_INTF_POS          (12)
#define DMA_INTSTS_DMA4_TX_ERR_INTF_MSK          (0x1UL << DMA_INTSTS_DMA4_TX_ERR_INTF_POS)
#define DMA_INTSTS_DMA4_TX_ERR_INT_FLAG          (0x1UL << DMA_INTSTS_DMA4_TX_ERR_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[11] - DMA channel 3 transfer error flag
 */
#define DMA_INTSTS_DMA3_TX_ERR_INTF_POS          (11)
#define DMA_INTSTS_DMA3_TX_ERR_INTF_MSK          (0x1UL << DMA_INTSTS_DMA3_TX_ERR_INTF_POS)
#define DMA_INTSTS_DMA3_TX_ERR_INT_FLAG          (0x1UL << DMA_INTSTS_DMA3_TX_ERR_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[10] - DMA channel 2 transfer error flag
 */
#define DMA_INTSTS_DMA2_TX_ERR_INTF_POS          (10)
#define DMA_INTSTS_DMA2_TX_ERR_INTF_MSK          (0x1UL << DMA_INTSTS_DMA2_TX_ERR_INTF_POS)
#define DMA_INTSTS_DMA2_TX_ERR_INT_FLAG          (0x1UL << DMA_INTSTS_DMA2_TX_ERR_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[9] - DMA channel 1 transfer error flag
 */
#define DMA_INTSTS_DMA1_TX_ERR_INTF_POS          (9)
#define DMA_INTSTS_DMA1_TX_ERR_INTF_MSK          (0x1UL << DMA_INTSTS_DMA1_TX_ERR_INTF_POS)
#define DMA_INTSTS_DMA1_TX_ERR_INT_FLAG          (0x1UL << DMA_INTSTS_DMA1_TX_ERR_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[8] - DMA channel 0 transfer error flag
 */
#define DMA_INTSTS_DMA0_TX_ERR_INTF_POS          (8)
#define DMA_INTSTS_DMA0_TX_ERR_INTF_MSK          (0x1UL << DMA_INTSTS_DMA0_TX_ERR_INTF_POS)
#define DMA_INTSTS_DMA0_TX_ERR_INT_FLAG          (0x1UL << DMA_INTSTS_DMA0_TX_ERR_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[4] - DMA channel 4 transaction complete interrupt flag
 */
#define DMA_INTSTS_DMA4_DONE_INTF_POS            (4)
#define DMA_INTSTS_DMA4_DONE_INTF_MSK            (0x1UL << DMA_INTSTS_DMA4_DONE_INTF_POS)
#define DMA_INTSTS_DMA4_DONE_INT_FLAG            (0x1UL << DMA_INTSTS_DMA4_DONE_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[3] - DMA channel 3 transaction complete interrupt flag
 */
#define DMA_INTSTS_DMA3_DONE_INTF_POS            (3)
#define DMA_INTSTS_DMA3_DONE_INTF_MSK            (0x1UL << DMA_INTSTS_DMA3_DONE_INTF_POS)
#define DMA_INTSTS_DMA3_DONE_INT_FLAG            (0x1UL << DMA_INTSTS_DMA3_DONE_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[2] - DMA channel 2 transaction complete interrupt flag
 */
#define DMA_INTSTS_DMA2_DONE_INTF_POS            (2)
#define DMA_INTSTS_DMA2_DONE_INTF_MSK            (0x1UL << DMA_INTSTS_DMA2_DONE_INTF_POS)
#define DMA_INTSTS_DMA2_DONE_INT_FLAG            (0x1UL << DMA_INTSTS_DMA2_DONE_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[1] - DMA channel 1 transaction complete interrupt flag
 */
#define DMA_INTSTS_DMA1_DONE_INTF_POS            (1)
#define DMA_INTSTS_DMA1_DONE_INTF_MSK            (0x1UL << DMA_INTSTS_DMA1_DONE_INTF_POS)
#define DMA_INTSTS_DMA1_DONE_INT_FLAG            (0x1UL << DMA_INTSTS_DMA1_DONE_INTF_POS)

/*
 * Bit definition for DMA_INTSTS[0] - DMA channel 0 transaction complete interrupt flag
 */
#define DMA_INTSTS_DMA0_DONE_INTF_POS            (0)
#define DMA_INTSTS_DMA0_DONE_INTF_MSK            (0x1UL << DMA_INTSTS_DMA0_DONE_INTF_POS)
#define DMA_INTSTS_DMA0_DONE_INT_FLAG            (0x1UL << DMA_INTSTS_DMA0_DONE_INTF_POS)


/*---------------------------------------------------------------------------------------
 * USB Control Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t UP;                            //
    __IO uint32_t DOWN;                          //
    __IO uint32_t EOT;                           //
    __IO uint32_t BC2;                           //
    __IO uint32_t BC1;                           //
    __IO uint32_t BC0;                           //
    __IO uint32_t STS;                           //
    __IO uint32_t EPEN;                          //
    __I  uint32_t RESERVED;                      // Reserved
    __IO uint32_t PHYCMD;                        //
    __IO uint32_t DEVADDR;                       //
    __IO uint32_t BC4;                           //
    __IO uint32_t BC5;                           //
    __IO uint32_t BC3;                           //
    __IO uint32_t INTERFACE;                     //
} USB_TYPE_DEF;

typedef struct
{
    __IO uint32_t BUFEN;                         //
    __IO uint32_t BUF10_LINK;                    //
    __IO uint32_t BUF32_LINK;                    //
    __IO uint32_t BUF54_LINK;                    //
    __IO uint32_t PHYCTRL;                       //
    __IO uint32_t CTRL;                          //
    __IO uint32_t INTEN;                         //
    __IO uint32_t INTSTS;                        //
} USB2_TYPE_DEF;

/*
 * Bit definition for USB_UP[3] -
 */
#define USB_UP_EP3_UP_STREAM_POS                 (3)
#define USB_UP_EP3_UP_STREAM_MSK                 (0x1UL << USB_UP_EP3_UP_STREAM_POS)
#define USB_UP_EP3_UP_STREAM_START               (0x1UL << USB_UP_EP3_UP_STREAM_POS)

/*
 * Bit definition for USB_UP[2] -
 */
#define USB_UP_EP2_UP_STREAM_POS                 (2)
#define USB_UP_EP2_UP_STREAM_MSK                 (0x1UL << USB_UP_EP2_UP_STREAM_POS)
#define USB_UP_EP2_UP_STREAM_START               (0x1UL << USB_UP_EP2_UP_STREAM_POS)

/*
 * Bit definition for USB_UP[1] -
 */
#define USB_UP_EP1_UP_STREAM_POS                 (1)
#define USB_UP_EP1_UP_STREAM_MSK                 (0x1UL << USB_UP_EP1_UP_STREAM_POS)
#define USB_UP_EP1_UP_STREAM_START               (0x1UL << USB_UP_EP1_UP_STREAM_POS)

/*
 * Bit definition for USB_UP[0] -
 */
#define USB_UP_EP0_UP_STREAM_POS                 (0)
#define USB_UP_EP0_UP_STREAM_MSK                 (0x1UL << USB_UP_EP0_UP_STREAM_POS)
#define USB_UP_EP0_UP_STREAM_START               (0x1UL << USB_UP_EP0_UP_STREAM_POS)

/*
 * Bit definition for USB_DOWN[3] -
 */
#define USB_DOWM_EP3_DOWN_STREAM_FLAG_POS        (3)
#define USB_DOWM_EP3_DOWN_STREAM_FLAG_MSK        (0x1UL << USB_DOWM_EP3_DOWN_STREAM_FLAG_POS)
#define USB_DOWM_EP3_DOWN_STREAM_FLAG            (0x1UL << USB_DOWM_EP3_DOWN_STREAM_FLAG_POS)
#define USB_DOWM_EP3_DOWN_STREAM_CLEAR           (0x0UL << USB_DOWM_EP3_DOWN_STREAM_FLAG_POS)

/*
 * Bit definition for USB_DOWN[2] -
 */
#define USB_DOWM_EP2_DOWN_STREAM_FLAG_POS        (2)
#define USB_DOWM_EP2_DOWN_STREAM_FLAG_MSK        (0x1UL << USB_DOWM_EP2_DOWN_STREAM_FLAG_POS)
#define USB_DOWM_EP2_DOWN_STREAM_FLAG            (0x1UL << USB_DOWM_EP2_DOWN_STREAM_FLAG_POS)
#define USB_DOWM_EP2_DOWN_STREAM_CLEAR           (0x0UL << USB_DOWM_EP2_DOWN_STREAM_FLAG_POS)

/*
 * Bit definition for USB_DOWN[1] -
 */
#define USB_DOWM_EP1_DOWN_STREAM_FLAG_POS        (1)
#define USB_DOWM_EP1_DOWN_STREAM_FLAG_MSK        (0x1UL << USB_DOWM_EP1_DOWN_STREAM_FLAG_POS)
#define USB_DOWM_EP1_DOWN_STREAM_FLAG            (0x1UL << USB_DOWM_EP1_DOWN_STREAM_FLAG_POS)
#define USB_DOWM_EP1_DOWN_STREAM_CLEAR           (0x0UL << USB_DOWM_EP1_DOWN_STREAM_FLAG_POS)

/*
 * Bit definition for USB_DOWN[0] -
 */
#define USB_DOWM_EP0_DOWN_STREAM_FLAG_POS        (0)
#define USB_DOWM_EP0_DOWN_STREAM_FLAG_MSK        (0x1UL << USB_DOWM_EP0_DOWN_STREAM_FLAG_POS)
#define USB_DOWM_EP0_DOWN_STREAM_FLAG            (0x1UL << USB_DOWM_EP0_DOWN_STREAM_FLAG_POS)
#define USB_DOWM_EP0_DOWN_STREAM_CLEAR           (0x0UL << USB_DOWM_EP0_DOWN_STREAM_FLAG_POS)

/*
 * Bit definition for USB_EOT[2] -
 */
#define USB_EOT_EP2EOT_FLAG_POS                  (2)
#define USB_EOT_EP2EOT_FLAG_MSK                  (0x1UL << USB_EOT_EP2EOT_FLAG_POS)
#define USB_EOT_EP2EOT_FLAG                      (0x1UL << USB_EOT_EP2EOT_FLAG_POS)

/*
 * Bit definition for USB_EOT[1] -
 */
#define USB_EOT_EP1EOT_FLAG_POS                  (1)
#define USB_EOT_EP1EOT_FLAG_MSK                  (0x1UL << USB_EOT_EP1EOT_FLAG_POS)
#define USB_EOT_EP1EOT_FLAG                      (0x1UL << USB_EOT_EP1EOT_FLAG_POS)

/*
 * Bit definition for USB_EOT[0] -
 */
#define USB_EOT_EP0EOT_FLAG_POS                  (0)
#define USB_EOT_EP0EOT_FLAG_MSK                  (0x1UL << USB_EOT_EP0EOT_FLAG_POS)
#define USB_EOT_EP0EOT_FLAG                      (0x1UL << USB_EOT_EP0EOT_FLAG_POS)

/*
 * Bit definition for USB_STS[3] -
 */
#define USB_STS_RST_FLAG_POS                     (3)
#define USB_STS_RST_FLAG_MSK                     (0x1UL << USB_STS_RST_FLAG_POS)
#define USB_STS_RST_FLAG                         (0x1UL << USB_STS_RST_FLAG_POS)
#define USB_STS_RST_CLEAR                        (0x0UL << USB_STS_RST_FLAG_POS)

/*
 * Bit definition for USB_STS[2] -
 */
#define USB_STS_SUSPEND_FLAG_POS                 (2)
#define USB_STS_SUSPEND_FLAG_MSK                 (0x1UL << USB_STS_SUSPEND_FLAG_POS)
#define USB_STS_SUSPEND_FLAG                     (0x1UL << USB_STS_SUSPEND_FLAG_POS)
#define USB_STS_SUSPEND_CLEAR                    (0x0UL << USB_STS_SUSPEND_FLAG_POS)

/*
 * Bit definition for USB_STS[1] -
 */
#define USB_STS_SETUP_FLAG_POS                   (1)
#define USB_STS_SETUP_FLAG_MSK                   (0x1UL << USB_STS_SETUP_FLAG_POS)
#define USB_STS_SETUP_FLAG                       (0x1UL << USB_STS_SETUP_FLAG_POS)

/*
 * Bit definition for USB_STS[0] -
 */
#define USB_STS_STALL_FLAG_POS                   (0)
#define USB_STS_STALL_FLAG_MSK                   (0x1UL << USB_STS_STALL_FLAG_POS)
#define USB_STS_STALL_FLAG                       (0x1UL << USB_STS_STALL_FLAG_POS)

/*
 * Bit definition for USB_EPEN[5] - Enable EP5
 */
#define USB_EPEN_EP5_EN_POS                      (5)
#define USB_EPEN_EP5_EN_MSK                      (0x1UL << USB_EPEN_EP5_EN_POS)
#define USB_EPEN_EP5_ENABLE                      (0x1UL << USB_EPEN_EP5_EN_POS)
#define USB_EPEN_EP5_DISABLE                     (0x0UL << USB_EPEN_EP5_EN_POS)

/*
 * Bit definition for USB_EPEN[4] - Enable EP4
 */
#define USB_EPEN_EP4_EN_POS                      (4)
#define USB_EPEN_EP4_EN_MSK                      (0x1UL << USB_EPEN_EP4_EN_POS)
#define USB_EPEN_EP4_ENABLE                      (0x1UL << USB_EPEN_EP4_EN_POS)
#define USB_EPEN_EP4_DISABLE                     (0x0UL << USB_EPEN_EP4_EN_POS)

/*
 * Bit definition for USB_EPEN[3] - Enable EP3
 */
#define USB_EPEN_EP3_EN_POS                      (3)
#define USB_EPEN_EP3_EN_MSK                      (0x1UL << USB_EPEN_EP3_EN_POS)
#define USB_EPEN_EP3_ENABLE                      (0x1UL << USB_EPEN_EP3_EN_POS)
#define USB_EPEN_EP3_DISABLE                     (0x0UL << USB_EPEN_EP3_EN_POS)

/*
 * Bit definition for USB_EPEN[2] - Enable EP2
 */
#define USB_EPEN_EP2_EN_POS                      (2)
#define USB_EPEN_EP2_EN_MSK                      (0x1UL << USB_EPEN_EP2_EN_POS)
#define USB_EPEN_EP2_ENABLE                      (0x1UL << USB_EPEN_EP2_EN_POS)
#define USB_EPEN_EP2_DISABLE                     (0x0UL << USB_EPEN_EP2_EN_POS)

/*
 * Bit definition for USB_EPEN[1] - Enable EP1
 */
#define USB_EPEN_EP1_EN_POS                      (1)
#define USB_EPEN_EP1_EN_MSK                      (0x1UL << USB_EPEN_EP1_EN_POS)
#define USB_EPEN_EP1_ENABLE                      (0x1UL << USB_EPEN_EP1_EN_POS)
#define USB_EPEN_EP1_DISABLE                     (0x0UL << USB_EPEN_EP1_EN_POS)

/*
 * Bit definition for USB_EPEN[0] - Enable EP0
 */
#define USB_EPEN_EP0_EN_POS                      (0)
#define USB_EPEN_EP0_EN_MSK                      (0x1UL << USB_EPEN_EP0_EN_POS)
#define USB_EPEN_EP0_ENABLE                      (0x1UL << USB_EPEN_EP0_EN_POS)
#define USB_EPEN_EP0_DISABLE                     (0x0UL << USB_EPEN_EP0_EN_POS)

/*
 * Bit definition for USB_PHYCMD[5] - SIE FIFO Reset
 */
#define USB_PHYCMD_SIE_FIFO_RST_POS              (5)
#define USB_PHYCMD_SIE_FIFO_RST_MSK              (0x1UL << USB_PHYCMD_SIE_FIFO_RST_POS)
#define USB_PHYCMD_SIE_FIFO_RST                  (0x1UL << USB_PHYCMD_SIE_FIFO_RST_POS)

/*
 * Bit definition for USB_PHYCMD[4] - USB Interface Reset
 */
#define USB_PHYCMD_INTERFACE_RST_POS             (4)
#define USB_PHYCMD_INTERFACE_RST_MSK             (0x1UL << USB_PHYCMD_INTERFACE_RST_POS)
#define USB_PHYCMD_INTERFACE_RST                 (0x1UL << USB_PHYCMD_INTERFACE_RST_POS)

/*
 * Bit definition for USB_PHYCMD[2] -
 */
#define USB_PHYCMD_DEVICE_STALL_POS              (2)
#define USB_PHYCMD_DEVICE_STALL_MSK              (0x1UL << USB_PHYCMD_DEVICE_STALL_POS)
#define USB_PHYCMD_DEVICE_STALL                  (0x1UL << USB_PHYCMD_DEVICE_STALL_POS)

/*
 * Bit definition for USB_PHYCMD[1] -
 */
#define USB_PHYCMD_ADDR_CLR_POS                  (1)
#define USB_PHYCMD_ADDR_CLR_MSK                  (0x1UL << USB_PHYCMD_ADDR_CLR_POS)
#define USB_PHYCMD_ADDR_CLR                      (0x1UL << USB_PHYCMD_ADDR_CLR_POS)

/*
 * Bit definition for USB_PHYCMD[0] -
 */
#define USB_PHYCMD_SIE_RST_POS                   (0)
#define USB_PHYCMD_SIE_RST_MSK                   (0x1UL << USB_PHYCMD_SIE_RST_POS)
#define USB_PHYCMD_SIE_RST                       (0x1UL << USB_PHYCMD_SIE_RST_POS)

/*
 * Bit definition for USB2_BUFEN[5] - Enable buffer5
 */
#define USB2_BUFEN_BUF5_EN_POS                   (5)
#define USB2_BUFEN_BUF5_EN_MSK                   (0x1UL << USB2_BUFEN_BUF5_EN_POS)
#define USB2_BUFEN_BUF5_ENABLE                   (0x1UL << USB2_BUFEN_BUF5_EN_POS)
#define USB2_BUFEN_BUF5_DISABLE                  (0x0UL << USB2_BUFEN_BUF5_EN_POS)

/*
 * Bit definition for USB2_BUFEN[4] - Enable buffer4
 */
#define USB2_BUFEN_BUF4_EN_POS                   (4)
#define USB2_BUFEN_BUF4_EN_MSK                   (0x1UL << USB2_BUFEN_BUF4_EN_POS)
#define USB2_BUFEN_BUF4_ENABLE                   (0x1UL << USB2_BUFEN_BUF4_EN_POS)
#define USB2_BUFEN_BUF4_DISABLE                  (0x0UL << USB2_BUFEN_BUF4_EN_POS)

/*
 * Bit definition for USB2_BUFEN[3] - Enable buffer3
 */
#define USB2_BUFEN_BUF3_EN_POS                   (3)
#define USB2_BUFEN_BUF3_EN_MSK                   (0x1UL << USB2_BUFEN_BUF3_EN_POS)
#define USB2_BUFEN_BUF3_ENABLE                   (0x1UL << USB2_BUFEN_BUF3_EN_POS)
#define USB2_BUFEN_BUF3_DISABLE                  (0x0UL << USB2_BUFEN_BUF3_EN_POS)

/*
 * Bit definition for USB2_BUFEN[2] - Enable buffer2
 */
#define USB2_BUFEN_BUF2_EN_POS                   (2)
#define USB2_BUFEN_BUF2_EN_MSK                   (0x1UL << USB2_BUFEN_BUF2_EN_POS)
#define USB2_BUFEN_BUF2_ENABLE                   (0x1UL << USB2_BUFEN_BUF2_EN_POS)
#define USB2_BUFEN_BUF2_DISABLE                  (0x0UL << USB2_BUFEN_BUF2_EN_POS)

/*
 * Bit definition for USB2_BUFEN[1] - Enable buffer1
 */
#define USB2_BUFEN_BUF1_EN_POS                   (1)
#define USB2_BUFEN_BUF1_EN_MSK                   (0x1UL << USB2_BUFEN_BUF1_EN_POS)
#define USB2_BUFEN_BUF1_ENABLE                   (0x1UL << USB2_BUFEN_BUF1_EN_POS)
#define USB2_BUFEN_BUF1_DISABLE                  (0x0UL << USB2_BUFEN_BUF1_EN_POS)

/*
 * Bit definition for USB2_BUFEN[0] - Enable buffer0
 */
#define USB2_BUFEN_BUF0_EN_POS                   (0)
#define USB2_BUFEN_BUF0_EN_MSK                   (0x1UL << USB2_BUFEN_BUF0_EN_POS)
#define USB2_BUFEN_BUF0_ENABLE                   (0x1UL << USB2_BUFEN_BUF0_EN_POS)
#define USB2_BUFEN_BUF0_DISABLE                  (0x0UL << USB2_BUFEN_BUF0_EN_POS)

/*
 * Bit definition for USB2_BUF10_LINK[6:4] -
 */
#define USB2_BUF10_BUF1_LINK_POS                 (4)
#define USB2_BUF10_BUF1_LINK_MSK                 (0x7UL << USB2_BUF10_BUF1_LINK_POS)
#define USB2_BUF10_BUF1_LINK_USB_ISO_TX          (0x4UL << USB2_BUF10_BUF1_LINK_POS)
#define USB2_BUF10_BUF1_LINK_CPU                 (0x3UL << USB2_BUF10_BUF1_LINK_POS)
#define USB2_BUF10_BUF1_LINK_USB_RX              (0x0UL << USB2_BUF10_BUF1_LINK_POS)

/*
 * Bit definition for USB2_BUF10_LINK[2:0] -
 */
#define USB2_BUF10_BUF0_LINK_POS                 (0)
#define USB2_BUF10_BUF0_LINK_MSK                 (0x7UL << USB2_BUF10_BUF0_LINK_POS)
#define USB2_BUF10_BUF0_LINK_USB_ISO_TX          (0x4UL << USB2_BUF10_BUF0_LINK_POS)
#define USB2_BUF10_BUF0_LINK_CPU                 (0x3UL << USB2_BUF10_BUF0_LINK_POS)
#define USB2_BUF10_BUF0_LINK_USB_RX              (0x0UL << USB2_BUF10_BUF0_LINK_POS)

/*
 * Bit definition for USB2_BUF32_LINK[6:4] -
 */
#define USB2_BUF32_BUF3_LINK_POS                 (4)
#define USB2_BUF32_BUF3_LINK_MSK                 (0x7UL << USB2_BUF32_BUF3_LINK_POS)
#define USB2_BUF32_BUF3_LINK_USB_ISO_TX          (0x4UL << USB2_BUF32_BUF3_LINK_POS)
#define USB2_BUF32_BUF3_LINK_CPU                 (0x3UL << USB2_BUF32_BUF3_LINK_POS)
#define USB2_BUF32_BUF3_LINK_USB_RX              (0x0UL << USB2_BUF32_BUF3_LINK_POS)

/*
 * Bit definition for USB2_BUF32_LINK[2:0] -
 */
#define USB2_BUF32_BUF2_LINK_POS                 (0)
#define USB2_BUF32_BUF2_LINK_MSK                 (0x7UL << USB2_BUF32_BUF2_LINK_POS)
#define USB2_BUF32_BUF2_LINK_USB_ISO_TX          (0x4UL << USB2_BUF32_BUF2_LINK_POS)
#define USB2_BUF32_BUF2_LINK_CPU                 (0x3UL << USB2_BUF32_BUF2_LINK_POS)
#define USB2_BUF32_BUF2_LINK_USB_RX              (0x0UL << USB2_BUF32_BUF2_LINK_POS)

/*
 * Bit definition for USB2_BUF54_LINK[6:4] -
 */
#define USB2_BUF54_BUF5_LINK_POS                 (4)
#define USB2_BUF54_BUF5_LINK_MSK                 (0x7UL << USB2_BUF54_BUF5_LINK_POS)
#define USB2_BUF54_BUF5_LINK_USB_ISO_TX          (0x4UL << USB2_BUF54_BUF5_LINK_POS)
#define USB2_BUF54_BUF5_LINK_CPU                 (0x3UL << USB2_BUF54_BUF5_LINK_POS)
#define USB2_BUF54_BUF5_LINK_USB_RX              (0x0UL << USB2_BUF54_BUF5_LINK_POS)

/*
 * Bit definition for USB2_BUF54_LINK[2:0] -
 */
#define USB2_BUF54_BUF4_LINK_POS                 (0)
#define USB2_BUF54_BUF4_LINK_MSK                 (0x7UL << USB2_BUF54_BUF4_LINK_POS)
#define USB2_BUF54_BUF4_LINK_USB_ISO_TX          (0x4UL << USB2_BUF54_BUF4_LINK_POS)
#define USB2_BUF54_BUF4_LINK_CPU                 (0x3UL << USB2_BUF54_BUF4_LINK_POS)
#define USB2_BUF54_BUF4_LINK_USB_RX              (0x0UL << USB2_BUF54_BUF4_LINK_POS)

/*
 * Bit definition for USB2_CTRL[4] -
 */
#define USB2_CTRL_ISO_EN_POS                     (4)
#define USB2_CTRL_ISO_EN_MSK                     (0x1UL << USB2_CTRL_ISO_EN_POS)
#define USB2_CTRL_ISO_ENALBE                     (0x1UL << USB2_CTRL_ISO_EN_POS)
#define USB2_CTRL_ISO_DISABLE                    (0x0UL << USB2_CTRL_ISO_EN_POS)

/*
 * Bit definition for USB2_CTRL[3] -
 */
#define USB2_CTRL_SUSPEND_EN_POS                 (3)
#define USB2_CTRL_SUSPEND_EN_MSK                 (0x1UL << USB2_CTRL_SUSPEND_EN_POS)
#define USB2_CTRL_SUSPEND_ENALBE                 (0x1UL << USB2_CTRL_SUSPEND_EN_POS)
#define USB2_CTRL_SUSPEND_DISABLE                (0x0UL << USB2_CTRL_SUSPEND_EN_POS)

/*
 * Bit definition for USB2_CTRL[1] - Pull high resistor enable
 */
#define USB2_CTRL_PULLH_EN_POS                   (1)
#define USB2_CTRL_PULLH_EN_MSK                   (0x1UL << USB2_CTRL_PULLH_EN_POS)
#define USB2_CTRL_PULLH_ENABLE                   (0x1UL << USB2_CTRL_PULLH_EN_POS)
#define USB2_CTRL_PULLH_DISABLE                  (0x0UL << USB2_CTRL_PULLH_EN_POS)

/*
 * Bit definition for USB2_CTRL[0] -
 */
#define USB2_CTRL_PYH_EN_POS                     (0)
#define USB2_CTRL_PYH_EN_MSK                     (0x1UL << USB2_CTRL_PYH_EN_POS)
#define USB2_CTRL_PYH_ENABLE                     (0x1UL << USB2_CTRL_PYH_EN_POS)
#define USB2_CTRL_PYH_DISABLE                    (0x0UL << USB2_CTRL_PYH_EN_POS)

/*
 * Bit definition for USB2_INTEN[7] -
 */
#define USB2_INTEN_SIE_EN_POS                    (7)
#define USB2_INTEN_SIE_EN_MSK                    (0x1UL << USB2_INTEN_SIE_EN_POS)
#define USB2_INTEN_SIE_ENABLE                    (0x1UL << USB2_INTEN_SIE_EN_POS)
#define USB2_INTEN_SIE_DISABLE                   (0x0UL << USB2_INTEN_SIE_EN_POS)

/*
 * Bit definition for USB2_INTEN[2] -
 */
#define USB2_INTEN_UP_STREAM_INT_EN_POS          (2)
#define USB2_INTEN_UP_STREAM_INT_EN_MSK          (0x1UL << USB2_INTEN_UP_STREAM_INT_EN_POS)
#define USB2_INTEN_UP_STREAM_INT_ENABLE          (0x1UL << USB2_INTEN_UP_STREAM_INT_EN_POS)
#define USB2_INTEN_UP_STREAM_INT_DISABLE         (0x0UL << USB2_INTEN_UP_STREAM_INT_EN_POS)

/*
 * Bit definition for USB2_INTEN[1] -
 */
#define USB2_INTEN_DN_STREAM_INT_EN_POS          (1)
#define USB2_INTEN_DN_STREAM_INT_EN_MSK          (0x1UL << USB2_INTEN_DN_STREAM_INT_EN_POS)
#define USB2_INTEN_DN_STREAM_INT_ENABLE          (0x1UL << USB2_INTEN_DN_STREAM_INT_EN_POS)
#define USB2_INTEN_DN_STREAM_INT_DISABLE         (0x0UL << USB2_INTEN_DN_STREAM_INT_EN_POS)

/*
 * Bit definition for USB2_INTEN[0] -
 */
#define USB2_INTEN_USB_INT_EN_POS                (0)
#define USB2_INTEN_USB_INT_EN_MSK                (0x1UL << USB2_INTEN_USB_INT_EN_POS)
#define USB2_INTEN_USB_INT_ENABLE                (0x1UL << USB2_INTEN_USB_INT_EN_POS)
#define USB2_INTEN_USB_INT_DISABLE               (0x0UL << USB2_INTEN_USB_INT_EN_POS)

/*
 * Bit definition for USB2_INTSTS[2] -
 */
#define USB2_INTSTS_UP_STREAM_INTF_POS           (2)
#define USB2_INTSTS_UP_STREAM_INTF_MSK           (0x1UL << USB2_INTSTS_UP_STREAM_INTF_POS)
#define USB2_INTSTS_UP_STREAM_INT_FLAG           (0x1UL << USB2_INTSTS_UP_STREAM_INTF_POS)

/*
 * Bit definition for USB2_INTSTS[1] -
 */
#define USB2_INTSTS_DN_STREAM_INTF_POS           (1)
#define USB2_INTSTS_DN_STREAM_INTF_MSK           (0x1UL << USB2_INTSTS_DN_STREAM_INTF_POS)
#define USB2_INTSTS_DN_STREAM_INT_FLAG           (0x1UL << USB2_INTSTS_DN_STREAM_INTF_POS)

/*
 * Bit definition for USB2_INTSTS[0] -
 */
#define USB2_INTSTS_USB_INTF_POS                 (0)
#define USB2_INTSTS_USB_INTF_MSK                 (0x1UL << USB2_INTSTS_USB_INTF_POS)
#define USB2_INTSTS_USB_INT_FLAG                 (0x1UL << USB2_INTSTS_USB_INTF_POS)

/*---------------------------------------------------------------------------------------
 * System Tick Control Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t CTRL;                          // SYSTICK Control Register
    __IO uint32_t RELOAD;                        // SYSTICK Reload Data Register
    __IO uint32_t COUNT;                         // SYSTICK Actual Value Register
} SYSTICK_TYPE_DEF;

/*
 * Bit definition for SYSTICK_CTRL[16] - SYSTICK interrupt flag
 */
#define SYSTICK_CTRL_INTF_POS                       (16)
#define SYSTICK_CTRL_INTF_MSK                       (0x1UL << SYSTICK_CTRL_INTF_POS)
#define SYSTICK_CTRL_INT_FLAG                       (0x1UL << SYSTICK_CTRL_INTF_POS)

/*
 * Bit definition for SYSTICK_CTRL[2] - SYSTICK Clock Source Select
 */
#define SYSTICK_CTRL_CLK_SEL_POS                    (2)
#define SYSTICK_CTRL_CLK_SEL_MSK                    (0x1UL << SYSTICK_CTRL_CLK_SEL_POS)
#define SYSTICK_CTRL_CLK_SEL_HCLK                   (0x1UL << SYSTICK_CTRL_CLK_SEL_POS)
#define SYSTICK_CTRL_INT_SEL_OPTION                 (0x0UL << SYSTICK_CTRL_CLK_SEL_POS)

/*
 * Bit definition for SYSTICK_CTRL[1] - SYSTICK Interrupt enable
 */
#define SYSTICK_CTRL_INT_EN_POS                     (1)
#define SYSTICK_CTRL_INT_EN_MSK                     (0x1UL << SYSTICK_CTRL_INT_EN_POS)
#define SYSTICK_CTRL_INT_ENABLE                     (0x1UL << SYSTICK_CTRL_INT_EN_POS)
#define SYSTICK_CTRL_INT_DISABLE                    (0x0UL << SYSTICK_CTRL_INT_EN_POS)

/*
 * Bit definition for SYSTICK_CTRL[0] - SYSTICK Counter enable
 */
#define SYSTICK_CTRL_CNT_EN_POS                     (0)
#define SYSTICK_CTRL_CNT_EN_MSK                     (0x1UL << SYSTICK_CTRL_CNT_EN_POS)
#define SYSTICK_CTRL_CNT_ENABLE                     (0x1UL << SYSTICK_CTRL_CNT_EN_POS)
#define SYSTICK_CTRL_CNT_DISABLE                    (0x0UL << SYSTICK_CTRL_CNT_EN_POS)

/*
 * Bit definition for SYSTICK_RELOAD[23:0] - SYSTICK Re-Load Data
 */
#define SYSTICK_RELOAD_DATA_POS                      (0)
#define SYSTICK_RELOAD_DATA_MSK                      (0xFFFFFFUL << SYSTICK_RELOAD_DATA_POS)

/*
 * Bit definition for SYSTICK_CTRL[1] - SYSTICK reload
 */
#define SYSTICK_CTRL_TM_RELOAD_POS                 (1)
#define SYSTICK_CTRL_TM_RELOAD_MSK                 (0x1UL << SYSTICK_CTRL_TM_RELOAD_POS)
#define SYSTICK_CTRL_TM_RELOAD                     (0x1UL << SYSTICK_CTRL_TM_RELOAD_POS)


/*---------------------------------------------------------------------------------------
 * SPU Control Unit Memory Mapped Structure & Constant Definitions
 *---------------------------------------------------------------------------------------*/
typedef struct
{
    __IO uint32_t SPU_EN_CH0_15;                 // SPU Channel[15:0] Enable Control           0
    __IO uint32_t MAIN_VOLUME;                   // SPU Main Volume   Ctrl                     4
    __IO uint32_t INT_EN_CH0_15;                 // SPU Channel FIQ Enable                     8
    __IO uint32_t INT_STATUS_CH0_15;             // SPU Channel FIQ Status                     C
    __IO uint32_t BEAT_BASE_COUNTER;             // SPU Beat Base Counter                      10
    __IO uint32_t BEAT_COUNTER;                  // SPU Beat Counter                           14
    __IO uint32_t ENV_CLK_CH0_3;                 // SPU Envelope Interval selection            18
    __IO uint32_t ENV_CLK_CH4_7;                 // SPU Envelope Interval selection            1C
    __IO uint32_t ENV_CLK_CH8_11;                // SPU Envelope Interval selection            20
    __IO uint32_t ENV_CLK_CH12_15;               // SPU Envelope Interval selection            24
    __IO uint32_t ENV_RAMPDOWN_CH0_15;           // SPU Ch[15:0] Envelope ramp down            28
    __IO uint32_t STOP_STATUS_CH0_15;            // SPU Channel[15:0] Stop status              2C
    __I  uint32_t RESERVED1;                     // SPU Reserved                               30
    __IO uint32_t CTRL_FLAG;                     // SPU Control Flag                           34
    __IO uint32_t COMPRESSOR;                    // SPU Compressor control                     38
    __IO uint32_t CH_STATUS_CH0_15;              // SPU Channel[15:0] status                   3C
    __I  uint32_t RESERVED2;                     // SPU Reserved                               40
    __I  uint32_t RESERVED3;                     // SPU Reserved                               44
    __IO uint32_t WAVE_OUT;                      // SPU The 16-bits output                     48
    __I  uint32_t RESERVED4;                     // SPU The 16-bits output                     4C
    __IO uint32_t ENV_REPEAT_CH0_15;             // SPU Channel[15:0] repeat enable            50
    __IO uint32_t ENV_MODE_CH0_15;               // SPU Channel[15:0] envelope mode            54
    __IO uint32_t TONE_RELEASE_CH0_15;           // SPU Channel[15:0] tone release control     58
    __IO uint32_t ENV_INT_STATUS_CH0_15;         // SPU Channel[15:0] envelope IRQ status      5C
    __IO uint32_t PITCHBEND_EN_CH0_15;           // SPU Channel[15:0] pitch bend enable        60
    __I  uint32_t RESERVED5;                     // SPU Channel[15:0] softch phase             64
    __IO uint32_t ATTACK_RELEASE_TIME;           // SPU Attack/Release Time Control            68
    __I  uint32_t RESERVED6;                     // SPU Reserved                               6C
    __I  uint32_t RESERVED7;                     // SPU Reserved                               70
    __I  uint32_t RESERVED8;                     // SPU Reserved                               74
    __I  uint32_t RESERVED9;                     // SPU Reserved                               78
    __I  uint32_t RESERVED10;                    // SPU Reserved                               7C
    __IO uint32_t SPU_EN_CH16_31;                // SPU Channel[31:16] Enable Control          80
    __I  uint32_t RESERVED11;                    // SPU Reserved                               84
    __IO uint32_t INT_EN_CH16_31;                // SPU Channel FIQ Enable                     88
    __IO uint32_t INT_STATUS_CH16_31;            // SPU Channel FIQ Status                     8C
    __I  uint32_t RESERVED12;                    // SPU Reserved                               90
    __IO uint32_t POSTWAVE_CTRL;                 // SPU PostWave Control                       94
    __IO uint32_t ENV_CLK_CH16_19;               // SPU Envelope Interval selection            98
    __IO uint32_t ENV_CLK_CH20_23;               // SPU Envelope Interval selection            9C
    __IO uint32_t ENV_CLK_CH24_27;               // SPU Envelope Interval selection            A0
    __IO uint32_t ENV_CLK_CH28_31;               // SPU Envelope Interval selection            A4
    __IO uint32_t ENV_RAMPDOWN_CH16_31;          // SPU Ch[31:16] Envelope ramp down           A8
    __IO uint32_t STOP_STATUS_CH16_31;           // SPU Channel[31:16] Stop status             AC
    __I  uint32_t RESERVED13;                    // SPU Reserved                               B0
    __I  uint32_t RESERVED14;                    // SPU Reserved                               B4
    __I  uint32_t RESERVED15;                    // SPU Reserved                               B8
    __IO uint32_t CH_STATUS_CH16_31;             // SPU Channel[31:16] status                  BC
    __I  uint32_t RESERVED16;                    // SPU Reserved                               C0
    __I  uint32_t RESERVED17;                    // SPU Reserved                               C4
    __I  uint32_t RESERVED18;                    // SPU Reserved                               C8
    __IO uint32_t POSTWAVE_OUT;                  // SPU POSTWAVE DATA                          CC
    __IO uint32_t ENV_REPEAT_CH16_31;            // SPU Channel[31:16] repeat enable           D0
    __IO uint32_t ENV_MODE_CH16_31;              // SPU Channel[31:16] envelope mode           D4
    __IO uint32_t TONE_RELEASE_CH16_31;          // SPU Channel[31:16] tone release control    D8
    __IO uint32_t ENV_INT_STATUS_CH16_31;        // SPU Channel[31:16] envelope IRQ status     DC
    __IO uint32_t PITCHBEND_EN_CH16_31;          // SPU Channel[31:16] pitch bend enable       E0
    __I  uint32_t RESERVED19;                    // SPU Reserved                               E4
    __I  uint32_t RESERVED20;                    // SPU Reserved                               E8
    __I  uint32_t RESERVED21;                    // SPU Reserved                               EC
    __I  uint32_t RESERVED22;                    // SPU Reserved                               F0
    __I  uint32_t RESERVED23;                    // SPU Reserved                               F4
    __I  uint32_t RESERVED24;                    // SPU Reserved                               F8
    __I  uint32_t RESERVED25;                    // SPU Reserved                               FC
    __IO uint32_t SPU_EN_CH32_47;                // SPU Channel[47:32] Enable Control          100
    __I  uint32_t RESERVED26;                    // SPU Reserved                               104
    __IO uint32_t INT_EN_CH32_47;                // SPU Channel FIQ Enable                     108
    __IO uint32_t INT_STATUS_CH32_47;            // SPU Channel FIQ Status                     10C
    __I  uint32_t RESERVED27;                    // SPU Reserved                               110
    __I  uint32_t RESERVED28;                    // SPU Reserved                               114
    __IO uint32_t ENV_CLK_CH32_35;               // SPU Envelope Interval selection            118
    __IO uint32_t ENV_CLK_CH36_39;               // SPU Envelope Interval selection            11C
    __IO uint32_t ENV_CLK_CH40_43;               // SPU Envelope Interval selection            120
    __IO uint32_t ENV_CLK_CH44_47;               // SPU Envelope Interval selection            124
    __IO uint32_t ENV_RAMPDOWN_CH32_47;          // SPU Ch[47:32] Envelope ramp down           128
    __IO uint32_t STOP_STATUS_CH32_47;           // SPU Channel[47:32] Stop status             12C
    __I  uint32_t RESERVED29;                    // SPU Reserved                               130
    __I  uint32_t RESERVED30;                    // SPU Reserved                               134
    __I  uint32_t RESERVED31;                    // SPU Reserved                               138
    __IO uint32_t CH_STATUS_CH32_47;             // SPU Channel[47:32] status                  13C
    __I  uint32_t RESERVED32;                    // SPU Reserved                               140
    __I  uint32_t RESERVED33;                    // SPU Reserved                               144
    __I  uint32_t RESERVED34;                    // SPU Reserved                               148
    __I  uint32_t RESERVED35;                    // SPU Reserved                               14C
    __IO uint32_t ENV_REPEAT_CH32_47;            // SPU Channel[47:32] repeat enable           150
    __IO uint32_t ENV_MODE_CH32_47;              // SPU Channel[47:32] envelope mode           154
    __IO uint32_t TONE_RELEASE_CH32_47;          // SPU Channel[47:32] tone release control    158
    __IO uint32_t ENV_INT_STATUS_CH32_47;        // SPU Channel[47:32] envelope IRQ status     15C
    __IO uint32_t PITCHBEND_EN_CH32_47;          // SPU Channel[47:32] pitch bend enable       160
    __I  uint32_t RESERVED36;                    // SPU Reserved                               164
    __I  uint32_t RESERVED37;                    // SPU Reserved                               168
    __I  uint32_t RESERVED38;                    // SPU Reserved                               16C
    __I  uint32_t RESERVED39;                    // SPU Reserved                               170
    __I  uint32_t RESERVED40;                    // SPU Reserved                               174
    __I  uint32_t RESERVED41;                    // SPU Reserved                               178
    __I  uint32_t RESERVED42;                    // SPU Reserved                               17C
    __IO uint32_t SPU_EN_CH48_63;                // SPU Channel[47:32] Enable Control          180
    __I  uint32_t RESERVED43;                    // SPU Reserve                                184
    __IO uint32_t INT_EN_CH48_63;                // SPU Channel FIQ Enable                     188
    __IO uint32_t INT_STATUS_CH48_63;            // SPU Channel FIQ Status                     18C
    __I  uint32_t RESERVED44;                    // SPU Reserved                               190
    __I  uint32_t RESERVED45;                    // SPU Reserved                               194
    __IO uint32_t ENV_CLK_CH48_51;               // SPU Envelope Interval selection            198
    __IO uint32_t ENV_CLK_CH52_55;               // SPU Envelope Interval selection            19C
    __IO uint32_t ENV_CLK_CH56_59;               // SPU Envelope Interval selection            1A0
    __IO uint32_t ENV_CLK_CH60_63;               // SPU Envelope Interval selection            1A4
    __IO uint32_t ENV_RAMPDOWN_CH48_63;          // SPU Ch[47:32] Envelope ramp down           1A8
    __IO uint32_t STOP_STATUS_CH48_63;           // SPU Channel[47:32] Stop status             1AC
    __I  uint32_t RESERVED46;                    // SPU Reserved                               1B0
    __I  uint32_t RESERVED47;                    // SPU Reserved                               1B4
    __I  uint32_t RESERVED48;                    // SPU Reserved                               1B8
    __IO uint32_t CH_STATUS_CH48_63;             // SPU Channel[47:32] status                  1BC
    __I  uint32_t RESERVED49;                    // SPU Reserved                               1C0
    __I  uint32_t RESERVED50;                    // SPU Reserved                               1C4
    __I  uint32_t RESERVED51;                    // SPU Reserved                               1C8
    __I  uint32_t RESERVED52;                    // SPU Reserved                               1CC
    __IO uint32_t ENV_REPEAT_CH48_63;            // SPU Channel[47:32] repeat enable           1D0
    __IO uint32_t ENV_MODE_CH48_63;              // SPU Channel[47:32] envelope mode           1D4
    __IO uint32_t TONE_RELEASE_CH48_63;          // SPU Channel[47:32] tone release control    1D8
    __IO uint32_t ENV_INT_STATUS_CH48_63;        // SPU Channel[47:32] envelope IRQ status     1DC
    __IO uint32_t PITCHBEND_EN_CH48_63;          // SPU Channel[47:32] pitch bend enable       1E0
} SPU_TYPE_DEF;

typedef struct
{
    __IO uint32_t WAVE_ADDR;                     // 0
    __IO uint32_t WAVE_MODE;                     // 4
    __IO uint32_t WAVE_LOOPADDR;                 // 8
    __IO uint32_t PAN_VELOCITY;                  // C
    __IO uint32_t ENV_CTRL0;                     // 10
    __IO uint32_t ENV_DATA;                      // 14
    __IO uint32_t ENV_CTRL1;                     // 18
    __IO uint32_t ENV_INT_CTRL;                  // 1C
    __IO uint32_t ENV_ADDR;                      // 20
    __IO uint32_t WAVE_DATA0;                    // 24
    __IO uint32_t ENV_RAMPDOWN_STEP;             // 28
    __IO uint32_t WAVE_DATA;                     // 2C
    __I  uint32_t RESERVED0;                     // 30
    __IO uint32_t WAVE_ADPCM_CTRL;               // 34
    __IO uint32_t WAVE_ADDRH;                    // 38
    __IO uint32_t ENV_ADDRH;                     // 3C
    __I  uint32_t RESERVED[0x7F0];               // 0x1FC0 Byte Offset
    __IO uint32_t PHASE_VALUEH;                  // + 0x2000, Ch0 : 0x50083000
    __IO uint32_t PHASE_ACCH;                    // + 0x2004
    __IO uint32_t PITCHBEND_TARGETH;             // + 0x2008
    __IO uint32_t ENV_RAMPDOWN_CLK;              // + 0x200C
    __IO uint32_t PHASE_VALUE;                   // + 0x2010
    __IO uint32_t PHASE_ACC;                     // + 0x2014
    __IO uint32_t PITCHBEND_TARGET;              // + 0x2018
    __IO uint32_t PITCHBEND_CTRL;                // + 0x201C
} SPU_CH_TYPE_DEF;

/*
 * Bit definition for SPU->SPU_EN_CH0_15[15:0]
 */
#define SPU_CH0_EN_POS                           (0)
#define SPU_CH0_EN_MSK                           (0x1UL << SPU_CH0_EN_POS)
#define SPU_CH0_ENABLE                           (0x1UL << SPU_CH0_EN_POS)
#define SPU_CH0_DISABLE                          (0x0UL << SPU_CH0_EN_POS)
#define SPU_CH1_EN_POS                           (1)
#define SPU_CH1_EN_MSK                           (0x1UL << SPU_CH1_EN_POS)
#define SPU_CH1_ENABLE                           (0x1UL << SPU_CH1_EN_POS)
#define SPU_CH1_DISABLE                          (0x0UL << SPU_CH1_EN_POS)
#define SPU_CH2_EN_POS                           (2)
#define SPU_CH2_EN_MSK                           (0x1UL << SPU_CH2_EN_POS)
#define SPU_CH2_ENABLE                           (0x1UL << SPU_CH2_EN_POS)
#define SPU_CH2_DISABLE                          (0x0UL << SPU_CH2_EN_POS)
#define SPU_CH3_EN_POS                           (3)
#define SPU_CH3_EN_MSK                           (0x1UL << SPU_CH3_EN_POS)
#define SPU_CH3_ENABLE                           (0x1UL << SPU_CH3_EN_POS)
#define SPU_CH3_DISABLE                          (0x0UL << SPU_CH3_EN_POS)
#define SPU_CH4_EN_POS                           (4)
#define SPU_CH4_EN_MSK                           (0x1UL << SPU_CH4_EN_POS)
#define SPU_CH4_ENABLE                           (0x1UL << SPU_CH4_EN_POS)
#define SPU_CH4_DISABLE                          (0x0UL << SPU_CH4_EN_POS)
#define SPU_CH5_EN_POS                           (5)
#define SPU_CH5_EN_MSK                           (0x1UL << SPU_CH5_EN_POS)
#define SPU_CH5_ENABLE                           (0x1UL << SPU_CH5_EN_POS)
#define SPU_CH5_DISABLE                          (0x0UL << SPU_CH5_EN_POS)
#define SPU_CH6_EN_POS                           (6)
#define SPU_CH6_EN_MSK                           (0x1UL << SPU_CH6_EN_POS)
#define SPU_CH6_ENABLE                           (0x1UL << SPU_CH6_EN_POS)
#define SPU_CH6_DISABLE                          (0x0UL << SPU_CH6_EN_POS)
#define SPU_CH7_EN_POS                           (7)
#define SPU_CH7_EN_MSK                           (0x1UL << SPU_CH7_EN_POS)
#define SPU_CH7_ENABLE                           (0x1UL << SPU_CH7_EN_POS)
#define SPU_CH7_DISABLE                          (0x0UL << SPU_CH7_EN_POS)
#define SPU_CH8_EN_POS                           (8)
#define SPU_CH8_EN_MSK                           (0x1UL << SPU_CH8_EN_POS)
#define SPU_CH8_ENABLE                           (0x1UL << SPU_CH8_EN_POS)
#define SPU_CH8_DISABLE                          (0x0UL << SPU_CH8_EN_POS)
#define SPU_CH9_EN_POS                           (9)
#define SPU_CH9_EN_MSK                           (0x1UL << SPU_CH9_EN_POS)
#define SPU_CH9_ENABLE                           (0x1UL << SPU_CH9_EN_POS)
#define SPU_CH9_DISABLE                          (0x0UL << SPU_CH9_EN_POS)
#define SPU_CH10_EN_POS                          (10)
#define SPU_CH10_EN_MSK                          (0x1UL << SPU_CH10_EN_POS)
#define SPU_CH10_ENABLE                          (0x1UL << SPU_CH10_EN_POS)
#define SPU_CH10_DISABLE                         (0x0UL << SPU_CH10_EN_POS)
#define SPU_CH11_EN_POS                          (11)
#define SPU_CH11_EN_MSK                          (0x1UL << SPU_CH11_EN_POS)
#define SPU_CH11_ENABLE                          (0x1UL << SPU_CH11_EN_POS)
#define SPU_CH11_DISABLE                         (0x0UL << SPU_CH11_EN_POS)
#define SPU_CH12_EN_POS                          (12)
#define SPU_CH12_EN_MSK                          (0x1UL << SPU_CH12_EN_POS)
#define SPU_CH12_ENABLE                          (0x1UL << SPU_CH12_EN_POS)
#define SPU_CH12_DISABLE                         (0x0UL << SPU_CH12_EN_POS)
#define SPU_CH13_EN_POS                          (13)
#define SPU_CH13_EN_MSK                          (0x1UL << SPU_CH13_EN_POS)
#define SPU_CH13_ENABLE                          (0x1UL << SPU_CH13_EN_POS)
#define SPU_CH13_DISABLE                         (0x0UL << SPU_CH13_EN_POS)
#define SPU_CH14_EN_POS                          (14)
#define SPU_CH14_EN_MSK                          (0x1UL << SPU_CH14_EN_POS)
#define SPU_CH14_ENABLE                          (0x1UL << SPU_CH14_EN_POS)
#define SPU_CH14_DISABLE                         (0x0UL << SPU_CH14_EN_POS)
#define SPU_CH15_EN_POS                          (15)
#define SPU_CH15_EN_MSK                          (0x1UL << SPU_CH15_EN_POS)
#define SPU_CH15_ENABLE                          (0x1UL << SPU_CH15_EN_POS)
#define SPU_CH15_DISABLE                         (0x0UL << SPU_CH15_EN_POS)

/*
 * Bit definition for SPU->MAIN_VOLUME[6:0] - SPU Main Volume Ctrl
 */
#define SPU_MAIN_VOLUME_POS                      (0)
#define SPU_MAIN_VOLUME_MSK                      (0x7FUL << SPU_MAIN_VOLUME_POS)
#define SPU_MAIN_VOLUME_0x00                     (0x00UL << SPU_MAIN_VOLUME_POS)
#define SPU_MAIN_VOLUME_0x7F                     (0x7FUL << SPU_MAIN_VOLUME_POS)

/*
 * Bit definition for SPU->INT_EN_CH0_15[15:0] - SPU Channel FIQ Enable
 */
#define SPU_INT_CH0_EN_POS                       (0)
#define SPU_INT_CH0_EN_MSK                       (0x1UL << SPU_INT_CH0_EN_POS)
#define SPU_INT_CH0_ENABLE                       (0x1UL << SPU_INT_CH0_EN_POS)
#define SPU_INT_CH0_DISABLE                      (0x0UL << SPU_INT_CH0_EN_POS)
#define SPU_INT_CH1_EN_POS                       (1)
#define SPU_INT_CH1_EN_MSK                       (0x1UL << SPU_INT_CH1_EN_POS)
#define SPU_INT_CH1_ENABLE                       (0x1UL << SPU_INT_CH1_EN_POS)
#define SPU_INT_CH1_DISABLE                      (0x0UL << SPU_INT_CH1_EN_POS)
#define SPU_INT_CH2_EN_POS                       (2)
#define SPU_INT_CH2_EN_MSK                       (0x1UL << SPU_INT_CH2_EN_POS)
#define SPU_INT_CH2_ENABLE                       (0x1UL << SPU_INT_CH2_EN_POS)
#define SPU_INT_CH2_DISABLE                      (0x0UL << SPU_INT_CH2_EN_POS)
#define SPU_INT_CH3_EN_POS                       (3)
#define SPU_INT_CH3_EN_MSK                       (0x1UL << SPU_INT_CH3_EN_POS)
#define SPU_INT_CH3_ENABLE                       (0x1UL << SPU_INT_CH3_EN_POS)
#define SPU_INT_CH3_DISABLE                      (0x0UL << SPU_INT_CH3_EN_POS)
#define SPU_INT_CH4_EN_POS                       (4)
#define SPU_INT_CH4_EN_MSK                       (0x1UL << SPU_INT_CH4_EN_POS)
#define SPU_INT_CH4_ENABLE                       (0x1UL << SPU_INT_CH4_EN_POS)
#define SPU_INT_CH4_DISABLE                      (0x0UL << SPU_INT_CH4_EN_POS)
#define SPU_INT_CH5_EN_POS                       (5)
#define SPU_INT_CH5_EN_MSK                       (0x1UL << SPU_INT_CH5_EN_POS)
#define SPU_INT_CH5_ENABLE                       (0x1UL << SPU_INT_CH5_EN_POS)
#define SPU_INT_CH5_DISABLE                      (0x0UL << SPU_INT_CH5_EN_POS)
#define SPU_INT_CH6_EN_POS                       (6)
#define SPU_INT_CH6_EN_MSK                       (0x1UL << SPU_INT_CH6_EN_POS)
#define SPU_INT_CH6_ENABLE                       (0x1UL << SPU_INT_CH6_EN_POS)
#define SPU_INT_CH6_DISABLE                      (0x0UL << SPU_INT_CH6_EN_POS)
#define SPU_INT_CH7_EN_POS                       (7)
#define SPU_INT_CH7_EN_MSK                       (0x1UL << SPU_INT_CH7_EN_POS)
#define SPU_INT_CH7_ENABLE                       (0x1UL << SPU_INT_CH7_EN_POS)
#define SPU_INT_CH7_DISABLE                      (0x0UL << SPU_INT_CH7_EN_POS)
#define SPU_INT_CH8_EN_POS                       (8)
#define SPU_INT_CH8_EN_MSK                       (0x1UL << SPU_INT_CH8_EN_POS)
#define SPU_INT_CH8_ENABLE                       (0x1UL << SPU_INT_CH8_EN_POS)
#define SPU_INT_CH8_DISABLE                      (0x0UL << SPU_INT_CH8_EN_POS)
#define SPU_INT_CH9_EN_POS                       (9)
#define SPU_INT_CH9_EN_MSK                       (0x1UL << SPU_INT_CH9_EN_POS)
#define SPU_INT_CH9_ENABLE                       (0x1UL << SPU_INT_CH9_EN_POS)
#define SPU_INT_CH9_DISABLE                      (0x0UL << SPU_INT_CH9_EN_POS)
#define SPU_INT_CH10_EN_POS                      (10)
#define SPU_INT_CH10_EN_MSK                      (0x1UL << SPU_INT_CH10_EN_POS)
#define SPU_INT_CH10_ENABLE                      (0x1UL << SPU_INT_CH10_EN_POS)
#define SPU_INT_CH10_DISABLE                     (0x0UL << SPU_INT_CH10_EN_POS)
#define SPU_INT_CH11_EN_POS                      (11)
#define SPU_INT_CH11_EN_MSK                      (0x1UL << SPU_INT_CH11_EN_POS)
#define SPU_INT_CH11_ENABLE                      (0x1UL << SPU_INT_CH11_EN_POS)
#define SPU_INT_CH11_DISABLE                     (0x0UL << SPU_INT_CH11_EN_POS)
#define SPU_INT_CH12_EN_POS                      (12)
#define SPU_INT_CH12_EN_MSK                      (0x1UL << SPU_INT_CH12_EN_POS)
#define SPU_INT_CH12_ENABLE                      (0x1UL << SPU_INT_CH12_EN_POS)
#define SPU_INT_CH12_DISABLE                     (0x0UL << SPU_INT_CH12_EN_POS)
#define SPU_INT_CH13_EN_POS                      (13)
#define SPU_INT_CH13_EN_MSK                      (0x1UL << SPU_INT_CH13_EN_POS)
#define SPU_INT_CH13_ENABLE                      (0x1UL << SPU_INT_CH13_EN_POS)
#define SPU_INT_CH13_DISABLE                     (0x0UL << SPU_INT_CH13_EN_POS)
#define SPU_INT_CH14_EN_POS                      (14)
#define SPU_INT_CH14_EN_MSK                      (0x1UL << SPU_INT_CH14_EN_POS)
#define SPU_INT_CH14_ENABLE                      (0x1UL << SPU_INT_CH14_EN_POS)
#define SPU_INT_CH14_DISABLE                     (0x0UL << SPU_INT_CH14_EN_POS)
#define SPU_INT_CH15_EN_POS                      (15)
#define SPU_INT_CH15_EN_MSK                      (0x1UL << SPU_INT_CH15_EN_POS)
#define SPU_INT_CH15_ENABLE                      (0x1UL << SPU_INT_CH15_EN_POS)
#define SPU_INT_CH15_DISABLE                     (0x0UL << SPU_INT_CH15_EN_POS)

/*
 * Bit definition for INT_STATUS_CH0_15[15:0] - SPU Channel INT Status
 */
#define SPU_INT_FLAG_CH0_POS                     (0)
#define SPU_INT_FLAG_CH0_MSK                     (0x1UL << SPU_INT_FLAG_CH0_POS)
#define SPU_INT_FLAG_CH0                         (0x1UL << SPU_INT_FLAG_CH0_POS)
#define SPU_INT_FLAG_CH1_POS                     (1)
#define SPU_INT_FLAG_CH1_MSK                     (0x1UL << SPU_INT_FLAG_CH1_POS)
#define SPU_INT_FLAG_CH1                         (0x1UL << SPU_INT_FLAG_CH1_POS)
#define SPU_INT_FLAG_CH2_POS                     (2)
#define SPU_INT_FLAG_CH2_MSK                     (0x1UL << SPU_INT_FLAG_CH2_POS)
#define SPU_INT_FLAG_CH2                         (0x1UL << SPU_INT_FLAG_CH2_POS)
#define SPU_INT_FLAG_CH3_POS                     (3)
#define SPU_INT_FLAG_CH3_MSK                     (0x1UL << SPU_INT_FLAG_CH3_POS)
#define SPU_INT_FLAG_CH3                         (0x0UL << SPU_INT_FLAG_CH3_POS)
#define SPU_INT_FLAG_CH4_POS                     (4)
#define SPU_INT_FLAG_CH4_MSK                     (0x1UL << SPU_INT_FLAG_CH4_POS)
#define SPU_INT_FLAG_CH4                         (0x0UL << SPU_INT_FLAG_CH4_POS)
#define SPU_INT_FLAG_CH5_POS                     (5)
#define SPU_INT_FLAG_CH5_MSK                     (0x1UL << SPU_INT_FLAG_CH5_POS)
#define SPU_INT_FLAG_CH5                         (0x0UL << SPU_INT_FLAG_CH5_POS)
#define SPU_INT_FLAG_CH6_POS                     (6)
#define SPU_INT_FLAG_CH6_MSK                     (0x1UL << SPU_INT_FLAG_CH6_POS)
#define SPU_INT_FLAG_CH6                         (0x0UL << SPU_INT_FLAG_CH6_POS)
#define SPU_INT_FLAG_CH7_POS                     (7)
#define SPU_INT_FLAG_CH7_MSK                     (0x1UL << SPU_INT_FLAG_CH7_POS)
#define SPU_INT_FLAG_CH7                         (0x0UL << SPU_INT_FLAG_CH7_POS)
#define SPU_INT_FLAG_CH8_POS                     (8)
#define SPU_INT_FLAG_CH8_MSK                     (0x1UL << SPU_INT_FLAG_CH8_POS)
#define SPU_INT_FLAG_CH8                         (0x0UL << SPU_INT_FLAG_CH8_POS)
#define SPU_INT_FLAG_CH9_POS                     (9)
#define SPU_INT_FLAG_CH9_MSK                     (0x1UL << SPU_INT_FLAG_CH9_POS)
#define SPU_INT_FLAG_CH9                         (0x0UL << SPU_INT_FLAG_CH9_POS)
#define SPU_INT_FLAG_CH10_POS                    (10)
#define SPU_INT_FLAG_CH10_MSK                    (0x1UL << SPU_INT_FLAG_CH10_POS)
#define SPU_INT_FLAG_CH10                        (0x0UL << SPU_INT_FLAG_CH10_POS)
#define SPU_INT_FLAG_CH11_POS                    (11)
#define SPU_INT_FLAG_CH11_MSK                    (0x1UL << SPU_INT_FLAG_CH11_POS)
#define SPU_INT_FLAG_CH11                        (0x0UL << SPU_INT_FLAG_CH11_POS)
#define SPU_INT_FLAG_CH12_POS                    (12)
#define SPU_INT_FLAG_CH12_MSK                    (0x1UL << SPU_INT_FLAG_CH12_POS)
#define SPU_INT_FLAG_CH12                        (0x0UL << SPU_INT_FLAG_CH12_POS)
#define SPU_INT_FLAG_CH13_POS                    (13)
#define SPU_INT_FLAG_CH13_MSK                    (0x1UL << SPU_INT_FLAG_CH13_POS)
#define SPU_INT_FLAG_CH13                        (0x0UL << SPU_INT_FLAG_CH13_POS)
#define SPU_INT_FLAG_CH14_POS                    (14)
#define SPU_INT_FLAG_CH14_MSK                    (0x1UL << SPU_INT_FLAG_CH14_POS)
#define SPU_INT_FLAG_CH14                        (0x0UL << SPU_INT_FLAG_CH14_POS)
#define SPU_INT_FLAG_CH15_POS                    (15)
#define SPU_INT_FLAG_CH15_MSK                    (0x1UL << SPU_INT_FLAG_CH15_POS)
#define SPU_INT_FLAG_CH15                        (0x0UL << SPU_INT_FLAG_CH15_POS)

/*
 * Bit definition for BEAT_BASE_COUNTER[10:0] - SPU BEAT BASE COUNTER
 */
#define SPU_BEAT_BASE_COUNTER_POS                (0)
#define SPU_BEAT_BASE_COUNTER_MSK                (0x7FFUL << SPU_BEAT_BASE_COUNTER_POS)
#define SPU_BEAT_BASE_COUNTER_0x000              (0x000UL << SPU_BEAT_BASE_COUNTER_POS)
#define SPU_BEAT_BASE_COUNTER_0x7FF              (0x7FFUL << SPU_BEAT_BASE_COUNTER_POS)

/*
 * Bit definition for BEAT_COUNTER[15] - SPU BEAT COUNTER IRQ
 */
#define SPU_BEAT_COUNTER_INT_POS                 (15)
#define SPU_BEAT_COUNTER_INT_MSK                 (0x1UL << SPU_BEAT_COUNTER_INT_POS)
#define SPU_BEAT_COUNTER_INT_ENABLE              (0x1UL << SPU_BEAT_COUNTER_INT_POS)
#define SPU_BEAT_COUNTER_INT_DISABLE             (0x0UL << SPU_BEAT_COUNTER_INT_POS)

/*
 * Bit definition for BEAT_COUNTER[14] - SPU BEAT COUNTER IRQ FLAG
 */
#define SPU_BEAT_COUNTER_INT_FLAG_POS            (15)
#define SPU_BEAT_COUNTER_INT_FLAG_MSK            (0x1UL << SPU_BEAT_COUNTER_INT_FLAG_POS)
#define SPU_BEAT_COUNTER_INT_FLAG                (0x1UL << SPU_BEAT_COUNTER_INT_FLAG_POS)

/*
 * Bit definition for BEAT_COUNTER[13:0] - SPU BEAT COUNTER
 */
#define SPU_BEAT_COUNTER_POS                     (0)
#define SPU_BEAT_COUNTER_MSK                     (0x3FFFUL << SPU_BEAT_COUNTER_POS)
#define SPU_BEAT_COUNTER_0x0000                  (0x0000UL << SPU_BEAT_COUNTER_POS)
#define SPU_BEAT_COUNTER_0x3FFF                  (0x3FFFUL << SPU_BEAT_COUNTER_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH0_3[3:0] -
 */
#define SPU_ENV_CLK_CH0_POS                      (0)
#define SPU_ENV_CLK_CH0_MSK                      (0xFUL << SPU_ENV_CLK_CH0_POS)
#define SPU_ENV_CLK_CH0_0x0                      (0x0UL << SPU_ENV_CLK_CH0_POS)
#define SPU_ENV_CLK_CH0_0xF                      (0xFUL << SPU_ENV_CLK_CH0_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH0_3[7:4] -
 */
#define SPU_ENV_CLK_CH1_POS                      (4)
#define SPU_ENV_CLK_CH1_MSK                      (0xFUL << SPU_ENV_CLK_CH1_POS)
#define SPU_ENV_CLK_CH1_0x0                      (0x0UL << SPU_ENV_CLK_CH1_POS)
#define SPU_ENV_CLK_CH1_0xF                      (0xFUL << SPU_ENV_CLK_CH1_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH0_3[11:8] -
 */
#define SPU_ENV_CLK_CH2_POS                      (8)
#define SPU_ENV_CLK_CH2_MSK                      (0xFUL << SPU_ENV_CLK_CH2_POS)
#define SPU_ENV_CLK_CH2_0x0                      (0x0UL << SPU_ENV_CLK_CH2_POS)
#define SPU_ENV_CLK_CH2_0xF                      (0xFUL << SPU_ENV_CLK_CH2_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH0_3[15:12] -
 */
#define SPU_ENV_CLK_CH3_POS                      (12)
#define SPU_ENV_CLK_CH3_MSK                      (0xFUL << SPU_ENV_CLK_CH3_POS)
#define SPU_ENV_CLK_CH3_0x0                      (0x0UL << SPU_ENV_CLK_CH3_POS)
#define SPU_ENV_CLK_CH3_0xF                      (0xFUL << SPU_ENV_CLK_CH3_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH4_7[3:0] -
 */
#define SPU_ENV_CLK_CH4_POS                      (0)
#define SPU_ENV_CLK_CH4_MSK                      (0xFUL << SPU_ENV_CLK_CH4_POS)
#define SPU_ENV_CLK_CH4_0x0                      (0x0UL << SPU_ENV_CLK_CH4_POS)
#define SPU_ENV_CLK_CH4_0xF                      (0xFUL << SPU_ENV_CLK_CH4_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH4_7[7:4] -
 */
#define SPU_ENV_CLK_CH5_POS                      (4)
#define SPU_ENV_CLK_CH5_MSK                      (0xFUL << SPU_ENV_CLK_CH5_POS)
#define SPU_ENV_CLK_CH5_0x0                      (0x0UL << SPU_ENV_CLK_CH5_POS)
#define SPU_ENV_CLK_CH5_0xF                      (0xFUL << SPU_ENV_CLK_CH5_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH4_7[11:8] -
 */
#define SPU_ENV_CLK_CH6_POS                      (8)
#define SPU_ENV_CLK_CH6_MSK                      (0xFUL << SPU_ENV_CLK_CH6_POS)
#define SPU_ENV_CLK_CH6_0x0                      (0x0UL << SPU_ENV_CLK_CH6_POS)
#define SPU_ENV_CLK_CH6_0xF                      (0xFUL << SPU_ENV_CLK_CH6_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH4_7[15:12] -
 */
#define SPU_ENV_CLK_CH7_POS                      (12)
#define SPU_ENV_CLK_CH7_MSK                      (0xFUL << SPU_ENV_CLK_CH7_POS)
#define SPU_ENV_CLK_CH7_0x0                      (0x0UL << SPU_ENV_CLK_CH7_POS)
#define SPU_ENV_CLK_CH7_0xF                      (0xFUL << SPU_ENV_CLK_CH7_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH8_11[3:0] -
 */
#define SPU_ENV_CLK_CH8_POS                      (0)
#define SPU_ENV_CLK_CH8_MSK                      (0xFUL << SPU_ENV_CLK_CH8_POS)
#define SPU_ENV_CLK_CH8_0x0                      (0x0UL << SPU_ENV_CLK_CH8_POS)
#define SPU_ENV_CLK_CH8_0xF                      (0xFUL << SPU_ENV_CLK_CH8_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH8_11[7:4] -
 */
#define SPU_ENV_CLK_CH9_POS                      (4)
#define SPU_ENV_CLK_CH9_MSK                      (0xFUL << SPU_ENV_CLK_CH9_POS)
#define SPU_ENV_CLK_CH9_0x0                      (0x0UL << SPU_ENV_CLK_CH9_POS)
#define SPU_ENV_CLK_CH9_0xF                      (0xFUL << SPU_ENV_CLK_CH9_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH8_11[11:8] -
 */
#define SPU_ENV_CLK_CH10_POS                     (8)
#define SPU_ENV_CLK_CH10_MSK                     (0xFUL << SPU_ENV_CLK_CH10_POS)
#define SPU_ENV_CLK_CH10_0x0                     (0x0UL << SPU_ENV_CLK_CH10_POS)
#define SPU_ENV_CLK_CH10_0xF                     (0xFUL << SPU_ENV_CLK_CH10_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH8_11[15:12] -
 */
#define SPU_ENV_CLK_CH11_POS                     (12)
#define SPU_ENV_CLK_CH11_MSK                     (0xFUL << SPU_ENV_CLK_CH11_POS)
#define SPU_ENV_CLK_CH11_0x0                     (0x0UL << SPU_ENV_CLK_CH11_POS)
#define SPU_ENV_CLK_CH11_0xF                     (0xFUL << SPU_ENV_CLK_CH11_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH12_15[3:0] -
 */
#define SPU_ENV_CLK_CH12_POS                     (0)
#define SPU_ENV_CLK_CH12_MSK                     (0xFUL << SPU_ENV_CLK_CH12_POS)
#define SPU_ENV_CLK_CH12_0x0                     (0x0UL << SPU_ENV_CLK_CH12_POS)
#define SPU_ENV_CLK_CH12_0xF                     (0xFUL << SPU_ENV_CLK_CH12_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH12_15[7:4] -
 */
#define SPU_ENV_CLK_CH13_POS                     (4)
#define SPU_ENV_CLK_CH13_MSK                     (0xFUL << SPU_ENV_CLK_CH13_POS)
#define SPU_ENV_CLK_CH13_0x0                     (0x0UL << SPU_ENV_CLK_CH13_POS)
#define SPU_ENV_CLK_CH13_0xF                     (0xFUL << SPU_ENV_CLK_CH13_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH12_15[11:8] -
 */
#define SPU_ENV_CLK_CH14_POS                     (8)
#define SPU_ENV_CLK_CH14_MSK                     (0xFUL << SPU_ENV_CLK_CH14_POS)
#define SPU_ENV_CLK_CH14_0x0                     (0x0UL << SPU_ENV_CLK_CH14_POS)
#define SPU_ENV_CLK_CH14_0xF                     (0xFUL << SPU_ENV_CLK_CH14_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH12_15[15:12] -
 */
#define SPU_ENV_CLK_CH15_POS                     (12)
#define SPU_ENV_CLK_CH15_MSK                     (0xFUL << SPU_ENV_CLK_CH15_POS)
#define SPU_ENV_CLK_CH15_0x0                     (0x0UL << SPU_ENV_CLK_CH15_POS)
#define SPU_ENV_CLK_CH15_0xF                     (0xFUL << SPU_ENV_CLK_CH15_POS)

/*
 * Bit definition for SPU->ENV_RAMPDOWN_CH0_15[15:0]
 */
#define SPU_ENV_RAMPDOWN_CH0_POS                 (0)
#define SPU_ENV_RAMPDOWN_CH0_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH0_POS)
#define SPU_ENV_RAMPDOWN_CH0_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH0_POS)
#define SPU_ENV_RAMPDOWN_CH0_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH0_POS)
#define SPU_ENV_RAMPDOWN_CH1_POS                 (1)
#define SPU_ENV_RAMPDOWN_CH1_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH1_POS)
#define SPU_ENV_RAMPDOWN_CH1_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH1_POS)
#define SPU_ENV_RAMPDOWN_CH1_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH1_POS)
#define SPU_ENV_RAMPDOWN_CH2_POS                 (2)
#define SPU_ENV_RAMPDOWN_CH2_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH2_POS)
#define SPU_ENV_RAMPDOWN_CH2_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH2_POS)
#define SPU_ENV_RAMPDOWN_CH2_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH2_POS)
#define SPU_ENV_RAMPDOWN_CH3_POS                 (3)
#define SPU_ENV_RAMPDOWN_CH3_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH3_POS)
#define SPU_ENV_RAMPDOWN_CH3_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH3_POS)
#define SPU_ENV_RAMPDOWN_CH3_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH3_POS)
#define SPU_ENV_RAMPDOWN_CH4_POS                 (4)
#define SPU_ENV_RAMPDOWN_CH4_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH4_POS)
#define SPU_ENV_RAMPDOWN_CH4_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH4_POS)
#define SPU_ENV_RAMPDOWN_CH4_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH4_POS)
#define SPU_ENV_RAMPDOWN_CH5_POS                 (5)
#define SPU_ENV_RAMPDOWN_CH5_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH5_POS)
#define SPU_ENV_RAMPDOWN_CH5_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH5_POS)
#define SPU_ENV_RAMPDOWN_CH5_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH5_POS)
#define SPU_ENV_RAMPDOWN_CH6_POS                 (6)
#define SPU_ENV_RAMPDOWN_CH6_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH6_POS)
#define SPU_ENV_RAMPDOWN_CH6_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH6_POS)
#define SPU_ENV_RAMPDOWN_CH6_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH6_POS)
#define SPU_ENV_RAMPDOWN_CH7_POS                 (7)
#define SPU_ENV_RAMPDOWN_CH7_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH7_POS)
#define SPU_ENV_RAMPDOWN_CH7_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH7_POS)
#define SPU_ENV_RAMPDOWN_CH7_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH7_POS)
#define SPU_ENV_RAMPDOWN_CH8_POS                 (8)
#define SPU_ENV_RAMPDOWN_CH8_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH8_POS)
#define SPU_ENV_RAMPDOWN_CH8_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH8_POS)
#define SPU_ENV_RAMPDOWN_CH8_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH8_POS)
#define SPU_ENV_RAMPDOWN_CH9_POS                 (9)
#define SPU_ENV_RAMPDOWN_CH9_MSK                 (0x1UL << SPU_ENV_RAMPDOWN_CH9_POS)
#define SPU_ENV_RAMPDOWN_CH9_ENABLE              (0x1UL << SPU_ENV_RAMPDOWN_CH9_POS)
#define SPU_ENV_RAMPDOWN_CH9_DISABLE             (0x0UL << SPU_ENV_RAMPDOWN_CH9_POS)
#define SPU_ENV_RAMPDOWN_CH10_POS                (10)
#define SPU_ENV_RAMPDOWN_CH10_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH10_POS)
#define SPU_ENV_RAMPDOWN_CH10_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH10_POS)
#define SPU_ENV_RAMPDOWN_CH10_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH10_POS)
#define SPU_ENV_RAMPDOWN_CH11_POS                (11)
#define SPU_ENV_RAMPDOWN_CH11_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH11_POS)
#define SPU_ENV_RAMPDOWN_CH11_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH11_POS)
#define SPU_ENV_RAMPDOWN_CH11_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH11_POS)
#define SPU_ENV_RAMPDOWN_CH12_POS                (12)
#define SPU_ENV_RAMPDOWN_CH12_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH12_POS)
#define SPU_ENV_RAMPDOWN_CH12_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH12_POS)
#define SPU_ENV_RAMPDOWN_CH12_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH12_POS)
#define SPU_ENV_RAMPDOWN_CH13_POS                (13)
#define SPU_ENV_RAMPDOWN_CH13_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH13_POS)
#define SPU_ENV_RAMPDOWN_CH13_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH13_POS)
#define SPU_ENV_RAMPDOWN_CH13_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH13_POS)
#define SPU_ENV_RAMPDOWN_CH14_POS                (14)
#define SPU_ENV_RAMPDOWN_CH14_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH14_POS)
#define SPU_ENV_RAMPDOWN_CH14_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH14_POS)
#define SPU_ENV_RAMPDOWN_CH14_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH14_POS)
#define SPU_ENV_RAMPDOWN_CH15_POS                (15)
#define SPU_ENV_RAMPDOWN_CH15_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH15_POS)
#define SPU_ENV_RAMPDOWN_CH15_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH15_POS)
#define SPU_ENV_RAMPDOWN_CH15_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH15_POS)

/*
 * Bit definition for SPU->STOP_STATUS_CH0_15[15:0]
 */
#define SPU_STOP_FLAG_CH0_POS                    (0)
#define SPU_STOP_FLAG_CH0_MSK                    (0x1UL << SPU_STOP_FLAG_CH0_POS)
#define SPU_STOP_FLAG_CH0                        (0x1UL << SPU_STOP_FLAG_CH0_POS)
#define SPU_STOP_FLAG_CH1_POS                    (1)
#define SPU_STOP_FLAG_CH1_MSK                    (0x1UL << SPU_STOP_FLAG_CH1_POS)
#define SPU_STOP_FLAG_CH1                        (0x1UL << SPU_STOP_FLAG_CH1_POS)
#define SPU_STOP_FLAG_CH2_POS                    (2)
#define SPU_STOP_FLAG_CH2_MSK                    (0x1UL << SPU_STOP_FLAG_CH2_POS)
#define SPU_STOP_FLAG_CH2                        (0x1UL << SPU_STOP_FLAG_CH2_POS)
#define SPU_STOP_FLAG_CH3_POS                    (3)
#define SPU_STOP_FLAG_CH3_MSK                    (0x1UL << SPU_STOP_FLAG_CH3_POS)
#define SPU_STOP_FLAG_CH3                        (0x0UL << SPU_STOP_FLAG_CH3_POS)
#define SPU_STOP_FLAG_CH4_POS                    (4)
#define SPU_STOP_FLAG_CH4_MSK                    (0x1UL << SPU_STOP_FLAG_CH4_POS)
#define SPU_STOP_FLAG_CH4                        (0x0UL << SPU_STOP_FLAG_CH4_POS)
#define SPU_STOP_FLAG_CH5_POS                    (5)
#define SPU_STOP_FLAG_CH5_MSK                    (0x1UL << SPU_STOP_FLAG_CH5_POS)
#define SPU_STOP_FLAG_CH5                        (0x0UL << SPU_STOP_FLAG_CH5_POS)
#define SPU_STOP_FLAG_CH6_POS                    (6)
#define SPU_STOP_FLAG_CH6_MSK                    (0x1UL << SPU_STOP_FLAG_CH6_POS)
#define SPU_STOP_FLAG_CH6                        (0x0UL << SPU_STOP_FLAG_CH6_POS)
#define SPU_STOP_FLAG_CH7_POS                    (7)
#define SPU_STOP_FLAG_CH7_MSK                    (0x1UL << SPU_STOP_FLAG_CH7_POS)
#define SPU_STOP_FLAG_CH7                        (0x0UL << SPU_STOP_FLAG_CH7_POS)
#define SPU_STOP_FLAG_CH8_POS                    (8)
#define SPU_STOP_FLAG_CH8_MSK                    (0x1UL << SPU_STOP_FLAG_CH8_POS)
#define SPU_STOP_FLAG_CH8                        (0x0UL << SPU_STOP_FLAG_CH8_POS)
#define SPU_STOP_FLAG_CH9_POS                    (9)
#define SPU_STOP_FLAG_CH9_MSK                    (0x1UL << SPU_STOP_FLAG_CH9_POS)
#define SPU_STOP_FLAG_CH9                        (0x0UL << SPU_STOP_FLAG_CH9_POS)
#define SPU_STOP_FLAG_CH10_POS                   (10)
#define SPU_STOP_FLAG_CH10_MSK                   (0x1UL << SPU_STOP_FLAG_CH10_POS)
#define SPU_STOP_FLAG_CH10                       (0x0UL << SPU_STOP_FLAG_CH10_POS)
#define SPU_STOP_FLAG_CH11_POS                   (11)
#define SPU_STOP_FLAG_CH11_MSK                   (0x1UL << SPU_STOP_FLAG_CH11_POS)
#define SPU_STOP_FLAG_CH11                       (0x0UL << SPU_STOP_FLAG_CH11_POS)
#define SPU_STOP_FLAG_CH12_POS                   (12)
#define SPU_STOP_FLAG_CH12_MSK                   (0x1UL << SPU_STOP_FLAG_CH12_POS)
#define SPU_STOP_FLAG_CH12                       (0x0UL << SPU_STOP_FLAG_CH12_POS)
#define SPU_STOP_FLAG_CH13_POS                   (13)
#define SPU_STOP_FLAG_CH13_MSK                   (0x1UL << SPU_STOP_FLAG_CH13_POS)
#define SPU_STOP_FLAG_CH13                       (0x0UL << SPU_STOP_FLAG_CH13_POS)
#define SPU_STOP_FLAG_CH14_POS                   (14)
#define SPU_STOP_FLAG_CH14_MSK                   (0x1UL << SPU_STOP_FLAG_CH14_POS)
#define SPU_STOP_FLAG_CH14                       (0x0UL << SPU_STOP_FLAG_CH14_POS)
#define SPU_STOP_FLAG_CH15_POS                   (15)
#define SPU_STOP_FLAG_CH15_MSK                   (0x1UL << SPU_STOP_FLAG_CH15_POS)
#define SPU_STOP_FLAG_CH15                       (0x0UL << SPU_STOP_FLAG_CH15_POS)

/*
 * Bit definition for SPU->CTRL_FLAG[15]
 */
#define SPU_SATURATION_POS                       (15)
#define SPU_SATURATION_MSK                       (0x1UL << SPU_SATURATION_POS)
#define SPU_SATURATION_FLAG                      (0x1UL << SPU_SATURATION_POS)

/*
 * Bit definition for SPU->CTRL_FLAG[13]
 */
#define SPU_LOOP_INITIALS0S1_POS                 (13)
#define SPU_LOOP_INITIALS0S1_MSK                 (0x1UL << SPU_LOOP_INITIALS0S1_POS)
#define SPU_LOOP_INITIALS0S1_ENABLE              (0x1UL << SPU_LOOP_INITIALS0S1_POS)
#define SPU_LOOP_INITIALS0S1_DISABLE             (0x0UL << SPU_LOOP_INITIALS0S1_POS)

/*
 * Bit definition for SPU->CTRL_FLAG[11]
 */
#define SPU_COMPRESSOR_EN_POS                    (11)
#define SPU_COMPRESSOR_EN_MSK                    (0x1UL << SPU_COMPRESSOR_EN_POS)
#define SPU_COMPRESSOR_ENABLE                    (0x1UL << SPU_COMPRESSOR_EN_POS)
#define SPU_COMPRESSOR_DISABLE                   (0x0UL << SPU_COMPRESSOR_EN_POS)

/*
 * Bit definition for SPU->CTRL_FLAG[10:9]
 */
#define SPU_INTERPOLATION_EN_POS                 (9)
#define SPU_INTERPOLATION_EN_MSK                 (0x3UL << SPU_INTERPOLATION_EN_POS)
#define SPU_INTERPOLATION_ENABLE                 (0x2UL << SPU_INTERPOLATION_EN_POS)
#define SPU_INTERPOLATION_HQ_ENABLE              (0x0UL << SPU_INTERPOLATION_EN_POS)
#define SPU_INTERPOLATION_DISABLE                (0x3UL << SPU_INTERPOLATION_EN_POS)

/*
 * Bit definition for SPU->CTRL_FLAG[7:5] ???
 */
#define SPU_VOLUME_SEL_POS                       (5)
#define SPU_VOLUME_SEL_MSK                       (0x7UL << SPU_VOLUME_SEL_POS)

/*
 * Bit definition for SPU->CTRL_FLAG[4]
 */
#define SPU_OVERLOADING_POS                      (4)
#define SPU_OVERLOADING_MSK                      (0x1UL << SPU_INTERPOLATION_EN_POS)
#define SPU_OVERLOADING_FLAG                     (0x1UL << SPU_INTERPOLATION_EN_POS)

/*
 * Bit definition for SPU->CTRL_FLAG[3]
 */
#define SPU_PHASEADDER_INITIAL_POS               (3)
#define SPU_PHASEADDER_INITIAL_MSK               (0x1UL << SPU_INTERPOLATION_EN_POS)
#define SPU_PHASEADDER_INITIAL                   (0x1UL << SPU_INTERPOLATION_EN_POS)

/*
 * Bit definition for SPU->CTRL_FLAG[2:0]
 */
#define SPU_CH_NUMBER_SEL_POS                    (0)
#define SPU_CH_NUMBER_SEL_MSK                    (0x7UL << SPU_CH_NUMBER_SEL_POS)
#define SPU_CH_NUMBER_8CH                        (0x0UL << SPU_CH_NUMBER_SEL_POS)
#define SPU_CH_NUMBER_16CH                       (0x1UL << SPU_CH_NUMBER_SEL_POS)
#define SPU_CH_NUMBER_24CH                       (0x2UL << SPU_CH_NUMBER_SEL_POS)
#define SPU_CH_NUMBER_32CH                       (0x3UL << SPU_CH_NUMBER_SEL_POS)
#define SPU_CH_NUMBER_48CH                       (0x4UL << SPU_CH_NUMBER_SEL_POS)
#define SPU_CH_NUMBER_64CH                       (0x5UL << SPU_CH_NUMBER_SEL_POS)

/*
 * Bit definition for SPU->COMPRESSOR[15]
 */
#define SPU_COMPRESSOR_MODE_SEL_POS              (15)
#define SPU_COMPRESSOR_MODE_SEL_MSK              (0x1UL << SPU_COMPRESSOR_MODE_SEL_POS)
#define SPU_COMPRESSOR_MODE_PEAK                 (0x1UL << SPU_COMPRESSOR_MODE_SEL_POS)
#define SPU_COMPRESSOR_MODE_RMS                  (0x0UL << SPU_COMPRESSOR_MODE_SEL_POS)

/*
 * Bit definition for SPU->COMPRESSOR[14:8]
 */
#define SPU_COMPRESSOR_THRESHOLD_POS             (8)
#define SPU_COMPRESSOR_THRESHOLD_MSK             (0x7FUL << SPU_COMPRESSOR_THRESHOLD_POS)
#define SPU_COMPRESSOR_THRESHOLD_0x00            (0x00UL << SPU_COMPRESSOR_THRESHOLD_POS)
#define SPU_COMPRESSOR_THRESHOLD_0x7F            (0x7FUL << SPU_COMPRESSOR_THRESHOLD_POS)
#define SPU_COMPRESSOR_THRESHOLD_0x20            (0x20UL << SPU_COMPRESSOR_THRESHOLD_POS)
#define SPU_COMPRESSOR_THRESHOLD_0x50            (0x50UL << SPU_COMPRESSOR_THRESHOLD_POS)

/*
 * Bit definition for SPU->COMPRESSOR[7:6]
 */
#define SPU_COMPRESSOR_ATTACK_POS                (6)
#define SPU_COMPRESSOR_ATTACK_MSK                (0x3UL << SPU_COMPRESSOR_ATTACK_POS)
#define SPU_COMPRESSOR_ATTACKx1                  (0x0UL << SPU_COMPRESSOR_ATTACK_POS)
#define SPU_COMPRESSOR_ATTACKx4                  (0x1UL << SPU_COMPRESSOR_ATTACK_POS)
#define SPU_COMPRESSOR_ATTACKx16                 (0x2UL << SPU_COMPRESSOR_ATTACK_POS)
#define SPU_COMPRESSOR_ATTACKx64                 (0x3UL << SPU_COMPRESSOR_ATTACK_POS)

/*
 * Bit definition for SPU->COMPRESSOR[5:4]
 */
#define SPU_COMPRESSOR_RELEASE_POS               (4)
#define SPU_COMPRESSOR_RELEASE_MSK               (0x3UL << SPU_COMPRESSOR_RELEASE_POS)
#define SPU_COMPRESSOR_RELEASEx1                 (0x0UL << SPU_COMPRESSOR_RELEASE_POS)
#define SPU_COMPRESSOR_RELEASEx4                 (0x1UL << SPU_COMPRESSOR_RELEASE_POS)
#define SPU_COMPRESSOR_RELEASEx16                (0x2UL << SPU_COMPRESSOR_RELEASE_POS)
#define SPU_COMPRESSOR_RELEASEx64                (0x3UL << SPU_COMPRESSOR_RELEASE_POS)

/*
 * Bit definition for SPU->COMPRESSOR[3]
 */
#define SPU_COMPRESSOR_ZC_EN_POS                 (3)
#define SPU_COMPRESSOR_ZC_EN_MSK                 (0x1UL << SPU_COMPRESSOR_ZC_EN_POS)
#define SPU_COMPRESSOR_ZC_ENABLE                 (0x1UL << SPU_COMPRESSOR_ZC_EN_POS)
#define SPU_COMPRESSOR_ZC_DISABLE                (0x0UL << SPU_COMPRESSOR_ZC_EN_POS)

/*
 * Bit definition for SPU->COMPRESSOR[2:0]
 */
#define SPU_COMPRESSOR_Ratio_POS                 (0)
#define SPU_COMPRESSOR_Ratio_MSK                 (0x7UL << SPU_COMPRESSOR_Ratio_POS)
#define SPU_COMPRESSOR_Ratio_DIV_2               (0x0UL << SPU_COMPRESSOR_Ratio_POS)
#define SPU_COMPRESSOR_Ratio_DIV_3               (0x1UL << SPU_COMPRESSOR_Ratio_POS)
#define SPU_COMPRESSOR_Ratio_DIV_4               (0x2UL << SPU_COMPRESSOR_Ratio_POS)
#define SPU_COMPRESSOR_Ratio_DIV_5               (0x3UL << SPU_COMPRESSOR_Ratio_POS)
#define SPU_COMPRESSOR_Ratio_DIV_6               (0x4UL << SPU_COMPRESSOR_Ratio_POS)
#define SPU_COMPRESSOR_Ratio_DIV_7               (0x5UL << SPU_COMPRESSOR_Ratio_POS)
#define SPU_COMPRESSOR_Ratio_DIV_8               (0x6UL << SPU_COMPRESSOR_Ratio_POS)
#define SPU_COMPRESSOR_Ratio_Min                 (0x7UL << SPU_COMPRESSOR_Ratio_POS)

/*
 * Bit definition for SPU->CH_STATUS_CH0_15[15:0]
 */
#define SPU_BUSY_FLAG_CH0_POS                    (0)
#define SPU_BUSY_FLAG_CH0_MSK                    (0x1UL << SPU_BUSY_FLAG_CH0_POS)
#define SPU_BUSY_FLAG_CH0                        (0x1UL << SPU_BUSY_FLAG_CH0_POS)
#define SPU_BUSY_FLAG_CH1_POS                    (1)
#define SPU_BUSY_FLAG_CH1_MSK                    (0x1UL << SPU_BUSY_FLAG_CH1_POS)
#define SPU_BUSY_FLAG_CH1                        (0x1UL << SPU_BUSY_FLAG_CH1_POS)
#define SPU_BUSY_FLAG_CH2_POS                    (2)
#define SPU_BUSY_FLAG_CH2_MSK                    (0x1UL << SPU_BUSY_FLAG_CH2_POS)
#define SPU_BUSY_FLAG_CH2                        (0x1UL << SPU_BUSY_FLAG_CH2_POS)
#define SPU_BUSY_FLAG_CH3_POS                    (3)
#define SPU_BUSY_FLAG_CH3_MSK                    (0x1UL << SPU_BUSY_FLAG_CH3_POS)
#define SPU_BUSY_FLAG_CH3                        (0x0UL << SPU_BUSY_FLAG_CH3_POS)
#define SPU_BUSY_FLAG_CH4_POS                    (4)
#define SPU_BUSY_FLAG_CH4_MSK                    (0x1UL << SPU_BUSY_FLAG_CH4_POS)
#define SPU_BUSY_FLAG_CH4                        (0x0UL << SPU_BUSY_FLAG_CH4_POS)
#define SPU_BUSY_FLAG_CH5_POS                    (5)
#define SPU_BUSY_FLAG_CH5_MSK                    (0x1UL << SPU_BUSY_FLAG_CH5_POS)
#define SPU_BUSY_FLAG_CH5                        (0x0UL << SPU_BUSY_FLAG_CH5_POS)
#define SPU_BUSY_FLAG_CH6_POS                    (6)
#define SPU_BUSY_FLAG_CH6_MSK                    (0x1UL << SPU_BUSY_FLAG_CH6_POS)
#define SPU_BUSY_FLAG_CH6                        (0x0UL << SPU_BUSY_FLAG_CH6_POS)
#define SPU_BUSY_FLAG_CH7_POS                    (7)
#define SPU_BUSY_FLAG_CH7_MSK                    (0x1UL << SPU_BUSY_FLAG_CH7_POS)
#define SPU_BUSY_FLAG_CH7                        (0x0UL << SPU_BUSY_FLAG_CH7_POS)
#define SPU_BUSY_FLAG_CH8_POS                    (8)
#define SPU_BUSY_FLAG_CH8_MSK                    (0x1UL << SPU_BUSY_FLAG_CH8_POS)
#define SPU_BUSY_FLAG_CH8                        (0x0UL << SPU_BUSY_FLAG_CH8_POS)
#define SPU_BUSY_FLAG_CH9_POS                    (9)
#define SPU_BUSY_FLAG_CH9_MSK                    (0x1UL << SPU_BUSY_FLAG_CH9_POS)
#define SPU_BUSY_FLAG_CH9                        (0x0UL << SPU_BUSY_FLAG_CH9_POS)
#define SPU_BUSY_FLAG_CH10_POS                   (10)
#define SPU_BUSY_FLAG_CH10_MSK                   (0x1UL << SPU_BUSY_FLAG_CH10_POS)
#define SPU_BUSY_FLAG_CH10                       (0x0UL << SPU_BUSY_FLAG_CH10_POS)
#define SPU_BUSY_FLAG_CH11_POS                   (11)
#define SPU_BUSY_FLAG_CH11_MSK                   (0x1UL << SPU_BUSY_FLAG_CH11_POS)
#define SPU_BUSY_FLAG_CH11                       (0x0UL << SPU_BUSY_FLAG_CH11_POS)
#define SPU_BUSY_FLAG_CH12_POS                   (12)
#define SPU_BUSY_FLAG_CH12_MSK                   (0x1UL << SPU_BUSY_FLAG_CH12_POS)
#define SPU_BUSY_FLAG_CH12                       (0x0UL << SPU_BUSY_FLAG_CH12_POS)
#define SPU_BUSY_FLAG_CH13_POS                   (13)
#define SPU_BUSY_FLAG_CH13_MSK                   (0x1UL << SPU_BUSY_FLAG_CH13_POS)
#define SPU_BUSY_FLAG_CH13                       (0x0UL << SPU_BUSY_FLAG_CH13_POS)
#define SPU_BUSY_FLAG_CH14_POS                   (14)
#define SPU_BUSY_FLAG_CH14_MSK                   (0x1UL << SPU_BUSY_FLAG_CH14_POS)
#define SPU_BUSY_FLAG_CH14                       (0x0UL << SPU_BUSY_FLAG_CH14_POS)
#define SPU_BUSY_FLAG_CH15_POS                   (15)
#define SPU_BUSY_FLAG_CH15_MSK                   (0x1UL << SPU_BUSY_FLAG_CH15_POS)
#define SPU_BUSY_FLAG_CH15                       (0x0UL << SPU_BUSY_FLAG_CH15_POS)

/*
 * Bit definition for SPU->ENV_REPEAT_CH0_15[15:0]
 */
#define SPU_ENV_REPEAT_CH0_POS                   (0)
#define SPU_ENV_REPEAT_CH0_MSK                   (0x1UL << SPU_ENV_REPEAT_CH0_POS)
#define SPU_ENV_REPEAT_CH0_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH0_POS)
#define SPU_ENV_REPEAT_CH0_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH0_POS)
#define SPU_ENV_REPEAT_CH1_POS                   (1)
#define SPU_ENV_REPEAT_CH1_MSK                   (0x1UL << SPU_ENV_REPEAT_CH1_POS)
#define SPU_ENV_REPEAT_CH1_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH1_POS)
#define SPU_ENV_REPEAT_CH1_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH1_POS)
#define SPU_ENV_REPEAT_CH2_POS                   (2)
#define SPU_ENV_REPEAT_CH2_MSK                   (0x1UL << SPU_ENV_REPEAT_CH2_POS)
#define SPU_ENV_REPEAT_CH2_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH2_POS)
#define SPU_ENV_REPEAT_CH2_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH2_POS)
#define SPU_ENV_REPEAT_CH3_POS                   (3)
#define SPU_ENV_REPEAT_CH3_MSK                   (0x1UL << SPU_ENV_REPEAT_CH3_POS)
#define SPU_ENV_REPEAT_CH3_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH3_POS)
#define SPU_ENV_REPEAT_CH3_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH3_POS)
#define SPU_ENV_REPEAT_CH4_POS                   (4)
#define SPU_ENV_REPEAT_CH4_MSK                   (0x1UL << SPU_ENV_REPEAT_CH4_POS)
#define SPU_ENV_REPEAT_CH4_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH4_POS)
#define SPU_ENV_REPEAT_CH4_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH4_POS)
#define SPU_ENV_REPEAT_CH5_POS                   (5)
#define SPU_ENV_REPEAT_CH5_MSK                   (0x1UL << SPU_ENV_REPEAT_CH5_POS)
#define SPU_ENV_REPEAT_CH5_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH5_POS)
#define SPU_ENV_REPEAT_CH5_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH5_POS)
#define SPU_ENV_REPEAT_CH6_POS                   (6)
#define SPU_ENV_REPEAT_CH6_MSK                   (0x1UL << SPU_ENV_REPEAT_CH6_POS)
#define SPU_ENV_REPEAT_CH6_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH6_POS)
#define SPU_ENV_REPEAT_CH6_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH6_POS)
#define SPU_ENV_REPEAT_CH7_POS                   (7)
#define SPU_ENV_REPEAT_CH7_MSK                   (0x1UL << SPU_ENV_REPEAT_CH7_POS)
#define SPU_ENV_REPEAT_CH7_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH7_POS)
#define SPU_ENV_REPEAT_CH7_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH7_POS)
#define SPU_ENV_REPEAT_CH8_POS                   (8)
#define SPU_ENV_REPEAT_CH8_MSK                   (0x1UL << SPU_ENV_REPEAT_CH8_POS)
#define SPU_ENV_REPEAT_CH8_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH8_POS)
#define SPU_ENV_REPEAT_CH8_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH8_POS)
#define SPU_ENV_REPEAT_CH9_POS                   (9)
#define SPU_ENV_REPEAT_CH9_MSK                   (0x1UL << SPU_ENV_REPEAT_CH9_POS)
#define SPU_ENV_REPEAT_CH9_ENABLE                (0x1UL << SPU_ENV_REPEAT_CH9_POS)
#define SPU_ENV_REPEAT_CH9_DISABLE               (0x0UL << SPU_ENV_REPEAT_CH9_POS)
#define SPU_ENV_REPEAT_CH10_POS                  (10)
#define SPU_ENV_REPEAT_CH10_MSK                  (0x1UL << SPU_ENV_REPEAT_CH10_POS)
#define SPU_ENV_REPEAT_CH10_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH10_POS)
#define SPU_ENV_REPEAT_CH10_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH10_POS)
#define SPU_ENV_REPEAT_CH11_POS                  (11)
#define SPU_ENV_REPEAT_CH11_MSK                  (0x1UL << SPU_ENV_REPEAT_CH11_POS)
#define SPU_ENV_REPEAT_CH11_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH11_POS)
#define SPU_ENV_REPEAT_CH11_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH11_POS)
#define SPU_ENV_REPEAT_CH12_POS                  (12)
#define SPU_ENV_REPEAT_CH12_MSK                  (0x1UL << SPU_ENV_REPEAT_CH12_POS)
#define SPU_ENV_REPEAT_CH12_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH12_POS)
#define SPU_ENV_REPEAT_CH12_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH12_POS)
#define SPU_ENV_REPEAT_CH13_POS                  (13)
#define SPU_ENV_REPEAT_CH13_MSK                  (0x1UL << SPU_ENV_REPEAT_CH13_POS)
#define SPU_ENV_REPEAT_CH13_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH13_POS)
#define SPU_ENV_REPEAT_CH13_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH13_POS)
#define SPU_ENV_REPEAT_CH14_POS                  (14)
#define SPU_ENV_REPEAT_CH14_MSK                  (0x1UL << SPU_ENV_REPEAT_CH14_POS)
#define SPU_ENV_REPEAT_CH14_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH14_POS)
#define SPU_ENV_REPEAT_CH14_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH14_POS)
#define SPU_ENV_REPEAT_CH15_POS                  (15)
#define SPU_ENV_REPEAT_CH15_MSK                  (0x1UL << SPU_ENV_REPEAT_CH15_POS)
#define SPU_ENV_REPEAT_CH15_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH15_POS)
#define SPU_ENV_REPEAT_CH15_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH15_POS)

/*
 * Bit definition for SPU->ENV_MODE_CH0_15[15:0]
 */
#define SPU_ENV_MODE_CH0_POS                     (0)
#define SPU_ENV_MODE_CH0_MSK                     (0x1UL << SPU_ENV_MODE_CH0_POS)
#define SPU_ENV_MODE_CH0_MANUAL                  (0x1UL << SPU_ENV_MODE_CH0_POS)
#define SPU_ENV_MODE_CH0_AUTO                    (0x0UL << SPU_ENV_MODE_CH0_POS)
#define SPU_ENV_MODE_CH1_POS                     (1)
#define SPU_ENV_MODE_CH1_MSK                     (0x1UL << SPU_ENV_MODE_CH1_POS)
#define SPU_ENV_MODE_CH1_MANUAL                  (0x1UL << SPU_ENV_MODE_CH1_POS)
#define SPU_ENV_MODE_CH1_AUTO                    (0x0UL << SPU_ENV_MODE_CH1_POS)
#define SPU_ENV_MODE_CH2_POS                     (2)
#define SPU_ENV_MODE_CH2_MSK                     (0x1UL << SPU_ENV_MODE_CH2_POS)
#define SPU_ENV_MODE_CH2_MANUAL                  (0x1UL << SPU_ENV_MODE_CH2_POS)
#define SPU_ENV_MODE_CH2_AUTO                    (0x0UL << SPU_ENV_MODE_CH2_POS)
#define SPU_ENV_MODE_CH3_POS                     (3)
#define SPU_ENV_MODE_CH3_MSK                     (0x1UL << SPU_ENV_MODE_CH3_POS)
#define SPU_ENV_MODE_CH3_MANUAL                  (0x1UL << SPU_ENV_MODE_CH3_POS)
#define SPU_ENV_MODE_CH3_AUTO                    (0x0UL << SPU_ENV_MODE_CH3_POS)
#define SPU_ENV_MODE_CH4_POS                     (4)
#define SPU_ENV_MODE_CH4_MSK                     (0x1UL << SPU_ENV_MODE_CH4_POS)
#define SPU_ENV_MODE_CH4_MANUAL                  (0x1UL << SPU_ENV_MODE_CH4_POS)
#define SPU_ENV_MODE_CH4_AUTO                    (0x0UL << SPU_ENV_MODE_CH4_POS)
#define SPU_ENV_MODE_CH5_POS                     (5)
#define SPU_ENV_MODE_CH5_MSK                     (0x1UL << SPU_ENV_MODE_CH5_POS)
#define SPU_ENV_MODE_CH5_MANUAL                  (0x1UL << SPU_ENV_MODE_CH5_POS)
#define SPU_ENV_MODE_CH5_AUTO                    (0x0UL << SPU_ENV_MODE_CH5_POS)
#define SPU_ENV_MODE_CH6_POS                     (6)
#define SPU_ENV_MODE_CH6_MSK                     (0x1UL << SPU_ENV_MODE_CH6_POS)
#define SPU_ENV_MODE_CH6_MANUAL                  (0x1UL << SPU_ENV_MODE_CH6_POS)
#define SPU_ENV_MODE_CH6_AUTO                    (0x0UL << SPU_ENV_MODE_CH6_POS)
#define SPU_ENV_MODE_CH7_POS                     (7)
#define SPU_ENV_MODE_CH7_MSK                     (0x1UL << SPU_ENV_MODE_CH7_POS)
#define SPU_ENV_MODE_CH7_MANUAL                  (0x1UL << SPU_ENV_MODE_CH7_POS)
#define SPU_ENV_MODE_CH7_AUTO                    (0x0UL << SPU_ENV_MODE_CH7_POS)
#define SPU_ENV_MODE_CH8_POS                     (8)
#define SPU_ENV_MODE_CH8_MSK                     (0x1UL << SPU_ENV_MODE_CH8_POS)
#define SPU_ENV_MODE_CH8_MANUAL                  (0x1UL << SPU_ENV_MODE_CH8_POS)
#define SPU_ENV_MODE_CH8_AUTO                    (0x0UL << SPU_ENV_MODE_CH8_POS)
#define SPU_ENV_MODE_CH9_POS                     (9)
#define SPU_ENV_MODE_CH9_MSK                     (0x1UL << SPU_ENV_MODE_CH9_POS)
#define SPU_ENV_MODE_CH9_MANUAL                  (0x1UL << SPU_ENV_MODE_CH9_POS)
#define SPU_ENV_MODE_CH9_AUTO                    (0x0UL << SPU_ENV_MODE_CH9_POS)
#define SPU_ENV_MODE_CH10_POS                    (10)
#define SPU_ENV_MODE_CH10_MSK                    (0x1UL << SPU_ENV_MODE_CH10_POS)
#define SPU_ENV_MODE_CH10_MANUAL                 (0x1UL << SPU_ENV_MODE_CH10_POS)
#define SPU_ENV_MODE_CH10_AUTO                   (0x0UL << SPU_ENV_MODE_CH10_POS)
#define SPU_ENV_MODE_CH11_POS                    (11)
#define SPU_ENV_MODE_CH11_MSK                    (0x1UL << SPU_ENV_MODE_CH11_POS)
#define SPU_ENV_MODE_CH11_MANUAL                 (0x1UL << SPU_ENV_MODE_CH11_POS)
#define SPU_ENV_MODE_CH11_AUTO                   (0x0UL << SPU_ENV_MODE_CH11_POS)
#define SPU_ENV_MODE_CH12_POS                    (12)
#define SPU_ENV_MODE_CH12_MSK                    (0x1UL << SPU_ENV_MODE_CH12_POS)
#define SPU_ENV_MODE_CH12_MANUAL                 (0x1UL << SPU_ENV_MODE_CH12_POS)
#define SPU_ENV_MODE_CH12_AUTO                   (0x0UL << SPU_ENV_MODE_CH12_POS)
#define SPU_ENV_MODE_CH13_POS                    (13)
#define SPU_ENV_MODE_CH13_MSK                    (0x1UL << SPU_ENV_MODE_CH13_POS)
#define SPU_ENV_MODE_CH13_MANUAL                 (0x1UL << SPU_ENV_MODE_CH13_POS)
#define SPU_ENV_MODE_CH13_AUTO                   (0x0UL << SPU_ENV_MODE_CH13_POS)
#define SPU_ENV_MODE_CH14_POS                    (14)
#define SPU_ENV_MODE_CH14_MSK                    (0x1UL << SPU_ENV_MODE_CH14_POS)
#define SPU_ENV_MODE_CH14_MANUAL                 (0x1UL << SPU_ENV_MODE_CH14_POS)
#define SPU_ENV_MODE_CH14_AUTO                   (0x0UL << SPU_ENV_MODE_CH14_POS)
#define SPU_ENV_MODE_CH15_POS                    (15)
#define SPU_ENV_MODE_CH15_MSK                    (0x1UL << SPU_ENV_MODE_CH15_POS)
#define SPU_ENV_MODE_CH15_MANUAL                 (0x1UL << SPU_ENV_MODE_CH15_POS)
#define SPU_ENV_MODE_CH15_AUTO                   (0x0UL << SPU_ENV_MODE_CH15_POS)

/*
 * Bit definition for SPU->TONE_RELEASE_CH0_15[15:0]
 */
#define SPU_TONE_RELEASE_CH0_POS                 (0)
#define SPU_TONE_RELEASE_CH0_MSK                 (0x1UL << SPU_TONE_RELEASE_CH0_POS)
#define SPU_TONE_RELEASE_CH0_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH0_POS)
#define SPU_TONE_RELEASE_CH0_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH0_POS)
#define SPU_TONE_RELEASE_CH1_POS                 (1)
#define SPU_TONE_RELEASE_CH1_MSK                 (0x1UL << SPU_TONE_RELEASE_CH1_POS)
#define SPU_TONE_RELEASE_CH1_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH1_POS)
#define SPU_TONE_RELEASE_CH1_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH1_POS)
#define SPU_TONE_RELEASE_CH2_POS                 (2)
#define SPU_TONE_RELEASE_CH2_MSK                 (0x1UL << SPU_TONE_RELEASE_CH2_POS)
#define SPU_TONE_RELEASE_CH2_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH2_POS)
#define SPU_TONE_RELEASE_CH2_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH2_POS)
#define SPU_TONE_RELEASE_CH3_POS                 (3)
#define SPU_TONE_RELEASE_CH3_MSK                 (0x1UL << SPU_TONE_RELEASE_CH3_POS)
#define SPU_TONE_RELEASE_CH3_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH3_POS)
#define SPU_TONE_RELEASE_CH3_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH3_POS)
#define SPU_TONE_RELEASE_CH4_POS                 (4)
#define SPU_TONE_RELEASE_CH4_MSK                 (0x1UL << SPU_TONE_RELEASE_CH4_POS)
#define SPU_TONE_RELEASE_CH4_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH4_POS)
#define SPU_TONE_RELEASE_CH4_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH4_POS)
#define SPU_TONE_RELEASE_CH5_POS                 (5)
#define SPU_TONE_RELEASE_CH5_MSK                 (0x1UL << SPU_TONE_RELEASE_CH5_POS)
#define SPU_TONE_RELEASE_CH5_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH5_POS)
#define SPU_TONE_RELEASE_CH5_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH5_POS)
#define SPU_TONE_RELEASE_CH6_POS                 (6)
#define SPU_TONE_RELEASE_CH6_MSK                 (0x1UL << SPU_TONE_RELEASE_CH6_POS)
#define SPU_TONE_RELEASE_CH6_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH6_POS)
#define SPU_TONE_RELEASE_CH6_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH6_POS)
#define SPU_TONE_RELEASE_CH7_POS                 (7)
#define SPU_TONE_RELEASE_CH7_MSK                 (0x1UL << SPU_TONE_RELEASE_CH7_POS)
#define SPU_TONE_RELEASE_CH7_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH7_POS)
#define SPU_TONE_RELEASE_CH7_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH7_POS)
#define SPU_TONE_RELEASE_CH8_POS                 (8)
#define SPU_TONE_RELEASE_CH8_MSK                 (0x1UL << SPU_TONE_RELEASE_CH8_POS)
#define SPU_TONE_RELEASE_CH8_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH8_POS)
#define SPU_TONE_RELEASE_CH8_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH8_POS)
#define SPU_TONE_RELEASE_CH9_POS                 (9)
#define SPU_TONE_RELEASE_CH9_MSK                 (0x1UL << SPU_TONE_RELEASE_CH9_POS)
#define SPU_TONE_RELEASE_CH9_ENABLE              (0x1UL << SPU_TONE_RELEASE_CH9_POS)
#define SPU_TONE_RELEASE_CH9_DISABLE             (0x0UL << SPU_TONE_RELEASE_CH9_POS)
#define SPU_TONE_RELEASE_CH10_POS                (10)
#define SPU_TONE_RELEASE_CH10_MSK                (0x1UL << SPU_TONE_RELEASE_CH10_POS)
#define SPU_TONE_RELEASE_CH10_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH10_POS)
#define SPU_TONE_RELEASE_CH10_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH10_POS)
#define SPU_TONE_RELEASE_CH11_POS                (11)
#define SPU_TONE_RELEASE_CH11_MSK                (0x1UL << SPU_TONE_RELEASE_CH11_POS)
#define SPU_TONE_RELEASE_CH11_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH11_POS)
#define SPU_TONE_RELEASE_CH11_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH11_POS)
#define SPU_TONE_RELEASE_CH12_POS                (12)
#define SPU_TONE_RELEASE_CH12_MSK                (0x1UL << SPU_TONE_RELEASE_CH12_POS)
#define SPU_TONE_RELEASE_CH12_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH12_POS)
#define SPU_TONE_RELEASE_CH12_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH12_POS)
#define SPU_TONE_RELEASE_CH13_POS                (13)
#define SPU_TONE_RELEASE_CH13_MSK                (0x1UL << SPU_TONE_RELEASE_CH13_POS)
#define SPU_TONE_RELEASE_CH13_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH13_POS)
#define SPU_TONE_RELEASE_CH13_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH13_POS)
#define SPU_TONE_RELEASE_CH14_POS                (14)
#define SPU_TONE_RELEASE_CH14_MSK                (0x1UL << SPU_TONE_RELEASE_CH14_POS)
#define SPU_TONE_RELEASE_CH14_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH14_POS)
#define SPU_TONE_RELEASE_CH14_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH14_POS)
#define SPU_TONE_RELEASE_CH15_POS                (15)
#define SPU_TONE_RELEASE_CH15_MSK                (0x1UL << SPU_TONE_RELEASE_CH15_POS)
#define SPU_TONE_RELEASE_CH15_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH15_POS)
#define SPU_TONE_RELEASE_CH15_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH15_POS)

/*
 * Bit definition for SPU->ENV_INT_STATUS_CH0_15[15:0]
 */
#define SPU_ENV_INT_FLAG_CH0_POS                 (0)
#define SPU_ENV_INT_FLAG_CH0_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH0_POS)
#define SPU_ENV_INT_FLAG_CH0                     (0x1UL << SPU_ENV_INT_FLAG_CH0_POS)
#define SPU_ENV_INT_FLAG_CH1_POS                 (1)
#define SPU_ENV_INT_FLAG_CH1_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH1_POS)
#define SPU_ENV_INT_FLAG_CH1                     (0x1UL << SPU_ENV_INT_FLAG_CH1_POS)
#define SPU_ENV_INT_FLAG_CH2_POS                 (2)
#define SPU_ENV_INT_FLAG_CH2_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH2_POS)
#define SPU_ENV_INT_FLAG_CH2                     (0x1UL << SPU_ENV_INT_FLAG_CH2_POS)
#define SPU_ENV_INT_FLAG_CH3_POS                 (3)
#define SPU_ENV_INT_FLAG_CH3_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH3_POS)
#define SPU_ENV_INT_FLAG_CH3                     (0x0UL << SPU_ENV_INT_FLAG_CH3_POS)
#define SPU_ENV_INT_FLAG_CH4_POS                 (4)
#define SPU_ENV_INT_FLAG_CH4_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH4_POS)
#define SPU_ENV_INT_FLAG_CH4                     (0x0UL << SPU_ENV_INT_FLAG_CH4_POS)
#define SPU_ENV_INT_FLAG_CH5_POS                 (5)
#define SPU_ENV_INT_FLAG_CH5_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH5_POS)
#define SPU_ENV_INT_FLAG_CH5                     (0x0UL << SPU_ENV_INT_FLAG_CH5_POS)
#define SPU_ENV_INT_FLAG_CH6_POS                 (6)
#define SPU_ENV_INT_FLAG_CH6_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH6_POS)
#define SPU_ENV_INT_FLAG_CH6                     (0x0UL << SPU_ENV_INT_FLAG_CH6_POS)
#define SPU_ENV_INT_FLAG_CH7_POS                 (7)
#define SPU_ENV_INT_FLAG_CH7_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH7_POS)
#define SPU_ENV_INT_FLAG_CH7                     (0x0UL << SPU_ENV_INT_FLAG_CH7_POS)
#define SPU_ENV_INT_FLAG_CH8_POS                 (8)
#define SPU_ENV_INT_FLAG_CH8_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH8_POS)
#define SPU_ENV_INT_FLAG_CH8                     (0x0UL << SPU_ENV_INT_FLAG_CH8_POS)
#define SPU_ENV_INT_FLAG_CH9_POS                 (9)
#define SPU_ENV_INT_FLAG_CH9_MSK                 (0x1UL << SPU_ENV_INT_FLAG_CH9_POS)
#define SPU_ENV_INT_FLAG_CH9                     (0x0UL << SPU_ENV_INT_FLAG_CH9_POS)
#define SPU_ENV_INT_FLAG_CH10_POS                (10)
#define SPU_ENV_INT_FLAG_CH10_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH10_POS)
#define SPU_ENV_INT_FLAG_CH10                    (0x0UL << SPU_ENV_INT_FLAG_CH10_POS)
#define SPU_ENV_INT_FLAG_CH11_POS                (11)
#define SPU_ENV_INT_FLAG_CH11_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH11_POS)
#define SPU_ENV_INT_FLAG_CH11                    (0x0UL << SPU_ENV_INT_FLAG_CH11_POS)
#define SPU_ENV_INT_FLAG_CH12_POS                (12)
#define SPU_ENV_INT_FLAG_CH12_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH12_POS)
#define SPU_ENV_INT_FLAG_CH12                    (0x0UL << SPU_ENV_INT_FLAG_CH12_POS)
#define SPU_ENV_INT_FLAG_CH13_POS                (13)
#define SPU_ENV_INT_FLAG_CH13_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH13_POS)
#define SPU_ENV_INT_FLAG_CH13                    (0x0UL << SPU_ENV_INT_FLAG_CH13_POS)
#define SPU_ENV_INT_FLAG_CH14_POS                (14)
#define SPU_ENV_INT_FLAG_CH14_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH14_POS)
#define SPU_ENV_INT_FLAG_CH14                    (0x0UL << SPU_ENV_INT_FLAG_CH14_POS)
#define SPU_ENV_INT_FLAG_CH15_POS                (15)
#define SPU_ENV_INT_FLAG_CH15_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH15_POS)
#define SPU_ENV_INT_FLAG_CH15                    (0x0UL << SPU_ENV_INT_FLAG_CH15_POS)

/*
 * Bit definition for SPU->PITCHBEND_EN_CH0_15[15:0]
 */
#define SPU_PITCHBEND_CH0_POS                    (0)
#define SPU_PITCHBEND_CH0_MSK                    (0x1UL << SPU_PITCHBEND_CH0_POS)
#define SPU_PITCHBEND_CH0_ENABLE                 (0x1UL << SPU_PITCHBEND_CH0_POS)
#define SPU_PITCHBEND_CH0_DISABLE                (0x0UL << SPU_PITCHBEND_CH0_POS)
#define SPU_PITCHBEND_CH1_POS                    (1)
#define SPU_PITCHBEND_CH1_MSK                    (0x1UL << SPU_PITCHBEND_CH1_POS)
#define SPU_PITCHBEND_CH1_ENABLE                 (0x1UL << SPU_PITCHBEND_CH1_POS)
#define SPU_PITCHBEND_CH1_DISABLE                (0x0UL << SPU_PITCHBEND_CH1_POS)
#define SPU_PITCHBEND_CH2_POS                    (2)
#define SPU_PITCHBEND_CH2_MSK                    (0x1UL << SPU_PITCHBEND_CH2_POS)
#define SPU_PITCHBEND_CH2_ENABLE                 (0x1UL << SPU_PITCHBEND_CH2_POS)
#define SPU_PITCHBEND_CH2_DISABLE                (0x0UL << SPU_PITCHBEND_CH2_POS)
#define SPU_PITCHBEND_CH3_POS                    (3)
#define SPU_PITCHBEND_CH3_MSK                    (0x1UL << SPU_PITCHBEND_CH3_POS)
#define SPU_PITCHBEND_CH3_ENABLE                 (0x1UL << SPU_PITCHBEND_CH3_POS)
#define SPU_PITCHBEND_CH3_DISABLE                (0x0UL << SPU_PITCHBEND_CH3_POS)
#define SPU_PITCHBEND_CH4_POS                    (4)
#define SPU_PITCHBEND_CH4_MSK                    (0x1UL << SPU_PITCHBEND_CH4_POS)
#define SPU_PITCHBEND_CH4_ENABLE                 (0x1UL << SPU_PITCHBEND_CH4_POS)
#define SPU_PITCHBEND_CH4_DISABLE                (0x0UL << SPU_PITCHBEND_CH4_POS)
#define SPU_PITCHBEND_CH5_POS                    (5)
#define SPU_PITCHBEND_CH5_MSK                    (0x1UL << SPU_PITCHBEND_CH5_POS)
#define SPU_PITCHBEND_CH5_ENABLE                 (0x1UL << SPU_PITCHBEND_CH5_POS)
#define SPU_PITCHBEND_CH5_DISABLE                (0x0UL << SPU_PITCHBEND_CH5_POS)
#define SPU_PITCHBEND_CH6_POS                    (6)
#define SPU_PITCHBEND_CH6_MSK                    (0x1UL << SPU_PITCHBEND_CH6_POS)
#define SPU_PITCHBEND_CH6_ENABLE                 (0x1UL << SPU_PITCHBEND_CH6_POS)
#define SPU_PITCHBEND_CH6_DISABLE                (0x0UL << SPU_PITCHBEND_CH6_POS)
#define SPU_PITCHBEND_CH7_POS                    (7)
#define SPU_PITCHBEND_CH7_MSK                    (0x1UL << SPU_PITCHBEND_CH7_POS)
#define SPU_PITCHBEND_CH7_ENABLE                 (0x1UL << SPU_PITCHBEND_CH7_POS)
#define SPU_PITCHBEND_CH7_DISABLE                (0x0UL << SPU_PITCHBEND_CH7_POS)
#define SPU_PITCHBEND_CH8_POS                    (8)
#define SPU_PITCHBEND_CH8_MSK                    (0x1UL << SPU_PITCHBEND_CH8_POS)
#define SPU_PITCHBEND_CH8_ENABLE                 (0x1UL << SPU_PITCHBEND_CH8_POS)
#define SPU_PITCHBEND_CH8_DISABLE                (0x0UL << SPU_PITCHBEND_CH8_POS)
#define SPU_PITCHBEND_CH9_POS                    (9)
#define SPU_PITCHBEND_CH9_MSK                    (0x1UL << SPU_PITCHBEND_CH9_POS)
#define SPU_PITCHBEND_CH9_ENABLE                 (0x1UL << SPU_PITCHBEND_CH9_POS)
#define SPU_PITCHBEND_CH9_DISABLE                (0x0UL << SPU_PITCHBEND_CH9_POS)
#define SPU_PITCHBEND_CH10_POS                   (10)
#define SPU_PITCHBEND_CH10_MSK                   (0x1UL << SPU_PITCHBEND_CH10_POS)
#define SPU_PITCHBEND_CH10_ENABLE                (0x1UL << SPU_PITCHBEND_CH10_POS)
#define SPU_PITCHBEND_CH10_DISABLE               (0x0UL << SPU_PITCHBEND_CH10_POS)
#define SPU_PITCHBEND_CH11_POS                   (11)
#define SPU_PITCHBEND_CH11_MSK                   (0x1UL << SPU_PITCHBEND_CH11_POS)
#define SPU_PITCHBEND_CH11_ENABLE                (0x1UL << SPU_PITCHBEND_CH11_POS)
#define SPU_PITCHBEND_CH11_DISABLE               (0x0UL << SPU_PITCHBEND_CH11_POS)
#define SPU_PITCHBEND_CH12_POS                   (12)
#define SPU_PITCHBEND_CH12_MSK                   (0x1UL << SPU_PITCHBEND_CH12_POS)
#define SPU_PITCHBEND_CH12_ENABLE                (0x1UL << SPU_PITCHBEND_CH12_POS)
#define SPU_PITCHBEND_CH12_DISABLE               (0x0UL << SPU_PITCHBEND_CH12_POS)
#define SPU_PITCHBEND_CH13_POS                   (13)
#define SPU_PITCHBEND_CH13_MSK                   (0x1UL << SPU_PITCHBEND_CH13_POS)
#define SPU_PITCHBEND_CH13_ENABLE                (0x1UL << SPU_PITCHBEND_CH13_POS)
#define SPU_PITCHBEND_CH13_DISABLE               (0x0UL << SPU_PITCHBEND_CH13_POS)
#define SPU_PITCHBEND_CH14_POS                   (14)
#define SPU_PITCHBEND_CH14_MSK                   (0x1UL << SPU_PITCHBEND_CH14_POS)
#define SPU_PITCHBEND_CH14_ENABLE                (0x1UL << SPU_PITCHBEND_CH14_POS)
#define SPU_PITCHBEND_CH14_DISABLE               (0x0UL << SPU_PITCHBEND_CH14_POS)
#define SPU_PITCHBEND_CH15_POS                   (15)
#define SPU_PITCHBEND_CH15_MSK                   (0x1UL << SPU_PITCHBEND_CH15_POS)
#define SPU_PITCHBEND_CH15_ENABLE                (0x1UL << SPU_PITCHBEND_CH15_POS)
#define SPU_PITCHBEND_CH15_DISABLE               (0x0UL << SPU_PITCHBEND_CH15_POS)

/*
 * Bit definition for SPU->ATTACK_RELEASE_TIME[15:8]
 */
#define SPU_COMPRESSOR_ATTACK_TIME_POS           (8)
#define SPU_COMPRESSOR_ATTACK_TIME_MSK           (0xFFUL << SPU_COMPRESSOR_ATTACK_TIME_POS)
#define SPU_COMPRESSOR_ATTACK_TIME_0x00          (0x00UL << SPU_COMPRESSOR_ATTACK_TIME_POS)
#define SPU_COMPRESSOR_ATTACK_TIME_0xFF          (0xFFUL << SPU_COMPRESSOR_ATTACK_TIME_POS)
#define SPU_COMPRESSOR_ATTACK_TIME_0x10          (0x10UL << SPU_COMPRESSOR_ATTACK_TIME_POS)

/*
 * Bit definition for SPU->ATTACK_RELEASE_TIME[7:0]
 */
#define SPU_COMPRESSOR_RELEASE_TIME_POS          (0)
#define SPU_COMPRESSOR_RELEASE_TIME_MSK          (0xFFUL << SPU_COMPRESSOR_RELEASE_TIME_POS)
#define SPU_COMPRESSOR_RELEASE_TIME_0x00         (0x00UL << SPU_COMPRESSOR_RELEASE_TIME_POS)
#define SPU_COMPRESSOR_RELEASE_TIME_0xFF         (0xFFUL << SPU_COMPRESSOR_RELEASE_TIME_POS)
#define SPU_COMPRESSOR_RELEASE_TIME_0x10         (0x10UL << SPU_COMPRESSOR_RELEASE_TIME_POS)

/*
 * Bit definition for SPU->SPU_EN_CH16_31[15:0]
 */
#define SPU_CH16_EN_POS                          (0)
#define SPU_CH16_EN_MSK                          (0x1UL << SPU_CH16_EN_POS)
#define SPU_CH16_ENABLE                          (0x1UL << SPU_CH16_EN_POS)
#define SPU_CH16_DISABLE                         (0x0UL << SPU_CH16_EN_POS)
#define SPU_CH17_EN_POS                          (1)
#define SPU_CH17_EN_MSK                          (0x1UL << SPU_CH17_EN_POS)
#define SPU_CH17_ENABLE                          (0x1UL << SPU_CH17_EN_POS)
#define SPU_CH17_DISABLE                         (0x0UL << SPU_CH17_EN_POS)
#define SPU_CH18_EN_POS                          (2)
#define SPU_CH18_EN_MSK                          (0x1UL << SPU_CH18_EN_POS)
#define SPU_CH18_ENABLE                          (0x1UL << SPU_CH18_EN_POS)
#define SPU_CH18_DISABLE                         (0x0UL << SPU_CH18_EN_POS)
#define SPU_CH19_EN_POS                          (3)
#define SPU_CH19_EN_MSK                          (0x1UL << SPU_CH19_EN_POS)
#define SPU_CH19_ENABLE                          (0x1UL << SPU_CH19_EN_POS)
#define SPU_CH19_DISABLE                         (0x0UL << SPU_CH19_EN_POS)
#define SPU_CH20_EN_POS                          (4)
#define SPU_CH20_EN_MSK                          (0x1UL << SPU_CH20_EN_POS)
#define SPU_CH20_ENABLE                          (0x1UL << SPU_CH20_EN_POS)
#define SPU_CH20_DISABLE                         (0x0UL << SPU_CH20_EN_POS)
#define SPU_CH21_EN_POS                          (5)
#define SPU_CH21_EN_MSK                          (0x1UL << SPU_CH21_EN_POS)
#define SPU_CH21_ENABLE                          (0x1UL << SPU_CH21_EN_POS)
#define SPU_CH21_DISABLE                         (0x0UL << SPU_CH21_EN_POS)
#define SPU_CH22_EN_POS                          (6)
#define SPU_CH22_EN_MSK                          (0x1UL << SPU_CH22_EN_POS)
#define SPU_CH22_ENABLE                          (0x1UL << SPU_CH22_EN_POS)
#define SPU_CH22_DISABLE                         (0x0UL << SPU_CH22_EN_POS)
#define SPU_CH23_EN_POS                          (7)
#define SPU_CH23_EN_MSK                          (0x1UL << SPU_CH23_EN_POS)
#define SPU_CH23_ENABLE                          (0x1UL << SPU_CH23_EN_POS)
#define SPU_CH23_DISABLE                         (0x0UL << SPU_CH23_EN_POS)
#define SPU_CH24_EN_POS                          (8)
#define SPU_CH24_EN_MSK                          (0x1UL << SPU_CH24_EN_POS)
#define SPU_CH24_ENABLE                          (0x1UL << SPU_CH24_EN_POS)
#define SPU_CH24_DISABLE                         (0x0UL << SPU_CH24_EN_POS)
#define SPU_CH25_EN_POS                          (9)
#define SPU_CH25_EN_MSK                          (0x1UL << SPU_CH25_EN_POS)
#define SPU_CH25_ENABLE                          (0x1UL << SPU_CH25_EN_POS)
#define SPU_CH25_DISABLE                         (0x0UL << SPU_CH25_EN_POS)
#define SPU_CH26_EN_POS                          (10)
#define SPU_CH26_EN_MSK                          (0x1UL << SPU_CH26_EN_POS)
#define SPU_CH26_ENABLE                          (0x1UL << SPU_CH26_EN_POS)
#define SPU_CH26_DISABLE                         (0x0UL << SPU_CH26_EN_POS)
#define SPU_CH27_EN_POS                          (11)
#define SPU_CH27_EN_MSK                          (0x1UL << SPU_CH27_EN_POS)
#define SPU_CH27_ENABLE                          (0x1UL << SPU_CH27_EN_POS)
#define SPU_CH27_DISABLE                         (0x0UL << SPU_CH27_EN_POS)
#define SPU_CH28_EN_POS                          (12)
#define SPU_CH28_EN_MSK                          (0x1UL << SPU_CH28_EN_POS)
#define SPU_CH28_ENABLE                          (0x1UL << SPU_CH28_EN_POS)
#define SPU_CH28_DISABLE                         (0x0UL << SPU_CH28_EN_POS)
#define SPU_CH29_EN_POS                          (13)
#define SPU_CH29_EN_MSK                          (0x1UL << SPU_CH29_EN_POS)
#define SPU_CH29_ENABLE                          (0x1UL << SPU_CH29_EN_POS)
#define SPU_CH29_DISABLE                         (0x0UL << SPU_CH29_EN_POS)
#define SPU_CH30_EN_POS                          (14)
#define SPU_CH30_EN_MSK                          (0x1UL << SPU_CH30_EN_POS)
#define SPU_CH30_ENABLE                          (0x1UL << SPU_CH30_EN_POS)
#define SPU_CH30_DISABLE                         (0x0UL << SPU_CH30_EN_POS)
#define SPU_CH31_EN_POS                          (15)
#define SPU_CH31_EN_MSK                          (0x1UL << SPU_CH31_EN_POS)
#define SPU_CH31_ENABLE                          (0x1UL << SPU_CH31_EN_POS)
#define SPU_CH31_DISABLE                         (0x0UL << SPU_CH31_EN_POS)

/*
 * Bit definition for SPU->INT_EN_CH16_31[15:0] - SPU Channel FIQ Enable
 */
#define SPU_INT_CH16_EN_POS                      (0)
#define SPU_INT_CH16_EN_MSK                      (0x1UL << SPU_INT_CH16_EN_POS)
#define SPU_INT_CH16_ENABLE                      (0x1UL << SPU_INT_CH16_EN_POS)
#define SPU_INT_CH16_DISABLE                     (0x0UL << SPU_INT_CH16_EN_POS)
#define SPU_INT_CH17_EN_POS                      (1)
#define SPU_INT_CH17_EN_MSK                      (0x1UL << SPU_INT_CH17_EN_POS)
#define SPU_INT_CH17_ENABLE                      (0x1UL << SPU_INT_CH17_EN_POS)
#define SPU_INT_CH17_DISABLE                     (0x0UL << SPU_INT_CH17_EN_POS)
#define SPU_INT_CH18_EN_POS                      (2)
#define SPU_INT_CH18_EN_MSK                      (0x1UL << SPU_INT_CH18_EN_POS)
#define SPU_INT_CH18_ENABLE                      (0x1UL << SPU_INT_CH18_EN_POS)
#define SPU_INT_CH18_DISABLE                     (0x0UL << SPU_INT_CH18_EN_POS)
#define SPU_INT_CH19_EN_POS                      (3)
#define SPU_INT_CH19_EN_MSK                      (0x1UL << SPU_INT_CH19_EN_POS)
#define SPU_INT_CH19_ENABLE                      (0x1UL << SPU_INT_CH19_EN_POS)
#define SPU_INT_CH19_DISABLE                     (0x0UL << SPU_INT_CH19_EN_POS)
#define SPU_INT_CH20_EN_POS                      (4)
#define SPU_INT_CH20_EN_MSK                      (0x1UL << SPU_INT_CH20_EN_POS)
#define SPU_INT_CH20_ENABLE                      (0x1UL << SPU_INT_CH20_EN_POS)
#define SPU_INT_CH20_DISABLE                     (0x0UL << SPU_INT_CH20_EN_POS)
#define SPU_INT_CH21_EN_POS                      (5)
#define SPU_INT_CH21_EN_MSK                      (0x1UL << SPU_INT_CH21_EN_POS)
#define SPU_INT_CH21_ENABLE                      (0x1UL << SPU_INT_CH21_EN_POS)
#define SPU_INT_CH21_DISABLE                     (0x0UL << SPU_INT_CH21_EN_POS)
#define SPU_INT_CH22_EN_POS                      (6)
#define SPU_INT_CH22_EN_MSK                      (0x1UL << SPU_INT_CH22_EN_POS)
#define SPU_INT_CH22_ENABLE                      (0x1UL << SPU_INT_CH22_EN_POS)
#define SPU_INT_CH22_DISABLE                     (0x0UL << SPU_INT_CH22_EN_POS)
#define SPU_INT_CH23_EN_POS                      (7)
#define SPU_INT_CH23_EN_MSK                      (0x1UL << SPU_INT_CH23_EN_POS)
#define SPU_INT_CH23_ENABLE                      (0x1UL << SPU_INT_CH23_EN_POS)
#define SPU_INT_CH23_DISABLE                     (0x0UL << SPU_INT_CH23_EN_POS)
#define SPU_INT_CH24_EN_POS                      (8)
#define SPU_INT_CH24_EN_MSK                      (0x1UL << SPU_INT_CH24_EN_POS)
#define SPU_INT_CH24_ENABLE                      (0x1UL << SPU_INT_CH24_EN_POS)
#define SPU_INT_CH24_DISABLE                     (0x0UL << SPU_INT_CH24_EN_POS)
#define SPU_INT_CH25_EN_POS                      (9)
#define SPU_INT_CH25_EN_MSK                      (0x1UL << SPU_INT_CH25_EN_POS)
#define SPU_INT_CH25_ENABLE                      (0x1UL << SPU_INT_CH25_EN_POS)
#define SPU_INT_CH25_DISABLE                     (0x0UL << SPU_INT_CH25_EN_POS)
#define SPU_INT_CH26_EN_POS                      (10)
#define SPU_INT_CH26_EN_MSK                      (0x1UL << SPU_INT_CH26_EN_POS)
#define SPU_INT_CH26_ENABLE                      (0x1UL << SPU_INT_CH26_EN_POS)
#define SPU_INT_CH26_DISABLE                     (0x0UL << SPU_INT_CH26_EN_POS)
#define SPU_INT_CH27_EN_POS                      (11)
#define SPU_INT_CH27_EN_MSK                      (0x1UL << SPU_INT_CH27_EN_POS)
#define SPU_INT_CH27_ENABLE                      (0x1UL << SPU_INT_CH27_EN_POS)
#define SPU_INT_CH27_DISABLE                     (0x0UL << SPU_INT_CH27_EN_POS)
#define SPU_INT_CH28_EN_POS                      (12)
#define SPU_INT_CH28_EN_MSK                      (0x1UL << SPU_INT_CH28_EN_POS)
#define SPU_INT_CH28_ENABLE                      (0x1UL << SPU_INT_CH28_EN_POS)
#define SPU_INT_CH28_DISABLE                     (0x0UL << SPU_INT_CH28_EN_POS)
#define SPU_INT_CH29_EN_POS                      (13)
#define SPU_INT_CH29_EN_MSK                      (0x1UL << SPU_INT_CH29_EN_POS)
#define SPU_INT_CH29_ENABLE                      (0x1UL << SPU_INT_CH29_EN_POS)
#define SPU_INT_CH29_DISABLE                     (0x0UL << SPU_INT_CH29_EN_POS)
#define SPU_INT_CH30_EN_POS                      (14)
#define SPU_INT_CH30_EN_MSK                      (0x1UL << SPU_INT_CH30_EN_POS)
#define SPU_INT_CH30_ENABLE                      (0x1UL << SPU_INT_CH30_EN_POS)
#define SPU_INT_CH30_DISABLE                     (0x0UL << SPU_INT_CH30_EN_POS)
#define SPU_INT_CH31_EN_POS                      (15)
#define SPU_INT_CH31_EN_MSK                      (0x1UL << SPU_INT_CH31_EN_POS)
#define SPU_INT_CH31_ENABLE                      (0x1UL << SPU_INT_CH31_EN_POS)
#define SPU_INT_CH31_DISABLE                     (0x0UL << SPU_INT_CH31_EN_POS)

/*
 * Bit definition for INT_STATUS_CH16_31[15:0] - SPU Channel INT Status
 */
#define SPU_INT_FLAG_CH16_POS                    (0)
#define SPU_INT_FLAG_CH16_MSK                    (0x1UL << SPU_INT_FLAG_CH16_POS)
#define SPU_INT_FLAG_CH16                        (0x1UL << SPU_INT_FLAG_CH16_POS)
#define SPU_INT_FLAG_CH17_POS                    (1)
#define SPU_INT_FLAG_CH17_MSK                    (0x1UL << SPU_INT_FLAG_CH17_POS)
#define SPU_INT_FLAG_CH17                        (0x1UL << SPU_INT_FLAG_CH17_POS)
#define SPU_INT_FLAG_CH18_POS                    (2)
#define SPU_INT_FLAG_CH18_MSK                    (0x1UL << SPU_INT_FLAG_CH18_POS)
#define SPU_INT_FLAG_CH18                        (0x1UL << SPU_INT_FLAG_CH18_POS)
#define SPU_INT_FLAG_CH19_POS                    (3)
#define SPU_INT_FLAG_CH19_MSK                    (0x1UL << SPU_INT_FLAG_CH19_POS)
#define SPU_INT_FLAG_CH19                        (0x0UL << SPU_INT_FLAG_CH19_POS)
#define SPU_INT_FLAG_CH20_POS                    (4)
#define SPU_INT_FLAG_CH20_MSK                    (0x1UL << SPU_INT_FLAG_CH20_POS)
#define SPU_INT_FLAG_CH20                        (0x0UL << SPU_INT_FLAG_CH20_POS)
#define SPU_INT_FLAG_CH21_POS                    (5)
#define SPU_INT_FLAG_CH21_MSK                    (0x1UL << SPU_INT_FLAG_CH21_POS)
#define SPU_INT_FLAG_CH21                        (0x0UL << SPU_INT_FLAG_CH21_POS)
#define SPU_INT_FLAG_CH22_POS                    (6)
#define SPU_INT_FLAG_CH22_MSK                    (0x1UL << SPU_INT_FLAG_CH22_POS)
#define SPU_INT_FLAG_CH22                        (0x0UL << SPU_INT_FLAG_CH22_POS)
#define SPU_INT_FLAG_CH23_POS                    (7)
#define SPU_INT_FLAG_CH23_MSK                    (0x1UL << SPU_INT_FLAG_CH23_POS)
#define SPU_INT_FLAG_CH23                        (0x0UL << SPU_INT_FLAG_CH23_POS)
#define SPU_INT_FLAG_CH24_POS                    (8)
#define SPU_INT_FLAG_CH24_MSK                    (0x1UL << SPU_INT_FLAG_CH24_POS)
#define SPU_INT_FLAG_CH24                        (0x0UL << SPU_INT_FLAG_CH24_POS)
#define SPU_INT_FLAG_CH25_POS                    (9)
#define SPU_INT_FLAG_CH25_MSK                    (0x1UL << SPU_INT_FLAG_CH25_POS)
#define SPU_INT_FLAG_CH25                        (0x0UL << SPU_INT_FLAG_CH25_POS)
#define SPU_INT_FLAG_CH26_POS                    (10)
#define SPU_INT_FLAG_CH26_MSK                    (0x1UL << SPU_INT_FLAG_CH26_POS)
#define SPU_INT_FLAG_CH26                        (0x0UL << SPU_INT_FLAG_CH26_POS)
#define SPU_INT_FLAG_CH27_POS                    (11)
#define SPU_INT_FLAG_CH27_MSK                    (0x1UL << SPU_INT_FLAG_CH27_POS)
#define SPU_INT_FLAG_CH27                        (0x0UL << SPU_INT_FLAG_CH27_POS)
#define SPU_INT_FLAG_CH28_POS                    (12)
#define SPU_INT_FLAG_CH28_MSK                    (0x1UL << SPU_INT_FLAG_CH28_POS)
#define SPU_INT_FLAG_CH28                        (0x0UL << SPU_INT_FLAG_CH28_POS)
#define SPU_INT_FLAG_CH29_POS                    (13)
#define SPU_INT_FLAG_CH29_MSK                    (0x1UL << SPU_INT_FLAG_CH29_POS)
#define SPU_INT_FLAG_CH29                        (0x0UL << SPU_INT_FLAG_CH29_POS)
#define SPU_INT_FLAG_CH30_POS                    (14)
#define SPU_INT_FLAG_CH30_MSK                    (0x1UL << SPU_INT_FLAG_CH30_POS)
#define SPU_INT_FLAG_CH30                        (0x0UL << SPU_INT_FLAG_CH30_POS)
#define SPU_INT_FLAG_CH31_POS                    (15)
#define SPU_INT_FLAG_CH31_MSK                    (0x1UL << SPU_INT_FLAG_CH31_POS)
#define SPU_INT_FLAG_CH31                        (0x0UL << SPU_INT_FLAG_CH31_POS)

/*
 * Bit definition for SPU->POSTWAVE_CTRL[15]
 */
#define SPU_POSTWAVE_DMA_EN_POS                  (15)
#define SPU_POSTWAVE_DMA_EN_MSK                  (0x1UL << SPU_POSTWAVE_DMA_EN_POS)
#define SPU_POSTWAVE_DMA_ENABLE                  (0x1UL << SPU_POSTWAVE_DMA_EN_POS)
#define SPU_POSTWAVE_DMA_DISABLE                 (0x0UL << SPU_POSTWAVE_DMA_EN_POS)

/*
 * Bit definition for SPU->POSTWAVE_CTRL[14]
 */
#define SPU_POSTWAVE_DIRECTOUT_EN_POS            (14)
#define SPU_POSTWAVE_DIRECTOUT_EN_MSK            (0x1UL << SPU_POSTWAVE_DIRECTOUT_EN_POS)
#define SPU_POSTWAVE_DIRECTOUT_ENABLE            (0x1UL << SPU_POSTWAVE_DIRECTOUT_EN_POS)
#define SPU_POSTWAVE_DIRECTOUT_DISABLE           (0x0UL << SPU_POSTWAVE_DIRECTOUT_EN_POS)

/*
 * Bit definition for SPU->POSTWAVE_CTRL[9:8]
 */
#define SPU_POSTWAVE_DMA_DOWNSAMPLE_POS          (8)
#define SPU_POSTWAVE_DMA_DOWNSAMPLE_MSK          (0x3UL << SPU_POSTWAVE_DMA_DOWNSAMPLE_POS)
#define SPU_POSTWAVE_DMA_DOWNSAMPLE_DIV1         (0x0UL << SPU_POSTWAVE_DMA_DOWNSAMPLE_POS)
#define SPU_POSTWAVE_DMA_DOWNSAMPLE_DIV6         (0x1UL << SPU_POSTWAVE_DMA_DOWNSAMPLE_POS)
#define SPU_POSTWAVE_DMA_DOWNSAMPLE_DIV12        (0x2UL << SPU_POSTWAVE_DMA_DOWNSAMPLE_POS)
#define SPU_POSTWAVE_DMA_DOWNSAMPLE_DIV31        (0x3UL << SPU_POSTWAVE_DMA_DOWNSAMPLE_POS)

/*
 * Bit definition for SPU->POSTWAVE_CTRL[7]
 */
#define SPU_POSTWAVE_FMT_POS                     (7)
#define SPU_POSTWAVE_FMT_MSK                     (0x1UL << SPU_POSTWAVE_FMT_POS)
#define SPU_POSTWAVE_FMT_SIGN                    (0x1UL << SPU_POSTWAVE_FMT_POS)
#define SPU_POSTWAVE_FMT_UNSIGN                  (0x0UL << SPU_POSTWAVE_FMT_POS)

/*
 * Bit definition for SPU->POSTWAVE_CTRL[6]
 */
#define SPU_POSTWAVE_SILENCE_EN_POS              (6)
#define SPU_POSTWAVE_SILENCE_EN_MSK              (0x1UL << SPU_POSTWAVE_SILENCE_EN_POS)
#define SPU_POSTWAVE_SILENCE_ENABLE              (0x1UL << SPU_POSTWAVE_SILENCE_EN_POS)
#define SPU_POSTWAVE_SILENCE_DISABLE             (0x0UL << SPU_POSTWAVE_SILENCE_EN_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH16_3[3:0] -
 */
#define SPU_ENV_CLK_CH16_POS                     (0)
#define SPU_ENV_CLK_CH16_MSK                     (0xFUL << SPU_ENV_CLK_CH16_POS)
#define SPU_ENV_CLK_CH16_0x0                     (0x0UL << SPU_ENV_CLK_CH16_POS)
#define SPU_ENV_CLK_CH16_0xF                     (0xFUL << SPU_ENV_CLK_CH16_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH16_3[7:4] -
 */
#define SPU_ENV_CLK_CH17_POS                     (4)
#define SPU_ENV_CLK_CH17_MSK                     (0xFUL << SPU_ENV_CLK_CH17_POS)
#define SPU_ENV_CLK_CH17_0x0                     (0x0UL << SPU_ENV_CLK_CH17_POS)
#define SPU_ENV_CLK_CH17_0xF                     (0xFUL << SPU_ENV_CLK_CH17_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH16_3[11:8] -
 */
#define SPU_ENV_CLK_CH18_POS                     (8)
#define SPU_ENV_CLK_CH18_MSK                     (0xFUL << SPU_ENV_CLK_CH18_POS)
#define SPU_ENV_CLK_CH18_0x0                     (0x0UL << SPU_ENV_CLK_CH18_POS)
#define SPU_ENV_CLK_CH18_0xF                     (0xFUL << SPU_ENV_CLK_CH18_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH16_3[15:12] -
 */
#define SPU_ENV_CLK_CH19_POS                     (12)
#define SPU_ENV_CLK_CH19_MSK                     (0xFUL << SPU_ENV_CLK_CH19_POS)
#define SPU_ENV_CLK_CH19_0x0                     (0x0UL << SPU_ENV_CLK_CH19_POS)
#define SPU_ENV_CLK_CH19_0xF                     (0xFUL << SPU_ENV_CLK_CH19_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH20_7[3:0] -
 */
#define SPU_ENV_CLK_CH20_POS                     (0)
#define SPU_ENV_CLK_CH20_MSK                     (0xFUL << SPU_ENV_CLK_CH20_POS)
#define SPU_ENV_CLK_CH20_0x0                     (0x0UL << SPU_ENV_CLK_CH20_POS)
#define SPU_ENV_CLK_CH20_0xF                     (0xFUL << SPU_ENV_CLK_CH20_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH20_7[7:4] -
 */
#define SPU_ENV_CLK_CH21_POS                     (4)
#define SPU_ENV_CLK_CH21_MSK                     (0xFUL << SPU_ENV_CLK_CH21_POS)
#define SPU_ENV_CLK_CH21_0x0                     (0x0UL << SPU_ENV_CLK_CH21_POS)
#define SPU_ENV_CLK_CH21_0xF                     (0xFUL << SPU_ENV_CLK_CH21_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH20_7[11:8] -
 */
#define SPU_ENV_CLK_CH22_POS                     (8)
#define SPU_ENV_CLK_CH22_MSK                     (0xFUL << SPU_ENV_CLK_CH22_POS)
#define SPU_ENV_CLK_CH22_0x0                     (0x0UL << SPU_ENV_CLK_CH22_POS)
#define SPU_ENV_CLK_CH22_0xF                     (0xFUL << SPU_ENV_CLK_CH22_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH20_7[15:12] -
 */
#define SPU_ENV_CLK_CH23_POS                     (12)
#define SPU_ENV_CLK_CH23_MSK                     (0xFUL << SPU_ENV_CLK_CH23_POS)
#define SPU_ENV_CLK_CH23_0x0                     (0x0UL << SPU_ENV_CLK_CH23_POS)
#define SPU_ENV_CLK_CH23_0xF                     (0xFUL << SPU_ENV_CLK_CH23_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH24_11[3:0] -
 */
#define SPU_ENV_CLK_CH24_POS                     (0)
#define SPU_ENV_CLK_CH24_MSK                     (0xFUL << SPU_ENV_CLK_CH24_POS)
#define SPU_ENV_CLK_CH24_0x0                     (0x0UL << SPU_ENV_CLK_CH24_POS)
#define SPU_ENV_CLK_CH24_0xF                     (0xFUL << SPU_ENV_CLK_CH24_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH24_11[7:4] -
 */
#define SPU_ENV_CLK_CH25_POS                     (4)
#define SPU_ENV_CLK_CH25_MSK                     (0xFUL << SPU_ENV_CLK_CH25_POS)
#define SPU_ENV_CLK_CH25_0x0                     (0x0UL << SPU_ENV_CLK_CH25_POS)
#define SPU_ENV_CLK_CH25_0xF                     (0xFUL << SPU_ENV_CLK_CH25_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH24_11[11:8] -
 */
#define SPU_ENV_CLK_CH26_POS                     (8)
#define SPU_ENV_CLK_CH26_MSK                     (0xFUL << SPU_ENV_CLK_CH26_POS)
#define SPU_ENV_CLK_CH26_0x0                     (0x0UL << SPU_ENV_CLK_CH26_POS)
#define SPU_ENV_CLK_CH26_0xF                     (0xFUL << SPU_ENV_CLK_CH26_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH24_11[15:12] -
 */
#define SPU_ENV_CLK_CH27_POS                     (12)
#define SPU_ENV_CLK_CH27_MSK                     (0xFUL << SPU_ENV_CLK_CH27_POS)
#define SPU_ENV_CLK_CH27_0x0                     (0x0UL << SPU_ENV_CLK_CH27_POS)
#define SPU_ENV_CLK_CH27_0xF                     (0xFUL << SPU_ENV_CLK_CH27_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH28_15[3:0] -
 */
#define SPU_ENV_CLK_CH28_POS                     (0)
#define SPU_ENV_CLK_CH28_MSK                     (0xFUL << SPU_ENV_CLK_CH28_POS)
#define SPU_ENV_CLK_CH28_0x0                     (0x0UL << SPU_ENV_CLK_CH28_POS)
#define SPU_ENV_CLK_CH28_0xF                     (0xFUL << SPU_ENV_CLK_CH28_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH28_15[7:4] -
 */
#define SPU_ENV_CLK_CH29_POS                     (4)
#define SPU_ENV_CLK_CH29_MSK                     (0xFUL << SPU_ENV_CLK_CH29_POS)
#define SPU_ENV_CLK_CH29_0x0                     (0x0UL << SPU_ENV_CLK_CH29_POS)
#define SPU_ENV_CLK_CH29_0xF                     (0xFUL << SPU_ENV_CLK_CH29_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH28_15[11:8] -
 */
#define SPU_ENV_CLK_CH30_POS                     (8)
#define SPU_ENV_CLK_CH30_MSK                     (0xFUL << SPU_ENV_CLK_CH30_POS)
#define SPU_ENV_CLK_CH30_0x0                     (0x0UL << SPU_ENV_CLK_CH30_POS)
#define SPU_ENV_CLK_CH30_0xF                     (0xFUL << SPU_ENV_CLK_CH30_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH28_15[15:12] -
 */
#define SPU_ENV_CLK_CH31_POS                     (12)
#define SPU_ENV_CLK_CH31_MSK                     (0xFUL << SPU_ENV_CLK_CH31_POS)
#define SPU_ENV_CLK_CH31_0x0                     (0x0UL << SPU_ENV_CLK_CH31_POS)
#define SPU_ENV_CLK_CH31_0xF                     (0xFUL << SPU_ENV_CLK_CH31_POS)

/*
 * Bit definition for SPU->ENV_RAMPDOWN_CH16_31[15:0]
 */
#define SPU_ENV_RAMPDOWN_CH16_POS                (0)
#define SPU_ENV_RAMPDOWN_CH16_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH16_POS)
#define SPU_ENV_RAMPDOWN_CH16_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH16_POS)
#define SPU_ENV_RAMPDOWN_CH16_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH16_POS)
#define SPU_ENV_RAMPDOWN_CH17_POS                (1)
#define SPU_ENV_RAMPDOWN_CH17_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH17_POS)
#define SPU_ENV_RAMPDOWN_CH17_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH17_POS)
#define SPU_ENV_RAMPDOWN_CH17_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH17_POS)
#define SPU_ENV_RAMPDOWN_CH18_POS                (2)
#define SPU_ENV_RAMPDOWN_CH18_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH18_POS)
#define SPU_ENV_RAMPDOWN_CH18_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH18_POS)
#define SPU_ENV_RAMPDOWN_CH18_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH18_POS)
#define SPU_ENV_RAMPDOWN_CH19_POS                (3)
#define SPU_ENV_RAMPDOWN_CH19_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH19_POS)
#define SPU_ENV_RAMPDOWN_CH19_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH19_POS)
#define SPU_ENV_RAMPDOWN_CH19_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH19_POS)
#define SPU_ENV_RAMPDOWN_CH20_POS                (4)
#define SPU_ENV_RAMPDOWN_CH20_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH20_POS)
#define SPU_ENV_RAMPDOWN_CH20_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH20_POS)
#define SPU_ENV_RAMPDOWN_CH20_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH20_POS)
#define SPU_ENV_RAMPDOWN_CH21_POS                (5)
#define SPU_ENV_RAMPDOWN_CH21_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH21_POS)
#define SPU_ENV_RAMPDOWN_CH21_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH21_POS)
#define SPU_ENV_RAMPDOWN_CH21_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH21_POS)
#define SPU_ENV_RAMPDOWN_CH22_POS                (6)
#define SPU_ENV_RAMPDOWN_CH22_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH22_POS)
#define SPU_ENV_RAMPDOWN_CH22_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH22_POS)
#define SPU_ENV_RAMPDOWN_CH22_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH22_POS)
#define SPU_ENV_RAMPDOWN_CH23_POS                (7)
#define SPU_ENV_RAMPDOWN_CH23_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH23_POS)
#define SPU_ENV_RAMPDOWN_CH23_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH23_POS)
#define SPU_ENV_RAMPDOWN_CH23_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH23_POS)
#define SPU_ENV_RAMPDOWN_CH24_POS                (8)
#define SPU_ENV_RAMPDOWN_CH24_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH24_POS)
#define SPU_ENV_RAMPDOWN_CH24_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH24_POS)
#define SPU_ENV_RAMPDOWN_CH24_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH24_POS)
#define SPU_ENV_RAMPDOWN_CH25_POS                (9)
#define SPU_ENV_RAMPDOWN_CH25_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH25_POS)
#define SPU_ENV_RAMPDOWN_CH25_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH25_POS)
#define SPU_ENV_RAMPDOWN_CH25_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH25_POS)
#define SPU_ENV_RAMPDOWN_CH26_POS                (10)
#define SPU_ENV_RAMPDOWN_CH26_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH26_POS)
#define SPU_ENV_RAMPDOWN_CH26_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH26_POS)
#define SPU_ENV_RAMPDOWN_CH26_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH26_POS)
#define SPU_ENV_RAMPDOWN_CH27_POS                (11)
#define SPU_ENV_RAMPDOWN_CH27_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH27_POS)
#define SPU_ENV_RAMPDOWN_CH27_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH27_POS)
#define SPU_ENV_RAMPDOWN_CH27_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH27_POS)
#define SPU_ENV_RAMPDOWN_CH28_POS                (12)
#define SPU_ENV_RAMPDOWN_CH28_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH28_POS)
#define SPU_ENV_RAMPDOWN_CH28_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH28_POS)
#define SPU_ENV_RAMPDOWN_CH28_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH28_POS)
#define SPU_ENV_RAMPDOWN_CH29_POS                (13)
#define SPU_ENV_RAMPDOWN_CH29_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH29_POS)
#define SPU_ENV_RAMPDOWN_CH29_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH29_POS)
#define SPU_ENV_RAMPDOWN_CH29_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH29_POS)
#define SPU_ENV_RAMPDOWN_CH30_POS                (14)
#define SPU_ENV_RAMPDOWN_CH30_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH30_POS)
#define SPU_ENV_RAMPDOWN_CH30_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH30_POS)
#define SPU_ENV_RAMPDOWN_CH30_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH30_POS)
#define SPU_ENV_RAMPDOWN_CH31_POS                (15)
#define SPU_ENV_RAMPDOWN_CH31_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH31_POS)
#define SPU_ENV_RAMPDOWN_CH31_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH31_POS)
#define SPU_ENV_RAMPDOWN_CH31_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH31_POS)

/*
 * Bit definition for SPU->STOP_STATUS_CH16_31[15:0]
 */
#define SPU_STOP_FLAG_CH16_POS                   (0)
#define SPU_STOP_FLAG_CH16_MSK                   (0x1UL << SPU_STOP_FLAG_CH16_POS)
#define SPU_STOP_FLAG_CH16                       (0x1UL << SPU_STOP_FLAG_CH16_POS)
#define SPU_STOP_FLAG_CH17_POS                   (1)
#define SPU_STOP_FLAG_CH17_MSK                   (0x1UL << SPU_STOP_FLAG_CH17_POS)
#define SPU_STOP_FLAG_CH17                       (0x1UL << SPU_STOP_FLAG_CH17_POS)
#define SPU_STOP_FLAG_CH18_POS                   (2)
#define SPU_STOP_FLAG_CH18_MSK                   (0x1UL << SPU_STOP_FLAG_CH18_POS)
#define SPU_STOP_FLAG_CH18                       (0x1UL << SPU_STOP_FLAG_CH18_POS)
#define SPU_STOP_FLAG_CH19_POS                   (3)
#define SPU_STOP_FLAG_CH19_MSK                   (0x1UL << SPU_STOP_FLAG_CH19_POS)
#define SPU_STOP_FLAG_CH19                       (0x0UL << SPU_STOP_FLAG_CH19_POS)
#define SPU_STOP_FLAG_CH20_POS                   (4)
#define SPU_STOP_FLAG_CH20_MSK                   (0x1UL << SPU_STOP_FLAG_CH20_POS)
#define SPU_STOP_FLAG_CH20                       (0x0UL << SPU_STOP_FLAG_CH20_POS)
#define SPU_STOP_FLAG_CH21_POS                   (5)
#define SPU_STOP_FLAG_CH21_MSK                   (0x1UL << SPU_STOP_FLAG_CH21_POS)
#define SPU_STOP_FLAG_CH21                       (0x0UL << SPU_STOP_FLAG_CH21_POS)
#define SPU_STOP_FLAG_CH22_POS                   (6)
#define SPU_STOP_FLAG_CH22_MSK                   (0x1UL << SPU_STOP_FLAG_CH22_POS)
#define SPU_STOP_FLAG_CH22                       (0x0UL << SPU_STOP_FLAG_CH22_POS)
#define SPU_STOP_FLAG_CH23_POS                   (7)
#define SPU_STOP_FLAG_CH23_MSK                   (0x1UL << SPU_STOP_FLAG_CH23_POS)
#define SPU_STOP_FLAG_CH23                       (0x0UL << SPU_STOP_FLAG_CH23_POS)
#define SPU_STOP_FLAG_CH24_POS                   (8)
#define SPU_STOP_FLAG_CH24_MSK                   (0x1UL << SPU_STOP_FLAG_CH24_POS)
#define SPU_STOP_FLAG_CH24                       (0x0UL << SPU_STOP_FLAG_CH24_POS)
#define SPU_STOP_FLAG_CH25_POS                   (9)
#define SPU_STOP_FLAG_CH25_MSK                   (0x1UL << SPU_STOP_FLAG_CH25_POS)
#define SPU_STOP_FLAG_CH25                       (0x0UL << SPU_STOP_FLAG_CH25_POS)
#define SPU_STOP_FLAG_CH26_POS                   (10)
#define SPU_STOP_FLAG_CH26_MSK                   (0x1UL << SPU_STOP_FLAG_CH26_POS)
#define SPU_STOP_FLAG_CH26                       (0x0UL << SPU_STOP_FLAG_CH26_POS)
#define SPU_STOP_FLAG_CH27_POS                   (11)
#define SPU_STOP_FLAG_CH27_MSK                   (0x1UL << SPU_STOP_FLAG_CH27_POS)
#define SPU_STOP_FLAG_CH27                       (0x0UL << SPU_STOP_FLAG_CH27_POS)
#define SPU_STOP_FLAG_CH28_POS                   (12)
#define SPU_STOP_FLAG_CH28_MSK                   (0x1UL << SPU_STOP_FLAG_CH28_POS)
#define SPU_STOP_FLAG_CH28                       (0x0UL << SPU_STOP_FLAG_CH28_POS)
#define SPU_STOP_FLAG_CH29_POS                   (13)
#define SPU_STOP_FLAG_CH29_MSK                   (0x1UL << SPU_STOP_FLAG_CH29_POS)
#define SPU_STOP_FLAG_CH29                       (0x0UL << SPU_STOP_FLAG_CH29_POS)
#define SPU_STOP_FLAG_CH30_POS                   (14)
#define SPU_STOP_FLAG_CH30_MSK                   (0x1UL << SPU_STOP_FLAG_CH30_POS)
#define SPU_STOP_FLAG_CH30                       (0x0UL << SPU_STOP_FLAG_CH30_POS)
#define SPU_STOP_FLAG_CH31_POS                   (15)
#define SPU_STOP_FLAG_CH31_MSK                   (0x1UL << SPU_STOP_FLAG_CH31_POS)
#define SPU_STOP_FLAG_CH31                       (0x0UL << SPU_STOP_FLAG_CH31_POS)

/*
 * Bit definition for SPU->CH_STATUS_CH16_31[15:0]
 */
#define SPU_BUSY_FLAG_CH16_POS                   (0)
#define SPU_BUSY_FLAG_CH16_MSK                   (0x1UL << SPU_BUSY_FLAG_CH16_POS)
#define SPU_BUSY_FLAG_CH16                       (0x1UL << SPU_BUSY_FLAG_CH16_POS)
#define SPU_BUSY_FLAG_CH17_POS                   (1)
#define SPU_BUSY_FLAG_CH17_MSK                   (0x1UL << SPU_BUSY_FLAG_CH17_POS)
#define SPU_BUSY_FLAG_CH17                       (0x1UL << SPU_BUSY_FLAG_CH17_POS)
#define SPU_BUSY_FLAG_CH18_POS                   (2)
#define SPU_BUSY_FLAG_CH18_MSK                   (0x1UL << SPU_BUSY_FLAG_CH18_POS)
#define SPU_BUSY_FLAG_CH18                       (0x1UL << SPU_BUSY_FLAG_CH18_POS)
#define SPU_BUSY_FLAG_CH19_POS                   (3)
#define SPU_BUSY_FLAG_CH19_MSK                   (0x1UL << SPU_BUSY_FLAG_CH19_POS)
#define SPU_BUSY_FLAG_CH19                       (0x0UL << SPU_BUSY_FLAG_CH19_POS)
#define SPU_BUSY_FLAG_CH20_POS                   (4)
#define SPU_BUSY_FLAG_CH20_MSK                   (0x1UL << SPU_BUSY_FLAG_CH20_POS)
#define SPU_BUSY_FLAG_CH20                       (0x0UL << SPU_BUSY_FLAG_CH20_POS)
#define SPU_BUSY_FLAG_CH21_POS                   (5)
#define SPU_BUSY_FLAG_CH21_MSK                   (0x1UL << SPU_BUSY_FLAG_CH21_POS)
#define SPU_BUSY_FLAG_CH21                       (0x0UL << SPU_BUSY_FLAG_CH21_POS)
#define SPU_BUSY_FLAG_CH22_POS                   (6)
#define SPU_BUSY_FLAG_CH22_MSK                   (0x1UL << SPU_BUSY_FLAG_CH22_POS)
#define SPU_BUSY_FLAG_CH22                       (0x0UL << SPU_BUSY_FLAG_CH22_POS)
#define SPU_BUSY_FLAG_CH23_POS                   (7)
#define SPU_BUSY_FLAG_CH23_MSK                   (0x1UL << SPU_BUSY_FLAG_CH23_POS)
#define SPU_BUSY_FLAG_CH23                       (0x0UL << SPU_BUSY_FLAG_CH23_POS)
#define SPU_BUSY_FLAG_CH24_POS                   (8)
#define SPU_BUSY_FLAG_CH24_MSK                   (0x1UL << SPU_BUSY_FLAG_CH24_POS)
#define SPU_BUSY_FLAG_CH24                       (0x0UL << SPU_BUSY_FLAG_CH24_POS)
#define SPU_BUSY_FLAG_CH25_POS                   (9)
#define SPU_BUSY_FLAG_CH25_MSK                   (0x1UL << SPU_BUSY_FLAG_CH25_POS)
#define SPU_BUSY_FLAG_CH25                       (0x0UL << SPU_BUSY_FLAG_CH25_POS)
#define SPU_BUSY_FLAG_CH26_POS                   (10)
#define SPU_BUSY_FLAG_CH26_MSK                   (0x1UL << SPU_BUSY_FLAG_CH26_POS)
#define SPU_BUSY_FLAG_CH26                       (0x0UL << SPU_BUSY_FLAG_CH26_POS)
#define SPU_BUSY_FLAG_CH27_POS                   (11)
#define SPU_BUSY_FLAG_CH27_MSK                   (0x1UL << SPU_BUSY_FLAG_CH27_POS)
#define SPU_BUSY_FLAG_CH27                       (0x0UL << SPU_BUSY_FLAG_CH27_POS)
#define SPU_BUSY_FLAG_CH28_POS                   (12)
#define SPU_BUSY_FLAG_CH28_MSK                   (0x1UL << SPU_BUSY_FLAG_CH28_POS)
#define SPU_BUSY_FLAG_CH28                       (0x0UL << SPU_BUSY_FLAG_CH28_POS)
#define SPU_BUSY_FLAG_CH29_POS                   (13)
#define SPU_BUSY_FLAG_CH29_MSK                   (0x1UL << SPU_BUSY_FLAG_CH29_POS)
#define SPU_BUSY_FLAG_CH29                       (0x0UL << SPU_BUSY_FLAG_CH29_POS)
#define SPU_BUSY_FLAG_CH30_POS                   (14)
#define SPU_BUSY_FLAG_CH30_MSK                   (0x1UL << SPU_BUSY_FLAG_CH30_POS)
#define SPU_BUSY_FLAG_CH30                       (0x0UL << SPU_BUSY_FLAG_CH30_POS)
#define SPU_BUSY_FLAG_CH31_POS                   (15)
#define SPU_BUSY_FLAG_CH31_MSK                   (0x1UL << SPU_BUSY_FLAG_CH31_POS)
#define SPU_BUSY_FLAG_CH31                       (0x0UL << SPU_BUSY_FLAG_CH31_POS)

/*
 * Bit definition for SPU->ENV_REPEAT_CH16_31[15:0]
 */
#define SPU_ENV_REPEAT_CH16_POS                  (0)
#define SPU_ENV_REPEAT_CH16_MSK                  (0x1UL << SPU_ENV_REPEAT_CH16_POS)
#define SPU_ENV_REPEAT_CH16_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH16_POS)
#define SPU_ENV_REPEAT_CH16_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH16_POS)
#define SPU_ENV_REPEAT_CH17_POS                  (1)
#define SPU_ENV_REPEAT_CH17_MSK                  (0x1UL << SPU_ENV_REPEAT_CH17_POS)
#define SPU_ENV_REPEAT_CH17_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH17_POS)
#define SPU_ENV_REPEAT_CH17_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH17_POS)
#define SPU_ENV_REPEAT_CH18_POS                  (2)
#define SPU_ENV_REPEAT_CH18_MSK                  (0x1UL << SPU_ENV_REPEAT_CH18_POS)
#define SPU_ENV_REPEAT_CH18_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH18_POS)
#define SPU_ENV_REPEAT_CH18_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH18_POS)
#define SPU_ENV_REPEAT_CH19_POS                  (3)
#define SPU_ENV_REPEAT_CH19_MSK                  (0x1UL << SPU_ENV_REPEAT_CH19_POS)
#define SPU_ENV_REPEAT_CH19_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH19_POS)
#define SPU_ENV_REPEAT_CH19_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH19_POS)
#define SPU_ENV_REPEAT_CH20_POS                  (4)
#define SPU_ENV_REPEAT_CH20_MSK                  (0x1UL << SPU_ENV_REPEAT_CH20_POS)
#define SPU_ENV_REPEAT_CH20_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH20_POS)
#define SPU_ENV_REPEAT_CH20_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH20_POS)
#define SPU_ENV_REPEAT_CH21_POS                  (5)
#define SPU_ENV_REPEAT_CH21_MSK                  (0x1UL << SPU_ENV_REPEAT_CH21_POS)
#define SPU_ENV_REPEAT_CH21_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH21_POS)
#define SPU_ENV_REPEAT_CH21_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH21_POS)
#define SPU_ENV_REPEAT_CH22_POS                  (6)
#define SPU_ENV_REPEAT_CH22_MSK                  (0x1UL << SPU_ENV_REPEAT_CH22_POS)
#define SPU_ENV_REPEAT_CH22_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH22_POS)
#define SPU_ENV_REPEAT_CH22_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH22_POS)
#define SPU_ENV_REPEAT_CH23_POS                  (7)
#define SPU_ENV_REPEAT_CH23_MSK                  (0x1UL << SPU_ENV_REPEAT_CH23_POS)
#define SPU_ENV_REPEAT_CH23_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH23_POS)
#define SPU_ENV_REPEAT_CH23_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH23_POS)
#define SPU_ENV_REPEAT_CH24_POS                  (8)
#define SPU_ENV_REPEAT_CH24_MSK                  (0x1UL << SPU_ENV_REPEAT_CH24_POS)
#define SPU_ENV_REPEAT_CH24_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH24_POS)
#define SPU_ENV_REPEAT_CH24_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH24_POS)
#define SPU_ENV_REPEAT_CH25_POS                  (9)
#define SPU_ENV_REPEAT_CH25_MSK                  (0x1UL << SPU_ENV_REPEAT_CH25_POS)
#define SPU_ENV_REPEAT_CH25_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH25_POS)
#define SPU_ENV_REPEAT_CH25_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH25_POS)
#define SPU_ENV_REPEAT_CH26_POS                  (10)
#define SPU_ENV_REPEAT_CH26_MSK                  (0x1UL << SPU_ENV_REPEAT_CH26_POS)
#define SPU_ENV_REPEAT_CH26_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH26_POS)
#define SPU_ENV_REPEAT_CH26_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH26_POS)
#define SPU_ENV_REPEAT_CH27_POS                  (11)
#define SPU_ENV_REPEAT_CH27_MSK                  (0x1UL << SPU_ENV_REPEAT_CH27_POS)
#define SPU_ENV_REPEAT_CH27_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH27_POS)
#define SPU_ENV_REPEAT_CH27_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH27_POS)
#define SPU_ENV_REPEAT_CH28_POS                  (12)
#define SPU_ENV_REPEAT_CH28_MSK                  (0x1UL << SPU_ENV_REPEAT_CH28_POS)
#define SPU_ENV_REPEAT_CH28_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH28_POS)
#define SPU_ENV_REPEAT_CH28_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH28_POS)
#define SPU_ENV_REPEAT_CH29_POS                  (13)
#define SPU_ENV_REPEAT_CH29_MSK                  (0x1UL << SPU_ENV_REPEAT_CH29_POS)
#define SPU_ENV_REPEAT_CH29_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH29_POS)
#define SPU_ENV_REPEAT_CH29_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH29_POS)
#define SPU_ENV_REPEAT_CH30_POS                  (14)
#define SPU_ENV_REPEAT_CH30_MSK                  (0x1UL << SPU_ENV_REPEAT_CH30_POS)
#define SPU_ENV_REPEAT_CH30_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH30_POS)
#define SPU_ENV_REPEAT_CH30_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH30_POS)
#define SPU_ENV_REPEAT_CH31_POS                  (15)
#define SPU_ENV_REPEAT_CH31_MSK                  (0x1UL << SPU_ENV_REPEAT_CH31_POS)
#define SPU_ENV_REPEAT_CH31_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH31_POS)
#define SPU_ENV_REPEAT_CH31_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH31_POS)

/*
 * Bit definition for SPU->ENV_MODE_CH16_31[15:0]
 */
#define SPU_ENV_MODE_CH16_POS                    (0)
#define SPU_ENV_MODE_CH16_MSK                    (0x1UL << SPU_ENV_MODE_CH16_POS)
#define SPU_ENV_MODE_CH16_MANUAL                 (0x1UL << SPU_ENV_MODE_CH16_POS)
#define SPU_ENV_MODE_CH16_AUTO                   (0x0UL << SPU_ENV_MODE_CH16_POS)
#define SPU_ENV_MODE_CH17_POS                    (1)
#define SPU_ENV_MODE_CH17_MSK                    (0x1UL << SPU_ENV_MODE_CH17_POS)
#define SPU_ENV_MODE_CH17_MANUAL                 (0x1UL << SPU_ENV_MODE_CH17_POS)
#define SPU_ENV_MODE_CH17_AUTO                   (0x0UL << SPU_ENV_MODE_CH17_POS)
#define SPU_ENV_MODE_CH18_POS                    (2)
#define SPU_ENV_MODE_CH18_MSK                    (0x1UL << SPU_ENV_MODE_CH18_POS)
#define SPU_ENV_MODE_CH18_MANUAL                 (0x1UL << SPU_ENV_MODE_CH18_POS)
#define SPU_ENV_MODE_CH18_AUTO                   (0x0UL << SPU_ENV_MODE_CH18_POS)
#define SPU_ENV_MODE_CH19_POS                    (3)
#define SPU_ENV_MODE_CH19_MSK                    (0x1UL << SPU_ENV_MODE_CH19_POS)
#define SPU_ENV_MODE_CH19_MANUAL                 (0x1UL << SPU_ENV_MODE_CH19_POS)
#define SPU_ENV_MODE_CH19_AUTO                   (0x0UL << SPU_ENV_MODE_CH19_POS)
#define SPU_ENV_MODE_CH20_POS                    (4)
#define SPU_ENV_MODE_CH20_MSK                    (0x1UL << SPU_ENV_MODE_CH20_POS)
#define SPU_ENV_MODE_CH20_MANUAL                 (0x1UL << SPU_ENV_MODE_CH20_POS)
#define SPU_ENV_MODE_CH20_AUTO                   (0x0UL << SPU_ENV_MODE_CH20_POS)
#define SPU_ENV_MODE_CH21_POS                    (5)
#define SPU_ENV_MODE_CH21_MSK                    (0x1UL << SPU_ENV_MODE_CH21_POS)
#define SPU_ENV_MODE_CH21_MANUAL                 (0x1UL << SPU_ENV_MODE_CH21_POS)
#define SPU_ENV_MODE_CH21_AUTO                   (0x0UL << SPU_ENV_MODE_CH21_POS)
#define SPU_ENV_MODE_CH22_POS                    (6)
#define SPU_ENV_MODE_CH22_MSK                    (0x1UL << SPU_ENV_MODE_CH22_POS)
#define SPU_ENV_MODE_CH22_MANUAL                 (0x1UL << SPU_ENV_MODE_CH22_POS)
#define SPU_ENV_MODE_CH22_AUTO                   (0x0UL << SPU_ENV_MODE_CH22_POS)
#define SPU_ENV_MODE_CH23_POS                    (7)
#define SPU_ENV_MODE_CH23_MSK                    (0x1UL << SPU_ENV_MODE_CH23_POS)
#define SPU_ENV_MODE_CH23_MANUAL                 (0x1UL << SPU_ENV_MODE_CH23_POS)
#define SPU_ENV_MODE_CH23_AUTO                   (0x0UL << SPU_ENV_MODE_CH23_POS)
#define SPU_ENV_MODE_CH24_POS                    (8)
#define SPU_ENV_MODE_CH24_MSK                    (0x1UL << SPU_ENV_MODE_CH24_POS)
#define SPU_ENV_MODE_CH24_MANUAL                 (0x1UL << SPU_ENV_MODE_CH24_POS)
#define SPU_ENV_MODE_CH24_AUTO                   (0x0UL << SPU_ENV_MODE_CH24_POS)
#define SPU_ENV_MODE_CH25_POS                    (9)
#define SPU_ENV_MODE_CH25_MSK                    (0x1UL << SPU_ENV_MODE_CH25_POS)
#define SPU_ENV_MODE_CH25_MANUAL                 (0x1UL << SPU_ENV_MODE_CH25_POS)
#define SPU_ENV_MODE_CH25_AUTO                   (0x0UL << SPU_ENV_MODE_CH25_POS)
#define SPU_ENV_MODE_CH26_POS                    (10)
#define SPU_ENV_MODE_CH26_MSK                    (0x1UL << SPU_ENV_MODE_CH26_POS)
#define SPU_ENV_MODE_CH26_MANUAL                 (0x1UL << SPU_ENV_MODE_CH26_POS)
#define SPU_ENV_MODE_CH26_AUTO                   (0x0UL << SPU_ENV_MODE_CH26_POS)
#define SPU_ENV_MODE_CH27_POS                    (11)
#define SPU_ENV_MODE_CH27_MSK                    (0x1UL << SPU_ENV_MODE_CH27_POS)
#define SPU_ENV_MODE_CH27_MANUAL                 (0x1UL << SPU_ENV_MODE_CH27_POS)
#define SPU_ENV_MODE_CH27_AUTO                   (0x0UL << SPU_ENV_MODE_CH27_POS)
#define SPU_ENV_MODE_CH28_POS                    (12)
#define SPU_ENV_MODE_CH28_MSK                    (0x1UL << SPU_ENV_MODE_CH28_POS)
#define SPU_ENV_MODE_CH28_MANUAL                 (0x1UL << SPU_ENV_MODE_CH28_POS)
#define SPU_ENV_MODE_CH28_AUTO                   (0x0UL << SPU_ENV_MODE_CH28_POS)
#define SPU_ENV_MODE_CH29_POS                    (13)
#define SPU_ENV_MODE_CH29_MSK                    (0x1UL << SPU_ENV_MODE_CH29_POS)
#define SPU_ENV_MODE_CH29_MANUAL                 (0x1UL << SPU_ENV_MODE_CH29_POS)
#define SPU_ENV_MODE_CH29_AUTO                   (0x0UL << SPU_ENV_MODE_CH29_POS)
#define SPU_ENV_MODE_CH30_POS                    (14)
#define SPU_ENV_MODE_CH30_MSK                    (0x1UL << SPU_ENV_MODE_CH30_POS)
#define SPU_ENV_MODE_CH30_MANUAL                 (0x1UL << SPU_ENV_MODE_CH30_POS)
#define SPU_ENV_MODE_CH30_AUTO                   (0x0UL << SPU_ENV_MODE_CH30_POS)
#define SPU_ENV_MODE_CH31_POS                    (15)
#define SPU_ENV_MODE_CH31_MSK                    (0x1UL << SPU_ENV_MODE_CH31_POS)
#define SPU_ENV_MODE_CH31_MANUAL                 (0x1UL << SPU_ENV_MODE_CH31_POS)
#define SPU_ENV_MODE_CH31_AUTO                   (0x0UL << SPU_ENV_MODE_CH31_POS)

/*
 * Bit definition for SPU->TONE_RELEASE_CH16_31[15:0]
 */
#define SPU_TONE_RELEASE_CH16_POS                (0)
#define SPU_TONE_RELEASE_CH16_MSK                (0x1UL << SPU_TONE_RELEASE_CH16_POS)
#define SPU_TONE_RELEASE_CH16_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH16_POS)
#define SPU_TONE_RELEASE_CH16_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH16_POS)
#define SPU_TONE_RELEASE_CH17_POS                (1)
#define SPU_TONE_RELEASE_CH17_MSK                (0x1UL << SPU_TONE_RELEASE_CH17_POS)
#define SPU_TONE_RELEASE_CH17_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH17_POS)
#define SPU_TONE_RELEASE_CH17_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH17_POS)
#define SPU_TONE_RELEASE_CH18_POS                (2)
#define SPU_TONE_RELEASE_CH18_MSK                (0x1UL << SPU_TONE_RELEASE_CH18_POS)
#define SPU_TONE_RELEASE_CH18_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH18_POS)
#define SPU_TONE_RELEASE_CH18_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH18_POS)
#define SPU_TONE_RELEASE_CH19_POS                (3)
#define SPU_TONE_RELEASE_CH19_MSK                (0x1UL << SPU_TONE_RELEASE_CH19_POS)
#define SPU_TONE_RELEASE_CH19_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH19_POS)
#define SPU_TONE_RELEASE_CH19_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH19_POS)
#define SPU_TONE_RELEASE_CH20_POS                (4)
#define SPU_TONE_RELEASE_CH20_MSK                (0x1UL << SPU_TONE_RELEASE_CH20_POS)
#define SPU_TONE_RELEASE_CH20_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH20_POS)
#define SPU_TONE_RELEASE_CH20_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH20_POS)
#define SPU_TONE_RELEASE_CH21_POS                (5)
#define SPU_TONE_RELEASE_CH21_MSK                (0x1UL << SPU_TONE_RELEASE_CH21_POS)
#define SPU_TONE_RELEASE_CH21_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH21_POS)
#define SPU_TONE_RELEASE_CH21_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH21_POS)
#define SPU_TONE_RELEASE_CH22_POS                (6)
#define SPU_TONE_RELEASE_CH22_MSK                (0x1UL << SPU_TONE_RELEASE_CH22_POS)
#define SPU_TONE_RELEASE_CH22_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH22_POS)
#define SPU_TONE_RELEASE_CH22_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH22_POS)
#define SPU_TONE_RELEASE_CH23_POS                (7)
#define SPU_TONE_RELEASE_CH23_MSK                (0x1UL << SPU_TONE_RELEASE_CH23_POS)
#define SPU_TONE_RELEASE_CH23_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH23_POS)
#define SPU_TONE_RELEASE_CH23_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH23_POS)
#define SPU_TONE_RELEASE_CH24_POS                (8)
#define SPU_TONE_RELEASE_CH24_MSK                (0x1UL << SPU_TONE_RELEASE_CH24_POS)
#define SPU_TONE_RELEASE_CH24_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH24_POS)
#define SPU_TONE_RELEASE_CH24_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH24_POS)
#define SPU_TONE_RELEASE_CH25_POS                (9)
#define SPU_TONE_RELEASE_CH25_MSK                (0x1UL << SPU_TONE_RELEASE_CH25_POS)
#define SPU_TONE_RELEASE_CH25_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH25_POS)
#define SPU_TONE_RELEASE_CH25_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH25_POS)
#define SPU_TONE_RELEASE_CH26_POS                (10)
#define SPU_TONE_RELEASE_CH26_MSK                (0x1UL << SPU_TONE_RELEASE_CH26_POS)
#define SPU_TONE_RELEASE_CH26_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH26_POS)
#define SPU_TONE_RELEASE_CH26_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH26_POS)
#define SPU_TONE_RELEASE_CH27_POS                (11)
#define SPU_TONE_RELEASE_CH27_MSK                (0x1UL << SPU_TONE_RELEASE_CH27_POS)
#define SPU_TONE_RELEASE_CH27_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH27_POS)
#define SPU_TONE_RELEASE_CH27_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH27_POS)
#define SPU_TONE_RELEASE_CH28_POS                (12)
#define SPU_TONE_RELEASE_CH28_MSK                (0x1UL << SPU_TONE_RELEASE_CH28_POS)
#define SPU_TONE_RELEASE_CH28_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH28_POS)
#define SPU_TONE_RELEASE_CH28_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH28_POS)
#define SPU_TONE_RELEASE_CH29_POS                (13)
#define SPU_TONE_RELEASE_CH29_MSK                (0x1UL << SPU_TONE_RELEASE_CH29_POS)
#define SPU_TONE_RELEASE_CH29_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH29_POS)
#define SPU_TONE_RELEASE_CH29_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH29_POS)
#define SPU_TONE_RELEASE_CH30_POS                (14)
#define SPU_TONE_RELEASE_CH30_MSK                (0x1UL << SPU_TONE_RELEASE_CH30_POS)
#define SPU_TONE_RELEASE_CH30_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH30_POS)
#define SPU_TONE_RELEASE_CH30_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH30_POS)
#define SPU_TONE_RELEASE_CH31_POS                (15)
#define SPU_TONE_RELEASE_CH31_MSK                (0x1UL << SPU_TONE_RELEASE_CH31_POS)
#define SPU_TONE_RELEASE_CH31_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH31_POS)
#define SPU_TONE_RELEASE_CH31_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH31_POS)

/*
 * Bit definition for SPU->ENV_INT_STATUS_CH16_31[15:0]
 */
#define SPU_ENV_INT_FLAG_CH16_POS                (0)
#define SPU_ENV_INT_FLAG_CH16_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH16_POS)
#define SPU_ENV_INT_FLAG_CH16                    (0x1UL << SPU_ENV_INT_FLAG_CH16_POS)
#define SPU_ENV_INT_FLAG_CH17_POS                (1)
#define SPU_ENV_INT_FLAG_CH17_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH17_POS)
#define SPU_ENV_INT_FLAG_CH17                    (0x1UL << SPU_ENV_INT_FLAG_CH17_POS)
#define SPU_ENV_INT_FLAG_CH18_POS                (2)
#define SPU_ENV_INT_FLAG_CH18_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH18_POS)
#define SPU_ENV_INT_FLAG_CH18                    (0x1UL << SPU_ENV_INT_FLAG_CH18_POS)
#define SPU_ENV_INT_FLAG_CH19_POS                (3)
#define SPU_ENV_INT_FLAG_CH19_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH19_POS)
#define SPU_ENV_INT_FLAG_CH19                    (0x0UL << SPU_ENV_INT_FLAG_CH19_POS)
#define SPU_ENV_INT_FLAG_CH20_POS                (4)
#define SPU_ENV_INT_FLAG_CH20_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH20_POS)
#define SPU_ENV_INT_FLAG_CH20                    (0x0UL << SPU_ENV_INT_FLAG_CH20_POS)
#define SPU_ENV_INT_FLAG_CH21_POS                (5)
#define SPU_ENV_INT_FLAG_CH21_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH21_POS)
#define SPU_ENV_INT_FLAG_CH21                    (0x0UL << SPU_ENV_INT_FLAG_CH21_POS)
#define SPU_ENV_INT_FLAG_CH22_POS                (6)
#define SPU_ENV_INT_FLAG_CH22_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH22_POS)
#define SPU_ENV_INT_FLAG_CH22                    (0x0UL << SPU_ENV_INT_FLAG_CH22_POS)
#define SPU_ENV_INT_FLAG_CH23_POS                (7)
#define SPU_ENV_INT_FLAG_CH23_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH23_POS)
#define SPU_ENV_INT_FLAG_CH23                    (0x0UL << SPU_ENV_INT_FLAG_CH23_POS)
#define SPU_ENV_INT_FLAG_CH24_POS                (8)
#define SPU_ENV_INT_FLAG_CH24_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH24_POS)
#define SPU_ENV_INT_FLAG_CH24                    (0x0UL << SPU_ENV_INT_FLAG_CH24_POS)
#define SPU_ENV_INT_FLAG_CH25_POS                (9)
#define SPU_ENV_INT_FLAG_CH25_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH25_POS)
#define SPU_ENV_INT_FLAG_CH25                    (0x0UL << SPU_ENV_INT_FLAG_CH25_POS)
#define SPU_ENV_INT_FLAG_CH26_POS                (10)
#define SPU_ENV_INT_FLAG_CH26_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH26_POS)
#define SPU_ENV_INT_FLAG_CH26                    (0x0UL << SPU_ENV_INT_FLAG_CH26_POS)
#define SPU_ENV_INT_FLAG_CH27_POS                (11)
#define SPU_ENV_INT_FLAG_CH27_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH27_POS)
#define SPU_ENV_INT_FLAG_CH27                    (0x0UL << SPU_ENV_INT_FLAG_CH27_POS)
#define SPU_ENV_INT_FLAG_CH28_POS                (12)
#define SPU_ENV_INT_FLAG_CH28_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH28_POS)
#define SPU_ENV_INT_FLAG_CH28                    (0x0UL << SPU_ENV_INT_FLAG_CH28_POS)
#define SPU_ENV_INT_FLAG_CH29_POS                (13)
#define SPU_ENV_INT_FLAG_CH29_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH29_POS)
#define SPU_ENV_INT_FLAG_CH29                    (0x0UL << SPU_ENV_INT_FLAG_CH29_POS)
#define SPU_ENV_INT_FLAG_CH30_POS                (14)
#define SPU_ENV_INT_FLAG_CH30_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH30_POS)
#define SPU_ENV_INT_FLAG_CH30                    (0x0UL << SPU_ENV_INT_FLAG_CH30_POS)
#define SPU_ENV_INT_FLAG_CH31_POS                (15)
#define SPU_ENV_INT_FLAG_CH31_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH31_POS)
#define SPU_ENV_INT_FLAG_CH31                    (0x0UL << SPU_ENV_INT_FLAG_CH31_POS)

/*
 * Bit definition for SPU->PITCHBEND_EN_CH16_31[15:0]
 */
#define SPU_PITCHBEND_CH16_POS                   (0)
#define SPU_PITCHBEND_CH16_MSK                   (0x1UL << SPU_PITCHBEND_CH16_POS)
#define SPU_PITCHBEND_CH16_ENABLE                (0x1UL << SPU_PITCHBEND_CH16_POS)
#define SPU_PITCHBEND_CH16_DISABLE               (0x0UL << SPU_PITCHBEND_CH16_POS)
#define SPU_PITCHBEND_CH17_POS                   (1)
#define SPU_PITCHBEND_CH17_MSK                   (0x1UL << SPU_PITCHBEND_CH17_POS)
#define SPU_PITCHBEND_CH17_ENABLE                (0x1UL << SPU_PITCHBEND_CH17_POS)
#define SPU_PITCHBEND_CH17_DISABLE               (0x0UL << SPU_PITCHBEND_CH17_POS)
#define SPU_PITCHBEND_CH18_POS                   (2)
#define SPU_PITCHBEND_CH18_MSK                   (0x1UL << SPU_PITCHBEND_CH18_POS)
#define SPU_PITCHBEND_CH18_ENABLE                (0x1UL << SPU_PITCHBEND_CH18_POS)
#define SPU_PITCHBEND_CH18_DISABLE               (0x0UL << SPU_PITCHBEND_CH18_POS)
#define SPU_PITCHBEND_CH19_POS                   (3)
#define SPU_PITCHBEND_CH19_MSK                   (0x1UL << SPU_PITCHBEND_CH19_POS)
#define SPU_PITCHBEND_CH19_ENABLE                (0x1UL << SPU_PITCHBEND_CH19_POS)
#define SPU_PITCHBEND_CH19_DISABLE               (0x0UL << SPU_PITCHBEND_CH19_POS)
#define SPU_PITCHBEND_CH20_POS                   (4)
#define SPU_PITCHBEND_CH20_MSK                   (0x1UL << SPU_PITCHBEND_CH20_POS)
#define SPU_PITCHBEND_CH20_ENABLE                (0x1UL << SPU_PITCHBEND_CH20_POS)
#define SPU_PITCHBEND_CH20_DISABLE               (0x0UL << SPU_PITCHBEND_CH20_POS)
#define SPU_PITCHBEND_CH21_POS                   (5)
#define SPU_PITCHBEND_CH21_MSK                   (0x1UL << SPU_PITCHBEND_CH21_POS)
#define SPU_PITCHBEND_CH21_ENABLE                (0x1UL << SPU_PITCHBEND_CH21_POS)
#define SPU_PITCHBEND_CH21_DISABLE               (0x0UL << SPU_PITCHBEND_CH21_POS)
#define SPU_PITCHBEND_CH22_POS                   (6)
#define SPU_PITCHBEND_CH22_MSK                   (0x1UL << SPU_PITCHBEND_CH22_POS)
#define SPU_PITCHBEND_CH22_ENABLE                (0x1UL << SPU_PITCHBEND_CH22_POS)
#define SPU_PITCHBEND_CH22_DISABLE               (0x0UL << SPU_PITCHBEND_CH22_POS)
#define SPU_PITCHBEND_CH23_POS                   (7)
#define SPU_PITCHBEND_CH23_MSK                   (0x1UL << SPU_PITCHBEND_CH23_POS)
#define SPU_PITCHBEND_CH23_ENABLE                (0x1UL << SPU_PITCHBEND_CH23_POS)
#define SPU_PITCHBEND_CH23_DISABLE               (0x0UL << SPU_PITCHBEND_CH23_POS)
#define SPU_PITCHBEND_CH24_POS                   (8)
#define SPU_PITCHBEND_CH24_MSK                   (0x1UL << SPU_PITCHBEND_CH24_POS)
#define SPU_PITCHBEND_CH24_ENABLE                (0x1UL << SPU_PITCHBEND_CH24_POS)
#define SPU_PITCHBEND_CH24_DISABLE               (0x0UL << SPU_PITCHBEND_CH24_POS)
#define SPU_PITCHBEND_CH25_POS                   (9)
#define SPU_PITCHBEND_CH25_MSK                   (0x1UL << SPU_PITCHBEND_CH25_POS)
#define SPU_PITCHBEND_CH25_ENABLE                (0x1UL << SPU_PITCHBEND_CH25_POS)
#define SPU_PITCHBEND_CH25_DISABLE               (0x0UL << SPU_PITCHBEND_CH25_POS)
#define SPU_PITCHBEND_CH26_POS                   (10)
#define SPU_PITCHBEND_CH26_MSK                   (0x1UL << SPU_PITCHBEND_CH26_POS)
#define SPU_PITCHBEND_CH26_ENABLE                (0x1UL << SPU_PITCHBEND_CH26_POS)
#define SPU_PITCHBEND_CH26_DISABLE               (0x0UL << SPU_PITCHBEND_CH26_POS)
#define SPU_PITCHBEND_CH27_POS                   (11)
#define SPU_PITCHBEND_CH27_MSK                   (0x1UL << SPU_PITCHBEND_CH27_POS)
#define SPU_PITCHBEND_CH27_ENABLE                (0x1UL << SPU_PITCHBEND_CH27_POS)
#define SPU_PITCHBEND_CH27_DISABLE               (0x0UL << SPU_PITCHBEND_CH27_POS)
#define SPU_PITCHBEND_CH28_POS                   (12)
#define SPU_PITCHBEND_CH28_MSK                   (0x1UL << SPU_PITCHBEND_CH28_POS)
#define SPU_PITCHBEND_CH28_ENABLE                (0x1UL << SPU_PITCHBEND_CH28_POS)
#define SPU_PITCHBEND_CH28_DISABLE               (0x0UL << SPU_PITCHBEND_CH28_POS)
#define SPU_PITCHBEND_CH29_POS                   (13)
#define SPU_PITCHBEND_CH29_MSK                   (0x1UL << SPU_PITCHBEND_CH29_POS)
#define SPU_PITCHBEND_CH29_ENABLE                (0x1UL << SPU_PITCHBEND_CH29_POS)
#define SPU_PITCHBEND_CH29_DISABLE               (0x0UL << SPU_PITCHBEND_CH29_POS)
#define SPU_PITCHBEND_CH30_POS                   (14)
#define SPU_PITCHBEND_CH30_MSK                   (0x1UL << SPU_PITCHBEND_CH30_POS)
#define SPU_PITCHBEND_CH30_ENABLE                (0x1UL << SPU_PITCHBEND_CH30_POS)
#define SPU_PITCHBEND_CH30_DISABLE               (0x0UL << SPU_PITCHBEND_CH30_POS)
#define SPU_PITCHBEND_CH31_POS                   (15)
#define SPU_PITCHBEND_CH31_MSK                   (0x1UL << SPU_PITCHBEND_CH31_POS)
#define SPU_PITCHBEND_CH31_ENABLE                (0x1UL << SPU_PITCHBEND_CH31_POS)
#define SPU_PITCHBEND_CH31_DISABLE               (0x0UL << SPU_PITCHBEND_CH31_POS)

/*
 * Bit definition for SPU->SPU_EN_CH32_47[15:0]
 */
#define SPU_CH32_EN_POS                          (0)
#define SPU_CH32_EN_MSK                          (0x1UL << SPU_CH32_EN_POS)
#define SPU_CH32_ENABLE                          (0x1UL << SPU_CH32_EN_POS)
#define SPU_CH32_DISABLE                         (0x0UL << SPU_CH32_EN_POS)
#define SPU_CH33_EN_POS                          (1)
#define SPU_CH33_EN_MSK                          (0x1UL << SPU_CH33_EN_POS)
#define SPU_CH33_ENABLE                          (0x1UL << SPU_CH33_EN_POS)
#define SPU_CH33_DISABLE                         (0x0UL << SPU_CH33_EN_POS)
#define SPU_CH34_EN_POS                          (2)
#define SPU_CH34_EN_MSK                          (0x1UL << SPU_CH34_EN_POS)
#define SPU_CH34_ENABLE                          (0x1UL << SPU_CH34_EN_POS)
#define SPU_CH34_DISABLE                         (0x0UL << SPU_CH34_EN_POS)
#define SPU_CH35_EN_POS                          (3)
#define SPU_CH35_EN_MSK                          (0x1UL << SPU_CH35_EN_POS)
#define SPU_CH35_ENABLE                          (0x1UL << SPU_CH35_EN_POS)
#define SPU_CH35_DISABLE                         (0x0UL << SPU_CH35_EN_POS)
#define SPU_CH36_EN_POS                          (4)
#define SPU_CH36_EN_MSK                          (0x1UL << SPU_CH36_EN_POS)
#define SPU_CH36_ENABLE                          (0x1UL << SPU_CH36_EN_POS)
#define SPU_CH36_DISABLE                         (0x0UL << SPU_CH36_EN_POS)
#define SPU_CH37_EN_POS                          (5)
#define SPU_CH37_EN_MSK                          (0x1UL << SPU_CH37_EN_POS)
#define SPU_CH37_ENABLE                          (0x1UL << SPU_CH37_EN_POS)
#define SPU_CH37_DISABLE                         (0x0UL << SPU_CH37_EN_POS)
#define SPU_CH38_EN_POS                          (6)
#define SPU_CH38_EN_MSK                          (0x1UL << SPU_CH38_EN_POS)
#define SPU_CH38_ENABLE                          (0x1UL << SPU_CH38_EN_POS)
#define SPU_CH38_DISABLE                         (0x0UL << SPU_CH38_EN_POS)
#define SPU_CH39_EN_POS                          (7)
#define SPU_CH39_EN_MSK                          (0x1UL << SPU_CH39_EN_POS)
#define SPU_CH39_ENABLE                          (0x1UL << SPU_CH39_EN_POS)
#define SPU_CH39_DISABLE                         (0x0UL << SPU_CH39_EN_POS)
#define SPU_CH40_EN_POS                          (8)
#define SPU_CH40_EN_MSK                          (0x1UL << SPU_CH40_EN_POS)
#define SPU_CH40_ENABLE                          (0x1UL << SPU_CH40_EN_POS)
#define SPU_CH40_DISABLE                         (0x0UL << SPU_CH40_EN_POS)
#define SPU_CH41_EN_POS                          (9)
#define SPU_CH41_EN_MSK                          (0x1UL << SPU_CH41_EN_POS)
#define SPU_CH41_ENABLE                          (0x1UL << SPU_CH41_EN_POS)
#define SPU_CH41_DISABLE                         (0x0UL << SPU_CH41_EN_POS)
#define SPU_CH42_EN_POS                          (10)
#define SPU_CH42_EN_MSK                          (0x1UL << SPU_CH42_EN_POS)
#define SPU_CH42_ENABLE                          (0x1UL << SPU_CH42_EN_POS)
#define SPU_CH42_DISABLE                         (0x0UL << SPU_CH42_EN_POS)
#define SPU_CH43_EN_POS                          (11)
#define SPU_CH43_EN_MSK                          (0x1UL << SPU_CH43_EN_POS)
#define SPU_CH43_ENABLE                          (0x1UL << SPU_CH43_EN_POS)
#define SPU_CH43_DISABLE                         (0x0UL << SPU_CH43_EN_POS)
#define SPU_CH44_EN_POS                          (12)
#define SPU_CH44_EN_MSK                          (0x1UL << SPU_CH44_EN_POS)
#define SPU_CH44_ENABLE                          (0x1UL << SPU_CH44_EN_POS)
#define SPU_CH44_DISABLE                         (0x0UL << SPU_CH44_EN_POS)
#define SPU_CH45_EN_POS                          (13)
#define SPU_CH45_EN_MSK                          (0x1UL << SPU_CH45_EN_POS)
#define SPU_CH45_ENABLE                          (0x1UL << SPU_CH45_EN_POS)
#define SPU_CH45_DISABLE                         (0x0UL << SPU_CH45_EN_POS)
#define SPU_CH46_EN_POS                          (14)
#define SPU_CH46_EN_MSK                          (0x1UL << SPU_CH46_EN_POS)
#define SPU_CH46_ENABLE                          (0x1UL << SPU_CH46_EN_POS)
#define SPU_CH46_DISABLE                         (0x0UL << SPU_CH46_EN_POS)
#define SPU_CH47_EN_POS                          (15)
#define SPU_CH47_EN_MSK                          (0x1UL << SPU_CH47_EN_POS)
#define SPU_CH47_ENABLE                          (0x1UL << SPU_CH47_EN_POS)
#define SPU_CH47_DISABLE                         (0x0UL << SPU_CH47_EN_POS)

/*
 * Bit definition for SPU->INT_EN_CH32_47[15:0] - SPU Channel FIQ Enable
 */
#define SPU_INT_CH32_EN_POS                      (0)
#define SPU_INT_CH32_EN_MSK                      (0x1UL << SPU_INT_CH32_EN_POS)
#define SPU_INT_CH32_ENABLE                      (0x1UL << SPU_INT_CH32_EN_POS)
#define SPU_INT_CH32_DISABLE                     (0x0UL << SPU_INT_CH32_EN_POS)
#define SPU_INT_CH33_EN_POS                      (1)
#define SPU_INT_CH33_EN_MSK                      (0x1UL << SPU_INT_CH33_EN_POS)
#define SPU_INT_CH33_ENABLE                      (0x1UL << SPU_INT_CH33_EN_POS)
#define SPU_INT_CH33_DISABLE                     (0x0UL << SPU_INT_CH33_EN_POS)
#define SPU_INT_CH34_EN_POS                      (2)
#define SPU_INT_CH34_EN_MSK                      (0x1UL << SPU_INT_CH34_EN_POS)
#define SPU_INT_CH34_ENABLE                      (0x1UL << SPU_INT_CH34_EN_POS)
#define SPU_INT_CH34_DISABLE                     (0x0UL << SPU_INT_CH34_EN_POS)
#define SPU_INT_CH35_EN_POS                      (3)
#define SPU_INT_CH35_EN_MSK                      (0x1UL << SPU_INT_CH35_EN_POS)
#define SPU_INT_CH35_ENABLE                      (0x1UL << SPU_INT_CH35_EN_POS)
#define SPU_INT_CH35_DISABLE                     (0x0UL << SPU_INT_CH35_EN_POS)
#define SPU_INT_CH36_EN_POS                      (4)
#define SPU_INT_CH36_EN_MSK                      (0x1UL << SPU_INT_CH36_EN_POS)
#define SPU_INT_CH36_ENABLE                      (0x1UL << SPU_INT_CH36_EN_POS)
#define SPU_INT_CH36_DISABLE                     (0x0UL << SPU_INT_CH36_EN_POS)
#define SPU_INT_CH37_EN_POS                      (5)
#define SPU_INT_CH37_EN_MSK                      (0x1UL << SPU_INT_CH37_EN_POS)
#define SPU_INT_CH37_ENABLE                      (0x1UL << SPU_INT_CH37_EN_POS)
#define SPU_INT_CH37_DISABLE                     (0x0UL << SPU_INT_CH37_EN_POS)
#define SPU_INT_CH38_EN_POS                      (6)
#define SPU_INT_CH38_EN_MSK                      (0x1UL << SPU_INT_CH38_EN_POS)
#define SPU_INT_CH38_ENABLE                      (0x1UL << SPU_INT_CH38_EN_POS)
#define SPU_INT_CH38_DISABLE                     (0x0UL << SPU_INT_CH38_EN_POS)
#define SPU_INT_CH39_EN_POS                      (7)
#define SPU_INT_CH39_EN_MSK                      (0x1UL << SPU_INT_CH39_EN_POS)
#define SPU_INT_CH39_ENABLE                      (0x1UL << SPU_INT_CH39_EN_POS)
#define SPU_INT_CH39_DISABLE                     (0x0UL << SPU_INT_CH39_EN_POS)
#define SPU_INT_CH40_EN_POS                      (8)
#define SPU_INT_CH40_EN_MSK                      (0x1UL << SPU_INT_CH40_EN_POS)
#define SPU_INT_CH40_ENABLE                      (0x1UL << SPU_INT_CH40_EN_POS)
#define SPU_INT_CH40_DISABLE                     (0x0UL << SPU_INT_CH40_EN_POS)
#define SPU_INT_CH41_EN_POS                      (9)
#define SPU_INT_CH41_EN_MSK                      (0x1UL << SPU_INT_CH41_EN_POS)
#define SPU_INT_CH41_ENABLE                      (0x1UL << SPU_INT_CH41_EN_POS)
#define SPU_INT_CH41_DISABLE                     (0x0UL << SPU_INT_CH41_EN_POS)
#define SPU_INT_CH42_EN_POS                      (10)
#define SPU_INT_CH42_EN_MSK                      (0x1UL << SPU_INT_CH42_EN_POS)
#define SPU_INT_CH42_ENABLE                      (0x1UL << SPU_INT_CH42_EN_POS)
#define SPU_INT_CH42_DISABLE                     (0x0UL << SPU_INT_CH42_EN_POS)
#define SPU_INT_CH43_EN_POS                      (11)
#define SPU_INT_CH43_EN_MSK                      (0x1UL << SPU_INT_CH43_EN_POS)
#define SPU_INT_CH43_ENABLE                      (0x1UL << SPU_INT_CH43_EN_POS)
#define SPU_INT_CH43_DISABLE                     (0x0UL << SPU_INT_CH43_EN_POS)
#define SPU_INT_CH44_EN_POS                      (12)
#define SPU_INT_CH44_EN_MSK                      (0x1UL << SPU_INT_CH44_EN_POS)
#define SPU_INT_CH44_ENABLE                      (0x1UL << SPU_INT_CH44_EN_POS)
#define SPU_INT_CH44_DISABLE                     (0x0UL << SPU_INT_CH44_EN_POS)
#define SPU_INT_CH45_EN_POS                      (13)
#define SPU_INT_CH45_EN_MSK                      (0x1UL << SPU_INT_CH45_EN_POS)
#define SPU_INT_CH45_ENABLE                      (0x1UL << SPU_INT_CH45_EN_POS)
#define SPU_INT_CH45_DISABLE                     (0x0UL << SPU_INT_CH45_EN_POS)
#define SPU_INT_CH46_EN_POS                      (14)
#define SPU_INT_CH46_EN_MSK                      (0x1UL << SPU_INT_CH46_EN_POS)
#define SPU_INT_CH46_ENABLE                      (0x1UL << SPU_INT_CH46_EN_POS)
#define SPU_INT_CH46_DISABLE                     (0x0UL << SPU_INT_CH46_EN_POS)
#define SPU_INT_CH47_EN_POS                      (15)
#define SPU_INT_CH47_EN_MSK                      (0x1UL << SPU_INT_CH47_EN_POS)
#define SPU_INT_CH47_ENABLE                      (0x1UL << SPU_INT_CH47_EN_POS)
#define SPU_INT_CH47_DISABLE                     (0x0UL << SPU_INT_CH47_EN_POS)

/*
 * Bit definition for INT_STATUS_CH32_47[15:0] - SPU Channel INT Status
 */
#define SPU_INT_FLAG_CH32_POS                    (0)
#define SPU_INT_FLAG_CH32_MSK                    (0x1UL << SPU_INT_FLAG_CH32_POS)
#define SPU_INT_FLAG_CH32                        (0x1UL << SPU_INT_FLAG_CH32_POS)
#define SPU_INT_FLAG_CH33_POS                    (1)
#define SPU_INT_FLAG_CH33_MSK                    (0x1UL << SPU_INT_FLAG_CH33_POS)
#define SPU_INT_FLAG_CH33                        (0x1UL << SPU_INT_FLAG_CH33_POS)
#define SPU_INT_FLAG_CH34_POS                    (2)
#define SPU_INT_FLAG_CH34_MSK                    (0x1UL << SPU_INT_FLAG_CH34_POS)
#define SPU_INT_FLAG_CH34                        (0x1UL << SPU_INT_FLAG_CH34_POS)
#define SPU_INT_FLAG_CH35_POS                    (3)
#define SPU_INT_FLAG_CH35_MSK                    (0x1UL << SPU_INT_FLAG_CH35_POS)
#define SPU_INT_FLAG_CH35                        (0x0UL << SPU_INT_FLAG_CH35_POS)
#define SPU_INT_FLAG_CH36_POS                    (4)
#define SPU_INT_FLAG_CH36_MSK                    (0x1UL << SPU_INT_FLAG_CH36_POS)
#define SPU_INT_FLAG_CH36                        (0x0UL << SPU_INT_FLAG_CH36_POS)
#define SPU_INT_FLAG_CH37_POS                    (5)
#define SPU_INT_FLAG_CH37_MSK                    (0x1UL << SPU_INT_FLAG_CH37_POS)
#define SPU_INT_FLAG_CH37                        (0x0UL << SPU_INT_FLAG_CH37_POS)
#define SPU_INT_FLAG_CH38_POS                    (6)
#define SPU_INT_FLAG_CH38_MSK                    (0x1UL << SPU_INT_FLAG_CH38_POS)
#define SPU_INT_FLAG_CH38                        (0x0UL << SPU_INT_FLAG_CH38_POS)
#define SPU_INT_FLAG_CH39_POS                    (7)
#define SPU_INT_FLAG_CH39_MSK                    (0x1UL << SPU_INT_FLAG_CH39_POS)
#define SPU_INT_FLAG_CH39                        (0x0UL << SPU_INT_FLAG_CH39_POS)
#define SPU_INT_FLAG_CH40_POS                    (8)
#define SPU_INT_FLAG_CH40_MSK                    (0x1UL << SPU_INT_FLAG_CH40_POS)
#define SPU_INT_FLAG_CH40                        (0x0UL << SPU_INT_FLAG_CH40_POS)
#define SPU_INT_FLAG_CH41_POS                    (9)
#define SPU_INT_FLAG_CH41_MSK                    (0x1UL << SPU_INT_FLAG_CH41_POS)
#define SPU_INT_FLAG_CH41                        (0x0UL << SPU_INT_FLAG_CH41_POS)
#define SPU_INT_FLAG_CH42_POS                    (10)
#define SPU_INT_FLAG_CH42_MSK                    (0x1UL << SPU_INT_FLAG_CH42_POS)
#define SPU_INT_FLAG_CH42                        (0x0UL << SPU_INT_FLAG_CH42_POS)
#define SPU_INT_FLAG_CH43_POS                    (11)
#define SPU_INT_FLAG_CH43_MSK                    (0x1UL << SPU_INT_FLAG_CH43_POS)
#define SPU_INT_FLAG_CH43                        (0x0UL << SPU_INT_FLAG_CH43_POS)
#define SPU_INT_FLAG_CH44_POS                    (12)
#define SPU_INT_FLAG_CH44_MSK                    (0x1UL << SPU_INT_FLAG_CH44_POS)
#define SPU_INT_FLAG_CH44                        (0x0UL << SPU_INT_FLAG_CH44_POS)
#define SPU_INT_FLAG_CH45_POS                    (13)
#define SPU_INT_FLAG_CH45_MSK                    (0x1UL << SPU_INT_FLAG_CH45_POS)
#define SPU_INT_FLAG_CH45                        (0x0UL << SPU_INT_FLAG_CH45_POS)
#define SPU_INT_FLAG_CH46_POS                    (14)
#define SPU_INT_FLAG_CH46_MSK                    (0x1UL << SPU_INT_FLAG_CH46_POS)
#define SPU_INT_FLAG_CH46                        (0x0UL << SPU_INT_FLAG_CH46_POS)
#define SPU_INT_FLAG_CH47_POS                    (15)
#define SPU_INT_FLAG_CH47_MSK                    (0x1UL << SPU_INT_FLAG_CH47_POS)
#define SPU_INT_FLAG_CH47                        (0x0UL << SPU_INT_FLAG_CH47_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH32_3[3:0] -
 */
#define SPU_ENV_CLK_CH32_POS                     (0)
#define SPU_ENV_CLK_CH32_MSK                     (0xFUL << SPU_ENV_CLK_CH32_POS)
#define SPU_ENV_CLK_CH32_0x0                     (0x0UL << SPU_ENV_CLK_CH32_POS)
#define SPU_ENV_CLK_CH32_0xF                     (0xFUL << SPU_ENV_CLK_CH32_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH32_3[7:4] -
 */
#define SPU_ENV_CLK_CH33_POS                     (4)
#define SPU_ENV_CLK_CH33_MSK                     (0xFUL << SPU_ENV_CLK_CH33_POS)
#define SPU_ENV_CLK_CH33_0x0                     (0x0UL << SPU_ENV_CLK_CH33_POS)
#define SPU_ENV_CLK_CH33_0xF                     (0xFUL << SPU_ENV_CLK_CH33_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH32_3[11:8] -
 */
#define SPU_ENV_CLK_CH34_POS                     (8)
#define SPU_ENV_CLK_CH34_MSK                     (0xFUL << SPU_ENV_CLK_CH34_POS)
#define SPU_ENV_CLK_CH34_0x0                     (0x0UL << SPU_ENV_CLK_CH34_POS)
#define SPU_ENV_CLK_CH34_0xF                     (0xFUL << SPU_ENV_CLK_CH34_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH32_3[15:12] -
 */
#define SPU_ENV_CLK_CH35_POS                     (12)
#define SPU_ENV_CLK_CH35_MSK                     (0xFUL << SPU_ENV_CLK_CH35_POS)
#define SPU_ENV_CLK_CH35_0x0                     (0x0UL << SPU_ENV_CLK_CH35_POS)
#define SPU_ENV_CLK_CH35_0xF                     (0xFUL << SPU_ENV_CLK_CH35_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH36_7[3:0] -
 */
#define SPU_ENV_CLK_CH36_POS                     (0)
#define SPU_ENV_CLK_CH36_MSK                     (0xFUL << SPU_ENV_CLK_CH36_POS)
#define SPU_ENV_CLK_CH36_0x0                     (0x0UL << SPU_ENV_CLK_CH36_POS)
#define SPU_ENV_CLK_CH36_0xF                     (0xFUL << SPU_ENV_CLK_CH36_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH36_7[7:4] -
 */
#define SPU_ENV_CLK_CH37_POS                     (4)
#define SPU_ENV_CLK_CH37_MSK                     (0xFUL << SPU_ENV_CLK_CH37_POS)
#define SPU_ENV_CLK_CH37_0x0                     (0x0UL << SPU_ENV_CLK_CH37_POS)
#define SPU_ENV_CLK_CH37_0xF                     (0xFUL << SPU_ENV_CLK_CH37_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH36_7[11:8] -
 */
#define SPU_ENV_CLK_CH38_POS                     (8)
#define SPU_ENV_CLK_CH38_MSK                     (0xFUL << SPU_ENV_CLK_CH38_POS)
#define SPU_ENV_CLK_CH38_0x0                     (0x0UL << SPU_ENV_CLK_CH38_POS)
#define SPU_ENV_CLK_CH38_0xF                     (0xFUL << SPU_ENV_CLK_CH38_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH36_7[15:12] -
 */
#define SPU_ENV_CLK_CH39_POS                     (12)
#define SPU_ENV_CLK_CH39_MSK                     (0xFUL << SPU_ENV_CLK_CH39_POS)
#define SPU_ENV_CLK_CH39_0x0                     (0x0UL << SPU_ENV_CLK_CH39_POS)
#define SPU_ENV_CLK_CH39_0xF                     (0xFUL << SPU_ENV_CLK_CH39_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH40_11[3:0] -
 */
#define SPU_ENV_CLK_CH40_POS                     (0)
#define SPU_ENV_CLK_CH40_MSK                     (0xFUL << SPU_ENV_CLK_CH40_POS)
#define SPU_ENV_CLK_CH40_0x0                     (0x0UL << SPU_ENV_CLK_CH40_POS)
#define SPU_ENV_CLK_CH40_0xF                     (0xFUL << SPU_ENV_CLK_CH40_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH40_11[7:4] -
 */
#define SPU_ENV_CLK_CH41_POS                     (4)
#define SPU_ENV_CLK_CH41_MSK                     (0xFUL << SPU_ENV_CLK_CH41_POS)
#define SPU_ENV_CLK_CH41_0x0                     (0x0UL << SPU_ENV_CLK_CH41_POS)
#define SPU_ENV_CLK_CH41_0xF                     (0xFUL << SPU_ENV_CLK_CH41_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH40_11[11:8] -
 */
#define SPU_ENV_CLK_CH42_POS                     (8)
#define SPU_ENV_CLK_CH42_MSK                     (0xFUL << SPU_ENV_CLK_CH42_POS)
#define SPU_ENV_CLK_CH42_0x0                     (0x0UL << SPU_ENV_CLK_CH42_POS)
#define SPU_ENV_CLK_CH42_0xF                     (0xFUL << SPU_ENV_CLK_CH42_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH40_11[15:12] -
 */
#define SPU_ENV_CLK_CH43_POS                     (12)
#define SPU_ENV_CLK_CH43_MSK                     (0xFUL << SPU_ENV_CLK_CH43_POS)
#define SPU_ENV_CLK_CH43_0x0                     (0x0UL << SPU_ENV_CLK_CH43_POS)
#define SPU_ENV_CLK_CH43_0xF                     (0xFUL << SPU_ENV_CLK_CH43_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH44_15[3:0] -
 */
#define SPU_ENV_CLK_CH44_POS                     (0)
#define SPU_ENV_CLK_CH44_MSK                     (0xFUL << SPU_ENV_CLK_CH44_POS)
#define SPU_ENV_CLK_CH44_0x0                     (0x0UL << SPU_ENV_CLK_CH44_POS)
#define SPU_ENV_CLK_CH44_0xF                     (0xFUL << SPU_ENV_CLK_CH44_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH44_15[7:4] -
 */
#define SPU_ENV_CLK_CH45_POS                     (4)
#define SPU_ENV_CLK_CH45_MSK                     (0xFUL << SPU_ENV_CLK_CH45_POS)
#define SPU_ENV_CLK_CH45_0x0                     (0x0UL << SPU_ENV_CLK_CH45_POS)
#define SPU_ENV_CLK_CH45_0xF                     (0xFUL << SPU_ENV_CLK_CH45_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH44_15[11:8] -
 */
#define SPU_ENV_CLK_CH46_POS                     (8)
#define SPU_ENV_CLK_CH46_MSK                     (0xFUL << SPU_ENV_CLK_CH46_POS)
#define SPU_ENV_CLK_CH46_0x0                     (0x0UL << SPU_ENV_CLK_CH46_POS)
#define SPU_ENV_CLK_CH46_0xF                     (0xFUL << SPU_ENV_CLK_CH46_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH44_15[15:12] -
 */
#define SPU_ENV_CLK_CH47_POS                     (12)
#define SPU_ENV_CLK_CH47_MSK                     (0xFUL << SPU_ENV_CLK_CH47_POS)
#define SPU_ENV_CLK_CH47_0x0                     (0x0UL << SPU_ENV_CLK_CH47_POS)
#define SPU_ENV_CLK_CH47_0xF                     (0xFUL << SPU_ENV_CLK_CH47_POS)

/*
 * Bit definition for SPU->ENV_RAMPDOWN_CH32_47[15:0]
 */
#define SPU_ENV_RAMPDOWN_CH32_POS                (0)
#define SPU_ENV_RAMPDOWN_CH32_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH32_POS)
#define SPU_ENV_RAMPDOWN_CH32_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH32_POS)
#define SPU_ENV_RAMPDOWN_CH32_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH32_POS)
#define SPU_ENV_RAMPDOWN_CH33_POS                (1)
#define SPU_ENV_RAMPDOWN_CH33_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH33_POS)
#define SPU_ENV_RAMPDOWN_CH33_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH33_POS)
#define SPU_ENV_RAMPDOWN_CH33_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH33_POS)
#define SPU_ENV_RAMPDOWN_CH34_POS                (2)
#define SPU_ENV_RAMPDOWN_CH34_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH34_POS)
#define SPU_ENV_RAMPDOWN_CH34_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH34_POS)
#define SPU_ENV_RAMPDOWN_CH34_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH34_POS)
#define SPU_ENV_RAMPDOWN_CH35_POS                (3)
#define SPU_ENV_RAMPDOWN_CH35_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH35_POS)
#define SPU_ENV_RAMPDOWN_CH35_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH35_POS)
#define SPU_ENV_RAMPDOWN_CH35_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH35_POS)
#define SPU_ENV_RAMPDOWN_CH36_POS                (4)
#define SPU_ENV_RAMPDOWN_CH36_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH36_POS)
#define SPU_ENV_RAMPDOWN_CH36_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH36_POS)
#define SPU_ENV_RAMPDOWN_CH36_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH36_POS)
#define SPU_ENV_RAMPDOWN_CH37_POS                (5)
#define SPU_ENV_RAMPDOWN_CH37_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH37_POS)
#define SPU_ENV_RAMPDOWN_CH37_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH37_POS)
#define SPU_ENV_RAMPDOWN_CH37_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH37_POS)
#define SPU_ENV_RAMPDOWN_CH38_POS                (6)
#define SPU_ENV_RAMPDOWN_CH38_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH38_POS)
#define SPU_ENV_RAMPDOWN_CH38_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH38_POS)
#define SPU_ENV_RAMPDOWN_CH38_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH38_POS)
#define SPU_ENV_RAMPDOWN_CH39_POS                (7)
#define SPU_ENV_RAMPDOWN_CH39_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH39_POS)
#define SPU_ENV_RAMPDOWN_CH39_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH39_POS)
#define SPU_ENV_RAMPDOWN_CH39_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH39_POS)
#define SPU_ENV_RAMPDOWN_CH40_POS                (8)
#define SPU_ENV_RAMPDOWN_CH40_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH40_POS)
#define SPU_ENV_RAMPDOWN_CH40_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH40_POS)
#define SPU_ENV_RAMPDOWN_CH40_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH40_POS)
#define SPU_ENV_RAMPDOWN_CH41_POS                (9)
#define SPU_ENV_RAMPDOWN_CH41_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH41_POS)
#define SPU_ENV_RAMPDOWN_CH41_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH41_POS)
#define SPU_ENV_RAMPDOWN_CH41_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH41_POS)
#define SPU_ENV_RAMPDOWN_CH42_POS                (10)
#define SPU_ENV_RAMPDOWN_CH42_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH42_POS)
#define SPU_ENV_RAMPDOWN_CH42_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH42_POS)
#define SPU_ENV_RAMPDOWN_CH42_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH42_POS)
#define SPU_ENV_RAMPDOWN_CH43_POS                (11)
#define SPU_ENV_RAMPDOWN_CH43_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH43_POS)
#define SPU_ENV_RAMPDOWN_CH43_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH43_POS)
#define SPU_ENV_RAMPDOWN_CH43_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH43_POS)
#define SPU_ENV_RAMPDOWN_CH44_POS                (12)
#define SPU_ENV_RAMPDOWN_CH44_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH44_POS)
#define SPU_ENV_RAMPDOWN_CH44_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH44_POS)
#define SPU_ENV_RAMPDOWN_CH44_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH44_POS)
#define SPU_ENV_RAMPDOWN_CH45_POS                (13)
#define SPU_ENV_RAMPDOWN_CH45_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH45_POS)
#define SPU_ENV_RAMPDOWN_CH45_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH45_POS)
#define SPU_ENV_RAMPDOWN_CH45_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH45_POS)
#define SPU_ENV_RAMPDOWN_CH46_POS                (14)
#define SPU_ENV_RAMPDOWN_CH46_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH46_POS)
#define SPU_ENV_RAMPDOWN_CH46_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH46_POS)
#define SPU_ENV_RAMPDOWN_CH46_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH46_POS)
#define SPU_ENV_RAMPDOWN_CH47_POS                (15)
#define SPU_ENV_RAMPDOWN_CH47_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH47_POS)
#define SPU_ENV_RAMPDOWN_CH47_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH47_POS)
#define SPU_ENV_RAMPDOWN_CH47_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH47_POS)

/*
 * Bit definition for SPU->STOP_STATUS_CH32_47[15:0]
 */
#define SPU_STOP_FLAG_CH32_POS                   (0)
#define SPU_STOP_FLAG_CH32_MSK                   (0x1UL << SPU_STOP_FLAG_CH32_POS)
#define SPU_STOP_FLAG_CH32                       (0x1UL << SPU_STOP_FLAG_CH32_POS)
#define SPU_STOP_FLAG_CH33_POS                   (1)
#define SPU_STOP_FLAG_CH33_MSK                   (0x1UL << SPU_STOP_FLAG_CH33_POS)
#define SPU_STOP_FLAG_CH33                       (0x1UL << SPU_STOP_FLAG_CH33_POS)
#define SPU_STOP_FLAG_CH34_POS                   (2)
#define SPU_STOP_FLAG_CH34_MSK                   (0x1UL << SPU_STOP_FLAG_CH34_POS)
#define SPU_STOP_FLAG_CH34                       (0x1UL << SPU_STOP_FLAG_CH34_POS)
#define SPU_STOP_FLAG_CH35_POS                   (3)
#define SPU_STOP_FLAG_CH35_MSK                   (0x1UL << SPU_STOP_FLAG_CH35_POS)
#define SPU_STOP_FLAG_CH35                       (0x0UL << SPU_STOP_FLAG_CH35_POS)
#define SPU_STOP_FLAG_CH36_POS                   (4)
#define SPU_STOP_FLAG_CH36_MSK                   (0x1UL << SPU_STOP_FLAG_CH36_POS)
#define SPU_STOP_FLAG_CH36                       (0x0UL << SPU_STOP_FLAG_CH36_POS)
#define SPU_STOP_FLAG_CH37_POS                   (5)
#define SPU_STOP_FLAG_CH37_MSK                   (0x1UL << SPU_STOP_FLAG_CH37_POS)
#define SPU_STOP_FLAG_CH37                       (0x0UL << SPU_STOP_FLAG_CH37_POS)
#define SPU_STOP_FLAG_CH38_POS                   (6)
#define SPU_STOP_FLAG_CH38_MSK                   (0x1UL << SPU_STOP_FLAG_CH38_POS)
#define SPU_STOP_FLAG_CH38                       (0x0UL << SPU_STOP_FLAG_CH38_POS)
#define SPU_STOP_FLAG_CH39_POS                   (7)
#define SPU_STOP_FLAG_CH39_MSK                   (0x1UL << SPU_STOP_FLAG_CH39_POS)
#define SPU_STOP_FLAG_CH39                       (0x0UL << SPU_STOP_FLAG_CH39_POS)
#define SPU_STOP_FLAG_CH40_POS                   (8)
#define SPU_STOP_FLAG_CH40_MSK                   (0x1UL << SPU_STOP_FLAG_CH40_POS)
#define SPU_STOP_FLAG_CH40                       (0x0UL << SPU_STOP_FLAG_CH40_POS)
#define SPU_STOP_FLAG_CH41_POS                   (9)
#define SPU_STOP_FLAG_CH41_MSK                   (0x1UL << SPU_STOP_FLAG_CH41_POS)
#define SPU_STOP_FLAG_CH41                       (0x0UL << SPU_STOP_FLAG_CH41_POS)
#define SPU_STOP_FLAG_CH42_POS                   (10)
#define SPU_STOP_FLAG_CH42_MSK                   (0x1UL << SPU_STOP_FLAG_CH42_POS)
#define SPU_STOP_FLAG_CH42                       (0x0UL << SPU_STOP_FLAG_CH42_POS)
#define SPU_STOP_FLAG_CH43_POS                   (11)
#define SPU_STOP_FLAG_CH43_MSK                   (0x1UL << SPU_STOP_FLAG_CH43_POS)
#define SPU_STOP_FLAG_CH43                       (0x0UL << SPU_STOP_FLAG_CH43_POS)
#define SPU_STOP_FLAG_CH44_POS                   (12)
#define SPU_STOP_FLAG_CH44_MSK                   (0x1UL << SPU_STOP_FLAG_CH44_POS)
#define SPU_STOP_FLAG_CH44                       (0x0UL << SPU_STOP_FLAG_CH44_POS)
#define SPU_STOP_FLAG_CH45_POS                   (13)
#define SPU_STOP_FLAG_CH45_MSK                   (0x1UL << SPU_STOP_FLAG_CH45_POS)
#define SPU_STOP_FLAG_CH45                       (0x0UL << SPU_STOP_FLAG_CH45_POS)
#define SPU_STOP_FLAG_CH46_POS                   (14)
#define SPU_STOP_FLAG_CH46_MSK                   (0x1UL << SPU_STOP_FLAG_CH46_POS)
#define SPU_STOP_FLAG_CH46                       (0x0UL << SPU_STOP_FLAG_CH46_POS)
#define SPU_STOP_FLAG_CH47_POS                   (15)
#define SPU_STOP_FLAG_CH47_MSK                   (0x1UL << SPU_STOP_FLAG_CH47_POS)
#define SPU_STOP_FLAG_CH47                       (0x0UL << SPU_STOP_FLAG_CH47_POS)

/*
 * Bit definition for SPU->CH_STATUS_CH32_47[15:0]
 */
#define SPU_BUSY_FLAG_CH32_POS                   (0)
#define SPU_BUSY_FLAG_CH32_MSK                   (0x1UL << SPU_BUSY_FLAG_CH32_POS)
#define SPU_BUSY_FLAG_CH32                       (0x1UL << SPU_BUSY_FLAG_CH32_POS)
#define SPU_BUSY_FLAG_CH33_POS                   (1)
#define SPU_BUSY_FLAG_CH33_MSK                   (0x1UL << SPU_BUSY_FLAG_CH33_POS)
#define SPU_BUSY_FLAG_CH33                       (0x1UL << SPU_BUSY_FLAG_CH33_POS)
#define SPU_BUSY_FLAG_CH34_POS                   (2)
#define SPU_BUSY_FLAG_CH34_MSK                   (0x1UL << SPU_BUSY_FLAG_CH34_POS)
#define SPU_BUSY_FLAG_CH34                       (0x1UL << SPU_BUSY_FLAG_CH34_POS)
#define SPU_BUSY_FLAG_CH35_POS                   (3)
#define SPU_BUSY_FLAG_CH35_MSK                   (0x1UL << SPU_BUSY_FLAG_CH35_POS)
#define SPU_BUSY_FLAG_CH35                       (0x0UL << SPU_BUSY_FLAG_CH35_POS)
#define SPU_BUSY_FLAG_CH36_POS                   (4)
#define SPU_BUSY_FLAG_CH36_MSK                   (0x1UL << SPU_BUSY_FLAG_CH36_POS)
#define SPU_BUSY_FLAG_CH36                       (0x0UL << SPU_BUSY_FLAG_CH36_POS)
#define SPU_BUSY_FLAG_CH37_POS                   (5)
#define SPU_BUSY_FLAG_CH37_MSK                   (0x1UL << SPU_BUSY_FLAG_CH37_POS)
#define SPU_BUSY_FLAG_CH37                       (0x0UL << SPU_BUSY_FLAG_CH37_POS)
#define SPU_BUSY_FLAG_CH38_POS                   (6)
#define SPU_BUSY_FLAG_CH38_MSK                   (0x1UL << SPU_BUSY_FLAG_CH38_POS)
#define SPU_BUSY_FLAG_CH38                       (0x0UL << SPU_BUSY_FLAG_CH38_POS)
#define SPU_BUSY_FLAG_CH39_POS                   (7)
#define SPU_BUSY_FLAG_CH39_MSK                   (0x1UL << SPU_BUSY_FLAG_CH39_POS)
#define SPU_BUSY_FLAG_CH39                       (0x0UL << SPU_BUSY_FLAG_CH39_POS)
#define SPU_BUSY_FLAG_CH40_POS                   (8)
#define SPU_BUSY_FLAG_CH40_MSK                   (0x1UL << SPU_BUSY_FLAG_CH40_POS)
#define SPU_BUSY_FLAG_CH40                       (0x0UL << SPU_BUSY_FLAG_CH40_POS)
#define SPU_BUSY_FLAG_CH41_POS                   (9)
#define SPU_BUSY_FLAG_CH41_MSK                   (0x1UL << SPU_BUSY_FLAG_CH41_POS)
#define SPU_BUSY_FLAG_CH41                       (0x0UL << SPU_BUSY_FLAG_CH41_POS)
#define SPU_BUSY_FLAG_CH42_POS                   (10)
#define SPU_BUSY_FLAG_CH42_MSK                   (0x1UL << SPU_BUSY_FLAG_CH42_POS)
#define SPU_BUSY_FLAG_CH42                       (0x0UL << SPU_BUSY_FLAG_CH42_POS)
#define SPU_BUSY_FLAG_CH43_POS                   (11)
#define SPU_BUSY_FLAG_CH43_MSK                   (0x1UL << SPU_BUSY_FLAG_CH43_POS)
#define SPU_BUSY_FLAG_CH43                       (0x0UL << SPU_BUSY_FLAG_CH43_POS)
#define SPU_BUSY_FLAG_CH44_POS                   (12)
#define SPU_BUSY_FLAG_CH44_MSK                   (0x1UL << SPU_BUSY_FLAG_CH44_POS)
#define SPU_BUSY_FLAG_CH44                       (0x0UL << SPU_BUSY_FLAG_CH44_POS)
#define SPU_BUSY_FLAG_CH45_POS                   (13)
#define SPU_BUSY_FLAG_CH45_MSK                   (0x1UL << SPU_BUSY_FLAG_CH45_POS)
#define SPU_BUSY_FLAG_CH45                       (0x0UL << SPU_BUSY_FLAG_CH45_POS)
#define SPU_BUSY_FLAG_CH46_POS                   (14)
#define SPU_BUSY_FLAG_CH46_MSK                   (0x1UL << SPU_BUSY_FLAG_CH46_POS)
#define SPU_BUSY_FLAG_CH46                       (0x0UL << SPU_BUSY_FLAG_CH46_POS)
#define SPU_BUSY_FLAG_CH47_POS                   (15)
#define SPU_BUSY_FLAG_CH47_MSK                   (0x1UL << SPU_BUSY_FLAG_CH47_POS)
#define SPU_BUSY_FLAG_CH47                       (0x0UL << SPU_BUSY_FLAG_CH47_POS)

/*
 * Bit definition for SPU->ENV_REPEAT_CH32_47[15:0]
 */
#define SPU_ENV_REPEAT_CH32_POS                  (0)
#define SPU_ENV_REPEAT_CH32_MSK                  (0x1UL << SPU_ENV_REPEAT_CH32_POS)
#define SPU_ENV_REPEAT_CH32_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH32_POS)
#define SPU_ENV_REPEAT_CH32_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH32_POS)
#define SPU_ENV_REPEAT_CH33_POS                  (1)
#define SPU_ENV_REPEAT_CH33_MSK                  (0x1UL << SPU_ENV_REPEAT_CH33_POS)
#define SPU_ENV_REPEAT_CH33_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH33_POS)
#define SPU_ENV_REPEAT_CH33_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH33_POS)
#define SPU_ENV_REPEAT_CH34_POS                  (2)
#define SPU_ENV_REPEAT_CH34_MSK                  (0x1UL << SPU_ENV_REPEAT_CH34_POS)
#define SPU_ENV_REPEAT_CH34_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH34_POS)
#define SPU_ENV_REPEAT_CH34_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH34_POS)
#define SPU_ENV_REPEAT_CH35_POS                  (3)
#define SPU_ENV_REPEAT_CH35_MSK                  (0x1UL << SPU_ENV_REPEAT_CH35_POS)
#define SPU_ENV_REPEAT_CH35_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH35_POS)
#define SPU_ENV_REPEAT_CH35_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH35_POS)
#define SPU_ENV_REPEAT_CH36_POS                  (4)
#define SPU_ENV_REPEAT_CH36_MSK                  (0x1UL << SPU_ENV_REPEAT_CH36_POS)
#define SPU_ENV_REPEAT_CH36_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH36_POS)
#define SPU_ENV_REPEAT_CH36_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH36_POS)
#define SPU_ENV_REPEAT_CH37_POS                  (5)
#define SPU_ENV_REPEAT_CH37_MSK                  (0x1UL << SPU_ENV_REPEAT_CH37_POS)
#define SPU_ENV_REPEAT_CH37_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH37_POS)
#define SPU_ENV_REPEAT_CH37_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH37_POS)
#define SPU_ENV_REPEAT_CH38_POS                  (6)
#define SPU_ENV_REPEAT_CH38_MSK                  (0x1UL << SPU_ENV_REPEAT_CH38_POS)
#define SPU_ENV_REPEAT_CH38_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH38_POS)
#define SPU_ENV_REPEAT_CH38_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH38_POS)
#define SPU_ENV_REPEAT_CH39_POS                  (7)
#define SPU_ENV_REPEAT_CH39_MSK                  (0x1UL << SPU_ENV_REPEAT_CH39_POS)
#define SPU_ENV_REPEAT_CH39_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH39_POS)
#define SPU_ENV_REPEAT_CH39_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH39_POS)
#define SPU_ENV_REPEAT_CH40_POS                  (8)
#define SPU_ENV_REPEAT_CH40_MSK                  (0x1UL << SPU_ENV_REPEAT_CH40_POS)
#define SPU_ENV_REPEAT_CH40_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH40_POS)
#define SPU_ENV_REPEAT_CH40_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH40_POS)
#define SPU_ENV_REPEAT_CH41_POS                  (9)
#define SPU_ENV_REPEAT_CH41_MSK                  (0x1UL << SPU_ENV_REPEAT_CH41_POS)
#define SPU_ENV_REPEAT_CH41_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH41_POS)
#define SPU_ENV_REPEAT_CH41_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH41_POS)
#define SPU_ENV_REPEAT_CH42_POS                  (10)
#define SPU_ENV_REPEAT_CH42_MSK                  (0x1UL << SPU_ENV_REPEAT_CH42_POS)
#define SPU_ENV_REPEAT_CH42_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH42_POS)
#define SPU_ENV_REPEAT_CH42_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH42_POS)
#define SPU_ENV_REPEAT_CH43_POS                  (11)
#define SPU_ENV_REPEAT_CH43_MSK                  (0x1UL << SPU_ENV_REPEAT_CH43_POS)
#define SPU_ENV_REPEAT_CH43_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH43_POS)
#define SPU_ENV_REPEAT_CH43_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH43_POS)
#define SPU_ENV_REPEAT_CH44_POS                  (12)
#define SPU_ENV_REPEAT_CH44_MSK                  (0x1UL << SPU_ENV_REPEAT_CH44_POS)
#define SPU_ENV_REPEAT_CH44_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH44_POS)
#define SPU_ENV_REPEAT_CH44_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH44_POS)
#define SPU_ENV_REPEAT_CH45_POS                  (13)
#define SPU_ENV_REPEAT_CH45_MSK                  (0x1UL << SPU_ENV_REPEAT_CH45_POS)
#define SPU_ENV_REPEAT_CH45_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH45_POS)
#define SPU_ENV_REPEAT_CH45_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH45_POS)
#define SPU_ENV_REPEAT_CH46_POS                  (14)
#define SPU_ENV_REPEAT_CH46_MSK                  (0x1UL << SPU_ENV_REPEAT_CH46_POS)
#define SPU_ENV_REPEAT_CH46_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH46_POS)
#define SPU_ENV_REPEAT_CH46_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH46_POS)
#define SPU_ENV_REPEAT_CH47_POS                  (15)
#define SPU_ENV_REPEAT_CH47_MSK                  (0x1UL << SPU_ENV_REPEAT_CH47_POS)
#define SPU_ENV_REPEAT_CH47_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH47_POS)
#define SPU_ENV_REPEAT_CH47_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH47_POS)

/*
 * Bit definition for SPU->ENV_MODE_CH32_47[15:0]
 */
#define SPU_ENV_MODE_CH32_POS                    (0)
#define SPU_ENV_MODE_CH32_MSK                    (0x1UL << SPU_ENV_MODE_CH32_POS)
#define SPU_ENV_MODE_CH32_MANUAL                 (0x1UL << SPU_ENV_MODE_CH32_POS)
#define SPU_ENV_MODE_CH32_AUTO                   (0x0UL << SPU_ENV_MODE_CH32_POS)
#define SPU_ENV_MODE_CH33_POS                    (1)
#define SPU_ENV_MODE_CH33_MSK                    (0x1UL << SPU_ENV_MODE_CH33_POS)
#define SPU_ENV_MODE_CH33_MANUAL                 (0x1UL << SPU_ENV_MODE_CH33_POS)
#define SPU_ENV_MODE_CH33_AUTO                   (0x0UL << SPU_ENV_MODE_CH33_POS)
#define SPU_ENV_MODE_CH34_POS                    (2)
#define SPU_ENV_MODE_CH34_MSK                    (0x1UL << SPU_ENV_MODE_CH34_POS)
#define SPU_ENV_MODE_CH34_MANUAL                 (0x1UL << SPU_ENV_MODE_CH34_POS)
#define SPU_ENV_MODE_CH34_AUTO                   (0x0UL << SPU_ENV_MODE_CH34_POS)
#define SPU_ENV_MODE_CH35_POS                    (3)
#define SPU_ENV_MODE_CH35_MSK                    (0x1UL << SPU_ENV_MODE_CH35_POS)
#define SPU_ENV_MODE_CH35_MANUAL                 (0x1UL << SPU_ENV_MODE_CH35_POS)
#define SPU_ENV_MODE_CH35_AUTO                   (0x0UL << SPU_ENV_MODE_CH35_POS)
#define SPU_ENV_MODE_CH36_POS                    (4)
#define SPU_ENV_MODE_CH36_MSK                    (0x1UL << SPU_ENV_MODE_CH36_POS)
#define SPU_ENV_MODE_CH36_MANUAL                 (0x1UL << SPU_ENV_MODE_CH36_POS)
#define SPU_ENV_MODE_CH36_AUTO                   (0x0UL << SPU_ENV_MODE_CH36_POS)
#define SPU_ENV_MODE_CH37_POS                    (5)
#define SPU_ENV_MODE_CH37_MSK                    (0x1UL << SPU_ENV_MODE_CH37_POS)
#define SPU_ENV_MODE_CH37_MANUAL                 (0x1UL << SPU_ENV_MODE_CH37_POS)
#define SPU_ENV_MODE_CH37_AUTO                   (0x0UL << SPU_ENV_MODE_CH37_POS)
#define SPU_ENV_MODE_CH38_POS                    (6)
#define SPU_ENV_MODE_CH38_MSK                    (0x1UL << SPU_ENV_MODE_CH38_POS)
#define SPU_ENV_MODE_CH38_MANUAL                 (0x1UL << SPU_ENV_MODE_CH38_POS)
#define SPU_ENV_MODE_CH38_AUTO                   (0x0UL << SPU_ENV_MODE_CH38_POS)
#define SPU_ENV_MODE_CH39_POS                    (7)
#define SPU_ENV_MODE_CH39_MSK                    (0x1UL << SPU_ENV_MODE_CH39_POS)
#define SPU_ENV_MODE_CH39_MANUAL                 (0x1UL << SPU_ENV_MODE_CH39_POS)
#define SPU_ENV_MODE_CH39_AUTO                   (0x0UL << SPU_ENV_MODE_CH39_POS)
#define SPU_ENV_MODE_CH40_POS                    (8)
#define SPU_ENV_MODE_CH40_MSK                    (0x1UL << SPU_ENV_MODE_CH40_POS)
#define SPU_ENV_MODE_CH40_MANUAL                 (0x1UL << SPU_ENV_MODE_CH40_POS)
#define SPU_ENV_MODE_CH40_AUTO                   (0x0UL << SPU_ENV_MODE_CH40_POS)
#define SPU_ENV_MODE_CH41_POS                    (9)
#define SPU_ENV_MODE_CH41_MSK                    (0x1UL << SPU_ENV_MODE_CH41_POS)
#define SPU_ENV_MODE_CH41_MANUAL                 (0x1UL << SPU_ENV_MODE_CH41_POS)
#define SPU_ENV_MODE_CH41_AUTO                   (0x0UL << SPU_ENV_MODE_CH41_POS)
#define SPU_ENV_MODE_CH42_POS                    (10)
#define SPU_ENV_MODE_CH42_MSK                    (0x1UL << SPU_ENV_MODE_CH42_POS)
#define SPU_ENV_MODE_CH42_MANUAL                 (0x1UL << SPU_ENV_MODE_CH42_POS)
#define SPU_ENV_MODE_CH42_AUTO                   (0x0UL << SPU_ENV_MODE_CH42_POS)
#define SPU_ENV_MODE_CH43_POS                    (11)
#define SPU_ENV_MODE_CH43_MSK                    (0x1UL << SPU_ENV_MODE_CH43_POS)
#define SPU_ENV_MODE_CH43_MANUAL                 (0x1UL << SPU_ENV_MODE_CH43_POS)
#define SPU_ENV_MODE_CH43_AUTO                   (0x0UL << SPU_ENV_MODE_CH43_POS)
#define SPU_ENV_MODE_CH44_POS                    (12)
#define SPU_ENV_MODE_CH44_MSK                    (0x1UL << SPU_ENV_MODE_CH44_POS)
#define SPU_ENV_MODE_CH44_MANUAL                 (0x1UL << SPU_ENV_MODE_CH44_POS)
#define SPU_ENV_MODE_CH44_AUTO                   (0x0UL << SPU_ENV_MODE_CH44_POS)
#define SPU_ENV_MODE_CH45_POS                    (13)
#define SPU_ENV_MODE_CH45_MSK                    (0x1UL << SPU_ENV_MODE_CH45_POS)
#define SPU_ENV_MODE_CH45_MANUAL                 (0x1UL << SPU_ENV_MODE_CH45_POS)
#define SPU_ENV_MODE_CH45_AUTO                   (0x0UL << SPU_ENV_MODE_CH45_POS)
#define SPU_ENV_MODE_CH46_POS                    (14)
#define SPU_ENV_MODE_CH46_MSK                    (0x1UL << SPU_ENV_MODE_CH46_POS)
#define SPU_ENV_MODE_CH46_MANUAL                 (0x1UL << SPU_ENV_MODE_CH46_POS)
#define SPU_ENV_MODE_CH46_AUTO                   (0x0UL << SPU_ENV_MODE_CH46_POS)
#define SPU_ENV_MODE_CH47_POS                    (15)
#define SPU_ENV_MODE_CH47_MSK                    (0x1UL << SPU_ENV_MODE_CH47_POS)
#define SPU_ENV_MODE_CH47_MANUAL                 (0x1UL << SPU_ENV_MODE_CH47_POS)
#define SPU_ENV_MODE_CH47_AUTO                   (0x0UL << SPU_ENV_MODE_CH47_POS)

/*
 * Bit definition for SPU->TONE_RELEASE_CH32_47[15:0]
 */
#define SPU_TONE_RELEASE_CH32_POS                (0)
#define SPU_TONE_RELEASE_CH32_MSK                (0x1UL << SPU_TONE_RELEASE_CH32_POS)
#define SPU_TONE_RELEASE_CH32_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH32_POS)
#define SPU_TONE_RELEASE_CH32_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH32_POS)
#define SPU_TONE_RELEASE_CH33_POS                (1)
#define SPU_TONE_RELEASE_CH33_MSK                (0x1UL << SPU_TONE_RELEASE_CH33_POS)
#define SPU_TONE_RELEASE_CH33_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH33_POS)
#define SPU_TONE_RELEASE_CH33_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH33_POS)
#define SPU_TONE_RELEASE_CH34_POS                (2)
#define SPU_TONE_RELEASE_CH34_MSK                (0x1UL << SPU_TONE_RELEASE_CH34_POS)
#define SPU_TONE_RELEASE_CH34_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH34_POS)
#define SPU_TONE_RELEASE_CH34_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH34_POS)
#define SPU_TONE_RELEASE_CH35_POS                (3)
#define SPU_TONE_RELEASE_CH35_MSK                (0x1UL << SPU_TONE_RELEASE_CH35_POS)
#define SPU_TONE_RELEASE_CH35_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH35_POS)
#define SPU_TONE_RELEASE_CH35_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH35_POS)
#define SPU_TONE_RELEASE_CH36_POS                (4)
#define SPU_TONE_RELEASE_CH36_MSK                (0x1UL << SPU_TONE_RELEASE_CH36_POS)
#define SPU_TONE_RELEASE_CH36_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH36_POS)
#define SPU_TONE_RELEASE_CH36_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH36_POS)
#define SPU_TONE_RELEASE_CH37_POS                (5)
#define SPU_TONE_RELEASE_CH37_MSK                (0x1UL << SPU_TONE_RELEASE_CH37_POS)
#define SPU_TONE_RELEASE_CH37_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH37_POS)
#define SPU_TONE_RELEASE_CH37_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH37_POS)
#define SPU_TONE_RELEASE_CH38_POS                (6)
#define SPU_TONE_RELEASE_CH38_MSK                (0x1UL << SPU_TONE_RELEASE_CH38_POS)
#define SPU_TONE_RELEASE_CH38_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH38_POS)
#define SPU_TONE_RELEASE_CH38_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH38_POS)
#define SPU_TONE_RELEASE_CH39_POS                (7)
#define SPU_TONE_RELEASE_CH39_MSK                (0x1UL << SPU_TONE_RELEASE_CH39_POS)
#define SPU_TONE_RELEASE_CH39_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH39_POS)
#define SPU_TONE_RELEASE_CH39_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH39_POS)
#define SPU_TONE_RELEASE_CH40_POS                (8)
#define SPU_TONE_RELEASE_CH40_MSK                (0x1UL << SPU_TONE_RELEASE_CH40_POS)
#define SPU_TONE_RELEASE_CH40_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH40_POS)
#define SPU_TONE_RELEASE_CH40_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH40_POS)
#define SPU_TONE_RELEASE_CH41_POS                (9)
#define SPU_TONE_RELEASE_CH41_MSK                (0x1UL << SPU_TONE_RELEASE_CH41_POS)
#define SPU_TONE_RELEASE_CH41_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH41_POS)
#define SPU_TONE_RELEASE_CH41_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH41_POS)
#define SPU_TONE_RELEASE_CH42_POS                (10)
#define SPU_TONE_RELEASE_CH42_MSK                (0x1UL << SPU_TONE_RELEASE_CH42_POS)
#define SPU_TONE_RELEASE_CH42_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH42_POS)
#define SPU_TONE_RELEASE_CH42_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH42_POS)
#define SPU_TONE_RELEASE_CH43_POS                (11)
#define SPU_TONE_RELEASE_CH43_MSK                (0x1UL << SPU_TONE_RELEASE_CH43_POS)
#define SPU_TONE_RELEASE_CH43_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH43_POS)
#define SPU_TONE_RELEASE_CH43_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH43_POS)
#define SPU_TONE_RELEASE_CH44_POS                (12)
#define SPU_TONE_RELEASE_CH44_MSK                (0x1UL << SPU_TONE_RELEASE_CH44_POS)
#define SPU_TONE_RELEASE_CH44_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH44_POS)
#define SPU_TONE_RELEASE_CH44_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH44_POS)
#define SPU_TONE_RELEASE_CH45_POS                (13)
#define SPU_TONE_RELEASE_CH45_MSK                (0x1UL << SPU_TONE_RELEASE_CH45_POS)
#define SPU_TONE_RELEASE_CH45_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH45_POS)
#define SPU_TONE_RELEASE_CH45_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH45_POS)
#define SPU_TONE_RELEASE_CH46_POS                (14)
#define SPU_TONE_RELEASE_CH46_MSK                (0x1UL << SPU_TONE_RELEASE_CH46_POS)
#define SPU_TONE_RELEASE_CH46_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH46_POS)
#define SPU_TONE_RELEASE_CH46_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH46_POS)
#define SPU_TONE_RELEASE_CH47_POS                (15)
#define SPU_TONE_RELEASE_CH47_MSK                (0x1UL << SPU_TONE_RELEASE_CH47_POS)
#define SPU_TONE_RELEASE_CH47_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH47_POS)
#define SPU_TONE_RELEASE_CH47_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH47_POS)

/*
 * Bit definition for SPU->ENV_INT_STATUS_CH32_47[15:0]
 */
#define SPU_ENV_INT_FLAG_CH32_POS                (0)
#define SPU_ENV_INT_FLAG_CH32_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH32_POS)
#define SPU_ENV_INT_FLAG_CH32                    (0x1UL << SPU_ENV_INT_FLAG_CH32_POS)
#define SPU_ENV_INT_FLAG_CH33_POS                (1)
#define SPU_ENV_INT_FLAG_CH33_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH33_POS)
#define SPU_ENV_INT_FLAG_CH33                    (0x1UL << SPU_ENV_INT_FLAG_CH33_POS)
#define SPU_ENV_INT_FLAG_CH34_POS                (2)
#define SPU_ENV_INT_FLAG_CH34_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH34_POS)
#define SPU_ENV_INT_FLAG_CH34                    (0x1UL << SPU_ENV_INT_FLAG_CH34_POS)
#define SPU_ENV_INT_FLAG_CH35_POS                (3)
#define SPU_ENV_INT_FLAG_CH35_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH35_POS)
#define SPU_ENV_INT_FLAG_CH35                    (0x0UL << SPU_ENV_INT_FLAG_CH35_POS)
#define SPU_ENV_INT_FLAG_CH36_POS                (4)
#define SPU_ENV_INT_FLAG_CH36_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH36_POS)
#define SPU_ENV_INT_FLAG_CH36                    (0x0UL << SPU_ENV_INT_FLAG_CH36_POS)
#define SPU_ENV_INT_FLAG_CH37_POS                (5)
#define SPU_ENV_INT_FLAG_CH37_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH37_POS)
#define SPU_ENV_INT_FLAG_CH37                    (0x0UL << SPU_ENV_INT_FLAG_CH37_POS)
#define SPU_ENV_INT_FLAG_CH38_POS                (6)
#define SPU_ENV_INT_FLAG_CH38_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH38_POS)
#define SPU_ENV_INT_FLAG_CH38                    (0x0UL << SPU_ENV_INT_FLAG_CH38_POS)
#define SPU_ENV_INT_FLAG_CH39_POS                (7)
#define SPU_ENV_INT_FLAG_CH39_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH39_POS)
#define SPU_ENV_INT_FLAG_CH39                    (0x0UL << SPU_ENV_INT_FLAG_CH39_POS)
#define SPU_ENV_INT_FLAG_CH40_POS                (8)
#define SPU_ENV_INT_FLAG_CH40_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH40_POS)
#define SPU_ENV_INT_FLAG_CH40                    (0x0UL << SPU_ENV_INT_FLAG_CH40_POS)
#define SPU_ENV_INT_FLAG_CH41_POS                (9)
#define SPU_ENV_INT_FLAG_CH41_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH41_POS)
#define SPU_ENV_INT_FLAG_CH41                    (0x0UL << SPU_ENV_INT_FLAG_CH41_POS)
#define SPU_ENV_INT_FLAG_CH42_POS                (10)
#define SPU_ENV_INT_FLAG_CH42_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH42_POS)
#define SPU_ENV_INT_FLAG_CH42                    (0x0UL << SPU_ENV_INT_FLAG_CH42_POS)
#define SPU_ENV_INT_FLAG_CH43_POS                (11)
#define SPU_ENV_INT_FLAG_CH43_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH43_POS)
#define SPU_ENV_INT_FLAG_CH43                    (0x0UL << SPU_ENV_INT_FLAG_CH43_POS)
#define SPU_ENV_INT_FLAG_CH44_POS                (12)
#define SPU_ENV_INT_FLAG_CH44_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH44_POS)
#define SPU_ENV_INT_FLAG_CH44                    (0x0UL << SPU_ENV_INT_FLAG_CH44_POS)
#define SPU_ENV_INT_FLAG_CH45_POS                (13)
#define SPU_ENV_INT_FLAG_CH45_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH45_POS)
#define SPU_ENV_INT_FLAG_CH45                    (0x0UL << SPU_ENV_INT_FLAG_CH45_POS)
#define SPU_ENV_INT_FLAG_CH46_POS                (14)
#define SPU_ENV_INT_FLAG_CH46_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH46_POS)
#define SPU_ENV_INT_FLAG_CH46                    (0x0UL << SPU_ENV_INT_FLAG_CH46_POS)
#define SPU_ENV_INT_FLAG_CH47_POS                (15)
#define SPU_ENV_INT_FLAG_CH47_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH47_POS)
#define SPU_ENV_INT_FLAG_CH47                    (0x0UL << SPU_ENV_INT_FLAG_CH47_POS)

/*
 * Bit definition for SPU->PITCHBEND_EN_CH32_47[15:0]
 */
#define SPU_PITCHBEND_CH32_POS                   (0)
#define SPU_PITCHBEND_CH32_MSK                   (0x1UL << SPU_PITCHBEND_CH32_POS)
#define SPU_PITCHBEND_CH32_ENABLE                (0x1UL << SPU_PITCHBEND_CH32_POS)
#define SPU_PITCHBEND_CH32_DISABLE               (0x0UL << SPU_PITCHBEND_CH32_POS)
#define SPU_PITCHBEND_CH33_POS                   (1)
#define SPU_PITCHBEND_CH33_MSK                   (0x1UL << SPU_PITCHBEND_CH33_POS)
#define SPU_PITCHBEND_CH33_ENABLE                (0x1UL << SPU_PITCHBEND_CH33_POS)
#define SPU_PITCHBEND_CH33_DISABLE               (0x0UL << SPU_PITCHBEND_CH33_POS)
#define SPU_PITCHBEND_CH34_POS                   (2)
#define SPU_PITCHBEND_CH34_MSK                   (0x1UL << SPU_PITCHBEND_CH34_POS)
#define SPU_PITCHBEND_CH34_ENABLE                (0x1UL << SPU_PITCHBEND_CH34_POS)
#define SPU_PITCHBEND_CH34_DISABLE               (0x0UL << SPU_PITCHBEND_CH34_POS)
#define SPU_PITCHBEND_CH35_POS                   (3)
#define SPU_PITCHBEND_CH35_MSK                   (0x1UL << SPU_PITCHBEND_CH35_POS)
#define SPU_PITCHBEND_CH35_ENABLE                (0x1UL << SPU_PITCHBEND_CH35_POS)
#define SPU_PITCHBEND_CH35_DISABLE               (0x0UL << SPU_PITCHBEND_CH35_POS)
#define SPU_PITCHBEND_CH36_POS                   (4)
#define SPU_PITCHBEND_CH36_MSK                   (0x1UL << SPU_PITCHBEND_CH36_POS)
#define SPU_PITCHBEND_CH36_ENABLE                (0x1UL << SPU_PITCHBEND_CH36_POS)
#define SPU_PITCHBEND_CH36_DISABLE               (0x0UL << SPU_PITCHBEND_CH36_POS)
#define SPU_PITCHBEND_CH37_POS                   (5)
#define SPU_PITCHBEND_CH37_MSK                   (0x1UL << SPU_PITCHBEND_CH37_POS)
#define SPU_PITCHBEND_CH37_ENABLE                (0x1UL << SPU_PITCHBEND_CH37_POS)
#define SPU_PITCHBEND_CH37_DISABLE               (0x0UL << SPU_PITCHBEND_CH37_POS)
#define SPU_PITCHBEND_CH38_POS                   (6)
#define SPU_PITCHBEND_CH38_MSK                   (0x1UL << SPU_PITCHBEND_CH38_POS)
#define SPU_PITCHBEND_CH38_ENABLE                (0x1UL << SPU_PITCHBEND_CH38_POS)
#define SPU_PITCHBEND_CH38_DISABLE               (0x0UL << SPU_PITCHBEND_CH38_POS)
#define SPU_PITCHBEND_CH39_POS                   (7)
#define SPU_PITCHBEND_CH39_MSK                   (0x1UL << SPU_PITCHBEND_CH39_POS)
#define SPU_PITCHBEND_CH39_ENABLE                (0x1UL << SPU_PITCHBEND_CH39_POS)
#define SPU_PITCHBEND_CH39_DISABLE               (0x0UL << SPU_PITCHBEND_CH39_POS)
#define SPU_PITCHBEND_CH40_POS                   (8)
#define SPU_PITCHBEND_CH40_MSK                   (0x1UL << SPU_PITCHBEND_CH40_POS)
#define SPU_PITCHBEND_CH40_ENABLE                (0x1UL << SPU_PITCHBEND_CH40_POS)
#define SPU_PITCHBEND_CH40_DISABLE               (0x0UL << SPU_PITCHBEND_CH40_POS)
#define SPU_PITCHBEND_CH41_POS                   (9)
#define SPU_PITCHBEND_CH41_MSK                   (0x1UL << SPU_PITCHBEND_CH41_POS)
#define SPU_PITCHBEND_CH41_ENABLE                (0x1UL << SPU_PITCHBEND_CH41_POS)
#define SPU_PITCHBEND_CH41_DISABLE               (0x0UL << SPU_PITCHBEND_CH41_POS)
#define SPU_PITCHBEND_CH42_POS                   (10)
#define SPU_PITCHBEND_CH42_MSK                   (0x1UL << SPU_PITCHBEND_CH42_POS)
#define SPU_PITCHBEND_CH42_ENABLE                (0x1UL << SPU_PITCHBEND_CH42_POS)
#define SPU_PITCHBEND_CH42_DISABLE               (0x0UL << SPU_PITCHBEND_CH42_POS)
#define SPU_PITCHBEND_CH43_POS                   (11)
#define SPU_PITCHBEND_CH43_MSK                   (0x1UL << SPU_PITCHBEND_CH43_POS)
#define SPU_PITCHBEND_CH43_ENABLE                (0x1UL << SPU_PITCHBEND_CH43_POS)
#define SPU_PITCHBEND_CH43_DISABLE               (0x0UL << SPU_PITCHBEND_CH43_POS)
#define SPU_PITCHBEND_CH44_POS                   (12)
#define SPU_PITCHBEND_CH44_MSK                   (0x1UL << SPU_PITCHBEND_CH44_POS)
#define SPU_PITCHBEND_CH44_ENABLE                (0x1UL << SPU_PITCHBEND_CH44_POS)
#define SPU_PITCHBEND_CH44_DISABLE               (0x0UL << SPU_PITCHBEND_CH44_POS)
#define SPU_PITCHBEND_CH45_POS                   (13)
#define SPU_PITCHBEND_CH45_MSK                   (0x1UL << SPU_PITCHBEND_CH45_POS)
#define SPU_PITCHBEND_CH45_ENABLE                (0x1UL << SPU_PITCHBEND_CH45_POS)
#define SPU_PITCHBEND_CH45_DISABLE               (0x0UL << SPU_PITCHBEND_CH45_POS)
#define SPU_PITCHBEND_CH46_POS                   (14)
#define SPU_PITCHBEND_CH46_MSK                   (0x1UL << SPU_PITCHBEND_CH46_POS)
#define SPU_PITCHBEND_CH46_ENABLE                (0x1UL << SPU_PITCHBEND_CH46_POS)
#define SPU_PITCHBEND_CH46_DISABLE               (0x0UL << SPU_PITCHBEND_CH46_POS)
#define SPU_PITCHBEND_CH47_POS                   (15)
#define SPU_PITCHBEND_CH47_MSK                   (0x1UL << SPU_PITCHBEND_CH47_POS)
#define SPU_PITCHBEND_CH47_ENABLE                (0x1UL << SPU_PITCHBEND_CH47_POS)
#define SPU_PITCHBEND_CH47_DISABLE               (0x0UL << SPU_PITCHBEND_CH47_POS)

/*
 * Bit definition for SPU->SPU_EN_CH48_63[15:0]
 */
#define SPU_CH48_EN_POS                          (0)
#define SPU_CH48_EN_MSK                          (0x1UL << SPU_CH48_EN_POS)
#define SPU_CH48_ENABLE                          (0x1UL << SPU_CH48_EN_POS)
#define SPU_CH48_DISABLE                         (0x0UL << SPU_CH48_EN_POS)
#define SPU_CH49_EN_POS                          (1)
#define SPU_CH49_EN_MSK                          (0x1UL << SPU_CH49_EN_POS)
#define SPU_CH49_ENABLE                          (0x1UL << SPU_CH49_EN_POS)
#define SPU_CH49_DISABLE                         (0x0UL << SPU_CH49_EN_POS)
#define SPU_CH50_EN_POS                          (2)
#define SPU_CH50_EN_MSK                          (0x1UL << SPU_CH50_EN_POS)
#define SPU_CH50_ENABLE                          (0x1UL << SPU_CH50_EN_POS)
#define SPU_CH50_DISABLE                         (0x0UL << SPU_CH50_EN_POS)
#define SPU_CH51_EN_POS                          (3)
#define SPU_CH51_EN_MSK                          (0x1UL << SPU_CH51_EN_POS)
#define SPU_CH51_ENABLE                          (0x1UL << SPU_CH51_EN_POS)
#define SPU_CH51_DISABLE                         (0x0UL << SPU_CH51_EN_POS)
#define SPU_CH52_EN_POS                          (4)
#define SPU_CH52_EN_MSK                          (0x1UL << SPU_CH52_EN_POS)
#define SPU_CH52_ENABLE                          (0x1UL << SPU_CH52_EN_POS)
#define SPU_CH52_DISABLE                         (0x0UL << SPU_CH52_EN_POS)
#define SPU_CH53_EN_POS                          (5)
#define SPU_CH53_EN_MSK                          (0x1UL << SPU_CH53_EN_POS)
#define SPU_CH53_ENABLE                          (0x1UL << SPU_CH53_EN_POS)
#define SPU_CH53_DISABLE                         (0x0UL << SPU_CH53_EN_POS)
#define SPU_CH54_EN_POS                          (6)
#define SPU_CH54_EN_MSK                          (0x1UL << SPU_CH54_EN_POS)
#define SPU_CH54_ENABLE                          (0x1UL << SPU_CH54_EN_POS)
#define SPU_CH54_DISABLE                         (0x0UL << SPU_CH54_EN_POS)
#define SPU_CH55_EN_POS                          (7)
#define SPU_CH55_EN_MSK                          (0x1UL << SPU_CH55_EN_POS)
#define SPU_CH55_ENABLE                          (0x1UL << SPU_CH55_EN_POS)
#define SPU_CH55_DISABLE                         (0x0UL << SPU_CH55_EN_POS)
#define SPU_CH56_EN_POS                          (8)
#define SPU_CH56_EN_MSK                          (0x1UL << SPU_CH56_EN_POS)
#define SPU_CH56_ENABLE                          (0x1UL << SPU_CH56_EN_POS)
#define SPU_CH56_DISABLE                         (0x0UL << SPU_CH56_EN_POS)
#define SPU_CH57_EN_POS                          (9)
#define SPU_CH57_EN_MSK                          (0x1UL << SPU_CH57_EN_POS)
#define SPU_CH57_ENABLE                          (0x1UL << SPU_CH57_EN_POS)
#define SPU_CH57_DISABLE                         (0x0UL << SPU_CH57_EN_POS)
#define SPU_CH58_EN_POS                          (10)
#define SPU_CH58_EN_MSK                          (0x1UL << SPU_CH58_EN_POS)
#define SPU_CH58_ENABLE                          (0x1UL << SPU_CH58_EN_POS)
#define SPU_CH58_DISABLE                         (0x0UL << SPU_CH58_EN_POS)
#define SPU_CH59_EN_POS                          (11)
#define SPU_CH59_EN_MSK                          (0x1UL << SPU_CH59_EN_POS)
#define SPU_CH59_ENABLE                          (0x1UL << SPU_CH59_EN_POS)
#define SPU_CH59_DISABLE                         (0x0UL << SPU_CH59_EN_POS)
#define SPU_CH60_EN_POS                          (12)
#define SPU_CH60_EN_MSK                          (0x1UL << SPU_CH60_EN_POS)
#define SPU_CH60_ENABLE                          (0x1UL << SPU_CH60_EN_POS)
#define SPU_CH60_DISABLE                         (0x0UL << SPU_CH60_EN_POS)
#define SPU_CH61_EN_POS                          (13)
#define SPU_CH61_EN_MSK                          (0x1UL << SPU_CH61_EN_POS)
#define SPU_CH61_ENABLE                          (0x1UL << SPU_CH61_EN_POS)
#define SPU_CH61_DISABLE                         (0x0UL << SPU_CH61_EN_POS)
#define SPU_CH62_EN_POS                          (14)
#define SPU_CH62_EN_MSK                          (0x1UL << SPU_CH62_EN_POS)
#define SPU_CH62_ENABLE                          (0x1UL << SPU_CH62_EN_POS)
#define SPU_CH62_DISABLE                         (0x0UL << SPU_CH62_EN_POS)
#define SPU_CH63_EN_POS                          (15)
#define SPU_CH63_EN_MSK                          (0x1UL << SPU_CH63_EN_POS)
#define SPU_CH63_ENABLE                          (0x1UL << SPU_CH63_EN_POS)
#define SPU_CH63_DISABLE                         (0x0UL << SPU_CH63_EN_POS)

/*
 * Bit definition for SPU->INT_EN_CH48_63[15:0] - SPU Channel FIQ Enable
 */
#define SPU_INT_CH48_EN_POS                      (0)
#define SPU_INT_CH48_EN_MSK                      (0x1UL << SPU_INT_CH48_EN_POS)
#define SPU_INT_CH48_ENABLE                      (0x1UL << SPU_INT_CH48_EN_POS)
#define SPU_INT_CH48_DISABLE                     (0x0UL << SPU_INT_CH48_EN_POS)
#define SPU_INT_CH49_EN_POS                      (1)
#define SPU_INT_CH49_EN_MSK                      (0x1UL << SPU_INT_CH49_EN_POS)
#define SPU_INT_CH49_ENABLE                      (0x1UL << SPU_INT_CH49_EN_POS)
#define SPU_INT_CH49_DISABLE                     (0x0UL << SPU_INT_CH49_EN_POS)
#define SPU_INT_CH50_EN_POS                      (2)
#define SPU_INT_CH50_EN_MSK                      (0x1UL << SPU_INT_CH50_EN_POS)
#define SPU_INT_CH50_ENABLE                      (0x1UL << SPU_INT_CH50_EN_POS)
#define SPU_INT_CH50_DISABLE                     (0x0UL << SPU_INT_CH50_EN_POS)
#define SPU_INT_CH51_EN_POS                      (3)
#define SPU_INT_CH51_EN_MSK                      (0x1UL << SPU_INT_CH51_EN_POS)
#define SPU_INT_CH51_ENABLE                      (0x1UL << SPU_INT_CH51_EN_POS)
#define SPU_INT_CH51_DISABLE                     (0x0UL << SPU_INT_CH51_EN_POS)
#define SPU_INT_CH52_EN_POS                      (4)
#define SPU_INT_CH52_EN_MSK                      (0x1UL << SPU_INT_CH52_EN_POS)
#define SPU_INT_CH52_ENABLE                      (0x1UL << SPU_INT_CH52_EN_POS)
#define SPU_INT_CH52_DISABLE                     (0x0UL << SPU_INT_CH52_EN_POS)
#define SPU_INT_CH53_EN_POS                      (5)
#define SPU_INT_CH53_EN_MSK                      (0x1UL << SPU_INT_CH53_EN_POS)
#define SPU_INT_CH53_ENABLE                      (0x1UL << SPU_INT_CH53_EN_POS)
#define SPU_INT_CH53_DISABLE                     (0x0UL << SPU_INT_CH53_EN_POS)
#define SPU_INT_CH54_EN_POS                      (6)
#define SPU_INT_CH54_EN_MSK                      (0x1UL << SPU_INT_CH54_EN_POS)
#define SPU_INT_CH54_ENABLE                      (0x1UL << SPU_INT_CH54_EN_POS)
#define SPU_INT_CH54_DISABLE                     (0x0UL << SPU_INT_CH54_EN_POS)
#define SPU_INT_CH55_EN_POS                      (7)
#define SPU_INT_CH55_EN_MSK                      (0x1UL << SPU_INT_CH55_EN_POS)
#define SPU_INT_CH55_ENABLE                      (0x1UL << SPU_INT_CH55_EN_POS)
#define SPU_INT_CH55_DISABLE                     (0x0UL << SPU_INT_CH55_EN_POS)
#define SPU_INT_CH56_EN_POS                      (8)
#define SPU_INT_CH56_EN_MSK                      (0x1UL << SPU_INT_CH56_EN_POS)
#define SPU_INT_CH56_ENABLE                      (0x1UL << SPU_INT_CH56_EN_POS)
#define SPU_INT_CH56_DISABLE                     (0x0UL << SPU_INT_CH56_EN_POS)
#define SPU_INT_CH57_EN_POS                      (9)
#define SPU_INT_CH57_EN_MSK                      (0x1UL << SPU_INT_CH57_EN_POS)
#define SPU_INT_CH57_ENABLE                      (0x1UL << SPU_INT_CH57_EN_POS)
#define SPU_INT_CH57_DISABLE                     (0x0UL << SPU_INT_CH57_EN_POS)
#define SPU_INT_CH58_EN_POS                      (10)
#define SPU_INT_CH58_EN_MSK                      (0x1UL << SPU_INT_CH58_EN_POS)
#define SPU_INT_CH58_ENABLE                      (0x1UL << SPU_INT_CH58_EN_POS)
#define SPU_INT_CH58_DISABLE                     (0x0UL << SPU_INT_CH58_EN_POS)
#define SPU_INT_CH59_EN_POS                      (11)
#define SPU_INT_CH59_EN_MSK                      (0x1UL << SPU_INT_CH59_EN_POS)
#define SPU_INT_CH59_ENABLE                      (0x1UL << SPU_INT_CH59_EN_POS)
#define SPU_INT_CH59_DISABLE                     (0x0UL << SPU_INT_CH59_EN_POS)
#define SPU_INT_CH60_EN_POS                      (12)
#define SPU_INT_CH60_EN_MSK                      (0x1UL << SPU_INT_CH60_EN_POS)
#define SPU_INT_CH60_ENABLE                      (0x1UL << SPU_INT_CH60_EN_POS)
#define SPU_INT_CH60_DISABLE                     (0x0UL << SPU_INT_CH60_EN_POS)
#define SPU_INT_CH61_EN_POS                      (13)
#define SPU_INT_CH61_EN_MSK                      (0x1UL << SPU_INT_CH61_EN_POS)
#define SPU_INT_CH61_ENABLE                      (0x1UL << SPU_INT_CH61_EN_POS)
#define SPU_INT_CH61_DISABLE                     (0x0UL << SPU_INT_CH61_EN_POS)
#define SPU_INT_CH62_EN_POS                      (14)
#define SPU_INT_CH62_EN_MSK                      (0x1UL << SPU_INT_CH62_EN_POS)
#define SPU_INT_CH62_ENABLE                      (0x1UL << SPU_INT_CH62_EN_POS)
#define SPU_INT_CH62_DISABLE                     (0x0UL << SPU_INT_CH62_EN_POS)
#define SPU_INT_CH63_EN_POS                      (15)
#define SPU_INT_CH63_EN_MSK                      (0x1UL << SPU_INT_CH63_EN_POS)
#define SPU_INT_CH63_ENABLE                      (0x1UL << SPU_INT_CH63_EN_POS)
#define SPU_INT_CH63_DISABLE                     (0x0UL << SPU_INT_CH63_EN_POS)

/*
 * Bit definition for INT_STATUS_CH48_63[15:0] - SPU Channel INT Status
 */
#define SPU_INT_FLAG_CH48_POS                    (0)
#define SPU_INT_FLAG_CH48_MSK                    (0x1UL << SPU_INT_FLAG_CH48_POS)
#define SPU_INT_FLAG_CH48                        (0x1UL << SPU_INT_FLAG_CH48_POS)
#define SPU_INT_FLAG_CH49_POS                    (1)
#define SPU_INT_FLAG_CH49_MSK                    (0x1UL << SPU_INT_FLAG_CH49_POS)
#define SPU_INT_FLAG_CH49                        (0x1UL << SPU_INT_FLAG_CH49_POS)
#define SPU_INT_FLAG_CH50_POS                    (2)
#define SPU_INT_FLAG_CH50_MSK                    (0x1UL << SPU_INT_FLAG_CH50_POS)
#define SPU_INT_FLAG_CH50                        (0x1UL << SPU_INT_FLAG_CH50_POS)
#define SPU_INT_FLAG_CH51_POS                    (3)
#define SPU_INT_FLAG_CH51_MSK                    (0x1UL << SPU_INT_FLAG_CH51_POS)
#define SPU_INT_FLAG_CH51                        (0x0UL << SPU_INT_FLAG_CH51_POS)
#define SPU_INT_FLAG_CH52_POS                    (4)
#define SPU_INT_FLAG_CH52_MSK                    (0x1UL << SPU_INT_FLAG_CH52_POS)
#define SPU_INT_FLAG_CH52                        (0x0UL << SPU_INT_FLAG_CH52_POS)
#define SPU_INT_FLAG_CH53_POS                    (5)
#define SPU_INT_FLAG_CH53_MSK                    (0x1UL << SPU_INT_FLAG_CH53_POS)
#define SPU_INT_FLAG_CH53                        (0x0UL << SPU_INT_FLAG_CH53_POS)
#define SPU_INT_FLAG_CH54_POS                    (6)
#define SPU_INT_FLAG_CH54_MSK                    (0x1UL << SPU_INT_FLAG_CH54_POS)
#define SPU_INT_FLAG_CH54                        (0x0UL << SPU_INT_FLAG_CH54_POS)
#define SPU_INT_FLAG_CH55_POS                    (7)
#define SPU_INT_FLAG_CH55_MSK                    (0x1UL << SPU_INT_FLAG_CH55_POS)
#define SPU_INT_FLAG_CH55                        (0x0UL << SPU_INT_FLAG_CH55_POS)
#define SPU_INT_FLAG_CH56_POS                    (8)
#define SPU_INT_FLAG_CH56_MSK                    (0x1UL << SPU_INT_FLAG_CH56_POS)
#define SPU_INT_FLAG_CH56                        (0x0UL << SPU_INT_FLAG_CH56_POS)
#define SPU_INT_FLAG_CH57_POS                    (9)
#define SPU_INT_FLAG_CH57_MSK                    (0x1UL << SPU_INT_FLAG_CH57_POS)
#define SPU_INT_FLAG_CH57                        (0x0UL << SPU_INT_FLAG_CH57_POS)
#define SPU_INT_FLAG_CH58_POS                    (10)
#define SPU_INT_FLAG_CH58_MSK                    (0x1UL << SPU_INT_FLAG_CH58_POS)
#define SPU_INT_FLAG_CH58                        (0x0UL << SPU_INT_FLAG_CH58_POS)
#define SPU_INT_FLAG_CH59_POS                    (11)
#define SPU_INT_FLAG_CH59_MSK                    (0x1UL << SPU_INT_FLAG_CH59_POS)
#define SPU_INT_FLAG_CH59                        (0x0UL << SPU_INT_FLAG_CH59_POS)
#define SPU_INT_FLAG_CH60_POS                    (12)
#define SPU_INT_FLAG_CH60_MSK                    (0x1UL << SPU_INT_FLAG_CH60_POS)
#define SPU_INT_FLAG_CH60                        (0x0UL << SPU_INT_FLAG_CH60_POS)
#define SPU_INT_FLAG_CH61_POS                    (13)
#define SPU_INT_FLAG_CH61_MSK                    (0x1UL << SPU_INT_FLAG_CH61_POS)
#define SPU_INT_FLAG_CH61                        (0x0UL << SPU_INT_FLAG_CH61_POS)
#define SPU_INT_FLAG_CH62_POS                    (14)
#define SPU_INT_FLAG_CH62_MSK                    (0x1UL << SPU_INT_FLAG_CH62_POS)
#define SPU_INT_FLAG_CH62                        (0x0UL << SPU_INT_FLAG_CH62_POS)
#define SPU_INT_FLAG_CH63_POS                    (15)
#define SPU_INT_FLAG_CH63_MSK                    (0x1UL << SPU_INT_FLAG_CH63_POS)
#define SPU_INT_FLAG_CH63                        (0x0UL << SPU_INT_FLAG_CH63_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH48_3[3:0] -
 */
#define SPU_ENV_CLK_CH48_POS                     (0)
#define SPU_ENV_CLK_CH48_MSK                     (0xFUL << SPU_ENV_CLK_CH48_POS)
#define SPU_ENV_CLK_CH48_0x0                     (0x0UL << SPU_ENV_CLK_CH48_POS)
#define SPU_ENV_CLK_CH48_0xF                     (0xFUL << SPU_ENV_CLK_CH48_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH48_3[7:4] -
 */
#define SPU_ENV_CLK_CH49_POS                     (4)
#define SPU_ENV_CLK_CH49_MSK                     (0xFUL << SPU_ENV_CLK_CH49_POS)
#define SPU_ENV_CLK_CH49_0x0                     (0x0UL << SPU_ENV_CLK_CH49_POS)
#define SPU_ENV_CLK_CH49_0xF                     (0xFUL << SPU_ENV_CLK_CH49_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH48_3[11:8] -
 */
#define SPU_ENV_CLK_CH50_POS                     (8)
#define SPU_ENV_CLK_CH50_MSK                     (0xFUL << SPU_ENV_CLK_CH50_POS)
#define SPU_ENV_CLK_CH50_0x0                     (0x0UL << SPU_ENV_CLK_CH50_POS)
#define SPU_ENV_CLK_CH50_0xF                     (0xFUL << SPU_ENV_CLK_CH50_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH48_3[15:12] -
 */
#define SPU_ENV_CLK_CH51_POS                     (12)
#define SPU_ENV_CLK_CH51_MSK                     (0xFUL << SPU_ENV_CLK_CH51_POS)
#define SPU_ENV_CLK_CH51_0x0                     (0x0UL << SPU_ENV_CLK_CH51_POS)
#define SPU_ENV_CLK_CH51_0xF                     (0xFUL << SPU_ENV_CLK_CH51_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH52_7[3:0] -
 */
#define SPU_ENV_CLK_CH52_POS                     (0)
#define SPU_ENV_CLK_CH52_MSK                     (0xFUL << SPU_ENV_CLK_CH52_POS)
#define SPU_ENV_CLK_CH52_0x0                     (0x0UL << SPU_ENV_CLK_CH52_POS)
#define SPU_ENV_CLK_CH52_0xF                     (0xFUL << SPU_ENV_CLK_CH52_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH52_7[7:4] -
 */
#define SPU_ENV_CLK_CH53_POS                     (4)
#define SPU_ENV_CLK_CH53_MSK                     (0xFUL << SPU_ENV_CLK_CH53_POS)
#define SPU_ENV_CLK_CH53_0x0                     (0x0UL << SPU_ENV_CLK_CH53_POS)
#define SPU_ENV_CLK_CH53_0xF                     (0xFUL << SPU_ENV_CLK_CH53_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH52_7[11:8] -
 */
#define SPU_ENV_CLK_CH54_POS                     (8)
#define SPU_ENV_CLK_CH54_MSK                     (0xFUL << SPU_ENV_CLK_CH54_POS)
#define SPU_ENV_CLK_CH54_0x0                     (0x0UL << SPU_ENV_CLK_CH54_POS)
#define SPU_ENV_CLK_CH54_0xF                     (0xFUL << SPU_ENV_CLK_CH54_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH52_7[15:12] -
 */
#define SPU_ENV_CLK_CH55_POS                     (12)
#define SPU_ENV_CLK_CH55_MSK                     (0xFUL << SPU_ENV_CLK_CH55_POS)
#define SPU_ENV_CLK_CH55_0x0                     (0x0UL << SPU_ENV_CLK_CH55_POS)
#define SPU_ENV_CLK_CH55_0xF                     (0xFUL << SPU_ENV_CLK_CH55_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH56_11[3:0] -
 */
#define SPU_ENV_CLK_CH56_POS                     (0)
#define SPU_ENV_CLK_CH56_MSK                     (0xFUL << SPU_ENV_CLK_CH56_POS)
#define SPU_ENV_CLK_CH56_0x0                     (0x0UL << SPU_ENV_CLK_CH56_POS)
#define SPU_ENV_CLK_CH56_0xF                     (0xFUL << SPU_ENV_CLK_CH56_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH56_11[7:4] -
 */
#define SPU_ENV_CLK_CH57_POS                     (4)
#define SPU_ENV_CLK_CH57_MSK                     (0xFUL << SPU_ENV_CLK_CH57_POS)
#define SPU_ENV_CLK_CH57_0x0                     (0x0UL << SPU_ENV_CLK_CH57_POS)
#define SPU_ENV_CLK_CH57_0xF                     (0xFUL << SPU_ENV_CLK_CH57_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH56_11[11:8] -
 */
#define SPU_ENV_CLK_CH58_POS                     (8)
#define SPU_ENV_CLK_CH58_MSK                     (0xFUL << SPU_ENV_CLK_CH58_POS)
#define SPU_ENV_CLK_CH58_0x0                     (0x0UL << SPU_ENV_CLK_CH58_POS)
#define SPU_ENV_CLK_CH58_0xF                     (0xFUL << SPU_ENV_CLK_CH58_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH56_11[15:12] -
 */
#define SPU_ENV_CLK_CH59_POS                     (12)
#define SPU_ENV_CLK_CH59_MSK                     (0xFUL << SPU_ENV_CLK_CH59_POS)
#define SPU_ENV_CLK_CH59_0x0                     (0x0UL << SPU_ENV_CLK_CH59_POS)
#define SPU_ENV_CLK_CH59_0xF                     (0xFUL << SPU_ENV_CLK_CH59_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH60_15[3:0] -
 */
#define SPU_ENV_CLK_CH60_POS                     (0)
#define SPU_ENV_CLK_CH60_MSK                     (0xFUL << SPU_ENV_CLK_CH60_POS)
#define SPU_ENV_CLK_CH60_0x0                     (0x0UL << SPU_ENV_CLK_CH60_POS)
#define SPU_ENV_CLK_CH60_0xF                     (0xFUL << SPU_ENV_CLK_CH60_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH60_15[7:4] -
 */
#define SPU_ENV_CLK_CH61_POS                     (4)
#define SPU_ENV_CLK_CH61_MSK                     (0xFUL << SPU_ENV_CLK_CH61_POS)
#define SPU_ENV_CLK_CH61_0x0                     (0x0UL << SPU_ENV_CLK_CH61_POS)
#define SPU_ENV_CLK_CH61_0xF                     (0xFUL << SPU_ENV_CLK_CH61_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH60_15[11:8] -
 */
#define SPU_ENV_CLK_CH62_POS                     (8)
#define SPU_ENV_CLK_CH62_MSK                     (0xFUL << SPU_ENV_CLK_CH62_POS)
#define SPU_ENV_CLK_CH62_0x0                     (0x0UL << SPU_ENV_CLK_CH62_POS)
#define SPU_ENV_CLK_CH62_0xF                     (0xFUL << SPU_ENV_CLK_CH62_POS)

/*
 * Bit definition for SPU->ENV_CLK_CH60_15[15:12] -
 */
#define SPU_ENV_CLK_CH63_POS                     (12)
#define SPU_ENV_CLK_CH63_MSK                     (0xFUL << SPU_ENV_CLK_CH63_POS)
#define SPU_ENV_CLK_CH63_0x0                     (0x0UL << SPU_ENV_CLK_CH63_POS)
#define SPU_ENV_CLK_CH63_0xF                     (0xFUL << SPU_ENV_CLK_CH63_POS)

/*
 * Bit definition for SPU->ENV_RAMPDOWN_CH48_63[15:0]
 */
#define SPU_ENV_RAMPDOWN_CH48_POS                (0)
#define SPU_ENV_RAMPDOWN_CH48_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH48_POS)
#define SPU_ENV_RAMPDOWN_CH48_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH48_POS)
#define SPU_ENV_RAMPDOWN_CH48_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH48_POS)
#define SPU_ENV_RAMPDOWN_CH49_POS                (1)
#define SPU_ENV_RAMPDOWN_CH49_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH49_POS)
#define SPU_ENV_RAMPDOWN_CH49_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH49_POS)
#define SPU_ENV_RAMPDOWN_CH49_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH49_POS)
#define SPU_ENV_RAMPDOWN_CH50_POS                (2)
#define SPU_ENV_RAMPDOWN_CH50_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH50_POS)
#define SPU_ENV_RAMPDOWN_CH50_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH50_POS)
#define SPU_ENV_RAMPDOWN_CH50_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH50_POS)
#define SPU_ENV_RAMPDOWN_CH51_POS                (3)
#define SPU_ENV_RAMPDOWN_CH51_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH51_POS)
#define SPU_ENV_RAMPDOWN_CH51_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH51_POS)
#define SPU_ENV_RAMPDOWN_CH51_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH51_POS)
#define SPU_ENV_RAMPDOWN_CH52_POS                (4)
#define SPU_ENV_RAMPDOWN_CH52_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH52_POS)
#define SPU_ENV_RAMPDOWN_CH52_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH52_POS)
#define SPU_ENV_RAMPDOWN_CH52_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH52_POS)
#define SPU_ENV_RAMPDOWN_CH53_POS                (5)
#define SPU_ENV_RAMPDOWN_CH53_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH53_POS)
#define SPU_ENV_RAMPDOWN_CH53_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH53_POS)
#define SPU_ENV_RAMPDOWN_CH53_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH53_POS)
#define SPU_ENV_RAMPDOWN_CH54_POS                (6)
#define SPU_ENV_RAMPDOWN_CH54_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH54_POS)
#define SPU_ENV_RAMPDOWN_CH54_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH54_POS)
#define SPU_ENV_RAMPDOWN_CH54_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH54_POS)
#define SPU_ENV_RAMPDOWN_CH55_POS                (7)
#define SPU_ENV_RAMPDOWN_CH55_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH55_POS)
#define SPU_ENV_RAMPDOWN_CH55_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH55_POS)
#define SPU_ENV_RAMPDOWN_CH55_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH55_POS)
#define SPU_ENV_RAMPDOWN_CH56_POS                (8)
#define SPU_ENV_RAMPDOWN_CH56_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH56_POS)
#define SPU_ENV_RAMPDOWN_CH56_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH56_POS)
#define SPU_ENV_RAMPDOWN_CH56_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH56_POS)
#define SPU_ENV_RAMPDOWN_CH57_POS                (9)
#define SPU_ENV_RAMPDOWN_CH57_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH57_POS)
#define SPU_ENV_RAMPDOWN_CH57_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH57_POS)
#define SPU_ENV_RAMPDOWN_CH57_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH57_POS)
#define SPU_ENV_RAMPDOWN_CH58_POS                (10)
#define SPU_ENV_RAMPDOWN_CH58_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH58_POS)
#define SPU_ENV_RAMPDOWN_CH58_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH58_POS)
#define SPU_ENV_RAMPDOWN_CH58_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH58_POS)
#define SPU_ENV_RAMPDOWN_CH59_POS                (11)
#define SPU_ENV_RAMPDOWN_CH59_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH59_POS)
#define SPU_ENV_RAMPDOWN_CH59_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH59_POS)
#define SPU_ENV_RAMPDOWN_CH59_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH59_POS)
#define SPU_ENV_RAMPDOWN_CH60_POS                (12)
#define SPU_ENV_RAMPDOWN_CH60_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH60_POS)
#define SPU_ENV_RAMPDOWN_CH60_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH60_POS)
#define SPU_ENV_RAMPDOWN_CH60_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH60_POS)
#define SPU_ENV_RAMPDOWN_CH61_POS                (13)
#define SPU_ENV_RAMPDOWN_CH61_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH61_POS)
#define SPU_ENV_RAMPDOWN_CH61_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH61_POS)
#define SPU_ENV_RAMPDOWN_CH61_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH61_POS)
#define SPU_ENV_RAMPDOWN_CH62_POS                (14)
#define SPU_ENV_RAMPDOWN_CH62_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH62_POS)
#define SPU_ENV_RAMPDOWN_CH62_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH62_POS)
#define SPU_ENV_RAMPDOWN_CH62_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH62_POS)
#define SPU_ENV_RAMPDOWN_CH63_POS                (15)
#define SPU_ENV_RAMPDOWN_CH63_MSK                (0x1UL << SPU_ENV_RAMPDOWN_CH63_POS)
#define SPU_ENV_RAMPDOWN_CH63_ENABLE             (0x1UL << SPU_ENV_RAMPDOWN_CH63_POS)
#define SPU_ENV_RAMPDOWN_CH63_DISABLE            (0x0UL << SPU_ENV_RAMPDOWN_CH63_POS)

/*
 * Bit definition for SPU->STOP_STATUS_CH48_63[15:0]
 */
#define SPU_STOP_FLAG_CH48_POS                   (0)
#define SPU_STOP_FLAG_CH48_MSK                   (0x1UL << SPU_STOP_FLAG_CH48_POS)
#define SPU_STOP_FLAG_CH48                       (0x1UL << SPU_STOP_FLAG_CH48_POS)
#define SPU_STOP_FLAG_CH49_POS                   (1)
#define SPU_STOP_FLAG_CH49_MSK                   (0x1UL << SPU_STOP_FLAG_CH49_POS)
#define SPU_STOP_FLAG_CH49                       (0x1UL << SPU_STOP_FLAG_CH49_POS)
#define SPU_STOP_FLAG_CH50_POS                   (2)
#define SPU_STOP_FLAG_CH50_MSK                   (0x1UL << SPU_STOP_FLAG_CH50_POS)
#define SPU_STOP_FLAG_CH50                       (0x1UL << SPU_STOP_FLAG_CH50_POS)
#define SPU_STOP_FLAG_CH51_POS                   (3)
#define SPU_STOP_FLAG_CH51_MSK                   (0x1UL << SPU_STOP_FLAG_CH51_POS)
#define SPU_STOP_FLAG_CH51                       (0x0UL << SPU_STOP_FLAG_CH51_POS)
#define SPU_STOP_FLAG_CH52_POS                   (4)
#define SPU_STOP_FLAG_CH52_MSK                   (0x1UL << SPU_STOP_FLAG_CH52_POS)
#define SPU_STOP_FLAG_CH52                       (0x0UL << SPU_STOP_FLAG_CH52_POS)
#define SPU_STOP_FLAG_CH53_POS                   (5)
#define SPU_STOP_FLAG_CH53_MSK                   (0x1UL << SPU_STOP_FLAG_CH53_POS)
#define SPU_STOP_FLAG_CH53                       (0x0UL << SPU_STOP_FLAG_CH53_POS)
#define SPU_STOP_FLAG_CH54_POS                   (6)
#define SPU_STOP_FLAG_CH54_MSK                   (0x1UL << SPU_STOP_FLAG_CH54_POS)
#define SPU_STOP_FLAG_CH54                       (0x0UL << SPU_STOP_FLAG_CH54_POS)
#define SPU_STOP_FLAG_CH55_POS                   (7)
#define SPU_STOP_FLAG_CH55_MSK                   (0x1UL << SPU_STOP_FLAG_CH55_POS)
#define SPU_STOP_FLAG_CH55                       (0x0UL << SPU_STOP_FLAG_CH55_POS)
#define SPU_STOP_FLAG_CH56_POS                   (8)
#define SPU_STOP_FLAG_CH56_MSK                   (0x1UL << SPU_STOP_FLAG_CH56_POS)
#define SPU_STOP_FLAG_CH56                       (0x0UL << SPU_STOP_FLAG_CH56_POS)
#define SPU_STOP_FLAG_CH57_POS                   (9)
#define SPU_STOP_FLAG_CH57_MSK                   (0x1UL << SPU_STOP_FLAG_CH57_POS)
#define SPU_STOP_FLAG_CH57                       (0x0UL << SPU_STOP_FLAG_CH57_POS)
#define SPU_STOP_FLAG_CH58_POS                   (10)
#define SPU_STOP_FLAG_CH58_MSK                   (0x1UL << SPU_STOP_FLAG_CH58_POS)
#define SPU_STOP_FLAG_CH58                       (0x0UL << SPU_STOP_FLAG_CH58_POS)
#define SPU_STOP_FLAG_CH59_POS                   (11)
#define SPU_STOP_FLAG_CH59_MSK                   (0x1UL << SPU_STOP_FLAG_CH59_POS)
#define SPU_STOP_FLAG_CH59                       (0x0UL << SPU_STOP_FLAG_CH59_POS)
#define SPU_STOP_FLAG_CH60_POS                   (12)
#define SPU_STOP_FLAG_CH60_MSK                   (0x1UL << SPU_STOP_FLAG_CH60_POS)
#define SPU_STOP_FLAG_CH60                       (0x0UL << SPU_STOP_FLAG_CH60_POS)
#define SPU_STOP_FLAG_CH61_POS                   (13)
#define SPU_STOP_FLAG_CH61_MSK                   (0x1UL << SPU_STOP_FLAG_CH61_POS)
#define SPU_STOP_FLAG_CH61                       (0x0UL << SPU_STOP_FLAG_CH61_POS)
#define SPU_STOP_FLAG_CH62_POS                   (14)
#define SPU_STOP_FLAG_CH62_MSK                   (0x1UL << SPU_STOP_FLAG_CH62_POS)
#define SPU_STOP_FLAG_CH62                       (0x0UL << SPU_STOP_FLAG_CH62_POS)
#define SPU_STOP_FLAG_CH63_POS                   (15)
#define SPU_STOP_FLAG_CH63_MSK                   (0x1UL << SPU_STOP_FLAG_CH63_POS)
#define SPU_STOP_FLAG_CH63                       (0x0UL << SPU_STOP_FLAG_CH63_POS)

/*
 * Bit definition for SPU->CH_STATUS_CH48_63[15:0]
 */
#define SPU_BUSY_FLAG_CH48_POS                   (0)
#define SPU_BUSY_FLAG_CH48_MSK                   (0x1UL << SPU_BUSY_FLAG_CH48_POS)
#define SPU_BUSY_FLAG_CH48                       (0x1UL << SPU_BUSY_FLAG_CH48_POS)
#define SPU_BUSY_FLAG_CH49_POS                   (1)
#define SPU_BUSY_FLAG_CH49_MSK                   (0x1UL << SPU_BUSY_FLAG_CH49_POS)
#define SPU_BUSY_FLAG_CH49                       (0x1UL << SPU_BUSY_FLAG_CH49_POS)
#define SPU_BUSY_FLAG_CH50_POS                   (2)
#define SPU_BUSY_FLAG_CH50_MSK                   (0x1UL << SPU_BUSY_FLAG_CH50_POS)
#define SPU_BUSY_FLAG_CH50                       (0x1UL << SPU_BUSY_FLAG_CH50_POS)
#define SPU_BUSY_FLAG_CH51_POS                   (3)
#define SPU_BUSY_FLAG_CH51_MSK                   (0x1UL << SPU_BUSY_FLAG_CH51_POS)
#define SPU_BUSY_FLAG_CH51                       (0x0UL << SPU_BUSY_FLAG_CH51_POS)
#define SPU_BUSY_FLAG_CH52_POS                   (4)
#define SPU_BUSY_FLAG_CH52_MSK                   (0x1UL << SPU_BUSY_FLAG_CH52_POS)
#define SPU_BUSY_FLAG_CH52                       (0x0UL << SPU_BUSY_FLAG_CH52_POS)
#define SPU_BUSY_FLAG_CH53_POS                   (5)
#define SPU_BUSY_FLAG_CH53_MSK                   (0x1UL << SPU_BUSY_FLAG_CH53_POS)
#define SPU_BUSY_FLAG_CH53                       (0x0UL << SPU_BUSY_FLAG_CH53_POS)
#define SPU_BUSY_FLAG_CH54_POS                   (6)
#define SPU_BUSY_FLAG_CH54_MSK                   (0x1UL << SPU_BUSY_FLAG_CH54_POS)
#define SPU_BUSY_FLAG_CH54                       (0x0UL << SPU_BUSY_FLAG_CH54_POS)
#define SPU_BUSY_FLAG_CH55_POS                   (7)
#define SPU_BUSY_FLAG_CH55_MSK                   (0x1UL << SPU_BUSY_FLAG_CH55_POS)
#define SPU_BUSY_FLAG_CH55                       (0x0UL << SPU_BUSY_FLAG_CH55_POS)
#define SPU_BUSY_FLAG_CH56_POS                   (8)
#define SPU_BUSY_FLAG_CH56_MSK                   (0x1UL << SPU_BUSY_FLAG_CH56_POS)
#define SPU_BUSY_FLAG_CH56                       (0x0UL << SPU_BUSY_FLAG_CH56_POS)
#define SPU_BUSY_FLAG_CH57_POS                   (9)
#define SPU_BUSY_FLAG_CH57_MSK                   (0x1UL << SPU_BUSY_FLAG_CH57_POS)
#define SPU_BUSY_FLAG_CH57                       (0x0UL << SPU_BUSY_FLAG_CH57_POS)
#define SPU_BUSY_FLAG_CH58_POS                   (10)
#define SPU_BUSY_FLAG_CH58_MSK                   (0x1UL << SPU_BUSY_FLAG_CH58_POS)
#define SPU_BUSY_FLAG_CH58                       (0x0UL << SPU_BUSY_FLAG_CH58_POS)
#define SPU_BUSY_FLAG_CH59_POS                   (11)
#define SPU_BUSY_FLAG_CH59_MSK                   (0x1UL << SPU_BUSY_FLAG_CH59_POS)
#define SPU_BUSY_FLAG_CH59                       (0x0UL << SPU_BUSY_FLAG_CH59_POS)
#define SPU_BUSY_FLAG_CH60_POS                   (12)
#define SPU_BUSY_FLAG_CH60_MSK                   (0x1UL << SPU_BUSY_FLAG_CH60_POS)
#define SPU_BUSY_FLAG_CH60                       (0x0UL << SPU_BUSY_FLAG_CH60_POS)
#define SPU_BUSY_FLAG_CH61_POS                   (13)
#define SPU_BUSY_FLAG_CH61_MSK                   (0x1UL << SPU_BUSY_FLAG_CH61_POS)
#define SPU_BUSY_FLAG_CH61                       (0x0UL << SPU_BUSY_FLAG_CH61_POS)
#define SPU_BUSY_FLAG_CH62_POS                   (14)
#define SPU_BUSY_FLAG_CH62_MSK                   (0x1UL << SPU_BUSY_FLAG_CH62_POS)
#define SPU_BUSY_FLAG_CH62                       (0x0UL << SPU_BUSY_FLAG_CH62_POS)
#define SPU_BUSY_FLAG_CH63_POS                   (15)
#define SPU_BUSY_FLAG_CH63_MSK                   (0x1UL << SPU_BUSY_FLAG_CH63_POS)
#define SPU_BUSY_FLAG_CH63                       (0x0UL << SPU_BUSY_FLAG_CH63_POS)

/*
 * Bit definition for SPU->ENV_REPEAT_CH48_63[15:0]
 */
#define SPU_ENV_REPEAT_CH48_POS                  (0)
#define SPU_ENV_REPEAT_CH48_MSK                  (0x1UL << SPU_ENV_REPEAT_CH48_POS)
#define SPU_ENV_REPEAT_CH48_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH48_POS)
#define SPU_ENV_REPEAT_CH48_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH48_POS)
#define SPU_ENV_REPEAT_CH49_POS                  (1)
#define SPU_ENV_REPEAT_CH49_MSK                  (0x1UL << SPU_ENV_REPEAT_CH49_POS)
#define SPU_ENV_REPEAT_CH49_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH49_POS)
#define SPU_ENV_REPEAT_CH49_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH49_POS)
#define SPU_ENV_REPEAT_CH50_POS                  (2)
#define SPU_ENV_REPEAT_CH50_MSK                  (0x1UL << SPU_ENV_REPEAT_CH50_POS)
#define SPU_ENV_REPEAT_CH50_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH50_POS)
#define SPU_ENV_REPEAT_CH50_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH50_POS)
#define SPU_ENV_REPEAT_CH51_POS                  (3)
#define SPU_ENV_REPEAT_CH51_MSK                  (0x1UL << SPU_ENV_REPEAT_CH51_POS)
#define SPU_ENV_REPEAT_CH51_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH51_POS)
#define SPU_ENV_REPEAT_CH51_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH51_POS)
#define SPU_ENV_REPEAT_CH52_POS                  (4)
#define SPU_ENV_REPEAT_CH52_MSK                  (0x1UL << SPU_ENV_REPEAT_CH52_POS)
#define SPU_ENV_REPEAT_CH52_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH52_POS)
#define SPU_ENV_REPEAT_CH52_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH52_POS)
#define SPU_ENV_REPEAT_CH53_POS                  (5)
#define SPU_ENV_REPEAT_CH53_MSK                  (0x1UL << SPU_ENV_REPEAT_CH53_POS)
#define SPU_ENV_REPEAT_CH53_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH53_POS)
#define SPU_ENV_REPEAT_CH53_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH53_POS)
#define SPU_ENV_REPEAT_CH54_POS                  (6)
#define SPU_ENV_REPEAT_CH54_MSK                  (0x1UL << SPU_ENV_REPEAT_CH54_POS)
#define SPU_ENV_REPEAT_CH54_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH54_POS)
#define SPU_ENV_REPEAT_CH54_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH54_POS)
#define SPU_ENV_REPEAT_CH55_POS                  (7)
#define SPU_ENV_REPEAT_CH55_MSK                  (0x1UL << SPU_ENV_REPEAT_CH55_POS)
#define SPU_ENV_REPEAT_CH55_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH55_POS)
#define SPU_ENV_REPEAT_CH55_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH55_POS)
#define SPU_ENV_REPEAT_CH56_POS                  (8)
#define SPU_ENV_REPEAT_CH56_MSK                  (0x1UL << SPU_ENV_REPEAT_CH56_POS)
#define SPU_ENV_REPEAT_CH56_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH56_POS)
#define SPU_ENV_REPEAT_CH56_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH56_POS)
#define SPU_ENV_REPEAT_CH57_POS                  (9)
#define SPU_ENV_REPEAT_CH57_MSK                  (0x1UL << SPU_ENV_REPEAT_CH57_POS)
#define SPU_ENV_REPEAT_CH57_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH57_POS)
#define SPU_ENV_REPEAT_CH57_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH57_POS)
#define SPU_ENV_REPEAT_CH58_POS                  (10)
#define SPU_ENV_REPEAT_CH58_MSK                  (0x1UL << SPU_ENV_REPEAT_CH58_POS)
#define SPU_ENV_REPEAT_CH58_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH58_POS)
#define SPU_ENV_REPEAT_CH58_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH58_POS)
#define SPU_ENV_REPEAT_CH59_POS                  (11)
#define SPU_ENV_REPEAT_CH59_MSK                  (0x1UL << SPU_ENV_REPEAT_CH59_POS)
#define SPU_ENV_REPEAT_CH59_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH59_POS)
#define SPU_ENV_REPEAT_CH59_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH59_POS)
#define SPU_ENV_REPEAT_CH60_POS                  (12)
#define SPU_ENV_REPEAT_CH60_MSK                  (0x1UL << SPU_ENV_REPEAT_CH60_POS)
#define SPU_ENV_REPEAT_CH60_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH60_POS)
#define SPU_ENV_REPEAT_CH60_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH60_POS)
#define SPU_ENV_REPEAT_CH61_POS                  (13)
#define SPU_ENV_REPEAT_CH61_MSK                  (0x1UL << SPU_ENV_REPEAT_CH61_POS)
#define SPU_ENV_REPEAT_CH61_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH61_POS)
#define SPU_ENV_REPEAT_CH61_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH61_POS)
#define SPU_ENV_REPEAT_CH62_POS                  (14)
#define SPU_ENV_REPEAT_CH62_MSK                  (0x1UL << SPU_ENV_REPEAT_CH62_POS)
#define SPU_ENV_REPEAT_CH62_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH62_POS)
#define SPU_ENV_REPEAT_CH62_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH62_POS)
#define SPU_ENV_REPEAT_CH63_POS                  (15)
#define SPU_ENV_REPEAT_CH63_MSK                  (0x1UL << SPU_ENV_REPEAT_CH63_POS)
#define SPU_ENV_REPEAT_CH63_ENABLE               (0x1UL << SPU_ENV_REPEAT_CH63_POS)
#define SPU_ENV_REPEAT_CH63_DISABLE              (0x0UL << SPU_ENV_REPEAT_CH63_POS)

/*
 * Bit definition for SPU->ENV_MODE_CH48_63[15:0]
 */
#define SPU_ENV_MODE_CH48_POS                    (0)
#define SPU_ENV_MODE_CH48_MSK                    (0x1UL << SPU_ENV_MODE_CH48_POS)
#define SPU_ENV_MODE_CH48_MANUAL                 (0x1UL << SPU_ENV_MODE_CH48_POS)
#define SPU_ENV_MODE_CH48_AUTO                   (0x0UL << SPU_ENV_MODE_CH48_POS)
#define SPU_ENV_MODE_CH49_POS                    (1)
#define SPU_ENV_MODE_CH49_MSK                    (0x1UL << SPU_ENV_MODE_CH49_POS)
#define SPU_ENV_MODE_CH49_MANUAL                 (0x1UL << SPU_ENV_MODE_CH49_POS)
#define SPU_ENV_MODE_CH49_AUTO                   (0x0UL << SPU_ENV_MODE_CH49_POS)
#define SPU_ENV_MODE_CH50_POS                    (2)
#define SPU_ENV_MODE_CH50_MSK                    (0x1UL << SPU_ENV_MODE_CH50_POS)
#define SPU_ENV_MODE_CH50_MANUAL                 (0x1UL << SPU_ENV_MODE_CH50_POS)
#define SPU_ENV_MODE_CH50_AUTO                   (0x0UL << SPU_ENV_MODE_CH50_POS)
#define SPU_ENV_MODE_CH51_POS                    (3)
#define SPU_ENV_MODE_CH51_MSK                    (0x1UL << SPU_ENV_MODE_CH51_POS)
#define SPU_ENV_MODE_CH51_MANUAL                 (0x1UL << SPU_ENV_MODE_CH51_POS)
#define SPU_ENV_MODE_CH51_AUTO                   (0x0UL << SPU_ENV_MODE_CH51_POS)
#define SPU_ENV_MODE_CH52_POS                    (4)
#define SPU_ENV_MODE_CH52_MSK                    (0x1UL << SPU_ENV_MODE_CH52_POS)
#define SPU_ENV_MODE_CH52_MANUAL                 (0x1UL << SPU_ENV_MODE_CH52_POS)
#define SPU_ENV_MODE_CH52_AUTO                   (0x0UL << SPU_ENV_MODE_CH52_POS)
#define SPU_ENV_MODE_CH53_POS                    (5)
#define SPU_ENV_MODE_CH53_MSK                    (0x1UL << SPU_ENV_MODE_CH53_POS)
#define SPU_ENV_MODE_CH53_MANUAL                 (0x1UL << SPU_ENV_MODE_CH53_POS)
#define SPU_ENV_MODE_CH53_AUTO                   (0x0UL << SPU_ENV_MODE_CH53_POS)
#define SPU_ENV_MODE_CH54_POS                    (6)
#define SPU_ENV_MODE_CH54_MSK                    (0x1UL << SPU_ENV_MODE_CH54_POS)
#define SPU_ENV_MODE_CH54_MANUAL                 (0x1UL << SPU_ENV_MODE_CH54_POS)
#define SPU_ENV_MODE_CH54_AUTO                   (0x0UL << SPU_ENV_MODE_CH54_POS)
#define SPU_ENV_MODE_CH55_POS                    (7)
#define SPU_ENV_MODE_CH55_MSK                    (0x1UL << SPU_ENV_MODE_CH55_POS)
#define SPU_ENV_MODE_CH55_MANUAL                 (0x1UL << SPU_ENV_MODE_CH55_POS)
#define SPU_ENV_MODE_CH55_AUTO                   (0x0UL << SPU_ENV_MODE_CH55_POS)
#define SPU_ENV_MODE_CH56_POS                    (8)
#define SPU_ENV_MODE_CH56_MSK                    (0x1UL << SPU_ENV_MODE_CH56_POS)
#define SPU_ENV_MODE_CH56_MANUAL                 (0x1UL << SPU_ENV_MODE_CH56_POS)
#define SPU_ENV_MODE_CH56_AUTO                   (0x0UL << SPU_ENV_MODE_CH56_POS)
#define SPU_ENV_MODE_CH57_POS                    (9)
#define SPU_ENV_MODE_CH57_MSK                    (0x1UL << SPU_ENV_MODE_CH57_POS)
#define SPU_ENV_MODE_CH57_MANUAL                 (0x1UL << SPU_ENV_MODE_CH57_POS)
#define SPU_ENV_MODE_CH57_AUTO                   (0x0UL << SPU_ENV_MODE_CH57_POS)
#define SPU_ENV_MODE_CH58_POS                    (10)
#define SPU_ENV_MODE_CH58_MSK                    (0x1UL << SPU_ENV_MODE_CH58_POS)
#define SPU_ENV_MODE_CH58_MANUAL                 (0x1UL << SPU_ENV_MODE_CH58_POS)
#define SPU_ENV_MODE_CH58_AUTO                   (0x0UL << SPU_ENV_MODE_CH58_POS)
#define SPU_ENV_MODE_CH59_POS                    (11)
#define SPU_ENV_MODE_CH59_MSK                    (0x1UL << SPU_ENV_MODE_CH59_POS)
#define SPU_ENV_MODE_CH59_MANUAL                 (0x1UL << SPU_ENV_MODE_CH59_POS)
#define SPU_ENV_MODE_CH59_AUTO                   (0x0UL << SPU_ENV_MODE_CH59_POS)
#define SPU_ENV_MODE_CH60_POS                    (12)
#define SPU_ENV_MODE_CH60_MSK                    (0x1UL << SPU_ENV_MODE_CH60_POS)
#define SPU_ENV_MODE_CH60_MANUAL                 (0x1UL << SPU_ENV_MODE_CH60_POS)
#define SPU_ENV_MODE_CH60_AUTO                   (0x0UL << SPU_ENV_MODE_CH60_POS)
#define SPU_ENV_MODE_CH61_POS                    (13)
#define SPU_ENV_MODE_CH61_MSK                    (0x1UL << SPU_ENV_MODE_CH61_POS)
#define SPU_ENV_MODE_CH61_MANUAL                 (0x1UL << SPU_ENV_MODE_CH61_POS)
#define SPU_ENV_MODE_CH61_AUTO                   (0x0UL << SPU_ENV_MODE_CH61_POS)
#define SPU_ENV_MODE_CH62_POS                    (14)
#define SPU_ENV_MODE_CH62_MSK                    (0x1UL << SPU_ENV_MODE_CH62_POS)
#define SPU_ENV_MODE_CH62_MANUAL                 (0x1UL << SPU_ENV_MODE_CH62_POS)
#define SPU_ENV_MODE_CH62_AUTO                   (0x0UL << SPU_ENV_MODE_CH62_POS)
#define SPU_ENV_MODE_CH63_POS                    (15)
#define SPU_ENV_MODE_CH63_MSK                    (0x1UL << SPU_ENV_MODE_CH63_POS)
#define SPU_ENV_MODE_CH63_MANUAL                 (0x1UL << SPU_ENV_MODE_CH63_POS)
#define SPU_ENV_MODE_CH63_AUTO                   (0x0UL << SPU_ENV_MODE_CH63_POS)

/*
 * Bit definition for SPU->TONE_RELEASE_CH48_63[15:0]
 */
#define SPU_TONE_RELEASE_CH48_POS                (0)
#define SPU_TONE_RELEASE_CH48_MSK                (0x1UL << SPU_TONE_RELEASE_CH48_POS)
#define SPU_TONE_RELEASE_CH48_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH48_POS)
#define SPU_TONE_RELEASE_CH48_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH48_POS)
#define SPU_TONE_RELEASE_CH49_POS                (1)
#define SPU_TONE_RELEASE_CH49_MSK                (0x1UL << SPU_TONE_RELEASE_CH49_POS)
#define SPU_TONE_RELEASE_CH49_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH49_POS)
#define SPU_TONE_RELEASE_CH49_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH49_POS)
#define SPU_TONE_RELEASE_CH50_POS                (2)
#define SPU_TONE_RELEASE_CH50_MSK                (0x1UL << SPU_TONE_RELEASE_CH50_POS)
#define SPU_TONE_RELEASE_CH50_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH50_POS)
#define SPU_TONE_RELEASE_CH50_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH50_POS)
#define SPU_TONE_RELEASE_CH51_POS                (3)
#define SPU_TONE_RELEASE_CH51_MSK                (0x1UL << SPU_TONE_RELEASE_CH51_POS)
#define SPU_TONE_RELEASE_CH51_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH51_POS)
#define SPU_TONE_RELEASE_CH51_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH51_POS)
#define SPU_TONE_RELEASE_CH52_POS                (4)
#define SPU_TONE_RELEASE_CH52_MSK                (0x1UL << SPU_TONE_RELEASE_CH52_POS)
#define SPU_TONE_RELEASE_CH52_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH52_POS)
#define SPU_TONE_RELEASE_CH52_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH52_POS)
#define SPU_TONE_RELEASE_CH53_POS                (5)
#define SPU_TONE_RELEASE_CH53_MSK                (0x1UL << SPU_TONE_RELEASE_CH53_POS)
#define SPU_TONE_RELEASE_CH53_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH53_POS)
#define SPU_TONE_RELEASE_CH53_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH53_POS)
#define SPU_TONE_RELEASE_CH54_POS                (6)
#define SPU_TONE_RELEASE_CH54_MSK                (0x1UL << SPU_TONE_RELEASE_CH54_POS)
#define SPU_TONE_RELEASE_CH54_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH54_POS)
#define SPU_TONE_RELEASE_CH54_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH54_POS)
#define SPU_TONE_RELEASE_CH55_POS                (7)
#define SPU_TONE_RELEASE_CH55_MSK                (0x1UL << SPU_TONE_RELEASE_CH55_POS)
#define SPU_TONE_RELEASE_CH55_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH55_POS)
#define SPU_TONE_RELEASE_CH55_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH55_POS)
#define SPU_TONE_RELEASE_CH56_POS                (8)
#define SPU_TONE_RELEASE_CH56_MSK                (0x1UL << SPU_TONE_RELEASE_CH56_POS)
#define SPU_TONE_RELEASE_CH56_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH56_POS)
#define SPU_TONE_RELEASE_CH56_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH56_POS)
#define SPU_TONE_RELEASE_CH57_POS                (9)
#define SPU_TONE_RELEASE_CH57_MSK                (0x1UL << SPU_TONE_RELEASE_CH57_POS)
#define SPU_TONE_RELEASE_CH57_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH57_POS)
#define SPU_TONE_RELEASE_CH57_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH57_POS)
#define SPU_TONE_RELEASE_CH58_POS                (10)
#define SPU_TONE_RELEASE_CH58_MSK                (0x1UL << SPU_TONE_RELEASE_CH58_POS)
#define SPU_TONE_RELEASE_CH58_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH58_POS)
#define SPU_TONE_RELEASE_CH58_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH58_POS)
#define SPU_TONE_RELEASE_CH59_POS                (11)
#define SPU_TONE_RELEASE_CH59_MSK                (0x1UL << SPU_TONE_RELEASE_CH59_POS)
#define SPU_TONE_RELEASE_CH59_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH59_POS)
#define SPU_TONE_RELEASE_CH59_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH59_POS)
#define SPU_TONE_RELEASE_CH60_POS                (12)
#define SPU_TONE_RELEASE_CH60_MSK                (0x1UL << SPU_TONE_RELEASE_CH60_POS)
#define SPU_TONE_RELEASE_CH60_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH60_POS)
#define SPU_TONE_RELEASE_CH60_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH60_POS)
#define SPU_TONE_RELEASE_CH61_POS                (13)
#define SPU_TONE_RELEASE_CH61_MSK                (0x1UL << SPU_TONE_RELEASE_CH61_POS)
#define SPU_TONE_RELEASE_CH61_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH61_POS)
#define SPU_TONE_RELEASE_CH61_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH61_POS)
#define SPU_TONE_RELEASE_CH62_POS                (14)
#define SPU_TONE_RELEASE_CH62_MSK                (0x1UL << SPU_TONE_RELEASE_CH62_POS)
#define SPU_TONE_RELEASE_CH62_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH62_POS)
#define SPU_TONE_RELEASE_CH62_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH62_POS)
#define SPU_TONE_RELEASE_CH63_POS                (15)
#define SPU_TONE_RELEASE_CH63_MSK                (0x1UL << SPU_TONE_RELEASE_CH63_POS)
#define SPU_TONE_RELEASE_CH63_ENABLE             (0x1UL << SPU_TONE_RELEASE_CH63_POS)
#define SPU_TONE_RELEASE_CH63_DISABLE            (0x0UL << SPU_TONE_RELEASE_CH63_POS)

/*
 * Bit definition for SPU->ENV_INT_STATUS_CH48_63[15:0]
 */
#define SPU_ENV_INT_FLAG_CH48_POS                (0)
#define SPU_ENV_INT_FLAG_CH48_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH48_POS)
#define SPU_ENV_INT_FLAG_CH48                    (0x1UL << SPU_ENV_INT_FLAG_CH48_POS)
#define SPU_ENV_INT_FLAG_CH49_POS                (1)
#define SPU_ENV_INT_FLAG_CH49_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH49_POS)
#define SPU_ENV_INT_FLAG_CH49                    (0x1UL << SPU_ENV_INT_FLAG_CH49_POS)
#define SPU_ENV_INT_FLAG_CH50_POS                (2)
#define SPU_ENV_INT_FLAG_CH50_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH50_POS)
#define SPU_ENV_INT_FLAG_CH50                    (0x1UL << SPU_ENV_INT_FLAG_CH50_POS)
#define SPU_ENV_INT_FLAG_CH51_POS                (3)
#define SPU_ENV_INT_FLAG_CH51_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH51_POS)
#define SPU_ENV_INT_FLAG_CH51                    (0x0UL << SPU_ENV_INT_FLAG_CH51_POS)
#define SPU_ENV_INT_FLAG_CH52_POS                (4)
#define SPU_ENV_INT_FLAG_CH52_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH52_POS)
#define SPU_ENV_INT_FLAG_CH52                    (0x0UL << SPU_ENV_INT_FLAG_CH52_POS)
#define SPU_ENV_INT_FLAG_CH53_POS                (5)
#define SPU_ENV_INT_FLAG_CH53_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH53_POS)
#define SPU_ENV_INT_FLAG_CH53                    (0x0UL << SPU_ENV_INT_FLAG_CH53_POS)
#define SPU_ENV_INT_FLAG_CH54_POS                (6)
#define SPU_ENV_INT_FLAG_CH54_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH54_POS)
#define SPU_ENV_INT_FLAG_CH54                    (0x0UL << SPU_ENV_INT_FLAG_CH54_POS)
#define SPU_ENV_INT_FLAG_CH55_POS                (7)
#define SPU_ENV_INT_FLAG_CH55_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH55_POS)
#define SPU_ENV_INT_FLAG_CH55                    (0x0UL << SPU_ENV_INT_FLAG_CH55_POS)
#define SPU_ENV_INT_FLAG_CH56_POS                (8)
#define SPU_ENV_INT_FLAG_CH56_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH56_POS)
#define SPU_ENV_INT_FLAG_CH56                    (0x0UL << SPU_ENV_INT_FLAG_CH56_POS)
#define SPU_ENV_INT_FLAG_CH57_POS                (9)
#define SPU_ENV_INT_FLAG_CH57_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH57_POS)
#define SPU_ENV_INT_FLAG_CH57                    (0x0UL << SPU_ENV_INT_FLAG_CH57_POS)
#define SPU_ENV_INT_FLAG_CH58_POS                (10)
#define SPU_ENV_INT_FLAG_CH58_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH58_POS)
#define SPU_ENV_INT_FLAG_CH58                    (0x0UL << SPU_ENV_INT_FLAG_CH58_POS)
#define SPU_ENV_INT_FLAG_CH59_POS                (11)
#define SPU_ENV_INT_FLAG_CH59_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH59_POS)
#define SPU_ENV_INT_FLAG_CH59                    (0x0UL << SPU_ENV_INT_FLAG_CH59_POS)
#define SPU_ENV_INT_FLAG_CH60_POS                (12)
#define SPU_ENV_INT_FLAG_CH60_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH60_POS)
#define SPU_ENV_INT_FLAG_CH60                    (0x0UL << SPU_ENV_INT_FLAG_CH60_POS)
#define SPU_ENV_INT_FLAG_CH61_POS                (13)
#define SPU_ENV_INT_FLAG_CH61_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH61_POS)
#define SPU_ENV_INT_FLAG_CH61                    (0x0UL << SPU_ENV_INT_FLAG_CH61_POS)
#define SPU_ENV_INT_FLAG_CH62_POS                (14)
#define SPU_ENV_INT_FLAG_CH62_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH62_POS)
#define SPU_ENV_INT_FLAG_CH62                    (0x0UL << SPU_ENV_INT_FLAG_CH62_POS)
#define SPU_ENV_INT_FLAG_CH63_POS                (15)
#define SPU_ENV_INT_FLAG_CH63_MSK                (0x1UL << SPU_ENV_INT_FLAG_CH63_POS)
#define SPU_ENV_INT_FLAG_CH63                    (0x0UL << SPU_ENV_INT_FLAG_CH63_POS)

/*
 * Bit definition for SPU->PITCHBEND_EN_CH48_63[15:0]
 */
#define SPU_PITCHBEND_CH48_POS                   (0)
#define SPU_PITCHBEND_CH48_MSK                   (0x1UL << SPU_PITCHBEND_CH48_POS)
#define SPU_PITCHBEND_CH48_ENABLE                (0x1UL << SPU_PITCHBEND_CH48_POS)
#define SPU_PITCHBEND_CH48_DISABLE               (0x0UL << SPU_PITCHBEND_CH48_POS)
#define SPU_PITCHBEND_CH49_POS                   (1)
#define SPU_PITCHBEND_CH49_MSK                   (0x1UL << SPU_PITCHBEND_CH49_POS)
#define SPU_PITCHBEND_CH49_ENABLE                (0x1UL << SPU_PITCHBEND_CH49_POS)
#define SPU_PITCHBEND_CH49_DISABLE               (0x0UL << SPU_PITCHBEND_CH49_POS)
#define SPU_PITCHBEND_CH50_POS                   (2)
#define SPU_PITCHBEND_CH50_MSK                   (0x1UL << SPU_PITCHBEND_CH50_POS)
#define SPU_PITCHBEND_CH50_ENABLE                (0x1UL << SPU_PITCHBEND_CH50_POS)
#define SPU_PITCHBEND_CH50_DISABLE               (0x0UL << SPU_PITCHBEND_CH50_POS)
#define SPU_PITCHBEND_CH51_POS                   (3)
#define SPU_PITCHBEND_CH51_MSK                   (0x1UL << SPU_PITCHBEND_CH51_POS)
#define SPU_PITCHBEND_CH51_ENABLE                (0x1UL << SPU_PITCHBEND_CH51_POS)
#define SPU_PITCHBEND_CH51_DISABLE               (0x0UL << SPU_PITCHBEND_CH51_POS)
#define SPU_PITCHBEND_CH52_POS                   (4)
#define SPU_PITCHBEND_CH52_MSK                   (0x1UL << SPU_PITCHBEND_CH52_POS)
#define SPU_PITCHBEND_CH52_ENABLE                (0x1UL << SPU_PITCHBEND_CH52_POS)
#define SPU_PITCHBEND_CH52_DISABLE               (0x0UL << SPU_PITCHBEND_CH52_POS)
#define SPU_PITCHBEND_CH53_POS                   (5)
#define SPU_PITCHBEND_CH53_MSK                   (0x1UL << SPU_PITCHBEND_CH53_POS)
#define SPU_PITCHBEND_CH53_ENABLE                (0x1UL << SPU_PITCHBEND_CH53_POS)
#define SPU_PITCHBEND_CH53_DISABLE               (0x0UL << SPU_PITCHBEND_CH53_POS)
#define SPU_PITCHBEND_CH54_POS                   (6)
#define SPU_PITCHBEND_CH54_MSK                   (0x1UL << SPU_PITCHBEND_CH54_POS)
#define SPU_PITCHBEND_CH54_ENABLE                (0x1UL << SPU_PITCHBEND_CH54_POS)
#define SPU_PITCHBEND_CH54_DISABLE               (0x0UL << SPU_PITCHBEND_CH54_POS)
#define SPU_PITCHBEND_CH55_POS                   (7)
#define SPU_PITCHBEND_CH55_MSK                   (0x1UL << SPU_PITCHBEND_CH55_POS)
#define SPU_PITCHBEND_CH55_ENABLE                (0x1UL << SPU_PITCHBEND_CH55_POS)
#define SPU_PITCHBEND_CH55_DISABLE               (0x0UL << SPU_PITCHBEND_CH55_POS)
#define SPU_PITCHBEND_CH56_POS                   (8)
#define SPU_PITCHBEND_CH56_MSK                   (0x1UL << SPU_PITCHBEND_CH56_POS)
#define SPU_PITCHBEND_CH56_ENABLE                (0x1UL << SPU_PITCHBEND_CH56_POS)
#define SPU_PITCHBEND_CH56_DISABLE               (0x0UL << SPU_PITCHBEND_CH56_POS)
#define SPU_PITCHBEND_CH57_POS                   (9)
#define SPU_PITCHBEND_CH57_MSK                   (0x1UL << SPU_PITCHBEND_CH57_POS)
#define SPU_PITCHBEND_CH57_ENABLE                (0x1UL << SPU_PITCHBEND_CH57_POS)
#define SPU_PITCHBEND_CH57_DISABLE               (0x0UL << SPU_PITCHBEND_CH57_POS)
#define SPU_PITCHBEND_CH58_POS                   (10)
#define SPU_PITCHBEND_CH58_MSK                   (0x1UL << SPU_PITCHBEND_CH58_POS)
#define SPU_PITCHBEND_CH58_ENABLE                (0x1UL << SPU_PITCHBEND_CH58_POS)
#define SPU_PITCHBEND_CH58_DISABLE               (0x0UL << SPU_PITCHBEND_CH58_POS)
#define SPU_PITCHBEND_CH59_POS                   (11)
#define SPU_PITCHBEND_CH59_MSK                   (0x1UL << SPU_PITCHBEND_CH59_POS)
#define SPU_PITCHBEND_CH59_ENABLE                (0x1UL << SPU_PITCHBEND_CH59_POS)
#define SPU_PITCHBEND_CH59_DISABLE               (0x0UL << SPU_PITCHBEND_CH59_POS)
#define SPU_PITCHBEND_CH60_POS                   (12)
#define SPU_PITCHBEND_CH60_MSK                   (0x1UL << SPU_PITCHBEND_CH60_POS)
#define SPU_PITCHBEND_CH60_ENABLE                (0x1UL << SPU_PITCHBEND_CH60_POS)
#define SPU_PITCHBEND_CH60_DISABLE               (0x0UL << SPU_PITCHBEND_CH60_POS)
#define SPU_PITCHBEND_CH61_POS                   (13)
#define SPU_PITCHBEND_CH61_MSK                   (0x1UL << SPU_PITCHBEND_CH61_POS)
#define SPU_PITCHBEND_CH61_ENABLE                (0x1UL << SPU_PITCHBEND_CH61_POS)
#define SPU_PITCHBEND_CH61_DISABLE               (0x0UL << SPU_PITCHBEND_CH61_POS)
#define SPU_PITCHBEND_CH62_POS                   (14)
#define SPU_PITCHBEND_CH62_MSK                   (0x1UL << SPU_PITCHBEND_CH62_POS)
#define SPU_PITCHBEND_CH62_ENABLE                (0x1UL << SPU_PITCHBEND_CH62_POS)
#define SPU_PITCHBEND_CH62_DISABLE               (0x0UL << SPU_PITCHBEND_CH62_POS)
#define SPU_PITCHBEND_CH63_POS                   (15)
#define SPU_PITCHBEND_CH63_MSK                   (0x1UL << SPU_PITCHBEND_CH63_POS)
#define SPU_PITCHBEND_CH63_ENABLE                (0x1UL << SPU_PITCHBEND_CH63_POS)
#define SPU_PITCHBEND_CH63_DISABLE               (0x0UL << SPU_PITCHBEND_CH63_POS)

//////////////////////////////////////////////////////////////////////////////////////
// SPU _SRAM
//////////////////////////////////////////////////////////////////////////////////////
/*
 * Bit definition for SPU_CH->WAVE_ADDR[15:0]
 */
#define SPU_CH_WAVE_ADDR_POS                     (0)
#define SPU_CH_WAVE_ADDR_MSK                     (0xFFFFUL << SPU_CH_WAVE_ADDR_POS)

/*
 * Bit definition for SPU_CH->WAVE_LOOPADDR[15:0]
 */
#define SPU_CH_WAVE_LOOPADDR_POS                (0)
#define SPU_CH_WAVE_LOOPADDR_MSK                (0xFFFFUL << SPU_CH_WAVE_LOOP_ADDR_POS)

/*
 * Bit definition for SPU_CH->WAVE_MODE[15]
 */
#define SPU_CH_WAVE_ALGORITHM_POS                (15)
#define SPU_CH_WAVE_ALGORITHM_MSK                (0x1UL << SPU_CH_WAVE_ALGORITHM_POS)
#define SPU_CH_WAVE_ALGORITHM_ADPCM              (0x1UL << SPU_CH_WAVE_ALGORITHM_POS)
#define SPU_CH_WAVE_ALGORITHM_PCM                (0x0UL << SPU_CH_WAVE_ALGORITHM_POS)

/*
 * Bit definition for SPU_CH->WAVE_MODE[14]
 */
#define SPU_CH_WAVE_PCM_POS                      (14)
#define SPU_CH_WAVE_PCM_MSK                      (0x1UL << SPU_CH_WAVE_PCM_POS)
#define SPU_CH_WAVE_PCM_16BIT                    (0x1UL << SPU_CH_WAVE_PCM_POS)
#define SPU_CH_WAVE_PCM_8BIT                     (0x0UL << SPU_CH_WAVE_PCM_POS)

/*
 * Bit definition for SPU_CH->WAVE_MODE[13:12]
 */
#define SPU_CH_WAVE_PLAYMODE_POS                 (12)
#define SPU_CH_WAVE_PLAYMODE_MSK                 (0x3UL << SPU_CH_WAVE_PLAYMODE_POS)
#define SPU_CH_WAVE_PLAYMODE_SOFT                (0x0UL << SPU_CH_WAVE_PLAYMODE_POS)
#define SPU_CH_WAVE_PLAYMODE_AUTOEND             (0x1UL << SPU_CH_WAVE_PLAYMODE_POS)
#define SPU_CH_WAVE_PLAYMODE_LOOP                (0x2UL << SPU_CH_WAVE_PLAYMODE_POS)
#define SPU_CH_WAVE_PLAYMODE_ADPCM_PCM_LOOP      (0x3UL << SPU_CH_WAVE_PLAYMODE_POS)

/*
 * Bit definition for SPU_CH->WAVE_MODE[11:6]
 */
#define SPU_CH_WAVE_LOOPADDR_MID_POS            (6)
#define SPU_CH_WAVE_LOOPADDR_MID_MSK            (0x3FUL << SPU_CH_WAVE_LOOP_ADDR_MID_POS)

/*
 * Bit definition for SPU_CH->WAVE_MODE[5:0]
 */
#define SPU_CH_WAVE_ADDR_MID_POS                 (0)
#define SPU_CH_WAVE_ADDR_MID_MSK                 (0x3FUL << SPU_CH_WAVE_ADDR_MID_POS)

/*
 * Bit definition for SPU_CH->PAN_VELOCITY[14:8]
 */
#define SPU_CH_VOLUME_PAN_POS                    (8)
#define SPU_CH_VOLUME_PAN_MSK                    (0x7FUL << SPU_CH_VOLUME_PAN_POS)
#define SPU_CH_VOLUME_PAN_DEFAULE                (0x40UL << SPU_CH_VOLUME_PAN_POS)

/*
 * Bit definition for SPU_CH->PAN_VELOCITY[6:0]
 */
#define SPU_CH_VELOCITY_POS                      (0)
#define SPU_CH_VELOCITY_MSK                      (0x7FUL << SPU_CH_VELOCITY_POS)
#define SPU_CH_VELOCITY_0x7F                     (0x7FUL << SPU_CH_VELOCITY_POS)

/*
 * Bit definition for SPU_CH->ENV_CTRL0[14:8]
 */
#define SPU_CH_ENV_TARGET_POS                    (8)
#define SPU_CH_ENV_TARGET_MSK                    (0x7FUL << SPU_CH_ENV_TARGET_POS)
#define SPU_CH_ENV_TARGET_0x7F                   (0x7FUL << SPU_CH_ENV_TARGET_POS)

/*
 * Bit definition for SPU_CH->ENV_CTRL0[7]
 */
#define SPU_CH_ENV_SIGN_POS                      (7)
#define SPU_CH_ENV_SIGN_MSK                      (0x1UL << SPU_CH_ENV_SIGN_POS)
#define SPU_CH_ENV_DECREMENT                     (0x1UL << SPU_CH_ENV_SIGN_POS)
#define SPU_CH_ENV_INCREMENT                     (0x0UL << SPU_CH_ENV_SIGN_POS)

/*
 * Bit definition for SPU_CH->ENV_CTRL0[6:0]
 */
#define SPU_CH_ENV_STEP_POS                      (0)
#define SPU_CH_ENV_STEP_MSK                      (0x7FUL << SPU_CH_ENV_STEP_POS)
#define SPU_CH_ENV_STEP_0x7F                     (0x7FUL << SPU_CH_ENV_STEP_POS)

/*
 * Bit definition for SPU_CH->ENV_DATA[15:8]
 */
#define SPU_CH_ENV_COUNTER_POS                   (8)
#define SPU_CH_ENV_COUNTER_MSK                   (0xFFUL << SPU_CH_ENV_COUNTER_POS)
#define SPU_CH_ENV_COUNTER_0xFF                  (0xFFUL << SPU_CH_ENV_COUNTER_POS)

/*
 * Bit definition for SPU_CH->ENV_DATA[6:0]
 */
#define SPU_CH_ENV_DATA_POS                      (0)
#define SPU_CH_ENV_DATA_MSK                      (0x7FUL << SPU_CH_ENV_DATA_POS)

/*
 * Bit definition for SPU_CH->ENV_CTRL1[15:9]
 */
#define SPU_CH_ENV_REPEAT_CNT_POS                (9)
#define SPU_CH_ENV_REPEAT_CNT_MSK                (0x7FUL << SPU_CH_ENV_REPEAT_CNT_POS)
#define SPU_CH_ENV_REPEAT_CNT_0x7F               (0x7FUL << SPU_CH_ENV_REPEAT_CNT_POS)

/*
 * Bit definition for SPU_CH->ENV_CTRL1[8]
 */
#define SPU_CH_ENV_REPEAT_EN_POS                 (8)
#define SPU_CH_ENV_REPEAT_EN_MAK                 (0x1UL << SPU_CH_ENV_REPEAT_EN_POS)
#define SPU_CH_ENV_REPEAT_ENABLE                 (0x1UL << SPU_CH_ENV_REPEAT_EN_POS)
#define SPU_CH_ENV_REPEAT_DISABLE                (0x0UL << SPU_CH_ENV_REPEAT_EN_POS)

/*
 * Bit definition for SPU_CH->ENV_CTRL1[7:0]
 */
#define SPU_CH_ENV_LOAD_CNT_POS                  (0)
#define SPU_CH_ENV_LOAD_CNT_MSK                  (0xFFUL << SPU_CH_ENV_LOAD_CNT_POS)
#define SPU_CH_ENV_LOAD_CNT_0xFF                 (0xFFUL << SPU_CH_ENV_LOAD_CNT_POS)

/*
 * Bit definition for SPU_CH->ENV_INT_CTRL[15:7]
 */
#define SPU_CH_ENV_INT_ADDR_POS                  (7)
#define SPU_CH_ENV_INT_ADDR_MSK                  (0x1FFUL << SPU_CH_ENV_INT_ADDR_POS)

/*
 * Bit definition for SPU_CH->ENV_INT_CTRL[6]
 */
#define SPU_CH_ENV_INT_EN_POS                    (6)
#define SPU_CH_ENV_INT_EN_MSK                    (0x1UL << SPU_CH_ENV_INT_EN_POS)
#define SPU_CH_ENV_INT_ENABLE                    (0x1UL << SPU_CH_ENV_INT_EN_POS)
#define SPU_CH_ENV_INT_DISABLE                   (0x0UL << SPU_CH_ENV_INT_EN_POS)

/*
 * Bit definition for SPU_CH->ENV_INT_CTRL[5:0]
 */
#define SPU_CH_ENV_ADDR_MID_POS                  (0)
#define SPU_CH_ENV_ADDR_MID_MSK                  (0x3FUL << SPU_CH_ENV_ADDR_MID_POS)

/*
 * Bit definition for SPU_CH->ENV_ADDR[15:0]
 */
#define SPU_CH_ENV_ADDR_POS                      (0)
#define SPU_CH_ENV_ADDR_MSK                      (0xFFFFUL << SPU_CH_ENV_ADDR_POS)

/*
 * Bit definition for SPU_CH->WAVE_DATA0[15:0]
 */
#define SPU_CH_WAVE_DATA0_POS                    (0)
#define SPU_CH_WAVE_DATA0_MSK                    (0xFFFFUL << SPU_CH_WAVE_DATA0_POS)

/*
 * Bit definition for SPU_CH->ENV_RAMPDOWN_STEP[15:9]
 */
#define SPU_CH_ENV_RAMPDOWN_STEP_POS             (9)
#define SPU_CH_ENV_RAMPDOWN_STEP_MSK             (0x7FUL << SPU_CH_ENV_RAMPDOWN_STEP_POS)
#define SPU_CH_ENV_RAMPDOWN_STEP_0x7F            (0x7FUL << SPU_CH_ENV_RAMPDOWN_STEP_POS)
#define SPU_CH_ENV_RAMPDOWN_STEP_0x10            (0x10UL << SPU_CH_ENV_RAMPDOWN_STEP_POS)
#define SPU_CH_ENV_RAMPDOWN_STEP_0x02            (0x02UL << SPU_CH_ENV_RAMPDOWN_STEP_POS)
#define SPU_CH_ENV_RAMPDOWN_STEP_0x01            (0x01UL << SPU_CH_ENV_RAMPDOWN_STEP_POS)
#define SPU_CH_ENV_RAMPDOWN_STEP_0x00            (0x00UL << SPU_CH_ENV_RAMPDOWN_STEP_POS)

/*
 * Bit definition for SPU_CH->ENV_RAMPDOWN_STEP[8:0]
 */
#define SPU_CH_ENV_REPEAT_OFFSET_POS             (0)
#define SPU_CH_ENV_REPEAT_OFFSET_MSK             (0x1FFUL << SPU_CH_ENV_REPEAT_OFFSET_POS)

/*
 * Bit definition for SPU_CH->WAVE_DATA[15:0]
 */
#define SPU_CH_WAVE_DATA_POS                     (0)
#define SPU_CH_WAVE_DATA_MSK                     (0xFFFFUL << SPU_CH_WAVE_DATA_POS)

/*
 * Bit definition for SPU_CH->WAVE_ADPCM_CTRL[15]
 */
#define SPU_CH_WAVE_ADPCM_SEL_POS                (15)
#define SPU_CH_WAVE_ADPCM_SEL_MSK                (0x1UL << SPU_CH_WAVE_ADPCM_SEL_POS)
#define SPU_CH_WAVE_ADPCM36                      (0x1UL << SPU_CH_WAVE_ADPCM_SEL_POS)
#define SPU_CH_WAVE_ADPCM_NORMAL                 (0x0UL << SPU_CH_WAVE_ADPCM_SEL_POS)

/*
 * Bit definition for SPU_CH->WAVE_ADPCM_CTRL[14]
 */
#define SPU_CH_WAVE_ADPCM_FINALFRAME_POS         (14)
#define SPU_CH_WAVE_ADPCM_FINALFRAME_MSK         (0x1UL << SPU_CH_WAVE_ADPCM_FINALFRAME_POS)
#define SPU_CH_WAVE_ADPCM_FINALFRAME             (0x1UL << SPU_CH_WAVE_ADPCM_FINALFRAME_POS)

/*
 * Bit definition for SPU_CH->WAVE_ADPCM_CTRL[13:9]
 */
#define SPU_CH_WAVE_ADPCM_FRAMEPOINT_POS         (9)
#define SPU_CH_WAVE_ADPCM_FRAMEPOINT_MSK         (0x1FUL << SPU_CH_WAVE_ADPCM_FRAMEPOINT_POS)
#define SPU_CH_WAVE_ADPCM_FRAMEPOINT_32P         (0x1FUL << SPU_CH_WAVE_ADPCM_FRAMEPOINT_POS)

/*
 * Bit definition for SPU_CH->WAVE_ADDRH[14:12]
 */
#define SPU_CH_WAVE_ADDRH1_POS                   (12)
#define SPU_CH_WAVE_ADDRH1_MSK                   (0x7UL << SPU_CH_WAVE_LOOP_ADDRH1_POS)

/*
 * Bit definition for SPU_CH->WAVE_ADDRH[11:6]
 */
#define SPU_CH_WAVE_LOOP_ADDRH_POS               (6)
#define SPU_CH_WAVE_LOOP_ADDRH_MSK               (0x3FUL << SPU_CH_WAVE_LOOP_ADDRH_POS)

/*
 * Bit definition for SPU_CH->WAVE_ADDRH[5:0]
 */
#define SPU_CH_WAVE_ADDRH_POS                    (0)
#define SPU_CH_WAVE_ADDRH_MSK                    (0x3FUL << SPU_CH_WAVE_ADDRH_POS)

/*
 * Bit definition for SPU_CH->ENV_ADDRH[8:0]
 */
#define SPU_CH_ENV_ADDRH_POS                     (0)
#define SPU_CH_ENV_ADDRH_MSK                     (0x1FFUL << SPU_CH_ENV_ADDRH_POS)

/*
 * Bit definition for SPU_CH->PHASE_VALUEH[2:0]
 */
#define SPU_CH_PHASE_VALUEH_POS                  (0)
#define SPU_CH_PHASE_VALUEH_MSK                  (0x7UL << SPU_CH_PHASE_VALUEH_POS)

/*
 * Bit definition for SPU_CH->PHASE_ACCH[2:0]
 */
#define SPU_CH_PHASE_ACCH_POS                    (0)
#define SPU_CH_PHASE_ACCH_MSK                    (0x7UL << SPU_CH_PHASE_ACCH_POS)

/*
 * Bit definition for SPU_CH->PITCHBEND_TARGETH[2:0]
 */
#define SPU_CH_PITCHBEND_TARGETH_POS             (0)
#define SPU_CH_PITCHBEND_TARGETH_MSK             (0x7UL << SPU_CH_PITCHBEND_TARGETH_POS)

/*
 * Bit definition for SPU_CH->ENV_RAMPDOWN_CLK[2:0]
 */
#define SPU_CH_ENV_RAMPDOWN_CLK_POS              (0)
#define SPU_CH_ENV_RAMPDOWN_CLK_MSK              (0x7UL << SPU_CH_ENV_RAMPDOWN_CLK_POS)

/*
 * Bit definition for SPU_CH->PHASE_VALUE[15:0]
 */
#define SPU_CH_PHASE_VALUE_POS                   (0)
#define SPU_CH_PHASE_VALUE_MSK                   (0xFFFFUL << SPU_CH_PHASE_VALUE_POS)

/*
 * Bit definition for SPU_CH->PHASE_ACC[15:0]
 */
#define SPU_CH_PHASE_ACC_POS                     (0)
#define SPU_CH_PHASE_ACC_MSK                     (0xFFFFUL << SPU_CH_PHASE_ACC_POS)

/*
 * Bit definition for SPU_CH->PITCHBEND_TARGET[15:0]
 */
#define SPU_CH_PITCHBEND_TARGET_POS              (0)
#define SPU_CH_PITCHBEND_TARGET_MSK              (0xFFFFUL << SPU_CH_PITCHBEND_TARGET_POS)

/*
 * Bit definition for SPU_CH->PITCHBEND_CTRL[15:13]
 */
#define SPU_CH_PITCHBEND_CLK_POS                 (13)
#define SPU_CH_PITCHBEND_CLK_MSK                 (0x7UL << SPU_CH_PITCHBEND_CLK_POS)

/*
 * Bit definition for SPU_CH->PITCHBEND_CTRL[12]
 */
#define SPU_CH_PITCHBEND_SIGN_POS                (12)
#define SPU_CH_PITCHBEND_SIGN_MSK                (0x1UL << SPU_CH_PITCHBEND_SIGN_POS)
#define SPU_CH_PITCHBEND_INCREMENT               (0x0UL << SPU_CH_PITCHBEND_SIGN_POS)
#define SPU_CH_PITCHBEND_DECREMENT               (0x1UL << SPU_CH_PITCHBEND_SIGN_POS)

/*
 * Bit definition for SPU_CH->PITCHBEND_CTRL[11:0]
 */
#define SPU_CH_PITCHBEND_STEP_POS                (0)
#define SPU_CH_PITCHBEND_STEP_MSK                (0xFFFUL << SPU_CH_PITCHBEND_STEP_POS)
#define SPU_CH_PITCHBEND_STEP_0xFFF              (0xFFFUL << SPU_CH_PITCHBEND_STEP_POS)


/*---------------------------------------------------------------------------------------
 * Peripheral memory map
 *---------------------------------------------------------------------------------------*/
#define SPI0_BASE                                (0x40090000)
#define SPI1_BASE                                (0x40091000)
#define UART0_BASE                               (0x400A0000)
#define UART1_BASE                               (0x400A1000)
#define I2C_BASE                                 (0x400B0000)
#define I2S_BASE                                 (0x400C0000)
#define WDG_BASE                                 (0x400D0000)
#define CTS_BASE                                 (0x400E0000)
#define TIMEBASE_BASE                            (0x400E0060)
#define SAR_ADC_BASE                             (0x40120000)
#define DS_ADC_BASE                              (0x40121000)
#define DAC_BASE                                 (0x40130000)
#define PWMIO_BASE                               (0x40160000)
#define QD_BASE                                  (0x40160050)
#define SMU_BASE                                 (0x50000000)
#define CLOCK_BASE                               (0x50001000)
#define RCU_BASE                                 (0x50003000)
#define ITU_BASE                                 (0x50004000)
#define ACU_BASE                                 (0x50005000)
#define KEYSCAN_BASE                             (0x50006000)
#define NVM_BASE                                 (0x50010000)
#define FLASH_BASE                               (0x50010000)
#define MAC_BASE                                 (0x50030000)
#define CCP0_BASE                                (0x50040000)
#define CCP1_BASE                                (0x50041000)
#define TM0_BASE                                 (0x50050000)
#define TM1_BASE                                 (0x50050010)
#define TM2_BASE                                 (0x50050020)
#define TM_INT_BASE                              (0x50050030)
#define SPIFC_BASE                               (0x50060000)
#define GPIOA_BASE                               (0x50070000)
#define GPIOA_BIT_OPERATION_BASE                 (0x50070020)
#define GPIOB_BASE                               (0x50070100)
#define GPIOB_BIT_OPERATION_BASE                 (0x50070120)
#define GPIOC_BASE                               (0x50070140)
#define GPIOC_BIT_OPERATION_BASE                 (0x50070160)
#define GPIOD_BASE                               (0x500701C0)
#define GPIOD_BIT_OPERATION_BASE                 (0x500701E0)
#define GPIOFUNC_BASE                            (0x50070200)
#define SPU_BASE                                 (0x50080E00)
#define DMA_INT_BASE                             (0x500F0000)
#define DMA0_BASE                                (0x500F0010)
#define DMA1_BASE                                (0x500F0020)
#define DMA2_BASE                                (0x500F0030)
#define DMA3_BASE                                (0x500F0040)
#define DMA4_BASE                                (0x500F0050)
#define USB_BASE                                 (0x40170004)
#define USB2_BASE                                (0x40180000)
#define BODYOPT_BASE                             (0x50005040)
#define EFUSE_BASE                               (0x50000050)
#define SYSTICK_BASE                             (0xE000E010)

//SPU Attribute Sram
#define SPU_BASE_CH0                             (0x50081000)
#define SPU_BASE_CH1                             (0x50081040)
#define SPU_BASE_CH2                             (0x50081080)
#define SPU_BASE_CH3                             (0x500810C0)
#define SPU_BASE_CH4                             (0x50081100)
#define SPU_BASE_CH5                             (0x50081140)
#define SPU_BASE_CH6                             (0x50081180)
#define SPU_BASE_CH7                             (0x500811C0)
#define SPU_BASE_CH8                             (0x50081200)
#define SPU_BASE_CH9                             (0x50081240)
#define SPU_BASE_CH10                            (0x50081280)
#define SPU_BASE_CH11                            (0x500812C0)
#define SPU_BASE_CH12                            (0x50081300)
#define SPU_BASE_CH13                            (0x50081340)
#define SPU_BASE_CH14                            (0x50081380)
#define SPU_BASE_CH15                            (0x500813C0)
#define SPU_BASE_CH16                            (0x50081400)
#define SPU_BASE_CH17                            (0x50081440)
#define SPU_BASE_CH18                            (0x50081480)
#define SPU_BASE_CH19                            (0x500814C0)
#define SPU_BASE_CH20                            (0x50081500)
#define SPU_BASE_CH21                            (0x50081540)
#define SPU_BASE_CH22                            (0x50081580)
#define SPU_BASE_CH23                            (0x500815C0)
#define SPU_BASE_CH24                            (0x50081600)
#define SPU_BASE_CH25                            (0x50081640)
#define SPU_BASE_CH26                            (0x50081680)
#define SPU_BASE_CH27                            (0x500816C0)
#define SPU_BASE_CH28                            (0x50081700)
#define SPU_BASE_CH29                            (0x50081740)
#define SPU_BASE_CH30                            (0x50081780)
#define SPU_BASE_CH31                            (0x500817C0)
#define SPU_BASE_CH32                            (0x50081800)
#define SPU_BASE_CH33                            (0x50081840)
#define SPU_BASE_CH34                            (0x50081880)
#define SPU_BASE_CH35                            (0x500818C0)
#define SPU_BASE_CH36                            (0x50081900)
#define SPU_BASE_CH37                            (0x50081940)
#define SPU_BASE_CH38                            (0x50081980)
#define SPU_BASE_CH39                            (0x500819C0)
#define SPU_BASE_CH40                            (0x50081A00)
#define SPU_BASE_CH41                            (0x50081A40)
#define SPU_BASE_CH42                            (0x50081A80)
#define SPU_BASE_CH43                            (0x50081AC0)
#define SPU_BASE_CH44                            (0x50081B00)
#define SPU_BASE_CH45                            (0x50081B40)
#define SPU_BASE_CH46                            (0x50081B80)
#define SPU_BASE_CH47                            (0x50081BC0)
#define SPU_BASE_CH48                            (0x50081C00)
#define SPU_BASE_CH49                            (0x50081C40)
#define SPU_BASE_CH50                            (0x50081C80)
#define SPU_BASE_CH51                            (0x50081CC0)
#define SPU_BASE_CH52                            (0x50081D00)
#define SPU_BASE_CH53                            (0x50081D40)
#define SPU_BASE_CH54                            (0x50081D80)
#define SPU_BASE_CH55                            (0x50081DC0)
#define SPU_BASE_CH56                            (0x50081E00)
#define SPU_BASE_CH57                            (0x50081E40)
#define SPU_BASE_CH58                            (0x50081E80)
#define SPU_BASE_CH59                            (0x50081EC0)
#define SPU_BASE_CH60                            (0x50081F00)
#define SPU_BASE_CH61                            (0x50081F40)
#define SPU_BASE_CH62                            (0x50081F80)
#define SPU_BASE_CH63                            (0x50081FC0)


/*---------------------------------------------------------------------------------------
 * Peripheral declaration
 *---------------------------------------------------------------------------------------*/
#define SPI0                                     ((SPI_TYPE_DEF *) SPI0_BASE)
#define SPI1                                     ((SPI_TYPE_DEF *) SPI1_BASE)
#define UART0                                    ((UART_TYPE_DEF *) UART0_BASE)
#define UART1                                    ((UART_TYPE_DEF *) UART1_BASE)
#define I2C                                      ((I2C_TYPE_DEF *) I2C_BASE)
#define I2S                                      ((I2S_TYPE_DEF *) I2S_BASE)
#define WDG                                      ((WDG_TYPE_DEF *) WDG_BASE)
#define CTS                                      ((CTS_TYPE_DEF *) CTS_BASE)
#define TIMEBASE                                 ((TIMEBASE_TYPE_DEF *) TIMEBASE_BASE)
#define SAR_ADC                                  ((SAR_ADC_TYPE_DEF *) SAR_ADC_BASE)
#define DS_ADC                                   ((DS_ADC_TYPE_DEF *) DS_ADC_BASE)
#define DAC                                      ((DAC_TYPE_DEF *) DAC_BASE)
#define PWMIO                                    ((PWMIO_TYPE_DEF *) PWMIO_BASE)
#define QD                                       ((QD_TYPE_DEF *) QD_BASE)
#define SMU                                      ((SMU_TYPE_DEF *) SMU_BASE)
#define CLOCK                                    ((CLOCK_TYPE_DEF *) CLOCK_BASE)
#define RCU                                      ((RCU_TYPE_DEF *) RCU_BASE)
#define ITU                                      ((ITU_TypeDef *) ITU_BASE)
#define ACU                                      ((ACU_TYPE_DEF *) ACU_BASE)
#define FLASH                                    ((FLASH_TYPE_DEF *) FLASH_BASE)
#define MAC                                      ((MAC_TYPE_DEF *) MAC_BASE)
#define CCP0                                     ((CCP_TYPE_DEF *) CCP0_BASE)
#define CCP1                                     ((CCP_TYPE_DEF *) CCP1_BASE)
#define TM0                                      ((TIMER_TYPE_DEF *) TM0_BASE)
#define TM1                                      ((TIMER_TYPE_DEF *) TM1_BASE)
#define TM2                                      ((TIMER_TYPE_DEF *) TM2_BASE)
#define TM_INT                                   ((TIMER_INT_TYPE_DEF *) TM_INT_BASE)
#define SPIFC                                    ((SPIFC_TYPE_DEF *) SPIFC_BASE)
#define GPIOA                                    ((GPIO_TYPE_DEF *) GPIOA_BASE)
#define GPIOA_OBIT                               ((GPIO_BIT_OPERATION_TYPE_DEF *) GPIOA_BIT_OPERATION_BASE)
#define GPIOB                                    ((GPIO_TYPE_DEF *) GPIOB_BASE)
#define GPIOB_OBIT                               ((GPIO_BIT_OPERATION_TYPE_DEF *) GPIOB_BIT_OPERATION_BASE)
#define GPIOC                                    ((GPIO_TYPE_DEF *) GPIOC_BASE)
#define GPIOC_OBIT                               ((GPIO_BIT_OPERATION_TYPE_DEF *) GPIOC_BIT_OPERATION_BASE)
#define GPIOD                                    ((GPIO_TYPE_DEF *) GPIOD_BASE)
#define GPIOD_OBIT                               ((GPIO_BIT_OPERATION_TYPE_DEF *) GPIOD_BIT_OPERATION_BASE)
#define IOFUNC                                   ((GPIOFUNC_TYPE_DEF *) GPIOFUNC_BASE)
#define SPU                                      ((SPU_TYPE_DEF *) SPU_BASE)
#define DMA_INT                                  ((DMA_INT_TYPE_DEF *) DMA_INT_BASE)
#define DMA0                                     ((DMA_TYPE_DEF *) DMA0_BASE)
#define DMA1                                     ((DMA_TYPE_DEF *) DMA1_BASE)
#define DMA2                                     ((DMA_TYPE_DEF *) DMA2_BASE)
#define DMA3                                     ((DMA_TYPE_DEF *) DMA3_BASE)
#define DMA4                                     ((DMA_TYPE_DEF *) DMA4_BASE)
#define USB                                      ((USB_TYPE_DEF *) USB_BASE)
#define USB2                                     ((USB2_TYPE_DEF *) USB2_BASE)
#define KEYSCAN                                  ((KEYSCAN_TYPE_DEF *) KEYSCAN_BASE)
#define BODYOPT                                  ((BODYOPT_TYPE_DEF *) BODYOPT_BASE)
#define EFUSE                                    ((EFUSE_TYPE_DEF *) EFUSE_BASE)
#define SYSTICK                                  ((SYSTICK_TYPE_DEF *) SYSTICK_BASE)

//SPU Attribute Sram
#define SPU_CH0                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH0 )    //attribute SRAM CH0
#define SPU_CH1                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH1 )    //attribute SRAM CH1
#define SPU_CH2                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH2 )    //attribute SRAM CH2
#define SPU_CH3                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH3 )    //attribute SRAM CH3
#define SPU_CH4                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH4 )    //attribute SRAM CH4
#define SPU_CH5                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH5 )    //attribute SRAM CH5
#define SPU_CH6                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH6 )    //attribute SRAM CH6
#define SPU_CH7                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH7 )    //attribute SRAM CH7
#define SPU_CH8                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH8 )    //attribute SRAM CH8
#define SPU_CH9                                  ((SPU_CH_TYPE_DEF *) SPU_BASE_CH9 )    //attribute SRAM CH9
#define SPU_CH10                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH10)    //attribute SRAM CH10
#define SPU_CH11                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH11)    //attribute SRAM CH11
#define SPU_CH12                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH12)    //attribute SRAM CH12
#define SPU_CH13                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH13)    //attribute SRAM CH13
#define SPU_CH14                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH14)    //attribute SRAM CH14
#define SPU_CH15                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH15)    //attribute SRAM CH15
#define SPU_CH16                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH16)    //attribute SRAM CH16
#define SPU_CH17                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH17)    //attribute SRAM CH17
#define SPU_CH18                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH18)    //attribute SRAM CH18
#define SPU_CH19                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH19)    //attribute SRAM CH19
#define SPU_CH20                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH20)    //attribute SRAM CH20
#define SPU_CH21                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH21)    //attribute SRAM CH21
#define SPU_CH22                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH22)    //attribute SRAM CH22
#define SPU_CH23                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH23)    //attribute SRAM CH23
#define SPU_CH24                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH24)    //attribute SRAM CH24
#define SPU_CH25                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH25)    //attribute SRAM CH25
#define SPU_CH26                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH26)    //attribute SRAM CH26
#define SPU_CH27                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH27)    //attribute SRAM CH27
#define SPU_CH28                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH28)    //attribute SRAM CH28
#define SPU_CH29                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH29)    //attribute SRAM CH29
#define SPU_CH30                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH30)    //attribute SRAM CH30
#define SPU_CH31                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH31)    //attribute SRAM CH31
#define SPU_CH32                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH32)    //attribute SRAM CH32
#define SPU_CH33                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH33)    //attribute SRAM CH33
#define SPU_CH34                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH34)    //attribute SRAM CH34
#define SPU_CH35                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH35)    //attribute SRAM CH35
#define SPU_CH36                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH36)    //attribute SRAM CH36
#define SPU_CH37                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH37)    //attribute SRAM CH37
#define SPU_CH38                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH38)    //attribute SRAM CH38
#define SPU_CH39                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH39)    //attribute SRAM CH39
#define SPU_CH40                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH40)    //attribute SRAM CH40
#define SPU_CH41                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH41)    //attribute SRAM CH41
#define SPU_CH42                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH42)    //attribute SRAM CH42
#define SPU_CH43                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH43)    //attribute SRAM CH43
#define SPU_CH44                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH44)    //attribute SRAM CH44
#define SPU_CH45                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH45)    //attribute SRAM CH45
#define SPU_CH46                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH46)    //attribute SRAM CH46
#define SPU_CH47                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH47)    //attribute SRAM CH47
#define SPU_CH48                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH48)    //attribute SRAM CH48
#define SPU_CH49                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH49)    //attribute SRAM CH49
#define SPU_CH50                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH50)    //attribute SRAM CH50
#define SPU_CH51                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH51)    //attribute SRAM CH51
#define SPU_CH52                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH52)    //attribute SRAM CH52
#define SPU_CH53                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH53)    //attribute SRAM CH53
#define SPU_CH54                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH54)    //attribute SRAM CH54
#define SPU_CH55                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH55)    //attribute SRAM CH55
#define SPU_CH56                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH56)    //attribute SRAM CH56
#define SPU_CH57                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH57)    //attribute SRAM CH57
#define SPU_CH58                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH58)    //attribute SRAM CH58
#define SPU_CH59                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH59)    //attribute SRAM CH59
#define SPU_CH60                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH60)    //attribute SRAM CH60
#define SPU_CH61                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH61)    //attribute SRAM CH61
#define SPU_CH62                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH62)    //attribute SRAM CH62
#define SPU_CH63                                 ((SPU_CH_TYPE_DEF *) SPU_BASE_CH63)    //attribute SRAM CH63


/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#if defined(__CC_ARM)
#include "RTE_Components.h"
#endif

#include "BootCode_SPIFC_GPCM3_FM1.h"
#include "DAC_GPCM3_FM1.h"
#include "DMA_GPCM3_FM1.h"
#include "DSADC_GPCM3_FM1.h"
#include "GPIO_GPCM3_FM1.h"
#include "I2C_GPCM3_FM1.h"
#include "MAC_GPCM3_FM1.h"
#include "SARADC_GPCM3_FM1.h"
#include "SPI_Flash_GPCM3_FM1.h"
#include "SPIFC_GPCM3_FM1.h"
#include "SYS_GPCM3_FM1.h"
#include "system_GPCM3_FM1.h"
#include "TimeBase_GPCM3_FM1.h"
#include "Timer_GPCM3_FM1.h"
#include "Uart_GPCM3_FM1.h"
#include "WDT_GPCM3_FM1.h"

/*---------------------------------------------------------------------------------------
 * Macro
 *---------------------------------------------------------------------------------------*/
#define SET_BIT(REG, BIT)                        ((REG) |= (BIT))
#define WRITE_REG(REG, VAL)                      ((REG) = (VAL))

#define READ_BIT(REG, BIT)                       ((REG) & (BIT))
#define READ_REG(REG)                            ((REG))

#define CLEAR_BIT(REG, BIT)                      ((REG) &= ~(BIT))
#define CLEAR_FLAG(REG, MASKBIT, CLEARBIT)       ((REG) &= ((MASKBIT) | (CLEARBIT)))
#define CLEAR_REG(REG)                           ((REG) = (0x00000000))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)      WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define SHIFT_L(CONTENT, BIT)                    ((CONTENT) << (BIT))
#define SHIFT_R(CONTENT, BIT)                    ((CONTENT) >> (BIT))

#define XOR_BIT(REG, BIT)                        ((REG) ^= (BIT))


/*---------------------------------------------------------------------------------------
 * Constant Definition
 *---------------------------------------------------------------------------------------*/
/*
 * Bit Mask Definitions
 */
#define BIT0                                     (0x00000001)
#define BIT1                                     (0x00000002)
#define BIT2                                     (0x00000004)
#define BIT3                                     (0x00000008)
#define BIT4                                     (0x00000010)
#define BIT5                                     (0x00000020)
#define BIT6                                     (0x00000040)
#define BIT7                                     (0x00000080)
#define BIT8                                     (0x00000100)
#define BIT9                                     (0x00000200)
#define BIT10                                    (0x00000400)
#define BIT11                                    (0x00000800)
#define BIT12                                    (0x00001000)
#define BIT13                                    (0x00002000)
#define BIT14                                    (0x00004000)
#define BIT15                                    (0x00008000)
#define BIT16                                    (0x00010000)
#define BIT17                                    (0x00020000)
#define BIT18                                    (0x00040000)
#define BIT19                                    (0x00080000)
#define BIT20                                    (0x00100000)
#define BIT21                                    (0x00200000)
#define BIT22                                    (0x00400000)
#define BIT23                                    (0x00800000)
#define BIT24                                    (0x01000000)
#define BIT25                                    (0x02000000)
#define BIT26                                    (0x04000000)
#define BIT27                                    (0x08000000)
#define BIT28                                    (0x10000000)
#define BIT29                                    (0x20000000)
#define BIT30                                    (0x40000000)
#define BIT31                                    (0x80000000)

/*
 * Byte Mask Definitions
 */
#define BYTE0_MSK                                (0x000000FF)
#define BYTE1_MSK                                (0x0000FF00)
#define BYTE2_MSK                                (0x00FF0000)
#define BYTE3_MSK                                (0xFF000000)

#define TRUE                                     (1)
#define FALSE                                    (0)

#define ENABLE                                   (1)
#define DISABLE                                  (0)

#endif

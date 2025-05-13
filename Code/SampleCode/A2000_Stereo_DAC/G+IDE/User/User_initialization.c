
/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "stdbool.h"


void BootCode_SystemInit(void);

/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/


/*********************************************************************
 *
 *  main()
 *********************************************************************/
void User_Initialization()
{

    /*
     * SPIFC init
     */
    BootCode_SystemInit();
    BootCode_SPIFC_Close();
    BootCode_SPIFC_Open();											 // SPIFC open
    BootCode_SPIFC_SetClkDiv(SPIFC_CLKSEL_HCLK_DIV1);
    BootCode_SPIFC_AutoMode(SPIFC_4IO_ENHANCE_MODE);					         // SPIFC auto mode.
    //SPIFC_AutoMode(SPIFC_2IO_MODE);					         // SPIFC auto mode.
    if(BootCode_SPIFC_TimingFineTune() == SPIFC_CALIBRATION_FAIL)     // Calibrate SPIFC clock timing.
    {
        while(1); // SPIFC clock timing calibration fail!
    }

    __ASM volatile ("ldr   r0, =user_Reset_Handler");
    __ASM volatile ("blx   r0");
}

void BootCode_SystemInit(void)
{
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
    MODIFY_REG(ACU->PLL_CTRL, ACU_PLL_CTRL_SYS_DIVC_MSK, ACU_PLL_CTRL_SYS_DIVC_122D88M);
    MODIFY_REG(ACU->PLL_SPI_DIV, ACU_PLL_SPI_DIV_DIVC_MSK, ACU_PLL_SPI_DIV_DIVC_98D304M);
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);
}




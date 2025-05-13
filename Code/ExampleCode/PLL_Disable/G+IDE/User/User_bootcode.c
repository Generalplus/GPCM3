
/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include "GPCM3_FM1.h"
#include "stdbool.h"


/*---------------------------------------------------------------------------------------
 * Prototype Function Declaration Area
 *---------------------------------------------------------------------------------------*/
void BootCode_SystemInit(void);
bool IntRomCheck(void);

extern uint8_t __user_init_load_addr;
extern uint8_t __user_init_start;
extern uint32_t __user_init_size;


#ifndef __NO_SYSTEM_INIT
#ifdef __cplusplus
extern "C"
#endif
void SystemInit_user()
{}
#endif


/*---------------------------------------------------------------------------------------
 * Gobal Variable Declaration Area
 *---------------------------------------------------------------------------------------*/


/*********************************************************************
 *
 *  main()
 *********************************************************************/

void MoveUserIniCode(void)
{
    uint8_t *pCode_LDA = &__user_init_load_addr;
    uint8_t *pCode_VMA = &__user_init_start;
    uint32_t l_CodeSize = (uint32_t)&__user_init_size;
    uint32_t iCount;

    for(iCount=0;iCount<l_CodeSize;iCount++)
    {
      pCode_VMA[iCount] = pCode_LDA[iCount];
    }

    return;
}

void User_BootCode()
{
    /******************************************************************
    *  Place your code here.
    ******************************************************************/
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
    MODIFY_REG(ACU->PLL_CTRL, ACU_PLL_CTRL_SYS_DIVC_MSK, ACU_PLL_CTRL_SYS_DIVC_122D88M);
    MODIFY_REG(ACU->PLL_SPI_DIV, ACU_PLL_SPI_DIV_DIVC_MSK, ACU_PLL_SPI_DIV_DIVC_15D360M);
    MODIFY_REG(ACU->PLL_CTRL, ACU_PLL_CTRL_PLL_EN_MSK, ACU_PLL_CTRL_PLL_DISABLE);   // PLL diabled
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);

    if(IntRomCheck() == false)
    {
      while(1);
    }

    if(SMU->SHUTDOWN_CTRL & SMU_SHUTDOWN_CTRL_SD_FLAG)
    {
      // Wakeup mode
    }

    MoveUserIniCode();
    User_Initialization();
}

bool IntRomCheck(void)
{
    uint8_t *pIntRom = 0;
    uint32_t iCount,Checksum;
    uint32_t *goldenChecksum = (uint32_t *)0x1FFC;

    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY1);
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, SMU_SYSLOCK_UNLOCK_KEY2);	       // SMU nulock
    MODIFY_REG(ACU->PLL_CTRL, ACU_PLL_CTRL_SYS_DIVC_MSK, ACU_PLL_CTRL_SYS_DIVC_61D440M);
    MODIFY_REG(SMU->SYSLOCK, SMU_SYSLOCK_UNLOCK_KEY_MSK, 0);

    for(iCount=0x100,Checksum=0;iCount<0x1FFc;iCount++)
    {
      Checksum += pIntRom[iCount];
    }
    if(Checksum != goldenChecksum[0])
    {
      return false;
    }
    else
    {
      return true;
    }
}


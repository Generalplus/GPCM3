/************************************************************************************
 * File:    startup.S
 * Purpose: Startup file for GPCM1xxx devices.
 *          Should use with GCC for ARM Embedded Processors
 * Version: V1.0
 * Date:    8 March 2017
 * Notice:  Changed for use with ARM IDE project wizard
 * Date:    8 March 2017
 *
 ************************************************************************************/
  .syntax unified
  .arch	armv6-m
  /* .fpu softvfp */


/************************************************************************************
 *
 *  The minimal vector table for GPCM1xxx.
 *  Device specific external interrupts can be added below.
 ************************************************************************************/
    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    //.long    __stack_end__         /* Top of Stack */
    .long    0x20004000            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    int_NMI_Handler           /* NMI Handler */
    .long    int_HardFault_Handler     /* Hard Fault Handler */
    .long    0                      /* Reserved */
    .long    0                      /* Reserved */
    .long    0                      /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    int_SVC_Handler           /* SVCall Handler */
    .long    0                    /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    int_PendSV_Handler        /* PendSV Handler */
    .long    int_SysTick_Handler       /* SysTick Handler */


    /* External interrupts */
    .long     int_USB_IRQHandler            //  0: USB Interrupt
    .long     int_VKEY_IRQHandler           //  1: Velocity Key	Interrupt
    .long     int_EXTI_IRQHandler           //  2: EXT0~3 Interrupt
    .long     int_MAC_IRQHandler            //  3: MAC Interrupt
    .long     int_QD_IRQHandler             //  4: QD Interrupt
    .long     int_SAR_ADC_IRQHandler        //  5: SAR-ADC Interrupt
    .long     int_DS_ADC_IRQHandler	        //  6: DS-ADC Interrupt
    .long     int_DAC_CH0_IRQHandler        //  7: DAC_CH0 Interrupt
    .long     int_DAC_CH1_IRQHandler        //  8: DAC_CH1 Interrupt
    .long     int_CCP0_IRQHandler           //  9: CCP0 Interrupt
    .long     int_CCP1_IRQHandler           // 10: CCP1 Interrupt
    .long     int_CTS_TM0_IRQHandler        // 11: CTS TM0 Interrupt
    .long     int_CTS_TM1_IRQHandler        // 12: CTS TM1 Interrupt
    .long     int_TIMEBASE_IRQHandler		    // 13: Timebase Interrupt
    .long     int_I2C_IRQHandler            // 14: I2C Interrupt
    .long     int_SPU_IRQHandler            // 15: SPU ENV, Pwspu, SPUfiq Interrupt
    .long     int_UART0_IRQHandler     	    // 16: UART0 Interrupt
    .long     int_UART1_IRQHandler     	    // 17: UART1 Interrupt
    .long     int_I2S_IRQHandler            // 18: I2S Interrupt
    .long     int_SPI0_IRQHandler           // 19: SPI0 Interrupt
    .long     int_SPI1_IRQHandler           // 20: SPI1 Interrupt
    .long     int_DMA0_IRQHandler           // 21: DMA0 Interrupt
    .long     int_DMA1_IRQHandler           // 22: DMA1 Interrupt
    .long     int_DMA2_IRQHandler           // 23: DMA2 Interrupt
    .long     int_DMA3_IRQHandler           // 24: DMA3 Interrupt
    .long     int_DMA4_IRQHandler           // 25: DMA4 Interrupt
    .long     int_TIMER0_IRQHandler         // 26: Timer0 Interrupt
    .long     int_TIMER1_IRQHandler         // 27: Timer1 Interrupt
    .long     int_TIMER2_IRQHandler         // 28: Timer2 Interrupt
    .long     int_KEYCHG_IRQHandler         // 29: Key Change Inerrupt
    .long     int_PDM_IRQHandler            // 30: PDM Interrupt
    .long     int_SPUBEAT_IRQHandler        // 31: SPU Beat Interrupt

    .size    __isr_vector, . - __isr_vector

/************************************************************************************
 *
 *  Reset_Handler()
 *  This function gets called at start of execution after a reset event.
 *  Copies data from ROM to RAM, clears BSS if defined,
 *  calls SystemInit() if defined, finally calls main()
 ************************************************************************************/
    .text
    .thumb
    .thumb_func
    .align 1
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    //
    // Clear watch dog
    //
    LDR		R0,	= 0x400D0000
    LDR		R1, = 0x000000AA
    STR		R1,	[R0]

    //
    // reset watch dog control register
    //
    LDR		R1, = 0x00000055
    STR		R1,	[R0]

    LDR		R2,	= 0x400D0004
    //LDR		R1, = 0x0FFF1210
    LDR		R1, = 0x0FFF1310  //CLK SEL -> LSIRC/16
    STR		R1,	[R2]

    LDR		R1, = 0x00000044
    STR		R1,	[R0]

    //
    // reset stack pointer
    //
    LDR   R0, = 0x20004000
    MOV   SP, R0

    LDR   R0, = 0x50000020  //Shut Down flag register
    LDR   R1, = 0x80000000  //Shut Down flag
    LDR   R0, [R0]
    TST   R0, R1
    BEQ   L_SystemInitial

    //
    // check BVD Value
    //
    LDR   R0, = 0x50005030  //ACU_BVD_CTRL
    LDR   R1, = 0x00000010  //BVDEN
    STR   R1, [R0]
    LDR   R1, = 0x0000000F
    LDR   R2, [R0]
    ANDS  R2, R2, R1
    BEQ   EnterPowerDownMode
L_SystemInitial:
    //
    // Init cache
    //
    LDR		R0,	= 0x50000030        // SMU_CACHE_CTRL
    LDR   R1, = 0
    STR		R1,	[R0]                 // Disable cache
    NOP                              // Add 4 NOP after cache disable
    NOP
    NOP
    NOP
    LDR   R1, = 0x1
    STR   R1, [R0]                // Cache enable

    //
    // Read IOA Pad status
    //
    LDR		R2,	= 0x50070018
    LDR   R2, [R2]
    PUSH {R2}

    //
    // Clear Shut down mode
    //
    LDR		R0,	= 0x5000005C  //SD_OFF
    LDR		R1, = 0x000000AA
    STR		R1,	[R0]

    LDR		R3,	= 0x50000000  //System unlock
    LDR		R1, = 0x000000AB
    STR		R1,	[R3]
    LDR   R1, = 0x00000012
    STR		R1,	[R3]

    //
    // Enable PLL
    //
    LDR		R0,	= 0x5000500C  //PLL Control
    LDR		R1, = 0x00000021
    STR		R1,	[R0]

    LDR   R1, = 0x0         //System lock
    STR   R1, [R3]

    //
    // Read boot table
    //
    LDR   R0, = BootLoaderProcess
    BLX   R0

    POP   {R2}

    //
    // Check option of SRAM size
    //
    LDR   R0, = 0x50000050 //eFuse_A
    LDR   R0, [R0]
    LDR   R1, = 0x00200000  // bit21 -> refer fuse flag
    TST   R0, R1
    BEQ   L_EVChip
    LDR   R1, = 0x00080000  // bit19
    TST   R0, R1
    BEQ   L_ReadBootCode
    B     L_SRAM_32KB
L_EVChip:
    LDR   R0, = 0x50005054 //BodyOption_4
    LDR   R1, = 0x00000040  //bit6
    LDR   R0, [R0]
    TST   R0, R1
    BEQ   L_ReadBootCode
L_SRAM_32KB:
    LDR   R0, = 0x20008000
    MOV   SP, R0
L_ReadBootCode:

    PUSH  {R2}
    //
    // Read user's boot code
    //
    LDR   R0, = ReadBootCodeProcess
    BLX   R0

    POP   {R2}
    LDR   R0, = 0x200000C4
    STR   R2, [R0]

    //LDR   R0, = 0x20000004
    LDR   R0, = 0x200000C0
    LDR   R0, [R0]
    BX    R0


/************************************************************************************
 *
 *  EnterPowerDownMode()
 ************************************************************************************/
    .text
    .thumb
    .thumb_func
    .align 1
    .globl    EnterPowerDownMode
    .type    EnterPowerDownMode, %function
EnterPowerDownMode:
    //
    // Disable GPIOA_IE
    //
    LDR		R0,	= 0x5007000C
    LDR		R1, = 0x00000000
    STR		R1,	[R0]

    //
    // Enable I16K wakeup source
    //
    LDR		R0,	= 0x50000060  //System Control Register
    LDR		R1, = 0x00000001
    LDR   R2, [R0]
    ORRS  R2, R2, R1
    STR		R2,	[R0]          //I16K wakeup source enable

    LDR		R0,	= 0x50000058  //SD_ON
    LDR		R1, = 0x00000055  //Shut down mode enable
    STR		R1,	[R0]

    LDR		R0,	= 0xE000ED10  //System Control Register
    LDR		R1, = 0x00000004
    LDR   R2, [R0]
    ORRS  R2, R2, R1
    STR		R2,	[R0]          //Deep sleep mode

    WFI



    .pool
    .size Reset_Handler, . - Reset_Handler

/************************************************************************************
 *
 *  Weak definition for exceptions.
 *  Any function with the same name will override the weak definition.
 ************************************************************************************/
/*  Macro to define default handlers. Default handler
 *  will be weak symbol and just dead loops. They can be
 *  overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_default_handler     int_NMI_Handler
    def_default_handler     int_HardFault_Handler
    def_default_handler     int_SVC_Handler
    def_default_handler     int_PendSV_Handler
    def_default_handler     int_SysTick_Handler

    def_default_handler     int_Default_Handler
    def_default_handler     int_WDG_IRQHandler

    def_default_handler     int_USB_IRQHandler
    def_default_handler     int_VKEY_IRQHandler
    def_default_handler     int_EXTI_IRQHandler
    def_default_handler     int_MAC_IRQHandler
    def_default_handler     int_QD_IRQHandler
    def_default_handler     int_SAR_ADC_IRQHandler
    def_default_handler     int_DS_ADC_IRQHandler
    def_default_handler     int_DAC_CH0_IRQHandler
    def_default_handler     int_DAC_CH1_IRQHandler
    def_default_handler     int_CCP0_IRQHandler
    def_default_handler     int_CCP1_IRQHandler
    def_default_handler     int_CTS_TM0_IRQHandler
    def_default_handler     int_CTS_TM1_IRQHandler
    def_default_handler     int_TIMEBASE_IRQHandler
    def_default_handler     int_I2C_IRQHandler
    def_default_handler     int_SPU_IRQHandler
    def_default_handler     int_UART0_IRQHandler
    def_default_handler     int_UART1_IRQHandler
    def_default_handler     int_I2S_IRQHandler
    def_default_handler     int_SPI0_IRQHandler
    def_default_handler     int_SPI1_IRQHandler
    def_default_handler     int_DMA0_IRQHandler
    def_default_handler     int_DMA1_IRQHandler
    def_default_handler     int_DMA2_IRQHandler
    def_default_handler     int_DMA3_IRQHandler
    def_default_handler     int_DMA4_IRQHandler
    def_default_handler     int_TIMER0_IRQHandler
    def_default_handler     int_TIMER1_IRQHandler
    def_default_handler     int_TIMER2_IRQHandler
    def_default_handler     int_KEYCHG_IRQHandler
    def_default_handler     int_PDM_IRQHandler
    def_default_handler     int_SPUBEAT_IRQHandler
    .end


  .section .ROMCODEVERSION
  .align 2
  .globl __ViaRomCodeVer
__ViaRomCodeVer:

  .long    0x01000000             /* CM3X */



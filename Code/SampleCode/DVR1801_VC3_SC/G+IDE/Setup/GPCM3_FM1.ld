/************************************************************************************
 *  File:     GPCM3_FM1.ld
 *  Purpose:  Linker script to configure memory regions for GPCM3_FM1
 *            and place sections and symbol values.
 *  Date:     8 March 2017
 *  Info:     Reset_Handler must be defined in code as entry point of reset handler.
 *
 *            By default the script is configured for the first flash and RAM bank
 *            of the device.
 *            If additional banks of the device shall be used,
 *            they can be configured manually.
 *
 *            Following symbols are defined in the script
 *            and can be used without definition in code:
 *
 ************************************************************************************/

/************************************************************************************
 *
 *  Memory regions and required amount of heap and stack
 ************************************************************************************/

/* --------------------------------------------------------------------------- */
/* memory layout and stack/heap size value 									   */
/* --------------------------------------------------------------------------- */
MEMORY
{
  BOOTHEADER (rx)   : ORIGIN = 0x04000000, LENGTH = 0x00000100
  VECTOR(rx)        : ORIGIN = 0x04000100, LENGTH = 0x00000100
  SPIFC (rx)  		  : ORIGIN = 0x04000200, LENGTH = 0x00100000 - 0x200
  RESOURCE   (rx)	  : ORIGIN = 0x04100000, LENGTH = 0x04000000 - 0x00100000
  RAM  (xrw)        : ORIGIN = 0x20000000, LENGTH = 0x00008000
}
_Min_Heap_Size  = 0x010;  /* required amount of heap  */
_Min_Stack_Size = 0x300;  /* required amount of stack */

/************************************************************************************
 *
 *  Code and data sections
 ************************************************************************************/
ENTRY(user_Reset_Handler)

SECTIONS
{
 /************************************************************************************
 * User Boot code
 ************************************************************************************/
  .bootheader :
  {
    __boot_table_start = .;
    KEEP(*(.BOOTHEADER*))     /*  User\BootHeader.s */

  } > BOOTHEADER

  .vector ORIGIN(RAM) : AT (ORIGIN(VECTOR))
  {
    __boot_code_start = .;
    /* vector */
    KEEP(*(.user_isr_vector*))  /*  User\startup.S */
    __vector_End = .;
  } > RAM

  .BootCode_SEC __vector_End : AT (ORIGIN(SPIFC))
  {
    /* Startup code */
    /*  *User_startup.o(.text* .rodata* .constdata* .data* .uninit* .bss*) */   /*  User\startup.S */
    *User_bootcode.o(.text* .rodata* .constdata* .data* .uninit* .bss* COMMON .ARM.attributes)    /*  User\startup.S */
    /**BootCode_SPIFC_GPCM3_FM1.o(.text* .rodata* .constdata* .data* .uninit* .bss* COMMON .ARM.attributes)*/    /*  User\startup.S */
  } > RAM
  PROVIDE(__boot_sec_size = . - __boot_code_start);
  . = ALIGN(4);
  PROVIDE(__user_init_start = .);
  __user_init_load_addr = (ORIGIN(SPIFC)) + (SIZEOF(.BootCode_SEC));

  .User_Init_SEC __user_init_start : AT ((ORIGIN(SPIFC)) + SIZEOF(.BootCode_SEC))
  {
    /* Startup code */
    *User_initialization.o(.text* .rodata* .constdata* .data* .uninit* .bss* COMMON .ARM.attributes)    /*  User\startup.S */
    *BootCode_SPIFC_GPCM3_FM1.o(.text* .rodata* .constdata* .data* .uninit* .bss* COMMON .ARM.attributes)    /*  User\startup.S */
  } > RAM
  PROVIDE(__user_init_size = . - __user_init_start);

  . = ALIGN(4);
  .spifc_ramcode_SEC __vector_End : AT ((ORIGIN(SPIFC)) + SIZEOF(.BootCode_SEC) + SIZEOF(.User_Init_SEC))
  {
    /* Startup code */
    *SPIFC_GPCM3_FM1.o(.text* .rodata* .constdata* .data* .uninit* .bss* COMMON .ARM.attributes)    /*  User\startup.S */
    *DMA_GPCM3_FM1.o(.text* .rodata* .constdata* .data* .uninit* .bss* COMMON .ARM.attributes)    /*  User\startup.S */
    __spifc_ramcode_end = .;
  } > RAM
  PROVIDE(__user_spifc_load_addr = (ORIGIN(SPIFC)) + SIZEOF(.BootCode_SEC) + SIZEOF(.User_Init_SEC));
  PROVIDE(__user_spifc_start = __vector_End);
  PROVIDE(__user_spifc_size = . - __user_spifc_start);

/************************************************************************************
 *
 *  text section (code)
 *  Starts with startup code and vectors
 *  Goes into FLASH
 ************************************************************************************/
    .RESOURCE_ :
  {
   . = ALIGN(4);
    KEEP(*A1801_FileMerger.o (.data*))
  } > RESOURCE

  . = ALIGN(4);
  .text ORIGIN(SPIFC) + SIZEOF(.BootCode_SEC) + SIZEOF(.User_Init_SEC) + SIZEOF(.spifc_ramcode_SEC):
  {
    . = ALIGN(4);
    KEEP(*dsp.o)
    . = ALIGN(4);
    __SPIFC_Timing_Calibration_Data = .;
    KEEP(*SPIFC_TimingFineTuneBin_0x5A810A3C.o (.SPIFC_Timing_Calibration_Data*))

    /* Code section */
    *(.text*)
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */

    KEEP(*(.init))
    KEEP(*(.fini))

    /* Rodata section (constants, strings, ...) */
    *(.rodata*)
    *(.constdata*)
    *(i.*)

	/* Generalplus Library Tag Information */
    KEEP(*(.libtag*))
    KEEP(*(*GP_LIBRARY_TAG_SECTION*))

    KEEP(*(.eh_frame*))
    KEEP (*(.ARM.attributes))

  } > SPIFC

/************************************************************************************
 *
 *  exception index table
 ************************************************************************************/
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > SPIFC

  __exidx_start = .;
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > SPIFC
  __exidx_end = .;

  /* Global symbol at the end of code (text section) */
  PROVIDE(__etext = .);
  PROVIDE(_etext = .);
  PROVIDE(etext = .);


/************************************************************************************
 *
 *  data section
 *  Goes into RAM
 ************************************************************************************/
  /*.data __vector_End : AT (__etext)*/
  .data __spifc_ramcode_end : AT (__etext)
  {
    /* Global symbol at start of data */
    __data_start__ = .;
    __data_start = . ;
    *(vtable)
    *(.data*)

    . = ALIGN(4);
    /* preinit data */
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP(*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);

    . = ALIGN(4);
    /* init data */
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);


    . = ALIGN(4);
    /* finit data */
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);


    KEEP(*(.jcr*))
    . = ALIGN(4);
    /* Global symbol at end of data */
    __data_end__ = .;
    _edata = .;
    PROVIDE (edata = .);

  } > RAM
   ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(SPIFC) + LENGTH(SPIFC), "Error: .data will not fit in region `SPIFC'")

/************************************************************************************
 *
 *  uninit section (Uninitialized data without run-time clean-up)
 *  Goes into RAM
 ************************************************************************************/

  .uninit (NOLOAD) : AT (LOADADDR(.data) + SIZEOF(.data))
  {
    . = ALIGN(4);
    /* Global symbol at start of uninit */
    __uninit_start__ = .;
    *(.uninit*)
    . = ALIGN(4);
    /* Global symbol at end of uninit */
    __uninit_end__ = .;
    PROVIDE (euninit = .);
  }> RAM
   ASSERT(LOADADDR(.uninit) + SIZEOF(.uninit) <= ORIGIN(SPIFC) + LENGTH(SPIFC), "Error: .uninit will not fit in region `SPIFC'")


/************************************************************************************
 *
 *  bss section (Uninitialized data with run-time clean-up)
 *  Goes into RAM
 ************************************************************************************/
  .bss euninit : AT (euninit)
  {
    . = ALIGN(4);
    /* Global symbol at start of bss */
    __bss_start = .;
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    /* Global symbol at end of bss */
    _bss_end__ = .;
    __bss_end__ = .;
    __end__ = . ;
    _end = .;
    PROVIDE (end = .);
  } > RAM

/************************************************************************************
 *
 *  user_heap_stack section
 *  Check that there is enough RAM left for stack and heap
 ************************************************************************************/
   end_of_ram = ORIGIN(RAM)+ LENGTH(RAM)-(_Min_Heap_Size + _Min_Stack_Size);
  ._user_heap_stack end_of_ram : AT (end_of_ram)
  {
    . = ALIGN(4);
    __heap_start__ = .;
    . = . + _Min_Heap_Size;
    __heap_end__ = .;
    . = ALIGN(4);
    _stack = .;
    __stack_start__ = .;
    . = . + _Min_Stack_Size;
    __stack_end__ = .;
    _estack = .;
    . = ALIGN(4);
  } >RAM


/************************************************************************************
 *
 *  Remove information from the standard libraries
 ************************************************************************************/
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

 /************************************************************************************
 *
 *  ARM debug information
 ************************************************************************************/
  /*.ARM.attributes 0 : { KEEP (*(.ARM.attributes)) }*/
}

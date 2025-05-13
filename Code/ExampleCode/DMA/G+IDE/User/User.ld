/************************************************************************************
 *  File:     GPCM1xxx.ld
 *  Purpose:  Linker script to configure memory regions for GPCM1xxx
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
  OPTIONS(rx)	    : ORIGIN = 0x04000000, LENGTH = 0x00000100
  SPIFC (xrx)  		: ORIGIN = 0x04000100, LENGTH = 0x02000000 - 0x100
  RAM  (xrw)        : ORIGIN = 0x20000000, LENGTH = 0x00003400

}
_Min_Heap_Size  = 0x200;  /* required amount of heap  */
_Min_Stack_Size = 0x400;  /* required amount of stack */

/************************************************************************************
 *
 *  Code and data sections
 ************************************************************************************/
ENTRY(Reset_Handler)

SECTIONS
{

/************************************************************************************
 *
 *  text section (code)
 *  Starts with startup code and vectors
 *  Goes into FLASH
 ************************************************************************************/

  .options :
  {
    __boot_table_start = .;
    KEEP(*(.BOOTHEADER*))
  } > OPTIONS

   .BootCode_SEC : AT (ORIGIN(SPIFC))
  {
    /* Startup code */
    KEEP(*(.isr_vector*))
    *startup.o(.text*)
  } > RAM

  PROVIDE(__boot_sec_size = SIZEOF(.BootCode_SEC));

  .text ORIGIN(SPIFC) + SIZEOF(.BootCode_SEC) :
  {
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
  .data : AT (__etext)
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
   ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(FLASH) + LENGTH(FLASH), "Error: .data will not fit in region `FLASH'")

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
   ASSERT(LOADADDR(.uninit) + SIZEOF(.uninit) <= ORIGIN(FLASH) + LENGTH(FLASH), "Error: .uninit will not fit in region `FLASH'")


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
  ._user_heap_stack :
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
 *  user_options section
 *
 ************************************************************************************/

 .USER_OPT0 0x0F000000 : {KEEP(*(.USER_OPT0 ))}
 .USER_OPT1 0x0F000004 : {KEEP(*(.USER_OPT1 ))}
 .UOPT2     0x0F000008 : {KEEP(*(.UOPT2 ))}
 .UOPT3     0x0F00000C : {KEEP(*(.UOPT3 ))}
 .UNUSED    0x0F000010 : {. = . + 0x10;} = 0xFF

/************************************************************************************
 *
 *  body_options section
 *
 ************************************************************************************/

 .UOPT8      0x0F000020 : {KEEP(*(.UOPT8 ))}
 .UOPT9      0x0F000024 : {KEEP(*(.UOPT9 ))}
 .UOPT10     0x0F000028 : {KEEP(*(.UOPT10 ))}
 .UOPT11     0x0F00002C : {KEEP(*(.UOPT11 ))}
 .UOPT12     0x0F000030 : {KEEP(*(.UOPT12 ))}

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
  .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) }
}

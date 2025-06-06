#ifndef __FILE_MERGER_H__
#define __FILE_MERGER_H__

/* This file is automatically generated by G+ FileMerger.
   Do not modify the contents to prevent from invalid addressing.
*/ 

#if defined (__GNUC__) || defined (__CC_ARM)  || defined (__ICCARM__) 	/* 32 bit ARM compiler */

	#include <stdint.h>
	#if defined (__CSRC_USED__)					/* use C source instead of elf obj file */
		extern const uint8_t* MS02_FileMerger_bin_Data;
		#define SEC_START_ADDR MS02_FileMerger_bin_Data
	#else
		extern uint8_t _binary_MS02_FileMerger_bin_start;
		extern uint32_t _binary_MS02_FileMerger_bin_size;

		#define SEC_START_ADDR &_binary_MS02_FileMerger_bin_start
	#endif

#else
	#define SEC_START_ADDR 0
#endif

#define _GROUP_TABLE                   0       	//Group Index:0           Group Name:Group_Table     Group Type:Unknown 
#define _GROUP_TABLE_ADDR              (SEC_START_ADDR + 0x000030)	//Group Addr :0x000030    Group Name:Group_Table     Group Type:Unknown 
#define _GPCM1_LIB                     0       	//File  Index:0           File  Name:GPCM1_LIB.lib32 
#define _GPCM1_LIB_ADDR                (SEC_START_ADDR + 0x000030)	//File  Addr :0x000030    File  Name:GPCM1_LIB.lib32 
#define _GPCM1_MIDI                    1       	//File  Index:1           File  Name:GPCM1_MIDI.BIN 
#define _GPCM1_MIDI_ADDR               (SEC_START_ADDR + 0x00ae76)	//File  Addr :0x00ae76    File  Name:GPCM1_MIDI.BIN 

#endif /* __FILE_MERGER_H__ */

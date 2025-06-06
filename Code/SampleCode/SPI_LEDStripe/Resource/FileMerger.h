#ifndef __FILE_MERGER_H__
#define __FILE_MERGER_H__

/* This file is automatically generated by G+ FileMerger.
   Do not modify the contents to prevent from invalid addressing.
*/ 

#if defined (__GNUC__) || defined (__CC_ARM)	/* 32 bit ARM compiler */

	#include <stdint.h>
	#if defined (__CSRC_USED__)					/* use C source instead of elf obj file */
		extern const uint8_t* PcmROM_bin_Data;
		#define SEC_START_ADDR PcmROM_bin_Data
	#else
		extern uint8_t _binary_PcmROM_bin_start;
		extern uint32_t _binary_PcmROM_bin_size;

		#define SEC_START_ADDR &_binary_PcmROM_bin_start
	#endif

#else
	#define SEC_START_ADDR 0
#endif

#define _GROUP_TABLE                   0       	//Group Index:0           Group Name:Group_Table     Group Type:Unknown 
#define _GROUP_TABLE_ADDR              (SEC_START_ADDR + 0x000010)	//Group Addr :0x000010    Group Name:Group_Table     Group Type:Unknown 
#define _XMAS1                         0       	//File  Index:0           File  Name:xmas1.pcm 
#define _XMAS1_ADDR                    (SEC_START_ADDR + 0x000010)	//File  Addr :0x000010    File  Name:xmas1.pcm 
#define _OURTOWN                       1       	//File  Index:1           File  Name:ourtown.pcm 
#define _OURTOWN_ADDR                  (SEC_START_ADDR + 0x040d72)	//File  Addr :0x040d72    File  Name:ourtown.pcm 
#define _AINT                          2       	//File  Index:2           File  Name:aint.pcm 
#define _AINT_ADDR                     (SEC_START_ADDR + 0x092988)	//File  Addr :0x092988    File  Name:aint.pcm 

#endif /* __FILE_MERGER_H__ */

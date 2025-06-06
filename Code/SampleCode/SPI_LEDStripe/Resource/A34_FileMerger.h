#ifndef __FILE_MERGER_H__
#define __FILE_MERGER_H__

/* This file is automatically generated by G+ FileMerger.
   Do not modify the contents to prevent from invalid addressing.
*/ 

#if defined (__GNUC__) || defined (__CC_ARM)	/* 32 bit ARM compiler */

	#include <stdint.h>
	#if defined (__CSRC_USED__)					/* use C source instead of elf obj file */
		extern const uint8_t* A34_FileMerger_bin_Data;
		#define SEC_START_ADDR A34_FileMerger_bin_Data
	#else
		extern uint8_t _binary_A34_FileMerger_bin_start;
		extern uint32_t _binary_A34_FileMerger_bin_size;

		#define SEC_START_ADDR &_binary_A34_FileMerger_bin_start
	#endif

#else
	#define SEC_START_ADDR 0
#endif

#define _GROUP_TABLE                   0       	//Group Index:0           Group Name:Group_Table     Group Type:Unknown 
#define _GROUP_TABLE_ADDR              (SEC_START_ADDR + 0x000030)	//Group Addr :0x000030    Group Name:Group_Table     Group Type:Unknown 
#define _HENQUEEN2_10K_34K             0       	//File  Index:0           File  Name:henqueen2_10K_34K.sp4 
#define _HENQUEEN2_10K_34K_ADDR        (SEC_START_ADDR + 0x000030)	//File  Addr :0x000030    File  Name:henqueen2_10K_34K.sp4 
#define _HEMELE3_10K_34K               1       	//File  Index:1           File  Name:hemele3_10K_34K.sp4 
#define _HEMELE3_10K_34K_ADDR          (SEC_START_ADDR + 0x018b10)	//File  Addr :0x018b10    File  Name:hemele3_10K_34K.sp4 
#define _2018G_10K_34K                 2       	//File  Index:2           File  Name:2018g_10K_34K.sp4 
#define _2018G_10K_34K_ADDR            (SEC_START_ADDR + 0x02c684)	//File  Addr :0x02c684    File  Name:2018g_10K_34K.sp4 

#endif /* __FILE_MERGER_H__ */

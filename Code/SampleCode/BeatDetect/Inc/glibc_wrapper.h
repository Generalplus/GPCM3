/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:     
 *   glibc_wrapper.h       
 * @Version: 
 *   V1.0.0
 * @Date: 
 *   December 30, 2019
 * @Abstract: 
 *
 **************************************************************************************************/
#ifndef _GLIBC_WRAPPER_H_
#define _GLIBC_WRAPPER_H_


#ifdef __cplusplus
extern "C"{
#endif


#if defined(__GNUC__)

#ifndef __attribute_copy__
/* Provide an empty definition when cdefs.h is not included.  */
# define __attribute_copy__(arg)
#endif

/* Define ALIASNAME as a strong alias for NAME.  */
# define strong_alias(name, aliasname) _strong_alias(name, aliasname)
# define _strong_alias(name, aliasname) \
  extern __typeof (name) aliasname __attribute__ ((alias (#name))) \
    __attribute_copy__ (name);

#include <string.h>

/* Set memory like memset, but different argument order and no return
   value required.  Also only integer caller-saves may be used.  */
inline __attribute__((always_inline)) void
__aeabi_memclr (void *dest, size_t n)
{
  memset (dest, 0, n);
}

/* Copy memory like memmove, but no return value required.  Can't
   alias to memmove because it's not defined in the same translation
   unit.  */
inline __attribute__((always_inline)) void
__aeabi_memmove (void *dest, const void *src, size_t n)
{
  memmove (dest, src, n);
}

/* Versions of the above which may assume memory alignment.  */
strong_alias (__aeabi_memclr, __aeabi_memclr4)
strong_alias (__aeabi_memclr, __aeabi_memclr8)

strong_alias (__aeabi_memmove, __aeabi_memmove4)
strong_alias (__aeabi_memmove, __aeabi_memmove8)


#endif
	
#ifdef __cplusplus
}
#endif

#endif

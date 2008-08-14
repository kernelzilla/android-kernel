/*
 * arch/arm/plat-omap/include/bridge/csl.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *  ======== csl.h ========
 *  Purpose:
 *      Platform independent C Standard library functions.
 *
 *  Public Functions:
 *      CSL_AnsiToWchar
 *      CSL_Atoi
 *      CSL_ByteSwap
 *      CSL_Exit
 *      CSL_Init
 *      CSL_NumToAscii
 *      CSL_Strcmp
 *      CSL_Strcpyn
 *      CSL_Strlen
 *      CSL_Strncat
 *      CSL_Strncmp
 *      CSL_Strtok
 *      CSL_Strtokr
 *      CSL_WcharToAnsi
 *      CSL_Wstrlen
 *
 *! Revision History:
 *! ================
 *! 07-Aug-2002 jeh: Added CSL_Strtokr().
 *! 21-Sep-2001 jeh: Added CSL_Strncmp.
 *! 22-Nov-2000 map: Added CSL_Atoi and CSL_Strtok
 *! 19-Nov-2000 kc:  Added CSL_ByteSwap().
 *! 09-Nov-2000 kc:  Added CSL_Strncat.
 *! 29-Oct-1999 kc:  Added CSL_Wstrlen().
 *! 20-Sep-1999 ag:  Added CSL_Wchar2Ansi().
 *! 19-Jan-1998 cr:  Code review cleanup (mostly documentation fixes).
 *! 29-Dec-1997 cr:  Changed CSL_lowercase to CSL_Uppercase, added
 *!                  CSL_AnsiToWchar.
 *! 30-Sep-1997 cr:  Added explicit cdecl descriptors to fxn definitions.
 *! 25-Jun-1997 cr:  Added CSL_strcmp.
 *! 12-Jun-1996 gp:  Created.
 */

#ifndef CSL_
#define CSL_

#include <host_os.h>
/*
 *  ======== CSL_Atoi ========
 *  Purpose:
 *      Convert a 1 or 2 digit string number into an integer
 *  Parameters:
 *      ptstrSrc:   pointer to string.
 *  Returns:
 *      Integer
 *  Requires:
 *      CSL initialized.
 *      ptstrSrc is a valid string pointer.
 *  Ensures:
 */
	extern s32 CSL_Atoi(IN CONST char *ptstrSrc);

/*
 *  ======== CSL_Exit ========
 *  Purpose:
 *      Discontinue usage of the CSL module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      CSL initialized.
 *  Ensures:
 *      Resources acquired in CSL_Init() are freed.
 */
	extern void CSL_Exit();

/*
 *  ======== CSL_Init ========
 *  Purpose:
 *      Initialize the CSL module's private state.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      A requirement for each of the other public CSL functions.
 */
	extern bool CSL_Init();

/*
 *  ======== CSL_NumToAscii ========
 *  Purpose:
 *      Convert a 1 or 2 digit number to a 2 digit string.
 *  Parameters:
 *      pstrNumber: Buffer to store converted string.
 *      dwNum:      Number to convert.
 *  Returns:
 *  Requires:
 *      pstrNumber must be able to hold at least three characters.
 *  Ensures:
 *      pstrNumber will be null terminated.
 */
	extern void CSL_NumToAscii(OUT char *pstrNumber, IN u32 dwNum);

/*
 *  ======== CSL_Strcmp ========
 *  Purpose:
 *      Compare 2 ASCII strings.  Works the same way as stdio's strcmp.
 *  Parameters:
 *      pstrStr1:   char * 1.
 *      pstrStr2:   char * 2.
 *  Returns:
 *      A signed value that gives the results of the comparison:
 *      Zero:   String1 equals String2.
 *      < Zero: String1 is less than String2.
 *      > Zero: String1 is greater than String2.
 *  Requires:
 *      CSL initialized.
 *      pstrStr1 is valid.
 *      pstrStr2 is valid.
 *  Ensures:
 */
	extern s32 CSL_Strcmp(IN CONST char *pstrStr1, IN CONST char *pstrStr2);

/*
 *  ======== CSL_Strcpyn ========
 *  Purpose:
 *      Safe strcpy function.
 *  Parameters:
 *      pstrDest:   Ptr to destination buffer.
 *      pstrSrc:    Ptr to source buffer.
 *      cMax:       Size of destination buffer.
 *  Returns:
 *      Ptr to destination buffer; or NULL if error.
 *  Requires:
 *      CSL initialized.
 *      pstrDest is valid.
 *      pstrSrc is valid.
 *  Ensures:
 *      Will not copy more than cMax bytes from pstrSrc into pstrDest.
 *      pstrDest will be terminated by a NULL character.
 */
	extern char *CSL_Strcpyn(OUT char *pstrDest, IN CONST char *pstrSrc,
				IN u32 cMax);

/*
 *  ======== CSL_Strstr ========
 *  Purpose:
 *      Find substring in a stringn.
 *  Parameters:
 *      haystack:   Ptr to string1.
 *      needle:    Ptr to substring to catch.
 *  Returns:
 *      Ptr to first char matching the substring in the main string.
 *  Requires:
 *      CSL initialized.
 *      haystack is valid.
 *      needle is valid.
 *  Ensures:
 */
	extern char *CSL_Strstr(IN CONST char *haystack, IN CONST char *needle);

/*
 *  ======== CSL_Strlen ========
 *  Purpose:
 *      Determine the length of a null terminated ASCI string.
 *  Parameters:
 *      pstrSrc:    pointer to string.
 *  Returns:
 *      String length in bytes.
 *  Requires:
 *      CSL initialized.
 *      pstrSrc is a valid string pointer.
 *  Ensures:
 */
	extern u32 CSL_Strlen(IN CONST char *pstrSrc);

/*
 *  ======== CSL_Strncat ========
 *  Purpose:
 *      Concatenate two strings together.
 *  Parameters:
 *      pszDest:    Destination string.
 *      pszSrc:     Source string.
 *      dwSize:     Number of characters to copy.
 *  Returns:
 *      Pointer to destination string.
 *  Requires:
 *      CSL initialized.
 *      pszDest and pszSrc are valid pointers.
 *  Ensures:
 */
	extern char *CSL_Strncat(IN char *pszDest,
				IN char *pszSrc, IN u32 dwSize);

/*
 *  ======== CSL_Strncmp ========
 *  Purpose:
 *      Compare at most n characters of two ASCII strings.  Works the same
 *      way as stdio's strncmp.
 *  Parameters:
 *      pstrStr1:   char * 1.
 *      pstrStr2:   char * 2.
 *      n:          Number of characters to compare.
 *  Returns:
 *      A signed value that gives the results of the comparison:
 *      Zero:   String1 equals String2.
 *      < Zero: String1 is less than String2.
 *      > Zero: String1 is greater than String2.
 *  Requires:
 *      CSL initialized.
 *      pstrStr1 is valid.
 *      pstrStr2 is valid.
 *  Ensures:
 */
	extern s32 CSL_Strncmp(IN CONST char *pstrStr1,
				IN CONST char *pstrStr2, IN u32 n);

/*
 *  ======== CSL_Strtok ========
 *  Purpose:
 *      Tokenize a NULL terminated string
 *  Parameters:
 *      ptstrSrc:       pointer to string.
 *      szSeparators:   pointer to a string of seperators
 *  Returns:
 *      char *
 *  Requires:
 *      CSL initialized.
 *      ptstrSrc is a valid string pointer.
 *      szSeparators is a valid string pointer.
 *  Ensures:
 */
	extern char *CSL_Strtok(IN char *ptstrSrc,
				IN CONST char *szSeparators);

/*
 *  ======== CSL_Strtokr ========
 *  Purpose:
 *      Re-entrant version of strtok.
 *  Parameters:
 *      pstrSrc:        Pointer to string. May be NULL on subsequent calls.
 *      szSeparators:   Pointer to a string of seperators
 *      ppstrCur:       Location to store start of string for next call to
 *                      to CSL_Strtokr.
 *  Returns:
 *      char * (the token)
 *  Requires:
 *      CSL initialized.
 *      szSeparators != NULL
 *      ppstrCur != NULL
 *  Ensures:
 */
	extern char *CSL_Strtokr(IN char *pstrSrc,
				 IN CONST char *szSeparators,
				 OUT char **ppstrCur);

#endif				/* CSL_ */

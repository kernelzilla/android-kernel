/*
 * arch/arm/plat-omap/include/bridge/dbldefs.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
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
 *  ======== dbldefs.h ========
 *
 *! Revision History
 *! ================
 *! 19-Mar-2002 jeh     Added DBL_Fxns type (to make it easier to switch
 *!                     between different loaders).
 *! 28-Sep-2001 jeh     Created from zl.h.
 */
#ifndef DBLDEFS_
#define DBLDEFS_

/*
 *  Bit masks for DBL_Flags.
 */
#define DBL_NOLOAD   0x0	/* Don't load symbols, code, or data */
#define DBL_SYMB     0x1	/* load symbols */
#define DBL_CODE     0x2	/* load code */
#define DBL_DATA     0x4	/* load data */
#define DBL_DYNAMIC  0x8	/* dynamic load */
#define DBL_BSS      0x20	/* Unitialized section */

#define DBL_MAXPATHLENGTH       255

#ifndef _SIZE_T			/* Linux sets _SIZE_T on defining size_t */
typedef unsigned int size_t;
#define _SIZE_T
#endif

/*
 *  ======== DBL_Flags ========
 *  Specifies whether to load code, data, or symbols
 */
typedef s32 DBL_Flags;

/*
 *  ======== DBL_SectInfo ========
 *  For collecting info on overlay sections
 */
struct DBL_SectInfo {
	const char *name;	/* name of section */
	u32 runAddr;		/* run address of section */
	u32 loadAddr;		/* load address of section */
	u32 size;		/* size of section (target MAUs) */
	DBL_Flags type;		/* Code, data, or BSS */
} ;

/*
 *  ======== DBL_Symbol ========
 *  (Needed for dynamic load library)
 */
struct DBL_Symbol {
	u32 value;
};

/*
 *  ======== DBL_AllocFxn ========
 *  Allocate memory function.  Allocate or reserve (if reserved == TRUE)
 *  "size" bytes of memory from segment "space" and return the address in
 *  *dspAddr (or starting at *dspAddr if reserve == TRUE). Returns 0 on
 *  success, or an error code on failure.
 */
typedef s32(*DBL_AllocFxn) (void *hdl, s32 space, u32 size, u32 align,
			u32 *dspAddr, s32 segId, s32 req, bool reserved);

/*
 *  ======== DBL_CinitFxn ========
 *  Process .cinit records.
 *  Parameters:
 *      hdl             - Opaque handle
 *      dspAddress      - DSP address of .cinit section
 *      buf             - Buffer containing .cinit section
 *      nBytes          - Size of .cinit section (host bytes)
 *      mtype           - Page? (does not need to be used)
 *
 *  Returns:
 *      nBytes          - Success
 *      < nBytes        - Failure
 *
 *  Note: Cinit processing can either be done by the DSP, in which case
 *  the .cinit section must have already been written, or on the host,
 *  in which case we need to use the data in buf.
 */
typedef s32(*DBL_CinitFxn) (void *hdl, u32 dspAddr, void *buf,
			    u32 nBytes, s32 mtype);

/*
 *  ======== DBL_CloseFxn ========
 */
typedef s32(*DBL_FCloseFxn) (void *);

/*
 *  ======== DBL_FreeFxn ========
 *  Free memory function.  Free, or unreserve (if reserved == TRUE) "size"
 *  bytes of memory from segment "space"
 */
typedef bool(*DBL_FreeFxn) (void *hdl, u32 addr, s32 space, u32 size,
			    bool reserved);

/*
 *  ======== DBL_FOpenFxn ========
 */
typedef void *(*DBL_FOpenFxn) (const char *, const char *);

/*
 *  ======== DBL_LogWriteFxn ========
 *  Function to call when writing data from a section, to log the info.
 *  Can be NULL if no logging is required.
 */
typedef DSP_STATUS(*DBL_LogWriteFxn) (void *handle, struct DBL_SectInfo *sect,
				      u32 addr, u32 nBytes);

/*
 *  ======== DBL_ReadFxn ========
 */
typedef s32(*DBL_ReadFxn) (void *, size_t, size_t, void *);

/*
 *  ======== DBL_SeekFxn ========
 */
typedef s32(*DBL_SeekFxn) (void *, long, int);

/*
 *  ======== DBL_SymLookup ========
 *  Symbol lookup function - Find the symbol name and return its value.
 *
 *  Parameters:
 *      handle          - Opaque handle
 *      pArg            - Opaque argument.
 *      name            - Name of symbol to lookup.
 *      sym             - Location to store address of symbol structure.
 *
 *  Returns:
 *      TRUE:           Success (symbol was found).
 *      FALSE:          Failed to find symbol.
 */
typedef bool(*DBL_SymLookup) (void *handle, void *pArg, void *rmmHandle,
			      const char *name, struct DBL_Symbol **sym);

/*
 *  ======== DBL_TellFxn ========
 */
typedef s32(*DBL_TellFxn) (void *);

/*
 *  ======== DBL_WriteFxn ========
 *  Write memory function.  Write "n" HOST bytes of memory to segment "mtype"
 *  starting at address "dspAddr" from the buffer "buf".  The buffer is
 *  formatted as an array of words appropriate for the DSP.
 */
typedef s32(*DBL_WriteFxn) (void *hdl, u32 dspAddr, void *buf,
			    u32 n, s32 mtype);

/*
 *  ======== DBL_Attrs ========
 */
struct DBL_Attrs {
	DBL_AllocFxn alloc;
	DBL_FreeFxn free;
	void *rmmHandle;	/* Handle to pass to alloc, free functions */
	DBL_WriteFxn write;
	void *wHandle;		/* Handle to pass to write, cinit function */

	DBL_LogWriteFxn logWrite;
	void *logWriteHandle;

	/* Symbol matching function and handle to pass to it */
	DBL_SymLookup symLookup;
	void *symHandle;
	void *symArg;

	/*
	 *  These file manipulation functions should be compatible with the
	 *  "C" run time library functions of the same name.
	 */
	s32(*fread) (void *, size_t, size_t, void *);
	s32(*fseek) (void *, long, int);
	s32(*ftell) (void *);
	s32(*fclose) (void *);
	void *(*fopen) (const char *, const char *);
} ;

/*
 *  ======== DBL_close ========
 *  Close library opened with DBL_open.
 *  Parameters:
 *      lib             - Handle returned from DBL_open().
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *  Ensures:
 */
typedef void(*DBL_CloseFxn) (struct DBL_LibraryObj *library);

/*
 *  ======== DBL_create ========
 *  Create a target object, specifying the alloc, free, and write functions.
 *  Parameters:
 *      pTarget         - Location to store target handle on output.
 *      pAttrs          - Attributes.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EMEMORY:    Memory allocation failed.
 *  Requires:
 *      DBL initialized.
 *      pAttrs != NULL.
 *      pTarget != NULL;
 *  Ensures:
 *      Success:        *pTarget != NULL.
 *      Failure:        *pTarget == NULL.
 */
typedef DSP_STATUS(*DBL_CreateFxn) (struct DBL_TargetObj **pTarget,
				    struct DBL_Attrs *attrs);

/*
 *  ======== DBL_delete ========
 *  Delete target object and free resources for any loaded libraries.
 *  Parameters:
 *      target          - Handle returned from DBL_Create().
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *  Ensures:
 */
typedef void(*DBL_DeleteFxn) (struct DBL_TargetObj *target);

/*
 *  ======== DBL_exit ========
 *  Discontinue use of DBL module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      cRefs > 0.
 *  Ensures:
 *      cRefs >= 0.
 */
typedef void(*DBL_ExitFxn) (void);

/*
 *  ======== DBL_getAddr ========
 *  Get address of name in the specified library.
 *  Parameters:
 *      lib             - Handle returned from DBL_open().
 *      name            - Name of symbol
 *      ppSym           - Location to store symbol address on output.
 *  Returns:
 *      TRUE:           Success.
 *      FALSE:          Symbol not found.
 *  Requires:
 *      DBL initialized.
 *      Valid library.
 *      name != NULL.
 *      ppSym != NULL.
 *  Ensures:
 */
typedef bool(*DBL_GetAddrFxn) (struct DBL_LibraryObj *lib, char *name,
			       struct DBL_Symbol **ppSym);

/*
 *  ======== DBL_getAttrs ========
 *  Retrieve the attributes of the target.
 *  Parameters:
 *      target          - Handle returned from DBL_Create().
 *      pAttrs          - Location to store attributes on output.
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *      pAttrs != NULL.
 *  Ensures:
 */
typedef void(*DBL_GetAttrsFxn) (struct DBL_TargetObj *target,
				struct DBL_Attrs *attrs);

/*
 *  ======== DBL_getCAddr ========
 *  Get address of "C" name on the specified library.
 *  Parameters:
 *      lib             - Handle returned from DBL_open().
 *      name            - Name of symbol
 *      ppSym           - Location to store symbol address on output.
 *  Returns:
 *      TRUE:           Success.
 *      FALSE:          Symbol not found.
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *      name != NULL.
 *      ppSym != NULL.
 *  Ensures:
 */
typedef bool(*DBL_GetCAddrFxn) (struct DBL_LibraryObj *lib, char *name,
				struct DBL_Symbol **ppSym);

/*
 *  ======== DBL_getSect ========
 *  Get address and size of a named section.
 *  Parameters:
 *      lib             - Library handle returned from DBL_open().
 *      name            - Name of section.
 *      pAddr           - Location to store section address on output.
 *      pSize           - Location to store section size on output.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ENOSECT:    Section not found.
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *      name != NULL.
 *      pAddr != NULL;
 *      pSize != NULL.
 *  Ensures:
 */
typedef DSP_STATUS(*DBL_GetSectFxn) (struct DBL_LibraryObj *lib, char *name,
				     u32 *addr, u32 *size);

/*
 *  ======== DBL_init ========
 *  Initialize DBL module.
 *  Parameters:
 *  Returns:
 *      TRUE:           Success.
 *      FALSE:          Failure.
 *  Requires:
 *      cRefs >= 0.
 *  Ensures:
 *      Success:        cRefs > 0.
 *      Failure:        cRefs >= 0.
 */
typedef bool(*DBL_InitFxn) (void);

/*
 *  ======== DBL_load ========
 *  Load library onto the target.
 *
 *  Parameters:
 *      lib             - Library handle returned from DBL_open().
 *      flags           - Load code, data and/or symbols.
 *      attrs           - May contain alloc, free, and write function.
 *      pulEntry        - Location to store program entry on output.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFREAD:     File read failed.
 *      DSP_EFWRITE:    Write to target failed.
 *      DSP_EDYNLOAD:   Failure in dynamic loader library.
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *      pEntry != NULL.
 *  Ensures:
 */
typedef DSP_STATUS(*DBL_LoadFxn) (struct DBL_LibraryObj *lib, DBL_Flags flags,
				  struct DBL_Attrs *attrs, u32 *entry);

/*
 *  ======== DBL_loadSect ========
 *  Load a named section from an library (for overlay support).
 *  Parameters:
 *      lib             - Handle returned from DBL_open().
 *      sectName        - Name of section to load.
 *      attrs           - Contains write function and handle to pass to it.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ENOSECT:    Section not found.
 *      DSP_EFWRITE:    Write function failed.
 *      DSP_ENOTIMPL:   Function not implemented.
 *  Requires:
 *      Valid lib.
 *      sectName != NULL.
 *      attrs != NULL.
 *      attrs->write != NULL.
 *  Ensures:
 */
typedef DSP_STATUS(*DBL_LoadSectFxn) (struct DBL_LibraryObj *lib,
				      char *pszSectName,
				      struct DBL_Attrs *attrs);

/*
 *  ======== DBL_open ========
 *  DBL_open() returns a library handle that can be used to load/unload
 *  the symbols/code/data via DBL_load()/DBL_unload().
 *  Parameters:
 *      target          - Handle returned from DBL_create().
 *      file            - Name of file to open.
 *      flags           - If flags & DBL_SYMB, load symbols.
 *      pLib            - Location to store library handle on output.
 *  Returns:
 *      DSP_SOK:            Success.
 *      DSP_EMEMORY:        Memory allocation failure.
 *      DSP_EFOPEN:         File open failure.
 *      DSP_EFREAD:         File read failure.
 *      DSP_ECORRUPTFILE:   Unable to determine target type.
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *      file != NULL.
 *      pLib != NULL.
 *      DBL_Attrs fopen function non-NULL.
 *  Ensures:
 *      Success:        Valid *pLib.
 *      Failure:        *pLib == NULL.
 */
typedef DSP_STATUS(*DBL_OpenFxn) (struct DBL_TargetObj *target, char *file,
				  DBL_Flags flags,
				  struct DBL_LibraryObj **pLib);

/*
 *  ======== DBL_readSect ========
 *  Read COFF section into a character buffer.
 *  Parameters:
 *      lib             - Library handle returned from DBL_open().
 *      name            - Name of section.
 *      pBuf            - Buffer to write section contents into.
 *      size            - Buffer size
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ENOSECT:    Named section does not exists.
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *      name != NULL.
 *      pBuf != NULL.
 *      size != 0.
 *  Ensures:
 */
typedef DSP_STATUS(*DBL_ReadSectFxn) (struct DBL_LibraryObj *lib, char *name,
				      char *content, u32 uContentSize);

/*
 *  ======== DBL_setAttrs ========
 *  Set the attributes of the target.
 *  Parameters:
 *      target          - Handle returned from DBL_create().
 *      pAttrs          - New attributes.
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid target.
 *      pAttrs != NULL.
 *  Ensures:
 */
typedef void(*DBL_SetAttrsFxn) (struct DBL_TargetObj *target,
				struct DBL_Attrs *attrs);

/*
 *  ======== DBL_unload ========
 *  Unload library loaded with DBL_load().
 *  Parameters:
 *      lib             - Handle returned from DBL_open().
 *      attrs           - Contains free() function and handle to pass to it.
 *  Returns:
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *  Ensures:
 */
typedef void(*DBL_UnloadFxn) (struct DBL_LibraryObj *library,
			      struct DBL_Attrs *attrs);

/*
 *  ======== DBL_unloadSect ========
 *  Unload a named section from an library (for overlay support).
 *  Parameters:
 *      lib             - Handle returned from DBL_open().
 *      sectName        - Name of section to load.
 *      attrs           - Contains free() function and handle to pass to it.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_ENOSECT:    Named section not found.
 *      DSP_ENOTIMPL
 *  Requires:
 *      DBL initialized.
 *      Valid lib.
 *      sectName != NULL.
 *  Ensures:
 */
typedef DSP_STATUS(*DBL_UnloadSectFxn) (struct DBL_LibraryObj *lib,
					char *pszSectName,
					struct DBL_Attrs *attrs);

struct DBL_Fxns {
	DBL_CloseFxn closeFxn;
	DBL_CreateFxn createFxn;
	DBL_DeleteFxn deleteFxn;
	DBL_ExitFxn exitFxn;
	DBL_GetAttrsFxn getAttrsFxn;
	DBL_GetAddrFxn getAddrFxn;
	DBL_GetCAddrFxn getCAddrFxn;
	DBL_GetSectFxn getSectFxn;
	DBL_InitFxn initFxn;
	DBL_LoadFxn loadFxn;
	DBL_LoadSectFxn loadSectFxn;
	DBL_OpenFxn openFxn;
	DBL_ReadSectFxn readSectFxn;
	DBL_SetAttrsFxn setAttrsFxn;
	DBL_UnloadFxn unloadFxn;
	DBL_UnloadSectFxn unloadSectFxn;
};

#endif				/* DBLDEFS_ */

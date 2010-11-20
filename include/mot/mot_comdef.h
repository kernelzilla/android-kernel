#ifndef COMDEF_H
#define COMDEF_H

/*===========================================================================

                   S T A N D A R D    D E C L A R A T I O N S


==============================================================================*/

/*=============================================================================

Motorola Revision History:
         Modification     Tracking
Author       Date          Number     Description of Changes
------   ------------    ----------   -----------------------------------------
w20142    04/23/2009     LIBss30959   Port to Android
w36210    02/20/2009     LIBss07115   Added support for flashing Android partitions.
w36210    02/16/2009     LIBss02247   Modified for Calgary Android.

==============================================================================*/

/* ------------------------------------------------------------------------
** Constants
** ------------------------------------------------------------------------ */

#ifdef TRUE
#undef TRUE
#endif

#ifdef FALSE
#undef FALSE
#endif

#define TRUE   1   /* Boolean true value. */
#define FALSE  0   /* Boolean false value. */

#define  ON   1    /* On value. */
#define  OFF  0    /* Off value. */

#ifndef NULL
  #define NULL  0
#endif


/* -----------------------------------------------------------------------
** Standard Types
** ----------------------------------------------------------------------- */

/* The following definitions are the same accross platforms.  This first
** group are the sanctioned types.
*/

typedef  unsigned char      boolean;     /* Boolean value type. */

#ifdef __linux__
typedef  uint32_t           uint32;      /* Unsigned 32 bit value */
#else
typedef  unsigned long int  uint32;      /* Unsigned 32 bit value */
#endif
typedef  unsigned short     uint16;      /* Unsigned 16 bit value */
typedef  unsigned char      uint8;       /* Unsigned 8  bit value */

#ifdef __linux__
typedef  int32_t            int32;       /* Signed 32 bit value */
#else
typedef  signed long int    int32;       /* Signed 32 bit value */
#endif

typedef  signed short       int16;       /* Signed 16 bit value */
typedef  signed char        int8;        /* Signed 8  bit value */

/* This group are the deprecated types.  Their use should be
** discontinued and new code should use the types above
*/
typedef  unsigned char     byte;         /* Unsigned 8  bit value type. */
typedef  unsigned short    word;         /* Unsinged 16 bit value type. */
typedef  unsigned long     dword;        /* Unsigned 32 bit value type. */

typedef  unsigned char     uint1;        /* Unsigned 8  bit value type. */
typedef  unsigned short    uint2;        /* Unsigned 16 bit value type. */
typedef  unsigned long     uint4;        /* Unsigned 32 bit value type. */

typedef  signed char       int1;         /* Signed 8  bit value type. */
typedef  signed short      int2;         /* Signed 16 bit value type. */
typedef  long int          int4;         /* Signed 32 bit value type. */

typedef  signed long       sint31;       /* Signed 32 bit value */
typedef  signed short      sint15;       /* Signed 16 bit value */
typedef  signed char       sint7;        /* Signed 8  bit value */

   
typedef  boolean            BOOLEAN;     /* Boolean value type. */
typedef  uint32             UINT32;      /* Unsigned 32 bit value */
typedef  uint16             UINT16;      /* Unsigned 16 bit value */
typedef  uint8              UINT8;       /* Unsigned 8  bit value */
typedef  int32              INT32;       /* Signed 32 bit value */
typedef  int16              INT16;       /* Signed 16 bit value */
typedef  int8               INT8;        /* Signed 8  bit value */

#endif  /* COMDEF_H */

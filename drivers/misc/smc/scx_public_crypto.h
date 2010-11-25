/*
 *Copyright (c)2006-2008 Trusted Logic S.A.
 *All Rights Reserved.
 *
 *This program is free software; you can redistribute it and/or
 *modify it under the terms of the GNU General Public License
 *version 2 as published by the Free Software Foundation.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *You should have received a copy of the GNU General Public License
 *along with this program; if not, write to the Free Software
 *Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 *MA 02111-1307 USA
 */

#ifndef __SCX_PUBLIC_CRYPTO_H
#define __SCX_PUBLIC_CRYPTO_H

#include "scxlnx_defs.h"
#if defined (KERNEL_2_6_27) || defined (KERNEL_2_6_29)
#include <mach/io.h>
#else
#include <asm-arm/arch-omap/io.h>
#endif

typedef unsigned long	U32;
typedef unsigned short  U16;
typedef unsigned char	U8;

typedef volatile unsigned long	VU32;
typedef volatile unsigned short  VU16;
typedef volatile unsigned char	VU8;

#define OUTREG32(a, b)(*(volatile unsigned long *)(a) = (unsigned long)b)
#define INREG32(a)(*(volatile unsigned long *)(a))
#define SETREG32(x, y)OUTREG32(x, INREG32(x) | (y))
#define CLRREG32(x, y)OUTREG32(x, INREG32(x) & ~(y))


#define PUBLIC_CRYPTO_DES_CLOCK_BIT		 (1<<26)
#define PUBLIC_CRYPTO_SHA_CLOCK_BIT		 (1<<1)
#define PUBLIC_CRYPTO_AES_CLOCK_BIT		 (1<<3)

#define BYTES_TO_LONG(a)(U32)(a[0] | (a[1]<<8) | (a[2]<<16) | (a[3]<<24))
#define LONG_TO_BYTE(a, b) {  a[0] = (U8)((b) & 0xFF);		  \
															a[1] = (U8)(((b) >> 8) & 0xFF);	\
															a[2] = (U8)(((b) >> 16) & 0xFF);  \
															a[3] = (U8)(((b) >> 24) & 0xFF); }

#define IS_4_BYTES_ALIGNED(x)((!((x) & 0x3)) ? 1 : 0)


/*Error code constants */
#define PUBLIC_CRYPTO_OPERATION_SUCCESS	0x00000000
#define PUBLIC_CRYPTO_ERR_ACCESS_DENIED	0x00000001
#define PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY	0x00000002
#define PUBLIC_CRYPTO_ERR_BAD_PARAMETERS  0x00000003
#define PUBLIC_CRYPTO_ERR_TIMEOUT			0x00000004
#define PUBLIC_CRYPTO_ERR_ALIGNMENT		 0x00000005

/*Des mode constants */
#define PUBLIC_CRYPTO_DES_MODE				0x00000000
#define PUBLIC_CRYPTO_DES3_MODE			  0x00000001

/*Encryption / Decryption mode constants */
#define PUBLIC_CRYPTO_ECB					  0x10100000
#define PUBLIC_CRYPTO_CBC					  0x10200000
#define PUBLIC_CRYPTO_CTR					  0x10500000

/*Direction constants */
#define PUBLIC_CRYPTO_ENCRYPTION			 0x00000003
#define PUBLIC_CRYPTO_DECRYPTION			 0x00000004

/*DMA mode constants */
#define PUBLIC_CRYPTO_DMA_USE_NONE		  0x00000000	  /*No DMA used						 */
#define PUBLIC_CRYPTO_DMA_USE_POLLING	  0x00000001	  /*DMA with active polling used */
#define PUBLIC_CRYPTO_DMA_USE_IRQ			0x00000002	  /*DMA with IRQ used				 */


#define PUBLIC_CRYPTO_REG_SET_BIT(x, y)OUTREG32(x, INREG32(x) | y);
#define PUBLIC_CRYPTO_REG_UNSET_BIT(x, y)OUTREG32(x, INREG32(x) & (~y));


#define AES_BLOCK_SIZE					  16
#define DES_BLOCK_SIZE					  8

#define HASH_MD5_LENGTH					 16
#define HASH_SHA1_LENGTH					20
#define HASH_SHA224_LENGTH				 28
#define HASH_SHA256_LENGTH				 32

#define PUBLIC_CRYPTO_DIGEST_MAX_SIZE	  32
#define PUBLIC_CRYPTO_IV_MAX_SIZE			16

#define PUBLIC_CRYPTO_ALG_CIPHER_AES		0x10000001
#define PUBLIC_CRYPTO_ALG_CIPHER_DES		0x10000002
#define PUBLIC_CRYPTO_ALG_CIPHER_3DES	  0x10000004

#define PUBLIC_CRYPTO_ALG_DIGEST_SHA1	  0x00000000
#define PUBLIC_CRYPTO_ALG_DIGEST_MD5		0x00000001
#define PUBLIC_CRYPTO_ALG_DIGEST_SHA224	0x00000002
#define PUBLIC_CRYPTO_ALG_DIGEST_SHA256	0x00000003



/*---------------------------------------------------------------------------- */
/*									 AES Context												 */
/*---------------------------------------------------------------------------- */

/* *
 *This structure contains the registers of the AES HW accelerator.
 */
typedef struct {
	VU32 AES_KEY4_L;	  /*0x00 */
	VU32 AES_KEY4_H;	  /*0x04 */
	VU32 AES_KEY3_L;	  /*0x08 */
	VU32 AES_KEY3_H;	  /*0x0C */
	VU32 AES_KEY2_L;	  /*0x10 */
	VU32 AES_KEY2_H;	  /*0x14 */
	VU32 AES_KEY1_L;	  /*0x18 */
	VU32 AES_KEY1_H;	  /*0x1c */

	VU32 AES_IV_1;		 /*0x20 */
	VU32 AES_IV_2;		 /*0x24 */
	VU32 AES_IV_3;		 /*0x28 */
	VU32 AES_IV_4;		 /*0x2C */

	VU32 AES_CTRL;		 /*0x30 */

	VU32 AES_DATA_1;	  /*0x34 */
	VU32 AES_DATA_2;	  /*0x38 */
	VU32 AES_DATA_3;	  /*0x3C */
	VU32 AES_DATA_4;	  /*0x40 */

	/*
	 *Warning: This field will be internally used for the CTR mode.
	 */
	VU32 AES_REV;		  /*0x44 */
	VU32 AES_MASK;		 /*0x48 */

	VU32 AES_SYSSTATUS;  /*0x4C */

} AESReg_t;


typedef struct {
	AESReg_t registers;
}
PUBLIC_CRYPTO_AES_CONTEXT;


/*---------------------------------------------------------------------------- */
/*								DES/DES3 Context												 */
/*---------------------------------------------------------------------------- */

/* *
 *This structure contains the registers of the DES HW accelerator.
 */
typedef struct {
  VU32 DES_KEY3_L;	  /*DES Key 3 Low Register						 */
  VU32 DES_KEY3_H;	  /*DES Key 3 High Register						 */
  VU32 DES_KEY2_L;	  /*DES Key 2 Low Register						 */
  VU32 DES_KEY2_H;	  /*DES Key 2 High Register						 */
  VU32 DES_KEY1_L;	  /*DES Key 1 Low Register						 */
  VU32 DES_KEY1_H;	  /*DES Key 1 High Register						 */
  VU32 DES_IV_L;		 /*DES Initialization Vector Low Register	 */
  VU32 DES_IV_H;		 /*DES Initialization Vector High Register  */
  VU32 DES_CTRL;		 /*DES Control Register							 */
  VU32 DES_DATA_L;	  /*DES Data Input/Output Low Register		 */
  VU32 DES_DATA_H;	  /*DES Data Input/Output High Register		 */
  VU32 DES_REV;		  /*DES Revision Register						  */
  VU32 DES_MASK;		 /*DES Mask and Reset Register				  */
  VU32 DES_SYSSTATUS;  /*DES System Status Register					 */
} Des3DesReg_t;

typedef struct {
	Des3DesReg_t registers;
}
PUBLIC_CRYPTO_DES_DES3_CONTEXT;


/*---------------------------------------------------------------------------- */
/*								 Digest Context												 */
/*---------------------------------------------------------------------------- */

#define HASH_BLOCK_BYTES_LENGTH		  64
#define HASH_CHUNK_BYTES_LENGTH		  64


/* *
 *This structure contains the registers of the SHA1/MD5 HW accelerator.
 */
typedef struct {
	VU32 DIGEST_A;	/*0x00 Digest A		 */
	VU32 DIGEST_B;	/*0x04 Digest B		 */
	VU32 DIGEST_C;	/*0x08 Digest C		 */
	VU32 DIGEST_D;	/*0x0C Digest D		 */
	VU32 DIGEST_E;	/*0x10 Digest E		 */
	VU32 DIGEST_F;	/*0x14 Digest F		 */
	VU32 DIGEST_G;	/*0x18 Digest G		 */
	VU32 DIGEST_H;	/*0x1C Digest H		 */
	VU32 DIGCNT;	  /*0x20 Digest count  */
	VU32 BYTE;		 /*0x24 Byte			 */
	VU32 IRQSTAT;	 /*0x28 Irq status	 */
	VU32 CTRL;		 /*0x2C Control		 */
	VU32 DIN_0;		/*0x30 Data 0		  */
	VU32 DIN_1;		/*0x34 Data 1		  */
	VU32 DIN_2;		/*0x38 Data 2		  */
	VU32 DIN_3;		/*0x3C Data 3		  */
	VU32 DIN_4;		/*0x40 Data 4		  */
	VU32 DIN_5;		/*0x44 Data 5		  */
	VU32 DIN_6;		/*0x48 Data 6		  */
	VU32 DIN_7;		/*0x4C Data 7		  */
	VU32 DIN_8;		/*0x50 Data 8		  */
	VU32 DIN_9;		/*0x54 Data 9		  */
	VU32 DIN_10;	  /*0x58 Data 10		 */
	VU32 DIN_11;	  /*0x5C Data 11		 */
	VU32 DIN_12;	  /*0x60 Data 12		 */
	VU32 DIN_13;	  /*0x64 Data 13		 */
	VU32 DIN_14;	  /*0x68 Data 14		 */
	VU32 DIN_15;	  /*0x6C Data 15		 */
	VU32 REV;		  /*0x70 Revision		 */
	VU32 MASK;		 /*0x74 Mask			 */
	VU32 SYSSTATUS;  /*0x78 System status */
} Sha1Md5Reg_t;

typedef struct {
	Sha1Md5Reg_t registers;
	U8	 pChunckBuffer[HASH_CHUNK_BYTES_LENGTH];
	U32	vChunckIndex;
	U32	vNbBytesProcessed;
}
PUBLIC_CRYPTO_HASH_CONTEXT;


typedef struct {
	 U32 hwa_config;
	 U8  iv[PUBLIC_CRYPTO_IV_MAX_SIZE];
} CryptoInitContext;

typedef enum
{
   CRYPTO_ACCELERATOR_AES1,
   CRYPTO_ACCELERATOR_SHA1,
   CRYPTO_ACCELERATOR_DES2,

   CRYPTO_ACCELERATOR_NB,

} e_CryptoAccelerator;

/*---------------------------------------------------------------------------- */
/*
 *Public crypto API (Top level)
 */

/* *
 *Initialize the public crypto (including DMA)
 */
u32 scxPublicCryptoInit(SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto);

/* *
 *Terminate the public crypto (including DMA)
 */
void scxPublicCryptoTerminate(SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto);

bool scxIsPublicCryptoExist(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		SCX_COMMAND_INVOKE_CLIENT_COMMAND *pCommand);

SCX_PUBLIC_CRYPTO_DESC *scxPublicCryptoCommandMatchAndCheck(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		SCX_COMMAND_INVOKE_CLIENT_COMMAND *pCommand,
		bool *pbIsSessionCorrupted);

int scxPublicCryptoPreProcessCommand(
		SCXLNX_SHMEM_DESC * pDeviceContextDesc,
		SCX_PUBLIC_CRYPTO_DESC *pSession,
		SCX_COMMAND_INVOKE_CLIENT_COMMAND *pCommand,
		SCX_ANSWER_INVOKE_CLIENT_COMMAND *pAnswer);

void scxPublicCryptoProcessCommand(
		SCX_PUBLIC_CRYPTO_DESC *pSession,
		SCX_ANSWER_INVOKE_CLIENT_COMMAND *pAnswer);

void scxPublicCryptoPostProcessCommand(
		SCX_PUBLIC_CRYPTO_DESC *pSession,
		SCX_ANSWER_INVOKE_CLIENT_COMMAND *pAnswer,
		bool bProcessedInPublic,
		uint32_t nPKCS11Error);

u32 scxPublicCryptoRPCInit(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer);

u32 scxPublicCryptoRPCUpdate(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer);

u32 scxPublicCryptoRPCFinal(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer);

u32 scxPublicCryptoRPCTerminate(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer);

u32 scxPublicCryptoRPCEnableClock(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer);

/*---------------------------------------------------------------------------- */
/*
 *Helper methods
 */
U32  scxPublicCryptoWaitForReadyBit(VU32 *pRegister, U32 vBit);
void scxPublicCryptoWaitForReadyBitInfinitely(VU32 *pRegister, U32 vBit);
void scxPublicCryptoEnableHwClock(e_CryptoAccelerator nAccelerator);
void scxPublicCryptoDisableHwClock(e_CryptoAccelerator nAccelerator);
void scxPublicCryptoEnableClock(e_CryptoAccelerator nAccelerator);
void scxPublicCryptoDisableClock(e_CryptoAccelerator nAccelerator);

/*---------------------------------------------------------------------------- */
/*									 AES operations											 */
/*---------------------------------------------------------------------------- */



/* *
 *This function initializes a Public AES Operation.
 *
 *pAesInitCtx Structure containing information (iv, control register)
 *		  related to an AES operation.
 *
 *pAesCtx	  Structure related to a Public AES Operation to be initialized.
 */
void PDrvCryptoInitAES(CryptoInitContext *pAesInitCtx,
							  PUBLIC_CRYPTO_AES_CONTEXT *pAesCtx);

/* *
 *This function performs an AES update operation.
 *
 *pAesCtx  Context of the Public AES Operation.
 *
 *pSrc	  Input buffer to process.
 *
 *pDest	 Output buffer containing the processed data.
 *
 *nbBlocks number of block(s)to process.
 *
 *dmaUse	specifies which mode of DMA to use :
 *					- PUBLIC_CRYPTO_DMA_USE_NONE,
 *					- or PUBLIC_CRYPTO_DMA_USE_POLLING,
 *					- or PUBLIC_CRYPTO_DMA_USE_IRQ.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoUpdateAES(PUBLIC_CRYPTO_AES_CONTEXT *pAesCtx,
								U8 *pSrc,
								U8 *pDest,
								U32 nbBlocks,
								U32 dmaUse);

/* *
 *This function terminates a Public AES Operation.
 *
 *pAesCtx  Context of the Public AES Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoTerminateAES(PUBLIC_CRYPTO_AES_CONTEXT *pAesCtx);

/* *
 *This function saves the registers from the AES HW accelerator into the
 *passed structure.
 *
 *pAesCtx Context of the Public AES Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoSaveAESRegisters(PUBLIC_CRYPTO_AES_CONTEXT *pAesCtx);

/* *
 *This function restores the registers of the AES HW accelerator
 *from the passed structure.
 *NB: The Registers related to the Key are not restored.
 *
 *pAesCtx Context of the Public AES Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoRestoreAESRegisters(PUBLIC_CRYPTO_AES_CONTEXT *pAesCtx);


/*---------------------------------------------------------------------------- */
/*								DES/DES3 operations											 */
/*---------------------------------------------------------------------------- */

/* *
 *This function initializes a Public DES Operation.
 *
 *pDesDes3InitCtx Structure containing information (iv, control register)
 *				related to an DES operation.
 *
 *pDesDes3Ctx	  Structure related to a Public DES Operation to be
 *				initialized.
 */
void PDrvCryptoInitDES(CryptoInitContext *pDesDes3InitCtx,
							  PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx);

/* *
 *This function performs a DES update operation.
 *
 *pDesDes3Ctx Context of the Public DES Operation.
 *
 *pSrc		  Input buffer to process.
 *
 *pDest		 Output buffer containing the processed data.
 *
 *nbBlocks	 Number of block(s)to process.
 *
 *dmaUse	specifies which mode of DMA to use :
 *					- PUBLIC_CRYPTO_DMA_USE_NONE,
 *					- or PUBLIC_CRYPTO_DMA_USE_POLLING,
 *					- or PUBLIC_CRYPTO_DMA_USE_IRQ.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoUpdateDES(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx,
								U8 *pSrc,
								U8 *pDest,
								U32 nbBlocks,
								U32 dmaUse);

/* *
 *This function terminates a Public DES Operation.
 *
 *pDesDes3Ctx  Context of the Public DES Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoTerminateDES(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx);

/* *
 *This function saves the registers from the DES HW accelerator into the
 *passed structure.
 *
 *pDesDes3Ctx Context of the Public DES Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoSaveDESRegisters(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx);

/* *
 *This function restores the registers of the DES HW accelerator
 *from the passed structure.
 *NB: The Registers related to the Key are not restored.
 *
 *pDesDes3Ctx Context of the Public DES Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoRestoreDESRegisters(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx);


/*---------------------------------------------------------------------------- */
/*								 Digest operations											 */
/*---------------------------------------------------------------------------- */

/* *
 *This function initializes a Public HASH Operation.
 *
 *hashAlgorithm algorithm of the HASH operation. Possible values are:
 *						  - PUBLIC_CRYPTO_ALG_DIGEST_MD5
 *						  - PUBLIC_CRYPTO_ALG_DIGEST_SHA1
 *						  - PUBLIC_CRYPTO_ALG_DIGEST_SHA224
 *						  - PUBLIC_CRYPTO_ALG_DIGEST_SHA256
 *
 *pHashCtx Structure related to a Public HASH Operation to be initialized.
 */
void PDrvCryptoInitHash(U32 hashAlgorithm,
								PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx);

/* *
 *This function performs a HASH update Operation.
 *
 *pHashCtx	Context of the Public HASH Operation.
 *
 *pData		Input buffer to process
 *
 *dataLength Length in bytes of the input buffer.
 *
 *dmaUse	  specifies which mode of DMA to use :
 *					- PUBLIC_CRYPTO_DMA_USE_NONE,
 *					- or PUBLIC_CRYPTO_DMA_USE_POLLING,
 *					- or PUBLIC_CRYPTO_DMA_USE_IRQ.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoUpdateHash(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx,
								U8 *pData,
								U32 dataLength,
								U32 dmaUse);

/* *
 *This function performs a HASH Final Operation.
 *
 *pHashCtx Context of the Public HASH Operation.
 *
 *pData	 Buffer to be filled out with the result of the digest operation.
 *
 *dmaUse	specifies which mode of DMA to use :
 *					- PUBLIC_CRYPTO_DMA_USE_NONE,
 *					- or PUBLIC_CRYPTO_DMA_USE_POLLING,
 *					- or PUBLIC_CRYPTO_DMA_USE_IRQ.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoFinalHash(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx,
								U8 *pDigest,
								U32 dmaUse);

/* *
 *This function terminates a Public HASH Operation.
 *
 *pHashCtx  Context of the Public HASH Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoTerminateHash(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx);

/* *
 *This function saves the registers from the HASH HW accelerator into the
 *passed structure.
 *
 *pHashCtx Context of the Public HASH Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoSaveHashRegisters(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx);

/* *
 *This function restores the registers of the HASH HW accelerator
 *from the passed structure.
 *
 *pHashCtx Context of the Public HASH Operation.
 *
 *It returns: - PUBLIC_CRYPTO_OPERATION_SUCCESS in case of success,
 *		  - PUBLIC_CRYPTO_ACCESS_DENIED otherwise
 */
U32 PDrvCryptoRestoreHashRegisters(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx);

/* *
 *Activates an inactivity timer for power management.
 *If an update command has not been received in a time period
 *(the delay of the timer)then the clocks will be disabled by
 *the timer callabck.
 */
#ifdef SMODULE_SMC_OMAP3430_POWER_MANAGEMENT
void scxPublicCryptoStartInactivityTimer(void);
#endif /*SMODULE_SMC_OMAP3430_POWER_MANAGEMENT */

#endif /*__SCX_PUBLIC_CRYPTO_H */

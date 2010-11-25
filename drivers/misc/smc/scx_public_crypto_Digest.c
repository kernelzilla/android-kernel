/*
 *Copyright (c)2006-2008 Trusted Logic S.A.
 * All Rights Reserved.
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
 * along with this program; if not, write to the Free Software
 *Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 *MA 02111-1307 USA
 */

#include "scxlnx_defs.h"
#include "scxlnx_util.h"
#include "scx_public_crypto.h"
#include "scx_public_dma.h"

/*
 *SHA2/MD5 Hardware Accelerator: Base address
 */
#define DIGEST1_REGS_HW_ADDR		  0x480A4000


/*
 *IRQSTAT register Masks
 */
#define DIGEST_IRQSTAT_OUTPUT_READY_BIT	(1<<0)
#define DIGEST_IRQSTAT_INPUT_READY_BIT	 (1<<1)
#define DIGEST_IRQSTAT_PARTHASH_READY	  (1<<2)


/*
 *CTRL register Masks
 */
#define DIGEST_CTRL_GET_ALGO(x)((x&0x6) >> 1)
#define DIGEST_CTRL_SET_ALGO(x, a)((a<<1) | (x&0xFFFFFFF9))
#define DIGEST_CTRL_ALGO_MD5		  0
#define DIGEST_CTRL_ALGO_SHA1		 1
#define DIGEST_CTRL_ALGO_SHA224	  2
#define DIGEST_CTRL_ALGO_SHA256	  3

#define DIGEST_CTRL_ALGO_CONST_BIT  (1<<3)
#define DIGEST_CTRL_CLOSE_HASH_BIT  (1<<4)

#define DIGEST_CTRL_GET_LENGTH(x)((x>>5) & 0x07FFFFFF)
#define DIGEST_CTRL_SET_LENGTH(x, l)((x&0x0000001F) | ((l<<5) & 0xFFFFFFE0))


/*
 *MASK register masks
 */
#define DIGEST_MASK_AUTOIDLE_BIT		 (1<<0)
#define DIGEST_MASK_SOFTRESET_BIT		(1<<1)
#define DIGEST_MASK_IT_EN_BIT			 (1<<2)
#define DIGEST_MASK_DMA_EN_BIT			(1<<3)

#define DIGEST_MASK_GET_SIDLE(x)((x&0x20) >> 4)
#define DIGEST_MASK_SET_SIDLE(x, s)((s<<4) | (x&0xFFFFFFCF))
#define DIGEST_MASK_SIDLE_FORCE_IDLE	0
#define DIGEST_MASK_SIDLE_NO_IDLE		1
#define DIGEST_MASK_SIDLE_SMART_IDLE	2

#define DIGEST_MASK_CONTEXT_BIT		  (1<<6)


/*
 *SYSTATUS register masks
 */
#define DIGEST_SYSTATUS_RESET_DONE	  (1<<0)


#define CL_HASH_ALGO_SHA1						 (0x00)
#define CL_HASH_ALGO_MD5						  (0x01)
#define CL_HASH_ALGO_SHA2_224					(0x02)
#define CL_HASH_ALGO_SHA2_256					(0x03)


static const U8 md5OverZeroBits[] = {	  0xd4, 0x1d, 0x8c, 0xd9, 0x8f, 0x00, 0xb2, 0x04,
														0xe9, 0x80, 0x09, 0x98, 0xec, 0xf8, 0x42, 0x7e };

static const U8 sha1OverZeroBits[] = {	 0xda, 0x39, 0xa3, 0xee, 0x5e, 0x6b, 0x4b, 0x0d,
														0x32, 0x55, 0xbf, 0xef, 0x95, 0x60, 0x18, 0x90,
														0xaf, 0xd8, 0x07, 0x09 };

static const U8 sha224OverZeroBits[] = {  0xd1, 0x4a, 0x02, 0x8c, 0x2a, 0x3a, 0x2b, 0xc9,
														0x47, 0x61, 0x02, 0xbb, 0x28, 0x82, 0x34, 0xc4,
														0x15, 0xa2, 0xb0, 0x1f, 0x82, 0x8e, 0xa6, 0x2a,
														0xc5, 0xb3, 0xe4, 0x2f };

static const U8 sha256OverZeroBits[] = {  0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14,
														0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
														0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c,
														0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55 };

static U32 static_Hash_HwPerform64bDigest(U32 *pData, U32 nProcessedBytes, U32 control);
static U32 static_Hash_HwReadDigest(U8 *pDigest, U8 nAlgo);


Sha1Md5Reg_t *g_pSha1Md5Reg_t = NULL;


static U32 PDrvCryptoUpdateHashWithDMA(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx,
										  U8 *pData,
										  U32 dataLength,
										  U32 dmaUse);

static U32 static_Hash_HwPerformDmaDigest(U32 *pData, U32 nProcessedBytes, U32 control, U32 dmaUse);

/*---------------------------------------------------------------------------
 *Extern declarations
 *--------------------------------------------------------------------------- */

extern u32 v7_dma_flush_range(u32 nVAStart, u32 nVAEnd);
extern u32 v7_dma_inv_range(u32 nVAStart, u32 nVAEnd);

/*---------------------------------------------------------------------------- */

void PDrvCryptoInitHash(U32 hashAlgorithm,
								PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx)
{
	switch (hashAlgorithm) {
	case PUBLIC_CRYPTO_ALG_DIGEST_MD5:
		hashAlgorithm = DIGEST_CTRL_ALGO_MD5;
		break;
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA1:
		hashAlgorithm = DIGEST_CTRL_ALGO_SHA1;
		break;
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA224:
		hashAlgorithm = DIGEST_CTRL_ALGO_SHA224;
		break;
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA256:
		hashAlgorithm = DIGEST_CTRL_ALGO_SHA256;
		break;
	}

	memset(pHashCtx, 0, sizeof(PUBLIC_CRYPTO_HASH_CONTEXT));

	/*(1) : We map the registers with the global variable */
	g_pSha1Md5Reg_t = (Sha1Md5Reg_t *)IO_ADDRESS(DIGEST1_REGS_HW_ADDR);

	/*(2)We set the algo type */
	pHashCtx->registers.CTRL = hashAlgorithm<<1;
}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoUpdateHash(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx,
								U8 *pData,
								U32 dataLength,
								U32 dmaUse)
{
	U32 nHWAlgo	= 0;
	U8  initValue = 0;
	U32 res = PUBLIC_CRYPTO_OPERATION_SUCCESS;

	if (dmaUse != PUBLIC_CRYPTO_DMA_USE_NONE) {
		return PDrvCryptoUpdateHashWithDMA(pHashCtx, pData, dataLength, dmaUse);
	}

	nHWAlgo	= DIGEST_CTRL_GET_ALGO(INREG32(&g_pSha1Md5Reg_t->CTRL));
	initValue = (pHashCtx->vNbBytesProcessed == 0) ? 1 : 0;

	/*(1)We take the chunk buffer wich contains the last saved data that
	 *	 could not be yet processed because we had not enough data to make
	 *	 a 64B buffer. Then we try to make a 64B buffer by concatenating it
	 *	 with the new passed data
	 */

	/*Is there any data in the chunk? If yes is it possible to make a 64B buffer
		with the new data passed ? */
	if ((pHashCtx->vChunckIndex != 0) && (pHashCtx->vChunckIndex + dataLength >= HASH_BLOCK_BYTES_LENGTH)) {
		U8 vLengthToComplete = HASH_BLOCK_BYTES_LENGTH - pHashCtx->vChunckIndex;

		/*So we fill the chunk buffer with the new data to complete to 64B */
		memcpy(pHashCtx->pChunckBuffer + pHashCtx->vChunckIndex, pData, vLengthToComplete);

		if (pHashCtx->vChunckIndex + dataLength == HASH_BLOCK_BYTES_LENGTH) {
			/*We'll keep some data for the final */
			pHashCtx->vChunckIndex = HASH_BLOCK_BYTES_LENGTH;
			return PUBLIC_CRYPTO_OPERATION_SUCCESS;
		}

		/*Then we send this buffer to the hash hardware */
		res = static_Hash_HwPerform64bDigest((U32 *)pHashCtx->pChunckBuffer, pHashCtx->vNbBytesProcessed,
														(HASH_BLOCK_BYTES_LENGTH << 5) | (initValue << 3) | (nHWAlgo << 1));
		initValue = 0;
		if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			return res;
		}
		pHashCtx->vNbBytesProcessed += HASH_BLOCK_BYTES_LENGTH;

		/*We have flushed the chunk so it is empty now */
		pHashCtx->vChunckIndex = 0;

		/*Then we have less data to proceed */
		pData		+= vLengthToComplete;
		dataLength -= vLengthToComplete;
	}

	/*(2)We process all the 64B buffer that we can */
	if (pHashCtx->vChunckIndex + dataLength >= HASH_BLOCK_BYTES_LENGTH) {
		/*Dirty patch to confirm : keep the last full block of 64B for the final.
			We must keep some data for the final. To remove the patch replace > by >= */
		while (dataLength > HASH_BLOCK_BYTES_LENGTH) {
			U8 pTempAlignedBuffer[HASH_BLOCK_BYTES_LENGTH];

			/*
			 *We process a 64B buffer
			 */

			/*We copy the data to process to an aligned buffer !LINUX only ??! */
			memcpy(pTempAlignedBuffer, pData, HASH_BLOCK_BYTES_LENGTH);

			/*Then we send this buffer to the hash hardware */
			res = static_Hash_HwPerform64bDigest((U32 *)pTempAlignedBuffer, pHashCtx->vNbBytesProcessed,
															(HASH_BLOCK_BYTES_LENGTH << 5) | (initValue << 3) | (nHWAlgo << 1));
			initValue = 0;
			if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
				return res;
			}
			pHashCtx->vNbBytesProcessed += HASH_BLOCK_BYTES_LENGTH;

			/*Then we decrease the remaining data of 64B */
			pData		+= HASH_BLOCK_BYTES_LENGTH;
			dataLength -= HASH_BLOCK_BYTES_LENGTH;
		}
	}

	/*(3)We look if we have some data that could not be processed yet because it is
		not large enough to fill a buffer of 64B */

	if (dataLength > 0) {
		if (pHashCtx->vChunckIndex + dataLength > HASH_BLOCK_BYTES_LENGTH) {
			/*Should never be in this case => Serious problem in the code !!! */
			return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
		} else {
			/*So we fill the chunk buffer with the new data to complete to 64B */
			memcpy(pHashCtx->pChunckBuffer + pHashCtx->vChunckIndex, pData, dataLength);
			pHashCtx->vChunckIndex += dataLength;
		}
	}

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoFinalHash(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx,
								U8 *pDigest,
								U32 dmaUse)
{
	U32 res = PUBLIC_CRYPTO_OPERATION_SUCCESS;
	U32 nHWAlgo	= 0;
	U32 vChunkLowIndex = 0;

	nHWAlgo	= DIGEST_CTRL_GET_ALGO(INREG32(&g_pSha1Md5Reg_t->CTRL));

	if (pHashCtx->vNbBytesProcessed + pHashCtx->vChunckIndex == 0) {
		/*In this case we return constant specific values */
		switch (nHWAlgo) {
		case DIGEST_CTRL_ALGO_MD5:
			memcpy(pDigest, md5OverZeroBits, HASH_MD5_LENGTH);
			break;
		case DIGEST_CTRL_ALGO_SHA1:
			memcpy(pDigest, sha1OverZeroBits, HASH_SHA1_LENGTH);
			break;
		case DIGEST_CTRL_ALGO_SHA224:
			memcpy(pDigest, sha224OverZeroBits, HASH_SHA224_LENGTH);
			break;
		case DIGEST_CTRL_ALGO_SHA256:
			memcpy(pDigest, sha256OverZeroBits, HASH_SHA256_LENGTH);
			break;
		}

		return PUBLIC_CRYPTO_OPERATION_SUCCESS;
	}

	/*
	 * at this stage, we have processed all possible blocks of 64 bytes.
	 *Let's finalize the hash operation.
	 */
	 if (pHashCtx->vChunckIndex == 0) {
		 /*We don't do the final if there is no data => to confirm, perhaps this patch can be removed */
		 return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	 }

	 {
		 U8  initValue = 0;
		 initValue = (pHashCtx->vNbBytesProcessed == 0) ? 1 : 0;

		 while (((signed int)((signed int)pHashCtx->vChunckIndex - (signed int)vChunkLowIndex)) > HASH_BLOCK_BYTES_LENGTH) {
			 res = static_Hash_HwPerform64bDigest((U32 *)((U32)pHashCtx->pChunckBuffer + vChunkLowIndex), pHashCtx->vNbBytesProcessed,
															(HASH_BLOCK_BYTES_LENGTH << 5) | (initValue << 3) | (nHWAlgo << 1));

			 if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
				 return res;
			 }

			 pHashCtx->vNbBytesProcessed += HASH_BLOCK_BYTES_LENGTH;
			 vChunkLowIndex += HASH_BLOCK_BYTES_LENGTH;
		 }

		 res = static_Hash_HwPerform64bDigest((U32 *)((U32)pHashCtx->pChunckBuffer + vChunkLowIndex), pHashCtx->vNbBytesProcessed,
						  ((pHashCtx->vChunckIndex - vChunkLowIndex) << 5) | 0x10 | (initValue << 3) | (nHWAlgo << 1));
	 }

	if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		/*TRACE_ERROR1("< pubdrv_doDigestFinalOperation: static_pubdrv_doDigestFinalOperation error [0x%08x]", (unsigned int)res); */
		return res;
	}

	/*Then write the result */
	res = static_Hash_HwReadDigest(pDigest, nHWAlgo);

	if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		return res;
	}

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;

}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoTerminateHash(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx)
{
	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoSaveHashRegisters(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx)
{
	pHashCtx->registers.DIGEST_A = INREG32(&g_pSha1Md5Reg_t->DIGEST_A);
	pHashCtx->registers.DIGEST_B = INREG32(&g_pSha1Md5Reg_t->DIGEST_B);
	pHashCtx->registers.DIGEST_C = INREG32(&g_pSha1Md5Reg_t->DIGEST_C);
	pHashCtx->registers.DIGEST_D = INREG32(&g_pSha1Md5Reg_t->DIGEST_D);
	pHashCtx->registers.DIGEST_E = INREG32(&g_pSha1Md5Reg_t->DIGEST_E);
	pHashCtx->registers.DIGEST_F = INREG32(&g_pSha1Md5Reg_t->DIGEST_F);
	pHashCtx->registers.DIGEST_G = INREG32(&g_pSha1Md5Reg_t->DIGEST_G);
	pHashCtx->registers.DIGEST_H = INREG32(&g_pSha1Md5Reg_t->DIGEST_H);
	pHashCtx->registers.CTRL	  = INREG32(&g_pSha1Md5Reg_t->CTRL);

	/*Clear Digest HWA registers */
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_A, 0x0);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_B, 0x0);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_C, 0x0);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_D, 0x0);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_E, 0x0);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_F, 0x0);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_G, 0x0);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_H, 0x0);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoRestoreHashRegisters(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx)
{
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_A, pHashCtx->registers.DIGEST_A);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_B, pHashCtx->registers.DIGEST_B);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_C, pHashCtx->registers.DIGEST_C);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_D, pHashCtx->registers.DIGEST_D);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_E, pHashCtx->registers.DIGEST_E);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_F, pHashCtx->registers.DIGEST_F);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_G, pHashCtx->registers.DIGEST_G);
	OUTREG32(&g_pSha1Md5Reg_t->DIGEST_H, pHashCtx->registers.DIGEST_H);
	OUTREG32(&g_pSha1Md5Reg_t->CTRL, pHashCtx->registers.CTRL);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

U32 static_Hash_HwPerform64bDigest(U32 *pData, U32 nProcessedBytes, U32 control)
{
	/*
	 *Restore the DIGCNT register first
	 */
	OUTREG32(&g_pSha1Md5Reg_t->DIGCNT, nProcessedBytes);

	/*
	 *Then set the register control to start the operation
	 */
	OUTREG32(&g_pSha1Md5Reg_t->CTRL, control);

	if (scxPublicCryptoWaitForReadyBit((VU32 *) &g_pSha1Md5Reg_t->IRQSTAT, DIGEST_IRQSTAT_INPUT_READY_BIT) != PUBLIC_CRYPTO_OPERATION_SUCCESS)
	{
		/* Crash the system as this should never occur */
		panic("Wait too long for DIGEST HW accelerator Input data to be ready\n");
	}

	/*
	 *The pData buffer is a buffer of at least 64 bytes.
	 *For an update operation, we are always processing a 64-bytes buffer.
	 *Only the final operation might process a smaller buffer.
	 *Let's make it simple, and always write all registers.
	 */
	OUTREG32(&g_pSha1Md5Reg_t->DIN_0, pData[0]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_1, pData[1]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_2, pData[2]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_3, pData[3]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_4, pData[4]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_5, pData[5]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_6, pData[6]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_7, pData[7]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_8, pData[8]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_9, pData[9]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_10, pData[10]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_11, pData[11]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_12, pData[12]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_13, pData[13]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_14, pData[14]);
	OUTREG32(&g_pSha1Md5Reg_t->DIN_15, pData[15]);

	/*
	 *Wait until the hash operation is finished.
	 */
	scxPublicCryptoWaitForReadyBitInfinitely((VU32 *) &g_pSha1Md5Reg_t->IRQSTAT, DIGEST_IRQSTAT_OUTPUT_READY_BIT);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*---------------------------------------------------------------------------- */

U32 static_Hash_HwPerformDmaDigest(U32 *pData, U32 nProcessedBytes, U32 control, U32 dmaUse)
{
	U32 bufIn_phys = virt_to_phys(pData);
	U32 dma_ch0 = OMAP3430_SMC_DMA_CH_0;	/*Hard Coded Tx Data */
	SCX_DMA_CHANNEL_PARAM ch0_parameters;
	U32 returnCode = PUBLIC_CRYPTO_OPERATION_SUCCESS;
	int sizeToProcess;

	sizeToProcess = (control & 0xFFFFFFE0) >> 5;

	/*Makes sure that if the dma channels that will need to be used are currently active,
	one can reprogram it (them)*/
	scxPublicDMADisableChannel(dma_ch0);

	if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
		/*Reset DMA int (DMA CTRL) - The DMA int (INT CTRL)is reset by the OS */
		scxPublicDMADisableL3IRQ();
		scxPublicDMAClearL3IRQ();
	}

	/*DMA1: Mem -> HASH */
	ch0_parameters.data_type =	DMA_CSDP_Srce_Endian_little
		| DMA_CSDP_Srce_Endian_Lock_off
		| DMA_CSDP_Dest_Endian_little
		| DMA_CSDP_Dest_Endian_Lock_off
		| DMA_CSDP_Write_Mode_none_posted
		| DMA_CSDP_Dest_Burst_64B
		| DMA_CSDP_Dest_packed_off
		| DMA_CSDP_WR_Add_Trslt
		| DMA_CSDP_Src_Burst_64B
		| DMA_CSDP_Src_packed_off
		| DMA_CSDP_RD_Add_Trslt
		| DMA_CSDP_Data_32b;
	ch0_parameters.elem_count = DMA_CEN_Elts_per_Frame_SHA;
	ch0_parameters.frame_count = sizeToProcess / HASH_BLOCK_BYTES_LENGTH;

	ch0_parameters.src_amode = 1; /*post increment */
	ch0_parameters.src_start = bufIn_phys;
	ch0_parameters.src_ei = DMA_CSEI_Default;
	ch0_parameters.src_fi = DMA_CSFI_Default;

	ch0_parameters.dst_amode = 0; /*const */
	ch0_parameters.dst_start = DIGEST1_REGS_HW_ADDR + 0x30;
	ch0_parameters.dst_ei = DMA_CDEI_Default;
	ch0_parameters.dst_fi = DMA_CDFI_Default; /*source frame index */

	ch0_parameters.trigger = DMA_CCR_Mask_Channel(DMA_CCR_Channel_Mem2SHA);
	ch0_parameters.sync_mode = 0x2; /*FS =1, BS=0 => An entire frame is
												  transferred once a DMA request is made */
	ch0_parameters.src_or_dst_synch = 0; /*Transfert is triggered by the Dest */

	dprintk(KERN_INFO "PDrvCryptoUpdateHASHWithDMA: scxPublicDMASetParams(ch0)\n");
	scxPublicDMASetParams(dma_ch0, &ch0_parameters);

	if (nProcessedBytes == 0) {
		/* Reset the digest hw => Because DIGCNT should be reset : it's the only way to do so */
		OUTREG32(&g_pSha1Md5Reg_t->MASK, DIGEST_MASK_SOFTRESET_BIT);

		scxPublicCryptoWaitForReadyBitInfinitely((VU32 *) &g_pSha1Md5Reg_t->SYSSTATUS, DIGEST_SYSTATUS_RESET_DONE);
	}

	dprintk(KERN_INFO "PDrvCryptoUpdateHASHWithDMA: Start DMA channel %d\n", (unsigned int)dma_ch0);

	/*
	 *Set the register control to start the operation
	 */
	OUTREG32(&g_pSha1Md5Reg_t->CTRL, control);


	/*
	 *Set the DIGCNT register
	 */
	OUTREG32(&g_pSha1Md5Reg_t->DIGCNT, nProcessedBytes);


	if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
		scxPublicDMAEnableL3IRQ();
	}

	dprintk(KERN_INFO "PDrvCryptoUpdateHASHWithDMA: Start DMA channel %d\n", (unsigned int)dma_ch0);

	v7_dma_flush_range((int)pData, (int)pData + sizeToProcess);

	/*Start operation */
	/*Triggers operation - Interrupt, Free Running + GO (DMA on)*/
	OUTREG32(&g_pSha1Md5Reg_t->MASK,
				INREG32(&g_pSha1Md5Reg_t->MASK) | DIGEST_MASK_DMA_EN_BIT|DIGEST_MASK_IT_EN_BIT);

	scxPublicDMAStart(dma_ch0, OMAP_DMA_DROP_IRQ|OMAP_DMA_BLOCK_IRQ);

	if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
		/*Suspends the process until the DMA IRQ occurs */
		dprintk(KERN_INFO "PDrvCryptoUpdateHASHWithDMA: Waiting for IRQ\n");
		returnCode = scxPublicDMAWait();
	} else {
		dprintk(KERN_INFO "PDrvCryptoUpdateHASHWithDMA: Polling DMA\n");
		returnCode = scxPublicDMAPoll(dma_ch0);
	}

	if (returnCode != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		dprintk(KERN_ERR "PDrvCryptoUpdateHASHWithDMA: Timeout\n");
		/*Do not exit function but clear properly the operation */
	}

	if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
		/*Acknoledge DMA interrupt */
		scxPublicDMADisableL3IRQ();
	}

	scxPublicDMAClearChannel(dma_ch0);

	/*The dma transfert is finished, now wait until the hash operation is finished. */
	scxPublicCryptoWaitForReadyBitInfinitely((VU32 *) &g_pSha1Md5Reg_t->IRQSTAT, DIGEST_IRQSTAT_OUTPUT_READY_BIT);

	/*Stop clocks */
	OUTREG32(&g_pSha1Md5Reg_t->MASK,
			  INREG32(&g_pSha1Md5Reg_t->MASK) & (~DIGEST_MASK_DMA_EN_BIT) & (~DIGEST_MASK_IT_EN_BIT));

	 /*Clear the interrupt */
	OUTREG32(&g_pSha1Md5Reg_t->CTRL, INREG32(&g_pSha1Md5Reg_t->CTRL) & 0x0000001F);/*clear length field */

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*---------------------------------------------------------------------------- */
/*
 *Static functions
 */

static U32 PDrvCryptoUpdateHashWithDMA(PUBLIC_CRYPTO_HASH_CONTEXT *pHashCtx,
										  U8 *pData,
										  U32 dataLength,
										  U32 dmaUse)
{
	/*
	 *Note: The DMA only sees physical addresses !
	 */
	U32 bufIn_phys = virt_to_phys(pData);
	U32 res = PUBLIC_CRYPTO_OPERATION_SUCCESS;
	U32 nHWAlgo	= 0;
	U32 vInitValue = 0;
	U32 vDirectProcess = 0;

	if (pHashCtx->vNbBytesProcessed == 0) {
		vInitValue = 1;
	}

	nHWAlgo	= DIGEST_CTRL_GET_ALGO(INREG32(&g_pSha1Md5Reg_t->CTRL));

	dprintk(KERN_INFO "PDrvCryptoUpdateHASHWithDMA: Use=%u, Len=%u, In=0x%08x\n",
					(unsigned int)dmaUse, (unsigned int)dataLength, (unsigned int)bufIn_phys);


	if (dataLength == 0) {
		 /*No need of setting the dma and crypto-processor */
		dprintk(KERN_INFO "PDrvCryptoUpdateHASHWithDMA: Nothing to process\n");
		return PUBLIC_CRYPTO_OPERATION_SUCCESS;
	}

	if (pData == 0) {
		 dprintk(KERN_ERR "PDrvCryptoUpdateHASHWithDMA: bufIn_phys/bufOut_phys NULL\n");
		 return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}

	/*Makes sure buffers are 4-bytes aligned */
	if (!IS_4_BYTES_ALIGNED((int)bufIn_phys)) {
		dprintk(KERN_ERR "PDrvCryptoUpdateHASHWithDMA: bufIn_phys/Out not 4 bytes aligned\n");
		return PUBLIC_CRYPTO_ERR_ALIGNMENT;
	}

	if (pHashCtx->vChunckIndex == 0) {
		vDirectProcess = 1;
	}
	if ((IS_4_BYTES_ALIGNED(HASH_CHUNK_BYTES_LENGTH-pHashCtx->vChunckIndex)) && (dataLength > HASH_CHUNK_BYTES_LENGTH-pHashCtx->vChunckIndex)) {
		/*Fill the chunk */
		U32 vLengthToComplete = HASH_CHUNK_BYTES_LENGTH - pHashCtx->vChunckIndex;

		/*So we fill the chunk buffer with the new data to complete to HASH_CHUNK_BYTES_LENGTH */
		memcpy(pHashCtx->pChunckBuffer + pHashCtx->vChunckIndex, pData, vLengthToComplete);
		pHashCtx->vChunckIndex = HASH_CHUNK_BYTES_LENGTH;
		pData		+= vLengthToComplete;
		dataLength -= vLengthToComplete;

		/* Process the chunk first with no dma */
		if (HASH_CHUNK_BYTES_LENGTH == HASH_BLOCK_BYTES_LENGTH) {
			res = static_Hash_HwPerform64bDigest((U32 *)pHashCtx->pChunckBuffer, pHashCtx->vNbBytesProcessed,
															(HASH_CHUNK_BYTES_LENGTH << 5) | (vInitValue << 3) | (nHWAlgo << 1));
		} else {
			res = static_Hash_HwPerformDmaDigest((U32 *)pHashCtx->pChunckBuffer, pHashCtx->vNbBytesProcessed,
															(HASH_CHUNK_BYTES_LENGTH << 5) | (vInitValue << 3) | (nHWAlgo << 1), dmaUse);

		}

		if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			 return res;
		}

		vInitValue = 0;

		pHashCtx->vNbBytesProcessed += HASH_CHUNK_BYTES_LENGTH;
		pHashCtx->vChunckIndex = 0;
		vDirectProcess = 1;
	}

	if ((vDirectProcess == 1) && (dataLength > HASH_BLOCK_BYTES_LENGTH)) {
		U32 vDmaProcessSize = dataLength & 0xFFFFFFC0;

		if (vDmaProcessSize == dataLength) {
			vDmaProcessSize = vDmaProcessSize - HASH_BLOCK_BYTES_LENGTH; /* We keep one block for the final */
		}

		res = static_Hash_HwPerformDmaDigest((U32 *)pData, pHashCtx->vNbBytesProcessed,
														(vDmaProcessSize << 5) | (vInitValue << 3) | (nHWAlgo << 1), dmaUse);

		if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			return res;
		}

		vInitValue = 0;

		pHashCtx->vNbBytesProcessed += vDmaProcessSize;
		pData		+= vDmaProcessSize;
		dataLength -= vDmaProcessSize;
	}

	/******************************** TO DO ******************************/
	/* Look if we can fill the chunk. If it is filled then process the dma */
	/* on the HASH_CHUNK_BYTES_LENGTH length. And put the remaining in the */
	/* chunk. Else put the remaining in the chunk and return the function. */
	/*********************************************************************/
	/* Is there any data in the chunk? If yes is it possible to make a HASH_CHUNK_BYTES_LENGTH buffer
		with the new data passed ? */
	if ((pHashCtx->vChunckIndex != 0) && (pHashCtx->vChunckIndex + dataLength >= HASH_CHUNK_BYTES_LENGTH)) {
		U32 vLengthToComplete = HASH_CHUNK_BYTES_LENGTH - pHashCtx->vChunckIndex;

		/*So we fill the chunk buffer with the new data to complete to HASH_CHUNK_BYTES_LENGTH */
		memcpy(pHashCtx->pChunckBuffer + pHashCtx->vChunckIndex, pData, vLengthToComplete);

		if (pHashCtx->vChunckIndex + dataLength == HASH_CHUNK_BYTES_LENGTH) {
			/*We'll keep some data for the final */
			pHashCtx->vChunckIndex = HASH_CHUNK_BYTES_LENGTH;
			return PUBLIC_CRYPTO_OPERATION_SUCCESS;
		}

		/*Then we send this buffer to the hash hardware */
		if (HASH_CHUNK_BYTES_LENGTH == HASH_BLOCK_BYTES_LENGTH) {
			res = static_Hash_HwPerform64bDigest((U32 *)pHashCtx->pChunckBuffer, pHashCtx->vNbBytesProcessed,
															(HASH_CHUNK_BYTES_LENGTH << 5) | (vInitValue << 3) | (nHWAlgo << 1));
		} else {
			res = static_Hash_HwPerformDmaDigest((U32 *)pHashCtx->pChunckBuffer, pHashCtx->vNbBytesProcessed,
															(HASH_CHUNK_BYTES_LENGTH << 5) | (vInitValue << 3) | (nHWAlgo << 1), dmaUse);
		}

		vInitValue = 0;

		if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			return res;
		}
		pHashCtx->vNbBytesProcessed += HASH_CHUNK_BYTES_LENGTH;

		/*We have flushed the chunk so it is empty now */
		pHashCtx->vChunckIndex = 0;

		/*Then we have less data to proceed */
		pData		+= vLengthToComplete;
		dataLength -= vLengthToComplete;
	}

	/*(2)We process all the HASH_CHUNK_BYTES_LENGTH buffer that we can */
	/*Dirty patch to confirm : keep the last full block of HASH_CHUNK_BYTES_LENGTH for the final.
		We must keep some data for the final. To remove the patch replace > by >= */
	while (dataLength > HASH_CHUNK_BYTES_LENGTH) {
		U8 pTempAlignedBuffer[HASH_CHUNK_BYTES_LENGTH];

		/*
		 *We process a HASH_CHUNK_BYTES_LENGTH buffer
		 */

		/*We copy the data to process to an aligned buffer !LINUX only ??! */
		memcpy(pTempAlignedBuffer, pData, HASH_CHUNK_BYTES_LENGTH);

		/*Then we send this buffer to the hash hardware */
		if (HASH_CHUNK_BYTES_LENGTH == HASH_BLOCK_BYTES_LENGTH) {
			res = static_Hash_HwPerform64bDigest((U32 *)pTempAlignedBuffer, pHashCtx->vNbBytesProcessed,
															(HASH_CHUNK_BYTES_LENGTH << 5) | (vInitValue << 3) | (nHWAlgo << 1));
		} else {
			res = static_Hash_HwPerformDmaDigest((U32 *)pTempAlignedBuffer, pHashCtx->vNbBytesProcessed,
															(HASH_CHUNK_BYTES_LENGTH << 5) | (vInitValue << 3) | (nHWAlgo << 1), dmaUse);
		}


		vInitValue = 0;

		if (res != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			return res;
		}
		pHashCtx->vNbBytesProcessed += HASH_CHUNK_BYTES_LENGTH;

		/*Then we decrease the remaining data of HASH_CHUNK_BYTES_LENGTH */
		pData		+= HASH_CHUNK_BYTES_LENGTH;
		dataLength -= HASH_CHUNK_BYTES_LENGTH;
	}


	/*(3)We look if we have some data that could not be processed yet because it is
		not large enough to fill a buffer of HASH_CHUNK_BYTES_LENGTH */
	if (dataLength > 0) {
		if (pHashCtx->vChunckIndex + dataLength > HASH_CHUNK_BYTES_LENGTH) {
			/*Should never be in this case => Serious problem in the code !!! */
			return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
		} else {
			/*So we fill the chunk buffer with the new data */
			memcpy(pHashCtx->pChunckBuffer + pHashCtx->vChunckIndex, pData, dataLength);
			pHashCtx->vChunckIndex += dataLength;
		}
	}

	dprintk(KERN_INFO "PDrvCryptoUpdateHASHWithDMA: Success\n");

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

U32 static_Hash_HwReadDigest(U8 *pDigest, U8 nAlgo)
{
	U8 vBigEndian = 0;
	U8 vNbRegs	 = 0;
	U32 *pReg;
	U32 tmp;
	U8 vOutIndex = 0;
	U8 pDigestBuf[32]; /*We need an aligned buffer */
	U32 i;

	if (nAlgo == DIGEST_CTRL_ALGO_MD5) {
		vBigEndian = 0;
		vNbRegs	 = 4;
	} else if (nAlgo == DIGEST_CTRL_ALGO_SHA1) {
		vBigEndian = 1;
		vNbRegs	 = 5;
	} else if (nAlgo == DIGEST_CTRL_ALGO_SHA224) {
		vBigEndian = 1;
		vNbRegs	 = 7;
	} else if (nAlgo == DIGEST_CTRL_ALGO_SHA256) {
		vBigEndian = 1;
		vNbRegs	 = 8;
	} else {
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}


	for (i = 0; i < vNbRegs; i++) {
		pReg = (((U32 *) &g_pSha1Md5Reg_t->DIGEST_A) + i);
		tmp = INREG32(pReg);

		if (vBigEndian) {
			pDigestBuf[vOutIndex++] = (U8)((tmp >> 24) & 0xFF);
			pDigestBuf[vOutIndex++] = (U8)((tmp >> 16) & 0xFF);
			pDigestBuf[vOutIndex++] = (U8)((tmp >> 8) & 0xFF);
			pDigestBuf[vOutIndex++] = (U8)(tmp & 0xFF);
		} else {
			pDigestBuf[vOutIndex++] = (U8)(tmp & 0xFF);
			pDigestBuf[vOutIndex++] = (U8)((tmp >> 8) & 0xFF);
			pDigestBuf[vOutIndex++] = (U8)((tmp >> 16) & 0xFF);
			pDigestBuf[vOutIndex++] = (U8)((tmp >> 24) & 0xFF);
		}
	}

	memcpy(pDigest, pDigestBuf, vNbRegs * 4);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

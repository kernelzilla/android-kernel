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
 *DES Hardware Accelerator: Base address
 */
#define DES2_REGS_HW_ADDR			  0x480C1000


/*
 *CTRL register Masks
 */
#define DES_CTRL_OUTPUT_READY_BIT	(1<<0)
#define DES_CTRL_INPUT_READY_BIT	 (1<<1)

#define DES_CTRL_GET_DIRECTION(x)(x&4)
#define DES_CTRL_DIRECTION_DECRYPT  0
#define DES_CTRL_DIRECTION_ENCRYPT  (1<<2)

#define DES_CTRL_GET_TDES(x)(x&8)
#define DES_CTRL_TDES_DES			  0
#define DES_CTRL_TDES_TRIPLE_DES	 (1<<3)

#define DES_CTRL_GET_MODE(x)(x&0x10)
#define DES_CTRL_MODE_ECB			  0
#define DES_CTRL_MODE_CBC			  (1<<4)


/*
 *MASK register masks
 */
#define DES_MASK_AUTOIDLE_BIT		 (1<<0)
#define DES_MASK_SOFTRESET_BIT		(1<<1)
#define DES_MASK_DMA_REQ_IN_EN_BIT  (1<<2)
#define DES_MASK_DMA_REQ_OUT_EN_BIT (1<<3)
#define DES_MASK_DIRECT_BUS_EN_BIT  (1<<4)
#define DES_MASK_START_BIT			 (1<<5)

#define DES_MASK_GET_SIDLE(x)(x&0xC0)
#define DES_MASK_SIDLE_FORCE_IDLE	0
#define DES_MASK_SIDLE_NO_IDLE		(1<<6)
#define DES_MASK_SIDLE_SMART_IDLE	(1<<7)


/*
 *SYSTATUS register masks
 */
#define DES_SYSTATUS_RESET_DONE	  (1<<0)


Des3DesReg_t *g_pDESReg_t = NULL;

/*---------------------------------------------------------------------------
 *Extern declarations
 *--------------------------------------------------------------------------- */

extern u32 v7_dma_flush_range(u32 nVAStart, u32 nVAEnd);
extern u32 v7_dma_inv_range(u32 nVAStart, u32 nVAEnd);


/*---------------------------------------------------------------------------- */
static U32 PDrvCryptoUpdateDESWithDMA(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesCtx,
								U8 *pSrc, U8 *pDest, U32 nbBlocks, U32 dmaUse);

void PDrvCryptoInitDES(CryptoInitContext *pDesDes3InitCtx,
							  PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx)
{
	U8 *pPassedIv = pDesDes3InitCtx->iv;

	memset(pDesDes3Ctx, 0, sizeof(PUBLIC_CRYPTO_DES_DES3_CONTEXT));

	/*(1) : We map the registers with the global variable */
	g_pDESReg_t = (Des3DesReg_t *)IO_ADDRESS(DES2_REGS_HW_ADDR);


	/*(2) : We initialize the registers from the init parameters */
	pDesDes3Ctx->registers.DES_CTRL = pDesDes3InitCtx->hwa_config;
	pDesDes3Ctx->registers.DES_IV_L = (U32)BYTES_TO_LONG(pPassedIv);
	pPassedIv += 4;
	pDesDes3Ctx->registers.DES_IV_H = (U32)BYTES_TO_LONG(pPassedIv);
}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoUpdateDES(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx,
								U8 *pSrc,
								U8 *pDest,
								U32 nbBlocks,
								U32 dmaUse)
{
	U32 nbr_of_blocks;
	U8 *pProcessSrc;
	U8 *pProcessDest;
	U32 vTemp;

	pProcessSrc  = pSrc;
	pProcessDest = pDest;

	if ((dmaUse == PUBLIC_CRYPTO_DMA_USE_POLLING) || (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ)) {
		return PDrvCryptoUpdateDESWithDMA(pDesDes3Ctx, pSrc, pDest, nbBlocks, dmaUse);
	}

	for (nbr_of_blocks = 0; nbr_of_blocks < nbBlocks ; nbr_of_blocks++) {
		/*(2) : We wait for the input ready */
		if (scxPublicCryptoWaitForReadyBit((VU32 *) &g_pDESReg_t->DES_CTRL, DES_CTRL_INPUT_READY_BIT) != PUBLIC_CRYPTO_OPERATION_SUCCESS)
		{
			/* Crash the system as this should never occur */
			panic("Wait too long for DES HW accelerator Input data to be ready\n");
		}

		/*(3) : We copy the 8 bytes of data src->reg */
		vTemp = (U32)BYTES_TO_LONG(pProcessSrc);
		OUTREG32(&g_pDESReg_t->DES_DATA_L, vTemp);
		pProcessSrc += 4;
		vTemp = (U32)BYTES_TO_LONG(pProcessSrc);
		OUTREG32(&g_pDESReg_t->DES_DATA_H, vTemp);
		pProcessSrc += 4;

		/*(4) : We wait for the output ready */
		scxPublicCryptoWaitForReadyBitInfinitely((VU32 *) &g_pDESReg_t->DES_CTRL, DES_CTRL_OUTPUT_READY_BIT);
		
		/*(5) : We copy the 8 bytes of data reg->dest */
		vTemp = INREG32(&g_pDESReg_t->DES_DATA_L);
		LONG_TO_BYTE(pProcessDest, vTemp);
		pProcessDest += 4;
		vTemp = INREG32(&g_pDESReg_t->DES_DATA_H);
		LONG_TO_BYTE(pProcessDest, vTemp);
		pProcessDest += 4;
	}

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoTerminateDES(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx)
{
	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoSaveDESRegisters(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx)
{
	/*(1) : Save the IV if we are in CBC mode */
	if (DES_CTRL_GET_MODE(INREG32(&g_pDESReg_t->DES_CTRL)) == DES_CTRL_MODE_CBC) {
		pDesDes3Ctx->registers.DES_IV_L = INREG32(&g_pDESReg_t->DES_IV_L);
		pDesDes3Ctx->registers.DES_IV_H = INREG32(&g_pDESReg_t->DES_IV_H);
	}

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

U32 PDrvCryptoRestoreDESRegisters(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesDes3Ctx)
{
	/*Write the IV ctx->reg */
	if (DES_CTRL_GET_MODE(INREG32(&pDesDes3Ctx->registers.DES_CTRL)) == DES_CTRL_MODE_CBC) {
		OUTREG32(&g_pDESReg_t->DES_IV_L, pDesDes3Ctx->registers.DES_IV_L);
		OUTREG32(&g_pDESReg_t->DES_IV_H, pDesDes3Ctx->registers.DES_IV_H);
	}

	/*Set the CTRL register from the context but resets the input&output ready and reserved bits */
	OUTREG32(&g_pDESReg_t->DES_CTRL, pDesDes3Ctx->registers.DES_CTRL & 0x1C);

	/*Set the MASK register from the context but resets reserved bits */
	OUTREG32(&g_pDESReg_t->DES_MASK, pDesDes3Ctx->registers.DES_MASK & 0xFF);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

/*
 *Static functions
 */

static U32 PDrvCryptoUpdateDESWithDMA(PUBLIC_CRYPTO_DES_DES3_CONTEXT *pDesCtx,
								U8 *pSrc, U8 *pDest, U32 nbBlocks, U32 dmaUse)
{
	/*
	 *Note: The DMA only sees physical addresses !
	 */

	U32 bufIn_phys = virt_to_phys(pSrc);
	U32 bufOut_phys = virt_to_phys(pDest);

	U32 dma_ch0 = OMAP3430_SMC_DMA_CH_0;
	U32 dma_ch1 = OMAP3430_SMC_DMA_CH_1;
	SCX_DMA_CHANNEL_PARAM ch0_parameters;
	SCX_DMA_CHANNEL_PARAM ch1_parameters;
	U32 nLength = nbBlocks * DES_BLOCK_SIZE;
	U32 returnCode;

	dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: Use=%u, Len=%u, In=0x%08x, Out=0x%08x\n",
					(unsigned int)dmaUse, (unsigned int)nLength, (unsigned int)bufIn_phys, (unsigned int)bufOut_phys);

	if (nLength == 0) {
		 /*No need of setting the dma and crypto-processor */
		dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: Nothing to process\n");
		return PUBLIC_CRYPTO_OPERATION_SUCCESS;
	}

	if ((bufIn_phys == 0) || (bufOut_phys == 0)) {
		 dprintk(KERN_ERR "PDrvCryptoUpdateDESWithDMA: bufIn_phys/bufOut_phys NULL\n");
		 return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}

	/*Makes sure buffers are 4-bytes aligned */
	if (!IS_4_BYTES_ALIGNED((int)bufIn_phys) || !IS_4_BYTES_ALIGNED((int)bufOut_phys)) {
		dprintk(KERN_ERR "PDrvCryptoUpdateDESWithDMA: bufIn_phys/Out not 4 bytes aligned\n");
		return PUBLIC_CRYPTO_ERR_ALIGNMENT;
	}

	/*
	 *Only one segment of the sg list to proceed--> no need of scatter gather algo
	 */

	/*Makes sure that if the dma channels that will need to be used are currently active,
	one can reprogram it (them)*/
	scxPublicDMADisableChannel(dma_ch0);
	scxPublicDMADisableChannel(dma_ch1);

	if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
		/*Reset DMA int (DMA CTRL) - The DMA int (INT CTRL)is reset by the OS */
		scxPublicDMADisableL3IRQ();
		scxPublicDMAClearL3IRQ();
	}

	/*DMA used for Input and Output */
	OUTREG32(&g_pDESReg_t->DES_MASK,
				INREG32(&g_pDESReg_t->DES_MASK)
				| DES_MASK_DMA_REQ_OUT_EN_BIT
				| DES_MASK_DMA_REQ_IN_EN_BIT);

	/*DMA1: Mem -> DES */
	ch0_parameters.data_type =	DMA_CSDP_Srce_Endian_little
		| DMA_CSDP_Srce_Endian_Lock_off
		| DMA_CSDP_Dest_Endian_little
		| DMA_CSDP_Dest_Endian_Lock_off
		| DMA_CSDP_Write_Mode_none_posted
		| DMA_CSDP_Dest_Burst_off
		| DMA_CSDP_Dest_packed_off
		| DMA_CSDP_WR_Add_Trslt
		| DMA_CSDP_Src_Burst_off
		| DMA_CSDP_Src_packed_off
		| DMA_CSDP_RD_Add_Trslt
		| DMA_CSDP_Data_32b;
	ch0_parameters.elem_count = DMA_CEN_Elts_per_Frame_DES;

	ch0_parameters.frame_count = nbBlocks;

	ch0_parameters.src_amode = 1; /*post increment */
	ch0_parameters.src_start = bufIn_phys;
	ch0_parameters.src_ei = DMA_CSEI_Default;
	ch0_parameters.src_fi = DMA_CSFI_Default;

	ch0_parameters.dst_amode = 0; /*const */
	ch0_parameters.dst_start = DES2_REGS_HW_ADDR + 0x24;
	ch0_parameters.dst_ei = DMA_CDEI_Default;
	ch0_parameters.dst_fi = DMA_CDFI_Default; /*source frame index */

	ch0_parameters.trigger = DMA_CCR_Mask_Channel(DMA_CCR_Channel_Mem2DES);
	ch0_parameters.sync_mode = 0x2; /*FS =1, BS=0 => An entire frame is
												  transferred once a DMA request is made */
	ch0_parameters.src_or_dst_synch = 0; /*Transfert is triggered by the Dest */

	dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: scxPublicDMASetParams(ch0)\n");
	scxPublicDMASetParams(dma_ch0, &ch0_parameters);

	dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: Start DMA channel %d\n", (unsigned int)dma_ch0);

	scxPublicDMAStart(dma_ch0, OMAP_DMA_DROP_IRQ);

	/*DMA2: DES -> Mem */
	ch1_parameters.data_type =	DMA_CSDP_Srce_Endian_little
		| DMA_CSDP_Srce_Endian_Lock_off
		| DMA_CSDP_Dest_Endian_little
		| DMA_CSDP_Dest_Endian_Lock_off
		| DMA_CSDP_Write_Mode_none_posted
		| DMA_CSDP_Dest_Burst_off
		| DMA_CSDP_Dest_packed_off
		| DMA_CSDP_WR_Add_Trslt
		| DMA_CSDP_Src_Burst_off
		| DMA_CSDP_Src_packed_off
		| DMA_CSDP_RD_Add_Trslt
		| DMA_CSDP_Data_32b;
	ch1_parameters.elem_count = DMA_CEN_Elts_per_Frame_DES;
	ch1_parameters.frame_count = nbBlocks;

	ch1_parameters.src_amode = 0; /*const */
	ch1_parameters.src_start = DES2_REGS_HW_ADDR + 0x24;
	ch1_parameters.src_ei = DMA_CSEI_Default;
	ch1_parameters.src_fi = DMA_CSFI_Default;

	ch1_parameters.dst_amode = 1; /*post increment */
	ch1_parameters.dst_start =  bufOut_phys;
	ch1_parameters.dst_ei = DMA_CDEI_Default;
	ch1_parameters.dst_fi = DMA_CDFI_Default; /*source frame index */

	ch1_parameters.trigger = DMA_CCR_Mask_Channel(DMA_CCR_Channel_DES2Mem);
	ch1_parameters.sync_mode = 0x2; /*FS =1, BS=0 => An entire frame is
				  transferred once a DMA request is made */
	ch1_parameters.src_or_dst_synch = 1; /*Transfert is triggered by the Src */

	dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: scxPublicDMASetParams(ch1)\n");
	scxPublicDMASetParams(dma_ch1, &ch1_parameters);

	if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
		scxPublicDMAEnableL3IRQ();
	}

	dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: Start DMA channel %d\n", (unsigned int)dma_ch1);
	scxPublicDMAStart(dma_ch1, OMAP_DMA_DROP_IRQ|OMAP_DMA_BLOCK_IRQ);

	/*
	 *The input data may be in the cache only,
	 * and the DMA is only working with physical addresses.
	 *So flush the cache to have data coherency.
	 */
	v7_dma_flush_range((u32)pSrc, (u32)(pSrc + nLength));

	/*Start operation */
	OUTREG32(&g_pDESReg_t->DES_MASK,
				INREG32(&g_pDESReg_t->DES_MASK) | DES_MASK_START_BIT);

	if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
		/*Suspends the process until the DMA IRQ occurs */
		dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: Waiting for IRQ\n");
		returnCode = scxPublicDMAWait();
	} else {
		dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: Polling DMA\n");
		returnCode = scxPublicDMAPoll(dma_ch1);
	}

	if (returnCode != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		dprintk(KERN_ERR "PDrvCryptoUpdateDESWithDMA: Timeout\n");
		/*Do not exit function but clear properly the operation */
	}

	if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
		/*Acknoledge DMA interrupt */
		scxPublicDMADisableL3IRQ();
	}

	scxPublicDMAClearChannel(dma_ch1);

	/*
	 *The dma transfert is complete
	 */

	/*Stop clocks */
	OUTREG32(&g_pDESReg_t->DES_MASK,
				INREG32(&g_pDESReg_t->DES_MASK)
				&(~DES_MASK_START_BIT));

	/*Unset DMA synchronisation requests */
	OUTREG32(&g_pDESReg_t->DES_MASK,
				INREG32(&g_pDESReg_t->DES_MASK)
				&(~DES_MASK_DMA_REQ_OUT_EN_BIT)
				&(~DES_MASK_DMA_REQ_IN_EN_BIT));

	if (returnCode != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: Error [0x%08x]\n", (unsigned int)returnCode);
		return returnCode;
	}

	/*
	 *The output data are in the physical memory.
	 *So invalidate the cache to have data coherency.
	 */
	v7_dma_inv_range((u32)pDest, (u32)(pDest + nLength));

	dprintk(KERN_INFO "PDrvCryptoUpdateDESWithDMA: Success\n");

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*
void debug_des_init_key(const u8 *pKey, U32 vSizeInBytes)
{
	u8 *pTempKey = (u8 *)pKey;

	OUTREG32(&g_pDESReg_t->DES_KEY1_L, 0x0);
	OUTREG32(&g_pDESReg_t->DES_KEY1_H, 0x0);
	OUTREG32(&g_pDESReg_t->DES_KEY2_L, 0x0);
	OUTREG32(&g_pDESReg_t->DES_KEY2_H, 0x0);
	OUTREG32(&g_pDESReg_t->DES_KEY3_L, 0x0);
	OUTREG32(&g_pDESReg_t->DES_KEY3_H, 0x0);

	OUTREG32(&g_pDESReg_t->DES_KEY1_L, BYTES_TO_LONG(pTempKey));  pTempKey += 4;
	OUTREG32(&g_pDESReg_t->DES_KEY1_H, BYTES_TO_LONG(pTempKey));  pTempKey += 4;

	if (vSizeInBytes >= 128) {
		OUTREG32(&g_pDESReg_t->DES_KEY2_L, BYTES_TO_LONG(pTempKey));  pTempKey += 4;
		OUTREG32(&g_pDESReg_t->DES_KEY2_H, BYTES_TO_LONG(pTempKey));  pTempKey += 4;
	}

	if (vSizeInBytes >= 192) {
		OUTREG32(&g_pDESReg_t->DES_KEY3_L, BYTES_TO_LONG(pTempKey));  pTempKey += 4;
		OUTREG32(&g_pDESReg_t->DES_KEY3_H, BYTES_TO_LONG(pTempKey));  pTempKey += 4;
	}
}


 */


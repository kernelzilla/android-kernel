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

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK
#include "scxlnx_benchmark.h"
#endif /*SMODULE_SMC_OMAP3430_BENCHMARK */


#define SMODULE_SMC_OMAP3430_PUBLIC_DMA

/*
 *The size limit to trigger DMA for AES, DES and Digest.
 *0xFFFFFFFF means "never"
 */
#ifdef SMODULE_SMC_OMAP3430_PUBLIC_DMA

#define DMA_TRIGGER_POLLING_AES			  128
#define DMA_TRIGGER_POLLING_DES			  128
#define DMA_TRIGGER_POLLING_DIGEST		  1024

#else

#define DMA_TRIGGER_POLLING_AES			  0xFFFFFFFF
#define DMA_TRIGGER_POLLING_DES			  0xFFFFFFFF
#define DMA_TRIGGER_POLLING_DIGEST		  0xFFFFFFFF

#endif

#define DMA_TRIGGER_IRQ_AES					0xFFFFFFFF
#define DMA_TRIGGER_IRQ_DES					0xFFFFFFFF
#define DMA_TRIGGER_IRQ_DIGEST				0xFFFFFFFF


#define S_SUCCESS						 0x00000000
#define S_ERROR_GENERIC				 0xFFFF0000
#define S_ERROR_ACCESS_DENIED		 0xFFFF0001
#define S_ERROR_BAD_FORMAT			 0xFFFF0005
#define S_ERROR_BAD_PARAMETERS		0xFFFF0006
#define S_ERROR_OUT_OF_MEMORY		 0xFFFF000C
#define S_ERROR_SHORT_BUFFER		  0xFFFF0010
#define S_ERROR_UNREACHABLE			0xFFFF3013
#define S_ERROR_SERVICE				 0xFFFF1000


#define CKR_OK										  0x00000000
#define CKR_ARGUMENTS_BAD							0x00000007
#define CKR_DATA_LEN_RANGE						  0x00000021
#define CKR_DEVICE_ERROR							 0x00000030
#define CKR_DEVICE_MEMORY							0x00000031
#define CKR_MECHANISM_INVALID					  0x00000070
#define CKR_TOKEN_NOT_PRESENT					  0x000000E0
#define CKR_BUFFER_TOO_SMALL						0x00000150

#define PUBLIC_CRYPTO_HW_CLOCK_ADDR1       (0x48004A10)
#define PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR1    (0x48004A30)
#define PUBLIC_CRYPTO_HW_CLOCK_ADDR2		 (0x48004A14)
#define PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR2	 (0x48004A34)

#define PUBLIC_CRYPTO_TIMEOUT_CONST		 (0x000FFFFF)

extern SCXLNX_DEVICE_MONITOR g_SCXLNXDeviceMonitor;

/*
 *RPC_CMD_PUBLIC_CRYPTO_INIT: IN and OUT structures
 */
typedef struct {
	u32	nReserved1;
	u32	hClientSession;
	u32	nReserved2;
	u32	nCommandID;
	u32	nReserved3;

	u32	nCryptoAlgId;		  /*PUBLIC_CRYPTO_ALG_XXX */

	CryptoInitContext cryptoInitCtx; /*For symmetric operations */
}
SCXLNX_RPC_CMD_PUBLIC_CRYPTO_INIT_IN;

typedef struct {
	u32	hPublicCryptoSession;
}
SCXLNX_RPC_CMD_PUBLIC_CRYPTO_INIT_OUT;


/*
 *RPC_CMD_PUBLIC_CRYPTO_UPDATE: IN structure only
 */
typedef struct {
	u32	hPublicCryptoSession;
	u32	nDataBuffer;
	u32	nDataLength;
	u32	nResultBuffer;
}
SCXLNX_RPC_CMD_PUBLIC_CRYPTO_UPDATE_IN;


/*
 *RPC_CMD_PUBLIC_CRYPTO_FINAL: IN and OUT structures
 */
typedef struct {
	u32	hPublicCryptoSession;
}
SCXLNX_RPC_CMD_PUBLIC_CRYPTO_FINAL_IN;

typedef struct {
	u8	 pDigestData[PUBLIC_CRYPTO_DIGEST_MAX_SIZE];
}
SCXLNX_RPC_CMD_PUBLIC_CRYPTO_FINAL_OUT;


/*
 *RPC_CMD_PUBLIC_CRYPTO_TERMINATE: IN structure only
 */
typedef struct {
	u32	hPublicCryptoSession;
}
SCXLNX_RPC_CMD_PUBLIC_CRYPTO_TERMINATE_IN;


/*Indicate the result buffer is NULL */
#define SERVICE_PKCS11_BUFFER_NULL	0

/*Indicate that the input and/or result buffers are mappable. */
#define SERVICE_PKCS11_BUFFER_MAPPABLE	 0x03


/*---------------------------------------------------------------------------- */

#define LIB_BEF_FLAG_VALUE_IMPLICIT	 0x06
#define LIB_BEF_BASE_TYPE_UINT32		 0x30

/*
 *Read a value on 4 bytes
 */
static u8 *static_befEncoderReadTU4(
		u8 *pDecoderArray,
		u32 *pnValue)
{
	 *pnValue = (((u32)(pDecoderArray[0])) |
					((u32)(pDecoderArray[1]) << 8) |
					((u32)(pDecoderArray[2]) << 16) |
					((u32)(pDecoderArray[3]) << 24));
	pDecoderArray += 4;

	return pDecoderArray;
}

/*
 *Read a uin32_t
 */
static u8 *static_befDecoderReadUint32(
		u8 *pDecoderArray,
		u8 *pDecoderArrayEnd,
		u32 *pnValue)
{
	/*BEF tag */
	if (pDecoderArray >= pDecoderArrayEnd) {
		return NULL;
	}

	if (*pDecoderArray != (LIB_BEF_BASE_TYPE_UINT32 | LIB_BEF_FLAG_VALUE_IMPLICIT)) {
		return NULL;
	}

	if (pDecoderArray + 5 > pDecoderArrayEnd) {
		return NULL;
	}

	pDecoderArray++;
	pDecoderArray = static_befEncoderReadTU4(pDecoderArray, pnValue);

	return pDecoderArray;
}

/*
 *Write a value on 4 bytes
 */
static u8 *static_befEncoderWriteTU4(
		u8 *pEncoderArray,
		u32	nValue)
{
	 *pEncoderArray++ = (u8)(nValue);
	 *pEncoderArray++ = (u8)((nValue >> 8) & 0xFF);
	 *pEncoderArray++ = (u8)((nValue >> 16) & 0xFF);
	 *pEncoderArray++ = (u8)((nValue >> 24) & 0xFF);

	return pEncoderArray;
}

/*
 *Write a uin32_t
 */
static u8 *static_befEncoderWriteUint32(
		u8 *pEncoderArray,
		u32	nValue)
{
	 *pEncoderArray++ = LIB_BEF_BASE_TYPE_UINT32 | LIB_BEF_FLAG_VALUE_IMPLICIT;
	pEncoderArray = static_befEncoderWriteTU4(pEncoderArray, nValue);

	return pEncoderArray;
}


#ifdef SMODULE_SMC_OMAP3430_POWER_MANAGEMENT
void scxPublicCryptoPowerManagementTimerCbk(unsigned long aPtr)
{
	SCXLNX_SM_COMM_MONITOR *pSMComm = (SCXLNX_SM_COMM_MONITOR *)aPtr;

	dprintk("scxPublicCryptoPowerManagementTimerCbk try lock\n");
	if (down_trylock(&(pSMComm->pubcrypto.HWALock)) == 0) {
		dprintk("scxPublicCryptoPowerManagementTimerCbk begin\n");
		/*We disable all the clocks to go in power safe */
		scxPublicCryptoDisableHwClock(PUBLIC_CRYPTO_AES_CLOCK_BIT);
		scxPublicCryptoDisableHwClock(PUBLIC_CRYPTO_DES_CLOCK_BIT);
		scxPublicCryptoDisableHwClock(PUBLIC_CRYPTO_SHA_CLOCK_BIT);
		/*We reset the current session so that we'll reinstall the key */
		pSMComm->pubcrypto.pCurrentSession = NULL;
		dprintk("scxPublicCryptoPowerManagementTimerCbk end\n");
		up(&(pSMComm->pubcrypto.HWALock));
	} else {
		dprintk("scxPublicCryptoPowerManagementTimerCbk will wait one more time\n");
		mod_timer(&pSMComm->pubcrypto.pPowerManagementTimer, jiffies+msecs_to_jiffies(pSMComm->nInactivityTimerExpireMs));
	}
}
#endif /*SMODULE_SMC_OMAP3430_POWER_MANAGEMENT */


/*
 *Convert a S_ERROR into a PKCS11 error (CKR_)
 */
uint32_t /*CK_RV */ scxPublicCryptoConvertOperationError(uint32_t /*S_RESULT */ errorCode)
{
	switch (errorCode) {
	case S_SUCCESS:
		return CKR_OK;

	case S_ERROR_SHORT_BUFFER:
		return CKR_BUFFER_TOO_SMALL;
	case S_ERROR_BAD_PARAMETERS:
		return CKR_ARGUMENTS_BAD;
	case S_ERROR_OUT_OF_MEMORY:
		return CKR_DEVICE_MEMORY;
	case S_ERROR_ACCESS_DENIED:
		return CKR_TOKEN_NOT_PRESENT;
	case S_ERROR_BAD_FORMAT:
		return CKR_DATA_LEN_RANGE;
	case S_ERROR_UNREACHABLE:
		break;
	}

	return CKR_DEVICE_ERROR;
}

/*---------------------------------------------------------------------------- */
/* *
 *Initialize the public crypto (including DMA)
 */
u32 scxPublicCryptoInit(SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto)
{
	u32 nError = PUBLIC_CRYPTO_OPERATION_SUCCESS;

#ifdef SMODULE_SMC_OMAP3430_PUBLIC_DMA

	int nChannel;

	nError = scxPublicDMAInit();
	if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		goto end;
	}

	/*
	 * Allocate 4 DMA channels
	 */
	for (nChannel = OMAP3430_SMC_DMA_CH_0; nChannel < OMAP3430_SMC_DMA_CH_0 + OMAP3430_SMC_DMA_CH_NB; nChannel++) {
		nError = scxPublicDMARequest(nChannel);
		if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			goto end;
		}
	}

end:

#endif /*SMODULE_SMC_OMAP3430_PUBLIC_DMA */

	init_MUTEX_LOCKED(&(pPubCrypto->HWALock));

	return nError;
}

/*---------------------------------------------------------------------------- */
/* *
 *Terminate the public crypto (including DMA)
 */
void scxPublicCryptoTerminate(SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto)
{
#ifdef SMODULE_SMC_OMAP3430_PUBLIC_DMA

	int nChannel;

	/*
	 *Release the 4 DMA channels
	 */
	for (nChannel = OMAP3430_SMC_DMA_CH_0; nChannel < OMAP3430_SMC_DMA_CH_0 + OMAP3430_SMC_DMA_CH_NB; nChannel++) {
		scxPublicDMARelease(nChannel);
	}

	scxPublicDMATerminate();

#endif /*SMODULE_SMC_OMAP3430_PUBLIC_DMA */
}

/*---------------------------------------------------------------------------- */
/*
 *Perform a crypto update operation.
 *THIS FUNCTION IS CALLED FROM THE RPC HANDLER.
 */
static u32 scxPublicCryptoUpdate(
		SCX_PUBLIC_CRYPTO_DESC * pSession,
		bool bDoRestoreAESDESRegisters)
{
	u32 nError = S_SUCCESS;
	u32 dmaUse = PUBLIC_CRYPTO_DMA_USE_NONE;

	dprintk(KERN_INFO "scxPublicCryptoUpdate(%p) : AlgId=0x%x, In=%p/%u, Out=%p/%u, Restore=%d\n",
						pSession, pSession->nCryptoAlgId,
						pSession->pDataBuffer, pSession->nDataLength,
						pSession->pResultBuffer, pSession->nResultLength,
						bDoRestoreAESDESRegisters);

	pSession->nResultActualLength = 0;

	switch (pSession->nCryptoAlgId) {
	case PUBLIC_CRYPTO_ALG_CIPHER_AES:
		if (pSession->nDataLength % AES_BLOCK_SIZE != 0) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : Bad input length [%u]\n",
						pSession, pSession->nDataLength);
			return S_ERROR_BAD_FORMAT;
		}

		pSession->nResultActualLength = pSession->nDataLength;

		if (pSession->pResultBuffer == NULL) {
			/*Only the result length is required */
			return S_SUCCESS;
		}

		if (pSession->nResultLength < pSession->nDataLength) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : Short result length [%u]\n",
						pSession, pSession->nResultLength);
			return S_ERROR_SHORT_BUFFER;
		}

      scxPublicCryptoEnableClock(CRYPTO_ACCELERATOR_AES1);

		/*Restore registers */
		if (bDoRestoreAESDESRegisters) {
			nError = PDrvCryptoRestoreAESRegisters((PUBLIC_CRYPTO_AES_CONTEXT *)pSession->pCryptoData);
			if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
				dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoRestoreAESRegisters failed [%u]\n",
							pSession, nError);
				return S_ERROR_UNREACHABLE;
			}
		}

		/*
		 *Choice of the processing type
		 */
		if (pSession->nDataLength >= DMA_TRIGGER_POLLING_AES) {
			if (pSession->nDataLength >= DMA_TRIGGER_IRQ_AES) {
				dmaUse = PUBLIC_CRYPTO_DMA_USE_IRQ;
			} else {
				dmaUse = PUBLIC_CRYPTO_DMA_USE_POLLING;
			}
		}

		/*Process Data */
		nError = PDrvCryptoUpdateAES(
				(PUBLIC_CRYPTO_AES_CONTEXT *)pSession->pCryptoData,
				pSession->pDataBuffer,
				pSession->pResultBuffer,
				pSession->nDataLength / AES_BLOCK_SIZE,
				dmaUse);
		if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoUpdateAES failed [%u]\n",
						pSession, nError);
			return S_ERROR_UNREACHABLE;
		}

		/*Save registers */
		nError = PDrvCryptoSaveAESRegisters((PUBLIC_CRYPTO_AES_CONTEXT *)pSession->pCryptoData);
		if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoSaveAESRegisters failed [%u]\n",
						pSession, nError);
			return S_ERROR_UNREACHABLE;
		}

	break;

	case PUBLIC_CRYPTO_ALG_CIPHER_DES:
	case PUBLIC_CRYPTO_ALG_CIPHER_3DES:
		if (pSession->nDataLength % DES_BLOCK_SIZE != 0) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : Bad input length [%u]\n",
						pSession, pSession->nDataLength);
			return S_ERROR_BAD_FORMAT;
		}

		pSession->nResultActualLength = pSession->nDataLength;

		if (pSession->pResultBuffer == NULL) {
			/*Only the result length is required */
			return S_SUCCESS;
		}

		if (pSession->nResultLength < pSession->nDataLength) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : Short result length [%u]\n",
						pSession, pSession->nResultLength);
			return S_ERROR_SHORT_BUFFER;
		}

      scxPublicCryptoEnableClock(CRYPTO_ACCELERATOR_DES2);

		/*Restore registers */
		if (bDoRestoreAESDESRegisters) {
			nError = PDrvCryptoRestoreDESRegisters((PUBLIC_CRYPTO_DES_DES3_CONTEXT *)pSession->pCryptoData);
			if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
				dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoRestoreDESRegisters failed [%u]\n",
							pSession, nError);
				return S_ERROR_UNREACHABLE;
			}
		}

		/*
		 *Choice of the processing type
		 */
		if (pSession->nDataLength >= DMA_TRIGGER_POLLING_DES) {
			if (pSession->nDataLength >= DMA_TRIGGER_IRQ_DES) {
				dmaUse = PUBLIC_CRYPTO_DMA_USE_IRQ;
			} else {
				dmaUse = PUBLIC_CRYPTO_DMA_USE_POLLING;
			}
		}

		/*Process Data */
		nError = PDrvCryptoUpdateDES(
				(PUBLIC_CRYPTO_DES_DES3_CONTEXT *)pSession->pCryptoData,
				pSession->pDataBuffer,
				pSession->pResultBuffer,
				pSession->nDataLength / DES_BLOCK_SIZE,
				dmaUse);
		if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoUpdateDES failed [%u]\n",
						pSession, nError);
			return S_ERROR_UNREACHABLE;
		}

		/*Save registers */
		nError = PDrvCryptoSaveDESRegisters((PUBLIC_CRYPTO_DES_DES3_CONTEXT *)pSession->pCryptoData);
		if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoSaveDESRegisters failed [%u]\n",
						pSession, nError);
			return S_ERROR_UNREACHABLE;
		}

	break;

	case PUBLIC_CRYPTO_ALG_DIGEST_MD5:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA1:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA224:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA256:

      scxPublicCryptoEnableClock(CRYPTO_ACCELERATOR_SHA1);

		/*Restore registers */
		nError = PDrvCryptoRestoreHashRegisters((PUBLIC_CRYPTO_HASH_CONTEXT *)pSession->pCryptoData);
		if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoRestoreHashRegisters failed [%u]\n",
						pSession, nError);
			return S_ERROR_UNREACHABLE;
		}

		/*
		 *Choice of the processing type
		 */
		if (pSession->nDataLength >= DMA_TRIGGER_POLLING_DIGEST) {
			if (pSession->nDataLength >= DMA_TRIGGER_IRQ_DIGEST) {
				dmaUse = PUBLIC_CRYPTO_DMA_USE_IRQ;
			} else {
				dmaUse = PUBLIC_CRYPTO_DMA_USE_POLLING;
			}
		}

		/*Process Data */
		nError = PDrvCryptoUpdateHash(
				(PUBLIC_CRYPTO_HASH_CONTEXT *)pSession->pCryptoData,
				pSession->pDataBuffer,
				pSession->nDataLength,
				dmaUse);
		if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoUpdateHash failed [%u]\n",
						pSession, nError);
			return S_ERROR_UNREACHABLE;
		}

		/*Save registers */
		nError = PDrvCryptoSaveHashRegisters((PUBLIC_CRYPTO_HASH_CONTEXT *)pSession->pCryptoData);
		if (nError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			dprintk(KERN_ERR "scxPublicCryptoUpdate(%p) : PDrvCryptoSaveHashRegisters failed [%u]\n",
						pSession, nError);
			return S_ERROR_UNREACHABLE;
		}

		break;

	default:
		BUG_ON(1);
		break;
	}

#ifdef SMODULE_SMC_OMAP3430_POWER_MANAGEMENT
	scxPublicCryptoStartInactivityTimer();
#endif
	return S_SUCCESS;
}


/*---------------------------------------------------------------------------- */
/*
 * A new public crypto session is initialized and added in the list.
 * And the crypto operation is initialized.
 *THIS FUNCTION IS CALLED FROM THE RPC HANDLER.
 */
u32 scxPublicCryptoRPCInit(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer)
{
	SCXLNX_RPC_CMD_PUBLIC_CRYPTO_INIT_IN *pPubCryptoInitIn;
	SCXLNX_RPC_CMD_PUBLIC_CRYPTO_INIT_OUT *pPubCryptoInitOut;

	SCX_PUBLIC_CRYPTO_DESC *pSession = NULL;
	u32 nRPCError = PUBLIC_CRYPTO_OPERATION_SUCCESS;

	pPubCryptoInitIn = (SCXLNX_RPC_CMD_PUBLIC_CRYPTO_INIT_IN *)pL0SharedBuffer;
	pPubCryptoInitOut = (SCXLNX_RPC_CMD_PUBLIC_CRYPTO_INIT_OUT *)pL0SharedBuffer;

	dprintk(KERN_INFO "scxPublicCryptoRPCInit: Session=0x%x, CmdID=0x%x, AlgId=0x%x\n",
				pPubCryptoInitIn->hClientSession, pPubCryptoInitIn->nCommandID, pPubCryptoInitIn->nCryptoAlgId);

	pSession = (SCX_PUBLIC_CRYPTO_DESC *)internal_kmalloc(sizeof(*pSession), GFP_KERNEL);
	if (pSession == NULL) {
		dprintk(KERN_ERR "scxPublicCryptoRPCInit: Out of memory for public session\n");
		nRPCError = PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY;
		goto error;
	}

	memset(pSession, 0, sizeof(*pSession));

	pSession->nMagicNumber = SCX_PUBLIC_CRYPTO_DESC_MAGIC;
	pSession->hClientSession = pPubCryptoInitIn->hClientSession;
	pSession->nCommandID = pPubCryptoInitIn->nCommandID;
	pSession->nCryptoAlgId = pPubCryptoInitIn->nCryptoAlgId;

	/*Pre-allocate a buffer */
	pSession->nInnerBufferLength = PAGE_SIZE;
	pSession->pInnerBuffer = (u8 *)internal_get_zeroed_page(GFP_KERNEL);
	if (pSession->pInnerBuffer == NULL) {
		dprintk(KERN_ERR "scxPublicCryptoRPCInit(%p) : Out of memory for inner buffer\n",
					pSession);
		nRPCError = PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY;
		goto error;
	}
	SET_PAGE_LOCKED(virt_to_page(pSession->pInnerBuffer));

	/*
	 *Initialize the session
	 */
	switch (pSession->nCryptoAlgId) {
	case PUBLIC_CRYPTO_ALG_CIPHER_AES:
		pSession->nCryptoDataLength = sizeof(PUBLIC_CRYPTO_AES_CONTEXT);
		pSession->pCryptoData = internal_kmalloc(pSession->nCryptoDataLength, GFP_KERNEL);
		if (pSession->pCryptoData == NULL) {
			dprintk(KERN_ERR "scxPublicCryptoRPCInit(%p) : Out of memory for AES context (%u bytes)\n",
						pSession, pSession->nCryptoDataLength);
			nRPCError = PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY;
			goto error;
		}

		dprintk(KERN_INFO "pPubCrypto->nAesSession(%u)\n", pPubCrypto->nAesSession);
		pPubCrypto->nAesSession++;

		/*
		 *Intializes the context
		 */
		PDrvCryptoInitAES(&(pPubCryptoInitIn->cryptoInitCtx),
								(PUBLIC_CRYPTO_AES_CONTEXT *)pSession->pCryptoData);
		break;

	case PUBLIC_CRYPTO_ALG_CIPHER_DES:
	case PUBLIC_CRYPTO_ALG_CIPHER_3DES:
		pSession->nCryptoDataLength = sizeof(PUBLIC_CRYPTO_DES_DES3_CONTEXT);
		pSession->pCryptoData = internal_kmalloc(pSession->nCryptoDataLength, GFP_KERNEL);
		if (pSession->pCryptoData == NULL) {
			dprintk(KERN_ERR "scxPublicCryptoRPCInit(%p) : Out of memory for DES context (%u bytes)\n",
						pSession, pSession->nCryptoDataLength);
			nRPCError = PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY;
			goto error;
		}

		dprintk(KERN_INFO "pPubCrypto->nDesSession(%u)\n", pPubCrypto->nDesSession);
		pPubCrypto->nDesSession++;

		/*
		 *Intializes the context
		 */
		PDrvCryptoInitDES(&(pPubCryptoInitIn->cryptoInitCtx),
								(PUBLIC_CRYPTO_DES_DES3_CONTEXT *)pSession->pCryptoData);
		break;

	case PUBLIC_CRYPTO_ALG_DIGEST_MD5:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA1:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA224:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA256:
		pSession->nCryptoDataLength = sizeof(PUBLIC_CRYPTO_HASH_CONTEXT);
		pSession->pCryptoData = internal_kmalloc(pSession->nCryptoDataLength, GFP_KERNEL);
		if (pSession->pCryptoData == NULL) {
			dprintk(KERN_ERR "scxPublicCryptoRPCInit(%p) : Out of memory for Digest context (%u bytes)\n",
						pSession, pSession->nCryptoDataLength);
			nRPCError = PUBLIC_CRYPTO_ERR_OUT_OF_MEMORY;
			goto error;
		}

		dprintk(KERN_INFO "pPubCrypto->nShaSession(%u)\n", pPubCrypto->nShaSession);
		pPubCrypto->nShaSession++;

		PDrvCryptoInitHash(pSession->nCryptoAlgId,
								 (PUBLIC_CRYPTO_HASH_CONTEXT *)pSession->pCryptoData);
		break;

	default:
		dprintk(KERN_ERR "scxPublicCryptoRPCInit(%p) : Bad CryptoAlgId [0x%x]\n",
					pSession, pSession->nCryptoAlgId);
		nRPCError = PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
		goto error;
	}

	/*
	 * Add the structure in the list
	 */
	list_add(&(pSession->list), &(pPubCrypto->sessions));

	pPubCryptoInitOut->hPublicCryptoSession = (u32)pSession;
	/*
	 *This public session is now ready to be intercepted,
	 *but the key is not installed in the PA yet.
	 *This will only apply for symetric operations.
	 */
	pPubCrypto->pCurrentSession = NULL;

	dprintk(KERN_INFO "scxPublicCryptoRPCInit(%p) : Session=0x%x [created]\n",
				pSession, pSession->hClientSession);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;

error:
	if (pSession != NULL) {
		if (pSession->pInnerBuffer != NULL) {
			CLEAR_PAGE_LOCKED(virt_to_page(pSession->pInnerBuffer));
			internal_free_page((unsigned long)pSession->pInnerBuffer);
		}
		internal_kfree(pSession->pCryptoData);
		internal_kfree(pSession);
	}

	return nRPCError;
}

/*---------------------------------------------------------------------------- */

/*
 *Perform a crypto update operation.
 *THIS FUNCTION IS CALLED FROM THE RPC HANDLER.
 */
u32 scxPublicCryptoRPCUpdate(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer)
{
	SCXLNX_RPC_CMD_PUBLIC_CRYPTO_UPDATE_IN *pPubCryptoUpdateIn;
	SCX_PUBLIC_CRYPTO_DESC *pSession;

	u32 nRPCError = PUBLIC_CRYPTO_OPERATION_SUCCESS;

	pPubCryptoUpdateIn = (SCXLNX_RPC_CMD_PUBLIC_CRYPTO_UPDATE_IN *)pL0SharedBuffer;

	/*
	 *Retrieve the crypto session
	 */
	pSession = (SCX_PUBLIC_CRYPTO_DESC *)pPubCryptoUpdateIn->hPublicCryptoSession;

	if ((pSession == NULL) || (pSession->nMagicNumber != SCX_PUBLIC_CRYPTO_DESC_MAGIC)) {
		dprintk(KERN_ERR "scxPublicCryptoRPCUpdate(%p) : Bad session handle\n",
					pSession);
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}

	dprintk(KERN_INFO "scxPublicCryptoRPCUpdate(%p) : Session=0x%x\n",
				pSession, pSession->hClientSession);

	/*TODO: Check the buffer from the PA */

	/*
	 *We restore the registers as we 've just set the key in the PA.
	 *This is only applicable for AES and DES.
	 */
	nRPCError = scxPublicCryptoUpdate(pSession, true);
	if (nRPCError != S_SUCCESS) {
		dprintk(KERN_ERR "scxPublicCryptoRPCUpdate(%p) : scxPublicCryptoUpdate failed [0x%x]\n",
					pSession, nRPCError);
	}

	/*
	 *This public session is now ready to be intercepted,
	 * as we have just installed the key in the PA.
	 */
	pPubCrypto->pCurrentSession = pSession;

	return nRPCError;
}

/*---------------------------------------------------------------------------- */

/*
 *Perform a crypto update operation.
 *THIS FUNCTION IS CALLED FROM THE RPC HANDLER.
 */
u32 scxPublicCryptoRPCEnableClock(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer)
{
	u32 nRPCError = PUBLIC_CRYPTO_OPERATION_SUCCESS;
	SCXLNX_RPC_CMD_PUBLIC_CRYPTO_UPDATE_IN *pPubCryptoUpdateIn;
	SCX_PUBLIC_CRYPTO_DESC *pSession;

	pPubCryptoUpdateIn = (SCXLNX_RPC_CMD_PUBLIC_CRYPTO_UPDATE_IN *)pL0SharedBuffer;
	/*
	 *Retrieve the crypto session
	 */
	pSession = (SCX_PUBLIC_CRYPTO_DESC *)pPubCryptoUpdateIn->hPublicCryptoSession;

	if ((pSession == NULL) || (pSession->nMagicNumber != SCX_PUBLIC_CRYPTO_DESC_MAGIC)) {
		dprintk(KERN_ERR "scxPublicCryptoRPCEnableClock(%p) : Bad session handle\n",
					pSession);
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}

	if (pSession->nCryptoAlgId == PUBLIC_CRYPTO_ALG_CIPHER_AES) {
      scxPublicCryptoEnableClock(CRYPTO_ACCELERATOR_AES1);
	} else if ((pSession->nCryptoAlgId == PUBLIC_CRYPTO_ALG_CIPHER_DES) || (pSession->nCryptoAlgId == PUBLIC_CRYPTO_ALG_CIPHER_3DES)) {
      scxPublicCryptoEnableClock(CRYPTO_ACCELERATOR_DES2);
	}

	/*Invalid the previous session */
	pPubCrypto->pCurrentSession = NULL;

	return nRPCError;
}

/*---------------------------------------------------------------------------- */

/*
 *Perform a crypto final operation (for digest only).
 *THIS FUNCTION IS CALLED FROM THE RPC HANDLER.
 */
u32 scxPublicCryptoRPCFinal(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer)
{
	SCXLNX_RPC_CMD_PUBLIC_CRYPTO_FINAL_IN *pPubCryptoFinalIn;
	SCXLNX_RPC_CMD_PUBLIC_CRYPTO_FINAL_OUT *pPubCryptoFinalOut;
	SCX_PUBLIC_CRYPTO_DESC *pSession;

	u32 nRPCError = PUBLIC_CRYPTO_OPERATION_SUCCESS;

	pPubCryptoFinalIn = (SCXLNX_RPC_CMD_PUBLIC_CRYPTO_FINAL_IN *)pL0SharedBuffer;
	pPubCryptoFinalOut = (SCXLNX_RPC_CMD_PUBLIC_CRYPTO_FINAL_OUT *)pL0SharedBuffer;

	/*
	 *Retrieve the crypto session
	 */
	pSession = (SCX_PUBLIC_CRYPTO_DESC *)pPubCryptoFinalIn->hPublicCryptoSession;

	if ((pSession == NULL) || (pSession->nMagicNumber != SCX_PUBLIC_CRYPTO_DESC_MAGIC)) {
		dprintk(KERN_ERR "scxPublicCryptoRPCFinal(%p) : Bad session handle\n",
					pSession);
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}

	dprintk(KERN_INFO "scxPublicCryptoRPCFinal(%p) : Session=0x%x\n",
				pSession, pSession->hClientSession);

	switch (pSession->nCryptoAlgId) {
	case PUBLIC_CRYPTO_ALG_CIPHER_AES:
	case PUBLIC_CRYPTO_ALG_CIPHER_DES:
	case PUBLIC_CRYPTO_ALG_CIPHER_3DES:
		/*Nothing to do */
		break;

	case PUBLIC_CRYPTO_ALG_DIGEST_MD5:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA1:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA224:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA256:

		/*We have to enable the clocks here because the inactivity timer
		 *may have disabled them.
		 */
      scxPublicCryptoEnableClock(CRYPTO_ACCELERATOR_SHA1);

		/*Restore registers */
		nRPCError = PDrvCryptoRestoreHashRegisters((PUBLIC_CRYPTO_HASH_CONTEXT *)pSession->pCryptoData);
		if (nRPCError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
			break;
		}

			/*Process Data */
		nRPCError = PDrvCryptoFinalHash(
				(PUBLIC_CRYPTO_HASH_CONTEXT *)pSession->pCryptoData,
				pPubCryptoFinalOut->pDigestData,
				0);
		break;

	default:
		BUG_ON(1);
		break;
	}

	if (nRPCError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		dprintk(KERN_ERR "scxPublicCryptoRPCFinal(%p) : PDrvCryptoFinalXxx failed [%u]\n",
					pSession, nRPCError);
	}

	return nRPCError;
}

/*---------------------------------------------------------------------------- */

/*
 * An existing public crypto session is finalized, released and removed from the list.
 *THIS FUNCTION IS CALLED FROM THE RPC HANDLER.
 */
u32 scxPublicCryptoRPCTerminate(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		void *pL0SharedBuffer)
{
	SCXLNX_RPC_CMD_PUBLIC_CRYPTO_TERMINATE_IN *pPubCryptoTerminateIn;
	SCX_PUBLIC_CRYPTO_DESC *pSession;

	u32 nRPCError = PUBLIC_CRYPTO_OPERATION_SUCCESS;

	pPubCryptoTerminateIn = (SCXLNX_RPC_CMD_PUBLIC_CRYPTO_TERMINATE_IN *)pL0SharedBuffer;

	/*
	 *Retrieve the crypto session
	 */
	pSession = (SCX_PUBLIC_CRYPTO_DESC *)pPubCryptoTerminateIn->hPublicCryptoSession;

	if ((pSession == NULL) || (pSession->nMagicNumber != SCX_PUBLIC_CRYPTO_DESC_MAGIC)) {
		dprintk(KERN_ERR "scxPublicCryptoRPCTerminate(%p) : Bad session handle\n",
					pSession);
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}

	switch (pSession->nCryptoAlgId) {
	case PUBLIC_CRYPTO_ALG_CIPHER_AES:
		nRPCError = PDrvCryptoTerminateAES((PUBLIC_CRYPTO_AES_CONTEXT *)pSession->pCryptoData);
		dprintk(KERN_INFO "pPubCrypto->nAesSession(%u)\n", pPubCrypto->nAesSession);
		if (pPubCrypto->nAesSession == 1) {
         scxPublicCryptoDisableClock(CRYPTO_ACCELERATOR_AES1);
			dprintk(KERN_INFO "\n\n\n\n DISABLE AES \n\n\n\n");
		}
		pPubCrypto->nAesSession--;
		break;

	case PUBLIC_CRYPTO_ALG_CIPHER_DES:
	case PUBLIC_CRYPTO_ALG_CIPHER_3DES:
		nRPCError = PDrvCryptoTerminateDES((PUBLIC_CRYPTO_DES_DES3_CONTEXT *)pSession->pCryptoData);
		dprintk(KERN_INFO "pPubCrypto->nDesSession(%u)\n", pPubCrypto->nDesSession);
		if (pPubCrypto->nDesSession == 1) {
         scxPublicCryptoDisableClock(CRYPTO_ACCELERATOR_DES2);
			dprintk(KERN_INFO "\n\n\n\n DISABLE DES \n\n\n\n");
		}
		pPubCrypto->nDesSession--;
		break;

	case PUBLIC_CRYPTO_ALG_DIGEST_MD5:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA1:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA224:
	case PUBLIC_CRYPTO_ALG_DIGEST_SHA256:
		nRPCError = PDrvCryptoTerminateHash((PUBLIC_CRYPTO_HASH_CONTEXT *)pSession->pCryptoData);
		dprintk(KERN_INFO "pPubCrypto->nShaSession(%u)\n", pPubCrypto->nShaSession);
		if (pPubCrypto->nShaSession == 1) {
         scxPublicCryptoDisableClock(CRYPTO_ACCELERATOR_SHA1);
			dprintk(KERN_INFO "\n\n\n\n DISABLE SHA \n\n\n\n");
		}
		pPubCrypto->nShaSession--;
		break;

	default:
		BUG_ON(1);
		break;
	}

	if (nRPCError != PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		dprintk(KERN_ERR "scxPublicCryptoRPCTerminate(%p) : PDrvCryptoTerminateXxx failed [%u]\n",
					pSession, nRPCError);
		return nRPCError;
	}

	dprintk(KERN_INFO "scxPublicCryptoRPCTerminate(%p) : Session=0x%x [deleted]\n",
				pSession, pSession->hClientSession);

	/*Remove the public session from the list */
	list_del(&(pSession->list));

	/*Release all resources associated with the public session */
	CLEAR_PAGE_LOCKED(virt_to_page(pSession->pInnerBuffer));
	internal_free_page((unsigned long)pSession->pInnerBuffer);
	internal_kfree(pSession->pCryptoData);
	internal_kfree(pSession);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}


/*---------------------------------------------------------------------------- */

/*
 *Check if the command must be intercepted or not.
 *THIS FUNCTION IS CALLED FROM THE USER THREAD (ioctl).
 */
SCX_PUBLIC_CRYPTO_DESC *scxPublicCryptoCommandMatchAndCheck(
		SCX_PUBLIC_CRYPTO_MONITOR *pPubCrypto,
		SCX_COMMAND_INVOKE_CLIENT_COMMAND *pCommand,
		bool *pbIsSessionCorrupted)
{
	SCX_PUBLIC_CRYPTO_DESC *pSession = NULL;

	dprintk(KERN_INFO "scxPublicCryptoCommandMatchAndCheck: CltSession=0x%x, CmdID=0x%x\n",
				pCommand->hClientSession, pCommand->nClientCommandIdentifier);

	 *pbIsSessionCorrupted = false;

	list_for_each_entry(pSession, &(pPubCrypto->sessions), list)
	{
		/*
		 *The hClientSession is unique across all device contexts.
		 *So, we don't need to check the device context handle.
		 * Actually, it is not set when creating the public session.
		 */
		if ((pSession->hClientSession == pCommand->hClientSession) && (pSession->nCommandID == pCommand->nClientCommandIdentifier)) {
				/*
				 *The command matches one public session
				 */
				if ((pSession != pPubCrypto->pCurrentSession) && ((pSession->nCryptoAlgId == PUBLIC_CRYPTO_ALG_CIPHER_AES) || (pSession->nCryptoAlgId == PUBLIC_CRYPTO_ALG_CIPHER_DES) ||
							(pSession->nCryptoAlgId == PUBLIC_CRYPTO_ALG_CIPHER_3DES))) {
					/*
					 *However, this looks to be another public session as
					 *the previous one, so set it as corrupted, and
					 *don't process the command.
					 *This must be done only for symmetric operations.
					 */
					dprintk(KERN_INFO "scxPublicCryptoCommandMatchAndCheck: Match found but corruption[%p/%p]\n",
								pSession, pPubCrypto->pCurrentSession);
					pPubCrypto->pCurrentSession = NULL;
					 *pbIsSessionCorrupted = true;

					return pSession;
				}

				dprintk(KERN_INFO "scxPublicCryptoCommandMatchAndCheck: Match found [%p]\n",
							pSession);
				return pSession;
			}
	}

	/*
	 *No match, so we can say that the current public crypto session (if any)
	 *is being corrupted, i.e. the key will have to be re-installed afterward.
	 */
	pPubCrypto->pCurrentSession = NULL;

	dprintk(KERN_INFO "scxPublicCryptoCommandMatchAndCheck: No match\n");

	return NULL;
}

/*---------------------------------------------------------------------------- */

/*
 *Pre-process the client command (crypto update operation),
 *i.e. allocate and copy the user input buffer into a local buffer.
 *THIS FUNCTION IS CALLED FROM THE USER THREAD (ioctl).
 */
int scxPublicCryptoPreProcessCommand(
		SCXLNX_SHMEM_DESC * pDeviceContextDesc,
		SCX_PUBLIC_CRYPTO_DESC *pSession,
		SCX_COMMAND_INVOKE_CLIENT_COMMAND *pCommand,
		SCX_ANSWER_INVOKE_CLIENT_COMMAND *pAnswer)
{
	/*
	 *We need to decode the input BEF buffer first.
	 *The BEF buffer is in the master heap starting at the
	 *{nClientParameterStartOffset} offset.
	 */
	u8 *pC2SDataBuffer;
	u8 *pC2SDataBufferEnd;
	u32	nS2CBufferSize = 0;

	u32	nDataBufferType;
	u8 *pDataBuffer;
	u32	nDataLength;

	u32	nResultBufferType;
	u8 *pResultBuffer = NULL;
	u32	nResultLength = 0;

	u32	nUserAddr;
	u32	nError = 0;

	dprintk(KERN_INFO "scxPublicCryptoPreProcessCommand(%p) : Session=0x%x\n",
				pSession, pSession->hClientSession);

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK
	SCXBenchmark(NULL);
#endif /*SMODULE_SMC_OMAP3430_BENCHMARK */

	pSession->nResultActualLength = 0xFFFFFFFF;

	/*Check the input message offset */
	if (pCommand->nClientParameterSize == 0) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad message offset\n", pSession);
		nError = S_ERROR_GENERIC;
		goto framework_error;
	}

	pC2SDataBuffer = pDeviceContextDesc->pAllocatedBuffer +
			pCommand->nClientParameterStartOffset;
	pC2SDataBufferEnd = pC2SDataBuffer + pCommand->nClientParameterSize;

	/*u32: Input buffer type */
	pC2SDataBuffer = static_befDecoderReadUint32(
								pC2SDataBuffer, pC2SDataBufferEnd, &nDataBufferType);
	if (pC2SDataBuffer == NULL) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad message [Input buffer type]\n",
					pSession);
		nError = S_ERROR_GENERIC;
		goto framework_error;
	}

	if (nDataBufferType != SERVICE_PKCS11_BUFFER_MAPPABLE) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad Input buffer type [0x%X]\n",
					pSession, nDataBufferType);
		nError = S_ERROR_GENERIC;
		goto framework_error;
	}

	/*u32: Input buffer length */
	pC2SDataBuffer = static_befDecoderReadUint32(
								pC2SDataBuffer, pC2SDataBufferEnd, &nDataLength);
	if (pC2SDataBuffer == NULL) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad message [Input buffer length]\n",
					pSession);
		nError = S_ERROR_GENERIC;
		goto framework_error;
	}

	/*u32: Input buffer virtual address */
	pC2SDataBuffer = static_befDecoderReadUint32(
								pC2SDataBuffer, pC2SDataBufferEnd, &nUserAddr);
	if (pC2SDataBuffer == NULL) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad message [Input buffer addr]\n",
					pSession);
		nError = S_ERROR_GENERIC;
		goto framework_error;
	}

	pDataBuffer = (u8 *)nUserAddr;
	if (pDataBuffer == NULL) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Invalid input client buffer [0x%X, %u]\n",
					pSession, nUserAddr, nDataLength);
		nError = S_ERROR_GENERIC;
		goto framework_error;
	}

	pSession->pS2CDataBuffer = NULL;

	if (pC2SDataBuffer < pC2SDataBufferEnd) {
		/*
		 *There are some more data to decode: Result buffer information.
		 *Go on with the decoding process...
		 */

		/*Check the output message offset */
		if (pCommand->nClientAnswerStartOffset == 0) {
			dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad result offset\n",
						pSession);
			nError = S_ERROR_GENERIC;
			goto framework_error;
		}

		pSession->pS2CDataBuffer = pDeviceContextDesc->pAllocatedBuffer +
				pCommand->nClientAnswerStartOffset;

		/*u32: Result buffer type */
		pC2SDataBuffer = static_befDecoderReadUint32(
									pC2SDataBuffer, pC2SDataBufferEnd, &nResultBufferType);
		if (pC2SDataBuffer == NULL) {
			dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad message [Result buffer type]\n",
						pSession);
			nError = S_ERROR_GENERIC;
			goto framework_error;
		}

		/*Result buffer type */
		if (nResultBufferType == SERVICE_PKCS11_BUFFER_NULL) {
			pResultBuffer = NULL;
			nResultLength = 0;
		} else {
			if (nResultBufferType != SERVICE_PKCS11_BUFFER_MAPPABLE) {
				dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad Result buffer type [0x%X]\n",
							pSession, nResultBufferType);
				nError = S_ERROR_GENERIC;
				goto framework_error;
			}

			/*u32: Result buffer length */
			pC2SDataBuffer = static_befDecoderReadUint32(
										pC2SDataBuffer, pC2SDataBufferEnd, &nResultLength);
			if (pC2SDataBuffer == NULL) {
				dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad message [Result buffer length]\n",
							pSession);
				nError = S_ERROR_GENERIC;
				goto framework_error;
			}

			/*u32: Result buffer virtual address */
			pC2SDataBuffer = static_befDecoderReadUint32(
										pC2SDataBuffer, pC2SDataBufferEnd, &nUserAddr);
			if (pC2SDataBuffer == NULL) {
				dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad message [Result buffer addr]\n",
							pSession);
				nError = S_ERROR_GENERIC;
				goto framework_error;
			}

			pResultBuffer = (u8 *)nUserAddr;
			if (pResultBuffer == NULL) {
				dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Invalid result client buffer [0x%X, %u]\n",
							pSession, nUserAddr, nResultLength);
				nError = S_ERROR_GENERIC;
				goto framework_error;
			}

			/*The output client message will contain, at least, a u32 (result length)*/
			if (pCommand->nClientAnswerSizeMax < 5) {
				dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Bad nClientAnswerSizeMax [%u]\n",
							pSession, pCommand->nClientAnswerSizeMax);
				nError = S_ERROR_GENERIC;
				goto framework_error;
			}
		}
	}  /*pC2SDataBuffer < pC2SDataBufferEnd */

	/*
	 *Make sure that we can access the input and output buffers
	 */
#if 0
	if (!access_ok(VERIFY_READ, pDataBuffer, nDataLength)) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Read access not granted for input buffer %p/%u\n",
					pSession, pDataBuffer, nDataLength);
		nError = S_ERROR_BAD_PARAMETERS;
		goto framework_error;
	}
#endif
	if (pResultBuffer != NULL) {
		if (!access_ok(VERIFY_WRITE, pResultBuffer, nResultLength)) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Write access not granted for output buffer %p/%u\n",
						pSession, pResultBuffer, nResultLength);
			nError = S_ERROR_BAD_PARAMETERS;
			goto framework_error;
		}
	}

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK
	SCXBenchmark(NULL);
#endif /*SMODULE_SMC_OMAP3430_BENCHMARK */

	if (nDataLength <= pSession->nInnerBufferLength) {
		pSession->pDataBuffer = pSession->pInnerBuffer;
	} else {
		pSession->pDataBuffer = (u8 *)internal_kmalloc(nDataLength, GFP_KERNEL);
		if (pSession->pDataBuffer == NULL) {
			dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Out of memory for buffer (%u bytes)\n",
						pSession, nDataLength);
			nError = S_ERROR_OUT_OF_MEMORY;
			goto service_error;
		}
	}

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK
	SCXBenchmark(NULL);
#endif /*SMODULE_SMC_OMAP3430_BENCHMARK */

	pSession->nDataLength = nDataLength;
	if (copy_from_user(pSession->pDataBuffer, pDataBuffer, nDataLength)) {
		dprintk(KERN_ERR "scxPublicCryptoPreProcessCommand(%p) : Read access not granted for input buffer %p/%u\n",
					pSession, pDataBuffer, nDataLength);
		nError = S_ERROR_ACCESS_DENIED;
		goto service_error;
	}

	pSession->nResultLength = nResultLength;
	pSession->pResultBuffer = pSession->pDataBuffer;

	pSession->pResultBufferUser = pResultBuffer;

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK
	SCXBenchmark(NULL);
#endif /*SMODULE_SMC_OMAP3430_BENCHMARK */

	return 0;

framework_error:
	pAnswer->nClientAnswerSize = nS2CBufferSize;
	pAnswer->nFrameworkStatus = nError;
	pAnswer->nServiceError = nError;
	return -EFAULT;

service_error:
	pAnswer->nClientAnswerSize = nS2CBufferSize;
	pAnswer->nFrameworkStatus = S_ERROR_SERVICE;
	pAnswer->nServiceError = nError;
	return -EFAULT;
}


/*---------------------------------------------------------------------------- */

/*
 *Process the client command (crypto update operation),
 *i.e. fully perform the update operation without the need of the PA.
 *THIS FUNCTION IS CALLED FROM THE USER THREAD (ioctl).
 */
void scxPublicCryptoProcessCommand(
		SCX_PUBLIC_CRYPTO_DESC *pSession,
		SCX_ANSWER_INVOKE_CLIENT_COMMAND *pAnswer)
{
	u32 nError;
	uint32_t nPKCS11Error;

	dprintk(KERN_INFO "scxPublicCryptoProcessCommand(%p) : Session=0x%x\n",
				pSession, pSession->hClientSession);

	/*
	 *We don't need to restore the registers as the key has not changed.
	 *This is only applicable for AES and DES.
	 */
	nError = scxPublicCryptoUpdate(pSession, false);
	nPKCS11Error = scxPublicCryptoConvertOperationError(nError);

	if (nPKCS11Error != CKR_OK) {
		dprintk(KERN_ERR "scxPublicCryptoProcessCommand(%p) : scxPublicCryptoUpdate failed [0x%x, 0x%x]\n",
					pSession, nError, nPKCS11Error);
	}

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK
	SCXBenchmark(NULL);
#endif /*SMODULE_SMC_OMAP3430_BENCHMARK */

	scxPublicCryptoPostProcessCommand(pSession, pAnswer, true, nPKCS11Error);
}

/*---------------------------------------------------------------------------- */

/*
 *Post-process the client command (crypto update operation),
 *i.e. copy the result into the user output buffer and release the resources.
 *THIS FUNCTION IS CALLED FROM THE USER THREAD (ioctl).
 */
void scxPublicCryptoPostProcessCommand(
		SCX_PUBLIC_CRYPTO_DESC *pSession,
		SCX_ANSWER_INVOKE_CLIENT_COMMAND *pAnswer,
		bool bProcessedInPublic,
		uint32_t nPKCS11Error)
{
	u32	nS2CBufferSize = 0;
	u32	nServiceError = nPKCS11Error;
	u32	nFrameworkStatus = S_SUCCESS;

	dprintk(KERN_INFO "scxPublicCryptoPostProcessCommand(%p) : Session=0x%x, FromPublic=%d, ErrCode=0x%x\n",
				pSession, pSession->hClientSession, bProcessedInPublic, nPKCS11Error);

	if (!bProcessedInPublic) {
		/*The message has been processed in secure, so the answer is already formatted */
		/*
		 *Copy the result into the user result buffer (if the nResultActualLength is meaningful)
		 */
		if (pSession->nResultActualLength != 0xFFFFFFFF) {
			if (copy_to_user(pSession->pResultBufferUser, pSession->pResultBuffer, pSession->nResultActualLength)) {
				dprintk(KERN_ERR "scxPublicCryptoPostProcessCommand(%p) : Write access not granted for output buffer %p/%u\n",
							pSession, pSession->pResultBufferUser, pSession->nResultActualLength);
				pAnswer->nFrameworkStatus = S_ERROR_SERVICE;
				pAnswer->nServiceError = CKR_DEVICE_ERROR;
			}
		}

		goto release_buffer;
	}

	/*
	 *The message has been processed in public...
	 */
	if ((nServiceError != CKR_OK) && (nServiceError != CKR_BUFFER_TOO_SMALL)) {
		/*
		 *There is a special case for the CKR_BUFFER_TOO_SMALL error,
		 * as the required result length must still be returned.
		 */
		nFrameworkStatus = S_ERROR_SERVICE;
		goto end;
	}

	if (pSession->pS2CDataBuffer != NULL) {
		u8 *pS2CDataBufferEnd = pSession->pS2CDataBuffer;
		/*
		 *Start the encoding...
		 *u32: Result buffer length
		 */
		pS2CDataBufferEnd = static_befEncoderWriteUint32(pS2CDataBufferEnd, pSession->nResultActualLength);

		/*Encoded output message size */
		nS2CBufferSize = pS2CDataBufferEnd - pSession->pS2CDataBuffer;

		if (nServiceError == CKR_BUFFER_TOO_SMALL) {
			nFrameworkStatus = S_ERROR_SERVICE;
			goto end;
		}

		/*
		 *Copy the result into the user result buffer
		 */
		if (copy_to_user(pSession->pResultBufferUser, pSession->pResultBuffer, pSession->nResultActualLength)) {
			dprintk(KERN_ERR "scxPublicCryptoPostProcessCommand(%p) : Write access not granted for output buffer %p/%u\n",
						pSession, pSession->pResultBufferUser, pSession->nResultActualLength);
			nFrameworkStatus = S_ERROR_SERVICE;
			nServiceError = CKR_DEVICE_ERROR;
			goto end;
		}
	}

end:
	/*
	 *Generate the answer
	 */
	pAnswer->nClientAnswerSize = nS2CBufferSize;
	pAnswer->nFrameworkStatus = nFrameworkStatus;
	pAnswer->nServiceError = nServiceError;;

release_buffer:

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK
	SCXBenchmark(NULL);
#endif /*SMODULE_SMC_OMAP3430_BENCHMARK */

	if (pSession->pDataBuffer != pSession->pInnerBuffer) {
		internal_kfree(pSession->pDataBuffer);
	}
	pSession->pDataBuffer = NULL;

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK
	SCXBenchmark(NULL);
#endif /*SMODULE_SMC_OMAP3430_BENCHMARK */
}


/*---------------------------------------------------------------------------- */

U32 scxPublicCryptoWaitForReadyBit(VU32 *pRegister, U32 vBit)
{
	U32 timeoutCounter = PUBLIC_CRYPTO_TIMEOUT_CONST;

	while ((!(INREG32(pRegister) & vBit))  &&
		((--timeoutCounter) != 0)) {
	}

	if (timeoutCounter == 0) {
		return PUBLIC_CRYPTO_ERR_TIMEOUT;
	}

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*---------------------------------------------------------------------------- */

void scxPublicCryptoWaitForReadyBitInfinitely(VU32 *pRegister, U32 vBit)
{
	while ( !(INREG32(pRegister) & vBit) )
	{
	}
}

/*---------------------------------------------------------------------------- */

void scxPublicCryptoEnableHwClock(e_CryptoAccelerator nAccelerator)
{
	U32 *pClockReg;
	U32 *pAutoIdleReg;
   U32 vClockBit;
   U32 nITFlags;

   //Ensure none concurrent access when changing clock registers
   local_irq_save(nITFlags);

   switch (nAccelerator)
   {
      case CRYPTO_ACCELERATOR_AES1:
      {
         vClockBit    = PUBLIC_CRYPTO_AES_CLOCK_BIT;
         pClockReg    = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_CLOCK_ADDR2);
         pAutoIdleReg = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR2);
      }
      break;
      case CRYPTO_ACCELERATOR_SHA1:
      {
         vClockBit = PUBLIC_CRYPTO_SHA_CLOCK_BIT;
         pClockReg    = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_CLOCK_ADDR2);
         pAutoIdleReg = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR2);
      }
      break;
      case CRYPTO_ACCELERATOR_DES2:
      {
         vClockBit = PUBLIC_CRYPTO_DES_CLOCK_BIT;
         pClockReg    = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_CLOCK_ADDR1);
         pAutoIdleReg = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR1);
      }
      break;
      default:
      {
         dprintk(KERN_ERR "scxPublicCryptoEnableHwClock(0x%x): Unknown accelerator.\n", nAccelerator);
         return;
      }
   }

	 *(pClockReg) |= vClockBit;
	 *(pAutoIdleReg) &= ~vClockBit;

   local_irq_restore(nITFlags);

	dprintk(KERN_INFO "scxPublicCryptoEnableHwClock\n");
}

/*---------------------------------------------------------------------------- */

void scxPublicCryptoDisableHwClock(e_CryptoAccelerator nAccelerator)
{
	U32 *pClockReg;
	U32 *pAutoIdleReg;
   U32 vClockBit;
   U32 nITFlags;

   //Ensure none concurrent access when changing clock registers
   local_irq_save(nITFlags);

   switch (nAccelerator)
   {
      case CRYPTO_ACCELERATOR_AES1:
      {
         vClockBit    = PUBLIC_CRYPTO_AES_CLOCK_BIT;
         pClockReg    = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_CLOCK_ADDR2);
         pAutoIdleReg = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR2);
      }
      break;
      case CRYPTO_ACCELERATOR_SHA1:
      {
         vClockBit = PUBLIC_CRYPTO_SHA_CLOCK_BIT;
         pClockReg    = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_CLOCK_ADDR2);
         pAutoIdleReg = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR2);
      }
      break;
      case CRYPTO_ACCELERATOR_DES2:
      {
         vClockBit = PUBLIC_CRYPTO_DES_CLOCK_BIT;
         pClockReg    = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_CLOCK_ADDR1);
         pAutoIdleReg = (U32 *)IO_ADDRESS(PUBLIC_CRYPTO_HW_AUTOIDLE_ADDR1);
      }
      break;
      default:
      {
         dprintk(KERN_ERR "scxPublicCryptoEnableHwClock(0x%x): Unknown accelerator.\n", nAccelerator);
         return;
      }
   }

	 *(pClockReg) &= ~vClockBit;
	 *(pAutoIdleReg) |= vClockBit;

   local_irq_restore(nITFlags);

	dprintk(KERN_INFO "scxPublicCryptoDisableHwClock\n");
}

/*---------------------------------------------------------------------------- */

void scxPublicCryptoEnableClock(e_CryptoAccelerator nAccelerator)
{
#ifdef SMODULE_SMC_OMAP3430_POWER_MANAGEMENT
	SCXLNX_SM_COMM_MONITOR *pSMComm = &(g_SCXLNXDeviceMonitor.sm);
	del_timer(&pSMComm->pubcrypto.pPowerManagementTimer);
#endif /*SMODULE_SMC_OMAP3430_POWER_MANAGEMENT */

   scxPublicCryptoEnableHwClock(nAccelerator);
}

/*---------------------------------------------------------------------------- */

void scxPublicCryptoDisableClock(e_CryptoAccelerator nAccelerator)
{
#ifdef SMODULE_SMC_OMAP3430_POWER_MANAGEMENT
	SCXLNX_SM_COMM_MONITOR *pSMComm = &(g_SCXLNXDeviceMonitor.sm);
	del_timer(&pSMComm->pubcrypto.pPowerManagementTimer);
#endif /*SMODULE_SMC_OMAP3430_POWER_MANAGEMENT */

   scxPublicCryptoDisableHwClock(nAccelerator);
}

/*---------------------------------------------------------------------------- */

#ifdef SMODULE_SMC_OMAP3430_POWER_MANAGEMENT
void scxPublicCryptoStartInactivityTimer(void)
{
	SCXLNX_SM_COMM_MONITOR *pSMComm = &(g_SCXLNXDeviceMonitor.sm);

	dprintk("Launch the timer\n");
	pSMComm->pubcrypto.pPowerManagementTimer.expires  = jiffies+msecs_to_jiffies(pSMComm->nInactivityTimerExpireMs);
	pSMComm->pubcrypto.pPowerManagementTimer.data	  = (unsigned long)pSMComm;
	pSMComm->pubcrypto.pPowerManagementTimer.function = scxPublicCryptoPowerManagementTimerCbk;
	add_timer(&pSMComm->pubcrypto.pPowerManagementTimer);
}
#endif /*SMODULE_SMC_OMAP3430_POWER_MANAGEMENT */


/*---------------------------------------------------------------------------- */

#ifdef SMODULE_SMC_OMAP3430_BENCHMARK

EXPORT_SYMBOL(scxPublicCryptoEnableClock);
EXPORT_SYMBOL(scxPublicCryptoDisableClock);

EXPORT_SYMBOL(PDrvCryptoInitAES);
EXPORT_SYMBOL(PDrvCryptoUpdateAES);
EXPORT_SYMBOL(PDrvCryptoTerminateAES);
EXPORT_SYMBOL(PDrvCryptoSaveAESRegisters);
EXPORT_SYMBOL(PDrvCryptoRestoreAESRegisters);

EXPORT_SYMBOL(PDrvCryptoInitDES);
EXPORT_SYMBOL(PDrvCryptoUpdateDES);
EXPORT_SYMBOL(PDrvCryptoTerminateDES);
EXPORT_SYMBOL(PDrvCryptoSaveDESRegisters);
EXPORT_SYMBOL(PDrvCryptoRestoreDESRegisters);

EXPORT_SYMBOL(PDrvCryptoInitHash);
EXPORT_SYMBOL(PDrvCryptoUpdateHash);
EXPORT_SYMBOL(PDrvCryptoFinalHash);
EXPORT_SYMBOL(PDrvCryptoTerminateHash);
EXPORT_SYMBOL(PDrvCryptoSaveHashRegisters);
EXPORT_SYMBOL(PDrvCryptoRestoreHashRegisters);

#endif /* SMODULE_SMC_OMAP3430_BENCHMARK */

/*
 * arch/arm/plat-omap/include/dspbridge/chnl.h
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
 *  ======== chnl.h ========
 *  Description:
 *      WCD channel interface: multiplexes data streams through the single
 *      physical link managed by a mini-driver.
 *
 *  Public Functions:
 *      CHNL_AddIOReq
 *      CHNL_AllocBuffer
 *      CHNL_CancelIO
 *      CHNL_Close
 *      CHNL_CloseOrphans
 *      CHNL_Create
 *      CHNL_Destroy
 *      CHNL_Exit
 *      CHNL_FlushIO
 *      CHNL_FreeBuffer
 *      CHNL_GetEventHandle
 *      CHNL_GetHandle
 *      CHNL_GetIOCompletion
 *      CHNL_GetId
 *      CHNL_GetMgr
 *      CHNL_GetMode
 *      CHNL_GetPosition
 *      CHNL_GetProcessHandle
 *      CHNL_Init
 *      CHNL_Open
 *
 *  Notes:
 *      See DSP API chnl.h for more details.
 *
 *! Revision History:
 *! ================
 *! 14-Jan-1997 gp: Updated based on code review feedback.
 *! 24-Oct-1996 gp: Move CloseOrphans into here from dspsys.
 *! 09-Sep-1996 gp: Added CHNL_GetProcessID() and CHNL_GetHandle().
 *! 10-Jul-1996 gp: Created.
 */

#ifndef CHNL_
#define CHNL_

#include <dspbridge/chnlpriv.h>

/*
 *  ======== CHNL_AddIOReq ========
 *  Purpose:
 *      Enqueue an I/O request for data transfer with the DSP on this channel.
 *      The direction (mode) is specified in the channel object.
 *  Parameters:
 *      hChnl:          Channel object handle.
 *      pHostBuf:       Host buffer address source.
 *      cBytes:         Number of PC bytes to transfer. A zero value indicates
 *                      that this buffer is the last in the output channel.
 *                      A zero value is invalid for an input channel.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EHANDLE:    Invalid hChnl.
 *      DSP_EPOINTER:   pHostBuf is invalid.
 *      CHNL_E_NOEOS:   User cannot mark EOS on an input channel.
 *      CHNL_E_CANCELLED: I/O has been cancelled on this channel.  No further
 *                      I/O is allowed.
 *      CHNL_E_EOS:     End of stream was already marked on a previous
 *                      IORequest on this output channel. Not returned for
 *                      input channels.
 *      CHNL_E_NOIORPS: No free IO request packets available for queuing.
 *      CHNL_E_BUFSIZE: Buffer submitted to this output channel is larger than
 *                      the size of the physical shared memory output window.
 *  Requires:
 *      CHNL_Init() called.
 *      pHostBuf points to memory which can be safely accessed at interrupt
 *      time without page fault.
 *  Ensures:
 *      The buffer will be transferred if the channel is ready; otherwise, will
 *      be queued for transfer when the channel becomes ready.  In any case,
 *      notifications of I/O completion are asynchronous.
 *      If cBytes is 0 for an output channel, subsequent CHNL_AddIOReq's on
 *      this channel will fail with error code CHNL_E_EOS.  The corresponding
 *      IOC for this I/O request will have its status flag set to
 *      CHNL_IOCSTATEOS.
 */
	extern DSP_STATUS CHNL_AddIOReq(struct CHNL_OBJECT *hChnl,
					void *pHostBuf,
					u32 cBytes);

/*
 *  ======== CHNL_AllocBuffer ========
 *  Purpose:
 *      Allocate a zero-initialized buffer to be used in data transfers though
 *      a channel managed by this channel manager.
 *  Parameters:
 *      ppBuf:              Location to store buffer pointer.
 *      hChnlMgr:           Handle to a valid channel manager.
 *      cBytes:             Size of buffer in bytes. Must be greater than zero.
 *  Returns:
 *      DSP_SOK:            Success.
 *      DSP_EPOINTER:       ppBuf is invalid.
 *      DSP_EHANDLE:        hChnlMgr is invalid.
 *      DSP_EMEMORY:        Insufficient memory to allocate buffer.
 *      DSP_EINVALIDARG:    Invalid cBytes value.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 *      DSP_SOK:            *ppBuf points to memory which can be accessed in
 *                          any context.
 *      else:               *ppBuf contains NULL if ppBuf != NULL.
 */
	extern DSP_STATUS CHNL_AllocBuffer(OUT void **ppBuf,
					   struct CHNL_MGR *hChnlMgr,
					   u32 cBytes);

/*
 *  ======== CHNL_CancelIO ========
 *  Purpose:
 *      Return all I/O requests to the client which have not yet been
 *      transferred.  The channel's I/O completion object is
 *      signalled, and all the I/O requests are queued as IOC's, with the
 *      status field set to CHNL_IOCSTATCANCEL.
 *      This call is typically used in abort situations, and is a prelude to
 *      CHNL_Close();
 *  Parameters:
 *      hChnl:          Channel object handle.
 *  Returns:
 *      DSP_SOK:           Success;
 *      DSP_EHANDLE:       Invalid hChnl.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 *      Subsequent I/O requests to this channel will not be accepted.
 */
	extern DSP_STATUS CHNL_CancelIO(struct CHNL_OBJECT *hChnl);

/*
 *  ======== CHNL_Close ========
 *  Purpose:
 *      Ensures all pending I/O on this channel is cancelled, discards all
 *      queued I/O completion notifications, then frees the resources allocated
 *      for this channel, and makes the corresponding logical channel id
 *      available for subsequent use.
 *  Parameters:
 *      hChnl:          Channel object handle.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EHANDLE:    Invalid hChnl.
 *  Requires:
 *      CHNL_Init() called.
 *      No thread must be blocked on this channel's I/O completion event.
 *  Ensures:
 *      DSP_SOK:        The I/O completion event for this channel is freed.
 *                      hChnl is no longer valid.
 */
	extern DSP_STATUS CHNL_Close(struct CHNL_OBJECT *hChnl);

/*
 *  ======== CHNL_CloseOrphans ========
 *  Purpose:
 *      Close open channels orphaned by a closing process.
 *  Parameters:
 *      hChnlMgr:       Channel manager holding the channels.
 *      hProcess:       Kernel mode handle of the process claiming the channels.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_SFALSE:     No channels were left open by this process.
 *      DSP_EHANDLE:    Invalid hChnlMgr handle.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 */
	extern DSP_STATUS CHNL_CloseOrphans(struct CHNL_MGR *hChnlMgr,
					    HANDLE hProcess);

/*
 *  ======== CHNL_Create ========
 *  Purpose:
 *      Create a channel manager object, responsible for opening new channels
 *      and closing old ones for a given board.
 *  Parameters:
 *      phChnlMgr:      Location to store a channel manager object on output.
 *      hDevObject:     Handle to a device object.
 *      pMgrAttrs:      Channel manager attributes.
 *      pMgrAttrs->cChannels:   Max channels
 *      pMgrAttrs->bIRQ:        Channel's I/O IRQ number.
 *      pMgrAttrs->fShared:     TRUE if the IRQ is shareable.
 *      pMgrAttrs->uWordSize:   DSP Word size in equivalent PC bytes..
 *  Returns:
 *      DSP_SOK:                Success;
 *      DSP_EHANDLE:            hDevObject is invalid.
 *      DSP_EINVALIDARG:        cChannels is 0.
 *      DSP_EMEMORY:            Insufficient memory for requested resources.
 *      CHNL_E_ISR:             Unable to plug channel ISR for configured IRQ.
 *      CHNL_E_MAXCHANNELS:     This manager cannot handle this many channels.
 *      CHNL_E_INVALIDIRQ:      Invalid IRQ number. Must be 0 <= bIRQ <= 15.
 *      CHNL_E_INVALIDWORDSIZE: Invalid DSP word size.  Must be > 0.
 *      CHNL_E_INVALIDMEMBASE:  Invalid base address for DSP communications.
 *      CHNL_E_MGREXISTS:       Channel manager already exists for this device.
 *  Requires:
 *      CHNL_Init() called.
 *      phChnlMgr != NULL.
 *      pMgrAttrs != NULL.
 *  Ensures:
 *      DSP_SOK:                Subsequent calls to CHNL_Create() for the same
 *                              board without an intervening call to
 *                              CHNL_Destroy() will fail.
 */
	extern DSP_STATUS CHNL_Create(OUT struct CHNL_MGR **phChnlMgr,
				      struct DEV_OBJECT *hDevObject,
				      IN CONST struct CHNL_MGRATTRS *pMgrAttrs);

/*
 *  ======== CHNL_Destroy ========
 *  Purpose:
 *      Close all open channels, and destroy the channel manager.
 *  Parameters:
 *      hChnlMgr:           Channel manager object.
 *  Returns:
 *      DSP_SOK:            Success.
 *      DSP_EHANDLE:        hChnlMgr was invalid.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 *      DSP_SOK:            Cancels I/O on each open channel.
 *                          Closes each open channel.
 *                          CHNL_Create may subsequently be called for the
 *                          same board.
 */
	extern DSP_STATUS CHNL_Destroy(struct CHNL_MGR *hChnlMgr);

/*
 *  ======== CHNL_Exit ========
 *  Purpose:
 *      Discontinue usage of the CHNL module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      CHNL_Init() previously called.
 *  Ensures:
 *      Resources, if any acquired in CHNL_Init(), are freed when the last
 *      client of CHNL calls CHNL_Exit().
 */
	extern void CHNL_Exit();

/*
 *  ======== CHNL_FlushIO ========
 *  Purpose:
 *      For an output stream (to the DSP), flush all pending IO requests to the
 *      output device.  This function will wait for IO completion for each of
 *      the queued IO requests, up to the specified timeout (per IOR).   For
 *      input streams (from the DSP), will cancel all pending IO requests.
 *  Parameters:
 *      hChnl:          Channel object handle.
 *      dwTimeOut:      Timeout in milliseconds to wait for I/O completion.
 *                      A value of CHNL_IOCINFINITE means to wait indefinitely.
 *                      The value of CHNL_IOCNOWAIT is not allowed.
 *  Returns:
 *      DSP_SOK:            Success;
 *      DSP_EHANDLE:        Invalid hChnl.
 *      DSP_EINVALIDARG:    dwTimeOut value of CHNL_IOCNOWAIT was given.
 *      CHNL_E_WAITTIMEOUT: Wait for flush of output stream timed out.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 *      DSP_SOK:        No I/O requests will be pending on this channel.
 */
	extern DSP_STATUS CHNL_FlushIO(struct CHNL_OBJECT *hChnl,
					u32 dwTimeOut);

/*
 *  ======== CHNL_FreeBuffer ========
 *  Purpose:
 *      Free a buffer previously allocated using CHNL_AllocBuffer().
 *  Parameters:
 *      hChnlMgr:       Handle to a valid channel manager.
 *      cBytes:         Size of buffer in bytes.
 *      pBuf:           Buffer pointer returned by CHNL_FreeBuffer().
 *  Returns:
 *      DSP_SOK:           Success.
 *      DSP_EHANDLE:       hChnlMgr is invalid.
 *      DSP_EINVALIDARG:   pBuf is NULL.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 */
	extern DSP_STATUS CHNL_FreeBuffer(struct CHNL_MGR *hChnlMgr,
					  u32 cBytes,
					  void *pBuf);

/*
 *  ======== CHNL_GetEventHandle ========
 *  Purpose:
 *      Retrieve this channel's I/O completion auto-reset event.
 *  Parameters:
 *      hChnl:          Handle to a valid channel object.
 *      phEvent:        Location to store the I/O completion event object.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EHANDLE:    Invalid hChnl.
 *      DSP_EPOINTER:   phEvent is invalid.
 *  Requires:
 *      CHNL_Init() called.

 */
	extern DSP_STATUS CHNL_GetEventHandle(struct CHNL_OBJECT *hChnl,
					      OUT HANDLE *phEvent);

/*
 *  ======== CHNL_GetHandle ========
 *  Purpose:
 *      Retrieve the channel handle given the logical ID and channel manager.
 *  Parameters:
 *      hChnlMgr:           Handle to a valid channel manager, or NULL.
 *      uChnlID:            Channel ID.
 *      phChnl:             Location to store channel handle.
 *  Returns:
 *      DSP_SOK:            Success;
 *      DSP_EHANDLE:        Invalid hChnlMgr.
 *      DSP_EPOINTER:       phChnl == NULL.
 *      CHNL_E_BADCHANID:   Invalid channel ID.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 *      DSP_SOK:            *phChnl points to a valid channel object,
 *                          if phChnl != NULL.
 */
	extern DSP_STATUS CHNL_GetHandle(struct CHNL_MGR *hChnlMgr,
					 u32 uChnlID,
					 OUT struct CHNL_OBJECT **phChnl);

/*
 *  ======== CHNL_GetIOCompletion ========
 *  Purpose:
 *      Optionally wait for I/O completion on a channel.  Dequeue an I/O
 *      completion record, which contains information about the completed
 *      I/O request.
 *  Parameters:
 *      hChnl:          Channel object handle.
 *      dwTimeOut:      Timeout in milliseconds to wait for completion.
 *                      A value of CHNL_IOCINFINITE means to wait indefinitely.
 *                      A value of CHNL_IOCNOWAIT will simply dequeue the
 *                      first available IOC.
 *      pIOC:           On output, contains host buffer address, bytes
 *                      transferred, and status of I/O completion.
 *      pIOC->status:   I/O Completion status: see chnldefs.h for definitions.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Invalid hChnl.
 *      DSP_EPOINTER:   pIOC is invalid.
 *      CHNL_E_NOIOC:   CHNL_IOCNOWAIT was specified as the dwTimeOut parameter
 *                      yet no I/O completions were queued.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 *      DSP_SOK:        If there are any remaining IOC's queued before this
 *                      call returns, the channel event object will be left
 *                      in a signalled state.
 *      If the return status is anything other than DSP_EPOINTER or DSP_SOK,
 *      then pIOC->pBuf will be set to NULL, pIOC->cBytes will be 0, and
 *      pIOC->status will be undefined.
 */
	extern DSP_STATUS CHNL_GetIOCompletion(struct CHNL_OBJECT *hChnl,
					       u32 dwTimeOut,
					       OUT struct CHNL_IOC *pIOC);

/*
 *  ======== CHNL_GetId ========
 *  Purpose:
 *      Retrieve the channel logical ID of this channel.
 *  Parameters:
 *      hChnl:          Handle to a valid channel object.
 *      pdwID:          Location to store logical ID.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EHANDLE:    Invalid hChnl.
 *      DSP_EPOINTER:   pdwID is invalid.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 */
	extern DSP_STATUS CHNL_GetId(struct CHNL_OBJECT *hChnl,
				     OUT u32 *pdwID);

/*
 *  ======== CHNL_GetMgr ========
 *  Purpose:
 *      Retrieve a channel manager handle, required for opening new channels
 *      and closing old ones on a given board.
 *  Parameters:
 *      hDevNode:       A valid system specific DEVNODE handle.
 *      phChnlMgr:      Location to store the channel manager handle on output.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    hDevNode is invalid.
 *      CHNL_E_NOMGR:   No channel manager exists for this board.
 *  Requires:
 *      CHNL_Init() called.
 *      phChnlMgr != NULL.
 *  Ensures:
 *      DSP_SOK: The DSP board represented by hDevNode was in the RUNNING state
 *            before this function returned.
 *      else: *phChnlMgr == NULL.
 */
	extern DSP_STATUS CHNL_GetMgr(struct CFG_DEVNODE *hDevNode,
				      OUT struct CHNL_MGR **phChnlMgr);

/*
 *  ======== CHNL_GetMode ========
 *  Purpose:
 *      Retrieve the mode flags of this channel.
 *  Parameters:
 *      hChnl:              Handle to a valid channel object.
 *      pMode:              Location to store mode flags.
 *  Returns:
 *      DSP_SOK:            Success;
 *      DSP_EHANDLE:        Invalid hChnl.
 *      DSP_EPOINTER:       pMode is invalid.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 */
	extern DSP_STATUS CHNL_GetMode(struct CHNL_OBJECT *hChnl,
				       OUT CHNL_MODE * pMode);

/*
 *  ======== CHNL_GetPosition ========
 *  Purpose:
 *      Retrieve the total number of bytes transferred on this channel.
 *  Parameters:
 *      hChnl:              Handle to a valid channel object.
 *      pcPosition:         Location to store number of bytes.
 *  Returns:
 *      DSP_SOK:            Success;
 *      DSP_EHANDLE:        Invalid hChnl.
 *      DSP_EPOINTER:       pcPosition is invalid.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 */
	extern DSP_STATUS CHNL_GetPosition(struct CHNL_OBJECT *hChnl,
					   OUT u32 *pcPosition);

/*
 *  ======== CHNL_GetProcessHandle ========
 *  Purpose:
 *      Retrieve the handle of the process owning this channel.
 *  Parameters:
 *      hChnl:              Channel handle.
 *      phProcess:          Location to store the process handle.  A NULL value
 *                          indicates the channel is either closed or is not
 *                          owned by any particular process.
 *  Returns:
 *      DSP_SOK:            Success;
 *      DSP_EHANDLE:        Invalid hChnl.
 *      DSP_EPOINTER:       phProcess is invalid.
 *  Requires:
 *      CHNL_Init() called.
 *  Ensures:
 */
	extern DSP_STATUS CHNL_GetProcessHandle(struct CHNL_OBJECT *hChnl,
						OUT HANDLE *phProcess);

/*
 *  ======== CHNL_Init ========
 *  Purpose:
 *      Initialize the CHNL module's private state.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occurred.
 *  Requires:
 *  Ensures:
 *      A requirement for each of the other public CHNL functions.
 */
	extern bool CHNL_Init();

/*
 *  ======== CHNL_Open ========
 *  Purpose:
 *      Open a new half-duplex channel to the DSP board.
 *  Parameters:
 *      phChnl:         Location to store a channel object handle.
 *      hChnlMgr:       Handle to channel manager, as returned by CHNL_GetMgr().
 *      uMode:          One of {CHNL_MODETODSP, CHNL_MODEFROMDSP} specifies
 *                      direction of data transfer.
 *      uChnlId:        If CHNL_PICKFREE is specified, the channel manager will
 *                      select a free channel id (default);
 *                      otherwise this field specifies the id of the channel.
 *      pAttrs:         Channel attributes.  Attribute fields are as follows:
 *      pAttrs->uIOReqs: Specifies the maximum number of I/O requests which can
 *                       be pending at any given time. All request packets are
 *                       preallocated when the channel is opened.
 *      pAttrs->hEvent: This field allows the user to supply an auto reset
 *                      event object for channel I/O completion notifications.
 *                      It is the responsibility of the user to destroy this
 *                      object AFTER closing the channel.
 *                      This channel event object can be retrieved using
 *                      CHNL_GetEventHandle().
 *      pAttrs->hReserved: The kernel mode handle of this event object.
 *
 *  Returns:
 *      DSP_SOK:                Success.
 *      DSP_EHANDLE:            hChnlMgr is invalid.
 *      DSP_EMEMORY:            Insufficient memory for requested resources.
 *      DSP_EINVALIDARG:        Invalid number of IOReqs.
 *      CHNL_E_BADMODE:         Invalid mode argument.
 *      CHNL_E_OUTOFSTREAMS:    No free channels available.
 *      CHNL_E_BADCHANID:       Channel ID is out of range.
 *      CHNL_E_CHANBUSY:        Channel is in use.
 *  Requires:
 *      CHNL_Init() called.
 *      phChnl != NULL.
 *      pAttrs != NULL.
 *      pAttrs->hEvent is a valid event handle.
 *  Ensures:
 *      DSP_SOK:        *phChnl is a valid channel.
 *      else:           *phChnl is set to NULL if (phChnl != NULL);
 */
	extern DSP_STATUS CHNL_Open(OUT struct CHNL_OBJECT **phChnl,
				    struct CHNL_MGR *hChnlMgr, CHNL_MODE uMode,
				    u32 uChnlId,
				    CONST IN struct CHNL_ATTRS *pAttrs);

#endif				/* CHNL_ */

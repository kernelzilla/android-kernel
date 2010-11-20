/*=============================================================================

   Module Name: esd_poll.h

   DESCRIPTION: Functions to handle ESD recovery polling.

===============================================================================

                       Motorola Confidential Proprietary
                   Advanced Technology and Software Operations
                (c) Copyright Motorola 2009, All Rights Reserved
  
                       Modification   Tracking
Author                    Date         Number    Description of Changes
---------------------  ------------  ----------  -----------------------------
Ryan Johnson (dgc483)  06/05/2009    LIBss56936  Created
=============================================================================*/

#ifndef _ESD_POLL_H_
#define _ESD_POLL_H_

/*=============================================================================
                        GLOBAL FUNCTION PROTOTYPES
=============================================================================*/
void esd_poll_start(void (*esd_handler)(void *arg), void* arg);
void esd_poll_stop(void (*esd_handler)(void *arg));

#endif /* _ESD_POLL_H_ */


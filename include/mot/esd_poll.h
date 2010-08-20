#ifndef _ESD_POLL_H_
#define _ESD_POLL_H_
/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/*=============================================================================
                                MACROS 
=============================================================================*/
#define ESD_POLL_START(handler, arg) \
   esd_poll_start(handler, arg, #handler)

/*=============================================================================
                        GLOBAL FUNCTION PROTOTYPES
=============================================================================*/
void esd_poll_start(void (*esd_handler)(void *arg), void* arg,
   const char* name);
void esd_poll_stop(void (*esd_handler)(void *arg));

#endif /* _ESD_POLL_H_ */


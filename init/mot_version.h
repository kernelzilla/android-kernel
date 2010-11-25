/*
 *
 *
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

#ifndef MOT_VERSION_H
#define MOT_VERSION_H

#if defined(CONFIG_KERNEL_MOTOROLA)
/* This file has been added to the standard QC release */
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
/*=============================================================================================*//**

@file mot_version.h
Defines the version of the different objects and components of the Core Software

*//*================================================================================================
					INCLUDE FILES
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
					GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

/*==================================================================================================
					MACROS
==================================================================================================*/

/*WARNING - lenght of PRODUCT_VERSION_STR should never be bigger than 18*/
#define PRODUCT_VERSION_STR "R6380E210_001.016"

/*==================================================================================================
					ENUMS
==================================================================================================*/

/*==================================================================================================
					STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/


/*==================================================================================================
					FUNCTION PROTOTYPES
==================================================================================================*/

#ifdef __cplusplus
}
#endif

#endif

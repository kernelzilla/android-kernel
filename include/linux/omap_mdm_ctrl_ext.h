/*
     Copyright (C) 2009 Motorola, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
     02111-1307  USA
*/

/* Bit mask of gpios */
enum {
   MASK_BP_READY_AP        = 0x0001,
   MASK_BP_READY2_AP       = 0x0002,
   MASK_BP_RESOUT          = 0x0004,
   MASK_BP_PWRON           = 0x0008,
   MASK_AP_TO_BP_PSHOLD    = 0x0010,
   MASK_AP_TO_BP_FLASH_EN  = 0x0020
};
typedef unsigned short omap_mdm_ctrl_gpiomask;

/* List of IOCtl commands */
#define OMAP_MDM_CTRL_IOCTL_GET_BP_READY_AP       0x0001
#define OMAP_MDM_CTRL_IOCTL_GET_BP_READY2_AP      0x0002
#define OMAP_MDM_CTRL_IOCTL_GET_BP_RESOUT         0x0003
#define OMAP_MDM_CTRL_IOCTL_SET_BP_PWRON          0x0010
#define OMAP_MDM_CTRL_IOCTL_SET_AP_TO_BP_PSHOLD   0x0020
#define OMAP_MDM_CTRL_IOCTL_SET_AP_TO_BP_FLASH_EN 0x0030
#define OMAP_MDM_CTRL_IOCTL_SET_INT_BP_READY_AP   0x0100
#define OMAP_MDM_CTRL_IOCTL_SET_INT_BP_READY2_AP  0x0200
#define OMAP_MDM_CTRL_IOCTL_SET_INT_BP_RESOUT     0x0300

/* List of Interrupt Options */
#define OMAP_MDM_CTRL_IRQ_RISING   0x01
#define OMAP_MDM_CTRL_IRQ_FALLING  0x02

/* List of GPIO output pull options */
#define OMAP_MDM_CTRL_GPIO_HIGH  1
#define OMAP_MDM_CTRL_GPIO_LOW   0

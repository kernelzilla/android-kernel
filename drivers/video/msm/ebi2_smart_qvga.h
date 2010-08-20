/* drivers/video/msm/ebi2_smart_qvga.h
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _EBI2_SMART_QVGA_H
#define _EBI2_SMART_QVGA_H

#ifndef FALSE
#define FALSE                             0
#define TRUE                              (!FALSE)
#endif /* ifndef FALSE */

#define EBI2_SMART_QVGA_TIMING            0x00808061
#define EBI2_SMART_QVGA_NAME              "ebi2_smart_qvga"
#define DISPLAY_PWR_OFF                   0
#define DISPLAY_PWR_ON                    1
#define DISPLAY_PWR_STANDBY               2

#define QVGA_WIDTH                        240
#define QVGA_HEIGHT                       320
#define END_TABLE                         0xFFFF
#define MSEC_DELAY                        0xFFFE
#define USEC_DELAY                        0xFFFD
#define CMD_ONLY                          0xFFFC

/* version 1 register definitions */
#define DISP_V1_ID_INFO_REG               0x0000
#define DISP_V1_RAM_WRITE_REG             0x0022
#define DISP_V1_SET_ENTRY_MODE_REG        0x0003
#define DISP_V1_HORZ_RAM_ADDR_POS_1_REG   0x0050
#define DISP_V1_HORZ_RAM_ADDR_POS_2_REG   0x0051
#define DISP_V1_VERT_RAM_ADDR_POS_1_REG   0x0052
#define DISP_V1_VERT_RAM_ADDR_POS_2_REG   0x0053
#define DISP_V1_RAM_ADDR_SET_H_REG        0x0020
#define DISP_V1_RAM_ADDR_SET_V_REG        0x0021

/* version 1 register settings/values */
#define DISP_V1                           0x9335

/* version 2 command definitions */
#define DISP_V2_READ_ID1_CMD              0x00DA
#define DISP_V2_READ_ID2_CMD              0x00DB
#define DISP_V2_READ_ID3_CMD              0x00DC
#define DISP_V2_RAM_WRITE_CMD             0x002C
#define DISP_V2_COL_ADDR_SET_CMD          0x002A
#define DISP_V2_PAGE_ADDR_SET_CMD         0x002B
#define DISP_V2_DISPLAY_ON_CMD            0x0029
#define DISP_V2_DISPLAY_OFF_CMD           0x0028
#define DISP_V2_DEEP_STANDBY_CMD          0x00DF
#define DISP_V2_DISPLAY_INV_CMD           0x0021
#define DISP_V2_IF_PIXEL_FMT_CMD          0x003A
#define DISP_V2_MEM_ACC_CTL_CMD           0x0036
#define DISP_V2_LEV2_ACCESS_CMD           0x00F0
#define DISP_V2_PS_SETUP1_CMD             0x00F3
#define DISP_V2_PS_SETUP2_CMD             0x00F6
#define DISP_V2_PS_SETUP3_CMD             0x00F5
#define DISP_V2_LEV3_ACCESS_CMD           0x00FC
#define DISP_V2_INTERNAL_REGS_CMD         0x00FD
#define DISP_V2_INTERNAL_VOLTS_CMD        0x00F4
#define DISP_V2_COLOR_SETUP_CMD           0x00F7
#define DISP_V2_SLEEP_OUT_CMD             0x0011
#define DISP_V2_ENABLE_TE_CMD             0x0035
#define DISP_V2_DISP_SETUP1_CMD           0x00F2
#define DISP_V2_DISP_SETUP2_CMD           0x00F8
#define DISP_V2_SEL_GAMMA_CMD             0x00F9
#define DISP_V2_POS_GAMMA_CMD             0x00FA
#define DISP_V2_NEG_GAMMA_CMD             0x00FB

/* version 2 register settings/values */
#define DISP_V2_ENTER_DEEP_STANDBY        0x0001
#define DISP_V2_GRANT_LEV_ACCESS          0x005A
#define DISP_V2_LOCK_LEV_ACCESS           0x00A5

#endif /* _EBI2_SMART_QVGA_H */

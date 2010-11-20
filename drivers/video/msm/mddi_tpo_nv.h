/* drivers/video/msm/mddi_tpo_nv.h
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

#ifndef _MDDI_TPO_NV_H
#define _MDDI_TPO_NV_H

#define SLEPCTRL              0x110
#define PANELCTRL1            0xB30
#define PANELCTRL2            0xB40
#define PANELCTRL3            0xB41
#define PANELCTRL4            0xB50
#define PANELCTRL5            0xB51
#define PANELCTRL6            0xB60
#define PANELCTRL7            0xB70
#define PANELCTRL8            0xB80
#define PANELCTRL9            0xB90
#define ENTRYMODE1            0x360
#define ENTRYMODE2            0x361
#define DRVCTRL               0xB00
#define DRV_OUT_CTRL1         0x371
#define DRV_OUT_CTRL2         0x372
#define WHRAMLOWST            0x2A0
#define WHRAMHIGHST           0x2A1
#define WHRAMLOWEND           0x2A2
#define WHRAMHIGHEND          0x2A3
#define WVRAMLOWST            0x2B0
#define WVRAMHIGHST           0x2B1
#define WVRAMLOWEND           0x2B2
#define WVRAMHIGHEND          0x2B3
#define RAMHSETL              0x2D0 
#define RAMHSETH              0x2D1
#define RAMVSETL              0x2D2
#define RAMVSETH              0x2D3
#define FPOSITION             0x350
#define FCYCLE                0x351
#define PWRCTRL2              0xC20
#define PWRCTRL4              0xC31
#define PUMPCTRL1             0xBA0
#define PUMPCTRL2             0xBA1
#define VCOMHCTRL             0xC50
#define VCOMDCCTRL            0xC52
#define PWRCTRL5              0xC40
#define SOUTCTRL1             0xBE0
#define SOUTCTRL2             0xBE1
#define WRCTRLD               0x530
#define WRCABC                0x550
#define WRCABCMB              0x5E0
#define SPECTRL               0xC10
#define DISPON                0x290

#define GMACTRL1_0            0xE00
#define GMACTRL1_1            0xE01
#define GMACTRL1_2            0xE02
#define GMACTRL1_3            0xE03
#define GMACTRL1_4            0xE04
#define GMACTRL1_5            0xE05
#define GMACTRL1_6            0xE06
#define GMACTRL1_7            0xE07
#define GMACTRL1_8            0xE08
#define GMACTRL1_9            0xE09
#define GMACTRL1_A            0xE0A
#define GMACTRL1_B            0xE0B
#define GMACTRL1_C            0xE0C
#define GMACTRL1_D            0xE0D
#define GMACTRL1_E            0xE0E
#define GMACTRL1_F            0xE0F

#define GMACTRL2_0            0xE10
#define GMACTRL2_1            0xE11
#define GMACTRL2_2            0xE12
#define GMACTRL2_3            0xE13
#define GMACTRL2_4            0xE14
#define GMACTRL2_5            0xE15
#define GMACTRL2_6            0xE16
#define GMACTRL2_7            0xE17
#define GMACTRL2_8            0xE18
#define GMACTRL2_9            0xE19
#define GMACTRL2_A            0xE1A
#define GMACTRL2_B            0xE1B
#define GMACTRL2_C            0xE1C
#define GMACTRL2_D            0xE1D
#define GMACTRL2_E            0xE1E
#define GMACTRL2_F            0xE1F

#define GMACTRL3_0            0xE20
#define GMACTRL3_1            0xE21
#define GMACTRL3_2            0xE22
#define GMACTRL3_3            0xE23
#define GMACTRL3_4            0xE24
#define GMACTRL3_5            0xE25
#define GMACTRL3_6            0xE26
#define GMACTRL3_7            0xE27
#define GMACTRL3_8            0xE28
#define GMACTRL3_9            0xE29
#define GMACTRL3_A            0xE2A
#define GMACTRL3_B            0xE2B
#define GMACTRL3_C            0xE2C
#define GMACTRL3_D            0xE2D
#define GMACTRL3_E            0xE2E
#define GMACTRL3_F            0xE2F

#define GMACTRL4_0            0xE30
#define GMACTRL4_1            0xE31
#define GMACTRL4_2            0xE32
#define GMACTRL4_3            0xE33
#define GMACTRL4_4            0xE34
#define GMACTRL4_5            0xE35
#define GMACTRL4_6            0xE36
#define GMACTRL4_7            0xE37
#define GMACTRL4_8            0xE38
#define GMACTRL4_9            0xE39
#define GMACTRL4_A            0xE3A
#define GMACTRL4_B            0xE3B
#define GMACTRL4_C            0xE3C
#define GMACTRL4_D            0xE3D
#define GMACTRL4_E            0xE3E
#define GMACTRL4_F            0xE3F

#define GMACTRL5_0            0xE40
#define GMACTRL5_1            0xE41
#define GMACTRL5_2            0xE42
#define GMACTRL5_3            0xE43
#define GMACTRL5_4            0xE44
#define GMACTRL5_5            0xE45
#define GMACTRL5_6            0xE46
#define GMACTRL5_7            0xE47
#define GMACTRL5_8            0xE48
#define GMACTRL5_9            0xE49
#define GMACTRL5_A            0xE4A
#define GMACTRL5_B            0xE4B
#define GMACTRL5_C            0xE4C
#define GMACTRL5_D            0xE4D
#define GMACTRL5_E            0xE4E
#define GMACTRL5_F            0xE4F

#define GMACTRL6_0            0xE50
#define GMACTRL6_1            0xE51
#define GMACTRL6_2            0xE52
#define GMACTRL6_3            0xE53
#define GMACTRL6_4            0xE54
#define GMACTRL6_5            0xE55
#define GMACTRL6_6            0xE56
#define GMACTRL6_7            0xE57
#define GMACTRL6_8            0xE58
#define GMACTRL6_9            0xE59
#define GMACTRL6_A            0xE5A
#define GMACTRL6_B            0xE5B
#define GMACTRL6_C            0xE5C
#define GMACTRL6_D            0xE5D
#define GMACTRL6_E            0xE5E
#define GMACTRL6_F            0xE5F
#define USERID                0xD10
#define REVISIONID            0xD20

#define MM_SLEEPOUT           0x1100
#define MM_DIMFUNC1           0x6902
#define MM_DIMFUNC2           0x6903
#define MM_DISP_BRIGHTNESS    0x5100
#define MM_WRCTRLD            0x5300
#define MM_WRCABC             0x5500
#define MM_SET_TEAR_ON        0x3500
#define MM_SET_TEAR_LINE      0x4401
#define MM_SET_ADDR_MODE      0x3600
#define MM_SET_ADDR_MODE_PARAM2 0x3601
#define MM_DDC_SAYS_SO        0xB100
#define MM_DISPON             0x2900
#define MM_PIXEL_FMT          0x3A00
#define MM_SET_RAM_ADDR0      0x2D00
#define MM_SET_RAM_ADDR1      0x2D01
#define MM_SET_RAM_ADDR2      0x2D02
#define MM_SET_RAM_ADDR3      0x2D03
#define MM_DISPOFF            0x2800
#define MM_SLEEPIN            0x1000

#define TPO_NV_VER_F_USERID   0x05
#define TPO_NV_VER_G_USERID   0x06

#define TPO_PLAT_DISP         0xB9F65395
#define TMD_PLAT_DISP         0xB9F65451

#endif /* _MDDI_TPO_NV_H */


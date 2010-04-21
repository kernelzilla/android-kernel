/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/

#ifndef RGM_TNETW1150_H
#define RGM_TNETW1150_H

#include "public_types.h"


/* Base addresses*/
/* They are not used inside registers definition in purpose to allow this header file*/
/* to be used as an easy reference to register -> address date base. Keep this as it*/
/* is very powerful for debugging purpose.*/
#define HOST_SLAVE_BASE	0x00300000
#define INT_BASE		0x00300400
#define REG_CONFIG_BASE	0x00300800
#define CLK_BASE		0x00300C00
#define SDMA_BASE		0x00301000
#define AES_BASE		0x00301400
#define WEP_BASE		0x00301800
#define TKIP_BASE		0x00301C00
#define SEEPROM_BASE	0x00302000
#define PAR_HOST_BASE	0x00302400
#define SDIO_BASE		0x00302800
#define UART_BASE		0x00302C00
#define USB11_BASE		0x00304000
#define LDMA_BASE		0x00304400
#define RX_BASE			0x00304800
#define ACCESS_BASE		0x00304c00
#define TX_BASE			0x00305000
#define RMAC_CSR_BASE	0x00305400
#define AFE_PM			0x00305800
#define VLYNQ_BASE		0x00308000
#define PCI_BASE		0x00308400
#define USB20_BASE		0x0030A000
#define PHY_BASE		0x003C0000


/* System DMA registers*/
/* Order of registers was changed*/
#define DMA_GLB_CFG                    (0x1000)
#define DMA_HDESC_OFFSET               (0x1004)
#define DMA_HDATA_OFFSET               (0x1008)
#define DMA_CFG0                       (0x100C) /* SDMA_HOST_CFG0 changed*/
#define DMA_CTL0                       (0x1010) /* SDMA_CTRL0 changed*/
#define DMA_LENGTH0                    (0x1014)
#define DMA_L_ADDR0                    (0x1018) /* SDMA_RD_ADDR ?*/
#define DMA_L_PTR0                     (0x101C) /* SDMA_RD_OFFSET ?*/
#define DMA_H_ADDR0                    (0x1020) /* SDMA_WR_ADDR ?*/
#define DMA_H_PTR0                     (0x1024) /* SDMA_WR_OFFSET ?*/
#define DMA_STS0                       (0x1028) /* Changed*/
#define DMA_CFG1                       (0x1030) /* SDMA_HOST_CFG1 changed*/
#define DMA_CTL1                       (0x1034) /* SDMA_CTRL1 changed*/
#define DMA_LENGTH1                    (0x1038)
#define DMA_L_ADDR1                    (0x103C)
#define DMA_L_PTR1                     (0x1040)
#define DMA_H_ADDR1                    (0x1044)
#define DMA_H_PTR1                     (0x1048)
#define DMA_STS1                       (0x104C)
#define DMA_HFRM_PTR                   (0x1050) /* New ?*/
#define DMA_DEBUG                      (0x1054) /* Changed*/

/* Local DMA registers*/
/* number changed from 4 to 2*/
#define LDMA_DEBUG                     (0x4400)
#define LDMA_CTL0                      (0x4404) /* Add 2 bits to support fix address (FIFO)*/
#define LDMA_STATUS0                   (0x4408)
#define LDMA_LENGTH0                   (0x440c)
#define LDMA_RD_ADDR0                  (0x4410)
#define LDMA_RD_OFFSET0                (0x4414)
#define LDMA_WR_ADDR0                  (0x4418)
#define LDMA_WR_OFFSET0                (0x441c)
#define LDMA_CTL1                      (0x4428) /* Add 2 bits to support fix address (FIFO)*/
#define LDMA_STATUS1                   (0x442c)
#define LDMA_LENGTH1                   (0x4430)
#define LDMA_RD_ADDR1                  (0x4434)
#define LDMA_RD_OFFSET1                (0x4438)
#define LDMA_WR_ADDR1                  (0x443c)
#define LDMA_WR_OFFSET1                (0x4440)
/* For TNETW compatability (if willbe )*/
#define LDMA_CUR_RD_PTR0               LDMA_RD_ADDR0
#define LDMA_CUR_WR_PTR0               LDMA_WR_ADDR0
#define LDMA_CUR_RD_PTR1               LDMA_RD_ADDR1
#define LDMA_CUR_WR_PTR1               LDMA_WR_ADDR1

/* Host Slave registers*/
#define SLV_SOFT_RESET                 (0x0000) /* self clearing*/
#define SLV_REG_ADDR                   (0x0004)
#define SLV_REG_DATA                   (0x0008)
#define SLV_REG_ADATA                  (0x000c)
#define SLV_MEM_CP                     (0x0010)
#define SLV_MEM_ADDR                   (0x0014)
#define SLV_MEM_DATA                   (0x0018)
#define SLV_MEM_CTL                    (0x001c) /* bit 19 moved to PCMCIA_CTL*/
#define SLV_END_CTL                    (0x0020) /* 2 bits moved to ENDIAN_CTL*/

/* Timer registers*/
/* Timer1/2 count MAC clocks*/
/* Timer3/4/5 count usec*/
#define TIM1_CTRL                      (0x0918)
#define TIM1_LOAD                      (0x091C)
#define TIM1_CNT                       (0x0920)
#define TIM2_CTRL                      (0x0924)
#define TIM2_LOAD                      (0x0928)
#define TIM2_CNT                       (0x092C)
#define TIM3_CTRL                      (0x0930)
#define TIM3_LOAD                      (0x0934)
#define TIM3_CNT                       (0x0938)
#define TIM4_CTRL                      (0x093C)
#define TIM4_LOAD                      (0x0940)
#define TIM4_CNT                       (0x0944)
#define TIM5_CTRL                      (0x0948)
#define TIM5_LOAD                      (0x094C)
#define TIM5_CNT                       (0x0950)

/* Watchdog registers*/
#define WDOG_CTRL                      (0x0954)
#define WDOG_LOAD                      (0x0958)
#define WDOG_CNT                       (0x095C)
#define WDOG_STS                       (0x0960)
#define WDOG_FEED                      (0x0964)

/* Interrupt registers*/
/* 64 bit interrupt sources registers ws ced. sme interupts were removed and new ones were added*/
/* Order was changed*/
#define FIQ_MASK                       (0x0400)
#define FIQ_MASK_L                     (0x0400)
#define FIQ_MASK_H                     (0x0404)
#define FIQ_MASK_SET                   (0x0408)
#define FIQ_MASK_SET_L                 (0x0408)
#define FIQ_MASK_SET_H                 (0x040C)
#define FIQ_MASK_CLR                   (0x0410)
#define FIQ_MASK_CLR_L                 (0x0410)
#define FIQ_MASK_CLR_H                 (0x0414)
#define IRQ_MASK                       (0x0418)
#define IRQ_MASK_L                     (0x0418)
#define IRQ_MASK_H                     (0x041C)
#define IRQ_MASK_SET                   (0x0420)
#define IRQ_MASK_SET_L                 (0x0420)
#define IRQ_MASK_SET_H                 (0x0424)
#define IRQ_MASK_CLR                   (0x0428)
#define IRQ_MASK_CLR_L                 (0x0428)
#define IRQ_MASK_CLR_H                 (0x042C)
#define ECPU_MASK                      (0x0448)
#define FIQ_STS_L                      (0x044C)
#define FIQ_STS_H                      (0x0450)
#define IRQ_STS_L                      (0x0454)
#define IRQ_STS_H                      (0x0458)
#define INT_STS_ND                     (0x0464)
#define INT_STS_RAW_L                  (0x0464)
#define INT_STS_RAW_H                  (0x0468)
#define INT_STS_CLR                    (0x04B4)
#define INT_STS_CLR_L                  (0x04B4)
#define INT_STS_CLR_H                  (0x04B8)
#define INT_ACK                        (0x046C)
#define INT_ACK_L                      (0x046C)
#define INT_ACK_H                      (0x0470)
#define INT_TRIG                       (0x0474)
#define INT_TRIG_L                     (0x0474)
#define INT_TRIG_H                     (0x0478)
#define HOST_STS_L                     (0x045C)
#define HOST_STS_H                     (0x0460)
#define HOST_MASK                      (0x0430)
#define HOST_MASK_L                    (0x0430)
#define HOST_MASK_H                    (0x0434)
#define HOST_MASK_SET                  (0x0438)
#define HOST_MASK_SET_L                (0x0438)
#define HOST_MASK_SET_H                (0x043C)
#define HOST_MASK_CLR                  (0x0440)
#define HOST_MASK_CLR_L                (0x0440)
#define HOST_MASK_CLR_H                (0x0444)

/* GPIO Interrupts*/
#define GPIO_INT_STS                   (0x0484) /* 22 GPIOs*/
#define GPIO_INT_ACK                   (0x047C)
#define GPIO_INT_MASK                  (0x0480)
#define GPIO_POS_MASK                  (0x04BC) /* New*/
#define GPIO_NEG_MASK                  (0x04C0) /* New*/

/* Protocol Interrupts*/
#define PROTO_INT_STS                  (0x0490) /* Add 2 PHY->MAC source interrupts*/
#define PROTO_INT_ACK                  (0x0488)
#define PROTO_INT_MASK                 (0x048C)

/* Host Interrupts*/
#define HINT_MASK                      (0x0494)
#define HINT_MASK_SET                  (0x0498)
#define HINT_MASK_CLR                  (0x049C)
#define HINT_STS_ND_MASKED             (0x04A0)
#define HINT_STS_ND  		           (0x04B0) /*1150 spec calls this HINT_STS_RAW*/
#define HINT_STS_CLR                   (0x04A4)
#define HINT_ACK                       (0x04A8)
#define HINT_TRIG                      (0x04AC)

/* Clock registers*/
#define CLK_CFG                        (0x0C00) /* new ARM clock bit */
#define CLK_CTRL                       (0x0C04) /* changed*/
#define BLK_RST                        (0x0C08) /* changed*/
#define CFG_USEC_STB                   (0x0C0C)
#define ARM_GATE_CLK_REG               (0x0C10) /* new*/
#define BUSY_STAT_REG                  (0x0C14) /* new*/
#define CFG_PHY_CLK88                  (0x0C18)
#define DYNAMIC_CLKGATE                (0x0C1C) /* new*/

/* AES registers*/
/* Major changes to this module*/
#define AES_START                      (0x1400)
#define AES_CFG                        (0x1404)
#define AES_CTL                        (0x1408)
#define AES_STATUS                     (0x140C)
#define AES_LENGTH                     (0x1410)
#define AES_RD_ADDR                    (0x1414)
#define AES_RD_OFFSET                  (0x1418)
#define AES_WR_ADDR                    (0x141C)
#define AES_WR_OFFSET                  (0x1420)
#define AES_CUR_RD_PTR                 (0x1424)
#define AES_CUR_WR_PTR                 (0x1428)
#define AES_KEY_0                      (0x142C)
#define AES_KEY_1                      (0x1430)
#define AES_KEY_2                      (0x1434)
#define AES_KEY_3                      (0x1438)
#define AES_NONCE_0                    (0x143C)
#define AES_NONCE_1                    (0x1440)
#define AES_NONCE_2                    (0x1444)
#define AES_NONCE_3                    (0x1448)
#define AES_MIC_0                      (0x144C)
#define AES_MIC_1                      (0x1450)
#define AES_MIC_2                      (0x1454)
#define AES_MIC_3                      (0x1458)
#define AES_ASSO_DATA_0                (0x145C)
#define AES_ASSO_DATA_1                (0x1460)
#define AES_ASSO_DATA_2                (0x1464)
#define AES_ASSO_DATA_3                (0x1468)
#define AES_NUM_OF_ROUNDS              (0x146C)
#define AES_TX_QUEUE_PTR               (0x1470)
#define AES_RX_QUEUE_PTR               (0x1474)
#define AES_STACK                      (0x1478)
#define AES_INT_RAW                    (0x147C)
#define AES_INT_MASK                   (0x1480)
#define AES_INT_STS                    (0x1484)

/* WEP registers*/
/* Order was changed*/
#define DEC_CTL                        (0x1800)
#define DEC_STATUS                     (0x1804)
#define DEC_MBLK                       (0x1808)
#define DEC_KEY_ADDR                   (0x180C)
#define DEC_KEY_LEN                    (0x1810)
#define DEC_ADDR_UPPER_BYTE            (0x1814) /* new*/
#define DEC_LEN                        (0x1818)
#define DEC_OFFSET                     (0x181C)
#define DEC_WR_MBLK                    (0x1820)
#define DEC_WR_OFFSET                  (0x1824)

/* TKIP MICHAEL reisters*/
/* order changed*/
#define MCHL_START0                    (0x1C00)
#define MCHL_DMV_START_MBLK0           (0x1C04) /* Changed to 23:5 format*/
#define MCHL_DMV_CUR_MBLK0             (0x1C10)
#define MCHL_DMV_OFFSET0               (0x1C08)
#define MCHL_DMV_LENGTH0               (0x1C0C)
#define MCHL_DMV_CFG0                  (0x1C14)
#define MCHL_KEY_L0                    (0x1C18)
#define MCHL_KEY_H0                    (0x1C1C)
#define MCHL_MIC_L0                    (0x1C20)
#define MCHL_MIC_H0                    (0x1C24)
#define MCHL_START1                    (0x1C28)
#define MCHL_DMV_START_MBLK1           (0x1C2C) /* Changed to 23:5 format*/
#define MCHL_DMV_CUR_MBLK1             (0x1C38)
#define MCHL_DMV_OFFSET1               (0x1C30)
#define MCHL_DMV_LENGTH1               (0x1C34)
#define MCHL_DMV_CFG1                  (0x1C3C)
#define MCHL_KEY_L1                    (0x1C40)
#define MCHL_KEY_H1                    (0x1C44)
#define MCHL_MIC_L1                    (0x1C48)
#define MCHL_MIC_H1                    (0x1C4C)
#define MCHL_CTL0                      (0x1C50) /* new name MCHL_CTRL0*/
#define MCHL_CTL1                      (0x1C54) /* new name MCHL_CTRL1*/
#define MCHL_UPPER_BYTE_ADDR0          (0x1C58) /* new*/
#define MCHL_UPPER_BYTE_ADDR1          (0x1C5C) /* new*/

/* SEEPROM registers*/
#define EE_CFG                         (0x0820)
#define EE_CTL                         (0x2000)
#define EE_DATA                        (0x2004)
#define EE_ADDR                        (0x2008)

/* Parallel Host (PCI/CARDBUS/PCMCIA/GS*/
#define CIS_LADDR                      (0x2400)
#define HI_CTL                         (0x2404)
#define LPWR_MGT                       (0x2408)
/*#define PDR0                         (0x04ec)*/
/*#define PDR1                         (0x04f0)*/
/*#define PDR2                         (0x04f4)*/
/*#define PDR3                         (0x04f8)*/
/*#define BAR2_ENABLE                  (0x04fc)*/
/*#define BAR2_TRANS                   (0x0500)*/
/*#define BAR2_MASK                    (0x0504)*/
#define PCI_MEM_SIZE1                  (0x2428)
#define PCI_MEM_OFFSET1                (0x242C)
#define PCI_MEM_OFFSET2                (0x2430)
/*#define PCI_IO_SIZE1                 (0x0514)*/
/*#define PCI_IO_OFFSET1               (0x0518)*/
/*#define PCI_IO_OFFSET2               (0x051c)*/
/*#define PCI_CFG_OFFSET               (0x0520)*/
#define PCMCIA_CFG                     (0x2444)
#define PCMCIA_CTL                     (0x2448)
#define PCMCIA_CFG2                    (0x244C) /* new*/
#define SRAM_PAGE                      (0x2450)
#define CFG_PULLUPDN                   (0x2454)
#define CIS_MAP                        (0x2458) /* new*/
#define ENDIAN_CTRL                    (0x245C) /* new*/
#define GS_SLEEP_ACCESS                (0x2480) /* new*/
#define PCMCIA_PWR_DN                  (0x04C4) 
#define PCI_OUTPUT_DLY_CFG             (0x2464) /* new*/

/* VLYNQ registers*/
/* VLYNQ2 was removed from hardware*/
#define VL1_REV_ID                     (0x8000) /* VLYNQ_REVISION*/
#define VL1_CTL                        (0x8004) /* VLYNQ_ CONTROL*/
#define VL1_STS                        (0x8008) /* VLYNQ_STATUS*/
#define VLYNQ_INTVEC                   (0x800C)
#define VL1_INT_STS                    (0x8010) /* VLYNQ_INTCR*/
#define VL1_INT_PEND                   (0x8014) /* VLYNQ_INTSR*/
#define VL1_INT_PTR                    (0x8018) /* VLYNQ_INTPTR*/
#define VL1_TX_ADDR                    (0x801C) /* VLYNQ_TX_MAP_ADDR*/
#define VL1_RX_SIZE1                   (0x8020) /* VLYNQ_RX_MAP_SIZE1*/
#define VL1_RX_OFF1                    (0x8024) /* VLYNQ_RX_MAP_OFFSET1*/
#define VL1_RX_SIZE2                   (0x8028) /* VLYNQ_RX_MAP_SIZE2*/
#define VL1_RX_OFF2                    (0x802C) /* VLYNQ_RX_MAP_OFFSET2*/
#define VL1_RX_SIZE3                   (0x8030) /* VLYNQ_RX_MAP_SIZE3*/
#define VL1_RX_OFF3                    (0x8034) /* VLYNQ_RX_MAP_OFFSET3*/
#define VL1_RX_SIZE4                   (0x8038) /* VLYNQ_RX_MAP_SIZE4*/
#define VL1_RX_OFF4                    (0x803C) /* VLYNQ_RX_MAP_OFFSET4*/
#define VL1_CHIP_VER                   (0x8040) /* VLYNQ_CHIP_VER*/
#define VLYNQ_AUTONEG                  (0x8044)
#define VLYNQ_MANNEG                   (0x8048)
#define VLYNQ_NEGSTAT                  (0x804C)
#define VLYNQ_ENDIAN                   (0x805C)
#define VL1_INT_VEC3_0                 (0x8060) /* VLYNQ_HW_INT3TO0_CFG*/
#define VL1_INT_VEC7_4                 (0x8064) /* VLYNQ_HW_INT7TO4_CFG*/
/* VLYNQ Remote configuration registers*/
#define VL1_REM_REV_ID                 (0x8080) /* VLYNQ_REM_REVISION*/
#define VL1_REM_CTL                    (0x8084) /* VLYNQ_REM_ CONTROL*/
#define VL1_REM_STS                    (0x8088) /* VLYNQ_REM_STATUS*/
#define VLYNQ_REM_INTVEC               (0x808C)
#define VL1_REM_INT_STS                (0x8090) /* VLYNQ_REM_INTCR*/
#define VL1_REM_INT_PEND               (0x8094) /* VLYNQ_REM_INTSR*/
#define VL1_REM_INT_PTR                (0x8098) /* VLYNQ_REM_INTPTR*/
#define VL1_REM_TX_ADDR                (0x809C) /* VLYNQ_REM_TX_MAP_ADDR*/
#define VL1_REM_RX_SIZE1               (0x80A0) /* VLYNQ_REM_RX_MAP_SIZE1*/
#define VL1_REM_RX_OFF1                (0x80A4) /* VLYNQ_REM_RX_MAP_OFFSET1*/
#define VL1_REM_RX_SIZE2               (0x80A8) /* VLYNQ_REM_RX_MAP_SIZE2*/
#define VL1_REM_RX_OFF2                (0x80AC) /* VLYNQ_REM_RX_MAP_OFFSET2*/
#define VL1_REM_RX_SIZE3               (0x80B0) /* VLYNQ_REM_RX_MAP_SIZE3*/
#define VL1_REM_RX_OFF3                (0x80B4) /* VLYNQ_REM_RX_MAP_OFFSET3*/
#define VL1_REM_RX_SIZE4               (0x80B8) /* VLYNQ_REM_RX_MAP_SIZE4*/
#define VL1_REM_RX_OFF4                (0x80BC) /* VLYNQ_REM_RX_MAP_OFFSET4*/
#define VL1_REM_CHIP_VER               (0x80C0) /* VLYNQ_REM_CHIP_VER*/
#define VLYNQ_REM_AUTONEG              (0x80C4)
#define VLYNQ_REM_MANNEG               (0x80C8)
#define VLYNQ_REM_NEGSTAT              (0x80CC)
#define VLYNQ_REM_ENDIAN               (0x80DC)
#define VL1_REM_INT_VEC3_0             (0x80E0) /* VLYNQ_REM_HW_INT3TO0_CFG*/
#define VL1_REM_INT_VEC7_4             (0x80E4) /* VLYNQ_REM_HW_INT7TO4_CFG*/

/* PCIIF*/
/**/
#define PCI_ID_REG                     (0x8400)
#define PCI_STATUS_SET_REG             (0x8410)
#define PCI_STATUS_CLR_REG             (0x8414)
#define PCI_HIMASK_SET_REG             (0x8420)
#define PCI_HIMASK_CLR_REG             (0x8424)
#define PCI_AMASK_SET_REG              (0x8430)
#define PCI_AMASK_CLR_REG              (0x8434)
#define PCI_CLKRUN_REG                 (0x8438)
#define PCI_BE_VENDOR_ID_REG           (0x8500)
#define PCI_BE_COMMAND_REG             (0x8504)
#define PCI_BE_REVISION_REG            (0x8508)
#define PCI_BE_CL_SIZE_REG             (0x850C)
#define PCI_BE_BAR0_MASK_REG           (0x8510)
#define PCI_BE_BAR1_MASK_REG           (0x8514)
#define PCI_BE_BAR2_MASK_REG           (0x8518)
#define PCI_BE_BAR3_MASK_REG           (0x851C)
#define PCI_BE_CIS_PTR_REG             (0x8528)
#define PCI_BE_SUBSYS_ID_REG           (0x852C)
#define PCI_BE_CAP_PTR_REG             (0x8534)
#define PCI_BE_INTR_LINE_REG           (0x853C)
#define PCI_BE_PM_CAP_REG              (0x8540)
#define PCI_BE_PM_CTRL_REG             (0x8544)
#define PCI_BE_PM_D0_CTRL_REG          (0x8560)
#define PCI_BE_PM_D1_CTRL_REG          (0x8564)
#define PCI_BE_PM_D2_CTRL_REG          (0x8568)
#define PCI_BE_PM_D3_CTRL_REG          (0x856C)
#define PCI_BE_SLV_CFG_REG             (0x8580)
#define PCI_BE_ARB_CTRL_REG            (0x8584)
                                       
#define FER                            (0x85A0) /* PCI_BE_STSCHG_FE_REG*/
#define FEMR                           (0x85A4) /* PCI_BE_STSCHG_FEM_REG*/
#define FPSR                           (0x85A8) /* PCI_BE_STSCHG_FPS_REG*/
#define FFER                           (0x85AC) /* PCI_BE_STSCHG_FFE_REG*/

#define PCI_BE_BAR0_TRANS_REG          (0x85C0)
#define PCI_BE_BAR1_TRANS_REG          (0x85C4)
#define PCI_BE_BAR2_TRANS_REG          (0x85C8)
#define PCI_BE_BAR3_TRANS_REG          (0x85CC)
#define PCI_BE_BAR4_TRANS_REG          (0x85D0)
#define PCI_BE_BAR5_TRANS_REG          (0x85D4)
#define PCI_BE_BAR0_REG                (0x85E0)
#define PCI_BE_BAR1_REG                (0x85E4)
#define PCI_BE_BAR2_REG                (0x85E8)
#define PCI_BE_BAR3_REG                (0x85EC)

#define PCI_PROXY_DATA                 (0x8700)
#define PCI_PROXY_ADDR                 (0x8704)
#define PCI_PROXY_CMD                  (0x8708)
#define PCI_CONTROL                    (0x8710)

/*#define CPC_REGION                   (f0100)*/
/*#define VLYNQ1_BASE                  (f00a0)*/
/*#define VLYNQ2_BASE                  (f00b0)*/
/*#define SCR_IADDR1                   (f00c0)*/
/*#define SCR_IDATA1                   (f00c0)*/
/*#define SCR_IADDR2                   (f00c0)*/
/*#define SCR_IDATA2                   (f00c0)*/

/* SDIO/WSPI*/
#define	CCCR_1                         (002800)
#define	CCCR_2                         (002804)
#define	CCCR_3                         (002808)
#define	FUN_BASE_REG_1                 (00280C)
#define	FUN_BASE_REG_2                 (002810)
#define	FUN_BASE_REG_3                 (002814)
#define	ADDR_MAP_SIZE_1                (002818)
#define	ADDR_MAP_SIZE_2                (002820)
#define	ADDR_MAP_SIZE_3                (002828)
#define	ADDR_MAP_OFFSET_1              (00281C)
#define	ADDR_MAP_OFFSET_2              (002824)
#define	ADDR_MAP_OFFSET_3              (00282C)
#define	ADDR_MAP_OFFSET_4              (002830)
#define	CIS_OFFSET                     (002834)
#define	CSA_OFFSET                     (002838)
#define	DEBUG_REG_1                    (002840)
#define	DEBUG_REG_2                    (00283C)
#define	INTR_MASK                      (002844)
#define	STATUS_REG                     (002848)
#define	WR_ERR_LENGTH                  (00284C)
#define	WR_ERR_ADDR                    (002850)
#define	OCR                            (002858)

/* UART*/
/* TODO - fill in registers*/

/* USB1.1 registers*/
/**/
#define USB_STS_CLR                    (0x4000)
#define USB_STS_ND                     (0x4004)
#define USB_INT_ACK                    (0x4008)
#define USB_MASK                       (0x400c)
#define USB_MASK_SET                   (0x4010)
#define USB_MASK_CLR                   (0x4014)
#define USB_WU                         (0x4018)
#define USB_EP0_OUT_PTR                (0x401c)
#define USB_EP0_OUT_VLD                (0x4020)
#define USB_EP0_OUT_LEN                (0x4024)
#define USB_EP0_IN_PTR                 (0x4028)
#define USB_EP0_IN_VLD                 (0x402c)
#define USB_EP0_IN_LEN                 (0x4030)
#define USB_EP1_CFG                    (0x4034)
#define USB_EP1_OUT_INT_CFG            (0x4038)
#define USB_EP1_OUT_PTR                (0x403c)
#define USB_EP1_OUT_VLD                (0x4040)
#define USB_EP1_OUT_CUR_MBLK           (0x4044)
#define USB_EP1_OUT_LEN                (0x4048)
#define USB_EP1_IN_START_MBLK          (0x404c)
#define USB_EP1_IN_LAST_MBLK           (0x4050)
#define USB_EP1_IN_VLD                 (0x4054)

#define USB_EP2_PTR                    (0x405c)
#define USB_EP2_VLD                    (0x4060)
#define USB_EP2_LEN                    (0x4064)
#define USB_EP3_OUT_PTR0               (0x4068)
#define USB_EP3_OUT_VLD0               (0x406c)
#define USB_EP3_OUT_LEN0               (0x4070)
#define USB_EP3_OUT_PTR1               (0x4074)
#define USB_EP3_OUT_VLD1               (0x4078)
#define USB_EP3_OUT_LEN1               (0x407c)
#define USB_EP3_IN_PTR0                (0x4080)
#define USB_EP3_IN_VLD0                (0x4084)
#define USB_EP3_IN_LEN0                (0x4088)
#define USB_EP3_IN_PTR1                (0x408c)
#define USB_EP3_IN_VLD1                (0x4090)
#define USB_EP3_IN_LEN1                (0x4094)
#define USB_EP1_OUT_END_MBLK           (0x4098)
#define USB_EP0_OUT_SETUP              (0x409c)
#define USB_EP0_STALL                  (0x40a0)
#define USB_EP1_IN_OFFSET              (0x40a4)

/* Device Configuration registers*/
#define SOR_CFG                        (0x0800)
#define ECPU_CTRL                      (0x0804)
#define HI_CFG                         (0x0808)
#define EE_START                       (0x080C)

/* IO Control registers*/
#define SERIAL_HOST_IOCFG0             (0x0894) /* new*/
#define SERIAL_HOST_IOCFG1             (0x0898) /* new*/
#define SERIAL_HOST_IOCFG2             (0x089C) /* new*/
#define SERIAL_HOST_IOCFG3             (0x08A0) /* new*/
#define GPIO_IOCFG0                    (0x08F4) /* new*/
#define GPIO_IOCFG1                    (0x08F8) /* new*/
#define GPIO_IOCFG2                    (0x08FC) /* new*/
#define GPIO_IOCFG3                    (0x0900) /* new*/
#define CHIP_ID_B                      (0x5674) /* new*/
#define CHIP_ID                        CHIP_ID_B/* Leave for TNETW compatability*/
#define CHIP_ID_1251_PG10	           (0x7010101)
#define CHIP_ID_1251_PG11	           (0x7020101)
#define CHIP_ID_1251_PG12	           (0x7030101)

#define SYSTEM                         (0x0810)
#define PCI_ARB_CFG                    (0x0814)
#define BOOT_IRAM_CFG                  (0x0818)
#define ENABLE                         (0x5450)
#define MBLK_CFG                       (0x5460)
#define RS232_BITINTERVAL              (0x0824)
#define TEST_PORT                      (0x096C)
#define DEBUG_PORT                     (0x0970)

/* GPIO registers*/
#define GPIO_OE                        (0x082C) /* 22 GPIOs*/
#define GPIO_OUT                       (0x0834)
#define GPIO_IN                        (0x0830)
#define GPO_CFG                        (0x083C)
#define PWRDN_BUS_L                    (0x0844)
#define PWRDN_BUS_H                    (0x0848)
#define DIE_ID_L                       (0x088C)
#define DIE_ID_H                       (0x0890)

/* Power Management registers*/
/* */
#define ELP_START                      (0x5800)
#define ELP_CFG_MODE                   (0x5804)
#define ELP_CMD                        (0x5808)
#define PLL_CAL_TIME                   (0x5810)
#define CLK_REQ_TIME                   (0x5814)
#define CLK_BUF_TIME                   (0x5818)

#define CFG_PLL_SYNC_CNT               (0x5820) /* Points to the CFG_PLL_SYNC_CNT_xx registers set*/
#define CFG_PLL_SYNC_CNT_I             (0x5820)
#define CFG_PLL_SYNC_CNT_II            (0x5824)
#define CFG_PLL_SYNC_CNT_III           (0x5828)

#define CFG_ELP_SLEEP_CNT              (0x5830) /* Points to the CFG_ELP_SLEEP_CNT_xx registers set*/
#define CFG_ELP_SLEEP_CNT_I            (0x5830)
#define CFG_ELP_SLEEP_CNT_II           (0x5834)
#define CFG_ELP_SLEEP_CNT_III          (0x5838)
#define CFG_ELP_SLEEP_CNT_IV           (0x583c)

#define ELP_SLEEP_CNT                  (0x5840) /* Points to the ELP_SLEEP_CNT_xx registers set*/
#define ELP_SLEEP_CNT_I                (0x5840)
#define ELP_SLEEP_CNT_II               (0x5844)
#define ELP_SLEEP_CNT_III              (0x5848)
#define ELP_SLEEP_CNT_IV               (0x584c)

#define ELP_WAKE_UP_STS                (0x5850)
#define CFG_SLP_CLK_SEL                (0x5860)
#define CFG_SLP_CLK_EN                 (0x5870)

#define CFG_WAKE_UP_EN_I               (0x5880)
#define CFG_WAKE_UP_EN_II              (0x5884)
#define CFG_WAKE_UP_EN_III             (0x5888)

#define CFG_ELP_PWRDN_I                (0x5890)
#define CFG_ELP_PWRDN_II               (0x5894)
#define CFG_ELP_PWRDN_III              (0x5898)

#define CFG_POWER_DOWN_I               (0x58a0)
#define CFG_POWER_DOWN_II              (0x58a4)
#define CFG_POWER_DOWN_III             (0x58a8)

#define CFG_BUCK_TESTMODE_I            (0x58b0)
#define CFG_BUCK_TESTMODE_II           (0x58b4)

#define POWER_STATUS_I                 (0x58C0)
#define POWER_STATUS_II                (0x58C4)

#define DIGLDO_BIAS_PROG_I             (0x58d0)
#define DIGLDO_BIAS_PROG_II            (0x58d4)

#define LDO2P8_BIAS_PROG_I             (0x58e0)
#define LDO2P8_BIAS_PROG_II            (0x58e4)

#define ADCLDO_BIAS_PROG               (0x58f0)

#define REFSYS_PROG_I                  (0x5910)
#define REFSYS_PROG_II                 (0x5914)

#define PM_TEST_I                      (0x5920)
#define PM_TEST_II                     (0x5924)

#define POR_PROG                       (0x5930)

#define TEST_PIN_DIR_I                 (0x5940)
#define TEST_PIN_DIR_II                (0x5944)

#define PROC_CTL                       (0x5950)

#define ADC_REF_WAKEUP_I               (0x5960)
#define ADC_REF_WAKEUP_II              (0x5964)
#define ADC_REF_WAKEUP_III             (0x5968)
#define ADC_REF_WAKEUP_IV              (0x596C)

#define VREG_WAKEUP_I                  (0x5970)
#define VREG_WAKEUP_II                 (0x5974)
#define VREG_WAKEUP_III                (0x5978)
#define VREG_WAKEUP_IV                 (0x597C)

#define PLL_WAKEUP_I                   (0x5980)
#define PLL_WAKEUP_II                  (0x5984)
#define PLL_WAKEUP_III                 (0x5988)
#define PLL_WAKEUP_IV                  (0x598C)

#define XTALOSC_WAKEUP_I               (0x5990)
#define XTALOSC_WAKEUP_II              (0x5994)
#define XTALOSC_WAKEUP_III             (0x5998)
#define XTALOSC_WAKEUP_IV              (0x599C)

/* ----------*/

#define POWER_MGMT2                    (0x0840)
#define POWER_MGMT                     (0x5098)
#define MAC_HW_DOZE                    (0x090c)
#define ECPU_SLEEP                     (0x0840)
#define DOZE_CFG                       (0x54bc)
#define DOZE2_CFG                      (0x081c)
#define WAKEUP_CFG                     (0x54c0)
#define WAKEUP_TIME_L                  (0x54c8)
#define WAKEUP_TIME_H                  (0x54c4)

/**/

/*#define CPU_WAIT_CFG                 (f0020)*/
/*#define CFG_QOS_ACM                  (f0046)*/

/* Scratch Pad registers*/
#define SCR_PAD0                       (0x5608)
#define SCR_PAD1                       (0x560C)
#define SCR_PAD2                       (0x5610)
#define SCR_PAD3                       (0x5614)
#define SCR_PAD4                       (0x5618)
#define SCR_PAD4_SET                   (0x561C)
#define SCR_PAD4_CLR                   (0x5620)
#define SCR_PAD5                       (0x5624)
#define SCR_PAD5_SET                   (0x5628)
#define SCR_PAD5_CLR                   (0x562C)
#define SCR_PAD6                       (0x5630)
#define SCR_PAD7                       (0x5634)
#define SCR_PAD8                       (0x5638)
#define SCR_PAD9                       (0x563C)

/* Spare registers*/
#define SPARE_A1                       (0x0994)
#define SPARE_A2                       (0x0998)
#define SPARE_A3                       (0x099C)
#define SPARE_A4                       (0x09A0)
#define SPARE_A5                       (0x09A4)
#define SPARE_A6                       (0x09A8)
#define SPARE_A7                       (0x09AC)
#define SPARE_A8                       (0x09B0)
#define SPARE_B1                       (0x5420)
#define SPARE_B2                       (0x5424)
#define SPARE_B3                       (0x5428)
#define SPARE_B4                       (0x542C)
#define SPARE_B5                       (0x5430)
#define SPARE_B6                       (0x5434)
#define SPARE_B7                       (0x5438)
#define SPARE_B8                       (0x543C)

/* RMAC registers (Raleigh MAC)*/

/* Station registers*/
#define DEV_MODE                       (0x5464)
#define STA_ADDR_L                     (0x546C)
#define STA_ADDR_H                     (0x5470)
#define BSSID_L                        (0x5474)
#define BSSID_H                        (0x5478)
#define AID_CFG                        (0x547C)
#define BASIC_RATE_CFG                 (0x4C6C)
#define BASIC_RATE_TX_CFG              (0x55F0)

/* Protocol timers registers*/
#define IFS_CFG0                       (0x5494)
#define IFS_CFG1                       (0x5498)
#define TIMEOUT_CFG                    (0x549C)
#define CONT_WIND_CFG                  (0x54A0)
#define BCN_INT_CFG                    (0x54A4)
#define RETRY_CFG                      (0x54A8)
#define DELAY_CFG                      (0x54B0)

/* Hardware Override registers*/
#define CCA_CFG                        (0x54CC)
#define CCA_FILTER_CFG                 (0x5480)
#define RADIO_PLL_CFG                  (0x555C)
#define CCA_MON                        (0x54D0)
#define TX_FRM_CTL                     (0x54D4)
#define CONT_TX_EN                     (0x50EC)
#define PHY_STANDBY_EN                 (0x5668)

/* Transmit Setup registers*/
#define TX_PING_PONG                   (0x5090)
#define TX_CFG0                        (0x5000)
#define TX_CFG1                        (0x5004)
#define TX_CFG2                        (0x5008)
#define MAX_LIFETIME                   (0x50FC)
#define TX_PANG_SEL                    (0x50E0)
#define TX_PANG0                       (0x50A0)
#define TX_PING0                       (0x5010)
#define TX_PONG0                       (0x5050)
#define TX_PANG1                       (0x50A4)
#define TX_PING1                       (0x5014)
#define TX_PONG1                       (0x5054)
#define TX_PANG2                       (0x50A8)
#define TX_PING2                       (0x5018)
#define TX_PONG2                       (0x5058)
#define TX_PANG3                       (0x50AC)
#define TX_PING3                       (0x501C)
#define TX_PONG3                       (0x505C)
#define TX_PANG4                       (0x50B0)
#define TX_PING4                       (0x5020)
#define TX_PONG4                       (0x5060)
#define TX_PANG5                       (0x50B4)
#define TX_PING5                       (0x5024)
#define TX_PONG5                       (0x5064)
#define TX_PANG6                       (0x50B8)
#define TX_PING6                       (0x5028)
#define TX_PONG6                       (0x5068)
#define TX_PANG7                       (0x50BC)
#define TX_PING7                       (0x502C)
#define TX_PONG7                       (0x506C)
#define TX_PANG8                       (0x50C0)
#define TX_PING8                       (0x5030)
#define TX_PONG8                       (0x5070)
#define TX_PANG9                       (0x50C4)
#define TX_PING9                       (0x5034)
#define TX_PONG9                       (0x5074)
#define TX_PANG10                      (0x50C8)
#define TX_PING10                      (0x5038)
#define TX_PONG10                      (0x5078)
#define TX_PANG11                      (0x50CC)
#define TX_PING11                      (0x503C)
#define TX_PONG11                      (0x507C)

/* Transmit Status registers*/
#define TX_STATUS                      (0x509C)
#define TX_PANG_EXCH                   (0x50D0)
#define TX_PING_EXCH                   (0x5040)
#define TX_PONG_EXCH                   (0x5080)
#define TX_PANG_ATT                    (0x50D4)
#define TX_PING_ATT                    (0x5044)
#define TX_PONG_ATT                    (0x5084)
#define TX_PANG_TIMESTAMP              (0x50DC)
#define TX_PING_TIMESTAMP              (0x504C)
#define TX_PONG_TIMESTAMP              (0x508C)

/* Transmit State registers*/
#define TX_STATE                       (0x5094)
#define TX_PANG_OVRD_CFG               (0x50D8)
#define TX_PING_OVRD_CFG               (0x5048)
#define TX_PONG_OVRD_CFG               (0x5088)
#define TX_HOLD_CFG                    (0x54D8)
#define TSF_ADJ_CFG1                   (0x54DC)
#define TSF_ADJ_CFG2                   (0x54E0)
#define TSF_ADJ_CFG3                   (0x54E4)
#define TSF_ADJ_CFG4                   (0x54E8)
#define CFG_OFDM_TIMES0                (0x5648)
#define CFG_OFDM_TIMES1                (0x564C)

/* Beacon/Probe Response registers*/
#define PRB_ADDR                       (0x54EC)
#define PRB_LENGTH                     (0x54F0)
#define BCN_ADDR                       (0x54F4)
#define BCN_LENGTH                     (0x54F8)
#define TIM_VALID0                     (0x54FC)
#define TIM_ADDR0                      (0x5500)
#define TIM_LENGTH0                    (0x5504)
#define TIM_VALID1                     (0x5654)
#define TIM_ADDR1                      (0x5658)
#define TIM_LENGTH1                    (0x565C)
#define TIM_SELECT                     (0x5660)
#define TSF_CFG                        (0x5508)

/* Other Hardware Generated Frames regi*/
#define CTL_FRM_CFG                    (0x550C)
#define MGMT_FRM_CFG                   (0x5510)
#define CFG_ANT_SEL                    (0x5664)
#define RMAC_ADDR_BASE                 (0x5680) /* new*/

/* Protocol Interface Read Write Interf*/
#define TXSIFS_TIMER                   (0x4C00)
#define TXPIFS_TIMER                   (0x4C04)
#define TXDIFS_TIMER                   (0x4C08)
#define SLOT_TIMER                     (0x4C0C)
#define BACKOFF_TIMER                  (0x4C10)
#define BCN_PSP_TIMER                  (0x4C14)
#define NAV                            (0x4C18)
#define TSF_L                          (0x4C1C)
#define TSF_H                          (0x4C20)
#define TSF_PREV_L                     (0x4CC4) /* new */
#define TSF_PREV_H                     (0x4CC8) /* new */
#define TOUT_TIMER                     (0x4C2C)
#define NEXT_TBTT_L                    (0x4C30)
#define NEXT_TBTT_H                    (0x4C34)
#define DTIM_CNT                       (0x4C38)
#define CONT_WIND                      (0x4C3C)
#define PRSP_REQ                       (0x4C40)
#define PRSP_DA_L                      (0x4C44)
#define PRSP_DA_H                      (0x4C48)
#define PRSP_RETRY                     (0x4C4C)
#define PSPOLL_REQ                     (0x4C50)
#define NEXT_SEQ_NUM                   (0x4C54)
#define PRSP_SEQ_NUM                   (0x4C58)
#define BCN_SEQ_NUM                    (0x4C5C)
#define MED_USAGE                      (0x4C24)
#define MED_USAGE_TM                   (0x4C28)
#define PRB_DLY                        (0x4C60)
#define STA_SRC                        (0x4C64)
#define STA_LRC                        (0x4C68)
#define CFG_ACM                        (0x4C70)
#define RAND_NUMB                      (0x4C6C)
#define CFG_ACK_CTS_DOT11A             (0x4C74)
#define CFG_ACK_CTS_DOT11B             (0x4C78)
#define ACM_IFS_CFG0                   (0x4C7C)
#define ACM_IFS_CFG1                   (0x4C80)
#define ACM_IFS_CFG2                   (0x4C84)
#define ACM_IFS_CFG3                   (0x4C88)
#define ACK_CTS_FRM_CFG                (0x4C8C)
#define CFG_RX_TSTMP_DLY0              (0x4C90)
#define CFG_RX_TSTMP_DLY1              (0x4C94)
#define CFG_RX_TSTMP_DLY2              (0x4C98)
#define CFG_RX_TSTMP_DLY3              (0x4C9C)
#define CCA_BUSY                       (0x4CA0)
#define CCA_BUSY_CLR                   (0x4CA4)
#define CCA_IDLE                       (0x4CA8)
#define CCA_IDLE_CLR                   (0x4CAC)

/* Receive Manager registers*/
#define RX_HEAD_PTR                    (0x567C) /* new*/
#define RX_TAIL_PTR                    (0x4898) /* new*/
#define RX_CURR_PTR                    (0x5678) /* new*/
#define RX_RESET                       (0x4800)
#define RX_MODMODE                     (0x4838) /* new*/
#define MAC_HEADER_BYTECNT             (0x4890)
#define RX_MAC_BYTECNT_INT             (0x489C)
#define MAC_HEADER_WORD0               (0x4868)
#define MAC_HEADER_WORD1               (0x486C)
#define MAC_HEADER_WORD2               (0x4870)
#define MAC_HEADER_WORD3               (0x4874)
#define MAC_HEADER_WORD4               (0x4878)
#define MAC_HEADER_WORD5               (0x487C)
#define MAC_HEADER_WORD6               (0x4880)
#define MAC_HEADER_WORD7               (0x4884)
#define MAC_HEADER_WORD8               (0x4888)
#define MAC_HEADER_WORD9               (0x488C)
#define RX_CFG                         (0x5514)
#define RX_FILTER_CFG                  (0x55B4)
#define RX_MC0_L                       (0x5518)
#define RX_MC0_H                       (0x551C)
#define RX_MC1_L                       (0x5520)
#define RX_MC1_H                       (0x5524)
#define STA_SSID0                      (0x4804)
#define STA_SSID1                      (0x4808)
#define STA_SSID2                      (0x480C)
#define STA_SSID3                      (0x4810)
#define STA_SSID4                      (0x4814)
#define STA_SSID5                      (0x4818)
#define STA_SSID6                      (0x481C)
#define STA_SSID7                      (0x4820)
#define SSID_LEN                       (0x4824)
#define RX_FREE_MEM                    (0x5528)
#define RX_CURR_MEM                    (0x552C)
#define MAC_TIMESTAMP                  (0x5560) /* Check place*/
#define RX_TIMESTAMP                   (0x5564)
#define RX_FRM_PTR                     (0x5568)
#define RX_FRM_LEN                     (0x556C)
#define RX_PLCP_HDR                    (0x5570)
#define RX_PLCP_SIGNAL                 (0x5574)
#define RX_PLCP_SERVICE                (0x5578) /* 16 bits ?*/
#define RX_PLCP_LENGTH                 (0x557C)
#define RX_FRM_CTL                     (0x5580)
#define RX_DUR_ID                      (0x5584)
#define RX_ADDR1_L                     (0x5588)
#define RX_ADDR1_H                     (0x558C)
#define RX_ADDR2_L                     (0x5590)
#define RX_ADDR2_H                     (0x5594)
#define RX_ADDR3_L                     (0x5598)
#define RX_ADDR3_H                     (0x559C)
#define RX_SEQ_CTL                     (0x55A0)
#define RX_WEP_IV                      (0x55A4)
#define RX_TIME_L                      (0x55A8)
#define RX_TIME_H                      (0x55AC)
#define RX_STATUS                      (0x55B0)
#define PLCP_ERR_CNT                   (0x4828)
#define FCS_ERR_CNT                    (0x482C)
#define RX_OVERFLOW_CNT                (0x4830)
#define RX_DEBUG1                      (0x4858)
#define RX_DEBUG2                      (0x485C)
#define RX_QOS_CFG                     (0x4848)
#define RX_QOS_CTL                     (0x4844)
#define RX_QOS_STATUS                  (0x4854) /* new name RX_QOS_STS*/
#define RX_TXOP_HOLDER_L               (0x484C)
#define RX_TXOP_HOLDER_H               (0x4850)
#define RX_FRM_CNT                     (0x4834) /* what is RX_FRM_CTR*/
#define CONS_FCS_ERR_CNT               (0x483C)
#define CONS_FCS_ERR_CFG               (0x4840)
#define RX_QOS_CTL_MASK                (0x48A0) /* new*/
#define RX_QOS_ACK_EN                  (0x48A4) /* new*/
#define RX_QOS_NOACK_EN                (0x48A8) /* new*/
#define RX_QOS_ACK_BITMAP              (0x48AC) /* new*/

/* Baseband Processor registers*/
#define SBB_CFG                        (0x55C8)
#define SBB_ADDR                       (0x55D0)
#define SBB_DATA                       (0x55D4)
#define SBB_CTL                        (0x55D8)

/* Radio Control Interface registers*/
#define RCI_CTL                        (0x55DC)
#define RCI_DATA                       (0x55E0)
#define RCI_CFG1                       (0x55E4)
#define RCI_CFG2                       (0x55E8)
#define RCI_CFG3                       (0x55EC)

#define TNET1150_LAST_REG_ADDR			PCI_CONTROL


/* Missing registers*/


#endif

/* mmc_tnetw1150_api.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright © Texas Instruments Incorporated (Oct 2005)
 * THIS CODE/PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDED BUT NOT LIMITED TO , THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * This program has been modified from its original operation by Texas
 * Instruments Incorporated. These changes are covered under version 2
 * of the GNU General Public License, dated June 1991.
 *
 * Copyright © Google Inc (Feb 2008)
 */
/*-------------------------------------------------------------------*/
#ifdef TIWLAN_MSM7000

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include "mmc_tnetw1150_api.h"
#include <linux/mmc/core.h>

#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>

#define SDIO_INVALID_PERIPHERAL_ADDRESS             0x1FFFF

/* CCCR                                                 0x000000 - 0x0000ff          */
/* Function Basic Register (Function 1)                 0x000100 - 0x0001ff          */
	/* 0x101          - Function 1 Extended standard I/O device type code        */
	/* 0x102          - RFU[4-0] EnableHighPower Supports High-Power[1-0] SHP[0] */
	/* 0x103 - 0x108  - RFU                                                      */
	/* 0x109 - 0x10b  - Pointer to Function 1 Card Information Structure         */
	/* 0x10c - 0x10e  - Pointer to Function 1 Code Storage Area                  */
	/* 0x10f          - Data access window to Function 1 Code Storage Area       */
	/* 0x110 - 0x111  - I/O block size for Function 1                            */
	/* 0x112 - 0x1ff  - RFU                                                      */
/* Function Basic Register (Function 2)                 0x000200 - 0x0002ff          */
/* Function Basic Register (Function 3)                 0x000300 - 0x0003ff          */
/* Function Basic Register (Function 4)                 0x000400 - 0x0004ff          */
/* Function Basic Register (Function 5)                 0x000500 - 0x0005ff          */
/* Function Basic Register (Function 6)                 0x000600 - 0x0006ff          */
/* Function Basic Register (Function 7)                 0x000700 - 0x0007ff          */
/* RFU                                			0x000800 - 0x000fff          */
/* CIS common and per-function area                     0x001000 - 0x017fff          */
/* RFU                                                  0x018000 - 0x01ffff          */
#define SDIO_FUNC1_OFFSET    0x1FFC0

typedef struct
{
	Peripheral_Address mem_start_addr;
	Peripheral_Address mem_part_size;
	Peripheral_Address mem_end_addr;
	Peripheral_Address reg_start_addr;
	Peripheral_Address reg_part_size;
	Peripheral_Address reg_end_addr;
} SDIO_TNETW_partitions;
static SDIO_TNETW_partitions TNETW_table;

static SDIO_TNETWConfigParams TNETW_params;

#ifdef CONFIG_MMC_TNET_INFO
typedef struct {
	u8  scr_space[SDIO_FUNC1_OFFSET];	/* 0x000000 - 0x01ffbf;
				   		   131072B(128kB)-64B=131008B(0x01ffc0) */
	u32 amap_size1;         		/* 0x01ffc0 */
	u32 amap_offset1;       		/* 0x01ffc4 */
	u32 amap_size2;         		/* 0x01ffc8 */
	u32 amap_offset2;       		/* 0x01ffcc */
	u32 amap_size3;         		/* 0x01ffd0 */
	u32 amap_offset3;       		/* 0x01ffd4 */
	u32 amap_offset4;       		/* 0x01ffd8 */
	u32 cis_offset;         		/* 0x01ffdc - Card Information Structure */
	u32 csa_offset;         		/* 0x01ffe0 - Code Storage Area */
	u8  filler[16];         		/* 0x01ffe4 - 0x01fff3 */
	u32 wr_err_len;         		/* 0x01fff4 */
	u32 wr_err_addr;        		/* 0x01fff8 */
	u32 status;             		/* 0x01fffc */
} SDIO_FUNC1AddressMap;
SDIO_FUNC1AddressMap mapping;

int sdio_tnetw1150_dump(int count)
{
	unsigned long from;
	unsigned long br_offset;
	unsigned long offset=0;
	unsigned long p1_offset=0;
	unsigned long p2_offset=0;
	struct mmc_request request;
	int lines;
	int i=0;
	int second_part=0;

	printk("%s:\n", __FUNCTION__);
#define TNETW1150_OFFSET 1024
	from=TNETW1150_OFFSET*count;
	if((from+TNETW1150_OFFSET)>(SDIO_FUNC1_OFFSET+64))
		return -1;
	br_offset=TNETW1150_OFFSET*count;
	if(br_offset>=TNETW_params.map_reg[0].reg_size) {
		second_part=1;
		p2_offset = SDIO_DRIVER_REG_PARTITION_START;
		offset = TNETW_params.map_reg[1].scr_offset - TNETW_params.map_reg[0].reg_size;
	}
	else {
		p1_offset += SDIO_DOWNLOAD_PARTITION_START;
		offset = TNETW_params.map_reg[0].scr_offset;
	}

	request.cmd=SD_IO_RW_DIRECT;
 	request.buffer_len=1;
	request.nob=0;
	request.block_len=0;
	request.buffer=&mapping.scr_space[from];
    printk(" TNETW1150 try br_offset:0x%08lx offset:0x%08lx):\n", br_offset, offset);
	do {
		request.arg = SDIO_CMD52_READ(0,FUNCTION_SELECT_1,0,br_offset);
		if(__submit_control_request(omap_mmc_dev_get_handle(), &request)!=SDIO_SUCCESS)
			return SDIO_FAILURE;
		udelay(1);
		*request.buffer = (char)omap_readw(OMAP_MMC_RSP6);
		i++;
		request.buffer++;
		br_offset++;
	} while(i<TNETW1150_OFFSET);

    printk(" TNETW1150 SCR Address Space (start:0x%08lx  end:0x%08lx part:%d offset:0x%08lx):\n", from, from+TNETW1150_OFFSET-1, second_part+1, (!second_part)?p1_offset:p2_offset);

	lines=TNETW1150_OFFSET/16;
	for(i=0; i<lines;i++, from+=16)
        printk("SDIO:0x%04lx(SCR:0x%04lx): %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n", from, (!second_part)?(from+p1_offset):(from-p2_offset+SDIO_REG_PARTITION_START), mapping.scr_space[from], mapping.scr_space[from+1], mapping.scr_space[from+2], mapping.scr_space[from+3], mapping.scr_space[from+4], mapping.scr_space[from+5], mapping.scr_space[from+6], mapping.scr_space[from+7], mapping.scr_space[from+8], mapping.scr_space[from+9], mapping.scr_space[from+10], mapping.scr_space[from+11], mapping.scr_space[from+12], mapping.scr_space[from+13], mapping.scr_space[from+14], mapping.scr_space[from+15]);

    return 0;
}
EXPORT_SYMBOL(sdio_tnetw1150_dump);

int sdio_tnetw1150_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len=0;
	int br_offset = SDIO_FUNC1_OFFSET;
	struct mmc_request request;
	u8 buf[64];
	u8 buf3[3];
	int bytes_to_read=64;
	int i;
#define CCCR_SIZE 17
	u8 buf_ccr[CCCR_SIZE];
	u8 buf_ccr_offset[CCCR_SIZE] = {
		CCCR_SDIO_REVISION, CCCR_SD_SPECIFICATION_REVISION, CCCR_IO_ENABLE,
		CCCR_IO_READY, CCCR_INT_ENABLE, CCCR_INT_PENDING,
		CCCR_IO_ABORT, CCCR_BUS_INTERFACE_CONTOROL, CCCR_CARD_CAPABILITY,
		CCCR_COMMON_CIS_POINTER, CCCR_COMMON_CIS_POINTER+1, CCCR_COMMON_CIS_POINTER+2,
		CCCR_BUS_SUSPEND, CCCR_FUNCTION_SELECT, CCCR_EXEC_FLAGS,
		CCCR_READY_FLAGS, CCCR_FNO_BLOCK_SIZE,
	};
#define FBR_SIZE 9
	u8 buf_fbr[FBR_SIZE];
	u16 buf_fbr_offset[FBR_SIZE] = {
		FBR_PTR_F1_CIS, FBR_PTR_F1_CIS+1, FBR_PTR_F1_CIS+2,
		FBR_PTR_F1_CSA, FBR_PTR_F1_CSA+1, FBR_PTR_F1_CSA+2,
		FBR_WIN_F1_CSA, FBR_F1_IO_BLK_SIZE, FBR_F1_IO_BLK_SIZE+1,
	};

	request.cmd=SD_IO_RW_DIRECT;
 	request.buffer_len=1;
	request.nob=0;
	request.block_len=0;
	request.buffer=&buf[0];
	for (i=0; i<bytes_to_read;i++,request.buffer++,br_offset++) {
		request.arg = SDIO_CMD52_READ(0,FUNCTION_SELECT_1,0,br_offset);
		if(__submit_control_request(omap_mmc_dev_get_handle(), &request)!=SDIO_SUCCESS)
			return SDIO_FAILURE;
		*request.buffer = (char)omap_readw(OMAP_MMC_RSP6);
	}
	memcpy(&mapping.amap_size1, &buf, 64);

    count -= 80; /* some reserve */
    len += (len<count)?sprintf(page+len, " TNETW1150 partitions:\n"):0;
    len += (len<count)?sprintf(page+len, " Memory:      "):0;
	len += (len<count)?sprintf(page+len, " start:0x%08lx  end:0x%08lx  size:0x%08lx bytes\n", TNETW_table.mem_start_addr, TNETW_table.mem_end_addr, TNETW_table.mem_part_size):0;
    len += (len<count)?sprintf(page+len, " Registers:   "):0;
	len += (len<count)?sprintf(page+len, " start:0x%08lx  end:0x%08lx  size:0x%08lx bytes\n", TNETW_table.reg_start_addr, TNETW_table.reg_end_addr, TNETW_table.reg_part_size):0;

    len += (len<count)?sprintf(page+len, " SDIO Function 1 Address Map:\n"):0;
	len += (len<count)?sprintf(page+len, " amap_size1:0x%08x  amap_offset1:0x%08x\n", mapping.amap_size1, mapping.amap_offset1):0;
	len += (len<count)?sprintf(page+len, " amap_size2:0x%08x  amap_offset2:0x%08x\n", mapping.amap_size2, mapping.amap_offset2):0;
	len += (len<count)?sprintf(page+len, " amap_size3:0x%08x  amap_offset3:0x%08x\n", mapping.amap_size3, mapping.amap_offset3):0;
	len += (len<count)?sprintf(page+len, "                        amap_offset4:0x%08x\n", mapping.amap_offset4):0;
	len += (len<count)?sprintf(page+len, " cis_offset:0x%08x  csa_offset  :0x%08x\n", mapping.cis_offset, mapping.csa_offset):0;
	len += (len<count)?sprintf(page+len, " wr_err_len:0x%08x  wr_err_addr :0x%08x\n", mapping.wr_err_len, mapping.wr_err_addr):0;
	len += (len<count)?sprintf(page+len, " status    :0x%08x\n", mapping.status):0;

	request.cmd=SD_IO_RW_DIRECT;
 	request.buffer_len=1;
	request.nob=0;
	request.block_len=0;
	request.buffer=&buf_ccr[0];
	for (i=0; i<CCCR_SIZE;i++,request.buffer++) {
		request.arg = SDIO_CMD52_READ(0,FUNCTION_SELECT_0,0,buf_ccr_offset[i]);
		if(__submit_control_request(omap_mmc_dev_get_handle(), &request)!=SDIO_SUCCESS)
			return SDIO_FAILURE;
		*request.buffer = (char)omap_readw(OMAP_MMC_RSP6);
	}

    len += (len<count)?sprintf(page+len, "\n Card Common Control Registers:\n"):0;
	len += (len<count)?sprintf(page+len, " \
 REVISION             :0x%02x  SD_SPEC_REVISION       :0x%02x   IO_ENABLE            :0x%02x\n \
 IO_READY             :0x%02x  INT_ENABLE             :0x%02x   INT_PENDING          :0x%02x\n \
 IO_ABORT             :0x%02x  BUS_INTERFACE_CONTOROL :0x%02x   CARD_CAPABILITY      :0x%02x\n \
 COMMON_CIS_POINTER[2]:0x%02x  COMMON_CIS_POINTER[1]  :0x%02x   COMMON_CIS_POINTER[0]:0x%02x\n \
 BUS_SUSPEND          :0x%02x  FUNCTION_SELECT        :0x%02x   EXEC_FLAGS           :0x%02x\n \
 READY_FLAGS          :0x%02x  FNO_BLOCK_SIZE         :0x%02x\n" , buf_ccr[0], buf_ccr[1], buf_ccr[2], buf_ccr[3], buf_ccr[4], buf_ccr[5], buf_ccr[6], buf_ccr[7], buf_ccr[8], buf_ccr[11], buf_ccr[10], buf_ccr[9], buf_ccr[12], buf_ccr[13], buf_ccr[14], buf_ccr[15], buf_ccr[16]):0;

	request.cmd=SD_IO_RW_DIRECT;
 	request.buffer_len=1;
	request.nob=0;
	request.block_len=0;

	request.buffer=&buf3[0];
	for (i=0; i<3;i++,request.buffer++) {
		request.arg = SDIO_CMD52_READ(0,FUNCTION_SELECT_1,0,FBR_PTR_F1_IO_DEV);
		if(__submit_control_request(omap_mmc_dev_get_handle(), &request)!=SDIO_SUCCESS)
			return SDIO_FAILURE;
		*request.buffer = (char)omap_readw(OMAP_MMC_RSP6);
	}

	request.buffer=&buf_fbr[0];
	for (i=0; i<FBR_SIZE;i++,request.buffer++) {
		request.arg = SDIO_CMD52_READ(0,FUNCTION_SELECT_1,0,buf_fbr_offset[i]);
		if(__submit_control_request(omap_mmc_dev_get_handle(), &request)!=SDIO_SUCCESS)
			return SDIO_FAILURE;
		*request.buffer = (char)omap_readw(OMAP_MMC_RSP6);
	}

    len += (len<count)?sprintf(page+len, "\n Function Basic Registers(Func 1):\n"):0;
	len += (len<count)?sprintf(page+len, " \
 Func 1 CSA enable and CSA support              :     0x%02x 0x%02x\n \
 Func 1 Ext stand. I/O device i/f and type code :     0x%02x 0x%02x\n \
 EHP|SHP[1]|SHP[0]                              :          0x%02x\n \
 Pointer to Func 1 Card Information Structure   :0x%02x 0x%02x 0x%02x\n \
 Pointer to Func 1 Code Storage Area            :0x%02x 0x%02x 0x%02x\n \
 Data access window to Func 1 Code Storage Area :          0x%02x\n \
 I/O block size for Func 1                      :     0x%02x 0x%02x\n", ((buf3[0]&0x80)>>7), ((buf3[0]&0x40)>>6), (buf3[1]&0x0f), buf3[1], (buf3[2]&0x07), buf_fbr[2], buf_fbr[1], buf_fbr[0], buf_fbr[5], buf_fbr[4], buf_fbr[3], buf_fbr[6], buf_fbr[8], buf_fbr[7]):0;

    *eof = 1;
    return len;
}
#endif /*CONFIG_MMC_TNET_STATISTICS*/


/*
  Initialization of TNETW memory configuration.
*/
extern int debug_level;
SDIO_Status SDIO_TNETWInit(SDIO_TNETWConfigParams *params)
{
    /*	debug_level=3;    */   
	/* printk("%s\n", __FUNCTION__); */

	memset(&TNETW_params, 0, sizeof(SDIO_TNETWConfigParams));

    if(!params) {
		/* printk("%s set to default\n", __FUNCTION__); */
		/* set to default value in case params is not presented */
		TNETW_params.num_of_parts = 2;
		/* First time initialization */
		TNETW_params.map_reg[0].reg_size = SDIO_DOWNLOAD_PARTITION_SIZE;
		TNETW_params.map_reg[0].scr_offset = SDIO_DOWNLOAD_PARTITION_START;
		/* After firmware has been downloaded, data memory region
		   has to be re-initialized as following:
		TNETW_params.map_reg[0].reg_size = SDIO_MEM_PARTITION_START;
		TNETW_params.map_reg[0].scr_offset = SDIO_MEM_PARTITION_SIZE;
		*/
		TNETW_params.map_reg[1].reg_size = SDIO_REG_PARTITION_SIZE;
		TNETW_params.map_reg[1].scr_offset = SDIO_REG_PARTITION_START;
	}
	else {
		/* printk("%s: params->num_of_parts=%d\n", __FUNCTION__, params->num_of_parts); */
		/* validate input parameters */
		switch(params->num_of_parts) {
		case 1:
			if(params->map_reg[0].reg_size > AMAP_ONE_REGION)
				return SDIO_FAILURE;
			break;
		case 2:
			if((params->map_reg[0].reg_size + params->map_reg[1].reg_size) > AMAP_ONE_REGION)
				return SDIO_FAILURE;
			break;
		case 3:
			if((params->map_reg[0].reg_size + params->map_reg[1].reg_size + params->map_reg[2].reg_size) > AMAP_ONE_REGION)
				return SDIO_FAILURE;
			break;
		case 4:
			if((params->map_reg[0].reg_size + params->map_reg[1].reg_size + params->map_reg[2].reg_size + params->map_reg[3].reg_size) > AMAP_ONE_REGION)
				return SDIO_FAILURE;
			break;
		default:
			return SDIO_FAILURE;
		}
		memcpy(&TNETW_params, params, sizeof(SDIO_TNETWConfigParams));
	}

#ifdef CONFIG_PROC_FS
#ifdef CONFIG_MMC_TNET_INFO
	create_proc_read_entry("sdio_tnetw1150", 0, NULL, sdio_tnetw1150_read_proc, NULL);
#endif
#endif

	/* printk("%s completed\n", __FUNCTION__); */

	return SDIO_SUCCESS;
}

SDIO_Status SDIO_TNETWReset(SDIO_TNETWConfigParams *params)
{
	/* printk("%s\n", __FUNCTION__); */

	memset(&TNETW_params, 0, sizeof(SDIO_TNETWConfigParams));

	if(!params) {
		/* printk("%s set to default\n", __FUNCTION__); */
		/* set to default value in case params is not presented */
		TNETW_params.num_of_parts = 2;

		/* After firmware has been downloaded, data memory region
		   has to be re-initialized as following */
		TNETW_params.map_reg[0].reg_size = SDIO_MEM_PARTITION_START;
		TNETW_params.map_reg[0].scr_offset = SDIO_MEM_PARTITION_SIZE;
		TNETW_params.map_reg[1].reg_size = SDIO_REG_PARTITION_SIZE;
		TNETW_params.map_reg[1].scr_offset = SDIO_REG_PARTITION_START;
	}
	else {
		/* printk("%s: params->num_of_parts=%d\n", __FUNCTION__, params->num_of_parts); */
		/* validate input parameters */
		switch(params->num_of_parts) {
		case 1:
			if(params->map_reg[0].reg_size > AMAP_ONE_REGION)
				return SDIO_FAILURE;
			break;
		case 2:
			if((params->map_reg[0].reg_size + params->map_reg[1].reg_size) > AMAP_ONE_REGION)
				return SDIO_FAILURE;
			break;
		case 3:
			if((params->map_reg[0].reg_size + params->map_reg[1].reg_size + params->map_reg[2].reg_size) > AMAP_ONE_REGION)
				return SDIO_FAILURE;
			break;
		case 4:
			if((params->map_reg[0].reg_size + params->map_reg[1].reg_size + params->map_reg[2].reg_size + params->map_reg[3].reg_size) > AMAP_ONE_REGION)
				return SDIO_FAILURE;
			break;
		default:
			return SDIO_FAILURE;
		}
		memcpy(&TNETW_params, params, sizeof(SDIO_TNETWConfigParams));
	}

	/* printk("%s completed\n", __FUNCTION__); */

	return SDIO_SUCCESS;
}


static SDIO_Status config_partition(SDIO_Handle sdioHandle, int partition_no, Peripheral_Address start_addr, SDIO_BufferLength part_size)
{
	struct sdio_func *func = (struct sdio_func *) sdioHandle;
	u8 data;
	int br_offset = SDIO_FUNC1_OFFSET + (partition_no-1)*8;
	int i, rc;

	/* printk("%s: partition_no=%d\n", __FUNCTION__, partition_no); */

	/* Set size - write out 4 bytes by 4 requests */
	if (partition_no < AMAP_MAX_REGIONS)
	{
		for(i=0;i<4;i++,br_offset++) {
			data = (part_size>>(8*i))&0xFF;

			/* put R/W Flag (1 for write); Function Number(1), RAW Flag(0),
	   	   	   Register Address - the address of the byte of data inside
	   	   	   of the selected function that will be written
 			   (br_offset for func1),
	   	   	   Write Data - for a direct write command, this is the byte=data,
	   		   that will be written to the selected address=br_offset).
			*/
			sdio_writeb(func, data, br_offset, &rc);
			if (rc < 0) {
				printk(KERN_ERR "%s: Error writing size\n", __FUNCTION__);
				return SDIO_FAILURE;
			}
            /* printk("%s: offset byte=%d data=0x%08x at=0x%08x\n", __FUNCTION__, i, data, br_offset); */
		}

        /* Set offset - write out 4 bytes by 4 requests */
        for(i=0;i<4;i++,br_offset++) {
            data = (start_addr>>(8*i))&0xFF;
            sdio_writeb(func, data, br_offset, &rc);
            if (rc < 0) {
                printk(KERN_ERR "%s: Error writing offset\n", __FUNCTION__);
                return SDIO_FAILURE;
            }
            /* printk("%s: offset byte=%d data=0x%08x at=0x%08x\n", __FUNCTION__, i, data, br_offset); */
        }
    }
#if 0
    if( partition_no == 2 ) {
        unsigned long id1;
        for(i=0,br_offset=0x1ce34;i<4;i++,br_offset++) {
            data = sdio_readb(func, br_offset, &rc);
            if (rc < 0) {
                printk(KERN_ERR "%s: Error reading offset\n", __FUNCTION__);
                return SDIO_FAILURE;
            }
            printk("%s: offset byte=%d data=0x%08x at=0x%08x\n", __FUNCTION__, i, data, br_offset);
        }
        rc = sdio_memcpy_fromio(func, &id1, 0x1ce34, 4); /* Dm: Important - DO NOT REMOVE !!! */
        if (rc < 0) {
            printk(KERN_ERR "%s: Error reading offset\n", __FUNCTION__);
            return SDIO_FAILURE;
        }
        printk("%s: data=0x%08x at=0x%08x\n", __FUNCTION__, id1, br_offset);
    }
#endif
    return SDIO_SUCCESS;
}

/*
  This function configures the slave SDIO device for the required
  operation mode.
*/
SDIO_Status SDIO_TNETWConfig(SDIO_Handle sdioHandle, Peripheral_ConfigParams **peripheral_info)
{
	SDIO_Status rc;

	/* printk("%s\n", __FUNCTION__); */

	TNETW_table.mem_start_addr = TNETW_params.map_reg[0].scr_offset;
	TNETW_table.mem_part_size = TNETW_params.map_reg[0].reg_size;
	TNETW_table.mem_end_addr = TNETW_table.mem_start_addr + TNETW_table.mem_part_size - 1;

	TNETW_table.reg_start_addr = TNETW_params.map_reg[1].scr_offset;
	TNETW_table.reg_part_size = TNETW_params.map_reg[1].reg_size;
	TNETW_table.reg_end_addr = TNETW_table.reg_start_addr + TNETW_table.reg_part_size - 1;
#if 0
	printk("%s: memory area: start_addr=0x%08lx end_addr=0x%08lx part_size=0x%08lx\n", __FUNCTION__, TNETW_table.mem_start_addr, TNETW_table.mem_end_addr, TNETW_table.mem_part_size);
	printk("%s: register area: start_addr=0x%08lx end_addr=0x%08lx part_size=0x%08lx\n", __FUNCTION__, TNETW_table.reg_start_addr, TNETW_table.reg_end_addr, TNETW_table.reg_part_size);
#endif
	/* Configure 17-bits address range in peripheral */
	rc=config_partition(sdioHandle, 1, TNETW_table.mem_start_addr, TNETW_table.mem_part_size);
	rc = (rc==SDIO_SUCCESS)?config_partition(sdioHandle, 2, TNETW_table.reg_start_addr, TNETW_table.reg_part_size):rc;

	if(*peripheral_info)
		*peripheral_info = (void *)&TNETW_table;

    /* printk("%s: TNETW1150 partitions:\n", __FUNCTION__);
	printk("Memory   : start:0x%08lx  end:0x%08lx  size:0x%08lx bytes\n", TNETW_table.mem_start_addr, TNETW_table.mem_end_addr, TNETW_table.mem_part_size);
	printk("Registers: start:0x%08lx  end:0x%08lx  size:0x%08lx bytes\n", TNETW_table.reg_start_addr, TNETW_table.reg_end_addr, TNETW_table.reg_part_size);
    */

	return rc;
}

/*
  This function performs convertion of peripheral adddress into 17 bits
  SDIO address. If found that the memory partition configuration must be
  changed, the memory is re-mapped in accordance to requested address.
*/
SDIO_Address SDIO_ConvertTNETWToSDIOMaster(Peripheral_Address in_tnetw_address, SDIO_BufferLength packet_size)
{
 	SDIO_Address out_sdio_address;

#ifdef CONFIG_SDIO_ADDRESS_MAPPING_BY_APPLICATION
	out_sdio_address = in_tnetw_address;
#else
	Peripheral_Address tnetw_last_address = in_tnetw_address + packet_size - 1;

	/* printk("%s:\n", __FUNCTION__); */

	if ((in_tnetw_address >= TNETW_table.mem_start_addr) && (tnetw_last_address <= TNETW_table.mem_end_addr)) {
		/* printk("%s part1 from=0x%08lx to 0x%08lx\n", __FUNCTION__, in_tnetw_address, tnetw_last_address); */
		/* address in the 1-st partition range (data memory space) */
		out_sdio_address = in_tnetw_address - TNETW_table.mem_start_addr;
	}
	else if ((in_tnetw_address >= TNETW_table.reg_start_addr) && (tnetw_last_address <= TNETW_table.reg_end_addr)) {
		/* printk("%s part2 from=0x%08lx to 0x%08lx\n", __FUNCTION__, in_tnetw_address, tnetw_last_address); */
		/* address in the 2-nd partition range (register memory space) */
		out_sdio_address = in_tnetw_address - TNETW_table.reg_start_addr + TNETW_table.mem_part_size;
	}
	else {
		/* printk("%s peripheral addresses from=0x%08lx to 0x%08lx is out of range\n", __FUNCTION__, in_tnetw_address, tnetw_last_address); */
		/* invalid address */
		return SDIO_INVALID_PERIPHERAL_ADDRESS;
	}
	/* printk("%s: in_tnetw_addr=0x%08lx out_sdio_addr=0x%08lx\n", __FUNCTION__, in_tnetw_address, out_sdio_address); */

#endif /* CONFIG_SDIO_ADDRESS_MAPPING_BY_APPLICATION */

	return out_sdio_address;
}

SDIO_Status SDIO_TNETW_Set_ELP_Reg(SDIO_Handle sdioHandle, Peripheral_Address start_addr, unsigned int data)
{
	struct sdio_func *func = (struct sdio_func *) sdioHandle;
	u8 data1 = 0;
	int br_offset = start_addr;
	int i, rc;

	/* Set size - write out 4 bytes by 4 requests */
	for(i=0;i<1;i++,br_offset++) {
		data1 = (data>>(8*i))&0xFF;

		/* put R/W Flag (1 for write); Function Number(1), RAW Flag(0),
	   	   	   Register Address - the address of the byte of data inside
	   	   	   of the selected function that will be written
 			   (br_offset for func1),
	   	   	   Write Data - for a direct write command, this is the byte=data,
	   		   that will be written to the selected address=br_offset).
		*/
		sdio_writeb(func, data1, br_offset, &rc);
		if (rc < 0) {
			printk(KERN_ERR "%s: Error writing size\n", __FUNCTION__);
			return SDIO_FAILURE;
		}
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_TNETW_Get_ELP_Reg(SDIO_Handle sdioHandle, Peripheral_Address start_addr, unsigned int *data)
{
	struct sdio_func *func = (struct sdio_func *) sdioHandle;
	int br_offset = start_addr;
	int rc;

	*(u8*)data = sdio_readb_ext(func, br_offset, &rc, 0x01);
	if (rc) {
		printk(KERN_ERR "%s: Error reading sdio register (%d)\n", __FUNCTION__, rc);
		return SDIO_FAILURE;
	}
	return SDIO_SUCCESS;
}

#endif /* TIWLAN_MSM7000 */

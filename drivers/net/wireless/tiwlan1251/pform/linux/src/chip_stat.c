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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include "mmc_tnetw1150_api.h"
#include "esta_drv.h"
#include "srcApi.h"
#include "osApi.h"
#include "whalHwRegs.h"

#include "tiwlnif.h"
#include "osUtil.h"

#ifdef TIWLAN_MSM7000
struct sdio_func *SDIO_GetFunc( void );
#endif

/* Module parameters */
static char tiwlan_chip_id[10] = "na";
module_param_string(chip_id, tiwlan_chip_id, sizeof(tiwlan_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(tiwlan_chip_id, "WiFi chip id");
static char fw_version[10] = "na";
module_param_string(fw_version, fw_version, sizeof(fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(fw_version, "WiFi firmware version");

/* export_wifi_fw_version
   Exports WiFi firmware version to /sys/module/wlan/parameters/fw_version
   Returns 0 if OK
*/
int export_wifi_fw_version( tiwlan_net_dev_t *drv )
{
	TIWLN_VERSION swVer;
	unsigned long swLen;
	int ret;

	if (!drv)
		return -EINVAL;

	swLen = sizeof(swVer);
	memset(&swVer, 0, swLen);
	ret = UtilGetSwVersion(&drv->adapter, (unsigned char *)&swVer, &swLen);
	if (ret != 0)
		return -EINVAL;

	fw_version[0] = swVer.FWVersion.major + 48;
	fw_version[1] = '.';
	fw_version[2] = swVer.FWVersion.minor + 48;
	fw_version[3] = '.';
	fw_version[4] = swVer.FWVersion.bugfix + 48;
	fw_version[5] = '.';
	fw_version[6] = swVer.FWVersion.subld + 48;
	fw_version[7] = '.';
	fw_version[8] = swVer.FWVersion.build + 48;
	fw_version[9] = '\0';
	return ret;
}

/* export_wifi_chip_id
   Reads WiFi chip id (0x07030101) and prints it
   Returns 0 if OK
*/
int export_wifi_chip_id( void )
{
	unsigned char chip_id[4];
	unsigned long amap;
#ifdef TIWLAN_MSM7000
	SDIO_Request_t req;
	struct sdio_func *func;

	func = SDIO_GetFunc();
	if (!func)
		return -EINVAL;

	/* configure partition */
	amap = SDIO_DOWNLOAD_PARTITION_START;
	req.buffer = (unsigned char *)&amap;
	req.buffer_len = 4;
	req.peripheral_addr = 0x1ffc4;
	SDIO_SyncWrite(func, &req);

	amap = SDIO_DOWNLOAD_PARTITION_SIZE;
	req.buffer = (unsigned char *)&amap;
	req.buffer_len = 4;
	req.peripheral_addr = 0x1ffc0;
	SDIO_SyncWrite(func, &req);

	amap = SDIO_REG_PARTITION_START;
	req.buffer = (unsigned char *)&amap;
	req.buffer_len = 4;
	req.peripheral_addr = 0x1ffcc;
	SDIO_SyncWrite(func, &req);

	amap = SDIO_REG_PARTITION_SIZE;
	req.buffer = (unsigned char *)&amap;
	req.buffer_len = 4;
	req.peripheral_addr = 0x1ffc8;
	SDIO_SyncWrite(func, &req);

	/* get TIWLAN1251 ID */
	memset(chip_id, 0, 4);
	req.buffer = chip_id;
	req.buffer_len = 4;	
	req.peripheral_addr = SDIO_DOWNLOAD_PARTITION_SIZE + CHIP_ID; /* 0x1BE74 */
	SDIO_SyncRead(func, &req);
#endif
#ifdef TIWLAN_OMAP1610
	/* Can not be read on this stage - SDIO stack is not initialized yet */
	amap = TNETW1251_CHIP_ID_PG1_2;
	memcpy(chip_id, &amap, 4);
#endif

	if ((chip_id[2] >= 1) && (chip_id[2] <= 3)){
		tiwlan_chip_id[0] = '1';
		tiwlan_chip_id[1] = '2';
		tiwlan_chip_id[2] = '5';
		tiwlan_chip_id[3] = '1';
		tiwlan_chip_id[4] = 'P';
		tiwlan_chip_id[5] = 'G';
		tiwlan_chip_id[6] = '1';
		tiwlan_chip_id[7] = '.';
	}
	switch(chip_id[2]){
	case 1:
		tiwlan_chip_id[8] = '0';
		printk("TIWLAN: 1251 PG 1.0\n");
		break;
	case 2:
		tiwlan_chip_id[8] = '1';
		printk("TIWLAN: 1251 PG 1.1\n");
		break;
	case 3:
		tiwlan_chip_id[8] = '2';
		printk("TIWLAN: 1251 PG 1.2\n");
		break;
	default:
		printk("TIWLAN: invalid chip id = 0x%2x%2x%2x%2x!\n",
			chip_id[3], chip_id[2], chip_id[1], chip_id[0]);
		return -EINVAL;
	};
	return 0;
}

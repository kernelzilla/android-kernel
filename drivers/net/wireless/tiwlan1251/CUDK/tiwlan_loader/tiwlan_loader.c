/*
 * tiwlan driver loader - utility to load firmware and calibration data
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 */

/* Copyright © Texas Instruments Incorporated (Oct 2005)
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
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>

#ifdef __LINUX__
#ifdef EEPROM_MEMORY_SUPPORT
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#endif
#endif

#ifdef ANDROID
#include <cutils/properties.h>

#define LOG_TAG "wlan_loader"

#include <cutils/log.h>
#include <hardware_legacy/power.h>
#define PROGRAM_NAME    "wlan_loader"
#endif
	
#include "paramOut.h"
#include "linux_ioctl_common.h"
#include "osApi.h"
#include "tiioctl.h"
#include "TI_AdapterApiC.h"
#include "TI_IPC_Api.h"
#include "osTIType.h"
#include "cli_cu_common.h"
/*---------------------------------------------------------*/
#ifndef offsetof
#define offsetof(type, field)    ((unsigned int) (&(((type *)(0))->field)))
#endif

#ifndef IFNAMSIZ
#define	IFNAMSIZ	16
#endif

#ifdef ANDROID

#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO

#ifdef ENABLE_LOG_ERROR
#define print_error(fmt, args...) \
    { LOGE(fmt , ## args); }
#else
#define print_error(fmt, args...) \
    do { } while (0)
#endif /* ENABLE_LOG_ERROR */

#ifdef ENABLE_LOG_DEBUG
#define print_debug(fmt, args...) \
    { LOGD(fmt , ## args); }
#else
#define print_debug(fmt, args...) \
    do { } while (0)
#endif /* ENABLE_LOG_DEBUG */

#ifdef ENABLE_LOG_INFO
#define print_info(fmt, args...) \
    { LOGI(fmt , ## args); }
#else
#define print_info(fmt, args...) \
    do { } while (0)
#endif /* ENABLE_LOG_INFO */

#else /* !ANDROID */

#define print_error printf
#define print_debug printf

#endif /* ifdef ANDROID */


/*---------------------------------------------------------*/
char g_drv_name[IFNAMSIZ + 1];
TI_HANDLE g_id_adapter = 0;
/*---------------------------------------------------------*/
int print_usage(void)
{
    print_info("Usage: ./wlan_loader [driver_name] [options]\n");
#ifdef HOST_PLATFORM_WIPP
    print_info("   -e <filename>  - eeprom image file name. default=/var/run/nvs_map.bin\n");
#else
    print_info("   -e <filename>  - eeprom image file name. default=./nvs_map.bin\n");
#endif
    print_info("   -i <filename>  - init file name. default=tiwlan.ini\n");
    print_info("   -f <filename>  - firmware image file name. default=firmware.bin\n");
    return 1;
}

/*  Return '0' if success */
int init_driver( char *adapter_name, char *eeprom_file_name, char *init_file_name, char *firmware_file_name )
{
#ifdef __LINUX__
    FILE *f1 = NULL, *f2 = NULL, *f3 = NULL;
    UINT32 eeprom_image_length = 0;
    UINT32 init_file_length = 0;
    UINT32 firmware_image_length = 0;
    UINT32 req_size = 0;
    tiwlan_dev_init_t *init_info = NULL;
#ifdef EEPROM_MEMORY_SUPPORT
    volatile unsigned long *nvsPtr = NULL;
    int fd = -1;
#endif    
#endif
    int rc = -1, count = 0;
    tiUINT32 tmpData = 1;

    print_debug("adapter %s, eeprom %s, init %s, firmware %s\n",
               adapter_name, eeprom_file_name, init_file_name, firmware_file_name);

    if( !adapter_name || !*adapter_name )
        return rc;

    g_id_adapter = TI_AdapterInit( adapter_name );

    if( g_id_adapter == 0 ) {
      print_error("wlan_loader: failed to initialize Utility-Adapter interface...aborting...\n");
      goto init_driver_end;
    }

#ifdef __LINUX__
    /* Send init request to the driver */
    if (eeprom_file_name) {
        if ((f1 = fopen(eeprom_file_name, "r")) == NULL) {
            print_error("Cannot open eeprom image file <%s>: %s\n",
                    eeprom_file_name, strerror(errno));
            goto init_driver_end;
        }
        if (fseek(f1, 0, SEEK_END)) {
            print_error("Cannot seek eeprom image file <%s>: %s\n",
                    eeprom_file_name, strerror(errno));
            goto init_driver_end;
        }
        eeprom_image_length = ftell(f1);
        rewind(f1);
    }
#ifdef EEPROM_MEMORY_SUPPORT
    else {
	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if( fd == -1 ) {
            print_error("Cannot access /dev/mem\n");
            goto init_driver_end;
	}
	nvsPtr = (volatile unsigned long *)mmap(0,0x1000,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0x13F13000);
	eeprom_image_length = *(nvsPtr + (0xE0C >> 2));
	print_debug("---Eeprom from Memory Size = %u\n", eeprom_image_length);
    }
#endif
#ifdef FIRMWARE_DYNAMIC_LOAD
    if( firmware_file_name) {
        if ((f2 = fopen(firmware_file_name, "r")) == NULL ) {
            print_error("Cannot open firmware file <%s>: %s\n",
                    firmware_file_name, strerror(errno));
            goto init_driver_end;
        }
        if( fseek(f2, 0, SEEK_END) ) {
            print_error( "Cannot seek firmware file <%s>: %s\n",
                    firmware_file_name, strerror(errno));
            goto init_driver_end;
        }
        firmware_image_length = ftell(f2);
        rewind(f2);
    }
#endif
    if( init_file_name) {
        if ((f3 = fopen(init_file_name, "r")) == NULL ) {
            print_error( "Cannot open init file <%s>: %s\n",
                    init_file_name, strerror(errno));
            goto init_driver_end;
        }
        if( fseek(f3, 0, SEEK_END) ) {
            print_error("Cannot seek init file <%s>: %s\n",
                    init_file_name, strerror(errno));
            goto init_driver_end;
        }
        init_file_length = ftell(f3);
        rewind(f3);
    }

    /* Now when we can calculate the request length. allocate it and read the files */
    req_size = offsetof(tiwlan_dev_init_t, data)+ eeprom_image_length + (init_file_length+1) + firmware_image_length;
    init_info = (tiwlan_dev_init_t *)malloc(req_size);
    if( !init_info ) {
        print_error("No memory to allocate init request (%d bytes)\n", req_size);
        goto init_driver_end;
    }
    init_info->eeprom_image_length   = eeprom_image_length;
    init_info->firmware_image_length = firmware_image_length;
    init_info->init_file_length      = init_file_length;
#ifdef EEPROM_MEMORY_SUPPORT
    if( (nvsPtr != NULL) && eeprom_image_length ) {
        memcpy(&init_info->data[0], (void *)(nvsPtr + (0xE40 >> 2)), eeprom_image_length);
        munmap( (void *)nvsPtr, 0x1000 );
        close( fd );
    }
    else
#endif    
    if( eeprom_image_length &&
        fread(&init_info->data[0], 1, eeprom_image_length, f1) < eeprom_image_length ) {
        print_error("Error reading eeprom image %s: %s\n", eeprom_file_name, strerror(errno));
        goto init_driver_end;
    }
    if( firmware_image_length &&
        fread(&init_info->data[eeprom_image_length], 1, firmware_image_length, f2) < firmware_image_length ) {
        print_error("Error reading firmware image %s: %s\n", firmware_file_name, strerror(errno));
        goto init_driver_end;
    }
    if( init_file_length &&
        fread(&init_info->data[eeprom_image_length+firmware_image_length], 1, init_file_length, f3) < init_file_length ) {
        print_error("Error reading init_file %s: %s\n", init_file_name, strerror(errno));
        goto init_driver_end;
    }

    do { /* Need to wait till driver will be created in sdio_probe() */
	print_debug("Configuring adapter\n");
        rc = IPC_DeviceIoControl(adapter_name, TIWLN_SET_INIT_INFO, init_info, req_size, NULL, 0, NULL);
	print_debug("Adapter configuration rc = %d\n", rc);
        if( rc != 0 ) {
            if( count > 4 ) {
                break;
            }
            count++;
            sleep(1);
        }
    } while( rc != 0 );

    /* Send configMge start command as the cli is started */
    if( rc == 0 ) {
	print_debug("Starting configMge\n");
        rc = IPC_DeviceIoControl(adapter_name, TIWLN_DRIVER_STATUS_SET, &tmpData, sizeof(tiUINT32), NULL, 0, NULL);
	print_debug("ConfigMge start rc = %d\n", rc);
    }

init_driver_end:
    if( f1 )
        fclose(f1);
    if( f2 )
        fclose(f2);
    if( f3 )
        fclose(f3);
    if( init_info )
        free(init_info);
#endif
    if( rc == 0 ) {
	print_debug("Driver configured\n");
    } else {
	print_debug("Driver configuration failed (%d)\n", rc);
    }
    return rc;
}

#ifdef ANDROID
int check_and_set_property(char *prop_name, char *prop_val)
{
    char prop_status[PROPERTY_VALUE_MAX];
    int count;

    for(count=4;( count != 0 );count--) {
        property_set(prop_name, prop_val);
        if( property_get(prop_name, prop_status, NULL) &&
            (strcmp(prop_status, prop_val) == 0) )
	    break;
    }
    return( count );
}
#endif

#ifdef __LINUX__
int main(int argc, char ** argv)
{
    int i;
#ifdef HOST_PLATFORM_WIPP
    char *eeprom_file_name = "/NVS/nvs_map.bin";
    char *init_file_name = "/voice/tiwlan.ini";
    char *firmware_file_name = "/apps/firmware.bin";
#else
    char *eeprom_file_name = "./nvs_map.bin";
    char *init_file_name = "./tiwlan.ini";
    char *firmware_file_name = "./firmware.bin";
#endif

    if( argc > 1 ) {
        i=1;
        if( argv[i][0] != '-' ) {
            strcpy( g_drv_name, argv[i++] );
        }
        for(;( i < argc );i++) {
            if( !strcmp(argv[i], "-h" ) || !strcmp(argv[i], "--help") )
                return print_usage();
            else if( !strcmp(argv[i], "-f" ) ) {
                firmware_file_name = argv[++i];
            }
            else if( !strcmp(argv[i], "-e") && ((i+1) < argc) ) {
                eeprom_file_name = argv[++i];
            }
            else if( !strcmp(argv[i], "-i") && ((i+1) < argc) ) {
                init_file_name = argv[++i];
            }
            else if( !strcmp(argv[i], "-n" ) ) {
               eeprom_file_name = NULL;
            }
            else {
                print_error("ticon: unknown parameter '%s'\n", argv[i] );
#ifdef ANDROID		
                check_and_set_property("wlan.driver.status", "failed");
#endif		
                return -1;
            }
        }
    }
#else
int start_cli()
{
    char *eeprom_file_name = NULL;
    char *init_file_name = NULL;
    char *firmware_file_name = NULL;
#endif
    if( !g_drv_name[0] ) {
#if defined(__LINUX__)
        strcpy(g_drv_name, TIWLAN_DRV_NAME "0" );
#else
        strcpy(g_drv_name, TIWLAN_DRV_NAME );
#endif
    }

#ifdef ANDROID
    acquire_wake_lock(PARTIAL_WAKE_LOCK, PROGRAM_NAME);
#endif
	
    if( init_driver(g_drv_name, eeprom_file_name, init_file_name, firmware_file_name) != 0 ) {
	print_error("init_driver() failed\n");
#ifdef ANDROID    
        check_and_set_property("wlan.driver.status", "failed");
        release_wake_lock(PROGRAM_NAME);
#endif    
        return -1;
    }

    TI_AdapterDeinit(g_id_adapter);
#ifdef ANDROID    
    check_and_set_property("wlan.driver.status", "ok");
    release_wake_lock(PROGRAM_NAME);
#endif    
    return 0;
}

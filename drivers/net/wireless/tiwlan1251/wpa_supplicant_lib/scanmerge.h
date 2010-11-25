/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*-------------------------------------------------------------------*/
#ifndef _SCANMERGE_H_
#define _SCANMERGE_H_

#include "common.h"
#include "driver.h"
#include "driver_ti.h"

#define SCAN_MERGE_COUNT        4

typedef struct SCANMERGE_STRUCT {
    struct wpa_scan_result scanres;
    unsigned long count;
} scan_merge_t;

void scan_init( struct wpa_driver_ti_data *mydrv );
void scan_exit( struct wpa_driver_ti_data *mydrv );
unsigned int scan_merge( struct wpa_driver_ti_data *mydrv,
                         struct wpa_scan_result *results,
                         unsigned int number_items, unsigned int max_size );

#endif

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
#include "includes.h"
#include "scanmerge.h"
#include "shlist.h"

/*-----------------------------------------------------------------------------
Routine Name: scan_init
Routine Description: Inits scan merge list
Arguments:
   mydrv   - pointer to private driver data structure
Return Value:
-----------------------------------------------------------------------------*/
void scan_init( struct wpa_driver_ti_data *mydrv )
{
    mydrv->last_scan = -1;
    shListInitList( &(mydrv->scan_merge_list) );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_free
Routine Description: Frees scan structure private data
Arguments:
   ptr - pointer to private data structure
Return Value:
-----------------------------------------------------------------------------*/
static void scan_free( void *ptr )
{
    os_free( ptr );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_exit
Routine Description: Cleans scan merge list
Arguments:
   mydrv   - pointer to private driver data structure
Return Value:
-----------------------------------------------------------------------------*/
void scan_exit( struct wpa_driver_ti_data *mydrv )
{
    shListDelAllItems( &(mydrv->scan_merge_list), scan_free );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_equal
Routine Description: Compares bssid of scan result and scan merge structure
Arguments:
   val   - pointer to scan result structure
   idata - pointer to scan merge structure
Return Value: 1 - if equal, 0 - if not
-----------------------------------------------------------------------------*/
static int scan_equal( void *val,  void *idata )
{
    struct wpa_scan_result *new_res = (struct wpa_scan_result *)val;
    struct wpa_scan_result *lst_res =
               (struct wpa_scan_result *)(&(((scan_merge_t *)idata)->scanres));

    return( !os_memcmp(new_res->bssid, lst_res->bssid, ETH_ALEN) );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_add
Routine Description: adds scan result structure to scan merge list
Arguments:
   head    - pointer to scan merge list head
   res_ptr - pointer to scan result structure
Return Value: Pointer to scan merge item
-----------------------------------------------------------------------------*/
static scan_merge_t *scan_add( SHLIST *head, struct wpa_scan_result *res_ptr )
{
    scan_merge_t *scan_ptr;

    scan_ptr = (scan_merge_t *)os_malloc( sizeof(scan_merge_t) );
    if( !scan_ptr )
        return( NULL );
    os_memcpy( &(scan_ptr->scanres), res_ptr, sizeof(struct wpa_scan_result) );
    scan_ptr->count = SCAN_MERGE_COUNT;
    shListInsLastItem( head, (void *)scan_ptr );
    return scan_ptr;
}

/*-----------------------------------------------------------------------------
Routine Name: scan_find
Routine Description: Looks for scan merge item in scan results array
Arguments:
   scan_ptr - pointer to scan merge item
   results - pointer to scan results array
   number_items - current number of items
Return Value: 1 - if item was found, 0 - otherwise
-----------------------------------------------------------------------------*/
static int scan_find( scan_merge_t *scan_ptr, struct wpa_scan_result *results,
                      unsigned int number_items )
{
    unsigned int i;

    for(i=0;( i < number_items );i++) {
        if( scan_equal( &(results[i]), scan_ptr ) )
            return 1;
    }
    return 0;
}

/*-----------------------------------------------------------------------------
Routine Name: scan_merge
Routine Description: Merges current scan results with previous
Arguments:
   mydrv   - pointer to private driver data structure
   results - pointer to scan results array
   number_items - current number of items
   max_size - maximum namber of items
Return Value: Merged number of items
-----------------------------------------------------------------------------*/
unsigned int scan_merge( struct wpa_driver_ti_data *mydrv,
                         struct wpa_scan_result *results,
                         unsigned int number_items, unsigned int max_size )
{
    SHLIST *head = &(mydrv->scan_merge_list);
    SHLIST *item, *del_item;
    scan_merge_t *scan_ptr;
    unsigned int i;

    if( mydrv->last_scan == SCAN_TYPE_NORMAL_PASSIVE ) { /* Merge results */
        for(i=0;( i < number_items );i++) { /* Find/Add new items */
            item = shListFindItem( head, &(results[i]), scan_equal );
            if( item ) {
                scan_ptr = (scan_merge_t *)(item->data);
                os_memcpy( &(scan_ptr->scanres), &(results[i]),
                           sizeof(struct wpa_scan_result) );
                scan_ptr->count = SCAN_MERGE_COUNT;
            }
            else {
                scan_add( head, &(results[i]) );
            }
        }
        item = shListGetFirstItem( head );  /* Add/Remove missing items */
        if( item == NULL )
            return( number_items );
        do {
            del_item = NULL;
            scan_ptr = (scan_merge_t *)(item->data);
            if( !scan_find( scan_ptr, results, number_items ) ) {
                scan_ptr->count--;
                if( scan_ptr->count == 0 ) {
                    del_item = item;
                }
                else {
                    if( number_items < max_size ) {
                        os_memcpy(&(results[number_items]),
                          &(scan_ptr->scanres),sizeof(struct wpa_scan_result));
                        number_items++;
                    }
                }
            }
            item = shListGetNextItem( head, item );
            shListDelItem( head, del_item, scan_free );
        } while( item != NULL );
    }
    else if( mydrv->last_scan == SCAN_TYPE_NORMAL_ACTIVE ) { /* Copy results */
        shListDelAllItems( head, scan_free );
        for(i=0;( i < number_items );i++) {
            if( scan_add( head, &(results[i]) ) == NULL )
                return( i );
        }
    }
    return( number_items );
}

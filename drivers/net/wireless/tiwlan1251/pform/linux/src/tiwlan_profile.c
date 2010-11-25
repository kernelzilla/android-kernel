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



#include "esta_drv.h"


static void tiwlan_profile_bus_access_start (void *os, unsigned);
static void tiwlan_profile_bus_access_end (void *os, unsigned);
static void tiwlan_profile_driver_entry_start (void *os, unsigned);
static void tiwlan_profile_driver_entry_end (void *os, unsigned);
static void tiwlan_profile_memory_alloc (void *os, unsigned);
static void tiwlan_profile_memory_free (void *os, unsigned);
static void tiwlan_profile_buf_alloc (void * os, unsigned);
static void tiwlan_profile_timer_create (void * os, unsigned);
static void tiwlan_profile_timer_destroy (void * os, unsigned);


int tiwlan_profile_create (tiwlan_net_dev_t *drv)
{
    drv->fpro [0] = NULL;
    drv->fpro [1] = NULL;
    drv->fpro [2] = NULL;
    drv->fpro [3] = NULL;
    drv->fpro [4] = tiwlan_profile_memory_alloc;
    drv->fpro [5] = tiwlan_profile_memory_free;
    drv->fpro [6] = tiwlan_profile_timer_create;
    drv->fpro [7] = tiwlan_profile_timer_destroy;
    drv->fpro [8] = tiwlan_profile_buf_alloc;

    drv->cpu_usage_estimator_start_time = 0;
    drv->cpu_usage_estimator_stop_time = 0;
    drv->max_heap_bytes_allocated = 0;
    drv->max_buf_bytes_allocated = 0;
    drv->cur_heap_bytes_allocated = 0;
    drv->max_number_of_timers = 0; 
    drv->cur_number_of_timers = 0;

    return 0;
}


/* Call register profiler banchmark */
int tiwlan_profile (tiwlan_net_dev_t *drv, unsigned bm, unsigned par)
{
    if (drv && bm < MAX_PROFILE_BM && drv->fpro [bm])
    {
        (*drv->fpro [bm]) (drv, par);
    }

    return 0;
}


/* Stop CPU estimation for a driver entry and maintains the resolution of the estimator */
void tiwlan_profile_bus_access_start (void * os, unsigned par)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;

    if (drv != NULL)
    {
        /* Save the current entry's start time */
        drv->bus_driver_entry_start_time = os_timeStampUs (drv);
    }
}


/* Starts CPU estimation for a bus driver entry */
void tiwlan_profile_bus_access_end (void * os, unsigned par)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;
    unsigned current_entry_cpu_usage;

    if (drv != NULL)
    {
        /* Save the current entry's start time */
        current_entry_cpu_usage = os_timeStampUs (drv) - drv->bus_driver_entry_start_time;

        /* Make sure that it is not a negative value */
        if ((int)current_entry_cpu_usage < 0) 
        {
            printk("\n\n%s: cpu usage estimation corrupted. entry_start=%u, entry_cpu_time = %d\n\n\n",
                   __FUNCTION__, drv->bus_driver_entry_start_time,  current_entry_cpu_usage);
        }
        /* Update the total time of driver CPU usage */
        else 
        {
            drv->total_us_of_bus_access_cpu_time += current_entry_cpu_usage;
        }
    }
}


/* Starts CPU estimation for a driver entry */
void tiwlan_profile_driver_entry_start (void * os, unsigned par)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;

    if (drv != NULL)
    {
        drv->driver_entry_start_time = os_timeStampUs (drv);
    }
}


/* Stop CPU estimation for a driver entry and maintains the resolution of the estimator */
void tiwlan_profile_driver_entry_end (void * os, unsigned par)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;
    unsigned current_entry_cpu_usage, driver_entry_end_time;

    if (drv == NULL)
        return;

    /* Get the current entry's end time */
    driver_entry_end_time = os_timeStampUs (drv);

    /* Calculate the current entries CPU run time */
    current_entry_cpu_usage = driver_entry_end_time - drv->driver_entry_start_time;

    /* Make sure that it is not a negative value */
    if ((int)current_entry_cpu_usage < 0) 
    {
        printk("\n\n%s: cpu usage estimation corrupted. entry_start=%u, entry_end=%u, entry_cpu_time = %d\n\n\n",
               __FUNCTION__, drv->driver_entry_start_time, driver_entry_end_time, current_entry_cpu_usage);
    }
    /* Update the total time of driver CPU usage */
    else 
    {
        drv->total_us_of_cpu_time += current_entry_cpu_usage;
    }
}


void tiwlan_profile_memory_alloc (void * os, unsigned size)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;

    if (drv != NULL) 
    {
        /* Increase current heap allocation counter */
        drv->cur_heap_bytes_allocated += size;
        /* Update maximum if execceded */
        if (drv->max_heap_bytes_allocated < drv->cur_heap_bytes_allocated) 
        {
            drv->max_heap_bytes_allocated = drv->cur_heap_bytes_allocated;
        }
    }
}


void tiwlan_profile_memory_free (void * os, unsigned size)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;

    if (drv != NULL) 
    {
        /* Decrease amount from heap allocation counter */
        drv->cur_heap_bytes_allocated -= size;
        /* Check for overflow */
        if ((int)drv->cur_heap_bytes_allocated < 0) 
        {
            printk("\n\n%s: memory heap allocation calculation corrupted. Size=%u, Current allocation = %d\n\n\n",
                   __FUNCTION__, size, drv->cur_heap_bytes_allocated);
            drv->cur_heap_bytes_allocated = 0;
        }
    }
}


void tiwlan_profile_buf_alloc (void * os, unsigned size)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;

    if (drv != NULL) 
    {
        drv->max_buf_bytes_allocated += size;
    }
}


void tiwlan_profile_timer_create (void * os, unsigned par)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;

    if (drv)
    {
        /* Increase the current active timer counter */
        drv->cur_number_of_timers ++;
        /* Update maximum if execceded */
        if (drv->max_number_of_timers < drv->cur_number_of_timers) 
        {
            drv->max_number_of_timers = drv->cur_number_of_timers;
        }
    }
}


void tiwlan_profile_timer_destroy (void * os, unsigned par)
{
    tiwlan_net_dev_t * drv = (tiwlan_net_dev_t *) os;

    if (drv)
    {
        /* Decrease the current active timer counter */
        drv->cur_number_of_timers --;
    }
}


/* 
 * Start CPU estimator 
 * NOTE: this function does not run in a driver context 
 */
int tiwlan_profile_cpu_usage_estimator_start (tiwlan_net_dev_t * drv, unsigned int resolution)
{
    /* 
     *  Reset estimation parameters - no need for spin lock since 
     *  estimator is not running 
     */
    drv->total_us_of_cpu_time = 0;
    drv->total_us_of_bus_access_cpu_time = 0;
    drv->cpu_usage_estimator_start_time = os_timeStampUs (drv);
    drv->cpu_usage_estimator_stop_time = 0;

    /* Set the new resolution */
    drv->cpu_usage_estimator_resolution = resolution;

    /* Register profiler banchmarks */
    drv->fpro [0] = tiwlan_profile_driver_entry_start;
    drv->fpro [1] = tiwlan_profile_driver_entry_end;
    drv->fpro [2] = tiwlan_profile_bus_access_start;
    drv->fpro [3] = tiwlan_profile_bus_access_end;

    return 0;
}


/* 
 * Stop CPU estimator and save the last CPU estimation 
 * NOTE: this function does not run in a driver context 
 */
int tiwlan_profile_cpu_usage_estimator_stop (tiwlan_net_dev_t * drv)
{
    drv->cpu_usage_estimator_stop_time = os_timeStampUs (drv);

    /* Unregister profiler banchmarks */
    drv->fpro [0] = NULL;
    drv->fpro [1] = NULL;
    drv->fpro [2] = NULL;
    drv->fpro [3] = NULL;

    return 0;
}


/* 
 * Reset CPU estimation 
 * NOTE: this function is not run in a driver context 
 */
int tiwlan_profile_cpu_usage_estimator_reset (tiwlan_net_dev_t * drv)
{
    /* Reset accumulated driver time and the last estimation */
    drv->total_us_of_cpu_time = 0;
    drv->total_us_of_bus_access_cpu_time = 0;
    drv->cpu_usage_estimator_start_time = 0;
    drv->cpu_usage_estimator_stop_time = 0;

    return 0;
}


/* Print to the screen the latest resource usage and CPU estimation */
int tiwlan_profile_report (tiwlan_net_dev_t *drv)
{
    unsigned total_time, drv_cpu_usage = 0, bus_cpu_usage = 0;

    printk ("\nDriver Resource Usage");
    printk ("\n=====================");
    printk ("\nMaximum Heap Memory Allocated: %u (bytes)", drv->max_heap_bytes_allocated);
    printk ("\nCurrent Heap Memory Allocated: %u (bytes)", drv->cur_heap_bytes_allocated);
    printk ("\nBuffer Memory Allocated: %u (bytes)", drv->max_buf_bytes_allocated);
    printk ("\nFirmware Image Memory Allocated: %u (bytes)", (unsigned)drv->firmware_image.size);
    printk ("\nEEPROM Image Memory Allocated: %u (bytes)", (unsigned)drv->eeprom_image.size);
    printk ("\nMaximum Active Timers: %u", drv->max_number_of_timers);
    printk ("\nCurrent Active Timers: %u", drv->cur_number_of_timers);

    /* Check that the estimation has been started and stopped stopped */
    if (drv->cpu_usage_estimator_stop_time != 0)
    {
        total_time = drv->cpu_usage_estimator_stop_time - drv->cpu_usage_estimator_start_time;

        total_time /= 100;

        if ((int)total_time > 0)
        {
            drv_cpu_usage = drv->total_us_of_cpu_time / total_time;
            bus_cpu_usage = drv->total_us_of_bus_access_cpu_time / total_time;

            printk ("\nTotal Test Run Time: %u (usec)", total_time);
            printk ("\nTotal Driver Run Time: %u (usec)", drv->total_us_of_cpu_time);
            printk ("\nTotal Bus Access Time: %u (usec)", drv->total_us_of_bus_access_cpu_time);
            printk ("\nTotal CPU Usage: %u%%", drv_cpu_usage);
            printk ("\nBus Access CPU Usage: %u%%", bus_cpu_usage);
            printk ("\n");
        }
    }

    return 0;
}


/* drivers/video/msm/msm_fb_esd.c
 *
 * MSM video driver - ESD module.
 *
 * Copyright (C) 2009 Motorola
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

#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include "msm_fb.h"

volatile int msm_fb_sent_frame = 0;

typedef enum
{
    ESD_STATE_NULL,
    ESD_STATE_RUNNING,
    ESD_STATE_STOPPING,
    ESD_STATE_UPDATE
}   ESD_STATE_E;

static volatile ESD_STATE_E esd_state = ESD_STATE_NULL, esd_mode = ESD_STATE_NULL;
static volatile long esd_sleep_time;
static wait_queue_head_t esd_wq;
static struct task_struct *esd_task;

#define WAIT_SECS 8

static int do_esd(void *_par)
{
    struct platform_device *pdev = _par;
    struct msm_fb_panel_data *info = pdev->dev.platform_data;

    esd_state = ESD_STATE_RUNNING;
    set_user_nice(current, -10);

    do
    {
        msm_fb_sent_frame = 0;
        wait_event_interruptible_timeout(esd_wq, esd_state != esd_mode, esd_sleep_time);

        switch(esd_state)
        {
            case ESD_STATE_UPDATE:
                esd_state = esd_mode;
            break;

            case ESD_STATE_RUNNING:
                if(MAX_SCHEDULE_TIMEOUT != esd_sleep_time)
                {
                    if(HZ*WAIT_SECS == esd_sleep_time)
                    {
                        info->check_esd(pdev, WAIT_SECS, msm_fb_sent_frame);
                    }
                    else
                    {
                        esd_sleep_time = HZ*WAIT_SECS;
                    }
                }
            break;

            default:
                printk("esdd_nt ignoring state: %d\n", esd_state);
            break;
        } // switch
    } while (!kthread_should_stop());
    esd_state = ESD_STATE_NULL;

    printk(KERN_INFO "esd thread killed\n");

    return 0;
}

int msm_fb_enable_esd_mgr(struct platform_device *pdev)
{
    if(pdev)
    {
        struct msm_fb_panel_data *info = pdev->dev.platform_data;

        if(info->check_esd != NULL)
        {
            printk(KERN_INFO "enabling esd detect\n");

            if(esd_state == ESD_STATE_NULL)
            {
                esd_mode = ESD_STATE_RUNNING;
                esd_sleep_time = HZ*90;

                init_waitqueue_head(&esd_wq);

                esd_task = kthread_run(do_esd, pdev, "esdd_nt");
            }
            else
            {
                esd_state = ESD_STATE_UPDATE;
                esd_mode = ESD_STATE_RUNNING;
                esd_sleep_time = HZ*20;
                wake_up(&esd_wq);
            }
        }
    }

    return 0;
}

int msm_fb_disable_esd_mgr(struct platform_device *pdev)
{
    if(esd_state != ESD_STATE_NULL)
    {
        printk(KERN_INFO "disabling esd detect\n");

        esd_state = ESD_STATE_UPDATE;
        esd_mode = ESD_STATE_STOPPING;

        esd_sleep_time = MAX_SCHEDULE_TIMEOUT;
        wake_up(&esd_wq);
    }

    return 0;
}

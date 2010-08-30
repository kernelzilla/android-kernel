/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MSM_FB_PANEL_H
#define MSM_FB_PANEL_H

#include "msm_fb_def.h"

struct msm_fb_data_type;

typedef void (*msm_fb_vsync_handler_type) (void *arg);

/////////////////////////////////////////////
// panel id type
/////////////////////////////////////////////
typedef struct panel_id_s {
    uint16 id;
    uint16 type;
} panel_id_type;

/////////////////////////////////////////////
// panel type list
/////////////////////////////////////////////
#define NO_PANEL       0xffff    /* No Panel */
#define MDDI_PANEL     1    /* MDDI */
#define EBI2_PANEL     2    /* EBI2 */
#define LCDC_PANEL     3    /* internal LCDC type */
#define EXT_MDDI_PANEL 4    /* Ext.MDDI */
#define TV_PANEL       5    /* TV */

/////////////////////////////////////////////
// panel class
/////////////////////////////////////////////
typedef enum {
    DISPLAY_LCD = 0,    // lcd = ebi2/mddi
    DISPLAY_LCDC,        // lcdc
    DISPLAY_TV,        // TV Out
    DISPLAY_EXT_MDDI,    // External MDDI
} DISP_TARGET;

/////////////////////////////////////////////
// panel device locaiton
/////////////////////////////////////////////
typedef enum {
    DISPLAY_1 = 0,        // attached as first device
    DISPLAY_2,        // attached on second device
    MAX_PHYS_TARGET_NUM,
} DISP_TARGET_PHYS;

/////////////////////////////////////////////
// panel info type
/////////////////////////////////////////////
struct lcd_panel_info {
    __u32 vsync_enable;
    __u32 refx100;
    __u32 v_back_porch;
    __u32 v_front_porch;
    __u32 v_pulse_width;
    __u32 hw_vsync_mode;
    __u32 vsync_notifier_period;
};

struct lcdc_panel_info {
    __u32 h_back_porch;
    __u32 h_front_porch;
    __u32 h_pulse_width;
    __u32 v_back_porch;
    __u32 v_front_porch;
    __u32 v_pulse_width;
    __u32 border_clr;
    __u32 underflow_clr;
    __u32 hsync_skew;
};

struct mddi_panel_info {
    __u32 vdopkt;
};

struct msm_panel_info {
    __u32 xres;
    __u32 yres;
    __u32 bpp;
    __u32 type;
    __u32 wait_cycle;
    DISP_TARGET_PHYS pdest;
    __u32 bl_max;
    __u32 bl_min;
    __u32 fb_num;
    __u32 clk_rate;
    __u32 clk_min;
    __u32 clk_max;

    union {
        struct mddi_panel_info mddi;
    };

    union {
        struct lcd_panel_info lcd;
        struct lcdc_panel_info lcdc;
    };
};

struct msm_fb_panel_data {
    struct msm_panel_info panel_info;
    void (*set_rect) (int x, int y, int xres, int yres);
    void (*set_vsync_notifier) (msm_fb_vsync_handler_type, void *arg);
    void (*set_backlight) (struct msm_fb_data_type *);

    /* function entry chain */
    int (*on) (struct platform_device *pdev);
    int (*off) (struct platform_device *pdev);
#ifdef CONFIG_MACH_MOT
    int (*check_esd) (struct platform_device *pdev, int num_secs_delay, int num_frames_sent);
    int (*wait_for_vsync) (struct platform_device *pdev);
    int (*pan_updt_supported) (struct platform_device *pdev);
    void (*enable_panel) (struct platform_device *pdev);
#endif
    struct platform_device *next;
};

/*===========================================================================
  FUNCTIONS PROTOTYPES
============================================================================*/
struct platform_device *msm_fb_device_alloc(struct msm_fb_panel_data *pdata,
                        u32 type, u32 id);
int panel_next_on(struct platform_device *pdev);
int panel_next_off(struct platform_device *pdev);

int lcdc_device_register(struct msm_panel_info *pinfo);

#ifndef CONFIG_MACH_MOT
int mddi_toshiba_device_register(struct msm_panel_info *pinfo,
                    u32 channel, u32 panel);
#endif

#endif /* MSM_FB_PANEL_H */

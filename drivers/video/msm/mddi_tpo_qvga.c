/* drivers/video/msm/mddi_tpo_qvga.c
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

#include <linux/device.h> /* for vreg.h */
#include <linux/delay.h>
#include <mach/vreg.h>
#include <mach/board.h>

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/leds.h>
#include <linux/gpio.h>
#include "mddi_tpo_qvga.h"
//#include <mot/esd_poll.h>
#include <mach/board.h>
//#include "board-mot.h" //GPIO/I2C addresses


/*=============================================================================
                        LOCAL CONSTANTS
=============================================================================*/
#define TPO_QVGA_PRIM      1
#define MDDI_SLEEP_CMD        0xFFFFFFFF
#define TABLE_END             0xEFFFFFFF

#define LCD_WIDTH             240
#define LCD_HEIGHT            320
#define V_SYNC_WIDTH          0
#define V_BACK_PORCH          0
#define V_FRONT_PORCH         0

#define TPO_VER_F_USERID   0x05
#define TPO_VER_G_USERID   0x06

#define TPO_PLAT_DISP         0xB9F65395
#define TMD_PLAT_DISP         0xB9F65451

/*=============================================================================
                        LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
=============================================================================*/
struct init_table {
    unsigned int reg;
    unsigned int val;
};

/*=============================================================================
                        LOCAL FUNCTION PROTOTYPES
=============================================================================*/

/*=============================================================================
                                 LOCAL MACROS
=============================================================================*/

/*=============================================================================
                                LOCAL VARIABLES
=============================================================================*/
static uint32 mddi_tpo_rows_per_second = LCD_HEIGHT * 100; // 90Hz
static uint32 mddi_tpo_rows_per_refresh = LCD_HEIGHT;
static boolean tpo_powered_on = FALSE;
static uint32 plat_disp_type = 0;

// static struct init_table const tpo_plat_esd_recovery[] =
// {
//    { 0x2D00,      0x0000 },
//    { 0x2D01,      0x0000 },
//    { 0x2D02,      0x0000 },
//    { 0x2D03,      0x0000 },
//    { TABLE_END,   0x0000 }, /* terminate table */
// };

// static struct init_table const tpo_a9_to_a11_init[] =
// {
//     { ENTRYMODE1,       0x0030 }, /* ENTRYMODE1: Entry Mode 1 (360h) */
//     { TABLE_END,        0x0000 }, /* terminate table */
// };

static struct init_table const tpo_hx8352b_init[] =
{
    { MDDI_SLEEP_CMD, 10 }, /* wait 10ms */
// EQ function
    { 0xE2, 0x15 },
    { 0xE5, 0x18 },
    { 0xE7, 0x18 },
    { 0xE8, 0x48 },
    { 0xEC, 0x09 },
    { 0xED, 0x06 },
    { 0xEF, 0x48 },

// Power Setting
    { 0x23, 0x76 },
    { 0x24, 0x57 },
    { 0x25, 0x71 },
    { 0x1B, 0x1E },

// Power on Setting
    { 0x01, 0x00 },
    { 0x1C, 0x04 },
    { 0x19, 0x01 },
    { MDDI_SLEEP_CMD, 5 },
    { 0x1F, 0x90 },
    { MDDI_SLEEP_CMD, 10 },
    { 0x1F, 0xD4 },
    { MDDI_SLEEP_CMD, 5 },

//Gamma Setting
    { 0x40, 0x08 },
    { 0x41, 0x31 },
    { 0x42, 0x2F },
    { 0x43, 0x3E },
    { 0x44, 0x3D },
    { 0x45, 0x3F },
    { 0x46, 0x2F },
    { 0x47, 0x79 },
    { 0x48, 0x08 },
    { 0x49, 0x06 },
    { 0x4A, 0x08 },
    { 0x4B, 0x0E },
    { 0x4C, 0x17 },

    { 0x50, 0x00 },
    { 0x51, 0x02 },
    { 0x52, 0x01 },
    { 0x53, 0x10 },

    { 0x54, 0x0E },
    { 0x55, 0x37 },
    { 0x56, 0x06 },
    { 0x57, 0x50 },
    { 0x58, 0x08 },
    { 0x59, 0x11 },
    { 0x5A, 0x17 },
    { 0x5B, 0x19 },
    { 0x5C, 0x17 },
    { 0x5D, 0xFF },

    { 0x02, 0x00 },
    { 0x03, 0x00 },
    { 0x04, 0x00 },
    { 0x05, 0xEF },
    { 0x06, 0x00 },
    { 0x07, 0x00 },
    { 0x08, 0x01 },
    { 0x09, 0xAF },

    { 0x60, 0x08 },
    { 0x84, 0x00 },
    { 0x85, 0x00 },

// Display ON Setting
    { 0x28, 0x20 },
    { MDDI_SLEEP_CMD, 40 },
    { 0x28, 0x38 },
    { MDDI_SLEEP_CMD, 40 },
    { 0x28, 0x3C },

//    { 0x17, 0x08 },
    { 0x16, 0x0B },

    { TABLE_END, 1},

};

static struct init_table const tpo_hx8352b_plat_init[] =
{
    //Wake up
// Standby off
    { 0x19, 0x01 },
    { MDDI_SLEEP_CMD, 10 },
    { 0x1F, 0x8C },
// Power on
    { 0x1F, 0x8C }, // Power Control 1
    { 0x1F, 0x84 }, // Power Control 1
    { MDDI_SLEEP_CMD, 10 },
    { 0x1F, 0x94 }, // Power Control 1
    { MDDI_SLEEP_CMD, 10 },
    { 0x1F, 0xD4 }, // Power Control 1
    { MDDI_SLEEP_CMD, 5 },

// Display ON Setting
    { 0x28, 0x20 },
    { MDDI_SLEEP_CMD, 40 },
    { 0x28, 0x38 },
    { MDDI_SLEEP_CMD, 40 },
    { 0x28, 0x3C },

    { TABLE_END, 1},

};

static struct init_table const tpo_hx8352b_plat_uninit[] =
{
    //Standby
// Display off Setting
    { 0x28, 0x3C },
    { MDDI_SLEEP_CMD, 40 },
    { 0x28, 0x38 },
    { MDDI_SLEEP_CMD, 40 },
    { 0x28, 0x20 },
// Power off
    { 0x1F, 0x94 }, // Power Control 1
    { MDDI_SLEEP_CMD, 10 },
    { 0x1F, 0x84 }, // Power Control 1
    { MDDI_SLEEP_CMD, 10 },
    { 0x1F, 0x8C }, // Power Control 1
    { MDDI_SLEEP_CMD, 5 },
// Standby on
    { 0x1F, 0x8D },
    { MDDI_SLEEP_CMD, 10 },
    { 0x19, 0x00 },

    { TABLE_END, 1},

};

static struct init_table const *disp_init = NULL;
//static struct init_table const *disp_uninit = tpo_hx8352b_plat_uninit;
//static struct init_table const *disp_esd_recovery = NULL;

/*=============================================================================
                              GLOBAL VARIABLES
=============================================================================*/
#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
extern mddi_client_capability_type mddi_client_capability_pkt;
#endif
/*=============================================================================
                              LOCAL FUNCTIONS
=============================================================================*/
static void xmit_panel_seq(struct init_table const *init_table)
{
   unsigned n;

   printk("xmit_panel_seq()\n");
   for (n = 0;; n++)
   {
      if (init_table[n].reg == TABLE_END)
      {
         break;
      }
      else if (init_table[n].reg == MDDI_SLEEP_CMD)
      {
         mddi_wait(init_table[n].val);
      }
      else
      {
         mddi_queue_register_write(init_table[n].reg, init_table[n].val, 0, 0);
      }
      printk(".");
   }
   printk("\nxmit_panel_seq() done\n");
}

// static void mddi_tpo_esd_poll(void* arg)
// {
//    uint32_t val;
//    struct platform_device *pdev = (struct platform_device *)arg;

//    msm_fb_down(pdev);
// #ifdef MDDI_TPO_ESD_DEBUG
//    printk(KERN_ERR "TPO NV - ESD register check START\n");
//    mddi_queue_register_write(ENTRYMODE2 ,0x00C4, 0, 0);
//    mddi_wait(20);
//    for (i = 0;; i++)
//    {
//       if (disp_init[i].reg == TABLE_END)
//       {
//          break;
//       }
//       else if (disp_init[i].reg != MDDI_SLEEP_CMD)
//       {
//          mddi_queue_register_read(disp_init[i].reg, &val, 1, 0);
//          if (disp_init[i].val != val)
//          {
//             printk(KERN_ERR "TPO NV reg 0x%X should be 0x%X, found 0x%X\n",
//                disp_init[i].reg, disp_init[i].val, val);
//          }
//       }
//    }
//    mddi_queue_register_write(ENTRYMODE2 ,0x00C5, 0, 0);
//    mddi_wait(30);
//    printk(KERN_ERR "TPO NV - ESD register check END\n");
// #endif /* MDDI_TPO_ESD_DEBUG */
//    if (disp_esd_recovery)
//    {
//       xmit_panel_seq(disp_esd_recovery);
//    }
//    msm_fb_up(pdev);
// }

static int mddi_tpo_seq_init(void)
{
   if (plat_disp_type)
   {
      disp_init = tpo_hx8352b_plat_init;
      //disp_esd_recovery = tpo_plat_esd_recovery;
   }
   else
   {
//       unsigned userid = 0;
//       unsigned revision = 0;

//       mddi_queue_register_read(USERID, &userid, TRUE, 0);
//       mddi_queue_register_read(REVISIONID, &revision, TRUE, 0);

       disp_init = tpo_hx8352b_init;
   }
   if (!disp_init)
   {
      return -EINVAL;
   }
   return 0;
}

static int mddi_tpo_lcd_on(struct platform_device *pdev)
{
   struct msm_fb_data_type *mfd;
   //unsigned int val = 3;
   //int rc;

   mfd = platform_get_drvdata(pdev);

   if (!mfd || mfd->panel.id != TPO_QVGA_PRIM)
      return -ENODEV;

   if (mfd->key != MFD_KEY)
      return -EINVAL;

   if (!disp_init)
   {
      if (mddi_tpo_seq_init())
      {
         return -EINVAL;
      }
   }

   if (tpo_powered_on == FALSE)
   {
      xmit_panel_seq(disp_init);
      //esd_poll_start(mddi_tpo_esd_poll, pdev);
      tpo_powered_on = TRUE;

      //HACK
      //turn on backlight
      gpio_set_value(89, 1);
   }
   return 0;
}

static int mddi_tpo_lcd_off(struct platform_device *pdev)
{
    //int rc;

   if (!disp_init)
   {
      if (mddi_tpo_seq_init())
      {
         return -EINVAL;
      }
   }

   if (tpo_powered_on == TRUE)
   {
       //esd_poll_stop(mddi_tpo_esd_poll);
      xmit_panel_seq(tpo_hx8352b_plat_uninit);
      tpo_powered_on = FALSE;
      //HACK
      //turn off backlight
      gpio_set_value(89, 0);
   }
   return 0;
}

static int __init mddi_tpo_probe(struct platform_device *pdev)
{
   msm_fb_add_device(pdev);

   return 0;
}

static struct platform_driver mddi_tpo_driver = {
   .probe = mddi_tpo_probe,
   .driver = {
      .name = "mddi_tpo_qvga"
   },
};

static struct msm_fb_panel_data mddi_tpo_panel_data = {
   .on = mddi_tpo_lcd_on,
   .off = mddi_tpo_lcd_off,
};

static struct platform_device mddi_tpo_device = {
   .name = "mddi_tpo_qvga",
   .id = TPO_QVGA_PRIM,
   .dev = {
      .platform_data = &mddi_tpo_panel_data,
   }
};

static int __init mddi_tpo_init(void)
{
   int ret;
   struct msm_panel_info *pinfo;

   /* HACK HACK HACK HACK HACK HACK HACK HACK HACK HACK
    *
    * This will be removed later.
    *
    * if FLIP_CLOSED, then don't enable this LCD. For now
    * LCD selection is a static boot time configuration.
    */
//    if (gpio_get_value(42))
//    {
//       return 0;
//    }
   /* HACK HACK HACK HACK HACK HACK HACK HACK HACK HACK */

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT

   ret = msm_fb_detect_client("mddi_tpo_qvga");
   if (ret == -ENODEV)
       return 0;

   mddi_wait(10); // wait to come completely out of reset
   plat_disp_type = mddi_get_client_id();

   if((plat_disp_type & 0x00ff) != 0x8352){
       printk("%s: panel appears to be unknown type. Exiting\n", __FUNCTION__);
// For Bringup, don't return
//       return 0;
   }

   printk("%s: client_id=0x%X, crc=0x%X, width=%d, height=%d\n",
      __FUNCTION__, plat_disp_type, mddi_client_capability_pkt.parameter_CRC,
      mddi_client_capability_pkt.Bitmap_Width,
      mddi_client_capability_pkt.Bitmap_Height);
    if (mddi_client_capability_pkt.Bitmap_Width == 0x00 ||
       mddi_client_capability_pkt.Bitmap_Height == 0x00)
   {
      printk("%s: panel appears to be missing. Exiting\n", __FUNCTION__);
// For Bringup, don't return
//       return 0;
    }
#endif

   ret = platform_driver_register(&mddi_tpo_driver);
   if (!ret) {
      pinfo = &mddi_tpo_panel_data.panel_info;
      pinfo->xres = LCD_WIDTH;
      pinfo->yres = LCD_HEIGHT;
      pinfo->type = MDDI_PANEL;
      pinfo->pdest = DISPLAY_1;
      pinfo->wait_cycle = 0;
      pinfo->bpp = 18;
      pinfo->fb_num = 2;
      pinfo->clk_min = 160000000;
      pinfo->clk_max = 160000000;
      pinfo->clk_rate = 160000000;
      pinfo->lcd.vsync_enable = TRUE;
      pinfo->lcd.refx100 =
         (mddi_tpo_rows_per_second * 100) /
         mddi_tpo_rows_per_refresh;
      pinfo->lcd.v_back_porch = V_BACK_PORCH;
      pinfo->lcd.v_front_porch = V_FRONT_PORCH;
      pinfo->lcd.v_pulse_width = V_SYNC_WIDTH;
      pinfo->lcd.hw_vsync_mode = FALSE;
      pinfo->lcd.vsync_notifier_period = (1 * HZ);
      pinfo->bl_max = 7;
      pinfo->bl_min = 1;

      ret = platform_device_register(&mddi_tpo_device);
      if (ret)
      {
         platform_driver_unregister(&mddi_tpo_driver);
      }
   }
   return ret;
}

module_init(mddi_tpo_init);


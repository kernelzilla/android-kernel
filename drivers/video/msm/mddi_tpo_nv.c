/* drivers/video/msm/mddi_tpo_nv.c
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

#include <linux/delay.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/leds.h>
#include <mach/gpio.h>
#include "mddi_tpo_nv.h"
#include "mddi_mipi_tpo_nv.h"

#define TPO_NV_HVGA_PRIM  1
#define MDDI_SLEEP_CMD     0xffffffff

#define LCD_WIDTH       320
#define LCD_HEIGHT      480
#define V_SYNC_WIDTH    0
#define V_BACK_PORCH    0
#define V_FRONT_PORCH   0

#define MANPRODID_TMD	0xB9F65451
#define MANPRODID_TPO   0xB9F65395

//#define MOT_VSYNC

extern void lmxxxx_set_pwm (unsigned);
/*=============================================================================
                        LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
=============================================================================*/
struct init_table {
    unsigned int reg;
    unsigned int val;
};

struct motfb_enable_vsync {
int enable;
};

struct motfb_write_reg {
unsigned reg;
unsigned val;
};

struct motfb_read_reg {
unsigned reg;
unsigned val;
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
#ifdef MOT_VSYNC
static DECLARE_WAIT_QUEUE_HEAD(mddi_tpo_nt_vsync_wait);
static int volatile mddi_tpo_nv_got_vsync_irq = -1;
static int mddi_tpo_nv_vsync_irq = -1;
#endif

static unsigned manprodid = 0;

static int platform_panel_enabled = 1;
static int mddi_tpo_nv_pan_updt_supported = 0;
static int mddi_tpo_nv_cabc_en = 0;
static struct platform_device *als_pdev = NULL;

static uint32 mddi_tpo_nv_rows_per_second = LCD_HEIGHT * 100; // 90Hz
static uint32 mddi_tpo_nv_rows_per_refresh = LCD_HEIGHT;
static boolean tpo_nv_powered_on = FALSE;

static struct init_table const tpo_nv_init_defines_vers_pre_F[] = {

//Wait 10 msec or more after HW reset
    {MDDI_SLEEP_CMD,10             }, // wait 10ms

//Set Sleep-Out command    - Set Register access packet Address = 110h, data = 0h
    {SLEPCTRL ,0x0000             },
 
//Wait 10 msec or more
    {MDDI_SLEEP_CMD,10            }, // wait 10ms

    {PANELCTRL1 ,0x0044           },
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x0087           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x007B           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x000F           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },

    {ENTRYMODE1 ,0x0030           }, //ENTRYMODE1: Entry Mode 1 (360h)
    {ENTRYMODE2 ,0x00C5           }, //0x00C4           }, //ENTRYMODE2: Entry Mode 2 (361h)

    {DRVCTRL ,0x0012              },
    {DRV_OUT_CTRL1 ,0x00EF        },
    {DRV_OUT_CTRL2 ,0x0001        },

    {SPECTRL ,0x0080              },

    {WHRAMLOWST ,0x0000           }, //Window Horizontal RAM Low Address Start
    {WHRAMHIGHST ,0x0000          }, //Window Horizontal RAM High Address Start (2A1h)
    {WHRAMLOWEND ,0x003F          }, //Window Horizontal RAM Low Address End (2A2h) 
    {WHRAMHIGHEND ,0x0001         }, //Window Horizontal RAM High Address End (2A3h)
    {WVRAMLOWST ,0x0000           }, //Window Vertical RAM Low Address Start (2B0h)
    {WVRAMHIGHST ,0x0000          }, //Window Vertical RAM High Address Start (2B1h) 
    {WVRAMLOWEND ,0x00DF          }, //Window Vertical RAM Low Address End (2B2h)
    {WVRAMHIGHEND ,0x0001         }, //Window Vertical RAM High Address End (2B3h) 
    {RAMHSETL ,0x0000             }, //RAMHSETL: RAM Low Address Set (Horizontal Address) (2D0h)    
    {RAMHSETH ,0x0000             }, //RAMHSETH: RAM High Address Set (Horizontal Address) (2D1h)
    {RAMVSETL ,0x0000             }, //RAMVSETL: RAM Low Address Set (Vertical Address) (2D2h) 
    {RAMVSETH ,0x0000             }, //RAMVSETH: RAM High Address Set (Vertical Address) (2D3h) 

    {FPOSITION, 0x01              }, //FPOSITION: Frame Tearing Effect Position (350h)
    {FCYCLE ,0x0000               }, //FCYCLE: Frame Tearing Effect Position and Output cycle(351h)

    {PWRCTRL2 ,0x0010             }, //Power Control 2 (0xC31)

    {PUMPCTRL1, 0x0000            }, //DC/DC on Glass Control 1 (0xBA0)
    {PUMPCTRL2, 0x0086            }, //DC/DC on Glass Control 2 (0xBA1)

    {VCOMHCTRL, 0x0033            }, //VCOM_H Voltage Control (C50h)
    {VCOMDCCTRL, 0x3A             }, //VCOM_DC Voltage Control (C52h)

    {PWRCTRL5 ,0x0038             }, //Power Control 5 (C40h)

    {SOUTCTRL1 ,0x0001            }, //Source Output Control 1 (BE0h)
    {SOUTCTRL2 ,0x0000            }, //Source Output Control 2 (BE1h)

    {WRCTRLD, 0x0000              }, //CABC: Write CTRL Display (530h)
    {WRCABC, 0x0000               }, //CABC: Content Adaptive Brightness Control (550h)

    // Red Gamma positive polarity
    {GMACTRL1_0 ,0x0001           },
    {GMACTRL1_1 ,0x0008           },
    {GMACTRL1_2 ,0x000E           },
    {GMACTRL1_3 ,0x000F           },
    {GMACTRL1_4 ,0x002A           },
    {GMACTRL1_5 ,0x0025           },
    {GMACTRL1_6 ,0x0016           },
    {GMACTRL1_7 ,0x0016           },
    {GMACTRL1_8 ,0x0010           },
    {GMACTRL1_9 ,0x003A           },
    {GMACTRL1_A ,0x0016           },
    {GMACTRL1_B ,0x002B           },
    {GMACTRL1_C ,0x0041           },
    {GMACTRL1_D ,0x0031           },
    {GMACTRL1_E ,0x0046           },
    {GMACTRL1_F ,0x0067           },

    // Red Gamma negative polarity
    {GMACTRL2_0 ,0x0000           },
    {GMACTRL2_1 ,0x0022           },
    {GMACTRL2_2 ,0x0036           },
    {GMACTRL2_3 ,0x001E           },
    {GMACTRL2_4 ,0x0033           },
    {GMACTRL2_5 ,0x0029           },
    {GMACTRL2_6 ,0x002C           },
    {GMACTRL2_7 ,0x000D           },
    {GMACTRL2_8 ,0x0008           },
    {GMACTRL2_9 ,0x0050           },
    {GMACTRL2_A ,0x001A           },
    {GMACTRL2_B ,0x0034           },
    {GMACTRL2_C ,0x004F           },
    {GMACTRL2_D ,0x005A           },
    {GMACTRL2_E ,0x005F           },
    {GMACTRL2_F ,0x0066           },

    // Green Gamma positive polarity
    {GMACTRL3_0 ,0x0016           },
    {GMACTRL3_1 ,0x001D           },
    {GMACTRL3_2 ,0x0023           },
    {GMACTRL3_3 ,0x0012           },
    {GMACTRL3_4 ,0x002B           },
    {GMACTRL3_5 ,0x0026           },
    {GMACTRL3_6 ,0x0020           },
    {GMACTRL3_7 ,0x0015           },
    {GMACTRL3_8 ,0x0010           },
    {GMACTRL3_9 ,0x003F           },
    {GMACTRL3_A ,0x0015           },
    {GMACTRL3_B ,0x0029           },
    {GMACTRL3_C ,0x0041           },
    {GMACTRL3_D ,0x0033           },
    {GMACTRL3_E ,0x003A           },
    {GMACTRL3_F ,0x0067           },

    // Green Gamma negative polarity
    {GMACTRL4_0 ,0x0000           },
    {GMACTRL4_1 ,0x002E           },
    {GMACTRL4_2 ,0x003A           },
    {GMACTRL4_3 ,0x001C           },
    {GMACTRL4_4 ,0x0034           },
    {GMACTRL4_5 ,0x002A           },
    {GMACTRL4_6 ,0x0027           },
    {GMACTRL4_7 ,0x000E           },
    {GMACTRL4_8 ,0x0009           },
    {GMACTRL4_9 ,0x0046           },
    {GMACTRL4_A ,0x001A           },
    {GMACTRL4_B ,0x0034           },
    {GMACTRL4_C ,0x004E           },
    {GMACTRL4_D ,0x0044           },
    {GMACTRL4_E ,0x004A           },
    {GMACTRL4_F ,0x0051           },

    // Blue Gamma positive polarity
    {GMACTRL5_0 ,0x0008           },
    {GMACTRL5_1 ,0x0017           },
    {GMACTRL5_2 ,0x0022           },
    {GMACTRL5_3 ,0x0015           },
    {GMACTRL5_4 ,0x0031           },
    {GMACTRL5_5 ,0x0029           },
    {GMACTRL5_6 ,0x0028           },
    {GMACTRL5_7 ,0x0015           },
    {GMACTRL5_8 ,0x0010           },
    {GMACTRL5_9 ,0x0043           },
    {GMACTRL5_A ,0x0015           },
    {GMACTRL5_B ,0x0029           },
    {GMACTRL5_C ,0x0040           },
    {GMACTRL5_D ,0x0036           },
    {GMACTRL5_E ,0x004A           },
    {GMACTRL5_F ,0x0067           },

    // Blue Gamma negative polarity
    {GMACTRL6_0 ,0x0000           },
    {GMACTRL6_1 ,0x001D           },
    {GMACTRL6_2 ,0x0031           },
    {GMACTRL6_3 ,0x001E           },
    {GMACTRL6_4 ,0x0036           },
    {GMACTRL6_5 ,0x002B           },
    {GMACTRL6_6 ,0x0023           },
    {GMACTRL6_7 ,0x000D           },
    {GMACTRL6_8 ,0x0009           },
    {GMACTRL6_9 ,0x003E           },
    {GMACTRL6_A ,0x0016           },
    {GMACTRL6_B ,0x002D           },
    {GMACTRL6_C ,0x0048           },
    {GMACTRL6_D ,0x0046           },
    {GMACTRL6_E ,0x0051           },
    {GMACTRL6_F ,0x005F           },

    {DISPON, 1                    }, //display on

    {MDDI_SLEEP_CMD,0             } // terminate table
};

static struct init_table const tpo_nv_init_defines_vers_F[] = 
{
//Wait 10 msec or more after HW reset
    {MDDI_SLEEP_CMD,10             }, // wait 10ms

//Set Sleep-Out command    - Set Register access packet Address = 110h, data = 0h
    {SLEPCTRL ,0x0000             },
 
//Wait 10 msec or more
    {MDDI_SLEEP_CMD,10            }, // wait 10ms

    {PANELCTRL1 ,0x0044           },
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x0087           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x007B           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x000F           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },

    {ENTRYMODE1 ,0x0030           }, //ENTRYMODE1: Entry Mode 1 (360h)
    {ENTRYMODE2 ,0x00C5           }, //0x00C4           }, //ENTRYMODE2: Entry Mode 2 (361h)

    {DRVCTRL ,0x0012              },
    {DRV_OUT_CTRL1 ,0x00EF        },
    {DRV_OUT_CTRL2 ,0x0003        },

    {SPECTRL ,0x0002              },

    {WHRAMLOWST ,0x0000           }, //Window Horizontal RAM Low Address Start
    {WHRAMHIGHST ,0x0000          }, //Window Horizontal RAM High Address Start (2A1h)
    {WHRAMLOWEND ,0x003F          }, //Window Horizontal RAM Low Address End (2A2h) 
    {WHRAMHIGHEND ,0x0001         }, //Window Horizontal RAM High Address End (2A3h)
    {WVRAMLOWST ,0x0000           }, //Window Vertical RAM Low Address Start (2B0h)
    {WVRAMHIGHST ,0x0000          }, //Window Vertical RAM High Address Start (2B1h) 
    {WVRAMLOWEND ,0x00DF          }, //Window Vertical RAM Low Address End (2B2h)
    {WVRAMHIGHEND ,0x0001         }, //Window Vertical RAM High Address End (2B3h) 
    {RAMHSETL ,0x0000             }, //RAMHSETL: RAM Low Address Set (Horizontal Address) (2D0h)    
    {RAMHSETH ,0x0000             }, //RAMHSETH: RAM High Address Set (Horizontal Address) (2D1h)
    {RAMVSETL ,0x0000             }, //RAMVSETL: RAM Low Address Set (Vertical Address) (2D2h) 
    {RAMVSETH ,0x0000             }, //RAMVSETH: RAM High Address Set (Vertical Address) (2D3h) 

    {FPOSITION ,0x1               }, //FPOSITION: Frame Tearing Effect Position (350h)
    {FCYCLE ,0x0000               }, //FCYCLE: Frame Tearing Effect Position and Output cycle(351h)

    {PWRCTRL2 ,0x0010             }, //Power Control 2 (0xC31)
    {PWRCTRL4 ,0x0010             }, //Power Control 4 (C31h)

    {PUMPCTRL1, 0x0000            }, //DC/DC on Glass Control 1 (0xBA0)
    {PUMPCTRL2, 0x0086            }, //DC/DC on Glass Control 2 (0xBA1)

    {WRCTRLD, 0x0000              }, //CABC: Write CTRL Display (530h)
    {WRCABC, 0x0000               }, //CABC: Content Adaptive Brightness Control (550h)

    {VCOMHCTRL, 0x0033            }, //VCOM_H Voltage Control (C50h)
    {VCOMDCCTRL, 0x0032           }, //VCOM_DC Voltage Control (C52h)

    {PWRCTRL5 ,0x0034             }, //Power Control 5 (C40h)

    {SOUTCTRL1 ,0x0001            }, //Source Output Control 1 (BE0h)
    {SOUTCTRL2 ,0x0000            }, //Source Output Control 2 (BE1h)

    // Red Gamma positive polarity
    {GMACTRL1_0 ,0x0000           },
    {GMACTRL1_1 ,0x0007           },
    {GMACTRL1_2 ,0x000C           },
    {GMACTRL1_3 ,0x0012           },
    {GMACTRL1_4 ,0x002C           },
    {GMACTRL1_5 ,0x0026           },
    {GMACTRL1_6 ,0x0012           },
    {GMACTRL1_7 ,0x0013           },
    {GMACTRL1_8 ,0x0011           },
    {GMACTRL1_9 ,0x0039           },
    {GMACTRL1_A ,0x0015           },
    {GMACTRL1_B ,0x0025           },
    {GMACTRL1_C ,0x003B           },
    {GMACTRL1_D ,0x0032           },
    {GMACTRL1_E ,0x0054           },
    {GMACTRL1_F ,0x006D           },

    // Red Gamma negative polarity
    {GMACTRL2_0 ,0x0001           },
    {GMACTRL2_1 ,0x0019           },
    {GMACTRL2_2 ,0x003B           },
    {GMACTRL2_3 ,0x0025           },
    {GMACTRL2_4 ,0x003B           },
    {GMACTRL2_5 ,0x002C           },
    {GMACTRL2_6 ,0x002E           },
    {GMACTRL2_7 ,0x000A           },
    {GMACTRL2_8 ,0x000A           },
    {GMACTRL2_9 ,0x004E           },
    {GMACTRL2_A ,0x0019           },
    {GMACTRL2_B ,0x0032           },
    {GMACTRL2_C ,0x004C           },
    {GMACTRL2_D ,0x0051           },
    {GMACTRL2_E ,0x0056           },
    {GMACTRL2_F ,0x005C           },

    // Green Gamma positive polarity
    {GMACTRL3_0 ,0x0000           },
    {GMACTRL3_1 ,0x0009           },
    {GMACTRL3_2 ,0x0011           },
    {GMACTRL3_3 ,0x0013           },
    {GMACTRL3_4 ,0x002E           },
    {GMACTRL3_5 ,0x0027           },
    {GMACTRL3_6 ,0x001B           },
    {GMACTRL3_7 ,0x0015           },
    {GMACTRL3_8 ,0x0011           },
    {GMACTRL3_9 ,0x003C           },
    {GMACTRL3_A ,0x0014           },
    {GMACTRL3_B ,0x0022           },
    {GMACTRL3_C ,0x0039           },
    {GMACTRL3_D ,0x0039           },
    {GMACTRL3_E ,0x0058           },
    {GMACTRL3_F ,0x006D           },

    // Green Gamma negative polarity
    {GMACTRL4_0 ,0x0001           },
    {GMACTRL4_1 ,0x0015           },
    {GMACTRL4_2 ,0x0034           },
    {GMACTRL4_3 ,0x0028           },
    {GMACTRL4_4 ,0x003F           },
    {GMACTRL4_5 ,0x002D           },
    {GMACTRL4_6 ,0x002B           },
    {GMACTRL4_7 ,0x000C           },
    {GMACTRL4_8 ,0x0006           },
    {GMACTRL4_9 ,0x0046           },
    {GMACTRL4_A ,0x0017           },
    {GMACTRL4_B ,0x002F           },
    {GMACTRL4_C ,0x004B           },
    {GMACTRL4_D ,0x004C           },
    {GMACTRL4_E ,0x0053           },
    {GMACTRL4_F ,0x005C           },

    // Blue Gamma positive polarity
    {GMACTRL5_0 ,0x0000           },
    {GMACTRL5_1 ,0x0005           },
    {GMACTRL5_2 ,0x0017           },
    {GMACTRL5_3 ,0x0013           },
    {GMACTRL5_4 ,0x0030           },
    {GMACTRL5_5 ,0x0027           },
    {GMACTRL5_6 ,0x0021           },
    {GMACTRL5_7 ,0x0013           },
    {GMACTRL5_8 ,0x000F           },
    {GMACTRL5_9 ,0x0040           },
    {GMACTRL5_A ,0x0016           },
    {GMACTRL5_B ,0x0027           },
    {GMACTRL5_C ,0x0042           },
    {GMACTRL5_D ,0x0037           },
    {GMACTRL5_E ,0x006C           },
    {GMACTRL5_F ,0x006D           },

    // Blue Gamma negative polarity
    {GMACTRL6_0 ,0x0001           },
    {GMACTRL6_1 ,0x0005           },
    {GMACTRL6_2 ,0x0035           },
    {GMACTRL6_3 ,0x001F           },
    {GMACTRL6_4 ,0x003A           },
    {GMACTRL6_5 ,0x002A           },
    {GMACTRL6_6 ,0x0027           },
    {GMACTRL6_7 ,0x000D           },
    {GMACTRL6_8 ,0x0009           },
    {GMACTRL6_9 ,0x0040           },
    {GMACTRL6_A ,0x0017           },
    {GMACTRL6_B ,0x002D           },
    {GMACTRL6_C ,0x004A           },
    {GMACTRL6_D ,0x0046           },
    {GMACTRL6_E ,0x0058           },
    {GMACTRL6_F ,0x005C           },

    {DISPON, 1                    }, //display on

    {MDDI_SLEEP_CMD,0             } // terminate table
};

static struct init_table const tpo_nv_init_defines_vers_G[] = 
{
//Wait 10 msec or more after HW reset
    {MDDI_SLEEP_CMD,10             }, // wait 10ms

//Set Sleep-Out command    - Set Register access packet Address = 110h, data = 0h
    {SLEPCTRL ,0x0000             },
 
//Wait 10 msec or more
    {MDDI_SLEEP_CMD,10            }, // wait 10ms

#ifdef NINETY_HZ
    {PANELCTRL1 ,0x002E           }, // 90Hz
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x005B           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x004F           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x0006           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },

    {PANELCTRL1 ,0x0044           }, // 60Hz
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x0087           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x007B           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x000F           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },
#endif

    {PANELCTRL1 ,0x0037           }, // 75Hz
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x006D           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x0061           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x0009           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },

    {ENTRYMODE1 ,0x0030           }, //ENTRYMODE1: Entry Mode 1 (360h)
    {ENTRYMODE2 ,0x00C5           }, //0x00C4           }, //ENTRYMODE2: Entry Mode 2 (361h)

    {DRVCTRL ,0x0012              },
    {DRV_OUT_CTRL1 ,0x00EF        },

    {PWRCTRL4 ,0x0010             }, //Power Control 4 (C31h)

    {PUMPCTRL1, 0x0000            }, //DC/DC on Glass Control 1 (0xBA0)
    {PUMPCTRL2, 0x0086            }, //DC/DC on Glass Control 2 (0xBA1)

    {FPOSITION, 0x1               }, //FPOSITION: Frame Tearing Effect Position (350h)
    {FCYCLE ,0x0000               }, //FCYCLE: Frame Tearing Effect Position and Output cycle(351h)

    {WRDISBV, 0xff                }, //CABC: Write Display Brightness (510h)
    {WRCTRLD, 0x13                }, //CABC: Write CTRL Display (530h) Disable dimming. It causes a 4..5
                                     //second delay in backlight enable when resumed
    {WRCABC, 0x02                 }, //CABC: Content Adaptive Brightness Control (550h)
    {0x6d9, 0                     }, //CABC:
    {0x6e4, 31                    }, //CABC:
    {0x6e5, 15                    }, //CABC:
    {0x6e6, 11                    }, //CABC: dimming time (fastest)
    {0x6e7, 0                     }, //CABC:

    {PWRCTRL5 ,0x0034             }, //Power Control 5 (C40h)

    {SOUTCTRL1 ,0x0001            }, //Source Output Control 1 (BE0h)
    {SOUTCTRL2 ,0x0000            }, //Source Output Control 2 (BE1h)

    {WHRAMLOWST ,0x0000           }, //Window Horizontal RAM Low Address Start
    {WHRAMHIGHST ,0x0000          }, //Window Horizontal RAM High Address Start (2A1h)
    {WHRAMLOWEND ,0x003F          }, //Window Horizontal RAM Low Address End (2A2h) 
    {WHRAMHIGHEND ,0x0001         }, //Window Horizontal RAM High Address End (2A3h)
    {WVRAMLOWST ,0x0000           }, //Window Vertical RAM Low Address Start (2B0h)
    {WVRAMHIGHST ,0x0000          }, //Window Vertical RAM High Address Start (2B1h) 
    {WVRAMLOWEND ,0x00DF          }, //Window Vertical RAM Low Address End (2B2h)
    {WVRAMHIGHEND ,0x0001         }, //Window Vertical RAM High Address End (2B3h) 
    {RAMHSETL ,0x0000             }, //RAMHSETL: RAM Low Address Set (Horizontal Address) (2D0h)    
    {RAMHSETH ,0x0000             }, //RAMHSETH: RAM High Address Set (Horizontal Address) (2D1h)
    {RAMVSETL ,0x0000             }, //RAMVSETL: RAM Low Address Set (Vertical Address) (2D2h) 
    {RAMVSETH ,0x0000             }, //RAMVSETH: RAM High Address Set (Vertical Address) (2D3h) 

    {DISPON, 1                    }, //display on

    {MDDI_SLEEP_CMD,0             } // terminate table
};

static struct init_table const tpo_nv_init_defines_plat_tpo[] = 
{
//Wait 10 msec or more after HW reset
    {MDDI_SLEEP_CMD,10            }, // wait 10ms

//Set Sleep-Out command    - Set Register access packet Address = 110h, data = 0h
    {MM_SLEEPOUT ,0x0000          },
 
//Wait 10 msec or more
    {MDDI_SLEEP_CMD,10            }, // wait 10ms

    {MM_DIMFUNC1, 0               }, //dimming control 1
    {MM_DIMFUNC2, 0x14            }, //dimming control 2

    {MM_DISP_BRIGHTNESS, 0xff     }, //display brightness
    {MM_WRCTRLD , 0x0013          },
    {MM_WRCABC , 0x0002           },
    {0x6902, 11                   }, //CABC: dimming time length (6 frames)
    {0x6d0d, 0x30                 },
    {0x6e02, 0xB4                 },
    {0x6e03, 1                    },
    {0x6e06, 0x04                 },

//Still mode PWM
    {0x6D00, 0xE6                 },
    {0x6D01, 0xD9                 },
    {0x6D02, 0xCC                 },
    {0x6D03, 0xBF                 },
    {0x6D04, 0xB3                 },
    {0x6D05, 0xA6                 },
    {0x6D06, 0x99                 },
    {0x6D07, 0x99                 },
    {0x6D08, 0x99                 },
    {0x6D09, 0x99                 },
//Moving mode PWM
    {0x6C02, 0xE6                 },
    {0x6C03, 0xD9                 },
    {0x6C04, 0xCC                 },
    {0x6C05, 0xBF                 },
    {0x6C06, 0xB3                 },
    {0x6C07, 0xA6                 },
    {0x6C08, 0x99                 },
    {0x6C09, 0x99                 },
    {0x6C0A, 0x99                 },
    {0x6C0B, 0x99                 },

    {MM_SET_TEAR_ON , 0x0000      },
    {MM_SET_TEAR_LINE ,   400     }, // to eliminate tearing, emit the FTE signal on the 400th line of the display 

    {MM_DDC_SAYS_SO, 0xA5         },

// Do NOT turn on display until after UI frame arrives.
// See mddi_tpo_nv_enable_panel() below
  //{MM_DISPON, 0                 }, //display on

    {MDDI_SLEEP_CMD,0             } // terminate table
};

static struct init_table const tpo_nv_init_defines_plat_tmd[] =
  {
    //Wait 10 msec or more after HW reset      
    {MDDI_SLEEP_CMD,10 }, // wait 10ms   
    //Set Sleep-Out command    - Set Register access packet Address = 110h, data = 0h    
    {MM_SLEEPOUT ,0x0000 },

    //Wait 10 msec or more     
    {MDDI_SLEEP_CMD,10 }, // wait 10ms 

    {MM_DISP_BRIGHTNESS, 0xff }, //display brightness   
    {MM_WRCTRLD , 0x002C },
    {MM_WRCABC , 0x0000 },
    {MM_SET_TEAR_ON , 0x0000 },
    {MM_SET_TEAR_LINE ,   144     }, // to eliminate tearing, emit the FTE signal on the 144th line of the display

// Do NOT turn on display until after UI frame arrives
// See mddi_tpo_nv_enable_panel() below
//    {MM_DISPON, 0 }, //display on 
    {MDDI_SLEEP_CMD,0 } // terminate table
  };

static struct init_table *tpo_nv_init_defines = tpo_nv_init_defines_vers_pre_F;

static struct init_table const tpo_nv_uninit_defines_interim[] =
{
   { DISPON, 0          }, //display off
   { MDDI_SLEEP_CMD, 5  }, // wait 5ms
   /*
    * Set Sleep-In command
    * Set Register access packet Address = 110h, data = 1h
    */
   { SLEPCTRL, 0x0001   },
   { MDDI_SLEEP_CMD, 0  } // terminate table
};

static struct init_table const tpo_nv_uninit_defines_platform[] =
{
   { MM_DISPOFF, 0      }, // turn display off
   { MDDI_SLEEP_CMD, 5  }, // wait 5ms
   { MM_SLEEPIN, 0      }, // put panel to sleep
   { MDDI_SLEEP_CMD, 20 }, // wait for a couple frames
   { MDDI_SLEEP_CMD, 0  } // terminate table
};

static struct init_table *tpo_nv_uninit_defines = tpo_nv_uninit_defines_interim;

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
    
    for(n = 0;;n++) {
        if(init_table[n].reg == MDDI_SLEEP_CMD ) {
            if(init_table[n].val == 0) break;
            mddi_wait(init_table[n].val);
        } else {
            if(mddi_queue_register_write(init_table[n].reg, init_table[n].val, TRUE, 0)) return;
        }
       printk(".");
    }
    printk("\nxmit_panel_seq() done\n");
}

static void mddi_tpo_nv_prim_lcd_init(struct platform_device *pdev)
{
   static unsigned userid = 0xFFFFFFFF;
   static unsigned revision;
   boolean init_display = TRUE;

   if (tpo_nv_powered_on == TRUE)
   {
      return;
   }

   msm_fb_down(pdev); // serialize access to the panel & MDDI while communicating

   if(manprodid != 0)
   {
     if((tpo_nv_init_defines != tpo_nv_init_defines_plat_tpo) && 
        (tpo_nv_init_defines != tpo_nv_init_defines_plat_tmd))
        {
            struct msm_fb_panel_data * pdata;

            init_display = FALSE; // skip reinitting the panel since bootloader already has

            pdata = (struct msm_fb_panel_data *)pdev->dev.platform_data;
            pdata->check_esd = 0;

            if(manprodid == MANPRODID_TMD)
            { 
              tpo_nv_init_defines = tpo_nv_init_defines_plat_tmd;
            }
            else
            {
              tpo_nv_init_defines = tpo_nv_init_defines_plat_tpo;
            }

            mddi_tpo_nv_cabc_en = 1;
            mddi_tpo_nv_pan_updt_supported = 1;
            tpo_nv_uninit_defines = tpo_nv_uninit_defines_platform;
        }
	printk("platform panel with man/prod id 0x%x!!\n", manprodid);
   }
   else
   {
       if(0xFFFFFFFF == userid)
       {
            init_display = FALSE; // skip reinitting the panel since bootloader already has.

            mddi_queue_register_read(USERID, &userid, TRUE, 0); // read panel Id once.
            mddi_queue_register_read(REVISIONID, &revision, TRUE, 0); // read panel rev once.

            switch(userid)
            {
                case 0x03:
                    /* disable PWM for backlight*/
                    lmxxxx_set_pwm(0);
                    mddi_tpo_nv_cabc_en = 0;
                break;
                
                case 0x05:
                    if(0x01 == revision)
                    {
                        tpo_nv_init_defines = tpo_nv_init_defines_vers_F;
                        /* disable PWM for backlight*/
                        lmxxxx_set_pwm(0);
                        mddi_tpo_nv_cabc_en = 0;
                    }
                    else
                    {
                        if(0x02 == revision)
                        {
                            tpo_nv_init_defines = tpo_nv_init_defines_vers_G;
                            mddi_tpo_nv_cabc_en = 1;
                        }
                    }
                break;

                case 0x06:
                        tpo_nv_init_defines = tpo_nv_init_defines_vers_G;
                        mddi_tpo_nv_cabc_en = 1;
                break;
            } // switch
       }
printk(KERN_INFO "panel USERID=%X, REV:%X\n", userid, revision);
   }

   if(TRUE == init_display) xmit_panel_seq(tpo_nv_init_defines);
   msm_fb_up(pdev);

   tpo_nv_powered_on = TRUE;
}

static void mddi_tpo_nv_lcd_powerdown(struct platform_device *pdev)
{
   if (tpo_nv_powered_on == FALSE)
   {
      return;
   }
   msm_fb_down(pdev); // serialize access to the panel & MDDI while communicating
   xmit_panel_seq(tpo_nv_uninit_defines);
   msm_fb_up(pdev);
   tpo_nv_powered_on = FALSE;
}

#ifdef CONFIG_FB_MSM_ESD

static int mddi_tpo_nv_check_esd(struct platform_device *pdev, int num_secs_in_delay, int num_frames_sent_while_delayed)
{
#ifdef MEASURE_TIME_BETWEEN_CHECKS
    static int tt1 = 0;
    static ktime_t t1 = {0}, vt = {0};
#endif

    if (tpo_nv_powered_on != TRUE)
    {
        return 0;
    }

#ifdef MEASURE_TIME_BETWEEN_CHECKS
    if(tt1 == 0)
    {
        t1 = ktime_get();
        vt = ktime_get_real();
        tt1++;
    }
    else
    {
        s64 dt;
        ktime_t t5;
        int actual_wait_sec;
        uint32 actual_wait_msec;

        dt = ktime_to_ns(ktime_sub(ktime_get(), t1));

        t5 = ktime_get_real();

        actual_wait_sec =
            (t5.tv.sec - vt.tv.sec);// * 1000000 + (t5.tv.nsec - vt.tv.nsec) / 1000;

        dt = div_s64(dt, 1000);

        actual_wait_msec = (uint32)(dt) / 1000;

        printk(KERN_INFO "tpo_nv_check_esd %lld %d %u\n", dt, actual_wait_sec, actual_wait_msec);

        t1 = ktime_get();
        vt = ktime_get_real();
#endif
        msm_fb_down(pdev); // serialize access to the panel while communicating via MDDI

        // Reconfigure the GRAM pointers (These go bad when ESD hits during Xfers). 
        // Characterized as "split-screen" errors.
        if(!mddi_queue_register_write(RAMHSETL, 0x0000, TRUE, 0) &&
           !mddi_queue_register_write(RAMHSETH, 0x0000, TRUE, 0) &&
           !mddi_queue_register_write(RAMVSETL, 0x0000, TRUE, 0) &&
           !mddi_queue_register_write(RAMVSETH, 0x0000, TRUE, 0))
        {
        // resend last frame iff we're recv'ing a low rate of frames
        if(num_frames_sent_while_delayed < num_secs_in_delay * 8)
        {
            extern void imdp_dma2_update(struct msm_fb_data_type *mfd);
            static struct msm_fb_data_type * mfd = NULL;

            if(NULL == mfd)
            {
                mfd = platform_get_drvdata(pdev);
                if(NULL == mfd)
                {
                    printk("Bad mfd retrieved from pdev: %X\n", pdev);
                    return 1;
                }
            }

            // resend the last screen image to correct a split screen w/o human intervention.
            imdp_dma2_update(mfd);
        }
        }
        msm_fb_up(pdev);

        return 0;

#ifdef MEASURE_TIME_BETWEEN_CHECKS
    }

    return 0;
#endif
}
#endif

static int mddi_tpo_nv_lcd_on(struct platform_device *pdev)
{
   struct msm_fb_data_type *mfd;

   mfd = platform_get_drvdata(pdev);

   if (!mfd || mfd->panel.id != TPO_NV_HVGA_PRIM)
      return -ENODEV;

   if (mfd->key != MFD_KEY)
      return -EINVAL;

   mddi_tpo_nv_prim_lcd_init(mfd->pdev);

#ifdef CONFIG_FB_MSM_ESD
   if (tpo_nv_powered_on == TRUE)
   {
       msm_fb_enable_esd_mgr(mfd->pdev);
   }
#endif

   als_pdev = mfd->pdev;

   platform_panel_enabled = 0;

   return tpo_nv_powered_on == FALSE;
}

static int mddi_tpo_nv_lcd_off(struct platform_device *pdev)
{
   struct msm_fb_data_type *mfd;

   mfd = platform_get_drvdata(pdev);

   if (!mfd || mfd->panel.id != TPO_NV_HVGA_PRIM)
      return -ENODEV;

   if (mfd->key != MFD_KEY)
      return -EINVAL;

#ifdef CONFIG_FB_MSM_ESD
   if (tpo_nv_powered_on == TRUE)
   {
       msm_fb_disable_esd_mgr(mfd->pdev);
   }
#endif
   mddi_tpo_nv_lcd_powerdown(mfd->pdev);
   return 0;
}

static int mddi_tpo_nv_pan_updt(struct platform_device *pdev)
{
   return mddi_tpo_nv_pan_updt_supported;
}

static void mddi_tpo_nv_enable_panel(struct platform_device *pdev)
{
    if(manprodid && !platform_panel_enabled) // only needed for platform panel
    {
        platform_panel_enabled = 1;
        msm_fb_down(pdev); // serialize access to the panel & MDDI while communicating
        mddi_queue_register_write(MM_DISPON, 0, TRUE, 0); 
        msm_fb_up(pdev);
    }
}

static void enable_cabc(struct platform_device *pdev, int en)
{
   unsigned reg_addr = 0;

   if(tpo_nv_init_defines_vers_G == tpo_nv_init_defines)
       reg_addr = WRCABC; // interim cabc command
   else if(manprodid == MANPRODID_TPO)
       reg_addr = MM_WRCABC; // platform display cabc command

   if(reg_addr)
   {
     msm_fb_down(pdev); // serialize access to the panel & MDDI while communicating
     mddi_queue_register_write(reg_addr, 2*en, TRUE, 0); 
     msm_fb_up(pdev);
     mddi_tpo_nv_cabc_en = en;
   }
}

static int mddi_tpo_nv_als_cb(unsigned old_zone, unsigned new_zone, uint32_t cookie)
{
    if (tpo_nv_powered_on == TRUE && als_pdev)
    {
        if(new_zone > 1 && mddi_tpo_nv_cabc_en) enable_cabc((struct platform_device *)als_pdev, 0); // moved from indoor to outdoor
        else if(new_zone < 2 && !mddi_tpo_nv_cabc_en) enable_cabc((struct platform_device *)als_pdev, 1); // moved from outdoor to indoor
    }

    return 0;
}

static int __init mddi_tpo_nv_probe(struct platform_device *pdev)
{
   extern int lm3535_register_als_callback (int (*cb)(unsigned old_zone, unsigned new_zone, uint32_t cookie), uint32_t cookie);

   msm_fb_add_device(pdev);

   lm3535_register_als_callback(mddi_tpo_nv_als_cb, 0);

   return 0;
}

#ifdef MOT_VSYNC
static int mddi_tpo_nv_wait_for_vsync(struct platform_device *pdev)
{
    if(mddi_tpo_nv_got_vsync_irq)
    {
        mddi_tpo_nv_got_vsync_irq = 0;
        enable_irq(mddi_tpo_nv_vsync_irq);
        return 1;
    }
    return 0;
}
#endif

static struct platform_driver mddi_tpo_nv_driver = {
   .probe = mddi_tpo_nv_probe,
   .driver = {
      .name = "mddi_tpo_nv_hvga"
   },
};

static struct msm_fb_panel_data mddi_tpo_nv_panel_data_0 = {
   .pan_updt_supported = mddi_tpo_nv_pan_updt,
   .enable_panel = mddi_tpo_nv_enable_panel,
   .on = mddi_tpo_nv_lcd_on,
   .off = mddi_tpo_nv_lcd_off,
#ifdef CONFIG_FB_MSM_ESD
   .check_esd = mddi_tpo_nv_check_esd
#endif
};

static struct platform_device mddi_tpo_nv_device_0 = {
   .name = "mddi_tpo_nv_hvga",
   .id = TPO_NV_HVGA_PRIM,
   .dev = {
      .platform_data = &mddi_tpo_nv_panel_data_0,
   }
};

#ifdef MOT_VSYNC
static irqreturn_t mddi_tpo_nv_vsync_interrupt(int irq, void *data)
{
   disable_irq(mddi_tpo_nv_vsync_irq);
   if(tpo_nv_powered_on)
   {
        mddi_tpo_nv_got_vsync_irq = 1;
        mdp_pipe_ctrl(MDP_DMA2_BLOCK, MDP_BLOCK_POWER_ON, TRUE);
        mdp_pipe_kickoff(MDP_DMA2_TERM, 0);
   }

   return IRQ_HANDLED;
}
#endif

static int __init mddi_tpo_nv_init(void)
{
   #ifdef MOT_VSYNC
   int gpio = 97;
   #endif
   int ret;
   struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
    msleep(10); // wait to come completely out of reset
    manprodid = mddi_get_client_id();
printk("\nget client id=0x%X h=%u w=%u\n\n", manprodid, mddi_client_capability_pkt.Bitmap_Width, mddi_client_capability_pkt.Bitmap_Height);
    if(mddi_client_capability_pkt.Bitmap_Width == 0 ||
       mddi_client_capability_pkt.Bitmap_Height == 0)
    {
        printk("\nmddi_tpo_nv_init() not able to detect panel! Leaving.\n");
        return -1;
    }
#endif

#ifdef MOT_VSYNC
    ret = gpio_request(gpio, "vsync");
    if (!ret)
    {
        ret = gpio_direction_input(gpio);
        if (!ret)
        {
            ret = mddi_tpo_nv_vsync_irq = gpio_to_irq(gpio);
            if (ret >= 0)
            {
               ret = request_irq(mddi_tpo_nv_vsync_irq,
                  mddi_tpo_nv_vsync_interrupt, IRQF_TRIGGER_RISING, "vsync",
                  NULL);
               if (!ret)
               {
                    mddi_tpo_nv_panel_data_0.wait_for_vsync = mddi_tpo_nv_wait_for_vsync;
               }
            }
        }
    }
#endif

   ret = platform_driver_register(&mddi_tpo_nv_driver);
   if (!ret) {
      pinfo = &mddi_tpo_nv_panel_data_0.panel_info;
      pinfo->xres = LCD_WIDTH;
      pinfo->yres = LCD_HEIGHT;
      pinfo->type = MDDI_PANEL;
      pinfo->pdest = DISPLAY_1;
      pinfo->wait_cycle = 0;
      pinfo->bpp = 18;
      pinfo->fb_num = 2;
      pinfo->clk_min = 160000000;//s192000000;//f137143000;//f153600000;//s122880000;
      pinfo->clk_max = 160000000;//s192000000;//f137143000;//f153600000;//s122880000;
      pinfo->clk_rate = 160000000;//s192000000;//f137143000;//f153600000;//s122880000;//s98304000;
      pinfo->lcd.vsync_enable = TRUE;
      pinfo->lcd.refx100 =
         (mddi_tpo_nv_rows_per_second * 100) /
         mddi_tpo_nv_rows_per_refresh;
      pinfo->lcd.v_back_porch = V_BACK_PORCH;
      pinfo->lcd.v_front_porch = V_FRONT_PORCH;
      pinfo->lcd.v_pulse_width = V_SYNC_WIDTH;
      #ifdef MOT_VSYNC
      pinfo->lcd.hw_vsync_mode = FALSE;
      #else
      	pinfo->lcd.hw_vsync_mode = TRUE;
      #endif
      pinfo->lcd.vsync_notifier_period = (1 * HZ);
      pinfo->bl_max = 7;
      pinfo->bl_min = 1;

      ret = platform_device_register(&mddi_tpo_nv_device_0);
      if (ret)
      {
         platform_driver_unregister(&mddi_tpo_nv_driver);
      }
   }

   return ret;
}

module_init(mddi_tpo_nv_init);

unsigned msm_fb_mddi_nv_get_manprodid(void)
{
   return manprodid;
}

EXPORT_SYMBOL(msm_fb_mddi_nv_get_manprodid);

void msm_fb_led_resumed(void)
{
    if(tpo_nv_powered_on == TRUE && als_pdev)
    {
       unsigned reg_addr = 0;

       if(tpo_nv_init_defines_vers_G == tpo_nv_init_defines)
           reg_addr = WRCTRLD; // interim cabc command
       else if(manprodid == MANPRODID_TPO)
           reg_addr = MM_WRCTRLD; // platform display cabc command

       if(reg_addr)
       {
            msm_fb_down(als_pdev); // serialize access to the panel & MDDI while communicating
            mddi_queue_register_write(reg_addr, 0x16, TRUE, 0); // enable dimming bit
            msm_fb_up(als_pdev);
       }
    }
}

EXPORT_SYMBOL(msm_fb_led_resumed);

void enable_vsync(struct platform_device *pdev, struct motfb_enable_vsync *pvsync)
{
}

void wr_reg(struct platform_device *pdev, struct motfb_write_reg *wreg)
{
   msm_fb_down(pdev); // serialize access to the panel & MDDI while communicating
   mddi_queue_register_write(wreg->reg, wreg->val, TRUE, 0);
   msm_fb_up(pdev);
}

void rd_reg(struct platform_device *pdev, struct motfb_read_reg *rreg)
{
   msm_fb_down(pdev); // serialize access to the panel & MDDI while communicating
   mddi_queue_register_read(rreg->reg, &rreg->val, TRUE, 0);
   msm_fb_up(pdev);
}

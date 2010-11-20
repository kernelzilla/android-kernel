/* drivers/video/msm/ebi2_smart_qvga.c
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

#include "msm_fb.h"
#include "ebi2_smart_qvga.h"

#include <linux/memory.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include "linux/proc_fs.h"

#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>

#define OUTPORT(x,y)    outp16(x,y)
#define INPORT(x)       inp16(x)
#define DISP_CMD_OUT(cmd) OUTPORT(DISP_CMD_PORT, cmd)
#define DISP_DATA_OUT(data) OUTPORT(DISP_DATA_PORT, data)
#define DISP_DATA_IN() INPORT(DISP_DATA_PORT)
#define DISP_REG_WRITE(addr, data) smart_qvga_disp_write_reg(addr, data)
#define DISP_REG_READ(addr)   smart_qvga_disp_read_reg(addr)

#if defined(CONFIG_KERNEL_MOTOROLA)
#if defined(CONFIG_MACH_PITTSBURGH)
#define FB_REUSE  1  /* framebuffer to reuse + 1, or fb0 in this case */
#else
#define FB_REUSE  0  /* do not reuse another framebuffer */
#endif
#endif

#define SMRTQ_PRINTK(fmt, ...) \
   if (smrtq_debug) \
      printk(fmt, ##__VA_ARGS__)

static bool smrtq_debug = FALSE;
static void *DISP_CMD_PORT = NULL;
static void *DISP_DATA_PORT = NULL;
static boolean disp_initialized = FALSE;
static uint8 display_pwr = DISPLAY_PWR_OFF;
static struct msm_fb_data_type *mfd = NULL;

#if defined(CONFIG_KERNEL_MOTOROLA)
static void smart_qvga_disp_set_rect(int x, int y, int xres, int yres, char __iomem *screen_base);
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
static void smart_qvga_disp_set_rect(int x, int y, int xres, int yres);
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */

static void smart_qvga_disp_init(struct platform_device *pdev);
static int smart_qvga_disp_off(struct platform_device *pdev);
static int smart_qvga_disp_on(struct platform_device *pdev);

struct init_table {
    uint16 reg;
    uint16 data;
};

static struct init_table const *disp_on = NULL;
static struct init_table const disp_on_v1[] =
{
   { MSEC_DELAY, 50 },
   { 0x0001, 0x0100 }, /* driver output control */
   { 0x0002, 0x0200 }, /* set inverion */
   { DISP_V1_SET_ENTRY_MODE_REG, 0x1030 }, /* set entry mode */
   { 0x0008, 0x0202 }, /* set back & front porch */
   { 0x0009, 0x0000 }, /* set scan interval */
   { 0x000a, 0x0000 }, /* set display control1 */
   { 0x000c, 0x0000 }, /* set RGB I/F display control */
   { 0x000d, 0x0000 }, /* set frame mark position */
   { 0x0060, 0x2700 }, /* set gate scan control */
   { 0x0061, 0x0001 }, /* Normally White */
   { 0x006a, 0x0000 }, /* set gate scan control */
   { MSEC_DELAY, 10 }, /* delay 10ms */
   { 0x0010, 0x1190 }, /* set BT, STB & SLP */
   { 0x0011, 0x0227 }, /* set VCi1 & step up circuits */
   { MSEC_DELAY, 80 }, /* delay 80ms */
   { 0x0012, 0x008c }, /* set VREGOUT1 */
   { MSEC_DELAY, 10 }, /* delay 10ms */
   { 0x0013, 0x1900 }, /* set VCOMAC */
   { 0x0029, 0x0024 }, /* set VCOMH */
   { 0x002b, 0x000C }, /* set frame rate  80Hz */
   { MSEC_DELAY, 10 }, /* delay 10ms */
   { DISP_V1_RAM_ADDR_SET_H_REG, 0x0000 }, /* set Gram horizontal address */
   { DISP_V1_RAM_ADDR_SET_V_REG, 0x0000 }, /* set Gram vertical address */
   /* ============Gamma============ */
   { 0x0030, 0x0400 }, /* gamma 2.2 */
   { 0x0031, 0x0306 },
   { 0x0032, 0x0002 },
   { 0x0035, 0x0302 },
   { 0x0036, 0x0004 },
   { 0x0037, 0x0507 },
   { 0x0038, 0x0003 },
   { 0x0039, 0x0703 },
   { 0x003c, 0x0203 },
   { 0x003d, 0x0004 },
   /* ============================= */
   { DISP_V1_HORZ_RAM_ADDR_POS_1_REG, 0x0000 }, /* set RAM address */
   { DISP_V1_HORZ_RAM_ADDR_POS_2_REG, 0x00ef },
   { DISP_V1_VERT_RAM_ADDR_POS_1_REG, 0x0000 },
   { DISP_V1_VERT_RAM_ADDR_POS_2_REG, 0x013f },
   { MSEC_DELAY, 10 }, /* delay 10ms */
   { 0x0007, 0x0133 }, /* display on */
   { END_TABLE, 0x0 }, /* terminate table */
};

static struct init_table const disp_on_v2[] =
{
   /* exit sleep mode */
   { CMD_ONLY,                   DISP_V2_SLEEP_OUT_CMD },
   /* wait for circuits to stabilize */
   { MSEC_DELAY,                 120 },
   /* allow internal settings access */
   { DISP_V2_LEV2_ACCESS_CMD,    DISP_V2_GRANT_LEV_ACCESS },
   { DISP_V2_LEV2_ACCESS_CMD,    DISP_V2_GRANT_LEV_ACCESS },
   /* tearing effect ON */
   { CMD_ONLY,                   DISP_V2_ENABLE_TE_CMD },
   /* setup default scanning direction */
   { DISP_V2_MEM_ACC_CTL_CMD,    0x00C8 },
   /* 65k colors */
   { DISP_V2_IF_PIXEL_FMT_CMD,   0x0055 },
   /* setup internal display settings */
   { DISP_V2_DISP_SETUP1_CMD,    0x0028 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0093 },
   { DISP_V2_DISP_SETUP1_CMD,    0x007F },
   { DISP_V2_DISP_SETUP1_CMD,    0x0008 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0008 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0000 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0000 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0015 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0048 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0004 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0007 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0001 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0000 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0000 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0063 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0008 },
   { DISP_V2_DISP_SETUP1_CMD,    0x0008 },
   /* setup internal display settings */
   { DISP_V2_COLOR_SETUP_CMD,    0x0002 },
   { DISP_V2_COLOR_SETUP_CMD,    0x0000 },
   { DISP_V2_COLOR_SETUP_CMD,    0x0010 },
   { DISP_V2_COLOR_SETUP_CMD,    0x0000 },
   /* setup internal display settings */
   { DISP_V2_DISP_SETUP2_CMD,    0x0011 },
   { DISP_V2_DISP_SETUP2_CMD,    0x0000 },
   { DISP_V2_DISP_SETUP2_CMD,    0x0000 },
   /* setup internal power supply settings */
   { DISP_V2_PS_SETUP2_CMD,      0x0000 },
   { DISP_V2_PS_SETUP2_CMD,      0x0001 },
   { DISP_V2_PS_SETUP2_CMD,      0x0007 },
   { DISP_V2_PS_SETUP2_CMD,      0x0000 },
   { DISP_V2_PS_SETUP2_CMD,      0x0001 },
   { DISP_V2_PS_SETUP2_CMD,      0x000C },
   { DISP_V2_PS_SETUP2_CMD,      0x0003 },
   { DISP_V2_PS_SETUP2_CMD,      0x000C },
   { DISP_V2_PS_SETUP2_CMD,      0x0003 },
   /* setup internal power supply settings */
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0006 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0000 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0000 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0000 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0022 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0064 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0001 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0002 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x002A },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x006B },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0006 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x002A },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0000 },
   { DISP_V2_INTERNAL_VOLTS_CMD, 0x0006 },
   /* setup internal power supply */
   { DISP_V2_PS_SETUP3_CMD,      0x0000 },
   { DISP_V2_PS_SETUP3_CMD,      0x0057 },
   { DISP_V2_PS_SETUP3_CMD,      0x0043 },
   { DISP_V2_PS_SETUP3_CMD,      0x0000 },
   { DISP_V2_PS_SETUP3_CMD,      0x0000 },
   { DISP_V2_PS_SETUP3_CMD,      0x000A },
   { DISP_V2_PS_SETUP3_CMD,      0x0000 },
   { DISP_V2_PS_SETUP3_CMD,      0x0000 },
   { DISP_V2_PS_SETUP3_CMD,      0x000D },
   { DISP_V2_PS_SETUP3_CMD,      0x000D },
   /* select red gamma change */
   { DISP_V2_SEL_GAMMA_CMD,      0x0004 },
   /* setup red positive gamma values */
   { DISP_V2_POS_GAMMA_CMD,      0x0000 },
   { DISP_V2_POS_GAMMA_CMD,      0x000F },
   { DISP_V2_POS_GAMMA_CMD,      0x0010 },
   { DISP_V2_POS_GAMMA_CMD,      0x000F },
   { DISP_V2_POS_GAMMA_CMD,      0x001B },
   { DISP_V2_POS_GAMMA_CMD,      0x0025 },
   { DISP_V2_POS_GAMMA_CMD,      0x0010 },
   { DISP_V2_POS_GAMMA_CMD,      0x003F },
   { DISP_V2_POS_GAMMA_CMD,      0x0028 },
   { DISP_V2_POS_GAMMA_CMD,      0x0005 },
   { DISP_V2_POS_GAMMA_CMD,      0x0000 },
   /* setup red negative gamma values */
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   { DISP_V2_NEG_GAMMA_CMD,      0x000F },
   { DISP_V2_NEG_GAMMA_CMD,      0x0025 },
   { DISP_V2_NEG_GAMMA_CMD,      0x002C },
   { DISP_V2_NEG_GAMMA_CMD,      0x0022 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0017 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0005 },
   { DISP_V2_NEG_GAMMA_CMD,      0x003F },
   { DISP_V2_NEG_GAMMA_CMD,      0x002A },
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   /* select green gamma change */
   { DISP_V2_SEL_GAMMA_CMD,      0x0002 },
   /* setup green positive gamma values */
   { DISP_V2_POS_GAMMA_CMD,      0x0000 },
   { DISP_V2_POS_GAMMA_CMD,      0x000F },
   { DISP_V2_POS_GAMMA_CMD,      0x0010 },
   { DISP_V2_POS_GAMMA_CMD,      0x000F },
   { DISP_V2_POS_GAMMA_CMD,      0x001B },
   { DISP_V2_POS_GAMMA_CMD,      0x0025 },
   { DISP_V2_POS_GAMMA_CMD,      0x0010 },
   { DISP_V2_POS_GAMMA_CMD,      0x003F },
   { DISP_V2_POS_GAMMA_CMD,      0x0028 },
   { DISP_V2_POS_GAMMA_CMD,      0x0005 },
   { DISP_V2_POS_GAMMA_CMD,      0x0000 },
   /* setup green negative gamma values */
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   { DISP_V2_NEG_GAMMA_CMD,      0x000F },
   { DISP_V2_NEG_GAMMA_CMD,      0x0025 },
   { DISP_V2_NEG_GAMMA_CMD,      0x002C },
   { DISP_V2_NEG_GAMMA_CMD,      0x0022 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0017 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0005 },
   { DISP_V2_NEG_GAMMA_CMD,      0x003F },
   { DISP_V2_NEG_GAMMA_CMD,      0x002A },
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   /* select blue gamma change */
   { DISP_V2_SEL_GAMMA_CMD,      0x0001 },
   /* setup blue positive gamma values */
   { DISP_V2_POS_GAMMA_CMD,      0x0000 },
   { DISP_V2_POS_GAMMA_CMD,      0x000F },
   { DISP_V2_POS_GAMMA_CMD,      0x0010 },
   { DISP_V2_POS_GAMMA_CMD,      0x000F },
   { DISP_V2_POS_GAMMA_CMD,      0x001B },
   { DISP_V2_POS_GAMMA_CMD,      0x0025 },
   { DISP_V2_POS_GAMMA_CMD,      0x0010 },
   { DISP_V2_POS_GAMMA_CMD,      0x003F },
   { DISP_V2_POS_GAMMA_CMD,      0x0028 },
   { DISP_V2_POS_GAMMA_CMD,      0x0005 },
   { DISP_V2_POS_GAMMA_CMD,      0x0000 },
   /* setup blue negative gamma values */
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   { DISP_V2_NEG_GAMMA_CMD,      0x000F },
   { DISP_V2_NEG_GAMMA_CMD,      0x0025 },
   { DISP_V2_NEG_GAMMA_CMD,      0x002C },
   { DISP_V2_NEG_GAMMA_CMD,      0x0022 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0017 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0005 },
   { DISP_V2_NEG_GAMMA_CMD,      0x003F },
   { DISP_V2_NEG_GAMMA_CMD,      0x002A },
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   { DISP_V2_NEG_GAMMA_CMD,      0x0000 },
   /* lock internal settings access */
   { DISP_V2_LEV2_ACCESS_CMD,    DISP_V2_LOCK_LEV_ACCESS },
   { DISP_V2_LEV2_ACCESS_CMD,    DISP_V2_LOCK_LEV_ACCESS },
   /* turn display on */
   { CMD_ONLY,                   DISP_V2_DISPLAY_ON_CMD },
   { END_TABLE,                  0x0000 }, /* terminate table */
};

static struct init_table const *enter_standby = NULL;
static struct init_table const enter_standby_v1[] =
{
   { 0x0007, 0x0131 }, /* set D1=0, D0=1 */
   { MSEC_DELAY, 10 },
   { 0x0007, 0x0130 }, /* set D1=0, D0=0 */
   { MSEC_DELAY, 10 },
   { 0x0000, 0x0000 }, /* display off */
   { 0x0010, 0x0080 }, /* SAP, BT[3:0], APE, AP, DSTB, SLP */
   { 0x0011, 0x0000 }, /* DC1[2:0], DC0[2:0], VC[2:0] */
   { 0x0012, 0x0000 }, /* VREG1OUT voltage */
   { 0x0013, 0x0000 }, /* VDV[4:0] for VCOM amplitude */
   { MSEC_DELAY, 50 },
   { 0x0010, 0x0081 }, /* SAP, BT[3:0], APE, AP, DSTB, SLP */
   { END_TABLE, 0x0 }, /* terminate table */
};

static struct init_table const enter_standby_v2[] =
{
   { CMD_ONLY,                   DISP_V2_DISPLAY_OFF_CMD },
   { DISP_V2_LEV2_ACCESS_CMD,    DISP_V2_GRANT_LEV_ACCESS },
   { DISP_V2_LEV2_ACCESS_CMD,    DISP_V2_GRANT_LEV_ACCESS },
   { DISP_V2_DEEP_STANDBY_CMD,   0x0001 },
   { MSEC_DELAY,                 100 },
   { END_TABLE,                  0x0000 }, /* terminate table */
};

static struct init_table const *exit_standby = NULL;
static struct init_table const exit_standby_v1[] =
{
   { 0x0010, 0x0080 }, /* SAP, BT[3:0], APE, AP, DSTB, SLP */
   { MSEC_DELAY, 50 },
   { 0x0010, 0x1490 }, /* SAP, BT[3:0], APE, AP, DSTB, SLP */
   { 0x0011, 0x0667 }, /* DC1[2:0], DC0[2:0], VC[2:0] */
   { MSEC_DELAY, 80 },
   { 0x0012, 0x000A }, /* set VREGOUT1 */
   { MSEC_DELAY, 10 },
   { 0x0013, 0x1900 }, /* VDV[4:0] for VCOM amplitude */
   { MSEC_DELAY, 10 },
   { 0x0007, 0x0133 }, /* 262K color and display ON */
   { END_TABLE, 0x0 }, /* terminate table */
};

static struct init_table const exit_standby_v2[] =
{
   { USEC_DELAY,  1 },
   { CMD_ONLY,    DISP_V2_DISPLAY_ON_CMD },
   { USEC_DELAY,  1 },
   { CMD_ONLY,    DISP_V2_DISPLAY_ON_CMD },
   { USEC_DELAY,  1 },
   { CMD_ONLY,    DISP_V2_DISPLAY_ON_CMD },
   { USEC_DELAY,  1 },
   { CMD_ONLY,    DISP_V2_DISPLAY_ON_CMD },
   { USEC_DELAY,  1 },
   { CMD_ONLY,    DISP_V2_DISPLAY_ON_CMD },
   { MSEC_DELAY,  10 },
   { END_TABLE,   0x0000 }, /* terminate table */
};

static void smart_qvga_disp_write_reg(uint16 reg, uint16 data)
{
   outp16(DISP_CMD_PORT, reg);
   outp16(DISP_DATA_PORT, data);
}

static uint16 smart_qvga_disp_read_reg(uint16 reg)
{
   outp16(DISP_CMD_PORT, reg);
   return inp16(DISP_DATA_PORT);
}

static void smart_qvga_xmit_seq(struct init_table const *init_table)
{
   uint16_t reg = END_TABLE;

   SMRTQ_PRINTK("%s - start\n", __FUNCTION__);
   while(init_table && init_table->reg != END_TABLE)
   {
      if (init_table->reg == MSEC_DELAY)
      {
         mdelay(init_table->data);
      }
      else if (init_table->reg == USEC_DELAY)
      {
         udelay(init_table->data);
      }
      else if (init_table->reg == CMD_ONLY)
      {
         outp16(DISP_CMD_PORT, init_table->data);
      }
      else
      {
         if (init_table->reg != reg)
         {
            outp16(DISP_CMD_PORT, init_table->reg);
         }
         outp16(DISP_DATA_PORT, init_table->data);
      }
      reg = init_table->reg;
      init_table++;
      SMRTQ_PRINTK(".");
   }
   SMRTQ_PRINTK("\n%s - done\n", __FUNCTION__);
}

#ifdef TEMP_7XXX_CODE_COMPLETE
extern void *ebi2_lcd_cfg0;
static void smart_qvga_auto_tune_ebi2_timing(void)
{
   uint32 addr_cs_setup   = 0x1F;
   uint32 wr_active       = 0x1F;
   uint32 wr_cs_hold      = 0x1F;
   uint32 cs_wr_rd_setup  = 0x1F;
   uint32 rd_active       = 0x1F;
   uint32 rd_cs_hold      = 0x1F;
   #define ADDR_CS_SETUP   (addr_cs_setup << 25)
   #define WR_ACTIVE       (wr_active << 20)
   #define WR_CS_HOLD      (wr_cs_hold << 15)
   #define CS_WR_RD_SETUP  (cs_wr_rd_setup << 10)
   #define RD_ACTIVE       (rd_active << 5)
   #define RD_CS_HOLD      (rd_cs_hold << 0)
   #define WAIT_CYCLES     (ADDR_CS_SETUP | WR_ACTIVE | \
                           WR_CS_HOLD | CS_WR_RD_SETUP | \
                           RD_ACTIVE | RD_CS_HOLD)
   uint32 wait_cycles;

   while(addr_cs_setup)
   {
      wait_cycles = WAIT_CYCLES;
      outp32(ebi2_lcd_cfg0, wait_cycles);
      DISP_REG_WRITE(DISP_V1_HORZ_RAM_ADDR_POS_1_REG, addr_cs_setup);
      if (DISP_REG_READ(DISP_V1_HORZ_RAM_ADDR_POS_1_REG) != addr_cs_setup)
      {
         addr_cs_setup++;
         wait_cycles = WAIT_CYCLES;
         break;
      }
      addr_cs_setup--;
   }
   SMRTQ_PRINTK("%s: addr_cs_setup=0x%x wait_cycles=0x%x\n", __FUNCTION__,
      addr_cs_setup, wait_cycles);

   while(wr_active)
   {
      wait_cycles = WAIT_CYCLES;
      outp32(ebi2_lcd_cfg0, wait_cycles);
      DISP_REG_WRITE(DISP_V1_HORZ_RAM_ADDR_POS_1_REG, wr_active);
      if (DISP_REG_READ(DISP_V1_HORZ_RAM_ADDR_POS_1_REG) != wr_active)
      {
         wr_active++;
         wait_cycles = WAIT_CYCLES;
         break;
      }
      wr_active--;
   }
   SMRTQ_PRINTK("%s: wr_active=0x%x wait_cycles=0x%x\n", __FUNCTION__,
      wr_active, wait_cycles);

   while(wr_cs_hold)
   {
      wait_cycles = WAIT_CYCLES;
      outp32(ebi2_lcd_cfg0, wait_cycles);
      DISP_REG_WRITE(DISP_V1_HORZ_RAM_ADDR_POS_1_REG, wr_cs_hold);
      if (DISP_REG_READ(DISP_V1_HORZ_RAM_ADDR_POS_1_REG) != wr_cs_hold)
      {
         wr_cs_hold++;
         wait_cycles = WAIT_CYCLES;
         break;
      }
      wr_cs_hold--;
   }
   SMRTQ_PRINTK("%s: wr_cs_hold=0x%x wait_cycles=0x%x\n", __FUNCTION__,
      wr_cs_hold, wait_cycles);

   while(cs_wr_rd_setup)
   {
      wait_cycles = WAIT_CYCLES;
      outp32(ebi2_lcd_cfg0, wait_cycles);
      DISP_REG_WRITE(DISP_V1_HORZ_RAM_ADDR_POS_1_REG, cs_wr_rd_setup);
      if (DISP_REG_READ(DISP_V1_HORZ_RAM_ADDR_POS_1_REG) != cs_wr_rd_setup)
      {
         cs_wr_rd_setup++;
         wait_cycles = WAIT_CYCLES;
         break;
      }
      cs_wr_rd_setup--;
   }
   SMRTQ_PRINTK("%s: cs_wr_rd_setup=0x%x wait_cycles=0x%x\n", __FUNCTION__,
      cs_wr_rd_setup, wait_cycles);

   while(rd_active)
   {
      wait_cycles = WAIT_CYCLES;
      outp32(ebi2_lcd_cfg0, wait_cycles);
      DISP_REG_WRITE(DISP_V1_HORZ_RAM_ADDR_POS_1_REG, rd_active);
      if (DISP_REG_READ(DISP_V1_HORZ_RAM_ADDR_POS_1_REG) != rd_active)
      {
         rd_active++;
         wait_cycles = WAIT_CYCLES;
         break;
      }
      rd_active--;
   }
   SMRTQ_PRINTK("%s: rd_active=0x%x wait_cycles=0x%x\n", __FUNCTION__,
      rd_active, wait_cycles);

   while(rd_cs_hold)
   {
      wait_cycles = WAIT_CYCLES;
      outp32(ebi2_lcd_cfg0, wait_cycles);
      DISP_REG_WRITE(DISP_V1_HORZ_RAM_ADDR_POS_1_REG, rd_cs_hold);
      if (DISP_REG_READ(DISP_V1_HORZ_RAM_ADDR_POS_1_REG) != rd_cs_hold)
      {
         rd_cs_hold++;
         wait_cycles = WAIT_CYCLES;
         break;
      }
      rd_cs_hold--;
   }
   SMRTQ_PRINTK("%s: rd_cs_hold=0x%x wait_cycles=0x%x\n", __FUNCTION__,
      rd_cs_hold, wait_cycles);
   
}
#endif /* TEMP_7XXX_CODE_COMPLETE */

static void smart_qvga_disp_init(struct platform_device *pdev)
{
   uint32 data;

   if (disp_initialized)
      return;

   mfd = platform_get_drvdata(pdev);

   DISP_CMD_PORT = mfd->cmd_port;
   DISP_DATA_PORT = mfd->data_port;

   data = DISP_REG_READ(DISP_V1_ID_INFO_REG);
   if (data == DISP_V1)
   {
      printk("%s: detected CLI v1 (0x%x)\n", __FUNCTION__, data);
      disp_on = disp_on_v1;
      exit_standby = exit_standby_v1;
      enter_standby = enter_standby_v1;
   }
   else
   {
      printk("%s: detected CLI v2 (0x%x)\n", __FUNCTION__, data);

      outp16(DISP_CMD_PORT, DISP_V2_READ_ID1_CMD);
      data = inp16(DISP_DATA_PORT);
      data = inp16(DISP_DATA_PORT);
      printk("%s: Driver IC manufacturer (0x%x)\n", __FUNCTION__, data);

      outp16(DISP_CMD_PORT, DISP_V2_READ_ID2_CMD);
      data = inp16(DISP_DATA_PORT);
      data = inp16(DISP_DATA_PORT);
      printk("%s: LCD module/driver (0x%x)\n", __FUNCTION__, data);

      outp16(DISP_CMD_PORT, DISP_V2_READ_ID3_CMD);
      data = inp16(DISP_DATA_PORT);
      data = inp16(DISP_DATA_PORT);
      printk("%s: LCD module/driver version (0x%x)\n", __FUNCTION__, data);

      disp_on = disp_on_v2;
      exit_standby = exit_standby_v2;
      enter_standby = enter_standby_v2;
   }

   disp_initialized = TRUE;
}

static int smart_qvga_disp_off(struct platform_device *pdev)
{
   if (!disp_initialized)
      smart_qvga_disp_init(pdev);

   switch (display_pwr)
   {
   case DISPLAY_PWR_OFF:
   case DISPLAY_PWR_STANDBY:
      break;
   case DISPLAY_PWR_ON:
      SMRTQ_PRINTK("%s - xmit \"enter_standby\"\n", __FUNCTION__);
      smart_qvga_xmit_seq(enter_standby);
      display_pwr = DISPLAY_PWR_STANDBY;
      break;
   default:
      printk(KERN_ERR "%s - display_pwr=%u state invalid\n", __FUNCTION__,
         display_pwr);
      break;
   }

   return 0;
}

#if defined(CONFIG_KERNEL_MOTOROLA)
static void smart_qvga_disp_set_rect(int x, int y, int xres, int yres,
   char __iomem *screen_base)
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
static void smart_qvga_disp_set_rect(int x, int y, int xres, int yres)
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
{
   uint32 i = 0;
#if defined(CONFIG_KERNEL_MOTOROLA)
   uint16 *bits = (uint16 *)screen_base;
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
   uint16 *bits = 0;
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */

   if (!disp_initialized)
      return;

   if (bits)
   {
      x = 0;
      y = 0;
      xres = QVGA_WIDTH;
      yres = QVGA_HEIGHT;
   }

   if (disp_on == disp_on_v1)
   {
      struct init_table const set_rect[] =
      {
         { DISP_V1_HORZ_RAM_ADDR_POS_1_REG,  x },
         { DISP_V1_HORZ_RAM_ADDR_POS_2_REG,  (x + xres - 1) },
         { DISP_V1_VERT_RAM_ADDR_POS_1_REG,  y },
         { DISP_V1_VERT_RAM_ADDR_POS_2_REG,  (y + yres - 1) },
         { DISP_V1_RAM_ADDR_SET_H_REG,       x },
         { DISP_V1_RAM_ADDR_SET_V_REG,       y },
         { CMD_ONLY,                         DISP_V1_RAM_WRITE_REG },
         { END_TABLE,                        0x0 }, /* terminate table */
      };
      SMRTQ_PRINTK("%s - xmit \"set_rect\"\n", __FUNCTION__);
      smart_qvga_xmit_seq(set_rect);
   }
   else if (disp_on == disp_on_v2)
   {
      struct init_table const set_rect[] =
      {
         { DISP_V2_COL_ADDR_SET_CMD,   0x0000 },
         { DISP_V2_COL_ADDR_SET_CMD,   0x0000 },
         { DISP_V2_COL_ADDR_SET_CMD,   ((xres -1) & 0xFF00 >> 8) },
         { DISP_V2_COL_ADDR_SET_CMD,   ((xres -1) & 0x00FF) },
         { DISP_V2_PAGE_ADDR_SET_CMD,  0x0000 },
         { DISP_V2_PAGE_ADDR_SET_CMD,  0x0000 },
         { DISP_V2_PAGE_ADDR_SET_CMD,  ((xres -1) & 0xFF00 >> 8) },
         { DISP_V2_PAGE_ADDR_SET_CMD,  ((xres -1) & 0x00FF) },
         { CMD_ONLY,                   DISP_V2_RAM_WRITE_CMD },
         { END_TABLE,                  0x0 }, /* terminate table */
      };
      SMRTQ_PRINTK("%s - xmit \"set_rect\"\n", __FUNCTION__);
      smart_qvga_xmit_seq(set_rect);
   }

   if (bits)
   {
      int x;
      int y;
      
      bits += (mfd->fbi->var.yoffset * mfd->fbi->var.xres_virtual);
      bits += mfd->fbi->var.xoffset;
      for (y = 0; y < mfd->fbi->var.yres; y++)
      {
         for (x = 0; x < mfd->fbi->var.xres; x++)
         {
            outp16(DISP_DATA_PORT, *bits++);
         }
         bits += (mfd->fbi->var.xres_virtual - mfd->fbi->var.xres);
      }
   }
}

static void smart_qvga_disp_clear(int x, int y, int xres, int yres, uint16 c)
{
   uint32 i = (xres * yres) - 1;

#if defined(CONFIG_KERNEL_MOTOROLA)
   smart_qvga_disp_set_rect(x, y, xres, yres, NULL);
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
   smart_qvga_disp_set_rect(x, y, xres, yres);
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
   for (; i > 0; i--)
   {
      outp16(DISP_DATA_PORT, c);
   }
}

static int smart_qvga_disp_on(struct platform_device *pdev)
{
   if (!disp_initialized)
      smart_qvga_disp_init(pdev);

   switch (display_pwr)
   {
   case DISPLAY_PWR_OFF:
      SMRTQ_PRINTK("%s - xmit \"disp_on\"\n", __FUNCTION__);
      smart_qvga_xmit_seq(disp_on);
      display_pwr = DISPLAY_PWR_ON;
      break;
   case DISPLAY_PWR_STANDBY:
      SMRTQ_PRINTK("%s - xmit \"exit_standby\"\n", __FUNCTION__);
      smart_qvga_xmit_seq(exit_standby);
      if (disp_on == disp_on_v2)
      {
         SMRTQ_PRINTK("%s - xmit \"disp_on\"\n", __FUNCTION__);
         smart_qvga_xmit_seq(disp_on);
      }
      display_pwr = DISPLAY_PWR_ON;
      break;
   case DISPLAY_PWR_ON:
      break;
   default:
      printk(KERN_ERR "%s - display_pwr=%u state invalid\n", __FUNCTION__,
         display_pwr);
      break;
   }

   return 0;
}

static int __init smart_qvga_probe(struct platform_device *pdev)
{
   msm_fb_add_device(pdev);

   return 0;
}

static struct platform_driver this_driver = {
   .probe   = smart_qvga_probe,
   .driver  = {
      .name = EBI2_SMART_QVGA_NAME,
   },
};

static struct msm_fb_panel_data smart_qvga_panel_data = {
   .on = smart_qvga_disp_on,
   .off = smart_qvga_disp_off,
   .set_rect = smart_qvga_disp_set_rect,
};

static struct platform_device this_device = {
   .name = EBI2_SMART_QVGA_NAME,
   .id   = 0,
   .dev  = {
      .platform_data = &smart_qvga_panel_data,
   }
};

static int __init smart_qvga_init(void)
{
   int ret;
   struct msm_panel_info *pinfo;

   ret = platform_driver_register(&this_driver);
   if (!ret) {
      pinfo = &smart_qvga_panel_data.panel_info;
      pinfo->xres = QVGA_WIDTH;
      pinfo->yres = QVGA_HEIGHT;
      pinfo->type = EBI2_PANEL;
      pinfo->pdest = DISPLAY_2;
#if defined(CONFIG_KERNEL_MOTOROLA)
      pinfo->wait_cycle = EBI2_SMART_QVGA_TIMING;
#else /* defined(CONFIG_KERNEL_MOTOROLA) */
      pinfo->wait_cycle = 0x00808000;
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
      pinfo->bpp = 16;
      pinfo->fb_num = 2;
#if defined(CONFIG_KERNEL_MOTOROLA)
      pinfo->fb_reuse = FB_REUSE;
#endif /* defined(CONFIG_KERNEL_MOTOROLA) */
      pinfo->lcd.vsync_enable = TRUE;
      pinfo->lcd.refx100 = 6000;
      pinfo->lcd.v_back_porch = 16;
      pinfo->lcd.v_front_porch = 4;
      pinfo->lcd.v_pulse_width = 0;
      pinfo->lcd.hw_vsync_mode = FALSE;
      pinfo->lcd.vsync_notifier_period = 0;

      ret = platform_device_register(&this_device);
      if (ret)
         platform_driver_unregister(&this_driver);
   }

   return ret;
}

module_init(smart_qvga_init);


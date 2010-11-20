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

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#ifndef CONFIG_MACH_MOT
#include <linux/kernel.h>
#endif
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9p012.h"
#ifdef CONFIG_MACH_MOT
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <mach/vreg.h>
#include <linux/sched.h>
#include <asm/param.h>
#include <linux/pm_qos_params.h>
#endif

/*=============================================================
    SENSOR REGISTER DEFINES
==============================================================*/
#define MT9P012_REG_MODEL_ID         0x0000
#define MT9P012_MODEL_ID             0x2801
#define MT9P012_MODEL_ID_5131        0x2803
#define REG_GROUPED_PARAMETER_HOLD   0x0104
#define GROUPED_PARAMETER_HOLD       0x0100
#define GROUPED_PARAMETER_UPDATE     0x0000
#define REG_COARSE_INT_TIME          0x3012
#define REG_VT_PIX_CLK_DIV           0x0300
#define REG_VT_SYS_CLK_DIV           0x0302
#define REG_PRE_PLL_CLK_DIV          0x0304
#define REG_PLL_MULTIPLIER           0x0306
#define REG_OP_PIX_CLK_DIV           0x0308
#define REG_OP_SYS_CLK_DIV           0x030A
#define REG_SCALE_M                  0x0404
#define REG_FRAME_LENGTH_LINES       0x300A
#define REG_LINE_LENGTH_PCK          0x300C
#define REG_X_ADDR_START             0x3004
#define REG_Y_ADDR_START             0x3002
#define REG_X_ADDR_END               0x3008
#define REG_Y_ADDR_END               0x3006
#define REG_X_OUTPUT_SIZE            0x034C
#define REG_Y_OUTPUT_SIZE            0x034E
#define REG_FINE_INTEGRATION_TIME    0x3014
#define REG_ROW_SPEED                0x3016
#define REG_SKEW                     0x309E
#define MT9P012_REG_RESET_REGISTER   0x301A
#define MT9P012_RESET_REGISTER_PWON  0x10CC
#define MT9P012_RESET_REGISTER_PWOFF 0x10C8
#define REG_READ_MODE                0x3040
#define REG_GLOBAL_GAIN              0x305E
#define REG_TEST_PATTERN_MODE        0x3070

#define MT9P012_REV_7
#ifdef CONFIG_MACH_MOT
#define MT9P012_5131

#ifndef DIFF
#define DIFF(a,b) ( ((a) > (b)) ? ((a) - (b)) : ((b) - (a)) )
#endif

#define MOT_SUPPRESS_READ_ERROR_MSGS

#define CAMERA_CONFIG_ATTEMPTS 5
#endif

#ifdef CONFIG_MACH_MOT
enum mt9p012_test_mode_t {
#else
enum mt9p012_test_mode {
#endif
	TEST_OFF,
	TEST_1,
	TEST_2,
#ifdef CONFIG_MACH_MOT
	TEST_3,
	TEST_W1_12B = 256,
	TEST_W1_10B,
	TEST_W1_8B
#else
	TEST_3
#endif
};

#ifdef CONFIG_MACH_MOT
enum mt9p012_resolution_t {
#else
enum mt9p012_resolution {
#endif
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

#ifdef CONFIG_MACH_MOT
enum mt9p012_reg_update_t {
#else
enum mt9p012_reg_update {
#endif
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

#ifdef CONFIG_MACH_MOT
enum mt9p012_setting_t {
#else
enum mt9p012_setting {
#endif
	RES_PREVIEW,
	RES_CAPTURE
};

#ifndef CONFIG_MACH_MOT
/* actuator's Slave Address */
#define MT9P012_AF_I2C_ADDR   0x0A
#endif
/* AF Total steps parameters */
#ifdef CONFIG_MACH_MOT
#define MT9P012_STEPS_NEAR_TO_CLOSEST_INF  24
#define MT9P012_TOTAL_STEPS_NEAR_TO_FAR    24
#else
#define MT9P012_STEPS_NEAR_TO_CLOSEST_INF  20
#define MT9P012_TOTAL_STEPS_NEAR_TO_FAR    20
#endif

#define MT9P012_MU5M0_PREVIEW_DUMMY_PIXELS 0
#define MT9P012_MU5M0_PREVIEW_DUMMY_LINES  0

/* Time in milisecs for waiting for the sensor to reset.*/
#define MT9P012_RESET_DELAY_MSECS   66

/* for 20 fps preview */
#ifdef CONFIG_MACH_MOT
#ifdef NEW_MCLK_PCLK
  #define MT9P012_DEFAULT_CLOCK_RATE  48000000
#else
  #define MT9P012_DEFAULT_CLOCK_RATE  24000000
#endif
#define MT9P012_DEFAULT_MAX_FPS     26 /* ???? */

#define S_WAIT(n) \
	mdelay(n)

#else

#define MT9P012_DEFAULT_CLOCK_RATE  24000000
#define MT9P012_DEFAULT_MAX_FPS     26	/* ???? */
#endif

#ifdef CONFIG_MACH_MOT
struct mt9p012_work_t {
#else
struct mt9p012_work {
#endif
	struct work_struct work;
};
#ifdef CONFIG_MACH_MOT
static struct mt9p012_work_t *mt9p012_sensorw;
#else
static struct mt9p012_work *mt9p012_sensorw;
#endif
static struct i2c_client *mt9p012_client;

#ifdef CONFIG_MACH_MOT
struct mt9p012_ctrl_t {
	struct  msm_camera_sensor_info *sensordata;
	int sensormode;
#else
struct mt9p012_ctrl {
	const struct msm_camera_sensor_info *sensordata;
	int sensormode;
#endif
	uint32_t fps_divider;	/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */

	uint16_t curr_lens_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

#ifdef CONFIG_MACH_MOT
	enum mt9p012_resolution_t prev_res;
	enum mt9p012_resolution_t pict_res;
	enum mt9p012_resolution_t curr_res;
	enum mt9p012_test_mode_t  set_test;
#else
	enum mt9p012_resolution prev_res;
	enum mt9p012_resolution pict_res;
	enum mt9p012_resolution curr_res;
	enum mt9p012_test_mode set_test;
#endif
};

#ifdef CONFIG_MACH_MOT

// EEPROM DATA
#define EEPROM_BLOCK_SZ 4096
#define EEPROM_SIZE (16*1024)

typedef enum
{
    EEP_DATA_UNKNOWN,
    EEP_DATA_READ,
    EEP_DATA_VALID,
    EEP_DATA_INVALID
} eep_data_state;

static int8_t awb_decision = WB_FLORESCENT;
static module_data_t module_data;
static eep_data_state eep_state = EEP_DATA_UNKNOWN;
static uint8_t * pEpromImage = NULL;
static bool load_snapshot_lsc = false;

// AUTO FOCUS
static uint8_t bam_check_freq(uint8_t freq);
static int32_t bam_move_lens(int16_t new_pos, uint8_t time_out);

extern uint8_t disable_lens_move;

static char af_addr = 0;
static struct wake_lock mt9p012_wake_lock;

#define MT9P012_AF_ADDR_BAM1  0x05 // SEMCO 5MP Sensor with OLD AF module - Bit Bang.
#define MT9P012_AF_ADDR_BAM2  0x45 // SEMCO 5MP Sensor with new BAM module
#define MT9P012_AF_ADDR_VCM   0x0C  // VPT 5MP SensorMT9P012_AF_ADDR_VCM    0x0C  // VPT 5MP Sensor

#endif /* #ifdef CONFIG_MACH_MOT */

static uint16_t bam_macro, bam_infinite;
static uint16_t bam_step_lookup_table[MT9P012_TOTAL_STEPS_NEAR_TO_FAR + 1];
#ifdef CONFIG_MACH_MOT
static uint16_t mt9p012_current_lens_position;
static struct mt9p012_ctrl_t *mt9p012_ctrl;
#else
static struct mt9p012_ctrl *mt9p012_ctrl;
#endif
static DECLARE_WAIT_QUEUE_HEAD(mt9p012_wait_queue);
DEFINE_MUTEX(mt9p012_mut);

/*=============================================================*/

#ifdef CONFIG_MACH_MOT
static int mt9p012_i2c_rxdata(unsigned short saddr, int slength,
			      unsigned char *rxdata, int rxlength, bool polling)
{
  struct i2c_msg msgs[] = {
	{   .addr   = saddr,
		.flags = 0 | (polling ? I2C_M_NAK_LIKELY : 0),
		.len   = slength,
		.buf   = rxdata,
	},
	{   .addr   = saddr,
		.flags = I2C_M_RD | (polling ? I2C_M_NAK_LIKELY : 0),
		.len   = rxlength,
		.buf   = rxdata,
	},
    };
    int rc = 0;
    int attempt = 0;
    do {
#ifndef MOT_SUPPRESS_READ_ERROR_MSGS
        if (rc < 0)
            CDBG("mt9p012: mt9p012_i2c_rxdata: ATTEMPT %d FAILED: rc=%d\n", attempt, rc);
#endif
        attempt++;
	    rc = i2c_transfer(mt9p012_client->adapter, msgs, 2);
	} while (!polling && rc < 0 && attempt < CAMERA_CONFIG_ATTEMPTS);
    if (rc < 0) {
#ifndef MOT_SUPPRESS_READ_ERROR_MSGS
	    CDBG("mt9p012_i2c_rxdata failed after %d attempts!\n", attempt);
#endif
	    return -EIO;
    }
	return 0;
}
#else
static int mt9p012_i2c_rxdata(unsigned short saddr, int slength,
			      unsigned char *rxdata, int rxlength)
{
	struct i2c_msg msgs[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = slength,
			.buf = rxdata,
		},
		{
			.addr = saddr,
			.flags = I2C_M_RD,
			.len = rxlength,
			.buf = rxdata,
		},
	};
	if (i2c_transfer(mt9p012_client->adapter, msgs, 2) < 0) {
		CDBG("mt9p012_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}
#endif

#ifdef CONFIG_MACH_MOT
static int32_t mt9p012_i2c_do_read_b(unsigned short saddr, uint8_t raddr,
	uint8_t *rdata, bool polling)
{
    int32_t rc = 0;
    unsigned char buf[2];

    if (!rdata)
        return -EIO;

    buf[0] = raddr;

    rc = mt9p012_i2c_rxdata(saddr, 1, buf, 1, polling);
    if (rc < 0) {
        if (!polling)
            CDBG("%s: failed! saddr = 0x%x, raddr = %d\n",
                    __func__, saddr, raddr);

        return rc;
    }

    *rdata = buf[0];

    return rc;
}

static int32_t mt9p012_i2c_poll_b(unsigned short saddr, uint8_t raddr,
    uint8_t *rdata)
{
    return mt9p012_i2c_do_read_b(saddr, raddr, rdata, true);
}

static int32_t mt9p012_i2c_read_b(unsigned short saddr, uint8_t raddr,
	uint8_t *rdata)
{
    return mt9p012_i2c_do_read_b(saddr, raddr, rdata, false);
}

#else
static int32_t mt9p012_i2c_read_b(unsigned short saddr, unsigned char raddr,
				  unsigned short *rdata)
{
	int32_t rc = 0;
	if (!rdata)
		return -EIO;
	rc = mt9p012_i2c_rxdata(saddr, 1, &raddr, 1);
	if (rc < 0)
		return rc;
	*rdata = raddr;
	if (rc < 0)
		CDBG("mt9p012_i2c_read_b failed!\n");
	return rc;
}
#endif

static int32_t mt9p012_i2c_read_w(unsigned short saddr, unsigned short raddr,
				  unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);

#ifdef CONFIG_MACH_MOT
	rc = mt9p012_i2c_rxdata(saddr, 2, buf, 2, false);
#else
	rc = mt9p012_i2c_rxdata(saddr, 2, buf, 2);
#endif
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		CDBG("mt9p012_i2c_read_w failed!\n");

	return rc;
}

static int32_t mt9p012_i2c_txdata(unsigned short saddr, unsigned char *txdata,
				  int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = length,
		 .buf = txdata,
		 },
	};
#ifdef CONFIG_MACH_MOT
    int rc = 0;
    int attempt = 0;

    do {
        if (rc < 0)
            CDBG("mt9p012: mt9p012_i2c_txdata: ATTEMPT %d FAILED: rc=%d\n", attempt, rc);
        attempt++;

        rc = i2c_transfer(mt9p012_client->adapter, msg, 1);
	} while (rc < 0 && attempt < CAMERA_CONFIG_ATTEMPTS);

    if (rc < 0) {
        CDBG("mt9p012_i2c_txdata failed after %d attempts\n", attempt);
		return -EIO;
    }
#else
	if (i2c_transfer(mt9p012_client->adapter, msg, 1) < 0) {
		CDBG("mt9p012_i2c_txdata failed\n");
		return -EIO;
	}
#endif
	return 0;
}

static int32_t mt9p012_i2c_write_b(unsigned short saddr, unsigned short baddr,
				   unsigned short bdata)
{
	int32_t rc = -EIO;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));
	buf[0] = baddr;
	buf[1] = bdata;
	rc = mt9p012_i2c_txdata(saddr, buf, 2);

	if (rc < 0)
		CDBG("i2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!\n",
		     saddr, baddr, bdata);

	return rc;
}

static int32_t mt9p012_i2c_write_w(unsigned short saddr, unsigned short waddr,
				   unsigned short wdata)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9p012_i2c_txdata(saddr, buf, 4);

	if (rc < 0)
		CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
		     waddr, wdata);

	return rc;
}

static int32_t mt9p012_i2c_write_w_table(struct mt9p012_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num; i++) {
		rc = mt9p012_i2c_write_w(mt9p012_client->addr,
					 reg_conf_tbl->waddr,
					 reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}

	return rc;
}

#ifdef CONFIG_MACH_MOT
static bool axi_qos_requested = false;

static int request_axi_qos(void)
{
    CVBS("%s\n",__func__);
    if (! axi_qos_requested) {
        if ( pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ, "msm_camera", 160000 ) < 0 ) {
            printk(KERN_ERR "unable to request AXI bus QOS\n");
            return -1;
        }
        else {
            CVBS("%s: request successful\n", __func__);
            axi_qos_requested = true;
            mdelay(5);
        }
    }
    return 0;
}

static int release_axi_qos(void)
{
    CVBS("%s\n",__func__);
    if ( axi_qos_requested ) {
        pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ, "msm_camera");
        CVBS("%s: release successful\n", __func__);
        axi_qos_requested = false;
        mdelay(5);
    }
    return 0;
}
#endif

#ifdef CONFIG_MACH_MOT
static int32_t mt9p012_test(enum mt9p012_test_mode_t mo)
#else
static int32_t mt9p012_test(enum mt9p012_test_mode mo)
#endif
{
	int32_t rc = 0;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	if (mo == TEST_OFF)
		return 0;
	else {
		rc = mt9p012_i2c_write_w_table(mt9p012_regs.ttbl,
					       mt9p012_regs.ttbl_size);
		if (rc < 0)
			return rc;

		rc = mt9p012_i2c_write_w(mt9p012_client->addr,
					 REG_TEST_PATTERN_MODE, (uint16_t) mo);
		if (rc < 0)
			return rc;
	}

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9p012_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;

	CDBG("%s: entered. enable = %d\n", __func__, is_enable);

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr, 0x3780,
				 ((uint16_t) is_enable) << 15);
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE);

	CDBG("%s: exiting. rc = %d\n", __func__, rc);
	return rc;
}

static int32_t mt9p012_set_lc(void)
{
	int32_t rc;
#ifdef CONFIG_MACH_MOT
    uint32_t index, i;

    if (eep_state == EEP_DATA_VALID)
    {
        rc = -EFAULT;

        CDBG("mt9p012 lsc_count = %d\n", module_data.lsc_count);

	if (module_data.lsc_count == 1)
	{
	  index = 0;
	}
        else if (load_snapshot_lsc == true)
        {
          index = awb_decision*LSC_DATA_ENTRIES;
          load_snapshot_lsc = false;
        }
	else
        {
	  index = LSC_DATA_ENTRIES;
	}

        CDBG("Starting LSC with AWB %d\n", index/LSC_DATA_ENTRIES);

        for (i = 0; i < LSC_DATA_ENTRIES; i++) {
            rc = mt9p012_i2c_write_w(mt9p012_client->addr,
                    module_data.lsc[i+index].lscAddr,
                    module_data.lsc[i+index].lscVal);
            if (rc < 0)
            {
                CDBG ("LSC data application failed\n");
                break;
            }
        }

        if (rc >= 0)
            rc = mt9p012_i2c_write_w (mt9p012_client->addr, 0x3780, 0x8000);

        CDBG ("LSC data applied to sensor successfully\n");
    }
    else
    {
        rc = mt9p012_i2c_write_w_table(mt9p012_regs.lctbl,
                mt9p012_regs.lctbl_size);
    }
#else /* #ifndef CONFIG_MACH_MOT */

	rc = mt9p012_i2c_write_w_table(mt9p012_regs.lctbl,
				       mt9p012_regs.lctbl_size);
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_w_table(mt9p012_regs.rftbl,
				       mt9p012_regs.rftbl_size);

#endif /* #ifdef CONFIG_MACH_MOT */
	return rc;
}

static void mt9p012_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;	/*Q10 */
#ifdef CONFIG_MACH_MOT
    uint32_t d1, d2;

	d1 = (uint32_t) ((mt9p012_regs.reg_pat[RES_PREVIEW].frame_length_lines *
      0x00000400) / mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines);

	d2 = (uint32_t) ((mt9p012_regs.reg_pat[RES_PREVIEW].line_length_pck *
      0x00000400) / mt9p012_regs.reg_pat[RES_CAPTURE].line_length_pck);

    divider = (uint32_t)(d1 * d2) / 0x00000400;

    /* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t) (fps * divider / 0x00000400);

#else /* #ifndef CONFIG_MACH_MOT */

	uint32_t pclk_mult;	/*Q10 */

	if (mt9p012_ctrl->prev_res == QTR_SIZE) {
		divider = (uint32_t)
		    (((mt9p012_regs.reg_pat[RES_PREVIEW].frame_length_lines *
		       mt9p012_regs.reg_pat[RES_PREVIEW].line_length_pck) *
		      0x00000400) /
		     (mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines *
		      mt9p012_regs.reg_pat[RES_CAPTURE].line_length_pck));

		pclk_mult =
		    (uint32_t) ((mt9p012_regs.reg_pat[RES_CAPTURE].
				 pll_multiplier * 0x00000400) /
				(mt9p012_regs.reg_pat[RES_PREVIEW].
				 pll_multiplier));
	} else {
		/* full size resolution used for preview. */
		divider = 0x00000400;	/*1.0 */
		pclk_mult = 0x00000400;	/*1.0 */
	}

	/* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t) (fps * divider * pclk_mult / 0x00000400 /
			    0x00000400);
#endif /* #ifdef CONFIG_MACH_MOT */
}

static uint16_t mt9p012_get_prev_lines_pf(void)
{
	if (mt9p012_ctrl->prev_res == QTR_SIZE)
		return mt9p012_regs.reg_pat[RES_PREVIEW].frame_length_lines;
	else
		return mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9p012_get_prev_pixels_pl(void)
{
	if (mt9p012_ctrl->prev_res == QTR_SIZE)
		return mt9p012_regs.reg_pat[RES_PREVIEW].line_length_pck;
	else
		return mt9p012_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint16_t mt9p012_get_pict_lines_pf(void)
{
	return mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9p012_get_pict_pixels_pl(void)
{
	return mt9p012_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint32_t mt9p012_get_pict_max_exp_lc(void)
{
	uint16_t snapshot_lines_per_frame;

	if (mt9p012_ctrl->pict_res == QTR_SIZE)
		snapshot_lines_per_frame =
		    mt9p012_regs.reg_pat[RES_PREVIEW].frame_length_lines - 1;
	else
		snapshot_lines_per_frame =
		    mt9p012_regs.reg_pat[RES_CAPTURE].frame_length_lines - 1;

	return snapshot_lines_per_frame * 24;
}

static int32_t mt9p012_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q10 format */
	int32_t rc = 0;
#ifndef CONFIG_MACH_MOT
	enum mt9p012_setting setting;
#endif

	mt9p012_ctrl->fps_divider = fps->fps_div;
	mt9p012_ctrl->pict_fps_divider = fps->pict_fps_div;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return -EBUSY;

#ifdef CONFIG_MACH_MOT
	rc =
		mt9p012_i2c_write_w(mt9p012_client->addr,
			REG_LINE_LENGTH_PCK,
			(mt9p012_regs.reg_pat[RES_PREVIEW].line_length_pck *
			fps->f_mult / 0x00000400));
#else
	if (mt9p012_ctrl->sensormode == SENSOR_PREVIEW_MODE)
		setting = RES_PREVIEW;
	else
		setting = RES_CAPTURE;
	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
			REG_FRAME_LENGTH_LINES,
			(mt9p012_regs.reg_pat[setting].frame_length_lines *
			fps->fps_div / 0x00000400));
#endif
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE);

	return rc;
}

#ifdef CONFIG_MACH_MOT
static int32_t mt9p012_write_exp_gain(
        uint16_t gain, uint32_t line, int8_t is_outdoor)
#else
static int32_t mt9p012_write_exp_gain(uint16_t gain, uint32_t line)
#endif
{
	uint16_t max_legal_gain = 0x01FF;
	uint32_t line_length_ratio = 0x00000400;
#ifdef CONFIG_MACH_MOT
	enum mt9p012_setting_t setting;
#else
	enum mt9p012_setting setting;
#endif
	int32_t rc = 0;
#ifdef CONFIG_MACH_MOT
	int8_t multiplier = 1;
	CVBS("Line:%d mt9p012_write_exp_gain line=%d sensormode=%d\n",
		__LINE__, line, mt9p012_ctrl->sensormode);
#else
	CDBG("Line:%d mt9p012_write_exp_gain \n", __LINE__);
#endif

	if (mt9p012_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		mt9p012_ctrl->my_reg_gain = gain;
		mt9p012_ctrl->my_reg_line_count = (uint16_t) line;
	}

#ifdef CONFIG_MACH_MOT
	if (gain > max_legal_gain)
		gain = max_legal_gain;
#else
	if (gain > max_legal_gain) {
		CDBG("Max legal gain Line:%d \n", __LINE__);
		gain = max_legal_gain;
	}
#endif

	/* Verify no overflow */
#ifdef CONFIG_MACH_MOT
	if ((mt9p012_ctrl->sensormode != SENSOR_SNAPSHOT_MODE)&&
	    (mt9p012_ctrl->sensormode != SENSOR_RAW_SNAPSHOT_MODE))
 {
		line = (uint32_t)(line * mt9p012_ctrl->fps_divider /
			0x00000400);
		setting = RES_PREVIEW;
        multiplier = 1;
    } else {
        line = (uint32_t)(line * mt9p012_ctrl->pict_fps_divider /
            0x00000400);
        setting = RES_CAPTURE;
        multiplier = 1;
        if (gain > max_legal_gain) {
			line = line >> 1;/*  divide by 2 */
            multiplier = 2;
        }
    }
#else
	if (mt9p012_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		line = (uint32_t) (line * mt9p012_ctrl->fps_divider /
				   0x00000400);
		setting = RES_PREVIEW;
	} else {
		line = (uint32_t) (line * mt9p012_ctrl->pict_fps_divider /
				   0x00000400);
		setting = RES_CAPTURE;
	}
#endif

	/* Set digital gain to 1 */
#ifdef MT9P012_REV_7
	gain |= 0x1000;
#else
	gain |= 0x0200;
#endif

	if ((mt9p012_regs.reg_pat[setting].frame_length_lines - 1) < line) {
		line_length_ratio = (uint32_t) (line * 0x00000400) /
		    (mt9p012_regs.reg_pat[setting].frame_length_lines - 1);
	} else
		line_length_ratio = 0x00000400;

#ifdef CONFIG_MACH_MOT
	rc =
		mt9p012_i2c_write_w(mt9p012_client->addr,
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
#else
	rc = mt9p012_i2c_write_w(mt9p012_client->addr, REG_GLOBAL_GAIN, gain);
#endif
	if (rc < 0) {
		CDBG("mt9p012_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}
#ifdef CONFIG_MACH_MOT
    if (mt9p012_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
        if(is_outdoor == 1) {
            CVBS("%s:%d, outdoor\n", __func__, __LINE__);
            rc = mt9p012_i2c_write_w(mt9p012_client->addr,
                    0x316c, 0xA4F0);
            if (rc < 0) {
                CDBG("%s:failed... Line:%d \n",
                        __func__, __LINE__);
                return rc;
            }
        } else if (is_outdoor == 0){
            CVBS("%s:%d, indoor\n", __func__, __LINE__);
            rc = mt9p012_i2c_write_w(mt9p012_client->addr,
                    0x316c, 0x4410);
            if (rc < 0) {
                CDBG("%s:failed... Line:%d \n",
                        __func__, __LINE__);
                return rc;
            }
        }
    }
    if (mt9p012_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
        CVBS("%s: value = %d\n", __func__,
                (mt9p012_regs.reg_pat[RES_PREVIEW]. \
                 line_length_pck * line_length_ratio / 0x00000400));
        rc = mt9p012_i2c_write_w(mt9p012_client->addr,
                REG_LINE_LENGTH_PCK,
                (uint16_t) (mt9p012_regs.reg_pat[RES_PREVIEW].\
                    line_length_pck * line_length_ratio / 0x00000400));
        if (rc < 0) {
            CDBG("%s:failed... Line:%d \n", __func__, __LINE__);
            return rc;
        }
    } else {
        CVBS("snapshot gain=%d\n",gain);
        if(gain > 0x10C0) {
            rc = mt9p012_i2c_write_w(mt9p012_client->addr,
                    REG_LINE_LENGTH_PCK,
                    (uint16_t)mt9p012_regs.reg_pat[RES_CAPTURE]. \
                    line_length_pck*2);
            CDBG("line_length_pck=%d\n",
                    (mt9p012_regs.reg_pat[RES_CAPTURE].line_length_pck*2));
            if (rc < 0) {
                CDBG("%s:failed,Line:%d\n", __func__, __LINE__);
                return rc;
            }
        }
    }
	rc =
		mt9p012_i2c_write_w(mt9p012_client->addr,
			REG_GLOBAL_GAIN, gain);
#else

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 REG_COARSE_INT_TIME, line);
#endif
	if (rc < 0) {
		CDBG("mt9p012_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

#ifdef CONFIG_MACH_MOT
	rc =
		mt9p012_i2c_write_w(mt9p012_client->addr,
			REG_COARSE_INT_TIME,
			(uint16_t)((uint32_t) line * 0x00000400 /
			line_length_ratio));
	if (rc < 0) {
		CDBG("mt9p012_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}
	rc =
		mt9p012_i2c_write_w(mt9p012_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		CDBG("mt9p012_i2c_write_w failed... Line:%d \n", __LINE__);
	CVBS("mt9p012_write_exp_gain: gain = %d, line = %d\n", gain, line);
#else
	CDBG("mt9p012_write_exp_gain: gain = %d, line = %d\n", gain, line);
#endif

	return rc;
}

static int32_t mt9p012_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

#ifdef CONFIG_MACH_MOT
	CVBS("Line:%d mt9p012_set_pict_exp_gain \n", __LINE__);
#else
	CDBG("Line:%d mt9p012_set_pict_exp_gain \n", __LINE__);
#endif

#ifdef CONFIG_MACH_MOT
	rc =
		mt9p012_write_exp_gain(gain, line, -1);
#else
	rc = mt9p012_write_exp_gain(gain, line);
#endif
	if (rc < 0) {
		CDBG("Line:%d mt9p012_set_pict_exp_gain failed... \n",
		     __LINE__);
		return rc;
	}

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER, 0x10CC | 0x0002);
	if (rc < 0) {
		CDBG("mt9p012_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

#ifdef CONFIG_MACH_MOT
	S_WAIT(5);
#else
	mdelay(5);
#endif

	return rc;
}

#ifdef CONFIG_MACH_MOT
static int32_t mt9p012_setting(enum mt9p012_reg_update_t rupdate,
	enum mt9p012_setting_t rt)
#else
static int32_t mt9p012_setting(enum mt9p012_reg_update rupdate,
			       enum mt9p012_setting rt)
#endif
{
	int32_t rc = 0;
    uint16_t readmode;

	switch (rupdate) {
	case UPDATE_PERIODIC: {
		if (rt == RES_PREVIEW || rt == RES_CAPTURE) {

			struct mt9p012_i2c_reg_conf ppc_tbl[] = {
				{REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD},
				{REG_ROW_SPEED,
				 mt9p012_regs.reg_pat[rt].row_speed},
				{REG_X_ADDR_START,
				 mt9p012_regs.reg_pat[rt].x_addr_start},
				{REG_X_ADDR_END,
				 mt9p012_regs.reg_pat[rt].x_addr_end},
				{REG_Y_ADDR_START,
				 mt9p012_regs.reg_pat[rt].y_addr_start},
				{REG_Y_ADDR_END,
				 mt9p012_regs.reg_pat[rt].y_addr_end},
                {REG_READ_MODE, readmode =  (!(af_addr == MT9P012_AF_ADDR_VCM))?
                                               mt9p012_regs.reg_pat[rt].read_mode:
                                               (mt9p012_regs.reg_pat[rt].read_mode & 0xFFF) |0xC000 },
				{REG_SCALE_M, mt9p012_regs.reg_pat[rt].scale_m},
				{REG_X_OUTPUT_SIZE,
				 mt9p012_regs.reg_pat[rt].x_output_size},
				{REG_Y_OUTPUT_SIZE,
				 mt9p012_regs.reg_pat[rt].y_output_size},

				{REG_LINE_LENGTH_PCK,
				 mt9p012_regs.reg_pat[rt].line_length_pck},
				{REG_FRAME_LENGTH_LINES,
				 (mt9p012_regs.reg_pat[rt].frame_length_lines *
				  mt9p012_ctrl->fps_divider / 0x00000400)},
				{REG_COARSE_INT_TIME,
				 mt9p012_regs.reg_pat[rt].coarse_int_time},
				{REG_FINE_INTEGRATION_TIME,
				 mt9p012_regs.reg_pat[rt].fine_int_time},
		        {REG_SKEW, mt9p012_regs.reg_pat[rt].skew},
				{REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE},
			};

            CVBS("%s: In mode : %d, new readmode for REG 3040 is %d \n", __func__, rt, readmode);
			rc = mt9p012_i2c_write_w_table(&ppc_tbl[0],
						       ARRAY_SIZE(ppc_tbl));
			if (rc < 0)
				return rc;

			rc = mt9p012_test(mt9p012_ctrl->set_test);
			if (rc < 0)
				return rc;

			rc = mt9p012_i2c_write_w(mt9p012_client->addr,
						 MT9P012_REG_RESET_REGISTER,
						 MT9P012_RESET_REGISTER_PWON |
						 0x0002);
			if (rc < 0)
				return rc;

#ifdef CONFIG_MACH_MOT
			S_WAIT(15);
#else
			mdelay(5);
#endif
			return rc;
		}
	}
	break;		/* UPDATE_PERIODIC */

#ifdef CONFIG_MACH_MOT
	case REG_INIT: {
	    if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
		struct mt9p012_i2c_reg_conf ipc_tbl1[] = {
		{MT9P012_REG_RESET_REGISTER, MT9P012_RESET_REGISTER_PWOFF},
		{REG_VT_PIX_CLK_DIV, mt9p012_regs.reg_pat[rt].vt_pix_clk_div},
		{REG_VT_SYS_CLK_DIV, mt9p012_regs.reg_pat[rt].vt_sys_clk_div},
		{REG_PRE_PLL_CLK_DIV, mt9p012_regs.reg_pat[rt].pre_pll_clk_div},
		{REG_PLL_MULTIPLIER, mt9p012_regs.reg_pat[rt].pll_multiplier},
		{REG_OP_PIX_CLK_DIV, mt9p012_regs.reg_pat[rt].op_pix_clk_div},
		{REG_OP_SYS_CLK_DIV, mt9p012_regs.reg_pat[rt].op_sys_clk_div},
#ifndef MT9P013
#ifdef MT9P012_REV_7
		{0x30B0, 0x0001},
		/* Jeff: {0x308E, 0xE060},
		{0x3092, 0x0A52}, */
		{0x3094, 0x4656},
		/* Jeff: {0x3096, 0x5652},
		{0x30CA, 0x8006},
		{0x312A, 0xDD02},
		{0x312C, 0x00E4},
		{0x3170, 0x299A}, */
#endif
		{0x0204, 0x0010},
		{0x0206, 0x0010},
		{0x0208, 0x0010},
		{0x020A, 0x0010},
		{0x020C, 0x0010},
#endif
		{0x316C, 0xA4F0},

#ifdef MT9P012_5131
		{0x3088, 0x6FFF}, //0x6FF6
		{0x3086, 0x2468},
#endif
		{MT9P012_REG_RESET_REGISTER, MT9P012_RESET_REGISTER_PWON},
		};

		struct mt9p012_i2c_reg_conf ipc_tbl3[] = {
		{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD},
		/* Set preview or snapshot mode */
		{REG_ROW_SPEED, mt9p012_regs.reg_pat[rt].row_speed},
		{REG_X_ADDR_START, mt9p012_regs.reg_pat[rt].x_addr_start},
		{REG_X_ADDR_END, mt9p012_regs.reg_pat[rt].x_addr_end},
		{REG_Y_ADDR_START, mt9p012_regs.reg_pat[rt].y_addr_start},
		{REG_Y_ADDR_END, mt9p012_regs.reg_pat[rt].y_addr_end},
        {REG_READ_MODE, readmode =  (!(af_addr == MT9P012_AF_ADDR_VCM))?
                                       mt9p012_regs.reg_pat[rt].read_mode:
                                       (mt9p012_regs.reg_pat[rt].read_mode & 0xFFF) |0xC000 },
		{REG_SCALE_M, mt9p012_regs.reg_pat[rt].scale_m},
		{REG_X_OUTPUT_SIZE, mt9p012_regs.reg_pat[rt].x_output_size},
		{REG_Y_OUTPUT_SIZE, mt9p012_regs.reg_pat[rt].y_output_size},
		{REG_LINE_LENGTH_PCK, mt9p012_regs.reg_pat[rt].line_length_pck},
		{REG_FRAME_LENGTH_LINES,
			mt9p012_regs.reg_pat[rt].frame_length_lines},
		{REG_COARSE_INT_TIME, mt9p012_regs.reg_pat[rt].coarse_int_time},
		{REG_FINE_INTEGRATION_TIME,
		  mt9p012_regs.reg_pat[rt].fine_int_time},
		{REG_SKEW, mt9p012_regs.reg_pat[rt].skew},
		{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE},
		};

		/* reset fps_divider */
		mt9p012_ctrl->fps_divider = 1 * 0x0400;

        CVBS("%s : In mode : %d, new readmode for REG 3040 is %d \n", __func__, rt, readmode);
		rc = mt9p012_i2c_write_w_table(&ipc_tbl1[0],
			ARRAY_SIZE(ipc_tbl1));
		if (rc < 0)
			return rc;
		S_WAIT(5);

		rc = mt9p012_i2c_write_w_table(&ipc_tbl3[0],
			ARRAY_SIZE(ipc_tbl3));
		if (rc < 0)
			return rc;

		rc = mt9p012_i2c_write_w(mt9p012_client->addr,
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		rc = mt9p012_set_lc();
		if (rc < 0)
			return rc;

		rc = mt9p012_i2c_write_w(mt9p012_client->addr,
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);

		if (rc < 0)
			return rc;
		}
	}
	break; /* case REG_INIT: */

#else /* #ifndef CONFIG_MACH_MOT */

	case REG_INIT:
		if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
			struct mt9p012_i2c_reg_conf ipc_tbl1[] = {
				{MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWOFF},
				{REG_VT_PIX_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].vt_pix_clk_div},
				{REG_VT_SYS_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].vt_sys_clk_div},
				{REG_PRE_PLL_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].pre_pll_clk_div},
				{REG_PLL_MULTIPLIER,
				 mt9p012_regs.reg_pat[rt].pll_multiplier},
				{REG_OP_PIX_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].op_pix_clk_div},
				{REG_OP_SYS_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].op_sys_clk_div},
#ifdef MT9P012_REV_7
				{0x30B0, 0x0001},
				{0x308E, 0xE060},
				{0x3092, 0x0A52},
				{0x3094, 0x4656},
				{0x3096, 0x5652},
				{0x30CA, 0x8006},
				{0x312A, 0xDD02},
				{0x312C, 0x00E4},
				{0x3170, 0x299A},
#endif
				/* optimized settings for noise */
				{0x3088, 0x6FF6},
				{0x3154, 0x0282},
				{0x3156, 0x0381},
				{0x3162, 0x04CE},
				{0x0204, 0x0010},
				{0x0206, 0x0010},
				{0x0208, 0x0010},
				{0x020A, 0x0010},
				{0x020C, 0x0010},
				{MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWON},
			};

			struct mt9p012_i2c_reg_conf ipc_tbl2[] = {
				{MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWOFF},
				{REG_VT_PIX_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].vt_pix_clk_div},
				{REG_VT_SYS_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].vt_sys_clk_div},
				{REG_PRE_PLL_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].pre_pll_clk_div},
				{REG_PLL_MULTIPLIER,
				 mt9p012_regs.reg_pat[rt].pll_multiplier},
				{REG_OP_PIX_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].op_pix_clk_div},
				{REG_OP_SYS_CLK_DIV,
				 mt9p012_regs.reg_pat[rt].op_sys_clk_div},
#ifdef MT9P012_REV_7
				{0x30B0, 0x0001},
				{0x308E, 0xE060},
				{0x3092, 0x0A52},
				{0x3094, 0x4656},
				{0x3096, 0x5652},
				{0x30CA, 0x8006},
				{0x312A, 0xDD02},
				{0x312C, 0x00E4},
				{0x3170, 0x299A},
#endif
				/* optimized settings for noise */
				{0x3088, 0x6FF6},
				{0x3154, 0x0282},
				{0x3156, 0x0381},
				{0x3162, 0x04CE},
				{0x0204, 0x0010},
				{0x0206, 0x0010},
				{0x0208, 0x0010},
				{0x020A, 0x0010},
				{0x020C, 0x0010},
				{MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWON},
			};

			struct mt9p012_i2c_reg_conf ipc_tbl3[] = {
				{REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_HOLD},
				/* Set preview or snapshot mode */
				{REG_ROW_SPEED,
				 mt9p012_regs.reg_pat[rt].row_speed},
				{REG_X_ADDR_START,
				 mt9p012_regs.reg_pat[rt].x_addr_start},
				{REG_X_ADDR_END,
				 mt9p012_regs.reg_pat[rt].x_addr_end},
				{REG_Y_ADDR_START,
				 mt9p012_regs.reg_pat[rt].y_addr_start},
				{REG_Y_ADDR_END,
				 mt9p012_regs.reg_pat[rt].y_addr_end},
				{REG_READ_MODE,
				 mt9p012_regs.reg_pat[rt].read_mode},
				{REG_SCALE_M, mt9p012_regs.reg_pat[rt].scale_m},
				{REG_X_OUTPUT_SIZE,
				 mt9p012_regs.reg_pat[rt].x_output_size},
				{REG_Y_OUTPUT_SIZE,
				 mt9p012_regs.reg_pat[rt].y_output_size},
				{REG_LINE_LENGTH_PCK,
				 mt9p012_regs.reg_pat[rt].line_length_pck},
				{REG_FRAME_LENGTH_LINES,
				 mt9p012_regs.reg_pat[rt].frame_length_lines},
				{REG_COARSE_INT_TIME,
				 mt9p012_regs.reg_pat[rt].coarse_int_time},
				{REG_FINE_INTEGRATION_TIME,
				 mt9p012_regs.reg_pat[rt].fine_int_time},
				{REG_GROUPED_PARAMETER_HOLD,
				 GROUPED_PARAMETER_UPDATE},
			};

			/* reset fps_divider */
			mt9p012_ctrl->fps_divider = 1 * 0x0400;

			rc = mt9p012_i2c_write_w_table(&ipc_tbl1[0],
						       ARRAY_SIZE(ipc_tbl1));
			if (rc < 0)
				return rc;

			rc = mt9p012_i2c_write_w_table(&ipc_tbl2[0],
						       ARRAY_SIZE(ipc_tbl2));
			if (rc < 0)
				return rc;

			mdelay(5);

			rc = mt9p012_i2c_write_w_table(&ipc_tbl3[0],
						       ARRAY_SIZE(ipc_tbl3));
			if (rc < 0)
				return rc;

			/* load lens shading */
			rc = mt9p012_i2c_write_w(mt9p012_client->addr,
						 REG_GROUPED_PARAMETER_HOLD,
						 GROUPED_PARAMETER_HOLD);
			if (rc < 0)
				return rc;

			rc = mt9p012_set_lc();
			if (rc < 0)
				return rc;

			rc = mt9p012_i2c_write_w(mt9p012_client->addr,
						 REG_GROUPED_PARAMETER_HOLD,
						 GROUPED_PARAMETER_UPDATE);

			if (rc < 0)
				return rc;
		}
		break;		/* case REG_INIT: */
#endif

	default:
#ifdef CONFIG_MACH_MOT
		rc = -EFAULT;
#else
		rc = -EINVAL;
#endif
		break;
	}			/* switch (rupdate) */

	return rc;
}

static int32_t mt9p012_video_config(int mode, int res)
{
	int32_t rc;

	switch (res) {
	case QTR_SIZE:
#ifdef CONFIG_MACH_MOT
		rc = mt9p012_setting(REG_INIT, RES_PREVIEW);
		if (rc < 0)
			return rc;

#endif
		rc = mt9p012_setting(UPDATE_PERIODIC, RES_PREVIEW);
		if (rc < 0)
			return rc;

		CDBG("mt9p012 sensor configuration done!\n");
		break;

	case FULL_SIZE:
#ifdef CONFIG_MACH_MOT
		rc = mt9p012_setting(REG_INIT, RES_CAPTURE);
#else
		rc = mt9p012_setting(UPDATE_PERIODIC, RES_CAPTURE);
#endif
		if (rc < 0)
			return rc;

#ifdef CONFIG_MACH_MOT
		rc =
		mt9p012_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0)
			return rc;

#endif
		break;

	default:
		return 0;
	}			/* switch */

	mt9p012_ctrl->prev_res = res;
	mt9p012_ctrl->curr_res = res;
	mt9p012_ctrl->sensormode = mode;

#ifdef CONFIG_MACH_MOT
	rc =
		mt9p012_write_exp_gain(mt9p012_ctrl->my_reg_gain,
			mt9p012_ctrl->my_reg_line_count, -1);
#else
	rc = mt9p012_write_exp_gain(mt9p012_ctrl->my_reg_gain,
				    mt9p012_ctrl->my_reg_line_count);
#endif

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER, 0x10cc | 0x0002);

	return rc;
}

static int32_t mt9p012_snapshot_config(int mode)
{
	int32_t rc = 0;

#ifdef CONFIG_MACH_MOT
    load_snapshot_lsc = true;
	rc = mt9p012_setting(REG_INIT, RES_CAPTURE);
	if (rc < 0)
		return rc;
#endif

	rc = mt9p012_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9p012_ctrl->curr_res = mt9p012_ctrl->pict_res;

	mt9p012_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9p012_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

#ifdef CONFIG_MACH_MOT
	rc = mt9p012_setting(REG_INIT, RES_CAPTURE);
	if (rc < 0)
		return rc;
#endif

	rc = mt9p012_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9p012_ctrl->curr_res = mt9p012_ctrl->pict_res;

	mt9p012_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9p012_power_down(void)
{
	int32_t rc = 0;

	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWOFF);

#ifdef CONFIG_MACH_MOT
	S_WAIT(5);
#else
	mdelay(5);
#endif
	return rc;
}

#ifdef CONFIG_MACH_MOT

/****************************** EEPROM CODE ******************************/

static int32_t eeprom_i2c_read_block(unsigned short raddr,
        uint8_t *rdata, uint16_t length)
{
    struct i2c_msg msg[2];
    uint16_t ee_addr = (0xa0>>1);
    int32_t rc = 0;
    unsigned char buf[4];
    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00)>>8;
    buf[1] = (raddr & 0x00FF);


    msg[0].addr = ee_addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = buf;

    msg[1].addr = ee_addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = length;
    msg[1].buf = rdata;

    rc = i2c_transfer(mt9p012_client->adapter, msg, 2);

    if (rc < 0)
        CDBG("eeprom i2c_read_block failed!\n");

    return rc;
}

bool read_eep (void)
{
    uint16_t i;
    uint16_t data_pending;
    bool ret_val = true;


    pEpromImage = kmalloc (EEPROM_SIZE, GFP_ATOMIC);
    if (pEpromImage == NULL)
    {
        CDBG("CAL memory not allocated");
        return false;
    }

    data_pending = EEPROM_SIZE;
    for (i = 0; data_pending > EEPROM_BLOCK_SZ; i++)
    {
       if ((eeprom_i2c_read_block (i*EEPROM_BLOCK_SZ,
                    &pEpromImage[i*EEPROM_BLOCK_SZ],
				  EEPROM_BLOCK_SZ))< 0)
        {
            CDBG("CAL block %d read fail\n", i);
            eep_state = EEP_DATA_INVALID;
            ret_val= false;
            break;
        }

        data_pending -= EEPROM_BLOCK_SZ;
    }

    if (!eeprom_i2c_read_block (i*EEPROM_BLOCK_SZ,
                &pEpromImage[i*EEPROM_BLOCK_SZ],
                data_pending))
    {
        CDBG("CAL block %d read fail\n", i);
        eep_state = EEP_DATA_INVALID;
        ret_val= false;
    }

    if (ret_val == false)
    {
        eep_state = EEP_DATA_INVALID;
        kfree(pEpromImage);
        pEpromImage=NULL;
    }
    return ret_val;
}

/************************ AF CODE BASED ON BAM SPEC **************************/

static void bam_init_ic (void)
{
    mt9p012_i2c_write_b(af_addr, 0x01, 0x34);
    mt9p012_i2c_write_b(af_addr, 0x07, 0x44);
    mt9p012_i2c_write_b(af_addr, 0x0A, 0xFF);
    mt9p012_i2c_write_b(af_addr, 0x16, 0x15);
}

static unsigned short bam_read_pos (void)
{
    uint8_t curr_pos_h;
    uint8_t curr_pos_l;

    mt9p012_i2c_write_b (af_addr, 0x01, (0x34 | 0x03));
    mt9p012_i2c_read_b (af_addr, 0x03, &curr_pos_h);
    mt9p012_i2c_read_b (af_addr, 0x04, &curr_pos_l);

    return ((curr_pos_h << 8) | curr_pos_l);
}

static unsigned short bam_read_infinite (void)
{
	uint8_t inf_pos_h = 0;
	uint8_t inf_pos_l = 0;
	unsigned short inf_pos;

	mt9p012_i2c_read_b(af_addr, 0x12, &inf_pos_h);
	mt9p012_i2c_read_b(af_addr, 0x13, &inf_pos_l);

	inf_pos = ((inf_pos_h << 8) | inf_pos_l);

	/* Default infinity at 0x250 */
	if (inf_pos == 0) {
		inf_pos = 0x250;
		CDBG("bam_read_infinite: AF position not calibrated");
	} else {
		inf_pos += 10;
	}

	return (inf_pos);
}

static unsigned short bam_read_macro (void)
{
	uint8_t macro_pos_h = 0;
	uint8_t macro_pos_l = 0;
    unsigned short macro_pos;

	mt9p012_i2c_read_b(af_addr, 0x14, &macro_pos_h);
	mt9p012_i2c_read_b(af_addr, 0x15, &macro_pos_l);

	macro_pos = ((macro_pos_h << 8) | macro_pos_l);

	if (macro_pos == 0)
		CDBG("bam_read_macro: AF position not calibrated");

	return (macro_pos);
}

static void bam_init_freq (void)
{
    uint8_t orig_freq, temp_freq, new_freq;

    if (af_addr == 0x45) {
        /* Validate the frequency of the actuator */
        mt9p012_i2c_read_b(af_addr, 0x08, &orig_freq);

        if ((orig_freq < 64) || (orig_freq > 127))
        {
            new_freq = orig_freq;
        }
        else
        {
            if (bam_check_freq (orig_freq) == 1)
            {
                new_freq = orig_freq;
            }
            else
            {
                /* Low Frequency */
                temp_freq = ((orig_freq > 67) ? (orig_freq - 3) : 64);
                if (bam_check_freq (temp_freq) == 1)
                {
                    new_freq = temp_freq;
                }
                else
                {
                    /* High Frequency */
                    temp_freq = ((orig_freq < 124) ? (orig_freq + 3) : 127);
                    if (bam_check_freq (temp_freq) == 1)
                    {
                        new_freq = temp_freq;
                    }
                    else
                    {
                        new_freq = orig_freq;
                    }
                }
            }
        }
        if (orig_freq != new_freq) {
            mt9p012_i2c_write_b(af_addr, 0x08, new_freq);
            CDBG("bam_init_freq: New AF frequency set to %d\n", new_freq);
        }
    }
}

static uint8_t bam_check_freq (uint8_t freq)
{
	unsigned short first_pos, second_pos;

	bam_move_lens(bam_macro, 40);
	bam_move_lens(bam_infinite, 40);
	bam_move_lens(bam_macro, 40);
	first_pos = bam_read_pos ();
	bam_move_lens (bam_infinite, 40);
	second_pos = bam_read_pos ();

	if (second_pos > first_pos + ((bam_infinite - bam_macro)/2) )
		return 1;
	else
		return 0;
}

static int32_t bam_move_lens(int16_t new_pos, uint8_t time_out)
{
    uint8_t index;
    uint8_t af_state;
    int32_t rc;

    if (af_addr == 0x05) {
        rc = mt9p012_i2c_write_b(af_addr, 0x01, 0x29);
        if (rc < 0)
            return rc;

        rc = mt9p012_i2c_write_b(af_addr, 0x05, (new_pos >> 8));
        if (rc < 0)
            return rc;

        rc = mt9p012_i2c_write_b(af_addr, 0x06, (new_pos & 0x00FF));
        if (rc < 0)
            return rc;

        rc = mt9p012_i2c_write_b(af_addr, 0x0B, time_out);
        if (rc < 0)
            return rc;

        rc = mt9p012_i2c_write_b(af_addr, 0x07, 0x27);
        if (rc < 0)
            return rc;

        time_out += time_out>>1;

		S_WAIT(time_out);

    } else {

        rc = mt9p012_i2c_write_b(af_addr, 0x01, (0x34)|(0x10));
        if (rc < 0)
            return rc;

        rc = mt9p012_i2c_write_b(af_addr, 0x05, ((new_pos >> 8)&(0x03)) );
        if (rc < 0)
            return rc;

        rc = mt9p012_i2c_write_b(af_addr, 0x06, (new_pos & 0x00FF));
        if (rc < 0)
            return rc;

        rc = mt9p012_i2c_write_b(af_addr, 0x0B, time_out);
        if (rc < 0)
            return rc;

        rc = mt9p012_i2c_write_b(af_addr, 0x07, (0x44)|(0x01));
        if (rc < 0)
            return rc;

        /* Wait up to 1.5 times delay"time_out"; 5ms at a time */
        time_out += time_out>>1;
        for (index = 0; index < (time_out/5); index++) {
            if (mt9p012_i2c_poll_b(af_addr, 0x02, &af_state) >= 0) {
                af_state &= 0x0c;
                if ((af_state == 0x04) || (af_state == 0x08))
                    break;
            }
            S_WAIT(5);
        }
    }

    return rc;
}
#endif /* #ifdef CONFIG_MACH_MOT */

static int32_t mt9p012_move_focus(int direction, int32_t num_steps)
{
	int32_t rc = 0;
	int16_t step_direction;
	int16_t actual_step;
	int16_t next_position;
	uint8_t code_val;
	uint8_t time_out;
	uint16_t actual_position_target;
#ifdef CONFIG_MACH_MOT
	uint16_t current_position_pi;
	uint16_t pi_gap;
#else
	uint8_t temp_pos;
#endif

#ifdef CONFIG_MACH_MOT
    if(disable_lens_move)
    {
        printk("CAM_TCMD: lens move disabled.");
        return 0;
    }
    if (af_addr == 0) {
        return 0;
    }

    CVBS("%s: IN, num_steps = %d\n", __func__, num_steps);
#endif

	if (num_steps > MT9P012_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = MT9P012_TOTAL_STEPS_NEAR_TO_FAR;
	else if (num_steps == 0) {
		CDBG("mt9p012_move_focus failed at line %d ...\n", __LINE__);
		return -EINVAL;
	}

	if (direction == MOVE_NEAR)
		step_direction = -1;
	else if (direction == MOVE_FAR)
		step_direction = 1;
	else {
		CDBG("mt9p012_move_focus failed at line %d ...\n", __LINE__);
		return -EINVAL;
	}

	if (mt9p012_ctrl->curr_lens_pos < mt9p012_ctrl->init_curr_lens_pos)
		mt9p012_ctrl->curr_lens_pos = mt9p012_ctrl->init_curr_lens_pos;

	actual_step = (int16_t) (step_direction * (int16_t) num_steps);
	next_position = (int16_t) (mt9p012_ctrl->curr_lens_pos + actual_step);

	if (next_position > MT9P012_TOTAL_STEPS_NEAR_TO_FAR)
		next_position = MT9P012_TOTAL_STEPS_NEAR_TO_FAR;
	else if (next_position < 0)
		next_position = 0;

#ifndef CONFIG_MACH_MOT
	if (num_steps >= 10)
		time_out = 100;
	else
		time_out = 30;
#endif
	code_val = next_position;
	actual_position_target = bam_step_lookup_table[code_val];

#ifdef CONFIG_MACH_MOT
	current_position_pi =
		bam_step_lookup_table[mt9p012_current_lens_position];
	pi_gap = DIFF(actual_position_target, current_position_pi);
	if (pi_gap < 20) {
	  time_out = 15;
	} else if (pi_gap < 50 ) {
	  time_out = 30;
	} else if (pi_gap < 70)	{
	  time_out = 60;
	} else if (pi_gap < 100) {
	  time_out = 80;
	} else {
	  time_out = 100;
	}

    CVBS("AF: step %d   value %03x\n", next_position, actual_position_target);

    rc = bam_move_lens( actual_position_target, time_out );

#else /* #ifndef CONFIG_MACH_MOT */

	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x01, 0x29);
	if (rc < 0)
		return rc;
	temp_pos = (uint8_t) (actual_position_target >> 8);
	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x05, temp_pos);
	if (rc < 0)
		return rc;
	temp_pos = (uint8_t) (actual_position_target & 0x00FF);
	/* code_val_lsb |= mode_mask; */
	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x06, temp_pos);
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x0B, time_out);
	if (rc < 0)
		return rc;
	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x07, 0x27);
	if (rc < 0)
		return rc;

	mdelay(time_out);

#endif /* #ifdef CONFIG_MACH_MOT */

	/* Storing the current lens Position */
	mt9p012_ctrl->curr_lens_pos = next_position;

	return rc;
}

static int32_t mt9p012_set_default_focus(void)
{
	int32_t rc = 0;

#ifdef CONFIG_MACH_MOT
    if(disable_lens_move)
    {
        printk("CAM_TCMD: lens move disabled.");
        return 0;
    }

    if (af_addr == 0) {
        return 0;
    }

    /* Write the digital code for current to the actuator */
    rc = bam_move_lens(bam_infinite, 40);
#else

	uint8_t temp_pos;

	/* Write the digital code for current to the actuator */
	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x01, 0x29);
	if (rc < 0)
		return rc;
	temp_pos = (uint8_t) (bam_infinite >> 8);

	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x05, temp_pos);
	if (rc < 0)
		return rc;
	temp_pos = (uint8_t) (bam_infinite & 0x00FF);
	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x06, temp_pos);
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x0B, 0x64);
	if (rc < 0)
		return rc;

	rc = mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x07, 0x27);
	if (rc < 0)
		return rc;

	mdelay(140);
#endif

	mt9p012_ctrl->curr_lens_pos = MT9P012_TOTAL_STEPS_NEAR_TO_FAR;

	return rc;
}

#ifdef CONFIG_MACH_MOT
static int mt9p012_sensor_power_on(struct msm_camera_sensor_info *data)
{
    int32_t rc;
    uint8_t temp_pos;

    CDBG("%s\n", __func__);

#ifdef CONFIG_MACH_MOT
    if (machine_is_motus()) {
        struct vreg *vreg;
        vreg = vreg_get(0,"gp1");
        if (!vreg) {
            CDBG("%s: vreg_get failed\n", __func__);
            return -EIO;
        }

        if ((rc = vreg_set_level(vreg, 2800))) {
            CDBG("%s: failed to set GP1 level\n", __func__);
            return rc;
        }

        if ((rc = vreg_enable(vreg))) {
            CDBG("%s: failed to enable GP1\n", __func__);
            return rc;
        }
    }
#endif

    rc = gpio_request(data->sensor_pwd, "mt9p012");
    if (!rc) {
        gpio_direction_output(data->sensor_pwd, 0);
        S_WAIT(20);
        gpio_direction_output(data->sensor_pwd, 1);
    }
    else {
        CDBG("%s gpio request %d failed\n", __func__, data->sensor_pwd);
        return rc;
    }

    rc = gpio_request(data->sensor_reset, "mt9p012");
    if (!rc) {
        gpio_direction_output(data->sensor_reset, 0);
        S_WAIT(20);
        gpio_direction_output(data->sensor_reset, 1);
    }
    else {
        CDBG("%s gpio request %d failed\n", __func__, data->sensor_reset);
        return rc;
    }

    /* enable AF actuator */

    /* determine AF actuator by i2c address */
    af_addr = MT9P012_AF_ADDR_BAM2;
    if ( mt9p012_i2c_read_b(af_addr, 0x12, &temp_pos) < 0 ) {
        af_addr = MT9P012_AF_ADDR_BAM1;
        if ( mt9p012_i2c_read_b(af_addr, 0x12, &temp_pos) < 0 ) {
            af_addr = MT9P012_AF_ADDR_VCM;
            if ( mt9p012_i2c_read_b(af_addr, 0x12, &temp_pos) < 0 ) {
                af_addr = 0;
                CDBG("* unable to determine AF actuator\n");
            }
        }
    }

    CDBG("mt9p012 AF addr = 0x%02x\n", af_addr);

    // request max AXI bus for camera
    rc = request_axi_qos();
    if (rc < 0) {
        CDBG("* request of axi qos failed\n");
        return rc;
    }

    return 0;
}

static int mt9p012_sensor_power_off(struct msm_camera_sensor_info *data)
{
    CDBG("%s\n", __func__);

    // make sure we released the AXI QOS
    release_axi_qos();

	gpio_direction_output(data->sensor_pwd, 0);
	gpio_free(data->sensor_pwd);

	gpio_direction_output(data->sensor_reset, 0);
	gpio_free(data->sensor_reset);

#ifdef CONFIG_MACH_MOT
    if (machine_is_motus()) {
        struct vreg *vreg;
        vreg = vreg_get(0,"gp1");
        if (!vreg) {
            CDBG("%s: vreg_get failed\n", __func__);
            return -EIO;
        }

        int32_t rc;
        if ((rc = vreg_disable(vreg))) {
            CDBG("%s: failed to disable GP1\n", __func__);
            return rc;
        }
    }
#endif

   return 0;
}
#endif /* #ifdef CONFIG_MACH_MOT */

#ifdef CONFIG_MACH_MOT
static int mt9p012_probe_init_done(struct msm_camera_sensor_info *data)
#else
static int mt9p012_probe_init_done(const struct msm_camera_sensor_info *data)
#endif
{
#ifdef CONFIG_MACH_MOT
    mt9p012_sensor_power_off(data);
#else
	gpio_direction_output(data->sensor_reset, 0);
	gpio_free(data->sensor_reset);
#endif
	return 0;
}

#ifdef CONFIG_MACH_MOT
static int mt9p012_probe_init_sensor(struct msm_camera_sensor_info *data)
#else
static int mt9p012_probe_init_sensor(const struct msm_camera_sensor_info *data)
#endif
{
	int32_t rc;
	uint16_t chipid;
#ifdef CONFIG_MACH_MOT
    uint16_t revnum;

    mt9p012_sensor_power_on(data);

	S_WAIT(20);
#else
	rc = gpio_request(data->sensor_reset, "mt9p012");
	if (!rc)
		gpio_direction_output(data->sensor_reset, 1);
	else
		goto init_probe_done;

	msleep(20);
#endif

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9P012_DEFAULT_CLOCK_RATE);

#ifdef CONFIG_MACH_MOT
	S_WAIT(20);
#else
	msleep(20);
#endif
	/* RESET the sensor image part via I2C command */
	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
#ifdef CONFIG_MACH_MOT
			MT9P012_REG_RESET_REGISTER, 0x10CC|0x0001|1<<5);
#else
			MT9P012_REG_RESET_REGISTER, 0x10CC | 0x0001);
#endif
	if (rc < 0) {
		CDBG("sensor reset failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

#ifdef CONFIG_MACH_MOT
	S_WAIT(MT9P012_RESET_DELAY_MSECS);
#else
	msleep(MT9P012_RESET_DELAY_MSECS);
#endif

	/* 3. Read sensor Model ID: */
	rc = mt9p012_i2c_read_w(mt9p012_client->addr,
				MT9P012_REG_MODEL_ID, &chipid);
	if (rc < 0)
		goto init_probe_fail;

#ifdef CONFIG_MACH_MOT
	CDBG("mt9p012 model_id = 0x%x\n", chipid);

    rc = mt9p012_i2c_read_w(mt9p012_client->addr,
        0x31FE, &revnum);
    if (rc < 0)
        goto init_probe_fail;

    CDBG("mt9p012 revision = %d\n", revnum);

	/* 4. Verify sensor ID: */
	if (chipid != MT9P012_MODEL_ID
	&&
	chipid != MT9P012_MODEL_ID_5131) {
		rc = -ENODEV;
		goto init_probe_fail;
	}
#else
	/* 4. Compare sensor ID to MT9T012VC ID: */
	if (chipid != MT9P012_MODEL_ID) {
		rc = -ENODEV;
		goto init_probe_fail;
	}
#endif

	rc = mt9p012_i2c_write_w(mt9p012_client->addr, 0x306E, 0x9000);
	if (rc < 0) {
		CDBG("REV_7 write failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

	/* RESET_REGISTER, enable parallel interface and disable serialiser */
	rc = mt9p012_i2c_write_w(mt9p012_client->addr, 0x301A, 0x10CC);
	if (rc < 0) {
		CDBG("enable parallel interface failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

	/* To disable the 2 extra lines */
	rc = mt9p012_i2c_write_w(mt9p012_client->addr, 0x3064, 0x0805);

	if (rc < 0) {
		CDBG("disable the 2 extra lines failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

#ifdef CONFIG_MACH_MOT
	S_WAIT(MT9P012_RESET_DELAY_MSECS);
#else
	msleep(MT9P012_RESET_DELAY_MSECS);
#endif
	goto init_probe_done;

init_probe_fail:
	mt9p012_probe_init_done(data);
init_probe_done:
	return rc;
}

#ifdef CONFIG_MACH_MOT
static int mt9p012_sensor_open_init(struct msm_camera_sensor_info *data)
#else
static int mt9p012_sensor_open_init(const struct msm_camera_sensor_info *data)
#endif
{
	int32_t rc = 0;
#ifndef CONFIG_MACH_MOT
	unsigned short temp_pos;
	uint16_t temp;
#endif
	uint8_t i;
#ifdef CONFIG_MACH_MOT
    int attempt = 0;
#endif

#ifdef CONFIG_MACH_MOT
	mt9p012_ctrl = kzalloc(sizeof(struct mt9p012_ctrl_t), GFP_KERNEL);
#else
	mt9p012_ctrl = kzalloc(sizeof(struct mt9p012_ctrl), GFP_KERNEL);
#endif
	if (!mt9p012_ctrl) {
		CDBG("mt9p012_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9p012_ctrl->fps_divider = 1 * 0x00000400;
	mt9p012_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9p012_ctrl->set_test = TEST_OFF;
	mt9p012_ctrl->prev_res = QTR_SIZE;
	mt9p012_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9p012_ctrl->sensordata = data;

#ifdef CONFIG_MACH_MOT
    wake_lock_init(&mt9p012_wake_lock, WAKE_LOCK_IDLE, "mt9p012");
    wake_lock(&mt9p012_wake_lock);

    do {
        if (rc < 0)
            CDBG("mt9p012: mt9p012_sensor_open_init: ATTEMPT FAILED: rc=%d\n", rc);
        attempt++;

	    /* enable mclk first */
	    msm_camio_clk_rate_set(MT9P012_DEFAULT_CLOCK_RATE);
	    S_WAIT(20);

        msm_camio_camif_pad_reg_reset();
        S_WAIT(20);

        rc = mt9p012_probe_init_sensor(data);
        if (rc < 0)
            continue;

        if (mt9p012_ctrl->prev_res == QTR_SIZE)
            rc = mt9p012_setting(REG_INIT, RES_PREVIEW);
        else
            rc = mt9p012_setting(REG_INIT, RES_CAPTURE);

        if (rc < 0) {
            CDBG("mt9p012_setting failed. rc = %d\n", rc);
            continue;
        }

        /* sensor : output enable */
        rc = mt9p012_i2c_write_w(mt9p012_client->addr,
                MT9P012_REG_RESET_REGISTER, MT9P012_RESET_REGISTER_PWON);
        if (rc < 0) {
            CDBG("sensor output enable failed. rc = %d\n", rc);
            continue;
        }

        S_WAIT(20);
        bam_infinite = 0;
        bam_macro    = 0;

        if (af_addr != 0) {
            bam_init_ic ();

            bam_infinite = bam_read_infinite();
            bam_macro = bam_read_macro();

            bam_init_freq();
        }

	    for (i = 0; i < MT9P012_TOTAL_STEPS_NEAR_TO_FAR; i++)
		    bam_step_lookup_table[i] = bam_macro +
		      i*(bam_infinite-bam_macro)/MT9P012_TOTAL_STEPS_NEAR_TO_FAR; ;

	    bam_step_lookup_table[MT9P012_TOTAL_STEPS_NEAR_TO_FAR] = bam_infinite;

        rc = mt9p012_set_default_focus();
    } while (rc < 0 && attempt < CAMERA_CONFIG_ATTEMPTS);

	if (rc >= 0)
		goto init_done;

#else /* #ifndef CONFIG_MACH_MOT */

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9P012_DEFAULT_CLOCK_RATE);
	mdelay(20);

	msm_camio_camif_pad_reg_reset();
	mdelay(20);

	rc = mt9p012_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail1;

	if (mt9p012_ctrl->prev_res == QTR_SIZE)
		rc = mt9p012_setting(REG_INIT, RES_PREVIEW);
	else
		rc = mt9p012_setting(REG_INIT, RES_CAPTURE);

	if (rc < 0) {
		CDBG("mt9p012_setting failed. rc = %d\n", rc);
		goto init_fail1;
	}

	/* sensor : output enable */
	rc = mt9p012_i2c_write_w(mt9p012_client->addr,
				 MT9P012_REG_RESET_REGISTER,
				 MT9P012_RESET_REGISTER_PWON);
	if (rc < 0) {
		CDBG("sensor output enable failed. rc = %d\n", rc);
		goto init_fail1;
	}

	/* enable AF actuator */
	rc = gpio_request(mt9p012_ctrl->sensordata->vcm_pwd, "mt9p012");
	if (!rc)
		gpio_direction_output(mt9p012_ctrl->sensordata->vcm_pwd, 1);
	else {
		CDBG("mt9p012_ctrl gpio request failed!\n");
		goto init_fail1;
	}

	mdelay(20);

	bam_infinite = 0;
	bam_macro = 0;
	/*initialize AF actuator */
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x01, 0x09);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x07, 0x2E);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x0A, 0x01);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x17, 0x06);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x16, 0x0A);

	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x01, 0x29);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x05, 0x00);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x06, 0x00);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x0B, 0x64);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x07, 0x27);
	mdelay(140);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x01, 0x29);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x05, 0x03);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x06, 0xFF);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x0B, 0x64);
	mt9p012_i2c_write_b(MT9P012_AF_I2C_ADDR >> 1, 0x07, 0x27);
	mdelay(140);

	if (mt9p012_i2c_read_b(MT9P012_AF_I2C_ADDR >> 1, 0x12, &temp_pos)
	    >= 0) {
		bam_infinite = (uint16_t) temp_pos;
		if (mt9p012_i2c_read_b
		    (MT9P012_AF_I2C_ADDR >> 1, 0x13, &temp_pos) >= 0)
			bam_infinite =
			    (bam_infinite << 8) | ((uint16_t) temp_pos);
	} else {
		bam_infinite = 100;
	}

	if (mt9p012_i2c_read_b(MT9P012_AF_I2C_ADDR >> 1, 0x14, &temp_pos)
	    >= 0) {
		bam_macro = (uint16_t) temp_pos;
		if (mt9p012_i2c_read_b
		    (MT9P012_AF_I2C_ADDR >> 1, 0x15, &temp_pos) >= 0)
			bam_macro = (bam_macro << 8) | ((uint16_t) temp_pos);
	}
	temp = (bam_infinite - bam_macro) / MT9P012_TOTAL_STEPS_NEAR_TO_FAR;
	for (i = 0; i < MT9P012_TOTAL_STEPS_NEAR_TO_FAR; i++)
		bam_step_lookup_table[i] = bam_macro + temp * i;

	bam_step_lookup_table[MT9P012_TOTAL_STEPS_NEAR_TO_FAR] = bam_infinite;

	rc = mt9p012_set_default_focus();
	if (rc >= 0)
		goto init_done;

#endif /* #ifdef CONFIG_MACH_MOT */

#ifndef CONFIG_MACH_MOT
init_fail1:
#endif
	mt9p012_probe_init_done(data);
	kfree(mt9p012_ctrl);
init_done:
	return rc;
}

static int mt9p012_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9p012_wait_queue);
	return 0;
}

static int32_t mt9p012_set_sensor_mode(int mode, int res)
{
#ifdef CONFIG_MACH_MOT
	int attempt = 0;
#endif
	int32_t rc = 0;

#ifdef CONFIG_MACH_MOT
	do {
        if (rc < 0)
            CDBG("mt9p012: set_sensor_mode: ATTEMPT FAILED: rc=%d\n", rc);
        attempt++;
#endif

		switch (mode) {
		case SENSOR_PREVIEW_MODE:
			rc = mt9p012_video_config(mode, res);
			break;

		case SENSOR_SNAPSHOT_MODE:
			rc = mt9p012_snapshot_config(mode);
			break;

		case SENSOR_RAW_SNAPSHOT_MODE:
			rc = mt9p012_raw_snapshot_config(mode);
			break;

		default:
			rc = -EINVAL;
			break;
	}
#ifdef CONFIG_MACH_MOT
    } while (rc < 0 && rc != -EINVAL && attempt < CAMERA_CONFIG_ATTEMPTS);
#endif

	return rc;
}

int mt9p012_sensor_config(void __user *argp)
{
	struct sensor_cfg_data *cdata;
	int rc = 0;

	cdata = kmalloc(sizeof(struct sensor_cfg_data), GFP_ATOMIC);
    
    if (cdata == NULL)
        return -ENOMEM;

    if (copy_from_user(cdata,
			   (void *)argp, sizeof(struct sensor_cfg_data))) {
        kfree(cdata);
		return -EFAULT;
    }

	mutex_lock(&mt9p012_mut);

	CDBG("%s: cfgtype = %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_PICT_FPS:
		mt9p012_get_pict_fps(cdata->cfg.gfps.prevfps,
				     &(cdata->cfg.gfps.pictfps));

		if (copy_to_user((void *)argp, cdata,
				 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata->cfg.prevl_pf = mt9p012_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
				 cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata->cfg.prevp_pl = mt9p012_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
				 cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata->cfg.pictl_pf = mt9p012_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
				 cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata->cfg.pictp_pl = mt9p012_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
				 cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata->cfg.pict_max_exp_lc = mt9p012_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
				 cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = mt9p012_set_fps(&(cdata->cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
#ifdef CONFIG_MACH_MOT
 		CVBS("Line:%d CFG_SET_EXP_GAIN gain=%d line=%d\n",
                         __LINE__, cdata->cfg.exp_gain.gain,
                         cdata->cfg.exp_gain.line);
  		rc =
  			mt9p012_write_exp_gain(cdata->cfg.exp_gain.gain,
 				cdata->cfg.exp_gain.line,
 				cdata->cfg.exp_gain.is_outdoor);
#else
		rc = mt9p012_write_exp_gain(cdata->cfg.exp_gain.gain,
					    cdata->cfg.exp_gain.line);
#endif
		break;

	case CFG_SET_PICT_EXP_GAIN:
		CDBG("Line:%d CFG_SET_PICT_EXP_GAIN \n", __LINE__);
		rc = mt9p012_set_pict_exp_gain(cdata->cfg.exp_gain.gain,
					       cdata->cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc = mt9p012_set_sensor_mode(cdata->mode, cdata->rs);
		break;

	case CFG_PWR_DOWN:
		rc = mt9p012_power_down();
		break;

	case CFG_MOVE_FOCUS:
		CDBG("mt9p012_ioctl: CFG_MOVE_FOCUS: dir=%d steps=%d\n",
		     cdata->cfg.focus.dir, cdata->cfg.focus.steps);
		rc = mt9p012_move_focus(cdata->cfg.focus.dir,
					cdata->cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = mt9p012_set_default_focus();

		break;

	case CFG_SET_EFFECT:
		rc = mt9p012_set_default_focus();
		break;

	case CFG_SET_LENS_SHADING:
		CDBG("%s: CFG_SET_LENS_SHADING\n", __func__);
		rc = mt9p012_lens_shading_enable(cdata->cfg.lens_shading);
		break;

	case CFG_GET_AF_MAX_STEPS:
		cdata->max_steps = MT9P012_STEPS_NEAR_TO_CLOSEST_INF;
		if (copy_to_user((void *)argp,
				 cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

#ifdef CONFIG_MACH_MOT
    case CFG_LOAD_TEST_PATTERN_DLI:
        mt9p012_ctrl->set_test = TEST_W1_10B;
        rc = mt9p012_setting(UPDATE_PERIODIC, RES_CAPTURE);
        break;

    case CFG_GET_EEPROM_DATA:
        if (pEpromImage != NULL)
        {
          memcpy (cdata->cfg.eeprom_data_ptr, pEpromImage, EEPROM_SIZE);
          if (copy_to_user ((void *)argp, cdata, sizeof(struct sensor_cfg_data)))
            rc = -EFAULT;
          else
            CDBG ("EEPROM data provided by kernel\n");
        }
        else
          rc= -EFAULT;
        break;

    case CFG_SET_MODULE_DATA:
        if (cdata->cfg.module_data.valid_data == true)
        {
            memcpy (&module_data, &cdata->cfg.module_data,
                  sizeof(module_data));

            eep_state = EEP_DATA_VALID;
        }
        else
        {
            CDBG ("EEP- INVALID parsed data passed in \n");
            module_data.valid_data = false;
            eep_state = EEP_DATA_INVALID;
        }

        if (pEpromImage != NULL)
        {
          kfree(pEpromImage);
          pEpromImage=NULL;
        }
        break;

    case CFG_GET_WB_GAINS:
        CVBS("%s: CFG_GET_WB_GAINS\n", __func__);
        if ((eep_state == EEP_DATA_VALID) && (module_data.lsc_count != 1))
        {
            cdata->cfg.wb_gains.g_gain = module_data.Ggain;
            cdata->cfg.wb_gains.r_gain = module_data.Rgain;
            cdata->cfg.wb_gains.b_gain = module_data.Bgain;
        }
        if (copy_to_user((void *)argp, cdata, sizeof(struct sensor_cfg_data)))
            rc = -EFAULT;
        break;

 	case CFG_SET_AWB_DECISION:
            awb_decision =  cdata->cfg.awb_decision;
            CVBS("awb_decision = %d\n", cdata->cfg.awb_decision);
            break;

    case CFG_GET_CAL_DATA_STATUS:
        CDBG("%s: CFG_GET_CAL_DATA_STATUS\n", __func__);
        memcpy (&cdata->cfg.module_data, &module_data,
                  sizeof(module_data));
       if (copy_to_user((void *)argp, cdata, sizeof(struct sensor_cfg_data)))
            rc = -EFAULT;
        break;

    case CFG_GET_AF_DEVICE_ADDRESS: 
        cdata->cfg.af_device_address = af_addr; //mt9p012_get_af_actuator_address()
        if (copy_to_user((void *)argp,
                cdata,
                sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
        break;
#endif  /* CONFIG_MACH_MOT */

	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&mt9p012_mut);
	kfree(cdata);
    return rc;
}

int mt9p012_sensor_release(void)
{
	int rc = -EBADF;

	mutex_lock(&mt9p012_mut);

	mt9p012_power_down();

#ifdef CONFIG_MACH_MOT
    mt9p012_sensor_power_off(mt9p012_ctrl->sensordata);
#else
	gpio_direction_output(mt9p012_ctrl->sensordata->sensor_reset, 0);
	gpio_free(mt9p012_ctrl->sensordata->sensor_reset);

	gpio_direction_output(mt9p012_ctrl->sensordata->vcm_pwd, 0);
	gpio_free(mt9p012_ctrl->sensordata->vcm_pwd);
#endif

	kfree(mt9p012_ctrl);
	mt9p012_ctrl = NULL;

#ifdef CONFIG_MACH_MOT
    wake_unlock(&mt9p012_wake_lock);
    wake_lock_destroy(&mt9p012_wake_lock);
#endif

	CDBG("mt9p012_release completed\n");

	mutex_unlock(&mt9p012_mut);

	return rc;
}

#ifdef CONFIG_MACH_MOT
static int mt9p012_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
#else
static int mt9p012_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
#endif
{
	int rc = 0;
	CDBG("mt9p012_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

#ifdef CONFIG_MACH_MOT
	mt9p012_sensorw = kzalloc(sizeof(struct mt9p012_work_t), GFP_KERNEL);
#else
	mt9p012_sensorw = kzalloc(sizeof(struct mt9p012_work), GFP_KERNEL);
#endif
	if (!mt9p012_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9p012_sensorw);
	mt9p012_init_client(client);
	mt9p012_client = client;

#ifdef CONFIG_MACH_MOT
	S_WAIT(50);

	CDBG("mt9p012_probe succeeded!\n");
    if (read_eep())
    {
        CDBG("EEPROM read successfully \n");
        eep_state = EEP_DATA_READ;
    }
#endif /* #ifdef CONFIG_MACH_MOT */
	return 0;

probe_failure:
	CDBG("mt9p012_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit mt9p012_remove(struct i2c_client *client)
{
	struct mt9p012_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
#ifdef CONFIG_MACH_MOT
	i2c_detach_client(client);
#endif
	mt9p012_client = NULL;
	kfree(sensorw);
	return 0;
}

#ifdef CONFIG_MACH_MOT
static const struct i2c_device_id mt9p012_id[] = {
	{ "mt9p012", 0},
	{ }
};
#endif
#ifndef CONFIG_MACH_MOT
static const struct i2c_device_id mt9p012_i2c_id[] = {
	{"mt9p012", 0}
};
#endif

#ifdef CONFIG_MACH_MOT
static struct i2c_driver mt9p012_driver = {
	.id_table = mt9p012_id,
	.probe  = mt9p012_probe,
	.remove = __exit_p(mt9p012_remove),
	.driver = {
		.name = "mt9p012",
	},
};
#endif
#ifndef CONFIG_MACH_MOT
static struct i2c_driver mt9p012_i2c_driver = {
	.id_table = mt9p012_i2c_id,
	.probe = mt9p012_i2c_probe,
	.remove = __exit_p(mt9p012_i2c_remove),
	.driver = {
		.name = "mt9p012",
	},
};
#endif

#ifdef CONFIG_MACH_MOT
static int32_t mt9p012_init(void)
{
	int32_t rc = 0;

	CDBG("mt9p012_init called\n");

	rc = i2c_add_driver(&mt9p012_driver);
	if (IS_ERR_VALUE(rc))
		goto init_fail;

	return rc;

init_fail:
	CDBG("mt9p012_init failed\n");
	return rc;
}

void mt9p012_exit(void)
{
	i2c_del_driver(&mt9p012_driver);
}

int mt9p012_probe_init(void *dev, void *ctrl)
{
	int rc = 0;
	struct msm_camera_sensor_info *info =
		(struct msm_camera_sensor_info *)dev;

	struct msm_sensor_ctrl_t *s = (struct msm_sensor_ctrl_t *)ctrl;

	rc = mt9p012_init();
	if (rc < 0)
		goto probe_done;

	msm_camio_clk_rate_set(MT9P012_DEFAULT_CLOCK_RATE);
	S_WAIT(20);

	rc = mt9p012_probe_init_sensor(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = mt9p012_sensor_open_init;
	s->s_release = mt9p012_sensor_release;
	s->s_config  = mt9p012_sensor_config;
	mt9p012_probe_init_done(info);

probe_done:
	CDBG("%s: rc = %d\n", __func__, rc);
	return rc;
}
#endif /* #ifdef CONFIG_MACH_MOT */

#ifndef CONFIG_MACH_MOT
static int mt9p012_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&mt9p012_i2c_driver);
	if (rc < 0 || mt9p012_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	msm_camio_clk_rate_set(MT9P012_DEFAULT_CLOCK_RATE);
	mdelay(20);

	rc = mt9p012_probe_init_sensor(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = mt9p012_sensor_open_init;
	s->s_release = mt9p012_sensor_release;
	s->s_config = mt9p012_sensor_config;
	mt9p012_probe_init_done(info);

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __mt9p012_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mt9p012_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9p012_probe,
	.driver = {
		.name = "msm_camera_mt9p012",
		.owner = THIS_MODULE,
	},
};

static int __init mt9p012_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9p012_init);
void mt9p012_exit(void)
{
	i2c_del_driver(&mt9p012_i2c_driver);
}
#endif /* #ifndef CONFIG_MACH_MOT */

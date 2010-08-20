/*
 * include/linux/qtouch_obp_ts.h - platform/protocol data for Quantum touch IC
 *
 * Copyright (C) 2009 Google, Inc.
  * Copyright (C) 2009 Motorola, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Derived from the Motorola OBP touch driver.
 *
 */
#ifdef __cplusplus
extern "C" {

#endif

/*==================================================================================================
					CONSTANTS
==================================================================================================*/

#ifndef _LINUX_QTOUCH_OBP_TS_H
#define _LINUX_QTOUCH_OBP_TS_H

/* Open it after mXT224C08 chip is integrated  */
//#define HARDWARE_VERSION_MXT224C08

/* elementary defines */
#ifndef FALSE
#define	FALSE	0
#define	TRUE	(!FALSE)
#endif

/* Name for touch screen devices */
#define QTOUCH_TS_NAME          "mXT224"

#define TOUCH_INT_N             124    /* interrupt pin */
#define TOUCH_RST_N             117    /* Reset pin */

#define MXT224_I2C_ADDR         0x4A   /* I2C address in application mode */
#define MXT224_BL_I2C_ADDR      0x24   /* I2C address in bootloader mode */

#define LCD_X_PIXEL             240    /* LCD screen x pixel */
#define LCD_Y_PIXEL             320    /* LCD screen y pixel */

/* Address for ID Information Block in chip */
#define QTM_OBP_ID_INFO_ADDR    0x0000

#define _NUM_FINGERS            10

/* -1 or +1 for trackball framework */
#define TINY_GET_TRACKBALL_REL_EVT(evt) ((evt  >= 0)  ? 1 : -1)
#define TINY_GET_NUMBER_OF_EVT(time)    ((time > 12)  ? 5 : 20)
#define TINY_GET_TH_IDX(i)              ((i    >= 2)  ? 1 : 0)
#define TINY_GET_DIRECTION(evt)         ((evt  >= 0)  ? 1 : -1)

#define TINY_MOVE_THRES_X       40
#define TINY_MOVE_THRES_Y       40

/* part of the info block */
struct qtm_id_info
{
   uint8_t family_id;
   uint8_t variant_id;
   uint8_t version;
   uint8_t build;
   uint8_t matrix_x_size;
   uint8_t matrix_y_size;
   uint8_t num_objs;
} __attribute__ ((packed));

/* an entry in the object table */
struct qtm_obj_entry
{
   uint8_t  type;
   uint16_t addr;
   uint8_t  size;
   uint8_t  num_inst;
   uint8_t  num_rids;
} __attribute__ ((packed));


/* for objects */
struct qtm_object
{
   struct qtm_obj_entry	 entry;
   uint8_t               report_id_min;
   uint8_t               report_id_max;
};

struct axis_map
{
   int  key;
   int  x;
   int  y;
};

struct coordinate_map
{
   int x_data;
   int y_data;
   int z_data;
   int w_data;
   int down;
};

/* Object type list */
enum
{
   QTM_OBJ_RESERVED0                    = 0,
   QTM_OBJ_RESERVED1                    = 1,
   QTM_OBJ_DBG_DELTAS                   = 2,
   QTM_OBJ_DBG_REFS                     = 3,
   QTM_OBJ_DBG_SIGS                     = 4,
   QTM_OBJ_GEN_MSG_PROC                 = 5,
   QTM_OBJ_GEN_CMD_PROC                 = 6,
   QTM_OBJ_GEN_PWR_CONF	                = 7,
   QTM_OBJ_GEN_ACQUIRE_CONF             = 8,
   QTM_OBJ_TOUCH_MULTI                  = 9,
   QTM_OBJ_TOUCH_SINGLE                 = 10,
   QTM_OBJ_TOUCH_XSLIDER                = 11,
   QTM_OBJ_TOUCH_YSLIDER                = 12,
   QTM_OBJ_TOUCH_XWHEEL                 = 13,
   QTM_OBJ_TOUCH_YWHEEL	        	= 14,
   QTM_OBJ_TOUCH_KEYARRAY               = 15,
   QTM_OBJ_PROCG_SIG_FILTER             = 16,
   QTM_OBJ_PROCI_LINEAR_TBL             = 17,
   QTM_OBJ_SPT_COM_CONFIG		= 18,
   QTM_OBJ_SPT_GPIO_PWM	                = 19,
   QTM_OBJ_PROCI_GRIPFACESUPPRESSION    = 20,
   QTM_OBJ_RESERVED3                    = 21,
   QTM_OBJ_PROCG_NOISE_SUPPRESSION      = 22,
   QTM_OBJ_TOUCH_PROXIMITY              = 23,
   QTM_OBJ_PROCI_ONE_TOUCH_GESTURE_PROC = 24,
   QTM_OBJ_SPT_SELF_TEST                = 25,
   QTM_OBJ_DEBUG_CTE_RANGE              = 26,
   QTM_OBJ_PROCI_TWO_TOUCH_GESTURE_PROC = 27,
   QTM_OBJ_SPT_CTE_CONFIG               = 28,
   QTM_OBJ_NOISESUPPRESSION_1           = 36,
   QTM_OBJ_DEBUG_DIAGNOSTIC             = 37,
   QTM_OBJ_SPT_USERDATA                 = 38,

   /* Max number of objects currently defined */
   QTM_OBP_MAX_OBJECT_NUM = QTM_OBJ_SPT_USERDATA + 1,
};

/* OBP structures as defined by the wire protocol. */

/* Note: Not all the structures below need an explicit packed attribute since
 * many of them just contain uint8_t's. However, the protocol is defined in
 * such a way that the structures may expand in the future, with
 * potential multi-byte fields. Thus, we will mark them all as packed to
 * minimize silly bugs in the future.
 */

/*******************************/
/*********** messages **********/
/*******************************/

/* generic message received from the message_processor object. size/buffer
 * defined at runtime after reading the info block */
struct qtm_obj_message
{
   uint8_t report_id;
   uint8_t msg[0];
} __attribute__ ((packed));

/* status message sent by the command processor - T6 */
#define QTM_CMD_PROC_STATUS_RESET       (1 << 7)
#define QTM_CMD_PROC_STATUS_OFL         (1 << 6)
#define QTM_CMD_PROC_STATUS_SIGERR      (1 << 5)
#define QTM_CMD_PROC_STATUS_CAL         (1 << 4)
#define QTM_CMD_PROC_STATUS_CFGERR      (1 << 3)
#define QTM_CMD_PROC_STATUS_COMSERR     (1 << 2)

struct qtm_cmd_proc_msg
{
   uint8_t  report_id;
   uint8_t  status;
   uint8_t  checksum_1;
   uint8_t  checksum_2;
   uint8_t  checksum_3;
} __attribute__ ((packed));

/* status message sent by the mutlitouch touch object - T9*/
#define QTM_TOUCH_MULTI_STATUS_TOUCH            (1 << 7)
#define QTM_TOUCH_MULTI_STATUS_PRESS            (1 << 6)
#define QTM_TOUCH_MULTI_STATUS_RELEASE          (1 << 5)
#define QTM_TOUCH_MULTI_STATUS_MOVE             (1 << 4)
#define QTM_TOUCH_MULTI_STATUS_VECTOR           (1 << 3)
#define QTM_TOUCH_MULTI_STATUS_AMPLITUDE        (1 << 2)
#define QTM_TOUCH_MULTI_STATUS_SUPPRESS         (1 << 1)

struct qtm_touch_multi_msg
{
   uint8_t report_id;
   uint8_t status;
   uint8_t xpos_msb;
   uint8_t ypos_msb;
   uint8_t xypos_lsb;
   uint8_t touch_area;
   uint8_t touch_amp;
   uint8_t touch_vect;
} __attribute__ ((packed));

/* status message sent by the keyarray touch object - T15 */
#define QTM_TOUCH_KEYARRAY_STATUS_TOUCH         (1 << 7)

struct qtm_touch_keyarray_msg
{
   uint8_t  report_id;
   uint8_t  status;
   uint32_t keystate;
} __attribute__ ((packed));

/* status message sent by the gpio pwm object - T19 */
#define	QTM_OBP_T19_STATUS_GPIO1_HIGH_MASK      0x08
#define QTM_OBP_KEY_RELEASE                     0
#define QTM_OBP_KEY_PRESS                       1

struct qtm_spt_gpio_pwm_msg
{
   uint8_t  report_id;
   uint8_t  gpio_status;
} __attribute__ ((packed));



/*******************************/
/**** configuration objects ****/
/*******************************/

/* GEN_COMMANDPROCESSOR_T6 */
struct qtm_gen_cmd_proc
{
   uint8_t reset;
   uint8_t backupnv;
   uint8_t calibrate;
   uint8_t reportall;
   uint8_t debugctrl;
   uint8_t diagnostic;
} __attribute__ ((packed));

/* GEN_POWERCONFIG_T7 */
struct qtm_gen_power_cfg
{
   uint8_t idle_acq_int;      /* in ms */
   uint8_t active_acq_int;    /* in ms */
   uint8_t active_idle_to;    /* in 200ms */
} __attribute__ ((packed));

/* GEN_ACQUIRECONFIG_T8 */
struct qtm_gen_acquire_cfg
{
   uint8_t charge_time;       /* in 250ns */
   uint8_t atouch_drift;      /* in 200ms */
   uint8_t touch_drift;       /* in 200ms */
   uint8_t drift_susp;        /* in 200ms */
   uint8_t touch_autocal;     /* in 200ms */
   uint8_t sync;
   uint8_t atch_cal_suspend_time;
   uint8_t atch_cal_suspend_thres;
} __attribute__ ((packed));

/* TOUCH_MULTITOUCHSCREEN_T9 */
struct qtm_touch_multi_cfg
{
   uint8_t  ctrl;
   uint8_t  x_origin;
   uint8_t  y_origin;
   uint8_t  x_size;
   uint8_t  y_size;
   uint8_t  aks_cfg;
   uint8_t  burst_len;
   uint8_t  tch_det_thr;
   uint8_t  tch_det_int;
   uint8_t  orient;
   uint8_t  mrg_to;
   uint8_t  mov_hyst_init;
   uint8_t  mov_hyst_next;
   uint8_t  mov_filter;
   uint8_t  num_touch;
   uint8_t  merge_hyst;
   uint8_t  merge_thresh;
   uint8_t  amp_hyst;
   uint16_t x_res;
   uint16_t y_res;
   uint8_t  x_low_clip;
   uint8_t  x_high_clip;
   uint8_t  y_low_clip;
   uint8_t  y_high_clip;
   uint8_t  x_edge_ctrl;
   uint8_t  x_edge_dist;
   uint8_t  y_edge_ctrl;
   uint8_t  y_edge_dist;
#if 0
   uint8_t  max_pos_jump;
#endif
} __attribute__ ((packed));

/* TOUCH_KEYARRAY_T15 */
struct qtm_touch_keyarray_cfg
{
   uint8_t ctrl;
   uint8_t x_origin;
   uint8_t y_origin;
   uint8_t x_size;
   uint8_t y_size;
   uint8_t aks_cfg;
   uint8_t burst_len;
   uint8_t tch_det_thr;
   uint8_t tch_det_int;
   uint8_t rsvd1;
   uint8_t rsvd2;
} __attribute__ ((packed));

/* SPT_COMMSCONFIG_T18 */
struct qtm_spt_commsconfig_cfg
{
   uint8_t  ctrl;
   uint8_t  command;
} __attribute__ ((packed));

/* SPT_GPIOPWM_T19*/
struct qtm_spt_gpio_pwm_cfg
{
   uint8_t ctrl;
   uint8_t report_mask;
   uint8_t pin_direction;
   uint8_t internal_pullup;
   uint8_t output_value;
   uint8_t wake_on_change;
   uint8_t pwm_enable;
   uint8_t pwm_period;
   uint8_t duty_cycle_0;
   uint8_t duty_cycle_1;
   uint8_t duty_cycle_2;
   uint8_t duty_cycle_3;
   uint8_t trigger_0;
   uint8_t trigger_1;
   uint8_t trigger_2;
   uint8_t trigger_3;
} __attribute__ ((packed));

/* PROCI_GRIPFACESUPPRESSION_T20 */
struct qtm_proci_grip_suppression_cfg
{
   uint8_t ctrl;
   uint8_t xlogrip;
   uint8_t xhigrip;
   uint8_t ylogrip;
   uint8_t yhigrip;
   uint8_t maxtchs;
   uint8_t reserve0;
   uint8_t szthr1;
   uint8_t szthr2;
   uint8_t shpthr1;
   uint8_t shpthr2;
   uint8_t supextto;
} __attribute__ ((packed));

/* PROCG_NOISESUPPRESSION_T22 */
struct qtm_procg_noise_suppression_cfg
{
   uint8_t  ctrl;
   uint8_t  outlier_filter_len;
   uint8_t  reserve0;
   uint16_t gcaf_upper_limit;
   uint16_t gcaf_lower_limit;
   uint8_t  gcaf_low_count;
   uint8_t  noise_threshold;
   uint8_t  reserve1;
   uint8_t  freq_hop_scale;
   uint8_t  burst_freq_0;
   uint8_t  burst_freq_1;
   uint8_t  burst_freq_2;
   uint8_t  burst_freq_3;
   uint8_t  burst_freq_4;
   uint8_t  idle_gcaf_valid;
} __attribute__ ((packed));

/* TOUCH_PROXIMITY_T23 */
struct qtm_touch_proximity_cfg
{
   uint8_t  ctrl;
   uint8_t  x_origin;
   uint8_t  y_origin;
   uint8_t  x_size;
   uint8_t  y_size;
   uint8_t  reserve0;
   uint8_t  blen;
   uint16_t tch_thresh;
   uint8_t  tch_detect_int;
   uint8_t  average;
   uint16_t rate;
#if 0
   uint16_t mov_thresh;
#endif
} __attribute__ ((packed));

/* PROCI_ONETOUCHGESTUREPROCESSOR_T24 */
struct qtm_proci_one_touch_gesture_proc_cfg
{
   uint8_t  ctrl;
   uint8_t  max_num;
   uint16_t gesture_enable;
   uint8_t  pres_proc;
   uint8_t  tap_time_out;
   uint8_t  flick_time_out;
   uint8_t  drag_time_out;
   uint8_t  short_press_time_out;
   uint8_t  long_press_time_out;
   uint8_t  repeat_press_time_out;
   uint16_t flick_threshold;
   uint16_t drag_threshold;
   uint16_t tap_threshold;
   uint16_t throw_threshold;
} __attribute__ ((packed));

/* SPT_SELFTEST_T25 */
struct qtm_spt_self_test_cfg
{
   uint8_t  ctrl;
   uint8_t  command;
   uint16_t high_signal_limit_0;
   uint16_t low_signal_limit_0;
   uint16_t high_signal_limit_1;
   uint16_t low_signal_limit_1;
   uint16_t high_signal_limit_2;
   uint16_t low_signal_limit_2;
} __attribute__ ((packed));

/* PROCI_TWOTOUCHGESTUREPROCESSOR_T27 */
struct qtm_proci_two_touch_gesture_proc_cfg
{
   uint8_t  ctrl;
   uint8_t  max_num;
   uint8_t  reserved1;
   uint8_t  gesture_enable;
   uint8_t  rotate_threshold;
   uint16_t zoom_threshold;
} __attribute__ ((packed));

/* SPT_CTECONFIG_T28 */
struct qtm_spt_cte_config_cfg
{
   uint8_t  ctrl;
   uint8_t  command;
   uint8_t  mode;
   uint8_t  idle_gcaf_depth;
   uint8_t  active_gcaf_depth;
   uint8_t  voltage;
} __attribute__ ((packed));

/* SPT_USERDATA_T38 */
struct qtm_spt_userdata_cfg
{
   uint8_t data_0;
   uint8_t data_1;
   uint8_t data_2;
   uint8_t data_3;
   uint8_t data_4;
   uint8_t data_5;
   uint8_t data_6;
   uint8_t data_7;
} __attribute__ ((packed));

/*******************************/
/******** platform data ********/
/*******************************/

/* Virtual keys */
struct vkey
{
   int  code;
   int  center_x;
   int  center_y;
   int  width;
   int  height;
};

struct virt_keys
{
   int          count;
   struct vkey  *keys;
};

/* key arrays */
struct qtouch_key
{
   uint8_t  channel;
   int      code;
};

struct qtouch_key_array
{
   struct qtm_touch_keyarray_cfg  *cfg;
   struct qtouch_key              *keys;
   int                            num_keys;
};

/* flags */
#define QTOUCH_FLIP_X           (1 << 0)
#define QTOUCH_FLIP_Y           (1 << 1)
#define QTOUCH_SWAP_XY          (1 << 2)
#define QTOUCH_USE_MULTITOUCH   (1 << 3)
#define QTOUCH_USE_KEYARRAY     (1 << 4)
#define QTOUCH_CFG_BACKUPNV     (1 << 5)
#define QTOUCH_EEPROM_CHECKSUM  (1 << 6)
#define QTOUCH_USE_TINYTOUCH    (1 << 7)

struct qtouch_ts_platform_data
{
   uint32_t             flags;
   unsigned long        irqflags;

   /* checksum for Information Block - 3 bytes*/
   uint8_t              nv_checksum[3];

   uint32_t             abs_min_x;
   uint32_t             abs_max_x;
   uint32_t             abs_min_y;
   uint32_t             abs_max_y;
   uint32_t             abs_min_p;
   uint32_t             abs_max_p;
   uint32_t             abs_min_w;
   uint32_t             abs_max_w;
   uint32_t             fuzz_x;
   uint32_t             fuzz_y;
   uint32_t             fuzz_p;
   uint32_t             fuzz_w;

   int                  (*hw_reset)(void);

   /* TODO: allow multiple key arrays */
   struct qtouch_key_array                      key_array;

   /* object configuration information from board */
   struct qtm_gen_power_cfg                     power_cfg;
   struct qtm_gen_acquire_cfg                   acquire_cfg;
   struct qtm_touch_multi_cfg                   multi_touch_cfg;
   struct qtm_touch_multi_cfg                   multi_touch_cfg2;
   struct qtm_spt_commsconfig_cfg               commsconfig_cfg;
   struct qtm_spt_gpio_pwm_cfg                  gpio_pwm_cfg;
   struct qtm_proci_grip_suppression_cfg        grip_suppression_cfg;
   struct qtm_procg_noise_suppression_cfg       noise_suppression_cfg;
   struct qtm_procg_noise_suppression_cfg       noise_suppression_cfg2;
   struct qtm_touch_proximity_cfg               touch_proximity_cfg;
   struct qtm_proci_one_touch_gesture_proc_cfg  one_touch_gesture_proc_cfg;
   struct qtm_spt_self_test_cfg	                self_test_cfg;
   struct qtm_proci_two_touch_gesture_proc_cfg  two_touch_gesture_proc_cfg;
   struct qtm_spt_cte_config_cfg                cte_config_cfg;
   struct qtm_spt_userdata_cfg                  userdata_cfg;

   struct virt_keys                             vkeys;
};


#define QTOUCH_I2C_NAME "touchpad"


/*! @brief  This enum describes Firmware Update completed or not */
enum QTOUCH_FW_UPDATE_STATUS_T
{
   QTOUCH_FW_UPDATE_NOT_DONE,
   QTOUCH_FW_UPDATE_DONE,
};


//enum MOT_TOUCH_BL_MODE_STATE_T
typedef enum
{
    /* Start at -20 to allow up to 20 error codes */
    MOT_TOUCH_BL_I2C_WRITE_ERROR = -20,
    MOT_TOUCH_BL_COPY_DATA_ERROR,
    MOT_TOUCH_BL_I2C_READ_ERROR,
    MOT_TOUCH_BL_DATA_WRITE_ERROR,
    MOT_TOUCH_BL_AFTER_CRC_FAIL,
    MOT_TOUCH_BL_GOT_BAD_CRC,
    MOT_TOUCH_BL_FW_VERSION_READ_ERROR,
    MOT_TOUCH_BL_APP_BAD_CRC,
    /* Status of the Boot loader allow up to 10 status messages */
    MOT_TOUCH_BL_WAITING_FOR_NOTHING = 0,
    MOT_TOUCH_BL_WAITING_FOR_STATUS,
    MOT_TOUCH_BL_WAITING_FOR_DATA,
    MOT_TOUCH_BL_WAITING_FOR_CRC,
    MOT_TOUCH_BL_AFTER_CRC_PASS,

    MOT_TOUCH_BL_FW_UPDATE_SUCCESS = 10,
    MOT_TOUCH_BL_FW_NOT_STARTED = 0xFF
} MOT_TOUCH_BL_MODE_STATE_T;

#define QTOUCH_READ_DATA_SIZE   8


/* Status of IC. */
#define MOT_TOUCH_MODE_UNKNOWN		0
#define MOT_TOUCH_MODE_NORMAL		1
#define MOT_TOUCH_MODE_BOOTLOADER	2
#define MOT_TOUCH_MODE_CRASH		3


#define MOT_TOUCH_IOCTL_BASE          88

/*! @brief Get the IOCTL command depending on IOCTL class and offset */
#define MOT_TOUCH_GET_IOCTL_OFFSET(class, offset) (class+offset)

/*! @brief  Various IOCTL classes for Touch IC are
* DEBUG: IOCTLS to retrieve any debug information from the driver
* BOOT_SUPPORT: IOCTLS used to perform firmware upgrade
* STATUS: IOCTLS used to obtain the statusof the IC/driver
* COMMAND: IOCTLS used to command the IC to perform some operation
* (in NORMAL mode)
*/
#define MOT_TOUCH_IOCTL_CLASS_DEBUG                00
#define MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT         20
#define MOT_TOUCH_IOCTL_CLASS_STATUS               40
#define MOT_TOUCH_IOCTL_CLASS_COMMAND              60

/* IOCTL codes */
/*! @brief  IOCTL codes for COMMAND class  */
/*! @brief IOCTL to enable touchscreen
 * IOCTL name: MOT_TOUCH_IOCTL_ENABLE_TOUCH
 * IOCTL description: This ioctl is used to enable the touchpad.
 * Used by Eventhub to enable touchscreen.
 * IOCTL example usage:
 *     ioctl(handle,MOT_TOUCH_IOCTL_ENABLE_TOUCH)
 */
#define MOT_TOUCH_IOCTL_ENABLE_TOUCH _IO(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND, 1))

/*! @brief IOCTL to disable touchscreen
 * IOCTL name: MOT_TOUCH_IOCTL_DISABLE_TOUCH
 * IOCTL description: This ioctl is used to disable the touchpad.
 * Used by Eventhub to disable touchscreen.
 * IOCTL example usage:
 *     ioctl(handle,MOT_TOUCH_IOCTL_DISABLE_TOUCH)
 */
#define MOT_TOUCH_IOCTL_DISABLE_TOUCH _IO(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND, 2))

/*! @brief IOCTL to start the calibration of Touch IC
 * IOCTL name: MOT_TOUCH_IOCTL_IC_CALIBRATION
 * IOCTL description: This ioctl is used to start
 * the calibration of touch IC.
 * Used by Eventhub to start calibration..
 * IOCTL example usage:
 *     ioctl(handle,MOT_TOUCH_IOCTL_IC_CALIBRATION)
 */
#define MOT_TOUCH_IOCTL_IC_CALIBRATION _IO(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND, 3))

/*! @brief IOCTL to set the current touch mode
 * IOCTL name: MOT_TOUCH_IOCTL_SET_TOUCH_MODE
 * IOCTL description: This ioctl is used to set
 * one of the following  touch mode.
 * KEY08_DONT_CHANGE_MODE  -2
 * KEY08_STANDBY           -1
 * KEY08_INITIAL_MODE       0
 * IOCTL example usage:
 *     unsigned int char_reg = KEY08_STANDBY;
 *     ioctl(handle,MOT_TOUCH_IOCTL_SET_TOUCH_MODE,&char_reg)
 */
#define MOT_TOUCH_IOCTL_SET_TOUCH_MODE _IOW(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND, 4), int*)

/*! @brief IOCTL to set the the irq
 * IOCTL name: MOT_TOUCH_IOCTL_SET_IRQ
 * IOCTL description: This ioctl is used to set
 * the irq.
 * enable irq  1
 * disable irq 2
 * IOCTL example usage:
 *     char char_reg = 1;
 *     ioctl(handle,MOT_TOUCH_IOCTL_SET_IRQ,&char_reg)
 */

#define MOT_TOUCH_IOCTL_SET_IRQ _IOW(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_COMMAND, 5), int*)

/*! @brief  IOCTL codes for STATUS class  */

/*! @brief IOCTL to get the current touch mode
 * IOCTL name: MOT_TOUCH_IOCTL_GET_TOUCH_MODE
 * IOCTL description: This ioctl is used to get
 * the current touch mode, which is
 *   one of the following:
 * KEY08_DONT_CHANGE_MODE  -2
 * KEY08_STANDBY           -1
 * KEY08_INITIAL_MODE       0
 * IOCTL example usage:
 *     unsigned int char_reg;
 *     ioctl(handle,MOT_TOUCH_IOCTL_GET_TOUCH_MODE,&char_reg)
 *     // char_reg contains the current touch mode
 */
#define MOT_TOUCH_IOCTL_GET_TOUCH_MODE _IOR(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_STATUS, 1), int*)

/*! @brief IOCTL to get the Firmware version
 * IOCTL name: MOT_TOUCH_IOCTL_GET_FW_VERSION
 * IOCTL description: This ioctl is used to get
 * the current Firmware version
 * IOCTL example usage:
 *     unsigned int char_reg;
 *     ioctl(handle,MOT_TOUCH_IOCTL_GET_FW_VERSION,&char_reg)
 *     //char_reg contains the current firmware version
 */
#define MOT_TOUCH_IOCTL_GET_FW_VERSION _IOR(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_STATUS, 2), int*)

/*! @brief  IOCTL codes for DEBUG  class  */

/*! @brief IOCTL to get current touch data
 * IOCTL name: MOT_TOUCH_IOCTL_GET_TOUCH_DATA:
 * IOCTL description: This IOCTL is used to get the current touch data
 * from the kernel to user space.
 * IOCTL example usage:
 *     MOT_TS_DATA_T ts_data;
 *     ioctl(handle,MOT_TOUCH_IOCTL_GET_TOUCH_DATA,&ts_data)
 *     // ts_data contains the current touch data
 */
#define MOT_TOUCH_IOCTL_GET_TOUCH_DATA _IOR(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_DEBUG, 1),\
 struct MOTO_TS_DATA*)

/*! @brief  IOCTL codes for BOOT_SUPPORT  class  */

/*! @brief IOCTL to get the state of driver in boot mode
 * IOCTL name: MOT_TOUCH_IOCTL_BL_GET_STATUS
 * IOCTL description: This ioctl is used to get
 * the state of driver in boot mode which
 * is of type MOT_TOUCH_BL_MODE_STATE_T
 * IOCTL example usage:
 *     MOT_TOUCH_BL_MODE_STATE_T char_reg;
 *     ioctl(handle,MOT_TOUCH_IOCTL_BL_GET_STATUS,&char_reg)
 *     // char_reg contains the current boot mode
 */
#define MOT_TOUCH_IOCTL_BL_GET_STATUS _IOR(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT, 1), int*)

/*! @brief get the IC mode
* IOCTL name: MOT_TOUCH_IOCTL_GET_IC_MODE
* IOCTL Description: This ioctl is used to get
* the current mode of the IC, which can be
* one of the following:
* MOT_TOUCH_MODE_UNKNOWN       0
* MOT_TOUCH_MODE_NORMAL        1
* MOT_TOUCH_MODE_BOOTLOADER    2
* IOCTL example usage:
*     unsigned int char_reg;
*     ioctl(handle,MOT_TOUCH_IOCTL_GET_IC_MODE,&char_reg)
*     // char_reg contains the current IC mode
*/
#define MOT_TOUCH_IOCTL_GET_IC_MODE _IOR(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT, 2), int)

/*! @brief IOCTL code for  putting the touch IC into boot mode
* IOCTL name : MOT_TOUCH_SET_BOOT_MODE:
* IOCTL Description: This IOCTL is used to set touch IC
* to BOOT mode.  The parameter is
* used to decide if the entire boot sequence
* has to be executed or only the unlock
* command has to be sent to the IC.
* IC will expect only the unlock command if it has
* found a CRC error in the existing application.
* example usage:
*     unsigned int boot_from_unlock = 1; // send only unlock command
*     ioctl(handle,MOT_TOUCH_SET_BOOT_MODE,&boot_from_unlock)
*/

#define MOT_TOUCH_IOCTL_SET_BOOT_MODE _IO(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT, 3))

/*! @brief IOCTL code for  putting the touch IC into normal mode
* IOCTL name : MOT_TOUCH_IOCTL_SET_NORMAL_MODE
* IOCTL Description: This IOCTL is used to set touch IC to NORMAL_MODE mode.
* example usage:
*     ioctl(handle,MOT_TOUCH_IOCTL_SET_NORMAL_MODE)
*/

#define MOT_TOUCH_IOCTL_SET_NORMAL_MODE _IO(MOT_TOUCH_IOCTL_BASE,\
 MOT_TOUCH_GET_IOCTL_OFFSET(MOT_TOUCH_IOCTL_CLASS_BOOT_SUPPORT, 4))

#endif /* _LINUX_QTOUCH_OBP_TS_H */

#ifdef __cplusplus
}
#endif


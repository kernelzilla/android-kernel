/* linux/arch/arm/mach-msm/board-mot.h
 *
 *  Copyright (C) 2009 Motorola, Inc.
 *  Author: Adam Zajac <Adam.Zajac@motorola.com>
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
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_MOT_H
#define __ARCH_ARM_MACH_MSM_BOARD_MOT_H

typedef enum {
    NOT_SUPPORTED = 0,
	M2 = 1,
	P2 = 2,
	P3 = 3
} T_MOT_HW_TYPE_EN;

/* # of modem rpc servers that must register before kernel init continues */
#define MSM_SERVER_SYNC_COUNT 27

#if defined(CONFIG_MACH_CALGARY)

#define MSM_PMEM_MDP_SIZE	0x00800000
#define MSM_PMEM_CAMERA_SIZE    0x00A00000
#define MSM_PMEM_ADSP_SIZE      0x01400000 // Required for RAW camera mode
#define MSM_PMEM_GPU1_SIZE	0x00800000
#define MSM_FB_SIZE		0x00200000 //we used to set it to 0x800000 - TBD
#define MSM_GPU_PHYS_SIZE	SZ_2M
#define PMEM_KERNEL_EBI1_SIZE	0x200000

/* Note, the following SCL defines should match the BR targ<>.h file */
/* IMPORTANT: Values must match those of the targ<BUILD>.h files in the AMSS */
#define SCL_APPS_BOOT_BASE 0
#define SCL_APPS_BOOT_SIZE 0x0100000
#define SCL_APPS_BOOT_SIZE 0x0100000
#define SCL_APPS_CODE_BASE 0x00200000
#define SCL_APPS_TOTAL_SIZE 0xA200000
#define SCL_MM_HEAP2_BASE 0x0A400000
#define SCL_MM_HEAP2_SIZE 0x4000000


#define TPO_NV_RST_VREG_DELAY 5 /* ms */

/* AP side GPIO map */
#define CAM_PWRDN                 0
#define CAM_RST_N                 1 
#define SD_DETECT_N_SIGNAL       17
#define LIS331DLH_INT1           18
#define WLAN_HOST_IRQ            19        
#define LIS331DLH_INT2           20
#define QWERTY_INT_N_SIGNAL      21
#define EMU_ID_PROT              23
#define HAPTICS_MSM_PWM          27
#define	SFH7743_GPIO_INTR        29
#define WLAN_REG_ON_SIGNAL       33
#define HAPTICS_AMP_EN        	 34
#define MSG_IND_LED_CTRL       	 35
#define LM3535_INT_N_SIGNAL      36
#define SLIDE_DETECT             42
#define I2C_SCL                  60
#define I2C_SDA                  61
#define DIG_COMP_INT_N           83
#define SFH7743_GPIO_EN          88 //FIXME: Using TourmalineP1 GPIO here; IR_PROX_EN does not seem to be defined for the P2 - follow up
#define LCD_HW_VSYNC             97
#define QWRTY_RST_N             107
#define WLAN_RST_N   	        108
#define DIG_COMP_RST_N          109
#define WLAN_WAKE_SIGNAL        111
#define BT_EXT_WAKE_SIGNAL      113
#define BT_HOST_WAKE_SIGNAL     114
#define BT_REG_ON_SIGNAL        115
#define BT_RESET_N_SIGNAL       116
#define TOUCH_RST_N             117
#define DISP_BL_SEL             123   /* used in case of flip detection */
#define	TOUCH_INT_N             124
#define LCD_RST_N_SIGNAL        128

#define CRUCIALTEC_OJ_NAME              "optical_joystick"
#define CRUCIALTEC_OJ_I2C_ADDR          0x33
#define CRUCIALTEC_OJ_MOTION_SIGNAL     92
#define CRUCIALTEC_OJ_NRST_SIGNAL       91
#define CRUCIALTEC_OJ_SHUTDOWN_SIGNAL   90

#define ADP8860_I2C_ADDR        0x2A
#define ADP8870_I2C_ADDR        0x2B
#define LM3535_I2C_ADDR         0x36
#define LM3535_REV6_I2C_ADDR    0x38
#define	KEY08_I2C_ADDR          0x11
#define	KEY08_BL_I2C_ADDR       0x5F
#define LIS331DLH_I2C_ADDR      0x18
#define ADP5588_I2C_ADDR        0x34
#define AKM8973_I2C_ADDR        0x1C

#define MOT_HW_REV              "Calgary P3"
#define MOT_SERIAL_NUM          "A555"
#define MOT_PRODUCT_STR         'M', 'o', 't', 'o', 'r', 'o', 'l', 'a', ' ', 'A', '5', '5', '5',
#define MOT_SERIAL_STR          'A', '5', '5', '5',
#define MOT_VID					0x22B8

#define MOT_PID					MOT_PID4 // Default
#define MOT_PID0				0x2d62 //Android Single ADB Device
#define MOT_PID1				0x2d71 //LAN+Modem+QC+MTP
#define MOT_PID2				0x2d72 //MSD
#define MOT_PID3				0x2d73 //MTP
#define MOT_PID4				0x2d74 //LAN+Modem+QC+MTP+ADB
#define MOT_PID5				0x2d75 //MSD+QC+ADB
#define MOT_PID6				0x2d76 //MTP+QC+ADB
#define MOT_PID7				0x2d77 //Flash
#define MOT_PID8				0x2d78 //CDROM
#define MOT_PID9				0x2d79 //LAN+QC

/* USB Mode Switch commands (used by PC tool to switch) */
#define MOT_USB_CONFIG_34       34     // 0x2d71 LAN+Modem+QC+MTP
#define MOT_USB_CONFIG_14       14     // 0x2d72 MSD  & 0x2d78 CD-ROM
#define MOT_USB_CONFIG_15       15     // 0x2d73 MTP
#define MOT_USB_CONFIG_35       35     // 0x2d74 LAN+Modem+QC+MTP+ADB
#define MOT_USB_CONFIG_36       36     // 0x2d75 MSD+QC+ADB
#define MOT_USB_CONFIG_37       37     // 0x2d76 MTP+QC+ADB
#define MOT_USB_CONFIG_47       47     // MOT_PID9

#elif defined(CONFIG_MACH_PITTSBURGH)

#define MSM_PMEM_MDP_SIZE	0x00800000
#define MSM_PMEM_CAMERA_SIZE    0x00A00000
#define MSM_PMEM_ADSP_SIZE      0x01400000 // Required for RAW camera mode
#define MSM_PMEM_GPU1_SIZE	0x00800000
#define MSM_FB_SIZE		0x00200000 //we used to set it to 0x800000 - TBD
#define MSM_GPU_PHYS_SIZE	SZ_2M
#define PMEM_KERNEL_EBI1_SIZE	0x200000

/* Note, the following SCL defines should match the BR targ<>.h file */
/* IMPORTANT: Values must match those of the targ<BUILD>.h files in the AMSS */
#define SCL_APPS_BOOT_BASE 0
#define SCL_APPS_BOOT_SIZE 0x0100000
#define SCL_APPS_BOOT_SIZE 0x0100000
#define SCL_APPS_CODE_BASE 0x00200000
#define SCL_APPS_TOTAL_SIZE 0xA200000
#define SCL_MM_HEAP2_BASE 0x0A400000
#define SCL_MM_HEAP2_SIZE 0x4000000


#define TPO_NV_RST_VREG_DELAY 5 /* ms */

/* AP side GPIO map */
#define CAM_PWRDN                 0
#define CAM_RST_N                 1 
#define SD_DETECT_N_SIGNAL       17
#define LIS331DLH_INT1           18
#define WLAN_HOST_IRQ            19        
#define LIS331DLH_INT2           20
#define QWERTY_INT_N_SIGNAL      21
#define EMU_ID_PROT              23
#define HAPTICS_MSM_PWM          27
#define	SFH7743_GPIO_INTR        29
#define WLAN_REG_ON_SIGNAL       82
#define HAPTICS_AMP_EN        	 26
//#define MSG_IND_LED_CTRL       	 35 /* Pittsburgh doesn't use this */
#define LM3535_INT_N_SIGNAL      76
#define SLIDE_DETECT             42
#define I2C_SCL                  60
#define I2C_SDA                  61
#define DIG_COMP_INT_N           83
#define SFH7743_GPIO_EN          88 //FIXME: Using TourmalineP1 GPIO here; IR_PROX_EN does not seem to be defined for the P2 - follow up
#define LCD_HW_VSYNC             97
#define QWRTY_RST_N             107
#define WLAN_RST_N   	        108
#define DIG_COMP_RST_N          109
#define WLAN_WAKE_SIGNAL        111
#define BT_EXT_WAKE_SIGNAL      113
#define BT_HOST_WAKE_SIGNAL     114
#define BT_REG_ON_SIGNAL        115
#define BT_RESET_N_SIGNAL       116
#define TOUCH_RST_N             117
#define DISP_BL_SEL             123   /* used in case of flip detection */
#define	TOUCH_INT_N             124
#define LCD_RST_N_SIGNAL        128

#define CRUCIALTEC_OJ_NAME              "optical_joystick"
#define CRUCIALTEC_OJ_I2C_ADDR          0x33
#define CRUCIALTEC_OJ_MOTION_SIGNAL     92
#define CRUCIALTEC_OJ_NRST_SIGNAL       91
#define CRUCIALTEC_OJ_SHUTDOWN_SIGNAL   90

#define ADP8860_I2C_ADDR        0x2A
#define ADP8870_I2C_ADDR        0x2B
#define LM3535_I2C_ADDR         0x36
#define LM3535_REV6_I2C_ADDR    0x38
#define	KEY08_I2C_ADDR          0x11
#define	KEY08_BL_I2C_ADDR       0x5F
#define LIS331DLH_I2C_ADDR      0x18
#define ADP5588_I2C_ADDR        0x34
#define AKM8973_I2C_ADDR        0x1C

#define MOT_HW_REV             "P0"
#define MOT_SERIAL_NUM         "A555"
#define MOT_VID					0x22B8

#define MOT_PID					MOT_PID4 // Default
#define MOT_PID0				0x2d62 //Android Single ADB Device
#define MOT_PID1				0x2d71 //LAN+Modem+QC+MTP
#define MOT_PID2				0x2d72 //MSD
#define MOT_PID3				0x2d73 //MTP
#define MOT_PID4				0x2d74 //LAN+Modem+QC+MTP+ADB
#define MOT_PID5				0x2d75 //MSD+QC+ADB
#define MOT_PID6				0x2d76 //MTP+QC+ADB
#define MOT_PID7				0x2d77 //Flash
#define MOT_PID8				0x2d78 //CDROM
#define MOT_PID9				0x2d79 //LAN+QC

/* USB Mode Switch commands (used by PC tool to switch) */
#define MOT_USB_CONFIG_34       34     // 0x2d71 LAN+Modem+QC+MTP
#define MOT_USB_CONFIG_14       14     // 0x2d72 MSD  & 0x2d78 CD-ROM
#define MOT_USB_CONFIG_15       15     // 0x2d73 MTP
#define MOT_USB_CONFIG_35       35     // 0x2d74 LAN+Modem+QC+MTP+ADB
#define MOT_USB_CONFIG_36       36     // 0x2d75 MSD+QC+ADB
#define MOT_USB_CONFIG_37       37     // 0x2d76 MTP+QC+ADB

#elif defined(CONFIG_MACH_SOCIAL)
#define MSM_PMEM_MDP_SIZE	0x00800000
#define MSM_PMEM_CAMERA_SIZE    0x00A00000
#define MSM_PMEM_ADSP_SIZE      0x01400000 // Required for RAW camera mode
#define MSM_PMEM_GPU1_SIZE	0x00800000
#define MSM_FB_SIZE		0x00200000 //we used to set it to 0x800000 - TBD
#define MSM_GPU_PHYS_SIZE	SZ_2M
#define PMEM_KERNEL_EBI1_SIZE	0x200000
/* Note, the following SCL defines should match the BR targ<>.h file */
/* IMPORTANT: Values must match those of the targ<BUILD>.h files in the AMSS */
#define SCL_APPS_BOOT_BASE 0
#define SCL_APPS_BOOT_SIZE 0x0100000
#define SCL_APPS_BOOT_SIZE 0x0100000
#define SCL_APPS_CODE_BASE 0x13600000
#define SCL_APPS_TOTAL_SIZE (0x30000000-SCL_APPS_CODE_BASE-SCL_MM_HEAP2_SIZE)
#define SCL_MM_HEAP2_BASE (0x30000000-SCL_MM_HEAP2_SIZE)
#define SCL_MM_HEAP2_SIZE 0x4000000


#define TPO_NV_RST_VREG_DELAY 5 /* ms */

/* AP side GPIO map */
#define CAM_PWRDN                 0
#define CAM_RST_N                 1 
#define SD_DETECT_N_SIGNAL       17
#define KXTF_INT_N               18
#define WLAN_HOST_IRQ            19        
#define LIS331DLH_INT2           20
#define QWERTY_INT_N_SIGNAL      21
#define EMU_ID_PROT              23
#define HAPTICS_MSM_PWM          27
#define	SFH7743_GPIO_INTR        29
#define ISL29011_INT_N_SIGNAL    29
#define WLAN_REG_ON_SIGNAL       82
#define HAPTICS_AMP_EN        	 26
//#define MSG_IND_LED_CTRL       	 35 /* Pittsburgh doesn't use this */
#define LM3535_INT_N_SIGNAL      76
#define SLIDE_DETECT             42
#define I2C_SCL                  60
#define I2C_SDA                  61
#define DIG_COMP_INT_N           83
#define SFH7743_GPIO_EN          88 //FIXME: Using TourmalineP1 GPIO here; IR_PROX_EN does not seem to be defined for the P2 - follow up
#define DOCK_DETECT_INT          93
#define LCD_HW_VSYNC             97
#define QWRTY_RST_N             107
#define WLAN_RST_N   	        108
#define DIG_COMP_RST_N          109
#define WLAN_WAKE_SIGNAL        111
#define BT_EXT_WAKE_SIGNAL      113
#define BT_HOST_WAKE_SIGNAL     114
#define BT_REG_ON_SIGNAL        115
#define BT_RESET_N_SIGNAL       116
#define TOUCH_RST_N             117
#define DISP_BL_SEL             123   /* used in case of flip detection */
#define	TOUCH_INT_N             124
#define LCD_RST_N_SIGNAL        128

#define CRUCIALTEC_OJ_NAME              "optical_joystick"
#define CRUCIALTEC_OJ_I2C_ADDR          0x33
#define CRUCIALTEC_OJ_MOTION_SIGNAL     92
#define CRUCIALTEC_OJ_NRST_SIGNAL       91
#define CRUCIALTEC_OJ_SHUTDOWN_SIGNAL   90

#define ADP8860_I2C_ADDR        0x2A
#define ADP8870_I2C_ADDR        0x2B
#define LM3535_I2C_ADDR         0x36
#define LM3535_REV6_I2C_ADDR    0x38
#define ISL29011_I2C_ADDR       0x44
/* Added for Social AT42QT602240*/
#define MXT224_I2C_ADDR         0x4A
#define MXT224_BL_I2C_ADDR      0x24
/* Ended */
#define	KEY08_I2C_ADDR          0x11
#define	KEY08_BL_I2C_ADDR       0x5F
#define LIS331DLH_I2C_ADDR      0x18
#define ADP5588_I2C_ADDR        0x34
#define AKM8973_I2C_ADDR        0x1C

#define MOT_HW_REV             "Social P2"
#define MOT_SERIAL_NUM         "MB512"
#define MOT_VID					0x22B8

#define MOT_PID					MOT_PID4 // Default
#define MOT_PID0				0x2d62 //Android Single ADB Device
#define MOT_PID1				0x2d71 //LAN+Modem+QC+MTP
#define MOT_PID2				0x2d72 //MSD
#define MOT_PID3				0x2d73 //MTP
#define MOT_PID4				0x2d74 //LAN+Modem+QC+MTP+ADB
#define MOT_PID5				0x2d75 //MSD+QC+ADB
#define MOT_PID6				0x2d76 //MTP+QC+ADB
#define MOT_PID7				0x2d77 //Flash
#define MOT_PID8				0x2d78 //CDROM
#define MOT_PID9				0x2d79 //LAN+QC

/* USB Mode Switch commands (used by PC tool to switch) */
#define MOT_USB_CONFIG_34       34     // 0x2d71 LAN+Modem+QC+MTP
#define MOT_USB_CONFIG_14       14     // 0x2d72 MSD  & 0x2d78 CD-ROM
#define MOT_USB_CONFIG_15       15     // 0x2d73 MTP
#define MOT_USB_CONFIG_35       35     // 0x2d74 LAN+Modem+QC+MTP+ADB
#define MOT_USB_CONFIG_36       36     // 0x2d75 MSD+QC+ADB
#define MOT_USB_CONFIG_37       37     // 0x2d76 MTP+QC+ADB
#define MOT_USB_CONFIG_38       38     /* 0x2d74 LAN+Modem+QC+MSD+ADB */
#define MOT_USB_CONFIG_39       39     /* 0x2d76 LAN+Modem+QC */
#define MOT_USB_CONFIG_47       47     // MOT_PID9

#elif defined(CONFIG_MACH_CIENA)

#define MSM_FB_SIZE             0x00200000
#define MSM_PMEM_MDP_SIZE       0x00800000
#define MSM_PMEM_ADSP_SIZE      0x01400000
#define MSM_PMEM_CAMERA_SIZE    0x00A00000

/* Note, the following SCL defines should match the BR targ<>.h file */
/* IMPORTANT: Values must match those of the targ<BUILD>.h files in the AMSS */
#define SCL_APPS_BOOT_BASE 0
#define SCL_APPS_BOOT_SIZE 0x0100000
#define SCL_APPS_CODE_BASE 0x00200000
#define SCL_APPS_TOTAL_SIZE 0x8B00000
#define SCL_MM_HEAP2_BASE 0x08D00000
#define SCL_MM_HEAP2_SIZE 0x4000000

#define TPO_NV_RST_DELAY 10 /* ms */

/* AP side GPIO map */
#define CAM_PWRDN                 0
#define CAM_RST_N                 1
#define SD_DETECT_N_SIGNAL       17
#define LIS331DLH_INT1           18
#define WLAN_HOST_IRQ            19
#define LIS331DLH_INT2           20
#define QWERTY_INT_N_SIGNAL      21
#define EMU_ID_PROT              23
#define HAPTICS_MSM_PWM          27
#define	SFH7743_GPIO_INTR        29
#define WLAN_REG_ON_SIGNAL       33
#define HAPTICS_AMP_EN        	 34
#define MSG_IND_LED_CTRL       	 35
#define LM3535_INT_N_SIGNAL      36
#define SLIDE_DETECT             42
#define I2C_SCL                  60
#define I2C_SDA                  61
#define DIG_COMP_INT_N           83
#define SFH7743_GPIO_EN          88 //FIXME: Using TourmalineP1 GPIO here; IR_PROX_EN does not seem to be defined for the P2 - follow up
#define LCD_HW_VSYNC             97
#define QWRTY_RST_N             107
#define WLAN_RST_N   	        108
#define DIG_COMP_RST_N          109
#define WLAN_WAKE_SIGNAL        111
#define BT_EXT_WAKE_SIGNAL      113
#define BT_HOST_WAKE_SIGNAL     114
#define BT_REG_ON_SIGNAL        115
#define BT_RESET_N_SIGNAL       116
#define TOUCH_RST_N             117
#define	TOUCH_INT_N             124
#define LCD_RST_N_SIGNAL        128

#define CRUCIALTEC_OJ_NAME              "optical_joystick"
#define CRUCIALTEC_OJ_I2C_ADDR          0x33
#define CRUCIALTEC_OJ_MOTION_SIGNAL     92
#define CRUCIALTEC_OJ_NRST_SIGNAL       91
#define CRUCIALTEC_OJ_SHUTDOWN_SIGNAL   90

#define LM3535_I2C_ADDR         0x36
#define LM3535_REV6_I2C_ADDR    0x38
#define	KEY08_I2C_ADDR          0x11
#define	KEY08_BL_I2C_ADDR       0x5F
#define LIS331DLH_I2C_ADDR      0x18
#define ADP5588_I2C_ADDR        0x34
#define AKM8973_I2C_ADDR        0x1C

#define MOT_HW_REV              "Calgary P3"
#define MOT_HW_REV_DEF          P3
#define MOT_SERIAL_NUM          "A555"
#define MOT_PRODUCT_STR         'M', 'o', 't', 'o', 'r', 'o', 'l', 'a', ' ', 'A', '5', '5', '5',
#define MOT_SERIAL_STR          'A', '5', '5', '5',
#define MOT_VID					0x22B8

#define MOT_PID					MOT_PID4 /* Default */
#define MOT_PID0				0x2d62 //Android Single ADB Device
#define MOT_PID1				0x2d71 //LAN+Modem+QC+MTP
#define MOT_PID2				0x2d72 //MSD
#define MOT_PID3				0x2d73 //MTP
#define MOT_PID4				0x2d74 //LAN+Modem+QC+MTP+ADB
#define MOT_PID5				0x2d75 //MSD+QC+ADB
#define MOT_PID6				0x2d76 //MTP+QC+ADB
#define MOT_PID7				0x2d77 //Flash
#define MOT_PID8				0x2d78 //CDROM
#define MOT_PID9				0x2d79 //LAN+QC

/* USB Mode Switch commands (used by PC tool to switch) */
#define MOT_USB_CONFIG_34       34     // 0x2d71 LAN+Modem+QC+MTP
#define MOT_USB_CONFIG_14       14     // 0x2d72 MSD  & 0x2d78 CD-ROM
#define MOT_USB_CONFIG_15       15     // 0x2d73 MTP
#define MOT_USB_CONFIG_35       35     // 0x2d74 LAN+Modem+QC+MTP+ADB
#define MOT_USB_CONFIG_36       36     // 0x2d75 MSD+QC+ADB
#define MOT_USB_CONFIG_37       37     // 0x2d76 MTP+QC+ADB
#define MOT_USB_CONFIG_47       47     // MOT_PID9
#else 

#error "Hardware NOT supported!"

#endif /* defined(CONFIG_MACH_PITTSBURGH) */

#define MOT_PID_L				(MOT_PID0 & 0x00FF)
#define MOT_PID_H				((MOT_PID0 & 0xFF00) >> 8)
#define MOT_VID_L				(MOT_VID & 0x00FF)
#define MOT_VID_H				((MOT_VID & 0xFF00) >> 8)



int mot_get_hw_type(void);
void mot_lcd_power(unsigned on);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_MOT_H */

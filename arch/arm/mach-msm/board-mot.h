#ifdef CONFIG_MSM_STACKED_MEMORY
#define MSM_SMI_BASE			0x100000
#define MSM_SMI_SIZE			0x700000

#define MSM_PMEM_GPU0_BASE		MSM_SMI_BASE
#define MSM_PMEM_GPU0_SIZE		0x700000
#endif

#define MSM_PMEM_MDP_BASE		0
#define MSM_PMEM_MDP_SIZE		0x800000 // QCom wanted 0xC00000
#define MSM_PMEM_ADSP_BASE		SMI_MEM_OFFSET // Discarded for Motus
#define MSM_PMEM_ADSP_SIZE		0xC00000   // adjusted at runtime for motus
#define	MSM_PMEM_ADSP_FLIP_SIZE 0x800000
#define MSM_PMEM_GPU1_SIZE		0x800000
#define MSM_FB_BASE				SMI_MEM_OFFSET + MSM_PMEM_ADSP_SIZE
#define MSM_FB_SIZE				0x96000	// 0x800000 in 6365
#define MSM_RAMCONSOLE_SIZE		0x020000

/*
 * This must match its namesake on the AMSS side
 */
#define	SCL_EBI1_MODEM_CODE_SIZE 	0x01300000

/*
 * This must point to the "free" area in SMI
 * which is top 32M, if we have 64M package.
 * The top end of top 32M will be used for
 * AMSS (19M), the bottom for framebuffer (8M).
 * Area in between can be used for any static
 * allocations, such as Android RAM console,
 * or anything else we need (shared mem, etc).
 */
#define SMI_MEM_OFFSET				0x02000000

/* 
 * Static allocations are now handled at runtime, 
 * but some initializers still refer to MSM_xxx_BASE.
 * They can be just 0. Keep these for now to make it
 * easier to fallback to Google's display stuff if needed.
 */
#define MSM_PMEM_GPU1_BASE		0
#define MSM_RAMCONSOLE_BASE		0

#ifndef TRUE
 #define TRUE (1)
#endif

#ifndef FALSE
 #define FALSE (0)
#endif

//typedef unsigned int				uint32;
//typedef unsigned int				boolean;

/* AP side GPIO map */
#define SD_DETECT_N_SIGNAL			17

#define LIS331DLH_I2C_ADDR			0x18
#define LIS331DLH_INT2				33

#define ADP5588_I2C_ADDR			0x34

#define LM3535_INT_N_SIGNAL     	31
#define LM3535_I2C_ADDR         	0x38

#define ADP8862_INT_N_SIGNAL     	31
#define ADP8862_I2C_ADDR         	0x2A

#define	KEY08_I2C_ADDR				0x11
#define	KEY08_BL_I2C_ADDR			0x5F
#define	KEY08_INT_N_SIGNAL			41

#define MINIPAD_I2C_ADDR            0x54
#define MINIPAD_BL_I2C_ADDR         0x25
#define MINIPAD_INT_N_SIGNAL        83

#define HAPTICS_MSM_PWM_LEGACY      18
#define QWERTY_INT_N_SIGNAL_LEGACY  27
#define LIS331DLH_INT1_LEGACY		28
#define WLAN_REG_ON_SIGNAL_LEGACY 	87

#define LIS331DLH_INT1				18
#define QWERTY_INT_N_SIGNAL   		21
#define HAPTICS_MSM_PWM        		27
#define HAPTICS_AMP_EN        		34

#define QWERTY_RST_N_SIGNAL 		32
#define COMPASS_RST_N_SIGNAL 		57
#define TOUCH_RST_N_SIGNAL 			84

#define WLAN_HOST_IRQ				19
#define WLAN_REG_ON_SIGNAL 			35
#define WLAN_RESET_N_SIGNAL 		89
#define WLAN_RST_N					WLAN_RESET_N_SIGNAL

#define BT_HOST_WAKE_SIGNAL			90
#define BT_EXT_WAKE_SIGNAL			91

#define BT_REG_ON_SIGNAL			92
#define BT_RESET_N_SIGNAL			93

#define CAM_PWD_SIGNAL				98
#define CAM_RESET_N_SIGNAL			99

#define LCD_RST_N_SIGNAL 			102

#define BQ27505_I2C_ADDR      		0x55
#define BQ27505_INT_N_SIGNAL  		49

#define	SFH7743_GPIO_INTR			29
#define	SFH7743_GPIO_EN				16


/* Keypad init provided by kpd.c */
extern int  morrison_init_kpd(void);
extern int  motus_init_kpd(void);
extern int  zeppelin_init_kpd(void);

extern unsigned mot_hw_rev;
extern unsigned smi_size;

typedef struct smem_mot_vendor1 {
	char fti[128];
	char fact_byte;
	char mem_info;
	char trusted_boot;	// trusted boot flag (if 1 certificates are verified, otherwise certificates are ignored)
	char bare_board;	// bare board flag (1 indicates being in board level GNPO test bench)
	char security_on;	// fuses modification flag (if 1 all following fuses modification is disabled - final customer SW)
} smem_mot_vendor1_type;


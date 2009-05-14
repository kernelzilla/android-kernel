#ifndef _LINUX_SPI_CPCAP_H
#define _LINUX_SPI_CPCAP_H

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 */

#include <linux/ioctl.h>

#define CPCAP_DEV_NAME "cpcap"
#define CPCAP_NUM_REG_CPCAP (CPCAP_REG_END - CPCAP_REG_START + 1)

enum cpcap_regulators {
        SW5,
        VAUDIO,
        VCAM,
        VCSI,
        VDAC,
        VDIG,
        VFUSE,
        VHVIO,
        VSDIO,
        VPLL,
        VRF1,
        VRF2,
        VRFREF,
        VWLAN1,
        VWLAN2,
        VSIM,
        VSIMCARD,
        VUSB,
        VVIB
};

enum cpcap_regulator_states {
        MODE_OFF,
        MODE_LOW_POWER,
        MODE_AUTO,
        MODE_ON
};

struct cpcap_regulator_data {
        int power_source;    /*The power source of that device. */
        int mode;            /*The default on mode of the device. */
        int standby_mode;    /*The default on standby mode of the device. */
        int standby_bit;     /*The default standby bit state of the device. */
};

struct cpcap_spi_init_data {
        unsigned short reg;
        unsigned short data;
};

struct cpcap_platform_data {
        struct cpcap_spi_init_data *init;
	int init_len;
        struct cpcap_regulator_data *regulator_init;
	int regulator_init_len;
};

/*
 * Enumeration of all registers in the cpcap. Note that the register
 * numbers on the CPCAP IC are not contiguous. The values of the enums below
 * are not the actual register numbers.
 */
enum {
	CPCAP_REG_START,        /* Start of CPCAP registers. */

	CPCAP_REG_INT1          /* Interrupt 1 */
	    = CPCAP_REG_START,
	CPCAP_REG_INT2,         /* Interrupt 2 */
	CPCAP_REG_INT3,         /* Interrupt 3 */
	CPCAP_REG_INT4,         /* Interrupt 4 */
	CPCAP_REG_INTM1,        /* Interrupt Mask 1 */
	CPCAP_REG_INTM2,        /* Interrupt Mask 2 */
	CPCAP_REG_INTM3,        /* Interrupt Mask 3 */
	CPCAP_REG_INTM4,        /* Interrupt Mask 4 */
	CPCAP_REG_INTS1,        /* Interrupt Sense 1 */
	CPCAP_REG_INTS2,        /* Interrupt Sense 2 */
	CPCAP_REG_INTS3,        /* Interrupt Sense 3 */
	CPCAP_REG_INTS4,        /* Interrupt Sense 4 */
	CPCAP_REG_ASSIGN1,      /* Resource Assignment 1 */
	CPCAP_REG_ASSIGN2,	/* Resource Assignment 2 */
	CPCAP_REG_ASSIGN3,	/* Resource Assignment 3 */
	CPCAP_REG_ASSIGN4,	/* Resource Assignment 4 */
	CPCAP_REG_ASSIGN5,	/* Resource Assignment 5 */
	CPCAP_REG_ASSIGN6,	/* Resource Assignment 6 */
	CPCAP_REG_VERSC1,	/* Version Control 1 */
	CPCAP_REG_VERSC2,	/* Version Control 2 */

	CPCAP_REG_MI1,	        /* Macro Interrupt 1 */
	CPCAP_REG_MIM1,         /* Macro Interrupt Mask 1 */
	CPCAP_REG_MI2,	        /* Macro Interrupt 2 */
	CPCAP_REG_MIM2,	        /* Macro Interrupt Mask 2 */
	CPCAP_REG_UCC1,	        /* UC Control 1 */
	CPCAP_REG_UCC2,	        /* UC Control 2 */
	CPCAP_REG_PC1,	        /* Power Cut 1 */
	CPCAP_REG_PC2,	        /* Power Cut 2 */
	CPCAP_REG_BPEOL,	/* BP and EOL */
	CPCAP_REG_PGC,	        /* Power Gate and Control */
	CPCAP_REG_MT1,	        /* Memory Transfer 1 */
	CPCAP_REG_MT2,	        /* Memory Transfer 2 */
	CPCAP_REG_MT3,	        /* Memory Transfer 3 */
	CPCAP_REG_PF,	        /* Print Format */

	CPCAP_REG_SCC,	        /* System Clock Control */
	CPCAP_REG_SW1,	        /* Stop Watch 1 */
	CPCAP_REG_SW2,	        /* Stop Watch 2 */
	CPCAP_REG_UCTM,	        /* UC Turbo Mode */
	CPCAP_REG_TOD1,	        /* Time of Day 1 */
	CPCAP_REG_TOD2,	        /* Time of Day 2 */
	CPCAP_REG_TODA1,	/* Time of Day Alarm 1 */
	CPCAP_REG_TODA2,	/* Time of Day Alarm 2 */
	CPCAP_REG_DAY,	        /* Day */
	CPCAP_REG_DAYA,	        /* Day Alarm */
	CPCAP_REG_VAL1,	        /* Validity 1 */
	CPCAP_REG_VAL2,	        /* Validity 2 */

	CPCAP_REG_SDVSPLL,	/* Switcher DVS and PLL */
	CPCAP_REG_SI2CC1,	/* Switcher I2C Control 1 */
	CPCAP_REG_Si2CC2,	/* Switcher I2C Control 2 */
	CPCAP_REG_S1C1,	        /* Switcher 1 Control 1 */
	CPCAP_REG_S1C2,	        /* Switcher 1 Control 2 */
	CPCAP_REG_S2C1,	        /* Switcher 2 Control 1 */
	CPCAP_REG_S2C2,	        /* Switcher 2 Control 2 */
	CPCAP_REG_S3C,	        /* Switcher 3 Control */
	CPCAP_REG_S4C1,	        /* Switcher 4 Control 1 */
	CPCAP_REG_S4C2,	        /* Switcher 4 Control 2 */
	CPCAP_REG_S5C,	        /* Switcher 5 Control */
	CPCAP_REG_S6C,	        /* Switcher 6 Control */
	CPCAP_REG_VCAMC,	/* VCAM Control */
	CPCAP_REG_VCSIC,	/* VCSI Control */
	CPCAP_REG_VDACC,	/* VDAC Control */
	CPCAP_REG_VDIGC,	/* VDIG Control */
	CPCAP_REG_VFUSEC,	/* VFUSE Control */
	CPCAP_REG_VHVIOC,	/* VHVIO Control */
	CPCAP_REG_VSDIOC,	/* VSDIO Control */
	CPCAP_REG_VPLLC,	/* VPLL Control */
	CPCAP_REG_VRF1C,	/* VRF1 Control */
	CPCAP_REG_VRF2C,	/* VRF2 Control */
	CPCAP_REG_VRFREFC,	/* VRFREF Control */
	CPCAP_REG_VWLAN1C,	/* VWLAN1 Control */
	CPCAP_REG_VWLAN2C,	/* VWLAN2 Control */
	CPCAP_REG_VSIMC,	/* VSIM Control */
	CPCAP_REG_VVIBC,	/* VVIB Control */
	CPCAP_REG_VUSBC,	/* VUSB Control */
	CPCAP_REG_VUSBINT1C,	/* VUSBINT1 Control */
	CPCAP_REG_VUSBINT2C,	/* VUSBINT2 Control */
	CPCAP_REG_URT,	        /* Useroff Regulator Trigger */
	CPCAP_REG_URM1,	        /* Useroff Regulator Mask 1 */
	CPCAP_REG_URM2,	        /* Useroff Regulator Mask 2 */

	CPCAP_REG_VAUDIOC,	/* VAUDIO Control */
	CPCAP_REG_CC,	        /* Codec Control */
	CPCAP_REG_CDI,	        /* Codec Digital Interface */
	CPCAP_REG_SDAC,	        /* Stereo DAC */
	CPCAP_REG_SDACDI,	/* Stereo DAC Digital Interface */
	CPCAP_REG_TXI,		/* TX Inputs */
	CPCAP_REG_TXMP,		/* TX MIC PGA's */
	CPCAP_REG_RXOA,		/* RX Output Amplifiers */
	CPCAP_REG_RXVC,		/* RX Volume Control */
	CPCAP_REG_RXCOA,	/* RX Codec to Output Amps */
	CPCAP_REG_RXSDOA,	/* RX Stereo DAC to Output Amps */
	CPCAP_REG_RXEPOA,	/* RX External PGA to Output Amps */
	CPCAP_REG_RXLL,		/* RX Low Latency */
	CPCAP_REG_A2LA,		/* A2 Loudspeaker Amplifier */
	CPCAP_REG_MIPIS1,	/* MIPI Slimbus 1 */
	CPCAP_REG_MIPIS2,	/* MIPI Slimbus 2 */
	CPCAP_REG_MIPIS3,	/* MIPI Slimbus 3. */
	CPCAP_REG_LVAB,		/* LMR Volume and A4 Balanced. */

	CPCAP_REG_CCC1,		/* Coulomb Counter Control 1 */
	CPCAP_REG_CRM,		/* Charger and Reverse Mode */
	CPCAP_REG_CCCC2,	/* Coincell and Coulomb Ctr Ctrl 2 */
	CPCAP_REG_CCS1,		/* Coulomb Counter Sample 1 */
	CPCAP_REG_CCS2,		/* Coulomb Counter Sample 2 */
	CPCAP_REG_CCA1,		/* Coulomb Counter Accumulator 1 */
	CPCAP_REG_CCA2,		/* Coulomb Counter Accumulator 2 */
	CPCAP_REG_CCM,		/* Coulomb Counter Mode */
	CPCAP_REG_CCO,		/* Coulomb Counter Offset */
	CPCAP_REG_CCI,		/* Coulomb Counter Integrator */

	CPCAP_REG_ADCC1,	/* A/D Converter Configuration 1 */
	CPCAP_REG_ADCC2,	/* A/D Converter Configuration 2 */
	CPCAP_REG_ADCD0,	/* A/D Converter Data 0 */
	CPCAP_REG_ADCD1,	/* A/D Converter Data 1 */
	CPCAP_REG_ADCD2,	/* A/D Converter Data 2 */
	CPCAP_REG_ADCD3,	/* A/D Converter Data 3 */
	CPCAP_REG_ADCD4,	/* A/D Converter Data 4 */
	CPCAP_REG_ADCD5,	/* A/D Converter Data 5 */
	CPCAP_REG_ADCD6,	/* A/D Converter Data 6 */
	CPCAP_REG_ADCD7,	/* A/D Converter Data 7 */
	CPCAP_REG_ADCAL1,	/* A/D Converter Calibration 1 */
	CPCAP_REG_ADCAL2,	/* A/D Converter Calibration 2 */

	CPCAP_REG_USBC1,	/* USB Control 1 */
	CPCAP_REG_USBC2,	/* USB Control 2 */
	CPCAP_REG_USBC3,	/* USB Control 3 */
	CPCAP_REG_UVIDL,	/* ULPI Vendor ID Low */
	CPCAP_REG_UVIDH,	/* ULPI Vendor ID High */
	CPCAP_REG_UPIDL,	/* ULPI Product ID Low */
	CPCAP_REG_UPIDH,	/* ULPI Product ID High */
	CPCAP_REG_UFC1,		/* ULPI Function Control 1 */
	CPCAP_REG_UFC2,		/* ULPI Function Control 2 */
	CPCAP_REG_UFC3,		/* ULPI Function Control 3 */
	CPCAP_REG_UIC1,		/* ULPI Interface Control 1 */
	CPCAP_REG_UIC2,		/* ULPI Interface Control 2 */
	CPCAP_REG_UIC3,		/* ULPI Interface Control 3 */
	CPCAP_REG_USBOTG1,	/* USB OTG Control 1 */
	CPCAP_REG_USBOTG2,	/* USB OTG Control 2 */
	CPCAP_REG_USBOTG3,	/* USB OTG Control 3 */
	CPCAP_REG_UIER1,	/* USB Interrupt Enable Rising 1 */
	CPCAP_REG_UIER2,	/* USB Interrupt Enable Rising 2 */
	CPCAP_REG_UIER3,	/* USB Interrupt Enable Rising 3 */
	CPCAP_REG_UIEF1,	/* USB Interrupt Enable Falling 1 */
	CPCAP_REG_UIEF2,	/* USB Interrupt Enable Falling 1 */
	CPCAP_REG_UIEF3,	/* USB Interrupt Enable Falling 1 */
	CPCAP_REG_UIS,		/* USB Interrupt Status */
	CPCAP_REG_UIL,		/* USB Interrupt Latch */
	CPCAP_REG_USBD,		/* USB Debug */
	CPCAP_REG_SCR1,		/* Scratch 1 */
	CPCAP_REG_SCR2,		/* Scratch 2 */
	CPCAP_REG_SCR3,		/* Scratch 3 */
	CPCAP_REG_VMC,		/* Video Mux Control */
	CPCAP_REG_OWDC,		/* One Wire Device Control */
	CPCAP_REG_GPIO0,	/* GPIO 0 Control */
	CPCAP_REG_GPIO1,	/* GPIO 1 Control */
	CPCAP_REG_GPIO2,	/* GPIO 2 Control */
	CPCAP_REG_GPIO3,	/* GPIO 3 Control */
	CPCAP_REG_GPIO4,	/* GPIO 4 Control */
	CPCAP_REG_GPIO5,	/* GPIO 5 Control */
	CPCAP_REG_GPIO6,	/* GPIO 6 Control */

	CPCAP_REG_MDLC,		/* Main Display Lighting Control */
	CPCAP_REG_KLC,		/* Keypad Lighting Control */
	CPCAP_REG_ADLC,		/* Aux Display Lighting Control */
	CPCAP_REG_REDC,		/* Red Triode Control */
	CPCAP_REG_GREENC,	/* Green Triode Control */
	CPCAP_REG_BLUEC,	/* Blue Triode Control */
	CPCAP_REG_CFC,		/* Camera Flash Control */
	CPCAP_REG_ABC,		/* Adaptive Boost Control */
	CPCAP_REG_BLEDC,	/* Bluetooth LED Control */
	CPCAP_REG_CLEDC,	/* Camera Privacy LED Control */

	CPCAP_REG_OW1C,		/* One Wire 1 Command */
	CPCAP_REG_OW1D,		/* One Wire 1 Data */
	CPCAP_REG_OW1I,		/* One Wire 1 Interrupt */
	CPCAP_REG_OW1IE,	/* One Wire 1 Interrupt Enable */
	CPCAP_REG_OW1,		/* One Wire 1 Control */
	CPCAP_REG_OW2C,		/* One Wire 2 Command */
	CPCAP_REG_OW2D,		/* One Wire 2 Data */
	CPCAP_REG_OW2I,		/* One Wire 2 Interrupt */
	CPCAP_REG_OW2IE,	/* One Wire 2 Interrupt Enable */
	CPCAP_REG_OW2,		/* One Wire 2 Control */
	CPCAP_REG_OW3C,		/* One Wire 3 Command */
	CPCAP_REG_OW3D,		/* One Wire 3 Data */
	CPCAP_REG_OW3I,		/* One Wire 3 Interrupt */
	CPCAP_REG_OW3IE,	/* One Wire 3 Interrupt Enable */
	CPCAP_REG_OW3,		/* One Wire 3 Control */
	CPCAP_REG_GCAIC,	/* GCAI Clock Control */
	CPCAP_REG_GCAIM,	/* GCAI GPIO Mode */
	CPCAP_REG_LGDIR,	/* LMR GCAI GPIO Direction */
	CPCAP_REG_LGPU,		/* LMR GCAI GPIO Pull-up */
	CPCAP_REG_LGPIN,	/* LMR GCAI GPIO Pin */
	CPCAP_REG_LGMASK,	/* LMR GCAI GPIO Mask */
	CPCAP_REG_LDEB,		/* LMR Debounce Settings */
	CPCAP_REG_LGDET,	/* LMR GCAI Detach Detect */
	CPCAP_REG_LMISC,	/* LMR Misc Bits */
	CPCAP_REG_LMACE,	/* LMR Mace IC Support */

	CPCAP_REG_END		/* End of CPCAP registers. */
	    = CPCAP_REG_LMACE,

	CPCAP_REG_MAX		/* The largest valid register value. */
	= CPCAP_REG_END
};



enum {
	CPCAP_IOCTL_NUM_TEST__START,
	CPCAP_IOCTL_NUM_TEST_READ_REG,
	CPCAP_IOCTL_NUM_TEST_WRITE_REG,
	CPCAP_IOCTL_NUM_TEST__END
};

struct cpcap_regacc {
	unsigned short reg;
	unsigned short value;
	unsigned short mask;
};

/*
 * Gets the contents of the specified cpcap register.
 *
 * INPUTS: The register number in the cpcap driver's format.
 *
 * OUTPUTS: The command writes the register data back to user space at the
 * location specified, or it may return an error code.
 */
#define CPCAP_IOCTL_TEST_READ_REG \
	_IOWR(0, CPCAP_IOCTL_NUM_TEST_READ_REG, struct cpcap_regacc*)

/*
 * Writes the specifed cpcap register.
 *
 * This function writes the specified cpcap register with the specified
 * data.
 *
 * INPUTS: The register number in the cpcap driver's format and the data to
 * write to that register.
 *
 * OUTPUTS: The command has no output other than the returned error code for
 * the ioctl() call.
 */
#define CPCAP_IOCTL_TEST_WRITE_REG \
	_IOWR(0, CPCAP_IOCTL_NUM_TEST_WRITE_REG, struct cpcap_regacc*)

struct cpcap_device {
	struct spi_device	*spi;

	void			*keydata;
};


static inline void cpcap_set_keydata(struct cpcap_device *cpcap, void *data)
{
	cpcap->keydata = data;
}

static inline void *cpcap_get_keydata(struct cpcap_device *cpcap)
{
	return cpcap->keydata;
}

int cpcap_regacc_write(struct cpcap_device *cpcap, unsigned short reg,
		       unsigned short value, unsigned short mask);

int cpcap_regacc_read(struct cpcap_device *cpcap, unsigned short reg,
		      unsigned short *vaue_ptr);

int cpcap_regacc_init(struct cpcap_device *cpcap);

void cpcap_broadcast_key_event(struct cpcap_device *cpcap,
			       unsigned int code, int value);

#endif /* CPCAP_H */

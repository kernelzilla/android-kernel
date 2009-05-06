/*
 * Copyright (C) 2007-2009 Motorola, Inc.
 *
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
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include "cpcap-main.h"
#include "cpcap-regbits.h"

#define IS_CPCAP(reg) ((reg) >= CPCAP_REG_START && (reg) <= CPCAP_REG_END)
#define AREG_INDEX(reg) ((reg) - CPCAP_MIN_AREG)
#define LREG_INDEX(reg) ((reg) - CPCAP_MIN_LREG)

static DEFINE_MUTEX(reg_access);

/*
 * This table contains information about a single register in the power IC.
 * It is used during register access to information such as the register address
 * and the modifiability of each bit in the register.  Special notes for
 * particular elements of this structure follows:
 *
 * constant_mask: A '1' in this mask indicates that the corresponding bit has a
 * 'constant' modifiability, and therefore must never be changed by any register
 * access.
 *
 * It is important to note that any bits that are 'constant' must have
 * synchronized read/write values.  That is to say, when a 'constant' bit is
 * read the value read must be identical to the value that must be written to
 * that bit in order for that bit to be read with the same value.
 *
 * rbw_mask: A '1' in this mask indicates that the corresponding bit (when not
 * being changed) should be written with the current value of that bit.  A '0'
 * in this mask indicates that the corresponding bit (when not being changed)
 * should be written with a value of '0'.
 */
static const struct {
	unsigned short address;         /* Address of the register */
	unsigned short constant_mask;	/* Constant modifiability mask */
	unsigned short rbw_mask;	/* Read-before-write mask */
} register_info_tbl[CPCAP_NUM_REG_CPCAP] = {
	{0, 0x0004, 0x0000},	/* Interrupt 1 */
	{1, 0x0000, 0x0000},	/* Interrupt 2 */
	{2, 0x0000, 0x0000},	/* Interrupt 3 */
	{3, 0xFC00, 0x0000},	/* Interrupt 4 */
	{4, 0x0004, 0xFFFF},	/* Interrupt Mask 1 */
	{5, 0x0000, 0xFFFF},	/* Interrupt Mask 2 */
	{6, 0x0000, 0xFFFF},	/* Interrupt Mask 3 */
	{7, 0xFC00, 0xFFFF},	/* Interrupt Mask 4 */
	{8, 0xFFFF, 0xFFFF},	/* Interrupt Sense 1 */
	{9, 0xFFFF, 0xFFFF},	/* Interrupt Sense 2 */
	{10, 0xFFFF, 0xFFFF},	/* Interrupt Sense 3 */
	{11, 0xFFFF, 0xFFFF},	/* Interrupt Sense 4 */
	{12, 0x80F8, 0xFFFF},	/* Resource Assignment 1 */
	{13, 0x0000, 0xFFFF},	/* Resource Assignment 2 */
	{14, 0x0004, 0xFFFF},	/* Resource Assignment 3 */
	{15, 0x0068, 0xFFFF},	/* Resource Assignment 4 */
	{16, 0x0000, 0xFFFF},	/* Resource Assignment 5 */
	{17, 0xFC00, 0xFFFF},	/* Resource Assignment 6 */
	{18, 0xFFFF, 0xFFFF},	/* Version Control 1 */
	{19, 0xFFFF, 0xFFFF},	/* Version Control 2 */
	{128, 0x0000, 0x0000},	/* Macro Interrupt */
	{129, 0x0000, 0xFFFF},	/* Macro Interrupt Mask */
	{130, 0x0000, 0xFFFF},	/* Macro Initiate */
	{131, 0xFFFF, 0xFFFF},	/* Macro Interrupt Sense */
	{132, 0xF000, 0xFFFF},	/* UC Control 1 */
	{133, 0xFC00, 0xFFFF},	/* UC Control 2 */
	{135, 0xFC00, 0xFFFF},	/* Power Cut 1 */
	{136, 0xFC00, 0xFFFF},	/* Power Cut 2 */
	{137, 0xFE00, 0xFFFF},	/* BP and EOL */
	{138, 0xFE00, 0xFFFF},	/* Power Gate and Control */
	{139, 0x0000, 0x0000},	/* Memory Transfer 1 */
	{140, 0x0000, 0x0000},	/* Memory Transfer 2 */
	{141, 0x0000, 0x0000},	/* Memory Transfer 3 */
	{142, 0x0000, 0xFFFF},	/* Print Format */
	{256, 0xFF00, 0xFFFF},	/* System Clock Control */
	{257, 0xFFFF, 0xFFFF},	/* Stop Watch 1 */
	{258, 0xFC7F, 0xFFFF},	/* Stop Watch 2 */
	{259, 0xFFFE, 0xFFFF},	/* UC Turbo Mode */
	{260, 0xFF00, 0xFFFF},	/* Time of Day 1 */
	{261, 0xFE00, 0xFFFF},	/* Time of Day 2 */
	{262, 0xFF00, 0xFFFF},	/* Time of Day Alarm 1 */
	{263, 0xFE00, 0xFFFF},	/* Time of Day Alarm 2 */
	{264, 0x8000, 0xFFFF},	/* Day */
	{265, 0x8000, 0xFFFF},	/* Day Alarm */
	{266, 0x0000, 0xFFFF},	/* Validity 1 */
	{267, 0x0000, 0xFFFF},	/* Validity 2 */
	{384, 0x2488, 0xFFFF},	/* Switcher DVS and PLL */
	{385, 0x8080, 0xFFFF},	/* Switcher I2C Control 1 */
	{386, 0xFF00, 0xFFFF},	/* Switcher I2C Control 2 */
	{387, 0x9080, 0xFFFF},	/* Switcher 1 Control 1 */
	{388, 0x8080, 0xFFFF},	/* Switcher 1 Control 2 */
	{389, 0x9080, 0xFFFF},	/* Switcher 2 Control 1 */
	{390, 0x8080, 0xFFFF},	/* Switcher 2 Control 2 */
	{391, 0xFA84, 0xFFFF},	/* Switcher 3 Control */
	{392, 0x9080, 0xFFFF},	/* Switcher 4 Control 1 */
	{393, 0x8080, 0xFFFF},	/* Switcher 4 Control 2 */
	{394, 0xFFD5, 0xFFFF},	/* Switcher 5 Control */
	{395, 0xFFF4, 0xFFFF},	/* Switcher 6 Control */
	{396, 0xFF48, 0xFFFF},	/* VCAM Control */
	{397, 0xFFA8, 0xFFFF},	/* VCSI Control */
	{398, 0xFF48, 0xFFFF},	/* VDAC Control */
	{399, 0xFF48, 0xFFFF},	/* VDIG Control */
	{400, 0xFF50, 0xFFFF},	/* VFUSE Control */
	{401, 0xFFE8, 0xFFFF},	/* VHVIO Control */
	{402, 0xFF40, 0xFFFF},	/* VSDIO Control */
	{403, 0xFFA4, 0xFFFF},	/* VPLL Control */
	{404, 0xFF52, 0xFFFF},	/* VRF1 Control */
	{405, 0xFFD4, 0xFFFF},	/* VRF2 Control */
	{406, 0xFFD4, 0xFFFF},	/* VRFREF Control */
	{407, 0xFFA8, 0xFFFF},	/* VWLAN1 Control */
	{408, 0xFD32, 0xFFFF},	/* VWLAN2 Control */
	{409, 0xE154, 0xFFFF},	/* VSIM Control */
	{410, 0xFFF2, 0xFFFF},	/* VVIB Control */
	{411, 0xFEA2, 0xFFFF},	/* VUSB Control */
	{412, 0xFFD4, 0xFFFF},	/* VUSBINT1 Control */
	{413, 0xFFD4, 0xFFFF},	/* VUSBINT2 Control */
	{414, 0xFFFE, 0xFFFF},	/* Useroff Regulator Trigger */
	{415, 0x0000, 0xFFFF},	/* Useroff Regulator Mask 1 */
	{416, 0xFC00, 0xFFFF},	/* Useroff Regulator Mask 2 */
	{512, 0xFF88, 0xFFFF},	/* VAUDIO Control */
	{513, 0x0000, 0xFEDF},	/* Codec Control */
	{514, 0x4000, 0xFFFF},	/* Codec Digital Interface */
	{515, 0xF000, 0xFCFF},	/* Stereo DAC */
	{516, 0xC000, 0xFFFF},	/* Stereo DAC Digital Interface */
	{517, 0x0000, 0xFFFF},	/* TX Inputs */
	{518, 0xF000, 0xFFFF},	/* TX MIC PGA's */
	{519, 0xF800, 0xFFFF},	/* RX Output Amplifiers */
	{520, 0x00C3, 0xFFFF},	/* RX Volume Control */
	{521, 0xF800, 0xFFFF},	/* RX Codec to Output Amps */
	{522, 0xE000, 0xFFFF},	/* RX Stereo DAC to Output Amps */
	{523, 0x8000, 0xFFFF},	/* RX Ext PGA to Output Amps */
	{524, 0x0000, 0xFFFF},	/* RX Low Latency */
	{525, 0xFF00, 0xFFFF},	/* A2 Loudspeaker Amplifier */
	{526, 0x0000, 0xFFFF},	/* MIPI Slimbus 1 */
	{527, 0xFF00, 0xFFFF},	/* MIPI Slimbus 2 */
	{528, 0xFFFC, 0xFFFF},	/* MIPI Slimbus 3 */
	{529, 0xFFFC, 0xFFFF},	/* LMR Volume and A4 Balanced */
	{640, 0xFFF0, 0xFFFF},	/* Coulomb Counter Control 1 */
	{641, 0xC000, 0xFFFF},	/* Charger and Reverse Mode */
	{642, 0xFFC0, 0xFFFF},	/* Coincell & Coulomb Ctr Ctrl 2 */
	{643, 0x0000, 0xFFFF},	/* Coulomb Counter Sample 1 */
	{644, 0xFF00, 0xFFFF},	/* Coulomb Counter Sample 2 */
	{645, 0x0000, 0xFFFF},	/* Coulomb Counter Accumulator 1 */
	{646, 0x0000, 0xFFFF},	/* Coulomb Counter Accumulator 2 */
	{647, 0xFC00, 0xFFFF},	/* Coulomb Counter Mode */
	{648, 0xFC00, 0xFFFF},	/* Coulomb Counter Offset */
	{649, 0xC000, 0xFFFF},	/* Coulomb Counter Integrator */
	{768, 0x0000, 0xFFFF},	/* A/D Converter Configuration 1 */
	{769, 0x0080, 0xFFFF},	/* A/D Converter Configuration 2 */
	{770, 0xFFFF, 0xFFFF},	/* A/D Converter Data 0 */
	{771, 0xFFFF, 0xFFFF},	/* A/D Converter Data 1 */
	{772, 0xFFFF, 0xFFFF},	/* A/D Converter Data 2 */
	{773, 0xFFFF, 0xFFFF},	/* A/D Converter Data 3 */
	{774, 0xFFFF, 0xFFFF},	/* A/D Converter Data 4 */
	{775, 0xFFFF, 0xFFFF},	/* A/D Converter Data 5 */
	{776, 0xFFFF, 0xFFFF},	/* A/D Converter Data 6 */
	{777, 0xFFFF, 0xFFFF},	/* A/D Converter Data 7 */
	{778, 0xFFFF, 0xFFFF},	/* A/D Converter Calibration 1 */
	{779, 0xFFFF, 0xFFFF},	/* A/D Converter Calibration 2 */
	{896, 0x0000, 0xFFFF},	/* USB Control 1 */
	{897, 0x0000, 0xFFFF},	/* USB Control 2 */
	{898, 0x8200, 0xFFFF},	/* USB Control 3 */
	{899, 0xFFFF, 0xFFFF},	/* ULPI Vendor ID Low */
	{900, 0xFFFF, 0xFFFF},	/* ULPI Vendor ID High */
	{901, 0xFFFF, 0xFFFF},	/* ULPI Product ID Low */
	{902, 0xFFFF, 0xFFFF},	/* ULPI Product ID High */
	{903, 0xFF80, 0xFFFF},	/* ULPI Function Control 1 */
	{904, 0xFF80, 0xFFFF},	/* ULPI Function Control 2 */
	{905, 0xFF80, 0xFFFF},	/* ULPI Function Control 3 */
	{906, 0xFF64, 0xFFFF},	/* ULPI Interface Control 1 */
	{907, 0xFF64, 0xFFFF},	/* ULPI Interface Control 2 */
	{908, 0xFF64, 0xFFFF},	/* ULPI Interface Control 3 */
	{909, 0xFFC0, 0xFFFF},	/* USB OTG Control 1 */
	{910, 0xFFC0, 0xFFFF},	/* USB OTG Control 2 */
	{911, 0xFFC0, 0xFFFF},	/* USB OTG Control 3 */
	{912, 0xFFE0, 0xFFFF},	/* USB Interrupt Enable Rise 1 */
	{913, 0xFFE0, 0xFFFF},	/* USB Interrupt Enable Rise 2 */
	{914, 0xFFE0, 0xFFFF},	/* USB Interrupt Enable Rise 3 */
	{915, 0xFFE0, 0xFFFF},	/* USB Interrupt Enable Fall 1 */
	{916, 0xFFE0, 0xFFFF},	/* USB Interrupt Enable Fall 2 */
	{917, 0xFFE0, 0xFFFF},	/* USB Interrupt Enable Fall 3 */
	{918, 0xFFFF, 0xFFFF},	/* USB Interrupt Status */
	{919, 0xFFFF, 0xFFFF},	/* USB Interrupt Latch */
	{920, 0xFFFF, 0xFFFF},	/* USB Debug */
	{921, 0xFF00, 0xFFFF},	/* Scratch 1 */
	{922, 0xFF00, 0xFFFF},	/* Scratch 2 */
	{923, 0xFF00, 0xFFFF},	/* Scratch 3 */
	{939, 0xFFFE, 0xFFFF},	/* Video Mux Control */
	{940, 0xFFFC, 0xFFFF},	/* One Wire Device Control */
	{941, 0x0D11, 0x3FFF},	/* GPIO 0 Control */
	{943, 0x0D11, 0x3FFF},	/* GPIO 1 Control */
	{945, 0x0D11, 0x3FFF},	/* GPIO 2 Control */
	{947, 0x0D11, 0x3FFF},	/* GPIO 3 Control */
	{949, 0x0D11, 0x3FFF},	/* GPIO 4 Control */
	{951, 0x0C11, 0x3FFF},	/* GPIO 5 Control */
	{953, 0x0C11, 0x3FFF},	/* GPIO 6 Control */
	{1024, 0x0000, 0xFFFF},	/* Main Display Lighting Control */
	{1025, 0x8000, 0xFFFF},	/* Keypad Lighting Control */
	{1026, 0x8000, 0xFFFF},	/* Aux Display Lighting Control */
	{1027, 0xFC00, 0xFFFF},	/* Red Triode Control */
	{1028, 0xFC00, 0xFFFF},	/* Green Triode Control */
	{1029, 0xFC00, 0xFFFF},	/* Blue Triode Control */
	{1030, 0xF000, 0xFFFF},	/* Camera Flash Control */
	{1031, 0xFFC3, 0xFFFF},	/* Adaptive Boost Control */
	{1032, 0xFC00, 0xFFFF},	/* Bluetooth LED Control */
	{1033, 0xFC00, 0xFFFF},	/* Camera Privacy LED Control */
	{1152, 0xFF00, 0xFFFF},	/* One Wire 1 Command */
	{1153, 0xFF00, 0xFFFF},	/* One Wire 1 Data */
	{1154, 0xFFFF, 0xFFFF},	/* One Wire 1 Interrupt */
	{1155, 0xFF00, 0xFFFF},	/* One Wire 1 Interrupt Enable */
	{1157, 0xFF00, 0xFFFF},	/* One Wire 1 Control */
	{1160, 0xFF00, 0xFFFF},	/* One Wire 2 Command */
	{1161, 0xFF00, 0xFFFF},	/* One Wire 2 Data */
	{1162, 0xFFFF, 0xFFFF},	/* One Wire 2 Interrupt */
	{1163, 0xFF00, 0xFFFF},	/* One Wire 2 Interrupt Enable */
	{1165, 0xFF00, 0xFFFF},	/* One Wire 2 Control */
	{1168, 0xFF00, 0xFFFF},	/* One Wire 3 Command */
	{1169, 0xFF00, 0xFFFF},	/* One Wire 3 Data */
	{1170, 0xFF00, 0xFFFF},	/* One Wire 3 Interrupt */
	{1171, 0xFF00, 0xFFFF},	/* One Wire 3 Interrupt Enable */
	{1173, 0xFF00, 0xFFFF},	/* One Wire 3 Control */
	{1174, 0xFF00, 0xFFFF},	/* GCAI Clock Control */
	{1175, 0xFF00, 0xFFFF},	/* GCAI GPIO Mode */
	{1176, 0xFFE0, 0xFFFF},	/* LMR GCAI GPIO Direction */
	{1177, 0xFFE0, 0xFFFF},	/* LMR GCAI GPIO Pull-up */
	{1178, 0xFF00, 0xFFFF},	/* LMR GCAI GPIO Pin */
	{1179, 0xFFE0, 0xFFFF},	/* LMR GCAI GPIO Mask */
	{1180, 0xFF00, 0xFFFF},	/* LMR Debounce Settings */
	{1181, 0xFF00, 0xFFFF},	/* LMR GCAI Detach Detect */
	{1182, 0xFF07, 0xFFFF},	/* LMR Misc Bits */
	{1183, 0xFFF8, 0xFFFF}	/* LMR Mace IC Support */
};

static const unsigned short
    audio_bits[CPCAP_MAX_AREG - CPCAP_MIN_AREG + 1] = {
	0x0060, /* CPCAP_REG_CPCAP_VAUDIOC */
	0xFFFF, /* CPCAP_REG_CPCAP_CC */
	0xBFFF, /* CPCAP_REG_CPCAP_CDI */
	0x0FFF, /* CPCAP_REG_CPCAP_SDAC */
	0x3FFF, /* CPCAP_REG_CPCAP_SDACDI */
	0x0FDF, /* CPCAP_REG_CPCAP_TXI */
	0x0FFF, /* CPCAP_REG_CPCAP_TXMP */
	0x03FF, /* CPCAP_REG_CPCAP_RXOA */
	0xFF3C, /* CPCAP_REG_CPCAP_RXVC */
	0x07FF, /* CPCAP_REG_CPCAP_RXCOA */
	0x1FFF, /* CPCAP_REG_CPCAP_RXSDOA */
	0x7FFF, /* CPCAP_REG_CPCAP_RXEPOA */
	0x7FFF, /* CPCAP_REG_CPCAP_RXLL */
	0x00FF, /* CPCAP_REG_CPCAP_A2LA */
	0xFFFF, /* CPCAP_REG_CPCAP_MIPIS1 */
	0x00FF, /* CPCAP_REG_CPCAP_MIPIS2 */
	0x0003, /* CPCAP_REG_CPCAP_MIPIS3 */
	0x0003  /* CPCAP_REG_CPCAP_LVAB */
};

static const unsigned short lighting_bits[CPCAP_MAX_LREG -
					  CPCAP_MIN_LREG + 1] = {
	0xFFFF, /* CPCAP_REG_CPCAP_MDLC */
	0x7FFF, /* CPCAP_REG_CPCAP_KLC */
	0x7FFF, /* CPCAP_REG_CPCAP_ADLC */
	0x03FF, /* CPCAP_REG_CPCAP_REDC */
	0x03FF, /* CPCAP_REG_CPCAP_GREENC */
	0x03FF, /* CPCAP_REG_CPCAP_BLUEC */
	0x0FFF, /* CPCAP_REG_CPCAP_CFC */
	0x003C, /* CPCAP_REG_CPCAP_ABC */
	0x03FF, /* CPCAP_REG_CPCAP_BLEDC */
	0x03FF  /* CPCAP_REG_CPCAP_CLEDC */
};

static int cpcap_spi_access(struct spi_device *spi, u8 *buf,
			    size_t len)
{
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf = buf,
		.len = len,
		.rx_buf = buf,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

static int cpcap_config_for_read(struct spi_device *spi, unsigned short reg,
				 unsigned short *data)
{
	int status = -ENOTTY;
	char buf[4];

	if (spi != NULL) {
		buf[3] = (reg >> 6) & 0x000000FF;
		buf[2] = (reg << 2) & 0x000000FF;
		buf[1] = 0;
		buf[0] = 0;

		status = cpcap_spi_access(spi, buf, 4);

		if (status == 0)
			*data = buf[0] | (buf[1] << 8);
	}

	return status;
}

static int cpcap_config_for_write(struct spi_device *spi, unsigned short reg,
				  unsigned short data)
{
	int status = -ENOTTY;
	char buf[4];

	if (spi != NULL) {
		buf[3] = ((reg >> 6) & 0x000000FF) | 0x80;
		buf[2] = (reg << 2) & 0x000000FF;
		buf[1] = (data >> 8) & 0x000000FF;
		buf[0] = data & 0x000000FF;

		status = cpcap_spi_access(spi, buf, 4);
	}

	return status;
}

int cpcap_regacc_read(struct spi_device *spi, unsigned short reg,
		      unsigned short *value_ptr)
{
	int retval = -EINVAL;

	if (IS_CPCAP(reg) && (value_ptr != 0)) {
		mutex_lock(&reg_access);

		cpcap_config_for_read(spi, register_info_tbl
				      [reg].address, value_ptr);

		mutex_unlock(&reg_access);
	}

	return retval;
}

int cpcap_regacc_write(struct spi_device *spi,
		       unsigned short reg,
		       unsigned short value,
		       unsigned short mask)
{
	int retval = -EINVAL;
	unsigned short old_value = 0;
	struct cpcap_platform_data *data;

	data = (struct cpcap_platform_data *)spi->controller_data;

	if (IS_CPCAP(reg) && (mask & register_info_tbl[reg].constant_mask) == 0) {
		mutex_lock(&reg_access);

		value &= mask;

		if ((register_info_tbl[reg].rbw_mask) != 0) {
			retval = cpcap_config_for_read(spi, register_info_tbl
						       [reg].address, &old_value);
			if (retval != 0)
				goto error;
		}

		old_value &= register_info_tbl[reg].rbw_mask;
		old_value &= ~mask;
		value |= old_value;
		cpcap_config_for_write(spi, register_info_tbl [reg].address,
				       value);

error:
		mutex_unlock(&reg_access);
	}

	return retval;
}

int cpcap_regacc_init(struct spi_device *spi)
{
	unsigned short i;
	unsigned short mask;
	int retval = 0;
	struct cpcap_platform_data *data;

	data = (struct cpcap_platform_data *)
		spi->controller_data;

	for (i = 0; i < (sizeof(data->init) / sizeof(data->init[0])); i++) {
		mask = 0xFFFF;
		mask &= ~(register_info_tbl[data->init[i].reg].constant_mask);

		retval = cpcap_regacc_write(spi, data->init[i].reg,
					    data->init[i].data,
					    mask);
	}

	return retval;
}

int cpcap_audio_read_reg(struct spi_device *spi, unsigned short reg,
			 unsigned short *value)
{
	int retval = -EINVAL;

	if ((reg >= CPCAP_MIN_AREG)
	    && (reg <= CPCAP_MAX_AREG)) {
		retval = cpcap_regacc_read(spi, reg, value);
	}

	return retval;
}
EXPORT_SYMBOL(cpcap_audio_read_reg);

int cpcap_audio_write_reg(struct spi_device *spi, unsigned short reg,
			  unsigned short value, unsigned short mask)
{
	int retval = -EINVAL;

	if ((reg >= CPCAP_MIN_AREG) &&
	    (reg <= CPCAP_MAX_AREG) &&
	    ((mask & ~(audio_bits[AREG_INDEX(reg)])) == 0)) {
		retval = cpcap_regacc_write(spi, reg, value, mask);
	}

	return retval;
}
EXPORT_SYMBOL(cpcap_audio_write_reg);

int cpcap_lighting_read_reg(struct spi_device *spi, unsigned short reg,
			    unsigned short *value)
{
	int retval = -EINVAL;

	if ((reg >= CPCAP_MIN_LREG)
	    && (reg <= CPCAP_MAX_LREG)) {
		retval = cpcap_regacc_read(spi, reg, value);
	} else if (reg == CPCAP_REG_CRM) {
		retval = cpcap_regacc_read(spi, reg, value);
		*value &= CPCAP_BIT_CHRG_LED_EN;
	}

	return retval;
}
EXPORT_SYMBOL(cpcap_lighting_read_reg);

int cpcap_lighting_write_reg(struct spi_device *spi, unsigned short reg,
			     unsigned short value, unsigned short mask)
{
	int retval = -EINVAL;

	if (((reg >= CPCAP_MIN_LREG) &&
	     (reg <= CPCAP_MAX_LREG) &&
	     ((mask & ~(lighting_bits[LREG_INDEX(reg)])) == 0)) ||
	    ((reg == CPCAP_REG_CRM) &&
	     (mask == CPCAP_BIT_CHRG_LED_EN))) {
		retval = cpcap_regacc_write(spi, reg, value, mask);
	}

	return retval;
}
EXPORT_SYMBOL(cpcap_lighting_write_reg);

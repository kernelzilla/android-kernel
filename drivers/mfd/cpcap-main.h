#ifndef CPCAP_MAIN_H
#define CPCAP_MAIN_H

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
 */

#define CPCAP_MIN_AREG CPCAP_REG_VAUDIOC
#define CPCAP_MAX_AREG CPCAP_REG_LVAB
#define CPCAP_MIN_LREG CPCAP_REG_MDLC
#define CPCAP_MAX_LREG CPCAP_REG_CLEDC

int cpcap_regacc_write(struct spi_device *spi, unsigned short reg,
		       unsigned short value, unsigned short mask);

int cpcap_regacc_read(struct spi_device *spi, unsigned short reg,
		      unsigned short *value_ptr);

int cpcap_regacc_init(struct spi_device *spi);

void cpcap_broadcast_key_event(struct spi_device *spi, unsigned int code,
			       int value);

#endif /* CPCAP_MAIN_H */

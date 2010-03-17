/*
 * Driver for the Semtech SX150x I2C GPIO Expanders
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __LINUX_I2C_SX150X_H
#define __LINUX_I2C_SX150X_H

/**
 * struct sx150x_platform_data - config data for SX150x driver
 * @gpio_base: The index number of the first GPIO assigned to this
 *             GPIO expander.  The expander will create a block of
 *             consecutively numbered gpios beginning at the given base,
 *             with the size of the block depending on the model of the
 *             expander chip.
 * @oscio_is_gpo: If set to true, the driver will configure OSCIO as a GPO
 *                instead of as an oscillator, increasing the size of the
 *                GP(I)O pool created by this expander by one.  The
 *                output-only GPO pin will be added at the end of the block.
 * @io_pullup_ena: A bit-mask which enables or disables the pull-up resistor
 *                 for each IO line in the expander.  Setting the bit at
 *                 position n will enable the pull-up for the IO at
 *                 the corresponding offset.  For chips with fewer than
 *                 16 IO pins, high-end bits are ignored.
 * @io_pulldn_ena: A bit-mask which enables-or disables the pull-down
 *                 resistor for each IO line in the expander. Setting the
 *                 bit at position n will enable the pull-down for the IO at
 *                 the corresponding offset.  For chips with fewer than
 *                 16 IO pins, high-end bits are ignored.
 * @io_open_drain_ena: A bit-mask which enables-or disables open-drain
 *                     operation for each IO line in the expander. Setting the
 *                     bit at position n enables open-drain operation for
 *                     the IO at the corresponding offset.  Clearing the bit
 *                     enables regular push-pull operation for that IO.
 *                     For chips with fewer than 16 IO pins, high-end bits
 *                     are ignored.
 * @io_polarity: A bit-mask which enables polarity inversion for each IO line
 *               in the expander.  Setting the bit at position n inverts
 *               the polarity of that IO line, while clearing it results
 *               in normal polarity. For chips with fewer than 16 IO pins,
 *               high-end bits are ignored.
 * @irq_summary: The 'summary IRQ' line to which the GPIO expander's INT line
 *               is connected, via which it reports interrupt events
 *               across all GPIO lines.  This must be a real,
 *               pre-existing IRQ line.
 *               Setting this value < 0 disables the irq_chip functionality
 *               of the driver.
 * @irq_base: The first 'virtual IRQ' line at which our block of GPIO-based
 *            IRQ lines will appear.  Similarly to gpio_base, the expander
 *            will create a block of irqs beginning at this number.
 *            This value is ignored if irq_summary is < 0.
 * @irq_sense: Default edge-detection settings for IO lines to be used when
 *             they are enabled as IRQs.  Two bits are used to provide the
 *             default for each line, beginning with the LSB.  In each pair,
 *             the first bit enables rising-edge sensitivity, and the second,
 *             falling. For chips with less than 16 IO lines, not all 32 bits
 *             are used.
 *
 *             33222222222211111111110000000000 BIT
 *             10987654321098765432109876543210
 *             frfrfrfrfrfrfrfrfrfrfrfrfrfrfrfr
 *              1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0
 *              5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 IO LINE
 *
 *             Example: to set all IO lines to rising-edge detection,
 *             set irq_sense to 0x55555555.
 */
struct sx150x_platform_data {
	unsigned gpio_base;
	bool     oscio_is_gpo;
	u16      io_pullup_ena;
	u16      io_pulldn_ena;
	u16      io_open_drain_ena;
	u16      io_polarity;
	int      irq_summary;
	unsigned irq_base;
	u32      irq_sense;
};

#endif /* __LINUX_I2C_SX150X_H */

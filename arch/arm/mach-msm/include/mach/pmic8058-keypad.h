/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __PMIC8058_KEYPAD_H__
#define __PMIC8058_KEYPAD_H__

#define MATRIX_MAX_ROWS		18
#define MATRIX_MAX_COLS		8

#define MATRIX_MAX_SIZE		(MATRIX_MAX_ROWS * MATRIX_MAX_COLS)

#define MAX_SCAN_DELAY		128
#define MIN_SCAN_DELAY		1

#define MAX_DEBOUNCE_B0_TIME	20
#define MIN_DEBOUNCE_B0_TIME	5

#define MAX_DEBOUNCE_A0_TIME	8
#define MIN_DEBOUNCE_A0_TIME	1

#define KEY_COL(val)	(((val) >> 16) & 0xff)
#define KEY_ROW(val)	(((val) >> 24) & 0xff)
#define KEY_VAL(val)	((val) & 0xffff)

#define KEY(row, col, val)	(((row % (MATRIX_MAX_ROWS)) << 24) |\
				 ((col % (MATRIX_MAX_COLS)) << 16)  |\
				 (val & 0xffff))

struct pmic8058_keypad_data {
	const char *input_name;
	const char *input_phys_device;

	unsigned int num_cols;
	unsigned int num_rows;

	unsigned int rows_gpio_start;
	unsigned int cols_gpio_start;

	unsigned int debounce_ms;
	unsigned int scan_delay_ms;
	unsigned int row_hold_us;

	int keymap_size;
	const unsigned int *keymap;

	unsigned int wakeup;
	unsigned int rep;
};

#endif /*__PMIC8058_KEYPAD_H__ */
